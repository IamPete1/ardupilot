#include "Copter.h"

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;
}

void Copter::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
   rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
   rangefinder.init(ROTATION_PITCH_270);

   rangefinder_state.init(g2.rangefinder_filt);

   // upward facing range finder
   rangefinder_up_state.init(g2.rangefinder_filt);
#endif
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

    rangefinder_state.update();
    rangefinder_up_state.update();
#endif
}

void Copter::RangeFinderState::update()
{
    const uint32_t now = AP_HAL::millis();

#if RANGEFINDER_TILT_CORRECTION == ENABLED
    const AP_AHRS &ahrs = AP::ahrs();
    const float tilt_correction = MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
#else
    const float tilt_correction = 1.0f;
#endif

    // update health
    alt_healthy = ((rangefinder.status_orient(rf_orient) == RangeFinder::Status::Good) &&
                   (rangefinder.range_valid_count_orient(rf_orient) >= RANGEFINDER_HEALTH_MAX));

    // tilt corrected but unfiltered, not glitch protected alt
    alt_cm = tilt_correction * rangefinder.distance_cm_orient(rf_orient);

    // remember inertial alt to allow us to interpolate rangefinder
    inertial_alt_cm = inertial_nav.get_position_z_up_cm();

    // glitch handling.  rangefinder readings more than RANGEFINDER_GLITCH_ALT_CM from the last good reading
    // are considered a glitch and glitch_count becomes non-zero
    // glitches clear after RANGEFINDER_GLITCH_NUM_SAMPLES samples in a row.
    // glitch_cleared_ms is set so surface tracking (or other consumers) can trigger a target reset
    const int32_t glitch_cm = alt_cm - alt_cm_glitch_protected;
    bool reset_terrain_offset = false;
    if (glitch_cm >= RANGEFINDER_GLITCH_ALT_CM) {
        glitch_count = MAX(glitch_count+1, 1);
    } else if (glitch_cm <= -RANGEFINDER_GLITCH_ALT_CM) {
        glitch_count = MIN(glitch_count-1, -1);
    } else {
        glitch_count = 0;
        alt_cm_glitch_protected = alt_cm;
    }
    if (abs(glitch_count) >= RANGEFINDER_GLITCH_NUM_SAMPLES) {
        // clear glitch and record time so consumers (i.e. surface tracking) can reset their target altitudes
        glitch_count = 0;
        alt_cm_glitch_protected = alt_cm;
        glitch_cleared_ms = now;
        reset_terrain_offset = true;
    }

    // filter rangefinder altitude
    const bool timed_out = now - last_healthy_ms > RANGEFINDER_TIMEOUT_MS;
    if (alt_healthy) {
        if (timed_out) {
            // reset filter if we haven't used it within the last second
            alt_cm_filt.reset(alt_cm);
            reset_terrain_offset = true;

        } else {
            alt_cm_filt.apply(alt_cm, 0.05f);
        }
        last_healthy_ms = now;
    }

    // handle reset of terrain offset
    if (reset_terrain_offset) {
        if (rf_orient == ROTATION_PITCH_90) {
            // upward facing
            terrain_offset_cm = inertial_alt_cm + alt_cm;
        } else {
            // assume downward facing
            terrain_offset_cm = inertial_alt_cm - alt_cm;
        }
    }

    // send downward facing lidar altitude and health to the libraries that require it
#if HAL_PROXIMITY_ENABLED
    if (rf_orient == ROTATION_PITCH_270) {
        AP_Proximity *proximity = AP::proximity();
        if ((proximity != nullptr) && (alt_healthy || timed_out)) {
            proximity->set_rangefinder_alt(enabled(), alt_healthy, alt_cm_filt.get());
        }
    }
#endif
}

// return true if rangefinder_alt can be used
bool Copter::RangeFinderState::alt_ok() const
{
    return enabled() && get_alt_healthy();
}

void Copter::RangeFinderState::update_terrain_offset(float alpha)
{
    const float new_terrain_offset_cm = inertial_alt_cm - alt_cm_glitch_protected;
    terrain_offset_cm += (new_terrain_offset_cm - terrain_offset_cm) * alpha;
}

// update rangefinder based terrain offset
// terrain offset is the terrain's height above the EKF origin
void Copter::update_rangefinder_terrain_offset()
{
    const float terrain_offset_alpha = copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt);

    rangefinder_state.update_terrain_offset(terrain_offset_alpha);
    rangefinder_up_state.update_terrain_offset(terrain_offset_alpha);

    if (rangefinder_state.get_alt_healthy() || (AP_HAL::millis() - rangefinder_state.get_last_healthy_ms() > RANGEFINDER_TIMEOUT_MS)) {
        wp_nav->set_rangefinder_terrain_offset(rangefinder_state.enabled(), rangefinder_state.get_alt_healthy(), rangefinder_state.get_terrain_offset_cm());
#if MODE_CIRCLE_ENABLED
        circle_nav->set_rangefinder_terrain_offset(rangefinder_state.enabled() && wp_nav->rangefinder_used(), rangefinder_state.get_alt_healthy(), rangefinder_state.get_terrain_offset_cm());
#endif
    }
}

/*
  get inertially interpolated rangefinder height. Inertial height is
  recorded whenever we update the rangefinder height, then we use the
  difference between the inertial height at that time and the current
  inertial height to give us interpolation of height from rangefinder
 */
bool Copter::RangeFinderState::get_height_interpolated_cm(int32_t& ret) const
{
    if (!alt_ok()) {
        return false;
    }
    ret = alt_cm_filt.get();
    float current_inertial_alt_cm = inertial_nav.get_position_z_up_cm();
    ret += current_inertial_alt_cm - inertial_alt_cm;
    return true;
}

Copter::RangeFinderState::RangeFinderState(RangeFinder& _rng, AP_InertialNav _inav, enum Rotation _orient) :
    rangefinder(_rng),
    inertial_nav(_inav),
    rf_orient(_orient)
{};

void Copter::RangeFinderState::init(float cut_off)
{
    alt_cm_filt.set_cutoff_frequency(cut_off);
    set_enable(true);
}

void Copter::RangeFinderState::set_enable(bool b)
{
    if (enable == b) {
        // Already in correct state
        return;
    }
    if (b && !rangefinder.has_orientation(rf_orient)) {
        // Cannot enable without valid rangefinder
        return;
    }
    enable = b;
}

bool Copter::RangeFinderState::enabled() const
{
    return enable;
}

bool Copter::RangeFinderState::get_alt_healthy() const
{
    return alt_healthy;
}

uint32_t Copter::RangeFinderState::get_last_healthy_ms() const
{
    return last_healthy_ms;
}

int16_t Copter::RangeFinderState::get_alt_cm() const
{
    return alt_cm;
}

int16_t Copter::RangeFinderState::get_alt_cm_glitch_protected() const
{
    return alt_cm_glitch_protected;
}

float Copter::RangeFinderState::get_alt_cm_filt() const
{
    return alt_cm_filt.get();
}

float Copter::RangeFinderState::get_terrain_offset_cm() const
{
    return terrain_offset_cm;
}

uint32_t Copter::RangeFinderState::get_glitch_cleared_ms() const
{
    return glitch_cleared_ms;
}

bool Copter::RangeFinderState::is_glitching() const
{
    return glitch_count != 0;
}
