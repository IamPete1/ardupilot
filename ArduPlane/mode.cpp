#include "Plane.h"

Mode::Mode()
{
}

void Mode::exit()
{
    // call sub-classes exit
    _exit();
}

bool Mode::enter()
{
    // cancel inverted flight
    plane.auto_state.inverted_flight = false;

    // don't cross-track when starting a mission
    plane.auto_state.next_wp_crosstrack = false;

    // reset landing check
    plane.auto_state.checked_for_autoland = false;

    // zero locked course
    plane.steer_state.locked_course_err = 0;

    // reset crash detection
    plane.crash_state.is_crashed = false;
    plane.crash_state.impact_detected = false;

    // reset external attitude guidance
    plane.guided_state.last_forced_rpy_ms.zero();
    plane.guided_state.last_forced_throttle_ms = 0;

#if OFFBOARD_GUIDED == ENABLED
    plane.guided_state.target_heading = -4; // radians here are in range -3.14 to 3.14, so a default value needs to be outside that range
    plane.guided_state.target_heading_type = GUIDED_HEADING_NONE;
    plane.guided_state.target_airspeed_cm = -1; // same as above, although an airspeed of -1 is rare on plane.
    plane.guided_state.target_alt = -1; // same as above, although a target alt of -1 is rare on plane.
    plane.guided_state.last_target_alt = 0;
#endif

#if CAMERA == ENABLED
    plane.camera.set_is_auto_mode(this == &plane.mode_auto);
#endif

    // zero initial pitch and highest airspeed on mode change
    plane.auto_state.highest_airspeed = 0;
    plane.auto_state.initial_pitch_cd = plane.ahrs.pitch_sensor;

    // disable taildrag takeoff on mode change
    plane.auto_state.fbwa_tdrag_takeoff_mode = false;

    // start with previous WP at current location
    plane.prev_WP_loc = plane.current_loc;

    // new mode means new loiter
    plane.loiter.start_time_ms = 0;

    // record time of mode change
    plane.last_mode_change_ms = AP_HAL::millis();

    // assume non-VTOL mode
    plane.auto_state.vtol_mode = false;
    plane.auto_state.vtol_loiter = false;

    // initialize speed variable used in AUTO and GUIDED for DO_CHANGE_SPEED commands
    plane.new_airspeed_cm = -1;

    bool enter_result = allow_enter_at_current_alt() && _enter();

    if (enter_result) {
        // -------------------
        // these must be done AFTER _enter() because they use the results to set more flags

        // start with throttle suppressed in auto_throttle modes
        plane.throttle_suppressed = does_auto_throttle();
#if HAL_ADSB_ENABLED
        plane.adsb.set_is_auto_mode(does_auto_navigation());
#endif

        // reset steering integrator on mode change
        plane.steerController.reset_I();

        // update RC failsafe, as mode change may have necessitated changing the failsafe throttle
        plane.control_failsafe();
    }

    return enter_result;
}

bool Mode::is_vtol_man_throttle() const
{
    if (plane.quadplane.is_tailsitter_in_fw_flight() &&
        plane.quadplane.assisted_flight) {
        // We are a tailsitter that has fully transitioned to Q-assisted forward flight.
        // In this case the forward throttle directly drives the vertical throttle so
        // set vertical throttle state to match the forward throttle state. Confusingly the booleans are inverted,
        // forward throttle uses 'does_auto_throttle' whereas vertical uses 'is_vtol_man_throttle'.
        return !does_auto_throttle();
    }
    return false;
}

// Deny changing into forward flight modes at low altitude
bool Mode::allow_enter_at_current_alt() const
{
    if (!hal.util->get_soft_armed() ||
            is_vtol_mode() ||
            !((plane.quadplane.options & QuadPlane::OPTION_FW_MODE_CHANGE_ALT_LIMIT) != 0)) {
        // always allowed to change mode if disarmed
        // always allowed to enter a vtol mode
        // check disabled
        return true;
    }
    float min_alt_cm = 0.0;
    bool have_min_alt = false;
    if (((plane.fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MIN) != 0)) {
        min_alt_cm = plane.fence.get_safe_alt_min()*100;
        have_min_alt = true;
    }
    if (plane.g.FBWB_min_altitude_cm > 0) {
        min_alt_cm = MAX(min_alt_cm, plane.g.FBWB_min_altitude_cm);
        have_min_alt = true;
    }
    if (!have_min_alt) {
        // no altitude limit set
        return true;
    }
    int32_t alt_cm;
    if (!plane.current_loc.initialised() || !plane.current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_cm)) {
        // don't know current altitude
        return true;
    }
    if (alt_cm < min_alt_cm) {
        // under threshold altitude
        gcs().send_text(MAV_SEVERITY_NOTICE, "Must be above %0.1fm for fixedwing mode: %s", min_alt_cm*0.01, name());
        return false;
    }
    return true;
}

