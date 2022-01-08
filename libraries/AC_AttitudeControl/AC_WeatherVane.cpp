/*
 * Aircraft Weathervane options common to vtol plane and copters
 */
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include "AC_WeatherVane.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    #define WVANE_PARAM_ENABLED 1
    #define WVANE_PARAM_SPD_MAX_DEFAULT 0
    #define WVANE_PARAM_VELZ_MAX_DEFAULT 0
#else
    #define WVANE_PARAM_ENABLED 0
    #define WVANE_PARAM_SPD_MAX_DEFAULT 2
    #define WVANE_PARAM_VELZ_MAX_DEFAULT 1
#endif


const AP_Param::GroupInfo AC_WeatherVane::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description{Copter}: Enable weather vaning.  When active, and the appropriate _OPTIONS bit is set for auto, or guided, the aircraft will yaw into wind once the vehicle is in a condition that meets that set by the WVANE parameters and there has been no pilot input for 2 seconds.
    // @Description{Plane}: Enable weather vaning.  When active, the aircraft will automatically yaw into wind when in a VTOL position controlled mode. Pilot yaw commands overide the weathervaning action.
    // @Values: 0:Disabled,1:Nose into wind,2:Nose or tail into wind,3:Side into wind
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AC_WeatherVane, _direction, WVANE_PARAM_ENABLED, AP_PARAM_FLAG_ENABLE),

    // @Param: GAIN
    // @DisplayName: Weathervaning gain
    // @Description: This converts the target roll/pitch angle of the aircraft into the correcting (into wind) yaw rate. e.g. Gain = 2, roll = 30 deg, pitch = 0 deg, yaw rate = 60 deg/s.
    // @Range: 0.5 4
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("GAIN", 2, AC_WeatherVane, _gain, 1.0),

    // @Param: ANG_MIN
    // @DisplayName: Weathervaning min angle
    // @Description: The minimum target roll/pitch angle before active weathervaning will start.  This provides a dead zone that is particularly useful for poorly trimmed quadplanes.
    // @Units: deg
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ANG_MIN", 3, AC_WeatherVane, _min_dz_ang_deg, 1.0),

    // @Param: HGT_MIN
    // @DisplayName: Weathervaning min height
    // @Description{Copter}: Above this height weathervaning is permitted.  If a range finder is fitted or if terrain is enabled, this parameter sets height AGL.  Otherwise, this parameter sets height above home.  Set zero to ignore minimum height requirement to activate weathervaning.
    // @Description{Plane}: Above this height weathervaning is permitted.  If RNGFND_LANDING is enabled or terrain is enabled then this parameter sets height AGL. Otherwise this parameter sets height above home.  Set zero to ignore minimum height requirement to activate weathervaning
    // @Units: m
    // @Range: 0 50
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HGT_MIN", 4, AC_WeatherVane, _min_height, 0.0),

    // @Param: SPD_MAX
    // @DisplayName: Weathervaning max ground speed
    // @Description: Below this ground speed weathervaning is permitted. Set to 0 to ignore this condition when checking if vehicle should weathervane.
    // @Units: m/s
    // @Range: 0 50
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("SPD_MAX", 5, AC_WeatherVane, _max_vel_xy, WVANE_PARAM_SPD_MAX_DEFAULT),

    // @Param: VELZ_MAX
    // @DisplayName: Weathervaning max vertical speed
    // @Description: The maximum climb or descent speed that the vehicle will still attempt to weathervane. Set to 0 to ignore this condition to get the aircraft to weathervane at any climb/descent rate.  This is particularly useful for aircraft with low disc loading that struggle with yaw control in decent.
    // @Units: m/s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("VELZ_MAX", 6, AC_WeatherVane, _max_vel_z, WVANE_PARAM_VELZ_MAX_DEFAULT),

    AP_GROUPEND
};


// Constructor
AC_WeatherVane::AC_WeatherVane(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

float AC_WeatherVane::get_yaw_rate_cds(const float roll_cdeg, const float pitch_cdeg)
{
    if (!active_msg_sent) {
        gcs().send_text(MAV_SEVERITY_INFO, "Weathervane Active");
        active_msg_sent = true;
    }

    // Normalise the roll and pitch targets to be between -1 and 1
    const float deadzone_cdeg = _min_dz_ang_deg*100.0;
    float output = 0.0;
    switch (get_direction()) {
        case Direction::OFF:
            last_output = 0.0;
            return 0.0;


        case Direction::NOSE_IN:
            if (!is_positive(pitch_cdeg)) {
                output = MAX(fabsf(roll_cdeg) - deadzone_cdeg, 0.0);
            } else {
                output = fabsf(roll_cdeg) + pitch_cdeg;
            }
            if (is_negative(roll_cdeg)) {
                output *= -1.0;
            }
            break;

        case Direction::NOSE_OR_TAIL_IN:
            output = MAX(fabsf(roll_cdeg) - deadzone_cdeg, 0.0);
            if (is_negative(roll_cdeg) != is_positive(pitch_cdeg)) {
                output *= -1.0;
            }
            break;

        case Direction::SIDE_IN:
            output = MAX(fabsf(pitch_cdeg) - deadzone_cdeg, 0.0);
            if (is_positive(pitch_cdeg) != is_positive(roll_cdeg)) {
                output *= -1.0;
            }
            break;
    }

    // Slew output
    last_output = 0.98 * last_output + 0.02 * output;

    // apply gain and scale factor to convert gains to same as originally used by plane
    return (last_output * _gain) / 45.0;
}

// Called on an interrupt to reset the weathervane controller
void AC_WeatherVane::reset(void)
{
    last_output = 0;
    active_msg_sent = false;
    first_activate_ms = 0;
    last_check_ms = AP_HAL::millis();
}

// Returns true if the vehicle is in a condition whereby weathervaning is allowed
// pilot_yaw can be an angle or a rate or rcin from yaw channel. It just needs to represent a pilot's request to yaw the vehicle.
bool AC_WeatherVane::should_weathervane(const int16_t pilot_yaw, const float hgt)
{
    // Check enabled
    if (get_direction() == Direction::OFF) {
        reset();
        return false;
    }

    // Check if weathervaning is allowed. This allows the use of switched
    // and mavlink commands to allow the wethervane controller
    if (!allowed) {
        reset();
        return false;
    }

    // Don't fight pilot inputs
    if (pilot_yaw != 0) {
        reset();
        return false;
    }

    // Check if we are above the minimum height to weather vane
    if (_min_height.get() > 0 && hgt <= _min_height.get()) {
        reset();
        return false;
    }

    // Check if we meet the horizontal velocity thresholds to allow weathervaning
    Vector3f vel_ned;
    // Check if we meet the vertical velocity thresholds to allow weathervaning
    if (!AP::ahrs().get_velocity_NED(vel_ned) || ((_max_vel_z.get() > 0) && (fabsf(vel_ned.z) > _max_vel_z.get()))) {
        reset();
        return false;
    }

    // set z speed to zero so we can get xy speed^2 from .length_squared()
    vel_ned.z = 0.0;
    if ((_max_vel_xy.get() > 0) && (vel_ned.length_squared() > (_max_vel_xy.get()*_max_vel_xy.get()))) {
        reset();
        return false;
    }


    const uint32_t now = AP_HAL::millis();
    // not run this function recently, make sure it has been reset
    if (now - last_check_ms > 250) {
        reset();
    }
    last_check_ms = now;

    /*
      Use a 2 second buffer to ensure weathervaning occurs once the vehicle has
      clearly achieved an acceptable condition.
    */
    if (first_activate_ms == 0) {
        first_activate_ms = now;
    }
    if (now - first_activate_ms < 2000) {
        return false;
    }

    // If we got this far then we should allow weathervaning
    return true;
}

AC_WeatherVane *AC_WeatherVane::_singleton;

namespace AP {
    AC_WeatherVane *weathervane()
    {
        return AC_WeatherVane::get_singleton();
    }
}
