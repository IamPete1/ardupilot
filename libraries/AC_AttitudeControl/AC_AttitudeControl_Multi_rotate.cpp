#ifdef ENABLE_SCRIPTING

#include "AC_AttitudeControl_Multi_rotate.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// run lowest level body-frame rate controller and send outputs to the motors
void AC_AttitudeControl_Multi_rotate::rate_controller_run() {

    Matrix3f rot;
    rot.from_euler312(radians(roll_offset_deg), radians(pitch_offset_deg), 0.0f);
    _ang_vel_body = rot * _ang_vel_body;

    AC_AttitudeControl_Multi::rate_controller_run();
}

AC_AttitudeControl_Multi_rotate *AC_AttitudeControl_Multi_rotate::_singleton = nullptr;

#endif // ENABLE_SCRIPTING
