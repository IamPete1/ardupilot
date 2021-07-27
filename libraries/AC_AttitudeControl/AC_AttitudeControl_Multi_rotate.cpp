#ifdef ENABLE_SCRIPTING

#include "AC_AttitudeControl_Multi_rotate.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// run lowest level body-frame rate controller and send outputs to the motors
void AC_AttitudeControl_Multi_rotate::rate_controller_run() {

    _current_target = _ang_vel_body;

    Matrix3f rot;
    rot.from_euler312(radians(roll_offset_deg), radians(pitch_offset_deg), 0.0f);
    Vector3f rotated_target = rot * _ang_vel_body;

    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();

    rotated_target += _sysid_ang_vel_body;

    Vector3f gyro_latest = _ahrs.get_gyro_latest();

    _motors.set_roll(get_rate_roll_pid().update_all(rotated_target.x, gyro_latest.x, _motors.limit.roll) + _actuator_sysid.x);
    _motors.set_roll_ff(get_rate_roll_pid().get_ff());

    _motors.set_pitch(get_rate_pitch_pid().update_all(rotated_target.y, gyro_latest.y, _motors.limit.pitch) + _actuator_sysid.y);
    _motors.set_pitch_ff(get_rate_pitch_pid().get_ff());

    _motors.set_yaw(get_rate_yaw_pid().update_all(rotated_target.z, gyro_latest.z, _motors.limit.yaw) + _actuator_sysid.z);
    _motors.set_yaw_ff(get_rate_yaw_pid().get_ff()*_feedforward_scalar);

    _sysid_ang_vel_body.zero();
    _actuator_sysid.zero();

    control_monitor_update();
}

AC_AttitudeControl_Multi_rotate *AC_AttitudeControl_Multi_rotate::_singleton = nullptr;

#endif // ENABLE_SCRIPTING
