/// @file    AP_SpdHgtControl_Heli.h
/// @brief   Helicopter Speed & Height Control. This is a instance of an
/// AP_SpdHgtControl class


#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Logger/AP_Logger.h>
#include <AC_PID/AC_PID.h>             // PID library
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library

#define AP_SPDHGTCTRL_VEL_P                       1.0f
#define AP_SPDHGTCTRL_VEL_I                       0.5f
#define AP_SPDHGTCTRL_VEL_D                       0.0f
#define AP_SPDHGTCTRL_VEL_IMAX                    1000.0f
#define AP_SPDHGTCTRL_VEL_FF                      0.15f
#define AP_SPDHGTCTRL_VEL_FILT_T_HZ               10.0f
#define AP_SPDHGTCTRL_VEL_FILT_E_HZ               10.0f
#define AP_SPDHGTCTRL_VEL_FILT_D_HZ               1.0f
#define AP_SPDHGTCTRL_VEL_DT                      0.0025f

class AP_SpdHgtControl_Heli {
public:
    AP_SpdHgtControl_Heli(AP_AHRS &ahrs,
                          AP_InertialNav& inav):
        _ahrs(ahrs),
        _inav(inav),
        _pid_vel(AP_SPDHGTCTRL_VEL_P, AP_SPDHGTCTRL_VEL_I, AP_SPDHGTCTRL_VEL_D, AP_SPDHGTCTRL_VEL_FF, AP_SPDHGTCTRL_VEL_IMAX, AP_SPDHGTCTRL_VEL_FILT_T_HZ, AP_SPDHGTCTRL_VEL_FILT_E_HZ, AP_SPDHGTCTRL_VEL_FILT_D_HZ, AP_SPDHGTCTRL_VEL_DT)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // reset controller
    void init_controller(void);

    // update speed controller for use in heli wp nav
    void update_speed_controller(void);

    // set_desired_speed - set speed for controller
    void set_desired_speed(float speed) { _vel_target = speed; };

    // get_pitch_target
    int32_t get_pitch(void) const { return (int32_t)_pitch_target; };

    AC_PID& get_vel_pid() { return _pid_vel; };

    // set max accel
    void set_max_accel(float accel_max)
    {
        if (accel_max > 40.0f) {
            _accel_max = accel_max;
        } else {
            _accel_max = 40.0f;  //hard coded acceleration limit, to prevent no airspeed gain
        }
    };

    // get target accel
    float get_accel_target(void) { return accel_target; }; 

    // get delta speed forward
    float get_delta_speed(void) { return delta_speed_fwd; }; 

    // this supports the user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    //calculates an estimate of forward airspeed
    float calc_speed_forward(void);

    // set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
    void set_dt(float delta_sec);


private:

    // reference to other libraries
    AP_AHRS&           _ahrs;
    AP_InertialNav&    _inav;


    AC_PID      _pid_vel;
    float       _vel_target;
    float       _pitch_target;
    float       _vel_error;
    float       _accel_max;
    float       _speed_forward_last;
    bool        _flag_limit_accel;
    float       _accel_limit_value;
    float       _accel_out_last;
    float       _cmd_vel;
    float       accel_target;
    float       delta_speed_fwd;
    float       _dt;
    float       _speed_forward;

    LowPassFilterFloat _accel_target_filter; // acceleration target filter


    //Temp:  For logging
    uint16_t log_counter = 0;

};
