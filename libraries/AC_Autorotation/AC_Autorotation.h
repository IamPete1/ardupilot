#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Motors/AP_Motors.h>          // motors library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude control library
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <Filter/Filter.h>                     // Filter library
#include <Filter/LowPassFilter.h>      // LowPassFilter class (inherits from Filter class)




// Head Speed (HS) controller default definitions
#define HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ          2.0f    // low-pass filter on accel error (unit: hz)
#define HS_CONTROLLER_HEADSPEED_P                     0.7f    // Defualt P gain for head speed controller (unit: -)
#define HS_CONTROLLER_HEADSPEED_D                     0.01f    // Defualt D gain for head speed controller (unit: -)
#define HS_CONTROLLER_HEADSPEED_TARGET                0.9f    // Defualt D gain for head speed controller (unit: -)




class AC_Autorotation
{
public:

    /// Constructor
AC_Autorotation(const AP_AHRS_View& ahrs,
                     const AP_InertialNav& inav,
                     AP_Motors& motors,
                     AC_AttitudeControl& attitude_control) :
    _ahrs(ahrs),
    _inav(inav),
    _motors(motors),
    _attitude_control(attitude_control)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    //-------- functions --------
    void init_hs_controller(void);

    void update_hs_glide_controller(float dt);  //Update head speed controller
    void update_att_glide_controller(float dt);  //Update attitude controller
    void set_collective(float _collective_filter_cutoff);

    float get_rpm() { return _current_rpm; }

    void set_head_speed_hover(float hsh) { _head_speed_hover = hsh; }

    void set_target_head_speed(float ths) { _target_head_speed = ths; }

    void set_col_cutoff_freq(float freq) { _col_cutoff_freq = freq; }

    void set_param_values(float* targ_hs, float* hs_hov, float* accel, float* targ_s, float* td_alt, float* ent_freq, float* glide_freq, float* bail_time);

    float get_last_collective() { return _collective_out; }

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];


protected:

    //----------------------------
    //---- internal variables ----
    //----------------------------
    float _current_rpm;

    float _collective_out = 0.0f;

    float _head_speed_error;  // error between target head speed and current head speed.  Normalised by hover head speed.

    float _last_head_speed_error;

    uint16_t _log_counter = 0;

    float _target_head_speed;  //(rpm) normalised head speed to be maintined in the autorotation (normalised by hover head speed)

    float _head_speed_hover;   //(rpm) head speed during normal hover

    float _col_cutoff_freq;

    // Parameter values
    AP_Float _param_hs_p;               //p value for rpm collective controller
    AP_Float _param_hs_d;               //d value for rpm collective controller
    AP_Float _param_target_head_speed;  //(rpm) normalised head speed to be maintined in the autorotation (normalised by hover head speed)
    AP_Float _param_head_speed_hover;   //(rpm) head speed during normal hover
    AP_Float _param_accel_max;
    AP_Float _param_target_speed;
    AP_Float _param_td_alt;
    AP_Float _param_col_entry_cutoff_freq;
    AP_Float _param_col_glide_cutoff_freq;
    AP_Float _param_bail_time;

    // low pass filter for collective trim
    LowPassFilterFloat col_trim_lpf;

    // References to other libraries
    const AP_AHRS_View&         _ahrs;
    const AP_InertialNav&       _inav;
    AP_Motors&                  _motors;
    AC_AttitudeControl&         _attitude_control;

};