#include "AC_Autorotation.h"

const AP_Param::GroupInfo AC_Autorotation::var_info[] = {

    // @Param: HS_P
    // @DisplayName: P gain for head spead controller
    // @Description: Increase value to increase sensitivity of head speed controller during autonomous autorotation.
    // @Units: -
    // @Range: 0.3 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("HS_P", 1, AC_Autorotation, _param_hs_p, HS_CONTROLLER_HEADSPEED_P),

    // @Param: HS_D
    // @DisplayName: D gain for head spead controller
    // @Description: Increase value to increase damping of head speed controller during autonomous autorotation.
    // @Units: -
    // @Range: 0.001 0.02
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("HS_D", 2, AC_Autorotation, _param_hs_d, HS_CONTROLLER_HEADSPEED_D),

    // @Param: HS_TARGET
    // @DisplayName: Target head speed for controller to achieve
    // @Description: Specify the target head speed as a percentage of hover headspeed
    // @Units: -
    // @Range: 0.8-1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("HS_TARGET", 3, AC_Autorotation, _param_target_head_speed, HS_CONTROLLER_HEADSPEED_TARGET),

    // @Param: HS_HOVER
    // @DisplayName: Temporary parameter for head speed in the hover
    // @Description: This will be replaced by hover collective learning in the future
    // @Units: -
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("HS_HOVER", 4, AC_Autorotation, _param_head_speed_hover, 1700.0f),

    // @Param: TARG_SP
    // @DisplayName: Target speed in cm/s for the autorotation controller to try and achieve/ maintain.
    // @Description: ---
    // @Units: cm/s
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("TARG_SP", 5, AC_Autorotation, _param_target_speed, 1100.0f),

    // @Param: TD_ALT
    // @DisplayName: Altitude at which to begin touch down phase
    // @Description: ---
    // @Units: cm
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("TD_ALT", 6, AC_Autorotation, _param_td_alt, 6.0f),

    // @Param: COL_FILT_E
    // @DisplayName: Cut-off frequency for collective feed forward during entry
    // @Description: ---
    // @Units: cm
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_E", 7, AC_Autorotation, _param_col_entry_cutoff_freq, 0.28f),

    // @Param: COL_FILT_G
    // @DisplayName: Cut-off frequency for collective feed forward during glide
    // @Description: ---
    // @Units: cm
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_G", 8, AC_Autorotation, _param_col_glide_cutoff_freq, 0.05f),

    // @Param: AS_ACC_MAX
    // @DisplayName: Maximum acceleration to apply in airspeed controller
    // @Description: ---
    // @Units: cm
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("AS_ACC_MAX", 9, AC_Autorotation, _param_accel_max, 100.0f),


    AP_GROUPEND
};


//initialisation of head speed controller
void AC_Autorotation::init_hs_controller()
{
    //reset 'last' terms
    _last_head_speed_error = 0.0f;

    //set initial collective position to be the collective position on initialisation
    _collective_out = 0.4f;//_motors.get_throttle();

    //Reset feed forward filter
    col_trim_lpf.reset(_collective_out);

}



void AC_Autorotation::update_hs_glide_controller(float dt)
{
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_instance();

    //Get current rpm, checking to ensure no nullptr
    if (rpm != nullptr) {
        _current_rpm = rpm->get_rpm(0);
    }

    //Normalised head speed
    float head_speed_norm = _current_rpm / _head_speed_hover;

    //set collective trim low pass filter cut off frequency
    col_trim_lpf.set_cutoff_frequency(_col_cutoff_freq);

    //Calculate the head speed error
    //Current rpm is normalised by the hover head speed.  Target head speed is defined as a percentage of hover speed
    _head_speed_error = head_speed_norm - _target_head_speed;

    float P_hs = _head_speed_error * _param_hs_p;

    //calculate head speed error differential
    float head_speed_error_differential = (_head_speed_error - _last_head_speed_error) / dt;

    float D_hs = head_speed_error_differential * _param_hs_d;

    //Adjusting collective trim using feed forward
    float FF_hs = col_trim_lpf.apply(_collective_out, dt); //note that the collective out has not yet been updated, so this value is the previous time steps collective position

    //Calculate collective position to be set
    _collective_out = P_hs + D_hs + FF_hs;

    // send collective to setting to motors output library
    set_collective(HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ);

    //save last head speed error term for differential calculation in next time step
    _last_head_speed_error = _head_speed_error;


    //Write to data flash log
    if (_log_counter++ % 20 == 0) {
        DataFlash_Class::instance()->Log_Write("ARO2", "TimeUS,P,D,hserr,hstarg,ColOut,FFCol", "Qffffff",
                                                AP_HAL::micros64(),
                                               (double)P_hs,
                                               (double)D_hs,
                                               (double)_head_speed_error,
                                               (double)_target_head_speed,
                                               (double)_collective_out,
                                               (double)FF_hs);
    }


}


//function to set collective and collective filter in motor library
void AC_Autorotation::set_collective(float collective_filter_cutoff)
{

    _motors.set_throttle_filter_cutoff(collective_filter_cutoff);
    _motors.set_throttle(_collective_out);

}


//function that sets parameter values in flight mode
void AC_Autorotation::set_param_values(float* targ_hs, float* hs_hov, float* accel, float* targ_s, float* td_alt, float* ent_freq, float* glide_freq)
{

    *targ_hs     = _param_target_head_speed;
    *hs_hov      = _param_head_speed_hover;
    *accel       = _param_accel_max;
    *targ_s      = _param_target_speed;
    *td_alt      = _param_td_alt;
    *ent_freq    = _param_col_entry_cutoff_freq;
    *glide_freq  = _param_col_glide_cutoff_freq;

}



