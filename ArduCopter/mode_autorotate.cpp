#include "Copter.h"
#include <AC_Autorotation/AC_Autorotation.h>
#include "mode.h"

#include <utility>

#if MODE_AUTOROTATE_ENABLED == ENABLED

/*
 * Init and run calls for autorotate flight mode
 */


bool Copter::ModeAutorotate::init(bool ignore_checks)
{

#if FRAME_CONFIG != HELI_FRAME
    //Only allow trad heli to use autorotation mode
    return false;
#endif

//Check that interlock is disengaged
if (motors->get_interlock()) {
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation Mode change fail: Interlock Engaged");
    return false;
}

    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_instance();

    //Get current rpm, checking to ensure no nullptr
    if (rpm != nullptr) {
        _initial_rpm = rpm->get_rpm(0);
    } else {
        //Prevent access to mode.  RPM sensor must be fitted to use autorotation mode
        gcs().send_text(MAV_SEVERITY_INFO, "Autorotation Mode change fail: No RPM sensor");
        return false;
    }

    //Display message 
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation initiated");

    //Retrieve parameter values from autorotation library
    arot->set_param_values(&_param_target_head_speed, &_param_head_speed_hover, &_param_accel_max, &_param_target_airspeed, &_param_td_alt, &_param_col_entry_cutoff_freq, &_param_col_glide_cutoff_freq);

     //set all inial flags to on
    _flags.entry_initial = 1;
    _flags.ss_glide_initial = 1;
    _flags.flare_initial = 1;
    _flags.touch_down_initial = 1;
    _flags.level_initial = 1;
    _flags.break_initial = 1;
    _flags.straight_ahead_initial = 1;

    //Prevent divide by zero error
    if (_param_head_speed_hover < 500) {
        _param_head_speed_hover = 500;  //Making sure that hover rpm is not unreasonably low
    }

    //Record initial speed to be maintained 
    _inital_airspeed = helispdhgtctrl->calc_speed_forward() * 100.0f;

    //initialise head speed/collective controller
    arot->init_hs_controller();

    //initialise speed/height controller
    helispdhgtctrl->init_controller();

    //TEMP:  TO BE REMOVED
    message_counter = 0;

    //setting default starting switches
    phase_switch = ENTRY;
    //phase_switch = SS_GLIDE;

    //set hover head speed in head speed controller
    arot->set_head_speed_hover(_param_head_speed_hover);

    //set 3 seconds on entry timer
    _entry_time_remain = AUTOROTATE_ENTRY_TIME;

    //The decay rate to reduce the head speed from the current to the target
    _hs_decay = ((_initial_rpm/_param_head_speed_hover) - _param_target_head_speed) / AUTOROTATE_HS_ADJUST_TIME; //Decay rate in head speed over 3 secs

    return true;
}



void Copter::ModeAutorotate::run()
{

    //check if interlock becomes engaged
    if (motors->get_interlock()) {
        set_mode(copter.prev_control_mode, MODE_REASON_AUTOROTATION_BAILOUT);
    }

    // current time
    now = millis(); //milliseconds

    //initialise internal variables
    float des_z;
    float curr_alt = inertial_nav.get_position().z;     // Current altitude
    float curr_vel_z = inertial_nav.get_velocity().z;


    //----------------------------------------------------------------
    //                  State machine logic
    //----------------------------------------------------------------

     //Setting default phase switch positions
     nav_pos_switch = STRAIGHT_AHEAD;

    //altitude check to jump to correct flight phase if at low height
    if (curr_alt <= _param_td_alt) {
        //jump to touch down phase
        //phase_switch = TOUCH_DOWN;
    }

    //Timer from entry phase to progress to glide phase
    if (phase_switch == ENTRY){

        //update entry phase timer
        _entry_time_remain -= G_Dt;

        if (_entry_time_remain < 0.0f) {
            //Flight phase can be progressed to steady state glide
            phase_switch = SS_GLIDE;
        }

    }


    //----------------------------------------------------------------
    //                  State machine actions
    //----------------------------------------------------------------
    switch (phase_switch) {

        case ENTRY:
        {
            //Entry phase functions to be run only once
            if (_flags.entry_initial == 1) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Entry Phase");
                #endif

                //set following trim low pass cut off frequency
                arot->set_col_cutoff_freq(_param_col_entry_cutoff_freq);

                //target head speed is set to rpm at initiation to prevent unwanted changes in attitude
                _target_head_speed = _initial_rpm/_param_head_speed_hover;

                //Set desired air speed target
                _aspeed = _param_target_airspeed;
                helispdhgtctrl->set_desired_speed(_aspeed);

                //set acceleration limit
                _att_accel_max = _param_accel_max;
                helispdhgtctrl->set_max_accel(_att_accel_max);

                //Prevent running the initial entry functions again
                _flags.entry_initial = 0;

            }

            //Slowly change the target head speed until the target 
            //head speed matches the parameter defined value
            if (_target_head_speed > _param_target_head_speed*1.005f  ||  _target_head_speed < _param_target_head_speed*0.995f) {
                _target_head_speed -= _hs_decay*G_Dt;
            } else {
                _target_head_speed = _param_target_head_speed;
            }

            //set target head speed in head speed controller
            arot->set_target_head_speed(_target_head_speed);

            //run airspeed/attitude controller
            helispdhgtctrl->set_dt(G_Dt);
            helispdhgtctrl->update_speed_controller();

            //retrieve pitch target from helispdhgtctrl 
            _pitch_target = helispdhgtctrl-> get_pitch();

            float pilot_roll, pilot_pitch;
            get_pilot_desired_lean_angles(pilot_roll, pilot_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

            //get pilot's desired yaw rate
            float pilot_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

            //update controllers
            arot->update_hs_glide_controller(G_Dt); //run head speed/ collective controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pilot_roll, _pitch_target, pilot_yaw_rate);
            //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pilot_roll, pilot_pitch, pilot_yaw_rate);

            break;

        }
        case SS_GLIDE:

        {
            //steady state glide functions to be run only once
            if (_flags.ss_glide_initial == 1) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "SS Glide Phase");
                #endif

                //set following trim low pass cut off frequency
                arot->set_col_cutoff_freq(_param_col_glide_cutoff_freq);

                //Set desired air speed target
                _aspeed = _param_target_airspeed;
                helispdhgtctrl->set_desired_speed(_aspeed);

               //set target head speed in head speed controller
                arot->set_target_head_speed(_target_head_speed);

                //set acceleration limit
                _att_accel_max = _param_accel_max;
                helispdhgtctrl->set_max_accel(_att_accel_max);

                //Prevent running the initial glide functions again
                _flags.ss_glide_initial = 0;
            }

            //run airspeed/attitude controller
            helispdhgtctrl->set_dt(G_Dt);
            helispdhgtctrl->update_speed_controller();

            //retrieve pitch target from helispdhgtctrl 
            _pitch_target = helispdhgtctrl-> get_pitch();

            //temp
            float pilot_roll, pilot_pitch;
            get_pilot_desired_lean_angles(pilot_roll, pilot_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

            //get pilot's desired yaw rate
            float pilot_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

            //update controllers
            arot->update_hs_glide_controller(G_Dt); //run head speed/ collective controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pilot_roll, _pitch_target, pilot_yaw_rate);
            //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pilot_roll, pilot_pitch, pilot_yaw_rate);
            
            break;
        }

        case FLARE:


            break;


        case TOUCH_DOWN:

            if (_flags.touch_down_initial == 1) {
                //functions and settings to be done once are done here.

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Touch Down Initiated");
                #endif

                //set limits of z velocity and z acceleration
                pos_control->set_speed_z(-800, 500);
                pos_control->set_accel_z(200000);

                //Get height that touchdown phase starts is initiated at
                _z_touch_down_start = curr_alt;

                //Set time that touchdown phase is initiated at
                _t_touch_down_initiate = now;

                // get collective aggression from parameter
                _collective_aggression = 0.15f;

                _flags.touch_down_initial = 0;

            }

            //Calculate desired z
            des_z = _z_touch_down_start * 0.1f * expf(-_collective_aggression * (now - _t_touch_down_initiate)/1000.0f); // + terrain_offset (cm)

            pos_control->set_alt_target(des_z);

            pos_control->update_z_controller();

            break;

    }



    switch (nav_pos_switch) {

        case STRAIGHT_AHEAD:

        //    if (_flags.straight_ahead_initial == 1) {
                //functions and settings to be done once are done here.

        //        #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        //            gcs().send_text(MAV_SEVERITY_INFO, "Straight ahead");
         //       #endif

        //        _flags.straight_ahead_initial = 0;
        //    }

        //    // convert pilot input to lean angles
        //    float target_roll, target_pitch;
        //    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

        //    // get pilot's desired yaw rate
        //    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        //    // call attitude controller
         //   attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);


            break;


        case INTO_WIND:

            break;


        case NEAREST_RALLY:

            break;

        case BREAK:


            break;


        case LEVEL:

            break;

    }



//  These message outputs are purely for debugging purposes.  Will be removed in future.
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (message_counter == 300) {
        //gcs().send_text(MAV_SEVERITY_INFO, "Initial Speed = %.3f",_inital_airspeed);
        //gcs().send_text(MAV_SEVERITY_INFO, "--- --- --- ---");
        message_counter = 0;
    }
message_counter++;
#endif


    //Write to data flash log
    if (log_counter++ % 20 == 0) {
        DataFlash_Class::instance()->Log_Write("AROT", "TimeUS,InVz,Glide,DPz,CRPM,AcL,ASTg,Pit", "Qfffffff",
                                                AP_HAL::micros64(),
                                               (double)curr_vel_z,
                                               (double)get_ned_glide_angle(),
                                               (double)des_z,
                                               (double)arot->get_rpm(),
                                               (double)_att_accel_max,
                                               (double)_aspeed,
                                               (double)_pitch_target);
    }

}



// Determine the glide path angle based on NED frame velocities.  This method determines the instaintanious glide angle,
// not accounting for the wind.  Wind direction approximation/learning is done in another function, using this function.
float Copter::ModeAutorotate::get_ned_glide_angle(void)
{
    float sink_velocity = inertial_nav.get_velocity().z; //(cm/s)

    //Convert sink velocity to m/s and convert sink velocity to be positive down
    sink_velocity /= -100.0f; //(m/s)

    //remove climb rates from sink velocity
    if (sink_velocity < 0) {
        sink_velocity = 0;
    }

    //Determine ground speed
    Vector2f _groundspeed_vector = ahrs.groundspeed_vector();
    float ground_speed_forward = _groundspeed_vector.x*ahrs.cos_yaw() + _groundspeed_vector.y*ahrs.sin_yaw(); //(m/s)

    //catch divide by zero
    if (ground_speed_forward < 0.01f) {
        ground_speed_forward = 0.01f;
    }

    //Calculate glide slope
    float windless_glide_slope = atanf(sink_velocity/ground_speed_forward);

    return windless_glide_slope;
}



//TODO: winding direction approximation learning via circuling descent


void Copter::ModeAutorotate::errormessage(int message_number)
{
    
    
    

}





#endif