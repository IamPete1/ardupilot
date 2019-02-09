/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  control code for tailsitters. Enabled by setting Q_FRAME_CLASS=10 
 */

#include "Plane.h"

/*
  return true when flying a tailsitter
 */
bool QuadPlane::is_tailsitter(void) const
{
    return available() && frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER;
}

/*
  check if we are flying as a tailsitter
 */
bool QuadPlane::tailsitter_active(void)
{
    if (!is_tailsitter()) {
        return false;
    }
    if (in_vtol_mode()) {
        return true;
    }
    // check if we are in ANGLE_WAIT fixed wing transition
    if (transition_state == TRANSITION_ANGLE_WAIT_FW) {
        return true;
    }
    return false;
}

/*
  run output for tailsitters
 */
void QuadPlane::tailsitter_output(void)
{
    if (!is_tailsitter()) {
        return;
    }

    // record plane gain outputs before overwriting with VTOL gains in case they are needed for interpolation
    float fw_aileron;
    float fw_elevator;
    float fw_rudder;
    float fw_tilt_left = 0;
    float fw_tilt_right = 0;
    //float fw_throttle_left;
    //float fw_throttle_right;

    if (tailsitter_active()) { // if in Qmode we need to match the Q rates with plane controller
        // no fixed wing yaw controller so cannot stabilize VTOL roll
        //const float roll_rate = attitude_control->get_rate_roll_pid().get_pid_info().desired * 100;
        const float pitch_rate = attitude_control->get_rate_pitch_pid().get_pid_info().desired * 100;
        const float yaw_rate = attitude_control->get_rate_yaw_pid().get_pid_info().desired * 100;
        const float speed_scaler = plane.get_speed_scaler();

        // due to reference frame change roll and yaw are swapped
        fw_aileron = plane.rollController.get_rate_out(-yaw_rate,  speed_scaler);
        fw_elevator = plane.pitchController.get_rate_out(pitch_rate, speed_scaler);
        //fw_rudder = plane.yawController.get_rate_out(roll_rate, speed_scaler);

        // use roll as rudder input and output direct as with plane
        fw_rudder = plane.channel_roll->get_control_in();

        /* Use VTOL throttle as no rudder stabilization in plane yet
        const float throttle = motors->get_throttle() * 100;
        const float rud_gain = float(plane.g2.rudd_dt_gain) / 100;

        // Calculate differential thrust in forward flight
        if (rud_gain > 0){
            float rudder = rud_gain * fw_rudder / float(SERVO_MAX);
            if (throttle < 0 && plane.aparm.throttle_min < 0) {
                // doing reverse thrust
                fw_throttle_left  = constrain_float(throttle + 50 * rudder, -100, 0);
                fw_throttle_right = constrain_float(throttle - 50 * rudder, -100, 0);
            } else if (throttle <= 0) {
                fw_throttle_left  = fw_throttle_right = 0;
            } else {
                // doing forward thrust
                fw_throttle_left  = constrain_float(throttle + 50 * rudder, 0, 100);
                fw_throttle_right = constrain_float(throttle - 50 * rudder, 0, 100);
            }
        } else {
            fw_throttle_left = throttle;
            fw_throttle_right = throttle;
        }
        */

    } else {
        fw_aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
        fw_elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
        fw_rudder = SRV_Channels::get_output_scaled(SRV_Channel::k_rudder);
        //fw_throttle_left = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleLeft);
        //fw_throttle_right = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleRight);
    }

    if (tailsitter.vectored_forward_gain > 0) {
        // thrust vectoring in fixed wing flight
        fw_tilt_left  = (fw_elevator + fw_aileron) * tailsitter.vectored_forward_gain;
        fw_tilt_right = (fw_elevator - fw_aileron) * tailsitter.vectored_forward_gain;
    }

    if ((!tailsitter_active() || in_tailsitter_vtol_transition()) && !assisted_flight) {
        // output tilts for forward flight
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, fw_tilt_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, fw_tilt_right);

        if (in_tailsitter_vtol_transition() && !throttle_wait && is_flying() && hal.util->get_soft_armed()) {
            /*
              during transitions to vtol mode set the throttle to the
              hover throttle, and set the altitude controller
              integrator to the same throttle level
             */
            uint8_t throttle = motors->get_throttle_hover() * 100;
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, throttle);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, throttle);
            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);
            pos_control->get_accel_z_pid().set_integrator(throttle*10);
        }
        return;
    }

    if (assisted_flight) {
        control_stabilize();
        motors_output(true);
    } else {
        motors_output(false);
    }

    // if in Qassist still a good idea to use copter I term and zero plane I to prevent windup
    plane.pitchController.reset_I();
    plane.rollController.reset_I();

    if (hal.util->get_soft_armed()) {
        // scale surfaces for throttle
        tailsitter_speed_scaling();
    }

    if (tailsitter.vectored_hover_gain > 0) {
        // thrust vectoring VTOL modes
        tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
        tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);
        /*
          apply extra elevator when at high pitch errors, using a
          power law. This allows the motors to point straight up for
          takeoff without integrator windup
         */
        int32_t pitch_error_cd = (plane.nav_pitch_cd - ahrs_view->pitch_sensor) * 0.5;
        float extra_pitch = constrain_float(pitch_error_cd, -4500, 4500) / 4500.0;
        float extra_sign = extra_pitch > 0?1:-1;
        // only use extra elevator in VTOL modes
        float extra_elevator = 0;
        if (!assisted_flight){
            extra_elevator = extra_sign * powf(fabsf(extra_pitch), tailsitter.vectored_hover_power) * 4500;
        }
        tilt_left  = extra_elevator + tilt_left * tailsitter.vectored_hover_gain;
        tilt_right = extra_elevator + tilt_right * tailsitter.vectored_hover_gain;
        if (fabsf(tilt_left) >= 4500 || fabsf(tilt_right) >= 4500) {
            // prevent integrator windup
            motors->limit.roll_pitch = 1;
            motors->limit.yaw = 1;
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
    }

    // apply speed scaling to interplate between fixed wing and VTOL outputs based on airspeed
    float aspeed;
    bool have_airspeed = ahrs.airspeed_estimate(&aspeed);
    // only bother if it will change the output
    if (aspeed >= (tailsitter.scaling_speed - tailsitter.scaling_range * 0.5) && have_airspeed) {
        // get VTOL mode outputs
        const float VTOL_aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
        const float VTOL_elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
        const float VTOL_rudder = SRV_Channels::get_output_scaled(SRV_Channel::k_rudder);
        const float VTOL_tilt_left  = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
        const float VTOL_tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);
        // No fixed wing yaw contoller so just use VTOL all the time
        //const float VTOL_throttle_left = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleLeft);
        //const float VTOL_throttle_right = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleRight);

        // caculate ratio of  gains
        float VTOL_rato = 0.5f + (aspeed - tailsitter.scaling_speed) * -(1.0f/tailsitter.scaling_range);
        VTOL_rato = constrain_float(VTOL_rato, 0.0f, 1.0f);
        const float fw_ratio = 1.0f - VTOL_rato;

        // caculate interpolated outputs
        const float aileron_interp =  VTOL_aileron * VTOL_rato + fw_aileron * fw_ratio;
        const float elevator_interp = VTOL_elevator * VTOL_rato + fw_elevator * fw_ratio;
        const float rudder_interp = VTOL_rudder * VTOL_rato + fw_rudder * fw_ratio;
        const float tilt_left_interp = VTOL_tilt_left * VTOL_rato + fw_tilt_left * fw_ratio;
        const float tilt_right_interp = VTOL_tilt_right * VTOL_rato + fw_tilt_right * fw_ratio;
        //const float throttle_left_interp = VTOL_throttle_left * VTOL_rato + fw_throttle_left * fw_ratio;
        //const float throttle_right_interp = VTOL_throttle_right * VTOL_rato + fw_throttle_right * fw_ratio;

        // set outputs
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron_interp);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator_interp);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder_interp);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left_interp);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right_interp);
        //SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, throttle_left_interp);
        //SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, throttle_right_interp);
    }

    if (tailsitter.input_mask_chan > 0 &&
        tailsitter.input_mask > 0 &&
        RC_Channels::get_radio_in(tailsitter.input_mask_chan-1) > 1700) {
        // the user is learning to prop-hang
        if (tailsitter.input_mask & TAILSITTER_MASK_AILERON) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_roll->get_control_in_zero_dz());
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_ELEVATOR) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_pitch->get_control_in_zero_dz());
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_THROTTLE) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_throttle_input(true));
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_RUDDER) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, plane.channel_rudder->get_control_in_zero_dz());
        }
    }
}


/*
  return true when we have completed enough of a transition to switch to fixed wing control
 */
bool QuadPlane::tailsitter_transition_fw_complete(void)
{
    if (plane.fly_inverted()) {
        // transition immediately
        return true;
    }
    int32_t roll_cd = labs(ahrs_view->roll_sensor);
    if (roll_cd > 9000) {
        roll_cd = 18000 - roll_cd;
    }
    if (labs(ahrs_view->pitch_sensor) > tailsitter.transition_angle*100 ||
        roll_cd > tailsitter.transition_angle*100 ||
        AP_HAL::millis() - transition_start_ms > uint32_t(transition_time_ms)) {
        return true;
    }
    // still waiting
    return false;
}


/*
  return true when we have completed enough of a transition to switch to VTOL control
 */
bool QuadPlane::tailsitter_transition_vtol_complete(void) const
{
    if (plane.fly_inverted()) {
        // transition immediately
        return true;
    }
    if (labs(plane.ahrs.pitch_sensor) > tailsitter.transition_angle*100 ||
        labs(plane.ahrs.roll_sensor) > tailsitter.transition_angle*100 ||
        AP_HAL::millis() - transition_start_ms > 2000) {
        return true;
    }
    // still waiting
    attitude_control->reset_rate_controller_I_terms();
    return false;
}

// handle different tailsitter input types
void QuadPlane::tailsitter_check_input(void)
{
    if (tailsitter_active() &&
        tailsitter.input_type == TAILSITTER_INPUT_PLANE) {
        // the user has asked for body frame controls when tailsitter
        // is active. We switch around the control_in value for the
        // channels to do this, as that ensures the value is
        // consistent throughout the code
        int16_t roll_in = plane.channel_roll->get_control_in();
        int16_t yaw_in = plane.channel_rudder->get_control_in();
        plane.channel_roll->set_control_in(yaw_in);
        plane.channel_rudder->set_control_in(-roll_in);
    }
}

/*
  return true if we are a tailsitter transitioning to VTOL flight
 */
bool QuadPlane::in_tailsitter_vtol_transition(void) const
{
    return is_tailsitter() && in_vtol_mode() && transition_state == TRANSITION_ANGLE_WAIT_VTOL;
}

/*
  account for speed scaling of control surfaces in hover
*/
void QuadPlane::tailsitter_speed_scaling(void)
{
    const float hover_throttle = motors->get_throttle_hover();
    const float throttle = motors->get_throttle();
    float scaling;

    if (is_zero(throttle)) {
        scaling = tailsitter.throttle_scale_max;
    } else {
        scaling = constrain_float(hover_throttle / throttle, 0, tailsitter.throttle_scale_max);
    }

    const SRV_Channel::Aux_servo_function_t functions[4] = {
        SRV_Channel::Aux_servo_function_t::k_aileron,
        SRV_Channel::Aux_servo_function_t::k_elevator,
        SRV_Channel::Aux_servo_function_t::k_tiltMotorLeft,
        SRV_Channel::Aux_servo_function_t::k_tiltMotorRight};
    for (uint8_t i=0; i<ARRAY_SIZE(functions); i++) {
        int32_t v = SRV_Channels::get_output_scaled(functions[i]);
        v *= scaling;
        v = constrain_int32(v, -SERVO_MAX, SERVO_MAX);
        SRV_Channels::set_output_scaled(functions[i], v);
    }
}
