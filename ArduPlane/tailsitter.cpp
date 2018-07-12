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
    if (!tailsitter_active() || in_tailsitter_vtol_transition()) {
        if (tailsitter.vectored_forward_gain > 0) {
            // thrust vectoring in fixed wing flight
            float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
            float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
            float tilt_left  = (elevator + aileron) * tailsitter.vectored_forward_gain;
            float tilt_right = (elevator - aileron) * tailsitter.vectored_forward_gain;
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
        } else {
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 0);
        }
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
            pid_accel_z.set_integrator(throttle*10);
        }
		if (!in_assisted_flight() || is_zero(tailsitter.vectored_hover_gain)){
			return;
		}
    }
    
	motors_output();

	if (!in_assisted_flight()){
		plane.pitchController.reset_I();
		plane.rollController.reset_I();
	}
	
	
    if (hal.util->get_soft_armed()) {
        // scale surfaces for throttle
        tailsitter_speed_scaling();
    }

    if (tailsitter.vectored_hover_gain > 0) {
        // thrust vectoring VTOL modes
        float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
        float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
			    		
        /*
          apply extra elevator when at high pitch errors, using a
          power law. This allows the motors to point straight up for
          takeoff without integrator windup
        */
        int32_t pitch_error_cd = (plane.nav_pitch_cd - ahrs_view->pitch_sensor) * 0.5;
        float extra_pitch = constrain_float(pitch_error_cd, -4500, 4500) / 4500;
        float extra_sign = extra_pitch > 0?1:-1;
		float extra_elevator = extra_sign * powf(fabsf(extra_pitch), tailsitter.vectored_hover_power) * 4500;	    
		float tilt_left  = extra_elevator + (elevator + aileron) * tailsitter.vectored_hover_gain;
        float tilt_right = extra_elevator + (elevator - aileron) * tailsitter.vectored_hover_gain;
        if (fabsf(tilt_left) >= 4500 || fabsf(tilt_right) >= 4500) {
            // prevent integrator windup
            motors->limit.roll_pitch = 1;
            motors->limit.yaw = 1;
        }
		
		// Scale output below trim to the same scaled value per deg of movement to that above trim
		/*
		if (tilt_left < 0) {
			tilt_left *= (tailsitter.vector_max_angle / tailsitter.vector_min_angle);
		    tilt_left = constrain_float(tilt_left, -SERVO_MAX, SERVO_MAX);			
		}	
		if (tilt_right < 0) {
			tilt_right *= (tailsitter.vector_max_angle / tailsitter.vector_min_angle);
			tilt_right = constrain_float(tilt_right, -SERVO_MAX, SERVO_MAX);
		}	
		
		// Scale throttle with aileron to give constant thrust independent of yaw
		float yaw_angle = ((aileron *  tailsitter.vectored_hover_gain) / SERVO_MAX) * radians(tailsitter.vector_max_angle);
		float thrust_multiplier = 1/cosf(fabsf(yaw_angle));
		thrust_multiplier = constrain_float(thrust_multiplier,1,1.2); // Constrain to maximum to 20% extra thrust, this is equiveltn to 35 deg of tilt due to yaw
		float throttle_left = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttleLeft)*thrust_multiplier, 0, 100);
        float throttle_right = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttleRight)*thrust_multiplier, 0, 100);
        */
		
		// Output Left and right Throttle and tilt
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
		//SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, throttle_left);
        //SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, throttle_right);
		
		// Use Plane PID's with airspeed scaling for controll serfaces
		if (!in_assisted_flight()){
			float roll_rate = attitude_control->get_rate_roll_pid().get_pid_info().desired * 100; 
			float pitch_rate =  attitude_control->get_rate_pitch_pid().get_pid_info().desired * 100;
			float speed_scaler = plane.get_speed_scaler();
			SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.rollController.get_rate_out(roll_rate,  speed_scaler));
			SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitchController.get_rate_out(pitch_rate, speed_scaler));
        } else {
			return;
		}
    }
    
    
    if (tailsitter.input_mask_chan > 0 &&
        tailsitter.input_mask > 0 &&
        hal.rcin->read(tailsitter.input_mask_chan-1) > 1700) {
        // the user is learning to prop-hang
        if (tailsitter.input_mask & TAILSITTER_MASK_AILERON) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_roll->get_control_in_zero_dz());
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_ELEVATOR) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_pitch->get_control_in_zero_dz());
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_THROTTLE) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.channel_throttle->get_control_in_zero_dz());
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
	float hover_throttle = 0.5;
	float scaling = 1;

	if (is_zero(tailsitter.throttle_scale)){
        hover_throttle = motors->get_throttle_hover();
	} else {
		hover_throttle = tailsitter.throttle_scale;
	}
	
	const float throttle = motors->get_throttle();
    if (is_zero(throttle)) {
        scaling = tailsitter.throttle_scale_max;
    } else {
        scaling = constrain_float(hover_throttle / throttle, 0, tailsitter.throttle_scale_max);
    }
	
    const SRV_Channel::Aux_servo_function_t functions[2] = {
        SRV_Channel::Aux_servo_function_t::k_aileron,
        SRV_Channel::Aux_servo_function_t::k_elevator};
    for (uint8_t i=0; i<ARRAY_SIZE(functions); i++) {
        int32_t v = SRV_Channels::get_output_scaled(functions[i]);
        v *= scaling;
        v = constrain_int32(v, -SERVO_MAX, SERVO_MAX);
        SRV_Channels::set_output_scaled(functions[i], v);
    }
}
    
