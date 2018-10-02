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
    Sailboat simulator class

    see explanation of lift and drag explained here: https://en.wikipedia.org/wiki/Forces_on_sails

    To-Do: add heel handling by calculating lateral force from wind vs gravity force from heel to arrive at roll rate or acceleration
*/

#include "SIM_Sailboat.h"
#include <AP_Math/AP_Math.h>
#include <string.h>
#include <stdio.h>

namespace SITL {

#define STEERING_SERVO_CH   0   // steering controlled by servo output 1
#define MAINSAIL_SERVO_CH   3   // main sail controlled by servo output 4

Sailboat::Sailboat(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    steering_angle_max(35),
    turning_circle(1.8)
{
}

// calculate the lift and drag as values from 0 to 1
// given an apparent wind speed in m/s and angle-of-attack in degrees
void Sailboat::calc_lift_and_drag(float wind_speed, float angle_of_attack_deg, float& lift, float& drag) const
{
    const uint16_t index_width_deg = 10;
    const uint8_t index_max = ARRAY_SIZE(lift_curve) - 1;

    float lift_coef;
    float drag_coef;

    // check extremes
    if (angle_of_attack_deg <= 0.0f) {
        lift_coef = lift_curve[0];
        drag_coef = drag_curve[0];
    } else if (angle_of_attack_deg >= index_max * index_width_deg) {
        lift_coef = lift_curve[index_max];
        drag_coef = drag_curve[index_max];
    } else {
        uint8_t index = constrain_int16(angle_of_attack_deg / index_width_deg, 0, index_max);
        float remainder = angle_of_attack_deg - (index * index_width_deg);
        lift_coef = linear_interpolate(lift_curve[index], lift_curve[index+1], remainder, 0.0f, index_width_deg);
        drag_coef = linear_interpolate(drag_curve[index], drag_curve[index+1], remainder, 0.0f, index_width_deg);
    }

    // apply scaling by wind speed
    lift = lift_coef; // * 0.5f * SSL_AIR_DENSITY * sail_area * pow(wind_speed,2.0f);
    drag = drag_coef; //* 0.5f * SSL_AIR_DENSITY * sail_area * pow(wind_speed,2.0f);
}

// return turning circle (diameter) in meters for steering angle proportion in the range -1 to +1
float Sailboat::get_turn_circle(float steering) const
{
    if (is_zero(steering)) {
        return 0;
    }
    return turning_circle * sinf(radians(steering_angle_max)) / sinf(radians(steering * steering_angle_max));
}

// return yaw rate in deg/sec given a steering input (in the range -1 to +1) and speed in m/s
float Sailboat::get_yaw_rate(float steering, float speed) const
{
    if (is_zero(steering) || is_zero(speed)) {
        return 0;
    }
    float d = get_turn_circle(steering);
    float c = M_PI * d;
    float t = c / speed;
    float rate = 360.0f / t;
    return rate;
}

// return lateral acceleration in m/s/s given a steering input (in the range -1 to +1) and speed in m/s
float Sailboat::get_lat_accel(float steering, float speed) const
{
    float yaw_rate = get_yaw_rate(steering, speed);
    float accel = radians(yaw_rate) * speed;
    return accel;
}

// Cauclate hull friction using delft Yacht series, (N)
// From paper - Approximation of the hydrodynamic forces on a sailing ytacht based on the delft systematic yacht hull series
// Hull drag only, keel and rudder not considered
float Sailboat::calc_hull_drag(float speed, float heel)
{
    // Froude number
    const float fn = speed / sqrt(GRAVITY_MSS * hull_coef.LWL);

   // reynolds number
    const float Rn = (abs(speed) * 0.7 * hull_coef.LWL) / hull_coef.v;

    //  Residuary resistance  //
    // Interpolate coefficents based on froude number
    const float a0 = -0.0005f; // test numbers // Coefficient_DYS
    const float a1 = -0.0005f;
    const float a2 =  0.0023f;
    const float a3 = -0.0086f;
    const float a4 = -0.0015f;
    const float a5 =  0.0061f;
    const float a6 =  0.001f;
    const float a7 =  0.0001f;
    const float a8 =  0.0052f;

    const float term_1 = a1 * (hull_coef.LCB/hull_coef.LWL) + a2 * hull_coef.Cp + a3 * pow(hull_coef.Disp,2.0f/3.0f)/hull_coef.AW + a4 * hull_coef.BWL/hull_coef.LWL;
    const float term_2 = a5 * pow(hull_coef.Disp,2.0f/3.0f)/hull_coef.AW + a6 * hull_coef.LCB/hull_coef.LCF + a7 * pow(hull_coef.LCB/hull_coef.LWL,2.0f) + a8 * pow(hull_coef.Cp,2.0f);

    const float rhs = a0 + (term_1 + term_2) * (pow(hull_coef.Disp,1.0f/3.0f) / hull_coef.LWL);
    const float Rrh = rhs * hull_coef.Disp * hull_coef.rho * GRAVITY_MSS;

    // Aditonal residuary resistance due to heel angle //
    // Interpolate coefficents based on froude number
    const float u0 = -0.0268f; // test numbers // Coefficient_DYS_heel
    const float u1 = -0.0014f;
    const float u2 = -0.0057f;
    const float u3 =  0.0016f;
    const float u4 = -0.007f;
    const float u5 = -0.0017f;

    const float rhs_1 = u0 + u1 * (hull_coef.LWL / hull_coef.BWL) + u2 * (hull_coef.BWL/hull_coef.Tc) + u3 * pow(hull_coef.BWL/hull_coef.Tc,2.0f) + u4 * hull_coef.LCB + u5 * pow(hull_coef.LCB,2.0f);
    const float Rrh_heel = rhs_1 * hull_coef.Disp * hull_coef.rho * GRAVITY_MSS * 6.0f * pow(abs(heel),1.7f);

    // Viscous resistance  //
    const float Cf = 0.075f / pow( logf(Rn) - 2.0f,2.0f);
    const float Sc = (1.97f + 0.171f * (hull_coef.BWL/hull_coef.Tc)) * pow(0.65f/hull_coef.Cm,1.0f/3.0f) * pow(hull_coef.LWL * hull_coef.Disp,0.5f);
    float Rfh = 0.5f  * hull_coef.rho * Sc * Cf;

    // heel ratio
    const float s0 = -4.112f; // test numbers // visconse heel
    const float s1 =  0.054f;
    const float s2 = -0.027f;
    const float s3 =  6.329f;

    const float ratio = 1.0f + 0.01f * (s0 + s1 * (hull_coef.BWL/hull_coef.Tc) + s2 * pow(hull_coef.BWL/hull_coef.Tc,2.0f) + s3 * hull_coef.Cm);
    Rfh *= ratio;

    // Total
    float total_drag = Rrh + Rrh_heel + Rfh;

    if (!is_positive(total_drag)) {
        total_drag = 0.0f;
    }

    return total_drag;
}

/*
  update the sailboat simulation by one time step
 */
void Sailboat::update(const struct sitl_input &input)
{
    // update wind
    update_wind(input);

    // in sailboats the steering controls the rudder, the throttle controls the main sail position
    float steering = 2*((input.servos[STEERING_SERVO_CH]-1000)/1000.0f - 0.5f);

    // calculate mainsail angle from servo output 4, 0 to 90 degrees
    float mainsail_angle_bf = linear_interpolate(1000.0f, 2000.0f, input.servos[MAINSAIL_SERVO_CH], 0.0f, 90.f);

    // calculate apparent wind in earth-frame (this is the direction the wind is coming from)
    Vector3f wind_apparent_ef = wind_ef + velocity_ef;
    const float wind_apparent_dir_ef = wrap_180(180.0f + degrees(atan2f(wind_apparent_ef.y, wind_apparent_ef.x)));
    const float wind_apparent_speed = safe_sqrt(sq(wind_apparent_ef.x)+sq(wind_apparent_ef.y));

    // Aparent wind angle in body frame
    const float wind_apparent_dir_bf = wrap_180(wind_apparent_dir_ef - degrees(AP::ahrs().yaw));

    // calculate angle-of-attack from wind to mainsail
    float aoa_deg = MAX(fabsf(wind_apparent_dir_bf) - mainsail_angle_bf, 0);

    // calculate Lift force (perpendicular to wind direction) and Drag force (parallel to wind direction)
    float lift_wf, drag_wf;
    calc_lift_and_drag(wind_apparent_speed, aoa_deg, lift_wf, drag_wf);

    // rotate lift and drag from wind frame into body frame
    const float sin_rot_rad = sinf(radians(wind_apparent_dir_bf));
    const float cos_rot_rad = cosf(radians(wind_apparent_dir_bf));
    const float force_fwd = (lift_wf * sin_rot_rad) + (drag_wf * cos_rot_rad);
    const float heel_force = (lift_wf * cos_rot_rad) + (drag_wf * sin_rot_rad);

    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef;

    // speed along x axis, +ve is forward
    float speed = velocity_body.x;

    // caculate heel angle
    float heel = safe_asin((heel_force * sail_cp)/(keel_mass * keel_lenght));

    // yaw rate in degrees/s
    float yaw_rate = get_yaw_rate(steering, speed);

    gyro = Vector3f(0,0,radians(yaw_rate));

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // accel in body frame due acceleration from sail and deceleration from hull friction
    accel_body = Vector3f((force_fwd + calc_hull_drag(speed, heel))/mass, 0, 0);

    // add in accel due to direction change
    accel_body.y += radians(yaw_rate) * speed;

    // now in earth frame
    Vector3f accel_earth = dcm * accel_body;

    // we are on the ground, so our vertical accel is zero
    accel_earth.z = 0;

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    // new position vector
    position += velocity_ef * delta_time;

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

} // namespace SITL
