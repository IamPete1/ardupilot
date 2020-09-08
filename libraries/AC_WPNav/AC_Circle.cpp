#include <AP_HAL/AP_HAL.h>
#include "AC_Circle.h"
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_Circle::var_info[] = {
    // @Param: RADIUS
    // @DisplayName: Circle Radius
    // @Description: Defines the radius of the circle the vehicle will fly when in Circle flight mode
    // @Units: cm
    // @Range: 0 200000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("RADIUS",  0,  AC_Circle, _radius, AC_CIRCLE_RADIUS_DEFAULT),

    // @Param: RATE
    // @DisplayName: Circle rate
    // @Description: Circle mode's turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise
    // @Units: deg/s
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RATE",    1, AC_Circle, _rate,    AC_CIRCLE_RATE_DEFAULT),

    // @Param: OPTIONS
    // @DisplayName: Circle options
    // @Description: 0:Enable or disable using the pitch/roll stick control circle mode's radius and rate
    // @Bitmask: 0:manual control, 1:face direction of travel, 2:Start at center rather than on perimeter, 3:Constant speed independent of radius change
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 2, AC_Circle, _options, 1),

    // @Param: MAX_RAD
    // @DisplayName: maximum Circle Radius
    // @Description: Defines the maximum radius of the circle the vehicle will fly when in Circle flight mode, 0 for disabled
    // @Units: cm
    // @Range: 0 200000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("MAX_RAD",  3,  AC_Circle, _max_radius, 0),

    // @Param: RAD_INC
    // @DisplayName: Distance the radius will increse by per revolution
    // @Description: Distance the radius will increse by per revolution
    // @Units: cm
    // @Range: 0 200000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("RAD_INC",  4,  AC_Circle, _radius_step, 0),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_Circle::AC_Circle(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _yaw(0.0f),
    _angle(0.0f),
    _angle_total(0.0f),
    _angular_vel(0.0f),
    _angular_vel_max(0.0f),
    _angular_accel(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init flags
    _flags.panorama = false;
}

/// init - initialise circle controller setting center specifically
///     caller should set the position controller's x,y and z speeds and accelerations before calling this
void AC_Circle::init(const Vector3f& center)
{
    _center = center;

    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.set_desired_accel_xy(0.0f,0.0f);
    _pos_control.set_desired_velocity_xy(0.0f,0.0f);
    _pos_control.init_xy_controller();

    // set initial position target to reasonable stopping point
    _pos_control.set_target_to_stopping_point_xy();
    _pos_control.set_target_to_stopping_point_z();

    // calculate velocities
    calc_velocities(true);

    // set start angle from position
    init_start_angle(false);
}

/// init - initialise circle controller setting center using stopping point and projecting out based on the copter's heading
///     caller should set the position controller's x,y and z speeds and accelerations before calling this
void AC_Circle::init()
{
    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.set_desired_accel_xy(0.0f,0.0f);
    _pos_control.set_desired_velocity_xy(0.0f,0.0f);
    _pos_control.init_xy_controller();

    // set initial position target to reasonable stopping point
    _pos_control.set_target_to_stopping_point_xy();
    _pos_control.set_target_to_stopping_point_z();

    // get stopping point
    const Vector3f& stopping_point = _pos_control.get_pos_target();

    // set circle center to circle_radius ahead of stopping point
    _center = stopping_point;
    if ((_options.get() & CircleOptions::INIT_AT_CENTER) == 0) {
        _center.x += _radius * _ahrs.cos_yaw();
        _center.y += _radius * _ahrs.sin_yaw();
    }

    // set radius and rate to param values
    _cur_radius = _radius;
    _cur_rate = _rate;

    // calculate velocities
    calc_velocities(true);

    // set starting angle from vehicle heading
    init_start_angle(true);
}

/// set_circle_rate - set circle rate in degrees per second
void AC_Circle::set_rate(float deg_per_sec)
{
    _cur_rate = deg_per_sec;
}

/// set_circle_rate - set circle rate in degrees per second
void AC_Circle::set_radius(float radius_cm)
{

    float max_rad = is_zero(_max_radius)?AC_CIRCLE_RADIUS_MAX:_max_radius;
    float new_rad = constrain_float(radius_cm, 0, MIN(AC_CIRCLE_RADIUS_MAX,max_rad));

    if (((_options.get() & CircleOptions::CONSTANT_SPEED) != 0) && !is_equal(_cur_radius, new_rad) && !is_zero(_cur_radius) && !is_zero(new_rad)) {
        // correct the current rate the change in radius to give a constant ground speed, ratio of curumfrneces
        _cur_rate *= _cur_radius / new_rad;
    }
    _cur_radius = new_rad;

}

/// update - update circle controller
void AC_Circle::update()
{
    calc_velocities(false);

    // calculate dt
    float dt = _pos_control.time_since_last_xy_update();
    if (dt >= 0.2f) {
        dt = 0.0f;
    }

    // ramp angular velocity to maximum
    if (_angular_vel < _angular_vel_max) {
        _angular_vel += fabsf(_angular_accel) * dt;
        _angular_vel = MIN(_angular_vel, _angular_vel_max);
    }
    if (_angular_vel > _angular_vel_max) {
        _angular_vel -= fabsf(_angular_accel) * dt;
        _angular_vel = MAX(_angular_vel, _angular_vel_max);
    }

    // update the target angle and total angle traveled
    float angle_change = _angular_vel * dt;
    _angle += angle_change;
    _angle = wrap_PI(_angle);
    _angle_total += angle_change;

    // if the circle_radius is zero we are doing panorama so no need to update loiter target
    if (!is_zero(_cur_radius)) {
        // calculate target position
        Vector3f target;
        target.x = _center.x + _cur_radius * cosf(-_angle);
        target.y = _center.y - _cur_radius * sinf(-_angle);

        // update position controller target
        _pos_control.set_xy_target(target.x, target.y);

        // heading is from vehicle to center of circle
        _yaw = get_bearing_cd(_inav.get_position(), _center);

        if ((_options.get() & CircleOptions::FACE_DIRECTION_OF_TRAVEL) != 0) {
            _yaw += is_positive(_cur_rate)?-9000.0f:9000.0f;
            _yaw = wrap_360_cd(_yaw);
        }

    } else {
        // set target position to center
        Vector3f target;
        target.x = _center.x;
        target.y = _center.y;
        target.z = _pos_control.get_alt_target();

        // update position controller target
        _pos_control.set_xy_target(target.x, target.y);

        // heading is same as _angle but converted to centi-degrees
        _yaw = _angle * DEGX100;
    }

    // update position controller
    _pos_control.update_xy_controller();
}

// get_closest_point_on_circle - returns closest point on the circle
//  circle's center should already have been set
//  closest point on the circle will be placed in result
//  result's altitude (i.e. z) will be set to the circle_center's altitude
//  if vehicle is at the center of the circle, the edge directly behind vehicle will be returned
void AC_Circle::get_closest_point_on_circle(Vector3f &result) const
{
    // return center if radius is zero
    if (_cur_radius <= 0) {
        result = _center;
        return;
    }

    // get current position
    Vector3f stopping_point;
    _pos_control.get_stopping_point_xy(stopping_point);

    // calc vector from stopping point to circle center
    Vector2f vec;   // vector from circle center to current location
    vec.x = (stopping_point.x - _center.x);
    vec.y = (stopping_point.y - _center.y);
    float dist = norm(vec.x, vec.y);

    // if current location is exactly at the center of the circle return edge directly behind vehicle
    if (is_zero(dist)) {
        result.x = _center.x - _cur_radius * _ahrs.cos_yaw();
        result.y = _center.y - _cur_radius * _ahrs.sin_yaw();
        result.z = _center.z;
        return;
    }

    // calculate closest point on edge of circle
    result.x = _center.x + vec.x / dist * _cur_radius;
    result.y = _center.y + vec.y / dist * _cur_radius;
    result.z = _center.z;
}

// calc_velocities - calculate angular velocity max and acceleration based on radius and rate
//      this should be called whenever the radius or rate are changed
//      initialises the yaw and current position around the circle
void AC_Circle::calc_velocities(bool init_velocity)
{
    // if we are doing a panorama set the circle_angle to the current heading
    if (_cur_radius <= 0) {
        _angular_vel_max = ToRad(_cur_rate);
        _angular_accel = MAX(fabsf(_angular_vel_max),ToRad(AC_CIRCLE_ANGULAR_ACCEL_MIN));  // reach maximum yaw velocity in 1 second
    }else{
        // calculate max velocity based on waypoint speed ensuring we do not use more than half our max acceleration for accelerating towards the center of the circle
        float velocity_max = MIN(_pos_control.get_max_speed_xy(), safe_sqrt(0.5f*_pos_control.get_max_accel_xy()*_cur_radius));

        // angular_velocity in radians per second
        _angular_vel_max = velocity_max/_cur_radius;
        _angular_vel_max = constrain_float(ToRad(_cur_rate),-_angular_vel_max,_angular_vel_max);

        // angular_velocity in radians per second
        _angular_accel = MAX(_pos_control.get_max_accel_xy()/_cur_radius, ToRad(AC_CIRCLE_ANGULAR_ACCEL_MIN));
    }

    // initialise angular velocity
    if (init_velocity) {
        _angular_vel = 0;
    }
}

// init_start_angle - sets the starting angle around the circle and initialises the angle_total
//  if use_heading is true the vehicle's heading will be used to init the angle causing minimum yaw movement
//  if use_heading is false the vehicle's position from the center will be used to initialise the angle
void AC_Circle::init_start_angle(bool use_heading)
{
    // initialise angle total
    _angle_total = 0;

    // if the radius is zero we are doing panorama so init angle to the current heading
    if (_cur_radius <= 0) {
        _angle = _ahrs.yaw;
        return;
    }

    // if use_heading is true
    if (use_heading) {
        _angle = wrap_PI(_ahrs.yaw-M_PI);
    } else {
        // if we are exactly at the center of the circle, init angle to directly behind vehicle (so vehicle will backup but not change heading)
        const Vector3f &curr_pos = _inav.get_position();
        if (is_equal(curr_pos.x,_center.x) && is_equal(curr_pos.y,_center.y)) {
            _angle = wrap_PI(_ahrs.yaw-M_PI);
        } else {
            // get bearing from circle center to vehicle in radians
            float bearing_rad = atan2f(curr_pos.y-_center.y,curr_pos.x-_center.x);
            _angle = wrap_PI(bearing_rad);
        }
    }
}
