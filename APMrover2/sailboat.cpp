#include "Rover.h"

#define SAILBOAT_AUTO_TACKING_TIMEOUT_MS 50000  // tacks in auto mode timeout if not successfully completed within this many milliseconds
#define SAILBOAT_TACKING_ACCURACY_DEG 10        // tack is considered complete when vehicle is within this many degrees of target tack angle
/*
To Do List
 - Improve tacking in light winds and bearing away in strong wings
 - consider drag vs lift sailing differences, ie upwind sail is like wing, dead down wind sail is like parachute
 - max speed paramiter and controller, for mapping you may not want to go too fast
 - mavlink sailing messages
 - motor sailing, some boats may also have motor, we need to decide at what point we would be better of just motoring in low wind, or for a tight loiter, or to hit waypoint exactly, or if stuck head to wind, or to reverse...
 - smart decision making, ie tack on windshifts, what to do if stuck head to wind
 - some sailing codes track waves to try and 'surf' and to allow tacking on a flat bit, not sure if there is much gain to be had here
 - add some sort of pitch monitoring to prevent nose diving in heavy weather
 - pitch PID for hydrofoils
 - more advanced sail control, ie twist
 - independent sheeting for main and jib
 - wing type sails with 'elevator' control
 - tack on depth sounder info to stop sailing into shallow water on indirect sailing routes
 - add option to do proper tacks, ie tacking on flat spot in the waves, or only try once at a certain speed, or some better method than just changing the desired heading suddenly
*/

const AP_Param::GroupInfo Sailboat::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Sailboat
    // @Description: This enables Sailboat functionality
    // @Values: 0:Disable,1:Enable sail assist only,2:Enable,3:Enable with speed controller
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, Sailboat, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ANGLE_MIN
    // @DisplayName: Sail min angle
    // @Description: Mainsheet tight, angle between centerline and boom
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_MIN", 2, Sailboat, sail_angle_min, 0),

    // @Param: ANGLE_MAX
    // @DisplayName: Sail max angle
    // @Description: Mainsheet loose, angle between centerline and boom
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_MAX", 3, Sailboat, sail_angle_max, 90),

    // @Param: ANGLE_IDEAL
    // @DisplayName: Sail ideal angle
    // @Description: Ideal angle between sail and apparent wind
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_IDEAL", 4, Sailboat, sail_angle_ideal, 25),

    // @Param: HEEL_MAX
    // @DisplayName: Sailing maximum heel angle
    // @Description: When in auto sail trim modes the heel will be limited to this value using PID control
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HEEL_MAX", 5, Sailboat, sail_heel_angle_max, 15),

    // @Param: SAIL_NO_GO_ANGLE
    // @DisplayName: Sailing no go zone angle
    // @Description: The typical closest angle to the wind the vehicle will sail at. the vehicle will sail at this angle when going upwind
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("NO_GO_ANGLE", 6, Sailboat, sail_no_go, 45),

    // @Param: SAIL_MAX_X_TRACK
    // @DisplayName: Sailing vehicle max cross track error
    // @Description: The sail boat will tack when it reaches this cross track error, defines a corridor of 2 times this value wide, 0 disables
    // @Units: m
    // @Range: 5 25
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAX_X_TRACK", 7, Sailboat, max_cross_track, 10),

    // @Param: SAIL_LOIT_RADIUS
    // @DisplayName: Loiter radius
    // @Description: Vehicle will sail when within this distance of the target position
    // @Units: m
    // @Range: 0 20
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LOIT_RADIUS", 8, Sailboat, loit_radius, 5),

    // @Param: SAIL_OPTIONS
    // @DisplayName: Sailboat options bitmask
    // @Description: Sailboat options bitmask
    // @Values: 0:None,1:Aux Sail,2:Aux Throttle Limit
    // @Bitmask: 0:Aux throttle main for sail,1:Aux Throttle Limit
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 9, Sailboat, sail_options, 0),

    // @Param: SAIL_MIN_WNDSPD
    // @DisplayName: Minimum wind speed to sail in
    // @Description: Sailboat minimum wind speed to continue sail in, at lower wind speeds the sailboat will motor if one is fitted
    // @Units: m/s
    // @Range: 0 5
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MIN_WNDSPD", 10, Sailboat, sail_assist_windspeed, 0),

    AP_GROUPEND
};


/*
  constructor
 */
Sailboat::Sailboat()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void Sailboat::init()
{
    // sailboat defaults
    if (sail_enabled()) {
        rover.g2.crash_angle.set_default(0);
    }

    if (nav_enabled()) {
        rover.g2.loit_type.set_default(1);
    }

    // if we have a throttle of some sort allow to use it
    if (rover.g2.motors.have_skid_steering() || SRV_Channels::function_assigned(SRV_Channel::k_throttle)) {
        throttle_state = sailboat_throttle::ASSIST;
    }
}

// update mainsail's desired angle based on wind speed and direction and desired speed (in m/s)
float Sailboat::update_sail_control(float desired_speed, float throttle_out)
{
    if (!sail_enabled()) {
        return throttle_out;
    }

    // relax sail if desired speed is zero, or motoring
    if (!is_positive(desired_speed) || throttle_state == sailboat_throttle::FORCE_MOTOR) {
        rover.g2.motors.set_mainsail(100.0f);
        return throttle_out;
    }

    // + is wind over starboard side, - is wind over port side, but as the sails are sheeted the same on each side it makes no difference so take abs
    float wind_dir_apparent = fabsf(rover.g2.windvane.get_apparent_wind_direction_rad());
    wind_dir_apparent = degrees(wind_dir_apparent);

    // set the main sail to the ideal angle to the wind
    float mainsail_angle = wind_dir_apparent -sail_angle_ideal;

    // make sure between allowable range
    mainsail_angle = constrain_float(mainsail_angle,sail_angle_min, sail_angle_max);

    // linear interpolate mainsail value (0 to 100) from wind angle mainsail_angle
    float mainsail = linear_interpolate(0.0f, 100.0f, mainsail_angle,sail_angle_min,sail_angle_max);

    // use PID controller to sheet out
    const float pid_offset = rover.g2.attitude_control.get_sail_out_from_heel(radians(sail_heel_angle_max), rover.G_Dt) * 100.0f;

    // See if we should sheet out to slow down
    float speed_offset = 0.0f;
    if (!is_positive(throttle_out) && enable == 3) {
        speed_offset = throttle_out;
        // never try to reverse against the sail
        throttle_out = 0.0f;
    }

    mainsail = constrain_float((mainsail + pid_offset + speed_offset), 0.0f ,100.0f);
    rover.g2.motors.set_mainsail(mainsail);

    // see if we should allow throttle
    if (throttle_state == sailboat_throttle::NEVER ||
       (throttle_state == sailboat_throttle::ASSIST && !throttle_assist())) {
        throttle_out = 0.0f;
        if (enable == 2) {
            rover.g2.attitude_control.relax_throttle();
        }
    }

    // see if we should limit the throttle allowed
    if ( (sail_options & options::AUX_THROTTLE) != 0 && (sail_options & options::AUX_THROTTLE_LIMIT) != 0 ) {
        RC_Channel *rc_ptr = rc().find_channel_for_option(RC_Channel::AUX_FUNC::SAILBOAT_THR);
        if (rc_ptr != nullptr) {
            rc_ptr->set_angle(100);
            rc_ptr->set_default_dead_zone(30);
            float aux_thr = rc_ptr->get_control_in();
            if (is_positive(throttle_out)) {
                aux_thr = MAX(aux_thr, 0.0f);
                throttle_out = MIN(throttle_out, aux_thr);
            } else {
                aux_thr = MIN(aux_thr, 0.0f);
                throttle_out = MAX(throttle_out, aux_thr);
            }
        }
    }

    return throttle_out;
}

// Velocity Made Good, this is the speed we are traveling towards the desired destination
// only for logging at this stage
// https://en.wikipedia.org/wiki/Velocity_made_good
float Sailboat::get_VMG() const
{
    // return zero if we don't have a valid speed
    float speed;
    if (!rover.g2.attitude_control.get_forward_speed(speed)) {
        return 0.0f;
    }
    
    // return speed if not heading towards waypoint
    if (!rover.control_mode->is_autopilot_mode()) {
        return speed;
    }

    return (speed * cosf(wrap_PI(radians(rover.g2.wp_nav.wp_bearing_cd() * 0.01f) - rover.ahrs.yaw)));
}

// handle user initiated tack while in acro mode
void Sailboat::handle_tack_request_acro()
{
    if (!nav_enabled()) {
        return;
    }
    // set tacking heading target to the current angle relative to the true wind but on the new tack
    currently_tacking = true;
    tack_heading_rad = wrap_2PI(rover.ahrs.yaw + 2.0f * wrap_PI((rover.g2.windvane.get_absolute_wind_direction_rad() - rover.ahrs.yaw)));
}

// return target heading in radians when tacking (only used in acro)
float Sailboat::get_tack_heading_rad() const
{
    return tack_heading_rad;
}

// handle user initiated tack while in autonomous modes (Auto, Guided, RTL, SmartRTL, etc)
void Sailboat::handle_tack_request_auto()
{
    // record time of request for tack.  This will be processed asynchronously by sailboat_calc_heading
    auto_tack_request_ms = AP_HAL::millis();
}

// clear tacking state variables
void Sailboat::clear_tack()
{
    tack_assist = false;
    currently_tacking = false;
    auto_tack_request_ms = 0;
}

// returns true if boat is currently tacking
bool Sailboat::tacking()
{
    return nav_enabled() && currently_tacking;
}

// returns true if sailboat should take a indirect navigation route to go upwind
// desired_heading should be in centi-degrees
bool Sailboat::use_indirect_route(float desired_heading_cd)
{
    if (!nav_enabled()) {
        return false;
    }

    // convert desired heading to radians
    const float desired_heading_rad = radians(desired_heading_cd * 0.01f);

    // check if desired heading is in the no go zone, if it is we can't go direct
    return fabsf(wrap_PI(rover.g2.windvane.get_absolute_wind_direction_rad() - desired_heading_rad)) <= radians(sail_no_go);
}

// if we can't sail on the desired heading then we should pick the best heading that we can sail on
// this function assumes the caller has already checked sailboat_use_indirect_route(desired_heading_cd) returned true
float Sailboat::calc_heading(float desired_heading_cd)
{
    // have we come here by mistake?
    if (!nav_enabled()) {
        return desired_heading_cd;
    }

    bool should_tack = false;

    // check for user requested tack
    uint32_t now = AP_HAL::millis();
    if (auto_tack_request_ms != 0) {
        // set should_tack flag is user requested tack within last 0.5 sec
        should_tack = ((now - auto_tack_request_ms) < 500);
        auto_tack_request_ms = 0;
    }

    // calculate left and right no go headings looking upwind
    const float left_no_go_heading_rad = wrap_2PI(rover.g2.windvane.get_absolute_wind_direction_rad() + radians(sail_no_go));
    const float right_no_go_heading_rad = wrap_2PI(rover.g2.windvane.get_absolute_wind_direction_rad() - radians(sail_no_go));

    // calculate current tack, Port if heading is left of no-go, STBD if right of no-go
    Sailboat_Tack current_tack;
    if (is_negative(rover.g2.windvane.get_apparent_wind_direction_rad())) {
        current_tack = TACK_PORT;
    } else {
        current_tack = TACK_STARBOARD;
    }

    // trigger tack if cross track error larger than waypoint_overshoot parameter
    // this effectively defines a 'corridor' of width 2*waypoint_overshoot that the boat will stay within
    const float cross_track_error = rover.g2.wp_nav.crosstrack_error();
    if ((fabsf(cross_track_error) >= max_cross_track) && !is_zero(max_cross_track) && !currently_tacking) {
        // make sure the new tack will reduce the cross track error
        // if were on starboard tack we are traveling towards the left hand boundary
        if (is_positive(cross_track_error) && (current_tack == TACK_STARBOARD)) {
            should_tack = true;
        }
        // if were on port tack we are traveling towards the right hand boundary
        if (is_negative(cross_track_error) && (current_tack == TACK_PORT)) {
            should_tack = true;
        }
    }

    // if tack triggered, calculate target heading
    if (should_tack) {
        gcs().send_text(MAV_SEVERITY_INFO, "Sailboat: Tacking");
        // calculate target heading for the new tack
        switch (current_tack) {
            case TACK_PORT:
                tack_heading_rad = right_no_go_heading_rad;
                break;
            case TACK_STARBOARD:
                tack_heading_rad = left_no_go_heading_rad;
                break;
        }
        currently_tacking = true;
        auto_tack_start_ms = AP_HAL::millis();
    }

    // if we are tacking we maintain the target heading until the tack completes or times out
    if (currently_tacking) {
        // check if we have reached target
        if (fabsf(wrap_PI(tack_heading_rad - rover.ahrs.yaw)) <= radians(SAILBOAT_TACKING_ACCURACY_DEG)) {
            clear_tack();
        } else if ((now - auto_tack_start_ms) > SAILBOAT_AUTO_TACKING_TIMEOUT_MS) {
            // tack has taken too long
            if (throttle_state == sailboat_throttle::ASSIST && (now - auto_tack_start_ms) < (2.0f * SAILBOAT_AUTO_TACKING_TIMEOUT_MS)) {
                // if we have throttle available use it for another time period to get the tack done
                tack_assist = true;
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Sailboat: Tacking timed out");
                clear_tack();
            }
        }
        // return tack target heading
        return degrees(tack_heading_rad) * 100.0f;
    }

    // return closest possible heading to wind
    if (current_tack == TACK_PORT) {
        return degrees(left_no_go_heading_rad) * 100.0f;
    } else {
        return degrees(right_no_go_heading_rad) * 100.0f;
    }
}

// update manual mode throttle, used for dual throttle inputs, one for sail one for motor
float Sailboat::update_manual_throttle(float desired_throttle)
{
    if (!sail_enabled()) {
        return desired_throttle;
    }

    // get auxiliary throttle value
    RC_Channel *rc_ptr = rc().find_channel_for_option(RC_Channel::AUX_FUNC::SAILBOAT_THR);
    float aux_thr = 0.0f;
    if (rc_ptr != nullptr) {
        rc_ptr->set_angle(100);
        rc_ptr->set_default_dead_zone(30);
        aux_thr = rc_ptr->get_control_in();
    } else {
        // aux throttle not set up
        rover.g2.motors.set_mainsail(desired_throttle);
        return desired_throttle;
    }

    // set throttle or sail
    const bool sail_throttle = (sail_options & options::AUX_THROTTLE) != 0;
    if (sail_throttle) {
        rover.g2.motors.set_mainsail(desired_throttle);
        if (!(rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE)) {
            desired_throttle = aux_thr;
        }
    } else {
        rover.g2.motors.set_mainsail(aux_thr);
    }

    return desired_throttle;
}

// check dual throttles for arming
bool Sailboat::aux_throttle_pre_arm_check()
{
    if (!sail_enabled()) {
        return false;
    }

    const bool sail_throttle = (sail_options & options::AUX_THROTTLE) != 0;
    if (sail_throttle) {
        // get auxiliary throttle value
        RC_Channel *rc_ptr = rc().find_channel_for_option(RC_Channel::AUX_FUNC::SAILBOAT_THR);
        float aux_thr = 0.0f;
        if (rc_ptr != nullptr) {
            rc_ptr->set_angle(100);
            rc_ptr->set_default_dead_zone(30);
            aux_thr = rc_ptr->get_control_in();
        }
        if (!is_zero(aux_thr)) {
            return true;
        }
    }

    return false;
}

// should we use the throttle?
bool Sailboat::throttle_assist() {
    if (throttle_state == sailboat_throttle::NEVER) {
        return false;
    }

    if ((!is_zero(sail_assist_windspeed) &&
        rover.g2.windvane.wind_speed_enabled() &&
        rover.g2.windvane.get_true_wind_speed() < sail_assist_windspeed) ||
        tack_assist) {
        return true;
    }

    return false;
}

// Should we use sailboat navigation?
bool Sailboat::nav_enabled()
{
    return enable >= 2 &&
           (throttle_state != sailboat_throttle::FORCE_MOTOR) &&
           (!throttle_assist() && !tack_assist);
}

