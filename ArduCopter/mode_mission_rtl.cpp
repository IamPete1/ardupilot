#include "Copter.h"

#if MODE_RTL_ENABLED == ENABLED && MODE_AUTO_ENABLED == ENABLED

/*
 * Init and run calls for RTL flight mode
 *
 * There are two parts to RTL, the high level decision making which controls which state we are in
 * and the lower implementation of the waypoint or landing controllers within those states
 */

// rtl_init - initialise rtl controller
bool Mission_RTL::init(bool ignore_checks)
{
    const bool tmp_switching_from_RTL = switching_from_RTL;
    switching_from_RTL = false;

    if ((((Type)g2.mission_RTL_type.get() == Type::CLOSEST_LANDING_SEQUENCE)  && copter.mode_auto.mission.jump_to_landing_sequence()) ||
        (((Type)g2.mission_RTL_type.get() == Type::SHORTEST_LANDING_SEQUENCE) && copter.mode_auto.mission.jump_to_shortest_landing_sequence()) ||
        (((Type)g2.mission_RTL_type.get() == Type::CLOSEST_MISSION_LEG)       && copter.mode_auto.mission.jump_to_closest_mission_leg()) ||
        (((Type)g2.mission_RTL_type.get() == Type::SHORTEST_MISSION_LEG)      && copter.mode_auto.mission.jump_to_shortest_mission_leg()) ) {
            copter.mode_auto.mission.set_force_resume(true);
            return copter.mode_auto.init(ignore_checks);
    }

    // Completed flight to home in RTL, then start landing sequence
    if (copter.flightmode->mode_number() == Mode::Number::RTL) {
        if (tmp_switching_from_RTL && copter.mode_auto.mission.jump_to_landing_sequence()) {
            copter.mode_auto.mission.set_force_resume(true);
            return copter.mode_auto.init(ignore_checks);
        }
        // no need to set mode back to RTL if comming from it, just refuse change
        gcs().send_text(MAV_SEVERITY_WARNING, "MRTL mission not configured correclty");
        return false;
    }

    bool ret = false;
    ModeReason rsn = ModeReason::MISSION_RTL_FAILED;

    if ((Type)g2.mission_RTL_type.get() == Type::HOME_THEN_LANDING_SEQUENCE) {
        gcs().send_text(MAV_SEVERITY_INFO, "RTL then DO_LAND_START");
        rsn = ModeReason::MISSION_RTL_HOME_FIRST;
        ret = true;
    }

    copter.set_mode(Mode::Number::RTL, rsn);

    if ((Type)g2.mission_RTL_type.get() == Type::NONE) {
        gcs().send_text(MAV_SEVERITY_WARNING, "MRTL not enabled");
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "MRTL mission not configured correclty");
    }

    return ret;
}

void Mission_RTL::run()
{
    copter.mode_auto.run();
}

void Mission_RTL::exit()
{
    copter.mode_auto.exit();
}

bool Mission_RTL::requires_GPS() const
{
    return copter.mode_auto.requires_GPS();
}

bool Mission_RTL::has_manual_throttle() const
{
    return copter.mode_auto.has_manual_throttle();
}

bool Mission_RTL::allows_arming(AP_Arming::Method method) const
{
    return copter.mode_rtl.allows_arming(method);
}

bool Mission_RTL::is_autopilot() const
{
    return copter.mode_auto.is_autopilot();
}

bool Mission_RTL::in_guided_mode() const
{
    return copter.mode_auto.in_guided_mode();
}

bool Mission_RTL::requires_terrain_failsafe() const
{
    return copter.mode_auto.requires_terrain_failsafe();
}

bool Mission_RTL::has_user_takeoff(bool must_navigate) const
{
    return copter.mode_auto.has_user_takeoff(must_navigate);
}

uint32_t Mission_RTL::wp_distance() const
{
    return copter.mode_auto.wp_distance();
}

int32_t Mission_RTL::wp_bearing() const
{
    return copter.mode_auto.wp_bearing();
}

float Mission_RTL::crosstrack_error() const
{
    return copter.mode_auto.crosstrack_error();
}

bool Mission_RTL::get_wp(Location &loc) const
{
    return copter.mode_auto.get_wp(loc);
}

bool Mission_RTL::is_taking_off() const
{
    return copter.mode_auto.is_taking_off();
}

bool Mission_RTL::is_landing() const
{
    return copter.mode_auto.is_landing();
}

#endif