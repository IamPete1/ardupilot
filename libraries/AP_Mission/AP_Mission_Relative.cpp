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
 * AP_Mission_Relative.cpp
 *
 *  Created on: 08.05.2020
 *      Author: WillyZehnder
 */
/// @file    AP_Mission_Relative.cpp
/// @brief   translates and rotates Missions according to the location where Mode-AUTO has been switched on

/*
 *   The AP_Mission_Relative library:
 *   - memorizes the Location where Mode-AUTO has been switched on (Basepoint)
 *   - moves the individual Waypoints according to Homepoint, Base-Point and Parameter-Settings
 */

#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_Mission_Relative.h"

//extern const AP_HAL::HAL& hal; ### ?

AP_Mission_Relative *AP_Mission_Relative::_singleton;

const AP_Param::GroupInfo AP_Mission_Relative::var_info[] = {

    // @Param: MOVE
    // @DisplayName: kind of moving the mission
    // @Description: Defines how the mission will be translated/rotated
    // @Range: 0 2
    // @Values: 0:No move, 1:PARALLEL translation, 2:additional ROTATION
    // @User: Advanced
    AP_GROUPINFO_FLAGS("MOVE",  1, AP_Mission_Relative, _kind_of_move, AP_MISSION_RELATIVE_KIND_OF_MOVE_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param: RADIUS
    // @DisplayName: radius to ignore translation
    // @Description: Radius around HomeLocation therein translation of Relative Mission will be ignored
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RADIUS",  2, AP_Mission_Relative, _no_translation_radius, AP_MISSION_RELATIVE_NO_TRANSLATION_RADIUS_DEFAULT),

    // @Param: OPTIONS
    // @DisplayName: selectable options for translation
    // @Description: Bitmask of what options to use in missions.
    // @Bitmask: 0: At restart skip altitude at first Waypoint, 1: At restart use altitude offset (Basepoint to first Waypoint) for all Waypoints
    // @User: Advanced
    AP_GROUPINFO("OPTIONS",  3, AP_Mission_Relative, _rel_options, AP_MISSION_RELATIVE_OPTIONS_DEFAULT),

    AP_GROUPEND
};

// constructor
AP_Mission_Relative::AP_Mission_Relative(void)
{
    AP_Param::setup_object_defaults(this, var_info); // load parameter defaults
/* ### ?
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Mission_Relative must be singleton");
    }
*/
    _singleton = this;

    restart_behaviour = Restart_Behaviour::RESTART_NOT_TRANSLATED;
};

void AP_Mission_Relative::memorize()
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "AP_Mission_relative::memorize");

    gcs().send_text(MAV_SEVERITY_DEBUG, "_no_translation_radius : %f", (float)_no_translation_radius);
    gcs().send_text(MAV_SEVERITY_DEBUG, "_kind_of_move : %i", (int)_kind_of_move);
    gcs().send_text(MAV_SEVERITY_DEBUG, "_rel_options : %i", (int)_rel_options);

    switch (_kind_of_move) { // fix the behaviour at restart of the Mission
    case 0:
        restart_behaviour = Restart_Behaviour::RESTART_NOT_TRANSLATED;
        break;
    case 1:
        restart_behaviour = Restart_Behaviour::RESTART_PARALLEL_TRANSLATED;
        break;
    case 2:
        restart_behaviour = Restart_Behaviour::RESTART_ROTATED_TRANSLATED;
        break;
    default:
        gcs().send_text(MAV_SEVERITY_NOTICE, "MIS__REL_MOVE out of range: %i", (int)_kind_of_move);
        restart_behaviour = Restart_Behaviour::RESTART_NOT_TRANSLATED;
    }

    if (restart_behaviour == Restart_Behaviour::RESTART_NOT_TRANSLATED) {
        return;
    }
    else
    { // memorize the Basepoint (location of switching to AUTO)
        _translation.calculated = false;

        // determine the distance in [m] between Homepoint and Basepoint, to decide if a translation will happen
        AP::ahrs().get_position(_basepoint_loc); // _basepoint_loc.alt is absolute in [cm] in every case
        AP_Mission::Mission_Command tmp;
        tmp.content.location = AP::ahrs().get_home();
        float tmp_distance = tmp.content.location.get_distance(_basepoint_loc); // in [m]
        gcs().send_text(MAV_SEVERITY_DEBUG, "Mission_Relative: tmp_distance: %f", tmp_distance);

        if (tmp_distance < _no_translation_radius) { // no translation if the distance to Homepoint is too small
            _translation.do_translation = false;
            return;
        }
        else {
            _translation.do_translation = true;
        }
        _translation.alt = tmp.content.location.alt; // altitude of Homepoint, necessary for calculation of altitude-adaption at Restart

        // check if one of the Waypoints has (terrain_alt == 1) -> no translation allowed
        uint16_t i=0;
        while (AP::mission()->get_next_nav_cmd(i, tmp) && (_translation.do_translation) && (tmp.id != MAV_CMD_DO_LAND_START)) { // advance to the next command
            // check if Waypoint
            if (!(tmp.content.location.lat == 0 && tmp.content.location.lng == 0)) {
                switch (tmp.id) { // no translation for TAKEOFF or LAND commands
                    case MAV_CMD_NAV_TAKEOFF:
                    case MAV_CMD_NAV_LAND:
                    case MAV_CMD_NAV_VTOL_TAKEOFF:
                    case MAV_CMD_NAV_VTOL_LAND:
                        break;
                    default:
                        if (tmp.content.location.terrain_alt == 1) {
                            _translation.do_translation = false;
                            gcs().send_text(MAV_SEVERITY_NOTICE, "Mission_Relative: terrain_alt -> No move possible");
                        }
                }
            }
            i++;// move on to next command
        }
    }
}

void AP_Mission_Relative::set_no_translation()
{
    _translation.do_translation = false;
}

void AP_Mission_Relative::moveloc(Location& loc, uint16_t id)
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "moveloc id:%i lat:%i lng:%i", id, loc.lat, loc.lng);

    //  calculate and do translation/rotation
    if (_translation.do_translation) { // no translation if generally off by parameter or if we are behind DO_LAND_START
        switch (id) { // no translation for TAKEOFF or LAND commands
            case MAV_CMD_NAV_TAKEOFF:
            case MAV_CMD_NAV_LAND:
            case MAV_CMD_NAV_VTOL_TAKEOFF:
            case MAV_CMD_NAV_VTOL_LAND:
                break;
            default:
                Location tmp;
                // calculate parallel translation from first Waypoint to Basepoint
                if ((!_translation.calculated)&&(restart_behaviour >= Restart_Behaviour::RESTART_PARALLEL_TRANSLATED)) {
                    if (AP_MISSION_RELATIVE_MASK_SKIP_FIRST_WP & _rel_options) {
                        loc.alt = _translation.alt;
                    }
                    _translation.calculated = true;
                    _first_wp_loc = loc; // memorize position of very first WayPoint

                    // calculate altitude-translation
                    if (loc.relative_alt == 1) {
                        _translation.alt = _basepoint_loc.alt - loc.alt - _translation.alt; // subtract Home-Altitude if WP-altitude is relative to
                    } else {
                        _translation.alt = _basepoint_loc.alt - loc.alt;
                    }

                    // get direction from Homepoint to Basepoint
                    tmp = AP::ahrs().get_home();
                    _translation.direction = tmp.get_bearing_to(_basepoint_loc);   // in centidegrees from 0 36000

                    // direction from Homepoint to Basepoint / amount of rotation (center is the Basepoint)
                    _translation.direction -= tmp.get_bearing_to(loc);
                    if (_translation.direction < 0) {
                        _translation.direction += 36000;
                    }

                    loc = _basepoint_loc;
                    switch (restart_behaviour) {
                    case Restart_Behaviour::RESTART_PARALLEL_TRANSLATED:
                        gcs().send_text(MAV_SEVERITY_INFO, "Restart Parallel Translated");
                        break;
                    case Restart_Behaviour::RESTART_ROTATED_TRANSLATED:
                        gcs().send_text(MAV_SEVERITY_INFO, "Restart Rotated");
                        break;
                    default:
                        break;
                    }
                    gcs().send_text(MAV_SEVERITY_INFO, "Restart alt-transmission %.2fm",  static_cast<float>(_translation.alt)/100.0);
                }
                else {
                    if (restart_behaviour >= Restart_Behaviour::RESTART_PARALLEL_TRANSLATED){ // do at least parallel translation
                        AP_Mission_Relative::translate(loc);
                    }
                    if (restart_behaviour >= Restart_Behaviour::RESTART_ROTATED_TRANSLATED){ // do additional rotation
                        AP_Mission_Relative::rotate(loc);
                    }
                }
        }
    }
}

void AP_Mission_Relative::translate(Location& loc)
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "AP_Mission_relative::translate");
    Location tmp;
    tmp = loc; // before translation
    loc.lat = _basepoint_loc.lat + (loc.lat - _first_wp_loc.lat);
    // correction of lng based on lat-translation
    loc.lng = _basepoint_loc.lng + (loc.lng - _first_wp_loc.lng) / tmp.longitude_scale() * loc.longitude_scale();

    if (AP_MISSION_RELATIVE_MASK_USE_ALT_OFFSET & _rel_options) { // do altitude translation
        loc.alt += _translation.alt;
    }
}

void AP_Mission_Relative::rotate(Location& loc)
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "AP_Mission_relative::rotate");
    Rotation tmprot;
    float rel_lat, rel_lng; // position of currently parallel translated WayPoint relative to Start-Waypoint
    float rel_distance;     // imaginary distance lat/lng from Start-Waypoint to current WP in [10^7 Degrees]
    rel_lat = loc.lat - _basepoint_loc.lat;
    rel_lng = (loc.lng - _basepoint_loc.lng)*loc.longitude_scale();;
    tmprot.direction = _basepoint_loc.get_bearing_to(loc);   // direction from Start_waypoint to current WP in centidegrees
    tmprot.direction = _translation.direction + tmprot.direction; // total rotation direction in centidegrees
    if (tmprot.direction < 0) {
        tmprot.direction =+ 36000;
    }
    rel_distance = sqrtf((rel_lat*rel_lat)+(rel_lng*rel_lng));
    tmprot.lat = (int32_t)(cosf((float)tmprot.direction/100.0*DEG_TO_RAD)*rel_distance);
    tmprot.lng = (int32_t)(sinf((float)tmprot.direction/100.0*DEG_TO_RAD)*rel_distance)/loc.longitude_scale();
    loc.lat = _basepoint_loc.lat + tmprot.lat;
    loc.lng = _basepoint_loc.lng + tmprot.lng;

}

namespace AP {
    AP_Mission_Relative &mission_relative() {
        return *AP_Mission_Relative::get_singleton();
    }
}
