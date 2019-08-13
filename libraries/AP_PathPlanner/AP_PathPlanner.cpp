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

#include "AP_PathPlanner.h"
#include <GCS_MAVLink/GCS.h>

#include "tsp_greedy.h"

#define MAX_PLANNING_POINTS 25 // max points we can pathplan (its n! hard!!)

extern const AP_HAL::HAL &hal;

HAL_Semaphore_Recursive AP_PathPlanner::_rsem;

const AP_Param::GroupInfo AP_PathPlanner::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Waypoint PathPlanning
    // @Description:  Enable Waypoint PathPlanning
    // @Values: 0:Disabled,1:Greedy,2:Two-Opt
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_PathPlanner, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

// constructor
AP_PathPlanner::AP_PathPlanner() :
    _points(32),
    _dist_mat(32),
    _cost_mat(32),
    _best_route(32),
    _test_route(32)
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton) {
        AP_HAL::panic("Too many Path Planners");
    }
#endif
    _singleton = this;

}

/*
 * Get the AP_PathPlanner singleton
 */
AP_PathPlanner *AP_PathPlanner::get_singleton()
{
    return _singleton;
}

// init the pathplanner
void AP_PathPlanner::init()
{
    _state = Planning_State::waiting;
    return;
}

// read in waypoints from mission
void AP_PathPlanner::load(uint16_t start) 
{
    if (!enabled()) {
        return;
    }

    AP_Mission* mission = AP_Mission::get_singleton();
    if (mission == nullptr) {
        _state = Planning_State::failed;
        return;
    }

    // reset start
    _start_point.index = 0;
    
    AP_Mission::Mission_Command tmp;
    // find previous point for the start
    for (uint16_t i = start - 1; i > 0; i--) {
        if (!mission->read_cmd_from_storage(i, tmp)) {
            _state = Planning_State::failed;
            return;
        }

        // can be any sort of nav command
        if (mission->is_nav_cmd(tmp)) {
            _start_point.index = tmp.index;

            Location temp_loc(tmp.content.location.lat, tmp.content.location.lng, tmp.content.location.alt, Location::AltFrame::ABSOLUTE);
            if (!temp_loc.get_vector_from_origin_NEU(_start_point.location)) {
                _state = Planning_State::failed;
                return;
            }
            break;
        }
    }
    // if we haven't found a start point then we will use the current location??

    // step through mission and load in points
    _num_points = 0;
    for (uint16_t i = start + 1; i < mission->num_commands(); i++) {
        if (!mission->read_cmd_from_storage(i, tmp)) {
            _state = Planning_State::failed;
            return;
        }

        // we have reached the stop waypoint
        if (tmp.id == MAV_DO_PATHPLAN_STARTSTOP) {
            break;
        }

        // add normal waypoint to list
        if (tmp.id == MAV_CMD_NAV_WAYPOINT) {
            _num_points++;
            if (!_points.expand_to_hold(_num_points)) {
                _state = Planning_State::failed;
                return;
            }
            _points[_num_points-1].index = tmp.index;
            //gcs().send_text(MAV_SEVERITY_INFO, "point %i, index %i",_num_points-1,tmp.index);

            Location temp_loc(tmp.content.location.lat, tmp.content.location.lng, tmp.content.location.alt, Location::AltFrame::ABSOLUTE);
            if (!temp_loc.get_vector_from_origin_NEU( _points[_num_points].location)) {
                _state = Planning_State::failed;
                return;
            }
        }

        // check for unrealistically large number of points
        // could never hope to solve
        if (_num_points > MAX_PLANNING_POINTS) {
            _state = Planning_State::failed;
            return;
        }
    }

    // find next location point for end
    for (uint16_t i = _points[_num_points-1].index + 1; i < mission->num_commands(); i++) {
        if (!mission->read_cmd_from_storage(i, tmp)) {
            _state = Planning_State::failed;
            return;
        }

        // can be any sort of nav command
        if (mission->is_nav_cmd(tmp)) {
            _end_point.index = tmp.index;

            Location temp_loc(tmp.content.location.lat, tmp.content.location.lng, tmp.content.location.alt, Location::AltFrame::ABSOLUTE);
            if (!temp_loc.get_vector_from_origin_NEU(_end_point.location)) {
                _state = Planning_State::failed;
                return;
            }
            break;
        }
    
    }

    // make sure we have enough room
    if (!_dist_mat.expand_to_hold(powf(_num_points + 1,2.0f)) ||
        !_cost_mat.expand_to_hold(powf(_num_points + 1,2.0f)) ||
        !_best_route.expand_to_hold(_num_points) ||
        !_test_route.expand_to_hold(_num_points)) {
        _state = Planning_State::failed;
        return;
    }

    /*
    build distance matrix between all points
    we can caulate the distance in advance as points can't be moved
    the cost to travel the given distance could change so is calcuclated 'live'
    start and end point are considered as the same
    */
    for (uint16_t i = 0; i < _num_points+1; i++) {
        for (uint16_t j = 0; j < _num_points+1; j++) {
            if (i == 0) {
                // use helper to fill vector like a matrix
                set_dist(i,j,_start_point.location - _points[j].location);
            } else if (j ==0) {
                set_dist(i,j,_points[i].location - _end_point.location);
            } else {
                set_dist(i,j,_points[i].location - _points[j].location);
            }
        }
    }

    _state = Planning_State::loaded;

    // load solver
    switch (_enabled) {
        case  PLANNER_DISABLED:
            // shouldnt have got this far
            _state = Planning_State::failed;
            return;
        case PLANNER_GREEDY:
            _solver = new tsp_greedy(*this);
            break;
        case PLANNER_TWO_OPT:
            //_solver = new tsp_two_opt(*this);
            break;
    }

    // make sure it loaded
    if (_solver == nullptr) {
        _state = Planning_State::failed;
        return;
    }

    // create low priority thread
    if (!_thread_created) {
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_PathPlanner::tsp_thread, void),
                                      "pathplanning",
                                       8192, AP_HAL::Scheduler::PRIORITY_IO, -1)) {
        _state = Planning_State::failed;                      
        return;
        }
    }

    // if we got this far then its all up and running
    _thread_created = true;
    _state = Planning_State::calculating;
}

// thread where tsp solvers are run
void AP_PathPlanner::tsp_thread()
{
    while (true) {
        // if we have converged and there is no change in costs there is nothing to do    
        if (_state == Planning_State::complete || (_state == Planning_State::converged && !costs_changed())) {
            // wait for a while and recheck
            hal.scheduler->delay(100);        
            continue;
        }

        // calculate the cost matrix
        for (uint16_t i = 0; i < _num_points+1; i++) {
            for (uint16_t j = 0; j < _num_points+1; j++) {
                set_cost(i,j,get_vehicle_cost(get_dist(i,j)));
            }
        }

        /*
        run the solver
        solver should return when it improves
        this allows us to use a better solution until a optimal one had been found
        */
        _state = _solver->run();

        // update the best route
        {
            WITH_SEMAPHORE(_rsem);
            for (uint16_t i = 0; i < _num_points; i++) {
                _best_route[i] = _test_route[i];
            }
        }

        // if we converged print the best route for debug
        if (_state == Planning_State::converged) {
            gcs().send_text(MAV_SEVERITY_INFO, "best route");
            for (uint16_t i = 0; i < _num_points; i++) {
                gcs().send_text(MAV_SEVERITY_INFO, "%i",_points[_best_route[i]].index);
            }
        }
    }
}

// check if condisions have changed sufficiently to warrant looking for new optimum
bool AP_PathPlanner::costs_changed()
{
    return false;
}

// return the cost to travel the given vector
float AP_PathPlanner::get_vehicle_cost(Vector3f dist)
{
    // basic distance based cost function
    float cost = powf(dist.x,2.0f); 
    cost += powf(dist.y,2.0f);
    //cost += powf(dist.z,2.0f);

    return cost;
}

// read in waypoints from mission
bool AP_PathPlanner::is_point(uint16_t index) const
{
    if (!enabled() || _state == Planning_State::failed) {
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "is_point %i",index);


    // check the start but not the end
    if (index == _start_point.index || index == _end_point.index) {
           return true;
    }

    for (uint16_t i = 0; i < _num_points; i++) {
       gcs().send_text(MAV_SEVERITY_INFO, "point %i, index %i",i,_points[i].index);
       if (index == _points[i].index) {
           return true;
       }
    }

    gcs().send_text(MAV_SEVERITY_INFO, "is_point false");

    return false;
}

// return the next point we should visit
uint16_t AP_PathPlanner::get_next_point()
{
    WITH_SEMAPHORE(_rsem);
    {
        // mark current point as visited
        if (_current_point == _start_point.index) {
            _start_point.visited = true;
        } else {
            for (uint16_t i = 0; i < _num_points; i++) {
                if (_current_point == _points[i].index) {
                    _points[i].visited = true;
                    break;
                }
            }
        }

        // find the first point in the path that has not been visited
        for (uint16_t i = 0; i < _num_points; i++) {
            if ( !_points[_best_route[i]].visited) {
                _current_point = _points[_best_route[i]].index;
                gcs().send_text(MAV_SEVERITY_INFO, "get next point %i",_current_point);
                return _current_point;

            }
        }

        // next point must be the end, set than and we are done
        _state = Planning_State::complete;
        gcs().send_text(MAV_SEVERITY_INFO, "get next point end %i",_end_point.index);
        return _end_point.index;
    }
}

// helper to fill dist vector like a matrix 
void AP_PathPlanner::set_dist(uint16_t i, uint16_t j, Vector3f dist)
{
    // calculate the position of the element in the vector
    uint16_t index = i + j + (_num_points * i);

    _dist_mat[index] = dist;
}

// helper to get dist vector like a matrix 
Vector3f AP_PathPlanner::get_dist(uint16_t i, uint16_t j)
{
    // calculate the position of the element in the vector
    uint16_t index = i + j + (_num_points * i);

    return _dist_mat[index];
}

// helper to fill cost vector like a matrix 
void AP_PathPlanner::set_cost(uint16_t i, uint16_t j, float cost)
{
    // calculate the position of the element in the vector
    uint16_t index = i + j + (_num_points * i);

    _cost_mat[index] = cost;
}

// helper to get cost vector like a matrix 
float AP_PathPlanner::get_cost(uint16_t i, uint16_t j)
{
    // calculate the position of the element in the vector
    uint16_t index = i + j + (_num_points * i);

    return _cost_mat[index];
}

AP_PathPlanner *AP_PathPlanner::_singleton = nullptr;

namespace AP {
    AP_PathPlanner *pathplanner()
    {
        return AP_PathPlanner::get_singleton();
    }
};
