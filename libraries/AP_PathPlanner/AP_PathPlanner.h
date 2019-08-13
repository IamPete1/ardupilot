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
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/AP_ExpandingArray.h>
#include <AP_Common/Location.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>

class tsp_backend;

class AP_PathPlanner
{
    friend class tsp_backend;
    friend class tsp_greedy;

public:
    AP_PathPlanner();

    /* Do not allow copies */
    AP_PathPlanner(const AP_PathPlanner &other) = delete;
    AP_PathPlanner &operator=(const AP_PathPlanner&) = delete;

    static AP_PathPlanner *get_singleton();

    void init();

    // read in waypoints from mission
    void load(uint16_t start);

    // return true if wind vane is enabled
    bool enabled() const {
        return _enabled != 0;
    }

    // return true is the waypoint is one included in the path planning
    bool is_point(uint16_t index) const;

    // return the next destination point
    uint16_t get_next_point(); 

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];
    
private:

    enum solvers {
        PLANNER_DISABLED = 0,
        PLANNER_GREEDY,
        PLANNER_TWO_OPT,
    };

    struct Path_Points {
        uint16_t index;
        Vector3f location;
        bool visited;
    };

    enum Planning_State {
        failed = 0,
        calculating,
        improved,
        converged,
        waiting,
        loaded,
        complete,
    };

    Planning_State _state;

    // starting and finishing points
    Path_Points _start_point;
    Path_Points _end_point;

    // number of point to path plan
    // not includeing start and finish
    uint16_t _num_points;
    uint16_t _current_point;

    // matrix containing mission index of waypoints to be path planed
    AP_ExpandingArray<Path_Points> _points;

    // array containing the distance to travel between all points
    AP_ExpandingArray<Vector3f> _dist_mat;
    AP_ExpandingArray<float> _cost_mat;

    // array of current best and new best
    AP_ExpandingArray<uint16_t> _best_route;
    AP_ExpandingArray<uint16_t> _test_route;

    static HAL_Semaphore_Recursive _rsem; // semaphore for multi-thread use
    bool _thread_created;

    // parameters
    AP_Int8 _enabled; // type of pathplanner, 0 disabled, 1 greedy, 2 two-opt

    // helper to fill dist vector like a matrix 
    void set_dist(uint16_t i, uint16_t j, Vector3f dist);
    Vector3f get_dist(uint16_t i, uint16_t j);
    void set_cost(uint16_t i, uint16_t j, float cost);
    float get_cost(uint16_t i, uint16_t j);

    // thread where tsp solvers are run
    void tsp_thread();

    // check if condisions have changed sufficiently to warent looking for new optimum
    bool costs_changed();

    // return the cost to travel the given vector
    float get_vehicle_cost(Vector3f dist);

    // the solver backend
    tsp_backend *_solver;

    static AP_PathPlanner *_singleton;
};

namespace AP {
    AP_PathPlanner *pathplanner();
};
