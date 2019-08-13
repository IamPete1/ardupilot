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

#include "tsp_greedy.h"

// constructor
tsp_greedy::tsp_greedy(AP_PathPlanner &frontend) :
    tsp_backend(frontend),
    _visited(32)
{
}

AP_PathPlanner::Planning_State tsp_greedy::run()
{
    // make sure there is room
    if (!_visited.expand_to_hold(_frontend._num_points)) {
        return AP_PathPlanner::Planning_State::failed;
    }

    //gcs().send_text(MAV_SEVERITY_INFO, "greedy running on %i points",  _frontend._num_points);


    // clear visited array
    for (uint16_t i = 0; i < _frontend._num_points; i++) {
        _visited[i] = false;
    }

    float cost;
    bool temp;
    float min_cost;
    uint16_t greedy_point = 0;
    uint16_t current_point = 0;
    _visited[0] = true;

    // find the cosest unvisited point until we reach the end
    for (uint16_t i = 0; i < _frontend._num_points +1; i++) {
        if (i != 0) {
            current_point = greedy_point;
            _visited[current_point] = true;
            gcs().send_text(MAV_SEVERITY_INFO, "%i", _frontend._points[current_point].index);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "%i", _frontend._start_point.index);
        }

        // try all destinations
        temp = false;
        for (uint16_t j = 0; j < _frontend._num_points +1; j++) {
            if (_visited[j]) {
                continue;
            }
            gcs().send_text(MAV_SEVERITY_INFO, "cost %i, %f", _frontend._points[j].index, _frontend.get_cost(current_point,j));
            //Vector3f dist = _frontend.get_dist(current_point,j);
            //gcs().send_text(MAV_SEVERITY_INFO, "cost %i, x:%f, y:%f, z:%f", _frontend._points[j].index, dist.x, dist.y, dist.z);

            if (!temp) {
                // first destination is the best so far
                min_cost = _frontend.get_cost(current_point,j);
                greedy_point = j;
                temp = true;
            } else {
                float temp_cost = _frontend.get_cost(current_point,j);
                if (temp_cost < min_cost) {
                    min_cost = temp_cost;
                    greedy_point = j;
                }
            }
        }

        _frontend._test_route[i] = greedy_point - 1;
        cost += min_cost;
    }

    // need to visit the last remaining point
     cost += _frontend.get_cost(greedy_point,0);

    // greedy converges on the first go
    return AP_PathPlanner::Planning_State::converged;
}









