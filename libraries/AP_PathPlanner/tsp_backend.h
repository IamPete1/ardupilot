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

#include "AP_PathPlanner.h"
#include <GCS_MAVLink/GCS.h>

class tsp_backend
{
public:
    // constructor
    tsp_backend(AP_PathPlanner &frontend) :
    _frontend(frontend)
    {   
    }

    // by default fail
    virtual AP_PathPlanner::Planning_State run() 
    {
        return AP_PathPlanner::Planning_State::failed;
    }

protected:
    AP_PathPlanner &_frontend;
    
};
