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
#ifdef ENABLE_SCRIPTING

#include "AP_MotorsMatrix_Scripting_interp.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define debug_print 0

// add a motor and give its testing order
bool AP_MotorsMatrix_Scripting_interp::add_motor(int8_t motor_num, uint8_t testing_order)
{
    if (initialised_ok()) {
        // no adding motors after init
        return false;
    }
    if (interp_table == nullptr) {
        gcs().send_text(MAV_SEVERITY_ERROR, "need at least one interpolation table before adding motors");
        return false;
    }
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {
        _test_order[motor_num] = testing_order;
        motor_enabled[motor_num] = true;
        return true;
    }
    return false;
}

bool AP_MotorsMatrix_Scripting_interp::add_table(factors_table *new_table)
{
    if (initialised_ok()) {
        // no changing tables after init
        return false;
    }

#if debug_print
    hal.console->printf("Got interpolation table for %0.2f\n",new_table->value);
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        hal.console->printf("%i - Roll: %0.2f, Pitch %0.2f, Yaw: %0.2f\n",i,new_table->roll[i],new_table->pitch[i],new_table->yaw[i]);
    }
#endif

    if (interp_table == nullptr) {
        interp_table = new_table;
        return true;
    }

    factors_table *table = interp_table;
    while (table->next != nullptr) {
        table = table->next;
    }

    if (new_table->value <= table->value) {
        gcs().send_text(MAV_SEVERITY_ERROR, "interpolation tables must have increasing values");
        return false;
    }

    table->next = new_table;
    return true;
}

bool AP_MotorsMatrix_Scripting_interp::init(uint8_t expected_num_motors)
{
    // Make sure the correct number of motors have been added
    uint8_t num_motors = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            num_motors++;
        }
    }

    set_initialised_ok(expected_num_motors == num_motors);

    if (!initialised_ok()) {
        _mav_type = MAV_TYPE_GENERIC;
        return false;
    }

    switch (num_motors) {
        case 3:
            _mav_type = MAV_TYPE_TRICOPTER;
            break;
        case 4:
            _mav_type = MAV_TYPE_QUADROTOR;
            break;
        case 6:
            _mav_type = MAV_TYPE_HEXAROTOR;
            break;
        case 8:
            _mav_type = MAV_TYPE_OCTOROTOR;
            break;
        case 10:
            _mav_type = MAV_TYPE_DECAROTOR;
            break;
        case 12:
            _mav_type = MAV_TYPE_DODECAROTOR;
            break;
        default:
            _mav_type = MAV_TYPE_GENERIC;
    }

    set_update_rate(_speed_hz);

    // trigger population of factors
    trigger_interp = true;

    return true;
}

// trigger a re-interpolation of roll pitch and yaw factors at the given point
void AP_MotorsMatrix_Scripting_interp::set_interpolation_point(float value)
{
    if (!initialised_ok()) {
        return;
    }
    if (is_equal(interpolation_value,value)) {
        return;
    }
    interpolation_value = value;
    trigger_interp = true;
}

// output - sends commands to the motors
void AP_MotorsMatrix_Scripting_interp::output_to_motors()
{
    // update the roll pitch and yaw factors
    if (trigger_interp) {
        interp();
        trigger_interp = false;
#if debug_print
        hal.console->printf("Interpolatied to %0.2f\n",interpolation_value);
        for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                hal.console->printf("%i - Roll: %0.2f, Pitch %0.2f, Yaw: %0.2f\n",i,_roll_factor[i],_pitch_factor[i],_yaw_factor[i]);
            }
        }
#endif
    }

    // call the base class ouput function
    AP_MotorsMatrix::output_to_motors();
}

void AP_MotorsMatrix_Scripting_interp::interp()
{
    factors_table *table;
    for (table = interp_table; table->next; table=table->next) {
        if (interpolation_value <= table->value) {
            // value is lower than current table
            load_from_table(table);
            return;
        }
        if (table->value < interpolation_value && interpolation_value < table->next->value) {
            // value is between two tables
            interpolate_tables(table,table->next);
            return;
        }
    }

    // value is higher than last table
    load_from_table(table);
}

void AP_MotorsMatrix_Scripting_interp::load_from_table(factors_table *table)
{
    if (table == nullptr) {
        AP_HAL::panic("lookup table nullptr");
    }
    memcpy(_roll_factor,table->roll,sizeof(_roll_factor));
    memcpy(_pitch_factor,table->pitch,sizeof(_pitch_factor));
    memcpy(_yaw_factor,table->yaw,sizeof(_yaw_factor));
}

void AP_MotorsMatrix_Scripting_interp::interpolate_tables(factors_table *table_low,factors_table *table_high)
{
    if (table_low == nullptr || table_high == nullptr) {
        AP_HAL::panic("lookup tables nullptr");
    }
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        _roll_factor[i]  = linear_interpolate(table_low->roll[i],  table_high->roll[i],  interpolation_value, table_low->value, table_high->value);
        _pitch_factor[i] = linear_interpolate(table_low->pitch[i], table_high->pitch[i], interpolation_value, table_low->value, table_high->value);
        _yaw_factor[i]   = linear_interpolate(table_low->yaw[i],   table_high->yaw[i],   interpolation_value, table_low->value, table_high->value);
    }
}

// singleton instance
AP_MotorsMatrix_Scripting_interp *AP_MotorsMatrix_Scripting_interp::_singleton;

#endif // ENABLE_SCRIPTING
