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

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>

// angle control default gains, roll and pitch the same
#define AP_UNDERSLUNG_PITCH_ANGLE_P     0.00f // set all gains to zero initially
#define AP_UNDERSLUNG_PITCH_ANGLE_I     0.00f
#define AP_UNDERSLUNG_PITCH_ANGLE_IMAX  1.00f
#define AP_UNDERSLUNG_PITCH_ANGLE_D     0.00f
#define AP_UNDERSLUNG_PITCH_ANGLE_FILT  5.00f
#define AP_UNDERSLUNG_PITCH_ANGLE_DT    0.05f // 20 Hz

#define AP_UNDERSLUNG_ROLL_ANGLE_P      0.00f // set all gains to zero initially
#define AP_UNDERSLUNG_ROLL_ANGLE_I      0.00f
#define AP_UNDERSLUNG_ROLL_ANGLE_IMAX   1.00f
#define AP_UNDERSLUNG_ROLL_ANGLE_D      0.00f
#define AP_UNDERSLUNG_ROLL_ANGLE_FILT   5.00f
#define AP_UNDERSLUNG_ROLL_ANGLE_DT     0.05f // 20 Hz

class AP_UnderSlung
{

public:

    enum SENSOR_TYPE {
        UNDERSLUNG_NONE       = 0,
        UNDERSLUNG_ANALOG     = 1,
    };

    AP_UnderSlung();

    /* Do not allow copies */
    AP_UnderSlung(const AP_UnderSlung &other) = delete;
    AP_UnderSlung &operator=(const AP_UnderSlung&) = delete;

    static AP_UnderSlung *get_singleton();

    // return true if under slung load is enabled
    bool enabled() const;

    // Initialize the under slung load object and prepare it for use
    void init();

    // update under slung load sensor
    void update();

    // return required acceleration (m/s^2)
    float get_accel_x() { return _x_output; }
    float get_accel_y() { return _y_output; }

    // start calibration routine
    bool start_calibration();

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:

    // read an analog port and the body frame angle of the load sensor in radians
    void update_load_angle();

    // calibrate
    void calibrate();

    // parameters
    AP_Int8 _type;                                  // type of under slung load sensor being used
    AP_Int8 _pitch_analog_pin;                      // analog pin connected to load sensor pitch axis
    AP_Float _pitch_analog_volt_min;                // minimum voltage read by load sensor pitch axis
    AP_Float _pitch_analog_volt_max;                // maximum voltage read by load sensor pitch axis
    AP_Float _pitch_analog_volt_center;             // voltage with load at neutral point
    AP_Float _pitch_max_angle;                      // angle when pitch sensor is at maximum and minimum voltage
    AP_Int8 _roll_analog_pin;                       // analog pin connected to load sensor roll axis
    AP_Float _roll_analog_volt_min;                 // minimum voltage read by load sensor roll axis
    AP_Float _roll_analog_volt_max;                 // maximum voltage read by load sensor roll axis
    AP_Float _roll_analog_volt_center;              // voltage with load at neutral point
    AP_Float _roll_max_angle;                       // angle when roll sensor is at maximum and minimum voltage
    AP_Int8 _calibration;                           // enter calibration

    AC_PID       pitch_pid = AC_PID(AP_UNDERSLUNG_PITCH_ANGLE_P,
                                    AP_UNDERSLUNG_PITCH_ANGLE_I,
                                    AP_UNDERSLUNG_PITCH_ANGLE_D,
                                    AP_UNDERSLUNG_PITCH_ANGLE_IMAX,
                                    AP_UNDERSLUNG_PITCH_ANGLE_FILT,
                                    AP_UNDERSLUNG_PITCH_ANGLE_DT);

    AC_PID        roll_pid = AC_PID(AP_UNDERSLUNG_ROLL_ANGLE_P,
                                    AP_UNDERSLUNG_ROLL_ANGLE_I,
                                    AP_UNDERSLUNG_ROLL_ANGLE_D,
                                    AP_UNDERSLUNG_ROLL_ANGLE_IMAX,
                                    AP_UNDERSLUNG_ROLL_ANGLE_FILT,
                                    AP_UNDERSLUNG_ROLL_ANGLE_DT);

    static AP_UnderSlung *_singleton;

    // voltage read by analog sensor
    float _current_analog_voltage_pitch;
    float _current_analog_voltage_roll;
    
    // load angle in body frame (radians)
    float _pitch_angle_bf;
    float _roll_angle_bf;

    // load angle in earth frame (radians)
    float _pitch_angle_ef;
    float _roll_angle_ef;

    // Acceleration output from PID controller (x and y relative to body yaw (m/s^2))
    float _pitch_output;
    float _roll_output;

    // Acceleration output from PID controller (x and y earth frame (m/s^2))
    float _x_output;
    float _y_output;

    // pin for reading analog voltage
    AP_HAL::AnalogSource *pitch_analog_source;
    AP_HAL::AnalogSource *roll_analog_source;
};

namespace AP {
    AP_UnderSlung *underslung();
};
