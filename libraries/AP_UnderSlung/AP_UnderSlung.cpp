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

#include <AP_UnderSlung/AP_UnderSlung.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <utility>


#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <board_config.h>
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

extern const AP_HAL::HAL& hal;

#define PITCH_DEFAULT_PIN 15                     // default pitch angle sensor analog pin
#define ROLL_DEFAULT_PIN 14                      // default roll angle sensor analog pin

const AP_Param::GroupInfo AP_UnderSlung::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Under slung load sensor Type
    // @Description: Under slung sensor type
    // @Values: 0:None,1:analog
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_UnderSlung, _type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PIT_PIN
    // @DisplayName: Under slung load sensor analog voltage pin for pitch direction
    // @Description: Analog input pin to read for load sensor pitch direction
    // @Values: 11:Pixracer,13:Pixhawk ADC4,14:Pixhawk ADC3,15:Pixhawk ADC6,15:Pixhawk2 ADC,50:PixhawkAUX1,51:PixhawkAUX2,52:PixhawkAUX3,53:PixhawkAUX4,54:PixhawkAUX5,55:PixhawkAUX6,103:Pixhawk SBUS
    // @User: Standard
    AP_GROUPINFO("PIT_PIN", 2, AP_UnderSlung, _pitch_analog_pin, PITCH_DEFAULT_PIN),

    // @Param: PIT_V_MIN
    // @DisplayName: analog voltage minimum pitch direction
    // @Description: Minimum voltage supplied by load pitch angle sensor
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("PIT_V_MIN", 3, AP_UnderSlung, _pitch_analog_volt_min, 0.0f),

    // @Param: PIT_V_MAX
    // @DisplayName: analog voltage maximum pitch direction
    // @Description: Maximum voltage supplied by load pitch angle sensor
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("PIT_V_MAX", 4, AP_UnderSlung, _pitch_analog_volt_max, 3.3f),

    // @Param: PIT_V_CENT
    // @DisplayName: analog voltage in pitch axis neutral position
    // @Description: the pitch angle voltage when slung load is in neutral position (straight down)
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("PIT_V_CENT", 5, AP_UnderSlung, _pitch_analog_volt_center, 1.65f),

    // @Param: PIT_ANGLE
    // @DisplayName: Angle when pitch under slung load sensor is at min and max voltage
    // @Description: Maximum and minimum angle the sensors is at when reading maximum and minimum voltage, the angle range must be symmetrical, voltage must increase as angle increases (load swinging ahead of copter)
    // @Units: deg
    // @Increment: 1
    // @Range: 0 60
    // @User: Standard
    AP_GROUPINFO("PIT_ANGLE", 6, AP_UnderSlung, _pitch_max_angle, 45),

    // @Param: RLL_PIN
    // @DisplayName: analog voltage minimum roll direction
    // @Description: Minimum voltage supplied by load roll angle sensor
    // @Values: 11:Pixracer,13:Pixhawk ADC4,14:Pixhawk ADC3,15:Pixhawk ADC6,15:Pixhawk2 ADC,50:PixhawkAUX1,51:PixhawkAUX2,52:PixhawkAUX3,53:PixhawkAUX4,54:PixhawkAUX5,55:PixhawkAUX6,103:Pixhawk SBUS
    // @User: Standard
    AP_GROUPINFO("RLL_PIN", 7, AP_UnderSlung, _roll_analog_pin, ROLL_DEFAULT_PIN),

    // @Param: RLL_V_MIN
    // @DisplayName: analog voltage minimum roll direction
    // @Description: Minimum voltage supplied by load roll angle sensor
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("RLL_V_MIN", 8, AP_UnderSlung, _roll_analog_volt_min, 0.0f),

    // @Param: RLL_V_MAX
    // @DisplayName: analog voltage maximum roll direction
    // @Description: Maximum voltage supplied by load roll angle sensor
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("RLL_V_MAX", 9, AP_UnderSlung, _roll_analog_volt_max, 3.3f),

    // @Param: Rll_V_CENT
    // @DisplayName: analog voltage in roll axis neutral position
    // @Description: the roll angle voltage when slung load is in neutral position (straight down)
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("Rll_V_CENT", 10, AP_UnderSlung, _roll_analog_volt_center, 1.65f),

    // @Param: RLL_ANGLE
    // @DisplayName: Angle when roll under slung load sensor is at min and max voltage
    // @Description: Maximum and minimum angle the sensors is at when reading maximum and minimum voltage, the angle range must be symmetrical, voltage must increase as angle increases (load swinging to right?? of copter)
    // @Units: deg
    // @Increment: 1
    // @Range: 0 60
    // @User: Standard
    AP_GROUPINFO("RLL_ANGLE", 11, AP_UnderSlung, _roll_max_angle, 45),

    // @Param: CAL
    // @DisplayName: under slung load sensor calibration start
    // @Description: Start under slung load sensor calibration by setting this to 1
    // @Values: 0:None, 1:Calibrate
    // @User: Standard
    AP_GROUPINFO("CAL", 12, AP_UnderSlung, _calibration, 0),


    AP_GROUPEND
};

// constructor
AP_UnderSlung::AP_UnderSlung()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton) {
        AP_HAL::panic("Too many Under Slung Load sensors");
    }
#endif
    _singleton = this;
}

/*
 * Get the AP_UnderSlung singleton
 */
AP_UnderSlung *AP_UnderSlung::get_singleton()
{
    return _singleton;
}

// return true if under slung sensor is enabled
bool AP_UnderSlung::enabled() const
{
    return (_type != UNDERSLUNG_NONE);
}

// Initialize the under slung object and prepare it for use
void AP_UnderSlung::init()
{
    // pins for reading the under slung sensor voltage
    pitch_analog_source = hal.analogin->channel(_pitch_analog_pin);
    roll_analog_source = hal.analogin->channel(_roll_analog_pin);
}

// update under slung load sensor, expected to be called at 20hz
void AP_UnderSlung::update()
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // check for calibration
    calibrate();

    // read body frame load angle
    update_load_angle();

    // convert to earth frame
    _pitch_angle_ef = _pitch_angle_bf + AP::ahrs().pitch;
    _roll_angle_ef = _roll_angle_bf + AP::ahrs().roll;

    // run PID controller to calculate required acceleration to return the load to the neutral pointer
    // currently the control always targets the load to be at its neutral point
    // I gain should be left at zero to ensure the under slung load offsets do not overpower the navigation controller
    // maybe go to PID control rates not angles??

}

bool AP_UnderSlung::start_calibration()
{
    if (enabled() && (_calibration == 0)) {
        _calibration = 1;
        return true;
    }
    return false;
}

// read an analog port and calculate angle of the under slung load
// assumes voltage increases as load moves ahead and to the right?? of the copter
void AP_UnderSlung::update_load_angle()
{
    float voltage_ratio;

    // read pitch if setup
    if (!is_zero((float)_pitch_analog_pin)) {
        pitch_analog_source->set_pin(_pitch_analog_pin);
        _current_analog_voltage_pitch = pitch_analog_source->voltage_average_ratiometric();

        if (_current_analog_voltage_pitch > _pitch_analog_volt_center) {
            voltage_ratio = linear_interpolate(0, 1.0f, _current_analog_voltage_pitch, _pitch_analog_volt_center, _pitch_analog_volt_max);
        } else {
            voltage_ratio = linear_interpolate(-1.0f, 0, _current_analog_voltage_pitch, _pitch_analog_volt_min, _pitch_analog_volt_center);
        }

        _pitch_angle_bf = voltage_ratio * radians(_pitch_max_angle);
    } else {
        _pitch_angle_bf = 0.0f;
    }

    // read roll if setup
    if (!is_zero((float)_roll_analog_pin)) {
        roll_analog_source->set_pin(_roll_analog_pin);
        _current_analog_voltage_roll = roll_analog_source->voltage_average_ratiometric();

        if (_current_analog_voltage_roll > _roll_analog_volt_center) {
            voltage_ratio = linear_interpolate(0, 1.0f, _current_analog_voltage_roll, _roll_analog_volt_center, _roll_analog_volt_max);
        } else {
            voltage_ratio = linear_interpolate(-1.0f, 0, _current_analog_voltage_roll, _roll_analog_volt_min, _roll_analog_volt_center);
        }

        _roll_angle_bf = voltage_ratio * radians(_roll_max_angle);
    } else {
        _roll_angle_bf = 0.0f;
    }
}



// calibrate load sensor
void AP_UnderSlung::calibrate()
{
    // exit immediately if armed or too soon after start
    if (hal.util->get_soft_armed()) {
        return;
    }

    // return if not calibrating
    if (_calibration == 0) {
        return;
    }

    /* Not yet done this yet
    switch (_type) {
        case WindVaneType::WINDVANE_HOME_HEADING:
        case WindVaneType::WINDVANE_PWM_PIN:
            gcs().send_text(MAV_SEVERITY_INFO, "WindVane: No cal required");
            _calibration.set_and_save(0);
            break;
        case WindVaneType::WINDVANE_ANALOG_PIN:
            // start calibration
            if (_cal_start_ms == 0) {
                _cal_start_ms = AP_HAL::millis();
                _cal_volt_max = _current_analog_voltage;
                _cal_volt_min = _current_analog_voltage;
                gcs().send_text(MAV_SEVERITY_INFO, "WindVane: Calibration started, rotate wind vane");
            }

            // record min and max voltage
            _cal_volt_max = MAX(_cal_volt_max, _current_analog_voltage);
            _cal_volt_min = MIN(_cal_volt_min, _current_analog_voltage);

            // calibrate for 30 seconds
            if ((AP_HAL::millis() - _cal_start_ms) > 30000) {
                // check for required voltage difference
                const float volt_diff = _cal_volt_max - _cal_volt_min;
                if (volt_diff >= WINDVANE_CALIBRATION_VOLT_DIFF_MIN) {
                    // save min and max voltage
                    _dir_analog_volt_max.set_and_save(_cal_volt_max);
                    _dir_analog_volt_min.set_and_save(_cal_volt_min);
                    _calibration.set_and_save(0);
                    gcs().send_text(MAV_SEVERITY_INFO, "WindVane: Calibration complete (volt min:%.1f max:%1.f)",
                            (double)_cal_volt_min,
                            (double)_cal_volt_max);
                } else {
                    gcs().send_text(MAV_SEVERITY_INFO, "WindVane: Calibration failed (volt diff %.1f below %.1f)",
                            (double)volt_diff,
                            (double)WINDVANE_CALIBRATION_VOLT_DIFF_MIN);
                }
                _cal_start_ms = 0;
            }
            break;
    }
    */
}

AP_UnderSlung *AP_UnderSlung::_singleton = nullptr;

namespace AP {
    AP_UnderSlung *underslung()
    {
        return AP_UnderSlung::get_singleton();
    }
};
