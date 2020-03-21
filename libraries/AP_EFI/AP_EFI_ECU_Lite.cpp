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
 
#include <AP_HAL/AP_HAL.h>
#include "AP_EFI_ECU_Lite.h"

#if EFI_ENABLED
#include <AP_SerialManager/AP_SerialManager.h>

#define MESSAGE_TIME_MS 1000

extern const AP_HAL::HAL &hal;

AP_EFI_ECU_Lite::AP_EFI_ECU_Lite(AP_EFI &_frontend):
    AP_EFI_Backend(_frontend)
{
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EFI, 0);
}

void AP_EFI_ECU_Lite::update()
{
    if (!_uart) {
        return;
    }

    // read any available data
    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        char c = _uart->read();
        if (decode(c)) {
            _latest = _temp;

            // successfully decoded a new reading
            internal_state.last_updated_ms = AP_HAL::millis();
            internal_state.run_time = _latest.running_time;
            internal_state.engine_speed_rpm = _latest.rpm;
            internal_state.fuel_remaining_pct = _latest.fuel;
            internal_state.lifetime_run_time = _latest.hobbs;

            //Overvoltage Throttle Hold
            //if (_latest.overvoltage == 1) {
                //ecu_lite_throttle_min = plane.g2.supervolo_ov_thr;
            //}

            // check if we should notify on any change of status
            check_status();

            // write the latest data to a log
            write_log();

            copy_to_frontend();
        }
    }
}

bool AP_EFI_ECU_Lite::get_battery(float &voltage, float &current) const
{
    voltage = _latest.voltage;
    current = _latest.amperage;
    return true;
}

void AP_EFI_ECU_Lite::check_status()
{
    const uint32_t now = AP_HAL::millis();

    if ((now - _last_message) > MESSAGE_TIME_MS) {
        _last_message = now;

        if (_latest.overvoltage == 1){
              gcs().send_text(MAV_SEVERITY_INFO, "BATTERIES DISCONNECTED");
        }

        // Hobbs Time (send once per engine cycle)
        if (_latest.rpm < 1 && _send_hobbs_message) {
            _send_hobbs_message = false;

            // Hobbs Time 
            int16_t hours = _latest.hobbs / 3600;
            int16_t tenths = (_latest.hobbs % 3600) / 360;
            gcs().send_text(MAV_SEVERITY_INFO, "Hobbs Time: %d.%d", hours, tenths);
        }

        // Reset Hobbs Message
        if (_latest.rpm > 3000) {
            _send_hobbs_message = true;
        }

        // if charging
        float charge_current_seconds;
        if (_latest.charging == 1) {

            //Send charge start message (once)
            if (_send_charge_message) {
                _send_charge_message = false;
                gcs().send_text(MAV_SEVERITY_INFO, "Charge Start");
            }

            //Charge Timer
            charge_current_seconds = (now - _charge_start_millis) / 1000;

            //Charge Calibration Messaging (optional)
            //if (plane.g2.supervolo_dev == 1){
            //    gcs().send_text(MAV_SEVERITY_INFO, "CT:%f PWM:%d V:%.1f A:%.1f ESC:%d Trim:%d", charge_current_seconds, _latest.pwm, _latest.voltage, _latest.amperage, _latest.esc_position, _latest.charge_trim);
            //}

            _send_charge_complete_message = true;

        } else {
            //Send charge complete message (once)
            if (_send_charge_complete_message) {
                _send_charge_complete_message = false;
                gcs().send_text(MAV_SEVERITY_INFO, "Charge Stop");

                charge_current_seconds = (now - _charge_start_millis) / 1000;
                int16_t minutes = floorf(charge_current_seconds / 60);
                int16_t seconds = charge_current_seconds - (minutes * 60);
                gcs().send_text(MAV_SEVERITY_INFO, "Charging Time %d.%d", minutes, seconds);
            }

            // Reset Current Charge Timer 
            _charge_start_millis = now;
            _send_charge_message = true;
        }
    }
}

void AP_EFI_ECU_Lite::write_log()
{
    const struct Log_EFI_ECU_Lite pkt{
        LOG_PACKET_HEADER_INIT(LOG_EFI_ECU_LITE_MSG),
        time_us       : AP_HAL::micros64(),
        running_time  : _latest.running_time,
        rpm           : _latest.rpm,
        voltage       : _latest.voltage,
        amperage      : _latest.amperage,
        fuel          : _latest.fuel,
        pwm           : _latest.pwm,
        charging      : _latest.charging,
        charge_trim   : _latest.charge_trim,
        esc_position  : _latest.esc_position,
        overvoltage   : _latest.overvoltage,
        hobbs         : _latest.hobbs,
        hobbs_message : _latest.hobbs_message
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// add a single character to the buffer and attempt to decode
// returns true if a complete sentence was successfully decoded
bool AP_EFI_ECU_Lite::decode(char c)
{
    // look for start of a string
    if (!_in_string) {
        if (c == 'R') {
            // Expecting R as first Char
            _sentence_valid = true;
            _term_number = 0;
            _term_offset = 0;
            _in_string = true;
            _term[_term_offset++] = c;
        }
        return false;
    }

    // end of a string
    if (c == '\n') {
        decode_latest_term();
        _in_string = false;
        return _sentence_valid;
    }

    // end of a term in the string
    if (c == ' ' || c == ':') {
        decode_latest_term();
        return false;
    }

    // otherwise add the char to the current term
    _term[_term_offset++] = c;

    // we have overrun the expected sentence
    if (_term_offset > TERM_BUFFER) {
        _in_string = false;
    }

    return false;
}

void AP_EFI_ECU_Lite::decode_latest_term()
{
    // null terminate and move onto the next term
    _term[_term_offset] = 0;
    _term_offset = 0;
    _term_number++;

    // debugging
    //gcs().send_text(MAV_SEVERITY_INFO, "ECU: %s",_term);

    switch (_term_number) {
        case 1:
            if (strcmp(_term, "RT") != 0) {
                _sentence_valid = false;
            }
            break;

        case 2:
            _temp.running_time = strtof(_term, NULL);
            // out of range values
            if (_temp.running_time < 0.0f) {
                _sentence_valid = false;
            }
            break;

        case 3:
            if (strcmp(_term, "RPM") != 0) {
                _sentence_valid = false;
            }
            break;

        case 4:
            _temp.rpm = strtof(_term, NULL);
            if (_temp.rpm < 0.0f) {
                _sentence_valid = false;
            }
            break;

        case 5:
            if (strcmp(_term, "V") != 0) {
                _sentence_valid = false;
            }
            break;

        case 6:
            _temp.voltage = strtof(_term, NULL);
            if (_temp.voltage < 0.0f) {
                _sentence_valid = false;
            }
            break;

        case 7:
            if (strcmp(_term, "A") != 0) {
                _sentence_valid = false;
            }
            break;

        case 8:
            _temp.amperage = strtof(_term, NULL);
            if (_temp.amperage < 0.0f) {
                _sentence_valid = false;
            }
            break;

        case 9:
            if (strcmp(_term, "F") != 0) {
                _sentence_valid = false;
            }
            break;

        case 10:
            _temp.fuel = strtof(_term, NULL);
            if (_temp.fuel < 0.0f || _temp.fuel > 100) {
                _sentence_valid = false;
            }
            break;

        case 11:
            if (strcmp(_term, "PWM") != 0) {
                _sentence_valid = false;
            }
            break;

        case 12:
            _temp.pwm = strtol(_term, NULL, 10);
            break;

        case 13:
            if (strcmp(_term, "CH") != 0) {
                _sentence_valid = false;
            }
            break;

        case 14:
            _temp.charging = strtol(_term, NULL, 10);
            break;

        case 15:
            if (strcmp(_term, "ESC") != 0) {
                _sentence_valid = false;
            }
            break;

        case 16:
            _temp.esc_position = strtol(_term, NULL, 10);
            break;

        case 17:
            if (strcmp(_term, "CT") != 0) {
                _sentence_valid = false;
            }
            break;

        case 18:
            _temp.charge_trim = strtol(_term, NULL, 10);
            break;

        case 19:
            if (strcmp(_term, "OV") != 0) {
                _sentence_valid = false;
            }
            break;

        case 20:
            _temp.overvoltage = strtol(_term, NULL, 10);
            break;

        case 21:
            if (strcmp(_term, "H") != 0) {
                _sentence_valid = false;
            }
            break;

        case 22:
            _temp.hobbs = strtol(_term, NULL, 10);
            break;
    }

}

#endif // EFI_ENABLED
