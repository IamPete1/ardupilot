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

#define MESSAGE_TIME_MS 5000

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
            internal_state.lifetime_run_time = _latest.engine_time;

            // check if we should notify on any change of status
            check_status();

            // write the latest data to a log
            write_log();

            copy_to_frontend();
        }
    }
}

bool AP_EFI_ECU_Lite::get_battery(float &voltage, float &current, float &mah) const
{
    voltage = _latest.voltage;
    current = _latest.amperage;
    mah = _latest.mah;
    return true;
}

void AP_EFI_ECU_Lite::check_status()
{
    const uint32_t now = AP_HAL::millis();

    if ((now - _last_message) > MESSAGE_TIME_MS) {
        _last_message = now;

        if (_latest.error_state == 1) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "POWER BUS ANOMALY");
        }

        if (_latest.error_state == 2 && _send_error_state_message) {
            _send_engine_time_message = false;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "BUS STABILIZED");
        }

        // Engine Time (send once per engine cycle)
        if (_latest.rpm < 1 && _send_engine_time_message) {
            _send_engine_time_message = false;

            // Engine Time 
            int16_t hours = _latest.engine_time / 3600;
            int16_t tenths = (_latest.engine_time % 3600) / 360;
            gcs().send_text(MAV_SEVERITY_INFO, "Engine Time: %d.%d", hours, tenths);
        }

        // Reset Engine Message
        if (_latest.rpm > 3000) {
            _send_engine_time_message = true;
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
        mah           : _latest.mah,
        fuel          : _latest.fuel,
        pwm           : _latest.pwm,
        charging      : _latest.charging,
        charge_trim   : _latest.charge_trim,
        esc_position  : _latest.esc_position,
        error_state   : _latest.error_state,
        engine_time   : _latest.engine_time,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// add a single character to the buffer and attempt to decode
// returns true if a complete sentence was successfully decoded
bool AP_EFI_ECU_Lite::decode(char c)
{
    // look for start of a string
    if (!_in_string) {
        if (c == '@') {
            _term_offset = 0;
            _payload[_term_offset++] = c;
            _in_string = true;
        }
        return false;
    }

    // otherwise add the char to the current term
    _payload[_term_offset++] = c;

    // end of a string
    if (_term_offset > TERM_BUFFER) {
        _in_string = false;
        return decode_msg();
    }

    return false;
}

bool AP_EFI_ECU_Lite::decode_msg()
{
    uint8_t offset = 0;
    uint32_t header;
    memcpy(&header, &_payload[offset], sizeof(header));
    offset += sizeof(header);

    // Should be '@ECU'
    if (header!= 1078281045) {
        return false;
    }

    memcpy(&_temp.running_time, &_payload[offset], sizeof(_temp.running_time));
    offset += sizeof(_temp.running_time);

    memcpy(&_temp.rpm, &_payload[offset], sizeof(_temp.rpm));
    offset += sizeof(_temp.rpm);

    memcpy(&_temp.voltage, &_payload[offset], sizeof(_temp.voltage));
    offset += sizeof(_temp.voltage);

    memcpy(&_temp.amperage, &_payload[offset], sizeof(_temp.amperage));
    offset += sizeof(_temp.amperage);

    memcpy(&_temp.mah, &_payload[offset], sizeof(_temp.mah));
    offset += sizeof(_temp.mah);

    memcpy(&_temp.fuel, &_payload[offset], sizeof(_temp.fuel));
    offset += sizeof(_temp.fuel);

    memcpy(&_temp.pwm, &_payload[offset], sizeof(_temp.pwm));
    offset += sizeof(_temp.pwm);

    memcpy(&_temp.charging, &_payload[offset], sizeof(_temp.charging));
    offset += sizeof(_temp.charging);

    memcpy(&_temp.charge_trim, &_payload[offset], sizeof(_temp.charge_trim));
    offset += sizeof(_temp.charge_trim);

    memcpy(&_temp.esc_position, &_payload[offset], sizeof(_temp.esc_position));
    offset += sizeof(_temp.esc_position);

    memcpy(&_temp.error_state, &_payload[offset], sizeof(_temp.error_state));
    offset += sizeof(_temp.error_state);

    memcpy(&_temp.engine_time, &_payload[offset], sizeof(_temp.engine_time));
    offset += sizeof(_temp.engine_time);

    uint32_t calc_crc = 0xFFFFFFFF;
    calc_crc = crc_crc32(calc_crc, _payload, offset-1);

    uint32_t received_crc;
    memcpy(&received_crc, &_payload[offset], sizeof(received_crc));

    return calc_crc == received_crc;
}

#endif // EFI_ENABLED
