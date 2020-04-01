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

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

#define TERM_BUFFER 46          // the length of message we expect

class AP_EFI_ECU_Lite: public AP_EFI_Backend {

public:
    // Constructor with initialization
    AP_EFI_ECU_Lite(AP_EFI &_frontend);

    // Update the state structure
    void update() override;

    // get battery voltage and current
    bool get_battery(float &voltage, float &current, float &mah) const override;

private:
    AP_HAL::UARTDriver *_uart;

    // add a single character to the buffer and attempt to decode
    // returns true if a complete sentence was successfully decoded
    bool decode(char c);

    // decode the whole message
    bool decode_msg();

    void write_log();

    void check_status();

    // Serial Protocol Variables
    struct ECU_Data {
        int32_t running_time;
        float rpm;
        float voltage;
        float amperage;
        float mah;
        float fuel;
        int16_t pwm;
        int16_t charging;
        int16_t charge_trim;
        int16_t esc_position;
        int16_t error_state;
        int32_t engine_time;
    };
    ECU_Data _temp;
    ECU_Data _latest;

    // decodeing vars
    uint8_t _payload[TERM_BUFFER];       // payload buffer
    uint8_t _term_offset;       // offset within the _term buffer where the next character should be placed
    bool _in_string;            // true if we should be decoding


    // status test varables
    uint32_t _last_message;
    bool _send_engine_time_message = true;
    bool _send_charge_message = true;
    bool _send_charge_complete_message;
    bool _send_error_state_message = true;
    uint32_t _charge_start_millis;


    // incomming data
    /*
    struct ECU_Data {
        uint32_t header = 1078281045; // '@ECU'
        int32_t running_time;
        float rpm;
        float voltage;
        float amperage;
        float mah;
        float fuel;
        int16_t pwm;
        int16_t charging;
        int16_t charge_trim;
        int16_t esc_position;
        int16_t error_state;
        int32_t engine_time;
        uint32_t crc;
    }; data // I calculate this to be 46 bytes long

    // assign all the data here

    data.crc = 0xFFFFFFFF
    data.crc = crc_crc32(data.crc, data, sizeof(data)-sizeof(uint32_t));
    uart->write((const uint8_t *)&data, sizeof(data));
    */
};
