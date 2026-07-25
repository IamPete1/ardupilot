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

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_VL53L5CX_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_I2C.h"
#include <AP_HAL/I2CDevice.h>

/*
 * Driver for the ST Microelectronics VL53L5CX multizone ToF sensor.
 *
 * Protocol ported from the ST VL53L5CX Ultra Lite Driver (ULD) and the
 * SparkFun VL53L5CX Arduino Library:
 *   https://github.com/sparkfun/SparkFun_VL53L5CX_Arduino_Library
 *
 * Initialization data (VL53L5CX_FIRMWARE, VL53L5CX_DEFAULT_CONFIGURATION,
 * VL53L5CX_DEFAULT_XTALK, VL53L5CX_GET_NVM_CMD) must be placed in
 * AP_RangeFinder_VL53L5CX_data.h, copied from vl53l5cx_buffers.h in the ST
 * VL53L5CX Ultra Lite Driver (ULD):
 *   https://www.st.com/en/embedded-software/stsw-img023.html
 */

class AP_RangeFinder_VL53L5CX : public AP_RangeFinder_Backend_I2C
{
public:

    using AP_RangeFinder_Backend_I2C::AP_RangeFinder_Backend_I2C;

    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          class AP_HAL::I2CDevice &dev)
    {
        return configure(NEW_NOTHROW AP_RangeFinder_VL53L5CX(_state, _params, dev));
    }

    void update() override;

    float max_distance() const override { return 4.0f; }
    float min_distance() const override { return 0.02f; }

protected:
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    bool init() override;

    void timer();

    // 16-bit register I2C helpers
    bool write_byte(uint16_t reg, uint8_t val);
    bool read_byte(uint16_t reg, uint8_t &val);
    bool write_block(uint16_t reg, const uint8_t *data, uint32_t len);
    bool read_block(uint16_t reg, uint8_t *data, uint32_t len);

    // DCI (Device Configuration Interface) – internal memory map access.
    // Protocol matches vl53l5cx_dci_write_data() / _dci_read_data() in ST ULD.
    bool dci_write(uint32_t index, const uint8_t *data, uint16_t size);
    bool dci_read(uint32_t index, uint8_t *data, uint16_t size);

    // Calibration helpers (ported from ST ULD internal functions)
    bool send_offset_data(const uint8_t *nvm_buf, uint8_t resolution);
    bool send_xtalk_data(uint8_t resolution);

    bool set_resolution();
    bool start_ranging();
    bool read_distance(uint16_t &distance_mm);

    // State
    uint8_t  _streamcount{0xFF};
    uint16_t _distance_mm{0};
    bool     _new_distance{false};
    uint32_t _data_read_size{0};  // computed in start_ranging()

    // Scratch buffers used during DCI and calibration operations.
    // _tmp must also hold the result stream (data_read_size = 1440 for 8x8 all-outputs).
    // _tmp2 must hold the largest calibration buffer (xtalk = 776 bytes).
    uint8_t _tmp[1500];
    uint8_t _tmp2[800];
    // NVM offset data (488 bytes) stashed during init for reuse by set_resolution()
    uint8_t _nvm_buf[488];

    // 8x8 mode: 64 zones
    static constexpr uint8_t NUM_ZONES = 64;

    // Target status values indicating a valid range (from ST ULD)
    static bool status_is_valid(uint8_t s) {
        return s == 5 || s == 9 || s == 10;
    }

    // Centre-zone indices for 8x8 (row-major, top-left = 0)
    // Rows 3-4, cols 3-4: zones 27, 28, 35, 36
    static constexpr uint8_t CENTRE_ZONES[] = {27, 28, 35, 36};
};

#endif  // AP_RANGEFINDER_VL53L5CX_ENABLED
