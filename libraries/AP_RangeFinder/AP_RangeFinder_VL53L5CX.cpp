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

/*
 * Driver for the ST VL53L5CX multizone Time-of-Flight sensor.
 *
 * Initialization sequence and protocol ported from the ST VL53L5CX Ultra Lite
 * Driver (ULD) vl53l5cx_api.c, as wrapped by the SparkFun VL53L5CX library:
 *   https://github.com/sparkfun/SparkFun_VL53L5CX_Arduino_Library
 *
 * Data arrays (firmware, configuration, xtalk, NVM command) are in
 * AP_RangeFinder_VL53L5CX_data.h – copy them from the ULD package available
 * at https://www.st.com/en/embedded-software/stsw-img023.html
 *
 * In 8x8 mode the sensor reports 64 zone distances.  This driver returns the
 * minimum valid distance among the four centre zones (27, 28, 35, 36).
 */

#include "AP_RangeFinder_VL53L5CX.h"

#if AP_RANGEFINDER_VL53L5CX_ENABLED

#include "AP_RangeFinder_VL53L5CX_data.h"
#include <AP_HAL/AP_HAL.h>
#include <string.h>

extern const AP_HAL::HAL &hal;

// Register / address constants (all on the currently selected page unless noted)
static constexpr uint16_t REG_PAGE           = 0x7FFF;  // page-select register
static constexpr uint16_t REG_BOOT_STATUS    = 0x0006;  // 0x01 = ROM booted; 0x00 = FW booted

// UI command interface (page 0x02)
static constexpr uint16_t UI_CMD_STATUS      = 0x2C00;  // read for DCI status
static constexpr uint16_t UI_CMD_START       = 0x2C04;  // read DCI response here
static constexpr uint16_t UI_CMD_END         = 0x2FFF;  // write DCI commands ending here

// DCI configuration indices (from ST ULD vl53l5cx_api.h)
static constexpr uint32_t DCI_DSS_CONFIG    = 0xAD38;
static constexpr uint32_t DCI_ZONE_CONFIG    = 0x5450;
static constexpr uint32_t DCI_RANGING_FREQ   = 0x5458;
static constexpr uint32_t DCI_RESOLUTION     = 0xCF2C;
static constexpr uint32_t DCI_PIPE_CONTROL   = 0xCF78;
static constexpr uint32_t DCI_SINGLE_RANGE   = 0xCD5C;
static constexpr uint32_t DCI_OUTPUT_LIST    = 0xCD78;
static constexpr uint32_t DCI_OUTPUT_CONFIG  = 0xCD60;
static constexpr uint32_t DCI_OUTPUT_ENABLES = 0xCD68;

// Buffer sizes (from ST ULD vl53l5cx_api.h)
static constexpr uint16_t VL53L5CX_NVM_DATA_SIZE       = 492;
static constexpr uint16_t VL53L5CX_OFFSET_BUFFER_SIZE  = 488;
static constexpr uint16_t VL53L5CX_XTALK_BUFFER_SIZE   = 776;
static constexpr uint16_t VL53L5CX_CONFIGURATION_SIZE  = 972;

// Block-header values for result parsing (from ST ULD vl53l5cx_api.h)
static constexpr uint32_t BH_START              = 0x0000000DU;
static constexpr uint32_t BH_METADATA           = 0x54B400C0U;
static constexpr uint32_t BH_COMMONDATA         = 0x54C00040U;
static constexpr uint32_t BH_AMBIENT_RATE       = 0x54D00104U;
static constexpr uint32_t BH_SPAD_COUNT         = 0x55D00404U;
static constexpr uint32_t BH_NB_TARGET_DETECTED = 0xCF7C0401U;
static constexpr uint32_t BH_SIGNAL_RATE        = 0xCFBC0404U;
static constexpr uint32_t BH_RANGE_SIGMA_MM     = 0xD2BC0402U;
static constexpr uint32_t BH_DISTANCE           = 0xD33C0402U;
static constexpr uint32_t BH_REFLECTANCE        = 0xD43C0401U;
static constexpr uint32_t BH_TARGET_STATUS      = 0xD47C0401U;
static constexpr uint32_t BH_MOTION_DETECT      = 0xCC5008C0U;

static constexpr uint16_t IDX_DISTANCE       = 0xD33C;
static constexpr uint16_t IDX_TARGET_STATUS  = 0xD47C;

// Resolution constants
static constexpr uint8_t  RESOLUTION_4X4     = 16;
static constexpr uint8_t  RESOLUTION_8X8     = 64;

// -------------------------------------------------------------------------
// SwapBuffer: reverse byte order within each 4-byte group (big↔little endian)
// Used to byte-swap the sensor result stream, matching the ULD SwapBuffer().
// -------------------------------------------------------------------------
static void swap_buffer(uint8_t *buf, uint32_t size)
{
    uint32_t tmp;
    for (uint32_t i = 0; i < size; i += 4) {
        tmp = (buf[i] << 24) | (buf[i + 1] << 16) | (buf[i + 2] << 8) | (buf[i + 3]);
        memcpy(&(buf[i]), &tmp, 4);
    }
}

// -------------------------------------------------------------------------
// 16-bit register I2C helpers
// -------------------------------------------------------------------------

bool AP_RangeFinder_VL53L5CX::write_byte(uint16_t reg, uint8_t val)
{
    uint8_t buf[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), val};
    return dev.transfer(buf, sizeof(buf), nullptr, 0);
}

bool AP_RangeFinder_VL53L5CX::read_byte(uint16_t reg, uint8_t &val)
{
    uint8_t addr[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    return dev.transfer(addr, sizeof(addr), &val, 1);
}

bool AP_RangeFinder_VL53L5CX::write_block(uint16_t reg, const uint8_t *data, uint32_t len)
{
    const uint32_t CHUNK = 30;
    for (uint32_t offset = 0; offset < len; offset += CHUNK) {
        uint32_t n = MIN(CHUNK, len - offset);
        uint16_t cur = reg + (uint16_t)offset;
        uint8_t buf[2 + CHUNK];
        buf[0] = (uint8_t)(cur >> 8);
        buf[1] = (uint8_t)(cur & 0xFF);
        memcpy(&buf[2], data + offset, n);
        if (!dev.transfer(buf, 2 + n, nullptr, 0)) {
            return false;
        }
    }
    return true;
}

bool AP_RangeFinder_VL53L5CX::read_block(uint16_t reg, uint8_t *data, uint32_t len)
{
    uint8_t addr[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    if (!dev.transfer(addr, sizeof(addr), nullptr, 0)) {
        return false;
    }
    const uint32_t CHUNK = 32;
    for (uint32_t offset = 0; offset < len; offset += CHUNK) {
        uint32_t n = MIN(CHUNK, len - offset);
        if (!dev.transfer(nullptr, 0, data + offset, n)) {
            return false;
        }
    }
    return true;
}

// -------------------------------------------------------------------------
// DCI (Device Configuration Interface)
//
// Protocol matches vl53l5cx_dci_write_data() / vl53l5cx_dci_read_data()
// in the ST ULD.  All transfers occur on page 0x02.
// -------------------------------------------------------------------------

bool AP_RangeFinder_VL53L5CX::dci_write(uint32_t index,
                                         const uint8_t *data,
                                         uint16_t size)
{
    if ((uint32_t)size + 12 > sizeof(_tmp)) {
        return false;
    }

    // Header (4 bytes) matching the ST ULD dci_write_data() format:
    // [size>>4, (size&0x0F)<<4, index>>8, index&0xFF]
    // The ULD copies data first, SwapBuffers the whole block, then overwrites
    // bytes[0..3] with the unswapped header — so the transmitted header is in
    // this pre-swap / big-endian-size format.
    _tmp[0] = (uint8_t)(index >> 8);
    _tmp[1] = (uint8_t)(index & (uint32_t)0xff);
    _tmp[2] = (uint8_t)(((size & (uint16_t)0xff0) >> 4));
    _tmp[3] = (uint8_t)((size & (uint16_t)0xf) << 4);


    // Payload (byte-swapped copy of data, matching ULD SwapBuffer step)
    memcpy(&_tmp[4], data, size);
    swap_buffer(&_tmp[4], size);

    // Footer (8 bytes)
    _tmp[4 + size + 0] = 0x00;
    _tmp[4 + size + 1] = 0x00;
    _tmp[4 + size + 2] = 0x00;
    _tmp[4 + size + 3] = 0x0F;
    _tmp[4 + size + 4] = 0x05;
    _tmp[4 + size + 5] = 0x01;
    _tmp[4 + size + 6] = (uint8_t)((size + (uint16_t)8) >> 8);
    _tmp[4 + size + 7] = (uint8_t)((size + (uint16_t)8) & (uint8_t)0xFF);

    // Write the whole packet ending at UI_CMD_END
    uint16_t addr = UI_CMD_END - (uint16_t)(size + 12) + 1;
    if (!write_block(addr, _tmp, size + 12)) {
        return false;
    }

    hal.scheduler->delay(10);
    uint8_t st[4] = {};
    return read_block(UI_CMD_STATUS, st, 4) && st[1] == 0x03;
}

bool AP_RangeFinder_VL53L5CX::dci_read(uint32_t index,
                                        uint8_t  *data,
                                        uint16_t  size)
{
    if ((uint32_t)size + 12 > sizeof(_tmp)) {
        return false;
    }

    // Command (12 bytes) — header in ULD dci_read format: [size_hi, size_lo, idx_hi, idx_lo]
    uint8_t cmd[12] = {
        (uint8_t)(index >> 8),
        (uint8_t)(index & 0xFF),
        (uint8_t)((size & 0xFF0) >> 4),
        (uint8_t)((size & 0xF) << 4),
        0x00, 0x00, 0x00, 0x0F,
        0x00, 0x02, 0x00, 0x08
    };
    if (!write_block(UI_CMD_END - 11, cmd, sizeof(cmd))) {
        return false;
    }

    // Poll UI_CMD_STATUS byte[1] == 0x03
    hal.scheduler->delay(10);
    uint8_t st[4] = {};
    if (!read_block(UI_CMD_STATUS, st, 4) || st[1] != 0x03) {
        return false;
    }

    // Read header (4 bytes) + data from UI_CMD_START
    if (!read_block(UI_CMD_START, _tmp, size + 12)) {
        return false;
    }
    swap_buffer(_tmp, size + 12);
    memcpy(data, &_tmp[4], size);
    return true;
}

// -------------------------------------------------------------------------
// Offset and xtalk calibration helpers
// Ported from _vl53l5cx_send_offset_data() and _vl53l5cx_send_xtalk_data()
// in the ST ULD vl53l5cx_api.c
// -------------------------------------------------------------------------

bool AP_RangeFinder_VL53L5CX::send_offset_data(const uint8_t *nvm_buf, uint8_t resolution)
{
    static const uint8_t dss_4x4[]  = {0x0F, 0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07};
    static const uint8_t footer[]   = {0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x01, 0xE4};

    uint8_t *buf = _tmp2;
    memcpy(buf, nvm_buf, VL53L5CX_OFFSET_BUFFER_SIZE);

    if (resolution == 16) {
        // 4x4: patch DSS config and downsample signal/range grids from 8x8 to 4x4.
        memcpy(&buf[0x10], dss_4x4, sizeof(dss_4x4));
        swap_buffer(buf, VL53L5CX_OFFSET_BUFFER_SIZE);

        uint32_t signal_grid[64];
        int16_t  range_grid[64];
        memcpy(signal_grid, &buf[0x3C],  sizeof(signal_grid));
        memcpy(range_grid,  &buf[0x140], sizeof(range_grid));

        for (int8_t j = 0; j < 4; j++) {
            for (int8_t i = 0; i < 4; i++) {
                signal_grid[i + 4*j] =
                    (signal_grid[(2*i) + (16*j) + 0] +
                     signal_grid[(2*i) + (16*j) + 1] +
                     signal_grid[(2*i) + (16*j) + 8] +
                     signal_grid[(2*i) + (16*j) + 9]) / (uint32_t)4;
                range_grid[i + 4*j] =
                    (range_grid[(2*i) + (16*j) + 0] +
                     range_grid[(2*i) + (16*j) + 1] +
                     range_grid[(2*i) + (16*j) + 8] +
                     range_grid[(2*i) + (16*j) + 9]) / (int16_t)4;
            }
        }
        memset(&range_grid[0x10],  0, 96);
        memset(&signal_grid[0x10], 0, 192);
        memcpy(&buf[0x3C],  signal_grid, sizeof(signal_grid));
        memcpy(&buf[0x140], range_grid,  sizeof(range_grid));
        swap_buffer(buf, VL53L5CX_OFFSET_BUFFER_SIZE);
    }
    // 8x8: NVM data is already in native 8x8 format — no modification needed.

    // Shift buffer down 8 bytes (remove DCI header)
    for (uint16_t k = 0; k < VL53L5CX_OFFSET_BUFFER_SIZE - 4; k++) {
        buf[k] = buf[k + 8];
    }
    memcpy(&buf[0x1E0], footer, sizeof(footer));

    // Write at 0x2e18 and poll
    if (!write_block(0x2e18, buf, VL53L5CX_OFFSET_BUFFER_SIZE)) {
        return false;
    }
    hal.scheduler->delay(100);
    uint8_t st[4] = {};
    return read_block(UI_CMD_STATUS, st, 4) && st[0] == 0x03;
}

bool AP_RangeFinder_VL53L5CX::send_xtalk_data(uint8_t resolution)
{
    // Use _tmp2 as working buffer (VL53L5CX_XTALK_BUFFER_SIZE = 776 bytes)
    uint8_t *buf = _tmp2;
    memcpy(buf, VL53L5CX_DEFAULT_XTALK, VL53L5CX_XTALK_BUFFER_SIZE);

    if (resolution == RESOLUTION_4X4) {
        static const uint8_t res4x4[]     = {0x0F, 0x04, 0x04, 0x17, 0x08, 0x10, 0x10, 0x07};
        static const uint8_t dss_4x4[]    = {0x00, 0x78, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08};
        static const uint8_t profile_4x4[]= {0xA0, 0xFC, 0x01, 0x00};

        // ULD order: apply res4x4/dss_4x4 BEFORE the first SwapBuffer.
        memcpy(&buf[0x08], res4x4,  sizeof(res4x4));
        memcpy(&buf[0x20], dss_4x4, sizeof(dss_4x4));

        // Swap to host order for signal-grid arithmetic
        swap_buffer(buf, VL53L5CX_XTALK_BUFFER_SIZE);

        uint32_t signal_grid[64];
        memcpy(signal_grid, &buf[0x34], sizeof(signal_grid));

        for (int8_t j = 0; j < 4; j++) {
            for (int8_t i = 0; i < 4; i++) {
                signal_grid[i + 4*j] =
                    (signal_grid[(2*i) + (16*j) + 0] +
                     signal_grid[(2*i) + (16*j) + 1] +
                     signal_grid[(2*i) + (16*j) + 8] +
                     signal_grid[(2*i) + (16*j) + 9]) / (uint32_t)4;
            }
        }
        memset(&signal_grid[0x10], 0, 192);
        memcpy(&buf[0x34], signal_grid, sizeof(signal_grid));

        // Swap back to device order; apply remaining modifications (unswapped)
        swap_buffer(buf, VL53L5CX_XTALK_BUFFER_SIZE);
        memcpy(&buf[0x134], profile_4x4, sizeof(profile_4x4));
        memset(&buf[0x078], 0, 4);
    }

    // Ensure page 0x02 is selected (UI_CMD region lives there)
    //if (!write_byte(REG_PAGE, 0x02)) {
    //    return false;
    //}
    if (!write_block(0x2cf8, buf, VL53L5CX_XTALK_BUFFER_SIZE)) {
        return false;
    }
    // Diagnostic read-back: read first 4 bytes at 0x2CF8 to confirm the xtalk
    // data actually landed.  Appears in I2C trace.
    //   0x9F 0xD8 0x00 0xC0 = data stored on page 0x02 (trigger register is
    //                          write-only; firmware received the write)
    //   0x00 0x00 0x00 0x00 = data NOT stored (wrong page or write-protection)
    //uint8_t xt_dbg[4] = {};
    //read_block(0x2cf8, xt_dbg, 4);
    // Also read 0x2FFF to see if it's a write-only trigger (always returns 0x00)
    // or if the data simply didn't land (also 0x00 but different root cause).
    //uint8_t xt_end_chk = 0;
    //read_byte(UI_CMD_END, xt_end_chk);

    hal.scheduler->delay(100);
    // Read all 4 status bytes for diagnosis (not just byte[1])
    uint8_t st4[4] = {};
    read_block(UI_CMD_STATUS, st4, 4);
    return st4[1] == 0x03;
}

// -------------------------------------------------------------------------
// Initialisation
// -------------------------------------------------------------------------

bool AP_RangeFinder_VL53L5CX::init()
{
    // init can take up to ~6 seconds (firmware upload + polling loops).
    // Tell the scheduler so the timer thread keeps the watchdog fed.
    hal.scheduler->expect_delay_ms(5000);
    WITH_SEMAPHORE(dev.get_semaphore());

    // ------------------------------------------------------------------
    // 1. Verify device identity (vl53l5cx_is_alive equivalent)
    //    Register 0x0000 = device_id = 0xF0
    //    Register 0x0001 = revision_id = 0x02
    // ------------------------------------------------------------------
    uint8_t device_id, revision_id;
    if (!write_byte(REG_PAGE, 0x00) ||
        !read_byte(0x0000, device_id)  ||
        !read_byte(0x0001, revision_id)||
        !write_byte(REG_PAGE, 0x02)) {
        return false;
    }
    if (device_id != 0xF0 || revision_id != 0x02) {
        return false;
    }

    // ------------------------------------------------------------------
    // 2. SW reboot sequence (vl53l5cx_init phase 1)
    // ------------------------------------------------------------------
    if (!write_byte(REG_PAGE, 0x00) ||
        !write_byte(0x0009, 0x04)   ||
        !write_byte(0x000F, 0x40)   ||
        !write_byte(0x000A, 0x03)) {
        return false;
    }
    uint8_t tmp;
    if (!read_byte(REG_PAGE, tmp)) { return false; }

    if (!write_byte(0x000C, 0x01) ||
        !write_byte(0x0101, 0x00) ||
        !write_byte(0x0102, 0x00) ||
        !write_byte(0x010A, 0x01) ||
        !write_byte(0x4002, 0x01) ||
        !write_byte(0x4002, 0x00) ||
        !write_byte(0x010A, 0x03) ||
        !write_byte(0x0103, 0x01) ||
        !write_byte(0x000C, 0x00) ||
        !write_byte(0x000F, 0x43)) {
        return false;
    }
    hal.scheduler->delay(1);

    if (!write_byte(0x000F, 0x40) ||
        !write_byte(0x000A, 0x01)) {
        return false;
    }
    hal.scheduler->delay(100);

    // Wait for ROM boot (reg 0x06 == 0x01)
    if (!write_byte(REG_PAGE, 0x00)) { return false; }
    hal.scheduler->delay(500);
    uint8_t boot_st;
    if (!read_byte(REG_BOOT_STATUS, boot_st) || boot_st != 0x01) {
        return false;
    }

    // ------------------------------------------------------------------
    // 3. Enable FW access
    // ------------------------------------------------------------------
    if (!write_byte(0x000E, 0x01) ||
        !write_byte(REG_PAGE, 0x02) ||
        !write_byte(0x0003, 0x0D)   ||
        !write_byte(REG_PAGE, 0x01)) {
        return false;
    }
    hal.scheduler->delay(100);
    uint8_t fw_acc;
    if (!read_byte(0x0021, fw_acc) || (fw_acc & 0x10) != 0x10) {
        return false;
    }
    if (!write_byte(REG_PAGE, 0x00) ||
        !write_byte(0x000C, 0x01)) {
        return false;
    }

    // ------------------------------------------------------------------
    // 4. Power ON status
    // ------------------------------------------------------------------
    if (!write_byte(REG_PAGE, 0x00) ||
        !write_byte(0x0101, 0x00) ||
        !write_byte(0x0102, 0x00) ||
        !write_byte(0x010A, 0x01) ||
        !write_byte(0x4002, 0x01) ||
        !write_byte(0x4002, 0x00) ||
        !write_byte(0x010A, 0x03) ||
        !write_byte(0x0103, 0x01) ||
        !write_byte(0x400F, 0x00) ||
        !write_byte(0x021A, 0x43) ||
        !write_byte(0x021A, 0x03) ||
        !write_byte(0x021A, 0x01) ||
        !write_byte(0x021A, 0x00) ||
        !write_byte(0x0219, 0x00) ||
        !write_byte(0x021B, 0x00)) {
        return false;
    }

    // ------------------------------------------------------------------
    // 5. Wake MCU and download firmware
    //    84KB split into 3 pages: 0x09 (32KB), 0x0a (32KB), 0x0b (20KB)
    // ------------------------------------------------------------------
    if (!write_byte(REG_PAGE, 0x00) ||
        !write_byte(0x000C, 0x00)   ||
        !write_byte(REG_PAGE, 0x01) ||
        !write_byte(0x0020, 0x07)   ||
        !write_byte(0x0020, 0x06)) {
        return false;
    }
    hal.scheduler->expect_delay_ms(15000);  // 84KB upload takes ~9s at 100kHz

    if (!write_byte(REG_PAGE, 0x09) ||
        !write_block(0x0000, VL53L5CX_FIRMWARE, 0x8000)) {
        return false;
    }
    if (!write_byte(REG_PAGE, 0x0a) ||
        !write_block(0x0000, VL53L5CX_FIRMWARE + 0x8000, 0x8000)) {
        return false;
    }
    if (!write_byte(REG_PAGE, 0x0b) ||
        !write_block(0x0000, VL53L5CX_FIRMWARE + 0x10000, 0x5000)) {
        return false;
    }
    if (!write_byte(REG_PAGE, 0x01)) {
        return false;
    }
    hal.scheduler->expect_delay_ms(5000);

    // ------------------------------------------------------------------
    // 6. Verify FW download
    // ------------------------------------------------------------------
    if (!write_byte(REG_PAGE, 0x02) ||
        !write_byte(0x0003, 0x0D)   ||
        !write_byte(REG_PAGE, 0x01)) {
        return false;
    }
    hal.scheduler->delay(100);
    if (!read_byte(0x0021, fw_acc) || (fw_acc & 0x10) != 0x10) {
        return false;
    }
    if (!write_byte(REG_PAGE, 0x00) ||
        !write_byte(0x000C, 0x01)) {
        return false;
    }

    // ------------------------------------------------------------------
    // 7. Reset MCU and wait for firmware boot
    //    Matches ULD sequence exactly:
    //      WrByte(0x0B,0x00), WrByte(0x0C,0x00), WrByte(0x0B,0x01)
    //      poll 0x0006==0x00 on PAGE 0x00
    //      WrByte(0x7fff,0x02)   ← switch to page 0x02 AFTER poll
    //      WrMulti(0x2fd8, NVM_CMD, ...)  ← NVM directly, no 0x0003=0x09
    // ------------------------------------------------------------------
    if (!write_byte(REG_PAGE, 0x00) ||
        !write_byte(0x0114, 0x00)   ||
        !write_byte(0x0115, 0x00)   ||
        !write_byte(0x0116, 0x42)   ||
        !write_byte(0x0117, 0x00)   ||
        !write_byte(0x000B, 0x00)   ||
        !write_byte(0x000C, 0x00)   ||   // ULD writes 0x00 here, not a read
        !write_byte(0x000B, 0x01)) {
        return false;
    }
    // Poll 0x0006==0x00 on page 0x00 (matching ULD — poll is on page 0x00,
    // not page 0x02 as was incorrectly assumed before).
    {
        bool fw_booted = false;
        for (uint16_t i = 0; i < 300; i++) {
            if (read_byte(REG_BOOT_STATUS, boot_st) && (boot_st == 0x00)) {
                fw_booted = true;
                break;
            }
            hal.scheduler->delay(10);
        }
        if (!fw_booted) {
            return false;
        }
    }
    // Switch to page 0x02 AFTER the boot poll (ULD: WrByte(0x7fff,0x02)).
    // The ULD then goes straight to the NVM write — there is NO write of
    // 0x0003=0x09 in vl53l5cx_init.  Writing it was putting the firmware
    // into ranging mode which makes UI_CMD read-only.
    if (!write_byte(REG_PAGE, 0x02)) {
        return false;
    }
    hal.scheduler->expect_delay_ms(5000);

    // ------------------------------------------------------------------
    // 8. Read NVM offset calibration and send to firmware
    // ------------------------------------------------------------------
    // NVM step is non-fatal: ranging still works without factory offset data.
    // Poll at 10 ms intervals (matching ST ULD).
    bool nvm_ok = false;
    if (!write_block(0x2fd8, VL53L5CX_GET_NVM_CMD, sizeof(VL53L5CX_GET_NVM_CMD))) {
        return false;
    }
    for (uint8_t i = 0; i < 200; i++) {
        hal.scheduler->delay(10);
        uint8_t nvm_st[4] = {};
        if (read_block(UI_CMD_STATUS, nvm_st, 4) && (nvm_st[0] == 0x02)) {
            nvm_ok = true;
            break;
        }
    }
    if (!nvm_ok) {
        return false;
    }

    // Read NVM data into _tmp (492 bytes, fits in the 800-byte scratch buffer)
    if (!read_block(UI_CMD_START, _tmp, VL53L5CX_NVM_DATA_SIZE)) {
        return false;
    }
    // Stash first 488 bytes for reuse by set_resolution()
    memcpy(_nvm_buf, _tmp, VL53L5CX_OFFSET_BUFFER_SIZE);
    // Send offset calibration (first 488 bytes of NVM data)
    if (!send_offset_data(_nvm_buf, RESOLUTION_4X4)) {
        return false;
    }
    hal.scheduler->expect_delay_ms(5000);

    // ------------------------------------------------------------------
    // 9. Send default xtalk correction (non-fatal — calibration data may
    //    be absent on some units; ranging proceeds with default xtalk)
    // ------------------------------------------------------------------
    if (!send_xtalk_data(RESOLUTION_4X4)) {
        return false;
    }

    // ------------------------------------------------------------------
    // 10. Write default configuration, poll for acceptance
    // ------------------------------------------------------------------
    if (//!write_byte(REG_PAGE, 0x02) ||
        !write_block(0x2c34, VL53L5CX_DEFAULT_CONFIGURATION, VL53L5CX_CONFIGURATION_SIZE)) {
        return false;
    }
    // Diagnostic: read 0x2FFF immediately after the write to verify 0xC8
    // actually landed.  This appears in the I2C trace.
    //   0xC8 → write OK but firmware not reacting
    //   0x00 → wrong page or write-protection active
    //uint8_t cfg_end_chk = 0;
    //read_byte(UI_CMD_END, cfg_end_chk);
    bool cfg_ok = false;
    for (uint8_t i = 0; i < 200; i++) {
        hal.scheduler->delay(10);
        uint8_t cfg_st4[4] = {};
        if (read_block(UI_CMD_STATUS, cfg_st4, 4) && (cfg_st4[1] == 0x03)) {
            cfg_ok = true;
            break;
        }
    }
    if (!cfg_ok) {
        return false;
    }

    // ------------------------------------------------------------------
    // 11. DCI: pipe control and single-range mode
    // ------------------------------------------------------------------
    uint8_t pipe_ctrl[4] = {1, 0x00, 0x01, 0x00};  // NB_TARGET_PER_ZONE=1
    if (!dci_write(DCI_PIPE_CONTROL, pipe_ctrl, sizeof(pipe_ctrl))) {
        return false;
    }
    uint32_t single_range = 0x01;
    if (!dci_write(DCI_SINGLE_RANGE, (uint8_t *)&single_range, sizeof(single_range))) {
        return false;
    }
    hal.scheduler->expect_delay_ms(5000);

    // ------------------------------------------------------------------
    // 12. Configure and start ranging
    // ------------------------------------------------------------------
    if (!start_ranging()) {
        return false;
    }

    // Register periodic callback at ~15 Hz
    dev.register_periodic_callback(
        66000,
        FUNCTOR_BIND_MEMBER(&AP_RangeFinder_VL53L5CX::timer, void));

    return true;
}

// -------------------------------------------------------------------------
// set_resolution()
//
// Configures the sensor for 8x8 mode via DCI.
// Matches vl53l5cx_set_resolution() in the ST ULD.
// -------------------------------------------------------------------------
bool AP_RangeFinder_VL53L5CX::set_resolution()
{
    uint8_t dss_cfg[16] = {};
    if (!dci_read(DCI_DSS_CONFIG, dss_cfg, sizeof(dss_cfg))) {
        return false;
    }
    dss_cfg[0x04] = 16;
    dss_cfg[0x06] = 16;
    dss_cfg[0x09] = 1;
    if (!dci_write(DCI_DSS_CONFIG, dss_cfg, sizeof(dss_cfg))) {
        return false;
    }

    uint8_t zone_cfg[8] = {};
    if (!dci_read(DCI_ZONE_CONFIG, zone_cfg, sizeof(zone_cfg))) {
        return false;
    }
    zone_cfg[0x00] = 8;
    zone_cfg[0x01] = 8;
    zone_cfg[0x04] = 4;
    zone_cfg[0x05] = 4;
    if (!dci_write(DCI_ZONE_CONFIG, zone_cfg, sizeof(zone_cfg))) {
        return false;
    }

    // Send offset calibration using stashed NVM data
    if (!send_offset_data(_nvm_buf, RESOLUTION_8X8)) {
        return false;
    }

    if (!send_xtalk_data(RESOLUTION_8X8)) {
        return false;
    }

    return true;
}

// -------------------------------------------------------------------------
// start_ranging()
//
// Computes data_read_size for the selected output set, configures the
// sensor output list/config/enables via DCI, then issues the start command.
//
// Matches vl53l5cx_start_ranging() in the ST ULD.
// -------------------------------------------------------------------------

// Block-header union for local size computation
union BlockHeader {
    uint32_t raw;
    struct {
        uint32_t type :  4;
        uint32_t size : 12;
        uint32_t idx  : 16;
    };
};

bool AP_RangeFinder_VL53L5CX::start_ranging()
{
    if (!set_resolution()) {
        return false;
    }

    // Set ranging freq
    uint8_t freq_buf[4] = {};
    if (!dci_read(DCI_RANGING_FREQ, freq_buf, sizeof(freq_buf))) {
        return false;
    }
    freq_buf[1] = 30;
    if (!dci_write(DCI_RANGING_FREQ, freq_buf, sizeof(freq_buf))) {
        return false;
    }

    // Get the resolution from zone config (rows x cols)
    uint8_t res_buf[8] = {};
    if (!dci_read(DCI_ZONE_CONFIG, res_buf, sizeof(res_buf))) {
        return false;
    }
    uint8_t resolution = res_buf[0x00] * res_buf[0x01];

    // Full output list matching ST ULD example (12 entries)
    static const uint32_t output[] = {
        BH_START,
        BH_METADATA,
        BH_COMMONDATA,
        BH_AMBIENT_RATE,
        BH_SPAD_COUNT,
        BH_NB_TARGET_DETECTED,
        BH_SIGNAL_RATE,
        BH_RANGE_SIGMA_MM,
        BH_DISTANCE,
        BH_REFLECTANCE,
        BH_TARGET_STATUS,
        BH_MOTION_DETECT,
    };
    static constexpr uint8_t N_OUTPUT = ARRAY_SIZE(output);

    uint32_t output_bh_enable[4] = {
        (1U << N_OUTPUT) - 1U,  // enable all N_OUTPUT entries
        0, 0, 0xC0000000U
    };

    // Calculate data_read_size (matches the ULD loop)
    _data_read_size = 0;
    for (uint8_t i = 0; i < N_OUTPUT; i++) {
        if (output[i] == 0) { continue; }
        if ((output_bh_enable[i / 32] & (1U << (i % 32))) == 0) { continue; }

        union BlockHeader bh;
        bh.raw = output[i];

        uint32_t msize;
        if (bh.type >= 1 && bh.type <= 12) {
            bh.size = resolution;
            msize = (uint32_t)bh.type * bh.size;
        } else {
            msize = bh.size;
        }
        _data_read_size += msize + 4;
    }
    _data_read_size += 20;  // fixed header overhead

    // Write output list with sizes corrected for 4x4 resolution, matching the
    // ULD which modifies the array in-place before writing to DCI.
    uint32_t out_copy[N_OUTPUT];
    memcpy(out_copy, output, sizeof(out_copy));
    for (uint8_t i = 0; i < N_OUTPUT; i++) {
        union BlockHeader bh;
        bh.raw = out_copy[i];
        if (bh.type >= 1 && bh.type <= 12) {
            bh.size = resolution;
            out_copy[i] = bh.raw;
        }
    }
    if (!dci_write(DCI_OUTPUT_LIST, (uint8_t *)out_copy, sizeof(out_copy))) {
        return false;
    }

    // Write output config (data_read_size, n+1)
    uint32_t header_config[2] = {_data_read_size, N_OUTPUT + 1};
    if (!dci_write(DCI_OUTPUT_CONFIG, (uint8_t *)header_config, sizeof(header_config))) {
        return false;
    }

    // Write output enables
    if (!dci_write(DCI_OUTPUT_ENABLES, (uint8_t *)output_bh_enable, sizeof(output_bh_enable))) {
        return false;
    }

    // Enable xshut bypass (page 0x00)
    if (!write_byte(REG_PAGE, 0x00) ||
        !write_byte(0x0009, 0x05)   ||
        !write_byte(REG_PAGE, 0x02)) {
        return false;
    }

    // Issue start command: write 4 bytes to UI_CMD_END - 3.
    uint8_t cmd[4] = {0x00, 0x03, 0x00, 0x00};
    if (!write_block(UI_CMD_END - 3, cmd, sizeof(cmd))) {
        return false;
    }

    hal.scheduler->delay(50);
    uint8_t st[4] = {};
    return read_block(UI_CMD_STATUS, st, 4) && st[1] == 0x03;
}

// -------------------------------------------------------------------------
// Result reading
// -------------------------------------------------------------------------

bool AP_RangeFinder_VL53L5CX::read_distance(uint16_t &out_mm)
{
    if (_data_read_size == 0 || _data_read_size > sizeof(_tmp)) {
        return false;
    }

    // Read entire result stream from address 0x0000
    if (!read_block(0x0000, _tmp, _data_read_size)) {
        return false;
    }

    // Byte-swap each 4-byte group (sensor is big-endian)
    swap_buffer(_tmp, _data_read_size);

    // Parse block headers starting at byte 16 (skip 16-byte stream header)
    int16_t dist[RESOLUTION_8X8];
    uint8_t status[RESOLUTION_8X8];
    bool have_dist   = false;
    bool have_status = false;

    for (uint32_t i = 16; i < _data_read_size; ) {
        union BlockHeader bh;
        memcpy(&bh.raw, &_tmp[i], 4);
        i += 4;

        uint32_t msize;
        if (bh.type >= 1 && bh.type <= 12) {
            msize = (uint32_t)bh.type * bh.size;
        } else {
            msize = bh.size;
        }

        if (bh.idx == IDX_DISTANCE && msize >= sizeof(dist)) {
            memcpy(dist, &_tmp[i], sizeof(dist));
            have_dist = true;
        } else if (bh.idx == IDX_TARGET_STATUS && msize >= sizeof(status)) {
            memcpy(status, &_tmp[i], sizeof(status));
            have_status = true;
        }

        i += msize;
    }

    if (!have_dist || !have_status) {
        return false;
    }

    // Find the minimum valid distance in the four centre zones.
    // Raw distance is in 1/4 mm units; divide by 4 for mm.
    uint16_t best  = UINT16_MAX;
    bool     found = false;

    for (uint8_t zi = 0; zi < ARRAY_SIZE(CENTRE_ZONES); zi++) {
        const uint8_t z = CENTRE_ZONES[zi];
        if (!status_is_valid(status[z])) {
            continue;
        }
        if (dist[z] <= 0) {
            continue;
        }
        uint16_t d_mm = (uint16_t)((uint16_t)dist[z] / 4);
        if (d_mm < best) {
            best  = d_mm;
            found = true;
        }
    }

    if (!found) {
        return false;
    }
    out_mm = best;
    return true;
}

// -------------------------------------------------------------------------
// Periodic timer (runs in I2C thread context)
// -------------------------------------------------------------------------

void AP_RangeFinder_VL53L5CX::timer()
{
    // Check data ready: read 4 bytes from address 0x0000 and verify all
    // four status conditions (matches vl53l5cx_check_data_ready() in ULD)
    uint8_t hdr[4];
    if (!read_block(0x0000, hdr, sizeof(hdr))) {
        return;
    }
    if (hdr[0] == _streamcount ||
        hdr[0] == 0xFF         ||
        hdr[1] != 0x05         ||
        (hdr[2] & 0x05) != 0x05||
        (hdr[3] & 0x10) != 0x10) {
        return;
    }
    _streamcount = hdr[0];

    uint16_t distance_mm;
    if (!read_distance(distance_mm)) {
        return;
    }

    WITH_SEMAPHORE(_sem);
    _distance_mm  = distance_mm;
    _new_distance = true;
    state.last_reading_ms = AP_HAL::millis();
}

// -------------------------------------------------------------------------
// ArduPilot backend interface
// -------------------------------------------------------------------------

void AP_RangeFinder_VL53L5CX::update()
{
    WITH_SEMAPHORE(_sem);

    if (_new_distance) {
        state.distance_m = _distance_mm * 0.001f;
        _new_distance    = false;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 500) {
        set_status(RangeFinder::Status::NoData);
    }
}

constexpr uint8_t AP_RangeFinder_VL53L5CX::CENTRE_ZONES[];

#endif  // AP_RANGEFINDER_VL53L5CX_ENABLED
