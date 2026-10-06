-- Read the negotiated USB-PD current from an STUSB4500 sink controller over I2C.
--
-- RDO_REG_STATUS (0x91) holds the Request Data Object the chip sent to the source.
-- Its operating-current field is the negotiated current, read directly. (The
-- STUSB4500 has no measured-voltage register, so voltage is left to be measured
-- externally.)

-- ---------------------------------------------------------------------------
-- Configuration
-- ---------------------------------------------------------------------------
local I2C_BUS     = 0       -- external I2C bus number
local UPDATE_MS   = 100     -- how often to poll and report
local SEVERITY    = 6       -- MAV_SEVERITY_INFO for gcs:send_text

local PARAM_TABLE_KEY = 73
assert(param:add_table(PARAM_TABLE_KEY, "CHARGE_", 1), "Charger: could not add param table")

--[[
  // @Param: CHARGE_USB_NVM
  // @DisplayName: STUSB4500 NVM check
  // @Description: When 1, the script runs a one-time check of the STUSB4500 USB-PD sink NVM and, if REQ_SRC_CURRENT is not set, reprograms sector 4 to set it. The parameter is then saved to 0 so the check runs only once. Set back to 1 to re-run.
  // @Values: 0:Disabled,1:Run check
  // @User: Standard
--]]
assert(param:add_param(PARAM_TABLE_KEY, 1, "USB_NVM", 1))
local USB_NVM = Parameter("CHARGE_USB_NVM")

-- ---------------------------------------------------------------------------
-- STUSB4500 register map
-- ---------------------------------------------------------------------------
local RDO_REG_STATUS_0 = 0x91   -- negotiated Request Data Object (4 bytes, LE)
local CC_STATUS        = 0x11   -- Type-C Rp advertisement (valid without PD)


local usb = assert(i2c:get_device(I2C_BUS, 0x28), "STUSB4500: i2c get_device failed")
usb:set_retries(10)

-- ---------------------------------------------------------------------------
-- Low level helpers
-- ---------------------------------------------------------------------------

-- Read a little-endian 32-bit register. Returns a number, or nil on bus error.
local function read_u32(reg)
  local t = usb:transfer(string.pack("B", reg), 4)
  if not t then
    return nil
  end
  local val = string.unpack("l", t)
  return uint32_t(val)
end

-- Source current advertised via the Type-C Rp resistor. This is valid even
-- when there is no USB-PD contract (e.g. plain USB 2.0 / legacy sources).
-- Returns amps, or nil on bus error.
local function typec_advertised_A()
  local cc = usb:read_registers(CC_STATUS)
  if cc == nil then
    return nil
  end
  local CONNECT_RESULT = (cc >> 4) & 0x01
  if CONNECT_RESULT ~= 1 then
    return 0
  end

  local CC1 = cc & 0x03
  local CC2 = (cc >> 2) & 0x03

  local state = CC1
  if state == 0 then
    state = CC2
  end
  if state == 0x01 then
    return 0.5 -- USB default (500mA for USB 2.0; chip can't tell 500 vs 900)
  elseif state == 0x02 then
    return 1.5
  elseif state == 0x03 then
    return 3.0
  end
  return 0
end

-- ---------------------------------------------------------------------------
-- NVM (FTP) access and field decode
-- ---------------------------------------------------------------------------
local FTP_CUST_PASSWORD_REG = 0x95
local FTP_CUST_PASSWORD     = 0x47
local FTP_CTRL_0            = 0x96
local FTP_CTRL_1            = 0x97
local RW_BUFFER             = 0x53

local FTP_CUST_RST_N = 0x40
local FTP_CUST_REQ   = 0x10
local FTP_CUST_SECT  = 0x07
local FTP_CUST_SER   = 0xF8   -- FTP_CTRL_1[7:3] sector select (for erase)
local READ_OPCODE    = 0x00
local WRITE_PL       = 0x01   -- shift data into program-load register
local WRITE_SER      = 0x02   -- shift data into sector-erase register
local ERASE_SECTOR   = 0x05   -- erase the selected sectors
local PROG_SECTOR    = 0x06   -- program the selected sector
local SOFT_PROG      = 0x07   -- soft-program (run after erase)

-- Unlock the NVM: password + power-up/reset pulse. Shared by the read and write
-- paths. This step alone causes no NVM wear. Returns true on success.
local function nvm_enable()
    -- Enter password
    if not usb:write_register(FTP_CUST_PASSWORD_REG, FTP_CUST_PASSWORD) then
        return false
    end

    -- Clear buffer
    if not usb:write_register(RW_BUFFER, 0x00) then
        return false
    end

    -- reset_n
    if not usb:write_register(FTP_CTRL_0, FTP_CUST_RST_N) then
        return false
    end

    -- reset controller
    if not usb:write_register(FTP_CTRL_0, 0x00) then
        return false
    end

    -- reset_n
    if not usb:write_register(FTP_CTRL_0, FTP_CUST_RST_N) then
        return false
    end

    return true
end

-- Leave NVM test mode and clear the password. Always call this when done.
local function nvm_exit()
  usb:write_register(FTP_CTRL_0, FTP_CUST_RST_N)
  usb:write_register(FTP_CTRL_1, 0x00)
  usb:write_register(FTP_CUST_PASSWORD_REG, 0x00)
end

-- Wait for the FTP controller to clear REQ (operation complete). true on success.
local function nvm_wait_req()
    local tries = 100
    while tries > 0 do
        local c = usb:read_registers(FTP_CTRL_0)
        if c == nil then
            return false
        end
        if (c & FTP_CUST_REQ) == 0 then
            return true
        end
        tries = tries - 1
    end
    return false
end

-- Read one 8-byte NVM sector as a string. Assumes nvm_enable() succeeded.
-- Returns an 8-byte string (index with string.byte / string.unpack) or nil.
local function nvm_read_sector(sector)
  -- issue READ opcode for this sector
  usb:write_register(FTP_CTRL_1, READ_OPCODE)
  usb:write_register(FTP_CTRL_0,
      (sector & FTP_CUST_SECT) | FTP_CUST_RST_N | FTP_CUST_REQ)

  -- poll until REQ clears (operation complete)
  if not nvm_wait_req() then
    return nil
  end

  return usb:transfer(string.pack("B", RW_BUFFER), 8)
end

-- 4-bit sink-PDO current code -> amps (code 0 means "use the flexible current")
local I_SNK_LUT = {
  [0] = 0.00, [1] = 0.50, [2] = 0.75, [3]  = 1.00, [4]  = 1.25, [5]  = 1.50,
  [6] = 1.75, [7] = 2.00, [8] = 2.25, [9]  = 2.50, [10] = 2.75, [11] = 3.00,
  [12] = 3.50, [13] = 4.00, [14] = 4.50, [15] = 5.00,
}
local GPIO_CFG_NAME = { [0] = "SW_CTRL", [1] = "ERROR_RECOVERY", [2] = "DEBUG", [3] = "SINK_POWER" }
local POWER_OK_NAME = { [0] = "CONFIG_1", [1] = "reserved", [2] = "CONFIG_2", [3] = "CONFIG_3" }

local function yn(v)
    if v ~= 0 then
        return "yes"
    end
    return "no"
end

-- Decode and print the named config fields for one NVM sector (per ST TN1.2).
-- Byte locals are named for their NVM address (da = 0xDA, e0 = 0xE0, ...).
local function nvn_decode(sec, s)
    -- Sectors 0 (IDs) and 2 are fully reserved: nothing can be changed
    if sec == 0 or sec == 2 then
        return
    end

    if sec == 1 then
        local C8, C9, CA = string.unpack("BBB", s)
        local GPIO_CFG = (C8 >> 4) & 0x03
        local VBUS_DCHG_MASK = (C9 >> 5) & 0x01
        local DISCHARGE_TIME_TO_0V = (CA >> 4) & 0x0F
        local VBUS_DISCH_TIME_TO_PDO = CA  & 0x0F

        gcs:send_text(SEVERITY, string.format(
            "GPIO %s VBUS_DCHG_MASK %s disch 0V=%d PDO=%d",
            GPIO_CFG_NAME[GPIO_CFG],
            yn(VBUS_DCHG_MASK),
            DISCHARGE_TIME_TO_0V,
            VBUS_DISCH_TIME_TO_PDO
        ))

    elseif sec == 3 then
        -- bank 3 (0xD8-0xDF): PDO currents, per-PDO voltage windows, options.
        -- Read from 0xDA (position 3): da, db, dc, dd, de.
        local DA, DB, DC, DD, DE = string.unpack("BBBBB", s, 3)

        local SNK_UNCONS_POWER = (DA >> 3) & 0x01
        local DPM_SNK_PDO_NUMB = (DA >> 1) & 0x03
        local USB_COMM_CAPABLE = DA & 0x01

        gcs:send_text(SEVERITY, string.format(
            "UNCONS_PWR %s PDO count %d  USB_COMM %s",
            yn(SNK_UNCONS_POWER),
            DPM_SNK_PDO_NUMB,
            yn(USB_COMM_CAPABLE)
        ))

        local LUT_SNK_PDO1_I = (DA >> 4) & 0x0F
        local LUT_SNK_PDO2_I = DC & 0x0F
        local LUT_SNK_PDO3_I = (DD >> 4) & 0x0F

        local SNK_HL1 = (DB >> 4) & 0x0F
        local SNK_HL2 = DD & 0x0F
        local SNK_HL3 = (DE >> 4) & 0x0F

        local SNK_LL1 = DB & 0x0F
        local SNK_LL2 = (DC >> 4) & 0x0F
        local SNK_LL3 = DE & 0x0F

        gcs:send_text(SEVERITY, string.format(
            "PDO1 %.2fA  -%d/+%d%%",
            I_SNK_LUT[LUT_SNK_PDO1_I],
            SNK_LL1,
            SNK_HL1
        ))

        gcs:send_text(SEVERITY, string.format(
            "PDO2 %.2fA  -%d/+%d%%",
            I_SNK_LUT[LUT_SNK_PDO2_I],
            SNK_LL2,
            SNK_HL2
        ))

        gcs:send_text(SEVERITY, string.format(
            "PDO3 %.2fA  -%d/+%d%%",
            I_SNK_LUT[LUT_SNK_PDO3_I],
            SNK_LL3,
            SNK_HL3
        ))

    elseif sec == 4 then
        -- bank 4 (0xE0-0xE7): PDO2/PDO3 voltages, flex current, flags.
        -- ST names the PDO2/PDO3 voltages SNK_PDO_FLEX1_V / SNK_PDO_FLEX2_V.
        local E0, E1, E2, E3, E4, E6 = string.unpack("BBBBBxB", s)  -- e5 spare, e7 alert mask

        local SNK_PDO_FLEX1_V = (((E0 >> 6) & 0x03) | (E1 << 2)) * 0.05
        local SNK_PDO_FLEX2_V = (((E3 & 0x03) << 8) | E2) * 0.05
        local SNK_PDO_FLEX_I = ((E3 >> 2) | ((E4 & 0x0F) << 6)) * 0.01

        gcs:send_text(SEVERITY, string.format(
            "PDO2 %.2fV  PDO3 %.2fV Flex %.2fA",
            SNK_PDO_FLEX1_V, SNK_PDO_FLEX2_V, SNK_PDO_FLEX_I))

        local POWER_OK_CFG = (E4 >> 5) & 0x03
        local REQ_SRC_CURRENT = (E6 >> 4) & 0x01
        local POWER_ONLY_ABOVE_5V = (E6 >> 3) & 0x01

        gcs:send_text(SEVERITY, string.format(
            "POWER_OK %s  REQ_SRC %s  >5V_only %s",
            POWER_OK_NAME[POWER_OK_CFG],
            yn(REQ_SRC_CURRENT),
            yn(POWER_ONLY_ABOVE_5V)
        ))
    end
end

-- Print all 5 NVM sectors as raw hex. One-shot diagnostic.
local function nvm_dump()
    if not nvm_enable() then
        gcs:send_text(SEVERITY, "NVM: enable failed")
        return
    end
    for sec = 0, 4 do
        local s = nvm_read_sector(sec)
        if s == nil then
            gcs:send_text(SEVERITY, string.format("NVM %d: read failed", sec))
        else
            gcs:send_text(SEVERITY, string.format(
                "NVM %d: %02X %02X %02X %02X %02X %02X %02X %02X",
                sec,
                string.byte(s, 1),
                string.byte(s, 2),
                string.byte(s, 3),
                string.byte(s, 4),
                string.byte(s, 5),
                string.byte(s, 6),
                string.byte(s, 7),
                string.byte(s, 8)
            ))

            nvn_decode(sec, s)
        end
    end
    nvm_exit()
end

-- Enter NVM WRITE mode and erase the sectors selected by `mask`
local function nvm_enter_write(mask)
    if not nvm_enable() then
        return false
    end

    -- Shift the sector-erase selection in
    usb:write_register(FTP_CTRL_1, ((mask << 3) & FTP_CUST_SER) | WRITE_SER)
    usb:write_register(FTP_CTRL_0, FTP_CUST_RST_N | FTP_CUST_REQ)
    if not nvm_wait_req() then
        return false
    end

    -- Soft-program
    usb:write_register(FTP_CTRL_1, SOFT_PROG)
    usb:write_register(FTP_CTRL_0, FTP_CUST_RST_N | FTP_CUST_REQ)
    if not nvm_wait_req() then
        return false
    end

    -- Erase
    usb:write_register(FTP_CTRL_1, ERASE_SECTOR)
    usb:write_register(FTP_CTRL_0, FTP_CUST_RST_N | FTP_CUST_REQ)
    if not nvm_wait_req() then
        return false
    end

    return true
end

-- Program one sector (0-4) from an 8-byte string. Assumes nvm_enter_write() erased
-- it first. Follows ST TN1.2 CUST_WriteSector.
local function nvm_write_sector(sector, data)
    -- load 8 data bytes into RW_BUFFER (write 0x53 then 8 bytes, auto-increment)
    if not usb:transfer(string.char(RW_BUFFER) .. data, 0) then
        return false
    end

    -- shift into the program-load register
    usb:write_register(FTP_CTRL_1, WRITE_PL)
    usb:write_register(FTP_CTRL_0, FTP_CUST_RST_N | FTP_CUST_REQ)
    if not nvm_wait_req() then
        return false
    end

    -- program the sector
    usb:write_register(FTP_CTRL_1, PROG_SECTOR)
    usb:write_register(FTP_CTRL_0, (sector & FTP_CUST_SECT) | FTP_CUST_RST_N | FTP_CUST_REQ)
    if not nvm_wait_req() then
        return false
    end

    return true
end

-- ---------------------------------------------------------------------------
-- Desired sink PDO configuration (priority is by PDO index: PDO3 > PDO2 > PDO1)
--   PDO3 (1st) = 9V @ 3A, PDO2 (2nd) = 15V @ 2A, PDO1 = fixed 5V, 3 PDOs.
-- Voltages are stored in 50mV units; currents as I_SNK_LUT index codes.
-- ---------------------------------------------------------------------------
local CFG_PDO_NUMB = 3
local CFG_PDO2_V   = 300   -- 15.0V / 0.05  -> SNK_PDO_FLEX1_V
local CFG_PDO3_V   = 400   -- 20.0V / 0.05  -> SNK_PDO_FLEX2_V
local CFG_PDO2_I   = 7     -- I_SNK_LUT[7]  = 2.0A -> LUT_SNK_PDO2_I
local CFG_PDO3_I   = 11    -- I_SNK_LUT[11] = 3.0A -> LUT_SNK_PDO3_I

-- Apply the PDO count and PDO2/PDO3 currents to an 8-byte sector-3 image.
local function patch_s3(s)
    local D8, D9, DA, DB, DC, DD, DE, DF = string.byte(s, 1, 8)
    DA = (DA & ~0x06) | ((CFG_PDO_NUMB & 0x03) << 1)   -- DPM_SNK_PDO_NUMB (DA 2:1)
    DC = (DC & ~0x0F) |  (CFG_PDO2_I  & 0x0F)          -- LUT_SNK_PDO2_I   (DC 3:0)
    DD = (DD & ~0xF0) | ((CFG_PDO3_I  & 0x0F) << 4)    -- LUT_SNK_PDO3_I   (DD 7:4)
    return string.char(D8, D9, DA, DB, DC, DD, DE, DF)
end

-- Apply PDO2/PDO3 voltages and REQ_SRC_CURRENT to an 8-byte sector-4 image.
local function patch_s4(s)
    local E0, E1, E2, E3, E4, E5, E6, E7 = string.byte(s, 1, 8)
    -- SNK_PDO_FLEX1_V (PDO2 V): low 2 bits in E0[7:6], upper 8 bits in E1
    E0 = (E0 & 0x3F) | ((CFG_PDO2_V & 0x03) << 6)
    E1 = (CFG_PDO2_V >> 2) & 0xFF
    -- SNK_PDO_FLEX2_V (PDO3 V): low 8 bits in E2, upper 2 bits in E3[1:0]
    E2 = CFG_PDO3_V & 0xFF
    E3 = (E3 & ~0x03) | ((CFG_PDO3_V >> 8) & 0x03)
    -- REQ_SRC_CURRENT: E6 (0xE6) bit 4
    E6 = E6 | (1 << 4)
    return string.char(E0, E1, E2, E3, E4, E5, E6, E7)
end

-- Verify/program the sink PDOs (sectors 3 & 4) plus REQ_SRC_CURRENT. Only the
-- sectors that differ from the desired configuration are erased/written, then
-- verified. No write happens if the NVM already matches.
local function nvn_check()
    if not nvm_enable() then
        gcs:send_text(SEVERITY, "NVM: enable failed")
        return false
    end
    local s3 = nvm_read_sector(3)
    local s4 = nvm_read_sector(4)
    nvm_exit()
    if s3 == nil or s4 == nil then
        gcs:send_text(SEVERITY, "NVM: read failed")
        return false
    end

    local new_s3 = patch_s3(s3)
    local new_s4 = patch_s4(s4)
    local wr3 = new_s3 ~= s3
    local wr4 = new_s4 ~= s4

    if not wr3 and not wr4 then
        gcs:send_text(SEVERITY, "NVM: PDOs configured correctly")
        return true
    end

    -- partial-erase only the sectors that changed (sector N -> mask bit N)
    local mask = (wr3 and (1 << 3) or 0) | (wr4 and (1 << 4) or 0)
    if not nvm_enter_write(mask) then
        nvm_exit()
        gcs:send_text(SEVERITY, "NVM: erase failed")
        return false
    end
    if wr3 and not nvm_write_sector(3, new_s3) then
        nvm_exit()
        gcs:send_text(SEVERITY, "NVM: program sector 3 failed")
        return false
    end
    if wr4 and not nvm_write_sector(4, new_s4) then
        nvm_exit()
        gcs:send_text(SEVERITY, "NVM: program sector 4 failed")
        return false
    end

    local v3 = wr3 and nvm_read_sector(3) or nil
    local v4 = wr4 and nvm_read_sector(4) or nil
    nvm_exit()

    if (wr3 and v3 ~= new_s3) or (wr4 and v4 ~= new_s4) then
        gcs:send_text(SEVERITY, "NVM: verify failed")
        return false
    end

    gcs:send_text(SEVERITY, "NVM: PDOs updated")
    return true
end

local last_limit
local last_is_pd
local bq_set_input_limit   -- forward declaration; defined in the BQ25798 section
local function update_current_limit(is_pd, limit)

    if (is_pd == last_is_pd) and (limit == last_limit) then
        -- No change
        return
    end
    last_is_pd = is_pd
    last_limit = limit

    -- match the charger input current limit to the negotiated source current
    bq_set_input_limit(limit)

    local pd_string = ""
    if is_pd then
        pd_string = "PD "
    end

    gcs:send_text(SEVERITY, string.format("USB: %s%0.2fA", pd_string, limit))
end

-- ---------------------------------------------------------------------------
-- BQ25798 charger: publish input & output to scripting battery monitors
-- ---------------------------------------------------------------------------
local BQ_ADDR     = 0x6B
local BQ_CHG_STAT_0 = 0x1B -- Charger Status 0 (PG/VINDPM/IINDPM/POORSRC/WD)
local BQ_CHG_STAT_2 = 0x1C
local BQ_CHG_CTRL_1 = 0x10 -- Charger Control 1 (watchdog)
local BQ_WD_RST     = 0x08 -- REG10 bit3: write 1 to reset (pet) the watchdog
local BQ_WD_RATE    = 0x03 -- REG10 bits 2:0 watchdog rate (2s)
local BQ_CHG_CTRL_5 = 0x14 -- Charger Control 5
local BQ_EN_IBAT    = 0x20 -- REG14 bit5: enable battery current sensing
local BQ_EN_EXTILIM = 0x02 -- REG14 bit1: ILIM_HIZ pin input-current clamp
local BQ_ADC_CTRL = 0x2E   -- bit7 ADC_EN, bit6 0=continuous
local BQ_IBUS_ADC = 0x31   -- s16, 1mA/LSB  input current
local BQ_IBAT_ADC = 0x33   -- s16, 1mA/LSB  battery/charge current
local BQ_VBUS_ADC = 0x35   -- u16, 1mV/LSB  input voltage
local BQ_VBAT_ADC = 0x3B   -- u16, 1mV/LSB  battery voltage
local BQ_VSYS_ADC = 0x3D   -- u16, 1mV/LSB  system voltage
local BQ_IINDPM   = 0x06   -- REG06 input current limit (IINDPM, 10mA/LSB)
local BQ_TDIE_ADC = 0x41   -- s16, 0.5C/LSB die temperature
local BQ_TS_ADC   = 0x3F   -- u16, TS pin as 1/1024 of REGN (10k NTC)
local BQ_TERM_CTRL = 0x09  -- REG09 (bit6 REG_RST, bit5 STOP_WD_CHG)
local BQ_REG_RST   = 0x40  -- REG09 bit6: reset all registers to POR defaults

local bq = assert(i2c:get_device(I2C_BUS, BQ_ADDR), "BQ25798: i2c get_device failed")
bq:set_retries(10)

-- Reset the charger to a known POR state on connect (REG09 bit6 = REG_RST),
-- clearing any autonomous state from before the autopilot booted. The ADC /
-- EN_IBAT config below is applied on top of the freshly reset defaults.
assert(bq:write_register(BQ_TERM_CTRL, BQ_REG_RST), "BQ25798: reset failed")
-- REG_RST self-clears when the reset completes; wait for that before configuring
local reset_done = false
for _ = 1, 100 do
    local r = bq:read_registers(BQ_TERM_CTRL)
    if r ~= nil and (r & BQ_REG_RST) == 0 then
        reset_done = true
        break
    end
end
assert(reset_done, "BQ25798: reset did not complete")

assert(bq:write_register(BQ_ADC_CTRL, 0x80), "BQ25798: ADC setup failed")
-- REG14: set EN_IBAT (bit5, else IBAT ADC reads 0) and clear EN_EXTILIM (bit1)
-- so the IINDPM register alone sets the input current limit (no ILIM_HIZ clamp).
-- Read-modify-write to preserve the other Charger Control 5 fields.
local bq_cc5 = bq:read_registers(BQ_CHG_CTRL_5)
assert(bq_cc5 ~= nil, "BQ25798: read Charger Control 5 failed")
assert(bq:write_register(BQ_CHG_CTRL_5, (bq_cc5 | BQ_EN_IBAT) & ~BQ_EN_EXTILIM),
    "BQ25798: Charger Control 5 set failed")

-- REG12 Charger Control 3: force continuous PWM in forward (charge) mode so the
-- switching matches the datasheet CCM waveforms:
--   PFM_FWD_DIS (bit4) - disable forward PFM (no pulse-skipping at light load)
--   DIS_FWD_OOA (bit0) - disable forward out-of-audio mode
local BQ_CHG_CTRL_3 = 0x12
local BQ_PFM_FWD_DIS = 0x10
local BQ_DIS_FWD_OOA = 0x01
local bq_cc3 = bq:read_registers(BQ_CHG_CTRL_3)
assert(bq_cc3 ~= nil, "BQ25798: read Charger Control 3 failed")
assert(bq:write_register(BQ_CHG_CTRL_3, bq_cc3 | BQ_PFM_FWD_DIS | BQ_DIS_FWD_OOA),
    "BQ25798: Charger Control 3 set failed")

-- Disable D+/D- Detection (there not connected)
local BQ_CHG_CTRL_2 = 0x11
assert(bq:write_register(BQ_CHG_CTRL_2, 0x00),
    "BQ25798: Charger Control 2 set failed")

-- Read a 16-bit big-endian ADC register (signed = two's complement).
-- Returns the raw value, or nil on bus error.
local function bq_read16(reg, signed)
  local t = bq:transfer(string.pack("B", reg), 2)
  if not t then
    return nil
  end
  return string.unpack(signed and ">i2" or ">I2", t)
end

-- Set the charger input current limit (REG06 IINDPM) from a source current in
-- amps, clamped to the 100mA-3300mA register range (10mA/LSB, 16-bit big-endian).
function bq_set_input_limit(amps)
    local reg = math.floor((amps * 100) + 0.5)   -- amps -> 10mA units
    if reg < 10 then reg = 10 end               -- 100mA minimum
    if reg > 330 then reg = 330 end             -- 3300mA maximum
    bq:transfer(string.char(BQ_IINDPM, (reg >> 8) & 0xFF, reg & 0xFF), 0)
end

local charge_state = BattMonitorScript_State()
local usb_state = BattMonitorScript_State()

-- CHG_STAT (REG1C bits 7:5) charge-state names
local CHG_STAT_NAME = {
    [0] = "Not Charging",
    [1] = "Trickle Charge",
    [2] = "Pre-Charge",
    [3] = "Fast Charge",
    [4] = "Taper",
    [6] = "Top-off",
    [7] = "Charge Done",
}
local last_chg_stat = 0
local last_vbus_stat = 0
local last_diag_ms  = uint32_t(0)

-- TS pin 103AT 10k NTC network (datasheet Fig 7-12): RT1 from REGN to TS,
-- RT2 in parallel with the NTC from TS to GND.
local TS_RT1  = 5230     -- REGN -> TS (ohms)
local TS_RT2  = 30000    -- TS -> GND, parallel with NTC (ohms)
local TS_R25  = 10000    -- NTC resistance at 25C
local TS_BETA = 3435     -- NTC beta (103AT)

-- Read the TS pin NTC and return the battery temperature in C, or nil.
local function bq_ts_temp()
    local raw = bq_read16(BQ_TS_ADC, false)
    if raw == nil then
        return nil
    end
    local p = raw / 1024.0             -- TS voltage as a fraction of REGN
    if p <= 0.0 or p >= 1.0 then
        return nil                     -- shorted or open thermistor
    end
    local rp = p * TS_RT1 / (1.0 - p)  -- RT2 || NTC
    local g = 1.0 / rp - 1.0 / TS_RT2
    if g <= 0.0 then
        return nil                     -- NTC out of range (very cold)
    end
    local ntc = 1.0 / g
    local tk = 1.0 / (1.0 / 298.15 + math.log(ntc / TS_R25) / TS_BETA)
    return tk - 273.15
end

-- Read the BQ25798 ADCs and publish input/output to the battery backends.
local function bq_update()
    -- Pat the watchdog
    bq:write_register(BQ_CHG_CTRL_1, 0x80 | BQ_WD_RST | BQ_WD_RATE)

    local charge_status = bq:read_registers(BQ_CHG_STAT_2)
    if charge_status == nil then
        return
    end

    bq_set_input_limit(3.0)

    --bq:write_register(0x0F, 0xA2)

    -- print the charge state (CHG_STAT, bits 7:5) when it changes
    local chg_stat = (charge_status >> 5) & 0x07
    if chg_stat ~= last_chg_stat then
        last_chg_stat = chg_stat
        gcs:send_text(SEVERITY, "Charger: " .. CHG_STAT_NAME[chg_stat])
    end

    local vbus_stat = (charge_status >> 1) & 0x0F
    if vbus_stat ~= last_vbus_stat then
        last_vbus_stat = vbus_stat
        gcs:send_text(SEVERITY, string.format("VBus: 0x%02X", vbus_stat))
    end

    -- diagnostic: while charging, report regulation loops, ICHG and JEITA state
    local diag_now = millis()
    if (diag_now - last_diag_ms) > uint32_t(5000) then
        last_diag_ms = diag_now
        local st0 = bq:read_registers(BQ_CHG_STAT_0) or 0   -- REG1B: IINDPM(7) VINDPM(6)
        local st2 = bq:read_registers(0x1D) or 0            -- REG1D: TREG(2)
        local st3 = bq:read_registers(0x1E) or 0            -- REG1E: VSYS(4)
        local st4 = bq:read_registers(0x1F) or 0            -- REG1F: TS cool(2) warm(1)
        local ichg = (bq_read16(0x03, false) or 0) & 0x1FF  -- REG03/04 ICHG, 10mA/LSB
        gcs:send_text(SEVERITY, string.format(
            "Chg%d VDPM%d IDPM%d TREG%d VSYS%d ICHG%d c%d w%d",
            chg_stat,
            (st0 >> 6) & 1, (st0 >> 7) & 1, (st2 >> 2) & 1, (st3 >> 4) & 1,
            ichg * 10, (st4 >> 2) & 1, (st4 >> 1) & 1))

        local ICO_STAT = (st2 >> 6) & 0x03
        print(string.format("ICO: %d", ICO_STAT))

        -- REG12 Charger Control 3: confirm PFM_FWD_DIS (bit4) + DIS_FWD_OOA (bit0)
        local d_r12 = bq:read_registers(0x12) or 0
        gcs:send_text(SEVERITY, string.format("R12=0x%02X", d_r12))

        --local VINDPM = bq:read_registers(0x05)
        --print(string.format("VINDPM: %0.01f", VINDPM * 0.1))

        -- input vs output power flow
        local d_vbus = (bq_read16(BQ_VBUS_ADC, false) or 0) * 0.001
        local d_ibus = (bq_read16(BQ_IBUS_ADC, true) or 0) * 0.001
        local d_vbat = (bq_read16(BQ_VBAT_ADC, false) or 0) * 0.001
        local d_ibat = (bq_read16(BQ_IBAT_ADC, true) or 0) * 0.001
        gcs:send_text(SEVERITY, string.format(
            "in %0.2fV %0.2fA  out %0.2fV %0.2fA", d_vbus, d_ibus, d_vbat, d_ibat))

        local d_iindpm = (bq_read16(BQ_IINDPM, false) or 0) & 0x1FF  -- REG06 IINDPM
        local d_ico    = (bq_read16(0x19, false) or 0) & 0x1FF       -- REG19 ICO_ILIM (effective)
        gcs:send_text(SEVERITY, string.format(
            "IINDPM %0.2fA ICO %0.2fA", d_iindpm * 0.01, d_ico * 0.01))

        local d_vsys = (bq_read16(BQ_VSYS_ADC, false) or 0) * 0.001           -- REG3D VSYS ADC
        local d_vreg = ((bq_read16(0x01, false) or 0) & 0x7FF) * 0.01        -- REG01/02 VREG (10mV/LSB)
        local d_r14  = bq:read_registers(BQ_CHG_CTRL_5) or 0         -- REG14 EN_IBAT bit5 / EN_EXTILIM bit1
        local d_r2f  = bq:read_registers(0x2F) or 0                  -- REG2F ADC disables (bit6 = IBAT_ADC_DIS)
        local d_vindpm = (bq:read_registers(0x05) or 0) * 0.1        -- REG05 VINDPM (100mV/LSB)
        gcs:send_text(SEVERITY, string.format(
            "VSYS %0.2fV VREG%0.2fV VINDPM %0.2fV R14=0x%02X R2F=0x%02X", d_vsys, d_vreg, d_vindpm, d_r14, d_r2f))

        -- fault registers (raw hex): FAULT_STATUS_0/1 (0x20/0x21), FAULT_FLAG_0/1 (0x22/0x23)
        local d_fs0 = bq:read_registers(0x20) or 0
        local d_fs1 = bq:read_registers(0x21) or 0
        local d_ff0 = bq:read_registers(0x22) or 0
        local d_ff1 = bq:read_registers(0x23) or 0
        gcs:send_text(SEVERITY, string.format(
            "FLT S0 0x%02X S1 0x%02X F0 0x%02X F1 0x%02X", d_fs0, d_fs1, d_ff0, d_ff1))
    end

    -- charger output = the battery being charged (VBAT/IBAT), NTC temperature
    local vbat = bq_read16(BQ_VBAT_ADC, false)
    local ibat = bq_read16(BQ_IBAT_ADC, true)
    local bat_valid = (vbat ~= nil) and (ibat ~= nil)
    charge_state:healthy(bat_valid)
    if bat_valid then
        charge_state:voltage(vbat * 0.001)
        charge_state:current_amps(ibat * -0.001)
    end
    local batt_temp = bq_ts_temp()
    if batt_temp ~= nil then
        charge_state:temperature(batt_temp)
    end
    battery:handle_scripting(1, charge_state)


    -- charger input = the source feeding the charger (VBUS/IBUS)
    local vbus = bq_read16(BQ_VBUS_ADC, false)
    local ibus = bq_read16(BQ_IBUS_ADC, true)
    local bus_valid = (vbus ~= nil) and (ibus ~= nil)
    usb_state:healthy(bus_valid)
    if bus_valid then
        usb_state:voltage(vbus * 0.001)
        usb_state:current_amps(ibus * 0.001)
    end
    local tdie = bq_read16(BQ_TDIE_ADC, true)
    if tdie ~= nil then
        usb_state:temperature(tdie * 0.5)
    end
    battery:handle_scripting(2, usb_state)

end

-- ---------------------------------------------------------------------------
-- Main loop
-- ---------------------------------------------------------------------------
local current_mask = uint32_t(0x3FF)
local last_inactive_ms = millis()
local function update()
    -- BQ25798 input/output monitoring runs regardless of USB-PD state
    bq_update()

    local now_ms = millis()
    local status = read_u32(RDO_REG_STATUS_0)
    if status == nil then
        -- The STUSB4500 is powered by the USB, so if USB is not present then it is expected to fail to read
        last_inactive_ms = now_ms
        last_limit = nil
        last_is_pd = nil
        return update, UPDATE_MS
    end

    -- Check NVM if set
    if USB_NVM:get() == 1 then
        if nvn_check() then
            USB_NVM:set_and_save(0)
        end
        nvm_dump()
        return update, UPDATE_MS
    end

    -- Wait at least 500ms for PD negotiation to complete
    if now_ms - last_inactive_ms < uint32_t(500) then
        return update, UPDATE_MS
    end

    -- Object position (bits 30:28) is non-zero only when a PD contract exists.
    local obj = ((status >> 28) & 0x07):toint()
    if obj ~= 0 then
        local current = ((status >> 10) & current_mask):tofloat() * 0.01
        --update_current_limit(true, current)
    else
        -- No PD contract: fall back to the Type-C Rp advertised current.
        local limit = typec_advertised_A()
        if limit ~= nil then
            --update_current_limit(false, limit)
        end
    end

    return update, UPDATE_MS
end

return update, UPDATE_MS
