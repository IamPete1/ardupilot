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
local UPDATE_MS   = 2000    -- how often to poll and report
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
  -- bits[1:0] = CC1 state, bits[3:2] = CC2 state; the connected pin is non-zero.
  local state = cc & 0x03
  if state == 0 then
    state = (cc >> 2) & 0x03
  end
  if state == 0x01 then
    return 0.5    -- USB default (500mA for USB 2.0; chip can't tell 500 vs 900)
  elseif state == 0x02 then
    return 1.5
  elseif state == 0x03 then
    return 3.0
  end
  return 0        -- not connected
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

-- Check REQ_SRC_CURRENT (sector 4, 0xE6 bit 4). If it isn't set, reprogram ONLY
-- sector 4 to set it, then verify. Reports the outcome to the GCS.
-- No write happens if the bit is already set.
local function nvn_check()
    if not nvm_enable() then
        gcs:send_text(SEVERITY, "REQ_SRC: NVM enable failed")
        return false
    end
    local s4 = nvm_read_sector(4)
    nvm_exit()
    if s4 == nil then
        gcs:send_text(SEVERITY, "REQ_SRC: NVM read failed")
        return false
    end

    local E6 = string.byte(s4, 7)
    local REQ_SRC_CURRENT = (E6 >> 4) & 0x01

    if REQ_SRC_CURRENT == 1 then
        gcs:send_text(SEVERITY, string.format("REQ_SRC_CURRENT configured correctly"))
        return true
    end

    -- Set the bit in the byte and sector
    local new_e6 = E6 | (0x01 << 4)
    local new_s4 = s4:sub(1, 6) .. string.char(new_e6) .. s4:sub(8, 8)

    if not nvm_enter_write(0x01 << 4) then
        nvm_exit()
        gcs:send_text(SEVERITY, "REQ_SRC: erase failed")
        return false
    end
    local ok = nvm_write_sector(4, new_s4)
    if not ok then
        nvm_exit()
        gcs:send_text(SEVERITY, "REQ_SRC: program failed")
        return false
    end

    local updated_s4 = nvm_read_sector(4)
    nvm_exit()

    if updated_s4 ~= new_s4 then
        gcs:send_text(SEVERITY, "REQ_SRC: verify failed")
        return false
    end

    gcs:send_text(SEVERITY, "REQ_SRC: NVM updated")
    return true
end

-- ---------------------------------------------------------------------------
-- Main loop
-- ---------------------------------------------------------------------------
local current_mask = uint32_t(0x3FF)
local function update()
  local status = read_u32(RDO_REG_STATUS_0)
  if status == nil then
    -- The STUSB4500 is powered by the USB, so if USB is not present then it is expected to fail to read
    return update, UPDATE_MS
  end

  if USB_NVM:get() == 1 then
    if nvn_check() then
        USB_NVM:set_and_save(0)
    end
    nvm_dump()
    return update, UPDATE_MS
  end

  -- Object position (bits 30:28) is non-zero only when a PD contract exists.
  local obj = ((status >> 28) & 0x07):toint()
  if obj ~= 0 then
    local current = ((status >> 10) & current_mask):tofloat() * 0.01
    local current_limit = (status & current_mask):tofloat() * 0.01
    gcs:send_text(SEVERITY, string.format("STUSB4500: PD %0.2fA / %0.2fA", current, current_limit))
  else
    -- No PD contract: fall back to the Type-C Rp advertised current.
    local limit = typec_advertised_A()
    if limit ~= nil then
      gcs:send_text(SEVERITY, string.format("STUSB4500: no PD, Type-C limit %0.2fA", limit))
    end
  end

  return update, UPDATE_MS
end

return update, UPDATE_MS
