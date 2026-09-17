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
local I2C_ADDR    = 0x28    -- STUSB4500 default 7-bit address (ADDR0/ADDR1 low)
local UPDATE_MS   = 2000    -- how often to poll and report
local SEVERITY    = 6       -- MAV_SEVERITY_INFO for gcs:send_text

-- ---------------------------------------------------------------------------
-- STUSB4500 register map
-- ---------------------------------------------------------------------------
local RDO_REG_STATUS_0 = 0x91   -- negotiated Request Data Object (4 bytes, LE)
local CC_STATUS        = 0x11   -- Type-C Rp advertisement (valid without PD)

local dev = i2c:get_device(I2C_BUS, I2C_ADDR)
assert(dev ~= nil, "STUSB4500: i2c get_device failed")
dev:set_retries(10)

-- ---------------------------------------------------------------------------
-- Low level helpers
-- ---------------------------------------------------------------------------

-- Read a single 8-bit register. Returns a number, or nil on bus error.
local function read_u8(reg)
  local t = dev:transfer(string.pack("B", reg), 1)
  if not t then
    return nil
  end
  local val = string.unpack("B", t)
  return val
end

-- Read a little-endian 32-bit register. Returns a number, or nil on bus error.
local function read_u32(reg)
  local t = dev:transfer(string.pack("B", reg), 4)
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
  local cc = read_u8(CC_STATUS)
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
-- Main loop
-- ---------------------------------------------------------------------------
local current_mask = uint32_t(0x3FF)
function update()
  local status = read_u32(RDO_REG_STATUS_0)
  if status == nil then
    -- The STUSB4500 is powered by the USB, so if USB is not present then it is expected to fail to read
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
