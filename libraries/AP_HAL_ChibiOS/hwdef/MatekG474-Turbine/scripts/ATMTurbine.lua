-- Read telem data from ATM turbine controller.
-- Protocol is RS232 a converter should be used.

local port = assert(serial:find_serial(0), "No Scripting Serial port found")
port:begin()

-- Engine types
local types = {
    PEGASUS = 0,
    OLYMPUS = 1,
    MERCURY = 2,
    TITAN = 3,
    NIKE = 4,
    LYNX = 5,
    ORION = 6,
}

-- Switch positions
local switchPositions = {
    Unknown = 0,
    EStop = 1,
    AutoStop = 2,
    Run = 3
}
local switchPosition = switchPositions.Unknown

-- Status
local statuses = {
    NoStartClearance = 0,
    StartClearance = 1,
    Starting = 2,
    StartedUp = 3,
    IdleCalibration = 4,
    Running = 5,
    MaxRPM = 6,
    Unknown = 7,
}
local status = statuses.Unknown

-- Telemetry object and mask
local telem = ESCTelemetryData()
local telemMask = 1 << 0 | -- temperature
                  1 << 2 | -- voltage
                  1 << 13 -- power percentage

local telemB = ESCTelemetryData()

-- Engine status
local errorMask = 0
local lastError = uint32_t(0)
local errorTimeOut = uint32_t(5000)
local type

-- Incoming message buffer
local buffer = ""

-- Parse normal data in both normal and error messages
local function parseNormal(data3, data4, data5)

    -- Temperature, we send as raw byte to fit into AP's available data types
    --local tempDeg = (data3 * 4.6) - 50.0
    telem:temperature_cdeg(math.floor((data3 * 100) + 0.5))

    -- Throttle position
    telem:power_percentage(math.floor((data4 / 2) + 0.5))

    -- VOut voltage if we know the type
    if type ~= nil then
        local voltage
        if (type == types.MERCURY) or (type == types.PEGASUS) then
            voltage = data5 * (6.25 / 255)
        else
            voltage = data5 * (8.30 / 255)
        end
        telem:voltage(voltage)
    end

end

local function parseStatus(data)

    local switchData = data & 0x07
    if switchData == 0x1 then
        switchPosition = switchPositions.EStop

    elseif switchData == 0x2 then
        switchPosition = switchPositions.AutoStop

    elseif switchData == 0x4 then
        switchPosition = switchPositions.Run

    else
        switchPosition = switchPositions.Unknown

    end

    local statusData = data & 0xF8
    if statusData == 0x00 then
        status = statuses.NoStartClearance

    elseif statusData == 0x08 then
        status = statuses.StartClearance

    elseif statusData == 0x16 then
        status = statuses.Starting

    elseif statusData == 0x20 then
        status = statuses.StartedUp

    elseif statusData == 0x40 then
        status = statuses.IdleCalibration

    elseif statusData == 0x60 then
        status = statuses.Running

    elseif statusData == 0xE0 then
        status = statuses.MaxRPM

    else
        status = statuses.Unknown

    end

end

-- Update engine type
local function updateType(newType)

    if type == newType then
        -- Type already correct
        return
    end
    type = newType

    local name
    if type == types.PEGASUS then
        name = "PEGASUS"

    elseif type == types.OLYMPUS then
        name = "OLYMPUS"

    elseif type == types.MERCURY then
        name = "MERCURY"

    elseif type == types.TITAN then
        name = "TITAN"

    elseif type == types.NIKE then
        name = "NIKE"

    elseif type == types.LYNX then
        name = "LYNX"

    elseif type == types.ORION then
        name = "ORION"

    end

    print("AMT: detected " .. name)

end

-- Parse alternate information message
local function parseAlternate(newType, data4, data5)

    updateType(newType)

    -- Supply voltage only valid if special info flag is set
    if data4 == 254 then
        telemB:voltage(5 + data5 * 0.1)
        esc_telem:update_telem_data(5, telemB, 1 << 2)
    end

end

-- Combine errors and statuses for feedback over telem
local function getStatus()
    return ((errorMask & 0xFF) << 8) | ((switchPosition & 0xF) << 4) | (status & 0xF)
end

-- Parse the buffer
local function parse()
    -- Need at least enough data to fit the message
    if #buffer < 6 then
        return
    end

    local leader, data1, data2, data3, data4, data5 = string.byte(buffer, 1, 6)
    if leader ~= 0xFF or -- header is always 0xFF
       data1 == 0xFF or -- data is never 0xFF
       data2 == 0xFF or
       data3 == 0xFF or
       data4 == 0xFF or
       data5 == 0xFF then
        -- Not synced to data, move one by one
        buffer = string.sub(buffer, 2)

        -- Run again on next section of buffer
        parse()

        return
    end

    -- Valid data packet
    if data1 == 0 then
        -- Error packet
        lastError = millis()
        errorMask = data2
        parseNormal(data3, data4, data5)

    elseif data1 == 0x03 then
        parseAlternate(types.PEGASUS, data4, data5)

    elseif data1 == 0x06 then
        parseAlternate(types.OLYMPUS, data4, data5)

    elseif data1 == 0x07 then
        parseAlternate(types.MERCURY, data4, data5)

    elseif data1 == 0x20 then
        parseAlternate(types.TITAN, data4, data5)

    elseif data1 == 0x28 then
        parseAlternate(types.NIKE, data4, data5)

    elseif data1 == 0x30 then
        parseAlternate(types.LYNX, data4, data5)

    elseif data1 == 0x38 then
        parseAlternate(types.ORION, data4, data5)

    elseif data1 == 0x05 then
        -- ECU setup data, not parsed

    else
        -- Get control mode and status
        parseStatus(data1)

        -- RPM if we know the type
        if type ~= nil then
            local rpm
            if type == types.MERCURY then
                rpm = data2 * 700
            else
                rpm = data2 * 500
            end
            esc_telem:update_rpm(4, rpm, getStatus())
        end
        parseNormal(data3, data4, data5)
    end
    esc_telem:update_telem_data(4, telem, telemMask)

    -- Remove parsed data from buffer
    buffer = string.sub(buffer, 7)

    -- Try to parse the next message
    parse()
end


local function update()

    -- Read incoming data
    local data = port:readstring(64)
    if data ~= nil then
        buffer = buffer .. data
        parse()
    end

    -- Clear error if out of date
    if (errorMask ~= 0) and ((millis() - lastError) > errorTimeOut) then
        errorMask = 0
    end

    return update, 50
end

return update()
