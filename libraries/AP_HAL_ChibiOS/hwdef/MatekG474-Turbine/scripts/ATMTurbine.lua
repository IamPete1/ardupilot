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
    LYNX = 5
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
    Unknown = 0,
    StartClearance = 1,
    Starting = 2,
    StartedUp = 3,
    IdleCalibration = 4,
    Running = 5,
    MaxRPM = 6,
}
local status = statuses.Unknown

-- Telemetry object and mask
local telem = ESCTelemetryData()
local telemMask = 1 << 0 | -- temperature
                  1 << 2 | -- voltage
                  1 << 13 -- power percentage

-- Engine status
local errorMask = 0
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
    if statusData == 0x08 then
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

-- Parse alternate information message
local function parseAlternate(newType)

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

    end

    print("AMT: detected " .. name)

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
    if leader == 0xFF and -- header is always 0xFF
       data1 ~= 0xFF and -- data is never 0xFF
       data2 ~= 0xFF and
       data3 ~= 0xFF and
       data4 ~= 0xFF and
       data5 ~= 0xFF then

        if data1 == 0 then
            errorMask = data2
            parseNormal(data3, data4, data5)

        elseif (data1 == 0x03) or (data1 == 0x10) then
            parseAlternate(types.PEGASUS)

        elseif (data1 == 0x06) or (data1 == 0x18) then
            parseAlternate(types.OLYMPUS)

        elseif (data1 == 0x07) or (data1 == 0x08) then
            parseAlternate(types.MERCURY)

        elseif data1 == 0x20 then
            parseAlternate(types.TITAN)

        elseif data1 == 0x28 then
            parseAlternate(types.NIKE)

        elseif data1 == 0x30 then
            parseAlternate(types.LYNX)

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

        -- Remove parsed data from bufffer
        buffer = string.sub(buffer, 7)
    end

    -- Remove one element and run again
    buffer = string.sub(buffer, 2)

    -- Run again on next section of buffer
    parse()

end


local function update()
    local readSize = 64

    local data = port:readstring(readSize)
    if data ~= nil then
        buffer = buffer .. data
        parse()
    end

    return update, 50
end

return update()
