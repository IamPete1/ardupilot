-- Control mode channel output

-- Serial port to receive remote stop command
local port = assert(serial:find_serial(1), "No Scripting Serial port found")
port:begin()

-- Incoming message buffer
local buffer = ""

-- Switch positions
local modes = {
    EStop = 1,
    AutoStop = 2,
    Run = 3
}

-- Channel for outgoing command to turbine
local outputServoChannel = 4

-- Channel for incoming command signal from FC
local commandServoChannel = 2

-- GPIO input from remote stop
local stopGPIOPin = 50
gpio:pinMode(stopGPIOPin, 0)

-- Telemetry object and mask
local telem = ESCTelemetryData()
local telemMask = 1 << 3 -- current

-- Return true if the vehicle is armed
local armed_mask = uint64_t(0, 2)
local function is_armed()
    if not periph then
       return arming:is_armed()
    end
    return (periph:get_vehicle_state() & armed_mask):toint() ~= 0
end

-- Read parse serial buffer
---@return string|nil -- source of stop if command received
local function parse()

    for source in string.gmatch(buffer, "[\r\n]+(.-): *StopStopStop.-[\r\n]") do
        if source then
            -- If we found a valid stop then we can dump the buffer
            buffer = ""
            return source
        end
    end

    -- Never expecting more than 50 chars
    local remove = #buffer - 49
    if remove > 0 then
        buffer = string.sub(buffer, remove)
    end

end

local remoteStop = false
local wasZero = false
local function update()

    local commandPWM = SRV_Channels:get_output_pwm_chan(commandServoChannel - 1)

    -- Commands may timeout after SRV_CMD_TIME_OUT
    local isZero = commandPWM == 0
    if isZero and not wasZero then
        print("Estop (0 PWM)")
    end
    wasZero = isZero

    local mode = modes.EStop
    if commandPWM > 1750 then
        mode = modes.Run
    elseif commandPWM > 1250 then
        mode = modes.AutoStop
    end

    -- Assume no remote stop command
    local remoteStopNew = nil

    -- Read incoming serial data
    local data = port:readstring(64)
    if (data ~= nil) and (#data > 0) then
        buffer = buffer .. data
        remoteStopNew = parse()
    end

    if remoteStopNew ~= nil then
        -- Always EStop if remote stop is active
        mode = modes.EStop
        print(string.format("EStop (remote: %s)", remoteStopNew))
        remoteStop = true

    elseif remoteStop then
        if mode == modes.EStop then
            -- Revert to user set mode once it moves to EStop
            remoteStop = false
        end
        mode = modes.EStop

    elseif SRV_Channels:get_safety_state() then
        if mode ~= modes.EStop then
            print("EStop (safety)")
        end
        mode = modes.EStop

    elseif not is_armed() and (mode == modes.Run) then
        mode = modes.EStop
        print("EStop (disarmed)")

    end

    local outputPWM = 1000
    if mode == modes.AutoStop then
        outputPWM = 1500
    elseif mode == modes.Run then
        outputPWM = 2000
    end

    SRV_Channels:set_output_pwm_chan(outputServoChannel - 1, outputPWM)

    -- Send back status in esc telem current field
    local status = 0
    if remoteStopNew then
        status = status | 0x1
    end
    if remoteStop then
        status = status | 0x2
    end
    status = status | (mode << 2)
    telem:current(status)

    esc_telem:update_telem_data(4, telem, telemMask)

    return update, 50
end

return update()
