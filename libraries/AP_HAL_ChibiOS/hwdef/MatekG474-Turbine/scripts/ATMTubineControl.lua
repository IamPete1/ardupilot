-- Control mode channel output

-- Switch positions
local modes = {
    EStop = 1,
    AutoStop = 2,
    Run = 3
}

-- Channel for outgoing command to turbine
local outputServoChannel = 2

-- Channel for incoming command signal from FC
local commandServoChannel = 3

-- GPIO input from remote stop
local stopGPIOPin = 0
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

local remoteStop = false
local function update()

    local commandPWM = SRV_Channels:get_output_pwm_chan(commandServoChannel)
    local mode = modes.EStop
    if commandPWM > 1750 then
        mode = modes.Run
    elseif commandPWM > 1250 then
        mode = modes.AutoStop
    end

    local remoteStopNew = gpio:read(stopGPIOPin)
    if remoteStopNew then
        -- Always EStop if remote stop is active
        mode = modes.EStop
        remoteStop = true

    elseif remoteStop then
        if mode == modes.EStop then
            -- Revert to user set mode once it moves to EStop
            remoteStop = false
        end
        mode = modes.EStop

    elseif SRV_Channels:get_safety_state() then
        mode = modes.EStop

    elseif not is_armed() and (mode == modes.Run) then
        mode = modes.EStop

    end

    local outputPWM = 1000
    if mode == modes.AutoStop then
        outputPWM = 1500
    elseif mode == modes.Run then
        outputPWM = 2000
    end

    SRV_Channels:set_output_pwm_chan(outputServoChannel, outputPWM)

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

    esc_telem:update_telem_data(5, telem, telemMask)

    return update, 50
end

return update()
