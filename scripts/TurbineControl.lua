-- Control turbine mode channel based on pilot input and MAVLink commands

local mavlink_msgs = require("MAVLink/mavlink_msgs")

local COMMAND_ACK_ID = mavlink_msgs.get_msgid("COMMAND_ACK")
local COMMAND_LONG_ID = mavlink_msgs.get_msgid("COMMAND_LONG")

local msg_map = {}
msg_map[COMMAND_ACK_ID] = "COMMAND_ACK"
msg_map[COMMAND_LONG_ID] = "COMMAND_LONG"

local MAV_CMD_WAYPOINT_USER_1 = 31000
mavlink:block_command(MAV_CMD_WAYPOINT_USER_1)

-- initialize MAVLink rx with buffer depth and number of rx message IDs to register
mavlink:init(10, 1)

-- register message id to receive
mavlink:register_rx_msgid(COMMAND_LONG_ID)

-- RC input using Scripting 10
local rcSwitch = assert(rc:find_channel_for_option(309), "no RCx_OPTION 309")

-- RC output using scripting funciton 5
local outputChan = assert(SRV_Channels:find_channel(98), "No scripting 5 channel")
SRV_Channels:set_output_pwm_chan(outputChan, 1000)

-- Switch positions
local modes = {
    Unknown = 0,
    EStop = 1,
    AutoStop = 2,
    Run = 3
}
local mode = modes.EStop

local function getModeString(val)
    if val == modes.EStop then
        return "EStop"

    elseif val == modes.AutoStop then
        return "AutoStop"

    elseif val == modes.Run then
        return "Run"
    end
    return "Unknown"
end

local function setMode(newMode, reason)
    if newMode == mode then
        return
    end
    mode = newMode

    gcs:send_text(4, string.format("Lynx set mode: %s (%s)", getModeString(mode), reason))
end

local lastRCMode
local function getSwitchMode()
    -- If there is no input then go to EStop
    if not rc:has_valid_input() then
        lastRCMode = nil
        return nil
    end

    local switch = rcSwitch:get_aux_switch_pos()
    local RCMode = modes.EStop
    if switch == 1 then
        RCMode = modes.AutoStop
    elseif switch == 2 then
        RCMode = modes.Run
    end

    -- Don't do anything on intial command or on command change
    if lastRCMode == nil then
        lastRCMode = RCMode
        return nil
    end

    -- Don't do anything if the command has not changed
    if lastRCMode == RCMode then
        return nil
    end

    lastRCMode = RCMode
    return RCMode
end

local function getRemoteStop()
    -- ESC current is used to transport data
    local val = esc_telem:get_current(5)
    if val == nil then
        return true
    end
    return (val & 1) ~= 0
end

local function sendMAVLinkACK(msg, chan, result)
    if (msg == nil) or (chan == nil) then
        return
    end
    local ack = {}
    ack.command = msg.command
    ack.result = result
    ack.progress = 0
    ack.result_param2 = 0
    ack.target_system = msg.sysid
    ack.target_component = msg.compid
    mavlink:send_chan(chan, mavlink_msgs.encode("COMMAND_ACK", ack))
end

local function getGCSMode()
    local msg, chan = mavlink:receive_chan()
    if msg == nil then
        -- No waiting message
        return
    end
    local parsed_msg = mavlink_msgs.decode(msg, msg_map)
    if (parsed_msg ~= nil) and (parsed_msg.msgid == COMMAND_LONG_ID) and (parsed_msg.command == MAV_CMD_WAYPOINT_USER_1) then
        -- Make sure mode is valid
        local cmdMode = parsed_msg.param1
        if (cmdMode ~= modes.EStop) and (cmdMode ~= modes.AutoStop) and (cmdMode ~= modes.Run) then
            cmdMode = nil
            sendMAVLinkACK(parsed_msg, chan, 2) -- MAV_RESULT_DENIED
        end
        return parsed_msg, chan, cmdMode
    end
end

local statuses = {
    Unknown = 0,
    StartClearance = 1,
    Starting = 2,
    StartedUp = 3,
    IdleCalibration = 4,
    Running = 5,
    MaxRPM = 6,
}
local function getStatusString(val)
    if val == statuses.StartClearance then
        return "StartClearance"

    elseif val == statuses.Starting then
        return "Starting"

    elseif val == statuses.StartedUp then
        return "StartedUp"

    elseif val == statuses.IdleCalibration then
        return "IdleCalibration"

    elseif val == statuses.Running then
        return "Running"

    elseif val == statuses.MaxRPM then
        return "MaxRPM"

    end
    return "Unknown"
end

local lastSwitchPos
local lastErrors
local lastSatus = statuses.Unknown
local function updateTelem()
    local _, statusVal = esc_telem:get_raw_rpm_and_error_rate(5)
    if statusVal == nil then
        return
    end
    local status = statusVal & 0xF
    local switchPos = (statusVal >> 4) & 0xF
    local errorMask = (statusVal >> 8) & 0xFF

    logger:write("LYNX", "status,switch,error", "BBB", status, switchPos, errorMask)

    if status ~= lastSatus then
        lastSatus = status
        gcs:send_text(4, string.format("Lynx status: %s", getStatusString(status)))
    end

    if switchPos ~= lastSwitchPos then
        lastSwitchPos = switchPos
        gcs:send_text(4, string.format("Lynx mode: %s", getModeString(switchPos)))
    end

    local haveErrors = errorMask ~= 0
    if haveErrors ~= lastErrors then
        lastErrors = haveErrors
        if haveErrors then
            gcs:send_text(4, string.format("Lynx errors: 0x%02X", errorMask))
        end
    end

    gcs:send_named_float("LYNX", statusVal)

end

local lastVehicleMode = nil
local lastArmed = nil
local function update()

    updateTelem()

    local powerPct = esc_telem:get_power_percentage(5)
    if powerPct ~= nil then
        gcs:send_named_float("LYNXThr", powerPct)
    end

    local failsafeReason = nil

    -- Check for hold mode
    local vehicleMode = vehicle:get_mode()
    if vehicleMode ~= lastVehicleMode then
        lastVehicleMode = vehicleMode
        -- Mode change
        if vehicleMode == 4 then
            failsafeReason = "Hold"
        end
    end

    -- Check for disarmed
    local armed = arming:is_armed()
    if lastArmed ~= armed then
        lastArmed = armed
        if not armed then
            failsafeReason = "Disarmed"
        end
    end

    -- Check for remote stop
    if getRemoteStop() then
        failsafeReason = "remote kill"
    end

    -- Read in RC switch, apply commaded mode on change
    local rcMode = getSwitchMode()

    -- Read in MAVLink command
    local msg, chan, GCSMode = getGCSMode()

    -- Failsafe always takes prority
    if failsafeReason ~= nil then
        setMode(modes.EStop, failsafeReason)
        sendMAVLinkACK(msg, chan, 1) -- MAV_RESULT_TEMPORARILY_REJECTED
        if rcMode ~= nil then
            gcs:send_text(4, string.format("Lynx RC mode change ignored (%s)", failsafeReason))
        end

    elseif rcMode ~= nil then
        if (rcMode == modes.Run) and not armed then
            gcs:send_text(4, string.format("Lynx RC mode change ignored (disarmed)"))
        else
            setMode(rcMode, "RC")
        end
        sendMAVLinkACK(msg, chan, 1) -- MAV_RESULT_TEMPORARILY_REJECTED

    elseif GCSMode ~= nil then
        if (GCSMode == modes.Run) and not armed then
            sendMAVLinkACK(msg, chan, 1) -- MAV_RESULT_TEMPORARILY_REJECTED
        else
            setMode(GCSMode, "GCS")
            sendMAVLinkACK(msg, chan, 0) -- MAV_RESULT_ACCEPTED
        end
    end

    local PWM = 1000
    if mode == modes.AutoStop then
        PWM = 1500
    elseif mode == modes.Run then
        PWM = 2000
    end

    SRV_Channels:set_output_pwm_chan_timeout(outputChan, PWM, 500)

    return update, 100
end

return update()
