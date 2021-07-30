

-- register for command long user 1 (31010) with buffer size of 5
local CMD_USER_1 = mavlink.register_command_long(31010, 5)
local CMD_USER_2 = mavlink.register_command_long(31011, 5)

-- mav result enum to send back for ack
local MAV_RESULT_ACCEPTED = 0
local MAV_RESULT_TEMPORARILY_REJECTED = 1
local MAV_RESULT_DENIED = 2
local MAV_RESULT_UNSUPPORTED = 3
local MAV_RESULT_FAILED = 4
local MAV_RESULT_IN_PROGRESS = 5


local function print_cmd_long(cmd, time_ms, chan)
    gcs:send_text(6,string.format("Got %i on chan %i : %f, %f, %f, %f, %f, %f, %f",cmd:command(),chan, cmd:param1(), cmd:param2(), cmd:param3(), cmd:param4(), cmd:param5(), cmd:param6(), cmd:param7() ))
    gcs:send_text(6,tostring(millis() - time_ms) .. " ms ago")
end

function update()

    local cmd, time_ms, chan = CMD_USER_1:receive()
    if cmd and chan then
        print_cmd_long(cmd, time_ms, chan)
        CMD_USER_1:send_ack(MAV_RESULT_ACCEPTED)
    end

    cmd, time_ms, chan = CMD_USER_2:receive()
    if cmd and chan then
        print_cmd_long(cmd, time_ms, chan)
        CMD_USER_2:send_ack(MAV_RESULT_TEMPORARILY_REJECTED)
    end

    return update, 1000
end

return update()
