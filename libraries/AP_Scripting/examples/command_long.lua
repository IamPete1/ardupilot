

-- register for command long user 1 (31010) with buffer size of 5
local CMD_USER_1 = mavlink.register_command_long(31010, 5)
local CMD_USER_2 = mavlink.register_command_long(31011, 5)

local MAV_CMD_DO_SEND_BANNER = command_long()
MAV_CMD_DO_SEND_BANNER:command(42428)

local tick = 0

-- mav result enum to send back for ack
local MAV_RESULT_ACCEPTED = 0
local MAV_RESULT_TEMPORARILY_REJECTED = 1
local MAV_RESULT_DENIED = 2
local MAV_RESULT_UNSUPPORTED = 3
local MAV_RESULT_FAILED = 4
local MAV_RESULT_IN_PROGRESS = 5


local function print_cmd_long(cmd, chan)
    gcs:send_text(6,string.format("Got %i on chan %i : %f, %f, %f, %f, %f, %f, %f",cmd:command(),chan, cmd:param1(), cmd:param2(), cmd:param3(), cmd:param4(), cmd:param5(), cmd:param6(), cmd:param7() ))
end

function update()

    local cmd, chan = CMD_USER_1:receive()
    if cmd and chan then
        print_cmd_long(cmd, chan)
        CMD_USER_1:send_ack(MAV_RESULT_ACCEPTED)
    end

    cmd, chan = CMD_USER_2:receive()
    if cmd and chan then
        print_cmd_long(cmd, chan)
        CMD_USER_2:send_ack(MAV_RESULT_TEMPORARILY_REJECTED)
    end

    tick = tick + 1
    if tick > 10 then
        tick = 0
        gcs:send_text(6,"Sending Banner")
        vehicle:command_long(MAV_CMD_DO_SEND_BANNER)
    end

    return update, 1000
end

return update()
