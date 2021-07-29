
#include "AP_Scripting_MAVLink_buffer.h"

// Add a new buffer to the linked list
void command_buffer::add_buffer(command_buffer * _new) {
    WITH_SEMAPHORE(sem);
    if (next == nullptr) {
        next =_new;
        return;
    }
    next->add_buffer(_new);
}

// write item to buffer if it is the watched command and ID
bool command_buffer::write(const mavlink_command_long_t &cmd, const uint32_t time_ms, const mavlink_channel_t chan) {
    WITH_SEMAPHORE(sem);
    bool ret = false;
    if ((watch_msgid == MAVLINK_MSG_ID_COMMAND_LONG) && (cmd.command == watch_CMD)) {
        struct scripting_cmd_long data {{.LONG = cmd}, time_ms, chan};
        buffer.push(data);
        ret = true;
    }
    if (next != nullptr) {
        ret |= next->write(cmd, time_ms, chan);
    }
    return ret;
}

// write item to buffer if it is the watched command and ID
bool command_buffer::write(const mavlink_command_int_t& cmd, const uint32_t time_ms, const mavlink_channel_t chan) {
    WITH_SEMAPHORE(sem);
    bool ret = false;
    if ((watch_msgid == MAVLINK_MSG_ID_COMMAND_INT) && (cmd.command == watch_CMD)) {
        struct scripting_cmd_long data {{.INT = cmd}, time_ms, chan};
        buffer.push(data);
        ret = true;
    }
    if (next != nullptr) {
        ret |= next->write(cmd, time_ms, chan);
    }
    return ret;
}

// pop command from buffer, save last channel for ack
bool command_buffer::receive(mavlink_command_long_t &cmd, uint32_t &time_ms, uint8_t &chan) {
    WITH_SEMAPHORE(sem);
    scripting_cmd_long command;
    if (buffer.pop(command)) {
        cmd = command.cmd.LONG;
        time_ms = command.time_ms;
        chan = command.chan;
        _last_chan = chan;
        return true;
    }
    return false;
}

// pop command from buffer, save last channel for ack
bool command_buffer::receive(mavlink_command_int_t & cmd, uint32_t &time_ms, uint8_t &chan) {
    WITH_SEMAPHORE(sem);
    scripting_cmd_long command;
    if (buffer.pop(command)) {
        cmd = command.cmd.INT;
        time_ms = command.time_ms;
        chan = command.chan;
        _last_chan = chan;
        return true;
    }
    return false;
}

// send ack back to the last channel
void command_buffer::send_ack(MAV_RESULT result) {
    if (_last_chan >= mavlink_channel_t::MAVLINK_COMM_0  && _last_chan <= mavlink_channel_t::MAVLINK_COMM_3) {
        send_chan_ack(result, (mavlink_channel_t)_last_chan);
    }
}

// send ack to a given channel
void command_buffer::send_chan_ack(MAV_RESULT result, mavlink_channel_t chan) {
    mavlink_msg_command_ack_send(chan, watch_CMD, result);
}
