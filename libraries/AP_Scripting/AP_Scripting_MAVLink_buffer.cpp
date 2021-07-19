
#include "AP_Scripting_MAVLink_buffer.h"

// Add a new buffer to the linked list
void command_long_buffer::add_buffer(command_long_buffer * _new) {
    WITH_SEMAPHORE(sem);
    if (next == nullptr) {
        next =_new;
        return;
    }
    next->add_buffer(_new);
}

// write item to buffer if it is the watched command
bool command_long_buffer::write(const mavlink_command_long_t& cmd, const mavlink_channel_t chan) {
    WITH_SEMAPHORE(sem);
    bool ret = false;
    if (cmd.command == watch) {
        struct scripting_cmd_long data {cmd, chan};
        buffer.push(data);
        ret = true;
    }
    if (next != nullptr) {
        ret |= next->write(cmd, chan);
    }
    return ret;
}

// pop command from buffer, save last channel for ack
bool command_long_buffer::receive(mavlink_command_long_t & cmd, uint8_t &chan) {
    WITH_SEMAPHORE(sem);
    scripting_cmd_long command;
    if (buffer.pop(command)) {
        cmd = command.cmd;
        chan = command.chan;
        _last_chan = chan;
        return true;
    }
    return false;
}

// send ack back to the last channel
void command_long_buffer::send_ack(MAV_RESULT result) {
    if (_last_chan >= mavlink_channel_t::MAVLINK_COMM_0  && _last_chan <= mavlink_channel_t::MAVLINK_COMM_3) {
        send_chan_ack(result, (mavlink_channel_t)_last_chan);
    }
}

// send ack to a given channel
void command_long_buffer::send_chan_ack(MAV_RESULT result, mavlink_channel_t chan) {
    mavlink_msg_command_ack_send(chan, watch, result, 0, 0, 0, 0);
}

// Add a new buffer to the linked list
void command_int_buffer::add_buffer(command_int_buffer * _new) {
    WITH_SEMAPHORE(sem);
    if (next == nullptr) {
        next =_new;
        return;
    }
    next->add_buffer(_new);
}

// write item to buffer if it is the watched command
bool command_int_buffer::write(const mavlink_command_int_t& cmd, const mavlink_channel_t chan) {
    WITH_SEMAPHORE(sem);
    bool ret = false;
    if (cmd.command == watch) {
        struct scripting_cmd_int data {cmd, chan};
        buffer.push(data);
        ret = true;
    }
    if (next != nullptr) {
        ret |= next->write(cmd, chan);
    }
    return ret;
}

// pop command from buffer, save last channel for ack
bool command_int_buffer::receive(mavlink_command_int_t & cmd, uint8_t &chan) {
    WITH_SEMAPHORE(sem);
    scripting_cmd_int command;
    if (buffer.pop(command)) {
        cmd = command.cmd;
        chan = command.chan;
        _last_chan = chan;
        return true;
    }
    return false;
}

// send ack back to the last channel
void command_int_buffer::send_ack(MAV_RESULT result) {
    if (_last_chan >= mavlink_channel_t::MAVLINK_COMM_0  && _last_chan <= mavlink_channel_t::MAVLINK_COMM_3) {
        send_chan_ack(result, (mavlink_channel_t)_last_chan);
    }
}

// send ack to a given channel
void command_int_buffer::send_chan_ack(MAV_RESULT result, mavlink_channel_t chan) {
    mavlink_msg_command_ack_send(chan, watch, result, 0, 0, 0, 0);
}
