#pragma once

#include <GCS_MAVLink/GCS.h>

class command_long_buffer {
public:
    // constructor
    command_long_buffer(uint8_t buffer_size, int16_t _watch):buffer(buffer_size),watch(_watch) {}

    // Add a new buffer to the linked list
    void add_buffer(command_long_buffer * _new);

    // write item to buffer if it is the watched command
    bool write(const mavlink_command_long_t& cmd, const mavlink_channel_t chan);

    // pop command from buffer, save last channel for ack
    bool receive(mavlink_command_long_t & cmd, uint8_t &chan);

    // send ack back to the last channel
    void send_ack(MAV_RESULT result);

    // send ack to a given channel
    void send_chan_ack(MAV_RESULT result, mavlink_channel_t chan);

private:

    // struct for object buffer
    struct scripting_cmd_long {
        mavlink_command_long_t cmd;
        mavlink_channel_t chan;
    };

    // the command ID to watch
    const int16_t watch;

    // last channel popped
    int8_t _last_chan = -1;

    ObjectBuffer<scripting_cmd_long> buffer;

    command_long_buffer * next;

    HAL_Semaphore sem;

};

class command_int_buffer {
public:
    // constructor
    command_int_buffer(uint8_t buffer_size, int16_t _watch):buffer(buffer_size),watch(_watch) {}

    // Add a new buffer to the linked list
    void add_buffer(command_int_buffer * _new);

    // write item to buffer if it is the watched command
    bool write(const mavlink_command_int_t& cmd, const mavlink_channel_t chan);

    // pop command from buffer, save last channel for ack
    bool receive(mavlink_command_int_t & cmd, uint8_t &chan);

    // send ack back to the last channel
    void send_ack(MAV_RESULT result);

    // send ack to a given channel
    void send_chan_ack(MAV_RESULT result, mavlink_channel_t chan);

private:

    // struct for object buffer
    struct scripting_cmd_int {
        mavlink_command_int_t cmd;
        mavlink_channel_t chan;
    };

    // the command ID to watch
    const int16_t watch;

    // last channel popped
    int8_t _last_chan = -1;

    ObjectBuffer<scripting_cmd_int> buffer;

    command_int_buffer * next;

    HAL_Semaphore sem;
};
