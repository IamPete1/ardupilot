#pragma once

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_Notify;

class NotifyDevice {
public:
    virtual ~NotifyDevice() {}
    // init - initialised the device
    virtual bool init(void) = 0;
    // update - updates device according to timed_updated.  Should be
    // called at 50Hz
    virtual void update() = 0;

    // handle a LED_CONTROL message, by default device ignore message
    virtual void handle_led_control(const mavlink_message_t &msg) {}

    // handle a PLAY_TUNE message, by default device ignore message
    virtual void handle_play_tune(const mavlink_message_t &msg) {}

    // play a MML tune
    virtual void play_tune(const char *tune) {}

    // RGB override
    // override other all methods for given duration in ms (0 = for ever)
    // used with scripting
    virtual void rgb_override(uint8_t r, uint8_t g, uint8_t b, uint8_t rate_hz, uint16_t duration_ms) {}

    // this pointer is used to read the parameters relative to devices
    const AP_Notify *pNotify;
};
