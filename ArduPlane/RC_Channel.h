#pragma once

#include <RC_Channel/RC_Channel.h>

class RC_Channel_Plane : public RC_Channel
{

public:

    static bool have_reverse_throttle_rc_option;

protected:

    void init_aux_function(aux_func_t ch_option,
                           aux_switch_pos_t ch_flag) override;
    void do_aux_function(aux_func_t ch_option, aux_switch_pos_t) override;

private:

    void do_aux_function_change_mode(Mode::Number number,
                                     aux_switch_pos_t ch_flag);
};

class RC_Channels_Plane : public RC_Channels
{
public:

    void init() override;

    RC_Channel_Plane obj_channels[NUM_RC_CHANNELS];
    RC_Channel_Plane *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    bool has_valid_input() const override;

    void set_control_channels(void);

    // primary input control channels
    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_rudder;

protected:

    // note that these callbacks are not presently used on Plane:
    int8_t flight_mode_channel_number() const override;

    bool k_param_rcmap_for_conversion(uint8_t &k_param_rcmap) const override;
};
