#pragma once
#ifdef ENABLE_SCRIPTING

#include "AC_AttitudeControl_Multi.h"

class AC_AttitudeControl_Multi_rotate : public AC_AttitudeControl_Multi {
public:
    AC_AttitudeControl_Multi_rotate(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt):
        AC_AttitudeControl_Multi(ahrs,aparm,motors,dt) {

        if (_singleton != nullptr) {
            AP_HAL::panic("Can only be one AC_AttitudeControl_Multi_rotate");
        }
        _singleton = this;
    }

    static AC_AttitudeControl_Multi_rotate *get_singleton() {
        return _singleton;
    }

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run() override;

    // set the attitude that will be used in 6DoF flight
    void set_offset_roll_pitch(float roll_deg, float pitch_deg) {
        roll_offset_deg = roll_deg;
        pitch_offset_deg = pitch_deg;
    }

private:

    float roll_offset_deg;
    float pitch_offset_deg;

    static AC_AttitudeControl_Multi_rotate *_singleton;

};

#endif // ENABLE_SCRIPTING
