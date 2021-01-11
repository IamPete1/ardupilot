#pragma once
#ifdef ENABLE_SCRIPTING

#include "AC_AttitudeControl_Multi.h"

class AC_AttitudeControl_Multi_6DoF : public AC_AttitudeControl_Multi {
public:
    AC_AttitudeControl_Multi_6DoF(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt):
        AC_AttitudeControl_Multi(ahrs,aparm,motors,dt) {

        if (_singleton != nullptr) {
            AP_HAL::panic("Can only be one AC_AttitudeControl_Multi_6DoF");
        }
        _singleton = this;
    }

    virtual ~AC_AttitudeControl_Multi_6DoF() {}

    static AC_AttitudeControl_Multi_6DoF *get_singleton() {
        return _singleton;
    }

    // Command a Quaternion attitude with feedforward and smoothing
    // not used anywhere in current code, panic so this implementaiton is not overlooked
    void input_quaternion(Quaternion attitude_desired_quat) override {
        AP_HAL::panic("input_quaternion not implemented AC_AttitudeControl_Multi_6DoF");
    }

    /*
        override input functions to attitude controller and convert desired angles into thrust angles and substitute for osset angles
    */

    // Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
    void input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)  override ;

    // Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing
    void input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw) override;

    /*
        all other input functions should zero thrust vectoring and behave as a normal copter
    */

    // Command euler yaw rate and pitch angle with roll angle specified in body frame
    // (used only by tailsitter quadplanes)
    void input_euler_rate_yaw_euler_angle_pitch_bf_roll(bool plane_controls, float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds) override;

    // Command an euler roll, pitch, and yaw rate with angular velocity feedforward and smoothing
    void input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds) override;

    // Command an angular velocity with angular velocity feedforward and smoothing
    void input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) override;

    // Command an angular velocity with angular velocity smoothing using rate loops only with no attitude loop stabilization
    void input_rate_bf_roll_pitch_yaw_2(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) override;

    // Command an angular velocity with angular velocity smoothing using rate loops only with integrated rate error stabilization
    void input_rate_bf_roll_pitch_yaw_3(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) override;

    // Command an angular step (i.e change) in body frame angle
    void input_angle_step_bf_roll_pitch_yaw(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd) override;

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run() override;

    // limiting lean angle based on throttle makes no sense for 6DoF, always allow 90 deg, return in centi-degrees
    float get_althold_lean_angle_max() const override { return 9000.0f; }

    /*
        Scripting access helpers, to allow roll and pitch angles or rates to be set independently
    */

    // set the attitude that will be used in 6DoF flight
    void set_offset_roll_pitch(float roll_deg, float pitch_deg);

    // set the rotation rate that will be used in 6DoF flight
    void set_rate_roll_pitch(float roll_degs, float pitch_degs);
    void set_rate_roll_pitch_yaw(float roll_degs, float pitch_degs, float yaw_degs);

    // enable or disable forward and lateral control, allow to switch from 6DoF down to 5 or 4DoF
    void set_forward_enable(bool b);
    void set_lateral_enable(bool b);

private:

    // helper to set forward and lateral motors from angle inputs
    void set_forward_lateral(float &euler_pitch_angle_cd, float &euler_roll_angle_cd);

    // determes we have valid roll and pitch rates to use and updates roll and pitch angles accordingly
    bool use_rate_input();

    // latest angle inputs
    float roll_offset_deg;
    float pitch_offset_deg;

    // latest rate inputs
    float roll_rate_degs;
    float pitch_rate_degs;
    float yaw_rate_degs;

    // override flag for yaw
    bool yaw_override;

    // angle and rate input timestamps
    uint32_t last_rate_input_ms;
    uint32_t last_angle_input_ms;

    // enable flags for forward and lateral axis
    bool forward_enable = true;
    bool lateral_enable = true;

    static AC_AttitudeControl_Multi_6DoF *_singleton;

};

#endif // ENABLE_SCRIPTING
