#pragma once
#ifdef ENABLE_SCRIPTING

#include "AP_MotorsMatrix.h"

#define AP_MOTORS_MATRIX_YAW_FACTOR_CW   -1
#define AP_MOTORS_MATRIX_YAW_FACTOR_CCW   1

/// @class      AP_MotorsMatrix
class AP_MotorsMatrix_Scripting_interp : public AP_MotorsMatrix {
public:

    /// Constructor
    AP_MotorsMatrix_Scripting_interp(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz)
    {
        if (_singleton != nullptr) {
            AP_HAL::panic("AP_MotorsMatrix_Scripting_interp must be singleton");
        }
        _singleton = this;
    };

    // get singleton instance
    static AP_MotorsMatrix_Scripting_interp *get_singleton() {
        return _singleton;
    }

    struct factors_table {
        float roll[AP_MOTORS_MAX_NUM_MOTORS];
        float pitch[AP_MOTORS_MAX_NUM_MOTORS];
        float yaw[AP_MOTORS_MAX_NUM_MOTORS];
        float value; // value that this table will used in interpolation
        factors_table *next;
    };

    // base class method must not be used
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override {};

    // Init to be called from scripting
    bool init(uint8_t expected_num_motors) override;

    // add a motor and give its testing order
    bool add_motor(int8_t motor_num, uint8_t testing_order);

    // add a interpolation point table
    bool add_table(factors_table *table);

    // trigger a re-interpolation of roll pitch and yaw factors at the given point
    void set_interpolation_point(float value);

    // output - sends commands to the motors
    void output_to_motors() override;

private:

    // re-interpolate roll pitch and yaw factors
    void interp();

    // load from the given table
    void load_from_table(factors_table *table);

    // intpolate between two tables
    void interpolate_tables(factors_table *table_low,factors_table *table_high);

    factors_table* interp_table;
    bool trigger_interp;
    float interpolation_value;

    static AP_MotorsMatrix_Scripting_interp *_singleton;
};

#endif // ENABLE_SCRIPTING
