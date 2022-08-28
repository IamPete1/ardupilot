#pragma once

#include <AP_HAL/AP_HAL.h>

// Only enable on H7 boards for ChibiOS, enable on everything else
#ifndef AP_MOTOR_FRAME_OPTIMAL_ENABLED
    #define AP_MOTOR_FRAME_OPTIMAL_ENABLED defined(STM32H7) || CONFIG_HAL_BOARD != HAL_BOARD_CHIBIOS
#endif

#if AP_MOTOR_FRAME_OPTIMAL_ENABLED

#include "AP_MotorsMatrix.h"
#include <AP_Math/matrix_vec_dsp.h>
#include "ecos/include/ecos.h"

class AP_MotorsMatrix_Optimal : public AP_MotorsMatrix {
public:

    using AP_MotorsMatrix::AP_MotorsMatrix;

    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    void output_armed_stabilizing() override;

protected:

    const char* _get_frame_string() const override;

private:

    static constexpr uint8_t max_num_motors = AP_MOTORS_MAX_NUM_MOTORS;
    static constexpr uint8_t max_num_constraints = (max_num_motors*2) + 1;

    // number of motors and constraints
    uint8_t num_motors;
    uint8_t num_constraints;

    // motor output factors
    MatrixD motor_factors;
    double motor_factors_data[max_num_motors*4];
    MatrixD motor_factors_trans;
    double motor_factors_trans_data[max_num_motors*4];

    // Hessian matrix
    MatrixD H;
    double H_data[max_num_motors*max_num_motors];
    MatrixD H_bar;
    double H_bar_data[max_num_motors*max_num_motors];

    // sparse representation of constraints matrix
    // full size matrix would be [num_motors, num_constraints]
    // [average_throttle', diag(max_throttle), diag(min_throttle)]
    // eg for four motors:
    // 1 1 0 0 0 1 0 0 0
    // 1 0 1 0 0 0 1 0 0
    // 1 0 0 1 0 0 0 1 0
    // 1 0 0 0 1 0 0 0 1
    struct sparse_A {
        float average_throttle[max_num_motors];
        float max_throttle[max_num_motors];
        float min_throttle[max_num_motors];
    } A;

    // sparse constraints matrix handling
    //void A_mult(const float*B, float*dest) const;
    //void At_mult(const float*B, float*dest) const;
    //void H_plus_A_mult_b_mult_At();

    // solver
    //void interior_point_solve();

    // interior_point_solve function local variables
    double x[max_num_motors];
    float z[max_num_constraints];
    float s[max_num_constraints];
    float s_inv[max_num_constraints];
    float rL[max_num_motors];
    float rs[max_num_constraints];
    float rsz[max_num_constraints];
    float f_bar[max_num_motors];
    float dx[max_num_motors];
    float dz[max_num_constraints];
    float ds[max_num_constraints];

    // temporay varables used in calculations
    float temp_con[max_num_constraints];
    float temp_mot[max_num_motors];

    // input and output to optimisation
    double inputs[4];
    double outputs[4];

    // set points and constraints
    float f[max_num_motors];
    float b[max_num_constraints];

    pfloat F[max_num_motors+1];

    ECOS_pwork* solution;


    // number of iterations and convergence flag
    uint8_t iter;
    bool converged;

    // ECOS CCS G matrix format
    struct sparce_CCS_G {
        // maximum number of none zero values in G matrix
        static constexpr idxint max_G_size = 2 + max_num_motors * 3 + max_num_motors * max_num_motors;
        pfloat values[max_G_size];
        idxint rows[max_G_size];
        idxint column[max_num_motors + 2];
    } G;
    pfloat h[max_num_constraints + max_num_motors + 2];

    // frame string with optimal prepended
    const char *string_prefix = "Optimal ";
    char *frame_string;

};

#endif // AP_MOTOR_FRAME_OPTIMAL_ENABLED
