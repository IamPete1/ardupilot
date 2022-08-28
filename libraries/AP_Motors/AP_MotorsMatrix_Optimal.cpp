/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_MotorsMatrix_Optimal.h"

#if AP_MOTOR_FRAME_OPTIMAL_ENABLED

// allows more accurate timing, do not fly!
#define DISABLE_INTERRUPTS_FOR_TIMMING 0

#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#pragma GCC optimize("O3")

extern const AP_HAL::HAL& hal;

void AP_MotorsMatrix_Optimal::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    AP_MotorsMatrix::init(frame_class, frame_type);
    if (!initialised_ok()) {
        // underlying class must init correctly
        return;
    }

    if (frame_class == MOTOR_FRAME_SCRIPTING_MATRIX) {
        // Scripting frame class not supported
        // Scripting is setup to use the AP_MotorsMatrix singleton, which wont exist if were here.
        // So there is no way for scripting to setup the motors, once we fix that it should work well...
        set_initialised_ok(false);
        return;
    }

    // hack to allow copying in of one cone setup matrix for testing
    if ((frame_class != MOTOR_FRAME_OCTA) || (frame_type != MOTOR_FRAME_TYPE_X)) {
        set_initialised_ok(false);
        return;
    }

    // conversion factors so the new mix comes out close to the old
    // should reduce the need to re-tune, I have not calculated them all yet...
    // there may be some way to derive from the motor matrix, but I can't work it out
    // these are still not a perfect conversion, I'm not sure why....
    // may also need a yaw conversion for the A V and tail frames... but there quads so this mixer won't help much in anycase
    double roll_conversion;
    double pitch_conversion;
    switch (frame_class) {
        case MOTOR_FRAME_QUAD: {
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                case MOTOR_FRAME_TYPE_NYT_PLUS:
                case MOTOR_FRAME_TYPE_PLUSREV:
                    roll_conversion = 0.5;
                    pitch_conversion = 0.5;
                    break;
                case MOTOR_FRAME_TYPE_X:
                case MOTOR_FRAME_TYPE_NYT_X:
                case MOTOR_FRAME_TYPE_BF_X:
                case MOTOR_FRAME_TYPE_BF_X_REV:
                case MOTOR_FRAME_TYPE_DJI_X:
                case MOTOR_FRAME_TYPE_CW_X:
                case MOTOR_FRAME_TYPE_V:
                case MOTOR_FRAME_TYPE_H:
                case MOTOR_FRAME_TYPE_VTAIL: // Not calculated, assume 1
                case MOTOR_FRAME_TYPE_ATAIL: // Not calculated, assume 1
                    roll_conversion = 1.0;
                    pitch_conversion = 1.0;
                    break;
                case MOTOR_FRAME_TYPE_Y4:
                    roll_conversion = 0.5;
                    pitch_conversion = 1.0;
                    break;
                default:
                    return;
            }
            break;
        }
        case MOTOR_FRAME_HEXA: {
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    roll_conversion = 1.0;
                    pitch_conversion = 0.75;
                    break;
                case MOTOR_FRAME_TYPE_X:
                case MOTOR_FRAME_TYPE_DJI_X:
                case MOTOR_FRAME_TYPE_CW_X:
                    roll_conversion = 0.75;
                    pitch_conversion = 1.0;
                    break;
                case MOTOR_FRAME_TYPE_H:
                    roll_conversion = 1.5;
                    pitch_conversion = 1.0;
                    break;
                default:
                    return;
            }
            break;
        }
        case MOTOR_FRAME_OCTA: {
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    roll_conversion = 1.0;
                    pitch_conversion = 1.0;
                    break;
                case MOTOR_FRAME_TYPE_X:
                case MOTOR_FRAME_TYPE_DJI_X:
                case MOTOR_FRAME_TYPE_CW_X:
                    *reinterpret_cast<uint64_t *>(&roll_conversion)  = 0x3FF2BEC33301885E;
                    *reinterpret_cast<uint64_t *>(&pitch_conversion) = 0x3FF2BEC33301885E;
                    //roll_conversion = 1.1715728;
                    //pitch_conversion = 1.1715728;
                    break;
                case MOTOR_FRAME_TYPE_V:
                    roll_conversion = 1.1939;
                    pitch_conversion = 1.109;
                    break;
                case MOTOR_FRAME_TYPE_H:
                    roll_conversion = 2.0;
                    pitch_conversion = 1.11;
                    break;
                case MOTOR_FRAME_TYPE_I:
                    roll_conversion = 1.11;
                    pitch_conversion = 2.0;
                    break;
                default:
                    return;
            }
            break;
        }
        case MOTOR_FRAME_OCTAQUAD: {
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    roll_conversion = 1.0;
                    pitch_conversion = 1.0;
                    break;
                case MOTOR_FRAME_TYPE_X:
                case MOTOR_FRAME_TYPE_V:
                case MOTOR_FRAME_TYPE_H:
                case MOTOR_FRAME_TYPE_CW_X:
                    roll_conversion = 2.0;
                    pitch_conversion = 2.0;
                    break;
                default:
                    return;
            }
            break;
        }
        case MOTOR_FRAME_DODECAHEXA: {
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    roll_conversion = 2.0;
                    pitch_conversion = 1.5;
                    break;
                case MOTOR_FRAME_TYPE_X:
                    roll_conversion = 1.5;
                    pitch_conversion = 2.0;
                    break;
                default:
                    return;
            }
            break;
        }
        case MOTOR_FRAME_Y6: {
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_Y6B:
                case MOTOR_FRAME_TYPE_Y6F:
                default:
                    roll_conversion = 1.0;
                    pitch_conversion = 0.75;
                    break;
            }
            break;
        }
        case MOTOR_FRAME_DECA: {
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    roll_conversion = 1.38;
                    pitch_conversion = 1.25;
                    break;
                case MOTOR_FRAME_TYPE_X:
                case MOTOR_FRAME_TYPE_CW_X:
                    roll_conversion = 1.25;
                    pitch_conversion = 1.38;
                    break;
                default:
                    return;
            }
            break;
        }
        default:
            return;
    }

    // update frame name
    if (frame_string != nullptr) {
        delete [] frame_string;
    }
    const size_t len = strlen(string_prefix)+strlen(AP_MotorsMatrix::_get_frame_string())+1;
    frame_string = new char[len];
    if (frame_string != nullptr) {
        hal.util->snprintf(frame_string, len, "%s%s", string_prefix, AP_MotorsMatrix::_get_frame_string());
    }

    // convert motor factors to matrix format
    num_motors = 0;
    for (uint8_t i = 0; i < max_num_motors; i++) {
        if (motor_enabled[i]) {
            num_motors++;
        }
    }
    num_constraints = (num_motors*2) + 1;

    init_mat(motor_factors, num_motors, 4, motor_factors_data);
    init_mat(motor_factors_trans, 4, num_motors, motor_factors_trans_data);
    for (uint8_t i = 0; i < num_motors; i++) {
        motor_factors_data[i*4 + 0] = double(_roll_factor[i]) / roll_conversion;
        motor_factors_data[i*4 + 1] = double(_pitch_factor[i]) / pitch_conversion;
        motor_factors_data[i*4 + 2] = double(_yaw_factor[i]);
        motor_factors_data[i*4 + 3] = double(_throttle_factor[i]);
    }
    mat_trans(motor_factors, motor_factors_trans);

    // Weighting vector defines the relative weighting of roll, pitch, yaw and throttle
    // may want to change this on the fly in the future, but in that case we can no longer pre-compute the hessian
    // roll, pitch, yaw, throttle
    double w[4] = {50.0, 50.0, 1.0, 0.1};

    // setup hessian matrix
    // H = motor_factors * diag(w) * motor_factors'
    init_mat(H, num_motors, num_motors, H_data);
    init_mat(H_bar, num_motors, num_motors, H_bar_data);

    MatrixD motor_factors_tmp;
    double tmp_A[12*4];
    init_mat(motor_factors_tmp, num_motors, 4, tmp_A);
    per_element_mult_mv(motor_factors, w, motor_factors_tmp);

    mat_mult(motor_factors_tmp, motor_factors_trans, H);

    // setup constraints
    for (uint8_t i = 0; i < num_motors; i++) {
        A.average_throttle[i] = -motor_factors_data[i*4 + 3] / num_motors;
        A.max_throttle[i] = -1.0;
        A.min_throttle[i] =  1.0;
    }

    // re-scale by weights to save runtime calculation
    w[2] *= num_motors*0.25;
    w[3] *= num_motors;
    vec_scale(w, -1.0, w, 4);
    per_element_mult_mv(motor_factors, w, motor_factors);

    *reinterpret_cast<uint64_t *>(&motor_factors_data[0]) = 0x4021AD7BC01366C1;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[1]) = 0xC03556BDE009B366;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[2]) = 0x3FF0000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[3]) = 0xBFE999999999999A;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[4]) = 0xC021AD7BC01366C1;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[5]) = 0x403556BDE009B366;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[6]) = 0x3FF0000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[7]) = 0xBFE999999999999A;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[8]) = 0x403556BDE009B366;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[9]) = 0xC021AD7BC01366C1;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[10]) = 0xBFF0000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[11]) = 0xBFE999999999999A;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[12]) = 0x4021AD7BC01366C1;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[13]) = 0x403556BDE009B366;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[14]) = 0xBFF0000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[15]) = 0xBFE999999999999A;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[16]) = 0xC021AD7BC01366C1;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[17]) = 0xC03556BDE009B366;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[18]) = 0xBFF0000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[19]) = 0xBFE999999999999A;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[20]) = 0xC03556BDE009B366;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[21]) = 0x4021AD7BC01366C1;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[22]) = 0xBFF0000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[23]) = 0xBFE999999999999A;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[24]) = 0xC03556BDE009B366;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[25]) = 0xC021AD7BC01366C1;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[26]) = 0x3FF0000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[27]) = 0xBFE999999999999A;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[28]) = 0x403556BDE009B366;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[29]) = 0x4021AD7BC01366C1;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[30]) = 0x3FF0000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[31]) = 0xBFE999999999999A;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[32]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[33]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[34]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[35]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[36]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[37]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[38]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[39]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[40]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[41]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[42]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[43]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[44]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[45]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[46]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&motor_factors_data[47]) = 0x0000000000000000;

    // rest of h array is populated from constraints matrix
    *reinterpret_cast<uint64_t *>(&h[17]) = 0x3FF0000000000000;
    *reinterpret_cast<uint64_t *>(&h[18]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&h[19]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&h[20]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&h[21]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&h[22]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&h[23]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&h[24]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&h[25]) = 0x0000000000000000;
    *reinterpret_cast<uint64_t *>(&h[26]) = 0x0000000000000000;

    // rest of input matrix is set at runtime
    *reinterpret_cast<uint64_t *>(&F[8]) = 0x3FF0000000000000;

    // inputs to ECOS format
    // hardcoded G matrix for octa X
    *reinterpret_cast<uint64_t *>(&G.values[0]) = 0x3FC0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[1]) = 0x3FF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[2]) = 0xBFF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[3]) = 0x3FFEBF9C62A1B5C8;
    *reinterpret_cast<uint64_t *>(&G.values[4]) = 0xBFF5839042D8C2A4;
    *reinterpret_cast<uint64_t *>(&G.values[5]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[6]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[7]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[8]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[9]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[10]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[11]) = 0x3FC0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[12]) = 0x3FF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[13]) = 0xBFF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[14]) = 0xBFF5839042D8C2A4;
    *reinterpret_cast<uint64_t *>(&G.values[15]) = 0x3FFEBF9C62A1B5C8;
    *reinterpret_cast<uint64_t *>(&G.values[16]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[17]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[18]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[19]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[20]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[21]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[22]) = 0x3FC0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[23]) = 0x3FF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[24]) = 0xBFF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[25]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[26]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[27]) = 0x3FFEBF9C62A1B5C8;
    *reinterpret_cast<uint64_t *>(&G.values[28]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[29]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[30]) = 0xBFF5839042D8C2A4;
    *reinterpret_cast<uint64_t *>(&G.values[31]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[32]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[33]) = 0x3FC0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[34]) = 0x3FF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[35]) = 0xBFF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[36]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[37]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[38]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[39]) = 0x3FFEBF9C62A1B5C8;
    *reinterpret_cast<uint64_t *>(&G.values[40]) = 0xBFF5839042D8C2A4;
    *reinterpret_cast<uint64_t *>(&G.values[41]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[42]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[43]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[44]) = 0x3FC0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[45]) = 0x3FF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[46]) = 0xBFF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[47]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[48]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[49]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[50]) = 0xBFF5839042D8C2A4;
    *reinterpret_cast<uint64_t *>(&G.values[51]) = 0x3FFEBF9C62A1B5C8;
    *reinterpret_cast<uint64_t *>(&G.values[52]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[53]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[54]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[55]) = 0x3FC0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[56]) = 0x3FF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[57]) = 0xBFF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[58]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[59]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[60]) = 0xBFF5839042D8C2A4;
    *reinterpret_cast<uint64_t *>(&G.values[61]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[62]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[63]) = 0x3FFEBF9C62A1B5C8;
    *reinterpret_cast<uint64_t *>(&G.values[64]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[65]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[66]) = 0x3FC0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[67]) = 0x3FF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[68]) = 0xBFF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[69]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[70]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[71]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[72]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[73]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[74]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[75]) = 0x3FFEBF9C62A1B5C8;
    *reinterpret_cast<uint64_t *>(&G.values[76]) = 0xBFF5839042D8C2A4;
    *reinterpret_cast<uint64_t *>(&G.values[77]) = 0x3FC0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[78]) = 0x3FF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[79]) = 0xBFF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[80]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[81]) = 0x3FD278183F91E647;
    *reinterpret_cast<uint64_t *>(&G.values[82]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[83]) = 0x3FF1702602C9081C;
    *reinterpret_cast<uint64_t *>(&G.values[84]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[85]) = 0xBFF38461F9F01B86;
    *reinterpret_cast<uint64_t *>(&G.values[86]) = 0xBFF5839042D8C2A4;
    *reinterpret_cast<uint64_t *>(&G.values[87]) = 0x3FFEBF9C62A1B5C8;
    *reinterpret_cast<uint64_t *>(&G.values[88]) = 0xBFF0000000000000;
    *reinterpret_cast<uint64_t *>(&G.values[89]) = 0x3FF0000000000000;

    G.rows[0] = 0;
    G.rows[1] = 1;
    G.rows[2] = 9;
    G.rows[3] = 18;
    G.rows[4] = 19;
    G.rows[5] = 20;
    G.rows[6] = 21;
    G.rows[7] = 22;
    G.rows[8] = 23;
    G.rows[9] = 24;
    G.rows[10] = 25;
    G.rows[11] = 0;
    G.rows[12] = 2;
    G.rows[13] = 10;
    G.rows[14] = 18;
    G.rows[15] = 19;
    G.rows[16] = 20;
    G.rows[17] = 21;
    G.rows[18] = 22;
    G.rows[19] = 23;
    G.rows[20] = 24;
    G.rows[21] = 25;
    G.rows[22] = 0;
    G.rows[23] = 3;
    G.rows[24] = 11;
    G.rows[25] = 18;
    G.rows[26] = 19;
    G.rows[27] = 20;
    G.rows[28] = 21;
    G.rows[29] = 22;
    G.rows[30] = 23;
    G.rows[31] = 24;
    G.rows[32] = 25;
    G.rows[33] = 0;
    G.rows[34] = 4;
    G.rows[35] = 12;
    G.rows[36] = 18;
    G.rows[37] = 19;
    G.rows[38] = 20;
    G.rows[39] = 21;
    G.rows[40] = 22;
    G.rows[41] = 23;
    G.rows[42] = 24;
    G.rows[43] = 25;
    G.rows[44] = 0;
    G.rows[45] = 5;
    G.rows[46] = 13;
    G.rows[47] = 18;
    G.rows[48] = 19;
    G.rows[49] = 20;
    G.rows[50] = 21;
    G.rows[51] = 22;
    G.rows[52] = 23;
    G.rows[53] = 24;
    G.rows[54] = 25;
    G.rows[55] = 0;
    G.rows[56] = 6;
    G.rows[57] = 14;
    G.rows[58] = 18;
    G.rows[59] = 19;
    G.rows[60] = 20;
    G.rows[61] = 21;
    G.rows[62] = 22;
    G.rows[63] = 23;
    G.rows[64] = 24;
    G.rows[65] = 25;
    G.rows[66] = 0;
    G.rows[67] = 7;
    G.rows[68] = 15;
    G.rows[69] = 18;
    G.rows[70] = 19;
    G.rows[71] = 20;
    G.rows[72] = 21;
    G.rows[73] = 22;
    G.rows[74] = 23;
    G.rows[75] = 24;
    G.rows[76] = 25;
    G.rows[77] = 0;
    G.rows[78] = 8;
    G.rows[79] = 16;
    G.rows[80] = 18;
    G.rows[81] = 19;
    G.rows[82] = 20;
    G.rows[83] = 21;
    G.rows[84] = 22;
    G.rows[85] = 23;
    G.rows[86] = 24;
    G.rows[87] = 25;
    G.rows[88] = 17;
    G.rows[89] = 26;

    G.column[0] = 0;
    G.column[1] = 11;
    G.column[2] = 22;
    G.column[3] = 33;
    G.column[4] = 44;
    G.column[5] = 55;
    G.column[6] = 66;
    G.column[7] = 77;
    G.column[8] = 88;
    G.column[9] = 90;

/*
    idxint cone_dim[1] = {num_motors + 2};
    solution = ECOS_setup(num_motors + 1,                   // Number of variables
                          num_constraints + num_motors + 2, // Number of inequalities, number of rows of G
                          0,                                // Number of equality constraints
                          num_constraints,                  // Dimension of positive orthant
                          1,                                // Number of second order cones
                          cone_dim,                         // Array of length 'ncones', defines the dimension of each cone
                          0,                                // Number of exponential cones
                          G.values,                         // Sparse G matrix data array (column compressed storage)
                          G.column,                         // Sparse G matrix column index array (column compressed storage)
                          G.rows,                           // Sparse G matrix row index array (column compressed storage)
                          nullptr,                          // Sparse A matrix data array (column compressed storage) (can be all NULL if no equalities are present)
                          nullptr,                          // Sparse A matrix column index array (column compressed storage) (can be all NULL if no equalities are present)
                          nullptr,                          // Sparse A matrix row index array (column compressed storage) (can be all NULL if no equalities are present)
                          F,                                // Array of size n, cost function weights
                          h,                                // Array of size m, RHS vector of cone constraint
                          nullptr);                         // Array of size p, RHS vector of equalities (can be NULL if no equalities are present)

*/
    idxint cone_dim[1] = {num_motors + 2};
    solution = ECOS_setup(num_motors + 1,                   // Number of variables
                          num_constraints + num_motors + 2, // Number of inequalities, number of rows of G
                          num_constraints,                  // Dimension of positive orthant
                          1,                                // Number of second order cones
                          cone_dim,                         // Array of length 'ncones', defines the dimension of each cone
                          G.values,                         // Sparse G matrix data array (column compressed storage)
                          G.column,                         // Sparse G matrix column index array (column compressed storage)
                          G.rows,                           // Sparse G matrix row index array (column compressed storage)
                          F,                                // Array of size n, cost function weights
                          h);                               // Array of size m, RHS vector of cone constraint
}


// output - sends commands to the motors, 
void AP_MotorsMatrix_Optimal::output_armed_stabilizing()
{
    if (!initialised_ok()) {
        return;
    }
#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
#if DISABLE_INTERRUPTS_FOR_TIMMING
    void *istate = hal.scheduler->disable_interrupts_save();
#endif
    const uint32_t start_us = AP_HAL::micros();
#endif

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain(); // compensation for battery voltage and altitude
    inputs[0] = (_roll_in + _roll_in_ff) * compensation_gain;
    inputs[1] = (_pitch_in + _pitch_in_ff) * compensation_gain;
    inputs[2] = (_yaw_in + _yaw_in_ff) * compensation_gain;
    inputs[3] = get_throttle() * compensation_gain;
    float throttle_avg_max = _throttle_avg_max * compensation_gain;

    // If thrust boost is active then do not limit maximum thrust
    const float throttle_thrust_max = _thrust_boost_ratio + (1.0 - _thrust_boost_ratio) * _throttle_thrust_max * compensation_gain;

    // sanity check throttle is above zero and below current limited throttle
    inputs[3] = constrain_float(inputs[3], 0.0, throttle_thrust_max);

    // ensure that throttle_avg_max is between the input throttle and the maximum throttle
    throttle_avg_max = constrain_float(throttle_avg_max, inputs[3], throttle_thrust_max);

    //throttle_avg_max = 0.5;

    //inputs[0] = 0.0;
    //inputs[1] = 0.0;
    //inputs[2] = 0.0;
    //inputs[3] = 0.5;

    // calculate input vector
    mat_vec_mult(motor_factors, inputs, F);

    // constraints, average throttle, 1 and 0
    h[0] = throttle_avg_max;
    for (uint8_t i = 0; i < num_motors; i++) {
        h[1+i] = 1.0; // could set this to 0 if a failed motor is detected
        h[1+num_motors+i] = 0.0;
    }

    ECOS_solve(solution);

    for (uint8_t i = 0; i < num_motors; i++) {
        x[i] = solution->x[i];
    }

    //ECOS_cleanup(solution, 0);

    // workout what output was achieved
    mat_vec_mult(motor_factors_trans, x, outputs);
    outputs[2] /= num_motors*0.25;
    outputs[3] /= num_motors;

    // set limit flags, threshold of 1%
    const float threshold = 0.01;
    limit.roll = fabsf(inputs[0] - outputs[0]) > threshold ? 1 : 0;
    limit.pitch = fabsf(inputs[1] - outputs[1]) > threshold ? 1 : 0;
    limit.yaw = fabsf(inputs[2] - outputs[2]) > threshold ? 1 : 0;
    limit.throttle_lower = (outputs[3] - inputs[3]) > threshold ? 1 : 0;
    limit.throttle_upper = (inputs[3] - outputs[3]) > threshold ? 1 : 0;

    // copy to motor outputs
    for (uint8_t i = 0; i < num_motors; i++) {
        _thrust_rpyt_out[i] = x[i];
    }

    // determine throttle thrust for harmonic notch
    // compensation_gain can never be zero
    _throttle_out = outputs[3] / compensation_gain;

#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    const uint32_t end_us = AP_HAL::micros();

#if DISABLE_INTERRUPTS_FOR_TIMMING
    hal.scheduler->restore_interrupts(istate);
#endif

    // @LoggerMessage: MMIX
    // @Description: Motor mixer data
    // @Field: TimeUS: Time since system startup
    // @Field: Run: Runtime
    // @Field: iter: number of iterations
    // @Field: Conv: Converged to a solution
    // @Field: dR: desired roll
    // @Field: dP: desired pitch
    // @Field: dY: desired yaw
    // @Field: dT: desired throttle
    // @Field: aR: achieved roll
    // @Field: aP: achieved pitch
    // @Field: aY: achieved yaw
    // @Field: aT: achieved throttle
    AP::logger().Write("MMIX", "TimeUS,Run,iter,Conv,dR,dP,dY,dT,aR,aP,aY,aT", "QIBBffffffff",
                                           AP_HAL::micros64(),
                                           end_us - start_us,
                                           iter,
                                           converged ? 1 : 0,
                                           inputs[0],inputs[1],inputs[2],inputs[3],
                                           outputs[0],outputs[1],outputs[2],outputs[3]);
#endif

    //set_initialised_ok(false);
}
/*

// sparse A matrix handling
// x = A * B
void AP_MotorsMatrix_Optimal::A_mult(const float*B, float*dest) const
{
    for (uint8_t i = 0; i < num_motors; i++) {
        dest[i] = A.average_throttle[i]*B[0] + A.max_throttle[i]*B[1+i] + A.min_throttle[i]*B[1+num_motors+i];
    }
}

// x = A' * B
void AP_MotorsMatrix_Optimal::At_mult(const float*B, float*dest) const
{
    dest[0] = 0;
    for (uint8_t i = 0; i < num_motors; i++) {
        dest[0] += A.average_throttle[i]*B[i];
        dest[1+i] = A.max_throttle[i]*B[i];
        dest[1+num_motors+i] = A.min_throttle[i]*B[i];
    }
}

// H_temp = H + A*diag(z .* s_inv)*A'
// Note that we only calculate the lower triangle, matrix is expected to be symmetric
void AP_MotorsMatrix_Optimal::H_plus_A_mult_b_mult_At()
{
    for (uint8_t i = 0; i < num_motors; i++) {
        for (uint8_t j = 0; j <= i; j++) {
            H_bar_data[i*num_motors+j] = A.average_throttle[i] * A.average_throttle[j] * z[0] * s_inv[0];
        }
        H_bar_data[i*num_motors+i] += A.max_throttle[i]*A.max_throttle[i]*z[i+1]*s_inv[i+1] + A.min_throttle[i]*A.min_throttle[i]*z[i+num_motors+1]*s_inv[i+num_motors+1];
        for (uint8_t j = 0; j <= i; j++) {
            // MATLAB thinks this is better than adding first, floating point stuff I guess
            H_bar_data[i*num_motors+j] += H_data[i*num_motors+j];
        }
    }
}

// interior point method quadratic programming solver
// Inspired by: https://github.com/jarredbarber/eigen-QP
// solves min( 0.5*x'Hx + f'x )
// with constraints A'x >= b
void AP_MotorsMatrix_Optimal::interior_point_solve()
{
    // setup starting points
    for (uint8_t i = 0; i < num_motors; i++) {
        x[i] = 0.0;

        // rL = f - A*z
        rL[i] = f[i] - A.average_throttle[i] - A.max_throttle[i] - A.min_throttle[i];
    }
    for (uint8_t i = 0; i < num_constraints; i++) {
        z[i] = 1.0;
        s[i] = 1.0;
        s_inv[i] = 1.0;
        rsz[i] = 1.0;

        // rs = s + b
        rs[i] = 1.0 + b[i];
    }

    constexpr float eta = 0.95;
    constexpr float tol = 1e-5;
    constexpr float tol_sq = tol * tol;

    float tol_nA = tol * num_constraints;
    float mu = num_constraints;

    // limit to 10 iterations
    converged = false;
    for (iter = 0; iter < 10; iter++) {

        // Pre-decompose to speed up solve
        // H_bar = H + A*diag(z .* s_inv)*A'
        H_plus_A_mult_b_mult_At();

        // H_bar = chol(H_temp)
        if (!cholesky(H_bar)) {
            // Stop propogating numberical issues
            // could raise an internal error
            // moving to double would fix
            return;
        }

        float alpha;
        for (uint8_t i = 0; i < 2; i++) {
            // centring on fist iteration, correction on second

            // Solve system
            // f_bar = rL + A*((rsz-z*rs).*s_inv)
            for (uint8_t j = 0; j < num_constraints; j++) {
                // (rsz-z*rs).*s_inv
                temp_con[j] = (rsz[j] - z[j]*rs[j]) * s_inv[j];
            }
            A_mult(temp_con, f_bar);
            vec_add(f_bar, rL, f_bar, num_motors);

            forward_sub(H_bar, f_bar, dx);
            backward_sub_t(H_bar, dx, dx);

            // ds = A'dx + rs
            At_mult(dx, ds);

            alpha = 1.0;
            for (uint8_t j = 0; j < num_constraints; j++) {

                // ds = A'dx + rs
                ds[j] += rs[j];

                // dz = (rsz - z.*ds).*s_inv
                dz[j] = (rsz[j] - z[j]*ds[j]) * s_inv[j];

                // Compute alpha
                if (dz[j] > 0) {
                    alpha = MIN(alpha, z[j]/dz[j]);
                }
                if (ds[j] > 0) {
                    alpha = MIN(alpha, s[j]/ds[j]);
                }
            }

            if (i == 1) {
                break;
            }

            // affine duality gap and centering
            float mu_a = 0;
            const float offset = (mu_a*mu_a*mu_a)/(mu*mu*num_constraints);
            for (uint8_t j = 0; j < num_constraints; j++) {
                 // mu_a = dot(z-dz*alpha,s-ds*alpha)
                mu_a += (z[j]-dz[j]*alpha)*(s[j]*ds[j]*alpha);

                // apply centring parameter
                // rsz += ds.*dz - ((mu_a^3)/(num_constraints*mu^2))
                rsz[j] += ds[j]*dz[j] - offset;
            }

        }
        const float relax = alpha*eta;
        // Update x, z, s
        for (uint8_t j = 0; j < num_motors; j++) {
            // x -= dx*alpha*eta
            x[j] -= dx[j]*relax;
        }
        At_mult(x,temp_con);
        mu = 0.0; 
        float dot = 0.0;
        for (uint8_t j = 0; j < num_constraints; j++) {
            // z -= dz*alpha*eta
            z[j] -= dz[j]*relax;

            // s -= ds*alpha*eta
            s[j] -= ds[j]*relax;

            // mu = dot(z,s)
            mu += z[j]*s[j];

            // rs = s - A'*x + b
            rs[j] = s[j] - temp_con[j] + b[j];
            dot += rs[j]*rs[j];
        }
        if ((dot < tol_sq) || (mu < tol_nA)) {
            converged = true;
            return;
        }
        // rL = H*x + f - A*z
        mat_vec_mult(H, x, rL);
        A_mult(z, temp_mot);
        dot = 0.0;
        for (uint8_t j = 0; j < num_motors; j++) {
            rL[j] += f[j] - temp_mot[j];
            dot += rL[j]*rL[j];
        }
        if (dot < tol_sq) {
            converged = true;
            return;
        }

        // update values for next iteration
        // rsz = s.*z
        // s_inv = 1./s
        for (uint8_t j = 0; j < num_constraints; j++) {
            rsz[j] = s[j]*z[j];
            s_inv[j] = 1.0/s[j];
        }

    }
}
*/
const char* AP_MotorsMatrix_Optimal::_get_frame_string() const
{
    if (frame_string != nullptr) {
        return frame_string;
    }
    return AP_MotorsMatrix::_get_frame_string();
}

#endif // AP_MOTOR_FRAME_OPTIMAL_ENABLED
