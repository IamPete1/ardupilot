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
/*
  sailboat simulator class
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a sailboat simulator
 */
class Sailboat : public Aircraft {
public:
    Sailboat(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Sailboat(home_str, frame_str);
    }

private:

    // calculate the lift and drag as values from 0 to 1 given an apparent wind speed in m/s and angle-of-attack in degrees
    void calc_lift_and_drag(float wind_speed, float angle_of_attack_deg, float& lift, float& drag) const;

    // return turning circle (diameter) in meters for steering angle proportion in the range -1 to +1
    float get_turn_circle(float steering) const;

    // return yaw rate in deg/sec given a steering input (in the range -1 to +1) and speed in m/s
    float get_yaw_rate(float steering, float speed) const;

    // return lateral acceleration in m/s/s given a steering input (in the range -1 to +1) and speed in m/s
    float get_lat_accel(float steering, float speed) const;

    float calc_hull_drag(float speed, float heel);

    float steering_angle_max;   // vehicle steering mechanism's max angle in degrees
    float turning_circle;       // vehicle minimum turning circle diameter in meters

    // lift and drag curves.  index is angle/10deg
    // angle-of-attack            0      10     20     30     40     50     60     70     80     90     100    110    120    130    140    150    160    170+
    const float lift_curve[18] = {0.00f, 0.00f, 0.80f, 1.00f, 0.95f, 0.75f, 0.60f, 0.40f, 0.20f, 0.00f, -0.00f, -0.00f, -0.00f, -0.00f, -0.00f, -0.00f, -0.00f, -0.00f};
    const float drag_curve[18] = {0.10f, 0.10f, 0.12f, 0.15f, 0.20f, 0.27f, 0.35f, 0.50f, 0.70f, 1.00f, 0.70f, 0.50f, 0.35f, 0.27f, 0.20f, 0.15f, 0.12f, 0.10f};

    // Sail ceneter of pressure above cg
    const float sail_cp = 0.75f; // m
    const float sail_area = 0.41f;// m^2

    // Total mass of the boat
    const float mass = 4.0f; // kg

    // Hull drag coefficents for Delft Yacht Series
    // https://mail.radiosailingtechnology.com/index.php/hulls/estimating-the-hull-drag-of-an-iom-yacht-august-2014
    // Note that to large a devation from the Delft Yacht Series of data will result in inacuracys
    // Data for 'target 0.57' IOM hull from above link
    struct {
        const float AW = 0.14683f;    // Wetted Surface, m^2
        const float LWL = 0.9766f;    // Waterline Length, m
        const float BWL = 0.1668f;    // Waterline Beam, m
        const float Cp = 0.57f;       // Prismatic Coefficient
        const float Disp = 0.003601f; // Volume Displaced, m^3
        const float LCB = 0.5201f;    // C of B from forward perpendiculat, m
        const float LCF = 0.5461f;    // C of Water-plane from forward perpendicular, m
        const float AWP = 0.11186f;   // Water plane aresm, m^2
        const float Cm = 0.683f;      // Mid-ship section coefficent
        const float rho = 1025.0f;      // Water densiry, kg/m^3
        const float Tc = 0.0572f;     // Hull draft, m
        const float v = 1.13e-6f;     // Water Kinematic viscosity, m^2/s
    } hull_coef;

    /*
    // Coefficients for Delft Yacht Series look up table
    const float Coefficient_DYS[9][13] = {
    { 0.15f,    0.2f,     0.25f,    0.3f,     0.35f,    0.4f,     0.45f,    0.5f,     0.55f,    0.6f,     0.65f,    0.7f,     0.75f},   // Fn
    {-0.0005f, -0.0003f, -0.0002f, -0.0009f, -0.0026f, -0.0064f, -0.0218f, -0.0388f, -0.0347f, -0.0361f,  0.0008f,  0.0108f,  0.1023f}, // a0
    { 0.0023f,  0.0059f, -0.0156f,  0.0016f, -0.0567f, -0.4034f, -0.5261f, -0.5986f, -0.4764f,  0.0037f,  0.3728f, -0.1238f,  0.7726f}, // a1
    {-0.0086f, -0.0064f,  0.0031f,  0.0337f,  0.0446f, -0.125f,  -0.2945f, -0.3038f, -0.2361f, -0.296f,  -0.3667f, -0.2026f,  0.504f},  // a2
    {-0.0015f,  0.007f,  -0.0021f, -0.0285f, -0.1091f,  0.0273f,  0.2485f,  0.6033f,  0.8726f,  0.9661f,  1.3957f,  1.1282f,  1.7867f}, // a3
    { 0.0061f,  0.0014f, -0.007f,  -0.0367f, -0.0707f, -0.1341f, -0.2428f, -0.043f,   0.4219f,  0.6123f,  1.0343f,  1.1836f,  2.1934f}, // a4
    { 0.001f,   0.0013f,  0.0148f,  0.0218f,  0.0914f,  0.3578f,  0.6293f,  0.8332f,  0.899f,   0.7534f,  0.3230f,  4973.0f,   -1.5479f}, // a5
    { 0.0001f,  0.0005f,  0.001f,   0.0015f,  0.0021f,  0.0045f,  0.0081f,  0.0106f,  0.0096f,  0.01f,    0.0072f,  0.0038f, -0.0115f}, // a6
    { 0.0052f, -0.002f,  -0.0043f, -0.0172f, -0.0078f,  0.1115f,  0.2086f,  0.1336f, -0.2272f, -0.3352f, -0.4632f, -0.4477f, -0.0977f}  // a7
    };

    // Coefficients for Delft Yacht Series heel angle
    const float Coefficient_DYS_heel[7][7] = {
    { 0.25f,    0.3f,     0.35f,    0.4f,     0.45f,    0.5f,     0.55f},   // Fn
    {-0.0005f, -0.0003f, -0.0002f, -0.0009f, -0.0026f, -0.0064f, -0.0218f}, // u0
    { 0.0023f,  0.0059f, -0.0156f,  0.0016f, -0.0567f, -0.4034f, -0.5261f}, // u1
    {-0.0086f, -0.0064f,  0.0031f,  0.0337f,  0.0446f, -0.125f,  -0.2945f}, // u2
    {-0.0015f,  0.007f,  -0.0021f, -0.0285f, -0.1091f,  0.0273f,  0.2485f}, // u3
    { 0.0061f,  0.0014f, -0.007f,  -0.0367f, -0.0707f, -0.1341f, -0.2428f}, // u4
    { 0.001f,   0.0013f,  0.0148f,  0.0218f,  0.0914f,  0.3578f,  0.6293f}, // u5
    };

    // Coefficients for viscous heel angle
    const float Coefficient_visc_heel[5][7] = {
    { 5.0f,    10.f,     15.0f,    20.0f,     25.0f,    30.0f,    35.0f},   // heel angle deg
    {-0.0005f, -0.0003f, -0.0002f, -0.0009f, -0.0026f, -0.0064f, -0.0218f}, // s0
    { 0.0023f,  0.0059f, -0.0156f,  0.0016f, -0.0567f, -0.4034f, -0.5261f}, // s1
    {-0.0086f, -0.0064f,  0.0031f,  0.0337f,  0.0446f, -0.125f,  -0.2945f}, // s2
    {-0.0015f,  0.007f,  -0.0021f, -0.0285f, -0.1091f,  0.0273f,  0.2485f}, // s3
    };
*/
    // Righting force
    const float keel_mass = 2.0f; // kg
    const float keel_lenght = 0.3f; // m
};

} // namespace SITL
