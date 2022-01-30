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
  Simulator for the IntelligentEnergy 2.4kWh FuelCell generator
*/

#include <AP_Math/AP_Math.h>

#include "SIM_IntelligentEnergy_PPM.h"
#include "SITL.h"

#include <errno.h>

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo IntelligentEnergyPPM::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: IntelligentEnergy 2.4kWh FuelCell sim enable/disable
    // @Description: Allows you to enable (1) or disable (0) the FuelCell simulator
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, IntelligentEnergyPPM, enabled, 0),

    // @Param: STATE1
    // @DisplayName: Explicitly set state
    // @Description: Explicitly specify a state for the generator to be in
    // @User: Advanced
    AP_GROUPINFO("STATE1", 2, IntelligentEnergyPPM, set_state1, -1),

    // @Param: ERROR1
    // @DisplayName: Explicitly set error code
    // @Description: Explicitly specify an error code to send to the generator
    // @User: Advanced
    AP_GROUPINFO("ERROR1", 3, IntelligentEnergyPPM, err_code1, 0),

    // @Param: STATE2
    // @DisplayName: Explicitly set state
    // @Description: Explicitly specify a state for the generator to be in
    // @User: Advanced
    AP_GROUPINFO("STATE2", 4, IntelligentEnergyPPM, set_state2, -1),

    // @Param: ERROR2
    // @DisplayName: Explicitly set error code
    // @Description: Explicitly specify an error code to send to the generator
    // @User: Advanced
    AP_GROUPINFO("ERROR2", 5, IntelligentEnergyPPM, err_code2, 0),

    AP_GROUPEND
};

IntelligentEnergyPPM::IntelligentEnergyPPM() : IntelligentEnergy::IntelligentEnergy()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void IntelligentEnergyPPM::update(const struct sitl_input &input)
{
    if (!enabled.get()) {
        return;
    }
    update_send();
}

void IntelligentEnergyPPM::update_send()
{
    // just send a chunk of data at 1Hz:
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < 500) {
        return;
    }

    // Simulate constant current charge/discharge of the battery
    float amps = discharge ? -20.0f : 20.0f;

    // Simulate constant tank pressure. This isn't true in reality, but is good enough
    const float tank_bar = 250;

    // Update pack capacity remaining
    bat_capacity_mAh += amps*(now - last_sent_ms)/3600.0f;

    // From capacity remaining approximate voltage by linear interpolation
    const float min_bat_vol = 42.0f;
    const float max_bat_vol = 50.4f;
    const float max_bat_capactiy_mAh = 3300;

    battery_voltage = constrain_float(bat_capacity_mAh / max_bat_capactiy_mAh * (max_bat_vol - min_bat_vol) + min_bat_vol, min_bat_vol, max_bat_vol);

    // Decide if we need to charge or discharge the battery
    if (battery_voltage <= min_bat_vol) {
        discharge = false;
    } else if (battery_voltage >= max_bat_vol) {
        discharge = true;
    }

    float battery_pwr = battery_voltage * amps; // Watts

    // These are non-physical values
    const float pwr_out = battery_pwr*1.4f;
    //const uint32_t spm_pwr = battery_pwr*0.3f;

    uint32_t state1 = set_state1;
    if (set_state1 == -1) {
        state1 = 2; // Running
    }

    uint32_t state2 = set_state2;
    if (set_state2 == -1) {
        state2 = 2; // Running
    }

    last_sent_ms = now;

    char message[128];
    hal.util->snprintf(message, ARRAY_SIZE(message), "<%.1f,%.1f,%.1f,%u,%u,%.1f,%.1f,%u,%u,%.1f,%.1f,%.1f,%.1f,%u>\n",
             now*0.001,
             91.1,
             71.1,
             state1,
             (uint32_t)err_code1,
             92.2,
             72.2,
             state2,
             (uint32_t)err_code2,
             tank_bar,
             pwr_out,
             battery_voltage,
             amps,
             0);

    if ((unsigned)write_to_autopilot(message, strlen(message)) != strlen(message)) {
        AP_HAL::panic("Failed to write to autopilot: %s", strerror(errno));
    }
   ::printf("%s",message);
}
