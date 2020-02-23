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

#include "AP_EFI.h"

#if EFI_ENABLED

#include "AP_EFI_Serial_MS.h"
#include "AP_EFI_ECU_Lite.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_EFI::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: EFI communication type
    // @Description: What method of communication is used for EFI #1
    // @Values: 0:None,1:Serial-MS,2:ECU-Lite
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_EFI, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _COEF1
    // @DisplayName: EFI Calibration Coefficient 1
    // @Description: Used to calibrate fuel flow for MS protocol (Slope)
    // @Range: 0 1
    // @User: Advanced
    // @RebootRequired: False
    AP_GROUPINFO("_COEF1", 2, AP_EFI, coef1, 0),

    // @Param: _COEF2
    // @DisplayName: EFI Calibration Coefficient 2
    // @Description: Used to calibrate fuel flow for MS protocol (Offset)
    // @Range: 0 10
    // @User: Advanced
    // @RebootRequired: False
    AP_GROUPINFO("_COEF2", 3, AP_EFI, coef2, 0),

    AP_GROUPEND
};

AP_EFI *AP_EFI::singleton;

// Initialize parameters
AP_EFI::AP_EFI()
{
    singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// Initialize backends based on existing params
void AP_EFI::init(void)
{
    if (backend != nullptr) {
        // Init called twice, perhaps
        return;
    }

    switch(type) {
        case EFI_COMMUNICATION_TYPE_SERIAL_MS :
            backend = new AP_EFI_Serial_MS(*this);
            break;

        case EFI_Communication_Type_ECU_LITE:
            backend = new AP_EFI_ECU_Lite(*this);
            break;

    }
}

// Ask all backends to update the frontend
void AP_EFI::update()
{
    if (backend) {
        backend->update();
    }
}

bool AP_EFI::is_healthy(void) const
{
    if (backend) {
        return backend->is_healthy();
    }

    return false;
}

/*
  send EFI_STATUS
 */
void AP_EFI::send_mavlink_status(mavlink_channel_t chan)
{
    if (!backend) {
        return;
    }
    mavlink_msg_efi_status_send(
        chan,
        AP_EFI::is_healthy(),
        state.ecu_index,
        state.engine_speed_rpm,
        state.estimated_consumed_fuel_volume_cm3,
        state.fuel_consumption_rate_cm3pm,
        state.engine_load_percent,
        state.throttle_position_percent,
        state.spark_dwell_time_ms,
        state.atmospheric_pressure_kpa,
        state.intake_manifold_pressure_kpa,
        (state.intake_manifold_temperature - 273.0f),
        (state.cylinder_status[0].cylinder_head_temperature - 273.0f),
        state.cylinder_status[0].ignition_timing_deg,
        state.cylinder_status[0].injection_time_ms,
        0, 0, 0);
}

// get battery info from backend if available
bool AP_EFI::get_battery(float &voltage, float &current) const
{
    if (backend) {
        return backend->get_battery(voltage, current);
    }

    return false;
}

namespace AP {
AP_EFI *EFI()
{
    return AP_EFI::get_singleton();
}
}

#endif // EFI_ENABLED

