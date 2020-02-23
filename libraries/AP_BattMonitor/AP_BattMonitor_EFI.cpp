#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor_EFI.h"
#include "AP_BattMonitor.h"

extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_EFI::AP_BattMonitor_EFI(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    _state.voltage = 1.0; // show a fixed voltage of 1v
}

// read - read the voltage and current
void AP_BattMonitor_EFI::read()
{
    // get fuel cell lib pointer
    const AP_EFI* EFI = AP_EFI::get_singleton();
    if (EFI == nullptr) {
        _state.healthy = false;
        return;
    }
    // check that it is enabled
    if (!EFI->enabled()) {
        _state.healthy = false;
        return;
    }
    _state.healthy = EFI->is_healthy();

    float proportion_remaining = 0.0f;
    switch (_params._type) {
        case AP_BattMonitor_Params::BattMonitor_TYPE_EFI_TANK:
            proportion_remaining = EFI->get_tank_pct();
            _state.last_time_micros = AP_HAL::micros();

            // map consumed_mah to consumed percentage
            _state.consumed_mah = (1 - proportion_remaining) * _params._pack_capacity;

            // map consumed_wh using fixed voltage of 1
            _state.consumed_wh = _state.consumed_mah;

            break;

        case AP_BattMonitor_Params::BattMonitor_TYPE_EFI_BATTERY:
            float voltage;
            float current;
            if (EFI->get_battery(voltage, current)) {
                _state.last_time_micros = AP_HAL::micros();
                _state.voltage = voltage;
                _state.current_amps = current;
            }
            break;
        default:
            _state.healthy = false;
            break;
    }


}
