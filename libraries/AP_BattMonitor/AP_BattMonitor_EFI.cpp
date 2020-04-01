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
#if EFI_ENABLED
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
    const uint32_t now = AP_HAL::micros();

    float proportion_remaining = 0.0f;
    switch (_params._type) {
        case AP_BattMonitor_Params::BattMonitor_TYPE_EFI_TANK:
            proportion_remaining = EFI->get_tank_pct();
            _state.last_time_micros = now;

            // map consumed_mah to consumed percentage
            _state.consumed_mah = (1 - (proportion_remaining * 0.01f)) * _params._pack_capacity;

            // map consumed_wh using fixed voltage of 1
            _state.consumed_wh = _state.consumed_mah;

            break;

        case AP_BattMonitor_Params::BattMonitor_TYPE_EFI_BATTERY:
            float voltage;
            float current;
            float mah;
            if (EFI->get_battery(voltage, current, mah)) {
                _state.voltage = voltage;
                _state.current_amps = current;

                // update total current drawn since startup
                _state.consumed_mah = mah;
                _state.consumed_wh  = 0.001f * mah * _state.voltage;

                // record time
                _state.last_time_micros = now;
            }
            break;

        default:
            _state.healthy = false;
            break;
    }

#endif
}
