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

#include "AP_BattMonitor_IE_PPM.h"

#if GENERATOR_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Generator/AP_Generator_IE_PPM.h>

extern const AP_HAL::HAL& hal;

/*
    Fuel class
*/
// This is where we tell the battery monitor 'we have current' if we want to report a fuel level remaining
bool AP_BattMonitor_IE_PPM_Fuel::has_current(void) const
{
    switch ((AP_BattMonitor::Type)_params._type.get())
    {
    case AP_BattMonitor::Type::GENERATOR_IE_PPM_FUEL_1:
    case AP_BattMonitor::Type::GENERATOR_IE_PPM_FUEL_2:
        return true;
    default:
        return false;
    }
}

// This is where we tell the battery monitor 'we have consummed energy' if we want to report a fuel level remaining
bool AP_BattMonitor_IE_PPM_Fuel::has_consumed_energy(void) const
{
    switch ((AP_BattMonitor::Type)_params._type.get())
    {
    case AP_BattMonitor::Type::GENERATOR_IE_PPM_FUEL_1:
    case AP_BattMonitor::Type::GENERATOR_IE_PPM_FUEL_2:
        return true;
    default:
        return false;
    }
}

void AP_BattMonitor_IE_PPM_Fuel::init()
{
    switch ((AP_BattMonitor::Type)_params._type.get())
    {
    case AP_BattMonitor::Type::GENERATOR_IE_PPM_FUEL_1:
    case AP_BattMonitor::Type::GENERATOR_IE_PPM_FUEL_2:
        // Set params for users:
        // Fuel level is only reported as a percentage
        _params._pack_capacity.set(100);
        // Fuel only reports a fixed 1v, don't want batt monitor failsafes on this instance
        _params._low_voltage.set(0);
        _params._critical_voltage.set(0);
        break;
    default:
        break;
    }
}

// Read the fuel level.  Should be called at 10hz
void AP_BattMonitor_IE_PPM_Fuel::read()
{
    _state.healthy = false;

    AP_Generator_IE_PPM * PPM = AP_Generator_IE_PPM::get_singleton();

    // Not healthy if we can't find a generator
    if (PPM == nullptr) {
        return;
    }

    if (!PPM->healthy()) {
        return;
    }

    switch ((AP_BattMonitor::Type)_params._type.get())
    {
    case AP_BattMonitor::Type::GENERATOR_IE_PPM_FUEL_1:
        _state.voltage = 1.0;
        _state.consumed_mah = (1 - PPM->get_fuel_remain_1()) * _params._pack_capacity.get();
        break;

    case AP_BattMonitor::Type::GENERATOR_IE_PPM_FUEL_2:
        _state.voltage = 1.0;
        _state.consumed_mah = (1 - PPM->get_fuel_remain_2()) * _params._pack_capacity.get();
        break;

    default:
        return;
    }

    // If we got this far then must be healthy
    _state.healthy = true;
    _state.last_time_micros = AP_HAL::micros();
}

/*
    Electrical class
*/
void AP_BattMonitor_IE_PPM_Elec::init()
{
    switch ((AP_BattMonitor::Type)_params._type.get())
    {
    case AP_BattMonitor::Type::GENERATOR_IE_PPM_ELEC_1:
    case AP_BattMonitor::Type::GENERATOR_IE_PPM_ELEC_2:
        // Set params for users:
        // Fuel level is only reported as a percentage
        _params._pack_capacity.set(100);
        // Fuel only reports a fixed 1v, don't want batt monitor failsafes on this instance
        _params._low_voltage.set(0);
        _params._critical_voltage.set(0);
        break;
    default:
        break;
    }
}

bool AP_BattMonitor_IE_PPM_Elec::has_current(void) const
{
    return true;
}

bool AP_BattMonitor_IE_PPM_Elec::has_consumed_energy(void) const
{
    return true;
}

// Read the electrical measurements from the generator
void AP_BattMonitor_IE_PPM_Elec::read()
{
    _state.healthy = false;

    AP_Generator_IE_PPM * PPM = AP_Generator_IE_PPM::get_singleton();

    // Not healthy if we can't find a generator
    if (PPM == nullptr) {
        return;
    }

    if (!PPM->healthy()) {
        return;
    }

    switch ((AP_BattMonitor::Type)_params._type.get())
    {
    case AP_BattMonitor::Type::GENERATOR_IE_PPM_ELEC_1:
        _state.voltage = 1.0;
        _state.consumed_mah = (1 - PPM->get_batt_remain_1()) * _params._pack_capacity.get();
        break;

    case AP_BattMonitor::Type::GENERATOR_IE_PPM_ELEC_2:
        _state.voltage = 1.0;
        _state.consumed_mah = (1 - PPM->get_batt_remain_2()) * _params._pack_capacity.get();
        break;

    case AP_BattMonitor::Type::GENERATOR_IE_PPM_ELEC_combined:
        _state.voltage = PPM->get_PPM_voltage();
        _state.current_amps = PPM->get_PPM_current();
        break;
    default:
        return;
    }

    // If we got this far then must be healthy
    _state.healthy = true;
    _state.last_time_micros = AP_HAL::micros();
}

AP_BattMonitor::Failsafe AP_BattMonitor_IE_PPM_Elec::update_failsafes()
{

    if ((AP_BattMonitor::Type)_params._type.get() != AP_BattMonitor::Type::GENERATOR_IE_PPM_ELEC_combined) {
        return AP_BattMonitor_Backend::update_failsafes();
    }
    AP_Generator *generator = AP::generator();

    AP_BattMonitor::Failsafe failsafe = AP_BattMonitor::Failsafe::None;
    if (generator != nullptr) {
        failsafe = generator->update_failsafes();
    }
    return MAX(AP_BattMonitor_Backend::update_failsafes(), failsafe);
}
#endif
