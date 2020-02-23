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

#include "AP_EFI_Backend.h"

extern const AP_HAL::HAL &hal;

AP_EFI_Backend::AP_EFI_Backend(AP_EFI &_frontend) :
    frontend(_frontend)
{
    internal_state.estimated_consumed_fuel_volume_cm3 = 0; // Just to be sure
}

void AP_EFI_Backend::copy_to_frontend() 
{
    WITH_SEMAPHORE(sem);
    frontend.state = internal_state;
}

float AP_EFI_Backend::get_coef1(void) const
{
    return frontend.coef1;
}

float AP_EFI_Backend::get_coef2(void) const
{
    return frontend.coef2;
}

bool AP_EFI_Backend::is_healthy()
{
    return (AP_HAL::millis() - internal_state.last_updated_ms) < HEALTHY_LAST_RECEIVED_MS;
}
#endif // EFI_ENABLED
