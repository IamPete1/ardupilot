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

#include "AP_ADSB_Backend.h"

#if HAL_ADSB_ENABLED

/*
  base class constructor.
*/
AP_ADSB_Backend::AP_ADSB_Backend(AP_ADSB &frontend, uint8_t instance) :
    _frontend(frontend),
    _instance(instance)
{
}

/**
* @brief Convert base 8 or 16 to decimal. Used to convert an octal/hexadecimal value stored on a GCS as a string field in different format, but then transmitted over mavlink as a float which is always a decimal.
* baseIn: base of input number
* inputNumber: value currently in base "baseIn" to be converted to base "baseOut"
*
* Example: convert ADSB squawk octal "1200" stored in memory as 0x0280 to 0x04B0
*          uint16_t squawk_decimal = convertMathBase(8, squawk_octal);
*/
uint32_t AP_ADSB_Backend::convert_base_to_decimal(const uint8_t baseIn, uint32_t inputNumber)
{
    // Our only sensible input bases are 16 and 8
    if (baseIn != 8 && baseIn != 16) {
        return inputNumber;
    }
    uint32_t outputNumber = 0;
    for (uint8_t i=0; i < 10; i++) {
        outputNumber += (inputNumber % 10) * powf(baseIn, i);
        inputNumber /= 10;
        if (inputNumber == 0) break;
    }
    return outputNumber;
}

#endif // HAL_ADSB_ENABLED

