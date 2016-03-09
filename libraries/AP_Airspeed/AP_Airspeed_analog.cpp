/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *   analog airspeed driver
 */
#include "AP_Airspeed_analog.h"

#include <AP_ADC/AP_ADC.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_Airspeed.h"

extern const AP_HAL::HAL &hal;

// scaling for 3DR analog airspeed sensor
#define VOLTS_TO_PASCAL 819

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_Airspeed_Analog::AP_Airspeed_Analog(AP_Airspeed &_airspeed, uint8_t instance, AP_Airspeed::Airspeed_State &_state) :
    AP_Airspeed_Backend(_airspeed, instance, _state)
{
    _source = hal.analogin->channel(_airspeed._pin[instance]);
    if (_source == NULL) {
        // failed to allocate a ADC channel? This shouldn't happen
        state[instance].status = AP_Airspeed::Airspeed_NotConnected;
        return;
    }
    _source = hal.analogin->channel(_airspeed._pin[instance]);
    set_status(AP_Airspeed::Airspeed_NoData);
}

// read the airspeed sensor
bool AP_Airspeed_Analog::get_differential_pressure(float &pressure)
{
    if (_source == NULL) {
        return false;
    }
    _source->set_pin(_pin);
    pressure = _source->voltage_average_ratiometric() * VOLTS_TO_PASCAL;
    return true;
}

bool AP_Airspeed_Analog::detect(AP_Airspeed &_airspeed, uint8_t instance, AP_Airspeed::Airspeed_State state)
{
    if (_airspeed[instance].pin != -1) {
        return true;
    }
    return false;
}
