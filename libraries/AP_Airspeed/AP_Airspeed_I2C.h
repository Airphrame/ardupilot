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
  backend driver for airspeed from I2C
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "Airspeed_Backend.h"

class AP_Airspeed_I2C : public AP_Airspeed_Backend
{
public:
    // constructor
    AP_Airspeed_I2C(AP_Airspeed &_frontend, uint8_t instance, AP_Airspeed::Airspeed_State &_state);

    // destructor
    ~AP_Airspeed_I2C(void);

    // static detection function
    static bool detect(AP_Airspeed &frontend, uint8_t instance);

    // update state
    void update(void) { }

    bool get_temperature(float &temperature)
    {
        if (state.status == AP_Airspeed::Airspeed_Good) {
            temperature = state.temperature;
            return true;
        }
        return false;
    }

private:
    void _measure();
    void _collect();
    void _timer();
    uint32_t _last_sample_time_ms;
    uint32_t _measurement_started_ms;
};
