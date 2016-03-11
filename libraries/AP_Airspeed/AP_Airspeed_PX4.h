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
  backend driver for airspeed from PX4Firmware
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "Airspeed_Backend.h"

class AP_Airspeed_PX4 : public AP_Airspeed_Backend {
public:
    // constructor
    AP_Airspeed_PX4(AP_Airspeed &_frontend, uint8_t instance, AP_Airspeed::Airspeed_State &_state);

    // destructor
    ~AP_Airspeed_PX4(void);

    // static detection function
    static bool detect(AP_Airspeed &frontend, uint8_t instance);

    // update state
    void update(void);

    bool get_temperature(float &temperature)
    {
        if (state.status == AP_Airspeed::Airspeed_Good) {
            temperature = state.temperature;
            return true;
        }
        return false;
    }

private:
    int _fd;
    uint64_t _last_timestamp;

    // we need to keep track of how many PX4 drivers have been loaded
    // so we can open the right device filename
    static uint8_t num_px4_instances;

    // try to open the PX4 driver and return its fd
    static int open_driver(void);
};
