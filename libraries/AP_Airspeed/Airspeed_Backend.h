// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#ifndef __AP_AIRSPEED_BACKEND_H__
#define __AP_AIRSPEED_BACKEND_H__

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "Airspeed.h"

class AP_Airspeed_Backend
{
public:
    // constructor. This incorporates initialization as well.
	AP_Airspeed_Backend(AP_Airspeed &_airspeed, uint8_t instance, AP_Airspeed::Airspeed_State &_state);

    // we declare a virtual destructor so that Airspeed drivers can
    // override with a custom destructor if need be
    virtual ~AP_Airspeed_Backend(void) {}

    // update the state structure
    virtual void update() = 0;

protected:

    // update status
    void update_status();

    AP_Airspeed &airspeed;
    AP_Airspeed::Airspeed_State &state;
};
#endif // __AP_AIRSPEED_BACKEND_H__
