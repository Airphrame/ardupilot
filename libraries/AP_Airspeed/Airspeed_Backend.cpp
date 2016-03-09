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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Airspeed.h"
#include "Airspeed_Backend.h"

/*
  base class constructor. 
  This incorporates initialization as well.
*/
AP_Airspeed_Backend::AP_Airspeed_Backend(Airspeed &_airspeed, uint8_t instance, Airspeed::Airspeed_State &_state) :
        airspeed(_airspeed),
        state(_state) 
{
}

// update status
void AP_Airspeed_Backend::update_status()
{
    // check distance
//    if ((int16_t)state.distance_cm > ranger._max_distance_cm[state.instance]) {
//        set_status(RangeFinder::RangeFinder_OutOfRangeHigh);
//    } else if ((int16_t)state.distance_cm < ranger._min_distance_cm[state.instance]) {
//        set_status(RangeFinder::RangeFinder_OutOfRangeLow);
//    } else {
//        set_status(RangeFinder::RangeFinder_Good);
//    }
}

// set status and update valid count
void AP_Airspeed_Backend::set_status(Airspeed::Airspeed_Status status)
{
    state.status = status;

//    // update valid count
//    if (status == RangeFinder::RangeFinder_Good) {
//        if (state.range_valid_count < 10) {
//            state.range_valid_count++;
//        }
//    } else {
//        state.range_valid_count = 0;
//    }
}
