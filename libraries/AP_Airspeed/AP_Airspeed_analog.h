/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_Analog : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_Analog(AP_Airspeed &_airspeed, uint8_t instance, AP_Airspeed::Airspeed_State &_state);

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure);

    // temperature not available via analog backend
    bool get_temperature(float &temperature) { return false; }

private:
    AP_HAL::AnalogSource *_source;
};
