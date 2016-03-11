/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include "Airspeed_Backend.h"

class AP_Airspeed_Analog : public AP_Airspeed_Backend
{
public:
    // constructor
    AP_Airspeed_Analog(AP_Airspeed &_frontend, uint8_t instance, AP_Airspeed::Airspeed_State &_state);

    // static detection function
    static bool detect(AP_Airspeed &frontend, uint8_t instance);

    // update state
    void update(void);

    bool get_temperature(float &temperature) { return false; }

private:
    AP_HAL::AnalogSource *_source;
};
