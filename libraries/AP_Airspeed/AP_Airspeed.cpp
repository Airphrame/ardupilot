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
 *   APM_Airspeed.cpp - airspeed (pitot) driver
 */
#include "AP_Airspeed.h"
#include "AP_Airspeed_analog.h"
#include "AP_Airspeed_PX4.h"
#include "AP_Airspeed_I2C.h"

//#include <AP_ADC/AP_ADC.h>
//#include <AP_Common/AP_Common.h>
//#include <AP_HAL/AP_HAL.h>
//#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

// the virtual pin for digital airspeed sensors
#define AP_AIRSPEED_I2C_PIN 65

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
 #define ARSPD_DEFAULT_PIN 1
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4  || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
 #include <sys/stat.h>
 #include <sys/types.h>
 #include <fcntl.h>
 #include <unistd.h>
 #include <systemlib/airspeed.h>
 #include <drivers/drv_airspeed.h>
 #include <uORB/topics/differential_pressure.h>
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V45)
 #define ARSPD_DEFAULT_PIN 0
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V51)
 #define ARSPD_DEFAULT_PIN 0
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V52)
 #define ARSPD_DEFAULT_PIN 0
#elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)
 #define ARSPD_DEFAULT_PIN 0
#elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52)
 #define ARSPD_DEFAULT_PIN 0
#elif defined(CONFIG_ARCH_BOARD_VRHERO_V10)
 #define ARSPD_DEFAULT_PIN 0
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
 #define ARSPD_DEFAULT_PIN 11
#else
 #define ARSPD_DEFAULT_PIN 15
#endif
#elif CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE
 #define ARSPD_DEFAULT_PIN 16
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
 #define ARSPD_DEFAULT_PIN AP_AIRSPEED_I2C_PIN
#else
 #define ARSPD_DEFAULT_PIN 0
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_Airspeed::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Airspeed enable
    // @Description: enable airspeed sensor
    // @Values: 0:Disable,1:Enable
    AP_GROUPINFO("_ENABLE",    0, AP_Airspeed, _enable[0], 1),

    // @Param: USE
    // @DisplayName: Airspeed use
    // @Description: use airspeed for flight control
    // @Values: 1:Use,0:Don't Use
    AP_GROUPINFO("_USE",    1, AP_Airspeed, _use[0], 0),

    // @Param: OFFSET
    // @DisplayName: Airspeed offset
    // @Description: Airspeed calibration offset
    // @Increment: 0.1
    AP_GROUPINFO("_OFFSET", 2, AP_Airspeed, _offset[0], 0),

    // @Param: RATIO
    // @DisplayName: Airspeed ratio
    // @Description: Airspeed calibration ratio
    // @Increment: 0.1
    AP_GROUPINFO("_RATIO",  3, AP_Airspeed, _ratio[0], 1.9936f),

    // @Param: PIN
    // @DisplayName: Airspeed pin
    // @Description: The analog pin number that the airspeed sensor is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated airspeed port on the end of the board. Set to 11 on PX4 for the analog airspeed port. Set to 15 on the Pixhawk for the analog airspeed port. Set to 65 on the PX4 or Pixhawk for an EagleTree or MEAS I2C airspeed sensor.
    // @User: Advanced
    AP_GROUPINFO("_PIN",  4, AP_Airspeed, _pin[0], ARSPD_DEFAULT_PIN),

    // @Param: AUTOCAL
    // @DisplayName: Automatic airspeed ratio calibration
    // @Description: If this is enabled then the APM will automatically adjust the ARSPD_RATIO during flight, based upon an estimation filter using ground speed and true airspeed. The automatic calibration will save the new ratio to EEPROM every 2 minutes if it changes by more than 5%. This option should be enabled for a calibration flight then disabled again when calibration is complete. Leaving it enabled all the time is not recommended.
    // @User: Advanced
    AP_GROUPINFO("_AUTOCAL",  5, AP_Airspeed, _autocal[0], 0),

    // @Param: TUBE_ORDER
    // @DisplayName: Control pitot tube order
    // @Description: This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the top connector on the sensor needs to be the dynamic pressure. If set to 1 then the bottom connector needs to be the dynamic pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft it receiving excessive pressure on the static port, which would otherwise be seen as a positive airspeed.
    // @User: Advanced
    AP_GROUPINFO("_TUBE_ORDER",  6, AP_Airspeed, _tube_order[0], 2),

    // @Param: SKIP_CAL
    // @DisplayName: Skip airspeed calibration on startup
    // @Description: This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("_SKIP_CAL",  7, AP_Airspeed, _skip_cal[0], 0),

    // @Param: _ADDR
    // @DisplayName: I2C Bus address of sensor
    // @Description: This sets the bus address of the sensor, where applicable. A value of 0 disables I2C for this sensor.
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("_ADDR", 8, AP_Airspeed, _address[0], I2C_ADDRESS_MS4525DO),

    // @Param: _TYPE
    // @DisplayName: Airspeed type
    // @Description: What type of airspeed device that is connected
    // @Values: 0:None,1:Analog,2:I2C,3:PX4-I2C
    // @User: Standard
    AP_GROUPINFO("_TYPE", 9, AP_Airspeed, _type[0], Airspeed_TYPE_NONE),

    // 10..15 left for future expansion

#if AIRSPEED_MAX_INSTANCES > 1

    // @Param: 2_ENABLE
    // @DisplayName: Airspeed enable
    // @Description: enable airspeed sensor
    // @Values: 0:Disable,1:Enable
    AP_GROUPINFO("2_ENABLE",    16, AP_Airspeed, _enable[1], 1),

    // @Param: 2_USE
    // @DisplayName: Airspeed use
    // @Description: use airspeed for flight control
    // @Values: 1:Use,0:Don't Use
    AP_GROUPINFO("2_USE",    17, AP_Airspeed, _use[1], 0),

    // @Param: 2_OFFSET
    // @DisplayName: Airspeed offset
    // @Description: Airspeed calibration offset
    // @Increment: 0.1
    AP_GROUPINFO("2_OFFSET", 18, AP_Airspeed, _offset[1], 0),

    // @Param: 2_RATIO
    // @DisplayName: Airspeed ratio
    // @Description: Airspeed calibration ratio
    // @Increment: 0.1
    AP_GROUPINFO("2_RATIO",  19, AP_Airspeed, _ratio[1], 1.9936f),

    // @Param: 2_PIN
    // @DisplayName: Airspeed pin
    // @Description: The analog pin number that the airspeed sensor is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated airspeed port on the end of the board. Set to 11 on PX4 for the analog airspeed port. Set to 15 on the Pixhawk for the analog airspeed port. Set to 65 on the PX4 or Pixhawk for an EagleTree or MEAS I2C airspeed sensor.
    // @User: Advanced
    AP_GROUPINFO("2_PIN",  20, AP_Airspeed, _pin[1], ARSPD_DEFAULT_PIN),

    // @Param: 2_AUTOCAL
    // @DisplayName: Automatic airspeed ratio calibration
    // @Description: If this is enabled then the APM will automatically adjust the ARSPD_RATIO during flight, based upon an estimation filter using ground speed and true airspeed. The automatic calibration will save the new ratio to EEPROM every 2 minutes if it changes by more than 5%. This option should be enabled for a calibration flight then disabled again when calibration is complete. Leaving it enabled all the time is not recommended.
    // @User: Advanced
    AP_GROUPINFO("2_AUTOCAL",  21, AP_Airspeed, _autocal[1], 0),

    // @Param: 2_TUBE_ORDER
    // @DisplayName: Control pitot tube order
    // @Description: This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the top connector on the sensor needs to be the dynamic pressure. If set to 1 then the bottom connector needs to be the dynamic pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft it receiving excessive pressure on the static port, which would otherwise be seen as a positive airspeed.
    // @User: Advanced
    AP_GROUPINFO("2_TUBE_ORDER",  22, AP_Airspeed, _tube_order[1], 2),

    // @Param: 2_SKIP_CAL
    // @DisplayName: Skip airspeed calibration on startup
    // @Description: This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("2_SKIP_CAL",  23, AP_Airspeed, _skip_cal[1], 0),

    // @Param: 2_ADDR
    // @DisplayName: I2C Bus address of sensor
    // @Description: This sets the bus address of the sensor, where applicable. A value of 0 disables I2C for this sensor.
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("2_ADDR", 23, AP_Airspeed, _address[1], I2C_ADDRESS_MS4525DO),

    // @Param: 2_TYPE
    // @DisplayName: Airspeed type
    // @Description: What type of airspeed device that is connected
    // @Values: 0:None,1:Analog,2:I2C,3:PX4-I2C
    // @User: Standard
    AP_GROUPINFO("_TYPE", 24, AP_Airspeed, _type[0], Airspeed_TYPE_NONE),
#endif

    AP_GROUPEND
};


/*
  this scaling factor converts from the old system where we used a
  0 to 4095 raw ADC value for 0-5V to the new system which gets the
  voltage in volts directly from the ADC driver
 */
#define SCALING_OLD_CALIBRATION 819 // 4095/5

AP_Airspeed::AP_Airspeed(const AP_Vehicle::FixedWing &parms)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init state and drivers
    memset(state,0,sizeof(state));
    memset(drivers,0,sizeof(drivers));

    for (uint8_t i=0; i<AIRSPEED_MAX_INSTANCES; i++) {
        state[i].EAS2TAS = 1.0f;
        //_calibration[i] = new Airspeed_Calibration(parms);
    }
};

void AP_Airspeed::init()
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }

    for (uint8_t i=0; i<AIRSPEED_MAX_INSTANCES; i++) {
        detect_instance(i);
        if (drivers[i] != NULL) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }

        state[i].last_pressure = 0;
        //_calibration[i].init(_ratio[i]);
        state[i].last_saved_ratio = _ratio[i];
        state[i].counter = 0;

        // initialise status
        state[i].status = Airspeed_Status::Airspeed_NotConnected;
    }
}


/*
  detect if an instance of a sensor is connected.
 */
void AP_Airspeed::detect_instance(uint8_t instance)
{
    uint8_t type = _type[instance];

    if (type == Airspeed_TYPE_ANALOG) {
        // note that analog must be the last to be checked, as it will
        // always come back as present if the pin is valid
        if (AP_Airspeed_Analog::detect(*this, instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Airspeed_Analog(*this, instance, state[instance]);
            return;
        }
    }
    if (type == Airspeed_TYPE_I2C) {
        if (AP_Airspeed_PX4::detect(*this, instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Airspeed_I2C(*this, instance, state[instance]);
            return;
        }
    }
    if (type == Airspeed_TYPE_PX4) {
        if (AP_Airspeed_PX4::detect(*this, instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Airspeed_PX4(*this, instance, state[instance]);
            return;
        }
    }
}

// read the airspeed sensor
float AP_Airspeed::get_pressure(uint8_t instance)
{
    if (!_enable[instance]) {
        return 0;
    }
    if (state[instance].hil_set) {
        state[instance].healthy = true;
        return state[instance].hil_pressure;
    }
//    float pressure = 0;
//    if (_pin[instance] == AP_AIRSPEED_I2C_PIN) {
//        state[instance].healthy = digital[instance].get_differential_pressure(pressure);
//    } else {
//        state[instance].healthy = analog[instance].get_differential_pressure(pressure);
//    }
    return state[instance].pressure;
}

// get a temperature reading if possible
bool AP_Airspeed::get_temperature(uint8_t instance, float &temperature)
{
    if (!_enable) {
        return false;
    }

    if (drivers[instance]->get_temperature(temperature)) {
        return true;
    }
    return false;
}

// calibrate the airspeed. This must be called at least once before
// the get_airspeed() interface can be used
void AP_Airspeed::calibrate(uint8_t instance, bool in_startup)
{
    float sum = 0;
    uint8_t count = 0;
    if (!_enable[instance]) {
        return;
    }
    if (in_startup && _skip_cal[instance]) {
        return;
    }
    // discard first reading
    get_pressure(instance);
    for (uint8_t i = 0; i < 10; i++) {
        hal.scheduler->delay(100);
        float p = get_pressure(instance);
        if (state[instance].healthy) {
            sum += p;
            count++;
        }
    }
    if (count == 0) {
        // unhealthy sensor
        hal.console->println("Airspeed sensor unhealthy");
        _offset[instance].set(0);
        return;
    }
    float raw = sum/count;
    _offset[instance].set_and_save(raw);
    state[instance].airspeed = 0;
    state[instance].raw_airspeed = 0;
}


/*
  update Airspeed state for all instances.
 */
void AP_Airspeed::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != NULL) {
            if (_type[i] == AP_Airspeed::Airspeed_TYPE_NONE) {
                // allow user to disable a sensor at runtime
                state[i].status = AP_Airspeed::Airspeed_NotConnected;
                continue;
            }
            drivers[i]->update();
            process_raw_data(i);
        }
    }

    // work out primary instance - first sensor returning good data
    for (int8_t i=num_instances-1; i>=0; i--) {
        if (drivers[i] != NULL && (state[i].status == AP_Airspeed::Airspeed_Good)) {
            primary_instance = i;
        }
    }
}

void AP_Airspeed::process_raw_data(uint8_t instance)
{
    float airspeed_pressure;
    if (!_enable[instance]) {
        return;
    }
    airspeed_pressure = get_pressure(instance) - _offset[instance];

    // remember raw pressure for logging
    state[instance].raw_pressure     = airspeed_pressure;

    /*
      we support different pitot tube setups so used can choose if
      they want to be able to detect pressure on the static port
     */
    switch ((enum pitot_tube_order)_tube_order[instance].get()) {
    case PITOT_TUBE_ORDER_NEGATIVE:
        airspeed_pressure = -airspeed_pressure;
        // no break
    case PITOT_TUBE_ORDER_POSITIVE:
        if (airspeed_pressure < -32) {
            // we're reading more than about -8m/s. The user probably has
            // the ports the wrong way around
            state[instance].healthy = false;
        }
        break;
    case PITOT_TUBE_ORDER_AUTO:
    default:
        airspeed_pressure = fabsf(airspeed_pressure);
        break;
    }
    airspeed_pressure               = MAX(airspeed_pressure, 0);
    state[instance].last_pressure   = airspeed_pressure;
    state[instance].raw_airspeed    = sqrtf(airspeed_pressure * _ratio[instance]);
    state[instance].airspeed        = 0.7f *  state[instance].airspeed  +  0.3f * state[instance].raw_airspeed;
    state[instance].last_update_ms  = AP_HAL::millis();
}

void AP_Airspeed::setHIL(float airspeed, float diff_pressure, float temperature)
{
    for (int instance=0; instance<AIRSPEED_MAX_INSTANCES; instance++) {
        state[instance].raw_airspeed = airspeed;
        state[instance].airspeed = airspeed;
        state[instance].last_pressure = diff_pressure;
        state[instance].last_update_ms = AP_HAL::millis();
        state[instance].hil_pressure = diff_pressure;
        state[instance].hil_set = true;
        state[instance].healthy = true;
    }
}

void AP_Airspeed::setHIL(float pressure) {
    for (int instance=0; instance<AIRSPEED_MAX_INSTANCES; instance++) {
        state[instance].healthy = true;
        state[instance].hil_set = true;
        state[instance].hil_pressure = pressure;
    }
}

