/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#ifndef __AIRSPEED_H__
#define __AIRSPEED_H__
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include "AP_Airspeed_Backend.h"
#include "AP_Airspeed_I2C.h"
#include "AP_Airspeed_PX4.h"
#include "AP_Airspeed_analog.h"

// Maximum number of Airspeed instances available on this platform
#define AIRSPEED_MAX_INSTANCES 2

class AP_Airspeed_Backend;
 
class Airspeed_Calibration {
public:
    friend class AP_Airspeed;
    // constructor
    Airspeed_Calibration(const AP_Vehicle::FixedWing &parms);

    // initialise the calibration
    void init(float initial_ratio);

    // take current airspeed in m/s and ground speed vector and return
    // new scaling factor
    float update(float airspeed, const Vector3f &vg);

private:
    // state of kalman filter for airspeed ratio estimation
    Matrix3f P; // covarience matrix
    const float Q0; // process noise matrix top left and middle element
    const float Q1; // process noise matrix bottom right element
    Vector3f state; // state vector
    const float DT; // time delta
    const AP_Vehicle::FixedWing &aparm;
};

class AP_Airspeed
{
public:
    // constructor
    AP_Airspeed(const AP_Vehicle::FixedWing &parms);

    enum pitot_tube_order {
        PITOT_TUBE_ORDER_POSITIVE = 0,
        PITOT_TUBE_ORDER_NEGATIVE = 1,
        PITOT_TUBE_ORDER_AUTO     = 2
    };

    enum Airspeed_Status {
        Airspeed_NotConnected = 0,
        Airspeed_NoData,
        Airspeed_Good
    };

    // RangeFinder driver types
    enum Airspeed_Type {
        Airspeed_TYPE_NONE   = 0,
        Airspeed_TYPE_ANALOG = 1,
        Airspeed_TYPE_I2C    = 2,
        Airspeed_TYPE_PX4    = 3,
    };

    // The Airspeed_State structure is filled in by the backend driver
    struct Airspeed_State {
        uint8_t                 instance;    // the instance number of this RangeFinder
        float                   raw_airspeed;
        float                   airspeed;
        float                   pressure;
        float                   last_pressure;
        float                   raw_pressure;
        float                   EAS2TAS;
        enum Airspeed_Status    status;     // sensor status
        bool                    healthy:1;
        bool                    hil_set:1;
        uint32_t                last_update_ms;
        float                   hil_pressure;
        float                   last_saved_ratio;
        uint8_t                 counter;
    };

    AP_Float        _offset[AIRSPEED_MAX_INSTANCES];
    AP_Float        _ratio[AIRSPEED_MAX_INSTANCES];
    AP_Int8         _use[AIRSPEED_MAX_INSTANCES];
    AP_Int8         _enable[AIRSPEED_MAX_INSTANCES];
    AP_Int8         _pin[AIRSPEED_MAX_INSTANCES];
    AP_Int8         _autocal[AIRSPEED_MAX_INSTANCES];
    AP_Int8         _tube_order[AIRSPEED_MAX_INSTANCES];
    AP_Int8         _skip_cal[AIRSPEED_MAX_INSTANCES];
    AP_Int8         _type[AIRSPEED_MAX_INSTANCES];


    // Return the number of airspeed sensor instances
    uint8_t num_sensors(void) const {
        return num_instances;
    }
    
    void init(void);

    // read the analog source and update _airspeed
    void read(void);

    // calibrate the airspeed. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    void calibrate(bool in_startup);

      // return the current airspeed in m/s
      float get_airspeed(uint8_t instance) const {
          return state[instance].airspeed;
      }
      float get_airspeed(void) const {
          return get_airspeed(primary_instance);
      }

      // return the unfiltered airspeed in m/s
      float get_raw_airspeed(uint8_t instance) const {
          return state[instance].raw_airspeed;
      }
      float get_raw_airspeed(void) const {
          return get_raw_airspeed(primary_instance);
      }

      // return the current airspeed in cm/s
      float get_airspeed_cm(uint8_t instance) const {
          return get_airspeed(instance)*100;
      }
      float get_airspeed_cm(void) const {
          return get_airspeed(primary_instance)*100;
      }

      // return the current airspeed ratio (dimensionless)
      float get_airspeed_ratio(uint8_t instance) const {
          return _ratio[instance];
      }
      float get_airspeed_ratio(void) const {
          return get_airspeed_ratio(primary_instance);
      }

      // get temperature if available
      bool get_temperature(uint8_t instance, float &temperature);
      bool get_temperature(float &temperature) {
          return get_temperature(primary_instance, temperature);
      }

      // set the airspeed ratio (dimensionless)
      void set_airspeed_ratio(uint8_t instance, float ratio) {
          _ratio[instance].set(ratio);
      }
      void set_airspeed_ratio(float ratio) {
          set_airspeed_ratio(primary_instance,ratio);
      }

      // return true if airspeed is enabled, and airspeed use is set
      bool use(uint8_t instance) const {
          return _enable[instance] && _use[instance];
      }
      bool use(void) const {
          return use(primary_instance);
      }

      // return true if airspeed is enabled
      bool enabled(uint8_t instance) const {
          return _enable[instance];
      }
      bool enabled(void) const {
          return enabled(primary_instance);
      }

      // force disable the airspeed sensor
      void disable(uint8_t instance) {
          _enable[instance].set(0);
      }
      void disable(void) {
          disable(primary_instance);
      }

      // return the differential pressure in Pascal for the last
      // airspeed reading. Used by the calibration code
      float get_differential_pressure(uint8_t instance) const {
          return  state[instance].last_pressure;
      }
      float get_differential_pressure(void) const {
          return get_differential_pressure(primary_instance);
      }

      // return the current offset
      float get_offset(uint8_t instance) const {
          return _offset[instance];
      }
      float get_offset(void) const {
          return get_offset(primary_instance);
      }

      // return the current raw pressure
      float get_raw_pressure(uint8_t instance) const {
          return state[instance].raw_pressure;
      }
      float get_raw_pressure(void) const {
          return get_raw_pressure(primary_instance);
      }

      // set the apparent to true airspeed ratio
      void set_EAS2TAS(uint8_t instance, float v) {
          state[instance].EAS2TAS = v;
      }
      void set_EAS2TAS(float v) {
          set_EAS2TAS(primary_instance, v);
      }

      // get the apparent to true airspeed ratio
      float get_EAS2TAS(uint8_t instance) const {
          return (instance<num_instances? state[instance].EAS2TAS : 0);

      }
      // get the apparent to true airspeed ratio
      float get_EAS2TAS(void) const {
          return get_EAS2TAS(primary_instance);
      }

    // query status
    Airspeed_Status status(uint8_t instance) const {
        if (drivers[instance] == NULL || _type[instance] == Airspeed_TYPE_NONE) {
            return Airspeed_NotConnected;
        }
        return state[instance].status;
    }
    Airspeed_Status status(void) const {
        return status(primary_instance);
    }

    // update airspeed ratio calibration
    void update_calibration(const Vector3f &vground);

	// log data to MAVLink
	void log_mavlink_send(mavlink_channel_t chan, const Vector3f &vground);

    // return health status of sensor
    bool healthy(uint8_t instance) const {
        return state[instance].healthy && fabsf(_offset[instance]) > 0;
    }
    bool healthy(void) const {
        return healthy(primary_instance);
    }

    // used by HIL to set the airspeed
    void setHIL(float pressure);
    void setHIL(float airspeed, float diff_pressure, float temperature);

    // return time in ms of last update
    uint32_t last_update_ms(uint8_t instance) const {
        return state[instance].last_update_ms;
    }
    uint32_t last_update_ms(void) const {
        return last_update_ms(primary_instance);
    }

    // return pressure
    uint32_t get_pressure(uint8_t instance);
    uint32_t get_pressure(void) {
        return get_pressure(primary_instance);
    }


    static const struct AP_Param::GroupInfo var_info[];


private:
    Airspeed_Calibration _calibration[AIRSPEED_MAX_INSTANCES];
    Airspeed_State state[AIRSPEED_MAX_INSTANCES];
    AP_Airspeed_Backend *drivers[AIRSPEED_MAX_INSTANCES];
    uint8_t primary_instance:3;
    uint8_t num_instances:3;

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);  

    AP_Airspeed_Analog analog[AIRSPEED_MAX_INSTANCES];
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    AP_Airspeed_PX4    digital[AIRSPEED_MAX_INSTANCES];
#else
    AP_Airspeed_I2C    digital[AIRSPEED_MAX_INSTANCES];
#endif
};
#endif // __AIRSPEED_H__
