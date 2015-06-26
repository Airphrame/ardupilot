// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

void Plane::init_barometer(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));    
    barometer.calibrate();

    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

void Plane::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.init();
#endif
}

/*
  read the rangefinder and update height estimate
 */
void Plane::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

    if (should_log(MASK_LOG_SONAR))
        Log_Write_Sonar();

    rangefinder_height_update();
#endif
}

/*
  ask airspeed sensor for a new value
 */
void Plane::read_airspeed(void)
{
    if (airspeed.enabled()) {
        airspeed.read();
        if (should_log(MASK_LOG_IMU)) {
            Log_Write_Airspeed();
        }
        calc_airspeed_errors();

        // supply a new temperature to the barometer from the digital
        // airspeed sensor if we can
        float temperature;
        if (airspeed.get_temperature(temperature)) {
            barometer.set_external_temperature(temperature);
        }
    }

    // update smoothed airspeed estimate
    float aspeed;
    if (ahrs.airspeed_estimate(&aspeed)) {
        smoothed_airspeed = smoothed_airspeed * 0.8f + aspeed * 0.2f;
    }

    check_for_airspeed_hardware_failure();
}

void Plane::check_for_airspeed_hardware_failure(void)
{
    if (!ahrs.airspeed_sensor_enabled() || (control_mode != AUTO)) {
        return;
    }

    static uint32_t time_since_after_takeoff = 0;
    if (flight_stage == AP_SpdHgtControl::FLIGHT_TAKEOFF) {
        time_since_after_takeoff = hal.scheduler->millis();
    }
    else if (is_flying() && (hal.scheduler->millis() > (time_since_after_takeoff + 5000))) {
        // >5 seconds after takeoff completes

        static uint32_t time_since_last_good_airspeed = hal.scheduler->millis();
        if (airspeed_error_cm < 800) {
            time_since_last_good_airspeed = hal.scheduler->millis();
        }
        // check if guidance is trying to prevent a stall by driving us into the ground
        else if (hal.scheduler->millis() > (time_since_last_good_airspeed + 3000) && // seconds of bad airspeed
            (SpdHgt_Controller->get_pitch_demand() <= -500) && // if trying to dive downward 5deg
            (SpdHgt_Controller->get_throttle_demand() >= aparm.throttle_max.get()) && // if driving the motor to gain speed
            (ahrs.groundspeed() > aparm.throttle_cruise * 1.5f) &&
            (auto_state.sink_rate > 2))  { // if diving down

            // Bad airspeed/pitot tube detected. Check param for behavior, maybe a bitmask?

            // param == 0: do nothing
            // param & 0x1: airspeed.disable();
            // param & 0x2: _use.set_and_save(0) and/or _enable.set_and_save(0)
            // param & 0x4: set_mode(RTL)
            // param & 0x8: ????????
        }
    }
}

void Plane::zero_airspeed(bool in_startup)
{
    airspeed.calibrate(in_startup);
    read_airspeed();
    // update barometric calibration with new airspeed supplied temperature
    barometer.update_calibration();
    gcs_send_text_P(SEVERITY_LOW,PSTR("zero airspeed calibrated"));
}

// read_battery - reads battery voltage and current and invokes failsafe
// should be called at 10hz
void Plane::read_battery(void)
{
    battery.read();
    compass.set_current(battery.current_amps());

    if (!usb_connected && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        low_battery_event();
    }
}


// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void Plane::read_receiver_rssi(void)
{
    // avoid divide by zero
    if (g.rssi_range <= 0) {
        receiver_rssi = 0;
    }else{
        rssi_analog_source->set_pin(g.rssi_pin);
        float ret = rssi_analog_source->voltage_average() * 255 / g.rssi_range;
        receiver_rssi = constrain_int16(ret, 0, 255);
    }
}

