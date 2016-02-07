/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

/*
  is_flying and crash detection logic
 */

#define CRASH_DETECTION_DELAY_MS            500
#define IS_FLYING_IMPACT_TIMER_MS           3000

/*
  Do we think we are flying?
  Probabilistic method where a bool is low-passed and considered a probability.
*/
void Plane::update_is_flying_5Hz(void)
{
    float aspeed;
    bool is_flying_bool;
    uint32_t now_ms = AP_HAL::millis();

    uint32_t ground_speed_thresh_cm = (g.min_gndspeed_cm > 0) ? ((uint32_t)(g.min_gndspeed_cm*0.9f)) : 500;
    bool gps_confirmed_movement = (gps.status() >= AP_GPS::GPS_OK_FIX_3D) &&
                                    (gps.ground_speed_cm() >= ground_speed_thresh_cm);

    // airspeed at least 75% of stall speed?
    bool airspeed_movement = ahrs.airspeed_estimate(&aspeed) && (aspeed >= (aparm.airspeed_min*0.75f));

    Vector3f accel_peak_pos = ins.get_accel_peak_hold_pos();
    Vector3f accel_peak_neg = ins.get_accel_peak_hold_neg();

    // detect any worth-while peaks in the accel_peak data and set a flag to log it.
    // z axis has a gravity offset and also peaks much more during a normal flight
    const float interesting_peak_thresh = 0.5f * GRAVITY_MSS; // interesting means anything above 0.5G
    bool accel_peak_data_looks_interesting = false;
    accel_peak_data_looks_interesting |= accel_peak_pos.x > interesting_peak_thresh;
    accel_peak_data_looks_interesting |= accel_peak_pos.y > interesting_peak_thresh;
    accel_peak_data_looks_interesting |= accel_peak_pos.z > (2*interesting_peak_thresh) - GRAVITY_MSS;
    accel_peak_data_looks_interesting |= accel_peak_neg.x < -interesting_peak_thresh;
    accel_peak_data_looks_interesting |= accel_peak_neg.y < -interesting_peak_thresh;
    accel_peak_data_looks_interesting |= accel_peak_neg.z < -(2*interesting_peak_thresh) - GRAVITY_MSS;


    if (quadplane.is_flying()) {
        is_flying_bool = true;

    } else if(arming.is_armed()) {
        // when armed assuming flying and we need overwhelming evidence that we ARE NOT flying
        // short drop-outs of GPS are common during flight due to banking which points the antenna in different directions
        bool gps_lost_recently = (gps.last_fix_time_ms() > 0) && // we have locked to GPS before
                        (gps.status() < AP_GPS::GPS_OK_FIX_2D) && // and it's lost now
                        (now_ms - gps.last_fix_time_ms() < 5000); // but it wasn't that long ago (<5s)

        if ((auto_state.last_flying_ms > 0) && gps_lost_recently) {
            // we've flown before, remove GPS constraints temporarily and only use airspeed
            is_flying_bool = airspeed_movement; // moving through the air
        } else {
            // we've never flown yet, require good GPS movement
            is_flying_bool = airspeed_movement || // moving through the air
                                gps_confirmed_movement; // locked and we're moving
        }

        if (control_mode == AUTO) {
            /*
              make is_flying() more accurate during various auto modes
             */

            // are we auto-landing?
            accel_peak_data_looks_interesting |= auto_state.land_in_progress;

            // Detect X-axis deceleration for probable ground impacts.
            // Limit the max probability so it can decay faster. This
            // will not change the is_flying state, anything above 0.1
            // is "true", it just allows it to decay faster once we decide we
            // aren't flying using the normal schemes
            if (g.crash_accel_threshold == 0) {
                crash_state.impact_detected = false;
            } else if (accel_peak_neg.x < -(g.crash_accel_threshold)) {
                // large deceleration detected, lets lower confidence VERY quickly
                crash_state.impact_detected = true;
                crash_state.impact_timer_ms = now_ms;
                if (isFlyingProbability > 0.2f) {
                    isFlyingProbability = 0.2f;
                }
            } else if (crash_state.impact_detected &&
                (now_ms - crash_state.impact_timer_ms > IS_FLYING_IMPACT_TIMER_MS)) {
                // no impacts seen in a while, clear the flag so we stop clipping isFlyingProbability
                crash_state.impact_detected = false;
            }

            switch (flight_stage)
            {
            case AP_SpdHgtControl::FLIGHT_TAKEOFF:
                break;

            case AP_SpdHgtControl::FLIGHT_NORMAL:
                if (in_preLaunch_flight_stage()) {
                    // while on the ground, an uncalibrated airspeed sensor can drift to 7m/s so
                    // ensure we aren't showing a false positive.
                    is_flying_bool = false;
                    crash_state.is_crashed = false;
                    auto_state.started_flying_in_auto_ms = 0;
                }
                break;

            case AP_SpdHgtControl::FLIGHT_VTOL:
                // TODO: detect ground impacts
                break;

            case AP_SpdHgtControl::FLIGHT_LAND_APPROACH:
                if (fabsf(auto_state.sink_rate) > 0.2f) {
                    is_flying_bool = true;
                }
                break;

            case AP_SpdHgtControl::FLIGHT_LAND_PREFLARE:
            case AP_SpdHgtControl::FLIGHT_LAND_FINAL:
                break;

            case AP_SpdHgtControl::FLIGHT_LAND_ABORT:
                if (auto_state.sink_rate < -0.5f) {
                    // steep climb
                    is_flying_bool = true;
                }
                break;

            default:
                break;
            } // switch
        }
    } else {
        // when disarmed assume not flying and need overwhelming evidence that we ARE flying
        is_flying_bool = airspeed_movement && gps_confirmed_movement;

        if ((control_mode == AUTO) &&
            ((flight_stage == AP_SpdHgtControl::FLIGHT_TAKEOFF) ||
             (flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL)) ) {
            is_flying_bool = false;
        }
    }

    if (!crash_state.impact_detected || !is_flying_bool) {
        // when impact is detected, enforce a clip. Only allow isFlyingProbability to go down, not up.
        // low-pass the result.
        // coef=0.15f @ 5Hz takes 3.0s to go from 100% down to 10% (or 0% up to 90%)
        isFlyingProbability = (0.85f * isFlyingProbability) + (0.15f * (float)is_flying_bool);
    }
    
    /*
      update last_flying_ms so we always know how long we have not
      been flying for. This helps for crash detection and auto-disarm
     */
    bool new_is_flying = is_flying();

    // we are flying, note the time
    if (new_is_flying) {

        auto_state.last_flying_ms = now_ms;

        if (!previous_is_flying) {
            // just started flying in any mode
            started_flying_ms = now_ms;
        }

        if ((control_mode == AUTO) &&
            ((auto_state.started_flying_in_auto_ms == 0) || !previous_is_flying) ) {

            // We just started flying, note that time also
            auto_state.started_flying_in_auto_ms = now_ms;
        }
    }
    previous_is_flying = new_is_flying;

    update_stall_detection();
    crash_detection_update();

    allow_accel_peak_mavlink_streaming = accel_peak_data_looks_interesting;

    if (should_log(MASK_LOG_MODE)) {
        Log_Write_Status();
    }
}

/*
  return true if we think we are flying. This is a probabilistic
  estimate, and needs to be used very carefully. Each use case needs
  to be thought about individually.
 */
bool Plane::is_flying(void)
{
    if (hal.util->get_soft_armed()) {
        // when armed, assume we're flying unless we probably aren't
        return (isFlyingProbability >= 0.1f);
    }

    // when disarmed, assume we're not flying unless we probably are
    return (isFlyingProbability >= 0.9f);
}

/*
 * Determine if we have crashed
 */
void Plane::crash_detection_update(void)
{
    if (control_mode != AUTO || !g.crash_detection_enable)
    {
        // crash detection is only available in AUTO mode
        crash_state.debounce_timer_ms = 0;
        crash_state.is_crashed = false;
        return;
    }

    uint32_t now_ms = AP_HAL::millis();
    bool crashed_near_land_waypoint = false;
    bool crashed = false;
    bool been_auto_flying = (auto_state.started_flying_in_auto_ms > 0) &&
                            (now_ms - auto_state.started_flying_in_auto_ms >= 2500);

    if (!is_flying() && arming.is_armed())
    {
        switch (flight_stage)
        {
        case AP_SpdHgtControl::FLIGHT_TAKEOFF:
            if (g.takeoff_throttle_min_accel > 0 &&
                    !throttle_suppressed) {
                // if you have an acceleration holding back throttle, but you met the
                // accel threshold but still not fying, then you either shook/hit the
                // plane or it was a failed launch.
                crashed = true;
                crash_state.debounce_time_total_ms = CRASH_DETECTION_DELAY_MS;
            }
            // TODO: handle auto missions without NAV_TAKEOFF mission cmd
            break;

        case AP_SpdHgtControl::FLIGHT_NORMAL:
            if (!in_preLaunch_flight_stage() && been_auto_flying) {
                crashed = true;
                crash_state.debounce_time_total_ms = CRASH_DETECTION_DELAY_MS;
            }
            break;

        case AP_SpdHgtControl::FLIGHT_VTOL:
            // we need a totally new method for this
            crashed = false;
            break;
            
        case AP_SpdHgtControl::FLIGHT_LAND_APPROACH:
            if (been_auto_flying) {
                crashed = true;
                crash_state.debounce_time_total_ms = CRASH_DETECTION_DELAY_MS;
            }
            // when altitude gets low, we automatically progress to FLIGHT_LAND_FINAL
            // so ground crashes most likely can not be triggered from here. However,
            // a crash into a tree would be caught here.
            break;

        case AP_SpdHgtControl::FLIGHT_LAND_PREFLARE:
        case AP_SpdHgtControl::FLIGHT_LAND_FINAL:
            // We should be nice and level-ish in this flight stage. If not, we most
            // likely had a crazy landing. Throttle is inhibited already at the flare
            // but go ahead and notify GCS and perform any additional post-crash actions.
            // Declare a crash if we are oriented more that 60deg in pitch or roll
            if (!crash_state.checkedHardLanding && // only check once
                been_auto_flying &&
                (labs(ahrs.roll_sensor) > 6000 || labs(ahrs.pitch_sensor) > 6000)) {
                crashed = true;
                crash_state.debounce_time_total_ms = CRASH_DETECTION_DELAY_MS;

                // did we "crash" within 75m of the landing location? Probably just a hard landing
                crashed_near_land_waypoint =
                        get_distance(current_loc, mission.get_current_nav_cmd().content.location) < 50;

                // trigger hard landing event right away, or never again. This inhibits a false hard landing
                // event when, for example, a minute after a good landing you pick the plane up and
                // this logic is still running and detects the plane is on its side as you carry it.
                crash_state.debounce_timer_ms = now_ms + CRASH_DETECTION_DELAY_MS;
            }

            crash_state.checkedHardLanding = true;
            break;

        default:
            break;
        } // switch
    } else {
        crash_state.checkedHardLanding = false;
    }

    if (!crashed) {
        // reset timer
        crash_state.debounce_timer_ms = 0;

    } else if (crash_state.debounce_timer_ms == 0) {
        // start timer
        crash_state.debounce_timer_ms = now_ms;

    } else if ((now_ms - crash_state.debounce_timer_ms >= crash_state.debounce_time_total_ms) && !crash_state.is_crashed) {
        crash_state.is_crashed = true;

        if (g.crash_detection_enable == CRASH_DETECT_ACTION_BITMASK_DISABLED) {
            if (crashed_near_land_waypoint) {
                gcs_send_text(MAV_SEVERITY_CRITICAL, "Hard landing detected. No action taken");
            } else {
                gcs_send_text(MAV_SEVERITY_EMERGENCY, "Crash detected. No action taken");
            }
        }
        else {
            if (g.crash_detection_enable & CRASH_DETECT_ACTION_BITMASK_DISARM) {
                disarm_motors();
            }
            auto_state.land_complete = true;
            if (crashed_near_land_waypoint) {
                gcs_send_text(MAV_SEVERITY_CRITICAL, "Hard landing detected");
            } else {
                gcs_send_text(MAV_SEVERITY_EMERGENCY, "Crash detected");
            }
        }
    }
}

/*
 * return true if we are in a pre-launch phase of an auto-launch, typically used in bungee launches
 */
bool Plane::in_preLaunch_flight_stage(void) {
    return (control_mode == AUTO &&
            throttle_suppressed &&
            flight_stage == AP_SpdHgtControl::FLIGHT_NORMAL &&
            mission.get_current_nav_cmd().id == MAV_CMD_NAV_TAKEOFF);
}

void Plane::update_stall_detection(void) {

    // only AUTO mode is supported.
    if (control_mode != AUTO || !arming.is_armed()) {
        memset(&stall_state, 0, sizeof(stall_state));
    }

    float aspeed;
    stall_state.roll_error = (nav_roll_cd - ahrs.roll_sensor) * 0.01f;
    stall_state.pitch_error = (nav_pitch_cd - ahrs.pitch_sensor) * 0.01f;
    stall_state.pitch_is_clipping = (nav_pitch_cd >= aparm.pitch_limit_max_cd);
    stall_state.is_below_stall_speed = ahrs.airspeed_estimate(&aspeed) && (aspeed < (aparm.airspeed_min));

    stall_state.pitch_error_integrator1 += stall_state.pitch_error;

    if (stall_state.is_below_stall_speed) {
        stall_state.pitch_error_integrator2 += stall_state.pitch_error;
    } else {
        stall_state.pitch_error_integrator2 = 0;
    }

    if (should_log(MASK_LOG_MODE)) {
        Log_Write_Stall();
    }

    if (aparm.test1 > 0 && is_stalled() && arming.is_armed() &&
        (ins.get_accel_peak_hold_neg().z < -35 || barometer.get_altitude() < 2)) {
        // impact detected while stalled or just above the ground, turn those motors off ASAP!
        gcs_send_text(MAV_SEVERITY_EMERGENCY, "Stall crash, auto-disarmed");
        arming.disarm();
    }

}

bool Plane::is_stalled(void) {
    uint32_t now = millis();
    bool alltitude_is_OK = (fabsf(stall_state.roll_error) < 10) && (fabsf(stall_state.pitch_error) < 10);

    if (auto_state.sink_rate < 11 &&
            (is_flying() && alltitude_is_OK)) {
        // not stalled
        stall_state.debounce_timer_ms = now;
        stall_state.probability = 0;

    } else if (now - stall_state.debounce_timer_ms >= 500) {
        stall_state.probability = 1;

    } else {
        stall_state.probability = 0;
    }

    return stall_state.probability >= 0.95;
}
