/*
 * Traction control -- slip ratio model, one PI loop per rear motor.
 *
 * Drive slip of each rear wheel against the front wheel on the same side
 * (fronts are undriven, so they give the ground speed):
 *
 *     slip = (w_rear - w_front - TC_SLIP_SPEED_ALLOW) / w_rear
 *
 * TC_SLIP_SPEED_ALLOW is a fixed rpm margin: at low speed the front sensor
 * reads 0 / lags behind, and a few rpm of difference already looks like a
 * huge ratio, so small absolute differences are ignored.
 *
 * 0 = pure rolling, 1 = rear spinning with the car stopped. While slip is
 * above the target slip the PI loop cuts that motor's torque request:
 *
 *     error     = slip - target_slip
 *     integral += ki * error * dt             (clamped to 0..max_cut)
 *                 ki = cut ki while slipping, TC_KI_RELEASE once back under
 *                 the target, so the cut lets go fast instead of lingering
 *     target    = kp * error + integral       (clamped to 0..max_cut)
 *     cut       moves towards target at most rise_rate / s going up
 *               and TC_CUT_FALL_RATE / s going down
 *     torque    = torque * (1 - cut)
 *
 * Like a race car's TC / TC Cut knobs, two levels set by the on-board ECU
 * on CAN ID 0x6A0 (CAN_ID_TC_CONFIG, see main.c):
 *   - TC (TC1), 0-10: *when* it acts. Picks target_slip; higher level ->
 *     lower slip allowed -> steps in earlier. 0 = TC off.
 *   - TC Cut (TC2), 1-10: *how hard* it acts once slipping. Picks kp, ki,
 *     max_cut and rise_rate; higher level -> bigger, faster cut.
 * Level 5 / 5 is the previously fixed tuning (target 0.20, kp 0.8, ki 3.0,
 * max cut 0.8, rise 10 / s).
 *
 * so it only ever reduces the request, never adds to it, and once slip
 * drops back under the target the integral bleeds the cut away again.
 * The speed filters, the rate limits and max_cut keep the torque from
 * stepping 100 -> 0 (and back) in one cycle, which the driver feels as a
 * hit through the driveline.
 *
 * Wheel speeds come from xTelemetryData: fronts are wheel RPM from 0x502
 * (low-passed here with TC_FRONT_TAU_S), rears are motor_velocity_f /
 * GEAR_RATIO (rear.wheel_speed_rl/rr, already low-passed in main.c) -- both have to be
 * in the same units for the ratio to mean anything. Below
 * TC_MIN_WHEEL_SPEED the ratio isn't meaningful (launch, noise at ~0), so
 * the loop stays out of the way there.
 */
#ifndef TRACTION_H
#define TRACTION_H

#include <stdint.h>

#include "telemetry.h"

#define TC_LEVEL_MAX         10U   /* TC (TC1) 0-10, 0 = off */
#define TC_CUT_LEVEL_MIN     1U    /* TC Cut (TC2) 1-10 */
#define TC_CUT_LEVEL_MAX     10U
#define TC_LEVEL_DEFAULT     5U    /* boot values, until the on-board ECU sends 0x6A0 */
#define TC_CUT_LEVEL_DEFAULT 5U

#define TC_KI_RELEASE        15.0f /* integral bleed per unit of slip error per second, slip under target */
#define TC_FRONT_TAU_S       0.03f /* front wheel speed low-pass time constant, s */
#define TC_SLIP_SPEED_ALLOW  10.0f /* rear may run this many rpm over the front before it counts as slip */
#define TC_CUT_FALL_RATE     3.0f  /* max cut decrease per second (80 % -> 0 in ~270 ms) */
#define TC_MIN_WHEEL_SPEED   30.0f /* rear wheel speed below which TC is inactive */
/* A front reading under TC_FRONT_DEAD_RPM while the other front reads over
 * TC_FRONT_ALIVE_RPM counts as a dead sensor: that side uses the other
 * front as its ground speed. */
#define TC_FRONT_DEAD_RPM    3.0f
#define TC_FRONT_ALIVE_RPM   40.0f

/**
 * @brief Clear both PI integrators. Call whenever TC is off or the motors
 *        aren't being driven, so it starts clean next time.
 */
void traction_reset( void );

/**
 * @brief Apply traction control to the per-motor torque request, tuned by
 *        data->tc_level / data->tc_cut_level. Level 0 (off) only fades out
 *        any cut still applied.
 * @param data Telemetry snapshot (call with xTelemetryMutex held).
 * @param torque Torque request per motor ([0] = left, [1] = right), cut in
 *        place.
 * @param dt_s Time since the previous call, in seconds.
 */
void traction_apply( const TelemetrySnapshot_t *data, int16_t torque[ 2 ], float dt_s );

#endif /* TRACTION_H */
