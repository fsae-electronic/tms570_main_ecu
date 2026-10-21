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
 * above TC_TARGET_SLIP the PI loop cuts that motor's torque request:
 *
 *     error     = slip - TC_TARGET_SLIP
 *     integral += ki * error * dt             (clamped to 0..TC_MAX_CUT)
 *                 ki = TC_KI while slipping, TC_KI_RELEASE once back under
 *                 the target, so the cut lets go fast instead of lingering
 *     target    = TC_KP * error + integral    (clamped to 0..TC_MAX_CUT)
 *     cut       moves towards target at most TC_CUT_RISE_RATE / s going up
 *               and TC_CUT_FALL_RATE / s going down
 *     torque    = torque * (1 - cut)
 *
 * so it only ever reduces the request, never adds to it, and once slip
 * drops back under the target the integral bleeds the cut away again.
 * The speed filters, the rate limits and TC_MAX_CUT keep the torque from
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

#define TC_TARGET_SLIP       0.20f /* longitudinal slip allowed before cutting, ~20 % */
#define TC_KP                0.8f  /* torque cut per unit of slip error */
#define TC_KI                3.0f  /* integral build-up per unit of slip error per second */
#define TC_KI_RELEASE        15.0f /* integral bleed per unit of slip error per second, slip under target */
#define TC_MAX_CUT           0.8f  /* never cut more than 80 % of the request (50 % saturated with slip 0.6-0.95 at launch) */
#define TC_FRONT_TAU_S       0.03f /* front wheel speed low-pass time constant, s */
#define TC_SLIP_SPEED_ALLOW  10.0f /* rear may run this many rpm over the front before it counts as slip */
#define TC_CUT_RISE_RATE     10.0f /* max cut increase per second (0 -> 80 % in ~80 ms; launch slip reaches ~1 in ~140 ms) */
#define TC_CUT_FALL_RATE     3.0f  /* max cut decrease per second (80 % -> 0 in ~270 ms) */
#define TC_MIN_WHEEL_SPEED   30.0f /* rear wheel speed below which TC is inactive */

/**
 * @brief Clear both PI integrators. Call whenever TC is off or the motors
 *        aren't being driven, so it starts clean next time.
 */
void traction_reset( void );

/**
 * @brief Apply traction control to the per-motor torque request.
 * @param data Telemetry snapshot (call with xTelemetryMutex held).
 * @param torque Torque request per motor ([0] = left, [1] = right), cut in
 *        place.
 * @param dt_s Time since the previous call, in seconds.
 */
void traction_apply( const TelemetrySnapshot_t *data, int16_t torque[ 2 ], float dt_s );

#endif /* TRACTION_H */
