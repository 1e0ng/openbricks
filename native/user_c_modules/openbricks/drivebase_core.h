// SPDX-License-Identifier: MIT
//
// drivebase_core — pure-C 2-DOF coupled drivebase controller.
//
// Composes:
//   - two ``ob_servo_t`` (left / right) by raw pointer
//   - two ``ob_trajectory_t`` (forward and turn) embedded inline
//   - two integer gains (kp_sum, kp_diff)
//
// Each tick:
//   1. Sample fwd + turn trajectories at the elapsed time → produce
//      target sum_pos + ff_vel_sum, target turn_pos + ff_vel_turn.
//      A trajectory that's run past its duration sticks at its end-
//      point (and its corresponding ``hold`` field locks the servo
//      at that target so wandering is corrected by feedback).
//   2. Read each servo's observer.pos_hat to compute the actual
//      sum_pos = (L + R) / 2 and diff_pos = (L - R) / 2.
//      For diff_pos, the binding may override with an IMU-derived
//      heading (slip-immune) by setting ``use_gyro = true`` and
//      writing the body-degree heading delta into
//      ``heading_override_body_deg`` before each tick.
//   3. Compute (sum_err, diff_err) and PID-with-FF the per-axis
//      command. Mix into per-servo target_dps:
//
//        left.target_dps  = fwd_cmd + diff_cmd
//        right.target_dps = fwd_cmd - diff_cmd
//
// The individual servos still run their own per-motor velocity
// loops at 1 kHz; the drivebase is a *setpoint source* on top of
// them, not a replacement.
//
// No MicroPython, no Python.h — POD struct + scalar math.

#pragma once

#include <stdbool.h>

#include "trajectory_core.h"
#include "servo_core.h"


// Sensible defaults that match the firmware implementation. Both
// bindings instantiate the same defaults if the caller doesn't
// override.
#define OB_DRIVEBASE_DEFAULT_KP_SUM    2.0
#define OB_DRIVEBASE_DEFAULT_KP_DIFF   5.0
#define OB_DRIVEBASE_DEFAULT_ACCEL_DPS2  720.0


typedef struct {
    // Servo handles — raw pointers into the bindings' ``ob_servo_t``
    // members. Caller is responsible for keeping them alive (the
    // firmware drivebase holds ``mp_obj_t`` strong refs separately;
    // the sim binding holds Py refs).
    ob_servo_t *left;
    ob_servo_t *right;

    // Physical parameters — π × wheel diameter, in millimetres.
    ob_float_t wheel_circumference_mm;
    ob_float_t axle_track_mm;

    // Coupled-controller gains.
    ob_float_t kp_sum;
    ob_float_t kp_diff;

    // Trajectories.
    ob_trajectory_t fwd;
    long            fwd_start_ms;
    bool            fwd_active;
    ob_float_t      fwd_hold;     // captured at move-start, used while inactive

    ob_trajectory_t turn;
    long            turn_start_ms;
    bool            turn_active;
    ob_float_t      turn_hold;

    // Heading source — when ``use_gyro`` is true, the binding writes
    // the wheel-degree-equivalent heading delta (computed from its
    // IMU's body-degree heading) into
    // ``heading_override_wheel_deg`` before each tick. Otherwise
    // the encoder differential is used.
    bool       use_gyro;
    ob_float_t heading_override_wheel_deg;

    bool done;     // last scheduled move finished
} ob_drivebase_t;


void ob_drivebase_init(ob_drivebase_t *db,
                       ob_servo_t *left, ob_servo_t *right,
                       ob_float_t wheel_diameter_mm,
                       ob_float_t axle_track_mm,
                       ob_float_t kp_sum,
                       ob_float_t kp_diff);


// Kick off a forward move. ``distance_mm`` is signed (negative ⇒
// reverse). ``speed_mm_s`` is the cruise speed. Heading is held at
// whatever it was at move-start.
void ob_drivebase_straight(ob_drivebase_t *db,
                           long now_ms,
                           ob_float_t distance_mm,
                           ob_float_t speed_mm_s);


// Kick off a turn-in-place. ``angle_deg`` is body-degrees, signed
// (positive = CCW = left in Pybricks convention). ``rate_dps`` is
// body-degrees per second (cruise rate). Forward progress is held.
void ob_drivebase_turn(ob_drivebase_t *db,
                       long now_ms,
                       ob_float_t angle_deg,
                       ob_float_t rate_dps);


// Cancel any active move. Servo target_dps is left at zero; the
// individual servos still hold their last position via their own
// hold logic.
void ob_drivebase_stop(ob_drivebase_t *db);


// One control tick — see file-top comment for the math. Reads
// ``observer.pos_hat`` from each servo, writes ``target_dps`` on
// each. If ``use_gyro`` is set, expects the binding to have written
// ``heading_override_wheel_deg`` (= -body_heading_delta * axle_track
// * π / wheel_circumference) before calling.
void ob_drivebase_tick(ob_drivebase_t *db, long now_ms);


// True iff no move is active.
bool ob_drivebase_is_done(const ob_drivebase_t *db);


// Convert an IMU-supplied body heading delta (in body degrees) to
// the wheel-degree differential the controller expects in
// ``heading_override_wheel_deg``. Pure utility — bindings can call
// this to do the conversion before tick.
ob_float_t ob_drivebase_body_to_wheel_diff(const ob_drivebase_t *db,
                                            ob_float_t body_heading_delta_deg);
