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
#define OB_DRIVEBASE_DEFAULT_ACCEL_DPS2  1500.0

// Arrival tolerances for ``done`` (wheel degrees). ``done`` used to
// be trajectory-TIME-based: the profile expiring declared the move
// complete while the robot was still settling — bench-measured as a
// repeatable +4.5 body-deg of uncorrected overshoot on every gyro'd
// turn's END (mid-square turns were silently corrected by the next
// move's absolute-frame feedback; the LAST one never was). The
// classic fallback always required measured arrival; this matches
// it. 3 wheel-deg ~= 0.8 mm of travel / ~2 body-deg of heading on
// the 88/136 bench geometry.
#define OB_DRIVEBASE_DONE_TOL_WHEEL_DEG  3.0

// Post-profile no-progress wait cap (ms), the progress epsilon that
// re-stamps it (wheel-deg), and the largest residual the cap may
// forgive (wheel-deg, ~7.8 body-deg on the 88/136 bench). The cap
// exists for the stiction regime — a few degrees the feedback
// cannot break loose in duty mode; a robot still CONVERGING keeps
// re-stamping the window and retains full arrival accuracy. A
// residual LARGER than the forgive limit means the move genuinely
// did not happen (blocked robot, dead heading source): done stays
// false and the move watchdog raises loudly instead of the program
// continuing from a silently wrong pose. See ``settling``.
#define OB_DRIVEBASE_SETTLE_MS                 400
#define OB_DRIVEBASE_SETTLE_PROGRESS_WHEEL_DEG 1.0
#define OB_DRIVEBASE_SETTLE_FORGIVE_WHEEL_DEG  12.0

// ---- 2.6.0 PID + landing settle -------------------------------------
// Position integral action, pbio integrator.c rules: the integral
// GROWS only in a band near the target (outside the deadzone so
// stiction can't hunt at rest, inside 2x the error that already
// saturates proportional control so far-away errors can't wind it
// up), per-tick growth is rate-capped, the total clamps at the
// value that alone commands ACTUATION_MAX, and DECREASING is always
// allowed. ki is dps per (wheel-deg * second).
#define OB_DRIVEBASE_DEFAULT_KI                8.0
#define OB_DRIVEBASE_INTEG_DEADZONE_WHEEL_DEG  1.0
#define OB_DRIVEBASE_INTEG_RATE_MAX_WHEEL_DEG  30.0
#define OB_DRIVEBASE_ACTUATION_MAX_DPS         600.0
// Landing settle: when a profile expires with residual above the
// arrival tolerance, the correction is a SHAPED mini-trajectory
// (measured position -> target at the settings accel, cruise capped
// below) instead of a raw P step — the same accel contract as every
// other motion, so no move boundary ever steps the wheel command.
// Bounded retries; after the last landing the classic forgive/cap
// machinery is the backstop for a genuinely stuck robot.
#define OB_DRIVEBASE_LANDING_DPS               120.0
#define OB_DRIVEBASE_MAX_LANDINGS              3
// Arrival additionally requires the reference to be SLOW — pbio's
// standstill condition for zero-end-speed maneuvers. Without it, a
// plant that crosses the position window mid-landing (still
// carrying the landing ramp's speed) latched done at speed, and
// the automatic then="coast" dispatch released the wheels while
// they were turning: stop, lurch, abrupt stop (bench 2026-08-17,
// the twitch that survived 2.6.0).
#define OB_DRIVEBASE_ARRIVAL_SPEED_TOL_DPS     60.0


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

    // Trajectory acceleration, wheel-degrees/s². Shared by both the
    // forward and turn profiles (they always were — this field just
    // makes the old hardcoded constant settable). Bindings expose it
    // as ``set_accel``; they validate > 0 before writing.
    ob_float_t accel_dps2;

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

    // Bounded settle: after BOTH profiles expire, arrival (errors <
    // tolerance) latches done immediately; the settle cap fires only
    // after OB_DRIVEBASE_SETTLE_MS with NO PROGRESS — the window
    // re-stamps whenever the worst-axis error improves by at least
    // the progress epsilon, so a robot still converging keeps its
    // full accuracy and only a STALLED residual (duty-mode stiction
    // on the last degrees of a turn) is forgiven. In gyro mode a
    // forgiven residual stays in the ABSOLUTE frame and the next
    // move pulls it back.
    bool       settling;
    long       settle_start_ms;
    ob_float_t settle_best_err;

    // PID integral state (2.6.0) — wheel-deg * seconds, per axis.
    ob_float_t integ_sum;
    ob_float_t integ_diff;
    ob_float_t ki;
    long       last_tick_ms;      // for the integral dt (0 = none yet)

    // Measured axis speeds (2.7.x) — EMA of the position derivative,
    // wheel-deg/s. pbio's standstill condition tests the PLANT's
    // speed, not the command: a wound integral can command hundreds
    // of dps into a robot that is demonstrably at rest, and a
    // command-based check then deadlocks done (the integral only
    // unwinds on opposing error, and error is ~0 at arrival).
    ob_float_t meas_vel_sum;
    ob_float_t meas_vel_diff;
    ob_float_t prev_sum_pos;
    ob_float_t prev_diff_pos;
    bool       have_prev_pos;

    // Landing settle bookkeeping (2.6.0). ``expiry_residual`` is the
    // worst-axis error captured at the move's FIRST profile expiry —
    // the bench-measurable answer to "how big is the gap the settle
    // actually closes".
    // Motion flight recorder (2.7.2): the tick samples the sum
    // axis every OB_DRIVEBASE_TRACE_STRIDE_MS into a rolling ring —
    // reference position/velocity, measured position, total
    // command, integral term. Five bench firmwares of settle-stat
    // summaries could not distinguish which curve stalls the
    // integral on the REAL plant; the ring shows the whole ending.
    #define OB_DRIVEBASE_TRACE_N          250
    #define OB_DRIVEBASE_TRACE_STRIDE_MS  16
    float      trace[OB_DRIVEBASE_TRACE_N][6];
    int        trace_next;
    int        trace_count;
    long       trace_last_ms;

    uint8_t    landings;
    bool       landing_active;    // a landing profile is in flight
    ob_float_t landing_best_err;  // best worst-axis err seen at a gate
    ob_float_t expiry_residual;
    // Per-axis diagnostics at the same capture point: which axis
    // carries the lag, and how much the integral was supplying (in
    // dps) when the profile expired — the numbers that say whether
    // the integral did its job during the approach.
    ob_float_t expiry_res_sum;
    ob_float_t expiry_res_diff;
    ob_float_t expiry_integ_sum_dps;
    ob_float_t expiry_integ_diff_dps;
    bool       expiry_captured;
} ob_drivebase_t;


void ob_drivebase_init(ob_drivebase_t *db,
                       ob_servo_t *left, ob_servo_t *right,
                       ob_float_t wheel_diameter_mm,
                       ob_float_t axle_track_mm,
                       ob_float_t kp_sum,
                       ob_float_t kp_diff);


// Kick off a forward move. ``distance_mm`` is signed (negative ⇒
// reverse). ``speed_mm_s`` is the cruise speed. Heading is held at
// whatever it was at move-start. ``carry`` (2.5.0, Pybricks
// Stop.NONE / then="continue"): the profile ends AT cruise instead
// of rest, and past its end the reference keeps advancing at that
// speed until the next command supersedes it — chained maneuvers
// without a stop between them.
void ob_drivebase_straight(ob_drivebase_t *db,
                           long now_ms,
                           ob_float_t distance_mm,
                           ob_float_t speed_mm_s,
                           bool carry);


// Kick off a turn-in-place. ``angle_deg`` is body-degrees, signed
// (positive = CCW = left in Pybricks convention). ``rate_dps`` is
// body-degrees per second (cruise rate). Forward progress is held.
void ob_drivebase_turn(ob_drivebase_t *db,
                       long now_ms,
                       ob_float_t angle_deg,
                       ob_float_t rate_dps);


// Kick off an arc — Pybricks ``DriveBase.curve(radius, angle)``
// semantics: drive along a circle of |radius_mm|, changing heading
// by ``angle_deg`` (positive = CW / right, the system-wide 1.24.0
// convention). The travel direction is the SIGN of ``radius_mm``:
// positive drives forward along the arc, negative backward. The
// centre of the robot covers ``|radians(angle)| * radius`` mm at
// ``speed_mm_s``.
//
// Implementation: BOTH trajectories run simultaneously, with the
// turn profile's cruise AND accel scaled by the turn/forward target
// ratio — proportional speed and accel means the two trapezoids
// have identical time shape (equal ramp times, equal totals), so
// the heading is proportional to distance at EVERY instant and the
// path is a true circle through the ramps, not just at endpoints.
//
// Degenerate inputs: radius 0 arms a pure turn (wheel rate =
// speed_mm_s at the rim); angle 0 covers zero distance and
// completes immediately.
// ``carry`` continues the ARC at full speed past the end (both
// axes keep their end speeds) — same contract as straight's.
void ob_drivebase_curve(ob_drivebase_t *db,
                        long now_ms,
                        ob_float_t radius_mm,
                        ob_float_t angle_deg,
                        ob_float_t speed_mm_s,
                        bool carry);


// Cancel any active move. Servo target_dps is left at zero; the
// individual servos still hold their last position via their own
// hold logic.
void ob_drivebase_stop(ob_drivebase_t *db);


// Decelerate to rest from the CURRENT commanded speeds with both
// axes closed-loop the whole way down — the controlled half of a
// brake/hold stop. Each axis lands wherever v0²/2a puts it (a pure
// decel at the accel limit — ``accel_dps2`` for the forward axis,
// ``turn_accel_dps2`` for the diff axis) and its expiry lock anchors
// the hold THERE. In gyro mode the diff axis rides the IMU exactly as
// it does during a straight, so a wheel that grips harder than the
// other cannot yaw the chassis while braking. An axis already at
// rest arms nothing and keeps its hold — in gyro mode ``turn_hold``
// is the absolute target, and re-capturing measured heading at a
// finished move's end would bank the arrival residual (the +7.6 deg
// per-square regression). Returns false when neither axis needed a
// ramp: the caller applies the end-state at once.
bool ob_drivebase_stop_decel(ob_drivebase_t *db,
                             long now_ms,
                             ob_float_t turn_accel_dps2);


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

// Copy out the flight-recorder ring, oldest first. Returns the
// number of rows written to ``out`` (each row: t_ms, ref_pos,
// meas_pos, ref_vel, cmd_sum, integ_dps).
int ob_drivebase_trace_dump(const ob_drivebase_t *db,
                            float out[][6], int max_rows);


// Settle diagnostics: residual at the last move's first profile
// expiry (wheel-deg, worst axis) and shaped-landing count.
void ob_drivebase_settle_stats(const ob_drivebase_t *db,
                               ob_float_t *res_sum,
                               ob_float_t *res_diff,
                               int *landings,
                               ob_float_t *integ_sum_dps,
                               ob_float_t *integ_diff_dps);


// Reset the gyro-mode absolute frame (turn_hold + override slot).
// Bindings call this on the use_gyro ENABLE transition so the
// current pose becomes both measured-zero and the initial target.
void ob_drivebase_gyro_frame_reset(ob_drivebase_t *db);
