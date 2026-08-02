// SPDX-License-Identifier: MIT
//
// st_move_core — per-slot position moves for serial-bus servos.
//
// The step-mode-in-C piece of the motor API: ``run_angle`` and
// ``hold`` for adopted ST-3215/ST-3032 motors, executed on the hard
// tick. Same architecture as drivebase_core, scaled down to one
// axis: a trapezoidal trajectory feeds a position-P + velocity-FF
// law whose output is a wheel-mode speed command — the servo's
// native STEP-mode registers are deliberately NOT used (their
// present-position register reads remaining-to-target, which would
// break the multi-turn odometry the whole bus pump is built on).
//
// Units are encoder counts (4096/rev) and counts/second — the
// sservo core's native domain — so the caller converts degrees
// exactly once, at the Python boundary.
//
// States: IDLE (no output), PROFILE (trajectory in flight), HOLD
// (position lock at the goal — entered automatically when the
// profile expires, or directly via ``hold_at``). ``done`` follows
// the drivebase's ARRIVAL semantics: the profile must have expired
// AND the measured position must be within tolerance — time-based
// done banked the settling error permanently (bench: +4.5 body-deg
// per gyro'd turn end before 1.43.1). Once arrived, ``done``
// latches; the hold keeps correcting if the shaft is disturbed.
//
// Context contract: pure C, no MicroPython, no allocation — callable
// from the esp_timer hard tick and from unix/c-unit tests alike.

#pragma once

#include <stdbool.h>

#include "trajectory_core.h"

// Position-loop gain, (counts/s) of command per count of error.
// Numerically identical ratio to the drivebase's kp_sum=2.0 (dps per
// wheel-degree) — the unit cancels.
#define OB_SMOVE_DEFAULT_KP          2.0

// Arrival tolerance. The drivebase's 3 wheel-degrees, in counts
// (3 * 4096 / 360 ≈ 34).
#define OB_SMOVE_DONE_TOL_COUNTS     34.0

typedef enum {
    OB_SMOVE_IDLE = 0,
    OB_SMOVE_PROFILE,
    OB_SMOVE_HOLD,
} ob_smove_state_t;

typedef struct {
    ob_smove_state_t state;
    bool             done;         // arrival reached (latched)
    ob_trajectory_t  traj;
    long             start_ms;
    ob_float_t       goal_counts;  // profile endpoint / hold target
    ob_float_t       kp;
    ob_float_t       tol_counts;
} ob_smove_t;

void ob_smove_init(ob_smove_t *m);

// Arm a relative move: from the current measured position, travel
// ``delta_counts`` at up to ``speed_cps`` with ``accel_cps2`` ramps.
// Supersedes any in-flight move or hold (new command wins).
void ob_smove_start(ob_smove_t *m, long now_ms,
                    ob_float_t from_counts, ob_float_t delta_counts,
                    ob_float_t speed_cps, ob_float_t accel_cps2);

// Lock position at ``counts`` immediately (PROFILE is superseded).
void ob_smove_hold_at(ob_smove_t *m, ob_float_t counts);

// Back to IDLE — no further output. The caller decides what the
// servo does next (coast, brake, a new command).
void ob_smove_stop(ob_smove_t *m);

bool ob_smove_is_done(const ob_smove_t *m);

// One tick: returns the commanded speed in counts/s for the current
// measured position (0 when IDLE). The caller stages it on the bus.
ob_float_t ob_smove_tick(ob_smove_t *m, long now_ms,
                         ob_float_t meas_counts);
