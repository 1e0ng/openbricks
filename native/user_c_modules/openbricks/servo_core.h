// SPDX-License-Identifier: MIT
//
// servo_core — pure-C servo state machine.
//
// What's here vs what's in the binding shells:
//
// HERE (firmware + sim share these bytes):
//   - Config (counts_per_rev, kp, invert)
//   - Live state (target_dps, active flag, trajectory tracking,
//     observer state, time baseline)
//   - The control law: ``ob_servo_tick(s, count, now_ms)``  →
//     produces a desired power in [-100, 100], applies the
//     trajectory + observer + P-with-feedforward composition.
//   - Trajectory / target setters that update the embedded
//     ``ob_trajectory_t`` and clock baseline.
//   - The angle conversion (count × 360 / counts_per_rev).
//
// IN THE BINDING SHELLS:
//   - Reading the raw encoder count (firmware: bound
//     ``encoder.count()``; sim: read MuJoCo ``jointpos`` sensor).
//   - Writing the bridge outputs given a target power
//     (firmware: H-bridge IN1/IN2 + PWM duty; sim: MuJoCo motor
//     ``ctrl`` value).
//   - Brake / coast bridge driving — firmware has a specific
//     IN1=IN2=1, duty=max pattern for the L298N; the sim just
//     zeros the actuator.
//
// No MicroPython headers, no Python.h. POD struct + scalar math.

#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "trajectory_core.h"
#include "observer_core.h"


// Power output range — firmware historically uses [-100, 100]
// (percent of full scale); the binding scales to its hardware-
// specific PWM resolution. The sim binding maps the same range
// directly onto MuJoCo motor ``ctrl`` (which expects normalised
// values; the chassis MJCF declares the range).
#define OB_SERVO_POWER_CLAMP 100.0


typedef struct {
    // Config (set at construction, never mutated by the tick body).
    int        counts_per_rev;
    ob_float_t kp;
    bool       invert;

    // Live state.
    ob_float_t target_dps;
    bool       active;

    // Per-motor trajectory tracking. Inactive while the parent
    // drivebase (if any) writes ``target_dps`` directly.
    ob_trajectory_t trajectory;
    long            traj_start_ms;
    bool            traj_active;
    bool            traj_done;

    // α-β observer, embedded.
    ob_observer_t observer;

    // Time baseline for observer dt computation. Updated once per tick.
    long last_time_ms;
} ob_servo_t;


// Default control gain. Pulled into a constant so both bindings
// instantiate the same default if no kp override is provided.
#define OB_SERVO_DEFAULT_KP        0.3
#define OB_SERVO_DEFAULT_OBS_ALPHA 0.5
#define OB_SERVO_DEFAULT_OBS_BETA  0.15
#define OB_SERVO_DEFAULT_RATED_DPS 300.0


// Initialise the servo state with defaults. ``counts_per_rev`` and
// ``kp`` come in from the binding (firmware constructor argument /
// sim chassis spec). The observer + trajectory are zeroed.
void ob_servo_init(ob_servo_t *s,
                   int counts_per_rev,
                   ob_float_t kp,
                   bool invert);


// Re-baseline the observer to the angle implied by ``count`` and
// the time baseline to ``now_ms``. Called whenever the servo first
// attaches to the scheduler, or after a ``reset_angle``.
void ob_servo_baseline(ob_servo_t *s, long count, long now_ms);


// Set a constant velocity target. Cancels any active trajectory.
void ob_servo_set_speed(ob_servo_t *s, ob_float_t dps);


// Kick off a trapezoidal trajectory: ``delta_deg`` away from the
// current shaft angle (computed from ``count``). The trajectory
// timeline starts at ``now_ms``.
void ob_servo_run_target(ob_servo_t *s, long count, long now_ms,
                         ob_float_t delta_deg,
                         ob_float_t cruise_dps,
                         ob_float_t accel);


// One control tick. Returns the desired power in
// [-OB_SERVO_POWER_CLAMP, +OB_SERVO_POWER_CLAMP]. Mutates the
// observer + trajectory state but doesn't touch any hardware —
// the binding is responsible for writing ``out`` to its bridge.
ob_float_t ob_servo_tick(ob_servo_t *s, long count, long now_ms);


// Convert a raw encoder count to a shaft angle in degrees.
ob_float_t ob_servo_count_to_angle_deg(const ob_servo_t *s, long count);


// "Done" flag for the embedded trajectory. ``run_speed`` resets it
// to ``true`` (no trajectory active means done by definition).
bool ob_servo_is_done(const ob_servo_t *s);
