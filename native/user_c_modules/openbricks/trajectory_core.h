// SPDX-License-Identifier: MIT
//
// trajectory_core — pure-C trapezoidal-profile planner.
//
// Why this exists: the same algorithm is needed in two very different
// build environments — the MicroPython firmware (via
// ``native/user_c_modules/openbricks/trajectory.c`` which wraps this
// file in an ``mp_obj_t`` shell) and the CPython-based openbricks-sim
// (via ``tools/openbricks-sim/native/openbricks_sim_native.c`` which
// wraps this file in a ``PyObject*`` shell). Both wrappers compile the
// same ``trajectory_core.c`` so the math is literally identical — no
// drift between firmware and sim.
//
// No MicroPython headers. No Python.h. No allocator. Just scalar math
// with a struct POD. Callers embed the struct inline or malloc it
// themselves.

#pragma once

#include <stdbool.h>

// Floating-point width for the planner. We default to ``double`` so
// the firmware's ``mp_float_t`` (which is ``double`` on the unix MP
// host that runs our tests, and ``float`` on ESP32) doesn't trip
// ``-Wdouble-promotion`` at the wrapper boundary. Explicit single-
// precision via ``-DOB_FLOAT_FLOAT`` is available for ports where
// every extra word of flash and every double-precision cycle matter;
// we don't use it today because the trajectory math is not hot
// enough for the precision drop to be meaningful.
#ifdef OB_FLOAT_FLOAT
typedef float  ob_float_t;
#define ob_sqrt sqrtf
#else
typedef double ob_float_t;
#define ob_sqrt sqrt
#endif

typedef struct {
    ob_float_t start;
    ob_float_t distance;     // signed (target - start)
    ob_float_t cruise;       // magnitude
    ob_float_t accel;        // magnitude
    ob_float_t direction;    // +1 or -1
    // Entry state (2.0.0): profiles begin at the axis's CURRENT
    // speed, relative to the move's direction (negative = moving the
    // wrong way; may exceed cruise). v0 = 0 reproduces the classic
    // from-rest trapezoid exactly.
    ob_float_t v0;           // entry speed, direction-relative
    // End state (2.5.0, Pybricks Stop.NONE): the exit ramp lands at
    // v3 instead of rest. v3 = 0 reproduces the stopping profile
    // exactly; v3 > 0 means the axis CARRIES its speed past t_total
    // (the caller keeps integrating the reference).
    ob_float_t v3;           // end speed, magnitude along direction
    ob_float_t t_entry;      // seconds in the entry ramp (v0 -> v_peak)
    ob_float_t a_entry;      // signed entry acceleration (±accel)
    ob_float_t d_entry;      // net displacement of the entry ramp
    ob_float_t t_ramp;       // seconds in the EXIT ramp (v_peak -> v3)
    ob_float_t t_cruise;     // seconds at cruise speed (0 for triangular)
    ob_float_t t_total;      // seconds to completion
    ob_float_t d_ramp;       // degrees covered in the exit ramp
    ob_float_t v_peak;       // peak speed actually reached (magnitude)
    bool       triangular;   // true if the profile never reaches cruise
} ob_trajectory_t;

// Configure ``t`` for a move from ``start`` to ``target`` with the
// given cruise speed and acceleration, entering at the axis's
// current speed ``v0_world`` (world frame — the init converts to the
// move's direction). Zero distance / zero cruise / zero accel yields
// a degenerate "no motion" profile with ``t_total = 0``. A distance
// too short to stop from ``v0`` yields a pure-decel profile that
// overshoots by the physics-mandated margin; position feedback owns
// the residual after expiry.
void ob_trajectory_init_v0(ob_trajectory_t *t,
                           ob_float_t start,
                           ob_float_t target,
                           ob_float_t cruise,
                           ob_float_t accel,
                           ob_float_t v0_world);

// Full form: additionally end the profile at speed ``v3_end``
// (magnitude, clamped to [0, cruise]) instead of rest — Pybricks
// Stop.NONE. Sampling at/after ``t_total`` reports velocity v3 with
// position at the target; integrating the carried reference beyond
// the target is the caller's job. A distance too short to slow from
// ``v0`` to ``v3`` raises this move's deceleration to land exactly
// on target (the 2.4.0 rule, generalized).
void ob_trajectory_init_v0v3(ob_trajectory_t *t,
                             ob_float_t start,
                             ob_float_t target,
                             ob_float_t cruise,
                             ob_float_t accel,
                             ob_float_t v0_world,
                             ob_float_t v3_end);

// From-rest form: ``ob_trajectory_init_v0`` with ``v0 = 0``.
void ob_trajectory_init(ob_trajectory_t *t,
                        ob_float_t start,
                        ob_float_t target,
                        ob_float_t cruise,
                        ob_float_t accel);

// Sample the profile at absolute time ``t_s``. Clamps below 0 (returns
// start position, zero velocity) and above ``t_total`` (returns end
// position, zero velocity). ``pos_out`` and ``vel_out`` receive the
// signed values.
void ob_trajectory_sample(const ob_trajectory_t *t,
                          ob_float_t t_s,
                          ob_float_t *pos_out,
                          ob_float_t *vel_out);
