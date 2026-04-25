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
    ob_float_t t_ramp;       // seconds in accel (and decel) phase
    ob_float_t t_cruise;     // seconds at cruise speed (0 for triangular)
    ob_float_t t_total;      // seconds to completion
    ob_float_t d_ramp;       // degrees covered in one ramp phase
    ob_float_t v_peak;       // peak speed actually reached (magnitude)
    bool       triangular;   // true if the profile never reaches cruise
} ob_trajectory_t;

// Configure ``t`` for a move from ``start`` to ``target`` with the
// given cruise speed and acceleration. Zero distance / zero cruise /
// zero accel yields a degenerate "no motion" profile with
// ``t_total = 0``.
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
