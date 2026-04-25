// SPDX-License-Identifier: MIT
//
// observer_core — pure-C two-state α-β position/velocity observer.
//
// Same algorithm as the firmware's ``_openbricks_native.Observer`` and
// the host-side ``openbricks_sim._native.Observer``; both wrappers
// compile this single source so the sim's smoothed-velocity estimate
// is bit-identical to the one running on the ESP32 control tick.
//
// No MicroPython headers, no Python.h, no allocator. Plain POD struct
// + scalar math. Callers embed inline or malloc themselves.

#pragma once

#include "trajectory_core.h"   // for ``ob_float_t``

typedef struct {
    ob_float_t alpha;     // position correction gain  (dimensionless)
    ob_float_t beta;      // velocity correction gain (per second)
    ob_float_t pos_hat;   // estimated position
    ob_float_t vel_hat;   // estimated velocity (same units as pos / second)
} ob_observer_t;

void ob_observer_init(ob_observer_t *o, ob_float_t alpha, ob_float_t beta);
void ob_observer_reset(ob_observer_t *o, ob_float_t pos);
void ob_observer_update(ob_observer_t *o, ob_float_t measured_pos, ob_float_t dt);
