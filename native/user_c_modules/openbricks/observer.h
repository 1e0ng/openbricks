// SPDX-License-Identifier: MIT
//
// MicroPython object wrapping ``ob_observer_t``. The algorithm lives
// in ``observer_core.{c,h}`` so the firmware build and the host-side
// ``openbricks_sim._native`` extension compile the same math.

#pragma once

#include "py/obj.h"

#include "observer_core.h"

typedef struct _observer_obj_t {
    mp_obj_base_t  base;
    ob_observer_t  core;
} observer_obj_t;

// Backward-compat wrappers around the core API. ``servo.c`` embeds an
// ``observer_obj_t`` directly and was written against ``mp_float_t``
// arguments; these forward to the ``ob_observer_*`` functions on
// ``.core`` so we don't have to retrofit the access pattern.
void openbricks_observer_init(observer_obj_t *o, mp_float_t alpha, mp_float_t beta);
void openbricks_observer_reset(observer_obj_t *o, mp_float_t pos);
void openbricks_observer_update(observer_obj_t *o, mp_float_t measured_pos, mp_float_t dt);
