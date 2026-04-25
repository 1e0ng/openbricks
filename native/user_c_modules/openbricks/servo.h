// SPDX-License-Identifier: MIT
//
// MicroPython object wrapping ``ob_servo_t``. The state machine +
// control law live in ``servo_core.{c,h}`` so the firmware build and
// the host-side ``openbricks_sim._native`` extension run identical
// math; the binding-shell-specific glue (Pin/PWM/encoder Python
// objects + cached bound methods) stays here.
//
// Sibling C modules (drivebase.c today, hub / kinematics later) reach
// into the embedded ``core`` struct's fields directly for zero-
// overhead reads in the 1 kHz tick. The struct layout is therefore
// part of the C-side ABI.

#pragma once

#include <stdbool.h>

#include "py/obj.h"

#include "servo_core.h"


typedef struct _servo_obj_t {
    mp_obj_base_t base;

    // Hardware handles (Python objects).
    mp_obj_t in1_pin;
    mp_obj_t in2_pin;
    mp_obj_t pwm;
    mp_obj_t encoder;

    // Cached bound methods — saves mp_load_attr per tick.
    mp_obj_t in1_value;
    mp_obj_t in2_value;
    mp_obj_t pwm_duty;
    mp_obj_t encoder_count;   // bound encoder.count — called every tick
    mp_obj_t encoder_reset;   // bound encoder.reset — called from reset_angle

    // The portable state machine — config (counts_per_rev, kp,
    // invert), embedded trajectory + observer, time baselines.
    ob_servo_t core;
} servo_obj_t;
