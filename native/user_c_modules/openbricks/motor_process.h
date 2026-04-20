// SPDX-License-Identifier: MIT
//
// Internal C API for subscribing native tick callbacks to the
// openbricks motor_process. Used by sibling C modules (servo.c, and
// later observer / trajectory / hub) to get a ~1 µs per-tick dispatch
// cost instead of the ~25 µs Python-callback path.
//
// The Python API (register/unregister of Python callables) lives in
// motor_process.c and is unchanged; both lists fire every tick, C first.

#pragma once

#include <stddef.h>

#include "py/obj.h"

// Singleton instance defined in motor_process.c. Exported so the module
// registration in openbricks_module.c can reference it.
typedef struct _motor_process_obj_t motor_process_obj_t;
extern motor_process_obj_t motor_process_singleton;

typedef void (*openbricks_tick_fn_t)(void *ctx);

// Subscribe a C callback. Dedup on (fn, ctx). Raises RuntimeError if the
// internal slot table is full.
void openbricks_motor_process_register_c(openbricks_tick_fn_t fn, void *ctx);

// Unsubscribe. Silent if the (fn, ctx) pair isn't registered.
void openbricks_motor_process_unregister_c(openbricks_tick_fn_t fn, void *ctx);

// Monotonic millisecond clock that advances by ``period_ms`` each tick.
// Used by servo / drivebase to measure elapsed trajectory time in a way
// that's deterministic on both firmware (tracking the timer cadence)
// and tests (where the virtual clock drives the timer, so real wall
// time never advances inside a simulated tick).
mp_int_t openbricks_motor_process_now_ms(void);
