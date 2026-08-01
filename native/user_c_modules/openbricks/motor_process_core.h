// SPDX-License-Identifier: MIT
//
// motor_process_core — C-callback registry + tick-driven clock.
//
// What this is and isn't:
//
// IS:  the data structure (a small static array of (fn, ctx) slots),
//      registration / dedup logic, the firing loop, and the
//      ``virtual_now_ms`` tick-driven monotonic clock. Both the
//      firmware (driven by ``machine.Timer``) and the host sim
//      (driven by MuJoCo's step) call into this shared core so the
//      same servo / drivebase tick functions run unmodified on both.
//
// ISN'T: the Python-callback list, the ``machine.Timer`` lifecycle,
//        or any of the GC root-pointer plumbing. Those are
//        binding-specific glue and live in ``motor_process.c``
//        (firmware) or in the host runtime (sim).
//
// No MicroPython headers, no Python.h. Plain POD.

#pragma once

#include <stddef.h>
#include <stdint.h>


// Slot count is small on purpose — at present we have at most 2
// servo motors + 1 drivebase + future hub instrumentation. Bump the
// bound here if a future controller raises the count.
#define OB_MAX_C_CALLBACKS 8


typedef void (*ob_tick_fn_t)(void *ctx);


typedef struct {
    ob_tick_fn_t fn;
    void        *ctx;
} ob_tick_slot_t;


// Upper bound on how far one wall-clocked fire may advance the
// virtual clock. A starved tick that finally runs after a long
// scheduler gap advances by the REAL elapsed time (that's the point
// of the wall clock: the robot kept moving during the gap, and
// trajectory/PID math must see true time) — but an unbounded jump
// from a pathological source (first-fire glitch, clock corruption)
// must not slam the controllers, so anything beyond this is clamped
// and the excess is dropped.
#define OB_TICK_MAX_CATCHUP_MS 1000


typedef struct {
    ob_tick_slot_t  c_callbacks[OB_MAX_C_CALLBACKS];
    size_t          n_c_callbacks;
    int             period_ms;
    long            virtual_now_ms;
    // Wall-clock tracking for ``fire_c_at``. ``wall_inited`` is 0
    // until the first timestamped fire; that first fire advances by
    // ``period_ms`` (there is no previous timestamp to diff against)
    // and primes ``last_wall_ms``.
    uint32_t        last_wall_ms;
    int             wall_inited;
} ob_motor_process_t;


// Reset to "no callbacks registered, period_ms = 1, clock = 0". Call
// once at program start; subsequent registers / fires keep state.
void ob_motor_process_init(ob_motor_process_t *m);


// Reset state to factory (used by the firmware's test-only ``reset``
// hook and by the sim runner between worlds). Equivalent to
// ``init`` but spelled out so the intent at the call site is clear.
void ob_motor_process_reset(ob_motor_process_t *m);


// Register a (fn, ctx) C-callback pair. Idempotent — a second
// registration with the same pair is a no-op. Returns 0 on success,
// -1 if the slot table is full (caller should raise an error).
int  ob_motor_process_register_c(ob_motor_process_t *m,
                                 ob_tick_fn_t fn, void *ctx);


// Unregister a (fn, ctx) pair. Silent if not registered.
void ob_motor_process_unregister_c(ob_motor_process_t *m,
                                   ob_tick_fn_t fn, void *ctx);


// Fire every C callback once, in registration order. Advances
// ``virtual_now_ms`` by ``period_ms`` BEFORE firing so subscribers
// see a consistent "now" when reading the clock.
//
// TICK-COUNTED variant: assumes every scheduled fire actually ran.
// On firmware that assumption is FALSE — machine.Timer callbacks are
// dispatched through micropython.schedule's bounded queue and dropped
// ticks silently dilate this clock (a bench 981 ms scheduler gap
// advanced it by single-digit ms). The firmware Timer path therefore
// uses ``fire_c_at`` below; this variant remains for deterministic
// callers (unix tests, the sim, whose MuJoCo step IS the clock).
void ob_motor_process_fire_c(ob_motor_process_t *m);


// Fire every C callback once, advancing the clock by the REAL elapsed
// time since the previous timestamped fire. ``wall_now_ms`` is any
// monotonic ms counter that wraps at 2^32 (mp_hal_ticks_ms on
// firmware); wrap-through is handled by unsigned subtraction. The
// first timestamped fire (and the first after ``reset``) has no
// predecessor to diff against and advances by ``period_ms``. A delta
// beyond OB_TICK_MAX_CATCHUP_MS is clamped. Interleaved ``fire_c``
// calls advance the clock but do not disturb wall tracking.
void ob_motor_process_fire_c_at(ob_motor_process_t *m, uint32_t wall_now_ms);


// Configure the tick period (milliseconds). Default 1 ms (1 kHz),
// matching pbio. The clock advances by this much per call to
// ``fire_c``.
void ob_motor_process_set_period_ms(ob_motor_process_t *m, int period_ms);


// Number of C callbacks currently registered. Useful for tests +
// for the firmware's auto-start logic ("only run the timer when
// there's work to do").
size_t ob_motor_process_count_c(const ob_motor_process_t *m);
