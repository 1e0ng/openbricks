// SPDX-License-Identifier: MIT
//
// openbricks — pcnt_encoder.c
//
// Native quadrature encoder backed by ESP32's hardware Pulse Counter
// (``esp32.PCNT``). The software ``openbricks.drivers.encoder`` driver
// runs its increment logic in Python via ``Pin.irq()`` — good to about
// 5–10 kHz edge rate before the handler dispatch drops counts. High-PPR
// encoders (the MG370 GMR at rated speed emits ~295 kHz) need hardware
// counting.
//
// This C type sits alongside the other openbricks native hot-path
// modules — observer, trajectory, servo, drivebase — so the per-tick
// encoder read that the Servo does at 1 kHz is a single ``mp_call``
// into C, not a hop through a Python property.
//
// We still compose with ``esp32.PCNT`` as the hardware frontend; the
// type holds one channel's PCNT object, caches its ``value`` bound
// method, and uses both channels of the same unit (4x decoding) by
// constructing a second PCNT object on ``channel=1`` during __init__.
//
// Wrap handling — threshold IRQs (since 1.4.0). PCNT's hardware counter
// is signed 16-bit (±32767). At the MG370 GMR's edge rate (68 028
// edges/wheel-rev × ~few rev/sec) the counter wraps 4-10 times per
// second. Earlier versions used a "delta > half-range = wrap" heuristic
// that only worked if ``count()`` was polled faster than ~RANGE/(2 ×
// edge_rate) — broke for open-loop ``run()`` with multi-second sleeps,
// where edges piled up unobserved and the heuristic's wrap correction
// missed multiple wraps in a row.
//
// Now we configure ``threshold0 = -16384`` and ``threshold1 = +16384``
// (quarter-range margin from the hardware limits) and register an IRQ
// that runs ``raw → accum, value(0)`` whenever a threshold is crossed.
// IRQ rate is bounded — at most ~1 IRQ per 16384 edges, so even at
// 200 k edges/sec that's 12 IRQ/sec, comfortable for the scheduler.
// Wraps simply can't happen: by the time the counter reaches +16384 (or
// -16384), the IRQ has already drained it back to zero.
//
// Works on the whole ESP32 family with PCNT — original ESP32 (8 units),
// ESP32-S2, ESP32-S3 (4 units). The ``esp32`` module is imported at
// construction time, so on non-Espressif platforms the driver simply
// fails to instantiate — motor drivers should pick the software
// encoder instead.

#include <stdbool.h>
#include <stdint.h>

#include "py/obj.h"
#include "py/runtime.h"
#include "shared/runtime/mpirq.h"


// PCNT hardware counter is 16-bit signed (±32767 usable). We keep the
// hard limits at ±32767 but install threshold IRQs at ±16384 — the IRQ
// drains the counter back to 0 long before it could approach the full
// limits, so the legacy wrap heuristic in pcnt_update_count is now
// just belt-and-suspenders.
#define PCNT_MIN              (-32767)
#define PCNT_MAX              (+32767)
#define PCNT_THRESHOLD_LOW    (-16384)
#define PCNT_THRESHOLD_HIGH   (+16384)
#define PCNT_RANGE            (PCNT_MAX - PCNT_MIN)   // = 65534

// Generous upper bound — ESP32 classic has 8, S2/S3 have 4. Encoders
// register themselves here so the single-handler IRQ dispatcher can
// look up which encoder a given IRQ belongs to via the parent PCNT
// pointer comparison below.
#define MAX_PCNT_UNITS 8

extern const mp_obj_type_t openbricks_pcnt_encoder_type;

typedef struct _pcnt_encoder_obj_t {
    mp_obj_base_t base;
    mp_obj_t      pcnt;         // esp32.PCNT object for channel 0 (shared unit)
    mp_obj_t      value_fn;     // cached bound method: pcnt.value
    int64_t       accum;        // signed cumulative total since construction
                                // / reset. 64-bit so the accumulator itself
                                // doesn't overflow within any realistic
                                // operating life. At MG370 GMR full speed
                                // (~100 k edges/sec), int32 overflows in 6
                                // hours; int64 takes ~2.9 million years.
    mp_int_t      last_raw;     // previous raw read, for wrap-aware delta
} pcnt_encoder_obj_t;

// Indexed by PCNT unit (0..MAX_PCNT_UNITS-1). Set by the constructor,
// read by the IRQ dispatcher to find which encoder this IRQ is for.
static pcnt_encoder_obj_t *_encoders_by_unit[MAX_PCNT_UNITS];


// --- helpers ----------------------------------------------------------

static mp_int_t pcnt_read_raw(pcnt_encoder_obj_t *self) {
    return mp_obj_get_int(mp_call_function_0(self->value_fn));
}

static void pcnt_write_raw_zero(pcnt_encoder_obj_t *self) {
    mp_call_function_1(self->value_fn, MP_OBJ_NEW_SMALL_INT(0));
}

// Drain whatever's currently in the hardware counter into accum, then
// reset the counter to 0. Called from the threshold IRQ; also called
// from count() to fold in any sub-threshold edges since the last IRQ.
static void pcnt_drain_to_accum(pcnt_encoder_obj_t *self) {
    mp_int_t raw = pcnt_read_raw(self);
    self->accum += (int64_t)raw;
    pcnt_write_raw_zero(self);
    self->last_raw = 0;
}

// Fold any hardware delta since the last read into ``accum``. With the
// threshold IRQ in place, the raw value is bounded to roughly
// ±PCNT_THRESHOLD_HIGH between calls, so wraps physically can't happen
// here — the legacy wrap correction is kept as a defensive no-op for
// platforms / tests where the IRQ doesn't fire.
static int64_t pcnt_update_count(pcnt_encoder_obj_t *self) {
    mp_int_t raw = pcnt_read_raw(self);
    mp_int_t delta = raw - self->last_raw;
    if (delta > PCNT_RANGE / 2) {
        delta -= PCNT_RANGE;
    } else if (delta < -(PCNT_RANGE / 2)) {
        delta += PCNT_RANGE;
    }
    self->accum += (int64_t)delta;
    self->last_raw = raw;
    return self->accum;
}

static void pcnt_reset_count(pcnt_encoder_obj_t *self, int64_t value) {
    self->accum = value;
    pcnt_write_raw_zero(self);
    self->last_raw = 0;
}


// --- IRQ dispatcher ---------------------------------------------------

// Single C handler registered with each PCNT unit's ``irq()`` slot.
// Finds the encoder via parent-PCNT pointer match, drains the
// hardware counter back to 0. ``mp_irq`` runs this in scheduled
// context (not a hard ISR), so calling back into Python — which
// pcnt.value() does internally — is safe.
//
// CRITICAL: clear ``irq.flags()`` BEFORE calling ``pcnt.value()`` to
// drain the counter. Looking at upstream ``esp32_pcnt_value()``:
//
//   while (true) {
//       pcnt_get_counter_value(...);
//       if (self->irq && self->irq->flags && handler != none) {
//           // The handler must call irq.flags() to clear flags,
//           // otherwise this will be an infinite loop.
//           mp_call_function_1(handler, parent);
//           continue;
//       }
//       break;
//   }
//
// — so each ``pcnt.value()`` call synchronously re-invokes our
// handler until flags are cleared. Without the clear, the first
// ``pcnt_read_raw`` inside our handler causes infinite recursion
// (handler → value() → handler → value() → ...). Clearing flags
// up front is the documented contract for PCNT IRQ handlers.
static mp_obj_t _pcnt_encoder_irq_dispatch(mp_obj_t irq_in) {
    mp_irq_obj_t *irq = MP_OBJ_TO_PTR(irq_in);
    if (irq->methods != NULL && irq->methods->info != NULL) {
        // Atomically read-and-clear ``self->irq->flags`` so the
        // synchronous-flush loop in pcnt.value() exits.
        (void)irq->methods->info(irq_in, MP_IRQ_INFO_FLAGS);
    }
    mp_obj_t parent_pcnt = irq->parent;
    for (int i = 0; i < MAX_PCNT_UNITS; i++) {
        pcnt_encoder_obj_t *enc = _encoders_by_unit[i];
        if (enc != NULL && enc->pcnt == parent_pcnt) {
            pcnt_drain_to_accum(enc);
            return mp_const_none;
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(_pcnt_encoder_irq_dispatch_obj,
                                  _pcnt_encoder_irq_dispatch);


// --- constructor ------------------------------------------------------

// PCNTEncoder(pin_a, pin_b, unit=0, filter=1023)
//
// Channel 0 — edge on A, direction from B.
// Channel 1 — edge on B, direction from A (polarity flipped so both
//             channels contribute to one running total, giving 4x
//             decoding).
static mp_obj_t pcnt_encoder_make_new(const mp_obj_type_t *type,
                                      size_t n_args, size_t n_kw,
                                      const mp_obj_t *all_args) {
    enum { ARG_pin_a, ARG_pin_b, ARG_unit, ARG_filter };
    static const mp_arg_t allowed[] = {
        { MP_QSTR_pin_a,  MP_ARG_INT | MP_ARG_REQUIRED, {.u_int = 0} },
        { MP_QSTR_pin_b,  MP_ARG_INT | MP_ARG_REQUIRED, {.u_int = 0} },
        { MP_QSTR_unit,   MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_filter, MP_ARG_INT, {.u_int = 1023} },
    };
    mp_arg_val_t parsed[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args,
                              MP_ARRAY_SIZE(allowed), allowed, parsed);

    mp_int_t pin_a_num = parsed[ARG_pin_a].u_int;
    mp_int_t pin_b_num = parsed[ARG_pin_b].u_int;
    mp_int_t unit      = parsed[ARG_unit].u_int;
    mp_int_t filter    = parsed[ARG_filter].u_int;

    if (unit < 0 || unit >= MAX_PCNT_UNITS) {
        mp_raise_ValueError(MP_ERROR_TEXT("PCNT unit out of range"));
    }

    // Grab Pin and PCNT classes from their modules.
    mp_obj_t esp32_mod   = mp_import_name(MP_QSTR_esp32,   mp_const_none, MP_OBJ_NEW_SMALL_INT(0));
    mp_obj_t machine_mod = mp_import_name(MP_QSTR_machine, mp_const_none, MP_OBJ_NEW_SMALL_INT(0));
    mp_obj_t PCNT_cls    = mp_load_attr(esp32_mod,   MP_QSTR_PCNT);
    mp_obj_t Pin_cls     = mp_load_attr(machine_mod, MP_QSTR_Pin);

    // Pin(pin_num, Pin.IN, Pin.PULL_UP) — both channels.
    mp_obj_t pin_in_mode = mp_load_attr(Pin_cls, MP_QSTR_IN);
    mp_obj_t pin_pull_up = mp_load_attr(Pin_cls, MP_QSTR_PULL_UP);
    mp_obj_t a_pos[3]    = { MP_OBJ_NEW_SMALL_INT(pin_a_num), pin_in_mode, pin_pull_up };
    mp_obj_t b_pos[3]    = { MP_OBJ_NEW_SMALL_INT(pin_b_num), pin_in_mode, pin_pull_up };
    mp_obj_t pin_a       = mp_call_function_n_kw(Pin_cls, 3, 0, a_pos);
    mp_obj_t pin_b       = mp_call_function_n_kw(Pin_cls, 3, 0, b_pos);

    // PCNT classes's INCREMENT / DECREMENT / NORMAL / REVERSE constants.
    mp_obj_t INCREMENT = mp_load_attr(PCNT_cls, MP_QSTR_INCREMENT);
    mp_obj_t DECREMENT = mp_load_attr(PCNT_cls, MP_QSTR_DECREMENT);
    mp_obj_t NORMAL    = mp_load_attr(PCNT_cls, MP_QSTR_NORMAL);
    mp_obj_t REVERSE   = mp_load_attr(PCNT_cls, MP_QSTR_REVERSE);

    // Channel 0 kwargs: channel, pin, rising, falling, mode_pin,
    // mode_low, mode_high, min, max, threshold0, threshold1, filter
    // — 12 kwargs.
    mp_obj_t ch0_args[] = {
        MP_OBJ_NEW_SMALL_INT(unit),                                          // positional: unit
        MP_OBJ_NEW_QSTR(MP_QSTR_channel),    MP_OBJ_NEW_SMALL_INT(0),
        MP_OBJ_NEW_QSTR(MP_QSTR_pin),        pin_a,
        MP_OBJ_NEW_QSTR(MP_QSTR_rising),     INCREMENT,
        MP_OBJ_NEW_QSTR(MP_QSTR_falling),    DECREMENT,
        MP_OBJ_NEW_QSTR(MP_QSTR_mode_pin),   pin_b,
        MP_OBJ_NEW_QSTR(MP_QSTR_mode_low),   NORMAL,
        MP_OBJ_NEW_QSTR(MP_QSTR_mode_high),  REVERSE,
        MP_OBJ_NEW_QSTR(MP_QSTR_min),        MP_OBJ_NEW_SMALL_INT(PCNT_MIN),
        MP_OBJ_NEW_QSTR(MP_QSTR_max),        MP_OBJ_NEW_SMALL_INT(PCNT_MAX),
        MP_OBJ_NEW_QSTR(MP_QSTR_threshold0), MP_OBJ_NEW_SMALL_INT(PCNT_THRESHOLD_LOW),
        MP_OBJ_NEW_QSTR(MP_QSTR_threshold1), MP_OBJ_NEW_SMALL_INT(PCNT_THRESHOLD_HIGH),
        MP_OBJ_NEW_QSTR(MP_QSTR_filter),     MP_OBJ_NEW_SMALL_INT(filter),
    };
    mp_obj_t pcnt_ch0 = mp_call_function_n_kw(PCNT_cls, 1, 12, ch0_args);

    // Channel 1 — direction-flipped modes so both contribute to one running total.
    mp_obj_t ch1_args[] = {
        MP_OBJ_NEW_SMALL_INT(unit),
        MP_OBJ_NEW_QSTR(MP_QSTR_channel),   MP_OBJ_NEW_SMALL_INT(1),
        MP_OBJ_NEW_QSTR(MP_QSTR_pin),       pin_b,
        MP_OBJ_NEW_QSTR(MP_QSTR_rising),    INCREMENT,
        MP_OBJ_NEW_QSTR(MP_QSTR_falling),   DECREMENT,
        MP_OBJ_NEW_QSTR(MP_QSTR_mode_pin),  pin_a,
        MP_OBJ_NEW_QSTR(MP_QSTR_mode_low),  REVERSE,
        MP_OBJ_NEW_QSTR(MP_QSTR_mode_high), NORMAL,
    };
    (void)mp_call_function_n_kw(PCNT_cls, 1, 7, ch1_args);

    // Start counting on the shared unit.
    mp_obj_t start_fn = mp_load_attr(pcnt_ch0, MP_QSTR_start);
    (void)mp_call_function_0(start_fn);

    pcnt_encoder_obj_t *self = mp_obj_malloc(pcnt_encoder_obj_t, type);
    self->pcnt     = pcnt_ch0;
    self->value_fn = mp_load_attr(pcnt_ch0, MP_QSTR_value);
    self->accum    = 0;
    self->last_raw = pcnt_read_raw(self);

    // Register self in the lookup table BEFORE arming the IRQ — avoids
    // a window where an early IRQ could fire and not find the encoder.
    _encoders_by_unit[unit] = self;

    // Arm the threshold IRQ. PCNT.IRQ_THRESHOLD0 | PCNT.IRQ_THRESHOLD1.
    // mpremote / unix MP fakes don't expose the IRQ_THRESHOLD constants
    // — wrap in nlr to keep the constructor working in tests where
    // the fake just no-ops irq().
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t T0 = mp_load_attr(PCNT_cls, MP_QSTR_IRQ_THRESHOLD0);
        mp_obj_t T1 = mp_load_attr(PCNT_cls, MP_QSTR_IRQ_THRESHOLD1);
        mp_int_t trigger = mp_obj_get_int(T0) | mp_obj_get_int(T1);
        mp_obj_t irq_fn  = mp_load_attr(pcnt_ch0, MP_QSTR_irq);
        mp_obj_t irq_args[] = {
            irq_fn,
            MP_OBJ_NULL,
            MP_OBJ_NEW_QSTR(MP_QSTR_handler), MP_OBJ_FROM_PTR(&_pcnt_encoder_irq_dispatch_obj),
            MP_OBJ_NEW_QSTR(MP_QSTR_trigger), MP_OBJ_NEW_SMALL_INT(trigger),
        };
        mp_call_method_n_kw(0, 2, irq_args);
        nlr_pop();
    }
    // else: PCNT IRQ couldn't be armed (fake / older platform). The
    // pcnt_update_count wrap heuristic is still in place and works for
    // low-edge-rate use; high-rate use needs the IRQ on real hardware.

    return MP_OBJ_FROM_PTR(self);
}


// --- methods ----------------------------------------------------------

static mp_obj_t pcnt_encoder_count(mp_obj_t self_in) {
    pcnt_encoder_obj_t *self = MP_OBJ_TO_PTR(self_in);
    // ``mp_obj_new_int_from_ll`` returns a multi-precision int when the
    // value exceeds mp_int_t range, so 32-bit MicroPython ports don't
    // silently truncate the 64-bit accumulator on read.
    return mp_obj_new_int_from_ll((long long)pcnt_update_count(self));
}
static MP_DEFINE_CONST_FUN_OBJ_1(pcnt_encoder_count_obj, pcnt_encoder_count);

static mp_obj_t pcnt_encoder_reset(size_t n_args, const mp_obj_t *args) {
    pcnt_encoder_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    // ``mp_obj_get_ll`` accepts both small and big Python ints; needed
    // for the case where a long-running ``count()`` returned a value
    // beyond int32 and the user feeds it back into ``reset()``.
    int64_t value = (n_args > 1) ? (int64_t)mp_obj_get_ll(args[1]) : 0;
    pcnt_reset_count(self, value);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pcnt_encoder_reset_obj, 1, 2, pcnt_encoder_reset);


// --- type -------------------------------------------------------------

static const mp_rom_map_elem_t pcnt_encoder_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_count), MP_ROM_PTR(&pcnt_encoder_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&pcnt_encoder_reset_obj) },
};
static MP_DEFINE_CONST_DICT(pcnt_encoder_locals_dict, pcnt_encoder_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    openbricks_pcnt_encoder_type,
    MP_QSTR_PCNTEncoder,
    MP_TYPE_FLAG_NONE,
    make_new,    pcnt_encoder_make_new,
    locals_dict, &pcnt_encoder_locals_dict
);
