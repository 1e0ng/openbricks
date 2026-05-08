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
#include "py/mphal.h"
#include "shared/runtime/mpirq.h"

// Diagnostic build (1.4.3): trace every PCNTEncoder hot-path call so a
// hardware hang is observable from the console. Set to 0 once the
// PCNT IRQ pipeline is verified working — leaving it on adds a UART
// write per scheduler tick, which dominates the hot path.
#define OPENBRICKS_PCNT_TRACE 1
#if OPENBRICKS_PCNT_TRACE
#define PCNT_TRACE(fmt, ...) mp_printf(&mp_plat_print, "[pcnt] " fmt "\n", ##__VA_ARGS__)
#else
#define PCNT_TRACE(fmt, ...) ((void)0)
#endif


// PCNT hardware counter is 16-bit signed (±32767). We use the min/max
// auto-reset feature: when the hardware counter hits ±PCNT_LIMIT, the
// peripheral itself resets it to 0 AND fires an IRQ. The IRQ handler
// adds the known limit value (±PCNT_LIMIT) to ``accum`` and returns —
// crucially WITHOUT reading the counter via pcnt.value(), because
// upstream's value() runs a synchronous-flush loop that dispatches
// us, which would re-enter value() if any other event arrived during
// the handler — infinite recursion.
//
// PCNT_LIMIT well below ±32767 leaves headroom for stray edges between
// the hardware reset and our handler running.
#define PCNT_LIMIT  (16384)
#define PCNT_MIN    (-PCNT_LIMIT)
#define PCNT_MAX    (+PCNT_LIMIT)
#define PCNT_RANGE  (PCNT_MAX - PCNT_MIN)   // = 32768 (full counter span between auto-resets)

// Generous upper bound — ESP32 classic has 8, S2/S3 have 4. Encoders
// register themselves here so the single-handler IRQ dispatcher can
// look up which encoder a given IRQ belongs to via the parent PCNT
// pointer comparison below.
#define MAX_PCNT_UNITS 8

// PCNT event-status bit masks, read from ``esp32.PCNT.IRQ_MIN`` /
// ``IRQ_MAX`` at first construction (so we don't have to depend on
// ESP-IDF headers from this user_c_module). Used by the IRQ
// dispatcher to tell which limit was hit from the flags returned by
// ``info(MP_IRQ_INFO_FLAGS)``.
static mp_int_t _pcnt_evt_l_lim = -1;
static mp_int_t _pcnt_evt_h_lim = -1;

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

// Diagnostic counters — incremented from every code path so the
// console trace can show "IRQ never fired" vs "IRQ stuck in a loop"
// vs "count() blocked on pcnt.value()".
static volatile uint32_t _diag_irq_count       = 0;
static volatile uint32_t _diag_count_calls     = 0;
static volatile uint32_t _diag_pcnt_value_pre  = 0;
static volatile uint32_t _diag_pcnt_value_post = 0;


// --- helpers ----------------------------------------------------------

static mp_int_t pcnt_read_raw(pcnt_encoder_obj_t *self) {
    _diag_pcnt_value_pre++;
    PCNT_TRACE("read_raw enter (pre=%u post=%u)",
               (unsigned)_diag_pcnt_value_pre,
               (unsigned)_diag_pcnt_value_post);
    mp_obj_t r = mp_call_function_0(self->value_fn);
    _diag_pcnt_value_post++;
    mp_int_t v = mp_obj_get_int(r);
    PCNT_TRACE("read_raw exit (post=%u) raw=%d",
               (unsigned)_diag_pcnt_value_post, (int)v);
    return v;
}

static void pcnt_write_raw_zero(pcnt_encoder_obj_t *self) {
    PCNT_TRACE("write_raw_zero enter");
    mp_call_function_1(self->value_fn, MP_OBJ_NEW_SMALL_INT(0));
    PCNT_TRACE("write_raw_zero exit");
}

// (Earlier versions had a ``pcnt_drain_to_accum`` helper that read the
// counter and reset it to 0 in the IRQ handler. That recursed through
// ``pcnt.value()``'s flush loop and locked up the chip on the first
// threshold crossing. The 1.4.2 redesign moves all counter access out
// of the IRQ handler — the handler just adds the known limit value to
// the accumulator. The hardware itself resets the counter via min/max
// auto-reset.)

// Fold any hardware delta since the last read into ``accum``. With the
// threshold IRQ in place, the raw value is bounded to roughly
// ±PCNT_THRESHOLD_HIGH between calls, so wraps physically can't happen
// here — the legacy wrap correction is kept as a defensive no-op for
// platforms / tests where the IRQ doesn't fire.
static int64_t pcnt_update_count(pcnt_encoder_obj_t *self) {
    mp_int_t prev_last_raw = self->last_raw;
    mp_int_t raw = pcnt_read_raw(self);
    mp_int_t delta = raw - prev_last_raw;
    int wrap_corr = 0;
    if (delta > PCNT_RANGE / 2) {
        delta -= PCNT_RANGE;
        wrap_corr = -1;
    } else if (delta < -(PCNT_RANGE / 2)) {
        delta += PCNT_RANGE;
        wrap_corr = +1;
    }
    self->accum += (int64_t)delta;
    self->last_raw = raw;
    PCNT_TRACE("update prev=%d raw=%d delta=%d wrap=%d accum_hi=%d accum_lo=%d irqs=%u",
               (int)prev_last_raw, (int)raw, (int)delta, wrap_corr,
               (int)(self->accum >> 32),
               (int)(self->accum & 0xFFFFFFFF),
               (unsigned)_diag_irq_count);
    return self->accum;
}

static void pcnt_reset_count(pcnt_encoder_obj_t *self, int64_t value) {
    self->accum = value;
    pcnt_write_raw_zero(self);
    self->last_raw = 0;
}


// --- IRQ dispatcher ---------------------------------------------------

// Single C handler registered with each PCNT unit's ``irq()`` slot.
// Runs in mp_irq scheduled context — not a hard ISR — so we *can*
// call back into Python; we just must not, because of the
// reentrancy trap below.
//
// REENTRANCY TRAP: ``esp32_pcnt_value()`` runs a while-true flush
// loop that dispatches the IRQ handler whenever ``self->irq->flags``
// is non-zero. Hardware can set those flags asynchronously at any
// time (a real edge crossing the limit). So if our handler reads or
// writes the counter via pcnt.value(), the flush loop in that
// recursive value() call will dispatch us again as soon as the
// hardware fires another IRQ — even if we cleared the previous
// flags. With a fast-spinning encoder, this locks the chip in an
// unbounded handler→value()→handler chain (1.4.0 / 1.4.1 hung the
// chip on the first threshold crossing during a hand-spin).
//
// The 1.4.2 design avoids the trap entirely:
//
//   * Configure PCNT with min = -PCNT_LIMIT and max = +PCNT_LIMIT.
//     The hardware peripheral itself resets the counter to 0 when
//     it hits either limit — no software write needed.
//   * Subscribe to IRQ_MIN | IRQ_MAX. When the IRQ fires, the
//     ``info(MP_IRQ_INFO_FLAGS)`` call returns which limit fired
//     (and atomically clears flags so the flush loop exits).
//   * Add the known limit value (±PCNT_LIMIT) to ``accum``. Done.
//
// No counter read, no counter write inside the handler. Reentrancy
// can't happen because there's no nested pcnt.value() call.
static mp_obj_t _pcnt_encoder_irq_dispatch(mp_obj_t pcnt_in) {
    _diag_irq_count++;
    uint32_t my_n = _diag_irq_count;
    PCNT_TRACE("irq#%u enter", (unsigned)my_n);

    // CRITICAL convention: ``mp_irq_methods_t.info`` is always called
    // with the PARENT object as ``self_in``, not the irq object —
    // see shared/runtime/mpirq.c:125 (mp_irq_flags) and the esp32
    // pcnt info impl which casts to esp32_pcnt_obj_t (line 388).
    //
    // Our handler receives the PCNT object directly (upstream's flush
    // loop and mp_irq_handler both pass ``self->irq->base.parent``).
    // So we get the irq object from the PCNT to access methods/flags
    // bookkeeping, but call ``info()`` with the PCNT.
    //
    // 1.4.0/1.4.1/1.4.2 cast the PCNT-as-handler-arg to mp_irq_obj_t*
    // (read garbage method pointer, info() did nothing).
    // 1.4.4 fixed that but passed the IRQ object to info() — also
    // wrong: esp32_pcnt_irq_info treats self_in as the PCNT, so
    // ``self->irq->flags`` read garbage and returned wild values
    // like 0x63205d74 with multiple bits set.
    mp_obj_t irq_attr = mp_load_attr(pcnt_in, MP_QSTR_irq);
    mp_obj_t irq_obj  = mp_call_function_0(irq_attr);
    mp_irq_obj_t *irq = MP_OBJ_TO_PTR(irq_obj);

    mp_uint_t flags = 0;
    if (irq->methods != NULL && irq->methods->info != NULL) {
        flags = irq->methods->info(pcnt_in, MP_IRQ_INFO_FLAGS);
    }
    PCNT_TRACE("irq#%u flags=0x%x evt_l=0x%x evt_h=0x%x",
               (unsigned)my_n, (unsigned)flags,
               (unsigned)_pcnt_evt_l_lim, (unsigned)_pcnt_evt_h_lim);

    for (int i = 0; i < MAX_PCNT_UNITS; i++) {
        pcnt_encoder_obj_t *enc = _encoders_by_unit[i];
        if (enc != NULL && enc->pcnt == pcnt_in) {
            if (_pcnt_evt_h_lim != -1 && (flags & (mp_uint_t)_pcnt_evt_h_lim)) {
                enc->accum += (int64_t)PCNT_LIMIT;
                PCNT_TRACE("irq#%u +H accum_hi=%d accum_lo=%d",
                           (unsigned)my_n,
                           (int)(enc->accum >> 32),
                           (int)(enc->accum & 0xFFFFFFFF));
            }
            if (_pcnt_evt_l_lim != -1 && (flags & (mp_uint_t)_pcnt_evt_l_lim)) {
                enc->accum -= (int64_t)PCNT_LIMIT;
                PCNT_TRACE("irq#%u -L accum_hi=%d accum_lo=%d",
                           (unsigned)my_n,
                           (int)(enc->accum >> 32),
                           (int)(enc->accum & 0xFFFFFFFF));
            }
            enc->last_raw = 0;
            PCNT_TRACE("irq#%u exit unit=%d", (unsigned)my_n, i);
            return mp_const_none;
        }
    }
    PCNT_TRACE("irq#%u exit no-match", (unsigned)my_n);
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
    // mode_low, mode_high, min, max, filter — 10 kwargs. We deliberately
    // DON'T set threshold0/threshold1; the IRQ uses min/max instead so
    // the hardware can auto-reset the counter without our software
    // having to call pcnt.value() (which would recurse through the
    // synchronous-flush loop).
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
        MP_OBJ_NEW_QSTR(MP_QSTR_filter),     MP_OBJ_NEW_SMALL_INT(filter),
    };
    mp_obj_t pcnt_ch0 = mp_call_function_n_kw(PCNT_cls, 1, 10, ch0_args);

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

    // Arm the limit IRQ. PCNT.IRQ_MIN | PCNT.IRQ_MAX. Cache the
    // event-status bit values into static globals so the IRQ
    // dispatcher can decode which limit fired without re-importing
    // the esp32 module from the handler.
    //
    // Wrapped in nlr to keep the constructor working in tests where
    // the fake esp32 module no-ops these attrs.
    int irq_armed = 0;
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t MIN_obj = mp_load_attr(PCNT_cls, MP_QSTR_IRQ_MIN);
        mp_obj_t MAX_obj = mp_load_attr(PCNT_cls, MP_QSTR_IRQ_MAX);
        _pcnt_evt_l_lim = mp_obj_get_int(MIN_obj);
        _pcnt_evt_h_lim = mp_obj_get_int(MAX_obj);
        mp_int_t trigger = _pcnt_evt_l_lim | _pcnt_evt_h_lim;
        mp_obj_t irq_fn  = mp_load_attr(pcnt_ch0, MP_QSTR_irq);
        mp_obj_t irq_args[] = {
            irq_fn,
            MP_OBJ_NULL,
            MP_OBJ_NEW_QSTR(MP_QSTR_handler), MP_OBJ_FROM_PTR(&_pcnt_encoder_irq_dispatch_obj),
            MP_OBJ_NEW_QSTR(MP_QSTR_trigger), MP_OBJ_NEW_SMALL_INT(trigger),
        };
        mp_call_method_n_kw(0, 2, irq_args);
        nlr_pop();
        irq_armed = 1;
    }
    PCNT_TRACE("ctor unit=%d pin_a=%d pin_b=%d min=%d max=%d "
               "evt_l=0x%x evt_h=0x%x irq_armed=%d initial_raw=%d",
               (int)unit, (int)pin_a_num, (int)pin_b_num,
               (int)PCNT_MIN, (int)PCNT_MAX,
               (unsigned)_pcnt_evt_l_lim, (unsigned)_pcnt_evt_h_lim,
               irq_armed, (int)self->last_raw);
    // else: PCNT IRQ couldn't be armed (fake / older platform). The
    // pcnt_update_count wrap heuristic is still in place and works for
    // low-edge-rate use; high-rate use needs the IRQ on real hardware.

    return MP_OBJ_FROM_PTR(self);
}


// --- methods ----------------------------------------------------------

static mp_obj_t pcnt_encoder_count(mp_obj_t self_in) {
    pcnt_encoder_obj_t *self = MP_OBJ_TO_PTR(self_in);
    _diag_count_calls++;
    PCNT_TRACE("count#%u enter (irqs-so-far=%u)",
               (unsigned)_diag_count_calls, (unsigned)_diag_irq_count);
    int64_t v = pcnt_update_count(self);
    PCNT_TRACE("count#%u exit accum_hi=%d accum_lo=%d",
               (unsigned)_diag_count_calls,
               (int)(v >> 32), (int)(v & 0xFFFFFFFF));
    // ``mp_obj_new_int_from_ll`` returns a multi-precision int when the
    // value exceeds mp_int_t range, so 32-bit MicroPython ports don't
    // silently truncate the 64-bit accumulator on read.
    return mp_obj_new_int_from_ll((long long)v);
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
