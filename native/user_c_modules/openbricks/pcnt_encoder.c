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
// Works on the whole ESP32 family with PCNT — original ESP32 (8 units),
// ESP32-S2, ESP32-S3 (4 units). The ``esp32`` module is imported at
// construction time, so on non-Espressif platforms the driver simply
// fails to instantiate — motor drivers should pick the software
// encoder instead.

#include <stdbool.h>

#include "py/obj.h"
#include "py/runtime.h"


// PCNT hardware counter is 16-bit signed (±32767 usable); we configure
// min/max at those limits explicitly so the hardware is also wrap-safe.
#define PCNT_MIN    (-32767)
#define PCNT_MAX    (+32767)
#define PCNT_RANGE  (PCNT_MAX - PCNT_MIN)   // = 65534

extern const mp_obj_type_t openbricks_pcnt_encoder_type;

typedef struct _pcnt_encoder_obj_t {
    mp_obj_base_t base;
    mp_obj_t      pcnt;         // esp32.PCNT object for channel 0 (shared unit)
    mp_obj_t      value_fn;     // cached bound method: pcnt.value
    mp_int_t      accum;        // signed total since construction / reset
    mp_int_t      last_raw;     // previous raw read, for wrap-aware delta
} pcnt_encoder_obj_t;


// --- helpers ----------------------------------------------------------

static mp_int_t pcnt_read_raw(pcnt_encoder_obj_t *self) {
    return mp_obj_get_int(mp_call_function_0(self->value_fn));
}

static void pcnt_write_raw_zero(pcnt_encoder_obj_t *self) {
    mp_call_function_1(self->value_fn, MP_OBJ_NEW_SMALL_INT(0));
}

// Fold any hardware delta since the last read into ``accum``, handling
// the 16-bit wrap. A step bigger than half the counter range means the
// hardware wrapped — put the wrap back.
static mp_int_t pcnt_update_count(pcnt_encoder_obj_t *self) {
    mp_int_t raw = pcnt_read_raw(self);
    mp_int_t delta = raw - self->last_raw;
    if (delta > PCNT_RANGE / 2) {
        delta -= PCNT_RANGE;
    } else if (delta < -(PCNT_RANGE / 2)) {
        delta += PCNT_RANGE;
    }
    self->accum += delta;
    self->last_raw = raw;
    return self->accum;
}

static void pcnt_reset_count(pcnt_encoder_obj_t *self, mp_int_t value) {
    self->accum = value;
    pcnt_write_raw_zero(self);
    self->last_raw = 0;
}


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
    // mode_low, mode_high, min, max, filter  (10 pairs).
    mp_obj_t ch0_args[] = {
        MP_OBJ_NEW_SMALL_INT(unit),                                          // positional: unit
        MP_OBJ_NEW_QSTR(MP_QSTR_channel),   MP_OBJ_NEW_SMALL_INT(0),
        MP_OBJ_NEW_QSTR(MP_QSTR_pin),       pin_a,
        MP_OBJ_NEW_QSTR(MP_QSTR_rising),    INCREMENT,
        MP_OBJ_NEW_QSTR(MP_QSTR_falling),   DECREMENT,
        MP_OBJ_NEW_QSTR(MP_QSTR_mode_pin),  pin_b,
        MP_OBJ_NEW_QSTR(MP_QSTR_mode_low),  NORMAL,
        MP_OBJ_NEW_QSTR(MP_QSTR_mode_high), REVERSE,
        MP_OBJ_NEW_QSTR(MP_QSTR_min),       MP_OBJ_NEW_SMALL_INT(PCNT_MIN),
        MP_OBJ_NEW_QSTR(MP_QSTR_max),       MP_OBJ_NEW_SMALL_INT(PCNT_MAX),
        MP_OBJ_NEW_QSTR(MP_QSTR_filter),    MP_OBJ_NEW_SMALL_INT(filter),
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
    return MP_OBJ_FROM_PTR(self);
}


// --- methods ----------------------------------------------------------

static mp_obj_t pcnt_encoder_count(mp_obj_t self_in) {
    pcnt_encoder_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(pcnt_update_count(self));
}
static MP_DEFINE_CONST_FUN_OBJ_1(pcnt_encoder_count_obj, pcnt_encoder_count);

static mp_obj_t pcnt_encoder_reset(size_t n_args, const mp_obj_t *args) {
    pcnt_encoder_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t value = (n_args > 1) ? mp_obj_get_int(args[1]) : 0;
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
