// SPDX-License-Identifier: MIT
//
// openbricks — encoder.c
//
// Software quadrature encoder — counts GPIO edges via machine.Pin IRQs.
// Counterpart to ``pcnt_encoder.c`` (hardware counter): same public API
// (``count`` / ``reset``), but without the ESP32 PCNT dependency, so this
// is the portable default for low-PPR encoders (JGB37-520 etc.) on any
// MP port.
//
// Everything runs in C, including the two IRQ paths that fire on every
// encoder edge. The previous Python implementation's handlers were ~10-30
// µs each through the MP dispatcher; the C version drops to ~1-5 µs and
// saves several percent of CPU on the typical JGB37 two-motor setup.
//
// Quadrature logic matches the Python original:
//   * A edge with A == B  -> count decreases (other channel leads)
//   * A edge with A != B  -> count increases
//   * B edge is the complement (A == B -> count++, else --).

#include <stdbool.h>

#include "py/obj.h"
#include "py/runtime.h"


extern const mp_obj_type_t openbricks_encoder_type;

typedef struct _encoder_obj_t {
    mp_obj_base_t base;
    mp_obj_t pin_a;
    mp_obj_t pin_b;
    mp_obj_t pin_a_value;    // cached: pin_a.value (bound method, called in ISR path)
    mp_obj_t pin_b_value;
    mp_obj_t irq_a_handler;  // bound method self._on_edge_a, kept alive here
    mp_obj_t irq_b_handler;
    mp_int_t count;
} encoder_obj_t;


// --- IRQ handlers (method bindings) ---
//
// Called as ``self._on_edge_a(pin)`` / ``self._on_edge_b(pin)`` from the
// MP IRQ dispatch when the respective edge fires. Keep them allocation-
// free: only call pin.value() (returns a small int) and mutate the C int
// ``count`` field.

static mp_obj_t encoder_on_edge_a(mp_obj_t self_in, mp_obj_t pin_in) {
    (void)pin_in;
    encoder_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_int_t a = mp_obj_get_int(mp_call_function_0(self->pin_a_value));
    mp_int_t b = mp_obj_get_int(mp_call_function_0(self->pin_b_value));
    if (a == b) {
        self->count -= 1;
    } else {
        self->count += 1;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(encoder_on_edge_a_obj, encoder_on_edge_a);

static mp_obj_t encoder_on_edge_b(mp_obj_t self_in, mp_obj_t pin_in) {
    (void)pin_in;
    encoder_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_int_t a = mp_obj_get_int(mp_call_function_0(self->pin_a_value));
    mp_int_t b = mp_obj_get_int(mp_call_function_0(self->pin_b_value));
    if (a == b) {
        self->count += 1;
    } else {
        self->count -= 1;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(encoder_on_edge_b_obj, encoder_on_edge_b);


// --- constructor ---
//
// QuadratureEncoder(pin_a, pin_b)
//
// Both pins are configured Pin.IN with PULL_UP (matching the Python
// original), and both edges on both channels wake the IRQ handlers.

static mp_obj_t encoder_make_new(const mp_obj_type_t *type,
                                 size_t n_args, size_t n_kw,
                                 const mp_obj_t *all_args) {
    enum { ARG_pin_a, ARG_pin_b };
    static const mp_arg_t allowed[] = {
        { MP_QSTR_pin_a, MP_ARG_INT | MP_ARG_REQUIRED, {.u_int = 0} },
        { MP_QSTR_pin_b, MP_ARG_INT | MP_ARG_REQUIRED, {.u_int = 0} },
    };
    mp_arg_val_t parsed[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args,
                              MP_ARRAY_SIZE(allowed), allowed, parsed);

    mp_int_t pin_a_num = parsed[ARG_pin_a].u_int;
    mp_int_t pin_b_num = parsed[ARG_pin_b].u_int;

    mp_obj_t machine_mod = mp_import_name(MP_QSTR_machine, mp_const_none, MP_OBJ_NEW_SMALL_INT(0));
    mp_obj_t Pin_cls     = mp_load_attr(machine_mod, MP_QSTR_Pin);
    mp_obj_t pin_in_mode = mp_load_attr(Pin_cls, MP_QSTR_IN);
    mp_obj_t pin_pull_up = mp_load_attr(Pin_cls, MP_QSTR_PULL_UP);
    mp_obj_t irq_rising  = mp_load_attr(Pin_cls, MP_QSTR_IRQ_RISING);
    mp_obj_t irq_falling = mp_load_attr(Pin_cls, MP_QSTR_IRQ_FALLING);

    mp_obj_t a_args[3] = { MP_OBJ_NEW_SMALL_INT(pin_a_num), pin_in_mode, pin_pull_up };
    mp_obj_t b_args[3] = { MP_OBJ_NEW_SMALL_INT(pin_b_num), pin_in_mode, pin_pull_up };
    mp_obj_t pin_a = mp_call_function_n_kw(Pin_cls, 3, 0, a_args);
    mp_obj_t pin_b = mp_call_function_n_kw(Pin_cls, 3, 0, b_args);

    // Combined RISING|FALLING trigger — count every edge on both channels.
    mp_int_t trigger = mp_obj_get_int(irq_rising) | mp_obj_get_int(irq_falling);

    encoder_obj_t *self = mp_obj_malloc(encoder_obj_t, type);
    self->base.type     = type;
    self->pin_a         = pin_a;
    self->pin_b         = pin_b;
    self->pin_a_value   = mp_load_attr(pin_a, MP_QSTR_value);
    self->pin_b_value   = mp_load_attr(pin_b, MP_QSTR_value);
    self->count         = 0;

    // Bind the C IRQ methods to this instance so Pin.irq gets a per-instance
    // callable that MP can invoke with ``handler(pin)``. Store them here
    // too so they can't be GC'd while the IRQ is live.
    self->irq_a_handler = mp_load_attr(MP_OBJ_FROM_PTR(self), MP_QSTR__on_edge_a);
    self->irq_b_handler = mp_load_attr(MP_OBJ_FROM_PTR(self), MP_QSTR__on_edge_b);

    // pin_a.irq(trigger=..., handler=...)
    mp_obj_t a_irq = mp_load_attr(pin_a, MP_QSTR_irq);
    mp_obj_t a_kw[] = {
        MP_OBJ_NEW_QSTR(MP_QSTR_trigger), MP_OBJ_NEW_SMALL_INT(trigger),
        MP_OBJ_NEW_QSTR(MP_QSTR_handler), self->irq_a_handler,
    };
    mp_call_function_n_kw(a_irq, 0, 2, a_kw);

    mp_obj_t b_irq = mp_load_attr(pin_b, MP_QSTR_irq);
    mp_obj_t b_kw[] = {
        MP_OBJ_NEW_QSTR(MP_QSTR_trigger), MP_OBJ_NEW_SMALL_INT(trigger),
        MP_OBJ_NEW_QSTR(MP_QSTR_handler), self->irq_b_handler,
    };
    mp_call_function_n_kw(b_irq, 0, 2, b_kw);

    return MP_OBJ_FROM_PTR(self);
}


// --- methods: count / reset / pin_a / pin_b ---

static mp_obj_t encoder_count(mp_obj_t self_in) {
    encoder_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(self->count);
}
static MP_DEFINE_CONST_FUN_OBJ_1(encoder_count_obj, encoder_count);

static mp_obj_t encoder_reset(size_t n_args, const mp_obj_t *args) {
    encoder_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    self->count = (n_args > 1) ? mp_obj_get_int(args[1]) : 0;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(encoder_reset_obj, 1, 2, encoder_reset);

// pin_a() / pin_b() getters — needed by tests to drive the fake pins'
// value() and fire their IRQ handlers. Not used on hot path.
static mp_obj_t encoder_pin_a(mp_obj_t self_in) {
    encoder_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return self->pin_a;
}
static MP_DEFINE_CONST_FUN_OBJ_1(encoder_pin_a_obj, encoder_pin_a);

static mp_obj_t encoder_pin_b(mp_obj_t self_in) {
    encoder_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return self->pin_b;
}
static MP_DEFINE_CONST_FUN_OBJ_1(encoder_pin_b_obj, encoder_pin_b);


// --- type ---

static const mp_rom_map_elem_t encoder_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_count),       MP_ROM_PTR(&encoder_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset),       MP_ROM_PTR(&encoder_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_pin_a),       MP_ROM_PTR(&encoder_pin_a_obj) },
    { MP_ROM_QSTR(MP_QSTR_pin_b),       MP_ROM_PTR(&encoder_pin_b_obj) },
    { MP_ROM_QSTR(MP_QSTR__on_edge_a),  MP_ROM_PTR(&encoder_on_edge_a_obj) },
    { MP_ROM_QSTR(MP_QSTR__on_edge_b),  MP_ROM_PTR(&encoder_on_edge_b_obj) },
};
static MP_DEFINE_CONST_DICT(encoder_locals_dict, encoder_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    openbricks_encoder_type,
    MP_QSTR_QuadratureEncoder,
    MP_TYPE_FLAG_NONE,
    make_new,    encoder_make_new,
    locals_dict, &encoder_locals_dict
);
