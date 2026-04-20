// SPDX-License-Identifier: MIT
//
// openbricks — motor_process.c
//
// The cooperative motor scheduler. A singleton owning a machine.Timer
// that fires at the configured period (1 ms by default — 1 kHz, matching
// pbio). Each tick fires two callback lists in order:
//
//   1. Fast path: native C callbacks registered via motor_process.h.
//      These are plain function-pointer calls, ~1 µs each. All
//      openbricks-provided closed-loop drivers (servo.c, and later
//      observer / trajectory / hub) live here.
//   2. Slow path: Python callables registered via the public API below.
//      Each call pays MicroPython dispatch overhead (~25 µs). Intended
//      for user-extensible hooks (e.g. a sensor logger), not for the
//      hot control loop.
//
// Public Python API (module: _openbricks_native, attribute: motor_process):
//
//     motor_process.register(callback)      -> None
//     motor_process.unregister(callback)    -> None
//     motor_process.start()                 -> None
//     motor_process.stop()                  -> None
//     motor_process.tick()                  -> None      (synchronous one-shot)
//     motor_process.is_running()            -> bool
//     motor_process.configure(period_ms=N)  -> None
//     motor_process.reset()                 -> None      (test helper)
//
// Internal C API: see motor_process.h. Sibling C modules use that to
// subscribe their tick bodies without going through Python.
//
// The Python equivalent in tests/_fakes.py mirrors this behaviour exactly;
// desktop tests exercise that fake, the firmware exercises this C module.

#include "py/runtime.h"
#include "py/objlist.h"
#include "py/mphal.h"

#include "motor_process.h"

#define DEFAULT_PERIOD_MS 1
#define MAX_C_CALLBACKS   8   // raise if the fleet of native controllers grows

// -----------------------------------------------------------------------
// Singleton state

struct _motor_process_obj_t {
    mp_obj_base_t base;
    mp_obj_t      callbacks;   // mp_obj_list_t of Python callables
    mp_obj_t      timer;       // machine.Timer instance or mp_const_none
    mp_int_t      period_ms;
};

typedef struct {
    openbricks_tick_fn_t fn;
    void *ctx;
} c_callback_slot_t;

extern const mp_obj_type_t openbricks_motor_process_type;

motor_process_obj_t motor_process_singleton = {
    .base       = { &openbricks_motor_process_type },
    .callbacks  = MP_OBJ_NULL,   // initialised lazily
    .timer      = MP_OBJ_NULL,
    .period_ms  = DEFAULT_PERIOD_MS,
};

static c_callback_slot_t c_callbacks[MAX_C_CALLBACKS];
static size_t n_c_callbacks = 0;

// Tick-driven monotonic clock. Advances by ``period_ms`` each time
// ``_on_tick`` fires — so time measurements in downstream C code
// (servo / drivebase) track the tick cadence rather than real wall
// time. This makes virtual-clock tests deterministic: each simulated
// tick advances the clock by exactly one period, same as on firmware.
static mp_int_t virtual_now_ms = 0;

mp_int_t openbricks_motor_process_now_ms(void) {
    return virtual_now_ms;
}

static motor_process_obj_t *mp_get(void) {
    motor_process_obj_t *self = &motor_process_singleton;
    if (self->callbacks == MP_OBJ_NULL) {
        self->callbacks = mp_obj_new_list(0, NULL);
        self->timer     = mp_const_none;
    }
    return self;
}

// -----------------------------------------------------------------------
// Internal C API (see motor_process.h)

// Forward declaration — mp_do_start is defined later in this file.
static void mp_do_start(motor_process_obj_t *self);

void openbricks_motor_process_register_c(openbricks_tick_fn_t fn, void *ctx) {
    for (size_t i = 0; i < n_c_callbacks; i++) {
        if (c_callbacks[i].fn == fn && c_callbacks[i].ctx == ctx) {
            return;  // already registered
        }
    }
    if (n_c_callbacks >= MAX_C_CALLBACKS) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("openbricks: too many C tick callbacks"));
    }
    c_callbacks[n_c_callbacks].fn  = fn;
    c_callbacks[n_c_callbacks].ctx = ctx;
    n_c_callbacks++;
    // pbio-style: the scheduler runs any time there's work. Once started,
    // it stays running for the life of the interpreter (no auto-stop).
    mp_do_start(mp_get());
}

void openbricks_motor_process_unregister_c(openbricks_tick_fn_t fn, void *ctx) {
    for (size_t i = 0; i < n_c_callbacks; i++) {
        if (c_callbacks[i].fn == fn && c_callbacks[i].ctx == ctx) {
            for (size_t j = i; j + 1 < n_c_callbacks; j++) {
                c_callbacks[j] = c_callbacks[j + 1];
            }
            n_c_callbacks--;
            return;
        }
    }
}

// -----------------------------------------------------------------------
// Tick dispatch. Registered as the machine.Timer callback (which receives
// the Timer instance as its single argument). C callbacks fire first;
// Python callbacks second via a stack snapshot so self-unregistration
// mid-tick is safe.

static mp_obj_t mp_on_tick(mp_obj_t timer_arg) {
    (void)timer_arg;
    motor_process_obj_t *self = mp_get();

    // Advance the tick-driven clock before firing callbacks so subscribers
    // see a consistent "now" for their deltas.
    virtual_now_ms += self->period_ms;

    // Fast path: native C callbacks. Tight loop, no MP allocation.
    for (size_t i = 0; i < n_c_callbacks; i++) {
        c_callbacks[i].fn(c_callbacks[i].ctx);
    }

    // Slow path: Python callbacks.
    size_t n;
    mp_obj_t *items;
    mp_obj_list_get(self->callbacks, &n, &items);
    if (n == 0) {
        return mp_const_none;
    }
    mp_obj_t *snap = m_new(mp_obj_t, n);
    for (size_t i = 0; i < n; i++) {
        snap[i] = items[i];
    }
    for (size_t i = 0; i < n; i++) {
        mp_call_function_0(snap[i]);
    }
    m_del(mp_obj_t, snap, n);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_on_tick_obj, mp_on_tick);

// -----------------------------------------------------------------------
// Timer lifecycle helpers. machine.Timer is accessed lazily via the
// ``machine`` module so we don't hard-depend on its header.

static void mp_do_start(motor_process_obj_t *self) {
    if (self->timer != mp_const_none) {
        return;
    }
    // Try to create a machine.Timer. On ports that don't ship a Timer
    // (unix MP, for example), leave self->timer as mp_const_none — the
    // scheduler is still usable via explicit ``tick()`` calls, which
    // is how the test suite exercises it.
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t machine_mod = mp_import_name(MP_QSTR_machine, mp_const_none, MP_OBJ_NEW_SMALL_INT(0));
        mp_obj_t timer_cls   = mp_load_attr(machine_mod, MP_QSTR_Timer);
        mp_obj_t periodic    = mp_load_attr(timer_cls, MP_QSTR_PERIODIC);
        mp_obj_t timer       = mp_call_function_1(timer_cls, MP_OBJ_NEW_SMALL_INT(-1));
        mp_obj_t init_method = mp_load_attr(timer, MP_QSTR_init);

        // mp_call_method_n_kw expects args = [func, self_or_NULL, positionals..., kw keys/vals].
        // Our ``init_method`` is already bound to the timer instance, so we
        // pass self = MP_OBJ_NULL and let MP skip the self-prepend.
        mp_obj_t args[] = {
            init_method,
            MP_OBJ_NULL,
            MP_OBJ_NEW_QSTR(MP_QSTR_period),   MP_OBJ_NEW_SMALL_INT(self->period_ms),
            MP_OBJ_NEW_QSTR(MP_QSTR_mode),     periodic,
            MP_OBJ_NEW_QSTR(MP_QSTR_callback), MP_OBJ_FROM_PTR(&mp_on_tick_obj),
        };
        mp_call_method_n_kw(0, 3, args);
        self->timer = timer;
        nlr_pop();
    }
    // else: machine.Timer unavailable — swallow the exception and keep
    // the singleton usable via tick().
}

static void mp_do_stop(motor_process_obj_t *self) {
    if (self->timer == mp_const_none) {
        return;
    }
    mp_obj_t deinit = mp_load_attr(self->timer, MP_QSTR_deinit);
    mp_call_function_0(deinit);
    self->timer = mp_const_none;
}

// -----------------------------------------------------------------------
// Python-facing methods

static mp_obj_t mp_register(mp_obj_t self_in, mp_obj_t callback) {
    (void)self_in;
    motor_process_obj_t *self = mp_get();
    size_t n;
    mp_obj_t *items;
    mp_obj_list_get(self->callbacks, &n, &items);
    for (size_t i = 0; i < n; i++) {
        if (mp_obj_equal(items[i], callback)) {
            return mp_const_none;
        }
    }
    mp_obj_list_append(self->callbacks, callback);
    mp_do_start(self);   // pbio-style auto-start on first subscription
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_register_obj, mp_register);

static mp_obj_t mp_unregister(mp_obj_t self_in, mp_obj_t callback) {
    (void)self_in;
    motor_process_obj_t *self = mp_get();
    size_t n;
    mp_obj_t *items;
    mp_obj_list_get(self->callbacks, &n, &items);
    for (size_t i = 0; i < n; i++) {
        if (mp_obj_equal(items[i], callback)) {
            mp_obj_t rm = mp_load_attr(self->callbacks, MP_QSTR_remove);
            mp_call_function_1(rm, callback);
            return mp_const_none;
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_unregister_obj, mp_unregister);

static mp_obj_t mp_start(mp_obj_t self_in) {
    (void)self_in;
    mp_do_start(mp_get());
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_start_obj, mp_start);

static mp_obj_t mp_stop(mp_obj_t self_in) {
    (void)self_in;
    mp_do_stop(mp_get());
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_stop_obj, mp_stop);

static mp_obj_t mp_tick(mp_obj_t self_in) {
    (void)self_in;
    return mp_on_tick(mp_const_none);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_tick_obj, mp_tick);

static mp_obj_t mp_is_running(mp_obj_t self_in) {
    (void)self_in;
    return mp_obj_new_bool(mp_get()->timer != mp_const_none);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_is_running_obj, mp_is_running);

static mp_obj_t mp_configure(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed[] = {
        { MP_QSTR_period_ms, MP_ARG_INT | MP_ARG_REQUIRED, {.u_int = DEFAULT_PERIOD_MS} },
    };
    mp_arg_val_t parsed[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args,
                     MP_ARRAY_SIZE(allowed), allowed, parsed);

    motor_process_obj_t *self = mp_get();
    self->period_ms = parsed[0].u_int;

    if (self->timer != mp_const_none) {
        mp_do_stop(self);
        mp_do_start(self);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mp_configure_obj, 1, mp_configure);

static mp_obj_t mp_period_ms(mp_obj_t self_in) {
    (void)self_in;
    return MP_OBJ_NEW_SMALL_INT(mp_get()->period_ms);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_period_ms_obj, mp_period_ms);

static mp_obj_t mp_is_registered(mp_obj_t self_in, mp_obj_t callback) {
    (void)self_in;
    motor_process_obj_t *self = mp_get();
    size_t n;
    mp_obj_t *items;
    mp_obj_list_get(self->callbacks, &n, &items);
    for (size_t i = 0; i < n; i++) {
        if (mp_obj_equal(items[i], callback)) {
            return mp_const_true;
        }
    }
    return mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_is_registered_obj, mp_is_registered);

static mp_obj_t mp_reset(mp_obj_t self_in) {
    (void)self_in;
    motor_process_obj_t *self = mp_get();
    mp_do_stop(self);
    mp_obj_t clear = mp_load_attr(self->callbacks, MP_QSTR_clear);
    mp_call_function_0(clear);
    self->period_ms   = DEFAULT_PERIOD_MS;
    n_c_callbacks     = 0;
    virtual_now_ms    = 0;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_reset_obj, mp_reset);

// -----------------------------------------------------------------------
// Type definition — hidden from Python; the singleton is the only
// instance users ever touch.

static const mp_rom_map_elem_t motor_process_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_register),      MP_ROM_PTR(&mp_register_obj) },
    { MP_ROM_QSTR(MP_QSTR_unregister),    MP_ROM_PTR(&mp_unregister_obj) },
    { MP_ROM_QSTR(MP_QSTR_start),         MP_ROM_PTR(&mp_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),          MP_ROM_PTR(&mp_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_tick),          MP_ROM_PTR(&mp_tick_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_running),    MP_ROM_PTR(&mp_is_running_obj) },
    { MP_ROM_QSTR(MP_QSTR_configure),     MP_ROM_PTR(&mp_configure_obj) },
    { MP_ROM_QSTR(MP_QSTR_period_ms),     MP_ROM_PTR(&mp_period_ms_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_registered), MP_ROM_PTR(&mp_is_registered_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset),         MP_ROM_PTR(&mp_reset_obj) },
};
static MP_DEFINE_CONST_DICT(motor_process_locals_dict, motor_process_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    openbricks_motor_process_type,
    MP_QSTR_MotorProcess,
    MP_TYPE_FLAG_NONE,
    locals_dict, &motor_process_locals_dict
);
