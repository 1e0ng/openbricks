// SPDX-License-Identifier: MIT
//
// openbricks — motor_process.c
//
// MicroPython binding shell for the cooperative motor scheduler.
// Owns:
//   - the Python-callback list (mp_obj_list_t of Python callables)
//   - the ``machine.Timer`` lifecycle
//
// Delegates to ``motor_process_core.{c,h}`` for:
//   - the C-callback registry
//   - the tick-driven monotonic clock
//   - the firing loop for the C-callback list
//
// The split is what lets the same servo / drivebase tick functions
// run unmodified in the host-side ``openbricks_sim._native``: the
// sim runner instantiates its own ``ob_motor_process_t`` and calls
// ``ob_motor_process_fire_c`` from the MuJoCo step loop, no
// MicroPython or Timer involved.
//
// Tick dispatch order (firmware path, called from the Timer ISR):
//
//   1. Fast path: native C callbacks via the shared core (~1 µs each).
//      All openbricks-provided closed-loop drivers (servo, drivebase,
//      future hub instrumentation) live here.
//   2. Slow path: Python callables registered via the public API.
//      Each call pays MicroPython dispatch overhead (~25 µs). For
//      user-extensible hooks (sensor loggers etc.), not the hot loop.
//
// Public Python API (module: _openbricks_native, attribute: motor_process):
//
//     motor_process.register(callback)      -> None
//     motor_process.unregister(callback)    -> None
//     motor_process.start()                 -> None
//     motor_process.stop()                  -> None
//     motor_process.tick()                  -> None       (synchronous one-shot)
//     motor_process.is_running()            -> bool
//     motor_process.configure(period_ms=N)  -> None
//     motor_process.reset()                 -> None       (test helper)

#include <stdbool.h>

#include "py/runtime.h"
#include "py/objlist.h"
#include "py/mphal.h"

#include "motor_process.h"
#include "motor_process_core.h"

#define DEFAULT_PERIOD_MS 1

// -----------------------------------------------------------------------
// Singleton state — the Python-binding side. The C-side state (callback
// table + virtual clock) lives in the shared core, in
// ``shared_core_singleton`` below.

struct _motor_process_obj_t {
    mp_obj_base_t base;
};

MP_REGISTER_ROOT_POINTER(mp_obj_t openbricks_mp_callbacks);
MP_REGISTER_ROOT_POINTER(mp_obj_t openbricks_mp_timer);

extern const mp_obj_type_t openbricks_motor_process_type;

motor_process_obj_t motor_process_singleton = {
    .base = { &openbricks_motor_process_type },
};

// The shared C-side state. Owned here so the public API
// (``openbricks_motor_process_register_c``, etc.) can forward to it
// without having to thread an instance pointer through every call site.
static ob_motor_process_t shared_core_singleton;
static bool               shared_core_initialised = false;


static ob_motor_process_t *core_get(void) {
    if (!shared_core_initialised) {
        ob_motor_process_init(&shared_core_singleton);
        shared_core_initialised = true;
    }
    return &shared_core_singleton;
}


// -----------------------------------------------------------------------
// Internal C API exported to siblings via motor_process.h. Forwards
// straight to the shared core; preserved as a thin wrapper so
// servo.c / drivebase.c didn't need a flag-day rename.

mp_int_t openbricks_motor_process_now_ms(void) {
    return (mp_int_t)core_get()->virtual_now_ms;
}


// Forward declaration — mp_do_start is defined later in this file.
static void mp_do_start(motor_process_obj_t *self);


void openbricks_motor_process_register_c(openbricks_tick_fn_t fn, void *ctx) {
    if (ob_motor_process_register_c(core_get(), fn, ctx) < 0) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("openbricks: too many C tick callbacks"));
    }
    // pbio-style: the scheduler runs any time there's work. Once started,
    // it stays running for the life of the interpreter (no auto-stop).
    mp_do_start(&motor_process_singleton);
}


void openbricks_motor_process_unregister_c(openbricks_tick_fn_t fn, void *ctx) {
    ob_motor_process_unregister_c(core_get(), fn, ctx);
}


// -----------------------------------------------------------------------
// Lazy init for the Python-side state. Same pattern as before — the
// list / timer slots are MP root-pointer ``MP_OBJ_NULL`` until the
// first access.

static motor_process_obj_t *mp_get(void) {
    motor_process_obj_t *self = &motor_process_singleton;
    if (MP_STATE_PORT(openbricks_mp_callbacks) == MP_OBJ_NULL) {
        MP_STATE_PORT(openbricks_mp_callbacks) = mp_obj_new_list(0, NULL);
        MP_STATE_PORT(openbricks_mp_timer)     = mp_const_none;
    }
    (void)core_get();   // make sure shared core is initialised too
    return self;
}


// -----------------------------------------------------------------------
// Tick dispatch. Registered as the machine.Timer callback (which
// receives the Timer instance as its single argument). C callbacks
// (via the shared core's ``fire_c``) fire first; Python callbacks
// second via a stack snapshot so self-unregistration mid-tick is safe.

static mp_obj_t mp_on_tick(mp_obj_t timer_arg) {
    (void)timer_arg;
    motor_process_obj_t *self = mp_get();
    (void)self;

    // Fast path — also advances the tick clock.
    ob_motor_process_fire_c(core_get());

    // Slow path: Python callbacks.
    size_t n;
    mp_obj_t *items;
    mp_obj_list_get(MP_STATE_PORT(openbricks_mp_callbacks), &n, &items);
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
    (void)self;
    if (MP_STATE_PORT(openbricks_mp_timer) != mp_const_none) {
        return;
    }
    // Try to create a machine.Timer. On ports that don't ship a Timer
    // (unix MP, for example), leave MP_STATE_PORT(openbricks_mp_timer)
    // as mp_const_none — the scheduler is still usable via explicit
    // ``tick()`` calls, which is how the test suite exercises it.
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t machine_mod = mp_import_name(MP_QSTR_machine, mp_const_none, MP_OBJ_NEW_SMALL_INT(0));
        mp_obj_t timer_cls   = mp_load_attr(machine_mod, MP_QSTR_Timer);
        mp_obj_t periodic    = mp_load_attr(timer_cls, MP_QSTR_PERIODIC);
        mp_obj_t timer       = mp_call_function_1(timer_cls, MP_OBJ_NEW_SMALL_INT(-1));
        mp_obj_t init_method = mp_load_attr(timer, MP_QSTR_init);

        // mp_call_method_n_kw expects args = [func, self_or_NULL,
        // positionals..., kw keys/vals]. ``init_method`` is already
        // bound to the timer instance, so we pass self = MP_OBJ_NULL
        // and let MP skip the self-prepend.
        mp_obj_t args[] = {
            init_method,
            MP_OBJ_NULL,
            MP_OBJ_NEW_QSTR(MP_QSTR_period),   MP_OBJ_NEW_SMALL_INT(core_get()->period_ms),
            MP_OBJ_NEW_QSTR(MP_QSTR_mode),     periodic,
            MP_OBJ_NEW_QSTR(MP_QSTR_callback), MP_OBJ_FROM_PTR(&mp_on_tick_obj),
        };
        mp_call_method_n_kw(0, 3, args);
        MP_STATE_PORT(openbricks_mp_timer) = timer;
        nlr_pop();
    }
    // else: machine.Timer unavailable — swallow the exception and
    // keep the singleton usable via ``tick()``.
}


static void mp_do_stop(motor_process_obj_t *self) {
    (void)self;
    if (MP_STATE_PORT(openbricks_mp_timer) == mp_const_none) {
        return;
    }
    mp_obj_t deinit = mp_load_attr(MP_STATE_PORT(openbricks_mp_timer), MP_QSTR_deinit);
    mp_call_function_0(deinit);
    MP_STATE_PORT(openbricks_mp_timer) = mp_const_none;
}


// -----------------------------------------------------------------------
// Python-facing methods

static mp_obj_t mp_register(mp_obj_t self_in, mp_obj_t callback) {
    (void)self_in;
    motor_process_obj_t *self = mp_get();
    size_t n;
    mp_obj_t *items;
    mp_obj_list_get(MP_STATE_PORT(openbricks_mp_callbacks), &n, &items);
    for (size_t i = 0; i < n; i++) {
        if (mp_obj_equal(items[i], callback)) {
            return mp_const_none;
        }
    }
    mp_obj_list_append(MP_STATE_PORT(openbricks_mp_callbacks), callback);
    mp_do_start(self);   // pbio-style auto-start on first subscription
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_register_obj, mp_register);

static mp_obj_t mp_unregister(mp_obj_t self_in, mp_obj_t callback) {
    (void)self_in;
    (void)mp_get();   // lazy init
    size_t n;
    mp_obj_t *items;
    mp_obj_list_get(MP_STATE_PORT(openbricks_mp_callbacks), &n, &items);
    for (size_t i = 0; i < n; i++) {
        if (mp_obj_equal(items[i], callback)) {
            mp_obj_t rm = mp_load_attr(MP_STATE_PORT(openbricks_mp_callbacks), MP_QSTR_remove);
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
    (void)mp_get();   // ensure lazy init has run before reading timer state
    return mp_obj_new_bool(MP_STATE_PORT(openbricks_mp_timer) != mp_const_none);
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
    ob_motor_process_set_period_ms(core_get(), parsed[0].u_int);

    if (MP_STATE_PORT(openbricks_mp_timer) != mp_const_none) {
        mp_do_stop(self);
        mp_do_start(self);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mp_configure_obj, 1, mp_configure);

static mp_obj_t mp_period_ms(mp_obj_t self_in) {
    (void)self_in;
    return MP_OBJ_NEW_SMALL_INT(core_get()->period_ms);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_period_ms_obj, mp_period_ms);

static mp_obj_t mp_is_registered(mp_obj_t self_in, mp_obj_t callback) {
    (void)self_in;
    (void)mp_get();
    size_t n;
    mp_obj_t *items;
    mp_obj_list_get(MP_STATE_PORT(openbricks_mp_callbacks), &n, &items);
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
    mp_obj_t clear = mp_load_attr(MP_STATE_PORT(openbricks_mp_callbacks), MP_QSTR_clear);
    mp_call_function_0(clear);
    ob_motor_process_reset(core_get());
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
