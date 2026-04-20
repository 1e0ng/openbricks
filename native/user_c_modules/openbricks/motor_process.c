// SPDX-License-Identifier: MIT
//
// openbricks — motor_process.c
//
// The cooperative motor scheduler. A singleton owning a machine.Timer
// that fires periodically and invokes every registered Python callable
// once per tick.
//
// Public API (module: _openbricks_native, attribute: motor_process):
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
// The Python equivalent in tests/_fakes.py mirrors this behaviour exactly;
// desktop tests exercise that fake, the firmware exercises this C module,
// and both must pass the same tests in tests/test_scheduler.py.
//
// pbio reference: pbio/src/motor_process.c + pbio/src/os.c. This is a
// simplified port — just the tick + subscription dispatch; observer /
// trajectory / servo state machine land in M2.

#include "py/runtime.h"
#include "py/objlist.h"
#include "py/mphal.h"

#define DEFAULT_PERIOD_MS 10

// -----------------------------------------------------------------------
// Singleton state

typedef struct _motor_process_obj_t {
    mp_obj_base_t base;
    mp_obj_t      callbacks;   // mp_obj_list_t of callables
    mp_obj_t      timer;       // machine.Timer instance or mp_const_none
    mp_int_t      period_ms;
} motor_process_obj_t;

extern const mp_obj_type_t openbricks_motor_process_type;

static motor_process_obj_t motor_process_singleton = {
    .base       = { &openbricks_motor_process_type },
    .callbacks  = MP_OBJ_NULL,   // initialised lazily
    .timer      = MP_OBJ_NULL,
    .period_ms  = DEFAULT_PERIOD_MS,
};

static motor_process_obj_t *mp_get(void) {
    motor_process_obj_t *self = &motor_process_singleton;
    if (self->callbacks == MP_OBJ_NULL) {
        self->callbacks = mp_obj_new_list(0, NULL);
        self->timer     = mp_const_none;
    }
    return self;
}

// -----------------------------------------------------------------------
// Tick dispatch. Registered as the machine.Timer callback (which receives
// the Timer instance as its single argument). Iterates a snapshot of the
// callback list so a callback may unregister itself mid-tick.

static mp_obj_t mp_on_tick(mp_obj_t timer_arg) {
    (void)timer_arg;
    motor_process_obj_t *self = mp_get();

    size_t n;
    mp_obj_t *items;
    mp_obj_list_get(self->callbacks, &n, &items);

    // Copy into a stack snapshot so ``unregister`` during iteration is safe.
    // For MP heap hygiene prefer alloca-style over another mp_obj_list_new.
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

static mp_obj_t mp_machine_timer_class(void) {
    mp_obj_t machine_mod = mp_import_name(MP_QSTR_machine, mp_const_none, MP_OBJ_NEW_SMALL_INT(0));
    return mp_load_attr(machine_mod, MP_QSTR_Timer);
}

static void mp_do_start(motor_process_obj_t *self) {
    if (self->timer != mp_const_none) {
        return;
    }
    mp_obj_t timer_cls = mp_machine_timer_class();
    mp_obj_t periodic  = mp_load_attr(timer_cls, MP_QSTR_PERIODIC);
    self->timer        = mp_call_function_1(timer_cls, MP_OBJ_NEW_SMALL_INT(-1));

    mp_obj_t init_method = mp_load_attr(self->timer, MP_QSTR_init);

    // Call timer.init(period=<ms>, mode=PERIODIC, callback=<mp_on_tick_obj>)
    mp_map_elem_t kwargs[3];
    kwargs[0].key   = MP_OBJ_NEW_QSTR(MP_QSTR_period);
    kwargs[0].value = MP_OBJ_NEW_SMALL_INT(self->period_ms);
    kwargs[1].key   = MP_OBJ_NEW_QSTR(MP_QSTR_mode);
    kwargs[1].value = periodic;
    kwargs[2].key   = MP_OBJ_NEW_QSTR(MP_QSTR_callback);
    kwargs[2].value = MP_OBJ_FROM_PTR(&mp_on_tick_obj);

    mp_obj_t args[] = {
        init_method,
        kwargs[0].key, kwargs[0].value,
        kwargs[1].key, kwargs[1].value,
        kwargs[2].key, kwargs[2].value,
    };
    mp_call_method_n_kw(0, 3, args);
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
    // Dedupe: only append if not present.
    size_t n;
    mp_obj_t *items;
    mp_obj_list_get(self->callbacks, &n, &items);
    for (size_t i = 0; i < n; i++) {
        if (mp_obj_equal(items[i], callback)) {
            return mp_const_none;
        }
    }
    mp_obj_list_append(self->callbacks, callback);
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
            // Remove in place by calling list.remove() on the Python object.
            mp_obj_t rm = mp_load_attr(self->callbacks, MP_QSTR_remove);
            mp_call_function_1(rm, callback);
            return mp_const_none;
        }
    }
    // Silent if not registered (matches the Python fake).
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
    (void)n_args; (void)pos_args;
    static const mp_arg_t allowed[] = {
        { MP_QSTR_period_ms, MP_ARG_INT | MP_ARG_REQUIRED, {.u_int = DEFAULT_PERIOD_MS} },
    };
    mp_arg_val_t parsed[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args,
                     MP_ARRAY_SIZE(allowed), allowed, parsed);

    motor_process_obj_t *self = mp_get();
    self->period_ms = parsed[0].u_int;

    // If the timer is running, restart so the new period takes effect.
    if (self->timer != mp_const_none) {
        mp_do_stop(self);
        mp_do_start(self);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mp_configure_obj, 1, mp_configure);

static mp_obj_t mp_reset(mp_obj_t self_in) {
    (void)self_in;
    motor_process_obj_t *self = mp_get();
    mp_do_stop(self);
    // Empty the callbacks list.
    mp_obj_t clear = mp_load_attr(self->callbacks, MP_QSTR_clear);
    mp_call_function_0(clear);
    self->period_ms = DEFAULT_PERIOD_MS;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_reset_obj, mp_reset);

// -----------------------------------------------------------------------
// Type definition — hidden from Python; the singleton is the only
// instance users ever touch.

static const mp_rom_map_elem_t motor_process_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_register),   MP_ROM_PTR(&mp_register_obj) },
    { MP_ROM_QSTR(MP_QSTR_unregister), MP_ROM_PTR(&mp_unregister_obj) },
    { MP_ROM_QSTR(MP_QSTR_start),      MP_ROM_PTR(&mp_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),       MP_ROM_PTR(&mp_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_tick),       MP_ROM_PTR(&mp_tick_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_running), MP_ROM_PTR(&mp_is_running_obj) },
    { MP_ROM_QSTR(MP_QSTR_configure),  MP_ROM_PTR(&mp_configure_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset),      MP_ROM_PTR(&mp_reset_obj) },
};
static MP_DEFINE_CONST_DICT(motor_process_locals_dict, motor_process_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    openbricks_motor_process_type,
    MP_QSTR_MotorProcess,
    MP_TYPE_FLAG_NONE,
    locals_dict, &motor_process_locals_dict
);

// -----------------------------------------------------------------------
// Module registration

static const mp_rom_map_elem_t openbricks_native_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),      MP_ROM_QSTR(MP_QSTR__openbricks_native) },
    { MP_ROM_QSTR(MP_QSTR_motor_process), MP_ROM_PTR(&motor_process_singleton) },
};
static MP_DEFINE_CONST_DICT(openbricks_native_globals, openbricks_native_globals_table);

const mp_obj_module_t openbricks_native_cmodule = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&openbricks_native_globals,
};

MP_REGISTER_MODULE(MP_QSTR__openbricks_native, openbricks_native_cmodule);
