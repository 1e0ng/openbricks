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
#include "imu_yaw_core.h"

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

static mp_obj_t tick_dispatch(int use_wall, uint32_t wall_ms) {
    motor_process_obj_t *self = mp_get();
    (void)self;

    // Fast path — also advances the tick clock. The wall-clocked
    // variant advances by REAL elapsed time: machine.Timer callbacks
    // ride micropython.schedule's droppable queue, so counting fires
    // (the old behaviour) dilated the controllers' clock under
    // scheduler starvation — a bench 981 ms gap advanced it by
    // single-digit ms, slowing every trajectory mid-run.
    if (use_wall) {
        ob_motor_process_fire_c_at(core_get(), wall_ms);
    } else {
        ob_motor_process_fire_c(core_get());
    }

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


// ---- hard tick (below-the-scheduler C callback) ----
//
// Provided by the esp32-port build-time patch (native/patches/
// esp32-openbricks-hard-tick.patch): a periodic C callback on the
// esp_timer service task, unaffected by the Python scheduler. The
// symbols are declared extern (this module cannot include ESP-IDF
// headers — ESP_PLATFORM is undefined here) and gated on the config
// macro the patch defines; ports without the patch (unix, sim) fall
// back to the scheduler-dispatched tick below.
//
// Current consumer: a probe counter proving on real hardware that
// the hook fires and keeps firing while Python blocks — the
// foundation gate for moving serial-bus motor control down here.
// CONTRACT for anything running in that context: no Python objects,
// no GC allocation, no mp_* VM calls. Pure C state only.
#if defined(MICROPY_OPENBRICKS_HARD_TICK) && MICROPY_OPENBRICKS_HARD_TICK
extern int ob_hard_tick_install(void (*fn)(void *), void *ctx, uint32_t period_us);
extern void ob_hard_tick_uninstall(void);
extern uint32_t ob_hard_ticks_ms(void);

static volatile uint32_t hard_tick_probe_count;

#if defined(MICROPY_OPENBRICKS_GPIO_SHIM) && MICROPY_OPENBRICKS_GPIO_SHIM
// Hard-button sampling (st_button_core): the program button handled
// on the hard tick — core 0, un-starvable. STOP (armed + press):
// interrupt injection + native-bus torque-off fire from C within
// ~2 ms of the debounced edge, bounded regardless of Python's state
// (the scheduler path measured gaps to 981 ms). START stays a flag
// the launcher's main-thread idle loop consumes — only the main
// thread can exec a program, and start is not latency-critical.
#include "st_button_core.h"
extern int ob_gpio_input_pullup(int pin);
extern int ob_gpio_read(int pin);
extern void ob_st_bus_estop_from_tick(void);

static ob_button_t hard_button;
static volatile int      hard_button_pin = -1;
static volatile uint8_t  hard_button_armed;
static volatile uint8_t  hard_button_start_pending;
static volatile uint32_t hard_button_stops;

static int hb_read(void *ctx) {
    (void)ctx;
    return ob_gpio_read(hard_button_pin) == 0;   // active-low
}

// A hard stop's injection is AT-LEAST-ONCE, not one-shot: the
// pending KeyboardInterrupt can be swallowed silently — by design —
// when it lands inside a dupterm stream method (Part 6 wraps those
// in `except BaseException` because a raising stream gets
// deactivated permanently). The Python watcher's retry machinery
// covers ITS stops; a hard-path stop outruns the watcher and armed
// nothing (bench 2026-08-03: hard_stops incremented, program ran
// 1.3 s more under a print storm until a second press). So the hard
// tick re-injects every RETRY ticks while the stop is in flight;
// the teardown's disarm ends the flight.
#define HARD_STOP_RETRY_TICKS 100    // ~100 ms at 1 kHz

static volatile uint16_t hard_stop_inflight;   // 0 = idle

static void hard_button_tick(void) {
    if (hard_button_pin < 0) {
        return;
    }
    if (hard_stop_inflight) {
        if (!hard_button_armed) {
            hard_stop_inflight = 0;    // teardown disarmed: delivered
        } else if (++hard_stop_inflight > HARD_STOP_RETRY_TICKS) {
            mp_sched_keyboard_interrupt();
            hard_stop_inflight = 1;
        }
    }
    ob_button_event_t e = ob_button_tick(&hard_button);
    if (e != OB_BUTTON_PRESSED) {
        return;
    }
    if (hard_button_armed) {
        hard_button_stops++;
        ob_st_bus_estop_from_tick();
        // ISR-safe by design (the UART RX ISR calls it); sets the
        // pending KeyboardInterrupt the running program unwinds on.
        mp_sched_keyboard_interrupt();
        hard_stop_inflight = 1;
    } else {
        hard_button_start_pending = 1;
    }
}
#else
static void hard_button_tick(void) { }
#endif

// The serial-bus pump (st_bus.c). One hook slot, one dispatcher:
// every hard-context consumer hangs off this function, in order.
extern void ob_st_bus_hard_poll(void);

// Defined with its install API in the unguarded yaw section below;
// tentatively declared here because this dispatcher (guarded,
// firmware-only) reads it — the unix build never compiles this
// function, which is how the use-before-declaration slipped local
// checks.
static void (*volatile hard_imu_fn)(void);

static void hard_tick_dispatch(void *ctx) {
    (void)ctx;
    // Aligned 32-bit increment; read side is a single aligned load.
    hard_tick_probe_count = hard_tick_probe_count + 1;
    hard_button_tick();          // BEFORE the pump: stop jumps the queue
    if (hard_imu_fn) {
        hard_imu_fn();           // IMU read + yaw feed (~13 us SPI)
    }
    ob_st_bus_hard_poll();
}
#endif

static mp_obj_t mp_hard_tick_available(mp_obj_t self_in) {
    (void)self_in;
    #if defined(MICROPY_OPENBRICKS_HARD_TICK) && MICROPY_OPENBRICKS_HARD_TICK
    return mp_const_true;
    #else
    return mp_const_false;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_hard_tick_available_obj, mp_hard_tick_available);

#if defined(MICROPY_OPENBRICKS_HARD_TICK) && MICROPY_OPENBRICKS_HARD_TICK
static mp_obj_t mp_hard_tick_selftest(mp_obj_t self_in) {
    // Install the probe at 1 kHz. Idempotent by way of the single
    // hook slot: a second call reports the already-running state.
    (void)self_in;
    int r = ob_hard_tick_install(hard_tick_dispatch, NULL, 1000);
    return mp_obj_new_bool(r == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_hard_tick_selftest_obj, mp_hard_tick_selftest);

static mp_obj_t mp_hard_tick_count(mp_obj_t self_in) {
    (void)self_in;
    return mp_obj_new_int_from_uint(hard_tick_probe_count);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_hard_tick_count_obj, mp_hard_tick_count);
#endif

// Whether the Timer path advances the clock by real elapsed time
// (mp_hal_ticks_ms) instead of one period per fire. OFF by default
// and enabled by the frozen boot.py ONLY on real hardware
// (sys.platform == "esp32"): the unix test environment drives a fake
// machine.Timer against a fake virtual clock, and wall-clocking
// there reads the REAL host clock — two clocks for one world, which
// sent trajectory tests 1000x past their targets when this was
// unconditional. A plain C static (not MP state) so it needs no GC
// root; boot.py re-runs on every soft reset and re-enables it.
static bool wall_clock_mode = false;

static mp_obj_t mp_on_tick(mp_obj_t timer_arg) {
    (void)timer_arg;
    if (wall_clock_mode) {
        return tick_dispatch(1, (uint32_t)mp_hal_ticks_ms());
    }
    return tick_dispatch(0, 0);
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
    //
    // Timer ID 2: ESP32-S3 only supports hardware timers 0..3 — virtual
    // Timer(-1) raises ValueError there. Timer 0 is taken by
    // ``openbricks.launcher`` (button watcher) and Timer 1 by
    // ``openbricks.bluetooth_button`` (BLE-toggle). Use Timer 2 for the
    // motor_process scheduler so all three coexist on a stock-build hub.
    // (Other ports — esp32 classic, unix MP — accept this ID too;
    // hardware-timer 2 exists on every port we ship.)
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t machine_mod = mp_import_name(MP_QSTR_machine, mp_const_none, MP_OBJ_NEW_SMALL_INT(0));
        mp_obj_t timer_cls   = mp_load_attr(machine_mod, MP_QSTR_Timer);
        mp_obj_t periodic    = mp_load_attr(timer_cls, MP_QSTR_PERIODIC);
        mp_obj_t timer       = mp_call_function_1(timer_cls, MP_OBJ_NEW_SMALL_INT(2));
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

static mp_obj_t mp_tick(size_t n_args, const mp_obj_t *args) {
    // tick()   -> deterministic: clock += period_ms (tests, sim)
    // tick(N)  -> wall-clocked with N as the monotonic-ms timestamp —
    //             exactly the firmware Timer path; the seam that lets
    //             the unix suite pin wall-dt/clamp/wrap semantics
    //             without sleeping.
    (void)args;
    if (n_args == 2) {
        return tick_dispatch(1, (uint32_t)mp_obj_get_int_truncated(args[1]));
    }
    return tick_dispatch(0, 0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_tick_obj, 1, 2, mp_tick);

static mp_obj_t mp_set_wall_clock(mp_obj_t self_in, mp_obj_t enable) {
    (void)self_in;
    (void)mp_get();   // lazy init
    wall_clock_mode = mp_obj_is_true(enable);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_set_wall_clock_obj, mp_set_wall_clock);

static mp_obj_t mp_wall_clock(mp_obj_t self_in) {
    (void)self_in;
    return mp_obj_new_bool(wall_clock_mode);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_wall_clock_obj, mp_wall_clock);

#if defined(MICROPY_OPENBRICKS_HARD_TICK) && MICROPY_OPENBRICKS_HARD_TICK \
    && defined(MICROPY_OPENBRICKS_GPIO_SHIM) && MICROPY_OPENBRICKS_GPIO_SHIM
static mp_obj_t mp_hard_button_config(mp_obj_t self_in, mp_obj_t pin_in) {
    (void)self_in;
    int pin = mp_obj_get_int(pin_in);
    if (pin == hard_button_pin) {
        // Already sampling this pin: keep the debounce state AND the
        // press/stop counters. The launcher re-runs its setup on
        // idle-loop restarts (and BLE session recovery can reboot
        // it); re-initing here zeroed the counters and destroyed the
        // evidence of whether the HARD path delivered a stop — bench
        // 2026-08-03: stats read (0, 0, 0, True) right after a
        // successful button stop.
        return mp_const_true;
    }
    if (ob_gpio_input_pullup(pin) != 0) {
        return mp_const_false;
    }
    ob_button_init(&hard_button, hb_read, NULL);
    hard_stop_inflight = 0;
    hard_button_pin = pin;   // set LAST: the tick keys off it
    // Ensure the dispatcher is installed (idempotent single slot).
    (void)ob_hard_tick_install(hard_tick_dispatch, NULL, 1000);
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_hard_button_config_obj, mp_hard_button_config);

static mp_obj_t mp_hard_button_arm(mp_obj_t self_in, mp_obj_t on_in) {
    (void)self_in;
    hard_button_armed = mp_obj_is_true(on_in) ? 1 : 0;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_hard_button_arm_obj, mp_hard_button_arm);

static mp_obj_t mp_hard_button_probe(mp_obj_t self_in) {
    // The sampler's OWN view of the world, for the bench diagnostic
    // (2026-08-03: dispatcher alive, config True, machine.Pin reads
    // the pad, Python watcher stops on the press — yet n_presses
    // stayed 0; every link visible from Python was healthy, so
    // expose the invisible ones): the pin the tick keys off,
    // ob_gpio_read's answer for it RIGHT NOW, and the debounce
    // machine's raw_last / window-count / stable_pressed. Since the
    // N-of-M rewrite (1.49.0), the fourth field is the number of
    // pressed samples in the last OB_BUTTON_WINDOW — during a held
    // press it should sit near 20; hovering mid-range = heavy
    // chatter; ob_gpio_read stuck at 1 while machine.Pin reads 0 =
    // the shim and the pad disagree.
    (void)self_in;
    int pin = hard_button_pin;
    mp_obj_t t[5] = {
        mp_obj_new_int(pin),
        mp_obj_new_int(pin >= 0 ? ob_gpio_read(pin) : -1),
        mp_obj_new_int(hard_button.raw_last),
        mp_obj_new_int(hard_button.win_count),
        mp_obj_new_int(hard_button.stable_pressed),
    };
    return mp_obj_new_tuple(5, t);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_hard_button_probe_obj, mp_hard_button_probe);

static mp_obj_t mp_hard_button_take_start(mp_obj_t self_in) {
    (void)self_in;
    if (hard_button_start_pending) {
        hard_button_start_pending = 0;
        return mp_const_true;
    }
    return mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_hard_button_take_start_obj, mp_hard_button_take_start);

static mp_obj_t mp_hard_button_stats(mp_obj_t self_in) {
    (void)self_in;
    mp_obj_t t[4] = {
        mp_obj_new_int_from_uint(hard_button.n_presses),
        mp_obj_new_int_from_uint(hard_button.n_releases),
        mp_obj_new_int_from_uint(hard_button_stops),
        mp_obj_new_bool(hard_button_armed),
    };
    return mp_obj_new_tuple(4, t);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_hard_button_stats_obj, mp_hard_button_stats);
#endif

// ---- hard-tick yaw integrator (imu_yaw_core) ----
//
// The raw-IMU heading source for the ICM-45686 arc. The (future)
// SPI driver feeds gyro-Z samples from the hard tick; until it
// exists, the feed binding is the test/diagnostic seam. Unguarded:
// the core is pure C and the unix suite drives it synthetically.
static ob_yaw_t hard_yaw;
static uint8_t  hard_yaw_inited;

static ob_yaw_t *hard_yaw_get(void) {
    if (!hard_yaw_inited) {
        ob_yaw_init(&hard_yaw, (ob_float_t)1.0);
        hard_yaw_inited = 1;
    }
    return &hard_yaw;
}

double openbricks_hard_yaw_deg(void) {
    return (double)ob_yaw_deg(hard_yaw_get());
}

void openbricks_hard_yaw_feed_c(double dt_ms, double rate_dps) {
    ob_yaw_feed(hard_yaw_get(), (ob_float_t)dt_ms, (ob_float_t)rate_dps);
}

void openbricks_hard_yaw_configure_c(double scale) {
    ob_yaw_init(hard_yaw_get(), (ob_float_t)scale);
}

void openbricks_hard_yaw_seed_bias_c(double bias_dps) {
    // NVS-persisted calibration from a previous boot (the pbio
    // trick): seed the estimator so a robot that boots and runs
    // immediately isn't stuck in the pre-lock window. The slow
    // tracker keeps refining from live stillness.
    ob_yaw_t *y = hard_yaw_get();
    y->bias_dps = (ob_float_t)bias_dps;
    y->bias_locked = 1;
}

static void (*volatile hard_imu_fn)(void);

void openbricks_hard_imu_install(void (*fn)(void)) {
    hard_imu_fn = fn;
}

static mp_obj_t mp_hard_yaw_config(mp_obj_t self_in, mp_obj_t scale_in) {
    // Set the rate multiplier (mounting sign + sensitivity trim) —
    // re-inits the integrator: calibration belongs to a mounting.
    (void)self_in;
    ob_yaw_init(hard_yaw_get(), (ob_float_t)mp_obj_get_float(scale_in));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_hard_yaw_config_obj, mp_hard_yaw_config);

static mp_obj_t mp_hard_yaw_feed(mp_obj_t self_in, mp_obj_t dt_ms_in,
                                 mp_obj_t rate_dps_in) {
    (void)self_in;
    ob_yaw_feed(hard_yaw_get(), (ob_float_t)mp_obj_get_float(dt_ms_in),
                (ob_float_t)mp_obj_get_float(rate_dps_in));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(mp_hard_yaw_feed_obj, mp_hard_yaw_feed);

static mp_obj_t mp_hard_yaw_deg(mp_obj_t self_in) {
    (void)self_in;
    return mp_obj_new_float((mp_float_t)ob_yaw_deg(hard_yaw_get()));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_hard_yaw_deg_obj, mp_hard_yaw_deg);

static mp_obj_t mp_hard_yaw_reset(mp_obj_t self_in) {
    (void)self_in;
    ob_yaw_reset(hard_yaw_get());
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_hard_yaw_reset_obj, mp_hard_yaw_reset);

static mp_obj_t mp_hard_yaw_seed_bias(mp_obj_t self_in, mp_obj_t bias_in) {
    (void)self_in;
    openbricks_hard_yaw_seed_bias_c(mp_obj_get_float(bias_in));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_hard_yaw_seed_bias_obj,
                                 mp_hard_yaw_seed_bias);

static mp_obj_t mp_hard_yaw_state(mp_obj_t self_in) {
    // (bias_dps, bias_locked, still_ms) — calibration diagnostics.
    (void)self_in;
    ob_yaw_t *y = hard_yaw_get();
    mp_obj_t t[3] = {
        mp_obj_new_float((mp_float_t)y->bias_dps),
        mp_obj_new_bool(y->bias_locked),
        mp_obj_new_float((mp_float_t)y->still_ms),
    };
    return mp_obj_new_tuple(3, t);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_hard_yaw_state_obj, mp_hard_yaw_state);

static mp_obj_t mp_now_ms(mp_obj_t self_in) {
    (void)self_in;
    (void)mp_get();   // lazy init
    return mp_obj_new_int(openbricks_motor_process_now_ms());
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_now_ms_obj, mp_now_ms);

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
    // Replace the callbacks list with a fresh empty one rather than
    // calling .clear() on the existing one. After a soft-reset, the
    // GC heap is wiped but root pointers are not always re-zeroed —
    // ``MP_STATE_PORT(openbricks_mp_callbacks)`` can hold a dangling
    // pointer that the allocator has since reused for an unrelated
    // object (we've seen bytearray). Calling ``mp_load_attr(...,
    // clear)`` on it then raises ``'bytearray' object has no attribute
    // 'clear'`` and the reset itself fails — which is exactly what
    // the launcher needs *not* to do, since it runs on every
    // ``openbricks run``. Reassigning is robust to whatever junk the
    // slot currently holds.
    MP_STATE_PORT(openbricks_mp_callbacks) = mp_obj_new_list(0, NULL);
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
    { MP_ROM_QSTR(MP_QSTR_now_ms),        MP_ROM_PTR(&mp_now_ms_obj) },
    { MP_ROM_QSTR(MP_QSTR_hard_yaw_config), MP_ROM_PTR(&mp_hard_yaw_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_hard_yaw_feed),  MP_ROM_PTR(&mp_hard_yaw_feed_obj) },
    { MP_ROM_QSTR(MP_QSTR_hard_yaw_deg),   MP_ROM_PTR(&mp_hard_yaw_deg_obj) },
    { MP_ROM_QSTR(MP_QSTR_hard_yaw_reset), MP_ROM_PTR(&mp_hard_yaw_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_hard_yaw_state), MP_ROM_PTR(&mp_hard_yaw_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_hard_yaw_seed_bias), MP_ROM_PTR(&mp_hard_yaw_seed_bias_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_wall_clock), MP_ROM_PTR(&mp_set_wall_clock_obj) },
    { MP_ROM_QSTR(MP_QSTR_wall_clock),    MP_ROM_PTR(&mp_wall_clock_obj) },
    { MP_ROM_QSTR(MP_QSTR_hard_tick_available), MP_ROM_PTR(&mp_hard_tick_available_obj) },
    #if defined(MICROPY_OPENBRICKS_HARD_TICK) && MICROPY_OPENBRICKS_HARD_TICK \
        && defined(MICROPY_OPENBRICKS_GPIO_SHIM) && MICROPY_OPENBRICKS_GPIO_SHIM
    { MP_ROM_QSTR(MP_QSTR_hard_button_config), MP_ROM_PTR(&mp_hard_button_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_hard_button_arm), MP_ROM_PTR(&mp_hard_button_arm_obj) },
    { MP_ROM_QSTR(MP_QSTR_hard_button_take_start), MP_ROM_PTR(&mp_hard_button_take_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_hard_button_stats), MP_ROM_PTR(&mp_hard_button_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_hard_button_probe), MP_ROM_PTR(&mp_hard_button_probe_obj) },
    #endif
    #if defined(MICROPY_OPENBRICKS_HARD_TICK) && MICROPY_OPENBRICKS_HARD_TICK
    { MP_ROM_QSTR(MP_QSTR_hard_tick_selftest), MP_ROM_PTR(&mp_hard_tick_selftest_obj) },
    { MP_ROM_QSTR(MP_QSTR_hard_tick_count), MP_ROM_PTR(&mp_hard_tick_count_obj) },
    #endif
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
