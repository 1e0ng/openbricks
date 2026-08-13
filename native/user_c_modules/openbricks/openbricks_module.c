// SPDX-License-Identifier: MIT
//
// Module registration for the ``_openbricks_native`` MicroPython module.
// Pulls together the types defined in sibling files (motor_process.c,
// servo.c, …) into a single user_c_module.
//
// The Python re-export layer at ``openbricks/_native.py`` imports from
// this module and presents a tidier public API to user code.

#include "py/runtime.h"

#include "motor_process.h"

// Types defined in sibling files.
extern const mp_obj_type_t openbricks_motor_process_type;
extern const mp_obj_base_t st_bus_singleton;
extern const mp_obj_base_t icm45686_singleton;
extern const mp_obj_type_t openbricks_servo_type;
extern const mp_obj_type_t openbricks_trajectory_type;
extern const mp_obj_type_t openbricks_observer_type;
extern const mp_obj_type_t openbricks_drivebase_type;
extern const mp_obj_type_t openbricks_pcnt_encoder_type;
extern const mp_obj_type_t openbricks_encoder_type;
extern const mp_obj_type_t openbricks_bno055_type;
extern const mp_obj_fun_builtin_fixed_t openbricks_exec_mpy_obj;

// Inject a KeyboardInterrupt into the MAIN thread — the same mechanism
// the REPL uses for a host Ctrl-C. This sets the VM's pending exception,
// which is checked between *every* bytecode regardless of scheduler
// state, so it interrupts whatever is running in the main thread.
//
// Why this exists: the launcher's program-button STOP path used to do
// ``micropython.schedule(func)`` where ``func`` raised KeyboardInterrupt.
// Bench-confirmed that does NOT stop the program — a scheduled callback
// that raises only unwinds the callback itself (printing a traceback);
// the running program keeps going. And the scheduled-callback queue is
// gated by a lock while another scheduled callback runs, so it can't
// even fire while a button-started program executes inside
// ``_scheduled_start``. The pending-exception path has neither problem.
static mp_obj_t openbricks_request_keyboard_interrupt(void) {
    mp_sched_keyboard_interrupt();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(openbricks_request_keyboard_interrupt_obj,
                                 openbricks_request_keyboard_interrupt);

// ---- hardware stop button -------------------------------------------------
//
// Stopping a *running user program* requires injecting the interrupt the
// way a host Ctrl-C does — ``mp_sched_keyboard_interrupt()`` from a context
// with NO Python frame. A *Python* Timer/Pin.irq callback has its own
// Python frame, so the pending exception fires inside the callback and the
// program runs on (bench-confirmed). And this user C module cannot use
// ESP-IDF headers (see pcnt_encoder.c), which ruled out the GPIO-ISR /
// esp_timer / FreeRTOS-task attempts — they compiled as inert stubs.
//
// The portable answer is the pattern motor_process already uses: a
// C-FUNCTION registered as a ``machine.Timer`` callback. Per MicroPython's
// mp_handle_pending (checks the pending exception BEFORE running scheduler
// callbacks), a pending KeyboardInterrupt set inside a C-function callback
// — which runs no Python bytecodes afterward — is not raised within the
// callback; it's raised at the next bytecode of the MAIN program. So:
//
//   * the launcher's Python ``_tick`` detects the press (reading the pin
//     level already works — START uses it) and calls ``request_stop()``;
//   * the C ``stop_tick`` Timer callback sees the request and injects the
//     interrupt, which lands in the running program.
//
// ``set_stop_armed(bool)`` gates it to program runs. ``stop_button_debug``
// reports (tick_count, fire_count, armed, requested) so any failure is read
// off a counter. All portable — no IDF headers, no platform guard, so it
// actually compiles INTO the firmware (the previous versions did not).

static volatile bool ob_stop_armed = false;
static volatile bool ob_stop_requested = false;
static volatile uint32_t ob_stop_ticks = 0;   // stop_tick invocations
static volatile uint32_t ob_stop_fires = 0;   // interrupts injected

// C-function machine.Timer callback — runs in the scheduler with no Python
// frame, so the injected KeyboardInterrupt reaches the running program.
static mp_obj_t openbricks_stop_tick(mp_obj_t timer_arg) {
    (void)timer_arg;
    ob_stop_ticks++;
    if (ob_stop_armed && ob_stop_requested) {
        ob_stop_requested = false;
        ob_stop_fires++;
        mp_sched_keyboard_interrupt();
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(openbricks_stop_tick_obj, openbricks_stop_tick);

static mp_obj_t openbricks_request_stop(void) {
    ob_stop_requested = true;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(openbricks_request_stop_obj, openbricks_request_stop);

static mp_obj_t openbricks_set_stop_armed(mp_obj_t armed_in) {
    ob_stop_armed = mp_obj_is_true(armed_in);
    if (!ob_stop_armed) {
        ob_stop_requested = false;   // drop a stale request on disarm
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(openbricks_set_stop_armed_obj, openbricks_set_stop_armed);

static mp_obj_t openbricks_stop_button_debug(void) {
    mp_obj_t items[4] = {
        mp_obj_new_int((mp_int_t)ob_stop_ticks),
        mp_obj_new_int((mp_int_t)ob_stop_fires),
        mp_obj_new_int(ob_stop_armed ? 1 : 0),
        mp_obj_new_int(ob_stop_requested ? 1 : 0),
    };
    return mp_obj_new_tuple(4, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(openbricks_stop_button_debug_obj,
                                 openbricks_stop_button_debug);

static const mp_rom_map_elem_t openbricks_native_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),           MP_ROM_QSTR(MP_QSTR__openbricks_native) },
    { MP_ROM_QSTR(MP_QSTR_request_keyboard_interrupt),
      MP_ROM_PTR(&openbricks_request_keyboard_interrupt_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_tick),
      MP_ROM_PTR(&openbricks_stop_tick_obj) },
    { MP_ROM_QSTR(MP_QSTR_request_stop),
      MP_ROM_PTR(&openbricks_request_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_stop_armed),
      MP_ROM_PTR(&openbricks_set_stop_armed_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_button_debug),
      MP_ROM_PTR(&openbricks_stop_button_debug_obj) },
    { MP_ROM_QSTR(MP_QSTR_motor_process),      MP_ROM_PTR(&motor_process_singleton) },
    { MP_ROM_QSTR(MP_QSTR_st_bus),             MP_ROM_PTR(&st_bus_singleton) },
    { MP_ROM_QSTR(MP_QSTR_icm45686),           MP_ROM_PTR(&icm45686_singleton) },
    { MP_ROM_QSTR(MP_QSTR_Servo),              MP_ROM_PTR(&openbricks_servo_type) },
    { MP_ROM_QSTR(MP_QSTR_TrapezoidalProfile), MP_ROM_PTR(&openbricks_trajectory_type) },
    { MP_ROM_QSTR(MP_QSTR_Observer),           MP_ROM_PTR(&openbricks_observer_type) },
    { MP_ROM_QSTR(MP_QSTR_DriveBase),          MP_ROM_PTR(&openbricks_drivebase_type) },
    { MP_ROM_QSTR(MP_QSTR_PCNTEncoder),        MP_ROM_PTR(&openbricks_pcnt_encoder_type) },
    { MP_ROM_QSTR(MP_QSTR_QuadratureEncoder),  MP_ROM_PTR(&openbricks_encoder_type) },
    { MP_ROM_QSTR(MP_QSTR_BNO055),             MP_ROM_PTR(&openbricks_bno055_type) },
    { MP_ROM_QSTR(MP_QSTR_exec_mpy),           MP_ROM_PTR(&openbricks_exec_mpy_obj) },
};
static MP_DEFINE_CONST_DICT(openbricks_native_globals, openbricks_native_globals_table);

const mp_obj_module_t openbricks_native_cmodule = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&openbricks_native_globals,
};

MP_REGISTER_MODULE(MP_QSTR__openbricks_native, openbricks_native_cmodule);
