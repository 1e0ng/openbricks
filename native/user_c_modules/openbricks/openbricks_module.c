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
extern const mp_obj_type_t openbricks_servo_type;
extern const mp_obj_type_t openbricks_trajectory_type;
extern const mp_obj_type_t openbricks_observer_type;
extern const mp_obj_type_t openbricks_drivebase_type;
extern const mp_obj_type_t openbricks_pcnt_encoder_type;
extern const mp_obj_type_t openbricks_encoder_type;
extern const mp_obj_type_t openbricks_bno055_type;

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
// The program-stop button must interrupt a *running user program*. Three
// facts (all bench-confirmed on the target) pin the design:
//
//   1. The interrupt only reaches the program when injected the way a host
//      Ctrl-C is — ``mp_sched_keyboard_interrupt()`` from a context with NO
//      Python frame. A Python Timer / Pin.irq callback runs in a soft
//      Python frame, so the KeyboardInterrupt unwinds the callback, not the
//      program. (esp32 Pin.irq has no hard= option, and an esp_timer
//      callback didn't fire on this hub either.)
//   2. The button *level* is reliably readable (the launcher's poll-based
//      START works) even though edge interrupts don't.
//   3. Ctrl-C — delivered from the BLE FreeRTOS *task* calling exactly this
//      function — DOES stop a running program here.
//
// So we run our own FreeRTOS task that polls the level and, on a falling
// edge while armed, injects the interrupt. A plain task's loop is
// guaranteed to run (it doesn't depend on the esp_timer service), and it's
// the same task→mp_sched_keyboard_interrupt path Ctrl-C already uses.
// ``install_stop_button(gpio)`` starts it; ``set_stop_armed(bool)`` gates
// it so an idle press doesn't tear down the boot loop. ``stop_button_debug``
// reports (task_started, loop_ticks, edges_fired, armed, gpio) so a failure
// is read off a counter, not guessed at.

#if defined(ESP_PLATFORM)

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mphalport.h"   // mp_hal_wake_main_task

static volatile bool ob_stop_armed = false;
static int ob_stop_gpio = -1;
static volatile uint32_t ob_stop_ticks = 0;   // loop iterations (is it running?)
static volatile uint32_t ob_stop_edges = 0;   // falling edges injected while armed
static TaskHandle_t ob_stop_task = NULL;

static void ob_stop_task_fn(void *arg) {
    (void)arg;
    int last = 1;
    if (ob_stop_gpio >= 0) {
        last = gpio_get_level((gpio_num_t)ob_stop_gpio);
    }
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(15));
        ob_stop_ticks++;
        if (ob_stop_gpio < 0) {
            continue;
        }
        int level = gpio_get_level((gpio_num_t)ob_stop_gpio);
        // Falling edge (press, active-low) while a program is running.
        if (ob_stop_armed && last != 0 && level == 0) {
            ob_stop_edges++;
            mp_sched_keyboard_interrupt();
            mp_hal_wake_main_task();
        }
        last = level;
    }
}

static mp_obj_t openbricks_install_stop_button(mp_obj_t gpio_in) {
    ob_stop_gpio = mp_obj_get_int(gpio_in);
    if (ob_stop_task == NULL) {
        // Priority above the MP main task so the poll runs even during a
        // tight loop; pinned to core 0 where the MP task lives.
        xTaskCreatePinnedToCore(ob_stop_task_fn, "ob_stop", 4096, NULL,
                                5, &ob_stop_task, 0);
    }
    return mp_const_none;
}

static mp_obj_t openbricks_set_stop_armed(mp_obj_t armed_in) {
    ob_stop_armed = mp_obj_is_true(armed_in);
    return mp_const_none;
}

static mp_obj_t openbricks_stop_button_debug(void) {
    mp_obj_t items[5] = {
        mp_obj_new_int(ob_stop_task != NULL ? 1 : 0),
        mp_obj_new_int((mp_int_t)ob_stop_ticks),
        mp_obj_new_int((mp_int_t)ob_stop_edges),
        mp_obj_new_int(ob_stop_armed ? 1 : 0),
        mp_obj_new_int(ob_stop_gpio),
    };
    return mp_obj_new_tuple(5, items);
}

#else  // non-esp32 (unix test build): no GPIO — inert stubs so the
       // launcher's calls are harmless and the module still links.

static bool ob_stop_armed_stub = false;

static mp_obj_t openbricks_install_stop_button(mp_obj_t gpio_in) {
    (void)gpio_in;
    return mp_const_none;
}

static mp_obj_t openbricks_set_stop_armed(mp_obj_t armed_in) {
    ob_stop_armed_stub = mp_obj_is_true(armed_in);
    return mp_const_none;
}

static mp_obj_t openbricks_stop_button_debug(void) {
    mp_obj_t items[5] = {
        mp_obj_new_int(0), mp_obj_new_int(0), mp_obj_new_int(0),
        mp_obj_new_int(ob_stop_armed_stub ? 1 : 0), mp_obj_new_int(-1),
    };
    return mp_obj_new_tuple(5, items);
}

#endif

static MP_DEFINE_CONST_FUN_OBJ_1(openbricks_install_stop_button_obj,
                                 openbricks_install_stop_button);
static MP_DEFINE_CONST_FUN_OBJ_1(openbricks_set_stop_armed_obj,
                                 openbricks_set_stop_armed);
static MP_DEFINE_CONST_FUN_OBJ_0(openbricks_stop_button_debug_obj,
                                 openbricks_stop_button_debug);

static const mp_rom_map_elem_t openbricks_native_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),           MP_ROM_QSTR(MP_QSTR__openbricks_native) },
    { MP_ROM_QSTR(MP_QSTR_request_keyboard_interrupt),
      MP_ROM_PTR(&openbricks_request_keyboard_interrupt_obj) },
    { MP_ROM_QSTR(MP_QSTR_install_stop_button),
      MP_ROM_PTR(&openbricks_install_stop_button_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_button_debug),
      MP_ROM_PTR(&openbricks_stop_button_debug_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_stop_armed),
      MP_ROM_PTR(&openbricks_set_stop_armed_obj) },
    { MP_ROM_QSTR(MP_QSTR_motor_process),      MP_ROM_PTR(&motor_process_singleton) },
    { MP_ROM_QSTR(MP_QSTR_Servo),              MP_ROM_PTR(&openbricks_servo_type) },
    { MP_ROM_QSTR(MP_QSTR_TrapezoidalProfile), MP_ROM_PTR(&openbricks_trajectory_type) },
    { MP_ROM_QSTR(MP_QSTR_Observer),           MP_ROM_PTR(&openbricks_observer_type) },
    { MP_ROM_QSTR(MP_QSTR_DriveBase),          MP_ROM_PTR(&openbricks_drivebase_type) },
    { MP_ROM_QSTR(MP_QSTR_PCNTEncoder),        MP_ROM_PTR(&openbricks_pcnt_encoder_type) },
    { MP_ROM_QSTR(MP_QSTR_QuadratureEncoder),  MP_ROM_PTR(&openbricks_encoder_type) },
    { MP_ROM_QSTR(MP_QSTR_BNO055),             MP_ROM_PTR(&openbricks_bno055_type) },
};
static MP_DEFINE_CONST_DICT(openbricks_native_globals, openbricks_native_globals_table);

const mp_obj_module_t openbricks_native_cmodule = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&openbricks_native_globals,
};

MP_REGISTER_MODULE(MP_QSTR__openbricks_native, openbricks_native_cmodule);
