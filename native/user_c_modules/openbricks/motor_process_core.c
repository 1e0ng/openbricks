// SPDX-License-Identifier: MIT
//
// Shared C-side scheduler core. See ``motor_process_core.h`` for the
// scope split (this module owns the C-callback list + virtual clock;
// Python-callback dispatch and Timer glue live in the binding shells).

#include "motor_process_core.h"


void ob_motor_process_init(ob_motor_process_t *m) {
    m->n_c_callbacks  = 0;
    m->period_ms      = 1;
    m->virtual_now_ms = 0;
}


void ob_motor_process_reset(ob_motor_process_t *m) {
    ob_motor_process_init(m);
}


int ob_motor_process_register_c(ob_motor_process_t *m,
                                ob_tick_fn_t fn, void *ctx) {
    for (size_t i = 0; i < m->n_c_callbacks; i++) {
        if (m->c_callbacks[i].fn == fn && m->c_callbacks[i].ctx == ctx) {
            return 0;  // already registered — idempotent
        }
    }
    if (m->n_c_callbacks >= OB_MAX_C_CALLBACKS) {
        return -1;
    }
    m->c_callbacks[m->n_c_callbacks].fn  = fn;
    m->c_callbacks[m->n_c_callbacks].ctx = ctx;
    m->n_c_callbacks++;
    return 0;
}


void ob_motor_process_unregister_c(ob_motor_process_t *m,
                                   ob_tick_fn_t fn, void *ctx) {
    for (size_t i = 0; i < m->n_c_callbacks; i++) {
        if (m->c_callbacks[i].fn == fn && m->c_callbacks[i].ctx == ctx) {
            for (size_t j = i; j + 1 < m->n_c_callbacks; j++) {
                m->c_callbacks[j] = m->c_callbacks[j + 1];
            }
            m->n_c_callbacks--;
            return;
        }
    }
}


void ob_motor_process_fire_c(ob_motor_process_t *m) {
    // Advance the clock first so subscribers see a consistent "now".
    m->virtual_now_ms += m->period_ms;
    for (size_t i = 0; i < m->n_c_callbacks; i++) {
        m->c_callbacks[i].fn(m->c_callbacks[i].ctx);
    }
}


void ob_motor_process_set_period_ms(ob_motor_process_t *m, int period_ms) {
    m->period_ms = period_ms;
}


size_t ob_motor_process_count_c(const ob_motor_process_t *m) {
    return m->n_c_callbacks;
}
