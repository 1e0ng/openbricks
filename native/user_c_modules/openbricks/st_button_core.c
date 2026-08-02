// SPDX-License-Identifier: MIT
// st_button_core — implementation. See header for the contract.

#include "st_button_core.h"

#include <string.h>


void ob_button_init(ob_button_t *b,
                    int (*read_pressed)(void *ctx), void *ctx) {
    memset(b, 0, sizeof(*b));
    b->read_pressed = read_pressed;
    b->ctx = ctx;
}


ob_button_event_t ob_button_tick(ob_button_t *b) {
    uint8_t raw = b->read_pressed(b->ctx) ? 1 : 0;
    if (raw == b->raw_last) {
        if (b->raw_count < 0xFFFF) {
            b->raw_count++;
        }
    } else {
        b->raw_last = raw;
        b->raw_count = 1;
    }
    if (b->raw_count < OB_BUTTON_DEBOUNCE_TICKS
        || raw == b->stable_pressed) {
        return OB_BUTTON_NONE;
    }
    b->stable_pressed = raw;
    if (raw) {
        b->n_presses++;
        return OB_BUTTON_PRESSED;
    }
    b->n_releases++;
    return OB_BUTTON_RELEASED;
}
