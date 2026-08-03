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
    b->raw_last = raw;
    // Sliding window: shift in the new sample, retire the oldest.
    // Before WINDOW samples have been seen, the retiring bit is
    // necessarily 0, so no fill counter is needed.
    uint32_t out = (b->window >> (OB_BUTTON_WINDOW - 1)) & 1u;
    b->window = ((b->window << 1) | raw)
                & ((1u << OB_BUTTON_WINDOW) - 1u);
    b->win_count = (uint8_t)(b->win_count + raw - out);

    if (!b->stable_pressed && b->win_count >= OB_BUTTON_ON_THRESH) {
        b->stable_pressed = 1;
        b->n_presses++;
        return OB_BUTTON_PRESSED;
    }
    if (b->stable_pressed && b->win_count <= OB_BUTTON_OFF_THRESH) {
        b->stable_pressed = 0;
        b->n_releases++;
        return OB_BUTTON_RELEASED;
    }
    return OB_BUTTON_NONE;
}
