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
    if (b->chatter_ticks) {
        b->chatter_ticks--;
    }
    if (b->refractory_ticks) {
        b->refractory_ticks--;
    }
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
        if (b->refractory_ticks) {
            // Same physical press re-confirming through a violent
            // bounce train (measured: 30 edges for 10 presses) —
            // count it as chatter, never as a new press.
            b->n_stale++;
            return OB_BUTTON_NONE;
        }
        b->refractory_ticks = OB_BUTTON_REFRACTORY_TICKS;
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


void ob_button_arm_transition(ob_button_t *b) {
    b->stale_press = (b->stable_pressed || b->win_count != 0) ? 1 : 0;
}


void ob_button_clear_stale(ob_button_t *b) {
    b->stale_press = 0;
    // Disarm ends the run: the cooldown must not outlive it and eat
    // the NEXT run's (deliberate) start press — post-stop start
    // suppression is the Python watcher's lockout, not ours. The
    // refractory follows the same boundary rule: same-press
    // idempotence is a within-run concept, and a stop press's
    // bounce landing after the disarm is the lockout's to swallow.
    b->chatter_ticks = 0;
    b->refractory_ticks = 0;
}


int ob_button_event_is_stale(ob_button_t *b, ob_button_event_t e) {
    if (!b->stale_press) {
        if (e == OB_BUTTON_PRESSED && b->chatter_ticks) {
            // Re-contact bounce of the stale press's release: a
            // short start tap can re-touch for >= ON_THRESH ms and
            // confirm as a "fresh" press — inside the cooldown it is
            // the SAME press's chatter, never a stop (bench
            // 2026-08-14: the newborn run died 42 ms in).
            b->n_stale++;
            return 1;
        }
        return 0;
    }
    if (e == OB_BUTTON_PRESSED) {
        // The hysteresis machine cannot emit a second PRESSED edge
        // without a RELEASED in between, so consuming this one edge
        // retires the marker: whatever presses next is new input —
        // after the release-chatter cooldown.
        b->stale_press = 0;
        b->chatter_ticks = OB_BUTTON_CHATTER_TICKS;
        b->n_stale++;
        return 1;
    }
    if (e == OB_BUTTON_RELEASED
        || (!b->stable_pressed && b->win_count == 0)) {
        // The stale press ended (released, or its partial window
        // decayed without ever confirming) — hand over to the
        // release-chatter cooldown before fresh presses count.
        b->stale_press = 0;
        b->chatter_ticks = OB_BUTTON_CHATTER_TICKS;
    }
    return 0;
}
