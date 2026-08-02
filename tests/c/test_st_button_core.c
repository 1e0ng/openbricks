// SPDX-License-Identifier: MIT
// Native tests for st_button_core — 1 kHz debounce + edge machine.

#include "harness.h"
#include "st_button_core.h"

static int level;
static int rd(void *ctx) {
    (void)ctx;
    return level;
}

static ob_button_t b;

static ob_button_event_t tick_n(int n) {
    ob_button_event_t last = OB_BUTTON_NONE;
    for (int i = 0; i < n; i++) {
        ob_button_event_t e = ob_button_tick(&b);
        if (e != OB_BUTTON_NONE) {
            last = e;
        }
    }
    return last;
}

TEST(clean_press_and_release_fire_one_edge_each) {
    ob_button_init(&b, rd, NULL);
    level = 0;
    CHECK_EQ_INT(tick_n(50), OB_BUTTON_NONE);
    level = 1;
    CHECK_EQ_INT(tick_n(OB_BUTTON_DEBOUNCE_TICKS - 1), OB_BUTTON_NONE);
    CHECK_EQ_INT(tick_n(1), OB_BUTTON_PRESSED);
    CHECK_EQ_INT(tick_n(100), OB_BUTTON_NONE);      // held: no repeat
    level = 0;
    CHECK_EQ_INT(tick_n(OB_BUTTON_DEBOUNCE_TICKS), OB_BUTTON_RELEASED);
    CHECK_EQ_INT(b.n_presses, 1);
    CHECK_EQ_INT(b.n_releases, 1);
}

TEST(chatter_shorter_than_debounce_is_rejected) {
    ob_button_init(&b, rd, NULL);
    level = 0;
    tick_n(50);
    // 10 ms flickers (below the 20-tick threshold), repeatedly.
    for (int k = 0; k < 8; k++) {
        level = 1;
        CHECK_EQ_INT(tick_n(OB_BUTTON_DEBOUNCE_TICKS / 2), OB_BUTTON_NONE);
        level = 0;
        CHECK_EQ_INT(tick_n(OB_BUTTON_DEBOUNCE_TICKS / 2), OB_BUTTON_NONE);
    }
    CHECK_EQ_INT(b.n_presses, 0);
}

TEST(mid_hold_flicker_does_not_release) {
    ob_button_init(&b, rd, NULL);
    level = 1;
    tick_n(OB_BUTTON_DEBOUNCE_TICKS + 5);
    CHECK_EQ_INT(b.stable_pressed, 1);
    level = 0;
    tick_n(OB_BUTTON_DEBOUNCE_TICKS / 3);          // brief bounce up
    level = 1;
    CHECK_EQ_INT(tick_n(100), OB_BUTTON_NONE);      // still held
    CHECK_EQ_INT(b.n_releases, 0);
}

TEST(raw_counter_saturates_without_wrap) {
    ob_button_init(&b, rd, NULL);
    level = 0;
    for (int i = 0; i < 80000; i++) {              // > 0xFFFF ticks
        ob_button_tick(&b);
    }
    CHECK_EQ_INT(b.raw_count, 0xFFFF);
    level = 1;
    CHECK_EQ_INT(tick_n(OB_BUTTON_DEBOUNCE_TICKS), OB_BUTTON_PRESSED);
}

int main(void) {
    RUN(clean_press_and_release_fire_one_edge_each);
    RUN(chatter_shorter_than_debounce_is_rejected);
    RUN(mid_hold_flicker_does_not_release);
    RUN(raw_counter_saturates_without_wrap);
    return harness_exit("st_button_core");
}
