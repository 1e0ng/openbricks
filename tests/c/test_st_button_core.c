// SPDX-License-Identifier: MIT
// Native tests for st_button_core — N-of-M majority debounce with
// hysteresis (the consecutive-stability rule missed 1-of-4 bench
// presses to contact chatter; majority voting absorbs it).

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
    // Clean press: fires at exactly ON_THRESH samples.
    CHECK_EQ_INT(tick_n(OB_BUTTON_ON_THRESH - 1), OB_BUTTON_NONE);
    CHECK_EQ_INT(tick_n(1), OB_BUTTON_PRESSED);
    CHECK_EQ_INT(tick_n(100), OB_BUTTON_NONE);      // held: no repeat
    level = 0;
    // Clean release: count must fall to OFF_THRESH.
    CHECK_EQ_INT(tick_n(OB_BUTTON_WINDOW), OB_BUTTON_RELEASED);
    CHECK_EQ_INT(b.n_presses, 1);
    CHECK_EQ_INT(b.n_releases, 1);
}

TEST(chattery_press_still_fires) {
    // THE bench failure mode: a real press whose line glitches high
    // for a sample every few ms. Consecutive-stability debounce
    // never fires (each glitch restarts the count); majority voting
    // must. Pattern: 4 pressed, 1 glitch, repeating — 80% duty.
    ob_button_init(&b, rd, NULL);
    level = 0;
    tick_n(50);
    ob_button_event_t got = OB_BUTTON_NONE;
    for (int i = 0; i < 60; i++) {
        level = (i % 5 == 4) ? 0 : 1;
        ob_button_event_t e = ob_button_tick(&b);
        if (e != OB_BUTTON_NONE) {
            got = e;
        }
    }
    CHECK_EQ_INT(got, OB_BUTTON_PRESSED);
    CHECK_EQ_INT(b.n_presses, 1);           // once, not per burst
}

TEST(fifty_percent_chatter_is_rejected) {
    // Alternating samples never reach the 15-of-20 majority — noise
    // without a dominant level is not a press.
    ob_button_init(&b, rd, NULL);
    level = 0;
    tick_n(50);
    for (int i = 0; i < 200; i++) {
        level = i & 1;
        CHECK_EQ_INT(ob_button_tick(&b), OB_BUTTON_NONE);
    }
    CHECK_EQ_INT(b.n_presses, 0);
}

TEST(short_tap_below_majority_is_rejected) {
    // A blip shorter than ON_THRESH pressed samples in any window
    // never fires.
    ob_button_init(&b, rd, NULL);
    level = 0;
    tick_n(50);
    for (int k = 0; k < 8; k++) {
        level = 1;
        CHECK_EQ_INT(tick_n(OB_BUTTON_ON_THRESH - 5), OB_BUTTON_NONE);
        level = 0;
        CHECK_EQ_INT(tick_n(OB_BUTTON_WINDOW), OB_BUTTON_NONE);
    }
    CHECK_EQ_INT(b.n_presses, 0);
}

TEST(mid_hold_flicker_does_not_release) {
    // Hysteresis: while held, chatter must drag the window count all
    // the way to OFF_THRESH to release — brief bounces cannot.
    ob_button_init(&b, rd, NULL);
    level = 1;
    tick_n(OB_BUTTON_WINDOW + 5);
    CHECK_EQ_INT(b.stable_pressed, 1);
    level = 0;
    tick_n(6);                              // brief bounce up (6 ms)
    level = 1;
    CHECK_EQ_INT(tick_n(100), OB_BUTTON_NONE);      // still held
    CHECK_EQ_INT(b.n_releases, 0);
}

TEST(window_count_stays_consistent_over_long_runs) {
    // The sliding count must not drift from the true popcount (the
    // += raw - out arithmetic), including through many transitions.
    ob_button_init(&b, rd, NULL);
    for (int i = 0; i < 100000; i++) {
        level = (i / 37) & 1;
        ob_button_tick(&b);
    }
    // Recompute popcount from the window directly.
    int pc = 0;
    for (int i = 0; i < OB_BUTTON_WINDOW; i++) {
        pc += (b.window >> i) & 1;
    }
    CHECK_EQ_INT(b.win_count, pc);
    // And the machine still works after the marathon.
    level = 1;
    CHECK(tick_n(OB_BUTTON_WINDOW) == OB_BUTTON_PRESSED
          || b.stable_pressed == 1);
}

TEST(clean_press_latency_is_on_thresh_ticks) {
    // The bounded-latency promise: a clean press is detected in
    // ON_THRESH ms at 1 kHz (15 ms — faster than the old 20).
    ob_button_init(&b, rd, NULL);
    level = 0;
    tick_n(50);
    level = 1;
    int ticks = 0;
    while (ob_button_tick(&b) != OB_BUTTON_PRESSED) {
        ticks++;
        if (ticks > 100) {
            break;
        }
    }
    CHECK_EQ_INT(ticks, OB_BUTTON_ON_THRESH - 1);
}

int main(void) {
    RUN(clean_press_and_release_fire_one_edge_each);
    RUN(chattery_press_still_fires);
    RUN(fifty_percent_chatter_is_rejected);
    RUN(short_tap_below_majority_is_rejected);
    RUN(mid_hold_flicker_does_not_release);
    RUN(window_count_stays_consistent_over_long_runs);
    RUN(clean_press_latency_is_on_thresh_ticks);
    return harness_exit("st_button_core");
}
