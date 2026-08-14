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

// Helper mirroring the binding's tick body: classify one tick's
// event through the stale filter; returns 1 when a (non-stale)
// PRESSED edge would reach the armed/unarmed dispatch.
static int tick_classifies_press(void) {
    ob_button_event_t e = ob_button_tick(&b);
    if (ob_button_event_is_stale(&b, e)) {
        return 0;
    }
    return e == OB_BUTTON_PRESSED;
}

// N armed ticks through the SAME path the binding runs (tick +
// stale filter every tick); returns how many stop-classified
// presses came out.
static int classify_n(int n) {
    int stops = 0;
    for (int i = 0; i < n; i++) {
        stops += tick_classifies_press();
    }
    return stops;
}

TEST(start_press_confirming_after_arm_is_not_a_stop) {
    // Bench 2026-08-09, run_4: flash I/O at run start stalled the
    // hard tick 140 ms, so the START press's debounced edge landed
    // on the first ticks AFTER the stop armed — and killed the run
    // 1 ms in. Timeline: press down, a few samples accumulate, the
    // arm happens mid-debounce, THEN the edge confirms.
    ob_button_init(&b, rd, NULL);
    level = 0;
    tick_n(50);
    level = 1;
    tick_n(5);                        // mid-debounce, no edge yet
    ob_button_arm_transition(&b);     // exec setup arms the stop
    CHECK_EQ_INT(classify_n(50), 0);  // the edge confirmed, but stale
    CHECK_EQ_INT(b.n_presses, 1);     // it DID fire at the debouncer
    CHECK_EQ_INT(b.n_stale, 1);
    // Release (the chatter cooldown runs), then a genuinely new
    // press after it: THAT one stops.
    level = 0;
    classify_n(OB_BUTTON_WINDOW + OB_BUTTON_CHATTER_TICKS);
    level = 1;
    CHECK_EQ_INT(classify_n(50), 1);
}

TEST(press_already_stable_at_arm_needs_release_before_stop) {
    // Slow variant: the press confirmed BEFORE the arm (normal
    // start path, the edge latched a start). Held through arming,
    // it must produce no stop; its release then re-enables stops.
    ob_button_init(&b, rd, NULL);
    level = 0;
    tick_n(50);
    level = 1;
    tick_n(50);                       // pressed edge fired, still held
    CHECK_EQ_INT(b.stable_pressed, 1);
    ob_button_arm_transition(&b);
    CHECK_EQ_INT(classify_n(100), 0); // held a while longer
    level = 0;
    classify_n(OB_BUTTON_WINDOW + 5); // release clears staleness...
    // ...into the release-chatter cooldown: a press inside it is the
    // start press's own re-contact, never a stop.
    level = 1;
    CHECK_EQ_INT(classify_n(50), 0);
    level = 0;
    classify_n(OB_BUTTON_WINDOW + OB_BUTTON_CHATTER_TICKS);
    level = 1;
    CHECK_EQ_INT(classify_n(50), 1);  // cooldown over: stops work
}

TEST(stale_window_decay_re_enables_stops) {
    // The press released before arming but its samples were still
    // draining from the window: the marker must retire on decay so
    // a real stop press shortly after is NOT swallowed.
    ob_button_init(&b, rd, NULL);
    level = 0;
    tick_n(50);
    level = 1;
    tick_n(5);                        // brief contact, never confirms
    level = 0;
    ob_button_arm_transition(&b);     // window still holds 5 samples
    classify_n(OB_BUTTON_WINDOW + 1); // decays to empty -> marker off
    // The decayed press's release chatter is covered by the same
    // cooldown; a REAL stop press works once it expires.
    classify_n(OB_BUTTON_CHATTER_TICKS);
    level = 1;
    CHECK_EQ_INT(classify_n(50), 1);
    CHECK_EQ_INT(b.n_stale, 0);
}

TEST(arm_with_idle_button_suppresses_nothing) {
    ob_button_init(&b, rd, NULL);
    level = 0;
    tick_n(50);
    ob_button_arm_transition(&b);     // nothing in flight
    CHECK_EQ_INT(b.stale_press, 0);
    level = 1;
    CHECK_EQ_INT(classify_n(50), 1);  // a real stop press works
}

TEST(clear_stale_on_disarm) {
    ob_button_init(&b, rd, NULL);
    level = 1;
    tick_n(5);
    ob_button_arm_transition(&b);
    CHECK_EQ_INT(b.stale_press, 1);
    ob_button_clear_stale(&b);
    CHECK_EQ_INT(b.stale_press, 0);
}

TEST(release_recontact_after_arm_is_not_a_stop) {
    // Bench 2026-08-14: a short start tap's release re-contacted for
    // >= ON_THRESH ms; with the stale marker already retired, the
    // re-contact confirmed as a "fresh" press and hard-stopped the
    // newborn run 42 ms in (no watcher note — the C path fired).
    // Inside the release-chatter cooldown that press is the SAME
    // press's bounce.
    ob_button_init(&b, rd, NULL);
    level = 0;
    tick_n(50);
    level = 1;
    tick_n(50);                       // start press confirmed, held
    ob_button_arm_transition(&b);     // run starts, stop armed
    level = 0;
    classify_n(OB_BUTTON_WINDOW + 5); // release: stale -> cooldown
    level = 1;                        // re-contact bounce...
    CHECK_EQ_INT(classify_n(30), 0);  // ...consumed, run survives
    CHECK_EQ_INT(b.n_stale >= 1, 1);
    level = 0;
    classify_n(OB_BUTTON_WINDOW + OB_BUTTON_CHATTER_TICKS);
    level = 1;
    CHECK_EQ_INT(classify_n(50), 1);  // deliberate stop still works
}

TEST(disarm_clears_the_chatter_cooldown) {
    // The cooldown must not outlive the run and eat the NEXT run's
    // start press (post-stop start suppression is the Python
    // watcher's lockout).
    ob_button_init(&b, rd, NULL);
    level = 0;
    tick_n(50);
    level = 1;
    tick_n(50);
    ob_button_arm_transition(&b);
    level = 0;
    classify_n(OB_BUTTON_WINDOW + 5); // cooldown armed
    ob_button_clear_stale(&b);        // program ends: disarm
    CHECK_EQ_INT(b.chatter_ticks, 0);
    level = 1;
    CHECK_EQ_INT(classify_n(50), 1);  // next press is fresh input
}

int main(void) {
    RUN(clean_press_and_release_fire_one_edge_each);
    RUN(chattery_press_still_fires);
    RUN(fifty_percent_chatter_is_rejected);
    RUN(short_tap_below_majority_is_rejected);
    RUN(mid_hold_flicker_does_not_release);
    RUN(window_count_stays_consistent_over_long_runs);
    RUN(clean_press_latency_is_on_thresh_ticks);
    RUN(start_press_confirming_after_arm_is_not_a_stop);
    RUN(press_already_stable_at_arm_needs_release_before_stop);
    RUN(stale_window_decay_re_enables_stops);
    RUN(arm_with_idle_button_suppresses_nothing);
    RUN(clear_stale_on_disarm);
    RUN(release_recontact_after_arm_is_not_a_stop);
    RUN(disarm_clears_the_chatter_cooldown);
    return harness_exit("st_button_core");
}
