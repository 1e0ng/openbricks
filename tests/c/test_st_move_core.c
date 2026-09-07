// SPDX-License-Identifier: MIT
// Native tests for st_move_core — per-slot position moves
// (trajectory + P + arrival done + hold).

#include <math.h>

#include "harness.h"
#include "st_move_core.h"

// Perfect-plant helper: integrate the commanded speed at 1 kHz.
// Returns the final position after ``ms`` ticks.
static ob_float_t run_plant(ob_smove_t *m, long start_ms, int ms,
                            ob_float_t pos) {
    for (int t = 0; t < ms; t++) {
        ob_float_t cmd = ob_smove_tick(m, start_ms + t, pos);
        pos += cmd * (ob_float_t)0.001;
    }
    return pos;
}

TEST(idle_outputs_zero_and_not_done) {
    ob_smove_t m;
    ob_smove_init(&m);
    CHECK(ob_smove_tick(&m, 0, 1234.0) == 0.0);
    CHECK(!ob_smove_is_done(&m));
}

TEST(move_converges_on_relative_goal) {
    ob_smove_t m;
    ob_smove_init(&m);
    // 2048 counts (half a rev) at 1000 counts/s, 4000 counts/s^2.
    ob_smove_start(&m, 0, 500.0, 2048.0, 1000.0, 4000.0);
    ob_float_t pos = run_plant(&m, 0, 4000, 500.0);
    CHECK(fabs(pos - 2548.0) < OB_SMOVE_DONE_TOL_COUNTS);
    CHECK(ob_smove_is_done(&m));
}

TEST(negative_move_converges) {
    ob_smove_t m;
    ob_smove_init(&m);
    ob_smove_start(&m, 0, 0.0, -4096.0, 800.0, 3000.0);
    ob_float_t pos = run_plant(&m, 0, 8000, 0.0);
    CHECK(fabs(pos - (-4096.0)) < OB_SMOVE_DONE_TOL_COUNTS);
    CHECK(ob_smove_is_done(&m));
}

TEST(launch_is_ramped_not_a_step) {
    ob_smove_t m;
    ob_smove_init(&m);
    ob_smove_start(&m, 0, 0.0, 4096.0, 2000.0, 1000.0);
    // First tick: trapezoid commands ~accel*dt of feedforward, plus
    // kp * (tiny target offset) — nowhere near cruise.
    ob_float_t cmd0 = ob_smove_tick(&m, 0, 0.0);
    CHECK(cmd0 >= 0.0);
    CHECK(cmd0 < 2000.0 * 0.2);
}

TEST(peak_speed_respects_cruise) {
    ob_smove_t m;
    ob_smove_init(&m);
    ob_smove_start(&m, 0, 0.0, 8192.0, 1500.0, 6000.0);
    ob_float_t pos = 0.0, peak = 0.0;
    for (int t = 0; t < 7000; t++) {
        ob_float_t cmd = ob_smove_tick(&m, t, pos);
        ob_float_t mag = (cmd < 0) ? -cmd : cmd;
        if (mag > peak) {
            peak = mag;
        }
        pos += cmd * (ob_float_t)0.001;
    }
    // FF is bounded by cruise; the P term adds only the tracking
    // error's worth on a healthy plant.
    CHECK(peak < 1500.0 * 1.2);
    CHECK(ob_smove_is_done(&m));
}

TEST(done_requires_arrival_not_just_time) {
    ob_smove_t m;
    ob_smove_init(&m);
    ob_smove_start(&m, 0, 0.0, 2048.0, 1000.0, 4000.0);
    // Stalled plant: position frozen at 0. Run far past t_total.
    for (int t = 0; t < 10000; t++) {
        (void)ob_smove_tick(&m, t, 0.0);
    }
    CHECK(!ob_smove_is_done(&m));    // never arrived -> never done
    // Now let it arrive: command reaches the goal, done latches.
    (void)ob_smove_tick(&m, 10001, 2048.0);
    CHECK(ob_smove_is_done(&m));
}

TEST(done_latches_through_disturbance_but_hold_corrects) {
    ob_smove_t m;
    ob_smove_init(&m);
    ob_smove_start(&m, 0, 0.0, 1000.0, 1000.0, 4000.0);
    ob_float_t pos = run_plant(&m, 0, 3000, 0.0);
    CHECK(ob_smove_is_done(&m));
    // Shove the shaft 200 counts off the goal.
    pos += 200.0;
    ob_float_t cmd = ob_smove_tick(&m, 3001, pos);
    CHECK(ob_smove_is_done(&m));     // still done (latched)
    CHECK(cmd < -100.0);             // but the hold pushes back
}

TEST(hold_at_is_done_and_corrects_both_directions) {
    ob_smove_t m;
    ob_smove_init(&m);
    ob_smove_hold_at(&m, 1000.0);
    CHECK(ob_smove_is_done(&m));
    CHECK(ob_smove_tick(&m, 0, 900.0) > 0.0);
    CHECK(ob_smove_tick(&m, 1, 1100.0) < 0.0);
    CHECK(ob_smove_tick(&m, 2, 1000.0) == 0.0);
}

TEST(stop_returns_to_idle_silence) {
    ob_smove_t m;
    ob_smove_init(&m);
    ob_smove_start(&m, 0, 0.0, 2048.0, 1000.0, 4000.0);
    (void)ob_smove_tick(&m, 100, 10.0);
    ob_smove_stop(&m);
    CHECK(ob_smove_tick(&m, 101, 10.0) == 0.0);
    CHECK(!ob_smove_is_done(&m));
}

TEST(new_start_supersedes_hold_and_clears_done) {
    ob_smove_t m;
    ob_smove_init(&m);
    ob_smove_hold_at(&m, 500.0);
    CHECK(ob_smove_is_done(&m));
    ob_smove_start(&m, 0, 500.0, 1024.0, 1000.0, 4000.0);
    CHECK(!ob_smove_is_done(&m));
    ob_float_t pos = run_plant(&m, 0, 3000, 500.0);
    CHECK(fabs(pos - 1524.0) < OB_SMOVE_DONE_TOL_COUNTS);
}

TEST(zero_delta_move_is_immediately_done_hold) {
    ob_smove_t m;
    ob_smove_init(&m);
    ob_smove_start(&m, 0, 700.0, 0.0, 1000.0, 4000.0);
    (void)ob_smove_tick(&m, 1, 700.0);
    CHECK(ob_smove_is_done(&m));
    // And it holds position like any arrived move.
    CHECK(ob_smove_tick(&m, 2, 650.0) > 0.0);
}

// ---- then= at arrival (3.9.0) ---------------------------------------

TEST(then_defaults_to_hold_and_hold_never_hands_back) {
    ob_smove_t m;
    ob_smove_init(&m);
    CHECK_EQ_INT(m.then, OB_SMOVE_THEN_HOLD);
    ob_smove_start(&m, 0, 0.0, 2048.0, 1000.0, 4000.0);
    ob_smove_set_then(&m, OB_SMOVE_THEN_COAST);
    CHECK_EQ_INT(m.then, OB_SMOVE_THEN_COAST);
    CHECK(!ob_smove_take_end(&m));          // not arrived: nothing yet
    // start resets the end-state (set it AFTER arming, like the
    // bindings do); an out-of-range code clamps to hold.
    ob_smove_start(&m, 0, 0.0, 2048.0, 1000.0, 4000.0);
    CHECK_EQ_INT(m.then, OB_SMOVE_THEN_HOLD);
    ob_smove_set_then(&m, 7);
    CHECK_EQ_INT(m.then, OB_SMOVE_THEN_HOLD);
    (void)run_plant(&m, 0, 4000, 0.0);
    CHECK(ob_smove_is_done(&m));
    CHECK(!ob_smove_take_end(&m));          // hold keeps the lock
    CHECK_EQ_INT(m.state, OB_SMOVE_HOLD);
    // IDLE never hands back either.
    ob_smove_stop(&m);
    ob_smove_set_then(&m, OB_SMOVE_THEN_COAST);
    CHECK(!ob_smove_take_end(&m));
}

TEST(coast_and_brake_hand_back_once_at_arrival_and_stay_done) {
    for (unsigned char then = OB_SMOVE_THEN_COAST;
         then <= OB_SMOVE_THEN_BRAKE; then++) {
        ob_smove_t m;
        ob_smove_init(&m);
        ob_smove_start(&m, 0, 0.0, 2048.0, 1000.0, 4000.0);
        ob_smove_set_then(&m, then);
        ob_float_t pos = 0.0;
        int handed = 0;
        long t;
        for (t = 0; t < 4000; t++) {
            ob_float_t cmd = ob_smove_tick(&m, t, pos);
            pos += cmd * (ob_float_t)0.001;
            if (ob_smove_take_end(&m)) {
                handed++;
                CHECK(ob_smove_is_done(&m));   // arrived, then handed
            }
        }
        CHECK_EQ_INT(handed, 1);                // exactly once
        CHECK(ob_smove_is_done(&m));            // a late poll still
                                                // reads the arrival
        CHECK_EQ_INT(m.state, OB_SMOVE_IDLE);
        CHECK(ob_smove_tick(&m, t, pos + 500.0) == 0.0);  // silent
        CHECK(!ob_smove_take_end(&m));
        CHECK(fabs(pos - 2048.0) < OB_SMOVE_DONE_TOL_COUNTS);
    }
}

TEST(hold_at_resets_then_to_hold) {
    ob_smove_t m;
    ob_smove_init(&m);
    ob_smove_start(&m, 0, 0.0, 100.0, 1000.0, 4000.0);
    ob_smove_set_then(&m, OB_SMOVE_THEN_BRAKE);
    ob_smove_hold_at(&m, 50.0);
    CHECK_EQ_INT(m.then, OB_SMOVE_THEN_HOLD);
    CHECK(ob_smove_is_done(&m));
    CHECK(!ob_smove_take_end(&m));
}

int main(void) {
    RUN(then_defaults_to_hold_and_hold_never_hands_back);
    RUN(coast_and_brake_hand_back_once_at_arrival_and_stay_done);
    RUN(hold_at_resets_then_to_hold);
    RUN(idle_outputs_zero_and_not_done);
    RUN(move_converges_on_relative_goal);
    RUN(negative_move_converges);
    RUN(launch_is_ramped_not_a_step);
    RUN(peak_speed_respects_cruise);
    RUN(done_requires_arrival_not_just_time);
    RUN(done_latches_through_disturbance_but_hold_corrects);
    RUN(hold_at_is_done_and_corrects_both_directions);
    RUN(stop_returns_to_idle_silence);
    RUN(new_start_supersedes_hold_and_clears_done);
    RUN(zero_delta_move_is_immediately_done_hold);
    return harness_exit("st_move_core");
}
