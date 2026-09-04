// SPDX-License-Identifier: MIT
// Native tests for motor_process_core — the tick registry + wall
// clock. Exact 2^32 wrap values, direct and cheap.

#include "harness.h"
#include "motor_process_core.h"

static ob_motor_process_t m;
static int fired;

static void cb(void *ctx) {
    (void)ctx;
    fired++;
}

TEST(wall_clock_wraps_through_2_32) {
    ob_motor_process_init(&m);
    ob_motor_process_fire_c_at(&m, 0xFFFFFFF0u);  // first: +period (1)
    long before = m.virtual_now_ms;
    ob_motor_process_fire_c_at(&m, 0x0000000Au);  // wrap: dt = 26
    CHECK_EQ_INT(m.virtual_now_ms - before, 26);
}

TEST(wall_clock_clamps_pathological_jumps) {
    ob_motor_process_init(&m);
    ob_motor_process_fire_c_at(&m, 1000);
    long before = m.virtual_now_ms;
    ob_motor_process_fire_c_at(&m, 1000 + 10 * OB_TICK_MAX_CATCHUP_MS);
    CHECK_EQ_INT(m.virtual_now_ms - before, OB_TICK_MAX_CATCHUP_MS);
}

TEST(starvation_gap_counted_not_dilated) {
    ob_motor_process_init(&m);
    ob_motor_process_fire_c_at(&m, 1000);
    long before = m.virtual_now_ms;
    ob_motor_process_fire_c_at(&m, 1981);         // the bench gap
    CHECK_EQ_INT(m.virtual_now_ms - before, 981);
}

TEST(legacy_fire_does_not_disturb_wall_tracking) {
    ob_motor_process_init(&m);
    ob_motor_process_fire_c_at(&m, 1000);
    ob_motor_process_fire_c(&m);                  // +period, no wall touch
    long before = m.virtual_now_ms;
    ob_motor_process_fire_c_at(&m, 1001);         // dt from ITS last
    CHECK_EQ_INT(m.virtual_now_ms - before, 1);
}

TEST(registry_dedups_and_bounds) {
    ob_motor_process_init(&m);
    fired = 0;
    CHECK_EQ_INT(ob_motor_process_register_c(&m, cb, NULL), 0);
    CHECK_EQ_INT(ob_motor_process_register_c(&m, cb, NULL), 0); // idempotent
    CHECK_EQ_INT(ob_motor_process_count_c(&m), 1);
    ob_motor_process_fire_c(&m);
    CHECK_EQ_INT(fired, 1);
    ob_motor_process_unregister_c(&m, cb, NULL);
    ob_motor_process_fire_c(&m);
    CHECK_EQ_INT(fired, 1);
    // Fill every slot; the next distinct registration must refuse.
    for (int i = 0; i < OB_MAX_C_CALLBACKS; i++) {
        CHECK_EQ_INT(ob_motor_process_register_c(&m, cb, (void *)(long)(i + 1)), 0);
    }
    CHECK_EQ_INT(ob_motor_process_register_c(&m, cb,
                                             (void *)(long)0x999), -1);
}

TEST(reset_reprimes_wall_tracker) {
    ob_motor_process_init(&m);
    ob_motor_process_fire_c_at(&m, 1000);
    ob_motor_process_fire_c_at(&m, 1500);
    ob_motor_process_reset(&m);
    ob_motor_process_fire_c_at(&m, 9999);         // first-after-reset
    CHECK_EQ_INT(m.virtual_now_ms, m.period_ms);
}

TEST(tick_stats_first_fire_primes_without_a_gap) {
    ob_tick_stats_t s;
    ob_tick_stats_init(&s);
    ob_tick_stats_update(&s, 5000, 1000, 250);
    CHECK_EQ_INT(s.fires, 1);
    CHECK_EQ_INT(s.late, 0);
    CHECK_EQ_INT(s.worst_dt_us, 0);
    CHECK_EQ_INT(s.last_dt_us, 0);
}

TEST(tick_stats_counts_late_fires_and_keeps_the_worst_gap) {
    ob_tick_stats_t s;
    ob_tick_stats_init(&s);
    ob_tick_stats_update(&s, 0, 1000, 250);
    ob_tick_stats_update(&s, 1000, 1000, 250);      // on time
    ob_tick_stats_update(&s, 2250, 1000, 250);      // at the tolerance
    ob_tick_stats_update(&s, 3501, 1000, 250);      // 1 us over: late
    ob_tick_stats_update(&s, 8000, 1000, 250);      // a 4.5 ms hold-off
    ob_tick_stats_update(&s, 9000, 1000, 250);
    CHECK_EQ_INT(s.fires, 6);
    CHECK_EQ_INT(s.late, 2);
    CHECK_EQ_INT(s.worst_dt_us, 4499);
    CHECK_EQ_INT(s.last_dt_us, 1000);
}

TEST(tick_stats_wrap_through_2_32_is_not_a_gap) {
    ob_tick_stats_t s;
    ob_tick_stats_init(&s);
    ob_tick_stats_update(&s, 0xFFFFFF00u, 1000, 250);
    ob_tick_stats_update(&s, 0x000002E8u, 1000, 250);   // dt = 1000
    CHECK_EQ_INT(s.late, 0);
    CHECK_EQ_INT(s.last_dt_us, 1000);
    CHECK_EQ_INT(s.worst_dt_us, 1000);
}

TEST(tick_stats_init_clears_everything) {
    ob_tick_stats_t s;
    ob_tick_stats_init(&s);
    ob_tick_stats_update(&s, 0, 1000, 250);
    ob_tick_stats_update(&s, 9000, 1000, 250);
    CHECK_EQ_INT(s.late, 1);
    ob_tick_stats_init(&s);
    CHECK_EQ_INT(s.fires, 0);
    CHECK_EQ_INT(s.late, 0);
    CHECK_EQ_INT(s.worst_dt_us, 0);
    CHECK(!s.inited);
}

int main(void) {
    RUN(tick_stats_first_fire_primes_without_a_gap);
    RUN(tick_stats_counts_late_fires_and_keeps_the_worst_gap);
    RUN(tick_stats_wrap_through_2_32_is_not_a_gap);
    RUN(tick_stats_init_clears_everything);
    RUN(wall_clock_wraps_through_2_32);
    RUN(wall_clock_clamps_pathological_jumps);
    RUN(starvation_gap_counted_not_dilated);
    RUN(legacy_fire_does_not_disturb_wall_tracking);
    RUN(registry_dedups_and_bounds);
    RUN(reset_reprimes_wall_tracker);
    return harness_exit("motor_process_core");
}
