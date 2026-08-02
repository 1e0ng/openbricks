// SPDX-License-Identifier: MIT
// Native tests for st_servo_core — slot planner + odometry unwrap.
// Boundary values set directly on the structs, under ASan/UBSan.

#include "harness.h"
#include "st_servo_core.h"

static ob_sservo_t sv;

static void reset(void) {
    ob_sservo_init(&sv);
}

static void feed_pos(int slot, uint16_t raw) {
    // Simulate: planner issued a POS read for ``slot``, reply arrives.
    sv.read_in_flight = slot;
    uint8_t pl[2] = { (uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8) };
    ob_sservo_read_result(&sv, 1, pl, 2);
}

TEST(encode_speed_matches_driver_and_clamps) {
    CHECK_EQ_INT(ob_sservo_encode_speed(1000), 1000);
    CHECK_EQ_INT(ob_sservo_encode_speed(-1000), 1000 | 0x8000);
    CHECK_EQ_INT(ob_sservo_encode_speed(0x7FFF), 0x7FFF);
    CHECK_EQ_INT(ob_sservo_encode_speed(0x8000), 0x7FFF);        // clamp
    CHECK_EQ_INT(ob_sservo_encode_speed(-0x7FFF), 0x7FFF | 0x8000);
    CHECK_EQ_INT(ob_sservo_encode_speed(-2147483647), 0x7FFF | 0x8000);
    CHECK_EQ_INT(ob_sservo_encode_speed(0), 0);
}

TEST(unwrap_exact_half_range_boundaries) {
    // Delta +2047 stays forward; -2048 is the backward tie-break —
    // the exact boundary of the (raw - last + 2048) & 0xFFF - 2048
    // arithmetic, unreachable precisely through the Python suite.
    reset();
    ob_sservo_attach(&sv, 0, 1, 0, 0);
    sv.slots[0].config_step = 3;
    feed_pos(0, 0);                        // prime
    feed_pos(0, 2047);
    CHECK_EQ_INT(ob_sservo_counts(&sv, 0), 2047);
    reset();
    ob_sservo_attach(&sv, 0, 1, 0, 0);
    sv.slots[0].config_step = 3;
    feed_pos(0, 0);
    feed_pos(0, 2048);                     // ambiguous: defined as -2048
    CHECK_EQ_INT(ob_sservo_counts(&sv, 0), -2048);
}

TEST(unwrap_across_zero_both_directions) {
    reset();
    ob_sservo_attach(&sv, 0, 1, 0, 0);
    sv.slots[0].config_step = 3;
    feed_pos(0, 4090);                     // prime
    feed_pos(0, 30);                       // +36 across the wrap
    CHECK_EQ_INT(ob_sservo_counts(&sv, 0), 36);
    feed_pos(0, 4090);                     // -36 back across
    CHECK_EQ_INT(ob_sservo_counts(&sv, 0), 0);
}

TEST(invert_mirrors_reported_frame) {
    reset();
    ob_sservo_attach(&sv, 0, 1, 1, 0);     // inverted
    sv.slots[0].config_step = 3;
    feed_pos(0, 0);
    feed_pos(0, 100);
    CHECK_EQ_INT(ob_sservo_counts(&sv, 0), -100);
}

TEST(planner_priorities_and_torque_dedup) {
    reset();
    ob_sservo_attach(&sv, 0, 7, 0, 45);
    ob_sservo_op_t op;

    // Config sequence first, exact order.
    static const uint8_t regs[3] = { OB_SREG_OP_MODE, OB_SREG_TORQUE,
                                     OB_SREG_GOAL_ACC };
    for (int i = 0; i < 3; i++) {
        ob_sservo_next_op(&sv, &op);
        CHECK_EQ_INT(op.kind, OB_SOP_WRITE);
        CHECK_EQ_INT(op.reg, regs[i]);
        ob_sservo_op_started(&sv, &op);
    }

    // Speed command: torque-on first (priority 1), then ONE sync.
    ob_sservo_set_speed(&sv, 0, 500);
    ob_sservo_next_op(&sv, &op);
    CHECK_EQ_INT(op.kind, OB_SOP_WRITE);
    CHECK_EQ_INT(op.reg, OB_SREG_TORQUE);
    CHECK_EQ_INT(op.data[0], 1);
    ob_sservo_op_started(&sv, &op);
    ob_sservo_next_op(&sv, &op);
    CHECK_EQ_INT(op.kind, OB_SOP_SYNC_SPEED);
    ob_sservo_op_started(&sv, &op);

    // THE torque-dedup regression: another set_speed while torque is
    // known-on must NOT re-stage a torque write (it starved the
    // syncs, 98 packets in 100).
    ob_sservo_set_speed(&sv, 0, 600);
    ob_sservo_next_op(&sv, &op);
    CHECK(op.kind != OB_SOP_WRITE || op.reg != OB_SREG_TORQUE);
}

TEST(fairness_sync_then_read_alternation) {
    reset();
    ob_sservo_attach(&sv, 0, 7, 0, 45);
    sv.slots[0].config_step = 3;
    sv.slots[0].torque_on = 1;
    ob_sservo_op_t op;
    // Dirty every "tick" like the drivebase does; ops must alternate
    // sync/read — the frozen-odometry regression.
    int syncs = 0, reads = 0;
    for (int i = 0; i < 10; i++) {
        ob_sservo_set_speed(&sv, 0, 100 + i);
        ob_sservo_next_op(&sv, &op);
        ob_sservo_op_started(&sv, &op);
        if (op.kind == OB_SOP_SYNC_SPEED) {
            syncs++;
        }
        if (op.kind == OB_SOP_READ_POS) {
            reads++;
            ob_sservo_read_result(&sv, 1, (const uint8_t *)"\x00\x00", 2);
        }
    }
    CHECK_EQ_INT(syncs, 5);
    CHECK_EQ_INT(reads, 5);
}

TEST(failed_reads_count_stale_and_recover) {
    reset();
    ob_sservo_attach(&sv, 0, 7, 0, 45);
    sv.slots[0].config_step = 3;
    sv.read_in_flight = 0;
    ob_sservo_read_result(&sv, 0, NULL, 0);
    sv.read_in_flight = 0;
    ob_sservo_read_result(&sv, 0, NULL, 0);
    CHECK_EQ_INT(sv.slots[0].reads_failed, 2);
    CHECK_EQ_INT(sv.slots[0].stale, 2);
    feed_pos(0, 10);
    CHECK_EQ_INT(sv.slots[0].stale, 0);    // success clears streak
}

TEST(bounds_are_guarded) {
    reset();
    CHECK_EQ_INT(ob_sservo_attach(&sv, -1, 1, 0, 0), -1);
    CHECK_EQ_INT(ob_sservo_attach(&sv, OB_SSERVO_SLOTS, 1, 0, 0), -1);
    CHECK_EQ_INT(ob_sservo_set_speed(&sv, 0, 1), -1);   // not attached
    CHECK_EQ_INT(ob_sservo_coast(&sv, 3), -1);
    CHECK_EQ_INT(ob_sservo_counts(&sv, 99), 0);
    ob_sservo_detach(&sv, -5);                          // silent
    // Result for a detached in-flight read is dropped, not written.
    ob_sservo_attach(&sv, 0, 1, 0, 0);
    sv.read_in_flight = 0;
    ob_sservo_detach(&sv, 0);
    CHECK_EQ_INT(sv.read_in_flight, -1);
}

int main(void) {
    RUN(encode_speed_matches_driver_and_clamps);
    RUN(unwrap_exact_half_range_boundaries);
    RUN(unwrap_across_zero_both_directions);
    RUN(invert_mirrors_reported_frame);
    RUN(planner_priorities_and_torque_dedup);
    RUN(fairness_sync_then_read_alternation);
    RUN(failed_reads_count_stale_and_recover);
    RUN(bounds_are_guarded);
    return harness_exit("st_servo_core");
}
