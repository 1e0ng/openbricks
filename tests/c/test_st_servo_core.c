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

    // Speed command: torque-on first (priority 1, as a sync since
    // the atomic-stop change), then ONE speed sync.
    ob_sservo_set_speed(&sv, 0, 500);
    ob_sservo_next_op(&sv, &op);
    CHECK_EQ_INT(op.kind, OB_SOP_SYNC_TORQUE);
    CHECK_EQ_INT(op.sync_n, 1);
    CHECK_EQ_INT(op.sync_ids[0], 7);
    CHECK_EQ_INT(op.sync_data[0], 1);
    ob_sservo_op_started(&sv, &op);
    CHECK_EQ_INT(sv.slots[0].torque_on, 1);
    ob_sservo_next_op(&sv, &op);
    CHECK_EQ_INT(op.kind, OB_SOP_SYNC_SPEED);
    ob_sservo_op_started(&sv, &op);

    // THE torque-dedup regression: another set_speed while torque is
    // known-on must NOT re-stage a torque write (it starved the
    // syncs, 98 packets in 100).
    ob_sservo_set_speed(&sv, 0, 600);
    ob_sservo_next_op(&sv, &op);
    CHECK(op.kind != OB_SOP_SYNC_TORQUE);
}

TEST(coast_both_wheels_rides_one_sync_torque_packet) {
    // The atomic-stop contract: two slots with pending torque
    // commands produce ONE sync-write covering both — a drivebase
    // coast releases both wheels at the same packet boundary.
    reset();
    ob_sservo_attach(&sv, 0, 2, 1, 45);
    ob_sservo_attach(&sv, 1, 1, 0, 45);
    sv.slots[0].config_step = 3;
    sv.slots[1].config_step = 3;
    sv.slots[0].torque_on = 1;
    sv.slots[1].torque_on = 1;
    CHECK_EQ_INT(ob_sservo_coast(&sv, 0), 0);
    CHECK_EQ_INT(ob_sservo_coast(&sv, 1), 0);
    ob_sservo_op_t op;
    ob_sservo_next_op(&sv, &op);
    CHECK_EQ_INT(op.kind, OB_SOP_SYNC_TORQUE);
    CHECK_EQ_INT(op.sync_n, 2);
    CHECK_EQ_INT(op.sync_ids[0], 2);
    CHECK_EQ_INT(op.sync_data[0], 0);
    CHECK_EQ_INT(op.sync_ids[1], 1);
    CHECK_EQ_INT(op.sync_data[1], 0);
    ob_sservo_op_started(&sv, &op);
    CHECK_EQ_INT(sv.slots[0].torque_cmd, -1);
    CHECK_EQ_INT(sv.slots[1].torque_cmd, -1);
    CHECK_EQ_INT(sv.slots[0].torque_on, 0);
    CHECK_EQ_INT(sv.slots[1].torque_on, 0);
    // Fully consumed: no second torque packet.
    ob_sservo_next_op(&sv, &op);
    CHECK(op.kind != OB_SOP_SYNC_TORQUE);
}

TEST(mixed_torque_values_share_one_sync_packet) {
    // Per-servo data in a sync-write is independent: one slot
    // coasting while the other torques on still takes one packet.
    reset();
    ob_sservo_attach(&sv, 0, 2, 0, 45);
    ob_sservo_attach(&sv, 1, 1, 0, 45);
    sv.slots[0].config_step = 3;
    sv.slots[1].config_step = 3;
    sv.slots[0].torque_on = 1;
    CHECK_EQ_INT(ob_sservo_coast(&sv, 0), 0);
    CHECK_EQ_INT(ob_sservo_set_speed(&sv, 1, 200), 0);  // stages torque-on
    ob_sservo_op_t op;
    ob_sservo_next_op(&sv, &op);
    CHECK_EQ_INT(op.kind, OB_SOP_SYNC_TORQUE);
    CHECK_EQ_INT(op.sync_n, 2);
    CHECK_EQ_INT(op.sync_data[0], 0);
    CHECK_EQ_INT(op.sync_data[1], 1);
    ob_sservo_op_started(&sv, &op);
    CHECK_EQ_INT(sv.slots[0].torque_on, 0);
    CHECK_EQ_INT(sv.slots[1].torque_on, 1);
}

TEST(torque_sync_skips_the_fairness_gate) {
    // A speed sync just went out (last_was_sync set); a torque
    // command must still go out NEXT — coast is e-stop-adjacent and
    // must never wait behind the sync/read alternation.
    reset();
    ob_sservo_attach(&sv, 0, 7, 0, 45);
    sv.slots[0].config_step = 3;
    sv.slots[0].torque_on = 1;
    sv.last_was_sync = 1;
    CHECK_EQ_INT(ob_sservo_coast(&sv, 0), 0);
    ob_sservo_op_t op;
    ob_sservo_next_op(&sv, &op);
    CHECK_EQ_INT(op.kind, OB_SOP_SYNC_TORQUE);
    // And it leaves the fairness state alone: the deferred speed
    // sync still owes a read first.
    ob_sservo_op_started(&sv, &op);
    CHECK_EQ_INT(sv.last_was_sync, 1);
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

TEST(parked_slots_do_not_halve_a_driving_wheels_odometry) {
    // Two wheels driving, two task motors parked. Under a flat
    // round-robin the wheels got half the reads they had when alone
    // on the bus — and that rate IS the drivebase heading loop's
    // bandwidth. Driving slots must keep nearly all of it.
    reset();
    for (int i = 0; i < 4; i++) {
        ob_sservo_attach(&sv, i, (uint8_t)(i + 1), 0, 45);
        sv.slots[i].config_step = 3;
        sv.slots[i].torque_on = 1;
    }
    sv.slots[0].hot = 1;      // left wheel driving
    sv.slots[1].hot = 1;      // right wheel driving
    sv.slots[2].hot = 0;      // task motors parked
    sv.slots[3].hot = 0;

    int reads[4] = {0, 0, 0, 0};
    ob_sservo_op_t op;
    for (int n = 0; n < 400; n++) {
        ob_sservo_next_op(&sv, &op);
        if (op.kind == OB_SOP_READ_POS) {
            reads[op.slot]++;
            ob_sservo_op_started(&sv, &op);
            ob_sservo_read_result(&sv, 1, (const uint8_t *)"\x00\x00", 2);
        } else {
            ob_sservo_op_started(&sv, &op);
        }
    }
    int hot = reads[0] + reads[1];
    int cold = reads[2] + reads[3];
    // Wheels take the large majority; parked motors still get a
    // trickle so angle()/speed() never go arbitrarily stale.
    CHECK(hot > cold * 5);
    CHECK(cold > 0);
    // And the two wheels share evenly between themselves.
    int diff = reads[0] - reads[1];
    CHECK(diff < 3 && diff > -3);
}

TEST(all_parked_still_get_polled) {
    // No slot driving: everything is cold, and the bus must still
    // service them rather than stall on an empty preference.
    reset();
    ob_sservo_attach(&sv, 0, 7, 0, 45);
    sv.slots[0].config_step = 3;
    sv.slots[0].hot = 0;
    ob_sservo_op_t op;
    int reads = 0;
    for (int n = 0; n < 20; n++) {
        ob_sservo_next_op(&sv, &op);
        if (op.kind == OB_SOP_READ_POS) {
            reads++;
            ob_sservo_op_started(&sv, &op);
            ob_sservo_read_result(&sv, 1, (const uint8_t *)"\x00\x00", 2);
        }
    }
    CHECK(reads > 10);
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


TEST(widened_feedback_decodes_speed_and_load) {
    ob_sservo_t sv;
    ob_sservo_init(&sv);
    ob_sservo_attach(&sv, 0, 7, 0, 45);
    sv.slots[0].config_step = 3;
    sv.read_in_flight = 0;
    // pos=0x123, speed=-300 (sign-magnitude b15), load raw bit10
    // SET = pushing POSITIVE (bench-pinned decode) -> +25.
    uint8_t pl[6] = {
        0x23, 0x01,
        (uint8_t)(300 & 0xFF), (uint8_t)(((300 >> 8) & 0x7F) | 0x80),
        (uint8_t)(25 & 0xFF), (uint8_t)(0x04),
    };
    ob_sservo_read_result(&sv, 1, pl, 6);
    CHECK_EQ_INT(ob_sservo_speed_steps(&sv, 0), -300);
    CHECK_EQ_INT(ob_sservo_load_raw(&sv, 0), 25);
    CHECK_EQ_INT(ob_sservo_feedback_fresh(&sv, 0), 1);
}

TEST(inverted_slot_flips_feedback_to_user_frame) {
    ob_sservo_t sv;
    ob_sservo_init(&sv);
    ob_sservo_attach(&sv, 0, 7, 1, 45);        // invert
    sv.slots[0].config_step = 3;
    sv.read_in_flight = 0;
    // Raw: speed +500, load b10-clear = -100. The slot invert flips
    // both into the user frame: speed -500, load +100.
    uint8_t pl[6] = { 0, 0, 0xF4, 0x01, 0x64, 0x00 };
    ob_sservo_read_result(&sv, 1, pl, 6);
    CHECK_EQ_INT(ob_sservo_speed_steps(&sv, 0), -500);
    CHECK_EQ_INT(ob_sservo_load_raw(&sv, 0), 100);
}

TEST(short_reply_keeps_position_but_no_feedback_freshness) {
    // A 2-byte reply still feeds odometry (defence in depth) but
    // must not claim speed/load freshness it never decoded.
    ob_sservo_t sv;
    ob_sservo_init(&sv);
    ob_sservo_attach(&sv, 0, 7, 0, 45);
    sv.slots[0].config_step = 3;
    sv.read_in_flight = 0;
    ob_sservo_read_result(&sv, 1, (const uint8_t *)"\x10\x00", 2);
    CHECK_EQ_INT(ob_sservo_counts(&sv, 0) >= 0, 1);
    CHECK_EQ_INT(ob_sservo_feedback_fresh(&sv, 0), 0);
}

TEST(failed_read_marks_feedback_stale) {
    ob_sservo_t sv;
    ob_sservo_init(&sv);
    ob_sservo_attach(&sv, 0, 7, 0, 45);
    sv.slots[0].config_step = 3;
    sv.read_in_flight = 0;
    uint8_t pl[6] = { 0, 0, 0x10, 0x00, 0x05, 0x04 };  // b10: +5
    ob_sservo_read_result(&sv, 1, pl, 6);
    CHECK_EQ_INT(ob_sservo_feedback_fresh(&sv, 0), 1);
    sv.read_in_flight = 0;
    ob_sservo_read_result(&sv, 0, NULL, 0);    // timeout
    CHECK_EQ_INT(ob_sservo_feedback_fresh(&sv, 0), 0);
    // Values retained (diagnostics) but flagged unfresh.
    CHECK_EQ_INT(ob_sservo_speed_steps(&sv, 0), 0x10);
}

int main(void) {
    RUN(widened_feedback_decodes_speed_and_load);
    RUN(inverted_slot_flips_feedback_to_user_frame);
    RUN(short_reply_keeps_position_but_no_feedback_freshness);
    RUN(failed_read_marks_feedback_stale);
    RUN(encode_speed_matches_driver_and_clamps);
    RUN(unwrap_exact_half_range_boundaries);
    RUN(unwrap_across_zero_both_directions);
    RUN(invert_mirrors_reported_frame);
    RUN(planner_priorities_and_torque_dedup);
    RUN(coast_both_wheels_rides_one_sync_torque_packet);
    RUN(mixed_torque_values_share_one_sync_packet);
    RUN(torque_sync_skips_the_fairness_gate);
    RUN(fairness_sync_then_read_alternation);
    RUN(parked_slots_do_not_halve_a_driving_wheels_odometry);
    RUN(all_parked_still_get_polled);
    RUN(failed_reads_count_stale_and_recover);
    RUN(bounds_are_guarded);
    return harness_exit("st_servo_core");
}
