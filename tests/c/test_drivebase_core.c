// SPDX-License-Identifier: MIT
// Native tests for drivebase_core's decelerating stop
// (ob_drivebase_stop_decel): the controlled half of a brake/hold,
// with both axes closed-loop through the ramp.

#include <math.h>

#include "harness.h"
#include "drivebase_core.h"

// Bench geometry (88 mm wheels, 136 mm track) with the default
// gains; accel 400 wheel-deg/s^2 like the MP harness.
static void setup(ob_drivebase_t *db, ob_servo_t *l, ob_servo_t *r) {
    memset(l, 0, sizeof(*l));
    memset(r, 0, sizeof(*r));
    ob_drivebase_init(db, l, r, 88.0, 136.0,
                      OB_DRIVEBASE_DEFAULT_KP_SUM,
                      OB_DRIVEBASE_DEFAULT_KP_DIFF);
    db->accel_dps2 = 400.0;
}

// Perfect plant at 1 kHz: each wheel follows its commanded speed
// exactly. ``max_step`` reports the largest one-tick change in the
// left command — the no-cliff check.
static void run_plant(ob_drivebase_t *db, long start_ms, int ms,
                      ob_float_t *max_step) {
    ob_float_t prev = db->left->target_dps;
    for (int t = 0; t < ms; t++) {
        ob_drivebase_tick(db, start_ms + t);
        ob_float_t d = fabs((double)(db->left->target_dps - prev));
        if (max_step != NULL && d > *max_step) {
            *max_step = d;
        }
        prev = db->left->target_dps;
        db->left->observer.pos_hat  += db->left->target_dps  * 0.001;
        db->right->observer.pos_hat += db->right->target_dps * 0.001;
        if (db->use_gyro) {
            // Honest IMU on a non-slipping chassis.
            db->heading_override_wheel_deg =
                (db->left->observer.pos_hat - db->right->observer.pos_hat)
                / 2.0;
        }
    }
}

TEST(at_rest_arms_nothing_and_keeps_the_absolute_hold) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    db.use_gyro  = true;
    db.turn_hold = 123.0;          // the absolute target of a done move
    db.fwd_hold  = 7.0;
    l.observer.pos_hat = 50.0;
    r.observer.pos_hat = 40.0;
    // The residual P-term at a done latch: well under standstill.
    l.target_dps = 12.0;
    r.target_dps = -12.0;
    CHECK(!ob_drivebase_stop_decel(&db, 1000, 400.0));
    CHECK(db.done);
    CHECK(!db.fwd_active);
    CHECK(!db.turn_active);
    CHECK(db.turn_hold == 123.0);  // NOT re-baselined to measured
    CHECK(db.fwd_hold == 45.0);    // holds where it is
}

TEST(zero_accel_arms_nothing) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    db.accel_dps2 = 0.0;
    l.target_dps = r.target_dps = 200.0;
    CHECK(!ob_drivebase_stop_decel(&db, 0, 0.0));
    CHECK(db.done);
}

TEST(cruise_ramps_to_rest_at_the_accel_limit_without_a_cliff) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    // Prime the tick timebase so dt is 1 ms from the first ramp tick
    // — BEFORE staging the commanded speeds: a tick in the hold
    // state writes its own (~0) commands, the way the pump syncs the
    // bridges from the slots right before staging a stop.
    ob_drivebase_tick(&db, 999);
    l.target_dps = r.target_dps = 200.0;
    CHECK(ob_drivebase_stop_decel(&db, 1000, 400.0));
    CHECK(!db.done);
    CHECK(db.fwd_active);
    CHECK(!db.turn_active);
    ob_float_t max_step = 0.0;
    run_plant(&db, 1000, 1500, &max_step);
    CHECK(db.done);
    // v0^2 / 2a = 200^2 / 800 = 50 wheel-deg of roll-out.
    ob_float_t travelled = (l.observer.pos_hat + r.observer.pos_hat) / 2.0;
    CHECK(fabs((double)(travelled - 50.0)) < 3.0);
    // Shaped all the way: no tick stepped the command by more than
    // the accel limit's share (plus P-term wiggle).
    CHECK(max_step < 2.0);
    CHECK(fabs((double)l.target_dps) < 5.0);
    CHECK(fabs((double)r.target_dps) < 5.0);
}

TEST(gyro_counter_steers_through_the_ramp) {
    // THE property: the heading loop stays closed while braking. A
    // heading that reads "veered right" mid-ramp must produce the
    // same counter-steer a straight would — right out-paces left.
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    db.use_gyro  = true;
    db.turn_hold = 0.0;
    ob_drivebase_tick(&db, 999);
    l.target_dps = r.target_dps = 200.0;
    CHECK(ob_drivebase_stop_decel(&db, 1000, 400.0));
    // First 50 ms honest, then pin a +5 body-deg (7.7 wheel-deg)
    // veer for 100 ms with the plant otherwise perfect.
    run_plant(&db, 1000, 50, NULL);
    ob_float_t l0 = l.observer.pos_hat, r0 = r.observer.pos_hat;
    for (int t = 0; t < 100; t++) {
        db.heading_override_wheel_deg =
            ob_drivebase_body_to_wheel_diff(&db, 5.0);
        ob_drivebase_tick(&db, 1050 + t);
        l.observer.pos_hat += l.target_dps * 0.001;
        r.observer.pos_hat += r.target_dps * 0.001;
    }
    CHECK(db.fwd_active);                       // still ramping
    CHECK(r.observer.pos_hat - r0 > l.observer.pos_hat - l0 + 1.0);
    CHECK(r.target_dps > l.target_dps + 20.0);
}

TEST(rotation_decelerates_on_the_turn_accel_and_moves_the_hold) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    db.use_gyro  = true;
    db.turn_hold = 10.0;           // refreshed by the caller: measured
    db.heading_override_wheel_deg = 10.0;
    l.observer.pos_hat = 10.0;
    r.observer.pos_hat = -10.0;
    ob_drivebase_tick(&db, 999);
    l.target_dps = 100.0;          // turning in place, 100 wheel-dps
    r.target_dps = -100.0;
    CHECK(ob_drivebase_stop_decel(&db, 1000, 200.0));
    CHECK(db.turn_active);
    CHECK(!db.fwd_active);         // forward axis at rest: hold
    run_plant(&db, 1000, 1500, NULL);
    CHECK(db.done);
    // 100^2 / (2 * 200) = 25 wheel-deg of rotation roll-out, landed
    // and locked into the (absolute) hold.
    CHECK(fabs((double)(db.turn_hold - 35.0)) < 3.0);
    ob_float_t diff = (l.observer.pos_hat - r.observer.pos_hat) / 2.0;
    CHECK(fabs((double)(diff - 35.0)) < 3.0);
}

TEST(stop_decel_keeps_the_move_diagnostics_but_restarts_the_integral) {
    // Dispatched by done() before the move call returns: settle
    // stats must still describe THAT move afterwards.
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    db.expiry_captured = true;
    db.expiry_residual = 4.2;
    db.landings        = 2;
    db.integ_sum       = 3.0;
    db.integ_diff      = -1.0;
    l.target_dps = r.target_dps = 150.0;
    CHECK(ob_drivebase_stop_decel(&db, 0, 400.0));
    CHECK(db.expiry_captured);
    CHECK(db.expiry_residual == 4.2);
    CHECK_EQ_INT(db.landings, 2);
    CHECK(db.integ_sum == 0.0);
    CHECK(db.integ_diff == 0.0);
    CHECK(!db.landing_active);
}

// ---- the rest of the core, on the same plant ------------------------
// drivebase_core had no C-level suite before this file (the MP
// harness covers it end to end); these keep the c-unit coverage of
// the file honest now that it is compiled into this binary.

// Plant with a tracking fraction: 0.6 = the MP harness's laggy wheel
// (a real settle residual at expiry), 0 = a blocked robot.
static void run_plant_track(ob_drivebase_t *db, long start_ms, int ms,
                            ob_float_t track) {
    for (int t = 0; t < ms; t++) {
        ob_drivebase_tick(db, start_ms + t);
        db->left->observer.pos_hat  += db->left->target_dps  * track * 0.001;
        db->right->observer.pos_hat += db->right->target_dps * track * 0.001;
    }
}

static ob_float_t sum_pos(const ob_drivebase_t *db) {
    return (db->left->observer.pos_hat + db->right->observer.pos_hat) / 2.0;
}

static ob_float_t diff_pos(const ob_drivebase_t *db) {
    return (db->left->observer.pos_hat - db->right->observer.pos_hat) / 2.0;
}

// 88 mm wheel: 200 mm = 260.4 wheel-deg; 90 body-deg on a 136 mm
// track = 139.1 wheel-deg of differential.
#define MM200_WHEEL_DEG  260.4
#define TURN90_WHEEL_DEG 139.1

TEST(straight_converges_then_stop_clears_the_move) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_straight(&db, 0, 200.0, 150.0, false);
    CHECK(!ob_drivebase_is_done(&db));
    run_plant_track(&db, 1, 3500, 1.0);
    CHECK(ob_drivebase_is_done(&db));
    CHECK(fabs((double)(sum_pos(&db) - MM200_WHEEL_DEG)) < 5.0);
    CHECK(fabs((double)diff_pos(&db)) < 2.0);
    ob_float_t rs, rd, is, id; int n;
    ob_drivebase_settle_stats(&db, &rs, &rd, &n, &is, &id);
    CHECK(rs < 3.0);                 // a perfect plant needs no landing
    CHECK_EQ_INT(n, 0);
    db.integ_sum = 4.0;
    ob_drivebase_stop(&db);
    CHECK(ob_drivebase_is_done(&db));
    CHECK(db.integ_sum == 0.0);
    CHECK(!db.fwd_active && !db.turn_active);
}

TEST(reverse_straight_and_post_move_hold_bleed_the_integral) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_straight(&db, 0, -200.0, 150.0, false);
    run_plant_track(&db, 1, 3500, 1.0);
    CHECK(ob_drivebase_is_done(&db));
    CHECK(fabs((double)(sum_pos(&db) + MM200_WHEEL_DEG)) < 5.0);
    // Post-move hold retires a wound integral smoothly, not in a step.
    db.integ_sum = 10.0;
    ob_drivebase_tick(&db, 3600);
    CHECK(db.integ_sum < 10.0 && db.integ_sum > 9.0);
}

TEST(straight_with_carry_keeps_the_reference_advancing) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_straight(&db, 0, 100.0, 150.0, true);
    run_plant_track(&db, 1, 3000, 1.0);
    // Well past the profile: still active, still commanding cruise
    // (195 wheel-dps), and done by pbio's Stop.NONE rule (measured
    // at/past the target).
    CHECK(db.fwd_active);
    CHECK(l.target_dps > 150.0);
    CHECK(ob_drivebase_is_done(&db));
    CHECK(sum_pos(&db) > MM200_WHEEL_DEG);
}

TEST(turn_is_cw_positive_and_converges) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_turn(&db, 0, 90.0, 60.0);
    run_plant_track(&db, 1, 4000, 1.0);
    CHECK(ob_drivebase_is_done(&db));
    CHECK(fabs((double)(diff_pos(&db) - TURN90_WHEEL_DEG)) < 5.0);
    CHECK(l.observer.pos_hat > 0.0 && r.observer.pos_hat < 0.0);
    CHECK(fabs((double)sum_pos(&db)) < 2.0);
}

TEST(turn_armed_while_translating_ramps_the_forward_axis_down) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_tick(&db, 999);
    l.target_dps = r.target_dps = 200.0;       // line-follow handing over
    ob_drivebase_turn(&db, 1000, -45.0, 60.0);
    CHECK(db.fwd_active);                      // a STOP trajectory, not a cliff
    run_plant_track(&db, 1000, 4000, 1.0);
    CHECK(ob_drivebase_is_done(&db));
    CHECK(fabs((double)(sum_pos(&db) - 50.0)) < 5.0);   // 200^2 / 800
    CHECK(fabs((double)(diff_pos(&db) + TURN90_WHEEL_DEG / 2.0)) < 5.0);
}

TEST(curve_zero_angle_completes_at_once) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_curve(&db, 0, 150.0, 0.0, 100.0, false);
    CHECK(ob_drivebase_is_done(&db));
    CHECK(!db.fwd_active && !db.turn_active);
}

TEST(curve_zero_radius_turns_in_place_and_ramps_forward_down) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_tick(&db, 999);
    l.target_dps = r.target_dps = 200.0;
    ob_drivebase_curve(&db, 1000, 0.0, 90.0, 100.0, false);
    CHECK(db.turn_active);
    CHECK(db.fwd_active);
    run_plant_track(&db, 1000, 5000, 1.0);
    CHECK(ob_drivebase_is_done(&db));
    CHECK(fabs((double)(diff_pos(&db) - TURN90_WHEEL_DEG)) < 5.0);
    CHECK(fabs((double)(sum_pos(&db) - 50.0)) < 5.0);
    // From rest the forward axis simply holds.
    setup(&db, &l, &r);
    ob_drivebase_curve(&db, 0, 0.0, 90.0, 100.0, false);
    CHECK(!db.fwd_active);
}

TEST(curve_arcs_with_proportional_axes) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_curve(&db, 0, 150.0, 90.0, 100.0, false);
    run_plant_track(&db, 1, 6000, 1.0);
    CHECK(ob_drivebase_is_done(&db));
    // Centre travels 150 * pi/2 = 235.6 mm = 306.8 wheel-deg.
    CHECK(fabs((double)(sum_pos(&db) - 306.8)) < 6.0);
    CHECK(fabs((double)(diff_pos(&db) - TURN90_WHEEL_DEG)) < 5.0);
}

TEST(curve_with_carry_keeps_both_axes_moving) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_curve(&db, 0, 150.0, 90.0, 100.0, true);
    run_plant_track(&db, 1, 6000, 1.0);
    CHECK(db.fwd_active && db.turn_active);
    CHECK(l.target_dps > 100.0);
    CHECK(l.target_dps > r.target_dps);        // still arcing right
    CHECK(ob_drivebase_is_done(&db));
}

TEST(laggy_plant_lands_with_a_shaped_landing_and_reports_it) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_straight(&db, 0, 200.0, 150.0, false);
    run_plant_track(&db, 1, 8000, 0.6);
    CHECK(ob_drivebase_is_done(&db));
    CHECK(fabs((double)(sum_pos(&db) - MM200_WHEEL_DEG)) < 5.0);
    ob_float_t rs, rd, is, id; int n;
    ob_drivebase_settle_stats(&db, &rs, &rd, &n, &is, &id);
    CHECK(rs > 0.0);                 // a real residual at expiry...
    CHECK(n >= 1);                   // ...closed by a landing
    CHECK(is != 0.0);                // the integral was carrying the lag
}

TEST(stiction_residual_is_forgiven_at_the_settle_cap) {
    // A plant that tracks until 8 wheel-deg short, then sticks: the
    // landings make no progress, the cap fires, done latches (the
    // residual is inside the forgive limit).
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_straight(&db, 0, 200.0, 150.0, false);
    for (int t = 1; t <= 8000; t++) {
        ob_drivebase_tick(&db, t);
        ob_float_t track = (sum_pos(&db) < MM200_WHEEL_DEG - 8.0)
                           ? 1.0 : 0.0;
        l.observer.pos_hat += l.target_dps * track * 0.001;
        r.observer.pos_hat += r.target_dps * track * 0.001;
    }
    CHECK(ob_drivebase_is_done(&db));
    CHECK(sum_pos(&db) < MM200_WHEEL_DEG - 5.0);
}

TEST(blocked_robot_never_latches_done) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_straight(&db, 0, 200.0, 150.0, false);
    run_plant_track(&db, 1, 6000, 0.0);
    CHECK(!ob_drivebase_is_done(&db));
    CHECK(l.target_dps > 0.0);       // still pushing toward the target
}

TEST(trace_dump_is_oldest_first_and_wraps) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    static float out[OB_DRIVEBASE_TRACE_N + 10][6];
    CHECK_EQ_INT(ob_drivebase_trace_dump(&db, out, 300), 0);
    ob_drivebase_straight(&db, 0, 200.0, 150.0, false);
    run_plant_track(&db, 1, 500, 1.0);
    int n = ob_drivebase_trace_dump(&db, out, 300);
    CHECK(n > 20 && n < OB_DRIVEBASE_TRACE_N);
    CHECK(out[0][0] < out[n - 1][0]);
    // Past the ring's length the dump starts at the oldest surviving
    // row, and a short buffer truncates.
    run_plant_track(&db, 501, 5000, 1.0);
    n = ob_drivebase_trace_dump(&db, out, 300);
    CHECK_EQ_INT(n, OB_DRIVEBASE_TRACE_N);
    CHECK(out[0][0] < out[n - 1][0]);
    CHECK_EQ_INT(ob_drivebase_trace_dump(&db, out, 10), 10);
}

TEST(gyro_frame_reset_and_body_to_wheel_mapping) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    // 1 body-deg on the 88/136 geometry = 136 * pi / (88 * pi) =
    // 1.545 wheel-deg of differential.
    CHECK(fabs((double)(ob_drivebase_body_to_wheel_diff(&db, 1.0)
                        - 136.0 / 88.0)) < 1e-6);
    db.turn_hold = 40.0;
    db.heading_override_wheel_deg = 41.0;
    db.integ_diff = 2.0;
    ob_drivebase_gyro_frame_reset(&db);
    CHECK(db.turn_hold == 0.0);
    CHECK(db.heading_override_wheel_deg == 0.0);
    CHECK(db.integ_diff == 0.0);
}

TEST(gyro_mode_arms_straight_and_curve_from_the_held_target) {
    // The absolute-frame arms: straight() and curve() take the diff
    // start from turn_hold, not the encoder differential — an honest
    // IMU on a perfect chassis then lands both on their targets.
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    db.use_gyro  = true;
    db.turn_hold = 0.0;
    ob_drivebase_straight(&db, 0, 100.0, 150.0, false);
    run_plant(&db, 1, 3000, NULL);
    CHECK(ob_drivebase_is_done(&db));
    CHECK(fabs((double)diff_pos(&db)) < 2.0);
    ob_drivebase_curve(&db, 3001, 150.0, 90.0, 100.0, false);
    run_plant(&db, 3002, 6000, NULL);
    CHECK(ob_drivebase_is_done(&db));
    CHECK(fabs((double)(diff_pos(&db) - TURN90_WHEEL_DEG)) < 5.0);
    CHECK(fabs((double)(db.turn_hold - TURN90_WHEEL_DEG)) < 5.0);
}

TEST(blocked_reverse_move_rate_caps_the_integral_growth) {
    // The negative side of the per-tick integral rate cap: a big
    // NEGATIVE error (target behind a blocked robot) grows the
    // integral by at most the cap per tick.
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_straight(&db, 0, -200.0, 150.0, false);
    run_plant_track(&db, 1, 3000, 0.0);
    CHECK(!ob_drivebase_is_done(&db));
    CHECK(db.integ_sum < 0.0);
    CHECK(db.integ_sum >= -(ob_float_t)OB_DRIVEBASE_ACTUATION_MAX_DPS
                          / (ob_float_t)OB_DRIVEBASE_DEFAULT_KI - 1e-6);
}

TEST(gyro_mode_holds_the_absolute_target_across_moves) {
    // The +7.6 deg/square rule at the core: six turn(20)s on a laggy
    // plant with an honest IMU land on the ABSOLUTE 120 body-deg.
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    db.use_gyro = true;
    long now = 0;
    for (int k = 0; k < 6; k++) {
        ob_drivebase_turn(&db, now, 20.0, 60.0);
        for (int t = 0; t < 6000 && !ob_drivebase_is_done(&db); t++) {
            now++;
            ob_drivebase_tick(&db, now);
            l.observer.pos_hat += l.target_dps * 0.6 * 0.001;
            r.observer.pos_hat += r.target_dps * 0.6 * 0.001;
            db.heading_override_wheel_deg = diff_pos(&db);
        }
        CHECK(ob_drivebase_is_done(&db));
        ob_drivebase_stop(&db);
    }
    ob_float_t body = diff_pos(&db) / (136.0 / 88.0);
    CHECK(fabs((double)(body - 120.0)) < 3.0);
}

int main(void) {
    RUN(straight_converges_then_stop_clears_the_move);
    RUN(reverse_straight_and_post_move_hold_bleed_the_integral);
    RUN(straight_with_carry_keeps_the_reference_advancing);
    RUN(turn_is_cw_positive_and_converges);
    RUN(turn_armed_while_translating_ramps_the_forward_axis_down);
    RUN(curve_zero_angle_completes_at_once);
    RUN(curve_zero_radius_turns_in_place_and_ramps_forward_down);
    RUN(curve_arcs_with_proportional_axes);
    RUN(curve_with_carry_keeps_both_axes_moving);
    RUN(laggy_plant_lands_with_a_shaped_landing_and_reports_it);
    RUN(stiction_residual_is_forgiven_at_the_settle_cap);
    RUN(blocked_robot_never_latches_done);
    RUN(trace_dump_is_oldest_first_and_wraps);
    RUN(gyro_frame_reset_and_body_to_wheel_mapping);
    RUN(gyro_mode_arms_straight_and_curve_from_the_held_target);
    RUN(blocked_reverse_move_rate_caps_the_integral_growth);
    RUN(gyro_mode_holds_the_absolute_target_across_moves);
    RUN(at_rest_arms_nothing_and_keeps_the_absolute_hold);
    RUN(zero_accel_arms_nothing);
    RUN(cruise_ramps_to_rest_at_the_accel_limit_without_a_cliff);
    RUN(gyro_counter_steers_through_the_ramp);
    RUN(rotation_decelerates_on_the_turn_accel_and_moves_the_hold);
    RUN(stop_decel_keeps_the_move_diagnostics_but_restarts_the_integral);
    return harness_exit("drivebase_core");
}
