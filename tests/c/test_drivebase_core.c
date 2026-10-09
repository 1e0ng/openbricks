// SPDX-License-Identifier: MIT
// Native tests for drivebase_core's decelerating stop
// (ob_drivebase_stop_decel): the controlled half of a brake/hold,
// with both axes closed-loop through the ramp.

#include <math.h>
// -std=c11 hides M_PI on GCC (drivebase_core.c carries the same guard).
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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


// ---- a stop lands when the robot has stopped (3.10.1) ---------------

TEST(stop_lands_when_the_wheels_rest_whatever_the_residual) {
    // Competition trap (2026-09-08): a brake that stops short of its
    // v0²/2a landing point — duty-mode stiction, a wheel against a
    // wall — is DONE when the robot has stopped: not after the 400 ms
    // settle cap, and not never (a residual past the forgive limit).
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    l.target_dps = r.target_dps = 300.0;
    CHECK(ob_drivebase_stop_decel(&db, 0, 400.0));
    CHECK(db.stopping);
    // The ramp 300 -> 0 at 400 takes 750 ms. Track it for 400 ms,
    // then the plant FREEZES: ~24 wheel-deg short, twice the forgive
    // limit — a 3.2.0–3.10.0 core never latched done here.
    run_plant_track(&db, 1, 400, 1.0);
    CHECK(!ob_drivebase_is_done(&db));
    int ms_to_done = -1;
    for (int i = 0; i < 2000; i++) {
        ob_drivebase_tick(&db, 401 + i);       // frozen plant: at rest
        if (ob_drivebase_is_done(&db)) {
            ms_to_done = i;
            break;
        }
    }
    CHECK(ms_to_done >= 0);
    CHECK(ms_to_done < 420);                   // the ramp's remainder + a beat
    CHECK(!db.landing_active);                 // a stop never re-arms a landing
    CHECK_EQ_INT(db.landings, 0);
    ob_float_t landing = db.fwd.start
                         + db.fwd.direction * fabs((double)db.fwd.distance);
    CHECK(fabs((double)(sum_pos(&db) - landing))
          > OB_DRIVEBASE_SETTLE_FORGIVE_WHEEL_DEG);
}

TEST(a_move_stopped_short_past_the_forgive_limit_stays_not_done) {
    // The stop rule is a STOP rule: a move whose plant freezes is a
    // move that didn't happen — done stays false (the caller's
    // watchdog names it) and it still tries its landing.
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_straight(&db, 0, 200.0, 150.0, false);
    CHECK(!db.stopping);
    run_plant_track(&db, 1, 300, 1.0);
    run_plant_track(&db, 301, 3000, 0.0);      // frozen
    CHECK(!ob_drivebase_is_done(&db));
    CHECK(db.landings > 0);
}

TEST(stopping_is_cleared_by_the_yield_by_arms_and_by_a_stop_at_rest) {
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    l.target_dps = r.target_dps = 150.0;
    CHECK(ob_drivebase_stop_decel(&db, 0, 400.0));
    CHECK(db.stopping);
    ob_drivebase_stop(&db);                    // the yield
    CHECK(!db.stopping);
    CHECK(ob_drivebase_stop_decel(&db, 0, 400.0));
    CHECK(db.stopping);
    ob_drivebase_straight(&db, 10, 100.0, 150.0, false);
    CHECK(!db.stopping);
    l.target_dps = r.target_dps = 150.0;
    CHECK(ob_drivebase_stop_decel(&db, 12, 400.0));
    CHECK(db.stopping);
    ob_drivebase_curve(&db, 14, 0.0, 90.0, 100.0, false);   // radius 0
    CHECK(!db.stopping);
    CHECK(ob_drivebase_stop_decel(&db, 16, 400.0));
    ob_drivebase_curve(&db, 18, 150.0, 90.0, 100.0, false);
    CHECK(!db.stopping);
    CHECK(ob_drivebase_stop_decel(&db, 19, 400.0));
    ob_drivebase_turn(&db, 19, 90.0, 60.0);
    CHECK(!db.stopping);
    l.target_dps = r.target_dps = 0.0;
    db.integ_sum = db.integ_diff = 0.0;
    CHECK(!ob_drivebase_stop_decel(&db, 20, 400.0));   // at rest: nothing armed
    CHECK(!db.stopping);
    CHECK(ob_drivebase_is_done(&db));
}


// ---- reference profile arithmetic (trajectory_core) -------------------

// Largest |velocity| and largest one-step position jump over a dense
// scan of the profile (4000 samples up to t_total).
static void scan_profile(const ob_trajectory_t *t, ob_float_t *max_vel,
                         ob_float_t *max_jump) {
    ob_float_t prev_pos, v;
    ob_trajectory_sample(t, 0.0, &prev_pos, &v);
    *max_vel = 0.0;
    *max_jump = 0.0;
    for (int k = 1; k <= 4000; k++) {
        ob_float_t p;
        ob_trajectory_sample(t, t->t_total * k / 4000.0, &p, &v);
        if (fabs((double)v) > *max_vel) {
            *max_vel = fabs((double)v);
        }
        if (fabs((double)(p - prev_pos)) > *max_jump) {
            *max_jump = fabs((double)(p - prev_pos));
        }
        prev_pos = p;
    }
}

TEST(reverse_entry_faster_than_cruise_lands_on_target) {
    // Entry moving the WRONG way faster than cruise (a straight()
    // armed against an opposite drive()): the entry ramp v0 -> vc at
    // +a covers (vc² - v0²)/2a, which is NEGATIVE here. v0 = -400,
    // cruise 200, accel 1500, D = 500:
    //   d_entry  = (200² - 400²) / 3000   = -40
    //   d_exit   =  200² / 3000           = 13.333
    //   t_entry  = (200 + 400) / 1500     = 0.4
    //   t_cruise = (500 + 40 - 13.333)/200 = 2.633333
    //   t_total  = 0.4 + 2.633333 + 0.133333 = 3.166667
    // The sign-flipped d_entry (+40) gave t_cruise 2.233333 and a
    // reference 80 short at t_total⁻ that snapped to 500 at expiry.
    ob_trajectory_t t;
    ob_float_t p, v, max_vel, max_jump;
    ob_trajectory_init_v0(&t, 0.0, 500.0, 200.0, 1500.0, -400.0);
    CHECK(!t.triangular);
    CHECK(fabs((double)(t.d_entry + 40.0)) < 1e-9);
    CHECK(fabs((double)(t.t_cruise - 2.633333333)) < 1e-6);
    CHECK(fabs((double)(t.t_total - 3.166666667)) < 1e-6);
    ob_trajectory_sample(&t, t.t_total - 1e-9, &p, &v);
    CHECK(fabs((double)(p - 500.0)) < 1e-6);
    CHECK(fabs((double)v) < 1e-5);
    scan_profile(&t, &max_vel, &max_jump);
    CHECK(max_vel <= 400.0 + 1e-9);
    CHECK(max_jump < 400.0 * t.t_total / 4000.0 + 1e-6);

    // Short move, D = 20 < v0²/2a = 53.3: the signed ramps fit
    // (-40 + 13.333 <= 20), so it is a trapezoid at cruise with
    // t_cruise = (20 + 40 - 13.333)/200 = 0.233333. The flipped sum
    // (53.3 > 20) sent it triangular with v_peak sqrt(110000) =
    // 331.7 — 166% of the commanded cruise.
    ob_trajectory_init_v0(&t, 0.0, 20.0, 200.0, 1500.0, -400.0);
    CHECK(!t.triangular);
    CHECK(t.v_peak == 200.0);
    CHECK(fabs((double)(t.t_cruise - 0.233333333)) < 1e-6);
    ob_trajectory_sample(&t, t.t_total - 1e-9, &p, &v);
    CHECK(fabs((double)(p - 20.0)) < 1e-6);
    scan_profile(&t, &max_vel, &max_jump);
    CHECK(max_vel <= 400.0 + 1e-9);

    // Shorter still and triangular for real: v0 = -250, D = 5,
    // aD + v0²/2 = 38750 < vc² = 40000, so vp = sqrt(38750) =
    // 196.85 <= cruise, d_entry = (38750 - 62500)/3000 = -7.9167,
    // d_ramp = 38750/3000 = 12.9167, summing to exactly 5.
    ob_trajectory_init_v0(&t, 0.0, 5.0, 200.0, 1500.0, -250.0);
    CHECK(t.triangular);
    CHECK(fabs((double)(t.v_peak - sqrt(38750.0))) < 1e-9);
    CHECK(t.v_peak <= 200.0);
    CHECK(fabs((double)(t.d_entry + 7.916666667)) < 1e-6);
    ob_trajectory_sample(&t, t.t_total - 1e-9, &p, &v);
    CHECK(fabs((double)(p - 5.0)) < 1e-6);

    // Same-direction fast entry is untouched (2.7.3): v0 = 350,
    // cruise 200, accel 800, D = 500 -> d_entry = +51.5625.
    ob_trajectory_init_v0(&t, 0.0, 500.0, 200.0, 800.0, 350.0);
    CHECK(!t.triangular);
    CHECK(fabs((double)(t.d_entry - 51.5625)) < 1e-9);
    ob_trajectory_sample(&t, t.t_total - 1e-9, &p, &v);
    CHECK(fabs((double)(p - 500.0)) < 1e-6);
}

TEST(straight_against_an_opposite_drive_has_no_step_at_expiry) {
    // drive(200, 0) leaves both wheels at +260 wheel-dps; straight
    // (-300 mm at 150 mm/s = cruise 195.3) at accel 400 enters at
    // v0 = -260. Pre-fix the reference sat 74.2 wheel-deg (57 mm)
    // behind the whole move and snapped forward at expiry.
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_tick(&db, 999);
    l.target_dps = r.target_dps = 260.0;
    ob_drivebase_straight(&db, 1000, -300.0, 150.0, false);
    ob_float_t p0, p1, v;
    ob_trajectory_sample(&db.fwd, db.fwd.t_total - 1e-9, &p0, &v);
    ob_trajectory_sample(&db.fwd, db.fwd.t_total, &p1, &v);
    CHECK(fabs((double)(p1 - p0)) < 1e-6);
    CHECK(db.fwd.v_peak <= db.fwd.cruise);
    ob_float_t max_step = 0.0;
    run_plant(&db, 1000, 6000, &max_step);
    CHECK(ob_drivebase_is_done(&db));
    CHECK(max_step < 2.0);                     // no one-tick cliff
    CHECK_EQ_INT(db.landings, 0);              // a perfect plant needs none
    // The spin twin: turn() against an opposing rotation.
    setup(&db, &l, &r);
    ob_drivebase_tick(&db, 999);
    l.target_dps = -260.0;
    r.target_dps = 260.0;
    ob_drivebase_turn(&db, 1000, 60.0, 60.0);
    ob_trajectory_sample(&db.turn, db.turn.t_total - 1e-9, &p0, &v);
    ob_trajectory_sample(&db.turn, db.turn.t_total, &p1, &v);
    CHECK(fabs((double)(p1 - p0)) < 1e-6);
    max_step = 0.0;
    run_plant(&db, 1000, 6000, &max_step);
    CHECK(ob_drivebase_is_done(&db));
    CHECK(max_step < 2.0);
}

TEST(carry_shorter_than_its_ramp_ends_at_the_reachable_speed) {
    // then=Stop.NONE over a distance too short to reach the carried
    // speed: speeding up from v0 to v3 covers (v3² - v0²)/2a, so the
    // end speed is clamped to v3 = sqrt(v0² + 2aD) and the profile is
    // one pure acceleration ending on target at v3. From rest, D = 1,
    // cruise/carry 200, accel 1500: v3 = sqrt(3000) = 54.772,
    // t_total = 54.772/1500 = 0.036515. Pre-fix: v_peak 146.6,
    // t_ramp -0.0356, reference 2.90 at t_total⁻ (93.3 dps) snapping
    // back to 1.0 and the FF stepping up to 200.
    ob_trajectory_t t;
    ob_float_t p, v;
    ob_trajectory_init_v0v3(&t, 0.0, 1.0, 200.0, 1500.0, 0.0, 200.0);
    CHECK(fabs((double)(t.v3 - sqrt(3000.0))) < 1e-9);
    CHECK(t.t_ramp >= 0.0);
    CHECK(t.t_ramp < 1e-9);
    CHECK(fabs((double)(t.t_total - sqrt(3000.0) / 1500.0)) < 1e-9);
    ob_trajectory_sample(&t, t.t_total - 1e-9, &p, &v);
    CHECK(fabs((double)(p - 1.0)) < 1e-6);
    CHECK(fabs((double)(v - t.v3)) < 1e-5);
    ob_trajectory_sample(&t, t.t_total, &p, &v);
    CHECK(p == 1.0);
    CHECK(v == t.v3);
    // Moving entry: v0 = 100, carry 350, D = 10 -> reach 37.5 > 10,
    // v3 = sqrt(100² + 2·1500·10) = 200, t_total = 100/1500 = 0.066667.
    ob_trajectory_init_v0v3(&t, 0.0, 10.0, 350.0, 1500.0, 100.0, 350.0);
    CHECK(fabs((double)(t.v3 - 200.0)) < 1e-9);
    CHECK(t.t_ramp >= 0.0);
    CHECK(fabs((double)(t.t_total - 0.0666666667)) < 1e-9);
    ob_trajectory_sample(&t, t.t_total - 1e-9, &p, &v);
    CHECK(fabs((double)(p - 10.0)) < 1e-6);
    CHECK(fabs((double)(v - 200.0)) < 1e-5);
    // A carry long enough for its ramp keeps the requested end speed.
    ob_trajectory_init_v0v3(&t, 0.0, 20.0, 200.0, 1500.0, 0.0, 200.0);
    CHECK(t.v3 == 200.0);
}

TEST(short_carry_straight_and_curve_hand_over_without_a_step) {
    // 1 mm carry straight from rest = 1.302 wheel-deg at accel 400:
    // v3 = sqrt(2·400·1.302) = 32.3 wheel-dps, carried on continuously.
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_tick(&db, 999);
    ob_drivebase_straight(&db, 1000, 1.0, 150.0, true);
    CHECK(db.fwd.t_ramp >= 0.0);
    CHECK(fabs((double)(db.fwd.v3
                        - sqrt(2.0 * 400.0 * 360.0 / (88.0 * M_PI)))) < 1e-6);
    ob_float_t max_step = 0.0;
    run_plant(&db, 1000, 400, &max_step);
    CHECK(max_step < 2.0);
    CHECK(db.fwd_active);
    CHECK(fabs((double)(l.target_dps - db.fwd.v3)) < 2.0);
    // Short carry arc: both axes clamp proportionally.
    setup(&db, &l, &r);
    ob_drivebase_tick(&db, 999);
    ob_drivebase_curve(&db, 1000, 20.0, 5.0, 150.0, true);
    CHECK(db.fwd.t_ramp >= 0.0 && db.turn.t_ramp >= 0.0);
    CHECK(fabs((double)(db.fwd.t_total - db.turn.t_total)) < 1e-9);
    max_step = 0.0;
    run_plant(&db, 1000, 400, &max_step);
    CHECK(max_step < 2.0);
}

TEST(turn_in_place_after_a_brake_is_a_move_not_a_stop) {
    // curve(radius=0) after a ramped brake must clear the stop flag
    // like every other arm: blocked 20 wheel-deg short (past the
    // forgive limit) it stays not done and tries its landing,
    // exactly like turn().
    ob_drivebase_t db; ob_servo_t l, r;
    setup(&db, &l, &r);
    ob_drivebase_tick(&db, 0);
    l.target_dps = r.target_dps = 300.0;
    CHECK(ob_drivebase_stop_decel(&db, 1, 400.0));
    run_plant_track(&db, 1, 2000, 1.0);
    CHECK(ob_drivebase_is_done(&db));
    CHECK(db.stopping);                        // a landed stop keeps it
    ob_drivebase_curve(&db, 2001, 0.0, 90.0, 100.0, false);
    CHECK(!db.stopping);
    for (int t = 2001; t < 8000; t++) {
        ob_drivebase_tick(&db, t);
        ob_float_t track = (diff_pos(&db) < TURN90_WHEEL_DEG - 20.0)
                           ? 1.0 : 0.0;
        l.observer.pos_hat += l.target_dps * track * 0.001;
        r.observer.pos_hat += r.target_dps * track * 0.001;
    }
    CHECK(!ob_drivebase_is_done(&db));
    CHECK(db.landings > 0);
}

int main(void) {
    RUN(stop_lands_when_the_wheels_rest_whatever_the_residual);
    RUN(a_move_stopped_short_past_the_forgive_limit_stays_not_done);
    RUN(stopping_is_cleared_by_the_yield_by_arms_and_by_a_stop_at_rest);
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
    RUN(reverse_entry_faster_than_cruise_lands_on_target);
    RUN(straight_against_an_opposite_drive_has_no_step_at_expiry);
    RUN(carry_shorter_than_its_ramp_ends_at_the_reachable_speed);
    RUN(short_carry_straight_and_curve_hand_over_without_a_step);
    RUN(turn_in_place_after_a_brake_is_a_move_not_a_stop);
    return harness_exit("drivebase_core");
}
