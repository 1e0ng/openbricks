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

int main(void) {
    RUN(at_rest_arms_nothing_and_keeps_the_absolute_hold);
    RUN(zero_accel_arms_nothing);
    RUN(cruise_ramps_to_rest_at_the_accel_limit_without_a_cliff);
    RUN(gyro_counter_steers_through_the_ramp);
    RUN(rotation_decelerates_on_the_turn_accel_and_moves_the_hold);
    RUN(stop_decel_keeps_the_move_diagnostics_but_restarts_the_integral);
    return harness_exit("drivebase_core");
}
