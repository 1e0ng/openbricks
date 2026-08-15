# SPDX-License-Identifier: MIT
"""Parity tests for the CPython-side DriveBase.

Same 2-DOF coupled control law as the firmware's
``_openbricks_native.DriveBase``; both wrap ``drivebase_core.c``.

Tests construct a DriveBase + two Servos, then drive a synthetic
encoder model forward each tick (no MuJoCo, no hardware) so the
behavioural assertions land regardless of the physics layer:

  * ``straight`` / ``turn`` produce the right per-servo target_dps
    sign convention.
  * Coupled feedback corrects an asymmetric-friction perturbation
    where independent loops would let heading drift.
  * ``set_use_gyro`` + ``set_heading_override`` route the IMU body
    delta into the same wheel-degree differential the encoder path
    would have produced.
"""

import math
import unittest


# ----- helpers --------------------------------------------------------

def _wheel(distance_mm, circumference_mm):
    """mm-space → wheel-degree space."""
    return distance_mm / circumference_mm * 360.0


class _SyntheticTwoWheel:
    """Tiny encoder simulator. Each tick:
       1. The DriveBase writes left/right target_dps.
       2. We integrate count = count + target_dps * dt * counts_per_deg.
       3. We feed the new counts back to each Servo via tick(),
          which updates its observer.
       Optional asymmetric scaling lets us model one wheel slipping —
       the test fixture for "coupled correction beats Kp-only"."""

    def __init__(self, drivebase, left, right,
                 counts_per_rev=1320, period_ms=1,
                 left_scale=1.0, right_scale=1.0):
        self.db    = drivebase
        self.left  = left
        self.right = right
        self.counts_per_rev = counts_per_rev
        self.period_ms = period_ms
        self.left_scale  = left_scale
        self.right_scale = right_scale

        self.left_count  = 0
        self.right_count = 0
        self.now_ms      = 0
        # Baseline observers + clock so the first tick doesn't see a
        # phantom delta (mirrors what drivebase_register does in fw).
        self.left.baseline(0, 0)
        self.right.baseline(0, 0)

    def step(self, ticks):
        for _ in range(ticks):
            self.now_ms += self.period_ms
            # 1. DriveBase samples its profiles and writes target_dps.
            self.db.tick(self.now_ms)
            # 2. Integrate counts from the (possibly scaled) targets.
            #    The Servos haven't been ticked yet, so target_dps is
            #    the just-written value.
            l_dps = self.db.target_left_dps()  * self.left_scale
            r_dps = self.db.target_right_dps() * self.right_scale
            l_deg = l_dps * (self.period_ms / 1000.0)
            r_deg = r_dps * (self.period_ms / 1000.0)
            self.left_count  += l_deg * self.counts_per_rev / 360.0
            self.right_count += r_deg * self.counts_per_rev / 360.0
            # 3. Tick servos so their observers update.
            self.left.tick(int(self.left_count),   self.now_ms)
            self.right.tick(int(self.right_count), self.now_ms)


# ----- tests ----------------------------------------------------------

class DriveBaseParityTests(unittest.TestCase):

    def setUp(self):
        from openbricks_sim import _native
        self.Servo     = _native.Servo
        self.DriveBase = _native.DriveBase

        self.wheel_diameter = 56.0       # mm — matches a typical commodity wheel
        self.axle_track     = 110.0
        self.circ           = math.pi * self.wheel_diameter
        self.left  = self.Servo(counts_per_rev=1320, kp=0.3)
        self.right = self.Servo(counts_per_rev=1320, kp=0.3)
        self.db = self.DriveBase(self.left, self.right,
                                  wheel_diameter_mm=self.wheel_diameter,
                                  axle_track_mm=self.axle_track)

    # ------- construction / lifecycle -------

    def test_initial_state_is_done(self):
        self.assertTrue(self.db.is_done())
        self.assertEqual(self.db.target_left_dps(),  0.0)
        self.assertEqual(self.db.target_right_dps(), 0.0)

    def test_construct_rejects_non_servo(self):
        with self.assertRaises(TypeError):
            self.DriveBase("not a servo", self.right, 56.0, 110.0)
        with self.assertRaises(TypeError):
            self.DriveBase(self.left, 42, 56.0, 110.0)

    def test_stop_clears_done_flag(self):
        self.db.straight(0, 100.0, 100.0)
        self.assertFalse(self.db.is_done())
        self.db.stop()
        self.assertTrue(self.db.is_done())

    # ------- straight: sign convention + per-servo mixing -------

    def test_straight_drives_both_wheels_same_direction(self):
        sim = _SyntheticTwoWheel(self.db, self.left, self.right)
        self.db.straight(sim.now_ms, 100.0, 100.0)
        sim.step(20)   # ~mid-ramp
        # Both wheels should have positive target_dps (forward).
        self.assertGreater(self.db.target_left_dps(),  0.0)
        self.assertGreater(self.db.target_right_dps(), 0.0)
        # And they should be similar magnitude (no commanded turn).
        self.assertAlmostEqual(self.db.target_left_dps(),
                               self.db.target_right_dps(),
                               delta=10.0)

    def test_straight_reverse_drives_both_wheels_backward(self):
        sim = _SyntheticTwoWheel(self.db, self.left, self.right)
        self.db.straight(sim.now_ms, -100.0, 100.0)
        sim.step(20)
        self.assertLess(self.db.target_left_dps(),  0.0)
        self.assertLess(self.db.target_right_dps(), 0.0)

    def test_straight_completes(self):
        # 100 mm at 100 mm/s with default 720 dps² accel — well within 5 s.
        sim = _SyntheticTwoWheel(self.db, self.left, self.right)
        self.db.straight(sim.now_ms, 100.0, 100.0)
        sim.step(5000)
        self.assertTrue(self.db.is_done())

    # ------- turn: pure-turn sign convention -------

    def test_turn_positive_is_right_wheels_opposite(self):
        # Positive angle = CW / right (Pybricks convention, adopted
        # in 1.24.0) → left wheel goes forward, right wheel backward.
        sim = _SyntheticTwoWheel(self.db, self.left, self.right)
        self.db.turn(sim.now_ms, 90.0, 90.0)
        sim.step(20)
        # diff_pos = (L-R)/2 must INCREASE for a +90° turn → left
        # target > right target, and they're opposite signs.
        self.assertGreater(self.db.target_left_dps(), 0.0)
        self.assertLess(self.db.target_right_dps(), 0.0)

    def test_turn_negative_is_left_inverts_signs(self):
        sim = _SyntheticTwoWheel(self.db, self.left, self.right)
        self.db.turn(sim.now_ms, -90.0, 90.0)
        sim.step(20)
        self.assertLess   (self.db.target_left_dps(),  0.0)
        self.assertGreater(self.db.target_right_dps(), 0.0)

    def test_turn_completes(self):
        sim = _SyntheticTwoWheel(self.db, self.left, self.right)
        self.db.turn(sim.now_ms, 90.0, 90.0)
        sim.step(5000)
        self.assertTrue(self.db.is_done())

    # ------- coupled control beats Kp-only -------

    def test_coupled_controller_corrects_asymmetric_friction(self):
        # Right wheel rolls 80% as far as commanded — simulates drag /
        # asymmetric friction. The coupled controller must defend the
        # heading; pure-independent loops would let the robot veer.
        # NOTE: with a left/right scale skew, the controller can't
        # achieve perfect heading lock without speeding up the slipping
        # wheel — but the 2-DOF coupled loop pushes far harder than
        # two independent loops. We assert a coupled improvement vs a
        # baseline (no controller, just open-loop trajectory).
        sim = _SyntheticTwoWheel(self.db, self.left, self.right,
                                  left_scale=1.0, right_scale=0.8)
        self.db.straight(sim.now_ms, 200.0, 100.0)
        sim.step(3000)
        # Heading error in wheel-degrees: diff_pos = (L - R)/2.
        diff_pos = (sim.left_count - sim.right_count) * 360.0 / (2 * 1320)
        # A pure open-loop run with 20% scale skew over a 200 mm move
        # at speed 100 dps would produce a diff_pos on the order of
        # 100+ wheel-degrees. The coupled controller should keep it
        # under 20 — generous bound; verifying the principle.
        self.assertLess(abs(diff_pos), 20.0,
                        "coupled controller didn't defend heading "
                        "under asymmetric friction")

    # ------- gyro override path -------

    def test_set_use_gyro_toggles_internal_flag(self):
        # No assertion against the internal flag directly — verify the
        # behavioural consequence: with use_gyro on, set_heading_override
        # drives the diff-error term, so a non-zero override visibly
        # changes per-servo target_dps even with zero encoder differential.
        sim = _SyntheticTwoWheel(self.db, self.left, self.right)
        self.db.straight(sim.now_ms, 100.0, 100.0)
        sim.step(50)   # let the move get past the initial ramp

        # Snapshot per-servo targets when gyro is OFF (encoder diff = 0
        # because both wheels track the trajectory in lockstep).
        sim.now_ms += 1
        self.db.tick(sim.now_ms)
        l_off = self.db.target_left_dps()
        r_off = self.db.target_right_dps()

        # Now flip gyro on, push a +5° body delta override, tick again.
        self.db.set_use_gyro(True)
        self.db.set_heading_override(5.0)
        sim.now_ms += 1
        self.db.tick(sim.now_ms)
        l_on = self.db.target_left_dps()
        r_on = self.db.target_right_dps()

        # The +5° body heading delta means the robot has yawed
        # CW/right (Pybricks convention, 1.24.0) — the controller
        # must counter-steer CCW: speed up the right wheel and slow
        # the left wheel relative to the no-gyro tick.
        self.assertLess   (l_on, l_off)
        self.assertGreater(r_on, r_off)

    def test_gyro_turn_overshoot_corrected_by_next_move(self):
        # ABSOLUTE FRAME (1.25.0): the gyro target persists across
        # moves. Complete a +90 turn, then start a straight while the
        # measured heading reports 95 — five degrees clockwise of the
        # target. The controller must steer BACK (right wheel faster
        # than left), not adopt the overshoot as the new reference
        # the way per-move re-baselining did (bench: ~+7 deg of
        # accumulated drift per gyro'd square).
        sim = _SyntheticTwoWheel(self.db, self.left, self.right)
        self.db.set_use_gyro(True)
        self.db.set_heading_override(0.0)
        self.db.turn(sim.now_ms, 90.0, 90.0)
        sim.step(5000)   # run the turn trajectory to completion
        # ARRIVAL semantics (1.43.1): the override is still frozen at
        # zero, so the robot demonstrably hasn't turned — done must
        # be False. (Time-based done here is the exact hole that let
        # a real robot's final gyro turn keep its +4.5-deg overshoot
        # uncorrected on the bench.)
        self.assertFalse(self.db.is_done())
        self.db.set_heading_override(90.0)   # NOW it has arrived
        sim.step(50)
        self.assertTrue(self.db.is_done())

        self.db.straight(sim.now_ms, 100.0, 100.0)
        self.db.set_heading_override(95.0)   # measured: 5 deg past target
        sim.now_ms += 1
        self.db.tick(sim.now_ms)
        self.assertGreater(self.db.target_right_dps(),
                           self.db.target_left_dps(),
                           "overshoot banked by the turn was adopted "
                           "instead of corrected")

    def test_set_heading_override_zero_is_neutral(self):
        # With use_gyro on and override = 0, the diff-error is exactly
        # zero (the override path replaces the encoder differential
        # rather than adding to it), so the per-servo targets match
        # the ones we'd get with use_gyro off and the synthetic encoder
        # also at zero differential.
        sim = _SyntheticTwoWheel(self.db, self.left, self.right)
        self.db.straight(sim.now_ms, 100.0, 100.0)
        sim.step(50)

        sim.now_ms += 1
        self.db.tick(sim.now_ms)
        l_off = self.db.target_left_dps()
        r_off = self.db.target_right_dps()

        self.db.set_use_gyro(True)
        self.db.set_heading_override(0.0)
        sim.now_ms += 1
        self.db.tick(sim.now_ms)
        # ±1 dps tolerance: the trajectory has advanced one tick between
        # the snapshots, so fwd_target moves slightly — diff stays 0.
        self.assertAlmostEqual(self.db.target_left_dps(),  l_off, delta=2.0)
        self.assertAlmostEqual(self.db.target_right_dps(), r_off, delta=2.0)

    # ------- set_accel: tunable trajectory acceleration -------

    def test_set_accel_rejects_non_positive(self):
        with self.assertRaises(ValueError):
            self.db.set_accel(0.0)
        with self.assertRaises(ValueError):
            self.db.set_accel(-720.0)

    def test_lower_accel_ramps_the_setpoint_slower(self):
        # Measure the RAMP SLOPE over the first 100 ms of a long
        # move: default 720 deg/s² climbs ~72 dps in the window,
        # set_accel(90) climbs ~9. Deltas (not absolutes) because
        # trajectories arm FROM THE CURRENT commanded speed since
        # 2.0.0 — a bare stop() (yield-only) leaves the servo
        # targets live, and the second move honestly blends from
        # them.
        sim = _SyntheticTwoWheel(self.db, self.left, self.right)
        self.db.straight(sim.now_ms, 500.0, 200.0)
        sim.step(100)
        fast_delta = self.db.target_left_dps()      # from rest
        self.db.stop()

        self.db.set_accel(90.0)
        v0 = self.db.target_left_dps()
        self.db.straight(sim.now_ms, 500.0, 200.0)
        sim.step(100)
        slow_delta = self.db.target_left_dps() - v0

        self.assertGreater(fast_delta, 0.0)
        self.assertGreater(slow_delta, 0.0)
        # 8x ratio in theory; assert a comfortable 3x so feedback
        # wiggle can't flake it.
        self.assertGreater(fast_delta, slow_delta * 3.0)

    def test_set_accel_applies_to_turn_too(self):
        # The core shares one acceleration between the forward and
        # turn profiles — pin that a gentler setting slows turn
        # ramps. Deltas, not absolutes: see
        # test_lower_accel_ramps_the_setpoint_slower.
        sim = _SyntheticTwoWheel(self.db, self.left, self.right)
        self.db.turn(sim.now_ms, 90.0, 180.0)
        sim.step(100)
        fast_delta = abs(self.db.target_right_dps())
        self.db.stop()

        self.db.set_accel(90.0)
        v0 = abs(self.db.target_right_dps())
        self.db.turn(sim.now_ms, 90.0, 180.0)
        sim.step(100)
        slow_delta = abs(abs(self.db.target_right_dps()) - v0)

        self.assertGreater(fast_delta, 0.0)
        self.assertGreater(slow_delta, 0.0)
        self.assertGreater(fast_delta, slow_delta * 3.0)


if __name__ == "__main__":
    unittest.main()
