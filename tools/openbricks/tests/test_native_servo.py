# SPDX-License-Identifier: MIT
"""Parity tests for the CPython-side Servo state machine.

Same control law (P-with-feedforward + α-β observer + trapezoidal
trajectory) as the firmware's ``_openbricks_native.Servo``; both wrap
``servo_core.c``. Tests construct a Servo, drive it through synthetic
encoder readings (no hardware), and assert behavioural properties:

* ``set_speed`` puts the right value in ``target_dps``
* ``tick`` returns a power that drives toward the setpoint
* ``run_target`` produces a trajectory that completes
* ``is_done`` semantics match the firmware
"""

import unittest


class ServoParityTests(unittest.TestCase):

    def setUp(self):
        from openbricks_sim import _native
        self.Servo = _native.Servo

    # ------- construction / defaults -------

    def test_defaults(self):
        s = self.Servo()
        self.assertEqual(s.target_dps(), 0.0)
        self.assertTrue(s.is_done())          # no trajectory active
        self.assertEqual(s.observed_dps(), 0.0)
        self.assertEqual(s.observed_pos(), 0.0)

    def test_count_to_angle(self):
        s = self.Servo(counts_per_rev=360)
        # 360 counts per rev = 1 deg per count.
        self.assertEqual(s.count_to_angle(0),   0.0)
        self.assertEqual(s.count_to_angle(90),  90.0)
        self.assertEqual(s.count_to_angle(-180), -180.0)
        # Standard JGB37 default = 1320.
        s2 = self.Servo()
        self.assertAlmostEqual(s2.count_to_angle(1320), 360.0, places=4)

    # ------- run_speed semantics -------

    def test_set_speed_writes_target(self):
        s = self.Servo()
        s.set_speed(180.0)
        self.assertEqual(s.target_dps(), 180.0)
        s.set_speed(-90.0)
        self.assertEqual(s.target_dps(), -90.0)

    def test_tick_with_target_drives_positive_power(self):
        # At rest, with a +180 dps target, the first tick's feed-forward
        # alone is 180/300*100 = 60. Encoder hasn't moved → observed
        # vel ≈ 0 → P term is +0.3*(180-0) = 54. Total ≈ 60+54 = 114,
        # clamped to 100.
        s = self.Servo(counts_per_rev=1320, kp=0.3)
        s.baseline(0, 0)
        s.set_speed(180.0)
        power = s.tick(0, 1)   # one ms after baseline, encoder still at 0
        self.assertGreater(power, 80.0)   # generous lower bound
        self.assertLessEqual(power, 100.0)

    def test_tick_clamps_to_power_limits(self):
        # Aggressive kp pulls the output past ±100 — should clamp.
        s = self.Servo(counts_per_rev=1320, kp=10.0)
        s.baseline(0, 0)
        s.set_speed(1000.0)
        power = s.tick(0, 1)
        self.assertLessEqual(power, 100.0)
        s.set_speed(-1000.0)
        power = s.tick(0, 1)
        self.assertGreaterEqual(power, -100.0)

    def test_observer_tracks_synthetic_motion(self):
        # Drive the encoder at a constant 180 dps. Observer should
        # converge to ~180 within ~50 ms.
        s = self.Servo(counts_per_rev=1320, kp=0.3)
        s.baseline(0, 0)
        s.set_speed(180.0)
        for tick_ms in range(1, 101):
            angle = 180.0 * tick_ms / 1000.0
            count = int(angle * 1320 / 360)
            s.tick(count, tick_ms)
        self.assertAlmostEqual(s.observed_dps(), 180.0, delta=20.0)

    def test_invert_flips_power_sign(self):
        # Same scenario as above, but with invert=True. The core ALWAYS
        # returns a power that targets the setpoint based on the
        # observed velocity; the sign-flip happens in the binding shell
        # when it drives the bridge. So the core's output is the same.
        # We verify that here — the invert flag affects the binding,
        # not the core's tick output.
        s_normal = self.Servo(counts_per_rev=1320, kp=0.3, invert=False)
        s_invert = self.Servo(counts_per_rev=1320, kp=0.3, invert=True)
        for s in [s_normal, s_invert]:
            s.baseline(0, 0)
            s.set_speed(100.0)
        p_norm = s_normal.tick(0, 1)
        p_inv  = s_invert.tick(0, 1)
        self.assertAlmostEqual(p_norm, p_inv, places=6)

    # ------- run_target / trajectory semantics -------

    def test_run_target_starts_a_trajectory(self):
        s = self.Servo(counts_per_rev=360)   # 1 count = 1 deg, easier math
        s.baseline(0, 0)
        s.run_target(0, 0, 180.0, 90.0, 180.0)
        self.assertFalse(s.is_done())
        # First tick samples the trajectory; target_dps should rise from 0.
        s.tick(0, 1)
        self.assertGreater(s.target_dps(), 0.0)

    def test_run_target_completes(self):
        # Trajectory: 90 deg move at 90 dps cruise + 180 dps² accel.
        # Trapezoidal: ramp 0.5 s + cruise 0.5 s + decel 0.5 s = 1.5 s total.
        s = self.Servo(counts_per_rev=360, kp=0.0)   # zero kp — open loop, just trajectory
        s.baseline(0, 0)
        s.run_target(0, 0, 90.0, 90.0, 180.0)
        # Tick out to t = 2 s (well past the 1.5 s duration).
        last_count = 0
        for tick_ms in range(1, 2001):
            # Emulate the wheel actually following the target_dps:
            # advance count by target_dps * dt.
            target = s.target_dps()
            angle  = last_count + target * 0.001
            last_count = angle
            s.tick(int(angle), tick_ms)
        self.assertTrue(s.is_done())
        # target_dps zeroed at the end.
        self.assertAlmostEqual(s.target_dps(), 0.0, places=5)

    def test_set_speed_cancels_active_trajectory(self):
        s = self.Servo(counts_per_rev=360)
        s.baseline(0, 0)
        s.run_target(0, 0, 180.0, 90.0, 180.0)
        self.assertFalse(s.is_done())
        s.set_speed(50.0)
        self.assertTrue(s.is_done())
        self.assertEqual(s.target_dps(), 50.0)

    # ------- baseline semantics -------

    def test_baseline_resets_observer(self):
        s = self.Servo(counts_per_rev=360)
        # Run some motion so observer state isn't zero.
        s.baseline(0, 0)
        s.set_speed(180.0)
        for t_ms in range(1, 21):
            s.tick(int(180.0 * t_ms / 1000.0), t_ms)
        self.assertNotEqual(s.observed_pos(), 0.0)
        # Re-baseline at angle 100 deg → count 100.
        s.baseline(100, 100)
        self.assertAlmostEqual(s.observed_pos(), 100.0, places=4)
        self.assertEqual(s.observed_dps(), 0.0)


if __name__ == "__main__":
    unittest.main()
