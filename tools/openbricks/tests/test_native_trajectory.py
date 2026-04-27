# SPDX-License-Identifier: MIT
"""Parity test: the CPython ``openbricks_sim._native.TrapezoidalProfile``
produces the same outputs as the firmware ``_openbricks_native``
module (both wrap the shared ``trajectory_core.c``).

The firmware side is exercised by the existing
``tests/test_trajectory.py`` under unix MicroPython. Here on the
CPython side we re-run the same shape of checks against a handful of
well-defined profiles and assert the numbers match what the math
predicts. When both surfaces agree with the closed-form expectations
they agree with each other.
"""

import math
import unittest


class TrajectoryParityTests(unittest.TestCase):
    """Run known-answer profiles through the CPython extension and
    check every output against the trapezoid-math closed form."""

    def setUp(self):
        # Import here so the test module still loads even if the
        # extension failed to build — the assertions below will surface
        # the problem clearly.
        from openbricks_sim import _native
        self.Profile = _native.TrapezoidalProfile

    # ---------- trapezoidal (reaches cruise) ----------

    def test_trapezoidal_180deg_90dps_180dps2(self):
        p = self.Profile(start=0.0, target=180.0,
                         cruise_dps=90.0, accel_dps2=180.0)
        # ramp 0.5 s, ramp dist 22.5°, cruise for (180-45)/90 = 1.5 s,
        # total 2.5 s
        self.assertFalse(p.is_triangular())
        self.assertAlmostEqual(p.duration(), 2.5, places=6)
        self.assertEqual(p.sample(0.0)[0], 0.0)
        # End of accel phase: t=0.5, pos=22.5, vel=90
        pos, vel = p.sample(0.5)
        self.assertAlmostEqual(pos, 22.5, places=5)
        self.assertAlmostEqual(vel, 90.0, places=5)
        # Middle of cruise: t=1.25, pos=22.5 + 90*0.75 = 90, vel=90
        pos, vel = p.sample(1.25)
        self.assertAlmostEqual(pos, 90.0, places=5)
        self.assertAlmostEqual(vel, 90.0, places=5)
        # End: t=2.5
        pos, vel = p.sample(2.5)
        self.assertAlmostEqual(pos, 180.0, places=5)
        self.assertAlmostEqual(vel, 0.0, places=5)
        # Past the end holds.
        pos, vel = p.sample(1e6)
        self.assertAlmostEqual(pos, 180.0, places=5)
        self.assertAlmostEqual(vel, 0.0, places=5)

    # ---------- triangular (too short to cruise) ----------

    def test_triangular_short_move(self):
        # Cruise 90 dps would need 45° to ramp up + 45° down = 90°.
        # Ask for only 40° → triangular.
        p = self.Profile(start=0.0, target=40.0,
                         cruise_dps=90.0, accel_dps2=180.0)
        self.assertTrue(p.is_triangular())
        # v_peak = sqrt(40 * 180) = sqrt(7200) ≈ 84.853
        # t_peak = v_peak / 180 ≈ 0.4714
        # duration = 2 * t_peak ≈ 0.9428
        self.assertAlmostEqual(p.duration(), 2 * math.sqrt(40.0 / 180.0),
                               places=5)
        v_peak_expected = math.sqrt(40.0 * 180.0)
        pos, vel = p.sample(p.duration() / 2.0)
        self.assertAlmostEqual(pos, 20.0, places=5)
        self.assertAlmostEqual(vel, v_peak_expected, places=4)
        pos, vel = p.sample(p.duration())
        self.assertAlmostEqual(pos, 40.0, places=5)
        self.assertAlmostEqual(vel, 0.0, places=5)

    # ---------- signed / negative moves ----------

    def test_negative_move_flips_signs(self):
        p = self.Profile(start=100.0, target=-50.0,
                         cruise_dps=60.0, accel_dps2=120.0)
        # Distance = -150°, all velocities / positions should go negative.
        self.assertFalse(p.is_triangular())
        pos_start, vel_start = p.sample(0.0)
        self.assertAlmostEqual(pos_start, 100.0, places=6)
        self.assertAlmostEqual(vel_start, 0.0, places=6)
        # Middle of motion: velocity should be negative.
        pos_mid, vel_mid = p.sample(p.duration() / 2.0)
        self.assertLess(vel_mid, 0.0)
        # End.
        pos_end, vel_end = p.sample(p.duration())
        self.assertAlmostEqual(pos_end, -50.0, places=5)
        self.assertAlmostEqual(vel_end, 0.0, places=5)

    # ---------- degenerate inputs ----------

    def test_zero_distance_is_immediately_done(self):
        p = self.Profile(start=30.0, target=30.0,
                         cruise_dps=10.0, accel_dps2=20.0)
        self.assertEqual(p.duration(), 0.0)
        pos, vel = p.sample(0.0)
        self.assertEqual(pos, 30.0)
        self.assertEqual(vel, 0.0)

    def test_zero_cruise_is_degenerate(self):
        p = self.Profile(start=0.0, target=90.0,
                         cruise_dps=0.0, accel_dps2=20.0)
        self.assertEqual(p.duration(), 0.0)

    def test_zero_accel_is_degenerate(self):
        p = self.Profile(start=0.0, target=90.0,
                         cruise_dps=20.0, accel_dps2=0.0)
        self.assertEqual(p.duration(), 0.0)

    # ---------- sample clamping ----------

    def test_sample_before_zero_clamps_to_start(self):
        p = self.Profile(start=10.0, target=100.0,
                         cruise_dps=30.0, accel_dps2=60.0)
        pos, vel = p.sample(-1.0)
        self.assertAlmostEqual(pos, 10.0, places=6)
        self.assertAlmostEqual(vel, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
