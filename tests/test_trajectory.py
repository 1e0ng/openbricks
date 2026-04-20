# SPDX-License-Identifier: MIT
"""Tests for ``_openbricks_native.TrapezoidalProfile``.

Exercises the Python fake in ``tests/_fakes.py``; the C implementation
in ``native/user_c_modules/openbricks/trajectory.c`` must produce
identical samples (the on-device suite in M2b runs the same assertions
against the C module via a mac-runner-plumbed ESP32)."""

import tests._fakes  # noqa: F401

import math
import unittest

from openbricks._native import TrapezoidalProfile


class TestTrapezoidalProfile(unittest.TestCase):
    def test_symmetric_profile(self):
        # distance 100, cruise 50 dps, accel 100 dps^2 -> trapezoidal:
        # t_ramp=0.5s, d_ramp=12.5 deg, t_cruise=1.5s, t_total=2.5s.
        t = TrapezoidalProfile(start=0, target=100, cruise_dps=50, accel_dps2=100)
        self.assertAlmostEqual(t.duration(), 2.5, places=6)
        self.assertFalse(t.is_triangular())

        pos, vel = t.sample(0.0)
        self.assertAlmostEqual(pos, 0.0, places=6)
        self.assertAlmostEqual(vel, 0.0, places=6)

        pos, vel = t.sample(0.25)   # mid-accel
        self.assertAlmostEqual(pos, 3.125, places=6)
        self.assertAlmostEqual(vel, 25.0, places=6)

        pos, vel = t.sample(0.5)    # end of accel
        self.assertAlmostEqual(pos, 12.5, places=6)
        self.assertAlmostEqual(vel, 50.0, places=6)

        pos, vel = t.sample(1.0)    # mid-cruise
        self.assertAlmostEqual(pos, 37.5, places=6)
        self.assertAlmostEqual(vel, 50.0, places=6)

        pos, vel = t.sample(2.0)    # start of decel
        self.assertAlmostEqual(pos, 87.5, places=6)
        self.assertAlmostEqual(vel, 50.0, places=6)

        pos, vel = t.sample(2.5)    # end
        self.assertAlmostEqual(pos, 100.0, places=6)
        self.assertAlmostEqual(vel, 0.0, places=6)

    def test_triangular_profile(self):
        # 5 deg at 50 dps cruise is unreachable with 100 dps^2 accel:
        # v_peak = sqrt(5 * 100) = ~22.36, t_total = 2 * sqrt(0.05) = ~0.4472.
        t = TrapezoidalProfile(start=0, target=5, cruise_dps=50, accel_dps2=100)
        self.assertTrue(t.is_triangular())
        self.assertAlmostEqual(t.duration(), 2 * math.sqrt(0.05), places=6)

        pos, vel = t.sample(t.duration() / 2)   # peak
        self.assertAlmostEqual(pos, 2.5, places=3)
        self.assertAlmostEqual(vel, math.sqrt(500), places=3)

        pos, vel = t.sample(t.duration())       # done
        self.assertAlmostEqual(pos, 5.0, places=6)
        self.assertAlmostEqual(vel, 0.0, places=6)

    def test_zero_distance(self):
        t = TrapezoidalProfile(start=7, target=7, cruise_dps=50, accel_dps2=100)
        self.assertEqual(t.duration(), 0.0)
        pos, vel = t.sample(0.0)
        self.assertAlmostEqual(pos, 7.0, places=6)
        self.assertAlmostEqual(vel, 0.0, places=6)

    def test_negative_distance_flips_signs(self):
        t = TrapezoidalProfile(start=100, target=0, cruise_dps=50, accel_dps2=100)
        self.assertAlmostEqual(t.duration(), 2.5, places=6)
        pos, vel = t.sample(1.25)   # mid
        self.assertAlmostEqual(pos, 50.0, places=6)
        self.assertAlmostEqual(vel, -50.0, places=6)
        pos, vel = t.sample(2.5)
        self.assertAlmostEqual(pos, 0.0, places=6)
        self.assertAlmostEqual(vel, 0.0, places=6)

    def test_sample_before_start_returns_start(self):
        t = TrapezoidalProfile(start=10, target=100, cruise_dps=50, accel_dps2=100)
        pos, vel = t.sample(-1.0)
        self.assertAlmostEqual(pos, 10.0, places=6)
        self.assertAlmostEqual(vel, 0.0, places=6)

    def test_sample_after_end_holds_target(self):
        t = TrapezoidalProfile(start=0, target=50, cruise_dps=50, accel_dps2=100)
        pos, vel = t.sample(100.0)
        self.assertAlmostEqual(pos, 50.0, places=6)
        self.assertAlmostEqual(vel, 0.0, places=6)

    def test_position_monotonically_approaches_target(self):
        t = TrapezoidalProfile(start=0, target=200, cruise_dps=100, accel_dps2=200)
        prev = -1.0
        for s in range(0, 301):
            ts = s * t.duration() / 300
            pos, vel = t.sample(ts)
            self.assertGreaterEqual(pos, prev - 1e-9)   # allow FP noise
            prev = pos
        self.assertAlmostEqual(pos, 200.0, places=4)


if __name__ == "__main__":
    unittest.main()
