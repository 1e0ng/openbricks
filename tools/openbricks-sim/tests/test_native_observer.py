# SPDX-License-Identifier: MIT
"""Parity tests for the CPython-side α-β Observer.

Same algorithm as the firmware's ``_openbricks_native.Observer``; both
wrap ``observer_core.c``. Test against closed-form expectations of
the α-β filter — when both surfaces agree with the math they agree
with each other.
"""

import unittest


class ObserverParityTests(unittest.TestCase):

    def setUp(self):
        from openbricks_sim import _native
        self.Observer = _native.Observer

    def test_default_gains(self):
        # Default alpha=0.5, beta=0.15.
        o = self.Observer()
        # At construction, position and velocity are zero.
        self.assertEqual(o.position(), 0.0)
        self.assertEqual(o.velocity(), 0.0)

    def test_reset_anchors_position_zeros_velocity(self):
        o = self.Observer()
        o.reset(123.4)
        self.assertAlmostEqual(o.position(), 123.4, places=5)
        self.assertEqual(o.velocity(), 0.0)
        # Calling reset() with no arg → 0.
        o.update(50.0, 0.01)
        self.assertNotEqual(o.position(), 0.0)
        o.reset()
        self.assertEqual(o.position(), 0.0)
        self.assertEqual(o.velocity(), 0.0)

    def test_single_update_matches_closed_form(self):
        # alpha=0.5, beta=0.15. From rest at p=0,v=0; measure 1.0 at dt=0.1.
        # predict: p̂' = 0 + 0*0.1 = 0
        #          v̂' = 0
        # residual = 1.0 - 0 = 1.0
        # p̂ = 0 + 0.5 * 1.0 = 0.5
        # v̂ = 0 + (0.15 / 0.1) * 1.0 = 1.5
        o = self.Observer(alpha=0.5, beta=0.15)
        pos, vel = o.update(1.0, 0.1)
        self.assertAlmostEqual(pos, 0.5, places=6)
        self.assertAlmostEqual(vel, 1.5, places=6)

    def test_update_returns_internal_state(self):
        # The update return value should be the same as position()/velocity().
        o = self.Observer()
        pos, vel = o.update(2.0, 0.05)
        self.assertEqual(pos, o.position())
        self.assertEqual(vel, o.velocity())

    def test_zero_or_negative_dt_is_a_no_op(self):
        o = self.Observer()
        o.update(50.0, 0.01)  # change state
        p0, v0 = o.position(), o.velocity()
        o.update(99.0, 0.0)  # no-op
        self.assertEqual(o.position(), p0)
        self.assertEqual(o.velocity(), v0)
        o.update(99.0, -0.1)  # no-op
        self.assertEqual(o.position(), p0)
        self.assertEqual(o.velocity(), v0)

    def test_constant_velocity_steady_state(self):
        # Feed a clean ramp at 100 deg/s; after ~50 steps the observer
        # should track velocity to within 1 % (well-tuned default gains).
        o = self.Observer()
        true_vel = 100.0
        dt = 0.001
        pos = 0.0
        for _ in range(200):
            pos += true_vel * dt
            o.update(pos, dt)
        self.assertAlmostEqual(o.velocity(), true_vel, delta=1.0)
        # Position estimate should also be very close to truth.
        self.assertAlmostEqual(o.position(), pos, delta=0.5)

    def test_step_change_converges_within_a_few_samples(self):
        # The observer should track a step change in true position
        # within a few ticks.
        o = self.Observer()
        o.reset(0.0)
        # Tracker is at 0, suddenly the encoder reads 100 — convergence
        # is critical for the firmware's mid-run interrupt-recovery path.
        for _ in range(20):
            o.update(100.0, 0.01)
        self.assertAlmostEqual(o.position(), 100.0, delta=1.0)
        # Velocity decays back towards zero (no real motion).
        self.assertAlmostEqual(o.velocity(), 0.0, delta=10.0)


if __name__ == "__main__":
    unittest.main()
