# SPDX-License-Identifier: MIT
"""Tests for ``_openbricks_native.Observer``.

Exercises the Python fake in ``tests/_fakes.py``; the C implementation
in ``native/user_c_modules/openbricks/observer.c`` must produce
identical outputs given identical inputs."""

import tests._fakes  # noqa: F401

import random
import unittest

from openbricks._native import Observer


class TestObserver(unittest.TestCase):
    def test_reset_initializes_state(self):
        o = Observer()
        o.reset(42.0)
        self.assertEqual(o.position(), 42.0)
        self.assertEqual(o.velocity(), 0.0)

    def test_update_tracks_constant_velocity_without_noise(self):
        o = Observer()
        o.reset(0.0)
        dt = 0.001
        true_vel = 200.0
        # Feed noise-free samples for half a second.
        for i in range(500):
            true_pos = true_vel * (i + 1) * dt
            o.update(true_pos, dt)
        # Filter should converge on the true velocity.
        self.assertAlmostEqual(o.velocity(), true_vel, delta=0.5)
        # And track position within a small lag.
        self.assertAlmostEqual(o.position(), 500 * dt * true_vel, delta=0.5)

    def test_update_reduces_velocity_variance_vs_finite_diff(self):
        """Exit criterion: the observer must meaningfully smooth the
        velocity estimate. Feed noisy position samples from a truly
        constant-velocity source; the observer's velocity variance
        should be at least 5x less than raw finite-difference.

        Uses Welford's online algorithm so we don't allocate two
        5000-element float lists — the previous list-based form ran
        right up against the MicroPython unix-port heap on Linux CI
        and tipped over whenever a co-located test added a few
        hundred bytes to the global module footprint. Online variance
        is ~constant memory and gives exactly the same answer."""
        random.seed(1)
        o = Observer()
        o.reset(0.0)

        dt = 0.001
        true_vel = 100.0
        noise_amp = 0.5   # +/- 0.25 deg of encoder quantization noise

        # Welford accumulators for raw and observed velocity samples
        # past the initial transient (i >= warmup_ticks).
        warmup_ticks = 1000
        n_raw = 0;  mean_raw = 0.0; m2_raw = 0.0
        n_obs = 0;  mean_obs = 0.0; m2_obs = 0.0

        prev = 0.0
        for i in range(5000):
            true_pos = true_vel * (i + 1) * dt
            noisy = true_pos + (random.random() - 0.5) * noise_amp
            raw_vel = (noisy - prev) / dt
            prev = noisy
            _p, obs_vel = o.update(noisy, dt)
            if i >= warmup_ticks:
                n_raw += 1
                d = raw_vel - mean_raw
                mean_raw += d / n_raw
                m2_raw   += d * (raw_vel - mean_raw)
                n_obs += 1
                d = obs_vel - mean_obs
                mean_obs += d / n_obs
                m2_obs   += d * (obs_vel - mean_obs)

        var_raw = m2_raw / n_raw
        var_obs = m2_obs / n_obs
        self.assertGreater(var_raw / var_obs, 5.0)

    def test_update_converges_on_step_input(self):
        """After a sudden velocity change, the observer catches up
        within a couple dozen ticks."""
        o = Observer()
        o.reset(0.0)
        dt = 0.001
        for i in range(100):
            o.update(0.0, dt)
        # Now start moving at 300 dps.
        vel = 300.0
        for i in range(100):
            true_pos = vel * (i + 1) * dt
            o.update(true_pos, dt)
        self.assertAlmostEqual(o.velocity(), vel, delta=10.0)

    def test_update_handles_zero_dt(self):
        o = Observer()
        o.reset(0.0)
        # dt=0 should leave state unchanged (no divide-by-zero).
        pos, vel = o.update(5.0, 0.0)
        self.assertEqual(pos, 0.0)
        self.assertEqual(vel, 0.0)

    def test_custom_gains_change_convergence_rate(self):
        """After the same few ticks, a high-β observer has tracked
        further toward the true velocity than a low-β one."""
        fast = Observer(alpha=0.9, beta=0.9)
        slow = Observer(alpha=0.1, beta=0.02)
        fast.reset(0.0)
        slow.reset(0.0)
        dt = 0.001
        true_vel = 100.0
        # Just 20 ticks — before either converges fully.
        for i in range(20):
            true_pos = true_vel * (i + 1) * dt
            fast.update(true_pos, dt)
            slow.update(true_pos, dt)
        # fast should be closer to the true velocity than slow.
        self.assertLess(abs(fast.velocity() - true_vel),
                        abs(slow.velocity() - true_vel))


if __name__ == "__main__":
    unittest.main()
