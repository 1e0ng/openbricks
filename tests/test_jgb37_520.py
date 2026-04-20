# SPDX-License-Identifier: MIT
"""Tests for the JGB37-520 closed-loop motor driver."""

import tests._fakes  # noqa: F401

import unittest

from openbricks.drivers.jgb37_520 import JGB37Motor


def _make_motor(**overrides):
    kwargs = dict(
        in1=1, in2=2, pwm=3,
        encoder_a=4, encoder_b=5,
        counts_per_output_rev=1320,
    )
    kwargs.update(overrides)
    return JGB37Motor(**kwargs)


class TestJGB37Motor(unittest.TestCase):
    def test_open_loop_run_passes_through_to_h_bridge(self):
        m = _make_motor()
        m.run(50)
        self.assertEqual(m._driver._in1.value(), 1)
        self.assertEqual(m._driver._in2.value(), 0)
        self.assertGreater(m._driver._pwm.duty(), 0)

    def test_brake_and_coast_passthrough(self):
        m = _make_motor()
        m.brake()
        self.assertEqual(m._driver._in1.value(), 1)
        self.assertEqual(m._driver._in2.value(), 1)
        m.coast()
        self.assertEqual(m._driver._pwm.duty(), 0)

    def test_angle_from_encoder_count(self):
        m = _make_motor()
        # 1320 counts per output rev -> 660 counts = 180 deg.
        m._enc._count = 660
        self.assertAlmostEqual(m.angle(), 180.0, places=6)

    def test_reset_angle_sets_encoder_count(self):
        m = _make_motor()
        m.reset_angle(90)
        # 90 deg * 1320 / 360 = 330 counts.
        self.assertEqual(m._enc._count, 330)
        m.reset_angle(0)
        self.assertEqual(m._enc._count, 0)

    def test_run_speed_positive_clamps_to_full_forward(self):
        # At measured=0 and target=300, feedforward=100 and P adds 0.3*300=90.
        # Total power would be 190, which must clamp to 100 -> full duty, in1 high.
        m = _make_motor()
        m.run_speed(300)
        self.assertEqual(m._driver._in1.value(), 1)
        self.assertEqual(m._driver._in2.value(), 0)
        self.assertEqual(m._driver._pwm.duty(), 1023)

    def test_run_speed_negative_clamps_to_full_reverse(self):
        m = _make_motor()
        m.run_speed(-300)
        self.assertEqual(m._driver._in1.value(), 0)
        self.assertEqual(m._driver._in2.value(), 1)
        self.assertEqual(m._driver._pwm.duty(), 1023)

    def test_run_speed_zero_stops_motor(self):
        m = _make_motor()
        m.run_speed(0)
        # Feedforward 0, error 0, power 0 -> both pins low, duty 0.
        self.assertEqual(m._driver._in1.value(), 0)
        self.assertEqual(m._driver._in2.value(), 0)
        self.assertEqual(m._driver._pwm.duty(), 0)


if __name__ == "__main__":
    unittest.main()
