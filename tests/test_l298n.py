# SPDX-License-Identifier: MIT
"""Tests for the L298N driver, using shared hardware fakes."""

import tests._fakes  # noqa: F401  (must import before any openbricks.* import)

import unittest

from openbricks.drivers.l298n import L298NMotor


class TestL298N(unittest.TestCase):
    def test_forward_sets_in1_high_in2_low(self):
        m = L298NMotor(in1=1, in2=2, pwm=3)
        m.run(50)
        self.assertEqual(m._in1.value(), 1)
        self.assertEqual(m._in2.value(), 0)
        self.assertAlmostEqual(m._pwm.duty(), 511, delta=2)  # 50% of 1023

    def test_reverse_sets_in1_low_in2_high(self):
        m = L298NMotor(in1=1, in2=2, pwm=3)
        m.run(-75)
        self.assertEqual(m._in1.value(), 0)
        self.assertEqual(m._in2.value(), 1)

    def test_brake_shorts_both_terminals(self):
        m = L298NMotor(in1=1, in2=2, pwm=3)
        m.brake()
        self.assertEqual(m._in1.value(), 1)
        self.assertEqual(m._in2.value(), 1)

    def test_coast_floats_both_terminals(self):
        m = L298NMotor(in1=1, in2=2, pwm=3)
        m.run(100)
        m.coast()
        self.assertEqual(m._in1.value(), 0)
        self.assertEqual(m._in2.value(), 0)
        self.assertEqual(m._pwm.duty(), 0)

    def test_power_clamped(self):
        m = L298NMotor(in1=1, in2=2, pwm=3)
        m.run(9999)
        self.assertLessEqual(m._pwm.duty(), 1023)
        m.run(-9999)
        self.assertLessEqual(m._pwm.duty(), 1023)

    def test_invert_swaps_direction(self):
        m = L298NMotor(in1=1, in2=2, pwm=3, invert=True)
        m.run(50)  # forward intent → reversed pins
        self.assertEqual(m._in1.value(), 0)
        self.assertEqual(m._in2.value(), 1)


if __name__ == "__main__":
    unittest.main()
