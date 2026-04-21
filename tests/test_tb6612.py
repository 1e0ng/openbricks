# SPDX-License-Identifier: MIT
"""Tests for the TB6612FNG driver alias."""

import tests._fakes  # noqa: F401

import unittest

from openbricks.drivers.l298n import L298NMotor
from openbricks.drivers.tb6612 import TB6612Motor


class TB6612AliasTests(unittest.TestCase):
    def test_is_literally_l298n_motor(self):
        """TB6612Motor and L298NMotor are the same class — the alias only
        exists for discoverability / config naming."""
        self.assertIs(TB6612Motor, L298NMotor)

    def test_forward_drives_in1_high_in2_low(self):
        # Sanity check: the alias still obeys the L298NMotor behaviour.
        m = TB6612Motor(in1=1, in2=2, pwm=3)
        m.run(50)
        self.assertEqual(m._in1.value(), 1)
        self.assertEqual(m._in2.value(), 0)


if __name__ == "__main__":
    unittest.main()
