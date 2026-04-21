# SPDX-License-Identifier: MIT
"""M3 exit-criterion tests: the native 2-DOF controller keeps heading
bounded under asymmetric-friction loads that the pure-Kp M1 fallback
can't.

Kept in its own test module because the coverage-variant debug build
occasionally segfaults mid-suite here — isolating these tests means
the segfault doesn't take down sibling drivebase tests, and whichever
runs before the crash still contributes ``.gcda`` data.
"""

import tests._fakes  # noqa: F401

import math
import unittest

from machine import Timer

from openbricks._native import motor_process
from openbricks.drivers.jgb37_520 import JGB37Motor
from openbricks.robotics.drivebase import DriveBase


def _reset_all():
    motor_process.reset()
    Timer.reset_for_test()


def _make_motor(in1, in2, pwm, ea, eb):
    return JGB37Motor(
        in1=in1, in2=in2, pwm=pwm,
        encoder_a=ea, encoder_b=eb,
        counts_per_output_rev=1320,
    )


def _install_asymmetric_sim(left, right, left_scale=1.0, right_scale=1.0):
    """Install a motor_process callback that integrates each motor's
    target_dps into its encoder count, scaled by a per-side factor
    (values < 1.0 simulate friction). Uses a float accumulator so
    slow-speed ticks aren't rounded away."""
    left_acc = [0.0]
    right_acc = [0.0]
    cpr_over_360 = 1320 / 360.0

    def tick():
        dt_s = motor_process.period_ms() / 1000.0
        left_acc[0]  += left._servo.target_dps()  * left_scale  * dt_s * cpr_over_360
        right_acc[0] += right._servo.target_dps() * right_scale * dt_s * cpr_over_360
        left._enc.reset(int(left_acc[0]))
        right._enc.reset(int(right_acc[0]))

    motor_process.register(tick)


class TestDriveBaseNative2DOF(unittest.TestCase):
    def setUp(self):
        _reset_all()

    def test_straight_keeps_heading_under_asymmetric_friction(self):
        """Left wheel has 10% more friction (advances at 0.9x its target).
        2-DOF coupling should still bring both wheels to within a few
        degrees of each other at the end of the move."""
        left  = _make_motor(1, 2, 3, 10, 11)
        right = _make_motor(4, 5, 6, 12, 13)
        _install_asymmetric_sim(left, right, left_scale=0.9, right_scale=1.0)

        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        db.settings(straight_speed=200, turn_rate=180)
        db.straight(100)

        left_angle  = left.angle()
        right_angle = right.angle()
        target_wheel_deg = 100 / (math.pi * 56) * 360

        self.assertGreaterEqual(left_angle,  target_wheel_deg * 0.9)
        self.assertGreaterEqual(right_angle, target_wheel_deg * 0.9)

        heading_err = abs(left_angle - right_angle)
        self.assertLess(heading_err, target_wheel_deg * 0.05)

    def test_turn_converges_in_native_path(self):
        left  = _make_motor(1, 2, 3, 10, 11)
        right = _make_motor(4, 5, 6, 12, 13)
        _install_asymmetric_sim(left, right)

        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        db.settings(turn_rate=180)
        db.turn(90)

        self.assertLess(left.angle(), 0)
        self.assertGreater(right.angle(), 0)

        arc_mm = math.radians(90) * (114 / 2)
        expected_diff = arc_mm / (math.pi * 56) * 360
        diff = (left.angle() - right.angle()) / 2.0
        self.assertLess(abs(abs(diff) - expected_diff), expected_diff * 0.1)


if __name__ == "__main__":
    unittest.main()
