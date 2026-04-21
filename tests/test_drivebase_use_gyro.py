# SPDX-License-Identifier: MIT
"""Tests for ``DriveBase.use_gyro(True)`` — Pybricks-style IMU heading feedback.

Kept in its own test module (separate from ``test_drivebase.py``) so that
unrelated segfaults in the coverage-variant debug build of other
drivebase tests don't prevent these from running and contributing
``.gcda`` data to the coverage upload.
"""

import tests._fakes  # noqa: F401

import time
import unittest

from machine import Timer

from openbricks._native import DriveBase as NativeDB, motor_process
from openbricks.drivers.jgb37_520 import JGB37Motor
from openbricks.drivers.l298n import L298NMotor
from openbricks.robotics.drivebase import DriveBase


def _reset_all():
    motor_process.reset()
    Timer.reset_for_test()


class _FakeIMU:
    """Minimal IMU shim — only ``.heading()``, which is the only thing
    the drivebase's gyro path reads. Tests mutate ``heading_value``
    between ticks to simulate real-world rotation."""

    def __init__(self, heading=0.0):
        self.heading_value = heading

    def heading(self):
        return self.heading_value


def _make_motor(in1, in2, pwm, ea, eb):
    return JGB37Motor(
        in1=in1, in2=in2, pwm=pwm,
        encoder_a=ea, encoder_b=eb,
        counts_per_output_rev=1320,
    )


class TestDriveBaseUseGyro(unittest.TestCase):
    def setUp(self):
        _reset_all()

    def test_requires_imu_attached(self):
        left  = _make_motor(1, 2, 3, 10, 11)
        right = _make_motor(4, 5, 6, 12, 13)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        with self.assertRaises(ValueError):
            db.use_gyro(True)

    def test_use_gyro_toggle_without_native_raises(self):
        # L298NMotor has no ``_servo`` — no native path, so use_gyro
        # should error rather than silently do nothing.
        left  = L298NMotor(in1=1, in2=2, pwm=3)
        right = L298NMotor(in1=4, in2=5, pwm=6)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        with self.assertRaises(RuntimeError):
            db.use_gyro(True)

    def test_gyro_heading_drives_diff_correction(self):
        """With use_gyro on, tick the native drivebase once with an IMU
        reading that differs from the captured offset. The heading-error
        term should push one wheel's target_dps above the other — the
        correction that would steer the robot back onto heading."""
        imu = _FakeIMU(heading=0.0)

        left  = _make_motor(1, 2, 3, 10, 11)
        right = _make_motor(4, 5, 6, 12, 13)

        # Reach in and build the native drivebase directly — easier than
        # routing through the Python DriveBase + blocking straight() for
        # this behavioural assertion.
        ndb = NativeDB(
            left=left._servo, right=right._servo,
            wheel_diameter_mm=56, axle_track_mm=114,
            imu=imu,
        )
        # Attach both servos to motor_process.
        left.run_speed(0)
        right.run_speed(0)

        # Enable gyro and start a straight move (0 mm so the target
        # stays put — we just want the heading feedback loop active).
        ndb.use_gyro(True)
        ndb.straight(0.0, 50.0)

        # Now inject a +10° heading (robot rotated CCW). Correction is
        # CW rotation → left wheel advances, right retreats → left_dps
        # ends up above right_dps.
        imu.heading_value = 10.0
        time.sleep_ms(1)   # one tick

        self.assertGreater(left._servo.target_dps(), right._servo.target_dps())

        ndb.stop()

    def test_use_gyro_false_reverts_to_encoder_feedback(self):
        """Toggling use_gyro off makes the controller ignore the IMU."""
        imu = _FakeIMU(heading=0.0)

        left  = _make_motor(1, 2, 3, 10, 11)
        right = _make_motor(4, 5, 6, 12, 13)
        ndb = NativeDB(
            left=left._servo, right=right._servo,
            wheel_diameter_mm=56, axle_track_mm=114,
            imu=imu,
        )
        left.run_speed(0)
        right.run_speed(0)

        ndb.use_gyro(False)      # default, but make it explicit
        ndb.straight(0.0, 50.0)

        # Heading reading is injected but use_gyro is off — encoder diff
        # is zero, so target_dps for left and right should stay equal.
        imu.heading_value = 10.0
        time.sleep_ms(1)

        self.assertAlmostEqual(
            left._servo.target_dps(), right._servo.target_dps(), places=3
        )

        ndb.stop()


if __name__ == "__main__":
    unittest.main()
