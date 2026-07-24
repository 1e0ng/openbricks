# SPDX-License-Identifier: MIT
"""Tests for the BNO055 IMU driver."""

import tests._fakes  # noqa: F401

import unittest

from machine import I2C  # type: ignore[import-not-found]  # provided by fakes

from openbricks.drivers.bno055 import BNO055


_CHIP_ID_REG      = 0x00
_EXPECTED_CHIP_ID = 0xA0
_EULER_H_LSB      = 0x1A
_GYR_X_LSB        = 0x14
_ACC_X_LSB        = 0x08


def _le16(value):
    value &= 0xFFFF
    return bytes([value & 0xFF, (value >> 8) & 0xFF])


def _make_i2c_with_chip_id(chip_id=_EXPECTED_CHIP_ID):
    i2c = I2C(0)
    i2c._regs[0x28] = {_CHIP_ID_REG: bytes([chip_id])}
    return i2c


class TestBNO055(unittest.TestCase):
    def test_init_rejects_wrong_chip_id(self):
        i2c = _make_i2c_with_chip_id(chip_id=0xBE)
        with self.assertRaises(OSError):
            BNO055(i2c)

    def test_heading_reports_signed_degrees(self):
        i2c = _make_i2c_with_chip_id()
        imu = BNO055(i2c)
        # Chip reports 270 deg CW (Euler scaling 16 LSB/deg -> raw
        # 4320 = 0x10E0). Driver negates to CCW-positive: -270,
        # wrapped by +360 (since < -180) -> +90.
        i2c._regs[0x28][_EULER_H_LSB] = _le16(4320) + _le16(0) + _le16(0)
        self.assertAlmostEqual(imu.heading(), 90.0, places=3)

    def test_heading_negative_below_180(self):
        i2c = _make_i2c_with_chip_id()
        imu = BNO055(i2c)
        # Chip reports 45 deg CW (raw 720 = 0x02D0) -> negated -45,
        # already within [-180, 180) so no wrap needed.
        i2c._regs[0x28][_EULER_H_LSB] = _le16(720) + _le16(0) + _le16(0)
        self.assertAlmostEqual(imu.heading(), -45.0, places=3)

    def test_heading_is_ccw_positive_matching_turn_convention(self):
        # Regression: the chip's Euler heading is CW-positive (Bosch
        # datasheet Table 3-13, "turning clockwise increases values"
        # -- true regardless of the Windows/Android UNIT_SEL format
        # bit). Everywhere else in openbricks, positive = left = CCW
        # (DriveBase.turn()'s documented convention; Pybricks itself
        # is CW-positive -- this is openbricks' own). Getting this
        # backwards turns DriveBase's closed-loop gyro correction
        # into positive feedback -- confirmed on real ST-3032 bench
        # hardware as tens of degrees of runaway drift per rep on a
        # gyro'd turn() sequence that should have returned to ~0.
        # A small CW rotation on the chip must read as a small
        # NEGATIVE heading here (CCW-positive), not positive.
        i2c = _make_i2c_with_chip_id()
        imu = BNO055(i2c)
        # 5 deg CW on the chip -> raw 80.
        i2c._regs[0x28][_EULER_H_LSB] = _le16(80) + _le16(0) + _le16(0)
        self.assertLess(imu.heading(), 0.0)
        self.assertAlmostEqual(imu.heading(), -5.0, places=3)

    def test_euler_returns_all_three_axes(self):
        i2c = _make_i2c_with_chip_id()
        imu = BNO055(i2c)
        # chip heading=10 (CW) -> negated to -10; roll=-20, pitch=30
        # (deg, untouched -- only heading's sign is corrected).
        # scale = 16 LSB/deg.
        i2c._regs[0x28][_EULER_H_LSB] = _le16(160) + _le16(-320) + _le16(480)
        h, r, p = imu.euler()
        self.assertAlmostEqual(h, -10.0, places=3)
        self.assertAlmostEqual(r, -20.0, places=3)
        self.assertAlmostEqual(p, 30.0, places=3)

    def test_angular_velocity_scales_by_16(self):
        i2c = _make_i2c_with_chip_id()
        imu = BNO055(i2c)
        # gyro scaling in driver is 16.0 LSB per deg/s -> raw 160 = 10 deg/s.
        i2c._regs[0x28][_GYR_X_LSB] = _le16(160) + _le16(-160) + _le16(0)
        gx, gy, gz = imu.angular_velocity()
        self.assertAlmostEqual(gx, 10.0, places=3)
        self.assertAlmostEqual(gy, -10.0, places=3)
        self.assertAlmostEqual(gz, 0.0, places=3)

    def test_acceleration_scales_by_100(self):
        i2c = _make_i2c_with_chip_id()
        imu = BNO055(i2c)
        # accel scaling 100.0 LSB per m/s^2 -> raw 981 ~= 9.81.
        i2c._regs[0x28][_ACC_X_LSB] = _le16(0) + _le16(0) + _le16(981)
        ax, ay, az = imu.acceleration()
        self.assertAlmostEqual(ax, 0.0, places=3)
        self.assertAlmostEqual(ay, 0.0, places=3)
        self.assertAlmostEqual(az, 9.81, places=3)


if __name__ == "__main__":
    unittest.main()
