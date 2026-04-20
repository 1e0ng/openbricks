# SPDX-License-Identifier: MIT
"""
Bosch BNO055 9-DOF IMU with onboard sensor fusion.

The BNO055 is unusual: it runs Kalman-like fusion on-chip and hands you
already-computed Euler angles / quaternions / linear acceleration. That makes
it the easiest IMU to bring up — you just configure the mode and read
registers. Fusion mode is ``NDOF`` (accel + gyro + mag + fusion).

Reference: BNO055 datasheet, section 3.3 (operating modes) and 4.3 (register
map). Default I2C address is 0x28 (0x29 if COM3 pin is pulled high).
"""

import time

from openbricks.interfaces import IMU

# Register addresses (page 0 of the BNO055 register map)
_CHIP_ID       = 0x00
_OPR_MODE      = 0x3D
_PWR_MODE      = 0x3E
_SYS_TRIGGER   = 0x3F
_UNIT_SEL      = 0x3B
_EULER_H_LSB   = 0x1A      # 6 bytes: heading, roll, pitch (little-endian int16)
_GYR_DATA_X_LSB = 0x14     # 6 bytes: gx, gy, gz
_ACC_DATA_X_LSB = 0x08     # 6 bytes: ax, ay, az

# Operating modes
_MODE_CONFIG = 0x00
_MODE_NDOF   = 0x0C        # 9-DOF fusion with fast magnetometer calibration

_EXPECTED_CHIP_ID = 0xA0


class BNO055(IMU):
    def __init__(self, i2c, address=0x28):
        self._i2c = i2c
        self._addr = address

        # Sanity check
        chip_id = self._read_u8(_CHIP_ID)
        if chip_id != _EXPECTED_CHIP_ID:
            raise OSError(
                "BNO055 not found at 0x%02x (got chip id 0x%02x)" % (address, chip_id)
            )

        # Switch to CONFIG mode before changing settings.
        self._write_u8(_OPR_MODE, _MODE_CONFIG)
        time.sleep_ms(25)

        # Reset the system.
        self._write_u8(_SYS_TRIGGER, 0x20)
        time.sleep_ms(650)
        # Wait for chip to be ready again.
        while self._read_u8(_CHIP_ID) != _EXPECTED_CHIP_ID:
            time.sleep_ms(10)

        # Normal power mode.
        self._write_u8(_PWR_MODE, 0x00)
        time.sleep_ms(10)

        # Units: degrees for Euler, deg/s for gyro, m/s^2 for accel.
        # bit 7=0 (Windows orientation), bit 2=0 (Celsius),
        # bit 1=0 (deg), bit 0=0 (m/s^2).
        self._write_u8(_UNIT_SEL, 0x00)

        # Engage 9-DOF fusion.
        self._write_u8(_SYS_TRIGGER, 0x00)
        self._write_u8(_OPR_MODE, _MODE_NDOF)
        time.sleep_ms(25)

    # ---- IMU interface ----
    def heading(self):
        """Heading (yaw) in degrees, wrapped to [-180, 180)."""
        h, _r, _p = self._read_euler()
        # BNO055 reports 0..360; convert to signed.
        if h > 180:
            h -= 360
        return h

    def euler(self):
        """(heading, roll, pitch) in degrees."""
        return self._read_euler()

    def angular_velocity(self):
        """(gx, gy, gz) in deg/s."""
        return self._read_vec(_GYR_DATA_X_LSB, 16.0)

    def acceleration(self):
        """(ax, ay, az) in m/s^2."""
        return self._read_vec(_ACC_DATA_X_LSB, 100.0)

    # ---- low-level I/O ----
    def _read_u8(self, reg):
        return self._i2c.readfrom_mem(self._addr, reg, 1)[0]

    def _write_u8(self, reg, value):
        self._i2c.writeto_mem(self._addr, reg, bytes([value]))

    def _read_euler(self):
        buf = self._i2c.readfrom_mem(self._addr, _EULER_H_LSB, 6)
        # Little-endian signed int16. BNO055 Euler scaling = 16 LSB per degree.
        h = _sint16(buf[0], buf[1]) / 16.0
        r = _sint16(buf[2], buf[3]) / 16.0
        p = _sint16(buf[4], buf[5]) / 16.0
        return (h, r, p)

    def _read_vec(self, reg, scale):
        buf = self._i2c.readfrom_mem(self._addr, reg, 6)
        return (
            _sint16(buf[0], buf[1]) / scale,
            _sint16(buf[2], buf[3]) / scale,
            _sint16(buf[4], buf[5]) / scale,
        )


def _sint16(lo, hi):
    value = lo | (hi << 8)
    if value & 0x8000:
        value -= 0x10000
    return value
