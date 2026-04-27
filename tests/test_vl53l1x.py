# SPDX-License-Identifier: MIT
"""Tests for openbricks.drivers.vl53l1x — laser ToF distance sensor."""

import tests._fakes  # noqa: F401

import unittest

from openbricks.drivers.vl53l1x import VL53L1X


class _ScriptedI2C:
    """Tracks 16-bit-register protocol: each ``writeto`` whose payload
    is exactly 2 bytes selects a register; the next ``readfrom`` reads
    from there. 3-byte ``writeto`` is a register-write."""

    def __init__(self, regs=None):
        # Maps int register-address → bytes (initial / latest value).
        self.regs = dict(regs) if regs else {}
        self.writes = []
        self._selected = None

    def writeto(self, addr, data):
        data = bytes(data)
        if len(data) == 2:
            # Read-prep: select a register for the next readfrom.
            self._selected = (data[0] << 8) | data[1]
        elif len(data) >= 3:
            # Register-write: 16-bit addr followed by data bytes.
            reg = (data[0] << 8) | data[1]
            payload = data[2:]
            self.regs[reg] = payload
            self.writes.append((addr, reg, payload))
            self._selected = reg

    def readfrom(self, addr, n):
        if self._selected is None:
            return b"\x00" * n
        chunk = self.regs.get(self._selected, b"\x00" * n)
        return chunk[:n] if len(chunk) >= n else chunk + b"\x00" * (n - len(chunk))


def _make_i2c_with_chip(extra_regs=None):
    """Return a scripted I2C with a valid VL53L1X chip-ID + extras."""
    regs = {0x010F: b"\xEA\xCC"}
    if extra_regs:
        regs.update(extra_regs)
    return _ScriptedI2C(regs)


class VL53L1XConstructionTests(unittest.TestCase):

    def test_correct_id_succeeds(self):
        i2c = _make_i2c_with_chip()
        sensor = VL53L1X(i2c)
        self.assertEqual(sensor._addr, 0x29)

    def test_l4cd_alt_id_also_accepted(self):
        i2c = _ScriptedI2C({0x010F: b"\xEB\xAA"})
        VL53L1X(i2c)   # no exception

    def test_wrong_id_raises(self):
        i2c = _ScriptedI2C({0x010F: b"\x00\x00"})
        with self.assertRaises(OSError):
            VL53L1X(i2c)


class VL53L1XMeasurementTests(unittest.TestCase):

    def test_distance_at_500mm(self):
        # 500 mm = 0x01F4 BE
        i2c = _make_i2c_with_chip({
            0x0031: b"\x00",            # GPIO_TIO_HV_STATUS bit 0 clear → done
            0x0096: b"\x01\xF4",        # final range = 500 mm
        })
        sensor = VL53L1X(i2c)
        self.assertEqual(sensor.distance_mm(), 500)

    def test_zero_treated_as_no_return(self):
        i2c = _make_i2c_with_chip({
            0x0031: b"\x00",
            0x0096: b"\x00\x00",
        })
        sensor = VL53L1X(i2c)
        self.assertEqual(sensor.distance_mm(), -1)

    def test_polling_timeout_returns_minus_one(self):
        # Status stays 0x01 (busy) → driver times out and returns -1.
        i2c = _make_i2c_with_chip({0x0031: b"\x01"})
        sensor = VL53L1X(i2c, timeout_ms=10)
        self.assertEqual(sensor.distance_mm(), -1)

    def test_distance_clears_interrupt(self):
        i2c = _make_i2c_with_chip({
            0x0031: b"\x00",
            0x0096: b"\x00\xC8",        # 200 mm
        })
        sensor = VL53L1X(i2c)
        sensor.distance_mm()
        # SYSTEM_INTERRUPT_CLEAR (0x0086) must be written at least
        # once during the measurement cycle.
        cleared = [w for w in i2c.writes if w[1] == 0x0086]
        self.assertTrue(cleared,
                         "driver must clear the data-ready interrupt")


if __name__ == "__main__":
    unittest.main()
