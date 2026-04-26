# SPDX-License-Identifier: MIT
"""Tests for openbricks.drivers.vl53l0x — laser ToF distance sensor."""

import tests._fakes  # noqa: F401

import unittest

from openbricks.drivers.vl53l0x import VL53L0X


class _ScriptedI2C:
    """Minimal I2C stand-in: pre-configured register reads + a
    write-log. Matches the subset of the I2C API ``VL53L0X`` calls
    (``readfrom_mem`` / ``writeto_mem``)."""

    def __init__(self, regs=None):
        # ``regs`` maps int register-address → bytes. Reads of any
        # register not in the dict return zero-filled bytes.
        self.regs = dict(regs) if regs else {}
        self.writes = []   # list of (addr, reg, bytes)

    def readfrom_mem(self, addr, reg, n):
        chunk = self.regs.get(reg, b"\x00" * n)
        return chunk[:n] if len(chunk) >= n else chunk + b"\x00" * (n - len(chunk))

    def writeto_mem(self, addr, reg, data):
        self.writes.append((addr, reg, bytes(data)))
        # Mirror writes into the read map so the next read returns
        # what was written. Useful for the SYSRANGE_START → poll
        # cycle: writing 0x01 to reg 0x00 then reading back works.
        self.regs[reg] = bytes(data)


def _make_i2c_with_chip(extra_regs=None):
    """Return an I2C scripted with a valid VL53L0X chip-ID + any
    extra register seeds the test wants."""
    regs = {0xC0: b"\xEE"}        # IDENTIFICATION_MODEL_ID
    if extra_regs:
        regs.update(extra_regs)
    return _ScriptedI2C(regs)


class VL53L0XConstructionTests(unittest.TestCase):

    def test_correct_id_succeeds(self):
        i2c = _make_i2c_with_chip()
        sensor = VL53L0X(i2c)
        self.assertEqual(sensor._addr, 0x29)

    def test_custom_address(self):
        i2c = _make_i2c_with_chip()
        sensor = VL53L0X(i2c, address=0x52)
        self.assertEqual(sensor._addr, 0x52)

    def test_wrong_id_raises(self):
        i2c = _ScriptedI2C({0xC0: b"\xAA"})
        with self.assertRaises(OSError):
            VL53L0X(i2c)


class VL53L0XMeasurementTests(unittest.TestCase):

    def test_distance_at_300mm(self):
        # 300 mm encoded big-endian: 0x012C → high=0x01, low=0x2C
        i2c = _make_i2c_with_chip({
            0x13: b"\x01",                # interrupt: ready immediately
            0x14 + 10: b"\x01\x2C",       # range: 300 mm BE
        })
        sensor = VL53L0X(i2c)
        self.assertEqual(sensor.distance_mm(), 300)

    def test_no_return_sentinel(self):
        # 8190 mm → 0x1FFE
        i2c = _make_i2c_with_chip({
            0x13: b"\x01",
            0x14 + 10: b"\x1F\xFE",
        })
        sensor = VL53L0X(i2c)
        self.assertEqual(sensor.distance_mm(), -1)

    def test_zero_reading_treated_as_no_return(self):
        i2c = _make_i2c_with_chip({
            0x13: b"\x01",
            0x14 + 10: b"\x00\x00",
        })
        sensor = VL53L0X(i2c)
        self.assertEqual(sensor.distance_mm(), -1)

    def test_polling_timeout_returns_minus_one(self):
        # Interrupt never sets — driver must fall through and
        # return -1 within the timeout.
        i2c = _make_i2c_with_chip({
            0x13: b"\x00",   # bit 0 always clear
        })
        sensor = VL53L0X(i2c, timeout_ms=10)
        self.assertEqual(sensor.distance_mm(), -1)

    def test_distance_clears_interrupt(self):
        i2c = _make_i2c_with_chip({
            0x13: b"\x01",
            0x14 + 10: b"\x00\x64",    # 100 mm
        })
        sensor = VL53L0X(i2c)
        sensor.distance_mm()
        # Last write must be the interrupt-clear (reg 0x0B = 0x01).
        clear_writes = [w for w in i2c.writes if w[1] == 0x0B]
        self.assertTrue(clear_writes,
                         "driver must clear the interrupt after reading")


if __name__ == "__main__":
    unittest.main()
