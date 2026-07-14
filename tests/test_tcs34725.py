# SPDX-License-Identifier: MIT
"""Tests for the TCS34725 color sensor driver."""

import tests._fakes  # noqa: F401

import unittest

from machine import I2C  # type: ignore[import-not-found]  # provided by fakes

from openbricks.drivers.tcs34725 import TCS34725


_ADDR = 0x29
_CMD  = 0x80
_AUTO = 0x20
_ID     = 0x12
_CDATAL = 0x14


def _le16(value):
    return bytes([value & 0xFF, (value >> 8) & 0xFF])


def _make_i2c_with_id(chip_id=0x44):
    i2c = I2C(0)
    i2c._regs[_ADDR] = {_CMD | _ID: bytes([chip_id])}
    return i2c


def _pack_rgbc(c, r, g, b):
    return _le16(c) + _le16(r) + _le16(g) + _le16(b)


class TestTCS34725(unittest.TestCase):
    def test_init_rejects_wrong_chip_id(self):
        i2c = _make_i2c_with_id(chip_id=0x00)
        with self.assertRaises(OSError):
            TCS34725(i2c)

    def test_init_accepts_tcs34727_variant(self):
        # 0x4D is the -TCS34727 variant; the driver accepts it too.
        i2c = _make_i2c_with_id(chip_id=0x4D)
        TCS34725(i2c)

    def test_raw_returns_four_channels(self):
        i2c = _make_i2c_with_id()
        s = TCS34725(i2c)
        i2c._regs[_ADDR][_CMD | _AUTO | _CDATAL] = _pack_rgbc(1000, 500, 250, 125)
        self.assertEqual(s.raw(), (1000, 500, 250, 125))

    def test_rgb_normalizes_against_clear(self):
        i2c = _make_i2c_with_id()
        s = TCS34725(i2c)
        # C=1000 -> scale = 255/1000. R=1000 -> 255. G=500 -> 127. B=0 -> 0.
        i2c._regs[_ADDR][_CMD | _AUTO | _CDATAL] = _pack_rgbc(1000, 1000, 500, 0)
        self.assertEqual(s.rgb(), (255, 127, 0))

    def test_rgb_clamps_when_channel_exceeds_clear(self):
        # A red channel brighter than the clear channel would overflow
        # the 0..255 range; the driver clamps to 255.
        i2c = _make_i2c_with_id()
        s = TCS34725(i2c)
        i2c._regs[_ADDR][_CMD | _AUTO | _CDATAL] = _pack_rgbc(1000, 2000, 500, 0)
        r, g, b = s.rgb()
        self.assertEqual(r, 255)

    def test_rgb_returns_zero_when_clear_is_zero(self):
        i2c = _make_i2c_with_id()
        s = TCS34725(i2c)
        i2c._regs[_ADDR][_CMD | _AUTO | _CDATAL] = _pack_rgbc(0, 100, 100, 100)
        self.assertEqual(s.rgb(), (0, 0, 0))

    def test_ambient_scales_against_integration_full_scale(self):
        # Datasheet MAX COUNT = 1024 x cycles. Default integration is
        # 24 ms = 10 cycles -> the clear channel saturates at 10240,
        # not 65535. The old 65535 divisor made ambient() top out
        # around 15 and read 0 on every real surface.
        i2c = _make_i2c_with_id()
        s = TCS34725(i2c)   # default integration_ms=24
        i2c._regs[_ADDR][_CMD | _AUTO | _CDATAL] = _pack_rgbc(10240, 0, 0, 0)
        self.assertEqual(s.ambient(), 100)
        i2c._regs[_ADDR][_CMD | _AUTO | _CDATAL] = _pack_rgbc(5120, 0, 0, 0)
        self.assertEqual(s.ambient(), 50)
        i2c._regs[_ADDR][_CMD | _AUTO | _CDATAL] = _pack_rgbc(0, 0, 0, 0)
        self.assertEqual(s.ambient(), 0)

    def test_ambient_typical_mat_reading_is_not_zero(self):
        # The hardware symptom that exposed the bug: a normal indoor
        # reading (clear of a few hundred counts) must not floor to 0.
        i2c = _make_i2c_with_id()
        s = TCS34725(i2c)
        i2c._regs[_ADDR][_CMD | _AUTO | _CDATAL] = _pack_rgbc(2048, 0, 0, 0)
        self.assertEqual(s.ambient(), 20)

    def test_ambient_clamps_at_100(self):
        # A count above the theoretical full scale (sensor quirk /
        # different ATIME written behind the driver's back) must clamp.
        i2c = _make_i2c_with_id()
        s = TCS34725(i2c)
        i2c._regs[_ADDR][_CMD | _AUTO | _CDATAL] = _pack_rgbc(65535, 0, 0, 0)
        self.assertEqual(s.ambient(), 100)

    def test_ambient_full_scale_caps_at_65535_for_long_integration(self):
        # 614.4 ms = 256 cycles -> 1024 x 256 = 262144, but the ADC
        # register is 16-bit: full scale caps at 65535.
        i2c = _make_i2c_with_id()
        s = TCS34725(i2c, integration_ms=614)
        i2c._regs[_ADDR][_CMD | _AUTO | _CDATAL] = _pack_rgbc(65535, 0, 0, 0)
        self.assertEqual(s.ambient(), 100)
        i2c._regs[_ADDR][_CMD | _AUTO | _CDATAL] = _pack_rgbc(32767, 0, 0, 0)
        self.assertEqual(s.ambient(), 49)




class FractionalIntegrationTests(unittest.TestCase):
    def test_chip_minimum_integration_constructs(self):
        # integration_ms=2.4 (one cycle) is how the line-follow
        # example gets low-latency reads for its D term. The float
        # used to reach time.sleep_ms and crash on hardware (the
        # test fake's sleep_ms is now int-strict, matching MP).
        i2c = _make_i2c_with_id()
        s = TCS34725(i2c, integration_ms=2.4, gain=16)
        # One cycle -> full scale 1024.
        self.assertEqual(s._full_scale, 1024)


if __name__ == "__main__":
    unittest.main()
