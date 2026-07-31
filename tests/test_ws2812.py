# SPDX-License-Identifier: MIT
"""Tests for the WS2812 / WS2812B strip driver."""

import tests._fakes  # noqa: F401
import tests._fakes_neopixel  # noqa: F401  (installs the neopixel fake)

import unittest

from openbricks import pins
from openbricks.drivers.ws2812 import WS2812


def _make(n=8, brightness=0.2, pin=4):
    return WS2812(pin=pin, n=n, brightness=brightness)


class ConstructionTests(unittest.TestCase):
    def test_default_is_eight_pixels_all_off_and_pushed(self):
        s = _make()
        self.assertEqual(len(s), 8)
        for i in range(8):
            self.assertEqual(s[i], (0, 0, 0))
            self.assertEqual(s._np[i], (0, 0, 0))
        # clear() at construction pushed a known state to the wire.
        self.assertEqual(s._np.writes, 1)

    def test_rejects_non_positive_n(self):
        with self.assertRaises(ValueError):
            _make(n=0)
        with self.assertRaises(ValueError):
            _make(n=-3)

    def test_rejects_out_of_range_brightness(self):
        with self.assertRaises(ValueError):
            _make(brightness=-0.1)
        with self.assertRaises(ValueError):
            _make(brightness=1.5)

    def test_reserved_pin_raises(self):
        pins.set_chip("esp32s3")
        try:
            with self.assertRaises(pins.ReservedPinError):
                _make(pin=26)   # flash/PSRAM region on the S3
        finally:
            pins.set_chip(None)


class BufferedUpdateTests(unittest.TestCase):
    def test_setitem_buffers_without_writing(self):
        s = _make()
        writes_before = s._np.writes
        s[3] = (255, 128, 10)
        self.assertEqual(s._np.writes, writes_before)   # buffer only
        self.assertEqual(s[3], (255, 128, 10))          # raw readback

    def test_show_pushes_all_pixels_in_one_write(self):
        s = _make(brightness=1.0)
        s[0] = (255, 0, 0)
        s[7] = (0, 0, 255)
        writes_before = s._np.writes
        s.show()
        self.assertEqual(s._np.writes, writes_before + 1)
        self.assertEqual(s._np[0], (255, 0, 0))
        self.assertEqual(s._np[7], (0, 0, 255))

    def test_negative_index_addresses_from_the_end(self):
        s = _make(brightness=1.0)
        s[-1] = (9, 9, 9)
        s.show()
        self.assertEqual(s._np[7], (9, 9, 9))


class BrightnessTests(unittest.TestCase):
    def test_show_scales_channels_but_buffer_stays_raw(self):
        s = _make(brightness=0.2)
        s[0] = (255, 100, 50)
        s.show()
        self.assertEqual(s._np[0], (51, 20, 10))   # int(x * 0.2)
        self.assertEqual(s[0], (255, 100, 50))     # unscaled readback

    def test_changing_brightness_rescales_on_next_show(self):
        s = _make(brightness=0.2)
        s[0] = (200, 200, 200)
        s.show()
        self.assertEqual(s._np[0], (40, 40, 40))
        s.brightness = 1.0
        s.show()
        self.assertEqual(s._np[0], (200, 200, 200))

    def test_brightness_setter_validates(self):
        s = _make()
        with self.assertRaises(ValueError):
            s.brightness = 2.0
        with self.assertRaises(ValueError):
            s.brightness = -0.5
        self.assertEqual(s.brightness, 0.2)   # unchanged after rejects


class FillClearTests(unittest.TestCase):
    def test_fill_sets_every_pixel_and_pushes_once(self):
        s = _make(brightness=1.0)
        writes_before = s._np.writes
        s.fill((10, 20, 30))
        self.assertEqual(s._np.writes, writes_before + 1)
        for i in range(len(s)):
            self.assertEqual(s[i], (10, 20, 30))
            self.assertEqual(s._np[i], (10, 20, 30))

    def test_clear_turns_everything_off(self):
        s = _make(brightness=1.0)
        s.fill((255, 255, 255))
        s.clear()
        for i in range(len(s)):
            self.assertEqual(s._np[i], (0, 0, 0))

    def test_fill_coerces_float_channels_to_int(self):
        s = _make(brightness=1.0)
        s.fill((10.9, 20.5, 30.1))
        self.assertEqual(s[0], (10, 20, 30))


if __name__ == "__main__":
    unittest.main()
