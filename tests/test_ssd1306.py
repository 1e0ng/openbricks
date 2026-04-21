# SPDX-License-Identifier: MIT
"""Tests for openbricks.drivers.ssd1306 — thin wrapper over micropython-lib."""

import tests._fakes            # noqa: F401
import tests._fakes_ssd1306    # noqa: F401

import unittest

from machine import I2C

from openbricks.drivers.ssd1306 import SSD1306


class SSD1306Tests(unittest.TestCase):
    def test_default_dimensions_128x64(self):
        d = SSD1306(I2C(0))
        self.assertEqual(d.width, 128)
        self.assertEqual(d.height, 64)

    def test_custom_dimensions_128x32(self):
        d = SSD1306(I2C(0), width=128, height=32)
        self.assertEqual(d.width, 128)
        self.assertEqual(d.height, 32)
        # Forwards dimensions to impl.
        self.assertEqual(d._impl.width, 128)
        self.assertEqual(d._impl.height, 32)

    def test_default_i2c_address_0x3C(self):
        d = SSD1306(I2C(0))
        self.assertEqual(d._impl.addr, 0x3C)

    def test_address_override(self):
        d = SSD1306(I2C(0), addr=0x3D)
        self.assertEqual(d._impl.addr, 0x3D)

    def test_text_forwards_to_impl(self):
        d = SSD1306(I2C(0))
        d.text("hello", 0, 0)
        self.assertEqual(d._impl.calls[-1], ("text", "hello", 0, 0, 1))

    def test_pixel_forwards_to_impl(self):
        d = SSD1306(I2C(0))
        d.pixel(10, 20, 1)
        self.assertEqual(d._impl.calls[-1], ("pixel", 10, 20, 1))

    def test_fill_show_forward(self):
        d = SSD1306(I2C(0))
        d.fill(1)
        d.show()
        self.assertEqual(d._impl.calls[-2], ("fill", 1))
        self.assertEqual(d._impl.calls[-1], ("show",))

    def test_clear_fills_zero_then_shows(self):
        d = SSD1306(I2C(0))
        d.clear()
        self.assertEqual(d._impl.calls, [("fill", 0), ("show",)])

    def test_delegates_arbitrary_framebuffer_method(self):
        """Attribute forwarding gives access to rect/contrast/etc."""
        d = SSD1306(I2C(0))
        d.rect(0, 0, 10, 10, 1)
        d.contrast(200)
        self.assertEqual(d._impl.calls[-2], ("rect", 0, 0, 10, 10, 1))
        self.assertEqual(d._impl.calls[-1], ("contrast", 200))


if __name__ == "__main__":
    unittest.main()
