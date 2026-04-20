# SPDX-License-Identifier: MIT
"""
Optional SSD1306 fake for tests that exercise the display driver.

Kept separate from ``tests/_fakes`` so test modules not touching the
display don't pay the memory cost on MicroPython's tight unix heap.
Import alongside ``tests._fakes`` only from ``test_ssd1306`` and
``test_hub``.

Our firmware freezes micropython-lib's ``ssd1306`` driver (which itself
depends on the C-level ``framebuf`` module); the unix MicroPython build
has neither. This stub records every call so tests can assert that the
openbricks wrapper forwards correctly.
"""

import sys


class _FakeSSD1306_I2C:
    def __init__(self, width, height, i2c, addr=0x3C, external_vcc=False):
        self.width = width
        self.height = height
        self.i2c = i2c
        self.addr = addr
        self.external_vcc = external_vcc
        self.calls = []

    def text(self, s, x, y, c=1):
        self.calls.append(("text", s, x, y, c))

    def pixel(self, x, y, c=None):
        self.calls.append(("pixel", x, y, c))

    def fill(self, c):
        self.calls.append(("fill", c))

    def show(self):
        self.calls.append(("show",))

    def rect(self, x, y, w, h, c):
        self.calls.append(("rect", x, y, w, h, c))

    def contrast(self, c):
        self.calls.append(("contrast", c))


class _FakeSSD1306Module:
    SSD1306_I2C = _FakeSSD1306_I2C


sys.modules["ssd1306"] = _FakeSSD1306Module
