# SPDX-License-Identifier: MIT
"""
SSD1306 OLED driver — thin wrapper around ``micropython-lib``'s
``ssd1306.SSD1306_I2C``.

The micropython-lib driver is already battle-tested and subclasses
``framebuf.FrameBuffer``, so we re-expose it through the openbricks
``Display`` interface. The four core methods (``text``, ``pixel``,
``fill``, ``show``) delegate explicitly; every other FrameBuffer
primitive (``rect``, ``line``, ``hline``, ``scroll``, ``contrast``, …)
reaches through via ``__getattr__``.

The ``ssd1306`` module is frozen into each board's firmware image (see
``native/boards/*/manifest.py``), so ``import`` works out of the box on
flashed hardware.
"""

from openbricks.hub import Display


class SSD1306(Display):
    """128×64 (or 128×32) SSD1306 OLED on I2C."""

    def __init__(self, i2c, addr=0x3C, width=128, height=64):
        from ssd1306 import SSD1306_I2C
        self._impl = SSD1306_I2C(width, height, i2c, addr=addr)
        self.width = width
        self.height = height

    def text(self, s, x, y, c=1):
        self._impl.text(s, x, y, c)

    def pixel(self, x, y, c=None):
        self._impl.pixel(x, y, c)

    def fill(self, c):
        self._impl.fill(c)

    def show(self):
        self._impl.show()

    # Anything not explicitly overridden (rect, line, contrast, invert,
    # scroll, blit, …) reaches the impl via attribute lookup.
    def __getattr__(self, name):
        return getattr(self._impl, name)
