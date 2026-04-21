# SPDX-License-Identifier: MIT
"""
SSD1306 OLED driver — thin wrapper around ``micropython-lib``'s
``ssd1306.SSD1306_I2C``.

The micropython-lib driver is already battle-tested and subclasses
``framebuf.FrameBuffer``. We re-expose its core methods (``text``,
``pixel``, ``fill``, ``show``) explicitly and delegate the rest
(``rect``, ``line``, ``hline``, ``scroll``, ``contrast``, …) via
``__getattr__``. An extra ``clear()`` convenience erases the buffer
and pushes a blank frame.

The ``ssd1306`` module is frozen into each board's firmware image (see
``native/boards/*/manifest.py``), so ``import`` works out of the box on
flashed hardware.

A Display is an external I2C component, not part of the hub — instantiate
one directly wherever you want to draw text / pixels.
"""


class SSD1306:
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

    def clear(self):
        self._impl.fill(0)
        self._impl.show()

    # Anything not explicitly overridden (rect, line, contrast, invert,
    # scroll, blit, …) reaches the impl via attribute lookup.
    def __getattr__(self, name):
        return getattr(self._impl, name)
