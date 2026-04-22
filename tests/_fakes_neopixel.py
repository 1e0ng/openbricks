# SPDX-License-Identifier: MIT
"""
Optional ``neopixel`` fake for tests that touch ``NeoPixelLED`` or the
S3 hub's default LED. Kept separate from ``tests/_fakes`` so the heap-
tight ``test_observer`` (and anything else that imports openbricks but
doesn't touch the LED) doesn't pay the cost. Import alongside
``tests._fakes`` only from tests that need it.
"""

import sys


class _FakeNeoPixel:
    def __init__(self, pin, n):
        self.pin = pin
        self.n = n
        self._pixels = [(0, 0, 0)] * n
        self.writes = 0

    def __setitem__(self, idx, rgb):
        self._pixels[idx] = tuple(rgb)

    def __getitem__(self, idx):
        return self._pixels[idx]

    def write(self):
        self.writes += 1


class _FakeNeopixelModule:
    NeoPixel = _FakeNeoPixel


sys.modules["neopixel"] = _FakeNeopixelModule
