# SPDX-License-Identifier: MIT
"""
WS2812 / WS2812B RGB LED strip driver (NeoPixel protocol).

Targets the common addressable-LED modules — the 8-LED "stick"
(WS2812B x8), rings, and cut-to-length strips — on a single data
GPIO. The bit-banged 800 kHz protocol itself comes from MicroPython's
built-in ``neopixel`` module; this wrapper adds what user code
actually wants on top of it:

* **Brightness scaling.** Raw WS2812s at full duty are dazzling and
  hot; ``brightness=0.2`` is a comfortable indoor default (same
  convention as the hub's onboard status LED). Colors are stored
  unscaled and multiplied only at ``show()``, so ``strip[i]`` reads
  back exactly what you assigned and changing ``brightness`` later
  re-scales everything on the next ``show()``.
* **Buffered updates.** Item assignment only touches the buffer;
  ``show()`` pushes the whole strip in one wire transaction — an
  animation frame is N assignments + one ``show()``, not N flickery
  writes. ``fill()`` / ``clear()`` are one-call conveniences that
  push immediately.

Usage::

    from openbricks.drivers.ws2812 import WS2812

    strip = WS2812(pin=21, n=8)         # WS2812B x8 stick
    strip.fill((0, 60, 0))              # everything green (pushed)
    strip[0] = (255, 0, 0)              # buffer only...
    strip[7] = (0, 0, 255)
    strip.show()                        # ...pushed together

Wiring the x8 stick: DIN → the data GPIO, 5V → 5 V supply, GND →
common ground. The ESP32's 3.3 V data line is out of spec for a
5 V-supplied WS2812B (V_IH = 0.7 × VDD = 3.5 V) but works with
virtually every module in practice; if you see glitches, power the
stick from 3.3 V (fine for the small x8 boards) or add a level
shifter.
"""

from machine import Pin

from openbricks import pins


class WS2812:
    """A strip of ``n`` WS2812/WS2812B RGB LEDs on one data pin."""

    def __init__(self, pin, n=8, brightness=0.2):
        """
        Args:
            pin: GPIO number the strip's DIN is wired to.
            n: number of LEDs (8 for the common WS2812B x8 stick).
            brightness: 0.0 – 1.0 scale applied to every channel at
                ``show()`` time. 0.2 is a comfortable indoor default.

        Raises:
            ValueError: on a non-positive ``n`` or a ``brightness``
                outside 0.0 – 1.0.
            openbricks.pins.ReservedPinError: when ``pin`` is
                reserved (flash/USB) on the detected chip.
        """
        n = int(n)
        if n <= 0:
            raise ValueError("n must be >= 1 (got %r)" % (n,))
        pins.check(pin, "WS2812 data")
        # Lazy import like the hub's NeoPixelLED: the module exists on
        # firmware (and as a fake under test); importing at call time
        # keeps CPython tooling that merely imports the driver happy.
        import neopixel
        self._np = neopixel.NeoPixel(Pin(pin), n)
        self._n = n
        self._buf = [(0, 0, 0)] * n
        self._brightness = self._check_brightness(brightness)
        # Known state at construction: all off, pushed to the wire.
        self.clear()

    @staticmethod
    def _check_brightness(value):
        value = float(value)
        if not 0.0 <= value <= 1.0:
            raise ValueError(
                "brightness must be within 0.0 - 1.0 (got %r)" % (value,))
        return value

    # ---- container protocol (buffered; call show() to push) ----

    def __len__(self):
        return self._n

    def __setitem__(self, index, color):
        r, g, b = color
        self._buf[index] = (int(r), int(g), int(b))

    def __getitem__(self, index):
        return self._buf[index]

    # ---- pushing to the wire ----

    def show(self):
        """Push the buffered colors to the strip (one transaction),
        applying the current ``brightness`` scale."""
        scale = self._brightness
        for i in range(self._n):
            r, g, b = self._buf[i]
            self._np[i] = (int(r * scale), int(g * scale), int(b * scale))
        self._np.write()

    def fill(self, color):
        """Set every LED to ``color`` (an ``(r, g, b)`` tuple) and
        push immediately."""
        r, g, b = color
        rgb = (int(r), int(g), int(b))
        for i in range(self._n):
            self._buf[i] = rgb
        self.show()

    def clear(self):
        """All LEDs off, pushed immediately."""
        self.fill((0, 0, 0))

    # ---- brightness ----

    @property
    def brightness(self):
        """Global brightness scale 0.0 – 1.0. Assigning re-scales the
        whole strip on the next ``show()`` (or right now via
        ``show()`` — colors are stored unscaled)."""
        return self._brightness

    @brightness.setter
    def brightness(self, value):
        self._brightness = self._check_brightness(value)
