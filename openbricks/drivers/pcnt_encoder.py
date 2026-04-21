# SPDX-License-Identifier: MIT
"""
Quadrature encoder using ESP32's hardware pulse-counter peripheral.

The stock ``openbricks.drivers.encoder.QuadratureEncoder`` runs its
increment logic in MicroPython via ``Pin.irq()``. That path tops out
somewhere around 5-10 kHz edge rate before the handler dispatch starts
losing counts. High-PPR encoders blow past that easily — the MG370 with
its 500-PPR GMR encoder, for instance, emits ~295 kHz at rated speed.

``PCNTQuadratureEncoder`` offloads the counting to the ESP32's PCNT
(Pulse Counter) peripheral — a hardware quadrature decoder that counts
at MHz rates without CPU involvement. One PCNT unit per encoder, both
channels of the unit configured for 4x decoding. ESP32 has 8 PCNT
units, ESP32-S3 has 4 — enough for any openbricks robot.

The counter itself is 16-bit signed (±32767). We handle wrap transparently
by detecting step sizes bigger than half the range between consecutive
``count()`` reads.

API matches ``QuadratureEncoder`` so motor drivers can swap between the
two without other changes. Note that this driver is **ESP32-only** — it
imports the ``esp32`` module at module load time.
"""

import esp32
from machine import Pin


_PCNT_MIN = -32767
_PCNT_MAX = 32767
_PCNT_RANGE = _PCNT_MAX - _PCNT_MIN  # = 65534


class PCNTQuadratureEncoder:
    """Hardware quadrature encoder via ESP32 PCNT.

    ``pin_a`` / ``pin_b``: GPIOs carrying the A and B channels. Both are
    configured as inputs with pull-up, matching the software driver's
    defaults.

    ``unit``: which of the chip's PCNT units to use (0..7 on ESP32, 0..3
    on ESP32-S3). Each encoder needs its own unit.

    ``filter``: glitch-rejection threshold in APB clock cycles (each cycle
    = 12.5 ns at 80 MHz APB). Pulses shorter than this are ignored.
    Range 0..1023 (= 0..12.8 µs). Default 1023 (max) is safe for most
    hobby encoders; drop to ~10 for high-PPR encoders at top speed where
    legitimate pulses can be that short.
    """

    def __init__(self, pin_a, pin_b, unit=0, filter=1023):
        PCNT = esp32.PCNT
        a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        b = Pin(pin_b, Pin.IN, Pin.PULL_UP)

        # Channel 0 — edge on A, direction from B.
        #   A rise + B low   -> INCREMENT
        #   A rise + B high  -> REVERSE(INCREMENT) = DECREMENT
        #   A fall + B low   -> DECREMENT
        #   A fall + B high  -> REVERSE(DECREMENT) = INCREMENT
        self._pcnt = PCNT(
            unit, channel=0,
            pin=a, rising=PCNT.INCREMENT, falling=PCNT.DECREMENT,
            mode_pin=b, mode_low=PCNT.NORMAL, mode_high=PCNT.REVERSE,
            min=_PCNT_MIN, max=_PCNT_MAX, filter=filter,
        )
        # Channel 1 — edge on B, direction from A. Shares the same unit
        # (same counter) so both channels feed into one running total,
        # giving 4x decoding.
        #   B rise + A low   -> REVERSE(INCREMENT) = DECREMENT
        #   B rise + A high  -> INCREMENT
        #   B fall + A low   -> REVERSE(DECREMENT) = INCREMENT
        #   B fall + A high  -> DECREMENT
        self._pcnt_ch1 = PCNT(
            unit, channel=1,
            pin=b, rising=PCNT.INCREMENT, falling=PCNT.DECREMENT,
            mode_pin=a, mode_low=PCNT.REVERSE, mode_high=PCNT.NORMAL,
        )
        self._pcnt.start()
        self._last_raw = self._pcnt.value()
        self._count = 0

    def count(self):
        """Return total signed counts since construction / last ``reset``."""
        raw = self._pcnt.value()
        delta = raw - self._last_raw
        # A step bigger than half the counter range means the 16-bit
        # hardware counter wrapped between reads. Fold the wrap out.
        if delta > _PCNT_RANGE // 2:
            delta -= _PCNT_RANGE
        elif delta < -(_PCNT_RANGE // 2):
            delta += _PCNT_RANGE
        self._count += delta
        self._last_raw = raw
        return self._count

    def reset(self, value=0):
        """Set the count to ``value`` and clear the hardware counter."""
        self._count = value
        self._pcnt.value(0)
        self._last_raw = 0
