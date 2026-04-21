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
by detecting step sizes bigger than half the range whenever ``_count`` is
read.

``_count`` is exposed as a property so that the native ``Servo`` C code
(which reads ``encoder._count`` directly via ``mp_load_attr``, see
``native/user_c_modules/openbricks/servo.c``) gets the live total on
every tick without needing a separate ``.count()`` call.

Works on the whole Espressif ESP32 **family** — both openbricks targets
(ESP32 classic with 8 PCNT units, ESP32-S3 with 4 PCNT units) expose the
same ``esp32.PCNT`` API and run this driver unchanged. It won't work on
non-Espressif MCUs (STM32, RP2040) because those don't ship the ``esp32``
module. ESP32-C3 / C6 dropped PCNT in favour of a newer MCPWM-based
capture architecture and aren't covered here yet.
"""

import esp32
from machine import Pin


_PCNT_MIN = -32767
_PCNT_MAX = 32767
_PCNT_RANGE = _PCNT_MAX - _PCNT_MIN  # = 65534


class PCNTQuadratureEncoder:
    """Hardware quadrature encoder via ESP32 PCNT."""

    def __init__(self, pin_a, pin_b, unit=0, filter=1023):
        # Initialise bookkeeping *before* any property access can fire.
        self._accum = 0
        self._last_raw = 0

        PCNT = esp32.PCNT
        a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        b = Pin(pin_b, Pin.IN, Pin.PULL_UP)

        # Channel 0 — edge on A, direction from B.
        self._pcnt = PCNT(
            unit, channel=0,
            pin=a, rising=PCNT.INCREMENT, falling=PCNT.DECREMENT,
            mode_pin=b, mode_low=PCNT.NORMAL, mode_high=PCNT.REVERSE,
            min=_PCNT_MIN, max=_PCNT_MAX, filter=filter,
        )
        # Channel 1 — edge on B, direction from A. Same unit / same counter
        # so both channels accumulate into one total (4x decoding).
        self._pcnt_ch1 = PCNT(
            unit, channel=1,
            pin=b, rising=PCNT.INCREMENT, falling=PCNT.DECREMENT,
            mode_pin=a, mode_low=PCNT.REVERSE, mode_high=PCNT.NORMAL,
        )
        self._pcnt.start()
        self._last_raw = self._pcnt.value()

    @property
    def _count(self):
        """Total signed counts — recomputed live from the PCNT peripheral.

        Each read folds the delta since the last read into ``_accum`` and
        handles 16-bit wrap (a step bigger than half the counter range
        means we wrapped). Called once per tick by the native Servo.
        """
        raw = self._pcnt.value()
        delta = raw - self._last_raw
        if delta > _PCNT_RANGE // 2:
            delta -= _PCNT_RANGE
        elif delta < -(_PCNT_RANGE // 2):
            delta += _PCNT_RANGE
        self._accum += delta
        self._last_raw = raw
        return self._accum

    @_count.setter
    def _count(self, value):
        """Servo's ``reset_angle`` writes straight into ``encoder._count``."""
        self._accum = int(value)
        self._pcnt.value(0)
        self._last_raw = 0

    # The .count() / .reset() methods are kept for API parity with the
    # software QuadratureEncoder so either can be dropped in wherever
    # the Python side needs a total.
    def count(self):
        return self._count

    def reset(self, value=0):
        self._count = value
