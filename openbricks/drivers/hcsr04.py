# SPDX-License-Identifier: MIT
"""
HC-SR04 ultrasonic distance sensor.

The HC-SR04 has two pins: ``trig`` (input — we drive it) and ``echo``
(output — it pulls high for as long as the round-trip echo took).
Sequence:

  1. Drive ``trig`` high for ~10 µs.
  2. Read the duration of the resulting pulse on ``echo``.
  3. distance_mm = pulse_us × speed_of_sound_mm_per_us / 2.

Speed of sound in air at room temperature is ~0.343 mm/µs, so a
60 cm target gives ~3.5 ms of echo pulse — well within the 30 ms
default timeout.

This driver is pure Python — there's no closed-loop control on a
range sensor, so the 1 kHz hot-path concern doesn't apply. The
``machine.time_pulse_us`` builtin does the actual measurement;
typical call latency is ~1 ms (pulse round-trip) which is fine for
the cold-path use cases (line-following, wall avoidance, mission
"approach until N mm" loops at <100 Hz).
"""

import machine
import time

from openbricks.distance import DistanceSensor


# Speed of sound in air at ~20 °C, in mm per µs. Round-trip; we
# halve it inside ``distance_mm``.
_SPEED_MM_PER_US = 0.343
_DEFAULT_TIMEOUT_US = 30_000   # ~5 m round-trip; matches sensor's stated max range


class HCSR04(DistanceSensor):
    """HC-SR04 ultrasonic distance sensor.

    Args:
        trig: GPIO pin number wired to the sensor's ``TRIG`` pin.
        echo: GPIO pin number wired to ``ECHO``.
        timeout_us: how long to wait for the echo. ``time_pulse_us``
            returns ``-1`` past this; we map that to ``-1`` from
            :meth:`distance_mm` to mean "no return".
    """

    def __init__(self, trig, echo, timeout_us=_DEFAULT_TIMEOUT_US):
        self._trig = machine.Pin(trig, machine.Pin.OUT, value=0)
        self._echo = machine.Pin(echo, machine.Pin.IN)
        self._timeout_us = int(timeout_us)

    def distance_mm(self):
        """Return the round-trip distance to the nearest reflector
        ahead, in millimetres, or ``-1`` if no echo arrived inside
        ``timeout_us``.
        """
        # 10 µs pulse on TRIG; the sensor's MCU detects the rising
        # edge and emits an 8-cycle 40 kHz burst.
        self._trig.value(0)
        time.sleep_us(2)
        self._trig.value(1)
        time.sleep_us(10)
        self._trig.value(0)

        try:
            pulse_us = machine.time_pulse_us(
                self._echo, 1, self._timeout_us)
        except OSError:
            return -1
        if pulse_us < 0:
            return -1
        return int(pulse_us * _SPEED_MM_PER_US / 2.0)
