# SPDX-License-Identifier: MIT
"""Tests for openbricks.drivers.hcsr04 — HC-SR04 ultrasonic distance sensor."""

import tests._fakes  # noqa: F401

import unittest

import machine
from openbricks.drivers.hcsr04 import HCSR04


_DEFAULT_NO_ECHO = staticmethod(lambda p, l, t: -1)


class HCSR04ConstructionTests(unittest.TestCase):

    def test_construct_with_pin_numbers(self):
        # Should not raise; internal Pins are no-op fakes.
        sensor = HCSR04(trig=12, echo=13)
        self.assertEqual(sensor._timeout_us, 30_000)

    def test_custom_timeout(self):
        sensor = HCSR04(trig=12, echo=13, timeout_us=10_000)
        self.assertEqual(sensor._timeout_us, 10_000)


class HCSR04MeasurementTests(unittest.TestCase):
    """Tests rebind ``machine.time_pulse_us`` to inject pulse durations."""

    def setUp(self):
        self._orig = machine.time_pulse_us

    def tearDown(self):
        machine.time_pulse_us = self._orig

    def test_no_echo_returns_minus_one(self):
        # Default fake returns -1 → "no return".
        sensor = HCSR04(trig=12, echo=13)
        self.assertEqual(sensor.distance_mm(), -1)

    def test_echo_at_300mm_returns_300(self):
        # Round-trip distance = 300 mm × 2 = 600 mm.
        # Time = 600 / 0.343 ≈ 1749 µs.
        machine.time_pulse_us = lambda p, l, t: 1749
        sensor = HCSR04(trig=12, echo=13)
        d = sensor.distance_mm()
        self.assertAlmostEqual(d, 300, delta=2)

    def test_echo_at_50mm(self):
        # Round-trip 100 mm → 100 / 0.343 ≈ 291 µs.
        machine.time_pulse_us = lambda p, l, t: 291
        sensor = HCSR04(trig=12, echo=13)
        d = sensor.distance_mm()
        self.assertAlmostEqual(d, 50, delta=1)

    def test_oserror_returns_minus_one(self):
        # MicroPython raises OSError on some timeout paths instead of
        # returning -1. The driver must catch that.
        def _raise(p, l, t):
            raise OSError("ETIMEDOUT")
        machine.time_pulse_us = _raise
        sensor = HCSR04(trig=12, echo=13)
        self.assertEqual(sensor.distance_mm(), -1)


if __name__ == "__main__":
    unittest.main()
