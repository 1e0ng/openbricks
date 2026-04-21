# SPDX-License-Identifier: MIT
"""Tests for the native ``QuadratureEncoder`` C type."""

import tests._fakes  # noqa: F401  (must import before any openbricks.* import)

import unittest

from openbricks._native import QuadratureEncoder


def _fire(pin):
    """Invoke the registered IRQ handler as the pin peripheral would."""
    pin._irq_handler(pin)


class TestQuadratureEncoder(unittest.TestCase):
    def test_starts_at_zero(self):
        enc = QuadratureEncoder(pin_a=1, pin_b=2)
        self.assertEqual(enc.count(), 0)

    def test_forward_cycle_counts_plus_four(self):
        # A leads B: A rises, B rises, A falls, B falls.
        enc = QuadratureEncoder(pin_a=1, pin_b=2)
        a, b = enc.pin_a(), enc.pin_b()

        a.value(1); _fire(a)           # a!=b  -> +1
        b.value(1); _fire(b)           # a==b  -> +1
        a.value(0); _fire(a)           # a!=b  -> +1
        b.value(0); _fire(b)           # a==b  -> +1

        self.assertEqual(enc.count(), 4)

    def test_reverse_cycle_counts_minus_four(self):
        # B leads A: B rises, A rises, B falls, A falls.
        enc = QuadratureEncoder(pin_a=1, pin_b=2)
        a, b = enc.pin_a(), enc.pin_b()

        b.value(1); _fire(b)           # a!=b  -> -1
        a.value(1); _fire(a)           # a==b  -> -1
        b.value(0); _fire(b)           # a!=b  -> -1
        a.value(0); _fire(a)           # a==b  -> -1

        self.assertEqual(enc.count(), -4)

    def test_reset_sets_count(self):
        enc = QuadratureEncoder(pin_a=1, pin_b=2)
        enc.reset(123)
        self.assertEqual(enc.count(), 123)
        enc.reset()
        self.assertEqual(enc.count(), 0)
        enc.reset(500)
        self.assertEqual(enc.count(), 500)


if __name__ == "__main__":
    unittest.main()
