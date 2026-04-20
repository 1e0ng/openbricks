# SPDX-License-Identifier: MIT
"""
Quadrature encoder counter using GPIO interrupts.

Works on ESP32 and most MicroPython ports that support IRQ callbacks with
``Pin.IRQ_RISING | Pin.IRQ_FALLING``. For maximum performance on ESP32 you
may want to switch to the PCNT peripheral; this implementation is chosen for
portability, not speed.

Counts are signed; direction is inferred from the phase relationship between
channels A and B.
"""

from machine import Pin


class QuadratureEncoder:
    def __init__(self, pin_a, pin_b):
        self._pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self._pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self._count = 0

        # Both edges on A, direction from B.
        self._pin_a.irq(
            trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
            handler=self._on_edge_a,
        )
        self._pin_b.irq(
            trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
            handler=self._on_edge_b,
        )

    # IRQ handlers. Keep them tiny — allocation inside an ISR is risky
    # under MicroPython (the GC may not be available). Integer mutation
    # of an attribute is safe.
    def _on_edge_a(self, _):
        if self._pin_a.value() == self._pin_b.value():
            self._count -= 1
        else:
            self._count += 1

    def _on_edge_b(self, _):
        if self._pin_a.value() == self._pin_b.value():
            self._count += 1
        else:
            self._count -= 1

    def count(self):
        """Return the raw edge count (4x the encoder's stated CPR)."""
        return self._count

    def reset(self, value=0):
        self._count = value
