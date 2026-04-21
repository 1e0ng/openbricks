# SPDX-License-Identifier: MIT
"""
Optional ``esp32.PCNT`` fake for tests exercising ``PCNTQuadratureEncoder``.

Kept separate from ``tests/_fakes`` so modules that don't touch PCNT
don't pay the memory cost on MicroPython's tight unix heap (same pattern
as ``tests/_fakes_ssd1306``). Import alongside ``tests._fakes`` only
from ``test_pcnt_encoder``.

The fake mirrors the real ``esp32.PCNT`` API: two channels of the same
``unit`` share one counter. ``value()`` / ``start()`` / ``stop()`` behave
as expected so tests can drive the "hardware" counter and observe how
the openbricks wrapper reacts.
"""

import sys


class _FakePCNT:
    # Match the real esp32.PCNT class attributes so tests can assert
    # against them without importing the real module.
    INCREMENT = 1
    DECREMENT = 2
    IGNORE = 0
    NORMAL = 0
    REVERSE = 1
    HOLD = 2
    IRQ_ZERO = 1
    IRQ_THRESHOLD0 = 2
    IRQ_THRESHOLD1 = 4
    IRQ_MIN = 8
    IRQ_MAX = 16

    # Per-unit shared "hardware" state. Channels 0 and 1 of the same unit
    # see the same counter.
    _UNITS = {}

    def __init__(self, unit, channel=0, pin=None, rising=None, falling=None,
                 mode_pin=None, mode_low=None, mode_high=None,
                 min=None, max=None, filter=None, **kwargs):
        self.unit = unit
        self.channel = channel
        self.pin = pin
        self.rising = rising
        self.falling = falling
        self.mode_pin = mode_pin
        self.mode_low = mode_low
        self.mode_high = mode_high
        self.min = min
        self.max = max
        self.filter = filter
        _FakePCNT._UNITS.setdefault(unit, {"value": 0, "started": False})

    def value(self, v=None):
        if v is None:
            return _FakePCNT._UNITS[self.unit]["value"]
        _FakePCNT._UNITS[self.unit]["value"] = int(v)
        return None

    def start(self):
        _FakePCNT._UNITS[self.unit]["started"] = True

    def stop(self):
        _FakePCNT._UNITS[self.unit]["started"] = False

    def irq(self, trigger=None, handler=None):
        return None

    def deinit(self):
        pass

    @classmethod
    def _reset_for_test(cls):
        cls._UNITS = {}


class _FakeESP32Module:
    PCNT = _FakePCNT


sys.modules["esp32"] = _FakeESP32Module
