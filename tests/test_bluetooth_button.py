# SPDX-License-Identifier: MIT
"""Tests for openbricks.bluetooth_button — long-press toggles BLE."""

import tests._fakes        # noqa: F401
import tests._fakes_ble    # noqa: F401  (fake esp32 + bluetooth modules)

import time
import unittest

from machine import Timer

from tests._fakes_ble import _FakeBLE, _FakeNVS
from openbricks.bluetooth_button import BluetoothToggleButton


class _StubButton:
    """Tiny Button stand-in — tests flip ``pressed_value`` between ticks."""

    def __init__(self):
        self.pressed_value = False

    def pressed(self):
        return self.pressed_value


class BluetoothToggleButtonTests(unittest.TestCase):
    def setUp(self):
        _FakeNVS._reset_for_test()
        _FakeBLE._reset_for_test()
        Timer.reset_for_test()

    # ---- short-press path ----

    def test_short_press_does_not_toggle(self):
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, long_press_ms=2000, poll_ms=50)
        helper.start()

        # Press for 500 ms then release — well under the 2000 ms threshold.
        btn.pressed_value = True
        time.sleep_ms(500)
        btn.pressed_value = False
        time.sleep_ms(100)

        # Default state (never written) is on — a no-op toggle would flip
        # to off. Confirm we're still at the default.
        self.assertFalse(_FakeBLE().active())   # never activated
        # NVS also hasn't been written.
        self.assertFalse("openbricks" in _FakeNVS._STORE)

    # ---- long-press path ----

    def test_long_press_fires_toggle_once(self):
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, long_press_ms=1000, poll_ms=50)
        helper.start()

        btn.pressed_value = True
        time.sleep_ms(1200)  # one threshold crossing plus a few extra ticks

        # Toggle should have fired exactly once: default True -> False.
        self.assertEqual(_FakeNVS._STORE["openbricks"]["ble_enabled"], 0)
        self.assertFalse(_FakeBLE().active())

    def test_long_press_does_not_re_fire_while_held(self):
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, long_press_ms=1000, poll_ms=50)
        helper.start()

        btn.pressed_value = True
        time.sleep_ms(5000)  # far past threshold, many polls

        # Still only one toggle (→ False).
        self.assertEqual(_FakeNVS._STORE["openbricks"]["ble_enabled"], 0)
        self.assertFalse(_FakeBLE().active())

    def test_release_and_second_long_press_fires_again(self):
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, long_press_ms=1000, poll_ms=50)
        helper.start()

        # 1st long-press: True -> False.
        btn.pressed_value = True
        time.sleep_ms(1200)
        btn.pressed_value = False
        time.sleep_ms(200)
        self.assertFalse(_FakeBLE().active())

        # 2nd long-press: False -> True.
        btn.pressed_value = True
        time.sleep_ms(1200)
        btn.pressed_value = False
        self.assertTrue(_FakeBLE().active())

    # ---- lifecycle ----

    def test_stop_halts_polling(self):
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, long_press_ms=500, poll_ms=50)
        helper.start()
        helper.stop()

        # After stop, holding the button past the threshold should NOT
        # trigger a toggle.
        btn.pressed_value = True
        time.sleep_ms(2000)

        self.assertFalse(_FakeBLE().active())
        self.assertFalse("openbricks" in _FakeNVS._STORE)

    def test_double_start_is_idempotent(self):
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, long_press_ms=500, poll_ms=50)
        helper.start()
        helper.start()   # second call should not stack a second Timer

        # Hold long enough to fire once.
        btn.pressed_value = True
        time.sleep_ms(700)
        btn.pressed_value = False

        # Still one toggle, not two.
        self.assertEqual(_FakeNVS._STORE["openbricks"]["ble_enabled"], 0)


if __name__ == "__main__":
    unittest.main()
