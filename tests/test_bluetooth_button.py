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
        # The long-press fires bluetooth.toggle(), which activates BLE
        # when flipping "off → on" and therefore needs a flashed hub
        # name. Plant one in NVS to mimic a properly-flashed device.
        import openbricks
        _FakeNVS(openbricks._HUB_NAME_NVS_NAMESPACE).set_blob(
            openbricks._HUB_NAME_NVS_KEY, b"TestHub")

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
        # NVS ble_enabled flag hasn't been written (hub_name is planted in
        # setUp, so the namespace itself does exist).
        self.assertFalse("ble_enabled" in _FakeNVS._STORE.get("openbricks", {}))

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
        self.assertFalse("ble_enabled" in _FakeNVS._STORE.get("openbricks", {}))

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


class _RecordingLED:
    """``StatusLED``-like stub that records every ``rgb()`` call."""

    def __init__(self):
        self.last_rgb = None
        self.calls = []

    def rgb(self, r, g, b):
        self.last_rgb = (r, g, b)
        self.calls.append((r, g, b))


class BluetoothToggleButtonLEDTests(unittest.TestCase):
    def setUp(self):
        _FakeNVS._reset_for_test()
        _FakeBLE._reset_for_test()
        Timer.reset_for_test()
        # The long-press fires bluetooth.toggle(), which activates BLE
        # when flipping "off → on" and therefore needs a flashed hub
        # name. Plant one in NVS to mimic a properly-flashed device.
        import openbricks
        _FakeNVS(openbricks._HUB_NAME_NVS_NAMESPACE).set_blob(
            openbricks._HUB_NAME_NVS_KEY, b"TestHub")

    def test_start_paints_blue_when_ble_enabled(self):
        btn = _StubButton()
        led = _RecordingLED()
        # Default state (never written) is enabled.
        BluetoothToggleButton(btn, led=led, long_press_ms=1000, poll_ms=50).start()
        self.assertEqual(led.last_rgb, (0, 0, 255))

    def test_start_paints_yellow_when_ble_disabled(self):
        btn = _StubButton()
        led = _RecordingLED()
        # Pre-persist an "off" state.
        from openbricks import bluetooth
        bluetooth.set_enabled(False)
        BluetoothToggleButton(btn, led=led, long_press_ms=1000, poll_ms=50).start()
        self.assertEqual(led.last_rgb, (255, 200, 0))

    def test_toggle_recolors_the_led(self):
        btn = _StubButton()
        led = _RecordingLED()
        helper = BluetoothToggleButton(btn, led=led, long_press_ms=500, poll_ms=50)
        helper.start()  # starts blue (on)

        # Long-press → toggles to off → LED should flip to yellow.
        btn.pressed_value = True
        time.sleep_ms(700)
        btn.pressed_value = False

        self.assertEqual(led.last_rgb, (255, 200, 0))

        # A second long-press flips back to on → blue.
        time.sleep_ms(100)
        btn.pressed_value = True
        time.sleep_ms(700)
        btn.pressed_value = False

        self.assertEqual(led.last_rgb, (0, 0, 255))

    def test_no_led_is_fine(self):
        """With led=None, no LED calls should be attempted; toggle still fires."""
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, led=None, long_press_ms=500, poll_ms=50)
        helper.start()

        btn.pressed_value = True
        time.sleep_ms(700)
        btn.pressed_value = False

        self.assertFalse(_FakeBLE().active())

    def test_custom_colors(self):
        btn = _StubButton()
        led = _RecordingLED()
        BluetoothToggleButton(
            btn, led=led, long_press_ms=1000, poll_ms=50,
            color_on=(0, 255, 0), color_off=(255, 0, 0),
        ).start()
        # Default state = on → custom green.
        self.assertEqual(led.last_rgb, (0, 255, 0))


if __name__ == "__main__":
    unittest.main()
