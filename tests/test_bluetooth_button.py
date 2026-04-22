# SPDX-License-Identifier: MIT
"""Tests for openbricks.bluetooth_button — short-press toggles BLE."""

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


def _plant_hub_name():
    # ``bluetooth.toggle()`` activates BLE when flipping "off → on" and
    # that path needs a flashed hub name. Plant one in NVS so the
    # toggle succeeds in tests.
    import openbricks
    _FakeNVS(openbricks._HUB_NAME_NVS_NAMESPACE).set_blob(
        openbricks._HUB_NAME_NVS_KEY, b"TestHub")


class BluetoothToggleButtonTests(unittest.TestCase):
    def setUp(self):
        _FakeNVS._reset_for_test()
        _FakeBLE._reset_for_test()
        Timer.reset_for_test()
        _plant_hub_name()

    def test_press_and_release_fires_toggle_once(self):
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, poll_ms=50)
        helper.start()

        btn.pressed_value = True
        time.sleep_ms(200)         # any press duration
        btn.pressed_value = False
        time.sleep_ms(100)         # release edge registered

        # Exactly one toggle: default True → False.
        self.assertEqual(_FakeNVS._STORE["openbricks"]["ble_enabled"], 0)
        self.assertFalse(_FakeBLE().active())

    def test_holding_button_does_not_re_fire(self):
        # While the button stays pressed, no toggle fires — we only
        # act on the release edge.
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, poll_ms=50)
        helper.start()

        btn.pressed_value = True
        time.sleep_ms(5000)        # held for ages

        # Nothing toggled yet.
        self.assertFalse("ble_enabled" in _FakeNVS._STORE.get("openbricks", {}))
        self.assertFalse(_FakeBLE().active())

        # Release now fires exactly one toggle.
        btn.pressed_value = False
        time.sleep_ms(100)
        self.assertEqual(_FakeNVS._STORE["openbricks"]["ble_enabled"], 0)

    def test_release_and_second_press_fires_again(self):
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, poll_ms=50)
        helper.start()

        # 1st press-release: True → False.
        btn.pressed_value = True
        time.sleep_ms(200)
        btn.pressed_value = False
        time.sleep_ms(100)
        self.assertFalse(_FakeBLE().active())

        # 2nd press-release: False → True.
        btn.pressed_value = True
        time.sleep_ms(200)
        btn.pressed_value = False
        time.sleep_ms(100)
        self.assertTrue(_FakeBLE().active())

    # ---- lifecycle ----

    def test_stop_halts_polling(self):
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, poll_ms=50)
        helper.start()
        helper.stop()

        btn.pressed_value = True
        time.sleep_ms(500)
        btn.pressed_value = False
        time.sleep_ms(100)

        # Nothing fired: the Timer is gone.
        self.assertFalse(_FakeBLE().active())
        self.assertFalse("ble_enabled" in _FakeNVS._STORE.get("openbricks", {}))

    def test_double_start_is_idempotent(self):
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, poll_ms=50)
        helper.start()
        helper.start()   # second call should not stack a second Timer

        btn.pressed_value = True
        time.sleep_ms(200)
        btn.pressed_value = False
        time.sleep_ms(100)

        # Exactly one toggle, not two.
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
        _plant_hub_name()

    def test_start_paints_blue_when_ble_enabled(self):
        btn = _StubButton()
        led = _RecordingLED()
        # Default state (never written) is enabled.
        BluetoothToggleButton(btn, led=led, poll_ms=50).start()
        self.assertEqual(led.last_rgb, (0, 0, 255))

    def test_start_paints_yellow_when_ble_disabled(self):
        btn = _StubButton()
        led = _RecordingLED()
        # Pre-persist an "off" state.
        from openbricks import bluetooth
        bluetooth.set_enabled(False)
        BluetoothToggleButton(btn, led=led, poll_ms=50).start()
        self.assertEqual(led.last_rgb, (255, 200, 0))

    def test_toggle_recolors_the_led(self):
        btn = _StubButton()
        led = _RecordingLED()
        helper = BluetoothToggleButton(btn, led=led, poll_ms=50)
        helper.start()  # starts blue (on)

        # Press-release → toggles to off → LED should flip to yellow.
        btn.pressed_value = True
        time.sleep_ms(200)
        btn.pressed_value = False
        time.sleep_ms(100)

        self.assertEqual(led.last_rgb, (255, 200, 0))

        # A second press-release flips back to on → blue.
        btn.pressed_value = True
        time.sleep_ms(200)
        btn.pressed_value = False
        time.sleep_ms(100)

        self.assertEqual(led.last_rgb, (0, 0, 255))

    def test_no_led_is_fine(self):
        """With led=None, no LED calls should be attempted; toggle still fires."""
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, led=None, poll_ms=50)
        helper.start()

        btn.pressed_value = True
        time.sleep_ms(200)
        btn.pressed_value = False
        time.sleep_ms(100)

        self.assertFalse(_FakeBLE().active())

    def test_custom_colors(self):
        btn = _StubButton()
        led = _RecordingLED()
        BluetoothToggleButton(
            btn, led=led, poll_ms=50,
            color_on=(0, 255, 0), color_off=(255, 0, 0),
        ).start()
        # Default state = on → custom green.
        self.assertEqual(led.last_rgb, (0, 255, 0))


if __name__ == "__main__":
    unittest.main()
