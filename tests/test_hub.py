# SPDX-License-Identifier: MIT
"""Tests for openbricks.hub — LED / Button drivers and per-board Hubs."""

import tests._fakes            # noqa: F401
import tests._fakes_neopixel   # noqa: F401  (S3 hub default LED uses neopixel)
import tests._fakes_ble        # noqa: F401  (hub auto-wires BLE by default)

import unittest

from machine import Timer

from tests._fakes_ble import _FakeBLE, _FakeNVS

from openbricks.hub import (
    Button,
    ESP32DevkitHub,
    ESP32S3DevkitHub,
    Hub,
    NeoPixelLED,
    PushButton,
    SimpleLED,
    StatusLED,
)


class SimpleLEDTests(unittest.TestCase):
    def test_defaults_off_on_construction(self):
        led = SimpleLED(2)
        self.assertEqual(led._pin.value(), 0)

    def test_on_off_drives_pin(self):
        led = SimpleLED(2)
        led.on()
        self.assertEqual(led._pin.value(), 1)
        led.off()
        self.assertEqual(led._pin.value(), 0)

    def test_active_low_inverts_polarity(self):
        led = SimpleLED(2, active_high=False)
        led.on()
        self.assertEqual(led._pin.value(), 0)
        led.off()
        self.assertEqual(led._pin.value(), 1)

    def test_rgb_raises_on_plain_led(self):
        led = SimpleLED(2)
        self.assertRaises(NotImplementedError, led.rgb, 255, 0, 0)


class PushButtonTests(unittest.TestCase):
    def test_not_pressed_when_pin_high_active_low(self):
        btn = PushButton(0, active_low=True)
        btn._pin.value(1)     # emulate pull-up resting state
        self.assertFalse(btn.pressed())

    def test_pressed_when_pin_pulled_to_zero(self):
        btn = PushButton(0, active_low=True)
        btn._pin.value(0)
        self.assertTrue(btn.pressed())

    def test_active_high_inverts_sense(self):
        btn = PushButton(0, active_low=False)
        btn._pin.value(0)     # emulate pull-down resting state
        self.assertFalse(btn.pressed())
        btn._pin.value(1)
        self.assertTrue(btn.pressed())


class ESP32DevkitHubTests(unittest.TestCase):
    # bluetooth=False on the structural tests so we test hub wiring in
    # isolation without pulling the BLE auto-wire into the frame.

    def test_is_a_hub_with_led_and_button(self):
        hub = ESP32DevkitHub(bluetooth=False)
        self.assertIsInstance(hub, Hub)
        self.assertIsInstance(hub.led, StatusLED)
        self.assertIsInstance(hub.button, Button)

    def test_default_pins(self):
        hub = ESP32DevkitHub(bluetooth=False)
        self.assertEqual(hub.led._pin.pin, 2)
        self.assertEqual(hub.button._pin.pin, 5)

    def test_pin_overrides(self):
        hub = ESP32DevkitHub(led_pin=17, bluetooth_button_pin=9, bluetooth=False)
        self.assertEqual(hub.led._pin.pin, 17)
        self.assertEqual(hub.button._pin.pin, 9)


class NeoPixelLEDTests(unittest.TestCase):
    def test_rgb_scales_by_brightness(self):
        led = NeoPixelLED(48, brightness=0.5)
        led.rgb(200, 100, 40)
        self.assertEqual(led._np[0], (100, 50, 20))

    def test_off_zeros_the_pixel(self):
        led = NeoPixelLED(48, brightness=1.0)
        led.rgb(200, 100, 40)
        led.off()
        self.assertEqual(led._np[0], (0, 0, 0))

    def test_on_restores_last_color(self):
        led = NeoPixelLED(48, brightness=1.0)
        led.rgb(10, 20, 30)
        led.off()
        led.on()
        self.assertEqual(led._np[0], (10, 20, 30))


class ESP32S3DevkitHubTests(unittest.TestCase):
    def test_onboard_led_is_neopixel(self):
        hub = ESP32S3DevkitHub(bluetooth=False)
        self.assertIsInstance(hub.led, NeoPixelLED)
        self.assertIsInstance(hub.button, Button)

    def test_button_default_pin(self):
        hub = ESP32S3DevkitHub(bluetooth=False)
        self.assertEqual(hub.button._pin.pin, 5)

    def test_led_default_pin_is_48(self):
        hub = ESP32S3DevkitHub(bluetooth=False)
        self.assertEqual(hub.led._np.pin.pin, 48)

    def test_led_pin_override(self):
        hub = ESP32S3DevkitHub(led_pin=21, bluetooth=False)
        self.assertEqual(hub.led._np.pin.pin, 21)

    def test_led_pin_none_disables(self):
        hub = ESP32S3DevkitHub(led_pin=None, bluetooth=False)
        self.assertIsNone(hub.led)


class HubBluetoothAutoWireTests(unittest.TestCase):
    """Default ``bluetooth=True`` should restore persisted state, start the
    long-press watcher, and (on RGB-capable hubs) paint the LED."""

    def setUp(self):
        _FakeNVS._reset_for_test()
        _FakeBLE._reset_for_test()
        Timer.reset_for_test()

    def test_s3_hub_default_activates_ble_and_paints_led_blue(self):
        hub = ESP32S3DevkitHub()
        self.assertIsNotNone(hub.bluetooth_toggle)
        self.assertTrue(_FakeBLE().active())             # default-on restored
        self.assertEqual(hub.led._np[0], (0, 0, 51))     # (0,0,255) scaled by 0.2

    def test_s3_hub_bluetooth_false_opts_out(self):
        hub = ESP32S3DevkitHub(bluetooth=False)
        self.assertIsNone(hub.bluetooth_toggle)
        # BLE stack was never touched; no timer installed.
        self.assertFalse(_FakeBLE().active())

    def test_classic_esp32_hub_default_activates_ble_without_led_color(self):
        # SimpleLED doesn't support rgb(), so the button helper's LED
        # paint is a no-op on the classic hub — but the toggle still
        # works and the persisted state still restores.
        hub = ESP32DevkitHub()
        self.assertIsNotNone(hub.bluetooth_toggle)
        self.assertTrue(_FakeBLE().active())


if __name__ == "__main__":
    unittest.main()
