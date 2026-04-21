# SPDX-License-Identifier: MIT
"""Tests for openbricks.hub — LED / Button drivers and per-board Hubs."""

import tests._fakes  # noqa: F401

import unittest

from openbricks.hub import (
    Button,
    ESP32DevkitHub,
    ESP32S3DevkitHub,
    Hub,
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
    def test_is_a_hub_with_led_and_button(self):
        hub = ESP32DevkitHub()
        self.assertIsInstance(hub, Hub)
        self.assertIsInstance(hub.led, StatusLED)
        self.assertIsInstance(hub.button, Button)

    def test_default_pins(self):
        hub = ESP32DevkitHub()
        self.assertEqual(hub.led._pin.pin, 2)
        self.assertEqual(hub.button._pin.pin, 0)

    def test_pin_overrides(self):
        hub = ESP32DevkitHub(led_pin=5, button_pin=9)
        self.assertEqual(hub.led._pin.pin, 5)
        self.assertEqual(hub.button._pin.pin, 9)


class ESP32S3DevkitHubTests(unittest.TestCase):
    def test_no_onboard_led_by_default(self):
        hub = ESP32S3DevkitHub()
        self.assertIsNone(hub.led)
        self.assertIsInstance(hub.button, Button)

    def test_button_default_pin(self):
        hub = ESP32S3DevkitHub()
        self.assertEqual(hub.button._pin.pin, 0)

    def test_external_led_attached_when_pin_given(self):
        hub = ESP32S3DevkitHub(led_pin=21)
        self.assertIsInstance(hub.led, SimpleLED)
        self.assertEqual(hub.led._pin.pin, 21)


if __name__ == "__main__":
    unittest.main()
