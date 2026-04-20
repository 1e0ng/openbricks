# SPDX-License-Identifier: MIT
"""
Hub — board-level peripherals (status LED, user button) for each
supported MCU devkit.

Concrete hubs:

* ``ESP32DevkitHub``    — ESP32 DevKitC-V4: single blue LED on GPIO 2,
  BOOT button on GPIO 0.
* ``ESP32S3DevkitHub``  — ESP32-S3 DevKitC-1: BOOT button on GPIO 0. The
  board's onboard LED is a WS2812 (not a plain digital LED), so no LED
  is attached by default — wire an external one and pass ``led_pin``.

Boards with more hardware (SSD1306 OLED, additional buttons, …) either
subclass one of these or pass explicit pin overrides.
"""

from machine import Pin

from openbricks.interfaces import Button, Hub, StatusLED


class SimpleLED(StatusLED):
    """Single-colour LED driven by one digital pin."""

    def __init__(self, pin, active_high=True):
        self._pin = Pin(pin, Pin.OUT)
        self._active_high = active_high
        self.off()

    def on(self):
        self._pin.value(1 if self._active_high else 0)

    def off(self):
        self._pin.value(0 if self._active_high else 1)


class PushButton(Button):
    """Digital pushbutton with configurable active level and internal pull."""

    def __init__(self, pin, active_low=True):
        pull = Pin.PULL_UP if active_low else Pin.PULL_DOWN
        self._pin = Pin(pin, Pin.IN, pull)
        self._active_low = active_low

    def pressed(self):
        v = self._pin.value()
        return v == 0 if self._active_low else v == 1


class ESP32DevkitHub(Hub):
    """ESP32 DevKitC-V4 onboard hub: blue LED on GPIO 2, BOOT button on GPIO 0.

    Pass an optional ``display`` (any ``Display``-conformant driver, e.g.
    ``openbricks.drivers.ssd1306.SSD1306``) to attach an OLED.
    """

    def __init__(self, led_pin=2, button_pin=0, display=None):
        self.led = SimpleLED(led_pin)
        self.button = PushButton(button_pin, active_low=True)
        self.display = display


class ESP32S3DevkitHub(Hub):
    """ESP32-S3 DevKitC-1 onboard hub: BOOT button on GPIO 0.

    The DevKitC-1 has no plain digital LED (only a WS2812 RGB). If
    you've wired an external LED, pass its pin as ``led_pin``. Optional
    ``display`` attaches any ``Display``-conformant driver.
    """

    def __init__(self, led_pin=None, button_pin=0, display=None):
        self.led = SimpleLED(led_pin) if led_pin is not None else None
        self.button = PushButton(button_pin, active_low=True)
        self.display = display
