# SPDX-License-Identifier: MIT
"""
Hub — board-level peripherals (status LED, user button) for each
supported MCU devkit.

Concrete hubs:

* ``ESP32DevkitHub``    — ESP32 DevKitC-V4: single blue LED on GPIO 2,
  BOOT button on GPIO 0.
* ``ESP32S3DevkitHub``  — ESP32-S3 DevKitC-1: onboard WS2812 RGB LED on
  GPIO 48, BOOT button on GPIO 0.

Both hubs auto-wire the BLE toggle button by default (``bluetooth=True``):
constructing a hub restores the persisted BLE state, installs a long-
press handler on the BOOT button, and — on the S3 — paints the WS2812
blue (BLE on) or yellow (BLE off). Pass ``bluetooth=False`` to keep
the button free for your own use.

External I2C components like SSD1306 OLEDs are **not** part of the hub
— they're wired to any I2C bus the user chooses, instantiated directly
from ``openbricks.drivers.ssd1306`` or similar, and used alongside the
hub rather than through it.
"""

from machine import Pin


# ---- abstract base classes ----
#
# Colocated with the concrete implementations to keep ``interfaces.py``
# small — every ``import openbricks`` loads that module, and MicroPython's
# unix-port heap is tight enough that extra class defs there have
# pushed unrelated tests over the allocation limit in the past.


class StatusLED:
    """A user-visible status LED on the hub.

    Plain single-colour LEDs implement ``on`` / ``off``. Addressable RGB
    LEDs (WS2812 and friends) additionally implement ``rgb``.
    """

    def on(self):
        raise NotImplementedError

    def off(self):
        raise NotImplementedError

    def rgb(self, r, g, b):
        """Set colour (each channel 0..255). Raises on non-addressable LEDs."""
        raise NotImplementedError


class Button:
    """A momentary pushbutton on the hub."""

    def pressed(self):
        """Return ``True`` while the button is held down."""
        raise NotImplementedError


class Hub:
    """Board-level peripherals baked into a specific MCU devkit."""

    led = None      # StatusLED
    button = None   # Button


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


class NeoPixelLED(StatusLED):
    """Single-pixel WS2812 / NeoPixel driver — the onboard LED on ESP32-S3
    DevKitC-1 and most modern ESP32-S3 compact boards.

    Unlike ``SimpleLED`` this one implements ``rgb(r, g, b)`` too, which is
    what the Bluetooth-toggle button uses for blue / yellow feedback.

    ``brightness`` scales every channel on write (0.0 – 1.0). 0.2 is a
    comfortable indoor default — the raw WS2812 at full brightness is
    dazzling.
    """

    def __init__(self, pin, brightness=0.2):
        import neopixel  # only available on firmware; tests install a fake
        self._np          = neopixel.NeoPixel(Pin(pin), 1)
        self._brightness  = float(brightness)
        self._last_rgb    = (255, 255, 255)  # used by on() after an off()
        self.off()

    def on(self):
        self.rgb(*self._last_rgb)

    def off(self):
        self._np[0] = (0, 0, 0)
        self._np.write()

    def rgb(self, r, g, b):
        self._last_rgb = (int(r), int(g), int(b))
        b_ = self._brightness
        self._np[0] = (int(r * b_), int(g * b_), int(b * b_))
        self._np.write()


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

    ``bluetooth`` default True wires the BOOT button to a long-press BLE
    toggle and restores the persisted state at boot (see
    ``openbricks.bluetooth_button`` / ``openbricks.bluetooth``). The
    onboard LED is single-colour so no colour feedback, just the toggle.
    Pass ``bluetooth=False`` to reserve the button for your own use.
    """

    def __init__(self, led_pin=2, button_pin=0, bluetooth=True):
        self.led = SimpleLED(led_pin)
        self.button = PushButton(button_pin, active_low=True)
        self.bluetooth_toggle = None
        if bluetooth:
            _install_bluetooth_toggle(self)


class ESP32S3DevkitHub(Hub):
    """ESP32-S3 DevKitC-1 onboard hub: WS2812 RGB LED on GPIO 48,
    BOOT button on GPIO 0.

    ``led_pin`` defaults to 48 (the DevKitC-1 onboard WS2812). Pass
    ``led_pin=None`` to disable the LED entirely, or any other GPIO for a
    WS2812 wired elsewhere. ``brightness`` (0.0 – 1.0) scales channel
    values on write; default 0.2 is comfortable indoors.
    """

    def __init__(self, led_pin=48, button_pin=0, brightness=0.2, bluetooth=True):
        self.led = NeoPixelLED(led_pin, brightness=brightness) if led_pin is not None else None
        self.button = PushButton(button_pin, active_low=True)
        self.bluetooth_toggle = None
        if bluetooth:
            _install_bluetooth_toggle(self)


# ---- BLE auto-wiring ----
#
# Called from the hub constructors when ``bluetooth=True`` (the default).
# Restores the persisted BLE state (default on for a fresh board),
# installs a long-press handler on the BOOT button that toggles it,
# and — if the hub has an RGB-capable LED — paints the LED to match
# the current state (blue = on, yellow = off).
#
# Kept out of the hub classes' namespace so the hub doesn't statically
# depend on ``openbricks.bluetooth`` — the imports happen at the first
# ``bluetooth=True`` construction only.

# Module-level flag: prevents a second ``bluetooth=True`` hub
# construction from spawning a parallel watcher on the same button
# (which would toggle twice per long-press and net no-op). First call
# wins; subsequent calls are a no-op. ``openbricks.__init__`` auto-boots
# an instance, so this normally fires there — a user who later does
# ``hub = ESP32S3DevkitHub()`` just gets a plain hub with
# ``hub.bluetooth_toggle is None``.
_bluetooth_toggle_installed = False


def _install_bluetooth_toggle(hub):
    global _bluetooth_toggle_installed
    if _bluetooth_toggle_installed:
        return
    from openbricks import bluetooth as _bt
    from openbricks.bluetooth_button import BluetoothToggleButton
    _bt.apply_persisted_state()
    hub.bluetooth_toggle = BluetoothToggleButton(hub.button, led=hub.led)
    hub.bluetooth_toggle.start()
    _bluetooth_toggle_installed = True


def _reset_bluetooth_toggle_for_test():
    """Test-only hook — drop the once-only latch between tests."""
    global _bluetooth_toggle_installed
    _bluetooth_toggle_installed = False
