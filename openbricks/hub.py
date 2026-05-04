# SPDX-License-Identifier: MIT
"""
Hub — board-level peripherals (status LED, user button) for each
supported MCU devkit.

Concrete hubs:

* ``ESP32DevkitHub``    — ESP32 DevKitC-V4: blue LED on GPIO 2.
* ``ESP32S3DevkitHub``  — ESP32-S3 DevKitC-1: onboard WS2812 RGB LED on
  GPIO 48.

Both hubs expect **two** momentary pushbuttons, each wired between a
GPIO and GND:

* ``bluetooth_button_pin`` (default **GPIO 5**) — short-press toggles
  BLE on/off. Watched by
  :class:`openbricks.bluetooth_button.BluetoothToggleButton` which
  the hub auto-wires when ``bluetooth=True`` (the default).
* The **program button** (default **GPIO 4**) — short-press starts
  or stops ``/program.py``. Watched by
  :mod:`openbricks.launcher` directly, which the frozen default
  ``main.py`` starts via ``launcher.run()``.

Two pins → no duration-based dispatch. Each button does one thing on
every short press.

We deliberately avoid GPIO 0 (BOOT) for either role: holding it
during reset puts the ESP32 into the ROM bootloader, so sharing that
pin with an application button would turn an accidental power-glitch
into a flash-mode drop.

Both hubs auto-wire the BLE toggle button by default (``bluetooth=True``):
**constructing a hub** restores the persisted BLE state, installs a
short-press handler on the ``bluetooth_button_pin``, and — on the S3 —
paints the WS2812 blue (BLE on) or yellow (BLE off). Put
``hub = ESP32S3DevkitHub()`` in your ``main.py`` to turn that on;
omit the call (or pass ``bluetooth=False``) to keep the button free
for your own use.

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

    led              = None   # StatusLED
    bluetooth_button = None   # Button — watched by BluetoothToggleButton


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
    """ESP32 DevKitC-V4 onboard hub: blue LED on GPIO 2, BLE-toggle
    button on GPIO 5.

    ``bluetooth`` default True wires the button to a long-press BLE
    toggle and restores the persisted state at boot (see
    ``openbricks.bluetooth_button`` / ``openbricks.bluetooth``). The
    onboard LED is single-colour so no colour feedback, just the toggle.
    Pass ``bluetooth=False`` to skip the wiring entirely, or
    ``bluetooth_button_pin=<N>`` to use a different GPIO. Wire the
    button between the chosen GPIO and GND; an internal pull-up is
    enabled.
    """

    def __init__(self, led_pin=2, bluetooth_button_pin=5, bluetooth=True):
        self.led = SimpleLED(led_pin)
        self.bluetooth_button = PushButton(bluetooth_button_pin, active_low=True)
        self.bluetooth_toggle = None
        if bluetooth:
            _install_bluetooth_toggle(self)


class ESP32S3DevkitHub(Hub):
    """ESP32-S3 DevKitC-1 onboard hub: WS2812 RGB LED on GPIO 48,
    BLE-toggle button on GPIO 5.

    ``led_pin`` defaults to 48 (the DevKitC-1 onboard WS2812). Pass
    ``led_pin=None`` to disable the LED entirely, or any other GPIO for a
    WS2812 wired elsewhere. ``brightness`` (0.0 – 1.0) scales channel
    values on write; default 0.2 is comfortable indoors.

    ``bluetooth_button_pin`` defaults to 5 — a safe, free pin on the
    DevKitC-1. Wire a momentary switch between GPIO 5 and GND; the
    pin is configured ``Pin.IN`` with ``PULL_UP`` so no external
    resistor is needed. Override via ``bluetooth_button_pin=<N>``.
    """

    def __init__(self, led_pin=48, bluetooth_button_pin=5, brightness=0.2,
                 bluetooth=True):
        self.led = NeoPixelLED(led_pin, brightness=brightness) if led_pin is not None else None
        self.bluetooth_button = PushButton(bluetooth_button_pin, active_low=True)
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

def _install_bluetooth_toggle(hub):
    """Called from the hub constructors when ``bluetooth=True``.

    Builds a long-press watcher on the hub's button (recolouring the
    LED on each toggle if it's RGB-capable) and stashes the watcher
    on ``hub.bluetooth_toggle`` so callers can ``stop()`` it later
    if they want the button for something else.

    Until 1.0.10 this also called ``bluetooth.apply_persisted_state()``
    to restore the persisted BLE-on/off state. That was wrong: the
    frozen ``main.py`` calls ``apply_persisted_state()`` *first*,
    bringing BLE up + advertising. A second call from here re-runs
    ``ble.active(True)`` — which under MicroPython's NimBLE port
    unconditionally tears down and re-inits the stack
    (``mp_bluetooth_init`` calls ``mp_bluetooth_deinit`` even when
    already active, stopping advertising and clearing registered
    services). The follow-up ``ble_repl.start()`` then early-returns
    because ``_state["bridge"]`` is still set from the first call,
    so services + advertising are never re-installed. End result:
    BLE active but invisible to scanners.

    main.py owns BLE lifecycle. The hub's role is the toggle button.
    """
    from openbricks.bluetooth_button import BluetoothToggleButton
    hub.bluetooth_toggle = BluetoothToggleButton(hub.bluetooth_button, led=hub.led)
    hub.bluetooth_toggle.start()
