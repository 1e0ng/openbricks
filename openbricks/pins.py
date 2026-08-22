# SPDX-License-Identifier: MIT
"""
Reserved-GPIO guard: fail pin misuse at construction time, loudly.

Drivers that take raw GPIO numbers call :func:`check` before touching
``machine.Pin``. Two kinds of mistakes are caught:

* **Chip-reserved pins** — pins that can never work for user wiring on
  the running chip: nonexistent numbers, the SPI-flash pins, the
  ESP32-S3's native-USB pair, and (for output roles) the classic
  ESP32's input-only range. Claiming these either crashes the chip,
  bricks the USB connection, or fails somewhere far from the real
  mistake; the guard names the pin and the reason instead.
* **Pins the firmware runtime already owns** — the program button, the
  Bluetooth-toggle button, and the status LED register themselves via
  :func:`claim` when the hub / launcher wire them at boot. A driver
  constructed on one of those pins gets an error naming the owner
  ("in use as the program button") plus the constructor argument that
  moves the owner elsewhere.

Chip detection reads ``os.uname().machine`` and recognizes the two
supported targets (ESP32, ESP32-S3). Off-chip — CPython tests, the
MuJoCo sim, unix MicroPython — no chip is detected and the
chip-reserved rules are skipped (the runtime-claims check still
applies); pins there are fakes, not wiring. Tests pin a chip
explicitly with :func:`set_chip`.

Strapping pins (0/3/45/46 on the S3) are deliberately *not* blocked:
they work as regular GPIOs after boot and are routinely used. The
wiring guides in ``docs/hardware.md`` cover the soft cases.
"""

import os


# ---- the ESP32-S3 wiring convention ---------------------------------------

# GPIO -> role, the single source of truth for "which pin does what"
# on the reference build (docs/hardware.md, "GPIO map"). Drivers
# default their pin arguments from it, shipped examples are tested
# against it (tests/test_example_pins.py), and the user-facing docs
# reproduce it. A pin absent from the table is free for user wiring
# (subject to :func:`check`).
ESP32S3_CONVENTION = {
    # ADC1 bank — the QTRLineSensor window. Nothing else may sit
    # here: every other role was moved off it deliberately.
    1: "qtr", 2: "qtr", 3: "qtr", 4: "qtr", 5: "qtr",
    6: "qtr", 7: "qtr", 8: "qtr", 9: "qtr", 10: "qtr",
    11: "spi_miso", 12: "spi_sck", 13: "spi_mosi", 17: "spi_cs",
    14: "uart1_tx", 41: "uart1_rx",
    15: "i2c_sda", 16: "i2c_scl",
    19: "usb", 20: "usb",
    21: "ws2812",
    38: "ble_button", 39: "program_button",
    43: "uart0_tx", 44: "uart0_rx",
    48: "status_led",
}
SERVO_BUS_TX = 14
SERVO_BUS_RX = 41


class ReservedPinError(ValueError):
    """A GPIO was requested that can't (or shouldn't) be user-wired."""


# ---- chip detection ---------------------------------------------------

# None = autodetect from os.uname(); set_chip() overrides.
_chip_override = None
_CHIPS = ("esp32", "esp32s3")


def set_chip(chip):
    """Force the chip model the guard validates against.

    ``chip`` is ``"esp32"``, ``"esp32s3"``, or ``None`` to return to
    autodetection. Used by tests; harmless elsewhere.
    """
    global _chip_override
    if chip is not None and chip not in _CHIPS:
        raise ValueError("chip must be one of %r or None (got %r)"
                         % (_CHIPS, chip))
    _chip_override = chip


def _detect_chip():
    if _chip_override is not None:
        return _chip_override
    try:
        machine_str = os.uname().machine
    except AttributeError:
        return None
    m = machine_str.replace("-", "").replace("_", "").upper()
    # Variant names contain the plain "ESP32" substring, so rule the
    # unsupported ones out first rather than misapplying classic-ESP32
    # pin rules to them.
    for variant in ("ESP32S3",):
        if variant in m:
            return "esp32s3"
    for variant in ("ESP32S2", "ESP32C3", "ESP32C6", "ESP32H2", "ESP32P4"):
        if variant in m:
            return None
    if "ESP32" in m:
        return "esp32"
    return None


# ---- runtime claims (buttons, LED) ------------------------------------

# pin -> (role, hint). Written by the hub / launcher when they wire
# their pins at boot; read by driver check() calls. Re-claiming the
# same pin overwrites — the hub classes and launcher own their
# lifecycle, this dict just mirrors it for error messages.
_claims = {}


def claim(pin, role, hint=""):
    """Record that the firmware runtime owns ``pin`` (e.g. a button)."""
    _claims[int(pin)] = (role, hint)


def release(pin):
    """Forget a claim — for callers that hand a pin back (e.g. after
    ``hub.bluetooth_toggle.stop()``). Unknown pins are a no-op."""
    _claims.pop(int(pin), None)


def _claims_reset():
    """Test helper: drop every claim."""
    _claims.clear()


# ---- validation --------------------------------------------------------


def _die(pin, role, reason):
    raise ReservedPinError(
        "GPIO %d requested for %s, but it is %s" % (pin, role, reason))


def check(pin, role, output=True):
    """Validate that ``pin`` is usable for ``role`` on this chip.

    Args:
        pin: GPIO number the caller is about to wire.
        role: human-readable purpose ("L298N IN1", "HC-SR04 echo") —
            appears in the error message.
        output: the pin must drive (True) or only read (False). Only
            matters on the classic ESP32, where GPIO 34-39 are
            input-only.

    Raises:
        ReservedPinError: with the pin, the role, and the reason.
    """
    pin = int(pin)

    if pin in _claims:
        owner, hint = _claims[pin]
        # Same role re-checking its own pin is fine — the frozen
        # main.py wires the hub/launcher at boot, and a user program
        # constructing its own Hub afterwards re-checks the same pins
        # under the same roles.
        if owner != role:
            _die(pin, role, "in use as the %s%s"
                 % (owner, (" (%s)" % hint) if hint else ""))

    chip = _detect_chip()
    if chip is None:
        return

    if chip == "esp32s3":
        if pin < 0 or pin > 48 or 22 <= pin <= 25:
            _die(pin, role,
                 "not a GPIO on the ESP32-S3 (valid: 0-21, 26-48)")
        if 26 <= pin <= 32:
            _die(pin, role,
                 "connected to the module's SPI flash on the ESP32-S3 "
                 "(GPIO 26-32)")
        if pin in (19, 20):
            _die(pin, role,
                 "the ESP32-S3's native USB D-/D+ pair — wiring it "
                 "breaks USB flashing and the USB REPL")
    else:  # classic esp32
        if pin < 0 or pin > 39 or pin == 20 or pin == 24 or 28 <= pin <= 31:
            _die(pin, role,
                 "not a GPIO on the ESP32 "
                 "(valid: 0-19, 21-23, 25-27, 32-39)")
        if 6 <= pin <= 11:
            _die(pin, role,
                 "connected to the module's SPI flash on the ESP32 "
                 "(GPIO 6-11)")
        if output and 34 <= pin <= 39:
            _die(pin, role,
                 "input-only on the ESP32 (GPIO 34-39 have no output "
                 "driver); use it for encoder/sensor inputs instead")
