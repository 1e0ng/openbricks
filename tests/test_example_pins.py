# SPDX-License-Identifier: MIT
"""Every shipped example must wire pins per the ESP32-S3 convention.

``openbricks.pins.ESP32S3_CONVENTION`` is the single source of truth
for which GPIO does what on the reference build (the GPIO map in
docs/hardware.md). These tests parse every ``examples/*.py`` (no
import — they drive real hardware at module level), collect each
wired pin with its role, and fail if:

* a pin is used for a role the convention assigns to something else
  (e.g. a motor line on the QTR bank, a UART on an I2C pin);
* one file wires the same GPIO to two different functions
  (``full_robot.py`` once had the IMU chip-select and the servo
  UART TX both on GPIO 17);
* the DC-motor example's docstring wiring table drifts from its code.

The driver defaults are checked the same way: constructing a
serial-bus servo or the native drivebase without pin arguments must
land on the convention's UART pins, not on pins another role owns.

Runs under both CPython and unix MicroPython: plain string scanning,
no ``re`` (MP's ``re`` lacks ``finditer``) and no ``ast``.
"""

import tests._fakes  # noqa: F401  (lets the firmware modules import)

import os
import unittest

from openbricks import pins
from openbricks.pins import ESP32S3_CONVENTION, SERVO_BUS_TX, SERVO_BUS_RX


_here = __file__
_idx = _here.rfind("/")
_EXAMPLES_DIR = (_here[:_idx] if _idx >= 0 else ".") + "/../examples"

# Diagnostics that deliberately sweep many GPIOs as pulled-up inputs
# or poke candidate LED pins; they wire nothing and are exempt.
_PROBES = ("button_probe.py", "led_probe.py")

# Keyword argument -> role the pin is being used for.
_KWARG_ROLES = {
    "tx=": "uart1_tx", "rx=": "uart1_rx",
    "sda=": "i2c_sda", "scl=": "i2c_scl",
    "sck=": "spi_sck", "mosi=": "spi_mosi", "miso=": "spi_miso",
    "cs=": "spi_cs",
    "in1=": "dc_motor", "in2=": "dc_motor", "pwm=": "dc_motor",
    "encoder_a=": "dc_motor", "encoder_b=": "dc_motor",
    "trig=": "ultrasonic", "echo=": "ultrasonic",
}
# ``NAME = n`` / ``NAME_PIN = n`` module constants -> role.
_CONST_ROLES = {
    "TX": "uart1_tx", "RX": "uart1_rx",
    "TX_PIN": "uart1_tx", "RX_PIN": "uart1_rx",
    "SERVO_TX": "uart1_tx", "SERVO_RX": "uart1_rx",
    "SDA_PIN": "i2c_sda", "SCL_PIN": "i2c_scl",
    "I2C_SDA": "i2c_sda", "I2C_SCL": "i2c_scl",
    "SCK": "spi_sck", "MOSI": "spi_mosi", "MISO": "spi_miso",
    "CS": "spi_cs",
    "DATA_PIN": "ws2812",
    "BUTTON_PIN": "program_button",
}
# Roles that may also sit on otherwise-free pins.
_MAY_USE_FREE = ("dc_motor", "ultrasonic")
# The DC build is documented as mutually exclusive with the QTR bar
# (docs/hardware.md, "Alternative: DC gear motors") — it alone may
# take pins from the ADC1 bank.
_QTR_EXEMPT = {"esp32_drivebase.py": "dc_motor"}


def _int_at(text, j):
    """The integer starting at ``text[j]`` (skipping ``Pin(``), or None."""
    if text.startswith("Pin(", j):
        j += 4
    k = j
    while k < len(text) and text[k].isdigit():
        k += 1
    return int(text[j:k]) if k > j else None


def _is_ident(c):
    # MicroPython's str has no isalnum().
    return c.isalpha() or c.isdigit() or c == "_"


def _ints_after(text, marker, guard=True):
    out = []
    start = 0
    while True:
        i = text.find(marker, start)
        if i < 0:
            return out
        # ``encoder_a=`` must not match inside ``_encoder_a=``-style
        # identifiers; require a non-identifier char before it.
        if guard and i > 0 and _is_ident(text[i - 1]):
            start = i + 1
            continue
        v = _int_at(text, i + len(marker))
        if v is not None:
            out.append(v)
        start = i + len(marker)


def _code_only(text):
    """Strip the module docstring and ``#`` comments."""
    if text.lstrip().startswith('"""') or '"""' in text[:200]:
        a = text.find('"""')
        b = text.find('"""', a + 3)
        if b > a:
            text = text[:a] + text[b + 3:]
    lines = []
    for line in text.split("\n"):
        h = line.find("#")
        lines.append(line[:h] if h >= 0 else line)
    return "\n".join(lines)


def _wired_pins(text):
    """[(role, pin), ...] for every pin the example wires."""
    code = _code_only(text)
    found = []
    for kw, role in _KWARG_ROLES.items():
        for p in _ints_after(code, kw):
            found.append((role, p))
    for line in code.split("\n"):
        if "=" not in line or line.startswith((" ", "\t")):
            continue
        lhs, rhs = line.split("=", 1)
        names = [n.strip() for n in lhs.split(",")]
        vals = [v.strip() for v in rhs.split(",")]
        if len(names) != len(vals):
            continue
        for name, val in zip(names, vals):
            if name in _CONST_ROLES and val.isdigit():
                found.append((_CONST_ROLES[name], int(val)))
    return found


def _examples():
    try:
        names = sorted(os.listdir(_EXAMPLES_DIR))
    except OSError:
        raise unittest.SkipTest("examples/ not present (not a checkout)")
    return [n for n in names if n.endswith(".py") and n not in _PROBES]


def _read(name):
    with open(_EXAMPLES_DIR + "/" + name) as f:
        return f.read()


class ExamplePinConventionTests(unittest.TestCase):

    def test_scanner_sees_the_known_wiring(self):
        # Guard against the markers silently matching nothing.
        found = _wired_pins(_read("full_robot.py"))
        self.assertTrue(("uart1_tx", 14) in found, found)
        self.assertTrue(("spi_cs", 17) in found, found)
        self.assertTrue(("i2c_sda", 15) in found, found)
        dc = _wired_pins(_read("esp32_drivebase.py"))
        self.assertEqual(len([r for r, _ in dc if r == "dc_motor"]), 10, dc)

    def test_every_example_follows_the_convention(self):
        bad = []
        for name in _examples():
            for role, pin in _wired_pins(_read(name)):
                owner = ESP32S3_CONVENTION.get(pin)
                if owner == role:
                    continue
                if owner is None and role in _MAY_USE_FREE:
                    continue
                if owner == "qtr" and _QTR_EXEMPT.get(name) == role:
                    continue
                bad.append("%s wires GPIO %d as %s but the convention "
                           "assigns it to %s" % (name, pin, role,
                                                  owner or "(free)"))
        self.assertEqual(bad, [], "\n".join(bad))

    def test_no_example_wires_one_gpio_twice(self):
        bad = []
        for name in _examples():
            seen = {}
            for role, pin in _wired_pins(_read(name)):
                if pin in seen and seen[pin] != role:
                    bad.append("%s wires GPIO %d as both %s and %s"
                               % (name, pin, seen[pin], role))
                seen.setdefault(pin, role)
        self.assertEqual(bad, [], "\n".join(bad))

    def test_no_example_touches_chip_reserved_pins(self):
        # Chip rules only: runtime claims (the launcher's button, the
        # hub's LED) belong to whichever earlier test wired them in
        # this process, and an example is *allowed* to name those
        # pins (hard_button_probe.py reads the program button).
        saved = dict(pins._claims)
        pins._claims_reset()
        pins.set_chip("esp32s3")
        try:
            for name in _examples():
                for role, pin in _wired_pins(_read(name)):
                    pins.check(pin, "%s %s" % (name, role),
                               output=(role != "uart1_rx"))
        finally:
            pins.set_chip(None)
            pins._claims.update(saved)

    def test_dc_example_docstring_matches_code(self):
        text = _read("esp32_drivebase.py")
        code = sorted(p for r, p in _wired_pins(text) if r == "dc_motor")
        doc = sorted(_ints_after(text, "-> GPIO ", guard=False))
        self.assertEqual(doc, code,
                         "wiring table in the docstring drifted from the "
                         "constructor arguments")


class DriverDefaultPinTests(unittest.TestCase):
    """Omitting ``tx``/``rx`` must land on the convention's UART pins."""

    def test_convention_reserves_the_servo_bus_pins(self):
        self.assertEqual(ESP32S3_CONVENTION[SERVO_BUS_TX], "uart1_tx")
        self.assertEqual(ESP32S3_CONVENTION[SERVO_BUS_RX], "uart1_rx")
        for p in range(1, 11):
            self.assertEqual(ESP32S3_CONVENTION[p], "qtr")

    def test_serial_servo_defaults(self):
        from openbricks.drivers.st3032 import ST3032Motor
        from openbricks.drivers.st3215 import ST3215, ST3215Motor
        for cls in (ST3215, ST3215Motor, ST3032Motor):
            s = cls(servo_id=11)
            uart = s._bus._uart
            self.assertEqual((uart.tx, uart.rx), (SERVO_BUS_TX, SERVO_BUS_RX),
                             cls.__name__)

    def test_make_uart_defaults(self):
        from openbricks.platforms.esp32 import make_uart
        u = make_uart()
        self.assertEqual((u.tx, u.rx), (SERVO_BUS_TX, SERVO_BUS_RX))


if __name__ == "__main__":
    unittest.main()
