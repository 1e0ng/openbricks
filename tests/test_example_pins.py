# SPDX-License-Identifier: MIT
"""Shipped examples must not wire reserved ESP32-S3 pins.

The DC-motor example once used GPIO 4/5/6 for the left motor — GPIO 4
is the launcher's default program button (polled as an input, so IN1
toggling low read as button presses and stopped the running program),
GPIO 5 is the BLE-toggle button, and GPIO 6 is the serial-bus UART RX
convention. The ICM-45686 examples once wired sck=8/mosi=9 straight
into the ADC1 bank the QTRLineSensor window owns. These tests parse
the examples (no import — they drive real hardware at module level)
and fail if any wired pin lands on a reserved one, or if the
docstring's wiring table drifts from the code.

Runs under both CPython and unix MicroPython: plain string scanning,
no ``re`` (MP's ``re`` lacks ``finditer``) and no ``ast``.
"""

import tests._fakes  # noqa: F401  (lets ``openbricks.launcher`` import)

import unittest

from openbricks import launcher


_here = __file__
_idx = _here.rfind("/")
_EXAMPLE = (_here[:_idx] if _idx >= 0 else ".") + "/../examples/esp32_drivebase.py"

# Pins the example must leave alone, and why. 22-25 don't exist on the
# S3; 26-37 are flash/PSRAM on the DevKitC-1 modules.
_RESERVED = {
    launcher.DEFAULT_BUTTON_PIN: "program button (launcher default)",
    38: "BLE-toggle button (hub default)",
    0: "strapping", 3: "strapping", 45: "strapping", 46: "strapping",
    19: "native USB D-", 20: "native USB D+",
    15: "I2C SDA convention", 16: "I2C SCL convention",
    14: "serial-bus UART TX convention",
    41: "serial-bus UART RX convention",
    48: "onboard WS2812 LED",
}
for _p in range(22, 38):
    _RESERVED[_p] = "nonexistent (22-25) or flash/PSRAM (26-37)"


def _ints_after(text, marker):
    """Every integer immediately following ``marker`` in ``text``."""
    out = []
    start = 0
    while True:
        i = text.find(marker, start)
        if i < 0:
            return out
        j = i + len(marker)
        k = j
        while k < len(text) and text[k].isdigit():
            k += 1
        if k > j:
            out.append(int(text[j:k]))
        start = k if k > j else j


def _line_ints(text, marker):
    """The comma-separated integers on ``marker``'s line, after it."""
    i = text.find(marker)
    if i < 0:
        return []
    j = text.find("\n", i)
    tail = text[i + len(marker):j if j >= 0 else len(text)]
    out = []
    for part in tail.split(","):
        part = part.strip()
        if part.isdigit():
            out.append(int(part))
    return out


class ExamplePinTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        try:
            with open(_EXAMPLE) as f:
                cls.text = f.read()
        except OSError:
            raise unittest.SkipTest("examples/ not present (not a checkout)")
        cls.code_pins = []
        for kw in ("in1=", "in2=", "pwm=", "encoder_a=", "encoder_b="):
            cls.code_pins.extend(_ints_after(cls.text, kw))
        cls.doc_pins = _ints_after(cls.text, "-> GPIO ")

    def test_example_parsed(self):
        # Two motors x five pins each; if the constructor shape changes
        # the markers above need updating rather than silently matching
        # nothing.
        self.assertEqual(len(self.code_pins), 10,
                         "expected 10 wired pins, parsed %r" % self.code_pins)

    def test_docstring_matches_code(self):
        self.assertEqual(sorted(self.doc_pins), sorted(self.code_pins),
                         "wiring table in the docstring drifted from the "
                         "constructor arguments")

    def test_no_reserved_pins(self):
        clashes = ["GPIO %d is %s" % (p, _RESERVED[p])
                   for p in self.code_pins if p in _RESERVED]
        self.assertEqual(clashes, [],
                         "example wires reserved pins:\n" + "\n".join(clashes))

    def test_no_duplicate_pins(self):
        self.assertEqual(len(set(self.code_pins)), len(self.code_pins),
                         "same GPIO wired to two functions: %r"
                         % sorted(self.code_pins))


class IcmExamplePinTests(unittest.TestCase):
    """Both ICM-45686 examples wire one SPI quad via
    ``SCK, MOSI, MISO, CS = ...``; it must dodge the reserved set
    AND the ADC1 bank (GPIO 1-10) that the QTRLineSensor window
    owns on the reference robot."""

    EXAMPLES = ("icm45686_bringup.py", "icm45686_square.py",
                "icm45686_axle_track.py", "full_robot.py")

    def _spi_pins(self, name):
        path = (_here[:_idx] if _idx >= 0 else ".") + "/../examples/" + name
        try:
            with open(path) as f:
                text = f.read()
        except OSError:
            raise unittest.SkipTest("examples/ not present (not a checkout)")
        pins = _line_ints(text, "SCK, MOSI, MISO, CS = ")
        self.assertEqual(len(pins), 4,
                         "%s: expected 4 SPI pins, parsed %r" % (name, pins))
        return pins

    def test_no_reserved_or_adc_bank_pins(self):
        for name in self.EXAMPLES:
            for p in self._spi_pins(name):
                self.assertFalse(p in _RESERVED,
                                 "%s wires GPIO %d: %s"
                                 % (name, p, _RESERVED.get(p)))
                self.assertFalse(1 <= p <= 10,
                                 "%s wires GPIO %d inside the ADC1 bank "
                                 "(QTR window convention)" % (name, p))

    def test_examples_agree_on_the_wiring(self):
        quads = [self._spi_pins(name) for name in self.EXAMPLES]
        for name, quad in zip(self.EXAMPLES, quads):
            self.assertEqual(quad, quads[0],
                             "%s wires different SPI pins than %s: "
                             "%r vs %r" % (name, self.EXAMPLES[0],
                                           quad, quads[0]))

    def test_no_duplicate_pins(self):
        for name in self.EXAMPLES:
            pins = self._spi_pins(name)
            self.assertEqual(len(set(pins)), len(pins),
                             "%s wires one GPIO twice: %r" % (name, pins))


if __name__ == "__main__":
    unittest.main()
