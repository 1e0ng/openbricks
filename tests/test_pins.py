# SPDX-License-Identifier: MIT
"""openbricks.pins — the reserved-GPIO guard.

Covers the three protection layers:

* chip-reserved rules (flash / USB / nonexistent / input-only), pinned
  per chip via ``set_chip`` since tests don't run on a real ESP32;
* runtime claims (buttons, LED) — a driver asking for a pin the hub or
  launcher already wired gets an error naming the owner;
* driver integration — the constructors actually call ``check``.
"""

import tests._fakes           # noqa: F401
import tests._fakes_neopixel  # noqa: F401  (S3 hub default LED)

import unittest

from openbricks import pins
from openbricks.drivers.l298n import L298NMotor
from openbricks.hub import ESP32S3DevkitHub


class _GuardCase(unittest.TestCase):
    def setUp(self):
        pins._claims_reset()

    def tearDown(self):
        pins.set_chip(None)
        pins._claims_reset()

    def expect_reserved(self, fn, *substrings):
        """``fn()`` must raise ReservedPinError whose message contains
        every substring. (MicroPython's unittest lacks the
        ``assertRaises(...) as ctx`` form, hence try/except.)"""
        try:
            fn()
        except pins.ReservedPinError as e:
            msg = str(e)
        else:
            self.fail("expected ReservedPinError, nothing raised")
            return
        for s in substrings:
            self.assertTrue(s in msg,
                            "expected %r in error message %r" % (s, msg))


class ChipRulesS3Tests(_GuardCase):
    def setUp(self):
        _GuardCase.setUp(self)
        pins.set_chip("esp32s3")

    def test_flash_pin_raises(self):
        self.expect_reserved(lambda: pins.check(26, "H-bridge IN1"),
                             "SPI flash", "GPIO 26", "H-bridge IN1")

    def test_nonexistent_pins_raise(self):
        for pin in (22, 25, 49, -1):
            self.expect_reserved(lambda p=pin: pins.check(p, "test"),
                                 "not a GPIO")

    def test_usb_pins_raise(self):
        for pin in (19, 20):
            self.expect_reserved(lambda p=pin: pins.check(p, "test"),
                                 "USB")

    def test_free_pins_pass(self):
        for pin in (1, 2, 17, 18, 21, 38, 47, 48):
            pins.check(pin, "test")

    def test_strapping_pins_allowed(self):
        # Usable as regular GPIOs after boot — documented, not blocked.
        for pin in (0, 3, 45, 46):
            pins.check(pin, "test")


class ChipRulesClassicTests(_GuardCase):
    def setUp(self):
        _GuardCase.setUp(self)
        pins.set_chip("esp32")

    def test_flash_pins_raise(self):
        for pin in (6, 11):
            self.expect_reserved(lambda p=pin: pins.check(p, "test"),
                                 "SPI flash")

    def test_nonexistent_pins_raise(self):
        for pin in (20, 24, 28, 31, 40):
            self.expect_reserved(lambda p=pin: pins.check(p, "test"),
                                 "not a GPIO")

    def test_input_only_pins_raise_for_outputs(self):
        self.expect_reserved(lambda: pins.check(34, "H-bridge PWM/EN"),
                             "input-only")

    def test_input_only_pins_pass_for_inputs(self):
        for pin in (34, 39):
            pins.check(pin, "encoder channel A", output=False)


class NoChipTests(_GuardCase):
    def test_chip_rules_skipped_off_chip(self):
        # Autodetect finds no ESP32 under CPython / unix MicroPython,
        # so even a would-be flash pin passes: pins here are fakes,
        # not wiring.
        pins.check(26, "test")

    def test_set_chip_rejects_unknown(self):
        try:
            pins.set_chip("esp8266")
        except ValueError:
            return
        self.fail("expected ValueError for unknown chip")


class ClaimTests(_GuardCase):
    def test_claimed_pin_raises_with_owner_and_hint(self):
        pins.claim(4, "program button",
                   "launcher.run(button_pin=...) moves it")
        self.expect_reserved(lambda: pins.check(4, "H-bridge IN1"),
                             "program button", "button_pin=")

    def test_claims_apply_without_chip_detection(self):
        # Unlike the chip rules, claims protect on any platform.
        pins.claim(5, "Bluetooth-toggle button")
        self.expect_reserved(lambda: pins.check(5, "test"),
                             "Bluetooth-toggle button")

    def test_same_role_may_recheck_its_own_pin(self):
        # Frozen main.py wires the hub at boot; a user program
        # constructing its own Hub re-checks the same pins under the
        # same roles and must not explode.
        pins.claim(5, "Bluetooth-toggle button")
        pins.check(5, "Bluetooth-toggle button", output=False)

    def test_release_frees_the_pin(self):
        pins.claim(4, "program button")
        pins.release(4)
        pins.check(4, "test")


class DriverIntegrationTests(_GuardCase):
    def test_l298n_rejects_flash_pin(self):
        pins.set_chip("esp32s3")
        self.expect_reserved(lambda: L298NMotor(in1=26, in2=1, pwm=17),
                             "SPI flash")

    def test_l298n_rejects_claimed_button_pin(self):
        pins.claim(4, "program button",
                   "launcher.run(button_pin=...) moves it")
        self.expect_reserved(lambda: L298NMotor(in1=4, in2=1, pwm=17),
                             "program button")

    def test_l298n_free_pins_construct(self):
        pins.set_chip("esp32s3")
        L298NMotor(in1=1, in2=2, pwm=17)

    def test_hub_claims_its_pins(self):
        hub = ESP32S3DevkitHub(bluetooth=False)
        self.assertIsNotNone(hub)
        self.expect_reserved(lambda: pins.check(5, "H-bridge IN1"),
                             "Bluetooth-toggle button")

    def test_hub_reconstruction_is_allowed(self):
        ESP32S3DevkitHub(bluetooth=False)
        ESP32S3DevkitHub(bluetooth=False)   # same roles, same pins — fine

    def test_hub_rejects_pin_owned_by_other_role(self):
        pins.claim(18, "program button")
        self.expect_reserved(
            lambda: ESP32S3DevkitHub(bluetooth_button_pin=18,
                                     bluetooth=False),
            "program button")


if __name__ == "__main__":
    unittest.main()
