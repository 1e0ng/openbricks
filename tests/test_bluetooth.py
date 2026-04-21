# SPDX-License-Identifier: MIT
"""Tests for openbricks.bluetooth — persistent BLE on/off state."""

import tests._fakes       # noqa: F401
import tests._fakes_ble   # noqa: F401  (installs fake esp32 + bluetooth modules)

import unittest

from tests._fakes_ble import _FakeBLE, _FakeNVS
from openbricks import bluetooth


class BluetoothStateTests(unittest.TestCase):
    def setUp(self):
        _FakeNVS._reset_for_test()
        _FakeBLE._reset_for_test()

    def test_default_is_enabled_when_no_key_stored(self):
        self.assertTrue(bluetooth.is_enabled())

    def test_set_enabled_true_persists_and_activates(self):
        bluetooth.set_enabled(True)
        self.assertTrue(bluetooth.is_enabled())
        self.assertTrue(_FakeBLE().active())

    def test_set_enabled_false_persists_and_deactivates(self):
        bluetooth.set_enabled(False)
        self.assertFalse(bluetooth.is_enabled())
        self.assertFalse(_FakeBLE().active())

    def test_toggle_flips_and_returns_new_state(self):
        # Default → True, toggle → False.
        new = bluetooth.toggle()
        self.assertFalse(new)
        self.assertFalse(bluetooth.is_enabled())
        self.assertFalse(_FakeBLE().active())
        # Toggle again → True.
        new = bluetooth.toggle()
        self.assertTrue(new)
        self.assertTrue(_FakeBLE().active())

    def test_state_survives_a_fresh_nvs_instance(self):
        """Persistence test: a second ``NVS`` handle (as if after reboot)
        should read the flag we committed earlier."""
        bluetooth.set_enabled(False)
        # Simulate reboot: drop all module-level caches in the fake BLE,
        # but keep NVS (which represents flash — persists across reboot).
        _FakeBLE._reset_for_test()
        self.assertFalse(bluetooth.is_enabled())

    def test_apply_persisted_state_activates_when_stored_on(self):
        bluetooth.set_enabled(True)
        _FakeBLE._reset_for_test()   # simulate reboot clearing radio state
        bluetooth.apply_persisted_state()
        self.assertTrue(_FakeBLE().active())

    def test_apply_persisted_state_deactivates_when_stored_off(self):
        bluetooth.set_enabled(False)
        _FakeBLE._reset_for_test()
        bluetooth.apply_persisted_state()
        self.assertFalse(_FakeBLE().active())

    def test_apply_persisted_state_defaults_on_for_fresh_device(self):
        """No key ever set — apply_persisted_state should enable the radio."""
        bluetooth.apply_persisted_state()
        self.assertTrue(_FakeBLE().active())


if __name__ == "__main__":
    unittest.main()
