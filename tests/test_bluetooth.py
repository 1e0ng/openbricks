# SPDX-License-Identifier: MIT
"""Tests for openbricks.bluetooth — persistent BLE on/off state."""

import tests._fakes       # noqa: F401
import tests._fakes_ble   # noqa: F401  (installs fake esp32 + bluetooth modules)

import unittest

from tests._fakes_ble import _FakeBLE, _FakeNVS
import openbricks
from openbricks import bluetooth


def _set_hub_name(name):
    """Helper: plant a hub name into the fake NVS under the same
    namespace/key that ``openbricks._read_hub_name`` reads from."""
    nvs = _FakeNVS(openbricks._HUB_NAME_NVS_NAMESPACE)
    nvs.set_blob(openbricks._HUB_NAME_NVS_KEY, name.encode())
    nvs.commit()


class BluetoothStateTests(unittest.TestCase):
    def setUp(self):
        _FakeNVS._reset_for_test()
        _FakeBLE._reset_for_test()
        # A name must be flashed for BLE to activate; plant one for the
        # tests that enable the radio. Tests exercising the no-name
        # error path clear it via _FakeNVS._reset_for_test() again.
        _set_hub_name("TestHub")

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

    def test_set_enabled_also_configures_gap_name_from_hub_name(self):
        """``openbricks.HUB_NAME`` should be pushed into the BLE GAP config
        before ``active(True)`` so advertisement carries the flashed name."""
        bluetooth.set_enabled(True)
        self.assertEqual(_FakeBLE._gap_name, "TestHub")

    def test_apply_persisted_state_also_configures_gap_name(self):
        bluetooth.apply_persisted_state()
        self.assertEqual(_FakeBLE._gap_name, "TestHub")


class HubNameRequiredTests(unittest.TestCase):
    """No hub name flashed → activating BLE must raise, never advertise
    under a shared default. Deactivating is always permitted."""

    def setUp(self):
        _FakeNVS._reset_for_test()  # clear any planted hub name
        _FakeBLE._reset_for_test()

    def test_hub_name_is_none_when_unflashed(self):
        self.assertIsNone(openbricks.HUB_NAME)

    def test_set_enabled_true_raises_without_hub_name(self):
        with self.assertRaises(bluetooth.HubNameNotSetError):
            bluetooth.set_enabled(True)
        # The BLE stack must not have been activated.
        self.assertFalse(_FakeBLE().active())

    def test_set_enabled_true_does_not_persist_on_error(self):
        """Failing to activate should not leave ``ble_enabled=1`` in NVS —
        otherwise the next boot would re-raise during apply_persisted_state
        even if the user explicitly turned BLE off later."""
        try:
            bluetooth.set_enabled(True)
        except bluetooth.HubNameNotSetError:
            pass
        # is_enabled() should still return the default (True) since no
        # value was committed, but the radio is off. The key property:
        # no stale ``1`` was written before the raise.
        nvs = _FakeNVS("openbricks")
        with self.assertRaises(OSError):
            nvs.get_i32("ble_enabled")

    def test_set_enabled_false_works_without_hub_name(self):
        """Silencing a nameless hub is always allowed."""
        bluetooth.set_enabled(False)
        self.assertFalse(_FakeBLE().active())
        self.assertFalse(bluetooth.is_enabled())

    def test_apply_persisted_state_raises_when_default_on_without_name(self):
        """Fresh device (no NVS keys) defaults to BLE-on, which needs a name."""
        with self.assertRaises(bluetooth.HubNameNotSetError):
            bluetooth.apply_persisted_state()

    def test_apply_persisted_state_ok_when_off_without_name(self):
        # Persist "off" via raw NVS write so we don't go through the
        # name-requiring set_enabled() path.
        nvs = _FakeNVS("openbricks")
        nvs.set_i32("ble_enabled", 0)
        nvs.commit()
        bluetooth.apply_persisted_state()
        self.assertFalse(_FakeBLE().active())


class HubNameReadTests(unittest.TestCase):
    """``openbricks.HUB_NAME`` reads freshly from NVS on each access."""

    def setUp(self):
        _FakeNVS._reset_for_test()

    def test_reflects_nvs_value(self):
        nvs = _FakeNVS(openbricks._HUB_NAME_NVS_NAMESPACE)
        nvs.set_blob(openbricks._HUB_NAME_NVS_KEY, b"RobotA")
        nvs.commit()
        self.assertEqual(openbricks.HUB_NAME, "RobotA")

    def test_returns_none_when_nvs_key_missing(self):
        self.assertIsNone(openbricks.HUB_NAME)

    def test_picks_up_rename_without_reimport(self):
        """A rename after import should be visible immediately — the
        module ``__getattr__`` does a fresh NVS read on every access."""
        nvs = _FakeNVS(openbricks._HUB_NAME_NVS_NAMESPACE)
        nvs.set_blob(openbricks._HUB_NAME_NVS_KEY, b"before")
        nvs.commit()
        self.assertEqual(openbricks.HUB_NAME, "before")
        nvs.set_blob(openbricks._HUB_NAME_NVS_KEY, b"after")
        nvs.commit()
        self.assertEqual(openbricks.HUB_NAME, "after")


if __name__ == "__main__":
    unittest.main()
