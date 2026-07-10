# SPDX-License-Identifier: MIT
"""Hardware-timer allocation: every runtime subsystem gets its own.

The ESP32 / ESP32-S3 have exactly four hardware timers (0..3), and
``machine.Timer(n).init()`` silently reconfigures timer ``n`` if it is
already running — the previous owner's callback just stops firing, no
error. That's how the 1.9.0 stop tick (installed on ``timer_id + 1``
= 1) killed the BLE-toggle button: the toggle wired Timer 1 at boot,
the launcher started afterwards and re-inited Timer 1 with the
stop-tick callback, and from then on pressing the BLE button did
nothing — no toggle, no LED repaint.

Reserved inventory: 0 = launcher poll, 1 = BLE toggle,
2 = motor_process, 3 = stop tick.
"""

import tests._fakes      # noqa: F401
import tests._fakes_ble  # noqa: F401  (toggle.start touches bluetooth state)

import unittest

from machine import Timer

from openbricks import launcher
from openbricks.bluetooth_button import BluetoothToggleButton


class _StubButton:
    def pressed(self):
        return False


class StopTickTimerIdTests(unittest.TestCase):
    def test_stop_timer_id_constant_is_3(self):
        self.assertEqual(launcher.STOP_TIMER_ID, 3)

    def test_stop_timer_avoids_every_other_subsystem(self):
        # assertFalse(... in ...): MicroPython's unittest has no
        # assertNotIn.
        self.assertFalse(
            launcher.STOP_TIMER_ID in (0, 1, 2),
            "STOP_TIMER_ID %d collides with a reserved slot "
            "(0=launcher, 1=bluetooth_button, 2=motor_process)"
            % launcher.STOP_TIMER_ID)


class BootOrderAllocationTests(unittest.TestCase):
    """Wire the BLE toggle then the launcher — the boot order of the
    frozen main.py — and assert no two active timers share a hardware
    ID. This is the test that fails on the pre-fix code (stop tick on
    Timer 1, same as the toggle)."""

    def setUp(self):
        from openbricks import pins
        Timer.reset_for_test()
        launcher._singleton = None
        pins._claims_reset()

    def tearDown(self):
        from openbricks import pins
        launcher._singleton = None
        Timer.reset_for_test()
        pins._claims_reset()

    def test_no_two_active_timers_share_a_hardware_id(self):
        toggle = BluetoothToggleButton(_StubButton())
        toggle.start()
        self.addCleanup(toggle.stop)

        inst = launcher._ensure_launcher()
        self.assertIsNotNone(inst)

        active_ids = [t._id for t in Timer._instances if t._active]
        self.assertEqual(
            len(active_ids), len(set(active_ids)),
            "two subsystems initialized the same hardware timer — the "
            "second init silently kills the first owner's callback. "
            "Active IDs: %r" % sorted(active_ids))


if __name__ == "__main__":
    unittest.main()
