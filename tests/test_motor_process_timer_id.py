# SPDX-License-Identifier: MIT
"""
Regression guard for the firmware 1.2.6 bug:

``motor_process.c`` used to construct ``machine.Timer(-1)`` (virtual
timer). ESP32-S3 doesn't support virtual timers — only hardware
0..3 — so the call raised ``ValueError`` and the surrounding
``nlr_push`` block silently swallowed it. ``motor_process.start()``
returned, ``is_running()`` reported ``False``, the closed-loop tick
never fired, ``run_speed()`` set the target dps but nothing
actuated it, ``DriveBase.straight()`` blocked forever in
``while not is_done()``.

The bug was invisible to ``tests/test_*`` (unix MicroPython
*does* support virtual timers, so the call succeeded under test).
This file is a static guard on the C source itself — runs under
plain CPython, no MicroPython needed, catches a re-introduction
of ``Timer(-1)`` regardless of which port the test happens to run
on.

Reserved hardware Timer slots on the firmware:

* Timer 0 — ``openbricks.launcher`` (button watcher)
* Timer 1 — ``openbricks.bluetooth_button`` (BLE-toggle)
* Timer 2 — ``motor_process`` (closed-loop scheduler)
* Timer 3 — free
"""

import os
import re
import unittest


_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
_MOTOR_PROCESS_C = os.path.join(
    _REPO_ROOT, "native", "user_c_modules", "openbricks", "motor_process.c"
)


class MotorProcessTimerIDTests(unittest.TestCase):

    def setUp(self):
        with open(_MOTOR_PROCESS_C) as f:
            self.src = f.read()

    def test_motor_process_constructs_a_timer_exactly_once(self):
        """One ``machine.Timer(N)`` call in mp_do_start. If this changes,
        the regex below needs updating."""
        calls = re.findall(
            r"mp_call_function_1\(\s*timer_cls\s*,\s*"
            r"MP_OBJ_NEW_SMALL_INT\(\s*(-?\d+)\s*\)\s*\)",
            self.src,
        )
        self.assertEqual(
            len(calls), 1,
            "expected exactly one machine.Timer(...) construction in "
            "motor_process.c; if you've refactored the call shape, update "
            "this regression test alongside it. Got: %r" % calls
        )

    def test_motor_process_uses_hardware_timer_id(self):
        """Timer ID must be in [0..3] — ESP32 and ESP32-S3 only have
        hardware timers in that range. ``-1`` (virtual) raises
        ValueError on ESP32-S3 and is silently swallowed in mp_do_start."""
        m = re.search(
            r"mp_call_function_1\(\s*timer_cls\s*,\s*"
            r"MP_OBJ_NEW_SMALL_INT\(\s*(-?\d+)\s*\)\s*\)",
            self.src,
        )
        self.assertIsNotNone(m, "no Timer constructor call found")
        timer_id = int(m.group(1))
        self.assertGreaterEqual(
            timer_id, 0,
            "motor_process.c constructs machine.Timer(%d). Virtual timers "
            "(negative IDs) are not supported on ESP32-S3 — the constructor "
            "raises ValueError and our nlr_push block swallows it, leaving "
            "the closed-loop tick permanently dormant. Use a hardware ID "
            "(0..3)." % timer_id
        )
        self.assertLessEqual(
            timer_id, 3,
            "Timer ID %d > 3 — ESP32 / ESP32-S3 only expose hardware "
            "timers 0..3." % timer_id
        )

    def test_motor_process_does_not_collide_with_launcher_or_bluetooth_button(self):
        """Timer 0 is the launcher button watcher
        (``openbricks.launcher.run``); Timer 1 is the BLE-toggle
        (``openbricks.bluetooth_button.BluetoothToggleButton``). A
        collision means whichever subsystem comes up second silently
        wedges the first."""
        m = re.search(
            r"mp_call_function_1\(\s*timer_cls\s*,\s*"
            r"MP_OBJ_NEW_SMALL_INT\(\s*(-?\d+)\s*\)\s*\)",
            self.src,
        )
        timer_id = int(m.group(1))
        self.assertNotIn(
            timer_id, (0, 1),
            "Timer ID %d collides with %s. Reserved slots: 0=launcher, "
            "1=bluetooth_button. motor_process should use Timer 2."
            % (timer_id, "launcher" if timer_id == 0 else "bluetooth_button")
        )


if __name__ == "__main__":
    unittest.main()
