# SPDX-License-Identifier: MIT
"""Tests for openbricks.launcher — button-gated program runner."""

import tests._fakes  # noqa: F401

import os
import unittest

from machine import Pin

from openbricks import launcher
from tests._fakes import advance_ms


_TEST_PROGRAM_PATH = "/tmp/_openbricks_launcher_test.py"


def _write_program(source):
    # MicroPython unix doesn't ship ``tempfile``; use a known path and
    # clean up in the test's addCleanup.
    with open(_TEST_PROGRAM_PATH, "w") as f:
        f.write(source)
    return _TEST_PROGRAM_PATH


def _cleanup_program():
    try:
        os.remove(_TEST_PROGRAM_PATH)
    except OSError:
        pass


def _path_exists(p):
    try:
        os.stat(p)
        return True
    except OSError:
        return False


def _make_button(initial_value=1):
    """Fake Pin acting as an active-low button. Default 1 = released."""
    p = Pin(5, Pin.IN, Pin.PULL_UP)
    p._value = initial_value
    return p


def _press(button, hold_ms, poll_ms=50, tick_fn=None):
    """Hold ``button`` for ``hold_ms``, then release. While held,
    call ``tick_fn`` every ``poll_ms`` to mimic the Timer callback."""
    button._value = 0  # pressed (active-low)
    elapsed = 0
    while elapsed < hold_ms:
        if tick_fn is not None:
            tick_fn()
        advance_ms(poll_ms)
        elapsed += poll_ms
    if tick_fn is not None:
        tick_fn()
    button._value = 1  # released
    if tick_fn is not None:
        tick_fn()


class PressClassificationTests(unittest.TestCase):
    """Launcher._tick should mark short presses as ``_pending = 'start'``
    and skip presses that cross the long-press threshold."""

    def setUp(self):
        self.btn = _make_button()
        self.launcher = launcher.Launcher(
            self.btn,
            program_path="/nonexistent.py",
            poll_ms=50,
            long_press_ms=1000,
        )

    def test_short_press_queues_start(self):
        _press(self.btn, hold_ms=200, tick_fn=self.launcher._tick)
        self.assertEqual(self.launcher._pending, "start")

    def test_long_press_does_not_queue_start(self):
        # Held past 1 s threshold → BLE watcher's territory.
        _press(self.btn, hold_ms=1500, tick_fn=self.launcher._tick)
        self.assertIsNone(self.launcher._pending)

    def test_borderline_press_below_threshold_is_short(self):
        # Just under the 1000 ms threshold still counts as short.
        _press(self.btn, hold_ms=900, tick_fn=self.launcher._tick)
        self.assertEqual(self.launcher._pending, "start")

    def test_release_without_press_is_noop(self):
        # Button never pressed — ticks should produce no state change.
        for _ in range(5):
            self.launcher._tick()
            advance_ms(50)
        self.assertIsNone(self.launcher._pending)
        self.assertIsNone(self.launcher._press_start_ms)


class DrainAndExecTests(unittest.TestCase):
    """The queued 'start' is consumed by ``_drain_pending``, which
    loads and execs the program file."""

    def setUp(self):
        self.btn = _make_button()
        self.addCleanup(_cleanup_program)

    def _write_program(self, source):
        return _write_program(source)

    def test_drain_execs_the_staged_program(self):
        # The exec'd program writes a marker file; we check it exists
        # after drain. Side-effecting via the filesystem works on both
        # CPython and MicroPython without depending on mutable builtins.
        marker = "/tmp/_openbricks_launcher_marker"
        try:
            os.remove(marker)
        except OSError:
            pass
        self.addCleanup(lambda: os.remove(marker) if _path_exists(marker) else None)
        path = self._write_program(
            "open(%r, 'w').write('ran')\n" % marker)
        launch = launcher.Launcher(self.btn, program_path=path)
        launch._pending = "start"
        launch._drain_pending()
        self.assertTrue(_path_exists(marker))

    def test_drain_clears_running_flag_after_exec(self):
        path = self._write_program("pass\n")
        launch = launcher.Launcher(self.btn, program_path=path)
        launch._pending = "start"
        launch._drain_pending()
        self.assertFalse(launch._running)

    def test_drain_survives_exception_in_program(self):
        # A raising program must not kill the launcher loop.
        path = self._write_program("raise ValueError('boom')\n")
        launch = launcher.Launcher(self.btn, program_path=path)
        launch._pending = "start"
        # Should not raise — exception is caught and printed.
        launch._drain_pending()
        self.assertFalse(launch._running)

    def test_drain_handles_missing_program_file(self):
        launch = launcher.Launcher(self.btn, program_path="/does/not/exist.py")
        launch._pending = "start"
        # Should not raise — prints a diagnostic and clears state.
        launch._drain_pending()
        self.assertFalse(launch._running)

    def test_drain_with_no_pending_is_noop(self):
        launch = launcher.Launcher(self.btn, program_path="/no.py")
        launch._drain_pending()   # no-op
        self.assertFalse(launch._running)


class StopRequestTests(unittest.TestCase):
    """Short press while ``_running`` is True requests mid-run stop.

    MicroPython hands the request to ``micropython.schedule`` so the
    ``KeyboardInterrupt`` lands between bytecodes of the running
    program. CPython (where ``schedule`` doesn't exist) falls back to
    a ``_pending = 'stop'`` flag on the launcher. We assert on
    whichever path is live.
    """

    def setUp(self):
        self.btn = _make_button()
        self.launcher = launcher.Launcher(
            self.btn, program_path="/ignored.py",
            poll_ms=50, long_press_ms=1000)
        # Pretend a program is already executing.
        self.launcher._running = True

    def test_short_press_while_running_calls_request_interrupt(self):
        # Swap the module-level helper for a recorder so we see the
        # call without letting the real raise fire (and without touching
        # ``micropython.schedule``, which is read-only on MP).
        calls = []
        original = launcher._request_interrupt
        launcher._request_interrupt = lambda inst: calls.append(inst)
        try:
            _press(self.btn, hold_ms=200, tick_fn=self.launcher._tick)
        finally:
            launcher._request_interrupt = original

        self.assertEqual(len(calls), 1)
        self.assertIs(calls[0], self.launcher)


if __name__ == "__main__":
    unittest.main()
