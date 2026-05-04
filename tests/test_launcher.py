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
    """Launcher._tick should mark short presses via ``_request_start``
    (flag-fallback on CPython, scheduled callback on MP) and skip
    presses that cross the long-press threshold."""

    def setUp(self):
        self.btn = _make_button()
        self.launcher = launcher.Launcher(
            self.btn,
            program_path="/nonexistent.py",
            poll_ms=50,
        )
        # Patch ``_request_start`` so these tests are runtime-agnostic.
        # On MP, the real implementation calls ``micropython.schedule``
        # which would otherwise bypass the ``_pending`` flag these
        # tests assert on.
        self._original_request_start = launcher._request_start
        launcher._request_start = lambda inst: setattr(inst, "_pending", "start")
        self.addCleanup(setattr, launcher, "_request_start", self._original_request_start)

    def test_short_press_queues_start(self):
        # On CPython (no ``micropython.schedule``), ``_request_start``
        # falls back to setting the ``_pending`` flag the main loop
        # drains. On MP it would schedule directly instead.
        _press(self.btn, hold_ms=200, tick_fn=self.launcher._tick)
        self.assertEqual(self.launcher._pending, "start")

    def test_short_press_while_idle_invokes_request_start(self):
        # Regardless of runtime, ``_request_start`` is the single
        # dispatch point for a short press while idle. Swap it for a
        # recorder to verify.
        calls = []
        original = launcher._request_start
        launcher._request_start = lambda inst: calls.append(inst)
        try:
            _press(self.btn, hold_ms=200, tick_fn=self.launcher._tick)
        finally:
            launcher._request_start = original

        self.assertEqual(len(calls), 1)
        self.assertIs(calls[0], self.launcher)

    def test_long_hold_still_fires_on_release(self):
        # On the program pin we don't distinguish long vs short — any
        # press-release cycle dispatches. The BLE-toggle button lives
        # on a separate pin, so this one never needs to skip long holds.
        _press(self.btn, hold_ms=1500, tick_fn=self.launcher._tick)
        self.assertEqual(self.launcher._pending, "start")

    def test_release_without_press_is_noop(self):
        # Button never pressed — ticks should produce no state change.
        for _ in range(5):
            self.launcher._tick()
            advance_ms(50)
        self.assertIsNone(self.launcher._pending)
        self.assertFalse(self.launcher._was_pressed)


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
            self.btn, program_path="/ignored.py", poll_ms=50)
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


class ScheduledStartTests(unittest.TestCase):
    """``_scheduled_start`` is the callback MicroPython runs between
    bytecodes after a short-press-while-idle. It execs the staged
    program and leaves the launcher back in the idle state, so
    pressing the button AGAIN starts it again — even when main.py's
    idle loop isn't running (which is the post-``openbricks-dev run``
    state)."""

    def setUp(self):
        self.btn = _make_button()
        self.launcher = launcher.Launcher(
            self.btn, program_path="/ignored")
        self.addCleanup(_cleanup_program)

    def test_scheduled_start_exec_s_the_program(self):
        marker = "/tmp/_openbricks_scheduled_start_marker"
        try:
            os.remove(marker)
        except OSError:
            pass
        self.addCleanup(
            lambda: os.remove(marker) if _path_exists(marker) else None)
        self.launcher._program_path = _write_program(
            "open(%r, 'w').write('ran')\n" % marker)
        launcher._scheduled_start(self.launcher)
        self.assertTrue(_path_exists(marker))
        self.assertFalse(self.launcher._running)

    def test_scheduled_start_is_noop_when_already_running(self):
        # A second scheduled_start while the first is still mid-exec
        # must not re-enter. Simulate by pre-setting _running and
        # asserting the function bails before touching the program file.
        self.launcher._program_path = "/bogus/no/such/file.py"
        self.launcher._running = True
        # Should not raise OSError on the nonexistent file — the early
        # return fires first.
        launcher._scheduled_start(self.launcher)
        self.assertTrue(self.launcher._running)  # unchanged


class TimerIdDefaultTests(unittest.TestCase):
    """Pin the timer_id defaults to a hardware-valid number on the
    esp32-s3 we ship for. Older MicroPython supported ``Timer(-1)``
    as a virtual software timer; the v1.27+ MP we vendor raises
    ``ValueError: invalid Timer number`` for anything outside 0..3.
    Pre-1.0.9 the defaults were -1 and main.py bricked at boot."""

    def test_run_default_timer_id_is_hardware_valid(self):
        # Inspect the function signature so we don't have to actually
        # construct a Timer (which requires the firmware machine module).
        # ``run`` should default to a small non-negative integer.
        import inspect
        sig = inspect.signature(launcher.run)
        default = sig.parameters["timer_id"].default
        self.assertGreaterEqual(default, 0)
        self.assertLessEqual(default, 3)

    def test_ensure_launcher_default_timer_id_is_hardware_valid(self):
        import inspect
        sig = inspect.signature(launcher._ensure_launcher)
        default = sig.parameters["timer_id"].default
        self.assertGreaterEqual(default, 0)
        self.assertLessEqual(default, 3)


class RunProgramTests(unittest.TestCase):
    """``launcher.run_program`` is the entry point ``openbricks-dev run``
    jumps into via raw REPL. Must set ``_running`` so button-stop works,
    propagate ``KeyboardInterrupt``, and swallow other exceptions (the
    user sees the traceback via the normal REPL error-framing path)."""

    def setUp(self):
        # The singleton is module-level; clear it so each test starts clean.
        launcher._singleton = None
        self.addCleanup(_cleanup_program)

    def _fake_ensure(self):
        """Patch _ensure_launcher to install a Launcher around a fake
        Pin — the real helper would try to create a machine.Pin + Timer."""
        btn = _make_button()
        inst = launcher.Launcher(btn)
        launcher._singleton = inst
        return inst

    def test_run_program_sets_running_flag_during_exec(self):
        inst = self._fake_ensure()
        # Program observes _running via the shared singleton.
        marker = "/tmp/_openbricks_run_program_marker"
        try:
            os.remove(marker)
        except OSError:
            pass
        self.addCleanup(lambda: os.remove(marker) if _path_exists(marker) else None)
        path = _write_program(
            "from openbricks import launcher as _l\n"
            "assert _l._singleton._running is True\n"
            "open(%r, 'w').write('ok')\n" % marker
        )
        launcher.run_program(path)
        self.assertTrue(_path_exists(marker))
        # Running flag cleared on exit.
        self.assertFalse(inst._running)

    def test_run_program_propagates_keyboard_interrupt(self):
        self._fake_ensure()
        path = _write_program("raise KeyboardInterrupt\n")
        with self.assertRaises(KeyboardInterrupt):
            launcher.run_program(path)
        # Running flag cleared even when exec raises.
        self.assertFalse(launcher._singleton._running)

    def test_run_program_swallows_other_exceptions(self):
        self._fake_ensure()
        path = _write_program("raise ValueError('boom')\n")
        # Should not raise — the exception is printed via _exec_program_raw.
        launcher.run_program(path)
        self.assertFalse(launcher._singleton._running)


if __name__ == "__main__":
    unittest.main()
