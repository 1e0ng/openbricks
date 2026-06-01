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


class RunProgramResetsMotorProcessTests(unittest.TestCase):
    """``run_program`` is the entry point for both BLE-triggered runs
    (``openbricks-dev run``) and button-press launches. Each call must
    wipe the native scheduler's callback list so the new program
    doesn't inherit dead servo / drivebase tick subscriptions from the
    previous run.

    Discovered 2026-05-06: a stale callback list left
    ``DriveBase.straight()`` blocked forever in
    ``while not is_done()`` — the new drivebase's ``register_c`` call
    silently failed (list contaminated with garbage from the
    previous run) and the trajectory tick never fired."""

    def setUp(self):
        self._original = launcher._reset_motor_process
        self._call_count = 0

        def _spy():
            self._call_count += 1

        launcher._reset_motor_process = _spy

    def tearDown(self):
        launcher._reset_motor_process = self._original

    def test_run_program_resets_motor_process_before_exec(self):
        # Tiny program that just touches a marker file; we don't care
        # what it does, only that _reset_motor_process fired before it.
        marker = "/tmp/_openbricks_run_program_marker"
        try:
            os.remove(marker)
        except OSError:
            pass
        self.addCleanup(lambda: os.remove(marker) if _path_exists(marker) else None)
        path = _write_program("open(%r, 'w').write('ran')\n" % marker)
        self.addCleanup(_cleanup_program)

        launcher.run_program(path)

        self.assertEqual(self._call_count, 1,
                         "run_program must call _reset_motor_process exactly once")
        self.assertTrue(_path_exists(marker),
                        "program should still execute after the reset")


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
        # Patch _ensure_launcher to capture the timer_id arg and not
        # actually construct a Timer (which requires the firmware
        # machine module). MicroPython's ``inspect`` is missing
        # ``signature`` so we read defaults via call-site capture.
        captured = {}
        def _stub(button_pin, poll_ms, timer_id):
            captured["timer_id"] = timer_id
            class _Inst:
                def _drain_pending(self):
                    raise KeyboardInterrupt()  # exit the run loop
            inst = _Inst()
            inst._program_path = None
            return inst
        orig = launcher._ensure_launcher
        launcher._ensure_launcher = _stub
        try:
            try:
                launcher.run()
            except KeyboardInterrupt:
                pass
        finally:
            launcher._ensure_launcher = orig
        # ESP32-S3 hardware timers are 0..3; -1 (virtual) raises
        # ``ValueError: invalid Timer number`` on the MP we vendor.
        self.assertGreaterEqual(captured["timer_id"], 0)
        self.assertLessEqual(captured["timer_id"], 3)


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


class EmergencyStopTests(unittest.TestCase):
    """The button-stop path must actually halt the motors, not just
    unwind the program. The injected ``KeyboardInterrupt`` exits the
    program but leaves serial-bus servos spinning at their last command —
    so ``_exec_program_raw``'s KeyboardInterrupt handler calls
    ``_stop_all_motors`` as the interrupt unwinds."""

    def setUp(self):
        from openbricks.drivers.st3215 import ST3215
        ST3215._buses = {}
        self.addCleanup(_cleanup_program)

    def test_request_interrupt_injects_real_keyboard_interrupt(self):
        # On a build with the native hook (the firmware + the native test
        # binary), _request_interrupt arms a real pending KeyboardInterrupt
        # — the Ctrl-C mechanism — which fires at the next bytecode and
        # actually stops a running program. On desktop (no native module)
        # it falls back to the _pending flag. Assert whichever path is live.
        inst = launcher.Launcher(_make_button())
        try:
            from _openbricks_native import request_keyboard_interrupt  # noqa: F401
            has_native = True
        except (ImportError, AttributeError):
            has_native = False

        if has_native:
            fired = False
            try:
                launcher._request_interrupt(inst)
                for _ in range(100000):     # let the pending exception land
                    pass
            except KeyboardInterrupt:
                fired = True
            self.assertTrue(fired, "native hook must inject a KeyboardInterrupt")
        else:
            launcher._request_interrupt(inst)
            self.assertEqual(inst._pending, "stop")

    def test_request_interrupt_calls_native_hook_when_present(self):
        # Exercise the native-hook code path even under CPython (where
        # _openbricks_native is absent) by injecting a fake module. Without
        # this, the ``request_keyboard_interrupt()`` line is only reached
        # in the MP job, which doesn't feed the openbricks-py coverage flag.
        import sys
        rec = []

        class _FakeNative:
            def request_keyboard_interrupt(self_):
                rec.append(1)

        saved = sys.modules.get("_openbricks_native")
        sys.modules["_openbricks_native"] = _FakeNative()
        try:
            inst = launcher.Launcher(_make_button())
            launcher._request_interrupt(inst)
        finally:
            if saved is not None:
                sys.modules["_openbricks_native"] = saved
            else:
                sys.modules.pop("_openbricks_native", None)
        self.assertEqual(rec, [1])        # native hook was called
        self.assertIsNone(inst._pending)  # no fallback flag set

    def test_stop_all_motors_broadcasts_torque_off_to_serial_bus(self):
        from openbricks.drivers.st3215 import ST3215Motor
        m = ST3215Motor(servo_id=1, uart_id=1, tx=17, rx=16)
        baseline = len(m._bus._uart._tx_log)
        launcher._stop_all_motors()
        # Expect a broadcast (ID 0xFE) WRITE (0x03) to the torque
        # register (0x28) with value 0 — coast every servo on the bus.
        found = False
        for pkt in m._bus._uart._tx_log[baseline:]:
            body = pkt[2:-1]
            if (body[0] == 0xFE and body[2] == 0x03 and
                    body[3] == 0x28 and body[4] == 0):
                found = True
        self.assertTrue(found, "expected a broadcast torque-off write")

    def test_exec_program_raw_stops_motors_on_keyboard_interrupt(self):
        calls = []
        original = launcher._stop_all_motors
        launcher._stop_all_motors = lambda: calls.append(1)
        path = _write_program("raise KeyboardInterrupt\n")
        try:
            with self.assertRaises(KeyboardInterrupt):
                launcher._exec_program_raw(path)
        finally:
            launcher._stop_all_motors = original
        self.assertEqual(calls, [1])

    def test_stop_all_motors_is_safe_with_no_motors(self):
        # No buses registered, native module maybe absent — must be a
        # quiet no-op, never raise.
        launcher._stop_all_motors()

    def test_stop_all_motors_tolerates_a_wedged_bus(self):
        # One bus failing to accept the broadcast must not stop us from
        # trying the others — an e-stop tries every avenue.
        from openbricks.drivers.st3215 import ST3215

        class _BoomBus:
            def write(self, *a, **k):
                raise OSError("bus wedged")

        ST3215._buses = {("wedged",): _BoomBus()}
        launcher._stop_all_motors()   # must not raise

    def test_stop_all_motors_survives_st3215_import_failure(self):
        # If the serial-bus driver can't be imported for any reason, the
        # e-stop must still not propagate — the KeyboardInterrupt has to
        # get through to unwind the program.
        import sys
        name = "openbricks.drivers.st3215"
        saved = sys.modules.get(name)

        class _Empty:
            pass

        sys.modules[name] = _Empty()   # lacks ST3215 / regs → import fails
        try:
            launcher._stop_all_motors()   # must not raise
        finally:
            if saved is not None:
                sys.modules[name] = saved
            else:
                sys.modules.pop(name, None)


if __name__ == "__main__":
    unittest.main()
