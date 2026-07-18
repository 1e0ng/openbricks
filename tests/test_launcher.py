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


def _tick_debounced(launcher_inst, ticks=None):
    """Tick enough consecutive polls for the debounced level logic
    to accept the current button state (1.15.3)."""
    from openbricks.launcher import Launcher
    for _ in range(ticks or Launcher.DEBOUNCE_TICKS):
        launcher_inst._tick()
        advance_ms(50)


def _make_button(initial_value=1):
    """Fake Pin acting as an active-low button. Default 1 = released."""
    p = Pin(5, Pin.IN, Pin.PULL_UP)
    p._value = initial_value
    return p


def _press(button, hold_ms, poll_ms=50, tick_fn=None):
    """Hold ``button`` for ``hold_ms``, then release. While held,
    call ``tick_fn`` every ``poll_ms`` to mimic the Timer callback.
    Transitions tick DEBOUNCE_TICKS times so the debounced level
    logic (1.15.3) accepts them, exactly as a real held press or
    clean release would over consecutive polls."""
    from openbricks.launcher import Launcher
    button._value = 0  # pressed (active-low)
    elapsed = 0
    ticks = 0
    while elapsed < hold_ms or ticks < Launcher.DEBOUNCE_TICKS:
        if tick_fn is not None:
            tick_fn()
        advance_ms(poll_ms)
        elapsed += poll_ms
        ticks += 1
    button._value = 1  # released
    for _ in range(Launcher.DEBOUNCE_TICKS):
        if tick_fn is not None:
            tick_fn()
        advance_ms(poll_ms)


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


class StopRestartRegressionTests(unittest.TestCase):
    """Pressing stop must not restart the program.

    The stop unwinds the program within milliseconds of the press, so
    ``_running`` is already False by the time the button is released
    (or bounces). The release/bounce then read as a fresh idle press
    and restarted the program the user just stopped. Fixed by firing
    the stop on press-down + consuming its release + a post-stop
    lockout on starts."""

    def setUp(self):
        from openbricks import estop
        estop.clear()
        self.addCleanup(estop.clear)
        self.btn = _make_button()
        self.launcher = launcher.Launcher(
            self.btn, program_path="/ignored.py", poll_ms=50)
        self.starts = []
        self.stops = []
        self._orig_start = launcher._request_start
        self._orig_stop = launcher._request_stop
        launcher._request_start = lambda inst: self.starts.append(inst)
        launcher._request_stop = lambda inst: self.stops.append(inst)
        self.addCleanup(
            setattr, launcher, "_request_start", self._orig_start)
        self.addCleanup(
            setattr, launcher, "_request_stop", self._orig_stop)

    def test_stop_fires_on_press_down_not_release(self):
        self.launcher._running = True
        self.btn._value = 0          # pressed, not yet released
        _tick_debounced(self.launcher)
        self.assertEqual(len(self.stops), 1,
                         "stop must fire the moment the press lands")

    def test_release_after_stop_does_not_start(self):
        self.launcher._running = True
        self.btn._value = 0
        _tick_debounced(self.launcher)        # stop fires; program unwinds...
        self.launcher._running = False
        advance_ms(100)
        self.btn._value = 1          # ...then the button is released
        _tick_debounced(self.launcher)
        self.assertEqual(self.starts, [],
                         "the stop press's own release restarted the "
                         "program")
        self.assertEqual(len(self.stops), 1)

    def test_bounce_after_stop_is_swallowed_by_lockout(self):
        self.launcher._running = True
        _press(self.btn, hold_ms=100, tick_fn=self.launcher._tick)
        self.launcher._running = False
        # Contact bounce / finger re-contact ~200 ms after the stop.
        advance_ms(200)
        _press(self.btn, hold_ms=100, tick_fn=self.launcher._tick)
        self.assertEqual(self.starts, [],
                         "a press right after the stop restarted the "
                         "program")

    def test_start_works_again_after_lockout(self):
        self.launcher._running = True
        _press(self.btn, hold_ms=100, tick_fn=self.launcher._tick)
        self.launcher._running = False
        advance_ms(launcher.Launcher.START_LOCKOUT_MS + 100)
        _press(self.btn, hold_ms=100, tick_fn=self.launcher._tick)
        self.assertEqual(len(self.starts), 1,
                         "a deliberate press after the lockout must "
                         "still start the program")

    def test_full_stop_press_cycle_fires_exactly_one_stop(self):
        # Hold shorter than STOP_RETRY_MS: a single press-release must
        # produce exactly one request (the press-down one) — the
        # release must not add a second. Longer holds legitimately
        # re-request while the program survives (see StopRetryTests).
        self.launcher._running = True
        _press(self.btn, hold_ms=100, tick_fn=self.launcher._tick)
        self.assertEqual(len(self.stops), 1)
        self.assertEqual(self.starts, [])


class StartLockoutTests(unittest.TestCase):
    """The post-stop lockout is anchored at the stopping press's
    RELEASE and sized at 400 ms — a deliberate quick restart after
    that works, and a swallowed press is announced + recorded instead
    of silently reading as \'start button not detected\'."""

    def setUp(self):
        self.btn = _make_button()
        self.launcher = launcher.Launcher(
            self.btn, program_path="/ignored.py", poll_ms=50)
        self.starts = []
        self._orig_start = launcher._request_start
        self._orig_stop = launcher._request_stop
        launcher._request_start = lambda inst: self.starts.append(inst)
        launcher._request_stop = lambda inst: None
        self.addCleanup(
            setattr, launcher, "_request_start", self._orig_start)
        self.addCleanup(
            setattr, launcher, "_request_stop", self._orig_stop)

    def _stop_cycle(self):
        self.launcher._running = True
        _press(self.btn, hold_ms=100, tick_fn=self.launcher._tick)
        self.launcher._running = False

    def test_quick_deliberate_restart_after_lockout_works(self):
        # The 750ms-from-fire lockout swallowed this exact press.
        self._stop_cycle()
        advance_ms(launcher.Launcher.START_LOCKOUT_MS + 60)
        _press(self.btn, hold_ms=100, tick_fn=self.launcher._tick)
        self.assertEqual(len(self.starts), 1)

    def test_lockout_anchored_at_release_not_fire(self):
        # Long-held stop press: fire at press-down, release 1 s later.
        # A press 200 ms after the RELEASE is inside the window even
        # though it is far past fire+400.
        self.launcher._running = True
        _press(self.btn, hold_ms=1000, tick_fn=self.launcher._tick)
        self.launcher._running = False
        advance_ms(100)
        _press(self.btn, hold_ms=100, tick_fn=self.launcher._tick)
        self.assertEqual(self.starts, [])

    def test_swallowed_press_is_announced_and_recorded(self):
        import builtins
        self._stop_cycle()
        advance_ms(50)
        printed = []
        orig_print = builtins.print
        builtins.print = lambda *a, **k: printed.append(
            " ".join(str(x) for x in a))
        try:
            _press(self.btn, hold_ms=100, tick_fn=self.launcher._tick)
        finally:
            builtins.print = orig_print
        self.assertEqual(self.starts, [])
        self.assertTrue(
            any("start press ignored" in s for s in printed), printed)
        tags = [e[1:] for e in launcher._EVENTS]
        self.assertTrue(
            any(tag == "release" and args and args[0] == "lockout-swallowed"
                for tag, args in tags), launcher._EVENTS)


class LauncherEventRingTests(unittest.TestCase):
    """The always-on button event ring — idle presses have no run log,
    this is their only trace."""

    def setUp(self):
        del launcher._EVENTS[:]
        launcher._EVENTS_NEXT[0] = 0
        self.addCleanup(lambda: (launcher._EVENTS.clear(),))

    def test_records_press_and_release(self):
        btn = _make_button()
        inst = launcher.Launcher(btn, program_path="/x.py", poll_ms=50)
        orig = launcher._request_start
        launcher._request_start = lambda i: None
        try:
            _press(btn, hold_ms=100, tick_fn=inst._tick)
        finally:
            launcher._request_start = orig
        tags = [e[1] for e in launcher._EVENTS]
        self.assertIn("press-down", tags)
        self.assertIn("release", tags)

    def test_ring_wraps(self):
        for i in range(launcher._EVENTS_MAX + 10):
            launcher._event("ev", i)
        self.assertEqual(len(launcher._EVENTS), launcher._EVENTS_MAX)

    def test_dump_events_prints(self):
        import builtins
        launcher._event("hello", 1)
        lines = []
        orig_print = builtins.print
        builtins.print = lambda *a, **k: lines.append(
            " ".join(str(x) for x in a))
        try:
            launcher.dump_events()
        finally:
            builtins.print = orig_print
        self.assertEqual(len(lines), 2)
        self.assertIn("hello", lines[1])


class StopRetryTests(unittest.TestCase):
    """A stop request must be re-injected until the program dies —
    a one-shot request is lost whenever the injected KeyboardInterrupt
    lands in a scheduled callback instead of the program (the
    "first press ignored, second press works" bug)."""

    def setUp(self):
        from openbricks import estop
        estop.clear()
        self.addCleanup(estop.clear)
        self.btn = _make_button()
        self.launcher = launcher.Launcher(
            self.btn, program_path="/ignored.py", poll_ms=50)
        self.stops = []
        self._orig_stop = launcher._request_stop
        launcher._request_stop = lambda inst: self.stops.append(inst)
        self.addCleanup(
            setattr, launcher, "_request_stop", self._orig_stop)

    def test_stop_is_retried_while_program_survives(self):
        self.launcher._running = True
        self.btn._value = 0
        _tick_debounced(self.launcher)                 # press: first request
        self.assertEqual(len(self.stops), 1)
        self.btn._value = 1
        for _ in range(20):                   # program stays alive 1 s
            advance_ms(50)
            self.launcher._tick()
        self.assertTrue(
            len(self.stops) >= 3,
            "expected retries every %d ms while the program survives; "
            "got %d request(s)" % (launcher.Launcher.STOP_RETRY_MS,
                                   len(self.stops)))

    def test_retries_cease_once_program_is_dead(self):
        self.launcher._running = True
        self.btn._value = 0
        _tick_debounced(self.launcher)
        self.btn._value = 1
        _tick_debounced(self.launcher)
        self.launcher._running = False        # program died
        n = len(self.stops)
        for _ in range(20):
            advance_ms(50)
            self.launcher._tick()
        self.assertEqual(len(self.stops), n,
                         "no retries after the program is dead")

    def test_press_engages_the_estop_latch(self):
        from openbricks import estop
        self.launcher._running = True
        self.btn._value = 0
        _tick_debounced(self.launcher)
        self.assertTrue(estop.is_engaged(),
                        "press-down must latch the e-stop immediately")


class DisarmBeforeCleanupTests(unittest.TestCase):
    """The KeyboardInterrupt handler must disarm the stop button
    BEFORE stopping motors — a retry landing inside the cleanup would
    abort the very motor-stop it asked for."""

    def setUp(self):
        self.addCleanup(_cleanup_program)

    def test_disarm_precedes_motor_stop(self):
        order = []
        orig_arm = launcher._arm_stop_button
        orig_stop_motors = launcher._stop_all_motors
        launcher._arm_stop_button = lambda armed: order.append(
            ("arm", armed))
        launcher._stop_all_motors = lambda: order.append(("motors",))
        self.addCleanup(
            setattr, launcher, "_arm_stop_button", orig_arm)
        self.addCleanup(
            setattr, launcher, "_stop_all_motors", orig_stop_motors)

        path = _write_program("raise KeyboardInterrupt\n")
        try:
            launcher._exec_program_raw(path)
        except KeyboardInterrupt:
            pass
        self.assertTrue(("motors",) in order)
        first_disarm = order.index(("arm", False))
        motors = order.index(("motors",))
        self.assertTrue(first_disarm < motors,
                        "disarm must come before motor cleanup: %r"
                        % order)


class EStopNoInjectionIntegrationTests(unittest.TestCase):
    """THE 'always works' test: the interrupt injection is disabled
    entirely (every request dropped — the worst-case eaten-interrupt
    world), the program drives a motor in a loop, the button is
    pressed mid-run — and the program must still terminate, because
    the e-stop latch turns its own next motor command into the stop."""

    def setUp(self):
        from openbricks import estop
        estop.clear()
        self.addCleanup(estop.clear)
        self.addCleanup(_cleanup_program)
        self.btn = _make_button()
        self.launcher = launcher.Launcher(
            self.btn, program_path=None, poll_ms=50)
        launcher._singleton = self.launcher
        self.addCleanup(setattr, launcher, "_singleton", None)
        # Injection permanently dead.
        self._orig_stop = launcher._request_stop
        launcher._request_stop = lambda inst: None
        self.addCleanup(
            setattr, launcher, "_request_stop", self._orig_stop)

    def test_motor_program_stops_without_any_injection(self):
        from openbricks import estop
        path = _write_program(
            "from openbricks import launcher\n"
            "from openbricks.drivers.l298n import L298NMotor\n"
            "inst = launcher._singleton\n"
            "m = L298NMotor(in1=1, in2=2, pwm=17)\n"
            "inst._test_iterations = 0\n"
            "for i in range(1000):\n"
            "    inst._test_iterations = i\n"
            "    if i == 3:\n"
            "        inst._btn._value = 0\n"   # the press lands
            "    inst._tick()\n"               # timer keeps polling
            "    m.run(30)\n"
        )
        self.launcher._program_path = path
        self.launcher._pending = "start"
        self.launcher._drain_pending()

        # i==3 presses; the debounce (1.15.3) confirms it on the
        # NEXT tick, so the synchronous estop raise lands on i==4's
        # motor command.
        self.assertEqual(
            getattr(self.launcher, "_test_iterations", None), 4,
            "program must die on its first motor command after the "
            "debounced press — with zero interrupt deliveries")
        self.assertFalse(self.launcher._running)
        self.assertFalse(
            estop.is_engaged(),
            "latch must be released once the program is dead")


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
    (``openbricks run``) and button-press launches. Each call must
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


class TickPumpsBleTxTests(unittest.TestCase):
    """Every ``_tick`` must pump ``ble_repl``'s TX buffer.

    The pump is the liveness backstop for a flush chain that died with
    bytes still buffered (scheduler queue full, or a paced
    notify-failure retry): without it, a finished writer — e.g. the
    ``openbricks log`` dump program after its last print — leaves the
    tail (and the raw-REPL terminator) stuck in ``_tx_buf`` forever."""

    def setUp(self):
        self.btn = _make_button()
        self.launcher = launcher.Launcher(
            self.btn, program_path="/ignored.py", poll_ms=50)

    def test_tick_calls_pump_tx(self):
        from openbricks import ble_repl
        calls = []
        orig = ble_repl.pump_tx
        ble_repl.pump_tx = lambda: calls.append(1)
        try:
            self.launcher._tick()
            self.launcher._tick()
        finally:
            ble_repl.pump_tx = orig
        self.assertEqual(len(calls), 2)

    def test_pump_failure_does_not_kill_the_tick(self):
        # The tick also owns the stop button — a raising backstop must
        # never unwind it.
        from openbricks import ble_repl

        def _boom():
            raise RuntimeError("pump exploded")

        orig = ble_repl.pump_tx
        ble_repl.pump_tx = _boom
        try:
            self.launcher._tick()   # must not raise
        finally:
            ble_repl.pump_tx = orig


class ButtonPressLogEntryTests(unittest.TestCase):
    """Every button press that interacts with a run leaves a log
    entry: a stop press notes into the active run's log, and a
    button-started run's log opens with a stamped "started:" line."""

    def setUp(self):
        self.btn = _make_button()
        self.launcher = launcher.Launcher(
            self.btn, program_path="/ignored.py", poll_ms=50)

    def test_stop_press_notes_into_log(self):
        from openbricks import log as log_mod
        notes = []
        orig_note = log_mod.note
        orig_stop = launcher._request_stop
        log_mod.note = lambda text: notes.append(text)
        launcher._request_stop = lambda inst: None
        try:
            self.launcher._running = True
            self.btn._value = 0           # press-down
            _tick_debounced(self.launcher)
        finally:
            log_mod.note = orig_note
            launcher._request_stop = orig_stop
        self.assertEqual(
            notes,
            ["button pressed -> stop",
             "estop engaged: motors killed, motion latched"])

    def test_note_failure_does_not_kill_the_tick(self):
        from openbricks import log as log_mod

        def _boom(text):
            raise RuntimeError("flash died")

        orig_note = log_mod.note
        orig_stop = launcher._request_stop
        log_mod.note = _boom
        launcher._request_stop = lambda inst: None
        try:
            self.launcher._running = True
            self.btn._value = 0
            _tick_debounced(self.launcher)        # must not raise
        finally:
            log_mod.note = orig_note
            launcher._request_stop = orig_stop

    def test_button_started_run_logs_origin_line(self):
        import tests.test_log as tlog
        from openbricks import log as log_mod
        tlog._wipe(tlog._TEST_LOG_DIR)
        prev_dir = log_mod.LOG_DIR
        log_mod.LOG_DIR = tlog._TEST_LOG_DIR
        prog = tlog._TEST_LOG_DIR + "_prog.py"
        try:
            with open(prog, "w") as f:
                f.write("print('body')\n")
            launcher._exec_program(prog, origin="button press")
            runs = log_mod.list_runs()
            self.assertEqual(len(runs), 1)
            data = log_mod.read_run(runs[0][0])
        finally:
            log_mod.LOG_DIR = prev_dir
            tlog._wipe(tlog._TEST_LOG_DIR)
            try:
                os.remove(prog)
            except OSError:
                pass
        self.assertIn("started: button press | firmware ", data)
        from openbricks import __version__ as fw_ver
        self.assertIn("firmware %s" % fw_ver, data)
        self.assertIn("program " + prog, data)
        self.assertIn("uptime ", data)
        self.assertIn("body", data)


class _FakePressCounter:
    """Stand-in for esp32.PCNT: tests bump .count to simulate hardware
    edges landing between (or during) ticks."""

    def __init__(self):
        self.count = 0
        self.raise_on_value = None

    def value(self):
        if self.raise_on_value is not None:
            raise self.raise_on_value
        return self.count


class HardwarePressLatchTests(unittest.TestCase):
    """The PCNT edge counter makes a press late-but-never-lost: a
    press falling entirely inside a scheduler blackout (measured
    ~200 ms; a quick press is ~120-160 ms) is invisible to level
    polling but still counted in silicon, and the next tick that DOES
    run must fire the stop."""

    def setUp(self):
        self.btn = _make_button()
        self.launcher = launcher.Launcher(
            self.btn, program_path="/ignored.py", poll_ms=50)
        self.pcnt = _FakePressCounter()
        self.launcher._press_pcnt = self.pcnt
        self.launcher._sync_press_counter()
        self.stops = []
        self.notes = []
        from openbricks import log as log_mod
        self._orig_stop = launcher._request_stop
        self._orig_note = log_mod.note
        self._log_mod = log_mod
        launcher._request_stop = lambda inst: self.stops.append(inst)
        log_mod.note = lambda text: self.notes.append(text)

    def tearDown(self):
        launcher._request_stop = self._orig_stop
        self._log_mod.note = self._orig_note

    def test_missed_press_fires_on_next_tick(self):
        # Press + release happened entirely between ticks: the level
        # is back HIGH, only the hardware count knows.
        self.launcher._running = True
        self.pcnt.count += 2   # press edge + one bounce edge
        self.launcher._tick()
        self.assertEqual(len(self.stops), 1)
        self.assertTrue(
            any("hardware counter" in n for n in self.notes), self.notes)
        # Counter consumed: the same edges never fire twice.
        del self.stops[:]
        self.launcher._tick()
        self.assertEqual(self.stops, [])

    def test_level_path_does_not_double_fire_after_latch(self):
        # Latch fired while the finger is still down; the level path
        # sees press-down next tick and must not fire a second stop.
        self.launcher._running = True
        self.pcnt.count += 1
        self.launcher._tick()          # latch fires, stop in flight
        self.assertEqual(len(self.stops), 1)
        self.btn._value = 0            # still held
        _tick_debounced(self.launcher)
        self.assertEqual(len(self.stops), 1)
        # Release of that press must not START anything (consumed).
        starts = []
        orig_start = launcher._request_start
        launcher._request_start = lambda inst: starts.append(inst)
        try:
            self.launcher._running = False
            self.btn._value = 1
            _tick_debounced(self.launcher)
        finally:
            launcher._request_start = orig_start
        self.assertEqual(starts, [])

    def test_idle_presses_do_not_stop_the_next_run(self):
        self.pcnt.count += 3           # fumbling at idle
        self.launcher._tick()          # idle branch consumes
        self.launcher._running = True
        self.launcher._tick()
        self.assertEqual(self.stops, [])

    def test_sync_before_run_consumes_stale_count(self):
        self.pcnt.count += 5
        self.launcher._sync_press_counter()
        self.launcher._running = True
        self.launcher._tick()
        self.assertEqual(self.stops, [])

    def test_sync_survives_counter_failure(self):
        self.pcnt.raise_on_value = RuntimeError("pcnt wedged")
        self.launcher._sync_press_counter()   # must not raise
        self.assertEqual(self.launcher._press_count_seen, 0)

    def test_remote_start_mid_hold_release_fires_stop(self):
        # Press-down while idle, program starts remotely during the
        # hold, release -> the release is a stop request.
        self.btn._value = 0
        _tick_debounced(self.launcher)                 # press-down at idle
        self.launcher._running = True         # remote start mid-hold
        self.btn._value = 1
        _tick_debounced(self.launcher)                 # release
        self.assertEqual(len(self.stops), 1)
        self.assertTrue(
            any(n == "button pressed -> stop" for n in self.notes),
            self.notes)

    def test_counter_failure_does_not_kill_tick_or_level_path(self):
        self.launcher._running = True
        self.pcnt.raise_on_value = RuntimeError("pcnt wedged")
        self.btn._value = 0            # real press, level path
        _tick_debounced(self.launcher)          # must not raise
        self.assertEqual(len(self.stops), 1)

    def test_no_counter_level_path_unchanged(self):
        self.launcher._press_pcnt = None
        self.launcher._running = True
        self.btn._value = 0
        _tick_debounced(self.launcher)
        self.assertEqual(len(self.stops), 1)


class PressCounterInstallTests(unittest.TestCase):
    """_install_press_counter drives esp32.PCNT correctly, and its
    unavailable-fallback is announced, not silent."""

    class _FakePCNTClass:
        INCREMENT = "inc"
        IGNORE = "ign"
        instances = []

        def __init__(self, unit, **kwargs):
            self.unit = unit
            self.kwargs = kwargs
            self.started = False
            type(self).instances.append(self)

        def start(self):
            self.started = True

    def test_installs_falling_edge_counter_on_unit_3(self):
        import sys
        fake_cls = self._FakePCNTClass
        del fake_cls.instances[:]

        class _FakeEsp32Module:
            PCNT = fake_cls
        had = "esp32" in sys.modules
        prev = sys.modules.get("esp32")
        sys.modules["esp32"] = _FakeEsp32Module
        try:
            pcnt = launcher._install_press_counter(4)
        finally:
            if had:
                sys.modules["esp32"] = prev
            else:
                del sys.modules["esp32"]
        self.assertIsNotNone(pcnt)
        self.assertEqual(pcnt.unit, launcher.STOP_PCNT_UNIT)
        self.assertEqual(pcnt.kwargs.get("falling"), fake_cls.INCREMENT)
        self.assertEqual(pcnt.kwargs.get("rising"), fake_cls.IGNORE)
        self.assertTrue(pcnt.started)

    def test_unavailable_is_announced_and_returns_none(self):
        # No esp32 module on this runtime (or construction failed):
        # must return None and print the tick-bound announcement.
        import builtins
        import sys
        printed = []
        orig_print = builtins.print
        builtins.print = lambda *a, **k: printed.append(
            " ".join(str(x) for x in a))
        had = "esp32" in sys.modules
        prev = sys.modules.get("esp32")
        if had:
            del sys.modules["esp32"]
        try:
            pcnt = launcher._install_press_counter(4)
        finally:
            builtins.print = orig_print
            if had:
                sys.modules["esp32"] = prev
        self.assertIsNone(pcnt)
        self.assertTrue(
            any("press latch unavailable" in s for s in printed), printed)


class ChatterRegressionTests(unittest.TestCase):
    """Contact-chatter defences (1.15.3), pinned against the bench
    event-ring capture: the start press's release chatter killed the
    newborn run three ways in one session — a phantom
    press-down(running) 54 ms after start, and PCNT latch stops 100
    and 180 ms after start."""

    def setUp(self):
        from openbricks import estop
        estop.clear()
        self.addCleanup(estop.clear)
        self.btn = _make_button()
        self.launcher = launcher.Launcher(
            self.btn, program_path="/ignored.py", poll_ms=50)
        self.starts = []
        self.stops = []
        self._orig_start = launcher._request_start
        self._orig_stop = launcher._request_stop
        launcher._request_start = lambda inst: self.starts.append(inst)
        launcher._request_stop = lambda inst: self.stops.append(inst)
        self.addCleanup(
            setattr, launcher, "_request_start", self._orig_start)
        self.addCleanup(
            setattr, launcher, "_request_stop", self._orig_stop)

    def test_one_tick_release_flicker_does_not_dispatch_start(self):
        # Press-down chatter: contact opens for a single poll mid-
        # hold. The old code read it as a release and STARTED the
        # program while the finger was still down.
        self.btn._value = 0
        _tick_debounced(self.launcher)      # press established
        self.btn._value = 1                 # 1-tick flicker
        self.launcher._tick()
        advance_ms(50)
        self.btn._value = 0                 # contact restored
        _tick_debounced(self.launcher)
        self.assertEqual(self.starts, [],
                         "a one-poll flicker dispatched a start")

    def test_one_tick_press_flicker_while_running_does_not_stop(self):
        # Release chatter, level flavour (the 54 ms press-down
        # (running) from the capture): a single-poll re-close while
        # the program runs must not fire a stop.
        self.launcher._running = True
        self.btn._value = 0                 # 1-tick chatter re-close
        self.launcher._tick()
        advance_ms(50)
        self.btn._value = 1
        _tick_debounced(self.launcher)
        self.assertEqual(self.stops, [],
                         "one-poll chatter while running fired a stop")

    def _with_counter(self):
        pcnt = _FakePressCounter()
        self.launcher._press_pcnt = pcnt
        self.launcher._sync_press_counter()
        return pcnt

    def test_latch_edges_in_start_grace_are_consumed(self):
        # Release chatter, PCNT flavour (the 100/180 ms latch-stops
        # from the capture): falling edges landing just after the
        # run starts are the start press's own chatter.
        pcnt = self._with_counter()
        self.launcher._run_started_ms = launcher._now_ms()
        self.launcher._running = True
        pcnt.count += 2                     # chatter edges
        self.launcher._tick()
        self.assertEqual(self.stops, [],
                         "latch fired on start-press chatter")
        # Consumed: the same edges never fire later either.
        advance_ms(launcher.Launcher.RUN_START_GRACE_MS + 100)
        self.launcher._tick()
        self.assertEqual(self.stops, [])

    def test_latch_edge_after_grace_still_stops(self):
        pcnt = self._with_counter()
        self.launcher._run_started_ms = launcher._now_ms()
        self.launcher._running = True
        advance_ms(launcher.Launcher.RUN_START_GRACE_MS + 100)
        pcnt.count += 1                     # a real press edge
        self.launcher._tick()
        self.assertEqual(len(self.stops), 1,
                         "post-grace latch press must stop")

    def test_real_stop_press_during_grace_stops_via_level_path(self):
        # The grace window must not create a stop-dead zone: a held
        # (debounce-confirmed) press right after start still stops
        # through the level path.
        self._with_counter()
        self.launcher._run_started_ms = launcher._now_ms()
        self.launcher._running = True
        self.btn._value = 0
        _tick_debounced(self.launcher)
        self.assertEqual(len(self.stops), 1,
                         "grace window swallowed a real stop press")

    def test_flicker_leaves_bounce_filtered_event(self):
        del launcher._EVENTS[:]
        launcher._EVENTS_NEXT[0] = 0
        self.btn._value = 0
        _tick_debounced(self.launcher)
        self.btn._value = 1                 # 1-tick flicker
        self.launcher._tick()
        advance_ms(50)
        self.btn._value = 0
        _tick_debounced(self.launcher)
        tags = [e[1] for e in launcher._EVENTS]
        self.assertIn("bounce-filtered", tags)


class TickStarvationTests(unittest.TestCase):
    """_tick measures its own inter-run gap: a machine.Timer callback
    is silently DROPPED when the scheduler queue is full, and during
    such a gap a button press is invisible. A stretched gap must leave
    a stamped note in the run log, and the worst gap rides the stop
    debrief."""

    def setUp(self):
        self.btn = _make_button()
        self.launcher = launcher.Launcher(
            self.btn, program_path="/ignored.py", poll_ms=50)

    def _capture_notes(self):
        from openbricks import log as log_mod
        notes = []
        orig = log_mod.note
        log_mod.note = lambda text: notes.append(text)
        return notes, log_mod, orig

    def test_healthy_cadence_notes_nothing(self):
        notes, log_mod, orig = self._capture_notes()
        try:
            for _ in range(5):
                self.launcher._tick()
                advance_ms(50)
        finally:
            log_mod.note = orig
        self.assertEqual(notes, [])

    def test_starved_gap_leaves_note_and_updates_max(self):
        notes, log_mod, orig = self._capture_notes()
        try:
            self.launcher._tick()
            advance_ms(50)
            self.launcher._tick()
            advance_ms(400)          # 8 lost ticks
            self.launcher._tick()
        finally:
            log_mod.note = orig
        self.assertEqual(len(notes), 1)
        self.assertTrue(notes[0].startswith("tick starved 400 ms"),
                        notes[0])
        self.assertEqual(self.launcher._tick_gap_max, 400)

    def test_debrief_reports_worst_tick_gap(self):
        orig_singleton = launcher._singleton
        launcher._singleton = self.launcher
        try:
            self.launcher._last_stop_ms = launcher._now_ms()
            self.launcher._stop_retry_count = 1
            self.launcher._tick_gap_max = 275
            s = launcher._stop_debrief()
        finally:
            launcher._singleton = orig_singleton
        self.assertIn("worst tick gap 275 ms", s)


class StopBreadcrumbTests(unittest.TestCase):
    """The stop path writes enough into the run log to tell WHICH link
    failed on a dead press: no 'button pressed' line = press never
    detected; no 'estop engaged' line = engage failed; no 'stopped:'
    line = program never died (injection eaten forever); the debrief
    carries press-to-death latency and retry count."""

    def setUp(self):
        self.btn = _make_button()
        self.launcher = launcher.Launcher(
            self.btn, program_path="/ignored.py", poll_ms=50)
        self._orig_singleton = launcher._singleton

    def tearDown(self):
        launcher._singleton = self._orig_singleton

    def _capture_notes(self):
        from openbricks import log as log_mod
        notes = []
        orig = log_mod.note
        log_mod.note = lambda text: notes.append(text)
        return notes, log_mod, orig

    def test_fire_stop_notes_estop_engaged(self):
        notes, log_mod, orig_note = self._capture_notes()
        orig_stop = launcher._request_stop
        launcher._request_stop = lambda inst: None
        try:
            self.launcher._fire_stop()
        finally:
            log_mod.note = orig_note
            launcher._request_stop = orig_stop
        self.assertEqual(
            notes, ["estop engaged: motors killed, motion latched"])
        self.assertEqual(self.launcher._stop_retry_count, 0)

    def test_fire_stop_notes_engage_failure(self):
        from openbricks import estop
        notes, log_mod, orig_note = self._capture_notes()
        orig_stop = launcher._request_stop
        orig_engage = estop.engage

        def _boom():
            raise RuntimeError("bus wedged")

        estop.engage = _boom
        launcher._request_stop = lambda inst: None
        try:
            self.launcher._fire_stop()   # must not raise
        finally:
            log_mod.note = orig_note
            launcher._request_stop = orig_stop
            estop.engage = orig_engage
        self.assertEqual(len(notes), 1)
        self.assertTrue(notes[0].startswith("estop engage FAILED:"),
                        notes[0])

    def test_retry_increments_count(self):
        orig_stop = launcher._request_stop
        launcher._request_stop = lambda inst: None
        try:
            self.launcher._running = True
            self.launcher._fire_stop()
            # Age the last request past STOP_RETRY_MS, tick, repeat.
            for expected in (1, 2):
                self.launcher._stop_retry_ms -= (
                    self.launcher.STOP_RETRY_MS + 1)
                self.launcher._tick()
                self.assertEqual(
                    self.launcher._stop_retry_count, expected)
        finally:
            launcher._request_stop = orig_stop

    def test_debrief_without_press(self):
        launcher._singleton = None
        self.assertIn("no stop press recorded", launcher._stop_debrief())

    def test_debrief_with_press_and_retries(self):
        launcher._singleton = self.launcher
        self.launcher._last_stop_ms = launcher._now_ms()
        self.launcher._stop_retry_count = 3
        s = launcher._stop_debrief()
        self.assertIn("ms after press", s)
        self.assertIn("3 retries", s)

    def test_interrupted_run_logs_stopped_debrief(self):
        import tests.test_log as tlog
        from openbricks import log as log_mod
        tlog._wipe(tlog._TEST_LOG_DIR)
        prev_dir = log_mod.LOG_DIR
        log_mod.LOG_DIR = tlog._TEST_LOG_DIR
        launcher._singleton = None   # debrief: host Ctrl-C variant
        prog = tlog._TEST_LOG_DIR + "_stop_prog.py"
        try:
            with open(prog, "w") as f:
                f.write("raise KeyboardInterrupt\n")
            launcher._exec_program(prog, origin="button press")
            runs = log_mod.list_runs()
            self.assertEqual(len(runs), 1)
            data = log_mod.read_run(runs[0][0])
        finally:
            log_mod.LOG_DIR = prev_dir
            tlog._wipe(tlog._TEST_LOG_DIR)
            try:
                os.remove(prog)
            except OSError:
                pass
        self.assertIn("started: button press", data)
        self.assertIn("stopped: KeyboardInterrupt", data)
        self.assertIn("no stop press recorded", data)


class StopRequestTests(unittest.TestCase):
    """A press while running requests a STOP (``_request_stop``), never a
    second START. ``_tick`` only flags it; the native C-function
    ``stop_tick`` Timer callback injects the interrupt."""

    def setUp(self):
        self.btn = _make_button()
        self.launcher = launcher.Launcher(
            self.btn, program_path="/ignored.py", poll_ms=50)
        # Pretend a program is already executing.
        self.launcher._running = True

    def test_short_press_while_running_requests_stop_not_start(self):
        starts = []
        stops = []
        orig_start = launcher._request_start
        orig_stop = launcher._request_stop
        launcher._request_start = lambda inst: starts.append(inst)
        launcher._request_stop = lambda inst: stops.append(inst)
        try:
            _press(self.btn, hold_ms=200, tick_fn=self.launcher._tick)
        finally:
            launcher._request_start = orig_start
            launcher._request_stop = orig_stop
        self.assertEqual(starts, [])               # no (re)start
        self.assertEqual(stops, [self.launcher])   # stop requested once

    def test_request_stop_calls_native_when_present(self):
        # Exercise the native path under CPython too (the openbricks-py
        # coverage flag is measured there, where _openbricks_native is
        # otherwise absent and only the fallback branch would run).
        import sys
        rec = []

        class _FakeNative:
            def request_stop(self_):
                rec.append(1)

        saved = sys.modules.get("_openbricks_native")
        sys.modules["_openbricks_native"] = _FakeNative()
        try:
            launcher._request_stop(self.launcher)
        finally:
            if saved is not None:
                sys.modules["_openbricks_native"] = saved
            else:
                sys.modules.pop("_openbricks_native", None)
        self.assertEqual(rec, [1])
        self.assertIsNone(self.launcher._pending)   # native path, no fallback

    def test_install_stop_tick_creates_timer_with_native_callback(self):
        # Cover the Timer-creation lines under CPython by making the native
        # stop_tick import succeed (it's otherwise absent on desktop).
        import sys

        class _FakeNative:
            def stop_tick(self_, timer_arg):
                pass

        saved = sys.modules.get("_openbricks_native")
        sys.modules["_openbricks_native"] = _FakeNative()
        try:
            t = launcher._install_stop_tick(1)
        finally:
            if saved is not None:
                sys.modules["_openbricks_native"] = saved
            else:
                sys.modules.pop("_openbricks_native", None)
        self.assertIsNotNone(t)   # a periodic Timer was created

    def test_request_stop_falls_back_to_pending_without_native(self):
        # Cover the fallback branch: when the native hook is missing,
        # _request_stop sets the _pending flag instead.
        import sys

        class _Empty:        # lacks request_stop -> import fails -> fallback
            pass

        inst = launcher.Launcher(_make_button())
        saved = sys.modules.get("_openbricks_native")
        sys.modules["_openbricks_native"] = _Empty()
        try:
            launcher._request_stop(inst)
        finally:
            if saved is not None:
                sys.modules["_openbricks_native"] = saved
            else:
                sys.modules.pop("_openbricks_native", None)
        self.assertEqual(inst._pending, "stop")


class ScheduledStartTests(unittest.TestCase):
    """``_scheduled_start`` is the callback MicroPython runs between
    bytecodes after a short-press-while-idle. It execs the staged
    program and leaves the launcher back in the idle state, so
    pressing the button AGAIN starts it again — even when main.py's
    idle loop isn't running (which is the post-``openbricks run``
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
                def _mark_idle_alive(self):
                    pass
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
    """``launcher.run_program`` is the entry point ``openbricks run``
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

    def test_stop_tick_injects_interrupt_when_armed_and_requested(self):
        # The core mechanism: the native C-function stop_tick callback,
        # when armed + a stop was requested, injects a real
        # KeyboardInterrupt that lands in the running code. (Off-hardware
        # there's no native module — skip.)
        try:
            from _openbricks_native import (
                stop_tick, request_stop, set_stop_armed)
        except (ImportError, AttributeError):
            return
        set_stop_armed(True)
        request_stop()
        fired = False
        try:
            stop_tick(None)              # C-function callback fires the kbd
            for _ in range(100000):      # let the pending exception land
                pass
        except KeyboardInterrupt:
            fired = True
        finally:
            set_stop_armed(False)
        self.assertTrue(fired, "stop_tick must inject a KeyboardInterrupt")

    def test_stop_tick_is_noop_when_not_armed(self):
        try:
            from _openbricks_native import (
                stop_tick, request_stop, set_stop_armed)
        except (ImportError, AttributeError):
            return
        set_stop_armed(False)
        request_stop()                   # request, but disarmed
        fired = False
        try:
            stop_tick(None)
            for _ in range(50000):
                pass
        except KeyboardInterrupt:
            fired = True
        self.assertFalse(fired, "disarmed stop_tick must not interrupt")

    def test_stop_button_debug_reports_state(self):
        try:
            from _openbricks_native import (
                set_stop_armed, request_stop, stop_button_debug)
        except (ImportError, AttributeError):
            return
        set_stop_armed(True)
        request_stop()
        d = stop_button_debug()          # (tick_count, fire_count, armed, requested)
        self.assertEqual(len(d), 4)
        self.assertEqual(d[2], 1)        # armed
        self.assertEqual(d[3], 1)        # requested
        set_stop_armed(False)            # also clears requested

    def test_exec_program_raw_arms_then_disarms_stop_button(self):
        # The native stop button is armed for the duration of the run and
        # disarmed afterward, so a press while idle can't tear down the
        # idle loop. Inject a fake native module to record the calls.
        import sys
        events = []

        class _FakeNative:
            def set_stop_armed(self_, armed):
                events.append(("armed", bool(armed)))

        saved = sys.modules.get("_openbricks_native")
        sys.modules["_openbricks_native"] = _FakeNative()
        path = _write_program("pass\n")
        try:
            launcher._exec_program_raw(path)
        finally:
            if saved is not None:
                sys.modules["_openbricks_native"] = saved
            else:
                sys.modules.pop("_openbricks_native", None)
        # Armed True before the run, False after — in that order.
        self.assertEqual(events, [("armed", True), ("armed", False)])

    def test_exec_program_raw_disarms_stop_button_on_keyboard_interrupt(self):
        # Even when the program is stopped mid-run, the finally must disarm.
        import sys
        events = []

        class _FakeNative:
            def set_stop_armed(self_, armed):
                events.append(bool(armed))

        saved = sys.modules.get("_openbricks_native")
        sys.modules["_openbricks_native"] = _FakeNative()
        path = _write_program("raise KeyboardInterrupt\n")
        try:
            with self.assertRaises(KeyboardInterrupt):
                launcher._exec_program_raw(path)
        finally:
            if saved is not None:
                sys.modules["_openbricks_native"] = saved
            else:
                sys.modules.pop("_openbricks_native", None)
        self.assertEqual(events[-1], False)   # disarmed despite the interrupt

    def test_native_request_keyboard_interrupt_actually_interrupts(self):
        # Pin the native primitive itself: on a build with the C module it
        # injects a real pending KeyboardInterrupt that stops a loop. (The
        # launcher's stop path is the C GPIO ISR, untestable off-hardware,
        # but it calls the same mp_sched_keyboard_interrupt under the hood.)
        try:
            from _openbricks_native import request_keyboard_interrupt
        except (ImportError, AttributeError):
            return  # desktop: nothing to assert
        fired = False
        try:
            request_keyboard_interrupt()
            for _ in range(100000):
                pass
        except KeyboardInterrupt:
            fired = True
        self.assertTrue(fired)

    def test_stop_all_motors_broadcasts_torque_off_to_serial_bus(self):
        from openbricks.drivers.st3215 import ST3215Motor
        m = ST3215Motor(servo_id=1, uart_id=1, tx=14, rx=6)
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


class StartPathSelectionTests(unittest.TestCase):
    """Regression tests for the "start button doesn't stop the program"
    bug: a program exec'd inside ``micropython.schedule`` runs with the
    scheduler LOCKED, which starves the Timer-dispatched stop path for
    the whole run. While the main-thread idle loop is draining, a start
    press must therefore queue ``_pending = "start"`` (main-thread
    exec, stop button works) and must NOT go through schedule-exec."""

    def setUp(self):
        self.btn = _make_button()
        self.launch = launcher.Launcher(self.btn)
        self.schedule_calls = []
        self._orig = launcher._start_via_schedule
        launcher._start_via_schedule = (
            lambda inst: self.schedule_calls.append(inst))
        self.addCleanup(
            setattr, launcher, "_start_via_schedule", self._orig)

    def test_alive_idle_loop_start_uses_pending_flag(self):
        self.launch._mark_idle_alive()
        launcher._request_start(self.launch)
        self.assertEqual(self.launch._pending, "start")
        self.assertEqual(self.schedule_calls, [])

    def test_no_idle_loop_falls_back_to_schedule(self):
        # Never marked — hub parked at the REPL. Degraded schedule-exec
        # (with its printed warning) is the only way to start.
        launcher._request_start(self.launch)
        self.assertIsNone(self.launch._pending)
        self.assertEqual(self.schedule_calls, [self.launch])

    def test_stale_idle_loop_falls_back_to_schedule(self):
        self.launch._mark_idle_alive()
        advance_ms(self.launch._poll_ms * 10)   # loop stopped draining
        launcher._request_start(self.launch)
        self.assertEqual(self.schedule_calls, [self.launch])

    def test_idle_loop_alive_tracks_recent_marks(self):
        self.assertFalse(self.launch._idle_loop_alive())
        self.launch._mark_idle_alive()
        self.assertTrue(self.launch._idle_loop_alive())
        advance_ms(self.launch._poll_ms * 2)
        self.assertTrue(self.launch._idle_loop_alive())
        advance_ms(self.launch._poll_ms * 10)
        self.assertFalse(self.launch._idle_loop_alive())

    def test_drain_remarks_liveness_after_long_program(self):
        # A start press landing right after a long program ends must
        # not misread the idle loop as gone: _drain_pending re-marks
        # liveness when the exec finishes.
        path = _write_program("pass\n")
        self.addCleanup(_cleanup_program)
        self.launch._program_path = path
        self.launch._mark_idle_alive()
        advance_ms(self.launch._poll_ms * 10)   # "program ran for ages"
        self.launch._pending = "start"
        self.launch._drain_pending()
        self.assertTrue(self.launch._idle_loop_alive())


def tearDownModule():
    # CPython 3.11/3.12 quirk: a KeyboardInterrupt that escapes an
    # ``exec()`` marks the interpreter as interrupt-killed (process
    # exit code 130) even when the caller catches it — fixed in later
    # CPython, absent on MicroPython. Several tests here exec
    # programs that raise KeyboardInterrupt (the stop-button paths);
    # whether the suite's exit code survived used to depend on test
    # ORDER (a later clean exec resets the flag). One deliberate
    # clean exec at module teardown makes it deterministic.
    exec("pass")


if __name__ == "__main__":
    unittest.main()
