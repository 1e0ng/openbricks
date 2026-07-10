# SPDX-License-Identifier: MIT
"""
Button-gated user-program launcher.

Pybricks-style workflow:

* ``openbricks upload`` stages a script at ``/program.py`` but
  does not run it — the user presses the program button to launch.
* ``openbricks run`` stages the same script and triggers the
  launcher immediately. Output streams back to the client; pressing
  the program button stops the program; when the program stops, the
  terminal exits.

Each press is a full press-release cycle. The program button has its
own GPIO (default ``4``), separate from the BLE-toggle button watched
by :mod:`openbricks.bluetooth_button` (default ``5``). Two pins → no
duration-based dispatch — every press on the program pin means
start-or-stop, and every press on the BLE pin means toggle-BLE.

Wiring:

* Press while idle → start ``/program.py``.
* Press while running → raise ``KeyboardInterrupt`` via
  ``micropython.schedule``, cleanly stopping the program.

The watcher runs off a ``machine.Timer`` kept alive for the whole hub
uptime (we never ``deinit`` it), so button-press-to-run survives
``openbricks run`` interrupting the main idle loop.

Typical ``main.py`` (the firmware ships a frozen default; users can
override by writing to ``/main.py`` in VFS):

    from openbricks import bluetooth, launcher
    bluetooth.apply_persisted_state()
    launcher.run()          # installs watcher + blocks on the idle loop
"""

import sys
import time


DEFAULT_BUTTON_PIN   = 4
DEFAULT_POLL_MS      = 50
DEFAULT_PROGRAM_PATH = "/program.py"

# Hardware-timer inventory (ESP32 / ESP32-S3 both have exactly 0..3):
#   0 — launcher button poll (this module)
#   1 — BLE-toggle button poll (openbricks.bluetooth_button)
#   2 — motor_process 1 kHz scheduler (native C module)
#   3 — stop-tick interrupt injector (this module)
# The stop tick used to take ``timer_id + 1`` = 1, silently stealing
# the BLE toggle's timer the moment the launcher started: the BLE
# button went dead and the status LED stopped following BLE state.
STOP_TIMER_ID = 3


def _now_ms():
    """Monotonic milliseconds — ``time.ticks_ms`` on MicroPython,
    wall-clock fallback for CPython tests."""
    ticks = getattr(time, "ticks_ms", None)
    if ticks is not None:
        return ticks()
    return int(time.time() * 1000)


def _ticks_diff(a, b):
    diff = getattr(time, "ticks_diff", None)
    if diff is not None:
        return diff(a, b)
    return a - b


class Launcher:
    """Shared state for the program-button watcher.

    Tests instantiate this directly and drive ``_tick`` with a fake
    Pin; production code uses ``_ensure_launcher()`` below, which
    installs a singleton + ``machine.Timer``.
    """

    def __init__(self, button, program_path=DEFAULT_PROGRAM_PATH,
                 poll_ms=DEFAULT_POLL_MS):
        self._btn            = button      # anything with ``.value()``, 0 = pressed
        self._program_path   = program_path
        self._poll_ms        = poll_ms

        self._running        = False
        self._was_pressed    = False
        self._pending        = None        # None | "start" | "stop"
        # Timers stay alive for hub uptime — we never ``.deinit()`` them.
        # Keeping the references here stops GC from collecting them.
        self._timer          = None    # Timer(0): START + stop-press detect
        self._stop_timer     = None    # Timer(STOP_TIMER_ID=3): stop_tick
        # Last time the main-thread idle loop drained. ``_request_start``
        # uses this to decide whether a queued "start" will actually be
        # picked up (idle loop alive → main-thread exec, button-stop
        # works) or whether it must fall back to the degraded
        # schedule-exec path (see ``_scheduled_start``).
        self._idle_drain_ms  = None

    def _mark_idle_alive(self):
        self._idle_drain_ms = _now_ms()

    def _idle_loop_alive(self):
        """True while the main-thread idle loop is actively draining
        (last pass within a few poll periods)."""
        if self._idle_drain_ms is None:
            return False
        return _ticks_diff(_now_ms(), self._idle_drain_ms) < self._poll_ms * 4

    # ---- timer callback ----

    def _tick(self, _timer=None):
        """Called on every ``poll_ms`` tick. Edge-detects press→release
        cycles. When idle, dispatches a program START; when running,
        flags a STOP via ``_request_stop``.

        This (a soft Python callback) only *detects* the press and sets a
        flag — it can't inject the interrupt, because a pending exception
        set in a Python callback frame unwinds the callback, not the
        program. The native C-function ``stop_tick`` Timer callback (set
        up in ``_ensure_launcher``) does the injection."""
        pressed = self._btn.value() == 0
        if pressed:
            self._was_pressed = True
            return
        if not self._was_pressed:
            return
        # Released after a press — fire once.
        self._was_pressed = False
        if not self._running:
            _request_start(self)
        else:
            # Running: flag a stop. The native C-function stop_tick Timer
            # callback injects the KeyboardInterrupt (this Python callback
            # can't — it would unwind itself).
            _request_stop(self)

    def _drain_pending(self):
        """Consume a queued ``_pending`` start — the PRIMARY start path.

        Runs in the main thread (called from ``run()``'s idle loop), so
        the program executes with the scheduler unlocked: Timer
        callbacks keep firing between its bytecodes and the stop
        button works. Programs must never be exec'd from a scheduled
        callback — see ``_request_start``.
        """
        if self._pending == "start" and not self._running:
            self._pending = None
            self._running = True
            try:
                _exec_program(self._program_path)
            finally:
                self._running = False
                # Re-mark liveness immediately so a start-press landing
                # in the instant after a long program ends doesn't
                # misread the idle loop as gone.
                self._mark_idle_alive()
            print("openbricks: idle. Press button to run", self._program_path)


# ---- emergency stop ----

def _stop_all_motors():
    """Best-effort: cut drive to every motor we can reach, regardless of
    the user program's structure.

    Raising ``KeyboardInterrupt`` to unwind the program does NOT stop the
    motors — a serial-bus servo keeps spinning at its last commanded
    velocity and the native 1 kHz scheduler keeps ticking. So the
    button-stop path calls this first, hitting both reachable groups:

    * **Native scheduler** — ``motor_process.stop()`` halts the 1 kHz
      tick that drives closed-loop (PWM/encoder) motors.
    * **Serial-bus servos** (ST-3215 / ST-3032) — broadcast a torque-off
      (coast) to *every* servo on *every* known bus via the shared bus
      registry and the broadcast ID. Torque-off halts a servo in any
      mode (wheel / step / position), so it doesn't matter what the
      program was doing.

    Every step is wrapped defensively: an emergency stop must try every
    avenue even if one bus is wedged — a failure to reach one motor must
    not prevent stopping the others. This is the one place a broad
    ``except`` is correct rather than papering over a bug.
    """
    try:
        from _openbricks_native import motor_process
        motor_process.stop()
    except Exception:
        pass
    try:
        from openbricks.drivers.st3215 import (
            ST3215, _REG_TORQUE, _BROADCAST_ID)
        for bus in list(ST3215._buses.values()):
            try:
                bus.write(_BROADCAST_ID, _REG_TORQUE, bytes([0]))
            except Exception:
                pass
    except Exception:
        pass


# ---- hardware stop button (native C-function Timer callback) ----

def _install_stop_tick(timer_id):
    """Start a fast ``machine.Timer`` whose callback is the native
    C-function ``stop_tick``.

    The stop interrupt must be injected from a context with no Python
    frame (a Python callback would unwind itself, not the program). The
    user C module can't use ESP-IDF headers, so the portable answer —
    the same one ``motor_process`` uses — is a C-function registered as a
    Timer callback: a C callback runs no Python bytecodes, so the pending
    ``KeyboardInterrupt`` it sets is raised in the running program, not in
    the callback. ``_tick`` (Python) does the press detection and calls
    ``request_stop``; this tick injects the interrupt. Returns the Timer
    (kept alive by the caller) or ``None`` off-hardware."""
    try:
        from machine import Timer
        from _openbricks_native import stop_tick
    except (ImportError, AttributeError):
        return None
    try:
        t = Timer(timer_id)
        t.init(period=20, mode=Timer.PERIODIC, callback=stop_tick)
        return t
    except (ValueError, OSError, TypeError):
        return None


def _request_stop(launcher_instance):
    """Flag a stop from the Python button watcher. The native C
    ``stop_tick`` Timer callback picks it up and injects the interrupt.
    No-op where the native module is absent."""
    try:
        from _openbricks_native import request_stop
        request_stop()
    except (ImportError, AttributeError):
        launcher_instance._pending = "stop"


def _arm_stop_button(armed):
    """Arm/disarm the native stop-button ISR. Armed only while a user
    program is executing, so a press while idle doesn't tear down the
    boot/idle loop. No-op where the native module is absent."""
    try:
        from _openbricks_native import set_stop_armed
        set_stop_armed(bool(armed))
    except (ImportError, AttributeError):
        pass


def _scheduled_start(launcher_instance):
    """DEGRADED fallback: run ``/program.py`` from the MicroPython
    scheduler queue.

    Only used when the main-thread idle loop is gone (hub parked at
    the REPL after ``openbricks run`` interrupted the frozen
    ``main.py``, or a dev Ctrl-C over USB). The Timer keeps firing,
    so routing start through ``micropython.schedule`` still lets a
    button press launch the program with nothing actively draining.

    The price — and why this is a fallback rather than the primary
    path: ``mp_sched_run_pending`` holds the scheduler LOCKED for the
    entire callback, i.e. for the entire user program. Timer
    callbacks are dispatched through that same scheduler queue, so
    while the program runs neither the button watcher ``_tick`` nor
    the native ``stop_tick`` injector can fire: **the stop button is
    dead for the whole run**. Announce it instead of degrading
    silently.
    """
    if launcher_instance._running:
        return  # already running; ignore (the raise path handles stop)
    print("openbricks: starting from REPL context — the stop button "
          "is unavailable for this run (use 'openbricks stop').")
    launcher_instance._running = True
    try:
        _exec_program(launcher_instance._program_path)
    finally:
        launcher_instance._running = False
    print("openbricks: idle. Press button to run",
          launcher_instance._program_path)


def _request_start(launcher_instance):
    """Queue a program start from the button-watcher Timer.

    Primary path: set ``_pending = "start"`` for the main-thread idle
    loop to drain — the program then executes in the main thread,
    where Timer callbacks keep firing between its bytecodes and the
    stop button works. (``_tick`` runs as a *scheduled* callback;
    exec'ing the program from here — or from anything scheduled —
    would hold the scheduler lock for the whole run and starve the
    stop path. That was the "start button doesn't stop the program"
    bug.)

    Degraded path: when the idle loop isn't draining (hub parked at
    the REPL), fall back to schedule-exec — see ``_scheduled_start``.

    Module-level so tests can swap it out.
    """
    if launcher_instance._idle_loop_alive():
        launcher_instance._pending = "start"
        return
    _start_via_schedule(launcher_instance)


def _start_via_schedule(launcher_instance):
    """Degraded-path dispatch, split out as a patchable seam for tests.
    Falls back to the ``_pending`` flag where ``micropython.schedule``
    isn't available (CPython)."""
    try:
        import micropython
        micropython.schedule(_scheduled_start, launcher_instance)
    except (ImportError, AttributeError, RuntimeError):
        launcher_instance._pending = "start"


# ---- program exec helpers ----

def _exec_program_raw(program_path):
    """Load and run ``program_path`` in a fresh namespace. Propagates
    ``KeyboardInterrupt``; prints other exceptions and returns.

    Wraps the run in a :func:`openbricks.log.session` so every
    ``print()`` / exception traceback is *also* tee'd to a flash
    file. The live console (USB-CDC / BLE-NUS) still sees everything;
    the file is only a backup for inspecting an untethered run later
    via ``openbricks log``.
    """
    with open(program_path) as f:
        code = f.read()
    from openbricks import log as _log
    # Arm the hardware stop button for the duration of the run. A press
    # now fires the native GPIO ISR, which injects a KeyboardInterrupt
    # into this exec; disarm in the finally so an idle press can't tear
    # down the boot/idle loop.
    _arm_stop_button(True)
    try:
        with _log.session() as sess:
            try:
                exec(code, {"__name__": "__main__"})
            except KeyboardInterrupt:
                # The button-stop's injected KeyboardInterrupt (and a REPL
                # Ctrl-C from ``openbricks run``) both unwind to here —
                # stop every motor before propagating so the robot halts
                # no matter how the program was running. Idempotent if
                # already stopped.
                _stop_all_motors()
                raise
            except Exception as e:
                pe = getattr(sys, "print_exception", None)
                if pe is not None:
                    pe(e)
                else:
                    import traceback
                    traceback.print_exception(type(e), e, e.__traceback__)
                # Tracebacks above go to the live console only — print()
                # is the only stream we tee. Mirror a short summary into
                # the log file so it shows up in ``openbricks log``
                # too.
                sess.write_text("Exception: %r\n" % (e,))
    finally:
        _arm_stop_button(False)


def _exec_program(program_path):
    """Button-gated path: swallow ``KeyboardInterrupt`` and missing-file
    errors so the idle loop keeps running between button presses."""
    try:
        _exec_program_raw(program_path)
        print("openbricks: program finished.")
    except OSError:
        print("openbricks: no program at", program_path)
    except KeyboardInterrupt:
        print("openbricks: stopped.")


# ---- singleton + timer wiring ----

_singleton = None


def _ensure_launcher(button_pin=DEFAULT_BUTTON_PIN,
                     poll_ms=DEFAULT_POLL_MS,
                     timer_id=0):
    """Install the Launcher singleton + persistent Timer. Idempotent.

    First call wins on pin/poll parameters; later calls just return
    the existing instance. This matters because ``run_program`` is
    entered after the frozen main.py has already called ``run()`` —
    we want the same watcher to keep firing, not a second one.
    """
    global _singleton
    if _singleton is not None:
        return _singleton
    from machine import Pin, Timer
    from openbricks import pins
    pins.check(button_pin, "program button", output=False)
    pins.claim(button_pin, "program button",
               "launcher.run(button_pin=...) moves it")
    btn = Pin(button_pin, Pin.IN, Pin.PULL_UP)
    _singleton = Launcher(btn, poll_ms=poll_ms)
    _singleton._timer = Timer(timer_id)
    _singleton._timer.init(
        period=poll_ms, mode=Timer.PERIODIC, callback=_singleton._tick)
    # The Timer(0) poll handles START and detects a stop press. STOP
    # delivery is a separate fast Timer whose callback is the native
    # C-function stop_tick (a soft Python callback can't inject the
    # interrupt into the running program; a C-function callback can).
    _singleton._stop_timer = _install_stop_tick(STOP_TIMER_ID)
    return _singleton


# ---- entry points ----

def run(program_path=DEFAULT_PROGRAM_PATH, button_pin=DEFAULT_BUTTON_PIN,
        poll_ms=DEFAULT_POLL_MS, timer_id=0):
    """Install the button watcher and block on the cooperative drain
    loop. Called from the frozen ``main.py``.

    ``timer_id=0`` is the first ESP32-S3 hardware timer. The previous
    default ``-1`` (virtual timer) was supported by older MicroPython
    builds but raises ``ValueError: invalid Timer number`` on the
    v1.27+ MP we vendor — esp32-s3 only exposes hardware timers 0..3.

    Intentionally blocks forever. If ``openbricks run`` later sends
    a Ctrl-C over the REPL to interrupt this loop, the Timer stays
    alive (we never ``deinit`` it) so subsequent ``run_program`` /
    button-press start continue to work.
    """
    launcher = _ensure_launcher(
        button_pin=button_pin, poll_ms=poll_ms, timer_id=timer_id)
    launcher._program_path = program_path
    print("openbricks: idle. Press button to run", program_path)
    while True:
        launcher._mark_idle_alive()
        launcher._drain_pending()
        time.sleep_ms(poll_ms)


def run_program(program_path=DEFAULT_PROGRAM_PATH):
    """Client-triggered entry for ``openbricks run``.

    Sets the ``_running`` flag, then exec's the program in the main
    thread (``_exec_program_raw`` arms the native stop button for the
    duration). Propagates ``KeyboardInterrupt`` so the raw-REPL
    disconnect signals "stopped" back to the client (which then
    exits).

    Wipes ``motor_process`` state before exec'ing the user script.
    ``openbricks run`` interrupts whatever was running before
    (typically a long-lived main.py) but the C-side motor_process
    callback list isn't tied to Python GC — without this reset, the
    new program inherits dead servo/drivebase tick-callback pointers
    from the previous program and ``register_c`` either silently
    rejects new subscriptions (list full) or schedules them alongside
    garbage, leaving ``DriveBase.straight()`` blocked forever in the
    ``while not is_done()`` loop.
    """
    _reset_motor_process()
    launcher = _ensure_launcher()
    launcher._running = True
    try:
        _exec_program_raw(program_path)
    finally:
        launcher._running = False


def _reset_motor_process():
    """Best-effort wipe of the native scheduler's callback list.
    Imported lazily so unix MP / CPython tests without the C module
    don't fail on launcher import."""
    try:
        from _openbricks_native import motor_process
    except ImportError:
        return
    motor_process.reset()
