# SPDX-License-Identifier: MIT
"""
Button-gated user-program launcher.

Pybricks-style workflow:

* ``openbricks-dev upload`` stages a script at ``/program.py`` but
  does not run it — the user presses the program button to launch.
* ``openbricks-dev run`` stages the same script and triggers the
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
``openbricks-dev run`` interrupting the main idle loop.

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
        self._pending        = None        # None | "start" | "stop"  (test fallback)
        # Timer stays alive for hub uptime — we never ``.deinit()`` it.
        # Keeping the reference here stops GC from collecting it.
        self._timer          = None

    # ---- timer callback ----

    def _tick(self, _timer=None):
        """Called on every ``poll_ms`` tick. Edge-detects press→release
        cycles and dispatches start/stop. Safe for IRQ context."""
        pressed = self._btn.value() == 0
        if pressed:
            self._was_pressed = True
            return
        if not self._was_pressed:
            return
        # Released after a press — fire once.
        self._was_pressed = False
        if self._running:
            _request_interrupt(self)
        else:
            _request_start(self)

    def _drain_pending(self):
        """Consume a queued ``_pending`` fallback.

        Only needed when ``_request_start`` / ``_request_interrupt``
        couldn't reach ``micropython.schedule`` (CPython tests). Under
        MicroPython, schedule runs the start/stop callbacks itself and
        this method is a no-op.
        """
        if self._pending == "start" and not self._running:
            self._pending = None
            self._running = True
            try:
                _exec_program(self._program_path)
            finally:
                self._running = False
            print("openbricks: idle. Press button to run", self._program_path)
        elif self._pending == "stop":
            self._pending = None


# ---- mid-run interrupt ----

def _raise_interrupt(_):
    raise KeyboardInterrupt


def _request_interrupt(launcher_instance):
    """Ask MicroPython to raise ``KeyboardInterrupt`` in the running
    program at the next bytecode boundary.

    Module-level so tests can swap it out — the ``micropython`` module
    is read-only on MP and can't be patched directly.
    """
    try:
        import micropython
        micropython.schedule(_raise_interrupt, None)
    except (ImportError, AttributeError, RuntimeError):
        launcher_instance._pending = "stop"


def _scheduled_start(launcher_instance):
    """Run ``/program.py`` from the MicroPython scheduler queue.

    Why schedule instead of setting a flag for the idle loop to drain:
    after ``openbricks-dev run`` interrupts the frozen ``main.py``,
    the idle loop is gone — we're sitting at the REPL. But the Timer
    keeps firing, so routing start through ``micropython.schedule``
    makes button-press-to-restart work even with nothing actively
    draining. This is what lets the user press the button again after
    ``run`` exits and have the robot start again.
    """
    if launcher_instance._running:
        return  # already running; ignore (the raise path handles stop)
    launcher_instance._running = True
    try:
        _exec_program(launcher_instance._program_path)
    finally:
        launcher_instance._running = False
    print("openbricks: idle. Press button to run",
          launcher_instance._program_path)


def _request_start(launcher_instance):
    """Schedule a program start from the button-watcher Timer.

    Module-level so tests can swap it out. Falls back to the
    ``_pending`` flag if ``micropython.schedule`` isn't available
    (CPython).
    """
    try:
        import micropython
        micropython.schedule(_scheduled_start, launcher_instance)
    except (ImportError, AttributeError, RuntimeError):
        launcher_instance._pending = "start"


# ---- program exec helpers ----

def _exec_program_raw(program_path):
    """Load and run ``program_path`` in a fresh namespace. Propagates
    ``KeyboardInterrupt``; prints other exceptions and returns.

    Used by ``run_program`` where the client needs to see
    ``KeyboardInterrupt`` propagate (so the disconnect signals
    "stopped" back to the user's terminal).
    """
    with open(program_path) as f:
        code = f.read()
    try:
        exec(code, {"__name__": "__main__"})
    except KeyboardInterrupt:
        raise
    except Exception as e:
        pe = getattr(sys, "print_exception", None)
        if pe is not None:
            pe(e)
        else:
            import traceback
            traceback.print_exception(type(e), e, e.__traceback__)


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
                     timer_id=-1):
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
    btn = Pin(button_pin, Pin.IN, Pin.PULL_UP)
    _singleton = Launcher(btn, poll_ms=poll_ms)
    _singleton._timer = Timer(timer_id)
    _singleton._timer.init(
        period=poll_ms, mode=Timer.PERIODIC, callback=_singleton._tick)
    return _singleton


# ---- entry points ----

def run(program_path=DEFAULT_PROGRAM_PATH, button_pin=DEFAULT_BUTTON_PIN,
        poll_ms=DEFAULT_POLL_MS, timer_id=-1):
    """Install the button watcher and block on the cooperative drain
    loop. Called from the frozen ``main.py``.

    Intentionally blocks forever. If ``openbricks-dev run`` later sends
    a Ctrl-C over the REPL to interrupt this loop, the Timer stays
    alive (we never ``deinit`` it) so subsequent ``run_program`` /
    button-press start continue to work.
    """
    launcher = _ensure_launcher(
        button_pin=button_pin, poll_ms=poll_ms, timer_id=timer_id)
    launcher._program_path = program_path
    print("openbricks: idle. Press button to run", program_path)
    while True:
        launcher._drain_pending()
        time.sleep_ms(poll_ms)


def run_program(program_path=DEFAULT_PROGRAM_PATH):
    """Client-triggered entry for ``openbricks-dev run``.

    Sets the ``_running`` flag so a button press routes through
    ``_request_interrupt``, then exec's the program in the main
    thread. Propagates ``KeyboardInterrupt`` so the raw-REPL
    disconnect signals "stopped" back to the client (which then
    exits).
    """
    launcher = _ensure_launcher()
    launcher._running = True
    try:
        _exec_program_raw(program_path)
    finally:
        launcher._running = False
