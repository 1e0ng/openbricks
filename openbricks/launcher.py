# SPDX-License-Identifier: MIT
"""
Button-gated user-program launcher.

Pybricks-style workflow:

* ``openbricks-dev download`` stages a script at ``/program.py`` but
  does not run it — the user presses the hub button to launch.
* ``openbricks-dev run`` stages the same script and then triggers the
  launcher immediately. Output streams back to the client; the hub
  button stops the program; when the program stops, the client exits.

Both paths go through the same ``Launcher`` singleton so button-stop
works regardless of how the program was started. A persistent
``machine.Timer`` polls the hub button and stays alive for the whole
hub uptime; the watcher is installed on the first call to
``_ensure_launcher()`` (from ``run`` in the frozen main.py, or from
``run_program`` when the client triggers one).

Wiring:

* Short press (held <``long_press_ms``): start if idle, stop if running.
* Long press (held >=``long_press_ms``): owned by
  ``openbricks.bluetooth_button.BluetoothToggleButton``. Both watchers
  share the pin safely — the launcher only reacts to short presses,
  the BLE watcher only reacts to long ones.

Typical ``main.py`` (the firmware ships a frozen default; users can
override by writing to ``/main.py`` in VFS):

    from openbricks import bluetooth, launcher
    bluetooth.apply_persisted_state()
    launcher.run()          # installs watcher + blocks on the idle loop
"""

import sys
import time


DEFAULT_BUTTON_PIN    = 5
DEFAULT_POLL_MS       = 50
DEFAULT_LONG_PRESS_MS = 1000  # aligned with bluetooth_button threshold
DEFAULT_PROGRAM_PATH  = "/program.py"


class Launcher:
    """Shared state for the button watcher.

    Tests instantiate this directly and drive ``_tick`` with a fake
    Pin; production code uses ``_ensure_launcher()`` below which
    installs a singleton + Timer.
    """

    def __init__(self, button, program_path=DEFAULT_PROGRAM_PATH,
                 poll_ms=DEFAULT_POLL_MS,
                 long_press_ms=DEFAULT_LONG_PRESS_MS):
        self._btn            = button      # anything with ``.value()``, 0 = pressed
        self._program_path   = program_path
        self._poll_ms        = poll_ms
        self._long_press_ms  = long_press_ms

        self._running        = False
        self._press_start_ms = None
        self._already_fired_long = False
        self._pending        = None        # None | "start" | "stop"
        # Timer stays alive for hub uptime — we never ``.deinit()`` it.
        # Keeping the reference here stops GC from collecting it.
        self._timer          = None

    # ---- timer callback ----

    def _tick(self, _timer=None):
        """Called on every ``poll_ms`` tick. Detects press/release edges
        and classifies short vs long. Safe for IRQ context."""
        pressed = self._btn.value() == 0

        if pressed:
            if self._press_start_ms is None:
                self._press_start_ms = time.ticks_ms()
                self._already_fired_long = False
            elif not self._already_fired_long and \
                    time.ticks_diff(time.ticks_ms(), self._press_start_ms) >= self._long_press_ms:
                self._already_fired_long = True
            return

        # Released.
        if self._press_start_ms is None:
            return
        duration = time.ticks_diff(time.ticks_ms(), self._press_start_ms)
        self._press_start_ms = None
        if self._already_fired_long:
            return
        if duration < self._long_press_ms:
            if self._running:
                _request_interrupt(self)
            else:
                self._pending = "start"

    def _drain_pending(self):
        """Consume a queued 'start' from the button-gated loop.

        Called from ``run()``'s cooperative drain loop. ``run_program``
        doesn't use this path — it sets ``_running`` directly and lets
        the caller drive exec.
        """
        if self._pending == "start" and not self._running:
            self._pending = None
            self._running = True
            try:
                _exec_program(self._program_path)
            finally:
                self._running = False
            print("openbricks: idle. Short-press button to run", self._program_path)
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


# ---- program exec helpers ----

def _exec_program_raw(program_path):
    """Load and run ``program_path`` in a fresh namespace. Propagates
    ``KeyboardInterrupt``; prints other exceptions and returns.

    Used by ``run_program`` where the client needs to see
    ``KeyboardInterrupt`` propagate (so the disconnect signals "stopped"
    back to the user's terminal).
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
                     long_press_ms=DEFAULT_LONG_PRESS_MS,
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
    _singleton = Launcher(btn, poll_ms=poll_ms, long_press_ms=long_press_ms)
    _singleton._timer = Timer(timer_id)
    _singleton._timer.init(
        period=poll_ms, mode=Timer.PERIODIC, callback=_singleton._tick)
    return _singleton


# ---- entry points ----

def run(program_path=DEFAULT_PROGRAM_PATH, button_pin=DEFAULT_BUTTON_PIN,
        poll_ms=DEFAULT_POLL_MS, long_press_ms=DEFAULT_LONG_PRESS_MS,
        timer_id=-1):
    """Install the button watcher and block on the cooperative drain
    loop. Called from the frozen ``main.py``.

    Intentionally blocks forever. If ``openbricks-dev run`` later sends
    a Ctrl-C over the REPL to interrupt this loop, the Timer stays
    alive (we never ``deinit`` it) so subsequent ``run_program`` /
    button-press stop continue to work.
    """
    launcher = _ensure_launcher(
        button_pin=button_pin, poll_ms=poll_ms,
        long_press_ms=long_press_ms, timer_id=timer_id)
    launcher._program_path = program_path
    print("openbricks: idle. Short-press button to run", program_path)
    while True:
        launcher._drain_pending()
        time.sleep_ms(poll_ms)


def run_program(program_path=DEFAULT_PROGRAM_PATH):
    """Client-triggered entry for ``openbricks-dev run``.

    Sets the ``_running`` flag so a short button press routes through
    ``_request_interrupt``, then exec's the program in the main
    thread. Propagates ``KeyboardInterrupt`` so the raw-REPL disconnect
    signals "stopped" back to the client (which then exits).
    """
    launcher = _ensure_launcher()
    launcher._running = True
    try:
        _exec_program_raw(program_path)
    finally:
        launcher._running = False
