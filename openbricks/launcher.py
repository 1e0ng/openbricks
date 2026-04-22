# SPDX-License-Identifier: MIT
"""
Button-gated user-program launcher.

Pybricks-style workflow: ``openbricks-dev download`` stages a script on
the hub's filesystem but does **not** run it. The hub sits idle with
BLE up until the user presses the hub button — only then does the
program execute. Another press while a program is running raises
``KeyboardInterrupt`` in the running program, so the user can stop the
robot mid-run.

Wiring:

* Short press (held <``long_press_ms``): start if idle, stop if running.
* Long press (held >=``long_press_ms``): owned by
  ``openbricks.bluetooth_button.BluetoothToggleButton`` which uses the
  same GPIO to toggle BLE on/off. Both watchers share the pin safely —
  the launcher only reacts to short presses, and the BLE watcher only
  reacts to long ones.

The watcher runs off a ``machine.Timer`` so it can fire a stop request
even while user code is executing — timer IRQs schedule callbacks that
MicroPython processes between bytecodes, which is what lets a second
button press raise ``KeyboardInterrupt`` mid-program.

Typical ``main.py`` (the firmware ships a frozen default; users can
override by writing to ``/main.py`` in VFS):

    from openbricks import bluetooth, launcher
    bluetooth.apply_persisted_state()
    launcher.run()                  # blocks; polls button in a Timer
"""

import sys
import time


DEFAULT_BUTTON_PIN    = 5
DEFAULT_POLL_MS       = 50
DEFAULT_LONG_PRESS_MS = 1000  # aligned with bluetooth_button threshold
DEFAULT_PROGRAM_PATH  = "/program.py"


class Launcher:
    """Shared state for the button watcher.

    Tests instantiate this directly and drive its ``_tick`` method with
    a fake Pin; production code uses the ``run()`` module-level entry
    point which wires the real ``machine.Timer`` + ``Pin``.
    """

    def __init__(self, button, program_path=DEFAULT_PROGRAM_PATH,
                 poll_ms=DEFAULT_POLL_MS,
                 long_press_ms=DEFAULT_LONG_PRESS_MS):
        self._btn            = button      # anything with ``.value()``, 0 = pressed
        self._program_path   = program_path
        self._poll_ms        = poll_ms
        self._long_press_ms  = long_press_ms

        # Running state — exec writes True on entry, False on exit.
        self._running        = False
        # Press-tracking state — timer callback mutates these.
        self._press_start_ms = None
        self._already_fired_long = False
        # Pending action from the most recent short press; consumed by
        # ``_drain_pending`` called from the main-thread loop.
        self._pending        = None        # None | "start" | "stop"

    # ---- timer callback ----

    def _tick(self, _timer=None):
        """Called on every ``poll_ms``. Detects press/release edges and
        classifies short vs long presses, queuing the action for the
        main-thread drain. Safe to run from IRQ context — no allocations
        in the hot path (flag writes only)."""
        pressed = self._btn.value() == 0

        if pressed:
            if self._press_start_ms is None:
                self._press_start_ms = time.ticks_ms()
                self._already_fired_long = False
            elif not self._already_fired_long and \
                    time.ticks_diff(time.ticks_ms(), self._press_start_ms) >= self._long_press_ms:
                # This becomes a long press — BLE watcher handles it;
                # mark so we don't fire a short on release.
                self._already_fired_long = True
            return

        # Not pressed.
        if self._press_start_ms is None:
            return
        duration = time.ticks_diff(time.ticks_ms(), self._press_start_ms)
        self._press_start_ms = None
        if self._already_fired_long:
            # Release after a long press — already handled by BLE watcher.
            return
        if duration < self._long_press_ms:
            # Short press. Queue start or stop.
            if self._running:
                # Mid-run stop. Delegate to the module-level helper so
                # tests can swap it out without patching MicroPython's
                # read-only ``micropython`` module.
                _request_interrupt(self)
            else:
                # Idle → start. Can't exec in IRQ context; leave a
                # sentinel for the main-thread drain loop.
                self._pending = "start"

    # ---- main-thread drain ----

    def _drain_pending(self):
        """Consume any queued start request. Called from the blocking
        main loop in ``run()`` — that loop is what actually calls
        ``_exec_program``, since exec can't run from a timer callback.
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
            # Only happens on non-MicroPython (tests); MP uses schedule.
            self._pending = None


# ---- mid-run interrupt ----
#
# A short press while a program runs needs to raise KeyboardInterrupt
# inside the executing frame. MicroPython honours exceptions raised
# from ``micropython.schedule`` callbacks at the next bytecode
# boundary, which is exactly the hook we need.

def _raise_interrupt(_):
    raise KeyboardInterrupt


def _request_interrupt(launcher_instance):
    """Ask MicroPython to raise ``KeyboardInterrupt`` in the running
    program at the next bytecode boundary.

    Implemented as a module-level function (not a method) so tests can
    replace it wholesale — patching ``micropython.schedule`` fails under
    real MP because the ``micropython`` module is read-only. Tests
    assign their own callable to ``launcher._request_interrupt``
    instead.
    """
    try:
        import micropython
        micropython.schedule(_raise_interrupt, None)
    except (ImportError, AttributeError, RuntimeError):
        # No micropython.schedule (CPython) or queue full: fall back to
        # a flag the test harness + cooperative main loop can observe.
        launcher_instance._pending = "stop"


# ---- program exec helper ----

def _exec_program(program_path):
    """Load and run ``program_path`` in a fresh namespace.

    Any exception — including ``KeyboardInterrupt`` from a second-press
    stop — is caught and reported so the launcher loop can recover.
    """
    try:
        with open(program_path) as f:
            code = f.read()
    except OSError:
        print("openbricks: no program at", program_path)
        return
    try:
        exec(code, {"__name__": "__main__"})
        print("openbricks: program finished.")
    except KeyboardInterrupt:
        print("openbricks: stopped.")
    except Exception as e:
        # MicroPython uses ``sys.print_exception``; CPython has the same
        # information under ``traceback.print_exc``. Prefer the MP form
        # since that's where this runs 99% of the time.
        pe = getattr(sys, "print_exception", None)
        if pe is not None:
            pe(e)
        else:
            import traceback
            traceback.print_exception(type(e), e, e.__traceback__)


# ---- entry points ----

def run(program_path=DEFAULT_PROGRAM_PATH, button_pin=DEFAULT_BUTTON_PIN,
        poll_ms=DEFAULT_POLL_MS, long_press_ms=DEFAULT_LONG_PRESS_MS,
        timer_id=-1):
    """Block forever, polling the hub button from a ``machine.Timer``.

    Short-press → start ``program_path``. Second short-press while
    running → ``KeyboardInterrupt``. Long presses are ignored (they're
    for the BLE toggle watcher, which is typically wired by
    ``openbricks.hub.ESP32*Hub()``).
    """
    from machine import Pin, Timer
    btn = Pin(button_pin, Pin.IN, Pin.PULL_UP)
    launcher = Launcher(btn, program_path=program_path,
                        poll_ms=poll_ms, long_press_ms=long_press_ms)
    timer = Timer(timer_id)
    timer.init(period=poll_ms, mode=Timer.PERIODIC, callback=launcher._tick)

    print("openbricks: idle. Short-press button to run", program_path)
    try:
        # Cooperative main loop: wake periodically to drain any queued
        # start request. ``_exec_program`` is synchronous — while it
        # runs, the Timer keeps ticking and can schedule an interrupt.
        while True:
            launcher._drain_pending()
            time.sleep_ms(poll_ms)
    finally:
        timer.deinit()
