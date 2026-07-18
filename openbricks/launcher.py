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

* Press while idle → start ``/program.py`` (on release, with a
  post-stop lockout against bounce).
* Press while running → the stop fires on press-DOWN: the e-stop
  latch engages (motors halt + motion commands raise — see
  :mod:`openbricks.estop`), and a ``KeyboardInterrupt`` injection is
  requested and *retried* until the program is actually dead.

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

# PCNT unit for the hardware press latch. Encoder motors claim units
# 0/1 (one unit per PCNTEncoder, see drivers/mg370.py); unit 3 exists
# on every PCNT-capable chip (classic ESP32 has 8 units, S3 has 4).
STOP_PCNT_UNIT = 3
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
        self._press_stopped  = False       # this press already fired STOP
        self._last_stop_ms   = None        # when the last STOP fired
        self._lockout_until_ms = None      # starts swallowed until here
        self._stop_retry_ms  = None        # last injection request time
        self._stop_retry_count = 0         # injections re-requested this stop
        self._tick_last_ms   = None        # last _tick run (starvation detect)
        self._tick_gap_max   = 0           # worst inter-tick gap seen
        self._press_pcnt     = None        # hardware falling-edge counter
        self._press_count_seen = 0         # counter value already consumed
        self._raw_last       = False       # last raw sample (debounce)
        self._start_press_open_ms = None   # counter-start press in flight
        self._press_consume_release = False  # eat this press's release
        self._raw_stable     = 0           # consecutive equal samples
        self._run_started_ms = None        # when _running went True
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

    def _sync_press_counter(self):
        """Mark the hardware press counter's current value as
        consumed. Called at idle and before each run starts, so edges
        counted OUTSIDE a run never stop the next one at birth."""
        if self._press_pcnt is None:
            return
        try:
            self._press_count_seen = self._press_pcnt.value()
        except Exception:
            pass

    def _mark_idle_alive(self):
        self._idle_drain_ms = _now_ms()

    def _idle_loop_alive(self):
        """True while the main-thread idle loop is actively draining
        (last pass within a few poll periods)."""
        if self._idle_drain_ms is None:
            return False
        return _ticks_diff(_now_ms(), self._idle_drain_ms) < self._poll_ms * 4

    # ---- timer callback ----

    # After a STOP, presses can't START the program again until this
    # lockout passes. The stop unwinds the program within milliseconds,
    # so contact bounce / finger re-contact after the stopping press
    # reads as a *fresh* press with ``_running`` already False —
    # without the lockout that second event restarted the program the
    # user just stopped. Anchored at the stopping press's RELEASE
    # (bounce is a release-adjacent phenomenon), re-armed at stop-fire
    # as a fallback for presses whose release was never observed. The
    # original 750 ms from stop-FIRE swallowed deliberate quick
    # restarts ("start press not detected" bench report); 400 ms from
    # release still swallows bounce and finger re-contact with
    # margin. 500 (was 400): the 1.15.3 debounce delays a re-contact
    # press's DISPATCH by up to two polls, so the window covers
    # re-contact at release+200 ms dispatching at ~release+400 ms.
    START_LOCKOUT_MS = 500

    # While a stop is in flight but the program hasn't died yet, the
    # injection is re-requested at this cadence. The injected
    # KeyboardInterrupt raises in whatever main-thread frame is
    # executing — a scheduled callback (e.g. the BLE TX flush) can eat
    # it — so a one-shot request meant a press could be silently lost
    # ("first press ignored, second press works"). Retries make an
    # eaten injection cost one retry period instead of the press. The
    # robot itself is already stopping either way: the e-stop latch
    # engaged at press-down, independent of injection delivery.
    STOP_RETRY_MS = 300

    # Contact-chatter defences (1.15.3). Bench event-ring capture of
    # "pressed start 4 times, only the 4th worked": the START press's
    # release chatter re-closed the contact and KILLED the newborn
    # run — once as a phantom press-down(running) 54 ms after start
    # (level path), twice as PCNT falling edges 100-180 ms after
    # start (hardware latch). Two defences, one per detector:
    #
    # * DEBOUNCE_TICKS — a level CHANGE must hold for this many
    #   consecutive polls before it's believed. Chatter flickers are
    #   ~10-20 ms; two 50 ms polls reject them while costing a real
    #   press ~50 ms of latency.
    # * RUN_START_GRACE_MS — for this long after a run starts, PCNT
    #   edges are CONSUMED as the start press's own chatter instead
    #   of fired as stops. A real second press physically can't
    #   arrive that fast, and the debounced level path still covers
    #   the window regardless.
    DEBOUNCE_TICKS = 2
    RUN_START_GRACE_MS = 400

    # Counter-driven START (1.15.4). The debounce above made fast
    # taps unreliable: a press must span two 50 ms polls to be
    # believed, so a crisp ~60 ms tap is a coin flip and a really
    # fast one lands between polls entirely (bench: "press too fast
    # -> doesn't start; a bit longer -> starts", with an EMPTY event
    # ring). The PCNT counter already sees every tap's edge in
    # silicon — so at idle the counter is now the START trigger
    # (press-DOWN latency, no tap too fast), the level path demotes
    # to state tracking, and edges within START_PRESS_OPEN_MS of a
    # dispatch are the same press's chatter, consumed silently.
    START_PRESS_OPEN_MS = 600

    def _fire_stop(self):
        """Everything a stop press triggers, in order of importance:
        latch the e-stop (motors die + motion commands raise, no
        interrupt needed), then request the interrupt injection that
        tears the program down."""
        from openbricks import estop
        self._last_stop_ms = _now_ms()
        self._stop_retry_ms = self._last_stop_ms
        self._stop_retry_count = 0
        self._lockout_until_ms = self._last_stop_ms + self.START_LOCKOUT_MS
        _event("stop-fire")
        try:
            estop.engage()
            # Breadcrumb AFTER the kill so the file write can't delay
            # it; no injection is pending yet (that request comes
            # next), so no KeyboardInterrupt can land in this write.
            _note("estop engaged: motors killed, motion latched")
        except Exception as e:
            # The latch itself must never fail the stop request.
            _note("estop engage FAILED: %r" % (e,))
        _request_stop(self)

    def _tick(self, _timer=None):
        """Called on every ``poll_ms`` tick.

        While a program runs, a STOP fires on **press-down**
        (Pybricks-style: reacts a poll period sooner, and the release
        that follows is consumed so it can't double-fire). The stop
        engages the e-stop latch — the robot halts and motion commands
        raise regardless of interrupt delivery — and the interrupt
        request is retried every ``STOP_RETRY_MS`` until the program
        is actually dead. While idle, a START fires on release of a
        full press-release cycle — unless it lands inside the
        post-stop lockout, which swallows the bounce that used to
        restart a just-stopped program.

        This (a soft Python callback) only *detects* the press and sets a
        flag — it can't inject the interrupt, because a pending exception
        set in a Python callback frame unwinds the callback, not the
        program. The native C-function ``stop_tick`` Timer callback (set
        up in ``_ensure_launcher``) does the injection."""
        # BLE TX liveness backstop: revive a flush chain that died with
        # bytes still buffered (scheduler queue full at re-schedule
        # time, or a paced notify-failure retry — see
        # ``ble_repl._flush``). Runs before the button logic so the
        # early ``return``s below can't skip it. Wrapped like
        # ``estop.engage()`` in ``_fire_stop``: a backstop must never
        # kill the tick that also owns the stop button.
        try:
            from openbricks import ble_repl
            ble_repl.pump_tx()
        except Exception:
            pass
        # Starvation self-detection: Timer callbacks dispatch through
        # micropython.schedule's bounded queue, and a full queue DROPS
        # ticks silently — during such a gap a button press is
        # invisible. Measure the gap between consecutive runs and
        # leave a stamped note in the run log when it stretches, so a
        # silently-missed press shows WHY right where it happened.
        now = _now_ms()
        if self._tick_last_ms is not None:
            gap = _ticks_diff(now, self._tick_last_ms)
            if gap > self._tick_gap_max:
                self._tick_gap_max = gap
            if gap >= self._poll_ms * 4:
                _note("tick starved %d ms (scheduler saturated?)" % gap)
        self._tick_last_ms = now
        if self._running:
            # Hardware press latch: the PCNT peripheral counted every
            # falling edge on the button in silicon, including during
            # scheduler blackouts (a program blocking in C I2C stalls
            # the drain for 100-200 ms, and machine.Timer ticks are
            # silently dropped while the queue is full — a quick
            # ~120-160 ms press fit entirely inside such a gap and
            # was never seen; the 4-presses-1-stop bench repro). The
            # tick may run LATE, but the edge count means the press
            # cannot be LOST.
            if self._press_pcnt is not None and self._stop_retry_ms is None:
                try:
                    n = self._press_pcnt.value()
                except Exception:
                    n = self._press_count_seen
                if n != self._press_count_seen:
                    in_grace = (
                        self._run_started_ms is not None
                        and _ticks_diff(_now_ms(), self._run_started_ms)
                        < self.RUN_START_GRACE_MS)
                    if in_grace:
                        # The start press's own release chatter.
                        _event("latch-grace-consumed",
                               n - self._press_count_seen)
                        self._press_count_seen = n
                    else:
                        self._press_count_seen = n
                        self._press_stopped = True  # consume the release
                        _event("latch-stop")
                        _note("button press latched by hardware counter -> stop")
                        self._fire_stop()
            if self._stop_retry_ms is not None and _ticks_diff(
                    _now_ms(), self._stop_retry_ms) >= self.STOP_RETRY_MS:
                # A stop is in flight but the program is still alive —
                # the previous injection was eaten. Re-request. (No log
                # note here: an injection is pending and could land
                # inside the file write, eaten by _tick instead of the
                # program. The count lands in the stop debrief line.)
                self._stop_retry_ms = _now_ms()
                self._stop_retry_count += 1
                _request_stop(self)
        else:
            self._stop_retry_ms = None
            if self._press_pcnt is not None:
                # Counter-driven START: a counted falling edge at idle
                # IS a press, regardless of whether the 50 ms level
                # sampling ever catches it.
                try:
                    n = self._press_pcnt.value()
                except Exception:
                    n = self._press_count_seen
                if n != self._press_count_seen:
                    self._press_count_seen = n
                    in_lockout = (
                        self._lockout_until_ms is not None
                        and _ticks_diff(self._lockout_until_ms, now) > 0)
                    if in_lockout:
                        # Post-stop bounce / re-contact edges.
                        _event("start-latch-swallowed", "lockout")
                        print("openbricks: start press ignored "
                              "(post-stop lockout)")
                    elif self._start_press_open_ms is not None:
                        # Chatter edges of the press that already
                        # dispatched.
                        _event("start-latch-consumed", "same-press")
                    else:
                        self._start_press_open_ms = now
                        _event("start-latch")
                        _request_start(self)
            if (self._start_press_open_ms is not None
                    and _ticks_diff(now, self._start_press_open_ms)
                    > self.START_PRESS_OPEN_MS):
                self._start_press_open_ms = None
        raw = self._btn.value() == 0
        if raw == self._raw_last:
            if self._raw_stable < self.DEBOUNCE_TICKS:
                self._raw_stable += 1
        else:
            self._raw_stable = 1
            self._raw_last = raw
        if self._raw_stable >= self.DEBOUNCE_TICKS:
            pressed = raw
        else:
            # Mid-flicker: hold the previous debounced state. A
            # chatter blip spans one poll at most; a real press or
            # release confirms on the next tick.
            pressed = self._was_pressed
            if raw != self._was_pressed:
                _event("bounce-filtered", "press" if raw else "release")
        if pressed:
            if not self._was_pressed:
                self._was_pressed = True
                _event("press-down", "running" if self._running else "idle")
                if self._running:
                    self._press_stopped = True
                    if self._stop_retry_ms is None:
                        # Not already stopping (the hardware latch may
                        # have fired for this same press). Note BEFORE
                        # _fire_stop: the injection request is not
                        # pending yet, so no KeyboardInterrupt can land
                        # inside this file write.
                        _note("button pressed -> stop")
                        self._fire_stop()
                else:
                    self._press_stopped = False
                    if self._start_press_open_ms is not None:
                        # The counter already dispatched this press's
                        # start — its release (which may arrive after
                        # the program is running) must not dispatch
                        # again or read as a mid-hold stop.
                        self._press_consume_release = True
            return
        if not self._was_pressed:
            return
        # Released after a press — dispatch at most once.
        self._was_pressed = False
        if self._press_stopped:
            # Release of the press that already fired the stop. Bounce
            # follows THIS moment — re-anchor the lockout here.
            self._press_stopped = False
            self._lockout_until_ms = _now_ms() + self.START_LOCKOUT_MS
            _event("release", "stop-consumed")
            return
        if self._press_consume_release:
            # Release of the press whose START the counter already
            # dispatched. Without this, a long-held start press whose
            # program is up by release time would hit the mid-hold
            # branch below and STOP the run it started.
            self._press_consume_release = False
            self._start_press_open_ms = None
            _event("release", "start-consumed")
            return
        if self._running:
            # Program came up between press-down and release (remote
            # start mid-hold) — a button event during a run means stop.
            if self._stop_retry_ms is None:
                _note("button pressed -> stop")
                self._fire_stop()
            return
        if self._lockout_until_ms is not None:
            remaining = _ticks_diff(self._lockout_until_ms, _now_ms())
            if remaining > 0:
                # Bounce / re-contact right after a stop: swallow it —
                # but SAY so. A silently-swallowed deliberate press
                # reads as "start button not detected".
                _event("release", "lockout-swallowed", remaining)
                print("openbricks: start press ignored "
                      "(%d ms left of post-stop lockout)" % remaining)
                return
            self._lockout_until_ms = None
        _event("release", "start")
        _request_start(self)

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
            from openbricks import estop
            estop.clear()   # fresh run — stale latch must not kill it
            self._sync_press_counter()
            self._run_started_ms = _now_ms()
            self._running = True
            try:
                _exec_program(self._program_path, origin="button press")
            finally:
                self._running = False
                estop.clear()
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


def _run_header(program_path):
    """Environment summary for the run log's first line: firmware
    version, program path, milliseconds since boot (a tiny uptime
    right after an unexpected reboot is itself a clue), and free
    heap. Every probe is independent — a failing one reports ? and
    must not cost the run its log."""
    try:
        from openbricks import __version__ as _ver
    except Exception:
        _ver = "?"
    try:
        import time as _t
        up = _t.ticks_ms()
    except Exception:
        up = "?"
    try:
        import gc
        free = gc.mem_free()
    except Exception:
        free = "?"
    return "firmware %s | program %s | uptime %s ms | free %s B" % (
        _ver, program_path, up, free)


def _stop_debrief():
    """One-line summary of the in-flight stop for the run log: how
    long after the press the program actually died, and how many
    injection retries that took. A KeyboardInterrupt with no recorded
    press is a host-side Ctrl-C (openbricks run / stop)."""
    inst = _singleton
    if inst is None or inst._last_stop_ms is None:
        return "no stop press recorded (host Ctrl-C?)"
    return "%d ms after press, %d retries; worst tick gap %d ms" % (
        _ticks_diff(_now_ms(), inst._last_stop_ms),
        inst._stop_retry_count, inst._tick_gap_max)


# In-memory button/dispatch event ring. Run logs only exist while a
# program runs — an IDLE press that gets swallowed (or never seen)
# leaves no trace anywhere else. 64 entries of (ticks_ms, tag, args),
# newest kept; dump with ``launcher.dump_events()`` over USB after a
# "my start press did nothing" report.
_EVENTS = []
_EVENTS_MAX = 64
_EVENTS_NEXT = [0]


def _event(tag, *args):
    try:
        entry = (_now_ms(), tag, args)
        if len(_EVENTS) < _EVENTS_MAX:
            _EVENTS.append(entry)
        else:
            i = _EVENTS_NEXT[0]
            _EVENTS[i] = entry
            _EVENTS_NEXT[0] = (i + 1) % _EVENTS_MAX
    except MemoryError:
        pass


def dump_events():
    """Print the launcher button-event ring, oldest first."""
    n = len(_EVENTS)
    start = _EVENTS_NEXT[0] if n == _EVENTS_MAX else 0
    print("launcher event ring (last %d):" % n)
    for k in range(n):
        ms, tag, args = _EVENTS[(start + k) % n]
        print("  %d %s %s" % (ms, tag, args))


def _install_press_counter(button_pin):
    """Hardware falling-edge counter on the stop button (esp32.PCNT).

    Tick-based level polling alone LOSES short presses: machine.Timer
    callbacks dispatch through micropython.schedule's bounded queue,
    and a program blocking inside C (I2C sensor reads) stalls the
    drain — measured blackouts reach ~200 ms while a quick press is
    ~120-160 ms. The PCNT peripheral counts the edge in silicon
    regardless of what Python is doing. Returns None off-ESP32 or if
    the unit is taken; the announcement makes the degraded (tick-bound)
    mode visible instead of silent."""
    try:
        import esp32
        from machine import Pin
        pcnt = esp32.PCNT(
            STOP_PCNT_UNIT,
            pin=Pin(button_pin, Pin.IN, Pin.PULL_UP),
            falling=esp32.PCNT.INCREMENT,
            rising=esp32.PCNT.IGNORE,
            filter=1023,   # max hardware glitch filter (~12.8 us)
        )
        pcnt.start()
        return pcnt
    except Exception as e:
        print("openbricks: hardware press latch unavailable (%r); "
              "stop presses are tick-bound" % (e,))
        return None


def _note(text):
    """Mirror a button event into the active run's log file (stamped;
    file only). Guarded — a logging failure must never break the tick
    that owns the stop button."""
    try:
        from openbricks import log as _log
        _log.note(text)
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
    _event("scheduled-start-exec")
    print("openbricks: starting from REPL context — the stop button "
          "is unavailable for this run (use 'openbricks stop').")
    from openbricks import estop
    estop.clear()
    launcher_instance._sync_press_counter()
    launcher_instance._run_started_ms = _now_ms()
    launcher_instance._running = True
    try:
        _exec_program(launcher_instance._program_path,
                      origin="button press (degraded REPL-parked path)")
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
        _event("start-dispatch", "pending")
        return
    _event("start-dispatch", "degraded-schedule")
    _start_via_schedule(launcher_instance)


def _start_via_schedule(launcher_instance):
    """Degraded-path dispatch, split out as a patchable seam for tests.
    Falls back to the ``_pending`` flag where ``micropython.schedule``
    isn't available (CPython)."""
    try:
        import micropython
        micropython.schedule(_scheduled_start, launcher_instance)
    except RuntimeError:
        # Scheduler queue full — the pending flag is a void if the
        # idle loop is dead, but at least the ring SAYS so now.
        _event("start-dispatch", "schedule-full")
        launcher_instance._pending = "start"
    except (ImportError, AttributeError):
        launcher_instance._pending = "start"


# ---- program exec helpers ----

def _exec_program_raw(program_path, origin=None):
    """Load and run ``program_path`` in a fresh namespace. Propagates
    ``KeyboardInterrupt``; prints other exceptions and returns.
    ``origin`` (e.g. "button press", "remote (openbricks run)") is
    written as the run log's first stamped line, so every button
    press that starts a program leaves a log entry.

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
            sess.write_text("started: %s | %s\n" % (
                origin or "unknown", _run_header(program_path)))
            try:
                exec(code, {"__name__": "__main__"})
            except KeyboardInterrupt:
                # The button-stop's injected KeyboardInterrupt (and a REPL
                # Ctrl-C from ``openbricks run``) both unwind to here.
                # Disarm FIRST: the stop request is retried until the
                # program dies, and a retry landing inside the cleanup
                # below would abort the very motor-stop it asked for.
                _arm_stop_button(False)
                # Then stop every motor before propagating so the robot
                # halts no matter how the program was running.
                # Idempotent if already stopped.
                _stop_all_motors()
                # Safe to write now: the button is disarmed, so no
                # further injection can land inside this file write.
                sess.write_text(
                    "stopped: KeyboardInterrupt (%s)\n" % _stop_debrief())
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


def _exec_program(program_path, origin=None):
    """Button-gated path: swallow ``KeyboardInterrupt`` and missing-file
    errors so the idle loop keeps running between button presses."""
    try:
        _exec_program_raw(program_path, origin=origin)
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
    _singleton._press_pcnt = _install_press_counter(button_pin)
    _singleton._sync_press_counter()
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
    from openbricks import estop
    estop.clear()   # fresh run — a stale latch must not kill it at birth
    launcher._sync_press_counter()
    launcher._run_started_ms = _now_ms()
    launcher._running = True
    try:
        _exec_program_raw(program_path, origin="remote (openbricks run)")
    finally:
        launcher._running = False
        # Program is dead; release the latch so idle-time commands
        # (REPL experiments, the next run) work again.
        estop.clear()


def _reset_motor_process():
    """Best-effort wipe of the native scheduler's callback list.
    Imported lazily so unix MP / CPython tests without the C module
    don't fail on launcher import."""
    try:
        from _openbricks_native import motor_process
    except ImportError:
        return
    motor_process.reset()
