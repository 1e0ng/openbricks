# SPDX-License-Identifier: MIT
"""
Short press on the BLE-toggle button toggles BLE on/off.

Wires a ``machine.Timer``-driven poll loop (default 50 ms) against a
``Button``-conformant object and calls ``openbricks.bluetooth.toggle()``
once per press-release cycle. State is persisted via NVS by the
``bluetooth`` module, so the new value survives reboots.

The same poll loop doubles as the **run indicator**: while a user
program is executing (``launcher.program_running()``), the status LED
flashes at 5 Hz instead of holding a solid colour — blue when BLE is
on, yellow when it's off, plain on/off blinking on single-colour
LEDs. When the program stops, the LED returns to its idle state
(solid state colour on RGB hubs, dark on single-colour hubs). It
renders the **transfer indicator**: while a host tool is writing to
the hub over BLE (``openbricks upload`` / ``run`` staging a program —
``ble_repl.host_active()``), the LED flashes purple at 10 Hz,
outranking the run blink. And it renders the **press
acknowledgment**: every program-button press flashes the LED for a
moment — red for the press that starts a run, green for the press
that stops one (``notify_press()``, called by the launcher's press
detectors) — outranking both.

This is a different physical button from the one ``openbricks.launcher``
watches for program start/stop — the BLE toggle lives on its own
GPIO (default 38 on the S3, 5 on the classic ESP32 — see :class:`openbricks.hub.Hub`) while the program
button is on GPIO 39. Two pins → no duration-based dispatch, every
press on this pin means "flip BLE".

Usage from ``main.py``:

    from openbricks import bluetooth
    from openbricks.bluetooth_button import BluetoothToggleButton
    from openbricks.hub import ESP32DevkitHub

    bluetooth.apply_persisted_state()
    hub = ESP32DevkitHub()
    BluetoothToggleButton(hub.bluetooth_button).start()

The helper is deliberately standalone (not baked into ``Hub``) so tests
can exercise it in isolation, and so boards without a button — or
users who want to drive the toggle from something other than a
physical press — can skip it.
"""

from machine import Timer


DEFAULT_POLL_MS = 50

# BLE on → blue, off → yellow. Picked for high contrast on the WS2812;
# override via ``BluetoothToggleButton(..., color_on=..., color_off=...)``
# if you want different hues.
DEFAULT_COLOR_ON  = (0, 0, 255)       # blue
DEFAULT_COLOR_OFF = (255, 200, 0)     # yellow

# Run-indicator blink: while a program runs the LED alternates
# state-colour / off, switching phase every RUN_BLINK_MS. 100 ms per
# phase = 5 Hz (3.7.0, asked as "double the 2 Hz": 125 ms is not a
# multiple of the 50 ms poll, so the nearest step up from 250 is 100
# — read as double-plus; was 500 ms/1 Hz until 1.80.0, 250 ms/2 Hz
# until 3.7.0).
RUN_BLINK_MS = 100

# Transfer indicator: while a host tool writes to the hub over BLE
# (an upload or a run's staging paste — ``ble_repl.host_active``),
# the LED alternates purple / off every TRANSFER_BLINK_MS. One poll
# tick per phase = 10 Hz, the fastest the poll can render and
# unmistakably busier than the run blink it outranks.
TRANSFER_BLINK_MS = 50
TRANSFER_COLOR = (160, 0, 255)      # purple

# Press acknowledgment: every program-button press paints the LED
# for PRESS_FLASH_MS before the normal presentation resumes — RED
# for the press that starts a run, GREEN for the press that stops
# one. The launcher's press detectors call notify_press(); the
# toggle's poll tick renders it, so the flash needs no timer of its
# own.
PRESS_FLASH_MS = 200
PRESS_COLOR_START = (255, 0, 0)     # red
PRESS_COLOR_STOP  = (0, 255, 0)     # green

_press_events = 0
_press_color = PRESS_COLOR_START


def notify_press(stop=False):
    """Record a program-button press. ``stop=True`` marks it as the
    press that stops a run (green flash); the default is a start
    press (red). The active :class:`BluetoothToggleButton` renders
    it on its next poll tick. Safe from any context — it only sets
    two module variables."""
    global _press_events, _press_color
    _press_color = PRESS_COLOR_STOP if stop else PRESS_COLOR_START
    _press_events += 1


def _launcher_program_running():
    """Default run-state probe: the launcher's module-level flag.

    Imported lazily so constructing a ``BluetoothToggleButton`` never
    drags the launcher in on hosts/tests that don't use it."""
    from openbricks import launcher
    return launcher.program_running()


def _ble_host_active():
    """Default transfer probe: ``ble_repl.host_active()`` — a BLE
    central wrote to the hub within the last second. Lazy for the same
    reason as the launcher probe."""
    from openbricks import ble_repl
    return ble_repl.host_active()


class BluetoothToggleButton:
    """Polls a button and toggles BLE on each press-release cycle.

    Optional RGB LED feedback (blue = BLE on, yellow = off). Call
    ``start()`` to begin polling on a ``machine.Timer``; ``stop()``
    releases the timer. The toggled state persists across reboots.
    """

    def __init__(self, button, led=None,
                 poll_ms=DEFAULT_POLL_MS, timer_id=1,
                 color_on=DEFAULT_COLOR_ON,
                 color_off=DEFAULT_COLOR_OFF,
                 program_running=None,
                 host_active=None):
        """
        Args:
            button: any object with a ``.pressed() -> bool`` method
                (the ``Button`` / ``PushButton`` from ``openbricks.hub``
                both qualify).
            led: optional RGB-capable ``StatusLED`` (i.e. one whose
                ``.rgb(r, g, b)`` is implemented — the
                ``NeoPixelLED`` on the S3 DevKitC-1 qualifies). When
                provided, ``start()`` immediately colours the LED based
                on the current persisted BLE state (blue = on, yellow
                = off) and each toggle recolours it. Pass ``None`` to
                skip LED feedback.
            poll_ms: polling period. Default 50 ms (20 Hz) — well under
                human reaction time, negligible CPU.
            timer_id: ``machine.Timer`` hardware ID (0..3 on
                ESP32-S3). Default 1 — the reserved inventory is
                0 = launcher poll, 1 = this toggle, 2 = motor_process,
                3 = the launcher's stop tick (``STOP_TIMER_ID``). The
                previous default ``-1`` (virtual timer) was supported
                by older MicroPython but raises ``ValueError: invalid
                Timer number`` on the v1.27+ MP we vendor.
            color_on, color_off: ``(r, g, b)`` tuples the LED is set to
                when BLE is enabled / disabled. Defaults: blue / yellow.
            program_running: zero-arg callable returning True while a
                user program executes — drives the 5 Hz run-indicator
                blink. Defaults to ``launcher.program_running`` (the
                flag every exec path maintains); tests inject their
                own probe.
            host_active: zero-arg callable returning True while a
                host tool is writing to the hub — drives the purple
                10 Hz transfer indicator. Defaults to
                ``ble_repl.host_active`` (stamped by every BLE write
                the bridge accepts); tests inject their own probe.
        """
        self._button = button
        self._led    = led
        self._poll_ms       = int(poll_ms)
        self._timer_id      = timer_id
        self._color_on      = tuple(color_on)
        self._color_off     = tuple(color_off)
        self._timer         = None
        self._program_running = (program_running if program_running
                                 is not None else _launcher_program_running)
        self._host_active = (host_active if host_active is not None
                             else _ble_host_active)
        # Run-indicator blink state. ``_blink_ticks`` converts the
        # wall-clock phase length into poll ticks (≥1 so a huge
        # poll_ms still blinks rather than dividing to zero).
        self._blink_ticks  = max(1, RUN_BLINK_MS // self._poll_ms)
        self._blink_count  = 0
        self._blink_lit    = True
        self._running_seen = False
        # Transfer-indicator blink state, same shape.
        self._transfer_ticks = max(1, TRANSFER_BLINK_MS // self._poll_ms)
        self._transfer_count = 0
        self._transfer_lit   = True
        self._transfer_seen  = False
        # Press-flash state: consume notify_press() events by counter
        # comparison (no shared flags to clear from other contexts).
        self._press_seen       = _press_events
        self._press_flash_left = 0
        # Stable callback object for bluetooth.add/remove_state_listener.
        # A bound method (``self._on_state_change``) is a fresh object on
        # every attribute access under MicroPython, where bound methods
        # compare by identity — remove() would never match. A closure
        # created once here keeps its identity for the toggle's lifetime.
        self._state_listener = (
            lambda enabled: self._paint(bool(enabled)))

        # Edge-detection state: True from the moment we first saw the
        # button pressed until the subsequent release (when we fire).
        self._was_pressed = False

    # ---- lifecycle ----

    def start(self):
        """Begin polling. Safe to call repeatedly — the second call is a no-op.

        On first call, paints the LED (if one was provided) to reflect
        the current persisted BLE state, and registers with
        ``bluetooth.add_state_listener`` so the LED also follows state
        changes made *without* the button — ``bluetooth.set_enabled``
        from user code or a tool over the REPL.
        """
        if self._timer is not None:
            return
        from openbricks import bluetooth
        bluetooth.add_state_listener(self._state_listener)
        self._apply_led_for_current_state()
        self._timer = Timer(self._timer_id)
        self._timer.init(
            period=self._poll_ms,
            mode=Timer.PERIODIC,
            callback=self._on_tick,
        )

    def stop(self):
        """Stop polling, release the timer, and unregister the LED
        state listener."""
        if self._timer is None:
            return
        from openbricks import bluetooth
        bluetooth.remove_state_listener(self._state_listener)
        self._timer.deinit()
        self._timer = None
        self._was_pressed = False
        self._blink_count  = 0
        self._blink_lit    = True
        self._running_seen = False
        self._transfer_count = 0
        self._transfer_lit   = True
        self._transfer_seen  = False
        self._press_flash_left = 0

    # ---- tick body ----

    def _on_tick(self, _timer):
        try:
            self._on_tick_body()
        except KeyboardInterrupt:
            # Same relay as launcher._tick: a hard-button stop
            # interrupt that lands in this poll callback must be
            # re-posted, not eaten with the callback's unwind.
            from openbricks.launcher import _resignal_stop_interrupt
            _resignal_stop_interrupt()

    def _on_tick_body(self):
        if self._button.pressed():
            self._was_pressed = True
        elif self._was_pressed:
            # Release after a press — fire once.
            self._was_pressed = False
            self._fire()
        # Precedence: the press flash outranks everything for its
        # short window, the transfer indicator outranks the run
        # blink, and the indicators ride every other tick — including
        # ticks where the button is held, so a press mid-run doesn't
        # freeze the blink.
        if self._press_flash_tick():
            return
        if self._transfer_indicator_tick():
            return
        self._run_indicator_tick()

    # ---- press acknowledgment (red flash on the program button) ----

    def _press_flash_tick(self):
        """Render the red press-acknowledgment window. Returns True
        while the flash owns the LED (the run indicator pauses
        underneath and re-enters cleanly afterwards)."""
        if self._press_seen != _press_events:
            self._press_seen = _press_events
            if self._led is None:
                return False
            self._press_flash_left = max(
                1, PRESS_FLASH_MS // self._poll_ms)
            self._press_show()
            return True
        if not self._press_flash_left:
            return False
        self._press_flash_left -= 1
        if self._press_flash_left:
            return True
        # Window over: hand the LED back. Clearing the seen flags
        # makes the transfer / run indicators re-enter with a fresh
        # lit phase on this same tick; at idle, repaint the solid
        # state colour.
        self._transfer_seen = False
        if self._program_running():
            self._running_seen = False
        else:
            self._restore_idle_led()
        return False

    def _press_show(self):
        try:
            self._led.rgb(*_press_color)
        except NotImplementedError:
            self._led.on()

    # ---- transfer indicator (purple 10 Hz while the host writes) ----

    def _transfer_indicator_tick(self):
        """Flash purple while a host tool is writing to the hub.
        Returns True while the indicator owns the LED; when the
        transfer ends it hands back exactly like the press flash —
        the run blink re-enters with a fresh lit phase, or the idle
        colour is repainted."""
        if self._led is None:
            return False
        if not self._host_active():
            if self._transfer_seen:
                self._transfer_seen = False
                if self._program_running():
                    self._running_seen = False
                else:
                    self._restore_idle_led()
            return False
        if not self._transfer_seen:
            self._transfer_seen  = True
            self._transfer_count = 0
            self._transfer_lit   = True
            self._transfer_show()
            return True
        self._transfer_count += 1
        if self._transfer_count >= self._transfer_ticks:
            self._transfer_count = 0
            self._transfer_lit = not self._transfer_lit
            self._transfer_show()
        return True

    def _transfer_show(self):
        if not self._transfer_lit:
            self._led.off()
            return
        try:
            self._led.rgb(*TRANSFER_COLOR)
        except NotImplementedError:
            self._led.on()

    # ---- run indicator (5 Hz blink while a program executes) ----

    def _run_indicator_tick(self):
        """Blink the LED while a user program runs; restore the idle
        state when it stops.

        Lit phase re-reads the BLE state each time, so toggling BLE
        mid-run switches the blink colour within one phase. On
        single-colour LEDs the blink is plain on/off and idle is dark
        (matching their existing idle state — ``_paint`` never touches
        them)."""
        if self._led is None:
            return
        if not self._program_running():
            if self._running_seen:
                self._running_seen = False
                self._restore_idle_led()
            return
        if not self._running_seen:
            # Program just started: begin the lit phase immediately so
            # the indicator reacts within one poll tick.
            self._running_seen = True
            self._blink_count = 0
            self._blink_lit = True
            self._blink_show()
            return
        self._blink_count += 1
        if self._blink_count >= self._blink_ticks:
            self._blink_count = 0
            self._blink_lit = not self._blink_lit
            self._blink_show()

    def _blink_show(self):
        """Render the current blink phase: state colour (or plain on)
        when lit, off when dark."""
        if not self._blink_lit:
            self._led.off()
            return
        from openbricks import bluetooth
        color = (self._color_on if bluetooth.is_enabled()
                 else self._color_off)
        try:
            self._led.rgb(*color)
        except NotImplementedError:
            self._led.on()

    def _restore_idle_led(self):
        """Program ended: back to the idle presentation. RGB hubs show
        the solid BLE-state colour; single-colour LEDs go dark (their
        idle state — ``_paint`` no-ops on them, so ``off`` is the only
        way a stale lit phase gets cleaned up)."""
        from openbricks import bluetooth
        color = (self._color_on if bluetooth.is_enabled()
                 else self._color_off)
        try:
            self._led.rgb(*color)
        except NotImplementedError:
            self._led.off()

    def _fire(self):
        # Imported inside the method so tests that don't install the BLE
        # fake don't explode at module-load time. In production both
        # imports succeed because the firmware freezes the module in.
        # The LED repaint rides the state listener registered in
        # ``start()`` — ``toggle()`` fires it.
        from openbricks import bluetooth
        bluetooth.toggle()

    def _apply_led_for_current_state(self):
        """Paint the LED to match the current persisted BLE state.
        Early-out with no LED so LED-less hubs skip the NVS read."""
        if self._led is None:
            return
        from openbricks import bluetooth
        self._paint(bluetooth.is_enabled())

    def _paint(self, enabled):
        """Paint the LED for ``enabled``, if an RGB-capable LED was
        provided. Silently no-ops on plain on/off LEDs (whose
        ``.rgb()`` raises ``NotImplementedError``) so the hub can pass
        ``self.led`` unconditionally without caring which variant it
        is."""
        if self._led is None:
            return
        color = self._color_on if enabled else self._color_off
        try:
            self._led.rgb(*color)
        except NotImplementedError:
            pass
