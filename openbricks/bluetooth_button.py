# SPDX-License-Identifier: MIT
"""
Long-press on the hub button toggles BLE on/off.

Wires a ``machine.Timer``-driven poll loop (default 50 ms) against a
``Button``-conformant object, watches for a sustained hold longer than
``long_press_ms`` (default 1 second), and calls
``openbricks.bluetooth.toggle()`` once per press. State is persisted
via NVS by the ``bluetooth`` module, so the new value survives reboots.

Usage from ``main.py``:

    from openbricks import bluetooth
    from openbricks.bluetooth_button import BluetoothToggleButton
    from openbricks.hub import ESP32DevkitHub

    bluetooth.apply_persisted_state()
    hub = ESP32DevkitHub()
    BluetoothToggleButton(hub.button).start()

The helper is deliberately standalone (not baked into ``Hub``) so tests
can exercise it in isolation, and so boards without a button — or
users who want to drive the toggle from something other than a
physical press — can skip it.
"""

import time

from machine import Timer


DEFAULT_LONG_PRESS_MS = 1000
DEFAULT_POLL_MS       = 50


class BluetoothToggleButton:
    def __init__(self, button, long_press_ms=DEFAULT_LONG_PRESS_MS,
                 poll_ms=DEFAULT_POLL_MS, timer_id=-1):
        """
        Args:
            button: any object with a ``.pressed() -> bool`` method
                (the ``Button`` / ``PushButton`` from ``openbricks.hub``
                both qualify).
            long_press_ms: how long the button must stay pressed before
                the toggle fires. Default 1000 ms.
            poll_ms: polling period. Default 50 ms (20 Hz) — well under
                human reaction time, negligible CPU.
            timer_id: ``machine.Timer`` ID. Default -1 uses the virtual
                software timer; use 0..3 for ESP32 hardware timers if
                you need to reserve -1 for something else.
        """
        self._button = button
        self._long_press_ms = int(long_press_ms)
        self._poll_ms       = int(poll_ms)
        self._timer_id      = timer_id
        self._timer         = None

        # Press-detection state.
        self._pressed_at    = 0       # ticks_ms when press began (0 = not pressing)
        self._triggered     = False   # True after the toggle has fired this hold

    # ---- lifecycle ----

    def start(self):
        """Begin polling. Safe to call repeatedly — the second call is a no-op."""
        if self._timer is not None:
            return
        self._timer = Timer(self._timer_id)
        self._timer.init(
            period=self._poll_ms,
            mode=Timer.PERIODIC,
            callback=self._on_tick,
        )

    def stop(self):
        """Stop polling and release the timer."""
        if self._timer is None:
            return
        self._timer.deinit()
        self._timer = None
        self._pressed_at = 0
        self._triggered  = False

    # ---- tick body ----

    def _on_tick(self, _timer):
        now = time.ticks_ms()
        if self._button.pressed():
            if self._pressed_at == 0:
                self._pressed_at = now
            elif (not self._triggered and
                  time.ticks_diff(now, self._pressed_at) >= self._long_press_ms):
                # Hold crossed the threshold — fire exactly once per hold.
                self._triggered = True
                self._fire()
        else:
            # Release resets both the timer and the latch, so the next
            # hold can fire again.
            self._pressed_at = 0
            self._triggered  = False

    def _fire(self):
        # Imported inside the method so tests that don't install the BLE
        # fake don't explode at module-load time. In production both
        # imports succeed because the firmware freezes the module in.
        from openbricks import bluetooth
        bluetooth.toggle()
