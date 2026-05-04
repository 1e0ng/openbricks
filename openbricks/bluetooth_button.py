# SPDX-License-Identifier: MIT
"""
Short press on the BLE-toggle button toggles BLE on/off.

Wires a ``machine.Timer``-driven poll loop (default 50 ms) against a
``Button``-conformant object and calls ``openbricks.bluetooth.toggle()``
once per press-release cycle. State is persisted via NVS by the
``bluetooth`` module, so the new value survives reboots.

This is a different physical button from the one ``openbricks.launcher``
watches for program start/stop — the BLE toggle lives on its own
GPIO (default 5, see :class:`openbricks.hub.Hub`) while the program
button is on GPIO 4. Two pins → no duration-based dispatch, every
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


class BluetoothToggleButton:
    def __init__(self, button, led=None,
                 poll_ms=DEFAULT_POLL_MS, timer_id=1,
                 color_on=DEFAULT_COLOR_ON,
                 color_off=DEFAULT_COLOR_OFF):
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
                ESP32-S3). Default 1 stays out of the way of the
                ``launcher`` (which takes timer 0). The previous
                default ``-1`` (virtual timer) was supported by older
                MicroPython but raises ``ValueError: invalid Timer
                number`` on the v1.27+ MP we vendor.
            color_on, color_off: ``(r, g, b)`` tuples the LED is set to
                when BLE is enabled / disabled. Defaults: blue / yellow.
        """
        self._button = button
        self._led    = led
        self._poll_ms       = int(poll_ms)
        self._timer_id      = timer_id
        self._color_on      = tuple(color_on)
        self._color_off     = tuple(color_off)
        self._timer         = None

        # Edge-detection state: True from the moment we first saw the
        # button pressed until the subsequent release (when we fire).
        self._was_pressed = False

    # ---- lifecycle ----

    def start(self):
        """Begin polling. Safe to call repeatedly — the second call is a no-op.

        On first call, paints the LED (if one was provided) to reflect the
        current persisted BLE state, so the boot indicator matches reality.
        """
        if self._timer is not None:
            return
        self._apply_led_for_current_state()
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
        self._was_pressed = False

    # ---- tick body ----

    def _on_tick(self, _timer):
        if self._button.pressed():
            self._was_pressed = True
            return
        if self._was_pressed:
            # Release after a press — fire once.
            self._was_pressed = False
            self._fire()

    def _fire(self):
        # Imported inside the method so tests that don't install the BLE
        # fake don't explode at module-load time. In production both
        # imports succeed because the firmware freezes the module in.
        from openbricks import bluetooth
        bluetooth.toggle()
        self._apply_led_for_current_state()

    def _apply_led_for_current_state(self):
        """Paint the LED to match the current persisted BLE state, if an
        RGB-capable LED was provided. Silently no-ops on plain on/off
        LEDs (whose ``.rgb()`` raises ``NotImplementedError``) so the
        hub can pass ``self.led`` unconditionally without caring which
        variant it is."""
        if self._led is None:
            return
        from openbricks import bluetooth
        color = self._color_on if bluetooth.is_enabled() else self._color_off
        try:
            self._led.rgb(*color)
        except NotImplementedError:
            pass
