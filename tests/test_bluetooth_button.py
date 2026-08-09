# SPDX-License-Identifier: MIT
"""Tests for openbricks.bluetooth_button — short-press toggles BLE."""

import tests._fakes        # noqa: F401
import tests._fakes_ble    # noqa: F401  (fake esp32 + bluetooth modules)

import time
import unittest

from machine import Timer

from tests._fakes_ble import _FakeBLE, _FakeNVS
from openbricks.bluetooth_button import BluetoothToggleButton


class _StubButton:
    """Tiny Button stand-in — tests flip ``pressed_value`` between ticks."""

    def __init__(self):
        self.pressed_value = False

    def pressed(self):
        return self.pressed_value


def _plant_hub_name():
    # ``bluetooth.toggle()`` activates BLE when flipping "off → on" and
    # that path needs a flashed hub name. Plant one in NVS so the
    # toggle succeeds in tests.
    import openbricks
    _FakeNVS(openbricks._HUB_NAME_NVS_NAMESPACE).set_blob(
        openbricks._HUB_NAME_NVS_KEY, b"TestHub")


class TimerIdDefaultTests(unittest.TestCase):
    """Pin the default ``timer_id`` to a hardware-valid value on
    esp32-s3 (0..3). The previous default ``-1`` (virtual timer)
    works on older MicroPython but raises ``ValueError: invalid
    Timer number`` on the v1.27+ MP we vendor — main.py bricked
    at boot in 1.0.8 because of this."""

    def setUp(self):
        _FakeNVS._reset_for_test()
        _FakeBLE._reset_for_test()
        Timer.reset_for_test()
        _plant_hub_name()

    def test_default_timer_id_is_hardware_valid(self):
        # Construct a BluetoothToggleButton with default timer_id and
        # read the stored value. ``inspect.signature`` isn't available
        # on the MicroPython unix port that runs the firmware test
        # job, so use call-site capture instead.
        from openbricks.bluetooth_button import BluetoothToggleButton
        helper = BluetoothToggleButton(_StubButton())
        # ESP32-S3 hardware timers are 0..3; -1 (virtual) raises
        # ``ValueError: invalid Timer number`` on the MP we vendor.
        self.assertGreaterEqual(helper._timer_id, 0)
        self.assertLessEqual(helper._timer_id, 3)

    def test_default_timer_id_doesnt_collide_with_launcher(self):
        # The launcher takes timer 0 (per its default in launcher.run);
        # bluetooth_button must pick a different ID so the two timers
        # don't conflict at runtime when both are wired.
        from openbricks.bluetooth_button import BluetoothToggleButton
        helper = BluetoothToggleButton(_StubButton())
        # Match-up to launcher's default 0 — must be different.
        self.assertNotEqual(helper._timer_id, 0,
                            "bluetooth_button must use a Timer ID "
                            "different from the launcher's (0)")


class BluetoothToggleButtonTests(unittest.TestCase):
    def setUp(self):
        _FakeNVS._reset_for_test()
        _FakeBLE._reset_for_test()
        Timer.reset_for_test()
        _plant_hub_name()

    def test_press_and_release_fires_toggle_once(self):
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, poll_ms=50)
        helper.start()

        btn.pressed_value = True
        time.sleep_ms(200)         # any press duration
        btn.pressed_value = False
        time.sleep_ms(100)         # release edge registered

        # Exactly one toggle: default True → False.
        self.assertEqual(_FakeNVS._STORE["openbricks"]["ble_enabled"], 0)
        self.assertFalse(_FakeBLE().active())

    def test_holding_button_does_not_re_fire(self):
        # While the button stays pressed, no toggle fires — we only
        # act on the release edge.
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, poll_ms=50)
        helper.start()

        btn.pressed_value = True
        time.sleep_ms(5000)        # held for ages

        # Nothing toggled yet.
        self.assertFalse("ble_enabled" in _FakeNVS._STORE.get("openbricks", {}))
        self.assertFalse(_FakeBLE().active())

        # Release now fires exactly one toggle.
        btn.pressed_value = False
        time.sleep_ms(100)
        self.assertEqual(_FakeNVS._STORE["openbricks"]["ble_enabled"], 0)

    def test_release_and_second_press_fires_again(self):
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, poll_ms=50)
        helper.start()

        # 1st press-release: True → False.
        btn.pressed_value = True
        time.sleep_ms(200)
        btn.pressed_value = False
        time.sleep_ms(100)
        self.assertFalse(_FakeBLE().active())

        # 2nd press-release: False → True.
        btn.pressed_value = True
        time.sleep_ms(200)
        btn.pressed_value = False
        time.sleep_ms(100)
        self.assertTrue(_FakeBLE().active())

    # ---- lifecycle ----

    def test_stop_halts_polling(self):
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, poll_ms=50)
        helper.start()
        helper.stop()

        btn.pressed_value = True
        time.sleep_ms(500)
        btn.pressed_value = False
        time.sleep_ms(100)

        # Nothing fired: the Timer is gone.
        self.assertFalse(_FakeBLE().active())
        self.assertFalse("ble_enabled" in _FakeNVS._STORE.get("openbricks", {}))

    def test_double_start_is_idempotent(self):
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, poll_ms=50)
        helper.start()
        helper.start()   # second call should not stack a second Timer

        btn.pressed_value = True
        time.sleep_ms(200)
        btn.pressed_value = False
        time.sleep_ms(100)

        # Exactly one toggle, not two.
        self.assertEqual(_FakeNVS._STORE["openbricks"]["ble_enabled"], 0)


class _RecordingLED:
    """``StatusLED``-like stub that records every ``rgb()`` call."""

    def __init__(self):
        self.last_rgb = None
        self.calls = []

    def rgb(self, r, g, b):
        self.last_rgb = (r, g, b)
        self.calls.append((r, g, b))

    def off(self):
        self.calls.append("off")


class BluetoothToggleButtonLEDTests(unittest.TestCase):
    def setUp(self):
        from openbricks import bluetooth
        from openbricks import launcher
        _FakeNVS._reset_for_test()
        _FakeBLE._reset_for_test()
        Timer.reset_for_test()
        del bluetooth._state_listeners[:]
        # The run indicator's default probe reads the launcher
        # singleton — clear any instance leaked by other test files
        # so no test here sees a phantom "program running".
        launcher._singleton = None
        self.addCleanup(setattr, launcher, "_singleton", None)
        _plant_hub_name()

    def test_programmatic_set_enabled_repaints_the_led(self):
        # The LED must follow BLE state changed WITHOUT the button —
        # e.g. user code or a tool calling bluetooth.set_enabled().
        # Before the state-listener fix only a button press repainted.
        from openbricks import bluetooth
        led = _RecordingLED()
        helper = BluetoothToggleButton(_StubButton(), led=led, poll_ms=50)
        helper.start()   # paints blue (default state on)
        self.assertEqual(led.last_rgb, (0, 0, 255))

        bluetooth.set_enabled(False)
        self.assertEqual(led.last_rgb, (255, 200, 0),
                         "LED must turn yellow when BLE is disabled "
                         "programmatically")

        bluetooth.set_enabled(True)
        self.assertEqual(led.last_rgb, (0, 0, 255),
                         "LED must turn blue when BLE is enabled "
                         "programmatically")

    def test_stop_unregisters_the_led_listener(self):
        from openbricks import bluetooth
        led = _RecordingLED()
        helper = BluetoothToggleButton(_StubButton(), led=led, poll_ms=50)
        helper.start()
        helper.stop()
        led.last_rgb = None
        bluetooth.set_enabled(False)
        self.assertIsNone(led.last_rgb,
                          "stopped toggle must not keep repainting")

    def test_repeated_start_stop_does_not_leak_listeners(self):
        from openbricks import bluetooth
        helper = BluetoothToggleButton(_StubButton(), led=_RecordingLED(),
                                       poll_ms=50)
        for _ in range(3):
            helper.start()
            helper.stop()
        self.assertEqual(len(bluetooth._state_listeners), 0)

    def test_start_paints_blue_when_ble_enabled(self):
        btn = _StubButton()
        led = _RecordingLED()
        # Default state (never written) is enabled.
        BluetoothToggleButton(btn, led=led, poll_ms=50).start()
        self.assertEqual(led.last_rgb, (0, 0, 255))

    def test_start_paints_yellow_when_ble_disabled(self):
        btn = _StubButton()
        led = _RecordingLED()
        # Pre-persist an "off" state.
        from openbricks import bluetooth
        bluetooth.set_enabled(False)
        BluetoothToggleButton(btn, led=led, poll_ms=50).start()
        self.assertEqual(led.last_rgb, (255, 200, 0))

    def test_toggle_recolors_the_led(self):
        btn = _StubButton()
        led = _RecordingLED()
        helper = BluetoothToggleButton(btn, led=led, poll_ms=50)
        helper.start()  # starts blue (on)

        # Press-release → toggles to off → LED should flip to yellow.
        btn.pressed_value = True
        time.sleep_ms(200)
        btn.pressed_value = False
        time.sleep_ms(100)

        self.assertEqual(led.last_rgb, (255, 200, 0))

        # A second press-release flips back to on → blue.
        btn.pressed_value = True
        time.sleep_ms(200)
        btn.pressed_value = False
        time.sleep_ms(100)

        self.assertEqual(led.last_rgb, (0, 0, 255))

    def test_no_led_is_fine(self):
        """With led=None, no LED calls should be attempted; toggle still fires."""
        btn = _StubButton()
        helper = BluetoothToggleButton(btn, led=None, poll_ms=50)
        helper.start()

        btn.pressed_value = True
        time.sleep_ms(200)
        btn.pressed_value = False
        time.sleep_ms(100)

        self.assertFalse(_FakeBLE().active())

    def test_custom_colors(self):
        btn = _StubButton()
        led = _RecordingLED()
        BluetoothToggleButton(
            btn, led=led, poll_ms=50,
            color_on=(0, 255, 0), color_off=(255, 0, 0),
        ).start()
        # Default state = on → custom green.
        self.assertEqual(led.last_rgb, (0, 255, 0))


class _IndicatorLED:
    """Records every visible LED transition, in order:
    ``("rgb", (r, g, b))`` for colour paints, ``"off"`` for off."""

    def __init__(self):
        self.history = []

    def rgb(self, r, g, b):
        self.history.append(("rgb", (r, g, b)))

    def off(self):
        self.history.append("off")

    def last(self):
        return self.history[-1] if self.history else None


class _PlainIndicatorLED:
    """Single-colour LED shape (``SimpleLED``): on/off only, ``rgb``
    raises ``NotImplementedError``."""

    def __init__(self):
        self.history = []

    def rgb(self, r, g, b):
        raise NotImplementedError

    def on(self):
        self.history.append("on")

    def off(self):
        self.history.append("off")

    def last(self):
        return self.history[-1] if self.history else None


_BLUE = ("rgb", (0, 0, 255))
_YELLOW = ("rgb", (255, 200, 0))


class RunIndicatorTests(unittest.TestCase):
    """While a user program runs the LED flashes the BLE-state colour
    at 2 Hz (250 ms lit / 250 ms dark); when the program stops it
    returns to the solid idle presentation.

    Timing model: poll_ms=50 and RUN_BLINK_MS=250 → phase flips every
    5 ticks. The virtual clock fires ticks on 50 ms boundaries, so
    cumulative sleeps of 100 / 350 / 600 ms after raising the run
    flag land deterministically in lit / dark / lit phases."""

    def setUp(self):
        from openbricks import bluetooth
        from openbricks import launcher
        _FakeNVS._reset_for_test()
        _FakeBLE._reset_for_test()
        Timer.reset_for_test()
        del bluetooth._state_listeners[:]
        launcher._singleton = None
        self.addCleanup(setattr, launcher, "_singleton", None)
        _plant_hub_name()

    def _running_helper(self, led):
        """Helper wired to an injected run flag: ``flag[0]`` drives
        the indicator (closure-captured list — MP closures have no
        attribute assignment)."""
        flag = [False]
        helper = BluetoothToggleButton(
            _StubButton(), led=led, poll_ms=50,
            program_running=lambda: flag[0])
        helper.start()
        return helper, flag

    def test_run_blinks_blue_when_ble_on(self):
        led = _IndicatorLED()
        helper, flag = self._running_helper(led)
        self.assertEqual(led.last(), _BLUE)   # idle paint

        flag[0] = True
        time.sleep_ms(100)                    # run seen → lit phase
        self.assertEqual(led.last(), _BLUE)
        time.sleep_ms(250)                    # 350 ms → dark phase
        self.assertEqual(led.last(), "off")
        time.sleep_ms(250)                    # 600 ms → lit again
        self.assertEqual(led.last(), _BLUE)

    def test_run_blinks_yellow_when_ble_off(self):
        from openbricks import bluetooth
        led = _IndicatorLED()
        helper, flag = self._running_helper(led)
        bluetooth.set_enabled(False)          # idle repaint → yellow

        flag[0] = True
        time.sleep_ms(100)
        self.assertEqual(led.last(), _YELLOW)
        time.sleep_ms(250)
        self.assertEqual(led.last(), "off")
        time.sleep_ms(250)
        self.assertEqual(led.last(), _YELLOW)

    def test_ble_toggle_mid_run_switches_blink_color(self):
        from openbricks import bluetooth
        led = _IndicatorLED()
        helper, flag = self._running_helper(led)

        flag[0] = True
        time.sleep_ms(100)
        self.assertEqual(led.last(), _BLUE)
        # Turn BLE off mid-run: the next lit phase must be yellow.
        bluetooth.set_enabled(False)
        time.sleep_ms(250)                    # dark phase
        self.assertEqual(led.last(), "off")
        time.sleep_ms(250)                    # next lit phase
        self.assertEqual(led.last(), _YELLOW)

    def test_program_end_restores_solid_state_color(self):
        led = _IndicatorLED()
        helper, flag = self._running_helper(led)

        flag[0] = True
        time.sleep_ms(350)                    # land in the dark phase
        self.assertEqual(led.last(), "off")

        flag[0] = False
        time.sleep_ms(100)                    # next tick restores idle
        self.assertEqual(led.last(), _BLUE)

        # Idle again: no further blinking.
        n = len(led.history)
        time.sleep_ms(1200)
        self.assertEqual(len(led.history), n,
                         "LED must hold solid once the program stops")

    def test_plain_led_blinks_on_off_and_idles_dark(self):
        led = _PlainIndicatorLED()
        helper, flag = self._running_helper(led)
        # Idle paint no-ops on single-colour LEDs.
        self.assertEqual(led.history, [])

        flag[0] = True
        time.sleep_ms(100)
        self.assertEqual(led.last(), "on")
        time.sleep_ms(250)
        self.assertEqual(led.last(), "off")
        time.sleep_ms(250)                    # lit again
        self.assertEqual(led.last(), "on")

        flag[0] = False
        time.sleep_ms(100)                    # idle = dark for plain LEDs
        self.assertEqual(led.last(), "off")

    def test_no_led_running_is_harmless(self):
        flag = [False]
        helper = BluetoothToggleButton(
            _StubButton(), led=None, poll_ms=50,
            program_running=lambda: flag[0])
        helper.start()
        flag[0] = True
        time.sleep_ms(700)
        flag[0] = False
        time.sleep_ms(100)    # nothing raised, nothing to assert

    def test_button_still_toggles_while_running(self):
        # The tick restructure (blink rides every tick) must not break
        # press-release detection mid-run.
        led = _IndicatorLED()
        btn = _StubButton()
        flag = [True]
        helper = BluetoothToggleButton(
            btn, led=led, poll_ms=50,
            program_running=lambda: flag[0])
        helper.start()
        time.sleep_ms(100)

        btn.pressed_value = True
        time.sleep_ms(200)
        btn.pressed_value = False
        time.sleep_ms(100)
        self.assertEqual(_FakeNVS._STORE["openbricks"]["ble_enabled"], 0)

    def test_default_probe_reads_the_launcher_flag(self):
        # No injected probe: the indicator follows
        # launcher.program_running() (the real wiring on firmware).
        from openbricks import launcher

        class _FakeLauncher:
            _running = False

        inst = _FakeLauncher()
        launcher._singleton = inst
        led = _IndicatorLED()
        helper = BluetoothToggleButton(_StubButton(), led=led, poll_ms=50)
        helper.start()

        inst._running = True
        time.sleep_ms(350)
        self.assertEqual(led.last(), "off",
                         "default probe must see the launcher flag")
        inst._running = False
        time.sleep_ms(100)
        self.assertEqual(led.last(), _BLUE)


_RED = ("rgb", (255, 0, 0))
_GREEN = ("rgb", (0, 255, 0))


class PressFlashTests(unittest.TestCase):
    """Every program-button press flashes the LED for PRESS_FLASH_MS
    — red for a start press, green for a stop press — then the
    normal presentation resumes: solid state colour at idle, the
    2 Hz blink mid-run."""

    def setUp(self):
        from openbricks import bluetooth
        from openbricks import launcher
        _FakeNVS._reset_for_test()
        _FakeBLE._reset_for_test()
        Timer.reset_for_test()
        del bluetooth._state_listeners[:]
        launcher._singleton = None
        self.addCleanup(setattr, launcher, "_singleton", None)
        _plant_hub_name()

    def _helper(self, led):
        flag = [False]
        helper = BluetoothToggleButton(
            _StubButton(), led=led, poll_ms=50,
            program_running=lambda: flag[0])
        helper.start()
        return helper, flag

    def test_idle_press_flashes_red_then_restores_state_color(self):
        from openbricks import bluetooth_button
        led = _IndicatorLED()
        helper, _flag = self._helper(led)
        self.assertEqual(led.last(), _BLUE)      # idle paint
        bluetooth_button.notify_press()
        time.sleep_ms(100)                       # next tick renders red
        self.assertEqual(led.last(), _RED)
        time.sleep_ms(300)                       # window over → idle
        self.assertEqual(led.last(), _BLUE)

    def test_stop_press_flashes_green(self):
        from openbricks import bluetooth_button
        led = _IndicatorLED()
        helper, _flag = self._helper(led)
        bluetooth_button.notify_press(stop=True)
        time.sleep_ms(100)
        self.assertEqual(led.last(), _GREEN)
        time.sleep_ms(300)
        self.assertEqual(led.last(), _BLUE)

    def test_mid_run_stop_press_overrides_blink_then_blink_resumes(self):
        from openbricks import bluetooth_button
        led = _IndicatorLED()
        helper, flag = self._helper(led)
        flag[0] = True
        time.sleep_ms(100)                       # lit phase
        self.assertEqual(led.last(), _BLUE)
        bluetooth_button.notify_press(stop=True)
        time.sleep_ms(100)
        self.assertEqual(led.last(), _GREEN)
        time.sleep_ms(300)                       # flash over, re-enter lit
        self.assertEqual(led.last(), _BLUE)
        time.sleep_ms(300)                       # and it still blinks
        self.assertEqual(led.last(), "off")

    def test_two_presses_extend_the_flash(self):
        from openbricks import bluetooth_button
        led = _IndicatorLED()
        helper, _flag = self._helper(led)
        bluetooth_button.notify_press(stop=True)
        time.sleep_ms(100)
        bluetooth_button.notify_press(stop=True)  # fire + teardown echo
        time.sleep_ms(150)                       # still inside window two
        self.assertEqual(led.last(), _GREEN)
        time.sleep_ms(300)
        self.assertEqual(led.last(), _BLUE)

    def test_plain_led_flashes_on_then_dark(self):
        from openbricks import bluetooth_button
        led = _PlainIndicatorLED()
        helper, _flag = self._helper(led)
        self.assertEqual(led.history, [])        # idle no-op
        bluetooth_button.notify_press()
        time.sleep_ms(100)
        self.assertEqual(led.last(), "on")
        time.sleep_ms(300)                       # idle on plain = dark
        self.assertEqual(led.last(), "off")

    def test_no_led_press_is_harmless(self):
        from openbricks import bluetooth_button
        helper, _flag = self._helper(None)
        bluetooth_button.notify_press()
        time.sleep_ms(200)                       # nothing raised

    def test_launcher_helper_feeds_the_counter(self):
        from openbricks import bluetooth_button
        from openbricks import launcher
        before = bluetooth_button._press_events
        launcher._notify_press_feedback()
        self.assertEqual(bluetooth_button._press_events, before + 1)
        self.assertEqual(bluetooth_button._press_color,
                         bluetooth_button.PRESS_COLOR_START)
        launcher._notify_press_feedback(stop=True)
        self.assertEqual(bluetooth_button._press_events, before + 2)
        self.assertEqual(bluetooth_button._press_color,
                         bluetooth_button.PRESS_COLOR_STOP)


class StopInterruptRelayTests(unittest.TestCase):
    """Same relay contract as launcher._tick: a hard-button stop
    interrupt landing in this poll callback is re-posted, not eaten."""

    def setUp(self):
        _FakeNVS._reset_for_test()
        _FakeBLE._reset_for_test()
        Timer.reset_for_test()
        _plant_hub_name()

    def test_interrupt_in_tick_body_is_relayed(self):
        from openbricks import launcher
        recorded = []
        orig = launcher._resignal_stop_interrupt
        launcher._resignal_stop_interrupt = lambda: recorded.append(1)
        self.addCleanup(setattr, launcher, "_resignal_stop_interrupt",
                        orig)

        class _BoomButton(_StubButton):
            def pressed(self):
                raise KeyboardInterrupt()

        helper = BluetoothToggleButton(_BoomButton(), poll_ms=50)
        helper._on_tick(None)      # must not raise
        self.assertEqual(len(recorded), 1)


if __name__ == "__main__":
    unittest.main()
