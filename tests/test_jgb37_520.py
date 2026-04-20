# SPDX-License-Identifier: MIT
"""Tests for the JGB37-520 closed-loop motor driver.

The driver is a thin Python wrapper over ``_openbricks_native.Servo``
(either the C implementation on firmware, or the Python fake installed
by ``tests/_fakes.py`` on desktop). Behavioural tests therefore exercise
the Servo type itself via the wrapper.
"""

import tests._fakes  # noqa: F401

import time
import unittest

from machine import Timer  # type: ignore[import-not-found]

from openbricks._native import motor_process
from openbricks.drivers.jgb37_520 import JGB37Motor


def _make_motor(**overrides):
    kwargs = dict(
        in1=1, in2=2, pwm=3,
        encoder_a=4, encoder_b=5,
        counts_per_output_rev=1320,
    )
    kwargs.update(overrides)
    return JGB37Motor(**kwargs)


class TestJGB37Motor(unittest.TestCase):
    def setUp(self):
        motor_process.reset()
        Timer.reset_for_test()

    # --- open-loop behaviour ---

    def test_run_passes_through_to_h_bridge(self):
        m = _make_motor()
        m.run(50)
        self.assertEqual(m._in1.value(), 1)
        self.assertEqual(m._in2.value(), 0)
        self.assertEqual(m._pwm.duty(), 511)  # 50% of 1023

    def test_run_reverse(self):
        m = _make_motor()
        m.run(-75)
        self.assertEqual(m._in1.value(), 0)
        self.assertEqual(m._in2.value(), 1)

    def test_brake_shorts_both_terminals(self):
        m = _make_motor()
        m.brake()
        self.assertEqual(m._in1.value(), 1)
        self.assertEqual(m._in2.value(), 1)
        self.assertEqual(m._pwm.duty(), 1023)

    def test_coast_floats_terminals(self):
        m = _make_motor()
        m.run(100)
        m.coast()
        self.assertEqual(m._in1.value(), 0)
        self.assertEqual(m._in2.value(), 0)
        self.assertEqual(m._pwm.duty(), 0)

    def test_power_clamped(self):
        m = _make_motor()
        m.run(9999)
        self.assertLessEqual(m._pwm.duty(), 1023)

    def test_invert_swaps_direction(self):
        m = _make_motor(invert=True)
        m.run(50)
        self.assertEqual(m._in1.value(), 0)
        self.assertEqual(m._in2.value(), 1)

    # --- encoder + angle ---

    def test_angle_from_encoder_count(self):
        m = _make_motor()
        m._enc._count = 660   # half a rev at 1320 cpr
        self.assertAlmostEqual(m.angle(), 180.0, places=6)

    def test_reset_angle_sets_encoder_count(self):
        m = _make_motor()
        m.reset_angle(90)
        self.assertEqual(m._enc._count, 330)
        m.reset_angle(0)
        self.assertEqual(m._enc._count, 0)

    # --- closed-loop: run_speed attaches, brake/coast/run detach ---

    def test_run_speed_attaches_and_auto_starts_scheduler(self):
        m = _make_motor()
        m.run_speed(100)
        self.assertTrue(motor_process.is_running())

    def test_brake_after_run_speed_brakes_bridge(self):
        m = _make_motor()
        m.run_speed(300)
        m.brake()
        self.assertEqual(m._in1.value(), 1)
        self.assertEqual(m._in2.value(), 1)

    def test_coast_after_run_speed_releases_bridge(self):
        m = _make_motor()
        m.run_speed(300)
        m.coast()
        self.assertEqual(m._in1.value(), 0)
        self.assertEqual(m._in2.value(), 0)
        self.assertEqual(m._pwm.duty(), 0)

    def test_scheduler_drives_bridge_at_1khz(self):
        """At the default 1 ms period, one sleep_ms(1) yields one tick of
        control. With target=300, measured=0, feedforward+P saturates at
        +100% -> full-forward H-bridge."""
        m = _make_motor()
        m.run_speed(300)
        time.sleep_ms(1)
        self.assertEqual(m._in1.value(), 1)
        self.assertEqual(m._in2.value(), 0)
        self.assertEqual(m._pwm.duty(), 1023)
        m.brake()

    def test_run_angle_reaches_target_when_encoder_is_fed(self):
        """Simulate the encoder advancing on each tick by registering a
        Python callback that mutates the count. The C-path control step
        fires first (drives the bridge), then the Python callback fires
        (advances the encoder); run_angle's blocking loop exits once the
        angle crosses target."""
        m = _make_motor()

        def spin():
            m._enc._count += 13

        motor_process.register(spin)
        m.run_angle(300, 90)
        self.assertGreaterEqual(m.angle(), 90)

    # --- internal structure ---

    def test_wrapper_holds_a_native_servo(self):
        m = _make_motor()
        # ``_servo`` is the native handle; tests that want to inspect it
        # can reach in like this.
        self.assertIsNotNone(m._servo)


if __name__ == "__main__":
    unittest.main()
