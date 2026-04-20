# SPDX-License-Identifier: MIT
"""Tests for the JGB37-520 closed-loop motor driver."""

import tests._fakes  # noqa: F401

import time
import unittest

from machine import Timer  # type: ignore[import-not-found]  # provided by fakes

from openbricks.drivers.jgb37_520 import JGB37Motor
from openbricks.tools.scheduler import MotorProcess


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
        MotorProcess.reset()
        Timer.reset_for_test()

    def test_open_loop_run_passes_through_to_h_bridge(self):
        m = _make_motor()
        m.run(50)
        self.assertEqual(m._driver._in1.value(), 1)
        self.assertEqual(m._driver._in2.value(), 0)
        self.assertGreater(m._driver._pwm.duty(), 0)

    def test_brake_and_coast_passthrough(self):
        m = _make_motor()
        m.brake()
        self.assertEqual(m._driver._in1.value(), 1)
        self.assertEqual(m._driver._in2.value(), 1)
        m.coast()
        self.assertEqual(m._driver._pwm.duty(), 0)

    def test_angle_from_encoder_count(self):
        m = _make_motor()
        # 1320 counts per output rev -> 660 counts = 180 deg.
        m._enc._count = 660
        self.assertAlmostEqual(m.angle(), 180.0, places=6)

    def test_reset_angle_sets_encoder_count(self):
        m = _make_motor()
        m.reset_angle(90)
        # 90 deg * 1320 / 360 = 330 counts.
        self.assertEqual(m._enc._count, 330)
        m.reset_angle(0)
        self.assertEqual(m._enc._count, 0)

    def test_run_speed_sets_target_but_does_not_command_bridge(self):
        m = _make_motor()
        m.run_speed(300)
        # The target is set, but without a scheduler tick the H-bridge should
        # still be idle.
        self.assertEqual(m._target_dps, 300.0)
        self.assertEqual(m._driver._in1.value(), 0)
        self.assertEqual(m._driver._in2.value(), 0)
        self.assertEqual(m._driver._pwm.duty(), 0)

    def test_control_step_positive_clamps_to_full_forward(self):
        m = _make_motor()
        m.run_speed(300)
        m._control_step()
        self.assertEqual(m._driver._in1.value(), 1)
        self.assertEqual(m._driver._in2.value(), 0)
        self.assertEqual(m._driver._pwm.duty(), 1023)

    def test_control_step_negative_clamps_to_full_reverse(self):
        m = _make_motor()
        m.run_speed(-300)
        m._control_step()
        self.assertEqual(m._driver._in1.value(), 0)
        self.assertEqual(m._driver._in2.value(), 1)
        self.assertEqual(m._driver._pwm.duty(), 1023)

    def test_control_step_zero_stops_motor(self):
        m = _make_motor()
        m.run_speed(0)
        m._control_step()
        self.assertEqual(m._driver._in1.value(), 0)
        self.assertEqual(m._driver._in2.value(), 0)
        self.assertEqual(m._driver._pwm.duty(), 0)

    def test_start_registers_control_step_with_scheduler(self):
        m = _make_motor()
        m.start()
        self.assertIn(m._control_step, MotorProcess.instance()._callbacks)
        self.assertTrue(MotorProcess.instance().is_running())

    def test_start_is_idempotent(self):
        m = _make_motor()
        m.start()
        m.start()
        # The singleton's dedupe keeps the callback list at length 1.
        self.assertEqual(
            MotorProcess.instance()._callbacks.count(m._control_step), 1,
        )

    def test_stop_unregisters_and_coasts(self):
        m = _make_motor()
        m.run_speed(300)
        m.start()
        m._control_step()  # engage the bridge
        self.assertEqual(m._driver._pwm.duty(), 1023)

        m.stop()
        self.assertNotIn(m._control_step, MotorProcess.instance()._callbacks)
        self.assertEqual(m._driver._in1.value(), 0)
        self.assertEqual(m._driver._in2.value(), 0)
        self.assertEqual(m._driver._pwm.duty(), 0)
        self.assertEqual(m._target_dps, 0.0)

    def test_scheduler_drives_bridge_during_sleep(self):
        m = _make_motor()
        m.start()
        m.run_speed(300)
        # A single tick period (10 ms) elapses -> one _control_step call.
        time.sleep_ms(10)
        self.assertEqual(m._driver._in1.value(), 1)
        self.assertEqual(m._driver._pwm.duty(), 1023)
        m.stop()

    def test_run_angle_reaches_target_when_encoder_is_fed(self):
        m = _make_motor()
        # Simulate the encoder incrementing in lock-step with the scheduler
        # by registering our own tick that advances the count. The scheduler
        # fires both callbacks each tick.
        step_count = [0]

        def fake_rotation(_t=None):
            # Advance the encoder by 13 counts per tick -> roughly
            # 13 * 360 / 1320 = 3.55 deg per tick. Over 100 ms (10 ticks)
            # the motor covers ~35.5 deg.
            m._enc._count += 13
            step_count[0] += 1

        MotorProcess.instance().register(fake_rotation)
        m.run_angle(300, 90)  # block until we reach +90 deg.
        self.assertGreaterEqual(m.angle(), 90)
        self.assertFalse(m._scheduled)  # run_angle stopped the motor


if __name__ == "__main__":
    unittest.main()
