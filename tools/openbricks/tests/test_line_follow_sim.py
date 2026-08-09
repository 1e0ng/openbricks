# SPDX-License-Identifier: MIT
"""The line-follow control law, driven in physics.

``examples/line_follow.py`` wires hardware at module level (the
user-preferred example style), so it cannot be imported. The pure
control-law block is extracted by its markers and exec'd — the same
trick ``tests/test_line_follow.py`` uses on the firmware side. That
test checks the law's arithmetic; this one puts the law, the shim
drivers, the drivebase and MuJoCo together and asks whether the
robot actually follows a line.

What this covers, and what it does not:

* IT COVERS the control law, the sensor plumbing (two cameras, mux
  channel binding), ``DriveBase.move_wheels`` reaching the wheels,
  and the intersection stop. A regression in any of those shows up
  here on every PR instead of on the bench.
* IT DOES NOT cover the serial bus. ``_SimStBus`` emulates the
  st_bus SURFACE rather than reproducing it, so the bus-contention,
  slot-assignment and program-boundary failures that dominated
  1.56-1.59 cannot appear in simulation by construction. A green
  run here means "the following logic still works", not "it will
  run on the robot".
"""

import pathlib
import unittest

from openbricks_sim.robot import SimRobot
from openbricks_sim import shim


# tests/ -> tools/openbricks/ -> tools/ -> repo root
_EXAMPLE = (pathlib.Path(__file__).resolve().parents[3]
            / "examples" / "line_follow.py")
_BEGIN = "# --- control law"
_END = "# --- end control law ---"


def _load_control_law():
    src = _EXAMPLE.read_text()
    if _BEGIN not in src or _END not in src:
        raise AssertionError(
            "control-law markers %r / %r not found in %s — they are "
            "load-bearing for this suite AND tests/test_line_follow.py"
            % (_BEGIN, _END, _EXAMPLE))
    ns = {}
    exec(src[src.index(_BEGIN):src.index(_END)], ns)
    return ns


class LineFollowInPhysicsTests(unittest.TestCase):

    def setUp(self):
        # The shim is process-global; another module may have left it
        # installed. Clearing first keeps this suite independent of
        # test ordering.
        try:
            shim.uninstall()
        except Exception:
            pass
        self.robot = SimRobot(world="practice-line")
        shim.install(self.robot.runtime)
        # From here on the shim IS installed: if anything below
        # raises, unittest skips tearDown, and the patched
        # time/machine would leak into every later suite in the
        # process. addCleanup runs regardless.
        self.addCleanup(self._uninstall)
        self.ns = _load_control_law()

    @staticmethod
    def _uninstall():
        try:
            shim.uninstall()
        except Exception:
            pass

    def tearDown(self):
        # Redundant with the addCleanup (uninstall is idempotent) —
        # kept so a reader sees the symmetry without chasing setUp.
        self._uninstall()

    def _build(self):
        """The example's own wiring, minus the module-level globals."""
        from machine import I2C, Pin
        from openbricks.drivers.st3032 import ST3032Motor
        from openbricks.drivers.tca9548a import TCA9548A
        from openbricks.drivers.tcs34725 import TCS34725
        from openbricks.robotics import DriveBase

        left_motor = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6,
                                 invert=True)
        right_motor = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        db = DriveBase(left_motor, right_motor,
                       wheel_diameter_mm=88, axle_track_mm=136)
        mux = TCA9548A(I2C(0, sda=Pin(15), scl=Pin(16), freq=400_000))
        # Channel 1 = left sensor, 0 = right — the example's mapping,
        # and what binds each shim sensor to its own camera.
        return db, TCS34725(mux[1]), TCS34725(mux[0])

    def _chassis_xy(self):
        d = self.robot.runtime.data
        return float(d.qpos[0]), float(d.qpos[1])

    def _run(self, ticks=900):
        """Drive the example's loop. Returns (stopped_at_intersection,
        max |y| deviation, distance advanced)."""
        import time as _t
        db, left_sensor, right_sensor = self._build()
        pid = self.ns["pid_wheel_speeds"]
        state = self.ns["PID_STATE0"]

        x0, y0 = self._chassis_xy()
        worst_y = 0.0
        stopped = False
        for _ in range(ticks):
            speeds, state = pid(left_sensor.ambient(),
                                right_sensor.ambient(), state, 0.010)
            if speeds is None:
                stopped = True
                break
            db.move_wheels(speeds[0], speeds[1])
            _t.sleep_ms(10)
            x, y = self._chassis_xy()
            worst_y = max(worst_y, abs(y - y0))
        db.stop()
        x, _ = self._chassis_xy()
        return stopped, worst_y, x - x0

    def test_the_robot_follows_the_line_and_stops_at_the_bar(self):
        stopped, worst_y, advanced = self._run()
        # It got somewhere: a follower that never drove would pass a
        # "stayed near the line" check trivially.
        self.assertGreater(advanced, 0.30,
                           "advanced only %.3f m" % advanced)
        # And it stayed ON the line rather than wandering off it. The
        # line is 20 mm wide; allow a generous half-chassis of drift
        # before calling it lost.
        self.assertLess(worst_y, 0.08,
                        "drifted %.3f m off the line" % worst_y)
        # The stop bar darkens both sensors: the law must report an
        # intersection rather than steering through it.
        self.assertTrue(stopped, "never detected the intersection")

    def test_a_single_dark_sensor_is_not_steered_toward(self):
        # The branch stub darkens ONE sensor. Steering toward it
        # would peel the robot off the main line; the law holds
        # course instead. Pinned here against the real geometry
        # rather than synthetic readings.
        pid = self.ns["pid_wheel_speeds"]
        cruise = self.ns["CRUISE_DPS"]
        thr = self.ns["LINE_AMBIENT"]
        speeds, _ = pid(thr - 5, thr + 40, self.ns["PID_STATE0"], 0.010)
        self.assertEqual(speeds, (cruise, cruise))


if __name__ == "__main__":
    unittest.main()
