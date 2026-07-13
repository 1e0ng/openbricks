# SPDX-License-Identifier: MIT
"""Unit tests for the line-follow example's control law.

The example wires real hardware at module level (the user-preferred
example style), so it can't be imported here. Instead the pure
control-law block is extracted by its markers and exec'd — the same
plain string scanning test_example_pins uses (works under both
CPython and unix MicroPython; no ``ast``, no ``re``).
"""

import tests._fakes  # noqa: F401

import unittest


_here = __file__
_idx = _here.rfind("/")
_EXAMPLE = (_here[:_idx] if _idx >= 0 else ".") + "/../examples/line_follow.py"

_BEGIN = "# --- control law"
_END = "# --- end control law ---"


def _load_control_law():
    with open(_EXAMPLE) as f:
        src = f.read()
    start = src.index(_BEGIN)
    end = src.index(_END)
    ns = {}
    exec(src[start:end], ns)
    return ns


class ControlLawTests(unittest.TestCase):
    DT = 0.01

    def setUp(self):
        self.ns = _load_control_law()
        self.pid = self.ns["_pid_wheel_speeds"]
        self.state0 = self.ns["PID_STATE0"]
        self.cruise = self.ns["CRUISE_DPS"]
        self.kp = self.ns["KP"]
        self.thr = self.ns["LINE_AMBIENT"]
        self.max_dps = self.ns["MAX_DPS"]

    def _step(self, l, r, state=None, dt=None):
        return self.pid(l, r, state or self.state0,
                        self.DT if dt is None else dt)

    def test_centred_drives_straight(self):
        speeds, _ = self._step(90, 90)
        self.assertEqual(speeds, (self.cruise, self.cruise))

    def test_steady_error_settles_to_pure_p(self):
        # Same error twice: derivative is zero on the second tick, so
        # the output is P (+ the clamped-off I at KI=0) exactly.
        _, st = self._step(80, 90)
        speeds, _ = self._step(80, 90, state=st)
        err = 80 - 90
        self.assertEqual(speeds, (int(self.cruise + self.kp * err),
                                  int(self.cruise - self.kp * err)))

    def test_derivative_damps_a_growing_error(self):
        # Error growing (drifting away): D adds to the correction
        # compared with the steady-state P-only output.
        _, st = self._step(85, 90)
        growing, _ = self._step(80, 90, state=st)
        _, st2 = self._step(80, 90)
        steady, _ = self._step(80, 90, state=st2)
        self.assertTrue(
            abs(growing[0] - growing[1]) > abs(steady[0] - steady[1]),
            "derivative must strengthen the correction while the "
            "error grows")

    def test_first_tick_has_no_derivative_kick(self):
        # prev_error starts as None: the first tick is pure P even
        # for a large error.
        speeds, _ = self._step(80, 90)
        err = 80 - 90
        self.assertEqual(speeds, (int(self.cruise + self.kp * err),
                                  int(self.cruise - self.kp * err)))

    def test_integral_accumulates_when_enabled(self):
        # KI large enough that per-tick growth survives the int()
        # quantisation of the wheel speeds.
        self.ns["KI"] = 100.0
        try:
            st = self.state0
            outputs = []
            for _ in range(3):
                speeds, st = self._step(85, 90, state=st)
                outputs.append(speeds[1] - speeds[0])
            self.assertTrue(outputs[2] > outputs[1] > outputs[0],
                            outputs)
        finally:
            self.ns["KI"] = 0.0

    def test_integral_windup_is_clamped(self):
        limit = self.ns["INTEGRAL_LIMIT"]
        st = self.state0
        for _ in range(10000):
            _, st = self._step(60, 90, state=st)
        self.assertTrue(abs(st[0]) <= limit + 1e-9, st)

    def test_both_dark_stops(self):
        decision, _ = self._step(self.thr - 1, self.thr - 1)
        self.assertIsNone(decision)

    def test_branch_is_ignored_and_resets_derivative_history(self):
        # One dark sensor: hold course; prev_error resets so the tick
        # after the branch can't derivative-kick.
        _, st = self._step(80, 90)              # normal steering
        speeds, st = self._step(self.thr - 1, 90, state=st)
        self.assertEqual(speeds, (self.cruise, self.cruise))
        self.assertIsNone(st[1])
        speeds, _ = self._step(70, 90, state=st)  # resume: pure P
        err = 70 - 90
        self.assertEqual(speeds, (int(self.cruise + self.kp * err),
                                  int(self.cruise - self.kp * err)))

    def test_right_branch_mirrors(self):
        speeds, _ = self._step(90, self.thr - 1)
        self.assertEqual(speeds, (self.cruise, self.cruise))

    def test_zero_dt_is_safe(self):
        speeds, st = self._step(80, 90, dt=0.0)
        self.assertTrue(speeds is not None)
        err = 80 - 90
        self.assertEqual(speeds, (int(self.cruise + self.kp * err),
                                  int(self.cruise - self.kp * err)))

    def test_clamp_never_reverses_never_exceeds_cap(self):
        for la in range(0, 101, 10):
            for ra in range(0, 101, 10):
                decision, _ = self._step(la, ra)
                if decision is None:
                    continue
                self.assertTrue(0 <= decision[0] <= self.max_dps, decision)
                self.assertTrue(0 <= decision[1] <= self.max_dps, decision)

    def test_shipped_defaults(self):
        # KI ships 0 (enable only for persistent arc drift), KD ships
        # small, DEBUG ships off, and the sensors run at the 2.4 ms
        # minimum integration (the whole point: D needs fresh data).
        self.assertEqual(self.ns["KI"], 0.0)
        self.assertTrue(0 < self.ns["KD"] <= 0.1)
        self.assertFalse(self.ns["DEBUG"])
        with open(_EXAMPLE) as f:
            src = f.read()
        self.assertEqual(src.count("gain=16, integration_ms=2.4"), 2)


if __name__ == "__main__":
    unittest.main()
