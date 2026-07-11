# SPDX-License-Identifier: MIT
"""Tests for the line-alignment demo's pure logic
(examples/line_align.py::align_on_line)."""

import sys
import unittest

# Same import arrangement as tests/test_color_array.py: the example
# lives in examples/, resolved via a relative sys.path entry that
# works under both CPython and tests/run.py. line_align has no
# module-level hardware imports.
if "examples" not in sys.path:
    sys.path.insert(0, "examples")

import line_align  # noqa: E402


class _FakeMotor:
    """Records every call into a shared event log so tests can assert
    cross-motor ordering, not just per-motor state."""

    def __init__(self, name, log):
        self._name = name
        self._log = log

    def run_speed(self, dps):
        self._log.append((self._name, "run_speed", dps))

    def brake(self):
        self._log.append((self._name, "brake"))


def _on_line_after(n_polls, poll_counter):
    """Sensor closure: sees the line from the ``n_polls``-th check
    (0-based on the shared poll counter list)."""
    def check():
        return poll_counter[0] >= n_polls
    return check


class AlignOnLineTests(unittest.TestCase):
    def _run(self, left_at, right_at, timeout_ms=100, poll_ms=10):
        """Drive align_on_line with sensors that trigger at the given
        poll indices (None = never). Returns the event log."""
        log = []
        left = _FakeMotor("L", log)
        right = _FakeMotor("R", log)
        polls = [0]

        def wait_ms(_ms):
            polls[0] += 1

        never = 10 ** 9
        line_align.align_on_line(
            left, right,
            _on_line_after(left_at if left_at is not None else never, polls),
            _on_line_after(right_at if right_at is not None else never, polls),
            approach_dps=100, poll_ms=poll_ms, timeout_ms=timeout_ms,
            wait_ms=wait_ms)
        return log

    def test_both_start_forward_at_approach_speed(self):
        log = self._run(left_at=0, right_at=0)
        self.assertEqual(log[0], ("L", "run_speed", 100))
        self.assertEqual(log[1], ("R", "run_speed", 100))

    def test_left_first_brakes_left_then_right(self):
        log = self._run(left_at=0, right_at=3)
        brakes = [e for e in log if e[1] == "brake"]
        self.assertEqual(brakes, [("L", "brake"), ("R", "brake")])

    def test_right_first_brakes_right_then_left(self):
        log = self._run(left_at=3, right_at=0)
        brakes = [e for e in log if e[1] == "brake"]
        self.assertEqual(brakes, [("R", "brake"), ("L", "brake")])

    def test_each_wheel_brakes_exactly_once(self):
        log = self._run(left_at=1, right_at=4)
        self.assertEqual(
            len([e for e in log if e == ("L", "brake")]), 1)
        self.assertEqual(
            len([e for e in log if e == ("R", "brake")]), 1)

    def test_simultaneous_detection(self):
        log = self._run(left_at=2, right_at=2)
        brakes = [e for e in log if e[1] == "brake"]
        self.assertEqual(len(brakes), 2)

    def test_timeout_raises_and_brakes_both(self):
        log = []
        left = _FakeMotor("L", log)
        right = _FakeMotor("R", log)
        try:
            line_align.align_on_line(
                left, right,
                lambda: False, lambda: False,
                poll_ms=10, timeout_ms=50, wait_ms=lambda _ms: None)
        except RuntimeError:
            pass
        else:
            self.fail("expected RuntimeError on timeout")
        brakes = sorted(e[0] for e in log if e[1] == "brake")
        self.assertEqual(brakes, ["L", "R"],
                         "both wheels must brake on timeout")

    def test_timeout_brakes_only_the_wheel_still_running(self):
        # Left found the line; right never does. On timeout the left
        # wheel must not get a second brake.
        log = []
        left = _FakeMotor("L", log)
        right = _FakeMotor("R", log)
        try:
            line_align.align_on_line(
                left, right,
                lambda: True, lambda: False,
                poll_ms=10, timeout_ms=50, wait_ms=lambda _ms: None)
        except RuntimeError:
            pass
        else:
            self.fail("expected RuntimeError on timeout")
        self.assertEqual(
            len([e for e in log if e == ("L", "brake")]), 1)
        self.assertEqual(
            len([e for e in log if e == ("R", "brake")]), 1)


if __name__ == "__main__":
    unittest.main()
