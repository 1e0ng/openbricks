# SPDX-License-Identifier: MIT
"""The hub-side servo re-ID example's decision law, arithmetic only.

Same extract-and-exec trick as ``tests/test_qtr_align.py``: the
example wires the UART at module level, so the pure resolve/refuse
block is pulled out by its markers. The contract mirrors the
``openbricks servo-id`` CLI tool: never guess between servos, never
create a duplicate ID."""

import tests._fakes  # noqa: F401

import unittest


def _load():
    with open("examples/servo_set_id.py") as f:
        src = f.read()
    begin = "# --- control law"
    end = "# --- end control law ---"
    if begin not in src or end not in src:
        raise AssertionError("control-law markers not found - they "
                             "are load-bearing here")
    ns = {}
    exec(src[src.index(begin):src.index(end)], ns)
    return ns


class ResolveTargetTests(unittest.TestCase):
    def setUp(self):
        self.resolve = _load()["resolve_target"]

    def test_single_servo_needs_no_old_id(self):
        self.assertEqual(self.resolve([1], None, 3), 1)

    def test_multiple_servos_refuse_without_old_id(self):
        with self.assertRaises(ValueError):
            self.resolve([1, 2], None, 3)

    def test_old_id_selects_among_multiple(self):
        self.assertEqual(self.resolve([1, 2], 2, 3), 2)

    def test_empty_bus_refuses(self):
        with self.assertRaises(ValueError):
            self.resolve([], None, 3)

    def test_absent_old_id_refuses(self):
        with self.assertRaises(ValueError):
            self.resolve([1, 2], 5, 3)

    def test_taken_new_id_refuses(self):
        # Two servos sharing an ID is the unrecoverable state the
        # whole contract exists to prevent.
        with self.assertRaises(ValueError):
            self.resolve([1, 2], 1, 2)

    def test_same_id_is_a_noop_error(self):
        with self.assertRaises(ValueError):
            self.resolve([1], 1, 1)

    def test_broadcast_id_rejected(self):
        with self.assertRaises(ValueError):
            self.resolve([1], None, 254)


if __name__ == "__main__":
    unittest.main()
