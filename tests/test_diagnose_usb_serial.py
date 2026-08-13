# SPDX-License-Identifier: MIT
"""Pure-logic tests for scripts/diagnose-usb-serial.py (verdict
classification + dump formatting). CPython-only: the script's CLI
imports are host-side."""

import os
import unittest

try:
    import importlib.util
except ImportError:            # unix MicroPython: skip module cleanly
    importlib = None


def _load():
    path = os.path.join(
        os.path.dirname(__file__), "..", "scripts",
        "diagnose-usb-serial.py")
    spec = importlib.util.spec_from_file_location("diag_usb", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class ClassifyTests(unittest.TestCase):
    def setUp(self):
        if importlib is None:
            self.skipTest("CPython-only")
        self.mod = _load()

    def _steps(self, **named):
        labels = [l for l, _, _ in self.mod.PROBE_STEPS]
        return [(l, named.get(l.split(" ")[0], b"")) for l in labels]

    def test_total_silence_is_dead(self):
        verdict = self.mod.classify(self._steps())
        self.assertIn("DEAD", verdict)

    def test_friendly_prompt_wins(self):
        steps = self._steps(newline=b"\r\n>>> ")
        self.assertIn("FRIENDLY", self.mod.classify(steps))

    def test_raw_banner_is_raw_state(self):
        steps = self._steps(raw=b"raw REPL; CTRL-B to exit\r\n>")
        self.assertIn("RAW-REPL", self.mod.classify(steps))

    def test_unsolicited_output_is_spewing(self):
        steps = self._steps(passive=b"tick 41\r\ntick 42\r\n")
        self.assertIn("SPEWING", self.mod.classify(steps))

    def test_bytes_that_match_nothing_are_odd(self):
        steps = self._steps(interrupt=b"\xfe\xfd\x00garble")
        self.assertIn("RESPONSIVE-BUT-ODD", self.mod.classify(steps))


class HexdumpTests(unittest.TestCase):
    def setUp(self):
        if importlib is None:
            self.skipTest("CPython-only")
        self.mod = _load()

    def test_printable_control_and_binary_render(self):
        self.assertEqual(
            self.mod.hexdump(b"ok\r\n\x04"), "ok\\r\\n\\x04")

    def test_truncation_is_marked(self):
        out = self.mod.hexdump(b"a" * 300, limit=256)
        self.assertTrue(out.endswith(" …(+44 bytes)"))


if __name__ == "__main__":
    unittest.main()
