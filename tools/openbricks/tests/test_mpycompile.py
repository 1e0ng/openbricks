# SPDX-License-Identifier: MIT
"""Host-side mpy-cross compilation — the real pinned compiler, plus
the format canary that must be revisited on MicroPython submodule
bumps."""

import tempfile
import unittest
from unittest.mock import patch

from openbricks_dev import mpycompile


class CompileSourceTests(unittest.TestCase):
    def test_valid_source_compiles_to_mpy_bytes(self):
        out = mpycompile.compile_source(b"print('x')\n", "t.py")
        self.assertTrue(out.startswith(mpycompile.MPY_HEADER_PREFIX))
        self.assertGreater(len(out), 4)

    def test_syntax_error_carries_file_and_line(self):
        # The whole feature: the compiler's message (with the display
        # name and line number) surfaces on the host in milliseconds,
        # before any BLE work.
        try:
            mpycompile.compile_source(b"def broken(:\n", "myscript.py")
            self.fail("must raise")
        except mpycompile.CompileError as e:
            msg = str(e)
            self.assertIn("SyntaxError", msg)
            self.assertIn("line 1", msg)

    def test_syntax_error_names_the_users_file_not_the_temp_path(self):
        # Field report 2026-08-17: the error said ``File ".../T/
        # tmprvqlqly3/program.py", line 54`` — mpy-cross's own
        # SyntaxError prints the temp file it read (-s only names
        # runtime tracebacks), leaving the user hunting a path that
        # no longer exists.
        src = b"x = 1\nwhile True\n    pass\n"
        try:
            mpycompile.compile_source(src, "linefollow_y.py")
            self.fail("must raise")
        except mpycompile.CompileError as e:
            msg = str(e)
            self.assertIn('File "linefollow_y.py", line 2', msg)
            self.assertNotIn("program.py", msg)
            self.assertNotIn(tempfile.gettempdir(), msg)
            # ...and quotes the offending source line.
            self.assertIn("while True", msg)

    def test_unparseable_frame_keeps_the_raw_message(self):
        # A frame whose line number can't be resolved against the
        # source (here: out of range) must not crash the rewriter —
        # the raw (path-substituted) message still surfaces.
        raw = ('Traceback (most recent call last):\n'
               '  File "/tmp/x/program.py", line 99\n'
               'SyntaxError: invalid syntax')
        msg = mpycompile._friendly_compile_message(
            raw, "/tmp/x/program.py", "short.py", b"one line\n")
        self.assertIn('File "short.py", line 99', msg)
        self.assertIn("SyntaxError", msg)

    def test_format_canary_matches_the_firmware_loader(self):
        # 'M' + format major 6 — native/micropython/py/persistentcode.h
        # MPY_VERSION. If a submodule bump changes it, this fails and
        # the pyproject mpy-cross pin must move in the same change.
        self.assertEqual(mpycompile.MPY_HEADER_PREFIX, b"M\x06")
        out = mpycompile.compile_source(b"x = 1\n", "t.py")
        self.assertEqual(out[:2], b"M\x06")

    def test_missing_mpy_cross_package_is_a_clear_error(self):
        with patch.dict("sys.modules", {"mpy_cross": None}):
            try:
                mpycompile.compile_source(b"x = 1\n", "t.py")
                self.fail("must raise")
            except mpycompile.CompileError as e:
                self.assertIn("mpy-cross package is not installed", str(e))

    def test_wrong_header_from_the_compiler_fails_on_the_host(self):
        # A compiler/firmware format divergence must die HERE with a
        # pin-the-pyproject message — not as a ValueError in the hub's
        # run log after a full upload.
        real_run = mpycompile.subprocess.run

        def _corrupting_run(cmd, **kwargs):
            proc = real_run(cmd, **kwargs)
            out_path = cmd[cmd.index("-o") + 1]
            with open(out_path, "wb") as f:
                f.write(b"M\x07corrupted")
            return proc

        with patch.object(mpycompile.subprocess, "run",
                          side_effect=_corrupting_run):
            try:
                mpycompile.compile_source(b"x = 1\n", "t.py")
                self.fail("must raise")
            except mpycompile.CompileError as e:
                self.assertIn("diverged", str(e))


if __name__ == "__main__":
    unittest.main()
