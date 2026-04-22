# SPDX-License-Identifier: MIT
"""Tests for ``openbricks_dev.run``.

``run`` stages the user's script at ``/program.py`` (same target as
``download``) and triggers the hub-side launcher to exec it. Output
streams back live. The hub-side button-press stop shows up here as a
``KeyboardInterrupt`` that the uploaded bootstrap catches and prints.

We drive the whole flow through a scripted NUS link — no real BLE.
"""

import argparse
import asyncio
import io
import os
import tempfile
import unittest
from unittest.mock import patch

from openbricks_dev import run as run_mod
from openbricks_dev._nus import NUSError


class _ScriptedLink:
    def __init__(self, responses):
        self._responses = list(responses)
        self.writes = []
        self.closed = False

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc, tb):
        await self.close()

    async def write(self, data):
        self.writes.append(bytes(data))

    async def read(self, timeout=None):
        if self._responses:
            return self._responses.pop(0)
        return b""

    async def close(self):
        self.closed = True


def _args(name="RobotA", script="s.py", scan_timeout=5.0):
    return argparse.Namespace(
        name=name, script=script, scan_timeout=scan_timeout)


# Hub-side response shorthands (kept in sync with test_download).
_BANNER      = b"raw REPL; CTRL-B to exit\r\n>"
_R_SUPPORTED = b"R\x01"
_WINDOW_8K   = b"\x00\x20"  # 0x2000 LE window — upload fits without mid-stream ACKs
_CTRL_D      = b"\x04"


class ComposeTests(unittest.TestCase):
    """Bootstrap composition — pure functions, no BLE."""

    def test_bootstrap_writes_user_bytes_to_program_py(self):
        boot = run_mod._compose_bootstrap(b"print('hi')\n")
        self.assertIn(b"'/program.py'", boot)
        self.assertIn(b"open(", boot)
        self.assertIn(b"'wb'", boot)
        self.assertIn(b"print('hi')", boot)

    def test_bootstrap_calls_launcher_run_program(self):
        boot = run_mod._compose_bootstrap(b"x=1\n")
        self.assertIn(b"from openbricks import launcher", boot)
        self.assertIn(b"launcher.run_program(", boot)

    def test_bootstrap_catches_keyboard_interrupt(self):
        """Button-press stop raises KeyboardInterrupt through
        run_program; the bootstrap must catch it so the hub prints a
        clean stop message instead of letting the raw-REPL surface an
        interrupt traceback."""
        boot = run_mod._compose_bootstrap(b"")
        self.assertIn(b"except KeyboardInterrupt", boot)
        self.assertIn(b"stopped by button press", boot)

    def test_bootstrap_user_bytes_round_trip(self):
        # Any bytes the user script could contain — NULs, high bits,
        # quotes — must survive ``repr()`` wrapping.
        tricky = b"\x00\xff\r\n'\"\\"
        boot = run_mod._compose_bootstrap(tricky)
        self.assertIn(repr(tricky).encode(), boot)


class RunFlowTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.NamedTemporaryFile(
            mode="w", suffix=".py", delete=False)
        self.tmp.write("print('hello from hub')\n")
        self.tmp.close()
        self.addCleanup(os.unlink, self.tmp.name)

    def _standard_responses(self, stdout_msg, stderr_msg=b""):
        return [
            b"",                          # drain after Ctrl-C interrupt
            _BANNER,                      # raw-REPL banner
            _R_SUPPORTED + _WINDOW_8K,    # raw-paste ack + window
            _CTRL_D,                      # end-of-paste ack
            stdout_msg + _CTRL_D,         # stdout + EOT
            stderr_msg + _CTRL_D,         # stderr + EOT
        ]

    def test_happy_path_streams_stdout(self):
        fake = _ScriptedLink(self._standard_responses(
            b"hello from hub\r\n"))

        async def _fake_connect(name, scan_timeout=5.0):
            return fake

        with patch.object(run_mod.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO) as out:
            rc = run_mod.run(_args(script=self.tmp.name))

        self.assertEqual(rc, 0)
        self.assertIn("hello from hub", out.getvalue())
        # No paste-mode "=== " echo pollution.
        self.assertNotIn("===", out.getvalue())

        joined = b"".join(fake.writes)
        # Confirm the upload writes /program.py and calls run_program.
        self.assertIn(b"'/program.py'", joined)
        self.assertIn(b"launcher.run_program", joined)
        # Confirm the control-byte sequence of raw-paste mode.
        self.assertIn(b"\x03\x03", joined)      # Ctrl-C interrupt
        self.assertIn(b"\r\x01", joined)        # Ctrl-A (enter raw)
        self.assertIn(b"\x05A\x01", joined)     # raw-paste request
        self.assertIn(b"\r\x02", joined)        # Ctrl-B (leave raw)
        self.assertTrue(fake.closed)

    def test_button_press_stop_surfaces_as_clean_message(self):
        # The hub's bootstrap catches KeyboardInterrupt and prints a
        # clean line — we assert that line reaches stdout rather than
        # a raw traceback.
        fake = _ScriptedLink(self._standard_responses(
            b"partial output\r\nopenbricks: stopped by button press.\r\n"))

        async def _fake_connect(name, scan_timeout=5.0):
            return fake

        with patch.object(run_mod.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO) as out:
            rc = run_mod.run(_args(script=self.tmp.name))

        self.assertEqual(rc, 0)
        self.assertIn("stopped by button press", out.getvalue())
        self.assertNotIn("Traceback", out.getvalue())

    def test_user_exception_stderr_is_surfaced(self):
        fake = _ScriptedLink(self._standard_responses(
            b"",
            b"Traceback (most recent call last):\r\n  ...\r\nValueError: boom\r\n",
        ))

        async def _fake_connect(name, scan_timeout=5.0):
            return fake

        with patch.object(run_mod.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO) as out:
            rc = run_mod.run(_args(script=self.tmp.name))

        self.assertEqual(rc, 0)
        self.assertIn("ValueError: boom", out.getvalue())


class RawPasteErrorTests(unittest.TestCase):
    def test_hub_without_raw_paste_support_errors(self):
        responses = [
            b"",
            _BANNER,
            b"R\x00",  # raw-paste NOT supported
        ]
        fake = _ScriptedLink(responses)

        tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False)
        tmp.write("pass\n")
        tmp.close()
        self.addCleanup(os.unlink, tmp.name)

        async def _fake_connect(name, scan_timeout=5.0):
            return fake

        with patch.object(run_mod.NUSLink, "connect", side_effect=_fake_connect):
            with self.assertRaises(run_mod.RunError) as ctx:
                asyncio.run(run_mod._run_async("RobotA", tmp.name, 5.0))
        self.assertIn("raw-paste", str(ctx.exception))


class ErrorPathTests(unittest.TestCase):
    def test_missing_script_raises_without_touching_ble(self):
        with patch.object(run_mod.NUSLink, "connect") as connect:
            with self.assertRaises(run_mod.RunError) as ctx:
                asyncio.run(
                    run_mod._run_async("RobotA", "/nonexistent.py", 5.0))
        connect.assert_not_called()
        self.assertIn("cannot read script", str(ctx.exception))

    def test_oversized_script_raises(self):
        big = tempfile.NamedTemporaryFile(mode="wb", suffix=".py", delete=False)
        big.write(b"x" * (run_mod._MAX_SCRIPT_BYTES + 1))
        big.close()
        self.addCleanup(os.unlink, big.name)

        with patch.object(run_mod.NUSLink, "connect") as connect:
            with self.assertRaises(run_mod.RunError) as ctx:
                asyncio.run(run_mod._run_async("RobotA", big.name, 5.0))
        connect.assert_not_called()
        self.assertIn("soft limit", str(ctx.exception))

    def test_connect_failure_propagates_as_run_error(self):
        tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False)
        tmp.write("pass\n")
        tmp.close()
        self.addCleanup(os.unlink, tmp.name)

        async def _raise(name, scan_timeout=5.0):
            raise NUSError("no hub named 'RobotA' found")

        with patch.object(run_mod.NUSLink, "connect", side_effect=_raise):
            with self.assertRaises(run_mod.RunError) as ctx:
                asyncio.run(run_mod._run_async("RobotA", tmp.name, 5.0))
        self.assertIn("no hub named", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
