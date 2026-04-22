# SPDX-License-Identifier: MIT
"""Tests for ``openbricks_dev.run`` — raw-paste mode over a stubbed NUS link.

No real BLE: the fake link replies to each write with a scripted
sequence of hub-side bytes that matches the raw-paste protocol
(Ctrl-A banner → R\\x01 ack → window size → flow-control ACKs → Ctrl-D
framing for stdout/stderr).

Covers:

* Raw-REPL entry (Ctrl-C×2, Ctrl-A, banner).
* Raw-paste handshake (``\\x05A\\x01`` → ``R\\x01`` + window).
* Script upload respects the window size.
* Live stdout streaming to the terminal with no paste-mode echo.
* Stderr (exception block) is surfaced after stdout.
* Missing script file → clean ``RunError`` without touching BLE.
* Connect failure propagates as ``RunError``.
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
    """Fake NUSLink that replays a pre-seeded response sequence.

    Each ``read`` pops the next queued bytes; empty queue returns ``b""``
    which ``_BufferedLink._fill`` maps to a ``RunError`` — useful for
    the timeout test. Tests must seed every byte the hub would produce
    during the protocol flow.
    """

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


def _args(name="RobotA", script="script.py", scan_timeout=5.0):
    return argparse.Namespace(name=name, script=script, scan_timeout=scan_timeout)


# Shorthands for bytes the hub sends back.
_BANNER = b"raw REPL; CTRL-B to exit\r\n>"
_R_SUPPORTED = b"R\x01"
_WINDOW_256 = b"\x00\x01"     # little-endian 256
_CTRL_D = b"\x04"


class _Fixture:
    """Build a script file and a scripted link for a happy-path run."""

    def __init__(self, script_text, stdout_bytes, stderr_bytes=b""):
        self.tmp = tempfile.NamedTemporaryFile(
            mode="w", suffix=".py", delete=False)
        self.tmp.write(script_text)
        self.tmp.close()
        # Response sequence the hub would emit, in order:
        #   1. after Ctrl-C×2 interrupt + drain: anything (empty is fine)
        #   2. after Ctrl-A: the raw-REPL banner
        #   3. after \x05A\x01: "R\x01" then window-size bytes
        #   4. after Ctrl-D end-of-data: Ctrl-D ack
        #   5. script output ending with Ctrl-D
        #   6. stderr ending with Ctrl-D
        self.responses = [
            b"",                            # drain after Ctrl-C
            _BANNER,                        # raw-REPL banner
            _R_SUPPORTED + _WINDOW_256,     # raw-paste ack + window
            _CTRL_D,                        # end-of-paste ack
            stdout_bytes + _CTRL_D,         # stdout + EOT
            stderr_bytes + _CTRL_D,         # stderr + EOT
        ]


class RunHappyPathTests(unittest.TestCase):
    def setUp(self):
        self._tmpfiles = []

    def tearDown(self):
        for p in self._tmpfiles:
            try:
                os.unlink(p)
            except OSError:
                pass

    def _fixture(self, script, stdout, stderr=b""):
        fx = _Fixture(script, stdout, stderr)
        self._tmpfiles.append(fx.tmp.name)
        return fx

    def test_happy_path_streams_stdout_without_echo(self):
        fx = self._fixture("print('hello from hub')\n", b"hello from hub\r\n")
        fake = _ScriptedLink(fx.responses)

        async def _fake_connect(name, scan_timeout=5.0):
            return fake

        with patch.object(run_mod.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO) as out:
            rc = run_mod.run(_args(script=fx.tmp.name))

        self.assertEqual(rc, 0)
        # The hub's stdout appears in the user's terminal…
        self.assertIn("hello from hub", out.getvalue())
        # …and the raw-paste protocol does NOT echo the script back with
        # a "=== " paste-mode prefix.
        self.assertNotIn("===", out.getvalue())
        self.assertNotIn("print('hello from hub')", out.getvalue())

    def test_stderr_is_surfaced_after_stdout(self):
        fx = self._fixture(
            "raise RuntimeError('boom')\n",
            b"",
            b"Traceback (most recent call last):\r\n  ...\r\nRuntimeError: boom\r\n",
        )
        fake = _ScriptedLink(fx.responses)

        async def _fake_connect(name, scan_timeout=5.0):
            return fake

        with patch.object(run_mod.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO) as out:
            rc = run_mod.run(_args(script=fx.tmp.name))

        self.assertEqual(rc, 0)
        self.assertIn("Traceback", out.getvalue())
        self.assertIn("RuntimeError: boom", out.getvalue())

    def test_control_byte_sequence(self):
        fx = self._fixture("print(1)\n", b"1\r\n")
        fake = _ScriptedLink(fx.responses)

        async def _fake_connect(name, scan_timeout=5.0):
            return fake

        with patch.object(run_mod.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO):
            run_mod.run(_args(script=fx.tmp.name))

        joined = b"".join(fake.writes)
        # Interrupt sequence: Ctrl-C Ctrl-C.
        self.assertIn(b"\x03\x03", joined)
        # Enter raw REPL.
        self.assertIn(b"\r\x01", joined)
        # Raw-paste request.
        self.assertIn(b"\x05A\x01", joined)
        # Script bytes got sent.
        self.assertIn(b"print(1)", joined)
        # End-of-data (Ctrl-D) after the script; appears multiple times
        # (end of paste + leave-raw-repl) so just check it's there.
        self.assertIn(b"\x04", joined)
        # Leave raw REPL (\r Ctrl-B) on teardown.
        self.assertIn(b"\r\x02", joined)
        self.assertTrue(fake.closed)


class RawPasteErrorTests(unittest.TestCase):
    def test_hub_without_raw_paste_support_errors(self):
        # Hub replies "R\x00" (old raw REPL only) → we refuse rather
        # than silently falling back, because the bridge is our own
        # firmware and should always support raw-paste.
        responses = [
            b"",                            # drain
            _BANNER,                        # raw-REPL banner
            b"R\x00",                       # NOT supported
        ]
        fake = _ScriptedLink(responses)

        tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False)
        tmp.write("print('x')\n")
        tmp.close()
        self.addCleanup(os.unlink, tmp.name)

        async def _fake_connect(name, scan_timeout=5.0):
            return fake

        with patch.object(run_mod.NUSLink, "connect", side_effect=_fake_connect):
            with self.assertRaises(run_mod.RunError) as ctx:
                asyncio.run(run_mod._run_async("RobotA", tmp.name, 5.0))
        self.assertIn("raw-paste", str(ctx.exception))


class ErrorPathTests(unittest.TestCase):
    def test_missing_script_raises_run_error_without_touching_ble(self):
        with patch.object(run_mod.NUSLink, "connect") as connect:
            with self.assertRaises(run_mod.RunError) as ctx:
                asyncio.run(
                    run_mod._run_async("RobotA", "/nonexistent.py", 5.0))
        connect.assert_not_called()
        self.assertIn("cannot read script", str(ctx.exception))

    def test_connect_failure_propagates_as_run_error(self):
        tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False)
        tmp.write("print(1)\n")
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
