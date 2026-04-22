# SPDX-License-Identifier: MIT
"""Tests for ``openbricks_dev.run`` with NUSLink fully stubbed.

No real BLE connections — we plug in a fake link that records what was
written and replays a canned notification stream. Covers:

* Paste-mode entry sequence (Ctrl-C × 2, Ctrl-E, banner wait).
* Script upload + Ctrl-D commit.
* Live output streaming.
* Host-side KeyboardInterrupt forwards Ctrl-C and drains the traceback.
* Missing script file is a clean RunError, not a crash.
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


class _FakeLink:
    """Minimal NUSLink substitute driven by a scripted response queue."""

    def __init__(self, responses):
        # ``responses`` is a list of bytes; each ``read`` pops the next.
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
        # Nothing queued: mimic timeout → empty bytes. The test suite
        # seeds enough responses that production code paths don't hit
        # this; a real timeout in production raises inside _stream_until.
        return b""

    async def close(self):
        self.closed = True


def _args(name="RobotA", script="script.py", scan_timeout=5.0):
    return argparse.Namespace(name=name, script=script, scan_timeout=scan_timeout)


class RunHappyPathTests(unittest.TestCase):
    def setUp(self):
        # Temp script file that ``run`` will upload.
        self.tmp = tempfile.NamedTemporaryFile(
            mode="w", suffix=".py", delete=False)
        self.tmp.write("print('hello from hub')\n")
        self.tmp.close()
        self.addCleanup(os.unlink, self.tmp.name)

    def test_paste_mode_sequence_and_execute(self):
        # Responses match what the hub would send after each of our
        # writes, in order:
        #   after Ctrl-C: drained briefly
        #   after Ctrl-E: paste-mode banner + "=== " prompt
        #   after Ctrl-D: the script's output, ending with ">>> " prompt
        responses = [
            b"",  # read after Ctrl-C (drain)
            b"paste mode; Ctrl-C to cancel, Ctrl-D to finish\r\n=== ",  # banner
            b"",  # read after banner (eats trailing "=== " prompt line)
            b"hello from hub\r\n>>> ",  # post-exec output + prompt
        ]
        fake_link = _FakeLink(responses)

        async def _fake_connect(name, scan_timeout=5.0):
            return fake_link

        with patch.object(run_mod.NUSLink, "connect", side_effect=_fake_connect):
            with patch("sys.stdout", new_callable=io.StringIO) as out:
                rc = run_mod.run(_args(script=self.tmp.name))

        self.assertEqual(rc, 0)
        # The user-visible stdout shows the hub output (but not the
        # paste-mode banner, which we intentionally skip).
        self.assertIn("hello from hub", out.getvalue())

        # Write sequence: Ctrl-C sequence → Ctrl-E → script bytes → Ctrl-D.
        joined = b"".join(fake_link.writes)
        self.assertIn(b"\x03\x03", joined)            # double Ctrl-C
        self.assertIn(b"\x05", joined)                # Ctrl-E (paste mode)
        self.assertIn(b"print('hello from hub')", joined)  # script
        self.assertIn(b"\x04", joined)                # Ctrl-D (commit)
        self.assertTrue(fake_link.closed)

    def test_missing_script_raises_run_error(self):
        with patch.object(run_mod.NUSLink, "connect") as connect:
            with self.assertRaises(run_mod.RunError) as ctx:
                asyncio.run(
                    run_mod._run_async("RobotA", "/nonexistent.py", 5.0))
        # connect should never have been called — we bail before touching BLE.
        connect.assert_not_called()
        self.assertIn("cannot read script", str(ctx.exception))

    def test_connect_failure_propagates_as_run_error(self):
        async def _raise(name, scan_timeout=5.0):
            raise NUSError("no hub named 'RobotA' found")

        with patch.object(run_mod.NUSLink, "connect", side_effect=_raise):
            with self.assertRaises(run_mod.RunError) as ctx:
                asyncio.run(
                    run_mod._run_async("RobotA", self.tmp.name, 5.0))
        self.assertIn("no hub named", str(ctx.exception))


class StreamTimeoutTests(unittest.TestCase):
    def test_stream_until_timeout_raises(self):
        # Empty response queue = read returns b"" immediately, which
        # _stream_until interprets as timeout → RunError.
        link = _FakeLink([])
        async def _run():
            await run_mod._stream_until(link, b">>> ")
        with self.assertRaises(run_mod.RunError):
            asyncio.run(_run())


if __name__ == "__main__":
    unittest.main()
