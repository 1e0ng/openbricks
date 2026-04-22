# SPDX-License-Identifier: MIT
"""Tests for ``openbricks_dev.download``.

``download`` stages a script at ``/program.py`` on the hub. The user
launches it via the hub button; the client never triggers
``machine.reset()``. These tests verify that the upload program has
no reset call and that the staged path is the launcher's target
(``/program.py``).
"""

import argparse
import asyncio
import io
import os
import tempfile
import unittest
from unittest.mock import patch

from openbricks_dev import download as dl
from openbricks_dev._nus import NUSError


def _args(
    name="RobotA",
    script="s.py",
    path="/program.py",
    scan_timeout=5.0,
):
    return argparse.Namespace(
        name=name, script=script, path=path, scan_timeout=scan_timeout,
    )


class ComposeTests(unittest.TestCase):
    """Pure-function tests for the upload-program generator."""

    def test_writes_payload_to_given_path(self):
        prog = dl._compose_upload_program("/program.py", b"payload")
        self.assertIn(b"'/program.py'", prog)
        self.assertIn(b"open(", prog)
        self.assertIn(b"'wb'", prog)
        self.assertIn(b"f.write(", prog)

    def test_custom_path_surfaces_in_program(self):
        prog = dl._compose_upload_program("/user/alt.py", b"X")
        self.assertIn(b"'/user/alt.py'", prog)

    def test_prints_byte_count(self):
        prog = dl._compose_upload_program("/program.py", b"AB")
        self.assertIn(b"print('downloaded', 2", prog)

    def test_does_not_issue_machine_reset(self):
        """Auto-reset would run user code immediately — the launcher
        workflow requires a manual button press to start, so the
        upload program MUST NOT call machine.reset()."""
        prog = dl._compose_upload_program("/program.py", b"X")
        self.assertNotIn(b"machine.reset", prog)
        self.assertNotIn(b"import machine", prog)

    def test_payload_bytes_round_trip(self):
        # Raw bytes with NULs / high bits / quotes / newlines must
        # survive ``repr()`` wrapping and come through verbatim.
        tricky = b"\x00\xff\r\n'\"\\x41"
        prog = dl._compose_upload_program("/p", tricky)
        self.assertIn(repr(tricky).encode(), prog)


# --- end-to-end through a scripted NUS link ---

_BANNER      = b"raw REPL; CTRL-B to exit\r\n>"
_R_SUPPORTED = b"R\x01"
_WINDOW_8K   = b"\x00\x20"  # 0x2000 LE — fits the upload without mid-stream ACKs
_CTRL_D      = b"\x04"


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


class DownloadFlowTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False)
        self.tmp.write("print('hello')\n")
        self.tmp.close()
        self.addCleanup(os.unlink, self.tmp.name)

    def _standard_responses(self, stdout_msg):
        return [
            b"",                          # drain after Ctrl-C interrupt
            _BANNER,                      # raw-REPL banner
            _R_SUPPORTED + _WINDOW_8K,    # raw-paste ack + window
            _CTRL_D,                      # end-of-paste ack
            stdout_msg + _CTRL_D,         # stdout + EOT
            _CTRL_D,                      # stderr + EOT
        ]

    def test_default_flow_stages_to_program_py_without_reset(self):
        fake = _ScriptedLink(self._standard_responses(
            b"downloaded 15 bytes to '/program.py'\r\n"))

        async def _fake_connect(name, scan_timeout=5.0):
            return fake

        with patch.object(dl.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO) as out:
            rc = dl.run(_args(script=self.tmp.name))

        self.assertEqual(rc, 0)
        joined = b"".join(fake.writes)
        # No reset should be issued anywhere in the upload.
        self.assertNotIn(b"machine.reset", joined)
        self.assertNotIn(b"import machine", joined)
        # Raw REPL fully entered and cleanly exited.
        self.assertIn(b"\x01", joined)      # Ctrl-A (enter raw)
        self.assertIn(b"\x05A\x01", joined) # raw-paste request
        self.assertIn(b"\r\x02", joined)    # Ctrl-B (leave raw)
        # The upload program writes to the default /program.py path.
        self.assertIn(b"'/program.py'", joined)
        # User script bytes got embedded in the upload literal.
        self.assertIn(b"hello", joined)
        self.assertTrue(fake.closed)
        # Confirmation printed to the user's terminal.
        self.assertIn("downloaded", out.getvalue())

    def test_custom_path_flag_targets_alt_location(self):
        fake = _ScriptedLink(self._standard_responses(
            b"downloaded 15 bytes to '/main.py'\r\n"))

        async def _fake_connect(name, scan_timeout=5.0):
            return fake

        with patch.object(dl.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO):
            rc = dl.run(_args(script=self.tmp.name, path="/main.py"))

        self.assertEqual(rc, 0)
        joined = b"".join(fake.writes)
        self.assertIn(b"'/main.py'", joined)

    def test_missing_script_raises_without_touching_ble(self):
        with patch.object(dl.NUSLink, "connect") as connect:
            with self.assertRaises(dl.DownloadError) as ctx:
                asyncio.run(dl._download_async(
                    "RobotA", "/nonexistent.py", "/program.py", 5.0))
        connect.assert_not_called()
        self.assertIn("cannot read script", str(ctx.exception))

    def test_oversized_script_raises(self):
        big = tempfile.NamedTemporaryFile(mode="wb", suffix=".py", delete=False)
        big.write(b"x" * (dl._MAX_SCRIPT_BYTES + 1))
        big.close()
        self.addCleanup(os.unlink, big.name)

        with patch.object(dl.NUSLink, "connect") as connect:
            with self.assertRaises(dl.DownloadError) as ctx:
                asyncio.run(dl._download_async(
                    "RobotA", big.name, "/program.py", 5.0))
        connect.assert_not_called()
        self.assertIn("soft limit", str(ctx.exception))

    def test_connect_failure_propagates_as_download_error(self):
        async def _raise(name, scan_timeout=5.0):
            raise NUSError("no hub named 'Ghost'")

        with patch.object(dl.NUSLink, "connect", side_effect=_raise):
            with self.assertRaises(dl.DownloadError):
                asyncio.run(dl._download_async(
                    "Ghost", self.tmp.name, "/program.py", 5.0))


if __name__ == "__main__":
    unittest.main()
