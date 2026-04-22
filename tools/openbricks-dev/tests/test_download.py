# SPDX-License-Identifier: MIT
"""Tests for ``openbricks_dev.download``.

We exercise the payload composition + upload-program generation with
plain function calls, and the full BLE flow through a scripted NUS
link (same fake as test_run.py) so no hardware is required.
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
    path="/main.py",
    no_reset=False,
    scan_timeout=5.0,
):
    return argparse.Namespace(
        name=name, script=script, path=path,
        no_reset=no_reset, scan_timeout=scan_timeout,
    )


class ComposeTests(unittest.TestCase):
    """Payload + upload-program generation — pure functions, no BLE."""

    def test_preamble_prepended_to_user_script(self):
        payload = dl._compose_payload(b"print('hi')\n")
        self.assertIn(b"from openbricks import bluetooth", payload)
        self.assertIn(b"apply_persisted_state()", payload)
        # User script ends up AFTER the preamble so it only runs once
        # BLE was already kicked off.
        pre_idx = payload.index(b"apply_persisted_state()")
        user_idx = payload.index(b"print('hi')")
        self.assertLess(pre_idx, user_idx)

    def test_preamble_is_exception_safe(self):
        # A broken openbricks install shouldn't bring the whole boot
        # sequence down — the preamble must be wrapped in try/except.
        payload = dl._compose_payload(b"")
        self.assertIn(b"try:", payload)
        self.assertIn(b"except Exception", payload)

    def test_upload_program_writes_to_given_path(self):
        prog = dl._compose_upload_program("/user/prog.py", b"X", reset_after=False)
        self.assertIn(b"'/user/prog.py'", prog)
        self.assertIn(b"open(", prog)
        self.assertIn(b"'wb'", prog)
        self.assertIn(b"f.write(", prog)

    def test_upload_program_prints_byte_count(self):
        prog = dl._compose_upload_program("/main.py", b"XY", reset_after=False)
        self.assertIn(b"print('downloaded', 2", prog)

    def test_upload_program_with_reset_ends_with_machine_reset(self):
        prog = dl._compose_upload_program("/main.py", b"X", reset_after=True)
        self.assertIn(b"import machine", prog)
        self.assertIn(b"machine.reset()", prog)

    def test_upload_program_without_reset_omits_machine(self):
        prog = dl._compose_upload_program("/main.py", b"X", reset_after=False)
        self.assertNotIn(b"machine.reset()", prog)

    def test_payload_round_trips_through_bytes_literal(self):
        # Raw bytes with NULs / high bits / newlines must survive being
        # baked into ``repr(...)`` and written to the hub verbatim.
        tricky = b"\x00\xff\r\n'\"\\x41"
        prog = dl._compose_upload_program("/p", tricky, reset_after=False)
        # The exact tricky bytes, wrapped in whatever quoting repr chose,
        # should appear somewhere in the generated source.
        self.assertIn(repr(tricky).encode(), prog)


# --- end-to-end with a scripted link ---

_BANNER = b"raw REPL; CTRL-B to exit\r\n>"
_R_SUPPORTED = b"R\x01"
_WINDOW_8K = b"\x00\x20"  # 0x2000 LE — big enough that the upload fits without needing mid-stream ACKs
_CTRL_D = b"\x04"


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
        self.tmp.write("print('hi')\n")
        self.tmp.close()
        self.addCleanup(os.unlink, self.tmp.name)

    def _standard_responses(self, stdout_msg):
        # After interrupt / drain / raw-REPL banner / raw-paste ack /
        # end-of-paste ack / stdout+EOT / stderr+EOT.
        return [
            b"",
            _BANNER,
            _R_SUPPORTED + _WINDOW_8K,
            _CTRL_D,
            stdout_msg + _CTRL_D,
            _CTRL_D,
        ]

    def test_no_reset_flow_writes_then_leaves_raw_repl(self):
        fake = _ScriptedLink(self._standard_responses(
            b"downloaded 73 bytes to '/main.py'\r\n"))

        async def _fake_connect(name, scan_timeout=5.0):
            return fake

        with patch.object(dl.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO) as out:
            rc = dl.run(_args(script=self.tmp.name, no_reset=True))

        self.assertEqual(rc, 0)
        self.assertIn("downloaded", out.getvalue())
        joined = b"".join(fake.writes)
        # Raw REPL entered, raw-paste request issued, leave-raw sent.
        self.assertIn(b"\x01", joined)       # Ctrl-A
        self.assertIn(b"\x05A\x01", joined)  # raw-paste request
        self.assertIn(b"\r\x02", joined)     # Ctrl-B (leave raw)
        # Upload program contains the user's print.
        self.assertIn(b"print('hi')", joined)
        self.assertTrue(fake.closed)

    def test_reset_flow_tolerates_stream_timeout(self):
        # With reset_after=True the hub disconnects mid-stream, so
        # _stream_output's read eventually raises RunError (b"" → timeout).
        # download.py should swallow that specific error path.
        fake = _ScriptedLink([
            b"",
            _BANNER,
            _R_SUPPORTED + _WINDOW_8K,
            _CTRL_D,
            # stdout starts but never terminates — simulates disconnect.
            b"downloaded 10 bytes to '/main.py'\r\n",
        ])

        async def _fake_connect(name, scan_timeout=5.0):
            return fake

        with patch.object(dl.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO):
            # Default (no --no-reset) → reset_after=True. Should return 0
            # despite the mid-stream cut-off.
            rc = dl.run(_args(script=self.tmp.name))

        self.assertEqual(rc, 0)

    def test_missing_script_raises_download_error_without_touching_ble(self):
        with patch.object(dl.NUSLink, "connect") as connect:
            with self.assertRaises(dl.DownloadError) as ctx:
                asyncio.run(dl._download_async(
                    "RobotA", "/nonexistent.py", "/main.py", False, 5.0))
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
                    "RobotA", big.name, "/main.py", False, 5.0))
        connect.assert_not_called()
        self.assertIn("soft limit", str(ctx.exception))

    def test_connect_failure_propagates_as_download_error(self):
        async def _raise(name, scan_timeout=5.0):
            raise NUSError("no hub named 'Ghost'")

        with patch.object(dl.NUSLink, "connect", side_effect=_raise):
            with self.assertRaises(dl.DownloadError):
                asyncio.run(dl._download_async(
                    "Ghost", self.tmp.name, "/main.py", False, 5.0))


if __name__ == "__main__":
    unittest.main()
