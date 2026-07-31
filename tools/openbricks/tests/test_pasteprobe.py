# SPDX-License-Identifier: MIT
"""Tests for ``openbricks paste-probe`` — the raw-paste burst-limit
measurement tool.

The probe exists because two desk-reasoned window values (2048, 1024)
both broke real hardware in different ways. Its job is to
CHARACTERISE failures rather than raise on the first one, so these
tests pin exactly that: each failure mode is reported and the largest
surviving size is computed from what actually completed.
"""

import argparse
import asyncio
import io
import unittest
from unittest import mock

from openbricks_dev import pasteprobe
from openbricks_dev import run as run_mod


_CTRL_D = b"\x04"
_R = b"R\x01"
_WINDOW = b"\x00\x08"          # 2048 LE — one burst, no mid-acks
_BANNER = b"raw REPL; CTRL-B to exit\r\n>"


class PaddedProgramTests(unittest.TestCase):
    def test_size_is_respected_and_marker_present(self):
        for size in (128, 512, 4096):
            prog = pasteprobe._padded_program(size)
            self.assertLessEqual(abs(len(prog) - size), 8, size)
            self.assertIn(pasteprobe._MARKER.encode(), prog)

    def test_tiny_size_degrades_to_just_the_marker(self):
        prog = pasteprobe._padded_program(4)
        self.assertIn(pasteprobe._MARKER.encode(), prog)

    def test_padding_is_comments_only(self):
        prog = pasteprobe._padded_program(2048).decode()
        body = [l for l in prog.splitlines()[1:] if l.strip()]
        self.assertTrue(all(l.startswith("#") for l in body))
        compile(prog, "<probe>", "exec")   # must stay valid Python


class _ProbeLink:
    """Hub model whose behaviour per paste is scripted: 'ok',
    'truncated' (runs a fragment → no marker) or 'hang' (stops
    acking, exactly the 1.32.1 symptom)."""

    def __init__(self, behaviours):
        self._behaviours = list(behaviours)
        self._pending = bytearray()
        self.writes = []

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc, tb):
        return False

    def stats(self):
        return {"connected": True, "notify_count": 1, "byte_count": 1,
                "last_byte_ago": 0.1, "uptime": 1.0}

    async def write(self, data):
        self.writes.append(bytes(data))
        if data == run_mod._RAW_PASTE_REQUEST:
            self._mode = self._behaviours.pop(0) if self._behaviours else "ok"
            self._pending += _R + _WINDOW + b"\x01"
        elif data == run_mod._CTRL_D and getattr(self, "_mode", None):
            if self._mode == "hang":
                self._mode = None          # never reply again
                return
            out = (b"" if self._mode == "truncated"
                   else pasteprobe._MARKER.encode() + b"\r\n")
            self._pending += _CTRL_D + out + _CTRL_D + _CTRL_D + b">"
            self._mode = None

    async def read(self, timeout=None):
        if self._pending:
            out = bytes(self._pending)
            self._pending = bytearray()
            return out
        if getattr(self, "_mode", None) == "ok":
            # Mid-paste: a healthy hub keeps granting windows as it
            # consumes, so pastes larger than the initial grant still
            # complete. ('hang' deliberately never gets here — that
            # mode clears _mode to model a hub that stops consuming.)
            return b"\x01"
        if timeout:
            await asyncio.sleep(min(timeout, 0.05))
        return b""


def _try(behaviour, size=256, timeout=0.2):
    link = _ProbeLink([behaviour])
    blink = run_mod._BufferedLink(link)
    return asyncio.run(pasteprobe._try_size(blink, link, size, timeout))


class FailureCharacterisationTests(unittest.TestCase):
    def test_ok_paste_reports_ok(self):
        ok, detail = _try("ok")
        self.assertTrue(ok, detail)

    def test_truncated_paste_is_named_as_truncation(self):
        ok, detail = _try("truncated")
        self.assertFalse(ok)
        self.assertIn("TRUNCATED", detail)
        self.assertIn("fragment", detail)

    def test_hung_paste_is_named_as_a_hang_not_a_crash(self):
        # The 1.32.1 symptom: hub stops consuming, no ack ever comes.
        ok, detail = _try("hang")
        self.assertFalse(ok)
        self.assertIn("HUNG", detail)


class ProbeSweepTests(unittest.TestCase):
    def _sweep(self, behaviours):
        link = _ProbeLink(behaviours)

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return link

        buf = io.StringIO()
        with mock.patch.object(pasteprobe.NUSLink, "connect", _fake_connect), \
             mock.patch.object(run_mod, "_enter_raw_repl",
                               new=mock.AsyncMock(return_value=None)), \
             mock.patch.object(run_mod, "_restore_idle_loop",
                               new=mock.AsyncMock(return_value=None)), \
             mock.patch("sys.stdout", buf):
            rc = pasteprobe.run(argparse.Namespace(
                name="ls", scan_timeout=1.0, max=512, timeout=0.2))
        return rc, buf.getvalue()

    def test_reports_largest_surviving_size_and_stops_at_first_failure(self):
        # 128 ok, 256 ok, 512 hangs → largest = 256, and the sweep
        # stops (the link is desynced after a failure).
        rc, out = self._sweep(["ok", "ok", "hang"])
        self.assertEqual(rc, 0)
        self.assertIn("largest paste that completed = 256", out)
        self.assertIn("MICROPY_REPL_STDIN_BUFFER_MAX <= 256", out)

    def test_all_failing_says_the_path_is_broken_not_size_limited(self):
        rc, out = self._sweep(["truncated"])
        self.assertEqual(rc, 0)
        self.assertIn("even the smallest paste failed", out)


if __name__ == "__main__":
    unittest.main()


class RemainingBranchTests(unittest.TestCase):
    """The error branches the happy/failure sweeps don't reach — each
    is a real hub condition the probe must name precisely, since its
    whole value is telling the operator WHICH failure they hit."""

    def test_asyncio_timeout_is_also_reported_as_a_hang(self):
        # The other way a stall can surface (wait_for firing before
        # the link's own read timeout).
        with mock.patch.object(run_mod, "_raw_paste_upload",
                               side_effect=asyncio.TimeoutError):
            ok, detail = _try("ok")
        self.assertFalse(ok)
        self.assertIn("HUNG", detail)

    def test_non_timeout_protocol_error_is_passed_through(self):
        with mock.patch.object(
                run_mod, "_raw_paste_upload",
                side_effect=run_mod.RunError("hub aborted the upload")):
            ok, detail = _try("ok")
        self.assertFalse(ok)
        self.assertIn("protocol error", detail)
        self.assertIn("aborted", detail)

    def test_incomplete_reply_after_paste_is_named(self):
        link = _ProbeLink(["ok"])
        blink = run_mod._BufferedLink(link)
        with mock.patch.object(run_mod, "_raw_paste_upload",
                               new=mock.AsyncMock(return_value=None)), \
             mock.patch.object(
                run_mod._BufferedLink, "read_until",
                new=mock.AsyncMock(side_effect=asyncio.TimeoutError)):
            ok, detail = asyncio.run(
                pasteprobe._try_size(blink, link, 256, 0.2))
        self.assertFalse(ok)
        self.assertIn("no complete reply", detail)

    def test_framing_desync_is_named(self):
        # Paste itself succeeds; the REPLY framing is wrong. Patch the
        # upload out so the handshake reads aren't intercepted too.
        link = _ProbeLink(["ok"])
        blink = run_mod._BufferedLink(link)
        with mock.patch.object(run_mod, "_raw_paste_upload",
                               new=mock.AsyncMock(return_value=None)), \
             mock.patch.object(run_mod._BufferedLink, "read_until",
                               new=mock.AsyncMock(return_value=b"")), \
             mock.patch.object(run_mod._BufferedLink, "read_exact",
                               new=mock.AsyncMock(return_value=b"?")):
            ok, detail = asyncio.run(
                pasteprobe._try_size(blink, link, 256, 0.2))
        self.assertFalse(ok)
        self.assertIn("framing desync", detail)

    def test_hub_exception_surfaces_its_last_line(self):
        link = _ProbeLink(["ok"])
        blink = run_mod._BufferedLink(link)
        calls = []

        async def _read_until(self, delim, timeout=30.0):
            calls.append(1)
            return b"" if len(calls) == 1 else b"MemoryError: alloc failed"

        with mock.patch.object(run_mod, "_raw_paste_upload",
                               new=mock.AsyncMock(return_value=None)), \
             mock.patch.object(run_mod._BufferedLink, "read_until",
                               new=_read_until), \
             mock.patch.object(run_mod._BufferedLink, "read_exact",
                               new=mock.AsyncMock(return_value=b">")):
            ok, detail = asyncio.run(
                pasteprobe._try_size(blink, link, 256, 0.2))
        self.assertFalse(ok)
        self.assertIn("hub raised", detail)
        self.assertIn("MemoryError", detail)

    def test_connect_failure_becomes_a_probe_error(self):
        async def _boom(name, scan_timeout=5.0, debug=False):
            raise pasteprobe.NUSError("no hub named 'ls'")
        with mock.patch.object(pasteprobe.NUSLink, "connect", _boom):
            with self.assertRaises(pasteprobe.PasteProbeError):
                asyncio.run(pasteprobe._probe_async("ls", 1.0, 512, 0.2))

    def test_max_size_stops_the_sweep_early(self):
        link = _ProbeLink(["ok"] * 10)

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return link

        buf = io.StringIO()
        with mock.patch.object(pasteprobe.NUSLink, "connect", _fake_connect), \
             mock.patch.object(run_mod, "_enter_raw_repl",
                               new=mock.AsyncMock(return_value=None)), \
             mock.patch.object(run_mod, "_restore_idle_loop",
                               new=mock.AsyncMock(return_value=None)), \
             mock.patch("sys.stdout", buf):
            rc = pasteprobe.run(argparse.Namespace(
                name="ls", scan_timeout=1.0, max=256, timeout=0.2))
        out = buf.getvalue()
        self.assertEqual(rc, 0)
        self.assertIn("largest paste that completed = 256", out)
        self.assertNotIn("512 bytes", out)      # --max respected

    def test_keyboard_interrupt_exits_130(self):
        with mock.patch.object(pasteprobe, "_probe_async",
                               side_effect=KeyboardInterrupt):
            rc = pasteprobe.run(argparse.Namespace(
                name="ls", scan_timeout=1.0, max=256, timeout=0.2))
        self.assertEqual(rc, 130)

    def test_every_size_passing_exhausts_the_sweep(self):
        # The loop-completes-normally path: a hub that survives all
        # of _SIZES (what a fixed firmware should look like).
        link = _ProbeLink(["ok"] * (len(pasteprobe._SIZES) + 2))

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return link

        buf = io.StringIO()
        with mock.patch.object(pasteprobe.NUSLink, "connect", _fake_connect), \
             mock.patch.object(run_mod, "_enter_raw_repl",
                               new=mock.AsyncMock(return_value=None)), \
             mock.patch.object(run_mod, "_restore_idle_loop",
                               new=mock.AsyncMock(return_value=None)), \
             mock.patch("sys.stdout", buf):
            rc = pasteprobe.run(argparse.Namespace(
                name="ls", scan_timeout=1.0, max=1 << 20, timeout=0.2))
        out = buf.getvalue()
        self.assertEqual(rc, 0)
        self.assertIn("largest paste that completed = %d"
                      % pasteprobe._SIZES[-1], out)
