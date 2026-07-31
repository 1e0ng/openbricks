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
