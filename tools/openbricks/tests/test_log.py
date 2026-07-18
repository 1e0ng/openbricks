# SPDX-License-Identifier: MIT
"""Tests for ``openbricks_dev.log`` — argparse + on-hub program shape."""

import argparse
import inspect
import sys
import unittest
from unittest.mock import patch

from openbricks_dev import cli, log as log_mod
from openbricks_dev._nus import NUSError


class ParserTests(unittest.TestCase):
    def setUp(self):
        self.parser = cli._build_parser()

    def test_log_required_args(self):
        args = self.parser.parse_args(["log", "-n", "RobotA"])
        self.assertEqual(args.command, "log")
        self.assertEqual(args.name, "RobotA")
        self.assertFalse(args.list)
        self.assertIsNone(args.run)

    def test_log_list_flag(self):
        args = self.parser.parse_args(["log", "-n", "RobotA", "--list"])
        self.assertTrue(args.list)

    def test_log_run_index(self):
        args = self.parser.parse_args(["log", "-n", "RobotA", "--run", "1"])
        self.assertEqual(args.run, 1)

    def test_log_requires_name(self):
        with self.assertRaises(SystemExit):
            with patch("sys.stderr"):
                self.parser.parse_args(["log"])


class ComposeProgramTests(unittest.TestCase):
    """The on-hub one-shot programs are pure strings — verify their
    shape without spinning up a hub. The actual transport is the
    same NUS / raw-paste path the run / download tests exercise."""

    def test_list_program_imports_log_module(self):
        prog = log_mod._compose_list_program()
        self.assertIn(b"from openbricks import log", prog)
        self.assertIn(b"_log.list_runs()", prog)

    def test_dump_program_for_specific_index_uses_read_run(self):
        prog = log_mod._compose_dump_program(1)
        self.assertIn(b"_log.read_run(1)", prog)

    def test_dump_program_for_latest_iterates_list_runs(self):
        prog = log_mod._compose_dump_program(None)
        self.assertIn(b"_log.list_runs()", prog)
        self.assertIn(b"--no log--", prog)

    def test_all_programs_sync_the_rtc(self):
        # The hub's per-line epoch stamps are only meaningful if its
        # RTC (which powers up at 2000-01-01, no NTP) gets set from
        # the host clock on every connect.
        for prog in (log_mod._compose_list_program(),
                     log_mod._compose_dump_program(None),
                     log_mod._compose_dump_program(1)):
            self.assertIn(b"machine.RTC().datetime(", prog)


class StampRendererTests(unittest.TestCase):
    """Leading int64 epoch-ms stamps become local-time brackets; the
    conversion to local time happens ONLY here, per the store-epoch/
    render-local rule."""

    class _Sink:
        def __init__(self):
            self.chunks = []
            self.flushes = 0

        def write(self, s):
            self.chunks.append(s)

        def flush(self):
            self.flushes += 1

        @property
        def text(self):
            return "".join(self.chunks)

    def _expected(self, ms):
        from datetime import datetime
        dt = datetime.fromtimestamp(ms / 1000.0)
        return "[%s.%03d]" % (dt.strftime("%Y-%m-%d %H:%M:%S"),
                              dt.microsecond // 1000)

    def test_stamped_line_rendered_in_local_time(self):
        sink = self._Sink()
        r = log_mod._StampRenderer(sink)
        r.write("1783950123456 left ambient: 33\n")
        r.drain()
        self.assertEqual(
            sink.text,
            "%s left ambient: 33\n" % self._expected(1783950123456))

    def test_unstamped_lines_pass_through(self):
        sink = self._Sink()
        r = log_mod._StampRenderer(sink)
        r.write("-- run_9 (/openbricks_logs/run_9.log) --\n")
        r.write("plain text\n")
        r.write("run_9\t123\t/openbricks_logs/run_9.log\n")
        r.drain()
        self.assertEqual(
            sink.text,
            "-- run_9 (/openbricks_logs/run_9.log) --\n"
            "plain text\n"
            "run_9\t123\t/openbricks_logs/run_9.log\n")

    def test_partial_lines_buffer_across_chunks(self):
        # BLE chunk boundaries land anywhere — including inside the
        # stamp digits. The renderer must reassemble before parsing.
        sink = self._Sink()
        r = log_mod._StampRenderer(sink)
        r.write("17839501")
        r.write("23456 hello\nnext")
        self.assertEqual(
            sink.text,
            "%s hello\n" % self._expected(1783950123456))
        r.drain()
        self.assertEqual(
            sink.text,
            "%s hello\nnext" % self._expected(1783950123456))

    def test_unsynced_hub_year_2000_stamp_still_renders(self):
        # 12-digit ms stamp = an RTC never set (2000-01-01 dates);
        # rendering it makes the unsynced state self-diagnosing.
        sink = self._Sink()
        r = log_mod._StampRenderer(sink)
        r.write("946684800000 boot print\n")
        r.drain()
        self.assertEqual(
            sink.text, "%s boot print\n" % self._expected(946684800000))

    def test_short_number_prefix_is_not_a_stamp(self):
        sink = self._Sink()
        r = log_mod._StampRenderer(sink)
        r.write("12345 not a stamp\n")
        r.drain()
        self.assertEqual(sink.text, "12345 not a stamp\n")


class _FakeLink:
    def __init__(self):
        self.writes = []
        self.closed = False

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc, tb):
        await self.close()

    async def write(self, data):
        self.writes.append(bytes(data))

    async def read(self, timeout=None):
        return b""

    async def close(self):
        self.closed = True


class RunDispatchTests(unittest.TestCase):
    """Dispatch through ``log_mod.run`` using stubbed NUS + raw-paste
    helpers — verify the right one-shot program is uploaded based on
    the args."""

    def setUp(self):
        self.fake = _FakeLink()
        self.uploaded = []
        self.streamed = []

        async def _fake_connect(name, scan_timeout=5.0):
            return self.fake

        def _bound_stub(real, record=None, pick=None):
            # Stub that enforces the REAL helper's signature, so a
            # caller passing the wrong arity fails here the same way
            # it would on a live hub. (0.10.23 shipped `log` calling
            # _stream_output without `link` because the old stub
            # accepted 2 args.)
            sig = inspect.signature(real)
            async def _stub(*args, **kwargs):
                bound = sig.bind(*args, **kwargs)
                if record is not None:
                    record.append(bound.arguments[pick] if pick
                                  else bound.arguments)
            return _stub

        self._patches = [
            patch.object(log_mod.NUSLink, "connect", side_effect=_fake_connect),
            patch.object(log_mod.run_mod, "_enter_raw_repl",
                         _bound_stub(log_mod.run_mod._enter_raw_repl)),
            patch.object(log_mod.run_mod, "_restore_idle_loop",
                         _bound_stub(log_mod.run_mod._restore_idle_loop)),
            patch.object(log_mod.run_mod, "_raw_paste_upload",
                         _bound_stub(log_mod.run_mod._raw_paste_upload,
                                     self.uploaded, pick="script_bytes")),
            patch.object(log_mod.run_mod, "_stream_output",
                         _bound_stub(log_mod.run_mod._stream_output,
                                     self.streamed)),
        ]
        for p in self._patches:
            p.start()
            self.addCleanup(p.stop)

    def _args(self, **kwargs):
        defaults = dict(name="RobotA", list=False, run=None,
                        scan_timeout=5.0)
        defaults.update(kwargs)
        return argparse.Namespace(**defaults)

    def test_default_dumps_latest_run(self):
        rc = log_mod.run(self._args())
        self.assertEqual(rc, 0)
        self.assertEqual(len(self.uploaded), 1)
        self.assertIn(b"list_runs", self.uploaded[0])
        self.assertNotIn(b"read_run(", self.uploaded[0])

    def test_run_index_dumps_specific(self):
        log_mod.run(self._args(run=2))
        self.assertIn(b"read_run(2)", self.uploaded[0])

    def test_list_flag_uses_list_program(self):
        log_mod.run(self._args(list=True))
        self.assertIn(b"list_runs()", self.uploaded[0])
        self.assertNotIn(b"read_run", self.uploaded[0])

    def test_stream_output_called_with_link_and_stdout(self):
        # Regression for 0.10.23: `openbricks log` called
        # _stream_output(blink, out) after the helper grew a `link`
        # parameter (upload.py was updated, log.py was not), so every
        # log invocation died with a TypeError before reading a byte.
        log_mod.run(self._args(list=True))
        self.assertEqual(len(self.streamed), 1)
        args = self.streamed[0]
        self.assertIs(args["link"], self.fake)
        # 0.11.0 wraps stdout in the timestamp renderer.
        self.assertIsInstance(args["out"], log_mod._StampRenderer)
        self.assertIs(args["out"]._out, sys.stdout)

    def test_raising_idle_restore_is_swallowed(self):
        # _restore_idle_loop failing during teardown must not turn a
        # successful log dump into an error.
        async def _bad_restore(link):
            raise RuntimeError("hub hung up first")
        with patch.object(log_mod.run_mod, "_restore_idle_loop",
                          _bad_restore):
            rc = log_mod.run(self._args(list=True))
        self.assertEqual(rc, 0)

    def test_connect_failure_raises_log_error(self):
        async def _raise(name, scan_timeout=5.0):
            raise NUSError("not found")
        with patch.object(log_mod.NUSLink, "connect", side_effect=_raise):
            with self.assertRaises(log_mod.LogError):
                log_mod.run(self._args(name="Ghost"))




class LogInterruptAndRendererFlushTests(unittest.TestCase):
    def test_ctrl_c_maps_to_130(self):
        orig = log_mod._log_async

        def _boom(*a, **k):
            raise KeyboardInterrupt()
        log_mod._log_async = _boom
        try:
            rc = log_mod.run(argparse.Namespace(
                name="X", list=True, run=None, scan_timeout=1.0))
        finally:
            log_mod._log_async = orig
        self.assertEqual(rc, 130)

    def test_renderer_flush_passes_through(self):
        class _Sink:
            def __init__(self):
                self.flushes = 0

            def write(self, s):
                pass

            def flush(self):
                self.flushes += 1
        sink = _Sink()
        r = log_mod._StampRenderer(sink)
        r.flush()
        self.assertEqual(sink.flushes, 1)


def setUpModule():
    # The verified idle-restore waits real seconds for the hub banner;
    # against scripted silent links that's pure sleep. Shrink for the
    # whole module, restore after.
    import openbricks_dev.run as _rm
    global _ORIG_RESTORE_WAIT
    _ORIG_RESTORE_WAIT = _rm._RESTORE_WAIT_S
    _rm._RESTORE_WAIT_S = 0.02


def tearDownModule():
    import openbricks_dev.run as _rm
    _rm._RESTORE_WAIT_S = _ORIG_RESTORE_WAIT


if __name__ == "__main__":
    unittest.main()
