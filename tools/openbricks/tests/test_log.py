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
        self.assertIs(args["out"], sys.stdout)

    def test_connect_failure_raises_log_error(self):
        async def _raise(name, scan_timeout=5.0):
            raise NUSError("not found")
        with patch.object(log_mod.NUSLink, "connect", side_effect=_raise):
            with self.assertRaises(log_mod.LogError):
                log_mod.run(self._args(name="Ghost"))


if __name__ == "__main__":
    unittest.main()
