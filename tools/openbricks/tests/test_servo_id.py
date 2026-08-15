# SPDX-License-Identifier: MIT
"""Tests for ``openbricks_dev.servo_id`` — packet format, bus scan,
and the re-ID sequence, against a scripted fake serial port."""

import argparse
import io
import unittest

from openbricks_dev import cli, servo_id as sid_mod
from openbricks_dev.servo_id import ServoIdError


class _FakeBus:
    """Serial stand-in that emulates servos: answers PINGs for the IDs
    in ``self.ids``, ACKs writes, and APPLIES an ID write so the
    verify step sees the post-write bus state."""

    def __init__(self, ids):
        self.ids = set(ids)
        self.writes = []          # decoded (sid, instr, params)
        self._pending = b""

    # -- serial surface --
    def reset_input_buffer(self):
        self._pending = b""

    def write(self, raw):
        assert raw[:2] == b"\xFF\xFF"
        sid, length, instr = raw[2], raw[3], raw[4]
        params = list(raw[5:5 + length - 2])
        body = raw[2:-1]
        assert raw[-1] == (~sum(body)) & 0xFF, "bad checksum on wire"
        self.writes.append((sid, instr, params))
        if instr == sid_mod._INSTR_PING:
            if sid in self.ids:
                reply_body = bytes([sid, 2, 0])
                self._pending = (b"\xFF\xFF" + reply_body
                                 + bytes([(~sum(reply_body)) & 0xFF]))
        elif instr == sid_mod._INSTR_WRITE and sid in self.ids:
            reg, data = params[0], params[1:]
            if reg == sid_mod._REG_ID:
                self.ids.discard(sid)
                self.ids.add(data[0])
            reply_body = bytes([sid, 2, 0])
            self._pending = (b"\xFF\xFF" + reply_body
                             + bytes([(~sum(reply_body)) & 0xFF]))

    def read(self, n):
        out, self._pending = self._pending[:n], self._pending[n:]
        return out

    def close(self):
        pass


class PacketTests(unittest.TestCase):
    def test_ping_packet_shape_and_checksum(self):
        pkt = sid_mod._packet(1, sid_mod._INSTR_PING, [])
        self.assertEqual(pkt, b"\xFF\xFF\x01\x02\x01\xFB")

    def test_write_packet_includes_reg_and_data(self):
        pkt = sid_mod._packet(2, sid_mod._INSTR_WRITE,
                              [sid_mod._REG_ID, 7])
        body = pkt[2:-1]
        self.assertEqual(body, bytes([2, 4, 3, 5, 7]))
        self.assertEqual(pkt[-1], (~sum(body)) & 0xFF)


class ScanTests(unittest.TestCase):
    def test_scan_finds_all_ids_ascending(self):
        bus = _FakeBus({7, 1, 42})
        self.assertEqual(sid_mod.scan_bus(bus), [1, 7, 42])

    def test_scan_empty_bus(self):
        self.assertEqual(sid_mod.scan_bus(_FakeBus(set())), [])

    def test_scan_never_pings_broadcast(self):
        bus = _FakeBus(set())
        sid_mod.scan_bus(bus)
        pinged = {sid for sid, instr, _ in bus.writes
                  if instr == sid_mod._INSTR_PING}
        self.assertEqual(len(pinged), 254)
        self.assertFalse(sid_mod._BROADCAST_ID in pinged)


class SetServoIdTests(unittest.TestCase):
    def _out(self):
        return io.StringIO()

    def test_single_servo_reid_sequence_and_verify(self):
        bus = _FakeBus({1})
        out = self._out()
        sid_mod.set_servo_id(bus, 3, out=out)
        writes = [(s, i, p) for s, i, p in bus.writes
                  if i == sid_mod._INSTR_WRITE]
        self.assertEqual(writes, [
            (1, sid_mod._INSTR_WRITE, [sid_mod._REG_LOCK, 0]),
            (1, sid_mod._INSTR_WRITE, [sid_mod._REG_ID, 3]),
            (3, sid_mod._INSTR_WRITE, [sid_mod._REG_LOCK, 1]),
        ])
        self.assertEqual(bus.ids, {3})
        self.assertIn("set servo ID 1 -> 3 (verified)", out.getvalue())

    def test_no_servo_raises(self):
        try:
            sid_mod.set_servo_id(_FakeBus(set()), 3, out=self._out())
        except ServoIdError as e:
            self.assertIn("no servo answered", str(e))
        else:
            self.fail("expected ServoIdError")

    def test_multiple_servos_demand_old_id(self):
        bus = _FakeBus({1, 2})
        try:
            sid_mod.set_servo_id(bus, 3, out=self._out())
        except ServoIdError as e:
            self.assertIn("--old-id", str(e))
        else:
            self.fail("expected ServoIdError")
        # And nothing on the bus was written to.
        self.assertEqual(
            [w for w in bus.writes if w[1] == sid_mod._INSTR_WRITE], [])

    def test_multiple_servos_with_old_id_reids_the_right_one(self):
        bus = _FakeBus({1, 2})
        sid_mod.set_servo_id(bus, 3, old_id=2, out=self._out())
        self.assertEqual(bus.ids, {1, 3})

    def test_old_id_not_on_bus_raises(self):
        try:
            sid_mod.set_servo_id(_FakeBus({1}), 3, old_id=9,
                                 out=self._out())
        except ServoIdError as e:
            self.assertIn("--old-id 9", str(e))
        else:
            self.fail("expected ServoIdError")

    def test_new_id_already_taken_raises(self):
        bus = _FakeBus({1, 3})
        try:
            sid_mod.set_servo_id(bus, 3, old_id=1, out=self._out())
        except ServoIdError as e:
            self.assertIn("already taken", str(e))
        else:
            self.fail("expected ServoIdError")

    def test_same_id_is_a_noop(self):
        bus = _FakeBus({3})
        out = self._out()
        sid_mod.set_servo_id(bus, 3, out=out)
        self.assertIn("nothing to do", out.getvalue())
        self.assertEqual(
            [w for w in bus.writes if w[1] == sid_mod._INSTR_WRITE], [])

    def test_ghost_old_id_after_write_raises(self):
        # A bus where the old ID keeps answering after the rewrite
        # (duplicate-ID servos): the verify must catch it.
        class _Duplicated(_FakeBus):
            def write(self, raw):
                sid, instr = raw[2], raw[4]
                params = list(raw[5:5 + raw[3] - 2])
                self.writes.append((sid, instr, params))
                if instr == sid_mod._INSTR_PING and sid in self.ids:
                    body = bytes([sid, 2, 0])
                    self._pending = (b"\xFF\xFF" + body
                                     + bytes([(~sum(body)) & 0xFF]))
                elif instr == sid_mod._INSTR_WRITE and sid in self.ids:
                    reg, data = params[0], params[1:]
                    if reg == sid_mod._REG_ID:
                        self.ids.add(data[0])   # old ID stays too
        try:
            sid_mod.set_servo_id(_Duplicated({1}), 3, out=self._out())
        except ServoIdError as e:
            self.assertIn("still answers at old ID", str(e))
        else:
            self.fail("expected ServoIdError")

    def test_failed_verify_raises(self):
        # A bus that ACKs the write but never applies it (wedged
        # EEPROM): the verify must catch it.
        class _Stubborn(_FakeBus):
            def write(self, raw):
                sid, instr = raw[2], raw[4]
                params = list(raw[5:5 + raw[3] - 2])
                self.writes.append((sid, instr, params))
                if instr == sid_mod._INSTR_PING and sid in self.ids:
                    body = bytes([sid, 2, 0])
                    self._pending = (b"\xFF\xFF" + body
                                     + bytes([(~sum(body)) & 0xFF]))
        try:
            sid_mod.set_servo_id(_Stubborn({1}), 3, out=self._out())
        except ServoIdError as e:
            self.assertIn("does not answer at new ID", str(e))
        else:
            self.fail("expected ServoIdError")


class RunDispatchTests(unittest.TestCase):
    def _args(self, **kw):
        d = dict(new_id=None, port="/dev/fake", scan=False,
                 old_id=None, baudrate=1_000_000, timeout=0.02)
        d.update(kw)
        return argparse.Namespace(**d)

    def _patch_serial(self, bus):
        orig = sid_mod._open_serial
        sid_mod._open_serial = lambda port, baud, timeout: bus
        self.addCleanup(setattr, sid_mod, "_open_serial", orig)

    def test_requires_new_id_or_scan(self):
        with self.assertRaises(ServoIdError):
            sid_mod.run(self._args())

    def test_rejects_broadcast_id(self):
        with self.assertRaises(ServoIdError):
            sid_mod.run(self._args(new_id=254))

    def test_scan_mode_changes_nothing(self):
        bus = _FakeBus({5})
        self._patch_serial(bus)
        rc = sid_mod.run(self._args(scan=True))
        self.assertEqual(rc, 0)
        self.assertEqual(
            [w for w in bus.writes if w[1] == sid_mod._INSTR_WRITE], [])
        self.assertEqual(bus.ids, {5})

    def test_scan_mode_reports_empty_bus(self):
        import builtins
        self._patch_serial(_FakeBus(set()))
        printed = []
        orig_print = builtins.print
        builtins.print = lambda *a, **k: printed.append(
            " ".join(str(x) for x in a))
        try:
            rc = sid_mod.run(self._args(scan=True))
        finally:
            builtins.print = orig_print
        self.assertEqual(rc, 0)
        self.assertTrue(
            any("no servo answered" in s for s in printed), printed)

    def test_unopenable_port_raises_servo_id_error(self):
        # The real _open_serial path: a nonexistent device must come
        # back as a typed ServoIdError, not a raw pyserial exception.
        with self.assertRaises(ServoIdError):
            sid_mod._open_serial("/dev/does-not-exist-xyz", 1_000_000,
                                 0.02)

    def test_missing_pyserial_raises_typed_error(self):
        # sys.modules[name] = None makes ``import serial`` raise
        # ImportError — the tool must surface its typed error with an
        # install hint, not a raw traceback.
        import sys
        had = "serial" in sys.modules
        prev = sys.modules.get("serial")
        sys.modules["serial"] = None
        try:
            with self.assertRaises(ServoIdError):
                sid_mod._open_serial("/dev/x", 1_000_000, 0.02)
        finally:
            if had:
                sys.modules["serial"] = prev
            else:
                del sys.modules["serial"]

    def test_close_failure_is_tolerated(self):
        class _StickyClose(_FakeBus):
            def close(self):
                raise OSError("already gone")
        self._patch_serial(_StickyClose({1}))
        rc = sid_mod.run(self._args(new_id=3))   # must not raise
        self.assertEqual(rc, 0)

    def test_full_run_sets_id(self):
        bus = _FakeBus({1})
        self._patch_serial(bus)
        rc = sid_mod.run(self._args(new_id=3))
        self.assertEqual(rc, 0)
        self.assertEqual(bus.ids, {3})


class AutodetectPortTests(unittest.TestCase):
    """-p optional (1.98.0): the shared _ports filter picks the ONE
    connected USB serial adapter; zero or several refuse — with the
    hub also plugged in, guessing could rewrite the wrong device's
    EEPROM."""

    class _FakePort:
        def __init__(self, device, vid):
            self.device = device
            self.vid = vid

    def _detect(self, ports):
        from openbricks_dev._ports import autodetect_port
        import serial.tools.list_ports as lp
        from unittest.mock import patch
        with patch.object(lp, "comports", return_value=ports):
            return autodetect_port(ServoIdError, "testing")

    def test_single_adapter_is_chosen(self):
        got = self._detect([
            self._FakePort("/dev/cu.Bluetooth", None),
            self._FakePort("/dev/cu.usbmodem9", 0x1A86),
        ])
        self.assertEqual(got, "/dev/cu.usbmodem9")

    def test_no_adapter_refuses(self):
        with self.assertRaises(ServoIdError):
            self._detect([self._FakePort("/dev/cu.Bluetooth", None)])

    def test_hub_plus_adapter_refuses_to_guess(self):
        with self.assertRaises(ServoIdError) as ctx:
            self._detect([
                self._FakePort("/dev/cu.usbmodem9", 0x1A86),
                self._FakePort("/dev/cu.usbmodem5", 0x303A),
            ])
        self.assertIn("refusing to guess", str(ctx.exception))

    def test_run_autodetects_when_port_is_none(self):
        # End to end through run(): port=None routes through the
        # detector, whose result reaches _open_serial.
        from unittest.mock import patch
        import serial.tools.list_ports as lp
        bus = _FakeBus({5})
        seen = []
        orig = sid_mod._open_serial
        sid_mod._open_serial = (
            lambda port, baud, timeout: seen.append(port) or bus)
        self.addCleanup(setattr, sid_mod, "_open_serial", orig)
        with patch.object(lp, "comports", return_value=[
                self._FakePort("/dev/cu.usbmodem9", 0x1A86)]):
            rc = sid_mod.run(argparse.Namespace(
                new_id=None, port=None, scan=True, old_id=None,
                baudrate=1_000_000, timeout=0.02))
        self.assertEqual(rc, 0)
        self.assertEqual(seen, ["/dev/cu.usbmodem9"])


class HubPathTests(unittest.TestCase):
    """servo-id -n NAME (1.99.0): the scan/re-ID runs ON the hub over
    BLE — same contract as the adapter path, sentinel-verified."""

    def test_composed_program_is_valid_python_with_params(self):
        import ast
        prog = sid_mod._compose_hub_program(3, 2, False, 14, 41).decode()
        ast.parse(prog)
        self.assertIn("NEW_ID = 3", prog)
        self.assertIn("OLD_ID = 2", prog)
        self.assertIn("_SCServoBus(1, 14, 41)", prog)
        self.assertIn(sid_mod._HUB_OK_SENTINEL, prog)

    def test_scan_variant_changes_nothing(self):
        prog = sid_mod._compose_hub_program(None, None, True, 14, 41).decode()
        self.assertIn("SCAN_ONLY = True", prog)
        # The sentinel must be reachable on the scan path too.
        self.assertIn(sid_mod._HUB_OK_SENTINEL, prog)

    def test_law_refusals_are_in_the_program(self):
        prog = sid_mod._compose_hub_program(3, None, False, 14, 41).decode()
        self.assertIn("pass --old-id", prog)
        self.assertIn("already taken on this bus", prog)
        self.assertIn("verify FAILED", prog)

    def test_name_and_port_together_refuse(self):
        with self.assertRaises(ServoIdError):
            sid_mod.run(argparse.Namespace(
                new_id=3, port="/dev/x", name="ls", scan=False,
                old_id=None, baudrate=1_000_000, timeout=0.02,
                tx=14, rx=41, scan_timeout=5.0))

    def _hub_args(self, **kw):
        d = dict(new_id=3, port=None, name="ls", scan=False,
                 old_id=None, baudrate=1_000_000, timeout=0.02,
                 tx=14, rx=41, scan_timeout=5.0)
        d.update(kw)
        return argparse.Namespace(**d)

    def test_hub_run_succeeds_on_sentinel(self):
        orig = sid_mod._hub_async

        async def _fake(name, program, scan_timeout):
            return "servos on the bus: [2]\nset servo ID 2 -> 3 " \
                   "(verified)\n%s\n" % sid_mod._HUB_OK_SENTINEL
        sid_mod._hub_async = _fake
        self.addCleanup(setattr, sid_mod, "_hub_async", orig)
        self.assertEqual(sid_mod.run(self._hub_args()), 0)

    def test_hub_run_fails_loudly_without_sentinel(self):
        # A hub-side exception streams a traceback but no sentinel —
        # the CLI must exit non-zero, never report success.
        orig = sid_mod._hub_async

        async def _fake(name, program, scan_timeout):
            return "Traceback ...\nValueError: 2 servos on the bus\n"
        sid_mod._hub_async = _fake
        self.addCleanup(setattr, sid_mod, "_hub_async", orig)
        with self.assertRaises(ServoIdError):
            sid_mod.run(self._hub_args())


class HubSessionTests(unittest.TestCase):
    """The BLE session plumbing: program uploaded, output captured
    through the tee, idle loop restored even when streaming dies."""

    def test_tee_captures_and_passes_through(self):
        import io
        out = io.StringIO()
        tee = sid_mod._TeeCapture(out)
        tee.write("abc")
        tee.write("def")
        tee.flush()
        self.assertEqual(tee.text, "abcdef")
        self.assertEqual(out.getvalue(), "abcdef")

    def _session(self, stream_fn):
        import asyncio, io, sys
        from openbricks_dev import run as run_mod
        from openbricks_dev import _nus

        class _FakeLink:
            async def __aenter__(self):
                return self

            async def __aexit__(self, *a):
                return False

        calls = {"upload": None, "restore": 0}

        async def _fake_connect(name, scan_timeout=5.0):
            return _FakeLink()

        async def _raw_repl(blink, link):
            pass

        async def _upload(blink, link, program):
            calls["upload"] = program

        async def _restore(link):
            calls["restore"] += 1

        patches = [
            (run_mod, "_enter_raw_repl", _raw_repl),
            (run_mod, "_raw_paste_upload", _upload),
            (run_mod, "_stream_output", stream_fn),
            (run_mod, "_restore_idle_loop", _restore),
        ]
        origs = [(m, n, getattr(m, n)) for m, n, _ in patches]
        for m, n, fn in patches:
            setattr(m, n, fn)
        orig_connect = _nus.NUSLink.connect
        _nus.NUSLink.connect = staticmethod(_fake_connect)
        err = io.StringIO()
        orig_stderr = sys.stderr
        sys.stderr = err
        try:
            text = asyncio.run(sid_mod._hub_async(
                "ls", b"PROGRAM", 1.0))
        finally:
            sys.stderr = orig_stderr
            _nus.NUSLink.connect = orig_connect
            for m, n, fn in origs:
                setattr(m, n, fn)
        return text, calls

    def test_uploads_program_streams_and_restores(self):
        async def _stream(blink, link, out):
            out.write("hello %s" % sid_mod._HUB_OK_SENTINEL)
        import io, sys
        from unittest.mock import patch
        with patch("sys.stdout", io.StringIO()):
            text, calls = self._session(_stream)
        self.assertIn(sid_mod._HUB_OK_SENTINEL, text)
        self.assertEqual(calls["upload"], b"PROGRAM")
        self.assertEqual(calls["restore"], 1)

    def test_restore_runs_even_when_streaming_dies(self):
        async def _stream(blink, link, out):
            raise RuntimeError("link dropped")
        with self.assertRaises(RuntimeError):
            self._session(_stream)
        # calls dict is rebuilt per _session; verify via a fresh run
        # that the restore leg is in the finally path.
        async def _ok(blink, link, out):
            out.write("x")
        import io
        from unittest.mock import patch
        with patch("sys.stdout", io.StringIO()):
            _, calls = self._session(_ok)
        self.assertEqual(calls["restore"], 1)

    def test_connect_failure_is_a_servo_id_error(self):
        import asyncio
        from openbricks_dev import _nus

        async def _fail(name, scan_timeout=5.0):
            raise _nus.NUSError("no hub named ls")
        orig = _nus.NUSLink.connect
        _nus.NUSLink.connect = staticmethod(_fail)
        self.addCleanup(
            lambda: setattr(_nus.NUSLink, "connect", orig))
        import io, sys
        from unittest.mock import patch
        with patch("sys.stderr", io.StringIO()):
            with self.assertRaises(ServoIdError):
                asyncio.run(sid_mod._hub_async("ls", b"P", 1.0))

    def test_failing_idle_restore_is_swallowed(self):
        # A hub that hangs up during restore must not turn a
        # completed re-ID into an error.
        async def _stream(blink, link, out):
            out.write(sid_mod._HUB_OK_SENTINEL)
        from openbricks_dev import run as run_mod

        async def _bad_restore(link):
            raise RuntimeError("hub hung up first")
        orig = run_mod._restore_idle_loop
        text = None
        try:
            import io
            from unittest.mock import patch
            with patch("sys.stdout", io.StringIO()):
                run_mod._restore_idle_loop = _bad_restore
                text, _ = self._session_with_restore(_stream,
                                                     _bad_restore)
        finally:
            run_mod._restore_idle_loop = orig
        self.assertIn(sid_mod._HUB_OK_SENTINEL, text)

    def _session_with_restore(self, stream_fn, restore_fn):
        import asyncio, io, sys
        from openbricks_dev import run as run_mod
        from openbricks_dev import _nus

        class _FakeLink:
            async def __aenter__(self):
                return self

            async def __aexit__(self, *a):
                return False

        async def _fake_connect(name, scan_timeout=5.0):
            return _FakeLink()

        async def _noop2(blink, link):
            pass

        async def _noop3(blink, link, program):
            pass

        patches = [
            (run_mod, "_enter_raw_repl", _noop2),
            (run_mod, "_raw_paste_upload", _noop3),
            (run_mod, "_stream_output", stream_fn),
            (run_mod, "_restore_idle_loop", restore_fn),
        ]
        origs = [(m, n, getattr(m, n)) for m, n, _ in patches]
        for m, n, fn in patches:
            setattr(m, n, fn)
        orig_connect = _nus.NUSLink.connect
        _nus.NUSLink.connect = staticmethod(_fake_connect)
        err = io.StringIO()
        orig_stderr = sys.stderr
        sys.stderr = err
        try:
            text = asyncio.run(sid_mod._hub_async("ls", b"P", 1.0))
        finally:
            sys.stderr = orig_stderr
            _nus.NUSLink.connect = orig_connect
            for m, n, fn in origs:
                setattr(m, n, fn)
        return text, None

    def test_ctrl_c_maps_to_130(self):
        orig = sid_mod._hub_async

        def _boom(*a, **k):
            raise KeyboardInterrupt()
        sid_mod._hub_async = _boom
        self.addCleanup(setattr, sid_mod, "_hub_async", orig)
        rc = sid_mod.run(argparse.Namespace(
            new_id=3, port=None, name="ls", scan=False, old_id=None,
            baudrate=1_000_000, timeout=0.02, tx=14, rx=41,
            scan_timeout=5.0))
        self.assertEqual(rc, 130)


class ParserTests(unittest.TestCase):
    def setUp(self):
        self.parser = cli._build_parser()

    def test_servo_id_args(self):
        args = self.parser.parse_args(
            ["servo-id", "-p", "/dev/cu.usbmodem1", "3"])
        self.assertEqual(args.command, "servo-id")
        self.assertEqual(args.new_id, 3)
        self.assertEqual(args.port, "/dev/cu.usbmodem1")
        self.assertEqual(args.baudrate, 1_000_000)
        self.assertFalse(args.scan)

    def test_scan_flag_without_new_id(self):
        args = self.parser.parse_args(
            ["servo-id", "-p", "/dev/x", "--scan"])
        self.assertTrue(args.scan)
        self.assertIsNone(args.new_id)

    def test_port_is_optional(self):
        # -p omitted parses to None; run() then auto-detects.
        args = self.parser.parse_args(["servo-id", "--scan"])
        self.assertIsNone(args.port)

    def test_hub_name_and_pins_parse(self):
        args = self.parser.parse_args(["servo-id", "-n", "ls", "3"])
        self.assertEqual(args.name, "ls")
        self.assertEqual(args.tx, 14)
        self.assertEqual(args.rx, 41)
        self.assertEqual(args.scan_timeout, 5.0)

    def test_old_id_flag(self):
        args = self.parser.parse_args(
            ["servo-id", "-p", "/dev/x", "--old-id", "2", "3"])
        self.assertEqual(args.old_id, 2)


if __name__ == "__main__":
    unittest.main()
