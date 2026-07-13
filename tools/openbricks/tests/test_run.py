# SPDX-License-Identifier: MIT
"""Tests for ``openbricks_dev.run``.

``run`` stages the user's script at ``/program.py`` (same target as
``upload``) and triggers the hub-side launcher to exec it. Output
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


class _DeafHubLink:
    """Link whose hub ignores interrupts until the N-th Ctrl-C —
    simulating the injected KeyboardInterrupt being eaten by a
    scheduled callback on the hub. Only once enough interrupts have
    landed does the Ctrl-A produce the raw-REPL banner."""

    def __init__(self, wake_after):
        self.writes = []
        self._wake_after = wake_after
        self._interrupts = 0
        self._banner_queued = False

    def stats(self):
        # Shape of NUSLink.stats() — read by _format_timeout when an
        # attempt times out.
        return {"connected": True, "notify_count": 0, "byte_count": 0,
                "last_byte_ago": None, "uptime": 1.0}

    async def write(self, data):
        self.writes.append(bytes(data))
        self._interrupts += bytes(data).count(b"\x03")
        if b"\x01" in data and self._interrupts >= self._wake_after:
            self._banner_queued = True

    async def read(self, timeout=None):
        if self._banner_queued:
            self._banner_queued = False
            return _BANNER
        return b""


class EnterRawReplRetryTests(unittest.TestCase):
    """One eaten Ctrl-C must not fail the connect — the handshake is
    retried (the client-side counterpart of the hub's stop-button
    e-stop hardening)."""

    def _enter(self, link):
        blink = run_mod._BufferedLink(link)
        asyncio.run(run_mod._enter_raw_repl(blink, link))
        return link

    def test_first_attempt_success(self):
        link = self._enter(_DeafHubLink(wake_after=1))
        # Exactly one Ctrl-A sent — no spurious retries.
        ctrl_as = sum(1 for w in link.writes if b"\x01" in w)
        self.assertEqual(ctrl_as, 1)

    def test_recovers_when_first_interrupts_are_eaten(self):
        # Hub wakes only on the 5th Ctrl-C: attempts 1-2 (2 Ctrl-C
        # each) are eaten, attempt 3 crosses the threshold. The old
        # one-shot handshake failed here with notify_count=0.
        link = self._enter(_DeafHubLink(wake_after=5))
        ctrl_as = sum(1 for w in link.writes if b"\x01" in w)
        self.assertEqual(ctrl_as, 3)

    def test_raises_after_all_attempts_exhausted(self):
        link = _DeafHubLink(wake_after=10 ** 9)
        blink = run_mod._BufferedLink(link)
        try:
            asyncio.run(run_mod._enter_raw_repl(blink, link))
        except run_mod.RunError as e:
            self.assertIn("attempt %d/%d" % (run_mod._RAW_REPL_ATTEMPTS,
                                             run_mod._RAW_REPL_ATTEMPTS),
                          str(e))
        else:
            self.fail("expected RunError after exhausting retries")
        ctrl_as = sum(1 for w in link.writes if b"\x01" in w)
        self.assertEqual(ctrl_as, run_mod._RAW_REPL_ATTEMPTS)


def _args(name="RobotA", script="s.py", scan_timeout=5.0, inline_code=None):
    return argparse.Namespace(
        name=name, script=script, scan_timeout=scan_timeout,
        inline_code=inline_code)


# Hub-side response shorthands (kept in sync with test_upload).
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

    def test_bootstrap_syncs_rtc_before_running(self):
        # The hub's run-log epoch stamps need a synced RTC; the sync
        # must come BEFORE the program starts so its prints get real
        # wall-clock time.
        boot = run_mod._compose_bootstrap(b"x=1\n")
        self.assertIn(b"machine.RTC().datetime(", boot)
        self.assertTrue(
            boot.index(b"machine.RTC().datetime(")
            < boot.index(b"launcher.run_program("))

    def test_rtc_sync_lines_encode_utc_now(self):
        from datetime import datetime, timezone
        before = datetime.now(timezone.utc)
        lines = run_mod.rtc_sync_lines()
        after = datetime.now(timezone.utc)
        joined = "\n".join(lines)
        self.assertIn("machine.RTC().datetime(", joined)
        # The encoded year/month/day must be today's UTC date (both
        # endpoints checked so a midnight rollover can't flake).
        self.assertTrue(
            ("(%d, %d, %d," % (before.year, before.month, before.day))
            in joined
            or ("(%d, %d, %d," % (after.year, after.month, after.day))
            in joined)

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

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
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
        self.assertIn(b"launcher.run()", joined)  # idle loop restored on exit
        self.assertTrue(fake.closed)

    def test_button_press_stop_surfaces_as_clean_message(self):
        # The hub's bootstrap catches KeyboardInterrupt and prints a
        # clean line — we assert that line reaches stdout rather than
        # a raw traceback.
        fake = _ScriptedLink(self._standard_responses(
            b"partial output\r\nopenbricks: stopped by button press.\r\n"))

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
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

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
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

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return fake

        with patch.object(run_mod.NUSLink, "connect", side_effect=_fake_connect):
            with self.assertRaises(run_mod.RunError) as ctx:
                asyncio.run(run_mod._run_async("RobotA", tmp.name, 5.0))
        self.assertIn("raw-paste", str(ctx.exception))
        # Even on the error path, the finally must hand the hub back
        # to the launcher idle loop (button start/stop keeps working).
        self.assertIn(b"launcher.run()", b"".join(fake.writes))


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

    def test_command_and_script_mutually_exclusive(self):
        with patch.object(run_mod.NUSLink, "connect") as connect:
            with self.assertRaises(run_mod.RunError) as ctx:
                asyncio.run(
                    run_mod._run_async("RobotA", "/path/to/script.py", 5.0,
                                       command="print('hi')"))
        connect.assert_not_called()
        self.assertIn("either", str(ctx.exception).lower())

    def test_neither_script_nor_command_raises(self):
        with patch.object(run_mod.NUSLink, "connect") as connect:
            with self.assertRaises(run_mod.RunError) as ctx:
                asyncio.run(
                    run_mod._run_async("RobotA", None, 5.0, command=None))
        connect.assert_not_called()
        self.assertIn("missing program", str(ctx.exception).lower())


class InlineCommandTests(unittest.TestCase):
    """``-c CODE`` runs the same flow as a SCRIPT path, but uses the
    inline string instead of a file's contents."""

    def test_command_bytes_reach_the_bootstrap(self):
        responses = [
            b"",
            _BANNER,
            _R_SUPPORTED + _WINDOW_8K,
            _CTRL_D,
            b"hello\r\n" + _CTRL_D,
            _CTRL_D,
        ]
        fake = _ScriptedLink(responses)

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return fake

        with patch.object(run_mod.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO):
            rc = run_mod.run(_args(script=None, inline_code="print('hello')"))

        self.assertEqual(rc, 0)
        joined = b"".join(fake.writes)
        # The inline ``CODE`` lands inside the ``f.write(...)`` repr in
        # the bootstrap — verify the bytes are present.
        self.assertIn(b"print('hello')", joined)
        self.assertIn(b"launcher.run_program", joined)


    def test_connect_failure_propagates_as_run_error(self):
        tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False)
        tmp.write("pass\n")
        tmp.close()
        self.addCleanup(os.unlink, tmp.name)

        async def _raise(name, scan_timeout=5.0, debug=False):
            raise NUSError("no hub named 'RobotA' found")

        with patch.object(run_mod.NUSLink, "connect", side_effect=_raise):
            with self.assertRaises(run_mod.RunError) as ctx:
                asyncio.run(run_mod._run_async("RobotA", tmp.name, 5.0))
        self.assertIn("no hub named", str(ctx.exception))




class _ProtocolLink(_ScriptedLink):
    """ScriptedLink + the stats() surface _format_timeout needs."""

    def stats(self):
        return {"connected": True, "notify_count": 5, "byte_count": 99,
                "last_byte_ago": 0.5, "uptime": 3.0}


def _drive(coro):
    import asyncio
    return asyncio.run(coro)


class RawPasteProtocolTests(unittest.TestCase):
    """Flow-control branches of the raw-paste upload: window refills,
    hub abort, junk bytes, and the end-of-upload handshake."""

    def test_window_refill_chunks_and_completes(self):
        link = _ProtocolLink([b"R\x01\x04\x00", b"\x01", b"\x01", b"\x04"])
        blink = run_mod._BufferedLink(link)
        _drive(run_mod._raw_paste_upload(blink, link, b"abcdef"))
        self.assertEqual(link.writes, [
            run_mod._RAW_PASTE_REQUEST, b"abcd", b"ef", run_mod._CTRL_D])

    def test_hub_abort_sends_ctrl_d_and_raises(self):
        link = _ProtocolLink([b"R\x01\x01\x00", b"\x04"])
        blink = run_mod._BufferedLink(link)
        try:
            _drive(run_mod._raw_paste_upload(blink, link, b"ab"))
        except run_mod.RunError as e:
            self.assertIn("hub aborted", str(e))
        else:
            self.fail("expected RunError")
        self.assertEqual(link.writes[-1], run_mod._CTRL_D)

    def test_junk_during_upload_raises(self):
        link = _ProtocolLink([b"R\x01\x01\x00", b"Z"])
        blink = run_mod._BufferedLink(link)
        try:
            _drive(run_mod._raw_paste_upload(blink, link, b"ab"))
        except run_mod.RunError as e:
            self.assertIn("unexpected byte", str(e))
        else:
            self.fail("expected RunError")

    def test_junk_after_end_raises(self):
        link = _ProtocolLink([b"R\x01\x04\x00", b"Z"])
        blink = run_mod._BufferedLink(link)
        try:
            _drive(run_mod._raw_paste_upload(blink, link, b"a"))
        except run_mod.RunError as e:
            self.assertIn("after raw-paste end", str(e))
        else:
            self.fail("expected RunError")

    def test_missing_raw_paste_support_raises(self):
        link = _ProtocolLink([b"XX"])
        blink = run_mod._BufferedLink(link)
        try:
            _drive(run_mod._raw_paste_upload(blink, link, b"a"))
        except run_mod.RunError as e:
            self.assertIn("did not acknowledge raw-paste", str(e))
        else:
            self.fail("expected RunError")


class StreamOutputTests(unittest.TestCase):
    def test_buffered_stdout_and_stderr_sections(self):
        import io
        link = _ProtocolLink([])
        blink = run_mod._BufferedLink(link)
        blink._buf = bytearray(b"out\x04trace\x04")
        out = io.StringIO()
        _drive(run_mod._stream_output(blink, link, out))
        self.assertIn("out", out.getvalue())
        self.assertIn("trace", out.getvalue())

    def test_live_then_silent_hub_raises_formatted_timeout(self):
        import io
        link = _ProtocolLink([b"live"])
        blink = run_mod._BufferedLink(link)
        out = io.StringIO()
        try:
            _drive(run_mod._stream_output(blink, link, out))
        except run_mod.RunError as e:
            self.assertIn("timed out reading from hub", str(e))
        else:
            self.fail("expected RunError")
        self.assertEqual(out.getvalue(), "live")


class FormatTimeoutHintTests(unittest.TestCase):
    class _StatsLink:
        def __init__(self, stats):
            self._stats = stats

        def stats(self):
            return self._stats

    def _msg(self, **overrides):
        stats = {"connected": True, "notify_count": 3, "byte_count": 42,
                 "last_byte_ago": 1.0, "uptime": 9.0}
        stats.update(overrides)
        return run_mod._format_timeout(
            self._StatsLink(stats), "step-x", b"")

    def test_went_quiet_hint(self):
        msg = self._msg(last_byte_ago=6.5)
        self.assertIn("went quiet for 6.5s", msg)

    def test_still_talking_hint(self):
        msg = self._msg(last_byte_ago=1.0)
        self.assertIn("protocol diverged", msg)

    def test_never_sent_hint(self):
        msg = self._msg(notify_count=0, last_byte_ago=None)
        self.assertIn("notify_count=0", msg)


class BufferedLinkDrainTests(unittest.TestCase):
    def test_drain_discards_pending_bytes(self):
        link = _ProtocolLink([b"stale"])
        blink = run_mod._BufferedLink(link)
        _drive(blink.drain(timeout=0.05))
        self.assertEqual(bytes(blink._buf), b"")

    def test_drain_tolerates_silent_link(self):
        import asyncio

        class _Slow:
            async def read(self, timeout=None):
                await asyncio.sleep(1.0)
                return b"late"
        blink = run_mod._BufferedLink(_Slow())
        _drive(blink.drain(timeout=0.05))   # must not raise
        self.assertEqual(bytes(blink._buf), b"")


class HostInterruptForwardingTests(unittest.TestCase):
    """Host-side Ctrl-C mid-run: _run_async forwards a Ctrl-C to the
    hub, drains the interrupt traceback, restores the idle loop, and
    re-raises so the client exits."""

    def test_cancelled_error_forwards_ctrl_c(self):
        import asyncio
        link = _ProtocolLink([])
        restore_calls = []
        stream_calls = []

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return link

        async def _stub(blink, l):
            return None

        async def _stub_upload(blink, l, script_bytes):
            return None

        async def _stream(blink, l, out):
            stream_calls.append(1)
            if len(stream_calls) == 1:
                raise asyncio.CancelledError()

        async def _restore(l):
            restore_calls.append(1)

        patches = [
            ("_enter_raw_repl", _stub),
            ("_raw_paste_upload", _stub_upload),
            ("_stream_output", _stream),
            ("_restore_idle_loop", _restore),
        ]
        orig_connect = run_mod.NUSLink.connect
        run_mod.NUSLink.connect = _fake_connect
        origs = [(n, getattr(run_mod, n)) for n, _ in patches]
        for n, fn in patches:
            setattr(run_mod, n, fn)
        try:
            with self.assertRaises(asyncio.CancelledError):
                asyncio.run(run_mod._run_async(
                    "X", None, 1.0, command="print(1)"))
        finally:
            run_mod.NUSLink.connect = orig_connect
            for n, fn in origs:
                setattr(run_mod, n, fn)
        # Ctrl-C forwarded to the hub, drain attempted, idle restored.
        self.assertIn(run_mod._CTRL_C, link.writes)
        self.assertEqual(len(stream_calls), 2)
        self.assertEqual(restore_calls, [1])


class RunKeyboardInterruptTests(unittest.TestCase):
    def test_ctrl_c_maps_to_130(self):
        import argparse
        orig = run_mod._run_async

        def _boom(*a, **k):
            raise KeyboardInterrupt()
        run_mod._run_async = _boom
        try:
            rc = run_mod.run(argparse.Namespace(
                name="x", script="s.py", scan_timeout=1.0,
                debug=False, inline_code=None))
        finally:
            run_mod._run_async = orig
        self.assertEqual(rc, 130)


if __name__ == "__main__":
    unittest.main()
