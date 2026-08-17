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
import signal
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
        if timeout == 0:
            # Non-blocking stale-drain: a real hub has sent nothing
            # ahead of a request unless the test staged it.
            stale = getattr(self, "stale", None)
            if stale:
                return stale.pop(0)
            return b""
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
_FLOW_ACK    = b"\x01"  # raw-paste mid-transfer window-refill byte


# Hub name used by staging fixtures — keys the sealed framing.
_HUB = "ls"


class ComposeTests(unittest.TestCase):
    """Stage-chunk + runner composition — pure functions, no BLE."""

    def test_stage_chunk_first_truncates_then_appends(self):
        first = run_mod._compose_stage_chunk("/program.py", b"abc", True, _HUB)
        rest = run_mod._compose_stage_chunk("/program.py", b"def", False, _HUB)
        self.assertIn(b"'wb'", first)
        self.assertIn(b"'ab'", rest)
        self.assertIn(b"'/program.py'", first)
        self.assertIn(b"abc", first)

    def test_stage_chunk_is_bounded_even_for_binary(self):
        # The whole point: each staged program must fit a fragmented
        # heap. Worst-case repr expansion (pure binary) stays ~4x.
        chunk = bytes(range(256)) * 2   # 512 binary bytes
        prog = run_mod._compose_stage_chunk("/program.py", chunk, True, _HUB)
        self.assertLess(len(prog), 2600)

    def test_bootstrap_calls_launcher_run_program(self):
        boot = run_mod._compose_runner('/program.py')
        self.assertIn(b"from openbricks import launcher", boot)
        self.assertIn(b"launcher.run_program(", boot)
        self.assertIn(b"'/program.py'", boot)

    def test_bootstrap_prints_firmware_label_first(self):
        # ``openbricks run`` shows the hub's firmware version (with
        # its official/customized provenance suffix) at the top of
        # the stream, before the program launches; pre-1.79 firmware
        # has no firmware_label and falls back to the bare version.
        boot = run_mod._compose_runner('/program.py')
        self.assertIn(b"openbricks.firmware_label()", boot)
        self.assertIn(b"except AttributeError", boot)
        self.assertIn(b"openbricks.__version__", boot)
        self.assertTrue(
            boot.index(b"firmware_label")
            < boot.index(b"launcher.run_program("))
        import ast
        ast.parse(boot.decode())

    def test_runner_is_fixed_size_no_payload(self):
        # The runner must not embed the script — it runs the staged
        # file, so its paste size is constant regardless of script
        # growth.
        boot = run_mod._compose_runner('/program.py')
        # Bound is a payload-embedding tripwire, not a byte budget:
        # bumped 600 -> 800 for the firmware-label banner (1.79.0).
        self.assertLess(len(boot), 800)
        self.assertNotIn(b"f.write", boot)

    def test_bootstrap_syncs_rtc_before_running(self):
        # The hub's run-log epoch stamps need a synced RTC; the sync
        # must come BEFORE the program starts so its prints get real
        # wall-clock time.
        boot = run_mod._compose_runner('/program.py')
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
        run_program; the runner must catch it so the hub prints a
        clean stop message instead of letting the raw-REPL surface an
        interrupt traceback."""
        boot = run_mod._compose_runner('/program.py')
        self.assertIn(b"except KeyboardInterrupt", boot)
        self.assertIn(b"stopped by button press", boot)
        # And the stop-path evidence rides in the same stream: the
        # hard-button counters are printed from INSIDE the run, before
        # any reboot/re-init can zero them (bench 2026-08-03: a
        # post-hoc stats read came back all-zero after a BLE session
        # recovery). Guarded for non-firmware runtimes.
        self.assertIn(b"hard_button_stats", boot)
        self.assertIn(b"except (ImportError, AttributeError)", boot)
        # The runner must still be valid Python.
        import ast
        ast.parse(boot.decode())

    def test_runner_targets_the_given_path(self):
        boot = run_mod._compose_runner("/program.mpy")
        self.assertIn(b"'/program.mpy'", boot)
        self.assertNotIn(b"os.remove", boot)

    def test_runner_removes_the_stale_source_before_launch(self):
        # Staging .mpy deletes /program.py: the launcher's button path
        # runs source when present, so a stale source would shadow
        # every subsequent compiled stage.
        boot = run_mod._compose_runner("/program.mpy", "/program.py")
        self.assertIn(b"os.remove('/program.py')", boot)
        self.assertTrue(
            boot.index(b"os.remove")
            < boot.index(b"launcher.run_program("))
        import ast
        ast.parse(boot.decode())

    def test_parse_fw_version_reads_the_probe_line(self):
        self.assertEqual(
            run_mod._parse_fw_version("fwv=1.92.0\r\n"), (1, 92, 0))
        self.assertEqual(
            run_mod._parse_fw_version("noise\nfwv=2.0.13\n"), (2, 0, 13))

    def test_parse_fw_version_rejects_garbage(self):
        self.assertIsNone(run_mod._parse_fw_version(""))
        self.assertIsNone(run_mod._parse_fw_version("fwv=1.92"))
        self.assertIsNone(run_mod._parse_fw_version("fwv=a.b.c"))
        self.assertIsNone(run_mod._parse_fw_version("version 1.92.0"))

    def test_stage_chunk_user_bytes_round_trip(self):
        # Any bytes the user script could contain — NULs, high bits,
        # quotes — must survive ``repr()`` wrapping.
        tricky = b"\x00\xff\r\n'\"\\"
        prog = run_mod._compose_stage_chunk("/program.py", tricky, True, _HUB)
        self.assertIn(repr(tricky).encode(), prog)


class RunFlowTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.NamedTemporaryFile(
            mode="w", suffix=".py", delete=False)
        self.tmp.write("print('hello from hub')\n")
        self.tmp.close()
        self.addCleanup(os.unlink, self.tmp.name)

    @staticmethod
    def _stage_round(n=1):
        """Scripted responses for ``n`` chunked staging execs: each is
        a raw-paste handshake, end-of-paste ack, empty stdout/stderr
        framing, and the raw-REPL prompt."""
        out = []
        for _ in range(n):
            out += [
                _R_SUPPORTED + _WINDOW_8K,    # staging paste ack + window
                _CTRL_D,                      # end-of-paste ack
                _CTRL_D,                      # empty stdout + EOT
                _CTRL_D,                      # empty stderr + EOT
                b">",                         # raw-REPL prompt
            ]
        return out

    @staticmethod
    def _probe_round(version=b"fwv=1.92.0\r\n"):
        """The firmware-version probe exec that precedes staging: one
        raw-paste handshake whose stdout carries the ``fwv=`` line."""
        return [
            _R_SUPPORTED + _WINDOW_8K,    # probe paste ack + window
            _CTRL_D,                      # end-of-paste ack
            version + _CTRL_D,            # stdout: version + EOT
            _CTRL_D,                      # empty stderr + EOT
            b">",                         # raw-REPL prompt
        ]

    def _standard_responses(self, stdout_msg, stderr_msg=b"", stage_rounds=1,
                            fw_version=b"fwv=1.92.0\r\n"):
        return [
            b"",                          # drain after Ctrl-C interrupt
            _BANNER,                      # raw-REPL banner
        ] + self._probe_round(fw_version) \
          + self._stage_round(stage_rounds) + [
            _R_SUPPORTED + _WINDOW_8K,    # runner paste ack + window
            _CTRL_D,                      # end-of-paste ack
            stdout_msg + _CTRL_D,         # stdout + EOT
            stderr_msg + _CTRL_D,         # stderr + EOT
        ]

    def test_old_firmware_gets_source_with_a_notice(self):
        # Pre-1.92.0 firmware can't run /program.mpy — the probe
        # downgrades to source staging and SAYS so on stderr. Nothing
        # about the downgrade is silent.
        fake = _ScriptedLink(self._standard_responses(
            b"hello from hub\r\n", fw_version=b"fwv=1.91.1\r\n"))

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return fake

        err = io.StringIO()
        with patch.object(run_mod.NUSLink, "connect",
                          side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO), \
             patch("sys.stderr", err):
            rc = run_mod.run(_args(script=self.tmp.name))

        self.assertEqual(rc, 0)
        joined = b"".join(fake.writes)
        # The SOURCE crossed the link, targeting the BUTTON slot's
        # source path (run = upload-then-run since 2.7.0, so a failed
        # run stays button-rerunnable), and no sibling delete was
        # composed on the source path.
        self.assertIn(b"hello from hub", joined)
        self.assertIn(b"'/program.py'", joined)
        self.assertNotIn(b"'/run.py'", joined)
        self.assertNotIn(b"os.remove", joined)
        self.assertIn("predates precompiled", err.getvalue())
        self.assertIn("1.91.1", err.getvalue())

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
        # Confirm the staging writes the BUTTON slot (run =
        # upload-then-run since 2.7.0) and calls run_program on it.
        self.assertIn(b"'/program.mpy'", joined)
        self.assertNotIn(b"'/run.mpy'", joined)
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
            # probe round: firmware version
            _R_SUPPORTED + _WINDOW_8K,
            _CTRL_D,
            b"fwv=1.92.0\r\n" + _CTRL_D,
            _CTRL_D,
            b">",
            # one staging round (inline code is < _STAGE_CHUNK_BYTES)
            _R_SUPPORTED + _WINDOW_8K,
            _CTRL_D,
            _CTRL_D,
            _CTRL_D,
            b">",
            # runner round
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
        # 1.92.0 firmware: the staged payload is the COMPILED program
        # (the source never crosses the link), and the runner targets
        # the .mpy path.
        self.assertIn(b"M\\x06", joined)   # .mpy header inside the
        self.assertNotIn(b"print('hello')", joined)  # staged repr
        self.assertIn(b"launcher.run_program", joined)
        # run stages to the BUTTON slot (2.7.0: upload-then-run, so
        # a failed run stays rerunnable by start press) — the .mpy
        # staging removes a stale source sibling so the launcher's
        # source-wins resolution can't run old code.
        self.assertIn(b"'/program.mpy'", joined)
        self.assertNotIn(b"'/run.mpy'", joined)
        self.assertIn(b"os.remove('/program.py')", joined)


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


class _BurstMeasuringLink(_ProtocolLink):
    """Hub model that advertises ``window`` and records the LARGEST
    burst the host writes before it stops to consume an ack.

    That burst is exactly what the hub's BLE GATT rx buffer must
    absorb: NimBLE drops writes into a full buffer silently, so a
    burst bigger than the buffer truncates the pasted program with
    no error anywhere (the 1.32.0 bug)."""

    def __init__(self, window, acks=64):
        super().__init__(
            [bytes([0x52, 0x01, window & 0xFF, window >> 8, 0x01])]
            + [b"\x01"] * acks + [b"\x04"])
        self.max_burst = 0
        self._burst = 0

    async def write(self, data):
        if data not in (run_mod._RAW_PASTE_REQUEST, run_mod._CTRL_D):
            self._burst += len(data)
            if self._burst > self.max_burst:
                self.max_burst = self._burst
        await super().write(data)

    async def read(self, timeout=None):
        chunk = await super().read(timeout=timeout)
        if chunk:          # an ack means the hub drained a window
            self._burst = 0
        return chunk


def _firmware_int(path, needle, sep):
    """Pull a constant out of a firmware source file, or skip the
    test when running outside a repo checkout (installed wheel)."""
    import os
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "..", "..")
    full = os.path.join(root, path)
    if not os.path.isfile(full):
        raise unittest.SkipTest("not a repo checkout: %s missing" % path)
    with open(full) as f:
        for line in f:
            if line.strip().startswith(needle):
                return int(line.split(sep)[1].strip().strip("()"))
    raise AssertionError("%s not found in %s" % (needle, path))


class HostBurstVsHubBufferTests(unittest.TestCase):
    """REGRESSION (1.32.0 -> 1.32.1), behavioural half.

    ``BleRxBufferTests`` (firmware suite) pins the two constants
    against each other arithmetically. This drives the REAL host
    protocol against a hub model advertising the shipped window and
    measures what the host actually bursts, so the guard holds even
    if the flow-control code changes how it batches writes."""

    def _paste(self, window):
        link = _BurstMeasuringLink(window)
        blink = run_mod._BufferedLink(link)
        program = run_mod._compose_stage_chunk(
            "/program.py", b"x" * 4096, True, "ls")
        _drive(run_mod._raw_paste_upload(blink, link, program))
        return link.max_burst

    def test_shipped_window_burst_fits_shipped_ble_buffer(self):
        try:
            buffer_max = _firmware_int(
                "native/boards/openbricks_esp32s3/mpconfigboard.h",
                "#define MICROPY_REPL_STDIN_BUFFER_MAX", "MAX")
        except AssertionError:
            # Stock window (1.33.1 reverted the bump) — MicroPython's
            # own default. The invariant still has to hold, and this
            # test is what will catch a future raise that outruns the
            # BLE buffer.
            buffer_max = 256
        rx = _firmware_int("openbricks/ble_repl.py",
                           "_RX_BUFFER_BYTES", "=")
        burst = self._paste(buffer_max // 2)   # window = buf_max / 2
        self.assertLessEqual(
            burst, rx,
            "host bursts %d bytes with the shipped window but the hub's "
            "BLE rx buffer is only %d - NimBLE drops the excess and the "
            "pasted program is silently truncated (the 1.32.0 bug)"
            % (burst, rx))

    def test_the_1_32_0_combination_would_overflow(self):
        # Documents the shipped-broken pairing: window 2048 against
        # the old 512-byte buffer.
        self.assertGreater(self._paste(2048), 512)


class RawPasteProtocolTests(unittest.TestCase):
    """Flow-control branches of the raw-paste upload: window refills,
    hub abort, junk bytes, and the end-of-upload handshake."""

    def test_window_refill_chunks_and_completes(self):
        link = _ProtocolLink([b"R\x01\x04\x00", b"\x01", b"\x01", b"\x04"])
        blink = run_mod._BufferedLink(link)
        _drive(run_mod._raw_paste_upload(blink, link, b"abcdef"))
        self.assertEqual(link.writes, [
            run_mod._RAW_PASTE_REQUEST, b"abcd", b"ef", run_mod._CTRL_D])

    def test_stale_banner_bytes_are_drained_before_handshake(self):
        # A retried raw-REPL entry can leave a duplicate banner both
        # in the pushback buffer (read_until stopped at the FIRST
        # banner) and queued on the link. The handshake must read the
        # hub's reply, not the leftovers — the "got b'ra'" wedge.
        link = _ProtocolLink([b"R\x01\x04\x00", b"\x04"])
        link.stale = [b"raw REPL; CTRL-B to exit\r\n>"]
        blink = run_mod._BufferedLink(link)
        blink._buf = bytearray(b"raw REPL; CTRL-B to exit\r\n>")
        _drive(run_mod._raw_paste_upload(blink, link, b"ab"))
        self.assertEqual(link.writes, [
            run_mod._RAW_PASTE_REQUEST, b"ab", run_mod._CTRL_D])

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

    def test_live_then_dropped_link_raises_formatted_timeout(self):
        import io

        class _DroppedLink(_ProtocolLink):
            def stats(self):
                s = _ProtocolLink.stats(self)
                s["connected"] = False
                return s

        link = _DroppedLink([b"live"])
        blink = run_mod._BufferedLink(link)
        out = io.StringIO()
        try:
            _drive(run_mod._stream_output(blink, link, out))
        except run_mod.RunError as e:
            self.assertIn("timed out reading from hub", str(e))
        else:
            self.fail("expected RunError")
        self.assertEqual(out.getvalue(), "live")

    def test_quiet_but_connected_hub_keeps_the_session_alive(self):
        # A program that drives silently for minutes is healthy; only
        # a dropped BLE link may end the wait. Two empty reads (each
        # a 30 s quiet window in production) then the output arrives.
        import io
        import sys as _sys
        link = _ProtocolLink([b"", b"", b"done\x04", b"\x04"])
        blink = run_mod._BufferedLink(link)
        out = io.StringIO()
        err = io.StringIO()
        orig = _sys.stderr
        _sys.stderr = err
        try:
            _drive(run_mod._stream_output(blink, link, out))
        finally:
            _sys.stderr = orig
        self.assertEqual(out.getvalue(), "done")
        self.assertEqual(err.getvalue().count("quiet but connected"), 1,
                         "the waiting note must print exactly once")


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
    """Host-side Ctrl-C mid-run: _run_async stops the ROBOT — via the
    verified, retried _enter_raw_repl primitive, not one unverified
    Ctrl-C write — reports the outcome on stderr either way, restores
    the idle loop, and re-raises so the client exits.

    Why not a bare write: a single injected interrupt can be eaten by
    a scheduled callback on the hub, and ``except Exception: pass``
    around it made "stopped" and "still driving at full speed"
    indistinguishable at the terminal."""

    def _interrupt(self, stream_exc, raw_repl=None, restore=None):
        """Run _run_async to its interrupt path with the protocol
        stubbed out. ``raw_repl``/``restore`` are per-call hooks fed
        the 1-based call count. Returns (call counts, stderr text)."""
        import sys
        link = _ProtocolLink([])
        calls = {"raw_repl": 0, "stream": 0, "restore": 0}

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return link

        async def _raw_repl(blink, l):
            calls["raw_repl"] += 1
            if raw_repl is not None:
                await raw_repl(calls["raw_repl"])

        async def _stream(blink, l, out):
            calls["stream"] += 1
            raise stream_exc()

        async def _restore(l):
            calls["restore"] += 1
            if restore is not None:
                await restore(calls["restore"])

        async def _stub_stage(blink, l, target_path, payload, hub_name):
            return None

        async def _stub_upload(blink, l, script_bytes):
            return None

        async def _stub_pick(blink, l, *slot_pair):
            return run_mod._TARGET_PATH, False, None

        patches = [
            ("_enter_raw_repl", _raw_repl),
            ("_pick_staging", _stub_pick),
            ("_stage_file", _stub_stage),
            ("_raw_paste_upload", _stub_upload),
            ("_stream_output", _stream),
            ("_restore_idle_loop", _restore),
        ]
        orig_connect = run_mod.NUSLink.connect
        run_mod.NUSLink.connect = _fake_connect
        origs = [(n, getattr(run_mod, n)) for n, _ in patches]
        for n, fn in patches:
            setattr(run_mod, n, fn)
        err = io.StringIO()
        orig_stderr = sys.stderr
        sys.stderr = err
        try:
            with self.assertRaises(
                    (KeyboardInterrupt, asyncio.CancelledError)):
                asyncio.run(run_mod._run_async(
                    "X", None, 1.0, command="print(1)"))
        finally:
            sys.stderr = orig_stderr
            run_mod.NUSLink.connect = orig_connect
            for n, fn in origs:
                setattr(run_mod, n, fn)
        return calls, err.getvalue()

    def test_keyboard_interrupt_stops_verified_and_reports(self):
        # THE case that actually happens at a terminal: asyncio.run
        # raises KeyboardInterrupt at the await point, not
        # CancelledError. The stop is _enter_raw_repl called a second
        # time — reaching the raw-REPL banner IS the proof the
        # program died — and the user is told so.
        calls, err = self._interrupt(KeyboardInterrupt)
        self.assertEqual(calls["raw_repl"], 2)
        self.assertEqual(calls["restore"], 1)
        self.assertIn("robot stopped.", err)
        self.assertNotIn("WARNING", err)

    def test_cancelled_error_takes_the_same_path(self):
        calls, err = self._interrupt(asyncio.CancelledError)
        self.assertEqual(calls["raw_repl"], 2)
        self.assertIn("robot stopped.", err)

    def test_unconfirmed_stop_is_loud_not_silent(self):
        # The link dies as the interrupt arrives — Ctrl-C often
        # follows the robot driving out of BLE range. The old code
        # swallowed the failure and printed only "aborted.": success
        # and a still-driving robot looked identical. Now the failure
        # is named, with what to do about it.
        async def dead_link(n):
            if n >= 2:                     # connect-time call succeeds
                raise OSError("link gone")
        calls, err = self._interrupt(KeyboardInterrupt,
                                     raw_repl=dead_link)
        self.assertIn("could not confirm the robot stopped", err)
        self.assertIn("hub button", err)
        self.assertEqual(calls["restore"], 1)

    def test_second_ctrl_c_does_not_abandon_the_stop(self):
        # A second Ctrl-C lands as another KeyboardInterrupt at
        # whatever await the stop sequence is in. It must restart the
        # stop, not abandon a moving robot mid-stop (KeyboardInterrupt
        # is a BaseException: the old ``except Exception`` guards let
        # it fly straight through).
        async def second_ctrl_c(n):
            if n == 2:
                raise KeyboardInterrupt()
        calls, err = self._interrupt(KeyboardInterrupt,
                                     raw_repl=second_ctrl_c)
        self.assertEqual(calls["raw_repl"], 3)     # stop re-run
        self.assertIn("robot stopped.", err)

    def test_ctrl_c_during_restore_is_absorbed(self):
        async def flaky_restore(n):
            if n == 1:
                raise KeyboardInterrupt()
        calls, err = self._interrupt(KeyboardInterrupt,
                                     restore=flaky_restore)
        self.assertEqual(calls["restore"], 2)
        self.assertNotIn("re-arm", err)

    def test_failed_restore_names_the_trap(self):
        # A dead idle loop means button presses silently do nothing
        # until a power-cycle — the exact trap _restore_idle_loop was
        # built to close. Failing to restore must say so, not pass.
        async def dead_restore(n):
            raise OSError("link gone")
        calls, err = self._interrupt(KeyboardInterrupt,
                                     restore=dead_restore)
        self.assertIn("could not re-arm", err)
        self.assertIn("power-cycle", err)


class VerifiedRestoreTests(unittest.TestCase):
    """_restore_idle_loop must confirm the hub's idle banner, retrying
    the send — a restore lost in session teardown used to leave the
    hub parked with a dead idle loop and silently dead button starts
    until a power-cycle."""

    def test_banner_confirms_on_first_attempt(self):
        link = _ProtocolLink([b"openbricks: idle. Press button to run /program.py\r\n"])
        _drive(run_mod._restore_idle_loop(link))
        sends = [w for w in link.writes
                 if run_mod._RESTORE_IDLE_SNIPPET.split(b"\r")[0] in w]
        self.assertEqual(len(sends), 1)

    def test_missing_banner_retries_and_warns(self):
        import io, sys
        link = _ProtocolLink([])   # hub never answers
        err = io.StringIO()
        orig, sys.stderr = sys.stderr, err
        try:
            _drive(run_mod._restore_idle_loop(link))
        finally:
            sys.stderr = orig
        sends = [w for w in link.writes
                 if run_mod._RESTORE_IDLE_SNIPPET.split(b"\r")[0] in w]
        self.assertEqual(len(sends), run_mod._RESTORE_ATTEMPTS)
        self.assertIn("could not confirm", err.getvalue())

    def test_banner_on_second_attempt_stops_retrying(self):
        class _SecondTimeLucky(_ProtocolLink):
            def __init__(self):
                super().__init__([])
                self._attempt = 0

            async def read(self, timeout=None):
                if self._attempt >= 2 and not self._responses:
                    return b"Press button to run /program.py\r\n"
                return await super().read(timeout=timeout)

            async def write(self, data):
                await super().write(data)
                if run_mod._CTRL_D in data:
                    self._attempt += 1
        link = _SecondTimeLucky()
        _drive(run_mod._restore_idle_loop(link))
        sends = [w for w in link.writes if run_mod._CTRL_D in w]
        self.assertEqual(len(sends), 2)

    def test_write_failure_retries_without_reading(self):
        # The disconnect race the verified restore exists for: every
        # write dies. Each attempt must move on (no read on a dead
        # link) and the warning must still be printed.
        class _DeadLink(_ProtocolLink):
            def __init__(self):
                super().__init__([])
                self.write_attempts = 0
                self.read_attempts = 0

            async def write(self, data):
                self.write_attempts += 1
                raise OSError("disconnected")

            async def read(self, timeout=None):
                self.read_attempts += 1
                return await super().read(timeout=timeout)

        import sys
        link = _DeadLink()
        err = io.StringIO()
        orig, sys.stderr = sys.stderr, err
        try:
            _drive(run_mod._restore_idle_loop(link))
        finally:
            sys.stderr = orig
        self.assertEqual(link.write_attempts, run_mod._RESTORE_ATTEMPTS)
        self.assertEqual(link.read_attempts, 0)
        self.assertIn("could not confirm", err.getvalue())

    def test_read_failure_moves_to_next_attempt(self):
        # Link dies mid-listen on attempt 1 (read raises), then
        # attempt 2 delivers the banner: confirm, no warning.
        class _ReadDiesOnce(_ProtocolLink):
            def __init__(self):
                super().__init__([])
                self._attempt = 0

            async def write(self, data):
                await super().write(data)
                if run_mod._CTRL_D in data:
                    self._attempt += 1

            async def read(self, timeout=None):
                if self._attempt == 1:
                    raise OSError("link reset")
                return b"Press button to run /program.py\r\n"

        import sys
        link = _ReadDiesOnce()
        err = io.StringIO()
        orig, sys.stderr = sys.stderr, err
        try:
            _drive(run_mod._restore_idle_loop(link))
        finally:
            sys.stderr = orig
        sends = [w for w in link.writes if run_mod._CTRL_D in w]
        self.assertEqual(len(sends), 2)
        self.assertEqual(err.getvalue(), "")


class ChunkedStagingTests(unittest.TestCase):
    """_stage_file: bounded-memory staging (the fragmented-heap fix —
    bench: 177 KB free / 5.2 KB max hole aborted a 9.4 KB one-shot
    paste at ~6.4 KB received)."""

    def _stage(self, payload):
        """Script the exact response sequence ``_stage_file`` needs
        for THIS payload: one round per real chunk, each round
        pre-loaded with enough raw-paste window refills (0x2000 =
        8 KB per ``_WINDOW_8K``) for its actual ON-THE-WIRE length —
        NOT the raw chunk length: ``_compose_stage_chunk`` wraps each
        chunk in a ``with open(...) as f: f.write(<repr>)`` program,
        so the paste is chunk bytes PLUS wrapper/repr overhead (~54
        bytes for printable payloads, more for escaped/binary ones).
        A chunk at or near one window (now the common case, since
        1.22.2 raised the chunk size to _MAX_SCRIPT_BYTES) needs
        mid-transfer ``_FLOW_ACK`` bytes or the real
        ``_raw_paste_upload`` loop stalls waiting for one and reads
        the next round's ack byte as a (mis-timed) abort instead —
        the exact wrapper overhead is what pushed a 65536-byte chunk
        just past an 8-window boundary and caught this the first
        time (65536 raw vs. 65590 on the wire)."""
        chunk_size = run_mod._STAGE_CHUNK_BYTES
        window = 0x2000
        offsets = list(range(0, len(payload), chunk_size)) or [0]
        responses = []
        for off in offsets:
            chunk = payload[off:off + chunk_size]
            # NOTE: the compressed framing embeds a random nonce, so
            # this program is not byte-identical to the one
            # _stage_file will build — but its LENGTH is (same chunk,
            # same-size nonce/b64), which is all the ack math needs.
            program = run_mod._compose_stage_chunk(
                "/program.py", chunk, first=(off == 0), hub_name=_HUB)
            acks_needed = max(0, -(-len(program) // window) - 1)
            responses += (
                [_R_SUPPORTED + _WINDOW_8K] + [_FLOW_ACK] * acks_needed
                + [_CTRL_D, _CTRL_D, _CTRL_D, b">"])
        link = _ProtocolLink(responses)
        blink = run_mod._BufferedLink(link)
        _drive(run_mod._stage_file(blink, link, "/program.py", payload,
                                   _HUB))
        return link

    def test_large_payload_splits_at_chunk_size(self):
        payload = b"x" * (run_mod._STAGE_CHUNK_BYTES * 2 + 100)
        link = self._stage(payload)
        writes = b"".join(link.writes)
        self.assertEqual(writes.count(b"'wb'"), 1)
        self.assertEqual(writes.count(b"'ab'"), 2)

    def test_every_staged_program_is_bounded(self):
        # The invariant that makes staging fragmentation-proof: no
        # single paste may exceed ~4x the chunk size. Force 4 chunks
        # of worst-case binary regardless of the configured chunk
        # size (currently == _MAX_SCRIPT_BYTES, one round trip for
        # any in-limit script — see the chunk-size header comment).
        length = run_mod._STAGE_CHUNK_BYTES * 3 + run_mod._STAGE_CHUNK_BYTES // 2
        payload = (bytes(range(256)) * (length // 256 + 2))[:length]
        link = self._stage(payload)
        pastes = [w for w in link.writes if b"f.write" in w]
        self.assertEqual(len(pastes), 4)
        for pkt in pastes:
            self.assertLess(len(pkt), run_mod._STAGE_CHUNK_BYTES * 5)

    @staticmethod
    def _decode_staged_writes(writes, hub_name=None):
        """Recover the payload bytes each staged program would write —
        the exact inverse a hub named ``hub_name`` applies. Handles
        BOTH framings: plain ``f.write(<repr>)`` and the sealed
        deflate + hub-name-keyed XOR + base64 form."""
        import base64
        import zlib
        hub_name = _HUB if hub_name is None else hub_name
        chunks = []
        for w in writes:
            if b"a2b_base64(" in w:
                nonce = eval(w.split(b"_sn = ", 1)[1].split(b"\n", 1)[0])
                lit = w.split(b"a2b_base64(", 1)[1].split(b")", 1)[0]
                sealed = base64.b64decode(eval(lit))
                comp = run_mod._keystream_xor(
                    sealed, hub_name.encode(), nonce)
                chunks.append(zlib.decompress(comp))
            elif b"f.write(" in w:
                lit = w.split(b"f.write(", 1)[1].rsplit(b")", 1)[0]
                chunks.append(eval(lit))
        return b"".join(chunks)

    def test_payload_reassembles_in_order(self):
        payload = bytes(range(256)) * 5   # 1280 bytes, 1 chunk (< current
                                          # chunk size) — reassembly must
                                          # still hold with a single round.
        link = self._stage(payload)
        self.assertEqual(self._decode_staged_writes(link.writes), payload)

    def test_small_payload_stages_plain(self):
        # Below _COMPRESS_MIN_BYTES (interactive -c snippets) the
        # import + b64/zlib overhead isn't worth it: plain repr framing,
        # no hub-side deflate dependency exercised.
        payload = b"print(1)\n" * 10   # 90 bytes
        self.assertLess(len(payload), run_mod._COMPRESS_MIN_BYTES)
        link = self._stage(payload)
        writes = b"".join(link.writes)
        self.assertNotIn(b"deflate", writes)
        self.assertEqual(self._decode_staged_writes(link.writes), payload)

    def test_large_payload_stages_compressed_and_smaller(self):
        # At/above the threshold the wire program must carry the
        # sealed (deflate + keyed XOR + b64) form and actually BE
        # smaller than the raw payload for realistic compressible
        # source (the whole point: staging is paced by the 128-byte
        # raw-paste window, so bytes on the wire ≈ time).
        payload = (b"# a comment line that compresses well \xe2\x80\x94 yes\n"
                   * 200)   # ~9 KB, unicode em-dash included
        link = self._stage(payload)
        program = next(w for w in link.writes if b"a2b_base64(" in w)
        self.assertIn(b"deflate.DeflateIO", program)
        self.assertIn(b"deflate.ZLIB", program)
        self.assertIn(b"openbricks.HUB_NAME", program)
        self.assertLess(len(program), len(payload) // 2)
        self.assertEqual(self._decode_staged_writes(link.writes), payload)

    def test_sealed_payload_only_decodes_with_the_addressed_name(self):
        # The binding property the hub-name key exists for: a hub
        # carrying a DIFFERENT name derives a different keystream and
        # the deflate step blows up — a mis-targeted staging fails
        # loudly instead of silently landing on the wrong robot.
        # (NOT confidentiality: the name is public in every BLE
        # advertisement — see the _XOR_BLOCK comment.)
        import zlib
        payload = b"print('hello')\n" * 100
        link = self._stage(payload)
        self.assertEqual(
            self._decode_staged_writes(link.writes, hub_name=_HUB),
            payload)
        with self.assertRaises(zlib.error):
            self._decode_staged_writes(link.writes, hub_name="intruder")

    def test_nonce_varies_between_stagings(self):
        payload = b"x" * 2048
        w1 = next(w for w in self._stage(payload).writes
                  if b"_sn = " in w)
        w2 = next(w for w in self._stage(payload).writes
                  if b"_sn = " in w)
        n1 = w1.split(b"_sn = ", 1)[1].split(b"\n", 1)[0]
        n2 = w2.split(b"_sn = ", 1)[1].split(b"\n", 1)[0]
        self.assertNotEqual(n1, n2)

    def test_compression_threshold_boundary(self):
        at = self._stage(b"x" * run_mod._COMPRESS_MIN_BYTES)
        below = self._stage(b"x" * (run_mod._COMPRESS_MIN_BYTES - 1))
        self.assertIn(b"deflate", b"".join(at.writes))
        self.assertNotIn(b"deflate", b"".join(below.writes))

    def test_hub_stderr_during_staging_raises_runerror(self):
        # e.g. OSError: 28 (filesystem full) while writing a chunk —
        # must surface, not stage on.
        responses = [
            _R_SUPPORTED + _WINDOW_8K, _CTRL_D,
            _CTRL_D,                                   # empty stdout
            b"OSError: 28\r\n" + _CTRL_D,             # stderr
            b">",
        ]
        link = _ProtocolLink(responses)
        blink = run_mod._BufferedLink(link)
        try:
            _drive(run_mod._stage_file(
                blink, link, "/program.py", b"data", _HUB))
        except run_mod.RunError as e:
            self.assertIn("OSError: 28", str(e))
        else:
            self.fail("expected RunError")

    def test_junk_instead_of_prompt_raises(self):
        # Framing desync (junk where the raw-REPL prompt should be)
        # must fail loudly, not stage the next chunk into the void.
        responses = [
            _R_SUPPORTED + _WINDOW_8K, _CTRL_D,
            _CTRL_D, _CTRL_D,
            b"?",                                      # not the prompt
        ]
        link = _ProtocolLink(responses)
        blink = run_mod._BufferedLink(link)
        try:
            _drive(run_mod._stage_file(
                blink, link, "/program.py", b"data", _HUB))
        except run_mod.RunError as e:
            self.assertIn("unexpected byte", str(e))
        else:
            self.fail("expected RunError")

    def test_empty_payload_still_truncates_the_file(self):
        # Uploading an empty script must leave an empty /program.py,
        # not yesterday's program.
        link = self._stage(b"")
        writes = b"".join(link.writes)
        self.assertIn(b"'wb'", writes)


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


def setUpModule():
    # The verified idle-restore waits real seconds for the hub banner;
    # against scripted silent links that's pure sleep. Shrink for the
    # whole module, restore after.
    import openbricks_dev.run as _rm
    global _ORIG_RESTORE_WAIT
    _ORIG_RESTORE_WAIT = _rm._RESTORE_WAIT_S
    _rm._RESTORE_WAIT_S = 0.02


class _CancelCountingTask:
    def __init__(self):
        self.cancels = 0

    def cancel(self):
        self.cancels += 1


class SigintRoutingTests(unittest.TestCase):
    """Ctrl-C routing: first press cancels the session task, repeats
    are absorbed, and loops without signal-handler support fall back
    to the raw-KeyboardInterrupt path."""

    def test_first_press_cancels_the_session_task(self):
        task = _CancelCountingTask()
        handler = run_mod._make_sigint_handler(task)
        handler()
        self.assertEqual(task.cancels, 1)

    def test_repeat_presses_acknowledge_without_cancelling_again(self):
        task = _CancelCountingTask()
        handler = run_mod._make_sigint_handler(task)
        stderr = io.StringIO()
        with patch("sys.stderr", stderr):
            handler()
            handler()
            handler()
        self.assertEqual(task.cancels, 1)
        self.assertIn("stopping", stderr.getvalue())

    def test_install_registers_sigint_on_a_capable_loop(self):
        registered = []

        class _Loop:
            def add_signal_handler(self, sig, cb):
                registered.append((sig, cb))

        def handler():
            pass

        self.assertTrue(run_mod._install_sigint(_Loop(), handler))
        self.assertEqual(registered, [(signal.SIGINT, handler)])

    def test_install_reports_false_where_loop_lacks_support(self):
        class _WindowsLoop:
            def add_signal_handler(self, sig, cb):
                raise NotImplementedError

        self.assertFalse(
            run_mod._install_sigint(_WindowsLoop(), lambda: None))


def tearDownModule():
    import openbricks_dev.run as _rm
    _rm._RESTORE_WAIT_S = _ORIG_RESTORE_WAIT


if __name__ == "__main__":
    unittest.main()
