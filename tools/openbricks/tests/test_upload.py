# SPDX-License-Identifier: MIT
"""Tests for ``openbricks_dev.upload``.

``upload`` stages a script at ``/program.py`` on the hub. The user
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

from openbricks_dev import upload as ul
from openbricks_dev._nus import NUSError


def _args(
    name="RobotA",
    script="s.py",
    path=None,
    scan_timeout=5.0,
):
    return argparse.Namespace(
        name=name, script=script, path=path, scan_timeout=scan_timeout,
    )


class ComposeTests(unittest.TestCase):
    """Pure-function tests for the post-staging confirm program.
    (The payload itself is staged chunked by ``run_mod._stage_file``
    — see test_run.py's ComposeTests for the chunk composer.)"""

    def test_confirm_prints_actual_on_hub_size(self):
        prog = ul._compose_confirm_program("/program.py", 2)
        self.assertIn(b"print('uploaded', os.stat('/program.py')[6]", prog)

    def test_confirm_asserts_expected_size(self):
        # A dropped staging chunk must not pass silently: the confirm
        # program cross-checks the on-hub size against what we sent.
        prog = ul._compose_confirm_program("/program.py", 1234)
        self.assertIn(b"== 1234", prog)
        self.assertIn(b"size mismatch", prog)

    def test_confirm_syncs_rtc(self):
        prog = ul._compose_confirm_program("/program.py", 1)
        self.assertIn(b"machine.RTC().datetime(", prog)

    def test_custom_path_surfaces_in_program(self):
        prog = ul._compose_confirm_program("/user/alt.py", 1)
        self.assertIn(b"'/user/alt.py'", prog)

    def test_confirm_carries_no_payload(self):
        # The confirm program is fixed-size: the payload must never be
        # embedded (that was the one-shot design that died on a
        # fragmented heap).
        prog = ul._compose_confirm_program("/program.py", 10_000)
        self.assertLess(len(prog), 600)
        self.assertNotIn(b"f.write", prog)

    def test_does_not_issue_machine_reset(self):
        """Auto-reset would run user code immediately — the launcher
        workflow requires a manual button press to start, so the
        upload program MUST NOT call machine.reset()."""
        prog = ul._compose_confirm_program("/program.py", 1)
        self.assertNotIn(b"machine.reset", prog)


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
        if timeout == 0:
            # Non-blocking stale-drain (the raw-paste handshake's
            # pre-request sweep): a real hub has sent nothing ahead
            # of the request unless a test staged it.
            stale = getattr(self, "stale", None)
            if stale:
                return stale.pop(0)
            return b""
        if self._responses:
            return self._responses.pop(0)
        return b""

    async def close(self):
        self.closed = True


_IDLE = b"openbricks: idle. Press button to run /program.mpy\r\n"


class UploadFlowTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False)
        self.tmp.write("print('hello')\n")
        self.tmp.close()
        self.addCleanup(os.unlink, self.tmp.name)
        # a private hub cache: every test starts with an unknown hub
        # unless it remembers one itself
        self.cache = tempfile.mkdtemp()
        env = patch.dict(os.environ, {"OPENBRICKS_CACHE_DIR": self.cache})
        env.start()
        self.addCleanup(env.stop)

    def _standard_responses(self, stdout_msg, fw_version=None,
                            probe=True, idle=_IDLE):
        """The hub side of one upload: the raw-REPL banner, the
        firmware probe (only for an unknown hub in the default flow),
        then the ONE staged program's paste ack and its stream — the
        version line, the confirmation, and the launcher's idle
        banner (the program never ends)."""
        out = [_BANNER]
        if fw_version is not None and probe:
            out += [
                _R_SUPPORTED + _WINDOW_8K,
                _CTRL_D,                  # end-of-paste ack
                fw_version + _CTRL_D,     # stdout: fwv line + EOT
                _CTRL_D,                  # empty stderr + EOT
                b">",                     # raw-REPL prompt
            ]
        version_line = fw_version if fw_version is not None else b"fwv=4.10.0\r\n"
        out += [
            _R_SUPPORTED + _WINDOW_8K,    # the staged program's paste ack
            _CTRL_D,                      # end-of-paste ack
            version_line,                 # its first line
            stdout_msg,                   # the confirmation
            idle,                         # the idle loop's banner
        ]
        return out

    def test_default_flow_stages_compiled_mpy_without_reset(self):
        fake = _ScriptedLink(self._standard_responses(
            b"uploaded 59 bytes to '/program.mpy'\r\n",
            fw_version=b"fwv=1.92.0\r\n"))

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return fake

        with patch.object(ul.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO) as out, \
             patch("sys.stderr", new_callable=io.StringIO) as err:
            rc = ul.run(_args(script=self.tmp.name))

        self.assertEqual(rc, 0)
        joined = b"".join(fake.writes)
        # No reset should be issued anywhere in the upload.
        self.assertNotIn(b"machine.reset", joined)
        # Raw REPL fully entered and cleanly exited.
        self.assertIn(b"\x01", joined)      # Ctrl-A (enter raw)
        self.assertIn(b"\x05A\x01", joined) # raw-paste request
        self.assertIn(b"launcher.run()", joined)  # the program ends in the idle loop
        self.assertEqual(joined.count(b"\x05A\x01"), 2, "probe + ONE staged program")
        # 1.92.0 firmware: the COMPILED payload lands at /program.mpy
        # (.mpy header inside the staged repr), and the confirm
        # program clears the stale source sibling.
        self.assertIn(b"'/program.mpy'", joined)
        self.assertIn(b"M\\x06", joined)
        self.assertIn(b"os.remove('/program.py')", joined)
        self.assertTrue(fake.closed)
        # Confirmation printed to the user's terminal, and the final
        # ready-line names the path that was ACTUALLY staged.
        self.assertIn("uploaded", out.getvalue())
        self.assertNotIn("fwv=", out.getvalue(), "the version line is not the user's")
        self.assertIn("/program.mpy", err.getvalue())
        self.assertIn("staged in ", err.getvalue())
        # the hub is remembered: the next upload skips the probe
        from openbricks_dev import _hubcache
        self.assertEqual(_hubcache.firmware_version("RobotA"), (1, 92, 0))

    def test_a_remembered_hub_skips_the_probe(self):
        from openbricks_dev import _hubcache
        _hubcache.remember_firmware("RobotA", "1.92.0")
        fake = _ScriptedLink(self._standard_responses(
            b"uploaded 59 bytes to '/program.mpy'\r\n",
            fw_version=b"fwv=1.92.0\r\n", probe=False))

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return fake

        with patch.object(ul.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO) as out, \
             patch("sys.stderr", new_callable=io.StringIO):
            rc = ul.run(_args(script=self.tmp.name))
        self.assertEqual(rc, 0)
        joined = b"".join(fake.writes)
        self.assertEqual(joined.count(b"\x05A\x01"), 1, "no probe: ONE exec")
        self.assertIn(b"'/program.mpy'", joined)
        self.assertIn(b"fwneedsrc", joined, "the staged program guards the .mpy itself")
        self.assertIn("uploaded", out.getvalue())

    def test_a_re_flashed_hub_is_caught_in_session_and_gets_source(self):
        # The cache says 1.92.0 but the hub now runs 1.91.1: the staged
        # program's guard refuses the .mpy, the host reads the version
        # line, consumes the refusal, announces, and stages source.
        from openbricks_dev import _hubcache
        _hubcache.remember_firmware("RobotA", "1.92.0")
        responses = [
            _BANNER,
            _R_SUPPORTED + _WINDOW_8K, _CTRL_D,     # the .mpy program
            b"fwv=1.91.1\r\n" + _CTRL_D,             # its stdout: the version, then the end
            b"Traceback (most recent call last):\r\nAssertionError: fwneedsrc\r\n" + _CTRL_D,
            b">",
            _R_SUPPORTED + _WINDOW_8K, _CTRL_D,     # the source program
            b"fwv=1.91.1\r\n",
            b"uploaded 15 bytes to '/program.py'\r\n",
            b"openbricks: idle. Press button to run /program.py\r\n",
        ]
        fake = _ScriptedLink(responses)

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return fake

        with patch.object(ul.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO) as out, \
             patch("sys.stderr", new_callable=io.StringIO) as err:
            rc = ul.run(_args(script=self.tmp.name))
        self.assertEqual(rc, 0)
        joined = b"".join(fake.writes)
        self.assertEqual(joined.count(b"\x05A\x01"), 2)
        self.assertIn(b"'/program.mpy'", joined)
        self.assertIn(b"'/program.py'", joined)
        self.assertIn("predates precompiled", err.getvalue())
        self.assertIn("uploaded 15 bytes", out.getvalue())
        self.assertEqual(_hubcache.firmware_version("RobotA"), (1, 91, 1))

    def test_a_hub_error_before_the_idle_loop_is_an_upload_error(self):
        from openbricks_dev import _hubcache
        _hubcache.remember_firmware("RobotA", "1.92.0")
        responses = [
            _BANNER,
            _R_SUPPORTED + _WINDOW_8K, _CTRL_D,
            b"fwv=1.92.0\r\n",
            b"uploaded 3 bytes to '/program.mpy'\r\n" + _CTRL_D,
            b"AssertionError: size mismatch after staging\r\n" + _CTRL_D + b">",
            # the restore that follows: banner
            b"openbricks: idle. Press button to run /program.mpy\r\n",
        ]
        fake = _ScriptedLink(responses)

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return fake

        with patch.object(ul.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stderr", new_callable=io.StringIO):
            with self.assertRaises(ul.UploadError) as ctx:
                asyncio.run(ul._upload_async("RobotA", self.tmp.name, None, 5.0))
        self.assertIn("size mismatch", str(ctx.exception))
        joined = b"".join(fake.writes)
        self.assertEqual(joined.count(b"launcher.run()"), 2, "the idle loop is restored after the failure")
        self.assertTrue(fake.closed)

    def test_default_flow_old_firmware_stages_source_with_notice(self):
        fake = _ScriptedLink(self._standard_responses(
            b"uploaded 15 bytes to '/program.py'\r\n",
            fw_version=b"fwv=1.91.1\r\n",
            idle=b"openbricks: idle. Press button to run /program.py\r\n"))

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return fake

        with patch.object(ul.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO), \
             patch("sys.stderr", new_callable=io.StringIO) as err:
            rc = ul.run(_args(script=self.tmp.name))

        self.assertEqual(rc, 0)
        joined = b"".join(fake.writes)
        # Source crossed the link, to /program.py, no sibling delete;
        # the downgrade is ANNOUNCED, never silent.
        self.assertIn(b"'/program.py'", joined)
        self.assertIn(b"hello", joined)
        self.assertNotIn(b"'/program.mpy'", joined)
        self.assertNotIn(b"os.remove", joined)
        self.assertIn("predates precompiled", err.getvalue())

    def test_custom_path_flag_stages_verbatim(self):
        fake = _ScriptedLink(self._standard_responses(
            b"uploaded 15 bytes to '/main.py'\r\n",
            idle=b"openbricks: idle. Press button to run /program.mpy\r\n"))

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return fake

        with patch.object(ul.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stdout", new_callable=io.StringIO):
            rc = ul.run(_args(script=self.tmp.name, path="/main.py"))

        self.assertEqual(rc, 0)
        joined = b"".join(fake.writes)
        self.assertIn(b"'/main.py'", joined)
        # Custom boot flows get the file AS-IS: source bytes, no
        # compile, no firmware-version probe, no .mpy guard.
        self.assertIn(b"hello", joined)
        self.assertEqual(joined.count(b"\x05A\x01"), 1, "no probe: ONE exec")
        self.assertNotIn(b"fwneedsrc", joined)
        self.assertNotIn(b"os.remove", joined)

    def test_missing_script_raises_without_touching_ble(self):
        with patch.object(ul.NUSLink, "connect") as connect:
            with self.assertRaises(ul.UploadError) as ctx:
                asyncio.run(ul._upload_async(
                    "RobotA", "/nonexistent.py", "/program.py", 5.0))
        connect.assert_not_called()
        self.assertIn("cannot read script", str(ctx.exception))

    def test_oversized_script_raises(self):
        big = tempfile.NamedTemporaryFile(mode="wb", suffix=".py", delete=False)
        big.write(b"x" * (ul._MAX_SCRIPT_BYTES + 1))
        big.close()
        self.addCleanup(os.unlink, big.name)

        with patch.object(ul.NUSLink, "connect") as connect:
            with self.assertRaises(ul.UploadError) as ctx:
                asyncio.run(ul._upload_async(
                    "RobotA", big.name, "/program.py", 5.0))
        connect.assert_not_called()
        self.assertIn("soft limit", str(ctx.exception))

    def test_connect_failure_propagates_as_upload_error(self):
        async def _raise(name, scan_timeout=5.0, debug=False):
            raise NUSError("no hub named 'Ghost'")

        with patch.object(ul.NUSLink, "connect", side_effect=_raise):
            with self.assertRaises(ul.UploadError):
                asyncio.run(ul._upload_async(
                    "Ghost", self.tmp.name, "/program.py", 5.0))




class UploadRestoreFailureTests(UploadFlowTests):
    def test_raising_idle_restore_is_swallowed(self):
        # _restore_idle_loop is the fallback when the staged program
        # never reached the idle loop; it failing must not mask the
        # real error. A successful upload never calls it at all.
        from openbricks_dev import _hubcache
        _hubcache.remember_firmware("RobotA", "1.92.0")
        fake = _ScriptedLink([
            _BANNER,
            _R_SUPPORTED + _WINDOW_8K, _CTRL_D,
            b"fwv=1.92.0\r\n" + _CTRL_D,
            b"OSError: [Errno 28] ENOSPC\r\n" + _CTRL_D + b">",
        ])
        restores = []

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return fake

        async def _bad_restore(link):
            restores.append(1)
            raise RuntimeError("hub hung up first")

        with patch.object(ul.NUSLink, "connect", side_effect=_fake_connect), \
             patch.object(ul.run_mod, "_restore_idle_loop", _bad_restore), \
             patch("sys.stdout", new_callable=io.StringIO), \
             patch("sys.stderr", new_callable=io.StringIO):
            with self.assertRaises(ul.UploadError) as ctx:
                asyncio.run(ul._upload_async("RobotA", self.tmp.name, None, 5.0))
        self.assertIn("ENOSPC", str(ctx.exception))
        self.assertEqual(restores, [1])
        self.assertTrue(fake.closed)

    def test_a_silent_hub_after_the_paste_is_a_timeout_error(self):
        from openbricks_dev import _hubcache
        _hubcache.remember_firmware("RobotA", "1.92.0")
        fake = _ScriptedLink([
            _BANNER,
            _R_SUPPORTED + _WINDOW_8K, _CTRL_D,
            b"fwv=1.92.0\r\n",
            # then nothing: every read is empty
        ])
        fake.stats = lambda: {"connected": True, "notify_count": 3, "byte_count": 30,
                              "last_byte_ago": 1.0, "uptime": 2.0}

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return fake

        with patch.object(ul.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stderr", new_callable=io.StringIO):
            with self.assertRaises(ul.UploadError) as ctx:
                asyncio.run(ul._upload_async("RobotA", self.tmp.name, None, 5.0))
        self.assertIn("waiting for the upload confirmation", str(ctx.exception))

    def test_a_hub_that_never_stops_talking_hits_the_deadline(self):
        from openbricks_dev import _hubcache
        _hubcache.remember_firmware("RobotA", "1.92.0")

        class _Chatty(_ScriptedLink):
            async def read(self, timeout=None):
                if timeout == 0:
                    return b""
                if self._responses:
                    return self._responses.pop(0)
                return b"still printing\r\n"

        fake = _Chatty([_BANNER, _R_SUPPORTED + _WINDOW_8K, _CTRL_D, b"fwv=1.92.0\r\n"])
        fake.stats = lambda: {"connected": True, "notify_count": 3, "byte_count": 30,
                              "last_byte_ago": 0.0, "uptime": 2.0}
        clock = [1000.0]

        def _monotonic():
            clock[0] += 3.0      # every look at the clock is 3 s later
            return clock[0]

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return fake

        with patch.object(ul.NUSLink, "connect", side_effect=_fake_connect), \
             patch.object(ul.time, "monotonic", _monotonic), \
             patch("sys.stderr", new_callable=io.StringIO):
            with self.assertRaises(ul.UploadError):
                asyncio.run(ul._upload_async("RobotA", self.tmp.name, None, 5.0))

    def test_a_program_that_does_not_report_its_version_is_an_error(self):
        from openbricks_dev import _hubcache
        _hubcache.remember_firmware("RobotA", "1.92.0")
        fake = _ScriptedLink([
            _BANNER,
            _R_SUPPORTED + _WINDOW_8K, _CTRL_D,
            b"something else entirely\r\n",
        ])

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            return fake

        with patch.object(ul.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stderr", new_callable=io.StringIO):
            with self.assertRaises(ul.run_mod.RunError) as ctx:
                asyncio.run(ul._upload_async("RobotA", self.tmp.name, None, 5.0))
        self.assertIn("did not report the firmware version", str(ctx.exception))


class UploadInterruptTests(unittest.TestCase):
    def test_upload_error_propagates_to_the_cli_layer(self):
        import argparse
        orig = ul._upload_async

        def _boom(*a, **k):
            raise ul.UploadError("no hub named X")
        ul._upload_async = _boom
        try:
            with self.assertRaises(ul.UploadError):
                ul.run(argparse.Namespace(
                    name="X", script="s.py", path=None,
                    scan_timeout=1.0))
        finally:
            ul._upload_async = orig

    def test_ctrl_c_maps_to_130(self):
        import argparse
        orig = ul._upload_async

        def _boom(*a, **k):
            raise KeyboardInterrupt()
        ul._upload_async = _boom
        try:
            rc = ul.run(argparse.Namespace(
                name="X", script="s.py", path="/program.py",
                scan_timeout=1.0))
        finally:
            ul._upload_async = orig
        self.assertEqual(rc, 130)


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



class UploadGuardTests(unittest.TestCase):
    """3.10.0: a second transfer to a hub another process is still
    transferring to is refused at once, before any scan."""

    def setUp(self):
        from openbricks_dev import _uplock
        self.dir = tempfile.mkdtemp()
        p = patch.object(_uplock, "LOCK_DIR", self.dir)
        p.start()
        self.addCleanup(p.stop)
        self.tmp = tempfile.NamedTemporaryFile("w", suffix=".py",
                                               delete=False)
        self.tmp.write("print('x')\n")
        self.tmp.close()
        self.addCleanup(os.unlink, self.tmp.name)

    def test_second_upload_while_a_transfer_is_ongoing_fails_at_once(self):
        from openbricks_dev import _uplock
        connects = []

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            connects.append(name)
            raise AssertionError("must not scan while an upload is ongoing")

        with _uplock.UploadLock("RobotA"), \
             patch.object(ul.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stderr", new_callable=io.StringIO):
            with self.assertRaises(ul.UploadError) as cm:
                ul.run(argparse.Namespace(
                    name="RobotA", script=self.tmp.name, path=None,
                    scan_timeout=5.0))
        self.assertTrue(str(cm.exception).startswith("an upload is ongoing"),
                        cm.exception)
        self.assertIn("'RobotA'", str(cm.exception))
        self.assertEqual(connects, [])

    def test_a_failed_connect_releases_the_lock(self):
        from openbricks_dev import _uplock

        async def _fake_connect(name, scan_timeout=5.0, debug=False):
            raise NUSError("hub not found")

        with patch.object(ul.NUSLink, "connect", side_effect=_fake_connect), \
             patch("sys.stderr", new_callable=io.StringIO):
            with self.assertRaises(ul.UploadError):
                ul.run(argparse.Namespace(
                    name="RobotA", script=self.tmp.name, path=None,
                    scan_timeout=5.0))
        with _uplock.UploadLock("RobotA"):
            pass


if __name__ == "__main__":
    unittest.main()
