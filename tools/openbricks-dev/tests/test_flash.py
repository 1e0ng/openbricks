# SPDX-License-Identifier: MIT
"""Tests for ``openbricks_dev.flash`` with esptool / mpremote mocked out.

We don't touch real hardware here — the test asserts the *shape* of the
commands flash.run composes (esptool args, mpremote NVS snippet) and
the verification flow (write, read back, compare).
"""

import argparse
import unittest
from unittest.mock import patch, MagicMock

from openbricks_dev import flash


def _args(**overrides):
    """Return an argparse.Namespace with sensible defaults for ``flash.run``."""
    base = dict(
        name="RobotA",
        port="/dev/ttyUSB0",
        firmware="firmware.bin",
        chip="auto",
        baud="460800",
        skip_erase=False,
    )
    base.update(overrides)
    return argparse.Namespace(**base)


class ValidateNameTests(unittest.TestCase):
    def test_empty_name_raises(self):
        with self.assertRaises(flash.FlashError):
            flash._validate_name("")

    def test_name_too_long_raises(self):
        with self.assertRaises(flash.FlashError):
            flash._validate_name("x" * 30)

    def test_name_with_nul_raises(self):
        with self.assertRaises(flash.FlashError):
            flash._validate_name("ok\x00bad")

    def test_name_just_under_limit_is_ok(self):
        # 29 bytes is the GAP cap — should be accepted.
        flash._validate_name("x" * 29)


class RunHappyPathTests(unittest.TestCase):
    """The full flash flow with every subprocess call stubbed.

    Verifies: erase_flash runs, write_flash runs with the expected args,
    mpremote writes the name blob, readback matches, and the final reset
    is attempted.
    """

    def setUp(self):
        # Pretend esptool / mpremote are both on PATH.
        self._which = patch("shutil.which",
                            side_effect=lambda name: "/usr/local/bin/" + name)
        self._which.start()
        # Never actually sleep in tests.
        self._sleep = patch("openbricks_dev.flash.time.sleep")
        self._sleep.start()

    def tearDown(self):
        self._which.stop()
        self._sleep.stop()

    def test_full_flow_success(self):
        # subprocess.call for erase_flash / write_flash / final reset.
        call_history = []

        def _fake_call(cmd):
            call_history.append(cmd)
            return 0

        # subprocess.run for every mpremote exec: first the wait-for-repl
        # probe, then the name write, then the readback. Readback output
        # must match the name we wrote.
        run_responses = iter([
            MagicMock(returncode=0, stdout="ok\n", stderr=""),
            MagicMock(returncode=0, stdout="wrote: 'RobotA'\n", stderr=""),
            MagicMock(returncode=0, stdout="RobotA\n", stderr=""),
        ])

        def _fake_run(cmd, capture_output=True, text=True):
            return next(run_responses)

        with patch("subprocess.call", side_effect=_fake_call), \
             patch("subprocess.run", side_effect=_fake_run):
            rc = flash.run(_args())

        self.assertEqual(rc, 0)
        # Three subprocess.call invocations: erase, write, reset.
        self.assertEqual(len(call_history), 3)
        # First: erase_flash.
        self.assertIn("erase_flash", call_history[0])
        # Second: write_flash at 0x0 with the firmware path.
        self.assertIn("write_flash", call_history[1])
        self.assertIn("0x0", call_history[1])
        self.assertIn("firmware.bin", call_history[1])
        # Third: mpremote reset at the end.
        self.assertIn("reset", call_history[2])

    def test_skip_erase_drops_erase_flash(self):
        call_history = []
        run_responses = iter([
            MagicMock(returncode=0, stdout="ok\n", stderr=""),
            MagicMock(returncode=0, stdout="wrote: 'RobotA'\n", stderr=""),
            MagicMock(returncode=0, stdout="RobotA\n", stderr=""),
        ])
        with patch("subprocess.call",
                   side_effect=lambda cmd: call_history.append(cmd) or 0), \
             patch("subprocess.run", side_effect=lambda *a, **k: next(run_responses)):
            flash.run(_args(skip_erase=True))
        self.assertEqual(len(call_history), 2)
        self.assertNotIn("erase_flash", call_history[0])

    def test_readback_mismatch_raises(self):
        run_responses = iter([
            MagicMock(returncode=0, stdout="ok\n", stderr=""),
            MagicMock(returncode=0, stdout="wrote: 'RobotA'\n", stderr=""),
            # Readback returns something different: simulate flash corruption.
            MagicMock(returncode=0, stdout="RobotB\n", stderr=""),
        ])
        with patch("subprocess.call", return_value=0), \
             patch("subprocess.run", side_effect=lambda *a, **k: next(run_responses)):
            with self.assertRaises(flash.FlashError) as ctx:
                flash.run(_args())
        self.assertIn("verification failed", str(ctx.exception))

    def test_write_flash_failure_raises(self):
        # erase_flash succeeds (rc=0); write_flash fails (rc=3).
        returncodes = iter([0, 3])
        with patch("subprocess.call",
                   side_effect=lambda cmd: next(returncodes)), \
             patch("subprocess.run"):
            with self.assertRaises(flash.FlashError) as ctx:
                flash.run(_args())
        self.assertIn("command failed", str(ctx.exception))


class ToolMissingTests(unittest.TestCase):
    def test_missing_esptool_raises(self):
        with patch("shutil.which", return_value=None):
            with self.assertRaises(flash.FlashError) as ctx:
                flash.run(_args())
        self.assertIn("esptool.py not found", str(ctx.exception))


class NameWriteSnippetTests(unittest.TestCase):
    """The mpremote ``exec`` snippet must embed the name in a form that
    ``openbricks._read_hub_name`` will accept back (bytes via set_blob)."""

    def test_snippet_uses_set_blob_with_bytes_name(self):
        captured = {}

        def _fake_run(cmd, capture_output=True, text=True):
            # cmd is ``[mpremote, connect, PORT, exec, <snippet>]``.
            captured["snippet"] = cmd[-1]
            return MagicMock(returncode=0, stdout="wrote: 'RobotA'\n", stderr="")

        with patch("subprocess.run", side_effect=_fake_run):
            flash._write_hub_name("/usr/local/bin/mpremote", "/dev/ttyUSB0", "RobotA")

        snippet = captured["snippet"]
        # Namespace + key come from the same constants openbricks reads.
        self.assertIn("'openbricks'", snippet)
        self.assertIn("'hub_name'", snippet)
        # The value is passed as a bytes literal, not str — otherwise
        # ``esp32.NVS.set_blob`` would TypeError on the hub.
        self.assertIn("b'RobotA'", snippet)
        self.assertIn("set_blob", snippet)
        self.assertIn("commit", snippet)


if __name__ == "__main__":
    unittest.main()
