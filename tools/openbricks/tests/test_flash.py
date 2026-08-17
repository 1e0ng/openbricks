# SPDX-License-Identifier: MIT
"""Tests for ``openbricks_dev.flash`` with esptool / mpremote mocked out.

We don't touch real hardware here — the test asserts the *shape* of the
commands flash.run composes (esptool args, mpremote NVS snippet) and
the verification flow (write, read back, compare).
"""

import argparse
import io
import os
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch, MagicMock

from openbricks_dev import flash


def _fake_image(base):
    """Write a minimal merged-image lookalike to a temp file and return
    its path: 0xFF padding with the partition-table magic at file offset
    ``0x8000 - base`` — exactly how MicroPython's makeimg.py lays out an
    image built for flash address ``base``."""
    buf = bytearray(b"\xff" * (flash._PT_FLASH_OFFSET + 2))
    off = flash._PT_FLASH_OFFSET - base
    buf[off:off + 2] = flash._PT_MAGIC
    fd, path = tempfile.mkstemp(suffix=".bin")
    with os.fdopen(fd, "wb") as f:
        f.write(buf)
    return path


def _args(**overrides):
    """Return an argparse.Namespace with sensible defaults for ``flash.run``."""
    base = dict(
        name="RobotA",
        port="/dev/ttyUSB0",
        firmware="firmware.bin",
        chip="auto",
        baud="460800",
        skip_erase=False,
        yes=False,
        verbose=False,
    )
    base.update(overrides)
    return argparse.Namespace(**base)


class _FakePort:
    def __init__(self, device, vid):
        self.device = device
        self.vid = vid


class AutodetectPortTests(unittest.TestCase):
    """--port omitted: exactly one connected ESP-ish device is used;
    zero or several refuse (flashing is destructive — never guess)."""

    def _detect(self, ports):
        import serial.tools.list_ports as lp
        with patch.object(lp, "comports", return_value=ports):
            return flash._autodetect_port()

    def test_single_esp_device_is_chosen(self):
        got = self._detect([
            _FakePort("/dev/cu.Bluetooth", None),
            _FakePort("/dev/cu.usbmodem42", 0x303A),
        ])
        self.assertEqual(got, "/dev/cu.usbmodem42")

    def test_bridge_chips_count_as_esp(self):
        got = self._detect([_FakePort("/dev/ttyUSB0", 0x10C4)])
        self.assertEqual(got, "/dev/ttyUSB0")

    def test_no_device_dies_with_hint(self):
        with self.assertRaises(flash.FlashError):
            self._detect([_FakePort("/dev/cu.Bluetooth", None)])

    def test_two_devices_refuse_to_guess(self):
        with self.assertRaises(flash.FlashError):
            self._detect([
                _FakePort("/dev/cu.usbmodem42", 0x303A),
                _FakePort("/dev/ttyUSB0", 0x1A86),
            ])


class DetectChipTests(unittest.TestCase):
    def _detect(self, stdout, binary="/usr/local/bin/esptool"):
        fake = subprocess.CompletedProcess([], 0, stdout=stdout, stderr="")
        with patch("openbricks_dev.flash.subprocess.run",
                   return_value=fake) as run:
            got = flash._detect_chip(binary, "/dev/x")
        return got, run.call_args[0][0]

    def test_parses_esptool_v5_column_format(self):
        # REAL v5.3.1 output (bench-verified): column-padded
        # "Chip type:" — the 1.22.0 parser only knew v4's "Chip is"
        # and silently failed on every v5 install.
        got, cmd = self._detect(
            "esptool v5.3.1\nConnecting...\n"
            "Chip type:          ESP32-S3 (QFN56) (revision v0.2)\n")
        self.assertEqual(got, "esp32s3")
        self.assertIn("chip-id", cmd)

    def test_parses_esptool_v4_chip_is_format(self):
        got, _ = self._detect("Detecting chip type... Chip is ESP32-S3 (QFN56)")
        self.assertEqual(got, "esp32s3")

    def test_parses_classic_esp32(self):
        got, _ = self._detect(
            "Chip type:          ESP32-D0WD-V3 (revision v3.1)")
        # Variant suffixes (D0WD, PICO...) are NOT chip families —
        # esptool --chip needs plain "esp32", not "esp32d0wd".
        self.assertEqual(got, "esp32")

    def test_legacy_binary_uses_snake_case_command(self):
        _, cmd = self._detect("Chip is ESP32", binary="/usr/bin/esptool.py")
        self.assertIn("chip_id", cmd)

    def test_unparseable_output_warns_with_last_line(self):
        import io, sys
        err = io.StringIO()
        orig, sys.stderr = sys.stderr, err
        try:
            got, _ = self._detect("A fatal error occurred: no sync")
        finally:
            sys.stderr = orig
        self.assertIsNone(got)
        self.assertIn("no sync", err.getvalue())

    def test_unparseable_output_returns_none(self):
        got, _ = self._detect("garbage")
        self.assertIsNone(got)


class ImageChipNameTests(unittest.TestCase):
    def _image(self, chip_id, magic=0xE9):
        hdr = bytearray(16)
        hdr[0] = magic
        hdr[12] = chip_id & 0xFF
        hdr[13] = (chip_id >> 8) & 0xFF
        fd, path = tempfile.mkstemp(suffix=".bin")
        with os.fdopen(fd, "wb") as f:
            f.write(bytes(hdr))
        self.addCleanup(os.unlink, path)
        return path

    def test_reads_s3_and_classic(self):
        self.assertEqual(flash._image_chip_name(self._image(0x0009)), "esp32s3")
        self.assertEqual(flash._image_chip_name(self._image(0x0000)), "esp32")

    def test_unknown_id_and_bad_magic_return_none(self):
        self.assertIsNone(flash._image_chip_name(self._image(0x4242)))
        self.assertIsNone(flash._image_chip_name(self._image(0x0009, magic=0x00)))


class LatestFirmwareDownloadTests(unittest.TestCase):
    """--firmware omitted: pick the newest release's asset for the
    detected chip, verify size, cache under ~/.cache/openbricks."""

    def setUp(self):
        self.cache = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.cache, True)
        self._cache_patch = patch.object(
            flash, "_FIRMWARE_CACHE_DIR", self.cache)
        self._cache_patch.start()
        self.addCleanup(self._cache_patch.stop)

    def _release_json(self):
        import json
        return json.dumps({
            "tag_name": "v9.9.9",
            "assets": [
                {"name": "openbricks-esp32-firmware-v9.9.9.bin",
                 "size": 4, "browser_download_url": "http://x/esp32"},
                {"name": "openbricks-esp32s3-firmware-v9.9.9.bin",
                 "size": 4, "browser_download_url": "http://x/s3"},
            ],
        }).encode()

    def test_downloads_matching_chip_asset(self):
        fetched = []

        def fake_get(url):
            fetched.append(url)
            if "releases" in url:
                return self._release_json()
            return b"BINS"
        with patch.object(flash, "_http_get", side_effect=fake_get):
            path, version = flash._latest_firmware_for("esp32s3")
        self.assertTrue(path.endswith("openbricks-esp32s3-firmware-v9.9.9.bin"))
        self.assertEqual(version, "9.9.9")
        self.assertEqual(open(path, "rb").read(), b"BINS")
        self.assertIn("http://x/s3", fetched)

    def test_esp32_prefix_does_not_match_s3_asset(self):
        def fake_get(url):
            if "releases" in url:
                return self._release_json()
            return b"BINS"
        with patch.object(flash, "_http_get", side_effect=fake_get):
            path, _version = flash._latest_firmware_for("esp32")
        self.assertTrue(path.endswith("openbricks-esp32-firmware-v9.9.9.bin"))

    def test_signature_asset_downloads_next_to_the_image(self):
        import json
        release = json.dumps({
            "tag_name": "v9.9.9",
            "assets": [
                {"name": "openbricks-esp32s3-firmware-v9.9.9.bin",
                 "size": 4, "browser_download_url": "http://x/s3"},
                {"name": "openbricks-esp32s3-firmware-v9.9.9.bin.sig",
                 "size": 3, "browser_download_url": "http://x/s3sig"},
            ],
        }).encode()

        def fake_get(url):
            if "releases" in url:
                return release
            if url.endswith("s3sig"):
                return b"SIG"
            return b"BINS"
        with patch.object(flash, "_http_get", side_effect=fake_get):
            path, _version = flash._latest_firmware_for("esp32s3")
        self.assertEqual(flash._read_sig_for(path), b"SIG")

    def test_release_without_signature_reads_none(self):
        def fake_get(url):
            if "releases" in url:
                return self._release_json()
            return b"BINS"
        with patch.object(flash, "_http_get", side_effect=fake_get):
            path, _version = flash._latest_firmware_for("esp32s3")
        self.assertIsNone(flash._read_sig_for(path))

    def test_cached_file_skips_download(self):
        name = "openbricks-esp32s3-firmware-v9.9.9.bin"
        with open(os.path.join(self.cache, name), "wb") as f:
            f.write(b"OLDB")   # size 4 matches the asset
        fetched = []

        def fake_get(url):
            fetched.append(url)
            return self._release_json()
        with patch.object(flash, "_http_get", side_effect=fake_get):
            flash._latest_firmware_for("esp32s3")
        self.assertEqual(len(fetched), 1)   # only the release lookup

    def test_truncated_download_dies(self):
        def fake_get(url):
            if "releases" in url:
                return self._release_json()
            return b"X"   # 1 byte, asset says 4
        with patch.object(flash, "_http_get", side_effect=fake_get):
            with self.assertRaises(flash.FlashError):
                flash._latest_firmware_for("esp32s3")

    def test_offline_lookup_dies_with_instructions(self):
        with patch.object(flash, "_http_get",
                          side_effect=OSError("no route")):
            with self.assertRaises(flash.FlashError):
                flash._latest_firmware_for("esp32s3")


class AutodetectErrorPathTests(unittest.TestCase):
    """The loud-degradation branches: pyserial missing, probe raising,
    unreadable image, missing asset, download-stage failure."""

    def test_missing_pyserial_dies_with_instructions(self):
        import sys
        with patch.dict(sys.modules, {"serial": None,
                                      "serial.tools": None,
                                      "serial.tools.list_ports": None}):
            with self.assertRaises(flash.FlashError):
                flash._autodetect_port()

    def test_probe_subprocess_exception_returns_none(self):
        with patch("openbricks_dev.flash.subprocess.run",
                   side_effect=OSError("no such port")):
            self.assertIsNone(
                flash._detect_chip("/usr/local/bin/esptool", "/dev/x"))

    def test_unreadable_image_returns_none(self):
        self.assertIsNone(
            flash._image_chip_name("/nonexistent/firmware.bin"))

    def test_http_get_uses_urllib_with_user_agent(self):
        import urllib.request
        resp = MagicMock()
        resp.__enter__ = lambda s: s
        resp.__exit__ = lambda s, *a: False
        resp.read = lambda: b"BODY"
        with patch.object(urllib.request, "urlopen",
                          return_value=resp) as uo:
            self.assertEqual(flash._http_get("http://x"), b"BODY")
        req = uo.call_args[0][0]
        self.assertEqual(req.get_header("User-agent"), "openbricks-flash")

    def test_release_without_matching_asset_dies(self):
        import json
        release = json.dumps({"tag_name": "v1", "assets": [
            {"name": "something-else.bin", "size": 1,
             "browser_download_url": "http://x/other"}]}).encode()
        with patch.object(flash, "_http_get", return_value=release):
            with self.assertRaises(flash.FlashError):
                flash._latest_firmware_for("esp32s3")

    def test_download_stage_failure_dies(self):
        import json
        release = json.dumps({"tag_name": "v1", "assets": [
            {"name": "openbricks-esp32s3-firmware-v1.bin", "size": 4,
             "browser_download_url": "http://x/s3"}]}).encode()
        with patch.object(flash, "_http_get",
                          side_effect=[release, OSError("reset")]):
            with self.assertRaises(flash.FlashError):
                flash._latest_firmware_for("esp32s3")


class RunAutodetectFlowTests(unittest.TestCase):
    """run() dispatch: port=None triggers autodetect, firmware=None
    triggers download (or dies on unknown chip), detected chip feeds
    esptool --chip on the happy path."""

    def setUp(self):
        self._which = patch("shutil.which",
                            side_effect=lambda name: "/usr/local/bin/" + name)
        self._which.start()
        self.addCleanup(self._which.stop)
        self._sleep = patch("openbricks_dev.flash.time.sleep")
        self._sleep.start()
        self.addCleanup(self._sleep.stop)

    def _s3_image(self):
        buf = bytearray(b"\xff" * (flash._PT_FLASH_OFFSET + 2))
        buf[0] = flash._IMAGE_MAGIC
        buf[12] = 0x09
        buf[13] = 0x00
        buf[flash._PT_FLASH_OFFSET:flash._PT_FLASH_OFFSET + 2] = flash._PT_MAGIC
        fd, path = tempfile.mkstemp(suffix=".bin")
        with os.fdopen(fd, "wb") as f:
            f.write(bytes(buf))
        self.addCleanup(os.unlink, path)
        return path

    def test_port_none_uses_autodetect(self):
        with patch.object(flash, "_autodetect_port",
                          return_value="/dev/cu.usbmodem42") as ad, \
             patch.object(flash, "_detect_chip", return_value=None), \
             patch("subprocess.run",
                   return_value=MagicMock(returncode=1, stdout="",
                                          stderr="")), \
             patch("subprocess.call", return_value=1):
            with self.assertRaises(flash.FlashError):
                # erase failing (return 1) aborts right after — we only
                # care that autodetect ran and its port was used.
                flash.run(_args(port=None, firmware=self._s3_image()))
        ad.assert_called_once()

    def test_firmware_none_unknown_chip_dies(self):
        with patch.object(flash, "_detect_chip", return_value=None), \
             patch("subprocess.run",
                   return_value=MagicMock(returncode=1, stdout="",
                                          stderr="")):
            with self.assertRaises(flash.FlashError):
                flash.run(_args(firmware=None, port="/dev/x"))

    def test_firmware_none_downloads_for_detected_chip(self):
        img = self._s3_image()
        with patch.object(flash, "_detect_chip", return_value="esp32s3"), \
             patch.object(flash, "_latest_firmware_for",
                          return_value=(img, "9.9.9")) as dl, \
             patch("subprocess.run",
                   return_value=MagicMock(returncode=1, stdout="",
                                          stderr="")), \
             patch("subprocess.call", return_value=1):
            with self.assertRaises(flash.FlashError):
                flash.run(_args(firmware=None, port="/dev/x"))
        dl.assert_called_once_with("esp32s3")

    def test_detected_chip_feeds_esptool_chip_arg(self):
        call_history = []

        def _fake_call(cmd):
            call_history.append(cmd)
            return 0
        run_responses = iter([
            MagicMock(returncode=1, stdout="", stderr=""),   # preflight probe
            MagicMock(returncode=0, stdout="ok\n", stderr=""),
            MagicMock(returncode=0, stdout="wrote: 'RobotA'\n", stderr=""),
            MagicMock(returncode=0, stdout="RobotA\n", stderr=""),
            MagicMock(returncode=0, stdout="9.9.9\n", stderr=""),
            MagicMock(returncode=0, stdout="marker: 9.9.9:customized\n",
                      stderr=""),
        ])
        with patch.object(flash, "_detect_chip",
                          return_value="esp32s3"), \
             patch("subprocess.call", side_effect=_fake_call), \
             patch("subprocess.run",
                   side_effect=lambda *a, **k: next(run_responses)):
            rc = flash.run(_args(firmware=self._s3_image(), port="/dev/x"))
        self.assertEqual(rc, 0)
        self.assertIn("esp32s3", call_history[0])   # erase --chip
        self.assertIn("esp32s3", call_history[1])   # write --chip


class ChipMismatchGuardTests(unittest.TestCase):
    """An esp32 image on an S3 chip (or vice versa) must die BEFORE
    the erase."""

    def setUp(self):
        self._which = patch("shutil.which",
                            side_effect=lambda name: "/usr/local/bin/" + name)
        self._which.start()
        self.addCleanup(self._which.stop)
        self._sleep = patch("openbricks_dev.flash.time.sleep")
        self._sleep.start()
        self.addCleanup(self._sleep.stop)

    def _s3_image_with_header(self):
        # PT magic for offset resolution + a bootloader header saying
        # esp32s3 at the file start.
        buf = bytearray(b"\xff" * (flash._PT_FLASH_OFFSET + 2))
        buf[0] = flash._IMAGE_MAGIC
        buf[12] = 0x09
        buf[13] = 0x00   # chip_id high byte — 0xFF fill would make 0xFF09
        buf[flash._PT_FLASH_OFFSET:flash._PT_FLASH_OFFSET + 2] = flash._PT_MAGIC
        fd, path = tempfile.mkstemp(suffix=".bin")
        with os.fdopen(fd, "wb") as f:
            f.write(buf)
        self.addCleanup(os.unlink, path)
        return path

    def test_mismatch_dies_before_erase(self):
        calls = []
        fake_probe = subprocess.CompletedProcess(
            [], 0, stdout="Chip type:          ESP32-D0WD (revision v3.1)",
            stderr="")

        def fake_call(cmd):
            calls.append(cmd)
            return 0
        with patch("openbricks_dev.flash.subprocess.run",
                   return_value=fake_probe), \
             patch("openbricks_dev.flash.subprocess.call",
                   side_effect=fake_call):
            with self.assertRaises(flash.FlashError):
                flash.run(_args(firmware=self._s3_image_with_header(),
                                port="/dev/x"))
        for cmd in calls:
            self.assertFalse(any("erase" in str(c) for c in cmd),
                             "erase ran before the mismatch check")


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
        # The chip probe (new in the autodetect feature) would invoke
        # real esptool; these tests exercise the flash flow, not the
        # probe — force the "unknown chip" path (chip stays 'auto',
        # image check skipped, exactly the pre-autodetect behaviour).
        self._probe = patch("openbricks_dev.flash._detect_chip",
                            return_value=None)
        self._probe.start()
        # A real (fake) image file on disk — flash.run reads it to
        # determine the write offset. S3 layout → offset 0x0.
        self.firmware = _fake_image(0x0)

    def tearDown(self):
        self._which.stop()
        self._sleep.stop()
        self._probe.stop()
        os.unlink(self.firmware)

    def _run_args(self, **overrides):
        overrides.setdefault("firmware", self.firmware)
        return _args(**overrides)

    @staticmethod
    def _responses(readback="RobotA"):
        """The mpremote exec sequence run() drives: the preflight
        version probe (chip not yet running openbricks — fails),
        wait-for-repl, name write, name readback, marker version
        read, marker write."""
        return iter([
            MagicMock(returncode=1, stdout="", stderr="no repl"),
            MagicMock(returncode=0, stdout="ok\n", stderr=""),
            MagicMock(returncode=0, stdout="wrote: 'RobotA'\n", stderr=""),
            MagicMock(returncode=0, stdout=readback + "\n", stderr=""),
            MagicMock(returncode=0, stdout="9.9.9\n", stderr=""),
            MagicMock(returncode=0, stdout="marker: 9.9.9:customized\n",
                      stderr=""),
        ])

    def test_full_flow_with_qtr_init_writes_the_cal(self):
        # --with-qtr-init inserts one mpremote exec between the name
        # readback and the fw marker: the /qtr.cal write+verify.
        snippets = []

        def _fake_call(cmd):
            return 0

        run_responses = iter([
            MagicMock(returncode=1, stdout="", stderr="no repl"),
            MagicMock(returncode=0, stdout="ok\n", stderr=""),
            MagicMock(returncode=0, stdout="wrote: 'RobotA'\n", stderr=""),
            MagicMock(returncode=0, stdout="RobotA\n", stderr=""),
            MagicMock(returncode=0, stdout="qtr-cal-ok 10 10\n", stderr=""),
            MagicMock(returncode=0, stdout="9.9.9\n", stderr=""),
            MagicMock(returncode=0, stdout="marker: 9.9.9:customized\n",
                      stderr=""),
        ])

        def _fake_run(cmd, capture_output=True, text=True, timeout=None):
            snippets.append(cmd[-1])
            return next(run_responses)

        with patch("subprocess.call", side_effect=_fake_call), \
             patch("subprocess.run", side_effect=_fake_run):
            rc = flash.run(self._run_args(with_qtr_init=True))

        self.assertEqual(rc, 0)
        self.assertTrue(any("'/qtr.cal'" in sn for sn in snippets),
                        "qtr cal write never composed")

    def test_full_flow_success(self):
        # subprocess.call for erase_flash / write_flash / final reset.
        call_history = []

        def _fake_call(cmd):
            call_history.append(cmd)
            return 0

        run_responses = self._responses()

        def _fake_run(cmd, capture_output=True, text=True, timeout=None):
            return next(run_responses)

        with patch("subprocess.call", side_effect=_fake_call), \
             patch("subprocess.run", side_effect=_fake_run):
            rc = flash.run(self._run_args())

        self.assertEqual(rc, 0)
        # Three subprocess.call invocations: erase, write, reset.
        self.assertEqual(len(call_history), 3)
        # First: erase-flash (esptool v5 kebab-case form).
        self.assertIn("erase-flash", call_history[0])
        # Second: write-flash at 0x0 (S3-layout image) with the firmware path.
        self.assertIn("write-flash", call_history[1])
        self.assertIn("0x0", call_history[1])
        self.assertIn(self.firmware, call_history[1])
        # Third: mpremote-driven hardware reset at the end. Since 0.10.17
        # this is ``resume exec --no-follow machine.reset()`` rather than
        # the ``mpremote reset`` alias (which would soft-reset the chip
        # into the BLE-active runtime first, then fail to re-enter raw
        # REPL); test both shapes by looking for ``machine.reset()`` in
        # the snippet args.
        self.assertTrue(
            any("machine.reset()" in a for a in call_history[2]) or
            "reset" in call_history[2],
            "expected an mpremote-driven reset; got %r" % call_history[2])

    def test_erasing_flash_says_what_was_lost(self):
        # A silent loss is a bug: the full-chip erase wipes the
        # staged program and saved calibrations, and the next button
        # press then does NOTHING with no log and no visible reason
        # (bench 2026-08-14). The done message must say so and name
        # the re-stage command.
        run_responses = self._responses()
        buf = io.StringIO()
        with patch("subprocess.call", side_effect=lambda cmd: 0), \
             patch("subprocess.run",
                   side_effect=lambda *a, **k: next(run_responses)), \
             patch("sys.stdout", buf):
            flash.run(self._run_args())
        out = buf.getvalue()
        self.assertIn("erased the hub's filesystem", out)
        self.assertIn("openbricks upload", out)
        self.assertIn("-n RobotA", out)

    def test_skip_erase_flash_omits_the_wipe_note(self):
        run_responses = self._responses()
        buf = io.StringIO()
        with patch("subprocess.call", side_effect=lambda cmd: 0), \
             patch("subprocess.run",
                   side_effect=lambda *a, **k: next(run_responses)), \
             patch("sys.stdout", buf):
            flash.run(self._run_args(skip_erase=True))
        self.assertNotIn("erased", buf.getvalue())

    def test_skip_erase_drops_erase_flash(self):
        call_history = []
        run_responses = self._responses()
        with patch("subprocess.call",
                   side_effect=lambda cmd: call_history.append(cmd) or 0), \
             patch("subprocess.run", side_effect=lambda *a, **k: next(run_responses)):
            flash.run(self._run_args(skip_erase=True))
        self.assertEqual(len(call_history), 2)
        self.assertNotIn("erase-flash", call_history[0])

    def test_readback_mismatch_raises(self):
        # Readback returns something different: simulate flash corruption.
        run_responses = self._responses(readback="RobotB")
        with patch("subprocess.call", return_value=0), \
             patch("subprocess.run", side_effect=lambda *a, **k: next(run_responses)):
            with self.assertRaises(flash.FlashError) as ctx:
                flash.run(self._run_args())
        self.assertIn("verification failed", str(ctx.exception))

    def test_write_flash_failure_raises(self):
        # erase-flash succeeds (rc=0); write-flash fails (rc=3).
        returncodes = iter([0, 3])
        with patch("subprocess.call",
                   side_effect=lambda cmd: next(returncodes)), \
             patch("subprocess.run"):
            with self.assertRaises(flash.FlashError) as ctx:
                flash.run(self._run_args())
        self.assertIn("command failed", str(ctx.exception))

    def test_classic_esp32_image_writes_at_0x1000(self):
        """The classic-ESP32 merged image starts at flash 0x1000; writing
        it at 0x0 (the pre-0.10.22 behavior) puts the bootloader where
        the ROM never looks and the board can't boot."""
        classic = _fake_image(0x1000)
        self.addCleanup(os.unlink, classic)
        call_history = []
        run_responses = self._responses()
        with patch("subprocess.call",
                   side_effect=lambda cmd: call_history.append(cmd) or 0), \
             patch("subprocess.run", side_effect=lambda *a, **k: next(run_responses)):
            flash.run(self._run_args(firmware=classic))
        self.assertIn("write-flash", call_history[1])
        self.assertIn("0x1000", call_history[1])
        self.assertNotIn("0x0", call_history[1])

    def test_unrecognizable_image_fails_before_erase(self):
        """No partition-table magic at either candidate offset → refuse
        to flash, and refuse *before* the erase wipes the chip."""
        fd, bogus = tempfile.mkstemp(suffix=".bin")
        with os.fdopen(fd, "wb") as f:
            f.write(b"\xff" * (flash._PT_FLASH_OFFSET + 2))
        self.addCleanup(os.unlink, bogus)
        call_history = []
        with patch("subprocess.call",
                   side_effect=lambda cmd: call_history.append(cmd) or 0), \
             patch("subprocess.run"):
            with self.assertRaises(flash.FlashError) as ctx:
                flash.run(self._run_args(firmware=bogus))
        self.assertIn("cannot determine the flash offset", str(ctx.exception))
        self.assertEqual(call_history, [],
                         "no esptool command may run before the image "
                         "layout is understood")


class ImageBaseOffsetTests(unittest.TestCase):
    """``_image_base_offset`` derives the write address from the image."""

    def test_s3_layout_gives_0x0(self):
        path = _fake_image(0x0)
        self.addCleanup(os.unlink, path)
        self.assertEqual(flash._image_base_offset(path), "0x0")

    def test_classic_layout_gives_0x1000(self):
        path = _fake_image(0x1000)
        self.addCleanup(os.unlink, path)
        self.assertEqual(flash._image_base_offset(path), "0x1000")

    def test_ambiguous_image_raises(self):
        # Magic at both candidate offsets — refuse to guess.
        buf = bytearray(b"\xff" * (flash._PT_FLASH_OFFSET + 2))
        for base in flash._IMAGE_BASES:
            off = flash._PT_FLASH_OFFSET - base
            buf[off:off + 2] = flash._PT_MAGIC
        fd, path = tempfile.mkstemp(suffix=".bin")
        with os.fdopen(fd, "wb") as f:
            f.write(buf)
        self.addCleanup(os.unlink, path)
        with self.assertRaises(flash.FlashError):
            flash._image_base_offset(path)

    def test_missing_file_raises(self):
        with self.assertRaises(flash.FlashError) as ctx:
            flash._image_base_offset("/nonexistent/firmware.bin")
        self.assertIn("cannot read firmware image", str(ctx.exception))

    def test_truncated_image_raises(self):
        # Shorter than the partition-table region → neither offset can
        # match; must raise, not IndexError.
        fd, path = tempfile.mkstemp(suffix=".bin")
        with os.fdopen(fd, "wb") as f:
            f.write(b"\xff" * 64)
        self.addCleanup(os.unlink, path)
        with self.assertRaises(flash.FlashError):
            flash._image_base_offset(path)


class ToolMissingTests(unittest.TestCase):
    def test_missing_esptool_raises(self):
        with patch("shutil.which", return_value=None):
            with self.assertRaises(flash.FlashError) as ctx:
                flash.run(_args())
        self.assertIn("esptool not found", str(ctx.exception))


class QtrStarterCalTests(unittest.TestCase):
    """``flash --with-qtr-init`` stores the reference-bench starter
    calibration at /qtr.cal in exactly the format
    ``QTRArray.save_calibration`` writes — pins list included, since
    ``load_calibration`` refuses a file recorded for other wiring."""

    def test_snippet_writes_valid_cal_json(self):
        import json as _json
        captured = {}

        def _fake_run(cmd, capture_output=True, text=True, timeout=None):
            captured["snippet"] = cmd[-1]
            return MagicMock(returncode=0,
                             stdout="qtr-cal-ok 10 10\n", stderr="")

        with patch("subprocess.run", side_effect=_fake_run):
            flash._write_qtr_starter_cal("mpremote", "/dev/ttyUSB0")

        snippet = captured["snippet"]
        self.assertIn("'/qtr.cal'", snippet)
        # The staged payload is the repr of a JSON document with the
        # save_calibration schema: 10 pins, 10 mins, 10 maxes, and
        # every span positive (min < max) — a starter file that fails
        # load_calibration would be worse than no file.
        start = snippet.index("f.write(") + len("f.write(")
        payload_repr = snippet[start:snippet.index("); f.close()")]
        payload = eval(payload_repr)   # repr of a str literal
        data = _json.loads(payload)
        self.assertEqual(data["pins"], list(range(1, 11)))
        self.assertEqual(len(data["min"]), 10)
        self.assertEqual(len(data["max"]), 10)
        for lo, hi in zip(data["min"], data["max"]):
            self.assertTrue(0 < lo < hi <= 65535, (lo, hi))

    def test_verification_failure_dies(self):
        def _fake_run(cmd, capture_output=True, text=True, timeout=None):
            return MagicMock(returncode=0, stdout="garbage\n", stderr="")

        with patch("subprocess.run", side_effect=_fake_run):
            try:
                flash._write_qtr_starter_cal("mpremote", "/p")
            except flash.FlashError as e:
                self.assertIn("starter QTR calibration", str(e))
            else:
                self.fail("must die on unverified write")


class NameWriteSnippetTests(unittest.TestCase):
    """The mpremote ``exec`` snippet must embed the name in a form that
    ``openbricks._read_hub_name`` will accept back (bytes via set_blob)."""

    def test_snippet_uses_set_blob_with_bytes_name(self):
        captured = {}

        def _fake_run(cmd, capture_output=True, text=True, timeout=None):
            # cmd ends in ``... exec <snippet>``; the chain in front
            # is mpremote-version-dependent (``connect PORT [resume]``).
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




class ToolDiscoveryTests(unittest.TestCase):
    def test_require_tool_found(self):
        with patch("shutil.which", return_value="/usr/bin/mpremote"):
            self.assertEqual(flash._require_tool("mpremote"),
                             "/usr/bin/mpremote")

    def test_require_tool_missing_dies_with_hint(self):
        with patch("shutil.which", return_value=None):
            try:
                flash._require_tool("mpremote")
            except flash.FlashError as e:
                self.assertIn("pip install esptool mpremote", str(e))
            else:
                self.fail("expected FlashError")

    def test_esptool_v5_names_preferred(self):
        with patch("shutil.which",
                   side_effect=lambda n: "/x/esptool"
                   if n == "esptool" else None):
            path, wr, er = flash._esptool_paths_and_commands()
        self.assertEqual((path, wr, er),
                         ("/x/esptool", "write-flash", "erase-flash"))

    def test_esptool_v4_fallback_names(self):
        with patch("shutil.which",
                   side_effect=lambda n: "/x/esptool.py"
                   if n == "esptool.py" else None):
            path, wr, er = flash._esptool_paths_and_commands()
        self.assertEqual((path, wr, er),
                         ("/x/esptool.py", "write_flash", "erase_flash"))

    def test_no_esptool_dies(self):
        with patch("shutil.which", return_value=None):
            with self.assertRaises(flash.FlashError):
                flash._esptool_paths_and_commands()


class NvsRoundTripFailureTests(unittest.TestCase):
    def _patch_exec(self, rc, out="", err=""):
        orig = flash._mpremote_exec
        flash._mpremote_exec = lambda m, p, s: (rc, out, err)
        self.addCleanup(setattr, flash, "_mpremote_exec", orig)

    def test_write_hub_name_failure_dies(self):
        self._patch_exec(1, err="nvs write blew up")
        try:
            flash._write_hub_name("mpremote", "/p", "ls")
        except flash.FlashError as e:
            self.assertIn("failed to write hub name", str(e))
            self.assertIn("nvs write blew up", str(e))
        else:
            self.fail("expected FlashError")

    def test_read_hub_name_failure_dies(self):
        self._patch_exec(1, err="nvs read blew up")
        try:
            flash._read_hub_name("mpremote", "/p")
        except flash.FlashError as e:
            self.assertIn("failed to read hub name back", str(e))
        else:
            self.fail("expected FlashError")

    def test_wait_for_repl_retries_until_ok(self):
        results = [(1, "", "busy"), (0, "ok", "")]
        orig = flash._mpremote_exec
        flash._mpremote_exec = lambda m, p, s: results.pop(0)
        self.addCleanup(setattr, flash, "_mpremote_exec", orig)
        with patch("time.sleep"):
            flash._wait_for_repl("mpremote", "/p", timeout_s=30)
        self.assertEqual(results, [])   # both polls consumed

    def test_wait_for_repl_timeout_reports_last_output(self):
        self._patch_exec(1, out="", err="port busy")
        try:
            flash._wait_for_repl("mpremote", "/p", timeout_s=0)
        except flash.FlashError as e:
            self.assertIn("timed out waiting for device REPL", str(e))
        else:
            self.fail("expected FlashError")


class VersionParseTests(unittest.TestCase):
    def test_tag_and_filename_forms(self):
        self.assertEqual(flash._parse_version_text("v1.77.1"), "1.77.1")
        self.assertEqual(flash._parse_version_text("1.77.1"), "1.77.1")
        self.assertEqual(
            flash._parse_version_text(
                "openbricks-esp32s3-firmware-v1.77.1.bin"),
            "1.77.1")

    def test_no_version_returns_none(self):
        self.assertIsNone(flash._parse_version_text("firmware.bin"))
        self.assertIsNone(flash._parse_version_text(""))
        self.assertIsNone(flash._parse_version_text(None))

    def test_version_tuple_orders_numerically(self):
        # String comparison would put 1.9.0 above 1.10.0.
        self.assertTrue(flash._version_tuple("1.10.0")
                        > flash._version_tuple("1.9.0"))
        self.assertEqual(flash._version_tuple("1.77.1"), (1, 77, 1))


class ConfirmTests(unittest.TestCase):
    def test_assume_yes_skips_the_prompt(self):
        def no_input(prompt):
            raise AssertionError("input() must not be called")
        self.assertTrue(flash._confirm("q?", True, input_fn=no_input))

    def test_tty_yes_and_no(self):
        with patch.object(flash.sys.stdin, "isatty", return_value=True):
            self.assertTrue(
                flash._confirm("q?", False, input_fn=lambda p: "y"))
            self.assertTrue(
                flash._confirm("q?", False, input_fn=lambda p: "YES"))
            self.assertFalse(
                flash._confirm("q?", False, input_fn=lambda p: ""))
            self.assertFalse(
                flash._confirm("q?", False, input_fn=lambda p: "n"))

    def test_non_interactive_stdin_dies_instead_of_hanging(self):
        with patch.object(flash.sys.stdin, "isatty", return_value=False):
            with self.assertRaises(flash.FlashError) as ctx:
                flash._confirm("q?", False,
                               input_fn=lambda p: "y")
        self.assertIn("--yes", str(ctx.exception))


class CurrentFirmwareProbeTests(unittest.TestCase):
    def _patch_exec(self, rc, out="", err=""):
        orig = flash._mpremote_exec
        flash._mpremote_exec = lambda m, p, s: (rc, out, err)
        self.addCleanup(setattr, flash, "_mpremote_exec", orig)

    def test_unreachable_repl_is_unknown(self):
        self._patch_exec(1)
        with patch("sys.stderr", new_callable=io.StringIO):
            self.assertEqual(
                flash._read_current_firmware("mpremote", "/p"),
                (None, None))

    def test_failed_probe_names_the_reason_on_stderr(self):
        # "could not enter raw repl" (hub state) and "failed to access
        # /dev/..." (host port contention) are DIFFERENT bugs; hiding
        # mpremote's message behind a generic "unknown" cost a bench
        # round-trip to tell them apart (2026-08-13).
        self._patch_exec(
            1, err="mpremote: failed to access /dev/cu.usbmodemX "
                   "(it may be in use by another program)\n")
        with patch("sys.stderr", new_callable=io.StringIO) as err:
            flash._read_current_firmware("mpremote", "/p")
        self.assertIn("probe: mpremote rc=1", err.getvalue())
        self.assertIn("failed to access", err.getvalue())

    def test_failed_probe_without_output_still_reports_rc(self):
        self._patch_exec(2)
        with patch("sys.stderr", new_callable=io.StringIO) as err:
            flash._read_current_firmware("mpremote", "/p")
        self.assertEqual(err.getvalue().strip(), "probe: mpremote rc=2")

    def test_failed_probe_falls_back_to_stdout_line(self):
        self._patch_exec(1, out="noise\ncould not enter raw repl\n")
        with patch("sys.stderr", new_callable=io.StringIO) as err:
            flash._read_current_firmware("mpremote", "/p")
        self.assertIn("could not enter raw repl", err.getvalue())

    def test_matching_official_marker(self):
        self._patch_exec(0, "ver=1.77.1\nsig=1.77.1:official\n")
        self.assertEqual(
            flash._read_current_firmware("mpremote", "/p"),
            ("1.77.1", "official"))

    def test_missing_marker_is_customized(self):
        self._patch_exec(0, "ver=1.77.1\nsig=\n")
        self.assertEqual(
            flash._read_current_firmware("mpremote", "/p"),
            ("1.77.1", "customized"))

    def test_stale_marker_version_is_customized(self):
        # Firmware replaced behind the CLI's back: the marker still
        # says an older version was official.
        self._patch_exec(0, "ver=1.77.1\nsig=1.70.0:official\n")
        self.assertEqual(
            flash._read_current_firmware("mpremote", "/p"),
            ("1.77.1", "customized"))

    def test_no_openbricks_import_is_unknown(self):
        self._patch_exec(0, "ver=\nsig=\n")
        self.assertEqual(
            flash._read_current_firmware("mpremote", "/p"),
            (None, None))


class VerboseOutputTests(unittest.TestCase):
    def test_vprint_silent_by_default_loud_when_enabled(self):
        import contextlib
        import io
        orig = flash._verbose
        try:
            flash._verbose = False
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                flash._vprint(">>> plumbing")
            self.assertEqual(out.getvalue(), "")
            flash._verbose = True
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                flash._vprint(">>> plumbing")
            self.assertIn(">>> plumbing", out.getvalue())
        finally:
            flash._verbose = orig


class MarkerWriteTests(unittest.TestCase):
    def test_marker_embeds_chip_version_and_verdict(self):
        snippets = []

        def fake_exec(m, p, s):
            snippets.append(s)
            if "openbricks.__version__" in s:
                return 0, "9.9.9\n", ""
            return 0, "marker: 9.9.9:official\n", ""
        orig = flash._mpremote_exec
        flash._mpremote_exec = fake_exec
        self.addCleanup(setattr, flash, "_mpremote_exec", orig)
        flash._write_fw_marker("mpremote", "/p", "official")
        self.assertIn("b'9.9.9:official'", snippets[-1])
        self.assertIn("'fw_sig'", snippets[-1])
        self.assertIn("set_blob", snippets[-1])
        self.assertIn("commit", snippets[-1])

    def test_marker_write_failure_dies(self):
        # A provenance marker that failed to land must be loud: the
        # next `openbricks flash` would otherwise read the previous
        # firmware's verdict as the current one.
        def fake_exec(m, p, s):
            if "openbricks.__version__" in s:
                return 0, "9.9.9\n", ""
            return 1, "", "NVS write refused"
        orig = flash._mpremote_exec
        flash._mpremote_exec = fake_exec
        self.addCleanup(setattr, flash, "_mpremote_exec", orig)
        with self.assertRaises(flash.FlashError) as ctx:
            flash._write_fw_marker("mpremote", "/p", "official")
        self.assertIn("provenance marker", str(ctx.exception))
        self.assertIn("NVS write refused", str(ctx.exception))

    def test_mpremote_exec_timeout_is_a_failure_not_a_hang(self):
        def fake_run(cmd, capture_output=True, text=True, timeout=None):
            raise flash.subprocess.TimeoutExpired(cmd, timeout)
        orig = flash.subprocess.run
        flash.subprocess.run = fake_run
        self.addCleanup(setattr, flash.subprocess, "run", orig)
        rc, out, err = flash._mpremote_exec("mpremote", "/p", "1+1")
        self.assertEqual(rc, -1)
        self.assertEqual(out, "")
        self.assertIn("timed out", err)

    def test_foreign_firmware_skips_the_marker(self):
        calls = []

        def fake_exec(m, p, s):
            calls.append(s)
            return 1, "", "ImportError"
        orig = flash._mpremote_exec
        flash._mpremote_exec = fake_exec
        self.addCleanup(setattr, flash, "_mpremote_exec", orig)
        flash._write_fw_marker("mpremote", "/p", "customized")
        self.assertEqual(len(calls), 1)   # only the version probe ran


class DowngradeConfirmFlowTests(unittest.TestCase):
    """run() with a running 9.9.9 and an older/equal target must ask
    before touching the chip; a declined prompt flashes nothing."""

    def setUp(self):
        self._which = patch("shutil.which",
                            side_effect=lambda name: "/usr/local/bin/" + name)
        self._which.start()
        self.addCleanup(self._which.stop)
        self._sleep = patch("openbricks_dev.flash.time.sleep")
        self._sleep.start()
        self.addCleanup(self._sleep.stop)
        self._probe = patch("openbricks_dev.flash._detect_chip",
                            return_value=None)
        self._probe.start()
        self.addCleanup(self._probe.stop)
        # A versioned firmware file name so the target version parses.
        src = _fake_image(0x0)
        self.firmware = os.path.join(
            os.path.dirname(src),
            "openbricks-esp32s3-firmware-v1.0.0-%s.bin"
            % os.path.basename(src))
        os.rename(src, self.firmware)
        self.addCleanup(os.unlink, self.firmware)

    def _run(self, confirm_answer):
        calls = []
        with patch.object(flash, "_read_current_firmware",
                          return_value=("9.9.9", "official")), \
             patch.object(flash, "_confirm",
                          return_value=confirm_answer) as confirm, \
             patch("subprocess.call",
                   side_effect=lambda cmd: calls.append(cmd) or 0), \
             patch("subprocess.run",
                   return_value=MagicMock(returncode=1, stdout="",
                                          stderr="")):
            rc = flash.run(_args(firmware=self.firmware, port="/dev/x"))
        return rc, calls, confirm

    def test_declined_downgrade_flashes_nothing(self):
        rc, calls, confirm = self._run(confirm_answer=False)
        self.assertEqual(rc, 0)
        self.assertEqual(calls, [])
        confirm.assert_called_once()
        self.assertIn("OLDER", confirm.call_args[0][0])

    def test_accepted_downgrade_proceeds_to_esptool(self):
        calls = []
        with patch.object(flash, "_read_current_firmware",
                          return_value=("9.9.9", "official")), \
             patch.object(flash, "_confirm", return_value=True), \
             patch.object(flash, "_wait_for_repl",
                          side_effect=flash.FlashError("no repl")), \
             patch("subprocess.call",
                   side_effect=lambda cmd: calls.append(cmd) or 0), \
             patch("subprocess.run",
                   return_value=MagicMock(returncode=1, stdout="",
                                          stderr="")):
            with self.assertRaises(flash.FlashError):
                flash.run(_args(firmware=self.firmware, port="/dev/x"))
        # erase + write both ran before the (stubbed) REPL wait died.
        self.assertEqual(len(calls), 2)
        self.assertTrue(any("write-flash" in c for c in calls[1]))

    def test_same_version_asks_to_reinstall(self):
        with patch.object(flash, "_read_current_firmware",
                          return_value=("1.0.0", "customized")), \
             patch.object(flash, "_confirm",
                          return_value=False) as confirm, \
             patch("subprocess.call", return_value=0), \
             patch("subprocess.run",
                   return_value=MagicMock(returncode=1, stdout="",
                                          stderr="")):
            rc = flash.run(_args(firmware=self.firmware, port="/dev/x"))
        self.assertEqual(rc, 0)
        self.assertIn("SAME version", confirm.call_args[0][0])

    def test_newer_target_does_not_prompt(self):
        with patch.object(flash, "_read_current_firmware",
                          return_value=("0.1.0", "official")), \
             patch.object(flash, "_confirm") as confirm, \
             patch("subprocess.call", return_value=1), \
             patch("subprocess.run",
                   return_value=MagicMock(returncode=1, stdout="",
                                          stderr="")):
            with self.assertRaises(flash.FlashError):
                flash.run(_args(firmware=self.firmware, port="/dev/x"))
        confirm.assert_not_called()


class MainStandaloneTests(unittest.TestCase):
    def test_flash_error_maps_to_rc_1(self):
        import io, sys
        orig_run = flash.run
        flash.run = lambda args: flash._die("boom")
        argv = sys.argv
        sys.argv = ["flash.py", "--name", "X", "--port", "/p",
                    "--firmware", "f.bin"]
        err = io.StringIO()
        orig_err, sys.stderr = sys.stderr, err
        try:
            rc = flash.main_standalone()
        finally:
            flash.run = orig_run
            sys.argv = argv
            sys.stderr = orig_err
        self.assertEqual(rc, 1)
        self.assertIn("error: boom", err.getvalue())


if __name__ == "__main__":
    unittest.main()
