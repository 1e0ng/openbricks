# SPDX-License-Identifier: MIT
"""Tests for ``openbricks_dev.scan`` — BleakScanner mocked out.

We never actually open a BLE adapter here; ``_discover`` is patched to
return a deterministic device list so row formatting, sorting, and
filtering are testable without hardware.
"""

import argparse
import io
import unittest
from unittest.mock import patch, MagicMock

from openbricks_dev import scan


def _args(timeout=5.0, all=False):
    return argparse.Namespace(timeout=timeout, all=all)


def _dev(address, name, rssi, adv_rssi=None, adv_name=None):
    """Build a (BLEDevice-like, AdvertisementData-like) pair.

    ``adv_rssi`` / ``adv_name`` let tests simulate the bleak 0.22+ shape
    where RSSI and local_name live on AdvertisementData; fall back to
    the legacy BLEDevice fields when omitted.
    """
    d = MagicMock()
    d.address = address
    d.name = name
    d.rssi = rssi
    adv = MagicMock()
    adv.rssi = adv_rssi
    adv.local_name = adv_name
    return (d, adv)


class FormatRowTests(unittest.TestCase):
    def test_named_device_renders(self):
        dev, adv = _dev("AA:BB", "RobotA", -55, adv_rssi=-55)
        row = scan._format_row(dev, adv, show_all=False)
        self.assertIn("RobotA", row)
        self.assertIn("AA:BB", row)
        self.assertIn("-55", row)

    def test_unnamed_device_hidden_by_default(self):
        dev, adv = _dev("AA:BB", None, -55, adv_rssi=-55)
        self.assertIsNone(scan._format_row(dev, adv, show_all=False))

    def test_unnamed_device_shown_with_all_flag(self):
        dev, adv = _dev("AA:BB", None, -55, adv_rssi=-55)
        row = scan._format_row(dev, adv, show_all=True)
        self.assertIsNotNone(row)
        self.assertIn("(no name)", row)
        self.assertIn("AA:BB", row)

    def test_rssi_missing_renders_question_mark(self):
        dev, adv = _dev("AA:BB", "RobotA", None, adv_rssi=None)
        row = scan._format_row(dev, adv, show_all=False)
        self.assertIn("?", row)

    def test_adv_local_name_used_when_device_name_missing(self):
        # bleak sometimes has the name on AdvertisementData rather than
        # the BLEDevice; the formatter should pick it up.
        dev, adv = _dev("AA:BB", None, -40, adv_rssi=-40, adv_name="RobotB")
        row = scan._format_row(dev, adv, show_all=False)
        self.assertIn("RobotB", row)


class RunTests(unittest.TestCase):
    def _patch_discover(self, devices):
        async def _fake(timeout):
            return devices
        return patch("openbricks_dev.scan._discover", side_effect=_fake)

    def test_sorted_by_rssi_strongest_first(self):
        devices = [
            _dev("AA:01", "Far",    -80, adv_rssi=-80),
            _dev("AA:02", "Close",  -30, adv_rssi=-30),
            _dev("AA:03", "Medium", -55, adv_rssi=-55),
        ]
        with self._patch_discover(devices), \
             patch("sys.stdout", new_callable=io.StringIO) as out:
            rc = scan.run(_args())
        self.assertEqual(rc, 0)
        text = out.getvalue()
        # Assert Close comes before Medium comes before Far in output.
        self.assertLess(text.index("Close"), text.index("Medium"))
        self.assertLess(text.index("Medium"), text.index("Far"))

    def test_empty_scan_prints_hint(self):
        with self._patch_discover([]), \
             patch("sys.stdout", new_callable=io.StringIO) as out:
            rc = scan.run(_args())
        self.assertEqual(rc, 0)
        self.assertIn("no named BLE devices", out.getvalue())

    def test_empty_scan_with_all_flag_different_message(self):
        with self._patch_discover([]), \
             patch("sys.stdout", new_callable=io.StringIO) as out:
            rc = scan.run(_args(all=True))
        self.assertEqual(rc, 0)
        self.assertIn("no BLE devices", out.getvalue())

    def test_scan_error_propagates(self):
        async def _boom(timeout):
            raise scan.ScanError("adapter not found")
        with patch("openbricks_dev.scan._discover", side_effect=_boom):
            with self.assertRaises(scan.ScanError):
                scan.run(_args())




class DiscoverTests(unittest.TestCase):
    """_discover against an injected fake bleak: modern dict-shaped
    results, legacy list results, scan failure, missing bleak."""

    def setUp(self):
        import sys
        self._had = "bleak" in sys.modules
        self._prev = sys.modules.get("bleak")
        self._sys = sys

    def tearDown(self):
        if self._had:
            self._sys.modules["bleak"] = self._prev
        else:
            self._sys.modules.pop("bleak", None)

    def _inject(self, result=None, raises=None):
        class _Scanner:
            @staticmethod
            async def discover(timeout=None, return_adv=True):
                if raises:
                    raise raises
                return result

        class _Bleak:
            BleakScanner = _Scanner
        self._sys.modules["bleak"] = _Bleak

    def _drive(self):
        import asyncio
        return asyncio.run(scan._discover(1.0))

    def test_dict_result_unpacks_device_adv_pairs(self):
        dev, adv = _dev("aa:bb", "A", -40)
        self._inject(result={"aa:bb": (dev, adv)})
        self.assertEqual(self._drive(), [(dev, adv)])

    def test_legacy_list_result_gets_none_adv(self):
        dev, _ = _dev("aa:bb", "A", -40)
        self._inject(result=[dev])
        self.assertEqual(self._drive(), [(dev, None)])

    def test_scan_failure_wraps_in_scan_error(self):
        self._inject(raises=OSError("adapter off"))
        try:
            self._drive()
        except scan.ScanError as e:
            self.assertIn("BLE scan failed", str(e))
        else:
            self.fail("expected ScanError")

    def test_missing_bleak_reports_install_hint(self):
        self._sys.modules["bleak"] = None
        try:
            self._drive()
        except scan.ScanError as e:
            self.assertIn("pip install bleak", str(e))
        else:
            self.fail("expected ScanError")


class RunRowSkipAndSortFallbackTests(unittest.TestCase):
    def _run_with(self, entries, all_flag=False):
        import argparse, io, sys
        orig = scan._discover

        async def _fake(timeout):
            return entries
        scan._discover = _fake
        out = io.StringIO()
        prev, sys.stdout = sys.stdout, out
        try:
            scan.run(argparse.Namespace(timeout=0.1, all=all_flag))
        finally:
            sys.stdout = prev
            scan._discover = orig
        return out.getvalue()

    def test_unnamed_device_skipped_in_listing(self):
        named = _dev("aa:01", "Hub", -40)
        unnamed = _dev("aa:02", None, -30)
        out = self._run_with([named, unnamed])
        self.assertIn("Hub", out)
        self.assertNotIn("aa:02", out)

    def test_sort_falls_back_to_device_rssi(self):
        # Legacy entries with adv=None must sort by the BLEDevice's
        # own rssi attribute.
        strong = _dev("aa:01", "Strong", -20)[0], None
        weak = _dev("aa:02", "Weak", -80)[0], None
        out = self._run_with([weak, strong])
        self.assertTrue(out.index("Strong") < out.index("Weak"))


class RunErrorWrapTests(unittest.TestCase):
    def test_unexpected_exception_becomes_scan_error(self):
        import argparse
        orig = scan._discover

        def _boom(timeout):
            raise ValueError("weird")
        scan._discover = _boom
        try:
            with self.assertRaises(scan.ScanError):
                scan.run(argparse.Namespace(timeout=0.1, all=False))
        finally:
            scan._discover = orig


if __name__ == "__main__":
    unittest.main()
