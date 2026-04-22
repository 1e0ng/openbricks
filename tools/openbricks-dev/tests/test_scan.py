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


if __name__ == "__main__":
    unittest.main()
