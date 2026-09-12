# SPDX-License-Identifier: MIT
"""The per-hub cache that lets ``run`` / ``upload`` skip the firmware
version probe."""

import os
import tempfile
import unittest
from unittest.mock import patch

from openbricks_dev import _hubcache as hc


class HubCacheTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        patcher = patch.dict(os.environ, {hc.CACHE_ENV: self.dir})
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_unknown_hub_is_none(self):
        self.assertIsNone(hc.firmware_version("Ghost"))

    def test_remembered_version_round_trips_and_ignores_provenance(self):
        hc.remember_firmware("RobotA", "4.9.1 (official)")
        self.assertEqual(hc.firmware_version("RobotA"), (4, 9, 1))
        self.assertTrue(os.path.exists(os.path.join(self.dir, "hubs.json")))
        hc.remember_firmware("RobotA", "1.91.0")
        self.assertEqual(hc.firmware_version("RobotA"), (1, 91, 0))
        hc.forget("RobotA")
        self.assertIsNone(hc.firmware_version("RobotA"))
        hc.forget("RobotA")     # twice is fine

    def test_garbage_versions_are_not_remembered(self):
        hc.remember_firmware("RobotA", "banana")
        hc.remember_firmware("RobotA", "1.2")
        self.assertIsNone(hc.firmware_version("RobotA"))
        self.assertIsNone(hc.parse_version(None))
        self.assertIsNone(hc.parse_version("x.y.z"))
        self.assertEqual(hc.parse_version("10.20.30-rc1"), None)
        self.assertEqual(hc.parse_version("10.20.30"), (10, 20, 30))

    def test_corrupt_file_reads_as_empty(self):
        with open(os.path.join(self.dir, "hubs.json"), "w") as f:
            f.write("{not json")
        self.assertIsNone(hc.firmware_version("RobotA"))
        with open(os.path.join(self.dir, "hubs.json"), "w") as f:
            f.write("[1, 2]")
        self.assertIsNone(hc.firmware_version("RobotA"))
        hc.remember_firmware("RobotA", "4.9.1")
        self.assertEqual(hc.firmware_version("RobotA"), (4, 9, 1))

    def test_unwritable_dir_is_silent(self):
        blocker = os.path.join(self.dir, "file")
        with open(blocker, "w") as f:
            f.write("x")
        # the cache dir is a FILE: makedirs and the write both fail, quietly
        with patch.dict(os.environ, {hc.CACHE_ENV: blocker}):
            hc.remember_firmware("RobotA", "4.9.1")
            self.assertIsNone(hc.firmware_version("RobotA"))
            hc.forget("RobotA")

    def test_default_dir_follows_xdg(self):
        with patch.dict(os.environ, {"XDG_CACHE_HOME": "/tmp/xdg"}, clear=False):
            os.environ.pop(hc.CACHE_ENV, None)
            self.assertEqual(hc.cache_dir(), os.path.join("/tmp/xdg", "openbricks"))
