# SPDX-License-Identifier: MIT
"""``openbricks.firmware_label()`` — the version + provenance suffix.

The verdict comes from the NVS marker ``openbricks/fw_sig``
(``b"<version>:<verdict>"``, written by ``openbricks flash``); the
version prefix guards staleness. Runs under both CPython and unix
MicroPython against the fake ``esp32.NVS``.
"""

import tests._fakes  # noqa: F401
import tests._fakes_ble  # noqa: F401  (installs the fake esp32 module)

import unittest

import openbricks
from tests._fakes_ble import _FakeNVS


class FirmwareLabelTests(unittest.TestCase):
    def setUp(self):
        _FakeNVS._reset_for_test()

    def tearDown(self):
        _FakeNVS._reset_for_test()

    def _mark(self, marker):
        nvs = _FakeNVS("openbricks")
        nvs.set_blob("fw_sig", marker)
        nvs.commit()

    def test_matching_official_marker(self):
        self._mark(("%s:official" % openbricks.__version__).encode())
        self.assertEqual(
            openbricks.firmware_label(),
            "%s (official)" % openbricks.__version__)

    def test_no_marker_is_customized(self):
        self.assertEqual(
            openbricks.firmware_label(),
            "%s (customized)" % openbricks.__version__)

    def test_customized_marker_stays_customized(self):
        self._mark(("%s:customized" % openbricks.__version__).encode())
        self.assertEqual(
            openbricks.firmware_label(),
            "%s (customized)" % openbricks.__version__)

    def test_stale_marker_version_is_customized(self):
        # Firmware replaced behind the CLI's back: the marker still
        # names an older version.
        self._mark(b"0.0.1:official")
        self.assertEqual(
            openbricks.firmware_label(),
            "%s (customized)" % openbricks.__version__)

    def test_garbage_marker_is_customized(self):
        self._mark(b"not a marker at all")
        self.assertEqual(
            openbricks.firmware_label(),
            "%s (customized)" % openbricks.__version__)

    def test_label_always_carries_the_version(self):
        self.assertTrue(
            openbricks.firmware_label().startswith(
                openbricks.__version__ + " ("))


if __name__ == "__main__":
    unittest.main()
