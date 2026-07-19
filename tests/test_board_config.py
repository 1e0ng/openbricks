# SPDX-License-Identifier: MIT
"""Firmware board-config drift guards (plain source scanning, runs
under CPython and unix MicroPython).

The S3 PSRAM regression these pin: through 1.19.x the board cmake
listed only ``sdkconfig.spiram_oct`` — which sets the PSRAM *mode*
but never ``CONFIG_SPIRAM=y`` itself — so the N16R8's 8 MB PSRAM was
silently unused and the GC heap was internal-RAM only (~234 KB;
bench fragmentation aborted a 9.4 KB ``openbricks run`` upload).
Enabling PSRAM takes BOTH upstream fragments, in order: spiram_sx
(enable + malloc integration + IGNORE_NOTFOUND) then spiram_oct
(quad → octal mode override).
"""

import tests._fakes  # noqa: F401

import unittest


_here = __file__
_idx = _here.rfind("/")
_ROOT = (_here[:_idx] if _idx >= 0 else ".") + "/.."

_S3_CMAKE = _ROOT + "/native/boards/openbricks_esp32s3/mpconfigboard.cmake"


def _read(path):
    with open(path) as f:
        return f.read()


class S3SpiramConfigTests(unittest.TestCase):
    def test_spiram_is_enabled_not_just_mode_selected(self):
        src = _read(_S3_CMAKE)
        self.assertIn("boards/sdkconfig.spiram_sx", src,
                      "spiram_sx missing: spiram_oct alone selects the "
                      "octal MODE but never enables PSRAM — the 1.19.x "
                      "silent-no-PSRAM bug")
        self.assertIn("boards/sdkconfig.spiram_oct", src)

    def test_enable_fragment_precedes_mode_override(self):
        # spiram_sx defaults to quad mode; spiram_oct must come AFTER
        # it so the octal override wins (N16R8 PSRAM is octal — quad
        # init aborts).
        src = _read(_S3_CMAKE)
        self.assertLess(src.index("boards/sdkconfig.spiram_sx"),
                        src.index("boards/sdkconfig.spiram_oct"))

    def test_upstream_enable_fragment_still_enables_spiram(self):
        # The guard's foundation: upstream's spiram_sx is the fragment
        # that sets CONFIG_SPIRAM=y (+ IGNORE_NOTFOUND so PSRAM-less
        # boards still boot). If upstream restructures these files,
        # this fails and the board config needs re-auditing.
        src = _read(
            _ROOT + "/native/micropython/ports/esp32/boards/sdkconfig.spiram_sx")
        self.assertIn("CONFIG_SPIRAM=y", src)
        self.assertIn("CONFIG_SPIRAM_IGNORE_NOTFOUND=y", src)
        oct_src = _read(
            _ROOT + "/native/micropython/ports/esp32/boards/sdkconfig.spiram_oct")
        self.assertIn("CONFIG_SPIRAM_MODE_OCT=y", oct_src)


if __name__ == "__main__":
    unittest.main()
