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


class RawPasteWindowTests(unittest.TestCase):
    """Pin the raw-paste window bump (1.32.0): both boards advertise a
    2 KB flow-control window (MICROPY_REPL_STDIN_BUFFER_MAX / 2)
    instead of MicroPython's stock 128, which capped BLE staging at
    ~0.5 KB/s (one 128-byte grant per ack round trip — bench-measured
    16.6 s for a 7.9 KB script). The enlarged advertised buffer is
    ONLY safe together with the stdin-ring patch: on the UART/USB
    paste path (mpremote — our own ``flash`` uses it) in-flight bytes
    queue in ``stdin_ringbuf``, upstream a hardcoded 260-byte array;
    overflow silently drops paste bytes."""

    _BOARDS = [
        _ROOT + "/native/boards/openbricks_esp32s3/mpconfigboard.h",
        _ROOT + "/native/boards/openbricks_esp32/mpconfigboard.h",
    ]
    _PATCH = (_ROOT +
              "/native/patches/esp32-stdin-ringbuf-configurable.patch")

    def _defines(self, src):
        vals = {}
        for line in src.splitlines():
            parts = line.split()
            if (len(parts) >= 3 and parts[0] == "#define"
                    and parts[1] in ("MICROPY_HW_STDIN_RINGBUF_LEN",
                                     "MICROPY_REPL_STDIN_BUFFER_MAX")):
                vals[parts[1]] = int(parts[2].strip("()"))
        return vals

    def test_both_boards_define_window_and_ring(self):
        for path in self._BOARDS:
            vals = self._defines(_read(path))
            self.assertIn("MICROPY_REPL_STDIN_BUFFER_MAX", vals, path)
            self.assertIn("MICROPY_HW_STDIN_RINGBUF_LEN", vals, path)
            self.assertGreater(vals["MICROPY_REPL_STDIN_BUFFER_MAX"],
                               256, path)

    def test_ring_holds_at_least_two_advertised_buffers(self):
        # The raw-paste host may have up to ~buffer-max bytes in
        # flight (initial window + one refill); the ring must absorb
        # that plus interactive slack, or the UART/USB path drops
        # paste bytes silently. 2x is the safety invariant.
        for path in self._BOARDS:
            vals = self._defines(_read(path))
            self.assertGreaterEqual(
                vals["MICROPY_HW_STDIN_RINGBUF_LEN"],
                2 * vals["MICROPY_REPL_STDIN_BUFFER_MAX"], path)

    def test_ring_patch_exists_and_is_the_config_hook(self):
        src = _read(self._PATCH)
        self.assertIn("#ifndef MICROPY_HW_STDIN_RINGBUF_LEN", src)
        self.assertIn(
            "stdin_ringbuf_array[MICROPY_HW_STDIN_RINGBUF_LEN]", src)
        self.assertIn("ports/esp32/mphalport.c", src)

    def test_build_script_applies_patches(self):
        # The ring size only becomes configurable because
        # build_firmware.sh git-applies native/patches/ before every
        # firmware build — if that loop disappears, the boards'
        # RINGBUF_LEN define silently no-ops and the enlarged window
        # overruns the stock 260-byte ring on UART/USB.
        src = _read(_ROOT + "/scripts/build_firmware.sh")
        self.assertIn("PATCHES_DIR", src)
        self.assertIn("apply", src)
