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
    """The window is STOCK again (1.33.1) after two hardware failures
    — 2048 truncated pasted programs, 1024 hung mid-paste — and the
    boards must not silently re-acquire an override without the
    safety machinery that a raised window needs.

    If a board ever defines ``MICROPY_REPL_STDIN_BUFFER_MAX`` again,
    the ring must cover it (``RINGBUF_LEN >= 2x``) AND the value must
    be backed by a real measurement from ``openbricks paste-probe``
    — the tests below enforce the mechanical half; the measurement is
    on the human."""

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

    def test_window_override_is_absent_or_ring_backed(self):
        for path in self._BOARDS:
            vals = self._defines(_read(path))
            if "MICROPY_REPL_STDIN_BUFFER_MAX" not in vals:
                continue          # stock window — nothing to guard
            self.assertIn(
                "MICROPY_HW_STDIN_RINGBUF_LEN", vals,
                "%s raises the paste window without enlarging the "
                "stdin ring — UART/USB pastes will drop bytes" % path)
            self.assertGreaterEqual(
                vals["MICROPY_HW_STDIN_RINGBUF_LEN"],
                2 * vals["MICROPY_REPL_STDIN_BUFFER_MAX"], path)

    def test_reverted_boards_carry_the_do_not_raise_warning(self):
        # The comment is the institutional memory: two shipped
        # releases broke here. Losing it invites a third attempt.
        for path in self._BOARDS:
            src = _read(path)
            if "MICROPY_REPL_STDIN_BUFFER_MAX" in src:
                continue
            self.assertIn("paste-probe", src,
                          "%s lost the measure-before-raising note" % path)

    def test_ring_patch_exists_and_is_the_config_hook(self):
        # Kept available for a future, MEASURED bump.
        src = _read(self._PATCH)
        self.assertIn("#ifndef MICROPY_HW_STDIN_RINGBUF_LEN", src)
        self.assertIn(
            "stdin_ringbuf_array[MICROPY_HW_STDIN_RINGBUF_LEN]", src)

    def test_build_script_applies_patches(self):
        src = _read(_ROOT + "/scripts/build_firmware.sh")
        self.assertIn("PATCHES_DIR", src)
        self.assertIn("apply", src)


class BleRxBufferTests(unittest.TestCase):
    """THE 1.32.0 REGRESSION GUARD.

    Raw-paste flow control lets the host keep two advertised windows
    (= one ``MICROPY_REPL_STDIN_BUFFER_MAX``) in flight before it
    waits for an ack. Every transport that carries a paste must be
    able to absorb that much unread data:

      * UART / USB → ``stdin_ringbuf`` (RawPasteWindowTests above).
      * BLE        → the NUS RX characteristic's GATT buffer, set in
                     ``openbricks/ble_repl.py`` (``_RX_BUFFER_BYTES``).

    1.32.0 raised the window to 2048 (in-flight 4096) and left the
    BLE buffer at 512 — NimBLE silently dropped the overflow, the
    chip compiled a fragment of the staged receiver, and
    ``openbricks run`` died with "hub did not confirm the staged
    chunk" (empty stdout AND stderr — the tell-tale of a program
    truncated to nothing). The UART-side invariant existed; the BLE
    side had no guard. It does now."""

    _BLE_REPL = _ROOT + "/openbricks/ble_repl.py"
    _BOARDS = [
        _ROOT + "/native/boards/openbricks_esp32s3/mpconfigboard.h",
        _ROOT + "/native/boards/openbricks_esp32/mpconfigboard.h",
    ]

    def _ble_rx_bytes(self):
        for line in _read(self._BLE_REPL).splitlines():
            if line.startswith("_RX_BUFFER_BYTES"):
                return int(line.split("=")[1].strip())
        self.fail("_RX_BUFFER_BYTES not found in ble_repl.py")

    def _buffer_max(self, path):
        for line in _read(path).splitlines():
            parts = line.split()
            if (len(parts) >= 3 and parts[0] == "#define"
                    and parts[1] == "MICROPY_REPL_STDIN_BUFFER_MAX"):
                return int(parts[2].strip("()"))
        # Absent = stock: MicroPython's own default buffer max.
        return 256

    def test_ble_buffer_absorbs_max_in_flight_paste(self):
        rx = self._ble_rx_bytes()
        for path in self._BOARDS:
            in_flight = self._buffer_max(path)   # 2 windows
            self.assertGreaterEqual(
                rx, 2 * in_flight,
                "BLE rx buffer (%d) must hold >= 2x the raw-paste "
                "in-flight bytes (%d) for %s — undersizing silently "
                "truncates pasted programs (the 1.32.0 bug)"
                % (rx, in_flight, path))

    def test_ble_buffer_is_the_default_used_by_bleuart(self):
        # The constant must actually be wired as the default — a
        # stale literal in the signature would re-open the bug.
        src = _read(self._BLE_REPL)
        self.assertIn("def __init__(self, ble, name, rxbuf=_RX_BUFFER_BYTES)",
                      src)
        self.assertIn("gatts_set_buffer(self._rx_handle, rxbuf, True)", src)


class UartTxRingTests(unittest.TestCase):
    """Non-blocking UART stdout (the 90-ms-per-print fix, final leg).

    The C behaviour itself is exercised by CI's qemu-smoke job — the
    banner arriving over UART0 IS the patched TX path working. What
    host tests can pin is the configuration contract: the patch stays
    inert at stock config, both boards opt in, and the build script
    still applies patches at all. Drift in any of these silently
    reverts print() to paying ~5.1 ms of wire time per line.
    """

    _BOARDS = [
        _ROOT + "/native/boards/openbricks_esp32s3/mpconfigboard.h",
        _ROOT + "/native/boards/openbricks_esp32/mpconfigboard.h",
    ]
    _PATCH = (_ROOT +
              "/native/patches/esp32-uart-repl-tx-nonblocking.patch")

    def _ring_define(self, src):
        for line in src.splitlines():
            parts = line.split()
            if (len(parts) >= 3 and parts[0] == "#define"
                    and parts[1] == "MICROPY_HW_UART_REPL_TX_RING"):
                return int(parts[2].strip("()"))
        return None

    def test_patch_is_inert_at_stock_config(self):
        # Upstream-identical default: the hook must be #ifndef-guarded
        # and default to 0 (blocking), same convention as the stdin
        # ring patch. A non-zero default would change behaviour for
        # any config that doesn't opt in.
        src = _read(self._PATCH)
        self.assertIn("#ifndef MICROPY_HW_UART_REPL_TX_RING", src)
        self.assertIn("#define MICROPY_HW_UART_REPL_TX_RING (0)", src)

    def test_patch_keeps_the_blocking_fallback(self):
        # The #else branch must retain upstream's blocking loop so a
        # board at 0 builds the original code, not a stub.
        src = _read(self._PATCH)
        self.assertIn("ulTaskNotifyTake", src)

    def test_both_boards_opt_in(self):
        for path in self._BOARDS:
            ring = self._ring_define(_read(path))
            self.assertIsNotNone(
                ring, "%s does not define MICROPY_HW_UART_REPL_TX_RING "
                "— print() pays blocking UART wire time again" % path)
            self.assertGreaterEqual(
                ring, 512,
                "%s ring (%d) is smaller than a burst of a few long "
                "lines — drops would start during ordinary output"
                % (path, ring))

    def test_ring_drains_and_disarms_in_the_isr(self):
        # The two ISR-side obligations: drain ring -> FIFO, and
        # disable the interrupt when the ring is empty (else an idle
        # line re-fires TXFIFO_EMPTY forever).
        src = _read(self._PATCH)
        self.assertIn("uart_hal_write_txfifo", src)
        self.assertIn("uart_hal_disable_intr_mask", src)
        self.assertIn("UART_INTR_TXFIFO_EMPTY", src)


class HardTickPatchTests(unittest.TestCase):
    """The below-the-scheduler tick (arc PR 2). The C behaviour needs
    hardware (esp_timer) — what host tests pin is the patch's shape:
    it must add the source to the build, define the config macro the
    user module keys off, and keep the ISR-context contract visible.
    """

    _PATCH = (_ROOT +
              "/native/patches/esp32-openbricks-hard-tick.patch")

    def test_patch_adds_source_macro_and_build_entry(self):
        src = _read(self._PATCH)
        self.assertIn("openbricks_hard_tick.c", src)
        self.assertIn("MICROPY_OPENBRICKS_HARD_TICK", src)
        self.assertIn("esp_timer_start_periodic", src)
        # cmake registration — without it the symbol never links.
        self.assertIn("esp32_common.cmake", src)

    def test_user_module_gates_on_the_macro(self):
        src = _read(_ROOT +
                    "/native/user_c_modules/openbricks/motor_process.c")
        self.assertIn("MICROPY_OPENBRICKS_HARD_TICK", src)
        self.assertIn("ob_hard_tick_install", src)

    def test_hook_contract_is_documented_in_the_patch(self):
        # The no-VM-calls contract is enforced by review; losing the
        # comment loses the review.
        src = _read(self._PATCH)
        self.assertIn("must not touch Python", src)


class BusUartPatchTests(unittest.TestCase):
    """The serial-bus UART shims (arc PR 3b): non-blocking IDF UART
    access for the hard-tick bus pump. Host tests pin the patch shape;
    the firmware CI job is the compile gate and the bench ping is the
    functional gate."""

    _PATCH = _ROOT + "/native/patches/esp32-openbricks-bus-uart.patch"

    def test_patch_adds_shims_macro_and_build_entry(self):
        src = _read(self._PATCH)
        self.assertIn("openbricks_bus_uart.c", src)
        self.assertIn("MICROPY_OPENBRICKS_BUS_UART", src)
        self.assertIn("esp32_common.cmake", src)
        # The non-blocking guarantees are the whole point.
        self.assertIn("uart_get_tx_buffer_free_size", src)

    def test_user_module_gates_on_the_macro(self):
        src = _read(_ROOT + "/native/user_c_modules/openbricks/st_bus.c")
        self.assertIn("MICROPY_OPENBRICKS_BUS_UART", src)
        self.assertIn("ob_bus_uart_open", src)
        # Two-context safety: the spinlock must exist.
        self.assertIn("atomic_flag", src)
