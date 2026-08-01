// SPDX-License-Identifier: MIT
// Board-specific macro overrides for the openbricks ESP32 firmware.
// Deliberately minimal — we only re-brand the board name and MCU
// string. Everything else comes from the ESP32 defaults, including
// BLE (enabled via sdkconfig.ble in mpconfigboard.cmake).

#define MICROPY_HW_BOARD_NAME "openbricks (ESP32)"
#define MICROPY_HW_MCU_NAME   "ESP32"

// WiFi is disabled in sdkconfig.board — also drop MP's network-wlan
// wrapper so the firmware doesn't carry a dead ``import network`` path.
#define MICROPY_PY_NETWORK        (0)
#define MICROPY_PY_NETWORK_WLAN   (0)

// Non-blocking UART stdout (native/patches/
// esp32-uart-repl-tx-nonblocking.patch). Same rationale as the S3
// board: upstream's blocking tx_strn made every print() pay UART
// wire time (~5.1 ms for a typical line at 115200) on the calling
// thread. NOTE the trade is sharper here than on the S3: classic
// ESP32 has no native USB, so UART0 IS the wired console — under a
// print storm deeper than the ring (~178 ms of line time) the wired
// console drops the remainder while BLE and the run log keep
// everything. An attached terminal still sees normal output at
// normal rates; only storm backlogs are lossy.
#define MICROPY_HW_UART_REPL_TX_RING (2048)

// Raw-paste flow control: STOCK (MicroPython's 256-byte buffer max
// => 128-byte window). 1.32.0 raised it to 2048 and 1.32.1 to 1024;
// BOTH broke staging over BLE on real hardware — at 2048 the hub
// compiled a truncated program (empty stdout AND stderr), at 1024 it
// stopped consuming mid-paste and hung with no flow-control ack. The
// cause is somewhere in the BLE input path's tolerance for multi-
// packet bursts, and it is NOT the GATT rx buffer alone (raising
// that to 8 KB did not fix 1024).
//
// Do NOT re-raise this without measuring first: run
// ``openbricks paste-probe -n NAME`` against the real hub and set
// the window from what actually survives. The
// stdin-ring patch (native/patches/) stays available for when that
// measurement exists.
