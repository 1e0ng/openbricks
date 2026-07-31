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

// Raw-paste flow control: 2 KB advertised window — same rationale
// and patch dependency as the ESP32-S3 board (see its
// mpconfigboard.h); the classic ESP32 has ample internal DRAM for
// the 8 KB static ring.
#define MICROPY_HW_STDIN_RINGBUF_LEN   (8192)
#define MICROPY_REPL_STDIN_BUFFER_MAX  (4096)
