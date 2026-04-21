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
