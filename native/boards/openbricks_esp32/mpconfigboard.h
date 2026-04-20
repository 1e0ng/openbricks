// SPDX-License-Identifier: MIT
// Board-specific macro overrides for the openbricks ESP32 firmware.
// Deliberately minimal — we only re-brand the board name and MCU
// string. Everything else comes from the ESP32 defaults.

#define MICROPY_HW_BOARD_NAME "openbricks (ESP32)"
#define MICROPY_HW_MCU_NAME   "ESP32"

// BLE disabled (see mpconfigboard.cmake) — prevent MP's nimble glue
// from compiling against headers the SDK no longer ships.
#define MICROPY_PY_BLUETOOTH (0)
#define MICROPY_BLUETOOTH_NIMBLE (0)
