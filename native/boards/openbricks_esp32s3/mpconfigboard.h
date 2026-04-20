// SPDX-License-Identifier: MIT
// Board-specific macro overrides for the openbricks ESP32-S3 firmware.

#define MICROPY_HW_BOARD_NAME "openbricks (ESP32-S3)"
#define MICROPY_HW_MCU_NAME   "ESP32S3"

// BLE disabled for parity with the ESP32 image.
#define MICROPY_PY_BLUETOOTH (0)
#define MICROPY_BLUETOOTH_NIMBLE (0)
