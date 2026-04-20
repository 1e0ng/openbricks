# SPDX-License-Identifier: MIT
# Board definition for the openbricks ESP32-S3 firmware image.
#
# Derived from MicroPython's ESP32_GENERIC_S3 board; the only openbricks-
# specific bit is freezing the ``openbricks`` Python package into the
# image so ``import openbricks.*`` works without needing to copy files
# via the REPL or mpremote.

set(IDF_TARGET esp32s3)

# Start from MicroPython's ESP32 base SDK config plus SPIRAM (most S3
# dev boards ship with octal or hex PSRAM) plus our board overrides.
# BLE is left off via sdkconfig.board to stay parallel with the ESP32
# image (we don't use BLE today).
set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
    boards/sdkconfig.spiram_sx
    ${MICROPY_BOARD_DIR}/sdkconfig.board
)

# Freeze the openbricks/ package into the image.
set(MICROPY_FROZEN_MANIFEST ${MICROPY_BOARD_DIR}/manifest.py)
