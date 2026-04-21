# SPDX-License-Identifier: MIT
# Board definition for the openbricks ESP32 firmware image.
#
# Derived from MicroPython's ESP32_GENERIC board; the only openbricks-
# specific bit is freezing the ``openbricks`` Python package into the
# image so ``import openbricks.*`` works without needing to copy files
# via the REPL or mpremote.

set(IDF_TARGET esp32)

# Start from MicroPython's ESP32 base SDK config, layer BLE support
# (NimBLE stack — enables ``bluetooth.BLE()`` for REPL over BLE and the
# mpremote-style code transfer workflow), then our board overrides.
# The sdkconfig.ble file pins NimBLE to core 1 and raises its stack to
# 6 KB so Python-level IRQ handlers have room to run.
set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
    boards/sdkconfig.ble
    ${MICROPY_BOARD_DIR}/sdkconfig.board
)

# Freeze the openbricks/ package into the image.
set(MICROPY_FROZEN_MANIFEST ${MICROPY_BOARD_DIR}/manifest.py)

# ``USER_C_MODULES`` is passed in by ``scripts/build_firmware.sh`` (or by
# the caller of ``make``) — we don't re-declare it here because the
# relative-path form that used to live below got broken by MicroPython's
# source-directory layout, and an absolute path is cleaner to inject
# from the build script anyway.
