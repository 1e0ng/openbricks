# SPDX-License-Identifier: MIT
# Board definition for the openbricks ESP32 firmware image.
#
# Derived from MicroPython's ESP32_GENERIC board; the only openbricks-
# specific bit is freezing the ``openbricks`` Python package into the
# image so ``import openbricks.*`` works without needing to copy files
# via the REPL or mpremote.

set(IDF_TARGET esp32)

# Start from MicroPython's ESP32 base SDK config and our board overrides.
# We deliberately skip ``sdkconfig.ble`` — MicroPython v1.28.0's nimble
# glue doesn't compile cleanly against ESP-IDF 5.4's new ble_hs_state
# API. openbricks doesn't use BLE today, so dropping it sidesteps the
# incompatibility. Re-add when either MP gets the fix upstream or we
# pin to ESP-IDF 5.2/5.3.
set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
    ${MICROPY_BOARD_DIR}/sdkconfig.board
)

# Freeze the openbricks/ package into the image.
set(MICROPY_FROZEN_MANIFEST ${MICROPY_BOARD_DIR}/manifest.py)

# ``USER_C_MODULES`` is passed in by ``scripts/build_firmware.sh`` (or by
# the caller of ``make``) — we don't re-declare it here because the
# relative-path form that used to live below got broken by MicroPython's
# source-directory layout, and an absolute path is cleaner to inject
# from the build script anyway.
