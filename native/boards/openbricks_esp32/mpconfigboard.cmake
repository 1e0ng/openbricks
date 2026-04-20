# SPDX-License-Identifier: MIT
# Board definition for the openbricks ESP32 firmware image.
#
# Derived from MicroPython's ESP32_GENERIC board; the only openbricks-
# specific bit is freezing the ``openbricks`` Python package into the
# image so ``import openbricks.*`` works without needing to copy files
# via the REPL or mpremote.

set(IDF_TARGET esp32)

set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
    boards/sdkconfig.ble
)

# Freeze the openbricks/ package into the image.
set(MICROPY_FROZEN_MANIFEST ${MICROPY_BOARD_DIR}/manifest.py)

# Register the openbricks user_c_module (the native motor scheduler).
set(USER_C_MODULES ${CMAKE_SOURCE_DIR}/../../user_c_modules/openbricks/micropython.cmake)
