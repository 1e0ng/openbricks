# SPDX-License-Identifier: MIT
# Board definition for the openbricks ESP32-S3 firmware image.
#
# Derived from MicroPython's ESP32_GENERIC_S3 board; the only openbricks-
# specific bit is freezing the ``openbricks`` Python package into the
# image so ``import openbricks.*`` works without needing to copy files
# via the REPL or mpremote.

set(IDF_TARGET esp32s3)

# Start from MicroPython's ESP32 base SDK config, plus SPIRAM.
# TWO fragments are required, in this order, matching upstream's
# ESP32_GENERIC_S3 SPIRAM_OCT variant:
#   * sdkconfig.spiram_sx  — actually ENABLES PSRAM (CONFIG_SPIRAM=y,
#     boot init, malloc integration, IGNORE_NOTFOUND so boards
#     without working PSRAM still boot) — defaults to quad mode;
#   * sdkconfig.spiram_oct — flips the mode to octal (N16R8 and the
#     in-package 8 MB AP_3v3 use octal PSRAM; quad-mode init aborts
#     with "PSRAM chip is not connected, or wrong PSRAM line mode").
# Through 1.19.x only spiram_oct was listed — it sets the MODE but
# never CONFIG_SPIRAM itself, so PSRAM was silently OFF and the GC
# heap was internal-RAM only (~234 KB; bench fragmentation aborted
# a 9.4 KB upload). With both fragments the GC auto-split heap
# (MICROPY_GC_SPLIT_HEAP_AUTO) grows into the 8 MB on demand.
# Plus BLE (NimBLE stack, configured in sdkconfig.ble — pins it to
# core 1 and raises the task stack to 6 KB for Python-level IRQ
# handlers), then our board overrides.
set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
    boards/sdkconfig.ble
    boards/sdkconfig.spiram_sx
    boards/sdkconfig.spiram_oct
    ${MICROPY_BOARD_DIR}/sdkconfig.board
)

# Freeze the openbricks/ package into the image.
set(MICROPY_FROZEN_MANIFEST ${MICROPY_BOARD_DIR}/manifest.py)
