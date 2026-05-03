# SPDX-License-Identifier: MIT
# Frozen manifest for the openbricks ESP32-S3 firmware image.
# See native/boards/openbricks_esp32/manifest.py for rationale.

include("$(PORT_DIR)/boards/manifest.py")

# Pull in micropython-lib's SSD1306 OLED driver — openbricks.drivers.ssd1306
# is a thin wrapper around it, so users never need to `mip install` it.
require("ssd1306")

package(
    "openbricks",
    base_path="$(MPY_DIR)/../..",
)

# v1.0.4: default ``main.py`` removed from the frozen manifest.
# The previous version freezed ``native/frozen/main.py`` which
# ran ``bluetooth.apply_persisted_state()`` + hub init on every
# boot. If any of that crashed or hung (which it did during
# bring-up on AP_3v3 chips with no hub name written), no traceback
# reached the REPL and the device looked silent. Without it, fresh
# boots drop to the REPL; users who want auto-run wire their own
# ``main.py`` via ``openbricks upload --path /main.py``.
