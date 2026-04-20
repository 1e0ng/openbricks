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
