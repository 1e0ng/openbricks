# SPDX-License-Identifier: MIT
# Frozen manifest for the openbricks ESP32-S3 firmware image.
# See native/boards/openbricks_esp32/manifest.py for rationale.

include("$(PORT_DIR)/boards/manifest.py")

package(
    "openbricks",
    base_path="$(MPY_DIR)/../..",
)
