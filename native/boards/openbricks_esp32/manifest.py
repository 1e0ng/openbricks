# SPDX-License-Identifier: MIT
# Frozen manifest for the openbricks ESP32 firmware image.
#
# Pulls in MicroPython's default manifest (asyncio, urequests, etc.) and
# adds the openbricks Python package so user code can ``import openbricks``
# without copying files.

include("$(PORT_DIR)/boards/manifest.py")

package(
    "openbricks",
    base_path="$(MPY_DIR)/../../..",  # resolve relative to the repo root
)
