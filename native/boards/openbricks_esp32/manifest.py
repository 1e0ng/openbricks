# SPDX-License-Identifier: MIT
# Frozen manifest for the openbricks ESP32 firmware image.
#
# Pulls in MicroPython's default manifest (asyncio, urequests, etc.) and
# adds the openbricks Python package so user code can ``import openbricks``
# without copying files.

include("$(PORT_DIR)/boards/manifest.py")

# Pull in micropython-lib's SSD1306 OLED driver — openbricks.drivers.ssd1306
# is a thin wrapper around it, so users never need to `mip install` it.
require("ssd1306")

package(
    "openbricks",
    # $(MPY_DIR) is native/micropython/, so two levels up is the repo
    # root. ``package("openbricks", base_path=<root>)`` freezes exactly
    # the ``openbricks/`` Python package — not the whole repo.
    base_path="$(MPY_DIR)/../..",
)

# openbricks_boot.py: runs on every boot (invoked from main.c via the
# patch in native/patches/), imports openbricks, triggers the BLE-toggle
# + LED auto-wiring before any user main.py runs.
module(
    "openbricks_boot.py",
    base_path="$(MPY_DIR)/../../native/boards",
)
