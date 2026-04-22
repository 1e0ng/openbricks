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

# Default ``main.py`` — drives the Pybricks-style button-gated launcher
# so ``openbricks-dev download`` can stage code without running it.
# Users who want different boot behaviour write to ``/main.py`` in VFS;
# that file takes priority over this frozen default.
freeze("$(MPY_DIR)/../../native/frozen")
