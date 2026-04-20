# SPDX-License-Identifier: MIT
# Frozen manifest for the openbricks ESP32 firmware image.
#
# Pulls in MicroPython's default manifest (asyncio, urequests, etc.) and
# adds the openbricks Python package so user code can ``import openbricks``
# without copying files.

include("$(PORT_DIR)/boards/manifest.py")

package(
    "openbricks",
    # $(MPY_DIR) is native/micropython/, so two levels up is the repo
    # root. ``package("openbricks", base_path=<root>)`` freezes exactly
    # the ``openbricks/`` Python package — not the whole repo.
    base_path="$(MPY_DIR)/../..",
)
