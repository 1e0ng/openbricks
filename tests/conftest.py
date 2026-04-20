# SPDX-License-Identifier: MIT
"""Ensure the fake ``machine`` module is installed before any openbricks
import. Importing ``tests._fakes`` has the side effect of registering it in
``sys.modules``."""

import tests._fakes  # noqa: F401
