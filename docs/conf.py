# SPDX-License-Identifier: MIT
"""Sphinx configuration for docs.openbricks.dev.

The API reference is autodoc'd straight from the firmware package in the
repo root, so the docs can never drift from the docstrings. MicroPython's
board-level modules (``machine`` & friends) don't exist on the CPython
that builds the docs — they're mocked, which is fine because autodoc only
reads signatures and docstrings.
"""

import os
import sys

# Repo root (firmware package) + host tooling (openbricks_dev CLI).
sys.path.insert(0, os.path.abspath(".."))
sys.path.insert(0, os.path.abspath("../tools/openbricks"))

import openbricks  # noqa: E402  (needs the sys.path insert above)

project = "openbricks"
author = "the openbricks contributors"
copyright = "2026, the openbricks contributors"  # noqa: A001
release = openbricks.__version__
version = release

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinx.ext.intersphinx",
    "myst_parser",
    "sphinxarg.ext",
]

# MicroPython-only modules that don't exist on the doc-building CPython.
autodoc_mock_imports = [
    "machine",
    "micropython",
    "esp32",
    "neopixel",
    "bluetooth",
    "_openbricks_native",
]
autodoc_member_order = "bysource"
autodoc_preserve_defaults = True

napoleon_google_docstring = True
napoleon_numpy_docstring = True

myst_enable_extensions = ["colon_fence"]
# Generate #anchors for h1-h3 so in-page links in the guides resolve.
myst_heading_anchors = 3

intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "micropython": ("https://docs.micropython.org/en/latest", None),
}

exclude_patterns = ["_build", "datasheets"]

html_theme = "sphinx_rtd_theme"
html_baseurl = "https://docs.openbricks.dev/"
html_static_path = ["_static"]
html_css_files = ["custom.css"]
html_theme_options = {
    "collapse_navigation": False,
    "navigation_depth": 3,
    "logo_only": False,
}
html_context = {
    "display_github": True,
    "github_user": "1e0ng",
    "github_repo": "openbricks",
    "github_version": "main",
    "conf_py_path": "/docs/",
}
