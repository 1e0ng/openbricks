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
    # NOTE: no "micropython" entry — nothing in docs/ referenced its
    # inventory, and the fetch made every -W docs build hostage to
    # docs.micropython.org's uptime (2026-07-25: the site 404'd and
    # blocked CI on an unrelated examples-only PR). Re-add only
    # together with an actual cross-reference that needs it.
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

# ---- PDF (sphinx -M latexpdf, compiled in CI and published next to
#      the HTML as /openbricks-docs.pdf — see website.yaml) ----
#
# xelatex rather than pdflatex: the docs lean on unicode (°, ², ³, →,
# ±, em-dashes) that pdflatex trips over; xelatex with Sphinx's
# default FreeSerif/FreeSans/FreeMono fonts (fonts-freefont-otf in
# CI) renders them natively.
latex_engine = "xelatex"
latex_documents = [
    (
        "index",
        "openbricks.tex",
        "openbricks documentation",
        author,
        "manual",
    ),
]
latex_elements = {
    # A4 fits the international audience; 11pt for readability.
    "papersize": "a4paper",
    "pointsize": "11pt",
    # Keep chapters opening on any page — the docs are guides, not a
    # book; blank verso pages between short chapters read as bloat.
    "extraclassoptions": "openany,oneside",
}
