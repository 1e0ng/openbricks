# SPDX-License-Identifier: MIT
"""``openbricks docs [topic]`` — read the documentation offline.

Opens the SAME Sphinx build as docs.openbricks.dev — API reference
included — from a bundle shipped inside the wheel
(``_docs/offline-docs.zip``, built by ``scripts/build-offline-docs.sh``
and synced by ``setup.py::_sync_docs``). Works on a laptop with no
internet connection and no repo checkout.

Until 1.63.0 this command re-rendered hand-written markdown guides
with its own styling, so everything generated from docstrings was
missing; that renderer (and its ``--text`` mode) is gone.
"""

import os
import sys
import tempfile
import webbrowser


class DocsError(Exception):
    """Raised for user-facing failures (unknown topic, missing bundle)."""


_BUNDLE = "offline-docs.zip"


def _bundle_path():
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "_docs", _BUNDLE)
    if not os.path.exists(path):
        raise DocsError(
            "the offline documentation bundle is missing (%s). A wheel "
            "always ships it; from a source checkout run "
            "scripts/build-offline-docs.sh first." % path)
    return path


def _extract():
    """Unpack the bundle to a temp dir, once per version.

    Extracted rather than served from the archive because a browser
    needs real files for the stylesheet, search index and inter-page
    links. Keyed by content hash so a CLI upgrade never opens the
    previous version's manual out of a stale directory.
    """
    import hashlib
    import zipfile
    src = _bundle_path()
    with open(src, "rb") as f:
        tag = hashlib.sha256(f.read()).hexdigest()[:12]
    out = os.path.join(tempfile.gettempdir(), "openbricks-docs-" + tag)
    index = os.path.join(out, "index.html")
    if not os.path.exists(index):
        with zipfile.ZipFile(src) as z:
            z.extractall(out)
    if not os.path.exists(index):
        raise DocsError(
            "the offline bundle has no index.html — it is corrupt; "
            "reinstall openbricks or rebuild it with "
            "scripts/build-offline-docs.sh")
    return out


def _page_for(topic):
    """Topic -> page within the extracted site. API pages live under
    ``api/``, guides at the top level."""
    if not topic:
        return "index.html"
    for rel in (topic + ".html", os.path.join("api", topic + ".html")):
        yield_path = rel
        if os.path.exists(os.path.join(_extract(), rel)):
            return yield_path
    raise DocsError(
        "no page %r in the manual — open the index with "
        "``openbricks docs`` and use its sidebar or search." % topic)


def run(args):
    """Open the offline manual.

    This is the SAME Sphinx build as docs.openbricks.dev — the API
    reference included. It used to be the hand-written guides only,
    re-rendered from markdown, so everything generated from
    docstrings was missing entirely.
    """
    topic = getattr(args, "topic", None)
    root = _extract()
    page = _page_for(topic)
    url = "file://" + os.path.join(root, page)
    if not webbrowser.open(url):
        raise DocsError(
            "could not open a browser (headless session?) — the manual "
            "is extracted at %s, or read it online at "
            "https://docs.openbricks.dev/" % root)
    print("opened %s" % url)
    return 0
