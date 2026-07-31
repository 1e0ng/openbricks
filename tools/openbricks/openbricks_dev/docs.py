# SPDX-License-Identifier: MIT
"""``openbricks docs [topic]`` — read the guides offline, in the terminal.

The documentation guides are MyST Markdown, which reads perfectly
well raw; this command ships them inside the wheel (synced from the
repo's ``docs/`` at build time — see ``setup.py::_sync_docs``) so
``openbricks docs hardware`` works on a laptop with no internet, no
repo checkout, and no browser.

Only the hand-written guide pages are bundled. The API reference is
autodoc-generated from docstrings, so its offline equivalent is
Python's own ``help()``::

    python -c "import openbricks_sim"          # anything importable
    help(openbricks_sim)

With no topic, the command lists what's available. With a topic, the
page is printed — through ``$PAGER`` (or ``less``) when stdout is a
terminal, plain otherwise, so piping to ``grep`` works naturally.
"""

import os
import re
import subprocess
import sys


class DocsError(Exception):
    """Raised for user-facing failures (unknown topic, missing bundle)."""


# Guide pages in reading order. Stems must match ``docs/*.md`` in the
# repo (synced verbatim into the wheel's ``_docs/``).
TOPICS = [
    "index",
    "install",
    "hardware",
    "cli",
    "simulator",
    "examples",
    "architecture",
    "build",
]


def _docs_dir():
    """Locate the guide sources.

    Two valid modes, same shape as ``setup.py``'s shared-core sync:

      * Installed wheel/sdist — ``openbricks_dev/_docs/`` was bundled
        at build time.
      * Repo checkout (dev, CI tests) — read straight from the
        repo-root ``docs/``.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    bundled = os.path.join(here, "_docs")
    if os.path.isdir(bundled):
        return bundled
    checkout = os.path.normpath(
        os.path.join(here, "..", "..", "..", "docs"))
    if os.path.isdir(checkout):
        return checkout
    raise DocsError(
        "documentation pages not found: neither the bundled copy (%s) "
        "nor a repo checkout (%s) exists — broken installation?"
        % (bundled, checkout))


# ---- MyST → plain-terminal cleanup -------------------------------------

_FRONT_MATTER = re.compile(r"\A---\n.*?\n---\n+", re.S)
# {doc}`visible text <target>` / {ref}`...` → visible text
_ROLE_WITH_TEXT = re.compile(r"\{[a-z]+\}`([^`<>]+?)\s*<[^`>]+>`")
# {class}`a.b.C` / {mod}`x` / {doc}`page` → `a.b.C` (keep code style)
_ROLE_BARE = re.compile(r"\{[a-z]+\}`([^`]+)`")


def _clean(markdown):
    """Strip MyST front-matter and inline roles for terminal reading."""
    markdown = _FRONT_MATTER.sub("", markdown)
    markdown = _ROLE_WITH_TEXT.sub(r"\1", markdown)
    markdown = _ROLE_BARE.sub(r"`\1`", markdown)
    return markdown


def _load(topic):
    path = os.path.join(_docs_dir(), topic + ".md")
    if not os.path.isfile(path):
        raise DocsError(
            "unknown topic %r — available: %s" % (topic, ", ".join(TOPICS)))
    with open(path, "r") as f:
        return _clean(f.read())


def _title(topic):
    """First ``# `` heading of the page, for the listing."""
    for line in _load(topic).splitlines():
        if line.startswith("# "):
            return line[2:].strip()
    return topic


def _emit(text):
    """Page on a terminal, plain print otherwise (pipes, tests)."""
    if sys.stdout.isatty():
        pager = os.environ.get("PAGER", "less -R")
        try:
            subprocess.run(pager.split(), input=text.encode())
            return
        except FileNotFoundError:
            pass  # no pager binary — fall through to plain print
    sys.stdout.write(text)
    if not text.endswith("\n"):
        sys.stdout.write("\n")


def run(args):
    topic = getattr(args, "topic", None)
    if not topic:
        lines = ["Offline documentation topics:", ""]
        for t in TOPICS:
            lines.append("  %-14s %s" % (t, _title(t)))
        lines += [
            "",
            "Read one with:  openbricks docs <topic>",
            "The full manual (with API reference) is at "
            "https://docs.openbricks.dev/ — or its PDF, "
            "https://docs.openbricks.dev/openbricks-docs.pdf",
        ]
        _emit("\n".join(lines) + "\n")
        return 0
    _emit(_load(topic))
    return 0
