#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Generate ``versions.json`` for docs.openbricks.dev.

The deploy job (website.yaml) lays archived docs builds — one
directory per released firmware version, each a full Sphinx HTML
tree with its ``openbricks-docs.pdf`` — from the ``docs-archive``
branch into the site next to the current build, then runs this to
write the manifest that _static/version-switcher.js renders as the
sidebar version flyout.

Usage:

    scripts/docs-versions.py ARCHIVE_DIR LATEST_VERSION > versions.json

``ARCHIVE_DIR`` is the directory whose immediate ``X.Y.Z`` children
are the archived builds; ``LATEST_VERSION`` is the version the site
root currently documents (``openbricks.__version__``).
"""

import json
import re
import sys
from pathlib import Path

_VERSION_DIR_RE = re.compile(r"^\d+\.\d+\.\d+$")


def version_key(version):
    """Sort key for ``X.Y.Z`` strings (numeric, not lexicographic —
    ``2.10.0`` sorts above ``2.9.0``)."""
    return tuple(int(part) for part in version.split("."))


def collect_versions(archive_dir):
    """The archived version directories under ``archive_dir``, newest
    first.

    A directory only counts with an ``index.html`` inside — a partial
    checkout or an interrupted archive commit must not become a dead
    link in the switcher. Raises ``FileNotFoundError`` if
    ``archive_dir`` itself is missing: the deploy checking out the
    docs-archive branch is a precondition, and its absence is a
    pipeline bug to surface, not an empty menu to render.
    """
    root = Path(archive_dir)
    if not root.is_dir():
        raise FileNotFoundError(
            "archive directory %s does not exist — the deploy must "
            "check out the docs-archive branch first" % archive_dir)
    versions = []
    for child in root.iterdir():
        if not child.is_dir() or not _VERSION_DIR_RE.match(child.name):
            continue
        if not (child / "index.html").is_file():
            raise RuntimeError(
                "archived docs at %s have no index.html — the "
                "archive commit for %s is incomplete; re-run the "
                "backfill for that tag" % (child, child.name))
        versions.append(child.name)
    return sorted(versions, key=version_key, reverse=True)


def build_manifest(versions, latest):
    """The ``versions.json`` payload: the root's version plus one
    entry per archived build, newest first."""
    if not _VERSION_DIR_RE.match(latest):
        raise ValueError(
            "latest version %r is not X.Y.Z" % (latest,))
    return {
        "latest": latest,
        "versions": [
            {
                "version": v,
                "url": "/%s/" % v,
                "pdf": "/%s/openbricks-docs.pdf" % v,
            }
            for v in versions
        ],
    }


def main(argv):
    if len(argv) != 3:
        print(__doc__.strip(), file=sys.stderr)
        return 2
    manifest = build_manifest(collect_versions(argv[1]), argv[2])
    json.dump(manifest, sys.stdout, indent=2)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
