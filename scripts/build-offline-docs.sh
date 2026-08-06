#!/usr/bin/env bash
# Build the offline documentation bundle that ``openbricks docs``
# opens.
#
# It is the SAME Sphinx build as docs.openbricks.dev — same sources,
# same extensions, same autodoc — so the API reference is included
# and cannot drift from the website. Before this, the CLI shipped a
# verbatim copy of the hand-written ``.md`` guides and re-rendered
# them with a markdown library, which meant everything generated
# from docstrings was simply absent.
#
# Three things are stripped, in descending order of size and
# ascending order of consequence:
#
#   .doctrees/      Sphinx's build cache (~6 MB). Never part of the
#                   published site; pure intermediate.
#   _modules/       ``viewcode`` re-rendering the whole source tree as
#                   HTML (~1.3 MB; st3215.html alone is 252 KB). Only
#                   the "[source]" links beside API entries break, and
#                   the source is in the repo.
#   legacy fonts    The theme ships Lato + FontAwesome as ttf/eot/svg/
#                   woff AND woff2, in two locations. Nothing reads
#                   them: docs/_static/custom.css sets a system font
#                   stack.
#
# 19 MB raw -> ~1.5 MB tree -> ~308 KB zipped.
#
# Usage:  scripts/build-offline-docs.sh [OUT_ZIP]
#         PYTHON=python3.11 scripts/build-offline-docs.sh
#
# PYTHON selects the interpreter that has Sphinx and can
# import ``openbricks`` (autodoc needs both).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/tools/openbricks/openbricks_dev/_docs/offline-docs.zip}"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

# -E: don't reuse a cached environment, so the bundle never depends on
# a stale .doctrees from someone's earlier local build.
"${PYTHON:-python3}" -m sphinx -b html -E -q "$ROOT/docs" "$WORK/html"

rm -rf "$WORK/html/.doctrees" "$WORK/html/_modules"
find "$WORK/html/_static" -type d -name fonts -prune -exec rm -rf {} + 2>/dev/null || true
find "$WORK/html/_static" -type f \
     \( -name '*.ttf' -o -name '*.eot' -o -name '*.svg' \
        -o -name '*.woff' -o -name '*.woff2' \) -delete

# Record a fingerprint of the SOURCES this bundle was built from —
# CI compares it against the checkout, so editing a docs/ page
# without rebuilding the bundle fails the drift check even though
# the page SET didn't change. GIT-TRACKED files only: a bare find
# also swept up untracked local Sphinx output (docs/_build) and the
# fingerprint never matched a clean CI checkout. (docs/ only:
# autodoc'd docstrings aren't captured, so a docstring-only change
# still needs the page-set check or a manual rebuild to surface.)
( cd "$ROOT" && git ls-files docs | LC_ALL=C sort \
    | xargs shasum -a 256 | shasum -a 256 | cut -d' ' -f1 ) \
    > "$WORK/html/.source-hash"

# Sorted entries and fixed timestamps, so rebuilding on the SAME
# machine is byte-stable and a diff shows real changes. Do not rely
# on byte-identity ACROSS machines: Sphinx emits slightly different
# HTML between versions, so CI checks page COVERAGE instead.
mkdir -p "$(dirname "$OUT")"
rm -f "$OUT"
find "$WORK/html" -exec touch -t 200001010000 {} +
( cd "$WORK/html" && find . -type f | LC_ALL=C sort | zip -qX "$OUT" -@ )

printf 'offline docs: %s (%s)\n' "$OUT" "$(du -h "$OUT" | cut -f1)"
