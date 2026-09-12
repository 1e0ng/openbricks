# SPDX-License-Identifier: MIT
"""What the CLI remembers about each hub between sessions.

Today that is one fact per hub name: the firmware version it reported
last time. ``run`` and ``upload`` use it to skip the in-session
version probe (a whole raw-paste round trip) and stage the compiled
program straight away; the staged program prints its version first
and refuses to run compiled code on firmware that cannot, so a hub
that was re-flashed to something older is caught in-session, never
guessed about.

The cache is best-effort: a missing, unreadable or unwritable file
costs one probe, never an error. It lives under
``$OPENBRICKS_CACHE_DIR`` when set, else ``$XDG_CACHE_HOME/openbricks``
or ``~/.cache/openbricks`` — the same tree the sim keeps its binary in.
"""

import json
import os
import time

CACHE_ENV = "OPENBRICKS_CACHE_DIR"
_FILE = "hubs.json"


def cache_dir():
    env = os.environ.get(CACHE_ENV)
    if env:
        return os.path.expanduser(env)
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(
        os.path.expanduser("~"), ".cache")
    return os.path.join(base, "openbricks")


def _path():
    return os.path.join(cache_dir(), _FILE)


def _load():
    try:
        with open(_path(), "r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def parse_version(text):
    """``(major, minor, patch)`` from ``"X.Y.Z"`` (a provenance suffix
    such as ``" (official)"`` is ignored), or None."""
    if not isinstance(text, str):
        return None
    parts = text.strip().split(" ")[0].split(".")
    if len(parts) < 3:
        return None
    try:
        return tuple(int(p) for p in parts[:3])
    except ValueError:
        return None


def firmware_version(name):
    """The firmware version ``name`` reported last time, as a tuple, or
    None when the hub has never been seen (or the entry is unusable)."""
    entry = _load().get(name)
    if not isinstance(entry, dict):
        return None
    return parse_version(entry.get("firmware"))


def remember_firmware(name, version_text):
    """Record the version a hub just reported. Silent on failure."""
    if parse_version(version_text) is None:
        return
    data = _load()
    data[name] = {"firmware": version_text.strip().split(" ")[0],
                  "seen": int(time.time())}
    try:
        os.makedirs(cache_dir(), exist_ok=True)
        tmp = _path() + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, sort_keys=True)
        os.replace(tmp, _path())
    except (OSError, ValueError):
        pass


def forget(name):
    """Drop what is remembered about a hub (tests, and a hub that was
    re-flashed under the same name)."""
    data = _load()
    if name in data:
        del data[name]
        try:
            with open(_path(), "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, sort_keys=True)
        except (OSError, ValueError):
            pass
