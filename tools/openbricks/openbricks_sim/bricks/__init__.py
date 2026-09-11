# SPDX-License-Identifier: MIT
"""The brick library: exact LEGO Technic geometry for the workbench.

Two sources feed the workbench's library:

* ``technic_bundle.json.zlib`` ships in the wheel — a curated set of
  popular Technic parts (``technic_parts.txt``) converted from the
  LDraw parts library by :mod:`openbricks_sim.bricks.ldraw`, with
  BrickLink catalogue weights (``weights.json``) where known.
* the full LDraw library, fetched on demand into a cache directory by
  ``openbricks bricks fetch`` (145 MB download, 600 MB unpacked, every
  LEGO part ever catalogued); ``openbricks bricks convert`` then turns
  any part number into a record the workbench can load.

LDraw is CC BY 2.0 / CC BY 4.0 (https://www.ldraw.org); the bundle
carries the attribution. LEGO® is a trademark of the LEGO Group, which
does not sponsor or endorse this tool. This module has no third-party
dependencies; the converter needs numpy (``pip install openbricks[sim]``).
"""
import base64
import json
import os
import pathlib
import shutil
import tempfile
import urllib.request
import zipfile
import zlib

try:                                    # Python ≥ 3.9
    from importlib.resources import files as _files
except ImportError:                     # pragma: no cover
    _files = None

LDRAW_URL = "https://library.ldraw.org/library/updates/complete.zip"
BUNDLE_NAME = "technic_bundle.json.zlib"
_HERE = pathlib.Path(__file__).resolve().parent


def data_path(name):
    """A data file shipped next to this module."""
    if _files is not None:
        return pathlib.Path(str(_files(__package__) / name))
    return _HERE / name                 # pragma: no cover


def bundle_bytes():
    """The shipped bundle, zlib-compressed JSON."""
    return data_path(BUNDLE_NAME).read_bytes()


def bundle_b64():
    """The shipped bundle as the page embeds it."""
    return base64.b64encode(bundle_bytes()).decode()


def load_bundle():
    """The shipped bundle as a dict (``parts`` keyed by LDraw number)."""
    return json.loads(zlib.decompress(bundle_bytes()).decode())


def merge_bundles(base, extras):
    """``base`` plus the parts of each extra bundle (later ones win)."""
    out = {k: v for k, v in base.items() if k != "parts"}
    out["parts"] = dict(base.get("parts", {}))
    for extra in extras:
        if not isinstance(extra, dict) or not isinstance(extra.get("parts"), dict):
            raise ValueError("a brick bundle is a JSON object with a \"parts\" object (see openbricks bricks convert)")
        out["parts"].update(extra["parts"])
    return out


def encode_bundle(bundle):
    """A bundle dict → the base64 zlib text the page inflates."""
    return base64.b64encode(zlib.compress(json.dumps(bundle, separators=(",", ":")).encode(), 9)).decode()


def ldraw_dir():
    """Where ``openbricks bricks fetch`` puts the library:
    ``$OPENBRICKS_LDRAW_DIR``, else ``$XDG_CACHE_HOME/openbricks/ldraw``,
    else ``~/.cache/openbricks/ldraw``."""
    env = os.environ.get("OPENBRICKS_LDRAW_DIR")
    if env:
        return pathlib.Path(env).expanduser()
    cache = os.environ.get("XDG_CACHE_HOME") or os.path.join(os.path.expanduser("~"), ".cache")
    return pathlib.Path(cache) / "openbricks" / "ldraw"


def library_present(root):
    root = pathlib.Path(root)
    return (root / "parts").is_dir() and (root / "p").is_dir()


def fetch_library(dest=None, url=LDRAW_URL, force=False, progress=None, opener=urllib.request.urlopen,
                  progress_every=10 * 1024 * 1024):
    """Download ``complete.zip`` and unpack ``parts/`` and ``p/`` into
    ``dest`` (default :func:`ldraw_dir`). Returns the library root.
    Skips the download when the library is already there unless
    ``force``. ``progress`` receives short status lines."""
    say = progress or (lambda s: None)
    root = pathlib.Path(dest) if dest else ldraw_dir()
    if library_present(root) and not force:
        say("LDraw library already at %s (use --force to refresh)" % root)
        return root
    root.mkdir(parents=True, exist_ok=True)
    tmp_zip = root / "complete.zip.part"
    say("downloading %s" % url)
    with opener(url) as resp, open(tmp_zip, "wb") as out:
        total = 0
        next_mark = progress_every
        while True:
            chunk = resp.read(1024 * 1024)
            if not chunk:
                break
            out.write(chunk)
            total += len(chunk)
            if total >= next_mark:
                say("  %.1f MB" % (total / (1024 * 1024)))
                next_mark += progress_every
    say("unpacking into %s" % root)
    staging = pathlib.Path(tempfile.mkdtemp(prefix="ldraw-", dir=str(root)))
    try:
        with zipfile.ZipFile(tmp_zip) as zf:
            members = [m for m in zf.namelist() if m.startswith("ldraw/") and not m.endswith("/")]
            for m in members:
                rel = m[len("ldraw/"):]
                target = staging / rel
                target.parent.mkdir(parents=True, exist_ok=True)
                with zf.open(m) as src, open(target, "wb") as dst:
                    shutil.copyfileobj(src, dst)
        if not library_present(staging):
            raise RuntimeError("the archive at %s has no ldraw/parts and ldraw/p directories" % url)
        for sub in ("parts", "p"):
            final = root / sub
            if final.exists():
                shutil.rmtree(final)
            shutil.move(str(staging / sub), str(final))
        for extra in staging.iterdir():        # licence, readme, LDConfig
            final = root / extra.name
            if final.exists():
                if final.is_dir():
                    shutil.rmtree(final)
                else:
                    final.unlink()
            shutil.move(str(extra), str(final))
    finally:
        shutil.rmtree(staging, ignore_errors=True)
        if tmp_zip.exists():
            tmp_zip.unlink()
    n_parts = sum(1 for p in (root / "parts").iterdir() if p.suffix.lower() == ".dat")
    say("done: %d part files at %s" % (n_parts, root))
    return root
