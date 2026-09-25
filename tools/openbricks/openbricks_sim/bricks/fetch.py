# SPDX-License-Identifier: MIT
"""One LEGO part by number, fetched from ldraw.org file by file and
converted for the workbench.

The shipped library holds a curated set; the whole LDraw library is
145 MB. A part the library lacks needs only its own file and the
handful of subparts and primitives it references (a median part is 15
files, a few dozen kilobytes), and ldraw.org serves every file of the
library on its own::

    https://library.ldraw.org/library/official/parts/2458.dat
    https://library.ldraw.org/library/official/parts/s/3001s01.dat
    https://library.ldraw.org/library/official/p/stud.dat
    https://library.ldraw.org/library/official/p/48/4-4cyli.dat
    https://library.ldraw.org/library/unofficial/parts/...   (the tracker)

:class:`FetchingLibrary` is the converter's library that downloads
what it cannot find, into the same cache layout ``bricks fetch``
unpacks the whole library into, so a full library is used as is and a
sparse one grows as parts are asked for. A missing reference is an
error, never a silently thinner mesh; a 404 on the part itself is
:class:`NotInLibrary`. ldraw.org allows 60 requests a minute; a 429 is
waited out once.

:func:`fetch_part` returns a one-part bundle, with the colours the part
comes in from Rebrickable's tables (cached too) unless asked not to;
``python -m openbricks_sim.bricks.fetch NUMBER --out FILE`` is what
the sim runs, speaking JSON lines on stdout as the run server does.
"""
import json
import os
import pathlib
import re
import sys
import time
import urllib.error
import urllib.request

from openbricks_sim import bricks

OFFICIAL = "https://library.ldraw.org/library/official/"
UNOFFICIAL = "https://library.ldraw.org/library/unofficial/"
RATE_LIMIT_WAIT_S = 65
FETCHED_SOURCE = "LDraw parts library, CC BY 2.0 / CC BY 4.0 (ldraw.org), fetched file by file"


class NotInLibrary(Exception):
    """ldraw.org has no part of that number, official or unofficial."""

    def __init__(self, number, urls):
        super().__init__("ldraw.org has no part %s (tried %s)" % (number, ", ".join(urls)))
        self.number = number
        self.urls = urls


class FetchError(Exception):
    """A file could not be fetched: the URL and why."""

    def __init__(self, url, reason):
        super().__init__("%s: %s" % (url, reason))
        self.url = url
        self.reason = reason


def relative_paths(name):
    """Where a referenced file may live in the library, in the order to
    try: ``s\\x`` is a subpart, ``48\\x`` and ``8\\x`` are hi- and lo-res
    primitives, a bare number is a part before a primitive, any other
    bare name a primitive before a part."""
    key = name.strip().replace("\\", "/").lower()
    if key.startswith("s/"):
        return ["parts/" + key]
    if key.startswith("48/") or key.startswith("8/"):
        return ["p/" + key]
    if re.match(r"^[0-9]+[a-z0-9]*\.dat$", key):
        return ["parts/" + key, "p/" + key]
    return ["p/" + key, "parts/" + key]


def data_dir():
    """Where the sim keeps what the user adds: ``$OPENBRICKS_DATA_DIR``,
    else ``$XDG_DATA_HOME/openbricks``, else ``~/.local/share/openbricks``
    (the sim's ``markers::data_dir`` reads the same three)."""
    env = os.environ.get("OPENBRICKS_DATA_DIR")
    if env:
        return pathlib.Path(env).expanduser()
    xdg = os.environ.get("XDG_DATA_HOME")
    if xdg:
        return pathlib.Path(xdg) / "openbricks"
    return pathlib.Path(os.path.expanduser("~")) / ".local" / "share" / "openbricks"


def bricks_dir():
    """Where fetched parts are kept for every later launch."""
    return data_dir() / "bricks"


def rebrickable_dir():
    """Where Rebrickable's tables are cached, beside the LDraw cache."""
    return bricks.ldraw_dir().parent / "rebrickable"


def _get(url, opener, sleep, say, waited):
    """GET ``url`` with our agent: the bytes, or None on 404; waits out
    one 429 (``waited`` remembers whether that was done already)."""
    req = urllib.request.Request(url, headers={"User-Agent": bricks.USER_AGENT})
    try:
        with opener(req) as resp:
            return resp.read()
    except urllib.error.HTTPError as e:
        if e.code == 404:
            return None
        if e.code == 429 and not waited[0]:
            waited[0] = True
            try:
                wait = min(int(e.headers.get("Retry-After", RATE_LIMIT_WAIT_S)), RATE_LIMIT_WAIT_S)
            except (TypeError, ValueError):
                wait = RATE_LIMIT_WAIT_S
            say("rate limited by ldraw.org, waiting %d s" % wait)
            sleep(wait)
            return _get(url, opener, sleep, say, waited)
        raise FetchError(url, "HTTP %d" % e.code)
    except urllib.error.URLError as e:
        raise FetchError(url, str(e.reason))


class FetchingLibrary(object):
    """The converter's library over ``root`` that fetches what it lacks
    from ldraw.org into ``root``, file by file. Built lazily on top of
    :class:`openbricks_sim.bricks.ldraw.Library` (which needs numpy)."""

    def __init__(self, root, opener=None, say=None, sleep=time.sleep):
        from openbricks_sim.bricks import ldraw
        self.root = pathlib.Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.lib = ldraw.Library(str(self.root))
        self.opener = opener or urllib.request.urlopen
        self.say = say or (lambda s: None)
        self.sleep = sleep
        self.fetched = []
        self.requests = 0
        self._waited = [False]

    # the converter's protocol: resolve(name) -> path or None, parse(path)
    def parse(self, path):
        return self.lib.parse(path)

    def resolve(self, name):
        path = self.lib.resolve(name)
        if path is not None:
            return path
        return self.fetch(name)

    def fetch(self, name):
        """The file for ``name`` from ldraw.org, saved under the library's
        layout and indexed; :class:`NotInLibrary` when neither the
        official library nor the tracker has it."""
        tried = []
        for rel in relative_paths(name):
            for base in (OFFICIAL, UNOFFICIAL):
                url = base + rel
                tried.append(url)
                self.requests += 1
                data = _get(url, self.opener, self.sleep, self.say, self._waited)
                if data is None:
                    continue
                target = self.root / pathlib.Path(rel)
                target.parent.mkdir(parents=True, exist_ok=True)
                tmp = target.with_name(target.name + ".part")
                tmp.write_bytes(data)
                os.replace(str(tmp), str(target))
                key = rel[len("parts/"):] if rel.startswith("parts/") else rel[len("p/"):]
                self.lib.index[key] = str(target)
                self.fetched.append(rel)
                self.say("fetched %s" % rel)
                return str(target)
        raise NotInLibrary(name[:-4] if name.lower().endswith(".dat") else name, tried)


def colours_for(number, opener=None, say=None, cache=None):
    """The colours ``number`` comes in and the palette they need, from
    Rebrickable's tables (downloaded once into ``cache``, reused after)."""
    from openbricks_sim.bricks import rebrickable
    opener = opener or urllib.request.urlopen
    say = say or (lambda s: None)
    cache = pathlib.Path(cache) if cache else rebrickable_dir()
    cache.mkdir(parents=True, exist_ok=True)
    rows = {}
    for table in ("colors", "elements"):
        path = cache / (table + ".csv.gz")
        if not path.exists():
            url = rebrickable.DOWNLOADS + table + ".csv.gz"
            say("fetching " + url)
            req = urllib.request.Request(url, headers={"User-Agent": bricks.USER_AGENT})
            try:
                with opener(req) as resp:
                    data = resp.read()
            except urllib.error.URLError as e:
                raise FetchError(url, str(getattr(e, "reason", e)))
            tmp = path.with_name(path.name + ".part")
            tmp.write_bytes(data)
            os.replace(str(tmp), str(path))
        rows[table] = rebrickable.fetch_table(table, opener=lambda req, p=path: open(str(p), "rb"))
    data = rebrickable.build([number], rows["colors"], rows["elements"], rebrickable.rebrickable_numbers(bricks.load_sets()))
    return data["parts"].get(number, {}), data["palette"]


def fetch_part(number, root=None, opener=None, say=None, colors=True, sleep=time.sleep, colors_cache=None):
    """A one-part bundle for ``number``: its files from ldraw.org (into
    ``root``, the LDraw cache by default), converted; with its colours
    from Rebrickable unless ``colors`` is False. Raises
    :class:`NotInLibrary` or :class:`FetchError`."""
    from openbricks_sim.bricks import ldraw
    say = say or (lambda s: None)
    lib = FetchingLibrary(root or bricks.ldraw_dir(), opener=opener, say=say, sleep=sleep)
    record = ldraw.convert_part(lib, ldraw.Builder(lib), number)
    if record is None:
        raise FetchError(OFFICIAL + "parts/" + number + ".dat", "the file has no faces")
    record["fetched"] = True
    bundle = {"format": ldraw.BUNDLE_FORMAT, "source": FETCHED_SOURCE, "units": {"length": "mm", "mass": "g"},
              "parts": {number: record}, "files": len(lib.fetched)}
    if colors:
        record["colors"], bundle["colors"] = colours_for(number, opener=opener, say=say, cache=colors_cache)
    say("%s: %s, %d triangles, %d files fetched, %d colours" % (
        number, record["name"], record["mesh"]["tris"], len(lib.fetched), len(record.get("colors", {}))))
    return bundle


def _emit(**ev):
    sys.stdout.write(json.dumps(ev) + "\n")
    sys.stdout.flush()


def main(argv=None):
    """``python -m openbricks_sim.bricks.fetch NUMBER --out FILE [--ldraw DIR] [--no-colors]``:
    JSON lines on stdout (``log``, then ``fetched`` or ``error``); exit 2
    when ldraw.org has no such part, 1 on any other failure."""
    argv = sys.argv[1:] if argv is None else list(argv)
    number, out, root, colors = None, None, None, True
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--out" and i + 1 < len(argv):
            out = argv[i + 1]
            i += 2
        elif a == "--ldraw" and i + 1 < len(argv):
            root = argv[i + 1]
            i += 2
        elif a == "--no-colors":
            colors = False
            i += 1
        elif not a.startswith("-") and number is None:
            number = a
            i += 1
        else:
            number = None
            break
    if not number or not out:
        print("usage: python -m openbricks_sim.bricks.fetch NUMBER --out FILE [--ldraw DIR] [--no-colors]", file=sys.stderr)
        return 2
    try:
        bundle = fetch_part(number, root=root, say=lambda s: _emit(ev="log", text=s), colors=colors)
    except NotInLibrary as e:
        _emit(ev="error", text=str(e))
        return 2
    except (FetchError, ImportError, OSError) as e:
        _emit(ev="error", text=str(e))
        return 1
    out_path = pathlib.Path(out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_path.with_name(out_path.name + ".part")
    tmp.write_text(json.dumps(bundle, separators=(",", ":")))
    os.replace(str(tmp), str(out_path))
    rec = bundle["parts"][number]
    _emit(ev="fetched", number=number, name=rec["name"], files=bundle["files"], colors=len(rec.get("colors", {})),
          out=str(out_path))
    return 0


if __name__ == "__main__":             # pragma: no cover
    sys.exit(main())
