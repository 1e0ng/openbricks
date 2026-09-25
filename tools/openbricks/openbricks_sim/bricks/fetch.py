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
error naming the part that needs it, never a silently thinner mesh;
a 404 on the part itself is :class:`NotInLibrary`. ldraw.org allows
60 requests a minute; every 429 is waited out (a part of two hundred
files takes a few of them), up to :data:`RATE_LIMIT_WAITS` times.

:func:`fetch_part` returns a one-part bundle, with the colours the part
comes in from Rebrickable's tables (cached too) unless asked not to;
``python -m openbricks_sim.bricks.fetch NUMBER --out FILE`` is what
the sim runs, speaking JSON lines on stdout as the run server does.
"""
import http.client
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
# the longest one 429 is waited out, and how many one fetch waits out
# before it gives up (ldraw.org's window is a minute; a part of two
# hundred files needs three of them)
RATE_LIMIT_WAIT_S = 65
RATE_LIMIT_WAITS = 10
# a connection that goes quiet for this long is an error, not a wait
FETCH_TIMEOUT_S = 30
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


class MissingReference(FetchError):
    """A file the part needs that ldraw.org has not, official or on the
    tracker: the part is incomplete there, which is not the same as
    absent."""

    def __init__(self, asked, name, urls):
        Exception.__init__(self, "%s needs %s, which ldraw.org has not (tried %s)" % (asked, name, ", ".join(urls)))
        self.url = urls[-1]
        self.reason = "not on ldraw.org"
        self.asked = asked
        self.name = name
        self.urls = urls


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
    (:func:`openbricks_sim.props.data_dir`, the one rule the sim's
    ``markers::data_dir`` applies too)."""
    from openbricks_sim import props
    return props.data_dir()


def bricks_dir():
    """Where fetched parts are kept for every later launch."""
    return bricks.user_bricks_dir()


def rebrickable_dir():
    """Where Rebrickable's tables are cached: under the cache directory,
    whatever ``$OPENBRICKS_LDRAW_DIR`` points at."""
    return bricks.cache_dir() / "rebrickable"


def is_ldraw_file(data):
    """Whether ``data`` starts like an LDraw file: its first non-blank
    line is the type-0 title. A 200 that carries an HTML page is not."""
    first = data.lstrip()[:2]
    return first[:1] == b"0" and (len(first) == 1 or first[1:2] in (b" ", b"\t", b"\r", b"\n"))


def _retry_after(headers):
    try:
        wait = int(headers.get("Retry-After", RATE_LIMIT_WAIT_S))
    except (TypeError, ValueError):
        wait = RATE_LIMIT_WAIT_S
    return max(1, min(wait, RATE_LIMIT_WAIT_S))


def _get(url, opener, sleep, say, waits):
    """GET ``url`` with our agent: the bytes, or None on 404. A 429 is
    waited out for as long as ldraw.org asks (at most
    :data:`RATE_LIMIT_WAIT_S`), ``waits`` counting them towards
    :data:`RATE_LIMIT_WAITS` for the library; any other trouble,
    including a connection that stalls or a body cut short, is a
    :class:`FetchError` naming the URL."""
    req = urllib.request.Request(url, headers={"User-Agent": bricks.USER_AGENT})
    while True:
        try:
            with opener(req, timeout=FETCH_TIMEOUT_S) as resp:
                return resp.read()
        except urllib.error.HTTPError as e:
            if e.code == 404:
                return None
            if e.code == 429 and waits[0] < RATE_LIMIT_WAITS:
                waits[0] += 1
                wait = _retry_after(e.headers)
                say("rate limited by ldraw.org, waiting %d s (%d of %d)" % (wait, waits[0], RATE_LIMIT_WAITS))
                sleep(wait)
                continue
            raise FetchError(url, "HTTP %d" % e.code)
        except urllib.error.URLError as e:
            raise FetchError(url, str(e.reason))
        except (http.client.HTTPException, OSError) as e:
            raise FetchError(url, "%s: %s" % (type(e).__name__, e))


class FetchingLibrary(object):
    """The converter's library over ``root`` that fetches what it lacks
    from ldraw.org into ``root``, file by file. Built lazily on top of
    :class:`openbricks_sim.bricks.ldraw.Library` (which needs numpy).
    With ``force``, the part files it already has (under ``parts/``: the
    part and its subparts, not the primitives) are fetched again.
    ``asked`` is the number being fetched: a miss on it is
    :class:`NotInLibrary`, a miss on anything else
    :class:`MissingReference`."""

    def __init__(self, root, opener=None, say=None, sleep=time.sleep, force=False):
        from openbricks_sim.bricks import ldraw
        self.root = pathlib.Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.lib = ldraw.Library(str(self.root))
        self.opener = opener or urllib.request.urlopen
        self.say = say or (lambda s: None)
        self.sleep = sleep
        self.force = force
        self.asked = None
        self.fetched = []
        self.refreshed = set()
        self.requests = 0
        self._waits = [0]

    # the converter's protocol: resolve(name) -> path or None, parse(path)
    def parse(self, path):
        return self.lib.parse(path)

    def resolve(self, name):
        path = self.lib.resolve(name)
        if path is None:
            return self.fetch(name)
        if self.force and self._key(name) not in self.refreshed and self._is_part_file(path):
            return self.fetch(name)
        return path

    @staticmethod
    def _key(name):
        return name.strip().replace("\\", "/").lower()

    def _is_part_file(self, path):
        rel = os.path.relpath(os.path.abspath(path), os.path.abspath(str(self.root)))
        return rel.split(os.sep)[0] == "parts"

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
                data = _get(url, self.opener, self.sleep, self.say, self._waits)
                if data is None:
                    continue
                if not is_ldraw_file(data):
                    raise FetchError(url, "not an LDraw file")
                target = self.root / pathlib.Path(rel)
                target.parent.mkdir(parents=True, exist_ok=True)
                tmp = target.with_name(target.name + ".part")
                tmp.write_bytes(data)
                os.replace(str(tmp), str(target))
                key = rel[len("parts/"):] if rel.startswith("parts/") else rel[len("p/"):]
                self.lib.index[key] = str(target)
                self.lib.cache.pop(str(target), None)
                self.fetched.append(rel)
                self.refreshed.add(self._key(name))
                self.say("fetched %s" % rel)
                return str(target)
        number = name.strip()[:-4] if name.lower().strip().endswith(".dat") else name.strip()
        if self.asked is None or number.lower() == self.asked.lower():
            raise NotInLibrary(number, tried)
        raise MissingReference(self.asked, name.strip(), tried)


def colours_for(number, opener=None, say=None, cache=None, also=(), force=False):
    """The colours ``number`` comes in and the palette they need, from
    Rebrickable's tables (downloaded once into ``cache``, reused after;
    again with ``force``). The part is looked up by ``number``, then
    by each of ``also`` (the number LDraw resolved it to) — each under
    Rebrickable's own spelling of it, then as a design id."""
    from openbricks_sim.bricks import rebrickable
    opener = opener or urllib.request.urlopen
    say = say or (lambda s: None)
    cache = pathlib.Path(cache) if cache else rebrickable_dir()
    cache.mkdir(parents=True, exist_ok=True)
    rows = {}
    for table in ("colors", "elements"):
        path = cache / (table + ".csv.gz")
        if force or not path.exists():
            url = rebrickable.DOWNLOADS + table + ".csv.gz"
            say("fetching " + url)
            req = urllib.request.Request(url, headers={"User-Agent": bricks.USER_AGENT})
            try:
                with opener(req, timeout=FETCH_TIMEOUT_S) as resp:
                    data = resp.read()
            except urllib.error.URLError as e:
                raise FetchError(url, str(getattr(e, "reason", e)))
            except (http.client.HTTPException, OSError) as e:
                raise FetchError(url, "%s: %s" % (type(e).__name__, e))
            tmp = path.with_name(path.name + ".part")
            tmp.write_bytes(data)
            os.replace(str(tmp), str(path))
        rows[table] = rebrickable.fetch_table(table, opener=lambda req, p=path, **kw: open(str(p), "rb"))
    numbers = [number] + [n for n in also if n and n != number]
    data = rebrickable.build(numbers, rows["colors"], rows["elements"], rebrickable.rebrickable_numbers(bricks.load_sets()))
    for n in numbers:
        entry = data["parts"].get(n)
        if entry:
            return entry, {cid: data["palette"][cid] for cid in entry}
    return {}, {}


def fetch_part(number, root=None, opener=None, say=None, colors=True, sleep=time.sleep, colors_cache=None, force=False):
    """A one-part bundle for ``number``: its files from ldraw.org (into
    ``root``, the LDraw cache by default), converted; with its colours
    from Rebrickable unless ``colors`` is False. With ``force`` the
    part's files and the tables are fetched again. Raises
    :class:`NotInLibrary` or :class:`FetchError`. A part Rebrickable
    lists no colours for says so in the bundle's ``note``."""
    from openbricks_sim.bricks import ldraw
    say = say or (lambda s: None)
    lib = FetchingLibrary(root or bricks.ldraw_dir(), opener=opener, say=say, sleep=sleep, force=force)
    lib.asked = number
    record = ldraw.convert_part(lib, ldraw.Builder(lib), number)
    if record is None:
        raise FetchError(OFFICIAL + "parts/" + number + ".dat", "the file has no faces")
    record["fetched"] = True
    bundle = {"format": ldraw.BUNDLE_FORMAT, "source": FETCHED_SOURCE, "units": {"length": "mm", "mass": "g"},
              "parts": {number: record}, "files": len(lib.fetched)}
    if colors:
        record["colors"], bundle["colors"] = colours_for(number, opener=opener, say=say, cache=colors_cache,
                                                         also=[record["ldraw"]], force=force)
        if not record["colors"]:
            names = number if record["ldraw"] == number else "%s or %s" % (number, record["ldraw"])
            bundle["note"] = "Rebrickable lists no colours for %s" % names
            say(bundle["note"])
    say("%s: %s, %d triangles, %d files fetched, %d colours" % (
        number, record["name"], record["mesh"]["tris"], len(lib.fetched), len(record.get("colors", {}))))
    return bundle


def _emit(**ev):
    sys.stdout.write(json.dumps(ev) + "\n")
    sys.stdout.flush()


def main(argv=None):
    """``python -m openbricks_sim.bricks.fetch NUMBER --out FILE [--ldraw DIR] [--no-colors] [--force]``:
    JSON lines on stdout (``log``, then ``fetched`` or ``error``); exit 2
    when ldraw.org has no such part, 1 on any other failure."""
    argv = sys.argv[1:] if argv is None else list(argv)
    number, out, root, colors, force = None, None, None, True, False
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
        elif a == "--force":
            force = True
            i += 1
        elif not a.startswith("-") and number is None:
            number = a
            i += 1
        else:
            number = None
            break
    if not number or not out:
        print("usage: python -m openbricks_sim.bricks.fetch NUMBER --out FILE [--ldraw DIR] [--no-colors] [--force]",
              file=sys.stderr)
        return 2
    try:
        bundle = fetch_part(number, root=root, say=lambda s: _emit(ev="log", text=s), colors=colors, force=force)
        out_path = pathlib.Path(out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = out_path.with_name(out_path.name + ".part")
        tmp.write_text(json.dumps(bundle, separators=(",", ":")))
        os.replace(str(tmp), str(out_path))
    except NotInLibrary as e:
        _emit(ev="error", text=str(e))
        return 2
    except (FetchError, ImportError, OSError) as e:
        _emit(ev="error", text=str(e))
        return 1
    rec = bundle["parts"][number]
    ev = dict(ev="fetched", number=number, name=rec["name"], files=bundle["files"], colors=len(rec.get("colors", {})),
              out=str(out_path))
    if bundle.get("note"):
        ev["note"] = bundle["note"]
    _emit(**ev)
    return 0


if __name__ == "__main__":             # pragma: no cover
    sys.exit(main())
