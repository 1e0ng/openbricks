# SPDX-License-Identifier: MIT
"""One part fetched from ldraw.org file by file (``openbricks_sim.bricks.fetch``)."""
import gzip
import http.client
import io
import json
import os
import tempfile
import unittest
from unittest import mock
import urllib.error
from contextlib import redirect_stderr, redirect_stdout

from openbricks_sim import bricks, props
from openbricks_sim.bricks import fetch

try:
    import numpy  # noqa: F401  - the converter's dependency
    HAVE_NUMPY = True
except ImportError:                      # pragma: no cover
    HAVE_NUMPY = False

from tests.ldraw_fixture import write_mini_library


class _Response(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def _http_error(url, code, headers=None):
    return urllib.error.HTTPError(url, code, "nope", headers or {}, io.BytesIO(b"<html>not here</html>"))


class _Site(object):
    """A stand-in ldraw.org: the mini library's files under the official
    tree, one under the tracker, a 429 the first time a given file is
    asked for when ``throttle`` names it, and a 429 (once) on every
    ``throttle_every``-th request."""

    def __init__(self, root, unofficial=(), throttle=(), throttle_every=None):
        self.root = root
        self.unofficial = set(unofficial)
        self.throttle = set(throttle)
        self.throttle_every = throttle_every
        self.requests = []
        self.timeouts = []
        self.throttled = set()
        self.count = 0

    def __call__(self, req, timeout=None):
        url = req.full_url
        self.requests.append(req)
        self.timeouts.append(timeout)
        self.count += 1
        if self.throttle_every and self.count % self.throttle_every == 0 and self.count not in self.throttled:
            self.throttled.add(self.count)
            raise _http_error(url, 429, {"Retry-After": "3"})
        for base in (fetch.OFFICIAL, fetch.UNOFFICIAL):
            if url.startswith(base):
                rel = url[len(base):]
                if rel in self.throttle and rel not in self.throttled:
                    self.throttled.add(rel)
                    raise _http_error(url, 429, {"Retry-After": "3"})
                official = base == fetch.OFFICIAL
                if (rel in self.unofficial) == official:
                    raise _http_error(url, 404)
                path = os.path.join(self.root, *rel.split("/"))
                if not os.path.exists(path):
                    raise _http_error(url, 404)
                with open(path, "rb") as fh:
                    return _Response(fh.read())
        raise AssertionError("unexpected URL " + url)


class PathRuleTests(unittest.TestCase):
    def test_where_a_reference_may_live(self):
        self.assertEqual(fetch.relative_paths("s\\3001s01.dat"), ["parts/s/3001s01.dat"])
        self.assertEqual(fetch.relative_paths("48\\4-4cyli.dat"), ["p/48/4-4cyli.dat"])
        self.assertEqual(fetch.relative_paths("8\\4-4cyli.dat"), ["p/8/4-4cyli.dat"])
        self.assertEqual(fetch.relative_paths("2458.dat"), ["parts/2458.dat", "p/2458.dat"])
        self.assertEqual(fetch.relative_paths("3648b.dat"), ["parts/3648b.dat", "p/3648b.dat"])
        self.assertEqual(fetch.relative_paths("stud.dat"), ["p/stud.dat", "parts/stud.dat"])
        self.assertEqual(fetch.relative_paths("4-4CYLI.DAT"), ["p/4-4cyli.dat", "parts/4-4cyli.dat"])

    def test_what_an_ldraw_file_starts_like(self):
        self.assertTrue(fetch.is_ldraw_file(b"0 Brick  1 x  2 with Pin\r\n0 Name: 2458.dat\r\n"))
        self.assertTrue(fetch.is_ldraw_file(b"\n0\tTitle\n"))
        self.assertTrue(fetch.is_ldraw_file(b"0\n1 16 0 0 0 1 0 0 0 1 0 0 0 1 stud.dat\n"))
        self.assertFalse(fetch.is_ldraw_file(b"<!DOCTYPE html>\n<html>not here</html>"))
        self.assertFalse(fetch.is_ldraw_file(b"0x1 not a title"))
        self.assertFalse(fetch.is_ldraw_file(b""))

    def test_the_data_dir_is_the_one_rule_the_sim_applies(self):
        with mock.patch.dict(os.environ, {"OPENBRICKS_DATA_DIR": "/x/data", "XDG_DATA_HOME": "/y"}):
            self.assertEqual(fetch.data_dir(), props.data_dir())
            self.assertEqual(str(fetch.data_dir()), os.path.join("/x", "data"))
            self.assertEqual(str(fetch.bricks_dir()), os.path.join("/x", "data", "bricks"))
            self.assertEqual(fetch.bricks_dir(), bricks.user_bricks_dir())
        with mock.patch.dict(os.environ, {"XDG_DATA_HOME": "/y"}, clear=False):
            os.environ.pop("OPENBRICKS_DATA_DIR", None)
            self.assertEqual(str(fetch.data_dir()), os.path.join("/y", "openbricks"))

    def test_the_tables_are_cached_under_the_cache_dir_whatever_the_ldraw_dir_is(self):
        with mock.patch.dict(os.environ, {"OPENBRICKS_LDRAW_DIR": "/opt/ldraw", "XDG_CACHE_HOME": "/c"}):
            self.assertEqual(str(bricks.ldraw_dir()), os.path.join("/opt", "ldraw"))
            self.assertEqual(str(bricks.cache_dir()), os.path.join("/c", "openbricks"))
            self.assertEqual(str(fetch.rebrickable_dir()), os.path.join("/c", "openbricks", "rebrickable"))
        with mock.patch.dict(os.environ, {"XDG_CACHE_HOME": "/c"}, clear=False):
            os.environ.pop("OPENBRICKS_LDRAW_DIR", None)
            self.assertEqual(str(bricks.ldraw_dir()), os.path.join("/c", "openbricks", "ldraw"))

    def test_a_sparse_cache_is_present_but_not_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            os.makedirs(os.path.join(tmp, "parts"))
            os.makedirs(os.path.join(tmp, "p"))
            self.assertTrue(bricks.library_present(tmp))
            self.assertFalse(bricks.library_complete(tmp))
            with open(os.path.join(tmp, "LDConfig.ldr"), "w") as fh:
                fh.write("0 Configuration\n")
            self.assertTrue(bricks.library_complete(tmp))
            # a sparse cache does not stop the whole library from being fetched
            said = []
            with mock.patch.object(bricks, "library_complete", return_value=True):
                bricks.fetch_library(dest=tmp, opener=lambda r: (_ for _ in ()).throw(AssertionError("no download")),
                                     progress=said.append)
            self.assertTrue(any("already" in s for s in said))


class UserBricksTests(unittest.TestCase):
    """The parts fetched by number, as the Python runtime loads them."""

    def test_the_users_parts_join_the_library_but_never_shadow_a_shipped_one(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(bricks.load_user_bricks(os.path.join(tmp, "none")), [])
            d = os.path.join(tmp, "bricks")
            os.makedirs(d)
            rec = {"name": "P", "mesh": {"verts": 0, "tris": 0}, "bbox": [[0, 0, 0], [1, 1, 1]]}
            with open(os.path.join(d, "2458.json"), "w") as fh:
                json.dump({"parts": {"2458": dict(rec)}, "colors": {"4": {"name": "Red", "rgb": "C91A09", "trans": False}}}, fh)
            with open(os.path.join(d, "3001.json"), "w") as fh:
                json.dump({"parts": {"3001": dict(rec, fetched=True)}}, fh)
            with open(os.path.join(d, "broken.json"), "w") as fh:
                fh.write("{")
            with open(os.path.join(d, "list.json"), "w") as fh:
                fh.write("[1]")
            with open(os.path.join(d, "notes.txt"), "w") as fh:
                fh.write("not a bundle")
            loaded = bricks.load_user_bricks(d)
            self.assertEqual([os.path.basename(str(p)) for p, _ in loaded], ["2458.json", "3001.json", "broken.json", "list.json"])
            self.assertTrue(loaded[0][1]["parts"]["2458"]["fetched"], "a file in the directory travels with a build")
            self.assertIn("Expecting", loaded[2][1])
            self.assertIn("parts", loaded[3][1])
            bundle, notes = bricks.library_bundle(d)
            self.assertEqual(bundle["parts"]["2458"]["name"], "P")
            self.assertNotEqual(bundle["parts"]["3001"]["name"], "P", "the shipped record wins")
            self.assertEqual(bundle["colors"]["4"]["name"], "Red")
            self.assertEqual(len(notes), 3, notes)
            self.assertTrue(any("ships 3001" in n and "3001.json" in n for n in notes), notes)
            self.assertTrue(any("broken.json" in n for n in notes) and any("list.json" in n for n in notes), notes)
            # the default directory is the data directory's
            with mock.patch.dict(os.environ, {"OPENBRICKS_DATA_DIR": tmp}):
                self.assertEqual(bricks.library_bundle()[0]["parts"]["2458"]["name"], "P")
                shipped = bricks.load_bundle()
                self.assertNotIn("2458", shipped["parts"], "the shipped bundle itself is untouched")


@unittest.skipIf(not HAVE_NUMPY, "numpy (the [sim] extra) is required")
class FetchingLibraryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.site_root = write_mini_library(os.path.join(self.tmp.name, "site"))
        self.cache = os.path.join(self.tmp.name, "cache")

    def tearDown(self):
        self.tmp.cleanup()

    def test_a_part_and_what_it_references_come_down_into_the_cache_layout(self):
        site = _Site(self.site_root)
        said = []
        lib = fetch.FetchingLibrary(self.cache, opener=site, say=said.append, sleep=lambda s: None)
        from openbricks_sim.bricks import ldraw
        rec = ldraw.convert_part(lib, ldraw.Builder(lib), "6666")
        self.assertEqual(rec["name"], "Test Pin from a Primitive")
        self.assertTrue(os.path.exists(os.path.join(self.cache, "parts", "6666.dat")))
        self.assertTrue(os.path.exists(os.path.join(self.cache, "p", "confric5.dat")))
        self.assertFalse(os.path.exists(os.path.join(self.cache, "parts", "6666.dat.part")))
        self.assertEqual(lib.fetched, ["parts/6666.dat", "p/confric5.dat"])
        # a part number is asked for under parts/ first, a primitive name under p/ first, and
        # every request names us and gives up on a stalled connection
        urls = [r.full_url for r in site.requests]
        self.assertEqual(urls, [fetch.OFFICIAL + "parts/6666.dat", fetch.OFFICIAL + "p/confric5.dat"])
        self.assertTrue(all(r.get_header("User-agent") == bricks.USER_AGENT for r in site.requests))
        self.assertEqual(site.timeouts, [fetch.FETCH_TIMEOUT_S] * 2)
        self.assertEqual(said, ["fetched parts/6666.dat", "fetched p/confric5.dat"])
        # the second time round nothing is fetched: the cache has it
        lib2 = fetch.FetchingLibrary(self.cache, opener=site, sleep=lambda s: None)
        ldraw.convert_part(lib2, ldraw.Builder(lib2), "6666")
        self.assertEqual(lib2.fetched, [])
        self.assertEqual(len(site.requests), 2)
        # unless asked to fetch the part's files again: the part, not the primitive
        lib3 = fetch.FetchingLibrary(self.cache, opener=site, sleep=lambda s: None, force=True)
        ldraw.convert_part(lib3, ldraw.Builder(lib3), "6666")
        self.assertEqual(lib3.fetched, ["parts/6666.dat"])
        self.assertEqual(len(site.requests), 3)
        # a file that is not an LDraw file is not kept: a 200 that carries a page is an error
        page = lambda req, timeout=None: _Response(b"<!DOCTYPE html><html>maintenance</html>")   # noqa: E731
        lib4 = fetch.FetchingLibrary(self.cache, opener=page, sleep=lambda s: None)
        with self.assertRaises(fetch.FetchError) as cm:
            lib4.resolve("1234.dat")
        self.assertEqual(cm.exception.reason, "not an LDraw file")
        self.assertFalse(os.path.exists(os.path.join(self.cache, "parts", "1234.dat")))

    def test_moved_to_and_the_tracker_and_a_rate_limit(self):
        site = _Site(self.site_root, unofficial={"parts/9999.dat"}, throttle={"parts/8888.dat"})
        waits = []
        said = []
        lib = fetch.FetchingLibrary(self.cache, opener=site, say=said.append, sleep=waits.append)
        from openbricks_sim.bricks import ldraw
        # 8888 is "~Moved to 9999": the converter follows, and 9999 is only on the tracker
        rec = ldraw.convert_part(lib, ldraw.Builder(lib), "8888")
        self.assertEqual(rec["ldraw"], "9999")
        urls = [r.full_url for r in site.requests]
        self.assertEqual(urls[:2], [fetch.OFFICIAL + "parts/8888.dat", fetch.OFFICIAL + "parts/8888.dat"], "429, then again")
        self.assertIn(fetch.UNOFFICIAL + "parts/9999.dat", urls)
        self.assertEqual(waits, [3])
        self.assertTrue(any(s.startswith("rate limited") and "(1 of %d)" % fetch.RATE_LIMIT_WAITS in s for s in said), said)
        # an alias of an alias is followed to the end: the part, not "Moved to"
        rec = ldraw.convert_part(lib, ldraw.Builder(lib), "8887")
        self.assertEqual((rec["ldraw"], rec["name"]), ("9999", "Test Box 40 x 20 x 10"))
        # over a plain library: an alias whose target is not there has no faces of its own, and one
        # that names itself is taken as it is
        plain = ldraw.Library(self.site_root)
        self.assertIsNone(ldraw.convert_part(plain, ldraw.Builder(plain), "8886"))
        rec = ldraw.convert_part(plain, ldraw.Builder(plain), "8885")
        self.assertEqual((rec["ldraw"], rec["name"]), ("8885", "Moved to 8885"))

    def test_every_rate_limit_is_waited_out_up_to_a_point(self):
        # a 429 on every third request, each waited out: a part of many files comes down
        site = _Site(self.site_root, throttle_every=3)
        waits = []
        lib = fetch.FetchingLibrary(self.cache, opener=site, sleep=waits.append)
        lib.asked = "4444"
        from openbricks_sim.bricks import ldraw
        with self.assertRaises(fetch.MissingReference):          # 4444, axlehol2, axlehol0, stud, then a miss
            ldraw.convert_part(lib, ldraw.Builder(lib), "4444")
        self.assertEqual(lib.fetched, ["parts/4444.dat", "p/axlehol2.dat", "p/axlehol0.dat", "p/stud.dat"])
        self.assertGreaterEqual(len(waits), 2, "more than one 429 was waited out")
        self.assertTrue(all(w == 3 for w in waits), waits)
        # ldraw.org's own figure is honoured, within reason
        self.assertEqual(fetch._retry_after({"Retry-After": "42"}), 42)
        self.assertEqual(fetch._retry_after({"Retry-After": "9999"}), fetch.RATE_LIMIT_WAIT_S)
        self.assertEqual(fetch._retry_after({"Retry-After": "soon"}), fetch.RATE_LIMIT_WAIT_S)
        self.assertEqual(fetch._retry_after({"Retry-After": "0"}), 1)
        self.assertEqual(fetch._retry_after({}), fetch.RATE_LIMIT_WAIT_S)

        # a site that never stops saying 429 is given up on after RATE_LIMIT_WAITS waits
        def always(req, timeout=None):
            raise _http_error(req.full_url, 429)
        waits = []
        lib = fetch.FetchingLibrary(self.cache, opener=always, sleep=waits.append)
        with self.assertRaises(fetch.FetchError) as cm:
            lib.resolve("1234.dat")
        self.assertIn("429", cm.exception.reason)
        self.assertEqual(waits, [fetch.RATE_LIMIT_WAIT_S] * fetch.RATE_LIMIT_WAITS)

    def test_a_part_nobody_has_and_a_broken_reference_are_errors(self):
        site = _Site(self.site_root)
        lib = fetch.FetchingLibrary(self.cache, opener=site, sleep=lambda s: None)
        with self.assertRaises(fetch.NotInLibrary) as cm:
            lib.resolve("0000.dat")
        self.assertEqual(cm.exception.number, "0000")
        self.assertEqual(len(cm.exception.urls), 4, "parts and p, official and the tracker")
        # a part whose reference nobody has is not converted with a hole in it: it is an
        # error that names the part and what it needs, not "no such part"
        from openbricks_sim.bricks import ldraw
        lib.asked = "4444"
        with self.assertRaises(fetch.MissingReference) as cm:
            ldraw.convert_part(lib, ldraw.Builder(lib), "4444")
        self.assertIsInstance(cm.exception, fetch.FetchError)
        self.assertEqual((cm.exception.asked, cm.exception.name), ("4444", "nothing-here.dat"))
        self.assertTrue(str(cm.exception).startswith("4444 needs nothing-here.dat, which ldraw.org has not"), str(cm.exception))
        self.assertIn("nothing-here.dat", cm.exception.url)
        # the part asked for, when it is the one missing, is still "no such part"
        lib.asked = "0000"
        with self.assertRaises(fetch.NotInLibrary):
            lib.resolve("0000.dat")
        # any other trouble names the URL
        def down(req, timeout=None):
            raise urllib.error.URLError("no route to host")
        lib = fetch.FetchingLibrary(self.cache, opener=down, sleep=lambda s: None)
        with self.assertRaises(fetch.FetchError) as cm:
            lib.resolve("1234.dat")
        self.assertIn("parts/1234.dat", cm.exception.url)
        self.assertIn("no route", cm.exception.reason)
        def teapot(req, timeout=None):
            raise _http_error(req.full_url, 418)
        lib = fetch.FetchingLibrary(self.cache, opener=teapot, sleep=lambda s: None)
        with self.assertRaises(fetch.FetchError):
            lib.resolve("1234.dat")

    def test_a_connection_that_dies_mid_body_or_stalls_is_an_error_naming_the_url(self):
        class Cut(_Response):
            def read(self):
                raise http.client.IncompleteRead(b"0 Brick")

        class Stalled(_Response):
            def read(self):
                raise TimeoutError("timed out")

        class Reset(_Response):
            def read(self):
                raise ConnectionResetError(54, "Connection reset by peer")

        for cls, word in ((Cut, "IncompleteRead"), (Stalled, "TimeoutError"), (Reset, "ConnectionResetError")):
            lib = fetch.FetchingLibrary(self.cache, opener=lambda req, timeout=None: cls(b""), sleep=lambda s: None)
            with self.assertRaises(fetch.FetchError) as cm:
                lib.resolve("1234.dat")
            self.assertIn("parts/1234.dat", cm.exception.url)
            self.assertIn(word, cm.exception.reason)


@unittest.skipIf(not HAVE_NUMPY, "numpy (the [sim] extra) is required")
class FetchPartTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.site_root = write_mini_library(os.path.join(self.tmp.name, "site"))
        self.cache = os.path.join(self.tmp.name, "cache")
        self.colors = os.path.join(self.tmp.name, "rebrickable")
        # 9999 is listed as itself; 7777 only as the design id of its moulds; 6666 not at all
        self.tables = {"colors": "id,name,rgb,is_trans\n4,Red,C91A09,f\n15,White,FFFFFF,f\n1,Blue,0055BF,f\n",
                       "elements": "element_id,part_num,color_id,design_id\n300121,9999,4,\n300101,9999,15,\n"
                                   "777701,7777b,1,7777\n777702,7777c,4,7777\n"}
        self.site = _Site(self.site_root)
        self.cdn = []

        def opener(req, timeout=None):
            url = req.full_url
            if url.startswith("https://cdn.rebrickable.com/"):
                self.cdn.append((req, timeout))
                name = url.rsplit("/", 1)[1].split(".")[0]
                return _Response(gzip.compress(self.tables[name].encode()))
            return self.site(req, timeout=timeout)
        self.opener = opener

    def tearDown(self):
        self.tmp.cleanup()

    def test_a_one_part_bundle_with_its_colours(self):
        said = []
        bundle = fetch.fetch_part("9999", root=self.cache, opener=self.opener, say=said.append, colors_cache=self.colors)
        self.assertEqual(bundle["format"], "openbricks-brick-bundle/1")
        self.assertIn("ldraw.org", bundle["source"])
        rec = bundle["parts"]["9999"]
        self.assertEqual(rec["name"], "Test Box 40 x 20 x 10")
        self.assertTrue(rec["fetched"])
        self.assertEqual(rec["colors"], {"4": ["300121"], "15": ["300101"]})
        self.assertEqual(sorted(bundle["colors"]), ["15", "4"])
        self.assertEqual(bundle["files"], 1)
        self.assertNotIn("note", bundle)
        self.assertTrue(said[-1].startswith("9999: Test Box"), said)
        # the tables were cached with our agent and a timeout, and are not fetched again
        self.assertEqual(len(self.cdn), 2)
        self.assertTrue(all(r.get_header("User-agent") == bricks.USER_AGENT and t == fetch.FETCH_TIMEOUT_S for r, t in self.cdn))
        fetch.fetch_part("7777", root=self.cache, opener=self.opener, colors_cache=self.colors)
        self.assertEqual(len(self.cdn), 2)
        # unless asked for afresh
        fetch.fetch_part("7777", root=self.cache, opener=self.opener, colors_cache=self.colors, force=True)
        self.assertEqual(len(self.cdn), 4)

    def test_colours_by_the_resolved_number_or_the_design_id_and_a_part_without_any(self):
        # 8888 is "~Moved to 9999", which Rebrickable lists: its colours come through the resolved number
        bundle = fetch.fetch_part("8888", root=self.cache, opener=self.opener, colors_cache=self.colors)
        rec = bundle["parts"]["8888"]
        self.assertEqual(rec["ldraw"], "9999")
        self.assertEqual(rec["colors"], {"4": ["300121"], "15": ["300101"]})
        self.assertEqual(sorted(bundle["colors"]), ["15", "4"], "the palette of those colours alone")
        # 7777's elements carry it as their design id, under mould numbers Rebrickable spells its own way
        bundle = fetch.fetch_part("7777", root=self.cache, opener=self.opener, colors_cache=self.colors)
        self.assertEqual(bundle["parts"]["7777"]["colors"], {"1": ["777701"], "4": ["777702"]})
        # a part Rebrickable has nothing on says so, rather than coming down colourless in silence
        said = []
        bundle = fetch.fetch_part("6666", root=self.cache, opener=self.opener, say=said.append, colors_cache=self.colors)
        self.assertEqual(bundle["parts"]["6666"]["colors"], {})
        self.assertEqual(bundle["note"], "Rebrickable lists no colours for 6666")
        self.assertIn(bundle["note"], said)
        bundle = fetch.fetch_part("8887", root=self.cache, opener=lambda req, timeout=None: self.opener(req, timeout),
                                  colors_cache=self.colors)
        self.assertEqual(bundle["parts"]["8887"]["colors"], {"4": ["300121"], "15": ["300101"]}, "two hops, then Rebrickable")
        self.tables["elements"] = "element_id,part_num,color_id,design_id\n"
        bundle = fetch.fetch_part("8887", root=self.cache, opener=self.opener, colors_cache=os.path.join(self.tmp.name, "c2"))
        self.assertEqual(bundle["note"], "Rebrickable lists no colours for 8887 or 9999")

    def test_a_file_with_no_faces_is_no_part(self):
        with self.assertRaises(fetch.FetchError) as cm:
            fetch.fetch_part("1111", root=self.cache, opener=self.opener, colors=False)
        self.assertEqual(cm.exception.reason, "the file has no faces")
        self.assertIn("parts/1111.dat", cm.exception.url)

    def test_without_colours_and_a_colour_table_that_cannot_be_had(self):
        bundle = fetch.fetch_part("9999", root=self.cache, opener=self.opener, colors=False, colors_cache=self.colors)
        self.assertNotIn("colors", bundle["parts"]["9999"])
        self.assertNotIn("colors", bundle)

        def no_cdn(req, timeout=None):
            if "rebrickable" in req.full_url:
                raise urllib.error.URLError("blocked")
            return self.site(req)
        with self.assertRaises(fetch.FetchError) as cm:
            fetch.fetch_part("9999", root=self.cache, opener=no_cdn, colors_cache=os.path.join(self.tmp.name, "empty"))
        self.assertIn("rebrickable", cm.exception.url)

        class Cut(_Response):
            def read(self):
                raise http.client.IncompleteRead(b"")

        def cut_cdn(req, timeout=None):
            if "rebrickable" in req.full_url:
                return Cut(b"")
            return self.site(req)
        with self.assertRaises(fetch.FetchError) as cm:
            fetch.fetch_part("9999", root=self.cache, opener=cut_cdn, colors_cache=os.path.join(self.tmp.name, "empty2"))
        self.assertIn("IncompleteRead", cm.exception.reason)

    def test_main_speaks_json_lines_and_exit_codes(self):
        out_file = os.path.join(self.tmp.name, "bricks", "9999.json")
        calls = []

        def fake(number, root=None, say=None, colors=True, force=False):
            calls.append((number, root, colors, force))
            say("fetched parts/%s.dat" % number)
            bundle = {"format": "openbricks-brick-bundle/1", "files": 1,
                      "parts": {number: {"name": "Box", "colors": {"4": ["1"]}}}}
            if number == "6666":
                bundle["parts"][number]["colors"] = {}
                bundle["note"] = "Rebrickable lists no colours for 6666"
            return bundle
        with mock.patch.object(fetch, "fetch_part", fake):
            out = io.StringIO()
            with redirect_stdout(out):
                rc = fetch.main(["9999", "--out", out_file])
        self.assertEqual(rc, 0)
        lines = [json.loads(l) for l in out.getvalue().splitlines()]
        self.assertEqual(lines[0], {"ev": "log", "text": "fetched parts/9999.dat"})
        self.assertEqual(lines[-1], {"ev": "fetched", "number": "9999", "name": "Box", "files": 1, "colors": 1, "out": out_file})
        with open(out_file) as fh:
            self.assertEqual(json.load(fh)["parts"]["9999"]["name"], "Box")
        self.assertFalse(os.path.exists(out_file + ".part"))
        self.assertEqual(calls, [("9999", None, True, False)])
        # --force and --no-colors reach the fetcher; a part without colours says why in the event
        with mock.patch.object(fetch, "fetch_part", fake):
            out = io.StringIO()
            with redirect_stdout(out):
                rc = fetch.main(["6666", "--out", out_file, "--force", "--no-colors", "--ldraw", self.cache])
        self.assertEqual(rc, 0)
        last = json.loads(out.getvalue().splitlines()[-1])
        self.assertEqual((last["colors"], last["note"]), (0, "Rebrickable lists no colours for 6666"))
        self.assertEqual(calls[-1], ("6666", self.cache, False, True))
        # an --out that cannot be written is an error event, not a traceback
        blocked = os.path.join(self.tmp.name, "a-file")
        with open(blocked, "w") as fh:
            fh.write("x")
        with mock.patch.object(fetch, "fetch_part", fake):
            out = io.StringIO()
            with redirect_stdout(out):
                rc = fetch.main(["9999", "--out", os.path.join(blocked, "9999.json")])
        self.assertEqual(rc, 1)
        last = json.loads(out.getvalue().splitlines()[-1])
        self.assertEqual(last["ev"], "error")
        self.assertIn("a-file", last["text"])

        def missing(number, root=None, say=None, colors=True, force=False):
            raise fetch.NotInLibrary(number, ["u1"])
        with mock.patch.object(fetch, "fetch_part", missing):
            out = io.StringIO()
            with redirect_stdout(out):
                rc = fetch.main(["0000", "--out", out_file, "--no-colors", "--ldraw", self.cache])
        self.assertEqual(rc, 2)
        self.assertEqual(json.loads(out.getvalue())["ev"], "error")
        self.assertIn("no part 0000", json.loads(out.getvalue())["text"])

        def broken(number, root=None, say=None, colors=True, force=False):
            raise fetch.FetchError("u2", "HTTP 500")
        with mock.patch.object(fetch, "fetch_part", broken):
            out = io.StringIO()
            with redirect_stdout(out):
                self.assertEqual(fetch.main(["9999", "--out", out_file]), 1)
        self.assertIn("HTTP 500", json.loads(out.getvalue())["text"])

        def incomplete(number, root=None, say=None, colors=True, force=False):
            raise fetch.MissingReference(number, "s\\1234s01.dat", ["u3", "u4"])
        with mock.patch.object(fetch, "fetch_part", incomplete):
            out = io.StringIO()
            with redirect_stdout(out):
                self.assertEqual(fetch.main(["1234", "--out", out_file]), 1, "incomplete, not absent")
        self.assertTrue(json.loads(out.getvalue())["text"].startswith("1234 needs s\\1234s01.dat"))
        err = io.StringIO()
        with redirect_stderr(err):
            self.assertEqual(fetch.main(["--out", out_file]), 2)
            self.assertEqual(fetch.main(["9999"]), 2)
            self.assertEqual(fetch.main(["9999", "--bogus"]), 2)
        self.assertIn("usage", err.getvalue())

    def test_the_real_converter_through_main(self):
        out_file = os.path.join(self.tmp.name, "bricks", "7777.json")
        with mock.patch.object(fetch.urllib.request, "urlopen", self.opener), \
                mock.patch.object(fetch, "rebrickable_dir", lambda: __import__("pathlib").Path(self.colors)):
            out = io.StringIO()
            with redirect_stdout(out):
                rc = fetch.main(["7777", "--out", out_file, "--ldraw", self.cache])
        self.assertEqual(rc, 0, out.getvalue())
        last = json.loads(out.getvalue().splitlines()[-1])
        self.assertEqual((last["ev"], last["number"], last["files"]), ("fetched", "7777", 1))
        with open(out_file) as fh:
            rec = json.load(fh)["parts"]["7777"]
        self.assertEqual(rec["name"], "Test Ring with Pin Hole")
        self.assertTrue(any(c["kind"] == "pin_hole" for c in rec["connectors"]))


if __name__ == "__main__":
    unittest.main()
