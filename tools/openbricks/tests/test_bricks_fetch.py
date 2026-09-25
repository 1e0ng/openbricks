# SPDX-License-Identifier: MIT
"""One part fetched from ldraw.org file by file (``openbricks_sim.bricks.fetch``)."""
import gzip
import io
import json
import os
import tempfile
import unittest
from unittest import mock
import urllib.error
from contextlib import redirect_stderr, redirect_stdout

from openbricks_sim import bricks
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
    tree, one under the tracker, and a 429 the first time a given file
    is asked for when ``throttle`` is set."""

    def __init__(self, root, unofficial=(), throttle=()):
        self.root = root
        self.unofficial = set(unofficial)
        self.throttle = set(throttle)
        self.requests = []
        self.throttled = set()

    def __call__(self, req):
        url = req.full_url
        self.requests.append(req)
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

    def test_the_data_dir_reads_the_same_three_as_the_sim(self):
        with mock.patch.dict(os.environ, {"OPENBRICKS_DATA_DIR": "/x/data", "XDG_DATA_HOME": "/y"}):
            self.assertEqual(str(fetch.data_dir()), "/x/data")
            self.assertEqual(str(fetch.bricks_dir()), os.path.join("/x/data", "bricks"))
        with mock.patch.dict(os.environ, {"XDG_DATA_HOME": "/y"}, clear=False):
            os.environ.pop("OPENBRICKS_DATA_DIR", None)
            self.assertEqual(str(fetch.data_dir()), os.path.join("/y", "openbricks"))
        with mock.patch.dict(os.environ, {"HOME": "/home/h"}, clear=False):
            os.environ.pop("OPENBRICKS_DATA_DIR", None)
            os.environ.pop("XDG_DATA_HOME", None)
            self.assertEqual(str(fetch.data_dir()), os.path.join("/home/h", ".local", "share", "openbricks"))

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
        # every request names us
        urls = [r.full_url for r in site.requests]
        self.assertEqual(urls, [fetch.OFFICIAL + "parts/6666.dat", fetch.OFFICIAL + "p/confric5.dat"])
        self.assertTrue(all(r.get_header("User-agent") == bricks.USER_AGENT for r in site.requests))
        self.assertEqual(said, ["fetched parts/6666.dat", "fetched p/confric5.dat"])
        # the second time round nothing is fetched: the cache has it
        lib2 = fetch.FetchingLibrary(self.cache, opener=site, sleep=lambda s: None)
        ldraw.convert_part(lib2, ldraw.Builder(lib2), "6666")
        self.assertEqual(lib2.fetched, [])
        self.assertEqual(len(site.requests), 2)

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
        self.assertTrue(any(s.startswith("rate limited") for s in said), said)

    def test_a_part_nobody_has_and_a_broken_reference_are_errors(self):
        site = _Site(self.site_root)
        lib = fetch.FetchingLibrary(self.cache, opener=site, sleep=lambda s: None)
        with self.assertRaises(fetch.NotInLibrary) as cm:
            lib.resolve("0000.dat")
        self.assertEqual(cm.exception.number, "0000")
        self.assertEqual(len(cm.exception.urls), 4, "parts and p, official and the tracker")
        # a part whose reference nobody has is not converted with a hole in it: it is an error
        from openbricks_sim.bricks import ldraw
        with self.assertRaises(fetch.NotInLibrary):
            ldraw.convert_part(lib, ldraw.Builder(lib), "4444")
        # any other trouble names the URL
        def down(req):
            raise urllib.error.URLError("no route to host")
        lib = fetch.FetchingLibrary(self.cache, opener=down, sleep=lambda s: None)
        with self.assertRaises(fetch.FetchError) as cm:
            lib.resolve("1234.dat")
        self.assertIn("parts/1234.dat", cm.exception.url)
        self.assertIn("no route", cm.exception.reason)
        def teapot(req):
            raise _http_error(req.full_url, 418)
        lib = fetch.FetchingLibrary(self.cache, opener=teapot, sleep=lambda s: None)
        with self.assertRaises(fetch.FetchError):
            lib.resolve("1234.dat")
        # a second 429 is not waited out again
        always = _Site(self.site_root, throttle={"parts/9999.dat"})
        always.throttled = set()
        def twice(req):
            raise _http_error(req.full_url, 429)
        lib = fetch.FetchingLibrary(self.cache, opener=twice, sleep=lambda s: None)
        with self.assertRaises(fetch.FetchError) as cm:
            lib.resolve("9999.dat")
        self.assertIn("429", cm.exception.reason)


@unittest.skipIf(not HAVE_NUMPY, "numpy (the [sim] extra) is required")
class FetchPartTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.site_root = write_mini_library(os.path.join(self.tmp.name, "site"))
        self.cache = os.path.join(self.tmp.name, "cache")
        self.colors = os.path.join(self.tmp.name, "rebrickable")
        tables = {"colors": "id,name,rgb,is_trans\n4,Red,C91A09,f\n15,White,FFFFFF,f\n",
                  "elements": "element_id,part_num,color_id,design_id\n300121,9999,4,\n300101,9999,15,\n"}
        self.site = _Site(self.site_root)

        def opener(req):
            url = req.full_url
            if url.startswith("https://cdn.rebrickable.com/"):
                self.site.requests.append(req)
                name = url.rsplit("/", 1)[1].split(".")[0]
                return _Response(gzip.compress(tables[name].encode()))
            return self.site(req)
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
        self.assertTrue(said[-1].startswith("9999: Test Box"), said)
        # the tables were cached with our agent, and are not fetched again
        cdn = [r for r in self.site.requests if r.full_url.startswith("https://cdn.rebrickable.com/")]
        self.assertEqual(len(cdn), 2)
        self.assertTrue(all(r.get_header("User-agent") == bricks.USER_AGENT for r in cdn))
        fetch.fetch_part("7777", root=self.cache, opener=self.opener, colors_cache=self.colors)
        self.assertEqual(len([r for r in self.site.requests if "rebrickable" in r.full_url]), 2)

    def test_without_colours_and_a_colour_table_that_cannot_be_had(self):
        bundle = fetch.fetch_part("9999", root=self.cache, opener=self.opener, colors=False, colors_cache=self.colors)
        self.assertNotIn("colors", bundle["parts"]["9999"])
        self.assertNotIn("colors", bundle)

        def no_cdn(req):
            if "rebrickable" in req.full_url:
                raise urllib.error.URLError("blocked")
            return self.site(req)
        with self.assertRaises(fetch.FetchError) as cm:
            fetch.fetch_part("9999", root=self.cache, opener=no_cdn, colors_cache=os.path.join(self.tmp.name, "empty"))
        self.assertIn("rebrickable", cm.exception.url)

    def test_main_speaks_json_lines_and_exit_codes(self):
        out_file = os.path.join(self.tmp.name, "bricks", "9999.json")
        with mock.patch.object(fetch, "fetch_part", lambda number, root=None, say=None, colors=True: (
                say("fetched parts/%s.dat" % number) or {"format": "openbricks-brick-bundle/1", "files": 1,
                                                          "parts": {number: {"name": "Box", "colors": {"4": ["1"]}}}})):
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

        def missing(number, root=None, say=None, colors=True):
            raise fetch.NotInLibrary(number, ["u1"])
        with mock.patch.object(fetch, "fetch_part", missing):
            out = io.StringIO()
            with redirect_stdout(out):
                rc = fetch.main(["0000", "--out", out_file, "--no-colors", "--ldraw", self.cache])
        self.assertEqual(rc, 2)
        self.assertEqual(json.loads(out.getvalue())["ev"], "error")
        self.assertIn("no part 0000", json.loads(out.getvalue())["text"])

        def broken(number, root=None, say=None, colors=True):
            raise fetch.FetchError("u2", "HTTP 500")
        with mock.patch.object(fetch, "fetch_part", broken):
            out = io.StringIO()
            with redirect_stdout(out):
                self.assertEqual(fetch.main(["9999", "--out", out_file]), 1)
        self.assertIn("HTTP 500", json.loads(out.getvalue())["text"])
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
