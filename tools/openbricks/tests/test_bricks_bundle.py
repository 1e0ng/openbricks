# SPDX-License-Identifier: MIT
"""The shipped Technic bundle and the library helpers: the data the
wheel carries must be complete, decodable and physically plausible."""
import io
import os
import tempfile
import unittest
import zipfile
from unittest import mock

from openbricks_sim import bricks

try:
    from openbricks_sim.bricks import ldraw
except ImportError:                      # pragma: no cover
    ldraw = None


class ShippedBundleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundle = bricks.load_bundle()

    def test_bundle_shape(self):
        self.assertEqual(self.bundle["format"], "openbricks-brick-bundle/1")
        self.assertIn("ldraw.org", self.bundle["source"])
        self.assertGreaterEqual(len(self.bundle["parts"]), 120)
        self.assertEqual(self.bundle.get("missing", []), [])

    def test_the_bundle_holds_the_curated_list_and_the_sets(self):
        listed = set()
        for line in bricks.data_path("technic_parts.txt").read_text().splitlines():
            line = line.split("#", 1)[0].strip()
            if line:
                listed.add(line)
        for s in bricks.load_sets().values():
            listed.update(s["parts"])
        self.assertEqual(sorted(listed), sorted(self.bundle["parts"]))

    def test_the_wro_sets_are_complete_and_counted(self):
        # 45811 is the WRO Brick Set (2016, the mission bricks); 45819 the
        # WRO Expansion Set (2023). Every part of both is in the bundle,
        # stamped with how many the set holds, and the inventory numbers
        # LDraw spells differently are kept as aliases.
        sets = bricks.load_sets()
        self.assertEqual(sorted(sets), ["45811", "45819"])
        self.assertEqual({sid: s["pieces"] for sid, s in sets.items()}, {"45811": 724, "45819": 568})
        for sid, s in sets.items():
            with self.subTest(set=sid):
                self.assertEqual(self.bundle["sets"][sid], {"name": s["name"], "year": s["year"], "pieces": s["pieces"]})
                self.assertIn("World Robot Olympiad", s["name"])
                self.assertEqual(sum(s["parts"].values()), s["pieces"])
                for num, qty in s["parts"].items():
                    self.assertEqual(self.bundle["parts"][num]["sets"][sid], qty, num)
                for other, num in s["aliases"].items():
                    self.assertIn(other, self.bundle["parts"][num]["aliases"], num)
        self.assertEqual(sets["45811"]["aliases"], {"41250": "22119", "78c18": "72039"})
        self.assertEqual(sets["45819"]["aliases"], {"32005a": "32005"})
        self.assertEqual(self.bundle["parts"]["72039"]["name"], "Technic Ribbed Hose 18L")
        self.assertEqual(self.bundle["parts"]["3001"]["sets"], {"45811": 288})

    def test_every_part_is_a_closed_mesh_with_mass_properties(self):
        for num, part in self.bundle["parts"].items():
            with self.subTest(part=num):
                self.assertEqual(part["mass_model"], "mesh")
                self.assertGreater(part["volume_mm3"], 0)
                self.assertGreater(part["mass_g"], 0)
                lo, hi = part["bbox"]
                self.assertTrue(all(h > l for l, h in zip(lo, hi)))
                self.assertEqual(len(part["inertia_per_g"]), 3)
                self.assertIn(part["source"], ("vendor", "placeholder"))
                self.assertTrue(part["mesh"]["pos"] and part["mesh"]["idx"] and part["mesh"]["nrm"])

    @unittest.skipIf(ldraw is None, "numpy (the [sim] extra) is required")
    def test_every_mesh_decodes_to_its_declared_size(self):
        for num, part in self.bundle["parts"].items():
            pos, nrm, idx = ldraw.unpack_mesh(part["mesh"])
            self.assertEqual(idx.shape[0], part["mesh"]["tris"], num)
            self.assertEqual(pos.shape[0], part["mesh"]["verts"], num)
            self.assertLess(idx.max(), pos.shape[0], num)

    def test_known_technic_features(self):
        p = self.bundle["parts"]
        beam = [c for c in p["32278"]["connectors"] if c["kind"] == "pin_hole"]
        self.assertEqual(len(beam), 15)
        self.assertTrue(all(c["length"] == 8.0 for c in beam))
        self.assertEqual(sorted(round(c["centre"][1]) for c in beam), list(range(-56, 57, 8)))
        frame = [c for c in p["64178"]["connectors"] if c["kind"] == "pin_hole"]
        self.assertEqual(len(frame), 28)
        pins = [c for c in p["2780"]["connectors"] if c["kind"] == "pin"]
        self.assertEqual([c["length"] for c in pins], [8.0, 8.0])
        axle = [c for c in p["3705"]["connectors"] if c["kind"] == "axle"]
        self.assertEqual(len(axle), 1)
        self.assertAlmostEqual(axle[0]["length"], 30.0, places=1)
        self.assertEqual([c["kind"] for c in p["3713"]["connectors"]], ["axle_hole"])
        joiner = [c for c in p["62462"]["connectors"] if c["kind"] == "pin_hole"]
        self.assertEqual(len(joiner), 2)

    def test_vendor_weights_are_physically_plausible_for_beams(self):
        for num in ("32278", "32525", "32524", "32316", "64178", "64179", "43857", "32523"):
            part = self.bundle["parts"][num]
            self.assertEqual(part["source"], "vendor", num)
            self.assertGreater(part["density_g_cm3"], 0.9, num)
            self.assertLess(part["density_g_cm3"], 1.3, num)

    def test_beam_15_is_the_right_size(self):
        lo, hi = self.bundle["parts"]["32278"]["bbox"]
        self.assertEqual([round(h - l, 1) for l, h in zip(lo, hi)], [7.2, 119.2, 8.0])

    def test_bundle_fits_the_page_budget(self):
        self.assertLess(len(bricks.bundle_b64()), 4_000_000)


class ColorsTests(unittest.TestCase):
    """The colours a part comes in and the LEGO element numbers that name
    each part-and-colour, from Rebrickable's tables (4.21.0)."""

    @classmethod
    def setUpClass(cls):
        cls.bundle = bricks.load_bundle()
        cls.colors = bricks.load_colors()

    def test_the_bundle_carries_a_palette_and_every_part_its_colours(self):
        palette = self.bundle["colors"]
        self.assertEqual(palette, self.colors["palette"])
        self.assertEqual(palette["72"], {"name": "Dark Bluish Gray", "rgb": "6C6E68", "trans": False})
        without = [n for n, p in self.bundle["parts"].items() if not p.get("colors")]
        self.assertEqual(without, self.colors["without"])
        self.assertLessEqual(len(without), 1, without)
        for num, part in self.bundle["parts"].items():
            for cid, elements in part.get("colors", {}).items():
                self.assertIn(cid, palette, num)
                self.assertTrue(elements and all(e.isdigit() for e in elements), (num, cid))
        # a beam 15 in dark bluish gray is element 4210687; in red 4163147
        self.assertIn("4210687", self.bundle["parts"]["32278"]["colors"]["72"])
        self.assertIn("4163147", self.bundle["parts"]["32278"]["colors"]["4"])

    def test_an_element_number_names_one_colour(self):
        # Rebrickable lists a few element numbers under two part numbers
        # (a mould renumbered under one element), never under two colours:
        # searching by element must land on one colour, whichever part.
        seen = {}
        for num, part in self.bundle["parts"].items():
            for cid, elements in part.get("colors", {}).items():
                for e in elements:
                    self.assertEqual(seen.setdefault(e, cid), cid, "element %s in two colours (%s)" % (e, num))
        self.assertGreater(len(seen), 4000)

    def test_build_from_rows_and_apply_to_a_bundle(self):
        from openbricks_sim.bricks import rebrickable
        colors = [{"id": "72", "name": "Dark Bluish Gray", "rgb": "6C6E68", "is_trans": "f"},
                  {"id": "4", "name": "Red", "rgb": "C91A09", "is_trans": "f"},
                  {"id": "41", "name": "Trans-Light Blue", "rgb": "AEEFEC", "is_trans": "t"}]
        elements = [{"element_id": "4210687", "part_num": "32278", "color_id": "72", "design_id": ""},
                    {"element_id": "32278199", "part_num": "32278", "color_id": "72", "design_id": ""},
                    {"element_id": "4163147", "part_num": "32278", "color_id": "4", "design_id": ""},
                    {"element_id": "1", "part_num": "3648b", "color_id": "41", "design_id": ""},
                    {"element_id": "2", "part_num": "9999", "color_id": "4", "design_id": ""},
                    {"element_id": "3", "part_num": "3070b", "color_id": "72", "design_id": "3070"},
                    {"element_id": "4", "part_num": "3069b", "color_id": "4", "design_id": "3069"}]
        data = rebrickable.build(["32278", "3648", "6590", "3070", "3069b"], colors, elements,
                                 {"3648": "3648b", "77": "x", "3069b": "3069z"})
        self.assertEqual(data["parts"]["32278"], {"4": ["4163147"], "72": ["4210687", "32278199"]})
        self.assertEqual(data["parts"]["3648"], {"41": ["1"]}, "Rebrickable's mould suffix is followed")
        self.assertEqual(data["parts"]["3070"], {"72": ["3"]}, "a number known only as a design id")
        self.assertEqual(data["parts"]["3069b"], {"4": ["4"]}, "the number itself, when its alias misses")
        self.assertEqual(data["without"], ["6590"])
        self.assertEqual(sorted(data["palette"]), ["4", "41", "72"], "only the colours used")
        self.assertTrue(data["palette"]["41"]["trans"] and not data["palette"]["4"]["trans"])
        self.assertEqual(data["rebrickable"], {"3069b": "3069z", "3648": "3648b"}, "only the numbers asked for")
        self.assertEqual(rebrickable.rebrickable_numbers({"s": {"aliases": {"78c18": "72039"}}})["72039"], "78c18")
        if ldraw is not None:
            bundle = {"parts": {"32278": {"name": "Beam 15"}, "6590": {"name": "Bush"}}, "missing": []}
            ldraw.apply_colors(bundle, data)
            self.assertEqual(bundle["colors"], data["palette"])
            self.assertEqual(bundle["parts"]["32278"]["colors"]["72"], ["4210687", "32278199"])
            self.assertNotIn("colors", bundle["parts"]["6590"])


    def test_the_tables_are_fetched_with_our_agent_and_main_writes_the_file(self):
        import gzip
        import json
        import tempfile
        from openbricks_sim.bricks import rebrickable
        tables = {"colors": "id,name,rgb,is_trans\n72,Dark Bluish Gray,6C6E68,f\n",
                  "elements": "element_id,part_num,color_id,design_id\n4210687,32278,72,\n"}
        seen = []

        def opener(req):
            seen.append(req)
            name = req.full_url.rsplit("/", 1)[1].split(".")[0]
            return _FakeResponse(gzip.compress(tables[name].encode()))

        rows = rebrickable.fetch_table("colors", opener=opener)
        self.assertEqual(rows, [{"id": "72", "name": "Dark Bluish Gray", "rgb": "6C6E68", "is_trans": "f"}])
        self.assertEqual(seen[0].full_url, rebrickable.DOWNLOADS + "colors.csv.gz")
        self.assertEqual(seen[0].get_header("User-agent"), bricks.USER_AGENT)
        elements = rebrickable.fetch_table("elements", opener=opener)
        with tempfile.TemporaryDirectory() as tmp, \
                mock.patch.object(rebrickable, "fetch_table", lambda name, opener=None: rows if name == "colors" else elements):
            out = os.path.join(tmp, "colors.json")
            self.assertEqual(rebrickable.main([out]), 0)
            with open(out) as fh:
                data = json.load(fh)
            self.assertEqual(data["parts"]["32278"], {"72": ["4210687"]})
            self.assertEqual(data["palette"], {"72": {"name": "Dark Bluish Gray", "rgb": "6C6E68", "trans": False}})
            self.assertIn("6590", data["without"])
            self.assertEqual(data["rebrickable"]["3648"], "3648b")
            if ldraw is not None:
                self.assertEqual(ldraw.read_colors(out)["palette"]["72"]["name"], "Dark Bluish Gray")
        self.assertEqual(rebrickable.main([]), 2, "usage")


class SetsTests(unittest.TestCase):
    @unittest.skipIf(ldraw is None, "numpy (the [sim] extra) is required")
    def test_apply_sets_stamps_records_and_lists_what_is_missing(self):
        bundle = {"parts": {"3001": {"name": "Brick 2 x 4"}, "22119": {"name": "Ball 52mm Diameter"}}, "missing": []}
        sets = {"45811": {"name": "WRO Brick Set", "year": 2016, "pieces": 6,
                          "parts": {"3001": 4, "22119": 1, "9999": 1}, "aliases": {"41250": "22119"}}}
        out = ldraw.apply_sets(bundle, sets)
        self.assertIs(out, bundle)
        self.assertEqual(bundle["sets"], {"45811": {"name": "WRO Brick Set", "year": 2016, "pieces": 6}})
        self.assertEqual(bundle["parts"]["3001"]["sets"], {"45811": 4})
        self.assertEqual(bundle["parts"]["22119"]["aliases"], ["41250"])
        self.assertNotIn("aliases", bundle["parts"]["3001"])
        self.assertEqual(bundle["missing"], ["9999"], "a set part the bundle lacks is never quiet")
        self.assertEqual(ldraw.set_numbers(sets), ["22119", "3001", "9999"])
        # the file reader keeps the sets and drops the source note
        import json
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "sets.json")
            with open(path, "w") as fh:
                json.dump({"source": "a note", **sets}, fh)
            self.assertEqual(ldraw.read_sets(path), sets)


class BundleHelperTests(unittest.TestCase):
    def test_merge_and_encode_round_trip(self):
        base = {"format": "openbricks-brick-bundle/1", "parts": {"a": {"name": "A"}}}
        merged = bricks.merge_bundles(base, [{"parts": {"b": {"name": "B"}}}, {"parts": {"a": {"name": "A2"}}}])
        self.assertEqual(merged["parts"]["b"]["name"], "B")
        self.assertEqual(merged["parts"]["a"]["name"], "A2")
        self.assertEqual(base["parts"]["a"]["name"], "A")          # the input is untouched
        import base64
        import json
        import zlib
        back = json.loads(zlib.decompress(base64.b64decode(bricks.encode_bundle(merged))).decode())
        self.assertEqual(back, merged)

    def test_merge_rejects_a_non_bundle(self):
        with self.assertRaises(ValueError):
            bricks.merge_bundles({"parts": {}}, [{"nope": 1}])
        with self.assertRaises(ValueError):
            bricks.merge_bundles({"parts": {}}, ["text"])

    def test_ldraw_dir_honours_the_environment(self):
        saved = {k: os.environ.get(k) for k in ("OPENBRICKS_LDRAW_DIR", "XDG_CACHE_HOME")}
        try:
            os.environ["OPENBRICKS_LDRAW_DIR"] = "/tmp/somewhere/ldraw"
            self.assertEqual(str(bricks.ldraw_dir()), os.path.join("/tmp", "somewhere", "ldraw"))
            del os.environ["OPENBRICKS_LDRAW_DIR"]
            os.environ["XDG_CACHE_HOME"] = "/tmp/xdg"
            self.assertEqual(str(bricks.ldraw_dir()), os.path.join("/tmp", "xdg", "openbricks", "ldraw"))
            del os.environ["XDG_CACHE_HOME"]
            self.assertTrue(str(bricks.ldraw_dir()).endswith(os.path.join(".cache", "openbricks", "ldraw")))
        finally:
            for k, v in saved.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v


def _zip_bytes(members):
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        for name, text in members.items():
            zf.writestr(name, text)
    return buf.getvalue()


class _FakeResponse(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class FetchLibraryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.calls = []

    def tearDown(self):
        self.tmp.cleanup()

    def test_the_download_names_itself(self):
        # library.ldraw.org answers Python's default agent with 403: the
        # request carries our own User-Agent, and goes to the library URL.
        opener = self.opener_for({"ldraw/parts/1.dat": "0 One\n", "ldraw/p/x.dat": "0 X\n"})
        bricks.fetch_library(dest=os.path.join(self.tmp.name, "lib"), opener=opener)
        req = self.calls[0]
        self.assertEqual(req.full_url, bricks.LDRAW_URL)
        self.assertEqual(req.get_header("User-agent"), bricks.USER_AGENT)
        self.assertTrue(bricks.USER_AGENT.startswith("openbricks"))

    def opener_for(self, members):
        payload = _zip_bytes(members)

        def opener(url):
            self.calls.append(url)
            return _FakeResponse(payload)
        return opener

    def test_downloads_unpacks_and_skips_when_present(self):
        opener = self.opener_for({"ldraw/parts/9999.dat": "0 Test\n", "ldraw/parts/s/9999s01.dat": "0 Sub\n",
                                  "ldraw/p/4-4cyli.dat": "0 Cyl\n", "ldraw/CAreadme.txt": "licence\n", "ldraw/models/car.ldr": "0 model\n",
                                  "ldraw/LDConfig.ldr": "0 Configuration\n"})
        said = []
        dest = os.path.join(self.tmp.name, "lib")
        # a cache grown part by part is there but not complete: it does not stop the download
        os.makedirs(os.path.join(dest, "parts"))
        os.makedirs(os.path.join(dest, "p"))
        self.assertTrue(bricks.library_present(dest) and not bricks.library_complete(dest))
        root = bricks.fetch_library(dest=dest, opener=opener, progress=said.append)
        self.assertEqual(str(root), dest)
        self.assertTrue(bricks.library_present(root) and bricks.library_complete(root))
        self.assertTrue(os.path.exists(os.path.join(dest, "parts", "s", "9999s01.dat")))
        self.assertTrue(os.path.exists(os.path.join(dest, "CAreadme.txt")))
        self.assertFalse(os.path.exists(os.path.join(dest, "complete.zip.part")))
        self.assertEqual([c.full_url for c in self.calls], [bricks.LDRAW_URL])
        self.assertTrue(any("1 part files" in s for s in said), said)
        bricks.fetch_library(dest=dest, opener=opener, progress=said.append)
        self.assertEqual(len(self.calls), 1)                      # already there: no second download
        self.assertTrue(any("already" in s for s in said))
        bricks.fetch_library(dest=dest, opener=opener, force=True, progress=said.append)
        self.assertEqual(len(self.calls), 2)

    def test_an_archive_without_the_library_is_refused(self):
        opener = self.opener_for({"readme.txt": "nothing here\n"})
        dest = os.path.join(self.tmp.name, "lib")
        with self.assertRaises(RuntimeError):
            bricks.fetch_library(dest=dest, opener=opener)
        self.assertFalse(bricks.library_present(dest))


class FetchProgressTests(unittest.TestCase):
    def test_progress_lines_every_interval(self):
        payload = _zip_bytes({"ldraw/parts/9999.dat": "0 Test\n", "ldraw/p/x.dat": "0 P\n", "ldraw/pad.bin": "x" * 3000})
        said = []
        with tempfile.TemporaryDirectory() as tmp:
            bricks.fetch_library(dest=os.path.join(tmp, "lib"), opener=lambda url: _FakeResponse(payload), progress=said.append, progress_every=1024)
        self.assertTrue(any(s.strip().endswith("MB") for s in said), said)
