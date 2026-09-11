# SPDX-License-Identifier: MIT
"""The shipped Technic bundle and the library helpers: the data the
wheel carries must be complete, decodable and physically plausible."""
import io
import os
import tempfile
import unittest
import zipfile

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

    def test_curated_list_matches_the_bundle(self):
        listed = []
        for line in bricks.data_path("technic_parts.txt").read_text().splitlines():
            line = line.split("#", 1)[0].strip()
            if line:
                listed.append(line)
        self.assertEqual(sorted(listed), sorted(self.bundle["parts"]))

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

    def opener_for(self, members):
        payload = _zip_bytes(members)

        def opener(url):
            self.calls.append(url)
            return _FakeResponse(payload)
        return opener

    def test_downloads_unpacks_and_skips_when_present(self):
        opener = self.opener_for({"ldraw/parts/9999.dat": "0 Test\n", "ldraw/parts/s/9999s01.dat": "0 Sub\n",
                                  "ldraw/p/4-4cyli.dat": "0 Cyl\n", "ldraw/CAreadme.txt": "licence\n", "ldraw/models/car.ldr": "0 model\n"})
        said = []
        dest = os.path.join(self.tmp.name, "lib")
        root = bricks.fetch_library(dest=dest, opener=opener, progress=said.append)
        self.assertEqual(str(root), dest)
        self.assertTrue(bricks.library_present(root))
        self.assertTrue(os.path.exists(os.path.join(dest, "parts", "s", "9999s01.dat")))
        self.assertTrue(os.path.exists(os.path.join(dest, "CAreadme.txt")))
        self.assertFalse(os.path.exists(os.path.join(dest, "complete.zip.part")))
        self.assertEqual(self.calls, [bricks.LDRAW_URL])
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
