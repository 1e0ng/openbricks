# SPDX-License-Identifier: MIT
"""The LDraw → brick converter on a synthetic library: geometry, mass
properties, winding rules, aliases, connector features, packing."""
import json
import os
import tempfile
import unittest
import zlib

try:
    import numpy as np
    from openbricks_sim.bricks import ldraw
except ImportError:                      # pragma: no cover - no [sim] extra
    np = None

from tests.ldraw_fixture import write_mini_library


@unittest.skipIf(np is None, "numpy (the [sim] extra) is required")
class ConverterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.root = write_mini_library(cls.tmp.name)
        cls.lib = ldraw.Library(cls.root)
        cls.builder = ldraw.Builder(cls.lib)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def convert(self, number, weights=None):
        part = ldraw.convert_part(self.lib, self.builder, number, weights)
        self.assertIsNotNone(part, number)
        return part

    def test_box_volume_extents_and_inertia(self):
        part = self.convert("9999")
        self.assertAlmostEqual(part["volume_mm3"], 40 * 20 * 10 * 0.4 ** 3, places=3)   # 512 mm³
        lo, hi = part["bbox"]
        self.assertEqual([round(h - l, 3) for l, h in zip(lo, hi)], [16.0, 4.0, 8.0])     # LDraw Z → Y, -Y → Z
        self.assertEqual([round(c, 3) for c in part["com"]], [0.0, 0.0, 0.0])
        i = part["inertia_per_g"]
        self.assertAlmostEqual(i[0][0], (4 ** 2 + 8 ** 2) / 12, places=3)
        self.assertAlmostEqual(i[1][1], (16 ** 2 + 8 ** 2) / 12, places=3)
        self.assertAlmostEqual(i[2][2], (16 ** 2 + 4 ** 2) / 12, places=3)
        self.assertAlmostEqual(i[0][1], 0.0, places=6)
        self.assertEqual(part["mass_model"], "mesh")
        self.assertEqual(part["mesh"]["tris"], 12)
        self.assertEqual(part["ldraw"], "9999")
        self.assertEqual(part["name"], "Test Box 40 x 20 x 10")

    def test_mirrored_reference_keeps_the_volume_positive(self):
        part = self.convert("9998")
        self.assertAlmostEqual(part["volume_mm3"], 2 * 512, places=3)
        self.assertEqual(part["mesh"]["tris"], 24)

    def test_moved_to_alias_follows_the_reference(self):
        part = self.convert("8888")
        self.assertEqual(part["ldraw"], "9999")
        self.assertAlmostEqual(part["volume_mm3"], 512, places=3)

    def test_bore_becomes_a_pin_hole(self):
        part = self.convert("7777")
        holes = [c for c in part["connectors"] if c["kind"] == "pin_hole"]
        self.assertEqual(len(holes), 1, part["connectors"])
        h = holes[0]
        self.assertEqual(h["length"], 8.0)
        self.assertEqual([round(v, 2) for v in h["centre"]], [0.0, 0.0, 0.0])
        self.assertEqual(abs(round(h["axis"][2])), 1)
        self.assertEqual(h["r"], 2.4)
        self.assertGreater(part["volume_mm3"], 0)

    def test_pin_primitive_becomes_a_pin_segment(self):
        part = self.convert("6666")
        pins = [c for c in part["connectors"] if c["kind"] == "pin"]
        self.assertEqual(len(pins), 1, part["connectors"])
        self.assertEqual(pins[0]["length"], 8.0)
        self.assertEqual([round(v, 2) for v in pins[0]["centre"]], [0.0, 0.0, 4.0])   # tip at LDraw -Y = our +Z

    def test_no_such_part_is_none_and_listed_missing(self):
        self.assertIsNone(ldraw.convert_part(self.lib, self.builder, "0000"))
        bundle = ldraw.convert_parts(self.lib, ["9999", "0000"])
        self.assertEqual(sorted(bundle["parts"]), ["9999"])
        self.assertEqual(bundle["missing"], ["0000"])
        self.assertEqual(bundle["format"], ldraw.BUNDLE_FORMAT)

    def test_weights_record_vendor_mass_and_density(self):
        part = self.convert("9999", {"9999": {"g": 0.5376, "dims": "2 x 1"}})
        self.assertEqual(part["source"], "vendor")
        self.assertEqual(part["mass_g"], 0.5376)
        self.assertAlmostEqual(part["density_g_cm3"], 1.05, places=2)
        self.assertIn("BrickLink 9999", part["source_note"])

    def test_without_a_weight_the_mass_is_a_flagged_estimate(self):
        part = self.convert("9999")
        self.assertEqual(part["source"], "placeholder")
        self.assertAlmostEqual(part["mass_g"], 0.54, places=2)

    def test_pack_and_unpack_round_trip(self):
        tris, _ = self.builder.build(self.lib.resolve("9999.dat"))
        tris = ldraw.to_ours(tris)
        mesh = ldraw.pack_mesh(tris)
        pos, nrm, idx = ldraw.unpack_mesh(mesh)
        self.assertEqual(idx.shape, (12, 3))
        self.assertEqual(pos.shape[0], mesh["verts"])
        self.assertEqual(mesh["verts"], 24)       # 8 corners × 3 crease-split normals
        self.assertTrue(np.allclose(np.abs(pos).max(axis=0), [8.0, 2.0, 4.0]))
        self.assertTrue(np.allclose(np.linalg.norm(nrm, axis=1), 1.0, atol=0.02))
        rebuilt = pos[idx]
        self.assertAlmostEqual(ldraw.mass_properties(rebuilt)[0], 512, places=2)

    def test_merge_connectors_splits_a_long_pin_per_module(self):
        a = np.array([0.0, 0.0, 0.0])
        b = np.array([0.0, -40.0, 0.0])          # 16 mm in LDraw -Y
        segs = ldraw.merge_connectors([("pin", a, b)])
        self.assertEqual([s["length"] for s in segs], [8.0, 8.0])
        self.assertEqual(sorted(round(s["centre"][2], 3) for s in segs), [4.0, 12.0])

    def test_merge_connectors_joins_pieces_of_one_hole(self):
        pieces = [("pin_hole", np.array([0.0, -10.0, 0.0]), np.array([0.0, -8.0, 0.0])),
                  ("pin_hole", np.array([0.0, 8.0, 0.0]), np.array([0.0, 10.0, 0.0]))]
        segs = ldraw.merge_connectors(pieces)
        self.assertEqual(len(segs), 1)
        self.assertEqual(segs[0]["length"], 8.0)

    def test_merge_drops_rim_only_fragments(self):
        segs = ldraw.merge_connectors([("pin_hole", np.array([0.0, 0.0, 0.0]), np.array([0.0, -2.0, 0.0]))])
        self.assertEqual(segs, [])

    def test_classify_names_the_primitive_families(self):
        self.assertEqual(ldraw.classify("confric5.dat"), "pin")
        self.assertEqual(ldraw.classify("connect.dat"), "pin")
        self.assertEqual(ldraw.classify("axlehol8.dat"), "axle")
        self.assertEqual(ldraw.classify("axlehole.dat"), "axle_hole")
        self.assertEqual(ldraw.classify("stud.dat"), "stud")
        self.assertEqual(ldraw.classify("stud4.dat"), "stud_hole")
        self.assertIsNone(ldraw.classify("beamhole.dat"))       # holes come from the mesh
        self.assertIsNone(ldraw.classify("4-4cyli.dat"))
        self.assertIsNone(ldraw.classify("s\\32013s01.dat"))

    def test_read_list_skips_comments(self):
        path = os.path.join(self.tmp.name, "list.txt")
        with open(path, "w") as fh:
            fh.write("# beams\n9999\n\n7777  # ring\n")
        self.assertEqual(ldraw.read_list(path), ["9999", "7777"])

    def test_main_writes_a_zlib_bundle(self):
        lst = os.path.join(self.tmp.name, "l.txt")
        with open(lst, "w") as fh:
            fh.write("9999\n7777\n")
        out = os.path.join(self.tmp.name, "b.json.zlib")
        self.assertEqual(ldraw.main([self.root, lst, out]), 0)
        with open(out, "rb") as fh:
            bundle = json.loads(zlib.decompress(fh.read()).decode())
        self.assertEqual(sorted(bundle["parts"]), ["7777", "9999"])
        out_json = os.path.join(self.tmp.name, "b.json")
        self.assertEqual(ldraw.main([self.root, lst, out_json]), 0)
        with open(out_json) as fh:
            self.assertEqual(sorted(json.load(fh)["parts"]), ["7777", "9999"])

    def test_main_usage(self):
        self.assertEqual(ldraw.main([]), 2)
