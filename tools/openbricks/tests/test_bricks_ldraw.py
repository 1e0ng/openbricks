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

    def test_pin_primitive_becomes_pin_segments(self):
        part = self.convert("6666")
        pins = sorted((c for c in part["connectors"] if c["kind"] == "pin"), key=lambda c: c["centre"][2])
        self.assertEqual(len(pins), 2, part["connectors"])
        self.assertEqual([p["length"] for p in pins], [8.0, 8.0])
        self.assertEqual([round(v, 2) for v in pins[1]["centre"]], [0.0, 0.0, 4.0])   # tip at LDraw -Y = our +Z
        self.assertEqual([round(v, 2) for v in pins[0]["centre"]], [0.0, 0.0, -4.0])

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
        self.assertEqual(mesh["scale"], 0.01)
        # a coarser quantum for scenery: the step travels with the record
        coarse = ldraw.pack_mesh(tris * 250.0, q=10.0)
        self.assertEqual(coarse["scale"], 0.1)
        pos, _, idx = ldraw.unpack_mesh(coarse)
        self.assertTrue(np.allclose(np.abs(pos).max(axis=0), [2000.0, 500.0, 1000.0]))
        self.assertAlmostEqual(ldraw.mass_properties(pos[idx])[0], 512 * 250.0 ** 3, delta=1e6)
        self.assertRaises(ValueError, ldraw.pack_mesh, tris * 400.0)   # 3.2 m does not fit the fine quantum

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
        self.assertIsNone(ldraw.classify("readme.txt"))

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


@unittest.skipIf(np is None, "numpy (the [sim] extra) is required")
class ConverterEdgeCaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.root = write_mini_library(cls.tmp.name)
        cls.lib = ldraw.Library(cls.root)
        cls.builder = ldraw.Builder(cls.lib)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def convert(self, number):
        part = ldraw.convert_part(self.lib, self.builder, number)
        self.assertIsNotNone(part, number)
        return part

    def test_invertnext_makes_a_shell(self):
        part = self.convert("5555")
        self.assertAlmostEqual(part["volume_mm3"], 512 - 512 / 8, places=3)

    def test_edge_only_primitive_still_places_a_feature_and_studs_are_found(self):
        part = self.convert("4444")
        kinds = sorted(c["kind"] for c in part["connectors"])
        self.assertEqual(kinds, ["axle_hole", "stud", "stud"], part["connectors"])
        axle = [c for c in part["connectors"] if c["kind"] == "axle_hole"][0]
        self.assertEqual(axle["length"], 8.0)              # from the edge lines' y extent
        studs = [c for c in part["connectors"] if c["kind"] == "stud"]
        self.assertTrue(all(s["length"] == 1.6 for s in studs))

    def test_two_pin_halves_share_the_cached_primitive(self):
        part = self.convert("6666")
        pins = sorted(round(c["centre"][2], 2) for c in part["connectors"] if c["kind"] == "pin")
        self.assertEqual(pins, [-4.0, 4.0])
        self.assertIn(self.lib.resolve("confric5.dat"), self.builder.extent_cache)

    def test_self_reference_terminates(self):
        part = self.convert("3333")
        self.assertGreater(part["volume_mm3"], 0)

    def test_garbage_lines_are_skipped(self):
        part = self.convert("2222")
        self.assertEqual(part["mesh"]["tris"], 2)
        self.assertEqual(part["mass_model"], "box")          # two triangles enclose nothing
        self.assertEqual(part["source"], "placeholder")

    def test_lines_only_part_is_not_a_brick(self):
        self.assertIsNone(ldraw.convert_part(self.lib, self.builder, "1111"))
        self.assertNotIn("readme.txt", self.lib.index)

    def test_mass_properties_of_nothing(self):
        vol, com, I = ldraw.mass_properties(np.zeros((0, 3, 3)))
        self.assertEqual(vol, 0.0)
        flat = np.array([[[0, 0, 0], [1, 0, 0], [0, 1, 0]]], dtype=float)   # zero enclosed volume
        vol, com, I = ldraw.mass_properties(flat)
        self.assertEqual(vol, 0.0)
        self.assertTrue(np.all(com == 0))

    def test_detect_bores_rejects_what_is_not_a_bore(self):
        self.assertEqual(ldraw.detect_bores(np.zeros((0, 3, 3))), [])
        # a flat wall: many faces, one normal direction
        wall = []
        for i in range(24):
            z0, z1 = i * 0.5, i * 0.5 + 0.5
            wall.append([[0, -1, z0], [0, 1, z0], [0, 1, z1]])
            wall.append([[0, -1, z0], [0, 1, z1], [0, -1, z1]])
        self.assertEqual(ldraw.detect_bores(np.array(wall, dtype=float)), [])
        # a bore with too few faces
        tris, _ = self.builder.build(self.lib.resolve("7777.dat"))
        tris = ldraw.to_ours(tris)
        self.assertEqual(ldraw.detect_bores(tris, min_votes=10_000), [])   # fewer parallel faces than votes needed
        self.assertEqual(ldraw.detect_bores(tris, min_votes=40), [])       # the bore's 32 faces are not enough
        self.assertEqual(len(ldraw.detect_bores(tris)), 1)
        # a bore too short to be a hole
        short = tris * np.array([1, 1, 0.2])
        self.assertEqual(ldraw.detect_bores(short), [])
        # a few stray bore faces further along the same axis do not make a second hole
        p1, p2, p3 = tris[:, 0], tris[:, 1], tris[:, 2]
        fn = np.cross(p2 - p1, p3 - p1)
        wall = tris[(np.abs(fn[:, 2]) < 1e-6) & (np.linalg.norm(tris.mean(axis=1)[:, :2], axis=1) < 3.0)]
        stray = wall[:4] + np.array([0, 0, 30.0])
        self.assertEqual(len(ldraw.detect_bores(np.concatenate([tris, stray]))), 1)

    def test_merge_connectors_keeps_distinct_features_apart(self):
        a = np.array([0.0, 0.0, 0.0])
        down = np.array([0.0, -20.0, 0.0])
        segs = ldraw.merge_connectors([
            ("pin_hole", a, down),
            ("axle_hole", a, down),                                    # another kind
            ("pin_hole", a, np.array([20.0, 0.0, 0.0])),              # not parallel
            ("pin_hole", np.array([30.0, 0.0, 0.0]), np.array([30.0, -20.0, 0.0])),   # parallel but off-axis
            ("pin_hole", np.array([0.0, -60.0, 0.0]), np.array([0.0, -80.0, 0.0])),   # collinear but a wall apart
            ("pin_hole", a, a),                                        # zero length
        ])
        self.assertEqual(len(segs), 5)
        self.assertEqual(sorted(s["kind"] for s in segs), ["axle_hole", "pin_hole", "pin_hole", "pin_hole", "pin_hole"])

    def test_main_with_weights(self):
        lst = os.path.join(self.tmp.name, "l.txt")
        with open(lst, "w") as fh:
            fh.write("9999\n")
        weights = os.path.join(self.tmp.name, "w.json")
        with open(weights, "w") as fh:
            json.dump({"9999": {"g": 0.5376}}, fh)
        out = os.path.join(self.tmp.name, "b.json")
        self.assertEqual(ldraw.main([self.root, lst, out, "--weights", weights]), 0)
        with open(out) as fh:
            self.assertEqual(json.load(fh)["parts"]["9999"]["source"], "vendor")
