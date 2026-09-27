# SPDX-License-Identifier: MIT
"""Maps as JSON (``openbricks-map/1``): read and checked, turned into the
model MuJoCo is given, converted from the MJCF the maps were before
4.32.0, written as a person reads them, and packed into one exported
file and back."""
import base64
import json
import tempfile
import unittest
from pathlib import Path

from openbricks_sim import mapfile

try:
    import mujoco
    import numpy as np
except ImportError:                      # pragma: no cover
    mujoco = None

_WORLDS = Path(__file__).resolve().parents[1] / "openbricks_sim" / "worlds"

# what the shipped maps used, in one MJCF: every element and attribute the converter takes
_MJCF = """<!--
  A test map.
    Its header, indented.
-->
<mujoco model="fixture">
  <compiler angle="degree" coordinate="local"/>
  <!-- physics as the maps have it -->
  <option timestep="0.001" iterations="20" solver="Newton" gravity="0 0 -9.81"/>
  <visual>
    <!-- a light from the eye -->
    <headlight diffuse="0.6 0.6 0.6" ambient="0.5 0.5 0.5" specular="0 0 0"/>
  </visual>
  <asset>
    <!-- the walls' colour -->
    <material name="wall" rgba="0.90 0.90 0.90 1"/>
    <material name="shiny" rgba="0.1 0.2 0.3 1" specular="0" shininess="0" reflectance="0"/>
  </asset>
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" diffuse="0.95 0.95 0.95"/>
    <!-- the floor,
         over two lines -->
    <geom name="floor" type="plane" pos="0.5 0 0" size="1.0 0.5 0.05" rgba="1 1 1 1" friction="0.9 0.02 0.0001"/>
    <geom name="wall_n" type="box" pos=" 0     0.579  0.02" size="1.181 0.0075 0.02" material="wall"/>
    <geom name="line" type="box" pos="0.60 0 0.001" size="0.70 0.010 0.001" rgba="0.03 0.03 0.03 1"/>
    <camera name="overhead" pos="0 0 2.6" xyaxes="1 0 0 0 1 0"/>
    <!-- trailing words -->
  </worldbody>
</mujoco>
"""


class ReadCheckTests(unittest.TestCase):
    def test_a_map_is_checked_and_its_faults_named(self):
        good = {"format": mapfile.FORMAT, "props": [{"name": "p", "ldr": "p.ldr", "pos": [0, 0, 0], "mass": 0.01}]}
        self.assertIs(mapfile.check(good), good)
        for bad, words in [
            ({"format": "nope"}, "is not an"),
            ([], "is not an"),
            ({"format": mapfile.FORMAT, "geoms": {}}, "'geoms' must be a list"),
            ({"format": mapfile.FORMAT, "props": [{"ldr": "p.ldr", "pos": [0, 0, 0], "mass": 1}]}, "needs a name"),
            ({"format": mapfile.FORMAT, "props": [{"name": "p", "pos": [0, 0, 0]}]}, "needs one model"),
            ({"format": mapfile.FORMAT, "props": [{"name": "p", "ldr": "a", "file": "b", "pos": [0, 0, 0], "mass": 1}]}, "needs one model"),
            ({"format": mapfile.FORMAT, "props": [{"name": "p", "file": "b", "pos": [0, 0]}]}, "pos must be 3 numbers"),
            ({"format": mapfile.FORMAT, "props": [{"name": "p", "ldr": "a", "pos": [0, 0, 0]}]}, "needs a mass"),
        ]:
            with self.subTest(bad=bad):
                with self.assertRaises(mapfile.MapError) as cm:
                    mapfile.check(bad)
                self.assertIn(words, str(cm.exception))

    def test_a_map_file_is_read_or_named_unreadable(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "map.json"
            with self.assertRaises(mapfile.MapError):
                mapfile.load(path)
            path.write_text("{not json")
            with self.assertRaises(mapfile.MapError):
                mapfile.load(path)
            m = {"format": mapfile.FORMAT, "name": "x", "lights": [{"pos": [0, 0, 1]}]}
            mapfile.save(m, path)
            self.assertEqual(mapfile.load(path), m)
            with self.assertRaises(mapfile.MapError):
                mapfile.save({"format": "nope"}, path)

    def test_every_shipped_map_reads(self):
        for path in sorted(_WORLDS.glob("*/map.json")):
            with self.subTest(map=path.parent.name):
                m = mapfile.load(path)
                self.assertTrue(m["name"])
                self.assertTrue(m.get("geoms"), "a map has a floor at least")
                self.assertEqual(path.read_text(), mapfile.dumps(m) + "\n", "stored as the formatter writes it")


class WrittenAsReadTests(unittest.TestCase):
    def test_vectors_stay_on_a_line_and_text_is_written_as_is(self):
        text = mapfile.dumps({"a": [0, 1.5, -2], "b": {"c": "é — ×"}, "d": [], "e": {},
                              "f": [{"g": 1}], "h": ["short", "words"], "i": ["x" * 60, "y" * 60]})
        self.assertIn('"a": [0, 1.5, -2]', text)
        self.assertIn('"c": "é — ×"', text)
        self.assertIn('"d": []', text)
        self.assertIn('"e": {}', text)
        self.assertIn('"h": ["short", "words"]', text)
        self.assertIn('"i": [\n', text, "long lines of text go one to a line")
        self.assertEqual(json.loads(text)["f"], [{"g": 1}])


class ToMjcfTests(unittest.TestCase):
    def test_the_model_text_carries_every_section_in_order(self):
        m = mapfile.from_mjcf(_MJCF)
        text = mapfile.to_mjcf(m, "/nowhere")
        order = [text.index(t) for t in ("<compiler", "<option", "<visual>", "<asset>", "<light", 'name="floor"', "<camera")]
        self.assertEqual(order, sorted(order))
        self.assertNotIn("note", text)
        self.assertIn('iterations="20"', text)
        self.assertIn('gravity="0 0 -9.81"', text)
        # a map with nothing but a name is still a model
        bare = mapfile.to_mjcf({"format": mapfile.FORMAT}, "/nowhere")
        self.assertIn("<worldbody>", bare)
        self.assertNotIn("<asset>", bare)
        self.assertEqual(mapfile._element("x", {}, ""), "<x/>")
        self.assertEqual(mapfile._value(True), "true")
        self.assertEqual(mapfile._value(False), "false")

    @unittest.skipIf(mujoco is None, "mujoco not installed")
    def test_a_converted_map_makes_the_model_its_mjcf_made(self):
        # the conversion's promise, on everything the maps used (the six shipped maps were checked
        # the same way, every body, geom, material, light and camera, when they became JSON)
        a = mujoco.MjModel.from_xml_string(_MJCF)
        b = mujoco.MjModel.from_xml_string(mapfile.to_mjcf(mapfile.from_mjcf(_MJCF), "/nowhere"))
        for field in ["geom_type", "geom_size", "geom_pos", "geom_quat", "geom_friction", "geom_rgba", "geom_matid",
                      "mat_rgba", "mat_specular", "mat_shininess", "mat_reflectance", "light_pos", "light_dir",
                      "light_diffuse", "cam_pos", "cam_quat"]:
            self.assertTrue(np.array_equal(getattr(a, field), getattr(b, field)), field)
        for field in ["timestep", "iterations", "solver", "gravity"]:
            self.assertTrue(np.array_equal(getattr(a.opt, field), getattr(b.opt, field)), field)
        for field in ["diffuse", "ambient", "specular"]:
            self.assertTrue(np.array_equal(getattr(a.vis.headlight, field), getattr(b.vis.headlight, field)), field)
        names = lambda mm: [mujoco.mj_id2name(mm, mujoco.mjtObj.mjOBJ_GEOM, i) for i in range(mm.ngeom)]  # noqa: E731
        self.assertEqual(names(a), names(b))


class FromMjcfTests(unittest.TestCase):
    def test_comments_become_about_and_notes_and_values_numbers(self):
        m = mapfile.from_mjcf(_MJCF)
        self.assertEqual(m["about"], ["A test map.", "  Its header, indented.", "", "physics as the maps have it", "", "trailing words"])
        self.assertEqual(list(m), ["format", "name", "about", "physics", "headlight", "materials", "lights", "geoms", "cameras"])
        self.assertEqual(m["physics"], {"timestep": 0.001, "iterations": 20, "solver": "Newton", "gravity": [0, 0, -9.81]})
        self.assertEqual(m["materials"][0], {"name": "wall", "rgba": [0.9, 0.9, 0.9, 1], "note": "the walls' colour"})
        floor = m["geoms"][0]
        self.assertEqual(floor["note"], ["the floor,", "over two lines"])
        self.assertEqual(floor["size"], [1.0, 0.5, 0.05])
        self.assertEqual(m["geoms"][1]["pos"], [0, 0.579, 0.02], "spaced values read")
        self.assertEqual(m["cameras"][0]["xyaxes"], [1, 0, 0, 0, 1, 0])
        self.assertEqual(mapfile._parse_value("pos", "1 two"), "1 two", "not all numbers: kept as text")
        self.assertEqual(mapfile._note([]), None)
        # an empty comment says nothing; one that ends in blank lines ends where its words do
        self.assertEqual(mapfile._note(["", "   \n  "]), None)
        self.assertEqual(mapfile._note([" words\n\n   "]), "words")
        self.assertNotIn("headlight", str(m.get("about")), "a comment inside <visual> is dropped with it")

    def test_placeholders_become_props(self):
        text = _MJCF.replace("<camera", '<lego_prop name="clef" ldr="props/clef.ldr" pos="0.1 0.2 0.005" mass="0.05" '
                                        'yaw="30" pitch="0" color="red" fixed="true"/>\n'
                                        '    <assembly_prop name="t" file="t.json" pos="0 0 0" roll="90"/>\n    <camera')
        m = mapfile.from_mjcf(text)
        self.assertEqual(m["props"], [
            {"name": "clef", "ldr": "props/clef.ldr", "pos": [0.1, 0.2, 0.005], "mass": 0.05, "yaw": 30.0, "color": "red", "fixed": True},
            {"name": "t", "file": "t.json", "pos": [0.0, 0.0, 0.0], "roll": 90.0},
        ])
        for bad in ['<lego_prop ldr="a" pos="0 0 0" mass="1"/>', '<assembly_prop name="t" file="t" pos="0 0"/>']:
            with self.assertRaises(mapfile.MapError):
                mapfile.from_mjcf(_MJCF.replace("<camera", bad + "\n    <camera"))

    def test_anything_the_maps_never_used_is_refused_by_name(self):
        for old, new, words in [
            ('<compiler angle="degree" coordinate="local"/>', '<compiler angle="radian"/>', "unsupported <compiler"),
            ("<headlight", "<quality/><headlight", "unsupported <visual><quality>"),
            ('<material name="wall"', '<hfield name="h" size="1 1 1 1"/><material name="wall"', "unsupported <asset><hfield>"),
            ("<light ", '<body name="b"/><light ', "unsupported <worldbody><body>"),
            ("<visual>", "<default/><visual>", "unsupported <default>"),
        ]:
            with self.subTest(words=words):
                with self.assertRaises(mapfile.MapError) as cm:
                    mapfile.from_mjcf(_MJCF.replace(old, new))
                self.assertIn(words, str(cm.exception))
        with self.assertRaises(mapfile.MapError):
            mapfile.from_mjcf("<not xml")
        with self.assertRaises(mapfile.MapError):
            mapfile.from_mjcf("<robot/>")
        # a map with no comments has no about
        self.assertNotIn("about", mapfile.from_mjcf('<mujoco><worldbody><geom type="plane" size="1 1 1"/></worldbody></mujoco>'))


class PackTests(unittest.TestCase):
    def test_pack_and_unpack_keep_every_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "src"
            (src / "props").mkdir(parents=True)
            (src / "mat.png").write_bytes(b"\x89PNG\x00\xff")
            (src / "props" / "a.ldr").write_text("1 4 0 0 0 1 0 0 0 1 0 0 0 1 3001.dat")
            (src / "props" / "odd.ldr").write_bytes(b"\xff\xfe not utf-8")
            (src / "props" / "bad.json").write_text("{not json")
            (src / "__pycache__").mkdir()
            (src / "__pycache__" / "x.pyc").write_bytes(b"x")
            outside = Path(tmp) / "elsewhere"
            outside.mkdir()
            (outside / "a.ldr").write_text("different")
            m = {"format": mapfile.FORMAT, "name": "packed",
                 "props": [{"name": "p", "ldr": "props/a.ldr", "pos": [0, 0, 0], "mass": 0.01},
                           {"name": "q", "ldr": str(outside / "a.ldr"), "pos": [0, 0, 0], "mass": 0.01}]}
            mapfile.save(m, src / "map.json")
            obj = mapfile.pack(m, src)
            self.assertEqual(sorted(obj["files"]), ["mat.png", "props/a-2.ldr", "props/a.ldr", "props/bad.json", "props/odd.ldr"])
            self.assertEqual(obj["files"]["props/a.ldr"], {"text": "1 4 0 0 0 1 0 0 0 1 0 0 0 1 3001.dat"})
            self.assertEqual(obj["files"]["mat.png"], {"base64": base64.b64encode(b"\x89PNG\x00\xff").decode()})
            self.assertIn("base64", obj["files"]["props/odd.ldr"], "not text after all")
            self.assertIn("base64", obj["files"]["props/bad.json"], "not JSON after all")
            self.assertEqual(obj["props"][1]["ldr"], "props/a-2.ldr", "an outside model of the same name keeps its own")
            self.assertEqual(mapfile.numbered("tower.assembly.json", 3), "tower-3.assembly.json")
            self.assertEqual(mapfile.numbered("README", 2), "README-2")
            self.assertEqual(m["props"][1]["ldr"], str(outside / "a.ldr"), "the map given is left as it was")
            out = Path(tmp) / "x.map.json"
            mapfile.export_to(m, src, out)
            back = Path(tmp) / "back"
            back.mkdir()
            got = mapfile.unpack(mapfile.read_export(out), back)
            self.assertNotIn("files", got)
            self.assertEqual((back / "mat.png").read_bytes(), b"\x89PNG\x00\xff")
            self.assertEqual((back / "props" / "odd.ldr").read_bytes(), b"\xff\xfe not utf-8")
            self.assertEqual((back / "props" / "a-2.ldr").read_text(), "different")
            self.assertEqual(mapfile.load(back / "map.json"), got)

    def test_an_export_that_breaks_the_rules_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = {"format": mapfile.FORMAT, "name": "x"}
            for files, words in [
                ([], "must be a table"),
                ({"/abs.txt": {"text": ""}}, "outside its folder"),
                ({"a/../../b": {"text": ""}}, "outside its folder"),
                ({"map.json": {"text": ""}}, "outside its folder"),
                ({"a.txt": "text"}, "must hold json, text or base64"),
                ({"a.txt": {"text": "", "json": 1}}, "must hold json, text or base64"),
                ({"a.txt": {"zip": ""}}, "not 'zip'"),
            ]:
                with self.subTest(words=words):
                    with self.assertRaises(mapfile.MapError) as cm:
                        mapfile.unpack(dict(base, files=files), tmp)
                    self.assertIn(words, str(cm.exception))
            with self.assertRaises(mapfile.MapError):
                mapfile.read_export(Path(tmp) / "none.json")
            # a model the map names inside its folder must be there
            with self.assertRaises(mapfile.MapError):
                mapfile.pack(dict(base, props=[{"name": "p", "file": "props/gone.json", "pos": [0, 0, 0]}]), tmp)
            with self.assertRaises(mapfile.MapError):
                mapfile.pack(dict(base, props=[{"name": "p", "file": "props/gone.json", "pos": [0, 0, 0]}]), None)


if __name__ == "__main__":
    unittest.main()
