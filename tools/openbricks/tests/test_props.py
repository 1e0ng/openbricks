# SPDX-License-Identifier: MIT
"""The map editor's data side: props found, moved, added and removed in
a JSON map, maps saved as the user's own where the server lists them,
old MJCF maps of the user's own converted, and maps exported and
imported as one JSON file."""
import copy
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from openbricks_sim import mapfile, props

try:
    import mujoco
except ImportError:                      # pragma: no cover
    mujoco = None

_WORLDS = Path(__file__).resolve().parents[1] / "openbricks_sim" / "worlds"
_ELEMENTARY = _WORLDS / "wro_2026_elementary_robot_rockstars" / "map.json"

_TWO = {
    "format": mapfile.FORMAT,
    "name": "two",
    "geoms": [{"name": "floor", "type": "plane", "size": [1, 1, 0.01]}],
    "props": [
        {"name": "clef", "ldr": "props/clef.ldr", "pos": [0.1, 0.2, 0.005], "mass": 0.05},
        {"name": "note_red", "ldr": "props/note.ldr", "pos": [-0.3, 0, 0.005], "mass": 0.02, "yaw": 30,
         "color": "red", "note": "the red note"},
    ],
}


class DataDirTests(unittest.TestCase):
    def test_the_data_directory_follows_the_markers_rule(self):
        self.assertEqual(props.data_dir({"OPENBRICKS_DATA_DIR": "/x/data"}, home="/h"), Path("/x/data"))
        self.assertEqual(props.data_dir({"XDG_DATA_HOME": "/xdg"}, home="/h"), Path("/xdg/openbricks"))
        self.assertEqual(props.data_dir({}, home="/h"), Path("/h/.local/share/openbricks"))
        # the one rule, pinned for both sides (the sim's markers::data_dir reads the same file)
        with open(os.path.join(os.path.dirname(__file__), "data_dir_cases.json")) as fh:
            cases = json.load(fh)["cases"]
        self.assertGreaterEqual(len(cases), 10)
        for case in cases:
            env = {k: v for k, v in case.items() if k != "expect"}
            got = props.data_dir(env)
            self.assertEqual(got.as_posix().lstrip("./") if got.as_posix().startswith("./") else got.as_posix(),
                             case["expect"], case)
        self.assertEqual(props.home_dir({"HOME": "/h", "USERPROFILE": "/u"}), "/h")
        self.assertEqual(props.home_dir({"USERPROFILE": "C:\\Users\\me"}), "C:\\Users\\me")
        self.assertEqual(props.home_dir({}), ".")
        self.assertEqual(props.data_dir({"OPENBRICKS_DATA_DIR": "~\\ob", "USERPROFILE": "C:\\Users\\me"}),
                         Path(os.path.join("C:\\Users\\me", "ob")))
        # the real environment goes through the same function
        with mock.patch.dict(os.environ, {"OPENBRICKS_DATA_DIR": "/x/data"}):
            self.assertEqual(props.data_dir(), Path("/x/data"))
        self.assertEqual(props.user_worlds_dir({}, home="/h"), Path("/h/.local/share/openbricks/worlds"))
        self.assertEqual(props.list_user_worlds({"OPENBRICKS_DATA_DIR": "/does/not/exist"}), [])


class PropTextTests(unittest.TestCase):
    def test_props_are_read_with_their_pose_colour_and_yaw(self):
        found = props.props_in(_TWO)
        self.assertEqual([p["name"] for p in found], ["clef", "note_red"])
        self.assertEqual(found[0]["pos"], (0.1, 0.2, 0.005))
        self.assertEqual((found[0]["yaw"], found[0]["color"], found[0]["mass"], found[0]["tag"]), (0.0, None, 0.05, "lego_prop"))
        self.assertEqual((found[1]["yaw"], found[1]["color"], found[1]["ldr"]), (30.0, "red", "props/note.ldr"))
        self.assertEqual(props.props_in({"format": mapfile.FORMAT}), [])

    def test_the_shipped_elementary_map_has_its_props(self):
        names = [p["name"] for p in props.props_in(mapfile.load(_ELEMENTARY))]
        self.assertIn("clef", names)
        self.assertIn("microphone", names)
        self.assertGreater(len(names), 10)

    def test_a_prop_is_pitched_and_rolled_as_a_workbench_part_is_turned(self):
        m = copy.deepcopy(_TWO)
        m["props"][0].update(yaw=10, pitch=90, roll=-45)
        clef = props.props_in(m)[0]
        self.assertEqual((clef["yaw"], clef["pitch"], clef["roll"]), (10.0, 90.0, -45.0))
        self.assertEqual((props.props_in(_TWO)[0]["pitch"], props.props_in(_TWO)[0]["roll"]), (0.0, 0.0))
        # kept when a move does not name them; each written only when it turns
        moved = props.with_prop_moved(m, "clef", 0.5, -0.25, 0.0)
        self.assertEqual(moved["props"][0], {"name": "clef", "ldr": "props/clef.ldr", "pos": [0.5, -0.25, 0.005], "mass": 0.05,
                                             "pitch": 90.0, "roll": -45.0})
        tipped = props.with_prop_moved(_TWO, "clef", 0.5, -0.25, 30.0, 90.0, 0.0, 0.024)
        self.assertEqual(tipped["props"][0], {"name": "clef", "ldr": "props/clef.ldr", "pos": [0.5, -0.25, 0.024], "mass": 0.05,
                                              "yaw": 30.0, "pitch": 90.0})
        upright = props.with_prop_moved(tipped, "clef", 0.5, -0.25, 30.0, 0.0, 0.0)
        self.assertNotIn("pitch", upright["props"][0])
        self.assertEqual(upright["props"][0]["pos"][2], 0.024, "the height kept when not given")
        self.assertEqual(_TWO["props"][0]["pos"], [0.1, 0.2, 0.005], "the map given is left as it was")
        # an assembly prop reads and writes them too
        doc = {"format": mapfile.FORMAT, "props": [{"name": "b", "file": "b.json", "pos": [0, 0, 0], "yaw": 5, "pitch": -90,
                                                    "roll": 180, "fixed": True}]}
        b = props.props_in(doc)[0]
        self.assertEqual((b["yaw"], b["pitch"], b["roll"], b["fixed"], b["tag"]), (5.0, -90.0, 180.0, True, "assembly_prop"))
        self.assertEqual(props.with_prop_fixed(doc, "b", True)["props"][0]["fixed"], True)
        # the quaternion: roll about x, then pitch about y, then yaw about z
        import math
        h = math.sqrt(0.5)
        for (yaw, pitch, roll), want in [((90, 0, 0), (h, 0, 0, h)), ((0, 90, 0), (h, 0, h, 0)), ((0, 0, 90), (h, h, 0, 0))]:
            got = props.euler_quat(yaw, pitch, roll)
            for a, b2 in zip(got, want):
                self.assertAlmostEqual(a, b2, places=9)
        from openbricks_sim import assembly
        for yaw, pitch, roll in [(30.0, 60.0, -20.0), (90.0, 90.0, 0.0), (-135.0, 10.0, 170.0)]:
            w, x, y, z = props.euler_quat(yaw, pitch, roll)
            q_mat = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
            e_mat = assembly.rot_mat([roll, pitch, yaw])
            for r in range(3):
                for c in range(3):
                    self.assertAlmostEqual(q_mat[r][c], e_mat[r][c], places=9, msg=(yaw, pitch, roll))
        self.assertEqual(props.quat_attr(0, 0, 0), "")
        self.assertTrue(props.quat_attr(0, 0, 1).startswith(' quat="'))

    def test_the_lowest_point_follows_the_turn(self):
        from openbricks_sim import assembly
        # one brick box 48 x 16 x 9.6 mm, its centre 4.8 mm below the origin
        brick = [{"quat": [1.0, 0.0, 0.0, 0.0], "pos_m": [0.0, 0.0, -0.0048], "half_m": [0.024, 0.008, 0.0048]}]
        self.assertAlmostEqual(assembly.prop_lowest_m(brick), -0.0096, places=6)
        # pitched a quarter: its length points down, and its centre turns to the origin's height
        self.assertAlmostEqual(assembly.prop_lowest_m(brick, props.euler_quat(0, 90, 0)), -0.024, places=6)
        # rolled a quarter: its width points down
        self.assertAlmostEqual(assembly.prop_lowest_m(brick, props.euler_quat(0, 0, 90)), -0.008, places=6)
        # turned about z alone: nothing changes
        self.assertAlmostEqual(assembly.prop_lowest_m(brick, props.euler_quat(37, 0, 0)), -0.0096, places=6)
        # upside down: the centre is 4.8 mm above the origin now
        self.assertAlmostEqual(assembly.prop_lowest_m(brick, props.euler_quat(0, 180, 0)), 0.0, places=6)

    def test_a_prop_moves_keeping_its_height_and_the_rest_of_the_map(self):
        moved = props.with_prop_moved(_TWO, "clef", 0.5, -0.25, 90.0)
        p = props.props_in(moved)[0]
        self.assertEqual((p["pos"], p["yaw"]), ((0.5, -0.25, 0.005), 90.0))
        self.assertEqual(moved["props"][1], _TWO["props"][1], "the other prop is untouched, its note too")
        self.assertEqual(moved["geoms"], _TWO["geoms"])
        # back to no turn: the yaw goes again
        self.assertNotIn("yaw", props.with_prop_moved(moved, "clef", 0.5, -0.25, 0.0)["props"][0])
        with self.assertRaises(props.PropError):
            props.with_prop_moved(_TWO, "nothing", 0.0, 0.0, 0.0)

    def test_another_prop_like_one_is_added_after_it_with_a_free_name(self):
        m, name = props.with_prop_added(_TWO, "note_red", 0.0, 0.1, 45.0)
        self.assertEqual(name, "note_red_2")
        found = props.props_in(m)
        self.assertEqual([p["name"] for p in found], ["clef", "note_red", "note_red_2"])
        new = found[2]
        self.assertEqual((new["pos"], new["yaw"], new["color"], new["ldr"], new["mass"]), ((0.0, 0.1, 0.005), 45.0, "red", "props/note.ldr", 0.02))
        self.assertNotIn("note", m["props"][2], "the copy leaves the original's note behind")
        m, name = props.with_prop_added(m, "note_red_2", 0.2, 0.2, 0.0)
        self.assertEqual(name, "note_red_3", "numbering continues from the family")
        self.assertEqual(props.unique_name(m, "fresh"), "fresh")
        with self.assertRaises(props.PropError):
            props.with_prop_added(_TWO, "nothing", 0.0, 0.0, 0.0)

    def test_a_prop_is_removed(self):
        m = props.with_prop_removed(_TWO, "clef")
        self.assertEqual([p["name"] for p in props.props_in(m)], ["note_red"])
        self.assertEqual(len(_TWO["props"]), 2, "the map given is left as it was")
        with self.assertRaises(props.PropError):
            props.with_prop_removed(_TWO, "nothing")

    def test_documents_are_props_too_and_props_can_be_stuck(self):
        m, name = props.with_model_added(_TWO, "tower", "/abs/tower.assembly.json", 0.25, -0.1, 30.0)
        self.assertEqual(name, "tower")
        found = props.props_in(m)
        self.assertEqual([p["name"] for p in found], ["clef", "note_red", "tower"])
        t = found[2]
        self.assertEqual((t["tag"], t["file"], t["pos"], t["yaw"], t["fixed"]), ("assembly_prop", "/abs/tower.assembly.json", (0.25, -0.1, 0.0), 30.0, False))
        self.assertEqual(m["props"][2], {"name": "tower", "file": "/abs/tower.assembly.json", "pos": [0.25, -0.1, 0.0], "yaw": 30.0})
        m, name = props.with_model_added(m, "tower", "/abs/tower.assembly.json", 0.0, 0.0, 0.0)
        self.assertEqual(name, "tower_2")
        # stuck: the flag is written, moving keeps it, a copy keeps it, freeing drops it
        stuck = props.with_prop_fixed(m, "tower", True)
        self.assertTrue(stuck["props"][2]["fixed"])
        moved = props.with_prop_moved(stuck, "tower", 0.5, 0.5, 0.0)
        self.assertEqual(moved["props"][2], {"name": "tower", "file": "/abs/tower.assembly.json", "pos": [0.5, 0.5, 0.0], "fixed": True})
        copied, cname = props.with_prop_added(moved, "tower", 0.6, 0.6, 0.0)
        self.assertEqual(cname, "tower_3")
        self.assertTrue(next(p for p in props.props_in(copied) if p["name"] == "tower_3")["fixed"], "a copy of a stuck prop is stuck")
        freed = props.with_prop_fixed(moved, "tower", False)
        self.assertNotIn("fixed", freed["props"][2])
        self.assertEqual(len(props.with_prop_removed(m, "tower_2")["props"]), 3)
        # a map with no props takes one
        bare, _ = props.with_model_added({"format": mapfile.FORMAT}, "x", "/x", 0, 0, 0)
        self.assertEqual([p["name"] for p in bare["props"]], ["x"])

    def test_names_become_directory_slugs(self):
        self.assertEqual(props.slug("My Layout 2 "), "my-layout-2")
        self.assertEqual(props.slug("WRO/elementary: notes!"), "wro-elementary-notes")
        with self.assertRaises(props.PropError):
            props.slug("  ")


class SaveTests(unittest.TestCase):
    def _src(self, tmp):
        src = Path(tmp) / "src"
        (src / "props").mkdir(parents=True)
        mapfile.save(_TWO, src / "map.json")
        (src / "mat.png").write_bytes(b"png")
        (src / "props" / "clef.ldr").write_text("1 4 0 0 0 1 0 0 0 1 0 0 0 1 3001.dat")
        (src / "README.md").write_text("notes")
        return src

    def test_a_map_is_saved_with_its_files_listed_and_replaced_by_name(self):
        with tempfile.TemporaryDirectory() as tmp:
            env = {"OPENBRICKS_DATA_DIR": tmp}
            src = self._src(tmp)
            alias, path = props.save_as(src, props.with_prop_moved(_TWO, "clef", 1.0, 1.0, 0.0), "My Layout", env=env)
            self.assertEqual(alias, "my-layout")
            self.assertEqual(Path(path), Path(tmp) / "worlds" / "my-layout" / "map.json")
            self.assertEqual(sorted(p.name for p in Path(path).parent.iterdir()), ["README.md", "map.json", "mat.png", "props"])
            self.assertEqual((Path(path).parent / "props" / "clef.ldr").read_text(), "1 4 0 0 0 1 0 0 0 1 0 0 0 1 3001.dat")
            self.assertEqual(mapfile.load(path)["props"][0]["pos"], [1.0, 1.0, 0.005])
            listed = props.list_user_worlds(env)
            self.assertEqual(listed, [{"alias": "my-layout", "path": path, "dir": str(Path(path).parent), "user": True}])
            # saving again under the same name replaces the map
            alias2, path2 = props.save_as(src, _TWO, "my layout", env=env)
            self.assertEqual((alias2, path2), (alias, path))
            self.assertEqual(mapfile.load(path), _TWO, "the map as given")
            self.assertEqual(len(props.list_user_worlds(env)), 1)
            # the map opened and saved over itself (its own directory the source) keeps its files
            own = Path(path).parent
            alias5, path5 = props.save_as(own, props.with_prop_moved(_TWO, "clef", 2.0, 2.0, 0.0), "my layout", env=env)
            self.assertEqual((alias5, path5), (alias, path))
            self.assertEqual(sorted(p.name for p in own.iterdir()), ["README.md", "map.json", "mat.png", "props"])
            self.assertEqual((own / "mat.png").read_bytes(), b"png")
            self.assertEqual(mapfile.load(path)["props"][0]["pos"], [2.0, 2.0, 0.005])
            # a shipped alias is never shadowed; a nameless map is refused
            with self.assertRaises(props.PropError):
                props.save_as(src, _TWO, "practice line", reserved=("practice-line",), env=env)
            with self.assertRaises(props.PropError):
                props.save_as(src, _TWO, "!!!", env=env)
            # a map without a source directory (the map alone) still saves
            alias3, path3 = props.save_as(None, _TWO, "bare", env=env)
            self.assertEqual(sorted(p.name for p in Path(path3).parent.iterdir()), ["map.json"])
            # a model kept under the data directory comes into the map's props/, referenced from there;
            # two different models of one name keep both
            staged = props.stage_file('{"a": 1}', "Tower Two", "assembly.json", env=env)
            self.assertTrue(staged.startswith(str(Path(tmp) / "props" / "tower-two-")) and staged.endswith(".assembly.json"))
            self.assertEqual(props.stage_file('{"a": 1}', "tower two", "assembly.json", env=env), staged, "the same text is kept once")
            other = props.stage_file('{"a": 2}', "tower two", "assembly.json", env=env)
            m, _ = props.with_model_added(_TWO, "tower", staged, 0.1, 0.1, 0.0)
            m, _ = props.with_model_added(m, "tower", other, 0.2, 0.2, 0.0)
            alias4, path4 = props.save_as(src, m, "with models", env=env)
            saved = Path(path4).read_text()
            self.assertNotIn(tmp, saved, "no absolute paths in a saved map")
            refs = [p["file"] for p in mapfile.load(path4)["props"] if "file" in p]
            self.assertEqual(len(refs), 2)
            for ref in refs:
                self.assertTrue(ref.startswith("props/") and (Path(path4).parent / ref).is_file(), ref)
            self.assertNotEqual(refs[0], refs[1])
            # two models of one file name from two folders: the second is kept as name-2
            for k, body in enumerate(["one", "two"]):
                d = Path(tmp) / ("elsewhere%d" % k)
                d.mkdir()
                (d / "same.assembly.json").write_text(body)
            m, _ = props.with_model_added(_TWO, "a", str(Path(tmp) / "elsewhere0" / "same.assembly.json"), 0, 0, 0)
            m, _ = props.with_model_added(m, "b", str(Path(tmp) / "elsewhere1" / "same.assembly.json"), 0, 0, 0)
            _, path6 = props.save_as(src, m, "same names", env=env)
            files6 = [p["file"] for p in mapfile.load(path6)["props"] if "file" in p]
            self.assertEqual(files6, ["props/same.assembly.json", "props/same-2.assembly.json"])
            self.assertEqual((Path(path6).parent / "props" / "same-2.assembly.json").read_text(), "two")
            with self.assertRaises(props.PropError):
                props.save_as(src, props.with_model_added(_TWO, "lost", "/no/such/model.assembly.json", 0, 0, 0)[0], "lost", env=env)

    def test_an_old_map_of_the_users_own_becomes_json_when_listed(self):
        # a map saved before 4.32.0 is a world.xml with placeholders; it is listed (and loads) as
        # the same map in JSON, the old file left where it was
        with tempfile.TemporaryDirectory() as tmp:
            env = {"OPENBRICKS_DATA_DIR": tmp}
            old = Path(tmp) / "worlds" / "old-layout"
            old.mkdir(parents=True)
            (old / "world.xml").write_text(
                '<!-- my layout -->\n<mujoco model="old">\n  <compiler angle="degree" coordinate="local"/>\n'
                '  <worldbody>\n    <!-- the floor -->\n    <geom name="floor" type="plane" size="1 1 0.01"/>\n'
                '    <lego_prop name="clef" ldr="props/clef.ldr" pos="0.1 0.2 0.005" mass="0.05" yaw="30" color="red"/>\n'
                '    <assembly_prop name="t" file="props/t.assembly.json" pos="0 0 0" pitch="90" fixed="true"/>\n'
                '  </worldbody>\n</mujoco>\n')
            listed = props.list_user_worlds(env)
            self.assertEqual([w["alias"] for w in listed], ["old-layout"])
            self.assertEqual(Path(listed[0]["path"]).name, "map.json")
            self.assertTrue((old / "world.xml").is_file(), "the old file is left where it was")
            m = mapfile.load(listed[0]["path"])
            self.assertEqual(m["about"], ["my layout"])
            self.assertEqual(m["geoms"], [{"name": "floor", "type": "plane", "size": [1, 1, 0.01], "note": "the floor"}])
            self.assertEqual(m["props"], [
                {"name": "clef", "ldr": "props/clef.ldr", "pos": [0.1, 0.2, 0.005], "mass": 0.05, "yaw": 30.0, "color": "red"},
                {"name": "t", "file": "props/t.assembly.json", "pos": [0.0, 0.0, 0.0], "pitch": 90.0, "fixed": True},
            ])
            # listed again: the JSON is read, not converted again
            (old / "world.xml").write_text("not xml any more")
            self.assertEqual(len(props.list_user_worlds(env)), 1)
            # a folder with neither is not a map, nor is a stray file beside the maps
            (Path(tmp) / "worlds" / "junk").mkdir()
            (Path(tmp) / "worlds" / ".DS_Store").write_text("")
            self.assertEqual(len(props.list_user_worlds(env)), 1)


class ExportImportTests(unittest.TestCase):
    def test_a_map_goes_out_as_one_json_file_and_comes_back_as_a_map_of_ones_own(self):
        with tempfile.TemporaryDirectory() as tmp:
            env = {"OPENBRICKS_DATA_DIR": str(Path(tmp) / "data")}
            src = SaveTests._src(self, tmp)
            (src / "props" / "note.ldr").write_text("1 4 0 0 0 1 0 0 0 1 0 0 0 1 3003.dat")
            staged = props.stage_file('{"format": "openbricks-assembly/1"}', "tower", "assembly.json", env=env)
            m, _ = props.with_model_added(_TWO, "tower", staged, 0.1, 0.1, 0.0)
            out = Path(tmp) / "layout.map.json"
            self.assertEqual(props.export_map(src, m, out), str(out))
            obj = json.loads(out.read_text())
            self.assertEqual(obj["format"], mapfile.FORMAT)
            self.assertEqual(sorted(obj["files"]), ["README.md", "mat.png", "props/clef.ldr", "props/note.ldr", "props/" + Path(staged).name])
            self.assertEqual(obj["files"]["README.md"], {"text": "notes"})
            self.assertEqual(obj["files"]["props/" + Path(staged).name], {"json": {"format": "openbricks-assembly/1"}})
            self.assertEqual(list(obj["files"]["mat.png"]), ["base64"])
            self.assertEqual(obj["props"][2]["file"], "props/" + Path(staged).name, "the added model is named from inside")
            self.assertNotIn(tmp, json.dumps({k: v for k, v in obj.items() if k != "files"}))
            # imported: a map of one's own under its name, every file back as it was
            alias, path = props.import_map(out, env=env)
            self.assertEqual(alias, "two")
            dest = Path(path).parent
            self.assertEqual((dest / "mat.png").read_bytes(), b"png")
            self.assertEqual((dest / "props" / "clef.ldr").read_text(), "1 4 0 0 0 1 0 0 0 1 0 0 0 1 3001.dat")
            self.assertEqual(json.loads((dest / "props" / Path(staged).name).read_text()), {"format": "openbricks-assembly/1"})
            self.assertEqual(mapfile.load(path)["props"][2]["file"], "props/" + Path(staged).name)
            self.assertEqual([w["alias"] for w in props.list_user_worlds(env)], ["two"])
            # imported again, or under a shipped map's name: the next free alias
            self.assertEqual(props.import_map(out, env=env)[0], "two-2")
            self.assertEqual(props.import_map(out, reserved=("two",), env=env)[0], "two-3")
            # not an export, or one naming a file outside its folder: refused, nothing left behind
            (Path(tmp) / "nope.json").write_text("{}")
            with self.assertRaises(props.PropError):
                props.import_map(Path(tmp) / "nope.json", env=env)
            evil = dict(obj, name="evil", files={"../escape.txt": {"text": "x"}})
            (Path(tmp) / "evil.json").write_text(json.dumps(evil))
            with self.assertRaises(props.PropError):
                props.import_map(Path(tmp) / "evil.json", env=env)
            self.assertFalse((Path(tmp) / "data" / "worlds" / "escape.txt").exists())
            self.assertFalse((Path(tmp) / "data" / "worlds" / "evil").exists())
            # a model gone before the export: refused by name
            with self.assertRaises(props.PropError):
                props.export_map(src, props.with_model_added(_TWO, "lost", "/no/such.assembly.json", 0, 0, 0)[0], out)


    def test_a_document_becomes_a_prop_body_free_or_stuck(self):
        from openbricks_sim import assembly, bricks
        from openbricks_sim.world import load_world, WorldLoadError
        from openbricks_sim.chassis import ChassisSpec
        bundle = bricks.load_bundle()
        num = sorted(bundle["parts"])[0]
        rec = bundle["parts"][num]
        doc = {"format": "openbricks-assembly/1", "units": {"length": "mm"},
               "parts": {"p": {"name": rec["name"], "category": "lego", "mass_g": rec["mass_g"], "ldraw": num}},
               "components": {"pair": {"children": [{"name": "a", "part": "p", "pos": [0, 0, 0], "rot": [0, 0, 0]},
                                                    {"name": "b", "part": "p", "pos": [40, 0, 0], "rot": [0, 0, 90]}]}},
               "robot": {"name": "pair", "root": "pair"}}
        doc["components"]["pair"]["children"][0]["color"] = 72   # placed in dark bluish gray (4.21.0)
        out, total = assembly.prop_bricks(doc, bundle)
        self.assertEqual([b["path"] for b in out], ["a", "b"])
        self.assertEqual([b["color"] for b in out], [72, None], "the LEGO colour rides with each brick")
        self.assertAlmostEqual(total, 2 * rec["mass_g"], places=6)
        self.assertEqual(out[1]["ldraw"], num)
        self.assertAlmostEqual(out[1]["mass_g"], rec["mass_g"], places=3)
        with self.assertRaises(assembly.AssemblyError):
            assembly.prop_bricks({"format": "nope"}, bundle)
        with self.assertRaises(assembly.AssemblyError):
            assembly.prop_bricks(dict(doc, robot={"root": "missing"}), bundle)
        with self.assertRaises(assembly.AssemblyError):
            assembly.prop_bricks(dict(doc, components={"pair": {"children": []}}), bundle)
        free = assembly.prop_body_xml("pair", (0.1, 0.2, 0.0), 90.0, False, out)
        self.assertIn("<freejoint/>", free)
        self.assertIn('quat="0.707107 0.000000 0.000000 0.707107"', free)
        # pitched a quarter about y, and rolled a quarter about x
        self.assertIn('quat="0.707107 0.000000 0.707107 0.000000"',
                      assembly.prop_body_xml("pair", (0.1, 0.2, 0.0), 0.0, False, out, pitch_deg=90.0).splitlines()[0])
        self.assertIn('quat="0.707107 0.707107 0.000000 0.000000"',
                      assembly.prop_body_xml("pair", (0.1, 0.2, 0.0), 0.0, False, out, roll_deg=90.0).splitlines()[0])
        self.assertEqual(free.count("<geom "), 2)
        stuck = assembly.prop_body_xml("pair", (0.1, 0.2, 0.0), 0.0, True, out)
        self.assertNotIn("<freejoint/>", stuck)
        self.assertNotIn("quat=", stuck.splitlines()[0])
        # in a map: the prop becomes a body, loads, and a stuck one has no joint
        with tempfile.TemporaryDirectory() as tmp:
            world = Path(tmp) / "w"
            (world / "props").mkdir(parents=True)
            (world / "props" / "pair.assembly.json").write_text(json.dumps(doc))
            m = {"format": mapfile.FORMAT, "name": "w", "geoms": [{"name": "floor", "type": "plane", "size": [1, 1, 0.01]}],
                 "props": [{"name": "pair", "file": "props/pair.assembly.json", "pos": [0.1, 0.2, 0.02]},
                           {"name": "post", "file": str(world / "props" / "pair.assembly.json"), "pos": [-0.2, 0, 0.02],
                            "yaw": 45, "fixed": True}]}
            mapfile.save(m, world / "map.json")
            md, d, merged = load_world(str(world / "map.json"), chassis_spec=ChassisSpec())
            self.assertIn('name="pair_brick:a"', merged)
            pair = mujoco.mj_name2id(md, mujoco.mjtObj.mjOBJ_BODY, "pair")
            post = mujoco.mj_name2id(md, mujoco.mjtObj.mjOBJ_BODY, "post")
            self.assertEqual(int(md.body_jntnum[pair]), 1, "free")
            self.assertEqual(int(md.body_jntnum[post]), 0, "stuck")
            self.assertGreater(float(md.body_mass[pair]), 0.0)
            mujoco.mj_forward(md, d)
            self.assertAlmostEqual(float(d.xpos[post][0]), -0.2, places=5)
            self.assertAlmostEqual(float(d.xpos[post][2]), 0.02, places=5, msg="a prop standing above the floor stays put")
            # a prop the map put under the floor (a build placed at 0 with bricks below its
            # origin) is lifted onto it: never under the map
            sunk = copy.deepcopy(m)
            sunk["props"][0]["pos"] = [0.1, 0.2, 0.0]
            m2, d2, _ = load_world(str(world / "map.json"), chassis_spec=ChassisSpec(), world_map=sunk)
            mujoco.mj_forward(m2, d2)
            pair2 = mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_BODY, "pair")
            self.assertAlmostEqual(float(d2.xpos[pair2][2]), -assembly.prop_lowest_m(out), places=5)
            self.assertGreater(float(d2.xpos[pair2][2]), 0.0)
            # tipped on its side, it is lifted by what reaches down turned: its 16 mm width
            tipped = copy.deepcopy(sunk)
            tipped["props"][0]["pitch"] = 90
            m3, d3, _ = load_world(str(world / "map.json"), chassis_spec=ChassisSpec(), world_map=tipped)
            mujoco.mj_forward(m3, d3)
            pair3 = mujoco.mj_name2id(m3, mujoco.mjtObj.mjOBJ_BODY, "pair")
            lift = -assembly.prop_lowest_m(out, props.euler_quat(0.0, 90.0, 0.0))
            self.assertAlmostEqual(float(d3.xpos[pair3][2]), lift, places=5)
            self.assertNotAlmostEqual(lift, -assembly.prop_lowest_m(out), places=3)
            # the lowest point of every brick box, turned as MuJoCo turned it, is on the floor
            lows = []
            for g in range(m3.ngeom):
                if int(m3.geom_bodyid[g]) != pair3:
                    continue
                r = d3.geom_xmat[g].reshape(3, 3)
                lows.append(float(d3.geom_xpos[g][2]) - sum(abs(r[2][k]) * float(m3.geom_size[g][k]) for k in range(3)))
            self.assertAlmostEqual(min(lows), 0.0, places=5)
            gone = copy.deepcopy(m)
            gone["props"][0]["file"] = "props/gone.assembly.json"
            with self.assertRaises(WorldLoadError):
                load_world(str(world / "map.json"), chassis_spec=ChassisSpec(), world_map=gone)
            broken = copy.deepcopy(m)
            (world / "props" / "bad.assembly.json").write_text('{"format": "nope"}')
            broken["props"][0]["file"] = "props/bad.assembly.json"
            with self.assertRaises(WorldLoadError):
                load_world(str(world / "map.json"), chassis_spec=ChassisSpec(), world_map=broken)

    @unittest.skipIf(mujoco is None, "mujoco not installed")
    def test_a_saved_shipped_map_loads_with_its_props_where_they_were_put(self):
        from openbricks_sim import robot as robot_mod
        from openbricks_sim.world import load_world
        from openbricks_sim.chassis import ChassisSpec
        with tempfile.TemporaryDirectory() as tmp, mock.patch.dict(os.environ, {"OPENBRICKS_DATA_DIR": tmp}):
            m = mapfile.load(_ELEMENTARY)
            m = props.with_prop_moved(m, "clef", 0.3, -0.2, 45.0)
            m, name = props.with_prop_added(m, "clef", -0.3, 0.2, 0.0)
            alias, path = props.save_as(_ELEMENTARY.parent, m, "clefs", reserved=robot_mod._BUILTIN_WORLDS)
            self.assertEqual(robot_mod._resolve_world("clefs"), path, "the user's map resolves by alias")
            self.assertIsNone(robot_mod._resolve_world("empty"))
            md, d, _ = load_world(path, chassis_spec=ChassisSpec())
            mujoco.mj_forward(md, d)
            clef = mujoco.mj_name2id(md, mujoco.mjtObj.mjOBJ_BODY, "clef")
            self.assertAlmostEqual(float(d.xpos[clef][0]), 0.3, places=4)
            self.assertAlmostEqual(float(d.xpos[clef][1]), -0.2, places=4)
            w, x, y, z = (float(v) for v in d.xquat[clef])
            self.assertAlmostEqual(w, 0.9238795, places=5)
            self.assertAlmostEqual(z, 0.3826834, places=5)
            second = mujoco.mj_name2id(md, mujoco.mjtObj.mjOBJ_BODY, name)
            self.assertGreaterEqual(second, 0, "the added clef is a body of its own")
            self.assertAlmostEqual(float(d.xpos[second][0]), -0.3, places=4)


if __name__ == "__main__":
    unittest.main()
