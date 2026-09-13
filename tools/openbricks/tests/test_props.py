# SPDX-License-Identifier: MIT
"""The map editor's text side: props found, moved, added and removed in
a world's MJCF, and maps saved as the user's own where the server
lists them."""
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from openbricks_sim import props

try:
    import mujoco
except ImportError:                      # pragma: no cover
    mujoco = None

_WORLDS = Path(__file__).resolve().parents[1] / "openbricks_sim" / "worlds"
_ELEMENTARY = _WORLDS / "wro_2026_elementary_robot_rockstars" / "world.xml"

_TWO = """<mujoco model="two">
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.01"/>
    <lego_prop name="clef" ldr="props/clef.ldr" pos="0.1 0.2 0.005" mass="0.05"/>
    <lego_prop name="note_red" ldr="props/note.ldr" pos="-0.3 0 0.005" mass="0.02" yaw="30" color="red"/>
  </worldbody>
</mujoco>
"""


class DataDirTests(unittest.TestCase):
    def test_the_data_directory_follows_the_markers_rule(self):
        self.assertEqual(props.data_dir({"OPENBRICKS_DATA_DIR": "/x/data"}, home="/h"), Path("/x/data"))
        self.assertEqual(props.data_dir({"XDG_DATA_HOME": "/xdg"}, home="/h"), Path("/xdg/openbricks"))
        self.assertEqual(props.data_dir({}, home="/h"), Path("/h/.local/share/openbricks"))
        self.assertEqual(props.user_worlds_dir({}, home="/h"), Path("/h/.local/share/openbricks/worlds"))
        self.assertEqual(props.list_user_worlds({"OPENBRICKS_DATA_DIR": "/does/not/exist"}), [])


class PropTextTests(unittest.TestCase):
    def test_props_are_read_with_their_pose_colour_and_yaw(self):
        found = props.props_in(_TWO)
        self.assertEqual([p["name"] for p in found], ["clef", "note_red"])
        self.assertEqual(found[0]["pos"], (0.1, 0.2, 0.005))
        self.assertEqual((found[0]["yaw"], found[0]["color"], found[0]["mass"]), (0.0, None, 0.05))
        self.assertEqual((found[1]["yaw"], found[1]["color"], found[1]["ldr"]), (30.0, "red", "props/note.ldr"))
        self.assertEqual(props.props_in("<mujoco/>"), [])
        with self.assertRaises(props.PropError):
            props.props_in('<lego_prop name="x" ldr="a.ldr" pos="1 2" mass="1"/>')

    def test_the_shipped_elementary_map_has_its_props(self):
        names = [p["name"] for p in props.props_in(_ELEMENTARY.read_text())]
        self.assertIn("clef", names)
        self.assertGreater(len(names), 10)

    def test_a_prop_moves_keeping_its_height_and_the_rest_of_the_text(self):
        moved = props.with_prop_moved(_TWO, "clef", 0.5, -0.25, 90.0)
        p = props.props_in(moved)[0]
        self.assertEqual((p["pos"], p["yaw"]), ((0.5, -0.25, 0.005), 90.0))
        self.assertIn('<lego_prop name="clef" ldr="props/clef.ldr" pos="0.50000 -0.25000 0.00500" mass="0.05" yaw="90"/>', moved)
        self.assertIn('color="red"', moved, "the other prop is untouched")
        self.assertEqual(moved.count("<lego_prop"), 2)
        # back to no turn: the yaw attribute goes again
        self.assertNotIn("yaw=", props.with_prop_moved(moved, "clef", 0.5, -0.25, 0.0).split("note_red")[0])
        with self.assertRaises(props.PropError):
            props.with_prop_moved(_TWO, "nothing", 0.0, 0.0, 0.0)

    def test_another_prop_like_one_is_added_after_it_with_a_free_name(self):
        text, name = props.with_prop_added(_TWO, "note_red", 0.0, 0.1, 45.0)
        self.assertEqual(name, "note_red_2")
        found = props.props_in(text)
        self.assertEqual([p["name"] for p in found], ["clef", "note_red", "note_red_2"])
        new = found[2]
        self.assertEqual((new["pos"], new["yaw"], new["color"], new["ldr"], new["mass"]), ((0.0, 0.1, 0.005), 45.0, "red", "props/note.ldr", 0.02))
        self.assertIn('\n    <lego_prop name="note_red_2"', text, "on its own line, indented like the original")
        text, name = props.with_prop_added(text, "note_red_2", 0.2, 0.2, 0.0)
        self.assertEqual(name, "note_red_3", "numbering continues from the family")
        self.assertEqual(props.unique_name(text, "fresh"), "fresh")
        with self.assertRaises(props.PropError):
            props.with_prop_added(_TWO, "nothing", 0.0, 0.0, 0.0)

    def test_a_prop_is_removed_with_its_line(self):
        text = props.with_prop_removed(_TWO, "clef")
        self.assertEqual([p["name"] for p in props.props_in(text)], ["note_red"])
        self.assertNotIn("\n\n", text.split("<worldbody>")[1].split("</worldbody>")[0].strip("\n"), "no blank line left behind")
        self.assertEqual(text.count("\n"), _TWO.count("\n") - 1)
        with self.assertRaises(props.PropError):
            props.with_prop_removed(_TWO, "nothing")

    def test_names_become_directory_slugs(self):
        self.assertEqual(props.slug("My Layout 2 "), "my-layout-2")
        self.assertEqual(props.slug("WRO/elementary: notes!"), "wro-elementary-notes")
        with self.assertRaises(props.PropError):
            props.slug("  ")


class SaveTests(unittest.TestCase):
    def test_a_map_is_saved_with_its_files_listed_and_replaced_by_name(self):
        with tempfile.TemporaryDirectory() as tmp:
            env = {"OPENBRICKS_DATA_DIR": tmp}
            src = Path(tmp) / "src"
            (src / "props").mkdir(parents=True)
            (src / "world.xml").write_text(_TWO)
            (src / "mat.png").write_bytes(b"png")
            (src / "props" / "clef.ldr").write_text("1 4 0 0 0 1 0 0 0 1 0 0 0 1 3001.dat")
            (src / "README.md").write_text("notes")
            alias, path = props.save_as(src, props.with_prop_moved(_TWO, "clef", 1.0, 1.0, 0.0), "My Layout", env=env)
            self.assertEqual(alias, "my-layout")
            self.assertEqual(Path(path), Path(tmp) / "worlds" / "my-layout" / "world.xml")
            self.assertEqual(sorted(p.name for p in Path(path).parent.iterdir()), ["README.md", "mat.png", "props", "world.xml"])
            self.assertEqual((Path(path).parent / "props" / "clef.ldr").read_text(), "1 4 0 0 0 1 0 0 0 1 0 0 0 1 3001.dat")
            self.assertIn('pos="1.00000 1.00000 0.00500"', Path(path).read_text())
            listed = props.list_user_worlds(env)
            self.assertEqual(listed, [{"alias": "my-layout", "path": path, "dir": str(Path(path).parent), "user": True}])
            # saving again under the same name replaces the map
            alias2, path2 = props.save_as(src, _TWO, "my layout", env=env)
            self.assertEqual((alias2, path2), (alias, path))
            self.assertIn('pos="0.1 0.2 0.005"', Path(path).read_text(), "the original text, as given")
            self.assertEqual(len(props.list_user_worlds(env)), 1)
            # a shipped alias is never shadowed; a nameless map is refused
            with self.assertRaises(props.PropError):
                props.save_as(src, _TWO, "practice line", reserved=("practice-line",), env=env)
            with self.assertRaises(props.PropError):
                props.save_as(src, _TWO, "!!!", env=env)
            # a map without a source directory (the text alone) still saves
            alias3, path3 = props.save_as(None, _TWO, "bare", env=env)
            self.assertEqual(sorted(p.name for p in Path(path3).parent.iterdir()), ["world.xml"])

    @unittest.skipIf(mujoco is None, "mujoco not installed")
    def test_a_saved_shipped_map_loads_with_its_props_where_they_were_put(self):
        from openbricks_sim import robot as robot_mod
        from openbricks_sim.world import load_world
        from openbricks_sim.chassis import ChassisSpec
        with tempfile.TemporaryDirectory() as tmp, mock.patch.dict(os.environ, {"OPENBRICKS_DATA_DIR": tmp}):
            text = _ELEMENTARY.read_text()
            text = props.with_prop_moved(text, "clef", 0.3, -0.2, 45.0)
            text, name = props.with_prop_added(text, "clef", -0.3, 0.2, 0.0)
            alias, path = props.save_as(_ELEMENTARY.parent, text, "clefs", reserved=robot_mod._BUILTIN_WORLDS)
            self.assertEqual(robot_mod._resolve_world("clefs"), path, "the user's map resolves by alias")
            self.assertIsNone(robot_mod._resolve_world("empty"))
            m, d, _ = load_world(path, chassis_spec=ChassisSpec())
            mujoco.mj_forward(m, d)
            clef = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "clef")
            self.assertAlmostEqual(float(d.xpos[clef][0]), 0.3, places=4)
            self.assertAlmostEqual(float(d.xpos[clef][1]), -0.2, places=4)
            w, x, y, z = (float(v) for v in d.xquat[clef])
            self.assertAlmostEqual(w, 0.9238795, places=5)
            self.assertAlmostEqual(z, 0.3826834, places=5)
            second = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
            self.assertGreaterEqual(second, 0, "the added clef is a body of its own")
            self.assertAlmostEqual(float(d.xpos[second][0]), -0.3, places=4)


if __name__ == "__main__":
    unittest.main()
