# SPDX-License-Identifier: MIT
"""Tests for world loading + chassis injection."""

import os
import tempfile
import unittest
from pathlib import Path

import mujoco

from openbricks_sim.chassis import ChassisSpec
from openbricks_sim import mapfile
from openbricks_sim.world import WorldLoadError, load_world


# Shipped worlds live INSIDE the ``openbricks_sim`` package as of
# 0.10.6 (so they land in the wheel via package-data — see
# pyproject.toml). Locate them via the package, not the test file:
# importlib.resources keeps the test correct under both editable
# installs and built wheels.
import openbricks_sim
_WORLDS = Path(openbricks_sim.__file__).resolve().parent / "worlds"

_BUILTIN_WORLDS = [
    "wro_2026_elementary_robot_rockstars",
    "wro_2026_junior_heritage_heroes",
    "wro_2026_senior_mosaic_masters",
    "practice_zones",
    "practice_walls",
]


class LoadWorldTests(unittest.TestCase):

    def test_missing_world_raises(self):
        with self.assertRaises(WorldLoadError):
            load_world("/tmp/does-not-exist/map.json")

    def test_an_mjcf_map_is_refused_by_name(self):
        # maps are JSON: an old world.xml path is told where the map is now
        with self.assertRaises(WorldLoadError) as cm:
            load_world(str(_WORLDS / "practice_line" / "world.xml"))
        self.assertIn("maps are JSON", str(cm.exception))
        self.assertIn("map.json", str(cm.exception))
        # a map.json that is no map, and one whose prop's model is gone, are refused too
        with tempfile.TemporaryDirectory() as tmp:
            bad = Path(tmp) / "map.json"
            bad.write_text('{"format": "nope"}')
            with self.assertRaises(WorldLoadError):
                load_world(str(bad))
            with self.assertRaises(WorldLoadError):
                load_world(str(bad), world_map={"format": mapfile.FORMAT,
                                                "props": [{"name": "x", "ldr": "no.ldr", "pos": [0, 0, 0], "mass": 0.01}]})

    def test_no_shipped_map_is_xml(self):
        # the user's rule: maps are JSON everywhere — no MJCF ships as a map
        for name in _BUILTIN_WORLDS + ["practice_line"]:
            with self.subTest(world=name):
                self.assertTrue((_WORLDS / name / "map.json").is_file())
                self.assertEqual(sorted(p.name for p in (_WORLDS / name).rglob("*.xml")), [])

    def test_each_shipped_world_loads_with_chassis(self):
        for name in _BUILTIN_WORLDS:
            path = str(_WORLDS / name / "map.json")
            with self.subTest(world=name):
                m, d, merged = load_world(path,
                                          chassis_spec=ChassisSpec(pos_x=1.0,
                                                                   pos_y=-0.42))
                # Chassis + 2 wheels + caster = 4 extra bodies on top
                # of whatever the world had.
                cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")
                self.assertGreaterEqual(cid, 1)
                # It should start at the requested spawn position.
                for _ in range(100):  # small settle so qpos is sane
                    mujoco.mj_step(m, d)
                self.assertAlmostEqual(d.xpos[cid, 0], 1.0, delta=0.02)
                self.assertAlmostEqual(d.xpos[cid, 1], -0.42, delta=0.02)
                # Chassis actuators are exposed.
                self.assertEqual(m.nu, 2)

    def test_chassis_can_drive_in_world(self):
        """Stepping with ctrl ≠ 0 advances the chassis."""
        path = str(_WORLDS / "wro_2026_elementary_robot_rockstars" / "map.json")
        m, d, _ = load_world(path,
                             chassis_spec=ChassisSpec(pos_x=1.0, pos_y=-0.42))
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        act_l = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "chassis_motor_l")
        act_r = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "chassis_motor_r")
        # Settle briefly.
        for _ in range(200):
            mujoco.mj_step(m, d)
        x0, y0 = float(d.xpos[cid, 0]), float(d.xpos[cid, 1])
        # Drive forward for 2 s.
        d.ctrl[act_l] = 0.3
        d.ctrl[act_r] = 0.3
        for _ in range(2000):
            mujoco.mj_step(m, d)
        x1, y1 = float(d.xpos[cid, 0]), float(d.xpos[cid, 1])
        # +ctrl on both wheels drives the chassis "forward" for this
        # hinge-axis orientation. The heading it settles into isn't
        # axis-aligned (it drives mostly along -Y here), so assert on
        # total planar displacement rather than any single world axis —
        # a per-axis projection is small and platform-numerics-sensitive
        # (it flaked on CI at |dx|=0.0018 while the chassis had actually
        # travelled ~0.13 m). Distance is orientation-independent and
        # clears the threshold by ~6x.
        dist = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        self.assertGreater(dist, 0.02,
                           "motor ctrl should move the chassis (moved "
                           "%.4f m: (%.3f,%.3f) -> (%.3f,%.3f))"
                           % (dist, x0, y0, x1, y1))


class LegoPropExpansionTests(unittest.TestCase):
    """A map's LDraw prop (``{"name", "ldr", "pos", "mass"}``) is made
    into a full ``<body>`` as the model is made (:func:`mapfile.to_mjcf`),
    reading the .ldr at load time. These tests pin that pipeline."""

    def _map(self, *props_):
        return {"format": mapfile.FORMAT, "name": "t", "props": list(props_)}

    def test_placeholder_replaced_with_body_geoms(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            (tmp / "props").mkdir()
            (tmp / "props" / "tiny.ldr").write_text(
                "0 Tiny test prop\n"
                "1 1 0 0 0 1 0 0 0 1 0 0 0 1 3003.dat\n")
            expanded = mapfile.to_mjcf(self._map({"name": "tinyp", "ldr": "props/tiny.ldr", "pos": [0, 0, 0], "mass": 0.005}), tmp)
            self.assertIn('<body name="tinyp"', expanded)
            self.assertIn('<freejoint/>', expanded)
            self.assertIn('material="lego_blue"', expanded)
            # 1 brick body + 4 stud cylinders (2x2 brick)
            self.assertEqual(expanded.count('<geom type="box"'), 1)
            self.assertEqual(expanded.count('<geom type="cylinder"'), 4)
            # a colour keyword or an LDraw code overrides the bricks'; anything else is refused
            red = mapfile.to_mjcf(self._map({"name": "r", "ldr": "props/tiny.ldr", "pos": [0, 0, 0], "mass": 0.005, "color": "red"}), tmp)
            self.assertIn('material="lego_red"', red)
            code = mapfile.to_mjcf(self._map({"name": "r", "ldr": "props/tiny.ldr", "pos": [0, 0, 0], "mass": 0.005, "color": "14"}), tmp)
            self.assertIn('material="lego_yellow"', code)
            with self.assertRaises(mapfile.MapError):
                mapfile.to_mjcf(self._map({"name": "r", "ldr": "props/tiny.ldr", "pos": [0, 0, 0], "mass": 0.005, "color": "mauve"}), tmp)
            # stuck: no free joint
            stuck = mapfile.to_mjcf(self._map({"name": "s", "ldr": "props/tiny.ldr", "pos": [0, 0, 0], "mass": 0.005, "fixed": True}), tmp)
            self.assertNotIn("<freejoint/>", stuck)

    def test_missing_ldr_raises_loadly(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(mapfile.MapError) as cm:
                mapfile.to_mjcf(self._map({"name": "x", "ldr": "props/no_such.ldr", "pos": [0, 0, 0], "mass": 0.01}), Path(tmp))
            self.assertIn("missing .ldr", str(cm.exception))

    def test_a_map_with_no_props_has_no_bodies(self):
        with tempfile.TemporaryDirectory() as tmp:
            m = {"format": mapfile.FORMAT, "name": "bare", "geoms": [{"type": "plane", "size": [1, 1, 0.1]}]}
            text = mapfile.to_mjcf(m, Path(tmp))
            self.assertIn('<geom type="plane" size="1 1 0.1"/>', text)
            self.assertNotIn("<body", text)

    def test_senior_barriers_have_dual_color_scheme(self):
        # F4.2 split the single ``senior_barrier.ldr`` (which used
        # the lego_prop ``color`` override and so forced every brick
        # — body + ball — to one colour) into two files with
        # per-brick LDraw colours, restoring the rules-PDF photo's
        # red-body+blue-ball / black-body+red-ball contrast. Pin
        # that the loaded model actually has multiple distinct
        # materials per barrier body — if a future refactor goes
        # back to a single colour override, this test catches it.
        path = (_WORLDS / "wro_2026_senior_mosaic_masters" / "map.json")
        m, _, _ = load_world(str(path), chassis_spec=ChassisSpec())

        def _materials_under(body_name):
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
            self.assertGreaterEqual(bid, 0, "%s body missing" % body_name)
            mats = set()
            for gi in range(m.ngeom):
                if int(m.geom_bodyid[gi]) == bid:
                    mid = int(m.geom_matid[gi])
                    if mid >= 0:
                        mats.add(mid)
            return mats

        for barrier in ("barrier_red_a", "barrier_red_b",
                        "barrier_black_a", "barrier_black_b"):
            mats = _materials_under(barrier)
            self.assertGreaterEqual(
                len(mats), 2,
                "%s should reference >=2 materials (body + ball "
                "colours); got %d — did the dual-colour scheme "
                "regress to a single ``color`` override?" % (
                    barrier, len(mats)))

    def test_senior_world_has_mosaic_frame_mesh(self):
        # The Senior "Mosaic Masters" world uses the WRO-published
        # 3D-printed mosaic frame as a MuJoCo ``<mesh>`` (the only
        # non-LDraw geometry in the prop set). Pin that:
        #   * exactly one mesh is declared in <asset>
        #   * a static geom named ``mosaic_frame`` references it
        #   * the geom is welded to the worldbody (body 0)
        path = (_WORLDS / "wro_2026_senior_mosaic_masters" / "map.json")
        m, _, _ = load_world(str(path), chassis_spec=ChassisSpec())
        self.assertEqual(
            m.nmesh, 1,
            "Senior world should declare exactly 1 mesh (the mosaic "
            "frame); got %d" % m.nmesh)
        gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "mosaic_frame")
        self.assertGreaterEqual(
            gid, 0, "mosaic_frame geom missing from Senior world")
        # Static scenery → parent body is the worldbody (id 0).
        self.assertEqual(
            int(m.geom_bodyid[gid]), 0,
            "mosaic_frame should be welded to the worldbody (static "
            "scenery), not parented to a movable body")

    def test_junior_world_ldraw_prop_has_many_geoms(self):
        # Integration test: a shipped LDraw prop (the Junior yellow
        # tower, ``"ldr": "props/yellow_tower.ldr"``) expands into a
        # body of brick boxes and stud cylinders. Pre-F2.1 props were
        # single-box approximations — pin the multi-geom shape so a
        # regression to single-box is caught.
        path = (_WORLDS / "wro_2026_junior_heritage_heroes" / "map.json")
        m, _, _ = load_world(str(path), chassis_spec=ChassisSpec())
        tower = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "yellow_tower_a")
        self.assertGreaterEqual(tower, 0, "yellow_tower_a body missing from model")
        count = sum(1 for i in range(m.ngeom) if int(m.geom_bodyid[i]) == tower)
        self.assertGreater(
            count, 40,
            "the yellow tower should have >40 geoms (bricks + studs); "
            "got %d — has the LDraw expansion regressed to a "
            "single-box approximation?" % count)

    def test_a_builds_bricks_carry_their_colour_for_the_sensors(self):
        # a Workbench build placed as a prop: each brick's box carries
        # the palette colour the viewer draws it in (what a colour
        # sensor reads off it), a brick with none the Workbench's
        # default LEGO colour; a colour the palette lacks is refused,
        # naming the prop and the brick
        import json
        from openbricks_sim import assembly, bricks
        num = sorted(bricks.load_bundle()["parts"])[0]
        red = bricks.palette()["4"]["rgb"]
        self.assertEqual(red, "C91A09")

        def doc(*colours):
            children = []
            for i, c in enumerate(colours):
                child = {"name": "b%d" % i, "part": "p", "pos": [0, 40.0 * i, 0], "rot": [0, 0, 0]}
                if c is not None:
                    child["color"] = c
                children.append(child)
            return {"format": "openbricks-assembly/1",
                    "parts": {"p": {"name": "brick", "category": "lego", "mass_g": 2.0, "ldraw": num}},
                    "components": {"one": {"children": children}}, "robot": {"name": "One", "root": "one"}}

        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            (tmp / "props").mkdir()
            (tmp / "props" / "two.assembly.json").write_text(json.dumps(doc(4, None)))
            mjcf = mapfile.to_mjcf(self._map({"name": "two", "file": "props/two.assembly.json", "pos": [0, 0, 0]}), tmp)
            self.assertIn('name="two_brick:b0"', mjcf)
            b0 = mjcf[mjcf.index('name="two_brick:b0"'):].split("/>")[0]
            b1 = mjcf[mjcf.index('name="two_brick:b1"'):].split("/>")[0]
            self.assertIn('rgba="0.7882 0.102 0.0353 1"', b0)
            self.assertIn('rgba="0.357 0.478 0.612 1"', b1)
            self.assertEqual(assembly.brick_rgb(None), assembly.UNCOLOURED_RGB)
            (tmp / "props" / "odd.assembly.json").write_text(json.dumps(doc(4, 987654)))
            with self.assertRaises(mapfile.MapError) as cm:
                mapfile.to_mjcf(self._map({"name": "odd", "file": "props/odd.assembly.json", "pos": [0, 0, 0]}), tmp)
            self.assertIn("odd", str(cm.exception))
            self.assertIn("b1", str(cm.exception))
            self.assertIn("987654", str(cm.exception))

    def test_elementary_world_props_are_workbench_builds_standing_apart_on_the_mat(self):
        # every prop of the Elementary map is a Workbench build of the
        # building instructions, shipped with the map: each stands on
        # the mat, none overlaps another, and a few seconds of physics
        # leave them where the map put them
        import numpy as np
        world = _WORLDS / "wro_2026_elementary_robot_rockstars"
        m_doc = mapfile.load(world / "map.json")
        names = [p["name"] for p in m_doc["props"]]
        self.assertEqual(sorted(names), sorted([
            "microphone", "keyboard", "guitar", "congas",
            "black_note", "blue_note", "red_note", "green_note", "white_note", "yellow_note",
            "cable", "cable_2", "clef", "speaker", "speaker_2", "amplifier"]))
        self.assertTrue(all("file" in p and "ldr" not in p for p in m_doc["props"]), m_doc["props"])
        on_disk = sorted("props/" + f.name for f in (world / "props").iterdir())
        self.assertEqual(on_disk, sorted({p["file"] for p in m_doc["props"]}), "the folder holds what the map names")
        m, d, _ = load_world(str(world / "map.json"), chassis_spec=ChassisSpec())
        mujoco.mj_forward(m, d)
        ids = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n): n for n in names}
        self.assertNotIn(-1, ids)
        for b, n in ids.items():
            low = min(float(d.geom_xpos[g][2] + (d.geom_xmat[g].reshape(3, 3) @ (m.geom_aabb[g, :3] + m.geom_aabb[g, 3:] * s))[2])
                      for g in range(m.ngeom) if int(m.geom_bodyid[g]) == b
                      for s in np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3))
            self.assertAlmostEqual(low, 0.0, places=4, msg="%s stands on the mat" % n)
        # each note is mostly its own colour, as a colour sensor reads it
        from collections import Counter
        from openbricks_sim import assembly
        for colour, code in (("black", 0), ("blue", 1), ("green", 2), ("red", 4), ("yellow", 14), ("white", 15)):
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, colour + "_note")
            most = Counter(tuple(round(float(v), 3) for v in m.geom_rgba[g][:3])
                           for g in range(m.ngeom) if int(m.geom_bodyid[g]) == b).most_common(1)[0][0]
            want = assembly.brick_rgb(code)
            self.assertLess(max(abs(a - b) for a, b in zip(most, want)), 0.002, (colour, most, want))
        touching = [(ids[int(m.geom_bodyid[c.geom1])], ids[int(m.geom_bodyid[c.geom2])])
                    for c in d.contact[:d.ncon]
                    if int(m.geom_bodyid[c.geom1]) in ids and int(m.geom_bodyid[c.geom2]) in ids
                    and int(m.geom_bodyid[c.geom1]) != int(m.geom_bodyid[c.geom2])]
        self.assertEqual(touching, [], "no prop starts in another")
        # the amplifier's body (96 x 48 mm, its knobs and light out front) on the black
        # outline the mat prints between the two grey cable areas, its front to the field
        from openbricks_sim import randomization
        amp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "amplifier")
        x, y = randomization.footprint_middle(m, d, "amplifier")
        self.assertAlmostEqual(y, 0.1717, places=3)
        self.assertAlmostEqual(x - 0.0056, -1.1200, places=3, msg="the body's middle, the front's 11 mm beyond it")
        self.assertAlmostEqual(float(d.xquat[amp][0]), float(d.xquat[amp][3]), places=6, msg="a quarter turn")
        start = {n: d.xpos[b].copy() for b, n in ids.items()}
        for _ in range(int(2.0 / m.opt.timestep)):
            mujoco.mj_step(m, d)
        for b, n in ids.items():
            moved = float(np.linalg.norm(d.xpos[b] - start[n]))
            self.assertLess(moved, 0.0005, "%s moved %.2f mm" % (n, moved * 1000))

    def test_elementary_world_microphone_is_a_workbench_build_standing_on_the_mat(self):
        # the microphone is a Workbench build shipped with the map (41 bricks with their
        # catalogue masses), placed where the mission's mat has it, standing on the floor:
        # a brick's origin is its top face, so the prop is lifted by the build's lowest point
        world = _WORLDS / "wro_2026_elementary_robot_rockstars"
        self.assertTrue((world / "props" / "microphone.assembly.json").is_file())
        self.assertFalse((world / "props" / "microphone.ldr").exists(), "the LDraw stand-in is gone")
        pyproject = Path(openbricks_sim.__file__).resolve().parents[1] / "pyproject.toml"
        if pyproject.is_file():
            self.assertIn('"worlds/*/props/*.assembly.json"', pyproject.read_text(), "shipped in the wheel")
        m, d, _ = load_world(str(world / "map.json"), chassis_spec=ChassisSpec())
        mic = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "microphone")
        self.assertGreaterEqual(mic, 0, "microphone body missing from model")
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) for i in range(m.ngeom) if int(m.geom_bodyid[i]) == mic]
        self.assertEqual(len(names), 41, names[:5])
        self.assertTrue(all(n.startswith("microphone_brick:") for n in names), names[:5])
        # the build's catalogue masses, each brick at least a gram so a free body has some
        from openbricks_sim import assembly
        import json
        with open(world / "props" / "microphone.assembly.json") as fh:
            out, total = assembly.prop_bricks(json.load(fh))
        self.assertAlmostEqual(total, 81.2, places=1)
        self.assertAlmostEqual(float(m.body_mass[mic]), sum(max(b["mass_g"], 1.0) for b in out) / 1000.0, places=4)
        mujoco.mj_forward(m, d)
        x, y, z = (float(v) for v in d.xpos[mic])
        self.assertAlmostEqual(x, -0.0644, places=4)
        self.assertAlmostEqual(y, -0.4936, places=4)
        self.assertAlmostEqual(z, 0.0096, places=4, msg="its lowest brick on the floor")
        w, _, _, qz = (float(v) for v in d.xquat[mic])
        self.assertAlmostEqual(abs(w), abs(qz), places=6, msg="turned a quarter, as it stands on the truck")
        lowest = min(float(d.geom_xpos[i][2]) - float(m.geom_size[i][2]) for i in range(m.ngeom) if int(m.geom_bodyid[i]) == mic)
        self.assertGreaterEqual(lowest, -1e-6, "nothing under the mat")


if __name__ == "__main__":
    unittest.main()
