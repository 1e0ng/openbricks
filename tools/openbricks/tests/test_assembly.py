# SPDX-License-Identifier: MIT
"""A chassis from an assembly: roles → spec, rolled-up mass properties,
brick geoms, and the model MuJoCo builds from them."""
import copy
import json
import os
import unittest

from openbricks_sim import assembly, bricks
from openbricks_sim.chassis import ChassisSpec, brick_geoms_xml, chassis_mjcf

try:
    import mujoco
except ImportError:                      # pragma: no cover
    mujoco = None

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXAMPLE = os.path.join(_HERE, "..", "..", "sim", "assets", "example.assembly.json")


def _example():
    with open(_EXAMPLE) as fh:
        return json.load(fh)


class DeriveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundle = bricks.load_bundle()
        cls.doc = _example()
        cls.spec, cls.inertial, cls.bricks, cls.notes = assembly.derive(cls.doc, cls.bundle)

    def test_locked_instances_are_plain_instances(self):
        # the sim's editor saves ``locked: true`` on instances it protects; the loader treats them like any other
        doc = copy.deepcopy(self.doc)
        root = doc["robot"]["root"]
        for ch in doc["components"][root]["children"]:
            ch["locked"] = True
        spec, inertial, bricks_out, notes = assembly.derive(doc, self.bundle)
        self.assertEqual(inertial["mass_kg"], self.inertial["mass_kg"])
        self.assertEqual(len(bricks_out), len(self.bricks))
        self.assertEqual(notes, [])

    def test_roles_become_the_flat_spec(self):
        s = self.spec
        self.assertAlmostEqual(s.wheel_radius, 0.0432, places=4)
        self.assertAlmostEqual(s.axle_length, 0.135, places=4)
        self.assertAlmostEqual(s.wheel_width, 0.020, places=4)
        self.assertAlmostEqual(s.caster_offset, 0.075, places=4)
        self.assertAlmostEqual(s.caster_radius, 0.0095, places=4)
        self.assertAlmostEqual(s.color_sensor_y, 0.045, places=4)
        self.assertAlmostEqual(s.color_sensor_yaw, 90.0, places=1)
        self.assertAlmostEqual(s.color_sensor_pitch, 0.0, places=1)
        self.assertAlmostEqual(s.line_sensor_x, 0.070, places=4)
        self.assertAlmostEqual(s.line_sensor_2_x, -0.030, places=4)
        self.assertAlmostEqual(s.pos_x, -0.547, places=4)
        self.assertEqual(s.yaw_deg, 90.0)
        self.assertEqual(self.notes, [])

    def test_mass_properties_match_the_workbench(self):
        self.assertAlmostEqual(self.inertial["mass_kg"], 0.33053, places=4)
        com = [v * 1000 for v in self.inertial["com_m"]]
        self.assertAlmostEqual(com[0], -22.9, delta=0.2)
        self.assertAlmostEqual(com[2], 16.6, delta=0.2)
        diag = [v * 1e9 for v in self.inertial["fullinertia"][:3]]
        self.assertAlmostEqual(diag[0], 545818, delta=50)
        self.assertAlmostEqual(diag[1], 563762, delta=50)
        self.assertAlmostEqual(diag[2], 796247, delta=50)

    def test_every_brick_becomes_a_geom(self):
        self.assertEqual(len(self.bricks), 21)
        paths = [b["path"] for b in self.bricks]
        self.assertIn("left/wheel", paths)
        self.assertIn("pin_fl", paths)
        frame = next(b for b in self.bricks if b["path"] == "frame")
        self.assertEqual(frame["ldraw"], "64178")
        self.assertEqual([round(h * 1000, 1) for h in frame["half_m"]], [20.0, 43.6, 4.0])
        xml = brick_geoms_xml(self.bricks)
        self.assertEqual(len(xml), 21)
        self.assertIn('name="chassis_brick:frame"', xml[0])
        self.assertIn('contype="0"', xml[0])

    def test_missing_wheels_is_an_error(self):
        doc = copy.deepcopy(self.doc)
        doc["robot"]["roles"]["wheel_left"] = ""
        with self.assertRaises(assembly.AssemblyError):
            assembly.derive(doc, self.bundle)
        doc["robot"]["roles"]["wheel_left"] = "nowhere/wheel"
        with self.assertRaises(assembly.AssemblyError):
            assembly.derive(doc, self.bundle)

    def test_bad_documents_are_refused(self):
        with self.assertRaises(assembly.AssemblyError):
            assembly.derive({"format": "nope"}, self.bundle)
        doc = copy.deepcopy(self.doc)
        doc["robot"]["root"] = "missing"
        with self.assertRaises(assembly.AssemblyError):
            assembly.derive(doc, self.bundle)
        doc = copy.deepcopy(self.doc)
        doc["components"]["drive_unit"]["children"].append({"name": "loop", "component": "drive_unit", "pos": [0, 0, 0], "rot": [0, 0, 0]})
        with self.assertRaises(assembly.AssemblyError):
            assembly.derive(doc, self.bundle)
        doc = copy.deepcopy(self.doc)
        doc["parts"]["esp32s3_board"]["shapes"] = []
        with self.assertRaises(assembly.AssemblyError):
            assembly.derive(doc, self.bundle)
        doc = copy.deepcopy(self.doc)
        doc["components"]["robot"]["children"][0]["part"] = "ghost"
        with self.assertRaises(assembly.AssemblyError):
            assembly.derive(doc, self.bundle)

    def test_ldraw_wheel_geometry_from_its_bounding_box(self):
        part = {"name": "rim", "mass_g": 3.0, "ldraw": "56145"}
        r, w = assembly._wheel_geometry(part, self.bundle, assembly.rot_mat([0, 0, 0]))
        self.assertGreater(r, 10.0)
        self.assertLess(w, 2 * r)
        with self.assertRaises(assembly.AssemblyError):
            assembly._wheel_geometry({"name": "x", "mass_g": 1}, self.bundle, assembly.rot_mat([0, 0, 0]))

    def test_quaternion_and_rotation_round_trip(self):
        for rpy in ([0, 0, 90], [90, 0, 0], [0, 90, 0], [30, -40, 120], [0, 0, 180], [180, 0, 0], [0, 180, 0]):
            m = assembly.rot_mat(rpy)
            w, x, y, z = assembly.quat_from_mat(m)
            self.assertAlmostEqual(w * w + x * x + y * y + z * z, 1.0, places=9, msg=str(rpy))
            # rebuild the matrix from the quaternion
            r = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(r[i][j], m[i][j], places=6, msg=str(rpy))

    def test_load_reads_a_file(self):
        spec, inertial, bricks_out, notes = assembly.load(_EXAMPLE, self.bundle)
        self.assertEqual(len(bricks_out), 21)

    def test_shape_part_properties(self):
        part = {"name": "box", "mass_g": 8.0, "shapes": [{"type": "box", "size": [40, 20, 10], "pos": [0, 0, 0]}]}
        m, com, i, bbox = assembly.part_props(part, self.bundle)
        self.assertEqual(m, 8.0)
        self.assertAlmostEqual(i[0][0], 8 / 12 * 500)
        self.assertEqual(bbox, ([-20.0, -10.0, -5.0], [20.0, 10.0, 5.0]))
        cyl = {"name": "c", "mass_g": 2.0, "shapes": [{"type": "cylinder", "radius": 5, "length": 10, "axis": "x"}]}
        m, com, i, bbox = assembly.part_props(cyl, self.bundle)
        self.assertAlmostEqual(i[0][0], 2 * 25 / 2)
        sph = {"name": "s", "mass_g": 1.0, "shapes": [{"type": "sphere", "radius": 3}]}
        self.assertAlmostEqual(assembly.part_props(sph, self.bundle)[2][1][1], 0.4 * 9)


@unittest.skipIf(mujoco is None, "mujoco (the [sim] extra) is required")
class ModelTests(unittest.TestCase):
    def test_standalone_model_carries_the_build(self):
        from openbricks_sim.robot import SimRobot
        doc = _example()
        robot = SimRobot(world=None, assembly=doc)
        m = robot.model
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or "" for i in range(m.ngeom)]
        self.assertEqual(sum(1 for n in names if n.startswith("chassis_brick:")), 21)
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self.assertAlmostEqual(float(m.body_mass[cid]), 0.33053, places=3)
        self.assertEqual(len(robot.assembly_bricks), 21)
        robot.run_for(0.2)
        x, y, yaw = robot.chassis_pose()
        self.assertAlmostEqual(yaw, 90.0, delta=1.0)

    def test_world_model_with_a_file_path(self):
        from openbricks_sim.robot import SimRobot
        robot = SimRobot(world="practice-line", assembly=_EXAMPLE)
        self.assertEqual(len(robot.assembly_bricks), 21)
        self.assertAlmostEqual(robot.chassis_spec.axle_length, 0.135, places=4)

    def test_chassis_spec_override_still_wins(self):
        from openbricks_sim.robot import SimRobot
        robot = SimRobot(world=None, assembly=_example(), chassis_spec=ChassisSpec(wheel_radius=0.05))
        self.assertEqual(robot.chassis_spec.wheel_radius, 0.05)

    def test_inertial_and_extra_geoms_in_the_fragment(self):
        frag = chassis_mjcf(ChassisSpec(), inertial={"mass_kg": 0.5, "com_m": [0.01, 0.0, 0.02], "fullinertia": [1e-4, 2e-4, 3e-4, 0, 0, 0]}, extra_geoms=['      <geom name="chassis_brick:x" type="box" size="0.01 0.01 0.01" contype="0" conaffinity="0" mass="0"/>\n'])
        self.assertIn('fullinertia="1.000000e-04', frag)
        self.assertIn('mass="0.5000"', frag)
        self.assertIn("chassis_brick:x", frag)
        plain = chassis_mjcf(ChassisSpec())
        self.assertIn('diaginertia="0.002 0.002 0.002"', plain)
        self.assertNotIn("chassis_brick", plain)
