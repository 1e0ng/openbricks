# SPDX-License-Identifier: MIT
"""The run server the sim drives: scene export, frames, the program
thread with pacing, pause / resume / stop, and the JSON-lines protocol."""
import io
import json
import os
import tempfile
import threading
import unittest

from openbricks_sim import server

try:
    import mujoco
except ImportError:                      # pragma: no cover
    mujoco = None

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXAMPLE = os.path.join(_HERE, "..", "..", "sim", "assets", "example.assembly.json")


def _events(text):
    return [json.loads(line) for line in text.splitlines() if line.strip()]


class _Feed:
    """A stdin stand-in: commands appear over time so the program thread
    runs between them."""

    def __init__(self, steps):
        self.steps = steps

    def __iter__(self):
        for delay, line in self.steps:
            if delay:
                server._real_sleep(delay)
            yield line + "\n"


class ProtocolTests(unittest.TestCase):
    def test_log_writer_splits_lines(self):
        out = io.StringIO()
        p = server.Protocol(out)
        w = server._LogWriter(p, "stdout")
        w.write("hello ")
        w.write("world\nsecond")
        w.flush()
        ev = _events(out.getvalue())
        self.assertEqual([e["text"] for e in ev], ["hello world", "second"])
        self.assertTrue(all(e["ev"] == "log" for e in ev))

    def test_worlds_list_and_texture_files(self):
        worlds = server.list_worlds()
        aliases = [w["alias"] for w in worlds]
        self.assertIn("empty", aliases)
        self.assertIn("practice-line", aliases)
        line = next(w for w in worlds if w["alias"] == "practice-line")
        self.assertTrue(os.path.isfile(line["path"]))
        tex = server._texture_files(line["path"])
        self.assertEqual(server._texture_files(None), {})
        for path in tex.values():
            self.assertTrue(os.path.isfile(path), path)

    def test_the_editor_commands_reach_the_session(self):
        from unittest import mock
        with tempfile.TemporaryDirectory() as tmp, mock.patch.dict(os.environ, {"OPENBRICKS_DATA_DIR": tmp}):
            out = io.StringIO()
            lines = [
                json.dumps({"cmd": "load", "world": "wro-2026-elementary", "assembly": _EXAMPLE}),
                json.dumps({"cmd": "move", "name": "clef", "x_mm": 100, "y_mm": 100, "yaw_deg": 90}),
                json.dumps({"cmd": "add", "from": "clef", "x_mm": -100, "y_mm": -100}),
                json.dumps({"cmd": "remove", "name": "clef_2"}),
                json.dumps({"cmd": "fix", "name": "clef", "fixed": True}),
                json.dumps({"cmd": "add_model", "name": "x", "doc": {"format": "nope"}, "x_mm": 0, "y_mm": 0}),
                json.dumps({"cmd": "save_world", "name": "wired"}),
                json.dumps({"cmd": "remove", "name": "ghost"}),
                json.dumps({"cmd": "quit"}),
            ]
            self.assertEqual(server.serve(_Feed([(0, l) for l in lines]), out), 0)
            ev = _events(out.getvalue())
            kinds = [e["ev"] for e in ev]
            self.assertEqual(kinds.count("scene"), 4, "load, add, remove, fix")
            self.assertTrue(any(e["ev"] == "error" and "assembly" in e["text"] for e in ev), [e for e in ev if e["ev"] == "error"])
            self.assertIn("saved", kinds)
            self.assertTrue(any(e["ev"] == "error" and "ghost" in e["text"] for e in ev), [e for e in ev if e["ev"] == "error"])
            self.assertTrue(os.path.isfile(os.path.join(tmp, "worlds", "wired", "world.xml")))

    def test_unknown_and_bad_commands_are_reported(self):
        out = io.StringIO()
        rc = server.serve(stdin=io.StringIO('{"cmd": "nope"}\nnot json\n{"cmd": "run", "script": "x.py"}\n{"cmd": "quit"}\n'), stdout=out)
        self.assertEqual(rc, 0)
        ev = _events(out.getvalue())
        self.assertEqual(ev[0]["ev"], "hello")
        texts = [e["text"] for e in ev if e["ev"] == "error"]
        self.assertTrue(any("unknown command" in t for t in texts))
        self.assertTrue(any("bad command line" in t for t in texts))
        self.assertTrue(any("load a world first" in t for t in texts))
        self.assertEqual(ev[-1]["ev"], "bye")

    def test_eof_ends_the_server(self):
        out = io.StringIO()
        self.assertEqual(server.serve(stdin=io.StringIO(""), stdout=out), 0)


@unittest.skipIf(mujoco is None, "mujoco (the [sim] extra) is required")
class SessionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def session(self, out, **kw):
        """A session that is always stopped at the end of the test, so a
        failed assertion never leaves a program thread (and the shim's
        global patches) behind."""
        s = server.Session(server.Protocol(out), **kw)
        self.addCleanup(s.stop)
        return s

    def script(self, body):
        path = os.path.join(self.tmp.name, "main.py")
        with open(path, "w") as fh:
            fh.write(body)
        return path

    def test_place_moves_the_chassis_and_is_refused_during_a_run(self):
        out = io.StringIO()
        s = self.session(out)
        s.load(world="practice-line", assembly=_EXAMPLE)
        scene = next(e for e in _events(out.getvalue()) if e["ev"] == "scene")
        self.assertEqual(scene["chassis"]["wheel_diameter_mm"], 86.4)
        self.assertEqual(scene["chassis"]["axle_track_mm"], 135.0)
        self.assertEqual(scene["chassis"]["spawn"], {"x_mm": -547.0, "y_mm": -150.0, "yaw_deg": 90.0})
        cid = scene["bodies"].index("chassis")
        s.place(100.0, 200.0, -45.0)
        frames = [e for e in _events(out.getvalue()) if e["ev"] == "frame"]
        x, y = frames[-1]["bodies"][cid][:2]
        self.assertAlmostEqual(x, 0.1, places=3)
        self.assertAlmostEqual(y, 0.2, places=3)
        self.assertAlmostEqual(s.robot.chassis_pose()[2], -45.0, places=6)
        s.run(self.script("robot.run_for(5.0)\n"))
        with self.assertRaises(RuntimeError):
            s.place(0.0, 0.0)
        s.stop()
        empty = server.Session(server.Protocol(io.StringIO()))
        with self.assertRaises(RuntimeError):
            empty.place(0.0, 0.0)

    def test_props_move_add_remove_and_save_as_a_map_of_the_users_own(self):
        from unittest import mock
        from openbricks_sim import props
        with mock.patch.dict(os.environ, {"OPENBRICKS_DATA_DIR": self.tmp.name}):
            out = io.StringIO()
            s = self.session(out)
            s.load(world="wro-2026-elementary", assembly=_EXAMPLE)
            scene = next(e for e in _events(out.getvalue()) if e["ev"] == "scene")
            self.assertEqual(len(scene["parents"]), len(scene["bodies"]))
            names = [p["name"] for p in scene["props"]]
            self.assertIn("clef", names)
            clef = next(p for p in scene["props"] if p["name"] == "clef")
            self.assertEqual((scene["bodies"][clef["body"]], clef["kind"], clef["yaw_deg"]), ("clef", "clef", 0.0))
            self.assertEqual(scene["parents"][clef["body"]], 0, "a prop is a body of the world's own")
            # move: the live body is there at once, turned, and a chassis place (a reset) keeps it there
            s.move_prop("clef", 300.0, -200.0, 45.0)
            frame = [e for e in _events(out.getvalue()) if e["ev"] == "frame"][-1]
            x, y, z, w, qx, qy, qz = frame["bodies"][clef["body"]]
            self.assertAlmostEqual(x, 0.3, places=4)
            self.assertAlmostEqual(y, -0.2, places=4)
            self.assertAlmostEqual(qz, 0.3826834, places=5)
            s.place(0.0, 0.0, 0.0)
            frame = [e for e in _events(out.getvalue()) if e["ev"] == "frame"][-1]
            self.assertAlmostEqual(frame["bodies"][clef["body"]][0], 0.3, places=4)
            self.assertIn('yaw="45"', s.world_xml)
            with self.assertRaises(props.PropError):
                s.move_prop("no_such_prop", 0.0, 0.0)
            # add: the world reloads with one more prop, the chassis staying put
            s.place(-400.0, 100.0, 90.0)
            name = s.add_prop("note_red", 100.0, 50.0, 10.0)
            self.assertEqual(name, "note_red_2")
            scene2 = [e for e in _events(out.getvalue()) if e["ev"] == "scene"][-1]
            self.assertEqual(len(scene2["props"]), len(scene["props"]) + 1)
            new = next(p for p in scene2["props"] if p["name"] == "note_red_2")
            self.assertEqual((new["kind"], new["yaw_deg"]), ("note_red", 10.0))
            frame = [e for e in _events(out.getvalue()) if e["ev"] == "frame"][-1]
            self.assertAlmostEqual(frame["bodies"][new["body"]][0], 0.1, places=4)
            cid = scene2["bodies"].index("chassis")
            self.assertAlmostEqual(frame["bodies"][cid][0], -0.4, places=3)
            self.assertAlmostEqual(s.robot.chassis_pose()[2], 90.0, places=3)
            # a document placed as a prop: free at first, its bricks in the scene; stuck on request
            from openbricks_sim import bricks
            num = sorted(bricks.load_bundle()["parts"])[0]
            doc = {"format": "openbricks-assembly/1", "parts": {"p": {"name": "brick", "category": "lego", "mass_g": 2.5, "ldraw": num}},
                   "components": {"one": {"children": [{"name": "b", "part": "p", "pos": [0, 0, 0], "rot": [0, 0, 0]}]}},
                   "robot": {"name": "One Brick", "root": "one"}}
            model_name = s.add_model("One Brick", doc, 250.0, -100.0, 30.0)
            self.assertEqual(model_name, "one_brick")
            scene_m = [e for e in _events(out.getvalue()) if e["ev"] == "scene"][-1]
            one = next(p for p in scene_m["props"] if p["name"] == "one_brick")
            self.assertEqual((one["kind"], one["fixed"], one["yaw_deg"]), ("One Brick", False, 30.0))
            self.assertEqual([b["ldraw"] for b in one["bricks"]], [num])
            frame = [e for e in _events(out.getvalue()) if e["ev"] == "frame"][-1]
            self.assertAlmostEqual(frame["bodies"][one["body"]][0], 0.25, places=4)
            self.assertIn(os.path.join(self.tmp.name, "props"), s.world_xml, "kept under the data directory until saved")
            s.fix_prop("one_brick", True)
            scene_f = [e for e in _events(out.getvalue()) if e["ev"] == "scene"][-1]
            self.assertTrue(next(p for p in scene_f["props"] if p["name"] == "one_brick")["fixed"])
            body = mujoco.mj_name2id(s.robot.model, mujoco.mjtObj.mjOBJ_BODY, "one_brick")
            self.assertEqual(int(s.robot.model.body_jntnum[body]), 0, "stuck: no joint")
            s.move_prop("one_brick", 100.0, 100.0, 0.0)
            frame = [e for e in _events(out.getvalue()) if e["ev"] == "frame"][-1]
            self.assertAlmostEqual(frame["bodies"][body][0], 0.1, places=4, msg="a stuck prop still moves by the editor")
            s.fix_prop("one_brick", False)
            body = mujoco.mj_name2id(s.robot.model, mujoco.mjtObj.mjOBJ_BODY, "one_brick")
            self.assertEqual(int(s.robot.model.body_jntnum[body]), 1, "free again")
            with self.assertRaises(Exception):
                s.add_model("bad", {"format": "nope"}, 0, 0)
            s.remove_prop("one_brick")
            # remove: back to the original count
            s.remove_prop("note_red_2")
            scene3 = [e for e in _events(out.getvalue()) if e["ev"] == "scene"][-1]
            self.assertEqual([p["name"] for p in scene3["props"]], names)
            # save: a map of the user's own, listed with the shipped ones and loadable by alias
            alias = s.save_world("Clef moved")
            self.assertEqual(alias, "clef-moved")
            ev = _events(out.getvalue())
            saved = [e for e in ev if e["ev"] == "saved"][-1]
            self.assertTrue(saved["path"].endswith(os.path.join("worlds", "clef-moved", "world.xml")))
            worlds = [e for e in ev if e["ev"] == "worlds"][-1]["worlds"]
            mine = next(w for w in worlds if w["alias"] == "clef-moved")
            self.assertTrue(mine["user"] and os.path.isfile(mine["path"]))
            self.assertIn("mat.png", os.listdir(mine["dir"]))
            self.assertFalse(next(w for w in worlds if w["alias"] == "practice-line")["user"])
            s.load(world="clef-moved", assembly=_EXAMPLE)
            frame = [e for e in _events(out.getvalue()) if e["ev"] == "frame"][-1]
            scene4 = [e for e in _events(out.getvalue()) if e["ev"] == "scene"][-1]
            clef4 = next(p for p in scene4["props"] if p["name"] == "clef")
            self.assertAlmostEqual(frame["bodies"][clef4["body"]][0], 0.3, places=4)
            self.assertEqual(clef4["yaw_deg"], 45.0)
            with self.assertRaises(props.PropError):
                s.save_world("practice-line")
            s.add_model("Kept Brick", doc, 0.0, 0.0)
            alias2 = s.save_world("With a model")
            saved_dir = os.path.join(self.tmp.name, "worlds", alias2)
            self.assertTrue(any(f.endswith(".assembly.json") for f in os.listdir(os.path.join(saved_dir, "props"))))
            self.assertNotIn(self.tmp.name + os.sep + "props", open(os.path.join(saved_dir, "world.xml")).read())
            s.load(world=alias2, assembly=_EXAMPLE)
            scene_s = [e for e in _events(out.getvalue()) if e["ev"] == "scene"][-1]
            self.assertTrue(any(p["name"] == "kept_brick" and p["bricks"] for p in scene_s["props"]))
            # the empty world has nothing to edit; a running program blocks edits
            e = self.session(io.StringIO())
            e.load(world="empty", assembly=_EXAMPLE)
            with self.assertRaises(RuntimeError):
                e.move_prop("clef", 0.0, 0.0)
            with self.assertRaises(RuntimeError):
                server.Session(server.Protocol(io.StringIO())).save_world("x")
            s.run(self.script("robot.run_for(5.0)\n"))
            with self.assertRaises(RuntimeError):
                s.add_prop("clef", 0.0, 0.0)
            s.stop()

    def test_scene_export_carries_mesh_assets(self):
        from openbricks_sim.bricks.ldraw import unpack_mesh
        out = io.StringIO()
        s = self.session(out)
        s.load(world="wro-2026-senior")
        scene = next(e for e in _events(out.getvalue()) if e["ev"] == "scene")
        frame = next(g for g in scene["geoms"] if g["type"] == "mesh")
        self.assertEqual(frame["mesh"], "mosaic_frame")
        rec = scene["meshes"]["mosaic_frame"]
        self.assertEqual(rec["scale"], 0.1)
        self.assertGreater(rec["tris"], 100)
        pos, nrm, idx = unpack_mesh(rec)
        self.assertEqual(idx.shape, (rec["tris"], 3))
        size = pos.max(axis=0) - pos.min(axis=0)
        self.assertTrue(100 < size.max() < 2000 and size.min() > 1, size)   # a real frame, in mm (not metres)

    def test_scene_export(self):
        out = io.StringIO()
        s = self.session(out)
        s.load(world="wro-2026-elementary", assembly=_EXAMPLE)
        ev = _events(out.getvalue())
        scene = next(e for e in ev if e["ev"] == "scene")
        self.assertIn("chassis", scene["bodies"])
        types = {g["type"] for g in scene["geoms"]}
        self.assertIn("plane", types)
        self.assertIn("box", types)
        self.assertEqual(sum(1 for g in scene["geoms"] if g["name"].startswith("chassis_brick:")), 21)
        self.assertEqual(len(scene["bricks"]), 21)
        mat = scene["materials"].get("mat")
        self.assertIsNotNone(mat)
        self.assertIn(mat["texture"], scene["textures"])
        self.assertTrue(scene["textures"][mat["texture"]].endswith("mat.png"))
        frame = next(e for e in ev if e["ev"] == "frame")
        self.assertEqual(len(frame["bodies"]), len(scene["bodies"]))
        self.assertEqual(len(frame["bodies"][0]), 7)
        # the load frame is a real pose set (a forward pass has run): the world body is
        # upright at the origin and the chassis stands on its wheels at the spawn, not a
        # row of zeros the viewer cannot draw
        self.assertEqual(frame["bodies"][0][3:], [1.0, 0.0, 0.0, 0.0])
        chassis = frame["bodies"][scene["bodies"].index("chassis")]
        self.assertGreater(chassis[2], 0.0)
        self.assertAlmostEqual(sum(q * q for q in chassis[3:]), 1.0, places=4)
        state = [e for e in ev if e["ev"] == "state"][-1]
        self.assertEqual(state["status"], "loaded")
        self.assertEqual(scene["meshes"], {})

    def test_run_finishes_and_logs(self):
        out = io.StringIO()
        s = self.session(out, frame_hz=200.0)
        s.load(world="empty", assembly=_EXAMPLE)
        s.set_speed(20.0)
        s.run(self.script("print('hello')\nrobot.run_for(0.3)\nprint('done', round(robot.chassis_pose()[2]))\n"))
        s.thread.join(timeout=30)
        ev = _events(out.getvalue())
        logs = [e["text"] for e in ev if e["ev"] == "log"]
        self.assertEqual(logs[0], "hello")
        self.assertTrue(logs[-1].startswith("done 90"), logs)
        states = [e["status"] for e in ev if e["ev"] == "state"]
        self.assertEqual(states[-1], "finished")
        frames = [e for e in ev if e["ev"] == "frame"]
        self.assertGreater(len(frames), 3)
        self.assertGreaterEqual(frames[-1]["t_ms"], 300)

    def test_pause_resume_and_stop(self):
        out = io.StringIO()
        s = self.session(out, frame_hz=100.0)
        s.load(world="empty", assembly=_EXAMPLE)
        s.set_speed(20.0)
        s.run(self.script("for i in range(2000):\n    robot.run_for(0.05)\n"))
        server._real_sleep(0.3)
        s.pause()
        self.assertEqual(s.status, "paused")
        server._real_sleep(0.05)
        t_paused = s.robot.runtime.now_ms
        server._real_sleep(0.3)
        self.assertLessEqual(s.robot.runtime.now_ms, t_paused + 1)  # frozen while paused (the step in flight may land)
        s.resume()
        self.assertEqual(s.status, "running")
        server._real_sleep(0.3)
        self.assertGreater(s.robot.runtime.now_ms, t_paused)
        s.stop()
        self.assertFalse(s.thread.is_alive())
        self.assertEqual(s.status, "stopped")
        states = [e["status"] for e in _events(out.getvalue()) if e["ev"] == "state"]
        self.assertEqual(states[-3:], ["paused", "running", "stopped"])

    def test_program_errors_are_reported(self):
        out = io.StringIO()
        s = self.session(out)
        s.load(world="empty", assembly=_EXAMPLE)
        s.run(self.script("robot.run_for(0.01)\nraise ValueError('boom')\n"))
        s.thread.join(timeout=30)
        state = [e for e in _events(out.getvalue()) if e["ev"] == "state"][-1]
        self.assertEqual(state["status"], "error")
        self.assertIn("boom", state["error"])
        # a second run on the same session works; sys.exit(3) is reported
        s.run(self.script("import sys\nsys.exit(3)\n"))
        s.thread.join(timeout=30)
        state = [e for e in _events(out.getvalue()) if e["ev"] == "state"][-1]
        self.assertEqual(state["status"], "finished")
        self.assertIn("exit status 3", state["error"])

    def test_load_and_run_guards(self):
        out = io.StringIO()
        s = self.session(out)
        with self.assertRaises(RuntimeError):
            s.run("x.py")
        s.load(world="empty", assembly=_EXAMPLE)
        s.set_speed(20.0)
        s.run(self.script("robot.run_for(5.0)\n"))
        with self.assertRaises(RuntimeError):
            s.run(self.script("pass\n"))
        with self.assertRaises(RuntimeError):
            s.load(world="empty")
        s.stop()
        self.assertEqual(s.status, "stopped")
        s.stop()                                                    # an idle stop changes nothing
        self.assertEqual(s.status, "stopped")
        s.set_speed(0.0)
        self.assertEqual(s.speed, 0.05)

    def test_serve_end_to_end(self):
        out = io.StringIO()
        script = self.script("print('go')\nfor i in range(400):\n    robot.run_for(0.05)\n")
        feed = _Feed([
            (0, '{"cmd": "worlds"}'),
            (0, json.dumps({"cmd": "load", "world": "empty", "assembly": _EXAMPLE})),
            (0, '{"cmd": "speed", "factor": 10}'),
            (0, json.dumps({"cmd": "run", "script": script})),
            (0.4, '{"cmd": "pause"}'),
            (0.2, '{"cmd": "resume"}'),
            (0.2, '{"cmd": "stop"}'),
            (0, '{"cmd": "quit"}'),
        ])
        rc = server.serve(stdin=feed, stdout=out, frame_hz=50.0)
        self.assertEqual(rc, 0)
        ev = _events(out.getvalue())
        kinds = [e["ev"] for e in ev]
        self.assertEqual(kinds[0], "hello")
        self.assertIn("worlds", kinds)
        self.assertIn("scene", kinds)
        self.assertIn("log", kinds)
        states = [e["status"] for e in ev if e["ev"] == "state"]
        self.assertIn("running", states)
        self.assertIn("paused", states)
        self.assertEqual(states[-1], "stopped")
        self.assertEqual(kinds[-1], "bye")
