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
