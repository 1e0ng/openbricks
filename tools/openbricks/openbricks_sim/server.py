# SPDX-License-Identifier: MIT
"""The run server the sim drives: ``python -m openbricks_sim.server``.

The sim (the native app) starts this as a child process and talks
JSON lines over its stdin / stdout. The physics, the driver shim and
the user's program all live here — MuJoCo's Python bindings, the
firmware's own driver code and the shared C cores are Python-side —
while the app renders what it is told.

Commands (one JSON object per line on stdin)::

    {"cmd": "worlds"}
    {"cmd": "load", "world": "practice-line", "assembly": "robot.assembly.json"}
    {"cmd": "run", "script": "main.py"}
    {"cmd": "place", "x_mm": -547, "y_mm": -150, "yaw_deg": 90}
    {"cmd": "pause"}   {"cmd": "resume"}   {"cmd": "stop"}
    {"cmd": "speed", "factor": 2.0}
    {"cmd": "quit"}

Events (one JSON object per line on stdout)::

    {"ev": "worlds", "worlds": [{"alias", "path", "dir"}, ...]}
    {"ev": "scene", "bodies": [...], "geoms": [...], "materials": {...},
                    "textures": {...}, "meshes": {...}, "bricks": [...],
                    "chassis": {"wheel_diameter_mm", "axle_track_mm", "spawn": {"x_mm", "y_mm", "yaw_deg"}},
                    "timestep_ms": 1}
    {"ev": "frame", "t_ms": 1234, "bodies": [[x, y, z, qw, qx, qy, qz], ...]}
    {"ev": "log", "text": "..."}            # the program's prints
    {"ev": "state", "status": "idle|loaded|running|paused|finished|stopped|error", ...}
    {"ev": "error", "text": "..."}

Frames are sent while a program runs, at most ``frame_hz`` per second
of wall time, and the run is paced to wall time × ``speed``.
"""
import io
import json
import os
import re
import sys
import threading
import time
import traceback

_real_sleep = time.sleep          # captured before the shim patches time.sleep
_real_monotonic = time.monotonic


class SimStopped(SystemExit):
    """Raised inside a tick to unwind the program on ``stop``."""


class Protocol:
    """JSON-lines writer (thread-safe) over a binary stream."""

    def __init__(self, out):
        self.out = out
        self.lock = threading.Lock()

    def send(self, **event):
        line = json.dumps(event, separators=(",", ":")) + "\n"
        with self.lock:
            self.out.write(line)
            self.out.flush()


class _LogWriter(io.TextIOBase):
    """``sys.stdout`` / ``sys.stderr`` replacement that turns the
    program's prints into ``log`` events."""

    def __init__(self, protocol, stream):
        self.protocol = protocol
        self.stream = stream
        self._buf = ""

    def writable(self):
        return True

    def write(self, s):
        self._buf += s
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            self.protocol.send(ev="log", stream=self.stream, text=line)
        return len(s)

    def flush(self):
        if self._buf:
            self.protocol.send(ev="log", stream=self.stream, text=self._buf)
            self._buf = ""


def list_worlds():
    from openbricks_sim import robot as robot_mod
    out = []
    for alias in robot_mod._BUILTIN_WORLDS:
        path = robot_mod._resolve_world(alias)
        out.append({"alias": alias, "path": path, "dir": os.path.dirname(path) if path else None})
    return out


def _texture_files(world_path):
    """``{texture name: absolute file}`` from a world's ``<texture … file="…">``."""
    if not world_path:
        return {}
    text = open(world_path).read()
    base = os.path.dirname(os.path.abspath(world_path))
    out = {}
    for m in re.finditer(r"<texture\b[^>]*>", text):
        tag = m.group(0)
        name = re.search(r'name="([^"]+)"', tag)
        file = re.search(r'file="([^"]+)"', tag)
        if name and file:
            out[name.group(1)] = os.path.join(base, file.group(1))
    return out


def _packed_mesh(model, mid):
    """A mesh asset as the compiled model holds it (vertices already
    centred and scaled as MuJoCo renders them), in mm at 0.1 mm steps."""
    import numpy as np
    from openbricks_sim.bricks.ldraw import pack_mesh
    va, vn = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
    fa, fn = int(model.mesh_faceadr[mid]), int(model.mesh_facenum[mid])
    verts = np.asarray(model.mesh_vert[va:va + vn], dtype=np.float64) * 1000.0
    faces = np.asarray(model.mesh_face[fa:fa + fn], dtype=np.int64)
    return pack_mesh(verts[faces], q=10.0)


def chassis_info(spec):
    """The chassis geometry a route planner needs, in mm and degrees."""
    return {"wheel_diameter_mm": round(float(spec.wheel_radius) * 2000.0, 3),
            "axle_track_mm": round(float(spec.axle_length) * 1000.0, 3),
            "spawn": {"x_mm": round(float(spec.pos_x) * 1000.0, 3), "y_mm": round(float(spec.pos_y) * 1000.0, 3),
                      "yaw_deg": round(float(spec.yaw_deg), 3)}}


def export_scene(model, world_path=None, bricks=(), chassis_spec=None):
    """Everything a viewer needs once: bodies, geoms with their local
    pose and material, materials with texture names, texture files,
    the mesh assets the mesh geoms name, and the chassis geometry."""
    import mujoco
    geom_types = {mujoco.mjtGeom.mjGEOM_PLANE: "plane", mujoco.mjtGeom.mjGEOM_SPHERE: "sphere",
                  mujoco.mjtGeom.mjGEOM_CAPSULE: "capsule", mujoco.mjtGeom.mjGEOM_ELLIPSOID: "ellipsoid",
                  mujoco.mjtGeom.mjGEOM_CYLINDER: "cylinder", mujoco.mjtGeom.mjGEOM_BOX: "box",
                  mujoco.mjtGeom.mjGEOM_MESH: "mesh"}
    bodies = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ("body%d" % i) for i in range(model.nbody)]
    materials = {}
    for i in range(model.nmat):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MATERIAL, i) or ("mat%d" % i)
        ids = model.mat_texid[i]
        if hasattr(ids, "__len__"):            # MuJoCo >= 3.1: one id per texture role, RGB is role 1
            tex = int(ids[1]) if len(ids) > 1 and int(ids[1]) >= 0 else int(ids[0])
        else:
            tex = int(ids)
        materials[name] = {
            "rgba": [float(v) for v in model.mat_rgba[i]],
            "texture": (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TEXTURE, tex) if tex >= 0 else None),
            "texrepeat": [float(v) for v in model.mat_texrepeat[i]],
        }
    geoms = []
    meshes = {}
    for i in range(model.ngeom):
        gt = geom_types.get(int(model.geom_type[i]), "other")
        mat = int(model.geom_matid[i])
        mesh_file = None
        if gt == "mesh":
            mid = int(model.geom_dataid[i])
            mesh_file = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mid) or ("mesh%d" % mid)
            if mesh_file not in meshes:
                meshes[mesh_file] = _packed_mesh(model, mid)
        geoms.append({
            "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "",
            "type": gt,
            "body": int(model.geom_bodyid[i]),
            "size": [float(v) for v in model.geom_size[i]],
            "pos": [float(v) for v in model.geom_pos[i]],
            "quat": [float(v) for v in model.geom_quat[i]],
            "rgba": [float(v) for v in model.geom_rgba[i]],
            "material": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MATERIAL, mat) if mat >= 0 else None,
            "group": int(model.geom_group[i]),
            "mesh": mesh_file,
        })
    return {"bodies": bodies, "geoms": geoms, "materials": materials, "textures": _texture_files(world_path),
            "meshes": meshes, "bricks": list(bricks),
            "chassis": chassis_info(chassis_spec) if chassis_spec is not None else None,
            "timestep_ms": max(1, int(round(model.opt.timestep * 1000.0)))}


def frame_of(model, data, t_ms):
    return {"ev": "frame", "t_ms": int(t_ms), "bodies": [[round(float(v), 5) for v in data.xpos[i]] + [round(float(v), 6) for v in data.xquat[i]] for i in range(model.nbody)]}


class Session:
    """One loaded world + chassis; runs at most one program at a time."""

    def __init__(self, protocol, frame_hz=60.0):
        self.protocol = protocol
        self.frame_hz = frame_hz
        self.robot = None
        self.world = None
        self.world_path = None
        self.assembly = None
        self.speed = 1.0
        self.paused = threading.Event()
        self.paused.set()                 # set = not paused
        self.stop_flag = False
        self.thread = None
        self.status = "idle"
        self.error = None

    # ----------------------------------------------------------- loading
    def load(self, world=None, assembly=None, chassis=None):
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError("a program is running; stop it first")
        from openbricks_sim import robot as robot_mod
        from openbricks_sim.chassis import ChassisSpec
        spec = None
        if chassis:
            with open(chassis) as fh:
                spec = ChassisSpec(**json.load(fh))
        self.robot = robot_mod.SimRobot(world=world, chassis_spec=spec, assembly=assembly)
        self.world = world
        self.world_path = robot_mod._resolve_world(world)
        self.assembly = assembly
        self.status = "loaded"
        scene = export_scene(self.robot.model, self.world_path, self.robot.assembly_bricks, self.robot.chassis_spec)
        self.protocol.send(ev="scene", **scene)
        self._send_frame()
        self._send_state()

    def place(self, x_mm, y_mm, yaw_deg=0.0):
        """Put the chassis at a pose on the map (a route's start, or
        where the user dragged it); refused while a program runs."""
        if self.robot is None:
            raise RuntimeError("load a world first")
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError("a program is running; stop it first")
        self.robot.set_pose(float(x_mm), float(y_mm), float(yaw_deg))
        self._send_frame()
        self._send_state()

    def _send_frame(self):
        self.protocol.send(**frame_of(self.robot.model, self.robot.data, self.robot.runtime.now_ms))

    def _send_state(self, **extra):
        self.protocol.send(ev="state", status=self.status, t_ms=self.robot.runtime.now_ms if self.robot else 0, speed=self.speed, error=self.error, **extra)

    # ----------------------------------------------------------- running
    def run(self, script):
        if self.robot is None:
            raise RuntimeError("load a world first")
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError("a program is already running")
        self.stop_flag = False
        self.paused.set()
        self.error = None
        self.status = "running"
        self._send_state(script=script)
        self.thread = threading.Thread(target=self._run_script, args=(script,), daemon=True, name="program")
        self.thread.start()

    def _run_script(self, script):
        import runpy
        from openbricks_sim import shim
        robot = self.robot
        runtime = robot.runtime
        last_frame = [0.0]
        wall0 = [_real_monotonic()]
        sim0 = [runtime.now_ms]
        frame_period = 1.0 / self.frame_hz

        def tick(now_ms):
            if self.stop_flag:
                raise SimStopped(0)
            if not self.paused.is_set():
                self._send_frame()
                self.paused.wait()
                wall0[0] = _real_monotonic()
                sim0[0] = now_ms
            # pace to wall time × speed
            target_wall = wall0[0] + (now_ms - sim0[0]) / 1000.0 / max(self.speed, 1e-3)
            lag = target_wall - _real_monotonic()
            if lag > 0.0005:
                _real_sleep(min(lag, 0.05))
            elif lag < -0.5:
                wall0[0] = _real_monotonic()
                sim0[0] = now_ms
            now = _real_monotonic()
            if now - last_frame[0] >= frame_period:
                last_frame[0] = now
                self._send_frame()

        runtime.add_tick(tick)
        shim.install(runtime)
        old_out, old_err = sys.stdout, sys.stderr
        sys.stdout = _LogWriter(self.protocol, "stdout")
        sys.stderr = _LogWriter(self.protocol, "stderr")
        try:
            runpy.run_path(script, init_globals={"robot": robot, "drivebase": robot.drivebase, "left": robot.left, "right": robot.right}, run_name="__main__")
            self.status = "stopped" if self.stop_flag else "finished"
        except SimStopped:
            self.status = "stopped"
        except SystemExit as e:
            self.status = "stopped" if self.stop_flag else "finished"
            if e.code not in (None, 0) and not self.stop_flag:
                self.error = "exit status %r" % (e.code,)
        except BaseException:  # noqa: BLE001 - the program's failure is reported, not raised
            self.status = "error"
            self.error = traceback.format_exc()
        finally:
            sys.stdout.flush()
            sys.stderr.flush()
            sys.stdout, sys.stderr = old_out, old_err
            try:
                shim.uninstall()
            except Exception:  # noqa: BLE001
                pass
            runtime.remove_tick(tick)
            try:
                self._send_frame()
            except Exception:  # noqa: BLE001
                pass
            self._send_state()

    def pause(self):
        if self.status == "running":
            self.paused.clear()
            self.status = "paused"
            self._send_state()

    def resume(self):
        if self.status == "paused":
            self.status = "running"
            self.paused.set()
            self._send_state()

    def stop(self):
        if self.thread is not None and self.thread.is_alive():
            self.stop_flag = True
            self.paused.set()
            self.thread.join(timeout=10.0)
        elif self.status in ("running", "paused"):     # a thread that died without reporting
            self.status = "stopped"
            self._send_state()

    def set_speed(self, factor):
        self.speed = max(0.05, min(20.0, float(factor)))
        if self.robot is not None:
            self._send_state()


def serve(stdin=None, stdout=None, frame_hz=60.0):
    """Read commands until EOF or ``quit``; returns the exit status."""
    stdin = stdin or sys.stdin
    out = stdout or sys.stdout
    protocol = Protocol(out)
    session = Session(protocol, frame_hz=frame_hz)
    protocol.send(ev="hello", version=_version(), pid=os.getpid())
    for line in stdin:
        line = line.strip()
        if not line:
            continue
        try:
            cmd = json.loads(line)
        except ValueError as exc:
            protocol.send(ev="error", text="bad command line: %s" % exc)
            continue
        name = cmd.get("cmd")
        try:
            if name == "worlds":
                protocol.send(ev="worlds", worlds=list_worlds())
            elif name == "load":
                session.load(world=cmd.get("world"), assembly=cmd.get("assembly"), chassis=cmd.get("chassis"))
            elif name == "run":
                session.run(cmd["script"])
            elif name == "place":
                session.place(cmd["x_mm"], cmd["y_mm"], cmd.get("yaw_deg", 0.0))
            elif name == "pause":
                session.pause()
            elif name == "resume":
                session.resume()
            elif name == "stop":
                session.stop()
            elif name == "speed":
                session.set_speed(cmd.get("factor", 1.0))
            elif name == "quit":
                session.stop()
                protocol.send(ev="bye")
                return 0
            else:
                protocol.send(ev="error", text="unknown command %r" % (name,))
        except Exception as exc:  # noqa: BLE001 - every command failure is reported on the wire
            protocol.send(ev="error", text="%s: %s" % (type(exc).__name__, exc))
    session.stop()
    return 0


def _version():
    from openbricks_dev import __version__
    return __version__


if __name__ == "__main__":     # pragma: no cover
    sys.exit(serve())
