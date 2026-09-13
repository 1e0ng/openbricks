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
    {"cmd": "move", "name": "clef", "x_mm": 300, "y_mm": -200, "yaw_deg": 45}   # a prop
    {"cmd": "add", "from": "note_red", "x_mm": 0, "y_mm": 0, "yaw_deg": 0}     # another like it
    {"cmd": "add_model", "name": "tower", "doc": {...openbricks-assembly/1...}, "x_mm": 0, "y_mm": 0, "yaw_deg": 0}
    {"cmd": "fix", "name": "clef", "fixed": true}   # stuck to the map (false: free again)
    {"cmd": "remove", "name": "note_red_2"}
    {"cmd": "save_world", "name": "My layout"}   # the map as it stands, as the user's own
    {"cmd": "pause"}   {"cmd": "resume"}   {"cmd": "stop"}
    {"cmd": "speed", "factor": 2.0}
    {"cmd": "quit"}

Events (one JSON object per line on stdout)::

    {"ev": "worlds", "worlds": [{"alias", "path", "dir", "user"}, ...]}
    {"ev": "scene", "bodies": [...], "parents": [...], "geoms": [...], "materials": {...},
                    "textures": {...}, "meshes": {...}, "bricks": [...],
                    "props": [{"name", "body", "kind", "color", "yaw_deg", "fixed", "bricks": [...]}, ...],
                    "chassis": {"wheel_diameter_mm", "axle_track_mm", "spawn": {"x_mm", "y_mm", "yaw_deg"}},
                    "timestep_ms": 1}
    {"ev": "saved", "alias": "my-layout", "path": ".../worlds/my-layout/world.xml"}
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

from openbricks_sim import props

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
    """The shipped maps, then the user's own from the data directory."""
    from openbricks_sim import robot as robot_mod
    out = []
    for alias in robot_mod._BUILTIN_WORLDS:
        path = robot_mod._resolve_world(alias)
        out.append({"alias": alias, "path": path, "dir": os.path.dirname(path) if path else None, "user": False})
    out.extend(w for w in props.list_user_worlds() if w["alias"] not in robot_mod._BUILTIN_WORLDS)
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
    return {"bodies": bodies, "parents": [int(v) for v in model.body_parentid], "geoms": geoms, "materials": materials,
            "textures": _texture_files(world_path),
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
        self.chassis_spec = None
        # the map's text with every prop where the editor put it: what a save writes
        self.world_xml = None
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
        self.chassis_spec = spec
        self.world_xml = open(self.world_path).read() if self.world_path else None
        self.status = "loaded"
        self._send_scene()
        self._send_frame()
        self._send_state()

    def _send_scene(self):
        scene = export_scene(self.robot.model, self.world_path, self.robot.assembly_bricks, self.robot.chassis_spec)
        scene["props"] = self._props()
        self.protocol.send(ev="scene", **scene)

    def _props(self):
        """The map's props as the viewer needs them: name, body id, kind
        (an LDraw model's file stem, or a document's name), colour, yaw,
        whether it is stuck to the map, and a document's bricks so the
        viewer can draw them exactly."""
        if not self.world_xml:
            return []
        import mujoco
        out = []
        world_dir = os.path.dirname(self.world_path) if self.world_path else ""
        for p in props.props_in(self.world_xml):
            bid = mujoco.mj_name2id(self.robot.model, mujoco.mjtObj.mjOBJ_BODY, p["name"])
            entry = {"name": p["name"], "body": int(bid), "color": p.get("color"), "yaw_deg": p["yaw"], "fixed": bool(p["fixed"]), "bricks": []}
            if p["tag"] == "lego_prop":
                entry["kind"] = os.path.splitext(os.path.basename(p["ldr"]))[0]
            else:
                path = p["file"] if os.path.isabs(p["file"]) else os.path.join(world_dir, p["file"])
                kind, bricks_out = self._model(path)
                entry["kind"] = kind
                entry["bricks"] = bricks_out
            out.append(entry)
        return out

    def _model(self, path):
        """A document's name and bricks, read once per file."""
        cache = self.__dict__.setdefault("_models", {})
        if path not in cache:
            from openbricks_sim import assembly as assembly_mod
            with open(path) as fh:
                doc = json.load(fh)
            bricks_out, _ = assembly_mod.prop_bricks(doc)
            robot = doc.get("robot") or {}
            cache[path] = (robot.get("name") or robot.get("root") or "model", bricks_out)
        return cache[path]

    # ------------------------------------------------------ the map editor
    def _editable(self, what):
        if self.robot is None:
            raise RuntimeError("load a world first")
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError("a program is running; stop it first")
        if not self.world_xml:
            raise RuntimeError("the empty world has no props to %s" % what)

    def move_prop(self, name, x_mm, y_mm, yaw_deg=0.0):
        """Put a prop at a pose on the map: the live body moves at once
        (its height kept, and the pose a reset returns to updated), and
        the map's text remembers it."""
        self._editable("move")
        import math
        import mujoco
        x, y, yaw = float(x_mm) / 1000.0, float(y_mm) / 1000.0, float(yaw_deg)
        self.world_xml = props.with_prop_moved(self.world_xml, name, x, y, yaw)
        model, data = self.robot.model, self.robot.data
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            raise RuntimeError("no body named %r in the loaded map" % (name,))
        half = math.radians(yaw) / 2.0
        quat = (math.cos(half), 0.0, 0.0, math.sin(half))
        jnt = int(model.body_jntadr[bid]) if int(model.body_jntnum[bid]) > 0 else -1
        if jnt >= 0 and int(model.jnt_type[jnt]) == int(mujoco.mjtJoint.mjJNT_FREE):
            q, v = int(model.jnt_qposadr[jnt]), int(model.jnt_dofadr[jnt])
            z = float(data.qpos[q + 2])
            data.qpos[q:q + 3] = (x, y, z)
            data.qpos[q + 3:q + 7] = quat
            data.qvel[v:v + 6] = 0.0
            model.qpos0[q:q + 7] = data.qpos[q:q + 7]
        else:
            model.body_pos[bid] = (x, y, float(model.body_pos[bid][2]))
            model.body_quat[bid] = quat
        mujoco.mj_forward(model, data)
        self._send_frame()
        self._send_state()

    def add_prop(self, from_name, x_mm, y_mm, yaw_deg=0.0):
        """Another prop like ``from_name`` at a pose: the map's text gains
        it and the world reloads, the chassis staying where it stands.
        Returns the new prop's name."""
        self._editable("add to")
        xml, name = props.with_prop_added(self.world_xml, from_name, float(x_mm) / 1000.0, float(y_mm) / 1000.0, float(yaw_deg))
        self._reload(xml)
        return name

    def add_model(self, name, doc, x_mm, y_mm, yaw_deg=0.0):
        """A document (what the Workbench builds, or one brick) placed as
        a new, free prop: the document is kept under the data directory
        until the map is saved, the map's text gains the prop, and the
        world reloads. Returns the prop's name."""
        self._editable("add to")
        from openbricks_sim import assembly as assembly_mod
        if not isinstance(doc, dict):
            raise RuntimeError("add_model needs the document itself (a JSON object)")
        assembly_mod.prop_bricks(doc)          # loud before anything is written
        path = props.stage_file(json.dumps(doc, separators=(",", ":"), sort_keys=True), name, "assembly.json")
        xml, prop_name = props.with_model_added(self.world_xml, props.slug(name).replace("-", "_"), path,
                                                float(x_mm) / 1000.0, float(y_mm) / 1000.0, float(yaw_deg))
        self._reload(xml)
        return prop_name

    def fix_prop(self, name, fixed):
        """Stick a prop to the map (no free joint: nothing moves it but
        the editor), or free it again; the world reloads either way."""
        self._editable("edit")
        self._reload(props.with_prop_fixed(self.world_xml, name, bool(fixed)))

    def remove_prop(self, name):
        self._editable("edit")
        self._reload(props.with_prop_removed(self.world_xml, name))

    def _reload(self, xml):
        from openbricks_sim import robot as robot_mod
        pose = self._chassis_pose()
        self.robot = robot_mod.SimRobot(world=self.world, chassis_spec=self.chassis_spec, assembly=self.assembly, world_xml=xml)
        self.world_xml = xml
        if pose is not None:
            self.robot.set_pose(*pose)
        self._send_scene()
        self._send_frame()
        self._send_state()

    def _chassis_pose(self):
        try:
            return self.robot.chassis_pose()
        except Exception:  # noqa: BLE001 - a chassis-less model has no pose to keep
            return None

    def save_world(self, name):
        """Write the map as it stands — every prop where it is, the ones
        added included — as a map of the user's own, then list the maps
        again. Returns its alias."""
        self._editable("save")
        from openbricks_sim import robot as robot_mod
        alias, path = props.save_as(os.path.dirname(self.world_path), self.world_xml, name, reserved=robot_mod._BUILTIN_WORLDS)
        self.protocol.send(ev="worlds", worlds=list_worlds())
        self.protocol.send(ev="saved", alias=alias, path=path)
        return alias

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
            elif name == "move":
                session.move_prop(cmd["name"], cmd["x_mm"], cmd["y_mm"], cmd.get("yaw_deg", 0.0))
            elif name == "add":
                session.add_prop(cmd["from"], cmd["x_mm"], cmd["y_mm"], cmd.get("yaw_deg", 0.0))
            elif name == "add_model":
                session.add_model(cmd["name"], cmd["doc"], cmd["x_mm"], cmd["y_mm"], cmd.get("yaw_deg", 0.0))
            elif name == "fix":
                session.fix_prop(cmd["name"], cmd.get("fixed", True))
            elif name == "remove":
                session.remove_prop(cmd["name"])
            elif name == "save_world":
                session.save_world(cmd["name"])
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
