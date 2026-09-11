# SPDX-License-Identifier: MIT
"""A chassis from an assembly: ``robot.assembly.json`` → MuJoCo.

The workbench (the sim, or ``openbricks sim workbench``) writes a
robot as a tree of components made of bricks; this module turns that
into the chassis the runtime drives:

* the **roles** name the parts the runtime binds — the two drive
  wheels, the caster, the reflectance sites, the colour sensor, the
  range sensor, the IMU. From their positions the same
  :class:`~openbricks_sim.chassis.ChassisSpec` fields the flat file
  carries are derived, with the axle midpoint as the chassis origin,
  so :func:`~openbricks_sim.chassis.chassis_mjcf` builds the proven
  physics skeleton (wheels on hinges, caster, sites, cameras, motors,
  sensors) unchanged;
* the **mass properties** of the build — mass, centre of mass and
  the full inertia tensor, rolled up brick by brick from the recorded
  weights and the exact LDraw meshes — replace the flat spec's box
  guess as the chassis body's ``<inertial>``;
* every **brick** becomes a visual geom named ``chassis_brick:<path>``
  (its bounding box, no contact, no mass) so a viewer can draw the
  build and the sim can put the exact mesh in its place.

Frames: the assembly is X forward, Y left, Z up in millimetres; the
chassis fragment is metres.
"""
import json
import math

from openbricks_sim import bricks
from openbricks_sim.chassis import ChassisSpec

ROLES = ("wheel_left", "wheel_right", "caster", "line_sensor", "line_sensor_2",
         "color_sensor", "distance_sensor", "imu")
MODULE_MM = 8.0


class AssemblyError(ValueError):
    """The assembly cannot become a chassis (missing roles, bad parts)."""


# ------------------------------------------------------------ maths (mm)
def rot_mat(rpy):
    """``R = Rz(yaw) · Ry(pitch) · Rx(roll)`` from degrees, as rows."""
    r, p, y = [math.radians(a) for a in rpy]
    sr, cr, sp, cp, sy, cy = math.sin(r), math.cos(r), math.sin(p), math.cos(p), math.sin(y), math.cos(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def mat_vec(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_t(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def _add(a, b):
    return [a[i] + b[i] for i in range(3)]


def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _scale(a, s):
    return [a[i] * s for i in range(3)]


def _shift(m, d):
    dd = sum(x * x for x in d)
    return [[m * (dd - d[i] * d[i]) if i == j else -m * d[i] * d[j] for j in range(3)] for i in range(3)]


def _mat_add(a, b):
    return [[a[i][j] + b[i][j] for j in range(3)] for i in range(3)]


def quat_from_mat(m):
    """``(w, x, y, z)`` of a rotation matrix given as rows."""
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)


# ---------------------------------------------------------- part geometry
def _shape_half(shape):
    t = shape.get("type")
    if t == "box":
        return [s / 2.0 for s in shape["size"]]
    if t == "cylinder":
        r, h = shape["radius"], shape["length"] / 2.0
        return {"x": [h, r, r], "y": [r, h, r]}.get(shape.get("axis", "z"), [r, r, h])
    return [shape["radius"]] * 3


def _shape_volume(shape):
    t = shape.get("type")
    if t == "box":
        return shape["size"][0] * shape["size"][1] * shape["size"][2]
    if t == "cylinder":
        return math.pi * shape["radius"] ** 2 * shape["length"]
    return 4.0 / 3.0 * math.pi * shape["radius"] ** 3


def _shape_inertia(shape, m):
    t = shape.get("type")
    if t == "box":
        a, b, c = shape["size"]
        return [[m / 12 * (b * b + c * c), 0, 0], [0, m / 12 * (a * a + c * c), 0], [0, 0, m / 12 * (a * a + b * b)]]
    if t == "cylinder":
        ax = m * shape["radius"] ** 2 / 2
        tr = m / 12 * (3 * shape["radius"] ** 2 + shape["length"] ** 2)
        d = {"x": [ax, tr, tr], "y": [tr, ax, tr]}.get(shape.get("axis", "z"), [tr, tr, ax])
        return [[d[0], 0, 0], [0, d[1], 0], [0, 0, d[2]]]
    k = 0.4 * m * shape["radius"] ** 2
    return [[k, 0, 0], [0, k, 0], [0, 0, k]]


def part_props(part, bundle):
    """``(mass_g, com_mm, inertia_g_mm2, bbox_mm)`` of one brick in its own frame."""
    mass = float(part["mass_g"])
    rec = bundle["parts"].get(part["ldraw"]) if part.get("ldraw") else None
    if rec is None and part.get("mesh") and part.get("bbox") and part.get("inertia_per_g"):
        rec = part
    if rec is not None:
        i = [[float(v) * mass for v in row] for row in rec["inertia_per_g"]]
        return mass, [float(v) for v in rec["com"]], i, ([float(v) for v in rec["bbox"][0]], [float(v) for v in rec["bbox"][1]])
    shapes = part.get("shapes") or []
    if not shapes:
        raise AssemblyError("brick %r has no geometry (no LDraw number, mesh or shapes)" % part.get("name", "?"))
    vols = [_shape_volume(s) for s in shapes]
    vt = sum(vols)
    masses = [mass * v / vt if vt > 0 else mass / len(shapes) for v in vols]
    com = [0.0, 0.0, 0.0]
    lo, hi = [math.inf] * 3, [-math.inf] * 3
    for s, m in zip(shapes, masses):
        p = s.get("pos", [0, 0, 0])
        com = _add(com, _scale(p, m))
        h = _shape_half(s)
        lo = [min(lo[k], p[k] - h[k]) for k in range(3)]
        hi = [max(hi[k], p[k] + h[k]) for k in range(3)]
    com = _scale(com, 1.0 / mass) if mass > 0 else [0.0, 0.0, 0.0]
    inertia = [[0.0] * 3 for _ in range(3)]
    for s, m in zip(shapes, masses):
        inertia = _mat_add(inertia, _mat_add(_shape_inertia(s, m), _shift(m, _sub(s.get("pos", [0, 0, 0]), com))))
    return mass, com, inertia, (lo, hi)


def flatten(doc, comp_id, p0=(0.0, 0.0, 0.0), r0=None, path=(), stack=()):
    """Every brick under a component with its pose in that component's frame."""
    r0 = r0 or [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    out = []
    comp = doc["components"].get(comp_id)
    if comp is None:
        raise AssemblyError("component %r does not exist" % comp_id)
    for ch in comp.get("children", []):
        r = mat_mul(r0, rot_mat(ch.get("rot", [0, 0, 0])))
        p = _add(list(p0), mat_vec(r0, ch["pos"]))
        cpath = path + (ch["name"],)
        if ch.get("part"):
            out.append({"path": "/".join(cpath), "part": ch["part"], "pos": p, "rot": r})
        elif ch.get("component"):
            if ch["component"] in stack:
                raise AssemblyError("component %r contains itself" % ch["component"])
            out.extend(flatten(doc, ch["component"], p, r, cpath, stack + (ch["component"],)))
    return out


def mass_properties(doc, bundle, leaves):
    """Mass (g), centre of mass (mm) and inertia about it (g·mm²) of the build."""
    placed = []
    for leaf in leaves:
        part = doc["parts"].get(leaf["part"])
        if part is None:
            raise AssemblyError("brick %r is not in the library" % leaf["part"])
        m, com, i, _ = part_props(part, bundle)
        c = _add(leaf["pos"], mat_vec(leaf["rot"], com))
        placed.append((m, c, mat_mul(mat_mul(leaf["rot"], i), mat_t(leaf["rot"]))))
    mass = sum(p[0] for p in placed)
    com = _scale(_sum_vec(_scale(c, m) for m, c, _ in placed), 1.0 / mass) if mass > 0 else [0.0, 0.0, 0.0]
    inertia = [[0.0] * 3 for _ in range(3)]
    for m, c, i in placed:
        inertia = _mat_add(inertia, _mat_add(i, _shift(m, _sub(c, com))))
    return mass, com, inertia


def _sum_vec(vs):
    out = [0.0, 0.0, 0.0]
    for v in vs:
        out = _add(out, v)
    return out


def _wheel_geometry(part, bundle, rot):
    """``(radius_mm, width_mm)`` of a wheel brick: its cylinder shape, else the
    round bounding box across the axle; the axle is the brick's Y axis turned by
    the instance."""
    for s in part.get("shapes") or []:
        if s.get("type") == "cylinder":
            return float(s["radius"]), float(s["length"])
    rec = bundle["parts"].get(part.get("ldraw")) if part.get("ldraw") else part if part.get("bbox") else None
    if rec is None:
        raise AssemblyError("wheel brick %r needs a cylinder shape or a mesh" % part.get("name"))
    size = [rec["bbox"][1][k] - rec["bbox"][0][k] for k in range(3)]
    # the two equal-ish extents are the diameter; the odd one out is the width
    order = sorted(range(3), key=lambda k: size[k])
    if abs(size[order[1]] - size[order[2]]) <= abs(size[order[0]] - size[order[1]]):
        return (size[order[1]] + size[order[2]]) / 4.0, size[order[0]]
    return (size[order[0]] + size[order[1]]) / 4.0, size[order[2]]


def _look_vector(part, rot):
    axis = ((part.get("frames") or {}).get("look") or {}).get("axis", [0, 0, 1])
    v = mat_vec(rot, axis)
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return [x / n for x in v]


def derive(doc, bundle=None):
    """The chassis of an assembly: a :class:`ChassisSpec`, the build's
    mass properties relative to the axle midpoint, and the brick geoms.
    Raises :class:`AssemblyError` when the wheel roles are missing."""
    bundle = bundle if bundle is not None else bricks.load_bundle()
    if doc.get("format") != "openbricks-assembly/1":
        raise AssemblyError("not an openbricks-assembly/1 file")
    robot = doc.get("robot") or {}
    root = robot.get("root")
    if root not in doc.get("components", {}):
        raise AssemblyError("robot.root must name a component")
    leaves = flatten(doc, root)
    by_path = {leaf["path"]: leaf for leaf in leaves}
    roles = robot.get("roles") or {}

    def role(name):
        p = roles.get(name)
        if not p:
            return None
        leaf = by_path.get(p)
        if leaf is None:
            raise AssemblyError("role %s points at %r, which is not in the build" % (name, p))
        return leaf

    wl, wr = role("wheel_left"), role("wheel_right")
    if wl is None or wr is None:
        raise AssemblyError("both wheel roles (wheel_left, wheel_right) must be assigned")
    part_l, part_r = doc["parts"][wl["part"]], doc["parts"][wr["part"]]
    r_l, w_l = _wheel_geometry(part_l, bundle, wl["rot"])
    r_r, _ = _wheel_geometry(part_r, bundle, wr["rot"])
    _, com_l, _, _ = part_props(part_l, bundle)
    _, com_r, _, _ = part_props(part_r, bundle)
    c_l = _add(wl["pos"], mat_vec(wl["rot"], com_l))
    c_r = _add(wr["pos"], mat_vec(wr["rot"], com_r))
    origin = _scale(_add(c_l, c_r), 0.5)
    track = math.sqrt(sum((c_l[k] - c_r[k]) ** 2 for k in range(3)))
    if track < 1.0:
        raise AssemblyError("the two wheels sit at the same place")
    rel = lambda v: _sub(v, origin)  # noqa: E731
    mm = lambda v: round(v / 1000.0, 6)  # noqa: E731
    mass, com, inertia = mass_properties(doc, bundle, leaves)
    excluded = {roles.get("wheel_left"), roles.get("wheel_right"), roles.get("caster")}
    lo, hi = [math.inf] * 3, [-math.inf] * 3
    body_mass = 0.0
    bricks_out = []
    for leaf in leaves:
        part = doc["parts"][leaf["part"]]
        m, _, _, bbox = part_props(part, bundle)
        corners = [[bbox[i & 1][0], bbox[(i >> 1) & 1][1], bbox[(i >> 2) & 1][2]] for i in range(8)]
        world = [_add(leaf["pos"], mat_vec(leaf["rot"], c)) for c in corners]
        if leaf["path"] not in excluded:
            body_mass += m
            for c in world:
                lo = [min(lo[k], c[k]) for k in range(3)]
                hi = [max(hi[k], c[k]) for k in range(3)]
        centre = [(bbox[0][k] + bbox[1][k]) / 2 for k in range(3)]
        half = [(bbox[1][k] - bbox[0][k]) / 2 for k in range(3)]
        bricks_out.append({
            "path": leaf["path"],
            "part": leaf["part"],
            "ldraw": part.get("ldraw"),
            "pos_m": [mm(v) for v in rel(_add(leaf["pos"], mat_vec(leaf["rot"], centre)))],
            "quat": [round(v, 6) for v in quat_from_mat(leaf["rot"])],
            "half_m": [mm(max(v, 0.25)) for v in half],
            "category": part.get("category", "other"),
        })
    fields = dict(
        wheel_radius=mm(r_l), wheel_width=mm(w_l), wheel_mass=mm(part_l["mass_g"]), axle_length=mm(track),
        body_mass=mm(body_mass),
        pos_x=mm(float((robot.get("spawn") or {}).get("pos_mm", [0, 0])[0])),
        pos_y=mm(float((robot.get("spawn") or {}).get("pos_mm", [0, 0])[1])),
        yaw_deg=float((robot.get("spawn") or {}).get("yaw_deg", 0.0)),
    )
    if r_r and abs(r_r - r_l) > 0.5:
        fields["notes"] = ["the wheels differ in radius (%.1f vs %.1f mm); the left one sizes the drive" % (r_l, r_r)]
    if body_mass > 0 and lo[0] < math.inf:
        fields.update(body_length=mm(hi[0] - lo[0]), body_width=mm(hi[1] - lo[1]), body_height=mm(hi[2] - lo[2]))
    ca = role("caster")
    if ca is not None:
        part = doc["parts"][ca["part"]]
        sphere = next((s for s in part.get("shapes") or [] if s.get("type") == "sphere"), None)
        _, com_c, _, bbox = part_props(part, bundle)
        radius = sphere["radius"] if sphere else min(bbox[1][k] - bbox[0][k] for k in range(3)) / 2.0
        centre = _add(ca["pos"], mat_vec(ca["rot"], com_c))
        fields.update(caster_radius=mm(radius), caster_offset=mm(-rel(centre)[0]), caster_mass=mm(part["mass_g"]))
    cs = role("color_sensor")
    if cs is not None:
        c = rel(cs["pos"])
        look = _look_vector(doc["parts"][cs["part"]], cs["rot"])
        fields.update(color_sensor_x=mm(c[0]), color_sensor_y=mm(c[1]), color_sensor_z=mm(c[2]),
                      color_sensor_yaw=round(math.degrees(math.atan2(look[1], look[0])), 2),
                      color_sensor_pitch=round(math.degrees(math.asin(max(-1.0, min(1.0, look[2])))), 2))
    l1 = role("line_sensor")
    if l1 is not None:
        fields["line_sensor_x"] = mm(rel(l1["pos"])[0])
    l2 = role("line_sensor_2")
    if l2 is not None:
        c = rel(l2["pos"])
        fields.update(line_sensor_2_x=mm(c[0]), line_sensor_2_y=mm(c[1]))
    notes = fields.pop("notes", [])
    spec = ChassisSpec(**fields)
    inertial = {
        "mass_kg": mass / 1000.0,
        "com_m": [mm(v) for v in rel(com)],
        # g·mm² → kg·m²
        "fullinertia": [inertia[0][0] * 1e-9, inertia[1][1] * 1e-9, inertia[2][2] * 1e-9,
                        inertia[0][1] * 1e-9, inertia[0][2] * 1e-9, inertia[1][2] * 1e-9],
    }
    return spec, inertial, bricks_out, notes


def load(path, bundle=None):
    """``derive`` for a file path."""
    with open(path) as fh:
        doc = json.load(fh)
    return derive(doc, bundle)
