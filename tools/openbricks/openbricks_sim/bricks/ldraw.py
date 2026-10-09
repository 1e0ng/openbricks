# SPDX-License-Identifier: MIT
"""LDraw part files → openbricks brick records.

The LDraw parts library (https://www.ldraw.org, CC BY 2.0 / CC BY 4.0)
holds exact geometry for every LEGO part. This module turns a part
number into a *brick record*:

* an exact triangle mesh (quantised positions, crease-smoothed normals,
  an index buffer — the same encoding the workbench page renders);
* mass properties for a uniform solid — volume, centroid and the
  inertia tensor per gram — from the closed mesh by signed tetrahedra,
  so a recorded weight turns into a full inertia tensor;
* connection features: Technic pins, axles, studs and stud tubes come
  from the LDraw connector primitives a part is built from; round pin
  holes are found on the finished mesh, because LDraw authors build a
  hole a dozen different ways (see ``detect_bores``), and so are the
  pins drawn from plain cylinders (``detect_shafts``) and the axle
  holes drawn from plain rectangles (``detect_axle_holes``).

Frames: LDraw is X right, Y down, Z back, 1 LDU = 0.4 mm. Records are
X forward, Y left, Z up in millimetres: ``ours = (X, Z, -Y) * 0.4``, a
proper rotation, so face winding (and therefore the sign of the
volume) is preserved.

``python -m openbricks_sim.bricks.ldraw LDRAW_DIR LIST OUT --weights
weights.json --sets sets.json`` rebuilds the shipped bundle: the curated
list plus every part of the sets, each record stamped with the sets that
hold it; ``openbricks bricks convert`` converts parts on demand from the
cached library.
"""
import base64
import json
import math
import os
import re
import sys
import zlib

import numpy as np

LDU = 0.4
CREASE_COS = math.cos(math.radians(30))
Q = 100.0                       # position quantum: 0.01 mm
RADIUS_MM = {"pin": 2.4, "pin_hole": 2.4, "axle": 2.4, "axle_hole": 2.4,
             "stud": 2.4, "stud_hole": 2.4}
BUNDLE_FORMAT = "openbricks-brick-bundle/1"
SOURCE = "LDraw parts library, CC BY 2.0 / CC BY 4.0 (ldraw.org)"


# ---------------------------------------------------------------- files
class Library:
    """An unpacked LDraw library: ``parts/`` (+ ``parts/s/``) and
    ``p/`` (+ ``p/48/``, ``p/8/``). Files are addressed the way part
    files reference them: ``32278.dat``, ``s\\32013s01.dat``,
    ``48\\4-4cyli.dat``."""

    def __init__(self, root):
        self.root = root
        self.index = {}
        for sub in ("parts", "p"):
            base = os.path.join(root, sub)
            for dirpath, _, files in os.walk(base):
                rel = os.path.relpath(dirpath, base)
                for f in files:
                    if not f.lower().endswith(".dat"):
                        continue
                    key = f.lower() if rel == "." else (rel.replace(os.sep, "/") + "/" + f).lower()
                    self.index.setdefault(key, os.path.join(dirpath, f))
        self.cache = {}

    def resolve(self, name):
        key = name.strip().replace("\\", "/").lower()
        return self.index.get(key)

    def parse(self, path):
        """Lines that matter: refs (type 1), polygons (3, 4), lines (2),
        BFC winding / INVERTNEXT, and the title."""
        if path in self.cache:
            return self.cache[path]
        title = ""
        winding_cw = False
        lines = []
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            first = True
            for raw in fh:
                s = raw.strip()
                if not s:
                    continue
                parts = s.split()
                t = parts[0]
                if t == "0":
                    if first:
                        title = s[1:].strip()
                    if len(parts) >= 2 and parts[1] == "BFC":
                        if "CERTIFY" in parts:
                            winding_cw = "CW" in parts[2:]
                        elif "INVERTNEXT" in parts:
                            lines.append(("invertnext",))
                elif t == "1" and len(parts) >= 15:
                    try:
                        nums = [float(x) for x in parts[2:14]]
                    except ValueError:
                        continue
                    lines.append(("ref", np.array(nums[0:3]), np.array(nums[3:12]).reshape(3, 3), " ".join(parts[14:])))
                elif t in ("3", "4"):
                    n = 3 if t == "3" else 4
                    try:
                        pts = np.array([float(x) for x in parts[2:2 + 3 * n]]).reshape(n, 3)
                    except ValueError:
                        continue
                    lines.append(("poly", pts))
                elif t == "2" and len(parts) >= 8:
                    try:
                        pts = np.array([float(x) for x in parts[2:8]]).reshape(2, 3)
                    except ValueError:
                        continue
                    lines.append(("line", pts))
                first = False
        entry = {"title": title, "cw": winding_cw, "lines": lines, "path": path}
        self.cache[path] = entry
        return entry


# ---------------------------------------------------------- connectors
def classify(name):
    """The LDraw primitive families that ARE connection features (their
    local axis is Y). Pin holes are deliberately absent: they are found
    on the mesh, as are the axle holes no primitive names. A ``stud*``
    primitive that runs +Y from its origin is a tube into the part
    (``stud2s``, ``stud23``), not a stud."""
    n = name.replace("\\", "/").lower().split("/")[-1]
    if not n.endswith(".dat"):
        return None
    n = n[:-4]
    if re.match(r"^(confric|connect)\d*$", n):
        return "pin"                      # male Technic pin segment, tip at -Y
    if n in ("axlehol8", "axle", "axles", "axlebeam", "axleho10"):
        return "axle"                     # male axle, unit height along Y
    if re.match(r"^(axl\dhole|axlehole|axlehol[2-79]|axl\dhol\d+|axlehol0)$", n) or n in ("axlehol4", "axlehol5"):
        return "axle_hole"
    if re.match(r"^stud[34]", n) or n in ("stud2s", "stud23", "stud23d"):
        return "stud_hole"                # the tubes under a brick, and the open tubes that run +Y into a part
    if re.match(r"^stud\d*[a-z]?$", n) and not n.startswith("studp"):
        return "stud"
    return None


# -------------------------------------------------------- tessellation
class Builder:
    """Walks a part's reference tree, accumulating triangles in LDraw
    coordinates with BFC winding resolved, and connector segments."""

    def __init__(self, lib):
        self.lib = lib
        self.extent_cache = {}

    def local_y_extent(self, path):
        """[ymin, ymax] of a primitive's own geometry (faces, else lines)."""
        if path in self.extent_cache:
            return self.extent_cache[path]
        tris, lines_pts = [], []
        self._walk(path, np.zeros(3), np.eye(3), False, tris, None, lines_pts, 0)
        if tris:
            pts = np.concatenate([t.reshape(-1, 3) for t in tris])
        elif lines_pts:
            pts = np.concatenate(lines_pts)
        else:
            pts = np.zeros((1, 3))
        ext = (float(pts[:, 1].min()), float(pts[:, 1].max()))
        self.extent_cache[path] = ext
        return ext

    def build(self, path):
        tris, conns = [], []
        self._walk(path, np.zeros(3), np.eye(3), False, tris, conns, None, 0)
        return (np.array(tris).reshape(-1, 3, 3) if tris else np.zeros((0, 3, 3))), conns

    def _walk(self, path, tr, m, inverted, tris, conns, lines_pts, depth):
        if depth > 40:
            return
        entry = self.lib.parse(path)
        flip = inverted != entry["cw"]
        invert_next = False
        for ln in entry["lines"]:
            kind = ln[0]
            if kind == "invertnext":
                invert_next = True
                continue
            if kind == "poly":
                pts = ln[1] @ m.T + tr
                if flip:
                    pts = pts[::-1]
                if len(pts) == 3:
                    tris.append(pts)
                else:
                    tris.append(pts[[0, 1, 2]])
                    tris.append(pts[[0, 2, 3]])
            elif kind == "line":
                if lines_pts is not None:
                    lines_pts.append(ln[1] @ m.T + tr)
            elif kind == "ref":
                ctr, cm, name = ln[1], ln[2], ln[3]
                child_tr = m @ ctr + tr
                child_m = m @ cm
                child_inv = inverted != invert_next
                if np.linalg.det(cm) < 0:
                    child_inv = not child_inv
                cpath = self.lib.resolve(name)
                child_conns = conns
                if conns is not None:
                    c = classify(name)
                    if c and cpath:
                        child_conns = None      # one feature; don't re-detect its innards
                        ymin, ymax = self.local_y_extent(cpath)
                        if c == "pin":
                            ymax = min(ymax, 0.0)   # the collar sits at +Y; the pin runs to -Y
                        if c == "stud" and ymin >= 0.0:
                            # a male stud runs to -Y; one that runs +Y from its origin is a tube into the
                            # part (stud2s, stud23), whatever its name, and keeps its real depth
                            c = "stud_hole"
                        elif c == "stud":
                            ymin, ymax = -4.0, 0.0    # a stud is 1.6 mm to its mate, however tall the primitive
                        a = child_m @ np.array([0.0, ymin, 0.0]) + child_tr
                        b = child_m @ np.array([0.0, ymax, 0.0]) + child_tr
                        if c == "stud" and np.linalg.norm(b - a) > 1e-9:
                            b = a + (b - a) / np.linalg.norm(b - a) * 4.0   # however the reference scales it
                        conns.append((c, a, b))
                if cpath is None:
                    continue
                self._walk(cpath, child_tr, child_m, child_inv, tris, child_conns, lines_pts, depth + 1)
            invert_next = False


# ------------------------------------------------------ mass properties
_C0 = np.array([[1 / 60, 1 / 120, 1 / 120], [1 / 120, 1 / 60, 1 / 120], [1 / 120, 1 / 120, 1 / 60]])


def mass_properties(tris):
    """Volume (mm³), centroid (mm) and the inertia tensor about the
    centroid for unit density (mm⁵), from signed tetrahedra against the
    origin. Positive volume means outward-facing winding."""
    if len(tris) == 0:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    p1, p2, p3 = tris[:, 0], tris[:, 1], tris[:, 2]
    det = np.einsum("ij,ij->i", p1, np.cross(p2, p3))
    vol = det.sum() / 6.0
    if abs(vol) < 1e-9:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    com = ((p1 + p2 + p3) * det[:, None]).sum(axis=0) / 24.0 / vol
    A = np.stack([p1, p2, p3], axis=2)
    cov = np.einsum("nij,jk,nlk,n->il", A, _C0, A, det)
    I0 = np.trace(cov) * np.eye(3) - cov
    Icom = I0 - vol * (np.dot(com, com) * np.eye(3) - np.outer(com, com))
    return float(vol), com, Icom


# ---------------------------------------------------------- mesh packing
def mesh_quantum(tris):
    """The step :func:`pack_mesh` packs a part at: ``Q`` (0.01 mm) when
    the mesh fits int16 at it, else ten times coarser for every ten it
    is over — a 47-stud hose spans 376 mm and packs at 0.1 mm. The
    step travels in the record (``scale``) and is said in the
    conversion log, never applied quietly."""
    q = Q
    span = float(np.abs(tris).max()) if len(tris) else 0.0
    while span * q > 32767:
        q /= 10.0
    return q


def coarse_note(part):
    """What a log line says of a record packed coarser than 0.01 mm (see
    :func:`mesh_quantum`); nothing for the others."""
    step = part["mesh"]["scale"]
    return "" if step == 1.0 / Q else " packed at %g mm steps (too long for 0.01 mm)" % step


def pack_mesh(tris, q=Q):
    """int16 positions (``1/q`` mm steps: 0.01 mm for bricks, coarser
    for scenery that must fit ±327 m at ``q=0.1`` and for the few parts
    longer than int16 holds at 0.01 mm, see :func:`mesh_quantum`), int8
    normals smoothed across edges below a 30° crease, and an index
    buffer — base64 strings. ``scale`` records the step so readers
    need not know ``q``."""
    p1, p2, p3 = tris[:, 0], tris[:, 1], tris[:, 2]
    fn = np.cross(p2 - p1, p3 - p1)
    ln = np.linalg.norm(fn, axis=1)
    keep = ln > 1e-9
    tris, fn, ln = tris[keep], fn[keep], ln[keep]
    fn = fn / ln[:, None]
    n = len(tris)
    corners = tris.reshape(-1, 3)
    qpos = np.round(corners * q).astype(np.int64)
    if len(qpos) and np.abs(qpos).max() > 32767:
        raise ValueError("the mesh spans %.0f mm, more than int16 holds at %g mm steps; pack it with a coarser q"
                         % (np.abs(corners).max(), 1.0 / q))
    keys = qpos[:, 0] * 4000037 * 4000037 + qpos[:, 1] * 4000037 + qpos[:, 2]
    order = np.argsort(keys, kind="stable")
    sorted_keys = keys[order]
    starts = np.r_[0, np.flatnonzero(np.diff(sorted_keys)) + 1]
    ends = np.r_[starts[1:], len(order)]
    face_of_corner = np.repeat(np.arange(n), 3)
    normals = np.zeros((len(corners), 3))
    for s, e in zip(starts, ends):
        idx = order[s:e]
        fns = fn[face_of_corner[idx]]
        w = (fns @ fns.T > CREASE_COS).astype(float)
        nm = w @ fns
        nl = np.linalg.norm(nm, axis=1)
        nl[nl < 1e-9] = 1
        normals[idx] = nm / nl[:, None]
    qn = np.clip(np.round(normals * 127), -127, 127).astype(np.int8)
    combo = np.concatenate([qpos, qn.astype(np.int64) + 128], axis=1)
    uniq, inv = np.unique(combo, axis=0, return_inverse=True)
    pos16 = uniq[:, :3].astype("<i2")
    nrm8 = (uniq[:, 3:] - 128).astype(np.int8)
    inv = inv.reshape(-1)
    idx32 = len(uniq) >= 65536
    idx = inv.astype("<u4") if idx32 else inv.astype("<u2")
    return {
        "verts": int(len(uniq)),
        "tris": int(n),
        "pos": base64.b64encode(pos16.tobytes()).decode(),
        "nrm": base64.b64encode(nrm8.tobytes()).decode(),
        "idx": base64.b64encode(idx.tobytes()).decode(),
        "idx32": bool(idx32),
        "scale": 1.0 / q,
    }


def unpack_mesh(mesh):
    """The inverse of ``pack_mesh``: positions (mm, float64, N×3),
    normals (N×3) and the triangle index array (M×3)."""
    pos = np.frombuffer(base64.b64decode(mesh["pos"]), dtype="<i2").reshape(-1, 3) * mesh.get("scale", 1.0 / Q)
    nrm = np.frombuffer(base64.b64decode(mesh["nrm"]), dtype=np.int8).reshape(-1, 3) / 127.0
    idx = np.frombuffer(base64.b64decode(mesh["idx"]), dtype="<u4" if mesh.get("idx32") else "<u2").reshape(-1, 3)
    return pos, nrm, idx


def to_ours(pts_ldu):
    """LDraw (X right, Y down, Z back) → ours (X forward, Y left, Z up), mm."""
    out = np.empty_like(pts_ldu)
    out[..., 0] = pts_ldu[..., 0]
    out[..., 1] = pts_ldu[..., 2]
    out[..., 2] = -pts_ldu[..., 1]
    return out * LDU


# ------------------------------------------------------------ pin holes
def _round_runs(tris, radius, outward, min_votes=12):
    """Round features of ``radius`` on a finished mesh, running along one
    of the part's axes: ``(axis index, centre in the other two, lo, hi)``
    per run. Every wall face of a 16-gon cylinder has its centroid one
    apothem from the axis — inward along its normal in a bore, outward
    in a shaft — so faces vote for axis positions, and a real one
    collects votes from normals all the way round (10 of 16 sectors).
    One run's faces lie within a module of each other. Each run ends with
    how many of the 16 sectors its normals cover."""
    if len(tris) == 0:
        return []
    p1, p2, p3 = tris[:, 0], tris[:, 1], tris[:, 2]
    fn = np.cross(p2 - p1, p3 - p1)
    ln = np.linalg.norm(fn, axis=1)
    ok = ln > 1e-9
    fn = fn[ok] / ln[ok][:, None]
    T = tris[ok]
    cent = T.mean(axis=1)
    apothem = radius * math.cos(math.pi / 16)
    runs = []
    for ax in range(3):
        e = np.zeros(3)
        e[ax] = 1
        mask = np.abs(fn @ e) < 0.08
        if mask.sum() < min_votes:
            continue
        pc = cent[mask] + (-apothem if outward else apothem) * fn[mask]
        oth = [i for i in range(3) if i != ax]
        uv = pc[:, oth]
        nuv = fn[mask][:, oth]
        lo_a = T[mask][:, :, ax].min(axis=1)
        hi_a = T[mask][:, :, ax].max(axis=1)
        keys = np.round(uv / 0.5).astype(np.int64)
        kk = keys[:, 0] * 1000003 + keys[:, 1]
        order = np.argsort(kk, kind="stable")
        sk = kk[order]
        starts = np.r_[0, np.flatnonzero(np.diff(sk)) + 1]
        ends = np.r_[starts[1:], len(order)]
        for s0, e0 in zip(starts, ends):
            idx = order[s0:e0]
            if len(idx) < min_votes:
                continue
            centre_uv = uv[idx].mean(axis=0)
            mids = (lo_a[idx] + hi_a[idx]) / 2
            aorder = np.argsort(mids)
            idx = idx[aorder]
            mids = mids[aorder]
            gaps = np.flatnonzero(np.diff(mids) > 7.5)   # one run's faces lie within a module
            for grp in np.split(np.arange(len(idx)), gaps + 1):
                g = idx[grp]
                if len(g) < min_votes:
                    continue
                angles = np.arctan2(nuv[g][:, 1], nuv[g][:, 0])
                covered = len(set((np.floor((angles + np.pi) / (2 * np.pi) * 16).astype(int) % 16).tolist()))
                if covered < 10:
                    continue                      # a wall or a fillet, not a round feature
                runs.append((ax, oth, centre_uv, float(lo_a[g].min()), float(hi_a[g].max()), covered))
    return runs


def _segment(kind, ax, oth, centre_uv, lo, hi):
    a = np.zeros(3)
    b = np.zeros(3)
    a[oth] = centre_uv
    b[oth] = centre_uv
    a[ax] = lo
    b[ax] = hi
    return (kind, a, b)


def _along_a_line(a, b, lines, within):
    """Whether segment ``a``-``b`` lies along one of ``lines`` (``(p,
    q)`` pairs): parallel, within ``within`` mm of its line and
    overlapping its extent."""
    u = (b - a) / np.linalg.norm(b - a)
    for p, q in lines:
        if np.linalg.norm(q - p) < 1e-9:
            continue                      # a primitive with no extent names no line
        v = (q - p) / np.linalg.norm(q - p)
        d = a - p
        if abs(abs(float(u @ v)) - 1) > 0.02 or np.linalg.norm(d - (d @ v) * v) > within:
            continue
        t0, t1 = sorted([float((a - p) @ v), float((b - p) @ v)])
        if t0 < float((q - p) @ v) and t1 > 0.0:
            return True
    return False


def detect_bores(tris, radius=2.4, min_votes=12, min_len=2.0, axle_holes=()):
    """Round bores of the pin radius on a finished mesh (the faces vote
    inward, see :func:`_round_runs`). Technic holes run along one of the
    part's axes. A wall shorter than a module is a chamfered ring: 5-8.5
    mm means a thick beam's 8 mm hole, under 3 mm a thin liftarm's 4 mm
    one; a plate's 3.2 mm wall is its whole hole. The rounded ends of an
    axle hole's four arms sit at the pin radius too and cover 12 of the
    16 sectors: a bore short of all 16 along one of ``axle_holes`` (``(a,
    b)`` pairs, ours) is that axle hole, not a pin hole. Returns
    ``(kind, a, b)`` segments in the mesh's own frame."""
    out = []
    for ax, oth, centre_uv, lo, hi, covered in _round_runs(tris, radius, outward=False, min_votes=min_votes):
        if hi - lo < min_len:
            continue
        if covered < 16:
            kind, a, b = _segment("pin_hole", ax, oth, centre_uv, lo, hi)
            if _along_a_line(a, b, axle_holes, 1.0):
                continue
        if 5.0 <= hi - lo <= 8.5:         # chamfered rings sit inside the module
            mid = (lo + hi) / 2
            lo, hi = mid - 4.0, mid + 4.0
        elif hi - lo < 3.0:               # a thin liftarm's wall between chamfers: a 4 mm hole
            mid = (lo + hi) / 2
            lo, hi = mid - 2.0, mid + 2.0
        out.append(_segment("pin_hole", ax, oth, centre_uv, lo, hi))
    return out


def detect_shafts(tris, radius=2.4, min_votes=12, min_len=4.0):
    """Round pin shafts of the pin radius on a finished mesh — the male
    of :func:`detect_bores` (the faces vote outward). The pins LDraw
    draws from plain cylinders and rib primitives — the "Type 2" pins
    61332, 42924, 39888, the WRO set's commonest — name no connector
    primitive :func:`classify` knows, and had no pins at all. A pin's
    halves run through its collar as one shaft, cut into module segments
    by :func:`merge_connectors`; a stud (1.6 mm) is shorter than
    ``min_len``. Returns ``(kind, a, b)`` segments in the mesh's own
    frame."""
    return [_segment("pin", ax, oth, centre_uv, lo, hi)
            for ax, oth, centre_uv, lo, hi, _ in _round_runs(tris, radius, outward=True, min_votes=min_votes)
            if hi - lo >= min_len]


def _clusters(values, gap):
    """The means of ``values`` grouped where sorted neighbours lie within
    ``gap`` of each other."""
    if len(values) == 0:
        return []
    v = np.sort(values)
    return [float(g.mean()) for g in np.split(v, np.flatnonzero(np.diff(v) > gap) + 1)]


AXLE_ARM_MM = 0.8          # an axle cross's arms are 2 LDU either side of their centre line
AXLE_REACH_MM = 2.4        # and reach the pin radius


def detect_axle_holes(tris, min_len=2.0):
    """Axle holes on a finished mesh, for the parts that draw one from
    plain rectangles and arcs rather than an axle-hole primitive (the
    24-tooth gear 3648). A cross-shaped hole along one of the part's
    axes, its arms along the other two, has eight flat side walls facing
    into it, each :data:`AXLE_ARM_MM` from the hole's centre along its
    normal and between that and :data:`AXLE_REACH_MM` out along its arm,
    and rounded arm ends at :data:`AXLE_REACH_MM` facing back at the
    centre (3648's two side arms open into the gear's web, so two ends
    are enough). A hole is a centre all eight walls (both faces of both
    arms, both ways) and two arm ends vote for over the same stretch of
    the axis; a male axle's walls face out, and vote 1.6 mm apart. One
    hole's faces lie within a module of each other. Returns ``(kind, a,
    b)`` segments in the mesh's own frame."""
    if len(tris) == 0:
        return []
    p1, p2, p3 = tris[:, 0], tris[:, 1], tris[:, 2]
    fn = np.cross(p2 - p1, p3 - p1)
    ln = np.linalg.norm(fn, axis=1)
    ok = ln > 1e-9
    fn = fn[ok] / ln[ok][:, None]
    T = tris[ok]
    cent = T.mean(axis=1)
    lo_all = T.min(axis=1)
    hi_all = T.max(axis=1)
    out = []
    for ax in range(3):
        oth = [i for i in range(3) if i != ax]
        side_on = np.abs(fn[:, ax]) < 0.08
        walls = []                        # per other axis: faces facing along it, and the centre each votes for
        for k in oth:
            m = np.flatnonzero(side_on & (np.abs(fn[:, k]) > 0.98))
            walls.append((m, cent[m, k] + AXLE_ARM_MM * np.sign(fn[m, k])))
        for cu in _clusters(walls[0][1], 0.1):
            for cv in _clusters(walls[1][1], 0.1):
                centre = {oth[0]: cu, oth[1]: cv}
                faces, kinds = [], []
                for j, k in enumerate(oth):
                    other = oth[1 - j]
                    m, vote = walls[j]
                    reach = cent[m, other] - centre[other]
                    near = (np.abs(vote - centre[k]) < 0.15) & (np.abs(reach) > AXLE_ARM_MM - 0.2) \
                        & (np.abs(reach) < AXLE_REACH_MM + 0.2)
                    faces.append(m[near])
                    # which wall: the axis it faces along, which way, and which arm it lines
                    kinds.append(j * 4 + (fn[m[near], k] > 0) * 2 + (reach[near] > 0))
                if len(set(np.concatenate(kinds).tolist())) < 8:
                    continue
                for j, k in enumerate(oth):
                    other = oth[1 - j]
                    off = cent[:, k] - centre[k]
                    across = np.abs(cent[:, other] - centre[other])
                    for way in (1, -1):           # the rounded end of the arm along k, facing back at the centre
                        end = np.flatnonzero(side_on & (fn[:, k] * way < -0.9) & (off * way > AXLE_REACH_MM - 0.4)
                                             & (off * way < AXLE_REACH_MM + 0.1) & (across < AXLE_ARM_MM))
                        faces.append(end)
                        kinds.append(np.full(len(end), 8 + j * 2 + (way > 0)))
                g = np.concatenate(faces)
                kind_of = np.concatenate(kinds)
                mids = (lo_all[g, ax] + hi_all[g, ax]) / 2
                order = np.argsort(mids)
                gaps = np.flatnonzero(np.diff(mids[order]) > 7.5)
                for grp in np.split(order, gaps + 1):
                    here = set(kind_of[grp].tolist())
                    if len([x for x in here if x < 8]) < 8 or len([x for x in here if x >= 8]) < 2:
                        continue                  # not every wall, or too few ends, along this stretch
                    lo, hi = float(lo_all[g[grp], ax].min()), float(hi_all[g[grp], ax].max())
                    if hi - lo >= min_len:
                        out.append(_segment("axle_hole", ax, oth, np.array([cu, cv]), lo, hi))
    return out


def merge_connectors(conns, extra_ours=()):
    """One physical feature is often built from several primitives:
    merge collinear pieces whose extents overlap or touch, then cut pins
    and pin holes into 8 mm (one module) segments so every segment mates
    one pin half. ``conns`` are in LDraw coordinates, ``extra_ours``
    already in ours."""
    items = [(k, to_ours(a), to_ours(b)) for k, a, b in conns] + list(extra_ours)
    out = []
    for kind, a, b in items:
        L = float(np.linalg.norm(b - a))
        if L < 0.05:
            continue
        u = (b - a) / L
        merged = False
        for o in out:
            if o["kind"] != kind:
                continue
            if abs(abs(float(u @ o["u"])) - 1) > 0.02:
                continue
            d = a - o["a"]
            if np.linalg.norm(d - (d @ o["u"]) * o["u"]) > 0.3:
                continue
            t0, t1 = sorted([float((a - o["a"]) @ o["u"]), float((b - o["a"]) @ o["u"])])
            lo, hi = 0.0, float((o["b"] - o["a"]) @ o["u"])
            if t0 > hi + 7.5 or t1 < lo - 7.5:
                continue
            nlo, nhi = min(lo, t0), max(hi, t1)
            base = o["a"].copy()
            o["a"] = base + o["u"] * nlo
            o["b"] = base + o["u"] * nhi
            merged = True
            break
        if not merged:
            out.append({"kind": kind, "a": a.copy(), "b": b.copy(), "u": u})
    result = []
    for o in out:
        L = float(np.linalg.norm(o["b"] - o["a"]))
        if o["kind"] == "pin_hole" and L < 3.0:
            continue
        pieces = [(o["a"], o["b"])]
        if o["kind"] in ("pin_hole", "pin") and L > 8.5:
            n = int(round(L / 8.0))
            step = (o["b"] - o["a"]) / n
            pieces = [(o["a"] + step * i, o["a"] + step * (i + 1)) for i in range(n)]
        for a, b in pieces:
            c = (a + b) / 2
            result.append({
                "kind": o["kind"],
                "centre": [round(float(v), 3) for v in c],
                "axis": [round(float(v), 5) for v in o["u"]],
                "length": round(float(np.linalg.norm(b - a)), 3),
                "r": RADIUS_MM[o["kind"]],
            })
    return result


def unnamed_pins(conns, tris):
    """The pins a part has that its LDraw primitives do not name: none
    when :func:`classify` found any pin (the mesh then only adds noise —
    a collar, a stop bush, a stud end), else the part's round shafts
    (:func:`detect_shafts`) except along an axle's line (a round axle end
    is not a pin). ``conns`` are the named connectors, in LDraw
    coordinates; the shafts come back in ours."""
    if any(kind == "pin" for kind, _, _ in conns):
        return []
    axles = [(to_ours(a), to_ours(b)) for kind, a, b in conns if kind == "axle"]

    def on_an_axle(seg):
        _, a, b = seg
        u = (b - a) / np.linalg.norm(b - a)
        for p, q in axles:
            v = (q - p) / np.linalg.norm(q - p)
            d = a - p
            if abs(abs(float(u @ v)) - 1) < 0.02 and np.linalg.norm(d - (d @ v) * v) < 0.5:
                return True
        return False

    return [seg for seg in detect_shafts(tris) if not on_an_axle(seg)]


def unnamed_axle_holes(conns, tris):
    """The axle holes a part has that its LDraw primitives do not name
    (:func:`detect_axle_holes`), except along a named one's line.
    ``conns`` are the named connectors, in LDraw coordinates; the holes
    come back in ours."""
    named = [(to_ours(a), to_ours(b)) for kind, a, b in conns if kind == "axle_hole"]
    return [seg for seg in detect_axle_holes(tris) if not _along_a_line(seg[1], seg[2], named, 0.3)]


# ------------------------------------------------------------- records
def convert_part(lib, builder, number, weights=None):
    """One part number → a brick record, or ``None`` when the library
    has no such part (or it has no faces). Follows ``~Moved to``, hop by
    hop (an alias may point at an alias), to the part that is there."""
    path = lib.resolve(number + ".dat")
    if path is None:
        return None
    entry = lib.parse(path)
    for _ in range(8):
        moved = re.match(r"~Moved to (\S+)", entry["title"])
        if not moved:
            break
        path2 = lib.resolve(moved.group(1) + ".dat")
        if not path2 or path2 == path:
            break
        path, entry = path2, lib.parse(path2)
    tris_ldu, conns = builder.build(path)
    if len(tris_ldu) == 0:
        return None
    tris = to_ours(tris_ldu)
    found_axle_holes = unnamed_axle_holes(conns, tris)
    axle_holes = [(to_ours(a), to_ours(b)) for kind, a, b in conns if kind == "axle_hole"] + \
        [(a, b) for _, a, b in found_axle_holes]
    vol, com, icom = mass_properties(tris)
    closed = vol > 1.0
    pts = tris.reshape(-1, 3)
    mn, mx = pts.min(axis=0), pts.max(axis=0)
    size = mx - mn
    box_i = np.diag([(size[1] ** 2 + size[2] ** 2) / 12, (size[0] ** 2 + size[2] ** 2) / 12, (size[0] ** 2 + size[1] ** 2) / 12])
    part = {
        "name": entry["title"].lstrip("~=_ ").strip(),
        "ldraw": os.path.basename(path)[:-4],
        "mesh": pack_mesh(tris, mesh_quantum(tris)),
        "bbox": [[round(float(v), 3) for v in mn], [round(float(v), 3) for v in mx]],
        "volume_mm3": round(vol, 2),
        "com": [round(float(v), 3) for v in (com if closed else (mn + mx) / 2)],
        "inertia_per_g": [[round(float(v), 4) for v in row] for row in (icom / vol if closed else box_i)],
        "mass_model": "mesh" if closed else "box",
        "connectors": merge_connectors(conns, detect_bores(tris, axle_holes=axle_holes) + found_axle_holes
                                       + unnamed_pins(conns, tris)),
    }
    w = (weights or {}).get(number) or (weights or {}).get(part["ldraw"])
    if w:
        part["mass_g"] = w["g"]
        part["source"] = "vendor"
        part["source_note"] = "BrickLink " + number + ": " + str(w["g"]) + " g" + (" (" + w["dims"] + ")" if w.get("dims") else "")
        if closed:
            part["density_g_cm3"] = round(w["g"] / (vol / 1000.0), 3)
    else:
        part["mass_g"] = round(vol / 1000.0 * 1.05, 2) if closed else 1.0
        part["source"] = "placeholder"
        part["source_note"] = ("mass estimated from the LDraw volume at 1.05 g/cm3 (ABS); weigh it"
                               if closed else "open mesh: mass placeholder, weigh it")
    return part


def convert_parts(lib, numbers, weights=None, log=None):
    """Many part numbers → a bundle dict. Missing numbers are listed
    under ``"missing"`` rather than raising, so one typo does not stop a
    long list."""
    builder = Builder(lib)
    bundle = {"format": BUNDLE_FORMAT, "source": SOURCE, "units": {"length": "mm", "mass": "g"}, "parts": {}, "missing": []}
    for num in numbers:
        part = convert_part(lib, builder, num, weights)
        if part is None:
            bundle["missing"].append(num)
            continue
        bundle["parts"][num] = part
        if log:
            log("%-8s %-52s tris=%6d conns=%2d vol=%9.1f mm3%s%s" % (
                num, part["name"][:52], part["mesh"]["tris"], len(part["connectors"]), part["volume_mm3"],
                "" if part["mass_model"] == "mesh" else " OPEN", coarse_note(part)))
    return bundle


def read_sets(path):
    """The sets file: ``{set id: {name, year, pieces, parts: {LDraw
    number: how many}, aliases: {inventory number: LDraw number}}}``;
    a top-level ``source`` string says where the inventories came from."""
    with open(path) as fh:
        data = json.load(fh)
    return {k: v for k, v in data.items() if isinstance(v, dict)}


def set_numbers(sets):
    """Every LDraw number the sets hold, sorted."""
    return sorted({num for s in sets.values() for num in s.get("parts", {})})


def apply_sets(bundle, sets):
    """Stamp each record with the sets that hold it (``sets``: set id →
    how many) and the numbers it goes by in their inventories where LDraw
    names it differently (``aliases``); the bundle lists the sets
    themselves under ``sets``. A set part the bundle lacks joins
    ``missing``, so a set is never quietly incomplete."""
    bundle["sets"] = {sid: {"name": s["name"], "year": s["year"], "pieces": s["pieces"]} for sid, s in sets.items()}
    for sid, s in sets.items():
        for num, qty in s.get("parts", {}).items():
            rec = bundle["parts"].get(num)
            if rec is None:
                if num not in bundle["missing"]:
                    bundle["missing"].append(num)
                continue
            rec.setdefault("sets", {})[sid] = qty
        for other, num in s.get("aliases", {}).items():
            rec = bundle["parts"].get(num)
            if rec is not None and other not in rec.setdefault("aliases", []):
                rec["aliases"].append(other)
    for rec in bundle["parts"].values():
        if "aliases" in rec:
            rec["aliases"].sort()
    return bundle


def read_colors(path):
    """The colours file ``openbricks_sim.bricks.rebrickable`` writes."""
    with open(path) as fh:
        return json.load(fh)


def apply_colors(bundle, colors):
    """Stamp each record with the colours its part comes in (``colors``:
    colour id → the LEGO element numbers of the part in that colour) and
    give the bundle the palette those colours draw with (``colors``:
    colour id → name, rgb, trans). A part the file knows nothing about
    keeps no colours and draws in its category's."""
    bundle["colors"] = dict(colors.get("palette", {}))
    for num, rec in bundle["parts"].items():
        entry = colors.get("parts", {}).get(num)
        if entry:
            rec["colors"] = {cid: list(els) for cid, els in entry.items()}
    return bundle


def read_list(path):
    """Part numbers from a list file: one per line, ``#`` comments."""
    numbers = []
    with open(path) as fh:
        for line in fh:
            line = line.split("#", 1)[0].strip()
            if line:
                numbers.append(line)
    return numbers


def write_bundle(bundle, path):
    """``.zlib`` → compressed JSON (what the wheel ships); anything else
    → plain JSON."""
    raw = json.dumps(bundle, separators=(",", ":")).encode()
    if path.endswith(".zlib"):
        with open(path, "wb") as fh:
            fh.write(zlib.compress(raw, 9))
    else:
        with open(path, "w") as fh:
            fh.write(raw.decode())
    return len(raw)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) < 3:
        print("usage: python -m openbricks_sim.bricks.ldraw LDRAW_DIR PARTS_LIST OUT[.zlib|.json] [--weights weights.json] [--sets sets.json] [--colors colors.json]", file=sys.stderr)
        return 2
    root, list_path, out_path = argv[0], argv[1], argv[2]
    weights = None
    if "--weights" in argv:
        with open(argv[argv.index("--weights") + 1]) as fh:
            weights = json.load(fh)
    sets = read_sets(argv[argv.index("--sets") + 1]) if "--sets" in argv else {}
    numbers = read_list(list_path)
    numbers += [n for n in set_numbers(sets) if n not in numbers]
    lib = Library(root)
    bundle = apply_sets(convert_parts(lib, numbers, weights, log=print), sets)
    if "--colors" in argv:
        bundle = apply_colors(bundle, read_colors(argv[argv.index("--colors") + 1]))
    n = write_bundle(bundle, out_path)
    print("parts: %d, missing: %s, json: %.1f KB" % (len(bundle["parts"]), bundle["missing"], n / 1024))
    return 0


if __name__ == "__main__":         # pragma: no cover
    sys.exit(main())
