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
  hole a dozen different ways (see ``detect_bores``).

Frames: LDraw is X right, Y down, Z back, 1 LDU = 0.4 mm. Records are
X forward, Y left, Z up in millimetres: ``ours = (X, Z, -Y) * 0.4``, a
proper rotation, so face winding (and therefore the sign of the
volume) is preserved.

``python -m openbricks_sim.bricks.ldraw LDRAW_DIR LIST OUT`` rebuilds
the shipped bundle; ``openbricks bricks convert`` converts parts on
demand from the cached library.
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
    on the mesh."""
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
    if re.match(r"^stud[34]", n):
        return "stud_hole"                # the tubes under a brick
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
                        if c == "stud":
                            ymin, ymax = -4.0, 0.0
                        a = child_m @ np.array([0.0, ymin, 0.0]) + child_tr
                        b = child_m @ np.array([0.0, ymax, 0.0]) + child_tr
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
def pack_mesh(tris, q=Q):
    """int16 positions (``1/q`` mm steps: 0.01 mm for bricks, coarser
    for scenery that must fit ±327 m at ``q=0.1``), int8 normals
    smoothed across edges below a 30° crease, and an index buffer —
    base64 strings. ``scale`` records the step so readers need not
    know ``q``."""
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
def detect_bores(tris, radius=2.4, min_votes=12, min_len=2.0):
    """Round bores of the pin radius on a finished mesh. Every wall face
    of a 16-gon bore has its centroid one apothem from the axis along
    its inward normal, so faces vote for axis positions; a real bore
    collects votes from normals all the way round. Technic holes run
    along one of the part's axes. A wall shorter than a module is a
    chamfered ring: 5-8.5 mm means a thick beam's 8 mm hole, under 3 mm
    a thin liftarm's 4 mm one; a plate's 3.2 mm wall is its whole hole.
    Returns ``(kind, a, b)`` segments in the mesh's own frame."""
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
    out = []
    for ax in range(3):
        e = np.zeros(3)
        e[ax] = 1
        mask = np.abs(fn @ e) < 0.08
        if mask.sum() < min_votes:
            continue
        pc = cent[mask] + apothem * fn[mask]
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
            gaps = np.flatnonzero(np.diff(mids) > 7.5)   # one bore's faces lie within a module
            for grp in np.split(np.arange(len(idx)), gaps + 1):
                g = idx[grp]
                if len(g) < min_votes:
                    continue
                angles = np.arctan2(nuv[g][:, 1], nuv[g][:, 0])
                covered = len(set((np.floor((angles + np.pi) / (2 * np.pi) * 16).astype(int) % 16).tolist()))
                if covered < 10:
                    continue                      # a wall or a fillet, not a bore
                lo, hi = float(lo_a[g].min()), float(hi_a[g].max())
                if hi - lo < min_len:
                    continue
                if 5.0 <= hi - lo <= 8.5:         # chamfered rings sit inside the module
                    mid = (lo + hi) / 2
                    lo, hi = mid - 4.0, mid + 4.0
                elif hi - lo < 3.0:               # a thin liftarm's wall between chamfers: a 4 mm hole
                    mid = (lo + hi) / 2
                    lo, hi = mid - 2.0, mid + 2.0
                a = np.zeros(3)
                b = np.zeros(3)
                a[oth] = centre_uv
                b[oth] = centre_uv
                a[ax] = lo
                b[ax] = hi
                out.append(("pin_hole", a, b))
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


# ------------------------------------------------------------- records
def convert_part(lib, builder, number, weights=None):
    """One part number → a brick record, or ``None`` when the library
    has no such part (or it has no faces). Follows ``~Moved to``."""
    path = lib.resolve(number + ".dat")
    if path is None:
        return None
    entry = lib.parse(path)
    moved = re.match(r"~Moved to (\S+)", entry["title"])
    if moved:
        path2 = lib.resolve(moved.group(1) + ".dat")
        if path2:
            path, entry = path2, lib.parse(path2)
    tris_ldu, conns = builder.build(path)
    if len(tris_ldu) == 0:
        return None
    tris = to_ours(tris_ldu)
    vol, com, icom = mass_properties(tris)
    closed = vol > 1.0
    pts = tris.reshape(-1, 3)
    mn, mx = pts.min(axis=0), pts.max(axis=0)
    size = mx - mn
    box_i = np.diag([(size[1] ** 2 + size[2] ** 2) / 12, (size[0] ** 2 + size[2] ** 2) / 12, (size[0] ** 2 + size[1] ** 2) / 12])
    part = {
        "name": entry["title"].lstrip("~=_ ").strip(),
        "ldraw": os.path.basename(path)[:-4],
        "mesh": pack_mesh(tris),
        "bbox": [[round(float(v), 3) for v in mn], [round(float(v), 3) for v in mx]],
        "volume_mm3": round(vol, 2),
        "com": [round(float(v), 3) for v in (com if closed else (mn + mx) / 2)],
        "inertia_per_g": [[round(float(v), 4) for v in row] for row in (icom / vol if closed else box_i)],
        "mass_model": "mesh" if closed else "box",
        "connectors": merge_connectors(conns, detect_bores(tris)),
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
            log("%-8s %-52s tris=%6d conns=%2d vol=%9.1f mm3%s" % (
                num, part["name"][:52], part["mesh"]["tris"], len(part["connectors"]), part["volume_mm3"],
                "" if part["mass_model"] == "mesh" else " OPEN"))
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
        print("usage: python -m openbricks_sim.bricks.ldraw LDRAW_DIR PARTS_LIST OUT[.zlib|.json] [--weights weights.json]", file=sys.stderr)
        return 2
    root, list_path, out_path = argv[0], argv[1], argv[2]
    weights = None
    if "--weights" in argv:
        with open(argv[argv.index("--weights") + 1]) as fh:
            weights = json.load(fh)
    lib = Library(root)
    bundle = convert_parts(lib, read_list(list_path), weights, log=print)
    n = write_bundle(bundle, out_path)
    print("parts: %d, missing: %s, json: %.1f KB" % (len(bundle["parts"]), bundle["missing"], n / 1024))
    return 0


if __name__ == "__main__":         # pragma: no cover
    sys.exit(main())
