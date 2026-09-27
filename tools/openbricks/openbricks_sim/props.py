# SPDX-License-Identifier: MIT
"""The props on a map, and maps of the user's own.

A map is JSON — ``map.json`` in a folder with the files it names (see
:mod:`openbricks_sim.mapfile`). Its ``props`` are the objects on the mat:

* ``{"name", "ldr", "pos", "mass", ["yaw", "pitch", "roll"], ["color"],
  ["fixed"]}`` — an LDraw model, the shipped maps' mission objects;
* ``{"name", "file", "pos", ["yaw", "pitch", "roll"], ["fixed"]}`` — an
  ``openbricks-assembly/1`` document, what the Workbench builds or a
  single brick from its library.

A prop is turned as a Workbench part is: roll about x, then pitch about
y, then yaw about z (``R = Rz(yaw)·Ry(pitch)·Rx(roll)``), about its
origin at ``pos``; it is free unless ``fixed`` (a free one has a free
joint and the physics, or the robot, moves it; a fixed one is welded to
the map). The map editor moves, adds, removes, sticks and unsticks props
by editing the map, so the map stays the source of truth, and saves the
result as a map of the user's own under the data directory, where the
run server lists it beside the shipped ones; a map is exported as one
JSON file and imported from one.

The data directory is the one the sim's markers use:
``$OPENBRICKS_DATA_DIR``, else ``$XDG_DATA_HOME/openbricks``, else
``~/.local/share/openbricks``; maps live in its ``worlds/<alias>/``,
and models added to a map not yet saved in ``props/``.
"""
import copy
import hashlib
import os
import re
import shutil
from pathlib import Path

from openbricks_sim import mapfile


def euler_quat(yaw_deg, pitch_deg=0.0, roll_deg=0.0):
    """The w-x-y-z quaternion of a prop turned ``roll`` about x, then
    ``pitch`` about y, then ``yaw`` about z (the Workbench's order)."""
    import math
    hy, hp, hr = (math.radians(float(a)) / 2.0 for a in (yaw_deg, pitch_deg, roll_deg))
    cy, sy, cp, sp, cr, sr = math.cos(hy), math.sin(hy), math.cos(hp), math.sin(hp), math.cos(hr), math.sin(hr)
    return (cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr,
            sy * cp * cr - cy * sp * sr)


def quat_attr(yaw_deg, pitch_deg=0.0, roll_deg=0.0):
    """The ``quat`` attribute of a body turned so, or nothing when it is
    not turned at all."""
    if not (float(yaw_deg) or float(pitch_deg) or float(roll_deg)):
        return ""
    return ' quat="%.6f %.6f %.6f %.6f"' % euler_quat(yaw_deg, pitch_deg, roll_deg)


class PropError(ValueError):
    """A prop the world does not have, or a map that cannot be saved."""


def home_dir(env=None):
    """The home directory as the sim sees it: ``$HOME``, else
    ``$USERPROFILE`` (Windows), else the working directory."""
    env = os.environ if env is None else env
    return env.get("HOME") or env.get("USERPROFILE") or "."


def _tilde(path, home):
    """A leading ``~`` stands for the home directory (a shell would have
    expanded it; a .env file, a plist or a Windows variable does not)."""
    if path == "~":
        return home
    if path.startswith("~/") or path.startswith("~\\"):
        return os.path.join(home, path[2:])
    return path


def data_dir(env=None, home=None):
    """Where this machine keeps the user's openbricks data:
    ``$OPENBRICKS_DATA_DIR``, else ``$XDG_DATA_HOME/openbricks``, else
    ``~/.local/share/openbricks``, with ``~`` :func:`home_dir` and a
    leading ``~`` in either variable standing for it. The sim's
    ``markers::data_dir`` applies the same rule (``tests/data_dir_cases.json``
    pins both), so what ``openbricks bricks fetch`` keeps is what
    ``openbricks sim`` loads."""
    env = os.environ if env is None else env
    home = home_dir(env) if home is None else home
    explicit = env.get("OPENBRICKS_DATA_DIR")
    if explicit:
        return Path(_tilde(explicit, home))
    xdg = env.get("XDG_DATA_HOME")
    if xdg:
        return Path(_tilde(xdg, home)) / "openbricks"
    return Path(home) / ".local" / "share" / "openbricks"


def user_worlds_dir(env=None, home=None):
    return data_dir(env, home) / "worlds"


# The shipped maps by alias, package-relative — one table for the runtime
# (``robot``) and the CLI. Resolved against the package root, so the
# call site doesn't need to know where it lives on disk.
BUILTIN_WORLDS = {
    "empty":               None,
    "wro-2026-elementary": "worlds/wro_2026_elementary_robot_rockstars/map.json",
    "wro-2026-junior":     "worlds/wro_2026_junior_heritage_heroes/map.json",
    "wro-2026-senior":     "worlds/wro_2026_senior_mosaic_masters/map.json",
    # Small practice scenes for learning / iteration. See
    # ``worlds/<name>/README.md`` for the layout + suggested missions.
    "practice-zones":      "worlds/practice_zones/map.json",
    "practice-walls":      "worlds/practice_walls/map.json",
    "practice-line":       "worlds/practice_line/map.json",
}


def resolve_world(world):
    """Aliases → on-disk path; ``None`` keeps the standalone preview.
    Shipped aliases first, then the user's own maps under the data
    directory (listing them converts one saved before maps were JSON),
    then a path to a ``map.json``."""
    if world is None or world == "empty":
        return None
    if world not in BUILTIN_WORLDS:
        for w in list_user_worlds():
            if w["alias"] == str(world):
                return w["path"]
    if world in BUILTIN_WORLDS:
        rel = BUILTIN_WORLDS[world]
        if rel is None:
            return None
        # Aliases are package-relative — the worlds directory ships
        # inside ``openbricks_sim/`` so the wheel bundles them, and
        # ``Path(__file__).parent`` resolves to the installed package
        # root regardless of how the user installed (pip, pipx,
        # editable, sdist-compile).
        pkg_root = Path(__file__).resolve().parent
        candidate = pkg_root / rel
        if candidate.is_file():
            return str(candidate)
        return world
    return world


def list_user_worlds(env=None, home=None):
    """The user's maps: ``{"alias", "path", "dir", "user": True}`` per
    ``worlds/<alias>/map.json`` under the data directory, by alias. A
    map of the user's own saved before maps were JSON (a ``world.xml``
    alone in its folder) is converted to ``map.json`` the first time it
    is listed; the old file is left where it was."""
    root = user_worlds_dir(env, home)
    if not root.is_dir():
        return []
    out = []
    for d in sorted(root.iterdir()):
        if not d.is_dir():
            continue
        path = d / mapfile.FILE
        legacy = d / "world.xml"
        if not path.is_file() and legacy.is_file():
            mapfile.save(mapfile.from_mjcf(legacy.read_text()), path)
        if path.is_file():
            out.append({"alias": d.name, "path": str(path), "dir": str(d), "user": True})
    return out


def props_in(m):
    """Every prop of a map, in order, as records: tag (``lego_prop`` for
    an LDraw model, ``assembly_prop`` for a document), name, its model
    (``ldr`` or ``file``), pos (m, a tuple), mass (kg, LDraw props), yaw,
    pitch and roll (deg), color, fixed."""
    out = []
    for p in m.get("props", []):
        rec = {
            "tag": "lego_prop" if "ldr" in p else "assembly_prop",
            "name": p["name"],
            "pos": tuple(float(v) for v in p["pos"]),
            "yaw": float(p.get("yaw", 0.0)),
            "pitch": float(p.get("pitch", 0.0)),
            "roll": float(p.get("roll", 0.0)),
            "fixed": bool(p.get("fixed", False)),
        }
        if "ldr" in p:
            rec.update(ldr=p["ldr"], mass=float(p["mass"]), color=p.get("color"))
        else:
            rec["file"] = p["file"]
        out.append(rec)
    return out


def _index(m, name):
    for i, p in enumerate(m.get("props", [])):
        if p["name"] == name:
            return i
    raise PropError("no prop named %r on this map" % (name,))


def _tidy(p):
    """A prop as the map keeps it: each angle only when it turns, fixed
    only when stuck, pos rounded to the micrometre."""
    p = dict(p)
    p["pos"] = [round(float(v), 6) for v in p["pos"]]
    for key in ("yaw", "pitch", "roll"):
        if key in p and not round(float(p[key]), 3):
            del p[key]
        elif key in p:
            p[key] = round(float(p[key]), 3)
    if not p.get("fixed"):
        p.pop("fixed", None)
    return p


def _with(m, i, p):
    m = copy.deepcopy(m)
    m["props"][i] = _tidy(p)
    return m


def with_prop_moved(m, name, x_m, y_m, yaw_deg, pitch_deg=None, roll_deg=None, z_m=None):
    """The map with the prop at ``(x_m, y_m)`` turned ``yaw_deg``; its
    pitch, roll and height set when given, else as the map has them."""
    i = _index(m, name)
    p = dict(m["props"][i])
    z = p["pos"][2] if z_m is None else float(z_m)
    p["pos"] = [float(x_m), float(y_m), z]
    p["yaw"] = float(yaw_deg)
    if pitch_deg is not None:
        p["pitch"] = float(pitch_deg)
    if roll_deg is not None:
        p["roll"] = float(roll_deg)
    return _with(m, i, p)


def with_prop_fixed(m, name, fixed):
    """The map with the prop stuck to it (or free again)."""
    i = _index(m, name)
    return _with(m, i, dict(m["props"][i], fixed=bool(fixed)))


def unique_name(m, base):
    """``base``, or ``base_2``, ``base_3``… — the first not taken."""
    taken = {p["name"] for p in m.get("props", [])}
    if base not in taken:
        return base
    stem = re.sub(r"_\d+$", "", base)
    n = 2
    while "%s_%d" % (stem, n) in taken:
        n += 1
    return "%s_%d" % (stem, n)


def with_prop_added(m, from_name, x_m, y_m, yaw_deg, new_name=None):
    """The map with another prop like ``from_name`` (same model, mass,
    colour, pitch, roll and sticking) at a place and heading, right
    after it. Returns ``(map, name)``."""
    i = _index(m, from_name)
    name = unique_name(m, new_name or from_name)
    src = m["props"][i]
    p = {k: v for k, v in src.items() if k != "note"}
    p.update(name=name, pos=[float(x_m), float(y_m), src["pos"][2]], yaw=float(yaw_deg))
    m = copy.deepcopy(m)
    m["props"].insert(i + 1, _tidy(p))
    return m, name


def with_model_added(m, name, file, x_m, y_m, yaw_deg, z_m=0.0):
    """The map with an assembly document placed as a new, free prop
    after the others. Returns ``(map, name)``."""
    name = unique_name(m, name)
    m = copy.deepcopy(m)
    m.setdefault("props", []).append(_tidy({"name": name, "file": str(file), "pos": [float(x_m), float(y_m), float(z_m)],
                                            "yaw": float(yaw_deg)}))
    return m, name


def with_prop_removed(m, name):
    """The map without the prop."""
    i = _index(m, name)
    m = copy.deepcopy(m)
    del m["props"][i]
    return m


def slug(name):
    """A map name as a directory name: lower case, dashes between words."""
    s = re.sub(r"[^a-z0-9]+", "-", name.strip().lower()).strip("-")
    if not s:
        raise PropError("a map needs a name")
    return s


def stage_file(text, base, ext, env=None, home=None):
    """Keep a model's text under the data directory's ``props/`` until the
    map is saved, named by its content so the same model is kept once.
    Returns the absolute path."""
    root = data_dir(env, home) / "props"
    root.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha256(text.encode()).hexdigest()[:8]
    path = root / ("%s-%s.%s" % (slug(base), digest, ext))
    if not path.exists():
        path.write_text(text)
    return str(path)


def _free_alias(alias, reserved=(), env=None, home=None):
    """``alias``, or the first ``alias-2``, ``alias-3``… that is neither a
    shipped map's nor a map of the user's own already."""
    root = user_worlds_dir(env, home)

    def taken(a):
        return a in reserved or (root / a).exists()

    if not taken(alias):
        return alias
    n = 2
    while taken("%s-%d" % (alias, n)):
        n += 1
    return "%s-%d" % (alias, n)


def save_as(src_dir, m, name, reserved=(), env=None, home=None):
    """Write a map: ``worlds/<slug>/`` under the data directory with the
    source map's files (its artwork, its props' models, its notes) and
    ``map.json`` as given — models referenced by absolute path (added
    since the map was loaded) copied into ``props/`` and referenced from
    there. A name that slugs to one of ``reserved`` (the shipped aliases)
    is refused, so a shipped map is never shadowed; saving over the
    user's own map of that name replaces it, and saving that map over
    itself (its own directory the source) keeps its files and writes the
    map. Returns ``(alias, path)``."""
    alias = slug(name)
    if alias in reserved:
        raise PropError("%r is a shipped map; choose another name" % (alias,))
    dest = user_worlds_dir(env, home) / alias
    src = Path(src_dir) if src_dir else None
    itself = src is not None and dest.is_dir() and src.resolve() == dest.resolve()
    if not itself:
        if dest.exists():
            shutil.rmtree(dest)
        if src is not None and src.is_dir():
            shutil.copytree(src, dest, ignore=shutil.ignore_patterns(mapfile.FILE, "world.xml", "__pycache__"))
        else:
            dest.mkdir(parents=True)
    m = copy.deepcopy(m)
    # models that live outside the map come along, and the map points at the copies
    for p in m.get("props", []):
        key = "ldr" if "ldr" in p else "file"
        ref = Path(p[key])
        if not ref.is_absolute():
            continue
        if not ref.is_file():
            raise PropError("prop %r: its model %s is gone" % (p["name"], ref))
        (dest / "props").mkdir(exist_ok=True)
        target = dest / "props" / ref.name
        n = 2
        while target.exists() and target.read_bytes() != ref.read_bytes():
            target = dest / "props" / mapfile.numbered(ref.name, n)
            n += 1
        if not target.exists():
            shutil.copyfile(ref, target)
        p[key] = "props/" + target.name
    path = dest / mapfile.FILE
    mapfile.save(m, path)
    return alias, str(path)


def export_map(src_dir, m, path):
    """Write the map as it stands, with every file it needs, as one JSON
    file at ``path`` (see :func:`mapfile.pack`)."""
    try:
        mapfile.export_to(m, src_dir, path)
    except mapfile.MapError as e:
        raise PropError(str(e)) from e
    return str(path)


def import_map(path, reserved=(), env=None, home=None):
    """An exported map made a map of the user's own: unpacked into
    ``worlds/<alias>/``, the alias its name (a shipped map's name, or a
    map already there, gets the next free ``-2``, ``-3``…). Returns
    ``(alias, path)``."""
    try:
        obj = mapfile.read_export(path)
    except mapfile.MapError as e:
        raise PropError(str(e)) from e
    base = obj.get("name") or Path(path).stem
    alias = _free_alias(slug(str(base).replace("_", " ")), reserved, env, home)
    dest = user_worlds_dir(env, home) / alias
    dest.mkdir(parents=True)
    try:
        mapfile.unpack(obj, dest)
    except mapfile.MapError as e:
        shutil.rmtree(dest)
        raise PropError(str(e)) from e
    return alias, str(dest / mapfile.FILE)
