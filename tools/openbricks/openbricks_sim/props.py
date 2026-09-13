# SPDX-License-Identifier: MIT
"""The props on a map, and maps of the user's own.

A prop is a placeholder in a world's MJCF that the loader expands into
a body:

* ``<lego_prop name="…" ldr="…" pos="x y z" mass="…" [yaw="deg"]
  [color="…"] [fixed="true"]/>`` — an LDraw model, the shipped maps'
  mission objects;
* ``<assembly_prop name="…" file="…" pos="x y z" [yaw="deg"]
  [fixed="true"]/>`` — an ``openbricks-assembly/1`` document, what the
  Workbench builds or a single brick from its library.

A prop is free unless ``fixed``: a free one has a free joint and the
physics (or the robot) moves it; a fixed one is welded to the map. The
map editor moves, adds, removes, sticks and unsticks props by
rewriting these placeholders in the world text, so the text stays the
source of truth, and saves the result as a new map under the data
directory, where the run server lists it beside the shipped ones.

The data directory is the one the sim's markers use:
``$OPENBRICKS_DATA_DIR``, else ``$XDG_DATA_HOME/openbricks``, else
``~/.local/share/openbricks``; maps live in its ``worlds/<alias>/``,
and models added to a map not yet saved in ``props/``.
"""
import hashlib
import os
import re
import shutil
from pathlib import Path

# Attribute order is fixed: name, ldr/file, pos, mass, then the optional
# yaw, color and fixed. ``world.py`` expands matches of these same
# patterns.
PROP_RE = re.compile(
    r'<lego_prop\s+name="(?P<name>[^"]+)"\s+'
    r'ldr="(?P<ldr>[^"]+)"\s+'
    r'pos="(?P<pos>[^"]+)"\s+'
    r'mass="(?P<mass>[^"]+)"'
    r'(?:\s+yaw="(?P<yaw>[^"]+)")?'
    r'(?:\s+color="(?P<color>[^"]+)")?'
    r'(?:\s+fixed="(?P<fixed>[^"]+)")?'
    r'\s*/>',
    re.DOTALL)
MODEL_RE = re.compile(
    r'<assembly_prop\s+name="(?P<name>[^"]+)"\s+'
    r'file="(?P<file>[^"]+)"\s+'
    r'pos="(?P<pos>[^"]+)"'
    r'(?:\s+yaw="(?P<yaw>[^"]+)")?'
    r'(?:\s+fixed="(?P<fixed>[^"]+)")?'
    r'\s*/>',
    re.DOTALL)


class PropError(ValueError):
    """A prop the world does not have, or a map that cannot be saved."""


def data_dir(env=None, home=None):
    """Where this machine keeps the user's openbricks data."""
    env = os.environ if env is None else env
    explicit = env.get("OPENBRICKS_DATA_DIR")
    if explicit:
        return Path(explicit)
    xdg = env.get("XDG_DATA_HOME")
    if xdg:
        return Path(xdg) / "openbricks"
    return Path(home if home is not None else Path.home()) / ".local" / "share" / "openbricks"


def user_worlds_dir(env=None, home=None):
    return data_dir(env, home) / "worlds"


def list_user_worlds(env=None, home=None):
    """The user's maps: ``{"alias", "path", "dir", "user": True}`` per
    ``worlds/<alias>/world.xml`` under the data directory, by alias."""
    root = user_worlds_dir(env, home)
    if not root.is_dir():
        return []
    out = []
    for d in sorted(root.iterdir()):
        path = d / "world.xml"
        if d.is_dir() and path.is_file():
            out.append({"alias": d.name, "path": str(path), "dir": str(d), "user": True})
    return out


def _floats(text, name, n):
    parts = text.split()
    try:
        values = tuple(float(t) for t in parts)
    except ValueError:
        values = ()
    if len(values) != n:
        raise PropError("prop %r pos must be %d floats; got %r" % (name, n, text))
    return values


def _flag(text):
    return str(text).strip().lower() in ("true", "1", "yes")


def props_in(world_xml):
    """Every prop placeholder in the text, in order: tag, name, its model
    (``ldr`` or ``file``), pos (m), mass (kg, LDraw props), yaw (deg),
    color, fixed, and its span in the text."""
    out = []
    for m in PROP_RE.finditer(world_xml):
        out.append({
            "tag": "lego_prop",
            "name": m.group("name"),
            "ldr": m.group("ldr"),
            "pos": _floats(m.group("pos"), m.group("name"), 3),
            "mass": float(m.group("mass")),
            "yaw": float(m.group("yaw")) if m.group("yaw") is not None else 0.0,
            "color": m.group("color"),
            "fixed": _flag(m.group("fixed")) if m.group("fixed") is not None else False,
            "span": m.span(),
        })
    for m in MODEL_RE.finditer(world_xml):
        out.append({
            "tag": "assembly_prop",
            "name": m.group("name"),
            "file": m.group("file"),
            "pos": _floats(m.group("pos"), m.group("name"), 3),
            "yaw": float(m.group("yaw")) if m.group("yaw") is not None else 0.0,
            "fixed": _flag(m.group("fixed")) if m.group("fixed") is not None else False,
            "span": m.span(),
        })
    out.sort(key=lambda p: p["span"][0])
    return out


def _find(world_xml, name):
    for p in props_in(world_xml):
        if p["name"] == name:
            return p
    raise PropError("no prop named %r on this map" % (name,))


def _element(p):
    """The placeholder text for a prop record (yaw written only when it
    turns, colour only when set, fixed only when stuck)."""
    if p["tag"] == "lego_prop":
        text = '<lego_prop name="%s" ldr="%s" pos="%.5f %.5f %.5f" mass="%g"' % (
            p["name"], p["ldr"], p["pos"][0], p["pos"][1], p["pos"][2], p["mass"])
    else:
        text = '<assembly_prop name="%s" file="%s" pos="%.5f %.5f %.5f"' % (
            p["name"], p["file"], p["pos"][0], p["pos"][1], p["pos"][2])
    yaw = round(float(p["yaw"]), 3)
    if yaw:
        text += ' yaw="%g"' % yaw
    if p["tag"] == "lego_prop" and p.get("color") is not None:
        text += ' color="%s"' % p["color"]
    if p.get("fixed"):
        text += ' fixed="true"'
    return text + "/>"


def _replace(world_xml, p, new):
    a, b = p["span"]
    return world_xml[:a] + _element(new) + world_xml[b:]


def with_prop_moved(world_xml, name, x_m, y_m, yaw_deg):
    """The text with the prop at ``(x_m, y_m)`` turned ``yaw_deg``; its
    height stays what the map gave it."""
    p = _find(world_xml, name)
    return _replace(world_xml, p, dict(p, pos=(float(x_m), float(y_m), p["pos"][2]), yaw=float(yaw_deg)))


def with_prop_fixed(world_xml, name, fixed):
    """The text with the prop stuck to the map (or free again)."""
    p = _find(world_xml, name)
    return _replace(world_xml, p, dict(p, fixed=bool(fixed)))


def unique_name(world_xml, base):
    """``base``, or ``base_2``, ``base_3``… — the first not taken."""
    taken = {p["name"] for p in props_in(world_xml)}
    if base not in taken:
        return base
    stem = re.sub(r"_\d+$", "", base)
    n = 2
    while "%s_%d" % (stem, n) in taken:
        n += 1
    return "%s_%d" % (stem, n)


def _indent_of(world_xml, at):
    line_start = world_xml.rfind("\n", 0, at) + 1
    if world_xml[line_start:at].strip() == "":
        return world_xml[line_start:at]
    return ""


def with_prop_added(world_xml, from_name, x_m, y_m, yaw_deg, new_name=None):
    """The text with another prop like ``from_name`` (same model, mass,
    colour and sticking) at a pose, placed right after it. Returns
    ``(text, name)``."""
    p = _find(world_xml, from_name)
    name = unique_name(world_xml, new_name or from_name)
    copy = dict(p, name=name, pos=(float(x_m), float(y_m), p["pos"][2]), yaw=float(yaw_deg))
    a, b = p["span"]
    return world_xml[:b] + "\n" + _indent_of(world_xml, a) + _element(copy) + world_xml[b:], name


def with_model_added(world_xml, name, file, x_m, y_m, yaw_deg, z_m=0.0):
    """The text with an assembly document placed as a new, free prop at
    the end of the world body. Returns ``(text, name)``."""
    end = world_xml.rfind("</worldbody>")
    if end < 0:
        raise PropError("the map has no <worldbody> to put a prop in")
    name = unique_name(world_xml, name)
    p = {"tag": "assembly_prop", "name": name, "file": str(file), "pos": (float(x_m), float(y_m), float(z_m)), "yaw": float(yaw_deg), "fixed": False}
    indent = _indent_of(world_xml, end)
    return world_xml[:end] + "  " + _element(p) + "\n" + indent + world_xml[end:], name


def with_prop_removed(world_xml, name):
    """The text without the prop (and the line break it sat on)."""
    p = _find(world_xml, name)
    a, b = p["span"]
    line_start = world_xml.rfind("\n", 0, a) + 1
    if world_xml[line_start:a].strip() == "":
        a = line_start - 1 if line_start > 0 else 0
    return world_xml[:a] + world_xml[b:]


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


def save_as(src_dir, world_xml, name, reserved=(), env=None, home=None):
    """Write a map: ``worlds/<slug>/`` under the data directory with the
    source map's files (its artwork, its props' models, its notes) and
    ``world.xml`` as given — models referenced by absolute path (added
    since the map was loaded) copied into ``props/`` and referenced
    from there. A name that slugs to one of ``reserved`` (the shipped
    aliases) is refused, so a shipped map is never shadowed; saving over
    the user's own map of that name replaces it. Returns ``(alias,
    path)``."""
    alias = slug(name)
    if alias in reserved:
        raise PropError("%r is a shipped map; choose another name" % (alias,))
    dest = user_worlds_dir(env, home) / alias
    src = Path(src_dir) if src_dir else None
    if dest.exists():
        shutil.rmtree(dest)
    if src is not None and src.is_dir():
        shutil.copytree(src, dest, ignore=shutil.ignore_patterns("world.xml", "__pycache__"))
    else:
        dest.mkdir(parents=True)
    # models that live outside the map come along, and the text points at the copies
    for p in reversed(props_in(world_xml)):
        key = "ldr" if p["tag"] == "lego_prop" else "file"
        ref = Path(p[key])
        if not ref.is_absolute():
            continue
        if not ref.is_file():
            raise PropError("prop %r: its model %s is gone" % (p["name"], ref))
        (dest / "props").mkdir(exist_ok=True)
        target = dest / "props" / ref.name
        n = 2
        while target.exists() and target.read_bytes() != ref.read_bytes():
            target = dest / "props" / ("%s-%d%s" % (ref.stem, n, ref.suffix))
            n += 1
        if not target.exists():
            shutil.copyfile(ref, target)
        world_xml = _replace(world_xml, p, dict(p, **{key: "props/" + target.name}))
    path = dest / "world.xml"
    path.write_text(world_xml)
    return alias, str(path)
