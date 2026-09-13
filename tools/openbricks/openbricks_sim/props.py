# SPDX-License-Identifier: MIT
"""The props on a map, and maps of the user's own.

A prop is a ``<lego_prop name="…" ldr="…" pos="x y z" mass="…"
[yaw="deg"] [color="…"]/>`` placeholder in a world's MJCF (the loader
expands it into an LDraw-built body). The map editor moves, adds and
removes props by rewriting those placeholders in the world text, so
the text stays the source of truth, and saves the result as a new map
under the data directory, where the run server lists it beside the
shipped ones.

The data directory is the one the sim's markers use:
``$OPENBRICKS_DATA_DIR``, else ``$XDG_DATA_HOME/openbricks``, else
``~/.local/share/openbricks``; maps live in its ``worlds/<alias>/``.
"""
import os
import re
import shutil
from pathlib import Path

# Attribute order is fixed: name, ldr, pos, mass, then the optional
# yaw and color. ``world.py`` expands matches of this same pattern.
PROP_RE = re.compile(
    r'<lego_prop\s+name="(?P<name>[^"]+)"\s+'
    r'ldr="(?P<ldr>[^"]+)"\s+'
    r'pos="(?P<pos>[^"]+)"\s+'
    r'mass="(?P<mass>[^"]+)"'
    r'(?:\s+yaw="(?P<yaw>[^"]+)")?'
    r'(?:\s+color="(?P<color>[^"]+)")?'
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
        raise PropError("lego_prop %r pos must be %d floats; got %r" % (name, n, text))
    return values


def props_in(world_xml):
    """Every prop placeholder in the text, in order: name, ldr, pos (m),
    mass (kg), yaw (deg), color (or None), and its span in the text."""
    out = []
    for m in PROP_RE.finditer(world_xml):
        out.append({
            "name": m.group("name"),
            "ldr": m.group("ldr"),
            "pos": _floats(m.group("pos"), m.group("name"), 3),
            "mass": float(m.group("mass")),
            "yaw": float(m.group("yaw")) if m.group("yaw") is not None else 0.0,
            "color": m.group("color"),
            "span": m.span(),
        })
    return out


def _find(world_xml, name):
    for p in props_in(world_xml):
        if p["name"] == name:
            return p
    raise PropError("no prop named %r on this map" % (name,))


def _element(p):
    """The placeholder text for a prop record (yaw written only when it
    turns, colour only when set)."""
    text = '<lego_prop name="%s" ldr="%s" pos="%.5f %.5f %.5f" mass="%g"' % (
        p["name"], p["ldr"], p["pos"][0], p["pos"][1], p["pos"][2], p["mass"])
    yaw = round(float(p["yaw"]), 3)
    if yaw:
        text += ' yaw="%g"' % yaw
    if p.get("color") is not None:
        text += ' color="%s"' % p["color"]
    return text + "/>"


def with_prop_moved(world_xml, name, x_m, y_m, yaw_deg):
    """The text with the prop at ``(x_m, y_m)`` turned ``yaw_deg``; its
    height stays what the map gave it."""
    p = _find(world_xml, name)
    p = dict(p, pos=(float(x_m), float(y_m), p["pos"][2]), yaw=float(yaw_deg))
    a, b = p["span"]
    return world_xml[:a] + _element(p) + world_xml[b:]


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


def with_prop_added(world_xml, from_name, x_m, y_m, yaw_deg, new_name=None):
    """The text with another prop like ``from_name`` (same model, mass and
    colour) at a pose, placed right after it. Returns ``(text, name)``."""
    p = _find(world_xml, from_name)
    name = unique_name(world_xml, new_name or from_name)
    copy = dict(p, name=name, pos=(float(x_m), float(y_m), p["pos"][2]), yaw=float(yaw_deg))
    a, b = p["span"]
    indent = ""
    line_start = world_xml.rfind("\n", 0, a) + 1
    if world_xml[line_start:a].strip() == "":
        indent = world_xml[line_start:a]
    return world_xml[:b] + "\n" + indent + _element(copy) + world_xml[b:], name


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


def save_as(src_dir, world_xml, name, reserved=(), env=None, home=None):
    """Write a map: ``worlds/<slug>/`` under the data directory with the
    source map's files (its artwork, its props' models, its notes) and
    ``world.xml`` as given. A name that slugs to one of ``reserved``
    (the shipped aliases) is refused, so a shipped map is never
    shadowed; saving over the user's own map of that name replaces it.
    Returns ``(alias, path)``."""
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
    path = dest / "world.xml"
    path.write_text(world_xml)
    return alias, str(path)
