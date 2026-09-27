# SPDX-License-Identifier: MIT
"""Maps as JSON: ``map.json``, the ``openbricks-map/1`` format.

A map is a folder: ``map.json`` and the files it names — the mat's
artwork, meshes, the props' models. MuJoCo reads no JSON (its models are
MJCF or URDF, both XML, or its own binary), so the loader turns a map
into MJCF text in memory (:func:`to_mjcf`) and nothing else ever holds
XML. The format::

    {
      "format": "openbricks-map/1",
      "name": "practice_line",
      "about": ["what the map is for, a line per entry"],
      "physics": {"timestep": 0.001, "iterations": 20, "solver": "Newton",
                  "gravity": [0, 0, -9.81]},
      "headlight": {"diffuse": [0.6, 0.6, 0.6], "ambient": [...], "specular": [...]},
      "textures": [{"name": "mat_tex", "type": "2d", "file": "mat.png"}],
      "materials": [{"name": "mat", "texture": "mat_tex", "texrepeat": [1, 1]}],
      "meshes": [{"name": "frame", "file": "frame.stl", "scale": [0.001, 0.001, 0.001]}],
      "lights": [{"pos": [0, 0, 1.5], "dir": [0, 0, -1]}],
      "geoms": [{"name": "floor", "type": "plane", "size": [1, 0.5, 0.05], "rgba": [1, 1, 1, 1]}],
      "props": [{"name": "clef", "ldr": "props/clef.ldr", "pos": [0.1, 0.2, 0.005], "mass": 0.05},
                {"name": "tower", "file": "props/tower.assembly.json", "pos": [0, 0, 0],
                 "yaw": 90, "pitch": 0, "roll": 0, "fixed": true}],
      "cameras": [{"name": "overhead", "pos": [0, 0, 2.6], "xyaxes": [1, 0, 0, 0, 1, 0]}]
    }

The elements carry MuJoCo's own attribute names and units (metres, and
vectors as arrays); any of them may carry a ``note``, which the loader
skips. A prop is an LDraw model (``ldr``, with a ``mass`` in kg and an
optional ``color``) or an ``openbricks-assembly/1`` document (``file``);
it is turned roll about x, then pitch about y, then yaw about z
(degrees), and is free unless ``fixed``.

An exported map is one JSON file: the map with a ``files`` table
holding every file of its folder (:func:`pack`, :func:`unpack`).
"""
import base64
import copy
import json
import os
import re
from pathlib import Path

FORMAT = "openbricks-map/1"
FILE = "map.json"

# Attributes whose values are words, not numbers.
_WORDS = {"name", "type", "file", "material", "texture", "mesh", "solver", "ldr", "color", "model"}
# LDraw colour keywords a prop's ``color`` takes, beside a numeric LDraw code.
COLOR_KEYWORDS = {"black": 0, "blue": 1, "green": 2, "red": 4, "yellow": 14, "white": 15, "gray": 7}
_SECTIONS = ("textures", "materials", "meshes", "lights", "geoms", "props", "cameras")


class MapError(ValueError):
    """A map that cannot be read, turned into a model, or packed."""


# ---------------------------------------------------------------- reading


def check(m, where="map"):
    """``m`` as a map, or a named error."""
    if not isinstance(m, dict) or m.get("format") != FORMAT:
        raise MapError("%s is not an %s file" % (where, FORMAT))
    for key in _SECTIONS:
        if not isinstance(m.get(key, []), list):
            raise MapError("%s: %r must be a list" % (where, key))
    for p in m.get("props", []):
        if not isinstance(p, dict) or not p.get("name"):
            raise MapError("%s: every prop needs a name" % where)
        if ("ldr" in p) == ("file" in p):
            raise MapError("%s: prop %r needs one model, an ldr or a file" % (where, p["name"]))
        pos = p.get("pos")
        if not (isinstance(pos, list) and len(pos) == 3 and all(isinstance(v, (int, float)) for v in pos)):
            raise MapError("%s: prop %r pos must be 3 numbers; got %r" % (where, p["name"], pos))
        if "ldr" in p and not isinstance(p.get("mass"), (int, float)):
            raise MapError("%s: LDraw prop %r needs a mass in kg" % (where, p["name"]))
    return m


def load(path):
    """The map in ``path`` (a ``map.json``)."""
    path = Path(path)
    try:
        with open(path) as fh:
            m = json.load(fh)
    except (OSError, ValueError) as e:
        raise MapError("could not read %s: %s" % (path, e)) from e
    return check(m, str(path))


def dumps(obj, indent=0):
    """JSON text for a map or a document, as a person reads it: nested
    objects and lists of objects indented, a list of plain values (a
    position, a colour) on one line, text as written (no escapes)."""
    pad, inner = "  " * indent, "  " * (indent + 1)
    if isinstance(obj, dict):
        if not obj:
            return "{}"
        items = ["%s%s: %s" % (inner, json.dumps(k, ensure_ascii=False), dumps(v, indent + 1)) for k, v in obj.items()]
        return "{\n" + ",\n".join(items) + "\n" + pad + "}"
    if isinstance(obj, list):
        if all(not isinstance(v, (dict, list)) for v in obj):
            flat = "[" + ", ".join(json.dumps(v, ensure_ascii=False) for v in obj) + "]"
            if len(flat) <= 100 or all(not isinstance(v, str) for v in obj):
                return flat
        return "[\n" + ",\n".join(inner + dumps(v, indent + 1) for v in obj) + "\n" + pad + "]"
    return json.dumps(obj, ensure_ascii=False)


def save(m, path):
    """Write the map to ``path`` as ``map.json`` text."""
    check(m, str(path))
    Path(path).write_text(dumps(m) + "\n")


# ------------------------------------------------------------- to MuJoCo


def _value(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (list, tuple)):
        return " ".join(_value(x) for x in v)
    if isinstance(v, float):
        return repr(v)
    return str(v)


def _element(tag, attrs, indent):
    parts = ['%s="%s"' % (k, _value(v)) for k, v in attrs.items() if k != "note"]
    return "%s<%s %s/>" % (indent, tag, " ".join(parts)) if parts else "%s<%s/>" % (indent, tag)


def _resolve(ref, map_dir):
    p = Path(ref)
    return p if p.is_absolute() else Path(map_dir) / p


def _prop_body(p, map_dir):
    from openbricks_sim import assembly as assembly_mod
    from openbricks_sim import lego_mjcf, props
    name = p["name"]
    pos = tuple(float(v) for v in p["pos"])
    yaw, pitch, roll = (float(p.get(k, 0.0)) for k in ("yaw", "pitch", "roll"))
    fixed = bool(p.get("fixed", False))
    if "ldr" in p:
        path = _resolve(p["ldr"], map_dir)
        if not path.is_file():
            raise MapError("prop %r references missing .ldr file %r" % (name, str(path)))
        color = p.get("color")
        code = None
        if color is not None:
            color = str(color)
            if color.isdigit():
                code = int(color)
            elif color in COLOR_KEYWORDS:
                code = COLOR_KEYWORDS[color]
            else:
                raise MapError("prop %r color=%r unrecognised — use a numeric LDraw code or one of %s"
                               % (name, color, sorted(COLOR_KEYWORDS)))
        return lego_mjcf.emit_prop_body(name, pos, path.read_text(), total_mass_kg=float(p["mass"]),
                                        color_override=code, yaw_deg=yaw, pitch_deg=pitch,
                                        roll_deg=roll, freejoint=not fixed)
    path = _resolve(p["file"], map_dir)
    if not path.is_file():
        raise MapError("prop %r references missing file %r" % (name, str(path)))
    try:
        with open(path) as fh:
            doc = json.load(fh)
        bricks_out, _ = assembly_mod.prop_bricks(doc)
    except (ValueError, assembly_mod.AssemblyError) as e:
        raise MapError("prop %r: %s" % (name, e)) from e
    # never under the map: a prop whose lowest brick would sink below the floor, turned as the
    # map turns it, is lifted onto it (one standing higher is left where the map put it)
    lowest = assembly_mod.prop_lowest_m(bricks_out, props.euler_quat(yaw, pitch, roll))
    if pos[2] + lowest < -1e-6:
        pos = (pos[0], pos[1], -lowest)
    try:
        return assembly_mod.prop_body_xml(name, pos, yaw, fixed, bricks_out, pitch_deg=pitch, roll_deg=roll)
    except assembly_mod.AssemblyError as e:
        raise MapError("prop %r: %s" % (name, e)) from e


def to_mjcf(m, map_dir):
    """The map as MJCF text for MuJoCo, its props expanded into bodies
    (files resolved against ``map_dir``). Held in memory only."""
    check(m)
    out = ['<mujoco model="%s">' % m.get("name", "map"),
           '  <compiler angle="degree" coordinate="local"/>']
    if m.get("physics"):
        out.append(_element("option", m["physics"], "  "))
    if m.get("headlight"):
        out += ["  <visual>", _element("headlight", m["headlight"], "    "), "  </visual>"]
    assets = ([("texture", a) for a in m.get("textures", [])] + [("material", a) for a in m.get("materials", [])]
              + [("mesh", a) for a in m.get("meshes", [])])
    if assets:
        out.append("  <asset>")
        out += [_element(tag, a, "    ") for tag, a in assets]
        out.append("  </asset>")
    out.append("  <worldbody>")
    out += [_element("light", a, "    ") for a in m.get("lights", [])]
    out += [_element("geom", a, "    ") for a in m.get("geoms", [])]
    out += [_prop_body(p, map_dir) for p in m.get("props", [])]
    out += [_element("camera", a, "    ") for a in m.get("cameras", [])]
    out += ["  </worldbody>", "</mujoco>", ""]
    return "\n".join(out)


# ------------------------------------------------------------ from MJCF


def _parse_value(key, text):
    if key in _WORDS:
        return text
    tokens = text.split()
    values = []
    for t in tokens:
        if re.fullmatch(r"-?\d+", t):
            values.append(int(t))
        else:
            try:
                values.append(float(t))
            except ValueError:
                return text
    return values[0] if len(values) == 1 else values


def _attrs(e):
    return {k: _parse_value(k, v) for k, v in e.attrib.items()}


def _comment_lines(text):
    """One XML comment's lines, as written, blank lines at either end
    dropped: a comment whose words start on ``<!--``'s own line has that
    line trimmed and the rest dedented together (they are indented to
    line up under it); one that starts on the next line is dedented as a
    block, keeping its own indents."""
    lines = [ln.rstrip() for ln in text.splitlines()]
    starts_below = bool(lines) and not lines[0].strip()
    while lines and not lines[0].strip():
        lines.pop(0)
    while lines and not lines[-1].strip():
        lines.pop()
    if not lines:
        return []

    def dedent(block):
        indent = min((len(ln) - len(ln.lstrip()) for ln in block if ln.strip()), default=0)
        return [ln[indent:] for ln in block]

    if starts_below:
        return dedent(lines)
    return [lines[0].strip()] + dedent(lines[1:])


def _note(comments):
    """Comments as a note: one line as text, more as a list, a blank line
    between one comment and the next; None for none."""
    out = []
    for c in comments:
        block = _comment_lines(c)
        if block:
            if out:
                out.append("")
            out += block
    if not out:
        return None
    return out[0] if len(out) == 1 else out


def from_mjcf(text):
    """A map from a map's old MJCF text (a shipped map before 4.32.0, or
    a map of the user's own saved then): its comments become ``about``
    and ``note`` entries, its prop placeholders props. Anything beyond
    what the maps used is refused by name."""
    import xml.etree.ElementTree as ET
    head = text.split("<mujoco", 1)[0]
    about = [c for c in re.findall(r"<!--(.*?)-->", head, re.S)]
    try:
        root = ET.fromstring(text, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    except ET.ParseError as e:
        raise MapError("not an MJCF map: %s" % e) from e
    if root.tag != "mujoco":
        raise MapError("not an MJCF map: its root is <%s>" % root.tag)
    m = {"format": FORMAT, "name": root.attrib.get("model", "map")}
    pending = []

    def take(d):
        n = _note(pending)
        pending.clear()
        if n is not None:
            d["note"] = n
        return d

    for e in root:
        if e.tag is ET.Comment:
            about.append(e.text or "")
        elif e.tag == "compiler":
            if dict(e.attrib) != {"angle": "degree", "coordinate": "local"}:
                raise MapError("unsupported <compiler %s>" % dict(e.attrib))
        elif e.tag == "option":
            m["physics"] = _attrs(e)
        elif e.tag == "visual":
            for v in e:
                if v.tag is ET.Comment:
                    continue
                if v.tag != "headlight":
                    raise MapError("unsupported <visual><%s>" % v.tag)
                m["headlight"] = _attrs(v)
        elif e.tag == "asset":
            for a in e:
                if a.tag is ET.Comment:
                    pending.append(a.text or "")
                    continue
                key = {"texture": "textures", "material": "materials", "mesh": "meshes"}.get(a.tag)
                if key is None:
                    raise MapError("unsupported <asset><%s>" % a.tag)
                m.setdefault(key, []).append(take(_attrs(a)))
        elif e.tag == "worldbody":
            for w in e:
                if w.tag is ET.Comment:
                    pending.append(w.text or "")
                    continue
                if w.tag in ("lego_prop", "assembly_prop"):
                    m.setdefault("props", []).append(take(_prop_from_placeholder(w)))
                    continue
                key = {"light": "lights", "geom": "geoms", "camera": "cameras"}.get(w.tag)
                if key is None:
                    raise MapError("unsupported <worldbody><%s>" % w.tag)
                m.setdefault(key, []).append(take(_attrs(w)))
            if pending:
                about += pending
                pending.clear()
        else:
            raise MapError("unsupported <%s>" % e.tag)
    n = _note(about) if about else None
    if n is not None:
        m["about"] = n if isinstance(n, list) else [n]
    ordered = {k: m[k] for k in ("format", "name", "about", "physics", "headlight") + _SECTIONS if k in m}
    return check(ordered)


def _prop_from_placeholder(w):
    a = w.attrib
    if "name" not in a or "pos" not in a:
        raise MapError("a <%s> needs a name and a pos" % w.tag)
    p = {"name": a["name"]}
    if w.tag == "lego_prop":
        p["ldr"] = a["ldr"]
    else:
        p["file"] = a["file"]
    pos = [float(t) for t in a["pos"].split()]
    if len(pos) != 3:
        raise MapError("prop %r pos must be 3 numbers; got %r" % (a["name"], a["pos"]))
    p["pos"] = pos
    if w.tag == "lego_prop":
        p["mass"] = float(a["mass"])
    for key in ("yaw", "pitch", "roll"):
        if key in a and float(a[key]):
            p[key] = float(a[key])
    if "color" in a:
        p["color"] = a["color"]
    if a.get("fixed", "").strip().lower() in ("true", "1", "yes"):
        p["fixed"] = True
    return p


# ------------------------------------------------------ export and import

_TEXT = {".ldr", ".dat", ".md", ".txt"}


def numbered(name, n):
    """A file name with ``-n`` before its extensions, all of them:
    ``tower.assembly.json`` → ``tower-2.assembly.json``."""
    stem, dot, ext = name.partition(".")
    return "%s-%d%s%s" % (stem, n, dot, ext)


def pack(m, map_dir):
    """The map as one JSON object to export: the map, and a ``files``
    table holding every file of its folder (the artwork, meshes, props'
    models, notes) — a model it names by absolute path (added since the
    map was loaded) is taken in under ``props/`` and named from there.
    A JSON file stays JSON, a text file text, anything else base64."""
    check(m)
    m = copy.deepcopy(m)
    files = {}
    root = Path(map_dir) if map_dir else None
    if root is not None and root.is_dir():
        for path in sorted(root.rglob("*")):
            if path.is_file() and path.name != FILE and "__pycache__" not in path.parts:
                files[path.relative_to(root).as_posix()] = path
    for p in m.get("props", []):
        key = "ldr" if "ldr" in p else "file"
        ref = Path(p[key])
        if not ref.is_absolute():
            if root is None or not (root / ref).is_file():
                raise MapError("prop %r: its model %s is not in the map's folder" % (p["name"], ref))
            continue
        if not ref.is_file():
            raise MapError("prop %r: its model %s is gone" % (p["name"], ref))
        rel = "props/" + ref.name
        n = 2
        while rel in files and files[rel].read_bytes() != ref.read_bytes():
            rel = "props/" + numbered(ref.name, n)
            n += 1
        files[rel] = ref
        p[key] = rel
    out = dict(m)
    out["files"] = {rel: _encode(path) for rel, path in files.items()}
    return out


def _encode(path):
    data = path.read_bytes()
    if path.suffix == ".json":
        try:
            return {"json": json.loads(data.decode("utf-8"))}
        except (UnicodeDecodeError, ValueError):
            pass
    if path.suffix in _TEXT:
        try:
            return {"text": data.decode("utf-8")}
        except UnicodeDecodeError:
            pass
    return {"base64": base64.b64encode(data).decode("ascii")}


def unpack(obj, dest_dir):
    """Write an exported map into ``dest_dir``: its files, then its
    ``map.json``. Returns the map. A file named outside the folder is
    refused."""
    check(obj, "the export")
    files = obj.get("files", {})
    if not isinstance(files, dict):
        raise MapError("the export's files must be a table")
    dest = Path(dest_dir)
    for rel, body in files.items():
        parts = Path(rel).parts
        if Path(rel).is_absolute() or ".." in parts or not parts or rel == FILE:
            raise MapError("the export names a file outside its folder: %r" % rel)
        if not isinstance(body, dict) or len(body) != 1:
            raise MapError("the export's file %r must hold json, text or base64" % rel)
        (kind, value), = body.items()
        target = dest / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        if kind == "json":
            target.write_text(json.dumps(value, indent=2) + "\n")
        elif kind == "text":
            target.write_text(value)
        elif kind == "base64":
            target.write_bytes(base64.b64decode(value))
        else:
            raise MapError("the export's file %r must hold json, text or base64, not %r" % (rel, kind))
    m = {k: v for k, v in obj.items() if k != "files"}
    save(m, dest / FILE)
    return m


def export_to(m, map_dir, path):
    """Write the map, packed, to ``path``."""
    with open(path, "w") as fh:
        json.dump(pack(m, map_dir), fh, indent=1)
        fh.write("\n")


def read_export(path):
    """An exported map read from ``path``."""
    try:
        with open(path) as fh:
            obj = json.load(fh)
    except (OSError, ValueError) as e:
        raise MapError("could not read %s: %s" % (path, e)) from e
    return check(obj, os.fspath(path))
