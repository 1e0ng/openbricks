# SPDX-License-Identifier: MIT
"""LDraw → MJCF converter for WRO mission props.

The WRO Brick Set 45811 (per General Rules §7.4 the source of all
RoboMission game elements) uses only 9 distinct part types — see
``_PART_REGISTRY`` below. Each prop is a list of LDraw ``1`` lines,
one per brick, each with a colour, a 3×3 rotation matrix, an XYZ
translation in LDraw Drawing Units (LDU), and a part filename
(``3001.dat`` etc.).

This module:

  * Parses LDraw ``.ldr`` text into a list of part placements.
  * Converts LDraw's right-handed Y-down coordinate system into
    MuJoCo's right-handed Z-up system (1 LDU = 0.4 mm).
  * For each placement, looks up the part's geometry from
    ``_PART_REGISTRY`` and emits MJCF ``<geom>`` lines for the
    brick body + each stud.
  * Wraps the lot in a ``<body>`` with a freejoint and an even
    mass distribution.

Coordinate conversion (LDraw → MJCF):

  * X: same axis, same direction.
  * LDraw +Y is **down** (gravity direction); MJCF +Z is **up**.
    So ``mjcf_z = -ldraw_y``.
  * LDraw +Z is "out of page" (toward viewer); MJCF +Y is forward.
    So ``mjcf_y = ldraw_z``.

Stud geometry (per the LDraw library):

  * Stud cylinder diameter = 12 LDU = 4.8 mm.
  * Stud height = 4 LDU = 1.6 mm above the brick top.
  * Studs are emitted as visual-only ``<geom contype="0"
    conaffinity="0">`` cylinders so they don't add tiny collision
    surfaces (which would slow physics + cause numerical noise).

What's intentionally omitted:

  * The cross-axle hole on parts like 3894 (1x6 with hole) — it's
    visually small and the closed-box approximation is fine for
    sim purposes. Adding the hole as a subtractive geom isn't
    supported in MJCF.
  * Anti-stud sockets on the underside of bricks — same reasoning.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Tuple


LDU_PER_STUD       = 20
LDU_PER_BRICK_H    = 24
LDU_PER_PLATE_H    = 8
LDU_PER_MM         = 2.5    # 1 LDU = 0.4 mm
STUD_DIAMETER_LDU  = 12
STUD_HEIGHT_LDU    = 4


# Standard LDraw colour codes mapped to MJCF material names. Worlds
# that splice in generated brick MJCF must define materials with
# these names in their ``<asset>`` blocks.
COLOR_NAME_BY_LDRAW_CODE = {
    0:  "lego_black",
    1:  "lego_blue",
    2:  "lego_green",
    4:  "lego_red",
    14: "lego_yellow",
    15: "lego_white",
    7:  "lego_gray",
    8:  "lego_gray",   # dark gray collapses to gray
    71: "lego_white",  # light gray-ish
}


@dataclass(frozen=True)
class _PartSpec:
    """Geometry of a single LDraw part type, in LDU.

    Origin of each part is at the LDraw convention: top-centre of
    the underside (the bottom of the studs' anti-mate surface), with
    +Y pointing down into the part body.

    ``w_ldu`` is along part-local +X, ``d_ldu`` is along part-local
    +Z, ``h_ldu`` is along part-local -Y (negative because LDraw +Y
    is down).

    ``stud_grid`` is a list of (sx, sz) stud-positions in LDU,
    measured from the part origin. Empty for tiles / plates without
    visible studs.
    """
    w_ldu: float
    d_ldu: float
    h_ldu: float
    stud_grid: Tuple[Tuple[float, float], ...]


def _grid_studs(w_studs: int, d_studs: int) -> Tuple[Tuple[float, float], ...]:
    """Return stud positions for an N×M brick top, with the part
    origin at the brick's lateral centre."""
    out = []
    for sx in range(w_studs):
        for sd in range(d_studs):
            cx = (sx - (w_studs - 1) / 2) * LDU_PER_STUD
            cz = (sd - (d_studs - 1) / 2) * LDU_PER_STUD
            out.append((cx, cz))
    return tuple(out)


# Registry of the 9 part types in the WRO 45811 brick set. Counts
# from John Holbrook's 45811.ldr (as of 2026-04-29):
#
#   3009.dat: 1x6 brick                         288×
#   3001.dat: 2x4 brick                         288×
#   3894.dat: 1x6 brick with axle hole           48× (modeled as 3009)
#   3003.dat: 2x2 brick                          48×
#   3069b.dat: 1x2 tile (smooth top, no studs)   24×
#   72039.dat: not common; placeholder           8× (modeled as 1x2)
#   3022.dat: 2x2 plate                          8×
#   30000.dat: not common; placeholder           8× (modeled as 1x2 tile)
#   22119.dat: not common; placeholder           4× (modeled as 1x2 plate)
#
# Counts are from the brick-set inventory; per-prop counts come
# from each prop's hand-built .ldr in worlds/<prop>/build.ldr.
_PART_REGISTRY = {
    "3001.dat": _PartSpec(   # 2x4 brick
        w_ldu=2 * LDU_PER_STUD, d_ldu=4 * LDU_PER_STUD, h_ldu=LDU_PER_BRICK_H,
        stud_grid=_grid_studs(2, 4)),
    "3002.dat": _PartSpec(   # 2x3 brick (in 45819 expansion)
        w_ldu=2 * LDU_PER_STUD, d_ldu=3 * LDU_PER_STUD, h_ldu=LDU_PER_BRICK_H,
        stud_grid=_grid_studs(2, 3)),
    "3003.dat": _PartSpec(   # 2x2 brick
        w_ldu=2 * LDU_PER_STUD, d_ldu=2 * LDU_PER_STUD, h_ldu=LDU_PER_BRICK_H,
        stud_grid=_grid_studs(2, 2)),
    "3009.dat": _PartSpec(   # 1x6 brick
        w_ldu=1 * LDU_PER_STUD, d_ldu=6 * LDU_PER_STUD, h_ldu=LDU_PER_BRICK_H,
        stud_grid=_grid_studs(1, 6)),
    "3010.dat": _PartSpec(   # 1x4 brick
        w_ldu=1 * LDU_PER_STUD, d_ldu=4 * LDU_PER_STUD, h_ldu=LDU_PER_BRICK_H,
        stud_grid=_grid_studs(1, 4)),
    "3894.dat": _PartSpec(   # 1x6 brick with cross-axle hole — same envelope
        w_ldu=1 * LDU_PER_STUD, d_ldu=6 * LDU_PER_STUD, h_ldu=LDU_PER_BRICK_H,
        stud_grid=_grid_studs(1, 6)),
    "3703.dat": _PartSpec(   # Technic 1x16 brick with holes (45819)
        w_ldu=1 * LDU_PER_STUD, d_ldu=16 * LDU_PER_STUD, h_ldu=LDU_PER_BRICK_H,
        stud_grid=_grid_studs(1, 16)),
    "3022.dat": _PartSpec(   # 2x2 plate
        w_ldu=2 * LDU_PER_STUD, d_ldu=2 * LDU_PER_STUD, h_ldu=LDU_PER_PLATE_H,
        stud_grid=_grid_studs(2, 2)),
    "3032.dat": _PartSpec(   # 4x6 plate (in 45819)
        w_ldu=4 * LDU_PER_STUD, d_ldu=6 * LDU_PER_STUD, h_ldu=LDU_PER_PLATE_H,
        stud_grid=_grid_studs(4, 6)),
    "3035.dat": _PartSpec(   # 4x8 plate (in 45819)
        w_ldu=4 * LDU_PER_STUD, d_ldu=8 * LDU_PER_STUD, h_ldu=LDU_PER_PLATE_H,
        stud_grid=_grid_studs(4, 8)),
    "3069b.dat": _PartSpec(  # 1x2 tile — smooth, no studs
        w_ldu=1 * LDU_PER_STUD, d_ldu=2 * LDU_PER_STUD, h_ldu=LDU_PER_PLATE_H,
        stud_grid=()),
    # Less-common parts in the set (8× each, sometimes in props,
    # sometimes spares). Modeled as nearest equivalent — visual
    # fidelity for these parts is approximate.
    "72039.dat": _PartSpec(
        w_ldu=1 * LDU_PER_STUD, d_ldu=2 * LDU_PER_STUD, h_ldu=LDU_PER_PLATE_H,
        stud_grid=()),
    "30000.dat": _PartSpec(
        w_ldu=1 * LDU_PER_STUD, d_ldu=2 * LDU_PER_STUD, h_ldu=LDU_PER_PLATE_H,
        stud_grid=()),
    "22119.dat": _PartSpec(
        w_ldu=2 * LDU_PER_STUD, d_ldu=2 * LDU_PER_STUD, h_ldu=LDU_PER_PLATE_H,
        stud_grid=_grid_studs(2, 2)),
}


@dataclass(frozen=True)
class Placement:
    """One LDraw part instance.

    ``rotation`` is a 9-tuple — the row-major 3×3 rotation matrix
    LDraw stores per part-instance line. Most WRO bricks use one
    of two transforms:

      * Identity ``(1,0,0, 0,1,0, 0,0,1)`` — brick stands the
        regular way (long axis along +Z in LDraw = +Y in MJCF).
      * 90° around Y ``(0,0,1, 0,1,0, -1,0,0)`` — brick rotated
        in the floor plane.

    ``translation`` is the part origin in LDU (LDraw frame).
    ``part`` is the .dat filename. ``ldraw_color`` is the LDraw
    colour code; we map it via :data:`COLOR_NAME_BY_LDRAW_CODE`.
    """
    rotation:    Tuple[float, ...]
    translation: Tuple[float, float, float]
    part:        str
    ldraw_color: int


def parse_ldr(text: str) -> List[Placement]:
    """Parse LDraw text into a list of :class:`Placement`.

    Comments (``0`` lines), sub-files / steps (``2``, ``3``, ``4``,
    ``5``), and steps-end markers are ignored. Only ``1`` lines
    (part instances) are returned."""
    out: List[Placement] = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("0"):
            continue
        toks = line.split()
        if toks[0] != "1":
            continue
        if len(toks) < 15:
            raise ValueError("malformed LDraw 1-line: " + repr(line))
        color = int(toks[1])
        x, y, z = (float(toks[2]), float(toks[3]), float(toks[4]))
        rot = tuple(float(t) for t in toks[5:14])
        part = toks[14]
        out.append(Placement(
            rotation=rot,
            translation=(x, y, z),
            part=part,
            ldraw_color=color))
    return out


def _ldraw_to_mjcf_xyz(x_ldu: float, y_ldu: float, z_ldu: float
                       ) -> Tuple[float, float, float]:
    """LDraw (Y-down) → MJCF (Z-up) translation, converting LDU to
    metres. Mapping: mjcf_x = ldraw_x; mjcf_y = ldraw_z;
    mjcf_z = -ldraw_y. Each value divided by ``LDU_PER_MM * 1000``
    to land in metres."""
    scale = 1.0 / (LDU_PER_MM * 1000.0)
    return (x_ldu * scale, z_ldu * scale, -y_ldu * scale)


def _emit_geom(name_prefix: str,
               kind: str,
               size_m: Tuple[float, ...],
               pos_m: Tuple[float, float, float],
               material: str,
               *,
               mass_kg: float = 0.0,
               visual_only: bool = False) -> str:
    """Format one ``<geom .../>`` line."""
    extras = ""
    if visual_only:
        extras += ' contype="0" conaffinity="0"'
    if mass_kg > 0:
        extras += ' mass="{:.5f}"'.format(mass_kg)
    size_str = " ".join("{:.5f}".format(s) for s in size_m)
    return ('<geom type="{kind}" '
            'size="{size}" pos="{px:.5f} {py:.5f} {pz:.5f}" '
            'material="{material}"{extras}/>').format(
                kind=kind, size=size_str,
                px=pos_m[0], py=pos_m[1], pz=pos_m[2],
                material=material, extras=extras)


def emit_placement(placement: Placement,
                   *,
                   indent: str = "    ",
                   per_part_mass_kg: float = 0.001,
                   color_override: int = None) -> str:
    """Convert one LDraw placement into MJCF geom lines (brick body
    + visual stud cylinders).

    The brick-body geom carries the placement's mass; stud cylinders
    are visual-only (zero mass) so the whole prop's CoM matches the
    bricks' centroids and the stud count doesn't artificially raise
    the moment of inertia.

    If ``color_override`` is given, it replaces the LDraw colour code
    on the placement (and on its studs). Useful for shared
    ``.ldr`` files where the geometry is identical across instances
    but the colour differs (e.g. Junior visitors and artefacts share
    the same body shape per colour)."""
    spec = _PART_REGISTRY.get(placement.part)
    if spec is None:
        raise KeyError("no LDraw part spec for {!r} — add to "
                       "_PART_REGISTRY".format(placement.part))
    effective_color = (color_override
                       if color_override is not None
                       else placement.ldraw_color)
    material = COLOR_NAME_BY_LDRAW_CODE.get(effective_color)
    if material is None:
        # 0 (LDraw black) maps cleanly; everything else falls back
        # to white as a safe default. Add the colour code to the
        # mapping if you hit this path on a real prop.
        material = "lego_white"

    # The part's brick BODY centre in LDraw frame: at part origin
    # plus (-h_ldu / 2) along Y (brick extends downward in LDraw).
    cx_ldraw, cy_ldraw, cz_ldraw = placement.translation
    body_cx = cx_ldraw
    body_cy = cy_ldraw - spec.h_ldu / 2  # LDraw +Y down → into the brick
    body_cz = cz_ldraw

    body_pos_mjcf = _ldraw_to_mjcf_xyz(body_cx, body_cy, body_cz)
    # Body half-extents: in LDraw the brick is w_ldu × h_ldu × d_ldu
    # (X × Y × Z); after mapping to MJCF (X × Z × -Y), the half-
    # extents along (mx, my, mz) are (w_ldu/2, d_ldu/2, h_ldu/2),
    # all scaled to metres.
    scale = 1.0 / (LDU_PER_MM * 1000.0)
    body_half = (spec.w_ldu / 2 * scale,
                 spec.d_ldu / 2 * scale,
                 spec.h_ldu / 2 * scale)

    lines = [indent + _emit_geom(
        placement.part, "box", body_half, body_pos_mjcf, material,
        mass_kg=per_part_mass_kg)]

    # Studs: visual-only cylinders on top of the brick. Each stud's
    # centre is on the brick's top surface, offset by stud_grid in
    # part-local X/Z. In LDraw, "top" of the brick is at Y = -h
    # (since +Y is down and the part origin is the underside of
    # the studs' base ring).
    if spec.stud_grid:
        stud_radius_m = STUD_DIAMETER_LDU / 2 * scale
        stud_half_h_m = STUD_HEIGHT_LDU / 2 * scale
        for sx_ldu, sz_ldu in spec.stud_grid:
            stud_cx_ldraw = cx_ldraw + sx_ldu
            stud_cy_ldraw = cy_ldraw - spec.h_ldu - STUD_HEIGHT_LDU / 2
            stud_cz_ldraw = cz_ldraw + sz_ldu
            stud_pos_mjcf = _ldraw_to_mjcf_xyz(
                stud_cx_ldraw, stud_cy_ldraw, stud_cz_ldraw)
            lines.append(indent + _emit_geom(
                placement.part + "_stud", "cylinder",
                (stud_radius_m, stud_half_h_m), stud_pos_mjcf, material,
                visual_only=True))
    return "\n".join(lines)


def emit_prop_body(name: str,
                   pos_world_m: Tuple[float, float, float],
                   ldr_text: str,
                   *,
                   freejoint: bool = True,
                   total_mass_kg: float = 0.05,
                   indent: str = "    ",
                   color_override: int = None) -> str:
    """Convert an LDraw model into a complete MJCF ``<body>`` block.

    ``pos_world_m`` is the body's origin in the world frame; the
    LDraw model's frame origin lands there. ``total_mass_kg`` is
    distributed evenly across the brick bodies (visual-only studs
    carry zero mass).

    ``color_override`` (optional LDraw colour code) overrides every
    placement's colour. Lets a single ``.ldr`` describe a prop's
    geometry once and have multiple ``<lego_prop>`` instances colour
    it differently (Junior visitors and artefacts use this)."""
    placements = parse_ldr(ldr_text)
    if not placements:
        raise ValueError("LDraw text contains no part instances")
    per_brick_mass = total_mass_kg / len(placements)

    inner = indent + "  "
    lines = []
    lines.append('{indent}<body name="{name}" '
                 'pos="{x:.5f} {y:.5f} {z:.5f}">'
                 .format(indent=indent, name=name,
                         x=pos_world_m[0], y=pos_world_m[1],
                         z=pos_world_m[2]))
    if freejoint:
        lines.append("{}<freejoint/>".format(inner))
    for placement in placements:
        lines.append(emit_placement(
            placement, indent=inner, per_part_mass_kg=per_brick_mass,
            color_override=color_override))
    lines.append("{}</body>".format(indent))
    return "\n".join(lines)
