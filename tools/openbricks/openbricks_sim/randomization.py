# SPDX-License-Identifier: MIT
"""Per-round randomization of WRO mission elements.

The WRO RoboMission rules call for some game elements to be re-placed
each round before the team's robot starts. Randomization is the part
of the challenge that forces robots to *perceive* the field rather
than rely on hardcoded positions. Per the WRO General Rules glossary
(``Robot Round`` definition): "Before the round starts with the first
team but after all robots are placed on the robot parking,
randomizations to game fields (if any) are done."

This module is the openbricks-sim equivalent. After ``load_world``
parses the MJCF and instantiates ``MjData``, the caller asks
``randomize(model, data, world="wro-2026-elementary", seed=N)`` to
permute the randomizable game elements. The change is applied
directly to ``data.qpos`` (every randomizable element rides on a
freejoint), and ``mj_forward`` refreshes derived quantities. The
return value is the chosen layout — body name → slot label — for
logging and test pins.

Randomization specs are per-world and pulled from each category's
Game Rules PDF. A world maps to a *tuple of specs*: most worlds
have one spec (one shuffle group), but Senior has four — one per
cement-element colour group, since the rules permute within each
colour's storage area independently. ``randomize()`` runs each
spec in sequence with a single seeded RNG so determinism is
preserved across the multi-group case.
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import mujoco


# ---------------------------------------------------------------------
# Spec types

@dataclass(frozen=True)
class _Slot:
    """A randomization slot: where a randomizable element can land.
    Coordinates are world frame in metres (mat origin at world
    origin, +X right toward start area, +Y "up" toward stage).

    The slot is where the element's footprint is centred: the middle
    of the box, along the mat, that holds all of its geoms. A build's
    origin is wherever its first brick sits, often a stud or more
    away from its middle, so it is the middle that lands on the
    square. Z and heading are absent — each element keeps the map's
    own (the notes are different heights, and the map turns them to
    face the way they stand on the real field).

    ``label`` is a human-readable id used in returned layouts and
    test assertions; pick something stable across PRs since tests
    pin it ("slot_1" not "slot at +0.50,+0.42").
    """
    x: float
    y: float
    label: str


@dataclass(frozen=True)
class _RandomizationSpec:
    """Spec for one world's randomization step.

    ``elements`` is the ordered list of randomizable bodies (each must
    be a body in the world XML with a freejoint). ``slots`` is the
    list of slot positions.

    Two randomization shapes are supported:

      * **Permutation** (``len(slots) == len(elements)``): each
        round, shuffle ``elements`` and place each at the
        corresponding slot. No two elements share a slot.
      * **Selection** (``len(slots) < len(elements)``): each round,
        randomly pick ``len(slots)`` elements and place them at the
        slots; the remaining elements get ``off_mat_pos`` (well
        below the mat, out of the chassis camera's frustum). Junior
        artefacts use this — 4 of 5 placed per round, the 5th
        unused.

    Fixed-position elements aren't listed here — they stay where
    the map put them and are explicitly NOT randomized per
    the rules.
    """
    elements: Sequence[str]
    slots: Sequence[_Slot]
    # Where to stash unselected elements (see "Selection" above).
    # Default is ~1 m below the mat surface — safely out of any
    # camera frame and won't interact with the chassis.
    off_mat_pos: Tuple[float, float, float] = (0.0, 0.0, -1.0)


# ---------------------------------------------------------------------
# Per-world specs

# Elementary "Robot Rockstars" — only the 4 notes black/white/yellow/
# blue get permuted. The red and green notes stay at fixed positions
# (per the Game Rules PDF p7 — "the positions of the green and red
# note are not random"). 4! = 24 distinct layouts.
#
# Slot coordinates come from pixel-inspecting the official
# mat.png artwork (13949×6750 px = 2362×1143 mm). The 4 light-green
# squares cluster as 4 connected components of identical size
# (35,532 px each, ≈32×32 mm) at the upper end of the mat:
#
#     world (+0.0499, +0.4881) m   <- slot_1
#     world (+0.1818, +0.4881) m   <- slot_2
#     world (+0.5775, +0.4881) m   <- slot_3
#     world (+0.7094, +0.4881) m   <- slot_4
#
# Spacing forms two pairs: ~0.13 m within each pair, ~0.40 m
# between pairs (the green-note's fixed position sits between
# the pairs). Re-derive via the script if the mat artwork ever
# updates: scripts/extract-wro-slot-coords.py (TODO — committed
# in this PR).
_ELEMENTARY = _RandomizationSpec(
    elements=("black_note", "white_note", "yellow_note", "blue_note"),
    slots=(
        _Slot(x=0.0499, y=0.4881, label="slot_1"),
        _Slot(x=0.1818, y=0.4881, label="slot_2"),
        _Slot(x=0.5775, y=0.4881, label="slot_3"),
        _Slot(x=0.7094, y=0.4881, label="slot_4"),
    ),
)


# Junior "Heritage Heroes" — per Game Rules p7 ("Summary
# randomization"): "Four of the artefacts are randomly placed on
# the four black squares at the lower end of the game field. Per
# round one artefact is not used."
#
# 5 artefacts × choose 4 × permute across 4 slots = 5 × 4! = 120
# distinct layouts.
#
# Slot coordinates extracted from the high-res Junior mat (the
# 13949×6750 px artwork shipped in 0.10.8) via
# ``scripts/extract-wro-slot-coords.py``. Run with target-rgb
# 0/0/0 (the squares are black against the mat's painted
# background), --tol 30, --lower-strip 0.20, and
# --min-cluster-px 5000. The script reports 4 identical-size
# clusters (~35,700 px ≈ 32×32 mm each) at:
#
#     world (-0.1052, -0.5055) m   <- slot_1
#     world (+0.0267, -0.5055) m   <- slot_2
#     world (+0.1586, -0.5055) m   <- slot_3
#     world (+0.2904, -0.5055) m   <- slot_4
#
# Spacing ~0.13 m between adjacent slots — the same pattern as
# Elementary's 4 light-green note-start squares.
_JUNIOR = _RandomizationSpec(
    elements=("artefact_red", "artefact_blue", "artefact_green",
              "artefact_yellow", "artefact_black"),
    slots=(
        _Slot(x=-0.1052, y=-0.5055, label="slot_1"),
        _Slot(x=+0.0267, y=-0.5055, label="slot_2"),
        _Slot(x=+0.1586, y=-0.5055, label="slot_3"),
        _Slot(x=+0.2904, y=-0.5055, label="slot_4"),
    ),
    # 5 elements, 4 slots → "select 4" semantics. 1 artefact lands
    # off-mat each round.
)


# Senior "Mosaic Masters" — per Game Rules p7: "All cement
# elements are randomly placed within the matching-coloured
# cement storage area."
#
# Each colour group (yellow / blue / green / white) has 10 cement
# elements; they get permuted within their colour's storage area
# each round. Modelled as 4 separate ``_RandomizationSpec`` per
# colour, all keyed under ``"wro-2026-senior"`` in ``_SPECS``;
# ``randomize()`` runs each spec in sequence with one shared RNG
# so a fixed seed still yields a fixed 40-element layout.
#
# Slot positions match the map's 5×2 grid per
# colour group (storage areas don't appear as colour-coded zones
# on the printed mat, so we keep the existing grid as the
# reference). Per-colour Y bands:
#   yellow  y ∈ {-0.45, -0.40}
#   blue    y ∈ {-0.30, -0.25}
#   green   y ∈ {-0.15, -0.10}
#   white   y ∈ { 0.00, +0.05}
# X bands are identical: {0.85, 0.90, 0.95, 1.00, 1.05}.
def _senior_color_spec(color, y_bottom):
    """Build a 10-element / 10-slot spec for one cement colour."""
    elements = tuple("cement_{}_{}".format(color, i + 1)
                     for i in range(10))
    slots = tuple(
        _Slot(x=0.85 + 0.05 * (i % 5),
              y=y_bottom + 0.05 * (i // 5),
              label="cement_{}_slot_{}".format(color, i + 1))
        for i in range(10))
    return _RandomizationSpec(elements=elements, slots=slots)


_SENIOR_YELLOW = _senior_color_spec("yellow", -0.45)
_SENIOR_BLUE   = _senior_color_spec("blue",   -0.30)
_SENIOR_GREEN  = _senior_color_spec("green",  -0.15)
_SENIOR_WHITE  = _senior_color_spec("white",   0.00)


# A world maps to a tuple of specs; ``randomize()`` runs each in
# sequence with a single seeded RNG. Most worlds have one spec
# (one shuffle group); Senior has four (one per cement colour
# since each colour permutes within its own storage area).
_SPECS: Dict[str, Tuple[_RandomizationSpec, ...]] = {
    "wro-2026-elementary": (_ELEMENTARY,),
    "wro-2026-junior":     (_JUNIOR,),
    "wro-2026-senior":     (_SENIOR_YELLOW, _SENIOR_BLUE,
                            _SENIOR_GREEN,  _SENIOR_WHITE),
}


# ---------------------------------------------------------------------
# Senior mosaic-frame pattern
#
# The Senior rules also randomize the mosaic *pattern* — the
# 12-cell layout the robot must reproduce inside the mosaic frame.
# The pattern is communicated to teams via a paper sheet placed
# under the frame; the robot reads it (typically with a colour
# sensor through the frame's open cells) to know which colour
# tile to place where. This isn't a movable LEGO body, so it
# doesn't go through ``_RandomizationSpec`` / ``randomize()``'s
# freejoint mover — it's pure information.
#
# Constraint: 12 cells, 3 each of yellow/blue/green/white (the
# typical balanced pattern WRO uses). 12! / (3!⁴) = 369,600
# distinct patterns. Same seeded RNG ``randomize()`` uses, so
# ``seed=N`` pins both body positions and the pattern.

_MOSAIC_CELL_COUNT  = 12
_MOSAIC_COLOR_COUNT = 4
assert _MOSAIC_CELL_COUNT % _MOSAIC_COLOR_COUNT == 0, \
    "balanced pattern needs cells divisible by colour count"
_MOSAIC_PER_COLOR   = _MOSAIC_CELL_COUNT // _MOSAIC_COLOR_COUNT
_MOSAIC_COLORS      = ("yellow", "blue", "green", "white")


def senior_mosaic_pattern(seed: Optional[int] = None) -> Dict[str, str]:
    """Return a deterministic Senior mosaic-frame pattern as a
    ``{cell_label: color}`` map.

    12 cells, 3 each of yellow/blue/green/white, permuted by the
    seeded RNG. Cell labels are ``mosaic_cell_1`` through
    ``mosaic_cell_12``, ordered row-major across the 4×3 frame
    grid. Same ``seed`` always returns the same pattern; a fresh
    RNG (``seed=None``) returns a non-deterministic one.

    This is a standalone helper — ``randomize()`` calls it for
    Senior worlds and folds the entries into the layout dict so
    the CLI's ``[randomize]`` log shows the pattern alongside
    the per-body shuffles.
    """
    return _mosaic_pattern_from_rng(random.Random(seed))


def _mosaic_pattern_from_rng(rng) -> Dict[str, str]:
    deck = list(_MOSAIC_COLORS) * _MOSAIC_PER_COLOR
    rng.shuffle(deck)
    return {"mosaic_cell_{}".format(i + 1): deck[i]
            for i in range(_MOSAIC_CELL_COUNT)}


# ---------------------------------------------------------------------
# Apply

def randomize(model, data, world: str,
              seed: Optional[int] = None) -> Dict[str, str]:
    """Apply the randomization spec for ``world`` to ``model + data``.

    Returns the chosen layout as a dict of ``body_name → slot_label``.
    Caller can log this for round-by-round records or pin in tests.

    ``seed`` is the only knob; pass an integer for deterministic
    layouts (the same seed always produces the same permutation), or
    None to use Python's default random source. WRO doesn't specify
    any particular RNG — this is the sim's choice. ``seed=round_number``
    is a fine convention if the user wants every round to be different
    but reproducible.

    Raises ``KeyError`` if ``world`` has no spec yet (Junior + Senior
    today). Raises ``ValueError`` if a named element doesn't exist as
    a freejointed body in the loaded model — guards against the spec
    drifting out of sync with the world XML.
    """
    if world not in _SPECS:
        raise KeyError(
            "no randomization spec for world {!r} (have: {})"
            .format(world, sorted(_SPECS.keys())))
    specs = _SPECS[world]
    rng = random.Random(seed)
    layout: Dict[str, str] = {}

    # Each spec is one independent shuffle group (e.g. one cement
    # colour for Senior, or the single Elementary note group). They
    # share one RNG so ``seed=N`` still pins the entire multi-group
    # outcome.
    for spec in specs:
        if len(spec.elements) < len(spec.slots):
            # ``elements`` < ``slots`` would leave some slots empty,
            # which the WRO rules don't currently call for — flag.
            raise RuntimeError(
                "{!r} spec has {} elements but {} slots — need at "
                "least as many elements as slots"
                .format(world, len(spec.elements), len(spec.slots)))
        perm = list(spec.elements)
        rng.shuffle(perm)
        selected = perm[:len(spec.slots)]
        unselected = perm[len(spec.slots):]
        # Place selected elements at slots, each footprint centred on
        # its slot as the map turns it.
        offsets = _footprint_offsets(model, selected)
        for element_name, slot in zip(selected, spec.slots):
            dx, dy = offsets[element_name]
            _place_freejoint_body(model, data, element_name,
                                  slot.x - dx, slot.y - dy)
            layout[element_name] = slot.label
        # Stash unselected elements off-mat so they don't interfere
        # with the chassis or the camera. Z is forced to off_mat_pos.z.
        for element_name in unselected:
            _place_freejoint_body(
                model, data, element_name,
                spec.off_mat_pos[0], spec.off_mat_pos[1],
                z_override=spec.off_mat_pos[2])
            layout[element_name] = "unused"

    # Senior also has a mosaic-pattern randomization (paper-under-
    # frame, not a LEGO body — see ``senior_mosaic_pattern``). Fold
    # it into the layout so callers / the CLI log see the pattern
    # alongside the body shuffles. Same RNG → same seed → same
    # pattern.
    if world == "wro-2026-senior":
        for cell_label, color in _mosaic_pattern_from_rng(rng).items():
            layout[cell_label] = color

    # Refresh derived quantities (xpos / cam_xpos / sensordata) so any
    # downstream code that reads positions sees the new placements
    # without needing its own ``mj_forward`` call.
    mujoco.mj_forward(model, data)
    return layout


def _place_freejoint_body(model, data, body_name: str,
                          x: float, y: float,
                          *, z_override: float = None) -> None:
    """Move a body that's attached via a freejoint to ``(x, y)``,
    keeping the map's resting height and heading for it.

    ``z_override`` (optional) bypasses the body_pos Z and uses the
    given value instead. Used to stash "unselected" elements
    off-mat in select-N-of-M randomization.

    The Z preservation matters: each randomizable note in the
    Elementary world is a different build and so has a different
    resting Z. Forcing all notes to the same Z would either sink the
    tall ones into the mat or float the short ones above it. The
    heading matters the same way: the map stands each note as it
    faces on the real field.

    Raises ``ValueError`` if the named body is missing or doesn't
    have a freejoint. Internal — exposed only to this module's
    ``randomize`` and tests."""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise ValueError("no body named " + repr(body_name) + " in model")
    # The body's first joint is its freejoint (the map convention
    # for randomizable elements is one freejoint per body).
    joint_id = int(model.body_jntadr[body_id])
    if joint_id < 0:
        raise ValueError(
            "body " + repr(body_name) + " has no joint — randomizable "
            "elements must have a freejoint")
    if int(model.jnt_type[joint_id]) != int(mujoco.mjtJoint.mjJNT_FREE):
        raise ValueError(
            "body " + repr(body_name) + "'s first joint is not a "
            "freejoint — randomization can only relocate freejointed bodies")
    qpos_addr = int(model.jnt_qposadr[joint_id])
    # Resting Z: the ``z_override`` if given, else the map's own
    # (MuJoCo keeps a freejoint's map pose in ``qpos0``). Override is
    # used to stash unselected elements in select-N-of-M
    # randomization off-mat.
    if z_override is not None:
        z = float(z_override)
    else:
        z = float(model.qpos0[qpos_addr + 2])
    # Freejoint qpos layout: [x, y, z, qw, qx, qy, qz]; the heading
    # is the map's.
    data.qpos[qpos_addr]     = x
    data.qpos[qpos_addr + 1] = y
    data.qpos[qpos_addr + 2] = z
    data.qpos[qpos_addr + 3:qpos_addr + 7] = model.qpos0[qpos_addr + 3:qpos_addr + 7]
    # Reset velocity for this freejoint too — otherwise a body that was
    # mid-motion at the time of randomization would keep its old
    # velocity, which is surprising for a "set up the scene" action.
    qvel_addr = int(model.jnt_dofadr[joint_id])
    for i in range(6):  # freejoint has 6 DOFs
        data.qvel[qvel_addr + i] = 0.0


def footprint_middle(model, data, body_name: str) -> Tuple[float, float]:
    """The ``(x, y)`` in metres, world frame, of the middle of a body's
    footprint as ``data`` has it: the centre of the box, along the
    mat, that holds all of its geoms and its descendants' geoms.

    This is where an element stands on the mat, whatever its origin:
    a Workbench build's origin is its first brick's, often a stud or
    more away from its middle. Needs ``data``'s geom poses current
    (``mj_forward`` or ``mj_kinematics`` after a qpos change).

    Raises ``ValueError`` if the body is missing or has no geoms."""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise ValueError("no body named " + repr(body_name) + " in model")
    lo = [float("inf")] * 2
    hi = [float("-inf")] * 2
    found = False
    for g in range(model.ngeom):
        b = int(model.geom_bodyid[g])
        while b != body_id and b != 0:
            b = int(model.body_parentid[b])
        if b != body_id:
            continue
        found = True
        centre = model.geom_aabb[g, :3]
        half = model.geom_aabb[g, 3:]
        rot = data.geom_xmat[g].reshape(3, 3)
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    corner = data.geom_xpos[g] + rot @ (centre + half * (sx, sy, sz))
                    for axis in (0, 1):
                        lo[axis] = min(lo[axis], float(corner[axis]))
                        hi[axis] = max(hi[axis], float(corner[axis]))
    if not found:
        raise ValueError("body " + repr(body_name) + " has no geoms to stand on")
    return (lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0


def _footprint_offsets(model, names: Sequence[str]) -> Dict[str, Tuple[float, float]]:
    """Each named body's footprint middle less its origin, along the
    mat, at the map's pose (``qpos0``, whatever ``data`` holds now).
    A shuffle keeps each element's heading, so the offset stays true
    wherever the element is put."""
    at_map = mujoco.MjData(model)
    mujoco.mj_kinematics(model, at_map)
    out = {}
    for name in names:
        x, y = footprint_middle(model, at_map, name)
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        out[name] = (x - float(at_map.xpos[body_id, 0]),
                     y - float(at_map.xpos[body_id, 1]))
    return out
