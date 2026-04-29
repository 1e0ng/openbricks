# SPDX-License-Identifier: MIT
"""Tests for per-round randomization of WRO mission elements.

The Elementary world's randomization is the only spec wired today
(Junior + Senior land in follow-up PRs). The test surface here
pins:

  * Determinism — same seed → same layout.
  * 24 distinct layouts across all seeds (4! permutations of 4
    notes across 4 slots).
  * Each randomizable note ends up at one of the spec's slot
    coordinates; no two notes share a slot.
  * Fixed-position notes (red, green) don't move.
  * Per-note resting Z is preserved (so a sphere-shaped note
    doesn't sink into the mat after randomization).
"""

import unittest
from collections import Counter
from pathlib import Path

import mujoco

from openbricks_sim import randomization
from openbricks_sim.chassis import ChassisSpec
from openbricks_sim.world import load_world


_ELEMENTARY_PATH = (Path(__file__).resolve().parent.parent
                    / "openbricks_sim" / "worlds"
                    / "wro_2026_elementary_robot_rockstars" / "world.xml")


def _make_elementary():
    """Load the Elementary world with the default chassis spec.

    ``load_world`` returns ``MjData`` straight from ``MjModel`` —
    ``data.xpos`` etc. are still zeroed until the first
    ``mj_forward`` populates them from ``qpos``. Force that here so
    test assertions read the world.xml's ``pos`` values (and any
    pre-randomization checks see the actual starting layout)."""
    model, data, merged = load_world(
        str(_ELEMENTARY_PATH), chassis_spec=ChassisSpec())
    mujoco.mj_forward(model, data)
    return model, data, merged


def _xy_of(model, data, body_name):
    body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return float(data.xpos[body_id, 0]), float(data.xpos[body_id, 1])


def _z_of(model, data, body_name):
    body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return float(data.xpos[body_id, 2])


class ElementaryRandomizationTests(unittest.TestCase):

    def test_deterministic_for_fixed_seed(self):
        # Same seed twice must place the same notes at the same
        # slot labels — pins the contract that round-N reproducibility
        # is on the seed alone.
        m1, d1, _ = _make_elementary()
        m2, d2, _ = _make_elementary()
        layout1 = randomization.randomize(
            m1, d1, world="wro-2026-elementary", seed=42)
        layout2 = randomization.randomize(
            m2, d2, world="wro-2026-elementary", seed=42)
        self.assertEqual(layout1, layout2)

    def test_different_seeds_eventually_yield_all_24_permutations(self):
        # 4 random notes × 4 slots = 4! = 24 layouts. Across enough
        # seeds we should see all 24 distinct (body→slot) maps.
        # If the implementation accidentally constrained to fewer
        # (e.g. only rotated, didn't shuffle), this test fails.
        seen = set()
        for seed in range(200):  # 200 seeds, plenty for 24 outcomes
            m, d, _ = _make_elementary()
            layout = randomization.randomize(
                m, d, world="wro-2026-elementary", seed=seed)
            seen.add(tuple(sorted(layout.items())))
        self.assertEqual(
            len(seen), 24,
            "expected 4! = 24 distinct layouts across 200 seeds; "
            "got %d — randomization may not be a true permutation"
            % len(seen))

    def test_each_note_lands_on_a_spec_slot(self):
        m, d, _ = _make_elementary()
        randomization.randomize(
            m, d, world="wro-2026-elementary", seed=7)
        # The 4 slot positions from the spec
        spec_xys = {(0.0499, 0.4881), (0.1818, 0.4881),
                    (0.5775, 0.4881), (0.7094, 0.4881)}
        for note in ("note_black", "note_white", "note_yellow", "note_blue"):
            x, y = _xy_of(m, d, note)
            # Allow tiny numerical wiggle from mj_forward
            matched = any(abs(x - sx) < 1e-3 and abs(y - sy) < 1e-3
                          for sx, sy in spec_xys)
            self.assertTrue(
                matched,
                "%s landed at (%.4f, %.4f) which isn't a spec slot"
                % (note, x, y))

    def test_no_two_notes_share_a_slot(self):
        m, d, _ = _make_elementary()
        randomization.randomize(
            m, d, world="wro-2026-elementary", seed=12345)
        positions = []
        for note in ("note_black", "note_white", "note_yellow", "note_blue"):
            positions.append(_xy_of(m, d, note))
        # Every (x, y) should appear exactly once.
        counts = Counter(
            (round(x, 3), round(y, 3)) for x, y in positions)
        for pos, cnt in counts.items():
            self.assertEqual(
                cnt, 1,
                "two notes at %s — randomization isn't a permutation" % (pos,))

    def test_fixed_position_notes_dont_move(self):
        # The red and green notes are fixed per the rules. Capture
        # their positions before randomization, randomize, verify
        # nothing moved.
        m, d, _ = _make_elementary()
        before_red = _xy_of(m, d, "note_red")
        before_green = _xy_of(m, d, "note_green")
        randomization.randomize(
            m, d, world="wro-2026-elementary", seed=99)
        after_red = _xy_of(m, d, "note_red")
        after_green = _xy_of(m, d, "note_green")
        self.assertEqual(before_red, after_red)
        self.assertEqual(before_green, after_green)

    def test_resting_z_is_preserved_per_note(self):
        # The 4 notes have different shapes (sphere / box / cylinder
        # of varying heights) and so different resting Z heights
        # in the world.xml. Randomization places each note at the
        # slot's (x, y) but must keep its existing Z — otherwise
        # the sphere note sinks into the mat or the tall cylinder
        # floats. Capture each note's pre-randomization Z, then
        # check post-randomization that Z is unchanged.
        m, d, _ = _make_elementary()
        before = {n: _z_of(m, d, n)
                  for n in ("note_black", "note_white",
                            "note_yellow", "note_blue")}
        randomization.randomize(
            m, d, world="wro-2026-elementary", seed=2026)
        for note, z_before in before.items():
            z_after = _z_of(m, d, note)
            self.assertAlmostEqual(
                z_before, z_after, delta=1e-4,
                msg="%s z changed: %.6f → %.6f" % (note, z_before, z_after))

    def test_unknown_world_raises_keyerror(self):
        m, d, _ = _make_elementary()
        with self.assertRaises(KeyError):
            # Use a world name that has no spec — junior + senior
            # gained specs in F3.J / F3.S, so they no longer raise.
            randomization.randomize(
                m, d, world="wro-2026-imaginary", seed=1)


_JUNIOR_PATH = (Path(__file__).resolve().parent.parent
                / "openbricks_sim" / "worlds"
                / "wro_2026_junior_heritage_heroes" / "world.xml")


def _make_junior():
    model, data, merged = load_world(
        str(_JUNIOR_PATH), chassis_spec=ChassisSpec())
    mujoco.mj_forward(model, data)
    return model, data, merged


class JuniorRandomizationTests(unittest.TestCase):
    """Pin the Junior 'select 4 of 5 artefacts' randomization. Per
    rules-PDF p7: the 4 selected artefacts go to the 4 black squares
    at the lower end of the field; 1 is unused per round."""

    def test_four_selected_one_unused(self):
        m, d, _ = _make_junior()
        layout = randomization.randomize(
            m, d, world="wro-2026-junior", seed=42)
        # 5 artefacts total in the spec.
        self.assertEqual(len(layout), 5)
        # Exactly 4 go to slots, 1 is "unused".
        unused_count = sum(1 for v in layout.values() if v == "unused")
        slot_count = sum(1 for v in layout.values() if v.startswith("slot_"))
        self.assertEqual(unused_count, 1)
        self.assertEqual(slot_count, 4)

    def test_each_artefact_lands_on_a_mat_extracted_slot(self):
        # F5 replaced the estimated y=-0.45 slots with the 4 black-
        # square positions extracted from the high-res Junior mat
        # (y=-0.5055; x ∈ {-0.1052, +0.0267, +0.1586, +0.2904}).
        # Pin that the 4 selected artefacts each land at one of
        # those exact (x, y) — within tolerance for mj_forward
        # numerical noise.
        m, d, _ = _make_junior()
        layout = randomization.randomize(
            m, d, world="wro-2026-junior", seed=314)
        spec_xys = {(-0.1052, -0.5055), (+0.0267, -0.5055),
                    (+0.1586, -0.5055), (+0.2904, -0.5055)}
        for body_name, slot in layout.items():
            if slot == "unused":
                continue
            body_id = mujoco.mj_name2id(
                m, mujoco.mjtObj.mjOBJ_BODY, body_name)
            x = float(d.xpos[body_id, 0])
            y = float(d.xpos[body_id, 1])
            matched = any(abs(x - sx) < 1e-3 and abs(y - sy) < 1e-3
                          for sx, sy in spec_xys)
            self.assertTrue(
                matched,
                "%s landed at (%.4f, %.4f) which isn't one of the "
                "mat-extracted Junior slots" % (body_name, x, y))

    def test_unused_artefact_goes_off_mat(self):
        # Unused artefact gets stashed at z = -1 m so it doesn't
        # appear in the chassis camera frustum or interact with
        # other props.
        m, d, _ = _make_junior()
        layout = randomization.randomize(
            m, d, world="wro-2026-junior", seed=99)
        for body_name, slot in layout.items():
            if slot != "unused":
                continue
            body_id = mujoco.mj_name2id(
                m, mujoco.mjtObj.mjOBJ_BODY, body_name)
            z = float(d.xpos[body_id, 2])
            self.assertLess(z, -0.5,
                            "unused artefact %s should be stashed "
                            "off-mat (z<-0.5); got z=%f"
                            % (body_name, z))

    def test_deterministic(self):
        m1, d1, _ = _make_junior()
        m2, d2, _ = _make_junior()
        layout1 = randomization.randomize(
            m1, d1, world="wro-2026-junior", seed=7)
        layout2 = randomization.randomize(
            m2, d2, world="wro-2026-junior", seed=7)
        self.assertEqual(layout1, layout2)

    def test_each_seed_can_yield_any_unused(self):
        # Across enough seeds, each of the 5 artefacts should land
        # in the "unused" slot at least once. If the RNG only ever
        # marked the same colour as unused, this test fails.
        seen_unused = set()
        for seed in range(500):
            m, d, _ = _make_junior()
            layout = randomization.randomize(
                m, d, world="wro-2026-junior", seed=seed)
            for body, slot in layout.items():
                if slot == "unused":
                    seen_unused.add(body)
        self.assertEqual(
            len(seen_unused), 5,
            "expected all 5 artefacts to be selected as 'unused' "
            "at least once across 500 seeds; got: %s" % seen_unused)


_SENIOR_PATH = (Path(__file__).resolve().parent.parent
                / "openbricks_sim" / "worlds"
                / "wro_2026_senior_mosaic_masters" / "world.xml")


def _make_senior():
    model, data, merged = load_world(
        str(_SENIOR_PATH), chassis_spec=ChassisSpec())
    mujoco.mj_forward(model, data)
    return model, data, merged


class SeniorRandomizationTests(unittest.TestCase):
    """Pin the Senior cement-element randomization. Per rules-PDF
    p7: cement elements are randomly placed within their matching-
    coloured cement storage area each round. The current spec
    permutes the 10 yellow elements; blue / green / white land in
    follow-up tightening."""

    def test_all_four_color_groups_permuted(self):
        # F5 expanded the Senior spec from yellow-only (10 elements)
        # to all 4 cement colours (40 elements). Pin: every cement
        # element appears in the layout, and each one lands at a
        # slot whose label has its own colour.
        m, d, _ = _make_senior()
        layout = randomization.randomize(
            m, d, world="wro-2026-senior", seed=2026)
        # 40 cement bodies + 12 mosaic-pattern cells (paper-under-frame).
        cement_layout = {k: v for k, v in layout.items()
                         if k.startswith("cement_")}
        self.assertEqual(
            len(cement_layout), 40,
            "Senior layout should cover 40 cement elements (10 each "
            "× 4 colours); got %d" % len(cement_layout))
        for color in ("yellow", "blue", "green", "white"):
            for i in range(1, 11):
                body = "cement_%s_%d" % (color, i)
                self.assertIn(body, layout, "%s missing from layout" % body)
                slot = layout[body]
                self.assertTrue(
                    slot.startswith("cement_%s_slot_" % color),
                    "%s landed at slot %r — should be in the %s "
                    "group's slots, not another colour's" % (body, slot, color))

    def test_each_color_permutes_independently(self):
        # The 4 colour groups must shuffle within their own slots,
        # never across colours. After a randomize(), every yellow
        # element must occupy one of the 10 yellow slots, every
        # blue element one of the 10 blue slots, etc.
        m, d, _ = _make_senior()
        randomization.randomize(
            m, d, world="wro-2026-senior", seed=314)
        # Expected slot positions per colour (matches the spec
        # generator: x ∈ {0.85..1.05}, y per colour).
        y_band = {"yellow": -0.45, "blue": -0.30,
                  "green":  -0.15, "white":  0.00}
        for color, y0 in y_band.items():
            expected_xys = {(round(0.85 + 0.05 * (i % 5), 3),
                             round(y0 + 0.05 * (i // 5), 3))
                            for i in range(10)}
            for i in range(1, 11):
                body = "cement_%s_%d" % (color, i)
                bid = mujoco.mj_name2id(
                    m, mujoco.mjtObj.mjOBJ_BODY, body)
                xy = (round(float(d.xpos[bid, 0]), 3),
                      round(float(d.xpos[bid, 1]), 3))
                self.assertIn(
                    xy, expected_xys,
                    "%s landed at %s — not one of the %s group's "
                    "expected slot positions" % (body, xy, color))

    def test_deterministic(self):
        m1, d1, _ = _make_senior()
        m2, d2, _ = _make_senior()
        layout1 = randomization.randomize(
            m1, d1, world="wro-2026-senior", seed=11)
        layout2 = randomization.randomize(
            m2, d2, world="wro-2026-senior", seed=11)
        self.assertEqual(layout1, layout2)


class SeniorMosaicPatternTests(unittest.TestCase):
    """Pin the paper-under-frame mosaic-pattern randomization. 12
    cells, 3 of each colour (yellow/blue/green/white) per round.
    Information-only — not a LEGO body."""

    def test_balanced_colors(self):
        # The pattern must be balanced: exactly 3 cells of each
        # of the 4 colours. WRO uses a balanced pattern so the
        # robot needs all 12 (3 × 4) tiles, not a lopsided count.
        from collections import Counter
        pat = randomization.senior_mosaic_pattern(seed=42)
        counts = Counter(pat.values())
        self.assertEqual(set(counts), {"yellow", "blue", "green", "white"})
        self.assertTrue(all(c == 3 for c in counts.values()),
                        "expected 3 of each colour; got %s" % dict(counts))

    def test_twelve_cells(self):
        pat = randomization.senior_mosaic_pattern(seed=7)
        self.assertEqual(len(pat), 12)
        self.assertEqual(set(pat),
                         {"mosaic_cell_%d" % (i + 1) for i in range(12)})

    def test_deterministic_for_fixed_seed(self):
        a = randomization.senior_mosaic_pattern(seed=314)
        b = randomization.senior_mosaic_pattern(seed=314)
        self.assertEqual(a, b)

    def test_distinct_seeds_yield_distinct_patterns(self):
        # 369,600 patterns are possible; even a small seed range
        # should produce many distinct ones.
        seen = set()
        for s in range(50):
            pat = randomization.senior_mosaic_pattern(seed=s)
            seen.add(tuple(pat.items()))
        self.assertGreater(len(seen), 40,
                           "expected >40 distinct patterns across 50 "
                           "seeds; got %d" % len(seen))

    def test_randomize_includes_pattern_for_senior(self):
        # ``randomize()`` for the Senior world must fold the
        # mosaic pattern into the returned layout, so callers
        # (e.g. the CLI log) see it alongside body shuffles.
        m, d, _ = _make_senior()
        layout = randomization.randomize(
            m, d, world="wro-2026-senior", seed=2026)
        # 40 cement bodies + 12 mosaic cells = 52 entries.
        self.assertEqual(len(layout), 52)
        cells = {k: v for k, v in layout.items()
                 if k.startswith("mosaic_cell_")}
        self.assertEqual(len(cells), 12)
        for color in cells.values():
            self.assertIn(color, ("yellow", "blue", "green", "white"))


if __name__ == "__main__":
    unittest.main()
