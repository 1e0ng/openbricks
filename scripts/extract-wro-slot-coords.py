#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Extract randomization slot coordinates from a WRO mat texture.

How the slot coordinates in
``tools/openbricks/openbricks_sim/randomization.py`` were produced.
Re-run when WRO updates the mat artwork (or when extending the
spec to a new category whose slots are visible as same-coloured
squares against a contrasting background).

Usage:

    scripts/extract-wro-slot-coords.py \
        --mat tools/openbricks/openbricks_sim/worlds/wro_2026_elementary_robot_rockstars/mat.png \
        --target-rgb 144 208 112 \
        --tol 32 \
        --upper-strip 0.15 \
        --min-cluster-px 5000

Output: each connected component of pixels close to ``--target-rgb``
within the upper ``--upper-strip`` of the mat image, with its
centroid in mat-local world coordinates (origin at mat centre,
+X right, +Y "up").

For the Elementary mat (default args above), this produces 4
identical-size clusters which are the 4 light-green note-start
squares.

For Junior / Senior, run with the appropriate target-rgb sampled
from the mat image (use a colour picker on the texture you care
about) and tune ``--upper-strip`` / ``--min-cluster-px``.

Requires PIL + numpy + scipy. The first two are mujoco's deps so
already on the dev machine; scipy needs ``pip install scipy``.
"""

import argparse
import sys
from pathlib import Path

import numpy as np


# Dimensions per the WRO General Rules §7.2.
_MAT_WIDTH_M = 2.362
_MAT_HEIGHT_M = 1.143


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Extract randomization slot coordinates from a WRO mat.")
    parser.add_argument("--mat", required=True,
                        help="Path to the rasterized mat.png")
    parser.add_argument("--target-rgb", type=int, nargs=3, required=True,
                        metavar=("R", "G", "B"),
                        help="Target colour (8-bit per channel) of the "
                             "slot-marking squares.")
    parser.add_argument("--tol", type=int, default=32,
                        help="Per-channel tolerance for the colour match.")
    parser.add_argument("--upper-strip", type=float, default=0.15,
                        help="Fraction of mat height (from top) to search. "
                             "0.15 catches the upper edge area where "
                             "Elementary's note-start squares live.")
    parser.add_argument("--lower-strip", type=float, default=0.0,
                        help="Like --upper-strip but searches the BOTTOM. "
                             "Set when slots are at the bottom of the mat. "
                             "Don't combine with --upper-strip.")
    parser.add_argument("--min-cluster-px", type=int, default=5000,
                        help="Drop connected components smaller than this "
                             "(noise / decorations / partial matches).")
    args = parser.parse_args(argv)

    # Lazy imports so the help text works without these installed.
    from PIL import Image
    from scipy.ndimage import label, center_of_mass

    Image.MAX_IMAGE_PIXELS = None  # silence the bomb warning on big mats
    img = np.asarray(Image.open(args.mat))
    H, W, _ = img.shape

    target = np.array(args.target_rgb, dtype=np.int16)
    diff = np.abs(img.astype(np.int16) - target)
    mask = np.all(diff <= args.tol, axis=2)

    # Restrict to upper or lower strip
    band = np.zeros_like(mask)
    if args.lower_strip > 0:
        if args.upper_strip > 0:
            print("error: pass either --upper-strip or --lower-strip, "
                  "not both", file=sys.stderr)
            return 2
        rows = int(args.lower_strip * H)
        band[H - rows:] = mask[H - rows:]
    else:
        rows = int(args.upper_strip * H)
        band[:rows] = mask[:rows]

    labels_arr, n = label(band)
    print(f"{n} connected components total ({int(band.sum()):,} matching px "
          f"in band)")

    clusters = []
    for i in range(1, n + 1):
        sz = int((labels_arr == i).sum())
        if sz < args.min_cluster_px:
            continue
        cy, cx = center_of_mass(band, labels_arr, i)
        clusters.append((sz, float(cx), float(cy)))

    if not clusters:
        print("no clusters above --min-cluster-px — try lower threshold "
              "or different --target-rgb / --tol", file=sys.stderr)
        return 1

    print(f"\n{len(clusters)} clusters above {args.min_cluster_px} px:")
    clusters.sort(key=lambda t: (t[2], t[1]))   # row, then column
    for sz, cx, cy in clusters:
        x_m = (cx - W / 2) * _MAT_WIDTH_M / W
        y_m = -(cy - H / 2) * _MAT_HEIGHT_M / H
        print(f"  px ({cx:7.1f}, {cy:6.1f})  world ({x_m:+.4f}, "
              f"{y_m:+.4f}) m   size {sz:>9,d}")

    print("\nFor a randomization.py spec, copy the world coordinates into "
          "_Slot(x=..., y=..., label=...) — labels are conventionally "
          "left-to-right by X (slot_1, slot_2, ...).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
