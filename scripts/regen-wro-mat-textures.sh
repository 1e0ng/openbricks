#!/usr/bin/env bash
# Regenerate the high-resolution WRO mat PNG textures from the
# official "Game Mat Printing File" PDFs. Run after the WRO season
# updates the source PDFs (which can happen mid-season for clarity
# fixes), or when adjusting render DPI.
#
# Source PDFs are NOT committed (large + WRO licensing); they're
# fetched on-demand to /tmp. The rendered PNGs ARE committed —
# they bundle into the wheel and feed Phase E1 colour-sensor
# texture sampling.
#
# DPI choice — see tools/openbricks/openbricks_sim/worlds/<name>/
# README.md for current value. 75 dpi gives ~7000×3375 px for a
# 2362×1143 mm mat (0.34 mm/pixel; the TCS34725's physical sampling
# spot is ~3 mm across, ~9 px — still comfortably oversampled).
# The render is then 256-colour palette-quantized: the mats are flat
# printed artwork, and the PyPI project hit its 10 GB storage limit
# when every release shipped ~17 MB of lossless mats into 16 wheels
# (2026-07-19). Do NOT raise DPI / drop the quantize step without
# checking the wheel-size guard in tests/test_wheel_bundles_worlds.
#
# Requires: curl, pdftoppm (poppler), Python PIL (quantize step).

set -euo pipefail

DPI=${DPI:-75}
TMP=${TMP:-/tmp/wro2026-mat-regen}
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORLDS_ROOT="$REPO_ROOT/tools/openbricks/openbricks_sim/worlds"

mkdir -p "$TMP"
cd "$TMP"

declare -a CATEGORIES=(
  "elementary:wro_2026_elementary_robot_rockstars:WRO-2026-GameMat-Elementary-Printing-File.pdf"
  "junior:wro_2026_junior_heritage_heroes:WRO-2026-GameMat-Junior-Printing-File.pdf"
  "senior:wro_2026_senior_mosaic_masters:WRO-2026-GameMat-Senior-Printing-File.pdf"
)

for entry in "${CATEGORIES[@]}"; do
  IFS=":" read -r short world_dir pdf <<< "$entry"
  echo "=== $short ==="
  if [[ ! -f "$pdf" ]]; then
    echo "fetching $pdf ..."
    curl -sL -o "$pdf" "https://wro-association.org/wp-content/uploads/$pdf"
  fi
  rm -f "${short}-${DPI}dpi-1.png"
  pdftoppm -r "$DPI" -png "$pdf" "${short}-${DPI}dpi"
  out="$WORLDS_ROOT/$world_dir/mat.png"
  cp "${short}-${DPI}dpi-1.png" "$out"
  # 256-colour palette quantize: near-lossless on flat printed
  # artwork, ~5x smaller on the wire. See the header comment.
  python3 - "$out" <<'PYEOF'
import sys
from PIL import Image
p = sys.argv[1]
Image.MAX_IMAGE_PIXELS = None
im = Image.open(p)
im = im.quantize(colors=256, method=Image.MEDIANCUT,
                 dither=Image.FLOYDSTEINBERG)
im.save(p, optimize=True)
PYEOF
  ls -la "$out"
done

echo
echo "Done. Diff in repo:"
cd "$REPO_ROOT"
git diff --stat tools/openbricks/openbricks_sim/worlds/*/mat.png
