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
# README.md for current value. 150 dpi gives ~14000×6750 px for a
# 2362×1143 mm mat (0.169 mm/pixel, well past the TCS34725's
# physical sampling spot).
#
# Requires: curl, pdftoppm (poppler).

set -euo pipefail

DPI=${DPI:-150}
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
  ls -la "$out"
done

echo
echo "Done. Diff in repo:"
cd "$REPO_ROOT"
git diff --stat tools/openbricks/openbricks_sim/worlds/*/mat.png
