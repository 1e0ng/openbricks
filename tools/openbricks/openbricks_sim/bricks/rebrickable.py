# SPDX-License-Identifier: MIT
"""The colours a LEGO part exists in, and the LEGO element numbers that
name each part-and-colour, from Rebrickable's public data files.

A LEGO part number names a mould; the part in a given colour is a
separate *element* with its own number (the one set inventories and
instructions print), so a beam 15 in dark bluish gray is element
4210687 while the same beam in red is 4163147. Rebrickable
(https://rebrickable.com/downloads/, CC BY 4.0) publishes the tables
that tie them together: ``colors.csv`` (id, name, RGB) and
``elements.csv`` (element id → part number and colour id).

``python -m openbricks_sim.bricks.rebrickable OUT.json`` downloads the
two tables and writes, for every part the shipped bundle holds, the
colours it comes in with their element numbers, plus the palette those
colours draw with; the bundle build stamps the records from it
(``ldraw.apply_colors``). Rebrickable spells a few numbers with a mould
suffix LDraw does without (``3648b`` for LDraw's ``3648``); the map of
those lives in the output too, so the next run keeps it.
"""
import csv
import gzip
import io
import json
import sys
import urllib.request

from openbricks_sim import bricks

DOWNLOADS = "https://cdn.rebrickable.com/media/downloads/"
SOURCE = "Rebrickable's public data files (rebrickable.com/downloads, CC BY 4.0): colors.csv and elements.csv"

# LDraw number → Rebrickable's, where Rebrickable adds a mould suffix
# or names the part differently. The sets' aliases (sets.json) are
# folded in at run time.
REBRICKABLE_NUMBERS = {
    "33299": "33299b",
    "32064": "32064c",
    "32556": "32556a",
    "32123": "32123b",
    "6538": "6538b",
    "3648": "3648b",
    "62821": "62821b",
    "3650": "3650c",
}


def fetch_table(name, opener=urllib.request.urlopen):
    """One of Rebrickable's tables as a list of dict rows."""
    req = urllib.request.Request(DOWNLOADS + name + ".csv.gz", headers={"User-Agent": bricks.USER_AGENT})
    with opener(req) as resp:
        raw = resp.read()
    text = gzip.GzipFile(fileobj=io.BytesIO(raw)).read().decode("utf-8")
    return list(csv.DictReader(io.StringIO(text)))


def rebrickable_numbers(sets):
    """LDraw number → Rebrickable's, from the fixed map and the sets' aliases."""
    out = dict(REBRICKABLE_NUMBERS)
    for s in sets.values():
        for other, ldraw in s.get("aliases", {}).items():
            out[ldraw] = other
    return out


def build(numbers, colors_rows, elements_rows, rebrickable=None):
    """``{"palette": {colour id: {name, rgb, trans}}, "parts": {LDraw
    number: {colour id: [element ids]}}, "rebrickable": {...}, "without":
    [numbers with no data]}`` for the given LDraw numbers."""
    rebrickable = rebrickable or {}
    colors = {r["id"]: r for r in colors_rows}
    by_part, by_design = {}, {}
    for r in elements_rows:
        by_part.setdefault(r["part_num"], {}).setdefault(r["color_id"], []).append(r["element_id"])
        if r.get("design_id"):
            by_design.setdefault(r["design_id"], {}).setdefault(r["color_id"], []).append(r["element_id"])
    parts, without, used = {}, [], set()
    for num in numbers:
        # Rebrickable's spelling of the number, the number itself, then the
        # LEGO design id its elements carry (a number Rebrickable spells
        # with a mould suffix it does not know is still that design)
        entry = by_part.get(rebrickable.get(num, num)) or by_part.get(num) or by_design.get(num)
        if not entry:
            without.append(num)
            continue
        parts[num] = {cid: sorted(els, key=int) for cid, els in sorted(entry.items(), key=lambda kv: int(kv[0]))}
        used.update(entry)
    palette = {}
    for cid in sorted(used, key=int):
        c = colors[cid]
        palette[cid] = {"name": c["name"], "rgb": c["rgb"], "trans": c["is_trans"] in ("t", "True", "true", "1")}
    return {"source": SOURCE, "rebrickable": {k: v for k, v in sorted(rebrickable.items()) if k in numbers},
            "palette": palette, "parts": parts, "without": without}


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print("usage: python -m openbricks_sim.bricks.rebrickable OUT.json", file=sys.stderr)
        return 2
    numbers = sorted(bricks.load_bundle()["parts"])
    data = build(numbers, fetch_table("colors"), fetch_table("elements"), rebrickable_numbers(bricks.load_sets()))
    with open(argv[0], "w") as fh:
        json.dump(data, fh, indent=1)
        fh.write("\n")
    n_el = sum(len(e) for p in data["parts"].values() for e in p.values())
    print("parts with colours: %d, colours: %d, element numbers: %d, without data: %s"
          % (len(data["parts"]), len(data["palette"]), n_el, data["without"] or "none"))
    return 0


if __name__ == "__main__":         # pragma: no cover
    sys.exit(main())
