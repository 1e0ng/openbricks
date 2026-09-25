# SPDX-License-Identifier: MIT
"""``openbricks bricks``: the LEGO brick library behind the workbench.

    openbricks bricks fetch 2458 3005       # these parts, file by file, into the sim's library
    openbricks bricks fetch                 # the full LDraw library into the cache (145 MB)
    openbricks bricks convert 32270 3648    # part numbers -> a bundle the workbench loads
    openbricks sim workbench --bricks more.json

The curated Technic set ships in the wheel; ``fetch NUMBER`` gets any
other part from ldraw.org with the few files it needs and keeps it
under the sim's data directory, where ``openbricks sim`` finds it on
every launch (the sim's library does the same from its search box);
``fetch`` alone brings the whole library, and ``convert`` turns numbers
from it into a bundle. ``fetch NUMBER`` and ``convert`` need numpy,
which ``pip install openbricks[sim]`` brings in.
"""
import json
import sys


def add_parser(sub):
    p = sub.add_parser(
        "bricks",
        help="The LEGO brick library: fetch the LDraw parts library, "
             "convert parts for the workbench.",
        description="The Assembly Workbench (``openbricks sim``) ships a "
                    "curated set of LEGO Technic parts with exact LDraw "
                    "geometry. ``bricks fetch`` downloads the whole LDraw "
                    "library into a cache; ``bricks convert`` turns any "
                    "part numbers into a bundle file for ``openbricks sim "
                    "workbench --bricks``.",
    )
    bs = p.add_subparsers(dest="bricks_command", metavar="ACTION")
    bs.required = True
    f = bs.add_parser("fetch", help="Fetch parts by number from ldraw.org into the sim's library, "
                                    "or (with no numbers) the whole LDraw parts library (145 MB).")
    f.add_argument("numbers", nargs="*", metavar="NUMBER",
                   help="LDraw part numbers (the LEGO design id, e.g. 2458): each is fetched with "
                        "the files it needs and kept under the sim's data directory. None: the "
                        "whole library.")
    f.add_argument("--dest", default=None, metavar="DIR",
                   help="The LDraw cache: where the whole library is unpacked, and where parts fetched "
                        "by number keep their files. Default: $OPENBRICKS_LDRAW_DIR, else "
                        "~/.cache/openbricks/ldraw.")
    f.add_argument("--force", action="store_true",
                   help="Download again: the whole library even if it is already there; a part's files "
                        "and Rebrickable's colour tables even if the cache has them.")
    f.add_argument("--no-colors", action="store_true",
                   help="Fetch parts without the colours Rebrickable lists for them.")
    c = bs.add_parser("convert", help="Convert LDraw part numbers into a brick bundle.")
    c.add_argument("numbers", nargs="+", metavar="NUMBER",
                   help="LDraw part numbers (the LEGO design id, e.g. 32270).")
    c.add_argument("--out", default=None, metavar="FILE",
                   help="Write the bundle JSON here (default: stdout).")
    c.add_argument("--weights", default=None, metavar="FILE",
                   help="JSON of {number: {\"g\": grams}} to record as the "
                        "parts' masses; unknown parts get a volume estimate.")
    c.add_argument("--ldraw", default=None, metavar="DIR",
                   help="The unpacked LDraw library (default: the cache "
                        "``bricks fetch`` fills).")
    return p


def run(args):
    if args.bricks_command == "fetch":
        return _fetch(args)
    if args.bricks_command == "convert":
        return _convert(args)
    print("error: unknown bricks action %r" % (args.bricks_command,), file=sys.stderr)
    return 2


def _fetch(args):
    from openbricks_sim import bricks
    if args.numbers:
        return _fetch_parts(args)
    root = bricks.fetch_library(dest=args.dest, force=args.force, progress=print)
    print("convert parts with: openbricks bricks convert NUMBER ... --out more.json")
    return 0 if bricks.library_complete(root) else 1


def _fetch_parts(args):
    from openbricks_sim.bricks import fetch
    try:
        import openbricks_sim.bricks.ldraw  # noqa: F401
    except ImportError:
        print("error: ``openbricks bricks fetch NUMBER`` needs numpy: pip install openbricks[sim]", file=sys.stderr)
        return 1
    from openbricks_sim import bricks
    out_dir = fetch.bricks_dir()
    shipped = bricks.load_bundle()["parts"]
    failed = 0
    for number in args.numbers:
        if number in shipped:
            # the shipped record (weighed, in its sets) wins over a fetched one: nothing to fetch
            print("error: %s is in the library already (%s)" % (number, shipped[number].get("name", "")), file=sys.stderr)
            failed += 1
            continue
        try:
            bundle = fetch.fetch_part(number, root=args.dest, say=lambda s: print("  " + s), colors=not args.no_colors,
                                      force=args.force)
            out_dir.mkdir(parents=True, exist_ok=True)
            out = out_dir / (number + ".json")
            tmp = out.with_name(out.name + ".part")
            tmp.write_text(json.dumps(bundle, separators=(",", ":")))
            tmp.replace(out)
        except (fetch.NotInLibrary, fetch.FetchError, OSError) as e:
            print("error: %s" % e, file=sys.stderr)
            failed += 1
            continue
        print("%s %s -> %s" % (number, bundle["parts"][number]["name"], out))
    if not failed:
        print("in the sim's library from its next launch: openbricks sim")
    return 1 if failed else 0


def _convert(args):
    from openbricks_sim import bricks
    try:
        import openbricks_sim.bricks.ldraw as ldraw
    except ImportError:
        print("error: ``openbricks bricks convert`` needs numpy: pip install openbricks[sim]", file=sys.stderr)
        return 1
    root = args.ldraw or bricks.ldraw_dir()
    if not bricks.library_present(root):
        print("error: no LDraw library at %s - run: openbricks bricks fetch" % root, file=sys.stderr)
        return 1
    weights = None
    if args.weights:
        with open(args.weights) as fh:
            weights = json.load(fh)
    lib = ldraw.Library(str(root))
    bundle = ldraw.convert_parts(lib, args.numbers, weights, log=lambda s: print(s, file=sys.stderr))
    if bundle["missing"]:
        print("not in the library: " + ", ".join(bundle["missing"]), file=sys.stderr)
    text = json.dumps(bundle, separators=(",", ":"))
    if args.out:
        with open(args.out, "w") as fh:
            fh.write(text)
        print("%d part(s) -> %s; open with: openbricks sim workbench --bricks %s" % (len(bundle["parts"]), args.out, args.out), file=sys.stderr)
    else:
        print(text)
    return 0 if bundle["parts"] else 1
