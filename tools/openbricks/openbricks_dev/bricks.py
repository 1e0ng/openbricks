# SPDX-License-Identifier: MIT
"""``openbricks bricks``: the LEGO brick library behind the workbench.

    openbricks bricks fetch                 # the full LDraw library into the cache (145 MB)
    openbricks bricks convert 32270 3648    # part numbers -> a bundle the workbench loads
    openbricks sim workbench --bricks more.json

The curated Technic set ships in the wheel; ``fetch`` + ``convert``
reach every other part LDraw catalogues. ``convert`` needs numpy, which
``pip install openbricks[sim]`` brings in; ``fetch`` needs nothing.
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
    f = bs.add_parser("fetch", help="Download and unpack the LDraw parts library (145 MB).")
    f.add_argument("--dest", default=None, metavar="DIR",
                   help="Where to unpack it. Default: $OPENBRICKS_LDRAW_DIR, "
                        "else ~/.cache/openbricks/ldraw.")
    f.add_argument("--force", action="store_true",
                   help="Download again even if the library is already there.")
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
    root = bricks.fetch_library(dest=args.dest, force=args.force, progress=print)
    print("convert parts with: openbricks bricks convert NUMBER ... --out more.json")
    return 0 if bricks.library_present(root) else 1


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
