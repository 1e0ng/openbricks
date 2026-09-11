# SPDX-License-Identifier: MIT
"""A tiny synthetic LDraw library for the brick tests.

Every face is written with LDraw's CCW convention, which in LDraw's
right-handed frame puts the right-hand-rule normal on the outside of
the solid — the same convention the converter relies on for the sign
of the volume.

Parts (numbers no real part uses):

* ``9999`` — a closed box 40 × 20 × 10 LDU (16 × 4 × 8 mm in the
  converter's frame: X stays, LDraw Z becomes Y, LDraw -Y becomes Z).
* ``9998`` — two boxes, one referenced through a mirror matrix, 40 LDU
  apart (exercises the determinant-flip winding rule).
* ``8888`` — ``~Moved to 9999`` alias.
* ``7777`` — a ring: outer radius 15 LDU, bore radius 6 LDU (the pin
  radius), 20 LDU (8 mm) tall — one pin hole for the bore detector.
* ``6666`` — a part made of the fake pin primitive ``confric5.dat``.

Primitives: ``p/confric5.dat`` is a closed 16-gon cylinder of radius 6
LDU from y = 0 down to y = -20 (a pin half, tip at -Y).
"""
import math
import os


def _quad(pts, outward):
    """A type-4 line whose right-hand normal points along ``outward``."""
    a, b, c, d = pts
    n = _cross(_sub(b, a), _sub(c, a))
    if _dot(n, outward) < 0:
        a, b, c, d = a, d, c, b
    return "4 16 " + " ".join("%g %g %g" % tuple(p) for p in (a, b, c, d))


def _sub(u, v):
    return (u[0] - v[0], u[1] - v[1], u[2] - v[2])


def _cross(u, v):
    return (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])


def _dot(u, v):
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]


def box_lines(hx, hy, hz):
    """Six outward quads of a box with half-sizes hx, hy, hz."""
    lines = []
    for axis, sign in ((0, 1), (0, -1), (1, 1), (1, -1), (2, 1), (2, -1)):
        h = [hx, hy, hz]
        pts = []
        for i in range(4):
            u = (-1, 1, 1, -1)[i]
            v = (-1, -1, 1, 1)[i]
            p = [0, 0, 0]
            p[axis] = sign * h[axis]
            o1, o2 = [k for k in range(3) if k != axis]
            p[o1] = u * h[o1]
            p[o2] = v * h[o2]
            pts.append(tuple(p))
        out = [0, 0, 0]
        out[axis] = sign
        lines.append(_quad(pts, tuple(out)))
    return lines


def cylinder_lines(radius, y0, y1, segments=16, inward=False, caps=True):
    """A cylinder along Y between y0 and y1; ``inward`` faces make a bore."""
    lines = []
    ring = [(radius * math.cos(2 * math.pi * i / segments), radius * math.sin(2 * math.pi * i / segments)) for i in range(segments)]
    for i in range(segments):
        (x0, z0), (x1, z1) = ring[i], ring[(i + 1) % segments]
        mid = ((x0 + x1) / 2, 0, (z0 + z1) / 2)
        outward = (-mid[0], 0, -mid[2]) if inward else (mid[0], 0, mid[2])
        lines.append(_quad([(x0, y0, z0), (x1, y0, z1), (x1, y1, z1), (x0, y1, z0)], outward))
    if caps:
        for y, outward in ((y0, (0, -1, 0)), (y1, (0, 1, 0))):
            for i in range(segments):
                (x0, z0), (x1, z1) = ring[i], ring[(i + 1) % segments]
                lines.append(_quad([(0, y, 0), (x0, y, z0), (x1, y, z1), (0, y, 0)], outward).replace("4 16", "4 16", 1))
    return lines


def ring_lines(r_out, r_in, y0, y1, segments=16):
    """A tube: outer wall outward, bore wall inward, annular ends."""
    lines = cylinder_lines(r_out, y0, y1, segments, caps=False) + cylinder_lines(r_in, y0, y1, segments, inward=True, caps=False)
    outer = [(r_out * math.cos(2 * math.pi * i / segments), r_out * math.sin(2 * math.pi * i / segments)) for i in range(segments)]
    inner = [(r_in * math.cos(2 * math.pi * i / segments), r_in * math.sin(2 * math.pi * i / segments)) for i in range(segments)]
    for y, outward in ((y0, (0, -1, 0)), (y1, (0, 1, 0))):
        for i in range(segments):
            j = (i + 1) % segments
            lines.append(_quad([(outer[i][0], y, outer[i][1]), (outer[j][0], y, outer[j][1]), (inner[j][0], y, inner[j][1]), (inner[i][0], y, inner[i][1])], outward))
    return lines


def _write(path, title, body):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        fh.write("0 " + title + "\n0 Name: " + os.path.basename(path) + "\n0 BFC CERTIFY CCW\n\n")
        fh.write("\n".join(body) + "\n")


def write_mini_library(root):
    """Populate ``root`` and return it."""
    parts = os.path.join(root, "parts")
    prims = os.path.join(root, "p")
    _write(os.path.join(parts, "9999.dat"), "Test Box 40 x 20 x 10", box_lines(20, 10, 5))
    _write(os.path.join(parts, "9998.dat"), "Test Two Boxes, One Mirrored", [
        "1 16 40 0 0 1 0 0 0 1 0 0 0 1 9999.dat",
        "1 16 -40 0 0 -1 0 0 0 1 0 0 0 1 9999.dat",
    ])
    _write(os.path.join(parts, "8888.dat"), "~Moved to 9999", ["1 16 0 0 0 1 0 0 0 1 0 0 0 1 9999.dat"])
    _write(os.path.join(parts, "7777.dat"), "Test Ring with Pin Hole", ring_lines(15, 6, -10, 10))
    _write(os.path.join(prims, "confric5.dat"), "Technic Friction Pin 1.0 Slotted with Split Base Collar (test stand-in)",
           cylinder_lines(6, -20, 0))
    _write(os.path.join(parts, "6666.dat"), "Test Pin from a Primitive", [
        "1 16 0 0 0 1 0 0 0 1 0 0 0 1 confric5.dat",
        "1 16 0 0 0 -1 0 0 0 -1 0 0 0 1 confric5.dat",          # the other half, mirrored about the collar
    ])
    # a shelled box: the outer box, and the inner one turned inside out with INVERTNEXT
    _write(os.path.join(parts, "5555.dat"), "Test Shelled Box", [
        "1 16 0 0 0 1 0 0 0 1 0 0 0 1 9999.dat",
        "0 BFC INVERTNEXT",
        "1 16 0 0 0 0.5 0 0 0 0.5 0 0 0 0.5 9999.dat",
    ])
    # connector primitives that carry only edge lines or nothing at all, and studs
    _write(os.path.join(prims, "axlehol2.dat"), "Technic Axle Hole Side Edges (test stand-in)",
           ["2 24 6 -10 0 6 10 0", "2 24 -6 -10 0 -6 10 0"])
    _write(os.path.join(prims, "axlehol0.dat"), "Technic Axle Hole Hint (test stand-in, no geometry)", ["0 // nothing"])
    _write(os.path.join(prims, "stud.dat"), "Stud (test stand-in)", cylinder_lines(6, -4, 0))
    _write(os.path.join(parts, "4444.dat"), "Test Brick with Edge-Only Axle Hole and Two Studs",
           box_lines(20, 12, 20) + [
               "1 16 0 0 0 1 0 0 0 1 0 0 0 1 axlehol2.dat",
               "1 16 0 0 0 1 0 0 0 1 0 0 0 1 axlehol0.dat",
               "1 16 10 -12 0 1 0 0 0 1 0 0 0 1 stud.dat",
               "1 16 -10 -12 0 1 0 0 0 1 0 0 0 1 stud.dat",
               "1 16 0 0 0 1 0 0 0 1 0 0 0 1 nothing-here.dat",   # a missing reference is skipped
           ])
    # a part that references itself: the depth guard must end the walk
    _write(os.path.join(parts, "3333.dat"), "Test Self Reference", box_lines(5, 5, 5) + ["1 16 0 0 0 1 0 0 0 1 0 0 0 1 3333.dat"])
    # malformed lines of every kind next to two valid triangles
    _write(os.path.join(parts, "2222.dat"), "Test Garbage Tolerance", [
        "1 16 x 0 0 1 0 0 0 1 0 0 0 1 9999.dat",
        "4 16 0 0 0 1 0 0 1 1 0 oops 1 0",
        "2 24 0 0 0 1 x 0",
        "5 24 0 0 0 1 0 0 0 1 0 1 1 0",
        "3 16 0 0 0 10 0 0 0 10 0",
        "3 16 0 0 0 0 10 0 0 0 10",
    ])
    # lines only: no faces, so no brick
    _write(os.path.join(parts, "1111.dat"), "Test Lines Only", ["2 24 0 0 0 10 0 0"])
    with open(os.path.join(parts, "readme.txt"), "w") as fh:
        fh.write("not a part\n")
    return root
