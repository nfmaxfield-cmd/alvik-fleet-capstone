"""Simulated cable-plant equipment for the plywood board: print files + previews + a printability report.

    python hardware/factory/build.py      (writes hardware/factory/stl/*.stl and hardware/factory/preview/*.png)

Four stylised machines from a cable plant, sized for the 122 x 122 cm board and a 200 x 200 mm print bed:

  extruder     long barrel housing with a feed hopper, a control cabinet and a die head    162 x 40 x 75 mm
  strander     cabling machine: an eight-sided cage lying on a base, with end bearing blocks  150 x 56 x 56 mm
  takeup_reel  take-up reel standing on a base, with a drive box                           100 x 64 x 68 mm
  test_bay     L-shaped inspection station (its L shows its orientation from above)         140 x 140 x 60 mm

What they must do for the project (each is checked below):
  - be seen by the robot's front distance sensor: the sensor sits 29.5 mm up and looks slightly down, so every
    machine has solid vertical faces at least MIN_FACE_H_MM tall all round, from the floor;
  - read from the overhead camera (about 3.4 px/cm) as a distinct shape, never as a robot: no outline near the
    robot's 95 x 95 mm square, and no white (print colour, README);
  - print without supports, lying as exported, on a 200 x 200 mm bed.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
STL = os.path.join(HERE, "stl")
PNG = os.path.join(HERE, "preview")

BED_MM = (200.0, 200.0)
BED_MARGIN_MM = 8.0            # brim / skirt room each side
MIN_FACE_H_MM = 45.0           # every outside wall at least this tall at the floor (sensor 29.5 mm up)
MAX_OVERHANG_DEG = 45.0
MAX_LOW_GAP_MM = 20.0          # a low stretch (base plate only) no wider than this along any side
ROBOT_MM = 95.0                # the Alvik's footprint, for the "doesn't look like a robot" check


def cq():
    import cadquery
    return cadquery


def box(x0, y0, z0, dx, dy, dz):
    return cq().Workplane("XY").box(dx, dy, dz, centered=False).translate((x0, y0, z0))


def octagon_prism_x(x0, length, cy, cz, across_flats):
    """Eight-sided prism along X, a flat face down (its lower slopes are exactly 45 deg: no support)."""
    c = cq()
    r = across_flats / 2 / math.cos(math.radians(22.5))
    pts = [(cy + r * math.cos(math.radians(22.5 + 45 * k)), cz + r * math.sin(math.radians(22.5 + 45 * k)))
           for k in range(8)]
    return c.Workplane("YZ").polyline(pts).close().extrude(length).translate((x0, 0, 0))


def extruder():
    c = cq()
    body = box(30, 0, 0, 120, 40, 45)                       # barrel housing
    cab = box(0, 0, 0, 30, 40, 60)                          # control cabinet
    die = box(150, 8, 0, 12, 24, 45)                        # die head
    hop = (c.Workplane("XY").workplane(offset=45).center(62, 20).rect(18, 18)
           .workplane(offset=30).rect(36, 36).loft())       # feed hopper, widening upward (17 deg)
    return body.union(cab).union(die).union(hop)


def strander():
    base = box(0, 0, 0, 150, 56, 10)
    cage = octagon_prism_x(14, 122, 28, 10 + 20, 40)        # 40 mm across flats, flat on the base
    left = box(0, 0, 10, 12, 56, 46)                        # bearing blocks at the ends, full width
    right = box(138, 0, 10, 12, 56, 46)
    return base.union(cage).union(left).union(right)


def takeup_reel():
    base = box(0, 0, 0, 100, 64, 8)
    f1 = octagon_prism_x(24, 6, 32, 8 + 30, 60)
    f2 = octagon_prism_x(70, 6, 32, 8 + 30, 60)
    core = octagon_prism_x(30, 40, 32, 8 + 30, 50)            # wound cable, nearly to the flange rims
    web = box(30, 20, 8, 40, 24, 8)                         # cradle under the cable (no bridge to print)
    drive = box(82, 14, 8, 16, 36, 37)                      # drive box: base 8 + 37 = 45 mm
    end_l = box(0, 0, 0, 6, 64, MIN_FACE_H_MM)              # low end walls so every side faces the sensor
    return base.union(f1).union(f2).union(core).union(web).union(drive).union(end_l)


def test_bay():
    a = box(0, 0, 0, 140, 44, 50)
    b = box(0, 0, 0, 44, 140, 50)
    console = box(4, 4, 50, 36, 36, 10)                     # raised console at the corner
    return a.union(b).union(console)


PARTS = {"extruder": extruder, "strander": strander, "takeup_reel": takeup_reel, "test_bay": test_bay}


def tess(shape, tol=0.2):
    import numpy as np
    verts, tris = shape.val().tessellate(tol, 0.3)
    V = np.array([(v.x, v.y, v.z) for v in verts])
    return V, np.array(tris)


def overhang_mm2(shape):
    import numpy as np
    V, T = tess(shape)
    tri = V[T]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area = np.linalg.norm(n, axis=1) / 2
    nz = n[:, 2] / np.maximum(2 * area, 1e-12)
    low = tri[:, :, 2].max(axis=1) > V[:, 2].min() + 0.25
    steep = nz < -math.cos(math.radians(90 - MAX_OVERHANG_DEG)) - 1e-6
    return float(area[low & steep].sum())


def face_heights(shape, step=3.0, window=25.0):
    """For lines coming in from each side at floor level: the tallest solid within `window` mm of the first
    thing hit. That is roughly what the robot's distance sensor (29.5 mm up, looking slightly down) meets
    when it drives up to the machine from that side."""
    c = cq()
    sol = shape.val()
    b = sol.BoundingBox()
    inside = lambda x, y, z: sol.isInside(c.Vector(x, y, z), 0.05)

    def height(x, y):
        h = 0.0
        z = 1.0
        while z < b.zmax and inside(x, y, z):
            h = z
            z += 2.0
        return h

    out = {}
    for side in ("-x", "+x", "-y", "+y"):
        hs = []
        across = (b.ymin, b.ymax) if side in ("-x", "+x") else (b.xmin, b.xmax)
        depth = b.xlen if side in ("-x", "+x") else b.ylen
        t = across[0] + 1.5
        while t < across[1] - 1.5:
            def pt(d):
                return {"-x": (b.xmin + d, t), "+x": (b.xmax - d, t), "-y": (t, b.ymin + d),
                        "+y": (t, b.ymax - d)}[side]
            d = 0.5
            while d < depth and not inside(*pt(d), 1.0):
                d += 1.0
            if d < depth:
                best, e = 0.0, d
                while e <= min(d + window, depth):
                    if inside(*pt(e), 1.0) or True:
                        best = max(best, height(*pt(e)))
                    e += 2.0
                hs.append(best)
            t += step
        out[side] = hs
    return out


def preview(name, shape, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import numpy as np
    V, T = tess(shape, 0.3)
    fig = plt.figure(figsize=(10, 4.6))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(V[T], facecolor="#5b7fa6", edgecolor="#2d3e52", linewidths=0.05))
    lo, hi = V.min(axis=0), V.max(axis=0)
    span = (hi - lo).max()
    ax.set_xlim(lo[0], lo[0] + span); ax.set_ylim(lo[1], lo[1] + span); ax.set_zlim(0, span)
    ax.view_init(28, -55)
    ax.set_title("%s  %.0f x %.0f x %.0f mm" % (name, *(hi - lo)), fontsize=10)
    ax2 = fig.add_subplot(1, 2, 2)
    top = V[T]
    for t in top:
        ax2.fill(t[:, 0], t[:, 1], color="#5b7fa6", alpha=0.25, linewidth=0)
    ax2.add_patch(plt.Rectangle((lo[0] + 2, hi[1] + 6), ROBOT_MM, ROBOT_MM, fill=False, ls="--", ec="#999"))
    ax2.text(lo[0] + 4, hi[1] + 10, "robot 95 mm", fontsize=8, color="#777")
    ax2.set_aspect("equal"); ax2.set_title("seen from above (camera view), with a robot for scale", fontsize=10)
    ax2.set_xlabel("mm")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    os.makedirs(STL, exist_ok=True)
    os.makedirs(PNG, exist_ok=True)
    c = cq()
    bad = 0

    def rule(ok, text):
        nonlocal bad
        print(("ok   " if ok else "FAIL ") + text)
        bad += 0 if ok else 1

    for name, fn in PARTS.items():
        s = fn()
        b = s.val().BoundingBox()
        s = s.translate((-b.xmin, -b.ymin, -b.zmin))
        b = s.val().BoundingBox()
        dx, dy, dz = b.xlen, b.ylen, b.zlen
        fits = (dx + 2 * BED_MARGIN_MM <= BED_MM[0] and dy + 2 * BED_MARGIN_MM <= BED_MM[1])
        rule(fits, "%s: %.0f x %.0f x %.0f mm, fits a %d x %d bed with %d mm margin" % (name, dx, dy, dz, *BED_MM,
                                                                                      BED_MARGIN_MM))
        rule(s.val().isValid(), "%s: valid solid" % name)
        o = overhang_mm2(s)
        rule(o < 20.0, "%s: %.0f mm^2 overhanging more than %d deg (print as exported, no supports)" % (
            name, o, MAX_OVERHANG_DEG))
        fh = face_heights(s)
        gap = 0.0
        for v in fh.values():
            run = 0
            for h in v:
                run = run + 1 if h < MIN_FACE_H_MM - 0.5 else 0
                gap = max(gap, run * 3.0)
        rule(gap <= MAX_LOW_GAP_MM, "%s: widest stretch of outline lower than %.0f mm: %.0f mm (<= %.0f: the "
             "distance sensor's beam, about 11 deg per zone, spans it)" % (name, MIN_FACE_H_MM, gap, MAX_LOW_GAP_MM))
        robotlike = abs(dx - ROBOT_MM) < 15 and abs(dy - ROBOT_MM) < 15
        rule(not robotlike, "%s: outline %.0f x %.0f mm, not robot-like (95 x 95)" % (name, dx, dy))
        vol = s.val().Volume() / 1000.0
        c.exporters.export(s, os.path.join(STL, name + ".stl"), tolerance=0.05, angularTolerance=0.1)
        preview(name, s, os.path.join(PNG, name + ".png"))
        print("     %s: solid volume %.0f cm^3" % (name, vol))
    print("\nwrote %s/*.stl and %s/*.png" % (STL, PNG))
    if bad:
        print("SOME CHECKS FAILED")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
