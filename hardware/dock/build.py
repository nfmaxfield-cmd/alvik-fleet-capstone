"""Build the home dock, the nose bumper, the camera check tile and the LED holder from dock_params.py:
STL files to print, a PNG preview of each, and a printability report.

    pip install cadquery matplotlib numpy      (cadquery 2.4+; on Linux add --break-system-packages if pip asks)
    python hardware/dock/build.py               (writes hardware/dock/stl/ and hardware/dock/preview/)

Every number comes from dock_params.py; change it there and run this again. It prints the three console lines
that put the bumper's numbers into the firmware, and FAILS (exit 1) if a part does not fit the bed, a wall is
thinner than MIN_WALL_MM, the bumper would cover the distance sensor, or the pocket is too deep for a
crooked robot to reach the back wall.

Print orientation: each STL is exported already lying on the bed the way that needs the least support (of five
tries); the report says which way and how much overhang is left. The PNG previews show the part as drawn
(robot / dock coordinates).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dock_params as D  # noqa: E402

STL = os.path.join(HERE, "stl")
PNG = os.path.join(HERE, "preview")


def cq():
    import cadquery
    return cadquery


def cyl(r, h, p, d):
    """A cylinder of radius r and length h from point p along direction d, as a Workplane."""
    c = cq()
    return c.Workplane("XY").add(c.Solid.makeCylinder(r, h, c.Vector(*p), c.Vector(*d)))


# ============================================================================= the dock
def dock_solid():
    """Dock, in dock coordinates (mm): X into the dock, back wall inner face at X = 0, Y left, Z up."""
    c = cq()
    hw, t, H = D.POCKET_W_MM / 2, D.WALL_T_MM, D.WALL_H_MM
    xp = -D.POCKET_DEPTH_MM
    xm = D.MOUTH_X_MM
    fl = D.FUNNEL_FLARE_MM
    # back block
    body = c.Workplane("XY").box(D.BACK_BLOCK_DEPTH_MM, 2 * (hw + t), H, centered=(False, True, False))
    # pocket side walls
    for sg in (1, -1):
        w = (c.Workplane("XY").center(xp / 2, sg * (hw + t / 2)).box(-xp, t, H, centered=(True, True, False)))
        body = body.union(w)
        pts = [(xm, sg * (hw + fl)), (xp, sg * hw), (xp, sg * (hw + t)), (xm, sg * (hw + fl + t))]
        f = c.Workplane("XY").polyline(pts).close().extrude(H)
        body = body.union(f)
    # floor stripe (a thin bridge between the funnel walls; dark filament = the stripe)
    x0, x1 = D.STRIPE_X0_MM, D.STRIPE_X1_MM

    def half(x):
        return hw + fl * (xp - x) / D.FUNNEL_LEN_MM if x < xp else hw
    sp = [(x0, -half(x0) - t / 2), (x1, -half(x1) - t / 2), (x1, half(x1) + t / 2), (x0, half(x0) + t / 2)]
    stripe = c.Workplane("XY").polyline(sp).close().extrude(D.STRIPE_T_MM)
    try:
        stripe = stripe.edges(">Z and |Y").chamfer(D.STRIPE_T_MM * 0.8)
    except Exception:
        pass
    body = body.union(stripe)
    # camera check tile recess on top of the back block
    r = D.TILE_MM + 2 * D.TILE_RECESS_CLEAR_MM
    rec = (c.Workplane("XY").workplane(offset=H - D.TILE_T_MM + 0.2).center(D.TILE_CENTER_X_MM, 0)
           .rect(r, r).extrude(D.TILE_T_MM))
    body = body.cut(rec)
    # finger notch to lift the tile out
    notch = c.Workplane("XY").workplane(offset=H - D.TILE_T_MM).center(D.TILE_CENTER_X_MM + r / 2, 0).circle(4).extrude(5)
    body = body.cut(notch)
    # screw tabs: two beside the back block, one outside each funnel wall near the mouth
    tabs = [(D.BACK_BLOCK_DEPTH_MM / 2, hw + t + D.TAB_W_MM / 2), (D.BACK_BLOCK_DEPTH_MM / 2, -(hw + t + D.TAB_W_MM / 2)),
            (xm + D.TAB_L_MM / 2 + 2, hw + fl + t + D.TAB_W_MM / 2 - 2),
            (xm + D.TAB_L_MM / 2 + 2, -(hw + fl + t + D.TAB_W_MM / 2 - 2))]
    for (tx, ty) in tabs:
        tab = c.Workplane("XY").center(tx, ty).box(D.TAB_L_MM, D.TAB_W_MM + 2, D.TAB_T_MM, centered=(True, True, False))
        tab = tab.faces(">Z").workplane().center(0, (1 if ty > 0 else -1) * 1).cskHole(
            D.SCREW_HOLE_DIA_MM, D.SCREW_HEAD_DIA_MM, 90)
        body = body.union(tab)
    return body, tabs


def tile_solid():
    c = cq()
    s = D.TILE_MM / D.TILE_SQUARES
    base = c.Workplane("XY").box(D.TILE_MM, D.TILE_MM, D.TILE_T_MM, centered=(True, True, False))
    for i in range(D.TILE_SQUARES):
        for j in range(D.TILE_SQUARES):
            if (i + j) % 2:
                continue
            sq = (c.Workplane("XY").workplane(offset=D.TILE_T_MM)
                  .center(-D.TILE_MM / 2 + (i + 0.5) * s, -D.TILE_MM / 2 + (j + 0.5) * s)
                  .box(s, s, D.TILE_RAISE_MM, centered=(True, True, False)))
            base = base.union(sq)
    return base


# ============================================================================= the nose bumper
def bumper_solid(mode):
    """Nose bumper in robot coordinates: X forward from the robot's front face, Y left, Z up from the floor.
    mode "m3": screws through the back plate into the robot's front M3 holes; "technic": side arms with
    Technic pin holes."""
    c = cq()
    W = D.ROBOT_WIDTH_MM
    z0, z1 = D.BUMPER_Z0_MM, D.BUMPER_Z1_MM
    Hh = z1 - z0
    depth = D.FACE_X_MM
    body = (c.Workplane("XY").workplane(offset=z0).center(depth / 2, 0)
            .box(depth, W, Hh, centered=(True, True, False)))
    # chamfer the two front vertical corner edges
    body = body.edges("|Z and >X").chamfer(D.CORNER_CHAMFER_MM)
    # switch pockets: open to the front and the top, a shelf below
    for sg in (1, -1):
        yo = sg * D.SWITCH_Y_OUTER_MM
        yi = sg * (D.SWITCH_Y_OUTER_MM - D.SWITCH_L_MM - 0.4)
        y0, y1 = min(yo, yi), max(yo, yi)
        # body pocket (switch body + 0.2 mm clearance), terminals behind it, lever slot in front
        pocket = (c.Workplane("XY").workplane(offset=z0 + D.SHELF_T_MM)
                  .center(D.BACKPLATE_T_MM + (depth - D.BACKPLATE_T_MM) / 2 + 0.5, (y0 + y1) / 2)
                  .box(depth - D.BACKPLATE_T_MM + 1.0, (y1 - y0) + 0.4, Hh, centered=(True, True, False)))
        body = body.cut(pocket)
        # screw holes through the shelf, M2 self-tapping into the plastic (pilot 1.8) - the switch's own holes
        # take the screw; ASSUMED hole layout (dock_params)
        ym = sg * D.SWITCH_LATERAL_MM
        for dy in (-D.SWITCH_HOLE_SPACING_MM / 2, D.SWITCH_HOLE_SPACING_MM / 2):
            hole = (c.Workplane("XY").workplane(offset=z0 - 1).center(D.SWITCH_BACK_X_MM + D.SWITCH_HOLE_X_MM, ym + dy)
                    .circle(0.9).extrude(D.SHELF_T_MM + 2))
            body = body.cut(hole)
        # wire channel through the back plate's top edge, behind the terminals
        ch = (c.Workplane("XY").workplane(offset=z1 - 3).center(D.BACKPLATE_T_MM / 2, ym)
              .box(D.BACKPLATE_T_MM + 1, 4, 4, centered=(True, True, False)))
        body = body.cut(ch)
    if mode == "m3":
        top = max(z1, max(z for _, z in D.FRONT_M3_HOLES_MM) + D.M3_HEAD_DIA_MM / 2 + 1.5)
        for (y, z) in D.FRONT_M3_HOLES_MM:
            if z + D.M3_HEAD_DIA_MM / 2 + 1.5 > z1:          # a tab up the robot's face beside the ToF window
                tab = (c.Workplane("XY").workplane(offset=z1 - 1).center(D.BACKPLATE_T_MM / 2, y)
                       .box(D.BACKPLATE_T_MM, D.M3_HEAD_DIA_MM + 3, top - z1 + 1, centered=(True, True, False)))
                body = body.union(tab)
            hole = cyl(D.M3_CLEAR_DIA_MM / 2, D.BACKPLATE_T_MM + 2, (-1, y, z), (1, 0, 0))
            cb = cyl(D.M3_HEAD_DIA_MM / 2 + 0.2, depth, (D.BACKPLATE_T_MM, y, z), (1, 0, 0))
            body = body.cut(hole).cut(cb)
    else:
        # a strap per Technic hole: up the front face (in front of it, beside the distance sensor), then back
        # over the top to the hole; a 4.85 mm hole for a friction pin
        t = D.BACKPLATE_T_MM
        top = D.ROBOT_TOP_Z_MM
        for (x, y) in D.TECHNIC_HOLES_MM:
            up = (c.Workplane("XY").workplane(offset=z1 - 1).center(t / 2, y)
                  .box(t, D.STRAP_W_MM, top + t - z1 + 1, centered=(True, True, False)))
            over = (c.Workplane("XY").workplane(offset=top).center((t - x - 6) / 2, y)
                    .box(t + x + 6, D.STRAP_W_MM, t, centered=(True, True, False)))
            body = body.union(up).union(over)
            body = body.cut(cyl(D.TECHNIC_PIN_HOLE_DIA_MM / 2, t + 2, (-x, y, top - 1), (0, 0, 1)))
    return body


def led_holder_solid():
    c = cq()
    xs = [p[0] for p in D.LED_POS_MM] + [p[0] for p in D.LED_MOUNT_TECHNIC_MM]
    ys = [p[1] for p in D.LED_POS_MM] + [p[1] for p in D.LED_MOUNT_TECHNIC_MM]
    m = D.LED_PLATE_MARGIN_MM
    x0, x1, y0, y1 = min(xs) - m, max(xs) + m, min(ys) - m, max(ys) + m
    plate = c.Workplane("XY").center((x0 + x1) / 2, (y0 + y1) / 2).box(x1 - x0, y1 - y0, D.LED_PLATE_T_MM,
                                                                        centered=(True, True, False))
    for (x, y) in D.LED_POS_MM:
        plate = plate.cut(c.Workplane("XY").workplane(offset=-1).center(x, y).circle(D.LED_DIA_MM / 2)
                          .extrude(D.LED_PLATE_T_MM + 2))
        plate = plate.cut(c.Workplane("XY").workplane(offset=D.LED_PLATE_T_MM - 1.0).center(x, y)
                          .circle(D.LED_FLANGE_DIA_MM / 2 + 0.1).extrude(2))
    for (x, y) in D.LED_MOUNT_TECHNIC_MM:
        plate = plate.cut(c.Workplane("XY").workplane(offset=-1).center(x, y)
                          .circle(D.TECHNIC_PIN_HOLE_DIA_MM / 2).extrude(D.LED_PLATE_T_MM + 2))
    # forward arrow notch on the edge (which way is front)
    plate = plate.cut(c.Workplane("XY").workplane(offset=-1).center(x1, (y0 + y1) / 2).polygon(3, 6).extrude(5))
    return plate


# ============================================================================= print orientation
ORIENTATIONS = [("as drawn (bottom on the bed)", None, 0), ("on its back (robot side on the bed)", (0, 1, 0), -90),
                 ("on its front", (0, 1, 0), 90), ("on its left side", (1, 0, 0), 90),
                 ("on its right side", (1, 0, 0), -90)]


def overhang_mm2(shape, max_deg=None):
    """Area (mm^2) of faces that look down more steeply than MAX_OVERHANG_DEG from vertical and are not on
    the bed: what would need support."""
    import numpy as np
    max_deg = D.MAX_OVERHANG_DEG if max_deg is None else max_deg
    verts, tris = shape.val().tessellate(0.2, 0.3)
    V = np.array([(v.x, v.y, v.z) for v in verts])
    T = np.array(tris)
    tri = V[T]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area = np.linalg.norm(n, axis=1) / 2
    nz = n[:, 2] / np.maximum(2 * area, 1e-12)
    zmin = V[:, 2].min()
    low = tri[:, :, 2].max(axis=1) > zmin + 0.25
    steep = nz < -math.cos(math.radians(90 - max_deg))
    return float(area[low & steep].sum())


def best_orientation(shape):
    """The orientation (of a few) that needs the least support; ties keep the earlier one. Returns
    (shape on the bed, description, overhang mm^2)."""
    best = None
    for desc, axis, deg in ORIENTATIONS:
        s = shape if axis is None else shape.rotate((0, 0, 0), axis, deg)
        b = s.val().BoundingBox()
        s = s.translate((-b.xmin, -b.ymin, -b.zmin))
        o = overhang_mm2(s)
        if best is None or o < best[2] - 1.0:
            best = (s, desc, o)
    return best


# ============================================================================= checks
def bbox(shape):
    b = shape.val().BoundingBox()
    return (b.xmin, b.ymin, b.zmin, b.xmax, b.ymax, b.zmax)


def checks(parts):
    """Printability and fit checks. Returns (ok, lines)."""
    lines, ok = [], True

    def rule(good, text):
        nonlocal ok
        ok = ok and good
        lines.append(("ok   " if good else "FAIL ") + text)

    bx, by = D.BED_MM
    for name, shape in parts.items():
        x0, y0, z0, x1, y1, z1 = bbox(shape)
        w, l = x1 - x0, y1 - y0
        fits = (w <= bx and l <= by) or (w <= by and l <= bx)
        rule(fits, "%s: %.1f x %.1f x %.1f mm, bed %.0f x %.0f" % (name, w, l, z1 - z0, bx, by))
    thin = [("dock walls", D.WALL_T_MM), ("bumper back plate", D.BACKPLATE_T_MM), ("switch shelf", D.SHELF_T_MM),
            ("bumper outer rib", D.FACE_RIB_MM), ("screw tabs", D.TAB_T_MM), ("LED plate", D.LED_PLATE_T_MM)]
    for what, t in thin:
        rule(t >= D.MIN_WALL_MM, "%s %.1f mm thick (>= %.1f)" % (what, t, D.MIN_WALL_MM))
    rule(D.TILE_T_MM >= 1.0, "check tile %.1f mm thick (thin by design; it lies in its recess)" % D.TILE_T_MM)
    # distance sensor: the bumper top must stay below its window, and below its lowest ray out to the face
    rule(D.BUMPER_Z1_MM <= D.TOF_WINDOW_BOTTOM_Z_MM - 2.0,
         "bumper top %.1f mm is 2 mm or more below the distance sensor window (%.1f mm)"
         % (D.BUMPER_Z1_MM, D.TOF_WINDOW_BOTTOM_Z_MM))
    ray_z = D.TOF_CENTER_Z_MM - (D.FACE_X_MM + D.TOF_RECESS_MM) * math.tan(math.radians(D.TOF_LOW_RAY_DEG))
    rule(D.BUMPER_Z1_MM + 1.0 <= ray_z, "the sensor's lowest ray (%.1f deg down, assumed) passes %.1f mm above the "
         "bumper's front edge (1 mm or more)" % (D.TOF_LOW_RAY_DEG, ray_z - D.BUMPER_Z1_MM))
    rule(D.BUMPER_Z0_MM >= D.STRIPE_T_MM + 3, "bumper bottom %.1f mm clears the floor stripe" % D.BUMPER_Z0_MM)
    rule(D.WALL_H_MM >= D.BUMPER_Z1_MM + 2, "dock walls (%.0f mm) stand above the bumper top" % D.WALL_H_MM)
    # the robot must reach the back wall at any angle: diagonal through the pocket depth
    lim = math.sqrt(D.POCKET_W_MM ** 2 - D.ROBOT_WIDTH_MM ** 2)
    rule(D.POCKET_DEPTH_MM <= 0.9 * lim, "pocket depth %.1f mm <= 90%% of %.1f mm (a crooked robot still reaches "
         "the back wall)" % (D.POCKET_DEPTH_MM, lim))
    lev = D.LEVER_PRESSED_MM < D.FACE_ABOVE_BODY_MM < D.LEVER_OPERATE_MM
    rule(lev, "bumper face sits between the lever's operate (%.1f) and end-of-travel (%.1f) positions: clicked, "
         "never crushed" % (D.LEVER_OPERATE_MM, D.LEVER_PRESSED_MM))
    rule(D.SWITCH_Y_OUTER_MM - D.SWITCH_L_MM > 3.0, "the two switch pockets leave a centre section")
    for name in ("nose_bumper_m3", "nose_bumper_technic"):
        x0, y0, z0, x1, y1, z1 = bbox(parts[name])
        rule(y1 - y0 <= D.ROBOT_WIDTH_MM + 0.01, "%s is no wider than the robot (%.1f mm): it fits the pocket"
             % (name, y1 - y0))
    straps = [abs(y) - D.STRAP_W_MM / 2 for _, y in D.TECHNIC_HOLES_MM]
    rule(min(straps) >= D.TOF_WINDOW_HALF_WIDTH_MM + 3, "Technic straps pass %.1f mm or more beside the sensor "
         "window" % (min(straps) - D.TOF_WINDOW_HALF_WIDTH_MM))
    for _, y in D.TECHNIC_HOLES_MM:
        rule(abs(y) + D.STRAP_W_MM / 2 <= D.ROBOT_WIDTH_MM / 2, "strap at y %.0f stays within the robot's width" % y)
    ang = math.degrees(math.atan2(D.FUNNEL_FLARE_MM, D.FUNNEL_LEN_MM))
    rule(ang <= 40, "funnel walls %.0f deg off the axis (vertical walls: no overhang)" % ang)
    return ok, lines


# ============================================================================= previews
def preview(name, shape, path, plan_z=None):
    """Two views: a shaded 3-D view, and a plan (top) view cut at height plan_z (mm) with every hole and pocket."""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    verts, tris = shape.val().tessellate(0.2, 0.3)
    V = np.array([(v.x, v.y, v.z) for v in verts])
    T = np.array(tris)
    fig = plt.figure(figsize=(10, 4.6), dpi=110)
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    tri = V[T]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    light = np.array([0.3, -0.5, 0.8]) / np.linalg.norm([0.3, -0.5, 0.8])
    shade = 0.35 + 0.6 * np.clip(n @ light, 0, 1)
    col = np.c_[0.30 * shade, 0.45 * shade, 0.62 * shade, np.ones_like(shade)]
    ax.add_collection3d(Poly3DCollection(tri, facecolors=col, edgecolors="none"))
    mn, mx = V.min(0), V.max(0)
    c, r = (mn + mx) / 2, (mx - mn).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_xlim(mn[0], mx[0])
    ax.set_ylim(mn[1], mx[1])
    ax.set_zlim(mn[2], mx[2])
    ax.set_box_aspect(tuple(max(v, 1.0) for v in (mx - mn)))
    ax.view_init(elev=35, azim=-60)
    ax.set_xlabel("x mm")
    ax.set_ylabel("y mm")
    sz = mx - mn
    ax.set_title("%s  %.0f x %.0f x %.0f mm" % (name, sz[0], sz[1], sz[2]), fontsize=10)
    ax2 = fig.add_subplot(1, 2, 2)
    z = plan_z if plan_z is not None else (mn[2] + mx[2]) / 2
    try:
        flat = cq().Workplane("XY").add(shape.val())     # section() cuts relative to the workplane: use z = 0
        for e in flat.section(z).edges().vals():
            pts = np.array([[p.x, p.y] for p in (e.positionAt(t) for t in np.linspace(0, 1, 24))])
            ax2.plot(pts[:, 0], pts[:, 1], color="#24384f", lw=1.0)
    except Exception:
        pass
    ax2.set_aspect("equal")
    ax2.grid(True, lw=0.3)
    ax2.set_xlabel("x mm")
    ax2.set_ylabel("y mm")
    ax2.set_title("outline cut at %.1f mm height, seen from above" % z, fontsize=10)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main():
    c = cq()
    os.makedirs(STL, exist_ok=True)
    os.makedirs(PNG, exist_ok=True)
    dock, _ = dock_solid()
    parts = {
        "dock": dock,
        "nose_bumper_m3": bumper_solid("m3"),
        "nose_bumper_technic": bumper_solid("technic"),
        "check_tile": tile_solid(),
        "led_holder": led_holder_solid(),
    }
    ok, lines = checks(parts)
    cuts = {"dock": 0.3, "nose_bumper_m3": D.BUMPER_Z0_MM + D.SHELF_T_MM + 2, "nose_bumper_technic":
            D.BUMPER_Z0_MM + D.SHELF_T_MM + 2, "check_tile": D.TILE_T_MM + 0.3, "led_holder": 1.0}
    for name, shape in parts.items():
        bed, how, over = best_orientation(shape)
        # small round holes lying sideways bridge fine; flag only real overhangs
        lines.append(("ok   " if over <= 40 else "WARN ") + "%s: print %s; %.0f mm^2 overhanging more than %.0f deg%s"
                     % (name, how, over, D.MAX_OVERHANG_DEG, "" if over <= 40 else " (needs support there)"))
        c.exporters.export(bed, os.path.join(STL, name + ".stl"), tolerance=0.05, angularTolerance=0.1)
        preview(name, shape, os.path.join(PNG, name + ".png"), cuts.get(name))
    print("\n".join(lines))
    print("\nwrote %s/*.stl and %s/*.png" % (STL, PNG))
    print("\nconsole lines for the firmware (after measuring and rebuilding):")
    for l in D.firmware_lines():
        print("  " + l)
    if not ok:
        print("\nSOME CHECKS FAILED: fix dock_params.py before printing")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
