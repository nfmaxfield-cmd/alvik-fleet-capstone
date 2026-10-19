"""Build the home dock, the nose bracket (bumper), the camera check tile and the LED holder from dock_params.py:
STL files to print, a PNG preview of each, and a printability report.

    pip install cadquery matplotlib numpy      (cadquery 2.4+; on Linux add --break-system-packages if pip asks)
    python hardware/dock/build.py               (writes hardware/dock/stl/ and hardware/dock/preview/)
    python hardware/dock/build.py --ref alvik_reference_design.step   (also checks against Arduino's CAD model)

Every number comes from dock_params.py; change it there and run this again. It prints the console lines that
put the bracket's numbers into the firmware, and FAILS (exit 1) if a part does not fit the bed, a wall is
thinner than MIN_WALL_MM, the bracket would cover the distance sensor, hit a tyre or need a screw longer than the
insert takes, the pocket is too deep for a crooked bracket to reach the back wall, or anything behind the pocket
(arms, screw heads, robot) comes closer to a dock wall than the pocket's own play.

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
                  .box(s - D.TILE_SQUARE_GAP_MM, s - D.TILE_SQUARE_GAP_MM, D.TILE_RAISE_MM,
                       centered=(True, True, False)))   # a hair smaller: squares that touch only at a
                                                          # corner make a non-manifold STL (slicer errors)
            base = base.union(sq)
    return base


# ============================================================================= the nose bracket
def robot_envelope(length=200.0):
    """The robot's shell as far as the bracket cares (robot frame): the side walls leaning out 1.1 deg, the front
    face at X = 0, from the floor to the top. Cut from the bracket blank, it leaves the U's inside."""
    c = cq()
    z0, z1 = 0.0, D.ROBOT_TOP_Z_MM + 10
    a, b = D.side_half_w(z0) + D.ARM_CLEAR_MM, D.side_half_w(z1) + D.ARM_CLEAR_MM
    sec = c.Workplane("YZ").polyline([(-a, z0), (a, z0), (b, z1), (-b, z1)]).close().extrude(length)
    return sec.translate((-length, 0, 0))


def bracket_solid():
    """Nose bracket in robot coordinates: X forward from the robot's front face, Y left, Z up from the floor.
    A bar across the front carrying the two switches, an arm back along each side to the front pair of M3
    inserts."""
    c = cq()
    yo = D.ARM_OUTER_Y_MM
    z0, z1 = D.BUMPER_Z0_MM, D.BUMPER_Z1_MM
    Hh = z1 - z0
    depth = D.FACE_X_MM
    xb = D.ARM_BACK_X_MM
    xi = D.SIDE_INSERT_X_MM
    zt = D.ARM_TOP_Z_MM
    # the bar, full bracket width
    body = (c.Workplane("XY").workplane(offset=z0).center(depth / 2, 0)
            .box(depth, 2 * yo, Hh, centered=(True, True, False)))
    body = body.edges("|Z and >X").chamfer(D.CORNER_CHAMFER_MM)
    # the arms: a side profile (X, Z), low at the front, rising to the screws, extruded across Y; the robot's
    # envelope is cut out of it below
    rise_x = xi + D.ARM_BOSS_R_MM + 4.0          # the top edge is full height from here back
    prof = [(depth - D.CORNER_CHAMFER_MM, z0), (xb, z0), (xb, zt), (rise_x, zt), (0.0, z1), (depth - D.CORNER_CHAMFER_MM, z1)]
    inner = D.side_half_w(z0) - 1.0
    for sg in (1, -1):
        arm = c.Workplane("XZ").polyline(prof).close().extrude(-(yo - inner))   # XZ normal is -Y: extrude -d = +Y
        arm = arm.translate((0, inner, 0)) if sg > 0 else arm.mirror("XZ").translate((0, -inner, 0))
        body = body.union(arm)
    body = body.cut(robot_envelope())
    # switch pockets: open to the front and the top, a shelf below (the terminals behind the body are open to
    # the top too: the wires go straight up from there)
    for sg in (1, -1):
        so = sg * D.SWITCH_Y_OUTER_MM
        si = sg * (D.SWITCH_Y_OUTER_MM - D.SWITCH_L_MM - 0.4)
        y0, y1 = min(so, si), max(so, si)
        pocket = (c.Workplane("XY").workplane(offset=z0 + D.SHELF_T_MM)
                  .center(D.BACKPLATE_T_MM + (depth - D.BACKPLATE_T_MM) / 2 + 0.5, (y0 + y1) / 2)
                  .box(depth - D.BACKPLATE_T_MM + 1.0, (y1 - y0) + 0.4, Hh, centered=(True, True, False)))
        body = body.cut(pocket)
        # M2 self-tapping pilot holes (1.8) through the shelf, under the switch's own holes (ASSUMED layout)
        ym = sg * D.SWITCH_LATERAL_MM
        for dy in (-D.SWITCH_HOLE_SPACING_MM / 2, D.SWITCH_HOLE_SPACING_MM / 2):
            hole = (c.Workplane("XY").workplane(offset=z0 - 1).center(D.SWITCH_BACK_X_MM + D.SWITCH_HOLE_X_MM, ym + dy)
                    .circle(0.9).extrude(D.SHELF_T_MM + 2))
            body = body.cut(hole)
        # M3 clearance holes through the arm, at the inserts
        for zi in D.SIDE_INSERT_Z_MM:
            body = body.cut(cyl(D.M3_CLEAR_DIA_MM / 2, 12, (xi, sg * (yo + 1), zi), (0, -sg, 0)))
    return body


def screw_heads(sg):
    """The two M3 heads on one side (sg = +1 left, -1 right), as solids standing out of the arm (robot frame)."""
    s = None
    for zi in D.SIDE_INSERT_Z_MM:
        h = cyl(D.M3_HEAD_DIA_MM / 2, D.M3_HEAD_H_MM, (D.SIDE_INSERT_X_MM, sg * D.ARM_OUTER_Y_MM, zi), (0, sg, 0))
        s = h if s is None else s.union(h)
    return s


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


def docked_outline(step=0.5):
    """[(robot X, half width, what)] along the robot with its bracket, seen from above (screw heads included)."""
    out = []
    x = D.FACE_X_MM
    xi, r = D.SIDE_INSERT_X_MM, D.M3_HEAD_DIA_MM / 2
    while x > -(D.ROBOT_LENGTH_MM + 1):
        if x >= D.ARM_BACK_X_MM:
            hw, what = D.ARM_OUTER_Y_MM, "bracket" if x >= 0 else "bracket arm"
            if abs(x - xi) <= r:
                hw, what = D.ARM_OUTER_Y_MM + D.M3_HEAD_H_MM, "arm's screw heads"
        else:
            hw, what = D.ROBOT_WIDTH_MM / 2, "robot's side"
        out.append((x, hw, what))
        x -= step
    return out


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
    arm_min = D.ARM_OUTER_Y_MM - D.side_half_w(D.ARM_TOP_Z_MM) - D.ARM_CLEAR_MM
    thin = [("dock walls", D.WALL_T_MM), ("bracket back plate", D.BACKPLATE_T_MM), ("switch shelf", D.SHELF_T_MM),
            ("bracket outer rib", D.FACE_RIB_MM), ("bracket arms (thinnest, at the top)", arm_min),
            ("screw tabs", D.TAB_T_MM), ("LED plate", D.LED_PLATE_T_MM)]
    for what, t in thin:
        rule(t >= D.MIN_WALL_MM, "%s %.1f mm thick (>= %.1f)" % (what, t, D.MIN_WALL_MM))
    rule(D.TILE_T_MM >= 1.0, "check tile %.1f mm thick (thin by design; it lies in its recess)" % D.TILE_T_MM)
    # distance sensor: the bar must stay below its window, and below its lowest ray out to the bracket's face
    rule(D.BUMPER_Z1_MM <= D.TOF_WINDOW_BOTTOM_Z_MM - 2.0,
         "bracket bar top %.1f mm is 2 mm or more below the distance sensor (%.1f mm)"
         % (D.BUMPER_Z1_MM, D.TOF_WINDOW_BOTTOM_Z_MM))
    ray_z = D.TOF_CENTER_Z_MM - (D.FACE_X_MM + D.TOF_RECESS_MM) * math.tan(math.radians(D.TOF_LOW_RAY_DEG))
    rule(D.BUMPER_Z1_MM + 1.0 <= ray_z, "the sensor's lowest ray (%.1f deg down, assumed) passes %.1f mm above the "
         "bar's front edge (1 mm or more)" % (D.TOF_LOW_RAY_DEG, ray_z - D.BUMPER_Z1_MM))
    rule(D.BUMPER_Z0_MM >= D.STRIPE_T_MM + 3, "bar bottom %.1f mm clears the floor stripe" % D.BUMPER_Z0_MM)
    bear = min(D.BAND_TOP_Z_MM, D.BUMPER_Z1_MM) - max(D.BUMPER_Z0_MM, D.FRONT_FACE_BOTTOM_Z_MM)
    rule(bear >= 5.0, "the bar's back bears on %.1f mm of the white band (5 or more): a push goes into the shell" % bear)
    rule(D.WALL_H_MM >= D.BUMPER_Z1_MM + 2, "dock walls (%.0f mm) stand above the bar top" % D.WALL_H_MM)
    # the arms and screws
    rule(D.ARM_TOP_Z_MM <= D.ROBOT_TOP_Z_MM - 0.5, "arms end %.1f mm below the robot's top" %
         (D.ROBOT_TOP_Z_MM - D.ARM_TOP_Z_MM))
    rule(D.ARM_BACK_X_MM - D.WHEEL_FRONT_X_MM >= 3.0, "arms end %.1f mm in front of the tyres (3 or more)"
         % (D.ARM_BACK_X_MM - D.WHEEL_FRONT_X_MM))
    for zi in D.SIDE_INSERT_Z_MM:
        t = D.ARM_OUTER_Y_MM - D.side_half_w(zi) - D.ARM_CLEAR_MM
        into = D.M3_SCREW_LEN_MM - t - D.ARM_CLEAR_MM
        rule(3.0 <= into <= D.SIDE_INSERT_DEPTH_MM - 0.3 and D.M3_SCREW_LEN_MM <= D.M3_SCREW_MAX_MM,
             "M3 x %.0f at %.1f mm: %.1f mm of arm, %.1f mm into the insert (3 to %.1f; Arduino's limit M3 x %.0f)"
             % (D.M3_SCREW_LEN_MM, zi, t, into, D.SIDE_INSERT_DEPTH_MM - 0.3, D.M3_SCREW_MAX_MM))
    # the bracket must reach the back wall at any angle: diagonal through the pocket depth
    lim = math.sqrt(D.POCKET_W_MM ** 2 - D.BRACKET_W_MM ** 2)
    rule(D.POCKET_DEPTH_MM <= 0.9 * lim, "pocket depth %.1f mm <= 90%% of %.1f mm (a crooked bracket still reaches "
         "the back wall)" % (D.POCKET_DEPTH_MM, lim))
    lev = D.LEVER_PRESSED_MM < D.FACE_ABOVE_BODY_MM < D.LEVER_OPERATE_MM
    rule(lev, "bracket face sits between the lever's operate (%.1f) and end-of-travel (%.1f) positions: clicked, "
         "never crushed" % (D.LEVER_OPERATE_MM, D.LEVER_PRESSED_MM))
    rule(D.SWITCH_Y_OUTER_MM - D.SWITCH_L_MM > 3.0, "the two switch pockets leave a centre section")
    x0, y0, z0, x1, y1, z1 = bbox(parts["nose_bracket"])
    rule(y1 - y0 <= D.BRACKET_W_MM + 0.01, "nose bracket is %.1f mm wide (BRACKET_W_MM %.1f): the pocket (%.1f) fits it"
         % (y1 - y0, D.BRACKET_W_MM, D.POCKET_W_MM))
    # docked straight: everything behind the pocket (arms, screw heads, the robot) clears the funnel walls by at
    # least the pocket's own play
    worst = None
    for xr, hw, what in docked_outline():
        xd = xr - D.FACE_X_MM
        if xd < D.MOUTH_X_MM:
            continue
        room = D.funnel_half_w(xd) - hw
        if worst is None or room < worst[0]:
            worst = (room, xd, what)
    rule(worst[0] >= D.POCKET_CLEAR_MM / 2 - 1e-6, "docked, the closest thing to a dock wall is the %s at dock X "
         "%.1f: %.2f mm clear (>= %.2f)" % (worst[2], worst[1], worst[0], D.POCKET_CLEAR_MM / 2))
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


def fit_preview(bracket, path):
    """How the bracket sits on the robot: side, top and front views with the robot's outline (from
    dock_params.py) drawn in grey."""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Rectangle
    grey, blue, red = "#9aa3ab", "#2f5d8a", "#c0392b"
    fig, axs = plt.subplots(1, 3, figsize=(15, 5.4), dpi=110, gridspec_kw={"width_ratios": [1.25, 1.25, 1.0]})
    L, top, fx = D.ROBOT_LENGTH_MM, D.ROBOT_TOP_Z_MM, D.FACE_X_MM
    xi, zs = D.SIDE_INSERT_X_MM, D.SIDE_INSERT_Z_MM

    def section(plane, at):
        # outline of the bracket cut by a plane, as point lists
        c = cq()
        big = 400
        slab = {"XY": c.Solid.makeBox(big, big, 0.02, c.Vector(-big / 2, -big / 2, at)),
                "XZ": c.Solid.makeBox(big, 0.02, big, c.Vector(-big / 2, at, -big / 2)),
                "YZ": c.Solid.makeBox(0.02, big, big, c.Vector(at, -big / 2, -big / 2))}[plane]
        cut = bracket.val().intersect(slab)
        verts, tris = cut.tessellate(0.1, 0.2)
        return np.array([(v.x, v.y, v.z) for v in verts]), tris

    def fill(ax, plane, at, i, j, color=blue):
        V, T = section(plane, at)
        for t in T:
            ax.add_patch(Polygon(V[list(t)][:, [i, j]], closed=True, fc=color, ec=color, lw=0.3))

    # side view: robot outline, recess band, insert holes, wheel; the bracket cut through the arm
    ax = axs[0]
    ax.add_patch(Rectangle((-L, D.FRONT_FACE_BOTTOM_Z_MM), L, top - D.FRONT_FACE_BOTTOM_Z_MM, fc="#eef1f3", ec=grey))
    ax.add_patch(Circle((-D.AXLE_TO_FRONT_MM, D.WHEEL_R_MM), D.WHEEL_R_MM, fc="#555", ec="#333", alpha=0.5))
    fill(ax, "XZ", D.ARM_OUTER_Y_MM - 1.0, 0, 2)
    for z in zs:
        ax.add_patch(Circle((xi, z), 1.5, fc="#b8860b", ec="k", lw=0.5))
        ax.add_patch(Circle((xi - 56.0, z), 1.5, fc="#b8860b", ec="k", lw=0.5))
    ax.add_patch(Circle((xi, 22.7), 2.4, fc="w", ec="k", lw=0.5))
    ax.axhline(0, color="k", lw=0.8)
    ax.set_title("from the left side: arm on the front inserts (gold)", fontsize=10)
    ax.set_xlim(-L - 3, fx + 6)
    ax.set_ylim(-2, top + 6)
    ax.set_xlabel("mm forward of the front face")
    ax.set_ylabel("mm above the floor")
    # top view: the bracket cut at the bar, around the robot's outline
    ax = axs[1]
    hw = D.ROBOT_WIDTH_MM / 2
    ax.add_patch(Rectangle((-L, -hw), L, 2 * hw, fc="#eef1f3", ec=grey))
    for sg in (1, -1):
        ax.add_patch(Rectangle((-D.AXLE_TO_FRONT_MM - D.WHEEL_R_MM, sg * hw - (8 if sg > 0 else 0)), 2 * D.WHEEL_R_MM,
                               8, fc="#555", alpha=0.5))
    fill(ax, "XY", D.BUMPER_Z0_MM + D.SHELF_T_MM + 2.5, 0, 1)
    fill(ax, "XY", zs[0], 0, 1, color="#5b86b0")
    for sg in (1, -1):
        y = sg * (D.SWITCH_Y_OUTER_MM - D.SWITCH_L_MM / 2)
        ax.add_patch(Rectangle((D.SWITCH_BACK_X_MM, y - D.SWITCH_L_MM / 2), D.SWITCH_H_MM, D.SWITCH_L_MM,
                               fc=red, alpha=0.6))
    ax.set_title("from above: switches (red) at the front corners", fontsize=10)
    ax.set_xlim(-L - 3, fx + 6)
    ax.set_ylim(-D.ARM_OUTER_Y_MM - 6, D.ARM_OUTER_Y_MM + 6)
    ax.set_xlabel("mm forward of the front face")
    ax.set_ylabel("mm left of the centre line")
    # front view: the bar under the recess and the distance sensor
    ax = axs[2]
    ax.add_patch(Rectangle((-hw, D.FRONT_FACE_BOTTOM_Z_MM), 2 * hw, top - D.FRONT_FACE_BOTTOM_Z_MM, fc="#eef1f3",
                           ec=grey))
    rec_hw = hw - 12.3
    ax.add_patch(Rectangle((-rec_hw, D.BAND_TOP_Z_MM), 2 * rec_hw, top - D.BAND_TOP_Z_MM, fc="#dde3e7", ec=grey))
    ax.add_patch(Rectangle((-D.TOF_WINDOW_HALF_WIDTH_MM, D.TOF_WINDOW_BOTTOM_Z_MM), 2 * D.TOF_WINDOW_HALF_WIDTH_MM,
                           2 * (D.TOF_CENTER_Z_MM - D.TOF_WINDOW_BOTTOM_Z_MM), fc="k"))
    fill(ax, "YZ", 1.0, 1, 2)
    ax.axhline(0, color="k", lw=0.8)
    ax.set_title("from the front: bar on the white band", fontsize=10)
    ax.set_xlim(-D.ARM_OUTER_Y_MM - 4, D.ARM_OUTER_Y_MM + 4)
    ax.set_ylim(-2, top + 6)
    ax.set_xlabel("mm left of the centre line")
    for a in axs:
        a.set_aspect("equal")
        a.grid(True, lw=0.3)
    fig.text(0.5, 0.02, "The blue bracket wraps the robot's nose (grey): its bar sits on the plain band under the "
             "sensor recess, and an arm on each side is screwed into the two front brass inserts.",
             ha="center", fontsize=10)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def docked_preview(dock, path):
    """The dock from above with a robot and its bracket docked straight, cut at the bar's height."""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    fig, ax = plt.subplots(figsize=(7.5, 7), dpi=110)
    c = cq()
    big = 600
    cut = dock.val().intersect(c.Solid.makeBox(big, big, 0.02, c.Vector(-big / 2, -big / 2, 10.0)))
    verts, tris = cut.tessellate(0.1, 0.2)
    V = np.array([(v.x, v.y) for v in verts])
    for t in tris:
        ax.add_patch(Polygon(V[list(t)], closed=True, fc="#333", ec="#333", lw=0.3))
    up, lo = [], []
    for xr, hw, what in docked_outline(0.25):
        up.append((xr - D.FACE_X_MM, hw))
        lo.append((xr - D.FACE_X_MM, -hw))
    ax.add_patch(Polygon(up + lo[::-1], closed=True, fc="#2f5d8a", alpha=0.35, ec="#2f5d8a"))
    xa = D.DOCKED_AXLE_X_MM
    ax.plot([xa], [0], "k+", ms=12)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.set_xlim(D.MOUTH_X_MM - 45, D.BACK_BLOCK_DEPTH_MM + 5)
    ax.set_xlabel("mm into the dock from the back wall's inner face")
    ax.set_ylabel("mm left of the dock's axis")
    ax.set_title("docked: pocket %.1f mm for a %.1f mm bracket, %.0f mm deep" % (D.POCKET_W_MM, D.BRACKET_W_MM,
                                                                                  D.POCKET_DEPTH_MM), fontsize=10)
    fig.text(0.5, 0.02, "The dock's walls (black) cut at 10 mm height, with a docked robot and bracket (blue): only "
             "the bracket's bar is in the narrow pocket; the arms and screw heads sit where the funnel has opened.",
             ha="center", fontsize=9, wrap=True)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def ref_check(path, bracket):
    """Interference between the bracket and Arduino's reference model (alvik_reference_design.step, in
    AKX00066-step.zip): the model is moved into the robot frame (front face at X = 0, floor at Z = 0)."""
    c = cq()
    ref = c.importers.importStep(path)
    ref = ref.rotate((0, 0, 0), (0, 0, 1), -90).translate((-D.AXLE_TO_FRONT_MM, 0, 10.1))
    inter = bracket.val().intersect(ref.val())
    vol = inter.Volume() if inter is not None else 0.0
    b = ref.val().BoundingBox()
    return vol, (b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax)


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--ref", help="Arduino's alvik_reference_design.step (from AKX00066-step.zip): also check the "
                                  "bracket does not run into the reference model")
    a = ap.parse_args(argv)
    c = cq()
    os.makedirs(STL, exist_ok=True)
    os.makedirs(PNG, exist_ok=True)
    dock, _ = dock_solid()
    bracket = bracket_solid()
    parts = {
        "dock": dock,
        "nose_bracket": bracket,
        "check_tile": tile_solid(),
        "led_holder": led_holder_solid(),
    }
    ok, lines = checks(parts)
    if a.ref:
        vol, bb = ref_check(a.ref, bracket)
        good = vol <= 5.0
        ok = ok and good
        lines.append(("ok   " if good else "FAIL ") + "bracket overlaps Arduino's reference model by %.2f mm^3 (5 or "
                     "less: the drafted face rounding); model spans X %.1f..%.1f, Y %.1f..%.1f, Z %.1f..%.1f" % ((vol,) + bb))
    cuts = {"dock": 0.3, "nose_bracket": D.BUMPER_Z0_MM + D.SHELF_T_MM + 2, "check_tile": D.TILE_T_MM + 0.3,
            "led_holder": 1.0}
    for name, shape in parts.items():
        bed, how, over = best_orientation(shape)
        # small round holes lying sideways bridge fine; flag only real overhangs
        lines.append(("ok   " if over <= 40 else "WARN ") + "%s: print %s; %.0f mm^2 overhanging more than %.0f deg%s"
                     % (name, how, over, D.MAX_OVERHANG_DEG, "" if over <= 40 else " (needs support there)"))
        c.exporters.export(bed, os.path.join(STL, name + ".stl"), tolerance=0.05, angularTolerance=0.1)
        preview(name, shape, os.path.join(PNG, name + ".png"), cuts.get(name))
    fit_preview(bracket, os.path.join(PNG, "nose_bracket_fit.png"))
    docked_preview(dock, os.path.join(PNG, "docked.png"))
    print("\n".join(lines))
    print("\nwrote %s/*.stl and %s/*.png" % (STL, PNG))
    print("\nconsole lines for the firmware (each robot, once the bracket is on):")
    for l in D.firmware_lines():
        print("  " + l)
    if not ok:
        print("\nSOME CHECKS FAILED: fix dock_params.py before printing")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
