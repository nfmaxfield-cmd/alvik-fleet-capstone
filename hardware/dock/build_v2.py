"""Dock version 2: a funnel inside for parking, and a pair of measuring posts on each of its three closed sides.

    python hardware/dock/build_v2.py      (writes stl/dock_v2.stl, preview/dock_v2.png, preview/dock_v2_layout.png)

Version 1 (build.py) funnels the robot straight before its switches touch anything, so it parks precisely but
erases the robot's own error on the way in. Version 2 keeps that funnel for parking and adds three measuring
stations on the outside:

  - each station is two narrow posts whose tips sit exactly one switch spacing apart (2 x SWITCH_LATERAL_MM),
    on a line square to the robot's approach, at switch-lever height, standing proud of everything else on that
    side by at least POST_CLEAR_MM, so the levers are the first thing to touch;
  - driving straight at a station, each switch clicks on its post: the two click distances give the distance to
    the station and the robot's angle to it; a lever that misses its post (the post meets the bracket's face
    instead) says the robot is off to that side by more than about WINDOW_MM: a sideways go/no-go check;
  - two stations facing 90 degrees apart (the back and one side) give a full position: x, y and heading.

The head (the solid block behind the pocket) is longer than in version 1 so a whole station fits along each
side; the screw tabs beside it are replaced by countersunk holes straight through the head (for #6 x 1 1/4 in
wood screws), so nothing on the measured sides stands in the robot's way. The funnel's two tabs move part-way
along the funnel walls (3 mm tall: the bracket's bar, 5 mm off the floor, passes over them). The floor stripe of version 1 is gone:
robots stay on the plywood.

Every robot number comes from dock_params.py (shared with version 1); the version 2 numbers are below.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dock_params as D  # noqa: E402
import build as B        # noqa: E402

# =============================================================================== version 2 numbers (mm)
HEAD_DEPTH_MM = 100.0          # head: from the pocket's back wall (dock X = 0) back to the back face
POST_STANDOFF_MM = 10.0        # post tip this far out from the face it stands on
POST_TIP_W_MM = 3.0            # tip width (across the approach): narrow, so the sideways check is tight
POST_BASE_W_MM = 9.0           # base width where it joins the dock: tapered for stiffness
POST_H_MM = 15.0               # post height from the floor (the levers sit about 7-13 mm up)
POST_CLEAR_MM = 3.0            # nothing else on a measured side comes closer than this to the tip line
SCREW_HOLES = [(12.0, 40.0), (12.0, -40.0), (88.0, 40.0), (88.0, -40.0)]   # (X, Y) through the head
SCREW_HOLE_DIA_MM = 3.8        # #6 wood screw clearance
SCREW_CSK_DIA_MM = 8.0         # countersink at the top
TAB_X_MM = -34.0               # the two funnel tabs' centre, dock X (part-way along the funnel)

# derived
POST_SPACING_MM = 2 * D.SWITCH_LATERAL_MM          # lever middle to lever middle
SIDE_FACE_Y_MM = D.POCKET_W_MM / 2 + D.WALL_T_MM    # the head's side faces
BACK_FACE_X_MM = HEAD_DEPTH_MM
SIDE_POST_XC_MM = HEAD_DEPTH_MM / 2                 # side stations centred on the head
TILE_CENTER_X_MM = HEAD_DEPTH_MM / 2
# sideways go/no-go window: the post tip must land on the lever (about the switch body's length), not on the
# bracket's face beside it
WINDOW_MM = D.SWITCH_L_MM / 2 - POST_TIP_W_MM / 2


def post(base_xy, out_dir, along_dir):
    """A tapered post standing on the floor: base centre on the dock face, pointing out_dir (unit), widths
    measured along along_dir (unit)."""
    c = B.cq()
    (bx, by), (ox, oy), (ax_, ay) = base_xy, out_dir, along_dir
    hb, ht, L = POST_BASE_W_MM / 2, POST_TIP_W_MM / 2, POST_STANDOFF_MM
    back = 2.0      # sink the base 2 mm into the dock so it fuses
    pts = [(bx - ox * back + ax_ * hb, by - oy * back + ay * hb),
           (bx + ox * L + ax_ * ht, by + oy * L + ay * ht),
           (bx + ox * L - ax_ * ht, by + oy * L - ay * ht),
           (bx - ox * back - ax_ * hb, by - oy * back - ay * hb)]
    return c.Workplane("XY").polyline(pts).close().extrude(POST_H_MM)


def stations():
    """[(name, tip points [(x, y)], approach direction the robot drives (unit), tip-line coordinate)]."""
    s = D.SWITCH_LATERAL_MM
    xc = SIDE_POST_XC_MM
    yt = SIDE_FACE_Y_MM + POST_STANDOFF_MM
    xt = BACK_FACE_X_MM + POST_STANDOFF_MM
    return [("back", [(xt, s), (xt, -s)], (-1.0, 0.0), xt),
            ("left", [(xc + s, yt), (xc - s, yt)], (0.0, -1.0), yt),
            ("right", [(xc + s, -yt), (xc - s, -yt)], (0.0, 1.0), -yt)]


def dock_v2_solid():
    c = B.cq()
    hw, t, H = D.POCKET_W_MM / 2, D.WALL_T_MM, D.WALL_H_MM
    xp, xm, fl = -D.POCKET_DEPTH_MM, D.MOUTH_X_MM, D.FUNNEL_FLARE_MM
    # head
    body = c.Workplane("XY").box(HEAD_DEPTH_MM, 2 * (hw + t), H, centered=(False, True, False))
    # pocket side walls and funnel walls (as version 1)
    for sg in (1, -1):
        body = body.union(c.Workplane("XY").center(xp / 2, sg * (hw + t / 2)).box(-xp, t, H, centered=(True, True, False)))
        pts = [(xm, sg * (hw + fl)), (xp, sg * hw), (xp, sg * (hw + t)), (xm, sg * (hw + fl + t))]
        body = body.union(c.Workplane("XY").polyline(pts).close().extrude(H))
    # camera check tile recess, finger notch
    r = D.TILE_MM + 2 * D.TILE_RECESS_CLEAR_MM
    body = body.cut(c.Workplane("XY").workplane(offset=H - D.TILE_T_MM + 0.2).center(TILE_CENTER_X_MM, 0)
                    .rect(r, r).extrude(D.TILE_T_MM))
    body = body.cut(c.Workplane("XY").workplane(offset=H - D.TILE_T_MM).center(TILE_CENTER_X_MM + r / 2, 0)
                    .circle(4).extrude(5))
    # screw holes straight through the head, countersunk at the top
    for (x, y) in SCREW_HOLES:
        body = body.cut(c.Workplane("XY").center(x, y).circle(SCREW_HOLE_DIA_MM / 2).extrude(H + 1))
        body = body.cut(c.Workplane("XY").workplane(offset=H - SCREW_CSK_DIA_MM / 2).center(x, y)
                        .circle(0.01).workplane(offset=SCREW_CSK_DIA_MM / 2 + 0.01).circle(SCREW_CSK_DIA_MM / 2 + 0.01)
                        .loft())
    # two screw tabs on the funnel walls, part-way along (the dock stays inside a 200 mm bed with a brim);
    # 3 mm tall: the bracket's bar, 5 mm off the floor, passes over them
    for sg in (1, -1):
        tx = TAB_X_MM
        wall_out = hw + t + fl * (xp - tx) / D.FUNNEL_LEN_MM          # funnel wall's outer face at tx
        ty = sg * (wall_out + D.TAB_W_MM / 2 - 2)
        tab = c.Workplane("XY").center(tx, ty).box(D.TAB_L_MM, D.TAB_W_MM + 2, D.TAB_T_MM, centered=(True, True, False))
        tab = tab.faces(">Z").workplane().center(0, sg * 1).cskHole(D.SCREW_HOLE_DIA_MM, D.SCREW_HEAD_DIA_MM, 90)
        body = body.union(tab)
    # posts
    s = D.SWITCH_LATERAL_MM
    for y in (s, -s):
        body = body.union(post((BACK_FACE_X_MM, y), (1, 0), (0, 1)))
    for sg in (1, -1):
        for x in (SIDE_POST_XC_MM + s, SIDE_POST_XC_MM - s):
            body = body.union(post((x, sg * SIDE_FACE_Y_MM), (0, sg), (1, 0)))
    return body


def checks(dock):
    c = B.cq()
    lines, ok = [], True

    def rule(good, text):
        nonlocal ok
        ok = ok and good
        lines.append(("ok   " if good else "FAIL ") + text)

    sol = dock.val()
    inside = lambda x, y, z: sol.isInside(c.Vector(x, y, z), 0.01)
    z_lever = D.BUMPER_Z0_MM + D.SHELF_T_MM + D.SWITCH_T_MM / 2          # the levers' height
    rule(POST_H_MM >= D.BUMPER_Z1_MM - 0.01 and z_lever < POST_H_MM,
         "posts stand %.0f mm tall; the levers are about %.1f mm up, the bar %.0f-%.0f mm" % (
             POST_H_MM, z_lever, D.BUMPER_Z0_MM, D.BUMPER_Z1_MM))
    for name, tips, (dx, dy), line in stations():
        # tip spacing and squareness
        (x1, y1), (x2, y2) = tips
        sp = math.hypot(x2 - x1, y2 - y1)
        rule(abs(sp - POST_SPACING_MM) < 0.01, "%s station: post tips %.2f mm apart (switch spacing %.2f)" % (
            name, sp, POST_SPACING_MM))
        # both tips are solid, and are the first thing the bracket's face meets: sample the band the bar sweeps
        # (bar half width + 5 mm each way) at every height the robot's front occupies (4.2 to 30 mm), from the
        # tip line POST_CLEAR_MM inwards; only the posts themselves may be there
        for (tx, ty) in tips:
            rule(inside(tx + dx * 0.4, ty + dy * 0.4, z_lever), "%s station: post tip at (%.1f, %.1f) is solid" % (
                name, tx, ty))
        mid = ((x1 + x2) / 2, (y1 + y2) / 2)
        ax_, ay = (x2 - x1) / sp, (y2 - y1) / sp
        half = D.BRACKET_W_MM / 2 + 5.0
        hits = []
        u = -half
        while u <= half:
            near_post = min(abs(u - (-sp / 2)), abs(u - sp / 2)) <= POST_BASE_W_MM / 2 + 0.5
            if not near_post:
                for depth in (0.2, POST_CLEAR_MM / 2, POST_CLEAR_MM):
                    for z in (D.FRONT_FACE_BOTTOM_Z_MM + 0.1, z_lever, D.BUMPER_Z1_MM, 25.0):
                        px = mid[0] + ax_ * u - dx * (-depth) * -1
                        py = mid[1] + ay * u - dy * (-depth) * -1
                        # depth measured from the tip line towards the dock (against the approach direction... the
                        # robot drives along (dx, dy), so the dock lies further along it)
                        px, py = mid[0] + ax_ * u + dx * depth, mid[1] + ay * u + dy * depth
                        if inside(px, py, z):
                            hits.append((round(u, 1), depth, z))
            u += 1.0
        rule(not hits, "%s station: nothing but the posts within %.0f mm of the tip line across the bar's width%s" % (
            name, POST_CLEAR_MM, "" if not hits else " (hits at %s)" % hits[:3]))
    rule(WINDOW_MM > 0, "sideways go/no-go: a lever meets its post if the robot is within about +-%.1f mm of "
                        "centre (switch length %.1f, post tip %.1f)" % (WINDOW_MM, D.SWITCH_L_MM, POST_TIP_W_MM))
    b = sol.BoundingBox()
    fits = b.xlen + 16 <= D.BED_MM[0] and b.ylen + 16 <= D.BED_MM[1]
    rule(fits, "dock_v2: %.0f x %.0f x %.0f mm, fits a %d x %d bed with a brim" % (b.xlen, b.ylen, b.zlen, *D.BED_MM))
    rule(sol.isValid(), "dock_v2: valid solid")
    return ok, lines


def layout_preview(dock, path):
    """For the report: the dock from above, a robot parked in the funnel and one robot at each measuring station."""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    c = B.cq()
    fig, ax = plt.subplots(figsize=(9, 8.2), dpi=130)
    big = 800
    cut = dock.val().intersect(c.Solid.makeBox(big, big, 0.02, c.Vector(-big / 2, -big / 2, 10.0)))
    verts, tris = cut.tessellate(0.1, 0.2)
    V = np.array([(v.x, v.y) for v in verts])
    for t in tris:
        ax.add_patch(Polygon(V[list(t)], closed=True, fc="#2b2b2b", ec="#2b2b2b", lw=0.3))
    # robot outline (bracket face at robot X = FACE_X, pointing +x in robot frame)
    rob = []
    for xr, hw, what in B.docked_outline(0.5):
        rob.append((xr, hw))
    outline = [(x, y) for x, y in rob] + [(x, -y) for x, y in rob[::-1]]
    outline = np.array(outline) - np.array([D.FACE_X_MM, 0])      # bracket face at the origin, robot behind (-x)

    def place(theta_deg, ox, oy, col, label, lx, ly):
        th = math.radians(theta_deg)
        R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        P = outline @ R.T + np.array([ox, oy])
        ax.add_patch(Polygon(P, closed=True, fc=col, alpha=0.30, ec=col, lw=1.2))
        ax.annotate("", xy=(ox, oy), xytext=(ox - 60 * math.cos(th), oy - 60 * math.sin(th)),
                    arrowprops=dict(arrowstyle="->", color=col, lw=1.2))
        ax.text(lx, ly, label, ha="center", va="center", fontsize=9.5, color="#111")

    gap = D.SW_GAP_MM
    # parked: bracket face against the pocket's back wall (dock X = 0), robot pointing +x
    place(0, -gap, 0, "#2f5d8a", "parked\n(funnel)", -100, 0)
    # back station: robot faces -x, its face at the tip line
    xt = BACK_FACE_X_MM + POST_STANDOFF_MM
    place(180, xt + gap, 0, "#c0602a", "measuring:\nback posts", xt + 100, 0)
    yt = SIDE_FACE_Y_MM + POST_STANDOFF_MM
    place(-90, SIDE_POST_XC_MM, yt + gap, "#2a8a5a", "measuring:\nleft posts", SIDE_POST_XC_MM, yt + 100)
    place(90, SIDE_POST_XC_MM, -yt - gap, "#7a4ab0", "measuring:\nright posts", SIDE_POST_XC_MM, -yt - 100)
    ax.set_aspect("equal")
    ax.set_xlim(-210, 260)
    ax.set_ylim(-230, 230)
    ax.axis("off")
    ax.set_title("Dock version 2: park in the funnel, measure at three post stations", fontsize=13)
    fig.text(0.5, 0.035, "Seen from above at switch height. Each coloured robot shows where it stops: parked in the "
             "funnel, or with both nose switches touching a pair of posts. The two clicks give its distance and "
             "angle; a lever that misses its post shows a sideways error. Any two stations facing 90 degrees apart "
             "give a full position. The posts are molded into the dock, so their spacing and position are exact and "
             "can't shift.", ha="center", fontsize=9, wrap=True)
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def main():
    c = B.cq()
    dock = dock_v2_solid()
    ok, lines = checks(dock)
    bed, how, over = B.best_orientation(dock)
    lines.append(("ok   " if over <= 40 else "WARN ") + "dock_v2: print %s; %.0f mm^2 overhanging more than %.0f deg"
                 % (how, over, D.MAX_OVERHANG_DEG))
    os.makedirs(B.STL, exist_ok=True)
    os.makedirs(B.PNG, exist_ok=True)
    c.exporters.export(bed, os.path.join(B.STL, "dock_v2.stl"), tolerance=0.05, angularTolerance=0.1)
    B.preview("dock_v2", dock, os.path.join(B.PNG, "dock_v2.png"), 10.0)
    layout_preview(dock, os.path.join(B.PNG, "dock_v2_layout.png"))
    print("\n".join(lines))
    print("\nstation tips (dock mm; X into the dock from the pocket's back wall, Y left):")
    for name, tips, d, line in stations():
        print("  %-5s %s, robot drives %s" % (name, ", ".join("(%.2f, %.2f)" % p for p in tips), d))
    print("\nwrote %s/dock_v2.stl and %s/dock_v2*.png" % (B.STL, B.PNG))
    if not ok:
        print("\nSOME CHECKS FAILED")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
