"""Light plate: the robot's camera lights on a plate above its top, screwed to the back pair of side inserts.

    python hardware/dock/build_light_plate.py        writes stl/light_plate.stl, preview/light_plate*.png

Goes with the nose bracket (switches, front insert pair). Both screw into the robot's own threaded inserts,
so the lights, the switches and the robot are fixed to each other; two parts because one part would need
support under the switch bar, and the bracket is already printed.

Lights: SK6812 RGBW single-pixel boards (round, ~10 mm, pads on the LED side), chained on one data line.
  - 5 at the front in a plus (directional: one large bright blob on camera = the front)
  - 2 at the back side by side (directional: a small blob = the back)
  - 6 in a row across the middle, 14 mm apart (ID: each on or off; codes that survive one missed light)
The pixel boards sit in shallow pockets on the top face; wire channels join them in chain order, starting at
the cable hole (the wires to the robot's board go down through it).

Geometry is in robot coordinates: X forward from the wheel axle (front face +47.6), Y left, Z up from the floor.
Positions borrowed from Andrew Muszynski's AprilTag plate (Alvik_Plate_Slim_v2.stl): plate ~22 mm above the
robot's top, legs on one insert pair (his holes are 15.9 mm apart; the inserts 16.0).
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dock_params as D  # noqa: E402
import build as B        # noqa: E402

# ------------------------------------------------------------------------------------------------ parameters
PIXEL_DIA_MM = 10.0            # ASSUMED from the listing (SMD5050 on a 10 mm round board): measure one and change
PIXEL_POCKET_CLEAR_MM = 0.5    # pocket is this much wider than the board
PIXEL_POCKET_DEPTH_MM = 1.2    # boards are ~1 mm thick: the LED and its pads stand just above the plate
PIXEL_PITCH_MM = 11.0          # cluster spacing: boards nearly touching (one blob on camera)
ID_PITCH_MM = 14.0             # ID lights: at least 13 mm apart (4.4 px on this camera) so they don't merge
PLATE_T_MM = 4.0
PLATE_UNDER_Z_MM = 59.0        # plate underside above the floor (Andrew's plate: ~59; robot top 37.2)
PLATE_X_MM = (-51.0, 58.0)     # back and front edge (robot X)
LEG_T_MM = 3.5
LEG_W_MM = 10.0
LEG_CLEAR_MM = 0.15
LEG_BOTTOM_Z_MM = D.SIDE_INSERT_Z_MM[0] - 5.0
BACK_INSERT_X_MM = -(D.AXLE_TO_FRONT_MM - 19.6)   # back insert pair: 19.6 mm in front of the back face (STEP)
GUSSET_MM = 12.0
CHANNEL_W_MM = 2.6             # three thin wires side by side
CHANNEL_D_MM = 1.6
CABLE_HOLE_MM = 7.0

FRONT = [(40.0, 0.0), (40.0 - PIXEL_PITCH_MM, 0.0), (40.0, -PIXEL_PITCH_MM), (40.0 + PIXEL_PITCH_MM, 0.0),
         (40.0, PIXEL_PITCH_MM)]
REAR = [(-40.0, -PIXEL_PITCH_MM / 2), (-40.0, PIXEL_PITCH_MM / 2)]
ID_X = 4.0
IDS = [(ID_X, y) for y in (35.0, 21.0, 7.0, -7.0, -21.0, -35.0)]
CABLE = (-28.0, 0.0)
# chain (data) order: cable -> rear pair -> ID row -> front cluster
CHAIN = REAR + IDS + [FRONT[1], FRONT[2], FRONT[3], FRONT[4], FRONT[0]]
ROLE = ["rear"] * 2 + ["id%d" % i for i in range(6)] + ["front"] * 5


def leg_y_inner():
    return D.side_half_w(D.ROBOT_TOP_Z_MM) + LEG_CLEAR_MM


def plate_solid():
    c = B.cq()
    yi = leg_y_inner()
    yo = yi + LEG_T_MM
    x0, x1 = PLATE_X_MM
    zt = PLATE_UNDER_Z_MM + PLATE_T_MM
    body = (c.Workplane("XY").workplane(offset=PLATE_UNDER_Z_MM).center((x0 + x1) / 2, 0)
            .box(x1 - x0, 2 * yo, PLATE_T_MM, centered=(True, True, False)))
    body = body.edges("|Z").fillet(4.0)
    xl = BACK_INSERT_X_MM
    for sg in (1, -1):
        leg = (c.Workplane("XY").workplane(offset=LEG_BOTTOM_Z_MM).center(xl, sg * (yi + LEG_T_MM / 2))
               .box(LEG_W_MM, LEG_T_MM, PLATE_UNDER_Z_MM - LEG_BOTTOM_Z_MM + 0.01, centered=(True, True, False)))
        body = body.union(leg)
        # gusset under the plate, inboard of the leg, above the robot's top (stiffens the leg sideways)
        g = (c.Workplane("YZ").workplane(offset=xl - LEG_W_MM / 2)
             .polyline([(sg * yi, PLATE_UNDER_Z_MM), (sg * yi, PLATE_UNDER_Z_MM - GUSSET_MM),
                        (sg * (yi - GUSSET_MM), PLATE_UNDER_Z_MM)]).close().extrude(LEG_W_MM))
        body = body.union(g)
        for zi in D.SIDE_INSERT_Z_MM:
            body = body.cut(B.cyl(D.M3_CLEAR_DIA_MM / 2, 12, (xl, sg * (yo + 1), zi), (0, -sg, 0)))
    # pockets
    r = (PIXEL_DIA_MM + PIXEL_POCKET_CLEAR_MM) / 2
    for (x, y) in FRONT + REAR + IDS:
        body = body.cut(c.Workplane("XY").workplane(offset=zt - PIXEL_POCKET_DEPTH_MM).center(x, y).circle(r)
                        .extrude(PIXEL_POCKET_DEPTH_MM + 1))
    # wire channels along the chain
    pts = [CABLE] + CHAIN
    for (xa, ya), (xb, yb) in zip(pts[:-1], pts[1:]):
        L = math.hypot(xb - xa, yb - ya)
        ang = math.degrees(math.atan2(yb - ya, xb - xa))
        ch = (c.Workplane("XY").workplane(offset=zt - CHANNEL_D_MM).center((xa + xb) / 2, (ya + yb) / 2)
              .rect(L, CHANNEL_W_MM).extrude(CHANNEL_D_MM + 1)
              .rotate(((xa + xb) / 2, (ya + yb) / 2, 0), ((xa + xb) / 2, (ya + yb) / 2, 1), ang))
        body = body.cut(ch)
    body = body.cut(c.Workplane("XY").workplane(offset=PLATE_UNDER_Z_MM - 1).center(*CABLE).circle(CABLE_HOLE_MM / 2)
                    .extrude(PLATE_T_MM + 2))
    return body


def checks(plate):
    ok = True

    def rule(good, text):
        nonlocal ok
        print(("ok   " if good else "FAIL ") + text)
        ok = ok and good
    allp = FRONT + REAR + IDS
    r = (PIXEL_DIA_MM + PIXEL_POCKET_CLEAR_MM) / 2
    yo = leg_y_inner() + LEG_T_MM
    inside = all(PLATE_X_MM[0] + 1 <= x - r and x + r <= PLATE_X_MM[1] - 1 and abs(y) + r <= yo - 1 for x, y in allp)
    rule(inside, "every pixel pocket is inside the plate with 1 mm to spare")
    dmin = min(math.hypot(a[0] - b[0], a[1] - b[1]) for i, a in enumerate(allp) for b in allp[i + 1:])
    rule(dmin >= 2 * r + 0.3, "pockets don't overlap (closest centres %.1f mm)" % dmin)
    gap_id = min(math.hypot(a[0] - b[0], a[1] - b[1]) for a in IDS for b in IDS if a != b)
    rule(gap_id >= 13.0, "ID lights at least 13 mm apart (%.1f)" % gap_id)
    gap_groups = min(math.hypot(a[0] - b[0], a[1] - b[1]) for a in IDS for b in FRONT + REAR)
    rule(gap_groups >= 13.0, "ID lights at least 13 mm from the directional lights (%.1f)" % gap_groups)
    rule(abs(-BACK_INSERT_X_MM) - LEG_W_MM / 2 > D.WHEEL_R_MM + 3, "legs clear the tyres (leg front edge %.1f mm behind "
         "the axle, tyre %.1f)" % (abs(BACK_INSERT_X_MM) - LEG_W_MM / 2, D.WHEEL_R_MM))
    rule(PLATE_UNDER_Z_MM - D.ROBOT_TOP_Z_MM - GUSSET_MM > 5, "gussets stay %.1f mm above the robot's top"
         % (PLATE_UNDER_Z_MM - D.ROBOT_TOP_Z_MM - GUSSET_MM))
    screw = LEG_T_MM + D.SIDE_INSERT_DEPTH_MM - 0.5
    rule(screw <= D.M3_SCREW_MAX_MM, "M3 x 8 screws: %.1f mm of leg + insert (Arduino's limit 10)" % (LEG_T_MM + 5.0))
    b = B.bbox(plate)
    sz = (b[3] - b[0], b[4] - b[1], b[5] - b[2])
    rule(max(sz[0], sz[1]) + 16 <= 200, "fits a 200 x 200 bed with a brim: %.0f x %.0f x %.0f mm" % sz)
    rule(plate.val().isValid(), "valid solid")
    led_z = PLATE_UNDER_Z_MM + PLATE_T_MM - PIXEL_POCKET_DEPTH_MM + 1.0 + 0.8
    print("     lights about %.0f mm above the floor (tools/lights height_cm %.1f)" % (led_z, led_z / 10))
    return ok, led_z


def layout_json(led_z):
    """Robot-frame light positions (cm, from the turning centre) for the camera software and the firmware."""
    out = dict(chain=[dict(index=i, role=ROLE[i], x_cm=round(p[0] / 10, 2), y_cm=round(p[1] / 10, 2))
                      for i, p in enumerate(CHAIN)], height_cm=round(led_z / 10, 2),
               note="x forward from the wheel axle (turning centre), y left; chain index = data order")
    p = os.path.join(HERE, "stl", "light_plate_layout.json")
    with open(p, "w") as f:
        json.dump(out, f, indent=1)
    return p


def plan_png(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, FancyBboxPatch, Rectangle
    fig, ax = plt.subplots(figsize=(7.2, 6.4), dpi=120)
    yo = leg_y_inner() + LEG_T_MM
    x0, x1 = PLATE_X_MM
    ax.add_patch(Rectangle((-D.AXLE_TO_FRONT_MM, -48), 2 * D.AXLE_TO_FRONT_MM, 96, fill=False, ec="#999", ls="--", lw=1))
    ax.text(-D.AXLE_TO_FRONT_MM + 1, -46, "robot outline", color="#888", fontsize=8)
    ax.add_patch(FancyBboxPatch((x0, -yo), x1 - x0, 2 * yo, boxstyle="round,pad=0,rounding_size=4", fc="#e9eef4",
                                ec="#24384f", lw=1.2))
    pts = [CABLE] + CHAIN
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#c9a227", lw=2, zorder=1)
    cols = {"front": "#2b7bbb", "rear": "#2b7bbb", "id": "#d1495b"}
    for i, (p, role) in enumerate(zip(CHAIN, ROLE)):
        col = cols["id"] if role.startswith("id") else cols[role]
        ax.add_patch(Circle(p, PIXEL_DIA_MM / 2, fc=col, ec="white", lw=1, zorder=2))
        ax.text(p[0], p[1], str(i), color="white", ha="center", va="center", fontsize=7, zorder=3)
    ax.add_patch(Circle(CABLE, CABLE_HOLE_MM / 2, fc="white", ec="#24384f", zorder=2))
    ax.text(CABLE[0] - 6, CABLE[1] + 6, "cable\nhole", fontsize=7, ha="right")
    for sg in (1, -1):
        ax.add_patch(Rectangle((BACK_INSERT_X_MM - LEG_W_MM / 2, sg * leg_y_inner() - (LEG_T_MM if sg < 0 else 0)),
                               LEG_W_MM, LEG_T_MM, fc="#24384f"))
    ax.annotate("", xy=(x1 + 8, 0), xytext=(x1 - 2, 0), arrowprops=dict(arrowstyle="->", lw=1.5))
    ax.text(x1 + 9, 0, "front", va="center", fontsize=9)
    ax.set_aspect("equal")
    ax.set_xlim(x0 - 8, x1 + 22)
    ax.set_ylim(-yo - 8, yo + 8)
    ax.set_xlabel("mm, forward from the wheel axle")
    ax.set_ylabel("mm, left")
    ax.set_title("Light plate from above: blue = directional (5 front, 2 back), red = ID;\n"
                 "numbers = data order along the chain (yellow = wire channels)", fontsize=9)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def overhang_above(shape, z_from, max_deg=None):
    import numpy as np
    max_deg = D.MAX_OVERHANG_DEG if max_deg is None else max_deg
    verts, tris = shape.val().tessellate(0.2, 0.3)
    V = np.array([(v.x, v.y, v.z) for v in verts])
    tri = V[np.array(tris)]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area = np.linalg.norm(n, axis=1) / 2
    nz = n[:, 2] / np.maximum(2 * area, 1e-12)
    high = tri[:, :, 2].min(axis=1) > V[:, 2].min() + z_from
    steep = nz < -math.cos(math.radians(90 - max_deg))
    return float(area[high & steep].sum())


def main():
    plate = plate_solid()
    ok, led_z = checks(plate)
    os.makedirs(B.STL, exist_ok=True)
    os.makedirs(B.PNG, exist_ok=True)
    # print with the plate's top face on the bed (pockets and channels are recesses there: no support)
    on_bed = plate.rotate((0, 0, 0), (1, 0, 0), 180)
    bb = on_bed.val().BoundingBox()
    on_bed = on_bed.translate((-bb.xmin, -bb.ymin, -bb.zmin))
    # the pockets and channels open onto the bed: their floors print as short bridges (11 mm at most), which any
    # printer does; count only overhangs above them
    over = overhang_above(on_bed, max(PIXEL_POCKET_DEPTH_MM, CHANNEL_D_MM) + 0.3)
    print(("ok   " if over < 60 else "FAIL ") + "printed upside down (top face on the bed), no support: %.0f mm^2 "
          "overhanging above the pocket floors (pocket floors bridge 11 mm at most)" % over)
    ok = ok and over < 60
    stl = os.path.join(B.STL, "light_plate.stl")
    on_bed.val().exportStl(stl, 0.05, 0.3)
    B.preview("light_plate", plate, os.path.join(B.PNG, "light_plate.png"), plan_z=PLATE_UNDER_Z_MM + 1)
    plan_png(os.path.join(B.PNG, "light_plate_layout.png"))
    print("wrote %s, %s" % (stl, layout_json(led_z)))
    if not ok:
        print("\nSOME CHECKS FAILED")
        sys.exit(1)


if __name__ == "__main__":
    main()
