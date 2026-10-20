"""The Alvik attachment, one printed part: the nose bracket (switches) and the light plate (camera lights).

    python hardware/dock/build_attachment.py      writes stl/attachment.stl, preview/attachment*.png

All eight of the robot's side inserts are used (M3 x 8, four per side): the bracket's arms on the front pair,
the plate's back legs on the back pair. Two front legs rise from the bracket's arms to the plate, so the plate
stands on four legs and the switches, the lights and the robot are fixed to each other by the print itself.

The bracket is build.py's (unchanged: the dock pocket and the firmware's bump_* numbers fit it); the plate,
its lights, pockets and wire channels are build_light_plate.py's.

Print: upside down (the plate's top face on the bed, so the light pockets and wire channels are clean), with
support under the switch bar and the front legs' tops, which hang over the plate's front edge. The report
gives the area that needs it.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dock_params as D        # noqa: E402
import build as B              # noqa: E402
import build_light_plate as L  # noqa: E402

FRONT_LEG_X_MM = D.AXLE_TO_FRONT_MM + D.SIDE_INSERT_X_MM      # robot X of the front insert pair (+28.0)
FRONT_LEG_W_MM = 10.0


def attachment_solid():
    c = B.cq()
    plate = L.plate_solid()
    bracket = B.bracket_solid().translate((D.AXLE_TO_FRONT_MM, 0, 0))     # build.py draws it from the front face
    body = plate.union(bracket)
    yo = D.ARM_OUTER_Y_MM
    yi = yo - D.ARM_T_MM
    for sg in (1, -1):
        # front leg: the arm's outer wall carried up from the arm's top edge to the plate
        leg = (c.Workplane("XY").workplane(offset=D.ARM_TOP_Z_MM - 4.0).center(FRONT_LEG_X_MM, sg * (yi + D.ARM_T_MM / 2))
               .box(FRONT_LEG_W_MM, D.ARM_T_MM, L.PLATE_UNDER_Z_MM - D.ARM_TOP_Z_MM + 4.01, centered=(True, True, False)))
        body = body.union(leg)
        g = (c.Workplane("YZ").workplane(offset=FRONT_LEG_X_MM - FRONT_LEG_W_MM / 2)
             .polyline([(sg * yi, L.PLATE_UNDER_Z_MM), (sg * yi, L.PLATE_UNDER_Z_MM - L.GUSSET_MM),
                        (sg * (yi - L.GUSSET_MM), L.PLATE_UNDER_Z_MM)]).close().extrude(FRONT_LEG_W_MM))
        body = body.union(g)
    # the robot's shell must stay clear of everything (the arms' inner faces follow it)
    body = body.cut(B.robot_envelope().translate((D.AXLE_TO_FRONT_MM, 0, 0)).intersect(
        c.Workplane("XY").box(400, 400, D.ROBOT_TOP_Z_MM, centered=(True, True, False))))
    return body


def main():
    part = attachment_solid()
    ok = True

    def rule(good, text):
        nonlocal ok
        print(("ok   " if good else "FAIL ") + text)
        ok = ok and good
    print("light plate:")
    ok_plate, led_z = L.checks(L.plate_solid())
    ok = ok and ok_plate
    print("attachment:")
    rule(part.val().isValid(), "valid solid")
    rule(len(part.val().Solids()) == 1, "one piece (%d solid)" % len(part.val().Solids()))
    bb = B.bbox(part)
    sz = (bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2])
    rule(max(sz[0], sz[1]) + 16 <= 200, "fits a 200 x 200 bed with a brim: %.0f x %.0f x %.0f mm" % sz)
    rule(abs(FRONT_LEG_X_MM) - FRONT_LEG_W_MM / 2 > D.WHEEL_R_MM + 3, "front legs clear the tyres")
    rule(D.ARM_OUTER_Y_MM * 2 <= D.POCKET_W_MM, "bracket still fits the dock pocket (%.1f in %.1f mm)"
         % (2 * D.ARM_OUTER_Y_MM, D.POCKET_W_MM))
    rule(L.PLATE_UNDER_Z_MM > D.WALL_H_MM + 5, "plate and legs above the dock walls (%.0f mm)" % D.WALL_H_MM)
    on_bed = part.rotate((0, 0, 0), (1, 0, 0), 180)
    b = on_bed.val().BoundingBox()
    on_bed = on_bed.translate((-b.xmin, -b.ymin, -b.zmin))
    sup = L.overhang_above(on_bed, max(L.PIXEL_POCKET_DEPTH_MM, L.CHANNEL_D_MM) + 0.3)
    print("     print upside down: %.0f mm^2 needs support (under the switch bar and the bracket arms); "
          "slicer: supports on, 'everywhere', tree supports if offered" % sup)
    os.makedirs(B.STL, exist_ok=True)
    stl = os.path.join(B.STL, "attachment.stl")
    on_bed.val().exportStl(stl, 0.05, 0.3)
    B.preview("attachment", part, os.path.join(B.PNG, "attachment.png"), plan_z=L.PLATE_UNDER_Z_MM + 1)
    L.plan_png(os.path.join(B.PNG, "light_plate_layout.png"))
    L.layout_json(led_z)
    print("wrote " + stl)
    if not ok:
        print("\nSOME CHECKS FAILED")
        sys.exit(1)


if __name__ == "__main__":
    main()
