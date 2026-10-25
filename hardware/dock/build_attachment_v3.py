"""Back-mounted attachment v3, pad sled, and dock v4: the robot BACKS into the dock and charges through two flat
pads under its rear, pressed from below by spring pins in a two-pronged tongue on the dock floor.

    python hardware/dock/build_attachment_v3.py

writes
    stl/attachment_v3.stl        the one attachment (Nick, 2026-10-25): bumper bar with the two switches at the
                                 BACK, camera light plate on top, cable clips under the plate
    stl/pad_sled.stl             a thin plate under the robot's rear carrying the two charging pads; screws to the
                                 bar's underside and is taped to the robot's flat underside
    stl/dock_v4.stl              dock v2's funnel and post stations (posts made taller for the higher rear bar), two
                                 bumps on the back wall for the switches, the two-pronged tongue with a spring pin in
                                 each prong, wiring grooves, cavity and cable channel
    stl/clearance_gauge.stl      fit test 1: six fingers 8.0-10.5 mm tall; slide them under the robot's rear to
                                 measure the real clearance (the one number everything here rests on)
    stl/attachment_v3_fit_test.stl  fit test 2: the bar and the lower arms only (about 25 min)
    stl/pogo_gauge_v.stl         fit test 3: five upright holes for the spring pins, to find the hole size and see
                                 the tip height
    preview/attachment_v3.png, pad_sled.png, dock_v4.png, docked_v4.png (sections), underside_v4.png (from below)
and prints a check report (ok / WARN / FAIL lines) and the firmware numbers for a back bumper.

Plan: docs/build-plans/self-charging-dock.md. Earlier versions (front bumper, pins in the back wall) stay as they
were: build_attachment_v2.py, stl/attachment_v2.stl, stl/dock_v3.stl.

Frames (mm)
  robot: X forward from the wheel axle, Y to the robot's left, Z up from the floor (as build_attachment.py)
  dock:  X into the dock from the back wall's face, Y left, Z up from the plywood (as dock_params.py).
         A docked robot faces OUT of the dock, so robot (x, y) -> dock (X_AXLE_DOCKED - x, -y).
Every number is SOURCED / MEASURED / ASSUMED (as dock_params.py) or CHOSEN.

The robot's underside (SOURCED [STEP]: Arduino's alvik_reference_design.step, measured 2026-10-25 by casting
vertical rays through it on a 4 mm grid and finer near the wheels and the caster; dock_params.py has the link):
  - flat at 10.1 mm above the floor everywhere between the wheels from the back edge to the front sensor board
    (one exception below);
  - the caster: a 9 mm steel ball ([DS] section 2.1) centred 41 mm behind the axle on the centre line, touching
    the floor; its holder hangs to 7.1 mm above the floor across +-9.5 mm and from 36 to 47 mm behind the axle;
  - the wheels: inner faces 41.2 mm either side of the centre line;
  - the back face's bottom edge: 10.1-10.4 mm up (the front's is 4.2: the line sensor board).
The model is a simplified shell: the battery cover, its screws and the real tyre squash are not in it. Fit test 1
measures the real clearance.
"""
import contextlib
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dock_params as D          # noqa: E402
import build as B                # noqa: E402
import build_light_plate as L    # noqa: E402
import build_attachment as A     # noqa: E402
import build_v2 as V2            # noqa: E402

# =============================================================================== the robot's underside
Z_UNDER_MM = 10.1              # SOURCED [STEP]: flat underside, behind the wheels. MEASURE: fit test 1
BACK_FACE_X_MM = -D.AXLE_TO_FRONT_MM   # SOURCED [STEP]: back face 47.4-48.0 behind the axle (as the front)
CASTER_X_MM = -41.0            # SOURCED [STEP]: ball centre, robot X
CASTER_BALL_R_MM = 4.5         # SOURCED [DS]: 9 mm steel ball
CASTER_HOLDER_HALF_W_MM = 9.5  # SOURCED [STEP]: holder +-9.5 mm wide ...
CASTER_HOLDER_X_MM = (-47.0, -36.0)   # ... from 36 to 47 mm behind the axle ...
CASTER_HOLDER_Z_MM = 7.1       # ... and hangs to 7.1 mm above the floor
WHEEL_INNER_Y_MM = 41.2        # SOURCED [STEP]: tyre inner face
TYRE_X_MM = 17.0               # SOURCED [STEP]: tyre within +-17 mm of the axle (radius 16.9)
CASTER_LOAD_N = 0.6            # ASSUMED: the robot's weight on the caster (192 g robot + about 30 g attachment;
                               # about 30 % on the caster). MEASURE: kitchen scale under the caster (check list)

# =============================================================================== the rear bumper bar
REAR_BAR_Z0_MM = Z_UNDER_MM    # CHOSEN: the bar's bottom level with the robot's underside, so the dock's pins
REAR_BAR_Z1_MM = Z_UNDER_MM + 10.0   # pass under it; 10 mm tall like the front bar it replaces
WIRE_HOLE_Y_MM = 29.0          # CHOSEN: the pad wires go up through the bar here (clear of the switch pockets)
WIRE_HOLE_DIA_MM = 2.6
WIRE_HOLE_X_MM = BACK_FACE_X_MM - 5.0
SLED_SCREW_Y_MM = 6.0          # CHOSEN: two M2 screws hold the sled to the bar's underside, either side of centre
SLED_SCREW_X_MM = BACK_FACE_X_MM - 5.5
M2_PILOT_MM = 1.8
M2_CLEAR_MM = 2.3
M2_HEAD_DIA_MM = 4.2           # SOURCED (ISO 7045 M2 pan head 4.0; +0.2)
M2_HEAD_H_MM = 1.3

# =============================================================================== the pad sled (robot side)
SLED_T_MM = 2.4                # CHOSEN: 12 layers; holds a 1.2 mm wire channel on top
PAD_X_MM = -24.0               # CHOSEN: pad centre 24 mm behind the axle: as far forward as the funnel allows
                               # (see checks: the caster must not reach the prongs before the funnel centres it).
                               # Further forward = less lift on the caster
PAD_Y_MM = 22.0                # CHOSEN: midway between the caster holder (9.5) and the wheels (41.2)
PAD_L_MM = 10.0                # along X
PAD_W_MM = 8.0                 # along Y
PAD_T_MM = 0.4                 # brass shim (0.4 mm, or 0.3-0.5: any) cut to 8 x 10 mm with a 3 mm tab; or copper-
                               # clad board. Sits in a recess so it is flush with the sled's underside
TAB_SLOT_MM = (1.6, 4.0)       # slot for the pad's bent-up solder tab, at the pad's front end
RAMP_L_MM = 4.0                # CHOSEN: ramp from the slot's roof down to the pad (31 deg)
SLOT_HALF_W_MM = 3.5           # CHOSEN: the slot the pin travels in, under the robot, before the ramp
SLOT_FLARE_HALF_W_MM = 7.0     # CHOSEN: the slot's mouth at the sled's back end, flared to catch an off-centre pin
SLOT_FLARE_L_MM = 8.0
SLED_HALF_W_MM = 36.0          # inside the wheels (41.2) with 5 mm to spare
CASTER_NOTCH_HALF_W_MM = 12.0  # caster holder +-9.5 plus 2.5
CASTER_NOTCH_FRONT_X_MM = -33.0
CH_W_MM = 2.0                  # wire channel on the sled's top (one 26-28 AWG wire)
CH_D_MM = 1.2
SLED_X0_MM = BACK_FACE_X_MM - D.FACE_X_MM + 3.0     # back end: 3 mm short of the bar's face
SLED_X1_MM = PAD_X_MM + PAD_L_MM / 2 + 3.0          # front end

# derived
Z_PAD_MM = Z_UNDER_MM - SLED_T_MM                    # the pads' contact face above the floor
RAMP_X0_MM = PAD_X_MM - PAD_L_MM / 2 - RAMP_L_MM    # where the ramp starts (the slot's end)

# =============================================================================== the dock
BUMP_OUT_MM = 4.0              # CHOSEN: the two switch bumps stand this far out of the back wall
BUMP_Y_MM = (30.0, D.POCKET_W_MM / 2)   # each bump from 30 mm out to the pocket's side wall: it presses the lever
                               # and stops the bar's face just inside the switch pocket (the outer rib is chamfered)
PRONG_HALF_W_MM = 6.0          # CHOSEN: prongs 12 mm wide, centred on the pads
PRONG_T_MM = 4.0               # CHOSEN: prong thickness (it lies on the plywood)
PRONG_NOSE_MM = 6.0            # prong runs this far past its pin, then a chamfered nose
PRONG_NOSE_CHAMFER_MM = 3.0
GROOVE_W_MM = 3.0              # wire groove under each prong (open to the plywood)
GROOVE_D_MM = 1.4
# Upright spring pins: none bought yet; every number ASSUMED from typical listings. Measure with fit test 3.
POGO_V_BARREL_DIA_MM = 3.0     # ASSUMED
POGO_V_PLUNGER_DIA_MM = 1.5    # ASSUMED
POGO_V_LEN_MM = 6.5            # ASSUMED: barrel bottom to tip, not pressed (solder tail not counted)
POGO_V_FREE_MM = 1.5           # ASSUMED: tip out of the barrel, not pressed ...
POGO_V_STROKE_MM = 1.5         # ASSUMED: ... and how far it can be pushed in
POGO_V_FORCE_N = 0.3           # ASSUMED: force per pin at 2/3 stroke. Buy LOW-FORCE pins (about 30 g or less):
                               # they push the robot's rear UP (checks: caster lift)
POGO_HOLE_PRESS_MM = 0.05      # CHOSEN: barrel hole this much over the barrel (glue holds it); fit test 3 sets it
COMPRESS_MM = 1.0              # CHOSEN: how far the pad pushes each pin in (2/3 of the stroke)
BOSS_WALL_MM = 1.2
CAVITY_X_MM = (6.0, 36.0)      # wiring cavity behind the wall, open underneath (as dock v3)
CAVITY_HALF_Y_MM = 27.0
CAVITY_TOP_Z_MM = 16.0
CHANNEL_HALF_W_MM = 3.0        # cable channel under the head to its back face (as dock v3)
CHANNEL_H_MM = 5.0
MARK_DIA_MM = 1.6
POST_H_MM = D.WALL_H_MM        # the post stations' posts, raised from 15 to the wall height: the rear bar is
                               # 10-20 mm up (its levers about 14.6), where dock v2's posts stop at 15

# derived: the docked robot, and the pins
X_AXLE_DOCKED_MM = -(BUMP_OUT_MM + D.FACE_X_MM + D.AXLE_TO_FRONT_MM)   # dock X of the wheel axle
X_PIN_MM = X_AXLE_DOCKED_MM - PAD_X_MM
X_TIP_MM = X_PIN_MM - PRONG_NOSE_MM
PIN_TIP_FREE_Z_MM = Z_PAD_MM + COMPRESS_MM
PIN_SEAT_Z_MM = PIN_TIP_FREE_Z_MM - POGO_V_LEN_MM
BARREL_TOP_Z_MM = PIN_SEAT_Z_MM + POGO_V_LEN_MM - POGO_V_FREE_MM
BOSS_TOP_Z_MM = BARREL_TOP_Z_MM - 0.5

# =============================================================================== cable clips under the plate
CLIP_W_MM = 5.0                # inside width: a USB-C cable is 3-4.5 mm
CLIP_H_MM = 4.5
CLIP_RIB_T_MM = 1.6
CLIP_LIP_MM = 0.6              # lips narrow the mouth to 3.8 mm: push the cable in, it stays
NANO_X_MM = 33.0               # ASSUMED from Arduino's picture (perspective_front.png): the Nano ESP32 lies across
                               # the front of the top, its USB-C at one end facing a side. MEASURE (check list)
CROSS_HALF_L_MM = 34.0         # front cross run, clear of the gussets
SPINE_Y_MM = 9.0               # spine beside the light plate's cable hole (at y = 0)


@contextlib.contextmanager
def patched(mod, **kw):
    old = {k: getattr(mod, k) for k in kw}
    for k, v in kw.items():
        setattr(mod, k, v)
    try:
        yield
    finally:
        for k, v in old.items():
            setattr(mod, k, v)


def box(x0, x1, y0, y1, z0, z1):
    c = B.cq()
    return (c.Workplane("XY").workplane(offset=z0).center((x0 + x1) / 2, (y0 + y1) / 2)
            .box(x1 - x0, y1 - y0, z1 - z0, centered=(True, True, False)))


def robot_to_dock(x, y):
    return X_AXLE_DOCKED_MM - x, -y


# =============================================================================== the attachment
def rear_bracket():
    """build.py's bracket at the rear bar's height, mirrored to the back (robot frame)."""
    with patched(D, BUMPER_Z0_MM=REAR_BAR_Z0_MM, BUMPER_Z1_MM=REAR_BAR_Z1_MM):
        br = B.bracket_solid().translate((D.AXLE_TO_FRONT_MM, 0, 0)).mirror("YZ")
    return br


def clips():
    """Cable clips under the plate (robot frame): a run across the front over the Nano (the plug can be at either
    end), a spine back to the rear, and a drop at the back towards the bar's wire holes."""
    c = B.cq()
    zt = L.PLATE_UNDER_Z_MM
    out = None

    def run(x0, y0, x1, y1):
        nonlocal out
        horiz = abs(y1 - y0) > abs(x1 - x0)
        for sg in (1, -1):
            off = sg * (CLIP_W_MM / 2 + CLIP_RIB_T_MM / 2)
            if horiz:
                rib = box(x0 + off - CLIP_RIB_T_MM / 2, x0 + off + CLIP_RIB_T_MM / 2, min(y0, y1), max(y0, y1),
                          zt - CLIP_H_MM, zt + 0.01)
                lip = box(x0 + sg * (CLIP_W_MM / 2 - CLIP_LIP_MM / 2) - CLIP_LIP_MM / 2,
                          x0 + sg * (CLIP_W_MM / 2 - CLIP_LIP_MM / 2) + CLIP_LIP_MM / 2 + 0.01 * sg,
                          min(y0, y1), max(y0, y1), zt - CLIP_H_MM, zt - CLIP_H_MM + 1.0)
            else:
                rib = box(min(x0, x1), max(x0, x1), y0 + off - CLIP_RIB_T_MM / 2, y0 + off + CLIP_RIB_T_MM / 2,
                          zt - CLIP_H_MM, zt + 0.01)
                lip = box(min(x0, x1), max(x0, x1), y0 + sg * (CLIP_W_MM / 2 - CLIP_LIP_MM / 2) - CLIP_LIP_MM / 2,
                          y0 + sg * (CLIP_W_MM / 2 - CLIP_LIP_MM / 2) + CLIP_LIP_MM / 2, zt - CLIP_H_MM,
                          zt - CLIP_H_MM + 1.0)
            piece = rib.union(lip)
            out = piece if out is None else out.union(piece)

    # front cross run, in two halves so the spine can join in the middle
    run(NANO_X_MM, -CROSS_HALF_L_MM, NANO_X_MM, -(SPINE_Y_MM + CLIP_W_MM / 2 + CLIP_RIB_T_MM + 2))
    run(NANO_X_MM, SPINE_Y_MM + CLIP_W_MM / 2 + CLIP_RIB_T_MM + 2, NANO_X_MM, CROSS_HALF_L_MM)
    # spine, from behind the cross run to the plate's back edge
    run(NANO_X_MM - CLIP_W_MM, SPINE_Y_MM, L.PLATE_X_MM[0] + 6.0, SPINE_Y_MM)
    return out


def attachment_v3_solid():
    c = B.cq()
    with patched(L, BACK_INSERT_X_MM=-L.BACK_INSERT_X_MM):      # the plate's own legs on the FRONT insert pair
        plate = L.plate_solid()
    body = plate.union(rear_bracket())
    yo = D.ARM_OUTER_Y_MM
    yi = yo - D.ARM_T_MM
    xl = -A.FRONT_LEG_X_MM                                       # the back insert pair (-28.0)
    for sg in (1, -1):
        # rear legs: the bar's arms carried up to the plate (A's front legs, mirrored)
        body = body.union(box(xl - A.FRONT_LEG_W_MM / 2, xl + A.FRONT_LEG_W_MM / 2, sg * yi + (0 if sg > 0 else -D.ARM_T_MM),
                              sg * yi + (D.ARM_T_MM if sg > 0 else 0), D.ARM_TOP_Z_MM - 4.0, L.PLATE_UNDER_Z_MM + 0.01))
        g = (c.Workplane("YZ").workplane(offset=xl - A.FRONT_LEG_W_MM / 2)
             .polyline([(sg * yi, L.PLATE_UNDER_Z_MM), (sg * yi, L.PLATE_UNDER_Z_MM - L.GUSSET_MM),
                        (sg * (yi - L.GUSSET_MM), L.PLATE_UNDER_Z_MM)]).close().extrude(A.FRONT_LEG_W_MM))
        body = body.union(g)
    body = body.union(clips())
    # the shell stays clear (the envelope runs from the back face forward)
    env = B.robot_envelope().translate((BACK_FACE_X_MM + 200.0, 0, 0))
    body = body.cut(env.intersect(c.Workplane("XY").box(400, 400, D.ROBOT_TOP_Z_MM, centered=(True, True, False))))
    # pad wire holes straight up through the bar; M2 pilot holes for the sled in its underside
    for sg in (1, -1):
        body = body.cut(B.cyl(WIRE_HOLE_DIA_MM / 2, REAR_BAR_Z1_MM - REAR_BAR_Z0_MM + 2,
                              (WIRE_HOLE_X_MM, sg * WIRE_HOLE_Y_MM, REAR_BAR_Z0_MM - 1), (0, 0, 1)))
        body = body.cut(B.cyl(M2_PILOT_MM / 2, 7.0, (SLED_SCREW_X_MM, sg * SLED_SCREW_Y_MM, REAR_BAR_Z0_MM - 1),
                              (0, 0, 1)))
    return body


# =============================================================================== the pad sled
def sled_solid():
    """Robot frame. Top face against the robot's underside and the bar's underside (both at Z_UNDER_MM)."""
    c = B.cq()
    z0, z1 = Z_PAD_MM, Z_UNDER_MM
    body = box(SLED_X0_MM, SLED_X1_MM, -SLED_HALF_W_MM, SLED_HALF_W_MM, z0, z1)
    body = body.edges("|Z").fillet(2.0)
    # caster notch (behind the back face the sled is a solid crossbar under the bar)
    body = body.cut(box(BACK_FACE_X_MM - 0.4, CASTER_NOTCH_FRONT_X_MM, -CASTER_NOTCH_HALF_W_MM, CASTER_NOTCH_HALF_W_MM,
                        z0 - 1, z1 + 1))
    for sg in (1, -1):
        y = sg * PAD_Y_MM
        # the pin's slot, flared at the back end
        body = body.cut(box(SLED_X0_MM - 1, RAMP_X0_MM, y - SLOT_HALF_W_MM, y + SLOT_HALF_W_MM, z0 - 1, z1 + 1))
        fl = (c.Workplane("XY").workplane(offset=z0 - 1)
              .polyline([(SLED_X0_MM - 1, y - SLOT_FLARE_HALF_W_MM), (SLED_X0_MM + SLOT_FLARE_L_MM, y - SLOT_HALF_W_MM),
                         (SLED_X0_MM + SLOT_FLARE_L_MM, y + SLOT_HALF_W_MM), (SLED_X0_MM - 1, y + SLOT_FLARE_HALF_W_MM)])
              .close().extrude(SLED_T_MM + 2))
        body = body.cut(fl)
        # the ramp: from the slot's roof (the robot's underside) down to the pad
        ramp = (c.Workplane("XZ").polyline([(RAMP_X0_MM - 0.01, z0 - 1), (RAMP_X0_MM - 0.01, z1 + 0.5),
                                             (RAMP_X0_MM + RAMP_L_MM, z0), (RAMP_X0_MM + RAMP_L_MM, z0 - 1)])
                .close().extrude(-(2 * SLOT_HALF_W_MM)).translate((0, y - SLOT_HALF_W_MM, 0)))
        body = body.cut(ramp)
        # pad recess (underside) and the tab slot through to the top
        body = body.cut(box(PAD_X_MM - PAD_L_MM / 2, PAD_X_MM + PAD_L_MM / 2, y - PAD_W_MM / 2, y + PAD_W_MM / 2,
                            z0 - 1, z0 + PAD_T_MM))
        xt = PAD_X_MM + PAD_L_MM / 2 - TAB_SLOT_MM[0] / 2 - 0.5
        body = body.cut(box(xt - TAB_SLOT_MM[0] / 2, xt + TAB_SLOT_MM[0] / 2, y - TAB_SLOT_MM[1] / 2, y + TAB_SLOT_MM[1] / 2,
                            z0 - 1, z1 + 1))
        # wire channel on top: out from the tab to WIRE_HOLE_Y, then back to under the bar's wire hole
        yh = sg * WIRE_HOLE_Y_MM
        body = body.cut(box(xt - CH_W_MM / 2, xt + CH_W_MM / 2, min(y, yh) - CH_W_MM / 2, max(y, yh) + CH_W_MM / 2,
                            z1 - CH_D_MM, z1 + 1))
        body = body.cut(box(WIRE_HOLE_X_MM - WIRE_HOLE_DIA_MM / 2, xt + CH_W_MM / 2, yh - CH_W_MM / 2, yh + CH_W_MM / 2,
                            z1 - CH_D_MM, z1 + 1))
        # M2 screws up into the bar, heads sunk flush in the underside
        p = (SLED_SCREW_X_MM, sg * SLED_SCREW_Y_MM)
        body = body.cut(B.cyl(M2_CLEAR_MM / 2, SLED_T_MM + 2, (p[0], p[1], z0 - 1), (0, 0, 1)))
        body = body.cut(B.cyl(M2_HEAD_DIA_MM / 2, M2_HEAD_H_MM + 1, (p[0], p[1], z0 - 1), (0, 0, 1)))
    # chamfer the back end's underside: a pin that meets the crossbar off its slot rides up instead of catching
    ch = (c.Workplane("XZ").polyline([(SLED_X0_MM - 1, z0 - 1), (SLED_X0_MM - 1, z0 + 1.5), (SLED_X0_MM + 2.5, z0 - 0.01),
                                      (SLED_X0_MM + 2.5, z0 - 1)]).close().extrude(-100).translate((0, -50, 0)))
    body = body.cut(ch)
    return body


# =============================================================================== the dock
def dock_v4_solid():
    c = B.cq()
    with patched(V2, POST_H_MM=POST_H_MM):
        body = V2.dock_v2_solid()
    # switch bumps on the back wall
    for sg in (1, -1):
        ya, yb = sorted((sg * BUMP_Y_MM[0], sg * (BUMP_Y_MM[1] + 0.5)))
        body = body.union(box(-BUMP_OUT_MM, 1.0, ya, yb, 0, D.WALL_H_MM))
    # the tongue: two prongs, each with an upright spring pin in a boss
    for sg in (1, -1):
        y = sg * PAD_Y_MM
        prong = box(X_TIP_MM, 2.0, y - PRONG_HALF_W_MM, y + PRONG_HALF_W_MM, 0, PRONG_T_MM)
        prong = prong.faces("<X").edges(">Z").chamfer(PRONG_NOSE_CHAMFER_MM, PRONG_T_MM - 1.0)
        body = body.union(prong)
        body = body.union(B.cyl(POGO_V_BARREL_DIA_MM / 2 + BOSS_WALL_MM, BOSS_TOP_Z_MM, (X_PIN_MM, y, 0), (0, 0, 1)))
        body = body.cut(B.cyl(POGO_V_BARREL_DIA_MM / 2 + POGO_HOLE_PRESS_MM, BOSS_TOP_Z_MM - PIN_SEAT_Z_MM + 1,
                              (X_PIN_MM, y, PIN_SEAT_Z_MM), (0, 0, 1)))
        body = body.cut(B.cyl(0.7, PIN_SEAT_Z_MM + 1.01, (X_PIN_MM, y, -1), (0, 0, 1)))       # solder tail
        body = body.cut(box(X_PIN_MM - GROOVE_W_MM / 2, CAVITY_X_MM[0] + 1, y - GROOVE_W_MM / 2, y + GROOVE_W_MM / 2,
                            -1, GROOVE_D_MM))
    body = body.cut(box(CAVITY_X_MM[0], CAVITY_X_MM[1], -CAVITY_HALF_Y_MM, CAVITY_HALF_Y_MM, -1, CAVITY_TOP_Z_MM))
    body = body.cut(box(CAVITY_X_MM[1] - 1, V2.HEAD_DEPTH_MM + 2, -CHANNEL_HALF_W_MM, CHANNEL_HALF_W_MM, -1, CHANNEL_H_MM))
    # + mark on top, above the + pin (the robot's left pad: dock Y negative)
    body = body.cut(c.Workplane("XY").workplane(offset=D.WALL_H_MM - 0.8).center(3.0, -PAD_Y_MM)
                    .circle(MARK_DIA_MM / 2).extrude(2))
    return body


# =============================================================================== fit-test parts
def clearance_gauge_solid():
    """Six fingers, 8.0 to 10.5 mm tall, on a 3 mm spine; the height is engraved as notches (1 notch = 8.0)."""
    c = B.cq()
    hs = [8.0, 8.5, 9.0, 9.5, 10.0, 10.5]
    pitch, fw, fl = 12.0, 8.0, 40.0
    body = box(0, 10.0, 0, pitch * len(hs), 0, 3.0)
    for i, h in enumerate(hs):
        y0 = i * pitch + (pitch - fw) / 2
        body = body.union(box(9.0, 10.0 + fl, y0, y0 + fw, 0, h))
        for k in range(i + 1):
            body = body.cut(box(1.0 + 1.4 * k, 1.8 + 1.4 * k, y0 + 1, y0 + fw - 1, 2.4, 4.0))
    return body, hs


def pogo_gauge_v_solid():
    """A prong-shaped block with five upright pin holes, barrel -0.2 ... +0.2 mm, each seated as in the dock."""
    sizes = [POGO_V_BARREL_DIA_MM + d for d in (-0.2, -0.1, 0.0, 0.1, 0.2)]
    pitch = 9.0
    w = pitch * len(sizes) + 4
    blk = box(0, 12.0, -w / 2, w / 2, 0, BOSS_TOP_Z_MM)
    for i, d in enumerate(sizes):
        y = -w / 2 + 2 + pitch * (i + 0.5)
        blk = blk.cut(B.cyl(d / 2, BOSS_TOP_Z_MM, (6.0, y, PIN_SEAT_Z_MM), (0, 0, 1)))
        blk = blk.cut(B.cyl(0.7, PIN_SEAT_Z_MM + 1, (6.0, y, -1), (0, 0, 1)))
        blk = blk.cut(box(6.0 - GROOVE_W_MM / 2, 13.0, y - GROOVE_W_MM / 2, y + GROOVE_W_MM / 2, -1, GROOVE_D_MM))
    blk = blk.cut(box(-1, 2.0, -w / 2 - 1, -w / 2 + 2.0, BOSS_TOP_Z_MM - 1.5, BOSS_TOP_Z_MM + 1))   # notch: smallest end
    return blk, sizes


# =============================================================================== checks
def checks(att, sled, dock):
    lines, ok = [], True

    def rule(good, text, warn=False):
        nonlocal ok
        if not good and not warn:
            ok = False
        lines.append(("ok   " if good else ("WARN " if warn else "FAIL ")) + text)

    rule(att.val().isValid() and len(att.val().Solids()) == 1, "attachment_v3: one valid solid")
    rule(sled.val().isValid() and len(sled.val().Solids()) == 1, "pad sled: one valid solid")
    rule(dock.val().isValid() and len(dock.val().Solids()) == 1, "dock_v4: one valid solid")
    b = B.bbox(att)
    rule(max(b[3] - b[0], b[4] - b[1]) + 16 <= D.BED_MM[0], "attachment_v3 fits the bed: %.0f x %.0f x %.0f mm"
         % (b[3] - b[0], b[4] - b[1], b[5] - b[2]))
    rule(abs(b[4] - b[1] - (D.BRACKET_W_MM + 1.1)) < 2.0 and D.BRACKET_W_MM < D.POCKET_W_MM,
         "rear bar as wide as the front bracket was (%.1f mm): the dock pocket (%.1f) and the post spacing still fit"
         % (D.BRACKET_W_MM, D.POCKET_W_MM))
    # --- the height stack under the robot
    rule(True, "height stack above the plywood (mm): prong top %.1f, boss top %.1f, barrel top %.1f, pin tip free %.1f, "
         "pad face %.1f, robot underside and bar bottom %.1f (SOURCED [STEP]; fit test 1 measures it)"
         % (PRONG_T_MM, BOSS_TOP_Z_MM, BARREL_TOP_Z_MM, PIN_TIP_FREE_Z_MM, Z_PAD_MM, Z_UNDER_MM))
    run_clear = Z_UNDER_MM - PIN_TIP_FREE_Z_MM
    rule(run_clear >= 1.0, "a free pin tip passes %.1f mm under the robot and the bar (1.0 or more) while it travels in "
         "its slot" % run_clear)
    rule(0.4 * POGO_V_STROKE_MM <= COMPRESS_MM <= 0.8 * POGO_V_STROKE_MM,
         "on the pad each pin is pushed in %.1f mm of its %.1f mm stroke (%.0f %%)"
         % (COMPRESS_MM, POGO_V_STROKE_MM, 100 * COMPRESS_MM / POGO_V_STROKE_MM))
    rule(Z_PAD_MM - BARREL_TOP_Z_MM >= 0.3, "pad face %.1f mm above the barrel's top: the pin never bottoms out"
         % (Z_PAD_MM - BARREL_TOP_Z_MM))
    rule(PIN_SEAT_Z_MM - GROOVE_D_MM >= 0.6, "%.1f mm of prong under each barrel, above the wire groove"
         % (PIN_SEAT_Z_MM - GROOVE_D_MM))
    rule(Z_PAD_MM >= 7.0, "lowest point the robot carries: the pads at %.1f mm (the caster's holder is at %.1f): "
         "nothing rubs the plywood" % (Z_PAD_MM, CASTER_HOLDER_Z_MM))
    rule(Z_UNDER_MM - Z_PAD_MM <= Z_UNDER_MM - 7.0, "sled %.1f mm thick" % SLED_T_MM)
    # --- sideways: prongs between the caster and the wheels
    p_in, p_out = PAD_Y_MM - PRONG_HALF_W_MM, PAD_Y_MM + PRONG_HALF_W_MM
    rule(p_in - CASTER_BALL_R_MM >= 5.0 and BOSS_TOP_Z_MM < CASTER_HOLDER_Z_MM or p_in >= CASTER_HOLDER_HALF_W_MM + 2,
         "prongs (%.0f-%.0f mm out) miss the caster ball (%.1f) and pass under its holder (%.1f mm up; prongs %.1f, "
         "bosses %.1f)" % (p_in, p_out, CASTER_BALL_R_MM, CASTER_HOLDER_Z_MM, PRONG_T_MM, BOSS_TOP_Z_MM))
    rule(WHEEL_INNER_Y_MM - p_out >= 8.0, "prongs %.1f mm inside the wheels" % (WHEEL_INNER_Y_MM - p_out))
    rule(PAD_Y_MM + SLOT_HALF_W_MM + 1.0 <= WIRE_HOLE_Y_MM - CH_W_MM / 2 and WIRE_HOLE_Y_MM + CH_W_MM / 2 + 1.0 <= SLED_HALF_W_MM,
         "sled wire channels run between the pin slots and the sled's edge")
    sw_inner = D.SWITCH_Y_OUTER_MM - D.SWITCH_L_MM - 0.4
    rule(WIRE_HOLE_Y_MM + WIRE_HOLE_DIA_MM / 2 + 1.5 <= sw_inner, "bar wire holes (%.1f mm out) clear of the switch "
         "pockets (from %.1f)" % (WIRE_HOLE_Y_MM, sw_inner))
    rule(CASTER_NOTCH_HALF_W_MM >= CASTER_HOLDER_HALF_W_MM + 2 and CASTER_NOTCH_FRONT_X_MM >= CASTER_HOLDER_X_MM[1] + 2,
         "sled notched around the caster's holder with 2 mm or more to spare")
    # --- along the axis: where the caster and the wheels meet the prongs
    x_bar_at_tip = X_TIP_MM + (D.FACE_X_MM + D.AXLE_TO_FRONT_MM + CASTER_X_MM)
    off = D.funnel_half_w(x_bar_at_tip) - D.BRACKET_W_MM / 2
    rule(off + CASTER_BALL_R_MM <= p_in - 0.5,
         "when the caster reaches the prongs' tips the funnel already holds the bar within %.1f mm of centre: the ball "
         "(out to %.1f) misses the prongs (from %.1f)" % (off, off + CASTER_BALL_R_MM, p_in))
    rule(X_AXLE_DOCKED_MM + TYRE_X_MM / 3 < X_TIP_MM,
         "docked, the tyres touch the plywood %.0f mm short of the prongs' tips: the wheels never roll onto them"
         % (X_TIP_MM - X_AXLE_DOCKED_MM - TYRE_X_MM / 3))
    x_bar_at_sled = X_PIN_MM + SLED_X0_MM + D.AXLE_TO_FRONT_MM + D.FACE_X_MM     # bar face when a pin meets the sled
    off2 = D.funnel_half_w(x_bar_at_sled) - D.BRACKET_W_MM / 2
    rule(off2 <= SLOT_FLARE_HALF_W_MM - POGO_V_PLUNGER_DIA_MM / 2,
         "when the pins reach the sled's back end the funnel holds the robot within %.1f mm; the slots' flared mouths "
         "take %.1f" % (off2, SLOT_FLARE_HALF_W_MM - POGO_V_PLUNGER_DIA_MM / 2), warn=True)
    # --- the pin on the pad
    play = D.POCKET_CLEAR_MM / 2
    rule(PAD_W_MM / 2 - POGO_V_PLUNGER_DIA_MM / 2 >= 2 * (play + 0.5),
         "sideways: the pin lands on the pad with %.1f mm to spare; docked, the robot is within %.1f mm (pocket play)"
         % (PAD_W_MM / 2 - POGO_V_PLUNGER_DIA_MM / 2, play))
    rule(PAD_L_MM / 2 - POGO_V_PLUNGER_DIA_MM / 2 >= 3.0,
         "along: the pin rests %.1f mm from either end of the pad (the bumps stop the robot to about 0.5 mm)"
         % (PAD_L_MM / 2 - POGO_V_PLUNGER_DIA_MM / 2))
    # --- the springs lift the robot's rear
    F = 2 * POGO_V_FORCE_N
    lift = F * abs(PAD_X_MM) / abs(CASTER_X_MM)
    rule(lift <= 0.5 * CASTER_LOAD_N,
         "the two pins push up %.2f N (%.0f g), %.0f mm behind the axle: that takes %.2f N off the caster, %.0f %% of its "
         "ASSUMED %.1f N load (keep under 50 %%; measure the load and the pins' force)"
         % (F, F / 9.81 * 1000, abs(PAD_X_MM), lift, 100 * lift / CASTER_LOAD_N, CASTER_LOAD_N), warn=True)
    # --- the bumps and the lever
    rule(D.FACE_ABOVE_BODY_MM > D.LEVER_PRESSED_MM and D.LEVER_FREE_MM > D.FACE_ABOVE_BODY_MM,
         "each bump presses its lever %.1f mm (it clicks at %.1f) and stops on the bar's face before the lever "
         "bottoms" % (D.LEVER_FREE_MM - D.FACE_ABOVE_BODY_MM, D.LEVER_FREE_MM - D.LEVER_OPERATE_MM))
    rule(BUMP_Y_MM[0] < D.SWITCH_Y_OUTER_MM - D.SWITCH_L_MM - 0.4 - 2.0,
         "bumps reach %.1f mm inside the switch pockets' inner ends: a flat face to stop on" %
         (D.SWITCH_Y_OUTER_MM - D.SWITCH_L_MM - 0.4 - BUMP_Y_MM[0]))
    rule(BUMP_Y_MM[0] > p_out + 1.0, "bumps clear of the prongs")
    z_lever = REAR_BAR_Z0_MM + D.SHELF_T_MM + D.SWITCH_T_MM / 2
    rule(POST_H_MM >= REAR_BAR_Z1_MM, "post stations: posts %.0f mm tall, the rear bar %.1f-%.1f mm, levers about %.1f"
         % (POST_H_MM, REAR_BAR_Z0_MM, REAR_BAR_Z1_MM, z_lever))
    with patched(D, BUMPER_Z0_MM=REAR_BAR_Z0_MM, BUMPER_Z1_MM=REAR_BAR_Z1_MM):
        with patched(V2, POST_H_MM=POST_H_MM):
            ok2, l2 = V2.checks(dock)
    lines += ["     (dock v2's checks on dock_v4) " + l for l in l2 if "nothing but the posts" not in l or l.startswith("FAIL")]
    ok = ok and ok2
    rule(D.WALL_H_MM - D.TILE_T_MM + 0.2 - CAVITY_TOP_Z_MM >= 3.0, "cavity roof %.1f mm under the check tile"
         % (D.WALL_H_MM - D.TILE_T_MM + 0.2 - CAVITY_TOP_Z_MM))
    rule(CAVITY_HALF_Y_MM + 3 < min(abs(y) for _, y in V2.SCREW_HOLES) - V2.SCREW_CSK_DIA_MM / 2,
         "cavity clear of the head's screw holes")
    rule(L.PLATE_UNDER_Z_MM - CLIP_H_MM - D.ROBOT_TOP_Z_MM >= 10, "cable clips %.0f mm above the robot's top (its touch "
         "buttons stay clear)" % (L.PLATE_UNDER_Z_MM - CLIP_H_MM - D.ROBOT_TOP_Z_MM))
    return ok, lines


def firmware_lines():
    return ["the bumper is now at the BACK: its face %.2f cm behind the axle (was bump_ahead_cm), switches +-%.2f cm "
            "(bump_half_cm), sw_gap_cm %.3f, bumper_cm %.2f behind the shell"
            % ((D.AXLE_TO_FRONT_MM + D.FACE_X_MM) / 10, D.SWITCH_LATERAL_MM / 10, D.SW_GAP_MM / 10, D.FACE_X_MM / 10),
            "docked, the axle is %.1f cm from the dock's back wall (the bumps add %.1f cm); the map's dock pose is the "
            "middle of the two bumps' faces, heading = the way a docked robot faces (out of the dock)"
            % (-X_AXLE_DOCKED_MM / 10, BUMP_OUT_MM / 10)]


# =============================================================================== pictures
def docked_png(att, sled, dock, path):
    """Side section through the + pin and a top section at 6 mm: robot, bar, sled, pins and prongs, docked."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle, Polygon
    from build_attachment_v2 import _fill
    blue, sledc, dark, gold, grey, red = "#2f5d8a", "#5b8c3a", "#3a3a3a", "#b8860b", "#d9dee2", "#c0392b"
    c = B.cq()
    # robot -> dock: rotate 180 deg about Z, then shift
    to_dock = lambda s: s.rotate((0, 0, 0), (0, 0, 1), 180).translate((X_AXLE_DOCKED_MM, 0, 0))
    attd, sledd = to_dock(att), to_dock(sled)
    fig, axs = plt.subplots(2, 1, figsize=(12, 10.5), dpi=115, gridspec_kw={"height_ratios": [1.0, 1.25]})
    ax = axs[0]
    yp = -PAD_Y_MM                            # + pin (robot's left pad)
    xb = X_AXLE_DOCKED_MM + D.AXLE_TO_FRONT_MM
    ax.add_patch(Polygon([(xb, Z_UNDER_MM), (xb - 95.25, Z_UNDER_MM), (xb - 95.25, 37.2), (xb, 37.2)], fc=grey, ec="#999"))
    ax.add_patch(Circle((X_AXLE_DOCKED_MM, D.WHEEL_R_MM), D.WHEEL_R_MM, fc="#555", alpha=0.35))
    ax.add_patch(Circle((X_AXLE_DOCKED_MM - CASTER_X_MM, CASTER_BALL_R_MM), CASTER_BALL_R_MM, fc="#888"))
    _fill(ax, dock, "XZ", yp, 0, 2, dark)
    _fill(ax, attd, "XZ", yp, 0, 2, blue)
    _fill(ax, sledd, "XZ", yp, 0, 2, sledc)
    r = POGO_V_BARREL_DIA_MM / 2
    ax.add_patch(Rectangle((X_PIN_MM - r, PIN_SEAT_Z_MM), 2 * r, BARREL_TOP_Z_MM - PIN_SEAT_Z_MM, fc=gold))
    ax.add_patch(Rectangle((X_PIN_MM - POGO_V_PLUNGER_DIA_MM / 2, BARREL_TOP_Z_MM), POGO_V_PLUNGER_DIA_MM,
                           Z_PAD_MM - BARREL_TOP_Z_MM, fc=gold))
    ax.add_patch(Rectangle((robot_to_dock(PAD_X_MM, 0)[0] - PAD_L_MM / 2, Z_PAD_MM), PAD_L_MM, PAD_T_MM, fc="#d4a017"))
    ax.axhline(0, color="k", lw=0.8)
    ax.set_xlim(-130, 25)
    ax.set_ylim(-3, 42)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.set_title("from the side, cut through the + pad and pin (robot's left, dock Y = %.0f)" % yp, fontsize=10)
    ax.set_xlabel("mm into the dock from the back wall's face")
    ax.set_ylabel("mm above the plywood")
    ax = axs[1]
    ax.add_patch(Rectangle((xb - 95.25, -48), 95.25, 96, fc=grey, ec="#999"))
    for sg in (1, -1):
        ax.add_patch(Rectangle((X_AXLE_DOCKED_MM - TYRE_X_MM, sg * WHEEL_INNER_Y_MM + (0 if sg > 0 else -7)), 2 * TYRE_X_MM,
                               7, fc="#555", alpha=0.5))
    ax.add_patch(Rectangle((X_AXLE_DOCKED_MM - CASTER_HOLDER_X_MM[1], -CASTER_HOLDER_HALF_W_MM),
                           CASTER_HOLDER_X_MM[1] - CASTER_HOLDER_X_MM[0], 2 * CASTER_HOLDER_HALF_W_MM, fc="#aaa"))
    ax.add_patch(Circle((X_AXLE_DOCKED_MM - CASTER_X_MM, 0), CASTER_BALL_R_MM, fc="#777"))
    _fill(ax, sledd, "XY", Z_PAD_MM + 0.5, 0, 1, sledc, alpha=0.8)
    _fill(ax, dock, "XY", 2.0, 0, 1, dark, alpha=0.9)
    _fill(ax, attd, "XY", REAR_BAR_Z0_MM + 0.5, 0, 1, blue, alpha=0.9)
    for sg in (1, -1):
        ax.add_patch(Circle((X_PIN_MM, sg * PAD_Y_MM), POGO_V_BARREL_DIA_MM / 2, fc=gold))
        ax.text(X_PIN_MM - 8, sg * PAD_Y_MM + sg * 9, "+ pin" if sg < 0 else "- pin", fontsize=8, ha="center")
    ax.text(X_AXLE_DOCKED_MM - CASTER_X_MM, 13, "caster", fontsize=8, ha="center")
    ax.text(X_AXLE_DOCKED_MM, 34, "wheel", fontsize=8, ha="center")
    ax.set_xlim(-130, 25)
    ax.set_ylim(-75, 75)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.set_title("from above, cut 2 mm up (dock and prongs), 8 mm up (sled) and 10.6 mm up (rear bar)", fontsize=10)
    ax.set_xlabel("mm into the dock from the back wall's face")
    ax.set_ylabel("mm left of the dock's axis")
    fig.text(0.5, 0.015, "Drawn from the CAD: a robot (grey) backed into dock v4 (black). Its rear bar (blue) rests on "
             "the two bumps; the dock's two prongs lie on the plywood either side of the caster, and the spring pin "
             "(gold) in each presses up on a brass pad in the sled (green) under the robot. The robot's underside, "
             "wheels and caster are Arduino's CAD numbers.", ha="center", fontsize=9.5, wrap=True)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def underside_png(att, sled, path):
    """The robot seen from below with the sled, the caster and the wheels, and the prongs' track."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle
    from build_attachment_v2 import _fill
    fig, ax = plt.subplots(figsize=(8.5, 8.0), dpi=115)
    ax.add_patch(Rectangle((-D.AXLE_TO_FRONT_MM, -48), 95.25, 96, fc="#e3e7ea", ec="#999"))
    _fill(ax, att, "XY", REAR_BAR_Z0_MM + 0.5, 0, 1, "#2f5d8a", alpha=0.9)
    _fill(ax, sled, "XY", Z_PAD_MM + 0.2, 0, 1, "#5b8c3a")
    for sg in (1, -1):
        ax.add_patch(Rectangle((-TYRE_X_MM, sg * WHEEL_INNER_Y_MM + (0 if sg > 0 else -6.8)), 2 * TYRE_X_MM, 6.8, fc="#444"))
        ax.add_patch(Rectangle((PAD_X_MM - PAD_L_MM / 2, sg * PAD_Y_MM - PAD_W_MM / 2), PAD_L_MM, PAD_W_MM, fc="#d4a017"))
        # the prong's track: where it lies once docked
        x_tip_r = X_AXLE_DOCKED_MM - X_TIP_MM
        x_wall_r = X_AXLE_DOCKED_MM             # the back wall's face, robot X
        ax.add_patch(Rectangle((x_wall_r, sg * PAD_Y_MM - PRONG_HALF_W_MM), x_tip_r - x_wall_r, 2 * PRONG_HALF_W_MM,
                               fc="none", ec="#222", ls="--", lw=1.0))
    ax.add_patch(Rectangle((CASTER_HOLDER_X_MM[0], -CASTER_HOLDER_HALF_W_MM), CASTER_HOLDER_X_MM[1] - CASTER_HOLDER_X_MM[0],
                           2 * CASTER_HOLDER_HALF_W_MM, fc="#aaa"))
    ax.add_patch(Circle((CASTER_X_MM, 0), CASTER_BALL_R_MM, fc="#666"))
    ax.text(CASTER_X_MM, 0, "caster", fontsize=8, ha="center", va="center", color="w")
    ax.text(0, 0, "flat underside\n%.1f mm up" % Z_UNDER_MM, fontsize=9, ha="center", va="center", color="#555")
    ax.text(30, 0, "front: line sensors\n(4.2 mm up)", fontsize=8, ha="center", va="center", color="#555")
    ax.set_xlim(-70, 55)
    ax.set_ylim(-60, 60)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.set_xlabel("mm forward of the wheel axle")
    ax.set_ylabel("mm left of the centre line (seen from above, through the robot)")
    ax.set_title("The robot's underside with the pad sled", fontsize=11)
    fig.text(0.5, 0.02, "Seen from above through the robot: the sled (green) carries the two brass pads (gold) "
             "between the caster and the wheels; dashed is where the dock's prongs lie once docked. Underside "
             "numbers from Arduino's CAD model.", ha="center", fontsize=9, wrap=True)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(path)
    plt.close(fig)


# =============================================================================== main
def on_bed(shape, flip=False):
    s = shape.rotate((0, 0, 0), (1, 0, 0), 180) if flip else shape
    b = s.val().BoundingBox()
    return s.translate((-b.xmin, -b.ymin, -b.zmin))


def export(shape, name):
    B.cq().exporters.export(shape, os.path.join(B.STL, name), tolerance=0.05, angularTolerance=0.1)


def main():
    c = B.cq()
    os.makedirs(B.STL, exist_ok=True)
    os.makedirs(B.PNG, exist_ok=True)
    att = attachment_v3_solid()
    sled = sled_solid()
    dock = dock_v4_solid()
    ok, lines = checks(att, sled, dock)
    print("attachment v3, pad sled, dock v4:")
    print("\n".join(lines))
    a_bed = on_bed(att, flip=True)
    sup = L.overhang_above(a_bed, max(L.PIXEL_POCKET_DEPTH_MM, L.CHANNEL_D_MM) + 0.3)
    print("     attachment_v3: print upside down (plate's top face on the bed), supports on, tree if offered: about "
          "%.0f mm^2 needs them (version 1: 1450)" % sup)
    a_bed.val().exportStl(os.path.join(B.STL, "attachment_v3.stl"), 0.05, 0.3)
    s_bed = on_bed(sled, flip=True)
    so = B.overhang_mm2(s_bed)
    ch_area = 2 * CH_W_MM * ((PAD_X_MM + PAD_L_MM / 2 - WIRE_HOLE_X_MM) + (WIRE_HOLE_Y_MM - PAD_Y_MM) + 4)
    print(("ok   " if so < ch_area + 40 else "WARN ") + "pad sled: print top face down (as exported), no supports; %.0f "
          "mm^2 faces down, about %.0f of it the roofs of the two wire channels (2 mm bridges); the pad recesses and "
          "screw counterbores face up" % (so, ch_area))
    export(s_bed, "pad_sled.stl")
    d_bed = on_bed(dock)
    over = B.overhang_mm2(d_bed)
    bridge = ((CAVITY_X_MM[1] - CAVITY_X_MM[0]) * 2 * CAVITY_HALF_Y_MM
              + (V2.HEAD_DEPTH_MM - CAVITY_X_MM[1]) * 2 * CHANNEL_HALF_W_MM + 2 * (CAVITY_X_MM[0] - X_PIN_MM) * GROOVE_W_MM)
    print(("ok   " if over <= bridge + 80 else "WARN ") + "dock_v4: print bottom down; %.0f mm^2 faces down, about %.0f "
          "of it the cavity roof, the cable channel and the prongs' wire grooves (bridges); support 'touching build "
          "plate' only if your printer sags" % (over, bridge))
    export(d_bed, "dock_v4.stl")
    # fit tests
    g, hs = clearance_gauge_solid()
    export(on_bed(g), "clearance_gauge.stl")
    print("ok   clearance gauge: fingers %s mm (one notch = the lowest), bottom down, no support, about 25 min"
          % " / ".join("%.1f" % h for h in hs))
    zt = D.SIDE_INSERT_Z_MM[0] + 3.3
    x_front = BACK_FACE_X_MM - D.ARM_BACK_X_MM + 1.0
    fit = att.intersect(box(-200, x_front, -200, 200, REAR_BAR_Z0_MM - 0.5, zt))
    nsol = len(fit.val().Solids())
    rule_fit = nsol == 1
    export(on_bed(fit), "attachment_v3_fit_test.stl")
    fb = fit.val().BoundingBox()
    print(("ok   " if rule_fit else "FAIL ") + "fit test: %.0f x %.0f x %.0f mm slice (the rear bar, switch pockets, wire "
          "and sled screw holes, the arms up to the low screw holes): bottom down, no support" % (fb.xlen, fb.ylen, fb.zlen))
    ok = ok and rule_fit
    pg, sizes = pogo_gauge_v_solid()
    export(on_bed(pg), "pogo_gauge_v.stl")
    print("ok   upright pin gauge: holes %s mm, smallest at the notched end; seat %.1f mm up, so a pressed-in pin's tip "
          "should stand %.1f mm above the floor" % (" / ".join("%.1f" % s for s in sizes), PIN_SEAT_Z_MM, PIN_TIP_FREE_Z_MM))
    B.preview("attachment_v3", att, os.path.join(B.PNG, "attachment_v3.png"), plan_z=REAR_BAR_Z0_MM + 3)
    B.preview("pad_sled", sled, os.path.join(B.PNG, "pad_sled.png"), plan_z=Z_PAD_MM + 0.2)
    B.preview("dock_v4", dock, os.path.join(B.PNG, "dock_v4.png"), 2.0)
    docked_png(att, sled, dock, os.path.join(B.PNG, "docked_v4.png"))
    underside_png(att, sled, os.path.join(B.PNG, "underside_v4.png"))
    print("\nwrote %s/{attachment_v3, pad_sled, dock_v4, clearance_gauge, attachment_v3_fit_test, pogo_gauge_v}.stl" % B.STL)
    print("      %s/{attachment_v3, pad_sled, dock_v4, docked_v4, underside_v4}.png" % B.PNG)
    print("\nfirmware and map numbers for the back bumper:")
    for l in firmware_lines():
        print("  " + l)
    print("  docked pins at dock X %.1f mm, Y +-%.1f; prong tips at X %.1f" % (X_PIN_MM, PAD_Y_MM, X_TIP_MM))
    if not ok:
        print("\nSOME CHECKS FAILED")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
