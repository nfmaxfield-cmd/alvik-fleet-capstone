"""Dock v5: a U the robot backs into, with hinged, sprung RAMP contacts; pad sled v5; attachment v5.

    python hardware/dock/build_dock_v5.py

writes
    stl/dock_v5.stl          the U: a base (bottom of the U) with two switch bumps on its inner face, the marker strip
                             recess on top, the wiring cavity and cable channel; two arms (short mouth flare, a groove
                             along their inner faces for the screw heads); post stations on the three outer faces; two
                             prongs on the floor, each carrying one contact: a stiff
                             brass plate hinged at its front, held up at its rear by a small spring
    stl/pad_sled_v5.stl      the pad sled: pads moved forward to 8 mm behind the axle (less lift on the caster), wider
                             slots so the plate's crown passes under the robot without touching it
    stl/attachment_v5.stl    attachment v3 with the two pad-wire holes moved out to 33 mm (they clear the wider slots)
    stl/marker_strip_v5.stl  the camera marker: 2 x 6 squares of 15 mm (90 x 30 mm), for the recess on the base
    preview/dock_v5.png, docked_v5.png (sections), contact_v5.png (the contact close up), lift_v5.png (lift on the caster
    while the robot backs in: v3 pins against v5 plates), sled_v5.png
and prints a check report (ok / WARN / FAIL) and the numbers for the firmware and the map.

Nick's decisions, 2026-10-25 10:39: the dock's contacts become a RAMP (raised at the back of each prong,
sloping down towards the front: the robot's pad rides onto it and pushes it down, so the clearance need not be
precise); a much shorter funnel; a U shape with bumps on the 3 outer edges and on the inner edge at the bottom of the
U; the marker a skinny rectangle along the bottom of the U. Plan: docs/build-plans/self-charging-dock.md.
Earlier versions stay as they were: build_attachment_v3.py (attachment v3, pad sled, dock v4).

Frames (mm), as build_attachment_v3.py:
  robot: X forward from the wheel axle, Y to the robot's left, Z up from the floor
  dock:  X into the dock from the base's inner face (the bottom of the U), Y left, Z up from the plywood.
         A docked robot faces OUT of the dock: robot (x, y) -> dock (X_AXLE_DOCKED - x, -y).
Every number is SOURCED / MEASURED / ASSUMED (as dock_params.py) or CHOSEN; each CHOSEN one has a DOE note in the plan
and a row in docs/doe-backlog.md.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dock_params as D              # noqa: E402
import build as B                    # noqa: E402
import build_light_plate as L        # noqa: E402
import build_v2 as V2                # noqa: E402
import build_attachment_v3 as V3     # noqa: E402
from build_attachment_v3 import box, patched   # noqa: E402

# =============================================================================== the robot (as v3; SOURCED [STEP])
Z_UNDER_MM = V3.Z_UNDER_MM            # 10.1: flat underside. MEASURE: the paper-stack check, step 1
CASTER_X_MM = V3.CASTER_X_MM          # -41.0
CASTER_LOAD_N = V3.CASTER_LOAD_N      # 0.6 ASSUMED: weight on the caster. MEASURE: check list item 4 (kitchen scale)
WHEEL_INNER_Y_MM = V3.WHEEL_INNER_Y_MM
BACK_FACE_X_MM = V3.BACK_FACE_X_MM    # -47.6
BAR_FACE_X_MM = BACK_FACE_X_MM - D.FACE_X_MM   # -61.0: the rear bar's face

# =============================================================================== attachment v5 (= v3, wire holes moved)
WIRE_HOLE_Y_MM = 33.0                 # CHOSEN: out of the wider slots' flared mouths (v3: 29)

# =============================================================================== pad sled v5 (robot side)
SLED_T_MM = 2.4                       # as v3
PAD_X_MM = -8.0                       # CHOSEN: pad centre 8 mm behind the axle (v3: 24). The contact pushes up here:
                                      # lift on the caster = force x 8 / 41. DOE: pad position x spring rate (plan)
PAD_Y_MM = 22.0                       # as v3: between the caster holder (9.5) and the wheels (41.2)
PAD_L_MM = 12.0                       # along X: the crown wipes 6 mm onto it and stops at its middle
PAD_W_MM = 10.0                       # along Y: the plate (6.35) lands within the pocket's 0.5 mm play with 1.3 to spare
PAD_T_MM = 0.4
TAB_SLOT_MM = (1.6, 4.0)
RAMP_L_MM = 3.0                       # the slot's front end eases down to the pad (a chamfer: the plate is the ramp)
SLOT_HALF_W_MM = 5.2                  # CHOSEN: plate half width 3.2 + the U's play in the arms 1.5 + 0.5
SLOT_FLARE_HALF_W_MM = 8.0            # the slot's mouth at the sled's back end
SLOT_FLARE_L_MM = 8.0
SLED_HALF_W_MM = 36.0
CASTER_NOTCH_HALF_W_MM = 12.0
CASTER_NOTCH_FRONT_X_MM = -33.0
CH_W_MM, CH_D_MM = 2.0, 1.2
SLED_X0_MM = BAR_FACE_X_MM + 3.0      # -58: back end, 3 mm short of the bar's face
SLED_X1_MM = PAD_X_MM + PAD_L_MM / 2 + 3.0     # +1: front end (just past the axle line)
Z_PAD_MM = Z_UNDER_MM - SLED_T_MM     # 7.7: the pads' face
PAD_X0_MM = PAD_X_MM - PAD_L_MM / 2   # -14: the pad's back edge
RAMP_X0_MM = PAD_X0_MM - RAMP_L_MM    # -17: where the slot ends

# =============================================================================== the contact: a hinged plate on a spring
# Nick, 2026-10-25 11:05: "a stiff strip that has springs under the rear, no bending". Each prong carries a STIFF
# brass plate. Its front end (towards the mouth) goes down through a slot across the prong (the hinge) and is bent
# flat under the prong (the foot), so it cannot lift out and nothing stands above the plate. A small compression spring in a pocket under its rear holds the rear
# up, so the plate is a ramp, highest at the rear (towards the bottom of the U). The robot backs in: its pad rides up
# the ramp and pushes the rear DOWN against the spring. The rear end is bent down in a short lip, so the contact is a
# rounded edge. Rigid plate: the force at the crown is k_spring x (a_spring / a_crown)^2 x travel.
PLATE_T_MM = 0.8                      # CHOSEN: 1/32 in (0.032 in) brass strip, 1/4 in wide (hobby metal racks): stiff
PLATE_W_MM = 6.35                     # 1/4 in, as sold
PLATE_E_MPA = 100000.0                # SOURCED (handbook): brass, about 100 GPa
PLATE_A_MM = 22.0                     # CHOSEN: hinge to crown, along X. DOE: with the spring (plan)
CROWN_FREE_Z_MM = 9.0                 # CHOSEN: crown top, not pressed: 1.1 under the robot's underside (it passes
                                      # free in the slots), 1.3 over the pad (the press). Set by the spring's length;
                                      # the paper-stack check sets it to the real robot
TAB_DOWN_MM = 4.0                     # the front end bent down 90 deg through the prong: it rocks in the hinge slot
FOOT_MM = 3.0                         # ... then bent flat towards the mouth under the prong (pliers, after fitting)
FOOT_RECESS_Z_MM = 1.8                # the foot's recess under the prong (open to the plywood)
HINGE_SLOT_X_MM = 1.4                 # slot across the prong for the 0.8 mm tab (clearance for the rocking)
CURL_MM, CURL_DEG = 2.5, 60.0         # the rear lip, bent down (a rounded crown; no edge for the pad to catch)
SPRING_K_N_MM = 0.20                  # CHOSEN target, ASSUMED until bought: 0.2 N/mm (20 g per mm). A 3 mm micro
                                      # compression spring of about 0.2 mm wire, 6 active coils: k = G d^4/(8 D^3 n)
                                      # = 0.12-0.3. Buy an assortment and pick by the finger test (plan)
SPRING_OD_MM = 3.0                    # ASSUMED
SPRING_SOLID_MM = 2.0                 # ASSUMED: (coils + 2) x wire
SPRING_BACK_MM = 2.0                  # spring centre this far in front of the crown
SPRING_FLOOR_Z_MM = 1.9               # spring pocket's floor above the plywood
LIP_FLOOR_Z_MM = 2.4                  # the lip's pocket floor: the stop (the plate can go no lower)
LEAD = "30 AWG silicone-insulated stranded wire"   # the flexible lead, soldered to the foot (it barely moves)

# =============================================================================== the dock (U)
BUMP_OUT_MM = V3.BUMP_OUT_MM          # 4.0: the two switch bumps on the base's inner face (as v4)
BUMP_Y_MM = (30.0, D.POCKET_W_MM / 2)
WALL_H_MM = D.WALL_H_MM               # 22: base and arms (the rear bar is 10.1-20.1 up, levers about 14.6)
POCKET_HALF_W_MM = D.POCKET_W_MM / 2  # 51.6: the last POCKET_DEPTH holds the bar to +-0.5 mm (as v4)
POCKET_DEPTH_MM = 14.0                # the bar (13.4 deep) seats in it
ARM_IN_HALF_W_MM = POCKET_HALF_W_MM + 1.0   # CHOSEN: 52.6: in the arms the bar has +-1.5 mm; the light plate's front
                                      # legs (51.65 out) clear by 0.95
ARM_T_MM = 10.0                       # CHOSEN: arm thickness (room for a countersunk #6 screw beside the groove)
ARM_END_X_MM = -105.0                 # CHOSEN: the mouth. DOE: arm length x flare (plan)
FLARE_L_MM = 10.0                     # CHOSEN: the funnel is now just this chamfer at the mouth ...
FLARE_MM = 7.0                        # ... opening 7 mm each side (35 deg). Catches a robot up to 8.5 mm off centre
GROOVE_Z_MM = (10.5, 19.0)            # groove along the arms' inner faces for the M3 heads (low inserts at 14.7 up)
GROOVE_HALF_W_MM = 55.0               # heads stand out to 53.2 (bracket arms) and 53.75 (plate legs)
GROOVE_X0_MM = -20.0                  # from just past the pocket to the mouth
BASE_D_MM = 36.0                      # CHOSEN: base depth: the 30 mm marker strip plus rims
HALF_OUT_MM = ARM_IN_HALF_W_MM + ARM_T_MM    # 62.6: the outer faces
PRONG_HALF_W_MM = 6.0                 # prongs 12 mm wide: the plate (6.35), the spring pocket and the wire groove
GROOVE_OFF_MM = 3.75                  # the wire groove runs this far outboard of the prong's centre (beside the pockets)
PRONG_T_MM = 4.0
GROOVE_W_MM, GROOVE_D_MM = 3.0, 1.4   # wire groove under each prong (open to the plywood)
TAB_W_MM = 10.0                       # screw tab beside each hinge, on the prong's outer side (#4 x 1/2 in wood
                                      # screw, countersunk): it pins the prong's far end to the plywood
TAB_HOLE_DIA_MM, TAB_CSK_DIA_MM = 3.0, 6.0
CAVITY_X_MM = (6.0, 30.0)
CAVITY_HALF_Y_MM = 27.0
CAVITY_TOP_Z_MM = 15.0
CHANNEL_HALF_W_MM, CHANNEL_H_MM = 3.0, 5.0
SCREW_HOLES = [(18.0, 54.0), (18.0, -54.0), (-88.0, 58.3), (-88.0, -58.3)]   # #6 x 1 1/4 in, countersunk from the top
POST_H_MM = WALL_H_MM                 # posts as tall as the walls (the rear bar is 10.1-20.1 up)
MARK_DIA_MM = 1.6
# the marker strip (camera): what it is for and why this size: the plan, "Marker"
STRIP_CELL_MM = 15.0                  # CHOSEN: 15 mm squares are 4.6 px at the camera's 3.1 px/cm (262 cm up, 640 x
                                      # 480; floor_calibration.json). DOE: the bump test of E00e Part C at 10 / 15 mm
STRIP_N = (6, 2)                      # 6 across the dock, 2 along it
STRIP_T_MM, STRIP_RAISE_MM, STRIP_GAP_MM = 1.2, 0.6, 0.3
STRIP_CLEAR_MM = 0.4
STRIP_X0_MM = 3.0                     # recess from 3 mm behind the inner face: the docked robot's light plate (59-63 mm
                                      # up, its back edge at dock X -14) never hides it from the camera (checked)

# derived
X_AXLE_DOCKED_MM = -(BUMP_OUT_MM + D.FACE_X_MM + D.AXLE_TO_FRONT_MM)    # -65.0
X_CROWN_MM = X_AXLE_DOCKED_MM - PAD_X_MM                                # -57.0
PLATE_RISE_MM = CROWN_FREE_Z_MM - PRONG_T_MM - PLATE_T_MM               # the plate lies on the prong at the hinge
PLATE_THETA = math.asin(PLATE_RISE_MM / PLATE_A_MM)
X_HINGE_MM = X_CROWN_MM - PLATE_A_MM * math.cos(PLATE_THETA)            # the tab's back face
X_SPRING_MM = X_CROWN_MM - SPRING_BACK_MM
A_RATIO = (X_SPRING_MM - X_HINGE_MM) / (X_CROWN_MM - X_HINGE_MM)       # spring arm / crown arm
SPRING_FREE_MM = PRONG_T_MM + (X_SPRING_MM - X_HINGE_MM) * math.tan(PLATE_THETA) - SPRING_FLOOR_Z_MM
_LIP_TIP_FREE = CROWN_FREE_Z_MM - PLATE_T_MM - CURL_MM * math.sin(math.radians(CURL_DEG))
DELTA_MAX_MM = min(PLATE_RISE_MM, _LIP_TIP_FREE - LIP_FLOOR_Z_MM)       # crown travel to the stop
X_PRONG_TIP_MM = X_HINGE_MM - PLATE_T_MM - FOOT_MM - 3.0
TAB_Y_MM = PAD_Y_MM + PRONG_HALF_W_MM + TAB_W_MM / 2 - 0.5               # the screw tab's hole, out from the axis
X_TAB_MM = X_HINGE_MM - 1.0
STRIP_LEN_MM = STRIP_N[0] * STRIP_CELL_MM       # 90, across the dock (Y)
STRIP_DEP_MM = STRIP_N[1] * STRIP_CELL_MM       # 30, along the dock (X)
STRIP_CX_MM = STRIP_X0_MM + STRIP_CLEAR_MM + STRIP_DEP_MM / 2


def contact_k():
    """Vertical stiffness at the crown, N/mm: the spring seen through the plate's lever."""
    return SPRING_K_N_MM * A_RATIO ** 2


def plate_bend_mm(F):
    """How much the plate itself bends under F at the crown (it overhangs the spring by SPRING_BACK + the lip)."""
    I = PLATE_W_MM * PLATE_T_MM ** 3 / 12.0
    L = SPRING_BACK_MM + 1.0
    return F * L ** 3 / (3 * PLATE_E_MPA * I)


def lift_frac(force_total_n, x_mm, caster_load=None):
    """Share of the caster's load taken off by an upward force at robot x (behind the axle: x < 0)."""
    return force_total_n * max(0.0, -x_mm) / abs(CASTER_X_MM) / (caster_load or CASTER_LOAD_N)


def robot_to_dock(x, y):
    return X_AXLE_DOCKED_MM - x, -y


# =============================================================================== robot side
def attachment_v5_solid():
    with patched(V3, WIRE_HOLE_Y_MM=WIRE_HOLE_Y_MM):
        return V3.attachment_v3_solid()


def sled_v5_solid():
    """Robot frame. Top face against the robot's underside and the bar's underside (both at Z_UNDER_MM)."""
    c = B.cq()
    z0, z1 = Z_PAD_MM, Z_UNDER_MM
    body = box(SLED_X0_MM, SLED_X1_MM, -SLED_HALF_W_MM, SLED_HALF_W_MM, z0, z1)
    body = body.edges("|Z").fillet(2.0)
    body = body.cut(box(BACK_FACE_X_MM - 0.4, CASTER_NOTCH_FRONT_X_MM, -CASTER_NOTCH_HALF_W_MM, CASTER_NOTCH_HALF_W_MM,
                        z0 - 1, z1 + 1))
    for sg in (1, -1):
        y = sg * PAD_Y_MM
        body = body.cut(box(SLED_X0_MM - 1, RAMP_X0_MM, y - SLOT_HALF_W_MM, y + SLOT_HALF_W_MM, z0 - 1, z1 + 1))
        fl = (c.Workplane("XY").workplane(offset=z0 - 1)
              .polyline([(SLED_X0_MM - 1, y - SLOT_FLARE_HALF_W_MM), (SLED_X0_MM + SLOT_FLARE_L_MM, y - SLOT_HALF_W_MM),
                         (SLED_X0_MM + SLOT_FLARE_L_MM, y + SLOT_HALF_W_MM), (SLED_X0_MM - 1, y + SLOT_FLARE_HALF_W_MM)])
              .close().extrude(SLED_T_MM + 2))
        body = body.cut(fl)
        ramp = (c.Workplane("XZ").polyline([(RAMP_X0_MM - 0.01, z0 - 1), (RAMP_X0_MM - 0.01, z1 + 0.5),
                                             (PAD_X0_MM, z0), (PAD_X0_MM, z0 - 1)])
                .close().extrude(-(2 * SLOT_HALF_W_MM)).translate((0, y - SLOT_HALF_W_MM, 0)))
        body = body.cut(ramp)
        body = body.cut(box(PAD_X0_MM, PAD_X_MM + PAD_L_MM / 2, y - PAD_W_MM / 2, y + PAD_W_MM / 2, z0 - 1, z0 + PAD_T_MM))
        xt = PAD_X_MM + PAD_L_MM / 2 - TAB_SLOT_MM[0] / 2 - 0.5
        body = body.cut(box(xt - TAB_SLOT_MM[0] / 2, xt + TAB_SLOT_MM[0] / 2, y - TAB_SLOT_MM[1] / 2,
                            y + TAB_SLOT_MM[1] / 2, z0 - 1, z1 + 1))
        yh = sg * WIRE_HOLE_Y_MM
        body = body.cut(box(xt - CH_W_MM / 2, xt + CH_W_MM / 2, min(y, yh) - CH_W_MM / 2, max(y, yh) + CH_W_MM / 2,
                            z1 - CH_D_MM, z1 + 1))
        body = body.cut(box(V3.WIRE_HOLE_X_MM - V3.WIRE_HOLE_DIA_MM / 2, xt + CH_W_MM / 2, yh - CH_W_MM / 2,
                            yh + CH_W_MM / 2, z1 - CH_D_MM, z1 + 1))
        p = (V3.SLED_SCREW_X_MM, sg * V3.SLED_SCREW_Y_MM)
        body = body.cut(B.cyl(V3.M2_CLEAR_MM / 2, SLED_T_MM + 2, (p[0], p[1], z0 - 1), (0, 0, 1)))
        body = body.cut(B.cyl(V3.M2_HEAD_DIA_MM / 2, V3.M2_HEAD_H_MM + 1, (p[0], p[1], z0 - 1), (0, 0, 1)))
    ch = (c.Workplane("XZ").polyline([(SLED_X0_MM - 1, z0 - 1), (SLED_X0_MM - 1, z0 + 1.5), (SLED_X0_MM + 2.5, z0 - 0.01),
                                      (SLED_X0_MM + 2.5, z0 - 1)]).close().extrude(-100).translate((0, -50, 0)))
    return body.cut(ch)


# =============================================================================== the plate (drawn; bought strip)
def contact_profile(delta=0.0):
    """The plate's TOP surface in the dock's XZ plane, [(X, Z)], with the crown pressed delta mm (a rigid rotation
    about the hinge): hinge, 24 points to the crown, then the lip's tip."""
    zc = CROWN_FREE_Z_MM - delta
    z0 = PRONG_T_MM + PLATE_T_MM
    pts = [(X_HINGE_MM + i / 24.0 * (X_CROWN_MM - X_HINGE_MM), z0 + i / 24.0 * (zc - z0)) for i in range(25)]
    tilt = math.atan2(zc - z0, X_CROWN_MM - X_HINGE_MM)
    a = math.radians(CURL_DEG) - (PLATE_THETA - tilt)
    pts.append((X_CROWN_MM + CURL_MM * math.cos(a), zc - CURL_MM * math.sin(a)))
    return pts


def plate_polys(delta=0.0):
    """The plate for drawing, in the dock's XZ plane: [body with lip, hinge tab] as point lists."""
    top = contact_profile(delta)
    body = top + [(x, z - PLATE_T_MM) for (x, z) in top[::-1]]
    z0 = PRONG_T_MM + PLATE_T_MM
    zb = PRONG_T_MM - TAB_DOWN_MM + 0.2
    tab = [(X_HINGE_MM - PLATE_T_MM, z0), (X_HINGE_MM, z0), (X_HINGE_MM, zb), (X_HINGE_MM - PLATE_T_MM - FOOT_MM, zb),
           (X_HINGE_MM - PLATE_T_MM - FOOT_MM, zb + PLATE_T_MM), (X_HINGE_MM - PLATE_T_MM, zb + PLATE_T_MM)]
    return [body, tab]


def spring_line(delta=0.0, coils=6):
    """A zigzag from the pocket's floor to the plate's underside, for drawing."""
    top = PRONG_T_MM + (X_SPRING_MM - X_HINGE_MM) * math.tan(PLATE_THETA) - delta * A_RATIO
    r = SPRING_OD_MM / 2
    n = 2 * coils
    return [(X_SPRING_MM + (r if i % 2 else -r) * (0 < i < n), SPRING_FLOOR_Z_MM + (top - SPRING_FLOOR_Z_MM) * i / n)
            for i in range(n + 1)]


# =============================================================================== the dock
def inner_half_w(x):
    """The U's inner half width at dock X (mm), at heights outside the screw-head groove."""
    if x >= -POCKET_DEPTH_MM:
        return POCKET_HALF_W_MM
    if x >= -POCKET_DEPTH_MM - 1.0:
        return POCKET_HALF_W_MM + (-POCKET_DEPTH_MM - x) * (ARM_IN_HALF_W_MM - POCKET_HALF_W_MM)
    if x >= ARM_END_X_MM + FLARE_L_MM:
        return ARM_IN_HALF_W_MM
    return ARM_IN_HALF_W_MM + FLARE_MM * (ARM_END_X_MM + FLARE_L_MM - x) / FLARE_L_MM


def post(base_xy, out_dir, along_dir):
    with patched(V2, POST_H_MM=POST_H_MM):
        return V2.post(base_xy, out_dir, along_dir)


def side_post_xc():
    return (BASE_D_MM + ARM_END_X_MM) / 2


def dock_v5_solid():
    c = B.cq()
    H = WALL_H_MM
    body = box(0, BASE_D_MM, -HALF_OUT_MM, HALF_OUT_MM, 0, H)
    pkt, arm = POCKET_HALF_W_MM, ARM_IN_HALF_W_MM
    xf = ARM_END_X_MM + FLARE_L_MM
    for sg in (1, -1):
        pts = [(0.5, pkt), (-POCKET_DEPTH_MM, pkt), (-POCKET_DEPTH_MM - 1.0, arm), (xf, arm),
               (ARM_END_X_MM, arm + FLARE_MM), (ARM_END_X_MM, HALF_OUT_MM), (0.5, HALF_OUT_MM)]
        a = c.Workplane("XY").polyline([(x, sg * y) for x, y in pts]).close().extrude(H)
        a = a.cut(box(ARM_END_X_MM - 1, GROOVE_X0_MM, *sorted((sg * 40.0, sg * GROOVE_HALF_W_MM)), *GROOVE_Z_MM))
        body = body.union(a)
        # the screw-head groove eases in at its inner end (no square shoulder for a head to catch on the way out)
        e = (c.Workplane("XY").workplane(offset=GROOVE_Z_MM[0])
             .polyline([(GROOVE_X0_MM - 0.01, sg * arm), (GROOVE_X0_MM - 0.01, sg * GROOVE_HALF_W_MM),
                        (GROOVE_X0_MM + 3.0, sg * arm)]).close().extrude(GROOVE_Z_MM[1] - GROOVE_Z_MM[0]))
        body = body.cut(e)
        # switch bumps on the base's inner face
        ya, yb = sorted((sg * BUMP_Y_MM[0], sg * (BUMP_Y_MM[1] + 0.5)))
        body = body.union(box(-BUMP_OUT_MM, 1.0, ya, yb, 0, H))
    # prongs: hinge slot (through), the foot's recess, spring pocket, lip pocket, wire groove, screw tab
    for sg in (1, -1):
        y = sg * PAD_Y_MM
        hw = PRONG_HALF_W_MM
        prong = box(X_PRONG_TIP_MM, 1.0, y - hw, y + hw, 0, PRONG_T_MM)
        prong = prong.faces("<X").edges(">Z").chamfer(1.5)
        body = body.union(prong)
        pw = PLATE_W_MM / 2 + 0.3
        body = body.cut(box(X_HINGE_MM - PLATE_T_MM - (HINGE_SLOT_X_MM - PLATE_T_MM) / 2,
                            X_HINGE_MM + (HINGE_SLOT_X_MM - PLATE_T_MM) / 2, y - pw, y + pw, -1, PRONG_T_MM + 1))
        # the foot's recess under the prong, towards the mouth
        body = body.cut(box(X_HINGE_MM - PLATE_T_MM - FOOT_MM - 0.6, X_HINGE_MM, y - pw, y + pw, -1, FOOT_RECESS_Z_MM))
        # spring pocket and the lip's pocket (its floor is the stop)
        body = body.cut(B.cyl(SPRING_OD_MM / 2 + 0.25, PRONG_T_MM, (X_SPRING_MM, y, SPRING_FLOOR_Z_MM), (0, 0, 1)))
        body = body.cut(box(X_CROWN_MM - 0.5, X_CROWN_MM + CURL_MM + 1.5, y - pw, y + pw, LIP_FLOOR_Z_MM,
                            PRONG_T_MM + 1))
        # wire groove from the hinge slot to the cavity, outboard of the pockets
        yo = y + sg * GROOVE_OFF_MM
        body = body.cut(box(X_HINGE_MM - HINGE_SLOT_X_MM, CAVITY_X_MM[0] + 1, yo - GROOVE_W_MM / 2, yo + GROOVE_W_MM / 2,
                            -1, GROOVE_D_MM))
        # screw tab beside the hinge, outboard
        xt, yt = X_TAB_MM, sg * TAB_Y_MM
        body = body.union(box(xt - TAB_W_MM / 2, xt + TAB_W_MM / 2, *sorted((sg * (PAD_Y_MM + hw - 0.5),
                                                                               sg * (TAB_Y_MM + TAB_W_MM / 2))), 0, PRONG_T_MM))
        body = body.cut(B.cyl(TAB_HOLE_DIA_MM / 2, PRONG_T_MM + 2, (xt, yt, -1), (0, 0, 1)))
        body = body.cut(c.Workplane("XY").workplane(offset=PRONG_T_MM - TAB_CSK_DIA_MM / 2).center(xt, yt)
                        .circle(0.01).workplane(offset=TAB_CSK_DIA_MM / 2 + 0.01).circle(TAB_CSK_DIA_MM / 2 + 0.01).loft())
    # cavity, cable channel, marker strip recess and finger notch
    body = body.cut(box(CAVITY_X_MM[0], CAVITY_X_MM[1], -CAVITY_HALF_Y_MM, CAVITY_HALF_Y_MM, -1, CAVITY_TOP_Z_MM))
    body = body.cut(box(CAVITY_X_MM[1] - 1, BASE_D_MM + 2, -CHANNEL_HALF_W_MM, CHANNEL_HALF_W_MM, -1, CHANNEL_H_MM))
    rx, ry = STRIP_DEP_MM + 2 * STRIP_CLEAR_MM, STRIP_LEN_MM + 2 * STRIP_CLEAR_MM
    body = body.cut(box(STRIP_X0_MM, STRIP_X0_MM + rx, -ry / 2, ry / 2, H - STRIP_T_MM + 0.2, H + 1))
    body = body.cut(c.Workplane("XY").workplane(offset=H - STRIP_T_MM).center(STRIP_CX_MM, ry / 2).circle(4).extrude(5))
    for (x, y) in SCREW_HOLES:
        body = body.cut(c.Workplane("XY").center(x, y).circle(V2.SCREW_HOLE_DIA_MM / 2).extrude(H + 1))
        body = body.cut(c.Workplane("XY").workplane(offset=H - V2.SCREW_CSK_DIA_MM / 2).center(x, y)
                        .circle(0.01).workplane(offset=V2.SCREW_CSK_DIA_MM / 2 + 0.01)
                        .circle(V2.SCREW_CSK_DIA_MM / 2 + 0.01).loft())
    # post stations: back face, and the two arms' outer faces
    s = D.SWITCH_LATERAL_MM
    for y in (s, -s):
        body = body.union(post((BASE_D_MM, y), (1, 0), (0, 1)))
    xc = side_post_xc()
    for sg in (1, -1):
        for x in (xc + s, xc - s):
            body = body.union(post((x, sg * HALF_OUT_MM), (0, sg), (1, 0)))
    # + mark on top of the base, above the + plate (the robot's left pad: dock Y negative)
    body = body.cut(c.Workplane("XY").workplane(offset=H - 0.8).center(BASE_D_MM - 1.6, -PAD_Y_MM)
                    .circle(MARK_DIA_MM / 2).extrude(2))
    return body


def marker_strip_solid():
    c = B.cq()
    nx, ny = STRIP_N
    cell = STRIP_CELL_MM
    body = box(0, ny * cell, 0, nx * cell, 0, STRIP_T_MM)
    g = STRIP_GAP_MM / 2
    for i in range(ny):
        for j in range(nx):
            if (i + j) % 2 == 0:
                body = body.union(box(i * cell + g, (i + 1) * cell - g, j * cell + g, (j + 1) * cell - g,
                                      STRIP_T_MM - 0.01, STRIP_T_MM + STRIP_RAISE_MM))
    return body


# =============================================================================== the entry: lift on the caster
def underside_v5(x, off=0.0, dz=0.0):
    """Robot underside height (mm) at robot x along the plate's line (robot y = PAD_Y + off): the sled's slot roof
    (the robot itself) or the sled/pad face. dz shifts it (a real robot sitting lower: dz < 0)."""
    y_rel = abs(off)
    in_slot = y_rel + PLATE_W_MM / 2 <= SLOT_HALF_W_MM or (x < SLED_X0_MM + SLOT_FLARE_L_MM and
                                                         y_rel + PLATE_W_MM / 2 <= SLOT_FLARE_HALF_W_MM)
    if x < BAR_FACE_X_MM:
        return None
    if x < SLED_X0_MM:
        return Z_UNDER_MM + dz
    if x <= RAMP_X0_MM and in_slot:
        return Z_UNDER_MM + dz
    if x < PAD_X0_MM and in_slot:
        return Z_UNDER_MM + dz - (x - RAMP_X0_MM) / RAMP_L_MM * SLED_T_MM
    if x <= SLED_X1_MM:
        return Z_PAD_MM + dz
    return Z_UNDER_MM + dz


def lift_curve_v5(s_list, off=0.0, dz=0.0):
    """[(s, lift share, force N per plate, contact robot x)]: s = mm still to go before the bumps. The plate turns
    about its hinge; the robot's surface that must push it down furthest is the contact."""
    k = contact_k()
    out = []
    sh = X_CROWN_MM - X_HINGE_MM
    for s in s_list:
        best = (0.0, None)
        for (X, zl) in contact_profile(0.0)[1:]:
            x = X_AXLE_DOCKED_MM - s - X
            zr = underside_v5(x, off, dz)
            if zr is None or X <= X_HINGE_MM:
                continue
            need = (zl - zr) * sh / (X - X_HINGE_MM)
            if need > best[0]:
                best = (need, (X, x))
        if best[1] is None:
            out.append((s, 0.0, 0.0, None))
            continue
        dc = min(best[0], DELTA_MAX_MM)
        X, x = best[1]
        F = k * dc * sh / (X - X_HINGE_MM)
        out.append((s, lift_frac(2 * F, x), F, x))
    return out


def lift_curve_v3(s_list):
    """v3: an upright pin 24 mm behind the axle, 0.3 N at 1.0 mm (ASSUMED, linear), under v3's sled profile."""
    out = []
    kp = V3.POGO_V_FORCE_N / V3.COMPRESS_MM
    for s in s_list:
        x = V3.PAD_X_MM - s          # the robot point over the pin (the robot is s mm short of docked)
        if x < V3.SLED_X0_MM:
            zr = V3.Z_UNDER_MM if x >= BAR_FACE_X_MM else None
        elif x <= V3.RAMP_X0_MM:
            zr = V3.Z_UNDER_MM
        elif x < V3.RAMP_X0_MM + V3.RAMP_L_MM:
            zr = V3.Z_UNDER_MM - (x - V3.RAMP_X0_MM) / V3.RAMP_L_MM * V3.SLED_T_MM
        else:
            zr = V3.Z_PAD_MM
        c = 0.0 if zr is None else max(0.0, V3.PIN_TIP_FREE_Z_MM - zr)
        F = kp * c
        out.append((s, lift_frac(2 * F, x), F, x))
    return out


# =============================================================================== known lengths
def known_lengths():
    """The dock is the system's one known length (Nick, 2026-10-25 11:05): what the software may take as exact, with
    who sees it. [(name, printed mm, who can measure it)]. A print can come out 0.1-0.3 % off (PLA shrink, the
    printer's steps): measure the printed part once with calipers or a steel rule and use the measured value."""
    return [("marker strip, long side (6 squares of %.1f)" % STRIP_CELL_MM, STRIP_LEN_MM,
             "camera: the strip's two outer edges / its 7 square edges along it"),
            ("marker strip, short side (2 squares)", STRIP_DEP_MM, "camera"),
            ("marker square pitch", STRIP_CELL_MM, "camera"),
            ("U inner width at the bottom (the pocket)", 2 * POCKET_HALF_W_MM, "the robot: the bar seats to +-0.5"),
            ("post tips of each station, centre to centre", 2 * D.SWITCH_LATERAL_MM, "the robot's two back switches"),
            ("back station's tip line to the bumps' faces (along the dock)", BASE_D_MM + V2.POST_STANDOFF_MM + BUMP_OUT_MM,
             "the robot: docked, then a back-touch on the back posts"),
            ("side stations: tip line to tip line across the dock", 2 * (HALF_OUT_MM + V2.POST_STANDOFF_MM),
             "the robot: a touch on each side station"),
            ("outer width of the U (arm face to arm face)", 2 * HALF_OUT_MM, "tape / camera")]


# =============================================================================== checks
def checks(att, sled, dock):
    lines, ok = [], True

    def rule(good, text, warn=False):
        nonlocal ok
        if not good and not warn:
            ok = False
        lines.append(("ok   " if good else ("WARN " if warn else "FAIL ")) + text)

    for nm, s in (("attachment_v5", att), ("pad sled v5", sled), ("dock_v5", dock)):
        rule(s.val().isValid() and len(s.val().Solids()) == 1, "%s: one valid solid" % nm)
    b = B.bbox(dock)
    v4 = (172, 174)
    rule(b[3] - b[0] + 16 <= D.BED_MM[0] and b[4] - b[1] + 16 <= D.BED_MM[1],
         "dock_v5: %.0f x %.0f x %.0f mm (dock v4: %d x %d), fits a 200 mm bed with a brim; %.0f %% of v4's footprint"
         % (b[3] - b[0], b[4] - b[1], b[5] - b[2], v4[0], v4[1], 100 * (b[3] - b[0]) * (b[4] - b[1]) / (v4[0] * v4[1])))
    # ----- the contact: hinged plate on a spring
    k = contact_k()
    d_nom = CROWN_FREE_Z_MM - Z_PAD_MM
    F = k * d_nom
    rule(True, "contact: brass plate %.1f x %.2f mm, hinge to crown %.0f mm, rising %.1f mm (%.0f deg); spring %.2f N/mm "
         "%.0f mm in front of the crown: %.3f N/mm at the crown" % (PLATE_T_MM, PLATE_W_MM, PLATE_A_MM, PLATE_RISE_MM,
                                                                   math.degrees(PLATE_THETA), SPRING_K_N_MM,
                                                                   SPRING_BACK_MM, k))
    rule(0.5 <= d_nom <= 2.5, "docked, each crown is pressed %.1f mm: %.2f N (%.0f g) per plate" % (d_nom, F, F / 9.81e-3))
    lf = lift_frac(2 * F, PAD_X_MM)
    rule(lf <= 0.25, "the two plates push up %.2f N, %.0f mm behind the axle: %.3f N off the caster, %.0f %% of its "
         "ASSUMED %.1f N (v3's pins: 59 %%; keep under 25 %%)" % (2 * F, -PAD_X_MM, lf * CASTER_LOAD_N, 100 * lf,
                                                                  CASTER_LOAD_N))
    for dz in (-0.5, -1.0):
        d = d_nom - dz
        rule(lift_frac(2 * k * d, PAD_X_MM) <= 0.35 and Z_UNDER_MM + dz - CROWN_FREE_Z_MM >= 0.0,
             "robot %.1f mm lower than the CAD: pressed %.1f mm, %.2f N per plate, %.0f %% off the caster; crown %.1f mm "
             "under the slot roof" % (-dz, d, k * d, 100 * lift_frac(2 * k * d, PAD_X_MM), Z_UNDER_MM + dz - CROWN_FREE_Z_MM),
             warn=dz < -0.6)
    hi = 0.9
    rule(d_nom - hi >= 0.3, "robot %.1f mm higher than the CAD: still pressed %.1f mm (%.2f N per plate)"
         % (hi, d_nom - hi, k * (d_nom - hi)))
    rule(Z_UNDER_MM - CROWN_FREE_Z_MM >= 0.8, "a free crown passes %.1f mm under the robot's underside and the bar (slot "
         "roof)" % (Z_UNDER_MM - CROWN_FREE_Z_MM))
    comp_max = DELTA_MAX_MM * A_RATIO
    rule(SPRING_FREE_MM - comp_max >= SPRING_SOLID_MM + 0.3,
         "spring: %.1f mm long free (pocket floor %.1f mm up); the lip's pocket stops the crown after %.1f mm, the spring "
         "then %.1f mm long (solid at about %.1f)" % (SPRING_FREE_MM, SPRING_FLOOR_Z_MM, DELTA_MAX_MM,
                                                     SPRING_FREE_MM - comp_max, SPRING_SOLID_MM))
    Fmax = k * DELTA_MAX_MM
    rule(plate_bend_mm(Fmax) < 0.05, "stiff: at the stop (%.2f N) the plate itself bends %.3f mm" % (Fmax, plate_bend_mm(Fmax)))
    rot = math.atan(DELTA_MAX_MM / (X_CROWN_MM - X_HINGE_MM))
    rule(FOOT_RECESS_Z_MM >= PLATE_T_MM + FOOT_MM * math.sin(rot) + 0.3 and PRONG_T_MM - FOOT_RECESS_Z_MM >= 1.6,
         "hinge: the foot under the prong swings up %.1f mm at the stop, its recess is %.1f deep (%.1f mm of prong above)"
         % (FOOT_MM * math.sin(rot), FOOT_RECESS_Z_MM, PRONG_T_MM - FOOT_RECESS_Z_MM))
    rule(True, "nothing on the prong stands above the plate: the robot's sled (%.1f mm) passes only over the plate"
         % Z_PAD_MM)
    rule(SPRING_FLOOR_Z_MM - GROOVE_D_MM >= 0.4 and GROOVE_OFF_MM - GROOVE_W_MM / 2 >= SPRING_OD_MM / 2 + 0.25 + 0.2 and
         GROOVE_OFF_MM + GROOVE_W_MM / 2 <= PRONG_HALF_W_MM - 0.6 and LIP_FLOOR_Z_MM - GROOVE_D_MM >= 0.8,
         "wire groove runs %.2f mm outboard, beside the spring pocket, under the lip pocket with %.1f mm of floor"
         % (GROOVE_OFF_MM, LIP_FLOOR_Z_MM - GROOVE_D_MM))
    # entry: worst lift while backing in, centred and at the U's full play
    ss = [i * 0.5 for i in range(0, 161)]
    for off, lab in ((0.0, "centred"), (ARM_IN_HALF_W_MM - D.BRACKET_W_MM / 2, "at the arms' full play")):
        cur = lift_curve_v5(ss, off=off)
        mx = max(cur, key=lambda r: r[1])
        first = max((r[0] for r in cur if r[1] > 0.005), default=0.0)
        rule(first <= 16.0 and mx[1] <= 0.25,
             "backing in %s (%.1f mm off): the pads first touch the plates %.1f mm before the bumps (nothing earlier); "
             "the most lift is %.0f %%, as the crown meets the pad's back edge (%.0f mm behind the axle); docked %.0f %%"
             % (lab, off, first, 100 * mx[1], -mx[3], 100 * cur[0][1]))
    off_bad = SLOT_HALF_W_MM - PLATE_W_MM / 2 + 0.6
    cur = lift_curve_v5(ss, off=off_bad)
    rule(False, "for comparison: a robot %.1f mm off centre (more than the arms allow) would drag a crown under the "
         "sled's plastic: up to %.0f %% lift (the lift chart's dashed line)" % (off_bad, 100 * max(r[1] for r in cur)),
         warn=True)
    # ----- sideways geometry
    p_in, p_out = PAD_Y_MM - PRONG_HALF_W_MM, PAD_Y_MM + PRONG_HALF_W_MM
    rule(p_in >= V3.CASTER_HOLDER_HALF_W_MM + 4.0, "prongs %.0f-%.0f mm out: clear of the caster's ball (4.5) and holder "
         "(9.5) by %.1f, and %.1f mm inside the wheels" % (p_in, p_out, p_in - V3.CASTER_HOLDER_HALF_W_MM,
                                                           WHEEL_INNER_Y_MM - p_out))
    play = ARM_IN_HALF_W_MM - D.BRACKET_W_MM / 2
    rule(SLOT_HALF_W_MM - PLATE_W_MM / 2 >= play + 0.4, "in the arms the robot is within +-%.1f mm; the crown has %.1f mm "
         "each side in its slot" % (play, SLOT_HALF_W_MM - PLATE_W_MM / 2))
    rule(PAD_W_MM / 2 - PLATE_W_MM / 2 >= D.POCKET_CLEAR_MM / 2 + 0.5, "docked (pocket +-%.1f): the plate lands on the "
         "pad with %.1f mm to spare each side" % (D.POCKET_CLEAR_MM / 2, PAD_W_MM / 2 - PLATE_W_MM / 2))
    rule(WIRE_HOLE_Y_MM - V3.WIRE_HOLE_DIA_MM / 2 >= PAD_Y_MM + SLOT_FLARE_HALF_W_MM + 1.0 and
         WIRE_HOLE_Y_MM + CH_W_MM / 2 + 1.0 <= SLED_HALF_W_MM,
         "pad wires: holes in the bar at +-%.0f mm and the sled's channels clear the slots' flared mouths (to %.0f)"
         % (WIRE_HOLE_Y_MM, PAD_Y_MM + SLOT_FLARE_HALF_W_MM))
    sw_inner = D.SWITCH_Y_OUTER_MM - D.SWITCH_L_MM - 0.4
    rule(WIRE_HOLE_Y_MM + V3.WIRE_HOLE_DIA_MM / 2 + 1.0 <= sw_inner, "bar wire holes clear of the switch pockets (from "
         "%.1f)" % sw_inner)
    # ----- along: where the caster and the wheels meet the prongs, the plate under the robot
    x_bar_at_tip = X_PRONG_TIP_MM - (CASTER_X_MM - BAR_FACE_X_MM)
    off_tip = (inner_half_w(min(x_bar_at_tip, -POCKET_DEPTH_MM - 1.1)) - D.BRACKET_W_MM / 2
               if x_bar_at_tip >= ARM_END_X_MM else FLARE_MM + play)
    where = ("%.0f mm into the U" % (x_bar_at_tip - ARM_END_X_MM) if x_bar_at_tip >= ARM_END_X_MM
             else "%.0f mm short of the mouth" % (ARM_END_X_MM - x_bar_at_tip))
    rule(off_tip + V3.CASTER_BALL_R_MM <= p_in - 2.0,
         "when the caster reaches the prongs' tips the bar is %s; a robot the mouth can catch (within %.1f mm) misses "
         "them with the ball by %.1f mm or more" % (where, off_tip, p_in - off_tip - V3.CASTER_BALL_R_MM))
    rule(TAB_Y_MM + TAB_W_MM / 2 <= WHEEL_INNER_Y_MM - 3.0, "prong screw tabs reach %.0f mm out (wheels from %.1f)"
         % (TAB_Y_MM + TAB_W_MM / 2, WHEEL_INNER_Y_MM))
    rule(Z_PAD_MM > V3.CASTER_HOLDER_Z_MM, "lowest thing the robot carries: the pads at %.1f mm" % Z_PAD_MM)
    zl_at_sled_front = PRONG_T_MM + PLATE_T_MM + PLATE_RISE_MM * max(0.0, (X_AXLE_DOCKED_MM - SLED_X1_MM - X_HINGE_MM)) / (
        X_CROWN_MM - X_HINGE_MM)
    rule(zl_at_sled_front <= Z_PAD_MM - 0.3, "the sled's front end never reaches higher than %.1f mm of the plate's slope "
         "(the sled's face is at %.1f)" % (zl_at_sled_front, Z_PAD_MM))
    rule(True, "the plate runs from its hinge %.0f mm in front of the axle back to the crown %.0f mm behind it; prongs "
         "from the base to dock X %.0f (the mouth is at %.0f)" % (X_AXLE_DOCKED_MM - X_HINGE_MM, -PAD_X_MM,
                                                                  X_PRONG_TIP_MM, ARM_END_X_MM))
    rule(X_PRONG_TIP_MM >= ARM_END_X_MM + 1.0, "the prongs end inside the U")
    # ----- the U and what passes through it
    rule(D.BRACKET_W_MM < 2 * POCKET_HALF_W_MM and 2 * POCKET_HALF_W_MM - D.BRACKET_W_MM <= 1.2,
         "pocket (last %.0f mm) %.1f mm wide for the %.1f mm bar: +-%.1f (as v4)" % (POCKET_DEPTH_MM, 2 * POCKET_HALF_W_MM,
                                                                                   D.BRACKET_W_MM, D.POCKET_CLEAR_MM / 2))
    leg_out = L.leg_y_inner() + L.LEG_T_MM
    head_out = max(D.ARM_OUTER_Y_MM, leg_out) + D.M3_HEAD_H_MM
    rule(ARM_IN_HALF_W_MM - leg_out >= 0.8 and GROOVE_HALF_W_MM - head_out >= 1.0,
         "arms: the light plate's front legs (%.2f out) clear by %.2f; the M3 heads (to %.2f) run in the groove (to %.1f), "
         "%.1f-%.1f mm up" % (leg_out, ARM_IN_HALF_W_MM - leg_out, head_out, GROOVE_HALF_W_MM, *GROOVE_Z_MM))
    zh = (D.SIDE_INSERT_Z_MM[0] - D.M3_HEAD_DIA_MM / 2, D.SIDE_INSERT_Z_MM[0] + D.M3_HEAD_DIA_MM / 2)
    rule(GROOVE_Z_MM[0] + 0.8 <= zh[0] and zh[1] + 0.8 <= GROOVE_Z_MM[1], "groove %.1f-%.1f mm up holds the low screws' "
         "heads (%.1f-%.1f); the high screws (30.7 up) are above the 22 mm walls" % (*GROOVE_Z_MM, *zh))
    cap = FLARE_MM + play
    rule(cap >= 8.0, "mouth: a %.0f mm chamfer opens %.0f mm each side: catches a robot up to %.1f mm off centre "
         "(v4's funnel: 32 over 50 mm)" % (FLARE_L_MM, FLARE_MM, cap))
    # bumps and levers
    rule(D.FACE_ABOVE_BODY_MM > D.LEVER_PRESSED_MM and D.LEVER_FREE_MM > D.FACE_ABOVE_BODY_MM,
         "each inner bump presses its back switch's lever %.1f mm (clicks at %.1f) and stops on the bar's face"
         % (D.LEVER_FREE_MM - D.FACE_ABOVE_BODY_MM, D.LEVER_FREE_MM - D.LEVER_OPERATE_MM))
    rule(BUMP_Y_MM[0] > p_out + 1.0, "bumps (from %.0f mm out) clear of the prongs (to %.0f)" % (BUMP_Y_MM[0], p_out))
    # the marker: the docked robot's light plate must not hide it from the camera
    H_cam, d_far = 262.0, 90.0          # MEASURED: floor_calibration.json camera height 262 cm; far side of the 4 ft board
    plate_z, strip_z = (L.PLATE_UNDER_Z_MM + L.PLATE_T_MM) / 10.0, WALL_H_MM / 10.0
    shift = d_far * (plate_z / (H_cam - plate_z) - strip_z / (H_cam - strip_z)) * 10.0
    x_plate = X_AXLE_DOCKED_MM - L.PLATE_X_MM[0]
    gap = STRIP_X0_MM + STRIP_CLEAR_MM - x_plate
    rule(gap - shift >= 2.0, "marker: the docked robot's light plate ends %.0f mm short of the strip; seen from the camera "
         "(262 cm up) up to 90 cm away it looks %.0f mm closer: %.0f mm stay clear" % (gap, shift, gap - shift))
    # post stations: dock v2's checks with v5's faces
    with patched(D, BUMPER_Z0_MM=V3.REAR_BAR_Z0_MM, BUMPER_Z1_MM=V3.REAR_BAR_Z1_MM):
        with patched(V2, POST_H_MM=POST_H_MM, BACK_FACE_X_MM=BASE_D_MM, SIDE_FACE_Y_MM=HALF_OUT_MM,
                     SIDE_POST_XC_MM=side_post_xc()):
            ok2, l2 = V2.checks(dock)
    lines += ["     (post stations) " + l.replace("dock_v2", "dock_v5") for l in l2]
    ok = ok and ok2
    rule(WALL_H_MM - STRIP_T_MM + 0.2 - CAVITY_TOP_Z_MM >= 3.0, "cavity roof %.1f mm thick under the marker"
         % (WALL_H_MM - STRIP_T_MM + 0.2 - CAVITY_TOP_Z_MM))
    rule(STRIP_X0_MM + STRIP_DEP_MM + 2 * STRIP_CLEAR_MM <= BASE_D_MM - 1.5, "marker recess inside the base")
    return ok, lines


def path_clear(att, dock, offs=(0.0, 1.4), steps=(0, 5, 15, 30, 50, 75, 100)):
    """The attachment sliding into the U: the volume shared with the dock (mm^3) at each step and sideways offset
    (in the pocket, the last POCKET_DEPTH mm, the offset is held to its 0.5 mm play). Should be 0."""
    c = B.cq()
    to_dock = lambda s, dx, dy: s.rotate((0, 0, 0), (0, 0, 1), 180).translate((X_AXLE_DOCKED_MM - dx, dy, 0))
    out = []
    for off in offs:
        for s in steps:
            o = off if s > POCKET_DEPTH_MM + 1 else min(off, D.POCKET_CLEAR_MM / 2 - 0.05)
            a = to_dock(att, s + 0.05, o)
            v = a.intersect(dock).val().Volume() if a.val().BoundingBox().xmax > -200 else 0.0
            out.append((off, s, v))
    return out


# =============================================================================== pictures
def docked_png(att, sled, dock, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle, Polygon
    from build_attachment_v2 import _fill
    blue, sledc, dark, gold, grey = "#2f5d8a", "#5b8c3a", "#3a3a3a", "#b8860b", "#d9dee2"
    to_dock = lambda s: s.rotate((0, 0, 0), (0, 0, 1), 180).translate((X_AXLE_DOCKED_MM, 0, 0))
    attd, sledd = to_dock(att), to_dock(sled)
    fig, axs = plt.subplots(2, 1, figsize=(12, 11), dpi=110, gridspec_kw={"height_ratios": [0.8, 1.5]})
    ax = axs[0]
    yp = -PAD_Y_MM
    xb = X_AXLE_DOCKED_MM + D.AXLE_TO_FRONT_MM
    ax.add_patch(Polygon([(xb, Z_UNDER_MM), (xb - 95.25, Z_UNDER_MM), (xb - 95.25, 37.2), (xb, 37.2)], fc=grey, ec="#999"))
    ax.add_patch(Circle((X_AXLE_DOCKED_MM, D.WHEEL_R_MM), D.WHEEL_R_MM, fc="#555", alpha=0.3))
    ax.add_patch(Circle((X_AXLE_DOCKED_MM - CASTER_X_MM, V3.CASTER_BALL_R_MM), V3.CASTER_BALL_R_MM, fc="#888"))
    _fill(ax, dock, "XZ", yp, 0, 2, dark)
    _fill(ax, attd, "XZ", yp, 0, 2, blue)
    _fill(ax, sledd, "XZ", yp, 0, 2, sledc)
    for poly in plate_polys(CROWN_FREE_Z_MM - Z_PAD_MM):
        ax.add_patch(Polygon(poly, closed=True, fc=gold, ec=gold))
    sp = spring_line(CROWN_FREE_Z_MM - Z_PAD_MM)
    ax.plot([q[0] for q in sp], [q[1] for q in sp], color="#555", lw=0.8)
    xpad = robot_to_dock(PAD_X_MM, 0)[0]
    ax.add_patch(Rectangle((xpad - PAD_L_MM / 2, Z_PAD_MM), PAD_L_MM, PAD_T_MM, fc="#d4a017"))
    ax.axhline(0, color="k", lw=0.8)
    ax.set_xlim(-125, 50)
    ax.set_ylim(-3, 42)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.set_title("from the side, cut through the + pad and plate (robot's left, dock Y = %.0f)" % yp, fontsize=10)
    ax.set_xlabel("mm into the dock from the bottom of the U")
    ax.set_ylabel("mm above the plywood")
    ax = axs[1]
    ax.add_patch(Rectangle((xb - 95.25, -48), 95.25, 96, fc=grey, ec="#999"))
    for sg in (1, -1):
        ax.add_patch(Rectangle((X_AXLE_DOCKED_MM - V3.TYRE_X_MM, sg * WHEEL_INNER_Y_MM + (0 if sg > 0 else -7)),
                               2 * V3.TYRE_X_MM, 7, fc="#555", alpha=0.5))
    ax.add_patch(Circle((X_AXLE_DOCKED_MM - CASTER_X_MM, 0), V3.CASTER_BALL_R_MM, fc="#777"))
    _fill(ax, sledd, "XY", Z_PAD_MM + 0.5, 0, 1, sledc, alpha=0.75)
    _fill(ax, dock, "XY", 2.0, 0, 1, dark, alpha=0.9)
    _fill(ax, attd, "XY", V3.REAR_BAR_Z0_MM + 0.5, 0, 1, blue, alpha=0.9)
    for sg in (1, -1):
        y = sg * PAD_Y_MM
        ax.add_patch(Rectangle((X_HINGE_MM - PLATE_T_MM, y - PLATE_W_MM / 2), X_CROWN_MM - X_HINGE_MM + PLATE_T_MM + 1.3,
                               PLATE_W_MM, fc=gold, alpha=0.95))
        ax.text(X_HINGE_MM - 4, y, "+ plate" if sg < 0 else "- plate", fontsize=8, ha="right", va="center")
    xs = STRIP_X0_MM + STRIP_CLEAR_MM
    for i in range(STRIP_N[1]):
        for j in range(STRIP_N[0]):
            ax.add_patch(Rectangle((xs + i * STRIP_CELL_MM, -STRIP_LEN_MM / 2 + j * STRIP_CELL_MM), STRIP_CELL_MM,
                                   STRIP_CELL_MM, fc="k" if (i + j) % 2 == 0 else "w", ec="#888", lw=0.3))
    ax.text(X_AXLE_DOCKED_MM - CASTER_X_MM, 11, "caster", fontsize=8, ha="center")
    ax.text(X_AXLE_DOCKED_MM, 33, "wheel", fontsize=8, ha="center")
    ax.text(STRIP_CX_MM, STRIP_LEN_MM / 2 + 4, "marker strip", fontsize=8, ha="center")
    ax.set_xlim(-125, 60)
    ax.set_ylim(-80, 80)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.set_title("from above: the dock cut 2 mm up (black), the sled (green), the rear bar (blue), the contact plates (gold)",
                 fontsize=10)
    ax.set_xlabel("mm into the dock from the bottom of the U")
    ax.set_ylabel("mm left of the dock's axis")
    fig.text(0.5, 0.012, "Drawn from the CAD: a robot (grey, its shell from Arduino's model) backed into dock v5 (black). "
             "Its rear bar rests on the two bumps at the bottom of the U, and the gold plate on each prong, hinged at "
             "its front and held up by a spring at its rear, is pressed down by a brass pad in the sled under the robot.", ha="center", fontsize=9.5, wrap=True)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def contact_png(dock, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from build_attachment_v2 import _fill
    fig, ax = plt.subplots(figsize=(11, 4.4), dpi=115)
    gold, dark = "#b8860b", "#3a3a3a"
    _fill(ax, dock, "XZ", -PAD_Y_MM, 0, 2, dark, alpha=0.85)
    dn = CROWN_FREE_Z_MM - Z_PAD_MM
    for d, a, lab in ((0.0, 0.4, "free"), (dn, 1.0, "docked (crown pressed %.1f mm)" % dn)):
        for i, poly in enumerate(plate_polys(d)):
            ax.add_patch(Polygon(poly, closed=True, fc=gold, ec=gold, alpha=a, label=lab if i == 0 else None))
        sp = spring_line(d)
        ax.plot([q[0] for q in sp], [q[1] for q in sp], color="#555", lw=0.9, alpha=a)
    xf = X_HINGE_MM - PLATE_T_MM - FOOT_MM + 0.8
    ax.plot([xf, xf - 1.5, xf - 1.5, X_HINGE_MM + 14], [0.4, 0.4, 0.6, 0.6], color="#c0392b", lw=1.0,
            label="flexible lead, soldered to the foot (to the cavity)")
    ax.axhline(Z_PAD_MM, color="#5b8c3a", lw=1.2)
    ax.text(X_CROWN_MM + 5, Z_PAD_MM + 0.2, "pad face %.1f mm" % Z_PAD_MM, fontsize=8, color="#5b8c3a")
    ax.axhline(Z_UNDER_MM, color="#888", lw=1.0, ls="--")
    ax.text(X_CROWN_MM + 5, Z_UNDER_MM + 0.2, "robot's underside %.1f mm (the slot's roof)" % Z_UNDER_MM, fontsize=8,
            color="#666")
    ax.annotate("crown %.1f mm up when free" % CROWN_FREE_Z_MM, (X_CROWN_MM, CROWN_FREE_Z_MM), (X_CROWN_MM - 18, 12.6),
                fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.7))
    ax.annotate("hinge: the front tab rocks in a slot through the prong; its foot, bent under, stops it lifting",
                (X_HINGE_MM - 0.4, PRONG_T_MM + 0.8), (X_HINGE_MM - 6, 14.4), fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.7))
    ax.annotate("spring %.2f N/mm, %.0f mm long" % (SPRING_K_N_MM, SPRING_FREE_MM), (X_SPRING_MM, 3.0),
                (X_SPRING_MM + 6, 2.6), fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.7))
    ax.annotate("", (X_CROWN_MM + 22, 12.0), (X_CROWN_MM + 5, 12.0), arrowprops=dict(arrowstyle="->", lw=1.0,
                                                                                     color="#2f5d8a"))
    ax.text(X_CROWN_MM + 13, 12.4, "robot backs in this way", color="#2f5d8a", fontsize=8, ha="center")
    ax.set_xlim(X_PRONG_TIP_MM - 2, X_CROWN_MM + 28)
    ax.set_ylim(-1, 16)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.legend(loc="lower right", fontsize=7.5)
    ax.set_xlabel("mm into the dock from the bottom of the U")
    ax.set_ylabel("mm above the plywood")
    ax.set_title("The contact on one prong: a stiff brass plate (%.1f x %.2f mm), hinged at the front, sprung at the rear"
                 % (PLATE_T_MM, PLATE_W_MM), fontsize=10.5)
    fig.text(0.5, 0.02, "Cut through the prong (dark) from the CAD: light gold is the plate free, dark gold where the "
             "robot's pad holds it once docked. Any pad height between the two lines works; only the press, and so "
             "the force, changes.", ha="center", fontsize=9, wrap=True)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def lift_png(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ss = [i * 0.25 for i in range(0, 241)]
    fig, ax = plt.subplots(figsize=(9.5, 5.0), dpi=115)
    v3 = lift_curve_v3(ss)
    ax.plot([r[0] for r in v3], [100 * r[1] for r in v3], color="#c0392b", lw=2.0, label="v3: spring pins, 24 mm behind "
            "the axle")
    for dz, col, ls, lab in ((0.0, "#2f5d8a", "-", "v5: sprung plates, 8 mm behind the axle, robot as Arduino's CAD"),
                             (-0.5, "#2f5d8a", ":", "v5: robot sitting 0.5 mm lower than the CAD")):
        cur = lift_curve_v5(ss, dz=dz)
        ax.plot([r[0] for r in cur], [100 * r[1] for r in cur], color=col, lw=2.0, ls=ls, label=lab)
    off_bad = SLOT_HALF_W_MM - PLATE_W_MM / 2 + 0.6
    cur = lift_curve_v5(ss, off=off_bad)
    ax.plot([r[0] for r in cur], [100 * r[1] for r in cur], color="#7f8c8d", lw=1.4, ls="--",
            label="v5 if the robot came in %.1f mm off centre (the arms stop that)" % off_bad)
    ax.set_xlim(60, 0)
    ax.set_ylim(0, 128)
    ax.set_yticks(range(0, 101, 20))
    ax.set_xlabel("mm the robot still has to back in before its switches touch the bumps")
    ax.set_ylabel("% of the caster's load taken off")
    ax.set_title("Lift on the caster while a robot backs in (both contacts together)", fontsize=11)
    ax.grid(True, lw=0.3)
    ax.legend(fontsize=8, loc="upper left", ncol=2)
    fig.text(0.5, 0.015, "Computed from the CAD numbers and an ASSUMED 0.6 N on the caster (check list item 4 measures "
             "it): the share of the caster's load the contacts take off at each point of the back-in; above 100 % the "
             "caster would leave the floor.", ha="center", fontsize=8.5, wrap=True)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(path)
    plt.close(fig)


# =============================================================================== main
def on_bed(shape, flip=False):
    s = shape.rotate((0, 0, 0), (1, 0, 0), 180) if flip else shape
    b = s.val().BoundingBox()
    return s.translate((-b.xmin, -b.ymin, -b.zmin))


def export(shape, name):
    B.cq().exporters.export(shape, os.path.join(B.STL, name), tolerance=0.05, angularTolerance=0.1)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    os.makedirs(B.STL, exist_ok=True)
    os.makedirs(B.PNG, exist_ok=True)
    att = attachment_v5_solid()
    sled = sled_v5_solid()
    dock = dock_v5_solid()
    ok, lines = checks(att, sled, dock)
    print("dock v5, pad sled v5, attachment v5:")
    print("\n".join(lines))
    if "--no-path" not in argv:
        res = path_clear(att, dock)
        bad = [(o, s, v) for o, s, v in res if v > 0.5]
        print(("ok   " if not bad else "FAIL ") + "backing in (7 points from 100 mm out to docked, centred and 1.4 mm "
              "off): the attachment never runs into the U%s" % ("" if not bad else ": %s" % bad))
        ok = ok and not bad
    export(on_bed(dock), "dock_v5.stl")
    over = B.overhang_mm2(on_bed(dock))
    bridge = ((CAVITY_X_MM[1] - CAVITY_X_MM[0]) * 2 * CAVITY_HALF_Y_MM + 2 * (-X_HINGE_MM) * GROOVE_W_MM
              + 2 * (GROOVE_Z_MM[1] - GROOVE_Z_MM[0]) * 0 + 2 * (GROOVE_X0_MM - ARM_END_X_MM) * (GROOVE_HALF_W_MM - ARM_IN_HALF_W_MM))
    print(("ok   " if over <= bridge + 150 else "WARN ") + "dock_v5: print bottom down, brim on; %.0f mm^2 faces down, "
          "about %.0f of it the cavity roof, the wire grooves and the screw-head grooves' roofs (short bridges)"
          % (over, bridge))
    s_bed = on_bed(sled, flip=True)
    print("ok   pad sled v5: print top face down (as exported), no supports; %.0f mm^2 faces down (wire-channel roofs)"
          % B.overhang_mm2(s_bed))
    export(s_bed, "pad_sled_v5.stl")
    on_bed(att, flip=True).val().exportStl(os.path.join(B.STL, "attachment_v5.stl"), 0.05, 0.3)
    print("ok   attachment v5: attachment v3 with the pad-wire holes at +-%.0f mm; print as v3 (upside down, supports)"
          % WIRE_HOLE_Y_MM)
    export(on_bed(marker_strip_solid()), "marker_strip_v5.stl")
    print("ok   marker strip: %d x %d squares of %.0f mm (%.0f x %.0f mm), white with black raised squares (filament "
          "change at %.1f mm)" % (STRIP_N[0], STRIP_N[1], STRIP_CELL_MM, STRIP_LEN_MM, STRIP_DEP_MM, STRIP_T_MM))
    B.preview("dock_v5", dock, os.path.join(B.PNG, "dock_v5.png"), 2.0)
    B.preview("pad_sled_v5", sled, os.path.join(B.PNG, "sled_v5.png"), plan_z=Z_PAD_MM + 0.2)
    docked_png(att, sled, dock, os.path.join(B.PNG, "docked_v5.png"))
    contact_png(dock, os.path.join(B.PNG, "contact_v5.png"))
    lift_png(os.path.join(B.PNG, "lift_v5.png"))
    print("\nwrote %s/{dock_v5, pad_sled_v5, attachment_v5, marker_strip_v5}.stl" % B.STL)
    print("      %s/{dock_v5, sled_v5, docked_v5, contact_v5, lift_v5}.png" % B.PNG)
    print("\nfirmware and map numbers (back bumper as v3):")
    for l in V3.firmware_lines():
        print("  " + l)
    print("  marker strip centre %.1f mm behind the bottom of the U (session_checks DOCK_TILE_CENTER_CM %.2f), %.0f x %.0f mm"
          % (STRIP_CX_MM, STRIP_CX_MM / 10.0, STRIP_DEP_MM, STRIP_LEN_MM))
    print("  crowns at dock X %.1f, Y +-%.1f; prong tips at X %.1f" % (X_CROWN_MM, PAD_Y_MM, X_PRONG_TIP_MM))
    print("\nknown lengths (Nick: \"the one known length will be the dock\"): printed sizes, measure the print to check")
    for name, v, how in known_lengths():
        print("  %-52s %7.2f mm   %s" % (name, v, how))
    if not ok:
        print("\nSOME CHECKS FAILED")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
