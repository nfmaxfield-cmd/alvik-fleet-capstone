"""Dock v6: the pads move onto the robot's BATTERY COVER; two springs under each contact plate; a slightly wider mouth.

    python hardware/dock/build_dock_v6.py

writes
    stl/dock_v6.stl            the U as v5, with: the crowns moved to sit under the battery cover's pads, each plate
                               on TWO springs side by side (one under each edge: the plate cannot rock), the wire
                               groove under the middle of the prong, and the mouth's chamfer 20 mm long opening 12 mm
                               each side (catches a robot 13.5 mm off centre; v5: 8.5)
    stl/pad_carrier_v6.stl     the pad carrier: a 2.4 mm plate stuck to the battery cover with thin double-sided tape
                               (no drilling, the cover's screw stays free), two pads of the same 1/4 in brass strip, lanes
                               for the crowns, wire channels on its top face to the back edge
    (attachment: attachment_v5.stl, unchanged; marker strip: marker_strip_v5.stl, unchanged)
    preview/dock_v6.png, docked_v6.png (sections), contact_v6.png (one contact from the side and end on: the two springs),
    lift_v6.png (lift on the caster while backing in: v5 against v6), carrier_v6.png, carrier_on_photo_v6.png (the
    carrier drawn over Nick's photo of the underside, to check it sits on the battery cover)
and prints a check report (ok / WARN / FAIL) and the numbers for the firmware and the map.

Nick's decisions, 2026-10-25 13:03 (plan: docs/build-plans/self-charging-dock.md):
1. The pads attach to the BATTERY COVER (behind the wheels, in front of the caster), so they come off with it; the
   cover's centre screw stays clear.
2. Each contact plate gets TWO springs, one each side, stiffer, so the plate doesn't wobble.
3. The U may widen slightly at the mouth if that helps the first approach (decided: yes, the plan says why).
6. Parts: 1/32 x 1/4 in brass strip and a small spring assortment; no calipers (steel rule, pennies, paper).
Earlier versions stay as they were (build_dock_v5.py and before).

Frames (mm), as v5:
  robot: X forward from the wheel axle, Y to the robot's left, Z up from the floor
  dock:  X into the dock from the base's inner face (the bottom of the U), Y left, Z up from the plywood.
         A docked robot faces OUT of the dock: robot (x, y) -> dock (X_AXLE_DOCKED - x, -y).
Every number is SOURCED / MEASURED / ASSUMED (as dock_params.py) or CHOSEN; each CHOSEN one has a DOE note in the plan
and a row in docs/doe-backlog.md. PHOTO = measured on Nick's underside photo (docs/hardware/photos/
alvik-underside-2026-10-25.png) at 11.06 px/mm (the wheel centres 89.2 mm apart; the caster lands within 1.2 mm of
Arduino's CAD), about +-1.5 mm: MEASURE before printing (plan, Appendix B, step 0).
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
import build_dock_v5 as V5           # noqa: E402
from build_attachment_v3 import box, patched   # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
PHOTO = os.path.join(REPO, "docs", "hardware", "photos", "alvik-underside-2026-10-25.png")

# =============================================================================== the robot (as v5; SOURCED [STEP])
Z_UNDER_MM = V5.Z_UNDER_MM            # 10.1: flat underside (the cover is flush with it). MEASURE: paper stack, step 1
CASTER_X_MM = V5.CASTER_X_MM          # -41.0
CASTER_LOAD_N = V5.CASTER_LOAD_N      # 0.6 ASSUMED. MEASURE: Appendix B step 0b (kitchen scale under the caster)
WHEEL_INNER_Y_MM = V5.WHEEL_INNER_Y_MM
BACK_FACE_X_MM = V5.BACK_FACE_X_MM    # -47.6
BAR_FACE_X_MM = V5.BAR_FACE_X_MM      # -61.0

# =============================================================================== the battery cover (PHOTO)
COVER_FRONT_X_MM = -14.3              # PHOTO: the cover's front seam beside the screw's tab (both corners 14.3-14.4)
COVER_BACK_X_MM = -39.9               # PHOTO: its back seam (39.7-40.1)
COVER_HALF_W_MM = 35.5                # PHOTO: 71 mm across (seams at -36.9 and +35.0: the photo's centring is 1 mm off)
COVER_SCREW_X_MM = -9.6               # PHOTO: the screw, on the centre line, in a tab of the cover's front edge
COVER_TAB_R_MM = 8.0                  # PHOTO: the tab's seam runs about 8 mm round the screw
SCREW_HEAD_R_MM = 2.7                 # ASSUMED: a pan head about 5.4 mm across
CASTER_NOTCH_X_MM = -33.5             # PHOTO: the cover's back edge is notched round the caster from here back ...
CASTER_NOTCH_HALF_W_MM = 12.4         # ... this wide each side
PHOTO_PX_MM = 11.06                   # the photo's scale (above)
PHOTO_AXLE_PX = (632.0, 830.0)        # the axle's middle in the photo (between the two wheel centres)
PHOTO_LEFT_DIR = (0.9984, 0.0557)     # robot +Y (its left) in the photo: to the right (the underside is mirrored)

# =============================================================================== pad carrier v6 (robot side)
CARRIER_T_MM = 2.4                    # CHOSEN (as the v5 sled): 1.2 mm channels on top, 0.8 mm pad recess below
TAPE_T_MM = 0.2                       # ASSUMED: thin double-sided tape (carpet / mounting film); the paper-stack check
                                      # measures the real pad height, so any thin tape works
CARRIER_INSET_MM = 0.8                # CHOSEN: the carrier stays this far inside the cover's seams (lifting the cover
                                      # out must not catch it; the photo's +-1.5 mm is covered by the pad's margins)
CARRIER_X0_MM = COVER_BACK_X_MM + CARRIER_INSET_MM          # -39.1
CARRIER_X1_MM = COVER_FRONT_X_MM - CARRIER_INSET_MM         # -15.1
CARRIER_HALF_W_MM = COVER_HALF_W_MM - CARRIER_INSET_MM      # 34.7
NOTCH_X_MM = CASTER_NOTCH_X_MM + CARRIER_INSET_MM           # -32.7
NOTCH_HALF_W_MM = CASTER_NOTCH_HALF_W_MM + CARRIER_INSET_MM  # 13.2
PAD_Y_MM = 22.0                       # as v5: between the caster holder (9.5) and the wheels (41.2)
PAD_RIM_MM = 0.8                      # plastic in front of the pad
PAD_L_MM = 6.35                       # along X: a piece of the same 1/4 in brass strip, laid ACROSS the robot ...
PAD_W_MM = 10.0                       # ... cut 10 mm long (along Y): the plate (6.35) lands within the arms' play
PAD_T_MM = 0.8                        # 1/32 in strip
PAD_X1_MM = CARRIER_X1_MM - PAD_RIM_MM              # -15.9: the pad's front edge
PAD_X_MM = PAD_X1_MM - PAD_L_MM / 2                 # -19.1: its middle, where the crown sits when docked
PAD_X0_MM = PAD_X1_MM - PAD_L_MM                    # -22.25: its back edge (the crown meets it here)
SOLDER_HOLE_DIA_MM = 3.0              # through the carrier above the pad's middle: the wire is soldered to the pad's
                                      # back before the pad is glued in; the joint sits in this hole
RAMP_L_MM = 2.5                       # the lane's front end eases down to the pad (a chamfer)
SLOT_HALF_W_MM = 5.2                  # as v5: plate half width 3.2 + the arms' play 1.5 + 0.5
SLOT_FLARE_HALF_W_MM = 6.8            # the lane's mouth at the carrier's back end
SLOT_FLARE_L_MM = 5.0
CH_W_MM, CH_D_MM = 2.0, 1.2           # wire channel on the carrier's top (one 30 AWG silicone wire, about 0.6 mm)
CH_Y_MM = 31.0                        # CHOSEN: the channel runs back to the carrier's edge here, outboard of the lane
WIRE_HOLE_Y_MM = V5.WIRE_HOLE_Y_MM    # 33: attachment v5's holes up through the bar (unchanged)
Z_CARRIER_TOP_MM = Z_UNDER_MM - TAPE_T_MM           # 9.9
Z_PAD_MM = Z_CARRIER_TOP_MM - CARRIER_T_MM          # 7.5: the pads' face
RAMP_X0_MM = PAD_X0_MM - RAMP_L_MM                  # -24.75: where the lane ends

# =============================================================================== the contact: hinged plate, TWO springs
PLATE_T_MM = V5.PLATE_T_MM            # 0.8: 1/32 in brass strip, 1/4 in wide
PLATE_W_MM = V5.PLATE_W_MM            # 6.35
PLATE_E_MPA = V5.PLATE_E_MPA
PLATE_A_MM = 22.0                     # CHOSEN (as v5): hinge to crown
PRESS_MM = 1.0                        # CHOSEN: how far the docked pad pushes the crown down (v5: 1.3). With the
                                      # stiffer springs 1 mm gives the force; less press, less change when the robot
                                      # sits a little higher or lower than measured. DOE: with the spring rate (plan)
CROWN_FREE_Z_MM = Z_PAD_MM + PRESS_MM  # 8.5: set by the springs' length; the paper-stack check sets it to the robot
TAB_DOWN_MM, FOOT_MM = V5.TAB_DOWN_MM, V5.FOOT_MM
FOOT_RECESS_Z_MM = V5.FOOT_RECESS_Z_MM
HINGE_SLOT_X_MM = V5.HINGE_SLOT_X_MM
CURL_MM, CURL_DEG = V5.CURL_MM, V5.CURL_DEG
SPRING_N = 2                          # Nick 13:03: one each side of the plate
SPRING_K_N_MM = 0.15                  # CHOSEN target, EACH spring (15 g per mm; together 0.30 N/mm, v5's single spring
                                      # 0.20): the plate's crown then takes 0.25 N per mm pressed. Pick by the penny test
SPRING_OD_MM = 3.0                    # ASSUMED (the assortment's chart): 2.5-3.0 mm across
SPRING_POCKET_R_MM = SPRING_OD_MM / 2 + 0.25
SPRING_WALL_MM = 1.2                  # CHOSEN: printed wall between the two pockets (3 lines of a 0.4 mm nozzle)
SPRING_Y_OFF_MM = SPRING_POCKET_R_MM + SPRING_WALL_MM / 2    # 2.35: each spring's centre from the plate's middle
SPRING_SOLID_MM = V5.SPRING_SOLID_MM  # 2.0 ASSUMED
SPRING_BACK_MM = V5.SPRING_BACK_MM    # 2.0: the springs' centres this far in front of the crown
SPRING_FLOOR_Z_MM = 1.9               # the pockets' floor above the plywood (0.5 mm over the wire groove)
LIP_FLOOR_Z_MM = V5.LIP_FLOOR_Z_MM    # 2.4: the lip's pocket floor is the stop
LEAD = V5.LEAD

# =============================================================================== the dock (U)
BUMP_OUT_MM = V5.BUMP_OUT_MM
BUMP_Y_MM = V5.BUMP_Y_MM
WALL_H_MM = V5.WALL_H_MM
POCKET_HALF_W_MM = V5.POCKET_HALF_W_MM
POCKET_DEPTH_MM = V5.POCKET_DEPTH_MM
ARM_IN_HALF_W_MM = V5.ARM_IN_HALF_W_MM
ARM_T_MM = V5.ARM_T_MM
ARM_END_X_MM = -115.0                 # CHOSEN: the mouth, 10 mm further out than v5 (the chamfer is longer) ...
FLARE_L_MM = 20.0                     # CHOSEN: ... a 20 mm chamfer ...
FLARE_MM = 12.0                       # ... opening 12 mm each side (31 deg): catches a robot 13.5 mm off centre
HORN_WALL_MM = 3.0                    # the arm's end flares outward with it, keeping this much wall
GROOVE_Z_MM = V5.GROOVE_Z_MM
GROOVE_HALF_W_MM = V5.GROOVE_HALF_W_MM
GROOVE_X0_MM = V5.GROOVE_X0_MM
BASE_D_MM = V5.BASE_D_MM
HALF_OUT_MM = V5.HALF_OUT_MM
PRONG_HALF_W_MM = V5.PRONG_HALF_W_MM  # 6: the plate, two spring pockets (to 4.1 out) and 1.9 of wall
PRONG_T_MM = V5.PRONG_T_MM
GROOVE_W_MM, GROOVE_D_MM = V5.GROOVE_W_MM, V5.GROOVE_D_MM   # the wire groove, now under the prong's middle
TAB_W_MM = V5.TAB_W_MM
TAB_HOLE_DIA_MM, TAB_CSK_DIA_MM = V5.TAB_HOLE_DIA_MM, V5.TAB_CSK_DIA_MM
CAVITY_X_MM = V5.CAVITY_X_MM
CAVITY_HALF_Y_MM = V5.CAVITY_HALF_Y_MM
CAVITY_TOP_Z_MM = V5.CAVITY_TOP_Z_MM
CHANNEL_HALF_W_MM, CHANNEL_H_MM = V5.CHANNEL_HALF_W_MM, V5.CHANNEL_H_MM
SCREW_HOLES = V5.SCREW_HOLES
POST_H_MM = V5.POST_H_MM
MARK_DIA_MM = V5.MARK_DIA_MM
STRIP_CELL_MM, STRIP_N = V5.STRIP_CELL_MM, V5.STRIP_N
STRIP_T_MM, STRIP_CLEAR_MM, STRIP_X0_MM = V5.STRIP_T_MM, V5.STRIP_CLEAR_MM, V5.STRIP_X0_MM

# derived
X_AXLE_DOCKED_MM = V5.X_AXLE_DOCKED_MM                                    # -65.0
X_CROWN_MM = X_AXLE_DOCKED_MM - PAD_X_MM                                # -45.9
PLATE_RISE_MM = CROWN_FREE_Z_MM - PRONG_T_MM - PLATE_T_MM
PLATE_THETA = math.asin(PLATE_RISE_MM / PLATE_A_MM)
X_HINGE_MM = X_CROWN_MM - PLATE_A_MM * math.cos(PLATE_THETA)
X_SPRING_MM = X_CROWN_MM - SPRING_BACK_MM
A_RATIO = (X_SPRING_MM - X_HINGE_MM) / (X_CROWN_MM - X_HINGE_MM)
SPRING_FREE_MM = PRONG_T_MM + (X_SPRING_MM - X_HINGE_MM) * math.tan(PLATE_THETA) - SPRING_FLOOR_Z_MM
_LIP_TIP_FREE = CROWN_FREE_Z_MM - PLATE_T_MM - CURL_MM * math.sin(math.radians(CURL_DEG))
DELTA_MAX_MM = min(PLATE_RISE_MM, _LIP_TIP_FREE - LIP_FLOOR_Z_MM)
X_PRONG_TIP_MM = X_HINGE_MM - PLATE_T_MM - FOOT_MM - 3.0
TAB_Y_MM = PAD_Y_MM + PRONG_HALF_W_MM + TAB_W_MM / 2 - 0.5
X_TAB_MM = X_HINGE_MM - 1.0
STRIP_LEN_MM, STRIP_DEP_MM = V5.STRIP_LEN_MM, V5.STRIP_DEP_MM
STRIP_CX_MM = V5.STRIP_CX_MM


def contact_k():
    """Vertical stiffness at the crown, N/mm: both springs seen through the plate's lever."""
    return SPRING_N * SPRING_K_N_MM * A_RATIO ** 2


def roll_k():
    """The plate's resistance to rocking sideways about its long axis, N mm per degree (two springs at +-Y_OFF; one
    spring under the middle, as v5, gives none: the plate rests on a point)."""
    return SPRING_N * SPRING_K_N_MM * SPRING_Y_OFF_MM ** 2 * math.pi / 180.0


def plate_bend_mm(F):
    I = PLATE_W_MM * PLATE_T_MM ** 3 / 12.0
    L_ = SPRING_BACK_MM + 1.0
    return F * L_ ** 3 / (3 * PLATE_E_MPA * I)


def lift_frac(force_total_n, x_mm, caster_load=None):
    """Share of the caster's load taken off by an upward force at robot x (behind the axle: x < 0)."""
    return force_total_n * max(0.0, -x_mm) / abs(CASTER_X_MM) / (caster_load or CASTER_LOAD_N)


def robot_to_dock(x, y):
    return X_AXLE_DOCKED_MM - x, -y


def photo_px(x, y):
    """Robot (x, y) mm -> pixel (u, v) in Nick's underside photo."""
    lx, ly = PHOTO_LEFT_DIR
    bx, by = -ly, lx                      # robot BACKWARDS in the photo (down the picture)
    return (PHOTO_AXLE_PX[0] + PHOTO_PX_MM * (y * lx - x * bx), PHOTO_AXLE_PX[1] + PHOTO_PX_MM * (y * ly - x * by))


# =============================================================================== robot side: the pad carrier
def cover_outline():
    """The battery cover's seam (robot frame, PHOTO), [(x, y)] round the outline, the screw's tab drawn as an arc."""
    pts = [(COVER_BACK_X_MM, -COVER_HALF_W_MM), (COVER_FRONT_X_MM, -COVER_HALF_W_MM)]
    # the tab: an arc round the screw from where it meets the front seam on the right, forward, to the left
    a0 = math.acos(max(-1.0, min(1.0, (COVER_FRONT_X_MM - COVER_SCREW_X_MM) / COVER_TAB_R_MM)))
    for i in range(17):
        a = -a0 + i / 16.0 * 2 * a0                 # angle from the robot's forward direction
        pts.append((COVER_SCREW_X_MM + COVER_TAB_R_MM * math.cos(a), COVER_TAB_R_MM * math.sin(a)))
    pts += [(COVER_FRONT_X_MM, COVER_HALF_W_MM), (COVER_BACK_X_MM, COVER_HALF_W_MM),
            (COVER_BACK_X_MM, CASTER_NOTCH_HALF_W_MM), (CASTER_NOTCH_X_MM, CASTER_NOTCH_HALF_W_MM),
            (CASTER_NOTCH_X_MM, -CASTER_NOTCH_HALF_W_MM), (COVER_BACK_X_MM, -CASTER_NOTCH_HALF_W_MM)]
    return pts


def carrier_outline():
    h, n = CARRIER_HALF_W_MM, NOTCH_HALF_W_MM
    return [(CARRIER_X0_MM, -h), (CARRIER_X1_MM, -h), (CARRIER_X1_MM, h), (CARRIER_X0_MM, h), (CARRIER_X0_MM, n),
            (NOTCH_X_MM, n), (NOTCH_X_MM, -n), (CARRIER_X0_MM, -n)]


def carrier_v6_solid():
    """Robot frame. Top face (taped to the cover) at Z_CARRIER_TOP_MM, pads' face at Z_PAD_MM."""
    c = B.cq()
    z0, z1 = Z_PAD_MM, Z_CARRIER_TOP_MM
    body = c.Workplane("XY").workplane(offset=z0).polyline(carrier_outline()).close().extrude(CARRIER_T_MM)
    body = body.edges("|Z").fillet(1.0)
    for sg in (1, -1):
        y = sg * PAD_Y_MM
        # the crown's lane from the back edge to the ramp, with a flared mouth
        body = body.cut(box(CARRIER_X0_MM - 1, RAMP_X0_MM, y - SLOT_HALF_W_MM, y + SLOT_HALF_W_MM, z0 - 1, z1 + 1))
        fl = (c.Workplane("XY").workplane(offset=z0 - 1)
              .polyline([(CARRIER_X0_MM - 1, y - SLOT_FLARE_HALF_W_MM), (CARRIER_X0_MM + SLOT_FLARE_L_MM, y - SLOT_HALF_W_MM),
                         (CARRIER_X0_MM + SLOT_FLARE_L_MM, y + SLOT_HALF_W_MM), (CARRIER_X0_MM - 1, y + SLOT_FLARE_HALF_W_MM)])
              .close().extrude(CARRIER_T_MM + 2))
        body = body.cut(fl)
        ramp = (c.Workplane("XZ").polyline([(RAMP_X0_MM - 0.01, z0 - 1), (RAMP_X0_MM - 0.01, z1 + 0.5),
                                             (PAD_X0_MM, z0), (PAD_X0_MM, z0 - 1)])
                .close().extrude(-(2 * SLOT_HALF_W_MM)).translate((0, y - SLOT_HALF_W_MM, 0)))
        body = body.cut(ramp)
        # the pad's recess (0.8 deep) and the solder hole above its middle
        body = body.cut(box(PAD_X0_MM, PAD_X1_MM, y - PAD_W_MM / 2, y + PAD_W_MM / 2, z0 - 1, z0 + PAD_T_MM))
        body = body.cut(B.cyl(SOLDER_HOLE_DIA_MM / 2, CARRIER_T_MM + 2, (PAD_X_MM, y, z0 - 1), (0, 0, 1)))
        # wire channel on the top: out from the hole to CH_Y, then back to the carrier's back edge
        yc = sg * CH_Y_MM
        body = body.cut(box(PAD_X_MM - CH_W_MM / 2, PAD_X_MM + CH_W_MM / 2, min(y, yc) - CH_W_MM / 2,
                            max(y, yc) + CH_W_MM / 2, z1 - CH_D_MM, z1 + 1))
        body = body.cut(box(CARRIER_X0_MM - 1, PAD_X_MM + CH_W_MM / 2, yc - CH_W_MM / 2, yc + CH_W_MM / 2,
                            z1 - CH_D_MM, z1 + 1))
    # a centre-line notch at the front edge: lines up with the cover's screw when the carrier is stuck on
    body = body.cut(c.Workplane("XY").workplane(offset=z0 - 1).center(CARRIER_X1_MM, 0).polygon(3, 3.0)
                    .extrude(CARRIER_T_MM + 2))
    return body


# =============================================================================== the plate (drawn; bought strip)
def contact_profile(delta=0.0):
    zc = CROWN_FREE_Z_MM - delta
    z0 = PRONG_T_MM + PLATE_T_MM
    pts = [(X_HINGE_MM + i / 24.0 * (X_CROWN_MM - X_HINGE_MM), z0 + i / 24.0 * (zc - z0)) for i in range(25)]
    tilt = math.atan2(zc - z0, X_CROWN_MM - X_HINGE_MM)
    a = math.radians(CURL_DEG) - (PLATE_THETA - tilt)
    pts.append((X_CROWN_MM + CURL_MM * math.cos(a), zc - CURL_MM * math.sin(a)))
    return pts


def plate_polys(delta=0.0):
    top = contact_profile(delta)
    body = top + [(x, z - PLATE_T_MM) for (x, z) in top[::-1]]
    z0 = PRONG_T_MM + PLATE_T_MM
    zb = PRONG_T_MM - TAB_DOWN_MM + 0.2
    tab = [(X_HINGE_MM - PLATE_T_MM, z0), (X_HINGE_MM, z0), (X_HINGE_MM, zb), (X_HINGE_MM - PLATE_T_MM - FOOT_MM, zb),
           (X_HINGE_MM - PLATE_T_MM - FOOT_MM, zb + PLATE_T_MM), (X_HINGE_MM - PLATE_T_MM, zb + PLATE_T_MM)]
    return [body, tab]


def spring_top_z(delta=0.0):
    return PRONG_T_MM + (X_SPRING_MM - X_HINGE_MM) * math.tan(PLATE_THETA) - delta * A_RATIO


def spring_line(delta=0.0, coils=6, x=None):
    top = spring_top_z(delta)
    r = SPRING_OD_MM / 2
    n = 2 * coils
    xc = X_SPRING_MM if x is None else x
    return [(xc + (r if i % 2 else -r) * (0 < i < n), SPRING_FLOOR_Z_MM + (top - SPRING_FLOOR_Z_MM) * i / n)
            for i in range(n + 1)]


# =============================================================================== the dock
def inner_half_w(x):
    if x >= -POCKET_DEPTH_MM:
        return POCKET_HALF_W_MM
    if x >= -POCKET_DEPTH_MM - 1.0:
        return POCKET_HALF_W_MM + (-POCKET_DEPTH_MM - x) * (ARM_IN_HALF_W_MM - POCKET_HALF_W_MM)
    if x >= ARM_END_X_MM + FLARE_L_MM:
        return ARM_IN_HALF_W_MM
    return ARM_IN_HALF_W_MM + FLARE_MM * (ARM_END_X_MM + FLARE_L_MM - x) / FLARE_L_MM


def side_post_xc():
    return (BASE_D_MM + ARM_END_X_MM) / 2


def horn_out_mm():
    """How far the arm's end sticks out past the arm's outer face at the mouth."""
    return max(0.0, ARM_IN_HALF_W_MM + FLARE_MM + HORN_WALL_MM - HALF_OUT_MM)


def dock_v6_solid():
    c = B.cq()
    H = WALL_H_MM
    body = box(0, BASE_D_MM, -HALF_OUT_MM, HALF_OUT_MM, 0, H)
    pkt, arm = POCKET_HALF_W_MM, ARM_IN_HALF_W_MM
    xf = ARM_END_X_MM + FLARE_L_MM
    out_end = HALF_OUT_MM + horn_out_mm()
    for sg in (1, -1):
        pts = [(0.5, pkt), (-POCKET_DEPTH_MM, pkt), (-POCKET_DEPTH_MM - 1.0, arm), (xf, arm),
               (ARM_END_X_MM, arm + FLARE_MM), (ARM_END_X_MM, out_end), (xf, HALF_OUT_MM), (0.5, HALF_OUT_MM)]
        a = c.Workplane("XY").polyline([(x, sg * y) for x, y in pts]).close().extrude(H)
        a = a.cut(box(ARM_END_X_MM - 1, GROOVE_X0_MM, *sorted((sg * 40.0, sg * GROOVE_HALF_W_MM)), *GROOVE_Z_MM))
        body = body.union(a)
        e = (c.Workplane("XY").workplane(offset=GROOVE_Z_MM[0])
             .polyline([(GROOVE_X0_MM - 0.01, sg * arm), (GROOVE_X0_MM - 0.01, sg * GROOVE_HALF_W_MM),
                        (GROOVE_X0_MM + 3.0, sg * arm)]).close().extrude(GROOVE_Z_MM[1] - GROOVE_Z_MM[0]))
        body = body.cut(e)
        ya, yb = sorted((sg * BUMP_Y_MM[0], sg * (BUMP_Y_MM[1] + 0.5)))
        body = body.union(box(-BUMP_OUT_MM, 1.0, ya, yb, 0, H))
    for sg in (1, -1):
        y = sg * PAD_Y_MM
        hw = PRONG_HALF_W_MM
        prong = box(X_PRONG_TIP_MM, 1.0, y - hw, y + hw, 0, PRONG_T_MM)
        prong = prong.faces("<X").edges(">Z").chamfer(1.5)
        body = body.union(prong)
        pw = PLATE_W_MM / 2 + 0.3
        body = body.cut(box(X_HINGE_MM - PLATE_T_MM - (HINGE_SLOT_X_MM - PLATE_T_MM) / 2,
                            X_HINGE_MM + (HINGE_SLOT_X_MM - PLATE_T_MM) / 2, y - pw, y + pw, -1, PRONG_T_MM + 1))
        body = body.cut(box(X_HINGE_MM - PLATE_T_MM - FOOT_MM - 0.6, X_HINGE_MM, y - pw, y + pw, -1, FOOT_RECESS_Z_MM))
        # TWO spring pockets, side by side under the plate's edges
        for so in (1, -1):
            body = body.cut(B.cyl(SPRING_POCKET_R_MM, PRONG_T_MM, (X_SPRING_MM, y + so * SPRING_Y_OFF_MM,
                                                                   SPRING_FLOOR_Z_MM), (0, 0, 1)))
        body = body.cut(box(X_CROWN_MM - 0.5, X_CROWN_MM + CURL_MM + 1.5, y - pw, y + pw, LIP_FLOOR_Z_MM,
                            PRONG_T_MM + 1))
        # wire groove under the prong's middle (beneath the pockets), from the foot to the cavity
        body = body.cut(box(X_HINGE_MM - HINGE_SLOT_X_MM, CAVITY_X_MM[0] + 1, y - GROOVE_W_MM / 2, y + GROOVE_W_MM / 2,
                            -1, GROOVE_D_MM))
        xt, yt = X_TAB_MM, sg * TAB_Y_MM
        body = body.union(box(xt - TAB_W_MM / 2, xt + TAB_W_MM / 2, *sorted((sg * (PAD_Y_MM + hw - 0.5),
                                                                               sg * (TAB_Y_MM + TAB_W_MM / 2))), 0, PRONG_T_MM))
        body = body.cut(B.cyl(TAB_HOLE_DIA_MM / 2, PRONG_T_MM + 2, (xt, yt, -1), (0, 0, 1)))
        body = body.cut(c.Workplane("XY").workplane(offset=PRONG_T_MM - TAB_CSK_DIA_MM / 2).center(xt, yt)
                        .circle(0.01).workplane(offset=TAB_CSK_DIA_MM / 2 + 0.01).circle(TAB_CSK_DIA_MM / 2 + 0.01).loft())
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
    s = D.SWITCH_LATERAL_MM
    for y in (s, -s):
        body = body.union(V5.post((BASE_D_MM, y), (1, 0), (0, 1)))
    xc = side_post_xc()
    for sg in (1, -1):
        for x in (xc + s, xc - s):
            body = body.union(V5.post((x, sg * HALF_OUT_MM), (0, sg), (1, 0)))
    body = body.cut(c.Workplane("XY").workplane(offset=H - 0.8).center(BASE_D_MM - 1.6, -PAD_Y_MM)
                    .circle(MARK_DIA_MM / 2).extrude(2))
    return body


# =============================================================================== the entry: lift on the caster
def underside_v6(x, off=0.0, dz=0.0):
    """Robot underside height (mm) along the plate's line (robot y = PAD_Y + off) at robot x: the cover / body (the
    lane's roof) or the carrier / pad face. dz shifts it (a real robot sitting lower: dz < 0)."""
    y_rel = abs(off)
    in_slot = y_rel + PLATE_W_MM / 2 <= SLOT_HALF_W_MM or (x < CARRIER_X0_MM + SLOT_FLARE_L_MM and
                                                         y_rel + PLATE_W_MM / 2 <= SLOT_FLARE_HALF_W_MM)
    if x < BAR_FACE_X_MM:
        return None
    if x < CARRIER_X0_MM:
        return Z_UNDER_MM + dz
    if x <= RAMP_X0_MM and in_slot:
        return Z_UNDER_MM + dz
    if x < PAD_X0_MM and in_slot:
        return Z_UNDER_MM + dz - (x - RAMP_X0_MM) / RAMP_L_MM * (Z_UNDER_MM - Z_PAD_MM)
    if x <= CARRIER_X1_MM:
        return Z_PAD_MM + dz
    return Z_UNDER_MM + dz


def lift_curve_v6(s_list, off=0.0, dz=0.0):
    """[(s, lift share, force N per plate, contact robot x)]: s = mm still to go before the bumps (as v5)."""
    k = contact_k()
    out = []
    sh = X_CROWN_MM - X_HINGE_MM
    for s in s_list:
        best = (0.0, None)
        for (X, zl) in contact_profile(0.0)[1:]:
            x = X_AXLE_DOCKED_MM - s - X
            zr = underside_v6(x, off, dz)
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


def known_lengths():
    """As v5 (the dock is the system's known length), with v6's mouth."""
    out = []
    for name, v, how in V5.known_lengths():
        if name.startswith("side stations"):
            v = 2 * (HALF_OUT_MM + V2.POST_STANDOFF_MM)
        out.append((name, v, how))
    return out


# =============================================================================== checks
def checks(att, carrier, dock):
    lines, ok = [], True

    def rule(good, text, warn=False):
        nonlocal ok
        if not good and not warn:
            ok = False
        lines.append(("ok   " if good else ("WARN " if warn else "FAIL ")) + text)

    for nm, s in (("attachment_v5", att), ("pad carrier v6", carrier), ("dock_v6", dock)):
        rule(s.val().isValid() and len(s.val().Solids()) == 1, "%s: one valid solid" % nm)
    b = B.bbox(dock)
    rule(b[3] - b[0] + 16 <= D.BED_MM[0] and b[4] - b[1] + 16 <= D.BED_MM[1],
         "dock_v6: %.0f x %.0f x %.0f mm (v5: 151 x 145), fits a 200 mm bed with a brim" % (b[3] - b[0], b[4] - b[1],
                                                                                            b[5] - b[2]))
    # ----- the robot side: the carrier on the battery cover
    rule(CARRIER_X1_MM <= COVER_FRONT_X_MM - 0.5 and CARRIER_X0_MM >= COVER_BACK_X_MM + 0.5 and
         CARRIER_HALF_W_MM <= COVER_HALF_W_MM - 0.5 and NOTCH_HALF_W_MM >= CASTER_NOTCH_HALF_W_MM + 0.5,
         "carrier %.1f x %.1f mm lies %.1f mm inside the battery cover's seams (PHOTO), round its caster notch"
         % (CARRIER_X1_MM - CARRIER_X0_MM, 2 * CARRIER_HALF_W_MM, CARRIER_INSET_MM))
    gap_screw = (COVER_SCREW_X_MM - SCREW_HEAD_R_MM) - CARRIER_X1_MM
    rule(gap_screw >= 2.0 and CARRIER_X1_MM <= COVER_SCREW_X_MM - COVER_TAB_R_MM * 0.5,
         "the cover's screw stays free: its head ends %.1f mm in front of the carrier, which stays off the screw's tab"
         % gap_screw)
    margin_f = PAD_X1_MM - PAD_X_MM - 1.0
    rule(margin_f >= 1.5, "docked, the crown sits on the pad's middle, %.1f mm behind the axle; the cover's real seam "
         "can be %.1f mm off the photo's either way before the crown leaves the pad (step 0 measures it)"
         % (-PAD_X_MM, margin_f))
    lane_in, lane_out = PAD_Y_MM - SLOT_FLARE_HALF_W_MM, PAD_Y_MM + SLOT_FLARE_HALF_W_MM
    rule(lane_in - NOTCH_HALF_W_MM >= 0.8 and CH_Y_MM - CH_W_MM / 2 - lane_out >= 0.8 and
         CARRIER_HALF_W_MM - CH_Y_MM - CH_W_MM / 2 >= 1.5,
         "lanes %.1f-%.1f mm out (flared mouths): %.1f mm of carrier to the caster notch, %.1f to the wire channel at %.0f"
         % (lane_in, lane_out, lane_in - NOTCH_HALF_W_MM, CH_Y_MM - CH_W_MM / 2 - lane_out, CH_Y_MM))
    rule(abs(WIRE_HOLE_Y_MM - CH_Y_MM) <= 3.0 and WIRE_HOLE_Y_MM - PAD_Y_MM - PLATE_W_MM / 2 - 1.5 >= 4.0,
         "pad wires leave the carrier's back edge at +-%.0f mm, run %.0f mm along the robot's underside (taped) and go "
         "up through attachment v5's holes at +-%.0f mm, %.1f mm outboard of any crown"
         % (CH_Y_MM, CARRIER_X0_MM - V3.WIRE_HOLE_X_MM, WIRE_HOLE_Y_MM, WIRE_HOLE_Y_MM - PAD_Y_MM - PLATE_W_MM / 2 - 1.5))
    rule(Z_PAD_MM > V3.CASTER_HOLDER_Z_MM, "lowest thing the robot carries is still the caster holder (%.1f mm); the "
         "pads' face is %.1f mm up" % (V3.CASTER_HOLDER_Z_MM, Z_PAD_MM))
    # ----- the contact
    k = contact_k()
    F = k * PRESS_MM
    rule(True, "contact: brass plate %.1f x %.2f mm, hinge to crown %.0f mm, rising %.1f mm (%.0f deg); TWO springs of "
         "%.2f N/mm, %.2f mm either side of the plate's middle, %.0f mm in front of the crown: %.3f N/mm at the crown"
         % (PLATE_T_MM, PLATE_W_MM, PLATE_A_MM, PLATE_RISE_MM, math.degrees(PLATE_THETA), SPRING_K_N_MM,
            SPRING_Y_OFF_MM, SPRING_BACK_MM, k))
    rule(roll_k() > 0.01, "the plate resists rocking sideways: %.2f N mm per degree (one spring in the middle, v5: 0)"
         % roll_k())
    rule(0.2 <= F <= 0.35, "docked, each crown is pressed %.1f mm: %.2f N (%.0f g) per plate (v5: 0.21)"
         % (PRESS_MM, F, F / 9.81e-3))
    ld = lift_frac(2 * F, PAD_X_MM)
    rule(ld <= 0.45, "docked, the two plates push up %.2f N, %.1f mm behind the axle: %.0f %% of the caster's ASSUMED "
         "%.1f N (v5: 14 %%, pads 8 mm behind; v3: 59 %%)" % (2 * F, -PAD_X_MM, 100 * ld, CASTER_LOAD_N))
    for dz, lim in ((-0.4, 0.75), (-1.0, 0.9)):
        d = PRESS_MM - dz
        cur = lift_curve_v6([i * 0.5 for i in range(0, 121)], dz=dz)
        pk = max(r[1] for r in cur)
        rule(pk <= lim and Z_UNDER_MM + dz - CROWN_FREE_Z_MM >= 0.0,
             "robot %.1f mm lower than measured: pressed %.1f mm, %.2f N per plate, %.0f %% docked, %.0f %% at most "
             "while backing in; crown %.1f mm under the lane's roof" % (-dz, d, k * d, 100 * lift_frac(2 * k * d, PAD_X_MM),
                                                                       100 * pk, Z_UNDER_MM + dz - CROWN_FREE_Z_MM),
             warn=dz < -0.5)
    hi = 0.6
    rule(PRESS_MM - hi >= 0.3, "robot %.1f mm higher than measured: still pressed %.1f mm (%.2f N per plate)"
         % (hi, PRESS_MM - hi, k * (PRESS_MM - hi)))
    rule(Z_UNDER_MM - CROWN_FREE_Z_MM >= 0.8, "a free crown passes %.1f mm under the robot's underside, the cover and "
         "the bar (the lane's roof)" % (Z_UNDER_MM - CROWN_FREE_Z_MM))
    comp_max = DELTA_MAX_MM * A_RATIO
    rule(SPRING_FREE_MM - comp_max >= SPRING_SOLID_MM + 0.3,
         "springs: %.1f mm long free (pockets' floor %.1f mm up); the lip's pocket stops the crown after %.1f mm, the "
         "springs then %.1f mm long (solid at about %.1f)" % (SPRING_FREE_MM, SPRING_FLOOR_Z_MM, DELTA_MAX_MM,
                                                             SPRING_FREE_MM - comp_max, SPRING_SOLID_MM))
    Fmax = k * DELTA_MAX_MM
    rule(plate_bend_mm(Fmax) < 0.05, "stiff: at the stop (%.2f N) the plate itself bends %.3f mm" % (Fmax, plate_bend_mm(Fmax)))
    rot = math.atan(DELTA_MAX_MM / (X_CROWN_MM - X_HINGE_MM))
    rule(FOOT_RECESS_Z_MM >= PLATE_T_MM + FOOT_MM * math.sin(rot) + 0.3 and PRONG_T_MM - FOOT_RECESS_Z_MM >= 1.6,
         "hinge: the foot swings up %.1f mm at the stop in its %.1f mm recess" % (FOOT_MM * math.sin(rot), FOOT_RECESS_Z_MM))
    pocket_edge = SPRING_Y_OFF_MM + SPRING_POCKET_R_MM
    rule(PRONG_HALF_W_MM - pocket_edge >= 1.2 and SPRING_Y_OFF_MM - SPRING_POCKET_R_MM >= 0.5,
         "spring pockets %.2f mm across, %.1f mm of wall between them and %.1f mm to the prong's sides"
         % (2 * SPRING_POCKET_R_MM, 2 * (SPRING_Y_OFF_MM - SPRING_POCKET_R_MM), PRONG_HALF_W_MM - pocket_edge))
    over = SPRING_Y_OFF_MM + SPRING_OD_MM / 2 - PLATE_W_MM / 2
    rule(over <= SPRING_OD_MM / 3, "each spring's top coil bears on the plate, %.1f mm of it past the plate's edge"
         % max(0.0, over))
    rule(SPRING_FLOOR_Z_MM - GROOVE_D_MM >= 0.4 and LIP_FLOOR_Z_MM - GROOVE_D_MM >= 0.8,
         "wire groove under the prong's middle: %.1f mm of floor under the spring pockets, %.1f under the lip's pocket"
         % (SPRING_FLOOR_Z_MM - GROOVE_D_MM, LIP_FLOOR_Z_MM - GROOVE_D_MM))
    ss = [i * 0.5 for i in range(0, 161)]
    for off, lab in ((0.0, "centred"), (ARM_IN_HALF_W_MM - D.BRACKET_W_MM / 2, "at the arms' full play")):
        cur = lift_curve_v6(ss, off=off)
        mx = max(cur, key=lambda r: r[1])
        first = max((r[0] for r in cur if r[1] > 0.005), default=0.0)
        rule(first <= 12.0 and mx[1] <= 0.55,
             "backing in %s (%.1f mm off): the pads first touch the plates %.1f mm before the bumps (nothing earlier); "
             "the most lift is %.0f %%, as the crown meets the pad's back edge (%.0f mm behind the axle); docked %.0f %%"
             % (lab, off, first, 100 * mx[1], -mx[3], 100 * cur[0][1]))
    off_bad = SLOT_HALF_W_MM - PLATE_W_MM / 2 + 0.6
    cur = lift_curve_v6(ss, off=off_bad)
    rule(False, "for comparison: a robot %.1f mm off centre (more than the arms allow) would drag a crown under the "
         "carrier's plastic: up to %.0f %% lift (the lift chart's dashed line)" % (off_bad, 100 * max(r[1] for r in cur)),
         warn=True)
    # ----- sideways and along
    p_in, p_out = PAD_Y_MM - PRONG_HALF_W_MM, PAD_Y_MM + PRONG_HALF_W_MM
    play = ARM_IN_HALF_W_MM - D.BRACKET_W_MM / 2
    rule(SLOT_HALF_W_MM - PLATE_W_MM / 2 >= play + 0.4, "in the arms the robot is within +-%.1f mm; the crown has %.1f mm "
         "each side in its lane" % (play, SLOT_HALF_W_MM - PLATE_W_MM / 2))
    rule(PAD_W_MM / 2 - PLATE_W_MM / 2 >= D.POCKET_CLEAR_MM / 2 + 0.5, "docked (pocket +-%.1f): the plate lands on the "
         "pad with %.1f mm to spare each side" % (D.POCKET_CLEAR_MM / 2, PAD_W_MM / 2 - PLATE_W_MM / 2))
    x_bar_at_tip = X_PRONG_TIP_MM - (CASTER_X_MM - BAR_FACE_X_MM)
    off_tip = (inner_half_w(min(x_bar_at_tip, -POCKET_DEPTH_MM - 1.1)) - D.BRACKET_W_MM / 2
               if x_bar_at_tip >= ARM_END_X_MM else FLARE_MM + play)
    rule(off_tip + V3.CASTER_BALL_R_MM <= p_in - 2.0,
         "when the caster reaches the prongs' tips the bar is %.0f mm into the U, within +-%.1f mm: the ball misses "
         "them by %.1f mm or more" % (x_bar_at_tip - ARM_END_X_MM, off_tip, p_in - off_tip - V3.CASTER_BALL_R_MM))
    x_bar_at_crown = X_CROWN_MM - (CARRIER_X0_MM - BAR_FACE_X_MM)
    rule(inner_half_w(x_bar_at_crown) <= ARM_IN_HALF_W_MM + 1e-6,
         "when a crown reaches the carrier the bar is %.0f mm inside the straight arms (held to +-%.1f)"
         % (x_bar_at_crown - (ARM_END_X_MM + FLARE_L_MM), play))
    rule(TAB_Y_MM + TAB_W_MM / 2 <= WHEEL_INNER_Y_MM - 3.0, "prong screw tabs reach %.0f mm out (wheels from %.1f)"
         % (TAB_Y_MM + TAB_W_MM / 2, WHEEL_INNER_Y_MM))
    rule(X_PRONG_TIP_MM >= ARM_END_X_MM + 1.0, "the prongs end inside the U (plate from its hinge %.0f mm in front of the "
         "axle to the crown %.0f mm behind it)" % (X_AXLE_DOCKED_MM - X_HINGE_MM, -PAD_X_MM))
    # ----- the U
    rule(D.BRACKET_W_MM < 2 * POCKET_HALF_W_MM and 2 * POCKET_HALF_W_MM - D.BRACKET_W_MM <= 1.2,
         "pocket (last %.0f mm) %.1f mm wide for the %.1f mm bar (as v5)" % (POCKET_DEPTH_MM, 2 * POCKET_HALF_W_MM,
                                                                           D.BRACKET_W_MM))
    leg_out = L.leg_y_inner() + L.LEG_T_MM
    head_out = max(D.ARM_OUTER_Y_MM, leg_out) + D.M3_HEAD_H_MM
    rule(ARM_IN_HALF_W_MM - leg_out >= 0.8 and GROOVE_HALF_W_MM - head_out >= 1.0,
         "arms: the light plate's legs clear by %.2f; the M3 heads run in the groove (as v5)" % (ARM_IN_HALF_W_MM - leg_out))
    cap = FLARE_MM + play
    rule(cap >= 12.0, "mouth: a %.0f mm chamfer opens %.0f mm each side: catches a robot up to %.1f mm off centre "
         "(v5: 8.5; v4's funnel: 32 over 50 mm)" % (FLARE_L_MM, FLARE_MM, cap))
    near_post = side_post_xc() - D.SWITCH_LATERAL_MM
    bar_end = side_post_xc() - D.BRACKET_W_MM / 2
    rule(horn_out_mm() <= V2.POST_STANDOFF_MM - 3.0 and bar_end >= ARM_END_X_MM + FLARE_L_MM,
         "the arms' ends flare out %.1f mm past their outer faces, from %.0f mm on; a robot on a side station (bar to "
         "X %.0f, posts from %.0f) never reaches them" % (horn_out_mm(), ARM_END_X_MM + FLARE_L_MM, bar_end, near_post))
    rule(D.FACE_ABOVE_BODY_MM > D.LEVER_PRESSED_MM and D.LEVER_FREE_MM > D.FACE_ABOVE_BODY_MM,
         "each inner bump presses its back switch's lever %.1f mm and stops on the bar's face (as v5)"
         % (D.LEVER_FREE_MM - D.FACE_ABOVE_BODY_MM))
    rule(BUMP_Y_MM[0] > p_out + 1.0, "bumps (from %.0f mm out) clear of the prongs (to %.0f)" % (BUMP_Y_MM[0], p_out))
    H_cam, d_far = 262.0, 90.0
    plate_z, strip_z = (L.PLATE_UNDER_Z_MM + L.PLATE_T_MM) / 10.0, WALL_H_MM / 10.0
    shift = d_far * (plate_z / (H_cam - plate_z) - strip_z / (H_cam - strip_z)) * 10.0
    gap = STRIP_X0_MM + STRIP_CLEAR_MM - (X_AXLE_DOCKED_MM - L.PLATE_X_MM[0])
    rule(gap - shift >= 2.0, "marker: unchanged from v5, %.0f mm stay clear of the docked robot's light plate" % (gap - shift))
    with patched(D, BUMPER_Z0_MM=V3.REAR_BAR_Z0_MM, BUMPER_Z1_MM=V3.REAR_BAR_Z1_MM):
        with patched(V2, POST_H_MM=POST_H_MM, BACK_FACE_X_MM=BASE_D_MM, SIDE_FACE_Y_MM=HALF_OUT_MM,
                     SIDE_POST_XC_MM=side_post_xc()):
            ok2, l2 = V2.checks(dock)
    lines += ["     (post stations) " + l.replace("dock_v2", "dock_v6") for l in l2]
    ok = ok and ok2
    return ok, lines


def path_clear(att, dock, offs=(0.0, 1.4), steps=(0, 5, 15, 30, 50, 75, 100, 110)):
    c = B.cq()
    to_dock = lambda s, dx, dy: s.rotate((0, 0, 0), (0, 0, 1), 180).translate((X_AXLE_DOCKED_MM - dx, dy, 0))
    out = []
    for off in offs:
        for s in steps:
            o = off if s > POCKET_DEPTH_MM + 1 else min(off, D.POCKET_CLEAR_MM / 2 - 0.05)
            a = to_dock(att, s + 0.05, o)
            v = a.intersect(dock).val().Volume()
            out.append((off, s, v))
    return out


# =============================================================================== pictures
def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def docked_png(att, carrier, dock, path):
    plt = _plt()
    from matplotlib.patches import Rectangle, Circle, Polygon
    from build_attachment_v2 import _fill
    blue, green, dark, gold, grey = "#2f5d8a", "#5b8c3a", "#3a3a3a", "#b8860b", "#d9dee2"
    to_dock = lambda s: s.rotate((0, 0, 0), (0, 0, 1), 180).translate((X_AXLE_DOCKED_MM, 0, 0))
    attd, card = to_dock(att), to_dock(carrier)
    fig, axs = plt.subplots(2, 1, figsize=(12, 11.5), dpi=110, gridspec_kw={"height_ratios": [0.8, 1.6]})
    ax = axs[0]
    yp = -PAD_Y_MM
    xb = X_AXLE_DOCKED_MM + D.AXLE_TO_FRONT_MM
    ax.add_patch(Polygon([(xb, Z_UNDER_MM), (xb - 95.25, Z_UNDER_MM), (xb - 95.25, 37.2), (xb, 37.2)], fc=grey, ec="#999"))
    ax.add_patch(Circle((X_AXLE_DOCKED_MM, D.WHEEL_R_MM), D.WHEEL_R_MM, fc="#555", alpha=0.3))
    ax.add_patch(Circle((X_AXLE_DOCKED_MM - CASTER_X_MM, V3.CASTER_BALL_R_MM), V3.CASTER_BALL_R_MM, fc="#888"))
    xc0, xc1 = robot_to_dock(COVER_FRONT_X_MM, 0)[0], robot_to_dock(COVER_BACK_X_MM, 0)[0]
    ax.plot([xc0, xc1], [Z_UNDER_MM + 0.15] * 2, color="#c0392b", lw=2.2, label="battery cover (PHOTO)")
    _fill(ax, dock, "XZ", yp, 0, 2, dark)
    _fill(ax, attd, "XZ", yp, 0, 2, blue)
    _fill(ax, card, "XZ", yp, 0, 2, green)
    for poly in plate_polys(PRESS_MM):
        ax.add_patch(Polygon(poly, closed=True, fc=gold, ec=gold))
    sp = spring_line(PRESS_MM)
    ax.plot([q[0] for q in sp], [q[1] for q in sp], color="#555", lw=0.8)
    xpad = robot_to_dock(PAD_X_MM, 0)[0]
    ax.add_patch(Rectangle((xpad - PAD_L_MM / 2, Z_PAD_MM), PAD_L_MM, PAD_T_MM, fc="#d4a017"))
    ax.axhline(0, color="k", lw=0.8)
    ax.set_xlim(-125, 50)
    ax.set_ylim(-3, 42)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("from the side, cut through the + pad and plate (robot's left, dock Y = %.0f)" % yp, fontsize=10)
    ax.set_xlabel("mm into the dock from the bottom of the U")
    ax.set_ylabel("mm above the plywood")
    ax = axs[1]
    ax.add_patch(Rectangle((xb - 95.25, -48), 95.25, 96, fc=grey, ec="#999"))
    for sg in (1, -1):
        ax.add_patch(Rectangle((X_AXLE_DOCKED_MM - V3.TYRE_X_MM, sg * WHEEL_INNER_Y_MM + (0 if sg > 0 else -7)),
                               2 * V3.TYRE_X_MM, 7, fc="#555", alpha=0.5))
    cov = [robot_to_dock(x, y) for x, y in cover_outline()]
    ax.add_patch(Polygon(cov, closed=True, fc="none", ec="#c0392b", lw=1.4, ls="--"))
    sx, sy = robot_to_dock(COVER_SCREW_X_MM, 0)
    ax.add_patch(Circle((sx, sy), SCREW_HEAD_R_MM, fc="#999", ec="#555"))
    ax.add_patch(Circle((X_AXLE_DOCKED_MM - CASTER_X_MM, 0), V3.CASTER_BALL_R_MM, fc="#777"))
    _fill(ax, card, "XY", Z_PAD_MM + 0.5, 0, 1, green, alpha=0.75)
    _fill(ax, dock, "XY", 2.0, 0, 1, dark, alpha=0.9)
    _fill(ax, attd, "XY", V3.REAR_BAR_Z0_MM + 0.5, 0, 1, blue, alpha=0.9)
    for sg in (1, -1):
        y = sg * PAD_Y_MM
        ax.add_patch(Rectangle((X_HINGE_MM - PLATE_T_MM, y - PLATE_W_MM / 2), X_CROWN_MM - X_HINGE_MM + PLATE_T_MM + 1.3,
                               PLATE_W_MM, fc=gold, alpha=0.95))
        for so in (1, -1):
            ax.add_patch(Circle((X_SPRING_MM, y + so * SPRING_Y_OFF_MM), SPRING_OD_MM / 2, fc="none", ec="#222", lw=0.8))
        ax.text(X_HINGE_MM - 4, y, "+ plate" if sg < 0 else "- plate", fontsize=8, ha="right", va="center")
    ax.text(sx + 3, 4, "cover screw", fontsize=7.5)
    ax.text(X_AXLE_DOCKED_MM - CASTER_X_MM, 11, "caster", fontsize=8, ha="center")
    ax.text(X_AXLE_DOCKED_MM, 33, "wheel", fontsize=8, ha="center")
    ax.text(robot_to_dock(COVER_BACK_X_MM, 0)[0] - 1, -COVER_HALF_W_MM - 4, "battery cover (dashed red)",
            fontsize=8, ha="right", color="#c0392b")
    ax.set_xlim(-125, 60)
    ax.set_ylim(-80, 80)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.set_title("from above: the dock cut 2 mm up (black), the pad carrier (green) on the battery cover (red), the rear "
                 "bar (blue), the plates (gold) with their two springs", fontsize=9.5)
    ax.set_xlabel("mm into the dock from the bottom of the U")
    ax.set_ylabel("mm left of the dock's axis")
    fig.text(0.5, 0.012, "Drawn from the CAD and the cover measured on Nick's photo: a robot (grey) backed into dock v6 "
             "(black). The pads sit on the battery cover, behind the wheels and in front of the caster, clear of the "
             "cover's screw; each gold plate, hinged at its front and held up by two springs, is pressed down by its pad.",
             ha="center", fontsize=9.5, wrap=True)
    fig.tight_layout(rect=(0, 0.045, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def contact_png(dock, path):
    plt = _plt()
    from matplotlib.patches import Polygon, Rectangle
    from build_attachment_v2 import _fill
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12.5, 4.6), dpi=115, gridspec_kw={"width_ratios": [2.6, 1]})
    gold, dark = "#b8860b", "#3a3a3a"
    _fill(ax, dock, "XZ", -PAD_Y_MM - SPRING_Y_OFF_MM, 0, 2, dark, alpha=0.85)
    for d, a, lab in ((0.0, 0.4, "free"), (PRESS_MM, 1.0, "docked (crown pressed %.1f mm)" % PRESS_MM)):
        for i, poly in enumerate(plate_polys(d)):
            ax.add_patch(Polygon(poly, closed=True, fc=gold, ec=gold, alpha=a, label=lab if i == 0 else None))
        sp = spring_line(d)
        ax.plot([q[0] for q in sp], [q[1] for q in sp], color="#555", lw=0.9, alpha=a)
    ax.axhline(Z_PAD_MM, color="#5b8c3a", lw=1.2)
    ax.text(X_CROWN_MM + 5, Z_PAD_MM + 0.2, "pad face %.1f mm" % Z_PAD_MM, fontsize=8, color="#5b8c3a")
    ax.axhline(Z_UNDER_MM, color="#888", lw=1.0, ls="--")
    ax.text(X_CROWN_MM + 5, Z_UNDER_MM + 0.2, "battery cover %.1f mm (the lane's roof)" % Z_UNDER_MM, fontsize=8,
            color="#666")
    ax.annotate("", (X_CROWN_MM + 22, 12.4), (X_CROWN_MM + 5, 12.4), arrowprops=dict(arrowstyle="->", lw=1.0,
                                                                                     color="#2f5d8a"))
    ax.text(X_CROWN_MM + 13, 12.8, "robot backs in this way", color="#2f5d8a", fontsize=8, ha="center")
    ax.set_xlim(X_PRONG_TIP_MM - 2, X_CROWN_MM + 28)
    ax.set_ylim(-1, 15)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.legend(loc="lower right", fontsize=7.5)
    ax.set_xlabel("mm into the dock from the bottom of the U")
    ax.set_ylabel("mm above the plywood")
    ax.set_title("From the side, cut through one spring", fontsize=10)
    # end on, through the springs: the prong's section, the two pockets and springs, the plate across them
    _fill(ax2, dock, "YZ", X_SPRING_MM, 1, 2, dark, alpha=0.85)
    yc = -PAD_Y_MM
    top = spring_top_z(PRESS_MM)
    for so in (1, -1):
        sp = spring_line(PRESS_MM, x=yc + so * SPRING_Y_OFF_MM)
        ax2.plot([q[0] for q in sp], [q[1] for q in sp], color="#555", lw=0.9)
    ax2.add_patch(Rectangle((yc - PLATE_W_MM / 2, top), PLATE_W_MM, PLATE_T_MM, fc=gold))
    ax2.set_xlim(yc - 9, yc + 9)
    ax2.set_ylim(-1, 15)
    ax2.set_aspect("equal")
    ax2.grid(True, lw=0.3)
    ax2.set_xlabel("mm left of the dock's axis")
    ax2.set_title("End on, through the springs (docked)", fontsize=10)
    fig.suptitle("The contact on one prong: a stiff brass plate (%.1f x %.2f mm) hinged at the front, on two springs at "
                 "the rear" % (PLATE_T_MM, PLATE_W_MM), fontsize=10.5)
    fig.text(0.5, 0.02, "Cut through the prong (dark) from the CAD: the plate rests on a spring under each edge, so it "
             "springs back level instead of rocking; light gold is the plate free, dark gold where the robot's pad holds it once docked.",
             ha="center", fontsize=9, wrap=True)
    fig.tight_layout(rect=(0, 0.06, 1, 0.95))
    fig.savefig(path)
    plt.close(fig)


def lift_png(path):
    plt = _plt()
    ss = [i * 0.25 for i in range(0, 241)]
    fig, ax = plt.subplots(figsize=(9.5, 5.0), dpi=115)
    v5 = V5.lift_curve_v5(ss)
    ax.plot([r[0] for r in v5], [100 * r[1] for r in v5], color="#7f8c8d", lw=1.8, label="v5: pads 8 mm behind the axle "
            "(on a sled), one spring")
    for dz, ls, lab in ((0.0, "-", "v6: pads on the battery cover, %.0f mm behind the axle, two springs" % -PAD_X_MM),
                        (-0.4, ":", "v6, robot sitting 0.4 mm lower than measured")):
        cur = lift_curve_v6(ss, dz=dz)
        ax.plot([r[0] for r in cur], [100 * r[1] for r in cur], color="#2f5d8a", lw=2.0, ls=ls, label=lab)
    off_bad = SLOT_HALF_W_MM - PLATE_W_MM / 2 + 0.6
    cur = lift_curve_v6(ss, off=off_bad)
    ax.plot([r[0] for r in cur], [100 * r[1] for r in cur], color="#c0392b", lw=1.2, ls="--",
            label="v6 if the robot came in %.1f mm off centre (the arms stop that)" % off_bad)
    ax.set_xlim(40, 0)
    ax.set_ylim(0, 128)
    ax.set_yticks(range(0, 101, 20))
    ax.set_xlabel("mm the robot still has to back in before its switches touch the bumps")
    ax.set_ylabel("% of the caster's load taken off")
    ax.set_title("Lift on the caster while a robot backs in (both contacts together)", fontsize=11)
    ax.grid(True, lw=0.3)
    ax.legend(fontsize=8, loc="upper left")
    fig.text(0.5, 0.015, "Computed from the CAD numbers and an ASSUMED 0.6 N on the caster (Appendix B step 0b weighs "
             "it): how much of the caster's load the contacts take off at each point of the back-in; at 100 % the "
             "caster would leave the floor.", ha="center", fontsize=8.5, wrap=True)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def carrier_on_photo_png(path):
    """The cover's seam and the carrier, pads, lanes and channels drawn over Nick's underside photo."""
    plt = _plt()
    from PIL import Image
    im = Image.open(PHOTO).convert("RGB")
    fig, ax = plt.subplots(figsize=(7.2, 7.6), dpi=110)
    ax.imshow(im)

    def poly(pts, **kw):
        q = [photo_px(x, y) for x, y in pts]
        q.append(q[0])
        ax.plot([p[0] for p in q], [p[1] for p in q], **kw)

    poly(carrier_outline(), color="#2e7d32", lw=1.8, label="pad carrier")
    for sg in (1, -1):
        y = sg * PAD_Y_MM
        poly([(PAD_X0_MM, y - PAD_W_MM / 2), (PAD_X1_MM, y - PAD_W_MM / 2), (PAD_X1_MM, y + PAD_W_MM / 2),
              (PAD_X0_MM, y + PAD_W_MM / 2)], color="#d4a017", lw=2.2, label="brass pad" if sg > 0 else None)
        poly([(CARRIER_X0_MM, y - SLOT_HALF_W_MM), (RAMP_X0_MM, y - SLOT_HALF_W_MM), (RAMP_X0_MM, y + SLOT_HALF_W_MM),
              (CARRIER_X0_MM, y + SLOT_HALF_W_MM)], color="#1565c0", lw=1.0, ls="--",
             label="lane the crown runs in" if sg > 0 else None)
        u, v = photo_px(PAD_X_MM, y)
        ax.plot([u], [v], "x", color="#c62828", ms=9, mew=2, label="crown, docked" if sg > 0 else None)
        q = [photo_px(PAD_X_MM, y), photo_px(PAD_X_MM, sg * CH_Y_MM), photo_px(CARRIER_X0_MM, sg * CH_Y_MM),
             photo_px(V3.WIRE_HOLE_X_MM, sg * WIRE_HOLE_Y_MM)]
        ax.plot([p[0] for p in q], [p[1] for p in q], color="#e53935", lw=1.2,
                label="pad wire (to the bar's hole)" if sg > 0 else None)
    u, v = photo_px(0, 0)
    ax.plot([u], [v], "+", color="k", ms=10)
    ax.text(u + 10, v - 8, "axle middle", fontsize=7)
    ax.set_xlim(60, 1150)
    ax.set_ylim(1420, 560)
    ax.axis("off")
    ax.legend(loc="lower left", fontsize=7, framealpha=0.85)
    ax.set_title("The pad carrier drawn on Nick's photo of the underside (front at the top)", fontsize=9.5)
    fig.text(0.5, 0.01, "Measured on the photo at 11.06 px/mm (about +-1.5 mm): the carrier sits inside the battery "
             "cover's seams, clear of the screw; the robot's left is on the photo's right (seen from below).",
             ha="center", fontsize=8, wrap=True)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
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
    att = V5.attachment_v5_solid()
    carrier = carrier_v6_solid()
    dock = dock_v6_solid()
    ok, lines = checks(att, carrier, dock)
    print("dock v6, pad carrier v6 (attachment v5 and marker strip v5 unchanged):")
    print("\n".join(lines))
    if "--no-path" not in argv:
        res = path_clear(att, dock)
        bad = [(o, s, v) for o, s, v in res if v > 0.5]
        print(("ok   " if not bad else "FAIL ") + "backing in (8 points from 110 mm out to docked, centred and 1.4 mm "
              "off): the attachment never runs into the U%s" % ("" if not bad else ": %s" % bad))
        ok = ok and not bad
    export(on_bed(dock), "dock_v6.stl")
    print("ok   dock_v6: print bottom down, brim on; %.0f mm^2 faces down (cavity roof, wire grooves, screw-head "
          "grooves' roofs: short bridges, as v5)" % B.overhang_mm2(on_bed(dock)))
    c_bed = on_bed(carrier, flip=True)
    print("ok   pad carrier v6: print its top face down (as exported), no supports; %.0f mm^2 faces down"
          % B.overhang_mm2(c_bed))
    export(c_bed, "pad_carrier_v6.stl")
    B.preview("dock_v6", dock, os.path.join(B.PNG, "dock_v6.png"), 2.0)
    B.preview("pad_carrier_v6", carrier, os.path.join(B.PNG, "carrier_v6.png"), plan_z=Z_PAD_MM + 0.4)
    docked_png(att, carrier, dock, os.path.join(B.PNG, "docked_v6.png"))
    contact_png(dock, os.path.join(B.PNG, "contact_v6.png"))
    lift_png(os.path.join(B.PNG, "lift_v6.png"))
    if os.path.exists(PHOTO):
        carrier_on_photo_png(os.path.join(B.PNG, "carrier_on_photo_v6.png"))
    print("\nwrote %s/{dock_v6, pad_carrier_v6}.stl (attachment_v5.stl, marker_strip_v5.stl: build_dock_v5.py)" % B.STL)
    print("      %s/{dock_v6, carrier_v6, docked_v6, contact_v6, lift_v6, carrier_on_photo_v6}.png" % B.PNG)
    print("\nfirmware and map numbers (unchanged from v5):")
    for l in V3.firmware_lines():
        print("  " + l)
    print("  crowns at dock X %.1f, Y +-%.1f; prong tips at X %.1f; mouth at X %.0f" % (X_CROWN_MM, PAD_Y_MM,
                                                                                     X_PRONG_TIP_MM, ARM_END_X_MM))
    print("\nknown lengths: printed sizes, measure the print to check")
    for name, v, how in known_lengths():
        print("  %-52s %7.2f mm   %s" % (name, v, how))
    if not ok:
        print("\nSOME CHECKS FAILED")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
