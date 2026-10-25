"""Alvik attachment version 2 (switches + camera lights + charging pads, one printed part) and dock version 3
(dock version 2 + two spring contacts in the back wall).

    python hardware/dock/build_attachment_v2.py

writes
    stl/attachment_v2.stl          the attachment: build_attachment.py's part plus two charging pads in the bar
    stl/dock_v3.stl                build_v2.py's dock plus two spring (pogo) pins in the back wall, a wiring
                                   cavity behind it and a cable channel under the head
    stl/attachment_v2_fit_test.stl a 13 mm slice of the attachment (bar, lower arms, low screw holes, pads):
                                   print it first (about 20 min) and try it on the robot
    stl/pogo_gauge.stl             five test holes for the spring pins, to find the hole size your printer needs
    preview/attachment_v2.png, dock_v3.png, charging_contact.png, nose_bracket_size.png
and prints a printability / fit report like build.py's (ok / FAIL / WARN lines).

The plan this goes with: docs/build-plans/self-charging-dock.md.

What changes from version 1 (build_attachment.py), and what does not
  - The bracket's outside is identical: same width (the dock pocket still fits it), same face, same switches. So
    the firmware's bump_ahead_cm / bump_half_cm / sw_gap_cm / bumper_cm and every dock already printed stay valid.
  - Two charging pads in the bar's centre section, between the switches: the heads of two brass M3 countersunk
    screws, sitting PAD_RECESS_MM below the face (the face stays the hard stop and the touch-probe surface).
    Behind each, a nut slot open on top takes an M3 nut and the wire's ring terminal. A groove along the bar's top
    carries the two wires to the switch pockets, where they go up the robot's front corner with the switch wires
    (never across the dark recess, where the distance sensor looks out).
  - LEFT pad (robot's left) = + (VBUS), RIGHT pad = - (GND). A small dimple beside the + pad marks it.
  - The springs are in the dock, not on the robot: the nose is also the touch probe and bumps boards, which would
    bend pins sticking out of it; flat pads on the robot don't care.

Dock version 3 = dock version 2 (build_v2.py, unchanged: funnel, posts, tile, screw holes) plus
  - two stepped holes through the back wall, at the pads' height and spacing, for spring pins pressed in from
    behind until they stop on the step. The GND pin stands further out than the + pin, so it touches first;
  - a wiring cavity behind the back wall, open underneath (wire and solder from below; the dock sits on the
    plywood), under the check tile with a solid roof;
  - a cable channel under the head from the cavity to the back face (for a dock at the board's edge), and the
    cavity's open bottom over a hole drilled in the plywood (for a dock away from the edge).

Every number is marked SOURCED / MEASURED / ASSUMED (as dock_params.py) or CHOSEN (a design choice).
"""
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

# =============================================================================== charging pads (robot side)
PAD_Y_MM = 10.0            # CHOSEN: each pad this far from the centre line (20 mm apart: a dropped screwdriver or
                           # a coin is unlikely to bridge them; well clear of the switch pockets at 36-49 mm)
PAD_Z_MM = 10.0            # CHOSEN: pad centre above the floor = the bar's middle (bar 5-15 mm, dock_params)
PAD_HEAD_DIA_MM = 6.0      # SOURCED (ISO 10642 M3 countersunk head, 6.0 nominal; DIN 965 is 5.5: either fits)
PAD_HEAD_H_MM = 1.4        # derived: 90 deg countersink from 6.0 to the 3.2 shank hole = 1.4 mm deep
PAD_RECESS_MM = 0.3        # CHOSEN: head top this far behind the bracket's face (the face stays the hard stop)
PAD_SCREW_LEN_MM = 8.0     # CHOSEN: M3 x 8 countersunk (length includes the head)
M3_SHANK_HOLE_MM = 3.2     # clearance for the screw's thread
M3_NUT_AF_MM = 5.5         # SOURCED (ISO 4032 M3 nut: 5.5 across flats, 2.4 thick)
M3_NUT_T_MM = 2.4
RING_T_MM = 0.8            # ASSUMED: an M3 ring terminal's thickness
NUT_SLOT_CLEAR_MM = 0.3
GROOVE_W_MM = 2.6          # wire groove along the bar's top (two 26 AWG wires side by side)
GROOVE_D_MM = 2.0
MARK_DIA_MM = 1.6          # the + mark: a dimple beside the + pad (on the face, below its plane: harmless)
MARK_Y_MM = 16.5

# =============================================================================== spring pins (dock side)
# A through-hole spring-loaded contact ("pogo pin") with a solder tail. Not bought yet: every number ASSUMED from
# typical listings; measure yours (pogo_gauge.stl, the check list in the plan) and change them here.
POGO_BARREL_DIA_MM = 3.0   # ASSUMED: the fixed outer sleeve
POGO_PLUNGER_DIA_MM = 2.0  # ASSUMED: the moving tip
POGO_FREE_MM = 2.0         # ASSUMED: how far the tip stands out of the barrel's front end, not pressed
POGO_STROKE_MM = 1.5       # ASSUMED: how far the tip can be pushed in
POGO_BARREL_LEN_MM = 6.0   # ASSUMED: barrel length (the solder tail sticks out behind it)
POGO_HOLE_PRESS_MM = 0.05  # CHOSEN: barrel hole this much over the barrel (snug; a drop of glue holds it). Use the
                           # gauge's best hole instead once printed
POGO_FORCE_N = 0.6         # ASSUMED: spring force at mid stroke, per pin (measure: bench test 4)
STEP_VBUS_MM = 1.0         # CHOSEN: the step in front of each barrel (the barrel stops on it): + pin stands out
STEP_GND_MM = 0.7          #   free - step = 1.0 mm, GND pin 1.3 mm (GND touches first, lets go last)
CAVITY_X_MM = (6.0, 36.0)  # CHOSEN: wiring cavity behind the back wall, dock X (open underneath)
CAVITY_HALF_Y_MM = 22.0
CAVITY_TOP_Z_MM = 16.0     # roof up to the check tile's recess (21.0): 5 mm solid
CHANNEL_HALF_W_MM = 3.0    # cable channel under the head, cavity to back face (6 mm: a USB cable is 3-4.5 mm)
CHANNEL_H_MM = 5.0

DOCKED_SHIFT_MM = D.AXLE_TO_FRONT_MM + D.FACE_X_MM   # robot (axle) X -> dock X when the face is flush on the wall


# =============================================================================== the attachment
def attachment_v2_solid():
    """In the robot's axle frame (X forward from the wheel axle, Y left, Z up from the floor), as build_attachment."""
    c = B.cq()
    body = A.attachment_solid()
    xf = D.AXLE_TO_FRONT_MM + D.FACE_X_MM                  # bracket face
    x_head_top = xf - PAD_RECESS_MM
    x_cone_end = x_head_top - PAD_HEAD_H_MM
    x_tip = x_head_top - PAD_SCREW_LEN_MM
    x_slot1 = x_cone_end - 2.5                               # 2.5 mm of wall between the countersink and the slot
    x_slot0 = x_slot1 - (M3_NUT_T_MM + RING_T_MM + NUT_SLOT_CLEAR_MM)
    nut_corners = M3_NUT_AF_MM / math.cos(math.radians(30))
    for sg in (1, -1):
        y = sg * PAD_Y_MM
        # counterbore (the recess) and countersink
        body = body.cut(B.cyl(PAD_HEAD_DIA_MM / 2 + 0.15, PAD_RECESS_MM + 1.0, (xf + 1.0, y, PAD_Z_MM), (-1, 0, 0)))
        cone = c.Solid.makeCone(PAD_HEAD_DIA_MM / 2 + 0.15, M3_SHANK_HOLE_MM / 2, PAD_HEAD_H_MM + 0.01,
                                c.Vector(x_head_top + 0.005, y, PAD_Z_MM), c.Vector(-1, 0, 0))
        body = body.cut(c.Workplane("XY").add(cone))
        # shank hole, front to just past the screw tip (not into the robot)
        body = body.cut(B.cyl(M3_SHANK_HOLE_MM / 2, x_head_top - (x_tip - 0.6), (x_head_top, y, PAD_Z_MM), (-1, 0, 0)))
        # nut slot, open on top: the nut stands on a flat, centred on the screw
        z0 = PAD_Z_MM - M3_NUT_AF_MM / 2 - 0.1
        slot = (c.Workplane("XY").workplane(offset=z0).center((x_slot0 + x_slot1) / 2, y)
                .box(x_slot1 - x_slot0, nut_corners + NUT_SLOT_CLEAR_MM, D.BUMPER_Z1_MM + 1 - z0, centered=(True, True, False)))
        body = body.cut(slot)
    # wire groove along the bar's top, from switch pocket to switch pocket, over the nut slots
    xg = (x_slot0 + x_slot1) / 2
    yg = D.SWITCH_Y_OUTER_MM - D.SWITCH_L_MM          # the switch pockets' inner ends (approximately)
    groove = (c.Workplane("XY").workplane(offset=D.BUMPER_Z1_MM - GROOVE_D_MM).center(xg, 0)
              .box(GROOVE_W_MM, 2 * yg, GROOVE_D_MM + 1, centered=(True, True, False)))
    body = body.cut(groove)
    # the + mark beside the left (+) pad
    body = body.cut(B.cyl(MARK_DIA_MM / 2, 0.8 + 1.0, (xf + 1.0, MARK_Y_MM, PAD_Z_MM), (-1, 0, 0)))
    geo = dict(xf=xf, x_head_top=x_head_top, x_cone_end=x_cone_end, x_tip=x_tip, x_slot0=x_slot0, x_slot1=x_slot1,
               xg=xg, yg=yg)
    return body, geo


# =============================================================================== the dock
def pogo_geo():
    """Per pin: (name, y, step, protrusion when free, compression when the face is flush, ... at the switch click)."""
    out = []
    for name, sg, step in (("+ (VBUS)", 1, STEP_VBUS_MM), ("- (GND)", -1, STEP_GND_MM)):
        prot = POGO_FREE_MM - step
        comp_flush = prot - PAD_RECESS_MM
        comp_click = prot - PAD_RECESS_MM - D.SW_GAP_MM
        out.append((name, sg * PAD_Y_MM, step, prot, comp_flush, comp_click))
    return out


def dock_v3_solid():
    """Dock coordinates (dock_params): X into the dock from the back wall's inner face, Y left, Z up."""
    c = B.cq()
    body = V2.dock_v2_solid()
    for name, y, step, prot, _, _ in pogo_geo():
        # face bore for the plunger (the step), then the barrel's bore back to the cavity
        body = body.cut(B.cyl(POGO_PLUNGER_DIA_MM / 2 + 0.2, step + 1.0, (-1.0, y, PAD_Z_MM), (1, 0, 0)))
        body = body.cut(B.cyl(POGO_BARREL_DIA_MM / 2 + POGO_HOLE_PRESS_MM, CAVITY_X_MM[0] - step + 0.5,
                              (step, y, PAD_Z_MM), (1, 0, 0)))
    cav = (c.Workplane("XY").workplane(offset=-1.0).center(sum(CAVITY_X_MM) / 2, 0)
           .box(CAVITY_X_MM[1] - CAVITY_X_MM[0], 2 * CAVITY_HALF_Y_MM, CAVITY_TOP_Z_MM + 1.0, centered=(True, True, False)))
    body = body.cut(cav)
    ch = (c.Workplane("XY").workplane(offset=-1.0).center((CAVITY_X_MM[1] + V2.HEAD_DEPTH_MM + 2) / 2, 0)
          .box(V2.HEAD_DEPTH_MM + 2 - CAVITY_X_MM[1] + 1, 2 * CHANNEL_HALF_W_MM, CHANNEL_H_MM + 1.0, centered=(True, True, False)))
    body = body.cut(ch)
    # the + mark on the top, above the + pin
    body = body.cut(c.Workplane("XY").workplane(offset=D.WALL_H_MM - 0.8).center(3.0, PAD_Y_MM).circle(MARK_DIA_MM / 2)
                    .extrude(2))
    return body


def pogo_gauge_solid():
    """A block with five barrel holes (horizontal, like the dock's), each with the plunger step in front."""
    c = B.cq()
    sizes = [POGO_BARREL_DIA_MM + d for d in (-0.2, -0.1, 0.0, 0.1, 0.2)]
    pitch = 9.0
    w = pitch * len(sizes) + 4
    blk = c.Workplane("XY").box(10.0, w, PAD_Z_MM + 5, centered=(False, True, False))
    for i, d in enumerate(sizes):
        y = -w / 2 + 2 + pitch * (i + 0.5)
        blk = blk.cut(B.cyl(POGO_PLUNGER_DIA_MM / 2 + 0.2, 2.0, (-1.0, y, PAD_Z_MM), (1, 0, 0)))
        blk = blk.cut(B.cyl(d / 2, 12.0, (STEP_VBUS_MM, y, PAD_Z_MM), (1, 0, 0)))
    # notch on the top at the smallest hole's end
    blk = blk.cut(c.Workplane("XY").workplane(offset=PAD_Z_MM + 4).center(5.0, -w / 2 + 1.0).box(10.0, 2.0, 2.0))
    return blk, sizes


# =============================================================================== checks
def checks(att, geo, dock):
    c = B.cq()
    lines, ok = [], True

    def rule(good, text, warn=False):
        nonlocal ok
        if not good and not warn:
            ok = False
        lines.append(("ok   " if good else ("WARN " if warn else "FAIL ")) + text)

    v1 = A.attachment_solid()
    b1, b2 = B.bbox(v1), B.bbox(att)
    same = all(abs(p - q) < 0.01 for p, q in zip(b1, b2))
    rule(same, "attachment_v2 has version 1's outside (%.1f x %.1f x %.1f mm): the dock pocket and the firmware's "
         "bump_* / bumper_cm numbers stay as they are" % (b2[3] - b2[0], b2[4] - b2[1], b2[5] - b2[2]))
    rule(att.val().isValid() and len(att.val().Solids()) == 1, "attachment_v2: one valid solid")
    xf = geo["xf"]
    rule(geo["x_tip"] - 0.6 > D.AXLE_TO_FRONT_MM + D.BACKPLATE_T_MM,
         "pad screw (M3 x %.0f countersunk) ends %.1f mm in front of the robot's face; it never touches the robot"
         % (PAD_SCREW_LEN_MM, geo["x_tip"] - D.AXLE_TO_FRONT_MM))
    nut_corners = M3_NUT_AF_MM / math.cos(math.radians(30))
    rule(geo["x_tip"] < geo["x_slot0"] - 0.2, "the screw passes right through its nut (tip %.1f mm behind the nut)"
         % (geo["x_slot0"] - geo["x_tip"]))
    rule(geo["x_cone_end"] - geo["x_slot1"] >= D.MIN_WALL_MM, "%.1f mm of bar between each countersink and its nut "
         "slot (>= %.1f): the screw clamps the ring terminal against it" % (geo["x_cone_end"] - geo["x_slot1"], D.MIN_WALL_MM))
    head_r = PAD_HEAD_DIA_MM / 2 + 0.15
    rule(PAD_Z_MM - head_r >= D.BUMPER_Z0_MM + 1.0 and PAD_Z_MM + head_r <= D.BUMPER_Z1_MM - 1.0,
         "pad heads (%.1f mm) inside the bar's height (%.0f-%.0f mm) with 1 mm or more to spare"
         % (2 * head_r, D.BUMPER_Z0_MM, D.BUMPER_Z1_MM))
    sw_inner = D.SWITCH_Y_OUTER_MM - D.SWITCH_L_MM - 0.6
    rule(PAD_Y_MM + nut_corners / 2 + NUT_SLOT_CLEAR_MM + D.MIN_WALL_MM <= sw_inner,
         "nut slots (to %.1f mm from the centre) stay clear of the switch pockets (from %.1f mm)"
         % (PAD_Y_MM + nut_corners / 2 + NUT_SLOT_CLEAR_MM, sw_inner))
    rule(PAD_RECESS_MM > 0, "pad heads %.1f mm below the face: the face is still the hard stop and the touch-probe "
         "surface" % PAD_RECESS_MM)
    # the dock
    rule(dock.val().isValid() and len(dock.val().Solids()) == 1, "dock_v3: one valid solid")
    ok2, l2 = V2.checks(dock)
    lines += ["     (dock_v2's own checks on dock_v3) " + l for l in l2]
    ok = ok and ok2
    tile_bottom = D.WALL_H_MM - D.TILE_T_MM + 0.2
    rule(tile_bottom - CAVITY_TOP_Z_MM >= 3.0, "cavity roof %.1f mm thick under the check tile's recess"
         % (tile_bottom - CAVITY_TOP_Z_MM))
    rule(CAVITY_HALF_Y_MM + 3 < min(abs(y) for _, y in V2.SCREW_HOLES) - V2.SCREW_CSK_DIA_MM / 2,
         "cavity clear of the head's screw holes")
    for name, y, step, prot, cf, cc in pogo_geo():
        grip = CAVITY_X_MM[0] - step
        rule(grip >= 4.0, "%s pin: barrel held over %.1f mm of wall (4 or more)" % (name, grip))
        rule(0.3 * POGO_STROKE_MM <= cf <= 0.9 * POGO_STROKE_MM,
             "%s pin: stands %.1f mm out of the wall; pressed %.1f mm (%.0f%% of its %.1f mm stroke) when the face "
             "is flush: firm, never bottomed" % (name, prot, cf, 100 * cf / POGO_STROKE_MM, POGO_STROKE_MM))
        rule(cc > 0.1, "%s pin: already touching (pressed %.1f mm) when the switches click %.1f mm before flush, so "
             "'both switches clicked' means 'contact made'" % (name, cc, D.SW_GAP_MM))
    g = pogo_geo()
    rule(g[1][3] > g[0][3], "the GND pin stands %.1f mm further out: it touches first and lets go last"
         % (g[1][3] - g[0][3]))
    # sideways / height tolerance: the pin tip must land on the pad's flat head
    play = D.POCKET_CLEAR_MM / 2
    allow = PAD_HEAD_DIA_MM / 2 - POGO_PLUNGER_DIA_MM / 4
    rule(allow >= 2 * (play + 0.5), "pin lands on the pad with up to %.1f mm off-centre; docked, the robot can be "
         "%.1f mm off sideways (pocket play) and about 0.5 mm in height (ASSUMED: floor, tyres): %.1fx margin"
         % (allow, play, allow / (play + 0.5)))
    tilt = math.degrees(math.atan2(D.POCKET_CLEAR_MM, D.BRACKET_W_MM))
    rule(PAD_Y_MM * math.sin(math.radians(tilt)) < 0.3 * POGO_STROKE_MM,
         "a robot seated crooked within the pocket's play (%.2f deg) moves a pad %.2f mm along the pin: the springs "
         "take it up" % (tilt, PAD_Y_MM * math.sin(math.radians(tilt))))
    push = 2 * POGO_FORCE_N
    rule(push < 3.0, "the two springs push the robot back with about %.1f N (%.0f g, ASSUMED spring force): bench test 4 "
         "checks the braked wheels hold it" % (push, push / 9.81 * 1000), warn=push >= 1.5)
    # the size question: is the printed nose bracket too small?
    return ok, lines


def size_lines():
    inner_lo = 2 * (D.side_half_w(D.BUMPER_Z0_MM) + D.ARM_CLEAR_MM)
    inner_hi = 2 * (D.side_half_w(D.ARM_TOP_Z_MM) + D.ARM_CLEAR_MM)
    return [
        "robot: %.1f wide x %.2f long x %.1f tall (SOURCED / MEASURED Oct 19: 3 3/4 in square, 1 15/32 in tall)"
        % (D.ROBOT_WIDTH_MM, D.ROBOT_LENGTH_MM, D.ROBOT_TOP_Z_MM),
        "nose bracket: %.1f wide (robot + 3.0 mm arm + 0.15 mm gap each side), %.1f front to back (%.1f in front of "
        "the face, %.1f back along each side), bar %.0f mm tall, arms up to %.1f mm"
        % (D.BRACKET_W_MM, D.BRACKET_LEN_MM, D.FACE_X_MM, -D.ARM_BACK_X_MM, D.BUMPER_Z1_MM - D.BUMPER_Z0_MM,
           D.ARM_TOP_Z_MM),
        "between the arms: %.1f mm at the bar's bottom, %.1f at the arms' top (the robot's sides lean out 1.1 deg); "
        "measured robot 95.25 low on the shell: a snug fit" % (inner_lo, inner_hi),
        "so it is the right size for what it is (a bumper wrapping the front 25 mm of the robot); it looks small "
        "because it is a 10 mm bar. attachment_v2 (with the light plate) is %.0f x %.0f x %.0f mm and covers the robot"
        % (112, D.BRACKET_W_MM + 1.1, 58),
    ]


# =============================================================================== pictures
def _section(shape, plane, at, big=500):
    import numpy as np
    c = B.cq()
    slab = {"XY": c.Solid.makeBox(big, big, 0.02, c.Vector(-big / 2, -big / 2, at)),
            "XZ": c.Solid.makeBox(big, 0.02, big, c.Vector(-big / 2, at, -big / 2)),
            "YZ": c.Solid.makeBox(0.02, big, big, c.Vector(at, -big / 2, -big / 2))}[plane]
    cut = shape.val().intersect(slab)
    verts, tris = cut.tessellate(0.05, 0.1)
    return np.array([(v.x, v.y, v.z) for v in verts]), tris


def _fill(ax, shape, plane, at, i, j, color, shift=(0, 0, 0), alpha=1.0):
    from matplotlib.patches import Polygon
    import numpy as np
    V, T = _section(shape, plane, at)
    if len(V) == 0:
        return
    V = V + np.array(shift)
    for t in T:
        ax.add_patch(Polygon(V[list(t)][:, [i, j]], closed=True, fc=color, ec=color, lw=0.2, alpha=alpha))


def contact_png(att, dock, path):
    """Top and side sections through the docked contact: the attachment's bar, the pad screws, the dock's back wall
    and its spring pins (drawn: they are bought parts, not printed)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    blue, dark, gold, red, grey = "#2f5d8a", "#3a3a3a", "#b8860b", "#c0392b", "#d9dee2"
    sh = (-DOCKED_SHIFT_MM, 0, 0)
    fig, axs = plt.subplots(1, 2, figsize=(13, 5.6), dpi=120)
    # top view at the pads' height
    ax = axs[0]
    ax.add_patch(Rectangle((-DOCKED_SHIFT_MM - 47.6, -48), 95.25, 96, fc=grey, ec="#999"))
    _fill(ax, att, "XY", PAD_Z_MM, 0, 1, blue, sh)
    _fill(ax, dock, "XY", PAD_Z_MM, 0, 1, dark)
    for name, y, step, prot, cf, cc in pogo_geo():
        x_tip = -(prot - cf)          # flush: the tip is pushed back to the pad
        ax.add_patch(Rectangle((step, y - POGO_BARREL_DIA_MM / 2), POGO_BARREL_LEN_MM, POGO_BARREL_DIA_MM, fc=gold))
        ax.add_patch(Rectangle((x_tip, y - POGO_PLUNGER_DIA_MM / 2), step - x_tip, POGO_PLUNGER_DIA_MM, fc=gold))
        ax.text(POGO_BARREL_LEN_MM + 3.5, y, name + " pin", va="center", fontsize=8, color="#222")
    ax.text(sum(CAVITY_X_MM) / 2, 0, "wiring\ncavity", ha="center", va="center", fontsize=8, color="#555")
    for sg in (1, -1):
        ax.add_patch(Rectangle((-D.FACE_X_MM + D.BACKPLATE_T_MM + D.SWITCH_TERMINAL_MM,
                                sg * D.SWITCH_LATERAL_MM - D.SWITCH_L_MM / 2), D.SWITCH_H_MM, D.SWITCH_L_MM,
                               fc=red, alpha=0.7))
    ax.set_xlim(-40, 45)
    ax.set_ylim(-58, 58)
    ax.set_title("from above, cut at the pads' height (10 mm)", fontsize=10)
    ax.set_xlabel("mm into the dock from the back wall's face")
    ax.set_ylabel("mm left of the dock's axis")
    # side view through the + pad
    ax = axs[1]
    ax.add_patch(Rectangle((-DOCKED_SHIFT_MM - 47.6, D.FRONT_FACE_BOTTOM_Z_MM), 95.25, D.ROBOT_TOP_Z_MM - 4.2,
                           fc=grey, ec="#999"))
    _fill(ax, att, "XZ", PAD_Y_MM, 0, 2, blue, sh)
    _fill(ax, dock, "XZ", PAD_Y_MM, 0, 2, dark)
    name, y, step, prot, cf, cc = pogo_geo()[0]
    ax.add_patch(Rectangle((step, PAD_Z_MM - POGO_BARREL_DIA_MM / 2), POGO_BARREL_LEN_MM, POGO_BARREL_DIA_MM, fc=gold))
    ax.add_patch(Rectangle((-(prot - cf), PAD_Z_MM - POGO_PLUNGER_DIA_MM / 2), step + prot - cf, POGO_PLUNGER_DIA_MM,
                           fc=gold))
    xh = -PAD_RECESS_MM
    ax.add_patch(Rectangle((xh - PAD_SCREW_LEN_MM, PAD_Z_MM - 1.5), PAD_SCREW_LEN_MM - PAD_HEAD_H_MM, 3.0, fc=gold,
                           alpha=0.9))
    ax.add_patch(matplotlib.patches.Polygon([(xh, PAD_Z_MM - 3), (xh, PAD_Z_MM + 3), (xh - PAD_HEAD_H_MM, PAD_Z_MM + 1.5),
                                             (xh - PAD_HEAD_H_MM, PAD_Z_MM - 1.5)], closed=True, fc=gold))
    ax.axhline(0, color="k", lw=0.8)
    ax.set_xlim(-40, 45)
    ax.set_ylim(-2, 30)
    ax.set_title("from the side, cut through the + pad and pin", fontsize=10)
    ax.set_xlabel("mm into the dock from the back wall's face")
    ax.set_ylabel("mm above the plywood")
    for a in axs:
        a.set_aspect("equal")
        a.grid(True, lw=0.3)
    fig.text(0.5, 0.02, "Drawn from the CAD (build_attachment_v2.py): a docked robot (grey) with attachment v2 (blue) "
             "seated on dock v3 (black); each spring pin (gold) in the back wall presses on a brass screw head in the "
             "bar, between the two nose switches (red).", ha="center", fontsize=9.5, wrap=True)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def size_png(path):
    """The printed nose bracket against the robot, to scale, from above and from the front."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    br = B.bracket_solid()
    blue, grey = "#2f5d8a", "#e3e7ea"
    fig, axs = plt.subplots(1, 2, figsize=(12, 5.2), dpi=120, gridspec_kw={"width_ratios": [1.2, 1.0]})
    Lr, hw = D.ROBOT_LENGTH_MM, D.ROBOT_WIDTH_MM / 2
    ax = axs[0]
    ax.add_patch(Rectangle((-Lr, -hw), Lr, 2 * hw, fc=grey, ec="#888"))
    _fill(ax, br, "XY", D.SIDE_INSERT_Z_MM[0], 0, 1, blue)
    ax.annotate("", xy=(D.FACE_X_MM + 4, D.ARM_OUTER_Y_MM), xytext=(D.FACE_X_MM + 4, -D.ARM_OUTER_Y_MM),
                arrowprops=dict(arrowstyle="<->", lw=0.8))
    ax.text(D.FACE_X_MM + 6, 0, "%.1f mm\n(4 1/32 in)" % D.BRACKET_W_MM, va="center", fontsize=8.5)
    ax.annotate("", xy=(D.ARM_BACK_X_MM, -hw - 9), xytext=(D.FACE_X_MM, -hw - 9),
                arrowprops=dict(arrowstyle="<->", lw=0.8))
    ax.text((D.ARM_BACK_X_MM + D.FACE_X_MM) / 2, -hw - 15, "%.0f mm (1 1/2 in)" % D.BRACKET_LEN_MM, ha="center",
            fontsize=8.5)
    ax.text(-Lr / 2, 0, "robot\n%.0f x %.0f mm\n(3 3/4 in square)" % (D.ROBOT_WIDTH_MM, Lr), ha="center", va="center",
            fontsize=9, color="#555")
    ax.set_xlim(-Lr - 5, D.FACE_X_MM + 30)
    ax.set_ylim(-hw - 20, hw + 8)
    ax.set_title("from above (cut 15 mm up)", fontsize=10)
    ax.set_xlabel("mm forward of the robot's front face")
    ax = axs[1]
    ax.add_patch(Rectangle((-hw, D.FRONT_FACE_BOTTOM_Z_MM), 2 * hw, D.ROBOT_TOP_Z_MM - 4.2, fc=grey, ec="#888"))
    rec = hw - 12.3
    ax.add_patch(Rectangle((-rec, D.BAND_TOP_Z_MM), 2 * rec, D.ROBOT_TOP_Z_MM - D.BAND_TOP_Z_MM, fc="#cfd5da", ec="#999"))
    ax.add_patch(Rectangle((-D.TOF_WINDOW_HALF_WIDTH_MM, D.TOF_WINDOW_BOTTOM_Z_MM), 2 * D.TOF_WINDOW_HALF_WIDTH_MM, 3.2,
                           fc="k"))
    _fill(ax, br, "YZ", D.FACE_X_MM - 0.5, 1, 2, blue)
    ax.text(0, 35, "distance sensor", ha="center", fontsize=8)
    ax.text(0, 10, "bar: 10 mm tall", ha="center", va="center", fontsize=8.5, color="white")
    ax.axhline(0, color="k", lw=0.8)
    ax.set_xlim(-hw - 8, hw + 8)
    ax.set_ylim(-2, D.ROBOT_TOP_Z_MM + 6)
    ax.set_title("from the front", fontsize=10)
    ax.set_xlabel("mm left of the centre line")
    for a in axs:
        a.set_aspect("equal")
        a.grid(True, lw=0.3)
    fig.text(0.5, 0.02, "The nose bracket (blue, from build.py's CAD) drawn to scale on the robot (grey, Arduino's CAD "
             "and Nick's Oct 19 tape measurements): 6 mm wider than the robot, it wraps only the front of it, and its "
             "bar stays under the distance sensor's window, which is why it looks small.", ha="center", fontsize=9.5,
             wrap=True)
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.savefig(path)
    plt.close(fig)


# =============================================================================== main
def main():
    c = B.cq()
    os.makedirs(B.STL, exist_ok=True)
    os.makedirs(B.PNG, exist_ok=True)
    att, geo = attachment_v2_solid()
    dock = dock_v3_solid()
    ok, lines = checks(att, geo, dock)
    print("is the printed nose bracket too small?")
    for l in size_lines():
        print("     " + l)
    print("\nattachment v2 and dock v3:")
    print("\n".join(lines))
    # attachment: upside down (plate's top face on the bed), supports on, as version 1
    on_bed = att.rotate((0, 0, 0), (1, 0, 0), 180)
    b = on_bed.val().BoundingBox()
    on_bed = on_bed.translate((-b.xmin, -b.ymin, -b.zmin))
    sup = L.overhang_above(on_bed, max(L.PIXEL_POCKET_DEPTH_MM, L.CHANNEL_D_MM) + 0.3)
    print("     attachment_v2: print upside down (plate's top face on the bed) with supports on ('everywhere', tree "
          "supports if offered): %.0f mm^2 needs them (version 1 needed about 1450). %.0f x %.0f x %.0f mm"
          % (sup, b.xlen, b.ylen, b.zlen))
    on_bed.val().exportStl(os.path.join(B.STL, "attachment_v2.stl"), 0.05, 0.3)
    # dock: as drawn (bottom on the bed); the cavity and channel roofs are bridges
    bed = dock.translate((0, 0, 0))
    bb = bed.val().BoundingBox()
    bed = bed.translate((-bb.xmin, -bb.ymin, -bb.zmin))
    over = B.overhang_mm2(bed)
    bridge = (CAVITY_X_MM[1] - CAVITY_X_MM[0]) * 2 * CAVITY_HALF_Y_MM + (V2.HEAD_DEPTH_MM - CAVITY_X_MM[1]) * 2 * CHANNEL_HALF_W_MM
    print(("ok   " if over <= bridge + 60 else "WARN ") + "dock_v3: print bottom down, as dock_v2; %.0f mm^2 faces down, "
          "of which about %.0f is the cavity's roof (a %.0f mm bridge) and the channel's (%.0f mm): most printers bridge "
          "these without support; if yours sags, use support 'touching build plate' only"
          % (over, bridge, CAVITY_X_MM[1] - CAVITY_X_MM[0], 2 * CHANNEL_HALF_W_MM))
    c.exporters.export(bed, os.path.join(B.STL, "dock_v3.stl"), tolerance=0.05, angularTolerance=0.1)
    # fit test: the attachment from the bar's bottom to above the low screw holes
    zt = D.SIDE_INSERT_Z_MM[0] + 3.3
    # front part only (the bracket): the light plate's back legs would come out as loose pieces
    x_back = D.AXLE_TO_FRONT_MM + D.ARM_BACK_X_MM - 1.0
    fit = att.intersect(c.Workplane("XY").workplane(offset=D.BUMPER_Z0_MM - 0.5).center(x_back + 50, 0)
                        .box(100, 400, zt - D.BUMPER_Z0_MM + 0.5, centered=(True, True, False)))
    assert len(fit.val().Solids()) == 1, "fit test is not one piece"
    fb = fit.val().BoundingBox()
    fit_bed = fit.translate((-fb.xmin, -fb.ymin, -fb.zmin))
    c.exporters.export(fit_bed, os.path.join(B.STL, "attachment_v2_fit_test.stl"), tolerance=0.05, angularTolerance=0.1)
    print("ok   fit test: %.0f x %.0f x %.0f mm slice (the bar with its switch pockets and pads, the arms up to the low "
          "screw holes): bottom down, no support, about 20 min" % (fb.xlen, fb.ylen, fb.zlen))
    gauge, sizes = pogo_gauge_solid()
    gb = gauge.val().BoundingBox()
    c.exporters.export(gauge.translate((-gb.xmin, -gb.ymin, -gb.zmin)), os.path.join(B.STL, "pogo_gauge.stl"),
                       tolerance=0.05, angularTolerance=0.1)
    print("ok   pogo gauge: holes %s mm, smallest at the notched end; bottom down, no support"
          % " / ".join("%.1f" % s for s in sizes))
    B.preview("attachment_v2", att, os.path.join(B.PNG, "attachment_v2.png"), plan_z=PAD_Z_MM)
    B.preview("dock_v3", dock, os.path.join(B.PNG, "dock_v3.png"), PAD_Z_MM)
    contact_png(att, dock, os.path.join(B.PNG, "charging_contact.png"))
    size_png(os.path.join(B.PNG, "nose_bracket_size.png"))
    print("\nwrote %s/{attachment_v2, dock_v3, attachment_v2_fit_test, pogo_gauge}.stl" % B.STL)
    print("      %s/{attachment_v2, dock_v3, charging_contact, nose_bracket_size}.png" % B.PNG)
    print("\nfirmware numbers: unchanged (the bracket's outside is version 1's):")
    for l in D.firmware_lines():
        print("  " + l)
    if not ok:
        print("\nSOME CHECKS FAILED")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
