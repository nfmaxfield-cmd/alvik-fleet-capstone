"""Every dimension of the home dock, the nose bracket (bumper) and the LED holder, in mm.

Where each robot number comes from (Oct 19):
  SOURCED   Arduino's own files for the Alvik (AKX00066):
            [TS]   tech specs, https://github.com/arduino/docs-content/blob/main/content/hardware/edu/
                   solution-and-kits/alvik/tech-specs.yml  (width 96, length 95, height 37 mm; 8 M3 screw mounts)
            [DS]   datasheet, https://docs.arduino.cc/resources/datasheets/AKX00066-datasheet.pdf  section 2.4.3:
                   four M3 screw inserts on each side, "maximum screw length recommended is 10 mm (M3x10)"
            [STEP] Arduino's reference CAD model, AKX00066-step.zip (alvik_reference_design.step, "structure v5",
                   2024-09-19) in https://github.com/arduino/docs-content/tree/main/content/hardware/edu/
                   solution-and-kits/alvik/downloads, loaded in cadquery and measured (numbers below)
            [SVG]  Arduino's front-view drawing, Front-Alvik_Robot.svg (same repo folder), scaled so the shell is
                   96 mm wide: front recess and distance sensor heights. It is an illustration, not a drawing with
                   dimensions: those numbers are +/- 1 mm
  MEASURED  on Nick's robot, Oct 19 (tape measure)
  ASSUMED   a typical value, nothing measured or published

The STEP model's frame: x across the robot, +y forward (front face at y = +47.4..48.0, drafted), z up with the
floor at z = -10.1 (the wheels' and the caster's lowest points). Wheel axle at y = 0, z = 6.8 (16.9 mm above
the floor). Side inserts at y = +/-28.0, z = 4.6 and 20.6, a LEGO Technic hole between them at z = 12.6.
The model is the shell only, simplified: no front recess, no PCB, no screws.

##############################################################################
#                                                                            #
#   After printing, the numbers the FIRMWARE uses must match this file:      #
#       set bump_ahead_cm <BUMP_AHEAD_MM / 10>                               #
#       set bump_half_cm  <SWITCH_LATERAL_MM / 10>                           #
#       set sw_gap_cm     <SW_GAP_MM / 10>  (then measured in E00e)          #
#   (python hardware/dock/build.py prints these three lines.)                #
#                                                                            #
##############################################################################

Frames
  robot:  X forward from the robot's FRONT FACE (the shell's flat front, at the top of the white band below the
          sensor recess), Y to the robot's left, Z up from the floor. Lateral positions are from the centre line.
  dock:   X along the dock's axis, pointing INTO the dock (the way a docked robot faces), 0 at the back wall's
          inner face; Y to the left of that axis, 0 on it; Z up from the plywood.
The simulator (tools/sim/config.py DockModel) and the fleet map (tools/fleet/fleetmap.py) read this file, so the
simulated and planned dock is the printed one.
"""

# =============================================================================== the robot
ROBOT_WIDTH_MM = 96.0          # SOURCED [TS] 96 mm; [STEP] 96.0 across the shell's top edge and across the tyres.
                               # MEASURED Oct 19: 3 3/4 in (95.25): the sides lean in 1 deg (below), so a tape low
                               # on the shell reads ~95. Only the simulator uses it now (the dock pocket fits the
                               # bracket, which is wider)
ROBOT_LENGTH_MM = 95.25        # MEASURED Oct 19: 3 3/4 in. [TS] 95; [STEP] 94.8 at the bottom edge, 96.0 at the top
AXLE_TO_FRONT_MM = 47.6        # SOURCED [STEP]: wheel axle y = 0, front face y = 47.6 at the white band's top
                               # (47.4 at the shell's bottom edge, 48.0 at the top: the face leans back 1 deg)
FRONT_FACE_BOTTOM_Z_MM = 4.2   # SOURCED [STEP]: the shell's front edge (line sensor board) is 4.2 mm above the floor
BAND_TOP_Z_MM = 11.7           # SOURCED [SVG]: the plain white band below the recess ends here (the recess's rim
                               # starts); the bracket's bar bears on the band below this
TOF_WINDOW_BOTTOM_Z_MM = 29.5  # SOURCED [SVG]: bottom of the distance sensor's black chip (top centre of the recess)
TOF_WINDOW_HALF_WIDTH_MM = 3.2 # SOURCED [SVG]: half the chip's width
TOF_CENTER_Z_MM = 31.1         # SOURCED [SVG]: middle of the chip (the sensor's lens)
TOF_RECESS_MM = 4.0            # ASSUMED: how far the chip sits behind the front face (the recess depth; [STEP]
                               # does not model it). Larger = the checks below are stricter
TOF_LOW_RAY_DEG = 22.5         # ASSUMED: the sensor's lowest ray, below horizontal (VL53L7CX: 45 deg vertical view,
                               # mounted level)
ROBOT_TOP_Z_MM = 37.2          # SOURCED [STEP] 37.2 (z 27.1 - -10.1); [TS] 37. MEASURED Oct 19: 37.3 at the front
                               # (1 15/32 in), 36.5 at the back

# The sides: flat walls that lean OUT 1.1 deg going up (moulding draft), wheels in arches in the middle.
SIDE_HALF_W_Z0_MM = 47.39      # SOURCED [STEP]: half width of the shell's side wall at SIDE_Z0_MM ...
SIDE_Z0_MM = 5.1               # ... (5.1 mm above the floor, near the front)
SIDE_DRAFT = 0.0191            # SOURCED [STEP]: ... growing this much per mm of height (47.96 at 35 mm)
# Threaded inserts on each side (brass, M3): two near the front end, two near the back end.
SIDE_INSERT_X_MM = -19.6       # SOURCED [STEP]: the front pair, y = +28.0: 19.6 mm behind the front face (20.0
                               # behind the top edge). The back pair is at y = -28.0 (56 mm further back)
SIDE_INSERT_Z_MM = (14.7, 30.7)   # SOURCED [STEP]: heights above the floor of the low and the high insert (16 mm
                                  # apart); the LEGO Technic hole between them is at 22.7
SIDE_INSERT_DEPTH_MM = 5.6     # SOURCED [STEP]: depth of the insert's bore in the wall (the screw must not reach
                               # further than this into the robot)
M3_SCREW_MAX_MM = 10.0         # SOURCED [DS]: longest screw Arduino allows in the inserts
WHEEL_R_MM = 16.9              # SOURCED [STEP] (16.9 radius arch; [DS] 34 mm wheels)
WHEEL_FRONT_X_MM = -(AXLE_TO_FRONT_MM - WHEEL_R_MM)   # the tyre's front, robot X (at axle height)

# =============================================================================== nose switches (Pololu #1528-type)
# Pololu's own page for the 13.5 mm-lever mini switch (item #1528) gives the body as 12.8 x 5.8 x 6.5 mm
# (its description also says 6.0 wide). Pololu #1404 is NOT this switch: it is the 20 x 6.4 x 10.2 mm roller-lever one.
# Mounting holes and lever travel are not on Pololu's page: ASSUMED (typical subminiature lever switch).
SWITCH_L_MM = 12.8             # along the robot's width (Y)
SWITCH_T_MM = 5.8              # thickness: lies flat, so this is its height (Z)
SWITCH_H_MM = 6.5              # terminal side to lever side (X)
SWITCH_HOLE_DIA_MM = 2.2       # ASSUMED: 2 holes, through the thickness, for M2 or #2 screws
SWITCH_HOLE_SPACING_MM = 6.5   # ASSUMED: centre to centre, along SWITCH_L, centred on the body
SWITCH_HOLE_X_MM = 3.25        # ASSUMED: holes this far from the terminal side (mid-height of the body)
SWITCH_TERMINAL_MM = 3.5       # ASSUMED: terminals stick out this far behind the body (+ solder, wires)
LEVER_FREE_MM = 2.5            # ASSUMED: lever's free position above the body's lever side
LEVER_OPERATE_MM = 1.2         # ASSUMED: lever position where the switch clicks (operating position)
LEVER_PRESSED_MM = 0.5         # ASSUMED: lever position at the end of its travel (never go past this)

# =============================================================================== the nose bracket (bumper)
# A U: a bar across the front at the white band's height carrying the two switches at its corners, and an arm
# back along each side to the FRONT pair of side inserts (two M3 screws a side). The bar's back sits flat on the
# white band, so a push on the face goes straight into the shell; the screws only hold it there and stop it
# tipping. The arms make it wider than the robot: the dock's pocket is made to fit the BRACKET (below).
BUMPER_Z0_MM = 5.0             # bar bottom above the floor (0.8 above the shell's front edge; clears the 0.6 mm
                               # floor stripe and floor bumps)
BUMPER_Z1_MM = 15.0            # bar top: far below the distance sensor (checked by build.py)
BACKPLATE_T_MM = 2.4           # bar's plate against the robot's front face
FACE_RIB_MM = 2.0              # outer rib beside each switch pocket (part of the flat front face)
CORNER_CHAMFER_MM = 3.0        # 45 deg chamfer on the bar's two front corners (helps the funnel)
SHELF_T_MM = 1.6               # floor under each switch
# The front face (the hard stop) is this far in front of the switch body's lever side: between the operating and
# the fully pressed lever positions, so at full contact the switch has clicked and is not crushed.
FACE_ABOVE_BODY_MM = 1.0
ARM_T_MM = 3.0                 # arm thickness at its top (its outer face is vertical, its inner face follows the
                               # side wall's lean, so it is 3.6 at the bottom)
ARM_CLEAR_MM = 0.15            # gap between each arm and the side wall (the screws pull it closed)
ARM_TOP_Z_MM = SIDE_INSERT_Z_MM[1] + 5.0      # arm's top edge at the screws (below the robot's top)
ARM_BOSS_R_MM = 5.0            # material around each screw hole, from the hole's centre to the arm's back edge
M3_CLEAR_DIA_MM = 3.4          # M3 clearance hole through the arm
M3_HEAD_DIA_MM = 6.0           # pan / button head (stands out of the arm; behind the dock pocket: checked)
M3_HEAD_H_MM = 2.1             # head height (ISO 7380 button 1.65, Phillips pan 2.1)
M3_SCREW_LEN_MM = 8.0          # M3 x 8: 3 mm of arm + 5 mm into the insert (bore 5.6; Arduino's limit 10)

# derived (do not edit): where things end up
SWITCH_BACK_X_MM = BACKPLATE_T_MM + SWITCH_TERMINAL_MM               # switch body's terminal side
FACE_X_MM = SWITCH_BACK_X_MM + SWITCH_H_MM + FACE_ABOVE_BODY_MM      # bracket's front face
BUMPER_DEPTH_MM = FACE_X_MM                                          # firmware bumper_cm x 10
BUMP_AHEAD_MM = AXLE_TO_FRONT_MM + BUMPER_DEPTH_MM                   # firmware bump_ahead_cm x 10


def side_half_w(z):
    """Half width of the robot's side wall at height z above the floor (mm), from the [STEP] model."""
    return SIDE_HALF_W_Z0_MM + SIDE_DRAFT * (z - SIDE_Z0_MM)


ARM_OUTER_Y_MM = side_half_w(ARM_TOP_Z_MM) + ARM_CLEAR_MM + ARM_T_MM   # arm's outer face (vertical)
BRACKET_W_MM = 2 * ARM_OUTER_Y_MM                                      # the bracket's width: what the pocket fits
ARM_BACK_X_MM = SIDE_INSERT_X_MM - ARM_BOSS_R_MM                       # arm's back end, robot X
BRACKET_LEN_MM = FACE_X_MM - ARM_BACK_X_MM                             # bracket face to arm's back end
SWITCH_Y_OUTER_MM = ARM_OUTER_Y_MM - FACE_RIB_MM                       # switch body's outer end
SWITCH_LATERAL_MM = SWITCH_Y_OUTER_MM - SWITCH_L_MM / 2              # firmware bump_half_cm x 10 (lever middle)
SW_GAP_MM = LEVER_OPERATE_MM - FACE_ABOVE_BODY_MM                    # firmware sw_gap_cm x 10: face-to-wall gap at the click

# =============================================================================== the dock
# The pocket fits the BRACKET (its bar is what sits in the pocket, BRACKET_W_MM wide), not the bare robot: the
# printed bracket is the widest thing at the front and its width is exact to the print, where the shell leans
# and the tyres squash. Behind the pocket the funnel opens fast enough to clear the arms and the screw heads
# (build.py checks it).
POCKET_CLEAR_MM = 1.0          # pocket is this much wider than the bracket (0.5 mm each side)
POCKET_DEPTH_MM = 12.0         # straight part before the back wall: at most sqrt((Wb+clear)^2 - Wb^2) = 14.3 mm,
                               # so the bracket reaches the back wall at ANY angle (the switches then square it)
FUNNEL_LEN_MM = 50.0           # funnel, mouth to pocket, along the axis
FUNNEL_FLARE_MM = 32.0         # each side opens this much at the mouth (atan(32/50) = 33 deg)
WALL_T_MM = 3.2                # every wall (4 perimeters of a 0.4 mm nozzle)
WALL_H_MM = 22.0               # wall height above the plywood (bracket bar top is 15)
BACK_BLOCK_DEPTH_MM = 48.0     # solid block behind the back wall, carries the camera check tile on top
TILE_MM = 40.0                 # camera check tile: square side ...
TILE_SQUARES = 4               # ... in TILE_SQUARES x TILE_SQUARES squares (10 mm each)
TILE_T_MM = 1.2                # tile thickness; the dark squares stand TILE_RAISE_MM proud of it
TILE_RAISE_MM = 0.6
TILE_RECESS_CLEAR_MM = 0.4     # recess is this much larger than the tile (each way)
STRIPE_X0_MM = -52.0           # floor stripe: its far edge (the one a robot coming in reaches first) ...
STRIPE_X1_MM = -44.0           # ... and its near edge, dock X
STRIPE_T_MM = 0.6              # thickness (3 layers at 0.2 mm); edges chamfered so the wheels roll over
SCREW_HOLE_DIA_MM = 3.6        # screw tabs: for #6 / 3.5 mm wood screws, countersunk
SCREW_HEAD_DIA_MM = 7.0
TAB_W_MM, TAB_L_MM, TAB_T_MM = 12.0, 12.0, 3.0

# derived (do not edit)
POCKET_W_MM = BRACKET_W_MM + POCKET_CLEAR_MM
MOUTH_W_MM = POCKET_W_MM + 2 * FUNNEL_FLARE_MM
MOUTH_X_MM = -(POCKET_DEPTH_MM + FUNNEL_LEN_MM)
TILE_CENTER_X_MM = WALL_T_MM + (BACK_BLOCK_DEPTH_MM - WALL_T_MM) / 2   # camera tile centre, dock X (on top, Z = WALL_H)
DOCKED_AXLE_X_MM = -BUMP_AHEAD_MM                                     # wheel axle when docked, dock X


def funnel_half_w(x):
    """Inner half width of the dock at dock X (mm): the pocket, then the funnel opening towards the mouth."""
    xp = -POCKET_DEPTH_MM
    if x >= xp:
        return POCKET_W_MM / 2
    return POCKET_W_MM / 2 + FUNNEL_FLARE_MM * (xp - x) / FUNNEL_LEN_MM


# =============================================================================== the LED holder (top)
LED_DIA_MM = 5.1               # 5 mm LEDs
LED_FLANGE_DIA_MM = 6.0        # their rim sits in a shallow counterbore
LED_POS_MM = [(0.0, 0.0), (24.0, 0.0), (0.0, 14.0)]   # L shape, long arm forward: identity + heading
LED_PLATE_T_MM = 3.0
LED_PLATE_MARGIN_MM = 6.0
LED_MOUNT_TECHNIC_MM = [(8.0, 7.0), (16.0, 7.0)]      # CHOOSE: Technic pin holes in the plate (the robot's top has
                                                      # no Technic holes in [STEP]: fix it with double-sided tape)
TECHNIC_PIN_HOLE_DIA_MM = 4.85 # LEGO Technic pin hole (4.8 nominal; 4.85-4.9 prints to a friction fit)

# =============================================================================== printer
BED_MM = (200.0, 200.0)        # every part must fit (checked by build.py)
NOZZLE_MM = 0.4
MIN_WALL_MM = 1.6              # 4 perimeters... at least 2 x 0.4 mm lines, here doubled for strength
MAX_OVERHANG_DEG = 45.0


def firmware_lines():
    """The console commands that put this file's numbers into the firmware."""
    return ["set bump_ahead_cm %.2f" % (BUMP_AHEAD_MM / 10.0),
            "set bump_half_cm %.2f" % (SWITCH_LATERAL_MM / 10.0),
            "set sw_gap_cm %.3f" % (SW_GAP_MM / 10.0),
            "set bumper_cm %.2f" % (BUMPER_DEPTH_MM / 10.0)]


def as_dict():
    return {k: v for k, v in globals().items() if k.isupper()}
