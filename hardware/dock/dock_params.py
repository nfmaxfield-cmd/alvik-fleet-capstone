"""Every dimension of the home dock, the nose bumper and the LED holder, in mm.

##############################################################################
#                                                                            #
#   THE ROBOT NUMBERS BELOW ARE PLACEHOLDERS. NOTHING HERE WAS MEASURED ON   #
#   A REAL ALVIK. Measure each "MEASURE" line (README.md, "What to measure   #
#   before printing" says how), type the number in here, and run             #
#       python hardware/dock/build.py                                        #
#   again before printing anything. A wrong robot width makes the dock       #
#   pocket too tight (the robot jams) or too loose (the dock's 0.5 mm        #
#   promise is gone); wrong hole positions make the bumper unmountable.      #
#                                                                            #
#   After printing, the numbers the FIRMWARE uses must match this file:      #
#       set bump_ahead_cm <BUMP_AHEAD_MM / 10>                               #
#       set bump_half_cm  <SWITCH_LATERAL_MM / 10>                           #
#       set sw_gap_cm     <SW_GAP_MM / 10>  (then measured in E00e)          #
#   (python hardware/dock/build.py prints these three lines.)                #
#                                                                            #
##############################################################################

Frames
  robot:  X forward from the robot's FRONT FACE (the shell's flat front), Y to the robot's left, Z up from the
          floor. Lateral positions are measured from the robot's centre line.
  dock:   X along the dock's axis, pointing INTO the dock (the way a docked robot faces), 0 at the back wall's
          inner face; Y to the left of that axis, 0 on it; Z up from the plywood.
The simulator (tools/sim/config.py DockModel) reads this file, so the simulated dock is the printed one.
"""

# =============================================================================== the robot (MEASURE ALL)
ROBOT_WIDTH_MM = 95.0          # MEASURE: widest point of the robot's front half, wheels included (calipers)
ROBOT_LENGTH_MM = 95.0         # MEASURE: front face to rear face
AXLE_TO_FRONT_MM = 47.5        # MEASURE: wheel axle (middle of a wheel's hub) to the front face, along the floor
FRONT_FACE_BOTTOM_Z_MM = 6.0   # MEASURE: height of the front face's lowest edge above the floor
TOF_WINDOW_BOTTOM_Z_MM = 21.0  # MEASURE: height of the bottom edge of the front distance sensor's dark window
TOF_WINDOW_HALF_WIDTH_MM = 9.0 # MEASURE: half the window's width (the bumper's mounting tabs keep clear of it)
TOF_CENTER_Z_MM = 26.0         # MEASURE: height of the middle of that window (the sensor's lens)
TOF_RECESS_MM = 2.5            # MEASURE: how far the lens sits behind the front face
TOF_LOW_RAY_DEG = 22.5         # ASSUMED: the sensor's lowest ray, below horizontal (45 deg vertical view, level)

# How the bumper holds on: "m3" (screws into holes on the robot's front face) or "technic" (two straps up the
# front face's outer edges, beside the distance sensor, and back over the top to LEGO Technic friction pins in
# holes on the robot's top). build.py makes both; print the one that fits. Nothing may stick out sideways: the
# dock's pocket is only 1 mm wider than the robot.
FRONT_M3_HOLES_MM = [(-14.0, 12.0), (14.0, 12.0)]   # MEASURE: (Y lateral, Z height) of each M3 hole on the front
M3_CLEAR_DIA_MM = 3.4          # M3 clearance hole
M3_HEAD_DIA_MM = 6.0           # counterbore for an M3 cap/pan head
ROBOT_TOP_Z_MM = 42.0          # MEASURE: height of the robot's flat top above the floor (near the front)
TECHNIC_HOLES_MM = [(12.0, 36.0), (12.0, -36.0)]    # MEASURE: (distance back from the front face, Y lateral) of
                                                    # Technic holes on the robot's TOP, one per strap (8 mm pitch)
STRAP_W_MM = 7.0               # width of each strap (Y); must stay outside the sensor window's half width
TECHNIC_PIN_HOLE_DIA_MM = 4.85 # LEGO Technic pin hole (4.8 nominal; 4.85-4.9 prints to a friction fit)

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

# =============================================================================== the nose bumper
BUMPER_Z0_MM = 6.0             # bumper bottom above the floor (clears the 0.6 mm floor stripe and bumps)
BUMPER_Z1_MM = 15.0            # bumper top: under the distance sensor's lowest ray (checked by build.py)
BACKPLATE_T_MM = 2.4           # plate against the robot's front face
FACE_RIB_MM = 2.0              # outer rib beside each switch pocket (part of the flat front face)
CORNER_CHAMFER_MM = 3.0        # 45 deg chamfer on the bumper's two front corners (helps the funnel)
SHELF_T_MM = 1.6               # floor under each switch
# The front face (the hard stop) is this far in front of the switch body's lever side: between the operating and
# the fully pressed lever positions, so at full contact the switch has clicked and is not crushed.
FACE_ABOVE_BODY_MM = 1.0

# derived (do not edit): where things end up
SWITCH_BACK_X_MM = BACKPLATE_T_MM + SWITCH_TERMINAL_MM               # switch body's terminal side
FACE_X_MM = SWITCH_BACK_X_MM + SWITCH_H_MM + FACE_ABOVE_BODY_MM      # bumper's front face
BUMPER_DEPTH_MM = FACE_X_MM
BUMP_AHEAD_MM = AXLE_TO_FRONT_MM + BUMPER_DEPTH_MM                   # firmware bump_ahead_cm x 10
SWITCH_Y_OUTER_MM = ROBOT_WIDTH_MM / 2 - FACE_RIB_MM                 # switch body's outer end
SWITCH_LATERAL_MM = SWITCH_Y_OUTER_MM - SWITCH_L_MM / 2              # firmware bump_half_cm x 10 (lever middle)
SW_GAP_MM = LEVER_OPERATE_MM - FACE_ABOVE_BODY_MM                    # firmware sw_gap_cm x 10: face-to-wall gap at the click

# =============================================================================== the dock
POCKET_CLEAR_MM = 1.0          # pocket is this much wider than the robot (0.5 mm each side)
POCKET_DEPTH_MM = 12.0         # straight part before the back wall: at most sqrt((W+clear)^2 - W^2) = 13.8 mm,
                               # so the robot reaches the back wall at ANY angle (the switches then square it)
FUNNEL_LEN_MM = 50.0           # funnel, mouth to pocket, along the axis
FUNNEL_FLARE_MM = 32.0         # each side opens this much at the mouth (atan(32/50) = 33 deg)
WALL_T_MM = 3.2                # every wall (4 perimeters of a 0.4 mm nozzle)
WALL_H_MM = 22.0               # wall height above the plywood (bumper top is 18)
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
POCKET_W_MM = ROBOT_WIDTH_MM + POCKET_CLEAR_MM
MOUTH_W_MM = POCKET_W_MM + 2 * FUNNEL_FLARE_MM
MOUTH_X_MM = -(POCKET_DEPTH_MM + FUNNEL_LEN_MM)
TILE_CENTER_X_MM = WALL_T_MM + (BACK_BLOCK_DEPTH_MM - WALL_T_MM) / 2   # camera tile centre, dock X (on top, Z = WALL_H)
DOCKED_AXLE_X_MM = -BUMP_AHEAD_MM                                     # wheel axle when docked, dock X

# =============================================================================== the LED holder (top)
LED_DIA_MM = 5.1               # 5 mm LEDs
LED_FLANGE_DIA_MM = 6.0        # their rim sits in a shallow counterbore
LED_POS_MM = [(0.0, 0.0), (24.0, 0.0), (0.0, 14.0)]   # L shape, long arm forward: identity + heading
LED_PLATE_T_MM = 3.0
LED_PLATE_MARGIN_MM = 6.0
LED_MOUNT_TECHNIC_MM = [(8.0, 7.0), (16.0, 7.0)]      # MEASURE / CHOOSE: Technic pin holes to fix it (or use tape)

# =============================================================================== printer
BED_MM = (200.0, 200.0)        # every part must fit (checked by build.py)
NOZZLE_MM = 0.4
MIN_WALL_MM = 1.6              # 4 perimeters... at least 2 x 0.4 mm lines, here doubled for strength
MAX_OVERHANG_DEG = 45.0


def firmware_lines():
    """The console commands that put this file's numbers into the firmware."""
    return ["set bump_ahead_cm %.2f" % (BUMP_AHEAD_MM / 10.0),
            "set bump_half_cm %.2f" % (SWITCH_LATERAL_MM / 10.0),
            "set sw_gap_cm %.3f" % (SW_GAP_MM / 10.0)]


def as_dict():
    return {k: v for k, v in globals().items() if k.isupper()}
