# E0e: Home dock and touch probe

Status: planned (simulation done)    Week: 7    Run dates:

## Question
How exactly does a robot sit in the printed home dock after creeping in on its nose switches, and how exactly
does a touch with the nose switches find a board's face?

## Hypothesis
H1: over 20 dockings per robot, the docked pose spreads by at most 0.5 mm along the dock's axis, 0.6 mm
sideways and 0.3 deg (so a DOCK record is a pose fix good to about 0.5 mm / 0.3 deg, the fusion's
dock_sigma_cm 0.05 / dock_sigma_deg 0.3); 20 touches of a board from a tape-measured start give the board's
distance with a spread of 0.3 mm or less and a mean within 1 mm of the tape (the tape's own limit).
H0: the dock is no better than the camera (about 3 mm).

Simulation (tools/sim/tests/test_dock.py, 20 dockings per robot from hand-placed starts 1-8 cm before the mouth,
up to 1 cm to the side and 8 deg turned): all 60 docked; true docked pose spread (sd) 0.04-0.05 mm along the
axis (max 0.11), 0.46-0.52 mm sideways (max 0.68: the pocket's 0.5 mm play each side plus a tilted seat) and
0.09 deg (max 0.21). A touch from 3.0 cm: travel + sw_gap within 0.5 mm of 3.0 cm; 19 repeat touches spread
under 0.2 mm. The contact model and the switch numbers are guesses: this experiment measures them.

## Variables
| Kind | Variable | Levels or units |
| --- | --- | --- |
| Independent | Robot; start placement (offset sideways, turn) | Left / Center / Right; random within 1 cm, 8 deg |
| Dependent | Docked pose: front and rear marks by tape, 4 points | cm (to 1/16 in), then x, y, heading |
| Dependent | DOCK record: travel, pivot, stripe, drift | cm, deg, deg/s |
| Dependent | TOUCH travel from a tape-measured start | cm |
| Held constant | dock position (screwed down), plywood, firmware 2026-10-19a, bumper settings | |

## Setup
- Robot(s): Left, Center, Right, each with its nose bumper fitted and wired (hardware/dock/README.md), and
  `set sw_on 1` plus the three `set bump_... / sw_gap_cm` lines build.py printed.
- Dock screwed to the plywood, its pose in the map (`home_docks`, measured with the tape), check tile in place.
- Firmware commit: <`git rev-parse --short HEAD` when flashed>
- Camera recording on (the check tile and the robot are both in view).
- Ground truth: tape rig (E0 repeatability: <x> cm, <y> deg)

## Procedure

### Part A: 20 dockings per robot (about 25 min per robot)
1. After power-on, `calibrate` (the compass check: the DOCK record's pivot and drift need it).
2. In the console: `set dock_back_cm 0` (it stays docked so you can measure it) and `set dock_still_ms 3000`.
3. For each of the 20 rows of the sheet: put the robot down 3-15 cm in front of the dock mouth, roughly facing
   in, at the offset and turn the sheet says (by eye is fine). Type `run dock`. Wait for `dock finished`
   and a DOCK line with `docked`. Mark the robot's front and rear dots (as in T1) and tape-measure them
   (4 numbers). Pick the robot up for the next row.
4. A row that ends with anything but `docked`: write the result in the sheet's notes, keep the row
   (excluded = 1 only if the robot was knocked).

### Part B: 20 touches of a board (about 10 min per robot)
1. Stand a board square to the x axis. Put the robot square to it, its bumper face 3.0 cm (1 3/16 in) from
   the board's face, by tape (mark and measure the robot's front and rear dots too).
2. `set touch_n 20`, `set touch_back_cm 2`, then `run touch`. It touches the board 20 times, backing off 2 cm
   between touches.
3. Repeat from 6.0 cm (2 3/8 in) and 10.0 cm (3 15/16 in) once each (touch_n 1): the wheel distance's scale.

### Part C: the camera check tile
1. Leave the camera recording 5 minutes with nothing moving, then lean on the camera stand lightly.
2. `python tools/check_session.py <session> --video <recording> --dock-map <map.json> --calib <floor_calibration.json>`

## Trials
- Part A: 20 per robot: the sd of 20 tape-measured poses is known to about +-16%, enough to tell 0.5 mm from the
  tape's own 0.8 mm (1/32 in reading) spread; the tape spread is removed with E0's repeatability.
- Part B: 20 touches per start: the encoder spread is tiny, so 20 shows any slipping or bounce.
- Run order: data/measurements.csv (rows dock01..dock20 per robot, then touch3, touch6, touch10), with random
  start offsets: `tools/make_run_order.py` is not set up for this test yet; write the sheet by hand.

## Raw data
- data/measurements.csv: tape readings (start and docked/start marks), run_id, start offset and turn
- data/logs/: the console's session folder (dock.csv, touch.csv, raw.csv with swL / swR)

## Pass rules
| Check | Pass | What a fail means |
| --- | --- | --- |
| Every docking ends `docked` | 20 of 20 per robot (19 with a reason noted) | funnel too steep, switches misplaced, or the pocket too tight (ROBOT_WIDTH_MM wrong) |
| Docked position spread (tape), sd along the axis and sideways, after taking off the tape's own spread (E0) | <= 0.5 mm along, <= 0.6 mm sideways | the dock is not a 0.5 mm fix: raise dock_sigma_cm in the fusion to the measured sd |
| Docked heading spread (tape) | <= 0.3 deg | raise dock_sigma_deg |
| Mean docked pose vs the map's dock pose minus bump_ahead_cm | within 1 mm and 0.5 deg | bump_ahead_cm or the map's dock pose is off: correct the one the tape disagrees with |
| (travel_cm - stripe_c_cm) over the 20 dockings | sd <= 0.5 mm | the stripe reading is not a usable second reading (stripe_thr) |
| drift_dps while docked | the same for every docking within 0.05 deg/s | something moves the robot while docked |
| Touch from 3.0 cm: travel_cm + sw_gap_cm | within 1.0 mm of 3.0 cm (tape-limited); 20-touch sd <= 0.3 mm | sw_gap_cm wrong (set it to 3.0 - mean travel), or the switch's click point wanders |
| Touches from 3, 6, 10 cm: (travel + sw_gap) / tape distance | 1.000 +- 0.01 | the wheel distance's scale (also the calibration run's wheel scale) |
| Check tile still (Part C, before leaning) | dock_tile PASS (< 0.5 px) | the tile or the camera is loose |
| Check tile after leaning on the stand | dock_tile CHECK or FAIL | the check cannot see a bumped camera (make the tile bigger) |

## Results
(to fill in)

## Analysis
Docked pose sd per robot with a 95% interval (chi-square), tape spread removed in quadrature. Touch: mean and
sd of travel per start distance; regression of travel on tape distance (slope = wheel scale, intercept = -sw_gap).

## Conclusion
(to fill in)

## Deviations
