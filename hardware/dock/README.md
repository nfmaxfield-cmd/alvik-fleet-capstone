# Home dock, nose bumper, check tile and LED holder (3D printed)

A printed **dock** fixed to the plywood gives a robot a near-exact position fix: the robot creeps in, a funnel
centres it, and two lever switches on a printed **nose bumper** tell it when it sits flush against the dock's
back wall. The same switches make the robot a **touch probe** (it can feel where a board is). A printed
**check tile** on top of the dock gives the overhead camera a spot that never moves (a bumped camera shows up at
once), a dark **floor stripe** at the dock's mouth gives the line sensors a second reading of where the robot
is, and an **LED holder** carries three LEDs in an L for identity and heading.

Firmware: `firmware/home_lab/switches.h`, `dock.h` (2026-10-19a). Simulator: `tools/sim/fw_dock.py`.
Records: `docs/data-formats.md`, "Home dock and touch probe". Test: `docs/experiments/E00e-dock/`.

| File | What |
| --- | --- |
| `dock_params.py` | **every dimension** (mm). The robot's numbers are PLACEHOLDERS: measure them first |
| `build.py` | makes the parts from it: `stl/*.stl` (already lying the right way on the bed), `preview/*.png`, and a printability / fit report |
| `stl/dock.stl` | the dock: funnel, pocket, back wall, check-tile recess, floor stripe, 4 screw tabs |
| `stl/nose_bumper_m3.stl` | the nose bumper, held by two M3 screws in the robot's front face |
| `stl/nose_bumper_technic.stl` | the same bumper held by two straps over the top to LEGO Technic pins (needs support under the straps: prefer the M3 one if the robot has front holes) |
| `stl/check_tile.stl` | 40 mm camera check tile, 4 x 4 squares |
| `stl/led_holder.stl` | three 5 mm LEDs in an L |

## What to measure before printing

Everything below goes into `dock_params.py` (in mm). Use calipers if you have them; otherwise a ruler read to
half a millimetre. Then rebuild (next section). Do the first four before anything else: they decide whether the
robot fits the dock at all.

1. **ROBOT_WIDTH_MM**: the widest point of the robot's front half, wheels included. Close the caliper jaws
   on the outside of both wheels (or the shell, whichever sticks out more). The dock's pocket is made 1 mm
   wider than this, so an error of 0.5 mm here either jams the robot or loses the dock's accuracy.
2. **AXLE_TO_FRONT_MM**: from the middle of a wheel's hub (the axle) to the flat front face, along the floor.
   Lay the robot on its side, put a ruler along it.
3. **TOF_WINDOW_BOTTOM_Z_MM** and **TOF_CENTER_Z_MM**: height above the table of the bottom edge and of the
   middle of the dark window on the front (the distance sensor). The bumper must stay below it.
4. **FRONT_FACE_BOTTOM_Z_MM**: height of the front face's lowest edge above the table.
5. **FRONT_M3_HOLES_MM** (M3 bumper): for each threaded hole on the front face, its sideways distance from
   the centre line (left positive) and its height above the table. If there are no such holes, use the Technic
   bumper.
6. **ROBOT_TOP_Z_MM** and **TECHNIC_HOLES_MM** (Technic bumper only): the height of the flat top near the
   front, and for one Technic hole on each side of the top: how far back from the front face, and how far
   left (+) or right (-) of the centre line.
7. **TOF_RECESS_MM**: how far the sensor's lens sits behind the front face (look from the side).
8. **The switches** (when they arrive): body length / thickness / height, the two mounting holes' diameter,
   spacing and how far they are from the terminal side, and the lever positions: lever free, where it clicks
   (press it slowly against a ruler edge and listen), and fully pressed. Pololu #1528 (13.5 mm lever) is
   12.8 x 5.8 x 6.5 mm per Pololu's page; the hole and lever numbers in the file are ASSUMED typical values.
   Note: Pololu **#1404 is a different, bigger switch** (20 x 6.4 x 10.2 mm roller lever); it does not fit this
   bumper.
9. **ROBOT_LENGTH_MM**: front face to rear face (used by the simulator only).

Also check: **BED_MM** (your printer's bed) and **LED_POS_MM** / **LED_MOUNT_TECHNIC_MM** if you use the LED
holder.

## Build the files

In a **Terminal window on the laptop**, in the project folder (the one with `tools/` and `hardware/` in it),
type each box and press Enter:

```
pip install cadquery matplotlib numpy
```

(On Linux, if pip refuses, add `--break-system-packages` to the end.) You should see "Successfully installed".

```
python hardware/dock/build.py
```

You should see one line per check, each starting `ok`, then three lines that start with `set ...`. A line
starting `FAIL` names the number to fix in `dock_params.py`; `WARN` means a part needs support material there.
The STL files are in `hardware/dock/stl/`, the pictures in `hardware/dock/preview/`.

Then, in the robot console (`python tools/alvik_console.py`), type the three `set ...` lines it printed
(they put the bumper's size into the robot), for each robot.

## Print settings

PLA, 0.4 mm nozzle, 0.2 mm layers, 4 perimeters (walls), 20% infill, no supports (except the Technic bumper).
Each STL already lies the way that needs no support; do not rotate it in the slicer.

| Part | Colour | Notes |
| --- | --- | --- |
| dock | **black** | the floor stripe is part of the dock: black reads like tape to the line sensors. Brim on (the long thin walls) |
| nose bumper | any | prints on its back (the face that touches the robot) |
| check tile | white, then black | it is 1.2 mm of base and 0.6 mm raised squares: add a filament change (pause, M600) at layer height 1.2 mm; the squares print black on a white tile. Or print it white and colour the raised squares with a black marker |
| LED holder | any | |

## Wiring (each robot)

Two switches, three wires each pin out; use two of them. With the switch's terminals facing you:

| Switch pin | Goes to |
| --- | --- |
| C (common, usually the pin at the hinge end) | **GND** on the Nano ESP32 |
| NO (normally open) | **D5** for the LEFT switch, **D6** for the RIGHT switch |
| NC (normally closed) | nothing |

(Left and right as the robot sees them: standing behind the robot, looking where it looks.) The pins are set
as INPUT_PULLUP: free = HIGH, pressed = LOW. **Check first** that D5 and D6 are free on your Alvik (nothing
else plugged into them; the Alvik's own connectors may use some Nano pins). If not, wire to free pins and type
`set sw_left_pin <n>` / `set sw_right_pin <n>` in the console (n = the D number).

Then check each switch. In the robot console (`python tools/alvik_console.py`), type one line at a time:

```
set sw_on 1
```
```
set dock_max_cm 6
```
```
run touch
```

with the robot on the table facing at least 10 cm of open table. It creeps forward slowly; press the LEFT
lever with a finger. It should stop at once and the console should print a TOUCH line containing `,L,touched,`.
Do it again pressing the RIGHT lever: `,R,touched,`. If it rolls 6 cm and prints `no click within 6 cm`, that
switch is not wired (or is on another pin). Afterwards: `set dock_max_cm 40`.

## Assembly

1. Push each switch into its pocket, lever to the front, terminals to the back; screw it down with two M2 x 8
   self-tapping screws through its holes into the shelf.
2. Solder 2 wires to each switch (C and NO), run them up through the notch behind it, to the Nano's pins.
3. Fix the bumper to the robot: two M3 x 8 screws (M3 version), or two Technic friction pins (Technic version).
   The bumper's back must sit flat on the front face; its bottom about 6 mm above the floor.
4. Press each lever with a fingernail: it must click before the lever reaches the bumper face, and the bumper
   face must stop a flat wall before the lever bottoms out (`build.py` checks this from the lever numbers).
5. Dock: put it on the plywood where robots can reach its mouth from the front, with at least 25 cm clear in
   front of the mouth. Screw it down through the four tabs (3.5 mm wood screws, countersunk). Measure where it
   is (next section).
6. Drop the check tile into the recess on top of the dock's back block.

## Where the dock is (the map)

Add it to the fleet map (`map.json`, docs/data-formats.md "Fleet map"):
`"home_docks": [{"name": "h1", "x": ..., "y": ..., "heading": ...}]`, where x, y is the middle of the back
wall's **inner** face (the face the bumper touches) and heading is the way a docked robot faces. Measure the
two inner corners of the back wall with the tape from the axes; x, y is their middle; heading is the direction
from the robot's side of the dock towards the wall. A site setup (tools/site/setup.py) then surveys it like a
board (it is a landmark `dock h1` there) and the locked map carries the measured pose.

## How it works (in short)

The robot creeps in at about 1 cm/s with both wheels equal and the front stop off. The funnel walls turn and
push it towards the middle; the pocket at the end is only 12 mm deep, so a robot reaches the back wall at any
angle (sqrt(96^2 - 95^2) = 13.8 mm is the most that still lets a crooked 95 mm robot touch the back wall). When
one switch clicks, that side keeps pushing gently and the other side pivots in until the second switch clicks;
both push a moment longer (the face seats flush), then it stops: square to the wall, centred to the pocket's
0.5 mm play. In the simulator, 60 dockings from hand-placed starts (1-8 cm before the mouth, up to 1 cm to the
side and 8 deg turned) all docked, with the true pose spread about 0.05 mm along the axis, 0.5 mm sideways
(the pocket's play) and 0.1 deg; that spread depends on the simulator's guessed contact model and is what
E00e measures for real.
