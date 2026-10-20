# Home dock, nose bracket, check tile and LED holder (3D printed)

A printed **dock** fixed to the plywood gives a robot a near-exact position fix: the robot creeps in, a funnel
centres it, and two lever switches on a printed **nose bracket** (the bumper) tell it when it sits flush against
the dock's back wall. The same switches make the robot a **touch probe** (it can feel where a board is). A printed
**check tile** on top of the dock gives the overhead camera a spot that never moves (a bumped camera shows up at
once), a dark **floor stripe** at the dock's mouth gives the line sensors a second reading of where the robot
is, and an **LED holder** carries three LEDs in an L for identity and heading.

Firmware: `firmware/home_lab/switches.h`, `dock.h` (2026-10-19a; pins A0 / A1 from 2026-10-19b). Simulator:
`tools/sim/fw_dock.py`. Records: `docs/data-formats.md`, "Home dock and touch probe". Test: `docs/experiments/E00e-dock/`.

| File | What |
| --- | --- |
| `dock_params.py` | **every dimension** (mm), each robot number marked SOURCED (Arduino's files, with the link), MEASURED or ASSUMED |
| `build.py` | makes the parts from it: `stl/*.stl` (already lying the right way on the bed), `preview/*.png`, and a printability / fit report |
| `stl/dock.stl` | the dock: funnel, pocket, back wall, check-tile recess, floor stripe, 4 screw tabs |
| `stl/nose_bracket.stl` | the nose bracket: a bar across the robot's nose with the two switches, an arm down each side screwed into the robot's front side inserts |
| `stl/check_tile.stl` | 40 mm camera check tile, 4 x 4 squares |
| `stl/led_holder.stl` | three 5 mm LEDs in an L |
| `preview/nose_bracket_fit.png` | the bracket on the robot: side, top and front views |
| `preview/docked.png` | a docked robot and bracket in the dock, from above |
| `stl/dock_v2.stl` | **dock version 2** (`build_v2.py`): the same funnel for parking, plus a pair of measuring posts molded into each of its three closed sides; no floor stripe |
| `preview/dock_v2_layout.png` | version 2 from above with a robot parked and one at each post station |

## Dock version 2: park in the funnel, measure at the posts

Version 1's funnel pushes a crooked robot straight before its switches touch, so it parks precisely but
erases the robot's own error on the way in. Version 2 (`python hardware/dock/build_v2.py`) keeps the funnel
for parking and adds three measuring stations on the outside: on the back and on each side, two narrow posts
whose tips sit exactly one switch spacing apart (85.45 mm), square to the robot's approach, 10 mm proud of
everything else on that side.

- `run posts`: the robot creeps straight at a station; each lever clicks on its post. The travel at the first
  click gives the distance; the gyro's turn from the first click to the second gives the angle it came in at;
  a lever that misses its post (more than about 4.9 mm off sideways) says which way it is off. Two stations
  facing 90 degrees apart (the back and a side) give a full position.
- `run dock` (parking) now also logs the funnel's twist (`funnel_deg`, `twist_at_cm`): the approach angle
  the funnel erased is still recorded, from the gyro.
- **The posts are molded into the dock, not added on.** Their spacing and squareness come from the print (to
  a fraction of a millimetre) and can't drift or be knocked loose; the dock's position, which the camera
  watches through its check tile, carries the posts' positions with it; and there is no extra fixture to place,
  glue or keep aligned. One print, screwed down, gives a parking spot and three measuring stations.
- The head is 100 mm long (a whole station fits along each side) and is screwed down through four countersunk
  holes (#6 x 1 1/4 in wood screws); the funnel's two tabs sit part-way along its walls. 172 x 174 x 22 mm,
  prints flat without supports. No floor stripe: robots stay on the plywood.
- Each station needs about 25 cm of clear floor in front of it.

## Where the robot's numbers come from

The Alvik's front face has no screw holes; its **sides** have four brass M3 inserts each (two near the front, two
near the back, a LEGO Technic hole between each pair). The numbers come from Arduino's own files (links in
`dock_params.py`): the tech specs (96 x 95 x 37 mm), the datasheet (M3 inserts, screws no longer than M3 x 10), and
Arduino's CAD model of the shell (`AKX00066-step.zip`), measured in cadquery. With that model downloaded,
`python hardware/dock/build.py --ref alvik_reference_design.step` also checks that the bracket does not run into it.

## Check before printing (tape measure, 2 minutes)

The CAD model is Arduino's design, not your robot. Check these on one robot; if one is off by more than 1/32 in,
tell whoever keeps `dock_params.py` (the number to change is in brackets):

| Check | Expect |
| --- | --- |
| Width across the top edge of the shell (or across both tyres) | 3 25/32 in (96 mm) [ROBOT_WIDTH_MM] |
| The two brass inserts in the FRONT column on one side: centre to centre | 5/8 in (16 mm) |
| Lower front insert's centre above the table | 37/64 in, just under 5/8 (14.7 mm) [SIDE_INSERT_Z_MM] |
| Front column's centre behind the front face's top edge | 25/32 in, just over 3/4 (20 mm) [SIDE_INSERT_X_MM] |
| Top of the plain white band (bottom of the dark recess's rim) above the table | 15/32 in (11.7 mm) [BAND_TOP_Z_MM] |
| Bottom of the front face above the table | 11/64 in, about 3/16 (4.2 mm) [FRONT_FACE_BOTTOM_Z_MM] |

Also, when the switches arrive: body 12.8 x 5.8 x 6.5 mm (Pololu #1528, 13.5 mm lever), the two mounting holes'
spacing (6.5 mm assumed) and where the lever clicks (press it slowly against a ruler edge and listen). Pololu
**#1404 is a different, bigger switch** (20 x 6.4 x 10.2 mm roller lever); it does not fit.

## What to buy (per robot)

| Part | How many | Notes |
| --- | --- | --- |
| Pololu #1528 mini lever switch (13.5 mm lever) | 2 | |
| **M3 x 8 mm screws**, button head (ISO 7380) or pan head, stainless | 8 (one-piece attachment) | 3.3-4.1 mm of arm or leg (the side wall tapers), so 3.9-4.8 mm (8-9 turns) into the brass insert, whose hole is 5.6 deep. M3 x 10 would bottom out at the upper front pair. The 5 mm screws in the Alvik box are too short. Check on the first robot: about 8 turns of grip, and the head pulls the part flat |
| M2 x 8 self-tapping screws (or #2 x 5/16 in) | 4 | switch to bracket |
| Grove-to-female-jumper cable | 1 | GND from the Alvik's Grove connector |
| Hook-up wire, 26-28 AWG, with female jumper ends (or solder to the Nano's holes) | 4 | switch to Nano |

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

You should see one line per check, each starting `ok`, then four lines that start with `set ...`. A line
starting `FAIL` names the number to fix in `dock_params.py`; `WARN` means a part needs support material there.
The STL files are in `hardware/dock/stl/`, the pictures in `hardware/dock/preview/`.

Then, in the robot console (`python tools/alvik_console.py`), once the bracket is on, type the four `set ...`
lines it printed, for each robot (now: `set bump_ahead_cm 6.10`, `set bump_half_cm 4.27`, `set sw_gap_cm 0.020`,
`set bumper_cm 1.34`; the first three are already the firmware's defaults from 2026-10-19b). `bumper_cm` is how
far the bracket sticks out in front of the shell: the calibration run then keeps that much more room when it
rolls and spins. In the fleet map set `"robot": {"bumper_cm": "fitted"}` so routes allow for the longer and
wider robot, and give `run_sheet.py --bumper-cm 1.34` so its space instructions do. `python tools/preflight.py
--map <map.json>` checks that each robot's setting matches the map's.

## Print settings

PLA, 0.4 mm nozzle, 0.2 mm layers, 4 perimeters (walls), 20% infill, no supports.
Each STL already lies the way that needs no support; do not rotate it in the slicer.

| Part | Colour | Notes |
| --- | --- | --- |
| dock | **black** | the floor stripe is part of the dock: black reads like tape to the line sensors. Brim on (the long thin walls) |
| nose bracket | any | prints standing as it sits on the robot (bar and arms on the bed); the four screw holes bridge without support |
| check tile | white, then black | it is 1.2 mm of base and 0.6 mm raised squares: add a filament change (pause, M600) at layer height 1.2 mm; the squares print black on a white tile. Or print it white and colour the raised squares with a black marker |
| LED holder | any | |

## Wiring (each robot)

The Nano ESP32 on top of the robot has a row of breakout holes along its bottom edge: D13, 3.3V, B0, A0-A7,
VBUS, B1, VIN. The switches use **A0 and A1**; the LED holder (if used) **A2, A3, A7**. The Alvik itself uses D2,
D3, D4, A4, A5 and A6: leave those alone. GND comes from one of the Alvik's Grove connectors at the back through a
Grove-to-jumper cable (its **black** wire is GND).

Each switch has three pins; use two. With the switch's terminals facing you:

| Switch pin | Goes to |
| --- | --- |
| C (common, usually the pin at the hinge end) | **GND** (both switches' C wires join the Grove cable's black wire) |
| NO (normally open) | **A0** for the LEFT switch, **A1** for the RIGHT switch |
| NC (normally closed) | nothing |

(Left and right as the robot sees them: standing behind the robot, looking where it looks.) The pins are set
as INPUT_PULLUP: free = HIGH, pressed = LOW. The firmware's pin numbers are the Arduino IDE's default
numbering, **Tools > Pin Numbering > "By Arduino pin (default)"** (A0 = 17, A1 = 18): with the other setting the
upload stops with an error saying so. To use other pins type `set sw_left_pin <n>` / `set sw_right_pin <n>`
(D5 = 5, A0..A7 = 17..24).

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

1. Push each switch into its pocket at a front corner of the bracket, lever to the front, terminals to the back;
   screw it down with two M2 x 8 self-tapping screws through its holes into the shelf.
2. Solder 2 wires to each switch (C and NO). The pocket is open on top behind the switch: run the wires straight
   up there, up the flat white strip at that corner of the robot's front face (outside the dark recess: never
   across the recess, where the distance sensor looks out), over the top edge to the Nano.
3. Slide the bracket onto the robot's nose from the front: the bar goes on the plain white band below the dark
   recess, an arm along each side. Put an M3 x 8 screw through each of the four arm holes into the brass inserts
   behind them and tighten gently (snug, not hard: the inserts are in plastic). The bar's back must sit flat on
   the white band, its bottom about 5 mm (3/16 in) above the table.
4. Press each lever with a fingernail: it must click before the lever reaches the bracket's face, and the face
   must stop a flat wall before the lever bottoms out (`build.py` checks this from the lever numbers).
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
push it towards the middle. The pocket at the end fits the **bracket** (102.2 mm wide, the pocket 103.2), not the
bare robot: the bracket's arms make it 6 mm wider than the robot, and a printed part's width is exact where the
shell leans and the tyres squash. The pocket is only 12 mm deep, so the bracket reaches the back wall at any
angle (sqrt(103.2^2 - 102.2^2) = 14.3 mm is the most that still lets a crooked bracket touch it); only the
bar is ever in the pocket, and behind it the funnel has opened enough to clear the arms and screw heads by at
least the pocket's 0.5 mm (`build.py` checks it; `preview/docked.png` shows it). When
one switch clicks, that side keeps pushing gently and the other side pivots in until the second switch clicks;
both push a moment longer (the face seats flush), then it stops: square to the wall, centred to the pocket's
0.5 mm play. In the simulator, 60 dockings from hand-placed starts (1-8 cm before the mouth, up to 1 cm to the
side and 8 deg turned) all docked, with the true pose spread about 0.05 mm along the axis, 0.5 mm sideways
(the pocket's play) and 0.1 deg; that spread depends on the simulator's guessed contact model and is what
E00e measures for real.

## Light plate (camera lights): `build_light_plate.py`

A plate about 22 mm above the robot's top (the height Andrew Muszynski's AprilTag plate used), on two legs
screwed to the **back** pair of side inserts with M3 x 8 screws; the nose bracket keeps the front pair. Both
parts screw into the robot's own inserts, so lights, switches and robot are fixed to each other. Two parts
rather than one: a single part would need support under the switch bar.

It holds 13 SK6812 RGBW single-pixel boards (10 mm round, ASSUMED: measure one and change `PIXEL_DIA_MM`):
5 in a row across the front and 2 at the back (directional: bright bar = front), 6 ID lights in a zigzag across
the middle (17 mm between neighbours). Power: the 5 V pin of a servo connector (the robot's boost converter);
data: a Nano pin (3.3 V logic, at the edge of what 5 V pixels accept: check the first pixel, add a level shifter
if it misreads). Wire channels join them in data order from the cable hole; `stl/light_plate_layout.json` has every
light's position (robot frame, cm) and its index on the chain, for the camera software and the firmware.

Print: black, 0.20 mm layers, top face down as exported (the pockets' floors bridge 11 mm at most), no supports.
The plate covers the robot's top buttons and its own two top lights: runs start from the laptop, and the ID
lights replace the blink code (firmware for the pixel chain: to do).

## One-piece attachment: `build_attachment.py`

The nose bracket and the light plate as one printed part (`stl/attachment.stl`): two front legs rise from the
bracket's arms to the plate, so the plate stands on four legs and switches, lights and robot are fixed to each
other by the print. All eight side inserts are used (M3 x 8, four per side: the arms on the front pair, the back
legs on the back pair). It slides straight down over the robot from above. Print upside down (plate's top face
on the bed) with supports on (tree supports if offered): ~1450 mm^2 under the switch bar and the arms needs them.
