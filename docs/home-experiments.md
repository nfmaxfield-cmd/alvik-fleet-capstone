# Home experiments with one Alvik

Three steps: see what the sensors read, get data off the robot over WiFi, then
run three timed experiments that preview the lab's improvement ideas.

| Step | Code | What you get |
| --- | --- | --- |
| 1. Sensor explorer | `firmware/sensor_explorer` | The real readings for your floor, tape and stickers |
| 2. WiFi logger | `firmware/home_lab` + `tools/alvik_console.py` | Every reading saved to CSV on your laptop |
| 3a. Turn test | `run turn` | Turn accuracy and time, library `rotate()` vs IMU-feedback turns |
| 3b. Lap timer | `run lap` | Lap time and steering error vs speed and gains |
| 3c. Grid test | `run grid` | Time saved by driving through crossings instead of stopping |

## One-time setup

1. **Arduino IDE 2**: install the **Arduino ESP32 Boards** package (Boards
   Manager) and the **Arduino_Alvik** library (Library Manager). Board:
   **Arduino Nano ESP32**.
2. **WiFi secrets**: in `firmware/home_lab`, copy `secrets.example.h` to
   `secrets.h` and fill in your network. It must be 2.4 GHz. `secrets.h` is
   git-ignored, so it never reaches GitHub.
3. **Python 3.8+** on the laptop. No packages needed. Run the tools from the
   repo folder. The first time, Windows asks to let Python through the
   firewall: allow **private networks**.

## Flashing (each robot, same sketch)

Every robot gets the identical `home_lab` sketch. Each one names itself from
its WiFi hardware address (e.g. `AL-3F2A`), so there's nothing to edit per
robot.

1. Plug the robot's Nano ESP32 into the laptop by USB-C. Switch the robot's
   power switch **ON**: the sketch talks to the motor board at startup.
2. Arduino IDE: open `firmware/home_lab/home_lab.ino`, select **Arduino Nano
   ESP32** and its port, then **Upload**.
3. Open Serial Monitor (115200). Within ~15 s it prints
   `This robot is AL-xxxx` and `joined WiFi`. **Write the name on a piece of
   tape on the robot.**
4. If upload fails with "no device found", double-tap the Nano's reset button
   (its LED pulses) and upload again.
5. If Serial Monitor says the robot firmware doesn't match the library, see
   *Firmware mismatch* below before going further.

Flashing an Arduino sketch replaces the MicroPython the Alvik ships with.
That's expected. Arduino's MicroPython installer puts it back if you ever want
it.

**Firmware mismatch:** the motor board (STM32) runs its own firmware, and the
library expects a matching version. Updating it uses the library's
`bridge_firmware_updater` example plus STMicroelectronics' STM32CubeProgrammer.
The steps are in the [Arduino_Alvik README](https://github.com/arduino-libraries/Arduino_Alvik#how-to-update-firmware-of-arduino-alvik-carrier).
Do one robot first, and only if the warning appears.

## Updating firmware over WiFi (no cable)

> **Next USB flash (to do):** before uploading, add the `OTA_PASSWORD` line
> below to `secrets.h`, so every flash after that can go over WiFi.

Once each robot has been flashed by USB with firmware 2026-10-17c or later,
later updates can go over WiFi:

1. In `firmware/home_lab/secrets.h`, add a line with a password only you know:
   `#define OTA_PASSWORD "choose-a-password"`, then flash each robot by USB
   one last time. Serial Monitor shows `WiFi firmware updates on`.
2. With the robot switched on and idle, open **Tools > Port** in the Arduino
   IDE. Under "Network ports" it appears by name (e.g. `Left at 192.168.1.250`).
   It can take up to a minute to show up.
3. Pick it, click **Upload**, and type the password when asked. Both lights
   turn blue while it updates, then green; the robot restarts on its own.

Updates are refused while an experiment runs. If a robot doesn't show up as a
network port, the laptop and robot may be on different networks, or the
network blocks device discovery; the USB cable still works as before.

## Naming the robots: Left, Center, Right

In the console, point at each robot and give it its name once. It's saved
on the robot and survives power-off:

```
use AL-3F2A
identify          (that robot's lights flash white)
name Left
```

Repeat for `Center` and `Right`. `name` with nothing after it goes back to
the AL-xxxx name.

## Demo routines (open floor, no tape)

`run demo` runs a routine that depends on the robot's name:

- **Left:** forward 30 cm, turn right, forward 30 cm, then retraces in
  reverse (back 30, turn back left, back 30). It should end where it started,
  facing the same way.
- **Center:** forward 30 cm, one full circle of radius 20 cm, then back 30 cm
  to where it started.
- **Right:** no routine yet.

`set demo_route 1` (L), `2` (circle) or `3` (square, `square_cm` sides,
`square_laps` laps) runs that route on any robot; `0` goes back to choosing
by name. `run straight` drives `straight_cm` straight, with the library's
speed control (`straight_rpm 0`) or both wheels at a set rpm.

**Before any run you'll measure with a tape:** send `calibrate` first, with
the robot in the camera's view but off its mark. This is a logged calibration
run (about 65 s from firmware 2026-10-18c, 60 s in 2026-10-18b, 10 s before): the robot
stands still blinking its own light code (who is who), wiggles to check its
compass direction, then spins slowly twice around to the left and twice back
(`calib_spin_deg` 720, `calib_spin_rpm` 15: the gyro's scale, where its lights
sit relative to its turning centre, and a sweep of the front distance sensor
all the way round). From firmware 2026-10-18c it uses that sweep to choose
where to roll: it turns to the direction with room and rolls up to
`calib_roll_cm` (15 cm) forward and back there (which way it faces, and how far
its wheels really carry it, both seen by the camera), shorter if there is less
room, not at all if no direction has room for `calib_roll_min_cm` (5 cm); the
CALIB_LOOK record says what it chose. `calib_still_ms` (1 s) still periods
come in between for the gyro's drift. It needs a 25 cm (9 13/16") circle
around it to spin (it won't spin if something is within `calib_spin_r_cm`,
8 cm, of its middle) and, for the full roll, 35 cm (13 3/4") of clear floor
in some direction counted from its back (10 cm robot + 15 cm roll + 7 cm stop
distance + 3 cm margin). Keep it 25 cm from any open plywood edge: the front
sensor can't see an edge, so open space past it looks like room (the laptop
can cap the roll per direction with `set calib_dir_limits`, for the camera
map later; with the edge guard on, `safe_edge`, its forward roll stops at an
open edge, but the run then ends stopped, without the roll back). If it finds
no room to roll, or something sits inside its spin circle, it skips that part
and the run still ends `done`: that is not a failure but a third outcome,
"limited (no room)" (docs/data-formats.md, "Calibration outcomes").
run_sheet.py then says what was kept and lets you press r to move it and redo,
or Enter to continue. tools/blink/calib.py reads all of it from the video and the log
(calibration_results.csv). It doesn't end exactly where it started, so place
the robot on its mark afterwards. Turn, grid, demo and straight runs then
refuse to start until a robot has been calibrated since power-on
(run_sheet.py does this for you).

**During runs (firmware 2026-10-17e+):** both top lights blink the robot's
rhythm (one short flash every 0.3 s for Left, 0.4 s Center, 0.5 s Right; firmware 2026-10-18a+) so the camera can find
and name it on any floor; `set blink_codes 0` turns it off. Turn, demo and
straight runs also roll `start_nudge_cm` (2 cm) forward and back right after
the start flash, so the camera sees which way the robot faces; `set
start_nudge_cm 0` turns it off.

The measured experiments (E0, E1, ...) are set up in
[experiments/INDEX.md](experiments/INDEX.md).

## Survey drive: recalibrate a moved camera (firmware 2026-10-18b+)

`run survey` drives a square spiral outward from where the robot stands: legs
of 10, 10, 20, 20, 30, 30 ... cm with a left turn before each new leg, up to
`survey_size_cm` (80: 16 legs, about 7.7 m, just under 2 minutes, covering
about 1 m x 1 m around the start). It then turns to face the start, drives
straight back to it and turns to its starting heading. A leg ends early if
anything is within `survey_clear_cm` (10 cm) in the robot's path; the spiral
stops there and it drives back. Light: red. Needs the calibration run
(`calibrate`) since power-on, like turn, demo and straight runs. It is a logged
run (RUNINFO, PARAM, RAW at 50 a second with step = leg number, sync flashes,
blink codes, RUNEND) plus SURVEY_LEG and SURVEY_END records.

Then the camera's floor calibration comes from the survey itself: the robot's
own wheel + gyro path, hung from its taped start pose, paired with where the
camera saw it.

1. Pick a start spot near the middle of the floor the camera sees, with about
   50 cm clear all round. Put the robot there and mark its front and rear on
   the floor (pencil), as for E1. Record the marks in a sheet row (the E1
   columns: order, robot, level `survey`, commands `run survey`, xf0..yr0).
2. Start the camera recording, then `run survey` (one robot at a time).
3. When it is back, mark its end pose (xf..yr) in the same row and its run id.
4. Repeat from 2-3 other spots, each measured separately (see below).
5. Calibrate:

       python tools/calibrate_floor.py <recording> --survey --session logs/<x> \
           --sheet <that sheet> --detections <recording>/detections.csv --height-cm <tape>

run_sheet.py can run a survey row too, but it expects every start at the
origin; started there, half the spiral lies at negative x.

**How good (simulated, camera moved and turned 4 deg, 10 and 25 frames a
second):** the error comes almost all from the taped start headings (two marks
9.5 cm apart give about +/-1.3 deg), and the whole calibration turns with
them. Over everything the camera sees: 2 surveys, mean 0.8 cm (max 1.7) in one
session but 2.2 cm (max 4.6) in another, where both starts happened to be
taped about 1.5 deg off the same way; 4 surveys from separate spots, mean
0.6 cm (max 1.5); the 36 E1 runs, mean 0.2 cm (max 0.7) in both sessions.
With the start-heading rotation removed, the 2-survey calibrations were mean
0.17-0.37 cm: the spiral gets the shape right, the anchor sets the angle. So
for now the survey is a quick check or rough recalibration (4 runs, about 8
minutes), not a replacement for many taped runs.

## Running a fleet plan on the robots (firmware 2026-10-18b+)

`tools/fleet/run_plan.py` runs plan.json (tools/fleet/README.md): it starts a
`follow` run on every robot, releases each robot's steps as the others report
theirs done, passes camera fixes on, and re-plans when something new appears.

1. Every robot: `calibrate` (once per power-on), then put it on its start spot
   from map.json. Close the console (one program on the robots' port).
2. `python tools/fleet/run_plan.py --map map.json --plan plan.json`
   (`--calibrate` sends the calibrations for you; `--fixes udp` takes camera
   poses from the live fusion when that runs).
3. Ctrl+C stops every robot. The session folder has step.csv and follow.csv
   (what each robot did and where it thought it was) and coordinator.jsonl.

Try it first without robots: `python tools/fleet/run_plan.py --map
tools/fleet/examples/demo_map.json --dry-run` (simulated robots on this laptop;
add `--brick auto@15` or `--wifi-drop Center@20:3`).

**Full sensor log.** Every experiment except `stream` now logs every sensor
`raw_hz` times a second (default 50) to `raw.csv`, with a RUNINFO record before
the run (firmware version, battery, WiFi signal), every setting (`param.csv`),
and a RUNEND record after it. Both lights flash white for `sync_ms` at the
start and end of each run so video can be lined up with the robot's clock;
`set sync_flash 0` turns that off. The console adds the laptop's receive time
(`t_recv`) to every row and checks each robot's clock every 10 s
(`clock_sync.csv`). After a session, `python tools/check_session.py logs/<session>`
reports anything missing or out of line.

Give each robot at least a metre of clear floor. Center's circle swings 40 cm
to its left, and Left turns right, so keep Left's right side and Center's
left side clear of each other, or run them one at a time. Put a
piece of tape at each start point. Afterwards, measure how far each robot
stopped from its mark: the robot's own report is wheel odometry and can't see
slip. Distances and the circle size are settings (`demo_a_cm`, `demo_b_cm`,
`demo_r_cm`, `demo_speed_cms`).

## Plywood edge guard (firmware 2026-10-18b+, off until measured)

Where the plywood has no border board, the front distance sensor can't see the
small step down. The edge guard (`safe_edge`, firmware edge.h) can stop a
robot driving forward at an open edge from either of two cues: the line
sensors under the nose (`safe_edge 1`: their reading jumps when they pass the
edge; the painter's tape must not look the same) or the distance sensor's
bottom zone (`safe_edge 2`: it looks at the floor a few cm ahead, which drops
to the cement past the edge), or both (`3`). It is **off** until the E0c edge
rows are in: then `python tools/edge_check.py` says whether each cue separates
the edge from plywood and tape and prints the `set` commands to try. Try them
on one robot first, driving slowly at an open edge and across the tape. In the
simulator (guessed readings) the line cue stops with about 3 cm of wheel still
on the board, the bottom zone about 9 cm. Limits: backing up and turning in
place are not guarded, and a robot meeting the edge at a shallow slant (under
about 45 degrees) puts a wheel over it before the line sensors get there.
Lap and grid runs on dark tape: dark tape may look like the edge to the line
cue; use `safe_edge 2` or `0` for those. The calibration run is guarded only on
its forward roll (the roll its look-around chose, which can't see an open
edge): a trip there ends the run `stopped:plywood edge ahead` without the roll
back; calibrate again further from the edge.

## Step 1: sensor explorer

Upload `sensor_explorer`, open Serial Monitor at 115200, and switch the robot
on. Hold it still over each surface and note the readings:

| Surface | L | C | R | H | S | V |
| --- | --- | --- | --- | --- | --- | --- |
| Floor | | | | | | |
| Black tape (centered) | | | | | | |
| Tape crossing | | | | | | |
| Marker sticker | | | | | | |

Then set these in `home_lab/params.h`, or live with `set`:

- `lf_line_thr`: halfway between the floor and tape readings.
- `mk_h_lo` / `mk_h_hi`, `mk_s_min`, `mk_v_min`: a window around the
  sticker's H/S/V that black tape's readings fall outside. Red wraps past 0,
  which is why the default window runs 340 to 20.

If the first line says **MISMATCH**, update the robot's firmware with the
library's `bridge_firmware_updater` example before going on.

## Step 2: WiFi logger

1. Switch the robots on.
2. On the laptop: `python tools/alvik_console.py`
3. Each robot announces itself: `[AL-3F2A is talking from ...]`. Type
   `robots` to list them. `use AL-3F2A`, then `identify`, flashes that robot's
   lights white so you can match names to robots.
4. `use all`, then `run stream`. Move a robot by hand for 30 s, then `stop`.
5. `stats` shows packets dropped per robot. Under 1% is fine. Each session is
   saved to `logs/<date-time>/`, one CSV per record type, with a `robot`
   column.

Commands go to the robot(s) chosen with `use` (default: all). `AL-3F2A: run
turn` sends one command to one robot; the last 4 characters alone work too
(`3F2A: run turn`). `stop` always stops every robot. Robot commands: `help`,
`params`, `get <name>`, `set <name> <value>`, `route <letters>`,
`run stream|turn|lap|grid|demo|straight|survey|follow`, `stop`, `status`, `identify`,
`calibrate`, `clock`, `name <Left|Center|Right>`. Console-only:
`robots`, `use`, `stats`, `quit`.

**Several robots at once:** the turn test is safe to run on all of them
together (they turn in place), and it shows how much robots differ from
each other. The lap and grid tests have no collision avoidance, so run
those one robot at a time. The console asks you to confirm if you try them
on all robots. Without a laptop, each robot's buttons work:
LEFT/RIGHT pick an experiment (left LED: blue = stream, green = turn,
yellow = lap, purple = grid), OK starts, and CANCEL stops.

Afterwards: `python tools/summarize.py logs/<date-time>` (or `logs/*`).

## Step 3a: turn test

**Question:** how fast can the robot turn 90° and still land within 1–2°, and
does the library's `rotate()` or an IMU-feedback turn do better?

1. Put the robot on a smooth floor with a tape line under its nose to judge
   the final heading by eye.
2. `set turn_n 12` (a multiple of 4, so it should end where it started).
3. `set turn_method 0`, then `run turn`, 3 times.
4. `set turn_method 1`, then `run turn`, 3 times each at `turn_max_rpm` 25, 40
   and 55.
5. Summarize. Compare `mean_|err|` and `time_ms` across rows. `odo_err` is the
   same miss judged by wheel odometry. If it disagrees with the IMU, one of
   the two is off; your eye on the tape line breaks the tie.

## Step 3b: lap timer

**Question:** how fast can the robot follow tape before it loses the line, and
what does better steering buy?

1. Tape a loop about 1 m × 0.6 m with rounded corners. Put a short marker
   sticker (under 2 cm along the tape) on one straight.
2. Place the robot anywhere on the tape and `run lap`. Timing starts at the
   first sticker crossing; it then times `lap_n` laps.
3. Repeat at `lf_base_rpm` 20, 30, 40, 50 and 60 until it loses the line.
4. At the fastest speed that still works, try `lf_kp` and `lf_kd` changes.
   `set lf_trace 1` logs every control step for a closer look.

## Step 3c: grid test (the capstone preview)

**Question:** how much time does rolling straight through crossings save
compared with stopping at every one, the way the lab robots do?

1. Tape a 3×3 grid of crossings with squares of **at least 15 cm**. Below about
   9 cm the robot can miss crossings.
2. Start the robot on the tape before the first crossing, facing along it.
3. `route SSLSE` (straight, straight, left, straight, end), or any route that
   fits your grid.
4. `set grid_mode 0`, then `run grid`, 5 times. Put it back at the same start
   each time.
5. `set grid_mode 1`, then `run grid`, 5 times.
6. Summarize. The grid table gives the saving per run with a 95% interval.
   *An interval that stays above 0 means the saving is unlikely to be luck. It
   comes from Welch's t-test, which compares two averages without assuming
   they vary equally.*

If the robot stops a little before or after the crossing's center when it
turns, adjust `grid_center_cm`. That's the distance from the line sensors to
the wheel axle, and 3.5 cm is a guess.

## A library bug to know about

`alvik.rotate(angle)` in blocking mode waits `round(angle / 100)` seconds and
then checks for "done". For right turns of about 50° or more that number is
negative, and the comparison then waits for a huge unsigned time: the call
never returns. This is from reading Arduino_Alvik 1.1.1. Our code always uses
the non-blocking form. Worth confirming with one call, and reporting upstream
if it hangs.

## New tests that may now be needed

- Confirm the blocking `rotate(-90)` hang on your robot, as a one-line sketch.
- Measure `grid_center_cm` with a ruler (line sensors to wheel axle) and
  re-run the grid test.
- Measure how often the IMU yaw updates (STREAM log, robot turning slowly).
  If it's slow, IMU turns may hunt; lower `turn_min_rpm` if turns time out.
- Repeat the turn test on a different floor (carpet vs hard floor), since
  wheel slip changes both time and the odometry check.
- Check the lifted and front-obstacle safety stops don't trip during normal
  runs. Set `safe_lift 0` or `safe_front_cm 0` if they do.
- Edge guard, once tools/edge_check.py recommends settings: drive slowly at an
  open edge (head-on and at 45 and 30 degrees), across the painter's tape, and
  fast over plain plywood (the bottom zone tilts when the robot speeds up or
  brakes); measure how much wheel is left on the board each time.

## Possible inaccuracies in what this relied on

- **Every default in `params.h` is a starting guess**, not a measurement: the
  line threshold (300), marker color window, steering gains (kp 30, kd 0.5),
  turn speeds and the 3.5 cm sensor-to-axle distance.
- **The turn test grades the IMU method with the IMU itself.** The odometry
  column and your eye on the tape line are the independent checks.
- **Library `rotate()` timing includes about 300 ms of waits inside the
  library** (200 ms before sending, 100 ms on finishing).
  `lib_overhead_ms` shows the first part.
- **The grid test only approximates the lab.** Lab robots stop using camera
  positions and a 0.3 s settle; here the stop is triggered by the line sensors
  and the pause is `grid_stop_ms` (default 300 ms, matching the lab's settle).
- **Home results won't carry over 1:1.** The floor, tape, lighting and
  battery level all differ from the lab testbed. The direction and rough size
  of an effect are what transfer.
- **Untested on hardware.** The sketches were type-checked against stand-in
  headers built from the real library, and the laptop tools were tested
  against a simulated robot, but nothing has run on an Alvik yet.
