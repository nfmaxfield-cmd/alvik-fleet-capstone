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

Give each robot at least a metre of clear floor. Center's circle swings 40 cm
to its left, and Left turns right, so keep Left's right side and Center's
left side clear of each other, or run them one at a time. Put a
piece of tape at each start point. Afterwards, measure how far each robot
stopped from its mark: the robot's own report is wheel odometry and can't see
slip. Distances and the circle size are settings (`demo_a_cm`, `demo_b_cm`,
`demo_r_cm`, `demo_speed_cms`).

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
`run stream|turn|lap|grid`, `stop`, `status`, `identify`. Console-only:
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
