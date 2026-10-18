# E1 session simulator

Makes a complete, fake E1 recording session, so the analysis, fusion and
detector code can be built and tested before (and alongside) real runs, against
a ground truth that the real setup can never give.

    python tools/sim/simulate.py --out sim_out                 # console logs + tape sheet + truth
    python tools/sim/simulate.py --out sim_out --video         # + overhead camera recording
    python tools/sim/simulate.py --out sim_out --robots Left,Center,Right --seed 3
    python tools/sim/simulate.py --out sim_out --kidnap 5      # robot picked up and moved during sheet row 5
    python tools/sim/simulate.py --write-config my.json        # every assumed number, to edit
    python tools/sim/simulate.py --out sim_out --config my.json
    python tools/sim/simulate.py --out sim_out --video --config tools/sim/configs/real_oct18.json   # the real Oct 18 camera
    python -m unittest discover tools/sim/tests -v             # tests (about 30 s)
    python tools/sim/fleet_sim.py --map tools/fleet/examples/demo_map.json   # live simulated robots (see below)

Needs Python 3.8+, `numpy` and `opencv-python` (`pip install numpy opencv-python`).
A 36-run session takes about 10 s without video and about 4.5 minutes with it
on a 2-core machine (`--fps 15` halves the video time). The video is large:
about 25 MB per minute. `check_session.py --video` then needs about 16 minutes
for the 38-minute video.

## What it writes (under `--out`)

| Path | What | Same as the real thing? |
| --- | --- | --- |
| `session/` | console log folder: raw, runinfo, param, runend, sync, clock, clock_sync, straight, demo_step, demo_end CSVs, session.json | Yes: written by `tools/alvik_console.py`'s own `Session` class, fed the simulated packets at their simulated arrival times |
| `recording/` | `video.avi` (MJPG 640x480), `frames.csv`, `events.csv` (a "trial N" note before each run, and run_sheet.py's "calibration <robot> run <id>" after each calibration run), `meta.json` | Yes, camrec.py's format |
| `measurements.csv` | the tape sheet with start and end marks and run_id filled in | Yes, same columns as the E1 sheet |
| `truth.csv` | `t_laptop,robot,run,x_cm,y_cm,heading_deg,lifted`, 50 per second during every run (calibration runs included) | Simulation only |
| `camera_truth.csv` | `frame,t_wall,robot,x_px,y_px,heading_img_deg,led_on`: where each robot's top centre really is in every frame; `led_on` = 1 if either LED is lit (any colour) at the moment the frame is taken | Simulation only |
| `camera_detections.csv` | `frame,t_wall,x_px,y_px,score`: a pretend detector: noisy, anonymous, with misses and false alarms (frames with nothing detected have no row) | Simulation only |
| `floor_calibration.json` | the floor cm -> image px homography, plus the full camera model | Simulation only |
| `sim_info.json` | each robot's true wheel sizes, track, gyro errors, clock drift, `blink_code`; every sheet run's (`runs`) and calibration run's (`calib_runs`) true start and end pose and `blink_start` (laptop time the blink cycle counts from); kidnap details; the whole config | Simulation only |
| `console_output.txt` | what the console would have printed | |

The simulation-only files sit beside `session/` and `recording/`, not in them, so
those two folders look exactly like real ones to every tool.

Conventions: floor x along the taped x axis, y to its left, heading 0 = +x,
counter-clockwise positive (data-formats.md). `truth.csv` gives the **wheel-axle
midpoint** (the point odometry tracks); the tape marks are at the body centre
+/- length/2, which is the same point unless `axle_to_center_cm` is set. In the
image, +x is right and heading is counter-clockwise on screen (data-formats.md
"Labels"); camera positions are the robot's **top** centre, which is about 4 cm
above the floor, so it sits slightly further from the image centre than the
floor point below it (up to about 2 cm at the image edges).

## What is simulated

**Runs.** One sheet row = one run, in sheet order, using that row's robot and
commands. The firmware's run sequence is copied step by step: RUNINFO, every
PARAM (in params.h's order, read from the file), RAW logging starts, settle
500 ms, start flash (150 ms dark, `sync_ms` white, 150 ms dark), blink codes
start, start nudge, the experiment, blink codes stop, end flash, RUNEND. The straight run (`libraryMove`), the L route,
the square and the circle follow experiments.h. Library `move()` blocks 200 ms
before it starts and the successful `is_target_reached()` takes 100 ms, so the
RAW log has those gaps, as on the robot. RAW rows come from the same
"at most one per period, no catching up" rule as rawlog.h, sampled from control
loops that take 5-12 ms. The LED colours follow `showSelection()`: demo runs
are white, straight runs cyan; a calibration run keeps the chosen experiment's
colour; idle shows that colour on the left LED only. Between runs a hand picks the robot up and puts it back on the
start, with a small placement error; robots that are not running are parked
out of view.

**Calibration run** (firmware 2026-10-17e). As run_sheet.py does, each robot
gets one `calibrate` before its first sheet row: it is put down by hand
somewhere in view off the start mark (`calib_area_cm`, any heading), `calibrate`
is sent `calib_wait_s` later, and it runs as a logged run with experiment
`calib` (firmware 2026-10-18c, fw_look.py, a copy of look.h and runCalib):
RAW step 1 standing still `calib_blink_ms`, 2 the compass wiggle
(`calibrateYawSign(true)`: 300 ms at -20/+20 rpm, brake 300 ms, back 300 ms,
brake 300 ms, logged throughout), 3 still, 4 `spinBy(+calib_spin_deg)` (wheels
at -/+`calib_spin_rpm`, counted by the gyro heading, brake 1 deg early, front
stop off; the same turn-scrub slip model as other in-place turns), 5 still,
6 `spinBy(-calib_spin_deg)`, 7 still, 8 an IMU turn to the roll heading it
chose from the front sensor's sweep during the spins + still, 9 `move(+n)`
and 10 `move(-n)` (n = the roll that fits there, up to `calib_roll_cm`),
`calib_still_ms` still after each. The sweep sees `world_polys` (a `--map`'s
boards, or boards a test places) and the room walls; tools/sim/tests/test_look.py
puts boards around the robot. About 62-65 s with the defaults and a roll
(the 1 s `settle_end_ms` at the end included); one CALIB_LOOK record per run
(docs/data-formats.md), whose spin / roll / reason columns say what was
skipped or cut short (RUNEND still `done`: the "limited" outcome, not a
failure; with a board and `safe_edge` on, tests/test_edge.py has the edge
guard stop the forward roll).

**RAW raw_n and SYNC ms_on_after** (2026-10-18b): RAW rows carry `raw_n`
(rows sent before this one in the run), so lost rows show as jumps; SYNC
records carry `ms_on_after`, equal to `ms_on` here (the simulator does not
model how long the light-switching calls take). Its RUNINFO has `yaw_sign` 0 (not
checked yet); later runs have the sign it found. `after_calib_s` after its
RUNEND the robot is carried to the start mark (the same hand carry as between
runs). Turn, grid, demo and straight runs are refused before a calibration,
as on the robot.

**Start nudge.** Turn, demo and straight runs: right after the start flash,
RAW step -1, `move(+start_nudge_cm)`, 200 ms, `move(-start_nudge_cm)`, 300 ms,
then step 0 and the experiment (which zeroes odometry). Same slip model as
every move.

**Blink codes** (blink.h, 2026-10-18a rate codes). From the start flash to the
end flash both LEDs blink white: dark for 300 ms after `blink::start()`, then
one 100 ms flash every (n + 2) x 100 ms: Left 0.3 s, Center 0.4 s, Right
0.5 s, others 0.6 s (`blink_code` 1-4 overrides). The lights only change
when `rawlog::tick()` runs (every 5-14 ms in the control loops), go dark
(`hold()`) right before each library `move()` and stay so through its 200 ms
block (which drops a flash, or starts one late). Nothing darkens them before
the successful `is_target_reached()`, which blocks 100 ms, so a flash lit then
is stretched: the longest in 100 simulated runs was 211 ms, well short of the
300 ms sync flash. Flash-to-flash period (true LED timeline, same runs):
median exactly the code's period; 5th-95th percentile Left 291-345 ms,
Center 391-410, Right 491-510, others 569-659. Single intervals spanning a
move's block can be off by up to a period, so read identity from several
flashes. (Firmware 2026-10-17e's 200 ms pulses were stretched to 302 ms and
merged, and check_session.py missed 3 of 14 sync flashes.)

**Where the errors come from.** Moves stop when the wheel *encoders* reach the
distance, so along-track error comes from wheel radius error and slip, and a
left/right radius mismatch curves the path. Turns stop when the *gyro* says
the target heading is reached, so the real heading error comes from gyro bias
and scale error, not from the wheels. In-place turns scrub the wheels, so
wheel odometry over-counts every turn. The robot's own odometry (RAW
`x_cm,y_cm,th_deg`, DEMO_STEP, STRAIGHT, DEMO_END) is computed from its
quantized encoders with the *nominal* radius and track, separately from the
true motion. Odometry heading is not wrapped (a 3-lap square ends near 1080).

**Clocks and WiFi.** Each robot has its own boot time and a crystal error
(+/-30 ppm). The console's clock check (bursts of 3 every 10 s) is simulated,
including answers delayed while the robot is busy (PARAM burst, library
blocking, the sync flash). Packets arrive after 2-10 ms, occasionally much
later; RAW rows are lost at random. The Pi's clock is 40 ms ahead of the laptop's
and frames arrive about one frame after they are taken.

**Kidnap test** (`--kidnap ORDER` or `kidnap.run_order` in the config).
`at_s` (3 s) after the run's motion starts, the robot is lifted for 2 s
(RAW `lifted` = 1, `truth.csv` `lifted` = 1) and set down 20-40 cm away,
turned 30-120 degrees. Two modes:
- `pause` (default): the wheels stop while lifted, odometry doesn't change,
  the gyro feels the rotation, and the route carries on afterwards. The firmware
  stops any run when lifted unless `safe_lift` is 0, so the simulator adds
  `set safe_lift 0` to that row's commands (and the sheet). **Note:** with
  `safe_lift 0` the real firmware does NOT stop the wheels: `move()` keeps
  turning them in the air, so real odometry *would* change. "Wheels stop" is
  the requested test case, not what the firmware does today.
- `abort`: firmware as it is (`safe_lift 1`): the run stops with
  `stopped:robot lifted`, the end flash still runs.

**Camera.** Pinhole camera 115 cm up, giving 4 px/cm on the floor (640 px
across 160 cm), slightly turned (1 deg) and tilted (under 1 deg), optional
barrel distortion (`k1`). Floor with texture, uneven light and vignette; the
tape axes; robots drawn from above (rounded shell, wheels at the sides, light
top, dark front window, two LEDs near the front); hands that carry robots,
random hands reaching in (about 0.6 a minute) and an occasional box. LED colour
in each frame is the average over a 12 ms exposure. Detections: 1.5 px noise,
3% missed (60% when a hand covers the robot), false alarms on hands, boxes and
random clutter, and an optional `blind_region_px` where nothing is detected.

**Survey runs** (firmware 2026-10-18b, `run survey` in a sheet row): the
square spiral and the drive back, copied from experiments.h runSurvey() (and
pose.h's own pose and straight drives) in fw_follow.py. Optional sheet columns
start_x_cm, start_y_cm, start_heading_deg give that row its own start spot
(e.g. the middle of the camera view). SURVEY_LEG and SURVEY_END records as on
the robot.

**Front distance sensor.** Free-running at `tof.hz` (15) frames a second: a
reading is the newest frame, measured from where the robot was when the frame
was taken (on average 33 ms old). Each of the five zones reports the nearest
thing on 3 rays across its `zone_width_deg` (11 deg) slice. Below
`min_range_cm` (2) the reading is any value under 2 cm. It sees the room walls,
plus `Robot.world_polys` (boards and obstacles, set by fleet_sim.py) and the
other robots (`Robot.others`). Before 2026-10-18b it was one fresh ray per zone
on every call; RAW dC values now repeat between frames, as on the robot.

**Plywood board and edge guard** (firmware 2026-10-18b, edge.h). Off unless
`session.board_cm` ([x0, y0, x1, y1]) is set: then `session.open_edges`
("x0", "x1", "y0", "y1": sides with no border board) and `board_thick_cm` (1.9)
decide what the line sensors and the distance sensor's bottom zone see past an
open side. Line sensors there read `line.past_edge_reading` (1000, noise 40),
blended across the sensor's footprint; the bottom zone becomes geometric
(`tof.sensor_height_cm` 2.6, `bottom_deg` 16.8, spans 5.6 x 45 deg, nearest of
5 x 3 rays: 9.0 cm on plywood, about 15.8 cm once its whole floor patch is over
the cement). All GUESSES until E0c (edge rows, sensor height, plywood
thickness). Nothing falls off: tests check the wheel positions. The guard
itself (`safe_edge` and the `edge_*` settings) is copied from edge.h, with its
own random stream so switching it on doesn't change other draws;
tests/test_edge.py covers stopping distance, tape, calibration and survey runs.

## Live fleet simulation (fleet_sim.py)

Simulated robots that run the firmware's `follow` and `survey` code in real
time on real UDP, so tools/fleet/run_plan.py (and tools/alvik_console.py) talk
to them exactly as to the real robots. `run_plan.py --dry-run` starts it.

- Robots listen on 127.0.0.2, 127.0.0.3, ... port 5006 (map order) and send to
  127.0.0.1:5005; each robot's firmware loop (commands, `run follow`,
  `calibrate`, `status`, `clock`, `set`) runs in its own thread, paced to the
  wall clock. They start on their map start spots +/- `--start-noise` (0.5 cm,
  1.5 deg), calibrated unless `--uncalibrated`.
- The follower is fw_follow.py, a line-for-line copy of follow.h and pose.h:
  step buffer, acks, own pose with camera fixes, straight drives, IMU turns,
  the approach (readings + wheels), front checks, hold/halt, WiFi-quiet handling.
- Camera fixes: true pose + noise (`--fix-sigma` 0.3 cm, `--fix-sigma-deg` 1),
  `--fix-hz` 10 per robot, sent `--fix-delay` 0.1 s late as JSON to run_plan's
  position-source port (docs/data-formats.md, "Step follower").
- `--brick auto@T | x,y[,deg]@T`: a 20 x 9 cm brick appears T s after the first
  step (auto: on a robot's planned path ~3 s ahead, away from everyone); the
  front sensors see it at once, the camera reports it `--cam-delay` (0.5) s
  later (999 = never: only the robots' sensors can find it).
- `--wifi-drop NAME@T:DUR`: every packet to and from that robot is lost.
- Writes `truth.csv` (20 a second), `sim_report.json` (each docking measured on
  the true pose 0.3 s after the stop: front gap, sideways, heading; closest two
  robots; closest any robot came to a board it was not docking at or to the
  brick; contact; packets lost; the events) and `sim_log.txt`.

## Assumed numbers (replace with measurements)

Every number below is a guess unless marked "firmware". All of them are in
`config.py` and can be changed with `--config`. "+/-" values are drawn per
robot, uniformly.

| Setting | Value | Notes |
| --- | --- | --- |
| wheel radius (nominal) | 1.7 cm | **to be measured**; the robot's odometry uses it |
| wheel track (nominal) | 9.0 cm | **to be measured** |
| robot length / width / height | 9.5 / 9.5 / 4.0 cm | length sets the tape marks (+/- L/2) |
| body centre ahead of axle | 0 cm | E1 README asks for this to be measured |
| real radius vs nominal | +/-1.0% | both wheels together |
| left/right radius mismatch | +/-0.5% | curves straight runs |
| real track vs nominal | +/-3% | |
| move speed / ramp | 10 cm/s / 20 cm/s^2 | library default speed not documented |
| motor lag / brake lag | 50 / 30 ms | |
| wheel wander around move profile | 0.5 deg sd, 0.2 s | |
| slip in moves | 0.2% +/- 0.05% per wheel per move | |
| slip in in-place turns | 3% +/- 1% per turn | **big effect**: wheel odometry over-counts each 90 deg turn by ~2.7 deg |
| encoder resolution | 0.36 deg | |
| wheel speed noise | 0.4 rpm | |
| control loop extra time | 1.5-3.5 ms | on top of the firmware's delay(5)/delay(10) |
| library blocking | 200 ms per move, 100 ms at "reached" | from motion.h comments |
| IMU turn gains, tolerance, speeds | firmware | params.h (turn_kp 1, 10-40 rpm, 1.5 deg, 80 ms) |
| gyro bias | 0.05 deg/s sd per boot, wanders 0.002 deg/s per sqrt(s) | left over after the board's calibration |
| gyro scale error | +/-0.5% | |
| gyro noise | 0.07 deg/s per sample | |
| yaw sign | +1 (left = positive raw yaw) | unknown on hardware; the firmware measures it |
| accelerometer | noise 0.004 g (+0.01 g moving), bias 0.01 g sd | units g, az about 1 |
| ToF zones | +22, +11, 0, -11, -22 deg, 4.5 cm ahead of axle (0.25 cm behind the 4.75 cm front face) | noise 0.5 cm + 1%, max 350 cm; 15 frames/s; 11 deg per zone; untrustworthy below 2 cm |
| room walls (ToF) | x -80..220, y -120..140 cm | |
| line sensors | 3.5 cm ahead, 1.2 cm apart; floor 150, tape 850, past an open edge 1000 | tape axes 1.9 cm wide |
| battery | starts 90-100%, -3%/h idle, -20%/h moving | |
| WiFi RAW loss | 0.5% | bursts off by default (`burst_per_min`) |
| one-way delay | 1.5 ms + exponential 3 ms, 1% held 20-120 ms | |
| clock answers | 3% delayed 60-250 ms, 0.5% lost | |
| robot clock error | +/-30 ppm | |
| robots switched on before session | 1-15 min | |
| Pi clock vs laptop | +40 ms | |
| capture delay | 1 frame + 2 ms jitter | |
| frame drops | 0.2% | |
| exposure | 12 ms | |
| camera height / view | 115 cm / 160 cm across | 4 px per cm |
| camera yaw / tilt | 1.0 / (0.8, -0.5) deg | |
| placement error at start | +/-0.5 cm, +/-1 deg | |
| calibration spot | x 20-50, y 20-45 cm, any heading | "in view, off the mark"; the operator's choice |
| calibration pauses | 2 s put down -> `calibrate`, 2 s RUNEND -> picked up | run_sheet.py waits for Enter |
| tape measuring noise | 0.15 cm sd per coordinate, rounded to 0.1 cm | gives ~1.3 deg heading noise per pose |
| time between runs | 6 s (end marks) + 3 s (carry) + 8 s (start marks) + 1.5 s per command | real sessions are slower; only affects session length |

## What it does not simulate

- Wheels spinning in the air when lifted with `safe_lift 0` (see Kidnap).
- A failed compass check (`yaw check failed`): the wiggle always turns enough here.
- The robot rebooting mid-session (one calibration per robot).
- Collisions (robots pass through things; fleet_sim.py reports contact from the
  true outlines), the camera's box being seen by the ToF.
- Line-following experiments (lap, grid) and the turn test: only `straight`,
  `demo` and `calib` runs are simulated (the start nudge code covers turn
  runs, but no turn run is simulated).
- Camera motion blur, auto-exposure changes, and the robot's own shadow
  under a lifted robot.
- Lost non-RAW packets (headers, RUNEND) unless `burst_per_min` is turned on.
