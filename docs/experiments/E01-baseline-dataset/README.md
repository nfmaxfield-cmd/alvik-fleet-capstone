# E1: Baseline data set

Status: planned    Week: 5 (Oct 17-18)    Run dates:

## Question
How far off does an Alvik end up using only its own sensors, and how do its
individual sensors (wheel encoders, gyro) each do? Every run also records the
full raw data needed to test sensor fusion and sensor dropouts later by
replaying it in software.

## Hypotheses
- (a) After a closed route, the robot's real return error is larger than its
  wheel odometry reports, because odometry cannot see wheel slip. H0: tape
  = odometry.
- (b) Error grows with distance and with the number of turns: the 3-lap
  square ends further off than the L-path. H0: no difference.
- (c) Gyro heading drifts less than wheel-odometry heading over the square's
  12 turns. H0: no difference.

## Variables
| Kind | Variable | Levels or units |
| --- | --- | --- |
| Independent | route | straight 100 cm; L-path (30 cm, right turn, 30 cm, then back); square 50 cm sides x 3 laps. Each starts at its own spot on the 122 x 122 cm board (start_x/y_cm on the sheet): straight (10, 61), L (40, 80), square (35, 35), robot centre, facing +x |
| Dependent | end position error, along and across the start heading | cm, tape |
| Dependent | end heading error | deg, tape |
| Logged | every sensor, 50 times a second (RAW): wheel angles and speeds, robot speed, odometry pose, accelerometer, gyro, IMU angles, 5-zone distance sensor, top/bottom distance, line sensors, battery, charging | robot log |
| Logged | battery and WiFi signal at start and end, every setting, firmware version (RUNINFO, PARAM, RUNEND) | robot log |
| Logged | clock checks robot <-> laptop every 10 s (clock_sync.csv) | console |
| Logged | overhead video, every frame timestamped; light flash at each run's start and end | Pi |
| Blocked by | robot (Left, Center, Right): each robot runs every route | |
| Held constant | surface, lighting, camera position, route settings | |

## Setup
- Robots: Left, Center, Right. Firmware: 2026-10-17b or later (shown by `status`, logged with every run). Center's motor-board firmware updated to 1.1.1 on Oct 17.
- Repo commit:
- Surface:    Lighting:
- Camera: height above floor ___ cm; area seen ___ x ___ cm; real frame rate ___ fps.
- Clear floor: the L-path swings 30 cm to the robot's right and the square
  50 cm to its left, so keep x -10..130 cm and y -40..60 cm clear and in the
  camera's view.

## How position is measured
1. Put a thin strip of tape on the robot's **front and rear faces**, on its
   centerline (once).
2. To mark the robot: hold a small square flat on the floor, upright edge
   against the front face at the tape strip, and pencil a dot on the floor.
   Same at the rear.
3. Measure each dot's x and y from the taped axes, to 0.1 cm.
4. Position = midpoint of the two dots; heading = direction from rear dot to
   front dot. With dots about 9-10 cm apart, 1 mm of measuring error is about
   0.6 deg of heading.

Do this **before** each run (start marks) and **after** it (end marks). Errors
are measured from where the robot really started, so a slightly crooked
placement does not count as robot error.

## Before the runs: pipeline check (today)
1. Power the robot on. Start the console (`python tools/alvik_console.py`) and
   on the Pi `python3 camrec.py --label pipeline`.
2. Send `calibrate`, then place the robot in view.
3. Three runs: `set straight_cm 60`, `run straight`. Watch for the white
   flash at the start and end of each.
4. Quit the console (`quit`), stop the recorder (Ctrl+C), copy the recording to
   the laptop, then:
   `python tools/check_session.py logs/<session> --video <recording folder>`
5. Pass if: sensor rows lost under 2%; no gap over 300 ms (the library's own
   move and turn commands pause the robot's logging about 200 ms each, by
   design); clock round trips under 50 ms; every flash found in the video;
   "az still" near 1.00 (confirms the accelerometer columns are in g).
   If rows are lost, `set raw_hz 25` and repeat. If flashes are not found,
   lower the camera or brighten the lights' view.

## Procedure (the 36 runs)
1. Mark the three start spots on the board (robot centre, facing +x):
   straight (10, 61) cm = (3 15/16, 24) in; L (40, 80) cm = (15 3/4, 31 1/2) in;
   square (35, 35) cm = (13 3/4, 13 3/4) in. Rough is fine: run_sheet checks
   the measured start is within 5 cm.
2. On the Pi (overhead camera, 25 fps), first go to the repo folder:
   `cd ~/alvik-fleet-capstone`
   then:
   `python3 tools/pi/camrec.py --device 0 --fps 25 --label E1`
   and, optionally, the second camera in another window:
   `python3 tools/pi/camrec.py --device 2 --port 8001 --notes-port 0 --exposure auto --wb auto --label E1-emeet`
3. On the laptop, in the repo folder:
   `python tools/preflight.py --robots Left,Center,Right --pi st1maxfini.local`
4. On the laptop:
   `python tools/run_sheet.py docs/experiments/E01-baseline-dataset --pi st1maxfini.local --inches`
   (`--inches`: type tape readings as you read them, e.g. `36 3/32, 42 15/16, 39 27/32, 42 13/16`).
5. It walks you through every run in order: the calibration run once per
   robot per power-on (robot in view, off its mark), placing the robot at the
   row's start spot, typing the start marks, starting the run, typing the end
   marks. It sends the commands, fills in the run id and battery, marks each
   run in the video, checks the numbers, saves after every run, and shows each
   run's error right away. Type q to stop; running it again continues.
6. Afterwards: stop the recorders (Ctrl+C), copy the robot log folder into
   `data/logs/` and the recording into `data/video/` (video stays out of git).

E1b (Right at low battery) runs first, with its own folder and commands:
see `docs/experiments/E01b-low-battery/README.md`.

Runs are grouped by robot (all of Left's, then Center's, then Right's) and
shuffled within each robot (seed 21), so battery drain and floor wear don't
line up with one route.

## Training video for the detector (after the runs)
10 minutes, no measuring: robots without tape strips if possible, all flashed
robots, driving the three routes from different start spots and headings, plus
robots standing still at various places. `python3 camrec.py --label training`.

## Trials
4 runs per route per robot, 36 in total (12 per route across the robots).
Twelve per route gives a 95% interval of about +/-0.6 cm on each route's mean
error if the spread is near 1 cm; 4 per robot shows whether robots differ.
After the first robot's 12 runs, check the spread and add runs if needed.

## Raw data
- `data/measurements.csv`: tape marks, one row per run, with run_id
- `data/logs/<session>/`: console CSVs (raw, runinfo, param, runend, sync,
  demo_step, demo_end, straight, clock_sync, session.json)
- `data/video/<recording>/` (not in git): video.avi, frames.csv, events.csv, meta.json

## Results
`python tools/tape_results.py docs/experiments/E01-baseline-dataset --logs docs/experiments/E01-baseline-dataset/data/logs/*`

## Analysis
- (a) paired t-test, tape vs odometry end error, per route
- (b) end error by route, Welch's t-test L vs square
- (c) heading after each of the square's 12 turns from gyro vs wheels, against
  the end heading by tape (needs the RAW log)

## Known limits of this setup
- The tape marks the midpoint of the robot's front and rear faces, not its
  wheel axle. If the two differ, any end heading error adds a small offset
  (offset x sin(heading error)) when tape and odometry positions are compared.
  Measure the face-midpoint-to-axle distance once and note it here: ___ cm.
- Route end headings: DEMO_END reports the IMU's heading, DEMO_STEP the
  wheels'. Both are kept (`imu_heading_err`, `wheel_heading_err` in results.csv).
- Sensor rows lost over WiFi are gone for good; the check reports how many.

## Conclusion

## Deviations
