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
| Independent | route | straight 120 cm; L-path (30 cm, right turn, 30 cm, then back); square 50 cm sides x 3 laps |
| Dependent | end position error, along and across the start heading | cm, tape |
| Dependent | end heading error | deg, tape |
| Logged | every sensor, 50 times a second (RAW): wheel angles and speeds, robot speed, odometry pose, accelerometer, gyro, IMU angles, 5-zone distance sensor, top/bottom distance, line sensors, battery, charging | robot log |
| Logged | battery and WiFi signal at start and end, every setting, firmware version (RUNINFO, PARAM, RUNEND) | robot log |
| Logged | clock checks robot <-> laptop every 10 s (clock_sync.csv) | console |
| Logged | overhead video, every frame timestamped; light flash at each run's start and end | Pi |
| Held constant | robot (Left), surface, lighting, camera position, route settings | |

## Setup
- Robot: Left. Firmware: 2026-10-17a (printed by `status`, logged with every run).
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
5. Pass if: sensor rows lost under 2%; no gap over 200 ms; clock round trips
   under 50 ms; every flash found in the video; "az still" near 1.00.
   If rows are lost, `set raw_hz 25` and repeat. If flashes are not found,
   lower the camera or brighten the lights' view.

## Procedure (the 24 runs)
1. Start the console and the recorder (`python3 camrec.py --label E1`).
2. Send `calibrate` once after power-on, before placing the robot.
3. For each row of `data/measurements.csv`, in order:
   1. Place the robot at the origin facing +x. Mark and measure the start dots
      (xf0..yr0).
   2. Type the trial number into the recorder window (e.g. `trial 4`).
   3. Send the row's commands. Write the run id from the console
      (`run 3F2A-7: demo starting`) into the sheet.
   4. When it finishes (second flash), mark and measure the end dots (xf..yr).
   5. Every few runs, send `status` and write the battery into battery_pct.
4. Quit the console and stop the recorder. Copy both folders into
   `data/logs/` and `data/video/` (video stays out of git).

Run order is shuffled (seed 21) so battery drain and floor wear don't line up
with one route.

## Training video for the detector (after the runs)
10 minutes, no measuring: robots without tape strips if possible, all flashed
robots, driving the three routes from different start spots and headings, plus
robots standing still at various places. `python3 camrec.py --label training`.

## Trials
8 runs per route, 24 in total. Eight gives a 95% interval of about +/-0.8 cm
on each route's mean error if the spread is near 1 cm; the first 5 runs per
route are used to check that and add runs if needed.

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

## Conclusion

## Deviations
