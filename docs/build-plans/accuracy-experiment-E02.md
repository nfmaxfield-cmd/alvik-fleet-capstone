# Build plan: accuracy experiment E02 (2026-10-21)

Agreed with Nick (chat, 2026-10-21). Goal: statistics for his professor on how far the Alviks end from
where they meant to be (mean, standard deviation, confidence ranges, whether differences between
conditions are real), with the camera as the measuring tool. Tape measures only where absolutely
critical (none planned). Run as soon as it is ready; about 2 hours; Nick is not physically involved
during the runs.

## Design (decided)
- Conditions: U uncalibrated (corrections at factory: `corr reset`); C calibrated on-board sensors
  with corrections **learned during this session** from the camera-measured U runs (gyro scale,
  drift, wheel scale per robot, updated as data comes in: the self-calibration demo); K camera only
  (laptop steers by camera fixes, robot's own pose ignored); KC camera + calibrated sensors (robot
  runs calibrated and receives camera position fixes). Shuffled in balanced blocks; the first block
  is U-heavy so C has corrections to learn from (state this in the report).
- One robot moves at a time (others parked): the camera always knows who is moving: no blink naming
  needed. A 2-minute naming check at the start is recorded as data only.
- All three robots (Left, Center, Right); each gets its own corrections.
- Every run: (1) find itself: drive slowly into the x-axis board until a bump (motion-sensor jolt plus
  wheel stall; NO switches), push gently to sit square, back off, turn, same on the y-axis board ->
  true position and heading; the camera records the same moment; (2) drive one of three fixed
  routes (E01: straight 100 cm; L 30 cm, turn, 30 cm; square) with a 1 s pause at the far point;
  (3) return to the start; the camera measures far point and end against intended, and the robot
  logs its own belief. Return-to-start also compared pixel-to-pixel (start frame vs end frame), which
  needs no calibration.
- Bump data per bump: believed position at contact vs camera, jolt size, detection delay, overshoot.
- Boards: one x edge and one y edge fully blocked (secured with cement blocks), meeting at one corner;
  the other two edges open (edge guard data, free). Camera checks the boards never move.
- After every run the script parks the robot by camera. (Changed 2026-10-21b: "lost at 15 cm" flagged most U
  runs, which drift ~18 cm by design.) A run that NEEDED RECOVERY (camera lost it, off the board, a safety stop
  cut the route short, or it ended > 40 cm from its start) is noted in runs.csv `recovery`, kept in every
  statistic and never called an outlier; outliers are decided statistically within condition x route by the
  analysis, reported with and without. A camera watchdog stops any robot heading for an open edge or a parked
  robot (firmware edge guard and front stop first). Nick is called only if stuck or off the board; no answer in
  10 minutes: that robot is put aside, the others carry on.
- Parked robots (2026-10-21b) sit in the two corners off the routes' diagonal, facing the find's staging point;
  the route start spot is 28 cm from both boards so an uncalibrated return does not run into a board.
- No docks this session (the funnel would erase the error being measured).
- Firmware: load the new firmware (bump detection, find-yourself, modes, the 2026-10-20b gyro/corr
  work); U = corrections reset inside it.

## Lanes
F firmware (firmware/home_lab): bump detection, push-to-square, find-yourself (both boards), route
runner with far-point pause and return, accept camera position fixes and camera-steered waypoints,
per-run records (BUMP rows, route rows with believed poses at start/far/end), all in docs/data-formats.md;
simulator models (tools/sim) and tests; nothing that can drive a robot off the board without the edge
guard.
L laptop runner (tools/experiment/e02.py or similar, reusing alvik_console Link, PiNotes, live tracker,
fleet geometry): setup checks (boards seen and still, robots seen, calibration present), the naming
check, the block schedule, one robot at a time, live camera tracking of the moving robot, the K/KC
steering and fixes, learning corrections from U runs, lost-robot recovery, phase/run marks in both
camera videos, a plain-words console; resumable after a crash.
A analysis (tools/experiment/e02_analysis.py): per-run metrics from the overhead video + robot logs,
bump metrics, statistics per condition and per robot (mean, SD, 95% confidence interval, Welch's t-test
and a rank test, effect size), a chart of each condition's spread with one plain sentence under it,
a lean report (premise, data, conclusion, what's next, two sweeps).
Then: an end-to-end dry run of the whole session in the simulator, a code review, fixes, and the
step-by-step session guide for Nick (every window and tab named, one command per box).

## Contract between F (firmware) and L (laptop runner) — both lanes build to this; F documents the
final details in docs/data-formats.md "E02" and may extend it, but not change these names.
Commands (console text, as existing commands):
- `corr reset` / `corr` / `set gyro_scale v`, `set gyro_drift_dps v`, `set wheel_scale v` (existing,
  2026-10-20b corr.h): U = reset; C/KC = the learned values set before the run.
- `run find` : find itself against the x-axis board then the y-axis board (board sides and their
  floor positions from params: `set e02_xboard_side`, `set e02_yboard_side` and the camera-measured
  board lines `set e02_xboard_cm`, `set e02_yboard_cm`); ends with its pose reset from the bumps and a
  FIND record.
- `run e02` with `set e02_route 1|2|3` (straight 100 / L 30-30 / square), `set e02_mode 0|1`
  (0 = own sensors, 1 = camera-steered: drives only `go` legs the laptop sends), far-point pause
  `e02_pause_ms` 1000; returns to its start; E02 record with believed poses at start, far, end.
- `fix x y th t_wall` : a camera position fix (KC: blended into the pose; K: replaces it).
- `go d_cm` / `turn deg` : single legs for camera-steered mode.
- Records: BUMP (per bump: board, believed pose at contact, jolt, delay_ms, overshoot_cm),
  FIND (result, pose before/after), E02 (route, mode, poses start/far/end believed, timing), plus
  the existing raw rows, sync flash at run start (existing sync.csv).
Safety: the existing edge guard stays on in every mode; a stop command always works.
