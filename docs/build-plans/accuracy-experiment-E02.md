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
- If lost (camera sees > ~15 cm from expected): the script drives it back by camera, flags the run
  as an outlier, carries on; calls Nick only if stuck or off the board.
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
