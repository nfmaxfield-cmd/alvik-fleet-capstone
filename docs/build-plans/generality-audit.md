# Generality audit: what ties MIRLT to Alviks, plywood and this basement (2026-10-25)

**Premise.** Nick, 2026-10-24: "get this system ready to expand beyond Alviks and plywood. I want to be published."
This lists every place the code and hardware assume Alvik robots, a plywood board, a rectangle with two blocked
boards, one overhead camera, three robots, the Pi or the home WiFi. For each one it gives where the assumption
is, what breaks somewhere else, the smallest change that removes it, and a rough effort. It is an audit only: no
code was changed, and nothing below starts before Nick approves it (he asks before rewrites).

**Data.** Read for this audit: `tools/experiment/e02*.py`, `tools/sim`, `tools/pi`, `run-e02-pi.sh`,
`firmware/home_lab/*`, and the build plans and reports up to 2026-10-24. The earlier outline
(`generic-platform-outline.md`) proposed the layers. This audit gives the line-level list behind them.

**Conclusion, in short.**
- The ideas are general: joint self-calibration, the recovery ladder, learned sensor trust and the virtual border.
- The constants and the plumbing are not. Robot size, the arena shape and the two boards, the three park spots,
  the Alvik's library calls, IP addresses and firmware version strings all sit in about 40 places.
- Most of it is a refactor with no change in behaviour. Two pieces are real design work: the arena as a polygon,
  and a driver interface for robots that only take velocity commands (ROS 2).

Effort is Claude build hours including tests, plus Nick's time on hardware where it says so. All estimates are
guesses (see Possible inaccuracies).

---

## 1. Robot interface

| # | Where (file: function) | What it assumes | What breaks elsewhere | Smallest change | Effort |
|---|---|---|---|---|---|
| R1 | `e02.py`: `Runner.leg`, `Runner.drive_to`, `Runner.k_drive`; `e02_steer.py`: `plan_leg`, `Steer`; firmware `motion.h`, `e02.h` (`go d id`, `turn deg id`, `run e02`) | Differential drive that **turns on the spot**, driven in discrete legs it reports back (LEG records) | Car-like robots (no spin; `plan_leg` always turns first). Robots that only take velocity commands (ROS `cmd_vel`). Holonomic robots (work, but waste their sideways motion) | A laptop `RobotDriver` interface (`go`, `turn`, `stop`, `fix`, `status`, `capabilities`) with today's UDP protocol as the Alvik driver. A `turns_in_place` flag in the robot profile; when false, `plan_leg` plans arcs | 6-10 h interface (no behaviour change); +8-12 h ROS 2 driver |
| R2 | Firmware: **167 direct `alvik.` calls in 14 files** (`home_lab.ino` 39, `motion.h` 30, `experiments.h` 30, `e02.h` 19, `dock.h` 15, `rawlog.h` 11, `edge.h` 8, `follow.h` 6, `pose.h` 4, ...) | The Arduino_Alvik library everywhere | Any other Arduino-class robot means editing every file | A `hal.h` with about 12 functions (wheel speeds, pose or encoders, yaw and accelerations, range zones, line or cliff readings, battery, LEDs, brake), used by the E02 path first (`motion.h`, `pose.h`, `edge.h`, `e02.h`, `corr.h`) | 6-8 h for the E02 path; the rest when needed |
| R3 | `e02_fit.py`: `PRIOR`, `PRIOR_SD`, `BOUNDS`, `fit_robot`; `e02_explore.py`: `GYRO_RANGE` (0.85-1.25) | Odometry error is two scales and a drift (heading = g x raw - d x t; distance = w x counted). Priors from Alviks (gyro 1.075, wheels 0.979), bounds 0.97-1.18 and 0.92-1.05 | A robot whose true gyro scale is outside the bounds has every correct fit **refused**. Left/right wheel mismatch (a robot that curves) is not modelled: it lands in the heading terms | Priors and bounds move into the robot profile. Add one asymmetry term (curvature per cm driven) to `fit_turns` | 2 h profile; 4-6 h asymmetry term |
| R4 | `e02_fit.raw_series`, `e02.py`: `Listener`, `own_pose`; `docs/data-formats.md` E02 | The robot streams its own dead-reckoned pose and raw yaw (RAW at 50 Hz), plus LEG, FIND, BUMP and E02 records | Robots that report encoder ticks, IMU rates or a ROS `odom` topic | A record adapter in the driver (R1) that turns any robot's output into (t, counted distance, raw heading, leg events) | 3-4 h |
| R5 | Firmware `motion.h`, `corr.h`; `e02.h` bump detection (`bump_jolt_g` on `ax`) | IMU axes and signs as on the Alvik's LSM6DSOX (yaw grows left, `ax` along the drive) | A board mounted another way: the heading sign flips, the jolt cue is lost | IMU axis map and signs in the robot profile (the simulator already has `yaw_sign`). Explore already fits g, so a sign error would show there | 1-2 h |
| R6 | `e02_map.py`: `TOF_AHEAD_CM` 4.5, `SAFE_FRONT_CM` 7, `BOTTOM_AHEAD_CM` 9; `e02.py`: `front_check`, `ahead_cm`, `FRONT_MIN_COS`; `e02_trust.py` front | One forward multi-zone ToF (VL53L7CX, 8x8 zones) at a known offset | Lidar (360 deg, many more readings), sonar (wide cone), or no range sensor: checks run against the wrong geometry | A sensor profile (mount offset, field of view, range, noise). One generic "range ray against the map" measurement, used by the map, the trust code and the board correction. Checks are skipped when the sensor is absent | 4-6 h |
| R7 | Firmware `edge.h` (line level learned per run, `edge_thr`; bottom zone `edge_bot_cm`); `e02.py`: `SAFETY_SETTINGS` (edge_thr 150, edge_bot_cm 11) | A raised plywood sheet on cement: the line sensors read 52-84 on plywood and 241-573 past the edge; the floor drops one sheet's thickness | A flush floor (a warehouse, a table with tape): the guard never trips, or it trips at every dark tile or tape line. Another robot's cliff sensors read on a different scale | Cliff guard as a capability: line reflectance, ToF bottom, or none. A one-minute "surface check" learns the levels per floor (`tools/edge_check.py` already does it offline). With no physical drop, the virtual border (firmware 2026-10-25a and the camera's) is the edge | 3-4 h; one session per new surface |
| R8 | Firmware `e02.h` `run find` (stall, jolt, touch, then push-to-square); `e02.py`: `do_board_find` | A light robot with a flat, square face that can push against rigid boards | Heavy AGVs (pushing a wall is unsafe), round robots (they cannot square up), soft walls | Make "locate myself" a choice in the profile: board push (Alvik), edge find (`e02_edgefind`, already built), dock, or camera only | 2-3 h (the pieces exist) |
| R9 | Firmware `blink.h` `codeFor` (Left 1, Center 2, Right 3, others 4); `tools/check_session.py` `BLINK_NAMES`; `tools/sim/robot.py`; `e02.py`: `naming_check`, `led` commands while driving | Robots with controllable LEDs, named Left / Center / Right; no more than 4 codes | Robots without LEDs; more than 4 robots (they all become code 4) | Make identity by commanded motion the default (`Runner.identify`: a 40 deg turn, already built). Lights become an optional extra witness. Names and codes come from the robot list | 2 h |
| R10 | Firmware `net.h` (UDP, `name\|seq\|line`, ports 5005/5006, broadcast until the first command), `secrets.h` (2.4 GHz WPA2); `run-e02-pi.sh` `RUN_ARGS_DEFAULT`, `tools/pi/session_plan.sh` `ROBOTS` (192.168.1.250 / .252 / .253); `tools/pi/robot_sleep.py` | One home LAN with broadcast and fixed DHCP addresses | Campus or enterprise WiFi (802.1X, client isolation, blocked broadcast); any other address plan | A `site.json` robot list (name, kind, address, transport), read by both scripts and the runner. The existing broadcast hello does discovery. The transport sits behind R1 (UDP today; serial, MQTT or ROS 2 later) | 2 h list; 6-8 h ROS 2 bridge (with R1) |
| R11 | `e02.py`: `EXPECTED_FW`, `OK_FW`, `fw_border` (checks `firmware == "2026-10-25a"`) | Every robot runs this sketch | Every other robot is refused, or silently loses the border | A capability handshake: the robot lists what it does (border, sleep, find, e02 routes) and the runner checks capabilities, not a version string | 2-3 h |
| R12 | `ROBOT_CM` 9.5 / `ROBOT_HALF_CM` 4.75 / `FACE_CM` / `ROBOT_TOP_CM` 3.7, **repeated** in `e02_camera.py`, `e02_plan.py`, `e02_explore.py` (`ROBOT_CM`, `ROBOT_LENGTH_CM`, `size_scale`), `e02_analysis.py` (`TOP_CM`), `e02_lock.py` (`RADIUS`, the square "outline" look), `MotionTracker` (`OTHER_CM`, `MERGE_CM`, shape check), `tools/sim/config.py` | Every robot is a 9.5 cm square whose top is 3.7 cm up, and all robots are alike | Bigger, round or rectangular robots, and mixed fleets, get the wrong shape check, merge distance, park clearance and explore size check | One robot profile per kind (length, width, height, shape symmetry, axle offset), imported everywhere. Each robot's kind comes from the robot list (R10) | 3-4 h (refactor with tests) |

## 2. Arena (shape, edges, boards, surface)

| # | Where | What it assumes | What breaks elsewhere | Smallest change | Effort |
|---|---|---|---|---|---|
| A1 | `e02_plan.py`: `Board.__init__` (rect 0, 0, 122.15, 121.9), `on_board`, `edge_room`, `route_fits`; `e02_camera.py`: `BOARD_W_CM`, `BOARD_L_CM`, `make_calibration`, `find_board_corners` (one 4-sided bright region); `e02_explore.py`: `rectangle`, `rectangle_from_rays`, `board_frame`; `e02_map.py`: `fit_sides`; `e02.py`: `edge_sides` | The arena is **one axis-aligned rectangle** | L-shaped areas, aisles, round tables, several rooms | An arena polygon: a list of segments, each `wall`, `drop` or `virtual`. `Board` keeps its methods but computes them from the polygon. Explore already finds lines by RANSAC (`sides(n_max=4)`): raise n and close them into a polygon | 8-12 h laptop; 4-6 h explore |
| A2 | `e02_plan.Board` (`xboard`, `yboard`, `open_edges`); firmware `params.h` / `e02.h` (`e02_xboard_side`, `e02_yboard_side`, `e02_xboard_cm`, `e02_yboard_cm`, `e02_open_x_cm`, `e02_open_y_cm`); `run find` (x board, then y board); `e02_edgefind.solve` (two boundaries at a right angle) | **Exactly two blocked boards meeting at a right angle; two open edges, one per axis** | Arenas with 0, 1, 3 or 4 walls, or walls not at right angles: the find, the border and the board line updates do not apply | The firmware takes a list of up to N border lines (a, b, c), not x and y. The find touches "any two non-parallel walls" from the polygon. `solve` uses the real angle between them | 4-6 h firmware; 4 h laptop |
| A3 | `e02_explore` frame convention (the open edges' corner at (0, 0), boards at x = W, y = H); `e02_camerr.mark_xy` (board middle hard-coded as 61.0) | The board's middle is at 61 cm and the frame is tied to the boards | Any other size: the marks land on the wrong side | Falls out of A1 (frame from the polygon) | in A1 |
| A4 | `e02_plan.py`: `PARK_UV` (**3 park spots**), `START_CLEAR_CM`, `FIND_STAGE_CM`; `e02.py`: `SEARCH_LOOKOUTS` (40-85 cm), `BLOCKS_FIND_CM`; `e02_camerr.MARKS`; `e02_map.CORNER_SKIP_DEG` (the one open corner) | Three robots and a 4 ft board. One "find corner" where finds end | A fourth robot has no park spot. On a bigger arena the lookouts and marks cover only one corner; on a smaller one they fall off it | Generate park spots, lookouts and marks from the polygon and the number of robots (spaced points with clearance) | 3-5 h |
| A5 | Line levels (`edge_thr`), bottom zone (`edge_bot_cm`); bump by touch because the tyres slip on plywood at about 96%; `MotionTracker` thresholds (`thr` 16, `MOTION_THR` 9); `find_board_corners` (plywood brighter than its surroundings); `tools/sim` floor 52 | Plywood: matte, light, little texture, brighter than the cement around it | Glossy, dark or patterned floors (less tracker contrast, more glare), carpet (wheel slip changes the odometry scale; stall-based bumps fail), floors that look like their surroundings (the corner finder fails, but explore does not need it) | The surface check (R7) also learns the tracker's thresholds from the empty floor's noise. Record the surface in the session data. Corner finding becomes optional (explore replaces it) | 3-4 h; one session per surface |
| A6 | `CameraModel` floor plane; explore ("the floor is flat") | A flat, level floor | Ramps, warped sheets, uneven concrete | Keep it, and state it as a limit. A per-cell height error would show in the camera error map | 0 h |
| A7 | `e02.py`: `ahead_cm` (only boards and robots ahead) | Nothing else is on the floor (the brick demo adds bricks for itself) | Static obstacles on a real site are not in the front-sensor check | An obstacle layer in the site map, fed by the brick finder and the front sensor | 4 h |

## 3. Sensing (cameras)

| # | Where | What it assumes | What breaks elsewhere | Smallest change | Effort |
|---|---|---|---|---|---|
| S1 | `e02.py`: one `CAM.Feed`, one `make_tracker`; `e02_camera.CAMERA_NAMES`; explore fits one pinhole; trust has one `camera`; `run-e02-pi.sh --with-side` records the side camera, but "the runner uses the overhead only"; `e02_sidecam.py` is offline only | **One overhead camera sees the whole arena** | Arenas bigger than one view; blind spots (the top board's glare: lock-on held 8-36% of the time there) | A `CameraSource` interface (frames, model, id). One tracker per camera gives (t, x, y, sd, cam_id). Trust and KC take any number of cameras | 4 h interface; 10-16 h merging several cameras |
| S2 | `MotionTracker.__init__` (robot size in pixels taken once at the board's middle unless `SIZE_PER_SPOT`), `_shape_ok` (size and squareness); `e02_lock` (square outline); `e02_analysis.quarter_median` (heading known up to a quarter turn) | A camera looking straight down at a square robot | Slanted security cameras (scale changes 2-3x near to far: the side camera, 2026-10-24). Rectangular robots (heading ambiguous by a half turn, not a quarter) | `SIZE_PER_SPOT` on by default when the camera's tilt is large. The shape check comes from the profile's footprint projected through the camera model. The heading ambiguity order comes from the profile (4 square, 2 rectangle, 1 marked) | 4-6 h |
| S3 | `e02_explore.py`: principal point at the picture's centre, one lens term `k1`, `F_PRIOR_HFOV_DEG` 60, `MIN_PX_PER_CM`; `e02.py`: `EX_provisional_camera` (260 cm straight down) | A narrow webcam roughly overhead | Wide-angle and fisheye security cameras (often more than 100 deg), cameras far away | The camera model's type and priors come from a camera profile (pinhole + k1, + k2, fisheye) | 3-4 h |
| S4 | `tools/pi/camrec.py` (MJPEG `/live` on port 8000, notes on UDP 5007); `e02_camera.Feed` (`StreamSource`, fixed `capture_delay_s` 0.04) | The Pi recorder serves the pictures | IP cameras (RTSP/ONVIF), a USB camera on the same machine | Source plugins (RTSP via OpenCV, local device). The capture delay is measured (`tools/live/capture_delay.py` exists), not fixed | 2-4 h |
| S5 | `e02_camerr` evidence: the finds, all within 3 cm of one corner (night analysis B1) | Finds are spread over the board | Here the camera's error map is a constant shift (it cannot change lengths). On any site it says nothing away from the find corner | Finds and touches at several walls (A2 makes that natural), plus the tape marks once | in A2; 1 h of Nick's time for the marks |

## 4. Compute and runtime

| # | Where | What it assumes | What breaks elsewhere | Smallest change | Effort |
|---|---|---|---|---|---|
| C1 | `run-e02-pi.sh` (apt packages, tmux, exFAT/ext4 USB drive, `st1maxfini.local`, port 8090); `e02.py --pi` default `st1maxfini.local`; `tools/pi/upload_session.py` (`REPO_SLUG` `nfmaxfield-cmd/alvik-fleet-capstone`, token in `~/.config/mirlt/github_token`); `run-e02.bat` (Windows) | Nick's Pi, Nick's repo, Nick's laptop | Another lab cannot run it without editing scripts | Put hostnames, ports, the upload target and the video drive in `site.json` (R10). Upload becomes optional | 2-3 h |
| C2 | `MotionTracker.push`: 41-66 ms a picture with 3 robots; the Pi handles 14-16 of 25 pictures a second at 640x480 | A few robots in a small picture | More robots or bigger pictures mean fewer pictures a second, so more tracker jumps (night analysis B: jumps were inside 51-66% of stretches) | Track only a window round each robot (partly done), and log ms per robot so the limit is measured. Picture size goes in the camera profile | 4-6 h, plus a timing run |
| C3 | One runner process controls every robot, and robots move one at a time | A central controller and a small fleet | Fine to about 10 robots; beyond that the runner's loop is the bottleneck | Keep for the paper; state it | 0 h |
| C4 | Repo has **no LICENSE file** (README: public; Andrew's code is not included) | Private use | Others cannot legally reuse it, and artifact reviewers notice | Choose a licence (Nick's decision) | 0.5 h |

## 5. Experiment design

| # | Where | What it assumes | What breaks elsewhere | Smallest change | Effort |
|---|---|---|---|---|---|
| E1 | Firmware `e02.h` routes 1-3 (`e02_route`, straight 100, L 30-30, square 50); `e02_plan.route_local`, `route_fits`; `e02_analysis.ROUTES` | Routes sized for a 122 cm board, run by the robot's own firmware | Larger arenas (routes too short to show drift); robots without this firmware | Routes become waypoint lists in the experiment file, scaled to the arena, sent as legs (R1). The analysis reads the same file | 4-6 h |
| E2 | `e02_plan.regular_block` (robot = (route + condition + block) mod n), `blocks_for(n_robots=3)` | n robots = 3 = number of routes ("every 3 blocks give each robot each condition x route once" holds only then) | 2 or 5 robots: unbalanced condition-by-robot cells | A balanced design for any number of robots and routes (a Latin rectangle per block), with a test that checks balance for n = 1-6 | 2-3 h |
| E3 | One robot moves at a time (`e02_camera` docstring: "the thing that moves is that robot") | Identity comes from who was told to move | No test of robots moving together, which is the swarm case; slow for big fleets | `e02_multitrack.py` exists. Pair it with identity by commanded motion (R9), first as a separate condition | 12-20 h |
| E4 | `e02_analysis` measures end error with **the same camera the C condition learned from** | The camera is right | Any claim across cameras or sites is confounded with that camera's own error (session 1 report: "camera calibration error ... is not separated") | Independent truth: the tape marks (`e02_camerr.MARKS`, 7 points), board-touch finds (1-2 cm per the session analysis), and the pixel return check. Report camera-free measures beside the camera ones | 2 h code; about 1 h of Nick's time |
| E5 | Dozens of chosen constants (`BORDER_CM` 15, `JUMP_CM` 3, `MAX_RMS_CM` 1.5, `DEGRADED_K` 2, `GATE_MAX_CM` 3, `KC_POOR_CELL_SD_FACTOR` 1.5, ...) | This rig's noise and speeds | On another rig they are wrong in unknown directions | Express the top ones as multiples of noise measured in a short baseline stage at each site (the trust code already does this for the wheel gate) | 3-4 h for the top 5 |
| E6 | `e02_trust.HEALTHY` (camera 0.8, wheels 0.6, gyro 1.0, front 5.0) from the 2026-10-23/24 sessions | New robots and cameras have Alvik-like noise | A different robot looks "degraded" from its first run, or a bad one looks healthy | A bootstrap mode: healthy values learned per robot kind in its first session, flagged as provisional | 3 h |
| E7 | `tools/experiment/e02_sim.py`: `World` (`PL.Board()` 122 x 122, two boards), robot names and blink codes, "2.34 px/cm" camera | The simulator is this basement | Dry runs cannot test a new site or robot | The simulator reads the same arena polygon and robot profile (after A1 and R12). `tools/sim/config.py` is already fully parametric | 4-6 h |

**Already general (keep as is):** the joint camera-and-robot fit (any pinhole, any number of robots), the per-run
refit from stretches, the trust split into a shared (camera) term and own (robot) terms, the EKF in
`tools/fusion/ekf.py`, the recovery ladder, the camera drift watch (`CameraShiftWatch`), the lock-on tracker's
learned look (nothing robot-specific is given), the degrade test harness, and the parametric simulator models.

---

## Ranked: the five changes that widen it most for the least work

| Rank | Change (items) | Effort | What it opens up | Why this rank |
|---|---|---|---|---|
| 1 | **Robot profile + `site.json`** (R12, R10 list, C1, R11 handshake, R3 priors, R5 axes) | 8-10 h | A new robot kind or a new lab is one file. Prerequisite for 2-5 | Pure refactor, no behaviour change, every later change builds on it |
| 2 | **Arena polygon with segment kinds** (A1, A2, A3, A4; firmware border as a list of lines) | 16-22 h | Any shape; 0-4+ walls; walls or drops on any side; park spots for n robots | The biggest single limit (rectangle + two boards) is in the planner, the camera, explore, the firmware and the simulator |
| 3 | **Camera profile and sources** (S2, S3, S4) | 9-14 h | Slanted and wide-angle security cameras, IP cameras | Nick's stated aim (typical security cameras), and the self-calibration claim depends on it |
| 4 | **Independent truth + balanced schedule for any n** (E4, E2) | 4-5 h + 1 h of tape | Results that hold across cameras, and any fleet size | Cheap, and without it no generality result is convincing |
| 5 | **`RobotDriver` interface + ROS 2 driver** (R1, R4) | 14-22 h | TurtleBot3, iRobot Create 3 and other ROS 2 robots | Most expensive of the five, but it opens the largest group of research robots. Do it once a second robot is actually available |

Next after these: the firmware HAL (R2, 6-8 h, for another Arduino-class robot) and a surface check (R7, A5).

## The cheapest convincing demonstration of generality

| Option | Cost | Convincing because | Weakness |
|---|---|---|---|
| **Camera moved and tilted, explore from scratch** | 1 session, no code | Directly tests the self-calibration claim (Nick's goal: recalibrate after the camera is moved drastically) | Same robots and floor |
| **Second floor surface** (hardboard, vinyl or bare concrete, a 4 ft sheet) | 1 session after the surface check (R7: 3-4 h) | Shows the odometry, cliff and tracker learning transfer | Same robots and camera |
| Non-rectangular arena (L shape) in the simulator | After rank 2 (16-22 h) | Shows the polygon logic | The simulator is optimistic about accuracy (2026-10-24 notes) |
| A real L-shaped arena | Rank 2 + boards placed to block a corner | Real hardware | Most build work before any data |
| A different robot | Rank 1 + 5 (22-32 h) + a robot | The strongest single result | Needs a robot Nick may not have |

**Recommendation.** A 2 x 2 on real hardware in two sessions: camera pose (as now / moved and tilted) x floor
(plywood / second surface). Each cell starts from nothing (explore, no clicks) and is scored against the 7 tape
marks. It needs only the surface check (R7) and the independent truth (E4): about 6 h of build and 1 h of tape.
Add the L-shaped arena in the simulator once rank 2 is built, as supporting material. A different robot is the
strongest single addition. Whether to do it depends on whether one is available (question for Nick).

---

## Appendix A. How the effort numbers were made
Each estimate counts the files touched, a unit test per change, and one simulator dry run, from the sizes of
earlier builds of similar scope in `docs/build-reports/` (for example the virtual border and the trust build,
each a few hundred lines). None were timed against a tracker. The time tracker in Nick's other repo (`TIME.md`)
was not read for this audit.

## Appendix B. Counts behind the table
- `alvik.` calls per firmware file: `home_lab.ino` 39, `motion.h` 30, `experiments.h` 30, `e02.h` 19, `dock.h` 15,
  `rawlog.h` 11, `edge.h` 8, `follow.h` 6, `pose.h` 4, `blink.h` 2, `look.h` 2, `corr.h` 1 (167 in all).
- Robot-size constants: `ROBOT_CM` / `ROBOT_HALF_CM` / `ROBOT_LENGTH_CM` / `ROBOT_TOP_CM` / `TOP_CM` / `FACE_CM` are
  defined separately in 5 modules, plus the simulator's `RobotModel`.
- Hard-coded addresses: 3 robot IPs in 2 scripts; the Pi hostname in 3 places; the GitHub repo slug in 1.

---

## New tests that may now be needed
1. A balance test for the schedule with 1-6 robots (E2): every robot gets every condition equally often.
2. A dry run with a second, made-up robot profile (13 cm round, 0.8x speed, no turn on the spot) once rank 1
   exists, to prove nothing still reads the Alvik's constants.
3. A simulator arena with no walls at all (only drops, like a table) and one with four walls, after rank 2.
4. A surface check session on a second floor: line and bottom-zone levels, the tracker threshold, and a find.
5. Camera moved and tilted: explore from scratch, scored on the tape marks.
6. Trust bootstrap (E6): a healthy robot with no healthy values must not be flagged in its first session.
7. Tracker timing with 3, 6 and 10 simulated robots on the Pi (C2), counting jumps over 3 cm per stretch.

## Possible inaccuracies
- **Effort estimates are guesses** (Appendix A), not measured; the ROS 2 driver especially (it depends on the
  robot).
- **Coverage.** The tables come from reading module headers, constants and the functions named. Not every one of
  `e02.py`'s 6,958 lines was read. An Alvik-specific number inside a function body may be missing.
- **Older tools left out.** `tools/fleet`, `tools/site`, `tools/live`, `tools/recognition` (MIRLT studio) and the
  dock hardware were skimmed for the E02 path only. They carry their own assumptions (for example the dock's
  printed geometry).
- **"Breaks elsewhere" is predicted, not tested.** None of it was run (ports busy; no dry runs by request).
- **The ROS 2 audience claim** (rank 5) is a judgement about which robots research labs use, not a count.
- **The 2 x 2 demonstration's value** assumes the tape marks are good to about 0.5 cm (the `MARK_W` reasoning in
  `e02_camerr.py`), which is itself a chosen number. Whether a design of experiments should set the mark layout:
  yes, a leave-one-mark-out check of the error map (bandwidth 15 / 25 / 40 cm, already noted in `e02_camerr.py`)
  is the sensible way to choose how many marks and where.
