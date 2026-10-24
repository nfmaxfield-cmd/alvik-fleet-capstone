# E02 build after the 2026-10-22 morning session

Session: 09:51-10:43, release `e02-session-oct22` (video split in two: `cat video.part-aa video.part-ab > video.avi`).
1 run finished (Right U straight, far point "100 cm off": the camera did not follow it, so not a real result).
Nick away from 11:05; he asked: work until the usage runs out, as much in parallel as possible, then make the
logic more generic for other equipment and sensors.

## What went wrong (from the logs, the track and the footage)
1. The tracker lost moving robots for seconds and jumped: at 10:40:03, during Left's 167-deg turn, it took Right
   (7.6 cm from Right's last spot, just outside the 7.6 cm "another robot" zone) for Left; then "the camera saw it
   move 0 cm, its wheels say 10 cm" (it was watching Right). The runner gets all 25 pictures a second (track.csv).
2. Its position flipped 1.5-6 cm between two pieces of one robot, frame after frame.
3. A robot standing still with the tracker's position a few cm off was learned into the floor bit by bit; its
   outline then broke up (footage 10:40:12: the floor memory holds a ghost of Left).
4. "Something in front: turning away" looped up to 13 times: the turn legs between reset the count.
5. Every park spot is 17-20 cm from an open edge, under the 20 cm "move it in first" limit; by the x-board the
   move-in point was 1.4 cm from the board's line.
6. Center fell off the open edge (10:23): found 3.5 cm off the board, then driven blind; its edge guard had learned
   with all three line sensors over the edge ("all three on tape"), which switched the line cue off.
7. Pressing Enter with no hands nearly always worked: the help call was only a retry.
8. Right never named in the naming check (it was not parked: its parking after help did not finish).
9. One find check landed 45 cm off on the x-board (two robots' touches 93 cm apart): not explained yet.

## Done in this build
- [x] 4: obstacle legs counted on their own; back off 8 cm, then a quarter turn towards the board's middle
- [x] 5: move-in only within 14 cm of an open edge, to a point 25+ cm from both board lines
- [x] 7: three tries on its own (fresh floor, turn, heading, park) before Nick is called
- [x] 6: runner: a robot found at or over an open edge is never driven (Nick); firmware 2026-10-22b: all three
      line sensors off the plywood at the start = no forward drive that run (simulator mirrored)
- [x] 1-3: tracker: search circle grows for at most 0.6 s (was 1 s) at 30 cm/s (was 40); another robot's zone
      1.1 robot widths (was 0.8); pieces within 0.8 robot widths merged; a held robot's floor never learned within
      1.35 robot widths (1.25 while it moves); a held robot whose spot looks unchanged counts as seen; each leg tells the tracker where the robot can be (turn: 7 cm; drive d: d + 8 cm)
- [x] Live tracker had no board area at all (it followed hands at x = -47 cm): the runner now gives it the board and
      its walls; pixels beyond a wall are cut before outlines are made, so a robot against a board is not joined to
      the board's edge (replay of Left 10:39:58-10:40:55: seen 75% of frames, was 20% this morning): its outline joins the board's
      edge and fails the shape check. Next: a shape check that allows for a board beside it.
- [x] 9 explained: the firmware's own finds agree (bump points 121.5 / 124.1 cm on the y-board); the "touched at
      74.8 cm" came from camera positions at the bump moments, while the camera followed the wrong robot (1)
- [x] 8 explained: Right's parking after its help call did not finish, so it was not at its spot to be named
- [x] Front sensor as a free check (Nick): firmware 2026-10-22b adds "front=<cm>" to every finished leg; the runner
      compares it with the map (boards, other robots) and writes front_checks.csv. Recorded only, for now: one
      session's numbers set the limits before it steers anything
- [x] Finder sweep (Nick): when the camera can't see a lost robot turn after the retries, a parked robot turns in
      30-deg steps reading its front sensor; a reading 8+ cm short of the map is checked by the camera there
- [ ] Nick's ideas: robot-to-robot check (front_checks.csv already lists another robot ahead); finder robot sweep; the lost
      robot does a find to rescue itself; second camera (after the overhead tracker is solid); colour sensor (low)
- [x] Generic: outline for discussion, docs/build-plans/generic-platform-outline.md
- [x] The lost robot finds itself against the boards (rescue find): no firmware change needed (the firmware keeps
      its own pose between camera fixes; `run find` without a fix uses it). Only when last seen 15+ cm in from the
      open edges (park spots are 17+) and not moved by hand. Dry run with Left hidden from the camera for 140 s
      (E02_SIM_HIDE="Left:110:250"): 3 retries, Center's and Right's sweeps (one false hit at the y-board, checked
      by the camera and dropped), then Left found itself and was found again, no help call. With 60 s hidden: back
      on the 2nd retry. (The simulator never names robots in the naming check, before this build too.) Simulator: E02_SIM_HIDE="Left:110:170" hides a robot from the camera.
- [x] Today's footage as a regression test (tests/test_e02_realclips.py::test_tracker_on_oct22_footage)

## Before the next session
Nick uploads firmware 2026-10-22b to all three robots (same steps as this morning: Tools > Port, each robot).

## Tonight's build (after the evening session), Nick 20:06
1. **Cliff sense, measured (essential before mapping by feel):** from tonight's logs (Left's drive to the open
   corner at ~20:00, and every robot's RAW lineL/C/R and dbot_cm), the line sensors' and the front sensor's bottom
   zone's readings on plywood, at the edge and past it; set edge_thr / edge_bot_cm from them and turn both cues on
   (safe_edge 3). If one approach is too little data: a 10-minute edge test next time (tools/edge_check.py).
   Simulator test, then mapping back on.
2. **Mapping:** never aim a ray at the corner where the two open edges meet; the "camera saw it move 0 cm, wheels
   72 cm" retry loop (once a second, forever) must stop after a few tries and fall to the help ladder.
3. **Manual board borders:** answering **n** to "does the green outline sit on the board's edges?" lets Nick fix
   the outline himself (drag its corners in the live view, or click the 4 board corners in the picture) instead of
   stopping the runner.
4. **Setup questions optional:** the robots' mapping and board touches measure the board lines; the picture's
   questions become a cross-check (Nick, 19:33).
5. Check tonight's footage for the moving-robot fixes (newest picture, moving beats still, no ghosts).

## Backlog (Nick, 2026-10-23)
- **Unknown objects onto the map**: a front-sensor stop where the map is clear (no robot, no board) puts an obstacle
  on the map that paths go round; a parked robot in the way is asked to move aside. Not needed now; fold into a
  build later. Default slot: Thursday morning (last of the usage), sooner if a session shows a reason.
- **Engineer alerts in the live view** (decisions clicked, phone alert): later; assessment first
  (docs/build-plans/engineer-loop.md).
- **The robots calibrate the camera** (Nick, 2026-10-23): no corner clicks, no file; a robot drives known
  distances (wheels) in several places and directions while the camera watches its pixels move, and the pixels-to-cm
  mapping is solved from that. In the from-scratch rebuild. **Ask Nick questions before starting.**
- **Run a whole session on the Pi** (Nick, 2026-10-23, after the summary): stop-and-go driving on the robots' own
  sensors; tracking only near each robot's predicted spot; the gyro's heading picks the lock-on's rotation (1-2
  instead of 36); the full-picture detector every 2 s and at stops, possibly from a single sharp high-resolution
  frame (tests-to-run); idle checks when nothing moves. Questions for Nick first (laptop as viewer? recording on the
  Pi?). The Pi is a Pi 4 Model B. **Measured 2026-10-23 (tools/pi/tracker_timing.py, 30 s of this morning's
  footage, Left):** decoding 5.7 ms, motion detector 32 ms (about 26 pictures a second), detector + lock-on 35 ms
  (about 25 a second): close to the laptop's live 45 ms, not the 3-5x slower Claude estimated. Still to add: the
  Pi's own camera streaming and the runner in the same Pi. On that stretch the lock-on made it worse (seen 68% vs the
  detector's 95%): being fixed first.
- **Live lab graphs: say what they measure and where the data comes from** (Nick, 2026-10-23). Every chart in
  the live lab (and the live view) gets a minimal line: the quantity, its unit, and its source (e.g. "gyro scale
  per self-calibration fit, from fits.csv"). Next build.

## Next build (Nick, 2026-10-23 14:38-14:45): decisions
- **Explore = hybrid.** A short dance (each robot spins 360 deg, drives a few short legs near where it stands:
  gyro scales from "the picture looks the same again", wheel balance from straightness in the picture, a first
  camera fit), then all three robots drive rays outward in their own sectors; the rays map the border by feel and
  refine the camera fit. The robots' camera calibration **replaces** the corner-click one. The live lab shows the
  border points and the fitted edges as they come in.
- **Safety in the first minutes: the edge sensors only** (test mode).
- **Scale:** the robots' known size in the picture for now. Dock question below.
- **Remove a board without telling them:** in the second session today (needs the edge find: a robot locating
  itself against any two non-parallel boundaries, boards or edges).
- **Wheel slip (Nick: stress testers will make the floor slippery):** note, not for this build. The system must let
  wheels slip without losing too much confidence in its position or calibration. Ideas to weigh: detect slip (wheel
  speed vs gyro turn rate, vs the accelerometer, vs the camera, vs motor effort); down-weight the wheels while
  slipping (the filter's wheel noise grows with the slip signs); drop slipping legs from the self-calibration; map
  slippery patches as a layer of the site map and plan round them or slow down there.
- **A standardized dock as the scale reference (Nick's question):** yes, two ways. (1) The dock's printed shape is a
  known size in the picture (a reference object that is equipment, not a marker added for the camera). (2) Better:
  the dock's floor stripe sits a known printed distance from its back wall; a robot creeping in measures that
  distance with its wheels between the stripe and the switches clicking: an absolute wheel-scale check at every
  docking, which then fixes the camera's scale through the explore fit.

## Built 2026-10-23 afternoon: explore (item 1) and the edge find (item 2)
**Explore** (`e02_explore.py`, `Runner.explore`): with no calibration file, the robots calibrate the camera and map the
board themselves; `--classic-setup` keeps the corner clicks. Each robot dances a 15 cm square, then drives 7 rays out
to a board (front stop) or an open edge (edge guard) and back (a 175 deg turn and forward: backing up is blind).
Every stop pairs the camera's pixels with the robot's own wheels and gyro; one least-squares fit gives the camera
(focal length, lens bend, pose), each robot's wheel and gyro scale and where its own frame sits.
- **Tracking guard:** at each stop the camera's spot must fit what the wheels did (a drive of 8 cm or more must move it
  at least 0.4 px a cm; a turn on the spot must not jump it more than 25 px). If not, a wiggle turn finds the robot
  again; a stop that still does not fit is kept for the path but not for the camera. The fit also drops such stops,
  then refits without any stop more than max(10 px, 4 x the median) off. (Dry run 3: Left's tracker stayed on its old
  spot for 19 stops; refitted with the guard: 1.5 px.)
- **The reference robot** for the frames is the one the camera saw at most stops (not always Left).
- **Contacts** near another robot at that moment (where it stood then, not where it ended) are left out; a ray that
  turned more than 3 deg while stopping is left out.
- **Edge reach:** most edge stops in the simulator are the distance sensor's bottom zone, which sees the drop ~9 cm
  ahead of the axle (the line sensors: 3.5 cm). The stop's cue is recorded (`plywood edge (bottom|line)`) and the
  reach chosen by it (`e02_map.BOTTOM_AHEAD_CM` 9.0, from the simulator: **to be measured on the real board**).
  `refine_edges` estimates the reach from the rays' slants when they differ enough (reported).
- **Scale check by the robots' size** (Nick: "robot size for now"): the outline's long side at each stop against
  9.5 cm, reported at the end of explore; not applied (the real outline includes some shadow, not yet measured).
- **Live lab:** while exploring, the picture shows each robot's stops (its colour; grey cross where the camera's spot
  did not fit), the ray ends (orange: board; red: open edge) and the border so far (dashed), with a caption.
- **Simulator result (dry run 4, against the simulator's true camera, `explore_check.py`):** turned and shifted only,
  the robots' calibration is within 1.1 cm median (worst 1.6 cm) over the board; with a scale it is within 0.1 cm.
  So the camera's shape is exact; what is left is the size (+2%: the wheels' average sets it) and where the corner
  lies (~4 cm). Tracker in the runs after: 6.5 cm median against the truth (from the corner placement).
- **The real check:** `python tools/experiment/explore_check.py logs/e02-<session>` compares the robots'
  calibration with the corner-click one over the whole board.

**Edge find** (`e02_edgefind.py`, `Runner.edge_find`): locating against any two boundaries at right angles, boards or
open edges, with no camera. Touch boundary A twice 25 cm apart (heading against A: 1 cm of touch error is 2.3 deg),
then B once. `fix` with the result. Never two open edges together (the open corner). The camera watchdog lets that
robot reach an open edge meanwhile (the edge guard stops it).
- `--edge-find`: every find is an edge find.
- Otherwise a board find that meets an open edge where the board should be (`run find` fails: "an open plywood edge,
  not a board") is followed by an edge find, and the map is corrected: the board is gone, its side is an open edge
  for the watchdog and the planning (`Board.missing`, saved in the state).
- Simulator: `--dry-remove-board y@760` takes the y board away 760 s in, without telling the runner.

## Next build, first (Nick, 2026-10-23 18:53-19:06): sensors that don't meet expectations must not ruin a session
Session 2's explore failed on the real board (fit 136 px rms; wheel scales hit the bounds; all three robots ended
with a wheel over an open edge; then every compass check failed). To build before items 3-5:
- **Check before trust, stage by stage:** measure each robot's gyro scale (full spin: the picture repeats) and wheel
  scale (straight drives) in the dance, accepted only in a sensible range and when a second try repeats; the camera
  fit then uses those measured numbers. Each robot's data is scored on its own (px rms); a robot whose data does not
  fit is left out of the camera calibration and flagged, and the others carry it.
- **Never end the session over it:** after retries, fall back to the best calibration already on hand (the corner
  clicks, or an earlier session's), and **say so explicitly every time it is used: at setup, in each run's line, in
  the session summary and in the analysis report** (Nick: "make it explicitly clear to me if old calibration data is
  used to complete a run").
- **Explore ends with each robot driven back towards the middle**, never left at an edge.
- **A failed compass check stops and asks** (it means a wheel is off the board or the robot cannot spin).
- For the summary: the system doesn't trust any sensor it hasn't checked, the camera included, and one bad robot
  can't spoil the rest.

### Built 2026-10-23 evening (after session 2's failed explore)
**Why explore failed on the real board** (logs e02-20261023-183245, Nick's zip): (1) at an open edge the edge guard
refused every turn after the stop, so each robot stayed with a wheel over the edge and its later rays were lost
(Center spun its wheels 105 cm without moving); (2) the fit started from the best-seen robot's homography, whose
stops lay nearly in a line, and the start fell apart for the robots further away. Each robot alone fitted its own
pixels within 2.6-6.7 px: the data were good, the start was not.
- After an edge stop the robot backs off 10 cm (blind) before turning (`EXPLORE_EDGE_BACK_CM`); a stop where the robot
  had not left the edge is not a side point.
- Wheels turning without the robot moving (found again where it was) count as no move: logged as slip.
- A second start for the fit: the pixels laid flat as a camera overhead would see them, each robot's path fitted to
  that; the better of the two starts is kept. **On the real data: 4.4 px (was 136), gyro scales 1.076-1.086, and
  against the corner-click calibration 2.9 cm median (95%: 5.0 cm) turned and shifted only; with a 6% scale, 1.6 cm.**
  Which of the two has the size right (wheels, or the corner clicks' guessed lens height) is not known yet.
  Regression test: tests/test_e02_explore.py::test_real_board_explore_fits (data tests/data/explore_real_20261023.json).
- Check before trust: each robot alone (`robot_check`: a flat mapping of its own path to its pixels, <= 20 px, 70% of
  stops, gyro 0.85-1.25), then against the joint fit (<= 12 px); one that fails is left out of the camera
  calibration, said, and found again with the new calibration. The joint fit must be <= 8 px with no scale at a
  bound (`fit_check`).
- Never end the session: if explore fails its checks, `explore_fallback` uses the calibration on hand (confirmed the
  usual way) and says so: "!!! EXPLORE FAILED ... FALLING BACK", a line under every run, the session summary, and the
  analysis report's first note. `calib_note()` also marks every run that used a calibration made before the session.
- Explore ends with each robot driven back to where it started (its own wheels and gyro).
- A failed compass check stops and asks (Enter to check again, `skip`, `stop`); automatic mode: once more, then
  without it.
- `--dry-explore-fail` (dry run) tests the fallback.

**Item 3: board lines learned from the finds** (`learn_board_lines`): each finished board find adds where the camera
saw the robot press each board; after 3 touches the line moves to their median (each robot's own steady difference
taken out first, so one odd robot doesn't move a board), when 0.5 cm or more off, capped 8 cm from the picture's line;
logged (`board_line_updates`, `robot_board_offsets`) and said. The robots' own differences are reported, not applied
to their poses (they may be how the camera sees each robot's top; to check against the footage).
**Open edges:** every edge-guard stop in an idle leg is logged (`edge_touches`) with the map's line and said in the
session summary; not applied yet (first real edge stops tonight).

**Item 4: confidence pauses** (`confidence_pause`, threshold `CONF_PAUSE_CM` 3 cm, Nick): own uncertainty
0.5 + 1% of the distance + distance x (0.5 deg + 0.5% of the turn) since the last fix or find (chosen constants);
before a drive that would pass 3 cm it stands and takes a camera fix. Logged in confidence.csv with the camera's
correction (the check on the model). DOE: docs/experiments/DOE-confidence.md.

**Item 5: late pictures** (`pictures_late` > 1 s): no confidence pause (no fresh picture to take); the robot drives on
its own wheels and gyro, and after a leg the front sensor reading a board square on corrects its position along that
board's axis (`board_correct`, sent as a fix, logged as `board_corrections`; at most 15 cm, else ignored).
- **Open (2026-10-23 22:30 dry run):** the edge find can still fail when the robot's only allowed second boundary
  lies behind a parked robot (Right by the open x = 0 edge, Left parked between it and the far board; the
  two-open-edges rule forbids the near open edge). It fails cleanly (the run is redone, no help call). Fix next:
  move the parked robot aside, or allow the two open edges when the touch is far enough from their corner.
  Only matters once a board is gone (or with --edge-find).
- The sides check (`rect_check`) also passed on 4 recorded dry runs and rejected the bad one; the explore in the
  last dry run: 4.8 px, board 125.2 x 118.1 cm (truth 121.55 x 121.3), the y-board line then moved 7.7 cm by the
  finds (item 3 doing its job on explore's worst side).
- **Make way (Nick 22:40: "your call"; coordinated moves, first piece):** a parked robot within 20 cm of an edge-find
  path drives sideways out of it (on the board, 20 cm from open edges and other robots), and goes back to its own
  spot before the next run starts (straight away it met the finder, still by its spot). Logged (`make_way`).
- **Open (dry run 23:20):** after a board is gone, the routes are not re-checked against the new open edge: Left's
  straight route from the start spot ran at the edge where the board had been and the watchdog stopped it. Next:
  re-run `route_fits` when `Board.missing` changes, and leave out (or move) routes that no longer fit.

## Pending from the independent review (2026-10-24)
Full report: kept outside the repo (review of the summary, 2026-10-24). Agreed with Nick: K and the evening C run stay in the summary with these plans.
- **Camera-steered runs survive the runner's re-identify stop.** In session 1, 4 of 9 K runs were ended at the L route's far corner when the front-sensor-vs-map check sent `stop`, which also ended `run e02` mode 1 (logged as safety_stop; no edge guard fired). Fix: pause the check during a K run, or resume the run after it; label it apart from safety stops.
- **Bound the learned gyro drift.** Evening fits gave -0.08 to -0.25 deg/s against a standing-still 0.000; the one evening C run ended 13.2 cm off. Hold drift at the standing-still value (or within 0.05 deg/s of it); refit the evening stretches offline to check it explains the 13.2 cm (DOE note: 0.05 deg/s is chosen; the refit's predicted-vs-observed error decides it).
- Later: blend camera fixes by confidence instead of k = 0.5.

### Built 2026-10-24 (after the review and the four analyses; Nick: "finish the pending build", and the Pi run)
**KC, one weighted fix per stop** (`kc_fixes`, the review's KC findings): before, every still picture was blended half
way (a stop's correction 50-94% by timing) and 160 of 242 fixes arrived after the robot had moved on. Now one fix per
stop: the camera's still position, corrected by the camera's error at that place (`e02_camerr`), weighted against the
robot's own pose (RAW rows) by how sure each is (`CE.blend_weight`: own sd from the uncertainty model since the last
fix, camera sd from the error map, x1.5 in a poor camera-quality cell); sent pre-weighted with `e02_fix_k 1`. Every
stop logged in kc_stops.csv. The stand-still after each leg (`leg_pause_ms`, all mode-0 conditions alike) is set from
the session's picture delay: 300 + 1000 x (0.45 + p95 age + 0.25) ms, 1.0-2.5 s (chosen; DOE in the code).
**The camera's error by place** (`e02_camerr.py`): samples from tape marks (`--marks`: Nick puts a robot by hand on
7 marks measured from the boards' faces, `CE.MARKS`) and from every finished find (its pose is exact against both
boards); a Gaussian-kernel map (25 cm). Runs get `far_err_tape_cm` / `end_err_tape_cm` (live and in the analysis)
when the session had tape marks: KC is judged at the far point against the tape-checked camera.
**K survives the runner's re-find** (`_refind`): no `stop` to a robot in a camera-steered run; a runner stop is
labelled `runner_stop`, never `safety_stop` (`ST.judge_run`, the analysis's ended-early kinds).
**Gyro drift held** (`FIT.fit_robot(measured_drift=)`, `DRIFT_BAND_DPS` 0.05): beyond the band the drift is held at
its edge and the scale refitted. Offline on the evening's data this moves Left's scale to 1.096 (morning 1.072):
drift and scale trade off when every route turns the same way; the plan is a spin check for the scale on its own.
**Explore with the robots scattered** (`explore_gap_rays`): when the rays leave a side unmet, rays are aimed at the
45 deg sectors (round the middle of everything seen) with fewer than 2 ray ends, by the robot whose ray there is
clear of the others; `explore_one_ray` is the single ray, shared with `explore_rays`.
**Routes re-checked when a board goes missing** (`recheck_routes`): routes that no longer fit are left out and their
pending runs put aside (status `left_out`, not counted in the plan's total).
**Open edges learned from edge stops** (`learn_open_edges`): after 3 edge-guard stops of an edge, it moves to their
median, inwards up to 8 cm, outwards up to 3 cm from where it first stood (chosen; DOE with the crawl-speed runs).
**From the four analyses** (docs/build-reports/e02-oct24-analyses.md):
- Camera-drift watch (`CAM.CameraShiftWatch`): every 5 s a picture is aligned to the runs' first one using only what
  lies outside the plywood; the tracker takes the shift back out of every pixel (`px_shift`); said once past 1 px;
  logged (`camera_shift`). `--no-shift-watch` turns it off.
- Camera-quality map (`CAM.QualityMap`): per 20 cm cell, the robot's detail, the floor's brightness and the lock-on's
  share, built while the robots drive; quality_map.csv; poor cells raise the camera's sd for KC.
- The fix before a find carries the runner's heading only if it was measured and the robot's own gyro agrees within
  5 deg (`find_heading_ok`); otherwise position only (Center's 13-24 cm finds came from a camera heading 19 deg off).
**The Pi run** (docs/pi-run.md): `run-e02-pi.sh` (tmux: recorder + runner; USB drive checked; uploads at the end with
`tools/pi/upload_session.py`); `e02.py --on-pi` (camera at 127.0.0.1, the live view open on the WiFi, questions on the
live page with a 15-minute default: `--view-open`, `--web-ask`); `wait_robots` calls the robots by broadcast so they
answer the Pi. The laptop way is unchanged.
**Simulator:** `PLACE_<robot>.json` in the sim folder puts a robot down by hand (the mark check in dry runs).
Tests: tests/test_e02_camerr.py, tests/test_e02_webask.py, tools/pi/tests/test_upload_session.py.
