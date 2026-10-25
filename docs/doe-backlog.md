# DOE backlog: chosen numbers waiting for an experiment

Nick (standing rule): every chosen number says whether a design of experiment should set it. This file lists those
experiments so they get run, instead of staying a promise in a comment. Each session's plan starts by picking from
it.

- **Simulator-only rows:** Claude runs these without asking, whenever the simulator machine is free.
- **Rows that need the board, or Nick's time:** proposed in the next-session plan; Nick confirms.

Status: open / planned (date) / done (result, link).

**Count (2026-10-25 sweep):** 51 rows. 2 simulator-only. 23 need logs or recorded footage but no board: 17 can
run on what is already on disk, 6 wait for data a later session brings. Claude's call: logs-only rows are treated
like simulator rows and run without asking. 26 need the board, the bench or Nick's time. 3 in progress, 1 building,
none done. **Added 2026-10-25 (mark page follow-ups):** 3 rows in Camera and tracking (1 Board: the lights A/B for the floor glow; 2 Logs: the camera scale from known sizes, one of them a tolerance with no DOE). **Added 2026-10-25 (reverse find build):** 3 rows (2 Sim, 1 Board), below. **Added 2026-10-25 (backing in, learned uncertainty, find recovery, [write-up](build-reports/e02-oct25-backing-unc.md)):** 11 rows (5 Logs, 1 Logs + Sim, 2 Sim, 3 Board), 1 chosen with no DOE; the backing-cap row partly done. **Added 2026-10-25 (dock v5):** 6 rows in Docking and charging (2 Sim or Logs + Sim, 2 Bench/Board, 2 waiting on bench test 1); the charge-levels row is in progress. **Added 2026-10-25 (sensor discovery replay, [write-up](build-reports/e02-oct25-discovery-replay.md)):** 7 Logs rows (1 partly done, 1 chosen test design). **Added 2026-10-25 (discovery threshold and calibration moves, [write-up](build-reports/e02-oct25-discovery-threshold.md)):** 11 rows in Sensor discovery (5 Logs, 2 Logs + Sim, 3 Sim then Board, 1 chosen test design, 1 Nick's call); the accept-bar row is done (rule in the code). **Added 2026-10-25 (heading diagnosis, [write-up](build-reports/e02-oct25-heading.md)):** 1 Logs row (heading push). **Update 2026-10-25 (logs run, [write-up](build-reports/e02-oct25-logs-doe.md)):** 7 logs rows done or done
in their logs part (uncertainty model, drift band, jump cut, wheel fit, find heading, trust MIN_N, other trust
limits); 12 need more data or hand-checked truth; sample size has a preliminary count; the front sensor row was
done separately. Sweep sources: code comments (tools/experiment, firmware,
hardware/dock, tools/sim, tools/fusion), docs (build plans and reports, experiments, summary) and this project's
chat. Below the table: numbers already set by data, numbers where a DOE was ruled out, and chosen numbers whose note
names no test.

"Needs" key: **Sim** = simulator only. **Logs** = session logs or recorded footage already on disk. **Board** = a
session on the plywood (minutes are robot time). **Bench** = a bench test from the dock plan. **Nick** = his hands or
a decision.

## Border and safety

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Camera virtual border, away (e02.py BORDER_CM) | 15 cm | Drive at the border line at 4 and 10 cm/s, 5 times each, with and without a planned load; measure the overshoot past the trip point | Board, ~20 min | open |
| Camera virtual border, home (e02.py HOME_BORDER_CM; chat 2026-10-25) | 8 cm | Same runs as above; also the home sessions' border_shadow.csv counts | Board | open; first home session's shadow (10-25 15:10, [write-up](build-reports/e02-oct25-session-1510.md)): 9 trips, 6 false from a sign fault in border_check (to fix before counting), 3 real approaches at 14.1-14.6 cm, home's 8 cm caught the 2 forward ones at 7-8 cm |
| Robots' own border (e02.py FW_BORDER_CM / HOME_FW_BORDER_CM; chat 2026-10-25) | 10 / 6 cm | Same runs, with the camera covered for the last stretch (the robot's own count alone) | Board | open |
| Recheck before "lost" (e02.py RECHECK_S; build-reports/e02-oct24-right-session.md; chat 2026-10-25) | 3.5 s | Count rescues against holds over the next 3 sessions; replay sessions' tracks with 1-6 s | Logs + Sim | needs more data (2026-10-25 replay of 677 go legs: the old sessions refound within a median 1.9 s, so 41 of 46 disagreements cannot be followed to 3.5 s; count rescues in the next sessions; [write-up](build-reports/e02-oct25-logs-doe.md) §6) |
| Bad-camera border shrink (e02.py BORDER_EXTRA_SIGMA / BORDER_EXTRA_MAX_CM) | 2 sd / 10 cm | Simulator grid at camera noise 1, 2, 3 cm: holds and off-board count against runs completed | Sim | in progress (grid). Partial, chat 2026-10-25: at 4 cm extra noise, holds 17 → 7, but no scored runs (4 cm treated as beyond the system for now); 2 cm rerun running |
| Park spots moved in (e02.py PARK_OPEN_MAX_CM; chat 2026-10-25) | 22 cm | Simulator grid, noisy camera: parking holds and near-misses against the route clearance | Sim | in progress (grid) |
| Bottom-zone edge cue (e02.py SAFETY_SETTINGS edge_bot_cm; params.h default 13 is a placeholder) | 11 cm | Once the crawl runs (DOE-explore-crawl) log which cue fired and its reading: limit against false stops on plywood and distance past the edge; tests-to-run item 9 | Board (same runs as the crawl DOE) | open |

## Camera and tracking

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Detector check rate (e02_camera.py CHECK_S; Nick 2026-10-22: "start at 2 s") | 2 s | [DOE-tracking.md](experiments/DOE-tracking.md) DOE 1: 1 / 2 / 4 s on replayed footage; wrong-position time, time to catch, laptop cost | Logs (replay) + hand-checked truth, ~1 min per footage minute | needs hand-checked truth (a position checked about once a second; the clips have 3-6; [write-up](build-reports/e02-oct25-logs-doe.md) §12). 2026-10-25: the marking page is built ([how-to](how-to/mark-the-robot.md)); waits on Nick marking the three evening stretches (~20 min) |
| Detector-lock agreement (e02_camera.py AGREE_CM) | 6 cm | Same replay, 4 / 6 / 9 cm (DOE-tracking.md DOE 1) | Logs (replay) | needs hand-checked truth ([write-up](build-reports/e02-oct25-logs-doe.md) §12) |
| Lock trusted over detector (e02_camera.py LOCK_SURE) | 0.7 | DOE-tracking.md "other chosen numbers": same replay method | Logs (replay) | needs hand-checked truth ([write-up](build-reports/e02-oct25-logs-doe.md) §12) |
| Lock pinned during a turn (e02_camera.py PIN_CM) | 3 cm | DOE-tracking.md "other chosen numbers": same replay method | Logs (replay) | needs hand-checked truth ([write-up](build-reports/e02-oct25-logs-doe.md) §12) |
| Look matched every N degrees (e02_lock.py TURN_STEP_DEG) | 10° | DOE-tracking.md: same replay at 5 / 10 / 20°; fit against ms per picture | Logs (replay) | needs hand-checked truth ([write-up](build-reports/e02-oct25-logs-doe.md) §12) |
| Lock match accepted (e02_lock.py MATCH_OK) | 0.55 | Same replay at 0.45 / 0.55 / 0.65: wrong locks against lost locks ("to be set from footage: DOE") | Logs (replay) | needs hand-checked truth ([write-up](build-reports/e02-oct25-logs-doe.md) §12) |
| Floor vs robot edge limit (e02_lock.py MIN_LOOK; DOE-tracking.md) | 38 | Same replay on more footage, including the bright corner (floor 27, Right washed out at 49: thin margin) | Logs (more footage) | needs hand-checked truth and footage of the bright corner ([write-up](build-reports/e02-oct25-logs-doe.md) §12) |
| Robot found where its wheels put it (e02.py DETECT_NEAR_CM) | 6 cm | tests-to-run "Detect after a move": finds against wrong pick-ups at 4 / 6 / 9 cm, from the `detect_after_move` records | Logs (next sessions) | needs more data (5 pick-ups at 2.4-5.2 cm, all right; outlines beyond 6 cm are not logged, so 9 cm cannot be scored: log them; [write-up](build-reports/e02-oct25-logs-doe.md) §7) |
| Tracker health ok / amber / red limits (e02_view.py health; also the pause-to-confirm threshold, engineer-loop item 1) | chosen limits | [DOE-tracking.md](experiments/DOE-tracking.md) DOE 2: Part A threshold 0.3-0.9 by replay (ROC: false pauses against missed errors), Part B confirmation cost, 120 trials | Logs, then Board ~1 h | Part A needs hand-checked truth ([write-up](build-reports/e02-oct25-logs-doe.md) §12) |
| Poor camera cells (e02_camera.py QualityMap POOR_LOCK / POOR_DETAIL; e02.py KC_POOR_CELL_SD_FACTOR) | 0.4 / 25 / 1.5 | KC runs with and without the poor-cell weighting; the factor from tape marks in poor cells | Board (KC runs + tape marks) | open |
| Camera drift watch (e02_camera.py CameraShift PERIOD_S / WARN_PX; build-reports/e02-oct24-analyses.md) | 5 s / 1 px | Board-line checks with and without the shift correction, on recorded sessions | Logs | needs more data (Pi sessions: shift at most 0.43 px = 0.2 cm, under the board touches' own scatter; needs a session with real drift; [write-up](build-reports/e02-oct25-logs-doe.md) §11) |
| Side camera region bands and still moment (build-reports/e02-oct24-sidecam.md) | edge 12 cm, corner 25 cm; still 0.4 s under 1 cm | Small DOE (band x still rule), only if the trust map is built per region | Logs | open (only if the per-region trust map is built) |
| Motion tracker settings on the side view (build-reports/e02-oct24-sidecam.md) | the overhead's values | Tuning DOE once there is footage where the overhead loses robots | Logs (that footage) | open (waits for footage) |
| Floor glow beside the robot (firmware: alvik.begin() turns the underside colour-sensor light on and nothing turns it off during runs; home_lab.ino only does at sleep; Nick 2026-10-25: "run with the light off") | light on in every session | **Lights A/B, short board test.** One robot, the same 4-leg square route (about 40 s) driven 4 times in the order on, off, off, on (ABBA, so a slow change of daylight or battery cancels). Off = black electrical tape over the colour-sensor window at the front of the underside (no firmware change; the colour sensor is not used in E02 runs; the line sensors beside it stay uncovered), or a one-line firmware switch if Nick prefers. Recorded as usual, each run then marked on the mark page (one picture a second, about 160 marks). Responses: the tracker's position error against the marks (median and share over 5 cm; mark_robot.score_track on the replayed track) and the glow itself (mean brightness of a robot-sized patch just in front of the robot minus the floor's, read from the pictures at the marks; no hand work). A third arm (top lights dark) only if the glow turns out not to be the underside light. 4 runs: enough to see a glow that moves the tracker by centimetres, not to measure a small effect | Board, ~10 min + ~10 min marking | open (designed 2026-10-25, not run) |
| Camera scale from known sizes (tools/label/size_check.py LIMIT_PCT; the mark page's Size check) | ±2 % pass bar (chosen from the 5 cm budget: 2 % is 1.2 cm at 61 cm from the board's middle) | Not a DOE: a tolerance. The check itself is the measurement: about 15 hand fits of the robot's top spread over the board (one fit ±1.5 %, sd, measured in the tests), then the automatic outline on every marked picture, corrected by the hand fits' padding. A region that is off: re-make the calibration and check again | Logs (marked footage) + Nick ~10 min | open |
| Automatic outline padding (size_check.auto_padding) | measured per stretch from 3 or more pictures fitted both ways | Settles itself from the hand fits; whether one padding holds across the board and the lighting: the same data split by region (the bright corner against the rest) | Logs (marked footage) | open |

## Calibration and fits

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Confidence pause (e02.py CONF_PAUSE_CM; Nick 2026-10-23: "3 cm and DoE to finalize") | 3 cm | [DOE-confidence.md](experiments/DOE-confidence.md): 1.5 / 3 / 6 / off, about 72 runs in blocks of robot x route; end error against pause time | Board, ~1.5 h | open |
| Uncertainty model (e02.py UNC_BASE_CM / UNC_PER_CM / UNC_DEG / UNC_DEG_PER_DEG) | LEARNED every session (e02_unc.py); start values 0.5 cm, 1%, 0.5°, 1.4% per degree turned | DOE-confidence.md "before running it": regress confidence.csv correction_cm on distance and turn since the last fix. Dry run 2026-10-24: model said ~4 cm where the simulator's truth was 1.6 cm | Logs (confidence.csv), then Sim | done, logs part (2026-10-25): too confident (59% of 673 corrections inside its 95% range); refit: turning term 0.014 per degree (was 0.005), the rest unchanged; 17% of chains end with the heading 30°+ wrong, not covered. Sim next; [write-up](build-reports/e02-oct25-logs-doe.md) §1. Heading part traced (2026-10-25): 100 of the 112 come from fixes writing an unmeasured runner heading (±90 placeholder, U run's end heading echoed, pre-turn heading after a refind), not the gyro; without those chains the turning term is 0.0126 per degree; fixes proposed, [write-up](build-reports/e02-oct25-heading.md). Sim and learning built (2026-10-25, [write-up](build-reports/e02-oct25-backing-unc.md)): the four numbers are now fitted every session from each robot's own camera corrections (earlier sessions', then its own from 30 on), so the start values only matter for a robot with no history; the learner's own chosen numbers have their rows in "Backing in, learned uncertainty, find recovery" below |
| Gyro drift band (e02_fit.py DRIFT_BAND_DPS; build-plans/e02-oct22-build.md) | 0.05 °/s | Offline refit of the 2026-10-24 evening stretches at 0.02 / 0.05 / 0.1; predicted against measured end errors | Logs | done (2026-10-25): band 0 (drift held at the measured value) predicts best, 0.12-0.15 cm better than 0.05 (p < 0.001, 49 runs); recommended 0; [write-up](build-reports/e02-oct25-logs-doe.md) §2 |
| Tracker jump cut (e02_fit.py JUMP_CM; analyses_oct24/night_selfcal.py; build-reports/e02-oct24-night-analyses.md) | 3 cm | Already swept 2 / 3 / 4 cm (2 cm drops 20-35% of counted distance); still to do: score each cut by how well its wheel scales predict C-run end errors | Logs | done (2026-10-25): 2 / 3 / 4 cm give the same predicted end errors (≤ 0.09 cm apart); keep 3; [write-up](build-reports/e02-oct25-logs-doe.md) §3 |
| Wheel fit accepted (e02_fit.py MAX_RMS_CM; e02.py --fit-max-rms-cm) | 1.5 cm default, 2.5 cm for sessions (Nick 2026-10-24) | Replay sessions' fits at 1.5 / 2 / 2.5 / 3 cm: fits refused against C-run end error with the fit used | Logs | done (2026-10-25): no effect on predicted errors (every fit passes 1.5 cm with the jump cut); the fixed prior predicts as well as the session fit; back to 1.5 cm; [write-up](build-reports/e02-oct25-logs-doe.md) §4 |
| Camera noise per stretch (e02_fit.py SIGMA_DEG / SIGMA_CM, "assumed") | 1.5° / 0.5 cm | Two cameras timing the same stretch give a measured value (build-plans/second-camera-plan.md) | Board + second camera | open (waits for the second camera) |
| Camera error map smoothing (e02_camerr.py BANDWIDTH_CM; also how many marks and where, build-plans/generality-audit.md) | 25 cm, 7 marks | Leave-one-mark-out error at 15 / 25 / 40 cm, once a session takes the tape marks | Board (tape marks, Nick ~15 min), then Logs | open |
| Camera error map weights (e02_camerr.py MARK_W / NOISE_SD_CM / NO_DATA_SD_CM; "all chosen, DOE notes beside them") | 4 / 0.7 cm / 3 cm | Same tape-mark data: a held-out mark's error against the map's predicted sd, at each setting | Logs (after the tape marks) | open |
| Camera heading sd (e02.py KC_HEAD_SD_DEG; its note "DOE with tape marks" sits displaced on the WEB_ASK_TIMEOUT_S line) | 3° | Camera heading from a straight stretch's end against two tape marks' bearing | Board (tape marks) | open |
| Explore calibration accepted (e02.py MARK_ACCEPT_CM) | 2.5 cm | Explore's mark error against the runs' errors over sessions with --marks (2026-10-24 dry run: 4.3 cm passed explore's own checks, 29 robot losses followed) | Board (sessions with --marks) + Logs | needs more data (no real session has finished explore with --marks; [write-up](build-reports/e02-oct25-logs-doe.md) §5) |
| Heading before a find (e02.py FIND_HEAD_AGREE_DEG) | 5° | The find's heading correction against this limit, from FIND records (Center's finds 13-24 cm off came from a heading up to 19° wrong) | Logs | done, weak (2026-10-25, 39 finds): at 8-10° no find starts over 10° wrong (one at 5°); 10 suggested; [write-up](build-reports/e02-oct25-logs-doe.md) §8 |
| Heading push after a measured go leg (fix 3 of [e02-oct25-heading.md](build-reports/e02-oct25-heading.md); in the code since 2026-10-25: e02.py HEADING_PUSH_DEG) | 8° | The spread of a 10-15 cm go leg's camera direction against the robot's own heading on legs where both are right (heading_legs.csv); the threshold just above that noise | Logs | open |
| Front sensor offset and tolerance (e02.py FRONT_OFFSET_CM / FRONT_TOL_CM; tests-to-run item 10) | 0 / 8 cm | Set from front_checks.csv (e02_analysis prints suggested values). Data is in hand: four sessions give a spread of 4.0-4.6 cm and medians -3 to +1 (e02_trust.py HEALTHY), but not applied | Logs | open (data in hand) |

## Sensor trust

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Trust flags (e02_trust.py DEGRADED_K / RECOVERED_K / STANDS_OUT) | 2 / 1.5 / 1.5 | Swap test: 20 injected faults per sensor and level (simulator), then real; detection rate and false alarms per hour. Since the trust fix (2026-10-25) STANDS_OUT no longer gates a flag (each robot is judged on its own first); the 3 remaining false flags on the real replay are gyro flags at exactly 2.0 on the Pi sessions, so 2 / 2.5 matters most for the gyro | Sim, then Board | open |
| Still-camera healthy scatter (e02_trust.py CAMERA_STILL_HEALTHY; e02.py STILL_WINDOW_S) | 0.4 cm over 3 s | A robot standing still at 7 places for 30 s each | Board, ~10 min | open |
| Evidence before a sensor's k is used (e02_trust.py MIN_N; night-analyses test 2) | camera 4, wheels 3, gyro 3, front 6 | Replay the healthy 2026-10-23 sessions and count false flags at 3 / 5 / 8 (it flags Right's wheels on the morning today) | Logs | done, healthy half (2026-10-25): wheels/gyro 3 give 3 false flags in 4.8 h, 5 give none; 5 suggested after the swap test; [write-up](build-reports/e02-oct25-logs-doe.md) §9 |
| Baseline floors, each robot judged against its own history (e02_trust.py BASE_FLOOR; trust fix 2026-10-25, [write-up](build-reports/e02-oct25-trust-fix.md)) | wheels 0.25 cm, gyro 0.3°, front 2 cm a stretch / reading | Replay healthy sessions against each other's history (simulator grid baselines, then real sessions on one camera setup) at 0.2 / 0.25 / 0.35 (gyro 0.2 / 0.3 / 0.5): false flags against the 5% gyro and 8% wheel cases still caught | Sim + Logs | open |
| Real robots' run-to-run wheel scale spread, the runs check's bar (e02_trust.py RUN_SPREAD_PRIOR; trust fix 2026-10-25) | 2.6% (measured: median of 12 real robot-sessions, 0.4-4.8%) | Not chosen, but thin: more real sessions, split by camera setup (laptop / Pi) and by route, to see whether the spread is the camera's error across the board (then a per-region correction would lower the bar) or the wheels' own | Logs (next sessions) | open |
| Second check's "stands out" ratio (e02_trust.py STANDS_OUT_FLAG; trust fix 2026-10-25) | 3 x the healthy robots' median | Same replay as the floors at 2 / 3 / 4: the simulator's healthy robots differ by up to 1.5x, Right's 5% gyro was 4x | Sim + Logs | open |
| Evidence before a check speaks or a history replaces the prior (e02_trust.py MIN_N runs 4; HIST_MIN wheels 3, gyro 3, front 6, runs 4; trust fix 2026-10-25) | as listed | The MIN_N replay above, extended: how early a fault is caught against false flags at 4 / 8 runs and 3 / 5 fits | Sim + Logs | open |
| Shortest run whose wheel scale counts (e02_trust.py RUN_MIN_LEN_CM; trust fix 2026-10-25) | 40 cm | Spread of the run scales in healthy simulator and real sessions at 20 / 40 / 80 cm: the point below which short runs add camera noise rather than evidence | Sim + Logs | open |
| Face readings per robot per session (e02.py FACE_TARGET; trust fix 2026-10-25) | 8 | Simulator front:bias=5 and noise=6 at 4 / 8 / 16: how soon the sensor is flagged, against the time the readings cost (about 3 s each) | Sim | open |
| Finds before they speak for the camera (e02_trust.py CAMERA_FIND_MIN; trust fix 2026-10-25) | 8 items (4 finds) | Grid replay and simulator at 4 / 8 / 12: how soon a bad camera is caught (the still check catches it first) against false camera flags when every robot's wheels jitter (a find's back-off is wheel counting: two finds read 1.3-2.2 cm and flagged the camera at 4) | Sim + Logs | open |
| Front sensor at a board, before a robot has a history (e02_trust.py FACE_HEALTHY, FACE_MIN, BASE_FLOOR face; trust fix 2026-10-25) | 2 cm sd; 4 readings; floor 1 cm | The first real session's face readings (8 per robot) give the real healthy level; then the simulator front:noise at 2 / 4 / 6 cm against it for how small a fault is caught. Simulator healthy: 0.6 cm | Logs (next session) + Sim | open |
| Run with no end record taken as over (e02.py RUN_LIVE_MAX_S; trust fix 2026-10-25) | 600 s | None: a safety limit for a lost record; runs last under 5 minutes | - | chosen, no DOE |
| Other trust limits (e02_trust.py DECAY_PER_SESSION / GATE_K / GATE_MAX_CM / KC_GATE_SIGMA). The module says "DOE notes beside them"; none is written. The test here is Claude's suggestion | 0.5 / 1.5 / 3 cm / 3 sd | Score them in the same runs as the swap test plus the healthy replay: false refusals against faults caught | Sim + Logs | healthy half done (2026-10-25): GATE_K 1.2 refuses 2 good fits, 1.5-2 none; GATE_MAX_CM never reached; decay and KC_GATE_SIGMA 3 fine; keep all; faults-caught half needs the swap test; [write-up](build-reports/e02-oct25-logs-doe.md) §10 |

## Explore and edges

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Crawl speed at an unknown edge (e02.py EXPLORE_CRAWL_CMS; Nick 2026-10-24) | 4 cm/s | [DOE-explore-crawl.md](experiments/DOE-explore-crawl.md): 2 / 4 / 6 / 8 cm/s x 0° / 30° / 60°, 180 approaches; lowest-5% margin above 1 cm | Board, ~45 min (foam strip) | open |
| Bottom zone's reach ahead (e02_map.py BOTTOM_AHEAD_CM; EXPLORE_EDGE_BACK_CM 9.5 rests on it; build-plans/e02-oct22-build.md "to be measured") | 9.0 cm | The crawl runs' stops: camera-measured front wheel place against the plywood's end, bottom-zone stops only; explore's refine_edges estimate each session as a check | Board (same runs) + Logs | open |
| Open edge learning caps (e02.py EDGE_LEARN_IN_CM / EDGE_LEARN_OUT_CM) | 8 / 3 cm | Edge stops at tape-measured distances from the plywood's end (the crawl runs) | Board (same runs) | open |
| Heading drive length (e02.py heading drive 10 / 15 cm, HEADING_LONG_ROOM_CM 30; tests-to-run "Heading drive length") | 10 cm, 15 cm with 30 cm room | 10 / 15 / 20 cm x middle / beside a board / near an open edge, ~8 repeats per cell; heading error, camera losses, distance toward an edge | Board, ~30 min | open |
| Explore ray length and extension (e02.py EXPLORE_RAY_CM 105; firmware cap 110; the proposed "one more leg if the front reads under 60 cm" in [write-up](build-reports/e02-oct25-session-1510.md) fix 6) | 105 cm, no extension | Replay explore's stops offline with the ray cut at 80 / 105 / 105 + extension (the logged front readings say how far each side was); then on the board, robots put down in a cluster on one side, 3 sessions: sides met and explore time | Logs + Sim, then Board | open |
| Lost-robot limit during a turn on the spot (e02.py LOST_DRIVE_S 1.5 s, shared with drives) | 1.5 s | Camera gaps during turn legs across sessions (10-25 15:10: 3.0, 1.8, 1.8 s, all false holds): the gap length that separates "turning beside something" from a real loss | Logs (more sessions) | open |
| Explore home-leg clearance (e02.py EXPLORE_RAY_CLEAR_CM 25, rays only now) | 25 cm, rays only | Same clearance on home and return legs in the simulator's scattered starts: collisions against legs re-aimed and explore time | Sim | open |

## Docking and charging

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Charge levels (build-plans/self-charging-dock.md; tools/dock/charge_levels.py) | go from its parts (49-54 % on the logs' rates), leave = the knee (80 % PLACEHOLDER), top-up L - 20 | Method in place (2026-10-25): drain learned per robot from the logs (battery_rates.py), go = error + job + drive + queue + floor. Remaining: bench test 1's charge curve, then `charge_policy_sim.py --compare` | Bench test 1 + Sim | in progress (drain part done on 6 sessions; charge curve open) |
| Drain margin (charge_levels.py P k_se; v5) | rate + 2 standard errors | Simulator: each robot's drain drawn from its learned uncertainty, k = 1 / 2 / 3, 7 days x 5 seeds: flat batteries against uptime | Sim | open |
| Battery learner thresholds (battery_rates.py SESSION_MIN_HOURS, SHIFT_MIN / SHIFT_SE, RISE_SPLIT, MIN_HOURS; v5; sessions are no longer joined, 2026-10-25) | 0.75 / 0.25 / 0.75 h; 3 %/h and 3 se; 3 %; 0.1 h | Replay the sessions on disk with a drain change injected (+5 %/h from mid-session) and without: how soon it is flagged against false flags, at 0.25 / 0.5 / 1 h | Logs + Sim | open |
| Rest policy (charge_policy_sim.py --rest K; Nick 2026-10-25: rest robots off the dock before a second dock) | none chosen | `--compare` once bench test 1 and the sleep test give the charge rate and the sleep drain: the sleep drain decides whether resting helps (it fails below about 25 %/h at 3 %/h asleep) | Bench test 1 + sleep test, then Sim | open |
| Contact force: pad position, spring rate, press (build_dock_v6.py PAD_X_MM, SPRING_K_N_MM, PRESS_MM; v6 2026-10-25 13:03: pads on the battery cover, two springs per plate) | 19 mm behind the axle (set by the cover), 0.15 N/mm each, 1.0 mm (v5: 8 mm, one 0.2 N/mm, 1.3 mm) | Weigh the caster first (plan Appendix B step 0b). Then 2 spring rates (penny-test difference about 17 and 30 g) x 2 presses (one paper shim) x 20 back-ins: contact resistance, contact within 10 s, caster pinch at the 44 % peak | Bench + Nick (springs) | open |
| U arms and mouth (build_dock_v6.py ARM_IN_HALF_W_MM, FLARE_L_MM / FLARE_MM, ARM_END_X_MM) | +-1.5 mm play, 20 mm chamfer opening 12 mm (catches 13.5 mm; v5: 10 mm x 7), arms to 115 mm | Bench test 6's back-ins from staging offsets of 0 / 7 / 13 mm and 0 / 5 deg: count clean dockings and how far the funnel twists the robot (gyro); shorten the chamfer back if 7 mm never misses | Board + Nick | open |
| Driving's extra drain, shared across sessions (battery_rates.py fit / BatteryModel.extra; item 4, 2026-10-25) | each session its own standing base, one driving extra; default 4.0 +- 2.0 %/h (MEASURED: 3.6-5.6 per robot) | Logs-only: a 2-hour session with long driving stretches (e.g. 15 min driving, 15 min standing): does the extra stay the same between sessions (then pooling it is right), or move with the base (then fit moving per session too) | Logs (next long session) | open |
| Charge-curve learner (battery_rates.py CHARGE_KNEE_FRAC, CHARGE_BAND, CHARGE_MIN_H) | knee where the rate falls below 70 %, 5 % bands, 0.25 h of charging before use | Logs-only on bench test 1's charge log against the USB meter's mAh: the knee and rate it finds at 60 / 70 / 80 % and 2 / 5 % bands | Bench test 1's log | open |
| Pad carrier fit (build_dock_v6.py CARRIER_INSET_MM, TAPE_T_MM, the cover numbers COVER_*) | 0.8 mm inside the seams; 0.2 mm tape; cover from the photo | Not a DOE: measured (plan Appendix B step 0: steel rule on the cover, paper-stack pad height) | Nick (5 min) | open |
| Dock marker size (build_dock_v5.py STRIP_CELL_MM, STRIP_N) | 2 x 6 squares of 15 mm | E00e Part C: the 15 mm strip against the old 10 mm tile, a 1 px bump (shim under the camera mount) 10 times each, and a robot docked beside it: caught / false alarms | Board + Nick (~20 min) | open |
| Pin protrusion and pad recess (build_attachment_v2.py STEP_VBUS_MM / STEP_GND_MM / PAD_RECESS_MM) | +1.0 mm, GND 1.3 mm, recess 0.3 mm | Small DOE if bench test 6 shows misses or a shifted fix: two step heights x 20 dockings, contact rate and seat repeatability | Bench + Nick (printing) | open (only if test 6 shows misses; contacts may change to floor strips, chat 2026-10-25) |
| Pin hole fit (build_attachment_v2.py POGO_HOLE_PRESS_MM) | 0.05 mm over | Print the gauge (pogo_gauge.stl) and use its best hole | Nick (one print) | open |
| Dock fuse (self-charging-dock.md, PTC hold) | 1.1 A | Bench test 1's measured peak current x 1.5 | Bench, ~1.5 h (tests 1-3) | open |
| Dock sense and leaving (self-charging-dock.md: sense wait, retry, back-out) | 10 s, 1 retry, 3 cm | Bench test 6's timings | Bench | open |
| Docked position promise (experiments/ACCURACY-CHECKS.md C19; dock_sigma_cm / dock_sigma_deg) | 0.5 mm / 0.3° (simulator's guessed contact model) | E0e Part A: 20 tape-measured dockings per robot; repeat with the pins in (bench test 6) | Board + Nick | open |

## Timing and performance

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Slow thresholds (e02_load.py SLOW_FPS_FRAC / SLOW_AGE_FRAC / SLOW_HOLD_S, baseline BASE_ROWS; chat 2026-10-25) | 0.75 / 1.5 / 5 s; baseline 120 rows | Load at two strengths (cpu:1, disk upload) in a session; pick the thresholds that flag the load blocks and nothing else | Board session with --load | open |
| Pause for a camera fix (e02.py LEG_PAUSE_MIN_MS / LEG_PAUSE_MAX_MS, KC_STILL_S, KC_SEND_MARGIN_S) | 1.0-2.5 s; 0.45 s; 0.25 s | Fixed pauses of 1.0 / 1.4 / 2.0 s: share of stops whose fix landed against the time cost | Board (KC runs) | open |

## Other

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Dance error allowance (e02_dance.py DEFAULT_E95_CM; gap K_GAP) | 6 cm (x 2) | Measure at least 59 dance moves (MIN_PROOF_MOVES); the code then uses the measured 95th percentile instead | Board | open (no dance measured yet) |
| Sample-size minimums (build-plans/publication-plan.md) | 3 camera poses, 3 sessions, 20 injected trials, 10 healthy hours; sd 0.8-1.5 cm from one session | Not a DOE: a power calculation from the first new session's sd, in week 3 | Logs | open; preliminary (2026-10-25): all 13 C runs give 25 per arm with a robust sd, 247 with the ordinary sd; the outlier rule decides it; [write-up](build-reports/e02-oct25-logs-doe.md) §13 |
| Fusion settings, older lane (tools/fusion/engine.py; ACCURACY-CHECKS.md B19) | simulator values | Tune on half of T1, score on the other half | Logs | building (split rule), per ACCURACY-CHECKS.md |
| Live lab restart threshold (tools/experiment/e02_live.py `on_camera`, squared distance) | 60 | Done on logs ([report](build-reports/e02-oct25-real-fusion.md), `analyses_oct25/live_restart_doe.py`): threshold 15 / 30 / 60 / 120 / 240 / never x start rule (today / per-robot filter started at the camera / + heading from the camera's travel / + the runner's floor heading), all 15 real session folders. The threshold was not the cause: today's lab restarts its filter at every other robot's RAW row, at the library's own-frame pose (94,583 filter starts in 2.5 h). With the camera start rule the error is flat from 60 to never (prediction 95th pct 2.66-2.69 cm, 3.8-3.9 % of time over 5 cm); 120 gives 14 restarts an hour (60: 49). Recommended: 120 with the start rule (R4), not yet in the code. Next: a replay DOE of the straight-travel distance for the camera heading (3-8 cm; 5 chosen) | Logs | done (rule recommended, not applied) |
| First real-session fusion: camera-gap layout (real_fusion.py GAPS, GAP_SEP_S, NEAR_BOARD_CM) | 2-40 s gaps, 15 s between, 15 cm from boards | Not a DOE: analysis settings. A sensitivity rerun with 10 s between gaps and a 10 / 20 cm board cut | Logs | open (optional) |

## Reverse find (backing into the boards, firmware 2026-10-25b)

[Build report](build-reports/e02-oct25-reverse-find.md). All in firmware/home_lab/params.h.

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Look allowance (params.h e02_find_see_cm) | 20 cm | At the staging point, facing each board: 20 looks per robot with the board in, 5 with it lifted out (the robot stops before backing); put the limit between the two spreads, nearer the "in" side. The simulator only has a flat ray model, so this needs the board | Board, ~15 min (part of the first backing-in session) | open |
| Backing cap (params.h e02_find_back_margin_cm, e02_find_keep_cm) | 3 cm + 20% of travel; axle 2 cm from the line | Simulator grid: margin 2 / 3 / 5 x keep 2 / 4 / 5.5 x gyro corrected / uncorrected x camera fix noise 0.3 / 1 / 2 cm, 20 finds each: finds done, false "no board behind", and (board removed, look off) how near the axle and the rear caster come to the line. 5.5 keeps the caster on a plywood edge at the line; whether that matters depends on where the boards stand (question to Nick) | Sim | partly done (2026-10-25, [write-up](build-reports/e02-oct25-backing-unc.md)): boards stand against the plywood's side (Nick), so the line is the edge. Keep 2 / 3.5 / 4.6 cm x camera fix 0.8 cm 2° / 1.5 cm 4°, gyro corrected, 30 finds each, board in: false "no board behind" 2 / 4 / 8 of 30 and 4 / 9 / 21 of 30. Keep stays 2 (the look protects the caster; 4.6 would hold Nick on a quarter of the finds). Margin and the look-off caster distance not yet run |
| Nose-first approach before turning round (params.h e02_find_turn_cm) | 30 cm | Simulator: starts 30-90 cm from a board with no staging point, turn_cm 12 / 20 / 30 / 50: reset error, failures, time | Sim | open |

## Backing in, learned uncertainty, find recovery (2026-10-25)

[Build report](build-reports/e02-oct25-backing-unc.md).

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Uncertainty floors (e02_unc.py FLOOR) | 0.2 cm, 0.5%, 0.2°, 0.5% per degree | Replay the 10 logged sessions: fit each from its own corrections with floors at half / these / double; score by the share of the NEXT session's corrections inside 2.45 sd (95% if right) | Logs | open |
| Corrections before a session's own model is used (e02_unc.py SESSION_MIN) | 30 | The same replay at 15 / 30 / 60, plus a session with one robot's gyro scale changed on purpose (the simulator's --degrade): how soon the change is reported against false "changed" reports | Logs + Sim | open |
| Evidence before a term is fitted (e02_unc.py TERM_MIN, DRIVE_MIN_CM, TURN_MIN_DEG) | 6 corrections; 25 cm; 90° | The replay above: terms fitted on fewer against their full-session values | Logs | open |
| Change report (e02_unc.py SHIFT_FRAC, LR_CRIT) | 25% at 50 cm after 180°; chi-square 95% | Replay with sessions shuffled between robots: false reports per session | Logs | open |
| Camera noise taken out of the corrections (e02_unc.py CAM_SD_CM) | 0.8 cm per axis where e02_camerr has no number | The camera error map's tape-mark rows (Camera and tracking) | Board (tape marks) | open |
| Look in the find recovery (e02.py FIND_LOOK_TOL_CM) | 15 cm past where the map puts the board | Same readings as "Look allowance" above, taken from the recovery's spot (the robot's own, not the staging point) | Board (with the look allowance) | open |
| Camera line in the find recovery (e02.py FIND_LINE_TOL_CM) | 3 cm from the setup's line | The board's floor line re-measured with the board in and lifted out, 10 pictures each. With the boards against the plywood's side the plywood's own edge may look the same: this decides whether the picture check can tell at all | Board, ~5 min | open |
| Second staging point (e02.py FIND_RESTAGE_CM, FIND_RESTAGE_ALONG_CM) | 35 cm out, 25 cm along | Simulator: "no board behind" finds (gyro uncorrected) retried from 25 / 35 / 50 cm out: share that then succeed | Sim | open |
| Board visit backing in (tools/fleet planner touch_lead_cm, touch_over_cm) | 3 cm; 1 cm | Kinematic and fleet simulator: pose error at the pre-dock 0.3 / 1 / 2 cm x lead 2 / 3 / 5 x over 0.5 / 1 / 1.5: false "no click", visit time, and (board taken away) how far the rear caster goes past the edge | Sim | open |
| Own heading fresh enough to judge the runner's before a find (e02.py FIND_OWN_FRESH_S) | 15 s | Logs: each robot's standing-still gyro creep (the naming check's gyro run, and the heading drift between legs while parked in heading_legs.csv) x how long robots stand parked between finds | Logs | open |
| Holds for one find before it counts as failed (e02.py FIND_HOLD_MAX) | 3 | None: it trades Nick's time, not accuracy | - | chosen, no DOE |

## Sensor discovery (blind replay, 2026-10-25)

[Build report](build-reports/e02-oct25-discovery-replay.md). All in `tools/discovery/`. Each row is a logs-only rerun
of `tools/discovery/replay.py` at several values, scored on the held-out cases (robot Right; 10-24) and the disguised
reruns. Wrong accepted labels are counted against questions asked.

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Accept / ask bars (classify.py ACCEPT, ASK; design doc §7) | 0.90 / 0.60, with a 60 s motion gate | Done on log pieces ([report](build-reports/e02-oct25-discovery-threshold.md)): threshold 0.70-0.95 x motion (pieces with 10-160 s of wheel motion, whole logs, 9 s logs) x units plain / disguised, bootstrap over cases, run twice (before and after fixing the rules that scored full marks on little evidence). Rule chosen on dev, confirmed on held-out and now in the code: accept at 0.90, motion-type labels only after 60 s of motion (held-out 1 wrong in 1,374 plain, 2 in 1,211 disguised; 11.5 questions per robot). Next: rerun on a real calibration-battery log | Logs | done (rule in classify.decide); recheck on a real battery |
| Exact-relation bars (relations.py EXACT_R 0.98, classify.py COPY_R 0.97, PLANAR_R 0.80, PLANAR_EXCESS 0.30) | as listed | Sweep each (0.95-0.995; planar 0.6-0.95, excess 0.1-0.5); score derived-stream tags and false "derived" on sensors, plain and disguised | Logs | open |
| Relation windows and grid (features.py WIN_S 1 s, + 2 s; relations.py PLANAR_WIN_S 2 s; GRID_HZ 10; SEG_GAP_S 1 s) | as listed | Sweep windows 0.5-4 s and grid 5-20 Hz on the replay; count integral pairs found (r >= bar) in short and long cases | Logs | open |
| Sensor-group affinity (relations.py AFFINITY_GROUP) and stillness (flat run 1 s, edge trim 0.3 s) | 0.45; 1 s; 0.3 s | Sweep 0.3-0.8; score the groups against the true sensor groups (zones, line, IMU, orientation) | Logs | open |
| Score floors and weights (classify.py UNKNOWN_FLOOR 0.05, short-case ramp 30-120 s, IMU-axis cap 0.45, gravity level / noise 300, range/line weights, camera drive threshold features.py CAM_MOVE_CMS 2.5 cm/s) | as listed | Not a one-factor sweep: fit them on the development split, check on the held-out split, report the confidence calibration (are 0.9 guesses right 90% of the time?) | Logs | open |
| Unit snap window (calibrate.py SNAP; design doc §4) | 15% | Replay with streams rescaled by known factors (x10, x57.3, x1000) at windows of 5-25%; count right unit guesses against false snaps | Logs | open |
| Disguise factor set (replay.py disguise) | x0.1, 10, 57.3, 1/57.3, 2.54, 1000; half sign-flipped; 5-row delay | Not a DOE: a test design. Widen it with axis swaps and offsets (report, new test 5) | Logs | chosen |
| Motion gate before a motion-type label may be accepted (classify.py MOTION_GATE_S; threshold_doe.py GATES) | 60 s of wheel motion | Chosen on dev from 0 / 5 / 60 / 120 / 160 / 240 s (after the rule fixes; before them it was 240 s). Pieces at 30, 45 and 60 s would show how sharp the edge is; recheck on real calibration-battery logs | Logs | partly done |
| Motion-response ramp from "weak" to "strong" (classify.py AOR_RAMP; range vs line) | 1.5-3.0 (was 2.0-6.0 on the all-window figure) | Re-anchored on dev after the moving-windows fix: line groups 0.9-1.7, zone groups 1.6-13.6. Sweep the low end 1.0-2.5 and the high end 2-6 on dev, check on held-out; score range against line labels. Dev and held-out overlap at 1.6, so a board-approach calibration move may matter more than the ramp | Logs | open |
| Minimum moving windows for the motion response (features.py MOVE_MIN_WINDOWS) | 5 windows of 1 s | Sweep 3-20 on the replay and the log pieces; count range/line labels right against motion response left blank | Logs | open |
| Short-log ramp (classify.py SHORT_RAMP; duration from "fully short" to "long") | 30-120 s | It caps every log under 120 s: the synthetic compact batteries got 34 of 37 right but only 9 accepted; without it 26-27 accepted but 1-2 wrong per robot. Try replacing duration with motion seconds or evidence counts (turn pairs, drive ends, legs); fit on dev pieces and synthetic batteries, check on held-out | Logs + Sim | open |
| Tilt test (classify.py TILT_R; changing side within a factor e of the gravity axis's noise floor; groups of 4+ without an IMU axis left out) | 0.90; e; 4 | Built on the synthetic battery only (no real log has a deliberate tilt). Sweep TILT_R 0.8-0.97 on synthetic batteries with different tilt sizes (5-30°) and speeds; then confirm on a real calibration log with hand tilts | Sim, then Board | open |
| Calibration battery moves and repeats (battery.py PLANS / battery_doe.py FULL: stand 10 + 5 s, square 12 x 15 cm, tilt 2 x 20°, lift 2 x 5 cm, line crossing 2 x 25 cm, spins 4 x 90°, drive 2 x 20 cm) | as listed (about 140 s) | Done on/off per move on the synthetic robot (report). Next: vary the repeats (square 6 / 12 / 18 legs, tilts 1 / 2 / 3) to find the shortest battery that keeps every label, then one real battery log per robot | Sim, then Board | partly done |
| Threshold experiment design (threshold_doe.py LEVELS 10-160 s doubling, PIECES 2 per case and level, contiguous pieces from random starts) | as listed | Not a DOE: a test design. Widen with non-contiguous pieces (several short stretches) and more pieces per level when the machine is free | Logs | chosen |
| Acceptance criterion for a rule (threshold_doe.py CRITERION) | wrong share at most 1%, 95% upper bound at most 3% | Not a DOE: a risk tolerance. How many wrong labels per 100 accepted is acceptable is Nick's call (question in the report) | - | Nick's call |
| Confidence cap (classify.py CONF_CAP: confidence <= 0.6 + 0.4 x best score) | (0.6, 0.4) | Sweep the slope 0.3-0.5 with the threshold experiment (logs only, about 20 min each); score wrong accepted and questions on dev pieces, check held-out; also check confidence calibration (labels at 0.9 right 90% of the time?) | Logs | open |
| Heading tie-break and the gravity bar (classify.py HEADING_TIE, GRAVITY_LON) | 0.10; 300 | HEADING_TIE: sweep 0.02-0.2 on the synthetic ablation plans where the yaw/pose heading swap remains, and on the replay. GRAVITY_LON reuses the existing gravity bar (300); sweep 100-1000 on pieces | Logs + Sim | open |
| Printed calibration parts (report spec: one-wheel bump 1.0 cm x 10 cm, ramp 1.0 cm over 8 cm with a 6 cm plateau, crossing speed 3 cm/s) | as listed | Synthetic: bump height 0.5 / 1.0 / 1.5 cm x speed 2 / 3 / 5 cm/s (tilt test r and labels accepted); then one real log on one robot with the printed bump | Sim, then Board | open |

## Already set by data

- **Line edge cue edge_thr 150** (e02.py SAFETY_SETTINGS): plywood reads 52-84 in every E02 session, the two real
  edge trips 241 and 573. The note says a DOE would not add much.
- **Side camera's half-robot plane 1.85 cm** (e02_sidecam.py SIDE_Z_CM): measured best of 0 / 1 / 1.85 / 2.5 / 3.7 /
  5 cm (build-reports/e02-oct24-sidecam.md, appendix A2).
- **Healthy noise per sensor** (e02_trust.py HEALTHY: camera 0.8 cm, wheels 0.6 cm, gyro 1.0°, front 5.0 cm): the
  2026-10-23 and 2026-10-24 sessions. Note the night analyses' caveat: the replays that check them use the same data.
- **Robot timing for the dance** (e02_dance.py REAL_OCT21): measured from 56 go and 58 turn legs, 2026-10-21.
- **Leg pause itself** (e02.py leg_pause_ms): now computed from each session's measured picture delay; only its bounds
  and margins are chosen (row above).

## A DOE was ruled out

- Sanity bounds: e02_explore.py ROBOT_RMS_MAX_PX 20 and FIT_RMS_MAX_PX 8; e02_map.py CORNER_SKIP_DEG 25 (a safety
  margin).
- Backups that trade data for Nick's time (e02.py): HELP_DROP_N 3, COND_FAIL_N 3, HEADING_LONG_ROOM_CM 30. The last
  one conflicts with the heading-drive DOE in tests-to-run (row above); it is kept as a row.
- Side camera: SIFT ratio 0.8, RANSAC 4 px, "same robot" 8 cm, 21 pictures per median. A sweep is enough.
- Reverse find's look (e02.h lookAhead): 150 ms settle, 5 frames ~70 ms apart, median. Set by the sensor's ~15
  frames a second, not by tuning; a DOE would not change it (its allowance, e02_find_see_cm, has a row above).
- Dock geometry: pad spacing 20 mm, pad height 10 mm, sense divider, cavity and channel sizes.
- Dock v5 geometry (build_dock_v5.py): plate 22 mm hinge to crown and 0.8 mm brass, slot width 10.4 mm (follows from
  the arms' play), base depth 36 mm (fits the strip), groove and screw positions: set by geometry, not a DOE.
- Dock v6 geometry (build_dock_v6.py): the springs' spacing (a 1.2 mm printed wall between the pockets, the smallest that prints),
  the pad 10 mm across (covers the arms' play), the wire channel at 31 mm and the wire groove under the prong's middle,
  the arms' ends flaring 5 mm: set by geometry, not a DOE.
- Charge levels refit every 5 min at most (e02.py LEVELS_EVERY_S): display freshness, not a DOE.
- The 1 cm equivalence margin (publication-plan.md): it should come from what a task needs, not a DOE.
- Display-only numbers (chat 2026-10-25): live view refresh, 5-minute chart window, last 30 values per trend line,
  2 s between trust-table rebuilds.

## Chosen, but no test named

These notes say "chosen" and nothing else, so they are not rows yet. Claude's suggestion is in brackets where a test
seems sensible.

- Border details: firmware e02.h BORDER_TOWARDS_CM 0.5, BORDER_MOSTLY 0.5, BORDER_UNLATCH_CM 3; e02.py
  BORDER_TOWARDS_CMS 2, BORDER_TOWARDS_FRAC 0.5, BORDER_JUMP_CMS 50 [the drive-at-the-line runs and a track replay
  would check them].
- Lost robot: e02.py LOST_DRIVE_S 1.5, LOST_START_S 1.0, LOST_FROZEN_S 3, LOST_FROZEN_CM 10, LOST_FROZEN_FRAC 0.25
  [same logs as the RECHECK_S count]; HOLD_RESTOP_S 2.
- Explore: EXPLORE_PROBE_CM 12, MAKE_WAY_EDGE_CM 20, EXPLORE_HOME_FRAC 0.35, EXPLORE_HOME_IN_CM 30,
  EXPLORE_EDGE_BACK_CM 9.5, EXPLORE_SECTORS 8; e02_explore.py MIN_PX_PER_CM 0.4, STILL_JUMP_PX 25, SWERVE_DEG 3,
  ROBOT_INLIERS_MIN 0.7, GYRO_RANGE 0.85-1.25, FIT_ROBOT_MAX_PX 12, SIDE_MIN_POINTS 2, SIDE_NEAR_CM 6,
  SIDE_KIND_SHARE 0.66.
- Learning the map: BOARD_LEARN_MIN_N 3, BOARD_LEARN_STEP_CM 0.5, EDGE_LEARN_MIN_N 3.
- Matching: MARK_NEAR_CM 15, KC_WP_MAX_CM 15, RELOCK_STILL_N 12; tracker numbers in build-reports/e02-oct22-build.md
  and e02-fix-build.md (gate growth 0.6 s, 30 cm/s, robot widths, still 12 pictures, 14 grey levels; checked on
  footage) and the runner's back-off 8 cm, move-in 14 / 25 cm, sweep 30° / 70 cm / 8 cm (not checked).
- Fits and planning: e02_fit.py MAX_SE drift 0.3; e02_bricks.py MARGIN_CM 3.5 (a fallback until measured runs replace
  it) and GAP_K 1.0; e02_edgefind.py SIDESTEP_CM 25 / SIDESTEP_MIN_CM 10 (from geometry).
- Test design and reporting: e02_degrade.py HOLD_S 2; load blocks 300 s; SHADOW_EVERY_S 5; HEALTH_WINDOW_S 30;
  e02_analysis contact thresholds 1.5 cm/s and 0.35 cm [a sensitivity check in the analysis]; the 1.5 x IQR rule (the
  publication plan reports with and without it); the front-suggestion rule of 3 x spread; night-analysis cuts (15 cm
  near a board, 5 rpm driving).
- Nick's time: WEB_ASK_TIMEOUT_S 15 min, CAMERA_WAIT_S 300 s, the engineer loop's countdown defaults.
- Not yet tried: the second camera's mount (1.2-1.5 m, 35-45°).

**To measure, not a DOE:** the back attachment's axle-to-face distance (params.h e02_back_face_cm 6.10, from the
v3 design: tape-measure the printed part, as bump_ahead_cm was); the simulator's GUESSES (tools/sim/config.py: motion, bottom-zone geometry, seams, dock
contact; the E0c edge rows measure them), params.h tof_recess_cm and tof_lag_ms, the spring pins' ASSUMED sizes and
force (gauge print, bench tests 4-5), and the fusion-era assumptions in
[ACCURACY-CHECKS.md](experiments/ACCURACY-CHECKS.md).
