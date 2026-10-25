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
none done. **Added 2026-10-25 (reverse find build):** 3 rows (2 Sim, 1 Board), below. **Added 2026-10-25 (dock v5):** 6 rows in Docking and charging (2 Sim or Logs + Sim, 2 Bench/Board, 2 waiting on bench test 1); the charge-levels row is in progress. **Added 2026-10-25 (heading diagnosis, [write-up](build-reports/e02-oct25-heading.md)):** 1 Logs row (heading push). **Update 2026-10-25 (logs run, [write-up](build-reports/e02-oct25-logs-doe.md)):** 7 logs rows done or done
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
| Camera virtual border, home (e02.py HOME_BORDER_CM; chat 2026-10-25) | 8 cm | Same runs as above; also the home sessions' border_shadow.csv counts | Board | open |
| Robots' own border (e02.py FW_BORDER_CM / HOME_FW_BORDER_CM; chat 2026-10-25) | 10 / 6 cm | Same runs, with the camera covered for the last stretch (the robot's own count alone) | Board | open |
| Recheck before "lost" (e02.py RECHECK_S; build-reports/e02-oct24-right-session.md; chat 2026-10-25) | 3.5 s | Count rescues against holds over the next 3 sessions; replay sessions' tracks with 1-6 s | Logs + Sim | needs more data (2026-10-25 replay of 677 go legs: the old sessions refound within a median 1.9 s, so 41 of 46 disagreements cannot be followed to 3.5 s; count rescues in the next sessions; [write-up](build-reports/e02-oct25-logs-doe.md) §6) |
| Bad-camera border shrink (e02.py BORDER_EXTRA_SIGMA / BORDER_EXTRA_MAX_CM) | 2 sd / 10 cm | Simulator grid at camera noise 1, 2, 3 cm: holds and off-board count against runs completed | Sim | in progress (grid). Partial, chat 2026-10-25: at 4 cm extra noise, holds 17 → 7, but no scored runs (4 cm treated as beyond the system for now); 2 cm rerun running |
| Park spots moved in (e02.py PARK_OPEN_MAX_CM; chat 2026-10-25) | 22 cm | Simulator grid, noisy camera: parking holds and near-misses against the route clearance | Sim | in progress (grid) |
| Bottom-zone edge cue (e02.py SAFETY_SETTINGS edge_bot_cm; params.h default 13 is a placeholder) | 11 cm | Once the crawl runs (DOE-explore-crawl) log which cue fired and its reading: limit against false stops on plywood and distance past the edge; tests-to-run item 9 | Board (same runs as the crawl DOE) | open |

## Camera and tracking

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Detector check rate (e02_camera.py CHECK_S; Nick 2026-10-22: "start at 2 s") | 2 s | [DOE-tracking.md](experiments/DOE-tracking.md) DOE 1: 1 / 2 / 4 s on replayed footage; wrong-position time, time to catch, laptop cost | Logs (replay) + hand-checked truth, ~1 min per footage minute | needs hand-checked truth (a position checked about once a second; the clips have 3-6; [write-up](build-reports/e02-oct25-logs-doe.md) §12) |
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

## Calibration and fits

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Confidence pause (e02.py CONF_PAUSE_CM; Nick 2026-10-23: "3 cm and DoE to finalize") | 3 cm | [DOE-confidence.md](experiments/DOE-confidence.md): 1.5 / 3 / 6 / off, about 72 runs in blocks of robot x route; end error against pause time | Board, ~1.5 h | open |
| Uncertainty model (e02.py UNC_BASE_CM / UNC_PER_CM / UNC_DEG / UNC_DEG_PER_DEG) | 0.5 cm, 1%, 0.5°, 0.5% | DOE-confidence.md "before running it": regress confidence.csv correction_cm on distance and turn since the last fix. Dry run 2026-10-24: model said ~4 cm where the simulator's truth was 1.6 cm | Logs (confidence.csv), then Sim | done, logs part (2026-10-25): too confident (59% of 673 corrections inside its 95% range); refit: turning term 0.014 per degree (was 0.005), the rest unchanged; 17% of chains end with the heading 30°+ wrong, not covered. Sim next; [write-up](build-reports/e02-oct25-logs-doe.md) §1. Heading part traced (2026-10-25): 100 of the 112 come from fixes writing an unmeasured runner heading (±90 placeholder, U run's end heading echoed, pre-turn heading after a refind), not the gyro; without those chains the turning term is 0.0126 per degree; fixes proposed, [write-up](build-reports/e02-oct25-heading.md) |
| Gyro drift band (e02_fit.py DRIFT_BAND_DPS; build-plans/e02-oct22-build.md) | 0.05 °/s | Offline refit of the 2026-10-24 evening stretches at 0.02 / 0.05 / 0.1; predicted against measured end errors | Logs | done (2026-10-25): band 0 (drift held at the measured value) predicts best, 0.12-0.15 cm better than 0.05 (p < 0.001, 49 runs); recommended 0; [write-up](build-reports/e02-oct25-logs-doe.md) §2 |
| Tracker jump cut (e02_fit.py JUMP_CM; analyses_oct24/night_selfcal.py; build-reports/e02-oct24-night-analyses.md) | 3 cm | Already swept 2 / 3 / 4 cm (2 cm drops 20-35% of counted distance); still to do: score each cut by how well its wheel scales predict C-run end errors | Logs | done (2026-10-25): 2 / 3 / 4 cm give the same predicted end errors (≤ 0.09 cm apart); keep 3; [write-up](build-reports/e02-oct25-logs-doe.md) §3 |
| Wheel fit accepted (e02_fit.py MAX_RMS_CM; e02.py --fit-max-rms-cm) | 1.5 cm default, 2.5 cm for sessions (Nick 2026-10-24) | Replay sessions' fits at 1.5 / 2 / 2.5 / 3 cm: fits refused against C-run end error with the fit used | Logs | done (2026-10-25): no effect on predicted errors (every fit passes 1.5 cm with the jump cut); the fixed prior predicts as well as the session fit; back to 1.5 cm; [write-up](build-reports/e02-oct25-logs-doe.md) §4 |
| Camera noise per stretch (e02_fit.py SIGMA_DEG / SIGMA_CM, "assumed") | 1.5° / 0.5 cm | Two cameras timing the same stretch give a measured value (build-plans/second-camera-plan.md) | Board + second camera | open (waits for the second camera) |
| Camera error map smoothing (e02_camerr.py BANDWIDTH_CM; also how many marks and where, build-plans/generality-audit.md) | 25 cm, 7 marks | Leave-one-mark-out error at 15 / 25 / 40 cm, once a session takes the tape marks | Board (tape marks, Nick ~15 min), then Logs | open |
| Camera error map weights (e02_camerr.py MARK_W / NOISE_SD_CM / NO_DATA_SD_CM; "all chosen, DOE notes beside them") | 4 / 0.7 cm / 3 cm | Same tape-mark data: a held-out mark's error against the map's predicted sd, at each setting | Logs (after the tape marks) | open |
| Camera heading sd (e02.py KC_HEAD_SD_DEG; its note "DOE with tape marks" sits displaced on the WEB_ASK_TIMEOUT_S line) | 3° | Camera heading from a straight stretch's end against two tape marks' bearing | Board (tape marks) | open |
| Explore calibration accepted (e02.py MARK_ACCEPT_CM) | 2.5 cm | Explore's mark error against the runs' errors over sessions with --marks (2026-10-24 dry run: 4.3 cm passed explore's own checks, 29 robot losses followed) | Board (sessions with --marks) + Logs | needs more data (no real session has finished explore with --marks; [write-up](build-reports/e02-oct25-logs-doe.md) §5) |
| Heading before a find (e02.py FIND_HEAD_AGREE_DEG) | 5° | The find's heading correction against this limit, from FIND records (Center's finds 13-24 cm off came from a heading up to 19° wrong) | Logs | done, weak (2026-10-25, 39 finds): at 8-10° no find starts over 10° wrong (one at 5°); 10 suggested; [write-up](build-reports/e02-oct25-logs-doe.md) §8 |
| Heading push after a measured go leg (proposed fix 3 in [e02-oct25-heading.md](build-reports/e02-oct25-heading.md); not in the code yet) | 8° | The spread of a 10-15 cm go leg's camera direction against the robot's own heading on legs where both are right (heading_legs.csv); the threshold just above that noise | Logs | open |
| Front sensor offset and tolerance (e02.py FRONT_OFFSET_CM / FRONT_TOL_CM; tests-to-run item 10) | 0 / 8 cm | Set from front_checks.csv (e02_analysis prints suggested values). Data is in hand: four sessions give a spread of 4.0-4.6 cm and medians -3 to +1 (e02_trust.py HEALTHY), but not applied | Logs | open (data in hand) |

## Sensor trust

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Trust flags (e02_trust.py DEGRADED_K / RECOVERED_K / STANDS_OUT) | 2 / 1.5 / 1.5 | Swap test: 20 injected faults per sensor and level (simulator), then real; detection rate and false alarms per hour | Sim, then Board | open |
| Still-camera healthy scatter (e02_trust.py CAMERA_STILL_HEALTHY; e02.py STILL_WINDOW_S) | 0.4 cm over 3 s | A robot standing still at 7 places for 30 s each | Board, ~10 min | open |
| Evidence before a sensor's k is used (e02_trust.py MIN_N; night-analyses test 2) | camera 4, wheels 3, gyro 3, front 6 | Replay the healthy 2026-10-23 sessions and count false flags at 3 / 5 / 8 (it flags Right's wheels on the morning today) | Logs | done, healthy half (2026-10-25): wheels/gyro 3 give 3 false flags in 4.8 h, 5 give none; 5 suggested after the swap test; [write-up](build-reports/e02-oct25-logs-doe.md) §9 |
| Other trust limits (e02_trust.py DECAY_PER_SESSION / GATE_K / GATE_MAX_CM / KC_GATE_SIGMA). The module says "DOE notes beside them"; none is written. The test here is Claude's suggestion | 0.5 / 1.5 / 3 cm / 3 sd | Score them in the same runs as the swap test plus the healthy replay: false refusals against faults caught | Sim + Logs | healthy half done (2026-10-25): GATE_K 1.2 refuses 2 good fits, 1.5-2 none; GATE_MAX_CM never reached; decay and KC_GATE_SIGMA 3 fine; keep all; faults-caught half needs the swap test; [write-up](build-reports/e02-oct25-logs-doe.md) §10 |

## Explore and edges

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Crawl speed at an unknown edge (e02.py EXPLORE_CRAWL_CMS; Nick 2026-10-24) | 4 cm/s | [DOE-explore-crawl.md](experiments/DOE-explore-crawl.md): 2 / 4 / 6 / 8 cm/s x 0° / 30° / 60°, 180 approaches; lowest-5% margin above 1 cm | Board, ~45 min (foam strip) | open |
| Bottom zone's reach ahead (e02_map.py BOTTOM_AHEAD_CM; EXPLORE_EDGE_BACK_CM 9.5 rests on it; build-plans/e02-oct22-build.md "to be measured") | 9.0 cm | The crawl runs' stops: camera-measured front wheel place against the plywood's end, bottom-zone stops only; explore's refine_edges estimate each session as a check | Board (same runs) + Logs | open |
| Open edge learning caps (e02.py EDGE_LEARN_IN_CM / EDGE_LEARN_OUT_CM) | 8 / 3 cm | Edge stops at tape-measured distances from the plywood's end (the crawl runs) | Board (same runs) | open |
| Heading drive length (e02.py heading drive 10 / 15 cm, HEADING_LONG_ROOM_CM 30; tests-to-run "Heading drive length") | 10 cm, 15 cm with 30 cm room | 10 / 15 / 20 cm x middle / beside a board / near an open edge, ~8 repeats per cell; heading error, camera losses, distance toward an edge | Board, ~30 min | open |

## Docking and charging

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Charge levels (build-plans/self-charging-dock.md; tools/dock/charge_levels.py) | go from its parts (49-54 % on the logs' rates), leave = the knee (80 % PLACEHOLDER), top-up L - 20 | Method in place (2026-10-25): drain learned per robot from the logs (battery_rates.py), go = error + job + drive + queue + floor. Remaining: bench test 1's charge curve, then `charge_policy_sim.py --compare` | Bench test 1 + Sim | in progress (drain part done on 6 sessions; charge curve open) |
| Drain margin (charge_levels.py P k_se; v5) | rate + 2 standard errors | Simulator: each robot's drain drawn from its learned uncertainty, k = 1 / 2 / 3, 7 days x 5 seeds: flat batteries against uptime | Sim | open |
| Battery learner thresholds (battery_rates.py SESSION_MIN_HOURS, SHIFT_MIN / SHIFT_SE, RISE_SPLIT, JOIN_GAP_MIN, MIN_HOURS; v5) | 0.75 / 0.25 / 0.75 h; 3 %/h and 3 se; 3 %; 45 min; 0.1 h | Replay the sessions on disk with a drain change injected (+5 %/h from mid-session) and without: how soon it is flagged against false flags, at 0.25 / 0.5 / 1 h | Logs + Sim | open |
| Rest policy (charge_policy_sim.py --rest K; Nick 2026-10-25: rest robots off the dock before a second dock) | none chosen | `--compare` once bench test 1 and the sleep test give the charge rate and the sleep drain: the sleep drain decides whether resting helps (it fails below about 25 %/h at 3 %/h asleep) | Bench test 1 + sleep test, then Sim | open |
| Contact force: pad position, spring rate, crown height (build_dock_v5.py PAD_X_MM, SPRING_K_N_MM, CROWN_FREE_Z_MM) | 8 mm behind the axle, 0.2 N/mm, 9.0 mm | 2 spring rates (penny test: about 20 and 40 g at 1.5 mm) x 2 crown heights (one paper shim) x 20 back-ins: contact resistance, contact within 10 s, caster pinch | Bench + Nick (springs) | open |
| U arms and mouth (build_dock_v5.py ARM_IN_HALF_W_MM, FLARE_L_MM / FLARE_MM, ARM_END_X_MM) | +-1.5 mm play, 10 mm chamfer opening 7 mm, arms to 105 mm | Bench test 6's back-ins from staging offsets of 0 / 5 / 10 mm and 0 / 5 deg: count clean dockings; widen the chamfer only if 10 mm misses | Board + Nick | open |
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

## Reverse find (backing into the boards, firmware 2026-10-25b)

[Build report](build-reports/e02-oct25-reverse-find.md). All in firmware/home_lab/params.h.

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Look allowance (params.h e02_find_see_cm) | 20 cm | At the staging point, facing each board: 20 looks per robot with the board in, 5 with it lifted out (the robot stops before backing); put the limit between the two spreads, nearer the "in" side. The simulator only has a flat ray model, so this needs the board | Board, ~15 min (part of the first backing-in session) | open |
| Backing cap (params.h e02_find_back_margin_cm, e02_find_keep_cm) | 3 cm + 20% of travel; axle 2 cm from the line | Simulator grid: margin 2 / 3 / 5 x keep 2 / 4 / 5.5 x gyro corrected / uncorrected x camera fix noise 0.3 / 1 / 2 cm, 20 finds each: finds done, false "no board behind", and (board removed, look off) how near the axle and the rear caster come to the line. 5.5 keeps the caster on a plywood edge at the line; whether that matters depends on where the boards stand (question to Nick) | Sim | open |
| Nose-first approach before turning round (params.h e02_find_turn_cm) | 30 cm | Simulator: starts 30-90 cm from a board with no staging point, turn_cm 12 / 20 / 30 / 50: reset error, failures, time | Sim | open |

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
