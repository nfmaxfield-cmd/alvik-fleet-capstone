# School demo engine and Mission Control (2026-10-27, 00:06-02:30 build)

## Premise

Monday 2026-11-02: three Alviks on a school table with no boards, a camera at an unknown spot, the laptop running
everything. Routine: Show, then Center mows while Left and Right do Chicken x2, Follow the leader and Roam, then all
park in a row; Nick drops a brick mid-run and the robots react. Brief: [mission-control.md](../build-plans/mission-control.md)
(Nick's 00:08 answers: first safe re-plan at once; Chicken 15 / 8 cm; nothing locked in the operator view; 00:16: the
engineer view shows how the robots mapped the table). Simulator, tests and logs only; the Pi and tonight's session
were not touched.

## Data

**1. Re-plans** (`tools/fleet/bench_replan.py`, demo map, 2 obstacles x 10 moments, old and new on this machine;
[results](../results/replan-bench-2026-10-27.json)):

| | Old (search the whole budget) | New (first safe plan, then search in the background) |
|---|---|---|
| Re-plan wall time, median / worst | 0.95 / 1.97 s | **0.15** / 2.61 s |
| First safe plan, median / worst | 0.012 / 1.81 s | 0.015 / 0.89 s |
| New steps may start after | 2.0 s | **0.5 s** |
| Re-plans arriving after that (robots held) | 0 of 20 | 6 of 20 (worst 2.1 s; 4 are re-plans that strand a robot in both) |
| Sum of finish times (20 plans) | 650.9 s | 638.4 s |
| Closest two robots, any plan | 6.2 cm | 6.2 cm |

In the dry runs the background search swapped in a shorter plan when it found one ("done at 78 s instead of 84 s,
kept"; Monday M2: 198 s instead of 347 s). On Monday's long plans a first safe re-plan took **0.6-4.8 s** (8 events),
not 0.15 s: robots stood still that long.

**2-5. Warm-up and brick drop** (dry runs, school table assumed 152 x 76 cm):

| Run | Real table (simulator) | What the warm-up found |
|---|---|---|
| W3 | left open, right barrier, bottom barrier 0-70, top open, a box 8 x 6 cm | all four sides right; bottom barrier 0-77 cm; the box proposed as a no-go zone; 15 pictures (90 s) |
| W4 | left barrier, right open, bottom open, top barrier 60-152, a box | all four right; top barrier 75-152; the box proposed |
| Page | as W3, run from Mission Control | sides right; 2 top-side feels never ran (Center bumped the box, those feels were skipped: shown to Nick as pictures) |

- Open edges are placed **3.1-3.6 cm inside** the true edge (the bottom-zone cue's assumed 8 cm reach; the simulator
  trips at about 11 cm). Barriers within 0.4 cm.
- Wheel scale against the camera: 0.998-1.020 (stored 1.0, simulator truth 0.99-1.01). Gyro: 0.96-1.00; one 0.96
  flagged in W4 (simulator truth not checked).
- During feels the robots' front corners reach up to 1.2 cm past an open edge (W4, by design: the edge guard stops the
  line sensors at the edge); nobody hung.
- Brick drop through the simulated camera (`--brick-camera`: a hand puts it down, the what-changed watch reports it):
  **confirmed 1.1-1.3 s after it landed** in 6 runs, 0 false objects in about 4,000 checks, the hand never reported (5 of the 6 drops were at the same spot; the simulated camera draws neither the felt box nor shadows).

**6. Monday's routine** (dry runs with a camera-seen brick at 118 s):

| Run | Map | Took (planned) | Closest robots | Closest to brick / open edge | Notes |
|---|---|---|---|---|---|
| M1 | all sides open | stopped at 121 s | 11.3 cm | 4.5 / 4.4 cm | **problem:** Center 0.12 cm past a lane end, boxed in, no re-plan: fixed (lanes 1 cm inside, 0.5 cm escape tolerance) |
| M2 | all sides open | **197 s (230)** | 10.7 cm | 4.0 / 4.1 cm | every lane and activity done; brick re-plan 3.0 s, swap kept |
| Page | after the warm-up | 206 s (223) | 10.6 cm | - / 4.4 cm | **problem:** 2 false front-sensor stops at a barrier side: fixed (barrier sides expected) |
| M3 | after the warm-up | 205 s (223) | 8.2 cm | 3.0 / 4.4 cm | **problem:** 1 false stop at the felt box: fixed (felt objects expected) |
| M4 | after the warm-up | **229 s (223)** | 11.3 cm | 4.1 / 4.2 cm | no false stops; brick re-plan 0.63 s; no contact, nobody hung |

The plan's own numbers: closest 10.6 cm (never under 8), the Show's 37 moves on their beats except 3 (worst 1.4 s
late), Show 54 s, Center's pass about 139 s after it.

## What changed

| Item | Change | Where |
|---|---|---|
| 1 | Re-plans stop at the first safe plan; robots carry on only 0.5 s ahead; the search for a shorter plan starts at once and is swapped in only if everyone finishes sooner. Old behaviour: `replan_first_safe false`. | planner.py, run_plan.py, bench_replan.py |
| 2 | The workspace map: table rectangle, each side open / barrier / partial (barrier stretches) / unknown (= open), named spots A, B, C, no-go zones (proposed / confirmed; felt ones "solid"), `wait_s`, `until_s`. Open parts keep 3 cm more room. The runner's Board takes side kinds (unchanged without them). | fleetmap.py, workspace.py, e02_plan.py |
| 3 | **Firmware 2026-10-27a**: four-sided border (`border.h`, `bd_*`, off unless `bd_on 1`) in e02 runs and now in the step follower (a border stop ends the step; run_plan re-plans); second WiFi network (`WIFI_SSID2` / `WIFI_PASSWORD2`, placeholders, commented out); tried first-then-second, alternating. **Host-compiled with the stand-in headers only (both with and without the second network) and an 11-case host test; not compiled with the Arduino toolchain.** run_plan sends the border (`--border-cm 6`). The runner accepts 27a. | firmware/home_lab, tools/fwcheck, run_plan.py, e02.py lists |
| 4 | Warm-up: corner pauses (2 s), side feels every 40 cm (3 cm/s creep past the marked line), answers per side, partial stretches, proposed zones (from feels and from any front-sensor stop on the way), up to 15 pictures for Nick (zones, tracker losses, corners, unclear feels first), wheel / gyro against the camera, apply to the map. Engineer view: mapping picture. | warmup.py, mapview.py, run_plan.py (labelled steps, cam_track.csv, tracker_lost) |
| 5 | Brick watch: floor memory (e02_bricks / MotionTracker), robots masked, a patch touching the picture's edge or the table's outline = a hand (ignored, nothing confirmed meanwhile), 3 still checks at 4 Hz = an obstacle polygon to run_plan. Simulator table camera with hand and brick. Not yet wired to the real camera's tracker (track_live.py). | brickwatch.py, tools/sim/table_cam.py, fleet_sim.py |
| 6 | Run modes as planner tasks: instruction lists, lawnmower (lane ends slide along their lane off a brick or zone), Chicken, Follow the leader, Roam, Show (beat targets), Monday as one mission. Planner: right of way (`priority_first`), a mission clock through re-plans, pause steps. run_plan: Pause / Resume. | missions.py, planner.py, run_plan.py |
| P2 | Mission Control: six screens (Connect, Workspace, Warm-up, Mission, Run, Review), Details switch, preview with time slider, finish time and closest approach; every setting under "More" on its screen (nothing locked); simulator truth settings for dry runs. | mission_control.py / .html |

Decisions made on the way (Nick's design calls, flagged):
- **Monday's split:** Center mows lanes across the short side, sweeping the table end to end; Left and Right play in
  the part it is not in. Order **Chicken x2, Roam, Follow the leader** (Follow moved last: by then the mower has
  cleared an end of the table). Roam and Follow **46 s each, not 60**, so they end with Center. The robots **start on
  the Show's line-up spots, Center at the left end** (from the warm-up's park spots no plan could be found).
- Show: 5 figures, each robot in its own third of the table (54 s); crossings made the planner miss beats by 45 s.
- Chicken: both swerve to the same side when another robot stands on one side. Follow the leader: an open winding
  path, not a loop (a loop let the leader come round onto the follower).

## Conclusion

- The engine runs Nick's whole routine in the simulator end to end, with a camera-seen brick: every lane and activity
  done, robots never under 8 cm (closest 10.6-11.3 cm in the final runs), no contact, nobody over an edge.
- The warm-up got every side right in 3 layouts, including partial barriers, and proposed the unknown box as a no-go
  zone. Open edges come out about 3 cm too far in (a fixed cue offset, safe side).
- First safe re-plans are quick on short plans (0.15 s median) but 0.6-4.8 s on Monday's long plans.
- Each dry-run problem found was fixed before the last run (lane-end box-in, barrier sides and felt objects missing
  from the front check, JSON with Infinity on the page, warm-up feels lost to other robots' re-plans).

## What's next

1. **Nick flashes 2026-10-27a** to Left, Center, Right (Arduino IDE, `firmware/home_lab/home_lab.ino`) after
   filling `WIFI_SSID2` / `WIFI_PASSWORD2` in his own `secrets.h`.
2. The school table's size and the camera's floor calibration (the map is 152 x 76 cm by assumption).
3. Wire the brick watch into `tools/live/track_live.py` (real camera pictures); test with a hand on the home board.
4. Faster re-plans on long missions: plan only the next 30-60 s in detail (a horizon).
5. A home-board rehearsal: warm-up, then Monday at reduced size.

## Sweep 1: tests that may now be needed

- Firmware 27a on the bench: the four-sided border in a follow run (a robot driven towards an open side stops; along it
  does not), the second network (home off, school on), the border off with `bd_on 0`.
- Warm-up feels on real plywood edges and a real barrier (a board laid on its side): stop distance past the edge at
  3 cm/s, front stops at a barrier, and the bottom-zone cue's real reach (the 3 cm offset).
- The brick watch on recorded overhead video with real hands (bright room, shadows, the robots' own shadows).
- Monday on the home board, scaled to its size, with a real brick.
- The Pause / Resume path on real robots (simulator untested).
- Mission Control on the laptop's browser (Windows), and with the real robots' run_plan start-up (firmware check,
  calibration prompts) behind the page.
- `bench_replan.py` on a Monday-sized plan (it uses the 65 s demo map).

## Sweep 2: possible inaccuracies

- **The school table is assumed 152 x 76 cm**; all of Monday's fitting (Roam / Follow lengths, Chicken run, park row)
  depends on it.
- The simulator's edge cues, hang model and camera are its own guesses; the 3 cm open-edge offset may be the
  simulator's, not the robots'.
- The simulated brick watch sees a clean, evenly lit table; the real one has a bright room and shadows.
- Re-plan timings are from this 2-core cloud machine, often with a dry run beside them; the laptop's will differ.
- The Show's beat timing is the plan's; the robots' real lateness is not measured (no music Monday).
- Wheel / gyro check uses camera poses 0.15 s from the step's start / end; the gyro truth was not compared.
- Chosen numbers (each with a DOE note in the code and a row in doe-backlog.md, or "no DOE" with the reason):
  replan_first_budget_s 0.5 s, OPEN_MARGIN_CM 3, BORDER_CM 6, probe_cms 3, PROBE_SPACING_CM 40, BARRIER_NEAR_CM 6,
  SCALE_FLAG 3 %, CHECK_HZ 4 / CONFIRM_CHECKS 3, ROBOT_MASK_SCALE 1.6, LR_SLACK_S 30, BAND_PAD_S 5, MOWER_CLEAR_CM 30;
  no DOE: CORNER_IN_CM 13, PROBE_START_CM 14, CORNER_PAUSE_S 2, OBJECT_CM 8, WHEEL_MIN_CM 10, TRACK_LOST_S 1,
  STILL_CM 3, GROW_CM 1, chicken_run_cm 45, chicken_aside_cm 20, CHICKEN_MIN_RUN_CM 10, roam_hop_cm 30,
  park_gap_cm 24, FOLLOW_STEP_CM 25, ESCAPE_TOL_CM 0.5, border.h half width 5 cm, PREVIEW_STEP_S 0.25.

## Appendix A. Dry runs (session folders in the build's scratch space)

`run_plan.py --dry-run --yes` with `--send` the edge guard (`safe_edge 3`, as e02.py), plus:

| Run | Map / extra | Result |
|---|---|---|
| R1 | demo map, `--brick auto@15` | first-safe re-plans; improvement kept (78 instead of 84 s) |
| S1, S2 | school map; S2 `--border-cm 15` | S2: 3 border stops in the follower, re-planned, finished |
| W1-W4 | warm-up, `--border-cm 0`, `--true-sides`, `--true-object`, `--cam-gap` | W1-W2 lost feels to other robots' re-plans (fixed: halted feels retried, labels kept); W3, W4 right |
| B1 | school map, `--brick auto@8 --brick-camera` | brick confirmed 1.1 s after landing; finished |
| M1-M4 | Monday, `--brick 100,40,90@118 --brick-camera` (M1 95,35) | above |
| Page | warm-up and Monday from Mission Control (Playwright) | above; the screenshots |

Tests: 31 in `tools/fleet/tests/test_school_engine.py`. Whole suite after the last change: fleet, experiment, pi 633
passed, 20 skipped (9.1 min); simulator tests 124 passed. (`test_app_endpoints` had failed whenever the fleet tests ran
with the experiment tests, before this build too: a module-name clash with tools/label/app.py, fixed.)

## Appendix B. Screenshots (docs/screenshots/2026-10-27/)

Mission Control `mc-1` to `mc-6` (each screen, operator and Details views), the warm-up mapping view
(`warmup-mapping-view-w4.png`, `mc-3-warmup-results-details.png`), the brick drop through the camera
(`brick-drop-camera-hand-then-brick.png`), and one problem (`mc-problem-warmup-status-stuck-infinity-json.png`).
