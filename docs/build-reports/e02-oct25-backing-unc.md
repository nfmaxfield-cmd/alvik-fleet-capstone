# Build report: backing in everywhere, learned uncertainty, find recovery, heading fixes (2026-10-25)

**Premise.** Nick, 2026-10-25 10:39 and 11:05, and the heading diagnosis ([e02-oct25-heading.md](e02-oct25-heading.md)):
- A. "Let's switch all bumping activity to backing in."
- B. The boards stand on the cement, pressed against the side of the plywood (so a board's line is the plywood's
  edge), 6-8 inches tall. Keep or adjust the backing cap with that fact.
- C. "No board seen" / "no board behind": the runner works through its own recovery first; "contacting the user
  should be a last resort". When it does ask, plain words: what happened, which board, where the robot is, what to
  check, and what each answer does.
- D. The uncertainty model is learned every session from the robots' own camera corrections, without the
  simulator ("accuracy and autonomy are the goals"), starting from the logs refit (0.014 per degree turned).
- Heading fixes 1-4 from the heading diagnosis. The uncertainty learner must not learn from stretches that start
  from an unmeasured heading.

## What changed

| Item | Before | Now |
|---|---|---|
| A. Fleet board visits (tools/fleet) | Nose first: an approach on the front distance sensor to 1 cm, dwell, reverse out | With the back attachment (map `robot.back_bumper_cm`): turn away at the pre-dock, back to 3 cm before the board, creep back until a back switch clicks (follow step P), forward 1 cm, dwell, drive out **forwards**. The plan says `switches_back`; run_plan.py sends `sw_on 1`, `sw_back 1`, `e02_back_face_cm` |
| A. Fleet home docks | Mouth pose and docked pose facing into the dock | With the back attachment: facing out (back to the mouth); step K's heading = the map's + 180 (what firmware 25b expects) |
| A. Footprint | Nose bumper only | `Body` also takes a back attachment (longer behind, at least the bracket's width) |
| A. Simulators | Boards not solid in the fleet simulator; the kinematic simulator had no touch | The fleet simulator makes boards solid for the switches with a back attachment and logs backing-in dockings; the kinematic simulator models the touch and a board taken away |
| B. Backing cap | Comment assumed a board's face could be inside the edge | The fact is in the comments (e02.h, fw_e02.py, e02.py). `e02_find_keep_cm` **stays 2 cm** (below) |
| C. Find failures | Counted as a failed find (the run redone later) | A recovery ladder, then a hold with plain words (below). The live page shows the hold's detail and relabels its buttons for a find |
| D. Uncertainty | Four fixed numbers (turning 0.005 per degree) | `e02_unc.py` learns them per robot every session; starts from the logs refit (turning **0.014**); used for the confidence pauses and KC's weights; `--no-unc-learn` keeps the fixed ones |
| Heading fixes | Fixes wrote the runner's heading, measured or not | 1: a fix carries the heading only if measured, else "-" (the robot keeps its gyro heading). 2: the ±90° placeholder only for a robot with no pose at all. 3: after a go leg that measured the heading, a robot whose own heading is more than 8° off gets it at once (`heading_push`). 4: the mapping ray and the bricks crossing pass only measured headings |

**What deliberately stays forward, and why.**
- **The edge find** (e02_edgefind.py): it touches boundaries whose kind it does not know (a board or an open edge),
  and the open edges are found by the front line sensors (the edge guard). Backing towards a possible open edge is
  exactly the danger. It also never touches a board: its "board" stop is the front distance sensor's stop, 7 cm out.
- **Explore's rays and probes, and the mapping run** (e02_explore.py, e02_map.py): the same; front stop or edge
  guard, no contact.
- **The front-sensor board correction** (when pictures are late): a reading, no contact.
- **Turning on the spot, transit legs and the find's staging drive**: they look where they go.
- **Leaving a board**: always forward (front sensor and edge guard on).
- Parking already backs in. The find already backs in with `--switches-back` (firmware 2026-10-25b). Nothing else
  in the runner touches a board on purpose.

**B. The cap, with the boards against the plywood.** The line is the plywood's edge. With the board there, the
back face (6.1 cm behind the axle) touches it while the rear caster (4.1 cm behind the axle) is still 2 cm inside the
edge: fine. With the board gone and the look fooled, keep 2 cm would let the caster reach 2.1 cm past the edge; a
cap that keeps it on (keep 4.6) stops many finds that have the board right there:

| Camera fix error (sd) | Keep 2 cm | Keep 3.5 cm | Keep 4.6 cm (caster stays on) |
|---|---|---|---|
| 0.8 cm, 2° | 2 of 30 stop | 4 of 30 | 8 of 30 |
| 1.5 cm, 4° | 4 of 30 | 9 of 30 | 21 of 30 |

<sub>Simulator, board in, gyro corrected, 30 finds per cell (10 seeds each layout, spots anywhere): "stop" = a false
"no board behind". Script: scratchpad keep_sweep.py, not kept in the repo.</sub>

Kept at 2: the look before backing (the front sensor must see the board) is what protects the caster, and at
keep 4.6 a quarter of finds would need recovery. The fleet's board visits have no look, so their cap is tighter:
`touch_over_cm` 1 cm, which keeps the caster about 1 cm on the plywood less the pose error (kinematic simulator,
board taken away: the back face went 1.4 cm past the line at most).

**C. The recovery ladder** (e02.py `find_recover`, each step logged in `find_recoveries` and said):
1. A fresh camera fix (its position; its heading only if measured).
2. Look: turn to face where the board should be (first driving forward out of the way if it is right against the
   board) and read the front sensor; re-measure that board's floor line in the camera picture.
3. If either says the board is there: the find again from another staging point (35 cm out, at least 25 cm along
   the board from where it was).
4. If both say it is gone: the map marks it missing (an open edge from now on; the virtual border keeps the robots
   off it; routes are rechecked) and the robot finds itself against the other boundaries (the edge find).
5. Only then the hold: what the runner tried and found, then what happened, which board, where the robot is (by
   the camera, cm from each board's line), what to check (is the board there, standing on the cement against the
   plywood; anything in the way; upright; for "no board behind" also the back switches), and the answers: carry on
   (the find again after a fresh fix), the board is gone (the edge find), leave it out, stop.

**D. The learned uncertainty** (`tools/experiment/e02_unc.py`). Every camera fix that replaces a pose the robot
drove to on its own, and every KC stop, is a correction: its size against the driving and turning since the last
fix. The camera's own noise at that spot is taken out. The fit is maximum likelihood with a share of outliers
allowed and a floor on each term. The distance terms (per cm, and the heading's fixed part) cannot be told apart, so
"per cm" stays at 1 % and the heading part is fitted. A term with no evidence in the session keeps its starting
value. Per robot: earlier sessions' corrections (logs/unc_learned.json), else every robot's, else the logs refit;
from 30 corrections on, this session's own, and a loud line when they differ beyond chance. Stretches are not
learned after a run (its driving is not counted) or after a fix that wrote an unmeasured heading. Logged:
`unc_obs.csv` (every correction with the model's sd then), `confidence.csv` (+ distance and turn), the state's
`unc_model`.

## Data

**Dry run** (simulator, `--dry-run --auto --dry-true-calib --classic-setup`) against the two latest comparable
dry runs from before (same flags, 2026-10-24, no degrade):

| | Before: 2026-10-24 21:14 (0.4 h) | Before: 2026-10-24 19:55 (0.5 h) | Now: 2026-10-25 11:32 (0.3 h) |
|---|---|---|---|
| Runs done | 21 of 21 | 21 of 21 | 19 of 21 (2 failed: Right, finds) |
| Confidence pauses (all took a fix) | 8 | 8 | 12 |
| Correction at a pause, median / max | 4.2 / 14.4 cm | 3.5 / 12.7 cm | 2.0 / 12.7 cm |
| End error, median: U / C / K / KC | 18.9 / 1.42 / 0.95 / 0.72 cm | 14.5 / 1.80 / 0.21 / 1.05 cm | 14.2 / 1.23 / 0.23 / 0.99 cm |
| Holds | 0 | 0 | 3 (virtual border, all Right) |
| Learned model after 50 cm and a half turn | (fixed: 2.2 cm) | (fixed: 2.2 cm) | 1.7 cm for all three (every term at its floor), from 31 / 33 / 47 corrections |
| Corrections inside the model's 2.45 sd (95 % if right) | - | - | 88 % (113: 100 fixes, 13 KC stops) |
| Heading pushes / fixes with an unmeasured heading | - | - | 22 / 0 |

<sub>End error: the camera's position at the route's end against its start (state `end_err_cm`), 3 runs per
calibrated condition, so a difference of a few mm is noise. Data: each session's e02_state.json, unc_obs.csv.</sub>

What it shows:
- **The learner works without the simulator's help**: each robot switched to its own model at its 30th
  correction and said so. All three went to the floors: the simulator's robots err less on turns than the real
  ones (the logs DOE found this too), so this run says how the learner behaves, not what the real numbers are.
- **Pauses**: more of them (12 against 8) with smaller corrections (2.0 cm median against 3.5-4.2): the 0.014
  starting value paused earlier until each robot's own model took over. End errors are unchanged within noise.
- **A new failure mode, found and fixed after this run**: Right's simulated gyro drifts 0.57 °/s uncorrected (this
  seed drew an unusually bad one; the others 0.01-0.03). With heading fix 1, nothing wrote the runner's heading over
  its gyro any more, so while parked its heading crept 156° in 8 minutes. The fix before its find then kept that
  (its last reported heading, from minutes before, looked close to the runner's), and the find drove towards the
  open edge: 2 failed runs, 3 border holds. Fix: before a find, a robot's own heading counts against the runner's
  only if it reported it in the last 15 s; otherwise the runner's measured heading goes with the fix
  (`find_heading_ok`, `FIND_OWN_FRESH_S`; test added). Parking and other moves already send a measured heading
  (heading fix 2). DRYRUN2

**Fleet dry run** (run_plan.py against fleet_sim.py, demo map, back attachment): all 6 board visits backed in and
touched (creep 4.1-4.2 cm, as planned: 3 cm lead + 1 cm gap), back face 1 cm from the board within 0.13 cm, sideways
within 0.22 cm, heading within 0.5°; nearest two robots 6.0 cm; done in 79 s against 86 s planned.

**Tests.** `pytest tools/experiment/tests tools/sim/tests`: TESTCOUNT. New: tests/test_e02_backing_unc.py (19:
the learner, its wiring, heading fixes 1-3 with a simulator robot that reproduces the placeholder (a 20 cm drive
ended 15+ cm from its belief with the old fix, under 1 cm with the new one), the hold's words, the four ladder
outcomes, the look's room check, the live page). `pytest tools/fleet/tests`: 36 passed, 2 skipped (the two 90 s dry
runs; both pass with FLEET_DRYRUN=1). New: 6 back-attachment tests. Firmware syntax check (tools/fwcheck): clean
(comment only).

## Conclusion

Every intentional touch of a board or dock now backs in when the back attachment is fitted; what stays forward is
what looks for an unknown boundary or never touches. The cap stays at 2 cm because the look is the caster's real
protection. A find that finds no board now sorts itself out before calling Nick. The uncertainty model needs no
simulator: every session's own camera fixes set it. CONCLUSION_DRYRUN

## What's next

1. First board session with `--switches-back`: read `unc_obs.csv` and the session summary's uncertainty line
   (how many corrections, and the share inside 2.45 sd: 95 % if the model is right).
2. Re-run `heading_diag.py` on that session: 30°+ heading errors should drop from 17 % to about 2 %.
3. The camera picture check in the ladder: with the boards against the plywood's edge, the plywood's own edge may
   look like a board line (DOE row, 5 minutes on the board).
4. A fleet plan with the back attachment on the real robots: a short one (one board, one robot) before the demo map.

## Sweep 1: new tests that may now be needed

- **The ladder on the real board.** Lift one board out mid-session: does it end at step 4 (board marked missing, edge
  find) without a hold? Put it back: is it found again (the map keeps it missing for the session today)?
- **Picture check with the board out.** Ten pictures with the board in and out: can `measure_board_line` tell them
  apart at all when the board stands at the plywood's edge?
- **Fleet touch on the real switches.** One robot, one board: the creep's travel at the click against the planned
  4 cm, ten visits.
- **Home dock backwards with the planner.** `run_plan.py --home` with the back attachment on the real dock v4/v5.
- **Heading push rate.** How often `heading_pushes` fires in a real session, and whether any push was wrong (the
  camera's direction of a 10 cm leg is itself ±3-6°).
- **Learner against a changed robot.** A simulator session with one robot's gyro scale changed (`--degrade`):
  the "changed" line should come at its 30th correction.
- **Confidence-pause grid** with the learned model (the logs DOE's next step), now that pauses follow each robot.

## Sweep 2: possible inaccuracies in what this relied on

- **The model's shape.** sd grows linearly with distance and with turning; real heading errors are not that simple
  (a wrong heading written in, which the fixes now stop). The outlier share absorbs what the shape cannot.
- **Corrections include the camera.** Taken out with e02_camerr's sd, else 0.8 cm per axis (the logs DOE's healthy
  value), not measured per spot yet.
- **KC stops** use the planned distance and turning since the last KC fix (waypoints), not the driven ones.
- **Chosen numbers** (all with DOE rows): learner floors, 30 corrections, 6 per term, 25 % / 95 % change report;
  ladder look 15 cm, camera line 3 cm, staging 35 cm out and 25 cm along; fleet touch lead 3 cm and cap 1 cm;
  heading push 8°; holds per find 3 (no DOE: Nick's time).
- **The keep sweep** used the simulator's switch, contact and gyro models (guesses until the attachment is on
  camera), and only the board-in case; the look-off caster distance and the margin were not re-run.
- **The 0.014 start value** is the refit on all intact chains; without the unmeasured-heading chains the heading
  report found 0.0126. The learner replaces it with each robot's own after 30 corrections.
- **`home_docks` heading convention.** data-formats.md says "the way a docked robot faces (into the wall)", the v3
  attachment README says "heading out of the dock". The planner keeps the map's heading as mouth-to-wall (what the
  outline code draws) and adds 180° itself for the back attachment. If maps are written the v3 way, the docks would
  be planned backwards.
- **A board marked missing** stays missing for the session; the robots' own firmware border only knows one open
  edge per axis, so for the gone board's edge only the camera watchdog's border applies.
- **Simulator turning noise is lower than the real robots'** (logs DOE §1: the dry run's model said ~4 cm where
  the simulator's truth was 1.6 cm), so the dry run says little about how often real robots pause.
