# E0d: Site setup: is the locked map accurate?

Status: planned (simulation done)    Week: 7    Run dates:

## Question
After the site setup (tools/site/setup.py: robots survey every landmark, joint
fit, tape fill-in and audit), how far is each landmark point and open edge
from where a tape measure puts it, and is the map's stated uncertainty honest?

## Hypotheses
- H1: 90% of landmark points within 0.5 cm of the tape, none over 1.0 cm
  (simulation: 100% within 0.46 cm, mean 0.31 cm).
- H2: the stated uncertainty is honest: at least 85% of points within 2.45 x
  their stated sd (2.45 sd is the 95% radius of a 2-D normal error).
- H3: the site setup at least halves the error of the clicked map (median)
  (simulation: 1.03 -> 0.31 cm).
- H4: the change check flags a board moved 1 cm, and flags nothing over a
  session where nothing moved.
- H0: the survey does not improve on the clicks.

## Variables
| Kind | Variable | Levels or units |
| --- | --- | --- |
| Independent | map source | clicked only; per-robot calibration (tof_calib.py); site setup, robots only; site setup, locked (tape fill-in + audit merged) |
| Dependent | error of every landmark point and open-edge point | cm, tape measure from the axes' origin |
| Dependent | normalised error | error / stated sd |
| Dependent | user effort, robot time | clicks, tape points, minutes |
| Held constant | boards, floor, camera, its calibration, lighting | |

## Setup
- Robots: Left, Center, Right, each calibrated since power-on, firmware
  2026-10-18c (look-first calibration, edge guard).
- The floor as for T1, with the boards where they will stay for the week:
  write their names on them (A, B, ...).
- Camera calibration passed B1 (left-one-out error 0.35 cm or less).
- E0c done: C9 rows (range bias per robot -> `bench.json`, see
  tools/site/README.md) and the edge rows with `tools/edge_check.py` PASS (else:
  no open edges in this test, answer `none`).
- Ground truth: tape measure (T0 repeatability), read to 1/16 in.

## Procedure
All commands in a Terminal window on the laptop, in the project folder, one at
a time; the Pi's commands in the Pi's Terminal (RealVNC).

1. Floor and start spots in the fleet page (it saves `site/map.json`):

   ```
   python tools/fleet/app.py --map site/map.json
   ```

2. Per-robot calibration after power-on: in the console, `calibrate` for each
   robot. Then close the console. Save the calibration's tof calibration if you
   have one (`tools/fusion/tof_calib.py --calib-results ...`) as
   `site/tof_cal.json`; otherwise leave `--tof-cal` out below.
3. Site setup, steps 1-4 (camera check, landmarks, plan, survey); follow
   tools/site/README.md "Step by step", A to D:

   ```
   python tools/site/setup.py --site site --map site/map.json --calib <recording>/floor_calibration.json --recording <recording> --bench bench.json
   ```

4. Detector on the survey recording, clock check:

   ```
   python tools/check_session.py site/sessions/Left_views0 --video <survey recording>
   ```

   and the detector as for T1 (docs/experiments/INDEX.md, Workflow) ->
   `<survey recording>/detections.csv`.
5. Site setup, steps 5-8 (solve, tape fill-in, audit, lock):

   ```
   python tools/site/setup.py --site site --map site/map.json --calib <recording>/floor_calibration.json --live-log logs/<date-time>-live/messages.jsonl --bench bench.json
   ```

   Write down the tape readings it asks for as you type them (also in
   `site/audit.csv` and `site/map.json`).
6. **Ground truth.** Tape-measure EVERY point: each board's four floor corners
   (`A: left p1`, `A: left p2`, `A: right p1`, `A: right p2`; left/right of the
   direction p1 -> p2 as in the map; the program prints each point's rough
   position), each block's corners, and each open edge at 25, 50 and 75 % of its
   length (only the coordinate across the edge). Enter them in
   `data/truth.csv`: `point,x,y,unit` (inches like `36 3/32`, unit `in`).
   Measure without looking at the map's numbers.
7. Score:

   ```
   python tools/site/score.py --map site/map.json --truth docs/experiments/E00d-site-setup/data/truth.csv --clicked site/map_clicked.json
   ```

8. **Change check.** Run any T1-type plan with the check on and nothing moved:

   ```
   python tools/fleet/run_plan.py --map site/map.json --plan plan.json --site-map site/map.json
   ```

   Expect no "may have moved" line. Then move board A 1 cm (3/8 in) straight
   away from the robots, run the same plan again: expect "landmark A may have
   moved" within the plan. Put A back.

## Trials
- One site setup, every point tape-measured (about 20 boards' corners and 6
  edge points). Repeat the tape readings of 5 points twice to get the tape's own
  spread on this floor.
- The change check: 2 runs (unmoved, moved).

## Pass rules
- P1 (H1): 90% of points within 0.5 cm, none over 1.0 cm.
- P2 (H2): at least 85% of points within 2.45 x the stated sd.
- P3 (H3): median error of the locked map at most half that of the clicked map.
- P4 (H4): moved board flagged, nothing flagged when unmoved.
`score.py` prints P1-P3.

## Raw data
- data/truth.csv: tape readings of every point (point,x,y,unit)
- site/: the locked map, landmarks.csv, audit.csv, tof_calibration.json,
  site_state.json, sessions/ (copy the folder here as data/site/)
- the survey recording's detections and video clock

## Results (simulation, 2026-10-18)

`python tools/site/sim_eval.py --out results/site_eval --seeds 1 2 3`: the
floor of the fusion's landmark test (tools/site/examples/sim_site_world.json:
four boards and a 19 cm cinderblock, one board just past the x = 122 side, an
unmapped brick; open sides x0 and y1), each robot's front sensor with the
r7/r11 errors (zones turned 0.4-3.3 deg, range bias -0.3 to +0.4 cm, scale
+-1%), wheels 3% off, clicks 1 cm off per board end and per floor side (x and y
each), the camera 0.3 cm and 3 deg per sighting with a 0.15 cm rms calibration
error over the floor (an assumption), live camera fixes 10 a second, edge
approaches stopping 3 cm inside the clicked edge, the first-setup audit (one
point per landmark and open edge). Error per landmark: its worst tape point
(both ends of both long faces); 3 seeds x 5 landmarks, cm:

| Map | mean | median | worst | within 0.5 cm | open edges, mean / worst |
| --- | --- | --- | --- | --- | --- |
| (a) clicks only | 1.54 | 1.03 | 3.46 | 0 / 15 | 0.47 / 1.01 |
| (b) clicks + per-robot calibration runs (tof_calib.py on the spins) | 1.49 | 1.18 | 3.42 | 1 / 15 | 0.47 / 1.01 |
| (c) site setup, robots only | 0.69 | 0.56 | 2.23 | 7 / 15 | 0.58 / 1.00 |
| (c) + tape fill-in (what the audit checks) | 0.34 | 0.32 | 0.66 | 13 / 15 | 0.15 / 0.26 |
| (c) locked (audit readings merged too) | 0.27 | 0.25 | 0.79 | 13 / 15 | 0.14 / 0.23 |

The robots-only worst cases are board ends with something right behind them
(no corner bearing there: the end stays where the click put it, and the map
says so, sd about 1 cm) and open edges where the guard did not trip. Step 6
tapes those (3-6 points per seed). Every audit (7 points a seed) passed.
Merging the audit readings made one landmark worse (seed 2, A: 0.30 -> 0.79
cm): a tape reading is in the tape's frame, the robots' readings in the
camera's; see "Possible inaccuracies".

Edge readings with the robot centre never planned closer than 3 cm inside the
clicked edge: 23 of 48 approaches tripped the guard (seed 1: 15/15, seed 2:
1/15, seed 3: 6/18); with the earlier 0.5 cm stop 41 of 51 did. The guard trips
only where the real edge is within 0.5 cm outside the click.

Sensor model from the survey (seed 1, camera error 0): zone directions within
0.4 deg of the truth, range bias within 0.06 cm, scale within 0.2%, zone width
6.9-7.3 deg (true: the simulator's outer rays 7.33 deg apart).

Effort per seed: 14 clicks (2 per board, 2 per open side), 7 yes/no answers,
1 open-sides answer, 10-13 tape points (fill-in + one audit point per landmark
and edge), 3 robot placements: about 20-24 minutes of user time (10 s a click,
90 s a tape point, 30 s a robot). Robot time: 3 minutes of calibration runs,
39-46 minutes of survey (the three robots one after another, about 39 spots
each, a third of them corner sweeps, and 5-6 edge approaches).

### Fusion with the locked map

The fusion's landmark test sessions r7 and r11 (tools/fusion/README.md, 36
runs each, same robots and sensor errors), replayed with the current fusion
(`replay.py`, setups from that README) twice: as before (the boards 1 cm off in
the map, refined by tof_calib.py leave-one-run-out from the session's own runs)
and with the locked site map and the site setup's sensor model
(`--map site/landmarks.csv --tof-cal site/tof_calibration.json`: made from the
survey's own runs, none of the scored ones; site map of seed 1 with no camera
calibration error, as the sessions have their own camera frame; its worst
landmark point 0.40 cm off, the cinderblock's far end). End error / path RMSE
cm / heading RMSE deg, r7 and r11:

| setup | 1 cm map + tof_calib | locked site map |
| --- | --- | --- |
| camera alone | 0.270 / 0.308 / 1.02 and 0.249 / 0.338 / 1.05 | (same) |
| + landmark ranging, always merged (consider states) | 0.309 / 0.339 / 1.05 and 0.362 / 0.382 / 1.07 | **0.221 / 0.280 / 0.99** and **0.288 / 0.300 / 1.02** |
| + landmark ranging, after 1 s without the camera | 0.275 / 0.295 / 0.96 and 0.346 / 0.349 / 0.98 | 0.242 / 0.256 / 0.94 and 0.316 / 0.289 / 0.96 |
| no camera, landmark ranging | 0.617 / 0.597 / 0.91 and 0.700 / 0.672 / 0.99 | 0.450 / 0.496 / 0.83 and 0.516 / 0.533 / 0.95 |
| camera blocked 20 s | 1.053 and 1.070 | (same) |
| ... + landmark ranging | 0.552 / 0.552 / 0.95 and 0.608 / 0.618 / 1.05 | 0.419 / 0.455 / 0.87 and 0.476 / 0.489 / 0.99 |

The always-merge penalty with the camera working (mean of both sessions) goes
from +0.08 cm (0.34 vs 0.26) to none (0.25 vs 0.26): r7 now gains 0.05 cm, r11
still loses 0.04 cm. Path and heading are better than the camera alone in both
sessions. Without the camera, or with it blocked 20 s, landmark ranging is 20-27%
better than with the 1 cm map.

## Analysis
(after the real run) P1-P4 from score.py and the change-check runs. Then:
- If P1 fails but P2 passes: the map is honest but not good enough: more
  survey (closer spots, more corner sweeps) or more tape points.
- If P2 fails: the uncertainty is understated: look for the shared error that
  is not modelled (camera calibration pattern: B2; the zone width; boards
  leaning) before trusting any map's sigma.
- Compare the robots' sensor models from the survey with C9 (bench) and with
  each power-on's calibration run (A14).

## Conclusion

## Deviations

## New tests that may now be necessary
- Corner sweeps on the real sensor: one board end swept from 16 and 30 cm,
  bearing vs tape (the simulator's zones have sharp edges; the real ones may not).
- Edge approaches on the real board with the 3 cm stop: how often the guard
  trips, and the trip distance past the edge (sets `edge_k0_cm`).
- The live feed during a full survey: fix rate and latency per robot while one
  robot surveys (tools/live/eval_live.py).
- The change check over a whole T1 session with nothing moved (false flags).

## Possible inaccuracies
- Camera calibration error 0.15 cm rms (chosen, not measured): the floor of
  every point's accuracy; B1/B2 give the real value.
- Tape readings are merged with sd 0.1 cm in the tape's frame while every other
  reading is in the camera's: where the two frames differ by more, a tape point
  pulls the board's other points off (seed 2, A). Next: give tape points
  sd = hypot(0.1, camera calibration error) and re-run sim_eval.
- Trip distance past the edge prior 0.2 +- 0.5 cm, bench range priors (0.15 cm),
  per-reading noise 0.5 cm + 1%, and 0.15 cm that does not average away: chosen.
- Click error 1 cm per board end and floor side, 90 s per tape point, 10 s per
  click: assumptions behind the effort numbers.
- The simulator's zone model (nearest of 3 rays, 15 frames/s) and edge-guard
  model are guesses until E0c.
