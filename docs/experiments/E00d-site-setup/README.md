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
   python tools/site/setup.py --site site --map site/map.json --calib <recording>/floor_calibration.json --recording <recording> --bench bench.json --segment-spots 6 --audit-every-landmark
   ```

4. Detector on the survey recording, clock check:

   ```
   python tools/check_session.py site/sessions/Left_views0 --video <survey recording>
   ```

   and the detector as for T1 (docs/experiments/INDEX.md, Workflow) ->
   `<survey recording>/detections.csv`.
5. Site setup, steps 5-8 (solve, tape fill-in, audit, lock):

   ```
   python tools/site/setup.py --site site --map site/map.json --calib <recording>/floor_calibration.json --recording <survey recording> --detections <survey recording>/detections.csv --bench bench.json --audit-every-landmark
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

`python tools/site/sim_eval.py --out results/site_eval --seeds 1 2 3 --fusion-files`:
the floor of the fusion's landmark test (tools/site/examples/sim_site_world.json:
four boards and a 19 cm cinderblock, one board just past the x = 122 side, an
unmapped brick; open sides x0 and y1), each robot's front sensor with the
r7/r11 errors (zones turned 0.4-3.3 deg, range bias -0.3 to +0.4 cm, scale
+-1%), wheels 3% off, clicks 1 cm off per board end (x and y each), the camera
0.3 cm and 3 deg per sighting with a 0.15 cm rms calibration error over the
floor (an assumption), live camera fixes 10 a second. Error per landmark: its
worst tape point (both ends of both long faces); 3 seeds x 5 landmarks:

| Map | mean | median | worst | landmarks within 0.5 cm | open edges, mean / worst |
| --- | --- | --- | --- | --- | --- |
| (a) clicks only | 1.54 | 1.03 | 3.46 | 0 / 15 | 0.47 / 1.01 |
| (b) clicks + per-robot calibration runs (tof_calib.py on the spins) | 1.49 | 1.18 | 3.42 | 1 / 15 | 0.47 / 1.01 |
| (c) site setup, robots only, with bench range priors | 0.82 | 0.34 | 3.32 | 10 / 15 | 0.34 / 0.70 |
| (c) same without bench range priors | 0.58 | 0.37 | 2.58 | 10 / 15 | 0.34 / 0.70 |
| (c) + tape fill-in (what the audit checks) | 0.31 | 0.31 | 0.46 | 15 / 15 | 0.22 / 0.27 |
| (c) locked (audit readings merged too) | 0.30 | 0.31 | 0.42 | 15 / 15 | 0.11 / 0.17 |

The robots-only worst cases are board ends with something right behind them
(B's ends next to C and the brick, C's end against B, E standing past the
plywood): no corner bearing there, so the end stays where the click put it, and
the map says so (sd about 1 cm: 2 or 3 such ends per seed). Those are exactly
the points step 6 asks to tape (3-5 points per seed). Bench range priors made
no clear difference here (the r7/r11 range biases are small and boards are
seen from many spots, by three robots, with corner sweeps); their worth is on
boards seen from one side only.

Sensor model from the survey (seed 1, camera error 0): zone directions within
0.4 deg of the truth, range bias within 0.06 cm, scale within 0.2%, zone width
6.9-7.3 deg (true: the simulator's outer rays 7.33 deg apart).

Honesty: worst-point error / the landmark's stated sd, locked map, 15
landmarks: 0.5-2.1 (median 1.6) for the worst of 4 points each; every audit
(9 points) passed.

Effort per seed: 14 clicks (2 per board, 2 per open side), 7 yes/no answers,
1 open-sides answer, 6-8 tape points (fill-in + audit), 3 robot placements:
about 14-17 minutes of user time (10 s a click, 90 s a tape point, 30 s a
robot). Robot time: 3 minutes of calibration runs, 44-46 minutes of survey (the
three robots one after another: about 39 spots each, a third of them corner
sweeps, and 5-6 edge approaches; 13-15 of 15-18 approaches stopped by the edge
guard, the others ended at the drive's end inside the edge).

Fusion with the locked map: see "Fusion with the locked map" below.

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
