# E0b: Start-roll check (part of T0)

Status: planned    Week: 5 (Oct 18)    Run dates:

## Question
Is a 5 cm start roll better than 2 cm? Every T1 run starts with a short roll
forward and back so the camera sees which way the robot faces. A longer roll
should give the camera a more precise heading, but may leave the robot further
off its tape marks when it rolls back.

This also gives the repeat-heading measurement T0's skipped second pass was
for: the robot is marked twice without being lifted.

## Hypotheses
- (a) The camera's heading from a 5 cm roll is closer to the tape heading
  (smaller spread) than from a 2 cm roll. H0: same spread.
- (b) The 5 cm roll leaves the robot further off its marks. H0: no difference.

## Decision rule (set before the runs)
Use 5 cm for T1 if its camera heading spread is smaller than at 2 cm AND the
5 cm roll leaves the robot off its marks by no more than 0.2 cm and 0.3 deg on
average. Otherwise keep 2 cm.

## Variables
| Kind | Variable | Levels or units |
| --- | --- | --- |
| Independent | roll length | 2 cm, 5 cm (`start_nudge_cm`) |
| Dependent | robot off its marks after the roll | cm and deg, tape |
| Dependent | camera heading minus tape heading | deg |
| Blocked by | robot | Left, Center (Right stays low for E1b) |
| Held constant | spot (the L start spot, 40, 80 cm, facing +x), floor, camera, firmware 2026-10-18a | |

Each run is `set start_nudge_cm 2 or 5 / set straight_rpm 1 / set straight_cm 0 / run straight`:
the start flash, the roll, and no route.

## Procedure
Right after the camera calibration, before E1b. Same setup as T1 (camera
recording, robots on).
1. On the laptop, in the repo folder:
   `python tools/run_sheet.py docs/experiments/E00b-nudge-check --pi st1maxfini.local --inches`
2. It walks you through each run like T1: calibration run once per robot,
   place the robot on the L start spot, type the start marks, Enter to run,
   type the end marks. Don't touch the robot between the two sets of marks.
3. Afterwards, copy the logs and recording as in T1.

## Trials
5 per roll length per robot = 20 runs (seed 23), about 30 minutes. Shuffled
within each robot.

## Analysis
- Off-marks and tape repeatability: `python tools/nudge_check.py docs/experiments/E00b-nudge-check`
- Camera vs tape heading per roll length: run `tools/blink/calib.py` on the
  recording (T0's floor calibration applies: same camera position), then
  `python tools/heading_check.py docs/experiments/E00b-nudge-check --calib-results <recording>/calibration_results.csv`,
  then `nudge_check.py` again; it applies the decision rule.
- If 5 cm wins, the T1 sheets are remade with
  `python tools/make_run_order.py E1 --robots Left,Center,Right --reps 4 --seed 21 --nudge-cm 5 --force`
  (and E1B the same with `--robots Right --seed 22`), before any T1 run.

## Results

## Conclusion

## Deviations
