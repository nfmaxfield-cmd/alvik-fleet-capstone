# E<n>: <title>

Status: planned | running | done    Week: <n>    Run dates: <dates>

## Question
One sentence.

## Hypothesis
H1 with a number, e.g. "IMU turns land within 1.0 deg of 90; library turns miss by more than 2 deg."
H0: no difference.

## Variables
| Kind | Variable | Levels or units |
| --- | --- | --- |
| Independent | | |
| Dependent | | cm or deg, and how measured |
| Held constant | robot, surface, speed, battery range, ... | |

## Setup
- Robot(s): Left / Center / Right
- Firmware commit: <`git rev-parse --short HEAD` when flashed>
- Surface: <floor type>    Lighting: <room light, lamps on/off>
- Battery at start / end: <%> / <%>
- Ground truth: tape rig (E0 repeatability: <x> cm, <y> deg)

## Procedure
1. After power-on, send `calibrate`, then place the robot on its start mark.
2. ...

## Trials
- n per level, and why (effect size to detect, pilot spread).
- Run order: data/measurements.csv, made by `tools/make_run_order.py ... --seed <n>`.
- Blocking: by robot.

## Raw data
- data/measurements.csv: tape readings, one row per trial, with run_id
- data/logs/: the console's CSV files for these runs (copy the session folder here)

## Results
Paste the table printed by `python tools/tape_results.py <this folder> --logs data/logs/*`.

![chart](chart.svg)

## Analysis
Test used, result, p value. What else could explain it.

## Conclusion
H1 supported / not supported, in one sentence with the number.

## Deviations
Anything that differed from the plan. Excluded trials stay in the sheet with excluded = 1 and a reason.
