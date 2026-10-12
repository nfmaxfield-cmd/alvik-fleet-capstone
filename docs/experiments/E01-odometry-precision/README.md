# E1: Alvik precision, odometry only

Status: planned    Week: 5 (Oct 12-18)    Run dates:

## Question
How accurately does the robot go where it's told, with no external correction?

## Hypotheses
- (a) IMU-feedback turns land within 1 deg of 90 deg on average; library
  `rotate()` turns miss by more than 2 deg. H0: no difference in mean
  absolute heading error.
- (b) Along-track error grows in proportion to distance (a wheel-size error);
  cross-track error grows faster (a small heading drift that compounds).
  H0: error does not depend on distance.
- (c) After a closed path, the true return error is larger than odometry
  reports, because odometry cannot see wheel slip. H0: tape = odometry.
- (d) Error rises with speed and as the battery runs down. H0: no effect.

## Variables
| Kind | Variable | Levels or units |
| --- | --- | --- |
| Independent | path | turn; straight 30/60/120 cm; L-path; square 1 and 3 laps |
| Independent | turn method (part a) | library `rotate()`, IMU feedback |
| Independent | speed (part e) | 20, 50 wheel rpm on the 60 cm straight |
| Independent | battery (part e) | above 80%, below 30% (blocks) |
| Dependent | final position error, split into along-track and cross-track | cm, tape rig |
| Dependent | final heading error | deg, tape rig |
| Dependent | odometry's own estimate; route time | cm/deg, ms; robot log |
| Held constant | surface, lighting, `settle_ms` 400, square side 50 cm | |

## Setup
- Robot(s): Left. When Center and Right are flashed, each robot runs every
  level (blocked by robot): regenerate the sheets with `--robots Left,Center,Right`.
- Firmware commit:
- Surface:    Lighting:
- Battery at start / end:
- Ground truth: tape rig, E0 repeatability:

## Procedure
1. After power-on, send `calibrate` (the robot wiggles once to check its
   compass direction). Then place it on the origin, pointer along +x.
2. For each row of the part's sheet: send the commands in the row, wait for
   the run to finish, write the run id (from the console line
   `run XXXX-n: ... starting`), mark both ends of the pointer, measure, fill
   in the row, and put the robot back on the origin.
3. Write `battery_pct` from `status` every few trials.
4. Copy the console's session folder into `<part>/data/logs/`.

## Parts and trials
| Part | Folder | Levels | Trials | Why that many |
| --- | --- | --- | --- | --- |
| E1a Turns, 90 deg left | [a-turns](a-turns/) | library, IMU | 16 per method | Detects a 1 deg difference if trials spread about 1 deg (95% confidence, 80% power) |
| E1b Straight lines | [b-straight](b-straight/) | 30, 60, 120 cm | 8 per distance | +/-0.8 cm interval on each mean if spread is about 1 cm; gives cm of error per metre |
| E1c L-path | [c-l-path](c-l-path/) | 30 cm, right turn, 30 cm and back | 10 | Tape vs odometry in pairs |
| E1d Square, 50 cm sides | [d-square](d-square/) | 1 lap, 3 laps | 6 per level | Drift with distance when turns are included |
| E1e Speed and battery | [e-speed-battery](e-speed-battery/) | 20, 50 rpm x battery high, low | 4 per cell | 2x2 screen; E7 sweeps speed fully |

About 100 trials. Each sheet is shuffled (seeds 11-15), so battery drain and
floor wear don't line up with one level. Battery can't be shuffled, so E1e
runs its high-battery block first and its low-battery block later.

Repeatability = the SD of repeated trials at each level.

## Raw data
`<part>/data/measurements.csv` (tape) and `<part>/data/logs/` (robot).

## Results
Per part: `python tools/tape_results.py docs/experiments/E01-odometry-precision/<part> --logs docs/experiments/E01-odometry-precision/<part>/data/logs/*`

## Analysis
- a: Welch's t-test on absolute heading error.
- b: error vs distance fitted with a line, slope in cm per metre.
- c, d: paired t-test of tape vs odometry return error.
- e: mean per cell; speed and battery effects with 95% intervals.

## Conclusion

## Deviations
