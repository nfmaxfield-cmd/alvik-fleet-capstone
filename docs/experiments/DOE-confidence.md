# Design of experiment: the confidence-pause threshold

Nick, 2026-10-23: "3 cm and DoE to finalize." The runner keeps each robot's own position uncertainty (1 standard
deviation, cm) from what it has driven and turned since its last camera fix or find. Before a drive that would push
the uncertainty past the threshold, the robot stops and takes a fresh camera fix (a confidence pause). Every pause
is logged in `confidence.csv` (time, robot, uncertainty before and after the drive, threshold, what happened).

Chosen, not measured: the threshold (3 cm) and the uncertainty model (0.5 cm + 1% of the distance driven + the
distance times 0.5 deg + 0.5% of what it turned; e02.py `UNC_*`).

## Question
Which threshold gives the best trade-off between position accuracy and time spent pausing?

## Factors and levels
| Factor | Levels | Now |
|---|---|---|
| `CONF_PAUSE_CM` | 1.5, 3, 6, off | 3 |

"Off" is the control: no pauses at all.

## Material and plan
- Real sessions, as one more factor in the E02 schedule: each block runs every threshold once per robot and route,
  in random order (randomised complete blocks: block = robot x route).
- At least 6 runs per threshold per route (about 72 runs: one session of 1.5 hours).
- Pictures must be on time in this test (no WiFi copies; `perf.csv` checked afterwards). Late pictures are item 5's
  case and a separate test.

## Responses
- End error (camera, measured from the footage): the accuracy.
- Seconds spent in pauses per run, and the number of pauses: the cost.
- How often a pause found the robot further off than its uncertainty said (the camera's correction over the
  predicted SD): whether the uncertainty model itself is right. If corrections are routinely bigger than predicted,
  fit the `UNC_*` constants to them first (a regression of correction size on distance and turn since the last fix),
  then rerun the threshold test.

## Analysis
- End error by threshold: stratified permutation test (shuffles within robot x route), Holm correction across the
  pairs, as in the E02 analysis.
- Pause time by threshold: medians and 95% intervals.
- Recommendation: the largest threshold whose end error is not worse than the smallest one's (equivalence margin
  0.5 cm), because it pauses least.

## Before running it
The model's constants are chosen from session 1's calibrated runs. Check them first against the corrections
logged when a pause takes a fix (`confidence.csv`, column correction_cm: the camera against its own pose).
