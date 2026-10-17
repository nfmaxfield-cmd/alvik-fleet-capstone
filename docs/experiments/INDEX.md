# Experiment index

The running record of every experiment. Each row links to its full record.
Units: cm and degrees throughout. Ground truth: tape measure (see E0), and
from E4 on the overhead camera once it agrees with the tape.

Objective: how much adding sensors improves an Alvik's locational accuracy.
Core result: E6 (camera-corrected) against E1 (odometry only), against the
proposal target of under 5 cm.

| Exp | Week | Title | Status | Headline result |
| --- | --- | --- | --- | --- |
| [E0](E00-measurement-check/) | 5 (Oct 12-18) | Measurement check (tape rig) | planned | |
| [E1](E01-baseline-dataset/) | 5 (Oct 17-18) | Baseline data set: odometry-only accuracy, plus every sensor logged and filmed for fusion replay | planned | |
| E2 | 6 (Oct 19-25) | Lab system baseline | planned | |
| E3 | 7 (Oct 26-Nov 1) | Communication baseline | planned | |
| E4 | 8 (Nov 2-8) | AprilTag precision, static | planned | |
| E5 | 9 (Nov 9-15) | AprilTag tracking, moving | planned | |
| E6 | 10 (Nov 16-22) | Alvik with camera corrections | planned | |
| E7 | 11 (Nov 23-29) | Speed vs. accuracy | planned | |
| E8 | 12 (Nov 30-Dec 6) | Multi-robot coordination | planned | |

Start a new record by copying [TEMPLATE.md](TEMPLATE.md) into a folder
`E<nn>-<name>/README.md`, and fill in everything above Results before the
first trial.

## Workflow for a tape-measured experiment

1. Make the shuffled measurement sheet:
   `python tools/make_run_order.py E1 --robots Left --reps 8 --seed 21`
2. Start the console (`python tools/alvik_console.py`) and the Pi recorder
   (`python3 camrec.py --label <exp>`). Send `calibrate`.
3. Work down the sheet: mark and measure the robot's start dots, send the
   commands, write the run id from the console, mark and measure the end dots.
4. Copy the console's `logs/<date-time>/` folder into the experiment's
   `data/logs/`, and check it: `python tools/check_session.py <folder> --video <recording>`.
5. `python tools/tape_results.py <experiment folder> --logs <folder>/data/logs/*`
   prints the tables and writes `data/results.csv` and `chart.svg`.
6. Fill in Results, Analysis and Conclusion; update this index.
