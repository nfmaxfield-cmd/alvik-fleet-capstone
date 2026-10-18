# Experiment index

The running record of every experiment. Each row links to its full record.
Units: cm and degrees throughout. Ground truth: tape measure (see E0), and
from E4 on the overhead camera once it agrees with the tape.

Objective: how much merging several imperfect sensors improves an Alvik's
locational accuracy, and whether it holds up when sensors fail. Our own test
list T0-T9 replaces the original E1-E8 (reasons in the plan-changes note); the
first two records keep their E00/E01 folder names.

| Test | Week | Title | Status | Headline result |
| --- | --- | --- | --- | --- |
| [T0](E00-measurement-check/) | 5 (Oct 18) | Reference check (tape repeatability; also calibrates the camera) | planned | |
| [E0b](E00b-nudge-check/) | 5 (Oct 18) | Start-roll check: 2 vs 5 cm roll (camera heading vs staying on the marks); also the tape's heading repeatability | planned | |
| [E0c](E00c-rig-measurements/) | 6 | Rig and robot measurements: robot sizes, board, camera plumb point, front sensor vs distance, gyro warm-up, lifted flag | planned | |
| [E0d](E00d-site-setup/) | 7 | Site setup: robots + camera + clicks learn every landmark and open edge, tape audit, locked map; then every landmark tape-measured to score it | planned (simulation done) | Simulation: worst-point error per landmark, mean 1.54 cm clicked -> 0.82 robots only -> 0.31 cm with 3-5 tape fill-in points (all 15 within 0.5 cm); audits passed |
| [T1](E01-baseline-dataset/) | 5-6 (Oct 18-19) | Baseline data set: 36 runs, every sensor logged and filmed | planned | |
| [E1b](E01b-low-battery/) | 5 (Oct 18) | Low battery: Right's routes at low charge, paired with its full-charge T1 runs | planned | |
| T2 | 6 | Each sensor's own errors (camera included) | planned | |
| T3 | 7 | Fusion gain: every sensor combination on T1 | planned | |
| T4 | 8 | Failures: sensors off, blocked, late; robot picked up | planned | |
| T5 | 6-8 | Delay budget: WiFi, camera, processing | planned | |
| T6 | 8 | Difficult scenes: floors, lighting, hands, look-alikes | planned | |
| T7 | 9 | Several robots: identity and tracking | planned | |
| Dock | 10 | Docking task: planned routes to boards, 1 cm stops | planned | |
| T8 | 11 | Speed vs accuracy, with and without fusion | planned | |
| T9 | 11-12 | Comparison with the lab's AprilTag system | planned | |

Every possible source of inaccuracy and the check for it: [ACCURACY-CHECKS.md](ACCURACY-CHECKS.md).

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
