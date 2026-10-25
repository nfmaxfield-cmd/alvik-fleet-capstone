# DOE backlog: chosen numbers waiting for an experiment

Nick (standing rule): every chosen number says whether a design of experiment should set it. This file lists those
experiments so they get run, instead of staying a promise in a comment. Each session's plan starts by picking from
it.

- **Simulator-only rows:** Claude runs these without asking, whenever the simulator machine is free.
- **Rows that need the board, or Nick's time:** proposed in the next-session plan; Nick confirms.

Status: open / planned (date) / done (result, link).

| Number (where) | Value now | Experiment that would set it | Needs | Status |
|---|---|---|---|---|
| Camera virtual border, away (e02.py BORDER_CM) | 15 cm | Drive at the border line at 4 and 10 cm/s, 5 times each, with and without a planned load; measure the overshoot past the trip point | Board, ~20 min | open |
| Camera virtual border, home (HOME_BORDER_CM) | 8 cm | Same runs as above; also the home sessions' border_shadow.csv counts | Board | open |
| Robots' own border (FW_BORDER_CM / HOME_FW_BORDER_CM) | 10 / 6 cm | Same runs, with the camera covered for the last stretch (the robot's own count alone) | Board | open |
| Recheck before "lost" (RECHECK_S) | 3.5 s | Count rescues against holds over the next 3 sessions; replay sessions' tracks with 1-6 s | Logs + simulator | open |
| Slow thresholds (e02_load SLOW_FPS_FRAC / SLOW_AGE_FRAC / SLOW_HOLD_S) | 0.75 / 1.5 / 5 s | Load at two strengths (cpu:1, disk upload) in a session; pick the thresholds that flag the load blocks and nothing else | Board session with --load | open |
| Bad-camera border shrink (BORDER_EXTRA_SIGMA / MAX) | 2 sd / 10 cm | Simulator grid at camera noise 1, 2, 3 cm: holds and off-board count against runs completed | Simulator | in progress (grid) |
| Park spots moved in (PARK_OPEN_MAX_CM) | 22 cm | Simulator grid, noisy camera: parking holds and near-misses against the route clearance | Simulator | in progress (grid) |
| Trust flags (e02_trust DEGRADED_K / RECOVERED_K / STANDS_OUT) | 2 / 1.5 / 1.5 | Swap test: 20 injected faults per sensor and level (simulator), then real; detection rate and false alarms per hour | Simulator, then board | open |
| Still-camera healthy scatter (CAMERA_STILL_HEALTHY) | 0.4 cm | A robot standing still at 7 places for 30 s each | Board, ~10 min | open |
| Charge levels (dock plan) | 40% go / 90% leave | Simulator sweep once bench test 1 gives the real charge rate | Bench test 1, then simulator | open |
