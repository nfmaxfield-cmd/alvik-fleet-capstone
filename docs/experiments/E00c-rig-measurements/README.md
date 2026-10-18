# E0c: Rig and robot measurements (part of T0)

Status: planned    Week: 6 (this week)    Run dates:

## Question
Are the numbers the tools assume about the robots, the board and the camera
right? Each one was a guess or measured once (see
[ACCURACY-CHECKS.md](../ACCURACY-CHECKS.md), section C). This session
measures them so the tools use real values.

## What you need
Tape measure, carpenter's square, string, a nut or washer (for a plumb line),
a straightedge (the square's long arm works), a dime and a penny, a
thermometer, one vertical board, and a second person for the camera height.

## Part 1: by hand (about 40 min)
Fill in `data/rig_measurements.csv` (open it in Excel; type inches as you read
them, like `3 3/4`; save as CSV). Each row says what to measure and how.
- **Robots (C1 to C4):** length, width, front face to wheel axle, top height,
  light height, wheel sizes. All three robots.
- **Board (C5, C6):** both diagonals; the gap under a tight string along each
  edge; the 3-4-5 check on the taped axes (30 cm = 11 13/16 in along x,
  40 cm = 15 3/4 in along y, then the diagonal between those two marks:
  50 cm = 19 11/16 in if the axes are square).
- **Camera (C7):** hang the nut on the string from the middle of the lens so
  it just clears the floor, mark the floor under it, measure that point from
  the axes. Measure the lens height with a second person holding the tape.
- **Floor (C8):** straightedge across the board in 4 directions; at the worst
  gap, try the dime and the penny.
- **Room (C15):** temperature.

## Part 2: robots logging while still (about 50 min, plus 10 min warm-up)
Camera recording, as in T1. Robots switched on **just before** this part.
1. Stand the board on the floor with its face along x = 110 cm (43 5/16 in),
   square to the x axis, across y = 61 cm (24 in).
2. On the laptop, in the repo folder:
   `python tools/run_sheet.py docs/experiments/E00c-rig-measurements --pi st1maxfini.local --inches --run-timeout 900`
3. It walks you through each row:
   - **warmup** (Left, first): 10 minutes still. Do Part 1 meanwhile.
   - **still60:** 60 s still; don't touch the robot. It should finish "done".
   - **lift:** about 5 s after it starts, lift the robot and set it back down.
     It should stop with "lifted". When asked, answer `y` and mark it where
     it sits.
   - **tof10 ... tof100:** the robot faces the board, front face 10 to 100 cm
     from it (the start spot is given). 10 s still each.
   - **edge_n15, edge_n10, edge_n5:** take the board away first, so the far
     x edge of the plywood (x = 122 cm, 48 1/16 in) is open. The robot faces
     that edge with its front face 15, 10 and 5 cm (5 7/8, 3 15/16, 1 15/16 in)
     from it. Checks whether the distance sensor's lowest zone sees the floor
     drop away before the robot gets close.
   - **edge_in, edge_over:** the board still away. The robot faces the open edge.
     edge_in: its nose about 7/8 in inside the edge. edge_over: its nose
     sticking out about 1 in past the edge (the wheels stay about 1 in
     inside, so it can't tip). 10 s still each. Don't let it move.
   - **edge_tape:** the robot faces the x-axis tape (the origin-side edge),
     its nose resting on the tape. Checks that tape doesn't look like the edge.
4. **At the end, switch each robot off and on**, so every setting goes back to
   normal.

## Analysis
- Part 1 values replace the assumed ones in the tools (robot centre to axle,
  top height, light height, wheel size priors, board and camera geometry).
- Part 2: `tools/check_session.py` reports the gyro warm-up, rest noise and
  lifted flag; the front sensor's reading vs distance gives its noise and bias
  per zone (B26 in ACCURACY-CHECKS.md).
- Edge rows: the three line sensors under the nose read the plywood, the tape
  and the open edge, and the distance sensor's lowest zone reads the floor
  ahead at 15, 10 and 5 cm from the edge. If the edge reading is clearly different from both
  plywood and tape, the next firmware's edge guard (stop before the nose
  passes an open edge) gets its threshold from these numbers:
  `python tools/edge_check.py` reads these rows and prints the verdict and the
  settings (safe_edge, edge_dir, edge_thr, edge_max, edge_bot_cm).

## Results

## Conclusion

## Deviations
