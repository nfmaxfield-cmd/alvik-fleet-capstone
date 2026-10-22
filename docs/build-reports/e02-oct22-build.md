# E02 build after the 2026-10-22 session: report

**Premise.** The morning session (09:51-10:43, release `e02-session-oct22`) finished one run. Robots were lost
over and over, one looped against a board, and Center fell off an open edge. Nick: fix it from the footage and keep
going until the usage runs out, then make the logic more generic. Plan and checklist:
`docs/build-plans/e02-oct22-build.md`.

## Data: what the session showed
- The runner received all 25 camera pictures a second (track.csv), so the frame rate was never the problem.
- **The tracker took Right for Left.** At 10:40:03, during a 167° turn, it lost Left for a moment and its search
  circle grew to 50 cm. It took Right (7.6 cm from Right's last spot, just outside the 7.6 cm "another robot" zone)
  for Left. Every "the camera saw it move 0 cm" after that was the camera watching Right.
- A robot standing still was learned into the floor memory bit by bit. The footage at 10:40:12 shows a ghost of
  Left in it, after which Left's outline broke up and it was lost.
- **The obstacle loop.** The turn legs between obstacle stops reset the stop count, so it never gave up.
- **The park spots.** All of them are 17-20 cm from an open edge, under the 20 cm "move it in first" limit. By the
  x-board, the move-in point was 1.4 cm from the board.
- **Center's fall (10:23).** Its edge guard learned the floor with all three line sensors already over the edge.
  That read like tape and switched the guard off.
- **Pressing Enter with no hands** found the robot nearly every time.

## Conclusion: what changed (all pushed, tests pass: 156 experiment, 106 simulator)
1. **Obstacle stops:** a robot backs off 8 cm, then turns a quarter turn toward the board's middle. After 3 blocks
   it stops.
2. **Retries:** three tries on its own before Nick is called. If the camera can't see it at all, another robot
   sweeps for it with its front sensor.
3. **Move-in:** only within 14 cm of an open edge, to a point 25+ cm from both boards. A robot found at an open
   edge is never driven.
4. **Firmware 2026-10-22b:**
   - no forward drive when the line sensors start off the plywood;
   - every finished leg reports its front distance.
5. **Tracker:**
   - a lost robot's search circle grows for at most 0.6 s;
   - the "another robot" zone is 1.1 robot widths;
   - pieces of one robot are merged;
   - a still robot's floor is protected;
   - a held robot whose spot hasn't changed counts as seen;
   - every leg tells the tracker where the robot can be;
   - the live tracker now has the board area and walls (it had none, so it followed Nick's hands 47 cm off the
     board), and pixels beyond a wall are cut, so a robot against a board is not joined to it.

   On Left's failing minute of footage, the robot is seen in 75% of pictures, up from 20%, and never confused with
   another. On last night's four clips, all pass as before.
6. **Front-sensor check:** after every leg, the front reading is compared with the map and written to
   `front_checks.csv`. It's recorded only, until a real session sets the limits.
7. **Generic design outline:** `docs/build-plans/generic-platform-outline.md`, to discuss.

## What's next
- **Nick:** upload firmware 2026-10-22b to all three robots before the next session (the runner refuses older).
- A robot pressed against a board is still unseen for stretches (10:40:30-40 on the footage: 30% seen).
- The lost robot finding itself against the boards needs a firmware "find from my own pose" mode.
- The second camera, once the overhead tracker holds.

## Sweep 1: new tests that may now be needed
- Today's footage as a regression clip (like last night's four), once the release's two video parts are joined.
- A simulator run where a robot is hidden from the camera, to exercise the finder sweep end to end. Today it is
  only tested with a stub robot.
- The edge guard's new "no plywood at the start" stop, on the real robot at an open edge (simulator only so far).
- A dry run with a robot pushed by hand while held, now that a held robot counts as seen while its spot looks
  unchanged.

## Sweep 2: possible inaccuracies this relies on
- The Pi clock is ~1.6 s ahead of the laptop's. This was read off one leg's motion in the footage and used to line
  up the replay.
- The new limits are chosen, not measured:
  - tracker: 0.6 s gate growth, 30 cm/s top speed, 1.1 / 0.8 / 1.35 / 1.25 robot widths, 3 cm past a wall;
  - runner: 8 cm back-off, 14 / 25 cm move-in;
  - sweep: 30° steps, 70 cm range, 8 cm gap.
  The tracker numbers were checked on the footage and clips; the rest are not.
- The finder sweep assumes the front sensor sees a robot as well as a board. In the simulator it does; the real
  sensor's wide zones may see the board behind a robot.
- The front check uses the map's board lines at the board's face (from the picture). Mapping found the front stop
  halts robots 4.5 cm nearer than assumed, so a steady offset of that size is expected in `front_checks.csv`.
- The replay's "seen 75%" counts pictures where the tracker reported a position, not whether that position was
  right. It was checked by eye against the strip of frames for 10:40:00-10:40:20 only.
