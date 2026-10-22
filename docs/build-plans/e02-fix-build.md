# Build: E02 fixes + robot mapping (started 2026-10-21 23:50 ET)

Why: the first real E02 session (2026-10-21, 22:22-22:54) never got past setup. The camera tracker followed the
wrong thing (the spot a robot just left, Nick's hand, things past the board's edge), and camera-steered parking
drove Center into Right (not found yet) twice. Nick's rule for running again: it must run without him coming
back every 10 minutes. Footage: GitHub release `e02-setup-oct21` (setup.avi = original frames 52200..94512,
22:25:55-22:54:10; frames.csv, events.csv, meta.json, logs.zip with every launch's e02_log, track.csv, robot logs).

Agreed with Nick (chat, 23:03-23:49): no extra light on the board, no printed tags (it must work with poor
warehouse-like cameras); these fixes:
1. Identify every robot in place first (a short turn on the spot; nothing leaves its spot), then park.
2. Park on the robot's own sensors (wheels + gyro after a camera fix); the camera watches and checks.
3. Cross-check the camera against the wheels: a camera position the wheels can't explain is not steered by.
4. Lights on the moving robot (solid while moving, blink ID only at identification).
5. Nick's floor memory: the floor's look kept fresh wherever no robot is, the robot's outline from it, a
   robot-shape check (size, squareness), and motion as a second check.
6. Nick watches the first 3 runs, then leaves.
Then: robot perimeter mapping (feel the boards and the open edges; the map overlaid on the camera picture),
the start of the Live lab's self-calibration and brick demo (docs/build-plans/live-lab-outline.md).
Defaults Nick left to Claude (23:49, "no preference"): done = on the footage the tracker never follows the
wrong thing for over 1 s and identifies every robot; K keeps camera steering plus the cross-check; camera and
wheels disagree -> stop, re-find by a turn on the spot, call Nick only after two failures.

## Progress (updated as each piece is pushed; a restart after the 3 PM reset carries on from here)
- [x] A. Tracker: floor memory, board-area limit, robot-shape check, motion check, vacated-spot refresh,
      change-locator for identification
- [x] B. Replay harness on the real footage + hand-checked positions; old vs new tracker
- [x] C. Runner setup: identify all in place, then heading, then park in a safe order around known robots
- [x] D. Moves on the robot's own sensors with the camera cross-check; re-find on disagreement
- [x] E. Small fixes: help wording, --no-side, Pi timer past 59 min
- [x] F. Simulator dry run + tests for all of the above
- [x] G. Robot perimeter mapping (laptop-driven rays, edge guard + front stop; map vs camera outline)
- [x] H. Review, report with the two sweeps, TIME.md row

### Results so far
- A/B (00:50): replay on the footage (tools/experiment/e02_replay.py). Left's parking 22:37:23-56: old and new both
  right. Center 22:41:12-22:44:56 (the drive into Right, then Nick's hand at 22:44:42): the new tracker stays on Center
  the whole way, the old one jumped onto the hand (off the board). Center 22:47:35-22:52:30 (Nick's hand, robots
  moved by hand): with "stand still between legs" (hold) the new tracker never jumps to anything; it loses Center
  when Center is pressed against Right and after Nick moves it (correct: the runner then re-finds it by a turn).
  Note from the footage: at 22:41 the old tracker started Center at (69, 74), where nothing was; Center was at
  (87, 87) and Right at (105, 22): Nick's placement was right, the runner's start was wrong.
