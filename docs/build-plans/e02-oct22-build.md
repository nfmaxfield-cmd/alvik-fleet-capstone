# E02 build after the 2026-10-22 morning session

Session: 09:51-10:43, release `e02-session-oct22` (video split in two: `cat video.part-aa video.part-ab > video.avi`).
1 run finished (Right U straight, far point "100 cm off": the camera did not follow it, so not a real result).
Nick away from 11:05; he asked: work until the usage runs out, as much in parallel as possible, then make the
logic more generic for other equipment and sensors.

## What went wrong (from the logs, the track and the footage)
1. The tracker lost moving robots for seconds and jumped: at 10:40:03, during Left's 167-deg turn, it took Right
   (7.6 cm from Right's last spot, just outside the 7.6 cm "another robot" zone) for Left; then "the camera saw it
   move 0 cm, its wheels say 10 cm" (it was watching Right). The runner gets all 25 pictures a second (track.csv).
2. Its position flipped 1.5-6 cm between two pieces of one robot, frame after frame.
3. A robot standing still with the tracker's position a few cm off was learned into the floor bit by bit; its
   outline then broke up (footage 10:40:12: the floor memory holds a ghost of Left).
4. "Something in front: turning away" looped up to 13 times: the turn legs between reset the count.
5. Every park spot is 17-20 cm from an open edge, under the 20 cm "move it in first" limit; by the x-board the
   move-in point was 1.4 cm from the board's line.
6. Center fell off the open edge (10:23): found 3.5 cm off the board, then driven blind; its edge guard had learned
   with all three line sensors over the edge ("all three on tape"), which switched the line cue off.
7. Pressing Enter with no hands nearly always worked: the help call was only a retry.
8. Right never named in the naming check (it was not parked: its parking after help did not finish).
9. One find check landed 45 cm off on the x-board (two robots' touches 93 cm apart): not explained yet.

## Done in this build
- [x] 4: obstacle legs counted on their own; back off 8 cm, then a quarter turn towards the board's middle
- [x] 5: move-in only within 14 cm of an open edge, to a point 25+ cm from both board lines
- [x] 7: three tries on its own (fresh floor, turn, heading, park) before Nick is called
- [x] 6: runner: a robot found at or over an open edge is never driven (Nick); firmware 2026-10-22b: all three
      line sensors off the plywood at the start = no forward drive that run (simulator mirrored)
- [x] 1-3: tracker: search circle grows for at most 0.6 s (was 1 s) at 30 cm/s (was 40); another robot's zone
      1.1 robot widths (was 0.8); pieces within 0.8 robot widths merged; a held robot's floor never learned within
      1.35 robot widths (1.25 while it moves); a held robot whose spot looks unchanged counts as seen; each leg tells the tracker where the robot can be (turn: 7 cm; drive d: d + 8 cm)
- [x] Live tracker had no board area at all (it followed hands at x = -47 cm): the runner now gives it the board and
      its walls; pixels beyond a wall are cut before outlines are made, so a robot against a board is not joined to
      the board's edge (replay of Left 10:39:58-10:40:55: seen 75% of frames, was 20% this morning): its outline joins the board's
      edge and fails the shape check. Next: a shape check that allows for a board beside it.
- [x] 9 explained: the firmware's own finds agree (bump points 121.5 / 124.1 cm on the y-board); the "touched at
      74.8 cm" came from camera positions at the bump moments, while the camera followed the wrong robot (1)
- [x] 8 explained: Right's parking after its help call did not finish, so it was not at its spot to be named
- [x] Front sensor as a free check (Nick): firmware 2026-10-22b adds "front=<cm>" to every finished leg; the runner
      compares it with the map (boards, other robots) and writes front_checks.csv. Recorded only, for now: one
      session's numbers set the limits before it steers anything
- [x] Finder sweep (Nick): when the camera can't see a lost robot turn after the retries, a parked robot turns in
      30-deg steps reading its front sensor; a reading 8+ cm short of the map is checked by the camera there
- [ ] Nick's ideas: robot-to-robot check (front_checks.csv already lists another robot ahead); finder robot sweep; the lost
      robot does a find to rescue itself; second camera (after the overhead tracker is solid); colour sensor (low)
- [x] Generic: outline for discussion, docs/build-plans/generic-platform-outline.md
- [x] The lost robot finds itself against the boards (rescue find): no firmware change needed (the firmware keeps
      its own pose between camera fixes; `run find` without a fix uses it). Only when last seen 15+ cm in from the
      open edges (park spots are 17+) and not moved by hand. Dry run with Left hidden from the camera for 140 s
      (E02_SIM_HIDE="Left:110:250"): 3 retries, Center's and Right's sweeps (one false hit at the y-board, checked
      by the camera and dropped), then Left found itself and was found again, no help call. With 60 s hidden: back
      on the 2nd retry. (The simulator never names robots in the naming check, before this build too.) Simulator: E02_SIM_HIDE="Left:110:170" hides a robot from the camera.
- [x] Today's footage as a regression test (tests/test_e02_realclips.py::test_tracker_on_oct22_footage)

## Before the next session
Nick uploads firmware 2026-10-22b to all three robots (same steps as this morning: Tools > Port, each robot).

## Tonight's build (after the evening session), Nick 20:06
1. **Cliff sense, measured (essential before mapping by feel):** from tonight's logs (Left's drive to the open
   corner at ~20:00, and every robot's RAW lineL/C/R and dbot_cm), the line sensors' and the front sensor's bottom
   zone's readings on plywood, at the edge and past it; set edge_thr / edge_bot_cm from them and turn both cues on
   (safe_edge 3). If one approach is too little data: a 10-minute edge test next time (tools/edge_check.py).
   Simulator test, then mapping back on.
2. **Mapping:** never aim a ray at the corner where the two open edges meet; the "camera saw it move 0 cm, wheels
   72 cm" retry loop (once a second, forever) must stop after a few tries and fall to the help ladder.
3. **Manual board borders:** answering **n** to "does the green outline sit on the board's edges?" lets Nick fix
   the outline himself (drag its corners in the live view, or click the 4 board corners in the picture) instead of
   stopping the runner.
4. **Setup questions optional:** the robots' mapping and board touches measure the board lines; the picture's
   questions become a cross-check (Nick, 19:33).
5. Check tonight's footage for the moving-robot fixes (newest picture, moving beats still, no ghosts).

## Backlog (Nick, 2026-10-23)
- **Unknown objects onto the map**: a front-sensor stop where the map is clear (no robot, no board) puts an obstacle
  on the map that paths go round; a parked robot in the way is asked to move aside. Not needed now; fold into a
  build later. Default slot: Thursday morning (last of the usage), sooner if a session shows a reason.
- **Engineer alerts in the live view** (decisions clicked, phone alert): later; assessment first
  (docs/build-plans/engineer-loop.md).
- **The robots calibrate the camera** (Nick, 2026-10-23): no corner clicks, no file; a robot drives known
  distances (wheels) in several places and directions while the camera watches its pixels move, and the pixels-to-cm
  mapping is solved from that. In the from-scratch rebuild. **Ask Nick questions before starting.**
- **Run a whole session on the Pi** (Nick, 2026-10-23, after the summary): stop-and-go driving on the robots' own
  sensors; tracking only near each robot's predicted spot; the gyro's heading picks the lock-on's rotation (1-2
  instead of 36); the full-picture detector every 2 s and at stops, possibly from a single sharp high-resolution
  frame (tests-to-run); idle checks when nothing moves. Questions for Nick first (laptop as viewer? recording on the
  Pi?). The Pi is a Pi 4 Model B.
- **Live lab graphs: say what they measure and where the data comes from** (Nick, 2026-10-23). Every chart in
  the live lab (and the live view) gets a minimal line: the quantity, its unit, and its source (e.g. "gyro scale
  per self-calibration fit, from fits.csv"). Next build.
