# Build report: E02 fixes + robot mapping (2026-10-21 23:50 to 2026-10-22 01:48)

**Premise.** The first real E02 session never got past setup. The camera tracker followed the wrong thing (the
spot a robot had just left, a hand, things past the board's edge), and camera-steered parking drove Center into
Right, which had not been found yet. Nick's bar: it must run without him coming back every 10 minutes.
Plan and progress: `docs/build-plans/e02-fix-build.md`. Tested on the real footage (release `e02-setup-oct21`)
and in the simulator.

**What changed (what Nick sees).**

| Before | Now |
|---|---|
| Robots had to be put on particular spots | Put them anywhere open on the board, 30 cm apart |
| Each robot found by a 12 cm drive, then parked straight away | All found first by a small turn on the spot (nothing leaves its spot), then parked one by one in an order where no spot is taken, going round the others |
| One picture of the board at the start, patched by guesswork | Nick's floor memory: the floor kept fresh wherever no robot is; robots must be robot-shaped, on the board, and (between legs) where they were told to stay |
| The camera's word was final | After every leg the camera is checked against the robot's own wheels; if they disagree, it stops and finds the robot again by a turn; Nick is called only after two failures |
| Boards found from how bright the plywood looks (fooled by the shadow) | New, `--map`: one robot feels out the board (16 rays; front sensor at the boards, edge sensors at the open edges); also the first real test of the edge sensors |
| Help: put it back on its exact spot | Help: put it anywhere open and press Enter; it finds and parks itself |
| Side camera question every time; Pi timer wrapped at 59:59 | `--no-side`; the timer counts hours |

**Data.**
- Real footage replay (`tools/experiment/e02_replay.py`), same frames for the old and new tracker:
  - Left's parking, 22:37:23-56: both trackers right.
  - Center, 22:41:12-22:44:56 (it drove into Right, then Nick's hand came in at 22:44:42): the new tracker stayed on
    Center throughout; the old one jumped onto the hand, past the board's edge.
  - Center, 22:47:35-22:52:30 (hand, robots moved by hand): the new tracker never jumped to anything (no step
    over 4 cm between frames); it lost Center while it was pressed against Right and after Nick moved it, which
    is the safe outcome (the runner now finds it again by a turn).
  - Finding a robot by a turn, tried on the session's 30 real turns of 20 deg or more: 19 found within 6 cm of
    the old tracker's position, 1 found nothing, 10 landed 6-8 cm off. All 10 were Center turning 90 deg while
    pressed against Right (both bodies moved, and the point fell between them). The first try missed 2 lone Left
    turns: the size limit was too tight for a robot that also slid while turning; widened (the 19 includes them).
    The runner turns 40 deg (the most change for a square body), twice if needed, and never guesses: a robot it
    can't find, Nick is asked to put down somewhere open.
  - What the footage showed about the failure: at 22:41 the old runner started Center at (69, 74), where nothing
    was; Center was at (87, 87).
- Simulator (whole sessions, the same E02 code the robots run): robots near their spots: 5 runs, 0 help calls,
  0 watchdog stops, tracking error median 0.7 cm (95% under 1.2 cm). Every robot on another's spot: sorted out by
  sending one to the middle first, 0 help calls. Robots scattered + mapping + find check + 4 runs: 0 help calls.
- Tests: 83 passed (9 new for this build), plus the slow dry runs above.

**Review and what it changed.** An independent code review (a separate agent that had not seen the work)
found 7 problems the tests didn't cover, all fixed and tested:
1. Mapping turned the robot round right at an open edge; a turn on the spot is not edge-guarded and a wheel
   would have gone over. It now backs off 7 cm first, and a re-find near an open edge backs off (or asks Nick).
2. A robot put aside (nobody answered) disappeared from the runner's map, so the others could drive into it. It
   now stays on the map where it was last found; in setup, with a robot nowhere to be found, it stops instead.
3. A crash in the camera thread would have frozen every position silently. The camera feed is now restarted.
4. Parking could loop for ever when every spot was taken and the drive to the middle failed. Now capped.
5. The tracker could take a parked robot for the moving one when they touched, or hold on to an old position.
6. A ray's result could be read from the wrong leg after a re-find.
7. Paths only went round the first robot in the way. Now every piece is checked, up to two detours, and "no
   clear way" waits for the others to park first.
After the fixes: 85 tests pass (11 new); the scattered-start dry run that showed problem 7 (Center waited behind
Right) now parks all three and maps with 0 help calls.

**New: live view.** While the runner works, http://localhost:8090 on the laptop shows the camera picture with
what it thinks: each robot's name, the robot it follows, things that look like robots (yellow) and other changes
(grey), the mapping rays, and its last lines. Read-only.

**Conclusion.** The setup problems from the first session are fixed in replay and in simulation; whether the
real basement agrees is the morning's first 20 minutes, which Nick watches.

**What's next.**
1. Morning session: `docs/experiments/E02-accuracy/SESSION-GUIDE.md` (one command, now with `--map`; watch the
   finding, parking, mapping and the first 3 runs; then leave).
2. After it: copy the footage first; then the statistics report.
3. Live lab: the mapping is its first self-calibration piece.

## New tests that may now be needed
- Finding by a turn on the real board with all three robots (the footage had no deliberate 40-deg turns).
- The camera-vs-wheels limits (5 cm or 35%, +5 cm on the first leg after a turn) against real legs: how often it
  cries wolf.
- Mapping on the real board: where the edge sensors stop a robot (never measured), and whether the front stop
  stops it as far from a board as assumed.
- A real-footage clip kept as a regression test, so tracker changes are always checked against the basement.

## Possible inaccuracies in what this relied on
- Chosen, not derived: the tracker's numbers (still for 12 frames to learn floor; 14 grey levels extra on unknown
  floor; robot shape 0.3-2.6x the robot's area, sides 0.4-2x; 25 s to absorb something new; hold radius 6 cm),
  the 40-deg identify turn, the cross-check limits, the 22 cm ray clearance.
- Mapping's board points assume the front stop halts the robot 7 cm + 4.5 cm short of a board; in the simulator
  the felt boards came out about 2 cm too far, so the real offset may differ too. The find check's bumps are the
  better measure of the boards; mapping is for the open edges and the travel area.
- "Old tracker right / new tracker right" in the replay is judged by eye on contact sheets, with the robot's start
  position given by hand.
- The simulator's camera is cleaner than the basement (no halo, no shadows, no hands).
- Lights on the moving robot (agreed item 4) were not done: the new tracker didn't need them, and they need a
  firmware upload to all three robots (the Right robot didn't rejoin the WiFi after the last one). Kept for the
  Live lab.

## Ideas to build next (ranked by what helps E02 and the December demo most)
1. A live view for the runner: the camera picture in a browser with where it thinks each robot is, the floor
   memory, and each camera / wheels check as it happens. Makes the first 20 minutes easy to watch, and is the
   first screen of the Live lab.
2. A real-footage regression test: a 30-second clip from tonight (the hand at 22:44:42) kept as a GitHub release
   asset, replayed by the tests whenever the tracker changes.
3. Mapping corrects its own board offset from the find check's bumps (the front stop's real stopping distance),
   so the touch map and the bumps agree.
4. The statistics report pipeline dry-run end to end on simulator sessions, so the morning's data turns into
   the report the same hour.
5. Lights on the moving robot (firmware), when there is a reason to upload firmware anyway.

**Cost.** 1 h 58 of build time (estimated 5-8 h; TIME.md in the game repo); one review agent (~150k tokens).
Stopped here rather than building the ideas above, to leave usage for the morning session's help and analysis
before the 3 PM reset.
