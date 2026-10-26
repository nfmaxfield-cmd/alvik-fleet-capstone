# Runner fixes after the 15:45 session (MIRLT): turns that fail, the edge, and sounds on the live page

**Premise.** Nick approved this build at 18:46 on 2026-10-25. It covers fixes 1-7 of the
[15:45 analysis](e02-oct25-session-1545.md) with his three choices:
- a robot whose turns fail is left out and the others carry on, with a sound on the live page;
- near an edge, the heading is found by a 3 cm crawl;
- the runner is more careful around turns only, not slower everywhere.

At 19:20 the coordinator added two items:
- firmware 2026-10-25c, which refuses go and turn legs before a compass check;
- Right's table test, which says its turn sensor works.

Everything was tested in the simulator, with no board time. The Pi was not touched. The dark strip along the bottom
edge is painter's tape on the plywood (Nick), so it is surface, not a drop. That is now noted in the 15:10 report,
in the 15:45 report's question 2, and in the runner's edge constants.

**Sources.**
- Dry runs in `logs/` (Appendix A).
- The 15:45 session's logs from the release `e02-session-20261025-154522` (`raw.csv`, `clock.csv`, `runend.csv`,
  `e02_log.txt`).
- The summary script `tools/experiment/analyses_oct25/fixes_1545/runsum.py`.
- Unit tests `tests/test_e02_1545_fixes.py` (29 new). The whole suite passes: 435 passed, 18 skipped, and 2 failures. Both are in `e02_trackproc.py` (another agent's tracker-process commit, 9ad4e43) and fail on main without this build.

## Data

### Why Right's turns failed at 15:45 (the coordinator's question)

Right's compass check at 15:58:41 ran and reported "done". But it learned the wrong turn direction. During the
"wiggle left" step, Right's gyro rate and its wheels both read a left turn, yet its fused yaw reading fell.

| Robot | Yaw change over the left wiggle | Biggest one-sample yaw step | Gyro rate during it |
|---|---|---|---|
| Left | +10.2° | +3.0° | +14.0 °/s |
| Center | +4.9° | -2.8° | +22.5 °/s |
| **Right** | **-4.6°** | **-11.3°** (78.66 → 67.34, at the wiggle's start) | +18.9 °/s |

The firmware takes the direction from the sign of the yaw change, so Right's one bad yaw sample set it backwards.
Every turn after that drove its heading the wrong way, until the timeout. Right did not restart: it had one boot id
(F292) all session, and its clock never went back. Its check was not missing either. The table test's other path
(no compass check since power-on, which firmware 25c now refuses) is a second way to the same symptom.

### The fixes and what they did in the simulator

| Fix | What changed | Simulator result |
|---|---|---|
| 1. Failing turns | A watch on every turn. 2 failed turns in a row (timed out, or its own heading moved 10°+ the wrong way) trip it, and so do 2 heading corrections over 45° after its own turns within 5 legs. A tripped robot first gets one new compass check; if that does not mend it, it is stopped, named and left out. Every compass check is now verified by a 15° check turn (timeout capped at 2 s) and redone up to 3 times. A leg refused for "no compass check" triggers the check. A robot left out where the camera does not know it is found by its lights (no turns). | **A** (gyro reads backwards 5 s after the check): caught at Right's 2nd broken turn, **13 s after its first** (22 s after the fault began); left out, with a sound event; Left and Center scored 3 runs, 0 holds. **A2** (a fault no compass check mends): caught by the check turn 5 s after the first check; 3 checks, left out 30 s later, found by its lights at (33.7, 73.6) cm against true (36, 72). **A1f** (A's fault, with the repair): its check turn caught it, the 2nd compass check mended it, and Right parked and ran on (3 runs, 0 holds). **A3** (Right loses its compass check 40 s in): its next leg was refused, the runner ran the check, and Right drove on (2 runs, 0 holds). |
| 2. Heading near an edge | Within 20 cm of an open edge, no blind drive: a 3 cm crawl at 2 cm/s with the edge sensors on, twice at most. An edge stop means it backs straight out 6 cm. The heading comes from the camera's straight-line track, only if the tracker stayed locked for 80% of the crawl. | **B**: Left placed 11.6 cm from the bottom edge facing -126°: crawl measured **-127°**. In 4 earlier runs the camera near that edge saw only 33-40% of the true motion, and once gave 68° when its lock was lost. Those runs set the lock rule |
| 3. Cut and refused turns | A turn that did not finish counts only for what its own gyro turned; the plan then sends the rest. A refused move never counts as done. | **B**: a 108.7° turn cut at 41.8° by a hold → **61.3° sent, not 108.7°**; cut again at 44.0° by the watchdog → 21.3° sent; total 109.6° |
| 4. Touches and back-outs | Edge touches are logged only with a heading measured since the last turn. A back-out within 30 s of the robot's own edge stop is always allowed. "Still going" past the line needs 1 cm of real outward motion. | Unit tests, including 15:45's 0.6 cm jitter (16:40:36), which no longer stops. **B2** (an earlier version): an edge stop on the check drive, then a straight back-out |
| 5. Caution around turns | No turn on the spot within 9 cm of an open edge: it first moves straight away to 10.5 cm. Turns within 20 cm of an edge go at half speed. After every turn, a fresh picture before the next drive; within 25 cm of an edge, that first drive is a 4 cm check. Spins that carry the robot 2.5 cm towards an edge inside the border hold. Drives in the open are unchanged. | **B**: 2 moves away (5.9 + 3.5 cm) before the turn at 7 cm; 7 slow turns; check drive after the turn. **A**: 5 slow turns, 1 check drive |
| 6. Held spot after a drive | Within 5 s of a drive, a camera fix needs a picture where the camera really saw the robot again. | Unit test only (no held spot came up in the dry runs) |
| 7. Routes and set-aside robots | A route passing a set-aside robot nearer than 18 cm is not run. | **A4**: Right set aside 10.5 cm from the square route's path; **both square runs were not run** (orders 2 and 6), and 4 other runs scored. |
| Live page | A one-tap "Turn on sounds" control. A rising chime for each hold and help call, repeated every 30 s while it waits; a falling one when a robot is left out. | Node test of the page's sound logic; the page serves `alerts` in its status |

**Explore at a slow camera.** Run C (explore, camera at 10 pictures a second, 9.9 measured) **completed and found the board**: 119.3 x 121.2 cm (the simulator's board is 121.6 x 121.3), fit 3.6 px, in 15.6 minutes. Run C2 (another seed, run alongside four other simulators: worst 0.4 pictures a second, lag up to 4.8 s) failed its sides: 7 points, rays skipped because they would pass Left, and returns cut short. It fell back to the stored calibration safely. The last build saw the same failure on a cramped start (its run B). C3_LINE

### Problems the dry runs found and fixed during the build

1. **"Carry on" at a hold let a faulty robot drive again.** When the camera did not know yet where Right was, the
   fault fell back to a hold. Now the robot is left out regardless, and found by its lights.
2. **Its lights were found once in 2 tries:** the turn attempt's lights were still on in the "before" pictures.
3. **A 10 cm blind heading drive from 17 cm ended at 9 cm.** The crawl zone grew from the analysis's 15 cm to 20 cm
   (the drive's length plus the watchdog's 9 cm, plus 1).
4. **The watchdog stopped a standing robot** as the tracker settled 5 cm after a find. It now judges only a robot
   that may be moving, and waits 1.5 s after a find.
5. **The crawl's heading came out 44° and 165° wrong** when measured end to end from a find's rough position, or
   from detector-only positions. It now uses only locked pictures.
6. **After a poor explore fit, Center was seen at x = 135 (off the board) while really 3 cm from the edge**, and
   three 15 cm heading drives took it there (run C). A camera position off the board, or within 20 cm of any side,
   now gets the crawl too.

## Conclusion

1. **Right's 15:45 fault came from a compass check that learned the wrong direction from one bad yaw sample.** It
   was not a broken robot. The runner now verifies every compass check with a check turn and redoes it, so a robot
   like Right would most likely have been mended at 15:58:45, before its first real turn. A fault that a new check
   does not mend is caught in under a minute (13 s in run A, 30 s in A2) and that robot is left out with a sound.
2. **Left's crossing is covered at each step:** the crawl instead of the blind drive, moving away before turning,
   slow turns, only the rest of a cut turn, the check drive, and the back-out always allowed.
3. **The simulator's camera is poor within about 12 cm of the bottom edge:** 3 cm off, and seeing a third of the
   motion. Fix 2's numbers were set against that. The real tracker also lagged about 2 cm there at 15:45, but how
   far it is off on the board is not known.

## What's next

1. The board: the crawl DOE rows (doe-backlog.md, Border and safety), first heading crawls facing the bottom edge
   from 8 / 12 / 16 / 20 cm.
2. Right on the bench: 10 compass checks in a row, logging RAW, to see how often its yaw jumps.
3. Firmware (the coordinator's lane): take the compass direction from the gyro rate, or the wheels, rather than the
   fused yaw. Or refuse the check when they disagree, as Right's did by 23° (+18.9 °/s against -4.6°).
4. The next real session runs with these fixes. Read `compass_checks`, `turn_faults`, `heading_crawls`,
   `turns_cut` and `alerts` in the state first.

**Questions for Nick**

*Substantive*
1. Fix 1 now tries one new compass check before leaving a robot out (my addition: the 15:45 cause would have been
   mended by it). Keep it, or leave out at once as you decided?
2. The crawl near an edge trusts the camera's direction only when the tracker stays locked. When it doesn't, the
   robot asks for help after 3 tries (run B4, an earlier version: Left put aside after a 10-minute help call). Do
   you prefer that, or the outline-based heading (no movement) as a backup?
3. Explore's fit was poor in run C (one side's points spread 23 cm, Center's path 17.6 px, still used). Should a
   fit that loose stop and ask, rather than carry on?

*Other*

4. The live page needs one tap on "Turn on sounds" each time it is opened (a browser rule). Is the phone you watch
   on the one that should chime?

## Sweep 1: new tests that may now be needed

- On the board: the crawl near the bottom edge (heading error against the camera track), and turns on the spot at
  6 / 9 / 12 cm (body sweep).
- Right on the bench: repeated compass checks with RAW, and the check turn after each.
- The tracker's accuracy within 15 cm of the open edges on the real board (the simulator's is 3 cm off there).
- A dry run with a held spot after a drive, to exercise fix 6 end to end (unit-tested only).
- The live page's sound on a phone (iOS Safari's audio unlock differs from desktop browsers).
- Explore with a poor fit: whether to refuse a 17.6 px path fit.
- A watchdog stop during a turn on a tracker re-lock (run B, 19:32:00): harmless with fix 3, but a false stop.

## Sweep 2: possible inaccuracies in what this relied on

- **The compass finding rests on one RAW sequence** (Right, 15:58:41-42, at about 20 rows a second). The jump could
  be a dropped or delayed fused-yaw update rather than a sensor fault.
- **The simulator's turn faults are models:** a gyro reading backwards, a lost compass check, and a fault that
  re-breaks after each check. Right's real body turned about 80-100° and then stood still. That case was not
  modelled.
- **The simulator's camera near the bottom edge is worse than the rest of the board** (3 cm off, a third of the
  motion). Fix 2's lock share and 1.5 cm minimum were set against it.
- **Chosen numbers** (each with a DOE note in the code; rows in doe-backlog.md):
  - turn watch: 2 / 20° / 10° / 45° / 2 in 5;
  - crawl: 20 cm, 3 cm, 2 cm/s, 6 cm back, 5.5 cm minimum, 1.5 cm, 80% locked, 25% of the wheels;
  - turns: 9 cm, 1.5 cm margin, 20 cm slow zone at 20 rpm, 25 cm check zone, 4 cm check drive, 2.5 cm spin drift;
  - compass check: 15°, 2 s, 3 tries;
  - the rest: back-out 30 s, held fix 5 s / 1.5 s, route margin 3 cm, post-find settle 1.5 s, sound repeat 30 s.
- **The repair after a tripped turn watch** (a new compass check before leaving out) is unit-tested only: in the dry
  runs every fault was caught at the compass check turn first, or by the turn watch before the repair existed (A).
- **Fix 7 (routes) and fix 6 (held spots)** are shown by unit tests and one dry run (A4), not by a real session.

## Appendix A. Dry runs

All runs are `e02.py --dry-run --auto --home --skip-naming --skip-find-check --view-port 0`, each in its own network
namespace. Folders are in `logs/`.

| Run | Folder | Extra arguments | Result |
|---|---|---|---|
| A | 190910 | `--classic-setup --dry-true-calib --dry-turn-fault Right@5 --seed 21 --max-runs 3` | Right caught at its 2nd broken turn (13 s), left out; 3 runs, 0 holds (before the compass repair was added) |
| A2 | 193046 | `--dry-start s1545 --dry-turn-fault Right@5:persist --seed 31 --max-runs 1` | the check turn failed 3 times; left out; found by its lights |
| B | 193039 | `--dry-start s1545 --dry-hold-answer a --dry-scene Left:dturn:1:gap8:0.0,Left:dturn:1:a:1.5 --seed 22 --max-runs 1` | crawl -127°; 2 moves away; a cut turn finished by its rest, twice |
| C | 191421 | `--dry-camera-fps 10 --seed 23 --max-runs 1` (explore) | explore completed: board 119.3 x 121.2 cm, fit 3.6 px, 15.6 min; then a help call for Center (problem 6) |
| A1f | 194432 | as A, final code | check turn caught it; the 2nd compass check mended it; 3 runs |
| A3 | 193844 | `--dry-turn-fault Right@40:restart --seed 33 --max-runs 2` | refused leg, then a compass check, then drove on; 2 runs |
| A4 | 193836 | as A2, `--seed 32 --max-runs 6` | both square runs not run (Right 10.5 cm from the path); 4 runs |
| C2 | 193803 | as C, `--seed 25` (machine overloaded) | explore failed its sides (7 points); fell back to the stored calibration |
C3_ROW

Eight earlier runs (19:02-19:28) were stopped or superseded as the fixes changed. They found problems 1-5 above;
their folders are in the build's scratch space, not in `logs/`.

## Appendix B. What changed in the code

- `e02_turnwatch.py` (new): the turn watch, turn_done_deg, turn_rest.
- `e02.py`:
  - fix 1: `leg_health`, `turn_fault_found`, `compass_verify`, `compass_turn_check`, `_calibrate(verify)`, the
    retry on a refused "no compass check", and `identify` (lights only for a robot left out);
  - fix 2: `heading_crawl`, `track_heading`, `edge_back_out`, `acquire_heading`;
  - fix 3: `drive_to`'s turn branch;
  - fix 4: `edge_touch_log`, `backing_out`;
  - fix 5: `turn_speed_for`, `signed_room`, `away_leg`, `spin_check`, `drive_to`;
  - fix 6: `seen_since_drive`;
  - fix 7: `route_blocked_by_aside`;
  - the watchdog: `may_move`;
  - sounds: `alert`, `help_waiting`;
  - firmware 2026-10-25c added to the expected and OK versions.
- `e02_steer.py`: `hazard`'s "still going" rule and its jump limit.
- `e02_view.py`: the "Turn on sounds" control, the chimes, and `alerts` in `/status.json`.
- `e02_sim.py`: `--turn-fault ROBOT@S[:restart|:persist]` and the `s1545` start.
- `tools/sim/fw_e02.py`: 25c's refusal.
- `e02.py` options: `--dry-turn-fault`, `--dry-hold-answer`, `--dry-camera-fps`, and `--dry-start s1545`.
