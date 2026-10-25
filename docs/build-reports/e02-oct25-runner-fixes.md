# Runner fixes after the 15:10 session, the camera kept fast, and open edges told from objects beyond them (MIRLT)

**Premise.** Nick approved this build at 16:15 on 2026-10-25, with one more item added at about 17:05. Part 1 is
fixes 1-5 of the [15:10 diagnosis](e02-oct25-session-1510.md). Part 2 keeps the camera fast. Across the day's
sessions the pictures handled a second fell from 24 to 15 to 10, and a robot crossed home's 8 cm border. Part 3 is
the added item: explore must tell an open edge from an object standing beyond it. At 15:10 a light object past the
bottom edge read as "board" twice. Everything was tested in the simulator, plus a replay of 15:10's own data, with no
board time. The Pi was not touched.

**Sources.** Seven dry runs (`logs/e02-20261025-1633*` to `-18*-dryrun` in the build's worktree; the commands are in
Appendix A). The 15:10 session's `e02_state.json` and `perf.csv`, and two of its video frames. Scripts are in
`tools/experiment/analyses_oct25/runner_fixes/`: `runsum.py` sums up a run, `border_replay.py` replays the border's
old and new rule on the same tracks, and `fitbench.py` is the camera-against-fit replay. There are 24 new unit tests
in `tests/test_e02_runner_fixes.py`. The whole suite passes: 365 tests, 17 skipped.

**The 15:10 re-creation** (`--dry-start s1510`: the robots on 15:10's spots, `--home`, plus `--dry-scene` events):
- The border's calibration is shifted, so that line-sensor edge stops leave the robot's centre past the line.
- A hold during Right's 2nd ray, answered `o`.
- The camera loses Left for 3 s in its first 175° turn.
- A hold in Center's 2nd 175° turn, answered `c`.

## Data

| Fix | 15:10 (real) | Simulator / replay after the fix |
|---|---|---|
| 1. Border: which side of the line, and moving in or out | 6 of 9 border trips false; 4 holds, 2 cut drives | Same tracks, both rules (run A, border shifted -14 cm so edge stops end 4-5 cm past the line): edge back-offs tripped **7 of 20 with the old rule, 0 with the new**. Returns and homes: 2 real approaches tripped under both rules. Live (runs C, C2, border -14 cm): **0 border holds** |
| 2. A left-out robot drops out | Right's 6 unsent rays logged as "nothing met"; 2 gap rays given to Right, counted as driven, fallback skipped | Runs A and B: Right left out at its 2nd ray; "its last 4 rays not driven" logged, 0 sent; gap rays went to Left and Center (7 in A). A's board: 118.1 x 122.4 cm (true 121.6 x 121.3) |
| 3. Finish a cut turn | Left's 175° turn stopped at 72°, then a 56 cm wrong-way drive | A: Center's turn cut at 89.0°, then finished with 86.0° more. B: 89.9° + 85.1° |
| 4. Robots kept apart | Left's home leg pushed Center for 7 s | Parked-robot stops while exploring: 2 per run (A's first: true gap 16.1 cm, closing). Legs checked: A 3, B 7, C 4 (returns cut short or not driven, 1 home leg kept still). Closest true approach in A and B: 12.8 cm centre to centre. **Run C: 6.7 cm (a touch), found and fixed below** |
| 5. Not "lost" while turning | 3 lost holds during turns (camera gaps 1.8-3.0 s) | A, B: 3 s camera gap in Left's 175° turn, **no hold** |

**Part 2a: what slows the camera, and moving the fit out of its way.** The 15:10 perf data splits into two causes:
- **Explore's fits.** These were the two worst spells (3.8 and 5.4 pictures a second).
- **The tracker's own work while it follows a moving robot.** Through the rest of explore it ran at 15.8 a second
  and 59 ms a picture, busy 93 % of the time. After explore it ran at 24 a second and 35 ms.

The new stall watch (`e02_stall.py`, on in every session) shows where the camera's own time goes in the simulator.
Of the camera thread's sampled time, **68 % is the lock-on's template search** (`e02_lock.py _best`).

The fit now runs in a helper process (`e02_offload.py`) at lower priority. `--fits-in-runner` keeps the old way for
comparison.

| Fit replay (`fitbench.py`): 15:10's 151 stops, the real tracker on 15:10 frames | Fit in the runner | Fit in the helper |
|---|---|---|
| Pictures offered 25 a second (3 pairs) | 24.0 / 21.9 / 23.5 a second during the fit | 21.0 / 24.2 / 23.1 |
| Camera loop flat out, as the Pi's was (2 pairs) | **100 → 59 and 105 → 61** (-41 %) | **94 → 93** (-1 %) and 54 → 81 (machine busy) |
| How long the fit took | 24-25 s | 36-68 s (low priority, 2 shared cores) |

The fit replay sends pictures as fast as the loop takes them, or at 25 a second; it measures pictures handled a
second during the fit, as in perf.csv. With pictures at 25 a second this machine has spare time, so neither way slows
it. Flat out, as on the Pi, the fit in the runner takes 41 % of the camera's speed and the helper takes about none.

In the dry runs the simulator's camera stops at 15 a second, so neither way changed the pictures handled (median
14.7 with the helper and 15.0 in the runner). Only the replay shows the difference.

**Part 2b: the speed guard.** The guard works from the camera's lag: a picture's age plus one picture interval. A
robot's border moves out by how far that robot drives in the lag beyond the budget. The budget is 0.2 s at home and
0.56 s away; both come from the borders' own notes. At 15:10's worst second (lag 0.59 s) a robot at 10 cm/s gets
3.9 cm more, and a 4 cm/s crawl gets 1.6 cm. In the band, a robot driving towards the edge is stopped without a hold.
No robot in the band is sent a new move until the lag is back within budget for 2 seconds in a row.

Run C slowed the camera on purpose to about 4 a second, 20 s in every 45 (25 slow spells):
- 12 moves waited, 1.5-40 s each, and every one went once the camera caught up.
- No border holds, no robot off the board, and explore found the board at 120.1 x 121.7 cm.
- But Center's ray drove into parked Right (6.7 cm centre to centre). The watchdog's parked-robot check wanted
  6 pictures in 0.5 s, which means 12 a second, so at 4 a second it never looked. It now uses 1.2 s and 4 pictures
  at a slow rate. The parked robots' clearance also widens with the lag, like the border. Run C2 checks this: [C2].

**Part 3: an object beyond an open edge.** When the front stop ends a ray, the robot creeps on to find out what it
met:
- How it creeps: 3 cm steps, its edge guard on, the front stop moved in to 3 cm. The speed is the smaller of 2 cm/s
  and 1 cm per camera lag.
- If the edge sensors find the edge first, it is an **open edge**, and the object is logged as "beyond the edge, X cm
  on".
- If it reaches the object with floor under it, it is a **blocked side**.

The simulator models an object past an open edge as a brick standing off the plywood, which works. It cannot model
a board standing out from the plywood with floor under the gap: its floor ends at the plywood. The nearest test it
can do is a brick on the plywood 4 cm in from an open edge.

| Run | Creeps | Open edge (object beyond) | Blocked | Neither | Explore |
|---|---|---|---|---|---|
| D1, brick 5-10 cm past the x = 0 edge, front stop 2 cm | 9 | 1 (object about 6 cm on) | 7 | 1 | failed (sides) |
| D2, brick 4-8 cm inside the y = 0 edge, front stop 2 cm | 9 | 1 (at the brick's end) | 7 | 1 | failed (fit 8.9 px) |
| D1b, the same at 3 cm | [D1b] | | | | |
| D2b, the same at 3 cm | [D2b] | | | | |

At 2 cm both D runs' camera fits went bad: each robot's path fitted 6-20 px, against 1-4 in run A, and wheel scales
came out 0.89-1.10. A slanting creep touches with the bumper's corner before the front sensor's centre reads 2 cm, and
the wheels slip. The creep stop is now 3 cm, which leaves the bumper 1.5 cm clear. At 15:10 the edge lay 0-3 cm ahead
of the line sensors when the robot stopped, so 4 cm of creep still reaches it.

## Conclusions

1. **Fixes 1-5 do what the 15:10 report asked**, each shown on its own 15:10 case. The border no longer trips on a
   robot coming back in; on the same simulated tracks the old rule would have held 7 back-offs. A left-out robot is
   out of explore completely, and its unsent rays are not logged as driven. A turn a hold cut short is finished.
   Home and return legs are checked like rays. A turning robot gets 3.5 s before it is called lost.
2. **The fit is out of the camera's way.** With the camera flat out, as on the Pi, the fit in the runner takes 41 %
   of the camera's speed and the helper takes about none. The cost: the fit itself takes longer on 2 shared cores
   (36-68 s against 24 s). The Pi has 4 cores.
3. **The other big cost is the camera's own tracking** (68 % in the lock-on's search). Moving it out of the camera's
   thread is the camera loop itself, a bigger redesign; it is a question below. Meanwhile the speed guard covers it.
4. **The speed guard works on a slow camera, and it found a real gap.** The watchdog's parked-robot check was blind
   below 12 pictures a second, and 15:10 spent minutes at 10-14. That is fixed; run C2 is the check.
5. **Telling an open edge from an object beyond it works call by call**, in the simulator's terms. But at 2 cm the
   creep spoiled the camera fit. At 3 cm: [D-conclusion].
6. **Part 2c, what else slowed the 16:00 session: its logs are not uploaded yet.** The releases list ends at
   `e02-session-20261025-151004`, checked at 17:00 and 18:0x. So this could not be answered.

## What's next

1. Nick: upload the 16:00 session (the logs zip, and the recorder's small zip). Its `stalls.csv` and `threads.csv`
   will not be there, because the build came after it. Its perf.csv and the tracker's sources will show whether it
   was the tracking cost, as at 15:10, or something new.
2. The next real session is the first with the stall watch on. Read `stalls.csv` and `threads.csv` beside
   `perf.csv`.
3. The crawl DOE (DOE-explore-crawl.md) gains the creep's stop distance and speed. It also gains an object at
   0 / 5 / 10 cm past an open edge, and, on the board, a board 0-5 cm out from the plywood.
4. Before the next board session: the 15:10 report's setup items, plus this build's questions.

**Questions for Nick**

*Substantive*
1. The creep goes to 3 cm from what the front sensor sees, not to "the usual stop distance" (7 cm). At 7 cm the floor
   in front of the robot is not checked: at 15:10 the edge was 0-3 cm ahead of the line sensors at that stop. My
   assumption: closer, slower creeping is what you meant. If you want 7 cm kept, the 15:10 object would still read
   "board", and the alternative is a rule on the result instead (a "board" on a side whose other contacts are edges
   is dropped).
2. The tracker's own cost (lock-on search, 68 % of the camera's time) is the next limit on the Pi. Shall the camera
   loop move into its own process (a redesign of how the runner talks to the tracker, about 3-4 h)? Or try the
   cheaper route first: search a smaller area, or every other picture while a robot stands still.
3. The speed guard stops robots near an open edge and makes moves wait (at most 60 s) while the camera is slow. It
   does not slow robots down. Should it also drive slower while the camera is slow, at the cost of longer runs?

*Other*

4. The time tracker has its last block of rows twice, from another chat. I left it as it is; shall I remove the
   copy?

## Sweep 1: new tests that may now be needed

- A real session with the stall watch on: check the profile matches the Pi's own costs, not the simulator's.
- On the board: the creep at an open edge with an object 5-10 cm beyond it, and at a board standing a few cm out
  (the simulator cannot model the gap's floor).
- The speed guard against a real `--load` on the Pi (an upload), not only `--dry-slow-camera`.
- A creep that ends against a board at a slant: the camera checks that the robot really moved, so the fit is not
  spoiled when the wheels slip.
- The fit helper on the Pi: start-up time (it imports numpy and OpenCV) and how long the fit takes there.
- Explore when a cut stop is not seen by the camera: the leg's first part is then missing from the path. This was
  true before this build too.
- `explore_home` re-aimed beside its start (the 7.5 / 15 cm targets): seen in unit tests, not yet in a dry run.

## Sweep 2: possible inaccuracies in what this relied on

- **Lag = picture age + one picture interval** is a model of how long a robot drives unseen. The watchdog's own 0.1 s
  tick and the WiFi stop are left out.
- **The budgets (0.2 / 0.56 s) are copied from the borders' DOE notes**, which are themselves chosen, not measured.
- **The fit replay ran on this 2-core machine**, shared with another agent's simulator grid, and with an x86 tracker
  about 2x faster than the Pi's. Flat-out rates (100 a second) are not the Pi's. Only the relative drop is used, and
  one pair (54 → 81) is noise.
- **The simulator's camera tops out at 15 a second**, so the dry runs could not show part 2a. The replay stands in.
- **The border replay's edge stops are the simulator's**, shifted 14 cm to match 15:10's "4-5 cm past the line". The
  real cause (the stored outline about 1 cm inside the plywood, and where the robots stop) is not simulated.
- **The front reading at a front stop is not in the real robots' leg record** ("obstacle" only), so the creep takes
  7 cm (safe_front_cm) as the starting distance. The simulator adds `front=`.
- **"Blocked side" in the simulator was a brick on the plywood**, not a board out from it with floor under the gap.
- **Chosen numbers** (each has a DOE note in the code and a row in doe-backlog.md):
  - creep: 3 cm stop, 3 cm steps, 2 cm/s, 1 cm per lag, 15 cm cap;
  - speed guard: 0.5 / 15 cm, 60 s wait, 2 rows;
  - legs: shortening 0.75 / 0.5 / 0.25, home re-aim 7.5 / 15 cm;
  - resume limits: 3 times, 1 cm, 2°;
  - the helper's priority 10;
  - the stall watch's 0.1 s / 0.02 s;
  - the slow-camera hazard's 1.2 s / 4 pictures.

## Appendix A. Runs

All runs are `e02.py --dry-run --auto --home --skip-naming --skip-find-check --max-runs 1 --hours 0.3 --view-port 0`,
each in its own network namespace (`unshare -n`), so that they could run beside another agent's simulator grid.

| Run | Folder | Extra arguments | Result |
|---|---|---|---|
| A | 163324 | `--dry-start s1510 --dry-guard-shift=-9/-9 --dry-scene Right:ray:2:o,Left:bigturn:1:gap3:0.8,Center:bigturn:2:c:1.5 --seed 11` | board found after gap rays; 2 holds (both injected) |
| B | 164926 | as A + `--fits-in-runner` | explore failed (sides, 16 points): the cramped 15:10 layout, 1 of 2 |
| C | 170419 | `--dry-start s1510 --dry-guard-shift=-14/-14 --dry-slow-camera 45/20/200/100 --seed 11` | board found; 12 guard waits; the 6.7 cm touch |
| D1 | 172639 | `--dry-start parks --dry-bricks=-8,60,100,5,90 --dry-bricks-now --seed 12` | creep front stop 2 cm |
| D2 | 172440 | `--dry-start parks --dry-bricks=60,6,30,4,0 --dry-bricks-now --seed 13` | creep front stop 2 cm |
| D1b, D2b, C2 | [folders] | as D1, D2, C, after the 3 cm creep stop and the slow-camera hazard fix | [results] |

-9 cm in A was not enough to put the simulator's edge stops past the line (they end 8.7-10.3 cm in). The border
replay uses -14 cm.

## Appendix B. What changed in the code

- `e02_steer.py`: `inward_cm` (a signed distance). `hazard` works with no board (parked robots only), and at a slow
  camera (1.2 s, 4 pictures).
- `e02.py`, the five 15:10 fixes:
  - `border_check`, `in_border_band` and `border_shadow` use the signed distance (fix 1).
  - `lost_why` holds the watchdog's lost check, with `RECHECK_S` for turns (fix 5).
  - `explore_leg` finishes a leg a hold or the speed guard cut short; `_explore_leg_once` is the old body (fix 3).
  - Left-out robots: `explore`, `explore_rays`, `explore_one_ray`, `explore_home`, `explore_gap_rays`,
    `explore_stop` and the fallback (fix 2).
  - `explore_ray_blocked(length_cm)` and `explore_clear_leg` check the return and home legs, with the home leg
    re-aimed. The watchdog's parked check runs while exploring (fix 4).
- `e02.py`, the camera, the speed guard and the creep:
  - `heavy` sends `EX.fit` and `EX.robot_check` to the helper.
  - `camera_lag_s`, `speed_guard_cm`, `speed_guard_check`, `speed_guard_wait` and `in_band` are the speed guard.
  - `explore_creep` is the creep.
- `e02.py`, test hooks: `--dry-scene`, `--dry-guard-shift`, `--dry-slow-camera`, `--dry-bricks-now`, the
  `s1510` start, `--fits-in-runner` and `--no-stall-watch`.
- `e02_explore.py`: a stop's measured `reach_cm` places its contact.
- `e02_sim.py`: the `s1510` start.
- New modules: `e02_offload.py` (the helper process) and `e02_stall.py` (the stall watch: `stalls.csv`,
  `threads.csv`, and `stall_summary` in the state).
