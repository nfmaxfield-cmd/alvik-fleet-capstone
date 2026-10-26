# E02: sensor trust, each robot judged on its own first (2026-10-25)

Simulator and logs only. Code: `tools/experiment/e02_trust.py` (the checks), `tools/experiment/e02.py` (runs check
feed, face readings, still-camera check during finds, flag message), `tools/experiment/e02_view.py` (live view shows the
runner's baseline and k). Tests: `tests/test_e02_trust_fix.py` (new, 26, with `tests/data/trust_runs_stretches.json`:
the real 15:45 session's and two simulator sessions' stretches), `tests/test_e02_trust.py` (2 tests and its find helper
updated). Replays: `analyses_oct25/trust_fix_replay.py` (simulator grid evidence), `analyses_oct25/trust_fix_real_replay.py`
(real sessions). Simulator reruns: `/tmp/claude-0/grid-fix/*.txt` (earlier grid: `/tmp/claude-0/grid/`).

## Premise
The simulator grid (2026-10-24/25, one degraded sensor per session) showed three cases where the trust flags were wrong:

1. **Gyro, Right, 5% jitter: not flagged.** Right read 1.1° a stretch, the others 0.3°. "Bad" meant twice a fixed healthy
   level set from the real robots (1.0°), and the simulator's gyros are far quieter than the real ones.
2. **Wheels, all three robots, 8% jitter: only Center flagged.** The camera's share of the stretch noise was taken from
   what all the robots had in common, so a problem every robot shared was blamed on the camera.
3. **Front sensor, Left, 6 cm noise: the camera was flagged 3 times, Left never.**

Then the real session at 15:45 ([e02-oct25-session-1545.md](e02-oct25-session-1545.md)), already on an early version of
this fix, flagged Center's wheels (2.3 x, by their stretches) and Left's (2.4 x, by their runs). The analysis judged both
false: the error was the same on short and long stretches, and Center's wheel scale held 0.963-0.981.

Nick's rule (13:03): "we should not compare one robot's wheels or anything else to another robot until they've each been
fully calibrated independently." So each robot's sensors are judged first on their own, against references that involve no
other robot (the camera, the board touches) and against their own history. The robots are compared only afterwards.

## What changed
- **Every check compares this session with the sensor's own baseline**: that robot's earlier sessions (sessions in
  which the sensor was flagged are left out). The Alvik numbers are only the starting baseline for a sensor with no
  history.
- **Wheels are flagged by their runs.** Each run's wheel scale on its own (the camera's length divided by the wheels'
  length). The camera alone makes the runs differ a little; how much comes from the robot's own stretches about their
  run's scale. Only a run-to-run spread far beyond that counts against the wheels. A camera error that is the same at
  every length, or the same in every run, can't produce it. The wheels' stretch rms is still shown, but no longer flags
  on its own (on the real robots it is mostly the camera's error).
- **The camera is judged on its own evidence**, never on what the robots share: finds (each counts only once the robot
  that made it has had its wheels judged, since a find's pose leans on that robot's own counting) and a robot standing
  still (not during its find).
- **Front sensor: face readings.** Right after a nose-first find, a 2° turn reads the front sensor square to the y board,
  at a distance known from the board touches (no camera). Up to 8 per robot per session, judged as their own check.
- **Second check, only after every robot has its own verdict:** a robot 3 x the healthy robots' median (stretches for
  wheels and gyro, face readings for the front sensor) is flagged too. It can add a flag, never clear or block one.
- A robot sensor flagged in a session stays flagged to that session's end, and every flag message says which check
  raised it.

## Data

**Simulator reruns, before and after** (the grid's command, `--dry-run --auto --dry-true-calib --classic-setup --hours
0.4`, on a fresh trust file: no history, the hardest case)

| Session | Flagged before (after runs) | Flagged after (after runs) | Holds before / after | KC back at start, before / after |
|---|---|---|---|---|
| Healthy baseline | none | BASE_ROW | 0 / BASE_HOLDS | 0.9 / BASE_KC cm |
| Gyro, Right, 5% jitter | none | **Right's gyro 3.2 x (9; second check, 4.8 x the others)** | 3 / 0 | 0.8 / 0.6 cm |
| Wheels, all robots, 8% jitter | Center's wheels (7) | WHEELS_ROW | 1 / WHEELS_HOLDS | 0.4 / WHEELS_KC cm |
| Front, Left, 6 cm noise | the camera 3 times, Left never | **Left's front sensor 2.9 x (8; 5.8 cm at a board, 5.7 x the others); no camera flag** | 0 / 1 | 1.0 / 1.4 cm |

<sub>Source: runner logs, `/tmp/claude-0/grid/*.txt` (before) and `/tmp/claude-0/grid-fix/*.txt` (after), read by
`analyses_oct24/grid_summary.py`. k = this session's value divided by the sensor's own baseline. The gyro and front
sessions ran on the code before the last wheel change, which doesn't touch them (no wheel flags in either). The front
session's hold came before Left was flagged (a flagged front sensor no longer holds the session; before the fix, the false
camera flag had switched those holds off). The reruns shared the machine with other agents' work (a logs DOE on both
cores, then their own simulator sessions); slow pictures caused some of the holds. KC far is not shown: the scorer
sometimes takes a run's end point as its far point (44 and 99 cm on runs that back at start were under 1 cm off).</sub>

Each intermediate rerun found something that changed the code before the final runs (kept in `/tmp/claude-0/grid-fix`
as `*.log`):
- **Wheels, 8% on every robot:** the first setup finds read 1.2-2.2 cm (a find's back-off and moves are the robot's own
  counting) and flagged a healthy camera for 12 minutes, which blocked every wheel flag. Hence: a find counts for the
  camera only once its robot's wheels are judged.
- **Front, 6 cm noise:** Left read 4 cm at a board and 9.8 cm against the map, Center and Right 1 cm at a board; one 5 cm
  bar for both kinds let it pass at 1.7 x. Hence: face readings got their own check (2 cm prior), and the front sensor
  joined the second check.
- **Wheels again (first version of the camera-corrected runs check):** Center and Right flagged, Left not. Its 8 runs
  came in pairs, three pairs within 1% of each other and one 7% away, and the median-based test ignored the bad pair.
  Hence: the mean with only the single largest run left out.

**The real sessions through the final code** (assumed healthy, so every flag is a false flag)

| Session | Old code | New, no history | New, chained (earlier sessions as history) | Wheels: runs χ²/run (Center / Left / Right) |
|---|---|---|---|---|
| 2026-10-23 morning (laptop) | Right's wheels 2.2 | none | none | 0.3 / 0.7 / 0.6 |
| 2026-10-23 evening (laptop) | none | none | none | 0.7 / 1.8 / 0.4 |
| 2026-10-24 Pi stage 1 | Center's gyro 2.7, Left's wheels 2.8 | Center's gyro 2.1, Right's gyro 2.0 | Center's gyro 2.0 | 1.0 / 3.8 / 0.2 |
| 2026-10-24 Pi stage 2 | none | Left's and Right's gyro 2.0 | none | 1.4 / 0.5 / 0.6 |
| **2026-10-25 15:45** | (live, early version of this fix: Center's and Left's wheels) | **none** | not chained | 3.1 / 1.0 / - |

<sub>Source: the downloaded session folders (paths in `analyses_oct24/night_common.py`; 15:45 from its GitHub release's
logs zip); `analyses_oct25/trust_fix_real_replay.py`. χ²/run (fine print: each run's wheel scale minus the median,
squared, divided by the scatter the camera alone would give that run, squared; averaged with the one largest run left
out) is about 1 when the camera explains the spread; the flag line is 12. Old code, chained: also Right's wheels on the
morning (1 in all). The 2026-10-23 sessions have no find evidence in this replay and no replay has still-robot evidence,
so the camera is unknown on 10-23 (nothing can be flagged there) and sits at its 0.8 cm prior early on 10-24, which lifts
the gyro's starting bar slightly.</sub>

**The simulator grid's 14 sessions through the final code** (logs only, the runner's order)

| Session | Old code, in the simulator | New, no history | New, trained (healthy baseline as history) |
|---|---|---|---|
| Healthy baseline, camera drop 30%, kept pose | none | none | none |
| Camera noise 2 cm (3 sessions) | camera 4.0 / 8.2 / 5.0 | camera 3.7 / 8.2 / 7.2 | camera 3.7 / 8.2 / 7.2 |
| Camera noise 4 cm | camera 9.5 | camera 14.1 | camera 14.1 |
| Front, Left, bias 8 cm | Left's front 2.9 | Left's front 2.9 | Left's front 6.5 |
| Gyro, Right, 5% jitter | none | **Right's gyro 2.7** (second check) | **Right's gyro 4.0** (own history) |
| Wheels, Center, 5% / 10% jitter | Center 3.8 / 4.6 | Center 2.7 / 9.8 | Center 2.7 / 9.8 |
| Wheels, all robots, 8% jitter | Center only | **Left 7.6, Center 8.1, Right 2.8** | **Left 7.6, Center 8.1, Right 2.8** |

<sub>Source: the grid's trust files (`/tmp/claude-0/grid/trust-*.json`) plus each session's stretches from `fit_data` in
its `e02_state.json`; `analyses_oct25/trust_fix_replay.py`. One grid session (camera noise 4 cm, first run) has no trust
file. The old evidence can't show the still-camera fix or the find rule, so the front-noise case is in the rerun table
only. Healthy robots' χ²/run here: at most 3.2; the jittered wheels: 36 and up.</sub>

Two more findings that changed the code: real healthy wheels' runs are 0.4-4.8% apart (the simulator's 0.1-0.5%), so a
plain spread bar false-flagged the real robots; and pooled with a healthy history, a camera with 4 cm of noise was not
flagged at all (hence: the camera judged on this session's evidence).

**Face readings** (healthy baseline rerun): 8 per robot, all within 1.3 cm of the distance by the board touches. Front
evidence per robot per session went from 2-6 readings to 11-13.

## Conclusion
- The three grid failures are fixed in the simulator without a false flag on the healthy baseline: Right's gyro is caught
  (second check with no history, its own history once trained), every robot's jittered wheels are caught by their own
  runs, and Left's noisy front sensor is caught with no false camera flag.
- The 15:45 session's false wheel flags do not come back with the final code (Center χ²/run 3.1, Left 1.0, line 12), and
  no real session flags any wheels.
- On the real sessions the new code gives 5 false flags against the old code's 4. All 5 are gyro flags at 2.0-2.1 x on
  the Pi sessions, where the camera setup had changed since the laptop sessions. The gyro is now the sensor closest to
  false alarms; the same camera-corrected test does not separate it (real healthy gyros reach 10, a simulator fault 17).

## What's next
1. **Gyro flag line on real data.** The 5 false flags sit at k = 2.0-2.1. Two ways: the backlog's swap test at 2 / 2.5
   for the gyro, or a history kept per camera setup (laptop or Pi). Question for Nick.
2. **First real session on the final code:** its face readings set the real healthy level at a board (now a chosen 2 cm);
   its runs add a sixth real session to the χ² check.
3. **Trained simulator sessions:** one per fault with the healthy baseline's trust file as history, to confirm the trained
   replay in the loop.
4. Rows in `docs/doe-backlog.md` (Sensor trust): baseline floors, run spread (now a fallback), the runs check's line and
   camera floor, the second check's 3x, evidence counts, shortest run, face readings (count, prior), finds before they
   count, plus a note on the flag line.

## Sweep 1: new tests that may now be needed
- A trained simulator session per fault (gyro, wheels, front), from the healthy baseline's trust file.
- Wheel jitter of 2 and 4% in the simulator: the smallest fault the runs check catches.
- A wheel fault that is not per run (random slip inside a run): the runs check would read it as camera scatter.
- A camera-setup change between sessions (laptop history, then Pi): how many sensors get flagged, and whether a
  per-setup history fixes it.
- Front bias with history: the "changed" flag (from e02_front) has unit tests but no simulator session yet.
- A backing-in session (`--switches-back`): it takes no face readings (the robot ends facing away from the board).
- A camera fault that only shows at the finds (a distortion in one area): until the robots' wheels are judged, only the
  still-robot check speaks for the camera.

## Sweep 2: possible inaccuracies in what this relied on
- **Healthy-real assumption:** the real replay counts every flag as false; a real fault there would make the new code
  look worse than it is. The 15:45 analysis's judgement (Center's wheels healthy) is taken as given.
- **Camera errors independent between stretches of one run:** the runs check assumes it. A slow camera shares some error
  within a run (15:45 Center's χ² of 3.1 is above the 1 that independence gives); the line of 12 leaves room for that.
- **Simulator jitter is one random factor per pair of runs**, and every simulator fault is per run. A real fault that
  drifts slowly, or slips inside a run, would look different.
- **Chosen numbers** (DOE rows in the backlog): runs line χ²/run 12 and camera floor 0.3 cm; baseline floors (0.25 cm,
  0.3°, 2 cm, face 1 cm); face prior 2 cm; second check 3x; evidence counts (4 runs, 3 fits, 6 map readings, 4 face
  readings, 4 finds); shortest run 40 cm; face readings 8. DEGRADED_K 2 and RECOVERED_K 1.5 are unchanged and still chosen.
- **The Alvik prior** (camera 0.8 cm, wheels 0.6 cm, gyro 1.0°, front 5 cm) is still from the same 2026-10-23/24
  sessions the real replay uses.
- **The find's pose leaning on the wheels** is inferred from the evidence (setup finds 1.2-2.2 cm off only with wheel
  jitter), not read from the firmware's find code.
- **Machine load** during the reruns (other agents' work): timing and holds may differ on a quiet machine.

## Appendix: the checks
| Sensor | Own checks (no other robot) | Flags from |
|---|---|---|
| Camera | finds against the board touches (once the robot's wheels are judged); scatter standing still (not during a find) | either, against the Alvik 0.8 cm |
| Wheels | runs: run-to-run scale spread against the camera's own scatter (χ²/run); stretches (rms per fit, shown only) | runs; the second check |
| Gyro | stretches (heading rms per fit) | stretches against its own history (prior 1.0° with the camera's share); the second check |
| Front | against the map (camera's share off); at a board (face readings); "changed" from e02_front | any of them; the second check (face readings) |

While the camera is unknown or flagged, no wheel, gyro or front sensor is newly flagged: all their checks use the
camera. KC's robot uncertainty and the leg check are widened by a robot's wheel k only while its wheels are flagged.
