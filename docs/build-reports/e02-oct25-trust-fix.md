# E02: sensor trust, each robot judged on its own first (2026-10-25)

Simulator and logs only. Code: `tools/experiment/e02_trust.py` (the checks), `tools/experiment/e02.py` (runs check
feed, face readings, still-camera check during finds, flag message), `tools/experiment/e02_view.py` (live view shows the
runner's baseline and k). Tests: `tests/test_e02_trust_fix.py` (new, 24), `tests/test_e02_trust.py` (2 tests and its find helper updated).
Replays: `analyses_oct25/trust_fix_replay.py` (simulator grid evidence), `analyses_oct25/trust_fix_real_replay.py` (real
sessions). Simulator reruns: `/tmp/claude-0/grid-fix/*.txt` (earlier grid: `/tmp/claude-0/grid/`). Commits 559db42,
8d22f17, 95c8369, 74cc2c8, 28ffa16 and the one with this report.

## Premise
The simulator grid (2026-10-24/25, one degraded sensor per session) showed three cases where the trust flags were wrong:

1. **Gyro, Right, 5% jitter: not flagged.** Right read 1.1° a stretch, the others 0.3°. "Bad" meant twice a fixed healthy
   level set from the real robots (1.0°), and the simulator's gyros are far quieter than the real ones.
2. **Wheels, all three robots, 8% jitter: only Center flagged.** The camera's share of the stretch noise was taken from
   what all the robots had in common, so a problem every robot shared was blamed on the camera.
3. **Front sensor, Left, 6 cm noise: the camera was flagged 3 times, Left never.**

Nick's rule (13:03): "we should not compare one robot's wheels or anything else to another robot until they've each been
fully calibrated independently." So each robot's sensors are judged first on their own, against references that involve no
other robot (the camera, the board touches) and against their own history. The robots are compared only afterwards.

## What changed
- **Every check compares this session with the sensor's own baseline**: that robot's earlier sessions (sessions in
  which the sensor was flagged are left out). The Alvik numbers are only the starting baseline for a sensor with no
  history.
- **Wheels: a new runs check.** Each run's wheel scale on its own (the camera's length divided by the wheels' length).
  A healthy wheel gives about the same scale every run; a wheel that miscounts by a different amount every run does not.
  No other robot is needed.
- **The camera's share of a stretch comes from the camera's own evidence**, never from what the robots share. The
  camera is judged on this session's evidence, and a find counts for the camera only once the robot that made it has
  had its wheels judged (a find's pose leans on that robot's own counting).
- **Front sensor: face readings.** Right after a nose-first find, a 2° turn reads the front sensor square to the y board,
  at a distance known from the board touches (no camera). Up to 8 per robot per session, judged as their own check.
- **Second check, only after every robot has its own verdict:** a robot 3 x the healthy robots' median (stretches for
  wheels and gyro, face readings for the front sensor) is flagged too. It can add a flag, never clear or block one.
- **Still-camera check skips a robot during its find.** Case 3's camera flags came from this, not from the front
  sensor: each setup find was measured as a "still" robot, 4.8-7.2 cm of scatter.
- A robot sensor flagged in a session stays flagged to that session's end, and every flag message says which check
  raised it.

## Data

**Simulator reruns, before and after** (the grid's command, `--dry-run --auto --dry-true-calib --classic-setup --hours
0.4`, on a fresh trust file: no history, the hardest case)

| Session | Flagged before (after runs) | Flagged after (after runs) | Holds before / after | KC back at start, before / after |
|---|---|---|---|---|
| Healthy baseline | none | **none** | 0 / 1 | 0.9 / 1.2 cm |
| Gyro, Right, 5% jitter | none | **Right's gyro 3.2 x (9; second check, 4.8 x the others)** | 3 / 0 | 0.8 / 0.6 cm |
| Wheels, all robots, 8% jitter | Center's wheels (7) | **Left's wheels 4.7 x (4), Center's 2.1 x (8), both by their own runs**; Right not (1.8 x) | 1 / 2 | 0.4 / 1.8 cm |
| Front, Left, 6 cm noise | the camera 3 times, Left never | FRONT_ROW | FRONT_HOLDS | FRONT_KC |

<sub>Source: runner logs, `/tmp/claude-0/grid/*.txt` (before) and `/tmp/claude-0/grid-fix/*.txt` (after), read by
`analyses_oct24/grid_summary.py`. k = this session's noise divided by the sensor's own baseline. The reruns shared the
machine with other agents' work (a logs DOE on both cores, then their own simulator sessions); the holds after the fix
are the leg check before any wheel evidence (1), the virtual border with Right's unflagged jittered wheels (1) and a
lost picture while the loop ran at 9.9 a second against 15 (1). One KC run (Center, L route) scored 44 cm at the far
point because the scorer took its end point; its KC stops show it reached the far corner; it is left out of the KC
numbers here.</sub>

Two intermediate reruns found problems that changed the code before the final runs above (kept in `/tmp/claude-0/grid-fix`
as `*-previousmodule*` and `*-findrule*`):
- **Wheels, 8% on every robot:** the first setup finds read 1.2-2.2 cm (a find's back-off and its moves between the
  boards are the robot's own counting) and flagged a healthy camera for 12 minutes, which blocked every wheel flag.
  Hence: a find counts for the camera only once its robot's wheels are judged.
- **Front, 6 cm noise:** no false camera flag any more, but Left not flagged either. It read 4 cm at a board and 9.8 cm
  against the map, Center and Right 1 cm at a board, and one 5 cm bar for both kinds let it pass at 1.7 x. Its noisy
  readings caused 2 holds (the runner took them for a wrong camera position). Hence: face readings got their own check
  and the front sensor joined the second check. The final code flags Left on that session's evidence (6.4 x the others).

**All 14 grid sessions' evidence replayed through the new code** (logs only, in the runner's order)

| Session | Old code, in the simulator | New, no history | New, trained (healthy baseline session as history) |
|---|---|---|---|
| Healthy baseline, camera drop 30%, kept pose | none | none | none |
| Camera noise 2 cm (3 sessions) | camera 4.0 / 8.2 / 5.0 | camera 3.7 / 8.2 / 7.2 | camera 3.7 / 8.2 / 7.2 |
| Camera noise 4 cm | camera 9.5 | camera 14.1 | camera 14.1 |
| Front, Left, bias 8 cm | Left's front 2.9 | Left's front 2.9 | Left's front 6.5 |
| Gyro, Right, 5% jitter | none | **Right's gyro 2.7** (second check) | **Right's gyro 4.0** (own history) |
| Wheels, Center, 5% / 10% jitter | Center 3.8 / 4.6 | Center 2.1 / 3.5 (runs) | Center 2.1 / 3.5 |
| Wheels, all robots, 8% jitter | Center only | **Left 3.4, Center 4.5** (runs) | **Left 3.4, Center 4.5, Right 2.7** (Right: stretches against its own history) |

<sub>Source: the grid's trust files (`/tmp/claude-0/grid/trust-*.json`) plus each session's per-run wheel scales from
`fit_data` in its `e02_state.json`; `analyses_oct25/trust_fix_replay.py`. One grid session (camera noise 4 cm, first run)
has no trust file and is left out. The old evidence can't show the still-camera fix (it holds the scatter from the
finds) or the find rule (it doesn't say which robot made each find), so the front-noise case is in the rerun table
only.</sub>

Before the camera was judged on this session's own evidence, the trained replay showed the flaw this rule is about:
pooled with a healthy history, a camera with 4 cm of noise was not flagged at all, and 2 cm only at 2.0 x.

**The same checks on the real robots' sessions** (assumed healthy, so every flag is a false flag)

| Session | Old code | New, no history | New, chained (earlier sessions as history) | Run-to-run wheel scale spread (Center / Left / Right) |
|---|---|---|---|---|
| 2026-10-23 morning (laptop) | Right's wheels 2.2 | none | none | 0.8 / 1.1 / 2.7% |
| 2026-10-23 evening (laptop) | none | none | none | 2.7 / 3.6 / 3.1% |
| 2026-10-24 Pi stage 1 | Center's gyro 2.7, Left's wheels 2.8 | Center's gyro 2.1, Right's gyro 2.0 | Center's gyro 2.0 | 1.3 / 2.6 / 0.4% |
| 2026-10-24 Pi stage 2 | none | Left's and Right's gyro 2.0 | none | 4.8 / 4.2 / 2.0% |

<sub>Source: the downloaded session folders in `/tmp/claude-0` (paths in `analyses_oct24/night_common.py`);
`analyses_oct25/trust_fix_real_replay.py`. Old code: chained, it also flags Right's wheels on the morning (1 in all).
Spread = median distance from the median x 1.48, divided by the median (robust: one odd run doesn't move it). The
2026-10-23 sessions have no find evidence in this replay and no replay has still-robot evidence, so the camera is
unknown on 10-23 (nothing can be flagged there) and sits at its 0.8 cm prior early on 10-24, which lifts the gyro's
starting bar slightly.</sub>

Two findings from the real replay changed the code before the final reruns:
- **Real healthy wheels' runs are 0.4-4.8% apart (median 2.6%), against the simulator's 0.1-0.5%.** A 1% starting bar
  flagged all three robots on Pi stage 2. The runs check's bar is now the real median, 2.6%, and a robot's own history
  can raise it but never lower it.
- **An "every robot alike" exemption I had added hid Right's 8% wheels** in the trained replay, and the real all-alike
  rises (Pi stage 2: 1.6-1.9 x) stay under the flag line without it. It was removed.

**Face readings** (healthy baseline rerun): 8 per robot, all within 1.3 cm of the distance by the board touches. Front
evidence per robot per session went from 2-6 readings to 11-13.

## Conclusion
- Both grid failures are fixed in the simulator without a false flag on the healthy baseline: Right's gyro is caught
  (second check with no history, its own history once trained), and each robot's wheels are judged by their own runs.
- The front-noise case's camera flags were the still-camera check measuring a robot during its find; that is fixed.
  FRONT_CONCLUSION
- On the real sessions the new code gives 5 false flags against the old code's 4. All 5 are gyro flags at 2.0-2.1 x
  on the Pi sessions, where the camera setup had changed since the laptop sessions. The gyro is now the sensor closest
  to false alarms.
- With no history, a mild fault can still pass: Right's 8% wheel jitter gave runs 2.5-3.5% apart, inside the real
  robots' healthy range. Once a robot has a history, its stretches catch it (trained replay).

## What's next
1. **Gyro flag line on real data.** The 5 false flags sit at k = 2.0-2.1. Two ways: the backlog's swap test at 2 / 2.5
   for the gyro, or a history kept per camera setup (laptop or Pi), so a camera change doesn't make every robot's history
   wrong. Question for Nick.
2. **First real session:** its face readings set the real healthy level at a board (now a chosen 2 cm), and its runs add
   to the real run-to-run spread (2.6% from 12 robot-sessions). Both are logged automatically.
3. **Trained simulator sessions.** One session per fault with the healthy baseline's trust file as history, to confirm
   the trained replay in the loop.
4. Rows in `docs/doe-backlog.md` (Sensor trust): baseline floors, run spread, the second check's 3x, evidence counts,
   shortest run, face readings (count, prior), finds before they count, plus a note on the flag line.

## Sweep 1: new tests that may now be needed
- A trained simulator session per fault (gyro, wheels, front), from the healthy baseline's trust file.
- A camera-setup change between sessions (laptop history, then Pi): how many sensors get flagged, and whether a
  per-setup history fixes it.
- Front bias with history: the "changed" flag (from e02_front) has unit tests but no simulator session yet.
- A backing-in session (`--switches-back`): it takes no face readings (the robot ends facing away from the board), so
  its front evidence stays slow.
- A camera fault that only shows at the finds (a distortion in one area): until the robots' wheels are judged, only the
  still-robot check speaks for the camera.
- Real-session check that the 2° face-reading turn after a find doesn't spoil the next run's start (the exact find pose
  is kept; only the heading turns 2°).

## Sweep 2: possible inaccuracies in what this relied on
- **Healthy-real assumption:** the real replay counts every flag as false; a real fault there would make the new code
  look worse than it is.
- **Real run spread from 12 robot-sessions**, two camera setups mixed: measured, but thin.
- **Chosen numbers** (DOE rows in the backlog): baseline floors (0.25 cm, 0.3°, 2 cm, face 1 cm), face prior 2 cm,
  second check 3x, evidence counts (4 runs, 3 fits, 6 map readings, 4 face readings, 4 finds), shortest run 40 cm, face
  readings 8. DEGRADED_K 2 and RECOVERED_K 1.5 are unchanged and still chosen.
- **The Alvik prior** (camera 0.8 cm, wheels 0.6 cm, gyro 1.0°, front 5 cm) is still from the same 2026-10-23/24
  sessions the real replay uses.
- **Simulator jitter is per run** (a new random factor for each set of corrections). A real fault that drifts slowly
  would look different to the runs check.
- **One simulator session per case.** Right's cold 8% wheels sit on the line (1.8 x in the rerun, 2.1 x in a test).
- **Machine load** during the reruns (other agents' work): timing and holds may differ on a quiet machine; the flags
  don't depend on timing.
- **The find's pose:** "a find leans on the robot's own counting" is inferred from the evidence (setup finds 1.2-2.2 cm
  off only with wheel jitter), not from the firmware's find code.

## Appendix: the checks
| Sensor | Own checks (no other robot) | Baseline (no history: the prior) |
|---|---|---|
| Camera | finds against the board touches (once the robot's wheels are judged); scatter standing still (not during a find) | Alvik 0.8 cm |
| Wheels | stretches (rms per fit); runs (each run's scale, spread) | own earlier sessions (prior: 0.6 cm with the camera's share; 2.6%, never lower) |
| Gyro | stretches (heading rms per fit) | own earlier sessions (prior: 1.0° with the camera's share) |
| Front | against the map (camera's share off); at a board (face readings); "changed" from e02_front | own earlier sessions (prior: 5 cm map, 2 cm at a board) |

While the camera is unknown or flagged, no wheel, gyro or front sensor is newly flagged: all their checks use the
camera. KC's robot uncertainty and the leg check are widened by a robot's wheel k only while its wheels are flagged
(k now compares with the robot's own baseline, so a healthy robot can sit between 1 and 2).
