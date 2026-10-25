# Discovery: when to accept a label, and which calibration moves to add (2026-10-25)

Logs and simulation only, no board time. Follows the [blind discovery replay](e02-oct25-discovery-replay.md) and
[Right's front sensor and rpm](e02-oct25-right-front-and-rpm.md). Code: `tools/discovery/`:

- fixes in `features.py`, `relations.py`, `classify.py`;
- new `threshold_doe.py`, `battery.py`, `battery_doe.py`, `threshold_chart.py`;
- tests in `tests/test_battery_and_threshold.py` (45 discovery tests pass).

Results are in `tools/discovery/results/`. **The acceptance rule below is now in the code** (`classify.decide`,
`MOTION_GATE_S`): it held on the held-out cases.

## Premise

1. **Two flaws in the replay** (found by the follow-up report): the motion response counted every 1 s window,
   standing still included; and the wheel-speed calibration included 1 s either side of each drive.
2. **Accept labels?** Nick: "design an experiment and make a recommendation." When should the system label a
   stream without asking? It depends on the threshold, on how much motion the log holds, and on whether units are
   known.
3. **Calibration test moves.** The 9 s calibration log identifies almost nothing that moves, and the IMU's
   non-gravity axes were always asked about. Which short moves should a calibration run hold, preferably without
   Nick's hands?

## Design

**Threshold experiment** (`threshold_doe.py`). Three factors:

| Factor | Levels |
|---|---|
| Acceptance threshold | 0.70, 0.75, 0.80, 0.85, 0.90, 0.95 |
| Motion in the log | 9 s calibration logs as they are (0-5 s of motion). Rich logs cut into contiguous pieces holding 10, 20, 40, 80 or 160 s of wheel motion (2 per case and level, random starts), plus each whole log |
| Units | plain; disguised (each stream rescaled by 0.1-1000, half sign-flipped) |

- **Responses per robot** (one piece): wrong accepted labels, questions (confidence 0.60 to threshold) and unknowns.
- **Size:** 206 pieces from 52 cases, 15,244 labels.
- **Rules compared:** threshold × motion gate (0, 5, 60, 120, 160, 240 s). The gate is how much wheel motion must
  be in the log before a motion-type label may be accepted. Passive labels (clock, counter, id, constant, battery,
  step index, switch) are never gated.
- **Held-out discipline:** the rule is chosen on development cases only (Left and Center, 10-21 to 10-23), then
  checked once on held-out cases (Right; every robot on 10-24).
- **Criterion**, fixed in the code before any results: wrong share at most 1%, with its 95% upper bound at most 3%,
  in plain and disguised units alike. Among passing rules, the fewest questions wins.

**The experiment ran twice.**

- **Pass 1** (Appendix B) found wrong labels at full confidence that no threshold could remove.
- **Then I fixed the rules behind them**, judging only on development pieces and the synthetic robot:
  - one clock per robot;
  - a gravity axis must read 300x its noise;
  - confidence capped by the score itself;
  - the pose heading must be a running sum.
- **Pass 2** reran everything, and the rule was re-decided on dev.
- **Disclosure:** in pass 1, I printed held-out numbers next to dev numbers while exploring gates.

**Calibration moves** (`battery.py`, `battery_doe.py`). The project simulator has no tilt physics (its gx and gy
are bias plus noise), so I built a synthetic robot with the same 37 RAW columns:

- tilts turn gravity between accelerometer axes and turn the x/y gyro;
- a lift raises the line sensors and sets the lifted flag;
- tape raises the line sensors;
- rest noise is copied from the real logs; speeds are in robot units (`tools/raw_units.py`).

Each plan ran on 3 robots (seeds), plain and disguised. The plans:

- today's 9 s log;
- a full battery, then the full battery minus each move;
- a stand plus each move alone;
- two compact batteries;
- a hands-free battery (printed ramp, one-wheel bump, open-edge approach), with and without each printed part.

**Tilt test** (new, `classify.tilt_pairs`). A tilt gives direct evidence for the IMU's other axes. The x/y gyro's
integral is the tilt angle, and gravity moves onto an accelerometer axis by sin(angle), so that axis's change follows
the same integral. It fires on no real log on disk: those have no deliberate tilts.

## Data

### 1. The two fixes, and the final discoverer, on the full replay

| | Before today | After the two fixes | Final (rule fixes + 60 s gate) |
|---|---|---|---|
| Held-out logs with motion: streams right (407) | 378 | 401 | **401** |
| Accepted / wrong among them | 234 / 0 | 237 / 0 | **218 / 0** |
| Questions | 138 | 149 | 168 |
| Right's range zones right | 26 of 49 | 46 of 49 | **46 of 49** |
| Disguised: wrong among accepted | 13 of 209 | 10 of 209 | **2 of 169** |
| Disguised and delayed: wrong among accepted | 10 of 174 | 9 of 178 | 2 of 124 |
| Development: right (333); accepted / wrong | 327; 184 / 1 | 303; 187 / 1 | 303; 130 / 1 |
| Wheel speed, cm/s per logged unit (15 cases) | 0.148 (19% low) | 0.164 (10% low) | 0.164 |

- **Right's zones are recognised.** 24 zones on 4 slow development logs flipped to "line sensor", none of them
  accepted. With moving windows only, zones read 1.6-13.6x their rest noise and line sensors 0.9-1.7x, so the two
  overlap on slow logs.
- **The ramp was re-anchored.** I moved the classifier's ramp from "weak" to "strong" from 2-6 to 1.5-3.0, from the
  development groups. The held-out values were on screen when I chose it.
- **Wheel speed.** 8 of its 10 points low are the motor board's real 1.08. About 2.6% is still the method.

### 2. Acceptance threshold against motion (pass 2)

![Threshold experiment](e02-oct25-discovery-threshold.png)

Left: how many of every 100 accepted labels are wrong at each threshold, for held-out pieces with different
amounts of motion. Right: each rule's questions per robot against its wrong share, where lower-left is better.

**Rules compared** (wrong share among accepted labels, 95% interval¹; questions per robot):

| Rule | Dev plain | Dev disguised | **Held-out plain** | **Held-out disguised** | Questions, held-out (plain / disguised) |
|---|---|---|---|---|---|
| 0.80 | 0.9% (0.1-1.7) | 3.5% | 1.0% (0.4-1.7) | 3.4% (2.2-4.5) | 8.0 / 8.4 |
| 0.90 alone | 0.3% (0-0.7) | 1.2% (0.4-2.3) | 0.14% (0-0.3) | 0.9% (0.4-1.5) | 11.1 / 10.9 |
| 0.95 alone | 0.1% | 0.5% | 0.08% | 0.26% | 12.7 / 12.1 |
| **0.90, gate 60 s (chosen on dev)** | **0.22% (0-0.6)** | **0.25% (0-0.7)** | **0.07% (0-0.2), 1 of 1,374** | **0.17% (0-0.4), 2 of 1,211** | **11.8 / 11.4** |
| 0.90, gate 240 s | 0 of 744 | 0 of 710 | 0 of 1,009 (0-0.3) | 0 of 968 | 14.9 / 13.5 |
| 0.95, gate 60 s | 0 of 785 | 0 of 729 | 0 of 1,208 | 0 of 1,095 | 13.2 / 12.4 |

<sub>¹ 95% bootstrap interval: the cases (one robot in one session) are drawn again at random with replacement
2,000 times and the figure recomputed each time. The middle 95% of those values is the interval. Cases rather than
streams are resampled because streams of one robot rise and fall together. When no wrong label was seen, the upper
end is 3 / (labels accepted), the usual 95% bound for zero events.</sub>

- **Why a gate is needed:** on dev, 0.90 alone fails the criterion with disguised units (1.2%).
- **A near tie:** 0.80 with a 120 s gate tied the chosen rule on dev questions. On held-out it was worse (0.4% /
  0.7%).
- **The blind motion estimate also passes.** It counts moving 1 s windows, and the code uses it when the system
  did not command the moves. With it in place of the true seconds the rule still met the criterion (held-out 0.14%
  plain, 0.65% disguised). It never fell short of 60 s when the truth was 60 s or more (61 pieces), and passed 60 s
  wrongly on 7 of 145 pieces with less.
- **The 3 wrong labels left on held-out:** accelerometer axes taken for range zones on long pieces. On dev they
  are a yaw rate taken for a wheel speed, and the pose heading for a wheel angle.

**At 0.90, by motion** (held-out; wrong / accepted, ungated): plain 0/109, 1/174, 0/187, 0/207, 0/301, 1/374, 0/100
for 0-5, ~10, ~20, ~40, ~80, ~160 and 240+ s. Disguised: 0/109, 5/177, 2/186, 3/181, 1/245, 1/291, 0/82. The
disguised errors bunch at 10-40 s, which is what the 60 s gate removes.

### 3. Calibration test moves (synthetic robot, 37 streams, mean of 3 robots, plain, confidence ≥ 0.90)

| Plan | Length | Right | Could be accepted | Wrong among them | Questions |
|---|---|---|---|---|---|
| Today's 9 s log | 12 s | 18 | 11 | 0 | 1 |
| **Full battery (hands)** | 141 s | **36.7** | 26.7 | 0.3 | 10 |
| … minus square drive | 83 s | 33.7 | 8 | 0 | 26.7 |
| … minus hand tilt | 123 s | 33.7 | 20 | 1.3 | 15.7 |
| … minus lift | 133 s | 35.3 | 27.3 | 1.0 | 8.7 |
| … minus line crossing | 127 s | 35 | 26 | 2.0 | 9 |
| … minus spins / straight drive | 129 / 127 s | 37 / 36.7 | 27 / 26.3 | 0 / 0 | 9.3 / 10.3 |
| **Hands-free: bump + open edge, no ramp** | 146 s | **36** | **25** | **0** | **12** |
| Hands-free with the ramp too | 195 s | 36 | 18.7 | 0 | 18 |
| Hands-free minus bump | 164 s | 36 | 18 | 0 | 19 |

What each move does:

- **Square drive** (12 legs of 15 cm with 90° turns, about 58 s). It is the main source of accepted labels: 12+
  camera heading changes (gyro scale and sign), drive ends against the boards (range zones), and it takes the log
  past the 120 s duration ramp.
- **Hand tilt:** the IMU's 4 other axes go from asked to accepted.
- **One-wheel bump:** does the roll half of that hands-free (gx and ay accepted on all 3 robots).
- **Driven ramp (pitch):** fell short. The pitch axes' relation reached r 0.87 against the test's 0.90 bar, and in
  this model the ramp's nose-up made the range zones look over the boards, so no zone was accepted. Without the
  ramp, the pitch pair (gy, ax) is asked about: 2 questions per robot.
- **Lift:** without it, the bottom zone passed as an angle on 2 of 3 robots. The hands-free plans had no such error,
  but they also never exercise the lifted flag (it stays a constant).

These are the discoverer's confidence values. With the 60 s gate, a battery holding about 95 s of motion accepts
them as shown.

## Conclusion

1. **Accept labels: yes, with a 60 s motion gate.** The rule is now in the code:
   - accept a label without asking when its confidence is 0.90 or more;
   - a label that needs motion (wheels, IMU, range, line, derived streams) is only asked about until the log holds
     60 s of wheel motion;
   - passive labels (clock, id, constant, battery) are accepted from the start.

   Evidence, held-out: 1 wrong in 1,374 accepted labels (plain), 2 in 1,211 (disguised units), at about 11.5
   questions per robot. Before the fixes, the best rule (0.90 with a 240 s gate) left 0.7% wrong and asked 14-15
   questions.

2. **The risk each option carries** (held-out, after the fixes):

   | Option | Wrong labels it lets through | Questions per robot |
   |---|---|---|
   | 0.80 | about 1 in 100 (1 in 30 with unknown units) | 8 |
   | 0.90 alone | about 1 in 700 (1 in 110 with unknown units) | 11 |
   | **0.90 + 60 s gate** | **about 1 in 1,400 (1 in 600 with unknown units)** | **11.5** |
   | 0.90 + 240 s gate | none seen in 1,009 | 14-15 |

   The remaining risk is accelerometer axes taken for range zones on long logs.

3. **The fixes that made this possible:**
   - **One clock per robot:** a wheel that turned one way through a log was taken for a clock.
   - **Gravity bar:** a gravity axis must read 300x its noise; pitch_deg had stood in for gravity.
   - **Confidence cap:** confidence is capped at 0.6 + 0.4 × the best score, so a lone weak cue is asked about, not
     accepted.
   - **Heading must be a running sum:** in disguised units the pose test had paired a wheel angle with the fused yaw
     "steered" by the drive speed.
   - **Synthetic-robot fixes:** a flickering "angle" no longer makes its group-mates computed outputs, and the tilt
     test has two guards.

4. **Calibration moves to add, hands-free version** (about 2.5 minutes, about 95 s of it wheel motion, so past the
   60 s gate):

   | Move | Time | What it gives |
   |---|---|---|
   | Stand still at start and end | 10 s + 5 s | rest noise, which streams flicker together |
   | Square drive: 12 × (15 cm, turn 90°) | about 58 s | gyro against the camera, wheels, zones; most accepted labels |
   | One-wheel bump, forward and back, twice (printed part, spec below) | about 31 s at 3 cm/s | roll: x gyro and y accelerometer accepted |
   | Open-edge approach, twice (spec below) | about 14 s | line sensors and bottom zone stepping at a commanded moment |
   | Spins 4 × 90° and a straight drive at a board | about 12 s + 14 s | optional in simulation; the board drive gives the real zones board distances |

   **Left without hands:** the pitch pair (y gyro, x accelerometer: 2 questions per robot) and the lifted flag.
   A 5-second hand nose-tilt would close the first; only a lift exercises the flag.

### Hands-free test parts: specification (not built, not tested on a robot)

**One-wheel bump** (roll):

- *Part:* a printed half-cylinder strip, 1.0 cm high, 10 cm long in the direction of travel, 3 cm wide (one
  wheel's width plus margin), flat base with a 1 mm chamfer, PLA.
- *Roll it gives:* about atan(1.0 / 8.9 cm wheel track) = 6.4°.
- *Where:* on the plywood, 30 cm from both boards (inside the camera's view, clear of the 25 cm edge keep-out),
  long side along the x board. Taped down, or held by a pin into a drilled hole.
- *Move:* the robot drives 20 cm with its left wheel over it at 3 cm/s, back the same way, then repeats with the
  right wheel, so roll goes both ways.
- *What it reveals:* the x gyro (integral = roll) and the y accelerometer (gravity onto it); roll_deg found as
  computed.
- *Risks:* the wheel slips on the bump's face, or the robot yaws off it. The camera sees both.

**Ramp-and-plateau** (pitch, tested in simulation, not recommended yet):

- *Part:* full width 12 cm; 1.0 cm rise over 8 cm, a 6 cm plateau, 8 cm down.
- *Pitch it gives:* about 14° while the drive wheels climb and the rear caster is still on the floor.
- *Why not yet:* in simulation the pitch relation stayed at r 0.87 (bar 0.90), and the nose-up hid the boards from
  the range zones. Worth a real try only after the real bump has worked.

**Open-edge approach** (line sensors, no tape):

- *Move:* the robot faces one of the two open plywood edges (no border board) from 25 cm and creeps forward at about
  3 cm/s with the edge guard on (`safe_edge 3`, `edge_thr` 150). The guard trips when the line sensors leave the
  plywood (52-84 on plywood; the two real trips read 241 and 573), and the robot stops and backs 10 cm. Twice.
- *What it reveals:* a large line-sensor step at a commanded time, and the bottom zone's floor distance growing
  past the edge.
- *Risk:* falls (2026-10-23). It uses only the guard's own stop, which has stopped every real trip so far. Do it at
  creep speed, square to the edge, and keep the camera on.
- *If the guard should not be relied on:* a black printed strip, 2 cm wide, laid 15 cm ahead (the dock's dark
  stripe reads like black tape) does the same job.

## What's next

1. **Real calibration battery, one log per robot** (design rank 8 as a runner stage):
   - square drive, bump, open-edge approach;
   - optionally a hand nose-tilt and a lift.

   Then rerun the discoverer on it, and check the 60 s gate and the move table against real data.
2. **Print the bump** (spec above) and log it on one robot first.
3. **The yaw/pose heading swap** is fixed on the real logs, but still appears in 3 synthetic ablation plans. There
   both headings integrate the gyro, so the tie-break cannot separate them.
4. **Shortest battery:** synthetic sweep of square legs (6 / 12 / 18) and bump repeats.

---

## New tests that may now be needed

1. **The bump on a real robot:** does the tilt test fire on gx and ay, and only those?
2. **Open-edge approach:** do the line sensors step as the guard's trip readings suggest, and does the bottom zone
   follow?
3. **Bottom zone and pitch:** does the real bottom zone follow pitch as the synthetic one does? (The docs note it
   tilts when the robot speeds up.) This decides whether a lift is needed.
4. **Hand nose-tilt, 5 s:** does it close the pitch pair on a real log?
5. **Gate edge:** pieces at 30, 45 and 60 s of motion, to see how sharp the 60 s edge is.
6. **Confidence calibration after the cap:** are labels at 0.90 right 90% of the time?
7. **A combined pytest run across tools/** in CI (the `score.py` name clash).

## Possible inaccuracies

- **The synthetic robot is my model.** Chosen, not measured:
  - IMU physics;
  - the bottom zone's response to pitch;
  - line-sensor lift;
  - zones looking over the boards beyond 8° of pitch;
  - tape reading 600;
  - move speeds;
  - the printed parts' shapes and the times in the move tables.

  The rankings show what each move makes possible for this discoverer.
- **The rule fixes were judged on dev pieces and the synthetic robot only,** but they came after pass 1, whose
  held-out numbers I had seen. Pass 2's held-out check is the first on the fixed discoverer. It is not blind to the
  kinds of error pass 1 showed.
- **Motion seconds in the experiment come from the true wheel angles.** The code takes the commanded seconds, or
  the blind estimate (checked above).
- **Pieces come from sessions of finds and routes,** not batteries.
- **Few dev cases at high motion** (4 at ~40 to ~160 s).
- **Chosen numbers in the new code**, all in the DOE backlog:
  - the criterion (1% / 3%);
  - the gate levels tried;
  - AOR_RAMP 1.5-3.0;
  - MOVE_MIN_WINDOWS 5;
  - CONF_CAP (0.6, 0.4);
  - HEADING_TIE 0.10;
  - GRAVITY_LON 300 (the existing gravity bar);
  - TILT_R 0.90 and its two guards;
  - the battery's and printed parts' sizes.
- **"Questions" assume the person answers correctly.** The share where the truth is not among the 3 offered is
  larger on short pieces.

## Appendix A. Changes to the discoverer

| File | Change |
|---|---|
| `features.py` | `moving_response()`: motion response over moving 1 s windows (5 needed). Wheel-speed integral over the drive stretch only |
| `relations.py` | Moving rows from the streams that sit flat at rest; a planar heading must be a running sum |
| `classify.py` | `AOR_RAMP` 1.5-3.0; one clock per robot; gravity axis ≥ 300x its noise; confidence cap `CONF_CAP`; heading tie-break `HEADING_TIE`; tilt test; lockstep rule ignores flickering "angles"; `decide(conf, type, motion_s)` with `MOTION_GATE_S` 60; `motion_seconds()` blind estimate; `classify_case(feats, R, motion_s=None)` |
| `disc_score.py` | Renamed from `score.py` (clashed with `tools/site/score.py` in a combined pytest run) |
| `tests/synth.py`, `battery.py` | Speeds in robot units via `tools/raw_units.py`, with a `sim_info.json` marker; battery moves incl. ramp and bump |

## Appendix B. Pass 1 (before the rule fixes)

- **0.90 alone**, held-out: 2.6% wrong plain, 5.0% disguised.
- **Best rule then:** 0.90 with a 240 s gate (0.7% / 0.7%, 14-15 questions).
- **The 61 wrong labels at confidence ≥ 0.90**, 31 of them at the top value of 0.95:
  - roll/pitch taken for accelerometer axes: 22;
  - accelerometer axes taken for range zones: 11;
  - wheel angles taken for clocks: 7.
- **Pass 1 files:** `threshold_*_v1` were not kept in the repo. The numbers are in the git history of this report.

## Appendix C. Reruns

```
python3 tools/discovery/replay.py                       # about 5 min on 2 busy cores
nice -n 10 python3 tools/discovery/threshold_doe.py     # about 20 min; --analyse for the tables only
nice -n 10 python3 tools/discovery/battery_doe.py       # about 7 min
python3 tools/discovery/threshold_chart.py
```
