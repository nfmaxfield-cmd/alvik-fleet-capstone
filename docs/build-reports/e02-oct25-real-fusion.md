# First fusion on real sessions, and the Live lab's restart threshold (2026-10-25)

Logs only, no board time (Nick's standing rule: logs-only experiments run without asking). No runner or firmware code
changed. Follows [the RAW units fix](e02-oct25-units-fix.md), which made the offline fusion read the robots' speeds
right. Scripts: `tools/experiment/analyses_oct25/real_fusion.py` (part 1), `live_restart_doe.py` (part 2),
`real_fusion_report.py` (tables, charts). Outputs: `analyses_oct25/out/real_fusion_*.csv|txt`,
`out/live_restart_doe_*.csv|txt`.

## Premise

Every fusion number so far came from the simulator. Two questions on real data:
1. Does merging wheels + gyro + camera beat each source alone, and by how much? How fast does each robot drift
   without the camera, and how does that compare with the robot's own pose and with the runner's results?
2. The Live lab (the live view's filter, display only) still restarted 300-525 times per real session. Is the
   restart threshold (60, chosen) the cause, and what value or rule should replace it?

**Method, part 1.** Every find and every finished E02 route on disk (203 runs, 7 sessions with driving:
2026-10-22 08:16, 09:51 and 20:05, 10-23 morning and evening, 10-24 stages 1-2; the other 8 folders hold only
standing calibration runs) goes through `tools/fusion/engine.py` as it is, three ways:
- **wheels + gyro** (no camera), from the robot's own start belief;
- **wheels + gyro + camera** with the camera withheld for gaps of 2, 5, 10, 20, 40 s (15 s of camera between gaps);
- **wheels + gyro + camera** with the camera on all the time, and with the camera only for the first 10 s.

The reference is the camera itself (track.csv, "held" rows left out), scored leave-out: at the end of each gap the
fused estimate is compared with what the camera saw there. In the same gaps, each source alone is anchored at the
camera's last position before the gap: the camera held there, the camera moved on at its last speed, wheels + gyro's
displacement over the gap, and the robot's own pose's displacement (routes: the firmware's pose recomputed from RAW,
`oct25_common.reintegrate`, which matches the logged end pose). The camera's clock was lined up with the wheels by
speed correlation (shift +0.04 to +0.14 s per session).

**Heading check** (the runner's heading problem, [e02-oct25-heading.md](e02-oct25-heading.md)): each run's start
heading was compared with the camera's direction of travel on its first straight stretch (8 cm or more, one way,
wheels within 15 % of each other). 195 runs measured: **none started 20° or more wrong**. 8 runs could not be measured
(the camera had no rows during the run, or, in one 2026-10-22 session, saw the robot about 96 cm from where it
was) and are excluded. The placeholder headings sit in the idle moves between runs, which RAW does not cover, so they
never reached these stretches.

**Board-side camera.** Within about 15 cm of a blocked board the camera's position jumps 5-8 cm (the outline merges
with the board) and can stay off. At find ends, against the find's own pose from touching both boards, the camera is
5.1 cm off (median, 71 finds), the fused estimate with the camera on 3.6 cm. So the headline gap numbers use gap
ends 15 cm or more from both boards; the near-board ones are in Appendix A.

## Data

![Position error after a camera gap](e02-oct25-real-fusion-gaps.png)

The fused estimate (blue) stays closest to where the camera saw the robot after a gap of any length; wheels + gyro
alone is close behind, and the camera alone is lost after 2 s.

**Error at the end of a camera gap** (cm, median (90th percentile); 7 sessions, checked start heading, gap ends 15 cm or
more from the boards)

| Gap | Gaps | Fused | Wheels + gyro | Robot's own pose | Camera, moving on | Camera, held | Fused beat the best single source |
|---|---|---|---|---|---|---|---|
| 2 s | 273 | **1.4** (5.2) | 1.5 (3.9) | 2.5 (4.8) | 3.1 (12.1) | 16.5 (18.8) | 39 % |
| 5 s | 250 | **2.2** (5.7) | 2.7 (6.4) | 3.9 (7.7) | 8.0 (36.5) | 40.4 (45.0) | 50 % |
| 10 s | 126 | **3.4** (6.5) | 4.2 (7.6) | 4.6 (11.0) | 47.7 (96.4) | 46.9 (66.5) | 58 % |
| 20 s | 100 | **3.8** (6.6) | 4.6 (7.4) | 7.5 (16.4) | 142.7 (238) | 47.1 (60.9) | 46 % |
| 40 s | 7 | **5.1** (14.7) | 7.3 (19.6) | 19.2 (24.2) | 65.6 (132) | 46.0 (63.0) | 57 % |

<sub>Robot's own pose: routes only. 40 s: only 7 gaps (few runs last 50 s).</sub>

**Routes driven blind after their first 10 s** (57 routes; error at the end, each source anchored at the camera's
position at 10 s): fused 2.3 cm (90 %: 4.6), wheels + gyro 3.0 (7.3), robot's own pose 8.6 (17.5), camera held
36.8 (81.2).

![End-of-route error by condition](e02-oct25-real-fusion-routes.png)

On uncalibrated (U) and laptop-steered (K) routes the robot's own pose ends about 16-18 cm from where the camera saw
it; the fusion's wheels + gyro, from the same start, ends about 2-4 cm away.

**End of route, against the camera** (cm, median; checked start heading)

| Condition | Routes | Robot's own pose | Wheels + gyro (fusion, no camera) | Fused, camera first 10 s | Runner: ended from start (`end_err_cm`) | Runner: robot thought (`robot_thought_end_cm`) |
|---|---|---|---|---|---|---|
| U | 46 | 15.9 | **2.0** | 2.4 | 16.8 | 0.9 |
| C | 13 | 2.5 | **1.4** | 2.3 | 1.5 | 0.9 |
| K | 9 | 18.0 | **4.2** | 5.3 | 1.4 | 11.1 |
| KC | 12 | 1.5 | **1.3** | 1.9 | 2.6 | 1.1 |

<sub>The runner's two columns measure something else: how far from its start the robot really ended (camera) and
how far it believed it ended (its own pose). Their difference is close to the "robot's own pose" column on U.</sub>

**Drift without the camera** (wheels + gyro, start position aligned; per robot in Appendix B): routes 1.5-2.9 cm over
a 36 s, 4.5 m route (0.3-0.7 cm per metre driven; Left drifts about twice as much as Center and Right); finds 6-8.5 cm
(2.1-2.3 cm per metre), where the robot pushes against the boards and the wheels turn without moving it. The robot's
own pose drifts 3.1-4.4 cm per metre on U routes and 0.1-0.4 on C routes.

**With the camera on all the time** the fused estimate stays within 0.7 cm of the camera (median of runs), 4.8 cm at
the 95th percentile (the board-side jumps it refuses).

## Part 2: the Live lab restart threshold

PART2

## Conclusion *(Claude)*

1. **Yes, merging wheels + gyro + camera beats each source alone on real data, but only modestly over wheels + gyro
   anchored at the camera.** After a camera gap the fused estimate is 15-30 % closer than wheels + gyro (2.2 against
   2.7 cm after 5 s, 3.8 against 4.6 after 20 s, 5.1 against 7.3 after 40 s), about half the robot's own pose's
   error (3.8 against 7.5 cm after 20 s), and 10-40 times better than the camera alone. Gap by gap it beats the best
   single source only about half the time; at 2 s on routes it is slightly worse (2.4 against 1.7 cm), likely the
   tracker's frame-to-frame jitter pulling its speed and heading.
2. **The biggest gain on real data is not the camera: it is how the fusion uses the gyro.** From the same start,
   the fusion's wheels + gyro ends an uncalibrated route 2.0 cm from the camera; the robot's own pose ends 15.9 cm
   off. The firmware turns by the IMU's yaw, which reads about 7 % short until a gyro scale is learned (C routes:
   2.5 cm). The fusion integrates the raw gyro rate, whose scale is close to right (its 410-420° per square route is
   mostly a 0.6 °/s bias), and learns the bias whenever the wheels stand still. With no calibration at all it
   matches the calibrated C condition.
3. **The camera is not a perfect reference near the boards.** It jumps 5-8 cm there; the fusion's gate refuses
   those jumps, which is why, at find ends, the fused estimate is closer to the board-touch pose (3.6 cm) than the
   camera is (5.1 cm).
4. **The runner's heading problem did not touch these stretches** (0 of 195 runs started wrong). But one weakness
   showed: when the fusion starts far from the camera (one 2026-10-22 run, about 96 cm), it never recovers, because
   its gate refuses every camera fix and it has no restart rule.

PART2CONC

## What's next

1. **Nick decides** on the recommended changes below (none is applied; the firmware one needs the simulator first).
2. Rerun `real_fusion.py` after the next board session, especially with longer gaps (only 7 gaps of 40 s exist).
3. Fit the fusion's noise settings on real data (DOE backlog row "Fusion settings, older lane"): the 2 s result says
   the camera noise or its timing is not yet modelled right.

**Recommended changes (not applied: out of this task's scope)**

| # | Where | Change | Why |
|---|---|---|---|
| R1 | firmware `pose.h` heading (simulator first) | Heading from the gyro rate (`gz`), with its bias taken whenever the encoders say the robot stands still, instead of the IMU's yaw times a learned scale | U and K routes: own pose 16-18 cm off at the end, the same data integrated this way 2-4 cm. Removes the dependence on the gyro-scale fit |
| R2 | `tools/fusion/engine.py` | A restart rule: when the camera keeps landing outside a robot's possible area for N frames in a row while nothing else claims it, reset the position to the camera (as the Live lab does), heading unknown | One run never recovered from a 96 cm start |
| R3 | `tools/fusion/engine.py` / runner `camerr` | Widen the camera noise within 15 cm of a blocked board (or use the runner's camera-error map there) | The camera's board-side jumps of 5-8 cm |
| R4 | `tools/experiment/e02_live.py` | PART2REC | |

---

## New tests that may now be necessary

PART2TESTS
1. Simulator: heading from `gz` with standstill bias (R1) against today's yaw x scale, on U routes; then one board
   check of 4 U routes per robot.
2. Unit test for R2: a fusion run started 100 cm from the camera recovers within 1 s of driving.
3. The camera near the boards: score the tracker against marked footage within 15 cm of the boards (mark page), to
   size R3's widening.
4. A gyro-scale check on real data: integrated `gz` (minus the standstill bias) against the camera's turn on long
   spins, per robot, to confirm the "close to right" claim in conclusion 2.
5. Rerun `real_fusion.py` with the fusion's noise settings fitted on half the runs and scored on the other half.

## Possible inaccuracies

- **The reference is the camera**, whose own error (0.6 cm per axis while driving, 0.1-0.5 cm standing; 5-8 cm
  jumps at the boards; the floor calibration) is inside every number. Away from the boards that error is the same for
  every source, so the comparison holds better than the absolute values.
- **Single-source baselines are anchored at the camera's last raw sample** before the gap, which carries its
  frame-to-frame noise; the fused estimate starts from its own smoothed state. This favours the fused one slightly at
  short gaps.
- **Identity was given** (each robot's filter saw only its own track.csv rows); live, the fusion would have to assign
  sightings itself.
- **The engine's noise settings are the simulator's guesses** (ekf.Noise); nothing was tuned on these data.
- **The camera clock shift** is a speed-correlation estimate (+0.04 to +0.14 s; flat curve: correlation 0.53-0.71).
- **The gap layout is chosen** (first gap at 5 s, 15 s of camera between gaps; doubling lengths). A DOE is not needed
  for an analysis layout; a sensitivity rerun with 10 s between gaps would show whether re-convergence time matters.
- **The 15 cm "near a board" cut and the 20° heading cut are chosen** (analysis thresholds, no DOE needed).
- **Robot's own pose** for routes is the firmware's pose recomputed from RAW with the route's corrections; for KC
  that leaves out the camera fixes the robot received (the KC column then shows the robot's dead reckoning only).
  Finds have no own-pose trail here (the board resets mid-find are not modelled).
- **The board-touch pose at find ends** comes from board lines the camera measured, so it is not fully independent of
  the camera.
- **"Gyro scale close to right"** rests on the fusion's route results and one session's yaw/gz comparison, not on a
  direct measurement (new test 4).

PART2INACC

## Appendix A: gaps by kind and near the boards

APPX_A

## Appendix B: drift per robot

APPX_B
