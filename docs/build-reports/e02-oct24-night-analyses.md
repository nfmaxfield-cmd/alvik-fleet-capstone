# Three offline analyses of the 2026-10-24 Pi plan (night of 2026-10-24, no robots)

Sessions used (downloaded logs): Pi stage 1 `e02-20261024-153612`, stage 2 `e02-20261024-161612` (classic, U/C/K/KC),
stage 3 `e02-20261024-173242` (explore); laptop sessions for comparison `e02-20261023-085107` (morning) and
`e02-20261023-191347` (evening). Tonight's Right-alone session (`e02-20261024-193526`, during the 5.8 GB upload) is
not in this report: its GitHub release (`latency-test-20261024`) holds only the video parts and `rec-old-small.zip`,
which is the frame index of the 15:36-17:39 recording, not that session's logs.

Scripts (re-run any of them; offline, read only the session folders): `tools/experiment/analyses_oct24/night_common.py`
(loaders, and the rebuild of the runner's stretches), `night_trust_replay.py` (A), `night_selfcal.py` (B),
`night_latency_battery.py` (C). The rebuild reproduces the runner's own stretches exactly: same count for every robot
in every session, lengths within 0.012 cm (checked against each state file's `fit_data`).

**The short version.** The stage-2 refusals were caused by tracker jumps, not by the camera's scale or the wheels.
When the camera's track jumps 3 cm or more in a single picture (the lock-on catching up), the jump ends up inside a
"straight" stretch. The stretch check only measures sideways scatter, so a jump along the direction of travel gets
through. If the track is cut at those jumps, every wheel fit in stages 1 and 2 passes the old 1.5 cm limit. The
camera's error map cannot help: all its samples sit in one corner, so everywhere it applies it is the same constant
shift, and a constant shift does not change a length. The new trust code would have accepted the stage-2 fits, but
only by widening its limit to the 3 cm cap. Standing robots drain 11-16 % an hour, and driving adds little that can
be measured.

---

## A. The stage 1 and 2 data replayed through the sensor-trust code

**Premise.** `e02_trust.py` (new today) learns each sensor's noise (sd) and its trust factor k = learned sd / healthy
sd, and flags k ≥ 2. Would it have caught anything in stages 1-2, what wheel limit (`wheel_gate_cm`) would it have
set, and would stage 2's fits (all refused at 1.5 cm) have passed?

**Method.** `night_trust_replay.py` feeds `SensorTrust` in time order, the way e02.py does:
- **Camera:** at each find, the camera's error (`camera_error_samples`, source "find") corrected by the error map.
  Two variants: the map of the finds before it (what the runner can know at that moment), and leave-one-out (all
  the other finds of the session).
- **Wheels and gyro:** after each U run, that run's rebuilt stretches against the robot's pooled fit (`fit_robot`
  with the gate the trust sets at that moment), as in e02.py `stretch_rms_against`. The raw stretches could be
  rebuilt, so the fits.csv fallback was not needed.
- **Front:** front_checks.csv rows with a measured heading, the thing ahead within 50 cm, and met within 45° of head-on.
  The value used is the reading − `FRONT_OFFSET_CM` (0) − the map distance.

Each stage was replayed on a fresh trust file, and then stage 1 → stage 2 with the file carried over, halved at the
new session as `logs/sensor_trust.json` would be.

**Data.** Learned sd and k at the end of each replay (from the session logs above; as-run stretches; camera map
built from earlier finds).

| Replay | Camera (per axis) | Left wheels / gyro / front | Center wheels / gyro / front | Right wheels / gyro / front | Wheel limit at the end |
|---|---|---|---|---|---|
| Stage 1 alone | 0.88 cm, k 1.1 | 1.40 cm k 2.3 **F** / 1.0° k 1.0 / no data | 0.20 k 0.3 / 0.30° k 0.3 / 1.2 cm k 0.2 | 0.60 k 1.0 / 1.72° k 1.7 **F** / 2 readings | 3.0 cm (cap) |
| Stage 2 alone | 0.58 cm, k 0.7 | 0.60 k 1.0 / 1.0° k 1.0 / 4 readings | 1.97 k 3.3 **F** / 0.62° k 0.6 / 5.0 k 1.0 | 0.29 k 0.5 / 2.09° k 2.1 **F** / 5 readings | 3.0 cm (cap) |
| Stage 1 → 2 carried | 0.61 cm, k 0.8 | 0.60 k 1.0 / 1.0° / 4 readings | 1.08 k 1.8 **F** / 0.53° / 2.1 k 0.4 | 0.20 k 0.3 / 2.01° k 2.0 **F** / 13.2 cm k 2.6 **F** (7 readings) | 3.0 cm (cap) |
| Laptop morning 23rd (reference) | no samples (healthy 0.8) | 0.60 / 1.16° / 5.1 cm | 0.20 / 0.45° / 4.8 cm | 1.05 k 1.75 **F** / 1.0° / 3.6 cm | 2.3 cm |

<sub>**F** = flagged at the end of the replay (it went over k 2 and has not come back under 1.5). A robot with fewer
than 3 fits, or fewer than 6 front readings, sits at its healthy value. "sd" for the camera is a robust sd: the median
absolute value ÷ 0.6745, which equals the sd for normal data and ignores a few outliers.</sub>

Stage 2's fits under the trust gate (stage 2 alone, as the runner would run it):

| Fits (run order) | Gate the trust sets | Wheel fit rms | Passed? |
|---|---|---|---|
| Center 1, Right 2, Center 3, Left 4, Right 5 (16:29-16:40) | 2.48 cm (camera's share taken from the finds) | 1.61-2.65 | 3 of 5 (Center 1 and Left 4 not pinned down yet) |
| Center 6, Left 7, Right 8 (16:42-16:46) | 2.08-2.16 cm (the camera looks better at the finds as they pile up) | 2.12-2.54 | none |
| Left 9 onward (16:48-16:57) | **3.0 cm cap** (two robots now have 3 fits: the "shared term" takes over, 3.7 cm) | 2.09-2.35 | all 4 |

Carried over from stage 1, the gate is 3.0 cm from the first stage-2 fit, so the 10 of 12 fits that were pinned
down pass. Full fit-by-fit tables are in the appendix.

**Conclusion.**
1. **Would the stage-2 fits have passed?** Yes, but only because the gate rose to its 3 cm cap. The code's own warning
   fired ("every robot's stretches are noisier than the camera at the finds explains") and it accepted the fits anyway.
   Part B shows the noise was the camera track's jumps, so these accepted wheel scales are off by up to 1.8 %
   (Right 1.002 against 0.984 once the jumps are cut). That is about 3-4 cm over a 200 cm route.
2. **The flags are mostly the method, not the sensors.** With three robots, the camera's share is set from the
   median robot. So the robot above the median gets the excess as "its wheels" whenever the robots differ by about
   1 cm. It flagged Right's wheels on the healthy laptop morning, and Left (stage 1) and Center (stage 2) on the Pi.
   Each flag rests on 3-4 fits. With the jumps cut (part B), the morning has no flags and stage 2 has two that
   clear within 2 minutes.
3. **The camera at the finds measures repeatability, not accuracy.** All finds are at one spot (within 3 cm of
   113.8, 107.8). Its k (0.5-1.1) says nothing about the rest of the board. In the leave-one-out variant the camera
   is flagged at 15:41 and 16:20 and clears 4-5 minutes later. The cause is that the first three finds of each
   session were recorded before the runner moved the x-board line by 2.6-2.8 cm from the robots' touches, so their
   "true" positions are in the old frame.
4. Right's front flag (carried replay) rests on 7 readings, one of them +46 cm. The Pi sessions give only 0-7
   usable front readings per robot, against 14-21 on the laptop morning.

**What's next.** Hold off on trusting the 3 cm cap. Put the jump cut into the stretch finder first (B), then re-run
this replay. When the runner moves a board line, re-express the camera's error samples in the new frame (or start
the map again).

---

## B. Self-calibration with the camera corrected by its error map, and the alternatives

**Premise.** In stage 2 the camera and the wheels disagreed by 2.1-2.35 cm per stretch (1.2-1.5 on the 23rd's laptop
morning), while at the finds the map-corrected camera was within 0.4-0.9 cm. Does correcting the camera positions by
the map before the stretches are found bring the fits under 1.5 cm? Does the map cover the stretches? If not, what
does explain the gap?

**Method.** `night_selfcal.py` rebuilds every U run's stretches from track.csv and raw.csv. Score = each robot's
pooled wheel fit rms (`fit_wheels`, the number the runner gates at 1.5 cm). Tests, in order:
- B1: where the stretches are relative to the finds.
- B2: the map applied to every camera position before the stretches.
- B3: the camera's scale. (a) The error at the finds, turned into a picture shift and pushed through the session-1
  calibration file (roam1 homography). (b) A shared camera-scale model (direction, place on the board, and distance
  from the picture's middle) fitted with a wheel scale per robot, scored leave-one-run-out.
- B4: time. The camera's times shifted −0.3 to +0.3 s against the robots'.
- B5/B6: the tracker. Stretches with a camera jump inside them, and the track cut at jumps before the stretches are found.

**Data.** Wheel fit rms per robot, cm per stretch (rebuilt from track.csv + raw.csv; × = above the 1.5 cm gate):

| Test | Stage 1 L / C / R | Stage 2 L / C / R | Laptop morning L / C / R |
|---|---|---|---|
| As run | 1.82× / 1.21 / 1.89× | 2.35× / 2.34× / 2.09× | 1.20 / 1.17 / 1.50 |
| B2: camera corrected by the error map | 1.33 / 1.45 / 1.97× | 2.04× / 2.16× / 2.02× | (no finds' samples) |
| B4: best time shift (±0.05 s) | n/a | 2.30× / 2.33× / 2.09× (no shift helps) | 1.17 / 1.17 / 1.46 |
| **B6: track cut at jumps over 3 cm** | **1.27 / 1.17 / 0.88** | **1.28 / 1.10 / 0.80** | 0.84 / 0.79 / 0.88 |

Where the misses are, stage 2 (robust sd of camera − wheel scale × counted length; 91 stretches):

| Group | Share of stretches | Miss (robust sd) |
|---|---|---|
| A camera jump over 3 cm inside the stretch | 66 % (laptop morning: 51 %) | 5.96 cm (morning: 2.37) |
| No jump, open board | 25 % | 1.60 cm |
| No jump, near a board (within 15 cm) | 9 % | 1.88 cm |

<sub>"rms" here is the Huber-weighted root mean square of the fit's residuals, as in e02_fit. "Leave one run out"
means each run is predicted by a fit made without it.</sub>

Further results (the appendix has the tables):
- B1, coverage: stage 2's 27 finds all lie within 2.7 cm of one point. 30 % of stretch ends are within 25 cm (one
  kernel width) of it, and 24 % are beyond 75 cm, where the map gives no correction at all. Wherever it applies,
  the correction is the same (−3.5, −3.6 cm).
- B2: the stretches found both before and after correction changed length by 0.012 cm (median). The rms changes in
  the table come from stretches cut differently at the map's edge, where the correction drops by 4.7 cm in a step.
- B3a: the finds' error equals a picture shift of about (8.7, 10.7) px. Through the calibration that shift changes
  stretch lengths by 0.12 cm (median), 0.64 at worst.
- B3b: the shared scale model lowers nothing reliably (stage 2 leave-one-run-out robust sd 3.58 → 3.87 cm), and its
  terms swing from session to session.
- B4: shifting the camera's times helps by at most 0.05 cm.
- Run by run with the 3 cm cut, at the old 1.5 cm gate: all 9 stage-1 wheel fits and all 12 stage-2 wheel fits pass
  (rms 0.63-1.47). Gyro rms falls from about 3° to about 2° on the Pi, and from 1.2-1.6° to 0.9-1.2° on the morning.
- An example (stage 2, Center run 6, 16:40): camera and wheels agree at 6.6 cm/s for 1.8 s. Then one picture moves
  the camera's position 7.1 cm (108.1 → 115.2) in 0.12 s, and the stretch ends there: 19.2 cm against 12.7 counted.

**Conclusion.** Correcting by the error map cannot fix the stretches. A map built from one corner is a constant
shift, and a constant shift leaves lengths unchanged. Its only effect is the step at its edge. The camera's scale
(calibration shift, or a shared scale or direction term) and the timing are ruled out. The cause is tracker jumps
along the direction of travel inside stretches: 66 % of stage-2 stretches hold one, against 51 % on the laptop
morning, and those stretches miss by more (5.96 cm against 2.37). Cutting the track at jumps over 3 cm brings every Pi fit under 1.5 cm
and improves the laptop's fits too. The Pi handles 14-16 pictures/s against the laptop's 18-20 (C1). That may be
why it jumps more (the lock-on moves further per picture), but this report does not show it.

**What's next.** Proposed change, for Nick to approve first: in `e02_fit.camera_stretches`, cut the camera track
wherever one picture moves more than 3 cm, before the straightness split. Add a unit test from today's stage-2
track. The cut size is a chosen number: the 2/3/4 cm sweep in the appendix shows the trade (2 cm drops 20-35 % of
the counted distance). A DOE scored by C-run end errors would be the right way to settle it. For a map that
actually covers the board: the tape marks already listed in `e02_camerr.MARKS`.

---

## C. Latency and battery

**Premise.** How much delay do the camera pictures and the WiFi add on the Pi compared with the laptop? And how fast
do the robots drain, driving and standing? (Right went from 63 % at 17:32 to 35 % at 19:35, mostly standing.)

**Method.** `night_latency_battery.py`.
- **C1:** perf.csv, one row a second.
- **C2:** clock_sync.csv. The runner pings each robot 3 times about every 10 s; per burst, the shortest round trip.
- **C3:** every battery reading in the logs (runinfo, runend, raw rows, find, e02 and gyro records, and the log's
  "battery NN%" lines), reduced to a per-minute median per robot. Driving time comes from raw rows with the wheels
  turning, plus go/turn legs outside runs (leg.csv). Fit per robot: battery = b0 − standing rate × hours − extra
  rate × hours driving. Also the standing gaps between stages, and Right's 63 → 35 % (the 35 % comes from the brief;
  that log is not here).

**Data.** Camera processing and WiFi (perf.csv and robot-logs clock_sync.csv of each session):

| Session | Runs on | Pictures/s handled | Dropped | ms a picture (median / 95 %) | Picture age s (median / 95 %) | Seconds with a picture > 1 s old | Shortest WiFi round trip per burst, ms (median / 99 %) | Bursts > 100 ms |
|---|---|---|---|---|---|---|---|---|
| 23rd morning | laptop | 19.8 | 21 % | 44 / 100 | 0.12 / 0.29 | 12 | 14 / 165-260 | 2.1-2.5 % |
| 23rd evening | laptop | 17.7 | 30 % | 46 / 91 | 0.14 / 0.47 | 25 | 23-25 / 294-1061 | 7.5-9.4 % |
| Stage 1 | Pi | 16.2 | 32 % | 58 / 113 | 0.12 / 0.25 | 0 | 11-12 / 178-325 | 1.3-2.6 % |
| Stage 2 | Pi | 14.1 | 40 % | 66 / 127 | 0.13 / 0.23 | 0 | 11-12 / 342-394 | 3.9-5.1 % |
| Stage 3 | Pi | 15.4 | 28 % | 61 / 111 | 0.13 / 0.18 | 0 | 10-12 / 22-334 | 0-2.8 % |

<sub>Picture age = the time from the camera taking the picture to the robot's position being ready. Ranges are the
three robots.</sub>

Battery drain, % per hour (readings and driving time from the robot logs and e02_log.txt):

| Robot | Pi afternoon fit, standing / driving | 23rd morning fit, standing / driving | Standing gaps between stages | Right 17:32 → 19:35 (4 s driving) |
|---|---|---|---|---|
| Left | 15.3 / 24.2 | 15.2 / 17.9 | 15.7 (8 min) | |
| Center | 16.0 / 13.7 | 14.4 / 16.9 | 14.8 (20 min), 12.6 (10 min) | |
| Right | 14.1 / 13.3 | 12.7 / 17.9 | 10.7 (11 min), 11.2 (5 min) | **13.7** |

<sub>Each robot drove for 0.29-0.51 h of each day's span (about a quarter of the time), so the driving rate is poorly
pinned: it moves 10 %/h between days. The fits' rms is 0.25-0.33 %.</sub>

**Conclusion.**
- **Latency.** The Pi does 25-50 % more work per picture (58-66 ms against 44-46), handles 14-16 pictures/s against
  18-20, and drops more. But the picture age is the same as the laptop's (median 0.12-0.13 s) and its tail is
  shorter: no picture older than 1 s in any Pi stage. The WiFi round trip is lower on the Pi (11-12 ms median).
  Stage 2's tail (99 % at 340-390 ms, 4-5 % of bursts over 100 ms) is worse than stage 1's. There is no comparison
  with the upload session until its logs are uploaded.
- **Battery.** A switched-on robot standing still drains 11-16 % an hour, and driving adds too little to measure
  against that. Right's 13.7 %/h fits the standing gaps. A robot switched on at 98 % reaches the runner's 40 %
  warning in about 4 h whether it drives or not.
- **Sag bias.** Within a run, the reading while driving equals the reading while standing (difference −0.01 to
  +0.03 %, 70 runs). So the library's `get_battery_charge()` is smoothed, or not a raw voltage, and sag does not show
  at this 1 % resolution. If it is voltage-based anyway, readings taken just after driving read low and recover at
  rest, which would make standing drain look smaller and driving drain larger. No recovery shows: no reading rises
  after a rest, so this bias is at most about 1 %. A voltage-based % also falls faster below about 40 %, which would
  make Right's 63 → 35 % read faster than its true drain at mid-charge.

**What's next.** Firmware 2026-10-25a adds `sleep <min>`. Test what it saves: one robot asleep for 1 h, one awake,
reading before and after. Separate the driving cost: one robot driving loops for 30 min, one standing. Once the
night session's logs zip is up, re-run C1/C2 with it.

---

## New tests that may now be needed
1. **Jump cut in the stretch finder** (if approved): a unit test on today's stage-2 track (rms under 1.5 cm), plus a
   C-run check that fitted wheel scales predict end errors at a cut of 2 / 3 / 4 cm (a DOE).
2. **Trust false-alarm test on healthy sessions:** replay the 23rd morning and evening (healthy) and count flags. Today
   it flags Right's wheels on the morning. Try a minimum of more than 3 fits before a robot's k is used, and a shared
   term taken from the cleanest robot.
3. **Error samples across board-line moves:** a unit test that a board-line update re-expresses (or drops) earlier
   `camera_error_samples`.
4. **Tape marks** at the `e02_camerr.MARKS` points, so the camera's error is known away from the find corner. This is
   the only way to tell a camera error across the board from the wheels (the trust code's open question).
5. **Pi picture rate against tracker jumps:** the same U route with the tracker at a lower picture size (more
   pictures/s), counting jumps over 3 cm per stretch. Gyro rms is still about 2° on the Pi against about 1° on the
   laptop after the cut.
6. **Sleep drain** and a **driving-vs-standing drain** test (C, what's next).
7. **Upload session latency:** C1/C2 on `e02-20261024-193526` once its logs are on GitHub.

## Possible inaccuracies
- **Replay fidelity.** The replays rebuild the runner's stretches (exact match), but they run the current e02_fit and
  e02_trust over data collected with older code. The C/K/KC runs are not re-simulated, so a fit accepted earlier
  would not have changed what those runs did in this replay.
- **Circular reference values.** The trust code's healthy sds were set from these same sessions (docstring), so
  k ≈ 1 here is partly by construction.
- **The first find of each session** has no map (empty) and counts its full error (about 3 cm) as the camera's, in
  the runner-faithful variant. The runner does the same.
- **Chosen numbers in this report**, each with a DOE noted where it fits:
  - jump cut 3 cm (swept 2/3/4; DOE as in test 1);
  - "near a board" 15 cm;
  - driving = wheels above 5 rpm (sum);
  - a burst = pings within 1 s;
  - the shared scale model's five terms;
  - per-minute medians for battery.
- **B3a approximation.** It treats the camera's position as the true one when it applies the picture shift, and
  models only a shift, not a tilt. A tilt would need marks across the board to test (test 4).
- **Wheel scales across days.** Left fits 0.988 on the 23rd and 0.947-0.960 on the Pi; Center and Right 0.2-1.6 %
  lower. That could be a camera scale change between days (the camera moved; the 2026-10-24 analyses), the robots'
  battery level, or the floor. Nothing here separates them (test 4).
- **Battery inputs.** The battery % is an integer from `get_battery_charge()`, and what it measures is not checked.
  Right's 35 % at 19:35 is taken from the brief, not from a log. The robots are assumed to have stayed switched on
  between stages. Driving time misses any motion that is neither in a run's raw rows nor a logged leg.
- **The 23rd evening's worst picture age (894 s)** is a clock step, not a real delay. Only medians and 95 % values
  are compared.
- **Tonight's session:** latency with the upload running is not measured at all (no logs).

---

## Appendix

### A1. Camera at the finds, stage 2 (cm, camera − true; from `camera_error_samples`)
The raw error is about (−2.9, −1.2) for the first three finds and (−3.6, −4.0) after 16:21, when the runner moved
the x-board line by 2.83 cm from the robots' touches (`board_line_updates`).

| Variant | Robust sd per axis | rms | n |
|---|---|---|---|
| Stage 1, map of earlier finds | 0.88 | 1.21 | 24 |
| Stage 1, leave one out | 0.63 | 1.03 | 24 |
| Stage 2, map of earlier finds | 0.58 | 0.85 | 54 |
| Stage 2, leave one out | 0.42 | 0.71 | 54 |

### A2. Stage 2 alone, fit by fit (trust gate, as-run stretches; map of earlier finds)
| Time | Robot | Run | Stretches new/all | Gate cm | Fit rms cm | Wheels pass (trust gate) | At 1.5 | This run's rms cm / deg |
|---|---|---|---|---|---|---|---|---|
| 16:29 | Center | 1 | 6/6 | 2.48 | 2.65 | no (not pinned, ±0.033) | no | 4.04 / 1.57 |
| 16:31 | Right | 2 | 8/8 | 2.48 | 1.61 | yes | no | 0.80 / 3.42 |
| 16:36 | Center | 3 | 7/13 | 2.48 | 2.39 | yes | no | 3.41 / 2.03 |
| 16:38 | Left | 4 | 7/7 | 2.48 | 2.57 | no (not pinned) | no | 4.97 / 2.87 |
| 16:40 | Right | 5 | 9/17 | 2.48 | 1.87 | yes | no | 3.88 / 3.55 |
| 16:42 | Center | 6 | 8/21 | 2.16 | 2.54 | no | no | 5.82 / 2.88 |
| 16:44 | Left | 7 | 7/14 | 2.08 | 2.12 | no | no | 1.82 / 3.24 |
| 16:46 | Right | 8 | 9/26 | 2.08 | 2.12 | no | no | 4.98 / 3.60 |
| 16:48 | Left | 9 | 7/21 | 3.00 | 2.15 | yes | no | 3.19 / 2.92 |
| 16:51 | Right | 13 | 7/33 | 3.00 | 2.09 | yes | no | 3.47 / 3.36 |
| 16:53 | Left | 15 | 8/29 | 3.00 | 2.35 | yes | no | 4.10 / 2.84 |
| 16:57 | Center | 17 | 8/29 | 3.00 | 2.34 | yes | no | 2.60 / 4.00 |

Flag events, stage 2 alone:
- 16:42 Center wheels degraded (k 7.4);
- 16:43 Center gyro degraded (k 2.0), recovered at 16:46;
- 16:46 Right gyro degraded (k 2.9).

Stage 1 alone:
- 15:56 Center gyro degraded (k 2.8), recovered at 16:05;
- 16:05 Right wheels degraded (k 4.3), recovered at 16:08;
- 16:05 Right gyro degraded (k 2.0);
- 16:08 Left wheels degraded (k 2.3).

### A3. The same replays with the track cut at 3 cm jumps (`night_trust_replay.py --split 3`)
| Replay | Shared term cm / deg | "Shared unexplained" | Gate at the end | Flagged at the end |
|---|---|---|---|---|
| Stage 1 alone | 1.56 / 2.53 | no | 2.50 | Left wheels (k 2.2) |
| Stage 2 alone | 1.48 / 2.01 | no | 2.40 (1.34-2.48 along the way) | none (Center wheels and Right gyro flagged 16:46, cleared 16:48) |
| Stage 1 → 2 | 1.50 / 2.04 | no | 2.42 | Left wheels (k 1.6, hysteresis), Right front (k 2.6) |
| Laptop morning | 0.51 / 0.31 | no | 1.18 | none |

Every wheel fit in these replays passes, both at the trust gate and at 1.5 cm.

### B1. The cut-size sweep (wheel fit rms cm per stretch (stretches, cm counted); robust sd of all; gyro rms °)
| Session | Cut | Left | Center | Right | Robust sd | Gyro L/C/R |
|---|---|---|---|---|---|---|
| 23rd morning | none | 1.20 (52, 1691) | 1.17 (54, 1646) | 1.50 (62, 1588) | 1.35 | 1.6/1.2/1.5 |
| 23rd morning | 3 cm | 0.84 (55, 1622) | 0.79 (52, 1469) | 0.88 (58, 1277) | 0.77 | 1.2/1.1/0.9 |
| 23rd evening | none | 1.66 (33, 945) | 1.49 (23, 666) | 1.69 (31, 935) | 1.88 | 2.5/2.2/2.0 |
| 23rd evening | 3 cm | 1.35 (30, 867) | 0.97 (18, 598) | 0.94 (32, 778) | 1.31 | 1.6/1.8/1.9 |
| Stage 1 | none | 1.82 (18, 415) | 1.21 (18, 576) | 1.89 (24, 576) | 2.20 | 2.5/2.9/3.0 |
| Stage 1 | 4 cm | 1.30 (15, 406) | 1.17 (17, 554) | 1.11 (22, 531) | 1.38 | 2.0/2.8/2.2 |
| Stage 1 | 3 cm | 1.27 (15, 406) | 1.17 (17, 554) | 0.88 (21, 489) | 1.32 | 2.0/2.8/2.1 |
| Stage 1 | 2 cm | 1.12 (12, 374) | 0.90 (17, 513) | 0.61 (18, 333) | 0.93 | 1.9/2.0/2.2 |
| Stage 2 | none | 2.35 (29, 923) | 2.34 (29, 890) | 2.09 (33, 810) | 3.30 | 3.1/2.9/3.4 |
| Stage 2 | 4 cm | 1.35 (25, 886) | 1.21 (26, 842) | 1.20 (26, 749) | 1.20 | 2.1/2.3/2.5 |
| Stage 2 | 3 cm | 1.28 (26, 858) | 1.10 (26, 785) | 0.80 (26, 649) | 0.92 | 2.0/1.9/2.5 |
| Stage 2 | 2 cm | 1.07 (30, 733) | 0.94 (27, 677) | 0.58 (26, 522) | 0.86 | 2.0/1.8/2.2 |

Wheel scale per robot, as run / cut at 3 cm:

| Session | Left | Center | Right |
|---|---|---|---|
| 23rd morning | 0.991 / 0.988 | 0.993 / 0.989 | 1.021 / 1.000 |
| 23rd evening | 0.976 / 0.975 | 0.971 / 0.961 | 1.003 / 0.992 |
| Stage 1 | 0.963 / 0.947 | 0.981 / 0.978 | 1.000 / 0.998 |
| Stage 2 | 0.975 / 0.960 | 0.988 / 0.975 | 1.002 / 0.984 |

### B2. Coverage of the error map (stretch ends against the finds; map from `camera_error_samples`)
| Distance from the finds | Stage 1 share / map applied / map sd | Stage 2 share / map applied / map sd |
|---|---|---|
| 0-25 cm | 38 % / 100 % / 1.3 cm | 30 % / 100 % / 1.1 cm |
| 25-50 cm | 17 % / 100 % / 1.5 cm | 18 % / 100 % / 1.2 cm |
| 50-75 cm | 24 % / 62 % / 2.2 cm | 28 % / 96 % / 1.7 cm |
| 75 cm or more | 22 % / 0 % / 3.0 cm | 24 % / 0 % / 3.0 cm |

### B3. Shared camera-scale model, leave one run out (cm: rms / robust sd of the left-out run's stretches)
| Session | Wheel scale per robot only | Plus the shared camera terms |
|---|---|---|
| 23rd morning | 2.39 / 1.36 | 2.33 / 1.05 |
| 23rd evening | 3.14 / 1.93 | 3.00 / 1.83 |
| Stage 1 | 3.09 / 2.40 | 3.17 / 2.51 |
| Stage 2 | 4.29 / 3.58 | 3.95 / 3.87 |

The calibration's own scale (session-1 roam1 file) runs from 0.455 cm/px at (10, 10) to 0.399 at (112, 112). It is
the same file on the laptop and on the Pi.

### C1. Battery readings used
| Robot | 23rd morning | 23rd evening | Pi stages 1-3 |
|---|---|---|---|
| Left | 08:51 98 % → 11:04 63 % | 19:14 88 % → 19:53 77 % | 15:36 88 % → 17:33 55 % |
| Center | 08:51 98 % → 10:58 67 % | 19:14 73 % → 19:56 62 % | 15:36 88 % → 17:33 58 % |
| Right | 08:51 98 % → 11:01 68 % | 19:14 87 % → 19:58 77 % | 15:36 90 % → 17:33 63 % |

Standing gaps (last reading before, first after):
- **Stage 1 → 2:** Left 79 → 77 % in 8 min, Center 83 → 78 % in 20 min, Right 83 → 81 % in 11 min.
- **Stage 2 → 3:** Left 56 → 56 % in 3 min, Center 60 → 58 % in 10 min, Right 64 → 63 % in 5 min.
