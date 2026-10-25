# Logs-only DOE rows, run on the sessions on disk (2026-10-25, no robots)

Nick's standing approval: every DOE backlog row whose "Needs" is Logs runs without asking ("always better to utilize
non-board time to strengthen the use of board time"). This report covers those rows, in the order the paper needs
them. The front sensor offset and tolerance row was done separately and is not here.

**Data.** Session logs on disk: 2026-10-23 morning `e02-20261023-085107` and evening `e02-20261023-191347` (laptop);
2026-10-24 Pi stages 1-3 `e02-20261024-153612`, `-161612`, `-173242`; the Right-alone night session
`e02-20261024-193526`. For the uncertainty model also the five 2026-10-22 sessions in the GitHub release
`e02-session-oct22-evening` (`logs.zip`; older firmware). No footage was downloaded (no row here could be scored
from video without hand-checked truth; see row 12).

**Scripts** (offline, read only the session folders): `tools/experiment/analyses_oct25/`. Shared loaders:
`oct25_common.py` (builds on `analyses_oct24/night_common.py`). Outputs and printed tables: `analyses_oct25/out/`.

**One method note that several rows rest on.** To score a calibration setting by the end error it would have given,
`oct25_common.reintegrate()` recomputes the robot's own pose from a run's raw wheel and gyro rows, the firmware's way
(`pose.h`), with any gyro scale, drift and wheel scale. Checked: with the values the run used, it lands on the
robot's own logged end pose to within 0.02 cm on every run without camera fixes. The robot's own displacement is then
compared with the camera's, at the route's end, at the far point, and at every leg pause along the route.

## Results

| # | Row (constant, value now) | Result in one line | Status |
|---|---|---|---|
| 1 | Uncertainty model (`UNC_*`: 0.5 cm, 1 %, 0.5°, 0.5 %) | Too confident: 59 % of 673 corrections fall inside the model's 95 % range. Refit: turning counts 2.7 × more (0.0136 per degree, was 0.005); after 50 cm and 180° the sd is 3.6 cm, not 2.2. 17 % of chains end with the heading 30°+ wrong, which no setting of these four numbers covers | done (logs part); Sim next |
| 2 | Gyro drift band (`DRIFT_BAND_DPS` 0.05 °/s) | Holding the drift at the measured standing value (band 0) predicts where the robot went best: 0.12-0.15 cm closer at the far point and along the path than 0.05 (paired, p < 0.001, 48-49 runs). Letting the drift float is worst | done: 0 recommended |
| 3 | Tracker jump cut (`JUMP_CM` 3 cm) | 2, 3 and 4 cm cuts give the same predicted errors (differences ≤ 0.09 cm, p > 0.2). No cut is slightly worse at the end (+0.12 cm, p 0.016) and the same elsewhere | done: keep 3 |
| 4 | Wheel fit accepted (`MAX_RMS_CM` 1.5, 2.5 for sessions) | No effect on predicted end errors: with the 3 cm cut every fit on disk passes 1.5 cm; without it, 1.5-3 cm differ by ≤ 0.01 cm. Bigger finding: the fixed prior predicts the path as well as or better than the session's own fit | done |
| 5 | Explore calibration accepted (`MARK_ACCEPT_CM` 2.5) | No real session has finished explore with `--marks` (stage 3 stopped during identify at 17:38) | needs more data |
| 6 | Recheck before "lost" (`RECHECK_S` 3.5 s) | The replay finds the runner's own disagreements (35 of 39), but the runner then refound at once (median 1.9 s), so 41 of 46 disagreements can't be followed to 3.5 s. Of the rest: 1 rescued at 1 s, 1 still wrong at 6 s | needs more data |
| 7 | Robot found where its wheels put it (`DETECT_NEAR_CM` 6 cm) | 5 pick-ups at 2.4-5.2 cm, all followed by fixes that agree; at 4 cm two good ones would have been refused. Outlines 6-9 cm away are not logged, so 9 cm can't be scored | needs more data |
| 8 | Heading before a find (`FIND_HEAD_AGREE_DEG` 5°) | 39 finds: the camera's heading was the better one (median error 3.0° against 4.1°). At 8-10° no find starts more than 10° wrong (one does at 5°) | done (weak): 10 suggested |
| 9 | Evidence before a sensor's k is used (`MIN_N`) | Wheels and gyro at 3: 3 false flags in 4.8 healthy hours; at 5: none on a fresh file. Camera and front minimums change nothing here | done (healthy half): 5 suggested, swap test to confirm |
| 10 | Other trust limits (`GATE_K`, `GATE_MAX_CM`, `DECAY_PER_SESSION`, `KC_GATE_SIGMA`) | Healthy half: `GATE_K` 1.2 refuses 2 good fits, 1.5 and 2 none; `GATE_MAX_CM` is never reached; decay 0.25 adds one carried flag; `KC_GATE_SIGMA` 3-5 refuse the same 2 KC stops (16 and 19 cm off: rightly) | done (healthy half); keep all; Sim for faults caught |
| 11 | Camera drift watch (`PERIOD_S` 5 s, `WARN_PX` 1 px) | On the Pi sessions the camera moved at most 0.43 px (0.2 cm), less than the robots' own board touches scatter (0.11-0.61 cm). With and without the correction can't be told apart here | needs more data (a session with real drift) |
| 12 | Tracker replay rows (`CHECK_S`, `AGREE_CM`, `LOCK_SURE`, `PIN_CM`, `TURN_STEP_DEG`, `MATCH_OK`, `MIN_LOOK`, health limits Part A) | Not run. The response (time with the position more than 5 cm wrong) needs a hand-checked position about once a second; the clips have 3-6 checked fixes each | needs hand-checked truth |
| 13 | Sample-size minimums (publication plan, Appendix B) | Preliminary only. With all 13 C runs on disk, the per-arm count for the 1 cm equivalence test is 25 with a robust sd and 247 with the ordinary sd: one 13 cm run decides it | preliminary; week-3 calc stays |
| - | Waiting on other data | Side camera rows (no per-region trust map; no footage where the overhead loses robots), camera error map rows (tape marks), bottom zone reach (crawl runs), charge levels (bench test 1), fusion settings (another lane is building it) | not run |

<sub>p values: Wilcoxon signed-rank test, a paired test on ranks. It asks whether one setting beats the other run by
run, consistently, without assuming the errors are normal. Small p (under 0.05) = the difference is consistent.</sub>

## 1. Uncertainty model *(conclusions: Claude)*
**Premise.** The runner keeps each robot's position uncertainty (1 sd per axis) from what it drove and turned since
its last camera fix: 0.5 cm + 1 % of the distance + the distance × (0.5° + 0.5 % of the degrees turned). It pauses
for a fix when that would pass 3 cm. DOE-confidence.md says: check the constants against the corrections first. The
2026-10-24 dry run said ~4 cm where the simulator's truth was 1.6 cm.

**Data.** Every camera fix the runner sent between runs (`fix.csv`: the robot's own pose just before the fix against
the camera's), with the driving and turning since its previous fix or find from `leg.csv`. 673 chains from 10
sessions, kept only where the legs' own poses join up. `confidence.csv` alone has 22 corrections; it is checked too.

| Since the last fix | n | Correction median (cm) | 90 % | Model's sd | sd the data implies |
|---|---|---|---|---|---|
| 0-10 cm driven | 448 | 0.88 | 4.2 | 0.64 | 0.75 |
| 10-25 cm | 63 | 5.4 | 25 | 1.0 | 4.6 |
| 25-50 cm | 50 | 16 | 42 | 2.1 | 14 |
| 50-100 cm | 111 | 7.3 | 88 | 3.2 | 6.2 |

<sub>A 2-D error whose x and y parts each have the same sd has a median length of 1.18 sd and falls under 2.45 sd
95 % of the time (a Rayleigh distribution); "implied sd" = median ÷ 1.18.</sub>

The large ones come with a large heading correction at the same fix:

| Heading corrected at the fix | Share of chains | Position correction median | 90 % |
|---|---|---|---|
| under 10° | 73 % | 0.98 cm | 5.0 cm |
| 10-30° | 10 % | 5.2 cm | 29 cm |
| 30° or more | 17 % | 22 cm | 83 cm |

Fitted (maximum likelihood, with a share of outliers allowed), on the chains whose heading stayed within 30°:
base 0.63 cm, 1 % per cm (unchanged), 0.43°, **0.0136 per degree turned**. After 50 cm and 180° that gives 3.6 cm
(now 2.2). It still leaves 15 % outliers, and 78 % (not 95 %) of corrections inside 2.45 sd. On all chains the best
fit puts the turning term at 0.13 per degree, which only says the heading failures dominate.

The pauses in `confidence.csv` show the same: median correction 8.2 cm against a predicted 2.9 cm; 10 of 22 over
10 cm (one of them is shown below).

**Conclusion.** The model is too confident wherever the robot turned, and it has no term for the main failure: a
robot whose heading has gone 30° or more wrong (17 % of chains). Where a heading came with the previous fix, the
next correction was smaller (median 1.3 cm) than where it didn't (4.3 cm). An example from stage 1 (15:51): after
an uncalibrated U run, the fix set Right's position but kept its own heading (−135.9°, wrong by about 33°); 75 cm
later the next fix was 37.8 cm off. The refit numbers move the pause earlier: at 3 cm a straight drive after a
half turn now pauses after about 39 cm, not 73 cm.

**What's next.** Apply the refit (Recommended changes), then rerun the confidence-pause simulator grid before the
board DOE. Separately: find where the ≥ 30° heading errors come from. The candidates are a fix that keeps a wrong
heading (this example) and a wrong camera heading handed over at a fix. Row 8 is one piece of that.

## 2. Gyro drift band *(conclusions: Claude)*
**Premise.** The self-calibration may let the fitted gyro drift differ from the robot's measured standing-still
drift by at most 0.05 °/s. Should the band be 0.02, 0.05 or 0.1?

**Data.** 49 runs (36 U, 13 C; no camera fixes inside them) from the four classic sessions. Each run's calibration is
fitted from the robot's other U runs of the same session ("leave one run out"), at each band, and the run is
re-integrated with it. Errors are the robot's own displacement against the camera's.

| Band (°/s) | End error, mean (cm) | Far point, median | Along the path, median | Paired against 0.05 (far / path) |
|---|---|---|---|---|
| 0 (drift = measured) | 2.24 | 3.97 | 3.36 | −0.15 / −0.12 cm, p < 0.001 |
| 0.01 | 2.21 | 4.02 | 3.39 | −0.12 / −0.08, p < 0.001 |
| 0.02 | 2.20 | 4.07 | 3.41 | −0.08 / −0.04, p < 0.001 |
| 0.05 (now) | 2.44 | 4.23 | 3.55 | - |
| 0.1 | 2.85 | 4.38 | 3.55 | +0.00 / +0.00, p 0.05 / 0.003 |
| free | 3.28 | 4.50 | 3.55 | worse (p 0.046 / 0.003; at the end, p < 0.001) |

<sub>Cut 3 cm, wheel gate 1.5 cm. "Along the path" = rms over every leg pause of the route. End errors are small for
every setting because the routes come back to their start, where scale errors partly cancel.</sub>

**Conclusion.** The tighter the band, the better, and band 0 is best or tied on every measure. The fitted drift
always ran to the band's edge (±0.05 in every session, against a measured 0.000), so it is absorbing something
else, not gyro drift. The difference is small (0.1-0.2 cm) but consistent.

**What's next.** Set the band to 0 (Recommended changes): the drift is the measured one, the gyro scale is fitted.

## 3. Tracker jump cut *(conclusions: Claude)*
**Premise.** The camera track is cut at one-picture jumps over 3 cm before the straight stretches are found (night
analyses B6). The 2/3/4 cm sweep only scored the fit's own rms. Which cut predicts real errors best?

**Data.** The same 49 runs, cut at none / 2 / 3 / 4 cm, at band 0 (paired against 3 cm):

| Cut | End error (median diff) | Far point | Path |
|---|---|---|---|
| none | +0.12 cm, p 0.016 | −0.02, p 0.21 | +0.02, p 0.47 |
| 2 cm | +0.02, p 0.65 | −0.02, p 0.89 | +0.01, p 0.42 |
| 4 cm | +0.09, p 0.23 | +0.01, p 0.15 | +0.02, p 0.28 |

**Conclusion.** Within 2-4 cm the cut size does not matter for where the robot ends up. Cutting at all helps a little
at the end. The cut's real value is that fits stop being refused (row 4).

**What's next.** Keep 3 cm. No board DOE needed for it.

## 4. Wheel fit accepted, and the fit against the prior *(conclusions: Claude)*
**Premise.** A wheel fit whose camera-against-wheels rms exceeds 1.5 cm is refused (2.5 cm in sessions, Nick
2026-10-24). Does accepting more fits give better runs?

**Data.** The same runs, fitted only from the U runs before them (as the runner would). Gates 1.5 / 2 / 2.5 / 3 cm /
none: with the 3 cm cut every fit on disk passes 1.5 cm, so the gate changes nothing. Without the cut: mean end
error 2.93 cm at 1.5, 2.92 at 2.5-3.

Then each fitted part against the fixed prior (gyro 1.075, drift as measured, wheels 0.979), paired, 44-49 runs:

| Calibration used | Far point, median diff | Path | Path after the best start-heading turn |
|---|---|---|---|
| Fitted (all parts) | −0.02, p 0.92 | **+0.22 cm, p < 0.001** | **+0.23, p < 0.001** |
| Fitted wheel scale only | **−0.13, p 0.01** | +0.08, p 0.013 | +0.16, p 0.001 |
| Fitted gyro scale only | 0.00, p 0.28 | 0.00, p 0.67 | 0.00, p 0.11 |
| Fitted drift only | +0.18, p < 0.001 | +0.15, p < 0.001 | +0.01, p 0.04 |

<sub>"After the best start-heading turn": each run's track is turned about its start to match the camera best, which
takes out a wrong start heading that no calibration changes. The fitted values per session are in the appendix.</sub>

**Conclusion.** The gate doesn't matter on this data. The bigger result: the session's own fit doesn't predict where
the robot went any better than the fixed prior. The drift part makes it worse (row 2). The wheel part helps at the far
point and hurts slightly along the path. The camera is the only reference here, so a camera scale error across the
board would look like this too.

**What's next.** Keep 1.5 cm; the 2.5 cm session override is no longer needed once the jump cut is in. The question
of fit against prior needs a reference that isn't the camera: the tape marks (the camera error map rows of the backlog).

## 5. Explore calibration accepted
**Premise / data.** The row needs explore's mark error against the runs' errors, from sessions run with `--marks`.
The only real explore session (stage 3, 2026-10-24 17:32) stopped during identify (17:38) and logged no marks.
**Conclusion.** Nothing to score. **What's next.** Run explore to the end with `--marks` (Board row).

## 6. Recheck before "lost" *(conclusions: Claude)*
**Premise.** After a go leg where the camera and the wheels disagree, the runner now waits 3.5 s and looks again
before calling the robot lost (one event: a ghost held for 3 s). Replay the tracks at 1-6 s.

**Data.** `recheck_replay.py` puts all 677 runner-driven go legs on disk through the same check (`cross_check`, trust
factors at 1). It flags 46. 35 of the runner's 39 logged camera disagreements are among them, so the replay matches.
The sessions before 2026-10-24 19:39 refound at once, so the robot moved again a median 1.9 s after the leg (10-90 %:
1.0-4.1 s).

| Look again after | Rescued | Still disagreeing | No still reading | Robot moved first (can't say) |
|---|---|---|---|---|
| 1 s | 1 | 4 | 36 | 6 |
| 3.5 s | 1 | 1 | 14 | 31 |
| 6 s | 1 | 1 | 1 | 44 |

**Conclusion.** The old sessions can't score this: by 3.5 s almost every disagreement had already turned into a
refind. Disagreements are rare (46 in 677 legs, 7 %), so a 3.5 s recheck costs about 0.25 s a leg on average.

**What's next.** Every session from now on runs with the recheck. Count rescues against holds from its log line
("camera looked again: agrees with the wheels") over the next three sessions, as the row says.

## 7. Robot found where its wheels put it
**Data.** 5 `detect_after_move` records (stages 1-2): picked up 2.4, 3.4, 4.7, 3.3, 5.2 cm from the wheels' pose. The
next fix agreed with the robot (corrections 1.8-2.8 cm; 6.5 cm after 34 s of driving), and no camera-wheels
disagreement followed within 60 s. **Conclusion.** All five look right. At 4 cm two of them would have been refused
(a turn on its spot instead). Nothing says what 9 cm would add, because outlines further than 6 cm are not logged.
**What's next.** Log the nearest refused outline's distance too (Recommended changes); score after the next sessions.

## 8. Heading before a find *(conclusions: Claude)*
**Premise.** Before a find, the runner hands the robot the camera's heading only if it agrees with the robot's own
gyro heading within 5°. Otherwise it keeps its own heading.

**Data.** 39 finds (stages 1-2) with the check matched to the find. The find's first board touch measures how wrong
the heading was (`corr1_deg`). The other candidate's error follows from the gap between the two headings.

| Limit | Camera heading used | Heading error median | 90 % | Finds over 10° wrong |
|---|---|---|---|---|
| 0 (never) | 0 | 4.1° | 8.6° | 2 |
| 5° (now) | 27 | 3.3° | 8.4° | 1 |
| 8° | 34 | 3.2° | 7.0° | 0 |
| 10° | 35 | 3.0° | 6.7° | 0 |
| any | 36 | 3.0° | 7.0° | 1 |

**Conclusion.** The camera's heading is usually the better one, and 8-10° lets more of it through without letting in
the one 12° disagreement where the camera was wrong. Weak evidence: 39 finds, one day, and the heading error includes
the find's own turn to the board.

**What's next.** 10° (Recommended changes); recheck on the next sessions' `find_heading_checks`.

## 9-10. Sensor trust limits on healthy sessions *(conclusions: Claude)*
**Premise.** How many false flags, and refused good fits, does each trust setting give on sessions with no known
fault? (Assumption: none of these sessions had a faulty sensor.) The faults-caught half is the simulator swap test.

**Data.** `trust_healthy_replay.py`: the night analyses' trust replay (current e02_fit, 3 cm cut) over the 23rd
morning and evening and the 24th stages 1-2, 4.8 h of track. Each run uses a fresh trust file, plus stage 1 → 2 carried.

| Setting | False flags (fresh files) | Carried 1 → 2 | Good wheel fits refused (of 50) |
|---|---|---|---|
| `MIN_N` wheels/gyro 3 (now) | 3 (0.6 an hour): Left wheels, Right wheels, Center gyro | 1 | 0 |
| `MIN_N` wheels/gyro 5 | 0 | 2 | 0 |
| `MIN_N` wheels/gyro 8 | 0 | 1 | 0 |
| `MIN_N` front 4 / 6 / 8 / 12 | 4 / 3 / 3 / 3 | 1 / 1 / 0 / 0 | 0 |
| `MIN_N` camera 3-8 | 3 at every level | 1 | 0 |
| `GATE_K` 1.2 / 1.5 / 2.0 | 3 | 1 | 2 / 0 / 0 |
| `GATE_MAX_CM` 2 / 2.5 / 3 | 3 | 1 | 0 (the gate never reaches the cap) |
| `DECAY_PER_SESSION` 0.25 / 0.5-1.0 | 3 | 2 / 1 | 0 |

`KC_GATE_SIGMA` on stage 2's 10 KC stops: at 3, 4 or 5 sds the same 2 beliefs are refused (camera 16 and 19 cm from
the belief, combined sd 1.2-2.5 cm: the belief was wrong). At 2 sds two more are refused at 2.8 and 3.6 cm.

**Conclusion.** At 3 fits a robot is flagged on too little evidence (the night analyses saw the same thing). At 5,
the healthy sessions give no false flags on a fresh file. The cost is a later catch: about 5 U runs before a real
fault can be flagged. The other limits change nothing on healthy data at today's values.

**What's next.** `MIN_N` wheels and gyro 3 → 5, after the simulator swap test checks the delay. The carried replay's
flags (the front's rests on one +46 cm reading) belong to the front-sensor work already done.

## 11. Camera drift watch
**Data.** Stages 1-2 logged 1,162 shift measurements (`camera_shift`). The most the camera moved was 0.42-0.43 px
(0.17-0.20 cm on the board), never over the 1 px warning. Within 5 s it changed a median 0.04 px, within 60 s at most
0.28 px. The robots' own board touches scatter 0.11 cm (y-board) to 0.37-0.61 cm (x-board) sd.
**Conclusion.** On these sessions the correction moves positions by less than the touches' own scatter, so with and
without can't be told apart. A 60 s period would have missed nothing here.
**What's next.** Score it on a session where the camera really drifts. The 2 px drift seen earlier is in footage
with no shift log, so that needs a video replay.

## 12. Tracker replay rows
**Premise.** DOE-tracking.md DOE 1 and its "other chosen numbers" score each setting by the time the tracker's
position is more than 5 cm wrong. The truth is a position hand-checked once a second from a contact sheet.
**Data.** The clips in `tests/data/realclips` carry 3-6 eye-checked fixes each, not one a second; no other
hand-checked truth exists. **Conclusion.** These rows can't be scored without that truth. Seen share and ms per
picture alone would rank settings by cost, not by being right. **What's next.** Contact sheets for the three
2026-10-22 evening clips (about 6 footage minutes, so about 6 minutes of hand-checking), then the 9-cell replay.

## 13. Sample-size minimums (preliminary)
**Data.** Per-arm count for the C vs C-conv equivalence test (1 cm margin, power 0.8), from the C-arm sd:

| Source | n | sd → per arm | robust sd → per arm |
|---|---|---|---|
| C runs, 23rd morning | 9 | 0.76 → 10 | 0.87 → 14 |
| C runs, every session on disk | 13 | 3.79 → 247 | 1.20 → 25 |
| Re-integrated runs (row 2, band 0) | 49 | 2.08 → 74 | 1.16 → 24 |

<sub>Robust sd = median absolute deviation ÷ 0.6745: equals the sd for normal data, ignores a few outliers.</sub>

**Conclusion.** The count depends mostly on how the analysis treats the tail (the 23rd evening's single C run ended
13 cm off). With outliers handled, about 25 runs per arm; without, the test is out of reach.
**What's next.** Fix the outlier rule in the analysis plan before the week-3 count.

## Recommended changes (for Claude/Nick to apply; no code was changed)
1. **`e02_fit.DRIFT_BAND_DPS` 0.05 → 0** (hold the drift at the measured standing value). Evidence: row 2, band 0
   beats 0.05 by 0.15 cm at the far point and 0.12 cm along the path, p < 0.001, 48-49 runs; the fitted drift
   sat at the band edge in every session.
2. **`e02.py UNC_DEG_PER_DEG` 0.005 → 0.0136 and `UNC_BASE_CM` 0.5 → 0.63** (`UNC_DEG` 0.5 → 0.43 optional, it
   barely matters; `UNC_PER_CM` stays). Evidence: row 1, 551 chains, best fit by AIC; the model now covers 59 %
   of corrections inside its 95 % range. Expect about twice as many confidence pauses at 3 cm. Rerun the
   confidence-pause simulator grid after it.
3. **`e02_trust.MIN_N` wheels and gyro 3 → 5**, once the simulator swap test confirms the slower catch is
   acceptable. Evidence: row 9, 3 false flags in 4.8 healthy hours at 3, none at 5.
4. **`Runner.FIND_HEAD_AGREE_DEG` 5 → 10.** Evidence: row 8, 39 finds; weak.
5. **Sessions back to the 1.5 cm wheel gate** (no `--fit-max-rms-cm 2.5`). Evidence: row 4, no fit on disk needs
   more than 1.5 cm once the jump cut is in, and the gate does not change predicted errors.
6. **Logging only:** in `detect_after_move`, record the nearest refused outline's distance, so `DETECT_NEAR_CM`
   can be scored (row 7).

## Questions for Nick
Substantive:
- Row 4: the fixed prior predicted where the robots went as well as each session's own fit. Do we want the tape-mark
  session soon, so the paper's "self-calibration" claim gets a reference that isn't the same camera?
- Row 1: 17 % of between-run stretches ended with the heading 30°+ wrong. Should finding that source come before the
  confidence-pause DOE? (My suggestion: yes. The pause can't fix a heading the uncertainty model doesn't know is wrong.)

Other:
- Row 12: are you willing to hand-check about 6 minutes of contact sheets (about 6 minutes of your time) so the tracker
  rows can run?

## New tests that may now be needed
1. Simulator: the confidence-pause grid again with the refit uncertainty constants (row 1).
2. Where the ≥ 30° heading errors come from: replay the fixes before them, with the heading each carried (row 1).
3. A unit test that a camera fix after an uncalibrated run also corrects the heading, or that the uncertainty
   starts high when it doesn't (row 1's example).
4. Simulator swap test with `MIN_N` 3 / 5 / 8: detection delay against false flags (row 9).
5. A reference for calibration that isn't the camera (tape marks), to settle fit against prior (row 4).
6. Contact sheets and the 9-cell tracker replay (row 12).

## Possible inaccuracies
- **The camera is the reference everywhere.** Rows 1-4 compare the robots against the same camera that calibrates
  them. A camera scale or tilt error across the board would look like a calibration error.
- **Chains (row 1)** assume nothing moved the robot between a fix and the next one except the logged legs. The
  check that the legs' poses join up removes the clear breaks (31 of 704), not every one. 2026-10-22 sessions ran
  older firmware.
- **Row 1 fit choices:** the outlier part is a second Rayleigh with an sd of 15 cm (chosen); the "heading within 30°"
  subset is chosen (it selects on part of the outcome, which biases the turning term downward if anything).
- **Re-integration (rows 2-4)** gives the error the robot's own belief would have had; it assumes the robot would have
  driven the same path. With a different belief it would have steered differently, so these are errors of knowing,
  not of driving.
- **Leave one run out (rows 2-3)** trains on runs after the scored one, which the runner can't. Row 4 uses only
  earlier runs.
- **Healthy sessions (rows 9-10)** are assumed fault-free; any real fault there would count as a false flag.
- **The heading error at a find (row 8)** includes the find's own turn and drive to the board.
- **Recheck replay (row 6)** uses trust factors of 1 and no fresh-identify allowance; the runner's check widens with
  both.
- **Chosen numbers in this report:** windows of 0.5 s for the camera's start and end, 0.25 s around leg pauses; a leg
  pause = both wheels under 0.5 rpm for 0.6 s; chains within 10 min; the 2 cm allowance for chain joins; 60 s to
  "no disagreement followed" (row 7). A DOE would not add much to any of them; a sensitivity check would.

---

## Appendix

### A1. Fitted values per session (leave one run out, cut 3 cm, band 0.05; median over the robot's fits)
| Session | Left gyro / drift / wheels | Center | Right |
|---|---|---|---|
| 23rd morning | 1.076 / +0.050 / 0.987 | 1.082 / +0.039 / 0.990 | 1.077 / +0.035 / 1.003 |
| 23rd evening | 1.091 / −0.050 / 0.973 | 1.071 / −0.050 / 0.961 | 1.076 / −0.045 / 0.989 |
| Stage 2 | 1.087 / −0.050 / 0.959 | 1.075 / −0.050 / 0.976 | 1.081 / −0.050 / 0.983 |

### A2. Uncertainty fits (maximum likelihood; AIC lower is better)
| Fit | Chains | AIC | base | per cm | deg | per deg turned | outlier share | inside 2.45 sd |
|---|---|---|---|---|---|---|---|---|
| Model now | all 673 | 4303 | 0.50 | 0.010 | 0.50 | 0.0050 | 0.33 | 59 % |
| base + deg + per deg | heading within 30°, 551 | 2169 | 0.63 | 0.010 | 0.43 | 0.0136 | 0.155 | 78 % |
| Model now | heading within 30°, 551 | 2272 | 0.50 | 0.010 | 0.50 | 0.0050 | 0.20 | 70 % |
| base + deg + time | heading within 30°, 551 | 2178 | 0.46 | 0.010 | 1.40 | 0.0050 (+0.0016 cm/s) | 0.164 | 77 % |

<sub>AIC (Akaike information criterion): how well a model fits, with a penalty for each number it fits; compare
within the same set of chains only.</sub>

Full printouts: `analyses_oct25/out/unc_fit.txt`, `fit_settings_summary.txt`, `fit_settings_pairs.txt`,
`fit_parts_summary.txt`, `recheck_replay.txt`, `find_heading.txt`, `trust_healthy.txt`, `camera_shift.txt`,
`detect_near.txt`, `power_prelim.txt`.
