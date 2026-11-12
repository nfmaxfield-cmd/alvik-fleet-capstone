# Independent audit of alvik-fleet-capstone (12 Nov 2026, review only)

## Premise

- Audited: `nfmaxfield-cmd/alvik-fleet-capstone`, main at eeb34b3 (e18), then 1dc5299 (the hard simulator, added mid-audit at the coordinator's request).
- Standard: the repo's CLAUDE.md rules (switches, discover, observe then trust, the simulator breaks the guesses, thresholds from data, never give up, never still).
- Nothing was committed, pushed or edited in the repo. Simulator, tests and logs only. No robot, Pi or camera was touched.
- **Verified** = I ran it and saw it, or it is a plain reading of the cited lines. **My reading** = my own interpretation, not the team's, and not tested.
- My check scripts are in `/tmp/claude-0/-home-claude/fff6dc9c-5876-54f9-92a7-a08483d3d5c7/scratchpad/work/` (Appendix C).

## Findings

Severity: **H** = safety, or a wrong result the run trusts. Paths are under `tools/fleet/` unless said.

| # | Sev | What is wrong | Where | How verified | DOE? |
|---|---|---|---|---|---|
| 1 | H | **A free-roam run can end without sending `stop`.** At any end, the run saves calibration, state and workspace first and sends `stop` last. If a save fails (a full card), `stop` is never sent, and the simulator and bridge are not closed either. | `freeroam.py:1530-1544` | Ran a simulated free roam, made the end-of-run state write fail with "No space left": `close()` was never called, no `stop` went out. On a normal Ctrl+C the `stop` left 0.37 s after "every robot is sent stop" was printed. | no |
| 2 | H | **Simulator ("dry") free roams write into the real robots' calibration pool and the real site's workspace.** Only the send to the robot is skipped. The next real run sends the pooled value to the robot's flash (`corr save wheel_scale`). | `freeroam.py:351-353`, `363-372` | My three dry runs added three rows to `~/.config/mirlt/robots/Left.json` and changed its pooled value; `workspace/home.json` was rewritten. | no |
| 3 | H | **The out-and-back names the wrong robot when the camera noise is detector-sized.** The drive is capped at 4 cm, so above about 0.9 cm of noise the "moved" bar drops under the 4-sigma it is described as. The bar is also applied to the 3 farthest sightings of a whole watch, which grows with the number of pictures. | `tracking_layer.py:746-760`, `857-879` | Layer alone, 200 trials a cell. Lights noise 0.4-0.8 cm: 200 of 200 right. Detector noise 1.6 cm (tag-free's own number), robot really drives: first name right 94, **wrong 33**, none 73. Nobody moves at all (a lost command), noise 1.5 cm: a robot is named anyway in 16-40 % of trials. | **yes**: the cap, the far-end share and the bar, on real out-and-backs for each source |
| 4 | H | **Falls come from wrong camera corrections, then a reverse creep.** A correction puts the robot's pose tens of cm from the truth; it is then "not lost" with a small doubt; never-still's creep reverses it (no edge rule, no rear sensor) off the board. | corrections: `freeroam_core.py:1416-1506` ("named far" has no upper limit and no motion proof); creep: `1673-1785`; gate feedback: `tracking_layer.py:395-401` | Traced 7 falls in the hard simulator (5 harsh seeds, 1 wrong-names run). All 7: already hung, then reversed (5 "creep back", 2 recovery reverses). Run's position 38-137 cm from the truth, stated doubt 3-16 cm. Last correction 23-91 cm wrong in 6 of 7: "named far" 3, wiggle 2, elimination 1. | no |
| 5 | H | **One hung robot stops another, free robot from creeping.** With two robots unaccounted, only the holder of a "creep token" may creep. A hung robot re-creeps (in vain) before its own token runs out, so it holds the token for good. The "nobody waits more than three pauses" escape sits after the token check. | `freeroam_core.py:1819-1832` | Reproduced the hard simulator's 311 s stand (skew, seed 2): in 5,794 of 5,912 ticks Right was refused because Left (hung) held the token. Left sent 320 creeps in that run. This **refutes** the hard-simulator report's reading (a doubtful-neighbour wait). | no |
| 6 | H | **A hung robot is never freed.** (a) A failed recovery try is remembered for ever at that spot, so every 20 s retry skips all its tries. (b) No try drives forward, and the firmware refuses forward while tilted, so a robot that **backed** over the edge has no way out. (c) Its creeps are sent and refused. | `observe.py:801-822` (no expiry); `safety_rules.py:533-552`; `freeroam_core.py:3695-3748` | Hard simulator, measured seed 2: Left hung 267 s with its nose 4 deg from straight inward (it had reversed over the edge on a pose 11 cm wrong). Skew seed 2: "pivot_right: skipped" 54 times, "hold" 57 times. | no |
| 7 | M | **Two savers fight over `wheel_scale`, and one saves noise.** The start's discovery saves a pooled value; e18's end-of-run saver multiplies a ratio by the value read **before** discovery changed it. The next start overwrites e18's value. e18's rule checks the spread only, not whether the correction differs from 1. Discovery's `corr_now` is never set, so a relative fit is saved as an absolute value. | `freeroam.py:791-795`, `351-353`; `freeroam_core.py:2305-2325`; `start_discovery.py:469-478` | Stop-turn simulated run: "wheel_scale 1.0000 -> 1.0068 (+-0.013)", 0.9916 (+-0.011), 1.0069 (+-0.012): three corrections inside their own error bar, all saved. `grep corr_now`: read once, never written. | no (a significance test, not a number) |
| 8 | M | **The Safety box takes free text.** A value that is not exactly `0/off/no/false` turns a switch on. For the five switches whose default is off, typing "Auto" or "auto " turns on: curve scales saved on the robots, edge parking, calibration give-up. Six grouped switches are also read straight from the environment in missions and calibration, so a saved box value is ignored there while the run's start line says it is set. | `safety_group.py:192-204`; `mission_control.html:206-218`; `run_plan.py:3061, 4066, 4085`; `calib_run.py:113, 1832, 4293`; `frame_watch.py:40` | Ran it: `MIRLT_CURVE_SCALE_SAVE="Auto"` reads as on; `MIRLT_EDGE_RULE="Fixed"` is reported as set but is not `fixed`. The e18 test checks the bypass in 3 of 7 files only (`test_e18_safety_blind.py:36-51`). | no |
| 9 | M | **Tag-free hands on a wrong heading to a robot whose own heading is right.** The "over 2 sigma" test is repeated on every picture. | `tagfree.py:413-424`, `502-521` | The fuser alone, own heading correct, detector noise 15 deg (its own starting number), 2 pictures a second: a heading handed on in 20 % of 15 s stands (median 11 deg), 30 % in a minute. The report's "never overrides a good gyro heading" rests on 29 pictures. It also disagrees with e18's own model of heading doubt (2 deg + 1.6 per quarter turn). | **yes**: the hand-on rule against real standing robots |
| 10 | M | **A wrong camera mid-run is never concluded** (hard simulator #4: confirmed). The camera watch votes only on sightings already named to a robot. Mirrored sightings fall outside every gate, so there are no votes. "Proven" is never taken back. | `tracking_layer.py:506`, `1001-1049`, `1080-1103` | Code reading. | no |
| 11 | M | **A restart does not re-establish the robot's pose or heading** (hard simulator #8: confirmed in the real runner). `on_boot` sets the safety layers and the follow run again; it sends no pose, asks no heading check, and does not reset the heading-doubt count. e18's "keep its own heading" then rates the boot heading as good evidence. | `freeroam.py:523-547`; `freeroam_core.py:1516-1519` | Code reading. What the real firmware's heading is after a boot was not checked. | no |
| 12 | M | **The odometry-trust fit runs inside the control loop.** Every 5 new samples: quantile fits for 3 robots plus the fleet split test. | `odo_trust.py:291-328`, `529-551` | Timed here: 52 ms median, 604 ms worst, on an idle x86 core. The loop's tick is 50 ms; Layout B gives the planner one Pi core. Not measured on a Pi. | no (the Pi bench) |
| 13 | M | **Blind error is understated in two places.** (a) An unplanned loss still uses 2 cm + 0.07 cm per cm, while the code's own e18 prior for the same quantity is 2.3 + 0.28. (b) The trust model is fitted on 10-40 cm stretches and used for 125 cm excursions. | `freeroam_core.py:1227-1231`; `odo_trust.py:63-64`, `351-354` | (a) Hard simulator, lost robots: error median 5.7 cm against a stated 3.4 cm. (b) Synthetic record whose error grows faster than a line: model 8.1 cm against a true 11.7 cm at 126 cm. The growth law in (b) is my assumption. | **yes**: a real blind excursion series at 40, 80, 120 cm |
| 14 | M | **Camera names fall back to a guess.** With the overhead unplugged, the camera whose name is the side camera's becomes the overhead. Two same-name cameras are assigned by device order. The four red `test_launcher.py` tests are the old refusals. | `tools/pi/camnames.py:233-261`; `observe.py:465-497` | Ran `pick_cameras`: "EMEET ... used as the overhead" beside "the side camera (EMEET) is not plugged in". | no |
| 15 | M | **Missions: a held robot creeps with no doubt in its gap to a neighbour** (16 cm middle to middle), and stops creeping without a word once its doubt is large. | `run_plan.py:3862-3938` | Code reading. | yes, with CREEP_MAX_CM |
| 16 | M | **A close pass still ends in a guess at home.** After three wiggles that show nothing, "names kept as predicted". The home lights cannot show a wiggle, and the out-and-back is not used on this path. | `tracking_layer.py:1209-1221`, `1304-1316` | Code reading. | no |
| 17 | L | Trust samples are duplicated after a gap (one error counted 2-3 times). `school_pull.sh` still excludes RFID files, against CLAUDE.md of 12 Nov, and its `.replaced` folder only grows. `run_plan.py` has no SIGTERM handler. A detector sighting is stamped with the feed's time, not its picture's. | `odo_trust.py:229-237`; `tools/pi/school_pull.sh:56`; `run_plan.py:6351-6359`; `tracking_layer.py:520` | First item run; the rest code reading. | no |

### Claims the code or data do not support

- **e18, "told apart in 24 / 25 of 30".** Reproduced from `trials.jsonl`, but these count both robots *named*: 1 (held) and 4 (never still) of them include a wrong name. Named right: 23 and 21. Under never still, 8 of 30 trials had a wrong name.
- **e18, "the safety code reads them only through it (a test checks)".** Finding 8.
- **e18, OB_K "4 sigmas, about 3e-5 by chance".** That is for a mean of 3 sightings; the code uses the 3 farthest of the whole watch. Finding 3.
- **e18, "983 passed".** Two full runs here: 982 passed, 10 skipped, 1 failed each time, a different test each time (`test_feel_table` once, `test_e14` SIGTERM once). Both look load-related (my reading); neither was re-run alone.
- **Tests that pass without testing behaviour** (`test_e18_safety_blind.py`): calibration creep (`:199-201`, checks that a function name exists), saving at every end (`:422-429`, checks the order of text in the file, and so fixes the order that delays `stop`), mission right of way (`:292-293`).
- **Tag-free, "never overrides a good gyro heading".** Finding 9.
- **Detector-v1, 96 % of standing robots found.** The 124 standing test robots are mostly two parked robots in one spot each, from the same run as the training pictures. Labels and "0 false" are relative to the camera track. The report says most of this; the tag-free simulator then uses 0.96 and 0 false as facts.

### The hard simulator's six findings, and whether the simulator is fair

| Hard-simulator item | Verdict | Root cause |
|---|---|---|
| Hung robots never freed | **Confirmed** | Finding 6. |
| Falls under harsh | **Confirmed** | Finding 4. Not the timing: wrong corrections plus reverse creeps. |
| A free robot standing for minutes (skew) | **Confirmed, cause corrected** | Finding 5 (the creep token), not the doubtful-neighbour wait. The missing-camera stand (133 s) was not traced. |
| Wrong camera name never noticed | **Confirmed** | Finding 10. |
| Doubt understated while unseen | **Mostly refuted** | 67 of 89 breaches in one run (all the not-lost ones) were sampled while the simulated controller was frozen in a replayed Pi silence: its stale belief against the moving truth. The rest is real (finding 13a). |
| Heading 40-70 deg off after a restart | **Confirmed in code** | Finding 11. |

Fairness of the simulator (`fastsim_realism.py`, `tools/analysis/realism_measure.py`):

- **Lost spells 15-20x: a timeline artefact (verified).** The replayed timeline is built from every robot's named fix, keyed by its own stamp (`realism_measure.py:224-228`). It holds 1.97 "pictures" a second against the bridge's 1.6, and 2,750 of 28,350 arrive before an earlier one, which a real queue cannot do. Fed to every robot, that timeline alone trips the "lost" rule 352 times a robot-hour.
- **Watchdog stops 2x (my reading, not tested).** Quiet spells over 1 s match the real run; those over 1.5 s are 1.6-2x high. That is a length shift of a few tenths of a second, and the watchdog is 1.5 s.
- **The hang model cannot free a robot that backed over** (`freeroam_fastsim.py:674-683`): only a reverse frees. A fix for finding 6b cannot pass in this simulator as it stands.
- **The doubt-breach measure** samples a frozen controller (`fastsim_realism.py:548-555`).
- **Checked and small:** naming with stale own poses (lost 136 to 105 a robot-hour, one seed); the curve ratio used as a gyro error (breaches 90 to 69 and 108 to 97, two seeds).

## Conclusion

1. **The most serious faults are not in e18's new rules but at their joins** (my conclusion): creep with tilt and a wrong pose (4), creep turns with a hung robot (5), tag-free noise with the out-and-back (3), two calibration savers (7).
2. **Three faults are plain bugs with small fixes:** `stop` after saving (1), dry runs writing real calibration (2), fail memory that never expires (6a).
3. **Corrections are trusted too easily.** A correction of 80-90 cm is applied at once, the robot is then "not lost", and the gate widens with each correction. This is the common root of the falls.
4. **The hard simulator finds real problems, but two of its headline numbers are its own artefacts** (lost spells, doubt breaches), and one of its readings is wrong (the 311 s stand).
5. e18 has not run on a Pi. Finding 12 and the wheel_scale saver (7) will act on the first real run.

## What's next

Suggested order (each is a small build with its own test):

1. `stop` first at every end, each later step in its own `try` (1).
2. Dry runs use their own conf folder, or skip the pool and workspace writes (2).
3. Creep token: a token ends when the creep fails or the robot is hung; move the three-pauses escape before the token check (5).
4. Corrections: no pose set from a sighting further than the robot's possible error without motion proof; no reverse creep for a tilted robot whose last correction is unconfirmed; corrections do not widen the gate until confirmed (4).
5. Hang recovery: fail memory expires or varies the try; add a forward-inward try with the tilt stop raised when the nose points inward (firmware too); fix the simulator's hang model first (6).
6. Out-and-back: refuse when the needed length is over the cap; test the sighting's motion against the drive's own timeline, not its farthest points (3).
7. One owner for `wheel_scale`; read the value in use at save time; save only a correction that differs from 1 by more than its error (7).
8. Safety box as drop-downs with validated values; the six direct reads moved into the group; the test widened to all files (8).
9. Before the first e18 run on a Pi: time the trust fit on the Pi bench, or move it off the loop (12).

## Sweep 1: new tests that may now be needed

- A free-roam end with a failing disk write: `stop` must still go out.
- A dry run must leave `~/.config/mirlt` unchanged.
- Out-and-back at 1.0, 1.5 and 2.0 cm of noise, with the command lost, and with detector-only sightings.
- Two unaccounted robots, one hung: the other must creep within three pauses.
- A robot hung nose-in (backed over), in a simulator whose hang model allows a forward escape.
- A correction over 30 cm onto a mirrored or another robot's spot: no reverse may follow until it is confirmed.
- A typed Safety value ("Auto", "Fixed", a trailing space) must be refused or read as auto.
- A robot standing 5 minutes with a correct heading under tag-free: no heading handed on.
- Two runs in a row: `wheel_scale` read back at the second start equals what the first saved.
- A restart mid-run (real robot, by hand): its heading after the boot, and whether the run checks it.
- The trust fit's time on a Pi 4 core with three robots driving.
- The hard simulator's timeline rebuilt per robot, then its lost-spell rate against the real 4.7 a robot-hour.

## Sweep 2: possible inaccuracies in what the program relies on

- **This audit.** Out-and-back and heading results come from the layer or fuser alone with Gaussian, independent noise; a standing robot's detector error is probably the same in every picture, which would make finding 9 worse. Finding 13b's growth law is assumed. Findings 10, 11, 15, 16 are code reading. Timing in 1 and 12 is from this machine, not a Pi. Hard-simulator traces are 1-2 seeds each. The missing-camera stand and the two flaky tests were not traced.
- **Chosen, not derived** (DOE in brackets): OB_MAX 4 cm and OB_FAR_SHARE 0.75 (yes); the identity batch's noise range 0.2-0.8 cm (measured home lights 0.3-0.5 cm, so fair for lights only); STEADY_S 1 s against a derived pause (yes, together); tag-free SIGNIF_K 2, HAND_MIN_N 2, Q_TURN 0.07 (yes; Q_TURN is a curve-execution number used as heading drift); fail memory 5 cm / 25 % (no); RECOVER_RETRY_S 20 s (no); CREEP_REACH 8 cm (yes); WD_MS 1.5 s against measured silences of 371 a robot-hour over 1 s (yes: the clearance has no term for a silent controller).
- **Proxies.** With no RAW rows, trust shadows use the fused own pose's steps, which already contain camera pulls. Shadows are dropped at every pose re-set, so the largest errors (those that end in a correction) leave no sample. Detector labels come from the tracker it is meant to check.
- **Described as measured, but typed:** the unplanned-loss bubble (29 Oct straight legs, used on curves); the hang model (one hang, 6 Nov); the simulator's restart and wrong-names models (said in its report).
- **Side effect of this audit:** my dry runs changed `~/.config/mirlt` on the audit machine (finding 2). The clone is unmodified.

## Appendix A: numbers behind the findings

- **Finding 1.** After the interrupt: saving done +0.32 s, `stop` +0.37 s; three `fix` commands went out in between. With the state write failing: no `stop`, `main` raised `OSError(28)`.
- **Finding 3, nobody moves** (60 s, 200 trials): 2 pictures/s: 0, 0, 0, 1, 81 named at 0.3 / 0.5 / 0.8 / 1.0 / 1.5 cm; 5 pictures/s: 0, 0, 0, 0, 32; 12 pictures/s: 0, 0, 0, 2, 45.
- **Finding 3, robot drives** (90 s, 200 trials; right / wrong / none): lights 0.4 cm 200 / 0 / 0; lights 0.8 cm 200 / 0 / 0; detector 1.6 cm at 2 pictures/s 94 / 33 / 73; at 4 pictures/s 76 / 18 / 106; lights 1.5 cm 106 / 16 / 78.
- **Finding 9** (handed on, own heading correct): 15 deg noise: 19.9 % in 15 s, 29.7 % in 60 s, 50.0 % in 300 s; 10 deg: 5.0 / 18.6 / 47.5 %; 18 deg: 33.9 / 41.5 / 56.5 %.
- **Never still against naming one lost robot** (e17 rig, 20 trials): corrected 20 of 20 either way, median 1.6 s; with never still the longest took 23.2 s and 1 of 20 ended over 5 cm off. No finding.
- **Suites.** Fleet: 982 passed, 10 skipped, 1 failed (twice, different tests). Pi + analysis: 149 passed, 4 failed (the known `test_launcher.py` four).

## Appendix B: the seven falls

| Run | Robot | Move at the fall | Run's position off the truth | Stated doubt | Last correction (how, how wrong) |
|---|---|---|---|---|---|
| harsh 2 | Center | reverse | 107 cm | 16.0 cm | named far, 91 cm |
| harsh 4 | Center | creep back | 45 cm | 3.1 cm | wiggle, 51 cm |
| harsh 5 | Right | creep back | 38 cm | 6.1 cm | wiggle, 35 cm |
| harsh 6 | Left | recover: reverse after pivot | 78 cm | 7.8 cm | named far, 87 cm |
| harsh 7 | Left | creep back | 82 cm | 3.3 cm | named far, 24 cm |
| wrong names 2 | Right | creep back | 137 cm | 4.6 cm | elimination, 0.5 cm (right when made) |
| wrong names 2 | Center | creep back | 50 cm | 7.5 cm | elimination, 87 cm |

All seven were hung when they fell; six were "not lost".

## Appendix C: check scripts (all under `/tmp/claude-0/-home-claude/fff6dc9c-5876-54f9-92a7-a08483d3d5c7/scratchpad/work/`)

- `stop_order.py` (1, 7), `ob_false.py`, `ob_real.py` (3), `hs_fall.py` (4), `hs_stand2.py` (5), `hs_hung.py` (6), `hf.py` (9), `trust_time.py` (12), `trust_chk.py` (13, 17), `ns_one.py` (never still against naming).
- Hard-simulator fairness: `hs_lost.py`, `hs_fair.py`, `hs_breach.py`, `hs_b3.py`, `hs_b5.py`.
