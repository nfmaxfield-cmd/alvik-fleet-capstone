# Session notes, Oct 21: first real footage through Maxfield IRL Tracking (roam1)

## Premise
Until now every number in the recognition studio was tuned on synthetic recordings. This session
ran the studio's tracking, check-in measurement, robot-log alignment and Line up on the real
16-minute roam1 recording (overhead camera, 23,995 frames, about 25 frames/s) against Nick's
labels: 95 reviewed frames, 280 robot labels, 15 hand and 54 unknown. All work was done on a copy of
his project under `/tmp/claude-0/roam1/work/` (the originals are unchanged). The side camera was not
included.

## Floor calibration (made here, needed by items 4 and 5)
- **Board corners**, found by fitting straight lines to the board's 4 edges on a median of 8
  frames (edge fit error 0.3 to 1.4 px): bottom-right of the picture (387.7, 337.6), top-right
  (430.4, 62.0), top-left (141.1, 6.9), bottom-left (111.9, 298.9).
- **Rectangle**: x edges 122.15 cm (mean of 122.2 and 122.1), y edges 121.9 cm, lens height 262 cm.
  Nick's frame: origin at the bottom-right of the picture, x up the picture, y to the left.
- **Result**: fit error 0.9 cm (largest 1.3 cm); camera 8.8 deg off straight down; scale 2.19 to
  2.52 px/cm across the board (mean 2.34).
- **Cross-check against the robots**: a robot's short side measures 23.2 px (median of 95
  roaming robots), so about 9.9 cm against 9.5 cm real (the long side, 26.8 px, includes motion
  blur and shadow). The board size agrees within about 5%, so the **board corners were trusted**;
  the robot size only confirms them.
- **Not matching the Oct 18 setup**: `real_oct18.json` says about 3.4 px/cm. Today it is 2.34, so
  the camera is higher or the view is wider than on Oct 18. The 262 cm height is from Oct 18. It only
  affects where robot tops are placed (4 cm up: under 2% either way).

## Data and findings

### 1. Speed on 16 minutes of real footage
| Step | Real roam1 | Synthetic bench |
|---|---|---|
| Measuring object size (first time only) | 45 s, and it **failed** (fell back to 30 px; the real size is about 23 px) | (not measured) |
| Motion finder (first time only, then cached) | 245 s (this 2-core machine, with another job running) | (part of the 32 s) |
| Following with look (every opening) | 23.3 s | 32 s first pass |
| Holding the studio after a save | 0.22 to 0.28 s | 0.16 s |
| The rest of the re-track (in the background) | 3.7 to 5.8 s | (not measured) |

Following is faster than on the synthetic bench; a save holds the studio about 0.1 s longer. The
first opening of a new recording costs about 5 minutes, mostly the motion finder.

### 2. Tracker accuracy against Nick's labels
"Found": a guess within 12 px (about 5 cm, half a robot) of his label. Situations: near hand (a
hand label within 3 robot sizes), close pair (another robot within 2), still (picture unchanged over
±0.5 s), otherwise moving.

**No labels at all (motion and following only):**

| Situation | Labels | Found | Missed | Median / p90 error |
|---|---|---|---|---|
| Moving | 73 | 93% | 5 | 0.0 / 4.7 px |
| Still | 190 | 19% | 154 | 1.5 / 3.8 px |
| Near hand | 15 | 7% | 14 | 4.8 px |
| Close pair | 2 | 2 of 2 | 0 | 1.3 px |
| **All robots** | **280** | **38%** | **173** | 1.1 / 4.0 px |

Extra guesses: 2.3 per frame. Motion alone never sees a robot that has not moved yet: Center
sat still all session, and all three robots were parked for the first 8 minutes.

**With labels**: the labels of one half of the frames are given to the tracker, and its guesses
are scored on the other half's frames. The halves are alternating 60 s blocks; A has 52 frames,
B has 43.

| | Found (all) | Still | Moving | Near hand | Extras per frame | Score* |
|---|---|---|---|---|---|---|
| A, current settings | 76.5% | 67% | 100% | 3 of 3 | 1.44 | 0.614 |
| A, tuned on A | 76.5% | 67% | 100% | 3 of 3 | 0.79 | 0.674 |
| B, current settings (held out) | 86.6% | 90% | 97% | 33% | 0.60 | 0.786 |
| B, the A-tuned settings | 87.4% | 90% | 97% | 42% | 0.65 | 0.787 |

<sub>*Score = robot labels found / (robot labels + half the extra guesses), so it rewards finding
robots and penalises extra guesses. The A-tuned settings: still_ncc 0.80, max_absence_s 10,
confirm_n 8. The coordinate search covered still_ncc, max_absence_s, ref_break, step_ok,
reacquire_ok, chain_min_travel, confirm_n, max_speed and robot_px (24 vs 30), with 2 rounds and
36 runs.</sub>

- **No default changed.** The settings tuned on A did not improve the held-out half (0.786 to
  0.787, with more extra guesses). still_ncc 0.80 alone gained one label in B. Still robots'
  patches match at 0.95 to 0.97 on real footage, so that gain has no physical cause.
- 24 of the 36 still robots missed in A come from frames 0 to 1487, before the first label in B.
  Tracking runs forward only, so a label never reaches back to earlier frames.
- Most extras are old tracks left where a robot used to be, scored 0.02 to 0.35.

### 3. How often to check (measure_checkins on his labels)
- As built, the studio would have told Nick "check every **0 s**". Hands labelled 1 s apart had
  always moved on, and they were counted the same as robots. 8 of the 11 failures at gaps under
  2 s were hands or unknown things, and the under-2 s bin scored 92.4% against the 95% needed.
- **Changed:** hands and unknown things (the loose types) are no longer measured.
- **Robots only** (218 pairs): within a quarter of a robot of his label in 97.7% of pairs at gaps
  under 2 s, 100% at 2 to 10 s, 94.7% at 10 to 30 s, and 78% at 30 to 120 s (9 pairs).
- **The suggested check-in is now every 10 s** by the bins rule. The finer rule (95% of all pairs up
  to that gap) gives 41.7 s, which is the longest gap in his labels.

### 4. Robot logs (Left and Right, 11 routes each)
- **Clock**: the log runs 0.14 to 0.18 s ahead of the video (best speed matches: Right -0.18 s,
  corr 0.73; Left -0.16 s, corr 0.65; Line up found -0.15 s). The live session's own clock file
  gives an expected value independently: laptop - Pi = 0.215 s, and a frame is taken 0.04 s before
  the Pi stamps it, so -0.175 s. These agree.
- **Heading reset per run**: the gyro yaw never resets (largest jump across all 20 run boundaries
  is 0.1 deg). The library pose reset in 12 runs. In the other 10 it carried on from the previous
  run's end (heading about 400 deg). The reader handles both cases.
- **Gyro scale**: measured directly as camera turn ÷ gyro turn between straight stretches
  driven forward: **1.069** for Left (16 turns, IQR 1.044 to 1.090) and **1.075** for Right (21 turns,
  IQR 1.065 to 1.096). This confirms about 1.08.
- **The aligner's calibration fit does not reproduce that**: it gives gyro 1.29, drift
  1.5 to 2.3 deg/s and wheels 0.86, with an 8 cm misfit.

Draft labels, each label left out in turn and predicted from the rest (92 of his robot labels
could be named Left or Right; the rest are Center, or from outside the logged runs):

| | Position error median / p90 / max | Heading error vs video, median / p90 |
|---|---|---|
| With the fit (as the studio does it) | Left 2.4 / 5.7 / 12.9 cm; Right 2.4 / 6.3 / 23.8 cm | Left 27 / 46 deg; Right 13 / 29 deg |
| Without the fit (gyro 1.08, wheels 0.979) | Left 1.1 / 4.6 / 14.1 cm; Right 1.2 / 2.8 / 13.0 cm | Left 11 / 29 deg; Right 10 / 38 deg |

<sub>Heading errors are measured against the video's direction of travel on forward, straight
stretches. Nick's labels carry a heading on only 2 rows.</sub>

- **Names land on the right robots.** The speed match named track 71 Left and track 73 Right. The
  blink codes seen on video along those tracks agree in every clear 10 s window (19 of 19 at Left's
  0.3 s, 16 of 16 at Right's 0.5 s).

### 5. "Line up again" moved names to new object numbers: reproduced and fixed
- **Before**, three Line ups in a row:
  - names went 57/436, then 67/76, then 57/436 again;
  - the labels used to line up dropped from 2 to 0 to 2, out of 280.
- **Cause**: each name went to one object only, the lowest speed-match cost, and a short stretch
  can win that by a hair. Nick's labels give each robot many numbers over 16 minutes, so the labels
  on every other number were dropped.
- **Fix** (`outsidelink.name_objects`):
  - every object whose speed clearly matches one stream gets its name;
  - two objects with one name never overlap in time;
  - each name's main object is the one matching longest.
- **After**: 71 Left and 73 Right on every Line up, using 90 to 92 labels.
- Test: `test_line_up_names_every_object_of_a_stream_and_keeps_the_longest_as_its_main_one`.

### 6. Why the live tracker only ever named Right
Two causes, both confirmed in the recorded video (it was recording during the live attempt).
1. **Center's idle blink looks like Right's code.**
   - During the 07:02 runs, the three parked robots blinked exactly as designed: Left every
     0.300 s, Center 0.400 s, Right 0.500 s.
   - Outside runs only Center blinked, every **0.502 s**, which is Right's code.
   - Center's battery was at 15%. On Oct 19 a robot showed two blinking red lights at low battery,
     so this is likely the low-battery warning.
   - The "stray light named Right" (fixes at 07:05:04 and 07:05:20) was Center.
2. **Left and Center were named, but never sent a position.**
   - The live log shows Left named 9 times and Center once (Right 26 times).
   - A position also needs the two lights paired and front/back settled.
   - Tracks kept breaking: track numbers reached 210 in 4 minutes, with 155 merges, 17 flips, and 110 of
     162 sightings "code unclear".
   - Only Right, next to the board's edge, got far enough.

**What would fix it**:
- Make the low-battery or idle blink something no name code can be: a steady light, longer than
  420 ms, or red only.
- Or let the live tracker accept a name only while that robot's log says it is running.
- Neither needs a code change today.

## Conclusion
- On real footage the tracker finds moving robots well (93 to 100%).
- It finds still robots only from labels, and only forward in time from them.
- Extra guesses (0.6 to 1.4 per frame with labels) are the main cost to the person.
- No tracker default had a held-out gain worth changing.
- Two real-data bugs were fixed: the check-in answer of 0 s, and names jumping at Line up.
- The robot logs line up to about 1 cm when the aligner keeps the known gyro and wheel scales. Its
  own fit makes this worse.

## What's next
1. Fix or bypass the aligner's calibration fit (gyro 1.29 against 1.07 measured): keep the priors
   unless the fit lowers the leave-one-out error.
2. Fix measuring object size on real footage (it failed, so 30 px was used against about 23 px).
   Then re-tune with the motion finder at the right size: the 24 px detections are already made
   (280 s) but have not been scored.
3. Let a label also reach back in time for still objects (24 of the 36 still misses in half A).
4. Firmware or live tracker: an idle or low-battery blink that is not a name code.
5. Nick: redo the floor calibration in the studio with the corners above, on his copy.

## New tests that may now be needed
- Object size measurement on a real recording with parked robots, where it currently fails.
- The aligner's fit on real logs: it should never be worse in leave-one-out than not fitting.
- Line up on a recording whose labels renumber objects, end to end through the studio (the unit
  test covers naming; the real-data repro is a script, not in the repo).
- The live tracker with one robot showing an idle blink at another robot's rate.
- Check-ins split by type, if a project wants hands measured too.

## Possible inaccuracies in what this relied on
- **His "confirmed" labels are copies of the guesses of his session.** Errors measured on them
  read 0.0 px. This flatters position errors and check-in results. Only "corrected" labels are
  independent.
- **Thresholds chosen here, not derived**:
  - the found radius of 12 px;
  - the situation rules: still = mean picture change under 15 grey levels over ±0.5 s; close = 2
    robot sizes; near hand = 3;
  - the tuning score (half weight on extras);
  - the 60 s halves;
  - the naming limits: 3 cm/s, 0.6 ratio, 10 cm travel, 20% overlap.
- **Calibration assumptions**: the 262 cm lens height and the 122.15 cm mean x edge. The corners
  come from brightness edges, and the board's dark side face could shift the right and bottom edges
  by about 1 px.
- **Speed figures** come from a 2-core Linux container with jobs running alongside. Nick's laptop
  will differ.
- **Clock offset**: the expected -0.175 s assumes the live session's offset and capture delay
  still held during roam1 (same morning, same Pi).
- **Center's idle blink** being the low-battery warning is inferred from the battery level and Oct
  19, not checked against the firmware.
- **Left and Right names on labels** come from speed matching. Only tracks 71 and 73 were checked
  against blink codes.
