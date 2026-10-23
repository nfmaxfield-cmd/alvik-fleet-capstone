# Saturday summary: outline (for Nick's approval before writing)

Audience: Professor Sodhi, read through his AI. Headlines hard, data in the appendices. Only what we accomplished and
aim to accomplish; Nick's own project (Andrew's lab separate: credit shared data or knowledge only). Two pages of body,
premise, data, conclusion, what's next. Then appendices.

## Page 1
**Title:** MIRLT: robots that calibrate, locate, map and track themselves, with whatever equipment is there

**Premise (3 lines).** Three Arduino Alvik robots on a 4 ft board under one cheap overhead webcam. The aim: fuse
every source of data (wheels, gyro, front and edge sensors, camera) so the robots work out where they are and how
sure they are, instead of being steered by the camera (the predecessor's approach).

**Headline result (with a chart).** Self-calibration: the robots learned their own corrections from their first runs,
and the calibrated runs came back **1.4 cm** from their start on average, against **16.4 cm** uncalibrated (44 runs,
2026-10-23; difference 15.1 cm, 95% interval 11.4-18.7 cm; p < 0.0001). Adding camera fixes mid-route did not improve
on that (2.3 cm; no significant difference).
- Chart: every run's end error, by condition (U, C, KC, K): dots plus median line, so the spread shows.

**Four headlines** (each 3-4 lines, numbers, one pointer to its appendix):
1. **Tracking that holds in poor conditions.** A motion detector finds a robot; a lock-on tracker follows whatever
   that robot looks like (nothing told beforehand); the robots' wheels and gyro vouch for it in turns. On the three
   stretches of footage where the old tracker lost robots four times each: seen 94 / 93 / 96% of the time (detector
   alone: 41 / 40 / 72%). Off-the-shelf trackers as a yardstick: one drifted 24 cm without noticing; one was too slow
   for the camera.
2. **The robots map the board by feel.** Front sensor at the boards, edge sensors at the open edges; the boards
   felt within about 2-4 cm of the camera's lines; cliff sensing tuned from real readings (line sensors: plywood
   52-84, edge 241-573, limit 150).
3. **They recover on their own.** Retries, another robot sweeping for a lost one, finding itself against the boards,
   driving out blind on its own pose, then a person last. 44 runs this morning, 0 help calls after the restart; in
   the afternoon two robots swept for a lost one and found it.
4. **Delay hurts the camera, not the robots.** Picture delay measured live: 0.12 s typical. When a WiFi copy made
   pictures over 1 s late in 26% of seconds, the camera lost robots on 28% of drives instead of 5%. Our robots drive
   on their own sensors and take camera checks standing still, which is exactly why this matters.

## Page 2
**Made hard on purpose: versatility.** The table from hard-on-purpose.md, cut to the 6 strongest rows (camera
quality, no survey, no typed-in calibration, no markers, identical robots, robots put down anywhere), and the closing
argument (minimal labour, infrastructure and know-how; better data of any type raises the system's confidence).

**What's next** (aim to accomplish):
- run a whole session on the Raspberry Pi alone (a Pi 4, about 5-7 pictures a second: stop-and-go driving and
  tracking only near each robot make that enough);
- the robots calibrate the camera themselves (no corner clicks);
- the latency test (inject delay: camera-steered vs our approach);
- docking stations (0.5 mm fixes, charging, self-calibration at every dock);
- continuous driving (time-stamped camera corrections while moving);
- self-labelled footage to train a detector (no hand labelling);
- the designs of experiment (check rate, confidence threshold, heading drive, sharp single frames).

## Appendices
- A. Accuracy: per condition (n, median, mean, spread, 95% intervals), the tests (Welch, Mann-Whitney), per route
  and robot; how each run is measured (the measured-frame pictures: start, far point, end).
- B. Tracking: the three footage stretches, the yardstick table, the live session's tracker shares and re-locks.
- C. Mapping and edges: the ray map picture; felt vs camera board lines; cliff readings.
- D. Sensors from the finds: own pose off at the board touches (6-8 cm median after a run), gyro scale between the
  boards (1.002-1.010), front sensor vs map (median -1.1 cm, spread 6.4 cm).
- E. Processing and delay: perf.csv summaries for both sessions; the session-2 comparison.
- F. Recovery and backups: counts per robot; the backups list.
- G. Designs of experiment and the test list.
- H. Where every number comes from (files, releases).

## To settle with Nick
- Run accuracy: session 1's live numbers are in; the footage re-measurement adds the final ones (the earlier
  question "how to handle thin data" no longer applies: 44 runs).
- The 100 cm far point (run 16, camera lost) is excluded as a measurement failure, and said so.

## Nick's answers (2026-10-23 13:21)
- Title: as above. Lead: self-calibration.
- Failures: in the summary only with a plan to experiment on and overcome them; otherwise a private note to Nick.
  So headline 4 keeps the WiFi result, with its plan (no copies during sessions; the laptop-place test; the latency
  test). The top-board trap and the 24 s stall go to Nick privately unless their fixes are tested by Saturday.
- Every chart states what it measures and where the data comes from (one minimal line).

## Nick, 14:21
- Fable: only for the final review of the assessment, likely Saturday overnight; submission Sunday (time Sunday
  morning to act on it, even a new test run).
- Robots compared (session 1): camera/wheels disagreements per drive Left 4.6%, Center 4.6%, Right 7.2%; auto-retries
  Left 3, Center 1, Right 4; calibrated end error Left 1.2, Center 1.4, Right 1.7 cm. Left is not the worst; it does
  the most drives (mapping, first in the find check: 237 vs 194 / 166).
- Find error explained: a steady ~4 cm offset toward both boards (same every find) plus 2-6 cm scatter.

## For the summary (Nick, 14:45)
- State plainly how little the robots know at the start: the floor is flat, their own size and sensor positions,
  a camera somewhere above. Not the board's size, edges, boards, the camera's place, or their own corrections.
- Explain how uncalibrated wheels and gyros can be trusted: the camera and the robots calibrate each other (gyro
  scale from a full spin: the picture repeats at 360 deg whatever the camera's calibration; wheel balance from
  straightness: a camera keeps straight lines straight; the camera's floor map from many rays; distance from the
  wheels' average, cross-checked by the robots' known size).
- Wheel slip (stress tests on a slippery floor) is a known limit with a plan (build notes), so it can be named.

## Notes from the explore build (2026-10-23 afternoon), for the summary
- Self-calibration now includes the camera: with no calibration file, the robots dance a square and drive rays;
  one fit gives the camera and every robot's wheel and gyro scale. Simulator check against the true camera: shape
  within 0.1 cm; turned and shifted only, 1.1 cm median over the board. The remaining error is size (+2%: the wheels
  set it) and where the corner lies (~4 cm). The real number comes from the session (`explore_check.py` against the
  corner clicks). A failure with a plan: absolute size (the dock's printed stripe-to-wall distance; the robots' size).
- The robots catch their own tracking errors during explore: a stop whose camera spot does not fit what the wheels
  did (a 30-90 cm ray the picture did not show) is found again by a wiggle turn or left out of the camera fit.
- Edge stops: the distance sensor's bottom zone sees the drop ~9 cm ahead (simulator); the line sensors 3.5 cm. The
  robots record which one stopped them. Real value: first measured in this session.
- The edge find: a robot locates itself against any two boundaries at right angles (boards or open edges), no
  camera; a board taken away without telling them is detected and the map corrected (remove-a-board test).

## Nick, 18:24
- Minimalist feature to state: the robot itself is the only thing needed to calibrate the camera (no target, no
  clicks, no tape). Hard-on-purpose row 10 updated.
- The Fable review must not just confirm our numbers: it should look through the data (logs, track.csv, perf.csv,
  find.csv, explore stops, edge stops, footage frames) for observations we missed: patterns by robot, time,
  battery, place on the board, delay, tracker state; anything surprising or unexplained.
