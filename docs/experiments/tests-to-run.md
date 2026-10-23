# Tests to run when tracking is reliable

A running list (Nick, 2026-10-22: "keep a running list of tests we should run when we're ready"). Each: what it
shows, how, and what it needs first. Newest at the bottom of each section.

## Our approach vs camera-steered (the predecessor's)
1. **Latency hurts camera steering, not us.** Add a fixed delay to the camera pictures (0, 0.25, 0.5, 1, 2 s) and
   run the same routes in K (camera-steered) and C/KC (own sensors, camera checks at stops). Measure end error,
   far-point error, time per run, stops/recoveries. Expected: K degrades with delay; C/KC don't (the robot only takes
   a camera fix after standing still 300 ms, so a late picture still shows where it is). Can be run in the
   simulator first (delay injected in the feed); real runs need reliable tracking.
2. **End-to-end camera latency, measured.** A robot switches its lights on at a known robot time; the time the
   camera picture first shows them, on the laptop, is the camera path's delay (capture + WiFi + processing). Repeat
   ~50 times: mean, spread, worst. Also log per session: pictures skipped by the tracker, its time per picture.
3. **Camera blackout mid-route.** Cut the camera for a few seconds during runs: K stops or drives wrong, C/KC finish.
4. **Accuracy vs camera quality.** Lower frame rate and resolution in steps (a cheap warehouse camera): error by
   condition. Expected: K climbs fast, C/KC barely move.
5. **Load per robot.** Laptop processing and WiFi per moving robot for K vs C/KC: how many robots one camera/laptop
   can serve.

## Tracking and confidence
6. **Confidence threshold (DOE).** Which combined-confidence level should make a robot pause and confirm: false
   pauses vs wrong positions driven on, across levels; and how fast each witness confirms (a stop and a light blink:
   seconds; a board touch: much longer). Written up: DOE-tracking.md (DOE 2).
7. **Detector check rate (DOE).** Every stop + every 1 / 2 / 4 / 8 s: wrong locks caught and how soon, vs laptop load.
   Starting value 2 s (chosen). Written up: DOE-tracking.md (DOE 1, by replay: no robot time).
8. **Lock-on tracker vs off-the-shelf (OpenCV) vs the old floor-difference tracker** on the same footage: share of
   pictures followed, wrong-object locks, position error against hand-checked positions.
   First pass (2026-10-23, three evening stretches, distance from the runner's camera fixes): detector alone seen
   41 / 40 / 72%, 24 cm off Right twice; detector + lock-on 94 / 93 / 96%, all fixes within 6 cm, 31-40 ms a
   picture; OpenCV CSRT 17 ms but drifted 24 cm off Right without saying so; OpenCV MIL as close at the fixes, but
   67 ms a picture (too slow for 25 a second) and never says it lost the robot. Still to do: hand-checked truth
   every second, not only at the fixes.

## Sensors
9. **Cliff sense.** Each robot creeps up to the open edges at several angles (head-on, 30, 60 deg, the open
   corner), recording line sensors and the front sensor's bottom zone (tools/edge_check.py): set edge_thr and
   edge_bot_cm from data, then both cues on.
10. **Front sensor against the map.** Offset and spread from a session's front_checks.csv (the analysis prints the
    suggested values); then whether it can steer (re-find on disagreement).
11. **Gyro drift and scale over a session, per robot,** from the naming checks and the self-calibration history.

## Driving
- **Heading drive length (DOE).** A robot found by a turn on its spot learns which way it faces by a short drive the
  camera measures (10 cm; 15 cm where it has 30 cm of room from an open edge, chosen 2026-10-23). Factors: drive
  length (10 / 15 / 20 cm) x place (middle, beside a board, near an open edge). Responses: heading error (against
  the next leg's camera bearing), camera losses during the drive, distance toward an edge. About 30 min of robot
  time; repeats per cell ~8.
- **Detect after a move** (2026-10-23): how often the detector finds a robot where its wheels put it after the
  camera lost it while driving (`detect_after_move` in the state), against the old turn on its spot: time saved,
  wrong pick-ups (another robot or a ghost within 6 cm).
- **One sharp picture every check (Nick, 2026-10-23).** For the detector's periodic check (every 2 s), grab a single
  frame at the camera's highest resolution instead of the stream's 640x480, and match it with what the robots'
  sensors say. Test at each resolution the camera offers: position error at the checks, time to grab and process,
  and the effect on the stream. Later, the opposite end too: the laptop tracker at the lowest workable resolution,
  for the comparison (not now: perfect the tracking first).
