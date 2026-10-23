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
   seconds; a board touch: much longer).
7. **Detector check rate (DOE).** Every stop + every 1 / 2 / 4 / 8 s: wrong locks caught and how soon, vs laptop load.
   Starting value 2 s (chosen).
8. **Lock-on tracker vs off-the-shelf (OpenCV) vs the old floor-difference tracker** on the same footage: share of
   pictures followed, wrong-object locks, position error against hand-checked positions.

## Sensors
9. **Cliff sense.** Each robot creeps up to the open edges at several angles (head-on, 30, 60 deg, the open
   corner), recording line sensors and the front sensor's bottom zone (tools/edge_check.py): set edge_thr and
   edge_bot_cm from data, then both cues on.
10. **Front sensor against the map.** Offset and spread from a session's front_checks.csv (the analysis prints the
    suggested values); then whether it can steer (re-find on disagreement).
11. **Gyro drift and scale over a session, per robot,** from the naming checks and the self-calibration history.
