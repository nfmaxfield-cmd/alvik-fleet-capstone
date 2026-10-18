# Fusion: merging the sensors

Offline tools that replay recorded runs through the sensor fusion, with any
sensor switched off or degraded, and compare the results against the tape.

| File | What it does |
| --- | --- |
| `runs.py` | Run-file builder: puts each run's robot log, tape marks, camera sightings and blink lights on the laptop's clock; can save one folder per run |
| `ekf.py` | The Kalman filter for one robot: wheels, gyro, camera, blink light, heading fixes, landmark ranges |
| `engine.py` | Runs the filters for all robots, applies sensor dropouts, the tracking rules and the identity rules |
| `graph.py` | Factor graph: re-solves a whole run so every reading fits together |
| `landmarks.py` | Landmark ranging: map faces, distance-sensor ray geometry, which zones can be used |
| `replay.py` | Runs every sensor setup over the same runs and compares them |
| `tof_calib.py` | Distance-sensor calibration: each robot's zone directions and range error, and refined board positions, from runs the camera saw |
| `tests/` | Unit tests: light-offset model, identity combination, heading from motion, landmark ranging, wheel size, calibration, settings read from the session (`python -m pytest tools/fusion/tests -q`, about 1 minute) |

## Quick start

```
pip install numpy scipy
python tools/fusion/replay.py --session logs/<session> \
    --sheet docs/experiments/E01-baseline-dataset/data/measurements.csv --out results/E1-fusion
```

Without camera data, it compares wheels only, wheels + gyro, and wheel
distance + gyro heading. With camera sightings (`--detections`, `--calib`,
and `--recording` to line up the clocks by the light flashes) it adds the
camera setups: filter, factor graph, frames lost, camera blocked, low frame
rate, late frames, and camera only.

With the blink lights (`--blink <recording>/blink_detections.csv` from
tools/blink/blink.py, and `--calib-results <recording>/calibration_results.csv`
from calib.py for the start-nudge headings and the light's offset) it adds:
blink lights only, blink lights with heading from motion, detector + blink,
detector + blink with heading from motion, detector + blink in the factor
graph, and blink identity off (motion only). With detections it also adds:
camera with identity by motion, and detector positions only (no detector
heading, no blink) with heading from motion off / on, the case heading from
motion is meant for. The start-nudge headings are used only by these new
setups (`calib_heading`); the older setups ignore them, so their numbers
stay comparable whether or not the file is given. Simulator runs: `--truth truth.csv` adds path and
heading RMSE, and `camera_truth.csv` beside it (or `--camera-truth`) scores
which robot each camera fix went to ("right robot" column, `identity.csv`).

```
python tools/fusion/replay.py --session sim/session --sheet sim/measurements.csv \
    --detections sim/camera_detections.csv --calib sim/floor_calibration.json --recording sim/recording \
    --blink sim/recording/blink_detections.csv --calib-results sim/recording/calibration_results.csv \
    --truth sim/truth.csv --out results/sim
```

`runs.py` takes the same `--blink` and `--calib-results`: each run folder
then also gets `blink.csv` (light floor positions with the name
probabilities) and, in meta.json, `calib_headings` and `led_offset_cm`.

Your own setups: write a JSON list of settings (any of `engine.DEFAULT`) and
pass `--configs file.json`, e.g.

```json
[{"name": "gyro off 3-8 s", "sensors": ["wheel_v", "wheel_w", "gyro"], "sensor_off": {"gyro": [[3, 8]]}},
 {"name": "left half of floor blind", "sensors": ["wheel_v", "wheel_w", "gyro", "camera"], "cam_blind": [[-50, -50, 60, 100]]}]
```

## The tracking rules (engine.py)

1. **Possible area.** A camera sighting can only belong to a robot if it is
   inside that robot's uncertainty and no further than its top speed allows
   since the camera last saw it.
2. **Look-alikes stay separate.** Sightings no robot can claim are tracked as
   unknown objects and never handed to a robot.
3. **New things prove themselves.** An unknown object counts as real after 5
   sightings within 0.5 s.
4. **Robots don't vanish.** Unseen robots carry on by wheels and gyro, their
   possible area growing until they are seen again.
5. **Picked up.** When a robot reports being lifted, its position becomes
   unknown; once set down it takes over a confirmed unknown object that moves
   the way its wheels say it moves.

Late camera frames are applied at the moment the photo was taken: the filter
rewinds, applies the fix, and replays the wheel and gyro readings since.

## Blink lights, identity and heading (engine.py, ekf.py)

Switched on by the sensor `"blink"` and the settings `motion_identity`,
`heading_from_motion`; setups without them run exactly as before (checked:
every number in results.csv identical, 6-run and 36-run simulator sessions).

**The light as a measurement.** A blink row is the light (midpoint of the two
LEDs), not the robot centre. It sits a fixed offset ahead: (forward, left) =
the median of calib.py's `led_fwd_cm` / `led_left_cm` for that robot (all
robots' median if it has none; 2.75, 0 cm if there is no calibration_results.csv;
`led_offset_cm` overrides). So the filter expects it at
`position + R(heading) * offset`: one fix moves position and, a little,
heading. Light positions get the same lens term and the same 4 cm height
correction as the detector's robot-top point (`runs.load_blink`). Noise:
`Noise.blink_xy` = 0.5 cm (a guess: blink.py itself is ~0.04 cm in
simulation; the rest is calibration, height and offset error).

**Who is it.** Every sighting gets a probability for each robot whose run
is going, for "elsewhere" (a name not running now) and for "none" (not a
robot). Two clues, combined by naive Bayes: multiplied and renormalised,
which assumes they are independent given who it is, with a flat prior:

- *blink*: blink.py's per-track probabilities (`p_<name>`, `p_none`). A robot
  without its own column takes `p_other`. A detector sighting within 5 cm of
  a blink light in the same frame takes that light's probabilities, and its
  detector track keeps them between flashes with fading confidence (mixed
  with "no idea" by exp(-seconds since that light / 5 s): still >= 0.9 after
  a 0.6 s gap, gone about 1.5 s after the lights stop). The carried name is
  dropped when the track comes within 6 cm of another object.
- *motion*: each camera track (blink.py's tracks; detector sightings are
  chained frame to frame here) is compared every 0.5 s with each robot's own
  wheels + gyro: the track's displacement against the robot's dead-reckoned
  displacement, turned to the floor by its heading estimate (for a light, plus
  the light's swing as it turns), as a 2-D Gaussian likelihood (noise:
  camera noise, 5% of the distance, heading uncertainty x distance). "None"
  gets a broad likelihood (10 cm spread per window). Windows multiply, older
  ones count 0.7x per window; a window where nothing moved 1.5 cm says
  nothing; a track's record is dropped when another object comes within
  6 cm (tracks can swap there).

Rules:
1. Blink alone gives a robot >= `blink_name_p` (0.9): the sighting may only go
   to that robot, and only inside its possible area. Otherwise it is kept as
   an unknown object carrying the name, so rule 5 can't give it to another
   robot either. A confidently named sighting never goes to another robot.
2. Blink x motion gives a robot >= 0.9 (but neither alone did, or motion did
   alone): the position must agree too. Each robot's position likelihood is
   multiplied in (outside its possible area = impossible, "none" = 1 / view
   area); that robot must still have >= 0.9.
3. Otherwise the normal tracking rules, among robots that clues x position
   don't put under 5%.

Each fix used records where its identity came from: **blink** (blink alone
>= 0.9), **motion** (motion alone), **both** (each alone), **combined** (only
together), **position** (normal rules). replay.py prints the share per setup
and robot and writes `identity.csv` (for T7).

**Heading from motion** (`heading_from_motion`). When a robot drives
forward (both wheels > 3 cm/s, gyro turn < 10 deg/s for the whole second)
and its camera fixes moved > 3 cm/s over that second, the direction they
moved (light fixes turned back to the centre) is a heading fix at the end of
the window, corrected by the gyro's turn since the window's middle. Noise
sqrt(2) x position noise / distance moved + 1 deg, so it shrinks with speed.
At most one per second per robot; one more than 4 sigma off is dropped.
Off by default. In simulation it gained nothing when camera position fixes
exist (the filter already turns positions into heading as the robot drives);
it may help when only a detector without heading is available, which the
"detector positions only" pair of setups measures. First measurement (6-run simulator
session, detector at 30 frames/s): heading RMSE 0.66 deg off, 0.67 on, so no
gain there either; a slower or noisier detector is still untested.

**Start-nudge headings.** With `--calib-results`, each start nudge's
`heading_floor_deg` becomes a heading fix (0.8 deg) at its time, matched by
run id, in setups that opt in (`calib_heading`: on in the new built-in
setups, off by default and in the older ones).

## Tested so far (generated data, not yet real runs)

- Square route, 3 laps, wheels slipping 3% per turn, gyro bias 0.8 deg/s,
  camera noise 0.7 cm: end error wheels only 18.1 cm, wheels + gyro 0.8 cm,
  + camera 0.2 cm; still 0.2 cm with half the frames lost, the camera blocked
  for 5 s, or 200 ms late.
- A look-alike appearing 25 cm from the robot for 2 s: kept apart, robot
  estimate unaffected (0.1 cm error).
- Robot picked up and set down 30 cm away: re-found within 0.5 s of being
  set down (0.3 cm error after).

- Full simulated E1 session (`tools/sim`, 36 runs, 3 robots, seed 7), end
  error against the tape: wheels only 9.2 cm, wheels + gyro 3.8 cm, wheel
  distance + gyro heading 1.5 cm, + camera 0.15 cm (filter) and 0.13 cm
  (factor graph; along the whole path 0.23 vs 0.15 cm); 0.19 cm with half the
  frames lost or 200 ms late, 0.42 cm at 1 frame/s. The p-values treat runs
  as independent (p_holm, added later, corrects for comparing many setups).

- Blink lights, identity and heading (2026-10-18; simulator seed 7, full E1
  sheet, 36 runs, 3 robots one at a time; blink.py + calib.py on its video;
  `--calib-results` given; in this run every setup used the start-nudge
headings, which the older setups no longer do by default: numbers in
brackets are without them):

  | setup | end error cm | path RMSE cm | heading RMSE deg | right robot |
  | --- | --- | --- | --- | --- |
  | wheel distance + gyro heading | 1.29 (1.73 without nudge headings) | 0.82 | 1.14 (1.60) | - |
  | + camera (filter) | 0.12 | 0.22 | 0.77 (0.93) | 100% |
  | + blink lights only | 0.20 | 0.22 | 0.52 | 100% |
  | + detector + blink | 0.12 | 0.22 | 0.57 | 100% |
  | blink identity off (motion only) | 0.12 | 0.22 | 0.57 | 100% |
  | heading from motion on (any of the above) | same to 0.01 | same | same to 0.01 | 100% |

  The light's offset (2.73 cm ahead) makes it a heading measurement: blink
  lights cut heading error by about a third. The start-nudge headings help
  most where there is no camera. Heading from motion fired about once a
  second while driving but changed nothing: it is the same camera positions
  the filter already turns into heading as the robot moves. Identity sources
  (detector + blink, per robot): both 96%, blink alone 1.4-1.9%, position
  2.5-2.7%; with names off, motion 96%. In this session only one robot runs at a time, so "right
  robot" can't fail; see the next point.
- Identity with 2-3 robots at once (runs from that session overlaid in time
  and space, a scratch test, not in the repo): parallel squares 14 cm apart,
  parallel straights, two straights crossing 12 cm apart, three L routes:
  every setup put 100% of fixes on the right robot, including the position
  rules alone, so at 11 cm or more apart position is enough. With the squares
  on top of each other (physically impossible, 0 cm): position rules 99.47%
  right, blink names 100% of light fixes, detector + blink names 99.76%
  (detector sightings only get a name when a lit light is in the same frame).
  Before track memory was dropped near other objects (see the rules),
  identity by motion made that case worse (97.0%); now it never did.

- Landmark ranging (2026-10-18; tools/sim from branch follow-18b with a
  `--map` option, 36-run E1 sessions, seeds 7 and 11, five boards / a
  cinderblock 9-60 cm from the routes, plus an unmapped brick the fusion's map
  doesn't have; default settings). Seed 7 (seed 11 in brackets):

  | setup | end error cm | path RMSE cm | heading RMSE deg |
  | --- | --- | --- | --- |
  | wheels + gyro | 3.28 (3.83) | 2.06 (2.28) | 3.36 (3.77) |
  | + landmark ranging | 1.00 (1.17) | 0.84 (0.90) | 1.35 (1.46) |
  | + landmark ranging (factor graph) | 1.03 (1.36) | 0.63 (0.77) | 1.05 (1.17) |
  | wheel distance + gyro heading | 1.43 (1.35) | 1.05 (1.03) | 1.56 (1.42) |
  | ... + landmark ranging | 0.75 (0.85) | 0.63 (0.61) | 0.95 (0.95) |
  | + camera (filter) | 0.31 (0.29) | 0.37 (0.39) | 1.12 (1.09) |
  | + camera + landmark ranging | 0.39 (0.51) | 0.33 (0.35) | 1.15 (1.18) |
  | + camera, blocked 5 s mid-run | 0.25 (0.23) | 0.47 (0.51) | 1.20 (1.21) |
  | ... + landmark ranging | 0.30 (0.36) | 0.42 (0.43) | 1.22 (1.24) |
  | + camera, blocked 20 s mid-run | 1.01 (1.14) | 0.96 (1.08) | 1.51 (1.66) |
  | ... + landmark ranging | 0.91 (1.03) | 0.77 (0.84) | 1.36 (1.47) |

  Zone readings: 39-41% used, 1.5% rejected by the gate (3.7% of those tied
  to a face), the rest not tied to a face (12% none in range, 17% two faces
  close in angle, 15% over 20 deg, 13% reading out of range). Of about 47,000
  used readings, 1 disagreed with the true pose by more than 3 sigma: the
  unmapped brick and the room walls were kept out. Without the camera it
  roughly halves the path error; with the camera working it adds little to
  the path and makes the end error slightly worse (0.31 -> 0.39 cm), so the
  camera is still better than the model of the distance sensor. Every board
  moved 1 cm (tape error): + landmark ranging 1.23 cm end, 1.05 cm path
  (seed 7), still well under wheels + gyro.

## Landmark ranging (landmarks.py, engine.py, ekf.py, graph.py)

A board or block at a tape-measured spot gives the robot a position fix from
its own front distance sensor. Give the landmarks with `--map` (replay.py and
runs.py): a tools/fleet `map.json` or a `landmarks.csv`
(docs/data-formats.md, "Landmarks"). This adds the setups `+ landmark
ranging` (filter and factor graph), `wheel distance + gyro heading + landmark
ranging`, and with camera sightings `+ camera + landmark ranging` (filter and
factor graph), `+ camera, blocked 5 s mid-run + landmark ranging`, and a
20 s block with and without it. "Camera blocked the whole run + landmark
ranging" is `+ landmark ranging`. Your own setups: put `"tof"` in `sensors`.
Without `--map` nothing changes (checked: every output file byte-identical on
a 36-run simulator session).

**The measurement.** Each new sensor frame (RAW repeats a frame until the
next; `tof_new_frames_only`), for each of the five zones: the expected
reading is the distance from the sensor (`tof_forward_cm` 4.75 minus
`tof_recess_cm` 0.25 ahead of the centre) along the zone's direction
(`tof_zone_deg` +22, +11, 0, -11, -22, left positive) to the first mapped face
it hits. Noise 0.5 cm + 3% of the distance. A zone is used only if:

1. the reading is between 2 and 120 cm (`tof_min_cm`, `tof_max_cm`), and so is
   the prediction;
2. rays 5.5 deg either side hit the same face first (`tof_assoc_deg`; else two
   faces are close in angle: "ambiguous");
3. the ray lands at least 2 cm + 2 sigma (how unsure the filter is of where it
   lands) from the face's ends ("edge");
4. it meets the face within 20 deg of square-on (`tof_max_incidence_deg`,
   "grazing", see below);
5. the reading is within the gate of the prediction (`tof_gate_chi2` 10.83:
   1 dof, 99.9%) or it is rejected ("gated"): something unmapped is in front.

No identity is needed: the map says which face the ray hits from the
predicted pose. The factor graph re-uses the readings the filter used, tied
to the same faces. replay.py writes `landmark_ranging.csv` and `tof_*`
columns in results.csv: how many readings were used, gated, or not tied to a
face (and why), and the mean residual.

**Why 20 deg.** Where a zone's reading comes from inside its field matters
for faces hit at an angle: one ray down the middle (`tof_zone_width_deg` 0,
the default) or the nearest point within the zone's slice (> 0). In the
simulator (nearest of 3 rays across 7.33 deg) the single-ray model reads
0.2 sigma short on average, up to 0.5 sigma for the outer zones; with 60 deg
allowed that small bias, repeated 15 times a second, made landmark ranging
worse than none on the straight and L routes. Limiting to 20 deg keeps only
readings where the models agree. Once the zone footprint is measured on the
robot, set `tof_zone_width_deg` and raise the limit: in simulation, with the
right width and 60 deg, path error drops a further 0.84 -> 0.47 cm.


### Shared errors: consider states, joint update, adaptive weighting (options)

Many readings share the same error: a board 1 cm off its tape spot, a sensor
turned 2 deg on its mount, a range bias. Treated as independent, 100 readings
of that board count as 100 fixes and pull the pose the whole 1 cm. Options:

- `tof_consider` (Schmidt "consider" states): each face's offset, the
  sensor's range bias and its mount turn are carried as extra uncertainty that
  every reading of them shares (never estimated). The filter then knows the
  board can't place the robot better than the board itself is placed.
  Uncertainties: `tof_face_sigma_cm` 1.0, `tof_bias_sigma_cm` 0.3,
  `tof_yaw_sigma_deg` 1.0, or per face / robot from a calibration. The factor
  graph estimates the same three as unknowns with those priors.
- `tof_joint`: the zones of one frame in one update, their errors correlated
  by `tof_frame_corr` (the calibration measures it: about 0.1 in simulation).
- `tof_adapt`: a zone whose readings of a face keep disagreeing the same way
  gets that average disagreement added to its noise.
- `tof_every`: use every n-th frame. `tof_lag_inflate`: widen the noise for
  frame-to-frame correlation (the calibration's `lag1_corr`).
- `tof_heading_with_camera` false: while the camera saw the robot in the
  last second, landmark readings correct position only, not heading.

### Results (simulation, 2026-10-18)

Sessions r7 and r11 (seeds 7, 11; 36 runs each): realistic sensor errors on
purpose: each robot's zones turned 0.4-3.3 deg (mostly one way per robot),
range bias -0.3 to +0.4 cm and scale +-1%, every board 1 cm off in the map
given to the fusion, wheels 3% off; wheel size on in every setup. Calibration
by tof_calib.py, leave-one-run-out. End error / path RMSE cm / heading RMSE
deg, mean of both sessions:

| setup | camera working | camera blocked 20 s mid-run | no camera |
| --- | --- | --- | --- |
| camera (+ wheels + gyro) | 0.26 / 0.32 / 1.04 | 1.06 / 0.97 / 1.32 | - |
| + landmark ranging as before | 0.68 / 0.58 / 1.37 | 1.02 / 0.95 / 1.35 | 1.24 / 1.03 / 1.38 |
| + calibrated, consider states | 0.34 / 0.36 / 1.06 | 0.58 / 0.59 / 1.00 | 0.66 / 0.64 / 0.95 |
| + calibrated, consider, only when the camera missed the robot 1 s | 0.31 / 0.32 / 0.97 | 0.58 / 0.58 / 0.97 | (same as above) |
| camera, factor graph | 0.23 / 0.21 / 0.50 | | |
| + calibrated, consider, fallback (factor graph) | 0.34 / 0.30 / 0.54 | | |

Camera blocked 5 s: 0.21 / 0.43 / 1.10 without, 0.25 / 0.44 / 1.00 with the
fallback. Exact sensors and map (session w7e3, wheels 3% off): camera 0.27 / 0.31 /
1.02; with the calibrated fallback 0.28 / 0.30 / 0.95; camera blocked 20 s
1.05 / 0.93 / 1.23 -> 0.44 / 0.53 / 0.89; no camera, as before 1.04 / 0.79 /
1.15, calibrated with consider states 0.51 / 0.60 / 0.91. So: calibrated landmark ranging with consider states halves the
error whenever the camera is blocked or absent, and as a fallback it matches
the camera on path and improves heading, but with the camera working it still
costs 0.05 cm of end error on average (r7: 0.27 vs 0.27; r11: 0.35 vs 0.25),
and the factor graph is better without it. The target "never worse than the
camera alone" is NOT met for end error. Not shipped as default: the options
are off and the plain setups are byte-identical to before. Recommended until
real data says otherwise: `"tof_consider": true, "tof_camera_gap_s": 1.0` with
a calibration, in the filter (not the factor graph).

What was tried and dropped (r7/r11, camera working): joint update per frame
(no change: zone errors within a frame correlate only ~0.1), adaptive
down-weighting (no change), every 3rd frame (better than every frame without
consider states, still worse than the camera), position-only updates while
the camera tracks (r7: heading 1.05 -> 1.28 deg, worse), a 1 deg floor on the
mount-turn uncertainty (no change), steeper hits up to 60 deg (no clear
change).
Without calibration, consider states alone made the no-camera case worse
(1.6 -> 2.2 cm, r7, wheel size off): the 1 cm board uncertainty then limits what the boards
can teach.

**Calibration recovery** (r7, true errors known): zone directions within
1.6 deg (a common -0.75 deg shift), each board's position along the
direction the sensor sees it within 0.2-0.9 cm (worst: the board mostly seen
by one robot), robots' range bias / scale only partly. The readings then
agree with the map to the noise (rms 1.40 -> 0.93 cm; left over per run,
zone and face 0.2 cm). With the calibration runs' spins (branch follow-18b's
simulator, 9 runs, boards 1 cm off): zone directions within 0.8 deg (2.7 deg
without the spins) and boards within 0.06 cm.

### Calibration (tof_calib.py)

```
python tools/fusion/tof_calib.py --session <session> --sheet <measurements.csv> \
    --detections <detections.csv> --calib <floor_calibration.json> --recording <recording> \
    --map landmarks.csv [--calib-results <recording>/calibration_results.csv] --out <session>/tof_cal
python tools/fusion/replay.py ... --map landmarks.csv --tof-cal-per-run <session>/tof_cal/tof_calibration_loro.csv
```

The camera (with wheels and gyro, smoothed by the factor graph) gives the
robot's path; the map gives the boards' rough positions (tape, or clicked on
the video with calibrate_floor.py's Landmarks mode). The tool fits, by robust
least squares: each robot's five zone directions, range bias (relative to the
robots' average: a bias every sensor shares looks exactly like every board
being further, so it goes into the boards) and range scale; the zone width
and frame delay; each board's position and turn; the reading noise, and how
alike errors are within a frame and from frame to frame. Priors keep each
near nominal (zone 5 deg, bias 1 cm, scale 3%, board 2 cm / 3 deg). With
`--calib-results`, the calibration runs' 360 deg spins (firmware 2026-10-18b)
are used too: the sensor sweeps the boards at every angle, which pins the zone
directions down. It also fits once without each run (leave-one-run-out):
replay.py scores a run only with a calibration made without it, and refuses
otherwise. The spread of those fits (jackknife) is how sure each board and
sensor setting is; the consider states use it. Files: docs/data-formats.md.
`tof_use_cal` false makes a setup ignore the calibration.
The calibration runs' spin steps are found from the gyro (`spin_steps`: RAW
steps 4 and 6 from firmware 2026-10-18c's look-first run, 6 and 8 before;
until 2026-10-18 the tool used 6 and 8 for every run, which with 18c meant the
spin back and the turn to the roll heading).

The site setup (tools/site) writes the locked landmarks and each robot's sensor
model in this same format (`<site>/tof_calibration.json`, made from the survey's
own runs, so none of the scored runs is in it): `replay.py --tof-cal
<site>/tof_calibration.json --map <site>/landmarks.csv`. On r7/r11 (same setups
as above, current code) the always-merge setup's end error with the camera
working goes from 0.309 / 0.362 cm (1 cm map + tof_calib) to 0.221 / 0.288 cm
with the locked site map (camera alone 0.270 / 0.249): no penalty on average
(r7 gains 0.05 cm, r11 loses 0.04 cm), path RMSE better than the camera alone
in both; no camera 0.62 / 0.70 -> 0.45 / 0.52 cm. Details:
docs/experiments/E00d-site-setup.

Simulator results: see "Tested so far".

## Wheel size (engine.py `wheel_scale`, ekf.py, graph.py)

The wheels report distance with the nominal 1.7 cm radius; real wheels are a
little bigger or smaller and slip a little, so every robot's odometry reads a
few percent long or short. With `wheel_scale` on, the filter learns this for
each robot: one more number, the wheel scale k (wheels report k x the true
distance), starting at 1 (or at the calibration run's measured
`wheel_scale`, below) give or take 3%, allowed to drift slowly (0.05% per
second). Only a position sensor (camera, landmark ranging) can teach it;
with wheels and gyro alone nothing measures it and it stays exactly at its
start, so it cannot drift on its own (checked in the unit tests). A robot's
next run starts from what its previous run learned (`scale_carry`). The
factor graph treats k as one more unknown per run. `turn_scale` does the same
for the wheels' turn rate; it made wheels + gyro worse in every test (3.3 ->
4.0 cm, the turn scrub varies turn to turn) and stays off.

Simulation (36-run E1 sessions, seeds 7 and 11 for each level, robots' wheels
off by the amount shown, boards in the world; measured before the still-test
change below; end error vs tape / path RMSE, cm):

| setup | 0%: off | 0%: on | 3%: off | 3%: on | 5%: off | 5%: on |
| --- | --- | --- | --- | --- | --- | --- |
| wheels + gyro (no position sensor) | 3.66 / 2.15 | same | 3.86 / 2.48 | same | 3.93 / 2.88 | same |
| + camera | 0.26 / 0.35 | 0.28 / 0.35 | 0.59 / 0.72 | 0.26 / 0.35 | 1.19 / 1.14 | 0.26 / 0.30 |
| + camera, blocked 5 s | 0.20 / 0.45 | 0.23 / 0.46 | 0.50 / 0.92 | 0.23 / 0.48 | 1.24 / 1.51 | 0.24 / 0.46 |
| + camera, blocked 20 s | 1.03 / 0.93 | 1.06 / 0.94 | 1.60 / 1.55 | 1.26 / 1.09 | 2.75 / 2.30 | 1.65 / 1.29 |
| + landmark ranging | 1.09 / 0.86 | 1.07 / 0.85 | 1.22 / 1.14 | 1.08 / 0.87 | 1.37 / 1.45 | 1.12 / 0.83 |
| + camera + landmark ranging | 0.38 / 0.32 | 0.38 / 0.31 | 0.70 / 0.59 | 0.44 / 0.34 | 0.99 / 0.80 | 0.42 / 0.30 |

**Off by default**, because with exact wheels it costs a little (0.02-0.03
cm of end error with the camera): the learned k wanders between 1.000 and
1.012 depending on the route (squares pull it up), so it partly soaks up other
model errors, probably the heading in turns. Tighter priors (1-1.5%) or no
drift did not remove that. With wheels 3% or 5% off it more than halves the
camera's end and path error. Turn it on (`"wheel_scale": true` in a setup)
once T1 or the calibration runs show wheel-size errors of 1% or more.

**Calibration runs** (firmware 2026-10-18b: 15 cm roll, 360 deg spins): give
`--calib-results calibration_results.csv` and each robot's `wheel_scale`
(camera cm / odometry cm, rated ok) becomes its starting k = 1 / wheel_scale,
give or take its forward/back spread (at least 0.3%), and `gyro_scale`
multiplies its gyro (`calib_scales`, on; older files have neither column and
nothing changes). A measured wheel radius per robot (`--wheel-radius
Left=1.69,...`) also sets the starting k (1.7 / radius) and the wheel speeds
used by identity and heading from motion.

## Standing still (engine.py `still_mode`)

Zero-velocity updates (the filter is told speed and turn are exactly 0, which
also pins down the gyro's bias) need a reliable "still". The old test (wheel
speeds under 0.2 rpm, 0.1 cm/s, 0.5 deg/s) sat below the wheel-speed noise at
rest: in simulation it passed 13% of truly still rows. Now: still when
neither wheel's angle (RAW `wl_deg`, `wr_deg`) moved more than 0.75 deg (about
2 encoder counts) over the last 0.25 s, looking back only. It passes 80% of
truly still rows; 0.8% of moving rows also pass, almost all in the first
fraction of a millimetre as the wheels start. Simulation (seed 7 / seed 11,
end error cm): wheels + gyro 3.35 -> 3.03 / 4.19 -> 3.66, + camera 0.29 ->
0.29 / 0.55 -> 0.54, camera blocked 20 s 1.02 -> 0.88 / 1.64 -> 1.44,
landmark ranging 1.05 -> 0.96 / 1.15 -> 1.03, camera in the factor graph
0.27 -> 0.24 / 0.49 -> 0.47; one setup got worse (wheel distance + gyro
heading, seed 11: 1.98 -> 2.23). `still_mode: "speed"` gives the old test;
rows without wheel angles use it anyway.

## Settings from the session's own files

These used to be the simulator's numbers; now they come from the session:

- **Camera view area** (`clutter_cm2` "auto", used by the identity rules):
  the floor area of `view_floor_polygon_cm` in floor_calibration.json, or its
  image corners mapped to the floor. Was 19200 cm2 (the simulator's view).
- **Start-nudge heading noise** (`calib_heading_sigma_deg` "auto"):
  atan(0.15 px / how far the nudge moved on camera, `moved_px`), so a 2 cm
  roll seen as 7 px gets 1.2 deg, not 0.8. `--heading-check heading_check.csv`
  sets the 0.15 px from the spread measured on this setup;
  `start_nudge_heading_deg` in a `--noise-file` fixes it outright.
- **Frame rate**: from the recording's frames.csv, else its meta.json, else
  25 (was 30).
- **Video clock**: `<recording>/video_clock.json` (tools/check_session.py),
  with its drift when it says `use_drift`; `--video-offset` still wins.
- **Camera, camera-heading and blink-light noise** (were 0.8 cm, 8 deg,
  0.5 cm): `--noise-file measured_noise.json` (tools/accuracy_checks.py;
  docs/data-formats.md).
- **Wheel radius**: per robot, `--wheel-radius` (see Wheel size).

## Camera height

The floor calibration maps the floor plane to pixels, but the camera sees the
robot's top, about 4 cm up, which appears pushed away from the point below the
camera (about 1.7 cm at 50 cm out, with the camera 115 cm up). If the
calibration file gives `camera.height_cm` and `camera.nadir_floor_cm` (the
floor point straight below the lens), sightings are moved back to the floor
point below the robot's top. In simulation this took the camera end error
from 2.2 cm to 0.15 cm. Measure both when the camera is mounted.
A lens term `k1` (with `cx`, `cy`, `f_px`) in the file is applied too.

## Held-out calibrations (no circularity)

If the floor calibration was made from runs' tape marks or odometry
(`tools/calibrate_floor.py --from-runs` or `--survey`), those runs must not be
scored against the camera with it. `replay.py --calib-per-run
floor_calibration_loro.csv` (and `runs.py`, same option) converts each listed
run's camera sightings with the calibration made without that run;
`fit.used_runs` in a calibration file lists the runs it used. Both tools
refuse to score a run with a calibration whose `fit.used_runs` includes it
(a run missing from the per-run file falls back to `--calib`, and is refused
if `--calib` used it); `--allow-in-sample` overrides. With `--calib` alone they
warn when its `used_runs` overlap the runs. results.csv records, per run and
camera setup, `calib_file` and `calib_includes_run`.

```
python tools/fusion/replay.py --session ... --sheet ... --detections ... --recording ... \
    --calib <rec>/floor_calibration.json --calib-per-run <rec>/floor_calibration_loro.csv --out results/E1
```

**Survey calibrations and the wheels.** A survey fit hangs the camera on the
robots' own wheel + gyro paths, so a robot's wheel-size error goes into the
calibration. Holding out one run doesn't remove it: the robot's other runs
carry the same error, and the camera then agrees with that robot's wheels
for the wrong reason, which flatters camera-vs-wheels comparisons. replay.py
prints a note when a run is scored with a survey calibration that used the
same robot. **Headline T2/T3 results should use a click / tape-point
calibration, or a leave-one-robot-out survey** (`calibrate_floor.py --survey
--holdout robot`, which writes `floor_calibration_loro_robot.csv`: pass that
as `--calib-per-run`; needs two or more robots).

## Statistics in replay.py

Each setup is compared with wheels only on the same runs (paired t-test). With
about 19 setups, some p-values under 0.05 turn up by chance, so the table also
has `p_holm`: p adjusted for comparing many setups at once (Holm method:
the smallest p times the number of comparisons, the next times one fewer, and
so on, never decreasing). Routes are pooled in the main table; summary.csv
also has every setup per route (`route` column: runs, mean end error, 95%
interval), since a setup can help on squares and not on straights.

## Known limits

- Blink identity comes from blink.py run over the whole video: a track's
  name probabilities use flashes that come later. Live, a robot would get its
  name only after about 6 flashes.
- Naive Bayes treats blink and motion as independent, and motion windows as
  independent of each other (with forgetting and a +-30 log cap); both are
  from the same camera track, so the combined probability is overconfident.
- The light's noise (0.5 cm), the view area for "not a robot" (160 x 120 cm),
  the motion windows and the heading-from-motion thresholds are chosen, not
  fitted. Wheel speeds for heading from motion use the nominal 1.7 cm radius.
- A robot re-found from a light alone (rule 5, heading unknown) is placed
  at the light with offset-sized uncertainty (3.75 cm).
- Rule 5 with no tape start gives a confirmed unknown object to the first
  lost robot whose speed fits; a blink name stops a wrong pick only once the
  unknown object carries a name. Not yet tested with robots started out of
  order.
- The factor graph takes light and heading fixes (`+ detector + blink
  (factor graph)`), checked on 6 runs only.

- Wheels + gyro together came out worse than gyro heading alone in
  simulation (3.8 vs 1.5 cm): the filter trusts the wheels' turn rate more
  than it should during turns. Fitting the noise settings from E1 should fix
  this; check it is gone after the fit.

- The noise settings in `ekf.Noise` are starting guesses. Fit them from E1:
  gyro noise and bias from the still periods, wheel errors from tape vs
  odometry, camera noise from E4.
- Heading conventions: the robot's turn rate (`w_dps`) is taken as
  counter-clockwise positive; the gyro's sign and units are checked against
  it automatically for each robot. Check the first real L-path turns right in
  the estimate.
- A robot with no tape start pose is found from the camera, by the same rule
  as a picked-up robot; with several still robots in view that can pick the
  wrong one. E1 always has a tape start.
- The factor graph here smooths a whole run after the fact. A live
  sliding-window version for the Pi comes with E6.
- Landmark ranging: tested in simulation only, where the sensor model is
  itself a guess (3 rays per zone, 15 frames/s, 0.5 cm + 1% noise). Without
  options the five zones and successive frames are treated as independent;
  `tof_consider` carries the shared errors, but only per face, per robot range
  bias and per robot mount turn; errors particular to one zone and one face
  are covered only by widening (`systematic_cm`). Circles in map.json are not
  landmarks. The association check (+-5.5 deg) may be stricter than needed.
- tof_calib.py: a robot's range bias and scale trade off against the board
  positions (only partly separable without the calibration spins); a bias all
  robots share goes into the boards. Its reference is the camera path, so any
  camera error (floor calibration, video clock) goes into the calibration.
  Graph setups with `tof_consider` often hit the solver's step limit.
- Wheel size: one scale per robot for both wheels; a left/right difference
  (which curves the path) is not a state. Off by default (see Wheel size).
