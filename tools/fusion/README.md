# Fusion: merging the sensors

Offline tools that replay recorded runs through the sensor fusion, with any
sensor switched off or degraded, and compare the results against the tape.

| File | What it does |
| --- | --- |
| `runs.py` | Run-file builder: puts each run's robot log, tape marks, camera sightings and blink lights on the laptop's clock; can save one folder per run |
| `ekf.py` | The Kalman filter for one robot: wheels, gyro, camera, blink light, heading fixes |
| `engine.py` | Runs the filters for all robots, applies sensor dropouts, the tracking rules and the identity rules |
| `graph.py` | Factor graph: re-solves a whole run so every reading fits together |
| `replay.py` | Runs every sensor setup over the same runs and compares them |
| `tests/` | Unit tests: light-offset model, identity combination, heading from motion (`python -m pytest tools/fusion/tests -q`, a few seconds) |

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
