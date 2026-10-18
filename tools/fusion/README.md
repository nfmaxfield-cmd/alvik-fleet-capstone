# Fusion: merging the sensors

Offline tools that replay recorded runs through the sensor fusion, with any
sensor switched off or degraded, and compare the results against the tape.

| File | What it does |
| --- | --- |
| `runs.py` | Run-file builder: puts each run's robot log, tape marks and camera sightings on the laptop's clock; can save one folder per run |
| `ekf.py` | The Kalman filter for one robot: wheels, gyro, camera |
| `engine.py` | Runs the filters for all robots, applies sensor dropouts and the tracking rules |
| `graph.py` | Factor graph: re-solves a whole run so every reading fits together |
| `replay.py` | Runs every sensor setup over the same runs and compares them |

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
  as independent and aren't corrected for comparing ten setups.

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
scored against the camera with it. `runs.py --calib-per-run
floor_calibration_loro.csv` converts each listed run's camera sightings with the
calibration made without that run; `fit.used_runs` in a calibration file lists
the runs it used.

## Known limits

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
