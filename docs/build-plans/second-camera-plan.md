# Second camera: plan for tracking and self-calibration (2026-10-22)

**Premise.** The overhead camera loses robots in the blocked corner (where the x-board and y-board meet) and
along the board edges. From straight above, a robot's outline merges with the board and its shadow
(`docs/build-reports/e02-oct22-build.md`: a robot against a board was seen in only 30% of pictures). The side
camera (EMEET C950) is already recorded by the Pi but the runner ignores it (`--no-side`). This plan adds it to
tracking and to self-calibration.

## What exists already (read, not assumed)
- **Pi:** `tools/pi/camrec.py` records several cameras on one shared clock; camera i streams on port 8000+i.
  `e02.py` already has `--side-port` (8001) and only checks that the stream answers.
- **Floor calibration:** `e02_camera.find_board_corners(img)` → `e02_camera.make_calibration(corners, size, path)`
  → `floorcal.fit(spec, size)` (rectangle mode) → `cameras.CameraModel.from_points`. Loaded with
  `e02_camera.load_camera(path)`; checked by eye with `e02_camera.draw_check`. `make_calibration` hard-codes
  the name "overhead", and `find_board_corners` orders corners by picture position, which suits a camera
  looking straight down only.
- **Camera to camera:** `cameras.transfer(cam_a, cam_b, u, v, height_cm)`, `crosscam.match_frames`,
  `crosscam.cross_guesses`, and `recognition/referee.look` (the studio's second-camera referee).
- **Tracking:** `e02_camera.MotionTracker` (with `expect()` and `set_area()`), fed by `e02_camera.Feed`, one
  per stream. Robot size in pixels (`rpx`, used by `_shape_ok`) is computed once, at the board's middle.
- **Fusion:** `fusion/ekf.EKF.cam_xy(xy)` and `cam_d2(xy, t)` use one noise for every camera
  (`Noise.cam_xy`), fed by `e02_live.LiveLab.on_camera(robot, t, x, y)`. It only listens; the runner steers
  from `MotionTracker.where()`.
- **Self-calibration:** `e02_fit.camera_stretches`, `pair`, `fit_robot` (gyro scale, drift, wheel scale).
- **Simulator:** `e02_sim.SimCamera` renders one near-straight-down camera (`sim/camera.CameraGeometry`), one
  `/live` stream; `SimRecorder` writes `cam0` only; robots are flat parts at their heights, no side faces.
- **Real two-camera footage:** roam1 (2026-10-20, 961 s, side camera at 29 fps) can be replayed offline.

## The plan

**1. Mounting.** Raised at the open corner diagonally opposite the blocked corner (floor (0, 0) in the sim's
default layout), about 1.2-1.5 m up, looking down about 35-45° at the blocked corner. From there the two boards'
faces are the background and the line where floor meets board is a sharp line a robot breaks; a robot against
a board stands in front of it, never behind it. The cost: robots can hide each other, and the far corner has
the fewest pixels per cm. Looking along one board from its open end would help one edge only.

**2. Calibration to the floor.** Reuse `floorcal.fit` in rectangle mode with the board's 4 corners. Changes:
`make_calibration` takes a camera name (writes `cameras/side.json`); corners for the side camera are clicked
once in the studio (More → Floor calibration) or ordered by `find_board_corners` and then matched to the
overhead's corners by `cameras.transfer`. Then a cross-check: one robot standing at 5 spots, both cameras'
positions compared; the side camera's leftover offset (it sees the robot's side, so its outline centre leans
toward the camera) is fitted as a small correction by distance from the camera.

**3. Tracking with two cameras.** A second `Feed` and `MotionTracker` on the side stream. The tracker needs
the robot's size per position (`rpx` from `cam.px_per_cm` at the robot's spot, not once at the middle), since
the side view's scale changes about 2-3 times from near to far. Both feeds share the Pi clock, so their times
line up already.

**4. Which camera to trust where.** A per-camera noise (cm) that depends on the spot:
- from geometry: a base noise divided by pixels per cm there;
- overhead: raised within ~12 cm of a board or in the blocked corner;
- side: raised near its far edge, and set to "don't use" when another robot's known position lies between the
  side camera and the target (hidden);
- both: refined from the measured disagreement between the two cameras, region by region, once data exists.

A combiner gates each camera's fix against the filter's prediction (`cam_d2`) and passes both on when they
agree. When one camera has lost the robot, the other's position goes to the lost tracker as
`expect(x, y, radius)`, so it finds it without growing its search circle. On disagreement the lower-noise camera
wins and it is logged; a jump (the Left-for-Right mix-up) needs both cameras.

**5. Fusion.** `EKF.cam_xy(xy, sigma=None)` and `cam_d2(xy, t, sigma=None)`: same filter, per-fix noise from
the trust map. `LiveLab.on_camera` gains a camera name so the live view shows trust per camera. The runner keeps
steering from the combined tracker position; moving steering onto the filter is a separate decision.

**6. What it gives self-calibration.**
- More straight stretches: today's stretches stop where the overhead loses the robot (edges, corner).
- A measured camera noise: two cameras timing the same stretch give a real value for `SIGMA_CM` and `SIGMA_DEG`
  in `e02_fit` (now chosen).
- A camera-moved check: the cameras disagreeing in the board's middle means one was bumped (fits the
  2026-10-19 decision that cameras may move and be re-found).
- Board faces: `measure_board_line` on the side view checks the overhead's board lines (1-2 cm short).

## Steps, in order

| # | Step | Hours |
|---|------|-------|
| 1 | Mount the side camera; confirm camrec's exposure lock for it (roam1 went dark) | 1 (Nick) |
| 2 | Calibrate on roam1 footage first: name in `make_calibration`, corner matching, `draw_check` | 2 |
| 3 | Per-position robot size in `MotionTracker`; side tracker replayed on roam1 | 5 |
| 4 | Side-view lean correction from the 5-spot cross-check | 2 |
| 5 | Combiner: trust map, gating, `expect()` hand-over, hidden-robot rule | 4 |
| 6 | EKF per-fix noise; `LiveLab` per camera | 1.5 |
| 7 | Second stream in `e02.py` (replaces `--no-side`); check WiFi frame rate with both | 2 |
| 8 | Simulator second camera (below) | 5 |
| 9 | Self-calibration: stretches from the combined track; camera-to-camera noise | 2.5 |
| 10 | Real session dry run, footage review | 2 (Nick) + 2 |

Total about 29 hours, 3 of them Nick's at the board.

**Simulator needs:** a second, steeply tilted `CameraGeometry`; robot sides and standing boards drawn in 3D;
a second `/live` port; `SimRecorder` writing `cam1`; `E02_SIM_HIDE` per camera, so the overhead alone can lose
a robot.

## New tests that may be needed
- Calibration: side camera from roam1 corners, cross-checked against the overhead at 5 spots (max error limit).
- Tracker: a robot against each board and in the blocked corner, overhead hidden, side sees it; position
  within a set error.
- Hand-over: overhead loses the robot, the side's `expect()` brings it back without taking another robot.
- Hidden robot: two robots in line from the side camera; the side fix is not used.
- Disagreement: one calibration shifted 3 cm; the camera-moved check fires.
- Frame rate: both streams over WiFi at once, at least 15 pictures a second each.

## Possible inaccuracies this relies on
- The mount (height 1.2-1.5 m, 35-45°, the (0, 0) corner) is chosen from geometry, not tried; the room may not
  allow it. If not, step 1 is redone and the trust map changes.
- The blocked corner's location is taken from the simulator's default layout (boards at x and y maxima).
- The 12 cm "near a board" band and the 2-3 times scale change are estimates.
- The lean correction assumes the robot's outline is a box 3.7 cm tall; real outlines include lights and glare.
- The camrec exposure fix for the side camera was noted on 2026-10-20 but not checked in the code here.
- WiFi: the laptop got ~2 pictures a second at full JPEG quality from one stream (2026-10-22); two streams may
  need lower quality.
- Hour estimates are guesses, not from a time tracker.
