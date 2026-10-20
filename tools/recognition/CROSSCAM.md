# Cross-camera labels (cameras.py, crosscam.py)

**What it does.** Each camera gets a full pinhole model in the floor frame (x, y cm on the
floor, z cm up). A label in camera A (centre pixel + optional heading) is placed on the plane
at the object's height (robot top = 4 cm) and projected into camera B's frame taken at the
nearest moment. That gives the labelling tool a ready-made guess (`source = camera`). Labels
that both cameras have confirmed are compared on the floor, and any mismatch is listed.

**How accurate (synthetic, from the tests).** Two simulated 640x480 cameras: A is 262 cm up and
nearly straight down; B looks down at 45 deg from 200 cm away. Each is fitted from about 30 clicks
on a 20 cm grid with 0.5 px click noise, and A is given its tape height. Results over 4 seeds:
- A to B transfer of robot tops: mean 0.14–0.27 px, worst 0.8 px. Heading: worst under 1 deg.
- With only 8 clicks per camera: mean 0.5–0.8 px, worst 3.5 px.
- Floor position (z = 0): worst 0.1–0.35 cm.
- Built from a calibrate_floor.py file instead: robot tops land within 1 px of the true camera.

**Limits.**
- *Height.* Every object is assumed to sit at its class height above the floor (`class_heights`).
  An object on a conveyor, a box or a hand in the air is placed wrongly unless you give the height
  of that plane. On the oblique camera a 2 cm height error moves the guess about 2 cm.
- *Overhead camera needs its height.* From floor points alone, a camera looking straight down
  cannot tell "zoomed in and higher up" from "zoomed out and lower down". Give `height_cm`
  (tape, lens to floor), or click a few points at a known height (`floor_z`). Without either,
  the fit warns you and guesses about 185 cm instead of 262. Floor positions are still right, but
  transfers are off by up to 3 px. `from_floor_calibration` refuses an overhead file that has no
  `camera.height_cm`.
- *Lens distortion.* Only one radial term (k1) is modelled, which is the same model as
  calibrate_floor. A wide oblique lens needs clicks out to the image edges so that k1 is kept;
  if the lens has stronger distortion than k1 can describe, the edges will be off.
- *Clicking on the oblique camera.* Click the centre of the robot's top face, not the centre of
  the visible blob. The blob includes the side of the robot, so its centre is pulled downward.
- *Time sync.* camrec.py's `t_mono` starts at 0 in each process, so the two cameras do NOT share
  it. Use `t_wall` (the default in `load_frame_times`), or pass `offset_s`. A robot at 20 cm/s
  moves 0.4 cm in half a 25 fps frame. The score allows for this; set `max_dt` to tighten it.

**Functions.**
```
CameraModel.from_points(img_pts, floor_pts, image_size, K=None, dist=None, floor_z=None,
    height_cm=None, nadir_cm=None, height_sd_cm=1.0, nadir_sd_cm=2.0, k1="auto", name=None)
CameraModel.from_floor_calibration(path)       # needs camera.height_cm for an overhead view
cam.project(xyz) -> (u, v)      cam.to_floor(u, v, z=0) -> (x, y)      cam.save/load/describe()
transfer(cam_a, cam_b, u, v, height_cm, heading_deg=None, lead_cm=3.0) -> (u_b, v_b, heading_b, visible)
load_frame_times(frames_csv, column="t_wall") -> {frame: t}
match_frames(times_a, times_b, max_dt=None, offset_s=0.0) -> {frame_b: frame_a}
cross_guesses(objects_a_by_frame, cam_a, cam_b, times_a, times_b, size_b, class_heights,
    default_height_cm=4.0, score_scale=1.0, min_score=None, ...) -> {frame_b: [guess dicts]}
disagreements(labels_a_by_frame, labels_b_by_frame, cam_a, cam_b, times_a, times_b,
    class_heights, tol_cm=2.0, match_cm=15.0, ...) -> [{frame_a, frame_b, obj_a, obj_b, cls,
    dist_cm, kind: moved | missing_in_b | missing_in_a | class, ...}]
```
**Score** = exp(-½ (σ / 3 cm)²). σ combines three things: how far the object could have moved
in the time gap (20 cm/s), both cameras' left-one-out calibration errors, and the pixel shift
from an uncertain object height. Same-instant guesses score about 0.95.
Tests: `python -m pytest tools/recognition/tests -q`
