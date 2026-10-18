# Data formats

Every tool reads and writes these. Units: cm, degrees, seconds unless a
column name says otherwise. Heading 0 deg = +x, counter-clockwise (left)
positive. Floor axes: x along the taped x axis, y to its left.

## Console session folder (`logs/<date-time>/`, written by tools/alvik_console.py)

One CSV per record type, named `<type>.csv` (a second header change in one
session gives `<type>_2.csv`). Every row starts with two columns the console
adds:

- `robot`: robot name (e.g. Left)
- `t_recv`: laptop clock, Unix seconds, when the line arrived

then the robot's own columns:

| File | Columns after robot,t_recv |
| --- | --- |
| raw.csv | run,ms,step,wl_deg,wr_deg,wl_rpm,wr_rpm,v_cms,w_dps,x_cm,y_cm,th_deg,ax,ay,az,gx,gy,gz,roll_deg,pitch_deg,yaw_deg,dL_cm,dCL_cm,dC_cm,dCR_cm,dR_cm,dtop_cm,dbot_cm,lineL,lineC,lineR,battery,charging,lifted |
| runinfo.csv | run,experiment,fw_version,motor_fw,ms_start,battery,charging,wifi_dbm,route,yaw_sign,odo_sign (signs from the compass check, 0 = not checked; raw yaw_deg / th_deg times these = left-positive; from 2026-10-17d) |
| param.csv | run,name,value |
| runend.csv | run,result,ms_end,duration_ms,battery,wifi_dbm,raw_rows_sent,packets_sent |
| sync.csv | run,tag,ms_on,ms_off (tag = start or end; light flash on/off, robot ms) |
| clock.csv | token,robot_ms,boot |
| straight.csv | run,robot,result,cmd_cm,rpm,odo_x_cm,odo_y_cm,odo_theta_deg,imu_dyaw_deg,total_ms,battery (odo y and theta left-positive from 2026-10-17d; raw library sign before) |
| demo_step.csv | run,robot,step,x_cm,y_cm,theta_deg,ms (y and theta left-positive from 2026-10-17d) |
| demo_end.csv | run,robot,route,result,return_err_cm,heading_err_deg,total_ms,a_cm,b_cm,r_cm,battery |

Also in the folder:

- `clock_sync.csv`: robot,boot,token,t_send,t_recv,robot_ms,rtt_ms,offset_s
  (laptop asks, robot answers with its millis(); offset_s = (t_send+t_recv)/2 - robot_ms/1000)
- `session.json`: start, start_unix, laptop, python, repo_commit

Notes:
- `run` ids are `<boot>-<n>`: boot is 4 hex digits, random per power-on.
- `ms` is the robot's millis() (ms since its power-on).
- RAW `x_cm,y_cm,th_deg` are the robot's own wheel odometry, reset to 0 at the
  start of straight and demo runs. `step` counts completed route steps;
  -1 = the start nudge (2026-10-17e+). In a `calib` run: 1 blinking still,
  2 rolling forward, 3 back, 4 compass wiggle, 5 settling.
- RAW `lifted` (firmware 2026-10-17c+): 1 while the robot reports being picked up.
- RAW gx..gz: gyro, deg/s; ax..az: accelerometer, g (to be confirmed on
  hardware: az about 1 at rest). yaw_deg: the IMU's fused yaw.
- Wheel angles (wl_deg, wr_deg) are cumulative degrees; distance = angle/360 x
  2 pi r with wheel radius r (about 1.7 cm, to be measured).

## Camera recording (`~/recordings/<date-time>_<label>/`, written by tools/pi/camrec.py)

- `video.avi`: Motion-JPEG, 640x480, about 30 fps
- `frames.csv`: frame,t_wall,t_mono,cam_ms (t_wall = Pi clock, Unix seconds, when the frame arrived; cam_ms = the camera driver's own capture time in ms, optional, for checking delay)
- `events.csv`: t_wall,frame,text (notes typed while recording)
- `meta.json`: device,width,height,fps_requested,fps_reported,host,start,label,note,
  `camera_controls` (exposure / focus / white balance: `requested` (lock, auto, a
  number, or null = untouched), `auto_values` read after the 2 s warm-up, `after`
  = what the camera reports once set, `stats_auto` / `stats_after` = brightness
  and sharpness of a few frames, `steps` = every set tried (OpenCV or v4l2-ctl)
  and whether it took, `warnings`, `v4l2_after` = `v4l2-ctl --list-ctrls` if
  installed) and `camera_controls_end` (what the camera reports when recording
  stops). From 2026-10-18.

## Tape measurement sheet (`docs/experiments/<exp>/data/measurements.csv`)

order,robot,level,commands,run_id,target_x_cm,target_y_cm,target_heading_deg,
xf0_cm,yf0_cm,xr0_cm,yr0_cm,xf_cm,yf_cm,xr_cm,yr_cm,battery_pct,excluded,note

- xf/yf = front mark, xr/yr = rear mark, `0` = before the run (start), no
  suffix = after (end). Position = midpoint, heading = rear -> front.
- target_* are relative to the start pose (ahead, left, turned left).
- `excluded`: 1, yes, y, x or true (any case) leaves the run out of every analysis.

## Floor calibration (`floor_calibration.json`, beside the recording)

- `H_floor_to_px`: 3x3 matrix, floor cm -> image px: `[u, v, w] = H @ [x_cm, y_cm, 1]`, pixel = (u/w, v/w), pixel centres on whole numbers. Floor axes are the tape-sheet axes.
  Readers also accept `homography_floor_cm_to_px`, `H_floor_to_image`, `H`, `homography`, `matrix`.
- optional `camera.height_cm`: lens height above the floor (tape measure)
- optional `camera.nadir_floor_cm`: [x, y], the floor point straight below the lens (plumb line). With both, the fusion moves sightings of the robot's top (4 cm up; `object_height_cm` overrides) down to the floor point below it.
- optional `camera.height_source`, `camera.nadir_source`: how they were found
  ("tape (given)", "plumb line (given)", or "estimated: ..." from the fit).
- optional `k1`, `cx`, `cy`, `f_px`: radial lens term. The homography gives an
  ideal pixel (u, v); the recorded pixel is `c + f_px n (1 + k1 |n|^2)` with
  `n = ((u, v) - c) / f_px`. Absent or 0 = no distortion (older files). Read by
  tools/fusion/runs.py, replay.py and tools/detector/detcore.py; tools/blink
  ignores it (calibrate_floor.py prints how many cm that costs).
- optional `plane_height_cm`: if set, the homography maps that plane (the
  robot's top) instead of the floor (survey without a camera height).
- `view_floor_polygon_cm`: the image border mapped to the floor, [[x, y], ...]
  cm: the floor area the camera sees (for checking a route stays in view);
  `view_robot_polygon_cm`: where the robot's top centre is in view (needs the
  camera height).
- `fit`: how it was made (tools/calibrate_floor.py): `mode` (points / survey),
  n_points, n_inliers, rms_cm, max_cm, loo_rms_cm (left-one-out), coverage of
  `area_cm` (the floor bounds in tools/fleet/map.json if present, else
  0..122 x 0..122 cm; `--area` overrides), warnings, per-point errors (`points`) or per-run survey results
  (`survey`), and `used_runs`: runs whose tape marks or odometry were used.
  Don't score those runs' camera positions with this file: use
  `floor_calibration_loro.csv`.

Beside it, from tools/calibrate_floor.py:

- `floor_points.json`: the point pairs (resume file): `settings` and `points`,
  each {id, kind (floor / robot / run), u, v (px), x, y (cm), z (cm above the
  floor), frame, label, enabled, spot, group (run id), session}.
- `survey_pairs.csv`: u,v,x_cm,y_cm,vx_cms,vy_cms,sigma_cm,run,t,frame,kind,
  anchor_x_cm,anchor_y_cm,session (odometry path point, its velocity and
  uncertainty, paired with a camera sighting; accumulates across sessions).
  `kind` is `odo` for a path point or `tape_end` for a run's tape end marks
  (used only in calibrations that score other runs; `--no-end-marks` = off).
- `floor_calibration_loro.csv`: run_id,calibration,left_out,n_used,fit_rms_cm,
  note, then held-out errors of that run (heldout_start_err_cm /
  heldout_end_err_cm for run points, heldout_path_median_cm / _rms_cm and
  heldout_end_mark_err_cm for survey):
  which file in `floor_calibration_loro/` (made WITHOUT that run, or its fold)
  to score the run with.
- `floor_calibration_check.jpg`: a frame with the fitted 10 cm grid drawn on it.

## Labels for the camera detector (`<recording>/labels/`)

- `labels.csv`: frame,t_wall,obj,cls,x_px,y_px,heading_deg,heading_known,source,status,score
  - `obj`: object id within the recording (track id), integer
  - `cls`: alvik | hand | unknown | machine
  - `x_px,y_px`: object centre in image pixels (origin top-left, x right, y down); pixel centres on whole numbers, as in OpenCV (the top-left pixel's centre is 0,0)
  - `heading_deg`: image-frame heading, 0 = image +x (right), counter-clockwise
    positive as seen on screen (i.e. towards image -y); blank if unknown
  - `heading_known`: 1 if heading is given
  - `source`: motion | track | model | user | log
  - `status`: pending | confirmed | corrected | rejected | auto (accepted by the labelling tool as high-confidence, without a person looking; trained on, but filterable)
  - `score`: confidence 0..1 of the automatic guess (blank for user)
  - A frame with no objects that the user confirmed empty has one row with
    cls `none` (so empty frames count as reviewed negatives).
- `frames/<frame:06d>.jpg`: the frames chosen for labelling, extracted from video.avi
- `review.json`: which frames were selected and reviewed

## Detector interface (tools/detector)

A detector is a Python module with:

    load(path) -> model
    predict(model, frame_bgr) -> list of dict(cls, x, y, heading_deg or None, score)

(x, y in image pixels.) The labelling program can use any detector with this
interface to make its guesses.

## Blink detections (`<recording>/blink_detections.csv`, written by tools/blink/blink.py)

frame,t_wall,robot,x_px,y_px,code,score,track,lit,n_spots,period_s,shared,p_Left,p_Center,p_Right,p_other,p_none

- One row per frame in which a tracked blinking light (firmware 2026-10-18a
  rate code: one 100 ms flash every 0.3 s Left, 0.4 s Center, 0.5 s Right,
  0.6 s other) was lit; with `--interpolate` also rows between flashes (`lit` 0).
- `x_px,y_px`: the light's centre (midpoint of the two LEDs), image pixels as
  in labels.csv; a point on the robot's top (4 cm up), ahead of its centre.
- Per track (same on all its rows): `p_<name>` probability that the track is
  that robot (names follow `--names`), `p_none` that it is no robot code;
  `robot`/`code` filled only when the code is clear (else empty / 0);
  `score` = probability of the name given (or of the best code);
  `period_s` measured flash period; `shared` other flashing on its spot per flash.
- `track` id; `n_spots` 2 = both LEDs seen, 1 = one (moved to the pair
  midpoint), 0 = interpolated.

## Calibration results (`<recording>/calibration_results.csv`, written by tools/blink/calib.py)

run,robot_log,experiment,event,t_video,robot_blink,blink_code,blink_score,blink_p_log,identity,
others_moving,lights_moved,onset_diff_ms,heading_img_deg,heading_floor_deg,heading_method,
moved_px,moved_cm,commanded_cm,motion_quality,det_heading_floor_deg,light_x_px,light_y_px,
centre_x_px,centre_y_px,led_fwd_px,led_left_px,led_fwd_cm,led_left_cm,note

- One row per forward roll: `event` calib (RAW step 2 of a calib run) or
  nudge (RAW step -1, the start nudge). `t_video`: wheels start, video clock.
- `identity`: agree / DISAGREE between the blink code of the light that moved
  and the robot whose RAW log shows the roll then (`code not readable` if the
  light was not named); `blink_p_log`: the blink probability for that robot.
- `heading_img_deg` (image convention, as labels.csv) and `heading_floor_deg`
  (floor convention): direction of the forward roll.
- `led_fwd_*`, `led_left_*`: the light's offset from the robot's centre in
  the robot's own frame (forward, left), px and cm; needs robot-centre
  detections (`--detections`). Column details: tools/blink/README.md.
