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
- `session_checks.csv` (written by tools/check_session.py, every time it runs):
  check,scope,status,value,unit,rule,detail, one row per check and robot /
  run / recording. `status`: PASS, CHECK (look at it before trusting the
  data), FAIL, SKIP (data missing; `detail` says why), INFO (reported only).
  Unless the rule says otherwise: under the limit PASS, up to twice it CHECK,
  more FAIL. `value` is the number the rule is about, in `unit`. Checks:

  | check | scope | value | pass rule |
  | --- | --- | --- | --- |
  | wifi_loss | robot run | longest RAW gap, ms | gap < 300 ms (library move pauses are ~200 ms) and loss < 2% |
  | wifi_bursts | robot run | longest run of lost rows (not counting 90-300 ms gaps that start while the wheels are still: library pauses) | INFO; detail = holes by size |
  | robot_clock | robot boot | worst clock-fit error, ms | < 5 ms, round trip < 50 ms, drift < 200 ppm and measured (clock checks span > 5 s) |
  | video_clock_drift | recording | ms the video-vs-laptop drift adds over the session | < 5 ms and scatter around the line < 10 ms (SD). A drift within the scatter is CHECK, not FAIL. Writes `video_clock.json` (below) |
  | frame_drops | recording | % of frames missing | < 0.5% (late frames, followed by an early one, are not drops) |
  | frame_rate | recording | fps | within 2% of meta.json's fps |
  | frame_jitter | recording | SD of arrival time around the camera's own timing, ms | < 5 ms; uses cam_ms, else an even frame grid (proxy) |
  | exposure | recording | exposure_ms | PASS if meta.json gives it (locked), else CHECK |
  | camera_settings | recording | settings changed | none changed between camera_controls.after and camera_controls_end |
  | camera_moved | recording | largest shift of the image centre, px | < 0.5 px and turn < 0.1 deg (first 2 s against 3 moments between and the last 2 s; image registration on the parts that don't move) |
  | lighting | recording | largest brightness change of the static floor, % | < 10% (sampled every 2 s) |
  | sync_flash_length | recording | largest error of a sync flash's length, ms | within 15 ms + (frame interval - exposure) of sync_ms |
  | blink_flash_length | recording | longest blink flash, ms | every one < 220 ms (CHECK if <= 1% are longer) |
  | tilt | robot | worst accelerometer tilt at rest, deg | < 2 deg (floor tilt + sensor mounting, not separable) |
  | gravity | robot | total acceleration at rest, g | within 0.05 g of 1 |
  | rest_noise | robot | % of still rows that pass the fusion's still test | >= 95% (CHECK >= 50%) |
  | gyro_bias | robot boot | largest first-to-last change of the at-rest gyro reading, deg/s | < 0.2 deg/s |
  | lifted | robot run | rows with lifted = 1 | none (CHECK otherwise) |
  | tof_rate | robot | new distance-sensor frames per second | >= 10/s (CHECK >= 5) and no reading older than 200 ms |
  | note | session | | any other problem check_session found (CHECK) |

  "Still" (tilt, rest_noise, gyro_bias) = both wheel angles stay within 0.72
  deg (2 encoder steps) for 0.9 s or more, not lifted; 0.2 s cut from each end.

Notes:
- `run` ids are `<boot>-<n>`: boot is 4 hex digits, random per power-on.
- `ms` is the robot's millis() (ms since its power-on).
- RAW `x_cm,y_cm,th_deg` are the robot's own wheel odometry, reset to 0 at the
  start of straight and demo runs. `step` counts completed route steps;
  -1 = the start nudge (2026-10-17e+). In a `calib` run: 1 blinking still,
  2 rolling forward, 3 back, 4 compass wiggle, 5 settling.
- RAW `lifted` (firmware 2026-10-17c+): 1 while the robot reports being picked up.
- RAW `dL_cm, dCL_cm, dC_cm, dCR_cm, dR_cm`: the front distance sensor's five
  zones, left to right (the fusion takes them as +22, +11, 0, -11, -22 deg,
  left positive: `tof_zone_deg`, to be checked on hardware); `dtop_cm`,
  `dbot_cm` its top and bottom zones. The sensor makes fewer frames than RAW
  rows (about 15 a second expected, as tools/fleet's simulator assumes;
  tools/sim gives a new frame every row; check_session's tof_rate check
  measures it); RAW repeats the last frame until the next.
- RAW gx..gz: gyro, deg/s; ax..az: accelerometer, g (to be confirmed on
  hardware: az about 1 at rest). yaw_deg: the IMU's fused yaw.
- Wheel angles (wl_deg, wr_deg) are cumulative degrees; distance = angle/360 x
  2 pi r with wheel radius r (about 1.7 cm, to be measured).

## Camera recording (`~/recordings/<date-time>_<label>/`, written by tools/pi/camrec.py)

- `video.avi`: Motion-JPEG, 640x480, at the camera's set rate (25 fps on the Vitade; meta.json fps_reported)
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
- `video_clock.json` (written by tools/check_session.py when 2+ sync flashes
  were matched): the video clock against the laptop clock,
  `video t_wall = laptop time + offset_s + drift_ppm * 1e-6 * (laptop time - t_ref)`.
  Also `drift_significant` (slope beyond 3 standard errors), `use_drift`
  (significant and worth 1 ms or more over the session: tools should use the
  slope only then, else offset_s alone), `resid_sd_ms`, `n_flashes`, `span_s`,
  `from_edges` (slope from the flashes' switch-on times, or both edges).
  `offset_s` is on the switch-on timing convention of check_session's flash
  finder (the one tools/fusion/runs.py video_offset() uses).

## Tape measurement sheet (`docs/experiments/<exp>/data/measurements.csv`)

order,robot,level,commands,run_id,start_x_cm,start_y_cm,start_heading_deg,target_x_cm,target_y_cm,target_heading_deg,
xf0_cm,yf0_cm,xr0_cm,yr0_cm,xf_cm,yf_cm,xr_cm,yr_cm,battery_pct,excluded,note

- xf/yf = front mark, xr/yr = rear mark, `0` = before the run (start), no
  suffix = after (end). Position = midpoint, heading = rear -> front.
- start_* (optional): where to place the robot centre, floor cm, and its heading; run_sheet checks the measured start is within 5 cm. Missing = the origin.
- target_* are relative to the start pose (ahead, left, turned left).
- `excluded`: 1, yes, y, x or true (any case) leaves the run out of every analysis.

## Accuracy checks (written by tools/accuracy_checks.py, in T1's `data/` or `--out`)

- `accuracy_checks.csv`: id,check,subject,result,pass_rule,verdict,meaning,if_fails,value,n.
  One row per check and robot (or other subject). `verdict` PASS | CHECK |
  FAIL | NOT YET (an input is missing; `result` says which). `value` is the
  row's headline number in the unit its result names (e.g. % for scales, cm,
  deg/s, ms).
- `measured_noise.json`: the camera noise measured against the tape while the
  robot sits still (T0 placements, each run's start and end), for the fusion:
  - `cam_xy_cm`: robot detector, SD per floor axis of camera minus tape, cm
  - `cam_heading_deg`: robot detector heading minus tape heading, SD, deg
  - `blink_xy_cm`: blink light (tools/blink) minus where the tape puts the
    light (robot centre + the light's measured offset), SD per axis, cm
  - `start_nudge_heading_deg`: heading from the start nudge
    (calibration_results.csv) minus tape start heading, SD, deg (includes the
    tape's own heading error)
  - `null` = not measured yet. `_detail` holds the biases (mean differences)
    and counts, `_made` when it was written. The tape's own reading error
    (about 0.06 cm, T0) is not taken out.
- `data/split.csv` (T1 only): order,robot,route,level,run_id,half,excluded.
  The held-out split: within each robot x route, in sheet order, runs 1 and 4
  of every four are `tune`, runs 2 and 3 `score` (A B B A, so both halves
  span the session). It depends only on the sheet's order, never on results.
  **Rule: fusion settings (noise levels, gates, scales) are tuned only on the
  `tune` runs; reported fusion accuracy comes only from the `score` runs.**

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

## Fleet map (`map.json`, written by tools/fleet/app.py)

Floor frame as everywhere else: cm, x along the long taped axis, y to its
left, heading degrees, 0 = +x, counter-clockwise positive. Poses are
`[x, y, heading_deg]`. Anything left out takes the default shown.

```json
{
 "format": "alvik-fleet-map", "version": 1,
 "floor":  {"x_min": 0, "x_max": 122, "y_min": 0, "y_max": 122},
 "robot":  {"length_cm": 9.5, "width_cm": 9.5, "front_offset_cm": 4.75},
 "dock":   {"gap_cm": 1.0, "predock_cm": 15.0},
 "boards": [{"name": "A", "p1": [30, 2], "p2": [50, 2], "thickness_cm": 1.5, "side": "auto", "dock_at": 0.5}],
 "keepouts":  [{"name": "camera stand", "polygon": [[0, 110], [12, 110], [12, 122], [0, 122]]}],
 "obstacles": [{"id": "brick-1", "shape": "polygon", "points": [[50, 70], [70, 70], [70, 79], [50, 79]],
                "source": "camera", "time": "2026-10-18T14:03:12"},
               {"id": "cup", "shape": "circle", "center": [80, 40], "radius": 4, "source": "user", "time": "..."}],
 "robots": [{"name": "Left", "start": [20, 30, 0], "park": [20, 30, 0]}],
 "tasks":  {"Left": {"visit": ["A", {"goto": [60, 90]}], "any_order": false, "park": true}}
}
```

- `floor`: the plywood's edges. Robots keep their whole body (plus the safety
  margin) on it. The default is 4 ft x 4 ft; set the measured size.
- `robot`: outer size and how far the front face is ahead of the turning
  centre (the point the fusion tracks).
- `dock.gap_cm`: front face to board surface when docked.
  `dock.predock_cm`: how far out the slow, straight final approach starts.
- `boards`: the "machines". `p1`, `p2` are the two ends of the board's centre
  line; `thickness_cm` its thickness. `side`: which side robots dock on: `left`
  or `right` of the direction p1 -> p2, or `auto` (the side facing the middle
  of the floor). `dock_at`: where along the board, 0 = p1, 1 = p2.
- Derived for each board (not stored; shown in the page and copied into
  plan.json): **dock** = the pose square to the board on its open side,
  centre `gap_cm + front_offset_cm` from the surface, facing the board;
  **pre-dock** = `predock_cm` further out on the same line, same heading.
- `keepouts`: areas robots must not enter (no physical wall; the distance
  sensor does not see them).
- `obstacles`: things found on the floor: `shape` circle (`center`, `radius`)
  or polygon (`points`); `source` camera | distance_sensor | user; `time`
  when seen (ISO 8601). Optional `note`.
- `robots`: start and park poses (park defaults to start).
- `tasks` (optional): per robot, boards (by name) or `{"goto": [x, y]}` /
  `{"goto": [x, y, heading]}` points, in order; `any_order` lets the planner
  pick the order that finishes everyone soonest; `park` (default true) ends
  at the park pose. A plain list `["A", "B"]` also works.

The **map hash** in plan.json is sha256 (first 16 hex digits) of floor,
robot, dock, boards, keep-outs, obstacles (without time/note) and robots;
tasks and notes do not change it.

## Landmarks for landmark ranging (`map.json` or `landmarks.csv`, read by tools/fusion)

`replay.py --map` and `runs.py --map` take either file. The fusion compares
each RAW distance-sensor zone (`dL_cm, dCL_cm, dC_cm, dCR_cm, dR_cm`) with the
distance to the first mapped face along that zone's direction
(tools/fusion/README.md, "Landmark ranging").

- `map.json` (Fleet map above): each board is a rectangle around its centre
  line `p1`-`p2`, `thickness_cm` thick: its two long faces and two end faces.
  Polygon `obstacles` with `source` `user` (tape-measured, e.g. a cinderblock)
  add their edges. Obstacles found by the camera or distance sensor, and
  circles, are not used.
- `landmarks.csv`: `name,x1_cm,y1_cm,x2_cm,y2_cm`, one straight face per row:
  the two ends of the surface the sensor sees, floor cm (tape-sheet axes).
  Seen from either side. A board standing on edge is one row (its face toward
  the robots); a block can be up to four rows. `runs.py --map` writes the
  faces it read from a map.json in this format (`<out>/landmarks.csv`), with
  faces named `<board> left` / `right` (side of p1 -> p2) and `<board> end p1`
  / `end p2`.

Measure the face, not the board's centre: for a 1.5 cm board on its centre
line, the face is 0.75 cm closer to the robots.

Landmarks picked on the video (tools/calibrate_floor.py, Landmarks mode, or
`--landmarks-from`; tools/landmark_pick.py) are written to `landmarks.csv`
beside `floor_calibration.json` with extra columns after the five above
(the fusion ignores them): `landmark` (the board or block name), `kind`
(board / block), `source` (`user-picked`), `thickness_cm`, `height_cm`
(blank if not typed), `tape_fixed` (how many of its ends were replaced by a
tape measurement). A board is written as its four faces around the centre
line, named as runs.py does (`<name> left` / `right` / `end p1` / `end p2`);
a block as its edges (`<name> edge 1..4`), so the fusion picks the face the
robot sees. Saving replaces earlier `user-picked` rows and rows of a landmark
with the same name, keeps all other rows, and keeps the old file as
`landmarks.csv.bak`. With `--map`, boards go into `boards` (same name: p1,
p2, thickness replaced, `side` and `dock_at` kept) and blocks into
`obstacles` (polygon, source `user`, `note` starting `user-picked`; such
obstacles no longer on the page are removed); old map kept as `map.json.bak`.

The clicks are kept in `landmarks_picked.json` (same format as the
`--landmarks-from` input): `landmarks`, each {name, kind (board / block),
clicks [[u, v], ...] (px), frame, thickness_cm, height_cm, clicked_at
(block: floor / top), fix (per click: null, or a tape position "x, y" in
inches like "36 3/32, 12 1/2", or "x, y cm", or [x_cm, y_cm])}. Board clicks
are the two ends of its foot (where it meets the floor) on the side the
camera sees; the centre line is half a thickness further from the point
below the lens. Block clicks are 2 opposite corners (rectangle along the
floor axes), 3 corners in a row (4th completed) or 4 corners, on the floor,
or on its top (`clicked_at` top, needs `height_cm` and the camera height).

## Distance-sensor calibration (`tof_calibration.json`, written by tools/fusion/tof_calib.py)

What each robot's front distance sensor really reads, and where the boards
really are, learned from runs where the camera saw the robots. Read by
`replay.py --tof-cal` (and `--tof-cal-per-run`). JSON:

- `format` "alvik-tof-calibration", `version` 1.
- `used_runs`: run ids whose readings made it. replay.py refuses to score these
  runs with this file (no circularity) unless `--allow-in-sample`.
- `zone_width_deg`: where in its slice a zone reads (0 = one ray down the
  middle; > 0 = the nearest point within a slice this wide).
- `delay_s`: how old a sensor frame is when its RAW row is sent.
- `noise_cm`, `noise_rel`: reading noise, 1 standard deviation = noise_cm +
  noise_rel x distance.
- `frame_corr`: how alike the errors of zones in the same frame are (0 = not at
  all, 1 = the same); `lag1_corr`: the same for one zone from one frame to the
  next. `systematic_cm`: what the calibration leaves that stays put for a whole
  run (the spread of each run x zone x face's mean residual, noise taken out);
  the fusion widens each face's uncertainty by it.
- `robots`: per robot name, `zone_offset_deg` (5 numbers, dL..dR: the zone's
  real direction minus 22, 11, 0, -11, -22 deg; left positive), `range_bias_cm`
  and `range_scale` (reading = scale x true distance + bias; the robots' average
  bias is held at 0, since a bias they all share can't be told from the boards
  being further), `bias_sigma_cm` and
  `yaw_sigma_deg` (how sure those are), `readings` (how many went in).
- `boards`: per board, `dx_cm`, `dy_cm`, `turn_deg` (how far it moved from the
  map given) and `readings`.
- `landmarks`: the refined faces, one per entry: `name`, `x1_cm`, `y1_cm`,
  `x2_cm`, `y2_cm` (as landmarks.csv), `board`, `sigma_cm` (how sure its
  position is). `face_sigma_cm`: the largest of these.
- `map_given`: the map the boards started from.

Beside it: `landmarks_refined.csv` (the refined faces as a landmarks.csv),
`tof_calibration_loro.csv` (`run_id,calibration`: each run's calibration made
WITHOUT that run, in `loro/<run_id>.json`, same format) and `residuals.csv`
(every reading used: run, robot, zone, face, t, reading_cm, resid_nominal_cm,
resid_calibrated_cm).

## Measured sensor noise (`replay.py --noise-file`)

`replay.py --noise-file measured_noise.json` puts the noise measured on the
real setup into every setup, instead of the simulator's guesses. It reads
accuracy_checks.py's `measured_noise.json` (above) as it is: `cam_xy_cm`,
`cam_heading_deg`, `blink_xy_cm` (null = not measured: the setting stays) and
`start_nudge_heading_deg` (start-nudge heading noise; an upper bound, since
the tape's own heading error is in it). The same keys spelled
`camera_xy_cm` / `camera_heading_deg` work too, and an optional `"noise":
{...}` sets any other `ekf.Noise` setting in its own units (radians for
angles). Keys starting with `_` and anything else are ignored. A setup's own
`noise` settings win over the file.

## Fleet plan (`plan.json`, written by tools/fleet/planner.py and app.py)

What the laptop sends to the robots, one step at a time. Times are seconds
from the start of this plan (`t0_s` is 0; after an obstacle the new plan
counts from the moment the obstacle was handled).

Top level: `format` ("alvik-fleet-plan"), `version` 1, `plan_id`, `created`,
`map_hash`, `replan_of` (plan_id it replaces, or null), `settings` (every
planner setting used), `priority_order`, `docks` (dock and pre-dock pose of
every board used), `events` (after an obstacle: per robot `action` stopped |
continues | holds | done and the `reason`), `warnings`, `stranded` (robots
left where they are until the next re-plan), `summary` (makespan_s,
sum_finish_s, planning_time_s, orders_tried, min_separation_cm,
closest_pair, near_misses, min_static_clearance_cm, approach_min_clearance_cm,
planned_dock_gaps_cm), and `robots`:

```
"robots": {"Left": {"start": [x, y, deg], "visit_order": ["B", "C", "park"],
                    "task_offset": 0, "pending": null, "finish_s": 61.2,
                    "steps": [ ...step... ]}}
```

`pending` (only for a stranded robot): the work it still has to do
(`visits`, `task_offset`, `park`, `lead_board`, `lead_phase`).

Each step:

| Field | Meaning |
| --- | --- |
| `i` | step number within this robot's list |
| `type` | `turn` (in place), `drive` (straight), `approach` (slow straight to a board, stop on the distance sensor), `wait`, `park` (final turn in place at the park spot) |
| `phase` | travel, align (turn to face a board), approach, dwell (docked, a wait), backout, escape (straight move out of a tight spot), wait, hold (waiting for the new plan), park |
| `from`, `to` | poses at the start and end of the step, `[x, y, heading_deg]` |
| `speed` | cm/s for drive/approach, deg/s for turn/park, null for wait |
| `reverse` | drive/approach only: true = backwards (heading stays `from`'s) |
| `turn_deg` | turn/park only: signed turn, counter-clockwise positive |
| `t_start`, `t_end` | planned times; `move_s` = time spent moving (the rest is settling) |
| `board`, `task` | board name (align/approach/dwell/backout) and task number |
| `after` | `[[robot, step], ...]`: steps of other robots that must be reported done before this one starts |
| `stop_at_front_cm` | approach only: stop when the front distance sensor reads this |
| `max_travel_cm` | approach only: give up (and report) if it has driven this far without the reading |
| `front_clear_cm` | forward drives: what the front distance sensor should read at the end of the step (nearest board/obstacle straight ahead; absent = nothing within 100 cm). A reading well below it means something new is in the way |
| `not_before_s` | do not start before this time (first new step after an obstacle) |
| `committed` | step carried on from the previous plan while the new one was computed |

The `after` lists are the safety rule: two steps of different robots whose
swept areas come within the separation distance never run at the same time;
whichever the plan has first must be finished before the other starts.
A robot that obeys `after` cannot run into another robot, even when some run
late. `t_start` alone does not give that (see tools/fleet/README.md).
