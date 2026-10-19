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
| raw.csv | run,ms,step,wl_deg,wr_deg,wl_rpm,wr_rpm,v_cms,w_dps,x_cm,y_cm,th_deg,ax,ay,az,gx,gy,gz,roll_deg,pitch_deg,yaw_deg,dL_cm,dCL_cm,dC_cm,dCR_cm,dR_cm,dtop_cm,dbot_cm,lineL,lineC,lineR,battery,charging,lifted,swL,swR,raw_n (raw_n from 2026-10-18b; swL, swR from 2026-10-19a, before raw_n) |
| runinfo.csv | run,experiment,fw_version,motor_fw,ms_start,battery,charging,wifi_dbm,route,yaw_sign,odo_sign (signs from the compass check, 0 = not checked; raw yaw_deg / th_deg times these = left-positive; from 2026-10-17d) |
| param.csv | run,name,value |
| runend.csv | run,result,ms_end,duration_ms,battery,wifi_dbm,raw_rows_sent,packets_sent |
| sync.csv | run,tag,ms_on,ms_off,ms_on_after (tag = start or end; light flash on/off, robot ms; ms_on is taken just before the two set_color() calls that switch the lights on, ms_on_after (2026-10-18b+) just after them, so ms_on_after - ms_on is the switch-on call time) |
| clock.csv | token,robot_ms,boot |
| straight.csv | run,robot,result,cmd_cm,rpm,odo_x_cm,odo_y_cm,odo_theta_deg,imu_dyaw_deg,total_ms,battery (odo y and theta left-positive from 2026-10-17d; raw library sign before) |
| demo_step.csv | run,robot,step,x_cm,y_cm,theta_deg,ms (y and theta left-positive from 2026-10-17d) |
| demo_end.csv | run,robot,route,result,return_err_cm,heading_err_deg,total_ms,a_cm,b_cm,r_cm,battery |
| survey_leg.csv | run,leg,cmd_cm,driven_cm,end,x_cm,y_cm,th_deg,front_cm,ms: one row per leg of a `survey` run (2026-10-18b+); `end` full / obstacle (stopped early: something within survey_clear_cm in its path) / stopped / timeout; pose = the robot's own (wheels + gyro) in its start frame (0, 0, 0 where it was put down, x ahead, y left) |
| survey_end.csv | run,result,legs,path_cm,return_cm,x_cm,y_cm,th_deg,total_ms,size_cm,battery: `result` done, done:blocked (a leg ended at an obstacle, then it drove back), failed:..., aborted:...; x,y,th = own pose at the end (0,0,0 = back exactly where it started, by its own reckoning) |
| calib_look.csv | `calib` runs (2026-10-18c+), one row: run,spun,near_cm,heading_deg,turn_deg,room_cm,cap_cm,live_cm,roll_cm,result,room8,caps,spin,roll,reason: what the look-around saw and chose, what it did, and what it skipped or cut short and why (sent after the rolls; see "Calibration look-around" and "Calibration outcomes" below) |
| step.csv | `follow` runs: run,id,state,ms,x_cm,y_cm,th_deg,front_cm,info (see "Step follower" below) |
| edge.csv | plywood edge guard (firmware 2026-10-18b+, only with `safe_edge` on; edge.h): run,ms,event,lineL,lineC,lineR,baseL,baseC,baseR,dbot_cm,info. `event` base = the line sensors' plywood level learned at the start of the run (base* = each sensor's level; info says when a sensor on tape took the others' level), nobase = it could not be learned (info: all three on tape, or the robot moved too soon): the line cue is off for that run, trip = the guard stopped the robot (info: line sensors / bottom zone; RUNEND says limited:plywood edge, not a failure: "Run outcomes" below; in a calibration's forward roll it rolls back and ends done, limited (plywood edge)). After a trip the guard stays quiet until the wheels are still. line* and dbot_cm are the readings at that moment. Settings: safe_edge (0 off, 1 line sensors, 2 bottom zone, 3 both), edge_dir, edge_thr, edge_max, edge_n, edge_learn_ms, edge_min_rpm, edge_bot_cm, edge_bot_ms (params.h help; tools/edge_check.py recommends them from the E0c edge rows) |
| dock.csv | home dock (firmware 2026-10-19a+, dock.h; `dock` runs and follow step K), one row per docking: see "Home dock and touch probe" below |
| touch.csv | touch probe (2026-10-19a+; `touch` runs and follow step P), one row per touch: see "Home dock and touch probe" below |
| follow.csv | `follow` runs, follow_log_hz (10) a second: run,ms,step_id,state,queued,last_done,last_failed,x_cm,y_cm,th_deg,pos_sd_cm,th_sd_deg,fixes_used,fixes_rejected,front_cm,travel_cm,quiet_ms (see "Step follower") |

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
  | dock_tile | recording, dock | largest shift of a home dock's check tile from where it was in the first frame, px | < 0.5 px (~0.15 cm). Only with `check_session.py --dock-map map.json --calib floor_calibration.json`; the tile's place comes from the map's home_docks and the calibration; sampled every 2 s by ECC alignment of a patch around it (session_checks.PatternWatch, usable live too); frames where it is covered (match < 0.8) are left out. A shift means the camera or the dock moved |
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
  -1 = the start nudge (2026-10-17e+). In a `calib` run (firmware
  2026-10-18c+; tools/blink/calib.py reads each step) the robot looks
  around first and rolls where it found room (firmware look.h):

  | step | what the robot does | what it is for |
  | --- | --- | --- |
  | 1 | stands still blinking its code, `calib_blink_ms` (3 s) | who is who (blink code); gyro drift; front view (anything inside its spin circle?) |
  | 2 | compass wiggle (`calibrateYawSign(true)`, ~1.2 s) | gyro sign (RUNINFO yaw_sign of later runs) |
  | 3 | still `calib_still_ms` (1 s) | gyro drift; the camera's picture before the spin; front view again |
  | 4 | spin left `calib_spin_deg` (720 = two turns) at `calib_spin_rpm` (15, ~34 deg/s, ~22 s), counted by the gyro | gyro scale (camera rotation / gyro rotation), light offset (the lights circle the turning centre), the front sensor's sweep: where there is room to roll (`dL_cm..dR_cm`, `dtop_cm` at `raw_hz`) |
  | 5 | still `calib_still_ms` | picture between the spins; gyro drift |
  | 6 | spin right `calib_spin_deg` back | the same the other way (gyro drift cancels in the pair) |
  | 7 | still `calib_still_ms` | picture after the spins; gyro drift |
  | 8 | turns to the roll heading it chose (IMU turn; none if it already faces it), then still `calib_still_ms` | the picture before the roll; the front view along the lane is checked once more |
  | 9 | `move(+roll_cm)` (CALIB_LOOK `roll_cm`, up to `calib_roll_cm` 15), then still `calib_still_ms` | heading (direction of the roll on video, gyro-corrected for any curve), wheel distance scale (camera cm / odometry cm) |
  | 10 | `move(-roll_cm)`, then still `calib_still_ms` | the same measured the other way (heading spread, second wheel scale) |

  4-7 are skipped when `calib_spin_deg` is 0 or something is within
  `calib_spin_r_cm` (8) of the turning centre in steps 1-3 (then it knows
  only the view straight ahead); 9-10 are skipped when there is no room to
  roll. Skipping for lack of room is not a failure: the run still ends
  `done`, CALIB_LOOK's spin / roll / reason columns say what was skipped,
  and the tools count it as "limited" (see "Calibration outcomes"). The
  front stop (`safe_front_cm`) is on only during 9 and 10. With the plywood
  edge guard on (`safe_edge`), the forward roll (9) stops at an open edge the
  front sensor can't see; the robot then settles, rolls back the distance it
  covered (10, retracing its own path) and the run still ends `done`,
  limited (plywood edge): the roll is kept (CALIB_LOOK roll `edge`, roll_cm =
  the distance covered) if it covered at least `calib_roll_min_cm`, else it
  counts as skipped. The spins, the turn in 8 and the roll back are not
  guarded. About 62-65 s from RUNINFO to RUNEND with
  the defaults and a roll (the turn in step 8 takes 0-3 s; the 1 s
  `settle_end_ms` still at the end included), about 54 s without a roll.
  Firmware 2026-10-18b (rolled first, straight ahead): 1 blinking still,
  2 `move(+calib_roll_cm)` + still, 3 back + still, 4 compass wiggle, 5
  still, 6 spin left, 7 still, 8 spin right, 9 still (`calib_spin_deg` 0
  skipped 5-8). Before 2026-10-18b:
  1 blinking still, 2 roll max(3, `start_nudge_cm`) forward, 3 back (300 ms
  pauses), 4 compass wiggle, 5 300 ms settling. In a `survey` run
  (2026-10-18b+): the leg number (1, 2, ...), legs + 1 = the drive back to the
  start. In a `follow` run: the id of the step running (0 between steps).
- RAW `lifted` (firmware 2026-10-17c+): 1 while the robot reports being picked up.
- RAW `raw_n` (firmware 2026-10-18b+, last column): the row's number within the
  run (0, 1, 2, ... = rows sent before it; RUNEND raw_rows_sent is the count).
- RAW `swL`, `swR` (firmware 2026-10-19a+, just before raw_n, which stays last):
  the nose bumper's left and right switch, debounced (switches.h): 1 pressed,
  0 free, -1 switches not configured (`sw_on` 0, the default). Column order:
  ..., lifted, swL, swR, raw_n. Every tool reads RAW by column name, so files
  without them still load.
- From 2026-10-18b every logged run ends with `settle_end_ms` (1000) standing
  still, logging and blinking, before the end flash: a clean end pose for the
  camera and the end-marks check, and a still period for the gyro bias.
  A jump in raw_n is a row lost over WiFi; a gap in `ms` between consecutive
  raw_n is a real pause on the robot (library move()/rotate() block ~200 ms).
  Older logs have no raw_n column (everything reads columns by name, and the
  console starts a new raw_2.csv when the header changes mid-session).
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

### Calibration look-around (`calib_look.csv`, firmware 2026-10-18c+)

While it spins (RAW steps 4 and 6) the robot turns every front reading (the
five zones at +22..-22 deg and the top zone straight ahead, `tof_ahead_cm`
4.5 cm ahead of the axle) into a point around its turning centre and keeps the
nearest point per 3 deg. For each heading, the room is the distance from the
sensor to the nearest point inside the robot's lane (`path_half_cm` either
side of its centre line); the roll that fits is
min(`calib_roll_cm`, room - `safe_front_cm` - `calib_margin_cm`, laptop cap).
It picks the middle of the widest stretch of headings with room for the full
roll (straight ahead if every heading has room), else the heading with the
longest roll that fits; it does not roll if that is under
`calib_roll_min_cm` (5). It keeps the nearest of many noisy readings per
direction, and a zone reports the nearest thing across its ~11 deg, so the
room comes out short of the truth, never long: in the simulator (sensor
noise sd ~0.8 cm at 30 cm) 2-3.5 cm short, 0.25 cm without noise.
Headings are relative to the way the robot faced at step 3 (after the
wiggle, within a few degrees of where it was put down), left positive.

| column | meaning |
| --- | --- |
| spun | 1 = it spun (steps 4-7), 0 = it did not (`calib_spin_deg` 0, or something inside its spin circle) |
| near_cm | nearest point seen in steps 1-3, cm from the turning centre (0 = none); under `calib_spin_r_cm` = no spin |
| heading_deg | the roll heading it chose, relative (see above) |
| turn_deg | how far it turned in step 8 to face it, by the gyro (0 = no roll, or already facing it) |
| room_cm | room at that heading, cm ahead of the sensor (999 = nothing seen in the lane) |
| cap_cm | the laptop's cap at that heading (-1 = none) |
| live_cm | after the turn: nearest thing in the lane by the live front view during step 8's still (0 = nothing in range) |
| roll_cm | the roll it made (0 = none); after an edge-guard stop, the distance it covered (wheel odometry) |
| result | `roll` (full roll), `short` (less room: shorter roll), `no_room` (no heading had room for `calib_roll_min_cm`), `no_room_after_turn` (the live view after the turn had less room than the sweep), `edge` (the edge guard stopped the forward roll at a plywood edge) |
| room8 | room at 0, 45, ..., 315 deg, `;` between (-1 = not seen) |
| caps | the laptop's caps used, `deg:cm` with `;` between (empty = none) |
| spin | `done` (it spun), `skipped` (something inside its spin circle: no room to spin), `off` (`calib_spin_deg` 0) |
| roll | `full`, `short` (less room: a shorter roll), `edge` (cut short at a plywood edge, at least `calib_roll_min_cm`: kept) or `skipped` (no room to roll in any direction or after the turn, or stopped at an edge sooner than `calib_roll_min_cm`) |
| reason | why it is limited: `no room` (spin or roll skipped for lack of room), `plywood edge` (the edge guard stopped the roll), both as `no room; plywood edge`, or empty |

### Calibration outcomes (three states)

Every calibration run ends in one of three states (tools/session_checks.py
`calib_outcome()`, used by run_sheet.py, check_session.py, tools/blink/calib.py,
accuracy_checks.py and tools/fleet/run_plan.py):

| outcome | when | what it means |
| --- | --- | --- |
| `done` | RUNEND `done`, CALIB_LOOK `reason` empty | everything it tries was measured |
| `limited (<reason>)`: `limited (no room)`, `limited (plywood edge)`, `limited (no room; plywood edge)` | RUNEND `done`, CALIB_LOOK `reason` not empty | NOT a failure: the robot refused to spin or roll where it had no room, or the edge guard stopped its roll at an open plywood edge. What it did is kept (blink code, compass sign, the spins, and the roll if it made one: a roll cut short at an edge counts when it covered at least `calib_roll_min_cm`); the rest comes from the camera, later runs or a later calibration. calibration_results.csv leaves those values blank with quality `skipped: no room` or `skipped: plywood edge`; run_sheet.py says what was kept and offers `r` to move it and redo, Enter to go on |
| `failed` | RUNEND `failed:...` or `stopped:...` (a stop command, cancel button, lifted, an obstacle), or no RUNEND | it did not finish: calibrate again |

Logs from before the spin / roll / reason columns: CALIB_LOOK result
`no_room` or `no_room_after_turn` with RUNEND `done` counts as limited.

### Run outcomes (every experiment, firmware 2026-10-18c+)

RUNEND `result` is `done`, `limited:plywood edge`, `stopped:<why>` or
`failed:<why>` (tools/session_checks.py `run_outcome()`: done / limited /
failed). `limited:plywood edge` = the edge guard (`safe_edge`) stopped the
robot before it drove off an open edge of the board: the system refused to
leave the work area, which is not a failure (of localization or anything
else), but the run did not finish its route. So:

- run_sheet.py says so and asks: Enter marks the row `excluded` = 1 with the
  note `limited: plywood edge` (r = redo, m = measure it anyway: the note then
  says `limited: plywood edge (measured anyway)` and it still stays out of
  the statistics). Unless redone, it also adds a replacement row (same robot,
  level, commands and targets; a new order number; note `replaces order N
  (limited: plywood edge)`) at the end of that robot's block of rows, so each
  level keeps its planned number of runs; the replacement itself is an
  ordinary run.
- tape_results.py and accuracy_checks.py leave such rows out of every
  statistic (a measured row whose RUNEND says limited too) and print how many
  were left out; check_session.py and summarize.py count them apart from
  unfinished runs; simulate.py marks them the same way on its sheet.
- Follow runs (fleet) don't end at an edge: the guard stops only the step
  (STEP failed `plywood edge at X Y TH CUE`, "Step follower"), the robot holds,
  and tools/fleet/run_plan.py adds the edge to the map as a no-go strip
  (obstacle source `edge_guard`: a rectangle square to the map's floor side
  the robot drove at, when that side is within 40 deg of square on and 12 cm of
  the trip, else square to the robot's heading; starting 3.5 cm ahead of the axle for the line cue or 8 cm for the bottom
  zone, 40 cm either side, 25 cm deep: assumptions until hardware trips
  confirm them), logs it (`edge_boundary` in its log) and re-plans the
  affected robots; the others carry on. A follow run from older firmware that
  ends `limited:` ends the plan with result `limited: ...`, exit code 0.

A calibration run is the exception: its forward roll stopped at an edge rolls
back and ends `done` (limited (plywood edge) through CALIB_LOOK, above).

Laptop caps (the front sensor cannot see an open plywood edge): the console
command `set calib_dir_limits <deg> <cm> [<deg> <cm> ...]` (up to 8 pairs;
`:` or `,` may separate them) caps the roll to `cm` for headings within
22.5 deg of `deg` (relative, as above; the smaller cap where two overlap;
headings no pair covers are not capped). They apply to the next calibration
run only, then are cleared; `set calib_dir_limits off` clears them sooner and
`get calib_dir_limits` shows them. Off unless sent.

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
  installed, `exposure_cap` = 10000/fps: a locked or brightness-matched exposure
  never goes above it, `dynamic_framerate` = what happened to
  exposure_dynamic_framerate (set to 0 with v4l2-ctl if offered), `fps_check` =
  frame rate measured for ~2 s after setting a manual exposure: requested,
  measured, frames, seconds, ok (false below 0.9 x requested; also copied to the
  top level as `fps_check`)) and `camera_controls_end` (what the camera reports when recording
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

**Live stream** (`http://<pi>:8000/live`, camrec.py 2026-10-18+): every frame
camrec.py records, as an HTTP `multipart/x-mixed-replace; boundary=frame`
stream. Each part: `Content-Type: image/jpeg`, `Content-Length`, `X-Frame` (the
frame number, as in frames.csv), `X-Time` (its t_wall, Pi clock), then a
grey JPEG (quality 90; `/live?q=80` for less). A part is skipped (X-Frame jumps)
when the reader falls behind. Read by tools/live/track_live.py --stream.

## Tape measurement sheet (`docs/experiments/<exp>/data/measurements.csv`)

order,robot,level,commands,run_id,start_x_cm,start_y_cm,start_heading_deg,target_x_cm,target_y_cm,target_heading_deg,
xf0_cm,yf0_cm,xr0_cm,yr0_cm,xf_cm,yf_cm,xr_cm,yr_cm,battery_pct,excluded,note

- xf/yf = front mark, xr/yr = rear mark, `0` = before the run (start), no
  suffix = after (end). Position = midpoint, heading = rear -> front.
- start_* (optional): where to place the robot centre, floor cm, and its heading; run_sheet checks the measured start is within 5 cm. Missing = the origin.
- target_* are relative to the start pose (ahead, left, turned left).
- `excluded`: 1, yes, y, x or true (any case) leaves the run out of every analysis.
- Readers accept a UTF-8 byte-order mark (Excel's "CSV UTF-8" adds one);
  the tools write plain UTF-8 without it.

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
  `area_cm` (the floor bounds of the fleet map: `--map`, else ./map.json
  where tools/fleet/app.py saves it, else tools/fleet/map.json; else
  0..122 x 0..122 cm; `--area` overrides), warnings, per-point errors (`points`) or per-run survey results
  (`survey`), and `used_runs`: runs whose tape marks or odometry were used.
  Don't score those runs' camera positions with this file: use
  `floor_calibration_loro.csv` (tools/fusion/replay.py and runs.py refuse
  unless `--allow-in-sample`). Survey files also have `used_robots`: robots
  whose wheel odometry went into the fit.
- held-out files (`floor_calibration_loro/`) also have `held_out_runs` (and,
  leave-one-robot-out, `held_out_robots`).

Beside it, from tools/calibrate_floor.py:

- `floor_points.json`: the point pairs (resume file): `settings` and `points`,
  each {id, kind (floor / robot / run), u, v (px), x, y (cm), z (cm above the
  floor), frame, label, enabled, spot, group (run id), session}.
- `survey_pairs.csv`: u,v,x_cm,y_cm,vx_cms,vy_cms,sigma_cm,run,t,frame,kind,
  anchor_x_cm,anchor_y_cm,session,robot (odometry path point, its velocity and
  uncertainty, paired with a camera sighting; accumulates across sessions;
  `robot` from 2026-10-18, empty in older rows).
  `kind` is `odo` for a path point or `tape_end` for a run's tape end marks
  (used only in calibrations that score other runs; `--no-end-marks` = off).
- `floor_calibration_loro.csv`: run_id,calibration,left_out,n_used,fit_rms_cm,
  note,holdout (run), then held-out errors of that run (heldout_start_err_cm /
  heldout_end_err_cm for run points; heldout_<order>_<start|end>_err_cm for E00
  placement spots, one per sheet row, so both passes at a spot are kept;
  heldout_path_median_cm / _rms_cm and heldout_end_mark_err_cm for survey):
  which file in `floor_calibration_loro/` (made WITHOUT that run, or its fold)
  to score the run with.
- `floor_calibration_loro_robot.csv` and `floor_calibration_loro_robot/`
  (`--survey --holdout robot`): the same, one row per run, but each file is
  made without ANY run of that run's robot (holdout = robot, left_out = the
  robot). Use it like the per-run one (`--calib-per-run`).
- `floor_calibration_check.jpg`: a frame with the fitted 10 cm grid drawn on it.

## Fusion replay (`results/<name>/`, written by tools/fusion/replay.py)

- `results.csv`: config,run_id,robot,route,end_err_cm,end_head_err_deg,
  path_rmse_cm,path_max_cm,head_rmse_deg,cam_used,cam_blocked,kidnaps,
  hdg_motion,hdg_calib,id_right,id_wrong,id_false,id_blink,id_motion,id_both,
  id_combined,id_position,calib_file,calib_includes_run. The last two are filled
  for setups that use the camera or blink lights: the calibration that converted
  that run's sightings, and 1 if that calibration's fit.used_runs includes the
  run (only possible with `--allow-in-sample`).
- `summary.csv`: setup,route,runs,end error cm,95% interval,vs wheels only
  (paired),p,p_holm[,path RMSE cm,heading RMSE deg][,right robot]. Rows with
  route `all` pool every route (p_holm: p adjusted for comparing many setups at
  once, Holm method); the other rows are one setup on one route (runs, mean end
  error, interval only).

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

## Light-pattern detections (`<recording>/lights_detections.csv`, written by tools/lights/lights.py)

Optional second identity source (robots' top LED patterns; design: tools/blink/README.md "Light patterns").
Written by `python tools/lights/lights.py <recording>` or `blink.py --lights l3|strip`.

frame,t_wall,robot,x_px,y_px,heading_img_deg,code,score,track,kind,n_leds,off_fwd_cm,off_left_cm,led_height_cm,p_Left,p_Center,p_Right,p_none,source

- One row per frame per pattern track seen in that frame (all three L LEDs, two of them following a
  known track, or a strip).
- `robot`: the name, only when sure (posterior >= `--name-p` 0.9, enough seen, and no other track
  elsewhere carries or may carry that name); else empty. `code` its pattern code (`S0L`, `S0L/0011`;
  strip `1`..`5`), `score` the best name's probability.
- `x_px,y_px`: the pattern's LED centre (mean of its LEDs), image pixels, `led_height_cm` above the floor.
  `off_fwd_cm,off_left_cm`: that point in the robot frame from the turning centre (patterns.py geometry).
- `heading_img_deg`: which way the robot faces, 0 = image right, counter-clockwise on screen (as labels.csv).
- `p_<name>`: probability per listed name; `p_none`: not a listed pattern (another code or not a robot).
- `kind`: `full` (whole pattern) / `pair` (two of the L's LEDs on a known track); `n_leds`; `track` id;
  `source` = `lights`.
- tools/fusion: sensor `lights` (`replay.py --lights`), identity source name `lights` (`id_lights` in
  results.csv), heading fixes counted in `hdg_used["lights"]`.

Firmware parameters (lights.h, firmware 2026-10-19a; all off unless `lights_mode` is set):
`lights_mode` 0 off / 1 L on GPIO pins / 2 Modulino Pixels; `lights_seq` 0..4 side-LED cycle (1111, 0111,
0011, 0101, 0001; 160 ms slots); `lights_pin_c/_f/_s` GPIO numbers (-1 = not wired); `lights_strip_code`
1..5 (0 = by name); `lights_strip_bri` 0..100; `lights_idle` 1 = on also between runs. Console command
`lights`: shows the pattern for 10 s (idle only). The values go out with every run's PARAM records.

## Calibration results (`<recording>/calibration_results.csv`, written by tools/blink/calib.py)

run,robot_log,experiment,event,t_video,robot_blink,blink_code,blink_score,blink_p_log,identity,
others_moving,lights_moved,onset_diff_ms,heading_img_deg,heading_floor_deg,heading_method,
moved_px,moved_cm,commanded_cm,motion_quality,det_heading_floor_deg,light_x_px,light_y_px,
centre_x_px,centre_y_px,led_fwd_px,led_left_px,led_fwd_cm,led_left_cm,note

- One row per calib run and per forward roll: `event` calib (the forward
  roll of a calib run: RAW step 9 from 2026-10-18c, step 2 before; a
  2026-10-18c run that found no room to roll still gets a row, with the spin
  results, identity from the blink code only and `note` saying so) or nudge
  (RAW step -1, the start nudge). `t_video`: wheels start, video clock.
- `identity`: agree / DISAGREE between the blink code of the light that moved
  and the robot whose RAW log shows the roll then (`code not readable` if the
  light was not named); `blink_p_log`: the blink probability for that robot.
- `heading_img_deg` (image convention, as labels.csv) and `heading_floor_deg`
  (floor convention): direction of the forward roll.
- `led_fwd_*`, `led_left_*`: the light's offset from the robot's centre in
  the robot's own frame (forward, left), px and cm; needs robot-centre
  detections (`--detections`). Column details: tools/blink/README.md.
- Calib runs from firmware 2026-10-18b (blank for older runs and start nudges):
  heading_chord_deg, heading_gyro_corr_deg, heading_back_deg, heading_spread_deg,
  heading_quality, moved_back_cm, back_quality, wheel_cm, wheel_back_cm,
  wheel_scale, wheel_scale_spread, wheel_scale_quality, gyro_bias_dps,
  spin1_gyro_deg, spin1_cam_deg, spin2_gyro_deg, spin2_cam_deg, spin_match,
  gyro_scale, gyro_scale_spread, gyro_scale_trace, gyro_scale_quality, spin_centre_x_px,
  spin_centre_y_px, led_points, led_radius_cm, led_fit_rms_cm,
  led_angle_sd_deg, spin_led_fwd_cm, spin_led_left_cm, led_offset_quality;
  from 2026-10-18c also look_result, look_heading_deg, look_room_cm,
  look_spin, look_roll, look_reason (its CALIB_LOOK result, heading_deg,
  room_cm, spin, roll, reason) and `outcome` (`done`, `limited (<reason>)` or
  `failed`: "Calibration outcomes" above). `commanded_cm` is then the roll
  it made (CALIB_LOOK roll_cm: after an edge-guard stop, the distance it
  covered; the row's note says so). What the robot skipped is left blank
  (not measured) with quality `skipped: no room` (or `skipped: plywood edge`
  for a roll stopped too soon): no roll =
  heading and wheel scale (and the light's offset has no direction, no
  heading: led_radius_cm only); no spin = gyro scale and the light's offset.
  `heading_floor_deg` is then the heading the robot stood at before the roll
  (forward and back roll combined); `wheel_scale` = camera cm / wheel
  odometry cm (multiply odometry distances by it); `gyro_scale` = camera
  degrees / gyro degrees (multiply gyro turns by it); `spin_led_*` = the
  light's offset from the turning centre (the wheel-axle midpoint, the point
  the robot's pose refers to). Each `*_quality` is `ok`, `check: <why>`, or
  `skipped: no room` / `skipped: plywood edge` (not measured, and not a problem).
  Column details and thresholds: tools/blink/README.md.

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
  or polygon (`points`); `source` camera | distance_sensor | edge_guard (an open plywood edge a follow step stopped at) | user; `time`
  when seen (ISO 8601). Optional `note`.
- `robots`: start and park poses (park defaults to start).
- `home_docks` (optional, 2026-10-19a): printed home docks fixed to the plywood
  (hardware/dock). Each `{"name": "h1", "x": 112, "y": 100, "heading": 0}`:
  x, y = the middle of the back wall's INNER face (where the bumper touches),
  heading = the way a docked robot faces (into the wall). Optional sizes (cm,
  defaults = the printed part, hardware/dock/dock_params.py): `pocket_w_cm`
  9.6, `pocket_depth_cm` 1.2, `funnel_len_cm` 5.0, `funnel_flare_cm` 3.2,
  `wall_t_cm` 0.32, `back_depth_cm` 4.8, and `mouth_cm` 15 (a robot going home
  stops with its axle this far in front of the mouth, facing in, then creeps
  in: follow step K). The planner treats each dock's outline as an obstacle.
  The fusion (replay.py --map) takes them as `dock_poses` (DOCK records = pose
  fixes) and the back wall's inner face as a landmark face; a site setup makes
  each one a landmark (`dock <name>`, tape points = the inner face's two ends)
  and the locked map carries the measured x, y, heading.
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
- `corr_time_frames`: how many readings are worth one independent one (the
  zone errors' correlation over about 1 s, times what the other zones of the
  same frame add); `tof_weight: "corr"` widens each reading's noise by its
  square root.
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

## Landmark safety gate (`tof_gate.csv`, written by replay.py --tof-gate)

One row per landmark setup: `setup`, `tune_runs` (how many tuning runs it was
judged on), `decision` (landmarks ON / OFF), and `end_err_cm with/without`
(and `path_rmse_cm with/without` with --truth): the mean on the tuning runs
with landmarks and with the same setup without them. results.csv then has
`half` (tune / score) and `tof_gate` (on / off) columns and only the scoring
runs.

## Locked site map (`<site>/map.json` + `landmarks.csv`, written by tools/site/setup.py)

The floor as the site setup measured and checked it (tools/site/README.md).

**map.json** is a Fleet map (above), readable by every tool that reads one,
with the boards, user blocks and the floor's open sides moved to where the
setup found them, plus a `"site"` block:

| Field | Meaning |
| --- | --- |
| `format`, `version` | `"alvik-site-map"`; the map's version, 1, 2, ... (each lock of the same site folder adds 1) |
| `locked`, `locked_at` | true; ISO 8601 time of the lock |
| `map_hash` | Fleet map hash (above) of this map when locked. A reader treats the map as locked only if it still matches: editing the map in tools/fleet/app.py (which drops unknown blocks anyway) unlocks it |
| `camera_calibration` | `path`, `hash` (sha256, 16 hex) and fit quality (`mode`, `rms_cm`, `loo_rms_cm`, `n_points`) of the floor_calibration.json the survey used. A new camera calibration means a new site setup |
| `camera_frame_sigma_cm` | the camera calibration's own error, included in every sigma below |
| `landmarks` | per landmark: `kind` (board / block), `move_cm` [dx, dy] and `turn_deg` from the clicked position, `stretch_cm` (board made this much longer), their sds (`sd_move_cm`, `sd_turn_deg`, `sd_stretch_cm`), `sigma_cm` / `sigma_in` (the largest sd of any of its points), `points`: each tape-measurable point (`left p1`, `left p2`, `right p1`, `right p2` = the floor corners of each long face, left/right of p1 -> p2; blocks: `corner N`) with `x_cm`, `y_cm`, `sigma_cm` (larger semi-axis) |
| `edges` | per side `x0`, `x1`, `y0`, `y1`: `open`, `offset_cm` (outwards), `turn_deg`, `position_cm` (its x or y at the middle), `sigma_cm` (open sides) |
| `audit` | rounds: `round`, `passed`, `points` (each: `point`, `why` = largest uncertainty / random check, `est` [x, y] cm, `sigma_cm`, `tape` [x, y] cm, `error_cm`, `allowed_cm` = max(0.3, 3 x hypot(sigma, 0.1)), `passed`). The map is written only after a passed round |
| `fill_in` | points tape-measured before the audit because the robots left them less sure than 0.5 cm: `point`, `sigma_cm`, `tape` |
| `robots` | each robot's front sensor as the survey measured it: `zone_offset_deg` (5), `range_bias_cm`, `range_scale` and their sds (as tof_calibration.json) |
| `sensor` | `zone_width_deg` |
| `survey` | what was driven (session folders, robots) |

**landmarks.csv** (beside it): the faces as the fusion reads them
(`name,x1_cm,y1_cm,x2_cm,y2_cm`, Landmarks above), plus `landmark`, `kind`,
`source` (`site-setup`), `sigma_cm`, `sigma_in`, `map_version`, `locked_at`.
Earlier files are kept as `map.json.bak`, `landmarks.csv.bak`.

**tof_calibration.json** (beside it): the same landmarks and robots in the
distance-sensor calibration format (above; `source` `site-setup`, `used_runs` =
the survey's runs, none of them scored runs), so `replay.py --tof-cal` uses the
locked landmarks with their sigma.

**audit.csv**: one row per audited point: round, point, why, map_x_cm,
map_y_cm, sigma_cm, tape_x_cm, tape_y_cm, error_cm, allowed_cm, passed.

**site_state.json**: the setup's own progress (open sides, landmarks kept,
session folders); not read by other tools.

**site_changes.jsonl** (in a session folder, tools/site/change.py or
`run_plan.py --site-map`): one JSON line per flagged landmark: `t`, `landmark`,
`median_resid_cm` (reading minus map: + = further than the map), `threshold_cm`,
`readings`, `seconds`, `robots`, `message`.

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
| `type` | `turn` (in place), `drive` (straight), `approach` (slow straight to a board: distance sensor, then wheels), `wait`, `park` (final turn in place at the park spot) |
| `phase` | travel, align (turn to face a board), approach, dwell (docked, a wait), backout, escape (straight move out of a tight spot), wait, hold (waiting for the new plan), park |
| `from`, `to` | poses at the start and end of the step, `[x, y, heading_deg]` |
| `speed` | cm/s for drive/approach, deg/s for turn/park, null for wait |
| `reverse` | drive/approach only: true = backwards (heading stays `from`'s) |
| `turn_deg` | turn/park only: signed turn, counter-clockwise positive |
| `t_start`, `t_end` | planned times; `move_s` = time spent moving (the rest is settling) |
| `board`, `task` | board name (align/approach/dwell/backout) and task number |
| `after` | `[[robot, step], ...]`: steps of other robots that must be reported done before this one starts |
| `gap_cm` | approach only: front face to board surface when docked (the map's `dock.gap_cm`) |
| `handover_cm` | approach only: distance-sensor reading below which the wheels finish the approach (planner setting `handover_cm`, 3; the sensor's minimum range is about 2 cm) |
| `max_travel_cm` | approach only: never roll further than this from the step's start point: approach length + `gap_cm` - `approach_margin_cm` (15 + 1 - 0.5 = 15.5 cm), so even a robot that never gets a reading stops short of the board. Until 2026-10-18 it was length + 3 cm (18 cm: past the board) with `stop_at_front_cm` 1.0 (below the sensor's range); plans that old are refused by run_plan.py |
| `front_min_cm` | turns, forward drives and approaches: the smallest distance the front distance sensor (centre, from the front face) should see during the step, from the map (boards and obstacles; keep-outs and the floor edge are not walls); absent or null = nothing within 100 cm. The robot stops the step (failed:obstacle) below this minus `front_margin_cm`, so planned passes near boards don't trip the 7 cm stop |
| `front_clear_cm` | forward drives: what the front distance sensor should read at the end of the step (nearest board/obstacle straight ahead; absent = nothing within 100 cm). For a straight drive this equals `front_min_cm` |
| `not_before_s` | do not start before this time (first new step after an obstacle) |
| `committed` | step carried on from the previous plan while the new one was computed |

The `after` lists are the safety rule: two steps of different robots whose
swept areas come within the separation distance never run at the same time;
whichever the plan has first must be finished before the other starts.
A robot that obeys `after` cannot run into another robot, even when some run
late. `t_start` alone does not give that (see tools/fleet/README.md).

## Step follower (`run follow`, firmware 2026-10-18b+; laptop side tools/fleet/run_plan.py)

The laptop releases each robot's plan.json steps as their `after` lists are
done; the robot buffers up to 8, runs them in order on its own pose and reports
back. Commands go to the robot's command port (UDP 5006) like any console
command, one per packet, at most 159 characters; numbers in cm and degrees,
floor frame (map.json). `-` means "none".

Laptop -> robot (only while the follow run is going):

| Command | Meaning |
| --- | --- |
| `fs <id> T <h> <fmin>` | turn in place to heading h (IMU turn) |
| `fs <id> D <x0> <y0> <x1> <y1> <h> <cms> <rev> <fmin>` | straight drive along (x0,y0) -> (x1,y1) facing h, rev 1 = backwards; small heading corrections bring it back onto the line; it squares up first if more than 10 deg off h |
| `fs <id> A <x0> <y0> <x1> <y1> <h> <cms> <handover> <gap> <max_travel> <fmin>` | approach a board (below) |
| `fs <id> W <ms>` | wait (a dwell at a board; 0 = a step that only keeps the order) |
| `fs <id> K <wall_x> <wall_y> <h> <max_travel> <back>` | home dock (2026-10-19a+, needs `sw_on 1`): square up to h if more than 3 deg off, creep in until both nose switches click (dock.h), set its own pose to the dock's (wall point minus bump_ahead_cm along h; sd 0.05 cm, 0.3 deg), stand still dock_still_ms (gyro drift, taken off its heading from then on), back out `back` cm (0 = stay). A DOCK record; done info `docked; pivot <deg>; stripe <cm>`. plan.json step type `home` (fields dock, wall, max_travel_cm, back_cm), added by `run_plan.py --home` |
| `fs <id> P <h> <max_travel> <back>` | touch probe (2026-10-19a+, needs `sw_on 1`): square up to h, creep until a nose switch clicks, back off `back` cm. A TOUCH record; done info `touch <L|R> <travel cm>`. plan.json step type `touch` (site survey `--touch`) |
| `fix <t_ms> <x> <y> <th> <sigma_cm> [<sigma_deg>]` | camera pose of the robot (turning centre) at ROBOT time t_ms; th `-` = position only |
| `fpose <x> <y> <th> [<sigma_cm> <sigma_deg>]` | set the pose outright (no camera: the plan's start spot) |
| `hold [<id>]` | finish the step running (and queued steps up to id), drop the rest; acked with STEP `held` (id = the given id or 0) |
| `halt` | stop at once (the step running fails with `halted`), drop every queued step, stay in follow mode; acked with STEP `halted` (id 0) |
| `stop` | stop at once and end the run (RUNEND), as for every experiment |

Step ids are 1-65535 and must go up for each robot. A step whose id is not
above every id the robot has seen is answered `dup` and not run again, so a
resent step can't run twice; therefore the laptop sends a robot's next step
only once the one before it was acked (`queued` or `dup`), and sends no step
while a `hold`/`halt` is unacked. It resends an unacked step or control every
0.3 s and keeps up to 2 steps beyond the one running (`--ahead`).

`fmin` is the plan's `front_min_cm`. Front checks during the follow run
(firmware motion.h / follow.h):
- turns: centre zone below min(safe_front_cm, fmin + tof_recess_cm - front_margin_cm);
- forward drives: anything inside the robot's path (all five zones, each taken
  at its direction, kept if within path_half_cm of the centre line) closer than
  min(max(follow_look_cm, safe_front_cm), fmin + tof_recess_cm - front_margin_cm,
  distance still to go in this step + front_end_cm);
- backwards drives and waits: none;
- a reading must stay too close for front_confirm_ms (100 ms, about 2 sensor
  frames) before the step stops; then the step fails (`obstacle ...`), the
  queued steps are dropped, and the robot stands still in follow mode.
Lift, the cancel button and `stop` end the run as in every experiment.

**Approach** (A): drives along the line at `cms` (3 cm/s) holding heading h
(sideways corrections only more than 3 cm from the dock). Every tof_ms (66 ms)
it reads the centre zone: a reading below what the map expects (L + gap +
recess - travel) minus front_margin_cm twice in a row fails the step
(`obstacle`, something in front of the board); readings between tof_min_cm (2)
and fuse_from_cm (10) are averaged as "reading + wheel travel when it was
taken (travel - speed x tof_lag_ms)" = where the board is. At the first reading
at or below `handover` (with 3 or more readings averaged) the wheels roll the
rest so the reading would be gap + tof_recess_cm. If no readings come, it
rolls to the planned dock (`wheels only` in the done record). It never rolls
more than max_travel - (how far past the step's start point it already is, by
its own pose).

**Own pose** (pose.h): distance from the wheels (the library's odometry step
along its own heading), heading from the gyro tied to the floor by an offset.
A `fix` is blended in with a one-step Kalman gain var / (var + sigma^2) at the
pose the robot had at t_ms (from a 1.28 s history every 40 ms; the history and
current pose shift with it, rotated about that point for the heading part).
Ignored (fixes_rejected): older than fix_max_age_ms (1000), more than 0.2 s in
the future, older than the history, further than fix_gate standard deviations
(4) and 3 cm from the own pose (heading: 4 sd and 5 deg). The first fix (or
`fpose`) sets the pose; until then the robot runs no step (FOLLOW state
`nopose`). Variances grow by pose_q_cm per cm driven, pose_q_deg per degree
turned and pose_q_dps per second.

Robot -> laptop, record STEP (step.csv): run,id,state,ms,x_cm,y_cm,th_deg,front_cm,info

| state | when | info |
| --- | --- | --- |
| queued | step accepted into the buffer (the ack) | |
| dup | id not above the highest seen: not run again (also an ack) | |
| full | buffer (8) full: send again later | |
| bad | could not read the step | why |
| started / done | the step began / finished | done approach: "N readings; wheels from R cm", "wheels only: no distance readings" or "travel cap C cm reached; N readings" |
| failed | the step stopped | `obstacle D cm ahead; zones L CL C CR R` (the five readings, for the obstacle's outline), `plywood edge at X Y TH CUE` (2026-10-18c+, `safe_edge` on: the edge guard stopped the step at an open plywood edge; own pose and the cue, `line` or `bottom`; the robot holds and stays in follow mode, and run_plan.py maps the edge as a no-go strip and re-plans), `halted`, `turn timed out`, `drive timed out`, `approach timed out`, or the run's end reason (`robot lifted`, `stop command`, `cancel button`) |
| dropped | a queued step was removed | `hold`, `halt`, `an earlier step failed`, `run ended` |
| held / halted | ack of hold / halt | |

x, y, th: the robot's own (fused) pose at that moment; front_cm: centre zone.

Record FOLLOW (follow.csv), follow_log_hz a second: run, ms, step_id (running,
0 = none), state (nopose, idle, turn, drive, approach, wait), queued (steps in
the buffer), last_done, last_failed (ids), x_cm, y_cm, th_deg (own pose),
pos_sd_cm, th_sd_deg (its uncertainty), fixes_used, fixes_rejected, front_cm,
travel_cm (wheel travel in the current step), quiet_ms (since the last packet
from the laptop). The laptop treats every id up to last_done as done (steps run
in order), so a lost `done` record costs nothing. If the laptop goes quiet
(follow_quiet_ms, 1.5 s) the robot finishes what it has buffered, then stands
still and says so.

**Position source** for run_plan.py `--fixes` (the live camera / fusion, the
simulator, or a replay):
- `udp[:PORT]` (default 5010): one JSON object per UDP packet:
  `{"type": "fix", "robot": "Left", "t": <laptop Unix time the camera frame was taken>,
  "x": cm, "y": cm, "th": deg (or null), "sigma": cm, "sigma_th": deg}` (the
  robot's turning centre, floor frame), and
  `{"type": "obstacle", "id": "...", "shape": "polygon", "points": [[x, y], ...]}`
  (or `"shape": "circle", "center", "radius"`; `"source"`, default camera),
  reported once per id. run_plan.py turns `t` into robot time with each robot's
  clock offset (the lower envelope of laptop receive time - robot ms over the
  last 30 s of its STEP/FOLLOW records) and forwards at most fix_hz (10) fixes
  a second per robot.
- `replay:FILE`: CSV t,robot,x_cm,y_cm,heading_deg[,sigma_cm]; t in seconds
  from the start of the run, or laptop Unix time.

**The live link** (tools/live/track_live.py) sends these `fix` objects, plus
fields run_plan.py ignores: `id_p` (probability of the name, from the blink
rhythm; a fix is sent only at 0.9 or more and when no other light may carry
the same name), `path` (`fast`: from the frame itself, the flash was due
there; `late`: once the flash had ended and passed blink.py's checks), `t_pi`
(the frame's Pi t_wall), `frame` (frame number since track_live.py started),
`track`. With `--lights l3|strip` (off by default) it also sends light-pattern fixes, marked
`"src": "lights"`, `"path": "lights"` (`th` from the pattern, `sigma` 0.5 cm, `sigma_th` 2 deg); unsure
patterns are `sighting`s with `"src": "lights"`. `th` is always given (from the line between the two LEDs);
`sigma` 0.5 cm, `sigma_th` about 1.5 deg. With `--sightings` it also sends
`{"type": "sighting", "t_pi", "t", "x", "y" (light midpoint, cm), "track",
"best" (likeliest name), "probs" {name: p}, "p_none", "why" (why no fix:
`too few flashes`, `code unclear (p ..)`, `off its beat`, `other flashing on
its spot`, `named` = name sure but front/back not yet), "heading" (resolved /
unsure)}`; run_plan.py ignores those. track_live.py writes in its `--log`
folder (default `logs/<date-time>-live/`):
- `messages.jsonl`: every fix and sighting, sent or not (`sent`), with `t`
  (laptop time of the picture), `t_sent`, `proc_ms` (frame arrived -> sent);
- `clock.csv`: t_laptop,t_pi,offset_s,capture_delay_s once a second (offset_s =
  laptop - Pi clock: the smallest arrival - t_wall over the last 30 s);
- `live_summary.json`: frames, frames_missing (skipped by the Pi), processing_ms
  (median, p95, max per frame), fix_age_ms per path (picture taken -> sent,
  including the assumed capture delay), robots {name: fixes, fast, late,
  span_s, fixes_per_s, id_p_min}, tracker (counts: fixes, fast, late,
  sightings, contested, unresolved, fast_unconfirmed = fast fixes whose flash
  the full check never confirmed, sep_reject, flips, merged).
tools/live/eval_live.py adds `eval.json` (scores against simulator truth).

Files run_plan.py writes in its session folder, besides the console's CSVs:
`coordinator.jsonl` (one JSON object per line: t (laptop time), event =
step_sent, step_resent, step_report, step_failed, control, control_resent,
replan (t_run, source, cpu_s, budget_s, events, new_makespan_s, stranded,
obstacle, states, old_plan, and kept for an improvement try), waiting (why a
robot's next step is not sent), lost_task, runend, message), `plan_given.json`,
`plan_0.json`, `plan_1.json`, ... (the plan in force after each re-plan), and
`run_plan_summary.json` (result, duration_s, planned_makespan_s, resends,
steps_sent, fixes_sent, fixes_used_rejected, replans, dockings_reported,
boards_not_visited, sensor_obstacles; with --dry-run also `simulator`: the
simulator's sim_report.json). With --dry-run the simulator's own files are in
`sim/` (tools/sim/README.md).

## Home dock and touch probe (firmware 2026-10-19a+: dock.h, switches.h; hardware/dock)

Two lever switches on a printed nose bumper (left: `sw_left_pin`, default D5;
right: `sw_right_pin`, D6; INPUT_PULLUP, pressed = LOW) tell the robot when it
touches something. OFF until `set sw_on 1`; with it off the `dock` and `touch`
runs fail at once ("nose switches not configured") and RAW swL/swR read -1.
The bumper's numbers are settings (PARAM records): `bump_ahead_cm` (wheel axle
to the bumper face), `bump_half_cm` (each lever's contact, left/right of the
centre line), `sw_gap_cm` (face still this far from what it touches when a
lever clicks). Times are robot ms; distances wheel travel (library odometry)
from where the creep started.

`dock.csv` (one row per docking):

| Column | Meaning |
| --- | --- |
| run | run id |
| ms_start | the creep started (after learning the line sensors' floor level standing still, ~150 ms) |
| ms_first | the first switch clicked (debounced) |
| ms_left, ms_right | each switch's FIRST raw closing (before debouncing; 0 = never) |
| ms_both | both clicked: the robot is flush (the time the fusion applies the pose fix at) |
| first | L or R: which clicked first (the robot then pivoted the other side in) |
| result | `docked`, or why not: `no click within <n> cm`, `<side> switch never clicked`, `lost contact after seating ...`, `nose switches not configured (sw_on 0)`, `<side> switch pressed before the start`, `creep timed out (stalled?)` |
| travel_cm | wheel travel at the end (wheels slip while the funnel turns the robot, so this can exceed the geometric distance) |
| travel_left_cm, travel_right_cm | wheel travel at each switch's first closing (-1 = never) |
| pivot_deg | gyro heading change from the first click to both (0 without a compass check) |
| stripe_l_cm, stripe_c_cm, stripe_r_cm | wheel travel when each line sensor first read the dock's floor stripe (2 readings in a row stripe_thr past its level in stripe_dir; -1 = not seen). The dock fixes the distance from the stripe's far edge to the docked axle: (travel_cm - stripe_c_cm) should be the same every docking (dockfix.stripe_check: a wheel-size / stripe check) |
| frame | `floor` (follow: the own pose in the map frame) or `odo` (a `dock` run: library odometry, its own start frame) |
| x0_cm, y0_cm, th0_deg | own pose at the creep's start |
| x_cm, y_cm, th_deg | own pose when docked, BEFORE a follow step K resets it to the dock's |
| still_ms | how long it stood still docked (dock_still_ms) |
| drift_dps | the gyro heading's drift over that time, deg/s (left positive; 0 without a compass check) |
| gz_mean | mean raw gyro z over that time (motor board units) |
| drift_used | 1 = a follow step took the drift off its own heading (|drift| <= dock_drift_max_dps and a compass check) |
| n | docking number within a `dock` run (1..dock_n), or the follow step id |

`touch.csv` (one row per touch):

| Column | Meaning |
| --- | --- |
| run, ms_start | run id; the creep started |
| ms_click | the clicking switch's first raw closing |
| side | L, R, or LR (both pressed when it stopped) |
| result | `touched`, or why not (as for DOCK) |
| travel_cm | wheel travel at the click (from where the creep started) |
| travel_left_cm, travel_right_cm | per switch (-1 = never) |
| frame, x0_cm, y0_cm, th0_deg, x_cm, y_cm, th_deg | own pose at the start / after braking (as DOCK) |
| front0_cm | front distance sensor (centre zone) at the start: with travel_cm it is a distance-sensor check (reading = travel + bump_ahead + sw_gap - tof_ahead_cm) |
| n | touch number within a `touch` run (1..touch_n), or the follow step id |

What the touched face's position is: the lever's point, `bump_ahead_cm` ahead
of the axle and `bump_half_cm` to that side, plus `sw_gap_cm` along the
heading. The fusion (tools/fusion/dockfix.py) uses DOCK as a full pose fix
(dock_poses from the map) and TOUCH as a range to the nearest mapped face; the
site setup (`setup.py --touch`) ties each TOUCH to the still station it started
from.

