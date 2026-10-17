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
| runinfo.csv | run,experiment,fw_version,motor_fw,ms_start,battery,charging,wifi_dbm,route |
| param.csv | run,name,value |
| runend.csv | run,result,ms_end,duration_ms,battery,wifi_dbm,raw_rows_sent,packets_sent |
| sync.csv | run,tag,ms_on,ms_off (tag = start or end; light flash on/off, robot ms) |
| clock.csv | token,robot_ms,boot |
| straight.csv | run,robot,result,cmd_cm,rpm,odo_x_cm,odo_y_cm,odo_theta_deg,imu_dyaw_deg,total_ms,battery |
| demo_step.csv | run,robot,step,x_cm,y_cm,theta_deg,ms |
| demo_end.csv | run,robot,route,result,return_err_cm,heading_err_deg,total_ms,a_cm,b_cm,r_cm,battery |

Also in the folder:

- `clock_sync.csv`: robot,boot,token,t_send,t_recv,robot_ms,rtt_ms,offset_s
  (laptop asks, robot answers with its millis(); offset_s = (t_send+t_recv)/2 - robot_ms/1000)
- `session.json`: start, start_unix, laptop, python, repo_commit

Notes:
- `run` ids are `<boot>-<n>`: boot is 4 hex digits, random per power-on.
- `ms` is the robot's millis() (ms since its power-on).
- RAW `x_cm,y_cm,th_deg` are the robot's own wheel odometry, reset to 0 at the
  start of straight and demo runs. `step` counts completed route steps.
- RAW `lifted` (firmware 2026-10-17c+): 1 while the robot reports being picked up.
- RAW gx..gz: gyro, deg/s; ax..az: accelerometer, g (to be confirmed on
  hardware: az about 1 at rest). yaw_deg: the IMU's fused yaw.
- Wheel angles (wl_deg, wr_deg) are cumulative degrees; distance = angle/360 x
  2 pi r with wheel radius r (about 1.7 cm, to be measured).

## Camera recording (`~/recordings/<date-time>_<label>/`, written by tools/pi/camrec.py)

- `video.avi`: Motion-JPEG, 640x480, about 30 fps
- `frames.csv`: frame,t_wall,t_mono (t_wall = Pi clock, Unix seconds, when the frame arrived)
- `events.csv`: t_wall,frame,text (notes typed while recording)
- `meta.json`: device,width,height,fps_requested,fps_reported,host,start,label,note

## Tape measurement sheet (`docs/experiments/<exp>/data/measurements.csv`)

order,robot,level,commands,run_id,target_x_cm,target_y_cm,target_heading_deg,
xf0_cm,yf0_cm,xr0_cm,yr0_cm,xf_cm,yf_cm,xr_cm,yr_cm,battery_pct,excluded,note

- xf/yf = front mark, xr/yr = rear mark, `0` = before the run (start), no
  suffix = after (end). Position = midpoint, heading = rear -> front.
- target_* are relative to the start pose (ahead, left, turned left).

## Labels for the camera detector (`<recording>/labels/`)

- `labels.csv`: frame,t_wall,obj,cls,x_px,y_px,heading_deg,heading_known,source,status,score
  - `obj`: object id within the recording (track id), integer
  - `cls`: alvik | hand | unknown | machine
  - `x_px,y_px`: object centre in image pixels (origin top-left, x right, y down)
  - `heading_deg`: image-frame heading, 0 = image +x (right), counter-clockwise
    positive as seen on screen (i.e. towards image -y); blank if unknown
  - `heading_known`: 1 if heading is given
  - `source`: motion | track | model | user | log
  - `status`: pending | confirmed | corrected | rejected
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
