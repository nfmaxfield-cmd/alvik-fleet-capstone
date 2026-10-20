# Pi camera tools

## One-time setup (on the Pi, over SSH)

```
sudo apt install -y git python3-opencv
git clone https://github.com/nfmaxfield-cmd/alvik-fleet-capstone.git
```

To get updates later: `cd ~/alvik-fleet-capstone && git pull`

## Record

```
cd ~/alvik-fleet-capstone
python3 tools/pi/camrec.py --device 0 --fps 25 --label E1
```

- Preview on the laptop: http://st1maxfini.local:8000
- Live link: http://st1maxfini.local:8000/live sends every frame (grey JPEG,
  with its frame number and Pi time) to tools/live/track_live.py on the
  laptop, which turns the robots' blinking lights into positions for
  run_plan.py (tools/fleet/README.md, "Running with the live camera"). It
  uses the frame camrec.py already has (no second camera open) and costs
  nothing until the laptop connects; then about 1 ms of JPEG work per frame
  on this computer (measured on the laptop; a Pi is slower, see
  tools/live/README.md).
- `--no-record` previews only (for aiming the camera). `--device 2` picks
  another camera (see `v4l2-ctl --list-devices`).
- While recording, type a note and press Enter to mark a moment, e.g.
  `trial 5 run 3F2A-7 start`. Notes land in `events.csv` with the frame number.
- Ctrl+C stops and saves to `~/recordings/<date-time>_<label>/`.

## Two cameras

Both webcams plug into the Pi. One command records both at once into one
folder, with times on the same clock, so a frame from the overhead camera can
be matched to the side camera's frame taken at the same moment.

**1. Find the cameras' numbers.** In the SSH window to the Pi, type:

```
cd ~/alvik-fleet-capstone
```

then:

```
python3 tools/pi/camrec.py --list
```

and press Enter. After a few seconds you see one line per number, for example

    USE ->  0  Vitade AF                    sends pictures, 640x480, MJPG
            1  Vitade AF                    opens but sends no picture (the information-only half of a webcam: normal)
    USE ->  2  USB Camera                   sends pictures, 640x480, MJPG
            3  USB Camera                   opens but sends no picture ...

The two numbers marked `USE ->` are your cameras (each webcam shows up twice;
only one of its two numbers sends pictures). The last line suggests the
command to use. To tell which is which, unplug one and run `--list` again.

**2. Record both.** Put the overhead camera's number first:

```
python3 tools/pi/camrec.py --device 0,2 --names overhead,side --fps 25 --label E1
```

You should see, after about 5 seconds of setting up (both cameras at once):

- a `Camera: exposure ...` line for each, starting `cam0 (overhead):` and `cam1 (side):`
- `cam0 (overhead): Preview: http://st1maxfini.local:8000` and `cam1 (side): Preview: http://st1maxfini.local:8001`
- `recording 640x480 to /home/.../recordings/<date-time>_E1/cam0` (and `cam1`)
- `Type a note and press Enter to mark a moment. Ctrl+C stops.`

Open both preview addresses in the laptop's browser to check the pictures.
Notes you type (and notes from run_sheet.py) go into both cameras' events.csv.
Ctrl+C stops both and saves; the last line is `Session saved: ...`.

**What is saved:** `~/recordings/<date-time>_E1/` holds `cam0/` (overhead) and
`cam1/` (side), each exactly like a one-camera recording folder (video.avi,
frames.csv, events.csv, meta.json: open either one in the label tool or
check_session.py), and `session.json`, which says which camera is in which
folder, its device number, role and preview port. `t_mono` in both
frames.csv files counts from the same start on the Pi's clock: frames from
the two cameras with the same t_mono were received at the same moment.

**Live link for the second camera:** `track_live.py --stream st1maxfini.local:8001`
(the first camera stays on 8000, as before).

**Warnings start with `!`.** Watch for:

- `! cam1 (side): only 18.3 fps over the last 5 s (25 expected): frames are being lost`
  The USB bus or the Pi can't keep up. Move one camera to another USB port
  (try a few combinations; not through a USB hub), or lower `--fps`. Also printed at the end with how many frames were
  lost, and saved in meta.json (`frame_check`).
- `! ... the camera sends YUYV, not MJPG` The camera ignored the request for
  compressed video; two uncompressed 640x480 cameras don't fit on one USB bus.
- `Camera 2 opened but sent no picture` with two cameras: usually the USB bus
  is full. Try another port, as above.
- `! LOW DISK SPACE` before starting: the card has less than about 2 GB per
  camera per 10 minutes. Copy old recordings to the laptop and delete them
  from the Pi first. While recording, one more warning comes at 1 GB free,
  and at 0.3 GB the recording stops and saves by itself (a completely full
  card leaves a broken video.avi).

`--pixfmt YUYV` asks for uncompressed video instead (one camera only, if MJPG
misbehaves). `--exposure`, `--focus` and `--wb` apply to both cameras; by
default each camera locks its own automatic values.

## Exposure, focus and white balance (locked by default)

The webcam's automatic exposure, white balance and autofocus change the
picture's brightness and sharpness mid-recording, which confuses the flash
and blink detection (they look for brightness changes) and the detector. So
camrec.py, after a 2 s warm-up, reads the automatic values the camera settled
on and freezes them. It prints what it set and what the camera reports back,
for example

    Camera: exposure 80.0 (auto mode 1.0), focus 40.0 (autofocus 0.0), white balance 4600.0 K (auto 0.0), gain 0.0; brightness 118.0 -> 117.5, sharpness 310.2 -> 305.9

and saves the details in meta.json (`camera_controls`, and `camera_controls_end`
when it stops, to show nothing drifted). Brightness before and after should
match; if it doesn't, or a line starts with `!`, set the values by hand:

```
python3 camrec.py --exposure 120 --focus 0 --wb 4600   # manual values
python3 camrec.py --exposure auto                        # automatic exposure, focus/WB still locked
python3 camrec.py --no-lock                              # leave everything automatic (old behaviour)
```

Light the floor the same way for the whole session: a locked exposure won't
follow a lamp being switched on.

Frame rate: an exposure longer than one frame (10000 / fps in 100 us units,
333 at 30 fps) makes the webcam slow down. A locked or brightness-matched
exposure is capped there (the picture may then be darker: add light), and
`exposure_dynamic_framerate` is set to 0 with v4l2-ctl if the camera has it.
After setting the exposure camrec.py counts frames for 2 s; under 90% of the
requested rate it prints a FRAME RATE TOO LOW warning between rows of `!` and
records it in meta.json (`fps_check`). A manual `--exposure` above the cap is
kept as given, with that warning.

The Vitade AF's exact control names and ranges are not known until tried on
the Pi. List them (needs `sudo apt install -y v4l-utils`; camrec.py works
without it but uses it as a fallback and logs its list in meta.json):

```
v4l2-ctl -d /dev/video0 --list-ctrls
```

Look for `auto_exposure` (or `exposure_auto`: 1 = manual, 3 = auto),
`exposure_time_absolute` (or `exposure_absolute`, usually in 100 us units: keep
it under ~330 for 30 fps), `focus_automatic_continuous` (or `focus_auto`),
`focus_absolute`, `white_balance_automatic` and `white_balance_temperature`.
To try a value by hand while previewing: `v4l2-ctl -d /dev/video0 -c focus_absolute=30`.
If a control is missing from the list, the camera can't set it and camrec.py
says so and carries on.

## Floor calibration

On the laptop, after copying the recording: `python tools/calibrate_floor.py
<recording folder> --height-cm <lens height> --nadir <x>,<y>` (see its --help
and docs/experiments/E00-measurement-check). After moving the camera, survey
mode recalibrates from logged runs without clicking:
`python tools/calibrate_floor.py <recording> --survey --session <logs folder> --sheet <sheet> --detections <detections.csv> --height-cm <h>`.

## Copy recordings to the laptop (in Windows PowerShell)

```
scp -r st1maxfini@st1maxfini.local:~/recordings $HOME\Desktop\
```

Put the folder for each experiment run under its record's `data/video/`
(large video files are git-ignored; keep them on the laptop or a drive).
