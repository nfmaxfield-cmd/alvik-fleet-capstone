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
- `--no-record` previews only (for aiming the camera). `--device 2` picks
  another camera (see `v4l2-ctl --list-devices`).
- While recording, type a note and press Enter to mark a moment, e.g.
  `trial 5 run 3F2A-7 start`. Notes land in `events.csv` with the frame number.
- Ctrl+C stops and saves to `~/recordings/<date-time>_<label>/`.

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
