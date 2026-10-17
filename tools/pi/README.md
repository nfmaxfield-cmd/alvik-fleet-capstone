# Pi camera tools

## One-time setup (on the Pi, over SSH)

```
sudo apt install -y git python3-opencv
git clone https://github.com/nfmaxfield-cmd/alvik-fleet-capstone.git
```

To get updates later: `cd ~/alvik-fleet-capstone && git pull`

## Record

```
cd ~/alvik-fleet-capstone/tools/pi
python3 camrec.py --label E1b-straight
```

- Preview on the laptop: http://st1maxfini.local:8000
- `--no-record` previews only (for aiming the camera). `--device 2` picks
  another camera (see `v4l2-ctl --list-devices`).
- While recording, type a note and press Enter to mark a moment, e.g.
  `trial 5 run 3F2A-7 start`. Notes land in `events.csv` with the frame number.
- Ctrl+C stops and saves to `~/recordings/<date-time>_<label>/`.

## Copy recordings to the laptop (in Windows PowerShell)

```
scp -r st1maxfini@st1maxfini.local:~/recordings $HOME\Desktop\
```

Put the folder for each experiment run under its record's `data/video/`
(large video files are git-ignored; keep them on the laptop or a drive).
