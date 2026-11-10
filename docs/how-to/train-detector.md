# Train the camera detector on real school footage (labels made automatically)

Build e13 (10 Nov 2026). Where a roof tag was read, the robot's centre and heading are known, so every run that saved
its pictures is already a labelled recording. `tools/detector/autolabel.py` turns run folders into the detector's
labels folders (one per run and camera); `train.py` splits them **by run** (train / validation / test) and trains our
own model (CentreNet); `autolabel.evaluate_stops` scores any detector against the held-out runs' stops. The same
labelled footage can go to Dr. Sodhi's joint evaluation with the Machine Vision team (same frames, compare detectors);
nobody has been contacted about it.

## What a run folder needs

- its pictures: `frames_where.txt` (the USB stick's folder) or `<run>/frames/<camera>/track/*.avi` with their `.csv`
  (every Mission Control and `./run-fleet-pi.sh visit` run saves them unless "no frames" was chosen);
- `bridge/tag_poses.csv` (the tag bridge's reads), `run/camera_track.csv`, `tagcal/<camera>.json` (the camera model the
  run used), and for the scoring `run/stops.csv` (Visit points runs).

## The one command, when the footage folder arrives

On the **laptop** (it needs PyTorch; the Pi does not), in a **PowerShell** window, in the project folder
(`cd alvik-fleet-capstone`), with the runs copied to `D:\school-runs` (each run its own folder; every run with
`bridge/tag_poses.csv` below it is found):

```
python tools/detector/autolabel.py D:\school-runs --out D:\detector-e13
```

You should see one line per run and camera, "... pictures, N labelled, M left out", and at the end
"Labels for K recording(s) in D:\detector-e13\labels. Train: ...". Then train (the split by run is automatic; at least
3 runs are needed):

```
python tools/detector/train.py --labels D:\detector-e13\labels --out D:\detector-e13\model --max-minutes 30
```

Then export it for the Pi (one file, `best.onnx`):

```
python tools/detector/export.py D:\detector-e13\model\best.pt
```

## Scoring against the held-out stops

`D:\detector-e13\model\split.json` names the test runs. For each, in Python:
`autolabel.evaluate_stops(<run folder>, <its labels folder>, infer.load, infer.predict, "<best.onnx>")` gives the
detection rate, false detections, centre error in cm (median, 95 %), heading error and ms per frame on this machine.

## Using it in a run (off until you switch it on)

Copy `best.onnx` to the Pi as `~/.config/mirlt/detector/best.onnx`, then start runs with `MIRLT_ML_DETECTOR=auto`.
The tag bridge then runs it on every third overhead picture and sends its sightings as one more source (named by the
tracking layer, as the motion detector's are); `sources.csv` in the run folder shows each second which sources were
used. Without the file, or with the switch unset, nothing changes.

## What was proven here, and what was not

- Proven on simulated runs (`autolabel.py --sim 3`): labels sit 0.7 px (median) from the tag the camera read; frames
  with a robot in view but no confident read are left out (about 40 % of them, the robots driving); the stand-in
  detector (the motion detector) found every robot at the stops, centre error 0.5-0.6 cm median.
- **Not done here: training.** This machine cannot install PyTorch (its download is blocked). The model's speed on the
  Pi is an estimate: 5.3 ms a frame (320x240, one thread) on the 2-core test machine (README table); a Pi 4 is
  roughly 2-4x slower per core, so about 10-25 ms (Claude's reading, not measured; `infer.py --camera 0 --seconds 20`
  on the Pi measures it).
