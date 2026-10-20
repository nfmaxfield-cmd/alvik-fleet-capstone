# Recognition studio (tools/recognition)

Label camera recordings, train an object detector on them, and watch it improve, in one
browser window. Built for the Alvik robots, but object types are yours to define, so it
works for any objects a fixed camera can see (3D-printed parts on a conveyor, for example).

## Start

On Windows, double-click `start-recognition.bat` in the project folder. Or, in a terminal
in the project folder:

```
python tools/recognition/start.py
```

The first start installs what it needs (a few minutes: PyTorch is large). The studio opens
in your browser. Your work goes in the project folder `Documents\recognition-project`; give
another folder to keep separate projects:

```
python tools/recognition/start.py D:\projects\conveyor-parts
```

## The loop

1. **Add recording** (top bar): paste a recording folder from the Pi, or a two-camera
   session folder (both cameras are added).
2. **Label.** The computer guesses every object; press **Enter** when the guesses on a frame
   are right, or drag, add (N) and reject (Del) until they are. The list **Most useful next**
   shows the frames the guesses are least sure about first. Every fix is passed forward, so
   the frames after it get better guesses too.
3. **Train new model.** Trains on every confirmed frame, in the background, while you keep
   labelling.
4. **Guesses from:** pick the new model. Its guesses replace the motion guesses on every frame
   nobody has confirmed yet, so the next frames need fewer fixes. Repeat from 2.
5. **Models & learning curve** shows how much better each model is, scored on frames it never
   trained on. **Open the full report** gives the same as a page to keep or send.

**Starting from scratch:** choose "No model" in Guesses from. Your labels stay; only the
unconfirmed guesses change back to motion and tracking. Switch back to any model at any time:
every version is kept.

**Object types:** name, whether it has a front (then the model learns its heading), its
size in pixels (blank = measured from the video), and how high its labelled point sits above
the floor (for sharing labels between cameras). Keys 1-9 pick types in this order.

## Two cameras

Record both cameras at once on the Pi (`tools/pi/README.md`, "Two cameras"), add the session
folder, and calibrate each camera to the floor once (CROSSCAM.md). A label confirmed in one
camera then appears as a ready-made guess (source `camera`) in the other camera's frame from
the same moment.

## How the scores stay honest

- Frames are split by **time blocks** (30 s), never frame by frame: neighbouring frames are
  near-copies. A block goes wholly to training, to picking the best training round, or to
  scoring, and frames within 1 s of a block edge are left out.
- A block's part depends only on its time, so new labels never move a scoring block into
  training, and both cameras' views of one moment land in the same part.
- Only frames a person checked are scored.
- "No model" is scored on the same frames, from what the page shows before any training.

## Files

| File | What it is |
| --- | --- |
| `start.py` | installs what is missing, then starts the studio |
| `studio.py` | the studio: web server, switching recordings and models, the routes the page uses |
| `project.py` | the project folder: object types, recordings, models (`project.json`) |
| `trainer.py` | background training: the time-block split, training, export, scoring every model |
| `report.py` | the accuracy report (`/report`, or `python tools/recognition/report.py <project>`) |
| `cameras.py`, `crosscam.py` | camera models and label sharing between cameras (CROSSCAM.md) |

The labelling page itself is `tools/label/page.html`, the detector is `tools/detector/`.

A project folder:

```
project.json            object types, recordings (paths), models, current choices
labels/<recording>/     labels.csv, review.json, frames/, cache/
models/<vN>/            model.onnx, run/ (training), train.log, eval_*.csv
models/curve.json       every model scored on the same held-back frames
cameras/<name>.json     camera models (two-camera sharing)
```

Moving a project to another computer: copy the project folder and the recordings, then
`python tools/recognition/studio.py <project> --relink <folder the recordings are in now>`.

## Tests

```
python -m pytest tools/recognition/tests
```

Add `RECOGNITION_TRAIN=1` (PowerShell: `$env:RECOGNITION_TRAIN=1`) to include the few-minute
training test.
