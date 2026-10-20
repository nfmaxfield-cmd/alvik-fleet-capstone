# Maxfield IRL Tracking

*Identify. Recognize. Label.*

Maxfield IRL Tracking teaches a computer to find the objects a fixed camera sees: robots on a
table, parts on a conveyor, tools on a bench. You label recordings in one browser window, the
computer learns from your labels, and its guesses get better until most frames need no fixing.
Everything runs on your own computer; nothing is sent anywhere.

The steps below assume no programming experience.

## 1. Install (once)

1. **Python.** Install Python 3.9 or newer from [python.org](https://www.python.org/downloads/).
   On Windows, tick **"Add python.exe to PATH"** on the first screen of the installer.
2. **This folder.** Get the project folder (the one holding `start-recognition.bat` and
   `tools`) onto your computer: unzip it, or `git clone` it.
3. **Start it.** On Windows, double-click **`start-recognition.bat`** in that folder.
   A black window titled *Maxfield IRL Tracking* opens. On Mac or Linux, open a terminal in
   that folder and type:

   ```
   python3 tools/recognition/start.py
   ```

The first start installs what it needs (numpy, OpenCV, ONNX Runtime and PyTorch; a few
minutes, PyTorch is the large one). You see each step in the black window. When it is done,
the studio opens in your web browser. Later starts skip the install and open in seconds.

Keep the black window open while you work: it is the studio. Closing it (or Ctrl+C in it)
stops the studio; your work is already saved.

Your work goes in the project folder **`Documents\recognition-project`**. To keep separate
projects, give a folder name after the command (or drag a folder onto the `.bat` file):

```
python tools/recognition/start.py D:\projects\conveyor-parts
```

## 2. Recordings

A recording is a folder holding `video.avi`, `frames.csv` (one row per frame: `frame`,
`t_mono`, `t_wall`) and `meta.json` (`width`, `height`, `fps_reported`). The camera recorder in
`tools/pi` writes exactly this; `tools/pi/README.md` explains how to record. A two-camera
session is a folder with `session.json` and one recording folder per camera.

## 3. First start: set up the project

A new project asks three things before you judge any frame:

- **Project name.**
- **What you track.** Either a ready-made set, or your own list. The ready-made
  "Alvik robots" set is: alvik (has a front, marked point 4 cm above the floor), person
  (hands, arms, sometimes feet; height varies) and unknown. For anything else, pick
  **My own list** and name your objects, for example `cube`, `cylinder`, `gripper`.
- **For each object type:** its key (1-9) and colour (set by its place in the list),
  whether it **has a front** (then the model learns which way it faces), its **size** in
  pixels (leave blank: it is measured from the video), the **height of the marked point
  above the floor** (only used to pass labels to a second camera; tick **it varies** when
  it isn't fixed, like a hand or a lifted part) and a **note** saying where to mark it.

`unknown` is always kept, last: use it for anything that is none of the others, or that you
can't name yet. All of this can be changed later with **Object types**; a type already used
in labels can't be renamed or removed.

After the setup a short **tour** points at every part of the screen. It says which buttons
are safe to try and which change saved work. The **?** button at the top right shows it
again any time.

## 4. The loop

1. **Add recording** (top bar): paste a recording folder, or a two-camera session folder
   (both cameras are added). On Windows: open the folder in File Explorer, click the address
   bar, copy, and paste. The first time, the studio finds the moving objects in the whole
   video; a progress bar shows how far it is and about how long is left.
2. **Mark the work area** when it asks (see 8), or skip it.
3. **Label.** The computer guesses every object. Press **Enter** when the guesses on a frame
   are right, or drag, add (**N**) and reject (**Del**) until they are. **Most useful next**
   shows the frames the guesses are least sure about first. Every fix is passed forward, so
   the frames after it get better guesses too.
4. **Train new model.** Trains on every confirmed frame, in the background, while you keep
   labelling.
5. **Guesses from:** pick the new model. Its guesses replace the motion guesses on every frame
   nobody has confirmed yet, so the next frames need fewer fixes. Repeat from 3.
6. **Models & learning curve** shows how much better each model is, scored on frames it never
   trained on. **Open the full report** gives the same as a page to keep or send.

**When the guesses are good:** use **Spot check** to check a random sample of the
high-confidence frames, then **Accept remaining high-confidence frames** to accept the rest
without looking. **Undo auto-accept** takes them back.

**Same picture:** when you accept a frame, the picked frames within 10 seconds that show the
same picture (and whose guesses sit where your accepted objects are) are accepted with it. A
message says how many; **Z** takes them back together with your accept.

## 5. Keys

| Key | What it does |
| --- | --- |
| Enter | Accept this frame, go to the next one to do |
| click, drag | Select an object; drag to move it |
| H, then click | Set the heading (or drag the arrow tip, or Shift+drag) |
| F | Flip the heading 180° |
| U | Facing unknown (clears the arrow) |
| ? | Unsure what the selected object is (again: take it back) |
| Del / Backspace | Reject the selected guess (again: bring it back) |
| N | New object at the mouse (or double-click) |
| 1-9 | Object type of the selected object (and of new ones) |
| E | Frame is empty, go next |
| ← → | Previous / next frame (Shift: 10 at a time) |
| Tab | Select the next object |
| I | Change the selected object's number |
| G | Go to a frame number |
| M | Magnifier: a 3x lens around the mouse (again: off) |
| B | Picture: normal / brighter / high contrast (only how it is shown) |
| V | Full screen (also the button; Esc leaves it) |
| W | Work area (see 8) |
| Z | Undo (also Ctrl+Z) |
| Esc | Cancel / deselect |

The same list is in the side panel of the studio.

A dashed arrow is only a heading guess (the object turning, or maybe reversing): it is saved as
unknown unless you set it with H, a drag or F.

## 6. Unsure

When you can't tell what an object is (an object or a stain? which one is it?), select it and
press **?**. It is drawn as a dashed grey circle with a question mark and saved as `unsure`.
A frame holding an unsure object is kept out of training and scoring until you decide, so a
guess never sneaks into the model. The **Unsure** tab lists these frames. To decide: press a
number key (gives it that type), **Del** (it isn't an object), or **?** again (take it back).

## 7. Investigate

*(Coming in the next build: look at one spot across the whole recording, and fix every label
made there in one go. Key Q.)*

## 8. Work area

Each camera can have a work area: the part of the picture where the objects are (your table,
board or conveyor). Guesses outside it are left out, so the room around it no longer gets in
the way. The first time a camera is opened, the studio offers to mark it; **Work area** (or
**W**) changes it any time.

- **Corners:** click the corners of the area on the picture, then **Enter** (or Save).
  Backspace removes the last corner, Esc cancels.
- **Margin:** a band around the area where guesses are still kept, for things reaching in
  (a hand). Blank means one object size.
- **Ignore spots:** switch to *Ignore spots* and click a spot to ignore (a reflection, a
  blinking light); drag to set its size. Click it again to remove it.
- **Use the whole picture** removes the area (ignore spots stay).

Changing the work area keeps every label; only the guesses on frames nobody has confirmed
change. The slow first pass over the video is not repeated.

## 9. Two cameras

Record both cameras at once (`tools/pi/README.md`, "Two cameras"), add the session folder, and
calibrate each camera to the floor once (`CROSSCAM.md`). A label confirmed in one camera then
appears as a ready-made guess (source `camera`) in the other camera's frame from the same
moment. Types whose height varies are not passed between cameras: their spot in the other
picture can't be worked out, so you place them yourself.

While you label the first camera, the studio finds the moving objects in the other camera in
the background (its progress shows in the top bar), so switching cameras is quick.

## 10. Starting over

- **One frame:** **Reset this frame to the guesses** (Z brings your changes back).
- **The guesses:** choose "No model" in **Guesses from**. Your labels stay; only the
  unconfirmed guesses change back to motion and tracking. Switch back to any model at any
  time: every version is kept.
- **A whole new project:** start with another project folder (section 1). The old one stays
  as it is.

## 11. Where your files are

```
<project folder>/
  project.json            name, object types, recordings (paths), models, work areas, choices
  labels/<recording>/     labels.csv, review.json, frames/ (the picked frames as pictures), cache/
  models/<vN>/            model.onnx, run/ (training), train.log, eval_*.csv
  models/curve.json       every model scored on the same held-back frames
  cameras/<name>.json     camera calibrations (two cameras)
  report.html             the accuracy report, when written with report.py
```

- `labels.csv` has one row per object per picked frame: frame, time, object number, type,
  position (px), heading, and a status: `confirmed` / `corrected` (a person checked it),
  `auto` (accepted without looking), `unsure`, `rejected`, or `pending` (a guess nobody has
  checked). A frame marked empty has one row with type `none`.
- `review.json` says how each frame was finished (by a person, by the spot check, auto, or
  for having the same picture as a frame a person accepted).
- `project.json` keeps each camera's work area under `cameras`: `work_area` (corner points in
  pixels), `margin_px`, `ignore_spots` (x, y, radius in pixels).
- The recordings themselves stay where they are (they are large).

**Moving a project to another computer:** copy the project folder and the recordings, then

```
python tools/recognition/studio.py <project folder> --relink <folder the recordings are in now>
```

## How the scores stay honest

- Frames are split by **time blocks** (30 s), never frame by frame: neighbouring frames are
  near-copies. A block goes wholly to training, to picking the best training round, or to
  scoring, and frames within 1 s of a block edge are left out.
- A block's part depends only on its time, so new labels never move a scoring block into
  training, and both cameras' views of one moment land in the same part.
- Only frames a person checked are scored; frames holding an unsure object are left out.
- "No model" is scored on the same frames, from what the page shows before any training.

## For developers

| File | What it is |
| --- | --- |
| `start.py` | installs what is missing, then starts the studio |
| `studio.py` | the studio: web server, switching recordings and models, setup, work areas, the routes the page uses |
| `project.py` | the project folder: object types, presets, recordings, models, work areas (`project.json`) |
| `trainer.py` | background training: the time-block split, training, export, scoring every model |
| `report.py` | the accuracy report (`/report`, or `python tools/recognition/report.py <project>`) |
| `cameras.py`, `crosscam.py` | camera models and label sharing between cameras (CROSSCAM.md) |

The labelling page is `tools/label/page.html` (with `tools/label/app.py`, `tracker.py`,
`motion.py`, `select_frames.py`, `workarea.py`); the detector is `tools/detector/`.

Tests:

```
python -m pytest tools/recognition/tests tools/label/tests
```

Add `RECOGNITION_TRAIN=1` (PowerShell: `$env:RECOGNITION_TRAIN=1`) to include the few-minute
training test.
