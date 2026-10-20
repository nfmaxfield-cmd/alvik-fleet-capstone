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
| Q | Investigate the object or spot under the mouse (see 7) |
| Z | Undo (also Ctrl+Z) |
| Esc | Cancel / deselect |

The same list is in the side panel of the studio.

A dashed arrow is only a heading guess (the object turning, or maybe reversing): it is saved as
unknown unless you set it with H, a drag or F.

## 6. Unsure

When you can't tell what an object is (an object or a stain? which one is it?), select it and
press **?**. It is drawn as a dashed grey circle with a question mark and saved as `unsure`.
A frame holding an unsure object is kept out of training and scoring until you decide, so a
guess never sneaks into the model. Never force a guess: unsure is always a fine answer.

**The Unsure tab** (top bar) lists every frame holding an unsure object, so you can come back
to them in one go, for example after **Investigate** (7) has told you what the spot is. To
decide, select the object and press a number key (gives it that type), **Del** (it isn't an
object), or **?** again (take the unsure mark back). A frame leaves the tab as soon as nothing
on it is unsure.

## 7. Investigate

For the hard question "is this an object, or part of the background (a stain, a mark, a
reflection)?". Point at an object or a spot on the picture and press **Q** (or select an
object and press **Investigate a spot** on the right). The studio looks at that spot (a
circle about one object wide) across the whole recording; a progress bar shows how far it is
(**Stop** or Esc ends it). Looking changes nothing. Then it shows:

- **What it found**, in plain words:
  - *Looks the same through the whole recording*: most likely part of the background, or
    something that never moved. Button **Ignore this spot** adds the circle to the camera's
    ignore spots (8): guesses inside it are left out from now on. Your labels are kept.
  - *Looks as it usually does on this frame*, with the moments something different was there
    (objects passing over).
  - *Something is here that moves*: the guesses followed it on or off the spot. Buttons jump
    to where it came from and where it went.
  - *Something is here for a while and isn't there the rest of the time*: something put down
    and taken away, or (if only briefly) passing over, like a hand. Buttons jump to the
    moments it appeared and left.
- **This frame** and **how it usually looks** (the background: for every pixel, the middle
  value of all the pictures), side by side, with the circle drawn.
- **A line of how much the spot changed** over the recording. Above the dashed line counts as
  "something there" (shaded). Click the line to go to that moment.
- **The biggest changes** as short clips that play over and over.
- **The spot over time**: about 40 small pictures; an orange edge means it looks different from
  usual. Click one to go there.
- **Labels already made at this spot**, every frame with its type and status. Untick any you
  want to leave alone, then **Change all to this type**, **Reject all** or **Mark all unsure**.
  It is one step: **Z** takes the whole change back.
- **The second camera as a referee**, when the recording is part of a two-camera session and
  both cameras are calibrated (9): the other camera's picture at the same moment, with a circle
  where the object would be (its type's height above the floor; the floor spot if its height
  varies). Something standing up shows there; a flat mark doesn't. Without calibrations the
  studio says so and everything else works the same.

**Track timeline** (right side) shows every object the guesses followed as one lane across the
whole recording: a bar while it was followed, a grey tick where the following broke off, a red
mark where it jumped further than an object can move, a white dot where a frame has its label.
Click a moment to go there: a quick way to find where the following went wrong.

Going to a frame from Investigate shows the nearest picked frame (if that exact frame wasn't
picked for labelling); **Back to the last investigation** on the right returns to the results.
It works the same for every object type: nothing in it knows what your objects are.

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
  **Investigate** (7) adds one with its **Ignore this spot** button.
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

## 10. Time saved

The studio keeps a record of how long labelling takes, for your own planning and for a final
report. Press **Time saved** in the top bar:

- **Seconds per frame** a person spent, and the share of frames **accepted unchanged**, for each
  guess source (no model, model v1, v2, ...): this is how much each model saves you.
- **Changes per frame** (moves, adds, rejects, type changes before the frame was finished).
- Frames **accepted without looking** (Accept remaining high-confidence frames) and for having
  the **same picture** as a frame you accepted, and the time that saved (at the seconds per frame
  measured with no model).
- **Getting started:** how long the project setup took, and the first install.

How the seconds are counted: from when the frame is shown until you finish it (Enter or E),
with any pause longer than 20 seconds counted as 20 seconds, and at most 120 seconds per frame.
Frames you finished before this record existed are **estimated** from their save times (the
gap to the previous saved frame, at most 120 s) and counted separately, marked *estimated*;
which model was in use for them is only known before the first model was trained.

**Download every frame (CSV)** and **Download the summary (CSV)** give the numbers as tables
for a spreadsheet. The same summary is in the full report. From the command line:
`python tools/recognition/timesaved.py <project folder>` writes both tables into the project's
`stats` folder.

## 11. Share a project

**Share project** (top bar) saves the whole project as one file, `<name>.mxirl`: object types,
labels, the picked frames, every model and its scores, work areas, camera calibrations and the
time record. Recording videos are left out unless you tick the box (they are large; leave it
off when the other person already has the recordings). A progress bar shows how far it is, and
nothing in your project is changed. You can keep labelling meanwhile.

**Open shared project** opens such a file: give the file, a new (empty) folder for the
project, and, if you have them, the folder where your copy of the recordings is (they are
found by their folder names). It then says how many recordings were found and offers to switch
to the new project. The project you had open is not changed.

**Recording not found: locate it.** When a recording isn't where the project says (another
computer, a moved folder), picking it shows this screen instead of an error: paste the
recording's own folder (the one with `video.avi`), or a folder to search for every missing
recording. The labels are safe in the project the whole time; only the video is needed again.

From the command line:

```
python tools/recognition/share.py export <project folder> <file.mxirl> [--with-video]
python tools/recognition/share.py import <file.mxirl> <new folder> [--recordings <folder to search>]
```

## 12. Use the model without the studio

Once a model is trained, **`track-with-model.bat`** (next to `start-recognition.bat`) uses it on
its own: it follows the objects in a recording, a video file, a camera plugged into the
computer, or a network camera (for example the Pi's preview address), gives each object a
steady number, and saves where every object was, frame by frame, in a table (`.csv`) you can
open in Excel.

Double-click it and answer its plain questions: which project (Enter takes the usual one),
which model if there are several (Enter takes the one the studio uses, else the best scored),
and what to watch (1 a recording, 2 a video file, 3 a camera, 4 a network camera). For a live
camera a page opens in your web browser showing the picture, every object with a confidence
colour (green: sure; amber: fairly sure, new or briefly out of sight; red: check it by eye) and
the frames per second; **Stop** on that page ends it.

The table goes in the project's `tracks` folder. One row per object per frame: `frame`,
`t_wall` (the recorder's clock), `obj` (its steady number), `cls` (type), `x_px`, `y_px`,
`heading_deg` (for types with a front), `score`, then `x_cm`, `y_cm` (position on the floor,
only when the camera is calibrated, see 9) and `source` (`model`: found in that frame;
`track`: carried along from where it was last seen). It uses the camera's work area and ignore
spots, like the studio.

On Mac or Linux, or to skip the questions:

```
python3 tools/recognition/use_model.py <recording folder, video file, camera number or address> --project <project folder>
```

`--help` lists the rest (`--model v3`, `--out file.csv`, `--seconds 60`, `--every 2`, `--page`).

## 13. Starting over

- **One frame:** **Reset this frame to the guesses** (Z brings your changes back).
- **The guesses:** choose "No model" in **Guesses from**. Your labels stay; only the
  unconfirmed guesses change back to motion and tracking. Switch back to any model at any
  time: every version is kept.
- **A whole new project:** start with another project folder (section 1). The old one stays
  as it is.

## 14. Where your files are

```
<project folder>/
  project.json            name, object types, recordings (paths), models, work areas, choices
  labels/<recording>/     labels.csv, review.json, frames/ (the picked frames as pictures), cache/
  models/<vN>/            model.onnx, run/ (training), train.log, eval_*.csv
  models/curve.json       every model scored on the same held-back frames
  cameras/<name>.json     camera calibrations (two cameras)
  stats/<recording>.jsonl the time record: one line per finished frame (see 10)
  stats/install.json      how long the first install took
  tracks/                 tables written by track-with-model.bat (see 12)
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

**Moving a project to another computer:** use **Share project** (11), or copy the project folder
and the recordings, then

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
| `timesaved.py` | the time-saving stats of a project (tables per frame and per model) |
| `referee.py` | Investigate's second-camera view of a spot |
| `share.py` | a project as one `.mxirl` file (export, import, finding recordings again) |
| `use_model.py` | use a trained model without the studio (`track-with-model.bat`) |
| `cameras.py`, `crosscam.py` | camera models and label sharing between cameras (CROSSCAM.md) |

The labelling page is `tools/label/page.html` (with `tools/label/app.py`, `tracker.py`,
`motion.py`, `select_frames.py`, `workarea.py`, `investigate.py`, `timing.py`); the detector is
`tools/detector/`.

Tests:

```
python -m pytest tools/recognition/tests tools/label/tests
```

Add `RECOGNITION_TRAIN=1` (PowerShell: `$env:RECOGNITION_TRAIN=1`) to include the few-minute
training test.
