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
3. **Start it.**
   - **Windows:** double-click **`start-recognition.bat`** in that folder. A black window titled
     *Maxfield IRL Tracking* opens. If it says *"Python not found: install it from python.org and
     tick 'Add python.exe to PATH'"*, do step 1 again with that box ticked, then double-click the
     file again. (It looks for Python as `py -3` first, then as `python`.)
   - **Mac or Linux:** open a terminal in that folder. Make a separate Python environment once
     (many Linux systems refuse to install packages into the system's own Python):

     ```
     python3 -m venv ~/mxirl-venv
     ```

     Then, every time you start the studio:

     ```
     source ~/mxirl-venv/bin/activate
     python tools/recognition/start.py
     ```

**The first start installs what the studio needs** (numpy, OpenCV, ONNX Runtime and PyTorch).
Plan for **about 10 to 15 minutes** and **over 1 GB of downloads** on Windows (PyTorch is most of
it). It needs the internet this once; afterwards everything works offline. You see each step in
the black window. When it is done, the studio opens in your web browser. Later starts skip the
install and open in seconds.

Keep the black window open while you work: it is the studio. Closing it (or Ctrl+C in it)
stops the studio; your work is already saved.

**If the browser does not open by itself,** open your browser and go to
**http://127.0.0.1:8765** (the black window shows this address too). **If the black window says
"Port 8765 is busy",** the studio is already running in another black window: use that one, or
close it and start again.

**Windows notes**

- *"Windows protected your PC"* (SmartScreen) can appear the first time you double-click the
  `.bat` file: click **More info**, then **Run anyway**. The file is plain text: right-click it and
  choose **Edit** to see the few lines it runs.
- **OneDrive:** if your Documents folder is synced by OneDrive, the project folder is synced too.
  That works, but projects grow large (pictures, models) and OneDrive can hold a file for a
  moment while it uploads (the studio waits and tries again). To keep a project out of OneDrive,
  give it a folder outside it (see below), for example `C:\mxirl\my-project`.

**Where your work goes.** Without anything else, the project folder is `recognition-project` in
your Documents folder:

| System | Project folder |
| --- | --- |
| Windows | `C:\Users\<you>\Documents\recognition-project` |
| Mac | `/Users/<you>/Documents/recognition-project` |
| Linux | `~/Documents/recognition-project` (or `~/recognition-project` when there is no Documents folder) |

To keep separate projects, give a folder after the command, or (Windows) drag a folder onto
`start-recognition.bat`. Windows example:

```
python tools/recognition/start.py D:\projects\conveyor-parts
```

Mac or Linux example:

```
python tools/recognition/start.py ~/projects/conveyor-parts
```

## 2. Recordings

A recording is a folder holding `video.avi`, `frames.csv` (one row per frame: `frame`,
`t_mono`, `t_wall`) and `meta.json` (`width`, `height`, `fps_reported`). The camera recorder in
`tools/pi` writes exactly this; `tools/pi/README.md` explains how to record. A two-camera
session is a folder with `session.json` and one recording folder per camera.

**Recordings of a few minutes or more work best.** To keep the scores honest, training cuts the
recordings into time pieces (30 seconds each when there is enough footage) and uses each piece
for one job only: learning, picking the best training round, or scoring. A recording of under
two minutes gives too few pieces: the studio then makes the pieces shorter (down to 3 seconds),
which works, but neighbouring pieces look alike, so the scores flatter the model a little.

### Making a recording without the Pi

Any phone or webcam app will do. **Add recording** accepts an ordinary video file (mp4, mov,
avi, mkv ...): the studio makes it into a recording folder, with a progress bar, and adds it.

1. **Keep the camera still.** Put the phone or webcam on a stand, or prop it up, looking at the
   table or area your objects are on. The studio finds objects by what moves against a still
   background, so a camera held in the hand doesn't work.
2. **Record a few minutes or more** (see above for why), with the objects moving around and
   sometimes standing still, the way they do when you use them.
3. **Spread your labels across the recording** later: frames from different minutes teach the
   model more than many neighbouring frames, and every time piece needs some (the studio's
   *Most useful next* list helps with this, see 4).
4. Copy the video file to the computer, press **Add recording** and paste where the file is.
   Windows: hold Shift, right-click the file in File Explorer, choose **Copy as path**, and paste.

The recording folder is made next to the video file and named after it (`table-test.mp4` gives
the folder `table-test`); when that place can't be written, it goes in the project folder's
`recordings` folder. The video file itself is not changed. Each frame keeps its time from the
video. Large videos (for example a phone's 1920 x 1080) are made smaller, at most 960 pixels
wide, so finding the moving objects and labelling stay quick; this uses about 1 MB of disk per
second of video. Adding the same video file again reuses the folder made the first time.

From the command line (any system):

```
python tools/recognition/videofile.py <video file>
```

## 3. First start: set up the project

A new project asks three things before you judge any frame:

- **Project name.**
- **What you track.** Either a ready-made set, or your own list. The ready-made
  "Alvik robots" set is: alvik (has a front, marked point 4 cm above the floor), person
  (hands, arms, sometimes feet; height varies) and unknown. For anything else, pick
  **My own list** and name your objects, for example `cube`, `cylinder`, `gripper`.
- **For each object type:** a **name** (every row needs one: remove an empty row with ×), its
  key (1-9) and colour (set by its place in the list), whether it **has a front** (then the model
  learns which way it faces), its **size** in pixels (leave blank: it is measured from the video)
  and a **note** saying where to mark it. Once the project has a second camera, a column for the
  **height of the marked point above the floor** appears: it is only used to pass labels between
  the two cameras. Tick **it varies** when it isn't fixed (a hand, a lifted part); leave it blank
  when you don't know it. Either way, that type is not passed between cameras.

**Describe what you track in a sentence** is a third choice: the optional AI helper (17) reads
your sentence and suggests the object types, marked AI. You look them over and change them before
saving; nothing is saved until you press the button. If the helper is off, the choice lets you
switch it on there (your company's AI service and its key).

**How each type moves** (the last three columns, Object types): the fastest it moves (object sizes
per second), whether it **can be carried** (picked up and put down elsewhere) and whether it **can
stay still** for a long time. Leave them at "usual" unless the guesses lose an object or jump to the
wrong one; "usual" is 3 sizes per second for objects, 15 for hands and things reaching in.

`unknown` is always kept, last: use it for anything that is none of the others, or that you
can't name yet. All of this can be changed later with **Object types** (top bar, under **More**); a type already used
in labels can't be renamed or removed.

**Two or more types of object and no model yet:** before a model is trained, the guesses come
from motion, which can tell "an object" from "something reaching in" but not which of your types
an object is. So with two or more types (say `cube` and `cylinder`), the guesses show every object
as `unknown`, and you give each its type with the number keys. The studio says so when you save
such a list. Once a model is trained on your labels, its guesses use your types.

After the setup a **short tour** (8 steps) points at the parts you need to start. Its last step
offers **Show the full tour**, which goes through every button and says which are safe to try
and which change saved work. The **?** button at the top right shows the tour again any time.

## 4. The loop

1. **Add recording** (top bar): paste a video file (see 2, "Making a recording without the
   Pi"), a recording folder, or a two-camera session folder (both cameras are added). On
   Windows, for a folder: open it in File Explorer, click the address bar, copy, and paste. The
   first time a recording opens, the studio measures how big the objects are and finds the moving
   objects in the whole video; a progress bar shows how far it is and about how long is left.
2. **Mark the work area** when it asks (see 8), or skip it.
3. **Label.** The computer guesses every object. Press **Enter** when the guesses on a frame
   are right, or drag, add (**N**) and reject (**Del**) until they are. Every action shows a short
   message at the bottom that fades by itself: what changed, where (the frame, the object), and
   the exact key or button that takes it back. **Z** undoes the last change, **Shift+Z** puts back
   what Z took back. **Most useful next** shows first the frames the tracker asks you to look at
   (for example two lookalikes crossed and it can't tell which is which: the reason is written
   above the picture, and the tab counts them as "to look at first"), then the frames the guesses
   are least sure about, and takes them from each part of
   the training split in turn (learning, picking the best round, scoring; see "How the scores
   stay honest"), so every part gets frames early. **Confirmed frames by part** on the right
   shows how many each part has in this recording. Every fix is passed forward, so the frames
   after it get better guesses too. The line above the picture says why a frame was picked
   (for example "objects close together", "something appears").
4. **Train new model.** Trains on every confirmed frame, in the background, while you keep
   labelling. Guesses you rejected are used as "not an object here" examples. **Copy-paste** (in the
   same window) pastes objects you labelled onto other learning frames, so the model sees more of
   them: *Off*, *On*, or *Compare*, which trains twice (with and without, half the time each) and
   keeps whichever does better on the frames used for picking the best round (only the kept one is
   then scored on the held-back frames, so that score stays honest); Learning curve then shows both
   scores and the verdict in words. The top bar says how far it is in plain words ("round 4 of about 30, about 6 min
   left"). It needs at least 10 confirmed frames for learning and 1 in each of the other two
   parts, and the two smaller parts need frames a person looked at (auto-accepted frames are only
   used for learning); if it hasn't got them, it says how many each part has and what to do, before
   anything starts. When it is done, a
   box says how well the new model does. If the model was saved but **finds nothing yet**, the
   box says so, with the numbers it knows: how many frames and minutes it learned from, and what a
   model of this project that does find objects needed (or that no model here has yet); such a model is hidden from the lists (see 6) and the next
   model does not start from it.
5. **Guesses from:** pick the new model. Its guesses replace the motion guesses on every frame
   nobody has confirmed yet, so the next frames need fewer fixes. Repeat from 3.
6. **Learning curve** shows how much better each model is, scored on frames it never
   trained on. The curve has one place per model version, oldest to newest, with the number of
   confirmed frames it trained on written under it. When the scores rest on few held-back frames
   (under 30) or few different objects (under 5), a **Caution** box says so: treat them as a
   rough sign until you have labelled more. **Open the full report** gives the same as a page to
   keep or send. Below the curve is the list of every model, with what each training used (how many
   rejected guesses, and the copy-paste verdict). **By situation** shows where the best model does
   worst: objects alone, close together, near a person, at the edge, standing still (a group with
   few labels is marked "too few to trust"). **How often should I check?** measures, on your own
   labels in this recording, how far the guesses drift the longer it has been since you last
   labelled an object, and says how often to check each one (**Measure it on my labels**; it runs in
   the background and changes nothing).

   **Failed models** (training stopped, or the model finds nothing yet) are hidden from this
   window and from **Guesses from**. Tick **Show failed models** to see them; each has a
   **Delete** button (it asks first; your labels are not touched). Share project leaves them
   out unless you ask (see 11).

**When the guesses are good:** use **Spot check** to check a random sample of the
high-confidence frames, then **Accept remaining high-confidence frames** to accept the rest
without looking. The spot check says how many of the checked frames needed no correction, and
the range the true share very likely lies in (from only 10 checked frames, that range is wide;
checking more narrows it). **Undo auto-accept** takes them back.

**The progress bar** (top): green, frames you reviewed; grey, frames holding an object marked
unsure (not finished until you decide); dark green, accepted without looking; orange, changed
but not finished.

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
| K | The list of keys, in a window |
| B | Picture: normal / brighter / high contrast (only how it is shown) |
| V | Full screen (also the button; Esc leaves it) |
| W | Work area (see 8) |
| C | Short clip: about 1 s before and after this frame, over and over (C again or Esc stops; changes nothing) |
| Q | Investigate the object or spot under the mouse (see 7); after jumping to a frame from an investigation, Q goes back to it (Shift+Q: investigate the spot under the mouse instead) |
| Z | Undo the last change (also Ctrl+Z) |
| Shift+Z | Redo: put back what Z took back (also Ctrl+Y) |
| Esc | Cancel / deselect |

The same list is in the side panel of the studio, and the **Keys** button (top right, or **K**)
shows it in a window.

A dashed arrow is only a heading guess (the object turning, or maybe reversing): it is saved as
unknown unless you set it with H, a drag or F.

**C** helps when a still picture can't tell you what is happening (a hand or an object, something
being put down, which way it moves): it plays the video from about a second before to a second
after the frame, at its own speed, over and over. The labels are drawn only on the frame itself.
Moving does not show which way something faces (it may be reversing or turning on the spot).

**Every action says what it did.** A short message at the bottom (it fades by itself) says what
changed, where, and the key or button that takes it back, for example *Undid: moved #71 to 212,
140 on frame 9310; it is back as it was. Shift+Z puts it back.* Z takes back changes to labels,
Investigate's changes, accepting frames without looking, work-area and home-position changes, and
using or stopping outside data (15); Shift+Z puts each back again (until you make a new change). A
few things can't be taken back, and their message says so (deleting a failed model, drawing a new
spot-check sample, forgetting the AI helper's saved key).

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
(**Stop** or Esc ends it). Looking changes nothing. The window has three parts, kept apart so
it is clear which buttons change anything:

**1. What we found** (read only: nothing here changes anything)
- **The verdict**, in plain words:
  - *Never changes in this recording*: either part of the background (a mark, a stain, a
    fixture) or an object that stayed put the whole time. The recording alone can't tell
    which: check it in a recording of the empty area, or with the second camera (below).
  - *Looks as it usually does on this frame*, with the moments something different was there
    (objects passing over).
  - *Something is here that moves*: the guesses followed it on or off the spot.
  - *Something is here for a while and isn't there the rest of the time*: something put down
    and taken away, or (if only briefly) passing over, like a hand.
- **This frame** and **how it usually looks** (the usual picture, made from the whole
  recording, so that anything passing through is left out), side by side, with the circle drawn.
- **Ask the AI: object or background?** (only when the AI helper is on, see 17): sends the spot
  now, how it usually looks and up to 4 moments it looked different, as small pictures. Its answer is
  marked AI; **That fits** / **I don't think so** only notes your answer. To act on it, use part 3.
- **The second camera as a referee**, when the recording is part of a two-camera session and
  both cameras are calibrated (9): the other camera's picture at the same moment, with a circle
  where the object would be (its type's height above the floor; the floor spot if its height
  varies). Something standing up shows there; a flat mark doesn't. Without calibrations the
  studio says so and everything else works the same.
- How many labels were already made at the spot, and on which frames.

**2. Look around** (these only take you to a frame; nothing is changed)
- Buttons to where it came from and where it went, or when it appeared and left.
- **A line of how much the spot changed** over the recording, from "usual" at the bottom to
  "differs" at the top. Above the dashed line counts as "something there" (shaded). Click the
  line to go to that moment.
- **The moments something different was here**, each with a Jump button.
- **The biggest changes** as short clips that play over and over.
- **The spot over time**: about 40 small pictures; an orange edge means it looks different from
  usual. Click one to go there.

Every jump says where it took you (*Jumped to frame 6150 (4:12). Investigation closed: press Q
to go back to it.*). **Q** then brings back the investigation and the frame you were on; after
that, or once you close the window yourself, Q investigates the spot under the mouse again
(**Shift+Q** always does). **Back to the last investigation** on the right opens it too.

**3. Change things** (an orange box: these change saved labels). Every button says how far it
reaches in its own words, shows what will change, and asks before it changes anything; **Z**
takes it back in one step.
- **Reject here** rejects the guess at the spot on this frame only ("Reject here: guess #7 on
  frame 812 only"), and the tracker remembers what the empty spot looks like: later guesses there
  that look like it are pushed down hard, while something that looks different (an object driving
  over the stain) is still found. No other frame is changed. An object you added yourself at the
  spot is kept (Del removes it). Use it on a stain or a reflection the guesses keep finding.
- **Labels already made at this spot**: every frame with its type and status (click a frame
  number to go there). Untick any you want to leave alone; the buttons then say exactly what
  they reach, for example **Reject all 12 labels at this spot (frames 812-4410)**, **Change 5
  labels to box (frames 812-990)** or **Mark 3 labels unsure (frames 812-830)**. A preview lists
  the changes (frame, object, type, status) with **Cancel** and **Yes, change it**.

Earlier versions had an **Ignore this spot** button that left a spot out of every frame. It is
gone: a rejection now applies to one frame and informs later guesses, so something real standing
on that spot later is still found. Ignore spots made with it keep working, and Work area (8) shows
them and removes them. To leave something out for good (a shelf light, a reflection off to the
side), draw the work area so it is outside.

**Track timeline** (right side) shows every object the guesses followed as one lane across the
whole recording: a bar while it was followed, a grey tick where the following broke off, a red
mark where it jumped further than an object can move, a white dot where a frame has its label.
Click a moment to go there: a quick way to find where the following went wrong.

Going to a frame from Investigate or the timeline shows the nearest picked frame (if that exact
frame wasn't picked for labelling), and the message says so. Investigate works the same for every
object type: nothing in it knows what your objects are.

## 8. Work area

Each camera can have a work area: the part of the picture where the objects are (your table,
board or conveyor). Guesses outside it are left out, so the room around it no longer gets in
the way. The first time a camera is opened, the studio offers to mark it; **Work area** (or
**W**) changes it any time. While you mark it, its buttons take the place of the side panel, so
the picture stays where it is and fully visible.

- **Corners:** click the corners of the area on the picture, then **Enter** (or Save).
  Backspace removes the last corner, Esc cancels.
- **Margin:** a band around the area where guesses are still kept, for things reaching in
  (a hand). Blank means one object size.
- **Old ignore spots:** ignore spots made by an earlier version (red circles) keep working:
  guesses inside them are left out. Switch to *Old ignore spots* and click one to remove it. New
  ignore spots are not made any more (see 7, "Reject here"); to leave something out for good,
  draw the area so it is outside.
- **Use the whole picture** removes the area (old ignore spots stay).
- **Home positions:** spots where an object's place and facing are known exactly (see 16).
- **Z** puts the previous work area back after you saved a change; **Shift+Z** puts yours back.

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
report. Press **More**, then **Time saved**, in the top bar:

- **Seconds per frame** a person spent, and the share of frames **accepted unchanged**, for each
  guess source (no model, model v1, v2, ...): this is how much each model saves you.
- **Changes per frame** (moves, adds, rejects, type changes before the frame was finished).
- Frames **accepted without looking** (Accept remaining high-confidence frames) and for having
  the **same picture** as a frame you accepted, and roughly the time that saved (at the seconds
  per frame measured with no model; these are the easier frames, so it is an upper estimate).
- **Getting started:** how long the project setup took, and the first install.

Models are compared only on **measured** frames. The summary table has one row per guess source
and kind of time; its columns are `model` (no model, v1, v2 ...), `guesses` (motion or model,
"+camera" when the second camera helped), `measured` (True: timed on the page; False: estimated,
see below), `frames`, `seconds_per_frame`, `median_seconds`, `accepted_unchanged`,
`edits_per_frame`, `unsure`, `auto`, `same`, `estimated` and `seconds`, followed by the totals.
The frames are shown hardest first: the guesses a person checks are the least sure ones, so the
seconds per frame are higher than an average frame would take.

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

**Share project** (top bar, under **More**) saves the whole project as one file, `<name>.mxirl`: object types,
labels, the picked frames, every working model and its scores, work areas, camera calibrations
and the time record. Recording videos are left out unless you tick the box (they are large;
leave it off when the other person already has the recordings). **Failed models** (and models
that find nothing yet) are left out too, unless you tick **Also put in the failed models**. A
progress bar shows how far it is, and nothing in your project is changed. You can keep
labelling meanwhile.

**Open shared project** (also under **More**) opens such a file: give the file, a new (empty) folder for the
project, and, if you have them, the folder where your copy of the recordings is (they are
found by their folder names). It then says how many recordings were found and offers to switch
to the new project. The project you had open is not changed.

**Recording not found: locate it.** When a recording isn't where the project says (another
computer, a moved folder), picking it shows this screen instead of an error: paste the
recording's own folder (the one with `video.avi`), or a folder to search for every missing
recording. The labels are safe in the project the whole time; only the video is needed again.
Once located, it opens without redoing the slow first pass over the video (the saved guesses are
kept as long as it is the same video: same size, number of frames and length).

From the command line:

```
python tools/recognition/share.py export <project folder> <file.mxirl> [--with-video] [--with-failed-models]
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

The table goes in the project's `tracks` folder, with a new name every run (what was watched,
the date and time, and the model, for example `table-test_20261020-143005_v3.csv`), so an earlier
table is never overwritten. While it runs, the rows are written as they come to a file ending in
`.partial.csv`; at the end that becomes the table. If it is stopped early or something goes
wrong, the rows so far are still in the `.partial.csv` file, and the window says so.

If the table ends up with **no rows**, the model found no objects it was sure enough about: the
window says so. Try a lower minimum score (`--min-score 0.2`), or label more frames and train the
model longer, then run it again. A model the studio marks as "finds nothing yet" will give no rows.

One row per object per frame: `frame`,
`t_wall` (the recorder's clock), `obj` (its steady number), `cls` (type), `x_px`, `y_px`,
`heading_deg` (for types with a front), `score`, then `x_cm`, `y_cm` (position on the floor,
only when the camera is calibrated, see 9) and `source` (`model`: found in that frame;
`track`: carried along from where it was last seen). It uses the camera's work area and ignore
spots, like the studio.

On Mac or Linux (in the environment from section 1), or to skip the questions:

```
python tools/recognition/use_model.py <recording folder, video file, camera number or address> --project <project folder>
```

`--help` lists the rest: `--model v3`, `--out file.csv`, `--every 2` (every 2nd frame only),
`--page` (the live page), `--max-frames 500` (stop after 500 frames: for part of a recording) and
`--seconds 60`. **`--seconds` counts clock time while it runs, not seconds of video:** a recording
is usually gone through faster or slower than it plays, so `--seconds 60` stops after one minute
of running, wherever it has got to. It is meant for live cameras.

**Roll call** (with the live page, on by default for a camera): at the start the page shows the
first steady picture with what the model sees, each object numbered. **Looks right** confirms it, or
fix it first (drag, delete, add, type and **name** each object). Named objects keep their names in
the table (columns `name` and `confirmed`). While it runs, each object shows green, amber or red and
when it was last confirmed; when one's score stays low for a few seconds the page asks you to confirm
just that one (the others carry on). `--outside <kind>:<folder or file>` (for example a folder of
robot logs, or `csv:` and a positions file) lets the outside data suggest each object's name at the
roll call; it needs the camera's floor calibration, and the window says when it can't be used.

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
  cameras/<name>.json     camera (floor) calibrations: two cameras, robot logs, home presets
  ai_settings.json        the AI helper's switch and service (no key); never shared
  ai_log.jsonl            every AI call and your answers; never shared
  stats/<recording>.jsonl the time record: one line per finished frame (see 10)
  stats/install.json      how long the first install took
  recordings/             recordings made from video files, when the video's own folder can't be written (see 2)
  tracks/                 tables written by track-with-model.bat (see 12)
  report.html             the accuracy report, when written with report.py
```

- `labels/<recording>/` may also hold `outside.json` and `outside_drafts.json` (robot logs, 15) and
  `checkins.json` (How often should I check?, 4).
- `labels.csv` has one row per object per picked frame: frame, time, object number, type,
  position (px), heading, and a status: `confirmed` / `corrected` (a person checked it),
  `auto` (accepted without looking), `unsure`, `rejected`, or `pending` (a guess nobody has
  checked). A frame marked empty has one row with type `none`.
- `review.json` says how each frame was finished (by a person, by the spot check, auto, or
  for having the same picture as a frame a person accepted), and lists under `rejected_spots`
  the spots where **Reject here** was used (frame, x, y, radius), which the tracker remembers.
- `project.json` keeps each camera's work area under `cameras`: `work_area` (corner points in
  pixels), `margin_px`, `ignore_spots` (x, y, radius in pixels; made by earlier versions).
- The recordings themselves stay where they are (they are large).

**Moving a project to another computer:** use **Share project** (11), or copy the project folder
and the recordings, then

```
python tools/recognition/studio.py <project folder> --relink <folder the recordings are in now>
```

## 15. Robot logs and other outside data

**More**, then **Robot logs / outside data**, for the recording that is open. Outside data is
anything that knows where the objects are, on the same clock as the recording: a robot's own log
(its wheels and gyro), a machine's controller, or a **positions file** (a CSV with t, name, x, y and
optionally heading; see docs/data-formats.md). Paste its folder or file, pick the kind (or let the
studio find out) and press **Use it**. The studio then:

- matches each object in the data to an object on video by how they moved (the **roll call**): the
  name shows next to the object's number on the picture and in the list on the right;
- lines the data up on the floor at every home visit (16), at every label you made of it, and on a
  stretch where it drove forward on video, correcting the robot's own drift each time;
- makes a guess for every frame (source `log`; the line above the picture says "and robot logs"),
  so an object the data knows is found even while it stands still;
- helps decide which lookalike is which after two of them crossed.

It needs the camera's **floor calibration** (which pixel is which spot on the floor). Without one the
window says so and what to do (`CROSSCAM.md`). **Line up again** uses the labels and home visits you
made since. **Stop using it** (or **Z**) takes the guesses away again; your labels are never changed.

## 16. Home positions

A home position is a spot where an object's place (and facing) is known exactly: a dock, a parking
spot, a tray. Press **Work area** (or **W**), then **Home positions**: **Add a home position**, click
its spot, then click where it faces (Enter: no facing), name it and say whether it stays a while or
is touch-and-go. A **preset** places a ready-made one from two clicks, for example a dock (the window says
where to click; it needs the floor calibration). **×** removes one; **Save home positions** keeps the list (**Z** puts the old one back).

An object sitting still at a home position is a free confirmation: its guesses snap to the exact
place and facing, and robot logs are lined up again there. The panel says how many visits it found.

## 17. AI helper

Optional and off unless you switch it on (**More**, then **AI helper**): an AI service your company
already uses looks at **small pictures** and suggests answers. Choose the service, paste its key (kept
on this computer only, never in the project or a shared file), tick **Use the AI helper**, and Save.
The key is kept in your own settings folder (`.maxfield-irl` in your home folder). On Mac and Linux
only your account can read that file; on Windows it is protected by your Windows user profile (other
ordinary accounts on the computer can't open it, but an administrator can). A web address for the
service must start with `https://` (`http://` only for `localhost`). A shared project never brings
its own AI settings: they are left out when you open it, so the helper starts off there.
The window shows about what one question costs. While it is on, a purple **AI on** tag at the top says
where pictures can go (click it for the settings).

- **Ask the AI: object or background?** in Investigate (7).
- **Name this object** (right side, for the selected object): a type you have is one click (**Z**
  takes it back); a new name opens Object types with it added, for you to save.
- **Second check of a sample**: small pictures of 8-48 confirmed labels; it lists any it doubts.
  **Look** takes you there with the object selected; you decide.
- **Draft labels for a type**: rough places on this frame, as dashed purple circles; **Add it** makes
  one your label (then drag it onto the middle).
- **Describe what you track in a sentence** in the setup (3).

Every answer is marked **AI** and changes nothing until you take it. Your answers (taken, turned
down) are kept, and **Time saved** counts them per task with the cost so far.

## 18. Smarter following

- An object that stops is followed **by its look**: while nothing moves at its place, its position
  is held exactly; something passing over it (a hand) marks the frames for a look.
- When two lookalikes come together and part, both readings are kept until it is clear which is
  which. If it stays unclear, those frames come first in **Most useful next**, with the reason above
  the picture ("two lookalikes crossed here: which is which?").
- **Reject here** (7) teaches the tracker what an empty spot looks like; **Z** / **Shift+Z** undo and
  redo every change (5); **C** plays a short clip around the frame (5).
- How fast each type moves comes from Object types (3); **How often should I check?** is in
  Learning curve (4).

## How the scores stay honest

- Frames are split by **time blocks**, never frame by frame: neighbouring frames are
  near-copies. A block goes wholly to training (learning), to picking the best training round,
  or to scoring, and frames close to a block edge (1 s; less for short blocks) are left out.
- **The block length is chosen automatically** from how much footage the project has: 30 s
  when there is enough, shorter (20, 15, 10, 6, 4 or 3 s) when the recordings are too short to
  give every part some blocks. Once a model is trained, the length stays the same as long as it
  still fits, so frames never move between parts. This is why recordings of a few minutes or
  more work best (section 2).
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
| `outside.py`, `robotlogs.py`, `homes.py` | outside data (robot logs, positions files), a robot-log plug-in, home positions |
| `outsidelink.py` | the studio's glue for outside data and home positions |
| `ai_helper.py` | the optional AI helper |
| `videofile.py` | an ordinary video file made into a recording folder (Add recording) |
| `cameras.py`, `crosscam.py` | camera models and label sharing between cameras (CROSSCAM.md) |

The labelling page is `tools/label/page.html` (with `tools/label/app.py`, `tracker.py`,
`motion.py`, `select_frames.py`, `workarea.py`, `investigate.py`, `timing.py`); the detector is
`tools/detector/`.

Tests:

```
python -m pytest tools/recognition/tests tools/label/tests tools/detector/tests
```

Add `RECOGNITION_TRAIN=1` (PowerShell: `$env:RECOGNITION_TRAIN=1`) to include the few-minute
training test.
