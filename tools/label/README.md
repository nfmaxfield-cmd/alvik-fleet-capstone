# Labelling tool for the overhead camera

Makes labels for the camera detector (`tools/detector/`). The computer makes
the first guesses, and you only confirm or correct them. The first round's
guesses come from motion in the video. Later rounds can use a trained model.
The output follows docs/data-formats.md ("Labels for the camera detector").

Needs Python 3.8+ with `pip install numpy opencv-python`. It runs on Windows,
macOS and Linux and needs no internet connection.

## Workflow

1. **Record** on the Pi as usual (`tools/pi/camrec.py`), then copy the
   recording folder to the laptop (see tools/pi/README.md).

2. **Start the app** from the repo folder:

   ```
   python tools/label/app.py C:\Users\you\Desktop\recordings\20261017-101500_E1
   ```

   The first start goes through the whole video once, which takes about a
   minute per 3–5 minutes of video. It then picks about one frame per second,
   plus extra frames at tricky moments, and opens the page in your browser.
   Later starts are quick and continue where you stopped.

3. **Label.** Each picked frame shows the guesses. Each guess has a dot on the
   robot's centre, a ring the size of a robot, an arrow for the heading, and a
   tag with its number and status.

   | Key | What it does |
   | --- | --- |
   | **Enter** | Every guess is right: accept the frame and go to the next one |
   | click, drag | Select an object, then move it to the right spot |
   | **H** then click | Set the heading: click where the robot faces (or drag the arrow tip, or Shift+drag) |
   | **F** | Flip the heading 180° (for a robot that was reversing) |
   | **Delete** | Reject a wrong guess (press again to bring it back) |
   | **N** or double-click | Add an object at the mouse (a robot the guesses missed) |
   | **1 2 3 4** | Class: alvik, hand, unknown, machine |
   | **E** | No objects on this frame: mark it empty and go next |
   | **← →** | Previous / next frame (Shift: jump 10) |
   | **Z** | Undo (also Ctrl+Z) |
   | Tab, I, G, Esc | Next object, change an object's number, go to frame, cancel |

   - **Robots that are not moving yet** have no guess, because motion cannot
     see them. Press N on each one on the first frame. The tracker then
     follows them through the frames after it.
   - **A fix carries forward.** When you move, add or reject an object, the
     tracker re-runs from that frame, so the frames after it get the fix too.
     The page says how many later frames changed.
   - **A dashed arrow** is only a heading guess, shown while the robot turns on
     the spot or seems to back up. It is saved as "heading unknown" unless you
     set it with H, drag or F. A solid arrow is saved as given when you press
     Enter.

4. **Choose a list** at the top:
   - *All picked frames*: everything, in order.
   - *Low confidence*: only frames where a guess scored below the threshold
     (0.6 by default; you can change it on the right), plus moments where
     objects appear, vanish or come close to each other. These frames need
     your eyes most.
   - *Spot check*: a random 10% of the high-confidence frames nobody has
     looked at. Check each one (Enter if right, otherwise fix it). The panel
     shows how often the guesses were fully right, with a 95% range. That is
     the measured accuracy of the high-confidence guesses.

   A quick way through a long recording: do the low-confidence list, then the
   spot check. If the agreement is good enough, press *Accept remaining
   high-confidence frames*. That accepts their guesses without you looking.
   *Undo auto-accept* takes it back.

5. **Labels** are saved after every change (you can close the window any
   time) in `<recording>/labels/`:
   - `labels.csv`: one row per object per picked frame, columns as in
     docs/data-formats.md. For training, use the rows with status `confirmed`
     or `corrected`, and the `none` rows (frames checked and empty). Use
     `rejected` rows as examples of what is *not* an object. Rows with status
     `pending` are guesses nobody has checked yet.
   - `review.json`: the picked frames and the reason for each, plus who
     checked each frame and how. `how` is `user` (checked one by one), `spot`
     (checked in the spot check) or `auto` (auto-accepted). It also holds the
     spot check sample and result.
   - `frames/<frame:06d>.jpg`: the picked frames.
   - `cache/`: the saved guesses (delete it to make them again).

   A guess you moved keeps its source (`motion`, `track` or `model`) and its
   score, and gets status `corrected`. That way the file shows which kinds of
   guesses needed fixing. Objects you added have source `user` and no score.

## Later rounds: guesses from a trained model

Any detector with the interface in docs/data-formats.md works:

```
python tools/label/app.py <recording> --model tools/detector/detector.py --weights tools/detector/model.onnx
```

Add `--model-step 3` to run a slow model on every 3rd frame only (the tracker
fills in between). Other options: `--port 8766` if 8765 is busy, `--robot-px 32`
to give the robot size in pixels instead of measuring it, `--reselect` to pick
the frames again (frames already labelled are kept), `--no-browser`, and `--no-look`
(follow objects by the guesses only, not also by their look: a quicker first start, but an
object that stops for a long time may be lost).

## The pieces

| File | What it is |
| --- | --- |
| `app.py` | the program you start: local web server + labelling logic |
| `page.html` | the page (one file, nothing loaded from the internet) |
| `motion.py` | motion detector. It is also the project's non-learning comparison detector. Use it frame by frame (`detect(frame, state)`) or through the detector interface (`load`, `predict`). `python tools/label/motion.py <recording> --show` plays the video with its guesses |
| `tracker.py` | follows guesses from frame to frame. Gating stops a far-away look-alike from taking over a robot's number. Stopped robots stay put, headings carry over, and your corrections act as anchors |
| `select_frames.py` | picks the frames to label (named so because a `select.py` here would hide Python's own `select` module) |
| `tests/` | synthetic recordings with known truth (`synth.py`), an accuracy report (`evaluate.py`), and the tests |

Run the tests from the repo folder with `python -m pytest tools/label/tests`
(this needs `pip install pytest`; it takes about 3 minutes). To see the
accuracy numbers on the synthetic recordings, run:

```
python tools/label/tests/evaluate.py --make <some empty folder>
```
