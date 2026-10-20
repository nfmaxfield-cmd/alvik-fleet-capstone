# Build plan: recognition handoff version (2026-10-20)

Agreed with Nick 2026-10-20 (chat). Source list: `docs/recognition-next-build.md` (read it all; the
items below are the subset in this build, with the decisions made for them). Nick is not technical:
everything a person sees must be plain words. He labels on Windows (the studio at
`C:\Users\nfmax\Documents\recognition-project`, opened by `start-recognition.bat`); he has a real
project in progress, so **nothing may change the meaning of existing saved files** (labels.csv,
review.json, project.json). New fields are added with safe defaults; old projects open unchanged.

Code: `tools/recognition/` (studio, project, trainer, report, start, cameras, crosscam),
`tools/label/` (app.py = labelling session + web server, page.html = the whole UI, tracker.py,
motion.py, select_frames.py), `tools/detector/` (train, export, infer, evaluate).
Tests: `python3 -m pytest -q tools/recognition/tests tools/label/tests` (label tests take ~4 min;
the slow training test runs only with RECOGNITION_TRAIN=1). Every lane adds tests for what it builds
and runs the suites before reporting. **Lanes do not git commit** (the main session commits).

Rules for all lanes
- Generic first: no Alvik knowledge outside the ready-made "Alvik robots" preset. The words "Alvik",
  "alvik" may appear in code only for that preset, the Alvik-specific tools, and existing projects.
- Never a silent screen: any wait over ~0.3 s shows what is happening, a progress bar when the amount
  is known, and an honest time left (from the recent rate, not the first items).
- Name: **Maxfield IRL Tracking**, with "Identify. Recognize. Label." in smaller letters beneath.
  Never write out "in real life" anywhere.
- Keys in use: 1-9, arrows, Backspace/Delete, E, Enter, Esc, F, G, H, I, N, Tab, U, Z (undo).
  New keys in this build: `?` unsure, `Q` investigate, `M` magnifier, `B` picture brightness,
  `V` full screen, `W` work area. Every key is listed in the side panel's KEYS list.

## Lane 1 — polish (owns page.html, app.py, studio.py, project.py, select_frames.py, crosscam.py
start.py, report.py, README.md; lane 4 follows it on the same files)

1. Name and branding everywhere a person sees it (page title, top bar, launcher window/console
   lines, report heading, README heading). `start-recognition.bat` window title too.
2. Generic: a NEW project starts with no object types until setup. Colours come from a palette by
   position (not fixed per name). README opening is generic ("for any objects a fixed camera sees").
3. First-start setup (a project whose settings lack `setup_done`; existing projects that already have
   labels count as done and get a one-time "what's new" + the tour instead): the person confirms or
   edits, before judging any frame: project name; object types, either the ready-made **Alvik
   robots** set (alvik: has a front, height 4 cm; person: hands/arms/feet, height varies; unknown:
   height varies) or their own list (unknown is always kept as the catch-all); keys and colours shown.
   Then the tour. A **?** button in the top bar reopens the tour any time (it is a button; the `?`
   key is unsure).
4. Tour: short overlay steps pointing at each tab and button: what it does, which are safe to try,
   and which change saved work (Accept remaining high-confidence frames, Reset this frame, Train new
   model).
5. Never a silent screen: loading overlay for open recording, switch model ("Guesses from"), save
   object types, add recording, export/import, any reselect. Motion pass time-left from the recent
   rate (console and overlay).
6. Keys: `?` unsure (object status "unsure": drawn dashed grey with a ?, saved in labels.csv status
   column as `unsure`; frames holding an unsure object are kept out of training/validation/scoring
   until resolved — trainer.build_dataset must skip them; a new tab **Unsure** lists them).
   `M` magnifier (a 3x lens around the mouse, toggle). `B` cycles picture: normal / brighter / high
   contrast (display only). `V` full screen (and a button).
7. Same-picture frames: (a) select_frames applies the near-duplicate skip to every rule except
   start/end (new recordings; existing selections are kept as saved). (b) When the person accepts a
   frame, pending picked frames within 10 s whose picture is near-identical (the saved JPEGs, same
   test select_frames uses) and whose guesses sit where the accepted objects are, are accepted with it
   (state auto, how "same"), with a toast "Also accepted N frames with the same picture (Z undoes)";
   Z undoes them together.
8. Height "it varies": object types get a "varies" checkbox; the column is titled "Height of the
   marked point above the floor (for the second camera)". Types that vary are not passed between
   cameras (crosscam skips them). Existing projects: hand and unknown are set to varies on first
   open ONLY if the person confirms in the what's-new screen (default ticked).
9. Work area per camera: button **Work area** (key W): click the corners on the picture, Enter to
   finish, or "Use the whole picture". Stored in project.json `cameras[<camera>].work_area`
   (polygon, image px) plus `margin_px` (default one robot size) and `ignore_spots` (list of x, y, r).
   Guesses (motion and model) outside area+margin or inside an ignore spot are dropped before
   tracking (the cached motion pass is not re-run; filtering is applied when guesses are built, then
   re-track). First open of a camera with no work area offers to mark it (skippable).
   Alvik shortcut not in this build.
10. Second camera: when a session is added, start the second camera's motion pass in the background
   after the first finishes, so switching is instant (with its progress shown in the top bar). Only if
   it fits cleanly; otherwise leave a note.
11. README becomes the user manual for the other team (install, setup, the loop, keys, unsure,
   investigate placeholder section for lane 4, work area, two cameras, starting over, where files are).

## Lane 2 — the "use it" tool (new files only: tools/recognition/use_model.py,
`track-with-model.bat`, tests/test_use_model.py; may import from detector/ and label/)

Run a trained model without the studio, offline:
- Source: a recording folder or video file, or a live camera (index) or stream URL (e.g. the Pi's
  preview stream).
- Model: `--project <folder>` uses the project's chosen/best model (or `--model <id>`); `--onnx` path.
- Uses the project's work area and ignore spots; the label tracker gives each object a steady number.
- Output CSV per frame per object: frame, t_wall, obj, cls, x_px, y_px, heading_deg, score, and
  x_cm, y_cm when the camera has a floor calibration in the project (cameras.CameraModel).
- Live: a small local page (same look) with the picture, the objects, and a confidence colour per
  object (green / amber / red from score and tracking age), plus frames per second.
- The .bat asks plain questions if run with no arguments (which project, which recording or camera).
- Tests: on the synthetic recording; build a tiny model the cheapest way the existing tests do (or a
  stub detector behind the same interface) so the test runs in under a minute.

## Lane 3 — project as one file (new: tools/recognition/share.py, tests/test_share.py; small,
clearly marked additions to project.py are allowed: coordinate by keeping them in new functions)

- Export a project folder to one file `<name>.mxirl` (a zip): project.json, labels, models, cameras,
  stats, curve, report; recordings' videos are NOT included by default (`--with-video` includes
  them); a manifest lists what is inside and the recordings it refers to.
- Import into a new folder; recordings are re-linked by searching a folder the person gives
  (`--recordings`), matching on recording/session name; missing ones are listed in plain words and
  the project still opens (the studio must show "recording not found: locate it" rather than crash —
  note for lane 4 to wire in the UI).
- CLI: `python tools/recognition/share.py export <project> <file> [--with-video]`,
  `import <file> <dest> [--recordings <dir>]`. Functions callable from the studio.
- Round-trip test: export, import elsewhere, labels/models identical, studio opens it.

## Lane 4 — investigate, time-saving stats, project-file buttons (after lane 1; same files)

Investigate (`Q`, on the object or spot under the mouse; also a button):
1. Scan the recording at that spot (radius ~ one object size): a strip of ~40 small pictures over
   time, a line of how much the spot changed over time, the 5 biggest changes as short clips (a few
   frames each), and the spot on the background picture (empty board, from the motion pass or a
   median of samples). Shown with a progress bar while it scans.
2. Verdict in plain words: never changes and matches the background -> "part of the background" with
   a button **Ignore this spot** (adds to the camera's ignore_spots, re-tracks); moves -> where it
   went (jump buttons); appears and leaves -> "something put down / passing over", with jumps.
3. Every label already made at that spot, listed (frame, type, status), with **Change all to...**
   (type), **Reject all**, **Mark all unsure**. Undoable as one step.
4. Second camera as referee when both cameras are calibrated: the other camera's crop at the
   matching moment, at the floor point (height from the type, or floor if varies).
5. Unsure items and investigate both work with no Alvik knowledge.

Time-saving stats:
- Record per saved frame (append-only `stats/<rec>.jsonl` in the project): when shown, when saved,
  active seconds (capped at 120 s), number of edits, outcome (accepted as guessed / corrected /
  auto-accepted / same-picture / unsure), guesses source (none-model id), model version.
- Existing frames: estimated from review.json save times (gaps capped), marked "estimated".
- Install and setup times (start.py records install duration; setup screen records its time).
- Shown in the report and a **Time saved** panel: seconds per frame and share accepted unchanged, by
  model version; frames auto-accepted and same-picture; totals. Export as CSV for the final report.

Project file in the studio: **Share project** (export) and **Open shared project** (import) in the
top bar, with progress; the "recording not found: locate it" flow.

Track timeline (only if all above is done and tested): one lane per object across the recording,
breaks and jumps marked, click to jump.

## Checks (after the lanes)

1. All suites after every change. 2. New tests per feature. 3. Generic-wording check (a test that
fails if "alvik" appears in user-facing text outside the preset). 4. Browser click-through
(Playwright, Chromium at /opt/pw-browsers): every tab/button/key, no blank or silent screen,
screenshots saved. 5. Clean install in an empty venv following the README. 6. An old project opens
unchanged (built with the pre-build commit's code, then opened with the new). 7. Newcomer test: a
fresh agent installs and uses it from the README only. 8. Code review focused on lost or corrupted
labels. 9. Verifier: the build report against the repo. Then a fix round.
