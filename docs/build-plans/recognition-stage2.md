# Build plan: recognition stage 2 (2026-10-20 overnight, Nick asleep)

Nick: "Build as much as you can without it [the real footage]." Priority if usage runs tight:
smarter tracking first (his answer). Going past 90% of the weekly meter is allowed; finish the job.
Real footage (roam1, robot logs, his labelled project) arrives tomorrow as a GitHub release
`roam1-data`; anything that needs it is built and tested on synthetic data now and flagged for a
real-footage session. AI helper: built and tested with a stand-in; Nick will provide a key later.

Source of every item: `docs/recognition-next-build.md` (read it all). The handoff build's report:
`docs/build-reports/recognition-handoff-1.md`. The rules there still hold: existing saved files keep
their meaning; plain words; generic (Alvik knowledge only in the Alvik preset/plug-ins); never write
"in real life"; never a silent screen. New standing rules from Nick (2026-10-20):
- **Assume nobody reads first.** Every action announces what it just did in a short message that
  fades by itself: what changed, where, and the exact key or button that takes it back. Nothing that
  changes saved work, or moves the person elsewhere, happens silently. Check every message for the
  take-back part.
- **Rejections inform, never blanket-ignore.** No action excludes a spot for the whole recording;
  a rejection applies to that frame and strongly informs later decisions at that spot.

Lanes (parallel where files don't overlap; lanes don't commit, the main session does):

## T: smarter tracking (tools/label/tracker.py, new tools/label/appearance.py; tests)
1. Appearance chain: each track carries the object's picture patch; each frame, a still or
   undetected object is found by matching its patch near its last place (frame-to-frame), with
   confidence that falls with chain length and when the match to the person's confirmed patch
   weakens; a still object holds position exactly (no drift); something passing over breaks the chain
   and marks the frame for a look. Replaces "drop after 45 s unseen" for objects the person confirmed.
2. Split branches when two candidates fit: follow both (cap 3 per track), decide after ~1.5 s by
   patch match + plausible motion (per-type max speed), keep the winner; unresolved splits are output
   as a flag so the queue can put them first.
3. Rejections inform: a rejected guess stores the empty patch at that spot; later guesses there that
   look like it are scored down hard; anything that looks different is still found.
4. Check-in measurement: a function that replays the chain between pairs of confirmed frames of the
   same object and reports error vs chain length (for the real-footage session to set the limit).
5. Per-type motion settings (max speed, can be carried, can sit still) read from object types with
   generic defaults.
Tests on synthetic recordings with still objects, a robot that stops, crossings, a stain.

## U: interface (tools/label/app.py, page.html, studio.py; tests)
1. Messages for every action (rule above) and a redo (Shift+Z) for everything Z undoes.
2. Investigate made clear: three separated parts (what we found: read only / look around: jumps that
   change nothing / change things: in its own warning-coloured box); every change button states its
   reach ("Reject all 12 labels at this spot, frames 812-4410"), previews and confirms; jumps keep a
   one-click way back ("press Q to return").
3. Remove "Ignore this spot"; replace with "Reject here" behaviour from T3 (UI side), keeping
   existing ignore spots working (shown in Work area, removable).
4. Queue: unresolved splits and broken chains first in "Most useful next" (hook for T's flags).
5. Short looping clip (about 1 s either side) on a key (C) for judging what's happening.
6. Small items from the list that remain (Most useful next counts, sidebar refresh).

## L: outside data, home positions, robot logs (new files: tools/recognition/outside.py,
tools/recognition/robotlogs.py, tools/recognition/homes.py; tests)
1. Generic "outside data": a log of positions/headings/events on the shared clock (t_wall), mapped
   into the camera picture through the floor calibration; produces draft labels (with facing) and
   votes on split branches. Plug-in interface; the Alvik robot-log reader is one plug-in (read the
   log formats in docs/data-formats.md and tools/roam_session.py / alvik_console.py; robot poses are
   in each robot's own start frame: alignment to the floor = a starting pose from a home position,
   a confirmed label, or the first stretch of forward driving seen on video).
2. Home positions (generic form of the dock): spots where an object's position and facing are known
   exactly, marked in setup; a visit = a free confirmation; the dock design (hardware/dock) as the
   Alvik example. Simulated docked session for tests.
3. A synthetic generator for robot logs that match a synthetic recording (with realistic gyro scale
   error ~7%, drift, and wheel slip) so all of this is tested now.

## A: AI helper (new: tools/recognition/ai_helper.py; tests)
Off by default; company's own key; provider-agnostic (Anthropic adapter first, plus a generic
OpenAI-compatible one); stand-in AI for tests. Uses: "robot or stain?" on a spot, name an unknown
object, second check on a sample of confirmed labels, draft labels for a new type, set up a project
from a sentence. Suggestions only (source "ai"), every call logged (what was sent, cost, answer),
visible indicator, only crops sent. Settings: key entered in the studio, stored locally, never in
the project file or shared file.

## X: training extras (tools/recognition/trainer.py, report.py, tools/detector/*; tests)
1. Rejected guesses used as "not an object" examples (hard negatives).
2. Copy-paste augmentation behind a switch, kept only if it raises the score on real held-out
   frames (train with and without; the report shows both).
3. Report by situation: accuracy alone / close together / near a person / at the edge / still.

## R: live use (tools/recognition/use_model.py; tests)
Roll call at the start of a live run (the page shows what it sees; the person confirms or fixes
once; with outside data, robots can identify themselves); per-object confidence and "last
confirmed"; asks for a fresh confirmation when confidence drops.

## After the lanes
I: integration (wire T, L, A, X, R into the studio UI and the README). Then checks: suites, browser
click-through, old project opens unchanged (fixture /tmp/claude-0/fixture), newcomer, code review,
verifier; a fix round; report with the two sweeps and the cost.
