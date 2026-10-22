# Recognition studio: list for the next build

A running list, kept while using the studio on real footage. Each item says who raised it.
Nothing here is built yet; the next build works through it.

## Name and branding (Nick)

- Rename the software **Maxfield IRL Tracking**, with "Identify. Recognize. Label." written
  in smaller letters underneath (what IRL stands for). Applies to the page title, the top bar,
  the launcher window, the report, the README and the handbook section.

## Feels generic, not Alvik-specific (Nick)

The studio should not say "Alvik" until the project is about Alviks. Places it still does
(found 2026-10-20):

- Page title and top bar say "Alvik labelling" (`tools/label/page.html`).
- New projects start with the object types alvik, hand, unknown, machine
  (`tools/recognition/project.py` DEFAULT_CLASSES, `tools/label/app.py` CLASSES). Proposal: a
  new project starts with only "unknown", and the first screen asks what objects you track,
  with "Alvik robots" offered as a ready-made set.
- Colours are fixed per Alvik type name (`--alvik`, `--hand`, ...); give types colours from a
  palette by position instead.
- README opens "Built for the Alvik robots".

## Never a silent screen (Nick)

- Whenever the studio is working (opening, finding moving objects, training, switching
  models, saving), the page shows it: what it is doing, a progress bar, and a time left that
  is honest. Never a black or frozen picture with no sign of life. Check every wait, not only
  Add recording.

## Found while using it (Claude)

- After **Add recording** the page stayed blank instead of showing the progress bar
  (fixed 2026-10-20, commit after 8e1bcc5; reaches the laptop on the next pull).
- The black window's time estimate for "finding moving objects" was too high (said
  ~1000 s left; it took about 12 minutes for 24,000 frames). Estimate from the recent rate,
  not the first frames.
- Adding a two-camera session opens (and runs the motion pass on) the first camera only; the
  second camera's pass should start when you first switch to it (to confirm on roam1). Consider running both in the background, so the
  second is ready when you get there.
- No way to say "I can't tell which way it faces": the tracker sometimes marks a heading as
  known (solid arrow), and Enter saved it. Added the **U** key (facing unknown) 2026-10-20.

## Direction (facing) labels (Nick, 2026-10-20)

- A still frame often can't show which way a robot faces (grey picture, ~30 px robots, a
  roughly square body). Hand-set direction only where it is plainly visible.
- Proposal: facing comes from sources that know it, not from a person guessing:
  the robots' own logs (each robot records its heading; match its track on the shared clock),
  and later the light attachment (5 front, 2 rear lights: visible in one frame).
- Short looping clip (about 1 s either side) on hover or a key, for judging what's happening:
  useful for hand vs robot, objects being placed, and motion. Note: motion direction is not
  facing (robots reverse and spin in place).
- Tracker headings showed as solid and were saved as known when Enter was pressed. Fixed 2026-10-20:
  motion headings now show dashed (a guess) and save as unknown unless the person sets them.

## Chained guesses for still objects (Nick, 2026-10-20)

Problem seen: Center sat still in the corner all session and was never guessed. Guesses come
only from motion; a robot the person adds is carried forward at its spot, but fades and is
dropped after 45 s with no motion (tracker max_absence_s), and picked frames are usually
further apart than that.

Nick's idea: chain frame to frame. A confirmation informs the next frame, that frame the next,
and so on: neighbouring frames look almost identical, so each step is safe, where comparing
frame 1 with frame 100 directly may not be.

- Add an appearance link to the tracker: compare the small patch around each object with the
  previous frame's patch. A still object stays found for as long as its patch matches.
- Keep drift low automatically: (1) if nothing moved there, the position stays fixed (no
  drift at all for still objects); (2) check each step against the previous frame and against
  the patch the person confirmed; if they disagree, lower the confidence; (3) break the chain
  when something moves over the object (a hand), and ask for a look; (4) confidence falls
  with chain length.
- How often a person should confirm: measure, don't pick. Run the chain from one confirmed
  frame to the next confirmed frame of the same object and record the error against the
  chain length; the tolerance (about a quarter of a robot) sets the length. Then ask for a
  fresh confirmation when the chain's confidence drops, rather than every N frames.

## Frame picking (to discuss with Nick; likely kept)

Current rules (select_frames.py): about one frame per second, skipping near-duplicates; extra
frames when an object appears, disappears, two objects are close, or a guess scores low; the
first and last frame. "Most useful next" orders them least-confident first.
Gap seen: a still object is never detected, so it never makes a frame "interesting" either.
Gap seen (Nick, 2026-10-20): runs of near-identical frames. The near-duplicate skip only applies
to the once-a-second frames; "close" frames are taken every 0.5 s while two objects are close,
so robots parked side by side give a stack of the same picture. Skip near-duplicates for every
rule, and once a frame is confirmed, accept its unchanged neighbours with it (or show them as
one item: "5 frames, same picture").

## Split the chain when it's unsure (Nick, 2026-10-20)

When the chain finds two candidates that look alike, don't pick one: follow both branches,
and compare each branch further down with the confirmed path from before the split. A lookalike
rarely keeps looking alike (and moving plausibly), so the wrong branch dies out.
(Known as multiple hypothesis tracking, Reid 1979.)

- Cap the branches (keep the best few, drop the weakest), or they multiply.
- Decide a split only after a short window (about 1-2 s), then keep the winner.
- Evidence for each branch: looks like the confirmed patch; moves in a way a robot can
  (no jumps); later, agrees with the robot's own log.
- A split that never resolves (two identical Alviks crossing: they look the same forever) is
  exactly the frame to show the person: it should go to the top of "Most useful next".

## Making it work for any project (Nick, 2026-10-20: standing rule)

Every Alvik-specific idea gets written down with its generic form too. The studio should take
knowledge about the objects as optional plug-ins that a project can switch on, never as
built-in assumptions.

| Alvik-specific | Generic form |
| --- | --- |
| ID lights tell Left / Center / Right apart | "Identity mark": an optional per-type way to tell individuals apart (lights, colour, a printed tag, a shape). Without one, identity comes from motion continuity only, and unresolved crossings go to the person |
| Facing from robot logs and front/rear lights | "Facing source": optional per type (a log file on the shared clock, a visible front mark, or none). Types without a front have no facing at all (already: has_front) |
| Robots can't jump; max speed ~3 robot sizes/s | Per-type motion limits in Object types (max speed; can it be carried by hand; can it sit still for a long time) |
| Robot logs confirm a split branch | "Outside data": any log of positions or events on the shared clock (a robot arm's controller log, a conveyor's encoder) can vote on branches |
| Still robots on a fixed board | Appearance chain works for any still object; the per-type "can sit still" setting decides how long a still track is kept |

## First start: set up with the person, not for them (Nick, 2026-10-20)

- Before the first frame, a short setup that the person confirms or edits: what objects are
  tracked (with the ready-made "Alvik robots" set as one choice), their keys (1 alvik, 2 hand,
  3 unknown ...), colours, whether each has a front. Nothing judged until this is confirmed.
- A first-time tour of the screen: what each tab and button does, which ones are safe to try,
  and which change saved work (Accept remaining high-confidence frames, Reset this frame,
  Train new model). Can be reopened from a "?" button.

## Object height: allow "it varies" (Nick, 2026-10-20)

Height is only used to pass a label from one camera to the other (where the labelled point
sits above the floor). Hands get a fixed 10 cm and unknown objects 0 cm, both trusted to
+/- 0.5 cm: wrong for anything whose height changes (a hand) or differs per object (a cup vs a
box), so those guesses in the second camera would land in the wrong place with too much
confidence.
- Add an "it varies" choice (the default for hand and unknown). Such types are not passed
  between cameras, or are passed only as a low-confidence hint; the person places them.
- Generic form: the same choice for any type (a part lifted by a robot arm varies).
- Rename the column in plain words: "Height of the marked point above the floor (for the
  second camera)".

## Long or odd-shaped things (Nick, 2026-10-20)

- "hand" really means a person: hands, arms, sometimes feet. Rename it in the generic version
  (e.g. "person"), and say in the setup where to mark it (the end reaching in).
- Labels are one point per object with a fixed-size circle, so an arm reaching across the
  board can only be marked at one spot. Fine for robots; for long objects (arms, tools, a
  robot arm's links) consider an optional box or outline per type.

## Work area per camera (Nick, 2026-10-20)

The process should start by marking where to look. Today the whole picture is searched: the
motion pass, the frame picker and the model all spend effort on the room around the board.
- Setup step per camera: draw the work area on the first frame (click its corners), with a
  margin band around it, plus optional "ignore" spots inside (a reflection, a blinking light
  on a shelf).
- Uses: motion and the model only report things inside the area (or the margin, for hands
  reaching in); frames are picked for what happens inside; scores count inside only; faster.
- Alvik shortcut: the floor calibration's tape marks already give the board outline.
- Generic: the other team draws their table or the arm's reach.

## Claude's ideas (2026-10-20; Nick's answers below)

1. Fix tracks, not frames: a timeline with one lane per object across the recording; breaks,
   jumps and swaps are marked; one fix corrects every frame of that stretch.
2. Labels from outside data: robot logs (wheels + gyro, on the shared clock) placed into the
   camera picture give thousands of draft labels with facing; the person only checks.
3. "Use it" tool for the trained model: run it on a live camera or a new recording and save
   positions to CSV, with no studio needed (what the other team actually needs day to day).
4. Magnifier and picture controls: zoom around the mouse, brightness/contrast toggle.
5. Consistency flags: a confirmed label that jumps between nearby frames, or disagrees with the
   other camera (crosscam.disagreements exists), goes to the top of the list.
6. More data from the same labels: paste labelled objects onto empty-board frames (copy-paste
   augmentation), so every label is seen in many places and lightings.
7. Report by situation: accuracy split into robots alone / close together / near a hand / at
   the edge / standing still, so the next recording targets the weak one.
8. Labelling-time record: seconds per frame and fixes per frame over time; shows each model
   saving work (a strong number for the report).
9. Share a project as one file (labels, settings, models), for handing over or a second
   labeller.

Nick's answers (2026-10-20): yes to 1, 2, 3, 4, 5, 6, 8, 9. 7 only if it proves itself (below).
- 3, live use: start every live run with a roll call (the studio shows what it sees, the
  person confirms or fixes once; Alviks can identify themselves by blinking), then show a
  confidence per object all the time (green / amber / red, "last confirmed 2 min ago"), and
  ask for a fresh confirmation when it drops. Generic: the person does the roll call.
- 5: plus a one-key full-screen mode.
- 7: pasted objects can carry wrong shadows, seams or impossible spots. Only paste onto the
  work area of the same camera, never score on pasted frames, and keep it only if the score on
  real held-out frames goes up (train with and without; the report shows both).
- 9 and 8: track every time saving for the final report to the professor: seconds and fixes
  per frame, frames auto-accepted, labels from outside data vs by hand, per model version;
  also setup and install times. Each saved frame already stores its save time, so today's
  labelling can be counted.

## Lab changes that make training easier (to discuss)

- Aim the overhead camera so the board fills the picture (now about half): more pixels per
  robot for free.
- Every session: robot logs on and synced (already), and an empty-board shot at the start and end.
- The dock is the session's anchor (rewritten 2026-10-20, replaces "taped spots"):
  - Start and end every session parked in the docks. A parked robot's position, facing and
    identity are known exactly, so the first and last labels of every track are free and exact.
  - The robot's log starts at the dock, so its heading zero is the dock's known direction:
    that is what lets the logs be placed into the picture (draft labels with facing, idea 2).
  - Visits mid-session (parking, or a `run posts` touch at a post station) are automatic
    check-ins: the chain gets a fresh, exact confirmation without the person, and the log's
    drift is measured and corrected at the same moment.
  - The end-of-session return error is a free accuracy number for both the logs and the camera.
  - The check tile on each dock never moves: the camera checks it every frame, so a bumped
    camera is caught at once (live confidence drops; labels after the bump are flagged).
  - Generic form: "home positions": any fixture where an object's position is known exactly
    (a robot arm's home pose, a parts tray, a fixture on a conveyor). Marked in setup; every
    visit is a free confirmation.
- The light attachment: identity and facing visible in every frame. Until then, a printed
  high-contrast arrow on each robot's top gives facing in a still frame (keep it on for good:
  the model learns it).
- Planned variety per session, aimed at the report's weak situations (close pairs, hands,
  edges, still robots, a lamp switched on and off).

## "Investigate this" (Nick, 2026-10-20; all points agreed for the next build)

Seen: a small dark mark on the board that may be a robot or a stain; labelled as an Alvik on
several frames before doubt set in.
- Circle a spot, press a key: the studio looks at that spot across the whole recording and
  shows (1) a strip of thumbnails of the spot over time, (2) a line of how much the spot
  changed over time, with its biggest changes as short clips, (3) the spot on an empty-board
  frame (or the motion pass's background picture).
- Verdicts it can offer: never changes and is on the empty board -> part of the background:
  "ignore this spot" (feeds the rejected-spot list / ignore zones); moves -> show where it
  went; appears and leaves -> an object that was put down.
- Every label already made at that spot, listed with a "change all" button (fix past mistakes
  in one go).
- Second camera as a referee (once calibrated): a robot stands up 4 cm, so it shows in the
  side view at the matching spot; a stain doesn't.
- An "unsure" mark (key ?): saves the object as unsure, kept out of training until resolved,
  and listed in its own queue. Never force a guess.
- Generic: works for any object type; no Alvik knowledge needed.

## Optional AI helper, for projects allowed online (Nick, 2026-10-20; to design)

Off by default; the studio stays fully offline without it. A project switches it on with the
company's own AI access (API key), from any provider.
- Good at judgment calls on a few frames: "robot or stain?" on an investigated spot; naming
  unknown objects (cup, box, tool); a second check on a sample of confirmed labels (catches
  mistakes); first draft labels for a brand-new object type before any model exists; setting
  up a project from a sentence ("we track printed cubes and cylinders on a table").
- Not for every frame or exact positions: general AI is slow and costs per picture, and is
  weak at pinpointing 30-pixel objects in a grey frame. The trained model stays the workhorse.
- Its answers are suggestions (source "ai"), never confirmed without a person; every call is
  logged (what was sent, cost, answer); a visible indicator when anything leaves the computer;
  only the crop or frame needed is sent.
- Counted in the time-saving stats (frames where the AI's suggestion was accepted).

## Personalised for the other team (Nick, 2026-10-20; waiting on their answers)

Ask them first (questions sent via Nick). Likely candidates, depending on answers: positions in the
robot arm's own coordinates (camera-to-arm calibration by touching known points: their "dock");
draft labels from the arm's controller log (the arm knows where its gripper is, and where it put each
part: our data-merging principle); orientation for symmetric shapes (a cube's angle repeats every
90 degrees); the arm itself as an object type that hides parts; a live output their arm code can read.

## Investigate: clearer actions (Nick, 2026-10-20, after using it on roam1)

Nick: "The Q feature is amazing, that needs to be featured when we give a presentation. But it's
easy to press the wrong thing because it isn't too clear what means what."
- Group the window into three plainly separated parts: what we found (read only) / look around
  (jump buttons: change nothing) / change things (change all, reject all, unsure all, ignore this
  spot), the last in its own box with a warning colour.
- Every action button says its reach in its own label: "Ignore this spot in every frame of this
  camera", "Reject all 12 labels at this spot (frames 812-4410)"; show a short preview of what will
  change and ask to confirm before anything that changes saved work.
- One undo for every action, ignore-spot included (today: Z undoes change all; an ignore spot is
  removed in Work area, W).
- Presentation: a short scripted demo of Q (stain vs robot, then change all).

## Rejections inform, never blanket-ignore (Nick, 2026-10-20)

Nick: no scenario needs an area eliminated for the whole recording; a rejection should apply to that
frame, and what it teaches should strongly inform later decisions at that spot.
- Replace "Ignore this spot" (permanent, every frame) with: reject in this frame, and remember what
  the empty spot looks like there. Later guesses at that spot that look like the remembered empty
  patch are pushed down hard; something that looks different there (a robot driving over the stain)
  is still found. Generalises the tracker's existing "rejected spots" rule (which only hides a new
  track until it moves away).
- Rejected guesses are kept as "this is not an object" examples for training (hard negatives).
- Generic: works for any stain, reflection or mark. Truly permanent exclusions stay possible only
  through the work area (outside the area), which is where a shelf light or a reflection belongs.

## Standing rule: assume nobody reads first (Nick, 2026-10-20)

"We need to assume that people are lazy and not going to look for info or instructions before they
start trying to use the program."
- Every action announces what it just did in a short message that fades by itself (no X): what
  changed, where, and how to take it back. Examples: a Jump in Investigate: "Jumped to frame 6150
  (4:12). Investigation closed: press Q to go back to it." Z: "Undid: moved #71 on frame 9310
  back to where it was. Shift+Z puts it back." (Needs a redo: Shift+Z.)
- Check every message against the rule before shipping: each one must say what changed, where, and
  the exact key or button that takes it back (Nick caught an example missing the last part).
- Nothing that changes saved work happens without saying so; nothing closes or moves the person
  elsewhere silently.
- Every Investigate jump keeps a one-click way back to the investigation.

## Board edge vs shadow (Nick, 2026-10-21, E02 setup)
The roam1-based calibration's left (+y board) line sits on the board's shadow, not the plywood's
edge: at the bottom-left corner lit plywood is visible outside the green line (a few px, ~1-3 cm).
Next build: find edges on the lit plywood side (brightness step from plywood to anything darker,
shadow-aware), cross-check with the robots' bump positions against the boards (physical truth), and
let the person drag a corner to fix it in place in the outline check.

- (2026-10-22) Tracker ID switch on the synthetic crowd recording when the measured object size is 30.9-31.5 px (none at 30,
  30.5, 30.8, 31.6, 32): a sensitive spot in the tracker's matching. test_accuracy_crowd is marked "expected to fail" until
  it is fixed in the tracker (not by tuning the size).
