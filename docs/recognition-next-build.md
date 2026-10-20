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
- Tracker headings show as solid and were saved as known when Enter was pressed: they should
  show dashed (a guess) unless the person sets them.

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
