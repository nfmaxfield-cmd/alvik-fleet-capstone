# Build report: recognition stage 2 (2026-10-20/21, overnight)

**Premise.** Nick: "Build as much as you can without [the real footage]." Smarter tracking first; going
past 90% of the weekly meter allowed. Plan: `docs/build-plans/recognition-stage2.md`. Real footage
(roam1, robot logs, Nick's labels) arrives as the GitHub release `roam1-data`; everything below was
built and tested on synthetic recordings and flagged where real footage must confirm it.

**What was built** (32a05b7..a8776a0):

| Part | What a person gets |
|---|---|
| Smarter tracking | Objects followed by how they look when they stop moving: a robot parked all session stays found on every frame with no drift; a hand passing over breaks the chain and flags the frame; lookalikes that cross are told apart by how they moved and looked, or flagged for the person; per-type speed and "can stay still" settings; "How often should I check?" measured from your labels |
| Rejections inform | "Reject here" replaces "Ignore this spot": rejects in that frame, remembers what the empty spot looks like, keeps finding anything that looks different there; rejected guesses teach the model "not an object" (a synthetic stain: 8 false finds to 0) |
| Every action announced | Each action says what changed, where, and how to take it back; Shift+Z redo; a test fails the build if a message lacks the take-back part |
| Investigate | Three separated parts (what we found / look around / change things), every change previewed with its reach and confirmed; Q returns after a jump; clip key C |
| Positions logs (robot logs, any outside data) | Draft guesses with facing from a log on the recording's clock, re-aligned at every confirmation and home visit; clock offset found automatically or "line up by a moment" |
| Floor calibration | In the studio: click a rectangle's 4 corners and type its size (or measured points); grid drawn on the floor to check; fit error in cm |
| Home positions | Fixed spot, spot with a facing, tray, the Alvik dock: a visit by the right object is a free confirmation |
| AI helper | Off by default; own key kept on the computer, never in project or shared files; Anthropic or any OpenAI-style service; "object or background?", name an object, second check, draft labels, setup from a sentence; every call logged; suggestions only |
| Training | Hard negatives; copy-paste with an honest compare (decided on validation, scored on test once); report by situation; a pre-check that refuses with counts instead of failing |
| Live use | Roll call at the start, per-object confidence and "last confirmed", a prompt to re-confirm one object, names in the CSV |

**Checks:** suites 404 passed, 2 skipped (slow training tests, each passed on its own), plus 93 in
tools/tests; old projects from both earlier versions open with every saved label unchanged through
every new feature (two fixtures); a browser click-through and a newcomer test (a 3D-printed-shapes
team) found 1 blocker (floor calibration needed programming) and 17 should-fix items; a code review
found 8 (home visits trusted any still object; hard negatives could land on real robots; compare chose
on test frames; a clock-offset crash; an unlocked background read; AI key exposure through an
imported project file; saves blocking ~5 s while re-tracking; AI counting). All fixed in two rounds and
re-tested; re-track after a save now holds the studio 0.16 s (was 5.6 s).

**Conclusion.** Stage 2 is built and checked on synthetic data. What it most needs now is roam1.

**What's next.**
1. Nick uploads `roam1-data`; a real-footage session tunes the tracker's numbers, measures how often a
   person must check, lines up the robot logs (gyro scale, clock offset), and checks speed on 16 minutes.
2. Known open issue: "Line up again" can re-match log names to new object numbers (seen on synthetic
   roam data); look at it with the real logs.
3. Nick's AI key for a real test; personalisation for the other team once they answer.

**Cost.** Subagent tokens about 4.0M (lanes 2.0M, integration 0.5M, checks 1.0M, fix rounds 1.0M),
plus the main session's. Meter before: 72% (Nick, 20:45 ET). After: to be read by Nick.

## New tests that may now be needed
- All of the above on roam1 and Nick's labels (the tracker's ~20 chosen numbers; check-in interval).
- Robot-log alignment on the real logs: heading reset per run, gyro scale (~1.08), clock offset.
- Line up again keeping names on the same objects across re-runs.
- The AI helper against a real service (answer format, cost figures).
- Windows: the key file's protection, launchers, floor calibration clicks on a real board.

## Possible inaccuracies in what this relied on
- Every check ran on synthetic recordings, which are cleaner than the basement footage.
- Chosen, not derived: the tracker's thresholds (still match 0.90, chain break 0.50, split decision
  1.5 s, chain half-life 300 s, ...), the 750-frame immediate re-track window, hard-negative weight 3x
  and 0.5-size exclusion, copy-paste's 1-point gain rule, situation rules (1.5 sizes, 2 s), AI prices
  per token, home visit radius and stillness time.
- Log-alignment accuracy (about 0.6 cm with regular anchors, 7 cm with only a start anchor) comes from
  a synthetic session with modelled gyro and wheel errors.
