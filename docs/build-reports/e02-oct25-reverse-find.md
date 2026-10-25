# Build report: the find backs into the boards (firmware 2026-10-25b)

**Premise.** Nick, 2026-10-25 09:16: "one attachment in total with switches in the back". The two bumper switches,
the ID lights and the charging pads all sit on one attachment at the robot's back. The front distance sensor and
the front line sensors (the edge guard) stay clear, robots already back into their parking spots, and the dock is
entered backwards. So the find has to locate each board by backing into it, and the dock and the touch probe have to
work backwards too. The geometry comes from hardware/dock/build_attachment_v3.py (being built by another agent; not
committed when this was written): the rear bar is the nose bracket mirrored, its face 6.10 cm behind the axle.

**What changed.**

| Before | Now (firmware 2026-10-25b) |
|---|---|
| Find: nose first, bare front face, touch / stall / jolt cues | `e02_find_reverse 1`: face the board, **look** (the front sensor must see it where the map says), turn round, back in; a back switch clicking is the touch (stall / jolt still count); push backwards until both switches are pressed; leave by driving forward |
| Backing in the find was only ever a short retrace | The backing creep is capped: 3 cm (+ 20% of the travel since the camera fix) past where the board should be, never with the axle within 2 cm of the board's line ("no board behind") |
| A missing board: the edge guard caught it (nose first) | Backing in, the look catches it before any backing ("no board seen ... (not backing in)"); the cap is the second layer |
| Dock, touch, posts: nose switches only | `sw_back 1`: they creep backwards, pivot mirrored, leave forwards; the docked pose is the wall + `e02_back_face_cm` along the way it faces (out of the dock); follow step K and P too |
| Turn between the boards measured as a wrapped angle | Added up unwrapped: turning round can wind the gyro past 180 degrees (found here: turn_k 0.83 and a 5 cm error in the simulator) |
| FIND record | + `reverse` column (1 = backed in); `face_cm` is the back face then; BUMP `cue` `switch-L/R/LR`, `squared` 2 = both switches pressed, `front_cm` = the look's reading |
| Runner: firmware 2026-10-22b / 25a | + 25b; the robots' own border and `sleep` work on 25a and 25b; `--switches-back [ROBOTS]` sets `sw_on 1`, `sw_back 1`, `e02_find_reverse 1`, `e02_back_face_cm 6.10`, reads them back, and sends them again before every find |

Without the switches set up, or without a pose (no camera fix since power-on), a robot asked to back in finds nose
first as before and says so (FIND `reverse` 0; the runner prints a line). Defaults are 0, so robots without the
attachment are unchanged.

**Data (simulator; the switch and contact models are guesses until the real attachment is on camera).**

- Reverse finds, camera fix first, gyro corrected (as the robots run finds): 20 of 20 succeeded, worst reset error
  0.22 cm (10 with the boards at the low ends, 10 at the far corner, spots all over the sheet, any heading).
- Gyro uncorrected (7% short): 13 of 20 succeeded within 0.30 cm; the other 7 stopped with "no board behind". The
  extra turns leave the pose several cm off along the board, more than the tight backing cap allows. Every failure
  was this safe one. Nose first, the same robots tolerate it because their margin is 25 cm, not 3.
- A removed board over an open edge: the look reads the room beyond (139 cm against an expected 20) and the find
  stops before turning round; the robot never got nearer the edge than the staging point (25 cm). With the look switched off,
  the cap stopped the axle 2.08 cm from the line (`e02_find_keep_cm` 2).
- Docking backwards, 12 starts 1-8 cm out, up to 8 degrees crooked: all docked; axle within 0.3 mm of
  `back_face_cm` from the wall, sideways within the pocket's play, heading within 0.4 degrees.
- Tests: `pytest tools/experiment/tests tools/sim/tests`: 414 passed, 17 skipped. New: 7 reverse-find tests
  (test_e02.py ReverseFindTests), 4 back-switch dock / touch / follow tests (test_dock.py BackSwitchTests), 1 runner
  test (--switches-back). Firmware syntax check (tools/fwcheck) clean.
- Two older nose-first tests failed at first, with no logic change behind them. The seven new parameters lengthen
  the run-start PARAM dump, which shifts the simulator's random draw (the old code with seven dummy parameters fails
  identically). Far-corner find, seed 606: 1.45 cm against a 1.0 limit; 30 other seeds give median 0.11, max 0.30.
  Route 1 far point: 2.51 against 2.5; 20 seeds give median 1.6, 90% 2.8, max 4.5. The limits are now 1.5 and 3.0,
  with this spread written beside them.

**Conclusion.** In the simulator the backing-in find is as exact as nose first (under 0.25 cm with corrected gyros).
It is safe against a removed board, because the robot looks before it turns its back. It is less forgiving of a bad
pose: with an uncorrected gyro about a third of finds stop with "no board behind" and need a fresh camera fix
and a retry. On the robots, finds run with each robot's learned corrections, so this should be rare. The first
session will show it.

**What's next.**
1. Flash 2026-10-25b to each robot (it changes nothing until the settings are on).
2. When the v3 attachment is on: tape-measure the axle-to-back-face distance; if it is not 6.10 cm, change
   `BACK_SETTINGS` in tools/experiment/e02.py and the params.h default.
3. Check each back switch with `run touch` (docs/home-experiments.md, "Finding itself against the boards").
4. First session with `--switches-back`: 10 finds per robot on camera, plus the look DOE (doe-backlog.md, "Reverse
   find").

## Sweep 1: new tests that may now be needed

- **Real look readings.** At the staging point, does the front sensor see each board steadily at ~20 cm? Boards
  must stand tall enough for its middle zones. Lift a board out once: the find must stop at "no board seen".
- **Back switches on the real board.** Do the levers click before the bar's corner touches when the robot backs in
  5-10 degrees crooked? (Simulator: the corner touches first and the pivot brings the lever on; once, with the cap
  nearly used up, it ran out of travel first.)
- **Rear caster at a removed board.** Hand-hold test only: how far past a plywood edge can the back go before the
  caster drops? This decides `e02_find_keep_cm` (question below).
- **Turning round near a board.** Only when a board is further than 30 cm (no staging point): the back corners
  sweep ~8 cm. One run from far away with the staging point off.
- **Parking and KC with the attachment.** Routes and parking are unchanged, but the robot is now 6.1 cm long
  behind the axle (was 4.75). Park spots and the routes' room behind the robot may need the longer body.
  `board.face_cm` is raised to it after a backing-in find; the fleet map's `bumper_cm` is a front number.
- **Fleet plans with dock steps.** The firmware's dock step K now backs in when `sw_back` is 1, taking `h` as the
  way the docked robot faces (out of the dock: v3's map convention). The fleet planner (tools/fleet) still plans the
  approach to the dock nose first. It was not changed here; a plan with dock steps needs a backwards approach first.
- **Real toolchain build.** Like the last builds, this was compiled only with the stand-in headers (tools/fwcheck).

## Sweep 2: possible inaccuracies in what this relied on

- **`e02_back_face_cm` 6.10 is a design number** (v3 rear bar = the front bracket mirrored: 47.6 + 13.4 mm, with
  the shell's back taken as far behind the axle as its front). Not measured. A 5 mm error moves every reset by 5 mm.
- **Chosen numbers** (DOE rows in docs/doe-backlog.md, "Reverse find"): `e02_find_see_cm` 20,
  `e02_find_back_margin_cm` 3, `e02_find_keep_cm` 2, `e02_find_turn_cm` 30. The look's timing (5 frames, median)
  follows the sensor's frame rate; no DOE.
- **Where the boards stand.** The cap assumes a board's face is at or inside the plywood edge. If boards stand
  outside it, a removed board leaves even less room behind.
- **Simulator guesses.** The switch model (lever gap 0.2 mm, bounce), the contact and stall model, the
  flat-ray distance sensor (the look sees anything in its plane), and the 7% gyro error are guesses. The
  outline's back is the front bracket's shape mirrored; the real v3 bar and the caster's holder may differ.
- **Left and right on the back attachment.** The firmware takes the switch on A0 as the robot's own left. If the
  v3 wiring swaps them, only the `cue` column's L/R and the pivot direction change; the pivot then turns the wrong
  way and docking fails ("switch never clicked"), which is visible at once.
