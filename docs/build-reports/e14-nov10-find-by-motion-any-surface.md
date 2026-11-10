# e14: robots found by motion, calibration on any surface, readability by light and tag, pooled calibration (10 Nov, simulator and logs only)

## Premise

Seven items from the coordinator (10 Nov, from today's school runs with AL-EB54 and tag 4), then three mid-build
additions: pool calibration evidence across runs (Nick 16:40), the setup order for any surface (Nick 16:40; polygon
tables dropped at 16:45), and the school 16:22 bug (parking ended the run when the tour's last point lost the tag).
Nothing here touched a robot, a Pi, a camera or Tailscale. No RFID data. **Measured** = simulator, real-path practice
(`--practice-real`) or tests, the file named. **Claude's reading** = interpretation, marked.

**Nick's question at 15:23** ("Is the calibration built specific to this table?"): yes, until now. From the code:
`calib_run.Site` read the typed sticker points (P01-P20, zones) through `sticker_points.points_cm`, which crashed with
no points file; the park spots (P11, P01, P09), the tours (zones 1-2) and the outlet plates were all the school's. On a
new surface it would not have known what to do. Item 2 fixes this.

## Data

### 1. Every robot found by its own small move, anywhere (done; on by default)

`motion_find.py`, used by the calibration's start, the missions' binding (`wiggle_bind.bind_for_run`, so Visit points,
experiments and school free roam) and the practice runs:

1. **Clear space**: the derived clearance (e13's: 19-21 cm round each robot) is asked on Mission Control's Start
   button (OK: strokes; "not sure": each robot wiggles on the spot, which needs only its own footprint) and in a
   terminal; a robot whose pinned tag shows a stroke would not be clear wiggles anyway.
2. **Every camera** (`camera_discovery.find_cameras`); a camera that sees no robot is left out and said.
3. **One at a time**: a 6 cm stroke forward and back, or a +-30 deg wiggle. The motion gives its position (and from a
   stroke its heading); the roof tag whose reads moved with it is its tag (bound, no wiggle needed). A refused stroke
   is said as the robot's own words.
4. **Bring back**: a robot found where no tag moved with it drives in 5 cm steps, each step itself a motion sighting,
   towards where tags read, under the never-give-up rule (error sum + half the body + stop inside every edge; another
   robot or a keep-out in the way: another target). When the rule blocks it: a find-me wiggle, then NEEDS A HAND; never
   a timer. Once a tag reads near it, the wiggle binding confirms it.
5. **No tag at all**: a robot standing where the EVIDENCE proves tags read (earlier runs, the other robots' tags read
   during the finding), with no read at each such spot, is called tagless once the chance of missing a tag everywhere
   is under 1 % (Laplace read rates); it then runs on motion, lock-on and odometry (item 6).

| Simulator case (rig, `motion_find.run_rig`, school camera drawn at 1280 x 960) | Result (measured) |
|---|---|
| Two robots 25 cm apart (Alpha, Bravo) | each found by its own stroke, its tag bound; 2.2 / 1.9 cm from the truth |
| A robot at P15 whose tag never reads there (Charlie) | found 0.6 cm off; 6 steps, 6 sightings back to where its tag reads; ended 2.4 cm from the truth |
| A robot in a dark patch (pictures at 35 %, no tag reads) | found 2.6 cm off; 10 steps back; ended 0.6 cm off |
| An extra camera looking at the wall | left out ("SEES NO ROBOT") |
| Position error over 30 random spots | median 5.9 px (1.5-2 cm), p95 11.4 px: taken as the sightings' +- (6 px). A heading only from a stroke whose own picture was found again (template match): the changed area's drift pointed 100-130 deg wrong in 3 of 30 |

![rig](../screenshots/2026-11-10/e14-feature-rig-found-by-motion-and-brought-back-chart.png)

Each robot's true start (x), where the motion found it (circle), its drive back (line) and where its tag read (square).

**School calibration, robot at P15 with an unreadable tag** (`calib_run --dry --quick --sim-start P15 --sim-gap-at
179.7,150.5,25`, session `c4`): the simulated person did not confirm the clear space (P15 is 17 cm from the edge, the
clearance 21 cm), so it **wiggled**; found at (178, 149) +- 2.9 cm; a 3.0 cm stroke for its heading (as long as its
error sum allowed there); 9 steps, 11 sightings, 42 s; tag 2 read at (171, 128); the pin check bound it; the
calibration ran on and finished (gyro_scale 1.0290). Measured.

![P15](../screenshots/2026-11-10/e14-feature-calibration-p15-unread-tag-found-by-motion-and-brought-back-terminal.png)

Found in rehearsals and fixed: a woken practice robot refused its stroke (edge guard off): the guard is now sent
first and a refusal is said; a robot circled a target in a dim patch 0.5 cm at a time (a reached target is now not
tried again); a robot held beside a parked robot (another target now); a robot WITH a tag was called tagless in a dim
patch the camera model calls readable (now only proven spots count; 3 of 3 reads is 0.8, not certain).

Switches: `MIRLT_CAMERA_DISCOVERY=off` (the wiggle binding alone), `=e13` (e13's separate step).

### 2. Calibration on any surface (done)

`calib_run.spots_mode`: the typed stickers when present AND proven (a points file of stickers, every point on the
table), else **its own spots**: inside the table and the readable area, at least 25 cm from the edges (the margin + 8,
so no point needs the near-edge staging) and from every keep-out it knows, spread out (each the candidate furthest from
those taken), 25 cm or more apart, labelled S01..., the park spots chosen among them (the readable middle first).
Switch `--spots` / `MIRLT_CALIB_SPOTS` / site `calibration.spots` (auto, stickers, own). The school keeps its stickers.

| Dry run | Spots | Result (measured) |
|---|---|---|
| A site with **no points file** (`bare-1110`, 182 x 91 cm) | 9 own spots, park S05 / S09 / S01 | finished: wheel_scale 0.9657, gyro_scale 1.0317 (true 0.97 / 1.03); distance error 2.89 -> 0.34 cm/m |
| The sim's **new room** (`site_detect` new-1110: its points are a grid, not stickers) | 9 own spots | finished the same way; its report first crashed on Jay-Sun's four points (fixed: left out when they are not there) |

![own spots](../screenshots/2026-11-10/e14-feature-own-spots-on-a-table-with-no-stickers-chart.png)

The blue and orange dots are the spots the calibration chose on a table with no points file; green is where tags read.

### 3. Readable area by light and tag (done)

`readmap_store.py`: each run's OWN evidence (not its seed or carried-in history) kept as one session with its light
(the bridge's mean grey) and its tag, in `<conf dir>/readmap_store.json`; never overwritten or halved. At a run's start
the map is chosen for the light now (the bridge, or a few pictures from the recorder) and each tag: another light's
sessions fall off as exp(-0.5 (dgrey/8)^2) (grey 70 against 91: 3 %), another tag's at half weight, one session at most
10 tries a cell. The old `readmap.json`, e13's light maps and per-tag maps are carried in once. Free roam stays where
EVERY tag of the run reads at today's light (`readmap_roam.json`). The Start screen's Area line now says what is used:

![area](../screenshots/2026-11-10/e14-feature-start-screen-area-by-light-and-evidence.png)

Measured (test): four lit sessions reading to x 220 cm and one long dim session (tag 4) reading to x 30 cm: the lit map
still reads to x 220 (tag 4 too), the dim map for tag 4 to under 60 cm, and with the light unknown still over 200 cm.
Practice (school Visit points, shared store, d1-d7): sessions kept per light ("2 sessions at about this light ... 3 in
other light kept apart"). Dim calibration (grey 70) with tags unread past x 90 (c7e, then c7g on the final code): in c7e the tour points past x 90 were moved to the end once and then held a robot the simulator could not resolve (stopped by hand), which led to the hold's own motion sightings below; c7g finished (see D/F). Switch
`MIRLT_READMAP_STORE=0`.

### 4. No re-trying a point that just failed (done)

The 4-in-a-row came from `goto`'s aim-again after each recovery. Now a tour point whose tag was lost on the way goes to
the end of the tour once (said, the spot where it was lost added as evidence), and is skipped with its reason the
second time; parking and detours keep the aim-again (it must get back). Missions with "try anyway": the same in
run_plan (`defer_current_point`). e11's skip rules (4 losses, the view limit) are unchanged. Switch
`MIRLT_RETRY_LOST=again`. Tests.

### 5. Battery floor (done)

`engine_checks.battery_floor_used`: `MIRLT_MIN_BATTERY` first (as before), then Mission Control's new Advanced
"Battery floor %", then the site file's `battery_floor`, then the defaults (25, free roam 10); every run reads it (no
tmux needed) and says it: "BATTERY FLOOR: 20 % (Mission Control's Advanced 'Battery floor')". A calibration starting
under 40 % is marked **LOW-BATTERY CALIBRATION** in its report. DOE row added.

![floor](../screenshots/2026-11-10/e14-feature-advanced-battery-floor.png)

### 6. No-tag robots at school (done in practice)

The binding records a tagless robot with its motion position and heading; run_plan starts it there (`--start-pose`),
the tag bridge's motion detector is seeded there (`--seed-motion`, and a "seed" message), and its name gets a tag id no
tag carries. Practice, school Visit points, 3 robots, Charlie with no tag (`--practice-real --sim-tagless Charlie`,
v4): found by its stroke, driven to a second readable spot, called tagless, started at (110.1, 109.5, -81 deg); its
three points P11 / P12 / P13 were reached **1.4 / 0.3 / 0.2 cm** from the simulator's truth (stated +-2.4 cm, motion
only). With the final code and earlier lit evidence in the store (v9): Bravo, found at x 243 beyond the camera's reach
for tags, was driven back 18 steps (19 sightings, 99 s) until tag 1 read; Charlie was called tagless after 2 proven
spots and started at (49.9, 70.1, 71 deg); every point within 0.3 cm of the truth except Charlie's P11, **2.5 cm, just
outside its stated +-2.4** (measured). With no evidence in the store (v7, a fresh site) Charlie could not be called
tagless: the only proven spots were beside the other robots, and it held and asked for a hand, as designed.

![tagless](../screenshots/2026-11-10/e14-feature-visit-3-robots-one-tagless-terminal.png)

**What is still unsafe (Claude's reading):** a tagless robot's heading after its start comes only from its gyro and
its motion's direction of travel; a robot standing still for long in a dim area has no source at all and would be held
by the never-give-up rule rather than lost. School free roam with a tagless robot is not tried (free roam has no start
pose input yet).

### 7. Detector training (not done)

`download.pytorch.org` is refused by this machine's network policy (403). PyPI's torch is a CUDA build (539 MB) that
also needs about 3 GB of nvidia packages, over the disk budget; the download of the wheel alone did not finish in
20 min and was stopped. Nothing left on disk.

### Added mid-build

**A. Calibration evidence pooled across runs** (`calib_pool.py`; Nick 16:40). Every phase-A leg of a robot (WiFi name)
on a site, finished or partial runs, kept as raw quantities (so legs with different values in force pool), weighted by
age (half weight after 7 days) and battery (full from 40 %, logged per leg). A run starts from the pooled values,
drives only what is still uncertain (turns, still period, rolls each skipped when its pooled +- is already as narrow as
one finished run's), and a run that ends early still adds its legs and saves the pooled values once they are that sure.
The "as sure as a finished run" line is DERIVED: phase A's own design through the same least squares at the pool's own
noise. Test (synthetic legs, 30 seeds): three partial runs, each a third of phase A with different values in force,
reach the full run's values (means within its run-to-run spread; spread at most 1.8x, from three short still periods).
Practice (`poolstore2`, simulator true values 0.97 / 1.03 / 0.30): runs 1-3 each stopped by hand at 150 s; run 2
started from the pooled values, run 3 also left out the turns (the pooled gyro already as sure as a finished run's);
run 4 started from 3 runs' pool, drove no turns, fitted **wheel_scale 0.9713, gyro_scale 1.0301** (saved), drift
0.2752 +- 0.021 not yet as sure as a finished run's (+-0.015), kept. Measured. Found: pooling staging and tour legs
widened the wheel's +- 3x (camera-aimed legs): only phase A's legs are pooled. Switch `MIRLT_CALIB_POOL=0`.

**B. The setup order** (Nick 16:40/16:45): every run's start says "SETUP ORDER: 1 the table's 4 corners (... confirmed
/ not), 2 clear space, 3 each robot moves and is found and named"; step 2 is the Start button's question, step 3 item 1.
Step 4 (each robot's path, a crossing warning in space and time) is written into
`docs/build-plans/mission-control-redesign.md` (3c) with a mockup; any number of corners is noted there as a future idea
only (polygon work started and reverted, nothing committed).

![step 4](../screenshots/2026-11-10/e14-mockup-step4-each-robots-path.png)

**C. School 16:22** (AL-EB54, P16, "ENDED: the roof tag is not read while parking"): the tour had ended, `park()` set
`_ending`, and both the camera wait and the hold ended at once by design (e13: "parking still gives up"). Now parking
holds with the never-give-up hold (find-me wiggles, motion sightings) like any leg (`MIRLT_PARK_GIVEUP=1` e13's end),
and a recovery with no reliable spot yet heads for the nearest spot the readability map (this tag's, today's light) says
reads, instead of "no reliable spot known". Tests (a stand-in eye: parking held, then carried on when the tag read; the old end on its switch). Practice
(`--visit-order P10,P16`, the tag unread near P16 for 60 s, c9): the tag was lost on the way to P16, the recovery
brought it back in 5 s and it parked 0.4 cm from P09; the exact school case (the tag lost DURING parking) is shown by
the test only.

**D. The picture went dark** (Nick at school 17:2x: lights off, the picture black, the run ended and the robot slept;
minutes later the camera had adjusted). `dark_watch.py` in the tag bridge, on every picture: under 35 % of the camera's
own normal brightness (or under grey 12) while pictures still come = "THE PICTURE WENT DARK: the camera is adjusting to
the light" (the dim room's 70 of 91 is NOT dark); back at 60 % steady for 2 s, with how long it took
(`dark_events.csv`: the light drop against the time to adapt, a DOE row). Every run holds every robot: the calibration
sends `stop` and waits (no legs, no recovery drives, no wiggles, no end, no sleep; a heading check cut by the dark is
done again; a run that would end waits for the light first), run_plan pauses every robot, free roam holds every robot;
all carry on by themselves. The idle sleeper does not count dark time (`<conf dir>/camera_dark.json`). Optional:
`camera_exposure.py` nudges the exposure through v4l2-ctl (manual, up to 8x, gain x1.5, then back to auto), **off**
(`MIRLT_EXPOSURE_NUDGE=1`): the simulator has no model of a real camera's exposure, so it cannot show the nudge does no
harm; a board DOE decides. Tests: the watch on a synthetic minute of black (adapt 38.8 s logged), the nudge with a
stubbed v4l2-ctl, the sleeper, the calibration's hold. Simulator (`--sim-blackout 130:60`, c10, measured): the picture went dark 83 s into phase A while a leg was running; the robot held 60 s, "the picture is back after 61 s: carrying on", seen again 0.2 cm from its odometry, the tour ran and it parked 0.2 cm from P09. gyro_scale 1.0314 saved; wheel_scale and drift kept old (their +- wider than a finished run's, since the dark cut legs short): the pooling's rule doing its job.

The calibration's never-give-up hold now also watches its find-me wiggle in every camera's picture (the robot finding's
own analysis), so a robot held where its tag does not read is found by motion and the recovery tried from there
(before, only the bridge's motion tracker, seeded by tag reads, could see it). Dim calibration (c7f, then c7g after F, measured): c7f ended at the pin check (F); c7g: found at (148, 82) where its tag does not read, brought back 18 steps (18 sightings, 41 s), bound by motion, parked 1.5 cm from P09; P14/P17/P12 skipped as unreadable (3 tries, 0 read), P20/P13 moved to the end once then skipped, parked 0.3 cm from P09. gyro_scale 1.0296 saved; wheel and drift kept old. Its before/after check (0.13 to 2.82 cm/m) rests on the few readable points only: not a judgement of the fit (Claude's reading).

**E. School 17:02 (AL-EB54): the "while parking" end outside parking, and wheel_scale 0.73 +- 0.20.** The
`_ending` end hit twice where no parking was going on (the curve phase; Tour C1's first step). Root cause, found in
the code: `stage()` parks the robot at the run's START with `park("stage")`, and `park()` set `_ending = True` and
never cleared it, so from staging on every lost tag counted as "lost while parking" and ended the run (C's fix made
the end's hold never give up, but the flag was still wrong). Now only the run's own end (`park("park")`) sets it, and
it is put back when parking returns. Test: `park("stage")` leaves it False. The wheel_scale: that run's legs are not
on this machine, so this is **Claude's reading**, not measured: the run ended early by that bug, so its rolls were few
and some cut short or driven while the tag was half-read; a fit on a few short, scattered rolls gives exactly a wide
+- (the refusal was right). The pool now treats such a run carefully: each run's legs are weighted by that run's own
scatter (its residual sigma against its own fit; a run over 2x the typical run's gets weight (typical/its)^2), and
legs over 5 sigma from the pooled fit are dropped. Test: two good synthetic runs plus one run at 0.73 with 8 cm
scatter: the pooled wheel_scale stays within 3 se of the good runs' (the bad run's weight under 0.05); the 3-partial
test still passes. Measured (test).

**F. A robot brought back is bound by motion** (rehearsal c7f, found here): a robot found where its tag does not
read was brought back until tag 2 first read at the motion detector's spot, and then the old pin-check wiggle ran at
the dim border, saw the tag stand still and ended the run ("PIN CHECK FAILED"). The first read at the tracked spot,
while that robot alone moved, is the binding evidence now; the note says so.

### Tests

`tools/fleet/tests/test_e14.py`: 32. Two e12/e13 tests pinned to the old ways' switches. Full fleet suite on the final
head: FULL_SUITE.

## Conclusion

- A robot is now found by its own small move anywhere on the table, with or without a tag that reads, and driven back
  to where its tag reads without a hand (simulator and practice); the school's P15 case runs through to a finished
  calibration in the simulator.
- The calibration no longer needs stickers; on a table with no points file it makes its own spots and finishes.
- One dim session can no longer shrink the lit map; the light and the tag choose the map, and the Start screen says so.
- A school Visit run with a tagless robot starts, tracks and finishes in practice, its points within 1.4 cm.
- Partial calibrations now add up instead of starting over.
- The school's "while parking" end (16:22, 17:02) came from staging, not parking; fixed at its root.
- A dark picture is now a camera problem: every run holds and carries on by itself (simulated 60 s blackout).
- A run with wide, scattered legs (17:02's wheel 0.73 +- 0.20) no longer moves the pool much.
- Not done: detector training (blocked download); free roam with a tagless robot.

## What's next

- Nick at school: one calibration with AL-EB54 started at P15, Start's clear-space question answered "OK"; watch for
  "FOUND BY MOTION" and "BRING BACK".
- `python3 tools/fleet/calib_pool.py show AL-EB54` on the school Pi after the next calibration (it pools from then on;
  today's earlier partial runs are not in it: their legs.csv could be imported, a small script).
- Nick's choice of the Mission Control direction, then step 4.
- Training on the laptop (PyTorch there).

## Appendix: switches (all decided automatically; the old way kept)

`MIRLT_CAMERA_DISCOVERY=off|e13`, `MIRLT_CLEAR_SPACE_OK=1`, `--clear-space`, `--spots` / `MIRLT_CALIB_SPOTS`,
`MIRLT_READMAP_STORE=0`, `MIRLT_RETRY_LOST=again`, `MIRLT_CALIB_POOL=0`, `MIRLT_PARK_GIVEUP=1`, `MIRLT_EXPOSURE_NUDGE=1` (off: the nudge); Advanced "Battery floor";
simulator: `--sim-blackout T:DUR`, `--sim-dark`, `--sim-extra-camera`, `--sim-light`, `--sim-start X,Y`, `--sim-tagless`, `--readmap-store`.

## Sweep 1: new tests that may now be needed

- On the robots: the finding stroke at the school camera's far end (P15) and the bring-back with the real motion
  picture; its sightings against tag reads where the tag reads (sets MOTION_PX_SIGMA).
- Two robots 25 cm apart at school, and a robot with its tag covered, then uncovered (the tagless verdict).
- The Start button's clear-space question answered "Cancel": every robot wiggles, none strokes.
- A real dim session (lights off) then a lit one: the Start screen's Area line, and the lit map unchanged.
- Calibration on a different real surface (home board) with no stickers: own spots, the fit against a later free roam.
- Pooling on the robots: two calibrations stopped early, then one more; the pooled values against a full run.
- A real calibration with the tour's last point losing its tag: parking holds and returns.
- Battery floor from Mission Control's Advanced reaching a tmux-started calibration.

## Sweep 2: possible inaccuracies in what the work relied on

- **Simulator, not robots**, for every movement result; the motion analysis ran on drawn pictures (tag_cam), not the
  C920e's.
- MOTION_PX_SIGMA (6 px) is measured on drawn pictures only; the e13 rig's steep cameras gave under 2 px.
- **Chosen numbers** (DOE rows added): MIN_STROKE_CM 2, TAGLESS_P 1 %, WIGGLE_DEG 30, OWN_SPOTS_MIN_CM 25, LIGHT_SCALE 8
  grey levels, OTHER_TAG_W 0.5, SESSION_CAP 10, LOW_BATTERY_NOTE_PCT 40, AGE_HALF_DAYS 7, BATTERY_FULL_W_PCT 40.
- The simulated "person" confirms the clear space from the truth; a real person may say yes where a stroke is not clear.
- The tagless verdict trusts the readability evidence's light matching; a tag that reads poorly for its own reasons
  (bent, dirty) at a well-lit proven spot would be called tagless and run on motion (said loudly; its tag still binds it
  later if read).
- The light used at a calibration's start is the bridge's brightness so far (a few seconds); a light change mid-run is
  kept under the run's median.
- Pooling assumes the robot's wheels and gyro do not change between runs beyond the age weighting; a firmware or tyre
  change would need the pool cleared (not automatic).
- The full-run threshold uses phase A's rolls only (not the tour legs a finished run also fits), so it is a little
  looser than a real finished run.
- run_plan's deferral moves a point's goto and dwell together; a point shared by two robots' lists is deferred per robot.
