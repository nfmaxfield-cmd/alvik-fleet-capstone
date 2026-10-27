# Two-camera tracking for the ladder (2026-10-27, MIRLT)

**Premise.** At 11:20 `./run-fleet-pi.sh ladder pause` stopped at L0. The overhead camera found all three robots by
their lights, but the tracker never saw their bodies: the board sits under the lamp and is washed out (board 193 of
255; robot tops 14-24 grey levels from it for two robots, 0-3 for the third). Nick: "This is why I wanted to use both
cameras in conjunction with each other." Built and tested on the simulator and last night's recording; the Pi, the
cameras and the robots were not touched.

**What was built** (commits 0f1e998, c0b3b28, 952bf7b, plus this report's commit).

1. **Side camera calibration on the setup page** (`./run-fleet-pi.sh setup`, part 2). The page finds the side
   picture's 4 board corners, or Nick clicks or drags them. If not all 4 are in the picture, the page fits from the
   plywood grain alone. The corners are numbered like the overhead's (1 = the floor's 0,0) using points the
   overhead's calibration already knows: the plywood grain matched between the two pictures, and the robots' lights
   from the flash check. Of the 8 possible numberings, the one that puts those points where the side picture has them
   wins, and it must beat the next best by 2x. Check draws the grid and says how far the points land. A wrong
   numbering lands about 150 px off and gets a red warning; a right one lands about 0.5 px off. Save writes
   `~/.config/mirlt/calib/side_floorcal_<time>.json` (with the flash check's side lag) and a pointer
   `~/.config/mirlt/side_calib`, like the overhead's.
2. **Side camera exposure.** Every exposure lock or auto setting now switches automatic exposure on first. It then
   waits until the brightness stops changing: three 0.4 s readings within 3 %, 2-8 s. Before, the camera had a fixed
   2 s warm-up and a lock could stay on from an earlier program. A brightness check warns under 50 or over 215. The
   recorder locks the side camera this way (`--side-exposure lock`). The setup page settles both cameras when it
   opens and shows the side camera's brightness.
3. **Two-camera tracking** (`tools/experiment/e02_twocam.py`, used by the ladder and `cam_bridge`). Each camera keeps
   its own memory of the empty floor. Each camera's outlines go to the floor and are matched to one set of robots by
   prediction (MultiTracker's rules). A robot counts as seen when either camera sees it clearly, or when both see it
   and agree within 6 cm. When both see it and disagree, it is marked unsure. The tracker also:
   - takes out the side camera's lag;
   - measures distances with the side camera's line-of-sight stretch taken out;
   - learns the side camera's offset from the overhead while a robot stands still, but only from whole overhead
     outlines;
   - does not let the side camera judge a robot that is outside its picture or in line with another robot from
     where the side camera stands;
   - uses the side's whole outline instead of an overhead outline that covers only part of a robot.

   **Lights as markers:** when neither camera sees a body, the tracker uses the lights near the robot's prediction,
   moved 3 cm back along its heading. Between steps the ladder keeps every robot's lights on (`led 1 1 1 20000` every
   12 s). During runs, the robots' blink codes do the same job.
4. **L0** checks bodies with both cameras; lights alone still keep a robot out. It also prints which camera sees
   each robot.

## Data

![Which camera saw each robot](../screenshots/2026-10-27/two-camera-feature-real-footage-which-camera-saw-each-robot.png)

Each bar is one robot over 45 s of last night's recording; its colour shows which camera the tracker saw it with in
each picture.

| Last night's footage (45 s, 1,120 pictures; roam1 overhead, side grain calibration, lag 0.05 s) | Left | Center | Right |
|---|---|---|---|
| Overhead alone, as recorded: seen | 84 % | 100 % | 100 % |
| Board washed out (contrast cut to 12 % in software), overhead alone: seen | 0 % | 0 % | 0 % |
| Washed out, both cameras: seen (all by the side) | 0 % (out of the side picture) | 99.8 % | 99.8 % |
| Washed out, both cameras: distance from the overhead-alone spot, median / 95 % | - | 1.85 / 2.27 cm | 2.05 / 3.53 cm |
| As recorded, both cameras: Center seen by both, agreeing / Right by the side or both | | 100 % | 99 % |

The 1.9-2 cm matches the two-camera study's agreement (median 1.9 cm).

![Washed out, side carries](../screenshots/2026-10-27/two-camera-feature-real-footage-washed-overhead-side-carries.png)

Left: an overhead picture with the board washed out in software. Right: the side picture at the same moment.
Circles: where the tracker has each robot. Left sits at the side picture's top edge, so neither camera sees it.
Source: the 00:11 recording.

| Simulator (camera bridge, test cross, 2 robots; error against the simulator's truth) | median | 95 % | line check |
|---|---|---|---|
| Overhead alone, plain board (regression) | 0.47-0.49 cm | 0.88-1.06 cm | pass |
| Washed-out board + side camera (lag 0.06 s), seen by the side | 0.40-0.46 cm | 1.19-1.25 cm | pass |
| Same, first try (offset learned from pieces of robots, averaged in) | 4.0 cm | 4.9 cm | pass, but 4 cm off: fixed |

| Ladder L0, dry run, washed-out simulated board | Result |
|---|---|
| Overhead alone ([screenshot](../screenshots/2026-10-27/two-camera-problem-ladder-L0-dry-washed-out-overhead-alone-all-kept-out.png)) | FAIL: all three kept out, "only its lights" (the 11:20 stop, reproduced) |
| Both cameras ([screenshot](../screenshots/2026-10-27/two-camera-feature-ladder-L0-dry-washed-out-both-cameras-pass.png)) | PASS: all three "seen by the side camera only", within ~1 cm of where they stood |

**Full ladder dry run** (washed out, both cameras, L0-L5;
[screenshot](../screenshots/2026-10-27/two-camera-feature-ladder-full-dry-run-washed-out-L0-to-L5-pass.png)): every step
passed. Every robot's body was seen by the side camera only. The L3-L5 line check p95 was 0.48-1.05 cm (gate 3 cm),
and the start spot checks were 0.2-0.5 cm and 0.3-1.7 deg. The bodies came 9.3 cm apart at the normal-speed cross.
L2 kept Center out at -8.2 deg after four turns (limit 8). That is not the side camera: the same turn check with the
overhead alone on a plain board read -6.6 deg (each turn 1.2-2.1 deg short against the gyro's 89.2). The simulated
Center turns about 88.5 deg, and its total sits near the limit with either camera.

**Tests.** 9 two-camera tests (`tools/experiment/tests/test_e02_twocam.py`): the 11:20 failure reproduced; the side
sees the robots; both cameras on a plain board; lights only; side lag taken out (0.12 s); robots in line from the
side; cameras disagreeing; the line-of-sight stretch; the filled-in floor margin. There are 3 new setup-page tests
(numbering from the grain in any of the 8 ways round; numbering from 3 lights without knowing which is which; the
fake board saved end to end), 3 camrec exposure tests, and the bridge's dry run with a washed-out board plus the side
camera. All pass, alongside the existing suites: Pi 63, fleet 23, tracker and side camera 50.

**Every chosen number (DOE notes).**
- Agreement 6 cm (the study's 95 % agreement). **A DOE fits:** 4 / 6 / 8 cm against "cameras disagree" stops and
  the line check's p95 on the board.
- Side pictures used 10 a second (of its 29). **A DOE fits:** 5 / 10 / 15 against the Pi's overhead picture rate and
  the side's seen-rate on the washed-out board. This matters because the Pi was never tested.
- Side gate +2 cm, plus 0.08 s x speed for the lag's uncertainty. **A DOE fits:** 1 / 2 / 4 cm, side seen-rate against
  identity swaps in the simulator's crossings.
- Offset learning: 0.05 a picture, capped at 8 cm, still robots only, whole outlines only (at least 0.7 of a robot),
  used after 30 pictures. No DOE: these are the settings of a filter.
- Lights: 30 grey levels over the floor memory and at least 235 bright; lights-to-middle 3 cm; gate 10 cm; a sighting
  counts for 0.6 s; position sigma 2 cm. **A DOE fits** the threshold (20 / 30 / 40 against false lights on footage
  with lights on). The 3 cm should be **measured**, not set by a DOE: L0 pictures where the body and the lights are
  both seen give it.
- Filled-in floor margin: measured on the board (95th percentile of the fill-in error), kept within 4-14. No DOE: it
  is a measurement.
- "Hidden" from the side within 1.0 body width of the picture's border; outlines dropped within 0.6. No DOE: these
  come from the camera's geometry.
- Exposure settle: 3 % over three 0.4 s readings, 2-8 s; warnings under 50 / over 215. **A DOE fits** the settle rule
  (time to settle against brightness after locking, from a dark and a bright start).
- Setup page: numbering must win by 2x; a fit whose grain points land more than 15 px off is flagged; grain fit on
  12 cm cells (8 cm took 16 s here). No DOE: these are checks and a speed trade.
- Keeper renews the lights every 12 s (the firmware caps one `led` at 20 s). No DOE: this is a renewal period.

## Conclusion

The side camera sees what the washed-out overhead cannot. On last night's real pictures, with the overhead's board
contrast cut to 12 %, the overhead alone saw none of the three robots. With both cameras, the two robots inside the
side picture were seen 99.8 % of the time, 1.85-2.05 cm (median) from where the overhead alone put them on the
unwashed footage. In the simulator, the washed-out board with the side camera tracked as well as a plain board does
with the overhead (0.40-0.46 cm median) and passed the line check. A robot outside the side picture is still not
seen. L0 says so and keeps it out, instead of guessing.

## What's next

- Today, on the board: setup page part 2, then the flash check (it measures the side lag, which is saved with the
  next side Save), then L0. Nick's steps are in the reply.
- If L0 keeps a robot out as "seen by neither camera", look at where it sits in the side picture. The corners of the
  board nearest the side camera's edges may be outside its view.
- After the first real session: run `e02_twocam` on its recording with the overhead alone and with both cameras, as
  done here, and set the 6 cm and 10-a-second values from it.

## Sweep 1: new tests that may now be necessary

- The Pi's picture rate with the side camera tracked at 10 a second, during a planner run (the overhead must stay
  at 14+ a second).
- Whether the moved side camera sees all 4 board corners and all of the board. If not, check which spots it cannot
  see.
- The flash check's lights as anchors on the real board: does the side picture show the three lights clearly enough
  to number the corners?
- SIFT on the Pi's OpenCV and on the washed-out real board: the number of grain matches.
- Lights-to-middle distance from real L0 pictures (expected near 3 cm).
- Side exposure settling on the real EMEET C950: settle time and brightness after the lock.
- Identity at crossings when only the side camera sees both robots: L5, with "in line with" moments counted.
- The side offset learned on the real board against the study's 4.7 cm (test T1, taped marks).
- The turn check's total allowed of 8 deg against the simulated Center (about -7 to -8 deg with either camera): repeat
  checks of one healthy robot on the board would show whether 8 deg false-FAILs (the DOE already noted for it).

## Sweep 2: possible inaccuracies in what this relied on

- **The washed-out real footage was made in software** (overhead contrast cut to 12 % on the board). A real lamp also
  adds glare spots and may wash out the side camera partly.
- **The simulator's side camera** is a plain pinhole. Its robots are boxes with dim faces and bright tops, and its
  board is not washed out (assumed: the glare reaches the overhead, not the slanted side). The real side picture
  showed the robots clearly last night, but that was before the cameras were moved.
- **Last night's calibrations** (roam1 overhead, 25 Oct grain side) were used for last night's pictures. The two
  disagree by about 5 cm (the study), and the tracker took no offset off (it learns one only from whole overhead
  outlines). So the 1.85-2.05 cm includes their disagreement, not only tracking error.
- **The robots' starting spots in the real test** were read by eye from the first overhead picture.
- **The side lag** in the simulator and the default (0.04 s) are stand-ins. One earlier session had -0.40 s,
  unexplained. A wrong lag shows up as moving robots trailing; the test shows 0.12 s untold costs about 1 cm at 10 cm/s.
- **Lights:** 3 cm from the middle, two LEDs merging into one spot, and the 235 brightness floor are taken from the
  simulator and the flash check's figures, not measured on the real top. Idle robots show their mode colour on the
  left LED only; the keeper's white replaces that while the ladder watches.
- **The half-height plane (1.85 cm)** for the side outline's centre is chosen (e02_sidecam). The simulator's side
  outline is mostly the top, which leaves about 0.5-1 cm of error that the plane does not remove.
- **The setup page's lights anchors** use the overhead calibration in use when the flash check ran. Running the flash
  check before saving part 1 after a camera move gives wrong anchors; the 2x margin rejects them rather than trusting
  them.

## Appendix: how the real-footage test was run

Recording `e02-pi-plan-20261027-0011-recording`, 00:16:55-00:17:40 EDT. cam0 (25/s) and cam1 (15/s) frames were read
by byte range. Each camera's t_wall minus 0.04 s was used, and the side lag was 0.05 s. The starting spots were the
overhead pixels (142, 276), (222, 283), (387, 72) taken to the floor. "Washed out" pushed every pixel inside the
board's outline towards 193 at 12 % of its distance from the board's median. Scripts (scratch):
`realrun.py over|two|washover|washtwo`, `fig.py`, `overlay.py`. Not committed, because they read the scratch
download.

## Fix after the real L0 at 13:01: all three robots reported "body unseen"

**Premise.** The first real ladder run with both cameras (13:01, `./run-fleet-pi.sh ladder pause`) found all three
robots by their lights, then said neither camera could see any of their bodies. In both pictures a person sees all
three robots clearly.

**Data.** The real 13:01 pictures (both cameras) were replayed through the ladder's own camera code in real time,
with tracking started where the live run started it (13:01:24.8, from the robots' failure times in state.json).

| Replay of the 13:01 pictures | Left | Right | Center |
|---|---|---|---|
| Old code, this machine's speed | seen (overhead) | seen (overhead) | lights only |
| Old code, tracker slowed to Pi speed (3.5x) | unseen | unseen | unseen |
| New code, Pi speed (3.5x) | seen (both) | seen (side) | seen (side) |
| New code, this machine's speed | seen (both) | seen (side) | seen (side) |

<sub>Pi speed: each tracked picture also busy-waits for 2.5 times its own processing time, holding Python's lock (a
pessimistic stand-in: on the Pi, OpenCV's work runs partly in parallel). 3.5x is the ratio of earlier Pi
measurements, 26-73 ms a picture, to this machine's ~14 ms.</sub>

Two causes, both confirmed on the replay:
1. **The tracking fell behind the camera.** The ladder tracked every picture in the thread that reads the stream
   (about 21 a second from the overhead camera, 25-29 from the side camera). At Pi speed the work was more than one
   core could do, so pictures queued up, and the check asked for sightings from the last 0.9 s of clock time, while
   the tracker was still working through pictures from seconds earlier. The old code at Pi speed reproduces the live
   result exactly: Left had 298 overhead sightings, but none recent enough.
2. **The side camera's picture was too dark to track.** The recorder locks the side camera's exposure. The lock read
   as matched straight away (pictures from before the change were still waiting in the camera's buffer). The picture
   then settled at a mean of 20 of 255, against 79 under automatic exposure, and the recorder's own log warned
   "PICTURE VERY DARK". The robots then stood only 7-10 grey levels from the floor, under the tracker's threshold of
   16, so the side camera found nothing. The setup page's side picture (mean 83) was taken before this lock, which
   is why it looked fine.

![Side picture as recorded and brightened](../screenshots/2026-10-27/l0-1301-fix-problem-side-picture-dark-no-outlines-then-brightened.png)
The real 13:01 side picture: as recorded, no robot outlines are found; brightened 4x, all three are found (green rings).

**What changed.**
- **Ladder camera (`tools/fleet/ladder.py`, Eye).** One thread per camera now reads the stream and keeps only the
  newest picture. A separate tracking thread per camera takes the newest picture when it is due and drops the rest.
  Rates are capped at 12 a second for the overhead and 10 for the side camera.
- **Checks use the tracker's own time.** `still()` and `seen_by()` look back from the newest picture the tracker has
  worked through, not from the clock. If tracking is more than 0.5 s behind, L0 says "camera tracking is N s behind
  the camera".
- **New log, `tracking_times.csv`.** Every step folder gets one: each tracked picture's camera, time, cost in ms, how
  far behind it was and how many pictures were dropped. L0 also prints one line per camera.
- **Dark side pictures are brightened.** In `e02_twocam`, a side picture with a mean under 50 is multiplied up to a
  mean of 80 (at most 5x) before tracking. The factor is printed in L0 and saved in `summary()`.
- **Recorder exposure lock (`camrec.py`).** The lock now drops 6 buffered pictures before each brightness reading,
  so the exposure is actually matched. This only takes effect when the recorder is restarted.
- **Faster side-camera tracking.** A robot's size in the side picture is now kept per 1 cm square of floor, not
  projected again for every outline. That projection was a third of the side camera's cost: 29 ms a picture fell to
  13 ms on this machine. SciPy also loads when the camera starts, not inside the first tracked picture.
- **Cores (`run-fleet-pi.sh`).** The plan follows the e02 runner. The recorder runs on cores 0-1 (it records two
  cameras), and the ladder with its tracking runs on cores 2-3 (`taskset`).

**CPU budget, 4-core Pi 4** (this machine's cost per picture x 3.5, the stated Pi slowdown):

| | Pictures tracked a second | Cost a picture (Pi, est.) | Share of one core |
|---|---|---|---|
| Overhead | 12 (cap) | ~45-50 ms | ~0.55-0.6 |
| Side | 10 (cap) | ~45-50 ms | ~0.45-0.5 |
| Total | | | ~1.0-1.1 |

<sub>At 10 cm/s, the ladder's fastest leg, 12 pictures a second leaves under 1 cm between pictures. Chosen; DOE note:
8 / 12 / 16 a second against the Pi's measured cost and lag (tracking_times.csv) and the line check's p95.</sub>

Both trackers run in the ladder's one Python process, so the Python parts share one core and the OpenCV and NumPy
parts can use the second. At Pi speed in the replay the tracking reached 9.4 (overhead) and 7.9 (side) pictures a
second, never more than 0.23 s behind. When it is short of CPU, it now drops pictures instead of falling behind.
For L0 (nothing moves) that is enough. Moving each camera's tracking into its own process (as e02_trackproc does) is
**not done**. It is the next step if the real tracking_times.csv shows under ~8 pictures a second, or lag over
0.3 s, while robots drive.

**Conclusion.** With both fixes, the real 13:01 pictures give all three robots seen at Pi speed: Left by both cameras,
Right and Center by the side camera.

![Replay after the fix](../screenshots/2026-10-27/l0-1301-fix-feature-replay-all-three-seen-both-cameras.png)
After the fix, on the real 13:01 pictures: a ring where the tracker puts each robot, in both cameras' pictures.

**What's next.** Run L0 for real. Read the new `tracking_times.csv` in the L0 step folder (cost, lag and drops on
the real Pi) and the side camera's brightening line. Restart the recorder so the exposure-lock fix and the core
pinning apply.

**New tests that may now be necessary.**
- Unit tests for the Eye's drop-stale threads, using a fake stream and a slow tracker. The replay harness did this,
  but it is not in the suite.
- A recorder test where the fake camera's buffer holds pictures from before the change.
- A dry run with the simulated side camera made dark, to check brightening in the ladder end to end.

**Possible inaccuracies.**
- **The 3.5x Pi slowdown** is a ratio of earlier Pi measurements, not measured on this code. The busy-wait stand-in
  holds Python's lock, which is harsher than the Pi.
- **The brightening target (80) and limit (5x)** are chosen. Brightening raises the noise too, so false outlines on
  a dark picture are untested beyond this one recording.
- **Right's side position** is (89.7, 44.3), about 4.5 cm from where the overhead put it offline (85.9, 42.0). That
  matches the side camera's unexplained ~5 cm offset from 26 Oct. The offset is learned only while both cameras see
  a whole robot standing still, and here only Left was. L1's crawl measures headings from motion, so L0 is not
  affected, but Right's L0 spot may be off by that much.
- **The recorder fix** (dropping buffered pictures) is untested on the real camera.
- **Center's overhead outline** is still partial (washed out). Center relies on the side camera; when it is hidden
  from the side, it falls back to its lights.
