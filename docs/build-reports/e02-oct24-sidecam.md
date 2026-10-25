# Side camera on roam1: calibration, cross-check, both trackers (2026-10-24, updated 2026-10-25)

## Premise
Steps 2 and 3 of `docs/build-plans/second-camera-plan.md`, offline on roam1 (2026-10-20): calibrate the side camera to
the floor, check it against the overhead where a robot stands still, and replay the e02 tracker on the side view to see
how often it holds a robot the overhead loses. The first pass (Oct 24) had only the side camera's motion detections.
On Oct 25 Nick uploaded its video, frames.csv and meta to `roam1-data`, and this report uses them. The Oct 24
sightings-only results are kept in appendix A5.

**What the side picture allows.** The side camera stood at the y = 0 edge, about 1.7 m up. Its picture shows only one
board corner, (0, 0): the corner at (122, 0) is just past the right side and both far corners are above the top. The
4-corner method (`find_board_corners` → `order_corners_like` → `make_calibration`) cannot work on this mount. The side
camera was instead calibrated from **the plywood's grain**: points on the floor matched between the overhead's and the
side's robot-free pictures. It was then checked against the board's blue tape.

## Data

**Clock** (side `frames.csv`, the same Pi clock as the overhead). Side frame 0 is at 0.543 s (overhead: 0.551 s), at
29.09 fps with 14 frames missed (0.05%). The fitted clock of Oct 24 (offset 0.58 s) was within −0.14 to +0.24 s of the
real one (median 0.007 s). Everything below uses the real clock.

**Calibration from the picture** (`tools/experiment/data/side_floorcal_roam1.json`; the Oct 24 robot-sightings one,
refitted on the real clock, is kept as `side_floorcal_roam1_sightings.json`):

| | from the picture (grain) | from robot sightings |
|---|---|---|
| Points | 174 floor points (SIFT, RANSAC), x 3.5-121.8, y 3.3-111.9 cm | 3,269 robot pairs in 33 cells, x 20-110, y 0-100 cm |
| Fit error | 1.42 px; leave-one-out 0.42 cm (worst 1.06) | 2.12 px; leave-one-out 0.72 cm (worst 1.24) |
| Lens bend | k1 = 0.28 (kept: leave-one-out 0.64 → 0.42 cm) | none |
| Camera | (45.4, −54.2) cm, 168 cm up, 27.4° from straight down | (54.6, −31.4), 143 cm up, 24.5° |
| Edge x = 0 vs blue tape | **0.27 cm** (1.0 px) | 2.06 cm (6.8 px) |
| Edge y = 0 vs blue tape | 0.15 cm (0.6 px) | 0.09 cm (0.4 px) |

The two place a side pixel within 0.75 cm of each other (median over a 10 cm grid; 90%: 1.6 cm). The worst is
4.0 cm at the far-left corner (x under 20, y over 95), where the sightings fit had no points. The x = 122 edge has
no tape, so it can't be checked this way.

![side check](e02-oct24-sidecam-check.png)

A side camera picture from roam1 (frame 15000) with the board as the picture calibration places it: green = the
board's outline, on the blue tape at the left and near edges; yellow = the x = 122 board line, on the plywood's right
edge; red circle = corner (0, 0).

**Cross-check at still moments** (robot standing still for 0.4 s by the overhead's motion detections; its side
detections at the same moment taken to the floor with the picture calibration). There were 364 moments in 12 places.

| x cm | y cm | region | moments | median diff cm | worst cm | lean toward side camera, cm |
|---|---|---|---|---|---|---|
| 0-30 | 0-30 | middle | 3 | 3.3 | 4.0 | 1.8 |
| 0-30 | 30-60 | middle | 11 | 1.2 | 1.3 | −1.1 |
| 30-60 | 0-30 | middle | 53 | 1.7 | 4.9 | 0.2 |
| 30-60 | 30-60 | middle | 48 | 0.7 | 4.9 | 0.2 |
| 30-60 | 60-90 | middle | 52 | 0.7 | 6.4 | −0.1 |
| 30-60 | 90-120 | middle | 7 | 0.9 | 1.9 | 0.5 |
| 60-90 | 0-30 | middle | 13 | 0.9 | 1.5 | 0.2 |
| 60-90 | 30-60 | middle | 36 | 1.4 | 4.5 | −0.4 |
| 60-90 | 60-90 | middle | 24 | 2.4 | 2.8 | −1.4 |
| 90-120 | 0-30 | along the x=122 board | 36 | 1.7 | 10.6 | 0.7 |
| 90-120 | 30-60 | along the x=122 board | 48 | 0.8 | 8.6 | 0.4 |
| 90-120 | 90-120 | blocked corner | 33 | 1.7 | 2.8 | 0.7 |
| **all** | | | **364** | **1.25** (90%: 2.7) | **10.6** | **0.19** |

The two worst are explained (checked on the pictures). The 10.6 cm one at 123 s is a robot half outside the side
picture's right edge, so its outline is cut. The 8.6 cm run at 832-836 s is a tall dark box from the objects phase,
not a robot: a tall object leans far more than a robot.

**Lean** (the side camera sees the robot's near face). The side outline's centre is best taken on the plane 1.85 cm
up, half the robot. There the lean is 0.19 cm (median) and does not change with distance from the camera
(−0.18 cm per metre). On the floor plane the lean is −1.1 cm; on the robot's top (3.7 cm) it is +1.5 cm (appendix A2).
The robot-sightings calibration does about as well at still moments (median 1.29 cm, 90%: 2.9 cm), because it was
fitted to the overhead's own positions.

**Both trackers on the same robot** (`e02_replay.replay` with `e02_camera.MotionTracker` on each camera, following one
robot from Nick's labelled start with the other two parked; the side tracker on the 1.85 cm plane with
`size_per_spot`). Samples every 0.2 s; "holds" means it returned a position.

| run | stretch | robot | region | both hold | overhead only | side only | neither | trackers apart (median cm) |
|---|---|---|---|---|---|---|---|---|
| A1 | 175-300 s, calibration moves | (116, 11) spinning in place | along the x=122 board | 544 | 39 | 42 | 0 | 2.6 |
| A2 | 175-300 s | (116, 57) spinning | along the x=122 board | 623 | 0 | 2 | 0 | 1.4 |
| A3 | 175-300 s | (116, 105) spinning | blocked corner | 617 | 0 | 8 | 0 | **5.1** |
| B1 | 476-600 s, roaming | drove 8.4 m | middle / open edge y=0 | 565 / 54 | 2 / 0 | 0 / 0 | 0 | 1.7 |
| B2 | 476-600 s | drove 6.2 m | middle | 619 | 0 | 2 | 0 | 2.1 |
| B3 | 476-600 s | (112, 108) standing | blocked corner | 621 | 0 | 0 | 0 | **4.9** |

By place, the side tracker held a robot the overhead had lost:
- along the x = 122 board: 3.5% of the time (44 of 1,250 samples);
- in the blocked corner: 0.6% (8 of 1,246);
- in the middle: 0.2% (2 of 1,186).

The overhead held what the side had lost 3.1%, 0% and 0.2% of the time. The longest loss was 1.7 s for the overhead
and 0.4 s for the side (both in A1). The two trackers never followed different robots: they were never more than
8 cm apart.

**In the blocked corner the overhead holds the robot but misplaces it.** It sits about 5 cm toward the x = 122 edge
(x ≈ 120.6 against the side's 115.9 and the label's 116.0). It is 3.1 cm from the overhead's own motion detector there,
against 0.8-1.1 cm elsewhere. In the overhead picture the robot is washed out against the bright corner (as in the
Oct 24 glare finding), and the overhead tracker's centre lands on its edge toward the board.

## Conclusion
- **The side camera has a picture-based calibration**: 0.42 cm leave-one-out, 0.27 cm from the tape along the x = 0
  edge, and 1.25 cm median from the overhead at 364 still moments. It replaces the robot-sightings fit, which was 2 cm
  off along the x = 0 edge where it had no points.
- **No lean correction is needed at this mount** when the side outline is taken on the half-robot plane (lean
  0.2 cm, flat with distance). The plan's step 4 can be dropped for a mount like roam1's; it should be rechecked on a
  steeper mount.
- **On roam1 the overhead rarely loses a robot, even along the board and in the corner** (held 96.5% along the board,
  99.4% in the corner, 99.8% in the middle; the worst single robot 93%). So roam1
  can't show the side camera rescuing it: the side holds what the overhead lost only 0.2-3.5% of the time. Roam1 does
  not reproduce the losses seen in the e02 sessions (30% held against a board, Oct 22), and its robots did not go
  near the y = 122 board or the Oct 24 glare cells.
- **Where the side camera helped on roam1 was position, not holding.** In the blocked corner the overhead tracker is
  about 5 cm off toward the board and the side tracker is not.

## What's next
1. Footage from an e02 session with the side camera recorded, where the overhead is known to lose robots against the
   boards (Oct 22-24). This is the test roam1 can't give. Question for Nick below: was camera 1 recorded then?
2. Plan step 1 (the new mount): calibrate there with `e02_sidecam.py picture` (grain), or with all 4 corners if the
   board is fully in view (`order_corners_like` is ready). Recheck the lean on the half-robot plane.
3. Plan step 5 (combiner): in the blocked corner, trust the side camera's position over the overhead's.
4. Pi recording: the side camera's exposure ran on auto (`camera_controls_end`: auto_exposure 3), unlike the
   overhead's locked exposure; its pictures were dark (mean grey about 59).

## New tests that may now be needed
- An e02 session with both cameras: overhead held share against each board vs the side's, with a set gain to pass.
- Blocked corner: the combined position within 2 cm of the side's when the overhead is pulled toward the board.
- Side tracker on the plan's steeper mount: `size_per_spot` on vs off on real footage (roam1's mount changes scale only
  1.65 times, so here it was not needed).
- Lock-on tracker on a steep view: a robot driving from far to near keeps its lock (its look is taken once, at its
  start size).
- Grain calibration on a second session (camera moved): positions within 1 cm of a 4-corner calibration where all
  corners show.
- Exposure lock on the side camera at recording start (meta `camera_controls_end`).

## Possible inaccuracies
- **The side calibration rests on the overhead's**: the grain points are placed on the floor by the overhead's roam1
  calibration (leave-one-out 1.1 cm, worst 2.1 cm), so the side camera inherits its errors. The tape check is the only
  part independent of the overhead, and it covers two edges.
- **Tape check is a proxy**: it measures the distance to the nearest blue pixel, not the tape's centre line, and the
  overhead's corners were found on the board's brightness edge, not on the tape. Agreement within 1 px means "on the
  tape", not "on the board's true edge".
- **Floor points stop at y = 112 cm**: the strip y > 112 is extrapolated (the side picture's top is at y ≈ 117-120).
- **Tracker comparison has no ground truth**: "holds" is not "right". The two trackers agreed within 8 cm everywhere,
  but both could be wrong together. Nick's labels are mostly unreviewed tracks (2,261 of 3,638 robot labels pending),
  so they were used only for starting spots.
- **Stretches were chosen by me**: two windows of about 2 minutes, 3 robots each. Robots along the board mostly spun in
  place, which is easier than driving along it.
- **Board layout**: blocked boards at x = 122 and y = 122 come from the default `Board` layout. In the side picture a
  dark upright surface stands along the x = 122 edge, which may be a board, but that is not recorded.
- **Chosen numbers** (whether a designed experiment should set each):
  - half-robot plane 1.85 cm: chosen, then measured best of 0, 1, 1.85, 2.5, 3.7 and 5 cm (appendix A2); no DOE needed.
  - SIFT ratio 0.8, RANSAC 4 px, 21 pictures per median and their frame ranges: a sweep is enough, no DOE.
  - "same robot" 8 cm between trackers, samples every 0.2 s matched within 0.1 s: a sweep is enough.
  - still moment (0.4 s, under 1 cm), region bands (edge 12 cm, corner 25 cm): these set the tables' split. A small
    DOE (band × still rule) is worth it only if the trust map (plan step 4) is built per region.
  - MotionTracker's own numbers (threshold, shape limits, lock-on) were used unchanged on the side view. Tuning them
    for the side camera is a DOE question once footage where the overhead loses robots exists.

## Appendix
**A1. Picture calibration method** (`e02_sidecam.py picture`).
1. Median of 21 frames for each camera over the roaming phase (overhead frames 12100-18100, side 14100-21100), so
   robots that move are gone.
2. Local contrast evened (CLAHE), SIFT up to 8,000 features each (2,958 and 2,876 found), Lowe's ratio 0.8 (252
   matches).
3. Overhead points taken to the floor at z = 0 and kept on the board (198), then a RANSAC homography at 4 px (174
   inliers).
4. `CameraModel.from_points` with the mirror found and the lens bend kept only if it lowers the leave-one-out error.

The tape check is in `tape_check` (blue in HSV, nearest blue pixel to 23 points along each edge).

**A2. Which plane the side outline's centre is on** (still moments; median diff, 90%, lean median, lean change per metre
from the camera):

| plane | 0 | 1.0 | **1.85** | 2.5 | 3.7 | 5.0 cm |
|---|---|---|---|---|---|---|
| median diff cm | 1.69 | 1.30 | **1.25** | 1.32 | 1.91 | 2.57 |
| 90% cm | 3.43 | 3.03 | **2.74** | 2.91 | 3.60 | 4.38 |
| lean cm | −1.11 | −0.35 | **0.19** | 0.62 | 1.45 | 2.23 |
| lean per m | −1.30 | −0.70 | **−0.18** | 0.21 | 0.93 | 1.72 |

**A3. Tracker against its own camera's motion detector** (median cm, every 25th sample): corner A3 overhead 3.1, side
0.8; board A2 overhead 1.05, side 1.12; middle B1 overhead 0.78, side 0.82.

**A4. Code.**
- `e02_sidecam.py`: `FrameClock` (real clock, accepted wherever the fitted offset was), `median_picture`,
  `grain_pairs`, `fit_from_floor_points`, `tape_check`, `replay_pair`, `compare_tracks`, and a `picture` command.
  `main` now writes the sightings fit to `side_floorcal_roam1_sightings.json` and takes `--side-frames-csv`.
- `e02_replay.replay(..., tracker_kw=None)` passes MotionTracker arguments; the default is unchanged.
- From Oct 24: `e02_camera.make_calibration(name=)`, `order_corners_like`, `MotionTracker(size_per_spot=)`.
- Tests: `tests/test_e02_sidecam.py` (24) pass. The existing camera tests (camerr, explore, fixbuild, multitrack, oct22,
  realclips, one runner test) give 59 passed and 12 skipped (no clips).

**A5. Oct 24, before the side video (robot sightings only).**
- Clock fitted at 0.58 s.
- Sightings calibration: 1.15 cm median at 361 still moments (2-fold by minutes), worst 11.8 cm.
- Studio-detector coverage (half-second bins, 170-730 s), where the side camera saw a robot the overhead's detector
  missed: 14% in the blocked corner, 11% along the x = 122 board, 12% in the middle. The tracker replay above
  supersedes this: with MotionTracker the overhead loses far less than its motion detector suggested.
