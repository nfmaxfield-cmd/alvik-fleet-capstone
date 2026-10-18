# Site setup: learn where everything on the floor is, check it, lock it

Run the site setup when the floor is **new or changed** (a board moved, added or
taken away, the camera moved, the plywood moved). It is not the per-robot
calibration run: that one runs once after each power-on and calibrates the
robot itself (wheels, gyro, front sensor directions). The site setup learns the
**floor**: where each board, block and open plywood edge is, merging the
robots' front distance sensors, their wheels and gyro, their edge guard, the
camera, and what you click and tape-measure. It locks the map only after a tape
check passes.

| File | What it does |
| --- | --- |
| `setup.py` | The guided command: the eight steps below, plain-language questions |
| `survey.py` | Where each robot stands to measure each landmark; corner sweeps; edge approaches (routes from tools/fleet/planner.py) |
| `observe.py` | From the robots' logs and the camera to still poses ("stations") and their readings |
| `solve.py` | The joint fit: every landmark and edge, every robot's sensor model, every still pose, all at once, with uncertainties |
| `sitemap.py` | Landmarks, edges, the estimate, the audit (pick points, pass rule), the locked map files |
| `change.py` | Change check during normal runs: "landmark B may have moved" (also live: `run_plan.py --site-map`) |
| `simrun.py`, `pipeline.py` | The survey on the simulator (fast, not real time) |
| `sim_eval.py` | The simulation evaluation (docs/experiments/E00d-site-setup) |
| `examples/sim_site_world.json` | The simulated floor used by `--sim` and `sim_eval.py` |
| `tests/` | `python -m pytest tools/site/tests -q` (about 10 s) |

## Before you start

You need, done once already:

1. A camera calibration (`floor_calibration.json`), made with
   `python tools/calibrate_floor.py <recording folder>`, with a fit error of
   0.35 cm or less (the setup checks this).
2. The floor drawn in the fleet page (`python tools/fleet/app.py --map site/map.json`):
   the floor size and each robot's start spot. Boards can be added there or on
   the camera page in step 2.
3. Each robot calibrated since it was switched on (`calibrate` in the console).
4. The edge guard checked: E0c's edge rows and `python tools/edge_check.py`
   (docs/experiments/E00c-rig-measurements). Without that, answer "none" to the
   open-edge question: the robots then do not drive up to any edge.
5. Optional but better: the robots' range bias from E0c's C9 rows (robot
   square to a board at tape-measured distances) as a small file
   `bench.json`: `{"Left": {"bias_cm": 0.3, "bias_sd_cm": 0.15, "scale": 1.0, "scale_sd": 0.004}, ...}`.
   Without it, a range error every reading of a board shares cannot be told
   from the board being that much further away (see "What limits it").

## Step by step

All commands are typed in a **Terminal window on the laptop**, in the project
folder (the one with `tools/` in it). Type one box at a time and press Enter.

**A. Start the setup.**

```
python tools/site/setup.py --site site --map site/map.json --calib <recording>/floor_calibration.json --recording <recording>
```

You should see `=== Step 1: camera check ===` and either `OK` or `NOT GOOD
ENOUGH` with what to do.

**B. Step 2, landmarks.** It asks "Open the camera page now?". Press Enter (yes).
A browser page opens with the camera picture. In its **Landmarks** mode, click
both ends of every board where it meets the floor, and the corners of every
block. Press **Save landmarks**. Go back to the Terminal and press Enter. For
each landmark it asks "is it on the floor now, where the page shows it?": press
Enter for yes, type `n` and Enter for no. Then it lists the four sides of the
plywood and asks which have **no** border board; type them like `x0,y1` and
press Enter (or `none`).

**C. Step 3, plan.** It prints, per robot, how many spots and edge approaches it
will do and about how long. Nothing to do.

**D. Step 4, survey.** It asks you to start the camera recording on the Pi.
In the **Pi's Terminal** (over RealVNC):

```
python3 camrec.py --label site
```

Back on the laptop, press Enter. For each robot it says which one to put on its
start spot and to take the others **off the board**. Do that, press Enter. The
robot drives its plan (`run_plan.py` prints what it does). Without live camera
fixes (`--fixes none`, the default), use `--segment-spots 6`: the robot comes
back to its start spot every 6 spots, and you put it back exactly on its mark
when it asks. At the end it asks you to stop the camera recording (press Ctrl+C
in the Pi's Terminal).

**E. Detector and clocks.** Run the detector on the recording and line up the
clocks (as for any recorded run: docs/experiments/INDEX.md, "Workflow"), then
start the setup again with the detections:

```
python tools/site/setup.py --site site --map site/map.json --calib <recording>/floor_calibration.json --recording <recording> --detections <recording>/detections.csv --bench bench.json
```

Answer the questions as before. When it asks "A survey was already driven ...
Use it?", press Enter (yes): it does not drive again, it uses the runs it saved.

**F. Step 5, solve.** It prints each landmark's position and how sure it is, in
cm and inches.

**G. Step 6, tape.** Points the robots could not pin down (sure only to more
than 0.5 cm: for example a board end with another board right behind it) are
listed one at a time. Measure each with the tape from the axes' origin and type
`x, y` in inches, like `36 3/32, 12 1/2`, then Enter. (Or in cm: `91.6, 31.7 cm`.)

**H. Step 7, audit.** Two or three points the program picks (the least sure
ones, and one at random): measure them the same way. It prints PASS or FAIL for
each. On a FAIL, it measures the failing landmarks again with the robots and
uses your readings too; then it picks new points.

To check every landmark instead of the two least sure (recommended the first
time, about 1.5 minutes of tape work per landmark), add `--audit-every-landmark`.

**I. Step 8, lock.** On a PASS it writes `site/map.json` (version, date,
uncertainties, the audit, the camera calibration it was made with),
`site/landmarks.csv`, `site/tof_calibration.json` (for the fusion) and
`site/audit.csv`, and says `Locked: version N`.

### One landmark moved

```
python tools/site/setup.py --site site --map site/map.json --calib <recording>/floor_calibration.json --refresh B
```

only surveys board B again (and audits it).

### Try it on the simulator first

```
python tools/site/setup.py --site site_sim --sim --auto
```

runs all eight steps on simulated robots in about 15 minutes, with simulated
clicks and tape readings, and ends with `Locked: version 1` and the true error
of the locked map.

`--dry-run` instead runs the real-robot step 4 against simulated robots in real
time (`run_plan.py --dry-run`): a check that the plans run on the coordinator.

## How it works

**Survey (survey.py).** For every face of every landmark, spots where the front
sensor is 14, 24 or 38 cm away, square on or 18 deg to either side: at each the
robot stands still 2 s, turns 12 deg left, stands, turns 12 deg right, stands.
For every end of every long face, a **corner sweep** from 16 and 30 cm: the
robot turns in 2 deg steps over +-14 deg, standing still between steps. A zone's
reading jumps from the board to what is behind it between two steps: the
board's end lies in that direction (give or take half the zone's width). Ranging
a long face only says where the face is across it; the corner sweeps say where
the board's ends are. Two distances separate the corner from the zone's width.
For every open edge, up to 5 spots along it: the robot faces out and creeps at
2 cm/s with the edge guard on (`set safe_edge 1`); the guard stops it and ends
the run; the next run backs it 14 cm in (a run of its own: the guard learns
the plywood at the start of each run). Routes between spots come from the fleet
planner, so the plans run on `run_plan.py` and the firmware follower unchanged.

**Solve (solve.py).** One robust least-squares fit (factor graph) of: each
landmark's move, turn and length change from the clicks; each robot's five zone
directions, range bias and scale; the zone width; every still pose; each open
edge's offset and turn; how far past the edge the line sensors are when the
guard trips. Measurements: the camera's view of each still pose; wheels + gyro
between consecutive still poses; every zone's median reading against the face
its ray hits (the fusion's own rules for which face); corner bearings; edge
trips; the clicks; the per-robot calibration's zone directions (`--tof-cal`) and
the bench range bias (`--bench`); tape readings once taken. Uncertainty: the
fit's covariance, scaled up when the fit is worse than its noise model, plus
the camera calibration's own error (its left-one-out fit error), which every
nearby station shares and so never averages away.

**Pass rule.** Every audited point must be within **max(0.3 cm, 3 x sigma)** of
the tape, sigma = the map point's and the tape's (0.1 cm) standard deviations
combined. Why: 0.3 cm is about what a tape reading to 1/16 in held on a corner
can tell; 3 sigma for a 2-D point means a correct map fails a point 1.1% of the
time (3.3% over three points), while a map whose uncertainty is understated by
2x or more fails most audits. The least-sure points are audited because that is
where the map is most likely wrong; the random one checks the points the
program thinks are fine. Before the audit, any point less sure than 0.5 cm is
tape-measured and merged (step 6): the audit then checks a map that is meant
to be good everywhere.

**Locked map.** docs/data-formats.md, "Locked site map". Editing the map in the
fleet page afterwards removes the lock (its hash no longer matches).

**Change check (change.py).** During normal runs every front-sensor reading is
compared with the locked map at the robot's pose. When one landmark's readings
keep disagreeing (median over the last 60 readings, at least 30 over 5 s, more
than max(0.5 cm, 4 standard errors)), it prints `landmark B may have moved ...
Consider a site setup refresh for that area` and writes
`site_changes.jsonl`. Live: `python tools/fleet/run_plan.py --map site/map.json
--plan plan.json --site-map site/map.json`. Afterwards on a session:
`python tools/site/change.py --map site/map.json --session logs/<session>`.

## What limits it

- **The camera frame.** The map is in the camera calibration's frame. Its local
  error (0.15 cm rms assumed in simulation) is in every landmark's position and
  is added to its uncertainty. The fusion uses the same camera, so for the
  fusion that part cancels; against the tape it does not.
- **A range error shared by every reading of a board** looks exactly like the
  board being further away. Boards seen from both sides, corner sweeps and the
  bench range bias (`--bench`) separate them; without any of these a board's
  position across its face is only as good as the robots' range bias.
- **Ends with something right behind them** (a board end touching another
  board) give no corner: their uncertainty stays at the click's, and step 6
  asks for a tape reading.
- **Without live camera fixes** the robots drive on their own pose between the
  start spot and the spots; keep runs short (`--segment-spots`).
- **The edge approach** relies on the edge guard: the drive ends with the robot
  centre 0.5 cm inside the clicked edge (line sensors 3 cm past it). If the
  guard failed and the click was 1 cm too far out, the robot's wheels would
  reach the edge. The guard must pass its E0c check first.
- Simulation only so far (docs/experiments/E00d-site-setup has the plan for the
  real floor).
