# Fleet planner: boards, tasks, routes

Plans where each robot drives, on the laptop, so that three Alviks can visit
their boards ("machines") at the same time without meeting, and runs the plan
on the robots (run_plan.py, firmware 2026-10-18b+). The robots get the plan
step by step and buffer a few steps; they do not store routes.

| File | What it does |
| --- | --- |
| `app.py` + `page.html` | The page: draw the floor, boards, keep-outs and obstacles, give each robot its boards, Plan, watch it play out, drop a surprise obstacle, save |
| `planner.py` | The planner (also runs from the command line) |
| `fleetmap.py` | map.json: reading, checking, dock poses |
| `geometry.py` | Shapes and distances |
| `simulate_plan.py` | Kinematic simulator: drives plan.json with noise and delays, checks docking and spacing, can drop a brick mid-run (quick: 20 runs in a minute) |
| `run_plan.py` | Runs plan.json on the robots: releases steps by the "after" lists, resends until acked, passes camera fixes on, re-plans on new obstacles; `--dry-run` against simulated robots (tools/sim/fleet_sim.py) |
| `examples/` | `demo_map.json` (4 boards, crossing routes), `park_test_map.json` (first test: drive around, park where you started) |
| `DESIGN.md` | How this connects to the live system, routes on the robot vs on the laptop, what 1 cm docking needs |

File formats: docs/data-formats.md ("Fleet map", "Fleet plan").
Needs Python 3.9+ with `numpy` and `scipy`. No internet connection.

## Quick start

```
python tools/fleet/app.py --map map.json
```

The page opens in the browser. A new map starts as an empty 4 ft x 4 ft
floor. Set the measured floor size in the Map tab first.

1. **Map tab.** `+ Board`: click both ends of a board. Drag the white end dots,
   or type exact numbers under "Selected". A board's dock (the outline square
   1 cm off its face) and pre-dock dot appear on its open side. `+ Robot` puts a
   start spot; drag it, or type x, y and heading. The red shading shows where a
   robot's centre may not go.
2. **Tasks tab.** For each robot, add its boards in order. "any order" lets the
   planner choose the order. "park at the end" sends it back to its park spot.
3. **Plan tab.** Press Plan. The summary says when the last robot finishes,
   how close two robots ever come, and lists close passes; click one to jump
   there. Play, or drag the time slider. A robot outlined orange is in a close
   pass, red would be too close (should not happen).
4. **Surprise.** Move the slider to a moment, press "Drop obstacle at this
   time", click the map. The list shows which robots stop and why, and the plan
   continues from there.
5. **Save** writes map.json and plan.json.

From the command line:

```
python tools/fleet/planner.py --map tools/fleet/examples/demo_map.json --out plan.json
python tools/fleet/simulate_plan.py --map tools/fleet/examples/demo_map.json --plan plan.json --runs 20
python tools/fleet/simulate_plan.py --map tools/fleet/examples/demo_map.json --plan plan.json --brick auto@15
python tools/fleet/run_plan.py --map tools/fleet/examples/demo_map.json --plan plan.json --dry-run --brick auto@15
python tools/fleet/run_plan.py --map map.json --plan plan.json            # the real robots
python -m pytest tools/fleet/tests -q
```

## Running a plan (run_plan.py)

1. Robots on firmware 2026-10-18b+, each calibrated since power-on (`calibrate`
   in the console, or `--calibrate` here), standing on its map start spot.
   Close the console: one program on the robots' port.
2. `python tools/fleet/run_plan.py --map map.json --plan plan.json`. It stops
   anything left running, checks each robot's firmware, starts a `follow` run
   on each, gives each its pose (the camera's with `--fixes udp`, else the
   plan's start spot after you confirm the robots are on their spots), and
   starts.
3. Each robot gets its next step as soon as every step in its `after` list is
   reported done, at most 2 beyond the one it runs, each only after the one
   before it was acked. Unacked steps are resent every 0.3 s. Camera fixes go
   to each robot 10 times a second with the robot's own clock time of the
   frame, so it can correct where it was then.
4. Something new on the floor (the camera's `obstacle` message, a robot
   whose distance sensor stops a step, or a robot whose plywood edge guard
   stops a step at an open edge: that robot holds and the edge goes into the
   map as a no-go strip, source `edge_guard`, logged as `edge_boundary`;
   not a failure): the quick part of the obstacle rules
   runs at once (imminent robots get `halt`, carrying-on robots `hold` after
   their last carried-on step), the new plan is computed in the background
   while the others drive on, then takes over. A robot left without a route is
   retried every 3 s with more time; about 1 s after a new plan, a better one is
   tried with 6 s and kept only if everyone finishes sooner.
5. Every 10 s it prints how far each robot is; when no robot is moving it says
   what each is waiting for. At the end every robot gets `stop` (RUNEND), and
   run_plan_summary.json says what happened, including boards not visited.

Files and the protocol: docs/data-formats.md, "Step follower".

## Running with the live camera (the live link)

The overhead camera tells each robot where it is while the plan runs. Three
programs work together: **camrec.py** on the Pi (the camera), **track_live.py**
on the laptop (finds each robot by its blinking lights and works out where it
is), and **run_plan.py** on the laptop (drives the robots and passes the
positions on). How it works: tools/live/README.md.

You need: the Pi with the camera, the laptop, both with the newest code, and a
`floor_calibration.json` made for where the camera is now (tools/pi/README.md,
"Floor calibration"). The robots blink automatically during their run.

**Once, after the code changed: update both computers.**

On the Pi (in a PowerShell window, log in to the Pi first: type the line
below, press Enter, then type the Pi's password):

```
ssh st1maxfini@st1maxfini.local
```
```
cd ~/alvik-fleet-capstone
```
```
git pull
```

On the laptop (a second PowerShell window, in the project folder):

```
git pull
```

**Every time:**

1. **Start the camera (Pi window).** Type this and press Enter:

   ```
   python3 tools/pi/camrec.py --device 0 --fps 25 --label live
   ```

   You should see `Preview: http://st1maxfini.local:8000 (live link for
   tools/live/track_live.py: --stream st1maxfini.local:8000)` and `Recording
   640x480 to ...`. Leave this window alone (it also records the video).

2. **Start the live link (laptop, PowerShell window 1).** Put your
   calibration file's path after `--calib`:

   ```
   python tools/live/track_live.py --stream st1maxfini.local:8000 --calib C:\path\to\floor_calibration.json
   ```

   You should see `floor calibration ...; 3.4 px per cm` (about 3.4 for the
   camera at 262 cm), then a status line every 2 seconds:
   `... ms/frame  fixes: none yet  lights: none`. Once the robots' run starts
   and they blink, after 2-4 seconds it says `fixes: Left 12, Center 9, ...`.
   If it says `camera stream: ... trying again in 1 s` over and over, step 1
   isn't running, or the Pi's code is old (`git pull` on the Pi).
   If you have a `calibration_results.csv` from the robots' calibration runs
   (tools/blink/calib.py), add `--calib-results C:\path\to\calibration_results.csv`:
   it gives each robot's light position on its body exactly.

3. **Run the plan (laptop, PowerShell window 2).**

   ```
   python tools/fleet/run_plan.py --map map.json --plan plan.json --fixes udp
   ```

   It starts the robots' runs, then says `Waiting for camera fixes...` and,
   for each robot, `Left: camera puts it 0.8 cm / 1.2 deg from its start spot`.
   Then the robots drive. If it says `No camera fix for Center`, look at
   window 1: is Center listed after `fixes:`? (Not listed: is it in view and
   blinking? Its name must be exactly `Center`.)

4. **Stop.** Press Ctrl+C in window 2 first (that stops the robots), then in
   window 1, then in the Pi window (that saves the video).

Window 1 saves what it sent in `logs\<date-time>-live\` (messages.jsonl,
live_summary.json). It never sends a position under a name it isn't sure of:
a robot whose blink rhythm isn't clear yet, or two lights that could both be
the same robot, get no fixes until that clears up.

**Try it without robots:** play a recording back in real time and print what
would be sent (tools/sim/simulate.py --video makes a recording):

```
python tools/live/track_live.py --replay C:\path\to\recording --dry-run
```

## How the planner works

1. **Where a robot can be.** The floor becomes a grid of 1 cm cells. A cell is
   free if the robot could turn in place there without coming within the
   safety margin (1.5 cm) of a board, keep-out, obstacle or the floor edge.
   Turning in place sweeps a circle of radius 6.7 cm (half the robot's
   diagonal), so that is the minimum distance from the centre.
2. **Route.** The shortest way over the free cells, then only the corners the
   robot needs: straight drives between them, turns in place at them. That is
   how the Alvik moves most accurately.
3. **Docking.** Route to the board's pre-dock point, turn to face the board,
   creep 15 cm straight in at 3 cm/s, wait, back straight out. On the creep
   the robot averages its distance-sensor readings (each plus the wheel
   travel) and below `handover_cm` (3 cm) finishes on the wheels so the front
   face stops 1 cm from the board; it never rolls more than 15.5 cm
   (`max_travel_cm`: length + gap - `approach_margin_cm`).
4. **Timing, robot by robot.** The first robot gets its quickest timing. Each
   later robot is timed around the ones already planned. It may not do a step
   at the same time as another robot does a step whose swept area comes within
   5 cm of its own. Where that would happen, it waits, at the spot that keeps
   it moving longest. If waiting would take too long, it tries a route around
   the others. If two robots are too close to turn, one first drives a few cm
   straight to make room.
5. **Best order.** All orders of the three robots are tried, plus one more
   pass where each robot is re-timed against the others' finished timings. The
   plan where the last robot finishes soonest wins. Two robots that want the
   same board are handled by the same search: whoever lets everyone finish
   soonest goes first.
6. **"After" lists.** Each step lists the other robots' steps that must be done
   before it starts. The laptop releases a step only when those are done. The
   timing in the plan is the expected one, not a promise.

### Why "after" lists, not times

In the simulator, executing the same plan by the clock alone (each step
starts at its planned time), with 10% of steps starting up to 3 s late, made
robots touch in 15 of 15 runs. With the "after" lists: 0 of 30 runs, closest
5.6 cm.

## When something new appears (Nick's rules)

The camera (or a robot's front distance sensor) reports a new obstacle. Then:

- **Stop at once:** any robot whose next `stop_horizon_s` (2 s) of driving
  would bring it within the safety margin of the obstacle.
- **Carry on:** every other robot keeps going until the first step end at least
  `replan_budget_s` (2 s) from now, then holds there. The new plan starts from
  those spots, so it is ready by the time they arrive.
- **Hold early:** a robot that would come too close to a stopped robot while
  carrying on holds earlier, or stops if it is already that close.
- The page and the simulator print who stopped and why.
- A robot that gets no safe route this time stays put ("waiting for a route").
  It is re-planned 3 s later with more time allowed (6 s, then 12 s). Stopped
  robots are safe, and the others are on the move.
- About a second after the new plan starts, the simulator tries once more with
  6 s allowed, and keeps the result only if everyone finishes sooner.

## What was checked (simulated, not yet on robots)

Demo map (122 x 122 cm, 4 boards, 3 robots, 2 boards each, routes cross,
two robots share board A):

| What | Result |
| --- | --- |
| Planning | 3-9 s on this computer (park test 1.5 s); last robot done after 65 s; closest planned approach 6.5 cm |
| Kinematic sim, 30 runs: pose noise 0.3 cm, speed and turn errors, radio delays | all finished, no contact, closest 5.55 cm; 180/180 dockings within 0.25 cm of 1 cm, sideways error up to 0.5 cm, facing within 1 deg |
| Same with 10% of steps starting up to 3 s late | 30/30 finished, closest 5.57 cm, about 6 s later on average |
| Same, run by the clock only (no "after" lists) | robots touched in 15 of 15 runs |
| Realistic distance sensor (15 readings/s, 0.2 cm noise, last 3 cm on the wheels; the default since 2026-10-18b) | docking error up to 0.17 cm with the readings averaged (0.5 cm without). With the corrected travel cap (15.5 cm): 30 runs, 180 dockings, 179 within 0.25 cm, worst 0.30 cm short (the cap stopped it: the robot's pose was ~0.8 cm off at the start of the approach) |
| Park test (drive around, park where you started), 30 runs | all finished, closest 5.6 cm, back on the start spot |
| Obstacle dropped at 22 moments x 4 places (planner only) | a safe new plan every time; median about 1 s, worst 2.5 s |
| Brick dropped mid-run, 60 runs (3 s or 1.2 s ahead of a robot, at 15/30/40 s, one batch with the realistic sensor and stalls) | all finished, no contact, closest 5.36 cm, all boards visited; re-plan after the obstacle at most 1.74 s; about 10-22 s added to the finish. A robot was often left "waiting for a route" by the 2 s re-plan; a retry with more time always fixed it. One docking was 0.27 cm off with the realistic sensor (tolerance 0.25) |
| **run_plan.py --dry-run** (2026-10-18b): run_plan.py through alvik_console.Link, real UDP, against tools/sim/fleet_sim.py: three robots running a copy of the firmware follower (own pose, camera fixes 10/s with 0.3 cm noise and 0.1 s delay, distance sensor 15 frames/s, 0.5 cm + 1% noise, seeing boards, the brick and each other), demo map, 15 runs | all finished, no contact. No surprises (3 runs): 64.8-65.3 s (plan 65.0), closest robots 6.30 cm, no resends. Brick at 15 s, camera reports it (3 runs): 83-97 s, closest to the brick 3.49 cm. Brick the camera never reports, found by the robots' distance sensors (3 runs): 72 s, 2 sensor stops each, closest to the brick 1.26 cm. 3 s WiFi drop of one robot (3 runs + 1 earlier): the robot runs its buffered steps (up to 3 s), stands still once they are done, moves again 0.05-1.05 s after the link is back; 0-6 resends; finish time unchanged |
| Dockings in those runs (78), measured on the true pose | front face 1 cm target: mean +0.06 cm (stops slightly short), sd 0.10, worst -0.17 / +0.31 cm; sideways at most 0.27 cm; heading at most 0.6 deg |
| Same, no camera fixes (wheels + gyro only, from the start spot) | finished, but dockings up to 3.4 cm sideways and 3.7 deg, and two robots came 4.47 cm apart (plan: at least 5): the camera fixes are what keeps robots on their lines |
| Tests | `python -m pytest tools/fleet/tests -q`: 27 tests, about 1 minute (+1 end-to-end dry run with FLEET_DRYRUN=1, about 90 s) |
| Page | driven in a headless browser: draw, drag, plan, play, obstacle with retry, save; no script errors |

## Limits

- **The follower is untested on a robot.** Firmware 2026-10-18b compiles
  against stand-in headers; the follower's numbers above come from its Python
  copy in tools/sim. Things to measure first: the front sensor's offset behind
  the front face (`tof_recess_cm`, assumed 0.25), its frame rate and lag
  (`tof_ms`, `tof_lag_ms`), the five zones' directions (assumed +/-22, +/-11,
  0 deg), and how often a robot passing another trips `follow_look_cm` (12).
- **A robot stops for another robot it sees in its path.** The planner keeps
  bodies 5 cm apart, and drives only check the path they still drive (+2 cm),
  so this did not happen in the dry runs; if it does, run_plan.py re-plans
  without adding an obstacle.
- **Sensor-found obstacles grow piece by piece.** Without the camera, each
  stop adds what the five zones saw (one object, 6 cm deep); a long obstacle
  can take several stops (2 for the 20 cm brick).
- **Simulated only.** Speeds, turn rate and pauses are guesses (10 cm/s,
  60 deg/s, 0.3 s). Fit them from E1 runs; the plan's timing depends on them.
- **The safety comes from "after" lists, not speed.** A robot that is
  off its route by more than about 4 cm (5 cm separation minus about 1 cm of
  timing slack) is no longer covered. The follower must stay on the line,
  and the laptop should stop everyone if a fused pose drifts that far.
- **Not optimal.** Robots are planned one at a time, then improved. Routes are
  fixed lines with waits on them, plus three fallbacks (route around the
  others, step aside a few cm, wait for a later re-plan). In crowded moments
  one robot can wait much longer than needed: a brick in the middle of the
  demo floor can add 10-40 s.
- **Narrow gaps.** Gaps that the robot could drive through but not turn in
  (between 2 x 6.2 and 2 x 8.2 cm wide) are treated as closed.
- **Other robots in front of the distance sensor** are not modelled in the
  simulator; on the real floor a robot passing 5-7 cm ahead could trip the
  front safety stop (see DESIGN.md).
- **Steps carried on through a re-plan** (the "carry on" rule) are checked
  against each other and the stopped robots by their planned timing, not by
  "after" lists, for those 1-3 s. A robot that stalls exactly then is not
  covered.
- **Planning time** is Python on this computer; check `replan_budget_s` on
  the laptop that will run it (the page shows how long each re-plan took).
