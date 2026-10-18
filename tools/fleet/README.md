# Fleet planner: boards, tasks, routes

Plans where each robot drives, on the laptop, so that three Alviks can visit
their boards ("machines") at the same time without meeting. The robots get
the plan step by step; they do not store routes.

| File | What it does |
| --- | --- |
| `app.py` + `page.html` | The page: draw the floor, boards, keep-outs and obstacles, give each robot its boards, Plan, watch it play out, drop a surprise obstacle, save |
| `planner.py` | The planner (also runs from the command line) |
| `fleetmap.py` | map.json: reading, checking, dock poses |
| `geometry.py` | Shapes and distances |
| `simulate_plan.py` | Kinematic simulator: drives plan.json with noise and delays, checks docking and spacing, can drop a brick mid-run |
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
python -m pytest tools/fleet/tests -q
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
   creep 15 cm straight in at 3 cm/s until the front sensor reads 1 cm, wait,
   back straight out.
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
| Realistic distance sensor (15 readings/s, 0.2 cm noise, last 3 cm on the wheels) | docking error up to 0.17 cm with the readings averaged (0.5 cm without) |
| Park test (drive around, park where you started), 30 runs | all finished, closest 5.6 cm, back on the start spot |
| Obstacle dropped at 22 moments x 4 places (planner only) | a safe new plan every time; median about 1 s, worst 2.5 s |
| Brick dropped mid-run, 60 runs (3 s or 1.2 s ahead of a robot, at 15/30/40 s, one batch with the realistic sensor and stalls) | all finished, no contact, closest 5.36 cm, all boards visited; re-plan after the obstacle at most 1.74 s; about 10-22 s added to the finish. A robot was often left "waiting for a route" by the 2 s re-plan; a retry with more time always fixed it. One docking was 0.27 cm off with the realistic sensor (tolerance 0.25) |
| Tests | `python -m pytest tools/fleet/tests -q`: 23 tests, about 1 minute |
| Page | driven in a headless browser: draw, drag, plan, play, obstacle with retry, save; no script errors |

## Limits

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
