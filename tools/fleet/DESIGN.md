# Fleet planner: design note

How the laptop-side planner (this folder) connects to the live system, the
option of storing routes on the robots, and what docking 1 cm from a board
needs. Built since (2026-10-18b): the robot-side step follower (firmware
`run follow`, follow.h / pose.h) and the laptop runner run_plan.py, tested
end to end against simulated robots (tools/sim/fleet_sim.py; results in
README.md). The protocol as built is in docs/data-formats.md, "Step follower".

## 1. The live loop

```
 overhead camera (Pi) ──sightings──┐
 robot wheels + gyro ──WiFi log────┼─> fusion (EKF per robot) ──pose──┐
                                   │                                   v
                       map.json ──> planner ──plan.json──> coordinator (laptop)
                           ^                                 │  releases steps
                           └── new obstacles ─────────────── │  whose "after" are done
                                                             v
                                                 robot follower (firmware)
                                                 drives the step on its own
                                                 wheels + gyro, corrected by
                                                 camera fixes; reports progress
```

1. **Pose in.** The fusion engine (tools/fusion, run live) gives each robot's
   pose on the floor frame about 10 times a second. Calibration (blink code +
   roll) names each robot and fixes its heading before planning.
2. **Plan.** The planner takes those poses as start spots and makes
   plan.json. Planning the demo floor takes 3-9 s; re-planning after an
   obstacle about 1 s (worst seen 2.5 s).
3. **Steps out.** The coordinator (run_plan.py) sends each robot its next
   steps. A step is *released* when every step in its `after` list has been
   reported done. It keeps up to 2 released steps queued beyond the one
   running, so a robot runs straight through consecutive drive pieces without
   stopping; each step goes only after the one before it was acked (lost
   packets are resent, and must not overtake each other).
4. **Robot follows.** The follower runs each step on its own sensors:
   - **turn**: gyro to the absolute heading;
   - **drive**: straight along the line from `from` to `to`, steering on its
     pose, stopping at `to`. Measure "how far still to go" along the step's
     own heading, not along "here -> target". When a step is restarted almost
     finished (after a re-plan), "here -> target" is a fraction of a cm of
     pose noise and points anywhere. The simulator found this: one robot
     overshot by 6 cm and docked 1.8 cm off-centre;
   - **approach**: see section 3.
   Its pose comes from wheels + gyro between camera fixes. The laptop sends a
   `fix` (fused pose) about 5-10 times a second, and the robot blends it in, so
   errors never pile up. That is why "within 1 cm" docking is plausible with a
   0.3 cm fused pose.
5. **Progress back.** The robot reports `started` and `done` per step (with its
   pose), plus `blocked`, `approach_done` and `approach_failed`.

**Messages as built** (plain text commands on the console's command port;
the robot's answers are logged records; details in docs/data-formats.md,
"Step follower"): `fs <id> T|D|A|W ...` (a step; ids go up per robot instead
of plan_id + i, so a re-plan needs nothing on the robot), `fix <robot ms> x y
th sigma`, `fpose`, `hold [id]` (finish up to that step, drop the rest),
`halt` (stop now, stay in follow mode), `stop` (end the run). The robot
answers with STEP records (queued/dup = ack, started, done, failed with the
reason and, for an obstacle, the 5 front zone readings, dropped, held/halted)
and a FOLLOW record 10 times a second (its pose, uncertainty, last done id).

**Re-plan triggers** (all call `planner.obstacle_event`, which applies Nick's
rules: stop the robots that would reach the obstacle within `stop_horizon_s`,
let the others carry on until the new plan is ready, then switch):

- the camera confirms an unknown object on the floor (fusion rule 3: 5
  sightings in 0.5 s) that is not a robot, a board or the map;
- a robot reports `blocked` (front reading well below the step's
  `front_clear_cm`);
- a robot is more than about 3 cm off its step's line (the "after" lists
  only protect robots that stay on their lines);
- a robot is lifted, or an approach fails;
- a board is seen away from where map.json has it, or Nick changes a task;
- a robot was left without a route ("waiting for a route"): try again 3 s
  later, with more time allowed.

Lateness alone does not need a re-plan: the "after" lists keep late robots
safe. It only costs time, and the next re-plan picks that up.

**If the WiFi drops:** a robot finishes the steps it has buffered and holds.
Holding at a step boundary is always safe, because the other robots' steps
wait for it through their "after" lists. Simulated 3 s drop: the robot
finished its buffered step about 1.9 s in, stood still about 1.4 s, and was
moving again 0.3 s after the link came back (6 resends).

## 2. Routes stored on the robot instead?

Nick asked about this. Plan here, run there is the usual split. A route is
small: one robot's demo plan is about 30 steps, roughly 3 KB, so memory is
not the issue.

| | Laptop sends steps (built) | Whole route stored on the robot |
| --- | --- | --- |
| WiFi drop | robot holds after its current step, safe but stops | keeps going, but blind to the others |
| Robots giving way to each other | the laptop releases steps on "after", sees all robots | each robot needs the others' progress anyway: robot-to-robot messages, or the laptop again |
| Surprise obstacle | new steps in about 1 s | re-upload the remainder to every robot affected |
| Camera corrections | already on the laptop | must be sent anyway |
| Changing tasks | press Plan | re-flash or upload |
| Time between steps | one radio hop (2 steps sent ahead hides it) | none |
| Testing without a laptop | no | yes, for a fixed demo |

**Recommendation: hybrid.** Keep planning and step release on the laptop.
Have the robot buffer the released steps it has been sent (a small queue,
say up to 5), and finish what it holds if the link drops. The robots then get
the main benefit of storing routes (no stall on a WiFi blip) without losing
coordination. Fully on-robot routes only make sense for a single robot, or for
fixed demo routes like today's `run demo`.

## 3. Docking 1 cm from a board: another sensor-fusion example

The camera puts a robot within about 0.3 cm (simulation). That is not the
same as "front face 1 cm from the board", because the board's position on
the map is a tape measurement and the robot's front face is 4.75 cm from the
tracked point. The final approach should therefore measure the board
directly, with the front distance sensor (ST VL53L7CX on the Alvik).

**The catch:** the sensor's minimum range is about 2 cm (Pololu's spec for the
same chip; ST quotes up to 350 cm and a 60 x 60 deg field of view). A 1 cm
gap is below that, unless the sensor sits recessed at least 1 cm behind the
robot's front face. So the approach fuses two sensors:

1. **Approach** (pre-dock -> dock, 15 cm at 3 cm/s, facing the board). Below
   about 10 cm, take the distance readings.
2. **Fuse** each reading with how far the wheels have rolled:
   board position = reading + distance travelled. Average these (or run a
   1-D Kalman filter on them) from 10 cm down to 3 cm, about 35 readings at
   15 per second. In simulation, with 0.2 cm sensor noise, this gives a docking
   error of at most 0.17 cm. A single reading gave up to 0.5 cm.
3. **Hand over to the wheels** below 3 cm (inside the sensor's unreliable
   range). Roll the remaining (estimate - travelled - 1 cm) on wheel
   odometry; 2 cm at 2% wheel error is 0.04 cm.
4. **Square up** (optional): the left and right zones of the sensor see the
   board at slightly different distances if the robot is not square to it.
   The difference gives the angle, and a small gyro turn fixes it before the
   last centimetres.
5. **Camera** keeps the sideways position honest (the sensor cannot see
   sideways error against a flat board).

**Front checks in the follower (done, 2026-10-18b):** `safe_front_cm` (7 cm)
would stop every approach and many planned passes near boards. In a follow
run each step sets its own check from plan.json's `front_min_cm` (what the map
says the sensor should see at the least): turns use the centre zone below
min(7, front_min - 2 cm); straight drives look at all five zones inside the
robot's own path, out to 12 cm (an obstacle the robot's side would clip leaves
the sensor's 60 deg view below about 8 cm, so the 7 cm centre check alone let
a simulated robot drive into a brick), but only as far as the step still goes
plus 2 cm: a robot waiting just beyond the end of the step is allowed there by
the plan (5 cm apart), and looking past it made a simulated robot stop and
re-plan in a loop; the approach does its own check (a
reading 2 cm below what the board should give, twice in a row). A reading must
stay too close for 100 ms (two sensor frames) before a step stops: single
noisy readings tripped about 1 approach in 20 in simulation.

**To measure before trusting 1 cm:**
- the sensor's reading against a ruler at 0.5, 1, 2, 3, 5, 10 cm from a
  board, centre zone: offset, noise, and where it stops being linear;
- the sensor's position behind the front face (recess);
- how often the Alvik library actually updates `get_distance` (sets the
  approach speed: 3 cm/s at 15 Hz is 0.2 cm per reading);
- board height: the board must fill the sensor's view at 2-10 cm. It is
  60 deg wide, so at 5 cm the view is about 6 cm tall and 6 cm wide.
