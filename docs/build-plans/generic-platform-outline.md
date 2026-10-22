# Making the fleet logic generic: other robots, other sensors, other sites

Nick, 2026-10-22: "start looking at how we can make this logic more generic for a different setting, when this
software needs to work with new, different equipment with new, different sensors."
This is an outline to discuss before any rewrite (Nick asks before rewrites).

## Premise
Today every piece knows it is an Alvik on a 4 ft plywood board with one overhead camera:
- the runner (`tools/experiment/e02.py`) sends Alvik firmware commands (`go`, `turn`, `run find`, `led`) and reads
  Alvik records (LEG, RAW, FIND, BUMP, EDGE);
- the tracker (`e02_camera.py`) assumes a 9.5 cm square robot seen from above;
- the board planner (`e02_plan.py`) assumes two boards, two open edges, three park spots;
- the recovery steps (turn and look, lights, help) are written out by hand in the runner.
Moving to a warehouse AGV, a different camera or a lidar would mean editing all four.

## What changes between settings, and what stays
| Changes with the equipment or site | Stays the same |
|---|---|
| How to move a robot (commands, speeds, turn on the spot or not) | Plan a path round the others; check each move |
| What each sensor measures, how noisy, when valid | Fuse every measure by how much to trust it (the EKF) |
| What the robot looks like to each camera | Follow one moving thing; never take another for it |
| The site: walls, open edges, docks, no-go areas | A map of what a robot can touch, feel or see |
| How a lost robot can be found (lights, sound, sweep) | A ladder of recovery steps, cheapest first, people last |

## The proposed layers
1. **Robot profile** (one file per kind of robot): size and shape, top speed, how it turns, the commands for a leg,
   a stop and a light, and the records it sends back. The Alvik profile is today's firmware protocol.
2. **Sensor profiles**: for each sensor, *what it measures* (a distance ahead, a line under the robot, a position
   from above, a heading), its noise, its range and when not to trust it. Today's front sensor, line sensors,
   gyro, wheels and camera become five profiles. A new sensor means a new profile, not new runner code.
3. **Site map** (layers): hard walls (boards), drops (open edges), docks and park spots, no-go areas. Today
   `e02_plan.Board` is one hard-coded site; the mapping run (`e02_map.py`) already builds part of a map from the
   robot's own sensors.
4. **Position from everything**: one filter per robot (the live lab's EKF, `e02_live.py`, already does wheels +
   gyro + camera) that takes any sensor profile's measure: the front sensor against the map (today's
   `front_checks.csv`), robot-to-robot distances, a touch on a wall, a second camera. When they disagree, the
   filter's record of which has been right decides, rather than stopping.
5. **Recovery ladder**, built from what the robot profile and sensors offer: look again; lights; another robot
   sweeps for it; it finds itself against walls; then a person. Today's order (2026-10-22) is one instance of it.
6. **Simulator profiles**: the simulator (`e02_sim.py`, `tools/sim/robot.py`) takes the same robot, sensor and site
   profiles, so a new setting is dry-run before any hardware.

## What is already generic, and the first step
Already general: the path planning round other robots, the camera-vs-wheels check, the EKF, the floor-memory
tracker (any top-down camera, any robot size via its constant), the recovery prompts.
First step (small, no behaviour change): move the Alvik numbers (size, speeds, command words, record names) and the
board numbers into a robot profile and a site profile, read by the runner, tracker and simulator. Then a second
robot kind or site is a new file plus a dry run. Estimate: 3-5 hours with tests; to discuss before starting.

## Two sweeps
**New tests that may now be needed:** a dry run with a second, made-up robot profile (bigger, slower, no turn on
the spot) to prove the layers are really separate; a site with no boards at all (only open edges, like a table);
the recovery ladder with lights unavailable.
**Possible inaccuracies this relies on:** that every robot can report its own leg results (some AGVs only take
velocity commands); that one overhead camera's floor plane holds for a bigger site (several cameras would need to
be stitched); that sensor noise is steady enough to put in a profile (it changed with light and battery here).

## Running notes (Nick, 2026-10-22: "keep good notes and update after every run")
What each session or build showed about what is specific to the Alvik and this board, and what would carry over.
Newest first.

### 2026-10-22 afternoon (no session; simulator)
- **Carries over:** driving a robot on its own dead-reckoned pose when no camera sees it, with landmark checks from
  any range sensor against the map; a recovery ladder that ends in "get it out of the way", not just "find it".
- **Specific:** the find corner as the one place a stuck robot blocks everything (a site profile item: "areas that
  must stay clear"); Alvik LEG records carry the robot's own pose (a robot-profile item: "reports its own pose").

### 2026-10-22 (session 3 and the build after it)
- **Specific to the Alvik:** the line sensors read tape and "off the plywood" alike (firmware 22b now treats all
  three high at the start as no floor); the tyres slip at ~96% speed against a board, so a bump is felt by touch,
  not stall; the front sensor's middle zone is reported per leg (`front=`); the robot keeps its own pose between
  camera fixes, so a find works with the camera blind.
- **Specific to this board and camera:** the picture's board lines sit 1-2 cm short of the real faces (WALL_CM 3);
  the corner where both boards meet is where finds end and where robots are hardest to see; one overhead camera.
- **Carries over (now general code):** a recovery ladder (retry, finder sweep, own find, drive out blind on its own
  pose with landmark checks, retrace, then a person); a tracker that knows the site's walls and never takes another
  robot for the one it follows; every move telling the tracker where the robot can be; a sensor checked against the
  map (front sensor now, any range sensor later) with its offset and tolerance either from data or learned in the
  session (`--front-learn`).
- **For the profiles later:** robot size, turn and drive speeds, command words, record names (LEG, FIND, EDGE), the
  `front=` field; site walls, open edges, park spots, the find corner; sensor offset/tolerance pairs.
