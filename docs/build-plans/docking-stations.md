# When the self-calibrating docking stations go in: what changes

A running log (Nick, 2026-10-23: "keep a log of how this will be updated when we get the self calibrating docking
stations installed"). The dock is the printed funnel with a back wall and a floor stripe
(hardware/dock/README.md, firmware dock.h, test docs/experiments/E00e-dock). The robot needs the nose bracket with
its two lever switches. Add a line whenever a build touches something listed here. Newest at the bottom of each
section.

## What the dock gives that the board-push find gives today
| | Board-push find (today) | Dock |
|---|---|---|
| Position fix | pushes into two boards; the bump is felt by tyre slip; about 1-2 cm | both switches click against the back wall: about 0.5 mm, 0.3 deg (dock.h) |
| Heading | squared against a board, if it pushes square ("not square" fails happen) | the bumper face lies flat on the wall |
| Gyro drift | 120 s still checks (gyro.csv) | measured on every docking while it stands still (dock_still_ms) |
| Along-axis check | none | the floor stripe: a second, independent reading |
| Where | the blocked corner, shared by every robot (a robot put aside there blocks every find) | one dock per robot, fixed spots |
| Charging | none | possible later (the dock holds it square) |

## What will change in the software
1. **Finds become dockings.** `run find` (the board-push routine) is replaced by "drive to your dock". The find
   corner is no longer a shared bottleneck, so the "put aside in the find corner" handling (BLOCKS_FIND_CM,
   drive-out from the corner) can go.
2. **Park spots become docks.** `e02_plan.Board.park()` returns each robot's dock pose; parking ends with a dock
   creep instead of a drive to a spot.
3. **Self-calibration at every docking.** The gyro scale and drift and the wheel scale are fit from each trip
   between docks (exact start and end), not from camera fixes alone. A trip between docks becomes a measured run
   with no camera at all: the camera-free accuracy data.
4. **Recovery ladder.** A new step, "drive home to your dock", comes before the finder sweep. A robot that docks
   has an exact position, so "nobody knows where it is" ends there.
5. **Tracker.** At each docking the lock-on retakes the robot's look at a known pose. That is free labelled frames
   for the machine-learning training set.
6. **The site profile** (generic-platform-outline.md) gets "docks: pose, approach direction, stripe position".
7. **Firmware.** dock.h and switches.h exist (2026-10-19a+) but are untested on the robots with the bracket fitted;
   E00e-dock is the test.

## Data the board-push finds give now, worth keeping when docks come
Each find already records (find.csv):
- how far the robot's own pose was off at each board touch (`corr1_cm`, `corr1_deg`, `corr2_cm`, `corr2_deg`);
- the fixed pose;
- the retrace.

The RAW rows during the push show the tyre slip. The dock gives the same pose error, more exactly, so the analysis
should treat "pose error at a fix" as one quantity, whether it came from a board touch or a dock.

## Log
- 2026-10-23: started this log. No dock hardware fitted yet.
- 2026-10-23: the dock as the scale reference: its printed size in the picture, and the stripe-to-back-wall distance measured by the wheels at every docking (an absolute wheel-scale check). See e02-oct22-build.md.
- 2026-10-25: self-charging planned (docs/build-plans/self-charging-dock.md): dock v3 = dock v2 + spring pins in the back wall; attachment v2 = brass pads in the bar, same outside and firmware numbers. To change when built: a `charge` run (dock, confirm contact and charging, sleep, back out), a docked forward-move guard, a contact-sense pin; the runner sends robots at 40 % home and logs every charge's docking as a pose fix.
