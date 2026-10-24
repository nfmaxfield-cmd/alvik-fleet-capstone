# Design of experiment: how fast explore may drive at an unknown edge

Nick, 2026-10-24: "4 cm/s and include a DoS to get the right number." Two robots went over the open edge during
explore on 2026-10-23 (docs/build-reports/e02-oct23-falls.md). Explore's rays, its room check and its square now
crawl at `EXPLORE_CRAWL_CMS` (4 cm/s; the firmware's `e02_cms`, 10 cm/s otherwise); returns over ground just
driven go at 10 cm/s. Chosen, not measured.

## Question
How fast can a robot drive towards an open edge it has not met before and still stop with every wheel on the board,
at any angle it meets the edge?

## Factors and levels
| Factor | Levels | Now |
|---|---|---|
| Speed (`e02_cms`), cm/s | 2, 4, 6, 8 | 4 |
| Angle to the edge, deg from square on | 0, 30, 60 | (any) |

12 cells. Every cell 5 times per robot (3 robots): 180 approaches, about 45 minutes. Randomised order within each
robot. A thin foam strip below the open edge in case one goes over.

## How
A test script (not the session runner): the robot starts 30 cm from the open edge, turned to the cell's angle (the
camera checks it), and drives straight at the cell's speed until its edge guard stops it. Logged per approach: which
cue stopped it (bottom zone or line sensors), the robot's own distance driven, and from the camera the front
wheels' place against the plywood's end (the camera calibration and the board's measured outline).

## Responses
- **Margin**: the front wheels' distance from the plywood's end at the stop (cm; negative = a wheel over).
- Which cue stopped it.
- Time to cover 30 cm (the cost of crawling).

## Analysis
- Margin by speed and angle: the lowest 5% of margins per cell (a one-sided bound), because one fall is what
  matters, not the average.
- The bottom zone's share of stops by angle: the falls on 2026-10-23 suggest it misses slanting approaches.
- Recommendation: the fastest speed whose lowest-5% margin stays above 1 cm at every angle.
