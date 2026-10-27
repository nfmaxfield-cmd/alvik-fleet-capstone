# No-boards overnight session and re-plan speed (2026-10-26, 22:49 build)

## Premise

Nick took both boards off for tonight's unattended session, to collect edge data (Monday's school table has no
boards) and more two-camera footage. The runner assumed two boards. Nick also asked whether the fleet planner's
re-plan of about 1 s can be shorter.

## Data

**No boards.** Three simulator dry runs (`--dry-run --auto --no-boards --dry-true-calib --skip-naming`, seeds 50-52;
the simulator's boards are taken away at once):

| Run | Folder (logs/) | What happened |
|---|---|---|
| N1 | e02-20261026-225200-dryrun (stopped by hand) | The front-sensor check compared readings with boards that were not there: false "lost robot" holds every few legs, and Center's front sensor marked bad (32.8 x its baseline) |
| N2 | e02-20261026-225637-dryrun | Ended at the first block: the board re-measuring step had no board lines (KeyError) |
| N3 | e02-20261026-230000-dryrun | Edge finds against two open edges on every run ("found itself against the x=0 and y=0 sides (edge, edge)", 1-3 cm and 1-5 deg off its own pose); runs completed |

**Re-plans.** 20 obstacle re-plans on the demo map (two obstacles, 10 moments each), old and new code on the same
machine, one after the other:

| | Old | New |
|---|---|---|
| Wall time, median / worst | 1.04 / 2.15 s | 0.83 / 2.06 s |
| First safe plan found, median / worst | 0.027 / 1.11 s | 0.015 / 1.11 s |
| Priority orders tried in the budget, median | 9 | 17 |
| Sum of makespans (20 plans) | 631.9 s | 631.3 s |
| Closest two robots, any plan | 6.5 cm | 6.5 cm |

The re-plan is an anytime search: once a first safe plan exists it keeps trying other orders until 0.9 s
(`0.45 x replan_budget_s`), so its wall time is mostly the budget. The robots keep driving their already-sent steps
(up to 2 s ahead) while it runs; only a robot heading into the new obstacle stops.

## What changed

- `e02.py --no-boards`: both boards marked gone from the start (all four sides open edges for the virtual border,
  routes and parking); implies `--classic-setup` (explore assumes two boards), `--edge-find` and `--skip-find-check`.
  The front-sensor check (`ahead_cm`), the board re-measure (`boards_still`) and the tracker's walls skip a missing
  board; these also fix the same faults after a board is found gone mid-session.
- `tools/pi/session_plan.sh` plan `edges` (`./run-e02-pi.sh --plan edges --with-side`): E1 15 min on the robots'
  own sensors, then E2 all conditions until the batteries run out; holds wait 3 min before the robot rescues itself.
- `planner.py`: `Grid.turnable` decides from the grid unless near the threshold (same answers, checked on 46,000
  points); `_escape` answers are kept per grid; `first_plan_s` in the plan summary.

## Conclusion

- The no-boards session runs in the simulator with edge finds on every run. **Known gap:** the robots' own border
  holds only two edges (one x and one y) without a firmware change; the other two rely on the camera border and the
  edge sensors.
- The re-plan change made each search step quicker (about twice the orders tried), not the re-plan shorter. Most
  re-plans have a safe plan within 0.03 s; the rest of the time looks for a shorter one. The slow cases (robots
  crowded at the start) take about 1.1 s to find any safe plan, and the time there is spent in the robot-to-robot
  timing checks (`sweeps_meet`), not the clearance checks.

## What's next

- Nick's choice (Claude's suggestion): send the first safe plan at once and swap in a shorter one if the search finds
  it, or shorten the budget. Either cuts a typical re-plan to well under 0.1 s.
- The crowded-start case needs the timing checks made faster (vectorised), a separate piece of work.
- Real-robot check tonight: plan `edges`.

## Sweep 1: tests that may now be needed

- A real session with no boards (tonight): do edge finds work on the plywood's real edges, and how often does the
  border hold at 4 edges?
- Firmware border with four edges (a firmware change) before Monday.
- A dry run with no boards and command loss (`--dry-drop-pct 10`).
- The dry run's camera trust flapped between "bad" (2.6-3.2 x) and healthy within minutes; check whether real edge
  finds do the same.

## Sweep 2: possible inaccuracies

- The simulator's picture still shows the boards after they are taken away (its own limitation), so the camera side
  of a no-boards session is not tested.
- Re-plan times are from this cloud machine with a dry run sometimes sharing it; the laptop's will differ.
- `--answer-timeout-min 3` for the edges plan and E1's 15 minutes are chosen convenience settings, not measured (no
  DOE needed: they decide how long the night idles, not what is measured).
- `replan_budget_s` 2.0 is chosen; a simulator sweep (0.5 / 1 / 2 s: robots stopped and seconds lost) would set it.
