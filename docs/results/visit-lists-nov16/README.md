# Visit points: a list per robot (16 Nov 2026)

## Premise

Nick (16 Nov): Jay-Sun should be able to run two robots at the same time, point to point, each with its own list. Do not
change anything that affects how the robots perform. Before this, the run (`mission_run.py`) already accepted a list per
robot (`Alpha:P01,P05;Bravo:P11`, e11), but Mission Control's page only sent one list, split between the robots.

## Data (simulator only: Mission Control `--dry --sim-room school`, two simulated robots, `vp_shots.py`)

- Lists clicked on the page: Alpha P01, P08, P14, P17; Bravo P20, P11, P10, P03 (the routes cross between P14 and P10).
- The run received them as asked (`visit_order.json`) and both robots drove at once from the start (`stops.csv`).
  The closest the plan brought them was 25.1 cm (`mission_summary.json`, plan_gap). Stops were within about 1.5 cm of
  their points.
- The practice simulator restarts a robot on purpose: Alpha restarted at 64 s, and its last two points went to Bravo
  (the existing keep-going rule). All points were visited; the run ended "done" after 237 s.

## What changed

- `mission_control_table.html`: "One list, shared out" (the default, unchanged) or "A list per robot" in the run bar.
  Pick a robot, click its stickers on the map; each robot's points and route are drawn in its colour. The run bar and
  the Settings drawer now redraw after a click (before, they waited until the focus left them, so a clicked run type
  looked unchosen).
- `mission_run.py` (`parse_visit_order` only): a named robot not found at the start no longer stops the run. The robots
  that were found run their lists, and the missing robot's points are named as not visited. Names match whatever their
  case. Planning, driving and the robots' own code are untouched.
- The old page (`/classic`) still sends one shared list.

## What's next

- Jay-Sun's first real two-robot run, once the school Pi is set up.
- Found here, not changed (it is in the run's own code): after a point is handed from one robot to another, Bravo
  recorded two stops at P17 back to back (`stops.csv`).

## Sweep 1: tests that may now be needed

- The same run on the real school table with both robots, then with one robot switched off (its list not visited).
- A run where both lists send the robots to the same sticker at about the same time.

## Sweep 2: possible inaccuracies

- One practice run with a simulated camera and robots; the real table's light and slip are not in it.
- Which physical robot is "Alpha" comes from the wiggle and the site's pins; the page names robots before they are
  found, from the site's robot list.
