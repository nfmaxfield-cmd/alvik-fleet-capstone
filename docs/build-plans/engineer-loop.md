# The program, the live tracker and the engineer: what connects them, and what should

Nick, 2026-10-23: "I would like a full assessment of connections that should be made between the program, the live
tracker, and the engineer." This is an assessment, not a build. Unknown objects on the map are already in the
backlog (e02-oct22-build.md).

## Premise
There are three parties:
- **The program:** the runner, `e02.py`, and the robots' firmware. It decides what each robot does.
- **The live tracker:** the camera feed, the motion detector, the lock-on tracker and the live view at
  localhost:8090. It says where each robot is and how sure it is.
- **The engineer:** Nick today; an operator on a plant floor later. They step in when the program can't decide
  alone.

The goal is for the engineer to be needed rarely, to see at a glance why they are needed, to decide quickly from
anywhere, and for every decision to become data the system learns from.

## Data: the connections today
| From to | What passes now | Missing |
|---|---|---|
| Program to tracker | each leg (turn or drive, where it may end up), "standing still", which robot to follow, the board's walls, "take its look again" | the robot's own pose and confidence; the plan ahead (where it will drive next) |
| Tracker to program | positions, with the time each picture was taken | **its confidence** (`confidence()` exists; the runner doesn't use it yet) |
| Program to engineer | console lines, help instructions, video marks | why it decided, what it will try next, how long until it gives up |
| Engineer to program | Enter / `f` / `skip` in the console; Ctrl+C and restart flags | a choice among options; a click on the picture; anything from away from the laptop |
| Tracker to engineer | the live picture: the board, robots, a green ring, the floor-memory outlines | lock score, which tracker is carrying it, picture age and dropped pictures (all logged, not shown) |
| Robots' sensors to tracker | only through the runner (the wheels hold the lock during a turn) | the front sensor's "something ahead" as a cross-check on a robot's place |
| Everything to the record | e02_log.txt, track.csv, perf.csv, state, video marks | a still picture at each incident, and the engineer's decisions as data |

## Conclusion: the connections worth making, most valuable first
1. **Confidence drives the program** (tracker to program). When the combined confidence drops, the robot pauses
   and confirms (a blink or a short turn) before driving on, instead of finding out from a camera/wheels
   disagreement after a 10 cm leg. Needs DOE 2's threshold (DOE-tracking.md). Value: fewer losses, fewer help
   calls.
2. **A decision panel for the engineer** (program to engineer to program). On every problem it can't settle, the
   live view shows:
   - the problem in one line;
   - the picture, with the robot circled;
   - the options with the program's choice marked;
   - a countdown after which the default happens.

   The engineer clicks; a phone alert links to the page. Options by problem:
   | Problem | Options |
   |---|---|
   | At an open edge | turn anyway / back off / I'll move it |
   | Lost | search with another robot / put aside / I'll place it |
   | Blocked | it's a robot: move it aside / an object: map it / I'll remove it |
   | Camera and wheels disagree | trust camera / trust wheels / look again |

   Value: unattended sessions keep going, and the engineer never needs to be at the console.
3. **Click to place** (engineer to tracker). The engineer clicks the robot on the picture: an exact position,
   handed to the tracker as a new lock. This replaces "put it down, press Enter, it turns to be found". Every click
   is also a hand-labelled frame for the machine-learning training set and for the DOEs' "truth".
4. **Tracker health on screen** (tracker to engineer). Show:
   - which tracker is carrying the robot (both / lock-on / detector);
   - the lock score;
   - the picture's age, and pictures dropped.

   Each goes amber or red past a limit. This is already in track.csv and perf.csv, so it's cheap. Value: the
   engineer sees trouble coming before a help call.
5. **Incident stills and a timeline** (everything to the record). At every help call, recovery step, edge risk,
   drop or disagreement, save a still of the live view and a line with the video frame number. Value: the
   summary's screenshots come for free, and each incident's clip can be found straight away.
6. **The plan on the picture** (program to engineer). Draw the robot's planned path, the other robots' parked
   spots and their clear zones, and the map's edges as felt. Value: the engineer sees why it chose that path.
7. **The front sensor as a tracker witness** (robots to tracker). "Something 12 cm ahead" where the map has another
   robot confirms both robots' places. This is one more witness for the confidence (1).
8. **The live view away from the laptop.** The page served to the home network (a phone on the same WiFi); later,
   behind a login.

## What's next
Quick ones (about 1 hour together): 4 and 5. They also give the summary its screenshots. Then 2 and 3 (about 3-4
hours with tests; they need a few design questions first). 1 waits for DOE 2's data. 6, 7 and 8 come after.
Unknown objects on the map: Thursday (backlog).

## Sweep 1: new tests that may now be needed
- A dry run where each problem type comes up and the panel's default fires after its countdown.
- A replay check that a click-to-place lock starts on the robot, using the "refuse bare floor" rule.
- The live view's own load on the laptop (perf.csv with and without the page open).

## Sweep 2: possible inaccuracies this relies on
- The value ranking is my judgement, not measured. Help calls per hour (`help_calls` in the state) can test it
  after today.
- The phone alert assumes the laptop has internet; the countdown defaults are chosen numbers.
- The 3-4 hour estimate assumes the live view's server can take a click back (today it is read-only).
