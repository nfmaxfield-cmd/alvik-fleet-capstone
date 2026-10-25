# Designs of experiment: tracking confidence and the detector's check rate

Nick, 2026-10-22: "This will come from testing data. Design experiments to see what we should recommend for
confidence rating... I'm also curious how quick it could confirm... A pause and blink is less of a sacrifice than
driving all the way into boards." And on the check rate: "We can start with 2 seconds but write up a design of
experiment to be implemented later."

What the live tracker does now (2026-10-23, `e02_camera.MotionTracker` with `e02_lock.LockTracker`): the detector
finds a robot by motion and hands it to the lock-on tracker. The lock-on tracker follows that robot's own look. The
detector checks it every `CHECK_S` and at every stop. The wheels hold the lock to its spot during a turn on its spot.
Every number below was chosen, not measured.

## DOE 1: detector check rate and the agreement limit (replay: no robot time)
**Question.** How often must the detector check the lock, and how close must they agree, to catch a wrong lock
quickly without costing the laptop too much?

**Factors and levels** (full factorial, 3 x 3 = 9 cells):
| Factor | Levels | Now |
|---|---|---|
| `CHECK_S`: seconds between checks while driving | 1, 2, 4 | 2 |
| `AGREE_CM`: detector within this of the lock = agree | 4, 6, 9 | 6 |

Every stop is always checked as well. That is not a factor.

**Material.** Recorded footage replayed through the tracker, so no robot time is needed. Use the three evening
stretches already in the tests (Left 19:30, Center 20:09, Right 20:26) plus every stretch the next sessions add. That
gives at least 10 stretches, about 20 minutes. Each stretch is one block. All 9 cells run on every stretch, in random
order.

**Responses.**
- Wrong-position time: the share of pictures where the reported position is more than 5 cm from the true one. The
  true position is hand-checked on a picture every second (2026-10-25: marked on the laptop with
  tools/label/mark_robot.py, docs/how-to/mark-the-robot.md; read with its `load_truth()`, scored with `score_track()`).
- Time to catch a wrong lock: seconds from going wrong to being fixed.
- Seen share: pictures with a position.
- Laptop cost: ms per picture.

**Analysis.** Two-way ANOVA with stretch as a block, factor effects and their interaction, and 95% intervals per
cell. Recommend the cheapest cell whose wrong-position time is not significantly worse than the best.

**Effort.** About 1 hour of code (a replay loop over cells, already most of `tests/test_e02_realclips.py`) and 2-3
hours of machine time. Hand-checking truth is about 1 minute per stretch-minute.

## DOE 2: the confidence threshold and how to confirm (hardware)
**Question.** When the combined confidence drops, the robot should pause and confirm where it is. At what level
should it pause, and which confirmation is worth its cost?

The witnesses we have:
- the lock's match score (0-1);
- the detector's last check (age, and whether it agreed);
- the wheels against the tracked move (`cross_check`);
- the front sensor against the map (`front_checks.csv`).

**Part A, the threshold (replay, then confirmed live).** Combine the witnesses into one score: a logistic model fit
on the replays of DOE 1. The response is "the position is wrong" (more than 5 cm off). Then sweep the pause
threshold from 0.3 to 0.9 in steps of 0.1. For each level count:
- false pauses per hour: it paused but was right;
- missed errors per hour: it drove on while wrong.

Plot one against the other (an ROC curve), and choose the level by cost. A missed error costs a recovery, about 30 s
to 3 min from the session logs. A false pause costs one confirmation (Part B).

**Part B, how fast each confirmation is (hardware, about 1 hour).**
| Factor | Levels |
|---|---|
| Confirmation | (1) stand still 0.5 s and let the detector check; (2) stand still and blink the lights, found by `locate_flash`; (3) a short turn on its spot (today's `identify`); (4) drive to touch a board (`run find`) |
| Where on the board | middle; beside a board; open corner (the bright corner) |

Use 3 robots, 10 trials per cell (4 x 3 x 10 = 120 trials, about 1 hour), in random order within each robot.

**Responses.**
- Time to confirm.
- Success: the position after confirming is within 2 cm of a hand measurement on the picture.
- Risk: distance driven toward an edge.

**Analysis.** Two-way ANOVA on time, and a logistic model on success. Expected: the blink takes about 2 s, the turn
6 s and the board touch 30-60 s. The recommendation is the cheapest confirmation whose success rate is not worse,
which should make the pause cheap enough that a cautious threshold is affordable.

## Other chosen numbers a DOE could set
Same replay method as DOE 1:
- `MIN_LOOK` (38): the floor/robot edge limit. The margin is thin: floor 27, robot 49 at worst.
- `LOCK_SURE` (0.7): when the lock beats a still detector outline.
- `PIN_CM` (3): how far the lock may move during a turn on its spot.
- `TURN_STEP_DEG` (10): cost against fit.
