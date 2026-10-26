"""E02 turn health (2026-10-25 15:45 session, fix 1): a robot whose turns keep failing is caught quickly, stopped,
named and left out (Nick's decision, 18:46: "leave that robot out and carry on with the others").

At 15:45 Right's turning broke right after its compass check: from 15:58:58 every turn ran to the firmware's
timeout and its own heading moved the opposite way to the turn asked for (121 of 131 turns timed out; 46 of 46
turns of 20 deg or more went the wrong way by its own heading). The runner drove it on for 15 minutes into the
bottom edge's border band. Nothing counted those failures.

What counts as a turn fault (one turn leg):
  - it timed out ("turn timed out", the firmware's own words), or
  - by the robot's own heading it turned the wrong way: a turn of at least WRONG_MIN_DEG whose own heading moved
    more than WRONG_MOVED_DEG the other way.
A turn that did what it was asked clears the count. Two faults in a row (FAULT_RUN_N) trip it.

Also counted: a heading correction over CORR_DEG (the camera measured the robot's heading on a straight leg and
the robot's own was that far off) right after one of its own turns; CORR_N of those within CORR_WINDOW_LEGS legs
trip it too (the analysis's second rule: Right's own heading was 11-177 deg off the measured one 18 times).

Pure: no network, no clock. Unit-tested in tests/test_e02_1545_fixes.py.
"""
import math

FAULT_RUN_N = 2            # chosen (the 15:45 analysis): two failed turns in a row. Right's first two turns failed, so it
# would have been caught at 15:59:10, 12 s after its first turn. DOE note: yes, a logs replay: every session's turn
# legs (leg.csv), counting how often healthy robots have 1, 2, 3 failed turns in a row (false trips) against the
# 15:45 Right (doe-backlog.md "Turn health limits")
WRONG_MIN_DEG = 20.0       # chosen: below this a turn's own heading change is too small to judge its direction
WRONG_MOVED_DEG = 10.0     # chosen: the own heading moved at least this far the wrong way (not gyro noise). Same DOE row
CORR_DEG = 45.0            # the 15:45 analysis's rule: a correction over 45 deg ...
CORR_N = 2                 # ... twice ...
CORR_WINDOW_LEGS = 5       # ... within 5 legs. Same DOE row (logs replay of heading_pushes after turns)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def turn_fault(val, state, info, own_dth):
    """Words if one turn leg failed as a turn (timed out, or went the wrong way by its own heading), else ''.
    val: the turn asked for (deg, left +); state / info: the leg's result; own_dth: its own heading change over the
    leg (deg, None if not known)."""
    low = (info or "").lower()
    if "timed out" in low:
        return "a %.0f deg turn timed out" % val
    if own_dth is not None and abs(val) >= WRONG_MIN_DEG and own_dth * val < 0 and abs(own_dth) >= WRONG_MOVED_DEG:
        return "a %.0f deg turn went %.0f deg the other way (its own heading)" % (val, own_dth)
    return ""


class TurnWatch:
    """Per robot: the run of failed turns and the recent big heading corrections. feed_turn / feed_go / correction
    return words when the robot trips (once; it stays tripped), else ''."""

    def __init__(self, run_n=FAULT_RUN_N, corr_deg=CORR_DEG, corr_n=CORR_N, window=CORR_WINDOW_LEGS):
        self.run_n, self.corr_deg, self.corr_n, self.window = run_n, corr_deg, corr_n, window
        self.run = {}          # robot -> consecutive failed turns
        self.why = {}          # robot -> [words of each failed turn in the run]
        self.legs = {}         # robot -> legs counted
        self.corr = {}         # robot -> [leg number of each big correction after a turn]
        self.tripped = {}      # robot -> words

    def feed_turn(self, robot, val, state, info, own_dth):
        """One finished turn leg (refused ones are not turns: the robot did not try)."""
        if robot in self.tripped or state in ("refused", "dup"):
            return ""
        self.legs[robot] = self.legs.get(robot, 0) + 1
        w = turn_fault(val, state, info, own_dth)
        if not w:
            if state == "done":
                self.run[robot], self.why[robot] = 0, []
            return ""
        self.run[robot] = self.run.get(robot, 0) + 1
        self.why.setdefault(robot, []).append(w)
        if self.run[robot] >= self.run_n:
            self.tripped[robot] = "its last %d turns failed (%s)" % (self.run[robot], "; ".join(self.why[robot][-self.run_n:]))
            return self.tripped[robot]
        return ""

    def feed_go(self, robot):
        """A straight leg (counts towards the correction window)."""
        self.legs[robot] = self.legs.get(robot, 0) + 1

    def correction(self, robot, deg, after_turn):
        """A heading correction of deg (the camera's measured heading minus its own), after_turn: the robot's own
        heading came through one of its own turns since it was last measured."""
        if robot in self.tripped or not after_turn or abs(deg) <= self.corr_deg:
            return ""
        n = self.legs.get(robot, 0)
        lst = [k for k in self.corr.get(robot, []) if n - k < self.window] + [n]
        self.corr[robot] = lst
        if len(lst) >= self.corr_n:
            self.tripped[robot] = ("its own heading was over %.0f deg off the camera's %d times within %d legs, each "
                                   "after one of its turns" % (self.corr_deg, len(lst), self.window))
            return self.tripped[robot]
        return ""


def turn_done_deg(start_th, final_th):
    """How far a leg turned by the robot's own heading (deg, the short way), from its LEG records' th_deg."""
    if start_th is None or final_th is None or not (math.isfinite(start_th) and math.isfinite(final_th)):
        return None
    return wrap180(final_th - start_th)


def turn_rest(val, done):
    """What is left of a turn of val after done (deg), or 0.0 when it is done or overdone. A turn whose own heading
    moved the wrong way is not "done" by that: the rest is the whole turn plus what it went the wrong way."""
    if done is None:
        return None
    rest = val - done
    if rest * val <= 0:
        return 0.0
    return rest
