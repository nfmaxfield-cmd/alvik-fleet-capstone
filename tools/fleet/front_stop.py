#!/usr/bin/env python3
"""The distance-sensor stop, laptop side (Nick, 2026-10-29 16:2x, from the rules audit
docs/build-reports/e02-oct29-rules-audit.md, recs 1, 2, 4):

  1. 3 readings in a row before a stop, for every robot (run_plan OBSTACLE_READINGS, firmware obs_n).
  2. The floor check: while a robot stands still at connect, its five front zones are read for a few seconds; its stop
     limit (firmware follow_look_cm) is set a margin below the lowest level the floor reached for 3 readings in a row,
     never above its ceiling (12 cm; Left 10 cm, engine_checks.ROBOT_SETTINGS) and never below 8 cm (Left: 9 cm,
     ROBOT_MIN_CM, Nick 2026-10-29 20:06).
  3. Unconfirmed obstacles: the robot always stops; what its sensor saw goes on the planner's map as "unconfirmed"
     unless the camera or a second stop at the same place confirms it. Unconfirmed obstacles expire after
     UNCONFIRMED_S, and are dropped (the plan made again) rather than the run ending with "no plan".

Pure functions and a small registry, used by run_plan.py and tools/analysis/front_stop_replay.py (the replay of the
five DOE releases that checks these numbers on the logged readings).

Which rule wins for the stop limit (one rule, in plain words): **the lowest of three numbers: 12 cm, the robot's own
ceiling (Left: 10 cm, Nick 29 Oct 15:41), and its floor reading less the margin; but never under 8 cm.** The floor
check can only lower a robot's limit, never raise it above what was decided for it. Left with a floor at 15 cm (the
07:43 boot) keeps 10 cm; Left with a floor at 10.6 cm gets 9.6 cm.
"""
import math
import statistics

ZONE_DEG = (22.0, 11.0, 0.0, -11.0, -22.0)    # firmware motion.h: the five front zones' directions
ZONE_NAMES = ("L", "CL", "C", "CR", "R")
PATH_HALF_CM = 5.25                            # firmware params.h path_half_cm: a reading counts when this near the path

STOP_READINGS = 3        # Nick, 2026-10-29 16:2x (audit rec 1): new readings in a row under the limit before a stop.
                         # Replay of the five DOE releases (front_stop_replay.py): 12 cm with 3 in a row leaves Left 2 %
                         # and Center 0 % of drives with a false stop (2 in a row: 69 % / 41 %). DOE: 2 / 3 on the board
                         # (Left and Right, 10 drives each) against false stops and the gap left to a real brick
FLOOR_CAP_CM = 12.0      # firmware params.h follow_look_cm default: no robot's limit goes above it (Nick: "never above
                         # 12"). No DOE: the firmware's value, kept
FLOOR_MIN_CM = 8.0       # Nick: "never below about 8 cm". Below it the robot stops for a brick with under ~6 cm left
                         # after braking (safe_front_cm 7 is the turns' limit). DOE: the board's brick test at 8 / 10 /
                         # 12 cm answers how near is still safe
ROBOT_MIN_CM = {"Left": 9.0}   # Nick (2026-10-29 20:06, question 1 of the false-stops report): Left's stop limit is never
                               # below 9 cm (the floor check may lower it from 10 to 9, not further). Others: FLOOR_MIN_CM.
                               # In the simulator 9 cm left 7.5 cm (6.7-8.4) to a brick. DOE: the board's brick test at
                               # 9 / 10 cm on Left (Monday run sheet, appendix C)


def robot_min_cm(name):
    """The lowest stop limit the floor check may give this robot."""
    return max(FLOOR_MIN_CM, float(ROBOT_MIN_CM.get(name, FLOOR_MIN_CM)))


FLOOR_MARGIN_CM = 1.0    # chosen (audit rec 4: 1 cm): the limit sits this far under the floor's 3-in-a-row level. Replay:
                         # see the build report's margin table (0.5 / 1 / 1.5 / 2 cm). DOE: worth one, on the board: the
                         # floor check then 10 standing turns per robot, margin 0.5 / 1 / 2 cm, false stops counted
FLOOR_SAMPLE_S = 2.5     # chosen: how long the robot stands while its zones are read (about 35 new readings at ~15 a
                         # second; the DOE logs had 26-32 before each drive's first move). No DOE: more only costs time
FLOOR_MIN_READINGS = 15  # fewer new readings than this: no floor check (kept as before), said so. No DOE: plumbing
FLOOR_FAR_CM = 40.0      # in-path readings this far or more are "no floor seen" (Right's zones often read 80-120 cm
                         # standing: they look past the floor). No DOE: only decides "nothing near"
FLOOR_ROBOT_AHEAD_CM = 30.0  # another robot (or a known obstacle) this near in front: its body would be read as the
                             # floor, so the check is skipped for this robot. Chosen; no DOE (a guard)

UNCONFIRMED_S = 15.0     # Nick, 2026-10-29 16:2x ("about 15 s"): a sensor-only obstacle expires this long after its
                         # stop. DOE: worth one, in the simulator: 5 / 15 / 30 s with a false stop after every lane-end
                         # turn and a real brick dropped without the camera, against "no plan" ends, the extra time,
                         # and repeat stops at the same spot (a real brick hit twice is confirmed, so it never expires)
CONFIRM_NEAR_CM = 8.0    # a second stop whose outline comes this near an unconfirmed one confirms both; the camera
                         # confirms when its obstacle is this near (as run_plan.sensor_agreement). Chosen: the zones'
                         # spread at 12 cm is about 5 cm. No DOE: the simulator's repeat-brick test checks it works


def in_path(zones, half=PATH_HALF_CM):
    """firmware motion.h frontInPath(): the nearest forward distance among the zones whose hit point lies within the
    path's half-width; 0 when none does (zeros are no reading)."""
    best = 0.0
    for d, a in zip(zones, ZONE_DEG):
        if d is None or d <= 0:
            continue
        r = math.radians(a)
        lat, fwd = d * math.sin(r), d * math.cos(r)
        if abs(lat) <= half and (best == 0.0 or fwd < best):
            best = fwd
    return best


def fresh_only(rows):
    """The new readings among RAW rows (the sensor's value is held between its readings: a repeat is not new)."""
    out, prev = [], None
    for z in rows:
        z = tuple(z)
        if z != prev:
            out.append(z)
        prev = z
    return out


def floor_level(rows, n=STOP_READINGS):
    """The floor as the stop rule sees it, from standing readings (5-zone rows, held repeats allowed).
    floor_cm: the lowest level the in-path reading stayed under for n new readings in a row (the min over every run of
    n of its max): with a limit under it, these readings would never have stopped the robot. None when no floor is in
    reach (every in-path reading FLOOR_FAR_CM or more, or none) or too few readings."""
    fr = fresh_only(rows)
    vals = []
    for z in fr:
        v = in_path(z)
        vals.append(v if 0 < v < FLOOR_FAR_CM else math.inf)
    zones = {}
    for k, nm in enumerate(ZONE_NAMES):
        xs = [z[k] for z in fr if z[k] and 0 < z[k] < FLOOR_FAR_CM]
        if xs:
            zones[nm] = dict(median=round(statistics.median(xs), 1), lowest=round(min(xs), 1), n=len(xs))
    res = dict(n=len(fr), zones=zones, floor_cm=None, lowest_cm=None, median_cm=None)
    fin = [v for v in vals if v < math.inf]
    if fin:
        res["lowest_cm"] = round(min(fin), 2)
        res["median_cm"] = round(statistics.median(fin), 2)
    if len(vals) < max(n, FLOOR_MIN_READINGS):
        res["why"] = "only %d new readings (needs %d)" % (len(vals), FLOOR_MIN_READINGS)
        return res
    lv = min(max(vals[i:i + n]) for i in range(len(vals) - n + 1))
    res["floor_cm"] = round(lv, 2) if lv < math.inf else None
    if res["floor_cm"] is None:
        res["why"] = "no floor in reach of the stop (nothing under %.0f cm in its path)" % FLOOR_FAR_CM
    return res


def stop_limit(floor_cm, ceiling_cm=FLOOR_CAP_CM, margin_cm=FLOOR_MARGIN_CM, lo_cm=FLOOR_MIN_CM, cap_cm=FLOOR_CAP_CM):
    """(limit_cm, rule): the lowest of cap, the robot's ceiling and floor - margin, but not under lo. rule says which
    one set it: "cap", "ceiling", "floor" or "minimum"."""
    ceiling_cm = min(cap_cm, ceiling_cm if ceiling_cm is not None else cap_cm)
    cands = [(cap_cm, "cap"), (ceiling_cm, "ceiling")]
    if floor_cm is not None and floor_cm < math.inf:
        cands.append((floor_cm - margin_cm, "floor"))
    lim, rule = min(cands, key=lambda c: (c[0], c[1] != "cap"))
    if lim < lo_cm:
        return round(lo_cm, 1), "minimum"
    return round(math.floor(lim * 10 + 1e-6) / 10.0, 1), rule


def limit_words(name, fl, limit_cm, rule, ceiling_cm=FLOOR_CAP_CM, margin_cm=FLOOR_MARGIN_CM):
    """One plain sentence for the operator and the log."""
    f = fl.get("floor_cm") if fl else None
    if f is None:
        why = (fl or {}).get("why") or "not checked"
        return ("%s: stop limit %.1f cm (%s; floor check: %s)"
                % (name, limit_cm, "its own setting" if rule == "ceiling" else "the usual", why))
    base = "%s: the floor reads %.1f cm at its lowest (3 readings in a row) -> stop limit %.1f cm" % (name, f, limit_cm)
    if rule == "floor":
        return base + " (%.1f cm under its floor)" % margin_cm
    if rule == "ceiling":
        return base + " (its own limit; the floor allows up to %.1f)" % min(FLOOR_CAP_CM, f - margin_cm)
    if rule == "cap":
        return base + " (the usual 12 cm; the floor is well below it)"
    return (base + " (the lowest allowed: its floor is very near, %.1f cm; it may stop for nothing, or something is in "
            "front of it: check its sensor and the floor in front of it)" % f)


class Unconfirmed:
    """Sensor-only obstacles on the planner's map: {id: dict(t, robot, points, confirmed_by)}. now: a clock function."""

    def __init__(self, now, expire_s=UNCONFIRMED_S, near_cm=CONFIRM_NEAR_CM):
        self.now, self.expire_s, self.near_cm = now, expire_s, near_cm
        self.items = {}
        self.history = []          # every one: id, robot, added t, how it ended (expired / dropped / confirmed)

    @staticmethod
    def _dist(pa, pb):
        best = math.inf
        for a in pa:
            for b in pb:
                best = min(best, math.hypot(a[0] - b[0], a[1] - b[1]))
        return best

    def near(self, points, moving=None):
        """ids of the sensor obstacles of this run (on the map now, or already expired or dropped) whose outline comes
        within near_cm of these points. An expired or dropped one counts too: a real brick the robot meets again after
        its first stop was dropped is confirmed by the second stop (simulator, 29 Oct: without this a brick no camera
        saw was dropped and met 30 times, and touched). moving (Nick, 2026-10-29 20:06): True / False for the new stop:
        an earlier stop counts only if the robot was moving at one of the two (two standing stops after turns are the
        false-stop pattern). None: any earlier stop counts (as before)."""
        return [h["id"] for h in self.history if self._dist(points, h["points"]) < self.near_cm and
                (moving is None or moving or h.get("moving") in (True, None))]

    def add(self, ob, robot, moving=None):
        rec = dict(id=ob["id"], robot=robot, t=self.now(), points=[list(p) for p in ob["points"]], end=None,
                   moving=moving)
        self.items[ob["id"]] = rec
        self.history.append(rec)
        return rec

    def confirm(self, oid, by):
        rec = self.items.pop(oid, None)
        if rec is not None:
            rec.update(end="confirmed", by=by, t_end=self.now())
        return rec

    def expired(self):
        t = self.now()
        return [k for k, v in self.items.items() if t - v["t"] >= self.expire_s]

    def drop(self, oid, how):
        rec = self.items.pop(oid, None)
        if rec is not None:
            rec.update(end=how, t_end=self.now())
        return rec

    def summary(self):
        out = {"added": len(self.history)}
        for h in self.history:
            k = h["end"] or "still on the map"
            out[k] = out.get(k, 0) + 1
        return out
