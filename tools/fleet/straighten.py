#!/usr/bin/env python3
"""Start straightening (Nick, 2026-10-29 15:41): "Before Monday's run, 15 degrees should result in a correction, not a
stop."

Before the timed mission, each robot's measured pose (the camera's position, the facing check's heading from its own
two lights) is compared with its start pose in the plan:
  - heading off by more than TURN_DEG (any amount that is not measuring noise, backwards included): it turns on the
    spot to its start heading;
  - position off by more than POS_TOL_CM (POS_TOL_SIDE_CM when only the side camera sees it): it turns to face its start
    spot, drives there forwards on a straight, camera-watched line, then turns to its start heading. Never backwards
    (backing is blind to the edge guard). A robot facing an edge it is under EDGE_AHEAD_CM from turns first, so the
    firmware's edge guard never meets it;
  - then the facing check runs again (re-check); a robot still off gets one more correction; still off after that, or
    one that cannot be found, stops the start with plain words.
run_plan.py does the moving (--straighten, passed by mission_run.py: Monday and Mission Control); this module plans
the steps and says the words. Self-test: python3 tools/fleet/straighten.py --selftest
"""
import math
import sys

TURN_DEG = 3.0          # a heading further off than this is turned. Nick: "off by ANY amount"; 3 deg is the facing
                        # check's own spread (side camera median 1.4 deg, 28 Oct; simulator 1-3 deg), so a smaller
                        # "error" is the measure, not the robot. DOE: 2 / 3 / 5 deg against the first leg's line check
                        # p95 and the time spent turning (board, 10 starts each)
POS_TOL_CM = 3.0        # both cameras (or the overhead) see it: further off than this, it drives to its spot. Chosen:
                        # the bridge's "moved" rule (cam_bridge.MOVED_CM). DOE: 2 / 3 / 4 cm against the first leg's
                        # p95 and the start's time
POS_TOL_SIDE_CM = 5.0   # only the side camera sees it (Nick, question 60: the side camera alone reads ~3 cm off)
RECHECK_DEG = 8.0       # after a correction, the re-check passes within this. The mission's first leg ran whole at 12
                        # and 15 deg assumed (facing check dry runs, 29 Oct), and now the robot is told its measured
                        # heading. DOE: 5 / 8 / 12 deg against second corrections and the first leg's p95
TRIES = 2               # corrections before the start stops (Nick: "only stop if ... the correction fails twice")
SPIN_EDGE_CM = 7.0      # a turn on the spot needs the robot's middle this far from every edge (Nick: "the spin needs
                        # about 7 cm"; the 9.5 cm body's corners sweep 6.72 cm). Chosen; DOE note: 7 / 8 / 9 cm, the
                        # camera's position error near edges (overhead alone up to 4 cm, calibration-room report)
EDGE_AHEAD_CM = 12.0    # the firmware's edge guard refuses forward motion towards an open edge this close (Nick)
DRIVE_CMS = 7.5         # the drive to its spot. Chosen: the camera bridge's travel heading needs 5 cm in 1 s
                        # (cam_bridge.HEADING_MIN_CM), so 5 cm/s never gets one; 7.5 does. DOE: 5 / 7.5 / 10 cm/s
                        # against the end position error
TURN_DPS = 60.0         # for the time estimate only
BODY_HALF_CM = 4.75     # the 9.5 cm body
NET_CM = 8.0            # bodies never closer than the 8 cm net while straightening (the 8 cm lock)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def edge_dist(rect, x, y):
    x0, y0, x1, y1 = rect
    return min(x - x0, x1 - x, y - y0, y1 - y)


def ahead_to_edge(rect, x, y, h):
    """Distance from the robot's middle along heading h to the board's edge, cm."""
    x0, y0, x1, y1 = rect
    c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
    ds = []
    if c > 1e-9:
        ds.append((x1 - x) / c)
    if c < -1e-9:
        ds.append((x0 - x) / c)
    if s > 1e-9:
        ds.append((y1 - y) / s)
    if s < -1e-9:
        ds.append((y0 - y) / s)
    return min(ds) if ds else math.inf


def side_words(rect, h):
    """'the y = 61 side' for the side a heading points at."""
    x0, y0, x1, y1 = rect
    c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
    long_x = abs(x1 - x0) >= abs(y1 - y0)
    if abs(c) >= abs(s):
        return "the x = %g %s" % (round(x1 if c > 0 else x0, 1), "end" if long_x else "side")
    return "the y = %g %s" % (round(y1 if s > 0 else y0, 1), "side" if long_x else "end")


def seg_point_dist(a, b, p):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(ax + t * dx - p[0], ay + t * dy - p[1])


def seg_seg_dist(a, b, c, d):
    return min(seg_point_dist(a, b, c), seg_point_dist(a, b, d), seg_point_dist(c, d, a), seg_point_dist(c, d, b))


class Refuse(Exception):
    """The start stops: words say why and what to do (nothing unsafe was sent)."""


def plan_one(name, pose, start, rect, side_only=False):
    """One robot's correction from its measured pose (x, y, heading) to its start (x, y, heading).
    Returns dict(steps=[...], words=[...], path=((x, y), (x, y)) or None, need=bool, off_cm, off_deg, ahead_cm)."""
    x, y, h = pose
    sx, sy, sh = start
    off_cm = math.hypot(sx - x, sy - y)
    off_deg = wrap180(h - sh)
    tol = POS_TOL_SIDE_CM if side_only else POS_TOL_CM
    ahead = ahead_to_edge(rect, x, y, h) - BODY_HALF_CM        # its nose to the edge it faces
    out = dict(steps=[], words=[], path=None, need=False, off_cm=round(off_cm, 2), off_deg=round(off_deg, 1),
               ahead_cm=round(ahead, 1), tol_cm=tol)
    drive = off_cm > tol
    turn_only = not drive and abs(off_deg) > TURN_DEG
    if not drive and not turn_only:
        out["words"].append("%s is on its start spot (%.1f cm, %.0f deg off): no correction" % (name, off_cm,
                                                                                              abs(off_deg)))
        return out
    out["need"] = True
    room = edge_dist(rect, x, y)
    if room < SPIN_EDGE_CM:
        raise Refuse("%s is %.1f cm from the edge (its middle; a turn on the spot needs %.0f cm): it cannot turn "
                     "where it is. Nothing moved. Move it in by hand onto its start spot and start again."
                     % (name, room, SPIN_EDGE_CM))
    pose_now = [x, y, h]

    def turn_to(hh, why):
        if abs(wrap180(hh - pose_now[2])) <= TURN_DEG:
            return
        out["steps"].append({"type": "turn", "from": [round(pose_now[0], 2), round(pose_now[1], 2),
                                                     round(pose_now[2], 2)],
                             "to": [round(pose_now[0], 2), round(pose_now[1], 2), round(wrap180(hh), 2)],
                             "front_min_cm": None, "phase": "straighten", "why": why})
        pose_now[2] = wrap180(hh)

    if drive:
        to_spot = math.degrees(math.atan2(sy - y, sx - x))
        if ahead < EDGE_AHEAD_CM:
            out["words"].append("%s faces %s with its nose %.1f cm from it: it turns on the spot first (no backing)"
                                % (name, side_words(rect, h), max(0.0, ahead)))
        turn_to(to_spot, "face its start spot")
        out["steps"].append({"type": "drive", "from": [round(x, 2), round(y, 2), round(to_spot, 2)],
                             "to": [round(sx, 2), round(sy, 2), round(to_spot, 2)], "speed": DRIVE_CMS,
                             "reverse": False, "front_min_cm": None, "phase": "straighten", "why": "to its start spot"})
        pose_now[0], pose_now[1] = sx, sy
        out["path"] = ((x, y), (sx, sy))
        turn_to(sh, "its start heading")
        out["words"].append("%s was %.1f cm off its start spot (allowed %.0f cm%s) and %.0f deg off its start heading: "
                            "it turns, drives %.1f cm to its spot, and turns to face %s"
                            % (name, off_cm, tol, ", side camera alone" if side_only else "", abs(off_deg), off_cm,
                               side_words(rect, sh)))
    else:
        if ahead < EDGE_AHEAD_CM:
            out["words"].append("%s faces %s with its nose %.1f cm from it: it turns on the spot (no backing)"
                                % (name, side_words(rect, h), max(0.0, ahead)))
        turn_to(sh, "its start heading")
        out["words"].append("%s was %.0f deg off%s: it turns on the spot to face %s"
                            % (name, abs(off_deg), " (backwards)" if abs(off_deg) > 135 else "", side_words(rect, sh)))
    return out


def done_words(name, r, rect, start_h):
    """After a correction, in plain words: 'Left was 22 deg off: turned to face the y = 61 side'."""
    face = side_words(rect, start_h)
    if r.get("path") is None:
        return "%s was %.0f deg off%s: turned to face %s" % (name, abs(r["off_deg"]),
                                                            " (backwards)" if abs(r["off_deg"]) > 135 else "", face)
    return "%s was %.1f cm off its start spot%s: turned, drove %.1f cm to its spot, and faces %s" % (
        name, r["off_cm"], "" if abs(r["off_deg"]) <= TURN_DEG else " and %.0f deg off" % abs(r["off_deg"]),
        r["off_cm"], face)


def est_s(steps):
    t = 0.0
    for st in steps:
        if st["type"] == "turn":
            t += abs(wrap180(st["to"][2] - st["from"][2])) / TURN_DPS + 1.0
        elif st["type"] == "drive":
            t += math.hypot(st["to"][0] - st["from"][0], st["to"][1] - st["from"][1]) / st["speed"] + 1.0
    return t


def plan_all(poses, starts, rect, side_only=None):
    """Every robot's correction. poses / starts: {name: (x, y, heading)}. Robots that would come within the 8 cm net of
    each other go one after the other (the second waits for the first's last step); if even that is too close, the
    start stops. Returns dict(robots={name: plan_one(...)}, order=[...], words=[...])."""
    side_only = side_only or {}
    res = {n: plan_one(n, poses[n], starts[n], rect, side_only.get(n, False)) for n in poses}
    words = []
    for n in res:
        words += res[n]["words"]
    gap = 2 * BODY_HALF_CM + NET_CM

    def clear(a, b, a_path, b_where):
        """a drives a_path while b stands at b_where (a point) or drives (a segment)."""
        if a_path is None:
            return True
        if isinstance(b_where[0], tuple):
            return seg_seg_dist(a_path[0], a_path[1], b_where[0], b_where[1]) >= gap
        return seg_point_dist(a_path[0], a_path[1], b_where) >= gap

    names = list(res)
    after = {}
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            pa, pb = res[a]["path"], res[b]["path"]
            pos_a, pos_b = tuple(poses[a][:2]), tuple(poses[b][:2])
            st_a, st_b = tuple(starts[a][:2]), tuple(starts[b][:2])
            if pa is None and pb is None:
                continue
            if pa is not None and pb is not None:
                if clear(a, b, pa, pb):
                    continue
                # one after the other: a first (b standing where it is), then b (a on its start spot)
                if clear(a, b, pa, pos_b) and clear(b, a, pb, st_a):
                    after[b] = a
                elif clear(b, a, pb, pos_a) and clear(a, b, pa, st_b):
                    after[a] = b
                else:
                    raise Refuse("%s and %s are too near each other to drive to their start spots safely (the 8 cm "
                                 "net). Nothing moved. Put them on their start spots by hand and start again." % (a, b))
                continue
            mover, stander = (a, b) if pa is not None else (b, a)
            if not clear(mover, stander, res[mover]["path"], tuple(poses[stander][:2])):
                raise Refuse("%s's way to its start spot passes within the 8 cm net of %s. Nothing moved. Put %s on its "
                             "start spot by hand and start again." % (mover, stander, mover))
    for b, a in after.items():
        if res[b]["steps"]:
            words.append("%s waits until %s has reached its spot (their ways would come within the 8 cm net)" % (b, a))
    return dict(robots=res, after=after, words=words)


def recheck(name, pose, start, side_only=False):
    """After a correction: (ok, words)."""
    x, y, h = pose
    sx, sy, sh = start
    d = math.hypot(sx - x, sy - y)
    dh = abs(wrap180(h - sh))
    tol = POS_TOL_SIDE_CM if side_only else POS_TOL_CM
    ok = d <= tol and dh <= RECHECK_DEG
    return ok, "%s: re-check %.1f cm and %.0f deg off its start (allowed %.0f cm, %.0f deg): %s" % (
        name, d, dh, tol, RECHECK_DEG, "ready" if ok else "still off")


def _selftest():
    rect = (0.0, 0.0, 122.0, 61.0)
    # on its spot: nothing
    r = plan_one("Left", (17.5, 17.2, 91.0), (17, 17, 90), rect)
    assert not r["need"] and not r["steps"], r
    # 20 deg off: one turn
    r = plan_one("Left", (17, 17, 110), (17, 17, 90), rect)
    assert [s["type"] for s in r["steps"]] == ["turn"] and r["steps"][0]["to"][2] == 90, r
    # backwards: one turn of 180
    r = plan_one("Left", (17, 17, -90), (17, 17, 90), rect)
    assert len(r["steps"]) == 1 and abs(wrap180(r["steps"][0]["to"][2] - r["steps"][0]["from"][2])) == 180, r
    assert "backwards" in r["words"][-1], r["words"]
    # 8 cm off: turn, drive forwards, turn
    r = plan_one("Left", (25, 17, 90), (17, 17, 90), rect)
    assert [s["type"] for s in r["steps"]] == ["turn", "drive", "turn"], r
    assert not r["steps"][1]["reverse"]
    # side camera alone 4 cm off: allowed (5 cm)
    assert not plan_one("Left", (21, 17, 90), (17, 17, 90), rect, side_only=True)["need"]
    assert plan_one("Left", (21, 17, 90), (17, 17, 90), rect)["need"]
    # facing the y = 0 edge 9 cm in: turns first, says so, no backing
    r = plan_one("Left", (17, 9, -90), (17, 17, 90), rect)
    assert r["steps"][0]["type"] == "turn" and "no backing" in " ".join(r["words"]), r
    assert all(not s.get("reverse") for s in r["steps"])
    # under 7 cm from an edge: refuse
    try:
        plan_one("Left", (17, 5, 90), (17, 17, 90), rect)
        raise AssertionError("no refuse")
    except Refuse:
        pass
    # two robots: ways crossing within the net -> one after the other
    p = plan_all({"Left": (50, 30, 90), "Right": (30, 45, 0)}, {"Left": (17, 17, 90), "Right": (80, 45, 90)}, rect)
    assert p["after"] == {"Right": "Left"}, p
    # swapping along one line: no safe order -> refuse
    try:
        plan_all({"Left": (40, 17, 90), "Right": (25, 17, 90)}, {"Left": (17, 17, 90), "Right": (45, 17, 90)}, rect)
        raise AssertionError("no refuse")
    except Refuse:
        pass
    assert abs(ahead_to_edge(rect, 17, 17, 90) - 44) < 1e-9
    print("straighten selftest: ok")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        _selftest()
