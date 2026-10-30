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
    pose_now = [x, y, h]
    esc = None
    if room < SPIN_EDGE_CM:
        # too near an edge to turn on the spot (Nick 30 Oct 10:08: no stops): first a short straight drive away from
        # the nearest edge, forwards when its front points away from it, backwards when its back does (a back-up of
        # a few cm straight away from an edge the camera has just placed it at); then the usual correction
        esc = edge_escape(rect, x, y, h)
        arc = arc_escape_backing(rect, x, y, h) if esc is None else None
        if arc is not None:
            sts, (ex, ey, eh), words = arc
            out["steps"] += sts
            out["words"].append("%s is %.1f cm from the edge and stands along it (a turn on the spot needs %.0f cm): it "
                                "first %s" % (name, room, SPIN_EDGE_CM, words))
            pose_now[0], pose_now[1], pose_now[2] = ex, ey, eh
            x, y, h = ex, ey, eh
            off_cm = math.hypot(sx - x, sy - y)
            off_deg = wrap180(h - sh)
            drive = off_cm > tol
            turn_only = not drive and abs(off_deg) > TURN_DEG
            ahead = ahead_to_edge(rect, x, y, h) - BODY_HALF_CM
            out["escape"] = [round(ex, 2), round(ey, 2)]
        elif esc is None:
            raise Refuse("%s is %.1f cm from the edge (its middle; a turn on the spot needs %.0f cm) and stands along "
                         "it, so no straight drive takes it away: it cannot turn where it is"
                         % (name, room, SPIN_EDGE_CM))
    if room < SPIN_EDGE_CM and esc is not None:
        d, rev, (ex, ey) = esc
        out["steps"].append({"type": "drive", "from": [round(x, 2), round(y, 2), round(h, 2)],
                             "to": [round(ex, 2), round(ey, 2), round(h, 2)], "speed": DRIVE_CMS, "reverse": rev,
                             "front_min_cm": None, "phase": "straighten", "why": "away from the edge"})
        out["words"].append("%s is %.1f cm from the edge (a turn on the spot needs %.0f cm): it first %s %.1f cm "
                            "straight away from it" % (name, room, SPIN_EDGE_CM, "backs" if rev else "drives", d))
        pose_now[0], pose_now[1] = ex, ey
        x, y = ex, ey
        off_cm = math.hypot(sx - x, sy - y)
        drive = off_cm > tol
        ahead = ahead_to_edge(rect, x, y, h) - BODY_HALF_CM
        out["escape"] = [round(ex, 2), round(ey, 2)]
    if room < SPIN_EDGE_CM and not drive:
        out["words"].append("%s: then turns to its start heading" % name)

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


ESCAPE_EXTRA_CM = 2.0    # an edge escape ends this far past SPIN_EDGE_CM from the edge (camera error). No DOE: a guard
ESCAPE_COS = 0.35       # its front (or back) must point at least this much away from the nearest edge (cos of the
                        # angle to the inward normal; 0.35 = within 70 deg) for a straight drive to gain room. No DOE


def edge_escape(rect, x, y, h):
    """A straight drive (cm, reverse?, end point) that takes a robot standing under SPIN_EDGE_CM from an edge to
    SPIN_EDGE_CM + ESCAPE_EXTRA_CM from every edge, along its heading; None when it stands along the edge."""
    x0, y0, x1, y1 = rect
    want = SPIN_EDGE_CM + ESCAPE_EXTRA_CM
    c, s_ = math.cos(math.radians(h)), math.sin(math.radians(h))
    best = None
    for sign in (1.0, -1.0):
        dx, dy = sign * c, sign * s_
        need = 0.0
        ok = True
        for d0, comp in ((x - x0, dx), (x1 - x, -dx), (y - y0, dy), (y1 - y, -dy)):
            if d0 < want:
                if comp < ESCAPE_COS:
                    ok = False
                    break
                need = max(need, (want - d0) / comp)
        if not ok:
            continue
        ex, ey = x + dx * need, y + dy * need
        if edge_dist(rect, ex, ey) < want - 0.01:
            continue
        cand = (round(need, 2), sign < 0, (ex, ey))
        if best is None or (cand[1] is False and best[1] is True):    # forwards preferred (the edge guard sees ahead)
            best = cand
    return best


ARC_R_CM = (30.0, 20.0)    # the arc off an edge a robot stands along: radius tried first, then the tighter one. Chosen:
                           # the outer rear corner dips (r + 4.75) - sqrt(4.75^2 + (r + 4.75)^2) toward the edge, 0.3 cm
                           # at 30, 0.5 at 20 (a spin on the spot: 2 cm). DOE: worth one on the board (20 / 30 / 40 cm
                           # against the end spot's error) only if robots are often parked along an edge
ARC_CMS = 5.0


def arc_escape(rect, x, y, h):
    """A forward arc that takes a robot standing along an edge (under SPIN_EDGE_CM from it, a straight drive gaining
    nothing) SPIN_EDGE_CM + ESCAPE_EXTRA_CM in from every edge: (step, end pose, words) or None."""
    x0, y0, x1, y1 = rect
    want = SPIN_EDGE_CM + ESCAPE_EXTRA_CM
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    lx, ly = -math.sin(math.radians(h)), math.cos(math.radians(h))      # its left
    floor_ok = min(BODY_HALF_CM + 0.3, edge_dist(rect, x, y) - 0.05)       # its wheels stay on the board
    side = 1.0 if (cx - x) * lx + (cy - y) * ly >= 0 else -1.0           # arc towards the middle of the board
    for r in ARC_R_CM:
        c = (x + side * r * lx, y + side * r * ly)
        a0 = h - 90.0 * side
        for th in range(10, 91, 2):
            sweep = side * th
            ok = True
            for k in range(0, th + 1, 2):
                a = math.radians(a0 + side * k)
                px, py = c[0] + r * math.cos(a), c[1] + r * math.sin(a)
                if edge_dist(rect, px, py) < floor_ok:
                    ok = False
                    break
            if not ok:
                break
            a = math.radians(a0 + sweep)
            ex, ey = c[0] + r * math.cos(a), c[1] + r * math.sin(a)
            if edge_dist(rect, ex, ey) >= want:
                eh = wrap180(h + sweep)
                st = {"type": "arc", "arc": {"c": [round(c[0], 2), round(c[1], 2)], "r": r, "a0_deg": round(a0, 2),
                                             "sweep_deg": round(sweep, 1)},
                      "from": [round(x, 2), round(y, 2), round(h, 2)], "to": [round(ex, 2), round(ey, 2), round(eh, 2)],
                      "speed": ARC_CMS, "front_min_cm": None, "phase": "straighten", "why": "arc off the edge"}
                return st, (ex, ey, eh), "arcs %d deg %s (radius %.0f cm) off it" % (
                    th, "left" if side > 0 else "right", r)
    return None


def arc_escape_backing(rect, x, y, h):
    """arc_escape, after backing straight along the edge first (0, 10 or 20 cm) when it faces a corner: [steps], end
    pose, words; or None."""
    for back in (0.0, 10.0, 20.0):
        bx, by = x - back * math.cos(math.radians(h)), y - back * math.sin(math.radians(h))
        if back and edge_dist(rect, bx, by) < BODY_HALF_CM + 0.1:      # its wheels stay on the board
            break
        got = arc_escape(rect, bx, by, h)
        if got is None:
            continue
        st, end, words = got
        steps = [st]
        if back:
            steps.insert(0, {"type": "drive", "from": [round(x, 2), round(y, 2), round(h, 2)],
                             "to": [round(bx, 2), round(by, 2), round(h, 2)], "speed": DRIVE_CMS, "reverse": True,
                             "front_min_cm": None, "phase": "straighten", "why": "back along the edge"})
            words = "backs %.0f cm along it, then %s" % (back, words)
        return steps, end, words
    return None


def via_points(rect, a_from, a_to, b_at, others_clear, step=6.0):
    """A point v such that a drive a_from -> v -> a_to stays the 8 cm net clear of b_at (b standing) and both legs pass
    others_clear(seg) (a function). The nearest such detour first; None if none on a 6 cm grid."""
    x0, y0, x1, y1 = rect
    gap = 2 * BODY_HALF_CM + NET_CM
    m = SPIN_EDGE_CM + ESCAPE_EXTRA_CM
    cands = []
    xs = [x0 + m + k * step for k in range(int((x1 - x0 - 2 * m) / step) + 1)]
    ys = [y0 + m + k * step for k in range(int((y1 - y0 - 2 * m) / step) + 1)]
    for vx in xs:
        for vy in ys:
            v = (vx, vy)
            if not away_clear(a_from, v, b_at, gap) or seg_point_dist(v, a_to, b_at) < gap:
                continue
            if not others_clear((a_from, v)) or not others_clear((v, a_to)):
                continue
            cands.append((math.dist(a_from, v) + math.dist(v, a_to), v))
    return min(cands)[1] if cands else None


def away_clear(a_from, v, b, gap):
    """A drive a_from -> v keeps gap from b standing; a robot that already stands within gap of b passes when it only
    moves away from b (its distance grows all along a straight drive that starts moving away) and ends gap clear."""
    if math.dist(a_from, b) >= gap:
        return seg_point_dist(a_from, v, b) >= gap
    return ((v[0] - a_from[0]) * (a_from[0] - b[0]) + (v[1] - a_from[1]) * (a_from[1] - b[1]) >= 0 and
            math.dist(v, b) >= gap)


def park_points(rect, step=6.0):
    m = SPIN_EDGE_CM + ESCAPE_EXTRA_CM
    x0, y0, x1, y1 = rect
    return [(x0 + m + i * step, y0 + m + j * step) for i in range(int((x1 - x0 - 2 * m) / step) + 1)
            for j in range(int((y1 - y0 - 2 * m) / step) + 1)]


def go_steps(pose, to_xy, final_h, why):
    """From pose (x, y, h): turn to face to_xy, drive there forwards, and turn to final_h (None: no last turn)."""
    x, y, h = pose
    bx, by = to_xy
    steps = []
    hd = math.degrees(math.atan2(by - y, bx - x)) if math.hypot(bx - x, by - y) > 0.5 else h
    if abs(wrap180(hd - h)) > TURN_DEG:
        steps.append({"type": "turn", "from": [round(x, 2), round(y, 2), round(h, 2)],
                      "to": [round(x, 2), round(y, 2), round(hd, 2)], "front_min_cm": None, "phase": "straighten",
                      "why": "face its way " + why})
    if math.hypot(bx - x, by - y) > 0.5:
        steps.append({"type": "drive", "from": [round(x, 2), round(y, 2), round(hd, 2)],
                      "to": [round(bx, 2), round(by, 2), round(hd, 2)], "speed": DRIVE_CMS, "reverse": False,
                      "front_min_cm": None, "phase": "straighten", "why": why})
    if final_h is not None and abs(wrap180(final_h - hd)) > TURN_DEG:
        steps.append({"type": "turn", "from": [round(bx, 2), round(by, 2), round(hd, 2)],
                      "to": [round(bx, 2), round(by, 2), round(final_h, 2)], "front_min_cm": None,
                      "phase": "straighten", "why": "its start heading"})
    return steps, hd


def detour_steps(name, start_pose, via, start):
    """turn, drive to via, turn, drive to the start spot, turn to the start heading (all forwards)."""
    a, hd = go_steps(start_pose, via, None, "round the other robot")
    b, _ = go_steps((via[0], via[1], hd), start[:2], start[2], "to its start spot")
    return a + b


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
                    # a detour for one of them (Nick 30 Oct: no stops): a goes round b (standing) by a via point, then
                    # b drives straight once a is on its spot; else the other way round
                    done = False
                    for m1, m2 in ((a, b), (b, a)):
                        if res[m1].get("escape"):
                            continue
                        v = via_points(rect, tuple(poses[m1][:2]), tuple(starts[m1][:2]), tuple(poses[m2][:2]),
                                       lambda seg: True)
                        if v is None or not clear(m2, m1, res[m2]["path"], tuple(starts[m1][:2])):
                            continue
                        res[m1]["steps"] = detour_steps(m1, poses[m1], v, starts[m1])
                        res[m1]["path"] = (tuple(poses[m1][:2]), tuple(starts[m1][:2]))
                        res[m1]["detour"] = [round(v[0], 1), round(v[1], 1)]
                        words.append("%s and %s would pass within the 8 cm net: %s goes round by (%.0f, %.0f) first, "
                                     "then %s" % (a, b, m1, v[0], v[1], m2))
                        after[m2] = m1
                        done = True
                        break
                    for m1, m2 in (() if done else ((a, b), (b, a))):
                        # both start spots taken by the other: m1 parks out of the way first, m2 goes to its spot, then
                        # m1 goes from its parking spot (three moves, each waiting for the one before)
                        if res[m1].get("escape") or res[m2].get("escape"):
                            continue
                        pm1, pm2 = tuple(poses[m1][:2]), tuple(poses[m2][:2])
                        sm1, sm2 = tuple(starts[m1][:2]), tuple(starts[m2][:2])
                        best = None
                        for v in park_points(rect):
                            if not away_clear(pm1, v, pm2, gap) or seg_point_dist(pm2, sm2, v) < gap or \
                                    seg_point_dist(v, sm1, sm2) < gap:
                                continue
                            c = math.dist(pm1, v) + math.dist(v, sm1)
                            if best is None or c < best[0]:
                                best = (c, v)
                        if best is None:
                            continue
                        v = best[1]
                        first, hv = go_steps(poses[m1], v, None, "out of %s's way" % m2)
                        rest, _ = go_steps((v[0], v[1], hv), sm1, starts[m1][2], "to its start spot")
                        res[m1]["steps"] = first + rest
                        res[m1]["path"] = (pm1, sm1)
                        res[m1]["detour"] = [round(v[0], 1), round(v[1], 1)]
                        res[m1]["waits"] = [(len(first), m2, len(res[m2]["steps"]) - 1)]
                        after[m2] = (m1, len(first) - 1)
                        words.append("%s and %s stand on each other's way: %s first moves aside to (%.0f, %.0f), %s "
                                     "goes to its spot, then %s goes to its own" % (a, b, m1, v[0], v[1], m2, m1))
                        done = True
                        break
                    if not done:
                        raise Refuse("%s and %s are too near each other to drive to their start spots safely (the 8 "
                                     "cm net), and no detour was found" % (a, b))
                continue
            mover, stander = (a, b) if pa is not None else (b, a)
            if not clear(mover, stander, res[mover]["path"], tuple(poses[stander][:2])):
                v = None if res[mover].get("escape") else via_points(
                    rect, tuple(poses[mover][:2]), tuple(starts[mover][:2]), tuple(poses[stander][:2]),
                    lambda seg: True)
                if v is None:
                    raise Refuse("%s's way to its start spot passes within the 8 cm net of %s, and no detour was found"
                                 % (mover, stander))
                res[mover]["steps"] = detour_steps(mover, poses[mover], v, starts[mover])
                res[mover]["detour"] = [round(v[0], 1), round(v[1], 1)]
                words.append("%s's way to its start spot passes within the 8 cm net of %s: it goes round by (%.0f, "
                             "%.0f)" % (mover, stander, v[0], v[1]))
    for b, a in after.items():
        if res[b]["steps"] and not isinstance(a, tuple):
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
    # under 7 cm from an edge facing in: drives away from it first; along the edge: refuse
    r = plan_one("Left", (17, 5, 90), (17, 17, 90), rect)
    assert r["steps"][0]["why"] == "away from the edge" and not r["steps"][0]["reverse"], r
    r = plan_one("Left", (17, 5, -90), (17, 17, 90), rect)
    assert r["steps"][0]["why"] == "away from the edge" and r["steps"][0]["reverse"], r
    r = plan_one("Left", (17, 5, 0), (17, 17, 90), rect)        # along the edge: an arc off it first
    assert r["steps"][0]["type"] == "arc" and r["steps"][0]["arc"]["sweep_deg"] > 0, r
    assert edge_dist(rect, *r["steps"][0]["to"][:2]) >= SPIN_EDGE_CM
    # two robots: ways crossing within the net -> one after the other
    p = plan_all({"Left": (50, 30, 90), "Right": (30, 45, 0)}, {"Left": (17, 17, 90), "Right": (80, 45, 90)}, rect)
    assert p["after"] == {"Right": "Left"}, p
    # swapping along one line: one goes round the other by a detour
    p = plan_all({"Left": (40, 17, 90), "Right": (25, 17, 90)}, {"Left": (17, 17, 90), "Right": (45, 17, 90)}, rect)
    assert any(r.get("detour") for r in p["robots"].values()) and p["after"], p
    assert abs(ahead_to_edge(rect, 17, 17, 90) - 44) < 1e-9
    print("straighten selftest: ok")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        _selftest()
