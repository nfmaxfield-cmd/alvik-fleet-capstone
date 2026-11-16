#!/usr/bin/env python3
"""Production points and production runs (conference-table build, Nick 2026-11-02).

  points    the numbered stickers on the table, each a production point: its label (the sticker's number), its spot
            on the table (from the site fit: clicked in the cameras' pictures), optional typed coordinates (a check).
            Saved per site in <conf_dir>/tagcal/points.json (tag_setup.py) and as spots in Mission Control's map.
  homes     the robots park near the table's middle, equidistant from it, HOME_GAP_CM apart (centre to centre), along
            the table's length, all facing the same way (+y). The middle is in both cameras' pictures.
  run       each robot: start-up (run_plan: calibrated since power-on? if not, it calibrates; calibrated today on this
            power-on: kept), then its production points in the order Nick chose, waiting DWELL_S on each (still
            readings for the external recorder), then home. The planner keeps the two 8 cm apart (the net); a point
            the robots cannot reach safely is moved in from the edge and said, never a stop.

  typed     (2026-11-03) a site may give its points typed (Jay's sticker sheet: sticker_points.py, the site's "points":
            {"use": "typed"}): the typed spots are the production points, the cameras or corner tags only fix the frame.
            "use": "clicked" is the old way (each sticker clicked), kept as the switch. The site also gives its edge
            room ("edge_in_cm"), its homes at named points ("homes": {"use": "points"}; "middle" = the old homes),
            keep-outs of its own ("keepouts": a raised box) and its default production orders ("production": zones
            per robot, nearest-neighbour order with 2-opt, or typed orders)

    python3 tools/fleet/production.py --table 396x168 --points 1:40,30 2:300,120 --order Alpha=1,2 --order Bravo=2,1
    python3 tools/fleet/production.py --site school-conference      # the site's typed points, homes, box and orders

Changelog
  2026-11-16ko  (Nick 16 Nov: school Visit points ran into the outlet plates) the paths keep the edge rule's room from a
                site's keep-outs (keepout_room_cm: measured track error + stopping distance, 4.1 cm at school), the
                points stay at their stickers; MIRLT_KEEPOUT_ROOM=body is the way before (the body + 1.5 cm only).
"""
import argparse
import copy
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import fleetmap as FM     # noqa: E402
import workspace as WS    # noqa: E402

HOME_GAP_CM = 30.0       # chosen: centre to centre of the two homes. Each robot's turn circle is 6.72 cm (the 9.5 cm
                         # square), so two spinning in place keep 30 - 2 x 6.72 = 16.6 cm, twice the 8 cm net. DOE note:
                         # 22 / 30 / 40 cm against the closest approach while both leave and come home together
HOME_FACE_DEG = 90.0     # homes face +y (across the table, away from corner 1's long side). No DOE: tidy and the same
DWELL_S = 3.0            # each production point: the robot stands this long (the external recorder gets still reads to
                         # compare). Claude's addition, not asked for: 0 turns it off. DOE note: 0 / 3 / 10 s against
                         # the RFID position's spread at a still robot
import safety_rules as SR   # noqa: E402
import safety_group as SG   # noqa: E402
EDGE_IN_CM = SR.edge_room_cm("missions")   # (e08, A2) the one derived rule (15.8 cm lights; fixed: 17). Before: 17.0
                         # a point closer than this to an open edge (the robot's centre) is moved in to it: Nick's
                         # edge room "everywhere in the mission" (missions.SETTINGS edge_room_cm, 28 Oct 21:27: the
                         # body's nearest corner 10 cm from an open edge by plan, turns included). No DOE: Nick's rule


def homes(L, W, names, gap_cm=HOME_GAP_CM, x0=0.0, y0=0.0, at=None):
    """{robot: [x, y, heading]}: equidistant from the table's middle along x, gap_cm apart, facing HOME_FACE_DEG.
    at: {robot: [x, y(, heading)]} given homes (a site's homes at named points, 2026-11-03); the others as before."""
    if at:
        out = homes(L, W, [n for n in names if n not in at], gap_cm, x0, y0) if any(n not in at for n in names) else {}
        for n in names:
            if n in at:
                h = list(at[n])
                out[n] = [round(float(h[0]), 2), round(float(h[1]), 2), float(h[2]) if len(h) > 2 else HOME_FACE_DEG]
        return {n: out[n] for n in names}
    n = len(names)
    cx, cy = x0 + L / 2.0, y0 + W / 2.0
    out = {}
    for k, nm in enumerate(names):
        off = (k - (n - 1) / 2.0) * gap_cm
        out[nm] = [round(cx + off, 2), round(cy, 2), HOME_FACE_DEG]
    return out


def clamp_points(points, L, W, room_cm=EDGE_IN_CM, x0=0.0, y0=0.0):
    """Points too near an edge moved in to room_cm. Returns ({label: [x, y]}, [words for each moved]). The table runs
    from (x0, y0) to (x0 + L, y0 + W) (an outline the feel-out moved: feel_table.py)."""
    out, words = {}, []
    for lab, xy in points.items():
        x, y = float(xy[0]), float(xy[1])
        nx = min(max(x, x0 + room_cm), x0 + L - room_cm)
        ny = min(max(y, y0 + room_cm), y0 + W - room_cm)
        if abs(nx - x) > 1e-6 or abs(ny - y) > 1e-6:
            words.append("WARNING: point %s at (%.1f, %.1f) is %.1f cm from an open edge: the robot goes to (%.1f, %.1f) "
                         "instead (its centre stays %.0f cm in)" % (lab, x, y, min(x - x0, y - y0, x0 + L - x, y0 + W - y),
                                                                    nx, ny, room_cm))
        out[str(lab).upper()] = [round(nx, 2), round(ny, 2)]
    return out, words


BODY_R_CM = 6.8           # the robot's half diagonal (9.5 cm square): a point's robot stays this far off a keep-out


def _nearest_on_poly(p, poly):
    """(nearest point on the polygon's boundary, inside?) for a convex polygon."""
    P = [np.asarray(v, float) for v in poly]
    best, bd = None, 1e18
    inside = True
    sgn = None
    for k in range(len(P)):
        a, b = P[k], P[(k + 1) % len(P)]
        ab = b - a
        t = max(0.0, min(1.0, float(np.dot(p - a, ab) / max(1e-12, np.dot(ab, ab)))))
        q = a + t * ab
        d = float(np.linalg.norm(p - q))
        if d < bd:
            best, bd = q, d
        cr = ab[0] * (p - a)[1] - ab[1] * (p - a)[0]
        if sgn is None:
            sgn = cr >= 0
        elif (cr >= 0) != sgn:
            inside = False
    return best, inside


def clear_keepouts(points, keepouts, r=BODY_R_CM):
    """Points whose robot would overlap a keep-out moved just clear of it. Returns (points, words)."""
    out, words = {}, []
    for lab, xy in points.items():
        p = np.array(xy[:2], float)
        moved = False
        for _ in range(3):
            for k in keepouts or []:
                q, inside = _nearest_on_poly(p, k["polygon"])
                d = float(np.linalg.norm(p - q))
                if inside or d < r:
                    v = (p - q) / d if d > 1e-9 else np.array([0.0, 1.0])
                    if inside:
                        c = np.mean(np.asarray(k["polygon"], float), axis=0)
                        v = (q - c) / max(1e-9, np.linalg.norm(q - c))
                    p = q + v * (r + 0.5)
                    moved = k.get("name") or "a keep-out"
        if moved:
            what = "a reference tile" if str(moved).startswith("reference") else "the " + str(moved)
            words.append("WARNING: point %s at (%.1f, %.1f) was on or beside %s: the robot goes to "
                         "(%.1f, %.1f) instead" % (lab, xy[0], xy[1], what, p[0], p[1]))
        out[lab] = [round(float(p[0]), 2), round(float(p[1]), 2)]
    return out, words


def site_map(table, names, points=None, gap_cm=HOME_GAP_CM, outline=None, keepouts=None, room_cm=EDGE_IN_CM,
             home_at=None, keepout_r=BODY_R_CM, plan_keepouts=None):
    """The workspace map of a site's table: the measured rectangle, every side an open edge (a conference table has no
    walls), the production points as spots, each robot's start and park at its home. outline [x0, y0, x1, y1]: the
    table's outline when it is not (0, 0, L, W) (the feel-out moved a side: feel_table.py); table is then ignored.
    keepouts: areas no robot drives over (the reference tags' tiles, 2026-11-03: ref_tags.keepouts; a site's own, e.g.
    the school's raised box). room_cm: the edge room for the points (a site's edge_in_cm). home_at: {robot: [x, y,
    heading]} homes at given spots (a site's homes at points), else the middle homes."""
    if outline:
        x0, y0, x1, y1 = (float(v) for v in outline)
    else:
        x0, y0, x1, y1 = 0.0, 0.0, float(table[0]), float(table[1])
    L, W = x1 - x0, y1 - y0
    hs = homes(L, W, names, gap_cm, x0=x0, y0=y0, at=home_at)
    m = WS.blank_map((x0, y0, x1, y1), [(n, hs[n]) for n in names])
    for k in FM.SIDES:
        m = WS.set_side(m, k, "open", source="conference table (Nick: no walls)")
    pts, words = clamp_points(points or {}, L, W, room_cm=room_cm, x0=x0, y0=y0)
    m = copy.deepcopy(m)
    if keepouts:
        pts, w2 = clear_keepouts(pts, keepouts, r=keepout_r)
        words = words + w2
    m["spots"] = pts
    if keepouts:
        # (2026-11-16) plan_keepouts: the same keep-outs widened for the paths (keepout_room_cm); the points stay where
        # the stickers are (cleared from the measured outline only)
        m["keepouts"] = list(m.get("keepouts") or []) + [dict(k) for k in (plan_keepouts or keepouts)]
    m["notes"] = ["production points from the site fit"] + words
    m.setdefault("homes", hs)
    return FM.normalize(m), words


def save_map(mw, path):
    m = mw[0] if isinstance(mw, tuple) else mw
    FM.save(m, path)
    return path


def program(orders, dwell_s=DWELL_S, home=True, name="production", at_home=None):
    """A production program (missions.build blocks). orders: {robot: [point labels in order]}. at_home: {robot: label}
    of the point at its home (2026-11-03): when it is the robot's first point it is done where the robot stands (the
    stand only, no drive away and back)."""
    blocks = []
    robots = list(orders)
    at_home = {n: str(v).upper() for n, v in (at_home or {}).items()}
    for n, labs in orders.items():
        for k, lab in enumerate(labs):
            if k == 0 and at_home.get(n) == str(lab).upper():
                blocks.append({"block": "wait", "robots": [n], "seconds": float(max(dwell_s, 0.5)),
                               "label": "at point %s" % str(lab).upper()})
                continue
            blocks.append({"block": "go-to-spot", "robot": n, "spot": str(lab).upper(),
                           "label": "point %s" % str(lab).upper()})
            if dwell_s > 0:
                blocks.append({"block": "wait", "robots": [n], "seconds": float(dwell_s),
                               "label": "at point %s" % str(lab).upper()})
        if home:
            blocks.append({"block": "go-home", "robot": n})
    return {"name": name, "robots": robots, "blocks": blocks, "production": {"orders": orders, "dwell_s": dwell_s,
                                                                             "home": home, "at_home": at_home}}


# ---------------------------------------------------------------- a site's own production setup (2026-11-03)
def site_edge_cm(site):
    """The site's edge room for the points: "edge_in_cm" (school: 13), else EDGE_IN_CM (home: 17, as before)."""
    v = (site or {}).get("edge_in_cm")
    if SR.EDGE_RULE != "fixed" and site:
        # (e08, A2) the one derived rule for this site's tracking (roof tags: no camera edge bias): school 10.8 cm. The
        # site's typed edge_in_cm is the fixed rule's (MIRLT_EDGE_RULE=fixed)
        tr = site.get("run_tracking") or site.get("tracking") or "lights"
        return SR.edge_room_cm("school" if tr == "tags" else "missions", tracking=tr)
    return EDGE_IN_CM if v is None else float(v)


# ---------------------------------------------------------------- (2026-11-16) room round a keep-out
def keepout_room_cm(site=None, speed=None):
    """(Nick 16 Nov: school Visit points ran into the outlet plates) The room the paths keep from a site's keep-out (an
    outlet plate, the middle box, a reference tile) beyond the robot's body and the planner's 1.5 cm safety margin: the
    one derived edge rule's own terms, without the body the planner already counts (safety_rules.clearance_cm less
    HALF_DIAG_CM): the measured track error (TRACK_P95_CM, 1.7 cm), the stopping distance at that speed (the largest
    measured, 2.4 cm, when not known) and the camera's edge bias for the site's tracking (roof tags 0). School: 4.1 cm.
    Before: 0 (the body and 1.5 cm only), behind the Safety switch MIRLT_KEEPOUT_ROOM=body."""
    if SG.value("MIRLT_KEEPOUT_ROOM") == "body":
        return 0.0
    tr = (site or {}).get("run_tracking") or (site or {}).get("tracking") or "lights"
    return round(SR.clearance_cm(SR.TRACK_P95_CM, speed, tr, stops_here=True) - SR.HALF_DIAG_CM, 2)


def widen_polygon(poly, d, n=16):
    """A convex keep-out grown by d cm all round (rounded corners): its corners swept by a 16-sided circle drawn round
    the true one (so the grown outline is never inside the true offset), then the convex hull."""
    if d <= 0:
        return [list(map(float, q)) for q in poly]
    r = d / math.cos(math.pi / n)
    pts = [(float(x) + r * math.cos(2 * math.pi * k / n), float(y) + r * math.sin(2 * math.pi * k / n))
           for x, y in poly for k in range(n)]
    pts = sorted(set(pts))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, hi = [], []
    for q in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], q) <= 0:
            lo.pop()
        lo.append(q)
    for q in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], q) <= 0:
            hi.pop()
        hi.append(q)
    return [[round(a, 2), round(b, 2)] for a, b in lo[:-1] + hi[:-1]]


def widen_keepouts(keepouts, d):
    """Each keep-out widened by d cm for the paths; its measured outline kept as "measured_polygon"."""
    out = []
    for k in keepouts or []:
        k2 = dict(k)
        if d > 0:
            k2["measured_polygon"] = k["polygon"]
            k2["polygon"] = widen_polygon(k["polygon"], d)
            k2["note"] = ("%s; paths keep %.1f cm more (track error + stopping distance, keepout_room_cm)" % (
                k.get("note") or k.get("source") or "keep-out", d))
        out.append(k2)
    return out


def turn_room(L, W):
    """The room the planner needs round a point to turn there (its turn_thr: the body's turn circle + its safety margin,
    8.2 cm today) + 0.3 cm: a point beside a site keep-out is moved this far clear, so it is reached, not skipped (the
    school's first plan skipped P12 and P21 at 7.2-7.3 cm from the 12 x 12 in box with BODY_R_CM)."""
    import planner as PL
    try:
        m, _ = site_map((L, W), ["a"])
        return float(PL.Planner(m).grid.turn_thr) + 0.3
    except Exception:      # noqa: BLE001
        return BODY_R_CM + 2.0


def site_keepouts(site):
    """The site's own keep-outs ("keepouts": [{name, centre_in or centre_cm, size_in or size_cm, solid, note}]):
    rectangles on the table (the school's raised box, 12 x 12 in until Nick measures it). The planner keeps the
    robot's body clear of each (its own clearance); a point beside one is moved clear of it (clear_keepouts)."""
    out = []
    for k in (site or {}).get("keepouts") or []:
        if k.get("use", True) is False:
            continue
        if "centre_in" in k:
            cx, cy = (float(v) * 2.54 for v in k["centre_in"])
        else:
            cx, cy = (float(v) for v in k["centre_cm"])
        if "size_in" in k:
            sx, sy = (float(v) * 2.54 for v in k["size_in"])
        else:
            sx, sy = (float(v) for v in k["size_cm"])
        poly = [[cx - sx / 2, cy - sy / 2], [cx + sx / 2, cy - sy / 2], [cx + sx / 2, cy + sy / 2],
                [cx - sx / 2, cy + sy / 2]]
        d = {"name": k.get("name", "keep-out"), "polygon": [[round(a, 2), round(b, 2)] for a, b in poly],
             "source": "site keep-out (%s)" % (k.get("note") or "site file")}
        if k.get("solid", True):
            d["solid"] = True
        out.append(d)
    return out


def site_home_at(site, points):
    """{robot: [x, y, heading]} when the site's homes are at named points ("homes": {"use": "points", "Alpha": "P11",
    ...}), else None (the middle homes, as before: "use": "middle")."""
    h = (site or {}).get("homes") or {}
    if h.get("use", "middle") != "points":
        return None
    face = float(h.get("face_deg", HOME_FACE_DEG))
    out = {}
    for r in (site or {}).get("robots") or []:
        lab = h.get(r["name"])
        if lab is None:
            continue
        lab = str(lab).upper()
        if lab not in points:
            raise ValueError("home of %s is point %s, which is not a production point" % (r["name"], lab))
        out[r["name"]] = [points[lab][0], points[lab][1], face]
    return out or None


def site_home_points(site):
    h = (site or {}).get("homes") or {}
    if h.get("use", "middle") != "points":
        return {}
    return {r["name"]: str(h[r["name"]]).upper() for r in (site or {}).get("robots") or [] if h.get(r["name"])}


def tour_len(start, labs, pts, back=True):
    p, d = start, 0.0
    for l in labs:
        d += math.hypot(pts[l][0] - p[0], pts[l][1] - p[1])
        p = pts[l]
    if back:
        d += math.hypot(start[0] - p[0], start[1] - p[1])
    return d


def short_order(start, labs, pts, back=True):
    """A short order through labs from start (and back): nearest neighbour, then 2-opt until nothing shortens it.
    Not the proven shortest; within a few % of it for 20 points."""
    left, p, out = list(labs), start, []
    while left:
        nxt = min(left, key=lambda l: (math.hypot(pts[l][0] - p[0], pts[l][1] - p[1]), l))
        out.append(nxt)
        left.remove(nxt)
        p = pts[nxt]
    best = tour_len(start, out, pts, back)
    better = True
    while better:
        better = False
        for i in range(len(out) - 1):
            for j in range(i + 1, len(out)):
                cand = out[:i] + out[i:j + 1][::-1] + out[j + 1:]
                c = tour_len(start, cand, pts, back)
                if c < best - 1e-6:
                    out, best, better = cand, c, True
    return out


def site_orders(site, points, zones, home_xy):
    """The site's default production orders: "production": {"orders": {robot: [labels]}} as typed, else each robot's
    zones ("zones": {"Alpha": [1, 2], ...}) in a short order from its home and back; the point at its home first."""
    pr = (site or {}).get("production") or {}
    if pr.get("orders"):
        return {n: [str(x).upper() for x in v] for n, v in pr["orders"].items()}
    hp = site_home_points(site)
    out = {}
    for n, zs in (pr.get("zones") or {}).items():
        labs = [k for k in points if zones.get(k) in set(int(z) for z in zs)]
        first = [hp[n]] if hp.get(n) in labs else []
        rest = [k for k in labs if k not in first]
        out[n] = first + short_order(home_xy[n][:2], rest, points)
    return out


def site_production(site, names=None, table=None, outline=None, keepouts=None, seen=None, experiment=None, swap=False,
                    wiggle=None, home_at=None, orders=None, keep_order=False, dwell_s=None):
    """A site with typed points (sticker_points.py): (map, words, program, orders). table: (L, W) measured (else the
    site's typed size); keepouts: more keep-outs (the reference tiles); seen: {label: [x, y]} stickers the cameras
    fitted (the old click way): any far from its typed spot is flagged, never moved.
    experiment (2026-11-03, Jay-Sun's training runs; experiments.py): shakedown / e1-e4 / default: that experiment's
    orders (swap: the robots' halves exchanged), recorded in program["production"]["experiment"] with the pinned
    speeds; None = the site's default orders as before. wiggle: True / False / None (the site's sync_wiggle.use): the
    sync wiggle block at the start."""
    import sticker_points as SP
    names = names or [r["name"] for r in site["robots"]]
    pts, zones, words = SP.points_cm(site)
    tb = site.get("table") or {}
    L, W = table or (float(tb.get("length_cm") or 396.24), float(tb.get("width_cm") or 167.64))
    at = site_home_at(site, pts)
    if home_at:
        at = dict(at or {})
        at.update(home_at)                 # (11-05h) homes where the robots were SEEN standing (observe)
    kos = site_keepouts(site) + list(keepouts or [])
    kroom = keepout_room_cm(site)
    m, w2 = site_map((L, W), names, points=pts, outline=outline, keepouts=kos, room_cm=site_edge_cm(site),
                     home_at=at, keepout_r=turn_room(L, W), plan_keepouts=widen_keepouts(kos, kroom))
    if kos and kroom > 0:
        w2 = list(w2) + ["keep-outs (%s): the paths keep %.1f cm more than the robot's body from each (its measured track "
                         "error and stopping distance; MIRLT_KEEPOUT_ROOM=body: as before)" % (
                             ", ".join(k.get("name", "keep-out") for k in kos), kroom)]
    words = list(words) + list(w2) + SP.flag_seen(pts, seen)
    m["notes"] = ["production points typed (%s)" % os.path.basename(SP.points_file(site))] + words
    homes_ = {r["name"]: r["start"] for r in m["robots"]}
    meta = None
    given = orders
    if experiment and experiment != "custom":
        import experiments as EX
        orders, meta = EX.experiment_orders(site, m["spots"], zones, homes_, experiment, swap=swap)
    elif given is not None:
        # (11-05h) the table shared from what is seen (observe.split_points): each robot's points in a short order
        # from where it stands, the point it stands on first
        orders = {}
        for n, labs in given.items():
            labs = [l for l in labs if l in m["spots"]]
            hx = homes_.get(n, [0, 0])[:2]
            if keep_order:
                orders[n] = list(labs)          # (e07) free roam at a tag site: the given (random) order as it is
                continue
            first = [l for l in labs if math.hypot(m["spots"][l][0] - hx[0], m["spots"][l][1] - hx[1]) < 3.0][:1]
            rest = [l for l in labs if l not in first]
            orders[n] = first + short_order(hx, rest, m["spots"])
    else:
        orders = site_orders(site, m["spots"], zones, homes_)
    orders = {n: v for n, v in orders.items() if n in names}
    pr = site.get("production") or {}
    ah = site_home_points(site)
    if given is not None:
        ah = {}
        for n, labs in orders.items():
            hx = homes_.get(n, [0, 0])[:2]
            if labs and math.hypot(m["spots"][labs[0]][0] - hx[0], m["spots"][labs[0]][1] - hx[1]) < 3.0:
                ah[n] = labs[0]
    prog = program(orders, dwell_s=float(pr.get("dwell_s", DWELL_S) if dwell_s is None else dwell_s), at_home=ah)
    if meta is not None:
        prog["production"]["experiment"] = meta
        prog["name"] = "production %s%s" % (experiment, " (halves swapped)" if swap else "")
    import experiments as EX
    if EX.wiggle_on(site, {True: "on", False: "off"}.get(wiggle)):
        prog = EX.add_wiggle(prog, site)
    return m, words, prog, orders


def parse_orders(specs):
    out = {}
    for s in specs:
        n, _, labs = s.partition("=")
        out[n.strip()] = [x.strip().upper() for x in labs.split(",") if x.strip()]
    return out


def check_orders(orders, points):
    """Plain words for any label that is not a point (raised as ValueError by the caller)."""
    bad = []
    for n, labs in orders.items():
        for lab in labs:
            if str(lab).upper() not in points:
                bad.append("%s: no point %s (points: %s)" % (n, lab, ", ".join(sorted(points)) or "none yet"))
    return bad


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--table", default="396x168")
    ap.add_argument("--points", nargs="*", default=[], help="LABEL:x,y")
    ap.add_argument("--order", action="append", default=[], help="ROBOT=1,2,3")
    ap.add_argument("--out", default="")
    ap.add_argument("--site", default="", help="a site with typed points: its map, words and default orders")
    a = ap.parse_args(argv)
    if a.site:
        import sites as SI
        m, words, prog, orders = site_production(SI.load(a.site))
        print("\n".join(words))
        for n, labs in orders.items():
            print("%s: %s" % (n, ", ".join(labs)))
        if a.out:
            os.makedirs(a.out, exist_ok=True)
            save_map(m, os.path.join(a.out, "map.json"))
            with open(os.path.join(a.out, "program.json"), "w") as f:
                json.dump(prog, f, indent=1)
        return 0
    L, W = (float(v) for v in a.table.split("x"))
    pts = {}
    for p in a.points:
        lab, _, xy = p.partition(":")
        pts[lab] = [float(v) for v in xy.split(",")]
    orders = parse_orders(a.order)
    m, words = site_map((L, W), list(orders), pts)
    bad = check_orders(orders, m["spots"])
    if bad:
        raise SystemExit("; ".join(bad))
    prog = program(orders)
    print("\n".join(words))
    print(json.dumps(prog, indent=1))
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        save_map(m, os.path.join(a.out, "map.json"))
        with open(os.path.join(a.out, "program.json"), "w") as f:
            json.dump(prog, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
