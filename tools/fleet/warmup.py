#!/usr/bin/env python3
"""Warm-up on a new table (school demo, docs/build-plans/mission-control.md, 2026-10-27).

Starts from the table's rough rectangle (marked by Nick in MIRLT, or the camera's guess), every side "unknown".
The robots, through the ordinary planner and run_plan.py:
  1. creep into the corners and pause there (CORNER_PAUSE_S): the camera frames of those pauses confirm the corners;
  2. feel each side at a few points (a slow straight creep past the marked line, PROBE_*): the edge guard stopping it
     = an open edge there, the front sensor stopping it close to the line = a barrier, stopping well before the line
     = something standing there (a no-go zone proposed for Nick), nothing stopping it = the table goes on further
     than marked (shown to Nick);
  3. then go back to where they started.
analyze() reads the session folder and gives: each side's kind (open / barrier / partly both with the barrier
stretches), where each side really is against the marked line, proposed no-go zones, the frame list for Nick's check
(at most MAX_FRAMES: about 6 s each, 90 s in all), and each robot's stored wheel and gyro scales against the camera.
apply() writes the confirmed answers into the map.

    python3 tools/fleet/warmup.py plan --map table.json --out warm_map.json        # the warm-up's tasks
    python3 tools/fleet/run_plan.py --map warm_map.json --border-cm 0 ...          # drive it (border off: the
                                                                                   #  creeps go to the edge on purpose)
    python3 tools/fleet/warmup.py analyze --map warm_map.json --session logs/...   # sides, zones, frames, scales
"""
import argparse
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import fleetmap as FM    # noqa: E402
import workspace as WS   # noqa: E402

CORNER_IN_CM = 13.0      # chosen: the corner pause point, the robot's centre this far in from both sides: the least the
                         # planner allows near an unknown (open) side is r_turn 6.7 + safety 1.5 + open margin 3 = 11.2 cm.
                         # DOE: not needed (it only has to be planner-safe and in the camera's view)
CORNER_PAUSE_S = 2.0     # chosen: about 50 camera pictures at 25 a second, plenty for "is this the corner?". No DOE
PROBE_SPACING_CM = 40.0  # chosen: one side feel per 40 cm of side (2 on a 76 cm side, 4 on 152 cm). A barrier shorter
                         # than this between two feels can be missed. DOE note: 20 / 40 / 60 cm on the school table
                         # (barrier stretches found against warm-up minutes); docs/doe-backlog.md
PROBE_START_CM = 14.0    # chosen: a feel starts with the centre this far in (planner-safe, see CORNER_IN_CM). No DOE
PROBE_END_CM = 1.0       # the creep's end: centre 1 cm inside the marked line (front face 3.75 cm past it). The line
                         # sensors (3.5 cm ahead of the axle) are then 2.5 cm past it, so the edge guard ends a creep at
                         # an open edge before the end. Geometry, no DOE
BARRIER_NEAR_CM = 6.0    # chosen: a front-sensor stop whose seen point is within this of the marked line is the table's
                         # barrier; further in, an object standing on the table (a no-go zone proposed). Covers the
                         # marked line's error (~3-5 cm by eye in MIRLT). DOE note: from the school warm-up's contacts
OBJECT_CM = 8.0          # chosen: the proposed no-go zone's size round a felt object (the front sensor gives one point
                         # of it); Nick edits it. No DOE (a person confirms each zone)
EDGE_AHEAD_CM = {"line": 3.5, "bottom": 8.0, "": 3.5}   # where the edge is, ahead of the axle, by the cue that tripped
                         # (as run_plan.py EDGE_AHEAD_CM: the line sensors 3.5 cm ahead; the bottom zone ~8 cm)
MAX_FRAMES = 15          # Nick's check: 90 s or less at about 6 s a frame (brief, 2026-10-27)
SCALE_FLAG = 0.03        # chosen: a robot's wheel or gyro scale off by more than 3 % against the camera is shown to Nick
                         # (the camera's own ~1 % over 20+ cm drives). DOE note: from the school warm-up's spread
WHEEL_R_CM = 1.7         # nominal wheel radius (tools/raw_units.py robot_columns)
WHEEL_MIN_CM = 10.0      # chosen: drives this long or longer count in the wheel check (the planner cuts drives into
                         # 15 cm steps; the camera's ~0.5 cm noise is then 5 % of one drive, halved over a median of
                         # several). No DOE: it only picks which drives the check uses

SHORT = {"x_min": "left", "x_max": "right", "y_min": "bottom", "y_max": "top"}


# ---------------------------------------------------------------------------- the warm-up's tasks

def _inward(side):
    return {"x_min": (1, 0), "x_max": (-1, 0), "y_min": (0, 1), "y_max": (0, -1)}[side]


def side_probes(m, side, spacing=PROBE_SPACING_CM):
    """The feel points along a side: [(along, start xy, end xy)], evenly spread, at least 2."""
    f = m["floor"]
    lo, hi = (f["y_min"], f["y_max"]) if side.startswith("x") else (f["x_min"], f["x_max"])
    L = hi - lo
    n = max(2, int(math.ceil(L / spacing)))
    line = f[side]
    ix, iy = _inward(side)
    out = []
    for k in range(n):
        a = lo + L * (k + 0.5) / n
        a = min(max(a, lo + CORNER_IN_CM), hi - CORNER_IN_CM)
        if side.startswith("x"):
            s0 = (line + ix * PROBE_START_CM, a)
            s1 = (line + ix * PROBE_END_CM, a)
        else:
            s0 = (a, line + iy * PROBE_START_CM)
            s1 = (a, line + iy * PROBE_END_CM)
        out.append((round(a, 1), s0, s1))
    return out


def corners(m):
    f = m["floor"]
    out = []
    for sx in ("x_min", "x_max"):
        for sy in ("y_min", "y_max"):
            ix, _ = _inward(sx)
            _, iy = _inward(sy)
            out.append(("%s-%s corner" % (SHORT[sy], SHORT[sx]), (f[sx] + ix * CORNER_IN_CM, f[sy] + iy * CORNER_IN_CM)))
    return out


def plan_tasks(m):
    """The warm-up's task list for each robot in the map: corner pauses and side feels, nearest first, shared out by
    where the robots stand (each item goes to the robot whose start is nearest), then park where it started."""
    m = FM.normalize(m)
    robots = {r["name"]: r["start"] for r in m["robots"]}
    if not robots:
        raise FM.MapError("no robots in the map")
    items = []
    for label, xy in corners(m):
        items.append({"xy": xy, "visits": [{"goto": [xy[0], xy[1]], "label": label},
                                           {"wait_s": CORNER_PAUSE_S, "label": label}]})
    for side in FM.SIDES:
        for a, s0, s1 in side_probes(m, side):
            lab = "feel %s side at %.0f" % (SHORT[side], a)
            items.append({"xy": s0, "visits": [{"probe": [s0[0], s0[1], s1[0], s1[1]], "label": lab, "side": side,
                                                "along": a, "kind": "side"}]})
    own = {n: [] for n in robots}
    for it in items:
        n = min(robots, key=lambda r: math.dist(robots[r][:2], it["xy"]))
        own[n].append(it)
    tasks = {}
    for n, its in own.items():
        cur, seq = robots[n][:2], []
        left = list(its)
        while left:
            k = min(range(len(left)), key=lambda i: math.dist(cur, left[i]["xy"]))
            it = left.pop(k)
            seq += it["visits"]
            cur = it["xy"]
        tasks[n] = {"visit": seq, "park": True}
    m2 = dict(m)
    m2["tasks"] = tasks
    m2["warmup"] = {"made": FM.now_iso(), "rect": [m["floor"][k] for k in ("x_min", "y_min", "x_max", "y_max")]}
    return FM.normalize(m2) | {"warmup": m2["warmup"]}


# ---------------------------------------------------------------------------- reading the session

def _jsonl(path):
    out = []
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass
    return out


def _csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def classify_probe(m, side, along, rec):
    """One side feel's answer from its last labelled_step record: kind open / barrier / object / none / unfelt, the
    point found (floor cm), and how far it is from the marked line (+ = further out than marked)."""
    if rec is None:
        return {"side": side, "along": along, "kind": "unfelt", "why": "the feel never ran"}
    info = rec.get("info") or ""
    x, y, th = rec["pose"]
    cam = rec.get("cam")
    if cam and cam[0] is not None:
        x, y = cam[0], cam[1]          # the camera's place at the stop (the robot's own pose is blended with it anyway)
        if cam[2] is not None:
            th = cam[2]
    h = math.radians(th)
    line = m["floor"][side]
    ix, iy = _inward(side)

    def out_of_line(px, py):
        return (line - px) * ix if side.startswith("x") else (line - py) * iy    # + = past the marked line

    base = {"side": side, "along": along, "t_run": rec.get("t_run"), "robot": rec.get("robot"), "stop": [x, y, th]}
    if rec["state"] == "done":
        return dict(base, kind="none", why="nothing stopped the creep: the table goes on past the marked line here",
                    offset_cm=None)
    if info.startswith("plywood edge"):
        cue = info.split()[-1] if info.split() else ""
        ahead = EDGE_AHEAD_CM.get(cue if cue in EDGE_AHEAD_CM else "", 3.5)
        px, py = x + ahead * math.cos(h), y + ahead * math.sin(h)
        return dict(base, kind="open", point=[round(px, 1), round(py, 1)], offset_cm=round(out_of_line(px, py), 1),
                    why="the edge guard stopped it (%s)" % (cue or "line"))
    if info.startswith("obstacle"):
        try:
            d = float(info.split()[1])
        except (IndexError, ValueError):
            d = rec.get("front_cm") or 3.0
        fo = FM.normalize(m)["robot"]["front_offset_cm"]
        px, py = x + (fo + d) * math.cos(h), y + (fo + d) * math.sin(h)
        off = out_of_line(px, py)
        if off >= -BARRIER_NEAR_CM:
            return dict(base, kind="barrier", point=[round(px, 1), round(py, 1)], offset_cm=round(off, 1),
                        why="its front sensor met something %.1f cm ahead, at the side" % d)
        return dict(base, kind="object", point=[round(px, 1), round(py, 1)], offset_cm=round(off, 1),
                    why="its front sensor met something %.1f cm ahead, %.0f cm in from the marked side" % (d, -off))
    return dict(base, kind="unfelt", why="stopped for another reason (%s)" % info)


def side_kinds(m, probes, spacing=PROBE_SPACING_CM):
    """Each side's kind from its feels: all open -> open; all barrier -> barrier; both -> partial, barrier stretches
    round each barrier feel (half the spacing either way, merged); none felt -> unknown (kept away from as open)."""
    f = m["floor"]
    out = {}
    for side in FM.SIDES:
        ps = [p for p in probes if p["side"] == side]
        kinds = [p["kind"] for p in ps if p["kind"] in ("open", "barrier")]
        if not kinds:
            out[side] = {"kind": "unknown", "source": "warm-up: not felt"}
        elif all(k == "open" for k in kinds):
            out[side] = {"kind": "open", "source": "warm-up"}
        elif all(k == "barrier" for k in kinds):
            out[side] = {"kind": "barrier", "source": "warm-up"}
        else:
            lo, hi = (f["y_min"], f["y_max"]) if side.startswith("x") else (f["x_min"], f["x_max"])
            spans = []
            for p in sorted((p for p in ps if p["kind"] == "barrier"), key=lambda p: p["along"]):
                a, b = max(lo, p["along"] - spacing / 2), min(hi, p["along"] + spacing / 2)
                if spans and a <= spans[-1][1]:
                    spans[-1][1] = max(spans[-1][1], b)
                else:
                    spans.append([round(a, 1), round(b, 1)])
            out[side] = {"kind": "partial", "barrier": spans, "source": "warm-up"}
    return out


def scale_check(session):
    """Stored wheel and gyro scales against the camera, per robot: straight drives over WHEEL_MIN_CM (camera distance /
    wheel distance) and turns over 45 deg (camera heading change / the IMU's yaw change)."""
    raw = _csv(os.path.join(session, "raw.csv"))
    track = _csv(os.path.join(session, "cam_track.csv"))
    params = _csv(os.path.join(session, "param.csv"))
    ev = _jsonl(os.path.join(session, "coordinator.jsonl"))
    sent = [e for e in ev if e.get("event") == "step_sent"]
    kind = {(e["robot"], int(e["id"])): e["text"].split()[2] for e in sent if len(e.get("text", "").split()) > 2}
    # when each step started and ended, by the laptop's clock (STEP records go out at once; RAW rows come in batches)
    when = {}
    for e in ev:
        if e.get("event") == "step_report" and e.get("state") in ("started", "done"):
            when.setdefault((e["robot"], int(e["id"])), {})[e["state"]] = float(e["t"])
    stored = {}
    for p in params:
        if p["name"] in ("wheel_scale", "gyro_scale"):
            stored.setdefault(p["robot"], {})[p["name"]] = _num(p["value"])
    cam = {}
    for r in track:
        cam.setdefault(r["robot"], []).append((float(r["t"]), _num(r["x_cm"]), _num(r["y_cm"]), _num(r["heading_deg"])))

    def cam_at(robot, t):
        L = cam.get(robot) or []
        if not L:
            return None
        best = min(L, key=lambda c: abs(c[0] - t))
        return best if abs(best[0] - t) < 0.15 else None
    steps = {}
    for r in raw:
        sid = int(_num(r["step"]) or 0)
        if sid <= 0:
            continue
        steps.setdefault((r["robot"], sid), []).append(r)
    out = {}
    for (robot, sid), rows in steps.items():
        k = kind.get((robot, sid))
        if k not in ("D", "T") or len(rows) < 3:
            continue
        a, b = rows[0], rows[-1]
        w = when.get((robot, sid), {})
        if "started" not in w or "done" not in w:
            continue
        ca, cb = cam_at(robot, w["started"]), cam_at(robot, w["done"])
        if not ca or not cb:
            continue
        o = out.setdefault(robot, {"wheel": [], "gyro": []})
        if k == "D":
            dl = (_num(b["wl_deg"]) or 0) - (_num(a["wl_deg"]) or 0)
            dr = (_num(b["wr_deg"]) or 0) - (_num(a["wr_deg"]) or 0)
            wheel = abs(dl + dr) / 2.0 / 360.0 * 2 * math.pi * WHEEL_R_CM * (stored.get(robot, {}).get("wheel_scale") or 1.0)
            camd = math.hypot(cb[1] - ca[1], cb[2] - ca[2])
            if wheel > WHEEL_MIN_CM:
                o["wheel"].append(camd / wheel)
        else:
            dy = ((_num(b["yaw_deg"]) or 0) - (_num(a["yaw_deg"]) or 0) + 180) % 360 - 180
            dc = ((cb[3] or 0) - (ca[3] or 0) + 180) % 360 - 180
            if abs(dy) > 45 and ca[3] is not None and cb[3] is not None:
                o["gyro"].append(dc / dy * (stored.get(robot, {}).get("gyro_scale") or 1.0))
    res = {}
    for robot, o in out.items():
        r = {"stored": stored.get(robot, {})}
        for key in ("wheel", "gyro"):
            v = sorted(o[key])
            if v:
                med = v[len(v) // 2]
                r[key] = {"camera_over_stored": round(med, 4), "n": len(v), "flag": abs(med - 1.0) > SCALE_FLAG}
        res[robot] = r
    return res


def _centre(poly):
    return (sum(q[0] for q in poly) / len(poly), sum(q[1] for q in poly) / len(poly))


def analyze(m, session):
    """Everything the warm-up found: side kinds and where each side is, proposed no-go zones, the frame list, scales."""
    m = FM.normalize(m)
    ev = _jsonl(os.path.join(session, "coordinator.jsonl"))
    lab = [e for e in ev if e.get("event") == "labelled_step"]
    probes = []
    for side in FM.SIDES:
        for a, s0, s1 in side_probes(m, side):
            name = "feel %s side at %.0f" % (SHORT[side], a)
            recs = [e for e in lab if e["label"] == name and e["phase"] == "probe" and e["state"] in ("done", "failed")]
            probes.append(dict(classify_probe(m, side, a, recs[-1] if recs else None), label=name))
    sides = side_kinds(m, probes)
    where = {}
    for side in FM.SIDES:
        offs = sorted(p["offset_cm"] for p in probes if p["side"] == side and p.get("offset_cm") is not None
                      and p["kind"] in ("open", "barrier"))
        if offs:
            where[side] = {"marked": m["floor"][side], "offset_cm": offs[len(offs) // 2], "n": len(offs)}
    zones = []
    for p in probes:
        if p["kind"] == "object":
            c = p["point"]
            h = OBJECT_CM / 2
            zones.append({"name": "felt by %s near the %s side" % (p["robot"], SHORT[p["side"]]),
                          "polygon": [[c[0] - h, c[1] - h], [c[0] + h, c[1] - h], [c[0] + h, c[1] + h], [c[0] - h, c[1] + h]],
                          "status": "proposed", "source": "warm-up", "frame": p.get("t_run"),
                          "note": "%s's creep stopped %.0f cm in from the marked side" % (p["robot"], -p["offset_cm"])})
    # things the front sensors met on the way between feels (run_plan's re-plans with a distance_sensor obstacle):
    # well in from every side -> also a proposed zone (the polygon run_plan already planned round)
    for e in ev:
        o = e.get("obstacle") if e.get("event") == "replan" else None
        if not o or o.get("source") != "distance_sensor" or not o.get("points"):
            continue
        cx = sum(q[0] for q in o["points"]) / len(o["points"])
        cy = sum(q[1] for q in o["points"]) / len(o["points"])
        side, d_in, _ = WS.nearest_side(m, cx, cy)
        if d_in <= BARRIER_NEAR_CM or any(math.dist((cx, cy), _centre(z["polygon"])) < OBJECT_CM for z in zones):
            continue
        cause = next((ev_["robot"] for ev_ in e.get("events", []) if ev_.get("action") == "stopped"), None)
        zones.append({"name": "met by %s on the way" % (cause or "a robot"), "polygon": o["points"],
                      "status": "proposed", "source": "warm-up", "frame": e.get("t_run"),
                      "note": "a front sensor stopped a drive here, %.0f cm in from the %s side" % (d_in, SHORT[side])})
    frames = []
    for z in zones:
        frames.append({"t_run": z["frame"], "what": "no-go zone?", "question": "Is something standing here (%s)? "
                       "Keep it as a no-go zone, resize it or delete it." % z["note"], "priority": 0})
    for e in ev:
        if e.get("event") == "tracker_lost":
            frames.append({"t_run": e.get("t_run"), "robot": e["robot"], "what": "tracker lost the robot",
                           "question": "Where is %s in this picture? (the camera lost it here)" % e["robot"],
                           "cam": e.get("last_cam"), "priority": 1})
    pauses = {}
    for e in lab:
        if e["phase"] == "pause" and e["state"] in ("started", "done"):
            pauses.setdefault((e["robot"], e["label"]), {})[e["state"]] = e
    for (robot, label), d in pauses.items():
        a, b = d.get("started"), d.get("done")
        t = (a["t_run"] + b["t_run"]) / 2 if a and b else (a or b)["t_run"]
        frames.append({"t_run": round(t, 2), "robot": robot, "what": label, "cam": (b or a).get("cam"),
                       "question": "Is %s at the table's %s, with the corner where the outline says?" % (robot, label),
                       "priority": 2})
    for p in probes:
        if p["kind"] in ("none", "unfelt"):
            frames.append({"t_run": p.get("t_run"), "robot": p.get("robot"), "what": p["label"], "question":
                           "%s: %s. Where is the table's %s side here?" % (p["label"], p["why"], SHORT[p["side"]]),
                           "priority": 1})
        elif p["kind"] in ("open", "barrier"):
            frames.append({"t_run": p.get("t_run"), "robot": p.get("robot"), "what": p["label"], "question":
                           "%s found %s here (%s). Right?" % (p["robot"], "an open edge" if p["kind"] == "open"
                                                                else "a barrier", p["why"]), "priority": 3})
    frames.sort(key=lambda fr: (fr["priority"], fr["t_run"] if fr.get("t_run") is not None else 1e9))
    shown, rest = frames[:MAX_FRAMES], frames[MAX_FRAMES:]
    shown.sort(key=lambda fr: fr["t_run"] if fr.get("t_run") is not None else 1e9)
    # the mapping view (engineer view): each robot's camera track and every contact with its kind
    contacts = [{"robot": p.get("robot"), "kind": p["kind"], "point": p.get("point") or (p.get("stop") or [None, None])[:2],
                 "side": p["side"], "label": p["label"]} for p in probes if p["kind"] != "unfelt"]
    return {"rect": [m["floor"][k] for k in ("x_min", "y_min", "x_max", "y_max")], "sides": sides, "where": where,
            "probes": probes, "zones": zones, "frames": shown, "frames_not_shown": len(rest),
            "check_s": 6 * len(shown), "scales": scale_check(session), "contacts": contacts}


def apply(m, result, accept_zones=None, move_sides=False):
    """The warm-up's answers into the map: side kinds, the zones Nick kept (names; None = all proposed, still
    'proposed'), and (move_sides) each side moved to where it was found. Returns a normalized map."""
    m = dict(FM.normalize(m))
    m.pop("warmup", None)
    m["tasks"] = {}
    m["sides"] = result["sides"]
    if move_sides:
        f = dict(m["floor"])
        for side, w in result["where"].items():
            sign = 1 if side.endswith("max") else -1
            f[side] = round(f[side] + sign * w["offset_cm"], 1)
        m["floor"] = f
    for z in result["zones"]:
        if accept_zones is None or z["name"] in accept_zones:
            m = dict(WS.add_nogo(m, z["polygon"], name=z["name"], status="proposed" if accept_zones is None
                                 else "confirmed", source="warm-up", note=z.get("note"), frame=z.get("frame"))[0])
    return FM.normalize(m)


def words(result):
    out = []
    for side in FM.SIDES:
        sd = result["sides"][side]
        w = result["where"].get(side)
        where = (", %.1f cm %s than marked" % (abs(w["offset_cm"]), "further out" if w["offset_cm"] > 0 else "further in")
                 if w else "")
        if sd["kind"] == "partial":
            out.append("%s side: partly barrier (%s), the rest open%s" % (
                SHORT[side], ", ".join("%.0f-%.0f cm" % tuple(b) for b in sd["barrier"]), where))
        else:
            out.append("%s side: %s%s" % (SHORT[side], {"open": "open edge", "barrier": "barrier",
                                                        "unknown": "not felt (kept away from as an open edge)"}[sd["kind"]],
                                          where))
    for z in result["zones"]:
        out.append("proposed no-go zone: %s (%s)" % (z["name"], z["note"]))
    for robot, r in sorted(result["scales"].items()):
        bits = []
        for key in ("wheel", "gyro"):
            if key in r:
                bits.append("%s %.3f x stored (%d)%s" % (key, r[key]["camera_over_stored"], r[key]["n"],
                                                         " CHECK" if r[key]["flag"] else ""))
        out.append("%s: %s" % (robot, "; ".join(bits) or "no long drives or big turns to check"))
    out.append("%d frames for Nick's check (about %d s)%s" % (len(result["frames"]), result["check_s"],
               "; %d more not shown" % result["frames_not_shown"] if result["frames_not_shown"] else ""))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    a1 = sub.add_parser("plan")
    a1.add_argument("--map", required=True)
    a1.add_argument("--out", required=True)
    a2 = sub.add_parser("analyze")
    a2.add_argument("--map", required=True)
    a2.add_argument("--session", required=True)
    a2.add_argument("--out")
    a = ap.parse_args(argv)
    if a.cmd == "plan":
        m = plan_tasks(FM.load(a.map))
        with open(a.out, "w") as f:
            json.dump(m, f, indent=1)
        n = {k: len(v["visit"]) for k, v in m["tasks"].items()}
        print("warm-up tasks: %s -> %s" % (n, a.out))
    else:
        with open(a.map) as f:
            m = json.load(f)
        res = analyze(m, a.session)
        print("\n".join(words(res)))
        with open(a.out or os.path.join(a.session, "warmup_result.json"), "w") as f:
            json.dump(res, f, indent=1)


if __name__ == "__main__":
    main()
