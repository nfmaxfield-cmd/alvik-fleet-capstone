"""map.json: the floor, the boards ("machines"), keep-out areas, obstacles,
the robots' start/park spots and their task lists. Format in
docs/data-formats.md ("Fleet map").

Also works out each board's dock pose (where the robot stops, facing the
board, front face `gap_cm` from its surface) and pre-dock point (where the
slow, straight final approach starts).
"""
import copy
import datetime as dt
import hashlib
import json
import math
import os

import numpy as np

import geometry as G

FORMAT = "alvik-fleet-map"
VERSION = 1

DEFAULT_MAP = {
    "format": FORMAT,
    "version": VERSION,
    "units": "cm, degrees",
    "floor": {"x_min": 0.0, "x_max": 122.0, "y_min": 0.0, "y_max": 122.0},
    "robot": {"length_cm": 9.5, "width_cm": 9.5, "front_offset_cm": 4.75, "bumper_cm": 0.0, "back_bumper_cm": 0.0},
    "dock": {"gap_cm": 1.0, "predock_cm": 15.0},
    "boards": [],
    "keepouts": [],
    "obstacles": [],
    "robots": [],
    "tasks": {},
    "home_docks": [],
}

def _dock_params():
    """hardware/dock/dock_params.py (the printed dock and nose bracket), or None if it can't be read."""
    import importlib.util
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "hardware", "dock",
                     "dock_params.py")
    try:
        spec = importlib.util.spec_from_file_location("dock_params", p)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    except Exception:
        return None


_DP = _dock_params()


def _dp_cm(name, default):
    return round(getattr(_DP, name) / 10.0, 3) if _DP is not None and hasattr(_DP, name) else default


# The printed nose bracket (hardware/dock): how far it sticks out in front of the shell (BUMPER_DEPTH_MM / 10),
# and its width (BRACKET_W_MM / 10: its arms run down the robot's sides, so it is wider than the robot).
BUMPER_FITTED_CM = _dp_cm("BUMPER_DEPTH_MM", 1.34)
BUMPER_W_FITTED_CM = _dp_cm("BRACKET_W_MM", 10.22)

# A home dock (hardware/dock): x, y = middle of its back wall's inner face, heading = the way a docked robot
# faces. Its outline (cm, dock frame: X into the dock from the back wall, Y left) comes from these, the printed
# part's numbers (hardware/dock/dock_params.py, mm there; the pocket fits the bracket).
HOME_DOCK_DEFAULTS = {"pocket_w_cm": _dp_cm("POCKET_W_MM", 10.32), "pocket_depth_cm": _dp_cm("POCKET_DEPTH_MM", 1.2),
                      "funnel_len_cm": _dp_cm("FUNNEL_LEN_MM", 5.0), "funnel_flare_cm": _dp_cm("FUNNEL_FLARE_MM", 3.2),
                      "wall_t_cm": _dp_cm("WALL_T_MM", 0.32), "back_depth_cm": _dp_cm("BACK_BLOCK_DEPTH_MM", 4.8),
                      "mouth_cm": 15.0}
# mouth_cm: where a robot going home stops, facing in: its axle this far in front of the dock mouth

OBSTACLE_SOURCES = ("camera", "distance_sensor", "edge_guard", "user", "robot_out")   # edge_guard: an open plywood edge;
                                                                                   # robot_out: a robot that went silent

# General workspace (school demo, 2026-10-27; docs/build-plans/mission-control.md): the table's rectangle (floor) plus
# each side's kind, named spots and no-go zones. Nothing assumes boards. A map without "sides" keeps the old meaning
# (every floor edge a wall-like limit, no extra margin).
SIDES = ("x_min", "x_max", "y_min", "y_max")          # the line x = x_min, x = x_max, y = y_min, y = y_max
SIDE_KINDS = ("open", "barrier", "partial", "unknown")  # unknown: not felt yet, treated as open
OPEN_MARGIN_CM = 3.0    # chosen: extra room the planner keeps from an open (drop) edge, on top of safety_margin_cm:
                        # the camera's error near the table's edges (~3 cm, the brief's Chicken note; 3-5 cm seen at
                        # 15:10 / 21:14 near the plywood edges). DOE note: 2 / 3 / 5 cm on the school table's warm-up
                        # (edge-guard trips per hour against time lost); docs/doe-backlog.md
NOGO_STATUS = ("proposed", "confirmed")                # proposed: from a warm-up bump, for Nick to confirm or delete


class MapError(ValueError):
    pass


def now_iso():
    return dt.datetime.now().isoformat(timespec="seconds")


# ----------------------------------------------------------------------------
# Load, check, save
# ----------------------------------------------------------------------------

def _pt(v, what, n=2):
    if not isinstance(v, (list, tuple)) or len(v) < n:
        raise MapError("%s must be a list of %d numbers, got %r" % (what, n, v))
    try:
        return [float(x) for x in v[:n]] + ([float(v[2])] if n == 2 and len(v) > 2 else [])
    except (TypeError, ValueError):
        raise MapError("%s must be numbers, got %r" % (what, v))


def normalize(m):
    """Fill defaults, check types, return a clean copy. Raises MapError with a
    plain message on anything wrong."""
    if not isinstance(m, dict):
        raise MapError("map must be a JSON object")
    out = copy.deepcopy(DEFAULT_MAP)
    for k in ("floor", "robot", "dock"):
        out[k].update(m.get(k) or {})
    if str(out["robot"].get("bumper_cm")).lower() in ("fitted", "true", "yes"):
        out["robot"]["bumper_cm"] = BUMPER_FITTED_CM       # the printed nose bumper (hardware/dock)
    if str(out["robot"].get("back_bumper_cm")).lower() in ("fitted", "true", "yes"):
        out["robot"]["back_bumper_cm"] = BUMPER_FITTED_CM  # the one attachment at the back (v3: the bracket mirrored)
    for k in ("floor", "robot", "dock"):
        for kk, vv in out[k].items():
            try:
                out[k][kk] = float(vv)
            except (TypeError, ValueError):
                raise MapError("%s.%s must be a number" % (k, kk))
    f = out["floor"]
    if not (f["x_max"] > f["x_min"] and f["y_max"] > f["y_min"]):
        raise MapError("floor: x_max must exceed x_min and y_max exceed y_min")
    for k in ("notes", "mission", "surface_cm"):   # mission: missions.py's name, settings and planner settings (passed
        # on); surface_cm: the surface the cameras' setup sees (Mission Control, 2026-10-28: the home plywood)
        if k in m:
            out[k] = m[k]

    names = set()
    for i, b in enumerate(m.get("boards") or []):
        nm = str(b.get("name") or "board%d" % (i + 1))
        if nm in names:
            raise MapError("two boards are called %r" % nm)
        names.add(nm)
        bb = {"name": nm, "p1": _pt(b.get("p1"), "board %s p1" % nm)[:2],
              "p2": _pt(b.get("p2"), "board %s p2" % nm)[:2],
              "thickness_cm": float(b.get("thickness_cm", 1.5)),
              "side": str(b.get("side", "auto")),
              "dock_at": float(b.get("dock_at", 0.5))}
        if bb["side"] not in ("auto", "left", "right"):
            raise MapError("board %s side must be auto, left or right" % nm)
        if math.dist(bb["p1"], bb["p2"]) < 1.0:
            raise MapError("board %s is shorter than 1 cm" % nm)
        if not 0.0 <= bb["dock_at"] <= 1.0:
            raise MapError("board %s dock_at must be 0..1" % nm)
        out["boards"].append(bb)

    for i, k in enumerate(m.get("keepouts") or []):
        poly = [_pt(p, "keep-out %d point" % (i + 1))[:2] for p in (k.get("polygon") or [])]
        if len(poly) < 3:
            raise MapError("keep-out %d needs at least 3 points" % (i + 1))
        ko = {"name": str(k.get("name") or "keepout%d" % (i + 1)), "polygon": poly}
        if "status" in k:                  # a no-go zone: proposed (a warm-up bump) or confirmed; both are kept out of
            if k["status"] not in NOGO_STATUS:   # until deleted
                raise MapError("keep-out %s status must be proposed or confirmed" % ko["name"])
            ko["status"] = k["status"]
        for extra in ("source", "note", "frame"):
            if extra in k:
                ko[extra] = k[extra]
        if k.get("solid"):
            ko["solid"] = True        # a thing standing there (a felt object): the front sensors see it
        out["keepouts"].append(ko)

    for i, o in enumerate(m.get("obstacles") or []):
        out["obstacles"].append(_norm_obstacle(o, i))

    dnames = set()
    for i, d in enumerate(m.get("home_docks") or []):
        nm = str(d.get("name") or "home%d" % (i + 1))
        if nm in dnames:
            raise MapError("two home docks are called %r" % nm)
        dnames.add(nm)
        try:
            dd = {"name": nm, "x": float(d["x"]), "y": float(d["y"]), "heading": float(d["heading"])}
        except (KeyError, TypeError, ValueError):
            raise MapError("home dock %s needs numbers x, y (middle of its back wall's inner face, cm) and heading "
                           "(the way a docked robot faces, deg)" % nm)
        for k, dv in HOME_DOCK_DEFAULTS.items():
            try:
                dd[k] = float(d.get(k, dv))
            except (TypeError, ValueError):
                raise MapError("home dock %s %s must be a number" % (nm, k))
        if "sigma_cm" in d:
            dd["sigma_cm"] = float(d["sigma_cm"])
        out["home_docks"].append(dd)

    rnames = set()
    for i, r in enumerate(m.get("robots") or []):
        nm = str(r.get("name") or "robot%d" % (i + 1))
        if nm in rnames:
            raise MapError("two robots are called %r" % nm)
        rnames.add(nm)
        st = _pt(r.get("start"), "robot %s start" % nm)
        if len(st) < 3:
            st.append(0.0)
        pk = r.get("park")
        pk = _pt(pk, "robot %s park" % nm) if pk is not None else list(st)
        if len(pk) < 3:
            pk.append(st[2])
        out["robots"].append({"name": nm, "start": st[:3], "park": pk[:3]})

    if m.get("sides") is not None:
        out["sides"] = _norm_sides(m["sides"], f)
        out["open_margin_cm"] = float(m.get("open_margin_cm", OPEN_MARGIN_CM))
        if out["open_margin_cm"] < 0:
            raise MapError("open_margin_cm must be >= 0")
    spots = {}
    for nm, v in (m.get("spots") or {}).items():
        pt = _pt(v, "spot %s" % nm)
        if not (f["x_min"] <= pt[0] <= f["x_max"] and f["y_min"] <= pt[1] <= f["y_max"]):
            raise MapError("spot %s (%.1f, %.1f) is off the table" % (nm, pt[0], pt[1]))
        spots[str(nm)] = pt[:3]
    if spots:
        out["spots"] = spots

    tasks = m.get("tasks") or {}
    for rn, t in tasks.items():
        if rn not in rnames:
            raise MapError("tasks given for robot %r, which is not in robots" % rn)
        out["tasks"][rn] = _norm_tasks(t, names, rn, spots)
    return out


def _norm_sides(sides, f):
    """{"x_min": {"kind": "open" | "barrier" | "partial" | "unknown", "barrier": [[lo, hi], ...]}, ...}: the barrier
    spans (along the side: y for the x sides, x for the y sides) of a partial side; the rest of it is open. A side not
    given is "unknown" (treated as open)."""
    if not isinstance(sides, dict):
        raise MapError("sides must be an object with x_min, x_max, y_min, y_max")
    out = {}
    for k in sides:
        if k not in SIDES:
            raise MapError("unknown side %r (use %s)" % (k, ", ".join(SIDES)))
    for k in SIDES:
        v = sides.get(k) or {"kind": "unknown"}
        if isinstance(v, str):
            v = {"kind": v}
        kind = str(v.get("kind", "unknown"))
        if kind not in SIDE_KINDS:
            raise MapError("side %s: kind must be one of %s" % (k, ", ".join(SIDE_KINDS)))
        lo, hi = (f["y_min"], f["y_max"]) if k.startswith("x") else (f["x_min"], f["x_max"])
        spans = []
        if kind == "partial":
            for sp in v.get("barrier") or []:
                a, b = sorted(_pt(sp, "side %s barrier span" % k)[:2])
                a, b = max(a, lo), min(b, hi)
                if b > a:
                    spans.append([a, b])
            if not spans:
                raise MapError("side %s is partial but has no barrier span inside it" % k)
            spans.sort()
        sd = {"kind": kind}
        if spans:
            sd["barrier"] = spans
        for extra in ("source", "felt"):
            if extra in v:
                sd[extra] = v[extra]
        out[k] = sd
    return out


def side_parts(m, side):
    """(barrier spans, open spans) along a side, each [[lo, hi], ...] in floor cm along it. A map without sides: the
    whole side is a barrier-like limit (the old meaning)."""
    f = m["floor"]
    lo, hi = (f["y_min"], f["y_max"]) if side.startswith("x") else (f["x_min"], f["x_max"])
    sd = (m.get("sides") or {}).get(side)
    if sd is None or sd["kind"] == "barrier":
        return [[lo, hi]], []
    if sd["kind"] in ("open", "unknown"):
        return [], [[lo, hi]]
    bar = [list(x) for x in sd["barrier"]]
    opn, a = [], lo
    for b0, b1 in bar:
        if b0 > a:
            opn.append([a, b0])
        a = max(a, b1)
    if a < hi:
        opn.append([a, hi])
    return bar, opn


def _norm_obstacle(o, i):
    oid = str(o.get("id") or "obstacle%d" % (i + 1))
    shape = o.get("shape") or ("circle" if "center" in o else "polygon")
    src = o.get("source", "user")
    if src not in OBSTACLE_SOURCES:
        raise MapError("obstacle %s source must be one of %s" % (oid, ", ".join(OBSTACLE_SOURCES)))
    ob = {"id": oid, "shape": shape, "source": src, "time": o.get("time") or now_iso()}
    if shape == "circle":
        ob["center"] = _pt(o.get("center"), "obstacle %s center" % oid)[:2]
        ob["radius"] = float(o.get("radius", 3.0))
        if ob["radius"] <= 0:
            raise MapError("obstacle %s radius must be > 0" % oid)
    elif shape == "polygon":
        ob["points"] = [_pt(p, "obstacle %s point" % oid)[:2] for p in (o.get("points") or [])]
        if len(ob["points"]) < 3:
            raise MapError("obstacle %s needs at least 3 points" % oid)
    else:
        raise MapError("obstacle %s shape must be circle or polygon" % oid)
    if o.get("note"):
        ob["note"] = str(o["note"])
    return ob


def _norm_tasks(t, board_names, rn, spots=None):
    """Task list: ["A", "B"] or {"visit": [...], "any_order": false, "park": true}.
    A visit item is a board name, {"goto": [x, y]} / {"goto": [x, y, heading]}, {"spot": "A"} (a named spot: becomes
    a goto that keeps the name) or {"wait_s": 5} (stand still)."""
    if isinstance(t, list):
        t = {"visit": t}
    spots = spots or {}
    visit = []
    for it in t.get("visit") or []:
        if isinstance(it, str):
            if it not in board_names:
                raise MapError("robot %s: no board called %r" % (rn, it))
            visit.append(it)
        elif isinstance(it, dict) and "spot" in it:
            nm = str(it["spot"])
            if nm not in spots:
                raise MapError("robot %s: no spot called %r" % (rn, nm))
            visit.append({"goto": list(spots[nm]), "spot": nm})
        elif isinstance(it, dict) and "goto" in it:
            g = {"goto": _pt(it["goto"], "robot %s goto" % rn)}
            for k in ("spot", "label", "lane", "end", "run_in_cm", "risky"):
                if k in it:
                    g[k] = it[k]
            visit.append(g)
        elif isinstance(it, dict) and "wait_s" in it:
            w = float(it["wait_s"])
            if w < 0:
                raise MapError("robot %s: wait_s must be >= 0" % rn)
            visit.append({"wait_s": w, **({"label": str(it["label"])} if it.get("label") else {})})
        elif isinstance(it, dict) and "until_s" in it:
            # stand still until this time (s from the mission's start): a show's beat targets (missions.py)
            visit.append({"until_s": float(it["until_s"]), **({"label": str(it["label"])} if it.get("label") else {})})
        elif isinstance(it, dict) and "arc" in it:
            # a drive round a circle (Monday's circle; planner.py _arc_visit): {"arc": {"c": [x, y], "r": cm, "a0_deg":
            # where on the circle it starts, "sweep_deg": + anticlockwise}}
            a = it["arc"]
            try:
                arc = {"c": [float(a["c"][0]), float(a["c"][1])], "r": float(a["r"]), "a0_deg": float(a["a0_deg"]),
                       "sweep_deg": float(a["sweep_deg"])}
            except (KeyError, TypeError, ValueError, IndexError):
                raise MapError("robot %s: arc needs c [x, y], r, a0_deg and sweep_deg" % rn)
            if arc["r"] <= 0:
                raise MapError("robot %s: arc radius must be > 0" % rn)
            visit.append({"arc": arc, **({"label": str(it["label"])} if it.get("label") else {})})
        elif isinstance(it, dict) and "dance" in it:
            # dance where it stands (firmware follow step Z): seconds, 0 = until switched off
            visit.append({"dance": float(it["dance"]), **({"label": str(it["label"])} if it.get("label") else {})})
        elif isinstance(it, dict) and "sync" in it:
            # a meet point: the next step waits for every robot with the same sync id (planner._sync_after)
            visit.append({"sync": str(it["sync"]), **({"label": str(it["label"])} if it.get("label") else {})})
        elif isinstance(it, dict) and "probe" in it:
            # a slow straight creep (the warm-up's side feel): from (x0, y0) towards (x1, y1), which may be past the
            # table's edge; it ends at the first edge-guard trip or front-sensor stop (tools/fleet/warmup.py)
            pr = [float(v) for v in it["probe"]]
            if len(pr) != 4:
                raise MapError("robot %s: probe must be [x0, y0, x1, y1]" % rn)
            g = {"probe": pr, "front_cm": float(it.get("front_cm", 3.0))}
            for k in ("label", "side", "along", "kind"):
                if k in it:
                    g[k] = it[k]
            visit.append(g)
        else:
            raise MapError("robot %s: task %r is not a board name, {goto: [x, y]}, {spot: name} or {wait_s: s}"
                           % (rn, it))
    return {"visit": visit, "any_order": bool(t.get("any_order", False)), "park": bool(t.get("park", True))}


def load(path):
    with open(path, encoding="utf-8") as f:
        return normalize(json.load(f))


def save(m, path):
    m = normalize(m)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(m, f, indent=1)
        f.write("\n")
    os.replace(tmp, path)
    return m


def map_hash(m):
    """sha256 of everything that shapes the paths: floor, robot, dock settings,
    boards, keep-outs, obstacles, robots. Not tasks or notes."""
    m = normalize(m)
    keep = {k: m[k] for k in ("floor", "robot", "dock", "boards", "keepouts", "obstacles", "robots")}
    if m.get("sides"):
        keep["sides"] = m["sides"]
        keep["open_margin_cm"] = m["open_margin_cm"]
    keep["keepouts"] = [{k: v for k, v in o.items() if k in ("name", "polygon")} for o in keep["keepouts"]]
    keep["obstacles"] = [{k: v for k, v in o.items() if k not in ("time", "note")} for o in keep["obstacles"]]
    s = json.dumps(keep, sort_keys=True, separators=(",", ":"))
    return "sha256:" + hashlib.sha256(s.encode()).hexdigest()[:16]


def body_of(m):
    """The planner's footprint. A fitted nose bracket makes it longer (bumper_cm) and wider (bumper_w_cm, default
    the printed bracket's width)."""
    r = m["robot"]
    b = r.get("bumper_cm", 0.0)
    bw = r.get("bumper_w_cm", BUMPER_W_FITTED_CM) if b else 0.0
    rb = r.get("back_bumper_cm", 0.0)
    rbw = r.get("back_bumper_w_cm", BUMPER_W_FITTED_CM) if rb else 0.0
    return G.Body(r["length_cm"], r["width_cm"], r["front_offset_cm"], b, bw, rb, rbw)


def switches_back(m):
    """The switches are on the robot's back attachment (robot.back_bumper_cm set; Nick, 2026-10-25): every touch
    of a board or a home dock backs in."""
    return float(m["robot"].get("back_bumper_cm", 0.0) or 0.0) > 0.0


def back_face_cm(m):
    """Axle (the tracked centre) to the back attachment's face, cm (firmware e02_back_face_cm: 6.10 for v3)."""
    r = m["robot"]
    return r["length_cm"] - r["front_offset_cm"] + float(r.get("back_bumper_cm", 0.0) or 0.0)


# ----------------------------------------------------------------------------
# Docks
# ----------------------------------------------------------------------------

def dock_for(m, board):
    """Dock pose, pre-dock point and the board's open-side normal.

    The dock is on the board's open side, on the line through the point
    `dock_at` along the board (0.5 = its middle), square to the board. The
    robot's centre sits gap + front_offset from the board surface, so its
    front face is `gap_cm` from the surface; heading points at the board.
    With the switches at the back (switches_back): it backs in instead, so
    its back face is `gap_cm` from the surface and it faces away from the
    board ("reverse": True); the planner reaches it by backing until a back
    switch clicks, then driving forward gap_cm (follow step P).
    """
    p1 = np.array(board["p1"], float)
    p2 = np.array(board["p2"], float)
    u = (p2 - p1) / np.linalg.norm(p2 - p1)
    n_left = np.array([-u[1], u[0]])
    mid = p1 + board["dock_at"] * (p2 - p1)
    side = board["side"]
    if side == "auto":
        f = m["floor"]
        centre = np.array([(f["x_min"] + f["x_max"]) / 2, (f["y_min"] + f["y_max"]) / 2])
        side = "left" if np.dot(centre - mid, n_left) >= 0 else "right"
    n = n_left if side == "left" else -n_left
    surface = mid + n * board["thickness_cm"] / 2
    rev = switches_back(m)
    if rev:
        off = m["dock"]["gap_cm"] + back_face_cm(m)                                   # gap from the back face
        heading = math.atan2(n[1], n[0])                                              # facing away from the board
    else:
        off = m["dock"]["gap_cm"] + m["robot"]["front_offset_cm"] + m["robot"].get("bumper_cm", 0.0)   # from the bumper
        heading = math.atan2(-n[1], -n[0])
    d = surface + n * off
    p = d + n * m["dock"]["predock_cm"]
    return {"board": board["name"], "side": side, "normal": n.tolist(), "surface": surface.tolist(),
            "dock": [float(d[0]), float(d[1]), heading], "predock": [float(p[0]), float(p[1]), heading],
            "reverse": rev, "gap_cm": float(m["dock"]["gap_cm"])}


def docks(m):
    return {b["name"]: dock_for(m, b) for b in m["boards"]}


# ----------------------------------------------------------------------------
# Home docks (hardware/dock): a printed funnel the robot creeps into until both nose switches click
# ----------------------------------------------------------------------------

def _dock_frame(d):
    h = math.radians(d["heading"])
    c, s = math.cos(h), math.sin(h)
    return lambda px, py: [d["x"] + c * px - s * py, d["y"] + s * px + c * py]


def home_dock_outline(d):
    """The dock's footprint on the floor (polygon, cm): back block, pocket and funnel walls, mouth closed."""
    f = _dock_frame(d)
    hw = d["pocket_w_cm"] / 2 + d["wall_t_cm"]
    xm = -(d["pocket_depth_cm"] + d["funnel_len_cm"])
    hm = hw + d["funnel_flare_cm"]
    xp = -d["pocket_depth_cm"]
    pts = [(d["back_depth_cm"], -hw), (d["back_depth_cm"], hw), (xp, hw), (xm, hm), (xm, -hm), (xp, -hw)]
    return np.array([f(x, y) for x, y in pts], float)


def home_mouth_pose(m, d):
    """Where a robot going home stops before creeping in: [x, y, rad], axle mouth_cm in front of the mouth,
    facing into the dock (switches at the back: facing out of it, its back to the mouth, so it backs in)."""
    f = _dock_frame(d)
    x = -(d["pocket_depth_cm"] + d["funnel_len_cm"]) - d["mouth_cm"]
    p = f(x, 0.0)
    return [p[0], p[1], math.radians(d["heading"]) + (math.pi if switches_back(m) else 0.0)]


def home_docked_pose(d, bump_ahead_cm, back=False):
    """Robot pose [x, y, rad] when flush in the dock (axle bump_ahead_cm from the back wall's inner face; the map's
    dock heading points from the mouth to the wall). back: switches at the back, so the docked robot faces out."""
    p = _dock_frame(d)(-bump_ahead_cm, 0.0)
    return [p[0], p[1], math.radians(d["heading"]) + (math.pi if back else 0.0)]


def home_dock(m, name):
    for d in m.get("home_docks", []):
        if d["name"] == name:
            return d
    raise MapError("no home dock called %r" % name)


def pose_out(p):
    """Pose for files: [x, y, heading_deg] rounded."""
    return [round(float(p[0]), 2), round(float(p[1]), 2), round(math.degrees(G.wrap(float(p[2]))), 2)]


def pose_in(p):
    """Pose from files ([x, y, heading_deg]) to [x, y, rad]."""
    return [float(p[0]), float(p[1]), math.radians(float(p[2]) if len(p) > 2 else 0.0)]


# ----------------------------------------------------------------------------
# Static obstacles as shapes, and how far a point / shape is from them
# ----------------------------------------------------------------------------

class World:
    """The static obstacles of a map (boards, keep-outs, obstacles) plus the
    floor edge, with distance queries. `extra` adds circles or polygons
    (used for other robots' resting spots when re-routing)."""

    def __init__(self, m, exclude_boards=(), extra=()):
        self.m = m
        f = m["floor"]
        self.box = (f["x_min"], f["x_max"], f["y_min"], f["y_max"])
        # per-side parts: (side, span lo, span hi, extra margin); None = the old meaning (plain box)
        self.sides = None
        if m.get("sides"):
            om = float(m.get("open_margin_cm", OPEN_MARGIN_CM))
            self.sides = []
            for sd in SIDES:
                bar, opn = side_parts(m, sd)
                self.sides += [(sd, a, b, 0.0) for a, b in bar] + [(sd, a, b, om) for a, b in opn]
        self.polys = []      # (name, (K,2))
        self.circles = []    # (name, centre (2,), radius)
        for b in m["boards"]:
            if b["name"] in exclude_boards:
                continue
            self.polys.append(("board " + b["name"], G.segment_rect(b["p1"], b["p2"], b["thickness_cm"])))
        for k in m["keepouts"]:
            self.polys.append(("keep-out " + k["name"], np.array(k["polygon"], float)))
        for o in m["obstacles"]:
            self._add_obstacle(o)
        for d in m.get("home_docks", []):
            self.polys.append(("home dock " + d["name"], home_dock_outline(d)))
        for o in extra:
            self._add_obstacle(o)

    def _add_obstacle(self, o):
        nm = "obstacle " + o.get("id", "?")
        if o["shape"] == "circle":
            self.circles.append((nm, np.array(o["center"], float), float(o["radius"])))
        else:
            self.polys.append((nm, np.array(o["points"], float)))

    def edge_dist(self, p):
        """Distance from points to the floor edge (negative outside). With per-side kinds, an open part of a side
        counts open_margin_cm nearer than it is (the planner keeps that much more room from a drop)."""
        x0, x1, y0, y1 = self.box
        p = np.atleast_2d(p)
        if not self.sides:
            return np.minimum.reduce([p[:, 0] - x0, x1 - p[:, 0], p[:, 1] - y0, y1 - p[:, 1]])
        perp = {"x_min": p[:, 0] - x0, "x_max": x1 - p[:, 0], "y_min": p[:, 1] - y0, "y_max": y1 - p[:, 1]}
        d = np.full(len(p), np.inf)
        for sd, a, b, mg in self.sides:
            q = perp[sd]
            al = p[:, 1] if sd.startswith("x") else p[:, 0]
            g = np.maximum(np.maximum(a - al, al - b), 0.0)
            dd = np.where(q >= 0, np.hypot(q, g), q) - mg
            np.minimum(d, dd, out=d)
        return d

    def clearance(self, p):
        """Distance from points (N,2) to the nearest obstacle or floor edge."""
        p = np.atleast_2d(np.asarray(p, float))
        d = self.edge_dist(p)
        for _, poly in self.polys:
            d = np.minimum(d, G.points_to_polygon(p, poly))
        if self.circles:
            C = np.array([c for _, c, _ in self.circles])
            R = np.array([r for _, _, r in self.circles])
            for k in range(0, len(C), 64):
                dd = np.hypot(p[:, None, 0] - C[None, k:k + 64, 0], p[:, None, 1] - C[None, k:k + 64, 1]) - R[None, k:k + 64]
                d = np.minimum(d, dd.min(1))
        return d

    def shape_clearance(self, poly, with_names=False):
        """Distance from a filled convex polygon (e.g. a robot's swept area)
        to the nearest obstacle / floor edge; 0 if it touches, negative if it
        sticks out of the floor."""
        poly = np.asarray(poly, float)
        best, who = float(np.min(self.edge_dist(poly))), "floor edge"
        for nm, q in self.polys:
            d = G.polygon_polygon_dist(poly, q)
            if d < best:
                best, who = d, nm
        for nm, c, r in self.circles:
            if G.points_in_polygon(c[None], poly)[0]:
                d = 0.0
            else:
                d = float(G.points_to_polygon(c[None], poly)[0]) - r
                d = max(d, 0.0)
            if d < best:
                best, who = d, nm
        return (best, who) if with_names else best


def validate(m):
    """Plain-language warnings about the map (not fatal): docks off the floor
    or blocked, boards narrower than the robot, robots starting in an
    obstacle."""
    m = normalize(m)
    body = body_of(m)
    warns = []
    for b in m["boards"]:
        d = dock_for(m, b)
        L = math.dist(b["p1"], b["p2"])
        if L < body.width:
            warns.append("board %s is %.1f cm long, narrower than the robot (%.1f cm): the distance "
                         "sensor may see past its ends" % (b["name"], L, body.width))
        w_all = World(m)
        w_other = World(m, exclude_boards=(b["name"],))
        pc = float(w_all.clearance(np.array(d["predock"][:2])[None])[0])
        if pc < body.r_turn:
            warns.append("board %s: no room to turn at its pre-dock point (%.1f cm clear, needs %.1f)"
                         % (b["name"], pc, body.r_turn))
        sweep = body.drive_sweep(d["predock"][:2], d["dock"][:2], d["dock"][2])
        c, who = w_other.shape_clearance(sweep, with_names=True)
        if c <= 0.0:
            warns.append("board %s: the straight approach is blocked by the %s" % (b["name"], who))
    w = World(m)
    for r in m["robots"]:
        for what in ("start", "park"):
            p = pose_in(r[what])
            c, who = w.shape_clearance(body.corners([p])[0], with_names=True)
            if c <= 0.0:
                warns.append("robot %s %s spot overlaps the %s" % (r["name"], what, who))
    return warns
