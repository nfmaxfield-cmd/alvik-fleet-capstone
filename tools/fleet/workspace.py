"""The general workspace (school demo, docs/build-plans/mission-control.md, 2026-10-27): a table's rectangle, each
side's kind (open edge / barrier / partly both / not felt yet), named spots A, B, C... and no-go zones. It lives in
map.json (fleetmap.py: "sides", "open_margin_cm", "spots", keep-outs with a "status"), so the planner, run_plan.py
and the page read one file. Nothing here assumes boards.

This module adds what the other parts need from it:
  blank_map(rect, robots)       a map of a new table: every side "unknown" (kept away from like an open edge)
  set_side / add_spot / add_nogo / confirm_nogo / delete_nogo     edits (return a normalized copy)
  to_board(m)                   the E02 runner's Board (tools/experiment/e02_plan.py) with these sides
  firmware_params(m, border_cm) the robots' own border settings (firmware 2026-10-27a, border.h)
  words(m)                      plain words for the page and the logs
"""
import copy
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import fleetmap as FM  # noqa: E402

SIDE_WORDS = {"x_min": "left side (x = %.0f)", "x_max": "right side (x = %.0f)", "y_min": "bottom side (y = %.0f)",
              "y_max": "top side (y = %.0f)"}
KIND_CODE = {"barrier": 0, "open": 1, "unknown": 1, "partial": 2}    # firmware bd_k_* (unknown = open)
SPOT_NAMES = [chr(ord("A") + i) for i in range(26)]


def blank_map(rect, robots=(), open_margin_cm=FM.OPEN_MARGIN_CM, robot=None):
    """A new table (x0, y0, x1, y1 cm): every side unknown, no spots, no zones. robots: [(name, [x, y, heading])]."""
    x0, y0, x1, y1 = (float(v) for v in rect)
    m = {"floor": {"x_min": x0, "x_max": x1, "y_min": y0, "y_max": y1},
         "sides": {k: {"kind": "unknown"} for k in FM.SIDES}, "open_margin_cm": open_margin_cm,
         "robots": [{"name": n, "start": list(p), "park": list(p)} for n, p in robots], "tasks": {}}
    if robot:
        m["robot"] = dict(robot)
    return FM.normalize(m)


def set_side(m, side, kind, barrier=None, source=None):
    m = copy.deepcopy(m)
    m.setdefault("sides", {k: {"kind": "unknown"} for k in FM.SIDES})
    sd = {"kind": kind}
    if barrier:
        sd["barrier"] = [list(b) for b in barrier]
    if source:
        sd["source"] = source
    m["sides"][side] = sd
    return FM.normalize(m)


def next_spot_name(m):
    used = set((m.get("spots") or {}))
    return next(n for n in SPOT_NAMES if n not in used)


def add_spot(m, xy, name=None, heading=None):
    m = copy.deepcopy(m)
    nm = name or next_spot_name(m)
    m.setdefault("spots", {})[nm] = [float(xy[0]), float(xy[1])] + ([float(heading)] if heading is not None else [])
    return FM.normalize(m), nm


def add_nogo(m, polygon, name=None, status="proposed", source="user", note=None, frame=None):
    m = copy.deepcopy(m)
    n = sum(1 for k in m["keepouts"] if k.get("status")) + 1
    ko = {"name": name or "no-go %d" % n, "polygon": [list(map(float, p)) for p in polygon], "status": status,
          "source": source}
    if note:
        ko["note"] = note
    if frame is not None:
        ko["frame"] = frame
    m["keepouts"].append(ko)
    return FM.normalize(m), ko["name"]


def confirm_nogo(m, name):
    m = copy.deepcopy(m)
    for k in m["keepouts"]:
        if k["name"] == name:
            k["status"] = "confirmed"
            return FM.normalize(m)
    raise FM.MapError("no no-go zone called %r" % name)


def delete_nogo(m, name):
    m = copy.deepcopy(m)
    before = len(m["keepouts"])
    m["keepouts"] = [k for k in m["keepouts"] if k["name"] != name]
    if len(m["keepouts"]) == before:
        raise FM.MapError("no no-go zone called %r" % name)
    return FM.normalize(m)


def nogo_zones(m, status=None):
    return [k for k in m["keepouts"] if k.get("status") and (status is None or k["status"] == status)]


def circle_poly(c, r, n=12):
    return [[c[0] + r * math.cos(2 * math.pi * i / n), c[1] + r * math.sin(2 * math.pi * i / n)] for i in range(n)]


def side_line(m, side):
    f = m["floor"]
    return {"x_min": f["x_min"], "x_max": f["x_max"], "y_min": f["y_min"], "y_max": f["y_max"]}[side]


def nearest_side(m, x, y):
    """(side, distance in cm from the point to that side's line, along-coordinate)."""
    f = m["floor"]
    d = {"x_min": x - f["x_min"], "x_max": f["x_max"] - x, "y_min": y - f["y_min"], "y_max": f["y_max"] - y}
    k = min(d, key=lambda s: abs(d[s]))
    return k, d[k], (y if k.startswith("x") else x)


def to_board(m, face_cm=None):
    """The E02 runner's Board for this table: its rectangle, and open_edges() from the side kinds. The runner's
    x board / y board (the two-board layout) are set to a barrier side when there is one, else marked missing."""
    tools = os.path.dirname(HERE)
    exp = os.path.join(tools, "experiment")
    if exp not in sys.path:
        sys.path.insert(0, exp)
    import e02_plan as EP  # noqa: E402
    f = m["floor"]
    sides = {k: v["kind"] for k, v in (m.get("sides") or {}).items()} or {k: "barrier" for k in FM.SIDES}
    xb = 1 if sides.get("y_max") == "barrier" and sides.get("y_min") != "barrier" else 0
    yb = 1 if sides.get("x_max") == "barrier" and sides.get("x_min") != "barrier" else 0
    kw = {} if face_cm is None else {"face_cm": face_cm}
    b = EP.Board((f["x_min"], f["y_min"], f["x_max"], f["y_max"]), xb, yb, **kw)
    if sides.get("y_max" if xb else "y_min") != "barrier":
        b.missing.add("x")
    if sides.get("x_max" if yb else "x_min") != "barrier":
        b.missing.add("y")
    b.sides = sides
    return b


def firmware_params(m, border_cm):
    """The robots' own virtual border for all four sides (firmware 2026-10-27a, border.h), as {param: value}:
    bd_on 1, the four lines, each side's kind (0 barrier, 1 open, 2 partial) and, for a partial side, the one span
    along it that is open (from the lowest open part to the highest: a partial side with two open stretches is kept
    away from along all of them and the barrier between)."""
    f = m["floor"]
    p = {"bd_on": 1, "border_cm": float(border_cm), "bd_x0": f["x_min"], "bd_x1": f["x_max"], "bd_y0": f["y_min"],
         "bd_y1": f["y_max"]}
    short = {"x_min": "x0", "x_max": "x1", "y_min": "y0", "y_max": "y1"}
    for sd in FM.SIDES:
        kind = (m.get("sides") or {}).get(sd, {"kind": "barrier"})["kind"]
        p["bd_k_" + short[sd]] = KIND_CODE[kind]
        if kind == "partial":
            _, opn = FM.side_parts(m, sd)
            p["bd_lo_" + short[sd]] = round(min(a for a, _ in opn), 1)
            p["bd_hi_" + short[sd]] = round(max(b for _, b in opn), 1)
    return {k: (round(v, 2) if isinstance(v, float) else v) for k, v in p.items()}


def side_words(m, side):
    sd = (m.get("sides") or {}).get(side, {"kind": "barrier"})
    base = SIDE_WORDS[side] % side_line(m, side)
    if sd["kind"] == "partial":
        return "%s: partly barrier (%s), the rest open" % (
            base, ", ".join("%.0f-%.0f cm" % tuple(b) for b in sd["barrier"]))
    return "%s: %s" % (base, {"open": "open edge", "barrier": "barrier", "unknown": "not felt yet (kept away from "
                                                                                     "like an open edge)"}[sd["kind"]])


def words(m):
    f = m["floor"]
    out = ["table %.0f x %.0f cm" % (f["x_max"] - f["x_min"], f["y_max"] - f["y_min"])]
    if m.get("sides"):
        out += [side_words(m, s) for s in FM.SIDES]
    for nm, p in sorted((m.get("spots") or {}).items()):
        out.append("spot %s at (%.0f, %.0f)" % (nm, p[0], p[1]))
    for k in nogo_zones(m):
        out.append("no-go zone %s (%s%s)" % (k["name"], k["status"], ", " + k["note"] if k.get("note") else ""))
    return out
