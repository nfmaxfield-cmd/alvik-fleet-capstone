#!/usr/bin/env python3
"""How the robots mapped the table (Nick, 2026-10-27 00:16): the engineer view's picture of a warm-up. Each robot's
camera track in its own colour (Left #2952CC, Center #C2410C, Right #7A3EC8), red circles round the perimeter points
they found, by kind (open edge: red ring; barrier: red ring with a filled centre; something on the table: dashed red
ring and its proposed no-go zone), the table as marked (grey dashed) and each side as found (barrier: solid dark line;
open: red dashes; partly both: both). The live page's explore map (e02_view.draw_explore) uses the same idea: lines
where each robot drove, circles where it met the edge.

    python3 tools/fleet/mapview.py --map warm_map.json --session logs/<warm-up> --out mapping.html

svg(...) returns the picture alone for the Mission Control tab's engineer view.
"""
import argparse
import csv
import html
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import fleetmap as FM   # noqa: E402

ROBOT_COLOURS = {"Left": "#2952CC", "Center": "#C2410C", "Right": "#7A3EC8"}
INK, MUTED, BORDER, RED, GROUND = "#101828", "#5B6474", "#E4E7EC", "#D92D20", "#F6F7F9"


def read_track(session):
    out = {}
    p = os.path.join(session, "cam_track.csv")
    if os.path.exists(p):
        with open(p, newline="") as f:
            for r in csv.DictReader(f):
                try:
                    out.setdefault(r["robot"], []).append((float(r["x_cm"]), float(r["y_cm"])))
                except (TypeError, ValueError):
                    pass
    return out


def svg(m, result, tracks, width=760, pad_cm=8.0):
    """The mapping picture as an SVG string, floor cm with y up."""
    f = m["floor"]
    x0, x1, y0, y1 = f["x_min"] - pad_cm, f["x_max"] + pad_cm, f["y_min"] - pad_cm, f["y_max"] + pad_cm
    k = width / (x1 - x0)
    h = (y1 - y0) * k

    def P(x, y):
        return "%.1f,%.1f" % ((x - x0) * k, (y1 - y) * k)

    o = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" width="%d" height="%d" role="img" '
         'aria-label="How the robots mapped the table">' % (width, h, width, h)]
    o.append('<rect x="0" y="0" width="%d" height="%d" fill="#FFFFFF"/>' % (width, h))
    # the table as marked
    o.append('<polygon points="%s %s %s %s" fill="#FBFBFC" stroke="%s" stroke-width="1.5" stroke-dasharray="6 5"/>'
             % (P(f["x_min"], f["y_min"]), P(f["x_max"], f["y_min"]), P(f["x_max"], f["y_max"]), P(f["x_min"], f["y_max"]),
                "#98A2B3"))
    # each side as found
    sides = (result or {}).get("sides") or m.get("sides") or {}
    mm = dict(m, sides=sides) if sides else m
    for sd in FM.SIDES:
        if sd not in sides:
            continue
        bar, opn = FM.side_parts(FM.normalize(mm), sd)
        for spans, style in ((bar, 'stroke="%s" stroke-width="4"' % INK),
                             (opn, 'stroke="%s" stroke-width="3" stroke-dasharray="7 5"' % RED)):
            for a, b in spans:
                if sides[sd]["kind"] == "unknown":
                    style = 'stroke="#98A2B3" stroke-width="3" stroke-dasharray="2 5"'
                if sd.startswith("x"):
                    p, q = P(f[sd], a), P(f[sd], b)
                else:
                    p, q = P(a, f[sd]), P(b, f[sd])
                o.append('<line x1="%s" y1="%s" x2="%s" y2="%s" %s stroke-linecap="round"/>'
                         % (tuple(p.split(",")) + tuple(q.split(",")) + (style,)))
    # proposed no-go zones
    for z in (result or {}).get("zones") or []:
        o.append('<polygon points="%s" fill="%s" fill-opacity="0.10" stroke="%s" stroke-width="1.5" '
                 'stroke-dasharray="4 3"/>' % (" ".join(P(*q) for q in z["polygon"]), RED, RED))
    # tracks
    for n, pts in tracks.items():
        if len(pts) >= 2:
            o.append('<polyline points="%s" fill="none" stroke="%s" stroke-width="2" stroke-opacity="0.85" '
                     'stroke-linejoin="round"/>' % (" ".join(P(x, y) for x, y in pts), ROBOT_COLOURS.get(n, MUTED)))
            o.append('<circle cx="%s" cy="%s" r="4" fill="%s"/>' % (tuple(P(*pts[-1]).split(",")) +
                                                                  (ROBOT_COLOURS.get(n, MUTED),)))
    # contacts
    for c in (result or {}).get("contacts") or []:
        if not c.get("point") or c["point"][0] is None:
            continue
        cx, cy = P(*c["point"]).split(",")
        if c["kind"] == "open":
            o.append('<circle cx="%s" cy="%s" r="9" fill="none" stroke="%s" stroke-width="2.5"/>' % (cx, cy, RED))
        elif c["kind"] == "barrier":
            o.append('<circle cx="%s" cy="%s" r="9" fill="none" stroke="%s" stroke-width="2.5"/>'
                     '<circle cx="%s" cy="%s" r="4" fill="%s"/>' % (cx, cy, RED, cx, cy, RED))
        elif c["kind"] == "object":
            o.append('<circle cx="%s" cy="%s" r="9" fill="none" stroke="%s" stroke-width="2" stroke-dasharray="3 3"/>'
                     % (cx, cy, RED))
        else:
            o.append('<circle cx="%s" cy="%s" r="9" fill="none" stroke="#98A2B3" stroke-width="2"/>' % (cx, cy))
    o.append("</svg>")
    return "".join(o)


LEGEND = [("line", "Each robot's path, seen by the camera (Left blue, Center orange, Right purple)"),
          ("open", "Open edge found (the edge sensor stopped the robot)"),
          ("barrier", "Barrier found (the front sensor met it at the side)"),
          ("object", "Something on the table (a no-go zone proposed, dashed)"),
          ("side", "Sides as found: solid = barrier, red dashes = open edge, grey dots = not felt; grey dashed = "
                   "the table as marked")]


def page(m, result, tracks, title="How the robots mapped the table", source=""):
    rows = []
    for kind, text in LEGEND:
        if kind == "line":
            sw = "".join('<span style="display:inline-block;width:14px;height:3px;background:%s;margin-right:2px;'
                         'vertical-align:middle"></span>' % c for c in ROBOT_COLOURS.values())
        elif kind == "open":
            sw = '<svg width="20" height="20"><circle cx="10" cy="10" r="7" fill="none" stroke="%s" stroke-width="2.5"/></svg>' % RED
        elif kind == "barrier":
            sw = ('<svg width="20" height="20"><circle cx="10" cy="10" r="7" fill="none" stroke="%s" stroke-width="2.5"/>'
                  '<circle cx="10" cy="10" r="3" fill="%s"/></svg>' % (RED, RED))
        elif kind == "object":
            sw = ('<svg width="20" height="20"><circle cx="10" cy="10" r="7" fill="none" stroke="%s" stroke-width="2" '
                  'stroke-dasharray="3 3"/></svg>' % RED)
        else:
            sw = ('<svg width="20" height="20"><line x1="2" y1="10" x2="18" y2="10" stroke="%s" stroke-width="4"/>'
                  '</svg>' % INK)
        rows.append('<div class="lg">%s<span>%s</span></div>' % (sw, html.escape(text)))
    words = []
    if result:
        import warmup as WU
        words = WU.words(result)
    return """<!doctype html><html><head><meta charset="utf-8"><title>%s</title>
<style>
body{margin:0;background:%s;font-family:Inter,system-ui,-apple-system,Segoe UI,sans-serif;color:%s}
.wrap{max-width:820px;margin:24px auto;padding:0 16px}
.panel{background:#fff;border:1px solid %s;border-radius:12px;padding:20px}
h1{font-size:18px;margin:0 0 4px} .sub{color:%s;font-size:13px;margin:0 0 14px}
.lg{display:flex;gap:8px;align-items:center;font-size:12.5px;color:%s;margin:4px 0}
.note{font-size:13px;color:%s;margin:10px 0 0} ul{font-size:13px;margin:12px 0 0;padding-left:18px}
.src{font-size:11.5px;color:%s;margin-top:12px}
</style></head><body><div class="wrap"><div class="panel">
<h1>%s</h1><p class="sub">Engineer view (Details) &middot; warm-up</p>%s
<p class="note">The robots crept into each corner and felt each side; where each one stopped tells what that side is.</p>
<div style="margin-top:10px">%s</div><ul>%s</ul>
<div class="src">Measured: robot positions from the overhead camera (cam_track.csv) and where the robots' own edge and
front sensors stopped them (coordinator.jsonl). Source: %s</div>
</div></div></body></html>""" % (html.escape(title), GROUND, INK, BORDER, MUTED, MUTED, MUTED, MUTED, html.escape(title),
                                  svg(m, result, tracks), "".join(rows), "".join("<li>%s</li>" % html.escape(w) for w in words),
                                  html.escape(source))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--session", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    import warmup as WU
    with open(a.map) as f:
        m = FM.normalize(json.load(f))
    res = WU.analyze(m, a.session)
    with open(a.out, "w") as f:
        f.write(page(m, res, read_track(a.session), source=os.path.basename(os.path.normpath(a.session))))
    print(a.out)


if __name__ == "__main__":
    main()
