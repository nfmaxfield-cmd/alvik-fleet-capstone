#!/usr/bin/env python3
"""Score a locked site map against tape measurements of every point (E0d).

    python tools/site/score.py --map site/map.json --truth docs/experiments/E00d-site-setup/data/truth.csv \\
        [--clicked site/map_clicked.json]

truth.csv: point,x,y,unit (unit "in" or "cm"; inches like 36 3/32). point is as the
locked map names it: "A: left p1" (landmark: anchor) or "edge x0: 50%" (an open
edge: only the coordinate across the edge counts).

Prints, per point, the error and the map's stated sd, then the pass rules of
docs/experiments/E00d-site-setup/README.md; writes <map folder>/score.csv.
With --clicked (the map before the survey, as clicked), scores that too.
"""
import argparse
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (HERE, TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import sitemap as SM  # noqa: E402


def read_truth(path):
    import landmark_pick as LP
    out = {}
    with open(path, newline="", encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            unit = (r.get("unit") or "in").strip().lower()
            if unit == "cm":
                xy = (float(r["x"]), float(r["y"]))
            else:
                x, y = LP.inches(r["x"]), LP.inches(r["y"])
                if x is None or y is None:
                    raise ValueError(f"{path}: could not read {r['x']}, {r['y']} as inches")
                xy = (x * 2.54, y * 2.54)
            out[r["point"].strip()] = np.array(xy)
    return out


def map_points(map_path):
    """{point: (xy, sd, kind, normal)} from a locked map's site block."""
    import json
    with open(map_path) as f:
        m = json.load(f)
    site = m.get("site") or {}
    out = {}
    for name, l in (site.get("landmarks") or {}).items():
        for a, p in l["points"].items():
            out[f"{name}: {a}"] = (np.array([p["x_cm"], p["y_cm"]]), p["sigma_cm"], "point", None)
    import fleetmap as FM
    fl = FM.normalize(m)["floor"]
    for side, e in (site.get("edges") or {}).items():
        if not e.get("open"):
            continue
        edge = SM.Edge(side, fl, True)
        for frac in (0.25, 0.5, 0.75):
            p = edge.point(frac, np.array([0.0, e.get("turn_deg", 0.0)]))
            out[f"edge {side}: {int(frac * 100)}%"] = (p, e.get("sigma_cm") or 0.5, "edge", edge.n)
    return out


def score(points, truth):
    rows = []
    for k, (xy, sd, kind, n) in points.items():
        if k not in truth:
            continue
        d = truth[k] - xy
        e = abs(float(np.dot(d, n))) if kind == "edge" else float(np.hypot(*d))
        rows.append(dict(point=k, error_cm=round(e, 3), sigma_cm=round(sd, 3), z=round(e / max(sd, 1e-6), 2),
                         error_in=SM.to_in(e)))
    return rows


def verdict(rows, label):
    e = np.array([r["error_cm"] for r in rows])
    z = np.array([r["z"] for r in rows])
    if not len(e):
        print(f"{label}: no points in common with the truth file")
        return
    print(f"\n{label}: {len(e)} points; median error {np.median(e):.2f} cm, 90% within {np.quantile(e, 0.9):.2f} cm, "
          f"largest {e.max():.2f} cm ({SM.to_in(float(e.max()))})")
    p1 = np.quantile(e, 0.9) <= 0.5 and e.max() <= 1.0
    inside = float(np.mean(z <= 2.45))
    p2 = inside >= 0.85
    print(f"  P1 (90% of points within 0.5 cm, none over 1.0 cm): {'PASS' if p1 else 'FAIL'}")
    print(f"  P2 (errors honest: at least 85% within 2.45 x the stated sd; 95% expected): {inside:.0%} -> "
          f"{'PASS' if p2 else 'FAIL'}")
    return float(np.median(e))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True)
    ap.add_argument("--truth", required=True)
    ap.add_argument("--clicked", help="the clicked map (before the survey), a fleet map.json")
    args = ap.parse_args(argv)
    truth = read_truth(args.truth)
    rows = score(map_points(args.map), truth)
    for r in rows:
        print(f"  {r['point']:22s} off {r['error_cm']:.2f} cm ({r['error_in']}), stated sd {r['sigma_cm']:.2f} cm")
    m_site = verdict(rows, "Locked site map")
    out = os.path.join(os.path.dirname(os.path.abspath(args.map)), "score.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else ["point"])
        w.writeheader()
        w.writerows(rows)
    if args.clicked:
        import fleetmap as FM
        m = FM.load(args.clicked)
        est = SM.Estimate(SM.landmarks_from_map(m), SM.edges_from_map(m, []))
        pts = {p["key"]: (p["xy"], p["sigma"], "point", None) for p in est.points()}
        rc = score(pts, truth)
        m_click = verdict(rc, "Clicked map (no survey)")
        if m_site is not None and m_click:
            print(f"  P3 (site setup at most half the clicks' median error): {m_site:.2f} vs {m_click:.2f} cm -> "
                  f"{'PASS' if m_site <= 0.5 * m_click else 'FAIL'}")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
