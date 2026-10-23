#!/usr/bin/env python3
"""How good is the camera calibration the robots made themselves (explore)? Compares it with the corner-click
calibration over the whole board (Nick, 2026-10-23: explore replaces the corner clicks; this is the check that it
is as good).

For a grid of picture points over the board, both calibrations say where on the floor (robot-top height) that
point is. The two calibrations use different floor frames, so the explore one is first moved onto the reference
one (turned and shifted only: a rigid fit), and the distances left are the disagreement. Then the same with a scale
too: the scale it needs is the explore's distance error (the wheels set the size of everything; the robots' own
size is a cross-check).

    python tools/experiment/explore_check.py logs/e02-<session>             # against data/overhead_floorcal_roam1.json
    python tools/experiment/explore_check.py logs/e02-<session> --ref <calibration.json>
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, TOOLS, os.path.join(TOOLS, "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402

import e02_camera as CAM  # noqa: E402

TOP_CM = 3.7


def umeyama(a, b, scale):
    """b ~ s R a + t (2-D). Returns (s, R, t)."""
    ma, mb = a.mean(0), b.mean(0)
    A, B = a - ma, b - mb
    U, S, Vt = np.linalg.svd(B.T @ A)
    D = np.eye(2)
    if np.linalg.det(U @ Vt) < 0:
        D[1, 1] = -1
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / (A ** 2).sum()) if scale else 1.0
    return s, R, mb - s * R @ ma


def compare(cam_x, cam_ref, board_ref, n=9):
    """Floor points from a grid over the reference board, seen by the reference camera, then read back through
    the explore camera. Returns dict of the disagreement."""
    x0, y0, x1, y1 = board_ref
    ref, ex = [], []
    for gx in np.linspace(x0 + 5, x1 - 5, n):
        for gy in np.linspace(y0 + 5, y1 - 5, n):
            u, v = cam_ref.project((gx, gy, TOP_CM))
            fx, fy = cam_x.to_floor(u, v, TOP_CM)
            if np.isfinite(fx) and np.isfinite(fy):
                ref.append((gx, gy))
                ex.append((fx, fy))
    ref, ex = np.array(ref), np.array(ex)
    out = {}
    for name, sc in (("rigid", False), ("with_scale", True)):
        s, R, t = umeyama(ex, ref, sc)
        d = np.linalg.norm((s * (R @ ex.T)).T + t - ref, axis=1)
        out[name] = dict(scale=round(s, 4), median_cm=round(float(np.median(d)), 2),
                         p95_cm=round(float(np.percentile(d, 95)), 2), max_cm=round(float(d.max()), 2))
    out["points"] = len(ref)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--ref", default=os.path.join(HERE, "data", "overhead_floorcal_roam1.json"))
    ap.add_argument("--ref-board", default="0,0,122.15,121.9", help="the reference calibration's board, x0,y0,x1,y1 cm")
    a = ap.parse_args()
    cx = os.path.join(a.session, "cameras", "overhead.json")
    if not os.path.exists(cx):
        sys.exit("No explore calibration in %s (cameras/overhead.json)" % a.session)
    res = compare(CAM.load_camera(cx), CAM.load_camera(a.ref), [float(v) for v in a.ref_board.split(",")])
    st = json.load(open(os.path.join(a.session, "e02_state.json")))
    rep = (st.get("setup") or {}).get("explore") or {}
    print("Explore calibration (%s) against %s, %d floor points:" % (cx, os.path.basename(a.ref), res["points"]))
    r = res["rigid"]
    print("  turned and shifted only: median %.1f cm apart, 95%% within %.1f cm, worst %.1f cm" % (
        r["median_cm"], r["p95_cm"], r["max_cm"]))
    r = res["with_scale"]
    print("  with a scale too (x%.3f, i.e. explore's distances %+.1f%%): median %.1f cm, 95%% %.1f cm, worst %.1f cm" % (
        r["scale"], 100.0 * (1.0 / r["scale"] - 1.0), r["median_cm"], r["p95_cm"], r["max_cm"]))
    if rep:
        print("  explore's own numbers: fit %.1f px rms; board %.1f x %.1f cm; edge reach from the rays: %s" % (
            rep.get("rms_px", float("nan")), rep.get("W", float("nan")), rep.get("H", float("nan")),
            "%.1f cm" % rep["edge_ahead_cm"] if rep.get("edge_ahead_cm") is not None else "not told apart"))
    # where the edge guard stopped, by the reference calibration: the axle's distance from the nearest open edge
    # (the reference board's x0 / y0 sides by default: Nick's layout, boards at x1 and y1)
    rb = [float(v) for v in a.ref_board.split(",")]
    cref = CAM.load_camera(a.ref)
    reach = {}
    for n, stops in ((st.get("setup") or {}).get("explore_obs") or {}).items():
        for q in stops:
            if q.get("contact") != "edge" or q.get("skip"):
                continue
            x, y = cref.to_floor(q["u"], q["v"], TOP_CM)
            d = min(x - rb[0], y - rb[1])
            reach.setdefault(q.get("cue") or "?", []).append(round(float(d), 1))
    res["edge_stop_axle_to_edge_cm"] = reach
    for cue, v in sorted(reach.items()):
        print("  edge stops (%s cue): the axle %.1f cm from the open edge, median of %d (%s); e02_map assumes %.1f cm "
              "square on" % (cue, float(np.median(v)), len(v), ", ".join("%.1f" % d for d in v),
                             9.0 if cue == "bottom" else 3.5))
    json.dump(res, open(os.path.join(a.session, "explore_check.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
