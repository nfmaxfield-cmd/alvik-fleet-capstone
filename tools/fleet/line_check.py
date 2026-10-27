#!/usr/bin/env python3
"""Phase 0 gate (2026-10-27): how far from its planned lines did each robot drive, as the overhead camera saw it?

    python3 tools/fleet/line_check.py <run_plan session folder> <camera bridge folder> [--truth sim/truth.csv]
                                      [--png out.png] [--gate-cm 3]

Reads the plans in force (plan_given.json, plan_1.json, ...: every drive step of every plan, so a re-plan's new lines
count too), the run's start and end (coordinator.jsonl), and the bridge's track.csv (the camera's sure spots only).
For each robot: the distance of every camera spot to the nearest of its planned drive lines (cross-track error),
median / 95th percentile / max, and the share within the gate. Also the closest two robots came by the camera
(spots at most 0.15 s apart). With --truth (dry runs): the same from the simulator's true poses, and how far the
camera's spot was from the truth.

Gate (mission-control.md, Phase 0): every robot's 95th percentile within --gate-cm (3 cm). Writes line_check.json
next to the bridge's files and, with --png, a picture of the planned lines and the camera's spots.
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

GATE_CM = 3.0     # Nick's brief (Phase 0): "within about 3 cm of the planned lines". The 95th percentile is the
                  # chosen reading of "within" (a single camera jump should not fail a run; a pattern should).
                  # DOE: not needed (a pass mark, not a setting)


def seg_dist(p, a, b):
    ax, ay = a[0], a[1]
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-9:
        return math.hypot(p[0] - ax, p[1] - ay)
    f = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - ax - f * dx, p[1] - ay - f * dy)


def pct(v, q):
    if not v:
        return None
    s = sorted(v)
    k = (len(s) - 1) * q
    i = int(math.floor(k))
    j = min(i + 1, len(s) - 1)
    return s[i] + (s[j] - s[i]) * (k - i)


def load_lines(session):
    lines = {}
    files = [os.path.join(session, "plan_given.json")] + sorted(glob.glob(os.path.join(session, "plan_[0-9]*.json")))
    for fp in files:
        if not os.path.exists(fp):
            continue
        with open(fp) as f:
            plan = json.load(f)
        for n, r in plan["robots"].items():
            for st in r["steps"]:
                if st["type"] == "drive":
                    lines.setdefault(n, []).append((tuple(st["from"][:2]), tuple(st["to"][:2])))
                elif st["type"] in ("turn", "wait", "park"):
                    p = tuple(st["to"][:2]) if st.get("to") else None
                    if p:
                        lines.setdefault(n, []).append((p, p))
    return lines


def run_window(session):
    t0 = t1 = None
    with open(os.path.join(session, "coordinator.jsonl")) as f:
        for ln in f:
            try:
                e = json.loads(ln)
            except ValueError:
                continue
            if e.get("event") == "step_sent" and t0 is None:
                t0 = e["t"]
            if e.get("event") == "step_report" and e.get("state") == "done":
                t1 = e["t"]
    return t0, t1


def read_track(path, t0, t1, sure_only=True):
    out = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            t = float(r["t"])
            if (t0 and t < t0) or (t1 and t > t1):
                continue
            if sure_only and r.get("status", "seen") != "seen":
                continue
            out.setdefault(r["robot"], []).append((t, float(r["x"]), float(r["y"])))
    return out


def read_truth(path):
    out = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            out.setdefault(r["robot"], []).append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"])))
    return out


def at(series, t):
    import bisect
    ts = [q[0] for q in series]
    i = bisect.bisect_left(ts, t)
    if i <= 0 or i >= len(series):
        return None
    a, b = series[i - 1], series[i]
    f = (t - a[0]) / max(1e-9, b[0] - a[0])
    return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2])


def stats(v, gate):
    if not v:
        return dict(n=0)
    return dict(n=len(v), median_cm=round(pct(v, 0.5), 2), p95_cm=round(pct(v, 0.95), 2), max_cm=round(max(v), 2),
                within_gate_pct=round(100.0 * sum(1 for d in v if d <= gate) / len(v), 1))


def closest_pair(track, r_body=4.75):
    names = sorted(track)
    best = None
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            for t, x, y in track[a]:
                q = at(track[b], t)
                if q is None:
                    continue
                d = math.hypot(x - q[0], y - q[1])
                if best is None or d < best[0]:
                    best = (d, a, b, t)
    if best is None:
        return None
    return dict(centres_cm=round(best[0], 2), bodies_cm=round(best[0] - 2 * r_body, 2), robots=[best[1], best[2]],
                t=best[3])


def check(session, bridge, truth=None, gate=GATE_CM, png=None):
    lines = load_lines(session)
    t0, t1 = run_window(session)
    track = read_track(os.path.join(bridge, "track.csv"), t0, t1)
    allrows = read_track(os.path.join(bridge, "track.csv"), t0, t1, sure_only=False)
    res = dict(session=session, bridge=bridge, gate_cm=gate, window_s=None if t0 is None else round(t1 - t0, 1),
               robots={})
    tru = read_truth(truth) if truth else None
    for n in sorted(lines):
        cam = track.get(n, [])
        d = [min(seg_dist((x, y), a, b) for a, b in lines[n]) for _, x, y in cam]
        r = dict(camera=stats(d, gate), sure_share_pct=round(100.0 * len(cam) / max(1, len(allrows.get(n, []))), 1))
        if tru and n in tru:
            ts = [q for q in tru[n] if (t0 is None or q[0] >= t0) and (t1 is None or q[0] <= t1)]
            r["truth"] = stats([min(seg_dist((x, y), a, b) for a, b in lines[n]) for _, x, y in ts], gate)
            err = []
            for t, x, y in cam:
                q = at(tru[n], t)
                if q is not None:
                    err.append(math.hypot(x - q[0], y - q[1]))
            r["camera_vs_truth"] = stats(err, gate)
        r["pass"] = bool(d) and pct(d, 0.95) <= gate
        res["robots"][n] = r
    res["closest_pair_camera"] = closest_pair(track) if len(track) > 1 else None
    res["pass"] = bool(res["robots"]) and all(r["pass"] for r in res["robots"].values())
    with open(os.path.join(bridge, "line_check.json"), "w") as f:
        json.dump(res, f, indent=1)
    if png:
        draw(png, lines, track, res)
    return res


def draw(png, lines, track, res):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    col = {"Left": "#2952CC", "Center": "#C2410C", "Right": "#7A3EC8"}
    fig, ax = plt.subplots(figsize=(6.4, 6.4), dpi=110)
    for n, segs in lines.items():
        for a, b in segs:
            ax.plot([a[0], b[0]], [a[1], b[1]], color="#98A2B3", lw=1.2, zorder=1)
    for n, pts in track.items():
        r = res["robots"].get(n, {}).get("camera", {})
        ax.scatter([p[1] for p in pts], [p[2] for p in pts], s=3, color=col.get(n, "#101828"), zorder=2,
                   label="%s: p95 %.2f cm from its lines" % (n, r.get("p95_cm") or float("nan")))
    ax.set_xlim(0, 122)
    ax.set_ylim(0, 122)
    ax.set_aspect("equal")
    ax.set_xlabel("x, cm (roam1 floor frame)")
    ax.set_ylabel("y, cm")
    ax.set_title("Planned lines (grey) and the camera bridge's spots", fontsize=10)
    ax.legend(loc="upper right", fontsize=8, frameon=False)
    fig.text(0.01, 0.01, "Data: run_plan plans (every drive line) and cam_bridge track.csv (sure spots only).",
             fontsize=7, color="#5B6474")
    fig.tight_layout()
    fig.savefig(png)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("bridge")
    ap.add_argument("--truth")
    ap.add_argument("--gate-cm", type=float, default=GATE_CM)
    ap.add_argument("--png")
    a = ap.parse_args(argv)
    res = check(a.session, a.bridge, a.truth, a.gate_cm, a.png)
    for n, r in res["robots"].items():
        c = r["camera"]
        line = "%-6s camera: %s spots, median %s / p95 %s / max %s cm from its planned lines, %s%% within %.0f cm; " \
               "seen for sure in %s%% of pictures -> %s" % (
                   n, c.get("n"), c.get("median_cm"), c.get("p95_cm"), c.get("max_cm"), c.get("within_gate_pct"),
                   a.gate_cm, r["sure_share_pct"], "PASS" if r["pass"] else "FAIL")
        print(line)
        if "truth" in r:
            print("       simulator truth: p95 %s cm from its lines; camera spot vs truth median %s / p95 %s cm"
                  % (r["truth"].get("p95_cm"), r["camera_vs_truth"].get("median_cm"), r["camera_vs_truth"].get("p95_cm")))
    cp = res.get("closest_pair_camera")
    if cp:
        print("Closest two robots by the camera: %s and %s, bodies %.1f cm apart" % (cp["robots"][0], cp["robots"][1],
                                                                                   cp["bodies_cm"]))
    print("GATE: %s" % ("PASS" if res["pass"] else "FAIL"))
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
