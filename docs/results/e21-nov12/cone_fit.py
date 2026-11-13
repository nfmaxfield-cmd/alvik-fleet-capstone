#!/usr/bin/env python3
"""e21 (Nick 12 Nov, decision 1: one error SHAPE per robot, a cone along its own track): the measurements behind the
shape's starting guess, from real home free-roam logs. Logs only; nothing touches a robot.

    python3 docs/results/e21-nov12/cone_fit.py <run folder> [<run folder> ...] --out docs/results/e21-nov12/cone_fit.json

A run folder is a free roam's log folder (follow.csv, bridge/track.csv, events.csv, tracking.csv, camera.csv).

1. Stretches (Nick's cone, scratchpad/lossaudit/cone.py, copied here with its folders as arguments): every stretch of
   15-60 cm the robot drove while its own filter took NO camera fix (fixes_used unchanged), its own displacement against
   the camera's over the same stretch, split ALONG its own displacement and ACROSS it (sideways).
2. The 95 % fit of each (quantile regression, every coefficient >= 0):
     along  = b_al + a_al x s
     across = b_ac + a_ac x s + c_ac x (deg turned x s, in radians: the sideways reach of a heading doubt)
   s = cm driven; the start term b is the surprise circle (slips, bumps at the start of a stretch: the short stretches'
   along-path tail).
3. Event jumps (decision 1: "events widen it at once, by measured amounts"): for every event of each kind the robots
   report, the own-vs-camera gap 2-3 s after it less the gap in the second before it; its 90th and 95th percentiles.
4. Fix delays: camera.csv's measured delay (median and 90th percentile of each minute), for the firmware's age limit.

Changelog
  e21-2026-11-12a  first version (cone.py from the coordinator's loss audit, plus the fits, event jumps and delays).
"""
import argparse
import bisect
import collections
import csv
import json
import math
import os

VERSION = "cone-fit-e21-2026-11-12a"


def q(v, p):
    v = sorted(v)
    return v[int(round(p * (len(v) - 1)))] if v else None


def qreg(X, y, tau):
    import numpy as np
    from scipy.optimize import linprog
    X = np.asarray(X, float)
    y = np.asarray(y, float)
    n, k = X.shape
    c = np.r_[np.zeros(k), tau * np.ones(n), (1 - tau) * np.ones(n)]
    A = np.hstack([X, np.eye(n), -np.eye(n)])
    res = linprog(c, A_eq=A, b_eq=y, bounds=[(0, None)] * (k + 2 * n), method="highs")
    return [float(v) for v in res.x[:k]]


def load(R):
    F = collections.defaultdict(list)
    for r in csv.DictReader(open(os.path.join(R, "follow.csv"))):
        if r["state"] == "nopose":
            continue
        try:
            F[r["robot"]].append((float(r["t_recv"]), float(r["x_cm"]), float(r["y_cm"]), float(r["th_deg"]),
                                  int(r["fixes_used"]), r["run"]))
        except ValueError:
            pass
    C = collections.defaultdict(list)
    for r in csv.DictReader(open(os.path.join(R, "bridge", "track.csv"))):
        if r["status"] == "seen" and r["x"]:
            C[r["robot"]].append((float(r["t"]), float(r["x"]), float(r["y"]), r.get("why", "")))
    return F, C


def cam_at(c, ct, t, tol=0.15):
    i = bisect.bisect(ct, t)
    best = min(c[max(0, i - 1):i + 1], key=lambda z: abs(z[0] - t), default=None)
    return best if best and abs(best[0] - t) < tol else None


def event_times(R):
    ev = collections.defaultdict(list)
    try:
        for e in csv.DictReader(open(os.path.join(R, "events.csv"))):
            if e["kind"] in EVENT_KINDS:
                ev[e["robot"]].append(float(e["t"]))
    except FileNotFoundError:
        pass
    return ev


def stretches(R, F, C):
    rows = []
    EV = event_times(R)
    for rb, f in F.items():
        c = C[rb]
        ct = [z[0] for z in c]
        i = 0
        while i < len(f) - 1:
            j, dist = i, 0.0
            while j + 1 < len(f) and f[j + 1][4] == f[i][4] and f[j + 1][5] == f[i][5] and f[j + 1][0] - f[j][0] < 0.5:
                dist += math.hypot(f[j + 1][1] - f[j][1], f[j + 1][2] - f[j][2])
                j += 1
                if dist >= 60:
                    break
            if dist >= 15:
                a, b = f[i], f[j]
                ca, cb = cam_at(c, ct, a[0]), cam_at(c, ct, b[0])
                if ca and cb:
                    ox, oy = b[1] - a[1], b[2] - a[2]
                    L = math.hypot(ox, oy)
                    if L >= 10:
                        ex, ey = (cb[1] - ca[1]) - ox, (cb[2] - ca[2]) - oy
                        ux, uy = ox / L, oy / L
                        turn = abs((b[3] - a[3] + 180) % 360 - 180)
                        evn = sum(1 for te in EV.get(rb, ()) if a[0] - 1.0 <= te <= b[0])
                        rows.append(dict(run=os.path.basename(R), robot=rb, s=dist, along=ex * ux + ey * uy,
                                         across=-ex * uy + ey * ux, turn=turn, L=L, events=evn))
            i = max(j, i + 1)
    return rows


def gap_series(F, C):
    """Per robot: [(t, own-vs-camera gap cm)] at each camera sighting."""
    out = {}
    for rb, c in C.items():
        f = F.get(rb)
        if not f:
            continue
        ft = [z[0] for z in f]
        g = []
        for z in c:
            k = bisect.bisect(ft, z[0])
            if k <= 0 or k >= len(f) or f[k][0] - f[k - 1][0] > 1.0:
                continue
            a, b = f[k - 1], f[k]
            w = (z[0] - a[0]) / max(1e-6, b[0] - a[0])
            g.append((z[0], math.hypot(z[1] - (a[1] + w * (b[1] - a[1])), z[2] - (a[2] + w * (b[2] - a[2])))))
        out[rb] = g
    return out


EVENT_KINDS = {"front stop": "front_stop", "felt a robot": "felt_robot", "did not move": "did_not_move",
               "hazard": "hazard", "pose re-seeded": "reseed"}


def event_jumps(R, F, C, G):
    """The gap 2-3 s after each event less the gap in the second before it (cm), per kind."""
    evs = []
    try:
        for e in csv.DictReader(open(os.path.join(R, "events.csv"))):
            k = EVENT_KINDS.get(e["kind"])
            if k:
                evs.append((float(e["t"]), e["robot"], k))
    except FileNotFoundError:
        pass
    try:
        for e in csv.DictReader(open(os.path.join(R, "tracking.csv"))):
            if e["what"].startswith("close"):
                for rb in e["robot"].split("+"):
                    evs.append((float(e["t"]), rb, "merge"))
    except FileNotFoundError:
        pass
    for rb, c in C.items():                                  # the bridge's merged outlines
        for z in c:
            if "merged" in z[3]:
                evs.append((z[0], rb, "merge"))
    for rb, f in F.items():                                  # a restart: its follow run's id changes
        for a, b in zip(f, f[1:]):
            if a[5] != b[5]:
                evs.append((b[0], rb, "restart"))
    out = collections.defaultdict(list)
    seen = set()
    for t, rb, k in evs:
        key = (rb, k, round(t))
        if key in seen:
            continue
        seen.add(key)
        g = G.get(rb) or []
        before = [v for tt, v in g if t - 1.0 <= tt <= t]
        after = [v for tt, v in g if t + 2.0 <= tt <= t + 3.0]
        if before and after:
            out[k].append(max(after) - min(before))
    return out


def delays(R):
    rows = []
    try:
        for r in csv.DictReader(open(os.path.join(R, "camera.csv"))):
            rows.append((float(r["delay_med_s"]), float(r["delay_p90_s"])))
    except (FileNotFoundError, ValueError, KeyError):
        pass
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out")
    a = ap.parse_args()
    rows, jumps, dl = [], collections.defaultdict(list), {}
    for R in a.runs:
        F, C = load(R)
        rows += stretches(R, F, C)
        G = gap_series(F, C)
        for k, v in event_jumps(R, F, C, G).items():
            jumps[k] += v
        dl[os.path.basename(R)] = delays(R)
    print("stretches", len(rows))
    tab = []
    for lo, hi in ((15, 30), (30, 45), (45, 61)):
        s = [r for r in rows if lo <= r["s"] < hi]
        al = [abs(r["along"]) for r in s]
        ac = [abs(r["across"]) for r in s]
        tab.append(dict(lo=lo, hi=hi, n=len(s), along_med=q(al, .5), along_p90=q(al, .9), along_p95=q(al, .95),
                        across_med=q(ac, .5), across_p90=q(ac, .9), across_p95=q(ac, .95)))
        print("%d-%d cm n=%d along median %.1f p90 %.1f p95 %.1f | across median %.1f p90 %.1f p95 %.1f" % (
            lo, hi, len(s), q(al, .5), q(al, .9), q(al, .95), q(ac, .5), q(ac, .9), q(ac, .95)))
    st = [r for r in rows if r["turn"] < 10]
    cu = [r for r in rows if r["turn"] >= 45]
    turn_split = dict(straight_n=len(st), straight_across_med=q([abs(r["across"]) for r in st], .5),
                      straight_along_med=q([abs(r["along"]) for r in st], .5), curvy_n=len(cu),
                      curvy_across_med=q([abs(r["across"]) for r in cu], .5),
                      curvy_along_med=q([abs(r["along"]) for r in cu], .5))
    print(turn_split)
    fit = {}
    for tau in (0.90, 0.95):
        b_al, a_al = qreg([[1, r["s"]] for r in rows], [abs(r["along"]) for r in rows], tau)
        b_ac, a_ac, c_ac = qreg([[1, r["s"], math.radians(min(r["turn"], 90.0)) * r["s"]] for r in rows],
                                [abs(r["across"]) for r in rows], tau)
        fit["q%02d" % round(tau * 100)] = dict(b_along=b_al, a_along=a_al, b_across=b_ac, a_across=a_ac,
                                               c_across_per_rad_cm=c_ac)
        print("q%.2f along = %.2f + %.3f s | across = %.2f + %.3f s + %.3f x turn(rad) x s" % (
            tau, b_al, a_al, b_ac, a_ac, c_ac))
    quiet = [r for r in rows if not r["events"]]
    for tau in (0.90, 0.95):
        b_al, a_al = qreg([[1, r["s"]] for r in quiet], [abs(r["along"]) for r in quiet], tau)
        b_ac, a_ac, c_ac = qreg([[1, r["s"], math.radians(min(r["turn"], 90.0)) * r["s"]] for r in quiet],
                                [abs(r["across"]) for r in quiet], tau)
        fit["quiet_q%02d" % round(tau * 100)] = dict(b_along=b_al, a_along=a_al, b_across=b_ac, a_across=a_ac,
                                                     c_across_per_rad_cm=c_ac, n=len(quiet))
        print("no reported event (n=%d) q%.2f along = %.2f + %.3f s | across = %.2f + %.3f s + %.3f x turn(rad) x s" % (
            len(quiet), tau, b_al, a_al, b_ac, a_ac, c_ac))
    # (the along error is one-sided: back along its own track, stuck wheels) each side at 97.5 %
    bf, af = qreg([[1, r["s"]] for r in quiet], [max(0.0, r["along"]) for r in quiet], 0.975)
    bb, ab = qreg([[1, r["s"]] for r in quiet], [max(0.0, -r["along"]) for r in quiet], 0.975)
    fit["along_sides"] = dict(b_fwd=bf, a_fwd=af, b_back=bb + ab * 0.0, a_back=ab, q=0.975, n=len(quiet))
    print("no reported event, along each side at 97.5 %%: forward %.2f + %.3f s, back %.2f + %.3f s" % (bf, af, bb, ab))
    sh = [abs(r["along"]) for r in quiet if r["s"] < 30]
    fit["quiet_short_along"] = dict(n=len(sh), p90=q(sh, .9), p95=q(sh, .95))
    print("no reported event, 15-30 cm, along p90 %.1f p95 %.1f (n=%d)" % (q(sh, .9), q(sh, .95), len(sh)))
    # covered: the 95 % fit's coverage on its own stretches (in sample)
    f95 = fit["q95"]
    cov_al = sum(abs(r["along"]) <= f95["b_along"] + f95["a_along"] * r["s"] for r in rows) / len(rows)
    cov_ac = sum(abs(r["across"]) <= f95["b_across"] + f95["a_across"] * r["s"] + f95["c_across_per_rad_cm"] *
                 math.radians(min(r["turn"], 90.0)) * r["s"] for r in rows) / len(rows)
    print("in-sample coverage along %.3f across %.3f" % (cov_al, cov_ac))
    ej = {}
    for k, v in sorted(jumps.items()):
        pos = [max(0.0, x) for x in v]
        ej[k] = dict(n=len(v), p50=q(pos, .5), p90=q(pos, .9), p95=q(pos, .95))
        print("event %-13s n=%4d jump p50 %.1f p90 %.1f p95 %.1f" % (k, len(v), q(pos, .5), q(pos, .9), q(pos, .95)))
    alld = [x for v in dl.values() for x in v]
    dsum = dict(minutes=len(alld), med_of_med=q([x[0] for x in alld], .5), p90_of_p90=q([x[1] for x in alld], .9),
                max_p90=max([x[1] for x in alld] or [0]))
    print("delays", dsum)
    if a.out:
        json.dump(dict(version=VERSION, runs=[os.path.basename(R) for R in a.runs], n=len(rows), table=tab,
                       turn_split=turn_split, fit=fit, coverage_q95=dict(along=cov_al, across=cov_ac), event_jumps=ej,
                       delays=dsum, stretches=[{k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()}
                                               for r in rows]), open(a.out, "w"), indent=0)


if __name__ == "__main__":
    main()
