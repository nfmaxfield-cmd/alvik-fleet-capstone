#!/usr/bin/env python3
"""e21 (Nick 12 Nov, decision 4 and the acceptance): the real home runs replayed through the Pi's new consistency escape
(freeroam_core.consistent_escape). Logs only; nothing touches a robot.

    python3 docs/results/e21-nov12/replay_escape.py --onsets docs/audits/2026-11-12-loss/onsets.json \
        --runs <run folder> ... --out docs/results/e21-nov12/replay_escape.json

For every camera sighting of a robot (bridge/track.csv, its label), its own pose at that time (follow.csv) and its own
filter's sd then (pos_sd_cm, so the gate is the firmware's real one): the rule fires when the offsets of its recent
sightings agree with each other within the camera's noise (AGREE_K x sqrt 2 x sigma) and lie beyond the gate (4 sds, at
least 3 cm), for at least as many sightings as the noise cannot explain (ESCAPE_ALPHA), with no other robot (its camera
spot) within two half-bodies plus the gate of the spot. The first firing in each of the audit's 147 divergence episodes
(onsets.json: own pose 8 cm or more from the camera for 3 s or more) and in each "lost" spell of the two 11 Nov runs
(lost.csv) is compared with when the run really ended it.

The escape's identity guards that need the live controller (the camera's learned trust, the layer's elimination flag)
are not replayed: the bridge's label is taken as it was. Episodes the audit classed as a camera mix-up are counted
apart (a firing there could be a wrong correction).

Changelog
  e21-2026-11-12a  first version.
  e21-2026-11-12b  a driving robot's offsets count only beyond 12 cm, as the live rule (e21 step 7).
"""
import argparse
import bisect
import collections
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "tools", "fleet"))
import camera_trust as CT          # noqa: E402
import freeroam_core as FC         # noqa: E402

VERSION = "replay-escape-e21-2026-11-12b"
SIGMA = 1.0          # the camera's noise per sighting (cm): the bridge's measured standing scatter is about 1 cm (the
                     # audit's own assumption, Sweep 2); the live rule uses each robot's measured noise


def load(R):
    F = collections.defaultdict(list)
    for r in csv.DictReader(open(os.path.join(R, "follow.csv"))):
        if r["state"] == "nopose":
            continue
        try:
            F[r["robot"]].append((float(r["t_recv"]), float(r["x_cm"]), float(r["y_cm"]), float(r["pos_sd_cm"])))
        except ValueError:
            pass
    C = collections.defaultdict(list)
    for r in csv.DictReader(open(os.path.join(R, "bridge", "track.csv"))):
        if r["status"] == "seen" and r["x"]:
            C[r["robot"]].append((float(r["t"]), float(r["x"]), float(r["y"])))
    return F, C


def own_at(f, ft, t):
    k = bisect.bisect(ft, t)
    if k <= 0 or k >= len(f) or f[k][0] - f[k - 1][0] > 1.0:
        return None
    a, b = f[k - 1], f[k]
    w = (t - a[0]) / max(1e-6, b[0] - a[0])
    return a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2]), a[3]


def fires(F, C):
    """{robot: [t, ...]} the times the rule would fire (the window cleared after each firing, as the live rule)."""
    out = {}
    others = {rb: ([q[0] for q in c], c) for rb, c in C.items()}
    for rb, c in C.items():
        f = F.get(rb)
        if not f:
            continue
        ft = [q[0] for q in f]
        w, res = [], []
        for t, x, y in c:
            o = own_at(f, ft, t)
            if o is None:
                continue
            off = (x - o[0], y - o[1])
            gate = max(3.0, 4.0 * o[2])
            if math.hypot(*off) <= gate:
                w = []
                continue
            w.append((t, off))
            w = [q for q in w[-24:] if t - q[0] <= 4.0]
            mx = sum(q[1][0] for q in w) / len(w)
            my = sum(q[1][1] for q in w) / len(w)
            tol = CT.AGREE_K * SIGMA * math.sqrt(2.0)
            if any(math.hypot(q[1][0] - mx, q[1][1] - my) > tol for q in w):
                w = [q for q in w if math.hypot(q[1][0] - off[0], q[1][1] - off[1]) <= tol]
                continue
            dm = math.hypot(mx, my)
            # (as the live rule since e21 7) a driving robot (its own pose moved 1 cm or more in the last second): only
            # offsets beyond 12 cm (REACQUIRE_MAX_CM); smaller ones are e08's re-seed's
            o1 = own_at(f, ft, t - 1.0)
            if o1 is not None and math.hypot(o[0] - o1[0], o[1] - o1[1]) >= FC.STAND_CM and dm <= max(gate,
                                                                                                    FC.REACQUIRE_MAX_CM):
                continue
            p1 = math.exp(-max(0.0, dm - gate) ** 2 / (2 * SIGMA ** 2)) if dm > gate else 1.0
            need = max(FC.ESCAPE_MIN_N, int(math.ceil(math.log(FC.ESCAPE_ALPHA) / math.log(p1)))) if 0 < p1 < 1 else 10 ** 6
            if len(w) < need:
                continue
            near = False
            for ob, (ot, oc) in others.items():
                if ob == rb:
                    continue
                k = bisect.bisect(ot, t)
                for q in oc[max(0, k - 1):k + 1]:
                    if abs(q[0] - t) <= 0.5 and math.hypot(q[1] - x, q[2] - y) <= 2 * FC.HALF + gate:
                        near = True
                fo = F.get(ob)                  # the live rule also checks the other robots' own poses
                if fo and not near:
                    oo = own_at(fo, [z[0] for z in fo], t)
                    if oo is not None and math.hypot(oo[0] - x, oo[1] - y) <= 2 * FC.HALF + gate:
                        near = True
            if near:
                continue
            res.append(t)
            w = []
        out[rb] = res
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--onsets", required=True)
    ap.add_argument("--runs", nargs="+", required=True)
    ap.add_argument("--out")
    a = ap.parse_args()
    eps = json.load(open(a.onsets))
    byrun = {os.path.basename(R): R for R in a.runs}
    FI, lost_rows = {}, []
    for name, R in byrun.items():
        F, C = load(R)
        FI[name] = fires(F, C)
        try:
            for r in csv.DictReader(open(os.path.join(R, "lost.csv"))):
                lost_rows.append(dict(run=name, robot=r["robot"], t0=float(r["t0"]), t1=float(r["t1"]),
                                      dur=float(r["dur_s"])))
        except FileNotFoundError:
            pass
    rows = []
    for e in eps:
        if e["run"] not in FI:
            continue
        t_big = e["t_on"] + e["build_s"]
        t_end = t_big + e["dur_s"]
        fs = [t for t in FI[e["run"]].get(e["robot"], []) if e["t_on"] <= t <= t_end]
        mix = e["cat"].startswith("C") or "confused" in e["cat"]
        rows.append(dict(run=e["run"], robot=e["robot"], cat=e["cat"], dur_s=round(t_end - e["t_on"], 1), mixup=mix,
                         fired=bool(fs), sooner_s=round(t_end - fs[0], 1) if fs else None,
                         after_big_s=round(fs[0] - t_big, 1) if fs else None))
    lrows = []
    for L in lost_rows:
        if not L["run"].startswith("20261111"):
            continue
        fs = [t for t in FI[L["run"]].get(L["robot"], []) if L["t0"] <= t <= L["t1"]]
        lrows.append(dict(L, fired=bool(fs), sooner_s=round(L["t1"] - fs[0], 1) if fs else None))

    def summ(rs, key="sooner_s"):
        v = sorted(r[key] for r in rs if r["fired"])
        q = (lambda p: v[int(round(p * (len(v) - 1)))]) if v else (lambda p: None)
        return dict(n=len(rs), fired=len(v), sooner_median_s=q(0.5), sooner_p90_s=q(0.9),
                    sooner_total_s=round(sum(v), 1), real_total_s=round(sum(r.get("dur_s", r.get("dur", 0)) for r in rs),
                                                                         1))
    res = dict(version=VERSION, sigma_cm=SIGMA, episodes=summ(rows), episodes_not_mixup=summ([r for r in rows if not
                                                                                              r["mixup"]]),
               episodes_mixup=summ([r for r in rows if r["mixup"]]), lost_1111=summ(lrows),
               by_cat={c: summ([r for r in rows if r["cat"] == c]) for c in sorted({r["cat"] for r in rows})},
               fires_per_run={k: sum(len(v) for v in d.values()) for k, d in FI.items()}, rows=rows, lost_rows=lrows)
    print(json.dumps({k: v for k, v in res.items() if k not in ("rows", "lost_rows")}, indent=1))
    if a.out:
        json.dump(res, open(a.out, "w"), indent=0)


if __name__ == "__main__":
    main()
