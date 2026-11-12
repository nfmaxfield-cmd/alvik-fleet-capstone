#!/usr/bin/env python3
"""Replay one free-roam run's camera sightings through the tracking layer, e17 OFF or ON, and measure it. Logs only.

Copied from tools/analysis/layer_replay.py (replay(), commit 3f37317, e17) and extended with the measures this report
needs; the camera watch is ON by default (as the runs since 7 Nov had it; layer_replay's default is off, the 4th
argument 0 gives that); the feeding is unchanged (own poses interpolated at each picture's time, as the run fed them: --at-pictures).

    python3 replay_one.py RUN_FOLDER off|on|on-speed|on-noelim|on-nogate OUT.json [watch 1|0]      (env REPLAY_LAG_S, default 0)

off = the layer as the runs had it before e17: MIRLT_OWN_RESET_RULE=speed, no elimination, the fixed gate.
on  = e17's defaults: the velocity re-set rule, elimination, the gate from each robot's record.
The switches are read when tracking_layer is imported, so each mode runs in its own process (replay_all.py does that).

Changelog
  2026-11-12 v1  first version (replay-all-nov12); sightings after the run's last FOLLOW dropped; on-* modes switch one e17 part back
"""
import bisect
import csv
import json
import math
import os
import sys

VERSION = "replay-one-2026-11-12v1"
MODE = sys.argv[2] if len(sys.argv) > 2 else "on"
if MODE == "off":
    os.environ["MIRLT_OWN_RESET_RULE"] = "speed"
    os.environ["MIRLT_ELIMINATE"] = "0"
    os.environ["MIRLT_GATE_RECORD"] = "0"
else:
    for k in ("MIRLT_OWN_RESET_RULE", "MIRLT_ELIMINATE", "MIRLT_GATE_RECORD"):
        os.environ.pop(k, None)
    if "-speed" in MODE:                   # e17 with its re-set rule switched back (parts combine: on-speed-noelim)
        os.environ["MIRLT_OWN_RESET_RULE"] = "speed"
REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "tools", "fleet"))
import tracking_layer as TL   # noqa: E402

LAG_S = float(os.environ.get("REPLAY_LAG_S", "0"))   # 0: as the run fed it (own_at(picture time) on its FOLLOW
                      # receive times, freeroam.py); 0.44: layer_replay's CLI default (FOLLOW arrives ~0.44 s late)
STAND_CM = 3.0        # a bridge "stand": every sighting within 2 x the lights' sigma (1.5 cm) of the stand's first
BRIDGE_GAP_S = 2.0    # as layer_replay's "the bridge saw it": a sighting at least every 2 s


def pct(v, q):
    v = sorted(v)
    if not v:
        return None
    return v[min(len(v) - 1, int(q * len(v)))]


def own_interp(rows, ts, t):
    i = bisect.bisect(ts, t)
    if i <= 0:
        return rows[0]
    if i >= len(rows):
        return rows[-1]
    a, b = rows[i - 1], rows[i]
    f = (t - a[0]) / max(1e-6, b[0] - a[0])
    return (t, a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), b[3], b[4])


def main(folder, mode, out_path, watch=True):
    e17 = mode.startswith("on")
    elim, grec = e17 and "-noelim" not in mode, e17 and "-nogate" not in mode
    own = [(float(r["t_recv"]), r["robot"], float(r["x_cm"]), float(r["y_cm"]), float(r["th_deg"]), r["state"])
           for r in csv.DictReader(open(os.path.join(folder, "follow.csv"))) if r["state"] != "nopose"]
    fx = [(float(r["t"]), r["robot"], float(r["x"]), float(r["y"]))
          for r in csv.DictReader(open(os.path.join(folder, "bridge", "track.csv"))) if r["status"] == "seen"]
    # up to the run's end: the bridge can go on after it (7 Nov 14:44: 2 h more). Its start is kept: the live layer
    # also has the bridge's first sightings (L0) before the robots' first FOLLOW
    t_hi = max(q[0] for q in own)
    fx = [q for q in fx if q[0] <= t_hi]
    names = sorted({q[1] for q in own} | {q[1] for q in fx})
    L = TL.TrackingLayer(names, TL.sources_for("lights"), camera_watch=watch, log=lambda k, r: None,
                         eliminate=elim, gate_record=grec, board=(0.0, 0.0, 122.24, 121.92))
    rec_all = {n: [] for n in names}
    orig_record = L.record

    def record(name, t, d_cm, travel_cm=None):         # every correction the layer measures, kept for the drift record
        rec_all.setdefault(name, []).append((round(t, 2), round(float(d_cm), 2),
                                             None if travel_cm is None else round(float(travel_cm), 2)))
        orig_record(name, t, d_cm, travel_cm)
    L.record = record
    own.sort()
    per = {}
    for t, n, x, y, th, st in own:
        per.setdefault(n, []).append((t - LAG_S, x, y, th, st))
    per_ts = {n: [q[0] for q in v] for n, v in per.items()}
    batch = {}
    for t, n, x, y in sorted(fx):
        batch.setdefault(round(t, 3), []).append((n, x, y))
    seeded = set()
    got = {n: [] for n in names}
    wrong_t = {n: [] for n in names}
    taken = []
    seen = {n: [] for n in names}          # (t, x, y) bridge sightings by the bridge's own name
    gap_all = {n: [] for n in names}       # own vs camera at every bridge sighting (cm)
    gap_named = {n: [] for n in names}     # ... at the sightings the layer named for that robot
    for tp in sorted(batch):
        for n in names:
            rows = per.get(n)
            if rows:
                q = own_interp(rows, per_ts[n], tp)
                L.own(n, tp, q[1], q[2], q[3], q[4] == "idle")
        for n, x, y in batch[tp]:
            if n not in seeded:
                L.seed(n, tp, x, y, "start")
                seeded.add(n)
        dets = [TL.Det("motion", tp, x, y) for n, x, y in batch[tp]]
        bn = {id(d): n for d, (n, _, _) in zip(dets, batch[tp])}
        for f in L.feed(tp, dets):
            r = f["robot"]
            got[r].append(tp)
            src_n = next((bn[id(d)] for d in dets if abs(d.x - f["x"]) < 1e-6 and abs(d.y - f["y"]) < 1e-6), None)
            if src_n is None:
                src_n = min(batch[tp], key=lambda q: (q[1] - f["x"]) ** 2 + (q[2] - f["y"]) ** 2)[0]
            if src_n != r:
                wrong_t[r].append(tp)
            if per.get(r):
                q = own_interp(per[r], per_ts[r], tp)
                gap_named[r].append(math.hypot(q[1] - f["x"], q[2] - f["y"]))
            if f.get("how"):
                taken.append(dict(t=round(tp, 2), robot=r, how=f["how"], d_cm=f.get("d_cm"), bridge_name=src_n))
        for n, x, y in batch[tp]:
            seen[n].append((tp, x, y))
            if per.get(n):
                q = own_interp(per[n], per_ts[n], tp)
                gap_all[n].append(math.hypot(q[1] - x, q[2] - y))

    res = dict(version=VERSION, layer=TL.VERSION, mode=mode, folder=os.path.basename(folder.rstrip("/")),
               own_reset_rule=TL.OWN_RESET_RULE, camera_watch=watch, lag_s=LAG_S, eliminate=L.eliminate, gate_record=L.gate_record,
               counts={k: v for k, v in L.counts.items() if v}, robots={})
    for n in names:
        g = sorted(got[n])
        st = seen[n]
        sts = [q[0] for q in st]
        # bridge fix gap: the robot's own picture rate; the e17 "unaccounted" threshold max(1 s, 3 x its fix gap)
        gaps_b = [b - a for a, b in zip(sts, sts[1:]) if b - a < BRIDGE_GAP_S]
        med_gap = pct(gaps_b, 0.5) or 0.25
        G = max(1.0, 3.0 * med_gap)
        # unnamed spells while the bridge saw it (a sighting at least every 2 s all along), between layer namings
        bounds = ([sts[0]] if sts else []) + g + ([sts[-1]] if sts else [])
        bounds = sorted(bounds)
        longest, longest_t, spells = 0.0, None, []
        for a, b in zip(bounds, bounds[1:]):
            if b - a <= G:
                continue
            i, j = bisect.bisect_right(sts, a), bisect.bisect_left(sts, b)
            inside = [a] + sts[i:j] + [b]
            if any(v - u > BRIDGE_GAP_S for u, v in zip(inside, inside[1:])):
                # the bridge lost it too somewhere inside: take the longest stretch it did see
                seg, best = [inside[0]], 0.0
                for u, v in zip(inside, inside[1:]):
                    if v - u > BRIDGE_GAP_S:
                        best = max(best, seg[-1] - seg[0]); seg = [v]
                    else:
                        seg.append(v)
                best = max(best, seg[-1] - seg[0])
                dur = best
            else:
                dur = b - a
            if dur > longest:
                longest, longest_t = dur, a
            # steady: the longest bridge stand inside this spell (all sightings within STAND_CM of its first)
            pts = st[i:j]
            best_st, k0 = 0.0, 0
            for k in range(1, len(pts) + 1):
                if k == len(pts) or math.hypot(pts[k][1] - pts[k0][1], pts[k][2] - pts[k0][2]) > STAND_CM or \
                        pts[k][0] - pts[k - 1][0] > BRIDGE_GAP_S:
                    if k - k0 >= 3:
                        best_st = max(best_st, pts[k - 1][0] - pts[k0][0])
                    k0 = k
            if best_st > G:
                spells.append(dict(t=round(a, 2), unnamed_s=round(b - a, 1), steady_s=round(best_st, 1)))
        # swap episodes: wrong-name fixes, one episode per run of them closer than 2 s
        ep = 0
        prev = None
        for t in wrong_t[n]:
            if prev is None or t - prev > BRIDGE_GAP_S:
                ep += 1
            prev = t
        ga, gn = gap_all[n], gap_named[n]
        res["robots"][n] = dict(
            bridge_fixes=len(st), named=len(g), bridge_med_gap_s=round(med_gap, 3), unacc_s=round(G, 2),
            unnamed_longest_s=round(longest, 1), unnamed_longest_t=round(longest_t, 2) if longest_t else None,
            steady_spells=len(spells), steady_spells_s=round(sum(s["steady_s"] for s in spells), 1),
            steady_longest_s=max([s["steady_s"] for s in spells], default=0.0), spells=spells[:200],
            wrong_name=len(wrong_t[n]), swap_episodes=ep, wrong_t=[round(t, 2) for t in wrong_t[n][:200]],
            gap_all=dict(n=len(ga), p50=pct(ga, .5), p90=pct(ga, .9), p99=pct(ga, .99), max=max(ga, default=None),
                         over25=sum(v > 25 for v in ga)),
            gap_named=dict(n=len(gn), p50=pct(gn, .5), p90=pct(gn, .9), p99=pct(gn, .99), max=max(gn, default=None),
                           over25=sum(v > 25 for v in gn)),
            rec=rec_all.get(n, []), drift=dict(zip(("per_cm", "p90_cm"), L.drift(n))))
    res["taken"] = taken
    res["named_t"] = {n: [round(t, 2) for t in got[n]] for n in names}
    with open(out_path, "w") as fh:
        json.dump(res, fh)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3], (sys.argv[4] if len(sys.argv) > 4 else "1") != "0")
