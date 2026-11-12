#!/usr/bin/env python3
"""(e10) The one tracking layer replayed on a real free-roam session's logs: the bridge's fixes (bridge/track.csv) as
unnamed sightings, the robots' own poses (follow.csv, ~0.44 s after the picture), and which robot goes how long without
a fix. Logs only.

    python3 tools/analysis/layer_replay.py <session folder> [--gap-s 3] [--lag-s 0.44] [--watch]

7 Nov home run (20261107-122722-freeroam): with e09's layer Left went 13.2 s without a fix at 12:33:47 (an own-pose
re-set read as motion); with e10's none over 3 s. MIRLT_OWN_RESETS=0 gives the e09 behaviour.
"""
import argparse
import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "fleet"))
import tracking_layer as TL   # noqa: E402


def _own_interp(rows, t):
    """The robot's own pose at t from its FOLLOW rows (as the run's controller, own_at): interpolated."""
    import bisect
    ts = [q[0] for q in rows]
    i = bisect.bisect(ts, t)
    if i <= 0:
        return rows[0]
    if i >= len(rows):
        return rows[-1]
    a, b = rows[i - 1], rows[i]
    f = (t - a[0]) / max(1e-6, b[0] - a[0])
    return (t, a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), b[3], b[4])


def replay(folder, lag_s=0.44, watch=False, gap_s=3.0, e17=True, t_from=None, t_to=None, at_pictures=False):
    """(e17) e17=False: the layer as before e17 (no elimination, the fixed gate). Also returns, per robot, the
    longest stretch with the bridge seeing it but the layer naming nothing, how many fixes the layer named under
    another robot's bridge name (the bridge's own names as a check: they can be wrong too), and the e17 events."""
    own = [(float(r["t_recv"]), r["robot"], float(r["x_cm"]), float(r["y_cm"]), float(r["th_deg"]), r["state"])
           for r in csv.DictReader(open(os.path.join(folder, "follow.csv"))) if r["state"] != "nopose"]
    fx = [(float(r["t"]), r["robot"], float(r["x"]), float(r["y"]))
          for r in csv.DictReader(open(os.path.join(folder, "bridge", "track.csv"))) if r["status"] == "seen"]
    names = sorted({q[1] for q in own} | {q[1] for q in fx})
    logs = []
    L = TL.TrackingLayer(names, TL.sources_for("lights"), camera_watch=watch, log=lambda k, r: logs.append(r),
                         eliminate=e17, gate_record=e17, board=(0.0, 0.0, 122.24, 121.92))
    own.sort()
    per = {}
    for t, n, x, y, th, st in own:
        per.setdefault(n, []).append((t - lag_s, x, y, th, st))
    batch = {}
    for t, n, x, y in sorted(fx):
        if (t_from is not None and t < t_from) or (t_to is not None and t > t_to):
            continue
        batch.setdefault(round(t, 3), []).append((n, x, y))
    seeded, oi, got = set(), 0, {n: [] for n in names}
    wrong, taken, seen_by_bridge = {}, [], {n: [] for n in names}
    for tp in sorted(batch):
        if at_pictures:
            # (e17) as the run feeds it: each robot's own pose interpolated at the picture's time, once a picture
            # (the home Pi handled 4-5 pictures a second, so a re-seed's step can look like a drive)
            for n in names:
                rows = per.get(n)
                if rows:
                    q = _own_interp(rows, tp)
                    L.own(n, tp, q[1], q[2], q[3], q[4] == "idle")
        while not at_pictures and oi < len(own) and own[oi][0] <= tp + lag_s:
            t, n, x, y, th, st = own[oi]
            L.own(n, t - lag_s, x, y, th, st == "idle")
            oi += 1
        for n, x, y in batch[tp]:
            if n not in seeded:
                L.seed(n, tp, x, y, "start")
                seeded.add(n)
        dets = [TL.Det("motion", tp, x, y) for n, x, y in batch[tp]]
        bn = {id(d): n for d, (n, _, _) in zip(dets, batch[tp])}
        for f in L.feed(tp, dets):
            got[f["robot"]].append(tp)
            src_n = next((bn[id(d)] for d in dets if abs(d.x - f["x"]) < 1e-6 and abs(d.y - f["y"]) < 1e-6), None)
            if src_n is None:            # an averaged spot (elimination): the nearest sighting's bridge name
                src_n = min(batch[tp], key=lambda q: (q[1] - f["x"]) ** 2 + (q[2] - f["y"]) ** 2)[0]
            if src_n != f["robot"]:
                wrong[f["robot"]] = wrong.get(f["robot"], 0) + 1
            if f.get("how"):
                taken.append(dict(t=round(tp, 2), robot=f["robot"], how=f["how"], d_cm=f.get("d_cm"),
                                  bridge_name=src_n))
        for n, x, y in batch[tp]:
            seen_by_bridge[n].append(tp)
    gaps = [dict(robot=n, t=round(a, 2), gap_s=round(b - a, 1)) for n in names for a, b in zip(got[n], got[n][1:])
            if b - a > gap_s]
    # the longest stretch per robot the bridge saw it (a fix within 2 s all along) and the layer named nothing
    blind = {}
    for n in names:
        g = sorted(got[n])
        best = (0.0, None)
        for a, b in zip([seen_by_bridge[n][0]] + g if seen_by_bridge[n] else g, g + ([seen_by_bridge[n][-1]] if
                                                                                      seen_by_bridge[n] else [])):
            if b - a > best[0]:
                best = (b - a, a)
        blind[n] = dict(longest_s=round(best[0], 1), from_t=round(best[1], 2) if best[1] else None)
    return dict(layer=TL.VERSION, e17=e17, own_resets=TL.OWN_RESETS, gaps=gaps,
                counts={k: v for k, v in L.counts.items() if v}, fixes={n: len(v) for n, v in got.items()},
                bridge_fixes={n: len(v) for n, v in seen_by_bridge.items()}, wrong_name=wrong, unnamed_longest=blind,
                taken=taken, drift=L.summary().get("drift"),
                events=[r for r in logs if r.get("what") in ("new object", "own pose re-set", "camera not trusted") or
                        "elimination" in str(r.get("what")) or "named by" in str(r.get("what"))][:80])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder")
    ap.add_argument("--gap-s", type=float, default=3.0)
    ap.add_argument("--lag-s", type=float, default=0.44)
    ap.add_argument("--watch", action="store_true", help="the camera watch on")
    ap.add_argument("--before-e17", action="store_true", help="the layer as before e17 (no elimination, fixed gate)")
    ap.add_argument("--at-pictures", action="store_true", help="own poses fed once a picture, as the run does")
    a = ap.parse_args(argv)
    print(json.dumps(replay(a.folder, a.lag_s, a.watch, a.gap_s, e17=not a.before_e17, at_pictures=a.at_pictures),
                     indent=1))


if __name__ == "__main__":
    main()
