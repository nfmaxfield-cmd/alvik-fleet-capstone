#!/usr/bin/env python3
"""(e17, item 6) Each robot's drive, turn and curve ratios, camera against its own pose, from a free-roam run's logs
(the e14 calibration pool's quantities, read from a run that was not a calibration). Logs only.

    python3 tools/analysis/drift_ratios.py RUN_FOLDER [RUN_FOLDER ...] [--json OUT]

  drive   a straight drive between two stands (FOLLOW state "drive" only, its own pose moved 8 cm or more, no re-seed
          inside): camera distance / its own distance, each from the mean of the stand before and the stand after
          (so neither clock's delay matters)
  turn    two such drives with only turning on the spot between them: the camera's change of course / its own
          heading's change (|own change| 30 deg or more); also each drive's own heading against the camera's course
          (its heading error at that moment)
  curve   arcfit.csv (the run's own curve fits): k_ratio, length_ratio
Consistent = at least MIN_N of them and the median's 95 % half-width under CI_MAX (the e05 curve rule), and, with more
than one run, the runs' medians within their half-widths. Only a consistent ratio is a candidate correction (discover,
don't pre-map); applying it is the calibration pool's job (calib_pool.py), not this script's.
"""
import argparse
import bisect
import csv
import json
import math
import os
import statistics as st

MIN_N = 15          # as freeroam_core.ARC_FIT_MIN
CI_MAX = 0.03       # as freeroam_core.ARC_FIT_CI
STAND_S = 0.3       # a stand: its own pose still this long (FOLLOW idle)
MIN_CM = 8.0
MIN_TURN = 30.0


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def _load(folder):
    own, cam = {}, {}
    for r in csv.DictReader(open(os.path.join(folder, "follow.csv"))):
        if r["state"] == "nopose":
            continue
        own.setdefault(r["robot"], []).append((float(r["t_recv"]), r["state"], float(r["x_cm"]), float(r["y_cm"]),
                                               float(r["th_deg"])))
    for r in csv.DictReader(open(os.path.join(folder, "bridge", "track.csv"))):
        if r["status"] == "seen":
            cam.setdefault(r["robot"], []).append((float(r["t"]), float(r["x"]), float(r["y"])))
    return own, cam


def _segments(rows):
    """[(kind, i0, i1)] runs of the same state class: stand (idle), drive, turn, other."""
    cls = lambda s: "stand" if s == "idle" else (s if s in ("drive", "turn") else "other")   # noqa: E731
    out, i0 = [], 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or cls(rows[i][1]) != cls(rows[i0][1]):
            out.append((cls(rows[i0][1]), i0, i - 1))
            i0 = i
    return out


def _cam_mean(cam, ts, ta, tb, lag):
    i, j = bisect.bisect_left(ts, ta - lag), bisect.bisect_right(ts, tb - lag)
    pts = cam[i:j]
    if len(pts) < 2:
        return None
    return sum(p[1] for p in pts) / len(pts), sum(p[2] for p in pts) / len(pts)


def _jumped(rows, i0, i1):
    for k in range(i0 + 1, i1 + 1):
        a, b = rows[k - 1], rows[k]
        if math.hypot(b[2] - a[2], b[3] - a[3]) > 3.0 + 15.0 * max(0.0, b[0] - a[0]):
            return True
    return False


def ratios(folder, lag=0.5):
    own, cam = _load(folder)
    res = {}
    for n, rows in own.items():
        c = cam.get(n, [])
        ts = [q[0] for q in c]
        segs = _segments(rows)
        drives = []                # (ratio, own heading, cam course, index in segs)
        for k in range(1, len(segs) - 1):
            kind, i0, i1 = segs[k]
            if kind != "drive":
                continue
            pa, nb = segs[k - 1], segs[k + 1]
            if pa[0] != "stand" or nb[0] != "stand":
                continue
            if rows[pa[2]][0] - rows[pa[1]][0] < STAND_S or rows[nb[2]][0] - rows[nb[1]][0] < STAND_S:
                continue
            if _jumped(rows, pa[1], nb[2]):
                continue
            o0, o1 = rows[pa[2]], rows[nb[1]]
            od = math.hypot(o1[2] - o0[2], o1[3] - o0[3])
            if od < MIN_CM:
                continue
            c0 = _cam_mean(c, ts, rows[pa[1]][0], rows[pa[2]][0], lag)
            c1 = _cam_mean(c, ts, rows[nb[1]][0], rows[nb[2]][0], lag)
            if c0 is None or c1 is None:
                continue
            cd = math.hypot(c1[0] - c0[0], c1[1] - c0[1])
            course = math.degrees(math.atan2(c1[1] - c0[1], c1[0] - c0[0]))
            own_dir = math.degrees(math.atan2(o1[3] - o0[3], o1[2] - o0[2]))
            back = math.cos(math.radians(o0[4] - own_dir)) < 0
            drives.append(dict(ratio=cd / od, own_h=wrap(own_dir), cam_h=wrap(course), k=k, t=o0[0],
                               err=wrap(course - own_dir), back=back, own_cm=od))
        turns = []
        for a, b in zip(drives, drives[1:]):
            between = segs[a["k"] + 1:b["k"]]
            if not between or any(s[0] not in ("stand", "turn") for s in between):
                continue
            ot = wrap(b["own_h"] - a["own_h"])
            if abs(ot) < MIN_TURN:
                continue
            turns.append(wrap(b["cam_h"] - a["cam_h"]) / ot)
        res[n] = dict(drive=[d["ratio"] for d in drives], turn=turns, heading_err=[d["err"] for d in drives])
    p = os.path.join(folder, "arcfit.csv")
    if os.path.exists(p):
        for r in csv.DictReader(open(p)):
            d = res.setdefault(r["robot"], dict(drive=[], turn=[], heading_err=[]))
            d.setdefault("curve_k", []).append(float(r["k_ratio"]))
            d.setdefault("curve_len", []).append(float(r["length_ratio"]))
    return res


def summary(v):
    v = sorted(x for x in v if x is not None and math.isfinite(x))
    if not v:
        return dict(n=0)
    med = st.median(v)
    q1, q3 = v[len(v) // 4], v[(3 * len(v)) // 4]
    half = 1.58 * (q3 - q1) / math.sqrt(len(v))          # the median's ~95 % notch (McGill)
    return dict(n=len(v), median=round(med, 3), q1=round(q1, 3), q3=round(q3, 3), ci_half=round(half, 3),
                consistent=len(v) >= MIN_N and half < CI_MAX)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folders", nargs="+")
    ap.add_argument("--json", default="")
    a = ap.parse_args(argv)
    out = {}
    for f in a.folders:
        for n, d in ratios(f).items():
            for k, v in d.items():
                out.setdefault(n, {}).setdefault(k, {})[os.path.basename(f.rstrip("/"))] = summary(
                    [abs(x) for x in v] if k == "heading_err" else v)
    for n, d in out.items():
        for k, runs in d.items():
            ok = [s for s in runs.values() if s.get("n")]
            agree = len(ok) < 2 or all(abs(s["median"] - ok[0]["median"]) <= s["ci_half"] + ok[0]["ci_half"]
                                       for s in ok[1:])
            runs["_all_runs_agree"] = agree
    print(json.dumps(out, indent=1))
    if a.json:
        with open(a.json, "w") as fh:
            json.dump(out, fh, indent=1)


if __name__ == "__main__":
    main()
