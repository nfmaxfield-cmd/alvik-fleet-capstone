#!/usr/bin/env python3
"""Each home robot's odometry drift record, run by run and date by date (for e18's blind-drive allowance). Logs only.

    python3 drift.py RUNS_ROOT DOE_ROOT REPLAY_OUT OUT_DIR

Measures, each per robot per run:
  stretch blind stretches (see stretches()): own filter took no fix and no re-seed, >= 10 cm of own path, any
          motion (curves, drives, turns): cm of endpoint error per cm of own path. The main measure: free roam
          is almost all curves, so stand-drive-stand straight drives (below) are too few.
  blind   straight drives between two stands (FOLLOW idle -> drive -> idle; own pose moved >= 8 cm) during which the
          robot's own filter took NO camera fix (fixes_used unchanged from the stand before to the stand after) and no
          re-seed jump: its own wheels and gyro alone. Camera position = mean of the bridge's sightings over each stand
          (shifted 0.5 s, so neither clock's delay matters). Per drive:
            err_per_cm  |camera displacement - own displacement| / own distance   (cm of error per cm driven)
            scale       camera distance / own distance
            course_deg  camera course - own course
          Adapted from tools/analysis/drift_ratios.py (3f37317) with the no-fix filter added: with fixes inside, the
          own pose is pulled toward the camera and the error is understated (11 Nov 4 h run: 17 % of Left's drive
          records took a fix).
  fused   the same drives without the no-fix filter (what drift_ratios.py measures), for comparison.
  record  e17's own drift record: every correction the tracking layer measured in the e17-ON replay (an own-pose
          re-set or an elimination), its size / the cm its own pose drove since the camera last had it (>= 5 cm).
  turn90  DOE loops (27 Oct - 2 Nov): each camera-measured 90 deg turn check, camera angle / own (gyro) angle and
          the error in deg per turn (turns.csv).
  curve   arcfit.csv (runs since 6 Nov): k_ratio = camera curvature / asked.

Changelog
  2026-11-12 v1  first version (replay-all-nov12)
"""
import bisect
import csv
import glob
import json
import math
import os
import statistics as st
import sys

VERSION = "drift-2026-11-12v1"
STAND_S = 0.3
MIN_CM = 8.0
LAG = 0.5


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def segments(rows):
    cls = lambda s: "stand" if s == "idle" else (s if s in ("drive", "turn") else "other")   # noqa: E731
    out, i0 = [], 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or cls(rows[i][1]) != cls(rows[i0][1]):
            out.append((cls(rows[i0][1]), i0, i - 1))
            i0 = i
    return out


def cam_mean(cam, ts, ta, tb):
    i, j = bisect.bisect_left(ts, ta - LAG), bisect.bisect_right(ts, tb - LAG)
    pts = cam[i:j]
    if len(pts) < 2:
        return None
    return sum(p[1] for p in pts) / len(pts), sum(p[2] for p in pts) / len(pts)


def jumped(rows, i0, i1):
    for k in range(i0 + 1, i1 + 1):
        a, b = rows[k - 1], rows[k]
        if math.hypot(b[2] - a[2], b[3] - a[3]) > 3.0 + 15.0 * max(0.0, b[0] - a[0]):
            return True
    return False


def drives(folder):
    own, cam = {}, {}
    for r in csv.DictReader(open(os.path.join(folder, "follow.csv"))):
        if r["state"] == "nopose":
            continue
        own.setdefault(r["robot"], []).append((float(r["t_recv"]), r["state"], float(r["x_cm"]), float(r["y_cm"]),
                                               float(r["th_deg"]), int(r["fixes_used"])))
    for r in csv.DictReader(open(os.path.join(folder, "bridge", "track.csv"))):
        if r["status"] == "seen":
            cam.setdefault(r["robot"], []).append((float(r["t"]), float(r["x"]), float(r["y"])))
    out = {}
    for n, rows in own.items():
        c = sorted(cam.get(n, []))
        ts = [q[0] for q in c]
        segs = segments(rows)
        for k in range(1, len(segs) - 1):
            kind, i0, i1 = segs[k]
            pa, nb = segs[k - 1], segs[k + 1]
            if kind != "drive" or pa[0] != "stand" or nb[0] != "stand":
                continue
            if rows[pa[2]][0] - rows[pa[1]][0] < STAND_S or rows[nb[2]][0] - rows[nb[1]][0] < STAND_S:
                continue
            if jumped(rows, pa[1], nb[2]):
                continue
            o0, o1 = rows[pa[2]], rows[nb[1]]
            od = math.hypot(o1[2] - o0[2], o1[3] - o0[3])
            if od < MIN_CM:
                continue
            c0 = cam_mean(c, ts, rows[pa[1]][0], rows[pa[2]][0])
            c1 = cam_mean(c, ts, rows[nb[1]][0], rows[nb[2]][0])
            if c0 is None or c1 is None:
                continue
            cdx, cdy = c1[0] - c0[0], c1[1] - c0[1]
            odx, ody = o1[2] - o0[2], o1[3] - o0[3]
            cd = math.hypot(cdx, cdy)
            out.setdefault(n, []).append(dict(
                t=round(o0[0], 2), own_cm=round(od, 2), blind=o1[5] == o0[5],
                err_per_cm=math.hypot(cdx - odx, cdy - ody) / od, scale=cd / od,
                course_deg=wrap(math.degrees(math.atan2(cdy, cdx)) - math.degrees(math.atan2(ody, odx)))))
    return out


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * len(v)))] if v else None


def summ(v):
    v = [x for x in v if x is not None and math.isfinite(x)]
    if not v:
        return dict(n=0)
    v = sorted(v)
    q1, q3 = q(v, .25), q(v, .75)
    return dict(n=len(v), median=round(st.median(v), 4), q1=round(q1, 4), q3=round(q3, 4), p90=round(q(v, .9), 4),
                ci_half=round(1.58 * (q3 - q1) / math.sqrt(len(v)), 4))

STRETCH_MIN_CM = 10.0   # CHOSEN: a blind stretch counts from 10 cm of own path (camera noise ~2 cm is then < 20 %)
STRETCH_MAX_CM = 100.0  # CHOSEN: ... up to 1 m. 5 Nov 15:50 has "blind" stretches of 9-49 m ending 5-16 cm off: small
                        # re-sets under the jump test (3 cm + 15 cm/s) cannot be ruled out there, so they are left out
CAM_NEAR_S = 0.3        # a camera sighting within this of each end (the home bridge: 4-5 pictures a second)
LAG_FOLLOW = 0.44       # as layer_replay: FOLLOW arrives ~0.44 s after the picture


def stretches(folder):
    """Blind stretches: maximal runs of FOLLOW records with the robot's own filter taking no camera fix (fixes_used
    unchanged) and no own-pose jump (a re-seed), its own path >= STRETCH_MIN_CM. The camera position at each end is
    interpolated between the bridge's sightings (FOLLOW times shifted back LAG_FOLLOW). err_per_cm = |camera
    displacement - own displacement| / own path. This is what a blind drive accumulates on its wheels and gyro alone,
    in the moves free roam really makes (mostly curves)."""
    own, cam = {}, {}
    for r in csv.DictReader(open(os.path.join(folder, "follow.csv"))):
        if r["state"] == "nopose":
            continue
        own.setdefault(r["robot"], []).append((float(r["t_recv"]) - LAG_FOLLOW, r["state"], float(r["x_cm"]),
                                               float(r["y_cm"]), float(r["th_deg"]), int(r["fixes_used"])))
    for r in csv.DictReader(open(os.path.join(folder, "bridge", "track.csv"))):
        if r["status"] == "seen":
            cam.setdefault(r["robot"], []).append((float(r["t"]), float(r["x"]), float(r["y"])))

    def cam_at(c, ts, t):
        i = bisect.bisect_left(ts, t)
        if i <= 0 or i >= len(c) or t - c[i - 1][0] > CAM_NEAR_S or c[i][0] - t > CAM_NEAR_S:
            return None
        a, b = c[i - 1], c[i]
        f = (t - a[0]) / max(1e-6, b[0] - a[0])
        return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2])

    out = {}
    for n, rows in own.items():
        c = sorted(cam.get(n, []))
        ts = [q[0] for q in c]
        i0 = 0
        for i in range(1, len(rows) + 1):
            brk = i == len(rows) or rows[i][5] != rows[i - 1][5] or \
                math.hypot(rows[i][2] - rows[i - 1][2], rows[i][3] - rows[i - 1][3]) > \
                3.0 + 15.0 * max(0.0, rows[i][0] - rows[i - 1][0])
            if not brk:
                continue
            seg = rows[i0:i]
            i0 = i
            if len(seg) < 3:
                continue
            path = sum(math.hypot(b[2] - a[2], b[3] - a[3]) for a, b in zip(seg, seg[1:]))
            if path < STRETCH_MIN_CM or path > STRETCH_MAX_CM:
                continue
            c0, c1 = cam_at(c, ts, seg[0][0]), cam_at(c, ts, seg[-1][0])
            if c0 is None or c1 is None:
                continue
            odx, ody = seg[-1][2] - seg[0][2], seg[-1][3] - seg[0][3]
            cdx, cdy = c1[0] - c0[0], c1[1] - c0[1]
            turned = sum(abs(wrap(b[4] - a[4])) for a, b in zip(seg, seg[1:]))
            out.setdefault(n, []).append(dict(t=round(seg[0][0], 2), path_cm=round(path, 2), turned_deg=round(turned, 1),
                                              err_cm=math.hypot(cdx - odx, cdy - ody),
                                              err_per_cm=math.hypot(cdx - odx, cdy - ody) / path))
    return out


def main(runs_root, doe_root, replay_out, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    rows = []          # long table: run, date, robot, measure, value
    for f in sorted(glob.glob(os.path.join(runs_root, "*-freeroam"))):
        run = os.path.basename(f)
        if not os.path.exists(os.path.join(f, "follow.csv")):
            continue
        date = run[:8]
        for n, ds in drives(f).items():
            for d in ds:
                for m in ("err_per_cm", "scale", "course_deg"):
                    rows.append((run, date, n, ("blind_" if d["blind"] else "fixed_") + m, d[m], d["own_cm"]))
        for n, ds in stretches(f).items():
            for d in ds:
                rows.append((run, date, n, "stretch_err_per_cm", d["err_per_cm"], d["path_cm"]))
                rows.append((run, date, n, "stretch_err_cm", d["err_cm"], d["path_cm"]))
        p = os.path.join(f, "arcfit.csv")
        if os.path.exists(p):
            for r in csv.DictReader(open(p)):
                try:
                    rows.append((run, date, r["robot"], "curve_k", float(r["k_ratio"]), float(r["L"] or 0)))
                except ValueError:          # a fit with no ratio (blank)
                    pass
        rp = os.path.join(replay_out, f"{run}-on-w1.json")
        if os.path.exists(rp):
            j = json.load(open(rp))
            for n, r in j["robots"].items():
                for t, d, tr in r["rec"]:
                    if tr is not None and tr >= 5.0:
                        rows.append((run, date, n, "record_per_cm", d / tr, tr))
    for f in sorted(glob.glob(os.path.join(doe_root, "*", "*-doe", "turns.csv"))):
        run = os.path.basename(os.path.dirname(f))
        for r in csv.DictReader(open(f)):
            try:
                cdeg, odeg = float(r["camera_deg"]), float(r["own_deg"])
            except (TypeError, ValueError):
                continue
            if abs(odeg) < 30:
                continue
            rows.append((run, run[:8], r["robot"], "turn90_err_deg", float(r["error_deg"]), abs(odeg)))
            rows.append((run, run[:8], r["robot"], "turn90_ratio", cdeg / odeg, abs(odeg)))
    with open(os.path.join(out_dir, "drift_long.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["run", "date", "robot", "measure", "value", "own_cm_or_deg"])
        for r in rows:
            w.writerow([r[0], r[1], r[2], r[3], round(r[4], 5), round(r[5], 2)])
    # per run per robot per measure
    groups = {}
    for run, date, n, m, v, _ in rows:
        groups.setdefault((m, n, run), []).append(abs(v) if m.endswith("course_deg") or m.endswith("err_deg") else v)
    summary = {}
    for (m, n, run), v in sorted(groups.items()):
        summary.setdefault(m, {}).setdefault(n, {})[run] = summ(v)
    # between runs: Kruskal-Wallis across runs with >= 5 values (robot by robot) and the pooled median
    from scipy.stats import kruskal
    tests = {}
    for m, by in summary.items():
        for n, runs in by.items():
            vals = [groups[(m, n, r)] for r in runs if len(groups[(m, n, r)]) >= 5]
            pooled = [x for r in runs for x in groups[(m, n, r)]]
            res = dict(runs_with_5=len(vals), pooled=summ(pooled))
            if len(vals) >= 2:
                h, p = kruskal(*vals)
                res.update(H=round(float(h), 2), p=float("%.3g" % p))
            tests.setdefault(m, {})[n] = res
    json.dump(dict(version=VERSION, per_run=summary, between_runs=tests), open(os.path.join(out_dir, "drift.json"),
                                                                                  "w"), indent=1)
    print(json.dumps(tests, indent=1))


if __name__ == "__main__":
    main(*sys.argv[1:5])
