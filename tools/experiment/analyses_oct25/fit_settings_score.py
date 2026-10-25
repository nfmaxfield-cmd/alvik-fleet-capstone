"""DOE rows "gyro drift band" (e02_fit DRIFT_BAND_DPS), "tracker jump cut" (e02_fit JUMP_CM) and "wheel fit accepted"
(e02_fit MAX_RMS_CM / --fit-max-rms-cm), scored by the end error each setting's calibration would have given.

Score: for a run R (U or C) of a robot, its calibration is fitted from the robot's U-run stretches in the same session
without R, with the setting under test. The run's RAW rows are then re-integrated the firmware's way (pose.h tick,
checked: reproduces the robot's own end pose to 0.02 cm on every run without camera fixes) with that calibration, and
the robot's own displacement over the route (end - start) is compared with the camera's (median position in the 0.5 s
before the route starts and after it ends). That difference is the end error the run would have had, had it used that
calibration, apart from where it started (which is the same for every setting).

Two training schemes:
  - leave one run out (LORO): every other U run of the robot in the session (most data; for the band and the cut);
  - as the runner would (SEQ): only the U runs before R (for the wheel gate: what was refused at that moment).

A part the fit refuses falls back to the prior (e02_fit.PRIOR, drift from the naming check), as the runner's
move_corrections does before any accepted fit.

Usage: python3 fit_settings_score.py   (writes out/fit_settings_*.csv, prints the tables)
"""
import itertools
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oct25_common as C  # noqa: E402
from oct25_common import FIT  # noqa: E402

CUTS = (None, 2.0, 3.0, 4.0)
BANDS = (None, 0.0, 0.01, 0.02, 0.05, 0.1)
GATES = (1.5, 2.0, 2.5, 3.0, 99.0)
BASE = dict(cut=3.0, band=0.05, gate=1.5)


def cam_at(track, t0, t1):
    pts = [(x, y) for t, x, y, _ in track if t0 <= t <= t1]
    if len(pts) < 2:
        return None
    return float(np.median([p[0] for p in pts])), float(np.median([p[1] for p in pts]))


def test_runs(name):
    """Runs to score: U and C runs with no camera fixes inside (fixes_used 0), with a camera start and end."""
    tr = C.track(name)
    raw = C.raw_by_run(name)
    ys = C.yaw_signs(name)
    out = []
    for r in C.run_table(name):
        if r["cond"] not in ("U", "C") or int(float(r["rec"]["fixes_used"] or 0)) != 0:
            continue
        rr = raw.get(r["run"]) or []
        off = FIT.clock_offset(rr)
        if off is None:
            continue
        ts = num_ms(r["rec"]["ms_start"]) / 1000.0 + off
        te = num_ms(r["rec"]["ms_end"]) / 1000.0 + off
        c0 = cam_at(tr.get(r["robot"], []), ts - 0.5, ts)
        c1 = cam_at(tr.get(r["robot"], []), te + 0.2, te + 0.7)
        if c0 is None or c1 is None:
            continue
        r.update(raw=rr, ys=ys.get(r["run"], 1), cam0=c0, cam1=c1, off=off,
                 still=still_points(rr, num_ms(r["rec"]["ms_start"]), num_ms(r["rec"]["ms_end"])))
        out.append(r)
    return out


def num_ms(v):
    return float(v)


def still_points(raw, ms0, ms1, min_s=0.6):
    """Middles (robot ms) of the moments inside the route where both wheels stood still at least min_s (the leg
    pauses: after every leg the robot stands 1.0-2.5 s)."""
    rr = sorted((q for q in raw if q.get("ms") is not None and ms0 <= q["ms"] <= ms1), key=lambda q: q["ms"])
    out, start = [], None
    for q in rr:
        still = abs(q.get("wl_rpm") or 0) < 0.5 and abs(q.get("wr_rpm") or 0) < 0.5
        if still and start is None:
            start = q["ms"]
        if not still and start is not None:
            if q["ms"] - start >= 1000 * min_s:
                out.append(0.5 * (start + q["ms"]))
            start = None
    return out


def errors(r, vals, track):
    """(end, far, path) error in cm for run r under calibration vals: the robot's own displacement from the route's
    start against the camera's, at the end, at the far point, and as the rms over every leg pause along the route."""
    trail = C.reintegrate(r["raw"], r["rec"], r["ys"], vals["gyro_scale"], vals["gyro_drift_dps"], vals["wheel_scale"],
                          path=True)
    if trail is None:
        return None, None, None, None
    ms = np.array([q[0] for q in trail])
    sx, sy = trail[0][1], trail[0][2]

    def at(m):
        i = int(np.clip(np.searchsorted(ms, m), 0, len(ms) - 1))
        return trail[i][1] - sx, trail[i][2] - sy

    def err(bel, cam):
        if cam is None:
            return None
        return math.hypot(cam[0] - r["cam0"][0] - bel[0], cam[1] - r["cam0"][1] - bel[1])

    end = err((trail[-1][1] - sx, trail[-1][2] - sy), r["cam1"])
    far = None
    mf = float(r["rec"]["ms_far"] or 0)
    if mf:
        tf = mf / 1000.0 + r["off"]
        far = err(at(mf), cam_at(track, tf + 0.2, tf + 0.7))
    es, pairs = [], []
    for m in r["still"]:
        tt = m / 1000.0 + r["off"]
        cam = cam_at(track, tt - 0.25, tt + 0.25)
        e = err(at(m), cam)
        if e is not None:
            es.append(e)
            pairs.append((at(m), (cam[0] - r["cam0"][0], cam[1] - r["cam0"][1])))
    path = math.sqrt(sum(e * e for e in es) / len(es)) if es else None
    # the same after the best rotation about the start (takes out a wrong start heading, which no calibration
    # changes): what is left is what the calibration can explain
    rot = None
    if len(pairs) >= 3:
        sc = sum(b[0] * c[0] + b[1] * c[1] for b, c in pairs)
        sx_ = sum(b[0] * c[1] - b[1] * c[0] for b, c in pairs)
        a = math.atan2(sx_, sc)
        ca, sa = math.cos(a), math.sin(a)
        rot = math.sqrt(sum((c[0] - (ca * b[0] - sa * b[1])) ** 2 + (c[1] - (sa * b[0] + ca * b[1])) ** 2
                            for b, c in pairs) / len(pairs))
    return end, far, path, rot


def values_of(res, prior):
    """What the runner would use after this fit: the accepted parts, the prior for the rest."""
    v = dict(prior)
    f = res["fitted"]
    if res["gyro_ok"]:
        v.update(gyro_scale=f["gyro_scale"], gyro_drift_dps=f["gyro_drift_dps"])
    if res["wheel_ok"]:
        v.update(wheel_scale=f["wheel_scale"])
    return v


def main():
    os.makedirs(C.OUT, exist_ok=True)
    rows_out = []
    for name in C.CLASSIC:
        drift = C.measured_drift(name)
        runs = test_runs(name)
        tr = C.track(name)
        by_cut = {cut: C.stretches_by_run(name, jump_cm=cut) for cut in CUTS}
        for r in runs:
            robot = r["robot"]
            prior = dict(FIT.PRIOR)
            if drift.get(robot) is not None:
                prior["gyro_drift_dps"] = drift[robot]
            trk = tr.get(robot, [])
            e_used = errors(r, r["used"], trk)
            e_prior = errors(r, prior, trk)
            for cut in CUTS:
                per_run = by_cut[cut].get(robot, [])
                for scheme in ("LORO", "SEQ"):
                    if scheme == "LORO":
                        train = [o for order, run, obs in per_run if run != r["run"] for o in obs]
                    else:
                        train = [o for order, run, obs in per_run if order < r["order"] for o in obs]
                    for band, gate in itertools.product(BANDS, GATES):
                        if scheme == "LORO" and gate not in (1.5, 99.0):
                            continue
                        if not train:
                            res = None
                        else:
                            res = C.fit_with(train, drift.get(robot), band=band, max_rms=gate)
                        vals = values_of(res, prior) if res else prior
                        e = errors(r, vals, trk)
                        f = (res or {}).get("fitted") or {}
                        rows_out.append([name, robot, r["order"], r["cond"], r["route"], scheme, cut, band, gate,
                                         len(train), int(bool(res and res["gyro_ok"])),
                                         int(bool(res and res["wheel_ok"])),
                                         _r(f.get("rms_cm")), _r(f.get("gyro_drift_dps"), 4), _r(vals["gyro_scale"], 4),
                                         _r(vals["gyro_drift_dps"], 4), _r(vals["wheel_scale"], 4),
                                         *[_r(v) for v in e], *[_r(v) for v in e_used], *[_r(v) for v in e_prior],
                                         len(r["still"])])
        print("%s: %d runs scored" % (name, len(runs)), flush=True)
    hdr = ["session", "robot", "order", "cond", "route", "scheme", "cut", "band", "gate", "n_train", "gyro_ok",
           "wheel_ok", "fit_rms_cm", "fit_drift", "gyro_scale", "gyro_drift_dps", "wheel_scale", "end_err_cm",
           "far_err_cm", "path_err_cm", "rot_err_cm", "end_as_run", "far_as_run", "path_as_run", "rot_as_run",
           "end_prior", "far_prior", "path_prior", "rot_prior", "n_still"]
    C.write_csv(os.path.join(C.OUT, "fit_settings_scores.csv"), hdr, rows_out)
    summarise(rows_out)


def _r(v, n=3):
    return "" if v is None else round(v, n)


METRICS = dict(end=17, far=18, path=19, rot=20)


def summarise(rows_out):
    import collections
    from scipy.stats import wilcoxon
    idx = collections.defaultdict(dict)
    for row in rows_out:
        (name, robot, order, cond, route, scheme, cut, band, gate) = row[:9]
        idx[(scheme, cut, band, gate)][(name, robot, order)] = row

    def summary(key, ref, m):
        a, b = idx.get(key, {}), idx.get(ref, {})
        ks = sorted(k for k in set(a) & set(b) if a[k][m] != "" and b[k][m] != "")
        if not ks:
            return None
        ea = np.array([a[k][m] for k in ks], float)
        eb = np.array([b[k][m] for k in ks], float)
        d = ea - eb
        p = wilcoxon(ea, eb).pvalue if np.any(d != 0) else 1.0
        return len(ks), np.median(ea), np.percentile(ea, 90), np.mean(ea), np.median(d), p

    fmt = "  %-26s n %3d  median %5.2f  p90 %5.2f  mean %5.2f  median diff %+5.2f  p %.3f"
    for mname, m in METRICS.items():
        print("\n[%s error] LORO, gate 1.5 cm, by jump cut and drift band; paired against cut 3 / band 0.05" % mname)
        for cut, band in itertools.product(CUTS, BANDS):
            s_ = summary(("LORO", cut, band, 1.5), ("LORO", 3.0, 0.05, 1.5), m)
            if s_:
                print(fmt % ("cut %s band %s" % (cut, band), *s_))
        for cut in (3.0, None):
            print("[%s error] SEQ (runner-faithful), cut %s, band 0.05, by wheel gate; paired against 1.5" % (mname, cut))
            for gate in GATES:
                s_ = summary(("SEQ", cut, 0.05, gate), ("SEQ", cut, 0.05, 1.5), m)
                if s_:
                    print(fmt % ("gate %s" % gate, *s_))
    # as run and prior, for scale
    for mname, m in METRICS.items():
        base = [r for r in rows_out if r[5] == "LORO" and r[6] == 3.0 and r[7] == 0.05 and r[8] == 1.5]
        for label, sel in (("all", lambda r: True), ("C runs", lambda r: r[3] == "C"), ("U runs", lambda r: r[3] == "U")):
            q = [r for r in base if sel(r) and r[m] != "" and r[m + 8] != ""]
            if not q:
                continue
            fa = np.array([r[m] for r in q], float)
            pa = np.array([r[m + 8] for r in q], float)
            ra = [r[m + 4] for r in q if r[m + 4] != ""]
            from scipy.stats import wilcoxon
            p_ = wilcoxon(fa, pa).pvalue if np.any(fa != pa) else 1.0
            print("[%s, %s] n %d: fitted (cut 3, band 0.05, LORO) median %.2f / prior values %.2f (paired p %.3f) / "
                  "as run %.2f" % (mname, label, len(q), np.median(fa), np.median(pa), p_, np.median(ra)))


if __name__ == "__main__":
    main()
