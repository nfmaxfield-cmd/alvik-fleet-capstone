#!/usr/bin/env python3
"""Calibration evidence pooled across runs (build e14; Nick 10 Nov 16:40: "yes").

The school, 10 Nov: AL-EB54 had about ten partial calibrations. Each began again from the defaults 1 / 1 / 0, and
values reach the robot only when a run finishes its fit, so every turn, roll, still period and tag offset already
measured was thrown away.

Now every measured leg of a robot (by WiFi name) on a site is kept, finished and partial runs alike, in
<site conf dir>/calib_pool.json, as the RAW quantities the firmware's model needs (so legs driven with different values
in force pool together):

  turn   (raw gyro degrees, seconds, camera degrees)        camera = g x raw - d x t
  still  (raw gyro degrees, seconds)                        0      = g x raw - d x t
  roll   (counted wheel cm, camera cm)                      camera = w x counted + coast
  tag offset from the turning point (ahead, left, its rms)

Each leg is weighted by age (half weight every AGE_HALF_DAYS) and by the battery at the time (full from
BATTERY_FULL_W_PCT up, falling below it; logged per leg); the fit is calib_run.fit_values' own least squares with these
weights, so each value carries its pooled +-. A new run starts from the pooled values, measures only what is still
uncertain, and a run that ends early still adds its legs. The pooled values are saved on the robot once their +- is
below what ONE finished run would give at the pool's own measured noise (full_run_se: derived from phase A's design,
not picked).

Switch: MIRLT_CALIB_POOL=0 (each run on its own, as before e14).

    python3 tools/fleet/calib_pool.py show AL-EB54 [--dir ~/.config/mirlt/school-conference]
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

VERSION = "calib-pool-2026-11-10e14b"   # e14b: each run weighted by its own noise, outlier legs dropped (school 17:02)
FILE = "calib_pool.json"
AGE_HALF_DAYS = 7.0      # CHOSEN: a leg counts half after a week (tyres wear, a gyro warms differently; the school's
                         # calibrations a day apart agree to about 0.003 in wheel_scale). DOE row (e14)
BATTERY_FULL_W_PCT = 40.0   # CHOSEN (the low-battery calibration note's line): legs at or above this count fully;
                         # below, weight battery / 40 (a leg at 20 % counts half). DOE row: calibration against battery
MIN_TURN_DEG = 30.0      # calib_run's own: only turns of 30 deg and more fit the gyro. Not chosen here
MIN_ROLL_CM = 15.0       # calib_run's own: only drives of 15 cm and more fit the wheels. Not chosen here
MAX_ROWS = 4000          # per robot, the oldest dropped beyond this. Not tuned


def enabled():
    return os.environ.get("MIRLT_CALIB_POOL", "1") != "0"


def path(folder):
    return os.path.join(folder, FILE) if folder else ""


def load(folder):
    p = path(folder)
    if p and os.path.exists(p):
        try:
            with open(p) as f:
                d = json.load(f)
            if isinstance(d.get("robots"), dict):
                return d
        except (OSError, ValueError):
            pass
    return {"version": VERSION, "robots": {}}


def save(folder, d):
    p = path(folder)
    if not p:
        return ""
    os.makedirs(folder, exist_ok=True)
    with open(p + ".tmp", "w") as f:
        json.dump(d, f)
    os.replace(p + ".tmp", p)
    return p


def battery_w(b):
    if b is None:
        return 1.0
    return min(1.0, max(0.1, float(b) / BATTERY_FULL_W_PCT))


def age_w(t, now=None):
    now = time.time() if now is None else now
    return 0.5 ** (max(0.0, now - float(t)) / 86400.0 / AGE_HALF_DAYS)


def raw_rows(rows, values_at, battery_at, session, unwrap):
    """calib_run's rows -> pooled rows. values_at(t) -> the robot's values in force then; battery_at(t) -> % or None;
    unwrap(row) -> (belief turn, dt, camera turn). Interrupted, timed-out and unmeasured legs are left out."""
    out = []
    for r in rows:
        if r.get("interrupted") or r.get("cam0") is None or r.get("cam1") is None:
            continue
        t = float(r.get("t0") or time.time())
        v = values_at(t)
        gs, gd, ws = float(v["gyro_scale"]), float(v["gyro_drift_dps"]), float(v["wheel_scale"])
        b = battery_at(t)
        base = {"t": round(t, 1), "battery": b, "session": session, "phase": r.get("phase")}
        if r.get("kind") == "turn" and r.get("result") == "done" and abs(float(r["value"])) >= MIN_TURN_DEG:
            bel, dt_, cam = unwrap(r)
            out.append(dict(base, kind="turn", raw=(bel + gd * dt_) / gs, dt=dt_, cam=cam))
        elif r.get("kind") == "still":
            out.append(dict(base, kind="still", raw=(float(r["bel_turn"]) + gd * float(r["dt"])) / gs,
                            dt=float(r["dt"]), cam=0.0))
        elif r.get("kind") == "go" and r.get("result") == "done" and float(r["value"]) >= MIN_ROLL_CM:
            out.append(dict(base, kind="roll", counted=float(r["bel_len"]) / ws, cam=float(r["cam_len"])))
    return out


def add(folder, wifi, rows, offset=None, session="", when=None):
    """Keep a run's legs (and its tag offset) for this robot. Returns words."""
    if not wifi or (not rows and not offset):
        return "nothing to keep"
    d = load(folder)
    e = d["robots"].setdefault(wifi, {"rows": [], "offsets": [], "sessions": []})
    e["rows"].extend(rows)
    del e["rows"][:-MAX_ROWS]
    if offset:
        e["offsets"].append({"t": when or time.time(), "ahead": offset[0], "left": offset[1], "rms": offset[2],
                             "session": session})
    if session and session not in e["sessions"]:
        e["sessions"].append(session)
    save(folder, d)
    return "%d legs kept (%d in the pool for %s from %d runs)" % (len(rows), len(e["rows"]), wifi, len(e["sessions"]))


def wls(X, y, w, floor):
    """Weighted least squares (w: each leg's relative weight): (beta, cov, sigma of a weight-1 leg). floor: the
    smallest residual sigma believed (calib_run's own floors)."""
    w = np.asarray(w, float)
    Xw = X * w[:, None]
    A = X.T @ Xw
    beta = np.linalg.solve(A, Xw.T @ y)
    res = y - X @ beta
    s2 = max(float((w * res ** 2).sum()) / max(1.0, len(y) - X.shape[1]), floor ** 2)
    return beta, np.linalg.inv(A) * s2, math.sqrt(s2)


OUTLIER_K = 5.0          # CHOSEN: a leg further than this many robust sigmas from the pooled fit is left out (said
                         # in the count). No DOE: a guard (school 17:02: wheel_scale 0.73 +- 0.20 from bad legs)


NOISY_X = 2.0            # CHOSEN: a run counts as noisy only when its own scatter is over twice the typical run's (a
                         # run with a handful of legs scatters by chance; equal runs stay at weight 1). No DOE: a guard


def session_weights(X, y, w, rows, floor):
    """(e14, school 17:02: one run's wheel_scale 0.73 +- 0.20, from dim light and interrupted legs) Each run's legs are
    weighted by that run's OWN noise: its residual sigma against its own fit. A run over NOISY_X times the typical
    run's sigma gets weight (typical / its)^2, so a run whose legs scatter widely barely moves the pooled value; a run
    with too few legs to judge keeps weight 1. Returns {session: weight}."""
    by = {}
    for k, r in enumerate(rows):
        by.setdefault(r.get("session"), []).append(k)
    sig = {}
    for sess, ix in by.items():
        if len(ix) >= X.shape[1] + 2:
            try:
                sig[sess] = wls(X[ix], y[ix], w[ix], floor)[2]
            except np.linalg.LinAlgError:
                pass
    if not sig:
        return {sess: 1.0 for sess in by}
    typ = float(np.median(list(sig.values())))
    return {sess: ((typ / sig[sess]) ** 2 if sess in sig and sig[sess] > NOISY_X * typ else 1.0) for sess in by}


def _robust(X, y, w, floor, rows):
    """Each run weighted by its own noise, then legs far from the pooled fit dropped, then WLS again:
    (beta, cov, sigma, legs dropped, {session: weight})."""
    w = np.asarray(w, float)
    sw = session_weights(X, y, w, rows, floor)
    w2 = np.array([wi * sw.get(r.get("session"), 1.0) for wi, r in zip(w, rows)])
    beta, cov, s = wls(X, y, w2, floor)
    res = (y - X @ beta) * np.sqrt(w2)
    keep = np.abs(res) <= OUTLIER_K * s
    dropped = int((~keep).sum())
    if dropped and keep.sum() >= X.shape[1] + 1:
        beta, cov, s = wls(X[keep], y[keep], w2[keep], floor)
    else:
        dropped = 0
    return beta, cov, s, dropped, {k: round(v, 4) for k, v in sw.items()}


def fit(folder, wifi, now=None, extra=()):
    """The pooled values for this robot: {gyro_scale, gyro_drift_dps, wheel_scale, coast_cm, se_*, n_*, offset,
    weights, words}; empty {} when nothing is pooled. extra: rows not saved yet (this run's)."""
    e = (load(folder)["robots"].get(wifi) or {}) if wifi else {}
    rows = list(e.get("rows") or []) + list(extra)
    if not rows:
        return {}
    now = time.time() if now is None else now
    out = {"n_runs": len(set(r.get("session") for r in rows)), "weights": {}}
    g = [r for r in rows if r["kind"] in ("turn", "still")]
    wts = np.array([age_w(r["t"], now) * battery_w(r.get("battery")) for r in g])
    if len(g) >= 3 and any(r["kind"] == "still" for r in g):
        X = np.array([[r["raw"], -r["dt"]] for r in g])
        y = np.array([r["cam"] for r in g])
        beta, cov, s, out_g, sw_g = _robust(X, y, wts, 0.05, g)
        out["outliers_gyro"], out["run_weights_gyro"] = out_g, sw_g
        out.update(gyro_scale=float(beta[0]), gyro_drift_dps=float(beta[1]), se_gyro_scale=float(math.sqrt(cov[0, 0])),
                   se_gyro_drift_dps=float(math.sqrt(cov[1, 1])), sigma_turn_deg=s,
                   n_turns=sum(1 for r in g if r["kind"] == "turn"), n_still=sum(1 for r in g if r["kind"] == "still"))
        out["weights"]["gyro"] = [round(float(wts.min()), 3), round(float(wts.max()), 3)]
    rl = [r for r in rows if r["kind"] == "roll"]
    if rl:
        b = np.array([r["counted"] for r in rl])
        c = np.array([r["cam"] for r in rl])
        w = np.array([age_w(r["t"], now) * battery_w(r.get("battery")) for r in rl])
        if len(b) >= 4 and np.ptp(b) > 10:
            beta, cov, s, out_r, sw_r = _robust(np.stack([b, np.ones_like(b)], 1), c, w, 0.1, rl)
            out.update(wheel_scale=float(beta[0]), coast_cm=float(beta[1]), se_wheel_scale=float(math.sqrt(cov[0, 0])))
        else:
            beta, cov, s, out_r, sw_r = _robust(b[:, None], c, w, 0.1, rl)
            out.update(wheel_scale=float(beta[0]), coast_cm=0.0, se_wheel_scale=float(math.sqrt(cov[0, 0])))
        out["outliers_wheel"], out["run_weights_wheel"] = out_r, sw_r
        out.update(sigma_roll_cm=s, n_rolls=len(rl), counted_cm=float(b.sum()))
        out["weights"]["wheel"] = [round(float(w.min()), 3), round(float(w.max()), 3)]
    offs = e.get("offsets") or []
    if offs:
        w = np.array([age_w(o["t"], now) / max(0.05, float(o.get("rms") or 0.2)) ** 2 for o in offs])
        a = np.array([o["ahead"] for o in offs])
        l_ = np.array([o["left"] for o in offs])
        out["offset"] = (float((w * a).sum() / w.sum()), float((w * l_).sum() / w.sum()))
        out["n_offsets"] = len(offs)
    return out


def full_run_se(pooled):
    """What ONE finished run's phase A would give, at the pool's own measured noise (derived, not picked): its design
    (calib_run TURNS_A at TURN_DPS, one STILL_S still period, ROLLS_A out and back) through the same least squares.
    {param: 1-sigma SE}."""
    import calib_run as CR
    out = {}
    s = pooled.get("sigma_turn_deg")
    if s:
        rows = [[abs(t), -abs(t) / CR.TURN_DPS] for t in CR.TURNS_A] + [[0.0, -CR.STILL_S]]
        X = np.array(rows, float)
        cov = np.linalg.inv(X.T @ X) * s ** 2
        out["gyro_scale"], out["gyro_drift_dps"] = float(math.sqrt(cov[0, 0])), float(math.sqrt(cov[1, 1]))
    s = pooled.get("sigma_roll_cm")
    if s:
        b = np.array([L for L in CR.ROLLS_A for _ in (0, 1)], float)
        X = np.stack([b, np.ones_like(b)], 1)
        out["wheel_scale"] = float(math.sqrt(np.linalg.inv(X.T @ X)[0, 0])) * s
    return out


def verdict(pooled, full=None):
    """{param: "ok" | why not}: ok when within calib_run's bounds and its pooled +- is no wider than ONE finished run's
    would be at the pool's own noise (full_run_se: derived, not picked)."""
    import calib_run as CR
    full = full_run_se(pooled) if full is None else full
    out = {}
    for k in CR.PARAMS:
        if k not in pooled or k not in full:
            out[k] = "not measured yet"
            continue
        lo, hi = CR.BOUNDS[k]
        se, lim = pooled["se_" + k], full[k]
        if not lo <= pooled[k] <= hi:
            out[k] = "outside %.2f-%.2f" % (lo, hi)
        elif se > lim * 1.0001:
            out[k] = "+-%.4f; a finished run would give +-%.4f" % (1.96 * se, 1.96 * lim)
        else:
            out[k] = "ok"
    return out


def needs(pooled):
    """What a new run still has to measure: {"turns": bool, "still": bool, "rolls": bool, "offset": bool}."""
    v = verdict(pooled) if pooled else {}
    return {"turns": v.get("gyro_scale") != "ok" or not pooled.get("offset"),
            "still": v.get("gyro_drift_dps") != "ok",
            "rolls": v.get("wheel_scale") != "ok",
            "offset": not pooled.get("offset")}


def words(pooled, full=None):
    if not pooled:
        return "nothing pooled yet"
    full = full_run_se(pooled) if full is None else full
    v = verdict(pooled, full)
    parts = []
    import calib_run as CR
    for k in CR.PARAMS:
        if k in pooled:
            parts.append("%s %.4f +- %.4f (%s)" % (k, pooled[k], 1.96 * pooled["se_" + k], v[k]))
        else:
            parts.append("%s not measured yet" % k)
    return "%s; from %d runs (%d turns, %d still, %d rolls)" % ("; ".join(parts), pooled.get("n_runs", 0),
                                                             pooled.get("n_turns", 0), pooled.get("n_still", 0),
                                                             pooled.get("n_rolls", 0))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["show"])
    ap.add_argument("wifi")
    ap.add_argument("--dir", default="")
    a = ap.parse_args(argv)
    folder = a.dir
    if not folder:
        import sites as SI
        folder = SI.load().get("conf_dir") or ""
    print(words(fit(folder, a.wifi)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
