"""Self-calibration for E02: each robot's gyro scale, gyro drift and wheel scale, learned from what the
camera saw against what the robot's own sensors counted, run by run (the C and KC conditions use the
latest accepted values).

What is compared, per straight stretch the camera sees the robot drive:
  - direction: the camera's direction of travel (phi) against the gyro's raw heading (psi = yaw_sign x
    the RAW yaw_deg column, unwrapped; corrections never touch it) at the same moment;
  - length: the camera's length against the library's own counted distance (RAW x_cm, y_cm; the library
    does not apply wheel_scale either).
Because both robot numbers are raw, the fit does not depend on the corrections a run used.

Model (firmware motion.h: heading = gyro_scale x raw - gyro_drift_dps x time; corr.h: real cm =
wheel_scale x counted cm):
    phi = g * psi - d * t + c_run          (one unknown start heading c per run)
    camera length = w * counted length
Fitted by iteratively re-weighted least squares (Huber weights, so a bad stretch counts less) with weak
priors from the real-footage session (docs/session-notes/2026-10-21-real-footage.md: gyro scale 1.069
Left, 1.075 Right; wheels 0.979). A fit outside sane bounds, or not fixed well enough by the data, is
refused: the previous accepted values stay.
"""
import math

import numpy as np

PRIOR = dict(gyro_scale=1.075, gyro_drift_dps=0.0, wheel_scale=0.979)
PRIOR_SD = dict(gyro_scale=0.05, gyro_drift_dps=0.5, wheel_scale=0.03)
BOUNDS = dict(gyro_scale=(0.97, 1.18), gyro_drift_dps=(-2.0, 2.0), wheel_scale=(0.92, 1.05))
SIGMA_DEG = 1.5         # camera direction noise per stretch (assumed)
SIGMA_CM = 0.5          # camera length noise per stretch (assumed)
MIN_TURNS = 2           # stretches either side of a turn of 30 deg or more, at least this many
MIN_COUNTED_CM = 40.0   # total straight driving needed for the wheel scale
# How well a number must be pinned down before it is used. The drift's must be below its prior SD (0.5), or the
# check could never refuse anything (2026-10-21b: it was 0.6). 0.3 is chosen: about 2 turns' worth of data;
# with a measured drift (naming check, prior SD 0.1) the measurement itself pins it.
MAX_SE = dict(gyro_scale=0.03, gyro_drift_dps=0.3, wheel_scale=0.02)
MAX_RMS_DEG = 4.0
# 2026-10-24 review: evening fits gave -0.08 to -0.25 deg/s of "drift" while the robots' own standing-still test read
# 0.000 and the one evening C run ended 13.2 cm off. With a measured standing-still drift, the fitted drift may differ
# from it by at most DRIFT_BAND_DPS; beyond that the drift is held at the band's edge and the gyro scale refitted.
# Set by data (2026-10-25, docs/build-reports/e02-oct25-logs-doe.md): refitting 49 runs' stretches at 0 / 0.02 / 0.05 /
# 0.1, band 0 (the drift held at the robot's own standing-still measurement) predicted far points and paths best,
# 0.12-0.15 cm better than 0.05 (p < 0.001), and the free drift ran to the band's edge in every session. Was 0.05.
DRIFT_BAND_DPS = 0.0
MAX_RMS_CM = 1.5


def wrap180(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


# =============================================================================== stretches
def moving_segments(cam, min_speed=2.5, window_s=0.3, min_s=0.3):
    """[(i0, i1)] index ranges of cam [(t, x, y), ...] where the robot moves steadily one way: split where it
    stops and where its direction of travel swings by more than 45 deg (a reversal, a turn while rolling)."""
    if len(cam) < 3:
        return []
    t = np.array([c[0] for c in cam], float)
    xy = np.array([[c[1], c[2]] for c in cam], float)
    n = len(t)
    vel = np.zeros((n, 2))
    j0 = j1 = 0
    for i in range(n):
        while t[i] - t[j0] > window_s:
            j0 += 1
        while j1 + 1 < n and t[j1 + 1] - t[i] <= window_s:
            j1 += 1
        dt_ = t[j1] - t[j0]
        vel[i] = (xy[j1] - xy[j0]) / dt_ if dt_ > 0 else 0.0
    speed = np.hypot(vel[:, 0], vel[:, 1])
    mov = speed > min_speed
    segs, i = [], 0
    while i < n:
        if not mov[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and mov[j + 1]:
            a, b = vel[j], vel[j + 1]
            if np.dot(a, b) < math.cos(math.radians(45)) * np.hypot(*a) * np.hypot(*b):
                break
            j += 1
        if t[j] - t[i] >= min_s:
            segs.append((i, j))
        i = j + 1
    return segs


def _split_straight(pts, tol):
    """Douglas-Peucker split of an index range into nearly straight pieces: kept whole when the points'
    scatter about their best line is camera noise (rms <= tol / 2), else split at the point furthest from
    the chord."""
    if len(pts) < 3:
        return [pts]
    c = pts - pts.mean(0)
    _, _, vt = np.linalg.svd(c, full_matrices=False)
    rms = float(np.sqrt(np.mean((c @ np.array([-vt[0][1], vt[0][0]])) ** 2)))
    if rms <= tol / 2:
        return [pts]
    a, b = pts[0], pts[-1]
    d = np.hypot(*(b - a))
    if d < 1e-9:
        dev = np.hypot(*(pts - a).T)
    else:
        u = (b - a) / d
        dev = np.abs((pts[:, 0] - a[0]) * -u[1] + (pts[:, 1] - a[1]) * u[0])
    k = int(np.argmax(dev))
    if dev[k] <= tol or k in (0, len(pts) - 1):
        return [pts]
    return [("split", k)]


JUMP_CM = 3.0          # chosen (2026-10-24 night analysis B6: 2/3/4 cm swept; a DOE could settle it): one picture
JUMP_CMS = 25.0        # moving more than this, and faster than this (cm/s: the robots drive 4-20), is the tracker
                       # jumping (catching up after lagging), not the robot


def split_at_jumps(cam, jump_cm=JUMP_CM, jump_cms=JUMP_CMS):
    """Pieces of a camera track [(t, x, y)] split wherever one picture jumps (more than jump_cm, and faster than
    jump_cms: a gap of dropped pictures is not a jump)."""
    out, cur = [], []
    for q in cam:
        if cur:
            d = math.hypot(q[1] - cur[-1][1], q[2] - cur[-1][2])
            dt = max(1e-3, q[0] - cur[-1][0])
            if d > jump_cm and d / dt > jump_cms:
                out.append(cur)
                cur = []
        cur.append(q)
    if cur:
        out.append(cur)
    return out


def camera_stretches(cam, min_len=6.0, tol_cm=0.8, trim_s=0.1, min_speed=2.5, jump_cm=JUMP_CM):
    """Straight stretches of driving in a camera track [(t, x, y), ...]: list of dict(t0, t1, tm, dir
    (deg, direction of travel), len (cm), p0, p1). The track is first cut where the camera's position jumps
    (2026-10-24 night analysis: 66% of the Pi plan's stage-2 stretches held a 3-7 cm jump along the direction of
    travel, which the straightness check can't see; cut there, stage 2's camera-against-wheels rms went from
    2.1-2.35 to 0.8-1.3 cm a stretch). jump_cm None: no cut."""
    if jump_cm:
        pieces = split_at_jumps(cam, jump_cm)
        if len(pieces) > 1:
            return [s for piece in pieces for s in camera_stretches(piece, min_len, tol_cm, trim_s, min_speed, None)]
    out = []
    if len(cam) < 3:
        return out
    t = np.array([c[0] for c in cam], float)
    xy = np.array([[c[1], c[2]] for c in cam], float)

    def emit(i0, i1):
        # trim the ends (speeding up, slowing down)
        a = i0
        while a < i1 and t[a] - t[i0] < trim_s:
            a += 1
        b = i1
        while b > a and t[i1] - t[b] < trim_s:
            b -= 1
        if b - a < 2:
            return
        seg = xy[a:b + 1]
        L = float(np.hypot(*(seg[-1] - seg[0])))
        if L < min_len:
            return
        c = seg - seg.mean(0)
        _, _, vt = np.linalg.svd(c, full_matrices=False)
        u = vt[0]
        if np.dot(u, seg[-1] - seg[0]) < 0:
            u = -u
        resid = c @ np.array([-u[1], u[0]])
        if float(np.sqrt(np.mean(resid ** 2))) > tol_cm / 2 + 0.1:
            return
        out.append(dict(t0=float(t[a]), t1=float(t[b]), tm=float(0.5 * (t[a] + t[b])),
                        dir=float(math.degrees(math.atan2(u[1], u[0]))), len=L,
                        p0=[float(seg[0, 0]), float(seg[0, 1])], p1=[float(seg[-1, 0]), float(seg[-1, 1])]))

    def rec(i0, i1, depth=0):
        pts = xy[i0:i1 + 1]
        r = _split_straight(pts, tol_cm)
        if r and isinstance(r[0], tuple) and depth < 12:
            k = i0 + r[0][1]
            rec(i0, k, depth + 1)
            rec(k, i1, depth + 1)
        else:
            emit(i0, i1)

    for i0, i1 in moving_segments(cam, min_speed=min_speed):
        rec(i0, i1)
    return out


# =============================================================================== robot side
def clock_offset(raw):
    """Laptop time - robot time (s) from RAW rows [dict(t_recv, ms, ...)]: the lower envelope (the
    fastest packet had the least delay)."""
    vals = [r["t_recv"] - r["ms"] / 1000.0 for r in raw if r.get("t_recv") is not None and r.get("ms") is not None]
    return min(vals) if vals else None


def raw_series(raw, yaw_sign=1, offset=None):
    """RAW rows -> dict of arrays: t (laptop s), x, y (library counted cm), th (library heading, raw deg),
    psi (yaw_sign x unwrapped raw yaw, deg). Rows need ms, x_cm, y_cm, yaw_deg (th_deg optional)."""
    off = clock_offset(raw) if offset is None else offset
    rows = [r for r in raw if all(r.get(k) is not None for k in ("ms", "x_cm", "y_cm", "yaw_deg"))]
    rows.sort(key=lambda r: r["ms"])
    if not rows or off is None:
        return None
    t = np.array([r["ms"] / 1000.0 + off for r in rows])
    yaw = np.degrees(np.unwrap(np.radians([r["yaw_deg"] for r in rows])))
    th = np.array([r.get("th_deg") if r.get("th_deg") is not None else np.nan for r in rows], float)
    return dict(t=t, x=np.array([r["x_cm"] for r in rows], float), y=np.array([r["y_cm"] for r in rows], float),
                th=th, psi=(yaw_sign or 1) * yaw)


def pair(stretches, ser, run="run", max_step_cm=3.0):
    """Match camera stretches with the robot's own counting over the same time. Returns
    [dict(run, tm, phi, psi, cam_len, bel_len, forward)] (phi already turned round for backwards driving)."""
    out = []
    if ser is None:
        return out
    t = ser["t"]
    for s in stretches:
        k = np.nonzero((t >= s["t0"]) & (t <= s["t1"]))[0]
        if len(k) < 3:
            continue
        x0, y0 = np.interp(s["t0"], t, ser["x"]), np.interp(s["t0"], t, ser["y"])
        x1, y1 = np.interp(s["t1"], t, ser["x"]), np.interp(s["t1"], t, ser["y"])
        steps = np.hypot(np.diff(ser["x"][k]), np.diff(ser["y"][k]))
        if len(steps) and steps.max() > max_step_cm:
            continue                        # the library pose was reset inside this stretch
        bel = float(math.hypot(x1 - x0, y1 - y0))
        if bel < 1.0:
            continue
        thm = np.nanmean(ser["th"][k]) if np.any(np.isfinite(ser["th"][k])) else None
        forward = True
        if thm is not None and np.isfinite(thm):
            forward = (x1 - x0) * math.cos(math.radians(thm)) + (y1 - y0) * math.sin(math.radians(thm)) >= 0
        phi = s["dir"] if forward else float(wrap180(s["dir"] + 180.0))
        out.append(dict(run=run, tm=s["tm"], phi=float(phi), psi=float(np.mean(ser["psi"][k])),
                        cam_len=float(s["len"]), bel_len=bel, forward=bool(forward)))
    return out


# =============================================================================== the fit
def _huber_w(r, k):
    a = np.abs(r)
    return np.where(a <= k, 1.0, k / np.maximum(a, 1e-12))


def fit_turns(obs, prior=None, prior_sd=None, iters=20, fixed_d=None):
    """obs: [dict(run, tm, phi, psi)]. Returns dict(g, d, se_g, se_d, rms_deg, n, turns) or None.
    fixed_d: hold the drift at this value (deg/s) and fit only the scale (se_d is then 0)."""
    prior = dict(PRIOR, **(prior or {}))
    prior_sd = dict(PRIOR_SD, **(prior_sd or {}))
    g0, d0 = prior["gyro_scale"], prior["gyro_drift_dps"]
    runs = {}
    for o in obs:
        runs.setdefault(o["run"], []).append(o)
    rows = []                    # (run, psi, t, phi unwrapped)
    turns = 0
    for r, lst in runs.items():
        lst = sorted(lst, key=lambda o: o["tm"])
        if len(lst) < 2:
            continue
        b = lst[0]
        rows.append((r, 0.0, 0.0, 0.0))
        prev = b
        for o in lst[1:]:
            dpsi, dt_ = o["psi"] - b["psi"], o["tm"] - b["tm"]
            pred = g0 * dpsi - d0 * dt_
            dphi = float(wrap180(o["phi"] - b["phi"] - pred)) + pred
            rows.append((r, dpsi, dt_, dphi))
            if abs(o["psi"] - prev["psi"]) >= 30.0:
                turns += 1
            prev = o
    if len(rows) < 3:
        return None
    run_ids = sorted({r for r, *_ in rows}, key=str)
    idx = {r: i for i, r in enumerate(run_ids)}
    n, p = len(rows), 2 + len(run_ids)
    X = np.zeros((n, p))
    yv = np.zeros(n)
    for i, (r, dpsi, dt_, dphi) in enumerate(rows):
        X[i, 0], X[i, 1], X[i, 2 + idx[r]] = dpsi, -dt_, 1.0
        yv[i] = dphi + (fixed_d * dt_ if fixed_d is not None else 0.0)
    if fixed_d is not None:
        X[:, 1] = 0.0
    lg = SIGMA_DEG / prior_sd["gyro_scale"]
    ld = SIGMA_DEG / prior_sd["gyro_drift_dps"]
    P = np.zeros((2, p))
    P[0, 0], P[1, 1] = lg, ld
    py = np.array([lg * g0, ld * d0])
    w = np.ones(n)
    beta = None
    for _ in range(iters):
        sw = np.sqrt(w)
        A = np.vstack([X * sw[:, None], P])
        bv = np.concatenate([yv * sw, py])
        beta, *_ = np.linalg.lstsq(A, bv, rcond=None)
        res = yv - X @ beta
        w_new = _huber_w(res, 1.5 * SIGMA_DEG)
        if np.allclose(w_new, w, atol=1e-4):
            w = w_new
            break
        w = w_new
    res = yv - X @ beta
    dof = max(1, n - p)
    s2 = max(float(np.sum(w * res ** 2) / dof), SIGMA_DEG ** 2 * 0.25)
    A = np.vstack([X * np.sqrt(w)[:, None], P])
    cov = np.linalg.pinv(A.T @ A) * s2
    if fixed_d is not None:
        beta[1], cov[1, 1] = fixed_d, 0.0
    return dict(g=float(beta[0]), d=float(beta[1]), se_g=float(math.sqrt(max(cov[0, 0], 0))),
                se_d=float(math.sqrt(max(cov[1, 1], 0))), rms_deg=float(math.sqrt(np.sum(w * res ** 2) / np.sum(w))),
                n=n, turns=turns)


def fit_wheels(obs, prior=None, prior_sd=None, iters=20):
    """obs: [dict(cam_len, bel_len)]. Returns dict(w, se_w, rms_cm, n, counted_cm) or None."""
    prior = dict(PRIOR, **(prior or {}))
    prior_sd = dict(PRIOR_SD, **(prior_sd or {}))
    if not obs:
        return None
    b = np.array([o["bel_len"] for o in obs], float)
    c = np.array([o["cam_len"] for o in obs], float)
    w0 = prior["wheel_scale"]
    lw = SIGMA_CM / prior_sd["wheel_scale"]
    wt = np.ones(len(b))
    est = w0
    for _ in range(iters):
        num = float(np.sum(wt * b * c) + lw * lw * w0)
        den = float(np.sum(wt * b * b) + lw * lw)
        est = num / den
        res = c - est * b
        wt_new = _huber_w(res, 1.5 * SIGMA_CM)
        if np.allclose(wt_new, wt, atol=1e-4):
            wt = wt_new
            break
        wt = wt_new
    res = c - est * b
    dof = max(1, len(b) - 1)
    s2 = max(float(np.sum(wt * res ** 2) / dof), SIGMA_CM ** 2 * 0.25)
    se = math.sqrt(s2 / (float(np.sum(wt * b * b)) + lw * lw))
    return dict(w=float(est), se_w=se, rms_cm=float(math.sqrt(np.sum(wt * res ** 2) / np.sum(wt))), n=len(b),
                counted_cm=float(b.sum()))


def fit_robot(obs, previous=None, prior=None, prior_sd=None, measured_drift=None, max_rms_cm=None):
    """The whole self-calibration for one robot from all its paired stretches so far.
    previous: the last accepted values (dict gyro_scale, gyro_drift_dps, wheel_scale) or None.
    Returns dict(values (what C/KC should use now), accepted (bool: this fit was taken), fitted (raw fit
    numbers), reasons (why a part was refused), n_runs). measured_drift: the robot's own standing-still drift
    (deg/s); the fitted drift is kept within DRIFT_BAND_DPS of it (2026-10-24 review)."""
    prior = dict(PRIOR, **(prior or {}))
    values = dict(previous or {})
    reasons = []
    fitted = {}
    ok_turn = ok_wheel = False
    t = fit_turns(obs, prior, prior_sd)
    held = None
    if t is not None and measured_drift is not None and abs(t["d"] - measured_drift) > DRIFT_BAND_DPS:
        held = measured_drift + math.copysign(DRIFT_BAND_DPS, t["d"] - measured_drift)
        free_d = t["d"]
        t = fit_turns(obs, prior, prior_sd, fixed_d=held)
        reasons.append("gyro drift %+.2f deg/s fitted, more than %.2f from the %+.3f measured standing still: held at "
                       "%+.3f and the scale refitted" % (free_d, DRIFT_BAND_DPS, measured_drift, held))
    if t is None:
        reasons.append("not enough straight stretches seen yet for the gyro")
    else:
        fitted.update(gyro_scale=t["g"], gyro_drift_dps=t["d"], se_gyro_scale=t["se_g"], se_gyro_drift_dps=t["se_d"],
                      rms_deg=t["rms_deg"], turns=t["turns"], n_turn_obs=t["n"], drift_held=held)
        lo, hi = BOUNDS["gyro_scale"]
        dlo, dhi = BOUNDS["gyro_drift_dps"]
        if t["turns"] < MIN_TURNS:
            reasons.append("only %d turn(s) seen so far (need %d) for the gyro scale" % (t["turns"], MIN_TURNS))
        elif not (lo <= t["g"] <= hi):
            reasons.append("gyro scale %.3f is outside the sane range %.2f-%.2f: refused" % (t["g"], lo, hi))
        elif not (dlo <= t["d"] <= dhi):
            reasons.append("gyro drift %.2f deg/s is outside %.0f to %.0f: refused" % (t["d"], dlo, dhi))
        elif t["se_g"] > MAX_SE["gyro_scale"] or t["se_d"] > MAX_SE["gyro_drift_dps"]:
            reasons.append("gyro numbers not pinned down yet (+-%.3f, +-%.2f deg/s)" % (t["se_g"], t["se_d"]))
        elif t["rms_deg"] > MAX_RMS_DEG:
            reasons.append("camera and gyro disagree by %.1f deg on average: refused" % t["rms_deg"])
        else:
            ok_turn = True
            values.update(gyro_scale=round(t["g"], 4), gyro_drift_dps=round(t["d"], 4))
    wobs = [o for o in obs if o.get("bel_len") and o.get("cam_len")]
    wf = fit_wheels(wobs, prior, prior_sd)
    if wf is None:
        reasons.append("no straight driving seen yet for the wheels")
    else:
        fitted.update(wheel_scale=wf["w"], se_wheel_scale=wf["se_w"], rms_cm=wf["rms_cm"], counted_cm=wf["counted_cm"])
        lo, hi = BOUNDS["wheel_scale"]
        if wf["counted_cm"] < MIN_COUNTED_CM:
            reasons.append("only %.0f cm of straight driving seen (need %.0f) for the wheels"
                           % (wf["counted_cm"], MIN_COUNTED_CM))
        elif not (lo <= wf["w"] <= hi):
            reasons.append("wheel scale %.3f is outside the sane range %.2f-%.2f: refused" % (wf["w"], lo, hi))
        elif wf["se_w"] > MAX_SE["wheel_scale"]:
            reasons.append("wheel scale not pinned down yet (+-%.3f)" % wf["se_w"])
        elif wf["rms_cm"] > (max_rms_cm if max_rms_cm is not None else MAX_RMS_CM):
            reasons.append("camera and wheels disagree by %.1f cm per stretch: refused" % wf["rms_cm"])
        else:
            ok_wheel = True
            values.update(wheel_scale=round(wf["w"], 4))
    # parts are used as they pass (2026-10-24 Pi plan: the gyro passed every fit, but the wheels' refusal threw it
    # away with them for the whole session): a part not learned yet comes from the prior, and says so
    keys = ("gyro_scale", "gyro_drift_dps", "wheel_scale")
    prior_parts = set((previous or {}).get("prior_parts") or [])
    if ok_turn:
        prior_parts -= {"gyro_scale", "gyro_drift_dps"}
    if ok_wheel:
        prior_parts -= {"wheel_scale"}
    if any(k in values for k in keys):
        for k in keys:
            if k not in values:
                values[k] = round(prior[k], 4)
                prior_parts.add(k)
        values["prior_parts"] = sorted(prior_parts)
        if not prior_parts:
            values.pop("prior_parts", None)
    complete = all(k in values for k in keys)
    return dict(values=values if complete else None, accepted=ok_turn or ok_wheel, gyro_ok=ok_turn,
                wheel_ok=ok_wheel, fitted=fitted, reasons=reasons, n_runs=len({o["run"] for o in obs}))


def describe(values):
    if not values:
        return "no learned corrections yet"
    out = "gyro scale %.3f, drift %+.2f deg/s, wheels %.3f" % (
        values["gyro_scale"], values["gyro_drift_dps"], values["wheel_scale"])
    if values.get("prior_parts"):
        out += " (not learned yet, the starting values: %s)" % ", ".join(values["prior_parts"])
    return out
