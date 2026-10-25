"""Per-stream features for blind sensor discovery.

Every feature is computed from a stream's own values and times, the commanded legs and the camera track. No
feature reads a column name. Most are scale-free (correlations, ratios to the stream's own noise), so a stream
reported in other units, rescaled or sign-flipped gives the same features; the few that are not (counter steps of
exactly 1, a clock's rate) are marked where they are made.

Main entry: `case_features(case)` -> (Grid, {stream id: dict of features})."""
import re
from dataclasses import dataclass

import numpy as np

GRID_HZ = 10.0          # resample rate for relations (chosen: RAW is ~20 Hz, the front sensor ~15 Hz)
SEG_GAP_S = 1.0         # a gap in rows longer than this starts a new segment (logging paused between runs)
WIN_S = 1.0             # window for increment / integral relations (chosen)
REST_Q = 0.10           # rest noise = this quantile of per-window spread (chosen)

# a plain or scientific-notation number as printf writes it; never a leading zero before another digit (a run id
# such as '052E-3' would otherwise read as 0.052)
_NUM = re.compile(r"^-?(0|[1-9]\d*)(\.\d+)?([eE][+-]?\d+)?$")


@dataclass
class Grid:
    t: np.ndarray           # grid times (s)
    seg: np.ndarray         # segment number per grid point
    v: dict                 # stream id -> values on the grid (unwrapped where a wrap was found), NaN where missing
    dt: float


def parse(vals):
    """(numeric array, is_text). Text = any non-empty value that is not a plain decimal number (a run id such as
    '052E-3' would otherwise read as 0.052)."""
    nonempty = [v for v in vals if v not in ("", None)]
    if not nonempty:
        return np.full(len(vals), np.nan), True
    bad = sum(1 for v in nonempty[:: max(1, len(nonempty) // 2000)] if not _NUM.match(str(v)))
    if bad:
        return np.full(len(vals), np.nan), True
    out = np.array([float(v) if v not in ("", None) else np.nan for v in vals])
    return out, False


def segments(t):
    seg = np.zeros(len(t), dtype=int)
    if len(t) > 1:
        seg[1:] = np.cumsum(np.diff(t) > SEG_GAP_S)
    return seg


def detect_wrap(x, seg):
    """(wrapped?, period). A wrapped angle jumps by about its whole range between neighbouring rows while its
    ordinary steps are small."""
    f = np.isfinite(x)
    if f.sum() < 20:
        return False, None
    lo, hi = np.nanmin(x), np.nanmax(x)
    rng = hi - lo
    if rng <= 0:
        return False, None
    d = np.diff(x)
    same = (np.diff(seg) == 0) & np.isfinite(d)
    d = d[same]
    if len(d) < 10:
        return False, None
    big = np.abs(d) > 0.6 * rng
    small = np.abs(d[~big])
    if big.sum() < 1 or len(small) == 0:
        return False, None
    if np.percentile(small, 99) > 0.05 * rng:
        return False, None
    # the jumps must be close to one size (the period) and go from one end of the range to the other
    p = np.median(np.abs(d[big]))
    if np.std(np.abs(d[big])) > 0.1 * p:
        return False, None
    # a wrapped angle sweeps its whole range; a sensor that drops out to a limit value sits at two ends
    hist, _ = np.histogram(x[f], bins=10)
    if np.sum(hist > 0.002 * f.sum()) < 4:
        return False, None
    xa = x[:-1][same][big]
    xb = x[1:][same][big]
    near = lambda v: (v - lo < 0.1 * rng) | (hi - v < 0.1 * rng)
    if np.mean(near(xa) & near(xb)) < 0.9:
        return False, None
    return True, rng


def unwrap(x, seg, period):
    out = x.copy()
    for s in np.unique(seg):
        k = (seg == s) & np.isfinite(x)
        if k.sum() > 1:
            out[k] = np.unwrap(x[k], period=period)
    return out


def make_grid(t, num, seg):
    """Sample-and-hold every numeric stream onto a GRID_HZ grid inside each segment."""
    gt, gs = [], []
    for s in np.unique(seg):
        ts = t[seg == s]
        if ts[-1] - ts[0] < 1.0:
            continue
        g = np.arange(ts[0], ts[-1], 1.0 / GRID_HZ)
        gt.append(g)
        gs.append(np.full(len(g), s))
    if not gt:
        return Grid(np.zeros(0), np.zeros(0, dtype=int), {k: np.zeros(0) for k in num}, 1.0 / GRID_HZ)
    gt, gs = np.concatenate(gt), np.concatenate(gs)
    idx = np.searchsorted(t, gt, side="right") - 1
    idx = np.clip(idx, 0, len(t) - 1)
    ok = seg[idx] == gs
    v = {}
    for k, x in num.items():
        y = x[idx].astype(float)
        y[~ok] = np.nan
        v[k] = y
    return Grid(gt, gs, v, 1.0 / GRID_HZ)


def _corr(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    k = np.isfinite(a) & np.isfinite(b)
    if k.sum() < 4:
        return np.nan
    a, b = a[k], b[k]
    sa, sb = a.std(), b.std()
    if sa <= 0 or sb <= 0:
        return np.nan
    return float(np.mean((a - a.mean()) * (b - b.mean())) / (sa * sb))


def _robust_sd(x):
    x = x[np.isfinite(x)]
    if len(x) < 2:
        return np.nan
    return float(1.4826 * np.median(np.abs(x - np.median(x))))


def passive(x, t, seg, is_text):
    """Stage A features: what the stream looks like regardless of motion."""
    f = {"text": bool(is_text)}
    n = len(x)
    dur = max(1e-6, float(np.sum(np.diff(t)[np.diff(seg) == 0]))) if n > 1 else 1e-6
    f["duration_s"] = dur
    f["row_hz"] = n / dur
    if is_text:
        f.update(const=False, n_unique=0)
        return f
    fin = np.isfinite(x)
    f["finite_frac"] = float(fin.mean()) if n else 0.0
    xv = x[fin]
    u = np.unique(xv)
    f["n_unique"] = int(len(u))
    f["const"] = len(u) <= 1
    if len(u) <= 1:
        f["level"] = float(u[0]) if len(u) else np.nan
        return f
    f["integer"] = bool(np.all(np.abs(xv - np.round(xv)) < 1e-9))
    f["binary"] = len(u) == 2
    f["lo"], f["hi"] = float(xv.min()), float(xv.max())
    d = np.diff(x)
    same = (np.diff(seg) == 0) & np.isfinite(d)
    d = d[same]
    nz = np.abs(d[d != 0])
    f["change_frac"] = float((d != 0).mean()) if len(d) else 0.0
    f["change_hz"] = float((d != 0).sum() / dur)
    f["quantum"] = float(np.percentile(nz, 5)) if len(nz) else 0.0
    iqr = np.subtract(*np.percentile(xv, [75, 25]))
    span = xv.max() - xv.min()
    f["rel_quantum"] = float(f["quantum"] / span) if span > 0 else 0.0
    f["levels_per_span"] = float(len(u))
    f["up_frac"] = float((d > 0).mean()) if len(d) else 0.0
    f["down_frac"] = float((d < 0).mean()) if len(d) else 0.0
    # counter: every step the same positive size (not scale-free: a counter steps by one unit)
    pos = d[d > 0]
    f["step_mode_frac"] = float(np.mean(np.isclose(d, np.median(pos)))) if len(pos) else 0.0
    f["one_quantum_frac"] = float(np.mean(np.isclose(nz, f["quantum"], rtol=0.05))) if len(nz) else 0.0
    f["monotone_up"] = float((d >= 0).mean()) if len(d) else 0.0
    # clock: rises in proportion to elapsed time
    rates = []
    for s in np.unique(seg):
        k = (seg == s) & fin
        if k.sum() > 10 and t[k][-1] - t[k][0] > 1:
            c = _corr(t[k], x[k])
            if np.isfinite(c):
                rates.append(c)
    f["time_corr_in_seg"] = float(np.median(rates)) if rates else np.nan
    # trend over the whole case (battery-like streams fall slowly)
    f["trend_corr"] = _corr(t[fin], xv)
    # rest level / noise: quietest windows of the stream itself
    sds, meds = [], []
    w = max(5, int(round(1.0 * f["row_hz"])))
    for s in np.unique(seg):
        xs = x[(seg == s) & fin]
        for i in range(0, len(xs) - w + 1, w):
            seg_x = xs[i:i + w]
            sds.append(np.std(seg_x))
            meds.append(np.median(seg_x))
    sds, meds = np.array(sds), np.array(meds)
    if len(sds):
        q = np.quantile(sds, REST_Q)
        quiet = sds <= q + 1e-12
        f["rest_noise"] = float(max(q, 0.0))
        f["rest_level"] = float(np.median(meds[quiet]))
        f["window_sd_median"] = float(np.median(sds))
        f["quiet_frac"] = float(np.mean(sds <= max(q, 1e-12) * 1.0 + 1e-12))
    else:
        f["rest_noise"], f["rest_level"], f["window_sd_median"], f["quiet_frac"] = np.nan, np.nan, np.nan, np.nan
    sd_all = np.std(xv)
    f["level_over_noise"] = float(abs(f["rest_level"]) / max(f["rest_noise"], f["quantum"] or 1e-9, 1e-12)) \
        if np.isfinite(f.get("rest_level", np.nan)) else np.nan
    f["level_over_spread"] = float(abs(np.median(xv)) / sd_all) if sd_all > 0 else np.nan
    # typical 1 s spread over its rest noise, over every window (still ones included). relations.analyse replaces
    # active_over_rest with the same figure over the windows where the robot moves (moving_response below) once
    # it knows when the robot stands still; this all-window value stays as active_over_rest_all
    f["active_over_rest"] = float(f["window_sd_median"] / max(f["rest_noise"], f["quantum"] or 1e-9, 1e-12)) \
        if np.isfinite(f.get("window_sd_median", np.nan)) else np.nan
    f["active_over_rest_all"] = f["active_over_rest"]
    # exactly flat stretches of 1 s or more (an encoder or a computed pose at rest; a noisy sensor never is)
    w1 = max(5, int(round(f["row_hz"])))
    flat = np.zeros(n, bool)
    eq = np.r_[False, (np.diff(x) == 0) & (np.diff(seg) == 0)]
    i = 0
    while i < n:
        if eq[i]:
            j = i
            while j + 1 < n and eq[j + 1]:
                j += 1
            if j - i + 2 >= w1:
                flat[i - 1:j + 1] = True
            i = j + 1
        else:
            i += 1
    f["flat_frac"] = float(flat.mean())
    f["_flat"] = flat
    big = np.abs(d) > 0
    f["smooth"] = float(np.percentile(np.abs(d), 99) / span) if span > 0 and len(d) else np.nan
    # bounded? the share of the range the 1-99% band covers; sentinel-like extremes
    f["sat_frac"] = float(np.mean((xv >= xv.max()) | (xv <= xv.min())))
    return f


MOVE_MIN_WINDOWS = 5    # 1 s windows of motion needed before the motion response counts (chosen)


def moving_response(x, seg, moving, row_hz, rest_noise, quantum):
    """Motion response: the median 1 s spread over the windows where the robot moves (every row in the window
    moving), divided by the stream's rest noise. NaN with fewer than MOVE_MIN_WINDOWS such windows.

    Why not every window: a range zone that looks past the boards while parked flickers by tens of cm, and that
    flicker, not motion, set the all-window figure (Right's zones looked 3-7x where Left/Center's looked 12-29x;
    build-reports/e02-oct25-right-front-and-rpm.md)."""
    w = max(5, int(round(row_hz)))
    fin = np.isfinite(x)
    sds = []
    for s in np.unique(seg):
        idx = np.where((seg == s) & fin)[0]
        for i in range(0, len(idx) - w + 1, w):
            k = idx[i:i + w]
            if np.all(moving[k]):
                sds.append(np.std(x[k]))
    if len(sds) < MOVE_MIN_WINDOWS:
        return np.nan, len(sds)
    return float(np.median(sds) / max(rest_noise if np.isfinite(rest_noise) else 0, quantum or 1e-9, 1e-12)), len(sds)


def _win(gt, a, b):
    i0, i1 = np.searchsorted(gt, a), np.searchsorted(gt, b)
    return i0, i1


def leg_features(g, legs, sid):
    """Stage B features from the commanded legs: change over a leg, level in the leg's middle, pulse at its start,
    each against the commanded value (scale-free: correlations, and slopes kept only for calibration)."""
    y = g.v[sid]
    out = {}
    for kind in ("turn", "go"):
        dl, mid, val, rate, imp, absd = [], [], [], [], [], []
        for L in legs:
            if L.kind != kind or not L.ok:
                continue
            dur = L.t1 - L.t0
            if dur < 0.3 or abs(L.value) < 1e-6:
                continue
            a0, a1 = _win(g.t, L.t0 - 0.35, L.t0)
            b0, b1 = _win(g.t, L.t1, L.t1 + 0.35)
            m0, m1 = _win(g.t, L.t0 + 0.25 * dur, L.t1 - 0.25 * dur)
            s0, s1 = _win(g.t, L.t0, L.t0 + min(1.0, 0.5 * dur))
            if a1 - a0 < 1 or b1 - b0 < 1 or m1 - m0 < 1 or s1 - s0 < 1:
                continue
            if g.seg[a0] != g.seg[min(b1, len(g.t)) - 1]:
                continue
            pre, post = np.nanmedian(y[a0:a1]), np.nanmedian(y[b0:b1])
            if not (np.isfinite(pre) and np.isfinite(post)):
                continue
            dl.append(post - pre)
            mid.append(np.nanmedian(y[m0:m1]) - pre)
            imp.append(np.nansum(y[s0:s1] - pre) * g.dt)
            val.append(L.value)
            rate.append(L.value / dur)
        dl, mid, val, rate, imp = map(np.array, (dl, mid, val, rate, imp))
        n = len(val)
        out[kind + "_n"] = n
        if n < 4:
            for k in ("delta_corr", "mid_corr", "abs_delta_corr", "impulse_corr", "snr_mid", "snr_delta",
                      "delta_slope", "mid_slope", "delta_sign_agree"):
                out["%s_%s" % (kind, k)] = np.nan
            continue
        out[kind + "_delta_corr"] = _corr(dl, val)
        out[kind + "_mid_corr"] = _corr(mid, rate)
        out[kind + "_abs_delta_corr"] = _corr(np.abs(dl), np.abs(val))
        out[kind + "_impulse_corr"] = _corr(imp, np.sign(val))
        out[kind + "_delta_slope"] = float(np.median(dl / val))
        out[kind + "_mid_slope"] = float(np.median(mid / rate))
        sgn = np.sign(np.median(dl / val)) or 1.0
        out[kind + "_delta_sign_agree"] = float(np.mean(np.sign(dl) == sgn * np.sign(val)))
        out[kind + "_mid_abs"] = float(np.median(np.abs(mid)))
        out[kind + "_delta_abs"] = float(np.median(np.abs(dl)))
    return out


def camera_series(case, g):
    """Camera speed on the grid (cm/s, NaN where the camera has no fix within 0.3 s)."""
    if case.cam is None or len(case.cam) < 10 or len(g.t) == 0:
        return None
    ct, cx, cy = case.cam[:, 0], case.cam[:, 1], case.cam[:, 2]
    # 1 s smoothed positions
    sp = np.full(len(g.t), np.nan)
    i0 = np.searchsorted(ct, g.t - 0.5)
    i1 = np.searchsorted(ct, g.t + 0.5) - 1
    ok = (i1 > i0) & (i1 < len(ct)) & (i0 >= 0)
    ii0, ii1 = i0[ok], np.minimum(i1[ok], len(ct) - 1)
    dtc = ct[ii1] - ct[ii0]
    good = dtc > 0.5
    v = np.hypot(cx[ii1] - cx[ii0], cy[ii1] - cy[ii0]) / np.where(good, dtc, 1)
    v[~good] = np.nan
    tmp = np.full(ok.sum(), np.nan)
    tmp[:] = v
    sp[ok] = tmp
    return sp


def camera_features(case, g, sid, cam_speed):
    """Against the camera: a stream's size or its rate of change following the camera's speed, its change over a
    go leg against the camera's distance driven, and range-like streams against the camera's distance to the
    boards along the direction of travel."""
    out = {}
    y = g.v[sid]
    if cam_speed is None:
        return out
    w = int(round(1.0 / g.dt))
    dy = np.full(len(y), np.nan)
    dy[w:] = np.abs(y[w:] - y[:-w]) / (w * g.dt)
    same = np.zeros(len(y), bool)
    same[w:] = g.seg[w:] == g.seg[:-w]
    dy[~same] = np.nan
    out["cam_speed_corr_abs"] = _corr(np.abs(y - np.nanmedian(y)), cam_speed)
    out["cam_speed_corr_rate"] = _corr(dy, cam_speed)
    # per go leg: change vs signed camera distance; range-like: reading vs distance to the boards ahead
    ct = case.cam[:, 0] if case.cam is not None and len(case.cam) else np.zeros(0)
    dl, dc, rd, pr = [], [], [], []
    for L in case.legs:
        if L.kind != "go" or not L.ok or abs(L.value) < 3 or len(ct) == 0:
            continue
        j0 = np.searchsorted(ct, [L.t0 - 0.4, L.t0])
        j1 = np.searchsorted(ct, [L.t1, L.t1 + 0.4])
        if j0[1] - j0[0] < 1 or j1[1] - j1[0] < 1:
            continue
        p0 = np.median(case.cam[j0[0]:j0[1], 1:3], axis=0)
        p1 = np.median(case.cam[j1[0]:j1[1], 1:3], axis=0)
        disp = p1 - p0
        dist = float(np.hypot(*disp))
        if dist < 2:
            continue
        a0, a1 = _win(g.t, L.t0 - 0.35, L.t0)
        b0, b1 = _win(g.t, L.t1, L.t1 + 0.35)
        if a1 - a0 < 1 or b1 - b0 < 1:
            continue
        pre, post = np.nanmedian(y[a0:a1]), np.nanmedian(y[b0:b1])
        if not (np.isfinite(pre) and np.isfinite(post)):
            continue
        dl.append(post - pre)
        dc.append(np.sign(L.value) * dist)
        if case.board:
            head = disp / dist * np.sign(L.value)       # the way the robot faces
            for p, r in ((p0, pre), (p1, post)):
                d = _ray_to_boards(p, head, case.board)
                if d is not None and d < 120:
                    rd.append(r)
                    pr.append(d)
    dl, dc = np.array(dl), np.array(dc)
    out["cam_go_n"] = len(dl)
    if len(dl) >= 4:
        out["cam_go_delta_corr"] = _corr(dl, dc)
        out["cam_go_delta_slope"] = float(np.median(dl / dc))
    out["board_n"] = len(rd)
    if len(rd) >= 6:
        rd, pr = np.array(rd), np.array(pr)
        out["board_corr"] = _corr(rd, pr)
        A = np.vstack([pr, np.ones(len(pr))]).T
        sol, *_ = np.linalg.lstsq(A, rd, rcond=None)
        out["board_slope"], out["board_offset"] = float(sol[0]), float(sol[1])
    return out


def _ray_to_boards(p, head, board):
    """Distance from p along unit vector head to the x board line (x = board['x']) or y board line."""
    best = None
    for axis, line in ((0, board["x"]), (1, board["y"])):
        h = head[axis]
        if abs(h) < 1e-3:
            continue
        s = (line - p[axis]) / h
        if s > 0:
            other = p[1 - axis] + s * head[1 - axis]
            if -10 <= other <= 135:
                best = s if best is None else min(best, s)
    return best


CAM_MOVE_CMS = 2.5      # camera speed that counts as driving (chosen)
K_SCAN = np.concatenate([-np.logspace(-3, 3, 1200), np.logspace(-3, 3, 1200)])


def camera_episodes(case, g, cam_speed):
    """Stretches the camera sees the robot drive (speed > CAM_MOVE_CMS for 1 s+, 4 cm+): (i0, i1, heading deg,
    distance cm) on the grid. The heading is the direction of travel, which is the robot's facing or its
    opposite (the camera does not know which way the robot faces)."""
    if cam_speed is None:
        return []
    ct, cx, cy = case.cam[:, 0], case.cam[:, 1], case.cam[:, 2]
    mov = np.nan_to_num(cam_speed) > CAM_MOVE_CMS
    eps, i, N = [], 0, len(g.t)
    while i < N:
        if not mov[i]:
            i += 1
            continue
        j = i
        while j + 1 < N and mov[j + 1] and g.seg[j + 1] == g.seg[i]:
            j += 1
        if g.t[j] - g.t[i] >= 1.0:
            a0, a1 = np.searchsorted(ct, [g.t[i], g.t[j]])
            if a1 - a0 > 5:
                d = np.array([cx[a1 - 1] - cx[a0], cy[a1 - 1] - cy[a0]])
                if np.hypot(*d) > 4:
                    eps.append((i, j, float(np.degrees(np.arctan2(d[1], d[0]))), float(np.hypot(*d))))
        i = j + 1
    return eps


def camera_turn_pairs(g, eps, max_gap_s=30.0):
    """Consecutive drive stretches in one segment, and the camera's heading change between them (deg, folded
    to +-90 because either stretch may have been driven backwards)."""
    pairs = [(a, b) for a, b in zip(eps[:-1], eps[1:]) if g.seg[a[1]] == g.seg[b[0]]
             and g.t[b[0]] - g.t[a[1]] < max_gap_s]
    dh = np.array([((b[2] - a[2] + 90) % 180) - 90 for a, b in pairs])
    return pairs, dh


def _axial_scan(arr, dh, ks=K_SCAN):
    """Best k so that k * arr follows the camera's heading change (axial: mod 180). (R, k)."""
    ok = np.isfinite(arr)
    if ok.sum() < 8:
        return np.nan, np.nan
    a, h = arr[ok], dh[ok]
    R = np.mean(np.cos(np.radians(2 * (ks[:, None] * a[None, :] - h[None, :]))), axis=1)
    j = int(np.argmax(R))
    # refine around the best k
    k0 = ks[j]
    fine = k0 * np.linspace(0.97, 1.03, 61)
    Rf = np.mean(np.cos(np.radians(2 * (fine[:, None] * a[None, :] - h[None, :]))), axis=1)
    jf = int(np.argmax(Rf))
    return float(Rf[jf]), float(fine[jf])


def camera_turn_features(g, sid, pairs, dh, rng, bias=0.0):
    """A turning stream between two drive stretches: its change (an angle) or its integral (a rate) times some
    unit factor k should equal the camera's heading change. R near 1 = follows; R_null = the same scan against
    shuffled heading changes, so 'excess' is how much better than chance."""
    out = {"cam_turn_n": len(pairs)}
    if len(pairs) < 12:
        return out
    y = g.v[sid]
    dA = np.array([np.nanmedian(y[b[0]:b[0] + 3]) - np.nanmedian(y[max(0, a[1] - 2):a[1] + 1]) for a, b in pairs])
    # a rate's level at rest (its bias) is taken off before integrating
    b0 = bias if np.isfinite(bias) else 0.0
    iA = np.array([np.nansum(y[a[1]:b[0] + 1] - b0) * g.dt for a, b in pairs])
    perm = rng.permutation(len(dh))
    for nm, arr in (("ang", dA), ("int", iA)):
        R, k = _axial_scan(arr, dh)
        R0, _ = _axial_scan(arr, dh[perm])
        out["cam_turn_%s_R" % nm] = R
        out["cam_turn_%s_k" % nm] = k
        out["cam_turn_%s_excess" % nm] = R - R0 if np.isfinite(R) and np.isfinite(R0) else np.nan
    return out


def find_clock(parsed, t, seg):
    """The stream that is the robot's own clock: never falls inside a segment, rises in proportion to the laptop's
    arrival times (median r >= 0.999), and does not step by a fixed amount each row (that is a counter). Returns
    (stream id, robot-clock time per row aligned to the laptop clock) or (None, t)."""
    best = None
    for sid, (x, is_text) in parsed.items():
        if is_text or not np.all(np.isfinite(x)):
            continue
        d = np.diff(x)
        same = np.diff(seg) == 0
        if len(d) < 50 or np.any(d[same] < 0) or np.mean(d[same] > 0) < 0.95:
            continue
        pos = d[same][d[same] > 0]
        if np.mean(np.isclose(pos, np.median(pos))) > 0.9:
            continue        # a counter
        rs = []
        for s in np.unique(seg):
            k = seg == s
            if k.sum() > 20:
                rs.append(np.corrcoef(t[k], x[k])[0, 1])
        r = np.median(rs) if rs else 0
        if r >= 0.999 and (best is None or r > best[0]):
            best = (r, sid)
    if best is None:
        return None, t
    sid = best[1]
    x = parsed[sid][0]
    tt = t.copy()
    for s in np.unique(seg):
        k = np.where(seg == s)[0]
        if len(k) < 20:
            continue
        rate = (x[k[-1]] - x[k[0]]) / max(1e-9, t[k[-1]] - t[k[0]])     # clock units per second
        if rate <= 0:
            continue
        tc = x[k] / rate
        # arrival = send + a delay that is never negative: align on the low edge of (arrival - send)
        off = np.percentile(t[k] - tc, 5)
        tt[k] = tc + off
    return sid, tt


def episode_features(g, sid, eps):
    """Over the camera's drive stretches: how far the stream moved per cm the camera saw (a wheel angle or a
    position), and its typical size per cm/s of camera speed (a speed). Magnitudes only: the camera does not say
    whether a stretch was driven forwards or backwards."""
    out = {}
    if len(eps) < 4:
        return out
    y = g.v[sid]
    per_cm, per_speed = [], []
    for i0, i1, head, dist in eps:
        a, b = y[i0], y[i1]
        if np.isfinite(a) and np.isfinite(b) and abs(b - a) > 0:
            per_cm.append(abs(b - a) / dist)
        # a speed's integral over the stretch itself. (Until 2026-10-25 this took 1 s either side as well, on the
        # idea that a speed is zero outside a drive; the wheels also turn in those margins (speeding up, slowing
        # down, the turn before), which read the wheel speed 11% low: build-reports/e02-oct25-right-front-and-rpm.md)
        integ = np.nansum(np.abs(y[i0:i1 + 1])) * g.dt
        if np.isfinite(integ) and integ > 0:
            per_speed.append(integ / dist)
    if len(per_cm) >= 4:
        out["ep_per_cm"] = float(np.median(per_cm))
        out["ep_per_cm_spread"] = float(np.subtract(*np.percentile(per_cm, [75, 25])) / max(np.median(per_cm), 1e-12))
    if len(per_speed) >= 4:
        out["ep_per_cms"] = float(np.median(per_speed))
        out["ep_per_cms_spread"] = float(np.subtract(*np.percentile(per_speed, [75, 25])) /
                                         max(np.median(per_speed), 1e-12))
    out["ep_n"] = len(per_cm)
    return out


def case_features(case):
    """(Grid, {stream id: features}, cam_speed, {stream id: numeric row values})."""
    t0 = case.t
    seg = segments(t0)
    parsed = {sid: parse(vals) for sid, vals in case.streams.items()}
    clock_sid, t = find_clock(parsed, t0, seg)
    num, feats = {}, {}
    for sid, vals in case.streams.items():
        x, is_text = parsed[sid]
        f = passive(x, t, seg, is_text)
        f["is_time_base"] = sid == clock_sid
        if not is_text and not f.get("const"):
            wr, per = detect_wrap(x, seg)
            f["wrapped"] = bool(wr)
            if wr:
                f["wrap_period"] = per
                x = unwrap(x, seg, per)
            num[sid] = x
        feats[sid] = f
    g = make_grid(t, num, seg)
    case.t_used = t
    cam_speed = camera_series(case, g)
    eps = camera_episodes(case, g, cam_speed)
    pairs, dh = camera_turn_pairs(g, eps)
    rng = np.random.default_rng(7)
    for sid in num:
        feats[sid].update(leg_features(g, case.legs, sid))
        feats[sid].update(camera_features(case, g, sid, cam_speed))
        feats[sid].update(camera_turn_features(g, sid, pairs, dh, rng, bias=feats[sid].get("rest_level", 0.0)))
        feats[sid].update(episode_features(g, sid, eps))
        feats[sid]["cam_episodes"] = len(eps)
    return g, feats, cam_speed, num
