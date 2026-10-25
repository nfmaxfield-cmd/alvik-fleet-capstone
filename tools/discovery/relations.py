"""Relations between anonymous streams: which update together, which is the integral of which, which are copies or
combinations of others, and which pair is a planar position driven along a heading.

All tests are correlations on 2 s windows inside logging segments, so a stream's units, scale and sign do not
matter. Thresholds are chosen numbers (see EXACT_R and the DOE backlog)."""
import numpy as np

from . import features as F

EXACT_R = 0.98         # |correlation| counted as "one is an exact function of the other" (chosen)
AFFINITY_GROUP = 0.60  # co-update affinity that puts two streams in one sensor group (chosen)
PLANAR_R = 0.90        # circular agreement of a position pair's direction with a heading stream (chosen)
UNIT_FACTORS = (1.0, np.pi / 180.0, 180.0 / np.pi)   # same units, deg->rad, rad->deg


def windows(g, w_s=F.WIN_S):
    W = int(round(w_s / g.dt))
    starts, i, N = [], 0, len(g.t)
    while i + W < N:
        if g.seg[i] == g.seg[i + W]:
            starts.append(i)
            i += W
        else:
            i += 1
    return np.array(starts, dtype=int), W


def window_views(g, ids):
    """V (mean), D (end - start), S (integral) per window, as (n_windows, n_streams) arrays."""
    st, W = windows(g)
    if len(st) == 0:
        z = np.zeros((0, len(ids)))
        return z, z, z
    M = np.vstack([g.v[s] for s in ids]).T          # (N, k)
    idx = st[:, None] + np.arange(W)[None, :]
    blk = M[idx]                                     # (nw, W, k)
    V = np.nanmean(blk, axis=1)
    D = M[st + W] - M[st]
    S = np.nansum(blk, axis=1) * g.dt
    return V, D, S


def corr_matrix(A, B):
    """Pearson correlation of every column of A with every column of B, ignoring rows with NaN in either."""
    ok = np.all(np.isfinite(A), axis=1) & np.all(np.isfinite(B), axis=1)
    A, B = A[ok], B[ok]
    if len(A) < 8:
        return np.full((A.shape[1], B.shape[1]), np.nan)
    sa, sb = A.std(0), B.std(0)
    A = (A - A.mean(0)) / np.where(sa > 0, sa, np.nan)
    B = (B - B.mean(0)) / np.where(sb > 0, sb, np.nan)
    return (A.T @ B) / len(A)


def coupdate_affinity(num, seg, ids, feats, rows=None, min_changes=30):
    """How strongly two streams change at the same rows, above chance. max(Jaccard, containment above chance) on
    rows where each changes. Streams that change on (almost) every row carry no timing information and get 0."""
    ch = {}
    same = np.r_[False, np.diff(seg) == 0]
    for s in ids:
        x = num[s]
        c = np.r_[False, np.diff(x) != 0] & same & np.r_[False, np.isfinite(np.diff(x))]
        if rows is not None:
            c = c & rows
        ch[s] = c
    n = max(1, (same & rows).sum() if rows is not None else same.sum())
    P = {s: ch[s].sum() / n for s in ids}
    A = np.zeros((len(ids), len(ids)))
    for i, a in enumerate(ids):
        for j, b in enumerate(ids):
            if j <= i:
                continue
            pa, pb = P[a], P[b]
            if min(ch[a].sum(), ch[b].sum()) < min_changes or max(pa, pb) > 0.97:
                continue
            inter = (ch[a] & ch[b]).sum()
            union = (ch[a] | ch[b]).sum()
            jac = inter / union if union else 0.0
            rare, common = (a, b) if pa <= pb else (b, a)
            cont = inter / max(1, ch[rare].sum())
            pc = P[common]
            above = (cont - pc) / (1 - pc) if pc < 1 else 0.0
            A[i, j] = A[j, i] = max(jac, above)
    np.fill_diagonal(A, 1.0)
    return A


def groups_from_affinity(A, ids, thr=AFFINITY_GROUP):
    """Average-linkage clustering: merge the two clusters with the highest mean affinity while it is >= thr."""
    cl = [[i] for i in range(len(ids))]
    while True:
        best, bi, bj = thr, -1, -1
        for i in range(len(cl)):
            for j in range(i + 1, len(cl)):
                m = np.mean(A[np.ix_(cl[i], cl[j])])
                if m >= best:
                    best, bi, bj = m, i, j
        if bi < 0:
            break
        cl[bi] = cl[bi] + cl[bj]
        del cl[bj]
    return [[ids[k] for k in c] for c in cl]


def _r2_two(C, t, p, q):
    """R^2 of column t on columns p, q from a correlation matrix C."""
    Rxx = np.array([[1.0, C[p, q]], [C[q, p], 1.0]])
    r = np.array([C[t, p], C[t, q]])
    if not np.all(np.isfinite(Rxx)) or not np.all(np.isfinite(r)):
        return np.nan
    try:
        return float(r @ np.linalg.solve(Rxx, r))
    except np.linalg.LinAlgError:
        return np.nan


def planar_test(g, ids, D, cand_xy, cand_h):
    """For position pairs (X, Y) and heading streams H: does the direction of each window's (dX, dY) follow H?
    Returns list of (R, X, Y, H, unit factor, sign)."""
    st, W = windows(g)
    out = []
    if len(st) < 20:
        return out
    col = {s: i for i, s in enumerate(ids)}
    for xa in cand_xy:
        for ya in cand_xy:
            if xa == ya:
                continue
            dx, dy = D[:, col[xa]], D[:, col[ya]]
            mag = np.hypot(dx, dy)
            ok = np.isfinite(mag) & (mag > np.nanpercentile(mag, 60)) & (mag > 0)
            if ok.sum() < 15:
                continue
            psi = np.arctan2(dy[ok], dx[ok])
            for h in cand_h:
                if h in (xa, ya):
                    continue
                hv = g.v[h]
                mid = hv[st + W // 2][ok]
                if not np.all(np.isfinite(mid)):
                    fm = np.isfinite(mid)
                    if fm.sum() < 15:
                        continue
                else:
                    fm = np.ones(len(mid), bool)
                for u in UNIT_FACTORS:
                    for sg in (1.0, -1.0):
                        # axial (doubled angles): a backwards drive points the other way along the same line
                        z = np.exp(2j * (psi[fm] - sg * u * mid[fm]))
                        R = float(np.abs(np.mean(z)))
                        out.append((R, xa, ya, h, u, sg))
    out.sort(key=lambda r: -r[0])
    return out


def _halves(n, block=8):
    """Two interleaved halves of the windows (blocks of `block` windows alternate), so a relation is checked on
    data it was not found on."""
    h = (np.arange(n) // block) % 2
    return h == 0, h == 1


def _both(MA, MB):
    """Agreement of two halves' correlation matrices: the smaller magnitude, kept only when the signs agree."""
    same = np.sign(MA) == np.sign(MB)
    out = np.where(same, np.sign(MA) * np.minimum(np.abs(MA), np.abs(MB)), 0.0)
    return np.where(np.isfinite(out), out, 0.0)


def _holdout_r2(y, X, h0, h1):
    """Fit y ~ X (+ constant) on one half, R^2 on the other, both ways; the smaller."""
    out = []
    for a, b in ((h0, h1), (h1, h0)):
        ka = a & np.isfinite(y) & np.all(np.isfinite(X), axis=1)
        kb = b & np.isfinite(y) & np.all(np.isfinite(X), axis=1)
        if ka.sum() < 8 or kb.sum() < 8:
            return np.nan
        A = np.column_stack([X[ka], np.ones(ka.sum())])
        sol, *_ = np.linalg.lstsq(A, y[ka], rcond=None)
        pred = np.column_stack([X[kb], np.ones(kb.sum())]) @ sol
        sst = np.sum((y[kb] - y[kb].mean()) ** 2)
        if sst <= 0:
            return np.nan
        out.append(1 - np.sum((y[kb] - pred) ** 2) / sst)
    return float(min(out))


def motion_streams(feats, ids):
    """Streams that sit exactly flat for long stretches and change otherwise (wheel encoders, a computed pose):
    when all of them are flat, the robot is standing still."""
    return [s for s in ids if 0.08 <= (feats[s].get("flat_frac") or 0) <= 0.98 and (feats[s].get("change_hz") or 0) > 0.5]


def still_rows(feats, ids, n, min_streams=2):
    ms = motion_streams(feats, ids)
    if len(ms) < min_streams:
        return None, ms
    still = np.ones(n, bool)
    for s in ms:
        still &= feats[s]["_flat"]
    return still, ms


def analyse(case, g, feats, num):
    """All relations for a case. Returns dict with groups, pairs and per-stream relation summaries."""
    seg = F.segments(case.t)
    ids = [s for s in num if not feats[s].get("const") and not feats[s].get("text")]
    # streams that rise steadily with time (clocks, counters) are left out of the motion relations
    motion_ids = [s for s in ids if not (feats[s].get("monotone_up", 0) > 0.999 and
                                         feats[s].get("time_corr_in_seg", 0) > 0.99)]
    accum = {s: _accumulates(g, s) and (feats[s].get("smooth") or 1) < 0.1 for s in motion_ids}
    still, mstreams = still_rows(feats, motion_ids, len(seg))
    A_all = coupdate_affinity(num, seg, motion_ids, feats)
    if still is not None and still.sum() > 200:
        # groups from what flickers together at rest; streams that are flat at rest stay on their own
        A = coupdate_affinity(num, seg, motion_ids, feats, rows=still)
        grouping = "rest"
    else:
        A = np.where(A_all >= 0.85, A_all, 0.0)
        grouping = "motion"
    np.fill_diagonal(A, 1.0)
    grp = groups_from_affinity(A, motion_ids)
    group_of = {s: gi for gi, gr in enumerate(grp) for s in gr}
    aff = {(a, b): A[i, j] for i, a in enumerate(motion_ids) for j, b in enumerate(motion_ids)}
    for s in motion_ids:
        feats[s]["accumulates"] = accum[s]
        if still is not None and still.sum() > 50:
            x = num[s][still]
            d = np.diff(x)
            feats[s]["still_change_frac"] = float(np.mean(d[np.isfinite(d)] != 0)) if len(d) else np.nan
            feats[s]["still_sd"] = float(np.nanstd(x)) if len(x) else np.nan
        else:
            feats[s]["still_change_frac"] = np.nan

    V, D, S = window_views(g, motion_ids)
    rel = {s: {"accumulates": accum[s]} for s in motion_ids}
    res = {"ids": motion_ids, "groups": grp, "group_of": group_of, "affinity": aff, "rel": rel,
           "integration": [], "copies": [], "combos": [], "planar": [], "n_windows": len(D),
           "still_rows": int(still.sum()) if still is not None else 0, "motion_streams": mstreams, "grouping": grouping,
           "moving_frac": float(1 - still.mean()) if still is not None else np.nan}
    if len(D) < 16:
        return res
    h0, h1 = _halves(len(D))
    DS = _both(corr_matrix(D[h0], S[h0]), corr_matrix(D[h1], S[h1]))    # row's change vs column's integral
    DD = _both(corr_matrix(D[h0], D[h0]), corr_matrix(D[h1], D[h1]))
    VV = _both(corr_matrix(V[h0], V[h0]), corr_matrix(V[h1], V[h1]))
    active = np.sum(np.isfinite(D) & (np.abs(D) > 0), axis=0)
    k = len(motion_ids)
    for i, a in enumerate(motion_ids):
        r = rel[a]
        r["active_windows"] = int(active[i])
        best = (0.0, None)
        for j, b in enumerate(motion_ids):
            if i == j or active[j] < 12:
                continue
            v = DS[i, j]
            if abs(v) > abs(best[0]):
                best = (float(v), b)
        r["int_of"], r["int_r"] = best[1], best[0]
        best = (0.0, None)
        for j, b in enumerate(motion_ids):
            if i == j or active[j] < 12:
                continue
            v = DD[i, j]
            if not accum[a] and not accum[b]:
                v = np.sign(v) * min(abs(v), abs(VV[i, j])) if np.sign(v) == np.sign(VV[i, j]) else 0.0
            if abs(v) > abs(best[0]):
                best = (float(v), b)
        r["copy_of"], r["copy_r"] = best[1], best[0]
        r["vv"] = {motion_ids[j]: float(VV[i, j]) for j in range(k) if j != i}
        r["dd"] = {motion_ids[j]: float(DD[i, j]) for j in range(k) if j != i}
        r["ds"] = {motion_ids[j]: float(DS[i, j]) for j in range(k) if j != i}
    for a in motion_ids:
        if rel[a]["active_windows"] < 12:
            continue
        b = rel[a]["int_of"]
        if b is not None and abs(rel[a]["int_r"]) >= EXACT_R:
            res["integration"].append((a, b, rel[a]["int_r"]))
        b = rel[a]["copy_of"]
        if b is not None and abs(rel[a]["copy_r"]) >= EXACT_R and a < b and rel[b].get("copy_of") == a:
            res["copies"].append((a, b, rel[a]["copy_r"]))
    for a, b, r in res["integration"]:
        accum[a] = True
        rel[a]["accumulates"] = True
        feats[a]["accumulates"] = True
    # combinations of two, checked on held-out windows
    for i, a in enumerate(motion_ids):
        rel[a]["combo_r2"], rel[a]["combo_of"] = 0.0, (None, None)
        if rel[a]["active_windows"] < 12:
            continue
        single = max(abs(rel[a]["copy_r"]), abs(rel[a]["int_r"]))
        M = D if accum[a] else V
        y = M[:, i]
        cands = [j for j in range(k) if j != i and active[j] >= 12 and
                 abs((rel[a]["dd"] if accum[a] else rel[a]["vv"]).get(motion_ids[j], 0)) >= 0.2]
        best = (0.0, None, None)
        for x1 in range(len(cands)):
            for x2 in range(x1 + 1, len(cands)):
                p, q = cands[x1], cands[x2]
                Mp = D[:, p] if accum[a] or accum[motion_ids[p]] else V[:, p]
                Mq = D[:, q] if accum[a] or accum[motion_ids[q]] else V[:, q]
                if not accum[a]:
                    Mp, Mq = V[:, p], V[:, q]
                v = _holdout_r2(y, np.column_stack([Mp, Mq]), h0, h1)
                if np.isfinite(v) and v > best[0]:
                    best = (v, motion_ids[p], motion_ids[q])
        rel[a]["combo_r2"], rel[a]["combo_of"] = best[0], (best[1], best[2])
        if best[0] >= EXACT_R ** 2 and best[0] - single ** 2 > 0.02:
            res["combos"].append((a, best[1], best[2], best[0]))
    # planar position pairs: running sums that are not the integral of anything
    cand_xy = [s for s in motion_ids if (accum[s] or s in mstreams) and abs(rel[s]["int_r"]) < EXACT_R]
    cand_h = sorted(set([a for a, b, r in res["integration"]]))
    pl = planar_test(g, motion_ids, D, cand_xy, cand_h) if cand_xy and cand_h else []
    res["planar"] = [p for p in pl if p[0] >= PLANAR_R][:6]
    res["planar_best"] = pl[0] if pl else None
    return res


def _accumulates(g, s, short_s=2.0, long_s=20.0):
    """True when a stream is a running sum (an angle or a position): its changes over a minute are much bigger
    than over 2 s, where a bounded signal (a rate, a distance reading) stops growing. Variance-ratio test."""
    y = g.v[s]
    out = []
    for T in (short_s, long_s):
        n = int(round(T / g.dt))
        if len(y) <= n:
            return False
        d = y[n:] - y[:-n]
        ok = (g.seg[n:] == g.seg[:-n]) & np.isfinite(d)
        d = np.abs(d[ok])
        if len(d) < 20:
            return False
        out.append(np.percentile(d, 90))
    if out[0] <= 0:
        return False
    return bool(out[1] > 4.0 * out[0])
