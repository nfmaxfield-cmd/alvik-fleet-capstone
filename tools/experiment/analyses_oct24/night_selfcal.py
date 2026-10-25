"""Analysis B (2026-10-24 night): would the self-calibration's stretches agree better with the wheels if the camera
were corrected by its error map first, and does the map cover where the stretches are? Then the alternatives.

Everything is rebuilt offline from track.csv + raw.csv (night_common.rebuild_stretches reproduces the runner's own
stretches: same counts, lengths within 0.01 cm). The score is the one the runner gates on: each robot's pooled fit
(e02_fit.fit_robot over all its U-run stretches, Huber-weighted rms, cm per stretch) against 1.5 cm, plus the
robust sd of camera - wheel_scale x counted over all robots' stretches.

  B1 coverage: how far each stretch end is from the finds, the map's kernel weight there (corrected only where the
     weight is >= MIN_WEIGHT), the map's own sd there.
  B2 the camera corrected by the map (all of the session's finds) before the stretches.
  B3 alternative 1, the camera's scale across the board from the calibration file (roam1, session 1): a picture
     shift that reproduces the error at the finds, pushed through the homography, and what it does to lengths; and
     a shared camera-scale model (direction, place and distance-from-the-picture's-middle terms, one wheel scale
     per robot) fitted to the stretches, scored leave-one-run-out.
  B4 alternative 2, time: the camera's times shifted by -0.3..+0.3 s against the robots' before pairing.
  B5 alternative 3, the tracker: stretches near the boards (the find corner, where the 2026-10-23 quality map
     found the floor brightest and the lock-on weakest) and stretches with a jump in the camera track.

Usage: python3 night_selfcal.py"""
import json
import math
import sys

import numpy as np

import night_common as N
from night_common import FIT, TR, CE

SESS = ("oct23-morning", "oct23-evening", "stage1", "stage2")
NEAR_BOARD_CM = 15.0     # chosen: a stretch end this close to a board line counts as "near the boards"
JUMP_CM = 3.0            # chosen: one picture moving more than this is a tracker jump (drives are 8-10 cm/s, 0.4-0.8 cm
                         # a picture at 13-21 pictures a second); B6 sweeps 2 / 3 / 4 cm


def pooled(rb, gate=1.5):
    """Per robot: fit_robot's wheel rms (cm), wheel scale, se, pass at gate; and all residuals."""
    out, res = {}, []
    for r, obs in rb.items():
        f = FIT.fit_robot(obs, None, max_rms_cm=gate)["fitted"]
        wf = FIT.fit_wheels([o for o in obs if o.get("bel_len") and o.get("cam_len")])
        if wf is None:
            continue
        out[r] = dict(n=len(obs), rms=wf["rms_cm"], w=wf["w"], se=wf["se_w"], ok=wf["rms_cm"] <= gate)
        res += [o["cam_len"] - wf["w"] * o["bel_len"] for o in obs]
        out[r]["rms_deg"] = f.get("rms_deg")
    return out, res


def line(lab, rb):
    p, res = pooled(rb)
    cells = " | ".join("%s %.2f (n %d)%s" % (r[0], p[r]["rms"], p[r]["n"], "" if p[r]["ok"] else " x")
                       for r in N.ROBOTS if r in p)
    return "| %s | %s | %.2f |" % (lab, cells, TR.robust_sd(res) if res else float("nan"))


def calib(name):
    with open(N.path(name, "floor_calibration_for_blink.json")) as f:
        c = json.load(f)
    return np.array(c["H_floor_to_px"], float), c


def to_px(H, x, y):
    v = H @ np.array([x, y, 1.0])
    return v[:2] / v[2]


def to_floor(H, u, v):
    q = np.linalg.solve(H, np.array([u, v, 1.0]))
    return q[:2] / q[2]


def b1_coverage(name, rb):
    st = N.state(name)
    smp = [s for s in st.get("camera_error_samples") or [] if s["source"] == "find"]
    m = CE.CameraErrorMap(smp)
    ends = [p for obs in rb.values() for o in obs for p in (o["p0"], o["p1"])]
    fx = np.median([s["x"] for s in smp]), np.median([s["y"] for s in smp])
    spread = max(math.hypot(s["x"] - fx[0], s["y"] - fx[1]) for s in smp)
    d = np.array([math.hypot(p[0] - fx[0], p[1] - fx[1]) for p in ends])
    corr = [m.correction(p[0], p[1]) for p in ends]
    applied = np.array([c[2] < CE.NO_DATA_SD_CM for c in corr])
    sds = np.array([c[2] for c in corr])
    dxy = np.array([[c[0], c[1]] for c in corr])
    print("\n#### B1 %s: %d finds, all within %.1f cm of (%.1f, %.1f); %d stretch ends" % (
        name, len(smp), spread, fx[0], fx[1], len(ends)))
    print("| distance of a stretch end from the finds | share of ends | map applied | map sd there (median) |")
    for lo, hi in ((0, 25), (25, 50), (50, 75), (75, 200)):
        k = (d >= lo) & (d < hi)
        if k.sum():
            print("| %d-%d cm | %.0f%% | %.0f%% | %.1f cm |" % (lo, hi, 100 * k.mean(), 100 * applied[k].mean(),
                                                               np.median(sds[k])))
    print("correction applied: dx %.2f..%.2f, dy %.2f..%.2f (the same everywhere it is applied)" % (
        dxy[applied, 0].min(), dxy[applied, 0].max(), dxy[applied, 1].min(), dxy[applied, 1].max()))
    return m


def b2_corrected(name, m, rb):
    def fn(t, x, y):
        cx, cy, _ = m.corrected(x, y)
        return cx, cy
    rbc = N.rebuild_stretches(name, cam_fn=fn)
    # the same stretches (start within 0.05 s) before and after: how much did their lengths change, and which
    # stretches crossed the edge of the map (where the correction drops to nothing)?
    dl, cross, n_same = [], 0, 0
    for r, lst in rbc.items():
        old = {round(o["t0"], 1): o for o in rb.get(r, [])}
        for o in lst:
            q = old.get(round(o["t0"], 1))
            if q is None:
                continue
            n_same += 1
            dl.append(abs(o["cam_len"] - q["cam_len"]))
            a = m.correction(*q["p0"])[2] < CE.NO_DATA_SD_CM
            b = m.correction(*q["p1"])[2] < CE.NO_DATA_SD_CM
            cross += a != b
    nb = sum(len(v) for v in rb.values())
    nc = sum(len(v) for v in rbc.values())
    print("same stretch before and after: %d of %d / %d; their length change median %.3f cm, worst %.2f; %d cross "
          "the map's edge (a %.1f cm step there)" % (n_same, nb, nc, float(np.median(dl)) if dl else 0,
                                                     max(dl) if dl else 0, cross,
                                                     math.hypot(*m.correction(113.8, 107.8)[:2])))
    return rbc


def b3_shift(name, rb):
    """A picture shift (du, dv px) that reproduces the finds' error, through the homography: the length change."""
    H, c = calib(name)
    smp = [s for s in N.state(name).get("camera_error_samples") or [] if s["source"] == "find"]
    tx, ty = np.median([s["x"] for s in smp]), np.median([s["y"] for s in smp])
    ex, ey = np.median([s["dx"] for s in smp]), np.median([s["dy"] for s in smp])
    u0 = to_px(H, tx, ty)
    J = np.zeros((2, 2))
    for j, (ddx, ddy) in enumerate(((0.5, 0), (0, 0.5))):
        J[:, j] = (to_px(H, tx + ddx, ty + ddy) - u0) / 0.5
    d = J @ np.array([ex, ey])           # px shift giving that floor error at the finds
    f = lambda p: to_floor(H, *(to_px(H, p[0], p[1]) + d))
    dl = []
    for obs in rb.values():
        for o in obs:
            a, b = np.array(o["p0"]), np.array(o["p1"])
            # the camera saw f(true); "true" here = the camera's position less the shift's effect (close enough)
            dl.append(np.hypot(*(f(b) - f(a))) - np.hypot(*(b - a)))
    cmpx = 1.0 / np.linalg.norm(J, 2)
    print("\n#### B3a %s: the finds' error (%.2f, %.2f) cm = a picture shift of (%.2f, %.2f) px (%.2f cm a px "
          "there); through the calibration it changes stretch lengths by %.3f cm median, %.3f worst" % (
              name, ex, ey, d[0], d[1], cmpx, float(np.median(np.abs(dl))), float(np.max(np.abs(dl)))))
    # the calibration's own scale across the board: cm per px at the board's corners and middle
    pts = [(10, 10), (61, 61), (112, 112), (10, 112), (112, 10), (tx, ty)]
    sc = []
    for x, y in pts:
        a = to_px(H, x, y)
        sc.append("(%.0f,%.0f) %.3f" % (x, y, 1.0 / np.hypot(*(to_px(H, x + 1, y) - a))))
    print("calibration's cm per px along x at:", ", ".join(sc))
    return d


def design(o, H):
    """Shared camera-scale terms for one stretch: c - w b = b (a1 cos2phi + a2 sin2phi + g1 X + g2 Y + k R2)."""
    a, b = np.array(o["p0"]), np.array(o["p1"])
    m = 0.5 * (a + b)
    ph = math.atan2(b[1] - a[1], b[0] - a[0])
    u = to_px(H, m[0], m[1])
    r2 = ((u[0] - 320) ** 2 + (u[1] - 240) ** 2) / 400.0 ** 2
    bl = o["bel_len"]
    return [bl * math.cos(2 * ph), bl * math.sin(2 * ph), bl * (m[0] - 61) / 61.0, bl * (m[1] - 61) / 61.0, bl * r2]


def scale_model(name, rb, terms=True):
    """Leave-one-run-out: per-robot wheel scale (+ shared camera terms) fitted without the run, residuals of the
    run's stretches predicted. Huber IRLS (k = 1.5 x 0.5 cm, as e02_fit). Returns (rms of predictions, robust sd,
    coefficients fitted to all)."""
    H, _ = calib(name)
    obs = [(r, o) for r, lst in rb.items() for o in lst]
    robots = sorted(rb)

    def fit(sub):
        X, y = [], []
        for r, o in sub:
            row = [o["bel_len"] if r == q else 0.0 for q in robots]
            if terms:
                row += design(o, H)
            X.append(row)
            y.append(o["cam_len"])
        X, y = np.array(X), np.array(y)
        w = np.ones(len(y))
        for _ in range(30):
            sw = np.sqrt(w)
            beta, *_ = np.linalg.lstsq(X * sw[:, None], y * sw, rcond=None)
            res = y - X @ beta
            w = FIT._huber_w(res, 1.5 * FIT.SIGMA_CM)
        return beta

    def pred(beta, r, o):
        row = [o["bel_len"] if r == q else 0.0 for q in robots]
        if terms:
            row += design(o, H)
        return float(np.dot(beta, row))

    runs = sorted({o["run"] for _, o in obs})
    res = []
    for run in runs:
        tr = [(r, o) for r, o in obs if o["run"] != run]
        te = [(r, o) for r, o in obs if o["run"] == run]
        if not te or len({r for r, _ in tr}) < len(robots):
            continue
        beta = fit(tr)
        res += [o["cam_len"] - pred(beta, r, o) for r, o in te]
    res = np.array(res)
    return float(np.sqrt(np.mean(res ** 2))), TR.robust_sd(res), fit(obs)


def b4_time(name):
    print("\n#### B4 %s: camera times shifted against the robots' before pairing" % name)
    print("| shift s (camera earlier +) | " + " | ".join(N.ROBOTS) + " | robust sd all |")
    for sh in (-0.3, -0.2, -0.1, -0.05, 0.0, 0.05, 0.1, 0.2, 0.3):
        rb = N.rebuild_stretches(name, time_shift=sh)
        p, res = pooled(rb)
        print("| %+.2f | %s | %.2f |" % (sh, " | ".join("%.2f" % p[r]["rms"] if r in p else "-" for r in N.ROBOTS),
                                          TR.robust_sd(res)))


def tag_stretches(name, rb):
    """near the boards (either end within NEAR_BOARD_CM of a board line) / a jump in the camera track inside it."""
    b = N.state(name)["board"]
    tr = N.track(name)
    for r, lst in rb.items():
        cam = tr.get(r, [])
        ts = np.array([q[0] for q in cam])
        for o in lst:
            near = any(abs(p[0] - b["yboard_cm"]) < NEAR_BOARD_CM or abs(p[1] - b["xboard_cm"]) < NEAR_BOARD_CM
                       for p in (o["p0"], o["p1"]))
            i0, i1 = np.searchsorted(ts, o["t0"]), np.searchsorted(ts, o["t1"])
            jump = False
            for i in range(max(1, i0), min(len(cam), i1 + 1)):
                if math.hypot(cam[i][1] - cam[i - 1][1], cam[i][2] - cam[i - 1][2]) > JUMP_CM:
                    jump = True
                    break
            o["near"], o["jump"] = near, jump


def b5_tracker(name, rb):
    tag_stretches(name, rb)
    p, _ = pooled(rb)
    print("\n#### B5 %s: stretches by where they are and whether the camera track jumps inside them" % name)
    print("| group | stretches | share | robust sd of camera - w x counted, cm | median length cm |")
    allo = [(r, o) for r, lst in rb.items() for o in lst if r in p]
    for lab, k in (("open board, no jump", lambda o: not o["near"] and not o["jump"]),
                   ("near the boards, no jump", lambda o: o["near"] and not o["jump"]),
                   ("a jump in the track", lambda o: o["jump"]), ("all", lambda o: True)):
        sel = [(r, o) for r, o in allo if k(o)]
        if not sel:
            continue
        res = [o["cam_len"] - p[r]["w"] * o["bel_len"] for r, o in sel]
        print("| %s | %d | %.0f%% | %.2f | %.0f |" % (lab, len(sel), 100.0 * len(sel) / len(allo), TR.robust_sd(res),
                                                      N.median([o["cam_len"] for _, o in sel])))
    clean = {r: [o for o in lst if not o["near"] and not o["jump"]] for r, lst in rb.items()}
    return clean


def main():
    print("Scores: each robot's pooled wheel fit rms, cm per stretch (x = above the 1.5 cm gate); last column the "
          "robust sd of all robots' stretches (cm)")
    out = {}
    for name in SESS:
        rb = N.rebuild_stretches(name)
        out[name] = rb
    print("\n#### Baseline (as the runner fitted)")
    print("| session | robots | robust sd |")
    for name in SESS:
        print(line(name, out[name]))
    for name in ("stage1", "stage2"):
        m = b1_coverage(name, out[name])
        rbc = b2_corrected(name, m, out[name])
        print("\n#### B2 %s: the camera corrected by the error map (all the session's finds) before the stretches"
              % name)
        print(line(name + " as run", out[name]))
        print(line(name + " map-corrected", rbc))
        b3_shift(name, out[name])
    print("\n#### B3b shared camera-scale model, leave one run out (rms / robust sd of the predicted run's "
          "stretches, cm)")
    print("| session | wheel scale per robot only | + shared camera terms | terms fitted to all (a1 cos2phi, a2 sin2phi,"
          " g1 x, g2 y, k r2; shares of length) |")
    for name in SESS:
        a = scale_model(name, out[name], terms=False)
        b = scale_model(name, out[name], terms=True)
        print("| %s | %.2f / %.2f | %.2f / %.2f | %s |" % (name, a[0], a[1], b[0], b[1],
                                                         ", ".join("%+.3f" % v for v in b[2][-5:])))
    for name in ("oct23-morning", "stage2"):
        b4_time(name)
    print("\n#### B5 where the misses are")
    for name in SESS:
        b5_tracker(name, out[name])
    b6_split()


def accept_run_by_run(name, split, gate=1.5):
    """The runner's sequence again (fit_robot after every U run, with the prior and the measured drift), the
    stretches found with the camera track cut at jumps: which wheel fits pass at the gate."""
    st = N.state(name)
    rb = N.rebuild_stretches(name, split_jump_cm=split)
    by = {}
    for r, lst in rb.items():
        for o in lst:
            by.setdefault((r, o["order"]), []).append(o)
    data, prev, out = {}, {}, []
    for robot, order, t0, t1, runs in N.u_runs(name):
        p = dict(FIT.PRIOR)
        d = ((st.get("naming_check") or {}).get("gyro_raw_drift_dps") or {}).get(robot)
        if d is not None:
            p["gyro_drift_dps"] = round(p["gyro_scale"] * d, 4)
        lst = data.setdefault(robot, [])
        lst += by.get((robot, order), [])
        res = FIT.fit_robot(lst, prev.get(robot), prior=p, prior_sd=dict(gyro_drift_dps=0.1) if d is not None else None,
                            measured_drift=p["gyro_drift_dps"] if d is not None else None, max_rms_cm=gate)
        if res["values"] and res["accepted"]:
            prev[robot] = res["values"]
        f = res["fitted"]
        out.append((N.hm(t1), robot, order, len(lst), f.get("rms_cm"), f.get("wheel_scale"), res["wheel_ok"],
                    res["gyro_ok"], [x for x in res["reasons"] if "wheel" in x]))
    return out, prev


def b6_split():
    print("\n#### B6 the camera track cut at jumps before the stretches are found")
    print("| session | jump cut | Left rms cm (n, cm counted) | Center | Right | robust sd all | gyro rms deg L/C/R |")
    for name in SESS:
        for j in (None, 4.0, 3.0, 2.0):
            rb = N.rebuild_stretches(name, split_jump_cm=j)
            cells, res, degs = [], [], []
            for r in N.ROBOTS:
                obs = rb.get(r, [])
                wf = FIT.fit_wheels(obs)
                t = FIT.fit_turns(obs)
                res += [o["cam_len"] - wf["w"] * o["bel_len"] for o in obs]
                cells.append("%.2f (%d, %.0f)" % (wf["rms_cm"], wf["n"], wf["counted_cm"]))
                degs.append("%.1f" % t["rms_deg"] if t else "-")
            print("| %s | %s | %s | %.2f | %s |" % (name, "none" if j is None else "%.0f cm" % j, " | ".join(cells),
                                                    TR.robust_sd(res), "/".join(degs)))
    print("\nWheel scale per robot (pooled fit, prior 0.979 +-0.03): as run / cut at 3 cm")
    print("| session | Left | Center | Right |")
    for name in SESS:
        a = N.rebuild_stretches(name)
        b = N.rebuild_stretches(name, split_jump_cm=3.0)
        print("| %s | %s |" % (name, " | ".join("%.3f / %.3f" % (FIT.fit_wheels(a[r])["w"], FIT.fit_wheels(b[r])["w"])
                                                for r in N.ROBOTS)))
    for name in ("stage1", "stage2"):
        rows, prev = accept_run_by_run(name, 3.0)
        print("\nRun by run, %s, cut at 3 cm, gate 1.5 cm (the runner's sequence):" % name)
        print("| time | robot | run | stretches so far | rms cm | wheel scale | wheels pass | gyro pass | why not |")
        for t, r, o, n, rms, w, wok, gok, why in rows:
            print("| %s | %s | %d | %d | %s | %s | %s | %s | %s |" % (t, r, o, n, "%.2f" % rms if rms else "-",
                                                                   "%.3f" % w if w else "-", wok, gok, "; ".join(why)))
        print("values at the end:", {r: v for r, v in prev.items()})


if __name__ == "__main__":
    sys.exit(main())
