"""Factor graph smoother: re-solves a whole run at once so every reading fits
together as well as possible.

Where the Kalman filter moves forward in time and never revisits a decision,
the factor graph keeps the robot's pose at many moments ("keyframes", 20 per
second by default) as unknowns and finds the path that best agrees with:

  - the start pose (tape)
  - wheel distance between consecutive keyframes
  - heading change between keyframes from the gyro (minus its bias) and,
    less trusted, from the wheels
  - the gyro reading equals its bias whenever the wheels say the robot is still
  - camera fixes assigned to this robot by the filter's tracking rules

Each agreement is a "factor"; each disagreement is weighted by how much that
sensor is trusted. A robust (Huber) weighting stops a single wrong camera
fix from dragging the path. Adding a sensor type means adding one more kind
of factor, which is why this method scales to more sensors and robots.

Solved with scipy.optimize.least_squares (sparse).
"""
import math

import numpy as np

from ekf import Noise, D2R, wrap

try:
    from scipy.optimize import least_squares
    from scipy.sparse import lil_matrix
except ImportError:   # pragma: no cover
    least_squares = None


def smooth(tracked, kf_hz=20.0, use_gyro=True, use_wheel_w=True, use_camera=True, huber=1.5):
    """Smooth one robot's run. `tracked` is an engine.Tracked (gives the events,
    the initial guess and which camera fixes belong to this robot).
    Returns a list of (t, x, y, th) at the keyframes."""
    if least_squares is None:
        raise RuntimeError("the factor graph needs scipy: pip install scipy")
    n = Noise(**tracked.cfg["noise"])
    raw = [e for e in tracked.events if e[2] == "raw"]
    cams = [e for e in tracked.events if e[2] == "cam"] if use_camera else []
    if len(raw) < 3:
        return []
    t0, t1 = raw[0][0], raw[-1][0]
    K = max(2, int((t1 - t0) * kf_hz) + 1)
    tk = np.linspace(t0, t1, K)

    # initial guess: the filter's own estimate at each keyframe
    traj = tracked.trajectory()
    tt = np.array([p[0] for p in traj])
    guess = np.zeros(3 * K + 1)
    for k in range(K):
        j = min(int(np.searchsorted(tt, tk[k])), len(traj) - 1)
        guess[3 * k:3 * k + 3] = traj[j][1:4]
    guess[-1] = tracked.ekf.s[5]   # gyro bias

    # integrate the wheel and gyro readings between keyframes
    # Each reading holds from its own time to the next reading's. A reading
    # interval that crosses a keyframe is split between the two segments in
    # proportion to time, so no motion is lost at the boundaries.
    acc = [[0.0, 0.0, 0.0, 0.0, True] for _ in range(K - 1)]   # dist, gyro, dt, wheel turn, still
    k = 0
    for a, b in zip(raw, raw[1:]):
        ta, tb = a[0], b[0]
        if tb <= ta or a[3]["lifted"]:
            continue
        while k < K - 2 and tk[k + 1] <= ta:
            k += 1
        kk = k
        while kk < K - 1 and tk[kk] < tb:
            lo, hi = max(ta, tk[kk]), min(tb, tk[kk + 1])
            if hi > lo:
                dt = hi - lo
                s = acc[kk]
                s[0] += a[3]["v"] * dt
                s[1] += a[3]["g"] * dt
                s[2] += dt
                s[3] += a[3]["w"] * dt
                s[4] = s[4] and a[3]["still"]
            kk += 1
    segs = [(d, g, dt, w, still and dt > 0) for d, g, dt, w, still in acc]

    start = tracked.run.tape_start
    rows = []   # residual builders: (kind, k, extra)
    if start:
        rows.append(("prior", 0, start))
    for k, seg in enumerate(segs):
        rows.append(("odo", k, seg))
    cam_k = []
    for e in cams:
        k = int(round((e[0] - t0) * kf_hz))
        if 0 <= k < K:
            cam_k.append((k, e[3]))

    def residuals(p):
        out = []
        b = p[-1]
        for kind, k, x in rows:
            if kind == "prior":
                out += [(p[0] - x[0]) / 0.3, (p[1] - x[1]) / 0.3, wrap(p[2] - x[2] * D2R) / (1.0 * D2R)]
            else:
                d, g, dt, w, still = x
                xa, ya, tha = p[3 * k:3 * k + 3]
                xb, yb, thb = p[3 * k + 3:3 * k + 6]
                if use_gyro:
                    dth = g - b * dt
                    sg = n.g_base * math.sqrt(max(dt, 1e-3)) + n.g_rel * abs(g) + 1e-4
                    out.append(wrap(thb - tha - dth) / sg)
                if use_wheel_w or not use_gyro:
                    sw = n.w_base * math.sqrt(max(dt, 1e-3)) + n.w_rel * abs(w) + 1e-4
                    out.append(wrap(thb - tha - w) / sw)
                mid = tha + wrap(thb - tha) / 2
                sd = n.v_base * math.sqrt(max(dt, 1e-3)) + n.v_rel * abs(d) + 0.01
                out += [(xb - xa - d * math.cos(mid)) / sd, (yb - ya - d * math.sin(mid)) / sd]
                if use_gyro and still:
                    out.append((g / dt - b) / (n.g_base / math.sqrt(dt)))
        for k, det in cam_k:
            out += [(p[3 * k] - det["x"]) / n.cam_xy, (p[3 * k + 1] - det["y"]) / n.cam_xy]
        return np.array(out)

    # which unknowns each residual touches (keeps the solver fast)
    m = len(residuals(guess))
    J = lil_matrix((m, 3 * K + 1), dtype=int)
    r = 0
    for kind, k, x in rows:
        if kind == "prior":
            for i in range(3):
                J[r + i, i] = 1
            r += 3
            continue
        cnt = (1 if use_gyro else 0) + (1 if (use_wheel_w or not use_gyro) else 0) + 2
        for i in range(cnt):
            J[r + i, 3 * k:3 * k + 6] = 1
            J[r + i, 3 * K] = 1
        r += cnt
        if use_gyro and x[4]:
            J[r, 3 * K] = 1
            r += 1
    for k, _ in cam_k:
        J[r, 3 * k] = 1
        J[r + 1, 3 * k + 1] = 1
        r += 2
    # Plain least squares first (converges quickly from any reasonable guess),
    # then the robust weighting from there, so a few bad camera fixes can't
    # drag the path.
    # The inner sparse solver (lsmr) must be run to tight tolerances, or each
    # step is too rough for the outer solver ever to settle.
    opts = dict(jac_sparsity=J, x_scale="jac", max_nfev=200, tr_options=dict(atol=1e-12, btol=1e-12))
    sol = least_squares(residuals, guess, loss="linear", **opts)
    sol = least_squares(residuals, sol.x, loss="huber", f_scale=huber, **opts)
    if sol.status <= 0:
        print(f"  factor graph for {tracked.run.run_id}: solver stopped before settling ({sol.message})")
    p = sol.x
    return [(tk[k], p[3 * k], p[3 * k + 1], wrap(p[3 * k + 2])) for k in range(K)]
