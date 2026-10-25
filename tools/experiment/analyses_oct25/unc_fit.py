"""DOE row "uncertainty model" (e02.py UNC_BASE_CM / UNC_PER_CM / UNC_DEG / UNC_DEG_PER_DEG): does the runner's own
position uncertainty match how far off the robots really were when the camera next fixed them, and what constants
fit the logs?

Data: every camera fix the runner sent between runs (fix.csv: the robot's own pose just before the fix against the
camera's), with what the robot drove and turned since its previous fix or finished find (leg.csv done legs). That is
the regression DOE-confidence.md asks for, on far more points than confidence.csv alone (which logs only the pauses;
it is checked separately below). Fixes inside a run (KC blends) and fixes with a run in between are left out: the
route's own driving is not in leg.csv.

Model fitted: the correction (2-D distance, cm) is Rayleigh-distributed with a per-axis sd of
    sd = base + per_cm x d + d x rad(deg + deg_per_deg x turned)            (e02.py unc_cm)
by maximum likelihood, with a small share of outliers (a wrong pick-up or a stale pose) allowed as a wide second
Rayleigh (sd 15 cm). Also tried: a time term (s since the previous fix), and whether the previous reset gave the
robot a camera heading or only a position.

<sub>Rayleigh: the length of a 2-D error whose x and y parts are each normal with the same sd. 39% of such errors are
under 1 sd, 86% under 2 sd, 95% under 2.45 sd.</sub>

Usage: python3 unc_fit.py   (prints; writes out/unc_chain.csv and out/unc_fit.txt via tee)
"""
import math
import os
import sys

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oct25_common as C  # noqa: E402

EXTRA = {   # the 2026-10-22 sessions (GitHub release e02-session-oct22-evening, logs.zip), older firmware
    "oct22-0816": "oct25dl/e02-session-oct22-evening/x/e02-20261022-081629",
    "oct22-0951": "oct25dl/e02-session-oct22-evening/x/e02-20261022-095125",
    "oct22-1929": "oct25dl/e02-session-oct22-evening/x/e02-20261022-192858",
    "oct22-1954": "oct25dl/e02-session-oct22-evening/x/e02-20261022-195419",
    "oct22-2005": "oct25dl/e02-session-oct22-evening/x/e02-20261022-200532",
}
for k, v in EXTRA.items():
    C.SESSIONS.setdefault(k, v)
ALL = ("oct22-0816", "oct22-0951", "oct22-1929", "oct22-1954", "oct22-2005", "oct23-morning", "oct23-evening",
       "stage1", "stage2", "right-alone")
NOW = (0.5, 0.01, 0.5, 0.005)
OUT_SD = 15.0


def sd_of(p, d, t, dt=None, tdt=0.0):
    base, per_cm, deg, dpd = p
    s = base + per_cm * d + d * np.radians(deg + dpd * t)
    if dt is not None:
        s = s + tdt * dt
    return np.maximum(s, 0.05)


def nll(r, s, eps):
    """-log likelihood of distances r under Rayleigh(s) with a share eps of Rayleigh(OUT_SD)."""
    f1 = r / s ** 2 * np.exp(-r ** 2 / (2 * s ** 2))
    f2 = r / OUT_SD ** 2 * np.exp(-r ** 2 / (2 * OUT_SD ** 2))
    return -np.sum(np.log((1 - eps) * f1 + eps * f2 + 1e-300))


def coverage(r, s):
    z = r / s
    return np.mean(z < 1), np.mean(z < 2), np.mean(z < 2.45), np.median(z)


def main():
    ch = []
    for name in ALL:
        try:
            ch += C.fix_chain(name)
        except (OSError, KeyError) as e:
            print("  %s: skipped (%s)" % (name, e))
    use_all = [c for c in ch if c["chain"] and c["corr"] is not None and c["n_legs"] >= 1 and c["prev_dt"] < 600]
    use = [c for c in use_all if c["intact"]]
    print("fixes: %d in all, %d chains between runs with legs since the previous reset (within 10 min), %d of them "
          "intact (the legs' own poses join up)" % (len(ch), len(use_all), len(use)))
    hd = [c for c in use if c["dth"] is not None]
    for lab, f in (("under 10", lambda a: a < 10), ("10-30", lambda a: 10 <= a < 30), ("30 or more", lambda a: a >= 30)):
        q = [c["corr"] for c in hd if f(abs(c["dth"]))]
        print("  heading corrected by %-10s deg at the fix: n %3d (%2.0f%%), position correction median %5.2f cm, "
              "90%% %5.1f" % (lab, len(q), 100.0 * len(q) / len(hd), np.median(q), np.percentile(q, 90)))
    C.write_csv(os.path.join(C.OUT, "unc_chain.csv"),
                ["session", "robot", "t", "prev_kind", "prev_dt_s", "dist_cm", "turned_deg", "n_legs", "corr_cm",
                 "dth_deg", "prev_had_cam_heading", "sd_now_model_cm"],
                [[c["session"], c["robot"], round(c["t"], 2), c["prev_kind"], round(c["prev_dt"], 1), round(c["dist"], 1),
                  round(c["turned"], 1), c["n_legs"], round(c["corr"], 2), "" if c["dth"] is None else round(c["dth"], 1),
                  c["prev_had_cam_heading"], round(C.unc_model(c["dist"], c["turned"]), 2)] for c in use])
    r = np.array([c["corr"] for c in use])
    d = np.array([c["dist"] for c in use])
    t = np.array([c["turned"] for c in use])
    dt = np.array([c["prev_dt"] for c in use])
    s_now = sd_of(NOW, d, t)
    print("\nDistance since the previous reset: median %.0f cm (10-90%%: %.0f-%.0f); turned median %.0f deg" % (
        np.median(d), np.percentile(d, 10), np.percentile(d, 90), np.median(t)))
    print("Correction: median %.2f cm, 90%% %.2f, max %.1f" % (np.median(r), np.percentile(r, 90), r.max()))
    c1, c2, c95, zm = coverage(r, s_now)
    print("Model now (0.5 cm, 1%%, 0.5 deg, 0.5%%): predicted sd median %.2f cm; share under 1 sd %.0f%% (Rayleigh 39%%), "
          "under 2 sd %.0f%% (86%%), under 2.45 sd %.0f%% (95%%); median correction / sd %.2f (Rayleigh 1.18)" % (
              np.median(s_now), 100 * c1, 100 * c2, 100 * c95, zm))

    # bins by distance
    print("\nBy distance since the previous reset (cm): n, median correction, 90%, model sd median, implied sd "
          "(median correction / 1.177)")
    for lo, hi in ((0, 10), (10, 25), (25, 50), (50, 100), (100, 1e9)):
        k = (d >= lo) & (d < hi)
        if k.sum() >= 3:
            print("  %4.0f-%-5s n %3d  median %5.2f  p90 %5.2f  model %5.2f  implied %5.2f" % (
                lo, "" if hi > 1e8 else "%.0f" % hi, k.sum(), np.median(r[k]), np.percentile(r[k], 90),
                np.median(s_now[k]), np.median(r[k]) / 1.177))

    # fits
    def fit(free, with_dt=False, r=r, d=d, t=t, dt=dt):
        x0 = [0.5, 0.01, 0.5, 0.005, 0.0, 0.05]
        names = ["base", "per_cm", "deg", "deg_per_deg", "per_s", "eps"]

        def f(x):
            p = list(NOW)
            for i, n in enumerate(names[:4]):
                if n in free:
                    p[i] = x[i]
            s = sd_of(p, d, t, dt if with_dt else None, x[4] if with_dt else 0.0)
            return nll(r, s, x[5])
        bnds = [(0.05, 5), (0, 0.2), (0, 20), (0, 0.5), (0, 0.05), (0.0, 0.5)]
        res = minimize(f, x0, bounds=bnds, method="L-BFGS-B")
        return res, names

    sub = np.array([c["dth"] is not None and abs(c["dth"]) < 30 for c in use])
    for title, k in (("all intact chains", np.ones(len(use), bool)),
                     ("chains whose heading stayed within 30 deg (fix heading correction under 30)", sub)):
        print("\nMaximum-likelihood fits, %s (n %d; outlier share eps fitted; AIC = 2 x parameters + 2 x -log "
              "likelihood, lower is better):" % (title, k.sum()))
        variants = [("model now, outliers only", (), False), ("base only", ("base",), False),
                    ("base + per cm", ("base", "per_cm"), False), ("base + deg", ("base", "deg"), False),
                    ("base + deg + deg per deg turned", ("base", "deg", "deg_per_deg"), False),
                    ("all four", ("base", "per_cm", "deg", "deg_per_deg"), False),
                    ("base + deg + time", ("base", "deg"), True)]
        best = None
        for label, free, wdt in variants:
            res, names = fit(free, wdt, r=r[k], d=d[k], t=t[k], dt=dt[k])
            npar = len(free) + 1 + (1 if wdt else 0)
            aic = 2 * npar + 2 * res.fun
            x = res.x
            p = [x[i] if n in free else NOW[i] for i, n in enumerate(names[:4])]
            s_ = sd_of(p, d[k], t[k], dt[k] if wdt else None, x[4] if wdt else 0.0)
            c1, c2, c95, zm = coverage(r[k], s_)
            print("  %-34s AIC %7.1f  base %.2f per_cm %.4f deg %.2f deg/deg %.4f%s eps %.3f | under 1/2/2.45 sd "
                  "%.0f/%.0f/%.0f%%" % (label, aic, p[0], p[1], p[2], p[3], (" per_s %.4f" % x[4]) if wdt else "",
                                        x[5], 100 * c1, 100 * c2, 100 * c95))
            if best is None or aic < best[0]:
                best = (aic, label, p, x)
        print("  best by AIC: %s; sd it gives after 50 cm and 180 deg: %.2f cm (model now %.2f)" % (
            best[1], float(sd_of(best[2], np.array([50.0]), np.array([180.0]))[0]),
            float(sd_of(NOW, np.array([50.0]), np.array([180.0]))[0])))

    # heading at the previous reset
    print("\nDid the previous reset give a camera heading? (fix after fix only)")
    for v in (True, False):
        k = np.array([c["prev_had_cam_heading"] is v for c in use])
        if k.sum() >= 3:
            print("  %-5s n %3d  median correction %.2f  p90 %.2f  median correction/model sd %.2f  median |heading "
                  "correction| %.1f deg" % (v, k.sum(), np.median(r[k]), np.percentile(r[k], 90),
                                            np.median(r[k] / s_now[k]),
                                            np.median([abs(c["dth"]) for c, kk in zip(use, k) if kk and c["dth"] is not None])))
    k = np.array([c["prev_kind"] == "find" for c in use])
    print("  after a find: n %d median %.2f p90 %.2f" % (k.sum(), np.median(r[k]), np.percentile(r[k], 90)))

    # per session
    print("\nPer session: n, median correction, median correction / model sd")
    for name in ALL:
        k = np.array([c["session"] == name for c in use])
        if k.sum():
            print("  %-14s n %3d  median %5.2f  ratio %.2f" % (name, k.sum(), np.median(r[k]), np.median(r[k] / s_now[k])))

    # confidence.csv itself
    print("\nconfidence.csv (the pauses): correction against the model's sd before the drive")
    cc = [(n, row) for n in ("stage1", "stage2") for row in C.rows(n, "confidence.csv") if row.get("correction_cm")]
    rr = np.array([float(row["correction_cm"]) for _, row in cc])
    ss = np.array([float(row["sd_now_cm"]) for _, row in cc])
    print("  n %d; sd_now median %.2f; correction median %.2f, p90 %.1f; under 2.45 sd %.0f%%; over 10 cm: %d" % (
        len(rr), np.median(ss), np.median(rr), np.percentile(rr, 90), 100 * np.mean(rr < 2.45 * ss), np.sum(rr > 10)))


if __name__ == "__main__":
    main()
