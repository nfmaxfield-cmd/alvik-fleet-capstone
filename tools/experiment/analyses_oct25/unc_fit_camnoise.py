"""Follow-up to unc_fit.py (self-review): the correction at a fix is the robot's error plus the camera's own error, so
the fitted base term holds the camera's noise too. Refit with the camera's healthy noise (e02_trust HEALTHY camera,
0.8 cm per axis; also 0.5 and 1.1) added in quadrature: sd_total = sqrt(sd_robot^2 + cam^2), sd_robot = the e02.py
model. Same chains as unc_fit.py (intact, heading corrected under 30 deg), same outlier mixture.

Usage: python3 unc_fit_camnoise.py"""
import os
import sys

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oct25_common as C  # noqa: E402
import unc_fit as U  # noqa: E402

ch = []
for n in U.ALL:
    ch += C.fix_chain(n)
use = [c for c in ch if c["chain"] and c["intact"] and c["corr"] is not None and c["n_legs"] >= 1
       and c["prev_dt"] < 600 and c["dth"] is not None and abs(c["dth"]) < 30]
r = np.array([c["corr"] for c in use]); d = np.array([c["dist"] for c in use]); t = np.array([c["turned"] for c in use])
print("chains: %d" % len(use))
for cam in (0.5, 0.8, 1.1):
    def f(x):
        sr = U.sd_of(x[:4], d, t)
        return U.nll(r, np.sqrt(sr ** 2 + cam ** 2), x[4])
    res = minimize(f, [0.3, 0.01, 0.5, 0.005, 0.15], method="L-BFGS-B",
                   bounds=[(0.0, 5), (0, 0.2), (0, 20), (0, 0.5), (0, 0.5)])
    x = res.x
    now = U.nll(r, np.sqrt(U.sd_of(U.NOW, d, t) ** 2 + cam ** 2), 0.2)
    print("camera %.1f cm: robot base %.2f per_cm %.4f deg %.2f per deg turned %.4f outliers %.3f (-logL %.1f; "
          "model now with this camera: %.1f); robot sd after 50 cm + 180 deg %.2f cm (now %.2f)" % (
              cam, x[0], x[1], x[2], x[3], x[4], res.fun, now, float(U.sd_of(x[:4], np.array([50.0]), np.array([180.0]))[0]),
              float(U.sd_of(U.NOW, np.array([50.0]), np.array([180.0]))[0])))
