"""DOE row "sample-size minimums" (build-plans/publication-plan.md, Appendix B): a preliminary redo of the per-arm
counts with every C run on disk instead of session 1's alone. Not the week-3 calculation (that waits for the first new
session); this shows how much the answer moves.

C-arm sd from: runs.csv end_err_cm of the C runs (camera end against the start), and, as a second estimate, the
re-integrated end errors of fit_settings_score.py (every U and C run re-run with its leave-one-run-out calibration,
cut 3 cm, band 0). sd: the ordinary sd and a robust one (median absolute deviation / 0.6745), since one 13 cm run
dominates the ordinary one.
TOST per arm (independent arms, alpha 0.05 each side, power 0.8, true difference 0): n = 2 sd^2 (z0.95 + z0.90)^2 / m^2.

Usage: python3 power_prelim.py"""
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oct25_common as C  # noqa: E402

Z = 1.6449 + 1.2816


def n_tost(sd, m=1.0):
    return math.ceil(2 * sd * sd * Z * Z / (m * m))


def rsd(v):
    v = np.asarray(v, float)
    return float(np.median(np.abs(v - np.median(v))) / 0.6745)


def show(label, v):
    v = np.asarray(v, float)
    sd, r = float(np.std(v, ddof=1)), rsd(v)
    print("  %-48s n %3d  mean %.2f  sd %.2f -> %3d per arm;  robust sd %.2f -> %3d per arm" % (
        label, len(v), v.mean(), sd, n_tost(sd), r, n_tost(r)))


print("TOST per arm at a 1 cm margin")
allc = []
for name in C.CLASSIC:
    v = [float(r["end_err_cm"]) for r in C.rows(name, "runs.csv") if r["cond"] == "C" and r["status"] == "done"
         and r.get("end_err_cm")]
    if v:
        allc += v
        if len(v) > 1:
            show("C runs, %s" % name, v)
        else:
            print("  C runs, %-40s n   1  %.2f" % (name, v[0]))
show("C runs, all sessions on disk", allc)
p = os.path.join(C.OUT, "fit_settings_scores.csv")
if os.path.exists(p):
    rows = [r for r in csv.DictReader(open(p)) if r["scheme"] == "LORO" and r["cut"] == "3.0" and r["band"] == "0.0"
            and r["gate"] == "1.5" and r["end_err_cm"]]
    show("re-integrated end error, LORO calibration (U+C)", [float(r["end_err_cm"]) for r in rows])
    show("  the same, C runs only", [float(r["end_err_cm"]) for r in rows if r["cond"] == "C"])
