"""Paired comparisons read from out/fit_settings_scores.csv (run fit_settings_score.py first): drift band 0 against
0.02 and 0.05, and the jump cut against no cut, at each band. LORO, gate 1.5 cm. Prints median paired differences and
Wilcoxon signed-rank p (a paired test on ranks: small p = the difference is consistent across runs)."""
import csv
import os
import sys

import numpy as np
from scipy.stats import wilcoxon

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oct25_common as C  # noqa: E402

rows = list(csv.DictReader(open(os.path.join(C.OUT, "fit_settings_scores.csv"))))
idx = {}
for r in rows:
    if r["scheme"] != "LORO" or r["gate"] != "1.5":
        continue
    idx.setdefault((r["cut"], r["band"]), {})[(r["session"], r["robot"], r["order"])] = r


def cmp(a, b, m):
    A, B = idx[a], idx[b]
    ks = [k for k in A if k in B and A[k][m] and B[k][m]]
    x = np.array([float(A[k][m]) for k in ks]); y = np.array([float(B[k][m]) for k in ks])
    p = wilcoxon(x, y).pvalue if np.any(x != y) else 1.0
    return len(ks), np.median(x - y), np.mean(x - y), p


for m in ("end_err_cm", "far_err_cm", "path_err_cm", "rot_err_cm"):
    print("[%s]" % m)
    for a, b in ((("3.0", "0.0"), ("3.0", "0.02")), (("3.0", "0.0"), ("3.0", "0.05")), (("3.0", "0.02"), ("3.0", "0.05")),
                 (("3.0", "0.0"), ("3.0", "")), (("", "0.0"), ("3.0", "0.0")), (("", "0.05"), ("3.0", "0.05")),
                 (("2.0", "0.0"), ("3.0", "0.0")), (("4.0", "0.0"), ("3.0", "0.0"))):
        n, md, mn, p = cmp(a, b, m)
        print("  cut %-4s band %-5s minus cut %-4s band %-5s: n %d median %+.2f mean %+.2f p %.3f" % (
            a[0] or "none", a[1] or "free", b[0] or "none", b[1] or "free", n, md, mn, p))
