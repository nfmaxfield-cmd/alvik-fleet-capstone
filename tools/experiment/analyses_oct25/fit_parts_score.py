"""Follow-up to fit_settings_score.py: which part of the session-fitted calibration (gyro scale, gyro drift, wheel
scale) helps or hurts the predicted path, against the fixed prior (e02_fit.PRIOR). Same runs, same LORO fits
(cut 3 cm, band 0.05, gate 1.5 cm), each part swapped in alone.

Usage: python3 fit_parts_score.py   (prints; writes out/fit_parts.csv)"""
import os
import sys

import numpy as np
from scipy.stats import wilcoxon

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oct25_common as C  # noqa: E402
from oct25_common import FIT  # noqa: E402
import fit_settings_score as S  # noqa: E402

VARIANTS = ("prior", "fitted", "gyro scale only", "drift only", "wheel only", "gyro scale + drift")


def main():
    out = []
    for name in C.CLASSIC:
        drift = C.measured_drift(name)
        runs = S.test_runs(name)
        tr = C.track(name)
        per = C.stretches_by_run(name, jump_cm=3.0)
        for r in runs:
            robot = r["robot"]
            prior = dict(FIT.PRIOR)
            if drift.get(robot) is not None:
                prior["gyro_drift_dps"] = drift[robot]
            train = [o for order, run, obs in per.get(robot, []) if run != r["run"] for o in obs]
            res = C.fit_with(train, drift.get(robot), band=0.05, max_rms=1.5) if train else None
            fv = S.values_of(res, prior) if res else prior
            vs = {"prior": prior, "fitted": fv,
                  "gyro scale only": dict(prior, gyro_scale=fv["gyro_scale"]),
                  "drift only": dict(prior, gyro_drift_dps=fv["gyro_drift_dps"]),
                  "wheel only": dict(prior, wheel_scale=fv["wheel_scale"]),
                  "gyro scale + drift": dict(prior, gyro_scale=fv["gyro_scale"], gyro_drift_dps=fv["gyro_drift_dps"])}
            for k in VARIANTS:
                e = S.errors(r, vs[k], tr.get(robot, []))
                out.append([name, robot, r["order"], r["cond"], k, round(vs[k]["gyro_scale"], 4),
                            round(vs[k]["gyro_drift_dps"], 4), round(vs[k]["wheel_scale"], 4)] +
                           ["" if v is None else round(v, 3) for v in e])
    C.write_csv(os.path.join(C.OUT, "fit_parts.csv"), ["session", "robot", "order", "cond", "variant", "gyro_scale",
                                                        "gyro_drift_dps", "wheel_scale", "end", "far", "path", "rot"],
                out)
    for mi, m in ((8, "end"), (9, "far"), (10, "path"), (11, "rot")):
        base = {(r[0], r[1], r[2]): r[mi] for r in out if r[4] == "prior" and r[mi] != ""}
        print("[%s]" % m)
        for k in VARIANTS:
            q = [(r[mi], base[(r[0], r[1], r[2])]) for r in out if r[4] == k and r[mi] != "" and (r[0], r[1], r[2]) in base]
            a = np.array([x for x, _ in q]); b = np.array([y for _, y in q])
            p = wilcoxon(a, b).pvalue if np.any(a != b) else 1.0
            print("  %-20s n %d median %.2f mean %.2f  vs prior: median diff %+.2f p %.3f" % (
                k, len(a), np.median(a), a.mean(), np.median(a - b), p))
    # the fitted values themselves
    print("fitted values per session (median over the robot's LORO fits):")
    for name in C.CLASSIC:
        for robot in C.NC.ROBOTS:
            q = [r for r in out if r[0] == name and r[1] == robot and r[4] == "fitted"]
            if q:
                print("  %-14s %-6s gyro %.4f drift %+.4f wheel %.4f (n %d)" % (
                    name, robot, np.median([r[5] for r in q]), np.median([r[6] for r in q]),
                    np.median([r[7] for r in q]), len(q)))


if __name__ == "__main__":
    main()
