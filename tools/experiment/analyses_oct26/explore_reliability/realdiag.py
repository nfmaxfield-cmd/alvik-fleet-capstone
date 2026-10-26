"""realdiag.py SESSION_FOLDER [...] : a real session's explore stops (e02_state.json setup.explore_obs) against the
stored roam1 calibration (tools/experiment/data/overhead_floorcal_roam1.json, which fits the board in the video:
2026-10-26 09:55 analysis). Per robot: its straight legs' wheel distance against the camera's (roam1) distance, the
scale its whole path needs to match the camera (a similarity fit: what explore's scale would be on that robot alone),
how many of its stops the camera's spot jumped, and which board sides its contacts really reached."""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, EXP)
sys.path.insert(0, os.path.join(os.path.dirname(EXP), "recognition"))
import e02_explore as EX  # noqa: E402
import e02_camera as CAM  # noqa: E402

ROAM = CAM.load_camera(os.path.join(EXP, "data", "overhead_floorcal_roam1.json"))
W = H = 122.0


def floor(q):
    return np.array(ROAM.to_floor(float(q["u"]), float(q["v"]), EX.ROBOT_TOP_CM), float)


def sim_fit(P, Q):
    """Q ~ s R P + t (a mirror allowed): returns s and the median residual."""
    best = None
    for mir in (1.0, -1.0):
        Pm = P * np.array([1.0, mir])
        mp, mq = Pm.mean(0), Q.mean(0)
        U, S, Vt = np.linalg.svd((Q - mq).T @ (Pm - mp))
        D = np.eye(2)
        if np.linalg.det(U @ Vt) < 0:
            D[1, 1] = -1
        R = U @ D @ Vt
        s = float(np.trace(np.diag(S) @ D) / ((Pm - mp) ** 2).sum())
        r = np.linalg.norm((s * (R @ (Pm - mp).T)).T + mq - Q, axis=1)
        if best is None or np.median(r) < best[1]:
            best = (s, float(np.median(r)))
    return best


def main():
    for d in sys.argv[1:]:
        st = json.load(open(os.path.join(d, "e02_state.json")))
        obs = (st.get("setup") or {}).get("explore_obs") or {}
        print("=" * 90)
        print(os.path.basename(d.rstrip("/")))
        for n, stops in obs.items():
            if len(stops) < 5:
                print("  %s: %d stops" % (n, len(stops)))
                continue
            chk = EX.robot_check(stops)
            g = chk.get("g") or 1.07
            imp = EX.implausible(stops)
            path = np.array([(p[0], p[1]) for p in EX.rebuild(stops, 1.0, g)])
            ok = np.array([not q.get("skip") for q in imp])
            F = np.array([floor(q) for q in stops])
            s_all, med = sim_fit(path[ok], F[ok]) if ok.sum() >= 4 else (float("nan"), float("nan"))
            # straight legs of 8 cm or more between two trusted stops
            rat = []
            for i in range(1, len(stops)):
                q = stops[i]
                if "d" not in q or abs(q["d"]) < 8.0 or abs(q.get("dth", 0) or 0) > 3.0 or not (ok[i] and ok[i - 1]):
                    continue
                cm = float(np.linalg.norm(F[i] - F[i - 1]))
                rat.append(cm / abs(q["d"]))
            sides = {}
            for i, q in enumerate(stops):
                if not q.get("contact"):
                    continue
                x, y = F[i]
                dd = dict(x0=abs(x), y0=abs(y), x1=abs(x - W), y1=abs(y - H))
                s_ = min(dd, key=dd.get)
                if dd[s_] <= 20.0:
                    sides.setdefault(s_, []).append(q["contact"])
                else:
                    sides.setdefault("inside", []).append(q["contact"])
            print("  %s: %d stops (%d implausible); own check %s; whole path needs scale %.3f (median left %.1f cm); "
                  "straight legs camera/wheels median %s over %d; contacts %s" % (
                      n, len(stops), int((~ok).sum()), "ok %.1f px" % chk["rms_px"] if chk["ok"] else "FAIL " + chk["why"],
                      s_all, med, "%.3f" % np.median(rat) if rat else "-", len(rat),
                      {k: len(v) for k, v in sides.items()}))
        for t in (st.get("setup") or {}).get("explore_tries") or []:
            print("  try:", {k: t.get(k) for k in ("used", "left_out", "why")})


if __name__ == "__main__":
    main()
