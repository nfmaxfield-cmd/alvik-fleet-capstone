"""DOE row "heading before a find" (e02.py FIND_HEAD_AGREE_DEG, 5 deg): would another limit have handed the find a
better heading?

Per find (state find_heading_checks: the runner's camera heading, whether it was measured, the robot's own gyro
heading, and whether the camera's was used), matched to the robot's next finished find (find.csv). The find's first
board touch measures the heading the robot believed: corr1_deg = true - believed (firmware e02.h). So the error of the
heading that was used is -corr1_deg, and the other candidate's error follows from the gap between the two:
    e_camera = e_own + (camera - own),   e_own = e_camera - (camera - own).
At a limit L the runner uses the camera's heading when it was measured and within L of the own heading. Score: the
heading error each limit would have taken into the find (median and 90% of |error|), and the share of finds that
would have started more than 10 deg wrong.

What it cannot see: the heading error also grows during the find's own turn and drive to the board (the same for
every limit, so it does not change the comparison, only the floor).

Usage: python3 find_heading_limit.py   (prints; writes out/find_heading.csv)"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oct25_common as C  # noqa: E402

LIMITS = (0.0, 2.0, 3.0, 5.0, 8.0, 10.0, 15.0, 20.0, 1e9)


def main():
    rows_ = []
    for name in ("stage1", "stage2"):
        st = C.NC.state(name)
        finds = sorted((r for r in C.robot_rows(name, "find.csv") if r.get("result") == "ok"),
                       key=lambda r: float(r["t_recv"]))
        for t, robot, hcam, measured, own, ok in st.get("find_heading_checks") or []:
            f = [r for r in finds if r["robot"] == robot and 0 < float(r["t_recv"]) - t < 120][:1]
            if not f or hcam is None or own is None:
                continue
            gap = C.wrap180(hcam - own)
            e_used = -float(f[0]["corr1_deg"])
            if ok:
                e_cam, e_own = e_used, e_used - gap
            else:
                e_own, e_cam = e_used, e_used + gap
            rows_.append([name, robot, round(t, 1), bool(measured), round(gap, 1), bool(ok), round(e_cam, 1),
                          round(e_own, 1), round(float(f[0]["corr1_cm"]), 2)])
    C.write_csv(os.path.join(C.OUT, "find_heading.csv"),
                ["session", "robot", "t", "camera_measured", "camera_minus_own_deg", "camera_used", "err_camera_deg",
                 "err_own_deg", "corr1_cm"], rows_)
    print("finds matched: %d (camera heading measured in %d); |camera - own| median %.1f deg, 90%% %.1f" % (
        len(rows_), sum(r[3] for r in rows_), np.median([abs(r[4]) for r in rows_]),
        np.percentile([abs(r[4]) for r in rows_], 90)))
    ec = np.array([abs(r[6]) for r in rows_]); eo = np.array([abs(r[7]) for r in rows_])
    print("heading error at the first touch, if always the camera's: median %.1f, 90%% %.1f; if always the own: median "
          "%.1f, 90%% %.1f" % (np.median(ec), np.percentile(ec, 90), np.median(eo), np.percentile(eo, 90)))
    print("\nlimit  camera used  |error| median  90%%  over 10 deg")
    for L in LIMITS:
        e = np.array([abs(r[6]) if (r[3] and abs(r[4]) <= L) else abs(r[7]) for r in rows_])
        n_cam = sum(1 for r in rows_ if r[3] and abs(r[4]) <= L)
        print("  %-5s  %3d          %5.1f        %5.1f  %d" % ("inf" if L > 1e8 else "%g" % L, n_cam, np.median(e),
                                                           np.percentile(e, 90), int(np.sum(e > 10))))
    big = [r for r in rows_ if abs(r[7]) > 10 or abs(r[6]) > 10]
    print("\nfinds with a candidate over 10 deg wrong:")
    for r in big:
        print("  ", r)


if __name__ == "__main__":
    main()
