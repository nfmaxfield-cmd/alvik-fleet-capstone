"""DOE row "recheck before lost" (e02.py RECHECK_S, 3.5 s): replay every runner-driven go leg of the sessions on disk
through the camera-against-wheels check (cross_check), and, where they disagree, look again at the camera's track
1-6 s later, the way the recheck does.

Per go leg (leg.csv, done): the wheels' distance (started -> done pose), the camera's still position before the
leg and after it (e02_camera.still_position over the window settle() reads: from 0.35 s before 'done' for up to
2.25 s), and the check |camera - wheels| > max(CHECK_CM 5, CHECK_FRAC 0.35 x wheels) (trust factors at 1, no
fresh-identify allowance). For a disagreement, the recheck at s seconds reads the still position in the 0.6 s ending
0.25 + s + 0.6 s after 'done'; it rescues the leg if that agrees with the wheels. The window is cut where the robot
next moves (its next leg starts, or a firmware run starts): later than that the replay cannot say (censored).

What it cannot see: whether a rescued leg was really fine (no truth here; a ghost that settles where the wheels say
is the case RECHECK_S is for); the sessions before 2026-10-24 19:39 refound at once, so most of their
disagreements are censored within a second or two.

Usage: python3 recheck_replay.py   (prints; writes out/recheck_legs.csv)
"""
import bisect
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oct25_common as C  # noqa: E402
import e02_camera as CAM  # noqa: E402

SESS = C.CLASSIC + ("right-alone",)
CHECK_CM, CHECK_FRAC = 5.0, 0.35
DELAYS = (0.0, 1.0, 2.0, 3.0, 3.5, 4.0, 5.0, 6.0)


def still(tr_t, tr, t0, t1):
    i, j = bisect.bisect_left(tr_t, t0), bisect.bisect_right(tr_t, t1)
    pts = tr[i:j]
    if len(pts) < 3:
        return None
    # settle(): the first moment a still window of 0.3 s is found, reading forward
    for k in range(3, len(pts) + 1):
        sp = CAM.still_position([(q[0], q[1], q[2]) for q in pts[:k]], min_s=0.3)
        if sp:
            return sp
    return None


def main():
    out = []
    for name in SESS:
        trk = C.track(name)
        legs = C.robot_rows(name, "leg.csv")
        runs = C.robot_rows(name, "runinfo.csv")
        for robot in C.NC.ROBOTS:
            tr = trk.get(robot, [])
            if not tr:
                continue
            tr_t = [q[0] for q in tr]
            mine = [l for l in legs if l["robot"] == robot]
            started = {}
            for l in mine:
                if l.get("result") == "started":
                    started[l["id"]] = l
            moves = sorted([float(l["t_recv"]) for l in mine if l.get("result") == "started"] +
                           [float(r["t_recv"]) for r in runs if r["robot"] == robot])
            for l in mine:
                if l.get("result") != "done" or l.get("kind") != "go" or l["id"] not in started:
                    continue
                s = started[l["id"]]
                ts, td = float(s["t_recv"]), float(l["t_recv"])
                a = (C.num(s["x_cm"]), C.num(s["y_cm"]))
                b = (C.num(l["x_cm"]), C.num(l["y_cm"]))
                if None in a or None in b:
                    continue
                w = math.hypot(b[0] - a[0], b[1] - a[1])
                # p0: settle() before the leg reads the newest still 0.3 s (the robot stands still then)
                i0, j0 = bisect.bisect_left(tr_t, ts - 2.0), bisect.bisect_right(tr_t, ts)
                p0 = CAM.still_position([(q[0], q[1], q[2]) for q in tr[i0:j0]], min_s=0.3)
                v = C.num(l["value"]) or 0.0
                if v < 0 and any(abs(float(o["t_recv"]) - td) < 4.0 and o is not l and o.get("kind") == "go"
                                 and o.get("result") == "done" and C.num(o["value"]) == v for o in mine):
                    continue                  # a backing piece (a long back-off in pieces): the runner checks none
                if p0 is None:
                    continue
                k = bisect.bisect_right(moves, td + 0.05)
                t_next = moves[k] if k < len(moves) else 1e18
                res = {}
                for dly in DELAYS:
                    lo = td - 0.35 if dly == 0 else td - 0.1 + dly   # settle(): 0.6 s back from 0.25 s + the first settle + the wait
                    hi = lo + (2.25 if dly == 0 else 1.5)
                    if hi > t_next:
                        hi = t_next
                    if hi - lo < 0.3:
                        res[dly] = "censored"
                        continue
                    p1 = still(tr_t, tr, lo, hi)
                    if p1 is None:
                        res[dly] = "none"
                        continue
                    c = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                    res[dly] = "agree" if abs(c - w) <= max(CHECK_CM, CHECK_FRAC * w) else "disagree:%.1f" % c
                out.append([name, robot, l["id"], round(td, 2), round(w, 1), round(t_next - td, 1)] +
                           [res[dl] for dl in DELAYS])
    hdr = ["session", "robot", "leg", "t_done", "wheels_cm", "s_to_next_move"] + ["at_%gs" % d for d in DELAYS]
    C.write_csv(os.path.join(C.OUT, "recheck_legs.csv"), hdr, out)

    print("go legs replayed: %d" % len(out))
    first = [r for r in out if r[6] != "agree"]
    print("disagree (or no still position) at the first look: %d (%.1f%%)" % (len(first), 100.0 * len(first) / len(out)))
    for name in SESS:
        q = [r for r in out if r[0] == name]
        f = [r for r in q if r[6] != "agree"]
        if q:
            print("  %-14s legs %4d  first look not agreeing %3d  (no still position %d)" % (
                name, len(q), len(f), sum(1 for r in f if r[6] == "none")))
    print("\nOf those, by the recheck delay: rescued (agrees by then), still disagreeing, no reading, censored "
          "(the robot moved again first)")
    for i, dly in enumerate(DELAYS[1:], start=7):
        resc = sum(1 for r in first if any(r[j] == "agree" for j in range(7, i + 1)))
        dis = sum(1 for r in first if r[i].startswith("disagree") and not any(r[j] == "agree" for j in range(7, i + 1)))
        none = sum(1 for r in first if r[i] == "none" and not any(r[j] == "agree" for j in range(7, i + 1)))
        cen = sum(1 for r in first if r[i] == "censored" and not any(r[j] == "agree" for j in range(7, i + 1)))
        print("  %4.1f s: rescued %3d  disagree %3d  none %3d  censored %3d" % (dly, resc, dis, none, cen))
    gaps = [r[5] for r in first if r[5] < 1e6]
    if gaps:
        print("\nseconds from 'done' to the robot's next move after a first-look disagreement: median %.1f, 10-90%% "
              "%.1f-%.1f" % (np.median(gaps), np.percentile(gaps, 10), np.percentile(gaps, 90)))
    print("\nlegs where the first look AGREED but a later look disagrees (a settle the recheck would not have seen): %d"
          % sum(1 for r in out if r[6] == "agree" and any(str(x).startswith("disagree") for x in r[7:12])))


if __name__ == "__main__":
    main()
