#!/usr/bin/env python3
"""What each run did as it ran (e17 off by definition: every uploaded run predates e17), for the measures a layer
replay cannot reproduce: re-seed heading jumps and REJOINED after NEEDS A HAND. Logs only.

    python3 asrun.py RUNS_ROOT REPLAY_OUT OUT.json

Heading jumps: adapted from docs/results/e17-nov12/reseed_heading.py (3f37317): for each fpose the run sent
(pose.csv reseed rows; events heading re-set), the robot's own heading discontinuity when it landed (follow.csv), and
how fast it was turning when it was sent. Only "reseed" sends count as jumps (a heading re-set is meant to change the heading).
e17's rule holds a re-seed until the robot turns under 8 deg/s
(MIRLT_RESEED_TURN); the "with e17" column is an ESTIMATE: re-seeds sent while turning faster are assumed to land
standing (their jump is not counted), the rest are counted as they were.

REJOINED: for each REJOINED, the NEEDS A HAND spell before it; the bridge's sightings of that robot over the spell
say whether anybody moved it (moved >= MOVED_CM from where it stood: a person's move) or not (a false REJOINED: the
run's own pose came back to the camera by itself). With e17: whether the e17-ON replay named the robot by elimination
inside that spell (the run would then have said CORRECTED, not REJOINED).

Changelog
  2026-11-12 v1  first version (replay-all-nov12)
"""
import bisect
import csv
import glob
import json
import math
import os
import sys

VERSION = "asrun-2026-11-12v1"
TURN_DPS = 8.0      # as e17's re-seed rule (derived there: 8 deg/s x 0.3 s landing lag ~ the camera heading's 3 deg)
BIG_DEG = 15.0      # as e17's chart: a jump over 15 deg
MOVED_CM = 5.0      # CHOSEN: half the "hand's width (10 cm)" the run asks for, over 3 x the lights' 1.5 cm sigma


def w(a):
    return (a + 180) % 360 - 180


def _cam_at(c, ts, t, near=0.5):
    i = bisect.bisect_left(ts, t)
    if i <= 0 or i >= len(c) or t - c[i - 1][0] > near or c[i][0] - t > near:
        return None
    a, b = c[i - 1], c[i]
    f = (t - a[0]) / max(1e-6, b[0] - a[0])
    return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2])


def heading_jumps(R):
    if not os.path.exists(R + "/pose.csv"):
        return None
    fo = {}
    for r in csv.DictReader(open(R + "/follow.csv")):
        if r["state"] == "nopose":
            continue
        fo.setdefault(r["robot"], []).append((float(r["t_recv"]), float(r["x_cm"]), float(r["y_cm"]),
                                              float(r["th_deg"]), r["state"]))
    cam = {}
    for r in csv.DictReader(open(R + "/bridge/track.csv")):
        if r["status"] == "seen":
            cam.setdefault(r["robot"], []).append((float(r["t"]), float(r["x"]), float(r["y"])))
    for v in cam.values():
        v.sort()
    sends = []
    for r in csv.DictReader(open(R + "/pose.csv")):
        if r["what"] == "reseed":
            sends.append((float(r["t"]), r["robot"], "reseed"))
    for r in csv.DictReader(open(R + "/events.csv")):
        if r["kind"] in ("heading re-set",) or (r["kind"] == "off plan" and "heading re-set" in r["text"]):
            sends.append((float(r["t"]), r["robot"], "heading"))
    out = []
    for t, n, kind in sorted(sends):
        F = fo.get(n, [])
        ts = [q[0] for q in F]
        i = bisect.bisect(ts, t) - 1
        if i < 3 or i + 15 >= len(F):
            continue
        a = max(0, i - 5)
        rate = w(F[i][3] - F[a][3]) / max(0.05, F[i][0] - F[a][0])
        land = None
        for k in range(i + 1, min(len(F) - 1, i + 15)):
            if math.hypot(F[k][1] - F[k - 1][1], F[k][2] - F[k - 1][2]) > 2.5 and F[k][0] - F[k - 1][0] < 0.4:
                land = k
                break
        if land is None:
            continue
        k = land
        r0 = w(F[k - 1][3] - F[k - 2][3]) / max(0.05, F[k - 1][0] - F[k - 2][0])
        r1 = w(F[k + 2][3] - F[k + 1][3]) / max(0.05, F[k + 2][0] - F[k + 1][0])
        exp = 0.5 * (r0 + r1) * (F[k][0] - F[k - 1][0])
        disc = w(F[k][3] - F[k - 1][3] - exp)
        # was the jump right? The next 6 cm its own pose moves (within 15 s): its own course against the camera's
        # course over the same time. With the jump undone, its own course would be own course - disc.
        chk = None
        C = cam.get(n, [])
        cts = [q[0] for q in C]
        for m in range(k + 1, len(F)):
            if F[m][0] - F[k][0] > 15.0:
                break
            if math.hypot(F[m][1] - F[k][1], F[m][2] - F[k][2]) >= 6.0:
                ca, cb = _cam_at(C, cts, F[k][0]), _cam_at(C, cts, F[m][0])
                if ca and cb and math.hypot(cb[0] - ca[0], cb[1] - ca[1]) >= 3.0:
                    oc = math.degrees(math.atan2(F[m][2] - F[k][2], F[m][1] - F[k][1]))
                    cc = math.degrees(math.atan2(cb[1] - ca[1], cb[0] - ca[0]))
                    chk = dict(err_with=round(abs(w(cc - oc)), 1), err_without=round(abs(w(cc - (oc - disc))), 1))
                break
        out.append(dict(t=round(t, 2), robot=n, kind=kind, rate=round(rate, 1), disc=round(disc, 1), check=chk))
    rs = [o for o in out if o["kind"] == "reseed"]      # a re-seed sets the position; its heading only rides along
    turning = [o for o in rs if abs(o["rate"]) > TURN_DPS]
    still = [o for o in rs if abs(o["rate"]) <= TURN_DPS]
    med = lambda v: sorted(v)[len(v) // 2] if v else None    # noqa: E731
    big_still = [o for o in still if abs(o["disc"]) > BIG_DEG]
    chk = [o["check"] for o in big_still if o["check"]]
    return dict(n=len(out), reseeds=len(rs), heading_resets=len(out) - len(rs), sent_turning=len(turning),
                big_off=sum(abs(o["disc"]) > BIG_DEG for o in rs), big_on_est=len(big_still),
                big_still_checked=len(chk), big_still_jump_wrong=sum(1 for c in chk if c["err_with"] > c["err_without"]),
                med_turning=med([abs(o["disc"]) for o in turning]), med_still=med([abs(o["disc"]) for o in still]),
                sends=out)


def rejoins(R, replay_on):
    ev = list(csv.DictReader(open(R + "/events.csv")))
    cam = {}
    for r in csv.DictReader(open(R + "/bridge/track.csv")):
        if r["status"] == "seen":
            cam.setdefault(r["robot"], []).append((float(r["t"]), float(r["x"]), float(r["y"])))
    elim = [x for x in (replay_on or {}).get("taken", [])]
    out = []
    first_hand = {}
    for r in ev:
        n, k, t = r["robot"], r["kind"], float(r["t"])
        if k == "NEEDS A HAND" and n not in first_hand:
            first_hand[n] = t
        if k == "REJOINED":
            t0 = first_hand.pop(n, None)
            import re
            m = re.search(r"waited (\d+(?:\.\d+)?) s", r["text"])
            if m:                       # the run's own count of the wait is the spell (a later hand spell's start
                t0 = t - float(m.group(1))   # otherwise could be carried over)
            if t0 is None:
                continue
            c = [q for q in cam.get(n, []) if t0 <= q[0] <= t]
            cover = 0.0
            if len(c) >= 2:
                cover = sum(min(2.0, b[0] - a[0]) for a, b in zip(c, c[1:])) / max(1e-6, t - t0)
            if len(c) >= 3 and cover >= 0.5:
                x0 = sorted(q[1] for q in c[:5])[len(c[:5]) // 2]
                y0 = sorted(q[2] for q in c[:5])[len(c[:5]) // 2]
                moved = max(math.hypot(q[1] - x0, q[2] - y0) for q in c)
                verdict = "moved" if moved >= MOVED_CM else "not moved (false REJOINED)"
            else:
                moved, verdict = None, "camera did not see it enough to tell"
            e = [x for x in elim if x["robot"] == n and t0 <= x["t"] <= t]
            out.append(dict(robot=n, t_hand=round(t0, 2), t_rejoined=round(t, 2), wait_s=round(t - t0, 1),
                            camera_cover=round(cover, 2), moved_cm=None if moved is None else round(moved, 1),
                            verdict=verdict, e17_corrected_at=round(e[0]["t"], 2) if e else None,
                            e17_after_s=round(e[0]["t"] - t0, 1) if e else None))
    hands = sum(1 for r in ev if r["kind"] == "NEEDS A HAND")
    return dict(needs_a_hand_lines=hands, rejoined=out)


def main(root, replay_out, out_path):
    res = {}
    for R in sorted(glob.glob(os.path.join(root, "*-freeroam"))):
        run = os.path.basename(R)
        if not os.path.exists(R + "/follow.csv"):
            continue
        rp = os.path.join(replay_out, f"{run}-on-w1.json")
        on = json.load(open(rp)) if os.path.exists(rp) else None
        res[run] = dict(heading=heading_jumps(R), rejoin=rejoins(R, on))
    json.dump(dict(version=VERSION, runs=res), open(out_path, "w"), indent=1)
    for run, r in res.items():
        h = r["heading"]
        print(run, None if h is None else {k: v for k, v in h.items() if k != "sends"},
              [(x["robot"], x["wait_s"], x["moved_cm"], x["verdict"][:9], x["e17_after_s"]) for x in r["rejoin"]["rejoined"]])


if __name__ == "__main__":
    main(*sys.argv[1:4])
