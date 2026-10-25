"""Where the >= 30 deg heading errors between camera fixes come from (docs/build-reports/e02-oct25-heading.md).

The 2026-10-25 logs DOE (unc_fit.py) measured the heading error at a fix as dth = cam_th - before_th from fix.csv. But
fix.csv's cam_th is not a camera measurement: it is the heading the RUNNER sent with the fix (e02.py fix_now: its own
heads[robot], built from camera bearings of go legs plus the turns it commanded; "-" = position only). So dth is the
runner's heading against the robot's, and either may be the wrong one. This script measures both against the camera.

True heading (camera): the direction the robot's camera position moved during a go leg of >= MIN_MOVE_CM (reversed for
a backward leg). The firmware drives a go leg holding the heading it had at the start (pose.h straight), so
    robot error at a leg  e = wrap180(th_deg at the leg's start - camera direction).
Between two resets (a fix with a heading, or a find) the robot's heading is gyro only, so within a segment:
    e_first  (first measured leg after the reset)  ~ the error the reset wrote in (+ the gyro's error on turns before it)
    e_last   (last measured leg before the next fix) ~ the error the robot carried into the fix
    runner error at the fix  r = dth + e_last   (cam_th - true heading); cross-check: e_first of the next segment
    (the fix wrote cam_th in, so the robot's next error is the runner's error).

Causes, per chain whose fix shows |dth| >= 30 (the 17 % of unc_fit.py):
    runner      the runner's heading was the wrong one (|r| >= BIG, robot's own within BIG): the "error" is the
                runner's, and the fix writes it into the robot (hypothesis 1/2)
    written     the robot was wrong, and already from the previous reset (|e_first| >= BIG): a wrong heading written
                in at the previous fix (hypothesis 2)
    grew        the robot was wrong, and it grew inside the chain (e_last - e_first >= BIG): gyro / turns (3/4)
    both        both wrong
    unmeasured  no go leg long enough to measure the heading on one side

Also: quarter-turn clustering (hypothesis 1), the source of each wrong runner heading (the e02_log around the fix and the
legs before it), wheel-vs-gyro turn on the spot inside firmware runs (hypothesis 4), and robot / phase splits (5).

Usage: python3 heading_diag.py  (prints; writes out/heading_chains.csv, out/heading_legs.csv, out/heading_turns.csv,
out/heading_diag.png, out/heading_diag.txt via tee)
"""
import bisect
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oct25_common as C  # noqa: E402
import unc_fit as U  # noqa: E402  (adds the oct22 sessions to C.SESSIONS)

MIN_MOVE_CM = 8.0     # chosen: camera noise ~0.8 cm per axis -> direction sd ~ 1.1/8 rad ~ 8 deg per leg
BIG = 20.0            # chosen: an error this big is "wrong" (well above the 8 deg measurement noise)
w = C.wrap180


def track_med(tr, t0, t1):
    """Median camera position of a robot in [t0, t1] (held samples left out), or None."""
    ts = [q[0] for q in tr]
    i, j = bisect.bisect_left(ts, t0), bisect.bisect_right(ts, t1)
    pts = [(q[1], q[2]) for q in tr[i:j] if q[3] != "held"]
    if len(pts) < 2:
        return None
    return float(np.median([p[0] for p in pts])), float(np.median([p[1] for p in pts]))


def leg_table(name):
    """Every finished go / turn leg (done or failed) with its start/end times, own headings and, for go legs, the
    camera's direction of travel."""
    tr = C.track(name)
    legs = C.robot_rows(name, "leg.csv")
    start = {}
    out = []
    for r in sorted(legs, key=lambda r: float(r["t_recv"])):
        key = (r["robot"], r["id"])
        if r["result"] == "started":
            start[key] = r
            continue
        if r["result"] not in ("done", "failed") or key not in start:
            continue
        s = start.pop(key)
        t0, t1 = float(s["t_recv"]), float(r["t_recv"])
        val = C.num(r["value"])
        d = dict(session=name, robot=r["robot"], id=r["id"], kind=r["kind"], value=val, result=r["result"],
                 info=r.get("info") or "", run=r["run"], t0=t0, t1=t1, th0=C.num(s["th_deg"]), th1=C.num(r["th_deg"]),
                 x0=C.num(s["x_cm"]), y0=C.num(s["y_cm"]), x1=C.num(r["x_cm"]), y1=C.num(r["y_cm"]),
                 cam_dir=None, moved=None)
        if r["kind"] == "go" and val is not None:
            rt = tr.get(r["robot"], [])
            p0 = track_med(rt, t0 - 0.7, t0 - 0.02)
            p1 = track_med(rt, t1 + 0.25, t1 + 0.9)
            if p0 and p1:
                mv = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                d["moved"] = mv
                if mv >= MIN_MOVE_CM and mv >= 0.5 * abs(val):
                    b = math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))
                    d["cam_dir"] = b if val >= 0 else w(b + 180.0)
        out.append(d)
    return out


def resets(name):
    """robot -> sorted [(t, kind, row)]: fixes that set a heading (cam_th not nan, result replaced/set) and ok finds."""
    ev = {}
    for r in C.robot_rows(name, "fix.csv"):
        if not (r["result"] or "").startswith(("replaced", "set", "blended")):
            continue
        ev.setdefault(r["robot"], []).append((float(r["t_recv"]), "fix", r))
    for r in C.robot_rows(name, "find.csv"):
        if (r.get("result") or "") == "ok":
            ev.setdefault(r["robot"], []).append((float(r["t_recv"]), "find", r))
    for v in ev.values():
        v.sort(key=lambda e: e[0])
    return ev


def own_th_at(legs_r, t):
    """The robot's own heading just before t from its legs (done row th1 of the last leg ended before t)."""
    best = None
    for l in legs_r:
        if l["t1"] <= t and l["th1"] is not None:
            best = l
    return best


def log_lines(name):
    p = C.NC.path(name, "e02_log.txt")
    out = []
    if not os.path.exists(p):
        return out
    with open(p, errors="replace") as f:
        for line in f:
            out.append(line.rstrip("\n"))
    return out


def main():
    sessions = U.ALL
    rows_out, legs_out = [], []
    for name in sessions:
        try:
            ch = C.fix_chain(name)
            lt = leg_table(name)
            rs = resets(name)
        except (OSError, KeyError) as e:
            print("  %s: skipped (%s)" % (name, e))
            continue
        fixes_all = {(r["robot"], round(float(r["t_recv"]), 2)): r for r in C.robot_rows(name, "fix.csv")}
        by_robot = {}
        for l in lt:
            by_robot.setdefault(l["robot"], []).append(l)
            if l["cam_dir"] is not None:
                legs_out.append(l)
        use = [c for c in ch if c["chain"] and c["corr"] is not None and c["n_legs"] >= 1 and c["prev_dt"] < 600
               and c["intact"]]
        for c in use:
            robot, t = c["robot"], c["t"]
            r = fixes_all.get((robot, round(t, 2)))
            ev = rs.get(robot, [])
            # the previous reset and the next one after this fix (segments)
            prev_t = max([e[0] for e in ev if e[0] < t - 0.01] or [None]) if ev else None
            next_t = min([e[0] for e in ev if e[0] > t + 0.01] or [1e18])
            mine = by_robot.get(robot, [])
            seg = [l for l in mine if prev_t is not None and prev_t < l["t0"] and l["t1"] < t]
            nxt = [l for l in mine if t < l["t0"] and l["t1"] < next_t]
            meas = [l for l in seg if l["cam_dir"] is not None and l["th0"] is not None]
            meas_n = [l for l in nxt if l["cam_dir"] is not None and l["th0"] is not None]
            e_first = w(meas[0]["th0"] - meas[0]["cam_dir"]) if meas else None
            e_last = w(meas[-1]["th0"] - meas[-1]["cam_dir"]) if meas else None
            # turning between the last measured leg and the fix (the gyro's error on it is not in e_last)
            turn_after_last = sum(abs(l["value"] or 0) for l in seg if l["kind"] == "turn" and meas and l["t0"] > meas[-1]["t0"])
            turn_seg = sum(abs(l["value"] or 0) for l in seg if l["kind"] == "turn")
            e_next = w(meas_n[0]["th0"] - meas_n[0]["cam_dir"]) if meas_n else None
            turn_before_next = sum(abs(l["value"] or 0) for l in nxt if l["kind"] == "turn" and meas_n and l["t0"] < meas_n[0]["t0"])
            dth = c["dth"]
            r_err = None if (dth is None or e_last is None) else w(dth + e_last)
            # what the previous reset wrote, per the robot: its error at the first measured leg
            prev = [e for e in ev if e[0] < t - 0.01]
            prev_kind = prev[-1][1] if prev else ""
            prev_row = prev[-1][2] if prev else {}
            prev_cam_th = C.num(prev_row.get("cam_th")) if prev_kind == "fix" else None
            prev_pos_only = prev_kind == "fix" and (prev_cam_th is None or (isinstance(prev_cam_th, float) and math.isnan(prev_cam_th)))
            cause = ""
            if dth is not None and abs(dth) >= 30:
                if e_last is None and e_next is None:
                    cause = "unmeasured"
                else:
                    rob_bad = e_last is not None and abs(e_last) >= BIG
                    run_bad = (r_err is not None and abs(r_err) >= BIG) or (r_err is None and e_next is not None and abs(e_next) >= BIG)
                    if run_bad and not rob_bad:
                        cause = "runner"
                    elif rob_bad and run_bad:
                        cause = "both"
                    elif rob_bad:
                        cause = "written" if (e_first is not None and abs(e_first) >= BIG) else "grew"
                    else:
                        cause = "neither"
            rows_out.append(dict(
                session=name, robot=robot, t=t, hm=C.hm(t), corr=c["corr"], dist=c["dist"], turned=c["turned"],
                n_legs=c["n_legs"], prev_dt=c["prev_dt"], dth=dth, cam_th=None if r is None else C.num(r["cam_th"]),
                before_th=None if r is None else C.num(r["before_th"]), e_first=e_first, e_last=e_last, r_err=r_err,
                e_next=e_next, turn_after_last=turn_after_last, turn_before_next=turn_before_next, turn_seg=turn_seg,
                n_meas=len(meas), prev_kind=prev_kind, prev_pos_only=prev_pos_only, cause=cause,
                seq=" ".join("%s%+.0f%s" % (l["kind"][0], l["value"] or 0, "" if l["result"] == "done" else "!")
                             for l in seg)[:120]))
    big = [x for x in rows_out if x["dth"] is not None and abs(x["dth"]) >= 30]
    hd = [x for x in rows_out if x["dth"] is not None]
    print("Chains (as unc_fit.py): %d; with a heading at the fix %d; |dth| >= 30: %d (%.0f%%)" % (
        len(rows_out), len(hd), len(big), 100.0 * len(big) / max(1, len(hd))))

    # --- hypothesis 1: quarter-turn clustering of dth
    a = np.array([abs(x["dth"]) for x in big])
    print("\nH1 quarter turns: |dth| of the big ones, in 10-deg bins from 30:")
    hist, edges = np.histogram(a, bins=np.arange(30, 190, 10))
    print("  " + " ".join("%d-%d:%d" % (edges[i], edges[i + 1], hist[i]) for i in range(len(hist))))
    near = lambda v, c_, tol=7.0: abs(v - c_) <= tol
    k90 = sum(near(v, 90) for v in a)
    k180 = sum(near(v, 180) or v >= 173 for v in a)
    # a uniform spread over 30-180 puts 14/150 of them within 7 deg of 90 and 7/150 within 7 of 180
    print("  within 7 deg of 90: %d (uniform over 30-180 would give %.1f); of 180: %d (%.1f)" % (
        k90, len(a) * 14 / 150.0, k180, len(a) * 7 / 150.0))

    # --- the main split
    print("\nCause of each |dth| >= 30 (robot error e = own heading - camera direction of a go leg >= %.0f cm; "
          "runner error r = dth + e_last; BIG = %.0f deg):" % (MIN_MOVE_CM, BIG))
    cnt = {}
    for x in big:
        cnt[x["cause"]] = cnt.get(x["cause"], 0) + 1
    for k, v in sorted(cnt.items(), key=lambda kv: -kv[1]):
        q = [x["corr"] for x in big if x["cause"] == k]
        print("  %-10s n %3d  position correction median %5.1f cm" % (k, v, np.median(q)))
    # cross-check r_err with e_next (the next segment's first error = what the fix wrote)
    xx = [(x["r_err"], x["e_next"]) for x in rows_out if x["r_err"] is not None and x["e_next"] is not None
          and x["turn_before_next"] <= 90 and x["turn_after_last"] <= 90]
    if xx:
        d = np.array([abs(w(p - q)) for p, q in xx])
        print("  check: runner error from before the fix vs the robot's error after it (turns <= 90 deg between): "
              "n %d, |difference| median %.1f deg, 90%% %.1f" % (len(d), np.median(d), np.percentile(d, 90)))
    # all chains (not just big): how often each side is wrong
    allr = [x for x in rows_out if x["r_err"] is not None]
    print("  all chains with both measured: n %d; runner |r| >= %.0f: %d (%.0f%%); robot |e_last| >= %.0f: %d (%.0f%%)" % (
        len(allr), BIG, sum(abs(x["r_err"]) >= BIG for x in allr), 100.0 * np.mean([abs(x["r_err"]) >= BIG for x in allr]),
        BIG, sum(abs(x["e_last"]) >= BIG for x in allr), 100.0 * np.mean([abs(x["e_last"]) >= BIG for x in allr])))

    # does the position correction follow the robot's heading error or the runner's?
    pr = [x for x in rows_out if x["e_last"] is not None and x["r_err"] is not None]
    for lab, f in (("robot wrong (|e_last| >= BIG)", lambda x: abs(x["e_last"]) >= BIG),
                   ("robot right, runner wrong", lambda x: abs(x["e_last"]) < BIG and abs(x["r_err"]) >= BIG),
                   ("both right", lambda x: abs(x["e_last"]) < BIG and abs(x["r_err"]) < BIG)):
        q = [x["corr"] for x in pr if f(x)]
        if q:
            print("  %-30s n %3d  position correction median %5.1f cm, 90%% %5.1f" % (
                lab, len(q), np.median(q), np.percentile(q, 90)))

    # --- the robot's own error: written in vs grew; gyro per degree turned
    print("\nRobot's own heading error within a segment (first -> last measured leg), by turning between them:")
    gr = [(x["turn_seg"], x["e_first"], x["e_last"]) for x in rows_out if x["n_meas"] >= 2]
    for lo, hi in ((0, 90), (90, 270), (270, 720), (720, 1e9)):
        q = [abs(w(l - f)) for t_, f, l in gr if lo <= t_ < hi]
        if q:
            print("  turned %4d-%-5s n %3d  |growth| median %4.1f deg, 90%% %5.1f, max %5.1f" % (
                lo, "" if hi > 1e8 else "%d" % hi, len(q), np.median(q), np.percentile(q, 90), max(q)))
    ef = [x["e_first"] for x in rows_out if x["e_first"] is not None]
    print("  error at the first measured leg after a reset: n %d, |e| >= BIG: %d (%.0f%%)" % (
        len(ef), sum(abs(v) >= BIG for v in ef), 100.0 * np.mean([abs(v) >= BIG for v in ef])))

    # --- per leg: every measured go leg's error, and what the reset before it was
    write = os.path.join(C.OUT, "heading_legs.csv")
    C.write_csv(write, ["session", "robot", "t", "hm", "id", "value", "result", "th0", "cam_dir", "err", "moved"],
                [[l["session"], l["robot"], round(l["t0"], 2), C.hm(l["t0"]), l["id"], l["value"], l["result"],
                  l["th0"], round(l["cam_dir"], 1), round(w(l["th0"] - l["cam_dir"]), 1), round(l["moved"], 1)]
                 for l in legs_out if l["th0"] is not None])

    # --- robots, sessions
    print("\nBy robot / session (|dth| >= 30 share; causes):")
    for key in ("robot", "session"):
        vals = sorted(set(x[key] for x in hd))
        for v in vals:
            q = [x for x in hd if x[key] == v]
            b = [x for x in q if abs(x["dth"]) >= 30]
            cs = {}
            for x in b:
                cs[x["cause"]] = cs.get(x["cause"], 0) + 1
            print("  %-14s n %3d  big %3d (%2.0f%%)  %s" % (v, len(q), len(b), 100.0 * len(b) / len(q),
                                                         ", ".join("%s %d" % kv for kv in sorted(cs.items()))))

    C.write_csv(os.path.join(C.OUT, "heading_chains.csv"),
                ["session", "robot", "t", "hm", "corr_cm", "dist_cm", "turned_deg", "n_legs", "prev_dt_s", "dth_deg",
                 "runner_th_sent", "robot_th_before", "robot_err_first", "robot_err_last", "runner_err",
                 "robot_err_next", "turn_after_last", "turn_before_next", "turn_seg", "n_meas", "prev_kind",
                 "prev_pos_only", "cause", "legs"],
                [[x["session"], x["robot"], round(x["t"], 2), x["hm"], round(x["corr"], 1), round(x["dist"], 1),
                  round(x["turned"], 1), x["n_legs"], round(x["prev_dt"], 1), _r(x["dth"]), _r(x["cam_th"]),
                  _r(x["before_th"]), _r(x["e_first"]), _r(x["e_last"]), _r(x["r_err"]), _r(x["e_next"]),
                  round(x["turn_after_last"]), round(x["turn_before_next"]), round(x["turn_seg"]), x["n_meas"],
                  x["prev_kind"], x["prev_pos_only"], x["cause"], x["seq"]] for x in rows_out])
    return rows_out


def _r(v):
    return "" if v is None else round(v, 1)


def reset_kind(r, after_legs, face):
    """Which runner call sent this fix, from its signature (fix.csv has no call-site field):
    placeholder  cam_th exactly the board's face_xboard (+-90.0) and the next legs are acquire_heading's ('turn 2'
                 front check and / or a 10-15 cm heading drive): begin_moves() with heads[robot] None
    position     no heading sent ('-': nan)
    heading      any other fix with a heading."""
    th = C.num(r.get("cam_th"))
    if th is None or math.isnan(th):
        return "position"
    nxt = [(l["kind"], round(l["value"] or 0, 1)) for l in after_legs[:3]]
    acq = any(k == "turn" and v == 2.0 for k, v in nxt) or any(k == "go" and v in (10.0, 15.0) for k, v in nxt[:2])
    if abs(th - face) < 0.05 and acq:
        return "placeholder"
    return "heading"


def reset_table(names=U.ALL, max_turn_before=90.0, max_gap_s=120.0):
    """Every reset (heading fix or ok find) with the robot's heading error just before it (last measured leg of the
    previous segment) and just after it (first measured leg after, with at most max_turn_before deg turned first)."""
    out = []
    for name in names:
        try:
            lt = leg_table(name)
            rs = resets(name)
            board = C.NC.state(name).get("board") or {}
        except (OSError, KeyError):
            continue
        import e02_plan as PL
        try:
            face = PL.Board.from_dict(board).face_xboard if board else 90.0
        except Exception:  # noqa: BLE001
            face = 90.0
        by_robot = {}
        for l in lt:
            by_robot.setdefault(l["robot"], []).append(l)
        for robot, ev in rs.items():
            mine = by_robot.get(robot, [])
            for i, (t, kind, r) in enumerate(ev):
                t_next = ev[i + 1][0] if i + 1 < len(ev) else 1e18
                t_prev = ev[i - 1][0] if i > 0 else -1e18
                after = [l for l in mine if t < l["t0"] and l["t1"] < t_next and l["t0"] < t + max_gap_s]
                before = [l for l in mine if t_prev < l["t0"] and l["t1"] < t]
                ma = None
                turned = 0.0
                for l in after:
                    if l["kind"] == "go" and l["cam_dir"] is not None and l["th0"] is not None:
                        ma = l
                        break
                    if l["kind"] == "turn":
                        turned += abs(l["value"] or 0)
                mb = [l for l in before if l["kind"] == "go" and l["cam_dir"] is not None and l["th0"] is not None]
                mb = mb[-1] if mb else None
                tb = sum(abs(l["value"] or 0) for l in before if l["kind"] == "turn" and mb and l["t0"] > mb["t0"])
                if kind == "fix":
                    rk = reset_kind(r, after, face)
                    sent = C.num(r.get("cam_th"))
                    bth = C.num(r.get("before_th"))
                    dth = None if (sent is None or bth is None or math.isnan(sent) or math.isnan(bth)) else w(sent - bth)
                else:
                    rk, sent, dth = "find", C.num(r.get("after_th")), None
                out.append(dict(session=name, robot=robot, t=t, hm=C.hm(t), kind=rk, sent=sent, dth=dth,
                                err_after=None if (ma is None or turned > max_turn_before) else w(ma["th0"] - ma["cam_dir"]),
                                turn_before_meas=turned,
                                err_before=None if (mb is None or tb > max_turn_before) else w(mb["th0"] - mb["cam_dir"]),
                                n_after=len(after), first_legs=" ".join("%s%+.0f" % (l["kind"][0], l["value"] or 0) for l in after[:4]),
                                gap_prev=t - t_prev))
    return out


def resets_report():
    rt = reset_table()
    print("\nEvery reset: the heading error it wrote in (robot error at the first measured leg after it, <= 90 deg "
          "turned first) and the robot's error just before it:")
    print("  %-12s %5s %9s %14s %16s %16s" % ("kind", "n", "measured", "wrote >= 20", "was right before", "made it wrong"))
    for k in ("placeholder", "heading", "position", "find"):
        q = [x for x in rt if x["kind"] == k]
        m = [x for x in q if x["err_after"] is not None]
        bad = [x for x in m if abs(x["err_after"]) >= BIG]
        rb = [x for x in m if x["err_before"] is not None]
        made = [x for x in rb if abs(x["err_before"]) < BIG and abs(x["err_after"]) >= BIG]
        print("  %-12s %5d %9d %8d (%2.0f%%) %16d %16d" % (k, len(q), len(m), len(bad), 100.0 * len(bad) / max(1, len(m)),
                                                         sum(abs(x["err_before"]) < BIG for x in rb), len(made)))
    C.write_csv(os.path.join(C.OUT, "heading_resets.csv"),
                ["session", "robot", "t", "hm", "kind", "heading_sent", "dth_sent_minus_own", "err_before", "err_after",
                 "turn_before_meas", "first_legs"],
                [[x["session"], x["robot"], round(x["t"], 2), x["hm"], x["kind"], _r(x["sent"]), _r(x["dth"]),
                  _r(x["err_before"]), _r(x["err_after"]), round(x["turn_before_meas"]), x["first_legs"]] for x in rt])
    return rt


if __name__ == "__main__":
    main()
    resets_report()
