"""Analysis C (2026-10-24 night): picture delay, the computer's work per picture, the robots' WiFi round trips, and
battery drain driving vs standing.

  C1 perf.csv (one row a second, written by the runner): pictures handled a second, pictures dropped, the tracker's
     ms per picture (mean in the second, the second's worst), a picture's age when its position was ready (capture
     to position: camera + transfer + waiting + processing). Per session; "late" = a second whose worst age > 1 s
     (e02.py LATE_PICTURE_S).
  C2 robot-logs clock_sync.csv: the runner pings each robot 3 times every ~10 s; per burst the shortest round trip
     (the least-delayed ping: the network's floor at that moment). Per session and robot.
  C3 battery %: every reading the logs hold (runinfo, runend, raw rows, find / e02 / gyro records, the log's
     "battery NN%" lines), one value a minute per robot (the median), against the robot's time driving (raw rows
     with the wheels turning, plus go / turn legs outside runs from leg.csv) and standing. Fit per robot over the
     afternoon's stages: battery = b0 - idle_rate x hours - extra_rate x hours driving. Plus Right's 63% (17:32) to
     35% (19:35) from the night session's log (given; that log is not in this folder set).

Usage: python3 night_latency_battery.py"""
import collections
import math
import re
import sys

import numpy as np

import night_common as N

PERF_SESS = ("oct23-morning", "oct23-evening", "stage1", "stage2", "stage3")
LATE_S = 1.0
MOVING_RPM = 5.0          # chosen: |left rpm| + |right rpm| above this counts as driving
LOG_DAY = {"stage1": "2026-10-24", "stage2": "2026-10-24", "stage3": "2026-10-24",
           "oct23-morning": "2026-10-23", "oct23-evening": "2026-10-23"}


def c1_perf():
    print("\n#### C1 camera processing per session (perf.csv, one row a second)")
    print("| session | where it ran | seconds | pictures/s handled (median) | dropped share | ms a picture: median / 95% of the "
          "seconds' worst | picture age s: median / 95% / worst | late seconds (worst > 1 s) |")
    for name in PERF_SESS:
        rr = N.rows(name, "perf.csv")
        if not rr:
            continue
        f = lambda k: [N.num(r[k]) for r in rr if N.num(r[k]) is not None]
        fps, drop, msm, msx, am, ax = f("fps_handled"), f("dropped"), f("ms_mean"), f("ms_max"), f("age_mean_s"), \
            f("age_max_s")
        handled = sum(fps)
        where = "laptop" if name.startswith("oct23") else "Pi"
        print("| %s | %s | %d | %.1f | %.0f%% | %s | %.2f / %.2f / %.2f | %.1f%% (%d) |" % (
            name, where, len(rr), N.median(fps), 100.0 * sum(drop) / max(1.0, sum(drop) + handled),
            "%.0f / %.0f" % (N.median(msm), N.pct(msx, 0.95)) if max(msm) > 0 else "not recorded",
            N.median(am), N.pct(ax, 0.95), max(ax), 100.0 * sum(a > LATE_S for a in ax) / len(ax), sum(a > LATE_S for a in ax)))


def bursts(name):
    """robot -> [(t, shortest rtt ms of the burst, n pings)]"""
    rr = sorted(N.robot_rows(name, "clock_sync.csv"), key=lambda r: float(r["t_send"]))
    by = collections.defaultdict(list)
    for r in rr:
        by[r["robot"]].append((float(r["t_send"]), float(r["rtt_ms"])))
    out = {}
    for robot, v in by.items():
        b, cur = [], []
        for t, rtt in v:
            if cur and t - cur[-1][0] > 1.0:
                b.append(cur)
                cur = []
            cur.append((t, rtt))
        if cur:
            b.append(cur)
        out[robot] = [(c[0][0], min(x[1] for x in c), len(c)) for c in b]
    return out


def c2_rtt():
    print("\n#### C2 WiFi round trip to each robot (robot-logs clock_sync.csv; per burst of 3 pings, the shortest)")
    print("| session | robot | bursts | median ms | 90% | 99% | worst | bursts over 100 ms |")
    for name in PERF_SESS:
        b = bursts(name)
        for robot in N.ROBOTS:
            v = [x[1] for x in b.get(robot, [])]
            if not v:
                continue
            print("| %s | %s | %d | %.0f | %.0f | %.0f | %.0f | %.1f%% |" % (
                name, robot, len(v), N.median(v), N.pct(v, 0.9), N.pct(v, 0.99), max(v),
                100.0 * sum(x > 100 for x in v) / len(v)))


def log_battery(name):
    """(t, robot, %) from the log's 'Robot: ... battery NN%' lines (local time, the session's day)."""
    out = []
    import calendar
    day = LOG_DAY[name]
    y, m, d = (int(x) for x in day.split("-"))
    p = N.path(name, "e02_log.txt")
    with open(p, encoding="utf-8", errors="replace") as f:
        for ln in f:
            mm = re.match(r"\[(\d\d):(\d\d):(\d\d)\]\s+(\w+):.*battery (\d+)%", ln)
            if mm and mm.group(4) in N.ROBOTS:
                hh, mi, ss = int(mm.group(1)), int(mm.group(2)), int(mm.group(3))
                t = calendar.timegm((y, m, d, hh, mi, ss)) - N.TZ_H * 3600
                out.append((t, mm.group(4), float(mm.group(5))))
    return out


def battery_readings(name):
    out = []
    for fn, tk in (("runinfo.csv", "t_recv"), ("runend.csv", "t_recv"), ("find.csv", "t_recv"), ("e02.csv", "t_recv"),
                   ("gyro.csv", "t_recv")):
        for r in N.robot_rows(name, fn):
            b, t = N.num(r.get("battery")), N.num(r.get(tk))
            if b is not None and t is not None and b > 0:
                out.append((t, r["robot"], b))
    for run, rr in N.raw_by_run(name).items():
        for r in rr:
            if r["battery"] is not None and r["t_recv"] is not None and r["battery"] > 0:
                out.append((r["t_recv"], r["robot"], r["battery"]))
    out += log_battery(name)
    return out


def driving_intervals(name):
    """robot -> [(t0, t1)] driving: raw rows with the wheels turning (gaps under 0.2 s joined), and go / turn legs
    (started -> done / failed) outside runs."""
    out = collections.defaultdict(list)
    for run, rr in N.raw_by_run(name).items():
        rr = sorted((r for r in rr if r["t_recv"] is not None and r["wl_rpm"] is not None), key=lambda r: r["t_recv"])
        cur = None
        for r in rr:
            mv = abs(r["wl_rpm"]) + abs(r["wr_rpm"] or 0) > MOVING_RPM
            if mv:
                if cur and r["t_recv"] - cur[1] < 0.2:
                    cur[1] = r["t_recv"]
                else:
                    if cur:
                        out[r["robot"]].append(tuple(cur))
                    cur = [r["t_recv"], r["t_recv"]]
        if cur:
            out[rr[0]["robot"]].append(tuple(cur))
    started = {}
    for r in N.robot_rows(name, "leg.csv"):
        if r.get("run") not in ("-", ""):
            continue
        key = (r["robot"], r["id"])
        t = N.num(r["t_recv"])
        if r["result"] == "started":
            started[key] = t
        elif r["result"] in ("done", "failed", "stopped") and key in started:
            out[r["robot"]].append((started.pop(key), t))
    return {k: merge(v) for k, v in out.items()}


def merge(iv):
    iv = sorted(iv)
    out = []
    for a, b in iv:
        if out and a <= out[-1][1]:
            out[-1] = (out[-1][0], max(out[-1][1], b))
        else:
            out.append((a, b))
    return out


def driving_s(iv, t0, t1):
    return sum(max(0.0, min(b, t1) - max(a, t0)) for a, b in iv)


def per_minute(readings):
    by = collections.defaultdict(list)
    for t, b in readings:
        by[int(t // 60)].append(b)
    return sorted((k * 60.0 + 30.0, float(np.median(v))) for k, v in by.items())


def c3_battery():
    print("\n#### C3 battery")
    groups = (("2026-10-23 morning", ("oct23-morning",)), ("2026-10-23 evening", ("oct23-evening",)),
              ("2026-10-24 Pi stages 1-3", ("stage1", "stage2", "stage3")))
    print("| day / sessions | robot | first reading | last reading | hours | hours driving | % used | "
          "fit: standing %/h | fit: driving %/h | fit rms % |")
    fits = {}
    for lab, names in groups:
        rd = collections.defaultdict(list)
        drv = collections.defaultdict(list)
        for name in names:
            for t, r, b in battery_readings(name):
                rd[r].append((t, b))
            for r, iv in driving_intervals(name).items():
                drv[r] += iv
        for robot in N.ROBOTS:
            pm = per_minute(rd.get(robot, []))
            if len(pm) < 3:
                continue
            iv = merge(drv.get(robot, []))
            t0 = pm[0][0]
            X, y = [], []
            for t, b in pm:
                dh = driving_s(iv, t0, t) / 3600.0
                X.append([1.0, (t - t0) / 3600.0, dh])
                y.append(b)
            X, y = np.array(X), np.array(y)
            beta, *_ = np.linalg.lstsq(X, y, rcond=None)
            rms = float(np.sqrt(np.mean((y - X @ beta) ** 2)))
            hrs = (pm[-1][0] - t0) / 3600.0
            dh = driving_s(iv, t0, pm[-1][0]) / 3600.0
            fits[(lab, robot)] = (-beta[1], -(beta[1] + beta[2]))
            print("| %s | %s | %s %.0f%% | %s %.0f%% | %.2f | %.2f | %.0f | %.1f | %.1f | %.2f |" % (
                lab, robot, N.hm(pm[0][0]), pm[0][1], N.hm(pm[-1][0]), pm[-1][1], hrs, dh, pm[0][1] - pm[-1][1],
                -beta[1], -(beta[1] + beta[2]), rms))
    # gaps between sessions with no driving at all (the robots on, waiting): the cleanest standing drain
    print("\nStanding gaps between the Pi stages (no readings, no driving in between): last reading before, first after")
    print("| gap | robot | before | after | minutes | % lost | %/h |")
    pairs = (("stage1", "stage2"), ("stage2", "stage3"))
    for a, b in pairs:
        ra, rb = battery_readings(a), battery_readings(b)
        for robot in N.ROBOTS:
            la = sorted((t, x) for t, r, x in ra if r == robot)
            lb = sorted((t, x) for t, r, x in rb if r == robot)
            if not la or not lb:
                continue
            # the last minute's median before, the first minute's after (1% steps: the reading is an integer)
            ta, xa = la[-1]
            tb, xb = lb[0]
            mins = (tb - ta) / 60.0
            print("| %s -> %s | %s | %s %.0f%% | %s %.0f%% | %.0f | %.0f | %.1f |" % (
                a, b, robot, N.hm(ta), xa, N.hm(tb), xb, mins, xa - xb, 60.0 * (xa - xb) / mins if mins else 0))
    # Right, 17:32 -> 19:35 (the 35% from the night session's log, given in the brief)
    r3 = sorted((t, x) for t, r, x in battery_readings("stage3") if r == "Right")
    iv = driving_intervals("stage3").get("Right", [])
    t35 = r3[0][0] + (19 * 60 + 35 - (17 * 60 + 32)) * 60.0
    d = driving_s(iv, r3[0][0], t35)
    print("\nRight: %s %.0f%% -> 19:35 35%% (night log, given): %.2f h, of it %.0f s driving in stage 3 (its explore "
          "turns); %.1f %%/h" % (N.hm(r3[0][0]), r3[0][1], (t35 - r3[0][0]) / 3600.0, d,
                                (r3[0][1] - 35.0) / ((t35 - r3[0][0]) / 3600.0)))
    # does the reading sag while driving? within each run: the reading while the wheels turn against standing
    print("\nWithin runs (raw rows, 2026-10-24 stages 1-2): battery while driving minus while standing, same run")
    diffs = collections.defaultdict(list)
    for name in ("stage1", "stage2"):
        for run, rr in N.raw_by_run(name).items():
            mv = [r["battery"] for r in rr if r["wl_rpm"] is not None and abs(r["wl_rpm"]) + abs(r["wr_rpm"] or 0) > MOVING_RPM]
            st = [r["battery"] for r in rr if r["wl_rpm"] is not None and abs(r["wl_rpm"]) + abs(r["wr_rpm"] or 0) <= 0.5]
            if len(mv) > 50 and len(st) > 50:
                diffs[rr[0]["robot"]].append(np.mean(mv) - np.mean(st))
    for robot in N.ROBOTS:
        v = diffs.get(robot, [])
        if v:
            print("  %s: %d runs, mean %+.2f %%, range %+.2f..%+.2f" % (robot, len(v), np.mean(v), min(v), max(v)))
    return fits


def main():
    c1_perf()
    c2_rtt()
    print("\n(The night session e02-20261024-193526, Right alone with the 5.8 GB upload: its release "
          "latency-test-20261024 holds only the video parts and rec-old-small.zip, the 15:36-17:39 recording's frame "
          "index; no logs zip yet, so it is not in these tables.)")
    c3_battery()


if __name__ == "__main__":
    sys.exit(main())
