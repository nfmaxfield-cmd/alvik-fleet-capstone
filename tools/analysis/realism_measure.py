#!/usr/bin/env python3
"""Realism measures from a real free-roam run's logs (logs only; nothing touches a robot or a Pi).

Version realism-measure-2026-11-12a (hard simulator build). Changelog:
  2026-11-12a  first version: what the fast simulator's "measured" profile is built from, and the same measures the
               validation compares the simulator against (tools/fleet/fastsim_realism.py measure_sim()).

Each measure is a DISTRIBUTION (quantiles) or a rate per hour, per run. Definitions (the simulator side uses the same):
  pictures_s      pictures the camera bridge handled a second: distinct picture times in bridge/track.csv per 10 s
                  window (perf.csv's fps_handled is the recorder's rate, not the bridge's)
  fix_delay_s     a named fix's arrival at the run less its picture's time (camera_track.csv arrived - t)
  delay_windows   (10th, 50th, 90th percentile) of fix_delay_s in each 30 s window: the simulator's delay regimes
  quiet_s         the robots' own "time since the Pi was last heard" (follow.csv quiet_ms), every FOLLOW record
  timeline        the run's own timing for the simulator's replay: each named picture's (time, delay), the Pi-wide
                  silences (two or more robots' quiet spells overlapping: their common part) and each robot's own
  stalls          quiet spells: quiet_ms climbing past 1.0 s, one spell per climb; the spell's length is its top value;
                  "shared" when another robot's spell overlaps it (a Pi-wide stall, not one robot's WiFi)
  watchdog_per_h  steps the robots' watchdog stopped (step.csv failed "watchdog"), per robot-hour
  unnamed         spells where the bridge saw a robot (bridge/track.csv "seen") but the run's tracking layer named no
                  fix for it (camera_track.csv) for over max(1 s, 2.5 pictures); 2+ bridge sightings inside (picture_spells)
  cam_gaps        the bridge missing a robot from 2 or more pictures in a row (outline / light dropouts), in seconds;
                  seen_pct: the share of pictures that saw it; src_share: which camera the sightings came from
  own_cam_gap_cm  the robot's own pose (follow.csv) against the bridge's sighting within 0.3 s, once a second
  standing_sd_cm  each source's scatter on a standing robot: bridge sightings during 3 s with its own travel_cm still,
                  idle and its own heading within 2 deg (a turn on the spot moves the outline's middle)
  slips           wheels turning, body not: 1 s windows where the wheels' own travel (raw.csv wl_rpm / wr_rpm, the
                  mean of the two wheels, radius 1.7 cm: a turn on the spot is no travel) is 2 cm or more
                  and the camera's is under 30 % of it; consecutive windows are one slip; rate per robot-hour, length
  curve_ratio     arcfit.csv k_ratio (camera curvature / asked) per robot: the curve over-turn
  heading_err_deg own heading against the camera's course when the run noticed ("off plan ... its own heading A deg,
                  the camera sees it go B"; "heading re-set ... drove D deg off"), per hour and its size
  tilt            raw.csv roll / pitch away from each robot's level (its median): spells over 3 deg, per robot-hour
  close           10 s windows whose closest pair of robot middles came under 15 / 13.44 / 11 cm, per hour
  edge_offset_cm  surface_marks.csv cam_edge_cm: where the camera put the robot's middle against the edge when its
                  own line sensors found the edge (0 = the camera's edge line is exactly right for a 4 cm nose)
  did_not_move    "did not move" events per robot-hour (events.csv keeps 400 of a kind: the rate is taken over the
                  time the first 400 took when the cap is hit)
  lost / holds    lost.csv spells (standing = blind_cm under 1), NEEDS A HAND and REJOINED events per hour

    python3 tools/analysis/realism_measure.py RUN_FOLDER [RUN_FOLDER ...] --out measured.json [--no-raw]
"""
import argparse
import bisect
import csv
import json
import math
import os
import re
import statistics
import sys

VERSION = "realism-measure-2026-11-12a"
WHEEL_R_CM = 1.7       # the Alvik's nominal wheel radius (tools/fleet/calib_run.py)
STALL_S = 1.0          # a quiet spell starts here: 2.5 x the 0.4 s a fix normally takes at 4-5 pictures a second
Q = (0.05, 0.1, 0.25, 0.5, 0.75, 0.9, 0.95, 0.99)


def quant(v, qs=Q):
    v = sorted(x for x in v if x is not None and not (isinstance(x, float) and math.isnan(x)))
    if not v:
        return None
    return {("p%d" % round(q * 100)): round(v[min(len(v) - 1, int(q * len(v)))], 4) for q in qs} | dict(
        n=len(v), mean=round(sum(v) / len(v), 4), max=round(v[-1], 4))


def _csv(folder, name):
    p = os.path.join(folder, name)
    if not os.path.exists(p):
        return []
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


def _f(v, d=None):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


def spells(times, gap):
    """Gaps longer than `gap` between consecutive times: (start, end)."""
    return [(a, b) for a, b in zip(times, times[1:]) if b - a > gap]


def quiet_spells(by):
    """by: robot -> [(t, quiet_s)]. Returns (all quiet values, spells (robot, t0, t1, top), spells shared with
    another robot). A spell starts when quiet passes STALL_S; it heard something and climbed again = a new spell."""
    qs, st_all = [], []
    for n, rows in by.items():
        rows.sort()
        qs.extend(q for _, q in rows)
        cur = None
        for t, q in rows:
            if q >= STALL_S:
                if cur is None:
                    cur = [t - q, t, q]
                elif q < cur[2] - 0.15:
                    st_all.append((n, cur[0], cur[1], cur[2]))
                    cur = [t - q, t, q]
                else:
                    cur[1], cur[2] = t, max(cur[2], q)
            elif cur is not None:
                st_all.append((n, cur[0], cur[1], cur[2]))
                cur = None
        if cur is not None:
            st_all.append((n, cur[0], cur[1], cur[2]))
    shared = sum(1 for (n, a, b, q) in st_all if any(m != n and a2 < b and a < b2 for m, a2, b2, _ in st_all))
    return qs, st_all, shared


def picture_spells(pics, seen, named, names, min_missed=2, unnamed_s=1.0, k_int=2.5):
    """Per robot (the same definition for the real logs and the simulator):
    gaps     the robot missing from min_missed or more pictures in a row; length = from the picture before the gap to
             the one after it (seconds)
    unnamed  two named fixes further apart than max(unnamed_s, k_int x the picture interval there) with the camera
             seeing the robot at least twice between them (the bridge's picture times and the named fixes' stamps
             differ by tens of ms in the logs, so this is by time, not picture by picture)
    seen_pct the share of pictures with the robot seen (from its first sighting to its last)."""
    gaps, un, sp = [], [], {}
    pics = sorted(pics)
    for n in names:
        S, N = sorted(seen.get(n, ())), sorted(named.get(n, ()))
        if not S:
            continue
        Sset = set(S)
        a, b = S[0], S[-1]
        i0, i1 = bisect.bisect_left(pics, a), bisect.bisect_right(pics, b)
        P = pics[i0:i1]
        sp[n] = round(100.0 * len(S) / max(1, len(P)), 1)
        miss, last = 0, None
        for p in P:
            if p in Sset:
                if miss >= min_missed and last is not None:
                    gaps.append(p - last)
                miss, last = 0, p
            else:
                miss += 1
        for x, y in zip(N, N[1:]):
            if y - x <= unnamed_s:
                continue
            k = bisect.bisect_left(pics, x)
            loc = pics[max(0, k - 20):k + 1]
            iv = sorted(q2 - q1 for q1, q2 in zip(loc, loc[1:]))
            thr = max(unnamed_s, k_int * (iv[len(iv) // 2] if iv else 0.0))
            if y - x > thr and bisect.bisect_left(S, y) - bisect.bisect_right(S, x) >= 2:
                un.append(y - x)
    return dict(gaps=gaps, unnamed=un, seen_pct=sp)


def close_rates(seen, hours):
    """10 s windows whose closest pair of robot middles (bridge sightings, matched to the half second) came under
    15 / 13.44 (the 6.72 cm circles round the bodies touch) / 11 cm, per hour."""
    by = {}
    for n, rows in seen.items():
        for q in rows:
            by.setdefault(round(q[0] * 2) / 2, {})[n] = (q[1], q[2])
    win = {}
    for t, d in by.items():
        ns = list(d)
        for i in range(len(ns)):
            for j in range(i + 1, len(ns)):
                dd = math.dist(d[ns[i]], d[ns[j]])
                win[int(t // 10)] = min(win.get(int(t // 10), 999.0), dd)
    v = list(win.values())
    return {k: round(sum(1 for x in v if x < lim) / max(1e-6, hours), 1) for k, lim in (
        ("under_15", 15.0), ("under_13_44", 13.44), ("under_11", 11.0))}


def measure(folder, raw=True):
    out = dict(run=os.path.basename(folder.rstrip("/")), version=VERSION)
    follow = [r for r in _csv(folder, "follow.csv") if r.get("state") != "nopose"]
    names = sorted({r["robot"] for r in follow})
    out["robots"] = names
    tt = [float(r["t_recv"]) for r in follow]
    if not tt:
        return out
    t0, t1 = min(tt), max(tt)
    hours = (t1 - t0) / 3600.0
    out["hours"] = round(hours, 3)
    rh = hours * max(1, len(names))           # robot-hours
    # -- pictures and fix delay ------------------------------------------------------------------------------------
    ct = _csv(folder, "camera_track.csv")
    dl = [(float(r["t"]), float(r["arrived"]) - float(r["t"])) for r in ct if r.get("arrived")]
    out["fix_delay_s"] = quant([d for _, d in dl])
    win = {}
    for t, d in dl:
        win.setdefault(int((t - t0) // 30), []).append(d)
    wins = []
    for k in sorted(win):
        v = sorted(win[k])
        if len(v) >= 10:
            wins.append((round(v[int(0.1 * len(v))], 3), round(v[len(v) // 2], 3), round(v[int(0.9 * len(v))], 3)))
    out["delay_windows"] = wins
    # -- the Pi heard: quiet spells and the watchdog ----------------------------------------------------------------
    by = {}
    for r in follow:
        by.setdefault(r["robot"], []).append((float(r["t_recv"]), _f(r.get("quiet_ms"), 0.0) / 1000.0))
    qs, st_all, shared = quiet_spells(by)
    out["quiet_s"] = quant(qs)
    out["stalls"] = dict(per_robot_h=round(len(st_all) / rh, 2), length_s=quant([q for *_, q in st_all]),
                         shared_pct=round(100.0 * shared / max(1, len(st_all)), 1),
                         over_1_5s_per_robot_h=round(sum(1 for *_, q in st_all if q >= 1.5) / rh, 2))
    # the timeline (the simulator's replay): each spell's silence [last heard, heard again]; overlapping silences of
    # two or more robots are one Pi-wide stall (their common part), the rest one robot's WiFi
    sil = sorted((t1 - top, t1 + 0.05, n) for n, _, t1, top in st_all)
    pi, wifi, used = [], {}, set()
    for i, (a, b, n) in enumerate(sil):
        if i in used:
            continue
        grp = [(a, b, n)]
        for j in range(i + 1, len(sil)):
            if sil[j][0] >= b:
                break
            if sil[j][2] != n and j not in used:
                grp.append(sil[j])
                used.add(j)
        if len({g[2] for g in grp}) >= 2:
            s0, s1 = max(g[0] for g in grp), min(g[1] for g in grp)
            if s1 > s0:
                pi.append((round(s0 - t0, 2), round(s1 - t0, 2)))
        else:
            wifi.setdefault(n, []).append((round(a - t0, 2), round(b - t0, 2)))
    pics = {}
    for t, d in dl:
        pics.setdefault(round(t - t0, 2), []).append(d)
    out["timeline"] = dict(span_s=round(t1 - t0, 1), pics=[(k, round(sorted(v)[len(v) // 2], 3))
                                                         for k, v in sorted(pics.items())],
                           pi_stalls=pi, wifi=wifi)
    steps = _csv(folder, "step.csv")
    wd = [r for r in steps if r["state"] == "failed" and r.get("info", "").startswith("watchdog")]
    out["watchdog_per_h"] = round(len(wd) / hours, 1)
    out["watchdog_per_robot_h"] = {n: round(sum(1 for r in wd if r["robot"] == n) / hours, 1) for n in names}
    # -- named / seen / dropouts ------------------------------------------------------------------------------------
    br = [r for r in _csv(folder, os.path.join("bridge", "track.csv"))]
    pw = {}
    for tp in {r["t"] for r in br}:
        pw[int((float(tp) - t0) // 10)] = pw.get(int((float(tp) - t0) // 10), 0) + 1
    out["pictures_s"] = quant([v / 10.0 for k, v in pw.items() if 0 <= k < int((t1 - t0) // 10)])
    seen = {}
    for r in br:
        if r.get("status") == "seen":
            seen.setdefault(r["robot"], []).append((float(r["t"]), float(r["x"]), float(r["y"]), r.get("src") or ""))
    named = {}
    for r in ct:
        named.setdefault(r["robot"], []).append(float(r["t"]))
    pics = sorted({float(r["t"]) for r in br})
    sset = {n: sorted({s[0] for s in seen.get(n, [])}) for n in names}
    ps = picture_spells(pics, sset, named, names)
    out["unnamed"] = dict(per_robot_h=round(len(ps["unnamed"]) / rh, 2), length_s=quant(ps["unnamed"]),
                          over_5s_per_robot_h=round(sum(1 for u in ps["unnamed"] if u > 5) / rh, 3),
                          longest_s=round(max(ps["unnamed"]), 1) if ps["unnamed"] else 0.0)
    out["cam_gaps"] = dict(per_robot_h=round(len(ps["gaps"]) / rh, 2), length_s=quant(ps["gaps"]),
                           lengths=sorted(round(g, 2) for g in ps["gaps"]))
    out["seen_pct"] = ps["seen_pct"]
    out["close"] = close_rates(seen, hours)
    srcs = {}
    for n in names:
        for q in seen.get(n, []):
            srcs[q[3]] = srcs.get(q[3], 0) + 1
    out["src_share"] = {k: round(v / max(1, sum(srcs.values())), 4) for k, v in srcs.items()}
    # -- own pose against the camera; standing scatter -------------------------------------------------------------
    og, sd = [], {}
    for n in names:
        S = sorted(seen.get(n, []))
        st = [s[0] for s in S]
        last = -1e9
        F = sorted((float(r["t_recv"]), _f(r["x_cm"]), _f(r["y_cm"]), _f(r.get("travel_cm"), 0.0),
                    r.get("state") == "idle", _f(r.get("th_deg"), 0.0)) for r in follow if r["robot"] == n)
        for t, x, y, *_ in F:
            if t - last < 1.0:
                continue
            i = bisect.bisect_left(st, t)
            best = None
            for k in (i - 1, i):
                if 0 <= k < len(S) and abs(S[k][0] - t) <= 0.3:
                    if best is None or abs(S[k][0] - t) < abs(best[0] - t):
                        best = S[k]
            if best is None:
                continue
            last = t
            og.append(math.hypot(best[1] - x, best[2] - y))
        # standing: 3 s windows whose own travel did not change
        k0 = 0
        for k in range(len(F)):
            while F[k][0] - F[k0][0] > 3.0:
                k0 += 1
            if F[k][0] - F[k0][0] < 2.9 or abs(F[k][3] - F[k0][3]) > 0.05 or \
                    not all(q[4] for q in F[k0:k + 1]) or \
                    max(abs((q[5] - F[k0][5] + 180.0) % 360.0 - 180.0) for q in F[k0:k + 1]) > 2.0:
                continue
            i, j = bisect.bisect_left(st, F[k0][0]), bisect.bisect_right(st, F[k][0])
            grp = {}
            for s in S[i:j]:
                grp.setdefault(s[3], []).append(s)
            for src, v in grp.items():
                if len(v) >= 5:
                    mx, my = statistics.mean(q[1] for q in v), statistics.mean(q[2] for q in v)
                    sd.setdefault(src, []).append(math.sqrt(statistics.mean((q[1] - mx) ** 2 + (q[2] - my) ** 2
                                                                            for q in v) / 2))
            k0 = k + 1
    out["own_cam_gap_cm"] = quant(og)
    out["standing_sd_cm"] = {k: quant(v) for k, v in sd.items()}
    # -- curves, headings, events -----------------------------------------------------------------------------------
    arc = _csv(folder, "arcfit.csv")
    out["curve_ratio"] = {}
    for n in names:
        v = [_f(r["k_ratio"]) for r in arc if r["robot"] == n and _f(r.get("k_ratio")) is not None and
             0.5 < _f(r["k_ratio"]) < 1.6]
        if len(v) >= 5:
            out["curve_ratio"][n] = dict(mean=round(statistics.mean(v), 4), sd=round(statistics.stdev(v), 4),
                                         n=len(v))
    ev = _csv(folder, "events.csv")
    he = []
    for e in ev:
        m = re.search(r"its own heading (-?[\d.]+) deg, the camera sees it go (-?[\d.]+)", e.get("text", ""))
        if m:
            d = (float(m.group(1)) - float(m.group(2)) + 180.0) % 360.0 - 180.0
            he.append(abs(d))
        m = re.search(r"drove (-?[\d.]+) deg off its own heading", e.get("text", ""))
        if e["kind"] == "heading re-set" and m:
            he.append(abs(float(m.group(1))))
    out["heading_err_deg"] = dict(per_robot_h=round(len(he) / rh, 2), size=quant(he),
                                  over_20_per_robot_h=round(sum(1 for h in he if h > 20) / rh, 3))

    def kind_rate(kind):
        rows = [float(e["t"]) for e in ev if e["kind"] == kind]
        if len(rows) >= 400:                      # the events.csv cap: the rate over the time the 400 took
            span = (max(rows) - t0) / 3600.0
            return dict(per_robot_h=round(len(rows) / max(1e-6, span) / len(names), 2), capped=True, n=len(rows))
        return dict(per_robot_h=round(len(rows) / rh, 2), capped=False, n=len(rows))
    out["did_not_move"] = kind_rate("did not move")
    out["needs_hand"] = kind_rate("NEEDS A HAND")
    out["rejoined"] = kind_rate("REJOINED")
    out["front_stops"] = kind_rate("front stop")
    lost = _csv(folder, "lost.csv")
    sl = [r for r in lost if _f(r.get("blind_cm"), 99) < 1.0]
    out["lost"] = dict(per_robot_h=round(len(lost) / rh, 2), standing_per_robot_h=round(len(sl) / rh, 2),
                       length_s=quant([_f(r["dur_s"]) for r in lost]),
                       standing_jump_cm=quant([_f(r.get("jump_cm")) for r in sl]))
    sm = _csv(folder, "surface_marks.csv")
    out["edge_offset_cm"] = quant([_f(r["cam_edge_cm"]) for r in sm if r.get("kind") == "edge" and
                                   _f(r.get("cam_edge_cm")) is not None])
    cpu = _csv(folder, "cpu.csv")
    if cpu:
        out["cores_busy_pct"] = {c: quant([_f(r[c]) for r in cpu if _f(r.get(c)) is not None], (0.5, 0.9))
                                 for c in ("core0", "core1", "core2", "core3")}
    if raw and os.path.exists(os.path.join(folder, "raw.csv")):
        out.update(measure_raw(folder, names, seen, hours))
    return out


def measure_raw(folder, names, seen, hours):
    """raw.csv in one streaming pass: slips (wheels turn, body still) and tilt spells."""
    W = {n: [] for n in names}                   # per robot: (t, v_cms, roll, pitch)
    with open(os.path.join(folder, "raw.csv"), newline="") as f:
        rd = csv.reader(f)
        hdr = next(rd)
        ix = {k: hdr.index(k) for k in ("robot", "t_recv", "wl_rpm", "wr_rpm", "roll_deg", "pitch_deg")}
        for row in rd:
            try:
                n = row[ix["robot"]]
                if n in W:
                    v = (float(row[ix["wl_rpm"]]) + float(row[ix["wr_rpm"]])) / 2.0 * 2 * math.pi * WHEEL_R_CM / 60.0
                    W[n].append((float(row[ix["t_recv"]]), v, float(row[ix["roll_deg"]]),
                                 float(row[ix["pitch_deg"]])))
            except (ValueError, IndexError):
                continue
    slips, slip_len, tilts, tilt_max = 0, [], 0, []
    for n, rows in W.items():
        rows.sort()
        S = sorted(seen.get(n, []))
        st = [s[0] for s in S]
        if not rows:
            continue
        lr = statistics.median(r[2] for r in rows[::50])
        lp = statistics.median(r[3] for r in rows[::50])

        def cam_at(t):
            i = bisect.bisect_left(st, t)
            best = None
            for k in (i - 1, i):
                if 0 <= k < len(S) and abs(S[k][0] - t) <= 0.6:
                    if best is None or abs(S[k][0] - t) < abs(best[0] - t):
                        best = S[k]
            return best
        k0 = 0
        tw = rows[0][0]
        run = 0
        while tw < rows[-1][0]:
            k1 = k0
            wheel = 0.0
            while k1 + 1 < len(rows) and rows[k1 + 1][0] < tw + 1.0:
                wheel += abs(rows[k1][1]) * (rows[k1 + 1][0] - rows[k1][0])
                k1 += 1
            a, b = cam_at(tw), cam_at(tw + 1.0)
            slip = False
            if wheel >= 2.0 and a is not None and b is not None:
                cam = math.hypot(b[1] - a[1], b[2] - a[2])
                slip = cam < 0.3 * wheel
            if slip:
                run += 1
            elif run:
                slips += 1
                slip_len.append(run)
                run = 0
            k0 = k1 + 1 if k1 + 1 < len(rows) else k1
            tw += 1.0
        # tilt spells over 3 deg from its level
        inn, mx = False, 0.0
        for t, v, ro, pi in rows:
            tl = max(abs(ro - lr), abs(pi - lp))
            if tl > 3.0:
                if not inn:
                    tilts += 1
                    inn, mx = True, tl
                mx = max(mx, tl)
            elif inn and tl < 2.0:
                tilt_max.append(mx)
                inn = False
    rh = hours * max(1, len(names))
    return dict(slips=dict(per_robot_h=round(slips / rh, 2), length_s=quant(slip_len)),
                tilt=dict(per_robot_h=round(tilts / rh, 2), max_deg=quant(tilt_max)))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("folders", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--no-raw", action="store_true", help="skip raw.csv (slips, tilt): the slow part")
    a = ap.parse_args(argv)
    res = []
    for f in a.folders:
        m = measure(f, raw=not a.no_raw)
        res.append(m)
        print(m["run"], m.get("hours"), "h", file=sys.stderr)
    with open(a.out, "w") as fo:
        json.dump(dict(version=VERSION, runs=res), fo, indent=1)


if __name__ == "__main__":
    main()
