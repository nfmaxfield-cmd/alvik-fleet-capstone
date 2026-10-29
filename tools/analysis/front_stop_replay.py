#!/usr/bin/env python3
"""Replay the robots' distance-sensor stop rule on logged readings (Nick, 2026-10-29 16:2x: "replay all five sessions'
logged readings to confirm the false-stop count, with Left at 10 cm").

    python3 tools/analysis/front_stop_replay.py <releases folder> <out folder>

<releases folder> holds the unzipped DOE-loop releases (<session>/<session>/<loop>/drive*/raw.csv, step.csv,
coordinator.jsonl). For every forward drive step, every logged reading goes through the firmware's rule (motion.h
shouldAbort with follow.h's frontLimit and pose.h's end-of-step limit): the in-path distance (front_stop.in_path)
under min(limit, distance left + 2 cm, the plan's fmin + 0.25 - 2) for N new readings in a row, the count carried
across steps as the firmware does. Each rule variant gives the stops it would have made. In these five releases the
camera saw nothing at any of the 180 real stops (rules audit): every replayed stop is counted as a false stop.

The floor check (front_stop.floor_level / stop_limit) is replayed from the readings the robot logged while it stood
before its first move: once per boot (its first drive in the release: what run_plan does at connect, kept for the
boot) and, as a check, at every drive's start.

Writes <out>/front_stop_variants.csv (per session and robot), <out>/front_stop_floor.csv (the floor check's limit per
session, robot and boot, and per drive), <out>/front_stop_margin.csv and prints a summary table.
"""
import collections
import csv
import glob
import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "fleet"))
import front_stop as FS   # noqa: E402

WHEEL_R = 1.7
ZK = ("dL_cm", "dCL_cm", "dC_cm", "dCR_cm", "dR_cm")


def drives(root):
    for rp in sorted(glob.glob(os.path.join(root, "*", "*", "*", "drive*", "raw.csv"))):
        yield os.path.dirname(rp)


def load(d, robot):
    sent = {}
    for ln in open(os.path.join(d, "coordinator.jsonl")):
        try:
            e = json.loads(ln)
        except ValueError:
            continue
        if e.get("event") == "step_sent" and e.get("robot") == robot:
            sent[e["id"]] = e["text"]
    st = collections.defaultdict(dict)
    for r in csv.DictReader(open(os.path.join(d, "step.csv"))):
        if r.get("robot") != robot:
            continue
        try:
            st[int(r["id"])][r["state"]] = (int(r["ms"]), r["info"])
        except (ValueError, KeyError):
            pass
    rows, boot = [], None
    for r in csv.DictReader(open(os.path.join(d, "raw.csv"))):
        if r.get("robot") != robot:
            continue
        try:
            rows.append((int(r["ms"]), float(r["wl_deg"]), float(r["wr_deg"]), [float(r[k]) for k in ZK],
                         float(r["v_cms"]), int(r["step"])))
            boot = boot or (r.get("run") or "").split("-")[0]
        except (ValueError, KeyError):
            pass
    return sent, st, rows, boot


def step_kinds(sent, st):
    """{step id: (kind, length, fmin)}: D forward drives, T turns, W anything without a front check."""
    sby, dsteps = {}, []
    for sid, states in st.items():
        p = sent.get(sid, "").split()
        if len(p) < 3:
            continue
        kind = p[2]
        if kind == "D":
            if len(p) < 10 or p[9] not in ("0", "-"):
                sby[sid] = ("W", 0, None)              # backwards: no front check
                continue
            x0, y0, x1, y1 = map(float, p[3:7])
            fmin = float(p[10]) if len(p) > 10 and p[10] != "-" else None
            sby[sid] = ("D", math.hypot(x1 - x0, y1 - y0), fmin)
            if "started" in states:
                dsteps.append(sid)
        elif kind == "T":
            fmin = float(p[4]) if len(p) > 4 and p[4] != "-" else None
            sby[sid] = ("T", 0, fmin)
        else:
            sby[sid] = ("W", 0, None)
    return sby, dsteps


def replay(rows, sby, limit, n_need):
    """{step id: (ms, value)}: the first stop of each forward drive step under this rule (limit cm, n_need readings)."""
    prev, n, w0, out = None, 0, {}, {}
    for ms, wl, wr, d, vv, sid in rows:
        fresh = prev is None or d != prev
        prev = d
        stp = sby.get(sid)
        if stp is None or stp[0] == "W":
            continue
        kind, L, fmin = stp
        if kind == "T":
            lim = 7.0 if fmin is None else min(7.0, fmin + 0.25 - 2.0)
            dc = d[2]
        else:
            if sid not in w0:
                w0[sid] = (wl + wr) / 2.0
            trav = abs((wl + wr) / 2.0 - w0[sid]) * math.pi / 180.0 * WHEEL_R
            lim0 = limit if fmin is None else min(limit, fmin + 0.25 - 2.0)
            lim = min(lim0, max(L - trav, 0.0) + 2.0)
            dc = FS.in_path(d)
        if lim <= 0:
            continue
        if 0 < dc < lim:
            if fresh:
                n += 1
            if n >= n_need:
                if kind == "D" and sid not in out:
                    out[sid] = (ms, dc)
                n = 0
        else:
            n = 0
    return out


def standing_before_first_move(rows):
    pre = []
    for r in rows:
        if abs(r[4]) > 0.5:
            break
        pre.append(r[3])
    return pre


def main(argv=None):
    a = argv or sys.argv[1:]
    root, out = a[0], a[1]
    os.makedirs(out, exist_ok=True)
    data = []
    for d in drives(root):
        parts = d.split(os.sep)
        sess, drive = parts[-4], parts[-1]
        robot = drive.split("-")[1]
        sent, st, rows, boot = load(d, robot)
        if not rows:
            continue
        sby, dsteps = step_kinds(sent, st)
        # the reading that tripped a real stop reaches raw.csv only after the STEP record: added at the stop's time
        syn = []
        for sid in dsteps:
            f = st[sid].get("failed")
            if f and "zones" in f[1]:
                try:
                    z = [float(v) for v in f[1].split("zones", 1)[1].split()[:5]]
                except ValueError:
                    continue
                last = max((r for r in rows if r[0] <= f[0]), key=lambda r: r[0], default=rows[0])
                syn.append((f[0], last[1], last[2], z, last[4], sid))
        rows = sorted(rows + syn, key=lambda r: r[0])
        actual = sum(1 for sid in dsteps if "failed" in st[sid] and st[sid]["failed"][1].startswith("obstacle"))
        data.append(dict(sess=sess, drive=drive, robot=robot, boot=boot, rows=rows, sby=sby, dsteps=dsteps,
                         actual=actual, stand=standing_before_first_move(rows)))
    data.sort(key=lambda x: (x["sess"], x["drive"]))
    # the floor check at connect: the robot's first drive of each boot in each release (kept for that boot)
    first = {}
    for x in data:
        first.setdefault((x["sess"], x["robot"], x["boot"]), x)
    floor_boot = {k: FS.floor_level(v["stand"]) for k, v in first.items()}
    floor_drive = {(x["sess"], x["drive"]): FS.floor_level(x["stand"]) for x in data}

    def ceiling(robot):
        return 10.0 if robot == "Left" else FS.FLOOR_CAP_CM

    def fc_limit(x, margin, per_drive=False, with_ceiling=True):
        fl = floor_drive[(x["sess"], x["drive"])] if per_drive else floor_boot[(x["sess"], x["robot"], x["boot"])]
        return FS.stop_limit(fl.get("floor_cm"), ceiling(x["robot"]) if with_ceiling else FS.FLOOR_CAP_CM,
                             margin)[0]

    variants = collections.OrderedDict([
        ("before: 12 cm, 2 in a row", lambda x: (12.0, 2)),
        ("12 cm, 3 in a row", lambda x: (12.0, 3)),
        ("Left 10 cm, 2 in a row", lambda x: (ceiling(x["robot"]), 2)),
        ("Left 10 cm + 3 in a row", lambda x: (ceiling(x["robot"]), 3)),
        ("now: + floor check at connect", lambda x: (fc_limit(x, FS.FLOOR_MARGIN_CM), 3)),
        ("floor check at every drive", lambda x: (fc_limit(x, FS.FLOOR_MARGIN_CM, per_drive=True), 3)),
        ("floor check alone (no Left 10), 3 in a row", lambda x: (fc_limit(x, FS.FLOOR_MARGIN_CM, with_ceiling=False), 3)),
        ("floor check alone, 2 in a row", lambda x: (fc_limit(x, FS.FLOOR_MARGIN_CM, with_ceiling=False), 2)),
        ("floor check alone at every drive, 3 in a row",
         lambda x: (fc_limit(x, FS.FLOOR_MARGIN_CM, per_drive=True, with_ceiling=False), 3)),
    ])
    margins = (0.0, 0.5, 1.0, 1.5, 2.0)
    for m in margins:
        variants["margin %.1f (floor alone, every drive, 3 in a row)" % m] = (lambda mm: lambda x: (
            fc_limit(x, mm, per_drive=True, with_ceiling=False), 3))(m)
    res = collections.defaultdict(collections.Counter)
    limits = collections.defaultdict(list)
    for x in data:
        key = (x["sess"], x["robot"])
        res[key]["drives"] += 1
        res[key]["drive steps"] += len(x["dsteps"])
        res[key]["logged stops"] += x["actual"]
        for name, fn in variants.items():
            lim, n = fn(x)
            got = replay(x["rows"], x["sby"], lim, n)
            res[key][name] += len(got)
            res[key]["drives: " + name] += 1 if got else 0
            if name.startswith("floor check at every drive"):
                limits[key].append(lim)
    names = list(variants)
    with open(os.path.join(out, "front_stop_variants.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["session", "robot", "drives", "drive_steps", "logged_stops"] + ["stops: " + n for n in names] +
                   ["drives with a stop: " + n for n in names])
        for key in sorted(res):
            r = res[key]
            w.writerow([key[0], key[1], r["drives"], r["drive steps"], r["logged stops"]] + [r[n] for n in names] +
                       [r["drives: " + n] for n in names])
    with open(os.path.join(out, "front_stop_limits.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["session", "robot", "drives", "limit_min_cm", "limit_median_cm", "limit_max_cm",
                    "drives_at_ceiling_or_cap", "drives_under_10cm"])
        for key in sorted(limits):
            v = limits[key]
            top = 10.0 if key[1] == "Left" else FS.FLOOR_CAP_CM
            w.writerow([key[0], key[1], len(v), min(v), statistics.median(v), max(v), sum(1 for x in v if x >= top),
                        sum(1 for x in v if x < 10.0)])
    with open(os.path.join(out, "front_stop_floor.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["session", "robot", "boot", "new_readings", "floor_cm_3_in_a_row", "lowest_cm", "median_cm",
                    "limit_cm_now", "rule", "limit_cm_floor_alone", "zones", "drive_floor_cm_min", "drive_floor_cm_median",
                    "drive_floor_cm_max"])
        for (sess, robot, boot), fl in sorted(floor_boot.items()):
            lim, rule = FS.stop_limit(fl.get("floor_cm"), ceiling(robot))
            lim2, _ = FS.stop_limit(fl.get("floor_cm"))
            per = [floor_drive[(x["sess"], x["drive"])].get("floor_cm") for x in data
                   if x["sess"] == sess and x["robot"] == robot]
            per = [v for v in per if v is not None]
            w.writerow([sess, robot, boot, fl["n"], fl.get("floor_cm"), fl.get("lowest_cm"), fl.get("median_cm"), lim,
                        rule, lim2, json.dumps(fl.get("zones")), min(per) if per else "",
                        round(statistics.median(per), 2) if per else "", max(per) if per else ""])
    print("%-22s %-7s %6s %7s  %s" % ("session", "robot", "drives", "logged", "  ".join("%-10s" % n[:10] for n in names)))
    tot = collections.Counter()
    for key in sorted(res):
        r = res[key]
        tot["drives"] += r["drives"]
        tot["logged stops"] += r["logged stops"]
        for n in names:
            tot[n] += r[n]
            tot["drives: " + n] += r["drives: " + n]
        print("%-22s %-7s %6d %7d  %s" % (key[0], key[1], r["drives"], r["logged stops"],
                                          "  ".join("%3d/%-6s" % (r[n], "%d%%" % round(100 * r["drives: " + n] /
                                                                                        r["drives"])) for n in names)))
    print("%-22s %-7s %6d %7d  %s" % ("all", "", tot["drives"], tot["logged stops"],
                                      "  ".join("%3d/%-6s" % (tot[n], "%d%%" % round(100 * tot["drives: " + n] /
                                                                                    tot["drives"])) for n in names)))
    with open(os.path.join(out, "front_stop_summary.json"), "w") as f:
        json.dump({"variants": names, "per": {"%s|%s" % k: dict(v) for k, v in res.items()}, "total": dict(tot),
                   "floor_boot": {"%s|%s|%s" % k: v for k, v in floor_boot.items()},
                   "limits_now": {"%s|%s" % k: v for k, v in limits.items()}}, f, indent=1)


if __name__ == "__main__":
    main()
