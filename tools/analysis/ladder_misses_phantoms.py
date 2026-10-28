#!/usr/bin/env python3
"""Logs only (2026-10-28): two questions about the real 27 Oct ladder run (release p0-20261027-ladder-pm).

1. Line check misses (Nick, Q14 follow-up): for every planner drive, are the camera readings over 3 cm from the
   planned lines one-off pictures or runs of consecutive pictures, and what was the robot doing at the time
   (driving a line, turning on the spot, parking, standing between steps)?
2. Phantom distance-sensor stops: from the robot's own logged readings (raw.csv, the five front zones), would the new
   rule (firmware 2026-10-28a obs_n = 2: two NEW readings in a row inside the path and too close) have stopped it?

    python3 tools/analysis/ladder_misses_phantoms.py <unzipped 20261027-ladder folder> [--json out.json]

The replay of the stop rule follows firmware/home_lab/motion.h shouldAbort() + frontInPath(): a reading is new when
any zone changed; the in-path distance takes each zone at its direction (22 / 11 / 0 / -11 / -22 deg) and keeps it
when within path_half_cm (5.25) of the centre line; a follow drive's limit is frontLimit(fmin, true) = follow_look_cm
(12), lowered near a step's end to the distance left + front_end_cm (2). The distance left is not in raw.csv, so the
replay uses 12 cm throughout (the five trips read 11.0-11.9, all under 12 and over the 7 cm floor).
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "fleet"))
import line_check as LC     # noqa: E402

ZONE_DEG = (22.0, 11.0, 0.0, -11.0, -22.0)
PATH_HALF_CM = 5.25
LIMIT_CM = 12.0


def in_path(z):
    best = 0.0
    for d, a in zip(z, ZONE_DEG):
        if d <= 0:
            continue
        lat, fwd = d * math.sin(math.radians(a)), d * math.cos(math.radians(a))
        if abs(lat) <= PATH_HALF_CM and (best == 0 or fwd < best):
            best = fwd
    return best


def steps_in_force(drive):
    """[(t0, t1, kind, robot)] from the step reports: each step's started..done/failed, with its plan step's type."""
    plans = {}
    for fp in glob.glob(os.path.join(drive, "plan_*.json")):
        k = os.path.basename(fp)[5:-5]
        if k.isdigit():
            plans[int(k)] = json.load(open(fp))
    sent, rep = {}, {}
    for ln in open(os.path.join(drive, "coordinator.jsonl")):
        e = json.loads(ln)
        if e.get("event") == "step_sent":
            sent[(e["robot"], e["id"])] = (e.get("plan", 0), e["i"])
        elif e.get("event") == "step_report":
            rep.setdefault((e["robot"], e["id"]), {})[e["state"]] = e["t"]
    out = []
    for key, st in rep.items():
        if "started" not in st or key not in sent:
            continue
        pk, i = sent[key]
        plan = plans.get(pk) or json.load(open(os.path.join(drive, "plan_given.json")))
        s = plan["robots"][key[0]]["steps"][i]
        kind = "turn on the spot" if s["type"] == "turn" else ("drive" if s["type"] == "drive" else s["type"])
        end = st.get("done") or st.get("failed") or st.get("stopped") or st["started"] + 30
        out.append((st["started"], end, kind, key[0]))
    return sorted(out)


def miss_pattern(drive):
    res = LC.check(drive, os.path.join(drive, "bridge"))
    t0, t1 = LC.run_window(drive)
    lines = LC.load_lines(drive)
    track = LC.read_track(os.path.join(drive, "bridge", "track.csv"), t0, t1)
    steps = steps_in_force(drive)
    out = {}
    for n, r in res["robots"].items():
        cam = track.get(n, [])
        if not cam:
            continue
        d = [min(LC.seg_dist((x, y), a, b) for a, b in lines[n]) for _, x, y in cam]
        ts = [q[0] for q in cam]
        runs, cur = [], []
        for v, t, (_, x, y) in zip(d, ts, cam):
            if v > LC.GATE_CM:
                if cur and t - cur[-1][0] > LC.MISS_GAP_S:
                    runs.append(cur)
                    cur = []
                cur.append((t, v, x, y))
            elif cur:
                runs.append(cur)
                cur = []
        if cur:
            runs.append(cur)
        rr = []
        for run in runs:
            ta = run[0][0]
            doing = sorted({k for (a, b, k, rb) in steps if rb == n and a - 0.05 <= ta <= b + 0.05}) or ["between steps"]
            rr.append(dict(t_s=round(ta - t0, 2), pictures=len(run), lasted_s=round(run[-1][0] - ta, 2),
                           worst_cm=round(max(q[1] for q in run), 2), at=[round(run[0][2], 1), round(run[0][3], 1)],
                           doing=doing))
        def kind_at(t):
            k = [kk for (a, b, kk, rb) in steps if rb == n and a - 0.05 <= t <= b + 0.05]
            return k[0] if k else "between steps"
        out[n] = dict(readings=len(d), p95_cm=r["camera"].get("p95_cm"), misses=r["camera"].get("misses"),
                      runs=rr, series=[(round(t - t0, 3), round(v, 2), kind_at(t)) for v, t in zip(d, ts)])
        # how often the camera readings come: a run of k pictures spans this much time
        if len(ts) > 2:
            dts = sorted(b - a for a, b in zip(ts, ts[1:]))
            out[n]["picture_gap_median_s"] = round(dts[len(dts) // 2], 3)
    return out


AFTER_MS = 1000   # readings after the real stop used as "if it had gone on" (the robot stood still then: an
                  # approximation, flagged in the report). No DOE: a window


def replay(fresh, start_ms, k_trip, n_need):
    """obs_n = n_need with the count starting again at the step's start (follow.h 2026-10-28a): the index of the
    reading that would stop it, or None. fresh: [(ms, zones, in_path)], k_trip: the real trip reading's index."""
    run = 0
    for k, (m, z, d) in enumerate(fresh):
        if m < start_ms:
            continue
        if k > k_trip and m > fresh[k_trip][0] + AFTER_MS:
            break
        run = run + 1 if 0 < d < LIMIT_CM else 0
        if run >= n_need:
            return k
    return None


def phantom_stops(drive):
    """every 'obstacle ... ahead' step failure, replayed against the robot's raw readings."""
    raw = {}
    with open(os.path.join(drive, "raw.csv")) as f:
        for r in csv.DictReader(f):
            z = tuple(float(r[k]) for k in ("dL_cm", "dCL_cm", "dC_cm", "dCR_cm", "dR_cm"))
            raw.setdefault(r["robot"], []).append((int(r["ms"]), z))
    rows = list(csv.DictReader(open(os.path.join(drive, "step.csv"))))
    out = []
    for r in rows:
        if r["state"] != "failed" or "obstacle" not in r["info"]:
            continue
        rb, ms, info = r["robot"], int(r["ms"]), r["info"]
        trip = float(info.split()[1])
        sid = r["id"]
        start = max(int(q["ms"]) for q in rows if q["robot"] == rb and q["id"] == sid and q["state"] == "started"
                    and int(q["ms"]) <= ms)
        fresh, prev = [], None
        for m, z in raw.get(rb, []):
            if z != prev:
                fresh.append((m, z, in_path(z)))
                prev = z
        # the trip reading: its in-path distance is the reported one (0.1 cm), nearest the report in time
        cands = [k for k, q in enumerate(fresh) if abs(q[0] - ms) <= 300 and abs(round(q[2], 1) - trip) < 0.051]
        k_trip = min(cands, key=lambda k: abs(fresh[k][0] - ms)) if cands else None
        if k_trip is None:
            out.append(dict(robot=rb, folder=os.path.basename(drive), ms=ms, info=info, matched=False))
            continue
        in_step = [q for q in fresh if start <= q[0] <= fresh[k_trip][0]]
        lows_in_step = sum(1 for q in in_step if 0 < q[2] < LIMIT_CM)
        prev_low = k_trip > 0 and 0 < fresh[k_trip - 1][2] < LIMIT_CM
        nxt_low = k_trip + 1 < len(fresh) and 0 < fresh[k_trip + 1][2] < LIMIT_CM
        res = {}
        for n in (2, 3):
            k = replay(fresh, start, k_trip, n)
            res[n] = None if k is None else fresh[k][0] - fresh[k_trip][0]
        dts = [b[0] - a[0] for a, b in zip(fresh, fresh[1:]) if b[0] - a[0] < 500]
        out.append(dict(robot=rb, folder=os.path.basename(drive), ms=ms, info=info, matched=True, trip_cm=trip,
                        step_ran_ms=fresh[k_trip][0] - start, new_readings_in_step=len(in_step),
                        low_in_step=lows_in_step, previous_low=prev_low, next_low=nxt_low,
                        reading_gap_median_ms=sorted(dts)[len(dts) // 2] if dts else None,
                        obs2_stops_at_ms=res[2], obs3_stops_at_ms=res[3],
                        series=[(q[0] - fresh[k_trip][0], round(q[2], 2), q[0] >= start) for q in fresh
                                if -1200 <= q[0] - fresh[k_trip][0] <= AFTER_MS]))
    return out


def charts(res, outdir):
    """Two small-multiple charts for the report (docs/screenshots/<date>/)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
    BLUE, ORANGE, RED, GREY = "#2a78d6", "#eb6834", "#e34948", "#9a9893"
    os.makedirs(outdir, exist_ok=True)
    kcol = {"drive": BLUE, "turn on the spot": ORANGE, "park": "#7a3ec8", "between steps": GREY}
    drives = [(k, n, v) for k, rs in sorted(res["misses"].items()) for n, v in rs.items()
              if isinstance(v, dict) and v.get("series") and not k.startswith("152713")]
    fig, axs = plt.subplots(len(drives), 1, figsize=(8.6, 1.55 * len(drives) + 0.9), dpi=130, sharex=False)
    for ax, (k, n, v) in zip(axs, drives):
        for kind, c in kcol.items():
            pts = [(t, d) for t, d, kk in v["series"] if kk == kind]
            if pts:
                ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=7, color=c, label=kind, zorder=3,
                           linewidths=0)
        ax.axhline(3.0, color=RED, lw=1.2, ls="--", zorder=2)
        ms = v.get("misses") or {}
        ax.set_title("%s, %s: %d readings, p95 %.2f cm; over 3 cm: %d (%d single, %d in runs, longest %d pictures)"
                     % (k.split("/")[-1], n, v["readings"], v["p95_cm"] or 0, ms.get("misses", 0), ms.get("single", 0),
                        ms.get("in_runs", 0), ms.get("longest", 0)), fontsize=8, color=INK, loc="left")
        ax.set_ylim(0, 4.2)
        ax.set_ylabel("cm off line", fontsize=7, color=MUTED)
        ax.grid(axis="y", color=GRID, lw=0.7)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        ax.tick_params(colors=MUTED, labelsize=7)
    axs[-1].set_xlabel("seconds into the drive", fontsize=8, color=MUTED)
    h, l = [], []
    for ax in axs:
        for hh, ll in zip(*ax.get_legend_handles_labels()):
            if ll not in l:
                h.append(hh)
                l.append(ll)
    h.append(plt.Line2D([], [], color=RED, ls="--"))
    l.append("3 cm gate")
    fig.legend(h, l, loc="upper right", fontsize=7, frameon=False, ncol=5, markerscale=2)
    fig.text(0.01, 0.005, "Measured: each camera reading's distance from the robot's planned lines.\nSource: "
             "27 Oct ladder logs (release p0-20261027-ladder-pm), bridge/track.csv + plans + step reports.",
             fontsize=6.5, color=MUTED)
    fig.tight_layout(rect=(0, 0.035, 1, 0.97))
    p1 = os.path.join(outdir, "ladder-misses-per-drive-single-vs-runs.png")
    fig.savefig(p1)
    plt.close(fig)
    ph = [p for p in res["phantoms"] if p.get("matched")]
    fig, axs = plt.subplots(1, len(ph), figsize=(2.3 * len(ph) + 0.6, 3.4), dpi=130, sharey=True)
    for ax, p in zip(axs, ph):
        xs = [q[0] for q in p["series"]]
        ys = [min(q[1], 24.0) if q[1] > 0 else 24.0 for q in p["series"]]
        cs = [RED if 0 < q[1] < LIMIT_CM else (BLUE if q[2] else GREY) for q in p["series"]]
        ax.plot(xs, ys, color=GRID, lw=0.8, zorder=1)
        ax.scatter(xs, ys, s=12, c=cs, zorder=3, linewidths=0)
        ax.axhline(LIMIT_CM, color=RED, lw=1.0, ls="--")
        ax.axvline(0, color=INK, lw=0.8)
        verdict = ("2 in a row: still stops (%+d ms)" % p["obs2_stops_at_ms"] if p["obs2_stops_at_ms"] is not None
                   else "2 in a row: no stop")
        ax.set_title("%s %s\n%s" % (p["robot"], p["folder"].replace("drive-", ""), verdict), fontsize=7.5,
                     color=INK)
        ax.set_xlim(-1.2e3, 1.0e3)
        ax.set_ylim(8, 25)
        ax.set_yticks([8, 10, 12, 14, 16, 18, 20, 24])
        ax.set_yticklabels(["8", "10", "12", "14", "16", "18", "20", "far"])
        ax.grid(axis="y", color=GRID, lw=0.6)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        ax.tick_params(colors=MUTED, labelsize=7)
        ax.set_xlabel("ms from the stop", fontsize=7, color=MUTED)
    axs[0].set_ylabel("nearest thing in the path, cm", fontsize=7.5, color=MUTED)
    fig.text(0.01, 0.01, "Measured: each new distance-sensor reading, nearest zone inside the robot's path (red: under the "
             "12 cm limit; grey: before the stopped step began; blue: during it or after).\n. Source: 27 Oct ladder raw.csv + step.csv.",
             fontsize=6.5, color=MUTED)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    p2 = os.path.join(outdir, "ladder-phantom-stops-readings-around-each-stop.png")
    fig.savefig(p2)
    plt.close(fig)
    return p1, p2


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("folder")
    ap.add_argument("--json")
    ap.add_argument("--charts", help="folder for the two charts")
    a = ap.parse_args(argv)
    drives = sorted(set(os.path.dirname(os.path.dirname(p)) for p in
                        glob.glob(os.path.join(a.folder, "*", "*", "bridge", "track.csv"))))
    res = {"misses": {}, "phantoms": []}
    for d in drives:
        if not os.path.exists(os.path.join(d, "coordinator.jsonl")):
            continue
        key = os.path.relpath(d, a.folder)
        try:
            res["misses"][key] = miss_pattern(d)
        except Exception as e:     # noqa: BLE001
            res["misses"][key] = {"error": repr(e)}
        if os.path.exists(os.path.join(d, "raw.csv")):
            res["phantoms"] += phantom_stops(d)
    if a.charts:
        charts(res, a.charts)
    s = json.dumps(res, indent=1)
    if a.json:
        open(a.json, "w").write(s)
    print(s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
