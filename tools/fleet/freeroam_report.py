#!/usr/bin/env python3
"""The free roam's own report (2026-11-05): report.md + charts in the run folder, made at the end of every run
(freeroam.py) and again with `python3 tools/fleet/freeroam_report.py <folder>`.

Premise -> data -> conclusion -> next. Every chart says what is measured and where it comes from, with one plain
sentence under it. Reads the run folder's CSVs (written by freeroam_core.Controller through freeroam.py):
motion.csv (every 0.5 s, each robot: moving or not, by its own wheels and gyro), moves.csv, fallbacks.csv,
reflex.csv, lost.csv, battery.csv, breaks.csv, events.csv, run.json.

Changelog: 2026-11-12mc (Mission Control build) a "Pi health" section from pi_monitor.csv (tools/pi/pi_monitor.py),
with its chart. No version string before.
"""
import csv
import json
import math
import os
import sys
from collections import Counter, defaultdict

VERSION = "freeroam-report-2026-11-12mc"
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import freeroam_core as FC   # noqa: E402

COL = {"Left": "#2952CC", "Center": "#C2410C", "Right": "#7A3EC8"}


def rd(folder, name):
    p = os.path.join(folder, name + ".csv")
    if not os.path.exists(p):
        return []
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


def fnum(v, d=0.0):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


def motion_stats(mot):
    """Shares of the run's time. 'any' = wheels turning (driving or turning on the spot, from the robot's own
    odometry: 1 cm/s or 4 deg/s over 0.6 s); 'translating' = driving (1 cm/s or more)."""
    by_t = defaultdict(list)
    for r in mot:
        by_t[r["t"]].append(r)
    ts = sorted(by_t, key=float)
    n = len(ts)
    if not n:
        return None
    out = dict(samples=n, minutes=n * 0.5 / 60.0)
    for key in ("any", "translating"):
        cnt = [sum(int(fnum(q[key])) for q in by_t[t]) for t in ts]
        act = [sum(1 for q in by_t[t] if q["mode"] not in ("out", "gone")) for t in ts]
        out[key] = dict(ge1=100.0 * sum(c >= 1 for c in cnt) / n, ge2=100.0 * sum(c >= 2 for c in cnt) / n,
                        eq3=100.0 * sum(c >= 3 for c in cnt) / n,
                        min_ge2=sum(c >= 2 for c in cnt) * 0.5 / 60.0, min_eq3=sum(c >= 3 for c in cnt) * 0.5 / 60.0,
                        all_active=100.0 * sum(1 for c, a in zip(cnt, act) if a >= 2 and c >= a) /
                        max(1, sum(1 for a in act if a >= 2)))
    per = {}
    for name in sorted(set(q["robot"] for q in mot)):
        rows = [q for q in mot if q["robot"] == name and q["mode"] not in ("out", "gone", "start")]
        if rows:
            per[name] = dict(minutes=len(rows) * 0.5 / 60.0,
                             any=100.0 * sum(int(fnum(q["any"])) for q in rows) / len(rows),
                             translating=100.0 * sum(int(fnum(q["translating"])) for q in rows) / len(rows))
    # stand-stills: the longest stretch with no motion at all, per robot still in the run (from the same samples)
    for name in per:
        rows = sorted((float(q["t"]), int(fnum(q["any"])), q["mode"]) for q in mot if q["robot"] == name)
        run_s, longest, over3, start = 0.0, 0.0, 0, None
        for t, a_, mode in rows:
            if a_ or mode in ("out", "gone", "start"):
                if start is not None:
                    d = t - start
                    longest = max(longest, d)
                    over3 += int(d > 3.0)
                start = None
            elif start is None:
                start = t
        per[name]["longest_still_s"] = longest
        per[name]["still_over_3s"] = over3
    out["robot"] = per
    out["series"] = (ts, by_t)
    return out


def chart_motion(folder, ms):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ts, by_t = ms["series"]
    t0 = float(ts[0])
    mins = defaultdict(lambda: [0, 0, 0, 0])
    for t in ts:
        m = int((float(t) - t0) // 60)
        c = sum(int(fnum(q["any"])) for q in by_t[t])
        mins[m][min(c, 3)] += 1
    xs = sorted(mins)
    tot = [max(1, sum(mins[m])) for m in xs]
    ys = [[100.0 * mins[m][k] / tt for m, tt in zip(xs, tot)] for k in (3, 2, 1, 0)]
    fig, ax = plt.subplots(figsize=(9, 3.2))
    ax.stackplot(xs, ys, labels=["3 moving", "2 moving", "1 moving", "none moving"],
                 colors=["#1f6f43", "#58a776", "#c9e3cf", "#e6e6e6"])
    ax.set_xlabel("minutes since the roam started")
    ax.set_ylabel("% of the minute")
    ax.set_ylim(0, 100)
    ax.legend(loc="lower left", fontsize=8, ncol=4)
    ax.set_title("How many robots were moving at once, minute by minute")
    fig.text(0.01, 0.01, "Measured: each robot's own wheels and gyro (FOLLOW records, 10 a second), sampled every "
             "0.5 s; source: motion.csv", fontsize=7, color="#555")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    p = os.path.join(folder, "chart_motion.png")
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p


def chart_menu(folder, moves):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    grid = np.zeros((len(FC.TURN_BINS), len(FC.LEN_BINS) * len(FC.SPEED_BINS)))
    labels = {}
    for a, tb in enumerate(FC.TURN_BINS):
        for b, lb in enumerate(FC.LEN_BINS):
            for c, sb in enumerate(FC.SPEED_BINS):
                labels[FC.Menu.label((a, b, c))] = (a, b * len(FC.SPEED_BINS) + c)
    for m in moves:
        k = labels.get(m.get("cell"))
        if k and m["result"] == "done":
            grid[k] += 1
    fig, ax = plt.subplots(figsize=(9, 3.4))
    im = ax.imshow(grid, cmap="Greens", aspect="auto")
    ax.set_yticks(range(len(FC.TURN_BINS)))
    ax.set_yticklabels(["%+d deg" % t for t in FC.TURN_BINS], fontsize=7)
    ax.set_xticks(range(grid.shape[1]))
    ax.set_xticklabels(["%d-%d cm\n%g cm/s" % (lb[0], lb[1], sb) for lb in FC.LEN_BINS for sb in FC.SPEED_BINS],
                       fontsize=5)
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            ax.text(j, i, "%d" % grid[i, j], ha="center", va="center", fontsize=6,
                    color="white" if grid[i, j] > grid.max() * 0.6 else "black")
    ax.set_title("Menu coverage: finished moves per cell (turn x length x speed)")
    fig.colorbar(im, ax=ax, fraction=0.025)
    fig.text(0.01, 0.01, "Measured: moves the robot reported done (STEP done); source: moves.csv", fontsize=7,
             color="#555")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    p = os.path.join(folder, "chart_menu.png")
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p, grid


def chart_reflex(folder, refl):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6.5, 3.6))
    mk = {"speed": ("o", "#2952CC"), "steer": ("^", "#1f6f43"), "replan": ("s", "#C2410C"), "stop": ("x", "#b00")}
    for k, (m, c) in mk.items():
        pts = [(fnum(e["pred_before"]), fnum(e["actual_min_cc"])) for e in refl if e["kind"] == k
               and e["actual_min_cc"] not in ("", None)]
        if pts:
            ax.scatter(*zip(*pts), marker=m, color=c, label="%s (%d)" % (k, len(pts)), s=22)
    ax.axhline(FC.HARD_CC, color="#b00", lw=0.8, ls="--")
    ax.text(ax.get_xlim()[0], FC.HARD_CC + 0.2, " circles touch (%.1f cm)" % FC.HARD_CC, fontsize=7, color="#b00")
    ax.set_xlabel("predicted closest, middle to middle, before the change (cm)")
    ax.set_ylabel("closest the camera then saw (cm)")
    ax.legend(fontsize=7)
    ax.set_title("Reflex events: predicted vs what happened")
    fig.text(0.01, 0.01, "Measured: the reflex's prediction and the camera's positions during each event; "
             "source: reflex.csv", fontsize=7, color="#555")
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    p = os.path.join(folder, "chart_reflex.png")
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p


def chart_lost(folder, lost):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6.5, 3.6))
    xs = [fnum(e["blind_cm"]) for e in lost]
    ys = [fnum(e["jump_cm"]) for e in lost]
    ax.scatter(xs, ys, s=18, color="#7A3EC8", label="each time the camera found it again")
    xm = max(xs + [10.0])
    ax.plot([0, xm], [FC.BLIND_BASE, FC.BLIND_BASE + FC.BLIND_PER_CM * xm], color="#555", ls="--",
            label="bubble on straights: %.1f + %.2f cm per cm" % (FC.BLIND_BASE, FC.BLIND_PER_CM))
    ax.set_xlabel("cm driven while the camera could not see it")
    ax.set_ylabel("odometry vs camera when seen again (cm)")
    ax.legend(fontsize=7)
    ax.set_title("Dead reckoning: how far off the robot's own position was")
    fig.text(0.01, 0.01, "Measured: the robot's own position (wheels + gyro) against the camera's first fix after "
             "the gap; source: lost.csv", fontsize=7, color="#555")
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    p = os.path.join(folder, "chart_lost.png")
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p


def chart_battery(folder, bat, t0):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6.5, 3.2))
    for n in sorted(set(b["robot"] for b in bat)):
        pts = [((fnum(b["t"]) - t0) / 60.0, fnum(b["pct"])) for b in bat if b["robot"] == n]
        ax.plot(*zip(*pts), color=COL.get(n), label=n, lw=1.4)
    ax.axhline(FC.BATTERY_FLOOR, color="#b00", lw=0.8, ls="--")
    ax.set_xlabel("minutes since the roam started")
    ax.set_ylabel("battery %")
    ax.legend(fontsize=7)
    ax.set_title("Battery while roaming")
    fig.text(0.01, 0.01, "Measured: each robot's own charge reading (status answers, every few seconds); "
             "source: battery.csv", fontsize=7, color="#555")
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    p = os.path.join(folder, "chart_battery.png")
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p


def never_stops(folder, mot, run):
    """(11-05d) The trophy "never stops": each robot's motion style, the share of its time with speed > 0 (its own
    wheels: 1 cm/s or more over 0.6 s), 2 / 3 at once, and every speed-zero moment with why (zeros.csv)."""
    zeros = rd(folder, "zeros")
    if not mot:
        return []
    L = ["**Never stops** (motion style: %s; speed > 0 = the robot's own wheels show 1 cm/s or more over 0.6 s, "
         "every 0.5 s, motion.csv; speed-zero moments and why: zeros.csv)." % run.get("motion", "curves"), "",
         "| robot | style | speed > 0 | speed-zero moments | per hour | the main reasons |", "|---|---|---|---|---|---|"]
    by = {}
    for m in mot:
        if m.get("mode") in ("out", "gone", "start"):
            continue
        b = by.setdefault(m["robot"], [0, 0, m.get("style", "")])
        b[0] += 1
        b[1] += int(m.get("translating", "0") == "1")
    ts = sorted(set(m["t"] for m in mot))
    hours = max(1e-6, (fnum(ts[-1]) - fnum(ts[0])) / 3600.0) if ts else 1e-6
    for n, (tot, mv, st) in sorted(by.items()):
        z = [q for q in zeros if q["robot"] == n]
        why = Counter(q["why"].split(":")[0].split(" (")[0] for q in z)
        L.append("| %s | %s | %.1f %% | %d | %.0f | %s |" % (n, st or "-", 100.0 * mv / max(1, tot), len(z), len(z) / hours,
                                                         ", ".join("%s %d" % kv for kv in why.most_common(4)) or "-"))
    per_t = {}
    for m in mot:
        if m.get("mode") in ("out", "gone", "start"):
            continue
        per_t.setdefault(m["t"], []).append(m.get("translating") == "1")
    if per_t:
        n2 = sum(1 for v in per_t.values() if sum(v) >= 2)
        n3 = sum(1 for v in per_t.values() if sum(v) >= 3)
        L += ["", "Speed > 0 at the same time: at least 2 robots %.1f %%, all 3 %.1f %% of the time." % (
            100.0 * n2 / len(per_t), 100.0 * n3 / len(per_t))]
    arc = [q for q in rd(folder, "arcfit") if q.get("k") == "median"]
    if arc:
        L += ["", "Curves' calibration found in the run (arcfit.csv): " + "; ".join(
            "%s curvature x%s, length x%s of what was asked (%s pieces)" % (q["robot"], q["k_ratio"], q["length_ratio"],
                                                                          q["n"]) for q in arc) + "."]
    return L + [""]


def make(folder):
    run = {}
    if os.path.exists(os.path.join(folder, "run.json")):
        run = json.load(open(os.path.join(folder, "run.json")))
    mot, moves, fb, refl = rd(folder, "motion"), rd(folder, "moves"), rd(folder, "fallbacks"), rd(folder, "reflex")
    lost, bat, brk, ev = rd(folder, "lost"), rd(folder, "battery"), rd(folder, "breaks"), rd(folder, "events")
    ms = motion_stats(mot)
    L = ["# Home free roam: %s" % os.path.basename(folder.rstrip("/")), ""]
    L += ["## Premise", "",
          "Three robots move at the same time, all the time, on the home board until each battery reaches %d %% "
          "(or %.1f h). Moves come from a balanced menu of turn x length x speed; every move is planned as a "
          "reservation in space and time; a reflex on every camera frame changes speed or steers before anything "
          "stops.%s Seed %s." % (run.get("battery_floor", FC.BATTERY_FLOOR), run.get("hours", 4.0),
                                 " **Simulator run** (tools/sim/fleet_sim.py), not the robots." if run.get("dry")
                                 else "", run.get("seed")), "",
          "Motion style: **%s** (5 Nov 16:06: curves = robots on firmware 2026-11-05c drive continuous curving pieces "
          "from a menu of curvature x length x speed and never stop on purpose; robots on older firmware, or the "
          "stopturn choice, stop, turn on the spot and drive; the menu chart below counts the stop-turn moves only)."
          % run.get("motion", "stopturn (before 11-05d)"), ""]
    # (e11, 9 Nov school) the driving each robot actually used (its firmware decides whether curves are possible)
    used = {}
    for e_ in ev:
        if e_.get("kind") == "motion" and e_.get("robot"):
            used[e_["robot"]] = e_.get("text") or ""
    if used:
        L += ["Motion actually used: %s (source events.csv)." % "; ".join("%s %s" % kv for kv in sorted(used.items())),
              ""]
    L += ["## Data", ""]
    if ms:
        a, tr = ms["any"], ms["translating"]
        L += ["**The trophy: moving at the same time** (%.0f min of roaming; source motion.csv)." % ms["minutes"], "",
              "| | any motion (wheels turning, turns on the spot count) | translating (driving) |",
              "|---|---|---|",
              "| at least 2 robots moving | %.1f %% (%.0f min) | %.1f %% (%.0f min) |" % (
                  a["ge2"], a["min_ge2"], tr["ge2"], tr["min_ge2"]),
              "| all 3 moving | %.1f %% (%.0f min) | %.1f %% (%.0f min) |" % (a["eq3"], a["min_eq3"], tr["eq3"],
                                                                              tr["min_eq3"]),
              "| every robot still in the run moving | %.1f %% | %.1f %% |" % (a["all_active"], tr["all_active"]),
              ""]
        L += ["| robot | minutes in the run | any motion | translating | longest stand-still | stand-stills over 3 s |",
              "|---|---|---|---|---|---|"]
        for n, v in ms["robot"].items():
            L.append("| %s | %.0f | %.1f %% | %.1f %% | %.1f s | %d |" % (n, v["minutes"], v["any"], v["translating"],
                                                                   v["longest_still_s"], v["still_over_3s"]))
        L.append("")
        try:
            chart_motion(folder, ms)
            L += ["![motion](chart_motion.png)", "",
                  "Dark green is the share of each minute with all three robots moving; the paler bands are fewer.", ""]
        except Exception as e:     # noqa: BLE001
            L += ["(motion chart not made: %s)" % e, ""]
    L += never_stops(folder, mot, run)
    done = [m for m in moves if m["result"] == "done"]
    kinds = Counter(m["kind"] for m in moves)
    L += ["**Moves**: %d (%d finished). By kind: %s." % (len(moves), len(done), ", ".join(
        "%s %d" % kv for kv in kinds.most_common())), ""]
    try:
        _, grid = chart_menu(folder, moves)
        nz = grid[grid > 0]
        L += ["![menu](chart_menu.png)", "",
              "Each box counts finished moves of one turn, length and speed; even shading means an even menu "
              "(%d of %d cells used; fewest %d, most %d)." % (len(nz), grid.size, int(grid.min()), int(grid.max())),
              ""]
    except Exception as e:     # noqa: BLE001
        L += ["(menu chart not made: %s)" % e, ""]
    fbk = Counter((f["kind"]) for f in fb)
    L += ["**Fallbacks** (no menu move fitted; fallbacks.csv): %s." % (", ".join("%s %d" % kv for kv in
                                                                                fbk.most_common()) or "none"), ""]
    rk = Counter(e["kind"] for e in refl)
    ro = Counter(e["outcome"] for e in refl)
    L += ["**Reflex events** (reflex.csv): %s; outcomes: %s. Full stops (trophy breaks, breaks.csv): %d%s." % (
        ", ".join("%s %d" % kv for kv in rk.most_common()) or "none", ", ".join("%s %d" % kv for kv in
                                                                              ro.most_common()) or "-",
        len(brk), (": " + "; ".join("%s %s (%.0f s)" % (b["robot"], b["why"], fnum(b["dur_s"])) for b in brk[:8]))
        if brk else ""), ""]
    if refl:
        L += ["| id | time s | robot | other | change | predicted before | after | closest seen | method | outcome |",
              "|---|---|---|---|---|---|---|---|---|---|"]
        for e in refl[:40]:
            ch = {"speed": "speed x%s" % e["scale"], "steer": "steer to %s" % e["steer"], "replan": "new move",
                  "stop": "full stop"}.get(e["kind"], e["kind"])
            L.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
                e["id"], e["t"], e["actor"], e["other"], ch, e["pred_before"], e["pred_after"], e["actual_min_cc"],
                e["method"], e["outcome"]))
        if len(refl) > 40:
            L.append("| ... %d more in reflex.csv | | | | | | | | | |" % (len(refl) - 40))
        L.append("")
        try:
            chart_reflex(folder, refl)
            L += ["![reflex](chart_reflex.png)", "",
                  "Each mark is one time the reflex changed a robot's speed or direction; marks above the dashed "
                  "line never let the bodies' circles touch.", ""]
        except Exception as e:     # noqa: BLE001
            L += ["(reflex chart not made: %s)" % e, ""]
    # (11-05c) what the run discovered: keep-outs (felt by the robots' own front sensors / edge guards), walls, the
    # camera's measured delay and clock
    ko, cam = rd(folder, "keepouts"), rd(folder, "camera")
    if ko:
        found = [k for k in ko if k.get("what") == "found"]
        walls = [k for k in ko if str(k.get("what", "")).startswith("wall")]
        cleared = [k for k in ko if str(k.get("what", "")).startswith("cleared")]
        L += ["**Discovered** (keepouts.csv): %d keep-outs felt by the robots' own sensors (%d cleared again when a "
              "robot's camera track drove through), %d wall findings%s." % (
                  len(found), len(cleared), len(walls),
                  (": " + "; ".join("%s (%s)" % (w["what"], w["robot"]) for w in walls[:6])) if walls else ""), ""]
        L += ["| x cm | y cm | felt by | how | inside the edge cm |", "|---|---|---|---|---|"]
        L += ["| %s | %s | %s | %s | %s |" % (k["x"], k["y"], k["robot"], k["src"], k["edge_cm"]) for k in found[:20]]
        L += [""]
    if cam:
        c = cam[-1]
        L += ["**Camera timing** (camera.csv, measured during the run): fixes arrive %.2f s after the picture (median; "
              "90 %% within %.2f s); a robot counts as lost when no fix newer than %.2f s; the camera's clock %s." % (
                  fnum(c["delay_med_s"]), fnum(c["delay_p90_s"]), fnum(c["lost_after_s"]),
                  "matches this one (alignment %+.2f s, inside the deadband)" % fnum(c["clock_seen_s"])
                  if fnum(c["clock_applied_s"]) == 0 else "is %+.2f s from this one (applied)" %
                  fnum(c["clock_applied_s"])), ""]
    if lost:
        inside = sum(int(fnum(e["within"])) for e in lost)
        L += ["**Lost by the camera** (lost.csv): %d episodes, %.0f s in all; the odometry's jump when seen again: "
              "median %.1f cm, largest %.1f cm; %d of %d inside the bubble." % (
                  len(lost), sum(fnum(e["dur_s"]) for e in lost),
                  sorted(fnum(e["jump_cm"]) for e in lost)[len(lost) // 2], max(fnum(e["jump_cm"]) for e in lost),
                  inside, len(lost)), ""]
        try:
            chart_lost(folder, lost)
            L += ["![lost](chart_lost.png)", "",
                  "Each dot is one time the camera lost a robot: how far it drove unseen, and how far its own idea of "
                  "where it was turned out to be off; dots under the line were inside the safety bubble's straight-"
                  "line rule.", ""]
        except Exception as e:     # noqa: BLE001
            L += ["(lost chart not made: %s)" % e, ""]
    if bat:
        t0 = float(ms["series"][0][0]) if ms else fnum(bat[0]["t"])
        try:
            chart_battery(folder, bat, t0)
            L += ["![battery](chart_battery.png)", "",
                  "Each line is one robot's charge; it parks when its line reaches the dashed 10 % mark.", ""]
        except Exception as e:     # noqa: BLE001
            L += ["(battery chart not made: %s)" % e, ""]
    outs = [e for e in ev if e["kind"] == "out"]
    L += ["**How each robot finished**: %s. The run ended: %s." % (
        "; ".join("%s: %s" % (e["robot"], e["text"]) for e in outs) or "still roaming at the end",
        run.get("ended", "?")), ""]
    try:                                  # (12 Nov, Mission Control build) the Pi efficiency monitor's section
        sys.path.insert(0, os.path.join(os.path.dirname(HERE), "pi"))
        import pi_monitor as PM
        sec = PM.report(folder)
        if sec:
            L += ["#" + x if x.startswith("## ") else x for x in sec]
    except Exception as e:     # noqa: BLE001
        L += ["(Pi health section not made: %s)" % e, ""]
    L += ["## Conclusion", ""]
    if ms:
        L.append("- %s robots moved at the same time for %.0f minutes (%.1f %% of the run); all three for %.0f "
                 "minutes (%.1f %%), counting turns on the spot; driving alone: %.1f %% and %.1f %%." % (
                     "At least two", ms["any"]["min_ge2"], ms["any"]["ge2"], ms["any"]["min_eq3"], ms["any"]["eq3"],
                     ms["translating"]["ge2"], ms["translating"]["eq3"]))
    L.append("- Full stops (trophy breaks): %d. Reflex changes: %d (speed %d, steer %d, new move %d)." % (
        len(brk), len(refl), rk.get("speed", 0), rk.get("steer", 0), rk.get("replan", 0)))
    touched = ro.get("circles touched", 0)
    L.append("- Closest approaches: %s." % ("no reflex event let the bodies' circles touch" if not touched else
                                            "%d reflex events ended with the circles touching: look at the clips" %
                                            touched))
    L += ["", "## Next", "",
          "- Compare this run's estimated and real move times (moves.csv est_s against dur_s) and set the "
          "reservation slack from them.",
          "- Lost-tag jumps against the bubble rule: if any fell outside, widen BLIND_TURN_FRAC or BUBBLE_CAP.",
          "- The DOE rows in docs/doe-backlog.md (edge margin, gaps, free share, reflex horizon) from this run's logs.",
          ""]
    p = os.path.join(folder, "report.md")
    with open(p, "w") as f:
        f.write("\n".join(L))
    return p


if __name__ == "__main__":
    print(make(sys.argv[1]))
