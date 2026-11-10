#!/usr/bin/env python3
"""The calibration run's report (calib_run.py, 4 Nov 2026), written automatically at the run's end; also re-made from a
session folder:  python3 tools/fleet/calib_report.py logs/<date-time>-calib

premise -> data -> conclusion -> what's next; per-leg odometry error before (phase A) vs after (phase C: distance error
per metre, heading error per turn, drift), the fitted values and whether they read back, the four-point check against
Jay-Sun's numbers, battery used, and the two closing sweeps. Claude's own conclusions are marked as such."""
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

VERSION = "2026-11-10e14"   # e14: the LOW-BATTERY CALIBRATION note (started under 40 %); its own spots listed. 2026-11-10a (e12): the curve phase's section (arcs, fits, saved or not). No version before
PARAMS = ("wheel_scale", "gyro_scale", "gyro_drift_dps")


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load_legs(folder):
    rows = []
    p = os.path.join(folder, "legs.csv")
    if not os.path.exists(p):
        return rows
    with open(p) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def rows_all(folder):
    return load_legs(folder)


def groups(rows):
    """{label: rows} for phase A's tour (A1) and each phase-C tour (C1, C2, ...): the comparable legs (tour legs only,
    the camera-anchored edge finals left out: they are not odometry)."""
    out = {}
    for r in rows:
        if r["kind"] not in ("go", "turn") or r["result"] != "done" or r.get("interrupted") == "1":
            continue                   # (11-05c) a leg the camera lost the tag in: flagged, left out
        if r["phase"] == "A" and r["tour"] != "1":
            continue
        if r["phase"] not in ("A", "C") or "edge" in r["target"] or "park" in r["target"]:
            continue
        out.setdefault("%s%s" % (r["phase"], r["tour"]), []).append(r)
    return out


def metrics(rows):
    go = [r for r in rows if r["kind"] == "go" and _num(r["commanded"]) and _num(r["commanded"]) >= 10]
    tu = [r for r in rows if r["kind"] == "turn" and abs(_num(r["commanded"]) or 0) >= 10]
    per_m = [100.0 * abs(_num(r["along_err_cm"])) / _num(r["commanded"]) for r in go if _num(r["along_err_cm"]) is not None]
    cross_m = [100.0 * abs(_num(r["cross_err_cm"])) / _num(r["commanded"]) for r in go if _num(r["cross_err_cm"]) is not None]
    turn = [abs(_num(r["turn_err_deg"])) for r in tu if _num(r["turn_err_deg"]) is not None]
    turn90 = [90.0 * abs(_num(r["turn_err_deg"])) / abs(_num(r["commanded"])) for r in tu
              if _num(r["turn_err_deg"]) is not None]
    signed = [100.0 * _num(r["along_err_cm"]) / _num(r["commanded"]) for r in go if _num(r["along_err_cm"]) is not None]
    # (11-05h, school 5 Nov 19:34) the distance error split in two: a SCALE (cm per m, the slope of the along error
    # against the leg's length: what wheel_scale fixes) and a fixed OFFSET per leg (cm: the stop's overrun, what the
    # stop lead fixes). Per-m figures alone made a 0.5 cm overrun on 50 cm legs look like a +1 cm/m scale error
    sc = off = None
    pts = [(_num(r["commanded"]), _num(r["along_err_cm"])) for r in go if _num(r["along_err_cm"]) is not None]
    if len(pts) >= 4 and np.ptp([q[0] for q in pts]) > 10:
        k, c0 = np.polyfit([q[0] for q in pts], [q[1] for q in pts], 1)
        sc, off = 100.0 * float(k), float(c0)
    return {"dist_scale_cm_per_m": sc, "dist_offset_cm": off, "n_go": len(go), "n_turn": len(tu), "dist_err_cm_per_m_median": _med(per_m),
            "dist_err_cm_per_m_p95": _p95(per_m), "dist_err_signed_cm_per_m_mean": _mean(signed),
            "cross_cm_per_m_median": _med(cross_m), "turn_err_deg_median": _med(turn), "turn_err_deg_p95": _p95(turn),
            "turn_err_per_90_median": _med(turn90), "per_m": per_m, "turn": turn}


def _med(v):
    return float(np.median(v)) if v else None


def _mean(v):
    return float(np.mean(v)) if v else None


def _p95(v):
    return float(np.percentile(v, 95)) if v else None


def drift_rows(rows):
    out = []
    for r in rows:
        if r["kind"] == "still":
            dt_ = _num(r["dt_s"]) or 0
            if dt_ > 0:
                out.append(("%s%s" % (r["phase"], r["tour"]), _num(r["bel_turn_deg"]) / dt_,
                            _num(r["cam_turn_deg"]), dt_))
    return out


def charts(folder, g, fit, four):
    """PNG charts; [(file, one plain sentence)]."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out = []
    labs = [k for k in sorted(g, key=lambda s: (s[0] != "A", s))]
    if labs:
        fig, axs = plt.subplots(1, 2, figsize=(10, 4))
        for ax, key, ttl, unit in ((axs[0], "per_m", "Distance error per leg", "cm per metre driven"),
                                   (axs[1], "turn", "Heading error per turn", "degrees")):
            data = [metrics(g[k])[key] or [0] for k in labs]
            ax.boxplot(data, showfliers=True)
            ax.set_xticks(range(1, len(labs) + 1))
            ax.set_xticklabels(["before (A)" if k.startswith("A") else "after %s" % k for k in labs])
            for i, d in enumerate(data):
                ax.scatter(np.full(len(d), i + 1) + np.random.default_rng(i).uniform(-0.12, 0.12, len(d)), d, s=9,
                           alpha=0.5, color="#2952CC")
            ax.set_title(ttl)
            ax.set_ylabel(unit)
            ax.grid(alpha=0.3)
        fig.tight_layout()
        fn = "chart_leg_errors.png"
        fig.savefig(os.path.join(folder, fn), dpi=110)
        plt.close(fig)
        out.append((fn, "Each dot is one leg of a P01-P20 tour: how far the robot's own wheels and gyro were off where "
                        "the camera saw it stop, before the fit (A, the robot's old values) and after it (C tours, the "
                        "new values); measured by the one school camera, from legs.csv."))
    if four:
        fig, ax = plt.subplots(figsize=(6.5, 3.6))
        pts = list(four["jay_in"].keys())
        xs = np.arange(len(pts))
        series = [("Jay-Sun measured (no height correction)", [four["jay_in"][p] for p in pts], "#C2410C")]
        for lab, key, col in (("model, camera 145 cm above the table", "model_145_in", "#2952CC"),
                              ("model, camera 229 cm above the table", "model_229_in", "#7A8BA8"),
                              ("this run (with - without correction)", "run_in", "#1B7F3B")):
            if four.get(key) and all(four[key].get(p) is not None for p in pts):
                series.append((lab, [four[key][p] for p in pts], col))
        w = 0.8 / len(series)
        for i, (lab, vals, col) in enumerate(series):
            ax.bar(xs + (i - (len(series) - 1) / 2) * w, vals, w, label=lab, color=col)
        ax.set_xticks(xs)
        ax.set_xticklabels(pts)
        ax.set_ylabel("inches off, away from the camera")
        ax.legend(fontsize=7, loc="upper left")
        ax.grid(alpha=0.3, axis="y")
        fig.tight_layout()
        fn = "chart_four_points.png"
        fig.savefig(os.path.join(folder, fn), dpi=110)
        plt.close(fig)
        out.append((fn, "How far the robot's tag is put away from the camera when its 6.19 cm height is ignored, at "
                        "Jay-Sun's four points: his tape measurements (3 Nov) against the tag-height model for the two "
                        "readings of the camera's height%s." % (" and this run's camera" if four.get("run_in") else "")))
    tours = {}
    for r in rows_all(folder):
        if r["kind"] in ("go", "turn") and r["result"] == "done" and r["phase"] in ("A", "C") and r["tour"] != "0" \
                and r.get("interrupted") != "1":
            tours.setdefault("%s%s" % (r["phase"], r["tour"]), []).append(r)
    pick = [k for k in ("A1", "C1") if k in tours]
    if pick:
        import calib_run as CRN
        fig, axs = plt.subplots(1, len(pick), figsize=(5.2 * len(pick), 4.2), squeeze=False)
        for ax, k in zip(axs[0], pick):
            rs = tours[k]
            f = lambda r, a: (float(r[a + "_x"]), float(r[a + "_y"]), float(r[a + "_th"]))   # noqa: E731
            D = f(rs[0], "cam0")
            od, cm = [D[:2]], [D[:2]]
            for r in rs:
                D = CRN.compose(D, CRN.relative(f(r, "bel0"), f(r, "bel1")))
                od.append(D[:2])
                cm.append(f(r, "cam1")[:2])
            od, cm = np.array(od), np.array(cm)
            ax.plot(cm[:, 0], cm[:, 1], "-o", ms=2.5, lw=1.2, color="#2952CC", label="camera: where it stopped")
            ax.plot(od[:, 0], od[:, 1], "-", lw=1.2, color="#C2410C", label="its own wheels + gyro (no correction)")
            ax.add_patch(matplotlib.patches.Rectangle((0, 0), 396.2, 167.6, fill=False, lw=1))
            ax.axvline(78 * 2.54, color="grey", ls="--", lw=0.8)
            ax.set_xlim(-60, 230)
            ax.set_ylim(-60, 210)
            ax.set_aspect("equal")
            ax.set_title("%s tour %s" % ("before the fit:" if k[0] == "A" else "after the fit:", k))
            ax.set_xlabel("x cm (corner 1 = tag 20)")
            ax.set_ylabel("y cm")
            ax.legend(fontsize=7, loc="upper left")
        fig.tight_layout()
        fn = "chart_tracks.png"
        fig.savefig(os.path.join(folder, fn), dpi=110)
        plt.close(fig)
        out.append((fn, "One P01-P20 tour before and one after the fit: where the camera saw the robot stop after each "
                        "leg (blue) against where the robot's own wheels and gyro put it, added up from the tour's "
                        "start with no camera correction (orange); table outline and the camera's x = 78 in limit "
                        "(dashed); from legs.csv."))
    return out


def four_point(run, a):
    """Jay-Sun's four points: the model's offset for both camera-height readings, and (when the run reached them)
    the run's own: the camera's position with correction vs the same reading taken flat on the table."""
    import calib_run as CR
    pts = run.S.points
    if not all(p in pts for p in CR.JAY_IN):
        return None                      # (e14) its own spots (no stickers): Jay-Sun's four points are not on this table
    h = run.tag_h or 6.19
    behind = 45 * 2.54
    out = {"jay_in": dict(CR.JAY_IN), "tag_h_cm": h}
    for key, z in (("model_145_in", 145.0), ("model_229_in", 228.6)):
        r = CR.four_point([-behind, run.S.W / 2, z], h, pts)
        out[key] = {p: round(v[0], 2) for p, v in r.items()}
    cc = getattr(run, "cam_centre", None)
    if cc:
        out["camera_centre_cm"] = [round(v, 1) for v in cc]
        visits = {}
        p = os.path.join(run.folder, "events.jsonl")
        if os.path.exists(p):
            for line in open(p):
                e = json.loads(line)
                if e.get("kind") == "point" and e.get("point") in CR.JAY_IN and e.get("phase") == "C":
                    visits.setdefault(e["point"], []).append(e["cam"])
        run_in, rows = {}, {}
        C = np.asarray(cc, float)
        for q in CR.JAY_IN:
            if not visits.get(q):
                continue
            x, y = np.mean([v[0] for v in visits[q]]), np.mean([v[1] for v in visits[q]])
            k = C[2] / (C[2] - h)
            fx, fy = C[0] + (x - C[0]) * k, C[1] + (y - C[1]) * k
            d = math.hypot(fx - x, fy - y)
            run_in[q] = round(d / 2.54, 2)
            rows[q] = {"with_cm": [round(x, 1), round(y, 1)], "without_cm": [round(fx, 1), round(fy, 1)],
                       "sticker_cm": list(pts[q])}
        out["run_in"] = run_in or None
        out["run_rows"] = rows
    return out


def write(folder, run, a):
    rows = load_legs(folder)
    g = groups(rows)
    fit = getattr(run, "fit", None) or (json.load(open(os.path.join(folder, "fit.json")))
                                        if os.path.exists(os.path.join(folder, "fit.json")) else None)
    four = four_point(run, a)
    ch = charts(folder, g, fit, four)
    st = run.status
    L = []
    title = "Calibration run, zones 1-2%s" % (" (SIMULATOR)" if a.dry else "")
    L.append("# %s: %s" % (title, os.path.basename(folder)))
    L.append("")
    L.append("Written automatically by tools/fleet/calib_run.py %s at the run's end. Robot: **%s** (WiFi name %s), "
             "roof tag %s. Result: **%s**." % (st.get("version"), st.get("robot"), st.get("wifi_name"),
                                                st.get("tag"), getattr(run, "result", st.get("result"))))
    L.append("")
    if st.get("low_battery_start") is not None:
        # (e14) values measured on a weak battery
        L.append("> **LOW-BATTERY CALIBRATION**: the robot started at %s %% battery (under %d %%). Its values were "
                 "taken on a weak battery and may not hold on a full one (DOE row: calibration against battery). "
                 "Calibrate again on a charged battery before trusting them." % (
                     st.get("low_battery_start"), getattr(__import__("calib_run"), "LOW_BATTERY_NOTE_PCT", 40)))
        L.append("")
    if getattr(run, "S", None) is not None and getattr(run.S, "spots_mode", "stickers") == "own":
        L.append("Spots: **its own** (%s): %s." % (run.S.spots_why, ", ".join(
            "%s (%.0f, %.0f)" % (k, v[0], v[1]) for k, v in sorted(run.S.points.items()))))
        L.append("")
    L.append("## Premise")
    L.append("Nick (4 Nov): get enough data without calibration as a base level, use the camera only for safety (edges, "
             "keep-outs, the camera's view, parking), then fit the robot's corrections and run a series of tours "
             "with them. Every leg is driven on the robot's wheels and gyro alone; the one school camera measures "
             "where it actually stopped, and the gap is that leg's odometry error. Speed as in the experiments: "
             "10 cm/s, 60 deg/s turns, last 5 cm at 5 cm/s.")
    L.append("")
    L.append("## Data")
    b = getattr(run, "binding", None)
    if b:
        L.append("**Binding** (%s): %s." % (b["how"], ", ".join("%s (WiFi %s, %s) = tag %s%s" % (
            r["name"], r.get("wifi") or r["wifi_name"], r["ip"], r["tag_id"],
            " [%s]" % r["pin"] if r.get("pin") else "") for r in b["robots"])))
        L.append("")
        if b.get("refused"):
            L.append("**Refused for old firmware** (not moved; their roof tags kept out): %s." % "; ".join(
                b["refused"].values()))
            L.append("")
    if getattr(run, "before", None) and all(k in run.before for k in PARAMS):
        L.append("**The robot's values before** (read with `get`, saved in robot_values_before.json): %s." %
                 ", ".join("%s %.4f" % (k, run.before[k]) for k in PARAMS))
        L.append("")
    bw = getattr(run, "baseline_words", None) or (getattr(run, "status", {}) or {}).get("baseline_words")
    if bw:
        bm = getattr(run, "baseline", None) or (getattr(run, "status", {}) or {}).get("baseline")
        L.append("**Phase A started from %s**: %s." % (
            "the defaults (wheel_scale 1, gyro_scale 1, gyro_drift_dps 0)" if bm == "defaults" else
            "the pooled values (every earlier run's legs, calib_pool)" if bm == "pooled" else "the robot's own values", bw))
        L.append("")
    if fit:
        L.append("**Fit from phase A** (%d turns, %d still periods, %d straight legs; 95 %% +- = 1.96 x SE):" % (
            fit.get("n_turns", 0), fit.get("n_still", 0), fit.get("n_rolls", 0)))
        L.append("")
        tru = getattr(run, "sim_true", None)
        L.append("| value | before | fitted | +- (95 %%) | enough = +- under | verdict | sent | read back |%s" %
                 (" true (simulator) |" if tru else ""))
        L.append("|---|---|---|---|---|---|---|---|%s" % ("---|" if tru else ""))
        for k in PARAMS:
            rb = (fit.get("readback") or {}).get(k) or {}
            L.append("| %s | %.4f | %s | %s | %.3f | %s | %s | %s |%s" % (
                k, fit["old"][k], "%.4f" % fit[k] if k in fit else "-",
                "%.4f" % (1.96 * fit["se_" + k]) if "se_" + k in fit else "-", fit["enough"][k], fit["verdict"][k],
                "%.4f" % rb["sent"] if rb.get("sent") is not None else "-",
                ("%.3f %s" % (rb["read"], "OK" if rb.get("ok") else "NOT OK")) if rb.get("read") is not None else
                "no answer", (" %.3f |" % tru[k]) if tru else ""))
        L.append("")
        if fit.get("overrun_cm"):
            L.append("**Stop overrun** (how far past the asked length a leg ends with the new wheel_scale, from phase "
                     "A; the robot's usual stop shortfall %.2f cm taken off): %s. %s" % (
                         fit.get("stop_shortfall_cm", 0), ", ".join("%+.2f cm at %s cm/s (%s legs)" % (
                             v, k, (fit.get("overrun_n") or {}).get(k, "?")) for k, v in sorted(
                             fit["overrun_cm"].items(), key=lambda kv: float(kv[0]))),
                         "Phase C sends each leg short by this (the stop lead)." if fit.get("stop_lead_cm") else
                         "Not used (stop lead off, or the wheel fit was not used)."))
            L.append("")
        if getattr(run, "fit_offset", None):
            o = run.fit_offset
            L.append("**Tag offset from the turning point** (from the in-place turns): %.1f cm ahead, %.1f cm left "
                     "(%.1f cm; circle fit +- %.1f cm rms). Taken off every camera position in this run." % (
                         o["offset_cm"][0], o["offset_cm"][1], o["offset_len_cm"], o["rms_cm"]))
            L.append("")
    if g:
        L.append("**Per-leg odometry error, before (A: the old values) vs after (C: the fitted values)**, tour legs "
                 "only (the last 10 cm at an edge point is driven from the camera and left out):")
        L.append("")
        L.append("| tour | legs (go / turn) | distance error, cm per m (median / p95) | signed mean, cm per m | "
                 "scale part, cm per m | fixed part per leg, cm | "
                 "sideways, cm per m (median) | heading error per turn, deg (median / p95) |")
        L.append("|---|---|---|---|---|---|---|---|")
        for k in sorted(g, key=lambda s: (s[0] != "A", s)):
            m = metrics(g[k])
            L.append("| %s | %d / %d | %s / %s | %s | %s | %s | %s | %s / %s |" % (
                "A (before)" if k.startswith("A") else k + " (after)", m["n_go"], m["n_turn"],
                _fmt(m["dist_err_cm_per_m_median"]), _fmt(m["dist_err_cm_per_m_p95"]),
                _fmt(m["dist_err_signed_cm_per_m_mean"], "%+.2f"), _fmt(m["dist_scale_cm_per_m"], "%+.2f"),
                _fmt(m["dist_offset_cm"], "%+.2f"), _fmt(m["cross_cm_per_m_median"]),
                _fmt(m["turn_err_deg_median"]), _fmt(m["turn_err_deg_p95"])))
        L.append("")
    dr = drift_rows(rows)
    if dr:
        L.append("**Gyro drift standing still** (the robot's own heading change while the camera saw no turn): " +
                 "; ".join("%s %+.4f deg/s over %.0f s (camera %+.2f deg)" % (k, v, t, c) for k, v, c, t in dr) + ".")
        L.append("")
    cl = getattr(run, "closest", {})
    corr = 0
    ep = os.path.join(folder, "events.jsonl")
    skips, holds = [], []
    if os.path.exists(ep):
        for line in open(ep):
            e = json.loads(line)
            corr += e.get("kind") == "pose_corrected"
            if e.get("kind") == "skip_point":
                skips.append("%s (%s)" % (e["point"], e["why"]))
            if e.get("kind") == "hold":
                holds.append(e)
    L.append("**Safety**: closest the robot's middle came to an edge %.1f cm (margin %.0f cm), to a keep-out or tile "
             "%.1f cm (wanted: the body's half diagonal + 2 cm = 8.7 cm, less 0.5 cm of measuring), furthest x %.1f cm (the camera reads to "
             "%.1f cm); the camera corrected the pose before %d legs; holds for a lost tag: %d%s; points skipped: %s." % (
                 cl.get("edge_cm", float("nan")), run.S.edge, cl.get("keepout_cm", float("nan")),
                 cl.get("view_x_max", float("nan")), 78 * 2.54, corr, len(holds),
                 (" (%s s)" % ", ".join("%.1f" % h for h in run.holds)) if run.holds else "", ", ".join(skips) or "none"))
    L.append("")
    pk = getattr(run, "parked", None)
    sv = (fit or {}).get("saved") or {}
    if sv:
        L.append("**Saved on the robot** (Nick, 5 Nov: kept over power-off): %s%s." % (
            sv.get("words", ""), "" if sv.get("ok") or sv.get("how") in ("off", "nothing new") else " (NOT confirmed)"))
        L.append("")
    L.append("**End**: %s; `sleep 10` %s. Battery %s %% at the start, %s %% at the end." % (
        ("parked at %s, %.1f cm from the sticker" % (run.S.park_label, pk[1])) if pk else "NOT parked",
        "answered" if getattr(run, "slept", False) else "sent (no answer seen)", st.get("battery_start"),
        st.get("battery_end")))
    L.append("")
    if four:
        L.append("**Four-point check** (Jay-Sun re-checks these tomorrow): how far the camera puts the robot with and "
                 "without the tag-height correction (tag %.2f cm up), inches, away from the camera:" % four["tag_h_cm"])
        L.append("")
        L.append("| point | Jay-Sun (no correction) | model, camera 145 cm above the table | model, 229 cm | sticker (cm) | "
                 "this run, phase C stops: with correction (cm) | the same read without (cm) | difference (in) |")
        L.append("|---|---|---|---|---|---|---|---|")
    for p in (four or {}).get("jay_in") or {}:
        rr = (four.get("run_rows") or {}).get(p)
        L.append("| %s | %.1f | %.1f | %.1f | %.1f, %.1f | %s | %s | %s |" % (
            p, four["jay_in"][p], four["model_145_in"][p], four["model_229_in"][p], run.S.points[p][0],
            run.S.points[p][1],
            "%.1f, %.1f" % tuple(rr["with_cm"]) if rr else "not visited", "%.1f, %.1f" % tuple(rr["without_cm"])
            if rr else "-", "%.1f" % four["run_in"][p] if four.get("run_in") and p in four["run_in"] else "-"))
    L.append("")
    ar = getattr(run, "arc_result", None)
    if ar:
        # (2026-11-10a, e12) the curve phase
        L.append("### Curves (the curve phase)")
        L.append("")
        if not ar.get("on"):
            L.append("Not driven: %s." % ar.get("why"))
        else:
            L.append("Why it ran: %s. %s." % (ar.get("why"), ar.get("words") or "no decision"))
            L.append("")
            L.append("| use | radius cm | side | cm/s | result | camera reads | curvature x | length x |")
            L.append("|---|---|---|---|---|---|---|---|")
            for r_ in ar.get("rows") or getattr(run, "arc_rows", []):
                L.append("| %s | %g | %s | %.1f | %s | %s | %s | %s |" % (
                    r_.get("use"), r_.get("radius_cm"), r_.get("side"), r_.get("v_cms") or 0, r_.get("result"),
                    r_.get("n"), _fmt(r_.get("k_ratio"), "%.3f"), _fmt(r_.get("length_ratio"), "%.3f")))
            L.append("")
            L.append("Measured: each arc's circle fitted to the camera's reads (free roam's own fit); x1.000 = as asked.")
        L.append("")
    for fn, words in ch:
        L.append("![%s](%s)" % (fn, fn))
        L.append("")
        L.append(words)
        L.append("")
    L.append("## Conclusion")
    words = conclusion(g, fit, run, four)
    for w in words:
        L.append("- " + w)
    L.append("")
    L.append("## What's next")
    L.append("- Jay-Sun re-checks P01, P05, P11 and P15 with a tape (the four-point table above).")
    if sv.get("how") in ("save", "corr save") and sv.get("ok"):
        L.append("- The values are saved on the robot and come back at every power-on and wake. If the after-tours are "
                 "NOT better: `unsave` (firmware 2026-11-05a) or `corr reset` puts it back to 1 / 0 / 1.")
    else:
        L.append("- The values are NOT kept over power-off (%s): reflash the robot with firmware 2026-11-05a and run "
                 "this again, or keep them for today only." % (sv.get("words") or "no save step"))
    L.append("- Zones 3-4 need the second camera (EMEET C950 at the door end) before they can be calibrated or run.")
    L.append("")
    L.append("## Sweep 1: new tests that may now be needed")
    for t in ("A repeat of this run on another day (do the values hold across a battery and a power cycle?).",
              "The same tours at 5 and 13 cm/s (does wheel_scale depend on speed?).",
              "Turn rate: the camera's measured turn speed against the 60 deg/s asked (turn_max_rpm was derived, "
              "not measured).",
              "The four points re-measured by tape, with the height correction on (Jay-Sun, tomorrow).",
              "LOST_HOLD_S (1 s) against the camera's real dropout lengths in camera_track.csv."):
        L.append("- " + t)
    L.append("")
    L.append("## Sweep 2: possible inaccuracies in what this relied on")
    inacc = ("The camera's model comes from Measure with only tags 20, 23, 24, 25 (21 and 22 too small to read): its "
              "error grows towards zones 2's far side; every 'camera measured' number carries it.",
              "Tag height 6.19 cm is typed (Jay-Sun's tape), not measured by the camera (one camera cannot).",
              "The tag offset from the turning point is fitted from %d resting reads between the in-place turns; a "
              "tilted roof would look like an offset." % (len(__import__("calib_run").TURNS_A) + 1),
              "turn_max_rpm %.1f is worked out from the nominal wheel size for 60 deg/s, not measured." % (
                  __import__("calib_run").TURN_RPM),
              "Chosen numbers (no DOE yet): LOST_HOLD_S 1 s, MEASURE_S 1.2 s, the still periods 60 / 30 s, 12 turns, "
              "rolls 20/45/70 cm, the 'enough' +- targets, the 1 cm edge aim, the 10 cm camera-driven edge final.",
              "The fit assumes the drift is constant through the run; a gyro bias that wanders (the simulator's does) "
              "shows as a different drift in each still period.",
              "Distances per leg are from the robot's own odometry record (LEG start/done poses); a lost record "
              "means the leg is left out.") \
        + (("SIMULATOR: the camera here is a pinhole model with chosen noise (0.08 cm + 0.12 % of range), not "
                  "rendered pictures; the robot is tools/sim with its own physics.",) if a.dry else ())
    for t in inacc:
        L.append("- " + t)
    L.append("")
    with open(os.path.join(folder, "report.md"), "w") as f:
        f.write("\n".join(L))
    with open(os.path.join(folder, "four_points.json"), "w") as f:
        json.dump(four, f, indent=1)
    return " ".join(words[:2]) if words else str(getattr(run, "result", ""))


def _fmt(v, f="%.2f"):
    return "-" if v is None else f % v


def conclusion(g, fit, run, four):
    """Plain words (Claude's conclusions, marked)."""
    out = []
    res = str(getattr(run, "result", ""))
    if res != "finished":
        out.append("The run did not finish: %s." % res)
    a = next((g[k] for k in g if k.startswith("A")), None)
    cs = [g[k] for k in sorted(g) if k.startswith("C")]
    if fit:
        ok = [k for k in PARAMS if (fit.get("readback") or {}).get(k, {}).get("ok") and fit["verdict"].get(k) == "ok"]
        out.append("Fitted and read back on the robot: %s%s." % (
            ", ".join("%s %.4f" % (k, fit["new"][k]) for k in ok) or "none",
            "; kept old: %s" % ", ".join("%s (%s)" % (k, fit["verdict"][k]) for k in PARAMS if k not in ok)
            if len(ok) < 3 else ""))
    if a and cs:
        ma = metrics(a)
        mc = metrics([r for c in cs for r in c])
        if ma["dist_err_cm_per_m_median"] is not None and mc["dist_err_cm_per_m_median"] is not None:
            out.append("(Claude's conclusion) Distance error per metre went from %.2f to %.2f cm (median), heading error "
                       "per turn from %.2f to %.2f deg: %s." % (
                           ma["dist_err_cm_per_m_median"], mc["dist_err_cm_per_m_median"],
                           ma["turn_err_deg_median"] or 0, mc["turn_err_deg_median"] or 0,
                           "the new values are better" if mc["dist_err_cm_per_m_median"] < ma["dist_err_cm_per_m_median"]
                           and (mc["turn_err_deg_median"] or 0) <= (ma["turn_err_deg_median"] or 0) + 0.1 else
                           "NOT clearly better: look at the charts before using them"))
    if four:
        j = four["jay_in"]
        m = four["model_145_in"]
        worst = max(abs(j[p] - m[p]) for p in j)
        out.append("(Claude's conclusion) The tag-height model with the camera 145 cm above the table (7.5 ft above the "
                   "floor) gives %s inches at P01/P05/P11/P15, within %.1f in of Jay-Sun's %s; with 229 cm it gives %s, "
                   "so the camera's 7.5 ft is above the floor." % (
                       "/".join("%.1f" % m[p] for p in j), worst, "/".join("%.1f" % j[p] for p in j),
                       "/".join("%.1f" % four["model_229_in"][p] for p in j)))
    return out


def write_multi(folder, run, a):
    """(5 Nov) Several robots calibrated one after another: a summary table, then each robot's own report (written in
    its folder) as a section, its pictures linked in its folder."""
    subs = run.subs
    L = ["# Calibration run, zones 1-2, %d robots%s: %s" % (len(subs), " (SIMULATOR)" if a.dry else "",
                                                          os.path.basename(folder)), ""]
    L.append("Written automatically by tools/fleet/calib_run.py at the run's end. The robots were calibrated one "
             "after another, each at its own spot; while one drove, the others stood at theirs and were kept out of "
             "its way (a %.1f cm circle round where the camera saw each, plus the moving robot's clearance). "
             "Result: **%s**." % (__import__("calib_run").PARKED_R_CM, getattr(run, "result", "")))
    L.append("")
    L.append("| robot | WiFi name | tag | spot | result | wheel_scale | gyro_scale | gyro_drift_dps | saved on the robot "
             "| parked |")
    L.append("|---|---|---|---|---|---|---|---|---|---|")
    for s_ in subs:
        fit = getattr(s_, "fit", None) or {}
        new = fit.get("new") or {}
        sv = fit.get("saved") or {}
        pk = getattr(s_, "parked", None)
        L.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            s_.name, s_.rb.name, s_.tag, s_.S.park_label, getattr(s_, "result", "-"),
            *("%.4f" % new[k] if k in new else "-" for k in PARAMS),
            ("%s%s" % (sv.get("how"), "" if sv.get("ok") else " (not confirmed)")) if sv else "-",
            "%.1f cm off" % pk[1] if pk else "no"))
    L.append("")
    for s_ in subs:
        sub = os.path.basename(s_.folder)
        p = os.path.join(s_.folder, "report.md")
        if not os.path.exists(p):
            L += ["## %s" % s_.name, "", "No report (it did not start).", ""]
            continue
        body = open(p).read().splitlines()
        L.append("## %s (folder %s/)" % (s_.name, sub))
        L.append("")
        for line in body[1:]:
            if line.startswith("#"):
                line = "#" + line
            line = line.replace("](chart_", "](%s/chart_" % sub)
            L.append(line)
        L.append("")
    with open(os.path.join(folder, "report.md"), "w") as f:
        f.write("\n".join(L))
    return "%d robots: %s" % (len(subs), "; ".join("%s %s" % (s_.name, getattr(s_, "result", "-")) for s_ in subs))


def remake(folder, cam_z_cm=None, dry=None):
    """The report again from a finished run's folder (status.json, binding.json, fit.json, legs.csv, events.jsonl)."""
    import argparse
    import calib_run as CR
    import sites as SI

    def js(n):
        p = os.path.join(folder, n)
        return json.load(open(p)) if os.path.exists(p) else None
    st = js("status.json") or {}
    run = argparse.Namespace(folder=folder, status=st, S=CR.Site(SI.load("school-conference")),
                             binding=js("binding.json"), fit=js("fit.json"), result=st.get("result"),
                             before=(js("robot_values_before.json") or {}).get("values"), holds=[], rows=[],
                             tag_h=st.get("tag_h") or 6.19, cam_centre=st.get("cam_centre"), sim_true=st.get("sim_true"))
    if cam_z_cm is not None:
        run.cam_centre = CR.SchoolCamera(z_cm=cam_z_cm).C.tolist()
    ev = [json.loads(x) for x in open(os.path.join(folder, "events.jsonl"))]
    pk = [e for e in ev if e["kind"] == "parked"]
    run.parked = (pk[-1]["pose"], pk[-1]["miss_cm"]) if pk else None
    end = [e for e in ev if e["kind"] == "end"]
    run.slept = bool(end and end[-1].get("slept"))
    run.closest = st.get("closest") or {}
    run.fit_offset = next((e for e in ev if e["kind"] == "tag_offset"), None)
    a = argparse.Namespace(dry=st.get("dry") if dry is None else dry)
    return write(folder, run, a)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("folder")
    ap.add_argument("--cam-z-cm", type=float, default=None, help="simulator runs before 4 Nov 20:30: the camera height")
    a_ = ap.parse_args()
    print(remake(a_.folder, a_.cam_z_cm))
