#!/usr/bin/env python3
"""Replay recorded runs through the fusion with different sensor setups and
dropouts, and compare how far off each setup ends up.

    python tools/fusion/replay.py --session logs/20261018-101500 \\
        --sheet docs/experiments/E01-baseline-dataset/data/measurements.csv \\
        --out results/E1-fusion

    # with camera sightings (detector output or labels), lined up by the light flashes:
    python tools/fusion/replay.py --session ... --sheet ... \\
        --recording <camrec folder> --detections <detections.csv> --calib <floor_calibration.json> \\
        --out results/E1-fusion

    # simulator output also has the true path, so path errors are reported too:
    python tools/fusion/replay.py --session sim/session --sheet sim/measurements.csv \\
        --detections sim/recording/camera_detections.csv --calib sim/recording/floor_calibration.json \\
        --recording sim/recording --truth sim/truth.csv --out results/sim

    # blink lights (tools/blink/blink.py) and calib.py's headings / light offset:
    python tools/fusion/replay.py ... --blink sim/recording/blink_detections.csv \
        --calib-results sim/recording/calibration_results.csv --truth sim/truth.csv --out results/sim
    (with --truth, camera_truth.csv beside it, if there, scores which robot each fix went to)

Each setup ("configuration") is run over exactly the same recorded runs, so
setups are compared run by run (paired), which removes run-to-run luck.
Configurations come from --configs <file.json> (a list of settings, see
engine.DEFAULT) or the built-in list below.

Output in --out: results.csv (one row per run and setup), summary.csv, and
end_error.svg (every run's end error per setup); with blink or motion
identity also identity.csv (per setup and robot: how many fixes got their
identity from blink, motion, both, combined, or position alone, and, with
camera_truth.csv, how many went to the right robot).
"""
import argparse
import csv
import json
import math
import os
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
import engine as eng  # noqa: E402
import graph  # noqa: E402
import runs as rn  # noqa: E402
from ekf import D2R, wrap  # noqa: E402
from tape_results import paired, interval  # noqa: E402

BUILTIN = [
    dict(name="wheels only", sensors=["wheel_v", "wheel_w"]),
    dict(name="wheels + gyro", sensors=["wheel_v", "wheel_w", "gyro"]),
    dict(name="wheel distance + gyro heading", sensors=["wheel_v", "gyro"]),
    dict(name="+ camera (filter)", sensors=["wheel_v", "wheel_w", "gyro", "camera"]),
    dict(name="+ camera (factor graph)", sensors=["wheel_v", "wheel_w", "gyro", "camera"], method="graph"),
    dict(name="+ camera, half the frames lost", sensors=["wheel_v", "wheel_w", "gyro", "camera"], cam_drop_p=0.5),
    dict(name="+ camera, blocked 5 s mid-run", sensors=["wheel_v", "wheel_w", "gyro", "camera"], cam_off_mid_s=5.0),
    dict(name="+ camera at 1 frame/s", sensors=["wheel_v", "wheel_w", "gyro", "camera"], cam_rate_hz=1.0),
    dict(name="+ camera, 200 ms late", sensors=["wheel_v", "wheel_w", "gyro", "camera"], cam_latency_s=0.2),
    dict(name="camera only", sensors=["camera"]),
]
# need --blink (and --detections for the ones with "camera")
WWG = ["wheel_v", "wheel_w", "gyro"]
BUILTIN_BLINK = [
    dict(name="+ blink lights only", sensors=WWG + ["blink"], motion_identity=True, calib_heading=True),
    dict(name="+ blink lights, heading from motion", sensors=WWG + ["blink"], motion_identity=True,
         calib_heading=True, heading_from_motion=True),
    dict(name="+ detector + blink", sensors=WWG + ["camera", "blink"], motion_identity=True, calib_heading=True),
    dict(name="+ detector + blink, heading from motion", sensors=WWG + ["camera", "blink"], motion_identity=True,
         calib_heading=True, heading_from_motion=True),
    dict(name="+ detector + blink (factor graph)", sensors=WWG + ["camera", "blink"], motion_identity=True,
         calib_heading=True, method="graph"),
    dict(name="blink identity off (motion only)", sensors=WWG + ["camera", "blink"], motion_identity=True,
         calib_heading=True, blink_identity=False),
]
# need --detections only
BUILTIN_CAMERA_EXTRA = [
    dict(name="+ camera, identity by motion", sensors=WWG + ["camera"], motion_identity=True, calib_heading=True),
    # the case heading from motion is meant for: detector positions only, no detector heading, no blink
    dict(name="+ detector positions only (no heading)", sensors=WWG + ["camera"], cam_heading=False,
         calib_heading=True),
    dict(name="+ detector positions only, heading from motion", sensors=WWG + ["camera"], cam_heading=False,
         calib_heading=True, heading_from_motion=True),
]
BASELINE = "wheels only"


def at(traj, t):
    """(x, y, th) of a trajectory [(t, x, y, th, ...)] at time t (linear, heading wrapped)."""
    if not traj:
        return None
    lo, hi = 0, len(traj) - 1
    if t <= traj[0][0]:
        return traj[0][1:4]
    if t >= traj[-1][0]:
        return traj[-1][1:4]
    while hi - lo > 1:
        m = (lo + hi) // 2
        if traj[m][0] <= t:
            lo = m
        else:
            hi = m
    a, b = traj[lo], traj[hi]
    f = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0
    return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * wrap(b[3] - a[3]))


def load_truth(path):
    tr = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            tr.setdefault(r["run"], []).append((float(r["t_laptop"]), float(r["x_cm"]), float(r["y_cm"]),
                                                float(r["heading_deg"]) * D2R))
    for v in tr.values():
        v.sort()
    return tr


def evaluate(run, traj, truth=None):
    t_eval = run.sync["end"][0] if "end" in run.sync else run.t_end
    out = dict(end_err_cm=float("nan"), end_head_err_deg=float("nan"), path_rmse_cm=float("nan"),
               path_max_cm=float("nan"), head_rmse_deg=float("nan"))
    est = at(traj, t_eval)
    if est and run.tape_end:
        x, y, h = run.tape_end
        out["end_err_cm"] = math.hypot(est[0] - x, est[1] - y)
        out["end_head_err_deg"] = wrap(est[2] - h * D2R) / D2R
    if truth and run.run_id in truth:
        errs, herrs = [], []
        for t, x, y, h in truth[run.run_id]:
            if traj[0][0] <= t <= traj[-1][0]:
                e = at(traj, t)
                errs.append(math.hypot(e[0] - x, e[1] - y))
                herrs.append(wrap(e[2] - h) / D2R)
        if errs:
            out["path_rmse_cm"] = math.sqrt(sum(e * e for e in errs) / len(errs))
            out["path_max_cm"] = max(errs)
            out["head_rmse_deg"] = math.sqrt(sum(e * e for e in herrs) / len(herrs))
    return out


def load_camera_truth(path, H_px_to_floor, offset_s, parallax=None, dist=None):
    """Simulator camera_truth.csv -> {frame: [(robot, x, y, heading_rad)]}: each
    robot's true top centre moved down to the floor, as the fusion's sightings are."""
    out = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            x, y = float(r["x_px"]), float(r["y_px"])
            fx, fy = rn.floor_from_top(*rn.apply_h(H_px_to_floor, *rn.undistort_px(x, y, dist)), parallax)
            h = rn.heading_img_to_floor(H_px_to_floor, x, y, float(r["heading_img_deg"]), dist) * D2R
            out.setdefault(int(r["frame"]), []).append((r["robot"], fx, fy, h))
    return out


def identity_check(fix, robot, led, cam_truth, tol_cm=5.0):
    """Which robot really was at this fix? 'right', 'wrong' (another robot was
    nearest) or 'none' (no robot within tol_cm: a false alarm)."""
    best, bd = None, None
    for name, x, y, h in cam_truth.get(fix["frame"], []):
        if fix["kind"] == "blink":
            o = led.get(name, led[None])
            x, y = x + math.cos(h) * o[0] - math.sin(h) * o[1], y + math.sin(h) * o[0] + math.cos(h) * o[1]
        d = math.hypot(fix["x"] - x, fix["y"] - y)
        if bd is None or d < bd:
            best, bd = name, d
    if best is None or bd > tol_cm:
        return "none"
    return "right" if best == robot else "wrong"


ID_SOURCES = ("blink", "motion", "both", "combined", "position")


def identity_table(results, names, has_id):
    """Per setup and robot: fixes used, share by identity source, and share
    that went to the right robot. Only for setups that used identity."""
    out = []
    for n in names:
        rs = [r for r in results if r["config"] == n]
        if not any(r["id_blink"] + r["id_motion"] + r["id_both"] + r["id_combined"] for r in rs):
            continue
        for robot in sorted({r["robot"] for r in rs}):
            rr = [r for r in rs if r["robot"] == robot]
            tot = sum(r["cam_used"] for r in rr)
            row = dict(setup=n, robot=robot, fixes=tot)
            for k in ID_SOURCES:
                row[k] = f"{100 * sum(r['id_' + k] for r in rr) / tot:.1f}%" if tot else "-"
            if has_id:
                nt = sum(r["id_right"] + r["id_wrong"] + r["id_false"] for r in rr)
                row["right robot"] = f"{100 * sum(r['id_right'] for r in rr) / nt:.2f}%" if nt else "-"
                row["wrong robot"] = sum(r["id_wrong"] for r in rr)
                row["no robot there"] = sum(r["id_false"] for r in rr)
            out.append(row)
    return out


def dot_chart(results, names, path, metric="end_err_cm", label="End position error vs tape (cm)"):
    """Horizontal dot plot: one row per setup, every run a dot, mean and 95% interval."""
    rows = [(n, [r[metric] for r in results if r["config"] == n and not math.isnan(r[metric])]) for n in names]
    rows = [(n, v) for n, v in rows if v]
    if not rows:
        return
    hi = max(max(v) for _, v in rows) * 1.1 or 1
    L, R, T, rowh = 240, 30, 50, 34
    W, H = 820, T + rowh * len(rows) + 50
    X = lambda v: L + (W - L - R) * v / hi
    p = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" font-family="sans-serif" font-size="12">',
         f'<rect width="{W}" height="{H}" fill="white"/>',
         f'<text x="16" y="24" font-size="15" font-weight="600">{label}, by sensor setup</text>']
    step = 10 ** math.floor(math.log10(hi / 4)) if hi > 0 else 1
    step = next(s * step for s in (1, 2, 5, 10) if hi / (s * step) <= 6)
    v = 0.0
    while v <= hi:
        p.append(f'<line x1="{X(v):.1f}" x2="{X(v):.1f}" y1="{T - 6}" y2="{H - 40}" stroke="#e3e3e3"/>')
        p.append(f'<text x="{X(v):.1f}" y="{H - 24}" text-anchor="middle" fill="#555">{v:g}</text>')
        v += step
    for i, (n, vals) in enumerate(rows):
        y = T + rowh * i + rowh / 2
        p.append(f'<text x="{L - 10}" y="{y + 4:.1f}" text-anchor="end">{n}</text>')
        for j, val in enumerate(vals):
            jit = ((j * 37) % 9 - 4) * 1.6
            p.append(f'<circle cx="{X(val):.1f}" cy="{y + jit:.1f}" r="3.5" fill="#4a7ab8" fill-opacity="0.55"/>')
        m, sd, k, h = interval(vals)
        if k > 1:
            p.append(f'<line x1="{X(max(m - h, 0)):.1f}" x2="{X(m + h):.1f}" y1="{y + 11:.1f}" y2="{y + 11:.1f}" stroke="#222" stroke-width="1.5"/>')
        p.append(f'<line x1="{X(m):.1f}" x2="{X(m):.1f}" y1="{y + 6:.1f}" y2="{y + 16:.1f}" stroke="#c0392b" stroke-width="2.5"/>')
    p.append(f'<text x="{L}" y="{H - 6}" fill="#777" font-size="11">dots: runs · red tick: mean · black bar: 95% interval</text>')
    p.append("</svg>")
    with open(path, "w") as f:
        f.write("\n".join(p))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", required=True)
    ap.add_argument("--sheet")
    ap.add_argument("--recording")
    ap.add_argument("--detections")
    ap.add_argument("--calib")
    ap.add_argument("--video-offset", type=float)
    ap.add_argument("--blink", help="blink_detections.csv (tools/blink/blink.py): light sightings with names")
    ap.add_argument("--calib-results", help="calibration_results.csv (tools/blink/calib.py): start-nudge "
                    "headings and the light's offset on the robot")
    ap.add_argument("--truth", help="simulator truth.csv: also report errors along the whole path")
    ap.add_argument("--camera-truth", help="simulator camera_truth.csv (default: beside --truth): score identity")
    ap.add_argument("--configs", help="JSON file: list of settings (default: the built-in list)")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    runs, fits = rn.load_runs(args.session, args.sheet)
    runs = [r for r in runs if not r.excluded and r.tape_start]
    if not runs:
        sys.exit("No runs with tape start marks. Is --sheet right, and are run ids filled in?")
    dets, blink, crow, led, cam_truth = None, None, None, None, None
    if args.detections or args.blink or args.calib_results:
        if not args.calib and (args.detections or args.blink):
            sys.exit("--detections / --blink need --calib")
        off = args.video_offset
        if off is None:
            vo = rn.video_offset(args.session, args.recording, fits) if args.recording else None
            if vo is None:
                sys.exit("Couldn't line up video and robot clocks: give --recording (with flashes) or --video-offset")
            off = vo[0]
            print(f"video clock = laptop clock {off * 1000:+.0f} ms ({vo[2]} flashes)")
    if args.detections or args.blink:
        _, Hinv = rn.load_homography(args.calib)
        par = rn.load_parallax(args.calib)
        dist = rn.load_distortion(args.calib)
        if par is None:
            print("note: calibration has no camera height; robot-top positions not corrected for height")
    if args.detections:
        dets = rn.load_detections(args.detections, Hinv, off, parallax=par, dist=dist)
    if args.blink:
        blink = rn.load_blink(args.blink, Hinv, off, parallax=par, dist=dist)
        names = sorted({n for b in blink for n in b["probs"]})
        print(f"{len(blink)} blink light sightings (names: {', '.join(names) or 'none'})")
        missing = sorted({r.robot for r in runs} - set(names))
        if missing and "other" not in names:
            print(f"warning: no blink name column for {', '.join(missing)}: their lights can't be named")
    if args.calib_results:
        crow = rn.load_calib_results(args.calib_results, off)
        led, how = rn.led_offsets(crow)
        print(f"{sum(1 for c in crow if c['heading'] is not None)} calibration headings; light offset from {how}: "
              + ", ".join(f"{k or 'all'} {v[0]:.2f}/{v[1]:+.2f} cm" for k, v in sorted(led.items(), key=lambda kv: str(kv[0]))))
    elif args.blink:
        led, how = rn.led_offsets(None)
        print(f"light offset from {how} (no --calib-results)")
    ct_path = args.camera_truth or (os.path.join(os.path.dirname(os.path.abspath(args.truth)), "camera_truth.csv")
                                    if args.truth else None)
    if ct_path and os.path.exists(ct_path) and (dets or blink):
        cam_truth = load_camera_truth(ct_path, Hinv, off, parallax=par, dist=dist)
    truth = load_truth(args.truth) if args.truth else None
    configs = json.load(open(args.configs)) if args.configs else (
        BUILTIN + (BUILTIN_CAMERA_EXTRA if dets else []) + (BUILTIN_BLINK if blink else []))
    wsign = eng.wheel_turn_signs(runs)
    for robot, sgn in sorted(wsign.items()):
        if sgn < 0:
            print(f"{robot}: wheel turn rate (w_dps) is clockwise-positive; flipped to counter-clockwise")
    gyro_f = eng.gyro_factors(runs, wsign)
    for robot, (f, how) in sorted(gyro_f.items()):
        print(f"gyro units for {robot}: {how}{' (sign flipped)' if f < 0 else ''}")

    results = []
    names = []
    for c in configs:
        c = dict(c)
        method = c.pop("method", "ekf")
        cfg = eng.config(**c)
        if "camera" in cfg["sensors"] and not dets:
            continue
        if "blink" in cfg["sensors"] and not blink:
            continue
        names.append(cfg["name"])
        tracked, unknown = eng.run_engine(runs, dets, cfg, gyro_f, blink=blink, calib_rows=crow, led_offsets=led)
        led_used = led or {None: tuple(rn.LED_OFFSET_CM)}
        if cfg["led_offset_cm"] != "auto":
            led_used = {None: tuple(cfg["led_offset_cm"])}
        for run in runs:
            tr = tracked.get(run.run_id)
            if tr is None:
                continue
            if method == "graph":
                traj = graph.smooth(tr, use_gyro="gyro" in cfg["sensors"], use_wheel_w="wheel_w" in cfg["sensors"])
            elif cfg["cam_latency_s"] > 0:
                traj = tr.live           # what the robot knew at each moment, late fixes and all
            else:
                traj = tr.trajectory()
            ev = evaluate(run, traj, truth)
            src = {k: tr.id_sources.get(k, 0) for k in ID_SOURCES}
            chk = {"right": 0, "wrong": 0, "none": 0}
            if cam_truth is not None:
                for fx in tr.fixes:
                    chk[identity_check(fx, run.robot, led_used, cam_truth)] += 1
            n_chk = sum(chk.values())
            results.append(dict(config=cfg["name"], run_id=run.run_id, robot=run.robot, route=run.level,
                                cam_used=tr.cam_used, cam_blocked=tr.cam_blocked, kidnaps=len(tr.kidnaps), **ev,
                                hdg_motion=tr.hdg_used["motion"], hdg_calib=tr.hdg_used["calib"],
                                id_right=chk["right"] if cam_truth is not None else "",
                                id_wrong=chk["wrong"] if cam_truth is not None else "",
                                id_false=chk["none"] if cam_truth is not None else "",
                                **{"id_" + k: v for k, v in src.items()}))
        if unknown.confirmed_ever:
            print(f"  [{cfg['name']}] {len(unknown.confirmed_ever)} unknown object(s) seen and kept apart from the robots")

    os.makedirs(args.out, exist_ok=True)
    cols = ["config", "run_id", "robot", "route", "end_err_cm", "end_head_err_deg", "path_rmse_cm", "path_max_cm",
            "head_rmse_deg", "cam_used", "cam_blocked", "kidnaps", "hdg_motion", "hdg_calib", "id_right", "id_wrong",
            "id_false"] + ["id_" + k for k in ID_SOURCES]
    with open(os.path.join(args.out, "results.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in results:
            w.writerow({k: (f"{v:.3f}" if isinstance(v, float) else v) for k, v in r.items()})

    base = {r["run_id"]: r for r in results if r["config"] == BASELINE}
    has_truth = any(not math.isnan(r["path_rmse_cm"]) for r in results)
    has_id = cam_truth is not None
    hdr = ["setup", "runs", "end error cm", "95% interval", "vs wheels only (paired)", "p"] + (
        ["path RMSE cm", "heading RMSE deg"] if has_truth else []) + (["right robot"] if has_id else [])
    table, summ = [], []
    for n in names:
        rs = [r for r in results if r["config"] == n and not math.isnan(r["end_err_cm"])]
        if not rs:
            continue
        m, sd, k, h = interval([r["end_err_cm"] for r in rs])
        pr = [(r["end_err_cm"], base[r["run_id"]]["end_err_cm"]) for r in rs if r["run_id"] in base]
        diff = "-"
        pv = float("nan")
        if n != BASELINE and len(pr) >= 2:
            dm, dh, pv, _ = paired([a for a, _ in pr], [b for _, b in pr])
            diff = f"{dm:+.2f} ({dm - dh:+.2f} to {dm + dh:+.2f})"
        row = [n, k, f"{m:.2f}", f"{m - h:.2f} to {m + h:.2f}" if k > 1 else "-", diff,
               f"{pv:.2g}" if not math.isnan(pv) else "-"]
        if has_truth:
            row.append(f"{st.mean(r['path_rmse_cm'] for r in rs if not math.isnan(r['path_rmse_cm'])):.2f}")
            row.append(f"{st.mean(r['head_rmse_deg'] for r in rs if not math.isnan(r['head_rmse_deg'])):.2f}")
        if has_id:
            nr = sum(r["id_right"] for r in rs)
            nt = nr + sum(r["id_wrong"] + r["id_false"] for r in rs)
            row.append(f"{100 * nr / nt:.2f}%" if nt else "-")
        table.append(row)
        summ.append(dict(zip(hdr, row)))
    w = [max(len(str(x)) for x in col) for col in zip(hdr, *table)]
    print("\n" + "  ".join(h.ljust(w[i]) for i, h in enumerate(hdr)))
    for row in table:
        print("  ".join(str(c).ljust(w[i]) for i, c in enumerate(row)))
    print("\nPaired: each setup minus wheels-only on the same runs; negative = closer to the tape. p from a paired\n"
          "t-test (how likely a difference this big is by chance if the setups were really equal).")
    with open(os.path.join(args.out, "summary.csv"), "w", newline="") as f:
        dw = csv.DictWriter(f, fieldnames=hdr)
        dw.writeheader()
        dw.writerows(summ)
    if has_id:
        print("right robot: share of the camera fixes used that were really of the run's robot (camera_truth.csv:\n"
              "nearest true robot within 5 cm; a fix with no robot that close counts as wrong).")
    dot_chart(results, names, os.path.join(args.out, "end_error.svg"))
    wrote = ["results.csv", "summary.csv", "end_error.svg"]
    id_rows = identity_table(results, names, has_id)
    if id_rows:
        with open(os.path.join(args.out, "identity.csv"), "w", newline="") as f:
            dw = csv.DictWriter(f, fieldnames=list(id_rows[0].keys()))
            dw.writeheader()
            dw.writerows(id_rows)
        wrote.append("identity.csv")
        print("\nWhere each fix's identity came from (share of fixes used), per setup and robot:")
        hd = list(id_rows[0].keys())
        ww = [max(len(str(x)) for x in col) for col in zip(hd, *[[r[k] for k in hd] for r in id_rows])]
        print("  ".join(h.ljust(ww[i]) for i, h in enumerate(hd)))
        for r in id_rows:
            print("  ".join(str(r[k]).ljust(ww[i]) for i, k in enumerate(hd)))
    print(f"\nwrote {args.out}/" + ", ".join(wrote))
    return 0


if __name__ == "__main__":
    sys.exit(main())
