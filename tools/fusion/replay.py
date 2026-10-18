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

    # simulator output also has the true path, so path errors are reported too
    # (the simulator writes camera_detections.csv and floor_calibration.json beside recording/, not in it):
    python tools/fusion/replay.py --session sim/session --sheet sim/measurements.csv \\
        --detections sim/camera_detections.csv --calib sim/floor_calibration.json \\
        --recording sim/recording --truth sim/truth.csv --out results/sim

    # calibration made from the runs (calibrate_floor.py --from-runs / --survey): score each run with the
    # calibration made WITHOUT it (floor_calibration_loro.csv is written beside floor_calibration.json):
    python tools/fusion/replay.py ... --calib <rec>/floor_calibration.json \\
        --calib-per-run <rec>/floor_calibration_loro.csv --out results/E1-fusion

    # blink lights (tools/blink/blink.py) and calib.py's headings / light offset:
    python tools/fusion/replay.py ... --blink sim/recording/blink_detections.csv \
        --calib-results sim/recording/calibration_results.csv --truth sim/truth.csv --out results/sim
    (with --truth, camera_truth.csv beside it, if there, scores which robot each fix went to)

    # landmark ranging: the front distance sensor against boards / blocks at tape-measured spots
    # (tools/fleet map.json, or landmarks.csv: name,x1_cm,y1_cm,x2_cm,y2_cm, one face per row):
    python tools/fusion/replay.py ... --map map.json --truth sim/truth.csv --out results/sim

Each setup ("configuration") is run over exactly the same recorded runs, so
setups are compared run by run (paired), which removes run-to-run luck.
Configurations come from --configs <file.json> (a list of settings, see
engine.DEFAULT) or the built-in list below.

Circularity: a run is never scored with a calibration that its own tape marks or
odometry helped make (the file's fit.used_runs): replay refuses (unless
--allow-in-sample) and results.csv records, per run and camera setup, the
calibration used (calib_file) and whether it included that run
(calib_includes_run). A survey calibration made from the same robot's other
runs is allowed but noted: its wheel-size error is in both the camera fit and
the wheels. Headline T2/T3 numbers: click / tape-point calibration, or a
leave-one-robot-out survey (calibrate_floor.py --survey --holdout robot).

Statistics: each setup is compared with wheels only by a paired t-test; with
many setups some small p-values come by chance, so p_holm (Holm's adjustment
for that many comparisons) is printed next to p. summary.csv also breaks every
setup down by route (rows with route != "all": runs, mean end error, interval).

Output in --out: results.csv (one row per run and setup), summary.csv, and
end_error.svg (every run's end error per setup); with blink or motion
identity also identity.csv (per setup and robot: how many fixes got their
identity from blink, motion, both, combined, or position alone, and, with
camera_truth.csv, how many went to the right robot); with --map also
landmark_ranging.csv (per setup: distance-sensor zone readings used, rejected
by the gate, and why the others could not be tied to a mapped face) and tof_*
columns in results.csv. Without --map nothing changes.
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
import landmarks as lm  # noqa: E402
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
# need --map (landmark ranging: the front distance sensor against mapped boards / blocks)
BUILTIN_TOF = [
    dict(name="+ landmark ranging", sensors=WWG + ["tof"]),
    dict(name="+ landmark ranging (factor graph)", sensors=WWG + ["tof"], method="graph"),
    dict(name="wheel distance + gyro heading + landmark ranging", sensors=["wheel_v", "gyro", "tof"]),
]
# need --map and --detections ("camera blocked the whole run + landmark ranging" is "+ landmark ranging")
BUILTIN_TOF_CAMERA = [
    dict(name="+ camera + landmark ranging", sensors=WWG + ["camera", "tof"]),
    dict(name="+ camera + landmark ranging (factor graph)", sensors=WWG + ["camera", "tof"], method="graph"),
    dict(name="+ camera, blocked 5 s mid-run + landmark ranging", sensors=WWG + ["camera", "tof"], cam_off_mid_s=5.0),
    dict(name="+ camera, blocked 20 s mid-run", sensors=WWG + ["camera"], cam_off_mid_s=20.0),
    dict(name="+ camera, blocked 20 s mid-run + landmark ranging", sensors=WWG + ["camera", "tof"],
         cam_off_mid_s=20.0),
]
TOF_COLS = ["tof_" + o for o in lm.OUTCOMES] + ["tof_resid_mean_cm", "tof_resid_sd_cm"]
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


def landmark_table(results, names):
    """Per landmark setup: zone readings by outcome (share of all readings), and
    the used readings' residual (reading minus map prediction)."""
    out = []
    for n in names:
        rs = [r for r in results if r["config"] == n and "tof_used" in r]
        tot = sum(r["tof_" + o] for r in rs for o in lm.OUTCOMES)
        if not tot or not any(r["tof_used"] + r["tof_gated"] for r in rs):
            continue
        row = dict(setup=n, readings=tot)
        for o in lm.OUTCOMES:
            row[o] = f"{100 * sum(r['tof_' + o] for r in rs) / tot:.1f}%"
        tried = sum(r["tof_used"] + r["tof_gated"] for r in rs)
        row["gated of tied to a face"] = f"{100 * sum(r['tof_gated'] for r in rs) / tried:.1f}%" if tried else "-"
        used = [(r["tof_used"], r["tof_resid_mean_cm"]) for r in rs if r["tof_used"] and not math.isnan(r["tof_resid_mean_cm"])]
        nu = sum(k for k, _ in used)
        row["mean residual cm"] = f"{sum(k * m for k, m in used) / nu:+.2f}" if nu else "-"
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


def _relpath(p):
    """Path relative to the current folder if it is inside it, else absolute."""
    try:
        r = os.path.relpath(p)
    except ValueError:          # Windows: another drive
        return p
    return p if r.startswith("..") else r


def holm(pvals):
    """Holm-adjusted p-values (same order; NaN stays NaN): sort the m p-values,
    multiply the k-th smallest by (m - k + 1), keep them non-decreasing, cap at 1."""
    idx = sorted((i for i, p in enumerate(pvals) if not math.isnan(p)), key=lambda i: pvals[i])
    out = [float("nan")] * len(pvals)
    m, run = len(idx), 0.0
    for k, i in enumerate(idx):
        run = max(run, min(1.0, (m - k) * pvals[i]))
        out[i] = run
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", required=True)
    ap.add_argument("--sheet")
    ap.add_argument("--recording")
    ap.add_argument("--detections")
    ap.add_argument("--calib", help="floor_calibration.json (runs not covered by --calib-per-run use it)")
    ap.add_argument("--calib-per-run", help="floor_calibration_loro.csv from tools/calibrate_floor.py: each run's camera "
                    "sightings converted with the calibration made WITHOUT that run")
    ap.add_argument("--allow-in-sample", action="store_true",
                    help="score runs with a calibration made from them anyway (circular; marked in results.csv)")
    ap.add_argument("--video-offset", type=float)
    ap.add_argument("--blink", help="blink_detections.csv (tools/blink/blink.py): light sightings with names")
    ap.add_argument("--calib-results", help="calibration_results.csv (tools/blink/calib.py): start-nudge "
                    "headings and the light's offset on the robot")
    ap.add_argument("--truth", help="simulator truth.csv: also report errors along the whole path")
    ap.add_argument("--camera-truth", help="simulator camera_truth.csv (default: beside --truth): score identity")
    ap.add_argument("--configs", help="JSON file: list of settings (default: the built-in list)")
    ap.add_argument("--map", help="landmarks for landmark ranging: tools/fleet map.json (boards, user-measured "
                    "polygon obstacles) or landmarks.csv (name,x1_cm,y1_cm,x2_cm,y2_cm); adds the landmark setups")
    ap.add_argument("--tof-cal", help="tof_calibration.json (tools/fusion/tof_calib.py): the distance sensors' zone "
                    "directions, range bias, zone width, frame delay, noise and refined landmarks, for the landmark setups")
    ap.add_argument("--tof-cal-per-run", help="tof_calibration_loro.csv (tof_calib.py): each run scored with the "
                    "distance-sensor calibration made WITHOUT it; runs not listed use --tof-cal")
    ap.add_argument("--noise-file", help="measured sensor noise (JSON, docs/data-formats.md \"Measured noise\"): "
                    "camera, camera heading and blink light noise for every setup (a setup's own noise wins)")
    ap.add_argument("--heading-check", help="heading_check.csv (tools/heading_check.py): sets how much the "
                    "start-nudge headings are trusted from the spread measured on this setup")
    ap.add_argument("--wheel-radius", help="measured wheel radius per robot, e.g. Left=1.69,Center=1.71 (cm); "
                    "robots not listed keep 1.7")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    extra_cfg = {}          # settings from measured files, put in every setup that doesn't set them itself
    if args.noise_file:
        from ekf import load_noise_file
        extra_cfg["noise"] = load_noise_file(args.noise_file)
        print(f"measured noise from {args.noise_file}: " + ", ".join(f"{k} {v:.4g}" for k, v in extra_cfg["noise"].items()))
    if args.heading_check:
        px = rn.heading_px_sd(args.heading_check)
        if px is None:
            print(f"note: {args.heading_check} has too few runs; start-nudge heading noise stays at the setting")
        else:
            extra_cfg["calib_heading_px_sd"] = px
            print(f"start-nudge heading precision from {args.heading_check}: {px:.3f} px")
    if args.wheel_radius:
        wr = {"default": eng.DEFAULT["lib_wheel_radius_cm"]}
        for part in args.wheel_radius.split(","):
            name, _, v = part.partition("=")
            wr[name.strip()] = float(v)
        extra_cfg["wheel_radius_cm"] = wr

    all_runs, fits = rn.load_runs(args.session, args.sheet)
    runs = [r for r in all_runs if not r.excluded and r.tape_start]
    if not runs:
        sys.exit("No runs with tape start marks. Is --sheet right, and are run ids filled in?")
    blink_names, crow, led = None, None, None
    use_cam = bool(args.detections or args.blink)
    if not args.calib and use_cam:
        sys.exit("--detections / --blink need --calib")
    # ---- which calibration converts each run's camera sightings (circularity check)
    calib_of, infos, groups = {}, {}, {}
    if use_cam:
        calib_of, fell_back = rn.assign_calibrations([r.run_id for r in runs], args.calib, args.calib_per_run)
        for rid, why in sorted(fell_back.items()):
            print(f"note: {rid} {why} in {args.calib_per_run}: scored with {args.calib}")
        if not args.calib_per_run:
            over = sorted({r.run_id for r in runs} & rn.calib_info(args.calib)["used_runs"])
            if over:
                print(f"WARNING: {args.calib} was made from {len(over)} of the scored runs' tape marks / odometry "
                      f"({', '.join(over[:6])}{' ...' if len(over) > 6 else ''}). Score them with "
                      "--calib-per-run floor_calibration_loro.csv (written beside it by tools/calibrate_floor.py).")
        bad = rn.in_sample(calib_of, infos=infos)
        if bad and not args.allow_in_sample:
            sys.exit(f"\nRefusing to score {len(bad)} run(s) with a calibration made from them: "
                     + ", ".join(f"{rid} ({os.path.basename(p)})" for rid, p in bad[:6])
                     + f"{' ...' if len(bad) > 6 else ''}.\n{rn.IN_SAMPLE_HELP}\n"
                     "Use --calib-per-run floor_calibration_loro.csv (from tools/calibrate_floor.py), or "
                     "--allow-in-sample to score them anyway (results.csv marks them calib_includes_run = 1).")
        if bad:
            print(f"WARNING (--allow-in-sample): {len(bad)} run(s) scored with a calibration made from them; "
                  "their camera numbers are optimistic (calib_includes_run = 1 in results.csv)")
        robot_of = {r.run_id: r.robot for r in all_runs}
        same = rn.same_robot_survey(calib_of, robot_of, infos)
        if same:
            robots = sorted({rb for _, rb, _ in same})
            print(f"note: {len(same)} run(s) ({', '.join(robots)}) are scored with a SURVEY calibration made from "
                  "other runs of the same robot: its wheel-size error went into the camera fit too, so camera and "
                  "wheels share that error (holding out one run doesn't remove it). For headline T2/T3 numbers use a "
                  "click / tape-point calibration or a leave-one-robot-out survey (calibrate_floor.py --survey "
                  "--holdout robot).")
        for rid, p in calib_of.items():
            groups.setdefault(p, []).append(rid)
        if len(groups) > 1:
            print(f"{len(groups)} calibrations: each run's camera sightings converted with its own "
                  f"({args.calib_per_run})")
    if args.detections or args.blink or args.calib_results:
        off = args.video_offset
        if off is None:
            vo = rn.video_offset(args.session, args.recording, fits) if args.recording else None
            if vo is None:
                sys.exit("Couldn't line up video and robot clocks: give --recording (with flashes) or --video-offset")
            off = vo[0]
            print(f"video clock = laptop clock {off * 1000:+.0f} ms ({vo[2]} flashes)")
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
    ct_ok = bool(ct_path and os.path.exists(ct_path))

    # ---- camera sightings per calibration; each group of runs is replayed with the runs going at the same time
    by_t = sorted(runs, key=lambda r: r.t_start)
    units = []     # dict(runs scored, runs processed, dets, blink, cam_truth, calib)
    no_par = False
    for p, rids in groups.items():
        _, Hinv = rn.load_homography(p)
        par, dist = rn.load_parallax(p), rn.load_distortion(p)
        no_par |= par is None
        if len(groups) == 1:
            sub, lo, hi = runs, -math.inf, math.inf
        else:
            mine = [r for r in runs if r.run_id in set(rids)]
            t0, t1 = min(r.t_start for r in mine) - 5.0, max(r.t_end for r in mine) + 5.0
            sub = [r for r in by_t if r.t_end >= t0 and r.t_start <= t1]
            lo, hi = min(r.t_start for r in sub) - 1.0, max(r.t_end for r in sub) + 1.0
        u = dict(score=set(rids), runs=sub, calib=p, dets=None, blink=None, cam_truth=None)
        if args.detections:
            u["dets"] = [d for d in rn.load_detections(args.detections, Hinv, off, parallax=par, dist=dist)
                         if lo <= d["t"] <= hi]
        if args.blink:
            u["blink"] = [b for b in rn.load_blink(args.blink, Hinv, off, parallax=par, dist=dist) if lo <= b["t"] <= hi]
            blink_names = sorted({n for b in u["blink"] for n in b["probs"]}) if blink_names is None else blink_names
        if ct_ok and (u["dets"] or u["blink"]):
            frames = [d["frame"] for d in (u["dets"] or []) + (u["blink"] or [])]
            f0, f1 = min(frames), max(frames)
            u["cam_truth"] = {k: v for k, v in load_camera_truth(ct_path, Hinv, off, parallax=par, dist=dist).items()
                              if f0 <= k <= f1}
        units.append(u)
    if no_par:
        print("note: calibration has no camera height; robot-top positions not corrected for height")
    dets = any(u["dets"] for u in units)
    blink = any(u["blink"] for u in units)
    if args.detections:
        print(f"{sum(len(u['dets']) for u in units)} camera detections")
    if args.blink:
        print(f"{sum(len(u['blink']) for u in units)} blink light sightings (names: {', '.join(blink_names or []) or 'none'})")
        missing = sorted({r.robot for r in runs} - set(blink_names or []))
        if missing and "other" not in (blink_names or []):
            print(f"warning: no blink name column for {', '.join(missing)}: their lights can't be named")
    has_ct = any(u["cam_truth"] is not None for u in units)
    truth = load_truth(args.truth) if args.truth else None
    faces = lm.load(args.map) if args.map else None
    tof_cal = None
    if args.tof_cal or args.tof_cal_per_run:
        default = lm.load_cal(args.tof_cal) if args.tof_cal else None
        per = {}
        if args.tof_cal_per_run:
            cache = {}
            for rid, pth in lm.read_cal_per_run(args.tof_cal_per_run).items():
                per[rid] = cache.setdefault(pth, lm.load_cal(pth))
        bad = sorted(r.run_id for r in runs if r.run_id in (per.get(r.run_id) or default or {}).get("used_runs", []))
        if bad and not args.allow_in_sample:
            sys.exit(f"Refusing to score {len(bad)} run(s) with a distance-sensor calibration made from them "
                     f"({', '.join(bad[:6])}{' ...' if len(bad) > 6 else ''}). Give --tof-cal-per-run "
                     "tof_calibration_loro.csv (from tof_calib.py), or --allow-in-sample.")
        if bad:
            print(f"WARNING (--allow-in-sample): {len(bad)} run(s) scored with a distance-sensor calibration made "
                  "from them")
        tof_cal = {"per_run": per, "default": default}
        src = default or next(iter(per.values()), None)
        if src and src.get("faces") and not faces:
            faces = src["faces"]
        print(f"distance-sensor calibration: {len(per)} runs with their own (leave-one-run-out)"
              + (f", the rest {os.path.basename(args.tof_cal)}" if default else ""))
    if args.map:
        print(f"landmark ranging: {len(faces)} mapped faces from {args.map}")
        if not faces:
            print("warning: the map has no boards (or user-measured polygon obstacles): landmark setups skipped")
    configs = json.load(open(args.configs)) if args.configs else (
        BUILTIN + (BUILTIN_CAMERA_EXTRA if dets else []) + (BUILTIN_BLINK if blink else [])
        + (BUILTIN_TOF + (BUILTIN_TOF_CAMERA if dets else []) if faces else []))
    view_area = rn.view_area_cm2(args.calib) if (args.calib and use_cam) else None
    if view_area:
        print(f"camera view: {view_area:.0f} cm^2 of floor (from {os.path.basename(args.calib)})")
    wsign = eng.wheel_turn_signs(runs)
    signs = (wsign, eng.wheel_fwd_signs(runs))
    vmax = eng.fastest_speed(runs)
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
        for k, v in extra_cfg.items():
            if k == "noise":
                c["noise"] = dict(v, **c.get("noise", {}))
            else:
                c.setdefault(k, v)
        cfg = eng.config(**c)
        if cfg["clutter_cm2"] == "auto" and view_area:
            cfg["clutter_cm2"] = view_area
        if cfg["vmax"] == "auto":
            cfg["vmax"] = vmax           # from all runs, also when a group of them is replayed alone
        cam_cfg = "camera" in cfg["sensors"] or "blink" in cfg["sensors"]
        if "camera" in cfg["sensors"] and not dets:
            continue
        if "blink" in cfg["sensors"] and not blink:
            continue
        if "tof" in cfg["sensors"] and not faces:
            continue
        names.append(cfg["name"])
        led_used = led or {None: tuple(rn.LED_OFFSET_CM)}
        if cfg["led_offset_cm"] != "auto":
            led_used = {None: tuple(cfg["led_offset_cm"])}
        todo = units if cam_cfg else [dict(score={r.run_id for r in runs}, runs=runs, calib=None, dets=None,
                                           blink=None, cam_truth=None)]
        n_unknown = 0
        for u in todo:
            tracked, unknown = eng.run_engine(u["runs"], u["dets"], cfg, gyro_f, blink=u["blink"], calib_rows=crow,
                                              led_offsets=led, signs=signs, landmarks=faces, tof_cal=tof_cal)
            n_unknown += len(unknown.confirmed_ever)
            cam_truth = u["cam_truth"]
            for run in runs:
                if run.run_id not in u["score"]:
                    continue
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
                cp = calib_of.get(run.run_id) if cam_cfg else None
                tof = {}
                if faces:
                    tc = tr.tof_counts()
                    rs_ = tc.pop("resid")
                    tof = {"tof_" + k: v for k, v in tc.items()}
                    tof["tof_resid_mean_cm"] = st.mean(rs_) if rs_ else float("nan")
                    tof["tof_resid_sd_cm"] = st.stdev(rs_) if len(rs_) > 1 else float("nan")
                results.append(dict(**tof,config=cfg["name"], run_id=run.run_id, robot=run.robot, route=run.level,
                                    cam_used=tr.cam_used, cam_blocked=tr.cam_blocked, kidnaps=len(tr.kidnaps), **ev,
                                    hdg_motion=tr.hdg_used["motion"], hdg_calib=tr.hdg_used["calib"],
                                    id_right=chk["right"] if has_ct else "",
                                    id_wrong=chk["wrong"] if has_ct else "",
                                    id_false=chk["none"] if has_ct else "",
                                    **{"id_" + k: v for k, v in src.items()},
                                    calib_file=_relpath(cp) if cp else "",
                                    calib_includes_run=(int(run.run_id in infos[cp]["used_runs"]) if cp else "")))
        if n_unknown:
            print(f"  [{cfg['name']}] {n_unknown} unknown object(s) seen and kept apart from the robots")

    os.makedirs(args.out, exist_ok=True)
    cols = ["config", "run_id", "robot", "route", "end_err_cm", "end_head_err_deg", "path_rmse_cm", "path_max_cm",
            "head_rmse_deg", "cam_used", "cam_blocked", "kidnaps", "hdg_motion", "hdg_calib", "id_right", "id_wrong",
            "id_false"] + ["id_" + k for k in ID_SOURCES] + ["calib_file", "calib_includes_run"] + (
            TOF_COLS if faces else [])
    with open(os.path.join(args.out, "results.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in results:
            w.writerow({k: (f"{v:.3f}" if isinstance(v, float) else v) for k, v in r.items()})

    base = {r["run_id"]: r for r in results if r["config"] == BASELINE}
    has_truth = any(not math.isnan(r["path_rmse_cm"]) for r in results)
    has_id = has_ct
    hdr = ["setup", "runs", "end error cm", "95% interval", "vs wheels only (paired)", "p", "p_holm"] + (
        ["path RMSE cm", "heading RMSE deg"] if has_truth else []) + (["right robot"] if has_id else [])
    table, pvals = [], []
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
               f"{pv:.2g}" if not math.isnan(pv) else "-", None]
        if has_truth:
            row.append(f"{st.mean(r['path_rmse_cm'] for r in rs if not math.isnan(r['path_rmse_cm'])):.2f}")
            row.append(f"{st.mean(r['head_rmse_deg'] for r in rs if not math.isnan(r['head_rmse_deg'])):.2f}")
        if has_id:
            nr = sum(r["id_right"] for r in rs)
            nt = nr + sum(r["id_wrong"] + r["id_false"] for r in rs)
            row.append(f"{100 * nr / nt:.2f}%" if nt else "-")
        table.append(row)
        pvals.append(pv)
    for row, ph in zip(table, holm(pvals)):
        row[6] = f"{ph:.2g}" if not math.isnan(ph) else "-"
    summ = [dict(zip(hdr, row), route="all") for row in table]
    # per route (setup x route): runs, mean end error, 95% interval
    routes = sorted({r["route"] for r in results}, key=lambda v: (len(str(v)), str(v)))
    for n in names:
        for rt in routes:
            vals = [r["end_err_cm"] for r in results if r["config"] == n and r["route"] == rt
                    and not math.isnan(r["end_err_cm"])]
            if not vals:
                continue
            m, sd, k, h = interval(vals)
            summ.append({"setup": n, "route": rt, "runs": k, "end error cm": f"{m:.2f}",
                         "95% interval": f"{m - h:.2f} to {m + h:.2f}" if k > 1 else "-"})
    w = [max(len(str(x)) for x in col) for col in zip(hdr, *table)]
    print("\n" + "  ".join(h.ljust(w[i]) for i, h in enumerate(hdr)))
    for row in table:
        print("  ".join(str(c).ljust(w[i]) for i, c in enumerate(row)))
    print("  p_holm: p adjusted for comparing many setups at once (Holm method)")
    print("\nPaired: each setup minus wheels-only on the same runs; negative = closer to the tape. p from a paired\n"
          "t-test (how likely a difference this big is by chance if the setups were really equal). Routes are pooled\n"
          "here; summary.csv also has each setup per route (route column).")
    with open(os.path.join(args.out, "summary.csv"), "w", newline="") as f:
        dw = csv.DictWriter(f, fieldnames=["setup", "route"] + hdr[1:])
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
    if faces:
        tof_rows = landmark_table(results, names)
        if tof_rows:
            with open(os.path.join(args.out, "landmark_ranging.csv"), "w", newline="") as f:
                dw = csv.DictWriter(f, fieldnames=list(tof_rows[0].keys()))
                dw.writeheader()
                dw.writerows(tof_rows)
            wrote.append("landmark_ranging.csv")
            print("\nLandmark ranging: what happened to the distance-sensor zone readings (one per zone per sensor "
                  "frame), per setup:")
            hd = list(tof_rows[0].keys())
            ww = [max(len(str(x)) for x in col) for col in zip(hd, *[[r[k] for k in hd] for r in tof_rows])]
            print("  ".join(h.ljust(ww[i]) for i, h in enumerate(hd)))
            for r in tof_rows:
                print("  ".join(str(r[k]).ljust(ww[i]) for i, k in enumerate(hd)))
            print("used: merged; gated: rejected, too far from what the map predicts (something unmapped in front, "
                  "or the pose is off);\nthe other columns could not be tied to one mapped face (see "
                  "tools/fusion/README.md, Landmark ranging).")
    print(f"\nwrote {args.out}/" + ", ".join(wrote))
    return 0


if __name__ == "__main__":
    sys.exit(main())
