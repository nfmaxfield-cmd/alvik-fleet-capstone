#!/usr/bin/env python3
"""Run-file builder: puts everything recorded about each run on ONE clock
(the laptop's, Unix seconds) and saves it as one folder per run.

    python tools/fusion/runs.py --session logs/20261018-101500 \\
        --sheet docs/experiments/E01-baseline-dataset/data/measurements.csv \\
        --recording recordings/20261018-101455_E1 \\
        --detections recordings/20261018-101455_E1/detections.csv \\
        --calib recordings/20261018-101455_E1/floor_calibration.json \\
        --out runs/E1

Inputs (all but --session optional):
  --session     console log folder (raw.csv, runinfo.csv, sync.csv, clock_sync.csv, ...)
  --sheet       tape measurement sheet (start and end marks per run)
  --recording   camrec.py folder: its frames.csv and the light flashes in
                video.avi give the video-to-laptop clock offset
  --detections  camera detections: frame,t_wall,x_px,y_px,score[,cls,heading_deg]
                (from a detector, the simulator, or labels.csv)
  --calib       floor_calibration.json: {"H_floor_to_px": 3x3} (floor cm -> image px)
  --video-offset  skip the flash search and use this offset (s, video minus laptop)

Output, per run: <out>/<run_id>/raw.csv (t + every sensor column),
camera.csv (t, x_cm, y_cm, score, heading_deg), meta.json (robot, route,
result, battery, tape start/end poses, sync flash times, clock fit quality).
Plus <out>/manifest.csv listing every run and what is missing.

Clock conversions:
  robot ms   -> laptop: per robot and boot, a straight-line fit to the clock
                checks (fastest answer per burst), as in check_session.py
  Pi video   -> laptop: t_wall minus the average offset between each light
                flash in the video and the same flash in the robot's log
"""
import argparse
import csv
import json
import math
import os
import statistics as st
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import check_session as cs  # noqa: E402


def fnum(x, default=float("nan")):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def pose_from_marks(xf, yf, xr, yr):
    """Floor pose (x cm, y cm, heading deg) from front/rear marks."""
    return ((xf + xr) / 2, (yf + yr) / 2, math.degrees(math.atan2(yf - yr, xf - xr)))


class Run:
    """Everything about one run, on the laptop clock."""

    def __init__(self, run_id, robot):
        self.run_id = run_id
        self.robot = robot
        self.experiment = ""
        self.level = ""
        self.result = ""
        self.battery_start = self.battery_end = None
        self.raw = []            # list of dicts, each with "t" (laptop s) added
        self.sync = {}           # tag -> (t_on, t_off) laptop s
        self.tape_start = None   # (x, y, heading_deg) floor
        self.tape_end = None
        self.targets = None      # (ahead, left, turned) relative to start
        self.excluded = False
        self.params = {}
        self.clock = {}

    @property
    def t_start(self):
        return self.raw[0]["t"] if self.raw else None

    @property
    def t_end(self):
        return self.raw[-1]["t"] if self.raw else None


def load_runs(session, sheet=None):
    """Read a console session (and optionally the tape sheet) into Run objects."""
    fits = cs.clock_fits(session)
    runs = {}
    for r in cs.rows(session, "runinfo"):
        run = runs.setdefault(r["run"], Run(r["run"], r["robot"]))
        run.experiment = r.get("experiment", "")
        run.battery_start = fnum(r.get("battery"))
    for r in cs.rows(session, "runend"):
        run = runs.setdefault(r["run"], Run(r["run"], r["robot"]))
        run.result = r.get("result", "")
        run.battery_end = fnum(r.get("battery"))
    for r in cs.rows(session, "param"):
        if r["run"] in runs:
            runs[r["run"]].params[r["name"]] = fnum(r["value"])
    for r in cs.rows(session, "raw"):
        run = runs.get(r["run"])
        if run is None:
            run = runs.setdefault(r["run"], Run(r["run"], r["robot"]))
        fit = cs.fit_for(fits, r["robot"], r["run"])
        r = dict(r)
        r["t"] = cs.to_laptop(fit, fnum(r["ms"])) if fit else fnum(r.get("t_recv"))
        r["t_from"] = "clock" if fit else "arrival"   # arrival time is late by the WiFi delay
        run.raw.append(r)
    for r in cs.rows(session, "sync"):
        run = runs.get(r["run"])
        if run is None:
            continue
        fit = cs.fit_for(fits, run.robot, r["run"])
        if fit:
            run.sync[r["tag"]] = (cs.to_laptop(fit, fnum(r["ms_on"])), cs.to_laptop(fit, fnum(r["ms_off"])))
    for run in runs.values():
        run.raw.sort(key=lambda r: r["t"])
        fit = cs.fit_for(fits, run.robot, run.run_id)
        if fit:
            run.clock = dict(fit[2])
    if sheet and os.path.exists(sheet):
        with open(sheet, newline="") as f:
            for r in csv.DictReader(f):
                rid = (r.get("run_id") or "").strip()
                if rid not in runs:
                    continue
                run = runs[rid]
                run.level = r.get("level", "")
                run.excluded = (r.get("excluded") or "").strip() in ("1", "yes")
                s4 = [fnum(r.get(k)) for k in ("xf0_cm", "yf0_cm", "xr0_cm", "yr0_cm")]
                e4 = [fnum(r.get(k)) for k in ("xf_cm", "yf_cm", "xr_cm", "yr_cm")]
                if not any(math.isnan(v) for v in s4):
                    run.tape_start = pose_from_marks(*s4)
                if not any(math.isnan(v) for v in e4):
                    run.tape_end = pose_from_marks(*e4)
                tg = [fnum(r.get(k), 0.0) for k in ("target_x_cm", "target_y_cm", "target_heading_deg")]
                run.targets = tuple(0.0 if math.isnan(v) else v for v in tg)
    return [r for r in runs.values() if r.raw], fits


# ---------------------------------------------------------------- camera
def load_homography(path):
    """Returns (floor_to_px 3x3, px_to_floor 3x3) as nested lists."""
    with open(path) as f:
        d = json.load(f)
    H = d.get("H_floor_to_px") or d.get("H") or d.get("homography")
    if H is None:
        raise ValueError(f"{path}: expected a key H_floor_to_px with a 3x3 matrix")
    return H, invert3(H)


def invert3(m):
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        raise ValueError("homography is not invertible")
    return [[(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
            [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
            [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det]]


def apply_h(H, x, y):
    u = H[0][0] * x + H[0][1] * y + H[0][2]
    v = H[1][0] * x + H[1][1] * y + H[1][2]
    w = H[2][0] * x + H[2][1] * y + H[2][2]
    return u / w, v / w


def heading_img_to_floor(H_px_to_floor, x, y, heading_img_deg):
    """Convert an image-frame heading (0 = image right, CCW on screen) to floor heading."""
    a = math.radians(heading_img_deg)
    d = 3.0   # px step along the heading; screen CCW means towards image -y
    x2, y2 = x + d * math.cos(a), y - d * math.sin(a)
    fx, fy = apply_h(H_px_to_floor, x, y)
    gx, gy = apply_h(H_px_to_floor, x2, y2)
    return math.degrees(math.atan2(gy - fy, gx - fx))


def video_offset(session, recording, fits=None):
    """Average (video t_wall - laptop time) over the light flashes found in the
    video. Returns (offset_s, spread_s, n) or None."""
    fits = fits or cs.clock_fits(session)
    syncs = cs.rows(session, "sync")
    sync_ms = [fnum(r["value"]) for r in cs.rows(session, "param") if r["name"] == "sync_ms"]
    found, frames = cs.video_flashes(recording, st.median(sync_ms) if sync_ms else 300.0)
    if not found:
        return None
    offs = []
    for s in syncs:
        fit = cs.fit_for(fits, s["robot"], s["run"])
        if not fit:
            continue
        t = cs.to_laptop(fit, fnum(s["ms_on"]))
        cands = [f for f in found if abs(f[0] - t) < 1.0]
        if cands:
            offs.append(max(cands, key=lambda f: f[2])[0] - t)
    if not offs:
        return None
    return st.mean(offs), (st.stdev(offs) if len(offs) > 1 else float("nan")), len(offs)


def load_detections(path, H_px_to_floor, offset_s, labels_ok=("confirmed", "corrected")):
    """Camera detections as floor positions on the laptop clock.
    Accepts a detections CSV (frame,t_wall,x_px,y_px,score[,cls,heading_deg])
    or a labels.csv (uses confirmed/corrected alvik rows)."""
    out = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if "status" in r and r.get("status") not in labels_ok:
                continue
            cls = r.get("cls") or "alvik"
            if cls not in ("alvik", ""):
                continue
            x, y = fnum(r.get("x_px")), fnum(r.get("y_px"))
            t = fnum(r.get("t_wall"))
            if any(math.isnan(v) for v in (x, y, t)):
                continue
            fx, fy = apply_h(H_px_to_floor, x, y)
            h = fnum(r.get("heading_deg"))
            hf = heading_img_to_floor(H_px_to_floor, x, y, h) if not math.isnan(h) else None
            out.append(dict(t=t - offset_s, frame=int(fnum(r.get("frame"), -1)), x=fx, y=fy,
                            score=fnum(r.get("score"), 1.0), heading=hf))
    out.sort(key=lambda d: d["t"])
    return out


# ---------------------------------------------------------------- write
def write_runs(runs, detections, out, extra=None):
    os.makedirs(out, exist_ok=True)
    man = []
    for run in runs:
        d = os.path.join(out, run.run_id)
        os.makedirs(d, exist_ok=True)
        cols = ["t"] + [k for k in run.raw[0].keys() if k not in ("t",)]
        with open(os.path.join(d, "raw.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            for r in run.raw:
                w.writerow({**r, "t": f"{r['t']:.4f}"})
        n_det = 0
        if detections is not None:
            t0, t1 = run.t_start - 1.0, run.t_end + 1.0
            with open(os.path.join(d, "camera.csv"), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "frame", "x_cm", "y_cm", "score", "heading_deg"])
                for det in detections:
                    if t0 <= det["t"] <= t1:
                        n_det += 1
                        w.writerow([f"{det['t']:.4f}", det["frame"], f"{det['x']:.2f}", f"{det['y']:.2f}",
                                    f"{det['score']:.3f}", "" if det["heading"] is None else f"{det['heading']:.1f}"])
        meta = dict(run_id=run.run_id, robot=run.robot, experiment=run.experiment, level=run.level,
                    result=run.result, excluded=run.excluded, battery_start=run.battery_start,
                    battery_end=run.battery_end, t_start=run.t_start, t_end=run.t_end,
                    tape_start=run.tape_start, tape_end=run.tape_end, targets=run.targets,
                    sync=run.sync, clock=run.clock, params=run.params,
                    raw_time_from=run.raw[0].get("t_from"), **(extra or {}))
        with open(os.path.join(d, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2, default=lambda o: None if isinstance(o, float) and math.isnan(o) else o)
        missing = [k for k, v in (("tape", run.tape_end), ("sync", run.sync.get("start")),
                                  ("camera", n_det if detections is not None else None)) if not v]
        man.append([run.run_id, run.robot, run.level, run.result, len(run.raw), n_det,
                    "yes" if run.tape_end else "no", ";".join(missing) or "-"])
    with open(os.path.join(out, "manifest.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["run_id", "robot", "route", "result", "raw_rows", "camera_detections", "tape", "missing"])
        w.writerows(man)
    return man


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", required=True)
    ap.add_argument("--sheet")
    ap.add_argument("--recording")
    ap.add_argument("--detections")
    ap.add_argument("--calib")
    ap.add_argument("--video-offset", type=float)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    runs, fits = load_runs(args.session, args.sheet)
    print(f"{len(runs)} runs in {args.session}")
    dets, extra = None, {}
    if args.detections:
        if not args.calib:
            sys.exit("--detections needs --calib (floor_calibration.json)")
        H, Hinv = load_homography(args.calib)
        off = args.video_offset
        if off is None:
            if not args.recording:
                sys.exit("--detections needs --recording (to find the flashes) or --video-offset")
            vo = video_offset(args.session, args.recording, fits)
            if vo is None:
                sys.exit("No light flashes matched between video and robot logs; give --video-offset")
            off, spread, n = vo
            print(f"video clock = laptop clock {off * 1000:+.0f} ms (from {n} flashes, spread {spread * 1000:.0f} ms)")
        extra = dict(video_offset_s=off)
        dets = load_detections(args.detections, Hinv, off)
        print(f"{len(dets)} camera detections")
    man = write_runs(runs, dets, args.out, extra)
    w = [max(len(str(r[i])) for r in man + [["run_id", "robot", "route", "result", "raw", "cam", "tape", "missing"]]) for i in range(8)]
    for r in [["run_id", "robot", "route", "result", "raw", "cam", "tape", "missing"]] + man:
        print("  " + "  ".join(str(c).ljust(w[i]) for i, c in enumerate(r)))
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
