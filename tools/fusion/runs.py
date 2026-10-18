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
  --calib       floor_calibration.json: {"H_floor_to_px": 3x3} (floor cm -> image px); its lens
                term k1 (cx, cy, f_px), camera height and nadir are used if present
  --calib-per-run  floor_calibration_loro.csv from tools/calibrate_floor.py: each run's
                camera positions use the calibration made WITHOUT that run (so camera
                vs tape / odometry comparisons aren't circular); runs not listed use --calib.
                A run whose calibration was made with it (fit.used_runs) is refused
                unless --allow-in-sample; meta.json records calib_file and calib_includes_run
  --video-offset  skip the flash search and use this offset (s, video minus laptop). Without it,
                <recording>/video_clock.json (tools/check_session.py) is used if there, with its
                clock drift when it says use_drift; else the flashes are searched here
  --blink       blink_detections.csv from tools/blink/blink.py: light positions
                with per-robot name probabilities (needs --calib and the clock offset)
  --calib-results  calibration_results.csv from tools/blink/calib.py: start-nudge
                headings (per run) and the light's offset on the robot
  --map         landmarks for landmark ranging: tools/fleet map.json or landmarks.csv
                (name,x1_cm,y1_cm,x2_cm,y2_cm); written to <out>/landmarks.csv as the
                faces the fusion uses (each run's raw.csv already has the distance
                sensor's zones dL_cm..dR_cm)

Output, per run: <out>/<run_id>/raw.csv (t + every sensor column),
camera.csv (t, x_cm, y_cm, score, heading_deg), meta.json (robot, route,
result, battery, tape start/end poses, sync flash times, clock fit quality).
With --blink also blink.csv (t, frame, x_cm, y_cm: the LIGHT's floor point,
not the robot centre; track, n_spots, robot, p_<name>..., p_none); with
--calib-results meta.json also gets calib_headings ([t, heading_deg, event])
and led_offset_cm ([forward, left]).
With --map, meta.json gets landmarks_file (the map given) and landmark_faces
(how many faces). Plus <out>/manifest.csv listing every run and what is missing.

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
        with open(sheet, newline="", encoding="utf-8-sig") as f:   # -sig: Excel's "CSV UTF-8" starts with a BOM
            for r in csv.DictReader(f):
                rid = (r.get("run_id") or "").strip()
                if rid not in runs:
                    continue
                run = runs[rid]
                run.level = r.get("level", "")
                run.excluded = (r.get("excluded") or "").strip().lower() in ("1", "yes", "y", "x", "true")
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
    H = None
    for k in ("H_floor_to_px", "homography_floor_cm_to_px", "H_floor_to_image", "H", "homography", "matrix",
              "floor_to_image"):    # same keys as tools/detector/detcore.py
        if d.get(k) is not None:
            H = d[k]
            break
    if H is None:
        raise ValueError(f"{path}: expected a key H_floor_to_px with a 3x3 matrix")
    return H, invert3(H)


ALVIK_TOP_CM = 4.0   # the camera sees the robot's top, about 4 cm above the floor


def load_parallax(path, obj_height_cm=None):
    """Camera height and the floor point straight below the lens, if the
    calibration file gives them (camera.height_cm, camera.nadir_floor_cm or
    camera.center_floor_cm). Needed because the homography is exact only for
    points ON the floor: the robot's top appears pushed away from the point
    below the camera by height/(camera height - height), about 3.5% at 115 cm,
    which is 1.7 cm at 50 cm from the centre. Returns None if not given."""
    with open(path) as f:
        d = json.load(f)
    cam = d.get("camera") or {}
    hc = cam.get("height_cm")
    nadir = cam.get("nadir_floor_cm") or cam.get("center_floor_cm")
    if not hc or not nadir:
        return None
    h = obj_height_cm if obj_height_cm is not None else d.get("object_height_cm", ALVIK_TOP_CM)
    return (float(nadir[0]), float(nadir[1]), float(hc), float(h))


def load_distortion(path):
    """Radial lens term from floor_calibration.json: (k1, cx, cy, f_px), or None if
    absent or 0 (then pixels go straight into the homography, as before)."""
    with open(path) as f:
        d = json.load(f)
    k1 = d.get("k1") or 0.0
    if not k1:
        return None
    if any(d.get(k) is None for k in ("cx", "cy", "f_px")):
        raise ValueError(f"{path}: k1 given without cx, cy, f_px")
    return (float(k1), float(d["cx"]), float(d["cy"]), float(d["f_px"]))


def calib_info(path):
    """What a floor_calibration.json says about how it was made: dict(path,
    used_runs (set of run ids whose tape marks or odometry made it), mode
    (points / survey), used_robots (set; survey files from 2026-10-18 on, else
    empty), held_out_runs, held_out_robots)."""
    with open(path) as f:
        d = json.load(f)
    fit = d.get("fit") or {}
    return dict(path=os.path.abspath(path), used_runs={str(r) for r in fit.get("used_runs") or []},
                mode=fit.get("mode") or "points", used_robots={str(r) for r in fit.get("used_robots") or []},
                held_out_runs=[str(r) for r in d.get("held_out_runs") or []],
                held_out_robots=[str(r) for r in d.get("held_out_robots") or []])


def read_calib_per_run(path):
    """floor_calibration_loro.csv (or the leave-one-robot-out one) -> {run_id:
    absolute path of the calibration made without it, or None if the file lists
    the run but no calibration could be made without it}."""
    base = os.path.dirname(os.path.abspath(path))
    out = {}
    with open(path, newline="", encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            rid = (r.get("run_id") or "").strip()
            if rid:
                c = (r.get("calibration") or "").strip()
                out[rid] = os.path.join(base, c) if c else None
    return out


def assign_calibrations(run_ids, calib, per_run_csv=None):
    """Which calibration file scores each run: its held-out file from
    per_run_csv if listed there, else `calib`. Returns ({run_id: path},
    {run_id: why it fell back to calib} for runs not covered by per_run_csv)."""
    per = read_calib_per_run(per_run_csv) if per_run_csv else {}
    out, fell_back = {}, {}
    for rid in run_ids:
        p = per.get(rid)
        if p:
            out[rid] = os.path.abspath(p)
            continue
        out[rid] = os.path.abspath(calib) if calib else None
        if per_run_csv:
            fell_back[rid] = ("listed with no calibration (none could be made without it)" if rid in per
                              else "not listed")
    return out, fell_back


def in_sample(assign, robots=None, infos=None):
    """Runs whose assigned calibration was made with that very run:
    [(run_id, path)]. infos: {path: calib_info} cache (filled in)."""
    infos = {} if infos is None else infos
    bad = []
    for rid, p in sorted(assign.items()):
        if not p:
            continue
        if p not in infos:
            infos[p] = calib_info(p)
        if rid in infos[p]["used_runs"]:
            bad.append((rid, p))
    return bad


def same_robot_survey(assign, robot_of, infos):
    """Runs scored with a SURVEY calibration made from other runs of the same
    robot (their wheel odometry carries that robot's wheel-size error into the
    camera fit): [(run_id, robot, path)]."""
    out = []
    for rid, p in sorted(assign.items()):
        info = infos.get(p) if p else None
        if not info or info["mode"] != "survey":
            continue
        robot = robot_of.get(rid)
        used = info["used_robots"] or {robot_of[r] for r in info["used_runs"] if r in robot_of}
        if robot and robot in used:
            out.append((rid, robot, p))
    return out


IN_SAMPLE_HELP = ("A run's camera positions would be scored with a calibration made FROM that run (its tape marks "
                  "or odometry), so the camera would be checked against the data it was fitted to (circular).")


def undistort_px(x, y, dist):
    """Pixel as the camera recorded it -> pixel the homography expects (inverse of
    pixel = c + f n (1 + k1 |n|^2), n = (homography pixel - c) / f)."""
    if not dist:
        return x, y
    k1, cx, cy, f = dist
    dx, dy = (x - cx) / f, (y - cy) / f
    nx, ny = dx, dy
    for _ in range(20):
        fac = 1 + k1 * (nx * nx + ny * ny)
        nx, ny = dx / fac, dy / fac
    return cx + f * nx, cy + f * ny


def floor_from_top(x, y, parallax):
    """Map a point seen on the robot's top (projected to the floor plane by the
    homography) back to the floor point directly below it."""
    if not parallax:
        return x, y
    nx, ny, hc, h = parallax
    k = (hc - h) / hc
    return nx + (x - nx) * k, ny + (y - ny) * k


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


def heading_img_to_floor(H_px_to_floor, x, y, heading_img_deg, dist=None):
    """Convert an image-frame heading (0 = image right, CCW on screen) to floor heading."""
    a = math.radians(heading_img_deg)
    d = 3.0   # px step along the heading; screen CCW means towards image -y
    x2, y2 = x + d * math.cos(a), y - d * math.sin(a)
    fx, fy = apply_h(H_px_to_floor, *undistort_px(x, y, dist))
    gx, gy = apply_h(H_px_to_floor, *undistort_px(x2, y2, dist))
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
    placed = []
    for s in syncs:
        fit = cs.fit_for(fits, s["robot"], s["run"])
        if fit:
            placed.append(cs.to_laptop(fit, fnum(s["ms_on"])))
    fps = cs.recording_fps(recording, frames)
    coarse, picks = cs.match_flashes(found, placed, 1 / fps)
    offs = [p[0] - t for t, p in zip(placed, picks) if p]
    if coarse is None or len(offs) < 2:
        return None
    med = st.median(offs)
    offs = [o for o in offs if abs(o - med) <= 0.05]      # drop odd ones out
    if len(offs) < 2:
        return None
    return st.mean(offs), (st.stdev(offs) if len(offs) > 1 else float("nan")), len(offs)


class VideoClock:
    """The video clock against the laptop clock from <recording>/video_clock.json (tools/check_session.py;
    docs/data-formats.md): video t_wall = laptop + offset_s + drift_ppm * 1e-6 * (laptop - t_ref). The drift
    is used only when the file says use_drift (it is real and worth 1 ms or more), else offset_s alone."""

    def __init__(self, offset_s, drift_ppm=0.0, t_ref=0.0, path=None):
        self.offset_s, self.drift, self.t_ref, self.path = float(offset_s), float(drift_ppm) * 1e-6, float(t_ref), path

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = json.load(f)
        drift = float(d.get("drift_ppm") or 0.0) if d.get("use_drift") else 0.0
        return cls(d["offset_s"], drift, d.get("t_ref") or 0.0, path)

    def to_laptop(self, t_wall):
        return (t_wall - self.offset_s + self.drift * self.t_ref) / (1.0 + self.drift)

    def describe(self):
        s = f"video clock = laptop clock {self.offset_s * 1000:+.0f} ms"
        if self.drift:
            s += f", drifting {self.drift * 1e6:+.1f} ppm"
        return s + (f" (from {os.path.basename(self.path)})" if self.path else "")


def to_laptop(clock, t_wall):
    """Video t_wall -> laptop time: clock is a VideoClock or a plain offset (s, video minus laptop)."""
    return clock.to_laptop(t_wall) if isinstance(clock, VideoClock) else t_wall - clock


def video_clock(session, recording, fits=None, explicit=None):
    """How video time maps to laptop time: `explicit` (--video-offset) if given; else the recording's
    video_clock.json (check_session.py wrote it from the flashes: with the drift if use_drift); else the
    flash search here (video_offset). Returns (clock, text) or None."""
    if explicit is not None:
        return float(explicit), f"video clock = laptop clock {explicit * 1000:+.0f} ms (given)"
    if recording and os.path.exists(os.path.join(recording, "video_clock.json")):
        try:
            c = VideoClock.load(os.path.join(recording, "video_clock.json"))
            return c, c.describe()
        except (OSError, ValueError, KeyError, TypeError) as e:
            print(f"note: {recording}/video_clock.json unreadable ({e}); finding the flashes instead")
    vo = video_offset(session, recording, fits) if recording else None
    if vo is None:
        return None
    return vo[0], (f"video clock = laptop clock {vo[0] * 1000:+.0f} ms (from {vo[2]} flashes, spread "
                   f"{vo[1] * 1000:.0f} ms)")


def load_detections(path, H_px_to_floor, offset_s, labels_ok=("confirmed", "corrected", "auto"), parallax=None,
                    dist=None):
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
            fx, fy = floor_from_top(*apply_h(H_px_to_floor, *undistort_px(x, y, dist)), parallax)
            h = fnum(r.get("heading_deg"))
            hf = heading_img_to_floor(H_px_to_floor, x, y, h, dist) if not math.isnan(h) else None
            out.append(dict(t=to_laptop(offset_s, t), frame=int(fnum(r.get("frame"), -1)), x=fx, y=fy,
                            score=fnum(r.get("score"), 1.0), heading=hf))
    out.sort(key=lambda d: d["t"])
    return out


LED_OFFSET_CM = (2.75, 0.0)   # light (LED midpoint) ahead of the robot centre, cm (forward, left):
                              # the simulator's model; calib.py measures it (calibration_results.csv)


def load_blink(path, H_px_to_floor, offset_s, parallax=None, dist=None, lit_only=True):
    """blink_detections.csv (tools/blink/blink.py) as floor positions of the
    LIGHT on the laptop clock. Same corrections as load_detections: lens term,
    then homography, then the height correction (the LEDs are on the robot's
    top, about 4 cm up, like the detector's robot-top point). Rows filled in
    between flashes (lit 0, --interpolate) are left out unless lit_only=False.
    Each row: t, frame, x, y, track, robot (name if blink.py named it, else ""),
    probs {name: p}, p_none, n_spots, score."""
    out = []
    with open(path, newline="") as f:
        rd = csv.DictReader(f)
        names = [c[2:] for c in (rd.fieldnames or []) if c.startswith("p_") and c != "p_none"]
        for r in rd:
            if lit_only and fnum(r.get("lit"), 1.0) < 0.5:
                continue
            x, y, t = fnum(r.get("x_px")), fnum(r.get("y_px")), fnum(r.get("t_wall"))
            if any(math.isnan(v) for v in (x, y, t)):
                continue
            fx, fy = floor_from_top(*apply_h(H_px_to_floor, *undistort_px(x, y, dist)), parallax)
            probs = {n: fnum(r.get("p_" + n), 0.0) for n in names}
            probs = {n: (0.0 if math.isnan(p) else p) for n, p in probs.items()}
            pn = fnum(r.get("p_none"), 0.0)
            out.append(dict(t=to_laptop(offset_s, t), frame=int(fnum(r.get("frame"), -1)), x=fx, y=fy,
                            track=int(fnum(r.get("track"), -1)), robot=(r.get("robot") or "").strip(),
                            probs=probs, p_none=0.0 if math.isnan(pn) else pn,
                            n_spots=int(fnum(r.get("n_spots"), 0)), score=fnum(r.get("score"), 0.0)))
    out.sort(key=lambda d: d["t"])
    return out


def load_calib_results(path, offset_s):
    """calibration_results.csv (tools/blink/calib.py): one dict per roll with
    run, robot, event, t (laptop clock: wheels start), heading (floor deg or
    None), led (forward, left) cm or None, identity."""
    out = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            tv, h = fnum(r.get("t_video")), fnum(r.get("heading_floor_deg"))
            lf, ll = fnum(r.get("led_fwd_cm")), fnum(r.get("led_left_cm"))
            mp = fnum(r.get("moved_px"))
            ws, gs = fnum(r.get("wheel_scale")), fnum(r.get("gyro_scale"))
            ok = lambda k: (r.get(k) or "").strip().lower().startswith("ok")
            out.append(dict(run=(r.get("run") or "").strip(), robot=(r.get("robot_log") or "").strip(),
                            event=(r.get("event") or "").strip(), identity=(r.get("identity") or "").strip(),
                            t=None if math.isnan(tv) else to_laptop(offset_s, tv),
                            heading=None if math.isnan(h) else h,
                            led=None if (math.isnan(lf) or math.isnan(ll)) else (lf, ll),
                            moved_px=None if math.isnan(mp) else mp,
                            # calibration runs from firmware 2026-10-18b (15 cm roll, 360 deg spins): camera cm /
                            # odometry cm and camera deg / gyro deg, only where calib.py rated them ok
                            wheel_scale=ws if (ws > 0 and ok("wheel_scale_quality")) else None,
                            wheel_scale_spread=fnum(r.get("wheel_scale_spread")),
                            gyro_scale=gs if (gs > 0 and ok("gyro_scale_quality")) else None,
                            gyro_scale_spread=fnum(r.get("gyro_scale_spread"))))
    return out


def calib_scales(calib_rows):
    """Per robot, from the calibration runs' rolls and spins (calibration_results.csv): dict(k, sk) the
    wheel distance scale for the fusion (k = wheels' reading / true distance = 1 / wheel_scale) and how sure
    (the forward / back spread, at least 0.3%), and gs (multiply gyro turns by it). Medians over a robot's
    rows; robots or values without an ok row are left out."""
    by = defaultdict(list)
    for r in calib_rows or []:
        if r.get("event") == "calib" and r.get("identity", "").lower() != "disagree":
            by[r["robot"]].append(r)
    out = {}
    for robot, rows in by.items():
        d = {}
        ws = [r["wheel_scale"] for r in rows if r.get("wheel_scale")]
        if ws:
            sp = [abs(r["wheel_scale_spread"]) for r in rows if r.get("wheel_scale") and
                  not math.isnan(r.get("wheel_scale_spread", float("nan")))]
            d["k"] = 1.0 / st.median(ws)
            d["sk"] = max(st.median(sp) if sp else 0.0, 0.003)
        gs = [r["gyro_scale"] for r in rows if r.get("gyro_scale")]
        if gs:
            d["gs"] = st.median(gs)
        if d:
            out[robot] = d
    return out


def view_area_cm2(path):
    """Floor area (cm^2) the camera sees: the calibration file's view_floor_polygon_cm, else its image
    corners (image_size) mapped to the floor. None if neither is there."""
    with open(path) as f:
        d = json.load(f)
    poly = d.get("view_floor_polygon_cm")
    if not poly and d.get("image_size"):
        w, h = d["image_size"]
        Hinv = load_homography(path)[1]
        dist = load_distortion(path)
        poly = [apply_h(Hinv, *undistort_px(x, y, dist)) for x, y in ((0, 0), (w, 0), (w, h), (0, h))]
    if not poly or len(poly) < 3:
        return None
    a = 0.0
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        a += x0 * y1 - x1 * y0
    return abs(a) / 2


def heading_px_sd(path):
    """From heading_check.csv (tools/heading_check.py): how precisely the camera finds a start nudge's
    shift, in px, so start-nudge headings get the noise measured on this setup. The camera's heading spread:
    half the variance of (detector heading - nudge heading) where both exist (3 or more runs), else the
    whole spread of (camera - tape) (an upper bound: tape error is in it too). Turned into px with the
    median moved_px: px_sd = tan(spread) x median moved_px. None if the file has too few rows."""
    rows = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    diffs = [fnum(r.get("diff_deg")) for r in rows]
    diffs = [d for d in diffs if not math.isnan(d)]
    pairs = []
    for r in rows:
        a, b = fnum(r.get("det_heading_deg")), fnum(r.get("camera_heading_deg"))
        if not (math.isnan(a) or math.isnan(b)):
            pairs.append((a - b + 180.0) % 360.0 - 180.0)
    moved = [fnum(r.get("moved_px")) for r in rows]
    moved = [m for m in moved if m > 0]
    if len(diffs) < 3 or not moved:
        return None
    sd = math.sqrt(st.variance(pairs) / 2) if len(pairs) >= 3 else st.stdev(diffs)
    return math.tan(math.radians(sd)) * st.median(moved)


def led_offsets(calib_rows, default=LED_OFFSET_CM):
    """Light offset per robot (median of its rolls' led_fwd_cm / led_left_cm),
    the median over all robots under None, or `default` if no roll measured it.
    Returns ({robot or None: (forward, left)}, where it came from)."""
    by = defaultdict(list)
    for r in calib_rows or []:
        if r["led"] is not None and r["identity"].lower() != "disagree":
            by[r["robot"]].append(r["led"])
    allv = [v for vs in by.values() for v in vs]
    if not allv:
        return {None: tuple(default)}, f"setting ({default[0]:.2f}, {default[1]:.2f}) cm"
    med = lambda vs: (st.median(v[0] for v in vs), st.median(v[1] for v in vs))
    out = {robot: med(vs) for robot, vs in by.items()}
    out[None] = med(allv)
    return out, f"calibration_results.csv ({len(allv)} rolls)"


# ---------------------------------------------------------------- write
def write_runs(runs, detections, out, extra=None, blink=None, calib_rows=None, led=None, calib_of=None,
               calib_infos=None):
    """calib_of: {run_id: calibration file used for its camera positions}
    (written to meta.json as calib_file, with calib_includes_run)."""
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
        if isinstance(detections, dict):      # per-run detections (--calib-per-run): no silent fallback
            dets_run = detections.get(run.run_id)
        else:
            dets_run = detections
        if dets_run is not None:
            t0, t1 = run.t_start - 1.0, run.t_end + 1.0
            with open(os.path.join(d, "camera.csv"), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "frame", "x_cm", "y_cm", "score", "heading_deg"])
                for det in dets_run:
                    if t0 <= det["t"] <= t1:
                        n_det += 1
                        w.writerow([f"{det['t']:.4f}", det["frame"], f"{det['x']:.2f}", f"{det['y']:.2f}",
                                    f"{det['score']:.3f}", "" if det["heading"] is None else f"{det['heading']:.1f}"])
        if blink is not None:
            b_run = (blink.get(run.run_id) or []) if isinstance(blink, dict) else blink
            names = sorted({n for b in b_run for n in b["probs"]})
            t0, t1 = run.t_start - 1.0, run.t_end + 1.0
            with open(os.path.join(d, "blink.csv"), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "frame", "x_cm", "y_cm", "track", "n_spots", "robot"] + ["p_" + n for n in names]
                           + ["p_none"])
                for b in b_run:
                    if t0 <= b["t"] <= t1:
                        w.writerow([f"{b['t']:.4f}", b["frame"], f"{b['x']:.2f}", f"{b['y']:.2f}", b["track"],
                                    b["n_spots"], b["robot"]] + [f"{b['probs'].get(n, 0.0):.3f}" for n in names]
                                   + [f"{b['p_none']:.3f}"])
        run_extra = dict(extra or {})
        if calib_rows is not None:
            run_extra["calib_headings"] = [[c["t"], c["heading"], c["event"]] for c in calib_rows
                                           if c["run"] == run.run_id and c["t"] is not None and c["heading"] is not None]
        if led is not None:
            run_extra["led_offset_cm"] = list(led.get(run.robot, led[None]))
        if calib_of is not None:
            cp = calib_of.get(run.run_id)
            run_extra["calib_file"] = cp
            run_extra["calib_includes_run"] = run.run_id in (calib_infos or {}).get(cp, {}).get("used_runs", ())
        meta = dict(run_id=run.run_id, robot=run.robot, experiment=run.experiment, level=run.level,
                    result=run.result, excluded=run.excluded, battery_start=run.battery_start,
                    battery_end=run.battery_end, t_start=run.t_start, t_end=run.t_end,
                    tape_start=run.tape_start, tape_end=run.tape_end, targets=run.targets,
                    sync=run.sync, clock=run.clock, params=run.params,
                    raw_time_from=run.raw[0].get("t_from"), **run_extra)
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
    ap.add_argument("--calib-per-run", help="floor_calibration_loro.csv: score each run with the calibration made without it")
    ap.add_argument("--allow-in-sample", action="store_true",
                    help="convert a run's camera positions with a calibration made from that run anyway (circular)")
    ap.add_argument("--video-offset", type=float)
    ap.add_argument("--blink", help="blink_detections.csv (tools/blink/blink.py)")
    ap.add_argument("--calib-results", help="calibration_results.csv (tools/blink/calib.py)")
    ap.add_argument("--map", help="map.json (tools/fleet) or landmarks.csv: landmark faces for landmark ranging")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    runs, fits = load_runs(args.session, args.sheet)
    print(f"{len(runs)} runs in {args.session}")
    dets, extra = None, {}
    blink, crow, led = None, None, None
    if args.detections or args.blink or args.calib_results:
        if not args.calib and (args.detections or args.blink):
            sys.exit("--detections / --blink need --calib (floor_calibration.json)")
        if args.video_offset is None and not args.recording:
            sys.exit("--detections needs --recording (to find the flashes) or --video-offset")
        vc = video_clock(args.session, args.recording, fits, args.video_offset)
        if vc is None:
            sys.exit("No light flashes matched between video and robot logs; give --video-offset")
        off, how = vc
        print(how)
        if isinstance(off, VideoClock):
            extra = dict(video_offset_s=off.offset_s, video_drift_ppm=off.drift * 1e6, video_t_ref=off.t_ref)
        else:
            extra = dict(video_offset_s=off)
        if args.calib_results:
            crow = load_calib_results(args.calib_results, off)
            led, how = led_offsets(crow)
            print(f"{len(crow)} calibration rolls; light offset from {how}")
    calib_of, infos = None, {}
    if args.detections or args.blink:
        calib_of, fell_back = assign_calibrations([r.run_id for r in runs], args.calib, args.calib_per_run)
        for rid, why in sorted(fell_back.items()):
            print(f"note: {rid} {why} in {args.calib_per_run}: uses {args.calib}")
        bad = in_sample(calib_of, infos=infos)
        if not args.calib_per_run:
            over = sorted({r.run_id for r in runs} & calib_info(args.calib)["used_runs"])
            if over:
                print(f"WARNING: {args.calib} was made from {len(over)} of these runs ({', '.join(over[:8])}"
                      f"{' ...' if len(over) > 8 else ''}): give --calib-per-run floor_calibration_loro.csv")
        if bad and not args.allow_in_sample:
            sys.exit(f"Refusing: {len(bad)} run(s) would get camera positions from a calibration made with them: "
                     + ", ".join(f"{rid} ({os.path.basename(p)})" for rid, p in bad[:8])
                     + f"{' ...' if len(bad) > 8 else ''}.\n{IN_SAMPLE_HELP}\nUse --calib-per-run "
                     "floor_calibration_loro.csv (from tools/calibrate_floor.py), or --allow-in-sample to do it anyway "
                     "(meta.json then says calib_includes_run: true).")
        groups = defaultdict(list)
        for rid, p in calib_of.items():
            groups[p].append(rid)
        if args.calib_per_run:
            print(f"{sum(1 for p in calib_of.values() if p != os.path.abspath(args.calib))} runs use their "
                  f"held-out calibration ({args.calib_per_run})")
            extra["calib_per_run"] = os.path.abspath(args.calib_per_run)
        conv = {p: (load_homography(p)[1], load_parallax(p), load_distortion(p)) for p in groups}
        if any(c[1] is None for c in conv.values()):
            print("note: calibration has no camera height; robot-top positions not corrected for height")
        span = {r.run_id: (r.t_start - 1.0, r.t_end + 1.0) for r in runs}

        def per_run(load):
            """One list for every run: its own time window, converted with its own calibration
            (a plain list when every run uses the same calibration)."""
            if len(groups) == 1:
                p = next(iter(groups))
                rows = load(*conv[p])
                return rows, len(rows)
            out, n = {}, 0
            for p, rids in groups.items():
                rows = load(*conv[p])
                n = len(rows)
                for rid in rids:
                    out[rid] = [d for d in rows if span[rid][0] <= d["t"] <= span[rid][1]]
            return out, n
        if args.blink:
            blink, n = per_run(lambda Hi, par, dist: load_blink(args.blink, Hi, off, parallax=par, dist=dist))
            print(f"{n} blink light sightings")
        if args.detections:
            dets, n = per_run(lambda Hi, par, dist: load_detections(args.detections, Hi, off, parallax=par, dist=dist))
            print(f"{n} camera detections")
    if args.map:
        import landmarks as lm
        faces = lm.load(args.map)
        os.makedirs(args.out, exist_ok=True)
        lm.write_csv(faces, os.path.join(args.out, "landmarks.csv"))
        extra = dict(extra, landmarks_file=os.path.abspath(args.map), landmark_faces=len(faces))
        print(f"{len(faces)} landmark faces from {args.map} -> {os.path.join(args.out, 'landmarks.csv')}")
    man = write_runs(runs, dets, args.out, extra, blink=blink, calib_rows=crow, led=led, calib_of=calib_of,
                     calib_infos=infos)
    w = [max(len(str(r[i])) for r in man + [["run_id", "robot", "route", "result", "raw", "cam", "tape", "missing"]]) for i in range(8)]
    for r in [["run_id", "robot", "route", "result", "raw", "cam", "tape", "missing"]] + man:
        print("  " + "  ".join(str(c).ljust(w[i]) for i, c in enumerate(r)))
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
