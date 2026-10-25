#!/usr/bin/env python3
"""Every accuracy check that can be worked out from the T1 data, in one table.

    python tools/accuracy_checks.py docs/experiments/E01-baseline-dataset docs/experiments/E01b-low-battery \\
        docs/experiments/E00b-nudge-check docs/experiments/E00-measurement-check

Give it the experiment folders (any of them, any order: each is recognised by
its name, E01- = T1, E01b = low battery, E00b = start-roll check, E00- = T0;
or name them with --t1 / --low-battery / --nudge / --t0). From each folder it
reads:
  data/measurements.csv          the tape sheet
  data/logs/<session>/           console logs (raw.csv, runinfo.csv, straight.csv, ...)
  data/video/<recording>/        camera recordings (also any --recording folder), each with
                                 calibration_results.csv (tools/blink/calib.py),
                                 detections.csv or camera_detections.csv (robot detector),
                                 blink_detections.csv (tools/blink/blink.py),
                                 floor_calibration.json (+ floor_calibration_loro.csv and
                                 floor_points.json from tools/calibrate_floor.py) in the
                                 recording or the folder above it, or --calib

It prints one line per check (what it found, PASS / CHECK / FAIL, or "not
available yet (needs ...)" when an input is missing), then each check's pass
rule and what to do if it fails. Writes, in --out (default: T1's data folder):
  accuracy_checks.csv    the table
  measured_noise.json    the camera noise measured against the tape (for the fusion)
and T1's data/split.csv (which runs tune the fusion and which score it).

Checks: 1 mark spacing, 2 wheel distance scale, 3 wheel turn scale, 4 gyro
scale, 5 gyro bias/noise at rest (and at low battery), 6 wheels vs gyro on
turns, 7 gyro vs wheel timing, 8 camera vs tape at still moments, 9 robot moved
before its end marks, 10 error vs run order and battery, 11 blink identity,
12 robot vs robot, 13 statistics rules (bootstrap, E1b paired, held-out
split), 14 simulator preset for the real camera, 15 the fusion's "standing
still" test at rest.

--video-offset (s, video clock minus laptop clock) skips the slow search for
the light flashes in the video.
"""
import argparse
import csv
import datetime as dt
import functools
import glob
import json
import math
import os
import statistics as st
import sys
import textwrap
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (HERE, os.path.join(HERE, "fusion")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import check_session as cs  # noqa: E402
import runs as fr  # noqa: E402
import tape_results as tr  # noqa: E402
from heading_check import tape_heading  # noqa: E402
from summarize import t_quantile, t_cdf  # noqa: E402

PASS, CHECK, FAIL, NOTYET = "PASS", "CHECK", "FAIL", "NOT YET"
MARK_SPACING_CM = 9.5            # front face to rear face; T0 measured 9.52, SD 0.08 (n = 10)
SIM_PRESET = os.path.join(HERE, "sim", "configs", "real_oct18.json")

# The flash search reads the whole video; do it once per recording, however many sessions ask.
cs.video_flashes = functools.lru_cache(maxsize=None)(cs.video_flashes)


# ================================================================ small statistics
def num(x):
    try:
        v = float(x)
        return v if math.isfinite(v) else None
    except (TypeError, ValueError):
        return None


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def t_int(vals):
    """mean, sd, n, half-width of the 95% t-interval (nan when n < 2)."""
    vals = [v for v in vals if v is not None and math.isfinite(v)]
    if not vals:
        return float("nan"), float("nan"), 0, float("nan")
    return tr.interval(vals)


def paired_t(diffs):
    """Paired t-test on a list of differences: mean, half-width, p, n."""
    m, sd, n, h = t_int(diffs)
    if n < 2 or not sd:
        return m, h, float("nan"), n
    t = m / (sd / math.sqrt(n))
    return m, h, 2 * (1 - t_cdf(abs(t), n - 1)), n


def wilson(k, n):
    """95% interval for a proportion k/n (Wilson's method; works for k = 0)."""
    if n == 0:
        return float("nan"), float("nan")
    z = 1.96
    p = k / n
    c = (p + z * z / (2 * n)) / (1 + z * z / n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return max(0.0, c - h), min(1.0, c + h)


def within_group_slope(points):
    """points: [(group, x, y)]. Slope of y on x after taking out each group's own
    mean (so route and robot differences don't count as a trend). Returns
    (slope, half-width of 95% interval, n) or None."""
    by = defaultdict(list)
    for g, x, y in points:
        if x is not None and y is not None:
            by[g].append((x, y))
    xs, ys, ng = [], [], 0
    for pts in by.values():
        if len(pts) < 2:
            continue
        ng += 1
        mx, my = st.mean(p[0] for p in pts), st.mean(p[1] for p in pts)
        xs += [p[0] - mx for p in pts]
        ys += [p[1] - my for p in pts]
    n = len(xs)
    sxx = sum(x * x for x in xs)
    if n - ng - 1 < 1 or sxx == 0:
        return None
    b = sum(x * y for x, y in zip(xs, ys)) / sxx
    resid = sum((y - b * x) ** 2 for x, y in zip(xs, ys))
    se = math.sqrt(resid / (n - ng - 1) / sxx)
    return b, t_quantile(0.975, n - ng - 1) * se, n


def f(x, d=2, sign=False):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "-"
    return f"{x:+.{d}f}" if sign else f"{x:.{d}f}"


def ci_text(m, h, d=2, unit=""):
    if h is None or not math.isfinite(h):
        return f"{f(m, d, True)}{unit} (too few runs for an interval)"
    return f"{f(m, d, True)}{unit} (95% interval {f(m - h, d, True)} to {f(m + h, d, True)})"


def within(m, h, limit):
    """PASS if the whole 95% interval is inside +/-limit, FAIL if the mean is outside, else CHECK."""
    if m is None or not math.isfinite(m):
        return NOTYET
    if abs(m) > limit:
        return FAIL
    if h is not None and math.isfinite(h) and abs(m) + h <= limit:
        return PASS
    return CHECK


# ================================================================ inputs
ROLES = (("t1", "E01-"), ("low", "E01b"), ("nudge", "E00b"), ("t0", "E00-"))


def role_of(folder):
    name = os.path.basename(os.path.normpath(folder))
    for role, key in ROLES:
        if key.lower() in name.lower():
            return role
    return None


def route_of(level):
    lv = (level or "").strip().lower()
    if lv.startswith("straight"):
        return "straight"
    if lv.startswith("square"):
        return "square"
    if lv == "l" or lv.startswith("l-") or lv.startswith("lpath") or lv.startswith("l_"):
        return "L"
    return lv or "?"


def excluded(r):
    return (r.get("excluded") or "").strip().lower() in ("1", "yes", "y", "x", "true")


REC_FILES = ("frames.csv", "calibration_results.csv", "detections.csv", "camera_detections.csv",
             "blink_detections.csv", "floor_calibration.json")


class Experiment:
    def __init__(self, folder, role):
        self.folder, self.role = folder, role
        self.sheet = os.path.join(folder, "data", "measurements.csv")
        self.rows, self.trials = [], []
        if os.path.isfile(self.sheet):
            with open(self.sheet, newline="", encoding="utf-8-sig") as fh:
                self.rows = list(csv.DictReader(fh))
            self.trials = tr.load_trials(folder)
        self.sessions = sorted(d for d in glob.glob(os.path.join(folder, "data", "logs", "*")) if os.path.isdir(d))
        self.runs = {}            # run_id -> fusion Run (laptop clock)
        self.straight = {}        # run_id -> STRAIGHT record
        self.runinfo = {}         # run_id -> RUNINFO record
        self.session_of = {}
        for s in self.sessions:
            try:
                runs, _ = fr.load_runs(s, self.sheet if os.path.isfile(self.sheet) else None)
            except Exception as e:       # a broken log folder must not stop the other checks
                print(f"note: could not read {s}: {e}")
                continue
            for r in runs:
                self.runs[r.run_id] = r
                self.session_of[r.run_id] = s
            for r in cs.rows(s, "straight"):
                self.straight[r.get("run", "")] = r
            for r in cs.rows(s, "runinfo"):
                self.runinfo[r.get("run", "")] = r
        # runs the edge guard stopped before an open plywood edge ("limited:plywood edge"): not failures, but not
        # finished routes either, so every statistic here leaves them out; counted apart (self.limited)
        self.limited = tr.limited_trials(folder)
        self.drop_limited()
        self.recordings = sorted(d for pat in ("video/*", "recording*", "recordings/*")
                                 for d in glob.glob(os.path.join(folder, "data", pat))
                                 if os.path.isdir(d) and any(os.path.isfile(os.path.join(d, n)) for n in REC_FILES))

    def drop_limited(self):
        """Move measured trials whose RUNEND says limited (row not marked by run_sheet.py) to self.limited."""
        by_log = [t for t in self.trials
                  if cs.sc.run_outcome(getattr(self.runs.get((t.get("run_id") or "").strip()), "result", "done"))
                  == cs.sc.CALIB_LIMITED]
        self.limited += by_log
        self.trials = [t for t in self.trials if t not in by_log]

    def trial_runs(self):
        """[(trial, Run or None)] for every measured, not-excluded trial."""
        return [(t, self.runs.get((t.get("run_id") or "").strip())) for t in self.trials]


# ================================================================ per-run sensor summary
_SUMMARY = {}


def sensor_summary(run, runinfo=None):
    """Cached: see _sensor_summary."""
    key = (id(run), run.run_id)
    if key not in _SUMMARY:
        _SUMMARY[key] = _sensor_summary(run, runinfo)
    return _SUMMARY[key]


def _sensor_summary(run, runinfo=None):
    """What the robot's own sensors say about one run (left-positive degrees):
    wheel_total / gyro_total (accumulated heading change over the run, gyro with
    its at-rest bias taken out), still-period gyro bias and noise, turn segments,
    and turn-start lags. None if the RAW log is too short."""
    rows = []
    for r in run.raw:
        ms = num(r.get("ms"))
        if ms is None:
            continue
        rows.append(dict(ms=ms, t=r["t"], wl=num(r.get("wl_deg")), wr=num(r.get("wr_deg")),
                         lr=num(r.get("wl_rpm")) or 0.0, rr=num(r.get("wr_rpm")) or 0.0, gz=num(r.get("gz")),
                         yaw=num(r.get("yaw_deg")), th=num(r.get("th_deg")), x=num(r.get("x_cm")),
                         y=num(r.get("y_cm")), w=num(r.get("w_dps")), lifted=num(r.get("lifted")) or 0))
    rows = [r for r in rows if None not in (r["wl"], r["wr"], r["gz"], r["yaw"], r["th"])]
    rows.sort(key=lambda r: r["ms"])
    if len(rows) < 20:
        return None
    ri = runinfo or {}
    odo_sign = num(ri.get("odo_sign")) or 1.0
    n = len(rows)
    dwl = [0.0] + [rows[i]["wl"] - rows[i - 1]["wl"] for i in range(1, n)]
    dwr = [0.0] + [rows[i]["wr"] - rows[i - 1]["wr"] for i in range(1, n)]
    dyaw = [0.0] + [wrap180(rows[i]["yaw"] - rows[i - 1]["yaw"]) for i in range(1, n)]
    dth = [0.0]
    for i in range(1, n):
        a, b = rows[i - 1], rows[i]
        reset = b["x"] == 0 and b["y"] == 0 and b["th"] == 0 and not (a["x"] == 0 and a["y"] == 0 and a["th"] == 0)
        dth.append(0.0 if reset else (b["th"] - a["th"]) * odo_sign)
    # signs: odometry heading (th x odo_sign) is left-positive by definition; the IMU's yaw and the gyro
    # are matched to it (RUNINFO yaw_sign when the compass check ran, else from how they move together)
    yaw_sign = num(ri.get("yaw_sign")) or 0.0
    if not yaw_sign:
        c = sum(a * b for a, b in zip(dth, dyaw))
        yaw_sign = 1.0 if c >= 0 else -1.0
    dyaw = [d * yaw_sign for d in dyaw]
    c = sum(r["gz"] * d for r, d in zip(rows, dyaw))
    gz_sign = 1.0 if c >= 0 else -1.0
    gz = [r["gz"] * gz_sign for r in rows]
    # still rows: wheels not turning, not lifted
    still = [abs(dwl[i]) < 0.5 and abs(dwr[i]) < 0.5 and abs(rows[i]["lr"]) < 3.0 and abs(rows[i]["rr"]) < 3.0
             and rows[i]["lifted"] < 0.5 for i in range(n)]
    # how far the wheels have rolled so far (cm, both ways counted; 1.7 cm wheel radius is close enough
    # for this): the start nudge rolls about 2 x 2 cm and comes back to the start marks
    travel, acc = [], 0.0
    for i in range(n):
        acc += (abs(dwl[i]) + abs(dwr[i])) / 2 / 360 * 2 * math.pi * 1.7
        travel.append(acc)
    segs, cur = [], []
    for i in range(n):
        if still[i]:
            cur.append(i)
        elif cur:
            segs.append(cur)
            cur = []
    if cur:
        segs.append(cur)
    still_segs = []
    for sgm in segs:
        t0 = rows[sgm[0]]["ms"] + 200          # skip the first 0.2 s (settling after a move)
        keep = [i for i in sgm if rows[i]["ms"] >= t0]
        if keep and rows[keep[-1]]["ms"] - rows[keep[0]]["ms"] >= 500:
            still_segs.append(keep)
    still_vals = [gz[i] for sgm in still_segs for i in sgm]
    bias = st.mean(still_vals) if len(still_vals) >= 10 else None
    resid = [gz[i] - st.mean(gz[j] for j in sgm) for sgm in still_segs for i in sgm]
    noise = math.sqrt(sum(e * e for e in resid) / max(1, len(resid) - len(still_segs))) if len(resid) >= 10 else None
    dur = (rows[-1]["ms"] - rows[0]["ms"]) / 1000
    wheel_total = sum(dth)
    gyro_raw = sum(dyaw)
    gyro_total = gyro_raw - (bias or 0.0) * dur
    # in-place turns: wheels turning opposite ways
    turn_rows = [i for i in range(1, n) if abs(dwr[i] - dwl[i]) >= 1.0 and abs(dwr[i] + dwl[i]) <= 0.5 * abs(dwr[i] - dwl[i])]
    turns, cur = [], []
    for i in turn_rows:
        if cur and i - cur[-1] > 3:
            turns.append(cur)
            cur = []
        cur.append(i)
    if cur:
        turns.append(cur)
    tlist = []
    for tt in turns:
        a = max(1, tt[0] - 5)
        b = min(n - 1, tt[-1] + 15)
        w = sum(dth[a:b + 1])
        if abs(w) < 30:
            continue                       # compass wiggles and corrections, not route turns
        g = sum(dyaw[a:b + 1]) - (bias or 0.0) * (rows[b]["ms"] - rows[a - 1]["ms"]) / 1000
        tlist.append(dict(wheel=w, gyro=g, ms0=rows[tt[0]]["ms"], lag_ms=turn_lag(rows, gz, tt[0])))
    # still moments on the tape marks: before the route proper starts (wheels rolled no further than the
    # start nudge's forward-and-back, which comes back to the marks), and the last still stretch (to the
    # end of the log: the robot stays put for its end marks)
    nudge = (getattr(run, "params", {}) or {}).get("start_nudge_cm")
    nudge_travel = 2 * (nudge if nudge is not None and math.isfinite(nudge) and nudge > 0 else 2.0) + 1.0
    start_iv = [(rows[sg[0]]["t"] - (0.5 if sg[0] == 0 else -0.1), rows[sg[-1]]["t"] - 0.05)
                for sg in segs if travel[sg[-1]] < nudge_travel and rows[sg[-1]]["t"] - rows[sg[0]]["t"] >= 0.2]
    end_iv = None
    if segs and segs[-1][-1] == n - 1 and travel[-1] >= nudge_travel:
        end_iv = (rows[segs[-1][0]]["t"] + 0.2, rows[-1]["t"] + 0.5)   # still, by its own log, to RUNEND
    moving = [i for i in range(n) if not still[i]]
    return dict(start_iv=start_iv, end_iv=end_iv, travel_cm=travel[-1], nudge_travel_cm=nudge_travel,
                n=n, dur=dur, wheel_total=wheel_total, gyro_total=gyro_total, gyro_raw=gyro_raw, bias=bias,
                noise=noise, still_s=sum((rows[s[-1]]["ms"] - rows[s[0]]["ms"]) / 1000 for s in still_segs),
                turns=tlist, t_first=rows[0]["t"], t_last=rows[-1]["t"],
                t_move0=rows[moving[0]]["t"] if moving else None, t_move1=rows[moving[-1]]["t"] if moving else None)


def turn_lag(rows, gz, i0, before_ms=400, after_ms=700, max_lag_ms=200, step_ms=2.0):
    """Delay (ms) of the gyro's rate behind the wheels' turn rate at one turn start,
    by cross-correlation (the shift that lines the two curves up best). + = gyro later."""
    import numpy as np
    ms0 = rows[i0]["ms"]
    idx = [i for i in range(len(rows)) if ms0 - before_ms - max_lag_ms <= rows[i]["ms"] <= ms0 + after_ms + max_lag_ms]
    if len(idx) < 15:
        return None
    t = np.array([rows[i]["ms"] for i in idx], float)
    w = np.array([rows[i]["rr"] - rows[i]["lr"] for i in idx], float)
    g = np.array([gz[i] for i in idx], float)
    grid = np.arange(ms0 - before_ms, ms0 + after_ms, step_ms)
    wg = np.interp(grid, t, w)
    if np.dot(wg - wg.mean(), np.interp(grid, t, g) - np.interp(grid, t, g).mean()) < 0:
        w = -w
        wg = -wg
    wg = (wg - wg.mean()) / (wg.std() or 1)
    best, best_c = None, -2.0
    for lag in np.arange(-max_lag_ms, max_lag_ms + step_ms / 2, step_ms):
        gg = np.interp(grid + lag, t, g)
        sd = gg.std()
        if sd == 0:
            continue
        c = float(np.mean(wg * (gg - gg.mean()) / sd))
        if c > best_c:
            best, best_c = float(lag), c
    return best if best_c > 0.5 else None


# ================================================================ camera inputs
class Camera:
    """Every camera source found, on the laptop clock."""

    def __init__(self, recordings, sessions, args):
        self.notes = []
        self.calib_rows = []
        self.recs = []
        for rec in recordings:
            cal = args.calib or next((p for p in (os.path.join(rec, "floor_calibration.json"),
                                                  os.path.join(os.path.dirname(os.path.normpath(rec)),
                                                               "floor_calibration.json")) if os.path.isfile(p)), None)
            det = args.detections if (args.detections and len(recordings) == 1) else next(
                (p for p in (os.path.join(rec, "detections.csv"), os.path.join(rec, "camera_detections.csv"))
                 if os.path.isfile(p)), None)
            blink = os.path.join(rec, "blink_detections.csv")
            cres = os.path.join(rec, "calibration_results.csv")
            self.recs.append(dict(path=rec, calib=cal, det=det, blink=blink if os.path.isfile(blink) else None,
                                  cres=cres if os.path.isfile(cres) else None, offset=None, sessions=sessions))
        self.args = args
        self._conv = {}
        self._det = {}
        for r in self.recs:
            if r["cres"]:
                self.calib_rows += fr.load_calib_results(r["cres"], 0.0)
        self.led, self.led_how = fr.led_offsets(self.calib_rows)

    def offset(self, rec):
        if rec["offset"] is not None:
            return rec["offset"]
        if self.args.video_offset is not None:
            rec["offset"] = self.args.video_offset
            return rec["offset"]
        if not os.path.isfile(os.path.join(rec["path"], "frames.csv")):
            return None
        for s in rec["sessions"]:
            try:
                vo = fr.video_offset(s, rec["path"])
            except Exception as e:      # no opencv, unreadable video, ...
                self.notes.append(f"{rec['path']}: flash search failed ({e})")
                vo = None
            if vo:
                rec["offset"] = vo[0]
                print(f"{rec['path']}: video clock = laptop clock {vo[0] * 1000:+.0f} ms (from {vo[2]} light flashes); "
                      f"add --video-offset {vo[0]:.4f} next time to skip the search")
                return rec["offset"]
        self.notes.append(f"{rec['path']}: no light flashes matched the robot logs, so its camera data can't be "
                          "put on the laptop clock (give --video-offset)")
        rec["offset"] = float("nan")
        return rec["offset"]

    def conv(self, path):
        if path not in self._conv:
            self._conv[path] = (fr.load_homography(path)[1], fr.load_parallax(path), fr.load_distortion(path))
        return self._conv[path]

    def calib_for(self, rec, run_id):
        """The calibration that scores this run (held-out one if floor_calibration_loro.csv lists it),
        and whether that calibration was made from this run (circular)."""
        if not rec["calib"]:
            return None, False
        loro = os.path.join(os.path.dirname(rec["calib"]), "floor_calibration_loro.csv")
        assign, _ = fr.assign_calibrations([run_id], rec["calib"], loro if os.path.isfile(loro) else None)
        p = assign.get(run_id) or rec["calib"]
        return p, bool(fr.in_sample({run_id: p}))

    def detections(self, rec, calib, offset):
        key = (rec["det"], calib, offset)
        if key not in self._det:
            Hi, par, dist = self.conv(calib)
            self._det[key] = fr.load_detections(rec["det"], Hi, offset, parallax=par, dist=dist)
        return self._det[key]

    def blinks(self, rec, calib, offset):
        key = ("blink", rec["blink"], calib, offset)
        if key not in self._det:
            Hi, par, dist = self.conv(calib)
            self._det[key] = fr.load_blink(rec["blink"], Hi, offset, parallax=par, dist=dist)
        return self._det[key]

    def window(self, kind, run_id, t0, t1, prefer=()):
        """Camera rows of `kind` ('det' or 'blink') between laptop times t0..t1, from the
        recording that covers them. Returns (rows, in_sample) or (None, False)."""
        for rec in sorted(self.recs, key=lambda r: r["path"] not in prefer):   # the run's own experiment's first
            if not rec[kind] or not rec["calib"]:
                continue
            off = self.offset(rec)
            if off is None or not math.isfinite(off):
                continue
            calib, ins = self.calib_for(rec, run_id)
            rows = self.detections(rec, calib, off) if kind == "det" else self.blinks(rec, calib, off)
            got = [d for d in rows if t0 <= d["t"] <= t1]
            if got:
                return got, ins
        return None, False

    def has(self, kind):
        return any(r[kind] for r in self.recs)


def near(rows, x, y, radius=20.0):
    return [d for d in rows if math.hypot(d["x"] - x, d["y"] - y) <= radius]


def median_xy(rows):
    return st.median(d["x"] for d in rows), st.median(d["y"] for d in rows)


# ================================================================ the table
class Table:
    def __init__(self):
        self.rows = []
        self.info = {}

    def define(self, cid, name, rule, means, if_fails):
        self.info[cid] = dict(name=name, rule=rule, means=means, if_fails=if_fails)

    def add(self, cid, subject, result, verdict, value=None, n=None):
        i = self.info[cid]
        self.rows.append(dict(id=cid, check=i["name"], subject=subject, result=result, pass_rule=i["rule"],
                              verdict=verdict, meaning=i["means"], if_fails=i["if_fails"],
                              value="" if value is None else (round(value, 4) if isinstance(value, float) else value),
                              n="" if n is None else n))

    def notyet(self, cid, subject, needs):
        self.add(cid, subject, f"not available yet (needs {needs})", NOTYET)


# ================================================================ checks
def check_spacing(tb, exps):
    tb.define("1", "Mark spacing", f"each robot's mean front-to-rear spacing within 0.1 cm of {MARK_SPACING_CM} cm "
              "and SD at most 0.15 cm (CHECK up to 0.3 cm off)",
              "The tape heading assumes the front and rear dots sit 9.5 cm apart on the robot's centre line; "
              "this checks that every robot's dots really are.",
              "Measure the robot (front face to rear face); if one robot differs, use its own length; if the "
              "spread is large, the dots are not being placed at the middle of the faces: re-practise marking.")
    by = defaultdict(list)
    for e in exps.values():
        for r in e.rows:
            if excluded(r):
                continue
            for sfx in ("0", ""):
                v = [num(r.get(f"{k}{sfx}_cm")) for k in ("xf", "yf", "xr", "yr")]
                if None not in v:
                    by[r.get("robot", "?")].append(math.hypot(v[0] - v[2], v[1] - v[3]))
    if not by:
        tb.notyet("1", "all robots", "tape marks on any sheet")
        return
    for robot in sorted(by):
        m, sd, n, h = t_int(by[robot])
        off = m - MARK_SPACING_CM
        verdict = PASS if abs(off) <= 0.1 and (n < 2 or sd <= 0.15) else (CHECK if abs(off) <= 0.3 else FAIL)
        tb.add("1", robot, f"spacing {f(m)} cm (SD {f(sd)}, {n} mark pairs), {f(off, 2, True)} cm from {MARK_SPACING_CM}",
               verdict, off, n)


def straight_ratios(e):
    """[(robot, (tape / wheel - 1) in %, run_id)] for straight runs."""
    out = []
    for t, run in e.trial_runs():
        if route_of(t.get("level")) != "straight" or not t.get("start_measured"):
            continue
        rid = (t.get("run_id") or "").strip()
        s = e.straight.get(rid)
        if s:
            wheel = math.hypot(num(s.get("odo_x_cm")) or 0.0, num(s.get("odo_y_cm")) or 0.0)
        elif run is not None and run.raw:
            last = run.raw[-1]
            wheel = math.hypot(num(last.get("x_cm")) or 0.0, num(last.get("y_cm")) or 0.0)
        else:
            continue
        tx = num(t.get("target_x_cm")) or 0.0
        if wheel < 10 or t.get("along_err") is None:
            continue
        tape = tx + t["along_err"]
        out.append((t["robot"], (tape / wheel - 1) * 100, rid))
    return out


def check_wheel_distance(tb, t1):
    tb.define("2", "Wheel distance scale", "each robot's tape-vs-wheel distance within 1% (whole 95% interval)",
              "On straight runs, compares how far the tape says the robot went with how far its wheels say it went.",
              "Multiply the wheel radius the fusion uses for that robot by (1 + the difference); a large spread "
              "instead means slipping or a loose wheel.")
    if t1 is None:
        tb.notyet("2", "all robots", "the T1 folder")
        return {}
    by = defaultdict(list)
    for robot, pct, _ in straight_ratios(t1):
        by[robot].append(pct)
    if not by:
        tb.notyet("2", "all robots", "measured straight runs with start marks and their console logs")
        return {}
    for robot in sorted(by):
        m, sd, n, h = t_int(by[robot])
        tb.add("2", robot, f"tape distance is {ci_text(m, h, 2, '%')} of the wheels' over {n} runs",
               within(m, h, 1.0), m, n)
    return by


def square_runs(e):
    """[(trial, sensor summary, true total turn)] for square runs with start marks and logs."""
    out = []
    for t, run in e.trial_runs():
        if route_of(t.get("level")) != "square" or run is None or not t.get("start_measured"):
            continue
        s = sensor_summary(run, e.runinfo.get(run.run_id))
        if s is None:
            continue
        tape_dh = wrap180(t["heading"] - t["start_heading"])
        true_total = s["wheel_total"] + wrap180(tape_dh - s["wheel_total"])
        if abs(true_total) < 180:
            continue
        out.append((t, s, true_total))
    return out


def all_turns(e):
    """{robot: [turn dicts]} from square and L runs."""
    by = defaultdict(list)
    for t, run in e.trial_runs():
        if route_of(t.get("level")) not in ("square", "L") or run is None:
            continue
        s = sensor_summary(run, e.runinfo.get(run.run_id))
        if s:
            by[t["robot"]] += s["turns"]
    return by


def check_turn_and_gyro_scale(tb, t1):
    tb.define("3", "Wheel turn scale",
              "each robot's wheel-measured turning within 1% of the tape (whole 95% interval) on the square's 1080 deg",
              "Compares how much the wheels say the robot turned in the 3-lap square with how much it really "
              "turned (tape), which tests the wheel track width the odometry assumes.",
              "Set the fusion's wheel track for that robot to (nominal track) x (1 + the difference); if wheels "
              "undercount turns by a few percent the wheels scrub in turns: trust the gyro for turning.")
    tb.define("4", "Gyro scale", "each robot's gyro within 0.5% of the tape (whole 95% interval) over the square's 1080 deg",
              "Compares the gyro's total turn over the 3-lap square (its at-rest bias taken out) with the real "
              "turn by tape.",
              "Give the fusion a per-robot gyro scale of 1 / (1 + the difference).")
    if t1 is None:
        tb.notyet("3", "all robots", "the T1 folder")
        tb.notyet("4", "all robots", "the T1 folder")
        return
    sq = square_runs(t1)
    if not sq:
        tb.notyet("3", "all robots", "measured square runs with start marks and console logs")
        tb.notyet("4", "all robots", "measured square runs with start marks and console logs")
        return
    turns = all_turns(t1)
    for cid, key, lim in (("3", "wheel_total", 1.0), ("4", "gyro_total", 0.5)):
        by = defaultdict(list)
        for t, s, true_total in sq:
            by[t["robot"]].append((s[key] / true_total - 1) * 100)
        for robot in sorted(by):
            m, sd, n, h = t_int(by[robot])
            extra = ""
            if cid == "3":
                tt = [x["wheel"] / x["gyro"] - 1 for x in turns.get(robot, []) if abs(x["gyro"]) > 30]
                if tt:
                    extra = f"; per turn (square and L, {len(tt)} turns) wheels/gyro {f(st.median(tt) * 100, 1, True)}%"
            what = "wheels say" if cid == "3" else "gyro says"
            tb.add(cid, robot, f"{what} {ci_text(m, h, 2, '%')} of the real turn, {n} square runs{extra}",
                   within(m, h, lim), m, n)


def check_bias_noise(tb, t1, low):
    tb.define("5", "Gyro bias/noise at rest",
              "bias steady between runs (SD of each run's bias at most 0.05 deg/s); at low battery, no detectable "
              "change (95% interval of the paired difference includes 0)",
              "When the wheels are still the gyro should read 0; its average reading is the bias (drift per "
              "second) and its jitter is the noise. Low vs full battery uses Right's E1b runs paired with its T1 runs.",
              "If the bias wanders between runs the fusion must re-learn it at every still moment (not once per "
              "power-on); if low battery changes it, keep batteries above the E1b level or model it.")
    if t1 is None:
        tb.notyet("5", "all robots", "the T1 folder")
        return
    per = defaultdict(list)
    for t, run in t1.trial_runs():
        if run is None:
            continue
        s = sensor_summary(run, t1.runinfo.get(run.run_id))
        if s and s["bias"] is not None:
            per[t["robot"]].append((s["bias"], s["noise"]))
    if not per:
        tb.notyet("5", "all robots", "console logs with still periods (RAW)")
    for robot in sorted(per):
        b = [x[0] for x in per[robot]]
        nz = [x[1] for x in per[robot] if x[1] is not None]
        m, sd, n, h = t_int(b)
        verdict = PASS if n >= 2 and sd <= 0.05 else (CHECK if n >= 2 else NOTYET)
        tb.add("5", robot, f"bias {f(m, 3, True)} deg/s (SD between runs {f(sd, 3)}), noise {f(st.mean(nz) if nz else None, 3)} "
               f"deg/s, {n} runs", verdict, m, n)
    if low is None:
        tb.notyet("5", "Right, low vs full", "the E1b folder")
        return
    pairs = pair_low_full(low, t1)
    diffs_b, diffs_n = [], []
    for lt, ft in pairs:
        ls = sensor_summary(lt[1], low.runinfo.get(lt[1].run_id)) if lt[1] else None
        fs = sensor_summary(ft[1], t1.runinfo.get(ft[1].run_id)) if ft[1] else None
        if ls and fs and ls["bias"] is not None and fs["bias"] is not None:
            diffs_b.append(ls["bias"] - fs["bias"])
            if ls["noise"] is not None and fs["noise"] is not None:
                diffs_n.append(ls["noise"] - fs["noise"])
    if len(diffs_b) < 2:
        tb.notyet("5", "Right, low vs full", "at least 2 E1b runs paired with Right's T1 runs, with console logs")
        return
    mb, hb, pb, nb = paired_t(diffs_b)
    mn, hn, pn, nn = paired_t(diffs_n)
    ok = all(math.isfinite(h) and abs(m) <= h for m, h in ((mb, hb), (mn, hn)))
    tb.add("5", "Right, low vs full", f"low minus full: bias {ci_text(mb, hb, 3, ' deg/s')}; noise "
           f"{ci_text(mn, hn, 3, ' deg/s')}; {nb} pairs", PASS if ok else CHECK, mb, nb)


def check_wheels_vs_gyro(tb, t1):
    tb.define("6", "Wheels vs gyro on turns",
              "the gyro's end-heading error no larger than the wheels' (paired, 95% interval)",
              "For each square run, how far each sensor's heading ends from the tape heading; the smaller one is "
              "the one to trust for turning.",
              "If the wheels are clearly better, the gyro is mis-scaled or biased: fix check 4 and 5 first, then "
              "re-weight the fusion towards the wheels for turns.")
    if t1 is None:
        tb.notyet("6", "all robots", "the T1 folder")
        return
    sq = square_runs(t1)
    if not sq:
        tb.notyet("6", "all robots", "measured square runs with start marks and console logs")
        return
    d = [abs(s["gyro_total"] - tt) - abs(s["wheel_total"] - tt) for t, s, tt in sq]
    gw = [abs(s["gyro_total"] - tt) for t, s, tt in sq]
    ww = [abs(s["wheel_total"] - tt) for t, s, tt in sq]
    m, h, p, n = paired_t(d)
    verdict = CHECK if (math.isfinite(h) and m - h > 0) else PASS
    better = "gyro" if st.mean(gw) < st.mean(ww) else "wheels"
    tb.add("6", "all robots", f"end heading off by {f(st.mean(gw), 1)} deg (gyro) vs {f(st.mean(ww), 1)} deg (wheels); "
           f"gyro minus wheels {ci_text(m, h, 1, ' deg')}, {n} square runs: trust the {better}", verdict, m, n)


def check_lag(tb, t1):
    tb.define("7", "Gyro vs wheel timing",
              "median delay of the gyro behind the wheels at turn starts within 20 ms (one RAW row)",
              "At the start of each turn, the wheels' turn rate and the gyro's should rise together; a delay "
              "would make the fusion mix readings from different moments.",
              "Shift the gyro by the measured delay in the fusion (or the wheels, if negative).")
    if t1 is None:
        tb.notyet("7", "all robots", "the T1 folder")
        return
    by = defaultdict(list)
    for t, run in t1.trial_runs():
        if run is None:
            continue
        s = sensor_summary(run, t1.runinfo.get(run.run_id))
        if s:
            by[t["robot"]] += [x["lag_ms"] for x in s["turns"] if x["lag_ms"] is not None]
    if not by:
        tb.notyet("7", "all robots", "turning runs (square or L) with console logs")
        return
    for robot in sorted(by):
        v = sorted(by[robot])
        med = st.median(v)
        q1, q3 = v[len(v) // 4], v[(3 * len(v)) // 4]
        tb.add("7", robot, f"gyro {f(med, 0, True)} ms behind the wheels (middle half {f(q1, 0, True)} to {f(q3, 0, True)} ms, "
               f"{len(v)} turn starts)", PASS if abs(med) <= 20 else CHECK, med, len(v))


def still_windows(e):
    """[(trial, run_id, which, [(t0, t1), ...], (x, y, heading))]: when a run's robot sits
    still on its tape marks: 'start' (before the route, around the start nudge) and
    'end' (after its last move, while the end marks are drawn)."""
    out = []
    for t, run in e.trial_runs():
        if run is None:
            continue
        s = sensor_summary(run, e.runinfo.get(run.run_id))
        if not s:
            continue
        if t.get("start_measured") and s["start_iv"]:
            out.append((t, run.run_id, "start", s["start_iv"], (t["start_x"], t["start_y"], t["start_heading"])))
        if s["end_iv"]:
            out.append((t, run.run_id, "end", [s["end_iv"]], (t["x"], t["y"], t["heading"])))
        elif s["travel_cm"] < s["nudge_travel_cm"] and s["start_iv"]:        # E0b: the start roll alone; still at the end too
            out.append((t, run.run_id, "end", [(s["start_iv"][-1][0], s["t_last"] + 0.5)], (t["x"], t["y"], t["heading"])))
    return out


def cam_rows(cam, kind, rid, ivs, prefer=()):
    """Camera rows over several time intervals; (rows, in_sample)."""
    got, ins = [], False
    for t0, t1 in ivs:
        rows, i = cam.window(kind, rid, t0, t1, prefer)
        if rows:
            got += rows
            ins |= i
    return got, ins


def check_camera(tb, exps, cam, out_dir):
    tb.define("8", "Camera vs tape (still)",
              "bias at most 1 cm and SD at most 1 cm per floor axis (CHECK up to 2.5 cm); written to measured_noise.json",
              "Where the camera puts a still robot compared with the tape marks: T0 placements, and each run's "
              "start and end. The spread is the camera's measured noise, which the fusion uses.",
              "Redo the floor calibration (more points, spread over the floor) or check the detector; a bias only "
              "in one area points at the calibration, everywhere at the camera height or lens term.")
    noise = dict(cam_xy_cm=None, cam_heading_deg=None, blink_xy_cm=None, start_nudge_heading_deg=None)
    detail = {}
    dxy, dh, bxy, ins_any = [], [], [], False
    # T1-type runs: start and end still moments
    if cam.has("det") or cam.has("blink"):
        for role in ("t1", "low", "nudge"):
            e = exps.get(role)
            if e is None:
                continue
            for t, rid, which, ivs, (x, y, h) in still_windows(e):
                rows, ins = cam_rows(cam, "det", rid, ivs, e.recordings) if cam.has("det") else (None, False)
                if rows:
                    rows = near(rows, x, y)
                if rows and len(rows) >= 3:
                    mx, my = median_xy(rows)
                    dxy.append((mx - x, my - y))
                    ins_any |= ins
                    hs = [d["heading"] for d in rows if d.get("heading") is not None]
                    if hs:
                        dh.append(wrap180(st.median(hs) - h))
                brows, _ = cam_rows(cam, "blink", rid, ivs, e.recordings) if cam.has("blink") else (None, False)
                if brows:
                    lf, ll = cam.led.get(t["robot"], cam.led[None])
                    c, s_ = math.cos(math.radians(h)), math.sin(math.radians(h))
                    lx, ly = x + c * lf - s_ * ll, y + s_ * lf + c * ll     # where the light should be
                    brows = near(brows, lx, ly, 15.0)
                    if len(brows) >= 2:
                        mx, my = median_xy(brows)
                        bxy.append((mx - lx, my - ly))
    # T0 placements: the robot points clicked in calibrate_floor (they give the frame and the tape position)
    t0_used = 0
    for rec in cam.recs:
        fp = os.path.join(os.path.dirname(rec["calib"]), "floor_points.json") if rec["calib"] else None
        if not (fp and os.path.isfile(fp) and rec["det"]):
            continue
        with open(fp) as fh:
            pts = [p for p in (json.load(fh).get("points") or []) if p.get("kind") == "robot" and p.get("enabled", True)]
        for p in pts:
            calib, ins = cam.calib_for(rec, str(p.get("group") or ""))
            rows = [d for d in cam.detections(rec, calib, 0.0) if abs(d["frame"] - int(p.get("frame", -99))) <= 5]
            rows = near(rows, p["x"], p["y"], 15.0)
            if rows:
                mx, my = median_xy(rows)
                dxy.append((mx - p["x"], my - p["y"]))
                ins_any |= ins
                t0_used += 1
    # start nudge heading (calibration_results.csv) vs tape start heading
    nud = []
    for role in ("t1", "low", "nudge"):
        e = exps.get(role)
        if e is None:
            continue
        sheet = {(r.get("run_id") or "").strip(): r for r in e.rows if not excluded(r)}
        for c in cam.calib_rows:
            if c["event"] == "nudge" and c["heading"] is not None and "disagree" not in c["identity"].lower():
                r = sheet.get(c["run"])
                th = tape_heading(r) if r else None
                if th is not None:
                    nud.append(wrap180(c["heading"] - th))
    any_cam = False
    if dxy:
        any_cam = True
        bx, by_ = st.mean(d[0] for d in dxy), st.mean(d[1] for d in dxy)
        sx = st.stdev([d[0] for d in dxy]) if len(dxy) > 1 else float("nan")
        sy = st.stdev([d[1] for d in dxy]) if len(dxy) > 1 else float("nan")
        sxy = math.sqrt((sx * sx + sy * sy) / 2) if math.isfinite(sx) else None
        noise["cam_xy_cm"] = None if sxy is None else round(sxy, 3)
        detail["cam_bias_cm"] = [round(bx, 3), round(by_, 3)]
        detail["cam_n"] = len(dxy)
        detail["cam_t0_placements"] = t0_used
        worst_b = max(abs(bx), abs(by_))
        verdict = (PASS if sxy is not None and worst_b <= 1 and sxy <= 1 else
                   CHECK if sxy is not None and worst_b <= 2.5 and sxy <= 2.5 else FAIL if sxy is not None else CHECK)
        note = " (calibration made from these same points: optimistic; make floor_calibration_loro.csv)" if ins_any else ""
        tb.add("8", "detector position", f"camera minus tape: bias x {f(bx, 2, True)}, y {f(by_, 2, True)} cm; "
               f"SD {f(sxy)} cm per axis; {len(dxy)} still moments ({t0_used} from T0){note}", verdict, sxy, len(dxy))
    if dh:
        any_cam = True
        m, sd, n, h = t_int(dh)
        noise["cam_heading_deg"] = round(sd, 3) if math.isfinite(sd) else None
        detail["cam_heading_bias_deg"] = round(m, 3)
        tb.add("8", "detector heading", f"camera minus tape heading {f(m, 1, True)} deg, SD {f(sd, 1)} deg, {n} moments",
               PASS if math.isfinite(sd) and sd <= 5 and abs(m) <= 3 else CHECK, sd, n)
    if bxy:
        any_cam = True
        bx, by_ = st.mean(d[0] for d in bxy), st.mean(d[1] for d in bxy)
        sx = st.stdev([d[0] for d in bxy]) if len(bxy) > 1 else float("nan")
        sy = st.stdev([d[1] for d in bxy]) if len(bxy) > 1 else float("nan")
        sxy = math.sqrt((sx * sx + sy * sy) / 2) if math.isfinite(sx) else None
        noise["blink_xy_cm"] = None if sxy is None else round(sxy, 3)
        detail["blink_bias_cm"] = [round(bx, 3), round(by_, 3)]
        detail["blink_n"] = len(bxy)
        worst_b = max(abs(bx), abs(by_))
        verdict = (PASS if sxy is not None and worst_b <= 1 and sxy <= 1 else
                   CHECK if sxy is not None and worst_b <= 2.5 and sxy <= 2.5 else FAIL if sxy is not None else CHECK)
        tb.add("8", "blink light position", f"light minus where the tape says it is: bias x {f(bx, 2, True)}, y "
               f"{f(by_, 2, True)} cm; SD {f(sxy)} cm per axis; {len(bxy)} still moments (light offset from {cam.led_how})",
               verdict, sxy, len(bxy))
    if nud:
        any_cam = True
        m, sd, n, h = t_int(nud)
        noise["start_nudge_heading_deg"] = round(sd, 3) if math.isfinite(sd) else None
        detail["start_nudge_heading_bias_deg"] = round(m, 3)
        tb.add("8", "start-nudge heading", f"camera minus tape start heading {f(m, 1, True)} deg, SD {f(sd, 1)} deg "
               f"(includes the tape's own heading error), {n} runs", PASS if math.isfinite(sd) and sd <= 3 else CHECK, sd, n)
    if not any_cam:
        if not cam.recs:
            tb.notyet("8", "camera", "a recording folder (data/video/<recording> or --recording)")
        else:
            needs = []
            if not any(r["calib"] for r in cam.recs):
                needs.append("floor_calibration.json")
            if not (cam.has("det") or cam.has("blink") or cam.has("cres")):
                needs.append("detections.csv, blink_detections.csv or calibration_results.csv in the recording")
            if not needs:
                needs.append("camera sightings at the runs' still moments (check the video clock offset)")
            tb.notyet("8", "camera", ", ".join(needs))
    noise.update(_made=dt.datetime.now().isoformat(timespec="seconds"), _detail=detail,
                 _note="SD per floor axis (cm) or of heading (deg) of camera minus tape while the robot is still; "
                       "the tape's own reading error (about 0.06 cm, T0) is not taken out. null = not measured yet.")
    path = os.path.join(out_dir, "measured_noise.json")
    if not any(noise[k] is not None for k in ("cam_xy_cm", "cam_heading_deg", "blink_xy_cm", "start_nudge_heading_deg")) \
            and os.path.isfile(path):
        return path + " (kept: nothing measured this time)"
    with open(path, "w") as fh:
        json.dump(noise, fh, indent=2)
    return path


def settled_shift(rows, t0, next_start=None, win=0.5, step_cm=0.3, min_windows=3):
    """How far a robot that stopped at time t0 was moved before it was carried to
    its next start (cm; 0 if it never moved). rows: camera positions near it.
    Half-second medians are grouped into still stretches (each step under
    step_cm, at least min_windows long); the first is where it stopped. A later
    still stretch is a bump, unless it is closer to the next run's tape start
    (next_start, x, y) than to the stop: that is the robot put down for the next
    run. Stops when the camera loses it for a second. None if too few sightings."""
    meds, k, missing = [], 0, 0
    t_end = max(d["t"] for d in rows)
    while t0 + k * win <= t_end:
        w = [d for d in rows if t0 + k * win <= d["t"] < t0 + (k + 1) * win]
        k += 1
        if len(w) < 2:
            missing += 1
            if missing >= 2 and meds:
                break
            meds.append(None)
            continue
        missing = 0
        meds.append(median_xy(w))
    dist = lambda p, q: math.hypot(p[0] - q[0], p[1] - q[1])
    stretches, cur = [], []
    for m in meds:
        if m is not None and cur and dist(m, cur[-1]) <= step_cm:
            cur.append(m)
            continue
        if len(cur) >= min_windows:
            stretches.append(cur)
        cur = [m] if m is not None else []
    if len(cur) >= min_windows:
        stretches.append(cur)
    if not stretches:
        return None
    centre = lambda sx: (st.median(p[0] for p in sx), st.median(p[1] for p in sx))
    ref = centre(stretches[0])
    worst = 0.0
    for sx in stretches[1:]:
        c = centre(sx)
        if next_start is not None and dist(c, next_start) < dist(c, ref):
            break                      # put down for the next run
        worst = max(worst, dist(c, ref))
    return worst


def check_end_moved(tb, exps, cam):
    tb.define("9", "Moved before end marks",
              "no run where the camera sees the robot shift more than 0.5 cm between stopping and being picked up",
              "After a run ends the robot must sit still until both end dots are drawn; a bump while marking "
              "would put the tape error on the robot.",
              "Leave those runs' end marks out (mark excluded with the reason) and mark more carefully: hold the "
              "square against the floor, not the robot.")
    if not cam.has("det"):
        tb.notyet("9", "all runs", "robot detections from the video (detections.csv in the recording)")
        return
    shifts, moved = [], []
    for role in ("t1", "low", "nudge"):
        e = exps.get(role)
        if e is None:
            continue
        nxt = {}                       # run_id -> the same robot's next measured trial
        by_robot = defaultdict(list)
        for t in e.trials:
            by_robot[t["robot"]].append(t)
        for ts in by_robot.values():
            ts.sort(key=lambda t: num(t.get("order")) or 0)
            for a, b in zip(ts, ts[1:]):
                nxt[(a.get("run_id") or "").strip()] = b
        for t, rid, which, ivs, (x, y, h) in still_windows(e):
            if which != "end":
                continue
            t0 = ivs[0][0]
            nt = nxt.get(rid)
            t_stop = t0 + 120.0
            nrun = e.runs.get((nt.get("run_id") or "").strip()) if nt else None
            if nrun is not None and nrun.t_start is not None:
                t_stop = min(t_stop, nrun.t_start)
            rows, _ = cam.window("det", rid, t0, t_stop, e.recordings)
            if not rows:
                continue
            rows = near(rows, x, y, 15.0)
            if len(rows) < 5:
                continue
            sh = settled_shift(rows, t0, (nt["start_x"], nt["start_y"]) if nt and nt.get("start_measured") else None)
            if sh is None:
                continue
            shifts.append(sh)
            if sh > 0.5:
                moved.append(f"{rid} ({sh:.1f} cm)")
    if not shifts:
        tb.notyet("9", "all runs", "camera detections after the runs' ends (check the video clock offset)")
        return
    tb.add("9", "all runs", f"{len(moved)} of {len(shifts)} runs shifted more than 0.5 cm"
           + (": " + ", ".join(moved[:8]) if moved else f" (largest {max(shifts):.2f} cm)"),
           PASS if not moved else CHECK, max(shifts), len(shifts))


def check_trends(tb, t1, low):
    tb.define("10", "Error vs order/battery",
              "no trend: the 95% interval of the slope includes 0 (each robot and route compared with itself)",
              "Checks whether end error creeps up as the session goes on (floor wear, warming up) or as the "
              "battery runs down.",
              "Add run order or battery as a factor in the analysis (or recharge / clean the floor more often), "
              "and keep the tuning and scoring halves balanced in time (data/split.csv already is).")
    if t1 is None or not t1.trials:
        tb.notyet("10", "run order", "measured T1 runs")
        tb.notyet("10", "battery", "measured T1 runs")
        return
    pts = [((t["robot"], t["level"]), num(t.get("order")), t.get("pos_err")) for t in t1.trials]
    r = within_group_slope(pts)
    if r:
        b, h, n = r
        tb.add("10", "run order", f"end error changes {ci_text(b * 10, h * 10, 2, ' cm')} per 10 runs, {n} runs",
               CHECK if abs(b) > h else PASS, b, n)
    else:
        tb.notyet("10", "run order", "at least 2 measured runs per robot and route")

    def batt(e, t):
        run = e.runs.get((t.get("run_id") or "").strip())
        if run is not None and run.battery_start is not None and math.isfinite(run.battery_start):
            return run.battery_start
        return num(t.get("battery_pct"))
    pts = [((t["robot"], t["level"]), batt(e, t), t.get("pos_err")) for e in (t1, low) if e for t in e.trials]
    r = within_group_slope(pts)
    if r:
        b, h, n = r
        tb.add("10", "battery", f"end error changes {ci_text(b * 10, h * 10, 2, ' cm')} per 10% of battery, {n} runs",
               CHECK if abs(b) > h else PASS, b, n)
    else:
        tb.notyet("10", "battery", "battery readings (console logs or battery_pct) that differ between runs")


def check_identity(tb, cam):
    tb.define("11", "Blink identity", "no run where the blink code names a different robot than the log",
              "Each robot blinks its own code; calib.py checks that the light that moved belongs to the robot whose "
              "log shows the move. A mix-up would give one robot's video to another's data.",
              "Check the robots' blink_code settings and firmware, and leave the disagreeing runs' camera data out.")
    rows = [c for c in cam.calib_rows if c["identity"]]
    if not rows:
        tb.notyet("11", "all runs", "calibration_results.csv from tools/blink/calib.py")
        return
    agree = sum(1 for c in rows if c["identity"].lower() == "agree")
    dis = sum(1 for c in rows if "disagree" in c["identity"].lower())
    # calibrations limited by lack of room (no roll to name): their own state, neither read nor unreadable
    limited = sum(1 for c in rows if (c.get("outcome") or "").startswith(cs.sc.CALIB_LIMITED)
                  and c["identity"].lower() != "agree"
                  and "disagree" not in c["identity"].lower())
    unread = len(rows) - agree - dis - limited
    lo, hi = wilson(dis, agree + dis)
    verdict = FAIL if dis else (CHECK if unread > 0.2 * (len(rows) - limited) else PASS)
    tb.add("11", "all runs", f"{dis} wrong of {agree + dis} named rolls (error rate 95% interval {lo * 100:.0f}% to "
           f"{hi * 100:.0f}%); {unread} not readable"
           + (f"; {limited} calibration(s) limited (no roll to name: no room or a plywood edge; not counted)"
              if limited else ""),
           verdict, dis / (agree + dis) if agree + dis else None, agree + dis)


def check_robots(tb, t1):
    tb.define("12", "Robot vs robot",
              "no pair of robots differs in end error (95% interval of the route-matched difference includes 0)",
              "Compares the robots' end errors route by route (Center has newer motor firmware, 1.1.1), so a "
              "difference is the robot, not the route mix.",
              "Analyse and tune that robot separately (its own wheel radius, track and gyro scale from checks 2-4).")
    if t1 is None or not t1.trials:
        tb.notyet("12", "robots", "measured T1 runs")
        return
    g = defaultdict(list)
    for t in t1.trials:
        if t.get("pos_err") is not None:
            g[(t["robot"], route_of(t["level"]))].append(t["pos_err"])
    robots = sorted({k[0] for k in g})
    if len(robots) < 2:
        tb.notyet("12", "robots", "measured T1 runs from at least 2 robots")
        return
    fw = {}
    for ri in t1.runinfo.values():
        fw[ri.get("robot", "")] = ri.get("motor_fw", "")
    means = "; ".join(f"{rb} {f(st.mean([v for (r, _), vs in g.items() if r == rb for v in vs]))} cm"
                      + (f" (motor {fw[rb]})" if fw.get(rb) else "") for rb in robots)
    any_diff = False
    for i, a in enumerate(robots):
        for b in robots[i + 1:]:
            routes = [rt for (r, rt) in g if r == a and (b, rt) in g and len(g[(a, rt)]) and len(g[(b, rt)])]
            if not routes:
                continue
            d = st.mean(st.mean(g[(a, rt)]) - st.mean(g[(b, rt)]) for rt in routes)
            var, dfs = 0.0, 0
            for rt in routes:
                for r in (a, b):
                    v = g[(r, rt)]
                    if len(v) > 1:
                        var += st.variance(v) / len(v)
                        dfs += len(v) - 1
            if dfs < 1:
                continue
            se = math.sqrt(var) / len(routes)
            h = t_quantile(0.975, dfs) * se
            sig = abs(d) > h
            any_diff |= sig
            tb.add("12", f"{a} minus {b}", f"{ci_text(d, h, 2, ' cm')} end error, matched over {len(routes)} routes",
                   CHECK if sig else PASS, d, dfs)
    tb.add("12", "means", means, CHECK if any_diff else PASS)


def pair_low_full(low, t1, robot="Right"):
    """E1b runs paired with the same robot's T1 runs: same route, same place in that
    route's order on its sheet (1st low square with 1st full square, ...).
    Returns [((low trial, Run), (full trial, Run))] for pairs measured on both sides."""
    def idx(e):
        out = {}
        rows = sorted((r for r in e.rows if r.get("robot") == robot), key=lambda r: num(r.get("order")) or 0)
        count = defaultdict(int)
        for r in rows:
            rt = route_of(r.get("level"))
            out[(r.get("order"), r.get("robot"))] = (rt, count[rt])
            count[rt] += 1
        return out
    li, fi = idx(low), idx(t1)
    lt = {li[(t["order"], t["robot"])]: (t, low.runs.get((t.get("run_id") or "").strip()))
          for t in low.trials if (t["order"], t["robot"]) in li}
    ft = {fi[(t["order"], t["robot"])]: (t, t1.runs.get((t.get("run_id") or "").strip()))
          for t in t1.trials if (t["order"], t["robot"]) in fi}
    return [(lt[k], ft[k]) for k in sorted(set(lt) & set(ft))]


def split_rows(rows):
    """Held-out split of T1: within each robot x route (sheet order), runs 1 and 4 of
    every four tune the fusion and runs 2 and 3 score it (A B B A, so both halves
    span the session evenly). Depends only on the sheet's order, never on results."""
    groups = defaultdict(list)
    for r in rows:
        groups[(r.get("robot", ""), route_of(r.get("level")))].append(r)
    out = []
    for key in groups:
        for k, r in enumerate(sorted(groups[key], key=lambda r: num(r.get("order")) or 0)):
            half = "tune" if k % 4 in (0, 3) else "score"
            out.append(dict(order=r.get("order", ""), robot=r.get("robot", ""), route=route_of(r.get("level")),
                            level=r.get("level", ""), run_id=(r.get("run_id") or "").strip(), half=half,
                            excluded="1" if excluded(r) else ""))
    out.sort(key=lambda d: num(d["order"]) or 0)
    return out


def write_split(t1):
    path = os.path.join(t1.folder, "data", "split.csv")
    rows = split_rows(t1.rows)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=["order", "robot", "route", "level", "run_id", "half", "excluded"])
        w.writeheader()
        w.writerows(rows)
    return path, rows


def check_stats(tb, t1, low):
    tb.define("13", "Statistics rules",
              "end-error intervals: t and bootstrap agree and the t-interval stays above 0; E1b: paired, the 95% "
              "interval of low minus full includes 0; split: every robot and route has runs in both halves",
              "End error is never negative and is skewed, so a bootstrap interval (resampling the runs) is given "
              "beside the usual t-interval; E1b is compared run-for-run with Right's T1 runs; fusion settings are "
              "tuned only on the tuning half of T1 (data/split.csv) and scored on the other half.",
              "Report the bootstrap interval when the two disagree; if battery matters, report low-battery runs "
              "separately; never score the fusion on runs it was tuned on.")
    if t1 is None or not t1.rows:
        tb.notyet("13", "end error intervals", "the T1 sheet")
        tb.notyet("13", "held-out split", "the T1 sheet")
    else:
        by = defaultdict(list)
        for t in t1.trials:
            if t.get("pos_err") is not None:
                by[route_of(t["level"])].append(t["pos_err"])
        if not by:
            tb.notyet("13", "end error intervals", "measured T1 runs")
        for rt in sorted(by):
            m, sd, n, h = t_int(by[rt])
            lo, hi = tr.bootstrap_ci(by[rt])
            verdict = PASS if n >= 2 and math.isfinite(h) and m - h > 0 else CHECK
            if n < 2:
                tb.add("13", f"end error {rt}", f"mean {f(m)} cm from 1 run: too few for an interval", CHECK, m, n)
                continue
            tb.add("13", f"end error {rt}", f"mean {f(m)} cm; t-interval {f(m - h)} to {f(m + h)}; bootstrap {f(lo)} "
                   f"to {f(hi)} cm; {n} runs", verdict, m, n)
        path, rows = write_split(t1)
        groups = defaultdict(set)
        for r in rows:
            groups[(r["robot"], r["route"])].add(r["half"])
        bad = [f"{a}/{b}" for (a, b), hs in groups.items() if hs != {"tune", "score"}]
        tb.add("13", "held-out split", f"{sum(r['half'] == 'tune' for r in rows)} tuning and "
               f"{sum(r['half'] == 'score' for r in rows)} scoring rows written to {os.path.relpath(path)}"
               + (f"; only one half: {', '.join(bad)}" if bad else ""), CHECK if bad else PASS, n=len(rows))
    if low is None or t1 is None:
        tb.notyet("13", "E1b vs T1 (Right)", "the E1b and T1 folders")
        return
    pairs = pair_low_full(low, t1)
    d = [lt[0]["pos_err"] - ft[0]["pos_err"] for lt, ft in pairs
         if lt[0].get("pos_err") is not None and ft[0].get("pos_err") is not None]
    if len(d) < 2:
        tb.notyet("13", "E1b vs T1 (Right)", "at least 2 measured E1b runs with a matching Right T1 run")
        return
    m, h, p, n = paired_t(d)
    tb.add("13", "E1b vs T1 (Right)", f"low minus full end error {ci_text(m, h, 2, ' cm')}, p = {f(p, 3)}, "
           f"{n} route-matched pairs", PASS if abs(m) <= h else CHECK, m, n)


def check_still_test(tb, t1):
    """15: the fusion's still test (fusion/engine.py StillTest: no wheel-angle change over 0.25 s) on the
    T1 runs: of the rows in long still stretches (wheel angles unchanged for 0.5 s or more, after 0.2 s of
    settling), how many it calls still; of the rows where a wheel turns (more than 2 deg per row), how many it
    wrongly calls still."""
    tb.define("15", "Fusion's still test at rest",
              "at least 70% of the rows in still stretches called still, and at most 1% of the rows with the wheels "
              "turning",
              "The fusion tells the filter 'speed and turn are exactly 0' whenever it thinks the robot stands still, "
              "which also pins down the gyro's bias. Too strict and it rarely helps (the old wheel-speed thresholds "
              "passed only ~15% of still rows in simulation); too loose and it stops a moving robot.",
              "Too few still rows: raise still_deg (engine.py, 0.75 deg) if the encoders jitter at rest. Moving rows "
              "called still: lower still_deg or lengthen still_window_s (0.25 s).")
    if t1 is None:
        tb.notyet("15", "all robots", "the T1 folder")
        return
    import engine as eng
    cfg = eng.config()
    per = defaultdict(lambda: [0, 0, 0, 0])            # still rows, called still; moving rows, called still
    for t, run in t1.trial_runs():
        if run is None or not run.raw:
            continue
        rows = sorted(run.raw, key=lambda r: r["t"])
        wl = [num(r.get("wl_deg")) for r in rows]
        wr = [num(r.get("wr_deg")) for r in rows]
        if any(v is None for v in wl + wr):
            continue
        test = eng.StillTest(cfg)
        called = [test.step(r, eng.fnum(eng.RU.speed_cms(r.get("v_cms")))) for r in rows]   # true cm/s
        quiet = [False] + [abs(wl[i] - wl[i - 1]) < 0.5 and abs(wr[i] - wr[i - 1]) < 0.5 for i in range(1, len(rows))]
        i = 0
        acc = per[t["robot"]]
        while i < len(rows):
            if not quiet[i]:
                if (i > 0 and (num(rows[i].get("lifted")) or 0.0) < 0.5
                        and max(abs(wl[i] - wl[i - 1]), abs(wr[i] - wr[i - 1])) > 2.0):
                    acc[2] += 1
                    acc[3] += called[i]
                i += 1
                continue
            j = i
            while j < len(rows) and quiet[j]:
                j += 1
            seg = [k for k in range(i, j) if rows[k]["t"] >= rows[i]["t"] + 0.2]
            if seg and rows[seg[-1]]["t"] - rows[seg[0]]["t"] >= 0.5:
                acc[0] += len(seg)
                acc[1] += sum(called[k] for k in seg)
            i = j
    if not per or not any(a[0] for a in per.values()):
        tb.notyet("15", "all robots", "console logs with wheel angles (RAW wl_deg, wr_deg) and still periods")
        return
    for robot in sorted(per):
        s, cs_, m, cm = per[robot]
        if not s or not m:
            continue
        hit, false = 100 * cs_ / s, 100 * cm / m
        tb.add("15", robot, f"{hit:.0f}% of {s} still rows called still; {false:.2f}% of {m} moving rows called still",
               PASS if hit >= 70 and false <= 1.0 else CHECK, hit, s)


def check_sim_preset(tb, cam_recs):
    tb.define("14", "Sim matches real camera",
              "tools/sim/configs/real_oct18.json loads in the simulator, and its frame rate and picture size match "
              "the real recording's meta.json (when one is given)",
              "Future simulations use the real camera's height, frame rate, picture size and pixel scale, so their "
              "results carry over.",
              "Edit tools/sim/configs/real_oct18.json to the recording's numbers.")
    if not os.path.isfile(SIM_PRESET):
        tb.notyet("14", "preset", SIM_PRESET)
        return
    try:
        sys.path.insert(0, os.path.join(HERE, "sim"))
        import config as simcfg
        cfg = simcfg.load_config(SIM_PRESET)
    except Exception as e:
        tb.add("14", "preset", f"does not load: {e}", FAIL)
        return
    c = cfg.camera
    px_cm = c.width / c.floor_width_cm
    res = f"{c.width}x{c.height} at {c.fps:g} fps, {c.height_cm:g} cm up, {px_cm:.2f} px/cm"
    mism = []
    for rec in cam_recs:
        mp = os.path.join(rec, "meta.json")
        if os.path.isfile(mp):
            with open(mp) as fh:
                m = json.load(fh)
            fps = num(m.get("fps_reported")) or num(m.get("fps_requested"))
            if fps and abs(fps - c.fps) > 1:
                mism.append(f"{os.path.basename(rec)} records at {fps:g} fps")
            if m.get("width") and (int(m["width"]), int(m.get("height") or 0)) != (c.width, c.height):
                mism.append(f"{os.path.basename(rec)} is {m['width']}x{m.get('height')}")
    tb.add("14", "preset", res + ("; " + "; ".join(mism) if mism else ""), CHECK if mism else PASS)


# ================================================================ output
FINE_PRINT = [
    "SD: standard deviation, the typical distance of a value from the mean.",
    "95% interval: the range the true value very likely lies in, given these runs (t-interval, from the t "
    "distribution).",
    "Bootstrap interval: the same, found by re-drawing the runs at random many times; it suits values that are "
    "skewed or can't be negative, like end error.",
    "Paired: each low-battery run is compared with its own matching full-battery run, which removes route differences.",
    "Slope: how much the error changes per step of run order or battery (least squares).",
    "Error-rate interval: Wilson's 95% interval, which also works when no errors were seen.",
]


def print_table(tb, width=110):
    print(f"\n{'#':>3}  {'check':<24} {'for':<20} {'verdict':<8} result")
    print("-" * width)
    for r in tb.rows:
        head = f"{r['id']:>3}  {r['check'][:24]:<24} {r['subject'][:20]:<20} {r['verdict']:<8} "
        lines = textwrap.wrap(r["result"], width - len(head)) or [""]
        print(head + lines[0])
        for ln in lines[1:]:
            print(" " * len(head) + ln)
    print("\nWhat each check means, its pass rule, and what to do if it fails")
    for cid, i in tb.info.items():
        print(f"\n{cid:>3}. {i['name']}")
        for label, key in (("means", "means"), ("pass if", "rule"), ("if not", "if_fails")):
            for k, ln in enumerate(textwrap.wrap(i[key], width - 16)):
                print(f"     {label if k == 0 else '':<9} {ln}")
    print()
    for ln in FINE_PRINT:
        print(textwrap.fill(ln, width))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folders", nargs="*", help="experiment folders (recognised by name)")
    for role, flag in (("t1", "--t1"), ("low", "--low-battery"), ("nudge", "--nudge"), ("t0", "--t0")):
        ap.add_argument(flag, dest=role, help=f"experiment folder to use as {role}")
    ap.add_argument("--recording", nargs="*", default=[], help="camera recording folder(s)")
    ap.add_argument("--detections", help="robot detections CSV (when there is one recording)")
    ap.add_argument("--calib", help="floor_calibration.json (default: in or beside each recording)")
    ap.add_argument("--video-offset", type=float, help="video clock minus laptop clock, s (skips the flash search)")
    ap.add_argument("--out", help="where to write accuracy_checks.csv and measured_noise.json (default: T1's data folder)")
    a = ap.parse_args(argv)

    folders = {}
    for fo in a.folders:
        role = role_of(fo)
        if role is None:
            print(f"note: can't tell what {fo} is from its name; give it as --t1 / --low-battery / --nudge / --t0")
            continue
        folders[role] = fo
    for role in ("t1", "low", "nudge", "t0"):
        if getattr(a, role):
            folders[role] = getattr(a, role)
    if not folders:
        ap.error("give at least one experiment folder")
    exps = {role: Experiment(fo, role) for role, fo in folders.items()}
    t1, low = exps.get("t1"), exps.get("low")
    recs = list(dict.fromkeys([r for e in exps.values() for r in e.recordings] + list(a.recording)))
    sessions = [s for e in exps.values() for s in e.sessions]
    cam = Camera(recs, sessions, a)
    out_dir = a.out or os.path.join((t1 or next(iter(exps.values()))).folder, "data")
    os.makedirs(out_dir, exist_ok=True)

    print("Inputs: " + "; ".join(f"{role}: {e.folder} ({len(e.trials)} measured runs, {len(e.sessions)} log folders, "
                                 f"{len(e.runs)} logged runs"
                                 + (f"; {len(e.limited)} limited (plywood edge) left out" if e.limited else "") + ")"
                                 for role, e in exps.items()))
    n_lim = sum(len(e.limited) for e in exps.values())
    if n_lim:
        print(f"note: {n_lim} run(s) limited (plywood edge): the edge guard stopped them before an open edge. Not a "
              "failure, but not a finished route, so every check below leaves them out.")
    print(f"Recordings: {', '.join(recs) if recs else 'none'}")

    tb = Table()
    check_spacing(tb, exps)
    check_wheel_distance(tb, t1)
    check_turn_and_gyro_scale(tb, t1)
    check_bias_noise(tb, t1, low)
    check_wheels_vs_gyro(tb, t1)
    check_lag(tb, t1)
    noise_path = check_camera(tb, exps, cam, out_dir)
    check_end_moved(tb, exps, cam)
    check_trends(tb, t1, low)
    check_identity(tb, cam)
    check_robots(tb, t1)
    check_stats(tb, t1, low)
    check_sim_preset(tb, recs)
    check_still_test(tb, t1)
    for n in cam.notes:
        print("note: " + n)
    print_table(tb)
    path = os.path.join(out_dir, "accuracy_checks.csv")
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=["id", "check", "subject", "result", "pass_rule", "verdict", "meaning",
                                           "if_fails", "value", "n"])
        w.writeheader()
        w.writerows(tb.rows)
    print(f"\nwrote {path} and {noise_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
