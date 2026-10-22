#!/usr/bin/env python3
"""E02 analysis (lane A): how far do the Alviks end from where they meant to be, per condition.

After a session (the one command; docs/experiments/E02-accuracy/SESSION-GUIDE.md "After the session"):

    python tools/experiment/e02_analysis.py --session logs/e02-<date-time> --recording <the Pi's recording folder>

  reads everything from the runner's session folder (session_inputs()) and writes <session>/analysis/. A dry run
  recorded with E02_SIM_RECORD=1 needs no --recording (its sim/recording is found). Or, input by input:

    python tools/experiment/e02_analysis.py --recording ~/recordings/<overhead> --logs logs/<session> \\
        --runs e02_runs.csv --calib cameras/default.json --out docs/experiments/E02/results

Inputs (built to the contract in docs/build-plans/accuracy-experiment-E02.md; every column name the
readers accept is in ALIASES below, so a renamed column is a one-line change):
  --recording   overhead camera folder: video.avi, frames.csv (frame,t_wall), events.csv (marks),
                video_clock.json if present. Row i of frames.csv is frame i of the video.
  --logs        console session folder(s): bump.csv (BUMP), find.csv (FIND), e02.csv (E02),
                clock_sync.csv (robot clock). More than one folder allowed.
  --runs        the runner's run list: one row per run with run_id, robot, condition (U/C/K/KC), route
                (1/2/3 or straight/L/square), block, robot_run (the robot's own run number, joins the
                records), recovery (the runner's note that the run needed recovery: unseen, off_board,
                safety_stop, far_off; empty = no; an old `lost` column is read as recovery "lost"; it is a NOTE,
                never an outlier flag) and t_find, t_start, t_far, t_end
                (camera clock). Missing times are read from marks in events.csv, e.g. "E02 run 7 start".
                Without --runs, runs are built from the marks alone (condition etc. then blank).
  --calib       camera model json (tools/recognition/cameras.py) or floor_calibration.json.
  --background  optional picture of the empty board (else the robot is found against the floor next
                to it: tools/experiment/e02_analysis.py measure_pose()).

Outputs (in --out): runs.csv (one row per run), tracks/<run_id>.csv (the robot's measured pose in every
frame of the run), bumps.csv, summary.csv (mean, SD, CI per condition x route x robot, with and without
outliers), comparisons.csv (Welch, Mann-Whitney, Brown-Forsythe, effect sizes), charts/*.png, report.md; with --session
also front_sensor.csv (the front sensor after each leg against the map, from the runner's front_checks.csv) and
recovery.csv (auto-retries, sweeps, rescue finds, drive-outs, help calls per robot): setup_recovery().

What is measured, in plain words
  * Pose = centre of the robot's blob in the picture (pixels), turned into floor cm by the calibration
    (robot top 4 cm above the floor); heading = the rectangle's sides, resolved to the side nearest the
    hint (previous frame, or the believed heading), so it is right while the hint is within 45 deg.
  * Start = the robot standing still after find; far = the still stretch (1 s pause) away from the start;
    end = the last still stretch. Each is the median pose over that stretch.
  * Intended: far = start pose + the route's offset (ROUTES), end = start pose. So "error" is measured
    from where the camera saw it start (what the drive did), and abs_* columns from the believed start
    (what find + drive did together).
  * Believed (E02 record) vs camera is compared as displacement from the start, in the start's own frame,
    so it needs no agreement about axes between robot and floor.
  * Return to start in pixels: start vs end frame, robot centre and angle in the picture, no calibration.
  * Bump: the contact frame is the first frame where the robot's progress along its heading reaches the
    plateau it then stays on (it stopped advancing); its pose is compared with the BUMP record's believed
    pose (assumed in the robot's own frame, origin = where it was put down: --bump-frame own).
"""
import argparse
import csv
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (os.path.join(TOOLS, "recognition"), TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

CONDITIONS = ("U", "C", "K", "KC")
ROBOT_W_CM, ROBOT_L_CM = 9.5, 10.5        # body size, only used to size the search window
TOP_CM = 3.7                              # height of the robot's top above the floor (tools/fusion ALVIK_TOP_CM)
# route offsets in the start frame at the far point (dx ahead, |dy| to the side, |dheading|), as in
# tools/experiment/e02_plan.py route_local(): straight 100; L (30, then 30 sideways); square side 50, far point at
# its second corner. The side (left or right, "towards the board's middle") differs per run: it is taken from where
# the camera saw the far point (sign of its sideways offset), or `turn_sign` in the run list, or far_goal_* in E02.
ROUTES = {"straight": (100.0, 0.0, 0.0), "L": (30.0, 30.0, 90.0), "square": (50.0, 50.0, 90.0)}
ROUTE_NAMES = {"1": "straight", "straight": "straight", "straight100": "straight", "e01": "straight",
               "2": "L", "l": "L", "l30": "L", "l30-30": "L",
               "3": "square", "square": "square", "sq": "square"}

ALIASES = dict(
    run_id=["run_id", "id", "order", "n"],
    robot=["robot", "name"],
    condition=["condition", "cond"],
    route=["route", "e02_route"],
    block=["block"],
    turn_sign=["turn_sign"],
    robot_run=["robot_run", "log_run", "run_no", "e02_run"],
    find_run=["find_run"],
    order=["order"],
    status=["status"],
    recovery=["recovery", "needed_recovery", "lost", "flag_lost"],
    t_find=["t_find", "t_find_start", "find_t0"],
    t_start=["t_start", "t_route_start", "t_route"],
    t_far=["t_far"],
    t_end=["t_end", "t_route_end"],
)
RECORD_RUN = ["run", "robot_run"]
MARK_PHASES = {"find": "t_find", "start": "t_start", "route": "t_start", "far": "t_far", "end": "t_end"}


# =============================================================================== small maths
def wrap180(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


def compose(pose, rel):
    """Pose (x, y, th_deg) followed by a displacement in its own frame (dx ahead, dy left, dth)."""
    x, y, th = pose
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
    return (x + rel[0] * c - rel[1] * s, y + rel[0] * s + rel[1] * c, th + rel[2])


def relative(pose, origin):
    """pose expressed in the frame of origin: (ahead, left, dth)."""
    dx, dy = pose[0] - origin[0], pose[1] - origin[1]
    c, s = math.cos(math.radians(origin[2])), math.sin(math.radians(origin[2]))
    return (dx * c + dy * s, -dx * s + dy * c, float(wrap180(pose[2] - origin[2])))


def circ_median(angles_deg):
    a = np.asarray([x for x in angles_deg if np.isfinite(x)], float)
    if not len(a):
        return float("nan")
    ref = math.degrees(math.atan2(np.mean(np.sin(np.radians(a))), np.mean(np.cos(np.radians(a)))))
    return float(ref + np.median(wrap180(a - ref)))


def quarter_median(angles_deg):
    """Median of headings known only up to a quarter turn (a near-square body): each is brought within 45 deg of
    their four-fold circular mean before the median, so frames resolved to different sides do not mix; the result
    lies within 45 deg of the angle most frames had (the caller resolves the quarter)."""
    a = np.asarray([x for x in angles_deg if np.isfinite(x)], float)
    if not len(a):
        return float("nan")
    ref = math.degrees(math.atan2(np.mean(np.sin(np.radians(4 * a))), np.mean(np.cos(np.radians(4 * a))))) / 4.0
    near = ref + (a - ref + 45.0) % 90.0 - 45.0                      # each within 45 deg of ref
    med = float(np.median(near))
    # keep the quarter most frames were resolved to
    shift = 90.0 * np.round(wrap180(a - near) / 90.0)
    vals, counts = np.unique(np.round(shift).astype(int) % 360, return_counts=True)
    return float(wrap180(med + vals[np.argmax(counts)]))


def nanf(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return float("nan")
    return f


# =============================================================================== statistics
def describe(x, boot=4000, seed=1):
    """n, mean, SD, median, t-based 95% CI of the mean, bootstrap 95% CI of the SD."""
    from scipy import stats
    x = np.asarray([v for v in x if v is not None and np.isfinite(v)], float)
    n = len(x)
    out = dict(n=n, mean=float("nan"), sd=float("nan"), median=float("nan"), ci_lo=float("nan"),
               ci_hi=float("nan"), sd_lo=float("nan"), sd_hi=float("nan"))
    if n == 0:
        return out
    out["mean"], out["median"] = float(x.mean()), float(np.median(x))
    if n >= 2:
        sd = float(x.std(ddof=1))
        out["sd"] = sd
        half = float(stats.t.ppf(0.975, n - 1)) * sd / math.sqrt(n)
        out["ci_lo"], out["ci_hi"] = out["mean"] - half, out["mean"] + half
    if n >= 5 and boot > 0:       # fewer runs: re-drawing them gives no useful range for the SD
        rng = np.random.default_rng(seed)
        idx = rng.integers(0, n, size=(boot, n))
        sds = x[idx].std(axis=1, ddof=1)
        out["sd_lo"], out["sd_hi"] = (float(v) for v in np.percentile(sds, [2.5, 97.5]))
    return out


def compare(a, b):
    """Welch's t-test, Mann-Whitney U and Brown-Forsythe between two samples, with effect sizes.
    Positive differences / effect sizes mean a is larger than b."""
    from scipy import stats
    a = np.asarray([v for v in a if np.isfinite(v)], float)
    b = np.asarray([v for v in b if np.isfinite(v)], float)
    nan = float("nan")
    r = dict(n_a=len(a), n_b=len(b), mean_diff=nan, diff_lo=nan, diff_hi=nan, welch_t=nan, welch_df=nan,
             welch_p=nan, cohen_d=nan, mwu_U=nan, mwu_p=nan, rank_biserial=nan, bf_W=nan, bf_p=nan)
    if len(a) < 2 or len(b) < 2:
        return r
    ma, mb = a.mean(), b.mean()
    va, vb = a.var(ddof=1), b.var(ddof=1)
    r["mean_diff"] = float(ma - mb)
    se2 = va / len(a) + vb / len(b)
    if se2 > 0:
        df = se2 ** 2 / ((va / len(a)) ** 2 / (len(a) - 1) + (vb / len(b)) ** 2 / (len(b) - 1))
        t = (ma - mb) / math.sqrt(se2)
        h = float(stats.t.ppf(0.975, df)) * math.sqrt(se2)
        r.update(welch_t=float(t), welch_df=float(df), welch_p=float(2 * stats.t.sf(abs(t), df)),
                 diff_lo=float(ma - mb - h), diff_hi=float(ma - mb + h))
    sp = math.sqrt(((len(a) - 1) * va + (len(b) - 1) * vb) / (len(a) + len(b) - 2))
    if sp > 0:
        r["cohen_d"] = float((ma - mb) / sp)
    try:
        u = stats.mannwhitneyu(a, b, alternative="two-sided")
        r["mwu_U"], r["mwu_p"] = float(u.statistic), float(u.pvalue)
        r["rank_biserial"] = float(2 * u.statistic / (len(a) * len(b)) - 1)
    except ValueError:
        pass
    try:
        lv = stats.levene(a, b, center="median")
        r["bf_W"], r["bf_p"] = float(lv.statistic), float(lv.pvalue)
    except ValueError:
        pass
    return r


def strat_perm(a, sa, b, sb, n_perm=4000, seed=1):
    """Permutation test of condition A vs B that respects the design: the condition labels are shuffled only
    among runs of the same stratum (robot x route), so differences between robots and routes cannot make a
    condition look better. Statistic: the mean over strata holding both conditions of (mean A - mean B).
    Returns (difference, two-sided p, strata used); nan when no stratum holds both."""
    a = [(v, k) for v, k in zip(a, sa) if np.isfinite(v)]
    b = [(v, k) for v, k in zip(b, sb) if np.isfinite(v)]
    keys = sorted({k for _, k in a} & {k for _, k in b}, key=str)
    if not keys:
        return float("nan"), float("nan"), 0
    groups = []
    for k in keys:
        va = np.array([v for v, kk in a if kk == k])
        vb = np.array([v for v, kk in b if kk == k])
        groups.append((np.concatenate([va, vb]), len(va)))

    obs = float(np.mean([g[:na].mean() - g[na:].mean() for g, na in groups]))
    rng = np.random.default_rng(seed)
    tot = np.zeros(n_perm)
    for g, na in groups:
        idx = np.argsort(rng.random((n_perm, len(g))), axis=1)        # one shuffle of this stratum per row
        sh = g[idx]
        tot += sh[:, :na].mean(axis=1) - sh[:, na:].mean(axis=1)
    sim = tot / len(groups)
    hits = int(np.sum(np.abs(sim) >= abs(obs) - 1e-12))
    return obs, (hits + 1) / (n_perm + 1), len(keys)


def holm(ps):
    """Holm-adjusted p values (same order); nan stays nan."""
    ps = list(ps)
    idx = [i for i, p in enumerate(ps) if np.isfinite(p)]
    order = sorted(idx, key=lambda i: ps[i])
    out = [float("nan")] * len(ps)
    run = 0.0
    for k, i in enumerate(order):
        run = max(run, min(1.0, (len(order) - k) * ps[i]))
        out[i] = run
    return out


def iqr_outliers(values, k=1.5, min_n=4):
    """Boolean list: beyond Q1 - k*IQR / Q3 + k*IQR (Tukey fences). Fewer than min_n values: none."""
    v = np.asarray(values, float)
    ok = np.isfinite(v)
    flags = np.zeros(len(v), bool)
    if ok.sum() < min_n:
        return flags.tolist()
    q1, q3 = np.percentile(v[ok], [25, 75])
    lo, hi = q1 - k * (q3 - q1), q3 + k * (q3 - q1)
    flags[ok] = (v[ok] < lo) | (v[ok] > hi)
    return flags.tolist()


# =============================================================================== csv readers
def read_csv(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def pick(row, names, default=None):
    low = {k.strip().lower(): v for k, v in row.items() if k}
    for n in names:
        v = low.get(n)
        if v not in (None, ""):
            return v
    return default


def canon_route(v):
    if v in (None, ""):
        return ""
    return ROUTE_NAMES.get(str(v).strip().lower(), str(v).strip())


def canon_cond(v):
    s = str(v or "").strip().upper()
    return s


def stage_pose(row, stage):
    """(x, y, th) of stage start/far/end from an E02 record row, or None."""
    if row is None:
        return None
    vals = []
    for ax in ("x", "y", "th"):
        names = []
        for ax_n in ((ax, "theta", "heading") if ax == "th" else (ax,)):
            names += ["%s_%s_cm" % (stage, ax_n), "%s_%s_deg" % (stage, ax_n), "%s_%s" % (stage, ax_n),
                      "%s_%s_cm" % (ax_n, stage), "%s_%s_deg" % (ax_n, stage), "%s_%s" % (ax_n, stage)]
        vals.append(nanf(pick(row, names)))
    return tuple(vals) if all(np.isfinite(vals)) else None


class Logs:
    """bump.csv / find.csv / e02.csv of one or more console session folders, joined by (robot, run)."""

    def __init__(self, folders):
        self.bump, self.find, self.e02, self.clock = [], [], [], []
        for fo in folders or []:
            for attr, names in (("bump", ("bump.csv",)), ("find", ("find.csv",)), ("e02", ("e02.csv",)),
                                ("clock", ("clock_sync.csv",))):
                for n in names:
                    p = os.path.join(fo, n)
                    if os.path.exists(p):
                        getattr(self, attr).extend(read_csv(p))

    @staticmethod
    def _key(row):
        return (str(pick(row, ["robot"], "")), str(pick(row, RECORD_RUN, "")))

    def _rows(self, which, robot, run):
        return [r for r in getattr(self, which) if self._key(r) == (str(robot), str(run))]

    def e02_row(self, robot, run):
        r = self._rows("e02", robot, run)
        return r[-1] if r else None

    def find_row(self, robot, run):
        r = self._rows("find", robot, run)
        return r[-1] if r else None

    def bump_rows(self, robot, run):
        rows = self._rows("bump", robot, run)
        return sorted(rows, key=lambda r: nanf(pick(r, ["ms", "t_recv"], 0)))

    def robot_offset(self, robot, boot=None):
        """laptop time = robot_ms / 1000 + offset (median of the lowest-round-trip clock checks), or None. boot: the
        robot's boot id (the run id's prefix, e.g. 7ABD of 7ABD-4): a robot switched off and on restarts its clock."""
        rows = [r for r in self.clock if str(pick(r, ["robot"], "")) == str(robot)]
        if boot and any(str(pick(r, ["boot"], "")) == str(boot) for r in rows):
            rows = [r for r in rows if str(pick(r, ["boot"], "")) == str(boot)]
        pts = [(nanf(pick(r, ["rtt_ms"], 1e9)), nanf(pick(r, ["offset_s"]))) for r in rows]
        pts = sorted(p for p in pts if np.isfinite(p[1]))[:5]
        return float(np.median([p[1] for p in pts])) if pts else None


def video_clock(rec_folder):
    p = os.path.join(rec_folder, "video_clock.json") if rec_folder else None
    if p and os.path.exists(p):
        with open(p) as f:
            return json.load(f)
    return None


def laptop_to_video(t, vc):
    if not vc:
        return t
    off = vc.get("offset_s", 0.0)
    if vc.get("use_drift") and vc.get("t_ref") is not None:
        off += vc.get("drift_ppm", 0.0) * 1e-6 * (t - vc["t_ref"])
    return t + off


RUNNER_FIND = re.compile(r"^\s*e02\s+(\d+)\s+(\S+)\s+find\s+(start|end)\b", re.I)
RUNNER_TAG = re.compile(r"^\s*e02\s+(\d+)/\d+\s+(\S+)\s+(\S+)\s+r(\d)\b.*?\b(find|route)\s+(start|end)\b", re.I)
RUNNER_ROUTE = re.compile(r"^\s*e02\s+\d+/\d+\s+(\S+)\s+\S+\s+r\d\s+route\s+(start|end)\b", re.I)


def parse_marks(events):
    """events.csv rows -> {run_id: {t_find|t_start|t_far|t_end: t_wall}} from texts like 'E02 run 7 start', plus the
    runner's own marks (tools/experiment/e02.py), which carry no run id:
      'e02 <k>/<n> <robot> <cond> r<route> [<robot>] find start|end', '... route start|end'   -> key 'tag:<k>/<robot>'
      'e02 <order> <robot> find start|end' (older runner)                                      -> key 'order:<order>'
    A route mark without its own find belongs to that robot's last find. Tagged runs also carry robot, condition and
    route (keys 'robot', 'condition', 'route'), so load_runs can match them to run-list rows in session order."""
    out = {}
    last_find = {}
    for r in events:
        text = str(r.get("text", ""))
        t = nanf(r.get("t_wall"))
        mt = RUNNER_TAG.match(text)
        if mt:
            k, rb, cond, route, ph, se = mt.groups()
            key = "tag:%s/%s" % (k, rb)
            d = out.setdefault(key, dict(robot=rb, condition=canon_cond(cond), route=canon_route(route), seq=len(out)))
            if ph.lower() == "find":
                last_find[rb] = key
                if se.lower() == "start":
                    d["t_find"] = t
            else:
                d["t_start" if se.lower() == "start" else "t_end"] = t
            continue
        mf = RUNNER_FIND.match(text)
        if mf:
            last_find[mf.group(2)] = "order:" + mf.group(1)
            if mf.group(3).lower() == "start":
                out.setdefault("order:" + mf.group(1), {})["t_find"] = t
            continue
        mr = RUNNER_ROUTE.match(text)
        if mr:
            o = last_find.get(mr.group(1))
            if o is not None:
                out.setdefault(o, {})["t_start" if mr.group(2).lower() == "start" else "t_end"] = t
            continue
        m = re.search(r"\brun[\s_:#-]*([A-Za-z0-9]+)", text, re.I)
        ph = re.search(r"\b(find|start|route|far|end)\b", text, re.I)
        if not (m and ph):
            continue
        key = MARK_PHASES[ph.group(1).lower()]
        out.setdefault(m.group(1), {})[key] = nanf(r.get("t_wall"))
    return out


def load_runs(runs_path, marks, vc=None):
    """The run list as dicts: run_id, robot, condition, route, block, robot_run, recovery, t_* (camera clock)."""
    rows = read_csv(runs_path) if runs_path else []
    runs, used = [], set()
    if not rows:
        rows = [dict(run_id=k) for k in sorted(marks, key=lambda s: (len(s), s)) if ":" not in k]
        rows += [dict({x: v for x, v in marks[k].items() if x != "seq"}, run_id=k[4:].replace("/", "-"))
                 for k in sorted(marks, key=lambda k: marks[k].get("seq", 0)) if k.startswith("tag:")]
    rows = sorted(rows, key=lambda r: nanf(pick(r, ["order"], 0)) if pick(r, ["order"]) is not None else 0)
    for i, r in enumerate(rows):
        d = {k: pick(r, ALIASES[k]) for k in ALIASES if not k.startswith("t_")}
        if d["run_id"] is None:     # e.g. a failed run the robot never started: the runner's order number
            d["run_id"] = ("o%s" % d["order"]) if d.get("order") is not None else str(i + 1)
        d["run_id"] = str(d["run_id"])
        d["condition"] = canon_cond(d["condition"])
        d["route"] = canon_route(d["route"])
        rec = str(d["recovery"] or "").strip()
        if rec.lower() in ("0", "false", "no", "n", "ok", "none"):
            rec = ""
        elif rec.lower() in ("1", "true", "yes", "y"):
            rec = "lost"                                  # an old run list's lost flag
        d["recovery"] = rec
        mk = dict(marks.get("order:%s" % d.get("order"), {}), **marks.get(d["run_id"], {}))
        if not mk:
            for key in sorted((k for k in marks if k.startswith("tag:") and k not in used),
                              key=lambda k: marks[k]["seq"]):
                m = marks[key]
                if (m["robot"], m["condition"], m["route"]) == (d["robot"], d["condition"], d["route"]):
                    used.add(key)
                    mk = {k: v for k, v in m.items() if k.startswith("t_")}
                    break
        d["time_source"] = ""
        for k in ("t_find", "t_start", "t_far", "t_end"):
            v = nanf(pick(r, ALIASES[k]))
            if not np.isfinite(v):
                lap = nanf(pick(r, [n + "_laptop" for n in ALIASES[k]]))
                v = laptop_to_video(lap, vc) if np.isfinite(lap) else mk.get(k, float("nan"))
                if np.isfinite(v) and not np.isfinite(lap):
                    d["time_source"] = "video marks"
            d[k] = v
        runs.append(d)
    return runs


# =============================================================================== camera + video
def load_camera(path):
    import cameras
    if path is None:
        return None
    with open(path) as f:
        d = json.load(f)
    if "K" in d and "rvec" in d:
        return cameras.CameraModel.from_dict(d)
    return cameras.CameraModel.from_floor_calibration(path)


class Video:
    """Frames by number; row i of frames.csv is frame i of video.avi. Gray images."""

    def __init__(self, folder, video=None, fix_dir=None):
        import cv2
        self.cv2 = cv2
        self.folder = folder
        fr = read_csv(os.path.join(folder, "frames.csv"))
        self.t = np.array([nanf(r["t_wall"]) for r in fr], float)
        path = video or os.path.join(folder, "video.avi")
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise SystemExit("Could not open the video in %s" % folder)
        if int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT)) <= 0 and len(self.t):
            # no index (the recorder did not stop cleanly, or is still running): jumping to a frame lands on the wrong
            # picture. ffmpeg rewrites it with an index without re-encoding.
            self.cap.release()
            import shutil
            import subprocess
            fixed = os.path.join(fix_dir or folder, "video_indexed.avi")
            if not os.path.exists(fixed):
                if not shutil.which("ffmpeg"):
                    raise SystemExit("The video in %s has no index (the recording was not stopped with Ctrl+C), so "
                                     "frames cannot be found by number. Fix it with:\n  ffmpeg -i \"%s\" -c copy \"%s\"\n"
                                     "then run this again." % (folder, path, fixed))
                os.makedirs(os.path.dirname(fixed), exist_ok=True)
                print("The video has no index: writing one (%s) ..." % fixed, flush=True)
                subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", path, "-c", "copy", fixed], check=True)
            self.cap = cv2.VideoCapture(fixed)
            self.reindexed = fixed
        self.n = min(len(self.t), int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT)) or len(self.t))
        self._next = 0
        ev = os.path.join(folder, "events.csv")
        self.events = read_csv(ev) if os.path.exists(ev) else []

    def index_at(self, t):
        return int(np.clip(np.searchsorted(self.t[:self.n], t), 0, self.n - 1))

    def gray(self, i):
        if i != self._next:
            self.cap.set(self.cv2.CAP_PROP_POS_FRAMES, i)
        ok, fr = self.cap.read()
        self._next = i + 1
        if not ok:
            return None
        return self.cv2.cvtColor(fr, self.cv2.COLOR_BGR2GRAY) if fr.ndim == 3 else fr


def measure_pose(gray, hint_uv, hint_ang, size_px, bg=None, min_contrast=10.0):
    """Find the robot near hint_uv in one picture. Returns dict(u, v, ang, area, ok) (px; ang in deg, 0 = picture
    right, counter-clockwise on screen positive) or None. The robot is what differs from the floor next to it
    (bg: a picture of the empty board, else the median of a ring around the search window). Heading is the
    rectangle's four sides' directions; the one nearest hint_ang (if given) is taken."""
    import cv2
    H, W = gray.shape[:2]
    long_px = size_px * ROBOT_L_CM / ROBOT_W_CM
    area_exp = size_px * long_px
    half = int(round(1.9 * long_px))
    cu, cv_ = float(hint_uv[0]), float(hint_uv[1])
    res = None
    for _ in range(2):
        u0, u1 = max(0, int(cu) - half), min(W, int(cu) + half + 1)
        v0, v1 = max(0, int(cv_) - half), min(H, int(cv_) + half + 1)
        if u1 - u0 < 8 or v1 - v0 < 8:
            return None
        win = cv2.GaussianBlur(gray[v0:v1, u0:u1].astype(np.float32), (0, 0), 0.8)
        if bg is not None:
            ref = cv2.GaussianBlur(bg[v0:v1, u0:u1].astype(np.float32), (0, 0), 0.8)
        else:
            ring = np.concatenate([win[:3].ravel(), win[-3:].ravel(), win[:, :3].ravel(), win[:, -3:].ravel()])
            ref = float(np.median(ring))
        diff = np.abs(win - ref)
        top = float(np.percentile(diff, 99.5))
        if top < 1.5 * min_contrast:
            return None
        if bg is not None:
            # against a picture of the empty scene: everything clearly above the picture noise is robot (its pale top
            # differs from the floor far less than its dark wheels do; half the strongest difference would cut the
            # top in and out of the blob from frame to frame)
            noise = 1.4826 * float(np.median(np.abs(diff - np.median(diff))))
            mask = (diff > max(min_contrast, 4.0 * noise, 0.2 * top)).astype(np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        else:
            mask = (diff > max(min_contrast, 0.5 * top)).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, st, cen = cv2.connectedComponentsWithStats(mask)
        best, bd = None, 1e18
        for k in range(1, n):
            a = st[k, cv2.CC_STAT_AREA]
            if a < 0.35 * area_exp or a > 3.0 * area_exp:
                continue
            d = math.hypot(cen[k][0] + u0 - cu, cen[k][1] + v0 - cv_)
            if d < bd:
                best, bd = k, d
        if best is None:
            return None
        ys, xs = np.nonzero(lab == best)
        mu, mv = xs.mean() + u0, ys.mean() + v0
        res = (mu, mv, lab == best, int(len(xs)), diff, top, u0, v0)
        if math.hypot(mu - cu, mv - cv_) < 0.2 * half:
            break
        cu, cv_ = mu, mv
    mu, mv, comp, area, diff, top, u0, v0 = res
    # sub-pixel centre: the blob weighted by how much each pixel differs from the floor
    near = cv2.dilate(comp.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
    if bg is not None:          # the silhouette's centre: contrast weights would pull it towards the dark wheels
        w = comp.astype(np.float64)
    else:
        w = np.where(near, np.clip(diff - 0.15 * top, 0, None), 0.0)
    if w.sum() > 0:
        gy, gx = np.mgrid[0:diff.shape[0], 0:diff.shape[1]]
        mu, mv = float((w * gx).sum() / w.sum()) + u0, float((w * gy).sum() / w.sum()) + v0
    # heading mod 90: the direction of the blob's edges (gradient directions counted four-fold, so the four
    # sides of a near-square body agree), weighted by edge strength
    sm = cv2.GaussianBlur(diff, (0, 0), 1.0)
    gx, gy = cv2.Sobel(sm, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(sm, cv2.CV_32F, 0, 1, ksize=3)
    mag = np.hypot(gx, gy) * near
    z = (mag * np.exp(4j * np.arctan2(gy, gx))).sum()
    phi = float(wrap180(-math.degrees(np.angle(z)) / 4.0))
    cands = [float(wrap180(phi + 90 * k)) for k in range(4)]
    if hint_ang is not None and np.isfinite(hint_ang):
        ang = min(cands, key=lambda c: abs(float(wrap180(c - hint_ang))))
    else:
        ang = cands[0]
    return dict(u=float(mu), v=float(mv), ang=float(ang), area=area, ok=True)


def floor_pose(cam, m, centre_offset_cm=0.0):
    """measure_pose result -> floor pose (x, y, th_deg) of the axle centre; nan if no camera."""
    if cam is None or m is None:
        return (float("nan"),) * 3
    import cameras
    x, y = cam.to_floor(m["u"], m["v"], TOP_CM)
    th = cameras.image_heading_to_floor(cam, m["u"], m["v"], m["ang"], TOP_CM)
    th = float(wrap180(th))
    x -= centre_offset_cm * math.cos(math.radians(th))
    y -= centre_offset_cm * math.sin(math.radians(th))
    return (float(x), float(y), th)


def px_size(cam, uv_hint, floor_xy=None, default=23.0):
    if cam is None or floor_xy is None or not np.all(np.isfinite(floor_xy)):
        return default
    return float(ROBOT_W_CM * cam.px_per_cm(floor_xy[0], floor_xy[1], TOP_CM))


def track(video, cam, i_anchor, i_lo, i_hi, hint_uv, hint_ang, size_px, bg=None, max_gap=25):
    """Measure every frame from i_lo to i_hi, outward from the anchor frame (forward, then backward)."""
    res = {}
    for step, stop in ((1, i_hi), (-1, i_lo)):
        uv, ang, miss, i = hint_uv, hint_ang, 0, i_anchor
        while (i <= stop) if step > 0 else (i >= stop):
            g = video.gray(i)
            m = measure_pose(g, uv, ang, size_px, bg) if g is not None else None
            if m is not None and math.hypot(m["u"] - uv[0], m["v"] - uv[1]) > 0.5 * size_px * (miss + 1):
                m = None        # a jump no robot makes in a frame (the board's edge or a parked robot): not it
            if m is not None:
                uv, ang, miss = (m["u"], m["v"]), m["ang"], 0
            else:
                miss += 1
            if i not in res or res[i] is None:
                res[i] = m
            if miss > max_gap:
                break
            i += step
    return res


def locate_by_motion(video, i0, i1, size_px, n_probe=6, lag=4):
    """Fallback hint: where a robot is moving between frames i0 and i1 (frame difference): ((u, v), frame) or None."""
    import cv2
    best, bs, bi = None, 0.0, None
    for i in np.linspace(i0, max(i0, i1 - lag - 1), n_probe).astype(int):
        a, b = video.gray(int(i)), video.gray(int(i) + lag)
        if a is None or b is None:
            continue
        d = cv2.GaussianBlur(cv2.absdiff(a, b), (0, 0), 2.0)
        _, mx, _, loc = cv2.minMaxLoc(d)
        if mx > bs:
            bs, best, bi = mx, loc, int(i)
    return (best, bi) if bs >= 15 else None


# =============================================================================== still stretches, contact
def still_segments(t, xy, tol, min_dur):
    """[(i0, i1)] runs of frames where the position stays within tol of its start (>= min_dur seconds)."""
    segs, n, i = [], len(t), 0
    ok = np.isfinite(xy[:, 0]) & np.isfinite(xy[:, 1])
    while i < n:
        if not ok[i]:
            i += 1
            continue
        j = i
        anchor = np.nanmedian(xy[i:i + 5][ok[i:i + 5]], axis=0)
        while j + 1 < n and ok[j + 1] and np.hypot(*(xy[j + 1] - anchor)) < tol:
            j += 1
        if t[j] - t[i] >= min_dur:
            segs.append((i, j))
        i = j + 1 if j > i else i + 1
    return segs


def _smooth(x, k=3):
    x = np.asarray(x, float)
    if len(x) < k:
        return x
    pad = np.pad(x, k // 2, mode="edge")
    return np.array([np.median(pad[i:i + k]) for i in range(len(x))])


def _smooth_xy(xy, k=5):
    """Running median of each column over k frames, ignoring missing frames (a frame stays missing if it was)."""
    xy = np.asarray(xy, float)
    n, h = len(xy), k // 2
    out = np.full_like(xy, np.nan)
    for i in range(n):
        if not np.all(np.isfinite(xy[i])):
            continue
        w = xy[max(0, i - h):i + h + 1]
        w = w[np.all(np.isfinite(w), axis=1)]
        out[i] = np.median(w, axis=0)
    return out


def contact_frames(t, xy, head_deg, v_min=1.5):
    """Contact moments in a stretch of track: list of (index, progress_cm) where the robot, moving forward along
    its heading, stops advancing and stays stopped. The frame is the first on the plateau (within a tolerance of
    the level it then keeps). xy in cm, head_deg the robot's heading per frame."""
    n = len(t)
    if n < 12:
        return []
    h = np.radians(np.where(np.isfinite(head_deg), head_deg, np.nanmedian(head_deg)))
    fx = np.nan_to_num(xy[:, 0]) * np.cos(h) + np.nan_to_num(xy[:, 1]) * np.sin(h)      # progress along heading
    s = _smooth(fx, 3)
    k = 4
    v = np.zeros(n)
    for i in range(n):
        a, b = max(0, i - k), min(n, i + k + 1)
        if b - a >= 4:
            v[i] = np.polyfit(t[a:b], s[a:b], 1)[0]
    turn = np.abs(np.gradient(np.unwrap(np.radians(np.where(np.isfinite(head_deg), head_deg, 0.0))), t)) > math.radians(20)
    v = np.where(_smooth(turn.astype(float), 5) > 0, 0.0, v)         # a turn on the spot is not progress
    fwd = v[v > v_min]
    thr = max(v_min, 0.35 * float(np.percentile(fwd, 75))) if len(fwd) else float("inf")
    out, i = [], 0
    moving = v > thr
    while i < n:
        if not moving[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and moving[j + 1]:
            j += 1
        if j - i >= 3 and j + 6 < n:
            after = s[j + 2:j + 8]
            if np.ptp(after) < 1.5 and np.median(v[j + 2:j + 8]) < 0.35 * np.max(v[i:j + 1]):
                plateau = float(np.median(after))
                sig = 1.4826 * float(np.median(np.abs(after - plateau)))
                tol = max(0.35, 2.5 * sig)
                c = j
                while c > i and s[c - 1] >= plateau - tol:
                    c -= 1
                out.append((c, plateau))
        i = j + 1
    return out


# =============================================================================== per-run measurement
class Settings:
    def __init__(self, cam=None, bg=None, centre_offset_cm=0.0, bump_frame="auto", routes=None, robot_px=23.0,
                 lead_s=1.0, tail_s=2.0, still_tol_cm=0.6, still_s=0.5):
        self.cam, self.bg, self.centre_offset_cm, self.bump_frame = cam, bg, centre_offset_cm, bump_frame
        self.routes = dict(ROUTES, **(routes or {}))
        self.robot_px, self.lead_s, self.tail_s = robot_px, lead_s, tail_s
        self.still_tol_cm, self.still_s = still_tol_cm, still_s


def _nan3():
    return (float("nan"),) * 3


def _med_pose(frames, idxs, cam, off):
    """Median measurement over frames -> (floor pose, (u, v, ang) px, n)."""
    ms = [frames[i] for i in idxs if frames.get(i)]
    if not ms:
        return _nan3(), _nan3(), 0
    u, v = np.median([m["u"] for m in ms]), np.median([m["v"] for m in ms])
    a = quarter_median([m["ang"] for m in ms])
    fp = floor_pose(cam, dict(u=u, v=v, ang=a), off)
    return fp, (float(u), float(v), a), len(ms)


def err_pair(meas, intended):
    if not (np.all(np.isfinite(meas)) and np.all(np.isfinite(intended))):
        return float("nan"), float("nan")
    return (float(math.hypot(meas[0] - intended[0], meas[1] - intended[1])),
            float(wrap180(meas[2] - intended[2])))


def measure_run(video, run, logs, S):
    """All the measurements for one run: returns (row dict for runs.csv, bump rows, track rows)."""
    cam, off = S.cam, S.centre_offset_cm
    flags = []
    robot, rr = run["robot"], run.get("robot_run")
    fr = run.get("find_run") or rr          # the runner's find is a run of its own (FIND and BUMP rows carry its id)
    e02 = logs.e02_row(robot, rr) if rr is not None else None
    find = logs.find_row(robot, fr) if fr is not None else None
    bel = {st: stage_pose(e02, st) for st in ("start", "far", "end")}
    if bel["start"] is None and find is not None:
        bel["start"] = stage_pose(find, "after")
    route = run["route"]
    vc = getattr(S, "vc", None)
    if e02 is not None and vc and (run.get("time_source") or not np.isfinite(run["t_start"])):
        # times from video marks are when the laptop sent them (the route mark comes before the robot's drive to its
        # start spot): the robot's own E02 moments, on the camera clock, are better
        clk = logs.robot_offset(robot, str(rr).split("-")[0] if "-" in str(rr) else None)
        if clk is not None:
            run = dict(run)
            for k in ("start", "far", "end"):
                ms = nanf(pick(e02, ["ms_" + k]))
                if np.isfinite(ms):
                    run["t_" + k] = laptop_to_video(ms / 1000.0 + clk, vc)
            flags.append("times from the robot's E02 record")
    t_start, t_end = run["t_start"], run["t_end"]
    row = dict(run_id=run["run_id"], robot=robot, condition=run["condition"], route=route, block=run.get("block"),
               robot_run=rr, find_run=run.get("find_run") or "", status=run.get("status") or "",
               recovery=run.get("recovery", ""))
    if not (np.isfinite(t_start) and np.isfinite(t_end)):
        row["flags"] = "no start/end time"
        return row, [], []
    t_find = run["t_find"] if np.isfinite(run["t_find"]) else t_start - 30.0
    bumps = logs.bump_rows(robot, fr) if fr is not None else []
    t_lo = (t_find - 0.5) if bumps else (t_start - S.lead_s)
    i_lo, i_hi = video.index_at(t_lo), video.index_at(t_end + S.tail_s)
    i_anchor = video.index_at(t_start - 0.3)
    if S.bg is None and getattr(S, "run_background", True):
        # this run's empty scene: the median of pictures spread over the run, so the moving robot drops out while
        # the boards and the parked robots stay in (the robot is then what differs from it, even against a board)
        S_bg = run_background(video, i_lo, i_hi)
        if S_bg is not None:
            S = _with_bg(S, S_bg)

    # where to start looking: the run list's park spot, the believed start, else where something moves
    hint_uv, hint_ang, hint_xy = None, None, None
    if cam is not None and bel["start"] is not None:
        x, y, th = bel["start"]
        u, v = cam.project((x, y, TOP_CM))
        if np.isfinite(u) and cam.in_image(u, v):
            import cameras
            hint_uv, hint_xy = (u, v), (x, y)
            hint_ang = cameras.floor_heading_to_image(cam, x, y, TOP_CM, th)
    size = px_size(cam, hint_uv, hint_xy, S.robot_px)
    first = None
    if hint_uv is not None:
        g = video.gray(i_anchor)
        first = measure_pose(g, hint_uv, hint_ang, size, S.bg) if g is not None else None
    if first is None:
        loc = locate_by_motion(video, video.index_at(t_start), video.index_at(t_end), size)
        if loc is not None:
            flags.append("start found by motion")
            hint_uv, hint_ang = loc[0], None
            i_anchor = loc[1]
            first = measure_pose(video.gray(i_anchor), hint_uv, None, size, S.bg)
    if first is None:
        row["flags"] = "robot not found at start"
        return row, [], []
    frames = track(video, cam, i_anchor, i_lo, i_hi, (first["u"], first["v"]), first["ang"], size, S.bg)
    # the find, tracked forward from where the robot stood when it began (FIND before_x/y/th, floor frame when the
    # find started from a camera fix): tracking backwards from the start can lose it in the drive to the start spot
    bb = stage_pose(find, "before") if find is not None else None
    if bumps and bb is not None and cam is not None and nanf(pick(find, ["known_before"])) == 1 and \
            sum(1 for i in range(i_lo, i_anchor) if frames.get(i)) < 0.8 * (i_anchor - i_lo):
        import cameras
        u, v = cam.project((bb[0], bb[1], TOP_CM))
        if np.isfinite(u) and cam.in_image(u, v):
            i_f = video.index_at(t_find + 0.1)
            g = video.gray(i_f)
            m0 = measure_pose(g, (u, v), cameras.floor_heading_to_image(cam, bb[0], bb[1], TOP_CM, bb[2]), size,
                              S.bg) if g is not None else None
            if m0 is not None:
                more = track(video, cam, i_f, i_lo, i_anchor, (m0["u"], m0["v"]), m0["ang"], size, S.bg)
                for i, m in more.items():
                    if m is not None and not frames.get(i):
                        frames[i] = m
    idx = sorted(frames)
    t = video.t[idx]
    uv = np.array([[frames[i]["u"], frames[i]["v"]] if frames[i] else [np.nan, np.nan] for i in idx])
    ang = np.array([frames[i]["ang"] if frames[i] else np.nan for i in idx])
    fl = np.array([floor_pose(cam, frames[i], off) if frames[i] else _nan3() for i in idx]) if cam else \
        np.full((len(idx), 3), np.nan)
    xy = fl[:, :2] if cam else uv
    tol = S.still_tol_cm if cam else S.still_tol_cm * 2.3
    # still stretches on a 5-frame running median: a standing robot's blob edge can flicker between two outlines
    # (the measured centre then hops ~1 cm frame to frame); the stage poses are still medians of the raw frames
    segs = still_segments(t, _smooth_xy(xy, 5), tol, S.still_s)
    trk = [dict(run_id=run["run_id"], frame=idx[k], t_wall=round(float(t[k]), 4), u_px=uv[k, 0], v_px=uv[k, 1],
                ang_px_deg=ang[k], x_cm=fl[k, 0], y_cm=fl[k, 1], th_deg=fl[k, 2]) for k in range(len(idx))]
    pos = {i: k for k, i in enumerate(idx)}

    def seg_pose(seg):
        k0, k1 = seg
        lo, hi = k0 + min(2, (k1 - k0) // 4), k1 - min(2, (k1 - k0) // 4)
        return _med_pose(frames, idx[lo:hi + 1], cam, off)

    # start: the still stretch overlapping [t_start - lead, t_start + 0.5]; end: the last; far: between them
    s_seg = max((s for s in segs if t[s[0]] <= t_start + 0.5 and t[s[1]] >= t_start - S.lead_s),
                key=lambda s: t[s[1]] - t[s[0]], default=None)
    after = [s for s in segs if s_seg is None or s[0] > s_seg[1]]
    e_seg = after[-1] if after else None
    # the end: the still stretch the robot was in at its own end moment (it may be driven off to park within the
    # tail), else the last one
    at_end = [s for s in after if t[s[0]] <= t_end + 0.5 and t[s[1]] >= t_end - 0.2]
    if at_end:
        e_seg = max(at_end, key=lambda s: min(t[s[1]], t_end + 1.5) - max(t[s[0]], t_end - 0.5))
    mid = [s for s in after if s is not e_seg and t[s[1]] - t[s[0]] >= 0.6]
    f_seg = None
    if mid:
        if np.isfinite(run["t_far"]):
            f_seg = min(mid, key=lambda s: abs(0.5 * (t[s[0]] + t[s[1]]) - run["t_far"]))
        elif s_seg is not None:
            p0 = np.nanmedian(xy[s_seg[0]:s_seg[1] + 1], axis=0)
            f_seg = max(mid, key=lambda s: np.hypot(*(np.nanmedian(xy[s[0]:s[1] + 1], axis=0) - p0)))
    if os.environ.get("E02_DEBUG"):
        print("  segs rel t_start:", [(round(t[a] - t_start, 2), round(t[b] - t_start, 2)) for a, b in segs],
              "t_far %.2f t_end %.2f" % (run["t_far"] - t_start, t_end - t_start), "track %.2f..%.2f, %d/%d found" % (
                  t[0] - t_start, t[-1] - t_start, sum(1 for i in idx if frames[i]), len(idx)))
    for nm, sg in (("start", s_seg), ("far", f_seg), ("end", e_seg)):
        if sg is None:
            flags.append("no still stretch for %s" % nm)
    P = {}
    for nm, sg in (("start", s_seg), ("far", f_seg), ("end", e_seg)):
        P[nm] = seg_pose(sg) if sg else (_nan3(), _nan3(), 0)
    # headings: the body is nearly square, so each picture gives the heading only up to a quarter turn. Resolve it
    # with the robot's own record: its believed start heading (floor frame after a find from a camera fix) and its
    # believed turn since the start (never a quarter turn off: a U gyro is ~7% off)
    head_note = ""
    if cam is not None and bel["start"] is not None and np.isfinite(P["start"][0][2]):
        hs = bel["start"][2] if (find is not None and nanf(pick(find, ["known_before"])) == 1) else P["start"][0][2]
        new_th = {"start": snap90(P["start"][0][2], hs)}
        for nm in ("far", "end"):
            if bel[nm] is not None and np.isfinite(P[nm][0][2]):
                new_th[nm] = snap90(P[nm][0][2], new_th["start"] + float(wrap180(bel[nm][2] - bel["start"][2])))
        img_sign = _image_turn_sign(cam, P["start"][0])
        for nm, th in new_th.items():
            fp, px, n = P[nm]
            d = float(wrap180(th - fp[2]))
            if abs(d) > 1e-6:
                head_note = "heading resolved by the robot's record"
            P[nm] = ((fp[0], fp[1], th), (px[0], px[1], float(wrap180(px[2] + img_sign * d))), n)
    if head_note:
        flags.append(head_note)
    ms, mf, me = P["start"][0], P["far"][0], P["end"][0]
    rel = S.routes.get(route)
    if rel is not None and rel[1] != 0:
        sg = nanf(run.get("turn_sign"))
        if not np.isfinite(sg) and np.all(np.isfinite(ms)) and np.all(np.isfinite(mf)):
            sg = float(np.sign(relative(mf, ms)[1]))
        sg = sg if np.isfinite(sg) and sg != 0 else 1.0
        rel = (rel[0], sg * rel[1], sg * rel[2])
    row["far_intent_from"] = "route table" if rel is not None else ""
    if e02 is not None:
        g = [nanf(pick(e02, ["far_goal_%s" % a, "far_goal_%s_cm" % a, "far_goal_%s_deg" % a])) for a in ("x", "y", "th")]
        if all(np.isfinite(g)):
            rel = tuple(g)
            row["far_intent_from"] = "E02 far_goal (relative)"
        elif np.all(np.isfinite(g[:2])) and bel["start"] is not None:
            # firmware 2026-10-21b: far_goal_x/y is the far point the robot aimed for, in the same floor frame as
            # its believed start: the intended offset in the start's own frame, exact for this run (turn side too)
            ga, gl, _ = relative((g[0], g[1], 0.0), bel["start"])
            dth = rel[2] if rel is not None else 0.0
            if rel is not None and rel[1] != 0 and gl != 0:
                dth = math.copysign(abs(rel[2]), gl)
            rel = (ga, gl, dth)
            row["far_intent_from"] = "E02 far_goal"
    int_far = compose(ms, rel) if rel is not None and np.all(np.isfinite(ms)) else _nan3()
    int_end = ms
    row.update(dict(
        start_x=ms[0], start_y=ms[1], start_th=ms[2], far_x=mf[0], far_y=mf[1], far_th=mf[2],
        end_x=me[0], end_y=me[1], end_th=me[2],
        int_far_x=int_far[0], int_far_y=int_far[1], int_far_th=int_far[2], int_end_x=int_end[0],
        int_end_y=int_end[1], int_end_th=int_end[2], n_frames_start=P["start"][2], n_frames_far=P["far"][2],
        n_frames_end=P["end"][2]))
    row["far_err_cm"], row["far_err_deg"] = err_pair(mf, int_far)
    row["end_err_cm"], row["end_err_deg"] = err_pair(me, int_end)
    row["far_head_abs_deg"], row["end_head_abs_deg"] = abs(row["far_err_deg"]), abs(row["end_err_deg"])
    # pixel-to-pixel return: start vs end frame, no calibration
    ps, pe = P["start"][1], P["end"][1]
    row["px_return_dist"] = float(math.hypot(pe[0] - ps[0], pe[1] - ps[1])) if np.all(np.isfinite([*ps, *pe])) else float("nan")
    row["px_return_deg"] = float(wrap180(pe[2] - ps[2])) if np.all(np.isfinite([ps[2], pe[2]])) else float("nan")
    row["px_return_deg_abs"] = abs(row["px_return_deg"])
    # believed (E02 record) against the camera, as displacement from the start in the start's own frame
    for nm, mp in (("far", mf), ("end", me)):
        b0, b1 = bel["start"], bel[nm]
        if b0 and b1 and np.all(np.isfinite(ms)) and np.all(np.isfinite(mp)):
            rb, rm = relative(b1, b0), relative(mp, ms)
            row["bel_%s_err_cm" % nm] = float(math.hypot(rb[0] - rm[0], rb[1] - rm[1]))
            row["bel_%s_err_deg" % nm] = float(wrap180(rb[2] - rm[2]))
        else:
            row["bel_%s_err_cm" % nm] = row["bel_%s_err_deg" % nm] = float("nan")
    row["bel_end_head_abs_deg"] = abs(row["bel_end_err_deg"])
    # find: the believed start (floor frame after find) against the camera
    b0 = bel["start"]
    if b0 and np.all(np.isfinite(ms)):
        row["find_err_cm"], row["find_err_deg"] = err_pair(b0, ms)[0], float(wrap180(b0[2] - ms[2]))
        row["believed_in_floor_frame"] = int(row["find_err_cm"] < 20 and abs(row["find_err_deg"]) < 30)
        if row["believed_in_floor_frame"]:     # absolute errors: intended from the believed start, floor frame
            row["abs_far_err_cm"], row["abs_far_err_deg"] = err_pair(mf, compose(b0, rel) if rel else _nan3())
            row["abs_end_err_cm"], row["abs_end_err_deg"] = err_pair(me, b0)
    else:
        row["find_err_cm"] = row["find_err_deg"] = float("nan")
    if find is not None:
        row["find_result"] = pick(find, ["result"], "")
    for k in ("abs_far_err_cm", "abs_far_err_deg", "abs_end_err_cm", "abs_end_err_deg"):
        row.setdefault(k, float("nan"))
    row["loss_note"] = ""
    if run.get("recovery"):
        flags.append("needed recovery: %s" % run["recovery"])
    if not (np.isfinite(row["end_err_cm"])):
        flags.append("no end error")
    row["flags"] = "; ".join(flags)

    # bumps
    brows = []
    if bumps and s_seg is not None and np.all(np.isfinite(ms)):
        k_end = pos.get(min(idx[-1], video.index_at(t_start)), len(idx) - 1)
        kk = np.arange(0, max(k_end, 1))
        events = contact_frames(t[kk], xy[kk], np.where(np.isfinite(fl[kk, 2]), fl[kk, 2], 0.0) if cam else
                                -ang[kk]) if len(kk) else []
        pending = list(events)
        clk = logs.robot_offset(robot, str(fr).split("-")[0] if "-" in str(fr) else None)
        vc = getattr(S, "vc", None)
        for bi, b in enumerate(bumps):
            t_rec = nanf(pick(b, ["t_wall"]))
            if not np.isfinite(t_rec):
                ms_b = nanf(pick(b, ["ms", "ms_contact", "ms_detect"]))
                if np.isfinite(ms_b) and clk is not None:
                    t_rec = laptop_to_video(ms_b / 1000.0 + clk, vc)
            delay_s = nanf(pick(b, ["delay_ms"])) / 1000.0
            ev = None
            if pending:
                if np.isfinite(t_rec):
                    cand = min(pending, key=lambda e: abs(t[e[0]] - (t_rec - (delay_s if np.isfinite(delay_s) else 0))))
                    if abs(t[cand[0]] - (t_rec - (delay_s if np.isfinite(delay_s) else 0))) < 3.0:
                        ev = cand
                else:
                    ev = pending[0]
                if ev is not None:
                    pending.remove(ev)
            br = dict(run_id=run["run_id"], robot=robot, condition=run["condition"], bump=bi + 1,
                      board=pick(b, ["board", "axis"], ""), jolt=nanf(pick(b, ["jolt", "jolt_g", "jolt_size"])),
                      delay_ms=nanf(pick(b, ["delay_ms"])), overshoot_cm=nanf(pick(b, ["overshoot_cm", "overshoot"])))
            bp = tuple(nanf(pick(b, names)) for names in (["x_cm", "x", "pose_x", "x_contact"],
                                                           ["y_cm", "y", "pose_y", "y_contact"],
                                                           ["th_deg", "th", "pose_th", "theta_deg"]))
            br.update(bel_x=bp[0], bel_y=bp[1], bel_th=bp[2])
            if ev is None:
                br["note"] = "no contact found in the video"
                brows.append(br)
                continue
            c = ev[0]
            cp = (float(fl[c, 0]), float(fl[c, 1]), float(fl[c, 2]))
            if np.isfinite(cp[0]):
                br.update(cam_x=cp[0], cam_y=cp[1], cam_th=cp[2], contact_frame=idx[c], t_contact=float(t[c]))
            bf = bp
            frame = S.bump_frame
            if frame == "auto":
                # a find that started from a camera fix (known_before 1) works in floor coordinates; one with no
                # pose before started its own frame where it was put down
                kb = nanf(pick(find, ["known_before"])) if find is not None else float("nan")
                frame = "floor" if kb == 1 else "own"
            br["bump_frame"] = frame
            if frame == "own" and np.all(np.isfinite(bp)):
                # the robot's own frame starts where the find began: where the camera saw it then
                bf = compose(_find_origin(fl) or ms, bp)
            if np.all(np.isfinite(bf)) and np.isfinite(cp[0]):
                br["bel_floor_x"], br["bel_floor_y"], br["bel_floor_th"] = bf
                br["pos_err_cm"], br["head_err_deg"] = err_pair(bf, cp)
                br["head_err_deg"] = float(wrap180(bf[2] - cp[2]))
                u = (math.cos(math.radians(cp[2])), math.sin(math.radians(cp[2])))
                d = (bf[0] - cp[0], bf[1] - cp[1])
                br["along_err_cm"] = d[0] * u[0] + d[1] * u[1]
                br["cross_err_cm"] = -d[0] * u[1] + d[1] * u[0]
            if np.isfinite(t_rec):
                br["cam_delay_ms"] = float(1000.0 * (t_rec - t[c]))
            brows.append(br)
    return row, brows, trk


def run_background(video, i_lo, i_hi, n=41):
    """Per-pixel median of n pictures spread evenly over frames i_lo..i_hi, or None if too few could be read."""
    gs = []
    for i in np.linspace(i_lo, i_hi, n).astype(int):
        g = video.gray(int(i))
        if g is not None:
            gs.append(g)
    if len(gs) < 9:
        return None
    return np.median(np.stack(gs), axis=0).astype(np.uint8)


def _with_bg(S, bg):
    import copy
    S2 = copy.copy(S)
    S2.bg = bg
    return S2


def snap90(th, hint):
    """th moved by whole quarter turns to lie within 45 deg of hint (deg)."""
    if not (np.isfinite(th) and np.isfinite(hint)):
        return th
    return float(wrap180(th + 90.0 * round(float(wrap180(hint - th)) / 90.0)))


def _image_turn_sign(cam, pose):
    """+1 if a left (counter-clockwise) turn on the floor is counter-clockwise in the picture, else -1."""
    import cameras
    try:
        a = cameras.floor_heading_to_image(cam, pose[0], pose[1], TOP_CM, pose[2])
        b = cameras.floor_heading_to_image(cam, pose[0], pose[1], TOP_CM, pose[2] + 10.0)
        return 1.0 if float(wrap180(b - a)) > 0 else -1.0
    except Exception:
        return 1.0


def _find_origin(fl):
    """Camera pose at the first frames of the find (the own frame's origin)."""
    sel = np.nonzero(np.isfinite(fl[:, 0]))[0]
    if not len(sel):
        return None
    k = sel[:5]
    return (float(np.median(fl[k, 0])), float(np.median(fl[k, 1])), circ_median(fl[k, 2]))


# =============================================================================== tables
METRICS = [
    ("end_err_cm", "End position error (cm)"),
    ("end_head_abs_deg", "End heading error (deg, size)"),
    ("far_err_cm", "Far-point position error (cm)"),
    ("far_head_abs_deg", "Far-point heading error (deg, size)"),
    ("px_return_dist", "Return to start, picture only (px)"),
    ("px_return_deg_abs", "Return to start, angle in the picture (deg, size)"),
    ("bel_end_err_cm", "Robot's belief vs camera at the end (cm)"),
    ("find_err_cm", "Find position error (cm)"),
]
PRIMARY = ["end_err_cm", "end_head_abs_deg"]


def flag_runs(rows):
    """Add outlier_<metric> flags: beyond the Tukey fences (1.5 x IQR) within the same condition AND route (the
    routes' errors differ by design: pooled, every run of the longest route would look like an outlier). Purely
    statistical: a run the runner had to drive back (recovery) is not flagged for that."""
    for m, _ in METRICS:
        for key in {(r["condition"], r["route"]) for r in rows}:
            grp = [r for r in rows if (r["condition"], r["route"]) == key]
            fl = iqr_outliers([r.get(m, float("nan")) for r in grp])
            for r, f in zip(grp, fl):
                r["outlier_" + m] = int(bool(f))
    return rows


def scopes(rows):
    out = [("all", rows)]
    for rt in sorted({r["route"] for r in rows if r["route"]}):
        out.append(("route=" + rt, [r for r in rows if r["route"] == rt]))
    for rb in sorted({r["robot"] for r in rows if r["robot"]}):
        out.append(("robot=" + rb, [r for r in rows if r["robot"] == rb]))
    return out


def values(rows, metric, cond, trimmed):
    return [r[metric] for r in rows if r["condition"] == cond and np.isfinite(nanf(r.get(metric)))
            and not (trimmed and r.get("outlier_" + metric))]


def strata(rows, metric, cond, trimmed):
    return [(r.get("robot") or "", r.get("route") or "") for r in rows if r["condition"] == cond
            and np.isfinite(nanf(r.get(metric))) and not (trimmed and r.get("outlier_" + metric))]


def summarise(rows):
    summ, comps = [], []
    for scope, rs in scopes(rows):
        for m, _ in METRICS:
            for trimmed in (False, True):
                st = "without outliers" if trimmed else "all runs"
                for c in CONDITIONS:
                    v = values(rs, m, c, trimmed)
                    if v:
                        summ.append(dict(scope=scope, metric=m, set=st, condition=c, **describe(v)))
                pair = []
                for i, a in enumerate(CONDITIONS):
                    for b in CONDITIONS[i + 1:]:
                        va, vb = values(rs, m, a, trimmed), values(rs, m, b, trimmed)
                        if len(va) >= 2 and len(vb) >= 2:
                            d = dict(scope=scope, metric=m, set=st, cond_a=a, cond_b=b, **compare(va, vb))
                            if m in PRIMARY:
                                d["perm_diff"], d["perm_p"], d["perm_strata"] = strat_perm(
                                    va, strata(rs, m, a, trimmed), vb, strata(rs, m, b, trimmed))
                            else:
                                d["perm_diff"], d["perm_p"], d["perm_strata"] = float("nan"), float("nan"), 0
                            pair.append(d)
                # Holm family: the (up to) six condition pairs of one metric, in one scope, with or without outliers
                hp, hm, hq = holm([p["welch_p"] for p in pair]), holm([p["mwu_p"] for p in pair]), \
                    holm([p["perm_p"] for p in pair])
                for p, a, b, q in zip(pair, hp, hm, hq):
                    p["welch_p_holm"], p["mwu_p_holm"], p["perm_p_holm"] = a, b, q
                comps.extend(pair)
    return summ, comps


def write_csv(path, rows, cols=None):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    cols = cols or list(dict.fromkeys(k for r in rows for k in r))
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: ("" if (isinstance(r.get(k), float) and not np.isfinite(r.get(k))) else r.get(k, ""))
                        for k in cols})


# =============================================================================== charts
OKABE = {"U": "#E69F00", "C": "#0072B2", "K": "#009E73", "KC": "#CC79A7"}
MARK = {"straight": "o", "L": "s", "square": "^"}


def spread_chart(rows, metric, label, path, min_n=3):
    """Individual runs as dots per condition, with mean and 95% confidence range. None if too few runs."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    conds = [c for c in CONDITIONS if len(values(rows, metric, c, False)) >= min_n]
    if len(conds) < 2:
        return None
    fig, ax = plt.subplots(figsize=(6.2, 4.0), dpi=130)
    rng = np.random.default_rng(3)
    for k, c in enumerate(conds):
        rs = [r for r in rows if r["condition"] == c and np.isfinite(nanf(r.get(metric)))]
        for r in rs:
            x = k + rng.uniform(-0.18, 0.18)
            hollow = bool(r.get("outlier_" + metric))
            ax.scatter(x, r[metric], marker=MARK.get(r["route"], "o"), s=26, alpha=0.85,
                       facecolors="none" if hollow else OKABE[c], edgecolors=OKABE[c], linewidths=1.2, zorder=3)
        d = describe([r[metric] for r in rs])
        ax.errorbar(k + 0.32, d["mean"], yerr=[[d["mean"] - d["ci_lo"]], [d["ci_hi"] - d["mean"]]], fmt="D",
                    color="black", capsize=4, markersize=5, zorder=4)
    ax.set_xticks(range(len(conds)))
    ax.set_xticklabels(["%s (n=%d)" % (c, len(values(rows, metric, c, False))) for c in conds])
    ax.set_ylabel(label)
    ax.set_ylim(bottom=0)
    ax.grid(axis="y", alpha=0.25)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return path


def bump_chart(bumps, path, min_n=4):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    pts = [b for b in bumps if np.isfinite(nanf(b.get("bel_floor_x"))) and np.isfinite(nanf(b.get("cam_x")))]
    if len(pts) < min_n:
        return None
    fig, ax = plt.subplots(figsize=(5.2, 4.6), dpi=130)
    col = {"x": "#0072B2", "y": "#D55E00"}
    lim = []
    for brd in ("x", "y"):
        sel = [b for b in pts if str(b.get("board", "")).lower() == brd]
        if not sel:
            continue
        # the coordinate across the board it hit: x board -> x, y board -> y
        bel = [b["bel_floor_" + brd] for b in sel]
        cam = [b["cam_" + brd] for b in sel]
        ax.scatter(cam, bel, s=24, color=col[brd], alpha=0.8, label="%s board (n=%d)" % (brd, len(sel)))
        lim += bel + cam
    lo, hi = min(lim), max(lim)
    pad = 0.05 * max(hi - lo, 1)
    ax.plot([lo - pad, hi + pad], [lo - pad, hi + pad], color="grey", lw=1)
    ax.set_xlabel("camera at contact (cm)")
    ax.set_ylabel("robot's belief at contact (cm)")
    ax.legend(frameon=False)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return path


# =============================================================================== report
def f(v, d=2):
    return "-" if v is None or not np.isfinite(v) else ("%.*f" % (d, v))


def p_txt(p):
    return "-" if not np.isfinite(p) else ("<0.001" if p < 0.001 else "%.3f" % p)


def find_row_for(summ, scope, metric, cond, setname="all runs"):
    for r in summ:
        if (r["scope"], r["metric"], r["condition"], r["set"]) == (scope, metric, cond, setname):
            return r
    return None


TESTS_FINE_PRINT = [
    "<sub>Permutation p (the primary test): the condition labels are shuffled 4000 times, but only among "
    "runs of the same robot on the same route, and the difference in means is taken each time; p is the "
    "share of shuffles with a difference at least as large as the real one. It respects that the same "
    "robots and routes recur. Welch's t-test: (secondary) the difference in means divided by its standard "
    "error, without assuming equal spreads; it treats every run as independent. Mann-Whitney U "
    "(secondary): ranks all runs together and asks whether one condition's runs sit higher; it does not "
    "need a bell curve. Holm: each p corrected for the six condition pairs compared within one table (one "
    "measure, one route); tables are not corrected against each other. Cohen's d: the mean difference in "
    "units of the pooled SD (0.2 small, 0.5 medium, 0.8 large). Brown-Forsythe (Levene on distances from "
    "the median): tests whether the spreads differ; p is the chance of spreads this different if they "
    "were equal.</sub>", ""]


def write_report(path, rows, bumps, summ, comps, charts, notes, args_text="", setup=None):
    """setup: setup_recovery()'s result (its lines close the Data section), or None."""
    L = []
    n_by = {c: sum(1 for r in rows if r["condition"] == c) for c in CONDITIONS}
    n_out = sum(1 for r in rows if r.get("outlier_end_err_cm"))
    L += ["# E02: how far the Alviks end from where they meant to be", ""]
    L += ["## Premise", "",
          "Each run: the robot finds its position against the two boards, drives one of three fixed routes with "
          "a 1 s pause at the far point and returns to its start. The overhead camera measures where it "
          "started, paused and stopped; the error is the distance (cm) and heading difference (deg) between "
          "what the camera saw and what was intended. Four conditions: U uncalibrated, C calibrated on-board "
          "sensors (corrections learned during the session from U runs, so the first block is U-heavy), K camera "
          "steering only, KC camera plus calibrated sensors. Question: do the conditions differ, and by how much.", ""]
    rec = [r for r in rows if r.get("recovery")]
    kinds = sorted({r["recovery"] for r in rec})
    L += ["## Data", "",
          "%d runs (%s). %s %d are statistical outliers on end position error (beyond 1.5 x IQR within the same condition "
          "and route); every table gives the mean with and without them. Runs with a missing measurement are in "
          "runs.csv with a note and left out of the statistic they lack. The routes differ in length and turns, so "
          "each route is reported on its own (pooled numbers: summary.csv, scope all)." % (
              len(rows), ", ".join("%s %d" % (c, n_by[c]) for c in CONDITIONS),
              ("%d needed recovery afterwards (%s): the runner drove them back by camera; they stay in every statistic."
               % (len(rec), ", ".join("%s %d" % (k, sum(1 for r in rec if r["recovery"] == k)) for k in kinds)))
              if rec else "None needed recovery.",
              n_out), ""]
    n_meas = sum(1 for r in rows if np.isfinite(nanf(r.get("end_err_cm"))))
    if n_meas < len(rows):
        L += ["%d of %d runs have a camera-measured end error; the other %d (failed before or during the route, or not "
              "found in the video) are listed in runs.csv with the reason in the flags column." % (
                  n_meas, len(rows), len(rows) - n_meas), ""]
    L += ["<sub>1.5 x IQR (Tukey's fences): the IQR is the spread of the middle half of the runs (75th minus 25th "
          "percentile); a run more than 1.5 IQR below the 25th or above the 75th percentile of its condition and route is "
          "called an outlier. Fewer than 4 runs in a group: none are called.</sub>", ""]
    route_scopes = [s for s in dict.fromkeys(x["scope"] for x in summ) if s.startswith("route=")] or ["all"]
    first = True
    for m, lab in [x for x in METRICS if x[0] in PRIMARY]:
        for sc in route_scopes:
            if not any(find_row_for(summ, sc, m, c) for c in CONDITIONS):
                continue
            L += ["**%s, %s**" % (lab, sc.replace("route=", "route ") if sc != "all" else "all routes"), "",
                  "| Condition | n | mean | 95% CI of mean | SD | 95% CI of SD | median | mean without outliers (n) |",
                  "|---|---|---|---|---|---|---|---|"]
            for c in CONDITIONS:
                d = find_row_for(summ, sc, m, c)
                if not d:
                    continue
                t = find_row_for(summ, sc, m, c, "without outliers")
                L.append("| %s | %d | %s | %s to %s | %s | %s to %s | %s | %s (%d) |" % (
                    c, d["n"], f(d["mean"]), f(d["ci_lo"]), f(d["ci_hi"]), f(d["sd"]), f(d["sd_lo"]), f(d["sd_hi"]),
                    f(d["median"]), f(t["mean"]) if t else "-", t["n"] if t else 0))
            L.append("")
            if first:
                L += ["<sub>95% CI of the mean: the range that would contain the true mean in 95 of 100 repeats of "
                      "this experiment, from the t distribution (mean +/- t x SD / sqrt(n)). The CI of the SD is a "
                      "bootstrap: the runs are re-drawn with replacement 4000 times, the SD taken each time, and the "
                      "middle 95% kept; with few runs it is wide and slightly low-biased.</sub>", ""]
                first = False
    if charts.get("end_err_cm"):
        L += ["![End position error by condition](%s)" % charts["end_err_cm"], "",
              "*Each dot is one run's end position error (shapes mark the route, hollow dots are statistical "
              "outliers); the diamond and bar are the condition's mean with its 95% confidence range.*", ""]
    if charts.get("end_head_abs_deg"):
        L += ["![End heading error by condition](%s)" % charts["end_head_abs_deg"], "",
              "*Each dot is one run's heading difference between its end and its start, ignoring left or right; "
              "the diamond and bar are the mean with its 95% confidence range.*", ""]
    L += ["### Are the differences real", ""]
    shown = False
    for m, lab in [x for x in METRICS if x[0] in PRIMARY]:
        for sc in route_scopes:
            sel = [c for c in comps if c["scope"] == sc and c["metric"] == m and c["set"] == "all runs"]
            if not sel:
                continue
            L += ["**%s, %s**" % (lab, sc.replace("route=", "route ") if sc != "all" else "all routes"), "",
                  "| A vs B | mean A - B (95% CI) | permutation p (Holm) | Welch p (Holm) | Mann-Whitney p (Holm) | "
                  "Cohen's d | spread differs, p |",
                  "|---|---|---|---|---|---|---|"]
            for c in sel:
                L.append("| %s vs %s | %s (%s to %s) | %s (%s) | %s (%s) | %s (%s) | %s | %s |" % (
                    c["cond_a"], c["cond_b"], f(c["mean_diff"]), f(c["diff_lo"]), f(c["diff_hi"]),
                    p_txt(c.get("perm_p", float("nan"))), p_txt(c.get("perm_p_holm", float("nan"))),
                    p_txt(c["welch_p"]), p_txt(c["welch_p_holm"]), p_txt(c["mwu_p"]), p_txt(c["mwu_p_holm"]),
                    f(c["cohen_d"]), p_txt(c["bf_p"])))
            L.append("")
            if not shown:       # fine print right under the first table that uses these statistics
                L += TESTS_FINE_PRINT
            shown = True
    if not shown:
        L += ["No test yet: a comparison needs at least two conditions with two or more runs each on the same route.", ""]
    rob = [s for s in dict.fromkeys(x["scope"] for x in summ) if s.startswith("robot=")]
    if rob:
        L += ["Per robot (end position error, mean cm, all routes; secondary):", "",
              "| Robot | " + " | ".join(CONDITIONS) + " |", "|---|" + "---|" * len(CONDITIONS)]
        for sc in rob:
            cells = []
            for c in CONDITIONS:
                d = find_row_for(summ, sc, "end_err_cm", c)
                cells.append("%s (%d)" % (f(d["mean"]), d["n"]) if d else "-")
            L.append("| %s | %s |" % (sc[6:], " | ".join(cells)))
        L.append("")
    if charts.get("bump"):
        L += ["![Bump: believed against camera](%s)" % charts["bump"], "",
              "*Each dot is one bump: where the camera saw the robot at contact against where the robot believed it was; "
              "dots on the grey line are exact.*", ""]
    bs = [b for b in bumps if np.isfinite(nanf(b.get("pos_err_cm")))]
    if bs:
        d = describe([b["pos_err_cm"] for b in bs])
        L += ["Bumps with a contact frame: %d of %d. Believed-vs-camera position error mean %s cm (SD %s, "
              "median %s); detection delay reported by the robot median %s ms." % (
                  len(bs), len(bumps), f(d["mean"]), f(d["sd"]), f(d["median"]),
                  f(float(np.nanmedian([b["delay_ms"] for b in bs])), 0)), ""]
    if setup:
        L += setup["lines"]
    L += ["## Conclusion", ""]
    L += conclusion_lines(rows, summ, comps)
    L += ["", "## What's next", ""]
    L += (setup or {}).get("next", [])
    L += next_lines(rows, n_by, comps)
    L += ["", "## Two sweeps", "", "**New tests that may now be needed**", ""]
    L += ["- A mixed model (robot and block as random effects) to cross-check the permutation test once there are "
          "enough blocks.",
          "- A check that the learned C/KC corrections moved the right way (gyro scale, drift, wheel scale before and "
          "after learning, against what the camera-measured U runs say).",
          "- A repeat of a few runs with a tape measure at the far point and end, to test the camera's own error "
          "(not measured by this experiment).",
          "- A test of the calibration at the robot's height (a robot placed at known marks) before trusting sub-cm "
          "differences.",
          "- Order effects: error against run number within a robot (battery, floor wear), since conditions were "
          "shuffled in blocks rather than fully randomised."]
    L += ["", "**Possible inaccuracies in what this relied on**", ""]
    n_goal = sum(1 for r in rows if r.get("far_intent_from") == "E02 far_goal")
    n_tab = sum(1 for r in rows if r.get("far_intent_from") == "route table")
    L += ["- Intended far point: %d run(s) use the far point the robot itself aimed for (E02 far_goal, in its believed "
          "start's frame), %d use the route table (straight 100; L 30 then 30 sideways; square side 50, far point at the "
          "second corner), with the turn side read from where the camera saw the far point (a far point more than half "
          "a route length off would be misread). End error does not depend on either." % (n_goal, n_tab),
          "- The robot's centre is taken as the middle of its blob, with the robot top 4 cm above the floor and no "
          "offset to the axle (--centre-offset-cm 0): a few mm of bias that cancels in return errors but not in far errors.",
          "- Heading comes from the directions of the blob's edges, known only up to a quarter turn; the quarter is taken "
          "from the robot's own record (believed start heading after the find, believed turn since). In the simulator "
          "dry run this was within 4 deg of the truth; a believed turn more than 45 deg off would be misread by 90.",
          "- The camera's picture of the empty scene is the median of 41 pictures spread over each run (the moving robot "
          "drops out); a robot that stood still for most of its run would vanish into it (flagged: not found).",
          "- Camera calibration error (earlier fit about 0.9 cm RMS) is not separated from the robot's error; start-relative "
          "errors cancel most of it, absolute ones do not.",
          "- Bump believed poses are taken as floor coordinates when the find started from a camera fix (FIND "
          "known_before 1), else as the robot's own frame from where it was put down; a find whose record lacks "
          "known_before is read the second way.",
          "- Contact frame = where the robot stops advancing, at 25 fps (40 ms steps, about 0.2 cm at a gentle approach); "
          "the thresholds (1.5 cm/s, plateau tolerance 0.35 cm) are chosen, not derived.",
          "- Outliers by the 1.5 x IQR fence within condition and route are a chosen rule; with 5-8 runs per cell "
          "the fences are rough. The permutation test respects robot and route; Welch and Mann-Whitney treat runs "
          "as independent although the same robot, battery and learned corrections recur.",
          "- Log column names (BUMP, FIND, E02), the run list and the mark texts were checked end to end against the "
          "runner's simulator dry run (the firmware lane's model of the robots), not yet against a real session's files; "
          "the readers accept the aliases in ALIASES.",
          "- Camera times come from the runner's laptop-to-Pi clock offset (measured on the live picture) plus a fixed "
          "capture delay; an offset error of a few tenths of a second moves the start/far/end windows by that much."]
    nm = sum(1 for r in rows if "found by motion" in r.get("flags", ""))
    nfl = sum(1 for r in rows if r.get("believed_in_floor_frame") == 0)
    if nm:
        L.append("- %d run(s): the robot was first located by frame differencing, not from its believed start." % nm)
    if nfl:
        L.append("- %d run(s): the believed start is more than 20 cm or 30 deg from the camera's, so the believed "
                 "poses are probably not in the floor frame and the absolute (abs_*) errors are blank." % nfl)
    for n in list(notes) + (setup or {}).get("notes", []):
        L.append("- " + n)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as fh:
        fh.write("\n".join(L) + "\n")
    return path


def conclusion_lines(rows, summ, comps):
    out = []
    route_scopes = [s for s in dict.fromkeys(x["scope"] for x in summ) if s.startswith("route=")] or ["all"]
    for m, lab in [x for x in METRICS if x[0] in PRIMARY]:
        any_route = False
        for sc in route_scopes:
            ds = [(find_row_for(summ, sc, m, c), c) for c in CONDITIONS]
            ds = [(d, c) for d, c in ds if d and d["n"] >= 2]
            if len(ds) < 2:
                continue
            any_route = True
            ds.sort(key=lambda x: x[0]["mean"])
            best, worst = ds[0], ds[-1]
            where = sc.replace("route=", "route ") if sc != "all" else "all routes"
            s = "- %s, %s: lowest mean %s %s (n=%d), highest %s %s (n=%d)." % (
                lab, where, best[1], f(best[0]["mean"]), best[0]["n"], worst[1], f(worst[0]["mean"]), worst[0]["n"])
            pairs = [c for c in comps if c["scope"] == sc and c["metric"] == m and c["set"] == "all runs"]
            sig = [c for c in pairs if np.isfinite(c.get("perm_p_holm", float("nan"))) and c["perm_p_holm"] < 0.05]
            if sig:
                s += " Differences with Holm-adjusted permutation p < 0.05: " + ", ".join(
                    "%s vs %s (%s, d=%s)" % (c["cond_a"], c["cond_b"], f(c["mean_diff"]), f(c["cohen_d"])) for c in sig) + "."
            else:
                s += " No pair differs at Holm-adjusted permutation p < 0.05."
            out.append(s)
            flip = []
            for c in pairs:
                t = next((x for x in comps if x["scope"] == sc and x["metric"] == m and x["set"] == "without outliers"
                          and (x["cond_a"], x["cond_b"]) == (c["cond_a"], c["cond_b"])), None)
                if t and (c.get("perm_p_holm", 1) < 0.05) != (t.get("perm_p_holm", 1) < 0.05):
                    flip.append("%s vs %s" % (c["cond_a"], c["cond_b"]))
            if flip:
                out.append("  - Taking the statistical outliers out changes the verdict for: %s. Look at those runs "
                           "before relying on either." % ", ".join(flip))
        if not any_route:
            out.append("- %s: fewer than two conditions have two or more runs; no comparison yet." % lab)
    return out

def next_lines(rows, n_by, comps):
    out = []
    low = [c for c in CONDITIONS if 0 < n_by[c] < 10]
    if low:
        out.append("- Conditions with fewer than 10 runs (%s) give wide ranges; more runs there narrow them." % ", ".join(low))
    missing = [c for c in CONDITIONS if n_by[c] == 0]
    if missing:
        out.append("- No runs yet for: %s." % ", ".join(missing))
    nf = sum(1 for r in rows if r.get("flags"))
    if nf:
        out.append("- %d run(s) carry a note in runs.csv (flags column): open the track file and the video at those times." % nf)
    out.append("- Re-run this script after each batch of runs: python tools/experiment/e02_analysis.py with the same arguments.")
    return out


# =============================================================================== setup and recovery (runner records)
# The runner's own records of setup and recovery (tools/experiment/e02.py, 2026-10-22): front_checks.csv (the front
# sensor after each leg against what the map says is ahead) and e02_state.json auto_retries / help_calls /
# disagreements, plus its video marks. Older sessions have none of these: every reader returns empty.
FRONT_RANGE_CM = 50.0       # only things the map puts this near count (e02.py FRONT_CHECK_RANGE_CM)
FRONT_WITHIN_CM = 8.0       # share of readings this close to the map (e02.py FRONT_TOL_CM today)
FRONT_MIN_N = 8             # fewer readings: no suggested values (e02.py FRONT_LEARN_MIN)
FRONT_MIN_TOL_CM = 3.0      # suggested tolerance = 3 x spread, at least this (e02.py front_limits(), --front-learn)
HELP_WINDOW_S = 120.0       # an auto-retry with no help call for that robot this soon after it avoided one
MARK_RETRY = re.compile(r"^\s*e02\s+auto-retry\s+(\S+)\s+(\d+)\b", re.I)
MARK_SWEEP = re.compile(r"^\s*e02\s+sweep\s+(\S+)\s+for\s+(\S+)", re.I)
MARK_RESCUE = re.compile(r"^\s*e02\s+rescue\s+(\S+)\s+find\s+start\b", re.I)
MARK_DRIVE_OUT = re.compile(r"^\s*e02\s+drive\s+out\s+(\S+)", re.I)
MARK_HELP = re.compile(r"^\s*e02\s+help\s+([^\s:]+):", re.I)
RECOVERY_COLS = ["robot", "auto_retries", "retry_episodes", "help_calls_avoided", "sweeps", "rescue_finds",
                 "drive_outs", "help_calls", "disagreements"]


def front_sensor(session):
    """front_checks.csv rows with diff_cm (front - expected) and used (heading measured, map distance <= 50 cm), and
    the summary of the used ones: n, median, spread (MAD x 1.4826), share within 8 cm, suggested offset/tolerance."""
    path = os.path.join(session, "front_checks.csv") if session else ""
    rows = read_csv(path) if path and os.path.exists(path) else []
    diffs = []
    for r in rows:
        fr, ex = nanf(r.get("front_cm")), nanf(r.get("expected_cm"))
        hm = str(r.get("heading_measured", "")).strip() in ("1", "1.0", "True", "true")
        ok = hm and np.isfinite(fr) and np.isfinite(ex) and ex <= FRONT_RANGE_CM
        r["diff_cm"] = round(fr - ex, 2) if np.isfinite(fr) and np.isfinite(ex) else ""
        r["used"] = int(ok)
        if ok:
            diffs.append(fr - ex)
    s = dict(n_rows=len(rows), n=len(diffs), median=float("nan"), spread=float("nan"), within=float("nan"),
             offset=None, tol=None)
    if diffs:
        d = np.array(diffs, float)
        s["median"] = float(np.median(d))
        s["spread"] = float(np.median(np.abs(d - s["median"]))) * 1.4826
        s["within"] = float(np.mean(np.abs(d) <= FRONT_WITHIN_CM))
        if len(d) >= FRONT_MIN_N:
            s["offset"] = round(s["median"], 1)
            s["tol"] = round(max(FRONT_MIN_TOL_CM, 3.0 * s["spread"]), 1)
    return rows, s


def _state(session):
    sp = os.path.join(session, "e02_state.json") if session else ""
    if not sp or not os.path.exists(sp):
        return {}
    try:
        with open(sp) as fh:
            return json.load(fh)
    except ValueError:
        return {}


def recovery(session, events=None):
    """Per robot: auto-retries, finder sweeps (counted for the robot looked for), rescue finds (attempts), drive-outs,
    help calls, camera/wheels disagreements, and help calls avoided. Auto-retries and help calls come from
    e02_state.json (laptop clock) when it has them, else both from the video marks (camera clock): never mixed.
    Retries of one robot are one episode until a help call or a gap over 2 min; an episode avoided a help call when
    no help call for that robot follows its last retry within 2 min. events: events.csv rows (t_wall, text); without
    them the state's own copy of the marks (notes) is read."""
    st = _state(session)
    if events is None:
        events = [dict(t_wall=n[0], text=n[1]) for n in (st.get("notes") or []) if len(n) >= 2]
    marks = dict(retry=[], sweep=[], rescue=[], drive_out=[], help=[])
    for e in events:
        text, t = str(e.get("text", "")), nanf(e.get("t_wall"))
        for key, rx, grp in (("retry", MARK_RETRY, 1), ("sweep", MARK_SWEEP, 2), ("rescue", MARK_RESCUE, 1),
                             ("drive_out", MARK_DRIVE_OUT, 1), ("help", MARK_HELP, 1)):
            m = rx.match(text)
            if m:
                marks[key].append((t, m.group(grp)))
                break
    if "auto_retries" in st or "help_calls" in st:
        retries = [(nanf(x[0]), str(x[1])) for x in st.get("auto_retries") or [] if len(x) >= 2]
        helps = [(nanf(x[0]), str(x[1])) for x in st.get("help_calls") or [] if len(x) >= 2]
        source = "e02_state.json"
    else:
        retries, helps, source = marks["retry"], marks["help"], "video marks"
    disagree = [str(x[1]) for x in st.get("disagreements") or [] if len(x) >= 2]
    robots = sorted({r for _, r in retries + helps + marks["sweep"] + marks["rescue"] + marks["drive_out"]}
                    | set(disagree), key=lambda r: (len(r), r))
    out = []
    for rb in robots:
        rt = sorted(t for t, r in retries if r == rb)
        hp = sorted(t for t, r in helps if r == rb)
        episodes = []
        for t in rt:
            if episodes and t - episodes[-1][-1] <= HELP_WINDOW_S and \
                    not any(episodes[-1][-1] < h <= t for h in hp):
                episodes[-1].append(t)
            else:
                episodes.append([t])
        avoided = sum(1 for ep in episodes if not any(ep[-1] <= h <= ep[-1] + HELP_WINDOW_S for h in hp))
        out.append(dict(robot=rb, auto_retries=len(rt), retry_episodes=len(episodes), help_calls_avoided=avoided,
                        sweeps=sum(1 for _, r in marks["sweep"] if r == rb),
                        rescue_finds=sum(1 for _, r in marks["rescue"] if r == rb),
                        drive_outs=sum(1 for _, r in marks["drive_out"] if r == rb),
                        help_calls=len(hp), disagreements=disagree.count(rb)))
    if out:
        tot = dict(robot="all")
        for c in RECOVERY_COLS[1:]:
            tot[c] = sum(r[c] for r in out)
        out.append(tot)
    return out, source


def setup_recovery(session, out, events=None):
    """Writes front_sensor.csv and recovery.csv in out (when there is anything to write) and returns
    dict(lines=report lines for the "Setup and recovery" part of Data, notes=for the inaccuracies sweep, next=for
    What's next, front=summary, recovery=per-robot rows)."""
    fr_rows, fs = front_sensor(session)
    rec, source = recovery(session, events)
    if fr_rows:
        write_csv(os.path.join(out, "front_sensor.csv"), fr_rows,
                  ["t", "robot", "leg", "value", "x_cm", "y_cm", "heading_deg", "heading_measured", "front_cm",
                   "expected_cm", "what", "diff_cm", "used"])
    if rec:
        write_csv(os.path.join(out, "recovery.csv"), rec, RECOVERY_COLS)
    L, notes, nxt = ["### Setup and recovery", ""], [], []
    if not fr_rows:
        L += ["Front sensor: no front_checks.csv in this session (the runner records it from 2026-10-22).", ""]
    elif not fs["n"]:
        L += ["Front sensor: %d readings, none with a camera-measured heading and the map's distance within %.0f cm." % (
            fs["n_rows"], FRONT_RANGE_CM), ""]
    else:
        L += ["Front sensor against the map (%d of %d readings: heading measured by the camera, map distance within "
              "%.0f cm): the sensor reads %s cm %s than the map (median), spread %s cm, %.0f%% within %.0f cm. "
              "Per reading: front_sensor.csv." % (
                  fs["n"], fs["n_rows"], FRONT_RANGE_CM, f(abs(fs["median"]), 1),
                  "more" if fs["median"] >= 0 else "less", f(fs["spread"], 1), 100 * fs["within"], FRONT_WITHIN_CM), "",
              "<sub>Spread: the median distance of the readings from their median, x 1.4826 (MAD x 1.4826), which "
              "equals the SD for bell-shaped data but is not pulled by a few wild readings.</sub>", ""]
        if fs["offset"] is not None:
            L += ["Suggested for e02.py: **FRONT_OFFSET_CM = %.1f**, **FRONT_TOL_CM = %.1f** (offset = the median; "
                  "tolerance = 3 x spread, at least %.0f cm: the rule --front-learn uses)." % (
                      fs["offset"], fs["tol"], FRONT_MIN_TOL_CM), ""]
            nxt.append("- Set FRONT_OFFSET_CM = %.1f and FRONT_TOL_CM = %.1f in tools/experiment/e02.py (from %d front "
                       "sensor readings), if this was a real session." % (fs["offset"], fs["tol"], fs["n"]))
        else:
            L += ["Too few readings (%d; %d needed) to suggest FRONT_OFFSET_CM / FRONT_TOL_CM." % (fs["n"], FRONT_MIN_N), ""]
        notes.append("Front sensor vs map: the map distance comes from the camera's position and heading, so camera "
                     "error is inside the spread; the offset and tolerance suggested are a chosen rule (median, 3 x "
                     "spread), not derived from a miss-rate target.")
    if not rec:
        L += ["Recovery: none recorded (no auto-retries, sweeps, rescue finds, drive-outs or help calls).", ""]
    else:
        L += ["Recovery per robot (auto-retries and help calls from %s; recovery.csv):" % source, "",
              "| Robot | auto-retries (episodes) | help calls avoided | sweeps for it | rescue finds | drive-outs | "
              "help calls |", "|---|---|---|---|---|---|---|"]
        for r in rec:
            L.append("| %s | %d (%d) | %d | %d | %d | %d | %d |" % (
                r["robot"], r["auto_retries"], r["retry_episodes"], r["help_calls_avoided"], r["sweeps"],
                r["rescue_finds"], r["drive_outs"], r["help_calls"]))
        L += ["", "A help call is avoided when a robot's retries end with no help call for it within 2 minutes.", ""]
        notes.append("Help calls avoided: retries of one robot within 2 minutes of each other are one episode, and "
                     "an episode counts as avoided when no help call for that robot follows within 2 minutes; a "
                     "session that ended (or the robot put aside) in that window would also count. Sweeps are counted "
                     "for the robot looked for, rescue finds per attempt (up to 2 per rescue).")
    return dict(lines=L, notes=notes, next=nxt, front=fs, recovery=rec)


# =============================================================================== main
def resolve_recording(path):
    """The overhead camera's folder (the one with frames.csv) from: that folder itself, a camrec.py two-camera session
    folder (session.json: the camera whose role is overhead, else cam0), or a folder holding one of those."""
    if not path:
        return None
    if os.path.exists(os.path.join(path, "frames.csv")):
        return path
    sj = os.path.join(path, "session.json")
    if os.path.exists(sj):
        try:
            cams = json.load(open(sj)).get("cameras") or []
        except ValueError:
            cams = []
        for c in sorted(cams, key=lambda c: (str(c.get("role") or "") != "overhead", c.get("index", 0))):
            p = os.path.join(path, c.get("folder") or "cam%d" % c.get("index", 0))
            if os.path.exists(os.path.join(p, "frames.csv")):
                return p
    for sub in ("cam0", "recording"):
        p = resolve_recording(os.path.join(path, sub)) if os.path.isdir(os.path.join(path, sub)) else None
        if p:
            return p
    subs = sorted(d for d in (os.path.join(path, x) for x in os.listdir(path)) if os.path.isdir(d)) \
        if os.path.isdir(path) else []
    for d in subs:
        if os.path.exists(os.path.join(d, "frames.csv")) or os.path.exists(os.path.join(d, "session.json")):
            return resolve_recording(d)
    return None


def session_inputs(session, recording=None, calib=None):
    """Everything the analysis needs, from the runner's session folder (logs/e02-<date-time>[-dryrun]/):
    e02_runs.csv (times on the camera's clock), robot-logs-*/ (BUMP, FIND, E02, clock checks), the floor calibration
    the runner used (e02_state.json, else cameras/overhead.json in the folder, else the simulator's true camera),
    the laptop-to-Pi clock offset (e02_state.json setup.pi_offset_s: Pi time = laptop time - pi_offset_s), and the
    overhead recording (--recording, else <session>/recording or a dry run's sim/recording)."""
    import glob
    if not os.path.isdir(session):
        raise SystemExit("No session folder %s" % session)
    st = {}
    sp = os.path.join(session, "e02_state.json")
    if os.path.exists(sp):
        with open(sp) as f:
            st = json.load(f)
    runs = os.path.join(session, "e02_runs.csv")
    if not os.path.exists(runs):
        raise SystemExit("%s has no e02_runs.csv (the runner writes it after every run)." % session)
    logs = sorted(glob.glob(os.path.join(session, "robot-logs-*")), key=lambda p: (len(p), p))
    rec = resolve_recording(recording) if recording else None
    if recording and rec is None:
        raise SystemExit("No overhead recording (frames.csv + video.avi) in %s" % recording)
    if rec is None:
        for c in (os.path.join(session, "recording"), os.path.join(session, "sim", "recording")):
            rec = resolve_recording(c) if os.path.isdir(c) else None
            if rec:
                break
    if rec is None:
        raise SystemExit("Where is the overhead recording? Copy the Pi's recording folder (~/recordings/<date-time>_E02) "
                         "into %s as a folder named recording, or give --recording <folder>." % session)
    cal = calib
    if not cal:
        cands = [st.get("calibration") or "", os.path.join(session, "cameras", "overhead.json"),
                 os.path.join(session, "sim", "camera_model.json")]
        if st.get("calibration"):        # a laptop path moved with the folder: same name inside the session
            cands.append(os.path.join(session, "cameras", os.path.basename(st["calibration"].replace("\\", "/"))))
        cal = next((c for c in cands if c and os.path.exists(c)), None)
    if not cal:
        raise SystemExit("No floor calibration found (e02_state.json says %r). Give --calib <the camera json the runner "
                         "used, e.g. the studio's cameras/overhead.json>." % st.get("calibration"))
    po = (st.get("setup") or {}).get("pi_offset_s")
    vc = dict(offset_s=-float(po), source="e02_state.json setup.pi_offset_s") if po is not None else None
    return dict(recording=rec, logs=logs, runs=runs, calib=cal, vc=vc, out=os.path.join(session, "analysis"))


def check_coverage(video, runlist):
    """Notes when the run times fall outside the recording (wrong recording, or the clock offset is off)."""
    if not len(video.t):
        return ["The recording has no frames."]
    lo, hi = float(np.nanmin(video.t[:video.n])), float(np.nanmax(video.t[:video.n]))
    ts = [r["t_start"] for r in runlist if np.isfinite(r["t_start"])]
    out = [t for t in ts if t < lo or t > hi]
    if ts and len(out) == len(ts):
        raise SystemExit("None of the %d runs' times fall inside this recording (%.0f to %.0f s on the camera clock; runs "
                         "%.0f to %.0f): wrong recording, or the clock offset is wrong." % (len(ts), lo, hi, min(ts), max(ts)))
    return ["%d of %d runs fall outside the recording's time span and could not be measured." % (len(out), len(ts))] \
        if out else []


def analyse(recording, logs, runs, calib, out, background=None, bump_frame="auto", centre_offset_cm=0.0,
            routes=None, robot_px=23.0, only=None, quiet=False, vc=None, session=None):
    """Everything in one call: returns dict(rows, bumps, summary, comparisons, charts, setup). vc: the clock mapping used
    when the recording has no video_clock.json (session_inputs makes one from the runner's Pi clock offset). session:
    the runner's session folder, for its setup and recovery records (setup_recovery())."""
    import cv2
    cam = load_camera(calib) if calib else None
    video = Video(recording, fix_dir=out)
    vc = video_clock(recording) or vc
    marks = parse_marks(video.events)
    runlist = load_runs(runs, marks, vc)
    if only:
        runlist = [r for r in runlist if r["run_id"] in set(only)]
    cover_notes = check_coverage(video, runlist)
    lg = Logs(logs if isinstance(logs, (list, tuple)) else ([logs] if logs else []))
    bg = cv2.imread(background, 0) if background else None
    S = Settings(cam, bg, centre_offset_cm, bump_frame, routes, robot_px)
    S.vc = vc
    # runs whose list has no robot_run: join to the E02 records by order within the robot
    for rb in {r["robot"] for r in runlist if r["robot"]}:
        mine = sorted([r for r in runlist if r["robot"] == rb], key=lambda r: nanf(r["t_start"]))
        recs = sorted([x for x in lg.e02 if str(pick(x, ["robot"], "")) == rb],
                      key=lambda x: nanf(pick(x, ["ms_start", "ms", "t_recv"], 0)))
        if mine and all(r.get("robot_run") is None for r in mine) and len(recs) == len(mine):
            for r, x in zip(mine, recs):
                r["robot_run"] = pick(x, RECORD_RUN)
    rows, bumps = [], []
    os.makedirs(os.path.join(out, "tracks"), exist_ok=True)
    for run in runlist:
        row, brs, trk = measure_run(video, run, lg, S)
        rows.append(row)
        bumps.extend(brs)
        if trk:
            write_csv(os.path.join(out, "tracks", "%s.csv" % run["run_id"]), trk)
        if not quiet:
            print("run %s %-6s %-3s %-8s end %s cm %s deg %s" % (
                run["run_id"], run["robot"], run["condition"], run["route"], f(row.get("end_err_cm")),
                f(row.get("end_err_deg"), 1), row.get("flags", "")))
    flag_runs(rows)
    summ, comps = summarise(rows)
    write_csv(os.path.join(out, "runs.csv"), rows)
    write_csv(os.path.join(out, "bumps.csv"), bumps)
    write_csv(os.path.join(out, "summary.csv"), summ)
    write_csv(os.path.join(out, "comparisons.csv"), comps)
    charts = {}
    for m, lab in METRICS[:2]:
        p = spread_chart(rows, m, lab, os.path.join(out, "charts", m + ".png"))
        if p:
            charts[m] = os.path.join("charts", m + ".png")
    p = bump_chart(bumps, os.path.join(out, "charts", "bump_believed_vs_camera.png"))
    if p:
        charts["bump"] = os.path.join("charts", "bump_believed_vs_camera.png")
    notes = list(cover_notes)
    if getattr(video, "reindexed", None):
        notes.append("The recording's video had no index (not stopped cleanly); a copy with an index was made "
                     "(%s) without re-encoding. Frames past the last complete one are missing." % video.reindexed)
    nmk = sum(1 for r in runlist if r.get("time_source"))
    if nmk:
        notes.append("%d run(s) took some of their times from the video marks (the run list had none); a mark is the "
                     "moment the laptop sent it, up to a few tenths of a second off the robot's own moment." % nmk)
    if cam is None:
        notes.append("No camera calibration was given: only the picture-only return-to-start measure is valid.")
    if not vc:
        notes.append("No laptop-to-camera clock offset (no video_clock.json, no pi_offset_s): robot times were not mapped "
                     "to the camera clock, so bumps were matched by order, not time.")
    setup = setup_recovery(session, out, video.events or None) if session else None
    write_report(os.path.join(out, "report.md"), rows, bumps, summ, comps, charts, notes, setup=setup)
    return dict(rows=rows, bumps=bumps, summary=summ, comparisons=comps, charts=charts, setup=setup)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--session", help="the runner's session folder logs/e02-<date-time>: reads e02_runs.csv, "
                                      "robot-logs-*, the calibration and clock offset from it; --recording defaults to "
                                      "<session>/recording, --out to <session>/analysis")
    ap.add_argument("--recording", help="overhead camera folder (video.avi, frames.csv, events.csv), or the Pi's "
                                        "two-camera recording folder (the overhead one is picked)")
    ap.add_argument("--logs", nargs="*", default=[], help="console session folder(s) with bump.csv, find.csv, e02.csv")
    ap.add_argument("--runs", help="the runner's run list (csv)")
    ap.add_argument("--calib", help="camera model json or floor_calibration.json")
    ap.add_argument("--out")
    ap.add_argument("--background", help="picture of the empty board (optional)")
    ap.add_argument("--bump-frame", choices=("auto", "own", "floor"), default="auto",
                    help="frame of the BUMP believed poses: auto = floor when the find started from a camera fix "
                         "(FIND known_before 1), else the robot's own frame from where it was put down")
    ap.add_argument("--centre-offset-cm", type=float, default=0.0, help="blob centre ahead of the axle, cm")
    ap.add_argument("--routes-json", help='{"L": [dx, dy, dth], ...} far-point offsets overriding the defaults')
    ap.add_argument("--robot-px", type=float, default=23.0, help="robot short side in px when there is no calibration")
    ap.add_argument("--only", nargs="*", help="run ids to analyse")
    a = ap.parse_args(argv)
    routes = json.load(open(a.routes_json)) if a.routes_json else None
    vc = None
    if a.session:
        si = session_inputs(a.session, a.recording, a.calib)
        a.recording, a.calib = si["recording"], si["calib"]
        a.logs = a.logs or si["logs"]
        a.runs = a.runs or si["runs"]
        a.out = a.out or si["out"]
        vc = si["vc"]
        print("session %s\n  recording %s\n  calibration %s\n  robot logs %s\n  writing to %s" % (
            a.session, a.recording, a.calib, ", ".join(os.path.basename(p) for p in a.logs) or "none", a.out))
    else:
        a.recording = resolve_recording(a.recording) or a.recording
    if not a.recording or not a.out:
        ap.error("give --session <logs/e02-...>, or --recording and --out")
    res = analyse(a.recording, a.logs, a.runs, a.calib, a.out, a.background, a.bump_frame, a.centre_offset_cm, routes,
                  a.robot_px, a.only, vc=vc, session=a.session)
    print("wrote %s" % os.path.join(a.out, "report.md"))
    return res


if __name__ == "__main__":
    main()
