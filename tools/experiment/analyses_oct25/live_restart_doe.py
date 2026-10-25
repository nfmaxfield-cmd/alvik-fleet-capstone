"""Logs-only DOE: the Live lab filter's restart threshold (docs/build-reports/e02-oct25-real-fusion.md, part 2).

The Live lab (tools/experiment/e02_live.py, display only: it never steers the runner) restarts its filter from the
camera when a camera position is more than a squared distance of 60 (about 7.7 sigma, chosen) from where the filter
expected the robot. After the units fix, 300-525 restarts per real session remain. This replays every real session
through the Live lab as the replay viewer does (RAW rows at their arrival time, every track.csv camera sample) with:

  factor T     the restart threshold (squared Mahalanobis distance): 15, 30, 60 (today), 120, 240, 480, never
  factor S     how the filter starts (and what a restart keeps):
                 today    as e02_live.LiveLab: a new filter whenever a RAW row comes from another robot or another
                          run, started at that row's x_cm / y_cm / th_deg: the Alvik LIBRARY's own odometry, in its
                          own frame (not the floor's: often 100+ cm and any angle from where the camera sees it); a
                          restart keeps the heading
                 position one filter per robot (another robot's RAW rows no longer restart it), started at a new
                          run at the camera's latest position of that robot (seen within 1 s; until then no filter),
                          with the library heading as today; a restart keeps the heading
                 camera   as position, but the heading is marked unknown (sd 90 deg) and taken from the camera's direction of travel as soon
                          as the robot drives straight >= HFM_MOVE_CM (one way, |w| < 8 deg/s); after a restart, the same
                 floor    as camera, but the heading at the start is the robot's floor heading (the library heading plus
                          the offset between the firmware's floor pose and the library's at the run's start: e02.csv
                          start_th, find.csv before_th): what the runner could hand the lab. Runs without: as camera

Responses, per session and summed:
  restarts per hour of lab time (time the lab had a filter running for a robot sending RAW rows)
  position error against the camera: the filter's prediction just before each camera sample (median, 95th pct)
  time diverged: share of lab time whose prediction error was over DIVERGED_CM
  leave-camera-out: a second pass with the camera withheld for GAP_S every GAP_EVERY_S; the error at each gap's end

Usage: nice -n 10 python3 live_restart_doe.py [--sessions a,b]   (about 20-40 min on 2 cores)
Writes out/live_restart_doe.csv (per session x setting) and out/live_restart_doe.txt.
"""
import argparse
import csv
import glob
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(HERE)
TOOLS = os.path.dirname(EXP)
for p in (os.path.join(TOOLS, "fusion"), TOOLS, EXP, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import e02_live as LIVE    # noqa: E402
import ekf as EKF          # noqa: E402
import raw_units as RU     # noqa: E402
import real_fusion as RF   # noqa: E402  (session list and folders)

THRESHOLDS = (15, 30, 60, 120, 240, float("inf"))
GAP_THRESHOLDS = (60,)     # the leave-camera-out pass is run for these only (time: each pass is ~1 min per hour of log)
HEADINGS = ("today", "position", "camera", "floor")   # the start rule (factor S)
DIVERGED_CM = 5.0     # chosen: about the full fusion's 95th percentile against the camera on real runs
GAP_S = 2.0           # leave-camera-out pass: camera withheld this long ...
GAP_EVERY_S = 20.0    # ... every this long (chosen: short gaps, as a dropped camera feed would be)
HFM_MOVE_CM = 5.0     # camera heading: straight drive of at least this far (chosen; 0.6 cm camera noise -> ~10 deg)
HFM_MAX_S = 2.0       # ... within this long
D2R = math.pi / 180.0


class Lab(LIVE.LiveLabBefore):
    """The Live lab with the restart threshold and the start rule as parameters. With T=60 and start="today" it does
    exactly what e02_live.LiveLabBefore (the Live lab until 2026-10-25) does (checked: 301 restarts on stage2, as the units-fix report)."""

    def __init__(self, threshold=60.0, start="today", floor_offsets=None):
        super().__init__()
        self.T = threshold
        self.start = start
        self.offsets = floor_offsets or {}
        self.filters = {}         # robot -> (filter, run) kept while another robot's RAW rows arrive
        self.cam_last = {}        # robot -> (t, x, y): the camera's latest position of each robot
        self.hist = []            # (t, x, y) camera positions while the heading is unknown
        self.raw_hist = []        # (t, v, straight) recent RAW of the lab's robot
        self.hdg_known = True
        self.n_hfm = 0
        self.n_init = 0

    def on_raw(self, robot, rec, t):
        if self.start == "today":
            run = rec.get("run")
            if self.f is None or robot != self.robot or run != self.run:
                self.n_init += 1
            return super().on_raw(robot, rec, t)
        v, w = LIVE._num(RU.speed_cms(rec.get("v_cms"))), LIVE._num(RU.turn_dps(rec.get("w_dps")))
        th = LIVE._num(rec.get("th_deg"))
        run = rec.get("run")
        if robot != self.robot and self.f is not None:
            self.filters[self.robot] = (self.f, self.run, self.hdg_known)
        if robot != self.robot:
            f_run = self.filters.get(robot)
            if f_run is not None:
                self.f, self.run, self.hdg_known = f_run
            else:
                self.f, self.run = None, None
            self.robot = robot
            self.hist, self.raw_hist = [], []
        if self.f is None or run != self.run:
            c = self.cam_last.get(robot)
            if c is None or t - c[0] > 1.0 or th is None:
                self.f = None            # wait for the camera to see this robot
                return
            h = th + self.offsets[(robot, run)] if self.start == "floor" and (robot, run) in self.offsets else th
            rec0 = dict(rec, x_cm=str(c[1]), y_cm=str(c[2]), th_deg=str(h))
            self.f = None
            self.run = None
            self.n_init += 1
            super().on_raw(robot, rec0, t)
            self.hist = []
            self.hdg_known = True
            if self.start == "camera" or (self.start == "floor" and (robot, run) not in self.offsets):
                self._unknown_heading()
            return
        super().on_raw(robot, rec, t)
        if v is not None and w is not None:
            self.raw_hist.append((t, v, abs(w) < 8.0))
            while self.raw_hist and t - self.raw_hist[0][0] > HFM_MAX_S + 0.5:
                self.raw_hist.pop(0)

    def _unknown_heading(self):
        f = self.f
        f.P[2, :] = 0
        f.P[:, 2] = 0
        f.P[2, 2] = (90 * D2R) ** 2
        self.hdg_known = False

    def on_camera(self, robot, t, x, y):
        self.cam_last[robot] = (t, x, y)
        with self.lock:
            f = self.f
            if f is None or robot != self.robot or t < f.t - 1.0:
                return
            f.predict(max(t, f.t))
            before = f.s[:2].copy()
            if f.cam_d2((x, y)) > self.T:
                f.reset_position(x, y, sigma_xy=1.0)
                self.resets += 1
                self.last_reset = t
                if self.start in ("camera", "floor"):
                    self._unknown_heading()
                    self.hist = []
            f.cam_xy((x, y))
            d = float(math.hypot(x - before[0], y - before[1]))
            self.events.append((t, "camera", f.last_trust, d))
            self.last_cam = dict(t=t, d_cm=d, trust=f.last_trust)
            self.n_cam += 1
            if self.start in ("camera", "floor") and not self.hdg_known:
                self._try_heading(t, x, y)

    def _try_heading(self, t, x, y):
        self.hist.append((t, x, y))
        while self.hist and t - self.hist[0][0] > HFM_MAX_S:
            self.hist.pop(0)
        for t0, x0, y0 in self.hist:
            d = math.hypot(x - x0, y - y0)
            if d < HFM_MOVE_CM:
                break
            rows = [r for r in self.raw_hist if t0 <= r[0] <= t]
            if len(rows) < 5:
                break
            sgn = np.sign(np.mean([r[1] for r in rows]))
            if not all(np.sign(r[1]) == sgn and abs(r[1]) >= 1.0 and r[2] for r in rows):
                break
            h = math.atan2(y - y0, x - x0) + (math.pi if sgn < 0 else 0.0)
            self.f.reset_position(x, y, sigma_xy=1.0, th=EKF.wrap(h), sigma_th=10 * D2R)
            self.hdg_known = True
            self.n_hfm += 1
            break


# =============================================================================== the sessions
def load(name):
    d = RF.sdir(name)
    raw = []
    for p in sorted(glob.glob(os.path.join(d, "robot-logs-*", "raw.csv"))):
        for r in RF.csv_rows(p):
            t = RF.num(r.get("t_recv"))
            if t is not None:
                raw.append((t, r.get("robot", ""), {k: r.get(k) for k in ("run", "v_cms", "w_dps", "gz", "x_cm", "y_cm",
                                                                             "th_deg", "ms")}))
    raw.sort(key=lambda q: q[0])
    track = []
    for r in RF.csv_rows(os.path.join(d, "track.csv")):
        t, x, y = RF.num(r.get("t")), RF.num(r.get("x_cm")), RF.num(r.get("y_cm"))
        if t is not None and x is not None and y is not None:
            track.append((t, r.get("robot", ""), x, y))
    track.sort(key=lambda q: q[0])
    return raw, track


def floor_offsets(name, raw):
    """(robot, run) -> floor heading minus the library's th_deg, at a moment both are known."""
    lib = {}
    for t, robot, rec in raw:
        ms, th = RF.num(rec.get("ms")), RF.num(rec.get("th_deg"))
        if ms is not None and th is not None:
            lib.setdefault((robot, rec.get("run")), []).append((ms, th))
    out = {}

    def lib_at(key, ms):
        rows = lib.get(key)
        if not rows:
            return None
        best = min(rows, key=lambda q: abs(q[0] - ms))
        return best[1] if abs(best[0] - ms) < 200 else None

    for r in RF.logs(name, "e02.csv"):
        key, ms, th = (r["robot"], r["run"]), RF.num(r.get("ms_start")), RF.num(r.get("start_th"))
        if ms is not None and th is not None:
            lt = lib_at(key, ms)
            if lt is not None:
                # the offset at the route's start; the run's RAW starts ~10 s earlier, on the same library frame
                out[key] = th - lt
    for r in RF.logs(name, "find.csv"):
        key, th = (r["robot"], r["run"]), RF.num(r.get("before_th"))
        rows = lib.get(key)
        if th is not None and rows and (r.get("known_before") or "1") not in ("0", ""):
            out[key] = th - rows[0][1]
    return out


def replay(raw, track, lab, gap=False):
    """Feed the lab; return the responses."""
    i = j = 0
    errs, div_t, lab_t = [], 0.0, 0.0
    gap_errs = []
    last_raw_t = {}
    last_cam_t = None
    t_first = track[0][0] if track else 0.0
    in_gap_prev = False
    gap_start_pos = None
    while i < len(raw) or j < len(track):
        if j >= len(track) or (i < len(raw) and raw[i][0] <= track[j][0]):
            t, robot, rec = raw[i]
            i += 1
            if lab.f is not None and robot == lab.robot:
                lt = last_raw_t.get(robot)
                if lt is not None:
                    lab_t += min(0.5, t - lt)
            last_raw_t[robot] = t
            try:
                lab.on_raw(robot, rec, t)
            except Exception:
                pass
            continue
        t, robot, x, y = track[j]
        j += 1
        f = lab.f
        live = f is not None and robot == lab.robot and t >= f.t - 1.0
        in_gap = gap and ((t - t_first) % GAP_EVERY_S) < GAP_S
        if live:
            s, _ = f.predicted(max(t, f.t))
            e = math.hypot(s[0] - x, s[1] - y)
            if gap:
                if in_gap_prev and not in_gap:
                    gap_errs.append(e)
            else:
                errs.append(e)
                if last_cam_t is not None and e > DIVERGED_CM:
                    div_t += min(0.5, t - last_cam_t)
        in_gap_prev = in_gap
        last_cam_t = t
        if in_gap:
            continue
        try:
            lab.on_camera(robot, t, x, y)
        except Exception:
            pass
    return dict(restarts=lab.resets, lab_h=lab_t / 3600.0, errs=errs, div_s=div_t, gap_errs=gap_errs,
                n_hfm=getattr(lab, "n_hfm", 0), n_init=getattr(lab, "n_init", 0))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--sessions", default=",".join(RF.ALL))
    ap.add_argument("--out", default=os.path.join(HERE, "out"))
    ap.add_argument("--tag", default="", help="suffix for the output files (parallel halves)")
    a = ap.parse_args(argv)
    lines = []

    def log(s):
        print(s, flush=True)
        lines.append(s)

    rows = []
    for name in a.sessions.split(","):
        if not os.path.isdir(RF.sdir(name)):
            continue
        t0 = time.time()
        raw, track = load(name)
        if not raw or not track:
            log("%s: no RAW or no camera, skipped" % name)
            continue
        offs = floor_offsets(name, raw)
        for H in HEADINGS:
            for T in THRESHOLDS:
                r = replay(raw, track, Lab(T, H, offs))
                g = replay(raw, track, Lab(T, H, offs), gap=True) if T in GAP_THRESHOLDS else dict(gap_errs=[])
                e = np.array(r["errs"]) if r["errs"] else np.array([np.nan])
                ge = np.array(g["gap_errs"]) if g["gap_errs"] else np.array([np.nan])
                rows.append(dict(session=name, start=H, T=("never" if math.isinf(T) else int(T)),
                                 restarts=r["restarts"], lab_h=round(r["lab_h"], 4),
                                 restarts_per_h=round(r["restarts"] / r["lab_h"], 1) if r["lab_h"] > 0 else "",
                                 n_cam=len(r["errs"]), err_med=round(float(np.nanmedian(e)), 3),
                                 err_p95=round(float(np.nanpercentile(e, 95)), 2),
                                 diverged_share=round(r["div_s"] / (r["lab_h"] * 3600), 4) if r["lab_h"] > 0 else "",
                                 n_gaps=len(g["gap_errs"]), gap_err_med=round(float(np.nanmedian(ge)), 2),
                                 gap_err_p90=round(float(np.nanpercentile(ge, 90)), 2),
                                 camera_headings=r["n_hfm"], filter_starts=r["n_init"],
                                 floor_offsets=len(offs)))
        log("%s: %d RAW, %d camera, %d floor offsets, %.0f s" % (name, len(raw), len(track), len(offs),
                                                                 time.time() - t0))
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "live_restart_doe%s.csv" % a.tag), "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)
    # pooled
    log("\nPooled over sessions (restarts / lab hours; errors pooled by camera-sample-weighted median of sessions)")
    log("%-8s %-6s %9s %8s %9s %9s %9s %9s" % ("start", "T", "restarts", "per h", "err med", "err p95",
                                              "diverged", "gap p50"))
    for H in HEADINGS:
        for T in THRESHOLDS:
            tl = "never" if math.isinf(T) else int(T)
            sel = [r for r in rows if r["start"] == H and r["T"] == tl]
            n = sum(r["restarts"] for r in sel)
            h = sum(r["lab_h"] for r in sel)
            wts = np.array([r["n_cam"] for r in sel], float)
            med = float(np.average([r["err_med"] for r in sel], weights=wts)) if wts.sum() else float("nan")
            p95 = float(np.average([r["err_p95"] for r in sel], weights=wts)) if wts.sum() else float("nan")
            dv = sum(r["diverged_share"] * r["lab_h"] for r in sel if r["diverged_share"] != "") / h if h else float("nan")
            gw = np.array([r["n_gaps"] for r in sel], float)
            gm = float(np.average([r["gap_err_med"] for r in sel if r["n_gaps"]],
                                  weights=gw[gw > 0])) if gw.sum() else float("nan")
            log("%-8s %-6s %9d %8.0f %9.2f %9.2f %8.1f%% %9.2f" % (H, tl, n, n / h if h else float("nan"), med, p95,
                                                                  100 * dv, gm))
    with open(os.path.join(a.out, "live_restart_doe%s.txt" % a.tag), "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
