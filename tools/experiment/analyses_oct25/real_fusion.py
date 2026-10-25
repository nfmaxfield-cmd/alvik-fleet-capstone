"""The first offline fusion on REAL E02 sessions (docs/build-reports/e02-oct25-real-fusion.md).

Logs only: reads the session folders, never talks to robots or the camera, changes no runner or firmware code.
Now that the RAW speed columns are read right (tools/raw_units.py, 2026-10-25), tools/fusion/engine.py can run on the
robots' real logs. For every robot run that drives (finds and E02 routes) it runs:

  A   wheels + gyro only (engine sensors wheel_v, wheel_w, gyro), from the robot's own start belief;
  B   wheels + gyro + camera, with the camera blanked for gaps of L seconds (engine cam_off), L in GAPS;
  Bf  wheels + gyro + camera, camera never blanked (how closely the fusion follows the camera when it sees);
  Bc  wheels + gyro + camera, camera blanked from 10 s into the run to its end (a route driven blind after its
      first leg).

The reference is the camera itself (track.csv, the runner's camera positions of the moving robot; "held" rows left out:
they are a held robot's remembered spot, not a picture). So the camera is scored leave-out: in each gap the fusion
does not see the camera, and its estimate at the gap's end is compared with what the camera saw there. In the same
gaps, each source alone, anchored at the camera's last position before the gap:
  camera alone      the last position the camera saw (hold), and that position moved on at the camera's speed over
                    the last second (constant velocity);
  wheels + gyro     the camera's position at the gap start + A's displacement over the gap (A's own heading);
  robot's own pose  the camera's position at the gap start + the firmware's own displacement over the gap (routes
                    only: its pose re-computed from RAW with the route's corrections, oct25_common.reintegrate).

Heading (docs/build-reports/e02-oct25-heading.md): a run that starts from a wrong heading (the runner's placeholder or
echo written in by a fix) makes every dead-reckoned source wrong from the start. Each run's start heading is checked
against the camera's direction of travel on its first straight stretch of >= MIN_MOVE_CM (A's heading, which is the
start belief plus the gyro): ok (within BIG), wrong (BIG or more), unmeasured (no such stretch in the first
HDG_WINDOW_S). Headline numbers use ok runs; the others are reported apart.

Identity is taken from track.csv (the runner's tracker names the moving robot), so each robot's filter is given only
its own camera rows; the engine's possible-area gate still rejects camera rows that disagree with it.

Usage: nice -n 10 python3 real_fusion.py [--sessions a,b] (about 10-20 min on 2 cores)
Writes out/real_fusion_gaps.csv, out/real_fusion_runs.csv, out/real_fusion_routes.csv, out/real_fusion.txt.
"""
import argparse
import bisect
import csv
import glob
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(os.path.dirname(HERE))
for p in (os.path.join(TOOLS, "fusion"), TOOLS, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import oct25_common as C   # noqa: E402
import unc_fit as U        # noqa: E402,F401  (adds the 2026-10-22 sessions to C.SESSIONS)
import runs as RN          # noqa: E402
import engine as E         # noqa: E402

for k, v in {"oct21-2222": "oct25dl/e02-setup-oct21/x/e02-20261021-222237",
             "oct21-2236": "oct25dl/e02-setup-oct21/x/e02-20261021-223606",
             "oct21-2245": "oct25dl/e02-setup-oct21/x/e02-20261021-224552",
             "oct22-0807": "oct25dl/e02-session-oct22-evening/x/e02-20261022-080709"}.items():
    C.SESSIONS.setdefault(k, v)
ALL = ("oct21-2222", "oct21-2236", "oct21-2245", "oct22-0807", "oct22-0816", "oct22-0951", "oct22-1929",
       "oct22-1954", "oct22-2005", "oct23-morning", "oct23-evening", "stage1", "stage2", "stage3", "right-alone")

GAPS = (2, 5, 10, 20, 40)   # s, camera blanked (chosen: doubling steps around the 10-30 s between runner fixes)
GAP_FIRST_S = 5.0           # first gap starts this long into the run (the filter has seen the robot move)
GAP_SEP_S = 15.0            # camera seen this long between gaps (chosen: the filter re-converges in a few s)
CUT_S = 10.0                # Bc: camera seen for this long, then blind to the run's end
MIN_MOVE_CM = 8.0           # heading check: camera displacement of a straight stretch (as heading_diag.py)
STRAIGHT_DEG = 5.0          # ... while A's heading changes less than this
BIG = 20.0                  # ... a start heading this far off is "wrong" (as heading_diag.py)
HDG_WINDOW_S = 30.0         # ... first stretch within this long of the run's start, else "unmeasured"
REF_GAP_S = 0.15            # camera reference: samples bracketing t within this long ...
REF_JUMP_CM = 3.0           # ... and no more than this apart (a tracker jump otherwise)
LAG_RANGE = np.arange(-0.30, 0.301, 0.02)   # camera time shift tried against the wheels, s
w180 = C.wrap180


# =============================================================================== loading
def sdir(name):
    return os.path.join(C.NC.ROOT, C.SESSIONS[name])


def csv_rows(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def logs(name, fname):
    out = []
    for sub in sorted(glob.glob(os.path.join(sdir(name), "robot-logs-*"))):
        out += csv_rows(os.path.join(sub, fname))
    return out


def num(v):
    return C.num(v)


class Cam:
    """One robot's camera track (held rows left out), with a jump-checked reference position at any time."""

    def __init__(self, pts):
        pts.sort()
        self.t = np.array([p[0] for p in pts])
        self.x = np.array([p[1] for p in pts])
        self.y = np.array([p[2] for p in pts])
        self.src = [p[3] for p in pts]

    def shift(self, dt):
        self.t = self.t + dt

    def ref(self, t):
        i = int(np.searchsorted(self.t, t))
        if i == 0 or i >= len(self.t):
            return None
        a, b = i - 1, i
        if self.t[b] - self.t[a] > 2 * REF_GAP_S or t - self.t[a] > REF_GAP_S or self.t[b] - t > REF_GAP_S:
            return None
        if math.hypot(self.x[b] - self.x[a], self.y[b] - self.y[a]) > REF_JUMP_CM:
            return None
        f = (t - self.t[a]) / (self.t[b] - self.t[a]) if self.t[b] > self.t[a] else 0.0
        return (float(self.x[a] + f * (self.x[b] - self.x[a])), float(self.y[a] + f * (self.y[b] - self.y[a])))

    def last_before(self, t, within=0.5):
        i = int(np.searchsorted(self.t, t)) - 1
        if i < 0 or t - self.t[i] > within:
            return None
        return i

    def path_len(self, t0, t1):
        i, j = int(np.searchsorted(self.t, t0)), int(np.searchsorted(self.t, t1))
        if j - i < 2:
            return 0.0
        return float(np.sum(np.hypot(np.diff(self.x[i:j]), np.diff(self.y[i:j]))))

    def median_near(self, t, half=0.25):
        i, j = int(np.searchsorted(self.t, t - half)), int(np.searchsorted(self.t, t + half))
        if j - i < 2:
            return None
        return float(np.median(self.x[i:j])), float(np.median(self.y[i:j]))


def load_cams(name):
    by = {}
    for r in csv_rows(os.path.join(sdir(name), "track.csv")):
        t, x, y = num(r.get("t")), num(r.get("x_cm")), num(r.get("y_cm"))
        if t is None or x is None or y is None or (r.get("source") or "") == "held":
            continue
        by.setdefault(r["robot"], []).append((t, x, y, r.get("source") or ""))
    return {k: Cam(v) for k, v in by.items()}


def load_runs(name):
    out = []
    for sub in sorted(glob.glob(os.path.join(sdir(name), "robot-logs-*"))):
        rs, _ = RN.load_runs(sub)
        out += rs
    return out


def trimmed(run, t0, t1):
    r = RN.Run(run.run_id, run.robot)
    r.experiment = run.experiment
    r.raw = [q for q in run.raw if t0 - 1e-6 <= q["t"] <= t1 + 1e-6]
    r.params = run.params
    return r


def ms_to_t(run):
    ms = np.array([float(q["ms"]) for q in run.raw])
    tt = np.array([q["t"] for q in run.raw])
    return lambda m: float(np.interp(m, ms, tt))


# =============================================================================== camera lag
def camera_lag(runs, cams):
    """Shift (s) to add to the camera's times so its speed lines up best with the wheels' speed (driving runs)."""
    best = {}
    for lag in LAG_RANGE:
        num_, den = 0.0, 0.0
        acc = []
        for run in runs:
            cam = cams.get(run.robot)
            if cam is None or run.experiment not in ("e02", "find") or len(run.raw) < 200:
                continue
            tt = np.array([q["t"] for q in run.raw])
            v = np.abs(np.array([E.fnum(E.RU.speed_cms(q.get("v_cms"))) for q in run.raw]))
            grid = np.arange(tt[0] + 1, tt[-1] - 1, 0.1)
            if len(grid) < 20:
                continue
            ct = cam.t + lag
            i0, i1 = np.searchsorted(ct, grid - 0.2), np.searchsorted(ct, grid + 0.2)
            ok = (i1 - i0 >= 2) & (i1 < len(ct))
            if ok.sum() < 20:
                continue
            g = grid[ok]
            a, b = i0[ok], np.minimum(i1[ok], len(ct) - 1)
            cs = np.hypot(cam.x[b] - cam.x[a], cam.y[b] - cam.y[a]) / np.maximum(ct[b] - ct[a], 1e-3)
            vw = np.interp(g, tt, v)
            acc.append((vw, cs))
        if not acc:
            return 0.0, None
        vw = np.concatenate([q[0] for q in acc])
        cs = np.concatenate([q[1] for q in acc])
        cs = np.clip(cs, 0, 40)
        best[round(float(lag), 3)] = float(np.corrcoef(vw, cs)[0, 1])
    lag = max(best, key=best.get)
    return lag, best


# =============================================================================== the runs to score
def driving_runs(name, runs):
    """(trimmed Run, info) for each find and each done E02 route; info: kind, cond, start belief (x, y, th)."""
    e02 = {r["run"]: r for r in logs(name, "e02.csv")}
    finds = {r["run"]: r for r in logs(name, "find.csv")}
    rcsv = {r.get("e02_run"): r for r in csv_rows(os.path.join(sdir(name), "runs.csv")) if r.get("e02_run")}
    rinfo = {r["run"]: r for r in logs(name, "runinfo.csv")}
    out = []
    for run in runs:
        if run.experiment == "e02" and run.run_id in e02:
            rec = e02[run.run_id]
            if (rec.get("result") or "") != "done":
                continue
            m2t = ms_to_t(run)
            t0, t1 = m2t(float(rec["ms_start"])), m2t(float(rec["ms_end"]))
            start = tuple(num(rec[k]) for k in ("start_x", "start_y", "start_th"))
            if None in start or t1 - t0 < 5:
                continue
            rc = rcsv.get(run.run_id, {})
            ys = num((rinfo.get(run.run_id) or {}).get("yaw_sign")) or 1
            out.append((trimmed(run, t0, t1), dict(kind="route", cond=rc.get("cond", "?"), route=rc.get("route", ""),
                                                   order=rc.get("order", ""), start=start, rec=rec, rc=rc,
                                                   yaw_sign=1 if ys >= 0 else -1, raw_full=run)))
        elif run.experiment == "find" and run.run_id in finds:
            f = finds[run.run_id]
            start = tuple(num(f.get(k)) for k in ("before_x", "before_y", "before_th"))
            if None in start or (f.get("known_before") or "1") in ("0", ""):
                continue
            if run.raw[-1]["t"] - run.raw[0]["t"] < 5:
                continue
            out.append((run, dict(kind="find", cond="find", route="", order="", start=start, rec=f, rc={},
                                  result=f.get("result", ""), raw_full=run)))
    return out


# =============================================================================== the fusion setups
def gap_windows(L, dur):
    out, s = [], GAP_FIRST_S
    while s + L <= dur:
        out.append([s, s + L])
        s += L + GAP_SEP_S
    return out


def run_setup(runs_info, cam, cfg, start_from_camera):
    """Engine over one robot's runs; returns run_id -> trajectory arrays (t, x, y, th)."""
    rl = []
    for run, info in runs_info:
        x, y, th = info["start"]
        if start_from_camera:
            p = cam.median_near(run.raw[0]["t"]) if cam is not None else None
            if p is not None:
                x, y = p
        run.tape_start = (x, y, th)
        rl.append(run)
    if not rl:
        return {}
    dets = []
    if cam is not None and "camera" in cfg["sensors"]:
        spans = [(r.raw[0]["t"] - 1, r.raw[-1]["t"] + 1) for r in rl]
        for i, t in enumerate(cam.t):
            if any(a <= t <= b for a, b in spans):
                dets.append(dict(frame=i, t=float(t), x=float(cam.x[i]), y=float(cam.y[i])))
    tracked, _ = E.run_engine(rl, dets, cfg)
    out = {}
    for rid, tr in tracked.items():
        tj = np.array(tr.trajectory())
        out[rid] = dict(t=tj[:, 0], x=tj[:, 1], y=tj[:, 2], th=tj[:, 3], sx=tj[:, 4], cam_used=tr.cam_used)
    return out


def at(tj, t):
    i = int(np.searchsorted(tj["t"], t))
    i = min(max(i, 0), len(tj["t"]) - 1)
    if abs(tj["t"][i] - t) > 0.2 and i > 0 and abs(tj["t"][i - 1] - t) < abs(tj["t"][i] - t):
        i -= 1
    if abs(tj["t"][i] - t) > 0.3:
        return None
    return float(tj["x"][i]), float(tj["y"][i]), float(tj["th"][i])


def own_trail(run_info):
    """Firmware's own floor pose over a route: (t array, x, y, th deg), or None."""
    run, info = run_info
    if info["kind"] != "route":
        return None
    full = info["raw_full"]
    raw = [{k: num(q.get(k)) for k in ("ms", "x_cm", "y_cm", "yaw_deg", "th_deg")} for q in full.raw]
    rec = info["rec"]
    tr = C.reintegrate(raw, rec, info["yaw_sign"], num(rec["gyro_scale"]) or 1.0, num(rec["gyro_drift_dps"]) or 0.0,
                       num(rec["wheel_scale"]) or 1.0, path=True)
    if not tr:
        return None
    m2t = ms_to_t(full)
    return dict(t=np.array([m2t(q[0]) for q in tr]), x=np.array([q[1] for q in tr]), y=np.array([q[2] for q in tr]),
                th=np.array([math.radians(q[3]) for q in tr]))


# =============================================================================== heading check
def heading_check(tj, cam, raw):
    """Start-heading status from A's trajectory (start belief + gyro) against the camera's direction of travel."""
    t0 = tj["t"][0]
    tt = np.array([q["t"] for q in raw])
    v = np.array([E.fnum(E.RU.speed_cms(q.get("v_cms"))) for q in raw])
    grid = np.arange(t0, min(tj["t"][-1], t0 + HDG_WINDOW_S), 0.2)
    errs = []
    i = 0
    while i < len(grid):
        pa = cam.ref(grid[i])
        if pa is None:
            i += 1
            continue
        j = i + 1
        found = None
        while j < len(grid) and grid[j] - grid[i] < 6.0:
            pb = cam.ref(grid[j])
            if pb is not None and math.hypot(pb[0] - pa[0], pb[1] - pa[1]) >= MIN_MOVE_CM:
                found = (j, pb)
                break
            j += 1
        if found is None:
            i += 1
            continue
        j, pb = found
        a, b = at(tj, grid[i]), at(tj, grid[j])
        k = (tt >= grid[i]) & (tt <= grid[j])
        vm = float(np.mean(v[k])) if k.any() else 0.0
        if a is None or b is None or abs(vm) < 1.0:
            i += 1
            continue
        if abs(math.degrees(w180_rad(b[2] - a[2]))) < STRAIGHT_DEG:
            d = math.degrees(math.atan2(pb[1] - pa[1], pb[0] - pa[0]))
            if vm < 0:
                d += 180.0
            errs.append((grid[i] - t0, w180(math.degrees(0.5 * (a[2] + b[2])) - d)))
            i = j
            continue
        i += 1
    if not errs:
        return "unmeasured", None, []
    e0 = errs[0][1]
    return ("ok" if abs(e0) < BIG else "wrong"), e0, errs


def w180_rad(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# =============================================================================== scoring
def score_session(name, out_gaps, out_runs, out_routes, log):
    t_start = time.time()
    runs = load_runs(name)
    cams = load_cams(name)
    lag, curve = camera_lag(runs, cams)
    for c in cams.values():
        c.shift(lag)
    dr = driving_runs(name, runs)
    fixes = {}
    for r in logs(name, "fix.csv"):
        fixes.setdefault(r["robot"], []).append(float(r["t_recv"]))
    log("%s: %d driving runs (%d routes, %d finds); camera shifted %+.2f s (corr %.3f at best, %.3f at 0)" % (
        name, len(dr), sum(1 for _, i in dr if i["kind"] == "route"), sum(1 for _, i in dr if i["kind"] == "find"),
        lag, (curve or {}).get(lag, float("nan")), (curve or {}).get(0.0, float("nan"))))
    if not dr:
        return dict(session=name, lag=lag, n=0)
    robots = sorted({run.robot for run, _ in dr})
    for robot in robots:
        mine = [(r, i) for r, i in dr if r.robot == robot]
        cam = cams.get(robot)
        if cam is None:
            continue
        dur = max(r.raw[-1]["t"] - r.raw[0]["t"] for r, _ in mine)
        A = run_setup(mine, cam, E.config(name="A", sensors=["wheel_v", "wheel_w", "gyro"]), False)
        Bf = run_setup(mine, cam, E.config(name="Bf"), True)
        Bc = run_setup(mine, cam, E.config(name="Bc", cam_off=[[CUT_S, 1e6]]), True)
        B = {L: run_setup(mine, cam, E.config(name="B%d" % L, cam_off=gap_windows(L, dur)), True) for L in GAPS}
        for run, info in mine:
            rid = run.run_id
            a = A.get(rid)
            if a is None or len(a["t"]) < 20:
                continue
            status, e0, errs = heading_check(a, cam, run.raw)
            own = own_trail((run, info))
            t0, t1 = run.raw[0]["t"], run.raw[-1]["t"]
            moved = cam.path_len(t0, t1)
            fx = [t for t in fixes.get(robot, []) if t0 <= t <= t1]
            # how closely the full fusion follows the camera, and A's error against it (start aligned)
            res_bf, res_a, res_own = [], [], []
            c0 = cam.median_near(t0)
            a0 = at(a, t0)
            o0 = (own["x"][0], own["y"][0]) if own is not None else None
            for t in np.arange(t0 + 0.5, t1, 0.5):
                p = cam.ref(t)
                if p is None:
                    continue
                q = at(Bf[rid], t) if rid in Bf else None
                if q is not None:
                    res_bf.append(math.hypot(q[0] - p[0], q[1] - p[1]))
            pe = cam.median_near(t1 - 0.3)
            ae = at(a, t1 - 0.3)
            row = dict(session=name, robot=robot, run=rid, kind=info["kind"], cond=info["cond"], route=info["route"],
                       order=info["order"], dur_s=round(t1 - t0, 1), moved_cm=round(moved, 1),
                       hdg=status, hdg_err0=round(e0, 1) if e0 is not None else "",
                       hdg_err_med=round(float(np.median([abs(e) for _, e in errs])), 1) if errs else "",
                       n_fixes_in_run=len(fx), cam_used_bf=Bf.get(rid, {}).get("cam_used", ""),
                       bf_res_med=round(float(np.median(res_bf)), 2) if res_bf else "",
                       bf_res_p95=round(float(np.percentile(res_bf, 95)), 2) if res_bf else "")
            # end errors over the whole run, no camera after the start
            if pe is not None and ae is not None:
                row["A_end_abs"] = round(math.hypot(ae[0] - pe[0], ae[1] - pe[1]), 2)
                if c0 is not None and a0 is not None:
                    row["A_end_aligned"] = round(math.hypot((ae[0] - a0[0]) - (pe[0] - c0[0]),
                                                            (ae[1] - a0[1]) - (pe[1] - c0[1])), 2)
                bc = at(Bc[rid], t1 - 0.3) if rid in Bc else None
                if bc is not None:
                    row["Bc_end"] = round(math.hypot(bc[0] - pe[0], bc[1] - pe[1]), 2)
                    pc = cam.median_near(t0 + CUT_S)
                    if pc is not None and t1 - t0 > CUT_S + 5:
                        # each source alone from the camera's last view at CUT_S to the end
                        ac, oc = at(a, t0 + CUT_S), None
                        row["Bc_cam_hold"] = round(math.hypot(pc[0] - pe[0], pc[1] - pe[1]), 2)
                        if ac is not None:
                            row["Bc_A"] = round(math.hypot(pc[0] + ae[0] - ac[0] - pe[0], pc[1] + ae[1] - ac[1] - pe[1]), 2)
                        if own is not None:
                            oc_, oe_ = interp_trail(own, t0 + CUT_S), interp_trail(own, t1 - 0.3)
                            if oc_ is not None and oe_ is not None:
                                row["Bc_own"] = round(math.hypot(pc[0] + oe_[0] - oc_[0] - pe[0],
                                                                 pc[1] + oe_[1] - oc_[1] - pe[1]), 2)
                if own is not None:
                    oe = interp_trail(own, t1 - 0.3)
                    if oe is not None:
                        row["own_end_abs"] = round(math.hypot(oe[0] - pe[0], oe[1] - pe[1]), 2)
                        if c0 is not None:
                            row["own_end_aligned"] = round(math.hypot((oe[0] - o0[0]) - (pe[0] - c0[0]),
                                                                      (oe[1] - o0[1]) - (pe[1] - c0[1])), 2)
            if info["kind"] == "route":
                rec, rc = info["rec"], info["rc"]
                ex, ey = num(rec.get("end_x")), num(rec.get("end_y"))
                if pe is not None and ex is not None:
                    row["fw_end_abs"] = round(math.hypot(ex - pe[0], ey - pe[1]), 2)   # incl. KC fixes
                row["runner_end_err"] = rc.get("end_err_cm", "")
                row["runner_thought_end"] = rc.get("robot_thought_end_cm", "")
                row["fixes_used"] = rec.get("fixes_used", "")
                out_routes.append(row)
            out_runs.append(row)
            # gaps
            for L in GAPS:
                bl = B[L].get(rid)
                if bl is None:
                    continue
                for s, e in gap_windows(L, t1 - t0):
                    ts, te = t0 + s, t0 + e
                    i = cam.last_before(ts)
                    pe_ = cam.ref(te)
                    if i is None or pe_ is None:
                        continue
                    ps = (float(cam.x[i]), float(cam.y[i]))
                    g = dict(session=name, robot=robot, run=rid, kind=info["kind"], cond=info["cond"], hdg=status,
                             L=L, t_s=round(ts, 2), moved_cm=round(cam.path_len(ts, te), 1),
                             disp_cm=round(math.hypot(pe_[0] - ps[0], pe_[1] - ps[1]), 1),
                             fix_inside=int(any(ts <= t <= te for t in fixes.get(robot, []))))
                    q = at(bl, te)
                    if q is not None:
                        g["fused"] = round(math.hypot(q[0] - pe_[0], q[1] - pe_[1]), 2)
                    qf = at(Bf[rid], te) if rid in Bf else None
                    if qf is not None:
                        g["fused_seen"] = round(math.hypot(qf[0] - pe_[0], qf[1] - pe_[1]), 2)
                    g["cam_hold"] = round(math.hypot(ps[0] - pe_[0], ps[1] - pe_[1]), 2)
                    j = cam.last_before(ts - 1.0, within=0.3)
                    if j is not None and cam.t[i] > cam.t[j]:
                        vx = (cam.x[i] - cam.x[j]) / (cam.t[i] - cam.t[j])
                        vy = (cam.y[i] - cam.y[j]) / (cam.t[i] - cam.t[j])
                        dt = te - cam.t[i]
                        g["cam_cv"] = round(math.hypot(ps[0] + vx * dt - pe_[0], ps[1] + vy * dt - pe_[1]), 2)
                    a_s, a_e = at(a, ts), at(a, te)
                    if a_s is not None and a_e is not None:
                        g["wheels_gyro"] = round(math.hypot(ps[0] + a_e[0] - a_s[0] - pe_[0],
                                                            ps[1] + a_e[1] - a_s[1] - pe_[1]), 2)
                    if own is not None:
                        os_, oe_ = interp_trail(own, ts), interp_trail(own, te)
                        if os_ is not None and oe_ is not None:
                            g["own_pose"] = round(math.hypot(ps[0] + oe_[0] - os_[0] - pe_[0],
                                                             ps[1] + oe_[1] - os_[1] - pe_[1]), 2)
                    out_gaps.append(g)
    log("  %s done in %.0f s" % (name, time.time() - t_start))
    return dict(session=name, lag=lag, n=len(dr))


def interp_trail(tr, t):
    if t < tr["t"][0] - 0.3 or t > tr["t"][-1] + 0.3:
        return None
    return float(np.interp(t, tr["t"], tr["x"])), float(np.interp(t, tr["t"], tr["y"]))


# =============================================================================== summary
COLS_GAP = ("fused", "cam_hold", "cam_cv", "wheels_gyro", "own_pose")


def summarise(gaps, runs_, routes, log):
    def fl(r, k):
        v = r.get(k, "")
        return None if v in ("", None) else float(v)

    log("\n== Gaps: error at the gap's end against the camera, cm (median / 90th percentile; ok-heading runs)")
    log("%-6s %-6s %5s | %-13s %-13s %-13s %-13s %-13s | %s" % ("L s", "kind", "n", "fused", "camera hold",
                                                                 "camera c.v.", "wheels+gyro", "own pose", "fused best share"))
    for kind in ("route", "find", "all"):
        for L in GAPS:
            sel = [g for g in gaps if g["L"] == L and g["hdg"] == "ok" and (kind == "all" or g["kind"] == kind)
                   and fl(g, "fused") is not None and fl(g, "wheels_gyro") is not None]
            if not sel:
                continue
            cells = []
            for k in COLS_GAP:
                v = [fl(g, k) for g in sel if fl(g, k) is not None]
                cells.append("%5.2f / %5.2f" % (np.median(v), np.percentile(v, 90)) if v else "      -      ")
            best = np.mean([fl(g, "fused") <= min(fl(g, k) for k in ("cam_hold", "wheels_gyro") if fl(g, k) is not None)
                            for g in sel])
            log("%-6s %-6s %5d | %s | %.0f%%" % (L, kind, len(sel), " ".join(cells), 100 * best))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--sessions", default=",".join(ALL))
    ap.add_argument("--out", default=C.OUT)
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    lines = []

    def log(s):
        print(s, flush=True)
        lines.append(s)

    gaps, runs_, routes, meta = [], [], [], []
    for name in a.sessions.split(","):
        if not os.path.isdir(sdir(name)):
            log("%s: folder missing, skipped" % name)
            continue
        meta.append(score_session(name, gaps, runs_, routes, log))
    for fname, rows_ in (("real_fusion_gaps.csv", gaps), ("real_fusion_runs.csv", runs_),
                         ("real_fusion_routes.csv", routes)):
        keys = []
        for r in rows_:
            keys += [k for k in r if k not in keys]
        with open(os.path.join(a.out, fname), "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=keys)
            wr.writeheader()
            wr.writerows(rows_)
    with open(os.path.join(a.out, "real_fusion_meta.json"), "w") as f:
        json.dump(meta, f, indent=1)
    summarise(gaps, runs_, routes, log)
    with open(os.path.join(a.out, "real_fusion.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
