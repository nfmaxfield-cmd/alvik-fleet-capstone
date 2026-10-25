"""Shared helpers for the 2026-10-25 logs-only DOE rows (docs/build-reports/e02-oct25-logs-doe.md).

Offline only: reads session folders (night_common.SESSIONS, plus the Right-alone night session), never talks to robots
or the camera. Builds on analyses_oct24/night_common.py.

What this adds:
  - run_table(): every finished run with its camera end and the e02 record (start belief, constants used);
  - reintegrate(): the firmware's own pose (pose.h tick) recomputed from a run's RAW rows with any gyro scale, gyro
    drift and wheel scale, so a calibration can be scored by the end error it would have given;
  - stretches_by_run(): the runner's learn() stretches per U run, with the jump cut as a parameter;
  - fix_chain(): every camera fix with the driving and turning the robot did since its previous fix or find.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(EXP, "analyses_oct24"))
sys.path.insert(0, EXP)

import night_common as NC    # noqa: E402
from night_common import FIT, num, rows, robot_rows, track, raw_by_run, runinfo, median, pct, hm  # noqa: E402,F401

NC.SESSIONS.setdefault("right-alone", "r3/logs/e02-20261024-193526")
SESSIONS = NC.SESSIONS
CLASSIC = ("oct23-morning", "oct23-evening", "stage1", "stage2")   # the sessions with U/C/K/KC runs
OUT = os.path.join(HERE, "out")


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def e02_records(name):
    return {r["run"]: r for r in robot_rows(name, "e02.csv")}


def run_table(name):
    """Finished runs (runs.csv status done) with the camera's end, the robot's e02 record and the constants used."""
    e02 = e02_records(name)
    out = []
    for r in rows(name, "runs.csv"):
        if r.get("status") != "done" or not r.get("e02_run"):
            continue
        rec = e02.get(r["e02_run"])
        if rec is None:
            continue
        ce = (num(r.get("cam_end_x")), num(r.get("cam_end_y")))
        if None in ce:
            continue
        out.append(dict(order=int(r["order"]), robot=r["robot"], cond=r["cond"], route=r["route"], run=r["e02_run"],
                        find_run=r.get("find_run") or "", t0=num(r["t_start"]), t1=num(r["t_end"]), cam_end=ce,
                        cam_start=(num(r.get("cam_start_x")), num(r.get("cam_start_y"))),
                        end_err=num(r.get("end_err_cm")), rec=rec,
                        used=dict(gyro_scale=num(rec["gyro_scale"]), gyro_drift_dps=num(rec["gyro_drift_dps"]),
                                  wheel_scale=num(rec["wheel_scale"]))))
    return out


def reintegrate(raw, rec, yaw_sign, g, d, w, path=False):
    """The firmware's pose over the route (e02 record ms_start -> ms_end), started at the record's start pose and
    recomputed from RAW rows with gyro scale g, drift d (deg/s) and wheel scale w (pose.h tick: the library's step
    projected on its own heading, times w; heading = g x raw yaw steps - d x time). Returns (x, y, th) or None;
    path=True: the whole trail [(ms, x, y, th)]."""
    ms0, ms1 = num(rec["ms_start"]), num(rec["ms_end"])
    x, y, th = num(rec["start_x"]), num(rec["start_y"]), num(rec["start_th"])
    rr = sorted((q for q in raw if q.get("ms") is not None and ms0 - 1 <= q["ms"] <= ms1 + 1
                 and None not in (q.get("x_cm"), q.get("y_cm"), q.get("yaw_deg"), q.get("th_deg"))),
                key=lambda q: q["ms"])
    if len(rr) < 5 or None in (x, y, th):
        return None
    prev = rr[0]
    h = th
    trail = [(prev["ms"], x, y, h)]
    for q in rr[1:]:
        dx, dy = q["x_cm"] - prev["x_cm"], q["y_cm"] - prev["y_cm"]
        lth = math.radians(q["th_deg"])
        ds = (dx * math.cos(lth) + dy * math.sin(lth)) * w
        dyaw = wrap180(q["yaw_deg"] - prev["yaw_deg"]) * yaw_sign
        dt = (q["ms"] - prev["ms"]) / 1000.0
        dh = g * dyaw - d * dt
        mid = math.radians(h + 0.5 * dh)
        x += ds * math.cos(mid)
        y += ds * math.sin(mid)
        h = wrap180(h + dh)
        prev = q
        if path:
            trail.append((q["ms"], x, y, h))
    return trail if path else (x, y, h)


def yaw_signs(name):
    out = {}
    for run, r in runinfo(name).items():
        s = num(r.get("yaw_sign")) or 1
        out[run] = 1 if s >= 0 else -1
    return out


def stretches_by_run(name, jump_cm=FIT.JUMP_CM, robots=None):
    """robot -> list of (order, e02 run, [obs]) for every U run, as the runner's learn() pairs them, with the camera
    track cut at jumps over jump_cm (None: no cut)."""
    tr = track(name)
    raw = raw_by_run(name)
    ys = yaw_signs(name)
    out = {}
    for robot, order, t0, t1, runs in NC.u_runs(name):
        if robots and robot not in robots:
            continue
        cam = [(q[0], q[1], q[2]) for q in tr.get(robot, []) if t0 <= q[0] <= t1]
        st = FIT.camera_stretches(cam, jump_cm=jump_cm)
        obs = []
        for run in runs:
            ser = FIT.raw_series(raw.get(run) or [], yaw_sign=ys.get(run, 1))
            if ser is None:
                continue
            k = (ser["t"] >= ser["t"][0] - 0.1) & (ser["t"] <= ser["t"][-1] + 0.1)
            mine = [s for s in st if s["t0"] >= ser["t"][k][0] and s["t1"] <= ser["t"][k][-1]]
            obs += FIT.pair(mine, ser, run=run)
        e02_run = runs[-1] if runs else ""
        out.setdefault(robot, []).append((order, e02_run, obs))
    return out


def measured_drift(name):
    """The naming check's standing-still drift per robot, as the runner's prior() turns it (x the prior gyro scale)."""
    st = NC.state(name)
    d = ((st.get("naming_check") or {}).get("gyro_raw_drift_dps") or {})
    return {r: round(FIT.PRIOR["gyro_scale"] * v, 4) for r, v in d.items() if v is not None}


def fit_with(obs, drift, band=None, max_rms=None):
    """fit_robot as the runner calls it, with DRIFT_BAND_DPS and the wheel gate as parameters (band None: no hold)."""
    old = FIT.DRIFT_BAND_DPS
    try:
        FIT.DRIFT_BAND_DPS = 1e9 if band is None else band
        pr = dict(FIT.PRIOR)
        if drift is not None:
            pr["gyro_drift_dps"] = drift
        return FIT.fit_robot(obs, None, prior=pr, prior_sd=dict(gyro_drift_dps=0.1) if drift is not None else None,
                             measured_drift=drift, max_rms_cm=max_rms)
    finally:
        FIT.DRIFT_BAND_DPS = old


def fix_chain(name):
    """Every camera fix of the session (fix.csv) with what the robot drove and turned since its previous reset (a
    fix or a finished find), from the done legs in leg.csv. Fixes sent during a firmware run (run != '-') are KC
    blends; those and any fix with a run (route) between it and the previous reset are marked chain=False."""
    fixes = sorted(robot_rows(name, "fix.csv"), key=lambda r: float(r["t_recv"]))
    finds = [r for r in robot_rows(name, "find.csv") if (r.get("result") or "") == "ok"]
    legs = [r for r in robot_rows(name, "leg.csv") if r.get("result") == "done"]
    runs_ = robot_rows(name, "runinfo.csv")
    ev = {}
    for r in fixes:
        ev.setdefault(r["robot"], []).append(("fix", float(r["t_recv"]), r))
    for r in finds:
        ev.setdefault(r["robot"], []).append(("find", float(r["t_recv"]), r))
    out = []
    for robot, lst in ev.items():
        lst.sort(key=lambda e: e[1])
        my_legs = sorted(((float(l["t_recv"]), l) for l in legs if l["robot"] == robot), key=lambda e: e[0])
        my_runs = sorted(float(r["t_recv"]) for r in runs_ if r["robot"] == robot)
        prev = None
        for kind, t, r in lst:
            if kind == "fix" and prev is not None:
                t_prev = prev[1]
                between = [l for tl, l in my_legs if t_prev < tl <= t]
                dist = sum(abs(num(l["value"]) or 0.0) for l in between if l["kind"] == "go")
                turned = sum(abs(num(l["value"]) or 0.0) for l in between if l["kind"] == "turn")
                run_between = any(t_prev < tr_ < t for tr_ in my_runs)
                # chain intact: from the previous reset's pose, every done leg moves the own pose by no more than it
                # asked for (+2 cm), and this fix's "before" pose is where the last leg ended (within 2 cm)
                if prev[0] == "fix":
                    px, py = num(prev[2]["after_x"]), num(prev[2]["after_y"])
                else:
                    px, py = num(prev[2]["after_x"]), num(prev[2]["after_y"])
                intact = px is not None and py is not None
                for l in between:
                    lx, ly = num(l["x_cm"]), num(l["y_cm"])
                    if not intact or lx is None or ly is None:
                        intact = False
                        break
                    allow = (abs(num(l["value"]) or 0.0) if l["kind"] == "go" else 0.0) + 2.0
                    if math.hypot(lx - px, ly - py) > allow:
                        intact = False
                        break
                    px, py = lx, ly
                bx, by, bth = num(r["before_x"]), num(r["before_y"]), num(r["before_th"])
                cx, cy, cth = num(r["cam_x"]), num(r["cam_y"]), num(r["cam_th"])
                pth = None
                if prev[0] == "fix":
                    pth_cam, pth_bef = num(prev[2]["cam_th"]), num(prev[2]["before_th"])
                    # the previous fix carried no camera heading of its own when it equals the robot's own (to 0.1)
                    pth = None if pth_cam is None or pth_bef is None else abs(wrap180(pth_cam - pth_bef)) > 0.05
                bx_, by_ = num(r["before_x"]), num(r["before_y"])
                if intact and (bx_ is None or math.hypot(bx_ - px, by_ - py) > 2.0):
                    intact = False
                out.append(dict(session=name, robot=robot, t=t, run=r.get("run", "-"), prev_kind=prev[0], intact=intact,
                                prev_dt=t - t_prev, dist=dist, turned=turned, n_legs=len(between),
                                chain=(r.get("run", "-") == "-") and not run_between and
                                      (prev[0] == "find" or prev[2].get("run", "-") == "-"),
                                corr=None if None in (bx, by, cx, cy) else math.hypot(cx - bx, cy - by),
                                dth=None if None in (bth, cth) else wrap180(cth - bth),
                                prev_had_cam_heading=pth, k=num(r.get("k"))))
            prev = (kind, t, r)
    out.sort(key=lambda e: e["t"])
    return out


def unc_model(dist, turned, base=0.5, per_cm=0.01, deg=0.5, deg_per_deg=0.005):
    """e02.py unc_cm: 1 sd (cm) after `dist` cm driven and `turned` degrees turned since the last fix."""
    return base + per_cm * dist + dist * math.radians(deg + deg_per_deg * turned)


def write_csv(path, header, data):
    import csv
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(data)
