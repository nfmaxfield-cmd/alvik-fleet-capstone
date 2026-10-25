"""Shared loaders for the 2026-10-24 night analyses (docs/build-reports/e02-oct24-night-analyses.md).

Offline only: reads a session folder's CSVs and state, never talks to robots or the camera. Run the night_*.py
scripts with the session folders as they were downloaded (paths in SESSIONS; override with NIGHT_SESSIONS_ROOT)."""
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(HERE)
sys.path.insert(0, EXP)

import e02_fit as FIT        # noqa: E402
import e02_trust as TR       # noqa: E402
import e02_camerr as CE      # noqa: E402

ROOT = os.environ.get("NIGHT_SESSIONS_ROOT", "/tmp/claude-0")
SESSIONS = {
    "oct23-morning": "oct23/e02-20261023-085107",
    "oct23-evening": "evening/e02-20261023-191347",
    "stage1": "plan/153612/e02-20261024-153612",
    "stage2": "plan/161612/e02-20261024-161612",
    "stage3": "plan/173242/e02-20261024-173242",
}
ROBOTS = ("Left", "Center", "Right")
TZ_H = -4.0     # the sessions' local time (EDT) from Unix seconds


def path(name, *parts):
    return os.path.join(ROOT, SESSIONS[name], *parts)


def hm(t):
    """Unix seconds -> 'HH:MM' local (EDT)."""
    s = (t + TZ_H * 3600) % 86400
    return "%02d:%02d" % (s // 3600, (s % 3600) // 60)


def num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def rows(name, fname, sub=""):
    p = path(name, sub, fname) if sub else path(name, fname)
    if not os.path.exists(p):
        return []
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


def robot_rows(name, fname):
    """A robot-logs CSV from every robot-logs-N folder of the session."""
    out = []
    d = os.path.join(ROOT, SESSIONS[name])
    for sub in sorted(x for x in os.listdir(d) if x.startswith("robot-logs-")):
        out += rows(name, fname, sub)
    return out


def state(name):
    with open(path(name, "e02_state.json")) as f:
        return json.load(f)


def track(name):
    """robot -> sorted list of (t, x, y, source) from track.csv."""
    out = {}
    for r in rows(name, "track.csv"):
        t, x, y = num(r["t"]), num(r["x_cm"]), num(r["y_cm"])
        if t is None or x is None or y is None:
            continue
        out.setdefault(r["robot"], []).append((t, x, y, r.get("source") or ""))
    for v in out.values():
        v.sort()
    return out


def raw_by_run(name):
    """run id -> RAW rows as dicts with floats (what the runner's listener keeps)."""
    out = {}
    for r in robot_rows(name, "raw.csv"):
        d = {k: num(r.get(k)) for k in ("t_recv", "ms", "x_cm", "y_cm", "yaw_deg", "th_deg", "battery", "v_cms",
                                        "wl_rpm", "wr_rpm")}
        d["robot"] = r["robot"]
        out.setdefault(r["run"], []).append(d)
    return out


def runinfo(name):
    return {r["run"]: r for r in robot_rows(name, "runinfo.csv")}


def u_runs(name):
    """The U runs as the runner learned from them: (robot, order, t0, t1, [find run, e02 run])."""
    out = []
    for r in rows(name, "runs.csv"):
        if r["cond"] != "U" or not r.get("t_start") or not r.get("t_end"):
            continue
        out.append((r["robot"], int(r["order"]), float(r["t_start"]), float(r["t_end"]) + 0.5,
                    [x for x in (r.get("find_run"), r.get("e02_run")) if x]))
    return out


def split_at_jumps(cam, jump_cm):
    """Pieces of a camera track [(t, x, y)] split wherever one picture moves more than jump_cm (a tracker jump:
    drives are 8-10 cm/s, 0.5-0.8 cm a picture)."""
    out, cur = [], []
    for q in cam:
        if cur and math.hypot(q[1] - cur[-1][1], q[2] - cur[-1][2]) > jump_cm:
            out.append(cur)
            cur = []
        cur.append(q)
    if cur:
        out.append(cur)
    return out


def rebuild_stretches(name, cam_fn=None, time_shift=0.0, keep_pos=True, split_jump_cm=None):
    """The runner's learn() again, offline: per U run the camera's straight stretches (track.csv) paired with the
    robot's raw counting (raw.csv). cam_fn(t, x, y) -> (x, y) corrects each camera position first (part B);
    time_shift (s) moves the camera's times (camera time - shift) before pairing; split_jump_cm: the camera track is
    cut at jumps bigger than this before the stretches are found (the fix tested in part B). Returns robot -> [obs],
    each obs as FIT.pair gives plus p0, p1 (camera ends), order and the sources at the ends."""
    tr = track(name)
    raw = raw_by_run(name)
    info = runinfo(name)
    out = {}
    for robot, order, t0, t1, runs in u_runs(name):
        cam_all = [q for q in tr.get(robot, []) if t0 <= q[0] <= t1]
        cam = []
        for t, x, y, src in cam_all:
            if cam_fn is not None:
                x, y = cam_fn(t, x, y)
            cam.append((t - time_shift, x, y))
        if split_jump_cm:
            stretches = [s for piece in split_at_jumps(cam, split_jump_cm) for s in FIT.camera_stretches(piece)]
        else:
            stretches = FIT.camera_stretches(cam)
        src_at = {round(q[0] - time_shift, 3): q[3] for q in cam_all}
        for run in runs:
            rr = raw.get(run) or []
            ys = num((info.get(run) or {}).get("yaw_sign")) or 1
            ser = FIT.raw_series(rr, yaw_sign=1 if ys >= 0 else -1)
            if ser is None:
                continue
            k = (ser["t"] >= ser["t"][0] - 0.1) & (ser["t"] <= ser["t"][-1] + 0.1)
            mine = [s for s in stretches if s["t0"] >= ser["t"][k][0] and s["t1"] <= ser["t"][k][-1]]
            obs = FIT.pair(mine, ser, run=run)
            by_tm = {round(s["tm"], 4): s for s in mine}
            for o in obs:
                s = by_tm.get(round(o["tm"], 4))
                if s and keep_pos:
                    o.update(p0=s["p0"], p1=s["p1"], t0=s["t0"], t1=s["t1"], order=order,
                             src0=src_at.get(round(s["t0"], 3), ""), src1=src_at.get(round(s["t1"], 3), ""))
            out.setdefault(robot, []).extend(obs)
    return out


def median(v):
    v = sorted(v)
    n = len(v)
    if not n:
        return None
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


def pct(v, q):
    v = sorted(v)
    if not v:
        return None
    return v[min(len(v) - 1, int(q * len(v)))]
