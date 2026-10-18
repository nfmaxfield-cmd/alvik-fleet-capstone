#!/usr/bin/env python3
"""Distance-sensor calibration for landmark ranging: learn how each robot's front
distance sensor really reads the boards, and where the boards really are, from
runs where the camera saw the robot.

    python tools/fusion/tof_calib.py --session logs/<session> --sheet <measurements.csv> \\
        --detections <detections.csv> --calib <floor_calibration.json> --recording <recording> \\
        --map map.json --out <folder>

The camera (with wheels and gyro, smoothed by the factor graph) says where each
robot was; the map says roughly where the boards are (tape, or clicked in the
camera video). Every distance-sensor reading that hits a board is then a
check of the sensor and the board together. The tool finds the settings that
make the readings agree best:

  per robot   each zone's real direction (offset from 22, 11, 0, -11, -22 deg),
              range bias (cm; relative to the robots' average, which is held at 0:
              a bias every sensor shares looks exactly like every board being that
              much further, so it goes into the boards) and range scale
  all robots  zone width (where in its slice a zone reads; 0 = one ray down the
              middle), frame delay (how old a frame is when its RAW row is sent)
  per board   its position and turn (dx, dy cm, turn deg): REFINED LANDMARKS
  noise       reading noise (cm + fraction of the distance), how alike the errors
              of zones in the same frame are, and of a zone from frame to frame

Robust fit (soft-L1: a reading far off, e.g. a hand in front, counts little),
with priors that keep each setting near its nominal value unless the data say
otherwise (zone direction 5 deg, bias 1 cm, scale 3%, board 2 cm and 3 deg).

Calibration runs (firmware 2026-10-18b+, `--calib-results calibration_results.csv`
from tools/blink/calib.py): during their two slow 360 deg spins (RAW steps 6 and 8)
the front sensor sweeps the boards around it at every angle, the best data for
zone directions and width. Their reference pose: position from the camera (the
robot turns on the spot), heading from the gyro (x calib.py's gyro_scale, bias
from the still start) anchored to calib.py's measured start heading.

No circularity: a run must never be scored with a calibration made from it.
The tool also fits once without each run (leave-one-run-out) and writes
tof_calibration_loro.csv; give that to replay.py as --tof-cal-per-run. The
spread of those fits (jackknife) is how sure each refined board and sensor
setting is; the filter carries that as uncertainty (tof_consider).

Writes in --out: tof_calibration.json (all runs; for later sessions),
loro/<run>.json, tof_calibration_loro.csv, landmarks_refined.csv,
residuals.csv (every reading used, before and after).
"""
import argparse
import csv
import json
import math
import os
import re
import statistics as st
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
import engine as eng  # noqa: E402
import graph  # noqa: E402
import landmarks as lm  # noqa: E402
import runs as rn  # noqa: E402
from ekf import D2R  # noqa: E402

try:
    from scipy.optimize import least_squares
except ImportError:   # pragma: no cover
    least_squares = None

PRIOR = dict(zone_deg=5.0, bias_cm=1.0, scale=0.03, board_cm=2.0, board_deg=3.0)
GROUP_RE = re.compile(r"^(.*) (left|right|end p1|end p2|edge \d+)$")


def face_group(name):
    """Board / obstacle a face belongs to ("A left" -> "A"); a lone face is its own group."""
    m = GROUP_RE.match(name)
    return m.group(1) if m else name


# ---------------------------------------------------------------- data
def reference_paths(runs, dets, cfg_over=None, method="graph", gyro_f=None, signs=None):
    """Camera + wheels + gyro estimate of every run: {run_id: (t, x, y, th unwrapped) arrays}."""
    cfg = eng.config(name="reference", sensors=["wheel_v", "wheel_w", "gyro", "camera"], **(cfg_over or {}))
    tracked, _ = eng.run_engine(runs, dets, cfg, gyro_f, signs=signs)
    out = {}
    for run in runs:
        tr = tracked.get(run.run_id)
        if tr is None or tr.cam_used < 10:
            continue
        traj = graph.smooth(tr) if method == "graph" else tr.trajectory()
        if len(traj) < 3:
            continue
        a = np.array([p[:4] for p in traj], float)
        a[:, 3] = np.unwrap(a[:, 3])
        out[run.run_id] = a
    return out


SPIN_STEPS = {6, 8}


def spin_reference(run, dets, crow, gf, max_cm=8.0, win_s=0.3):
    """Reference path of a calibration run (see the module help): (t, x, y, th unwrapped) array at its RAW
    rows, or None (no camera, or no measured start heading)."""
    row = next((c for c in crow or [] if c["run"] == run.run_id and c.get("event") == "calib"
                and c.get("heading") is not None and c.get("t") is not None), None)
    if row is None or len(run.raw) < 10:
        return None
    t0, t1 = run.raw[0]["t"], run.raw[-1]["t"]
    pts = [(d["t"], d["x"], d["y"]) for d in dets if t0 - 0.5 <= d["t"] <= t1 + 0.5]
    if len(pts) < 20:
        return None
    P = np.array(pts)
    mx, my = np.median(P[:, 1]), np.median(P[:, 2])
    P = P[np.hypot(P[:, 1] - mx, P[:, 2] - my) < 15.0 + max_cm]       # this robot (it rolls 15 cm and spins)
    T = np.array([r["t"] for r in run.raw])
    g = np.array([eng.fnum(r.get("gz")) for r in run.raw]) * gf * row.get("gs", 1.0)
    still = [i for i, r in enumerate(run.raw) if eng.fnum(r.get("step")) == 1]
    bias = float(np.median(g[still])) if len(still) > 10 else 0.0
    th = np.r_[0.0, np.cumsum(0.5 * ((g[1:] - bias) + (g[:-1] - bias)) * np.diff(T))]
    th += row["heading"] * D2R - np.interp(row["t"], T, th)
    out = []
    for i, t in enumerate(T):
        m = np.abs(P[:, 0] - t) <= win_s
        if m.sum() >= 3:
            out.append((t, float(np.median(P[m, 1])), float(np.median(P[m, 2])), th[i]))
    return np.array(out) if len(out) > 10 else None


def collect(runs, refs, faces, cfg, new_frames_only=True, steps=None):
    """Every zone reading that ties to one mapped face at the reference pose (nominal sensor model).
    Returns a dict of arrays (one entry per reading) and the robot names."""
    robots = sorted({r.robot for r in runs if r.run_id in refs})
    rows = []
    P = np.diag([0.09, 0.09, (0.5 * D2R) ** 2, 1, 1, 1])
    for run in runs:
        ref = refs.get(run.run_id)
        if ref is None:
            continue
        T = ref[:, 0]
        prev = None
        for r in run.raw:
            z = tuple(eng.zone_reading(r.get(c)) for c in lm.ZONE_COLS)
            if new_frames_only and z == prev:
                continue
            prev = z
            t = r["t"]
            if eng.fnum(r.get("lifted")) > 0.5 or t < T[0] + 0.1 or t > T[-1] - 0.1:
                continue
            if steps and run.run_id in steps and int(eng.fnum(r.get("step"), -99)) not in steps[run.run_id]:
                continue
            j = min(max(int(np.searchsorted(T, t)), 1), len(T) - 1)
            f = (t - T[j - 1]) / (T[j] - T[j - 1])
            pose = ref[j - 1, 1:4] + f * (ref[j, 1:4] - ref[j - 1, 1:4])
            vel = (ref[j, 1:4] - ref[j - 1, 1:4]) / (T[j] - T[j - 1])
            s = np.array([pose[0], pose[1], pose[2], 0, 0, 0])
            for zi, (ang, zz) in enumerate(zip(cfg["tof_zone_deg"], z)):
                if zz is None or not cfg["tof_min_cm"] <= zz <= cfg["tof_max_cm"]:
                    continue
                why, info = lm.predict_zone(s, P, ang * D2R, faces, cfg)
                if why:
                    continue
                if abs(zz - info["h"]) > max(4.0, 0.15 * info["h"]):      # something unmapped in front
                    continue
                rows.append((run.run_id, robots.index(run.robot), zi, info["face"], t, zz, *pose, *vel))
    if not rows:
        return None, robots
    cols = list(zip(*rows))
    d = dict(run=np.array(cols[0]), robot=np.array(cols[1], int), zone=np.array(cols[2], int),
             face=np.array(cols[3], int), t=np.array(cols[4], float), z=np.array(cols[5], float),
             x=np.array(cols[6]), y=np.array(cols[7]), th=np.array(cols[8]),
             vx=np.array(cols[9]), vy=np.array(cols[10]), vth=np.array(cols[11]))
    return d, robots


# ---------------------------------------------------------------- model
class Model:
    """Vectorised prediction of every reading from the parameters (see the module help)."""

    def __init__(self, data, robots, faces, cfg, fit_width=True, fit_delay=True, board_cm=PRIOR["board_cm"]):
        self.d, self.robots, self.faces, self.cfg = data, robots, faces, cfg
        self.fwd = cfg["tof_forward_cm"] - cfg["tof_recess_cm"]
        self.nom = np.array(cfg["tof_zone_deg"], float)
        self.groups = sorted({face_group(f.name) for f in faces})
        self.fg = np.array([self.groups.index(face_group(f.name)) for f in faces], int)
        cen = {g: np.mean([np.r_[f.a, f.b].reshape(2, 2).mean(0) for f in faces if face_group(f.name) == g], 0)
               for g in self.groups}
        self.cen = np.array([cen[g] for g in self.groups])
        self.A = np.array([f.a for f in faces])
        self.Nn = np.array([f.n for f in faces])
        self.fit_width, self.fit_delay = fit_width, fit_delay
        self.board_cm = board_cm
        self.R = len(robots)
        self.G = len(self.groups)
        self.np = 7 * self.R + 2 + 3 * self.G

    def unpack(self, p):
        R, G = self.R, self.G
        r = p[:7 * R].reshape(R, 7)
        width, delay = p[7 * R], p[7 * R + 1]
        g = p[7 * R + 2:].reshape(G, 3)
        return r[:, :5], r[:, 5], r[:, 6], width, delay, g

    def x0(self, width0=0.0, delay0=0.0):
        p = np.zeros(self.np)
        p[7 * self.R] = width0
        p[7 * self.R + 1] = delay0
        return p

    def faces_at(self, g):
        """Face points and normals with each group moved (dx, dy) and turned (deg) about its centre."""
        phi = g[self.fg, 2] * D2R
        c, s = np.cos(phi), np.sin(phi)
        cen = self.cen[self.fg]
        rel = self.A - cen
        a = cen + np.c_[c * rel[:, 0] - s * rel[:, 1], s * rel[:, 0] + c * rel[:, 1]] + g[self.fg, :2]
        n = np.c_[c * self.Nn[:, 0] - s * self.Nn[:, 1], s * self.Nn[:, 0] + c * self.Nn[:, 1]]
        return a, n

    def predict(self, p, idx=None):
        d = self.d
        sel = slice(None) if idx is None else idx
        off, bias, scale, width, delay, g = self.unpack(p)
        a_f, n_f = self.faces_at(g)
        rb, zi, fi = d["robot"][sel], d["zone"][sel], d["face"][sel]
        x = d["x"][sel] - delay * d["vx"][sel]
        y = d["y"][sel] - delay * d["vy"][sel]
        th = d["th"][sel] - delay * d["vth"][sel]
        px, py = x + self.fwd * np.cos(th), y + self.fwd * np.sin(th)
        a, n = a_f[fi], n_f[fi]
        ang = (self.nom[zi] + off[rb, zi]) * D2R
        if width > 1e-6:
            side = (a[:, 0] - px) * n[:, 0] + (a[:, 1] - py) * n[:, 1]
            m = n * np.sign(side)[:, None]
            rel = np.arctan2(m[:, 1], m[:, 0]) - th - ang
            rel = (rel + np.pi) % (2 * np.pi) - np.pi
            ang = ang + np.clip(rel, -width * D2R / 2, width * D2R / 2)
        ux, uy = np.cos(th + ang), np.sin(th + ang)
        nu = n[:, 0] * ux + n[:, 1] * uy
        nu = np.where(np.abs(nu) < 1e-6, 1e-6, nu)
        dist = ((a[:, 0] - px) * n[:, 0] + (a[:, 1] - py) * n[:, 1]) / nu
        return dist * (1 + scale[rb]) + bias[rb]

    def priors(self, p):
        off, bias, scale, width, delay, g = self.unpack(p)
        # the sensors' COMMON range bias can't be told from every board being that much further (each board
        # is seen from one side): the robots' average bias is held at 0 and the common part goes into the boards
        return np.r_[off.ravel() / PRIOR["zone_deg"], bias / PRIOR["bias_cm"], scale / PRIOR["scale"],
                     g[:, 0] / self.board_cm, g[:, 1] / self.board_cm, g[:, 2] / PRIOR["board_deg"],
                     np.mean(bias) / 0.01]


def fit(model, sigma, idx=None, p0=None, max_nfev=60):
    """Robust least squares over the readings idx (all if None). sigma: per-reading noise (cm)."""
    if least_squares is None:
        raise RuntimeError("tof_calib needs scipy: pip install scipy")
    d = model.d
    sel = np.arange(len(d["z"])) if idx is None else idx
    z, sg = d["z"][sel], sigma[sel]
    R = model.R
    p0 = model.x0() if p0 is None else np.array(p0, float)
    free = np.ones(model.np, bool)                  # width / delay held where not fitted
    free[7 * R] = model.fit_width
    free[7 * R + 1] = model.fit_delay
    lo = np.full(model.np, -np.inf)
    hi = np.full(model.np, np.inf)
    lo[7 * R], hi[7 * R] = 0.0, 11.0
    lo[7 * R + 1], hi[7 * R + 1] = 0.0, 0.1
    for i in (7 * R, 7 * R + 1):                    # keep bounded starts just inside their bounds
        if free[i]:
            p0[i] = min(max(p0[i], lo[i] + 1e-3 * (hi[i] - lo[i])), hi[i] - 1e-3 * (hi[i] - lo[i]))

    def full(q):
        p = p0.copy()
        p[free] = q
        return p

    def res(q):
        p = full(q)
        return np.r_[(z - model.predict(p, sel)) / sg, model.priors(p)]
    lo_f, hi_f = lo[free], hi[free]
    q0 = np.clip(p0[free], np.where(np.isfinite(lo_f), lo_f, -1e9), np.where(np.isfinite(hi_f), hi_f, 1e9))
    sol = least_squares(res, q0, bounds=(lo_f, hi_f), loss="soft_l1", f_scale=2.0, x_scale="jac",
                        max_nfev=max_nfev)
    return full(sol.x)


def noise_model(model, p, idx=None):
    """Reading noise sd = a + b x distance, from robust spreads of the residuals in distance bins."""
    d = model.d
    sel = np.arange(len(d["z"])) if idx is None else idx
    h = model.predict(p, sel)
    r = d["z"][sel] - h
    bins = np.quantile(h, np.linspace(0, 1, 9))
    xs, ys, ws = [], [], []
    for lo, hi in zip(bins, bins[1:]):
        m = (h >= lo) & (h <= hi)
        if m.sum() < 30:
            continue
        rr = r[m]
        xs.append(float(np.median(h[m])))
        ys.append(1.4826 * float(np.median(np.abs(rr - np.median(rr)))))
        ws.append(m.sum())
    if len(xs) < 2:
        return 0.5, 0.03
    A = np.c_[np.ones(len(xs)), xs]
    w = np.sqrt(ws)
    a, b = np.linalg.lstsq(A * w[:, None], np.array(ys) * w, rcond=None)[0]
    return max(float(a), 0.1), max(float(b), 0.0)


def correlations(model, p, sigma):
    """(within-frame correlation of zone errors, lag-1 correlation of a zone's errors frame to frame)."""
    d = model.d
    e = (d["z"] - model.predict(p)) / sigma
    e = np.clip(e, -4, 4)
    key_frame = {}
    for i, (rid, t) in enumerate(zip(d["run"], d["t"])):
        key_frame.setdefault((rid, round(float(t), 4)), []).append(i)
    num = den = 0.0
    for ids in key_frame.values():
        if len(ids) > 1:
            v = e[ids]
            s = v.sum()
            num += (s * s - (v * v).sum()) / 2
            den += len(ids) * (len(ids) - 1) / 2
    var = float(np.mean(e * e))
    within = num / den / var if den else 0.0
    order = np.lexsort((d["t"], d["face"], d["zone"], d["run"]))
    a, b = [], []
    for i, j in zip(order, order[1:]):
        if (d["run"][i] == d["run"][j] and d["zone"][i] == d["zone"][j] and d["face"][i] == d["face"][j]
                and 0 < d["t"][j] - d["t"][i] < 0.15):
            a.append(e[i])
            b.append(e[j])
    lag1 = float(np.corrcoef(a, b)[0, 1]) if len(a) > 30 else 0.0
    return float(within), lag1


# ---------------------------------------------------------------- output
def cal_dict(model, p, noise, corr, used_runs, faces_ref, sig=None):
    off, bias, scale, width, delay, g = model.unpack(p)
    a_f, n_f = model.faces_at(g)
    faces = []
    for i, f in enumerate(model.faces):
        u = np.array([n_f[i][1], -n_f[i][0]])            # Face: n = (-u_y, u_x), so u = (n_y, -n_x)
        b = a_f[i] + f.length * u
        gname = model.groups[model.fg[i]]
        row = dict(name=f.name, x1_cm=round(float(a_f[i][0]), 3), y1_cm=round(float(a_f[i][1]), 3),
                   x2_cm=round(float(b[0]), 3), y2_cm=round(float(b[1]), 3), board=gname)
        if sig and gname in sig.get("board_cm", {}):
            row["sigma_cm"] = round(sig["board_cm"][gname], 3)
        faces.append(row)
    boards = {}
    for gi, gname in enumerate(model.groups):
        boards[gname] = dict(dx_cm=round(float(g[gi, 0]), 3), dy_cm=round(float(g[gi, 1]), 3),
                             turn_deg=round(float(g[gi, 2]), 3), readings=int(np.sum(model.fg[model.d["face"]] == gi)))
    robots = {}
    for ri, name in enumerate(model.robots):
        robots[name] = dict(zone_offset_deg=[round(float(v), 3) for v in off[ri]],
                            range_bias_cm=round(float(bias[ri]), 3), range_scale=round(1 + float(scale[ri]), 5),
                            readings=int(np.sum(model.d["robot"] == ri)))
        if sig:
            robots[name]["bias_sigma_cm"] = round(sig["bias_cm"].get(name, 0.3), 3)
            robots[name]["yaw_sigma_deg"] = round(sig["yaw_deg"].get(name, 1.0), 3)
    out = dict(format=lm.CAL_FORMAT, version=1, used_runs=sorted(used_runs), zone_width_deg=round(float(width), 3),
               delay_s=round(float(delay), 4), noise_cm=round(noise[0], 3), noise_rel=round(noise[1], 4),
               frame_corr=round(corr[0], 3), lag1_corr=round(corr[1], 3), robots=robots, boards=boards,
               landmarks=faces, map_given=faces_ref)
    if sig:
        out["face_sigma_cm"] = round(max(sig["board_cm"].values()) if sig["board_cm"] else 1.0, 3)
    return out


def jackknife(model, folds, p_full):
    """Spread of the leave-one-run-out fits (jackknife): how sure each board and sensor setting is."""
    n = len(folds)
    if n < 3:
        return None
    P = np.array(list(folds.values()))
    var = (n - 1) / n * ((P - P.mean(0)) ** 2).sum(0)
    sd = np.sqrt(var)
    off_sd, bias_sd, _, _, _, g_sd = model.unpack(sd)
    half = {}
    for gi, gname in enumerate(model.groups):
        L = max(f.length for f, fg in zip(model.faces, model.fg) if fg == gi)
        half[gname] = float(math.hypot(g_sd[gi, 0], g_sd[gi, 1]) + L / 2 * g_sd[gi, 2] * D2R)
    return dict(board_cm={k: max(v, 0.1) for k, v in half.items()},
                bias_cm={r: max(float(bias_sd[i]), 0.05) for i, r in enumerate(model.robots)},
                yaw_deg={r: max(float(off_sd[i].mean()), 0.1) for i, r in enumerate(model.robots)})


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", required=True)
    ap.add_argument("--sheet", required=True)
    ap.add_argument("--detections", required=True)
    ap.add_argument("--calib", required=True, help="floor_calibration.json")
    ap.add_argument("--calib-per-run", help="floor_calibration_loro.csv (as replay.py)")
    ap.add_argument("--recording")
    ap.add_argument("--video-offset", type=float)
    ap.add_argument("--map", required=True, help="map.json or landmarks.csv: the boards' approximate positions")
    ap.add_argument("--out", required=True)
    ap.add_argument("--calib-results", help="calibration_results.csv (tools/blink/calib.py): also use the "
                    "calibration runs' 360 deg spins (firmware 2026-10-18b+), with calib.py's start heading and "
                    "gyro scale")
    ap.add_argument("--reference", choices=["graph", "filter"], default="graph",
                    help="camera + wheels + gyro path used as the reference (default: factor graph)")
    ap.add_argument("--max-incidence", type=float, default=45.0, help="deg; steeper hits are left out")
    ap.add_argument("--board-sigma", type=float, default=PRIOR["board_cm"],
                    help="how far off the map's board positions may be, cm (prior; default 2)")
    ap.add_argument("--no-width", action="store_true", help="keep the zone width at 0 (one ray per zone)")
    ap.add_argument("--no-delay", action="store_true", help="keep the frame delay at 0")
    ap.add_argument("--no-loro", action="store_true", help="skip the leave-one-run-out fits")
    args = ap.parse_args(argv)

    all_runs, fits = rn.load_runs(args.session, args.sheet)
    runs = [r for r in all_runs if not r.excluded and r.tape_start]
    vc = rn.video_clock(args.session, args.recording, fits, args.video_offset)
    if vc is None:
        sys.exit("Couldn't line up video and robot clocks: give --recording (with flashes) or --video-offset")
    off, how = vc
    print(how)
    calib_of, _ = rn.assign_calibrations([r.run_id for r in runs], args.calib, args.calib_per_run)
    bad = rn.in_sample(calib_of)
    if bad:
        print(f"note: {len(bad)} run(s) get camera positions from a floor calibration made with them; the reference "
              "paths there are as good as that calibration (give --calib-per-run to avoid it)")
    dets = []
    for p in sorted(set(calib_of.values())):
        _, Hinv = rn.load_homography(p)
        rid = {r for r, q in calib_of.items() if q == p}
        span = [(r.t_start - 1, r.t_end + 1) for r in runs if r.run_id in rid]
        for dd in rn.load_detections(args.detections, Hinv, off, parallax=rn.load_parallax(p),
                                     dist=rn.load_distortion(p)):
            if any(a <= dd["t"] <= b for a, b in span):
                dets.append(dd)
    dets.sort(key=lambda dd: dd["t"])
    crow, spins = None, []
    if args.calib_results:
        crow = rn.load_calib_results(args.calib_results, off)
        for r in rn.calib_scales(crow).items():
            for c in crow:
                if c["robot"] == r[0]:
                    c["gs"] = r[1].get("gs", 1.0)
        spins = [r for r in all_runs if r.experiment == "calib" and not r.excluded]
        _, Hinv = rn.load_homography(args.calib)
        span = [(r.t_start - 1, r.t_end + 1) for r in spins]
        for dd in rn.load_detections(args.detections, Hinv, off, parallax=rn.load_parallax(args.calib),
                                     dist=rn.load_distortion(args.calib)):
            if any(a <= dd["t"] <= b for a, b in span):
                dets.append(dd)
        dets.sort(key=lambda dd: dd["t"])
    faces = lm.load(args.map)
    print(f"{len(runs)} runs, {len(dets)} camera sightings, {len(faces)} mapped faces")
    signs = (eng.wheel_turn_signs(runs), eng.wheel_fwd_signs(runs))
    print(f"reference paths (camera + wheels + gyro, {args.reference}) ...")
    refs = reference_paths(runs, dets, method=args.reference, signs=signs)
    steps = {}
    if spins:
        wsign = eng.wheel_turn_signs(all_runs)
        gyro_f = eng.gyro_factors(all_runs, wsign)
        n_ok = 0
        for r in spins:
            ref = spin_reference(r, dets, crow, gyro_f.get(r.robot, (D2R, ""))[0])
            if ref is not None:
                refs[r.run_id] = ref
                steps[r.run_id] = SPIN_STEPS
                n_ok += 1
        print(f"calibration runs: {n_ok} of {len(spins)} with a camera position and a measured start heading "
              "(their spins are used)")
    cfg = eng.config(tof_max_incidence_deg=args.max_incidence)
    data, robots = collect(runs + [r for r in spins if r.run_id in steps], refs, faces, cfg, steps=steps)
    if data is None:
        sys.exit("No distance-sensor reading could be tied to a mapped face: check the map and that the runs drive "
                 "near the boards")
    n = len(data["z"])
    print(f"{n} zone readings tied to a face ({len(refs)} runs, robots {', '.join(robots)})")
    model = Model(data, robots, faces, cfg, fit_width=not args.no_width, fit_delay=not args.no_delay,
                  board_cm=args.board_sigma)
    p = model.x0(0.0, 0.0)
    noise = (0.5, 0.03)
    for _ in range(2):                                   # fit, re-estimate the noise, fit again
        sigma = noise[0] + noise[1] * np.abs(data["z"])
        p = fit(model, sigma, p0=p)
        noise = noise_model(model, p)
    sigma = noise[0] + noise[1] * np.abs(data["z"])
    corr = correlations(model, p, sigma)
    used = sorted(set(data["run"]))
    folds = {}
    if not args.no_loro:
        print(f"leave-one-run-out fits ({len(used)}) ...")
        for rid in used:
            folds[rid] = fit(model, sigma, idx=np.flatnonzero(data["run"] != rid), p0=p, max_nfev=30)
    sig = jackknife(model, folds, p) if folds else None
    faces_ref = os.path.abspath(args.map)
    os.makedirs(os.path.join(args.out, "loro"), exist_ok=True)
    full = cal_dict(model, p, noise, corr, used, faces_ref, sig)
    with open(os.path.join(args.out, "tof_calibration.json"), "w") as f:
        json.dump(full, f, indent=1)
    lm.write_csv([lm.Face(r["name"], (r["x1_cm"], r["y1_cm"]), (r["x2_cm"], r["y2_cm"])) for r in full["landmarks"]],
                 os.path.join(args.out, "landmarks_refined.csv"))
    if folds:
        with open(os.path.join(args.out, "tof_calibration_loro.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["run_id", "calibration"])
            for rid, pf in folds.items():
                fn = os.path.join("loro", f"{rid}.json")
                with open(os.path.join(args.out, fn), "w") as g:
                    json.dump(cal_dict(model, pf, noise, corr, [r for r in used if r != rid], faces_ref, sig), g, indent=1)
                w.writerow([rid, fn])
    h0 = model.predict(model.x0())
    h1 = model.predict(p)
    with open(os.path.join(args.out, "residuals.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["run", "robot", "zone", "face", "t", "reading_cm", "resid_nominal_cm", "resid_calibrated_cm"])
        for i in range(n):
            w.writerow([data["run"][i], robots[data["robot"][i]], lm.ZONE_COLS[data["zone"][i]],
                        faces[data["face"][i]].name, f"{data['t'][i]:.4f}", f"{data['z'][i]:.2f}",
                        f"{data['z'][i] - h0[i]:.3f}", f"{data['z'][i] - h1[i]:.3f}"])
    r0, r1 = data["z"] - h0, data["z"] - h1
    rms = lambda v: math.sqrt(float(np.mean(np.clip(v, -5, 5) ** 2)))
    print(f"\nresidual (reading minus prediction): nominal mean {np.mean(r0):+.3f} cm, rms {rms(r0):.3f}; "
          f"calibrated mean {np.mean(r1):+.3f}, rms {rms(r1):.3f}")
    print(f"zone width {full['zone_width_deg']:.2f} deg, frame delay {1000 * full['delay_s']:.0f} ms, noise "
          f"{full['noise_cm']:.2f} cm + {100 * full['noise_rel']:.1f}% of the distance; errors alike within a frame "
          f"{full['frame_corr']:+.2f}, frame to frame {full['lag1_corr']:+.2f}")
    for name, r in full["robots"].items():
        print(f"  {name}: zones turned {', '.join(f'{v:+.1f}' for v in r['zone_offset_deg'])} deg, range bias "
              f"{r['range_bias_cm']:+.2f} cm, scale {r['range_scale']:.4f} ({r['readings']} readings)")
    for name, b in full["boards"].items():
        s = f" (+-{sig['board_cm'][name]:.2f} cm)" if sig else ""
        print(f"  board {name}: moved {b['dx_cm']:+.2f}, {b['dy_cm']:+.2f} cm, turned {b['turn_deg']:+.2f} deg{s}"
              f" ({b['readings']} readings)")
    print(f"wrote {args.out}/tof_calibration.json, landmarks_refined.csv, residuals.csv"
          + (", tof_calibration_loro.csv, loro/" if folds else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
