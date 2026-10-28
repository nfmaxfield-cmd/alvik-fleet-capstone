#!/usr/bin/env python3
"""Measure the Pi's tracking load, step by step (item 2 of the one-system build; trackfarm.py), on recorded or
simulated pictures, and check the positions against the full-picture tracker on the same pictures.

    python3 tools/fleet/bench_trackfarm.py real --l0 <folder with cam0.avi, rec/, setup/> --out DIR
    python3 tools/fleet/bench_trackfarm.py sim --truth <dry run>/sim/truth.csv --map <map.json> --out DIR [--washout]
    python3 tools/fleet/bench_trackfarm.py procs --l0 ... --out DIR      # the process split at Pi speed

Rows (each adds one step):
  A  before: one process, full pictures, /live JPEG decoded in the tracker (e02_twocam.TwoCamTracker, as the bridge)
  a  + shared pictures (no JPEG decode: the recorder's decoded picture from shared memory)
  b  + windows round each robot, a full scan every ROI_EVERY pictures and whenever a robot is not seen for sure
  c  + half size (a setting)
  e  + still pictures skipped (b and e together; c off)
  d  the process split: each camera's worker in its own process on its own core (procs)
Cost per picture: CPU time of the camera's own work (thread time), on this machine and x PI_FACTOR for the Pi.
Accuracy: each robot's tracked spot against row A's, picture by picture (both "seen").
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "sim"),
          TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2  # noqa: E402

PI_FACTOR = 3.5     # the Pi 4 against this machine, per picture (earlier Pi measurements 26-73 ms against ~14 ms here;
                    # e02-oct27-two-camera-tracking.md). An estimate, not measured on this code: said so in the report
SIDE_HZ = 10.0      # e02_twocam.SIDE_HZ: side pictures used a second


def load_avi(path, fcsv, t0, t1):
    ts = [float(r["t_wall"]) for r in csv.DictReader(open(fcsv))]
    cap = cv2.VideoCapture(path)
    out, i = [], 0
    while True:
        ok, im = cap.read()
        if not ok:
            break
        if i < len(ts) and t0 <= ts[i] <= t1:
            out.append((cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), ts[i]))
        i += 1
    return out


def real_data(d):
    """The 27 Oct 13:01 L0 recording: 30 s, robots standing still on the washed-out board (the replay of
    e02-oct27-two-camera-tracking.md, 'Fix after the real L0 at 13:01')."""
    import e02_camera as CAM
    S = os.path.join(d, "setup", "20261027-125333-p0-camera-setup")
    cam = CAM.load_camera(os.path.join(S, "overhead_floorcal_20261027-125803.json"))
    scam = CAM.load_camera(os.path.join(S, "side_floorcal_20261027-125812.json"))
    T0, T1 = 1793120480, 1793120510
    ov = load_avi(os.path.join(d, "cam0.avi"), os.path.join(d, "rec", "20261027-130102_P0", "cam0", "frames.csv"), T0, T1)
    sd = load_avi(os.path.join(d, "rec", "20261027-130102_P0", "cam1", "video.avi"),
                  os.path.join(d, "rec", "20261027-130102_P0", "cam1", "frames.csv"), T0, T1)
    spots = {"Left": (73.6, 77.59), "Right": (87.05, 41.55), "Center": (37.75, 45.44)}
    return dict(cam=cam, scam=scam, side_lag=0.04, rect=(0, 0, 122, 122), ov=ov, sd=sd, spots=spots, tb=1793120498,
                truth=None, odo=None)


def sim_data(truth_csv, map_path, washout=False, seconds=60.0, fps=25.0, t_from=0.0):
    """Pictures of a simulated run (its true poses, tools/sim/table_cam.py's two cameras), robots moving."""
    import table_cam as TC
    import e02_camera as CAM
    import fleetmap as FM
    m = FM.load(map_path)
    f = m["floor"]
    rect = (f["x_min"], f["y_min"], f["x_max"], f["y_max"])
    rows = {}
    for r in csv.DictReader(open(truth_csv)):
        rows.setdefault(r["robot"], []).append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"]),
                                                math.radians(float(r["heading_deg"]))))
    t0 = min(v[0][0] for v in rows.values()) + t_from
    tc = TC.TableCamera(rect, seed=12, px_per_cm=4.2)
    out = os.path.join("/tmp", "bench_trackfarm_%d" % os.getpid())
    os.makedirs(out, exist_ok=True)
    tc.true_model(os.path.join(out, "ov.json"))
    if washout:
        tc = TC.washed_out(tc)
    sc = TC.SideCamera(rect, seed=18)
    sc.true_model(os.path.join(out, "side.json"))
    cam = CAM.load_camera(os.path.join(out, "ov.json"))
    scam = CAM.load_camera(os.path.join(out, "side.json"))

    def at(name, t):
        s = rows[name]
        lo, hi = 0, len(s) - 1
        if t <= s[0][0]:
            return s[0][1:]
        if t >= s[-1][0]:
            return s[-1][1:]
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if s[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        a, b = s[lo], s[hi]
        k = (t - a[0]) / max(1e-6, b[0] - a[0])
        return a[1] + k * (b[1] - a[1]), a[2] + k * (b[2] - a[2]), a[3] + k * math.remainder(b[3] - a[3], 2 * math.pi)
    ov, sd, truth = [], [], []
    n = int(seconds * fps)
    for k in range(n):
        t = t0 + k / fps
        poses = [at(nm, t) for nm in rows]
        img = tc.render([(x, y, th) for x, y, th in poses])
        ov.append((img, t))
        sd.append((sc.render([(x, y, th) for x, y, th in poses]), t + 0.04))
        truth.append((t, {nm: p for nm, p in zip(rows, poses)}))
    spots = {nm: at(nm, t0)[:2] for nm in rows}
    odo = {nm: [(t, *at(nm, t)) for t in np.arange(t0 - 1, t0 + seconds + 1, 0.1)] for nm in rows}
    return dict(cam=cam, scam=scam, side_lag=0.04, rect=rect, ov=ov, sd=sd, spots=spots, tb=t0 + 5 / fps - 1e-6,
                truth=truth, odo=odo)


def jpeg_decode_ms(frames, k=40):
    """What the tracker paid to decode the recorder's /live JPEG (camrec live_part: quality 95, grey)."""
    ms = []
    for img, _ in frames[:k]:
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        c0 = time.thread_time()
        cv2.imdecode(np.frombuffer(jpg.tobytes(), np.uint8), cv2.IMREAD_GRAYSCALE)
        ms.append((time.thread_time() - c0) * 1000.0)
    return float(np.median(ms))


def _fusion(D, odo):
    import cam_bridge as CB
    import trackfarm as TF
    import e02_twocam as TW
    base = CB.make_tracker_class()
    F = TF.make_fusion_class(base)
    cx, cy = (D["rect"][0] + D["rect"][2]) / 2.0, (D["rect"][1] + D["rect"][3]) / 2.0
    fu = F(D["cam"], board_center=(cx, cy), board_rect=D["rect"], margin_cm=8.0, side_cam=D["scam"],
           side_lag_s=D["side_lag"], lights=True)
    for n, (x, y) in D["spots"].items():
        fu.add(n, x, y, heading=90.0)
    if odo:
        for n, lst in odo.items():
            for t, x, y, th in lst:
                fu.own_pose(n, x, y, math.degrees(th), t)
    return fu


def run_baseline(D, odo=True):
    """Row A: TwoCamTracker (the bridge's OdoTracker) on every overhead picture and the side at SIDE_HZ, one thread."""
    import cam_bridge as CB
    T = CB.make_tracker_class()
    cx, cy = (D["rect"][0] + D["rect"][2]) / 2.0, (D["rect"][1] + D["rect"][3]) / 2.0
    trk = T(D["cam"], board_center=(cx, cy), board_rect=D["rect"], margin_cm=8.0, side_cam=D["scam"],
            side_lag_s=D["side_lag"], lights=True)
    tb = D["tb"]
    spots = list(D["spots"].values())
    trk.set_background([im for im, t in D["ov"] if t < tb][-5:], robots=spots)
    trk.set_side_background([im for im, t in D["sd"] if t < tb][-5:], robots=spots)
    for n, (x, y) in D["spots"].items():
        trk.add(n, x, y, heading=90.0)
    if odo and D.get("odo"):
        for n, lst in D["odo"].items():
            for t, x, y, th in lst:
                trk.own_pose(n, x, y, math.degrees(th), t)
    ev = sorted([(t, "o", im) for im, t in D["ov"] if t >= tb] + [(t, "s", im) for im, t in D["sd"] if t >= tb],
                key=lambda q: q[0])
    cost = {"overhead": [], "side": []}
    pos = []
    for t, k, im in ev:
        c0 = time.thread_time()
        if k == "o":
            trk.push(im, t)
            cost["overhead"].append((time.thread_time() - c0) * 1000.0)
            pos.append((t, {n: (s["x"], s["y"], s["status"]) for n, s in trk.states().items()}))
        else:
            before = trk.side_n
            trk.push_side(im, t)
            if trk.side_n > before:
                cost["side"].append((time.thread_time() - c0) * 1000.0)
    return dict(cost=cost, pos=pos, counts=dict(trk.counts))


def run_farm(D, half=False, roi=True, still=False, roi_every=None):
    """Rows a-e: the two workers inline (no processes: the same outlines, deterministic), the fusion fed in time
    order, its snapshot passed back after every record."""
    import trackfarm as TF
    tb = D["tb"]
    spots = dict(D["spots"])
    kw = dict(half=half, roi=roi, still_skip=still)
    if roi_every:
        kw["roi_every"] = roi_every
    wo = TF.CamWorker("overhead", D["cam"], D["rect"], 8.0, spots, **kw)
    ws = TF.CamWorker("side", D["scam"], D["rect"], 8.0, spots, lights=False, **kw)
    fu = _fusion(D, D.get("odo"))
    # floor memory from the same 5 pictures as the baseline
    first_o = [im for im, t in D["ov"] if t < tb][-5:]
    first_s = [im for im, t in D["sd"] if t < tb][-5:]
    for im in first_o:
        wo.step(im, 0.0)
    for im in first_s:
        ws.step(im, 0.0)
    # (the side's dark-picture gain: as set_side_background would)
    ev = sorted([(t, "o", im) for im, t in D["ov"] if t >= tb] + [(t, "s", im) for im, t in D["sd"] if t >= tb],
                key=lambda q: q[0])
    cost = {"overhead": [], "side": []}
    modes = {"overhead": {}, "side": {}}
    pos = []
    side_t = None
    for t, k, im in ev:
        if k == "s":
            if side_t is not None and t - side_t < 1.0 / SIDE_HZ - 1e-3:
                continue
            side_t = t
        w = wo if k == "o" else ws
        w.snapshot(fu.snapshot(t))
        rec = w.step(im, t)
        ms, mode = w.cost[-1][1], w.cost[-1][2]
        cam = "overhead" if k == "o" else "side"
        cost[cam].append(ms)
        modes[cam][mode] = modes[cam].get(mode, 0) + 1
        if rec is not None:
            c0 = time.thread_time()
            fu.apply(rec)
            cost.setdefault("fusion", []).append((time.thread_time() - c0) * 1000.0)
        if k == "o":
            pos.append((t, {n: (s["x"], s["y"], s["status"]) for n, s in fu.states().items()}))
    return dict(cost=cost, pos=pos, modes=modes, counts=dict(fu.counts))


def compare(base, other):
    """Per robot: picture by picture (same overhead picture time), both seen: distance between the two trackers'
    spots; and seen-share."""
    bm = {round(t, 4): d for t, d in base["pos"]}
    out = {}
    for t, d in other["pos"]:
        b = bm.get(round(t, 4))
        if b is None:
            continue
        for n, (x, y, st) in d.items():
            o = out.setdefault(n, dict(d=[], seen=0, seen_base=0, n=0))
            o["n"] += 1
            o["seen"] += st == "seen"
            o["seen_base"] += b[n][2] == "seen"
            if st == "seen" and b[n][2] == "seen":
                o["d"].append(math.hypot(x - b[n][0], y - b[n][1]))
    res = {}
    for n, o in out.items():
        d = sorted(o["d"])
        res[n] = dict(median_cm=round(d[len(d) // 2], 3) if d else None,
                      p95_cm=round(d[min(len(d) - 1, int(0.95 * len(d)))], 3) if d else None,
                      max_cm=round(d[-1], 3) if d else None,
                      seen_pct=round(100.0 * o["seen"] / max(1, o["n"]), 1),
                      seen_base_pct=round(100.0 * o["seen_base"] / max(1, o["n"]), 1))
    return res


def against_truth(run, truth):
    tm = {round(t, 4): d for t, d in truth}
    errs = {}
    for t, d in run["pos"]:
        tr = tm.get(round(t, 4))
        if tr is None:
            continue
        for n, (x, y, st) in d.items():
            if st == "seen":
                errs.setdefault(n, []).append(math.hypot(x - tr[n][0], y - tr[n][1]))
    return {n: dict(median_cm=round(sorted(v)[len(v) // 2], 3), p95_cm=round(sorted(v)[int(0.95 * (len(v) - 1))], 3))
            for n, v in errs.items()}


def med(v):
    return round(float(np.median(v)), 2) if v else None


def p90(v):
    return round(float(np.percentile(v, 90)), 2) if v else None


def table(D, label):
    dec_o = jpeg_decode_ms(D["ov"])
    dec_s = jpeg_decode_ms(D["sd"])
    rows = []
    A = run_baseline(D)
    rows.append(("A before: one process, full pictures, JPEG decoded", A, dec_o, dec_s))
    variants = [("a + shared pictures (no JPEG decode)", dict(roi=False, still=False, half=False)),
                ("b + windows (full scan every %d)" % __import__("trackfarm").ROI_EVERY, dict(roi=True, still=False, half=False)),
                ("c + half size", dict(roi=True, still=False, half=True)),
                ("e + still pictures skipped (b + e)", dict(roi=True, still=True, half=False))]
    res = {"label": label, "pi_factor": PI_FACTOR, "jpeg_decode_ms": {"overhead": dec_o, "side": dec_s}, "rows": []}
    out = [A]
    for name, kw in variants:
        R = run_farm(D, **kw)
        out.append(R)
        rows.append((name, R, 0.0, 0.0))
    for (name, R, do, ds) in rows:
        co, cs = R["cost"]["overhead"], R["cost"]["side"]
        row = dict(row=name, overhead_ms=med(co), overhead_p90_ms=p90(co), side_ms=med(cs), side_p90_ms=p90(cs),
                   decode_ms=dict(overhead=do, side=ds),
                   overhead_pi_ms=round((med(co) + do) * PI_FACTOR, 1), side_pi_ms=round((med(cs) + ds) * PI_FACTOR, 1),
                   fusion_ms=med(R["cost"].get("fusion", [])), modes=R.get("modes"),
                   vs_full=compare(A, R) if R is not A else None,
                   vs_truth=against_truth(R, D["truth"]) if D.get("truth") else None,
                   counts=R.get("counts"))
        res["rows"].append(row)
    return res


def procs(D, out_dir, seconds=None, slow=PI_FACTOR):
    """Row d: the two workers as processes, each pinned to its own core (cores 0 and 1 here; the Pi: 1 and 2), the
    pictures in two FrameRings written at the cameras' real rate, each worker made `slow` times slower (it busy-waits
    (slow - 1) x its own cost after each picture: the Pi's speed). Reports pictures tracked a second per camera and
    how far behind the newest picture each worker ran."""
    import multiprocessing as mp
    import e02_shmframes as SF
    import trackfarm as TF
    rect = D["rect"]
    cal = os.path.join(out_dir, "procs")
    os.makedirs(cal, exist_ok=True)
    D["cam"].save(os.path.join(cal, "ov.json")) if hasattr(D["cam"], "save") else None
    D["scam"].save(os.path.join(cal, "side.json")) if hasattr(D["scam"], "save") else None
    with open(os.path.join(cal, "side.json")) as f:
        sd = json.load(f)
    sd["lag_s"] = D["side_lag"]
    with open(os.path.join(cal, "side.json"), "w") as f:
        json.dump(sd, f)
    H, W = D["ov"][0][0].shape
    rings = {"overhead": SF.FrameRing.create(slots=64, height=H, width=W),
             "side": SF.FrameRing.create(slots=64, height=D["sd"][0][0].shape[0], width=D["sd"][0][0].shape[1])}
    conns, ps = {}, []
    ncpu = os.cpu_count() or 1
    for k, (kind, calib) in enumerate((("overhead", "ov.json"), ("side", "side.json"))):
        a, b = mp.Pipe()
        cfg = dict(kind=kind, calib=os.path.join(cal, calib), rect=list(rect), starts={n: list(v) for n, v in
                                                                                       D["spots"].items()},
                   ring=rings[kind].name, core=k % ncpu, max_hz=30.0 if kind == "overhead" else SIDE_HZ,
                   capture_s=0.0, slow=slow)
        p = mp.Process(target=_slow_worker, args=(b, cfg), daemon=True)
        p.start()
        conns[kind] = a
        ps.append(p)
    fu = _fusion(D, D.get("odo"))
    ev = sorted([(t, "overhead", im) for im, t in D["ov"]] + [(t, "side", im) for im, t in D["sd"]],
                key=lambda q: q[0])
    if seconds:
        ev = [e for e in ev if e[0] - ev[0][0] <= seconds]
    t_base = ev[0][0]
    wall0 = time.time()
    got = {"overhead": [], "side": []}
    i = 0
    while i < len(ev) or time.time() - wall0 < ev[-1][0] - t_base + 2.0:
        now = time.time() - wall0
        while i < len(ev) and ev[i][0] - t_base <= now:
            t, kind, im = ev[i]
            rings[kind].write(im, t, t_pi=t)
            i += 1
        for kind, c in conns.items():
            while c.poll():
                rec = c.recv()
                if "cam" in rec:
                    fu.apply(rec)
                    got[kind].append((time.time() - wall0, rec["t"] - t_base, rec.get("ms"), rec.get("mode")))
                    c.send({"snap": fu.snapshot(rec["t"])})
        time.sleep(0.002)
        if i >= len(ev) and time.time() - wall0 > ev[-1][0] - t_base + 2.0:
            break
    for c in conns.values():
        c.send({"stop": True})
    for p in ps:
        p.join(timeout=5)
    for r in rings.values():
        r.close()
    span = ev[-1][0] - t_base
    out = {}
    for kind, lst in got.items():
        behind = [w - t for w, t, _, _ in lst]
        out[kind] = dict(tracked_per_s=round(len(lst) / max(1e-6, span), 2), median_behind_s=med(behind),
                         p90_behind_s=p90(behind), cost_ms_at_pi_speed=med([m for _, _, m, _ in lst if m]))
    return out


def _slow_worker(conn, cfg):
    """run_worker, made cfg["slow"] times slower (Pi speed): each step busy-waits (slow - 1) x its own CPU time."""
    import trackfarm as TF
    slow = float(cfg.get("slow", 1.0))
    orig = TF.CamWorker.step

    def step(self, img, t):
        c0 = time.thread_time()
        r = orig(self, img, t)
        spent = time.thread_time() - c0
        end = time.thread_time() + spent * (slow - 1.0)
        while time.thread_time() < end:
            pass
        if r is not None:
            r["ms"] = round(spent * slow * 1000.0, 2)
        return r
    TF.CamWorker.step = step
    TF.run_worker(conn, cfg)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("what", choices=["real", "sim", "procs"])
    ap.add_argument("--l0")
    ap.add_argument("--truth")
    ap.add_argument("--map")
    ap.add_argument("--washout", action="store_true")
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--from-s", type=float, default=0.0, help="sim: start this far into the run")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    if a.what in ("real", "procs") and a.l0:
        D = real_data(a.l0)
    else:
        D = sim_data(a.truth, a.map, washout=a.washout, seconds=a.seconds, t_from=a.from_s)
    if a.what == "procs":
        res = {"procs": procs(D, a.out)}
    else:
        res = table(D, a.what + (" washed out" if a.washout else ""))
    path = os.path.join(a.out, "bench_%s%s.json" % (a.what, "_washout" if a.washout else ""))
    with open(path, "w") as f:
        json.dump(res, f, indent=1)
    print(json.dumps(res, indent=1)[:6000])
    print("->", path)


if __name__ == "__main__":
    main()
