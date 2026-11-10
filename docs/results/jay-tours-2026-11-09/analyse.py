"""Jay-Sun's 7 school tours (9 Nov): stop accuracy against the points, repeatability, the camera's stated +- vs its
scatter, raw odometry drift, timing. Reads the run folders; writes stops_all.csv and summary.json."""
import csv, glob, json, math, os, sys, zipfile, io
import numpy as np

D = sys.argv[1]
OUT = sys.argv[2]
runs = sorted(glob.glob(os.path.join(D, "2026*")))


def rd(p):
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


# typed points (cm) from the bundle's site copy
z = glob.glob(os.path.join(runs[0], "jay-raw-*.zip"))[0]
with zipfile.ZipFile(z) as zf:
    name = [n for n in zf.namelist() if n.endswith("school-conference-points.csv")][0]
    pts = {r["id"]: (float(r["x_cm"]), float(r["y_cm"])) for r in csv.DictReader(io.TextIOWrapper(zf.open(name)))}

rows = []
csv_pts = dict(pts)
for ri, r in enumerate(runs):
    pts = {k: tuple(v[:2]) for k, v in json.load(open(os.path.join(r, "mission.json")))["spots"].items()}
    rn = os.path.basename(r)
    st = rd(os.path.join(r, "run", "stops.csv"))
    ct = rd(os.path.join(r, "run", "camera_track.csv"))
    ct_t = np.array([float(c["t"]) for c in ct]); ct_x = np.array([float(c["x_cm"]) for c in ct])
    ct_y = np.array([float(c["y_cm"]) for c in ct]); ct_s = np.array([float(c["sigma_cm"] or 0) for c in ct])
    od = rd(os.path.join(r, "run", "odometry_raw.csv"))
    od_t = np.array([float(o["t"]) for o in od]); od_x = np.array([float(o["x_odo_cm"]) for o in od])
    od_y = np.array([float(o["y_odo_cm"]) for o in od]); od_d = np.array([float(o["dist_cm"]) for o in od])
    od_seg = np.array([o["segment"] for o in od])
    prev_end = None
    t_first = float(st[0]["dwell_start_unix"])
    for s in st:
        t0, t1 = float(s["dwell_start_unix"]), float(s["dwell_end_unix"])
        p = s["point"]; tx, ty = pts[p]
        cx, cy = float(s["cam_x_cm"]), float(s["cam_y_cm"])
        m = (ct_t >= t0) & (ct_t <= t1)
        sx = float(np.std(ct_x[m])) if m.sum() > 2 else float("nan")
        sy = float(np.std(ct_y[m])) if m.sum() > 2 else float("nan")
        # settle: from arrival, first moment the camera position stays within 0.5 cm of the dwell median for the rest
        settle = float("nan")
        if m.sum() > 3:
            tt, xx, yy = ct_t[m], ct_x[m], ct_y[m]
            dev = np.hypot(xx - np.median(xx), yy - np.median(yy))
            ok = dev <= 0.5
            for k in range(len(ok)):
                if ok[k:].all():
                    settle = tt[k] - t0
                    break
        # raw odometry at the dwell's middle
        tm = 0.5 * (t0 + t1)
        k = int(np.searchsorted(od_t, tm)); k = min(max(k, 0), len(od_t) - 1)
        ox, oy = od_x[k], od_y[k]
        # leg: from the previous dwell end to this arrival
        leg_s = (t0 - prev_end) if prev_end else float("nan")
        rows.append(dict(run=rn, run_i=ri + 1, n=int(s["n"]), point=p, tx=tx, ty=ty, cx=cx, cy=cy,
                         err=math.hypot(cx - tx, cy - ty), ex=cx - tx, ey=cy - ty, pm=float(s["plus_minus_cm"]),
                         n_fixes=int(s["n_fixes"]), scat=math.hypot(sx, sy) if not math.isnan(sx) else float("nan"),
                         dwell=float(s["dwell_s"]), settle=settle, odo_err=math.hypot(ox - cx, oy - cy),
                         odo_dist=float(od_d[k]), odo_seg=od_seg[k], t_into=(t0 - t_first) / 60.0, leg_s=leg_s,
                         own_err=math.hypot(float(s["own_x_cm"]) - cx, float(s["own_y_cm"]) - cy),
                         t0=t0, zone=1 if int(p[1:]) <= 10 else 2))
        prev_end = t1

os.makedirs(OUT, exist_ok=True)
with open(os.path.join(OUT, "stops_all.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]))
    w.writeheader(); w.writerows(rows)

A = lambda k: np.array([r[k] for r in rows], float)
err, pm, scat, settle, dwell = A("err"), A("pm"), A("scat"), A("settle"), A("dwell")
# repeatability per point: spread of the camera stop positions across the 7 tours
per = {}
for r in rows:
    per.setdefault(r["point"], []).append((r["cx"], r["cy"]))
rep = {p: float(np.sqrt(np.mean(np.sum((np.array(v) - np.mean(v, 0)) ** 2, 1)))) for p, v in per.items()}
bias = {p: (float(np.mean(np.array(v)[:, 0]) - pts[p][0]), float(np.mean(np.array(v)[:, 1]) - pts[p][1])) for p, v in per.items()}
summ = dict(corrected={k: [csv_pts[k], pts[k]] for k in pts if k in csv_pts and math.hypot(csv_pts[k][0]-pts[k][0], csv_pts[k][1]-pts[k][1]) > 0.5},
    stops=len(rows), runs=len(runs),
    err_median=float(np.median(err)), err_p95=float(np.percentile(err, 95)), err_max=float(err.max()),
    err_mean_xy=[float(A("ex").mean()), float(A("ey").mean())],
    rep_rms_median=float(np.median(list(rep.values()))), rep_rms_max=float(max(rep.values())),
    rep_worst=max(rep, key=rep.get),
    pm_median=float(np.median(pm)), scat_median=float(np.nanmedian(scat)), scat_p95=float(np.nanpercentile(scat, 95)),
    settle_median=float(np.nanmedian(settle)), settle_p95=float(np.nanpercentile(settle, 95)),
    settle_max=float(np.nanmax(settle)),
    dwell_min=float(dwell.min()), dwell_max=float(dwell.max()),
    own_err_max=float(A("own_err").max()),
    odo_err_end_by_run={r["run"]: round(r["odo_err"], 1) for r in rows if r["n"] == 20},
    odo_err_median_by_run={rn: float(np.median([r["odo_err"] for r in rows if r["run"] == rn])) for rn in sorted(set(r["run"] for r in rows))},
    odo_segments={rn: sorted(set(r["odo_seg"] for r in rows if r["run"] == rn)) for rn in sorted(set(r["run"] for r in rows))},
    err_by_zone={z_: float(np.median([r["err"] for r in rows if r["zone"] == z_])) for z_ in (1, 2)},
    flagged_pts_err_median=float(np.median([r["err"] for r in rows if r["point"] in ("P03","P05","P12","P13","P15","P18","P19","P20")])),
    other_pts_err_median=float(np.median([r["err"] for r in rows if r["point"] not in ("P03","P05","P12","P13","P15","P18","P19","P20")])),
    leg_s_median=float(np.nanmedian(A("leg_s"))),
    rep=rep, bias=bias,
)
json.dump(summ, open(os.path.join(OUT, "summary.json"), "w"), indent=1)
print(json.dumps({k: v for k, v in summ.items() if k not in ("rep", "bias")}, indent=1))
