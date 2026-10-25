"""Analysis A (2026-10-24 night): replay the Pi plan's stage 1 and 2 through the sensor-trust code (e02_trust).

What it feeds, in time order, the way e02.py does:
  camera  at every find: the camera's error there (camera_error_samples, source "find": dx, dy = camera - true),
          corrected by the camera error map (e02_camerr) built from the finds before it (as the runner does: the map
          only holds earlier finds when a find is checked) -- and, as a second variant, from all the OTHER finds of
          the session (leave one out);
  wheels  after every U run: that run's stretches (rebuilt from track.csv + raw.csv, night_common.rebuild_stretches)
  gyro    against the robot's pooled fit (e02_fit.fit_robot over all its stretches so far, with the gate the trust
          sets at that moment), e02.py stretch_rms_against;
  front   every front_checks.csv row with a measured heading, the thing ahead within 50 cm and met within 45 deg of
          head on: reading - FRONT_OFFSET_CM (0) - the map distance.
Variants: each stage on its own (a fresh trust file) and stage 1 then stage 2 (the trust file carried, one
new_session between them, as logs/sensor_trust.json would be).

Usage: python3 night_trust_replay.py            (prints the tables used in the report)
       python3 night_trust_replay.py --split 3  (the stretches found with the camera track cut at 3 cm jumps: B)"""
import math
import sys

import night_common as N
from night_common import FIT, TR, CE

FRONT_OFFSET_CM, FRONT_CHECK_RANGE_CM, FRONT_MIN_COS = 0.0, 50.0, 0.7    # e02.py
SPLIT = float(sys.argv[sys.argv.index("--split") + 1]) if "--split" in sys.argv else None


def prior(st, robot):
    p = dict(FIT.PRIOR)
    d = ((st.get("naming_check") or {}).get("gyro_raw_drift_dps") or {}).get(robot)
    if d is not None:
        p["gyro_drift_dps"] = round(p["gyro_scale"] * d, 4)
    return p, d is not None


def stretch_rms_against(obs, fitted, pr):
    """e02.py stretch_rms_against, copied (e02.py is not imported: it starts the runner's machinery)."""
    w = fitted.get("wheel_scale") or pr["wheel_scale"]
    lens = [(o["cam_len"], o["bel_len"]) for o in obs if o.get("cam_len") and o.get("bel_len")]
    rc = TR.robust_sd([c - w * b for c, b in lens]) if lens else None
    g = fitted.get("gyro_scale") or pr["gyro_scale"]
    d = fitted.get("gyro_drift_dps") if fitted.get("gyro_drift_dps") is not None else pr["gyro_drift_dps"]
    t = FIT.fit_turns(obs, prior=dict(gyro_scale=g, gyro_drift_dps=d),
                      prior_sd=dict(gyro_scale=1e-6, gyro_drift_dps=1e-6))
    return rc, (t["rms_deg"] if t else None)


def front_events(name):
    out = []
    for r in N.rows(name, "front_checks.csv"):
        exp, fr, hd = N.num(r.get("expected_cm")), N.num(r.get("front_cm")), N.num(r.get("heading_deg"))
        if exp is None or fr is None or hd is None or r.get("heading_measured") != "1":
            continue
        what = r.get("what") or ""
        cos = abs(math.cos(math.radians(hd))) if what == "the y-board" else \
            abs(math.sin(math.radians(hd))) if what == "the x-board" else 1.0
        if exp > FRONT_CHECK_RANGE_CM or cos < FRONT_MIN_COS:
            continue
        out.append((float(r["t"]), "front", r["robot"], fr - FRONT_OFFSET_CM - exp))
    return out


def find_events(name, loo=False):
    smp = [s for s in (N.state(name).get("camera_error_samples") or []) if s.get("source") == "find"]
    out = []
    for i, s in enumerate(smp):
        others = [q for j, q in enumerate(smp) if (j != i if loo else j < i)]
        m = CE.CameraErrorMap(others)
        dx, dy, sd = m.correction(s["x"], s["y"])
        out.append((s["t"], "camera", s["robot"], (s["dx"] - dx, s["dy"] - dy, s["dx"], s["dy"], sd)))
    return out


def fit_events(name):
    """(t, 'fit', robot, (order, obs of this run)) per U run, t = the run's end."""
    rb = N.rebuild_stretches(name, split_jump_cm=SPLIT)
    by = {}
    for robot, obs in rb.items():
        for o in obs:
            by.setdefault((robot, o["order"]), []).append(o)
    out = []
    for robot, order, t0, t1, runs in N.u_runs(name):
        out.append((t1, "fit", robot, (order, by.get((robot, order), []))))
    return out


def replay(name, trust=None, loo=False, log=None):
    """Feed one session; returns (trust, fit rows, flag events)."""
    st = N.state(name)
    trust = trust or TR.SensorTrust()
    trust.new_session()
    ev = sorted(find_events(name, loo) + fit_events(name) + front_events(name), key=lambda e: e[0])
    data, prev, fits, flags = {}, {}, [], []
    robots = list(st["robots"])
    for t, kind, robot, v in ev:
        if kind == "camera":
            trust.camera_at_find(robot, v[0], v[1], t)
        elif kind == "front":
            trust.front_vs_map(robot, v, t)
        else:
            order, obs = v
            pr, measured = prior(st, robot)
            gate = trust.wheel_gate_cm()
            d = data.setdefault(robot, [])
            d += obs
            res = FIT.fit_robot(d, prev.get(robot), prior=pr, prior_sd=dict(gyro_drift_dps=0.1) if measured else None,
                                measured_drift=pr["gyro_drift_dps"] if measured else None, max_rms_cm=gate)
            old = FIT.fit_robot(d, None, prior=pr, prior_sd=dict(gyro_drift_dps=0.1) if measured else None,
                                measured_drift=pr["gyro_drift_dps"] if measured else None, max_rms_cm=1.5)
            if obs:
                rc, rd = stretch_rms_against(obs, res.get("fitted") or {}, pr)
                trust.stretch_fit(robot, rc, rd, len(obs), t)
            else:
                rc = rd = None
            if res["values"] and res["accepted"]:
                prev[robot] = res["values"]
            f = res["fitted"]
            fits.append(dict(t=t, robot=robot, order=order, n_new=len(obs), n=len(d), gate=gate,
                             rms_cm=f.get("rms_cm"), rms_deg=f.get("rms_deg"), wheel=f.get("wheel_scale"),
                             se_w=f.get("se_wheel_scale"), wheel_ok=res["wheel_ok"], gyro_ok=res["gyro_ok"],
                             wheel_ok_15=old["wheel_ok"], run_rc=rc, run_rd=rd, reasons=res["reasons"]))
        for s_, r_, what, kk in trust.update_flags(robots):
            flags.append((t, s_, r_, what, round(kk, 2)))
    return trust, fits, flags


def report(trust, robots):
    csd, cn = trust.camera_sd()
    out = dict(camera=(csd, cn, csd / TR.HEALTHY["camera"], trust.bad("camera")),
               shared_cm=trust.shared_stretch("wheels"), shared_deg=trust.shared_stretch("gyro"),
               cam_share=trust.camera_stretch_share("wheels"), unexplained=trust.shared_unexplained("wheels"),
               gate=trust.wheel_gate_cm(), robots={})
    for r in robots:
        ws, wn, how = trust.own_sd("wheels", r)
        gs, gn, _ = trust.own_sd("gyro", r)
        fs, fn = trust.front_sd(r)
        fv = [it["v"] for it in trust._items("front", r)]
        out["robots"][r] = dict(wheels=(ws, wn, trust.k("wheels", r), trust.bad("wheels", r)),
                                gyro=(gs, gn, trust.k("gyro", r), trust.bad("gyro", r)),
                                front=(fs, fn, trust.k("front", r), trust.bad("front", r)),
                                front_median=N.median(fv) if fv else None, how=how)
    return out


def show(title, trust, fits, flags, robots):
    rp = report(trust, robots)
    print("\n### %s" % title)
    c = rp["camera"]
    print("camera: sd %.2f cm per axis (%d readings), k %.2f%s" % (c[0], c[1], c[2], " FLAGGED" if c[3] else ""))
    print("shared stretch term: %s cm, %s deg; camera's share used: %.2f (%s); 'shared unexplained': %s" % (
        "%.2f" % rp["shared_cm"] if rp["shared_cm"] is not None else "-",
        "%.2f" % rp["shared_deg"] if rp["shared_deg"] is not None else "-",
        rp["cam_share"][0], rp["cam_share"][1], rp["unexplained"]))
    print("wheel gate at the end: %.2f cm" % rp["gate"])
    print("| robot | wheels sd (n) k | gyro sd (n) k | front sd (n) k | front median |")
    for r, d in rp["robots"].items():
        cell = lambda q, u: "%.2f %s (%d) k %.2f%s" % (q[0], u, q[1], q[2], " FLAGGED" if q[3] else "")
        print("| %s | %s | %s | %s | %s |" % (r, cell(d["wheels"], "cm"), cell(d["gyro"], "deg"), cell(d["front"], "cm"),
                                            "%+.1f" % d["front_median"] if d["front_median"] is not None else "-"))
    print("flag events:", [(N.hm(t), s, r, w, k) for t, s, r, w, k in flags] or "none")
    print("| time | robot | run | stretches new/all | gate cm | rms cm | se wheel | wheels pass (new gate) | at 1.5 | gyro | run rms cm / deg |")
    for f in fits:
        print("| %s | %s | %d | %d/%d | %.2f | %s | %s | %s | %s | %s | %s / %s |" % (
            N.hm(f["t"]), f["robot"], f["order"], f["n_new"], f["n"], f["gate"],
            "%.2f" % f["rms_cm"] if f["rms_cm"] is not None else "-",
            "%.3f" % f["se_w"] if f["se_w"] is not None else "-", f["wheel_ok"], f["wheel_ok_15"], f["gyro_ok"],
            "%.2f" % f["run_rc"] if f["run_rc"] is not None else "-",
            "%.2f" % f["run_rd"] if f["run_rd"] is not None else "-"))
        if not f["wheel_ok"]:
            print("|  | | | | | | | why: %s |" % "; ".join(x for x in f["reasons"] if "wheel" in x))
    return rp


def camera_table(name):
    print("\n### camera at the finds, %s (cm, camera - true; corrected by the map of earlier finds / of all others)"
          % name)
    a, b = find_events(name, False), find_events(name, True)
    print("| time | robot | raw dx, dy | earlier-finds map: resid dx, dy | leave-one-out: resid dx, dy |")
    for (t, _, r, v), (_, _, _, w) in zip(a, b):
        print("| %s | %s | %+.2f, %+.2f | %+.2f, %+.2f | %+.2f, %+.2f |" % (N.hm(t), r, v[2], v[3], v[0], v[1], w[0], w[1]))
    for lab, ev in (("earlier finds", a), ("leave one out", b)):
        vals = [x for e in ev for x in e[3][:2]]
        print("%s: robust sd %.2f cm per axis (n %d), rms %.2f" % (
            lab, TR.robust_sd(vals), len(vals), math.sqrt(sum(v * v for v in vals) / len(vals))))


def main():
    robots = list(N.ROBOTS)
    if SPLIT:
        print("(stretches found with the camera track cut at %.1f cm jumps)" % SPLIT)
        tr, f1, fl1 = replay("stage1")
        show("stage1 alone", tr, f1, fl1, robots)
        tr2, f2, fl2 = replay("stage2")
        show("stage2 alone", tr2, f2, fl2, robots)
        tr, f2, fl2 = replay("stage2", trust=tr)
        show("stage 1 then stage 2", tr, f2, fl1 + fl2, robots)
        tr, f, fl = replay("oct23-morning")
        show("2026-10-23 laptop morning", tr, f, fl, robots)
        return
    for name in ("stage1", "stage2"):
        camera_table(name)
    for name in ("stage1", "stage2"):
        for loo in (False, True):
            tr, fits, flags = replay(name, loo=loo)
            show("%s alone, camera map from %s" % (name, "all other finds" if loo else "earlier finds"),
                 tr, fits, flags, robots)
    tr, f1, fl1 = replay("stage1")
    tr, f2, fl2 = replay("stage2", trust=tr)
    show("stage 1 then stage 2 (trust file carried, halved at the new session)", tr, f2, fl1 + fl2, robots)
    # the 2026-10-23 laptop morning, for a healthy reference (no finds' camera samples that day: camera healthy)
    tr, f, fl = replay("oct23-morning")
    show("2026-10-23 laptop morning (reference; no camera samples, so the camera sits at healthy)", tr, f, fl, robots)


if __name__ == "__main__":
    sys.exit(main())
