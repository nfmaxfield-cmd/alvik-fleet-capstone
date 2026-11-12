"""e18 item 5: blind excursions through a dark patch (the camera sees nothing there) with good, typical and poor odometry
records. Each trial: 150 s of roaming in the lit part (its own record builds), then up to 5 excursions to random
points inside the patch (out, turn round, back). For each: allowed (full / shortened) or refused, how deep into the dark,
and when the camera has it again: the error its record predicted against the error found; also what the fixed 29 Oct
numbers (2 cm + 0.07 cm per cm, e17) would have predicted for the same stretch. One JSON line per excursion.

    python3 batch.py REPO_TOOLS_FLEET OUT.jsonl [SEEDS]
"""
import json, math, random, sys, time
FLEET = sys.argv[1]
sys.path.insert(0, FLEET)
sys.path.insert(0, FLEET + "/tests")
import e17_harness as H
import proven_workspace as PW

DARK = (70.0, 45.0, 100.0, 80.0)
CLASSES = {"good": (0.003, 0.0, 0.005), "typical": (0.02, 0.012, 0.02), "poor": (0.05, 0.03, 0.05)}
out = open(sys.argv[2], "w")
SEEDS = int(sys.argv[3]) if len(sys.argv) > 3 else 6


def stand(R, r):
    r.lost_hold = True
    for _ in range(60):
        if r.mode == "idle" and r.move is None:
            break
        R.run(0.5)
    r.lost_hold = False
    r.busy_until = 1e9
    R.run(2.0)


for cls, odo in CLASSES.items():
    for seed in range(1, SEEDS + 1):
        t0 = time.time()
        rng = random.Random(seed * 17 + len(cls))
        ws = PW.ProvenWorkspace(H.BOARD)
        ws.see(H.BOARD, None, [DARK])
        R = H.Rig({"Left": (40.0, 60.0, 0.0), "Center": (100.0, 105.0, 90.0), "Right": (20.0, 15.0, 180.0)},
                  odo={"Left": odo}, dark=[DARK], workspace=ws, raw=True, seed=seed, fuse_then=True)
        R.c.motion_style = "stopturn"
        R.run(0.5)
        r = R.c.R["Left"]
        r.busy_until = 0.0
        R.run(150.0)
        for k in range(5):
            stand(R, r)
            tgt = (rng.uniform(DARK[0] + 4, DARK[2] - 4), rng.uniform(DARK[1] + 4, DARK[3] - 4))
            tr = R.c.trust["Left"]
            n_chk = len([q for q in tr.cal if q["kind"] == "excursion"])
            p = R.c.pos(r)
            res = R.c.blind_excursion(r, tgt, why="batch")
            rec = dict(cls=cls, seed=seed, k=k, x0=round(p[0], 1), y0=round(p[1], 1), tx=round(tgt[0], 1),
                       ty=round(tgt[1], 1), ok=int(bool(res.get("ok"))), want_cm=round(res.get("want_cm") or 0, 1),
                       depth_cm=round(res.get("depth_cm") or 0, 1), r_max=res.get("r_max"),
                       blind_cm_planned=res.get("blind_cm"), r40=round(tr.radius(40.0), 2), k_cal=round(tr.k_cal, 3),
                       fit=[round(v, 4) for v in tr.fit()[:4]])
            if res.get("ok"):
                R.run(40.0)
                chk = [q for q in tr.cal if q["kind"] == "excursion"][n_chk:]
                rows = [l for l in R.logs if l.get("table") == "blind_checks" and l.get("excursion") == 1]
                if chk and rows:
                    row = rows[-1]
                    rec.update(pred_cm=chk[-1]["pred"], act_cm=chk[-1]["act"], within=chk[-1]["within"],
                               blind_cm=row["blind_cm"], fixed_pred_cm=round(2.0 + 0.07 * row["blind_cm"], 2),
                               fixed_within=int(row["act_cm"] <= 2.0 + 0.07 * row["blind_cm"]))
            ho = [q for q in tr.cal if q["kind"] == "held-out sample"]
            rec.update(heldout_n=len(ho), heldout_within=sum(q["within"] for q in ho),
                       true_heading_off_deg=round(R.off["Left"][2], 1), secs=round(time.time() - t0, 1))
            out.write(json.dumps(rec) + "\n")
            out.flush()
            print(rec)
