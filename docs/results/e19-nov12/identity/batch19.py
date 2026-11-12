"""e18 item 1: two robots unaccounted, standing near each other, the home lights camera (no spin flag, the spot at the
robot's middle: a turn on the spot is invisible). Random separation, own-pose offsets and camera noise; e18 (the
out-and-back after the wiggle) against MIRLT_OUTBACK_ID off (e17: wiggles only). Two conditions: "held" (both stand,
held, as Left and Right at home on 11 Nov) and "held, never still" (the default: held robots creep between identity
moves).
Writes one JSON line per trial.

    python3 batch.py REPO_TOOLS_FLEET OUT.jsonl [N]
"""
import json, math, random, sys, time
FLEET = sys.argv[1]
sys.path.insert(0, FLEET)
sys.path.insert(0, FLEET + "/tests")
import e17_harness as H

out = open(sys.argv[2], "w")
N = int(sys.argv[3]) if len(sys.argv) > 3 else 30
NLO, NHI, FPS = float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])
K0 = int(sys.argv[7]) if len(sys.argv) > 7 else 0
for k in range(K0, K0 + N):
    rng = random.Random(1000 + k)
    sep = rng.uniform(15.0, 35.0)
    ang = rng.uniform(0, 360)
    ax, ay = rng.uniform(35, 85), rng.uniform(35, 85)
    bx, by = ax + sep * math.cos(math.radians(ang)), ay + sep * math.sin(math.radians(ang))
    truth = {"Left": (ax, ay, rng.uniform(-180, 180)), "Right": (bx, by, rng.uniform(-180, 180)),
             "Center": (110.0, 110.0, 0.0)}
    offs = {}
    for n in ("Left", "Right"):
        d, a = rng.uniform(20, 50), rng.uniform(0, 360)
        offs[n] = (d * math.cos(math.radians(a)), d * math.sin(math.radians(a)), 0.0)
    noise = rng.uniform(NLO, NHI)
    for cond in ("held", "held, never still"):
      for mode in ("e18",):
        t0 = time.time()
        R = H.Rig(truth, own_off=offs, lights_off_cm=0.0, spin_flag=False, seed=k + 1, noise=noise, fps=FPS,
                  layer_kw=dict(outback=(mode == "e18")), never_still=(cond != "held"))
        R.run(0.1)
        for n in ("Left", "Right"):
            R.c.start_lost(R.c.R[n], "batch: the camera lost it")
            R.c.R[n].lost_hold = True             # held where it stands (no dead-reckoned roaming)
        named, wrong_at = {}, {}
        mind = [1e9, 1e9]                   # closest middles before both were named, and after
        def hook(R_):
            d_ = math.dist(R_.truth["Left"][:2], R_.truth["Right"][:2])
            k_ = 1 if len(named) == 2 else 0
            mind[k_] = min(mind[k_], d_)
            for n in ("Left", "Right"):
                if R_.c.R[n].__dict__.get("corrected") is None:
                    R_.c.R[n].lost_hold = True
                if n not in named and R_.c.R[n].__dict__.get("corrected") is not None:
                    named[n] = R_.t
                    wrong_at[n] = round(R_.gap(n), 1)      # how far its pose is from the truth when it is named
        R.run(45.0, hook=hook)
        wrong = [n for n in ("Left", "Right") if n in named and wrong_at[n] > 5.0]     # named on the wrong spot
        later = [n for n in ("Left", "Right") if n in named and wrong_at[n] <= 5.0 and R.gap(n) > 5.0]
        rec = dict(trial=k, cond=cond, mode=mode, sep_cm=round(sep, 1), noise_cm=round(noise, 2),
                   named={n: round(v, 1) for n, v in named.items()}, both=int(len(named) == 2),
                   gap_at_naming=wrong_at, lost_later=later,
                   t_both=round(max(named.values()), 1) if len(named) == 2 else None, wrong=wrong,
                   gaps={n: round(R.gap(n), 1) for n in ("Left", "Right")},
                   outbacks=R.layer.counts.get("outbacks", 0), outback_named=R.layer.counts.get("outback_named", 0),
                   outback_refused=R.layer.counts.get("outback_refused", 0),
                   wiggles=R.layer.counts.get("elim_wiggles", 0), revoked=R.layer.counts.get("elim_revoked", 0),
                   min_middles_cm=round(min(mind), 1), min_before_cm=round(mind[0], 1),
                   min_after_cm=round(mind[1], 1) if mind[1] < 1e8 else None, secs=round(time.time() - t0, 1))
        out.write(json.dumps(rec) + "\n")
        out.flush()
        print(rec)
