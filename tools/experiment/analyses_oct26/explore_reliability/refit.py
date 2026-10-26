"""refit.py RUN_FOLDER [VARIANT ...] : a dry run's explore stops refitted offline (the robot checks, the joint fit, the
per-robot drop, as e02 explore_finish does) under fit variants, each scored against the simulator's true camera
(explore's cm per true cm over the whole board) and the true wheel scales. VARIANT: base | tilt (prior: the camera
looks roughly straight down, sd 10 deg, weight 10) | tiltK:SD:W."""
import json
import os
import sys
import time

import numpy as np

import exsum
from exsum import EX


def run_fit(obs, size):
    robots = {n: v for n, v in obs.items() if len(v) >= 5}
    checks = {n: EX.robot_check(v) for n, v in robots.items()}
    use = {n: v for n, v in robots.items() if checks[n]["ok"]}
    if not use:
        return None, "no robot passes its own check", []
    sol = EX.fit(use, size)
    worse = [n for n, v in sol["per_robot_rms_px"].items() if not (v <= EX.FIT_ROBOT_MAX_PX)]
    if worse and len(worse) < len(use):
        use = {n: v for n, v in use.items() if n not in worse}
        sol = EX.fit(use, size)
    return sol, EX.fit_check(sol), list(use)


def variant(v):
    EX.TILT_PRIOR_SD_DEG, EX.TILT_PRIOR_W, EX.F_PRIOR_PX = None, 3.0, None
    EX.S_PRIOR_SD_LOG, EX.S_PRIOR_W = None, 3.0
    if v.startswith("s"):                      # sSD:W[+rest]: each robot's wheel scale near 1 (sd SD in log)
        head, _, v = v.partition("+")
        sd_, w_ = head[1:].split(":")
        EX.S_PRIOR_SD_LOG, EX.S_PRIOR_W = float(sd_), float(w_)
    if v.startswith("f"):                      # fF[+rest]: the webcam's focal length known, F px (sd 5 %)
        head, _, v = v.partition("+")
        EX.F_PRIOR_PX = float(head[1:])
    if v == "tilt":
        EX.TILT_PRIOR_SD_DEG, EX.TILT_PRIOR_W = 10.0, 10.0
    elif v.startswith("tiltK"):
        _, sd, w = v.split(":")
        EX.TILT_PRIOR_SD_DEG, EX.TILT_PRIOR_W = float(sd), float(w)


def main():
    d = sys.argv[1]
    vs = sys.argv[2:] or ["base", "tilt"]
    st = json.load(open(os.path.join(d, "e02_state.json")))
    info = json.load(open(os.path.join(d, "sim", "sim_info.json")))
    obs = st["setup"]["explore_obs"]
    for v in vs:
        variant(v)
        t0 = time.time()
        sol, bad, used = run_fit(obs, (640, 480))
        if sol is None:
            print(json.dumps(dict(run=os.path.basename(d), variant=v, why=bad)))
            continue
        sc, med, p95, _ = exsum.cam_scale(sol, d)
        print(json.dumps(dict(run=os.path.basename(d.rstrip("/")), variant=v, used=used, rms=round(sol["rms_px"], 2),
                              tilt=round(EX._tilt_deg(sol["rvec"]), 1), f=round(sol["f"]), cam_scale=round(sc, 3),
                              res_med=round(med, 2), bad=bad,
                              s={n: "%.3f/%.3f" % (sol["s"][n], info["robots"][n]["w_true"]) for n in sol["s"]},
                              sec=round(time.time() - t0))), flush=True)


if __name__ == "__main__":
    main()
