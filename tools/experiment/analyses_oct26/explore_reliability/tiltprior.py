"""tiltprior.py : explore's camera fit on real sessions' stops with and without a prior that the camera looks
roughly straight down, scored against roam1 (whole-board scale, and the camera's tilt). Cases: 09:55 Center alone
(the failure), 15:10 all three, 21:14 Center + Right."""
import json
import sys
import time

import cv2
import numpy as np

from realdiag import EX, ROAM, sim_fit

R = "/tmp/claude-0/-home-claude/fff6dc9c-5876-54f9-92a7-a08483d3d5c7/scratchpad/real/"
cases = [("09:55 Center", "20261026-095541", ["Center"]), ("15:10 all", "20261025-151004", ["Left", "Center", "Right"]),
         ("21:14 C+R", "20261025-211419", ["Center", "Right"]), ("15:10 Right", "20261025-151004", ["Right"]),
         ("21:14 Center", "20261025-211419", ["Center"])]
grid = [(x, y) for x in np.arange(5, 120, 10.0) for y in np.arange(5, 120, 10.0)]
G = np.array(grid)
uv = np.array([ROAM.project((x, y, EX.ROBOT_TOP_CM)) for x, y in grid])
variants = [(None, 3.0, None), (10.0, 3.0, None), (5.0, 3.0, None), (10.0, 10.0, None)]
if len(sys.argv) > 1:
    # TILT_SD,W[,F_PX]: 0 = no tilt prior; F_PX = the webcam's known focal length (prior sd 5 %)
    variants = [(float(a.split(",")[0]) or None, float(a.split(",")[1]),
                 float(a.split(",")[2]) if len(a.split(",")) > 2 else None) for a in sys.argv[1:]]
for lab, s_, names in cases:
    obs = json.load(open(R + s_ + "/e02_state.json"))["setup"]["explore_obs"]
    for sd, w, fpx in variants:
        EX.TILT_PRIOR_SD_DEG, EX.TILT_PRIOR_W, EX.F_PRIOR_PX = sd, w, fpx
        t0 = time.time()
        sol = EX.fit({n: obs[n] for n in names}, (640, 480))
        A = np.array([EX.world_of(sol, u, v) for u, v in uv])
        sc, med = sim_fit(A, G)
        print("%-13s f %-5s prior %-5s w %4.1f: rms %.2f px, tilt %.1f deg, f %.0f, roam1 cm per explore cm %.3f (median left "
              "%.1f cm) [%.0f s]" % (lab, fpx, sd, w, sol["rms_px"], EX._tilt_deg(sol["rvec"]), sol["f"], sc, med,
                                    time.time() - t0))
        sys.stdout.flush()
