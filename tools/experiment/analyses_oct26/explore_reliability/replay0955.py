"""replay0955.py SESSION_FOLDER [ROBOT ...] : explore's camera fit on the named robots' stops (default: those its
own run used), scored against roam1 on (a) the robots' own stops and (b) a grid over the whole board: where the
camera's scale goes wrong. Variants: as built; the lens-bend term fixed at 0; the robots' known size fixing the
scale."""
import json
import os
import sys

import numpy as np

from realdiag import EX, ROAM, sim_fit, floor

d = sys.argv[1]
st = json.load(open(os.path.join(d, "e02_state.json")))
obs = st["setup"]["explore_obs"]
names = sys.argv[2:] or [n for n in obs if EX.robot_check(obs[n])["ok"]]
robots = {n: obs[n] for n in names}
size = (640, 480)
sol = EX.fit(robots, size)
print("robots", names, "rms %.2f f %.0f k1 %.3f s %s" % (sol["rms_px"], sol["f"], sol["k1"], sol["s"]))
grid = [(x, y) for x in np.arange(5, 120, 10.0) for y in np.arange(5, 120, 10.0)]
G = np.array(grid)
uv = np.array([ROAM.project((x, y, EX.ROBOT_TOP_CM)) for x, y in grid])
A = np.array([EX.world_of(sol, u, v) for u, v in uv])
s_grid, med_grid = sim_fit(A, G)
own = [q for n in names for q in robots[n]]
B = np.array([EX.world_of(sol, q["u"], q["v"]) for q in own])
F = np.array([floor(q) for q in own])
s_own, med_own = sim_fit(B, F)
print("roam1 cm per explore cm: whole board %.3f (median left %.1f cm); at the robots' own stops %.3f (%.1f cm)" % (
    s_grid, med_grid, s_own, med_own))
sc = EX.size_scale(robots, sol, size)
print("size check (outline long side / 9.5 cm):", sc)
print("stops spread on roam1: x %.0f-%.0f, y %.0f-%.0f cm" % (F[:, 0].min(), F[:, 0].max(), F[:, 1].min(), F[:, 1].max()))
