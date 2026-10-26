"""realreplay.py SESSION_FOLDER [...] : explore's finish replayed offline on real sessions' stops (evalfits.finish),
before (base) and with this build's fit changes (repair + sprior + seen), each camera against the stored roam1
calibration over the whole board (roam1 cm per explore cm; 1.0 = right size)."""
import json
import os
import sys

import numpy as np

import evalfits
from realdiag import EX, ROAM, sim_fit

grid = [(x, y) for x in np.arange(5, 120, 10.0) for y in np.arange(5, 120, 10.0)]
G = np.array(grid)
uv = np.array([ROAM.project((x, y, EX.ROBOT_TOP_CM)) for x, y in grid])
for d in sys.argv[1:]:
    obs = json.load(open(os.path.join(d, "e02_state.json")))["setup"].get("explore_obs") or {}
    if not obs:
        continue
    for v in ("base", "repair+sprior+seen"):
        evalfits.set_variant(v)
        r, sol, use = evalfits.finish(json.loads(json.dumps(obs)), seen="seen" in v)
        if sol is not None:
            A = np.array([EX.world_of(sol, u, v_) for u, v_ in uv])
            r["roam1_per_explore"] = round(sim_fit(A, G)[0], 3)
        print(os.path.basename(d.rstrip("/")), v, json.dumps(r), flush=True)
