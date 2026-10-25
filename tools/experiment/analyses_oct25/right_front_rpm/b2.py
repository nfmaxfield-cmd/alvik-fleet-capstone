# Where the replay's 19% comes from: its speed calibration integrates |speed| over each camera drive stretch plus 1 s
# either side, while the angle calibration uses the angle change across the stretch alone.
import sys, numpy as np
sys.path.insert(0, '/home/claude/alvik-fleet-capstone/tools/discovery')
import load, features as F
S = load.find_sessions()
cases = [('e02-20261023-114620','Right'),('e02-20261023-114620','Left'),('e02-20261024-153612','Center'),
         ('e02-20261024-153612','Right'),('e02-20261023-191347','Left'),('e02-20261024-161612','Right')]
tot = {k: [] for k in ('replay','exact','ang_wide')}
for sid, rob in cases:
    case, key = load.load_case(S[sid], rob)
    g, feats, cs, num = F.case_features(case)
    inv = {v: k for k, v in key.items()}
    A, V = g.v[inv['wl_deg']], g.v[inv['wl_rpm']]
    eps = F.camera_episodes(case, g, cs)
    r_rep, r_ex, r_aw = [], [], []
    for i0, i1, hd, dist in eps:
        ang = abs(A[i1] - A[i0])
        if not np.isfinite(ang) or ang < 30: continue
        wide = np.nansum(np.abs(V[max(0, i0 - 10):i1 + 11])) * g.dt * 6      # rpm*s*6 = degrees
        exact = abs(np.nansum(V[i0:i1 + 1]) * g.dt * 6)
        ang_w = np.nansum(np.abs(np.diff(A[max(0, i0 - 10):i1 + 11])))
        r_rep.append(wide / ang); r_ex.append(exact / ang); r_aw.append(ang_w / ang)
    print(sid, rob, 'stretches', len(r_rep), 'replay-window |rpm| integral / angle %.3f' % np.median(r_rep),
          ' stretch-only rpm integral / angle %.3f' % np.median(r_ex), ' |angle| moved in the wide window / across stretch %.3f' % np.median(r_aw),
          ' replay ep_per_cms %.3f' % (1 / feats[inv['wl_rpm']].get('ep_per_cms', np.nan)), flush=True)
    tot['replay'] += r_rep; tot['exact'] += r_ex; tot['ang_wide'] += r_aw
print({k: round(float(np.median(v)), 3) for k, v in tot.items()})
