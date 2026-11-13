#!/usr/bin/env python3
# e22 (Claude's tracing for the report; simulator and logs only): Each robot's live error shape (what shape_live.json carries) from a hard-simulator run at given times.
# Run from the repo root (the report's Appendix B has the commands). Changelog: e22-2026-11-13a first version.
"""Capture each robot's live shape (what shape_live.json carries) from a hard-simulator run at given times."""
import sys, json
sys.path.insert(0, sys.argv[1])
import freeroam_fastsim as F, fastsim_realism as R
prof = R.load_profile(sys.argv[2]); Cls = R.make_class(F)
s = Cls(int(sys.argv[3]), float(sys.argv[4]), profile=sys.argv[2], prof=prof, camera="detections", sensors=True, edge_physics=True)
times = [float(x) for x in sys.argv[5].split(",")]
c = s.ctrl
snaps = []
o = c.tick
def tick(t):
    res = o(t)
    if times and t >= times[0]:
        times.pop(0)
        rob = []
        for n, r in c.R.items():
            p = c.pos(r); b = s.bots[n]
            rob.append(dict(name=n, x=round(p[0], 1) if p else None, y=round(p[1], 1) if p else None, th=r.own[3] % 360 if r.own else None,
                            shape_draw=r.shape.draw(), shape=r.shape.summary(), stage=getattr(r, "stage", None), lost=r.lost is not None,
                            seen=r.lost is None and c.fresh(r), rescue=bool(r.__dict__.get("rescue")), truth=(round(b.x, 1), round(b.y, 1)),
                            battery=r.battery, doing=r.mode))
        snaps.append(dict(t=round(t, 1), robots=rob))
    return res
c.tick = tick
s.run()
json.dump(snaps, open(sys.argv[6], "w"), default=str)
print([ (sn["t"], [(r["name"], r["stage"], r["shape_draw"]["extent"]) for r in sn["robots"]]) for sn in snaps])
