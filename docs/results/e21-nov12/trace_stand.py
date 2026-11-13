"""Trace the long free stands of one hard-simulator run: per robot, its state every 5 s while still, and the
controller's log events during the stand. usage: trace_stand.py code profile seed hours [faults]
(e21, Claude's tracing for the report: run from the repo root with code = tools/fleet; simulator only.)
Changelog: e21-2026-11-13a first version."""
import sys, os, json, collections
code, profile, seed, hours = sys.argv[1], sys.argv[2], int(sys.argv[3]), float(sys.argv[4])
faults = sys.argv[5].split(",") if len(sys.argv) > 5 and sys.argv[5] else []
sys.path.insert(0, code)
import freeroam_fastsim as F, fastsim_realism as R
prof = R.load_profile(profile, None)
Cls = R.make_class(F)
ev = collections.defaultdict(list)
snap = collections.defaultdict(list)
class T(Cls):
    def _stand_end(self, n, r, t):
        Cls._stand_end(self, n, r, t)
s = T(seed, hours, profile=profile, prof=prof, faults={k: True for k in faults},
      hdg_fault=None, camera="detections", sensors=True, edge_physics=True)
c = s.ctrl
olog = c.log
def log(kind, d=None, *a, **k):
    try:
        rb = (d or {}).get("robot") or (d or {}).get("name")
        ev[rb].append((round(c.t, 1), kind, json.dumps(d, default=str)[:200]))
    except Exception:
        pass
    return olog(kind, d, *a, **k)
c.log = log
otick = c.tick
def tick(t):
    out = otick(t)
    if int(t * 10) % 50 == 0:
        for n, r in c.R.items():
            st = s.m_still.get(n, 0) if hasattr(s, "m_still") else 0
            if st > 30:
                p = c.pos(r)
                snap[n].append((round(t, 1), round(st, 0), r.mode, getattr(r, "stage", None),
                                r.lost is not None, (getattr(r, "held", None) or {}).get("kind") if getattr(r, "held", None) else None,
                                bool(r.__dict__.get("search")), bool(r.__dict__.get("head_unknown")),
                                None if p is None else (round(p[0], 1), round(p[1], 1)),
                                round(r.shape.extent(), 1) if getattr(r, "shape", None) else None))
    return out
c.tick = tick
summ = s.run()
b = s.breakage(summ)
L = sorted([q for q in b["stands"] if not q[5]], key=lambda q: -q[2])[:3]
print("longest free stands", L)
for t0, n, d, *_ in L:
    print("==== %s stand from %.0f for %.0f s" % (n, t0, d))
    for x in snap[n]:
        if t0 <= x[0] <= t0 + d + 1:
            print("  ", x)
    for x in ev[n]:
        if t0 - 20 <= x[0] <= t0 + d + 1:
            print("   ev", x)
