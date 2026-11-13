#!/usr/bin/env python3
# e22 (Claude's tracing for the report; simulator and logs only): Trace the long stands of one hard-simulator run (each robot every 10 s while it stands: its mode, stage, lost, shape, truth, the fixes reaching it and its move).
# Run from the repo root (the report's Appendix B has the commands). Changelog: e22-2026-11-13a first version.
"""Diagnose long stands: every 10 s for each robot still > 30 s: truth, own, pos, lost, stage, unsure, fixes in last 10 s,
layer dets near truth, why_zero reason. usage: diag_stand.py code profile seed hours [faults]"""
import sys, os, json, collections, math
code, profile, seed, hours = sys.argv[1], sys.argv[2], int(sys.argv[3]), float(sys.argv[4])
faults = sys.argv[5].split(",") if len(sys.argv) > 5 and sys.argv[5] else []
sys.path.insert(0, code)
import freeroam_fastsim as F, fastsim_realism as R
prof = R.load_profile(profile, None)
Cls = R.make_class(F)
s = Cls(seed, hours, profile=profile, prof=prof, faults={k: True for k in faults}, hdg_fault=None,
        camera="detections", sensors=True, edge_physics=True)
c = s.ctrl
fixes = collections.defaultdict(list)
o_fix = c.on_fix
def on_fix(name, t, x, y, th=None, **kw):
    fixes[name].append((c.t, x, y, kw.get("how"), kw.get("cam")))
    del fixes[name][:-300]
    return o_fix(name, t, x, y, th, **kw)
c.on_fix = on_fix
zero = {}
o_wz = c.why_zero
def why_zero(r, why):
    zero[r.name] = (round(c.t, 1), why)
    return o_wz(r, why)
c.why_zero = why_zero
evs = collections.defaultdict(list)
o_ev = c.event
def event(r, kind, text):
    if r is not None:
        evs[r.name].append((round(c.t, 1), kind, text[:150]))
        del evs[r.name][:-30]
    return o_ev(r, kind, text)
c.event = event
otick = c.tick
last = [0]
out = []
def tick(t):
    res = otick(t)
    if t - last[0] >= 10.0:
        last[0] = t
        for n, r in c.R.items():
            st = s.m_still.get(n, 0)
            if st > 30:
                b = s.bots[n]; p = c.pos(r)
                fx = [f for f in fixes[n] if f[0] > t - 10]
                out.append(dict(t=round(t), n=n, still=round(st), mode=r.mode, stage=getattr(r, "stage", None),
                    lost=r.lost is not None, unsure=r.__dict__.get("unsure") is not None,
                    held=(r.held or {}).get("kind") if getattr(r, "held", None) else None,
                    search=r.__dict__.get("search") is not None, hu=bool(r.__dict__.get("head_unknown")),
                    truth=(round(b.x,1), round(b.y,1), round(b.th)), own=(round(b.ox,1), round(b.oy,1)),
                    pos=None if p is None else (round(p[0],1), round(p[1],1)), hung=getattr(b, "hung", False),
                    ext=round(r.shape.extent(),1), room=round(c.shape_room(r, p, 5.0),1) if p else None,
                    nfix=len(fx), fixd=[round(math.hypot(f[1]-b.x, f[2]-b.y),1) for f in fx[-3:]],
                    hows=sorted(set(str(f[3]) for f in fx)), zero=zero.get(n),
                    busy=round(r.busy_until - t, 1), tilted=getattr(r, "tilted", False),
                    rec=bool(r.__dict__.get("recover")), start_held=bool(r.__dict__.get("start_held")),
                    mv=None if r.move is None else dict(kind=r.move.get("kind"), phase=r.move.get("phase"), age=round(t - r.move.get("t0", t), 1),
                        steps={str(i): (r.steps.get(i) or {}).get("status") for i in (r.move.get("turn_id"), r.move.get("drive_id")) if i}),
                    simstate=getattr(b, "state", None), simcur=(getattr(b, "cur", None) or {}).get("type") if isinstance(getattr(b, "cur", None), dict) else None, simq=len(getattr(b, "q", []) or []),
                    ev=evs[n][-4:]))
    return res
c.tick = tick
summ = s.run()
b = s.breakage(summ)
print("falls", b["falls"], "longest free", b["longest_free_stand_s"], "stands", [q for q in b["stands"] if q[2] > 60])
seen = set()
for d in out:
    key = (d["n"], d["mode"], d["stage"], d["lost"], d["unsure"], d["held"], d["search"], str(d["zero"][1] if d["zero"] else None)[:40])
    if key in seen and d["still"] % 120 > 10:
        continue
    seen.add(key)
    print(json.dumps(d, default=str))
