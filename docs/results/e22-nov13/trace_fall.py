#!/usr/bin/env python3
# e22 (Claude's tracing for the report; simulator and logs only): Every pose set from outside (fpose) with the truth error it leaves and the code path that sent it, and every fall with the robot's last events.
# Run from the repo root (the report's Appendix B has the commands). Changelog: e22-2026-11-13a first version.
"""Every pose set from outside (fpose) with the truth error it leaves, and the code path that sent it; plus falls with the
controller pose error at the fall. usage: diag_fpose.py code profile seed hours [faults]"""
import sys, os, json, collections, math, inspect
code, profile, seed, hours = sys.argv[1], sys.argv[2], int(sys.argv[3]), float(sys.argv[4])
faults = sys.argv[5].split(",") if len(sys.argv) > 5 and sys.argv[5] else []
sys.path.insert(0, code)
import freeroam_fastsim as F, fastsim_realism as R
prof = R.load_profile(profile, None)
Cls = R.make_class(F)
s = Cls(seed, hours, profile=profile, prof=prof, faults={k: True for k in faults}, hdg_fault=None,
        camera="detections", sensors=True, edge_physics=True)
c = s.ctrl
rows = []
o_out = c.out
def out(r, text, *a, **k):
    try:
        if str(text).startswith("fpose"):
            n = r.name if hasattr(r, "name") else r
            b = s.bots[n]
            v = text.split()
            x, y = float(v[1]), float(v[2])
            st = [f.function for f in inspect.stack()[1:6]]
            others = min([math.hypot(x - o.x, y - o.y) for m, o in s.bots.items() if m != n] or [999])
            rows.append(dict(t=round(c.t, 1), n=n, err=round(math.hypot(x - b.x, y - b.y), 1),
                             near_other=round(others, 1), path=st))
    except Exception as e:
        rows.append(dict(err="?", e=str(e)))
    return o_out(r, text, *a, **k)
c.out = out
o_ev = c.event
falls = []
evh = {}
def event(r, kind, text):
    evh.setdefault(r.name if r else None, []).append((c.t, kind, text))
    if kind == "fell (sim)":
        b = s.bots[r.name]; p = c.pos(r)
        falls.append(dict(hist=[(round(a,1),k_,str(x_)[:90]) for a,k_,x_ in evh.get(r.name, [])[-8:]], t=round(c.t, 1), n=r.name, err=round(math.hypot(p[0] - b.ox, p[1] - b.oy), 1) if p else None,
                          pos_err=round(math.hypot(p[0] - b.x, p[1] - b.y), 1) if p else None, mode=r.mode,
                          kind=(r.move or {}).get("kind"), lost=r.lost is not None, stage=getattr(r, "stage", None)))
    return o_ev(r, kind, text)
c.event = event
summ = s.run()
b = s.breakage(summ)
print("falls", b["falls"], "contacts", b["contacts"], "longest free", b["longest_free_stand_s"])
for f in falls:
    h = f.pop("hist")
    print("FALL", json.dumps(f))
    for x in h:
        print("     ", x)
big = [r_ for r_ in rows if isinstance(r_.get("err"), float) and r_["err"] > 12]
print("fposes", len(rows), "over 12 cm", len(big))
cnt = collections.Counter(tuple(r_["path"][:3]) for r_ in big)
for k, v in cnt.most_common(12):
    print(v, k)
for r_ in big[:40]:
    print(json.dumps(r_))
