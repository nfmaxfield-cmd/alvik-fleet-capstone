"""Find every moment a robot's controller pose went from right (<10 cm from the truth) to far wrong (>25 cm) and list
what the controller did in the 6 s before. usage: trace_onset.py code profile seed hours [faults]
(e21, Claude's tracing for the report: run from the repo root with code = tools/fleet; simulator only.)
Changelog: e21-2026-11-13a first version."""
import sys, json, collections
code, profile, seed, hours = sys.argv[1], sys.argv[2], int(sys.argv[3]), float(sys.argv[4])
faults = sys.argv[5].split(",") if len(sys.argv) > 5 and sys.argv[5] else []
sys.path.insert(0, code)
import freeroam_fastsim as F, fastsim_realism as R
prof = R.load_profile(profile, None)
s = R.make_class(F)(seed, hours, profile=profile, prof=prof, faults={k: True for k in faults},
                    hdg_fault=None, camera="detections", sensors=True, edge_physics=True)
c = s.ctrl
ev = collections.defaultdict(lambda: collections.deque(maxlen=400))
olog = c.log
def log(kind, d=None, *a, **k):
    d0 = d or {}
    rb = d0.get("robot") or d0.get("name")
    if kind not in ("motion", "battery", "fw_refusals", "zeros", "edge_judgement", "fallbacks", "breaks"):
        ev[rb].append((round(c.t, 2), kind, d0.get("kind") or d0.get("what") or d0.get("why") or "",
                       json.dumps(d0, default=str)[:180]))
    return olog(kind, d, *a, **k)
c.log = log
good = {}
onsets = []
otick = c.tick
def tick(t):
    out = otick(t)
    if int(round(t * 20)) % 5 == 0:
        for n, r in c.R.items():
            b = s.bots[n]
            if b.dead:
                continue
            p = c.pos(r)
            if p is None:
                continue
            e = ((p[0] - b.x) ** 2 + (p[1] - b.y) ** 2) ** 0.5
            if e < 10.0:
                good[n] = t
            elif e > 25.0 and n in good and t - good[n] < 6.0:
                near = min([((o.x - p[0]) ** 2 + (o.y - p[1]) ** 2) ** 0.5 for m, o in s.bots.items() if m != n] + [999])
                onsets.append((round(t, 1), n, round(e, 1), round(near, 1), r.lost is not None,
                               [x for x in ev[n] if x[0] >= good[n] - 6.0]))
                del good[n]
    return out
c.tick = tick
summ = s.run()
b = s.breakage(summ)
print("RESULT falls %d swaps %d onsets %d" % (b["falls"], b["swaps"], len(onsets)))
for t, n, e, near, lost, E in onsets:
    print("ONSET t=%.1f %s err=%.1f nearest_other_to_its_pose=%.1f lost=%s" % (t, n, e, near, lost))
    for x in E:
        print("    ", x[0], x[1], "|", x[2], "|", x[3][:150])
