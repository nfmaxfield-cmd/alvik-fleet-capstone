"""Trace each fall of one hard-simulator run: the last 45 s of the robot's controller events and a 1 s state trail
(truth, own pose, stage, lost, mode, current step). usage: trace_fall.py code profile seed hours [faults] [ENV=V...]
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
trail = collections.defaultdict(list)
s = Cls(seed, hours, profile=profile, prof=prof, faults={k: True for k in faults},
        hdg_fault=None, camera="detections", sensors=True, edge_physics=True)
c = s.ctrl
olog = c.log
bumpt = collections.defaultdict(list)
ob = s._bumps
def _b():
    before = {n: (b.x, b.y) for n, b in s.bots.items()}
    ob()
    for n, b in s.bots.items():
        if (b.x, b.y) != before[n]:
            bumpt[n].append(round(c.t, 1))
s._bumps = _b
def dump(n):
    tr = trail[n]
    def err(x):
        return None if x[6] is None else round(((x[2]-x[6][0])**2 + (x[3]-x[6][1])**2) ** 0.5, 1)
    last = tr[-1] if tr else None
    e10 = [err(x) for x in tr[-12:]]
    kinds = collections.Counter(x[1] if x[1] != "events" else x[3] for x in ev[n] if x[0] >= c.t - 60)
    print("SUMMARY fall %s t=%.1f err_now=%s err_last12=%s bump_last3s=%s stage=%s lost=%s step=%s esc=%d slips=%d corr=%d swaps_named=%s" % (
        n, c.t, err(last) if last else None, e10, [x for x in bumpt[n] if x >= c.t - 3], last[8] if last else None,
        last[9] if last else None, last[11] if last else None, kinds.get("escapes", 0) + kinds.get("consistent sightings", 0),
        kinds.get("slips", 0), kinds.get("camera corrected", 0) + kinds.get("pose", 0), ""))
    print("======== FALL %s at %.1f" % (n, c.t))
    for x in trail[n][-45:]:
        print("   st", x)
    for x in ev[n]:
        if x[0] >= c.t - 45 and x[1] not in ("motion", "battery", "fw_refusals", "zeros"):
            print("   ev", x)
def log(kind, d=None, *a, **k):
    try:
        rb = (d or {}).get("robot") or (d or {}).get("name")
        ev[rb].append((round(c.t, 1), kind, json.dumps(d, default=str)[:260], (d or {}).get("kind")))
        if kind == "events" and (d or {}).get("kind") == "fell (sim)":
            dump(rb)
    except Exception as e:
        print("trace err", e)
    return olog(kind, d, *a, **k)
c.log = log
otick = c.tick
def tick(t):
    out = otick(t)
    if int(round(t * 20)) % 20 == 0:
        for n, r in c.R.items():
            b = s.bots[n]
            p = c.pos(r)
            cur = getattr(b, "cur", None)
            trail[n].append((round(t, 1), "true", round(b.x, 1), round(b.y, 1), round(b.th), "own",
                             None if p is None else (round(p[0], 1), round(p[1], 1), round(p[2]) if len(p) > 2 else None),
                             r.mode, getattr(r, "stage", None), "L" if r.lost is not None else "",
                             "hung" if getattr(b, "hung", False) else "",
                             None if not cur else (cur.get("type"), bool(cur.get("rev")))))
    return out
c.tick = tick
summ = s.run()
b = s.breakage(summ)
print("falls", b["falls"], "contacts", b["contacts"], "swaps", b["swaps"], "free stand", b["longest_free_stand_s"])
