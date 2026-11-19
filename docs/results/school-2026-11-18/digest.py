#!/usr/bin/env python3
"""Per-run digest of Jay-Sun's Visit points runs (16 and 18 Nov 2026). Read-only on the data.
Usage: python3 -I digest.py <jayruns folder> [out.json]
For each run folder: start time, lists asked, points reached per robot (run/stops.csv, else stdout POINT lines),
how it ended, event counts from run_plan_stdout.txt, closest robot-robot distance and closest approach to the
site's keep-outs and table edges from run/camera_track.csv (camera) and run/step.csv (the robots' own pose)."""
import csv, json, math, os, re, sys, collections

ROOT = sys.argv[1]
BODY = 9.5; HALF = BODY / 2; HALF_DIAG = 6.72
TABLE = (396.2, 167.6)
IN = 2.54
KEEP = {"middle outlet box": ((78.03, 32.97), (6.44, 6.94)), "zone 1 outlet plate": ((25.22, 33.0), (2.44, 6.62)),
        "zone 4 outlet plate": ((130.625, 33.0), (2.25, 6.62))}
RECT = {k: ((c[0] - s[0] / 2) * IN, (c[1] - s[1] / 2) * IN, (c[0] + s[0] / 2) * IN, (c[1] + s[1] / 2) * IN)
        for k, (c, s) in KEEP.items()}

def rect_dist(x, y, r):
    dx = max(r[0] - x, 0, x - r[2]); dy = max(r[1] - y, 0, y - r[3])
    return math.hypot(dx, dy)

def rows(p):
    if not os.path.exists(p): return []
    with open(p, newline="") as f:
        return list(csv.DictReader(f))

def track_stats(tr, key_t="t", kx="x_cm", ky="y_cm"):
    """closest pair distance (same time stamp, or within 0.5 s), closest keep-out and edge approach of a centre"""
    by = collections.defaultdict(list)
    for r in tr:
        try: by[r["robot"]].append((float(r[key_t]), float(r[kx]), float(r[ky])))
        except (ValueError, KeyError, TypeError): pass
    out = {"n": {k: len(v) for k, v in by.items()}}
    names = sorted(by)
    best = None
    if len(names) == 2:
        a, b = by[names[0]], sorted(by[names[1]])
        import bisect
        tb = [q[0] for q in b]
        for t, x, y in a:
            i = bisect.bisect_left(tb, t)
            for j in (i - 1, i):
                if 0 <= j < len(b) and abs(b[j][0] - t) <= 0.5:
                    d = math.hypot(x - b[j][1], y - b[j][2])
                    if best is None or d < best[0]: best = (round(d, 1), round(t, 1), (x, y), b[j][1:])
    out["pair"] = best
    ko = {}; edge = {}
    for n in names:
        for t, x, y in by[n]:
            for k, r in RECT.items():
                d = rect_dist(x, y, r)
                if (n, k) not in ko or d < ko[(n, k)][0]: ko[(n, k)] = (round(d, 1), round(t, 1), round(x, 1), round(y, 1))
            e = min(x, y, TABLE[0] - x, TABLE[1] - y)
            if n not in edge or e < edge[n][0]: edge[n] = (round(e, 1), round(t, 1), round(x, 1), round(y, 1))
    out["keepout"] = {"%s|%s" % k: v for k, v in ko.items()}
    out["edge"] = edge
    return out

PAT = collections.OrderedDict([
    ("not_moved", r"did NOT move"), ("impossible", r"not physically possible"), ("follow_silent", r"nothing from the laptop"),
    ("cam_disagree", r"WARNING: .*camera and its own pose disagree"), ("veer", r"RECOVERING"),
    ("held", r"\] \w+ is held: "), ("needs_hand", r"NEEDS A HAND"), ("unsure_edge", r"unsure near an edge"),
    ("tilt_stop", r"tilt stop stopped it"), ("edge_guard", r"edge guard"), ("obstacle", r"obstacle [0-9.]+ cm ahead"),
    ("moved_end", r"moved to the end of its list"), ("skipped_pt", r"skipped: the camera lost|cannot be reached"),
    ("handed", r"its points not done go to"), ("blind", r"no camera sees it: driving"), ("new_plan", r"\] new plan in"),
])

res = []
for d in sorted(os.listdir(ROOT)):
    p = os.path.join(ROOT, d)
    if not os.path.isdir(p): continue
    so = open(os.path.join(p, "run_plan_stdout.txt"), errors="replace").read() if os.path.exists(os.path.join(p, "run_plan_stdout.txt")) else ""
    R = {"run": d, "start": d[9:11] + ":" + d[11:13] + ":" + d[13:15], "day": d[6:8]}
    vo = {}
    if os.path.exists(os.path.join(p, "visit_order.json")):
        vo = json.load(open(os.path.join(p, "visit_order.json")))
    R["asked"] = vo.get("asked"); R["orders"] = vo.get("orders") or {}; R["flagged"] = sorted((vo.get("flagged") or {}))
    m = re.search(r"Found: (.*)", so); R["found"] = m.group(1) if m else None
    m = re.search(r"AUTO: the camera sees (\d+) roof tag", so); R["tags_seen"] = int(m.group(1)) if m else None
    m = re.search(r"Bound: (.*)", so); R["bound"] = m.group(1) if m else None
    m = re.search(r"NOT STARTED: (.{0,260})", so); R["not_started"] = m.group(1) if m else None
    m = re.search(r"^Result: (.*?) after ([0-9.]+) s \(first plan said ([0-9.]+) s\), (\d+) plan", so, re.M)
    R["result"] = m.group(1) if m else None; R["dur_s"] = float(m.group(2)) if m else None
    R["planned_s"] = float(m.group(3)) if m else None; R["plans"] = int(m.group(4)) if m else None
    R["traceback"] = re.findall(r"^(KeyError: .*|KeyboardInterrupt)$", so, re.M)
    R["battery"] = re.findall(r"(Alpha|Bravo): firmware [^,]+, battery (\d+) %, boot (\w+)", so)[:2]
    # points reached
    st = rows(os.path.join(p, "run", "stops.csv"))
    got = collections.defaultdict(list)
    if st:
        for r in st: got[r["robot"]].append((r["point"], r["source"], r["plus_minus_cm"]))
    else:
        for t, pt, rb, rest in re.findall(r"^\[ *([0-9.]+)\] POINT point (P\d+) \((\w+)\): (.*)$", so, re.M):
            got[rb].append((pt, rest, ""))
    R["got"] = {k: [x[0] for x in v] for k, v in got.items()}
    R["got_src"] = {k: collections.Counter(("side" if "side" in x[1] else "overhead" if "overhead" in x[1] else "odometry") for x in v) for k, v in got.items()}
    # progress per robot against its own list, in order (a list may repeat a point)
    prog = {}
    for rb, lst in R["orders"].items():
        g = list(R["got"].get(rb, [])); n = 0; missing = []
        for pt in lst:
            if pt in g: g.remove(pt); n += 1
            else: missing.append(pt)
        prog[rb] = (n, len(lst), missing)
    R["progress"] = prog
    R["others_points"] = {rb: [x for x in v if x not in (R["orders"].get(rb) or [])] for rb, v in R["got"].items() if R["orders"]}
    live = so.split("plan plan-")[1] if "plan plan-" in so else ""
    live = live.split("\nResult:")[0]
    R["counts"] = {k: len(re.findall(v, live)) for k, v in PAT.items()}
    cpu = [float(x) for x in re.findall(r"new plan in ([0-9.]+) s", live)]
    R["plan_cpu"] = (round(sorted(cpu)[len(cpu) // 2], 1), round(max(cpu), 1), len(cpu)) if cpu else None
    m = re.search(r"planned in ([0-9.]+) s", so); R["first_plan_cpu"] = float(m.group(1)) if m else None
    # tracks: only while the plan ran (from the "start" sync mark)
    t0 = None
    sm = os.path.join(p, "sync_marks.jsonl")
    cj = os.path.join(p, "run", "coordinator.jsonl")
    if os.path.exists(cj):
        for l in open(cj):
            try: e = json.loads(l)
            except ValueError: continue
            if e.get("event") == "message" and "started:" in e.get("text", "") and "plan plan-" in e.get("text", ""):
                t0 = e["t"]; break
    R["t_plan"] = t0
    ct = [r for r in rows(os.path.join(p, "run", "camera_track.csv")) if t0 is None or float(r["t"]) >= t0]
    if not ct:
        ct = [r for r in rows(os.path.join(p, "run", "cam_track.csv")) if t0 is None or float(r["t"]) >= t0]
    R["cam"] = track_stats(ct) if ct else None
    sp = [r for r in rows(os.path.join(p, "run", "step.csv")) if t0 is None or float(r["t_recv"]) >= t0]
    R["own"] = track_stats(sp, key_t="t_recv") if sp else None
    od = [r for r in rows(os.path.join(p, "run", "odometry_raw.csv")) if t0 is None or float(r["t"]) >= t0]
    R["odo"] = track_stats(od, kx="x_odo_cm", ky="y_odo_cm") if od else None
    pm = os.path.join(p, "pi_monitor.md")
    if os.path.exists(pm):
        txt = open(pm).read()
        R["pi"] = {k: re.search(r"\| %s[^|]*\| ([0-9.]+) \| ([0-9.]+) \| ([0-9.]+) \|" % k, txt).groups()
                   for k in ("temperature C", "overhead picture lag s", "side picture lag s", "core 1 busy")
                   if re.search(r"\| %s[^|]*\| ([0-9.]+) \| ([0-9.]+) \| ([0-9.]+) \|" % k, txt)}
    res.append(R)

if len(sys.argv) > 2:
    json.dump(res, open(sys.argv[2], "w"), indent=1, default=str)
for R in res:
    print("=" * 8, R["run"])
    print("  found:", R["found"], "| tags seen:", R["tags_seen"], "| bound:", R["bound"])
    print("  asked:", (R["asked"] or "")[:150])
    print("  progress:", {k: "%d/%d" % v[:2] for k, v in R["progress"].items()}, "missing:", {k: v[2] for k, v in R["progress"].items() if v[2]})
    print("  got:", {k: len(v) for k, v in R["got"].items()}, "src:", {k: dict(v) for k, v in R["got_src"].items()}, "others' points:", {k: v for k, v in (R.get("others_points") or {}).items() if v})
    print("  result:", R["result"], R["dur_s"], "planned", R["planned_s"], "plans", R["plans"], "| not started:", (R["not_started"] or "")[:120], "| tb:", R["traceback"])
    print("  battery:", R["battery"], "| counts:", {k: v for k, v in R["counts"].items() if v}, "| re-plan cpu med/max/n:", R["plan_cpu"], "first plan", R["first_plan_cpu"])
    for nm in ("cam", "own", "odo"):
        T = R[nm]
        if T: print("  %s: n %s pair %s | keepout %s | edge %s" % (nm, T["n"], T["pair"], {k: v for k, v in T["keepout"].items() if v[0] < 12}, T["edge"]))
    if R.get("pi"): print("  pi:", R["pi"])
