#!/usr/bin/env python3
"""Per-run digest of Jay-Sun's 16 school Visit points runs of 23 Nov 2026. Read-only on the data.
Usage: python3 -I digest23.py <jayruns23 folder> <school-conference-points.csv> [out.json]
For each run folder: start, robots found and bound, lists asked, stops credited / refused by the run (point_confidence
events), each stop checked against the camera track (median of run/camera_track.csv during the dwell against the
point's typed place), how it ended, event counts, re-plan times, and the closest approach of each robot's middle to
each keep-out rectangle and table edge from the camera track while the plan ran.
Changelog: 2026-11-24 first version (from school-2026-11-18/digest.py)."""
import csv, json, math, os, re, sys, collections, statistics as S

ROOT = sys.argv[1]
PTS = {r["id"]: (float(r["x_cm"]), float(r["y_cm"])) for r in csv.DictReader(open(sys.argv[2]))}
IN = 2.54
PTS["P09"] = (17 * IN, 57.5 * IN); PTS["P10"] = (33 * IN, 37.5 * IN)   # the site file's overrides
TABLE = (396.2, 167.6)
KEEP = {"middle outlet box": ((78.03, 32.97), (6.44, 6.94)), "zone 1 outlet plate": ((25.22, 33.0), (2.44, 6.62)),
        "zone 4 outlet plate": ((130.625, 33.0), (2.25, 6.62))}
RECT = {k: ((c[0] - s[0] / 2) * IN, (c[1] - s[1] / 2) * IN, (c[0] + s[0] / 2) * IN, (c[1] + s[1] / 2) * IN)
        for k, (c, s) in KEEP.items()}
HALF = 4.75          # half the 9.5 cm body: a middle closer than this to a rectangle has the body over it
CHECK_CM = 5.5       # the run's own reach tolerance (e23c), used for the camera check too

def rect_dist(x, y, r):
    return math.hypot(max(r[0] - x, 0, x - r[2]), max(r[1] - y, 0, y - r[3]))

def rows(p):
    if not os.path.exists(p): return []
    with open(p, newline="") as f: return list(csv.DictReader(f))

def jl(p):
    out = []
    if os.path.exists(p):
        for l in open(p, errors="replace"):
            try: out.append(json.loads(l))
            except ValueError: pass
    return out

PAT = collections.OrderedDict([
    ("not_moved", r"did NOT move"), ("impossible", r"not physically possible"),
    ("cam_disagree", r"camera and its own pose disagree"), ("veer", r"RECOVERING"),
    ("held", r"\] \w+ is held: "), ("needs_hand", r"NEEDS A HAND"), ("unsure_edge", r"unsure near an edge"),
    ("tilt_stop", r"tilt stop"), ("edge_guard", r"edge guard"), ("obstacle", r"obstacle [0-9.]+ cm ahead"),
    ("moved_end", r"moved to the end of its list"), ("skipped_pt", r"skipped: |cannot be reached|no room to turn"),
    ("handed", r"its points not done go to"), ("blind", r"no camera sees it: driving"), ("new_plan", r"\] new plan in"),
    ("not_reached", r"NOT REACHED"), ("old_fix", r"carried forward"), ("rejoin_mid_plan", r"rejoined while a plan made without it"),
])

res = []
for d in sorted(os.listdir(ROOT)):
    p = os.path.join(ROOT, d)
    if not os.path.isdir(p): continue
    so = open(os.path.join(p, "run_plan_stdout.txt"), errors="replace").read() if os.path.exists(os.path.join(p, "run_plan_stdout.txt")) else ""
    R = {"run": d, "start": d[9:11] + ":" + d[11:13]}
    m = re.search(r"Found: (.*)", so); R["found"] = m.group(1) if m else None
    R["bound"] = (re.search(r"Bound: (.*)", so) or [None, None])[1]
    R["names"] = (re.search(r"NAMES in this run only[^:]*: (.*)", so) or [None, None])[1]
    R["orders"] = {a: b.split(" -> ") for a, b in re.findall(r"VISIT ORDER (\w+): (.*)", so)}
    R["mode"] = "lists" if R["orders"] else None
    if not R["orders"] and os.path.exists(os.path.join(p, "split.json")):      # table share: the planner chose the lists
        R["orders"] = json.load(open(os.path.join(p, "split.json"))).get("orders") or {}; R["mode"] = "table share"
    R["speed"] = (re.search(r"speeds pinned for this site: drive (\d+) cm/s", so) or [None, None])[1]
    m = re.search(r"NOT STARTED: (.{0,200})", so); R["not_started"] = m.group(1) if m else None
    m = re.search(r"^Result: (.*?) after ([0-9.]+) s \(first plan said ([0-9.]+) s\), (\d+) plan", so, re.M)
    R["result"] = m.group(1) if m else None; R["dur_s"] = float(m.group(2)) if m else None
    m2 = re.search(r"plan plan-\w+ started: (\d) robots, (\d+) steps, planned ([0-9.]+) s", so)
    R["planned_s"] = float(m2.group(3)) if m2 else None
    R["first_plan_cpu"] = (lambda q: float(q.group(1)) if q else None)(re.search(r"planned in ([0-9.]+) s", so))
    R["interrupt"] = "KeyboardInterrupt" in so
    R["tb"] = sorted(set(re.findall(r"^(\w+(?:Error|Exception): .*)$", so, re.M)))
    R["pin_disagrees"] = re.findall(r"PIN DISAGREES: (.{0,120})", so)
    R["battery"] = re.findall(r"(Alpha|Bravo): firmware [^,]+, battery (\d+) %", so)[:2]
    R["card_gb"] = (re.search(r"The Pi's card: ([0-9.]+) GB free", so) or [None, None])[1]
    R["cam2_left_out"] = bool(re.search(r"stream-8001: SEES NO ROBOT", so))
    R["refit"] = bool(re.search(r"CAMERA REFITTED", so))
    live = so.split("plan plan-")[1] if "plan plan-" in so else ""
    live = live.split("\nResult:")[0]
    R["counts"] = {k: len(re.findall(v, live)) for k, v in PAT.items() if re.search(v, live)}
    L = jl(os.path.join(p, "run", "coordinator.jsonl"))
    t0 = next((e["t"] for e in L if e.get("event") == "message" and "plan plan-" in e.get("text", "") and "started" in e["text"]), None)
    t1 = L[-1]["t"] if L else None
    R["t0"] = t0; R["ran_s"] = round(t1 - t0, 1) if t0 else None
    rp = [e for e in L if e.get("event") == "replan"]
    cpu = [e["cpu_s"] for e in rp]
    R["replans"] = {"n": len(cpu), "med": round(S.median(cpu), 2) if cpu else None, "max": round(max(cpu), 1) if cpu else None,
                    "busy_s": round(sum(cpu), 1), "kept": sum(1 for e in rp if e.get("kept", True)),
                    "planned_pts": collections.Counter(e.get("points_planned") for e in rp if "points_planned" in e).most_common(4)}
    R["ev"] = dict(collections.Counter(e.get("event") for e in L if e.get("event") in (
        "veer", "veer_not_at_point", "robot_out", "robot_rejoined", "old_fix_carried", "cam_guard_reset", "point_skipped",
        "points_reassigned", "did_not_move", "step_failed", "rejoin_undone", "move_check_late", "held_creep", "tracker_lost", "blind_start")))
    R["out_why"] = collections.Counter(re.sub(r"\(.*", "", e["why"]).strip()[:60] for e in L if e.get("event") == "robot_out").most_common()
    R["skipped"] = [(e["robot"], e["point"].replace("point ", ""), e["why"][:70]) for e in L if e.get("event") == "point_skipped"]
    R["reassigned"] = [(e["robot"], {k: [x.replace("point ", "") for x in v] for k, v in e["to"].items()}, e["why"][:60]) for e in L if e.get("event") == "points_reassigned"]
    # camera track while the plan ran
    ct = collections.defaultdict(list)
    for r in rows(os.path.join(p, "run", "camera_track.csv")):
        try: ct[r["robot"]].append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"]), r["cams"]))
        except ValueError: pass
    R["cam_src"] = "run/camera_track.csv"
    if not ct:                                # a run ended by hand before its exports: the bridge's fixes are the camera
        R["cam_src"] = "bridge/fixes.csv"
        for r in rows(os.path.join(p, "bridge", "fixes.csv")):
            try: ct[r["robot"]].append((float(r["t"]), float(r["x"]), float(r["y"]), r["cams"]))
            except ValueError: pass
    # stops: the run's verdict and the camera's
    stops = []
    for e in L:
        if e.get("event") not in ("point_confidence", "point_not_reached"): continue
        pt = e["point"].replace("point ", ""); rb = e["robot"]
        a, b = e["t_abs"] - e["dwell_s"], e["t_abs"]
        w = [(x, y) for t, x, y, c in ct.get(rb, []) if a - 0.05 <= t <= b + 0.05]
        tgt = PTS.get(pt)
        cam = None
        if w and tgt:
            cx, cy = S.median(q[0] for q in w), S.median(q[1] for q in w)
            cam = round(math.hypot(cx - tgt[0], cy - tgt[1]), 1)
        own = round(math.hypot(e["own_x"] - tgt[0], e["own_y"] - tgt[1]), 1) if tgt and e.get("own_x") is not None else None
        stops.append({"robot": rb, "point": pt, "t_run": e["t_run"], "credited": bool(e.get("reached", True)), "miss_run": e.get("miss_cm"),
                      "pm": e.get("plus_minus_cm"), "src": e.get("source"), "cam_n": len(w), "cam_miss": cam, "own_miss": own,
                      "interrupted": e.get("interrupted")})
    R["stops"] = stops
    prog = {}
    for rb, lst in R["orders"].items():
        cred = [s for s in stops if s["credited"]]
        # a credited stop counts for the list it is on, whoever drove it (handed-over points)
        want = collections.Counter(lst)
        got_c = collections.Counter(); true_c = collections.Counter(); nocam = 0
        for s in cred:
            if s["point"] in want and got_c[s["point"]] < want[s["point"]]:
                got_c[s["point"]] += 1
                if s["cam_miss"] is None: nocam += 1
                elif s["cam_miss"] <= CHECK_CM: true_c[s["point"]] += 1
        prog[rb] = {"asked": len(lst), "credited": sum(got_c.values()), "cam_ok": sum(true_c.values()), "no_cam": nocam,
                    "missing": [x for x in lst if got_c[x] < want[x]]}
    R["progress"] = prog
    R["refused"] = [(s["robot"], s["point"], s["miss_run"], s["cam_miss"]) for s in stops if not s["credited"]]
    R["cred_far"] = [(s["robot"], s["point"], s["miss_run"], s["cam_miss"], s["pm"], s["cam_n"]) for s in stops if s["credited"] and (s["cam_miss"] is None or s["cam_miss"] > CHECK_CM)]
    # closest approaches while the plan ran
    ko = {}; edge = {}
    for rb, tr in ct.items():
        for t, x, y, c in tr:
            if t0 is None or t < t0: continue
            for k, r in RECT.items():
                dd = rect_dist(x, y, r)
                if (rb, k) not in ko or dd < ko[(rb, k)][0]: ko[(rb, k)] = (round(dd, 1), round(t - t0, 1), round(x, 1), round(y, 1), c)
            for nm, e in (("y=0", y), ("y=167.6", TABLE[1] - y), ("x=0", x), ("x=396.2", TABLE[0] - x)):
                if (rb, nm) not in edge or e < edge[(rb, nm)][0]: edge[(rb, nm)] = (round(e, 1), round(t - t0, 1), round(x, 1), round(y, 1), c)
    R["keepout"] = {"%s|%s" % k: v for k, v in ko.items()}
    R["edge"] = {"%s|%s" % k: v for k, v in edge.items()}
    # two robots: closest middles in the same camera moment
    pair = None
    if len(ct) == 2:
        a, b = [dict((round(t, 3), (x, y)) for t, x, y, c in ct[k] if t0 and t >= t0) for k in sorted(ct)]
        for t in a.keys() & b.keys():
            dd = math.hypot(a[t][0] - b[t][0], a[t][1] - b[t][1])
            if pair is None or dd < pair[0]: pair = (round(dd, 1), round(t - t0, 1))
    R["pair"] = pair
    res.append(R)

if len(sys.argv) > 3: json.dump(res, open(sys.argv[3], "w"), indent=1, default=str)
for i, R in enumerate(res, 1):
    print("=" * 6, i, R["run"], "speed", R["speed"], "card", R["card_gb"], "cam2 left out of binding" if R["cam2_left_out"] else "", "REFIT" if R["refit"] else "")
    print("  mode:", R["mode"], "| camera from", R["cam_src"]); print("  found:", R["found"], "| bound:", R["bound"], "|", R["names"], "| battery", R["battery"])
    if R["pin_disagrees"]: print("  PIN:", R["pin_disagrees"])
    if R["not_started"]: print("  NOT STARTED:", R["not_started"])
    print("  result:", R["result"], "dur", R["dur_s"], "ran", R["ran_s"], "planned", R["planned_s"], "first plan cpu", R["first_plan_cpu"], "| interrupt", R["interrupt"], R["tb"])
    for rb, v in R["progress"].items():
        print("  %s: asked %d, credited %d, camera agrees %d (no camera in dwell %d); missing %s" % (rb, v["asked"], v["credited"], v["cam_ok"], v["no_cam"], ",".join(v["missing"])))
    print("  refused (robot, point, run miss, camera miss):", R["refused"])
    print("  credited but camera > %.1f cm or none:" % CHECK_CM, R["cred_far"])
    print("  counts:", R["counts"]); print("  events:", R["ev"]); print("  out:", R["out_why"])
    print("  replans:", R["replans"]); print("  skipped:", R["skipped"]); print("  reassigned:", R["reassigned"])
    print("  keepout min:", {k: v for k, v in R["keepout"].items()}); print("  edge min:", R["edge"]); print("  pair:", R["pair"])
