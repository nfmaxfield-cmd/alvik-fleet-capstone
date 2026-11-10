#!/usr/bin/env python3
"""Run a mission from one command, with or without Mission Control (one system, 2026-10-28; Nick 18:36: "also runnable
from one command without the page", the safety net for Monday). Mission Control's Run screen starts this too, so the
page and the command are the same run.

    python3 tools/fleet/mission_run.py --table 61x122                        # Monday's default on the robots (Pi)
    python3 tools/fleet/mission_run.py --map logs/mc/map.json --program logs/mc/program.json
    python3 tools/fleet/mission_run.py --table 70x130 --dry                  # the simulator, both simulated cameras
    python3 tools/fleet/mission_run.py --table 61x122 --dry --washout --drop-pct 5 --dead Right@60

What it does, in order (each step logged in the session folder, logs/<date-time>-mission[-dry]/):
  1. the workspace: --map (Mission Control's, with the warm-up's sides and zones) or a blank table of --table size
     (every side "not felt yet": kept away from like an open edge);
  2. the mission: --program (blocks) or Monday's default for --robots (missions.monday_program), built for this table
     and planned (refused if the plan's closest approach is under 8 cm);
  3. the cameras: the camera bridge (cam_bridge.py; --farm: each camera tracked in its own process) on the recorder's
     pictures (real) or the simulator's two cameras (dry);
  4. the run: run_plan.py with every engine check (battery, restarts, settings read back, silent robots, the 8 cm net),
     each robot straightened at the start: its measured pose (camera position, the facing check's heading) against
     its start pose; off: a turn on the spot or a drive to its spot, then the facing check again (straighten.py);
  5. the record (item 6): run_plan's logs (both cameras' positions, every move and its timing, battery, messages),
     the Pi's temperature and throttling (cpu.csv, perf.csv: the heat tracker), the pictures at the tracking rate and
     a clip 5 s either side of every stop (frames: on the USB drive, <drive>/fleet-frames/<session>/frames, since
     2026-10-28; no drive: <session>/frames), the line check, mission_summary.json; --upload: a release with a tag of
     its own (upload_session.py --unique-tag), the frames as their own asset (rec-frames-small.zip).
Tracking during the run (2026-11-03, Nick: "I want to be able to run with or without tags"): --tracking tags (roof
AprilTags, tag bridge), lights (the robots' lights and outlines, camera bridge; home's way) or none (no camera at all:
each robot starts on its home by hand and drives on its wheels and gyro; no bridge, no facing check, no feel-out; the
+- at each point grows 0.07 cm per cm driven). auto = the site's run_tracking, else its tracking. A site with typed
production points (sticker_points.py) gives the map, homes, keep-outs and default orders in every mode.
Ctrl+C stops every robot (run_plan's own stop). Free space (diskguard.py, 2026-10-28): not started when the card is
under its floor; frame saving stops under the frames' floor; under the card's floor during the run, run_plan is sent
Ctrl+C (every robot stops) and the run ends as it would on Ctrl+C, with the upload.
"""
VERSION = "2026-11-10a"   # 2026-11-10a (e12): the camera self-check at the start of a tags run (camera_selfcheck.py; --no-camera-check). No version string before

import argparse
import datetime as dt
import json
import math
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, TOOLS, os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "pi"), os.path.join(TOOLS, "live"),
          os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)

import fleetmap as FM        # noqa: E402
import missions as MS        # noqa: E402
import planner as P          # noqa: E402
import workspace as WS       # noqa: E402
import engine_checks as EC   # noqa: E402
import diskguard as DG       # noqa: E402

PY = sys.executable
EDGE_GUARD = ["set safe_edge 3", "set edge_dir 0", "set edge_thr 150", "set edge_n 3", "set edge_bot_cm 11",
              "set bumper_cm 0"]    # the robots' edge guard, as run-fleet-pi.sh / ladder.SAFE (lost at every power-off)
PLAN_LIMIT_S = 60.0     # planning time allowed for the mission (Monday plans in 4-5 s here). No DOE: a limit
DROP_CM = 74.0          # the simulator's conference table: the floor this far below the top past an open edge (a
                        # typical conference table's height; the site's table.drop_cm when given). No DOE: a model
BRIDGE_START_S = 5.0    # the camera bridge learns the empty floor round the robots before run_plan starts (as
                        # run-fleet-pi.sh). No DOE: the bridge needs 5 pictures


def guard_cmds(args):
    """(11-05h, Nick 5 Nov 20:46) The robot-side safety layers for this run: the edge guard with the site's mode
    (school 2: the bottom zone's drop only; the line sensors read the seam tape) and the tilt stop (observe)."""
    import observe as OB
    se = OB.site_safe_edge(getattr(args, "site_", None), 3)
    out = [("set safe_edge %d" % se) if c.startswith("set safe_edge") else c for c in EDGE_GUARD]
    return out + [c for c in OB.safety_commands(se) if not c.startswith(("set safe_edge", "set edge_bot_cm"))]


def roam_controller(args):
    """(e10, C3) Which controller drives the school's free roam: "freeroam" (the default: the home controller, one
    pipeline; its Mission Control practice run passed 7 Nov) or "planner" (e07: the planner wanders over each robot's
    share of the points; the switch). --roam-controller, or MIRLT_SCHOOL_ROAM."""
    return getattr(args, "roam_controller", None) or os.environ.get("MIRLT_SCHOOL_ROAM", "freeroam")


def table_map(size, robots):
    """A blank table: 'WxL' or 'LxW' cm (the longer side along x)."""
    a, b = (float(v) for v in size.lower().split("x"))
    L, W = max(a, b), min(a, b)
    return WS.blank_map((0, 0, L, W), [(n, [20 + 30 * k, 15, 90]) for k, n in enumerate(robots)])


def say(s):
    print(time.strftime("%H:%M:%S ") + s, flush=True)


def uses_refs(args):
    """Reference tags on the simulated table (2026-11-03): on unless --no-refs or the site switches them off."""
    if getattr(args, "no_refs", False):
        return False
    rt = (getattr(args, "site_", None) or {}).get("ref_tags")
    return True if rt is None else bool(rt.get("use", True))


def corner_clicks(args):
    """Corner clicking (a switch per site, 2026-11-03): --corner-clicks on/off, else the site's corner_clicks, else on."""
    v = getattr(args, "corner_clicks", "auto") or "auto"
    if v != "auto":
        return v == "on"
    return bool((getattr(args, "site_", None) or {}).get("corner_clicks", True))


def observed_split(args, robots, out):
    """(11-05h, Observe then trust) Homes where the robots were seen standing (the binding; the simulator: where it put
    them) and the points shared between however many robots there are over the area the cameras read (observe:
    ReadMap + split_points). Points outside it are skipped with a reason. production.split "halves" (or --split
    halves): the site's fixed zones (kept). An experiment keeps its own orders. Returns (home_at or None, orders or
    None)."""
    import observe as OB
    import sticker_points as SP
    if getattr(args, "experiment_", None):
        return None, None
    mode = args.split if getattr(args, "split", "auto") != "auto" else str(
        ((args.site_ or {}).get("production") or {}).get("split") or "auto")
    pts, zones, _w = SP.points_cm(args.site_)
    face = float(((args.site_ or {}).get("homes") or {}).get("face_deg", 90.0))
    seen = dict(getattr(args, "seen_", None) or {})
    for n, lab in (getattr(args, "sim_homes_", None) or {}).items():
        if n not in seen and lab in pts:
            seen[n] = list(pts[lab])
    home_at = {}
    for n in robots:
        if n in seen:
            x, y = seen[n][:2]
            near = min(pts, key=lambda l: math.dist(pts[l], (x, y)))
            home_at[n] = [pts[near][0], pts[near][1], face] if math.dist(pts[near], (x, y)) <= 12.0 else [x, y, face]
    sh = ((args.site_ or {}).get("homes") or {})
    missing = [n for n in robots if n not in home_at and not (isinstance(sh.get(n), str) and sh.get(n) in pts)]
    if missing:
        # (11-05h) a robot nobody has seen yet and the site has no home for (a third robot, a plan made before the
        # wiggle): a readable point clear of the others, never the table's middle (the outlet box is there)
        picks = dry_homes(args, [n for n in robots if n not in missing] + missing)
        for n in missing:
            lab = picks.get(n)
            if lab in pts:
                home_at[n] = [pts[lab][0], pts[lab][1], face]
    if not getattr(args, "auto_robots", False) and mode == "halves":
        return (home_at or None), None
    rm = run_readmap(args, out)
    args.readmap_ = rm
    pos = {}
    for n in robots:
        if n in home_at:
            pos[n] = home_at[n][:2]
        else:
            hp = ((args.site_ or {}).get("homes") or {}).get(n)
            pos[n] = pts[hp] if isinstance(hp, str) and hp in pts else (0.0, 0.0)
    halves = None
    if mode == "halves":
        import production as PR
        halves = PR.site_orders(args.site_, pts, zones, {n: list(p_) + [face] for n, p_ in pos.items()})
    readable = rm.readable if rm is not None else None
    orders, skipped, words = OB.split_points(pts, pos, readable, mode=mode, halves=halves)
    args.skipped_ = skipped
    for w in words:
        say("TABLE SHARING: " + w)
    if skipped:
        say("SKIPPED (%d points): %s" % (len(skipped), "; ".join("%s: %s" % kv for kv in sorted(
            skipped.items(), key=lambda kv: int(kv[0][1:]) if kv[0][1:].isdigit() else 0))))
    if rm is not None:
        say("READABLE AREA (# reads, . not; x along the table, y up):\n" + rm.text())
    if out:
        with open(os.path.join(out, "split.json"), "w") as f:
            json.dump({"mode": mode, "robots": pos, "orders": orders, "skipped": skipped, "words": words,
                       "homes": home_at}, f, indent=1)
    # the point each robot stands on (seen) is always its own, readable or not
    for n, h in home_at.items():
        lab = next((l for l in pts if math.dist(pts[l], h[:2]) < 1.0), None)
        if lab:
            for o in orders:
                if lab in orders[o] and o != n:
                    orders[o].remove(lab)
            if lab not in orders.get(n, []):
                orders.setdefault(n, []).insert(0, lab)
            skipped.pop(lab, None)
    return (home_at or None), orders


ROAM_DWELL_S = 0.5        # (e07) free roam at a tag site: a half-second stand at each spot (chosen: enough for the
                          # point's position to be logged; no DOE: it is a wander, not a measurement)
ROAM_S_PER_POINT = 14.0   # ... and about this long a leg at production speed (curves), to size the wander. Chosen from
                          # the e06 rehearsals' legs (8-20 s); the run's time limit ends it anyway


def parse_visit_order(spec, robots):
    """(e11, Nick 9 Nov) 'P01,P05,P11,P05' (every robot shares the list: one robot takes it all; more robots take it in
    turn, in order, a run of points each) or 'Alpha:P01,P05;Bravo:P11' (each robot its own list). Labels are made
    P01-style (p5 -> P05). Returns {robot: [labels]} or raises ValueError in plain words."""
    def lab(t):
        t = t.strip().upper()
        if not t:
            return None
        if t[0] == "P" and t[1:].isdigit():
            return "P%02d" % int(t[1:])
        if t.isdigit():
            return "P%02d" % int(t)
        raise ValueError("%r is not a sticker name (P01 to P40)" % t)
    spec = (spec or "").strip()
    if not spec:
        return {}
    if ":" in spec:
        out = {}
        for part in spec.split(";"):
            if not part.strip():
                continue
            n, _, lst = part.partition(":")
            n = n.strip()
            if n not in robots:
                raise ValueError("no robot %s in this run (robots: %s)" % (n, ", ".join(robots)))
            out[n] = [x for x in (lab(t) for t in lst.split(",")) if x]
        return out
    labs = [x for x in (lab(t) for t in spec.replace(" ", ",").split(",")) if x]
    if len(robots) <= 1:
        return {robots[0]: labs} if robots else {}
    k = int(math.ceil(len(labs) / float(len(robots))))
    return {n: labs[i * k:(i + 1) * k] for i, n in enumerate(robots) if labs[i * k:(i + 1) * k]}


def visit_orders(args, robots, out):
    """(e11, Nick 9 Nov) Visit points in the order Nick chose (typed, clicked on the map or in the webcam picture):
    points the camera cannot read are flagged before the run and skipped (--visit-keep-unreadable keeps them); a point
    straight after itself is visited once; repeats later in the list are kept. Writes visit_order.json."""
    import sticker_points as SP
    pts, _z, _w = SP.points_cm(args.site_)
    lists = parse_visit_order(args.visit_order, robots)
    unknown = sorted({l for v in lists.values() for l in v if l not in pts})
    if unknown:
        raise SystemExit("NOT STARTED: no such sticker on this site: %s. Nothing moved." % ", ".join(unknown))
    rm = getattr(args, "readmap_", None) or run_readmap(args, None)
    keep = bool(getattr(args, "visit_keep_unreadable", False))
    flagged, final = {}, {}
    for n, labs in lists.items():
        seq = []
        for l in labs:
            ok = True
            if rm is not None:
                try:
                    ok = bool(rm.readable(*pts[l][:2])[0])
                except Exception:      # noqa: BLE001
                    ok = True
            if not ok:
                flagged[l] = "the camera cannot read its roof tag there (the readable area, %s)" % (
                    "kept as asked" if keep else "skipped")
                if not keep:
                    continue
            if seq and seq[-1] == l:
                continue
            seq.append(l)
        final[n] = seq
    for l, why in sorted(flagged.items()):
        say("VISIT ORDER: %s: %s%s" % (l, why, "" if keep else " (the page's 'try unreadable points anyway', or --visit-keep-unreadable, "
                                                                "keeps it)"))
    for n in robots:
        if n in final:
            say("VISIT ORDER %s: %s" % (n, " -> ".join(final[n]) or "(nothing left to visit)"))
        else:
            say("VISIT ORDER %s: no points (it stays at its home)" % n)
    if out:
        with open(os.path.join(out, "visit_order.json"), "w") as f:
            json.dump({"asked": args.visit_order, "lists": lists, "orders": final, "flagged": flagged,
                       "kept_unreadable": keep}, f, indent=1)
    if not any(final.values()):
        raise SystemExit("NOT STARTED: every point in the order is outside the area the camera reads (%s). Nothing "
                         "moved. Keep them anyway with 'keep them' (--visit-keep-unreadable)." % ", ".join(sorted(flagged)))
    return {n: v for n, v in final.items()}


VISIT_DWELL_S = 8.0      # (e11, Jay-Sun 9 Nov, Experiment A tours) the stop at each point of a chosen order: his number


def visit_dwell_of(args):
    v = getattr(args, "visit_dwell_s", None)
    return float(VISIT_DWELL_S if v is None else v)


def roam_orders(orders, minutes, seed=1):
    """(e07, free roam at a tag site) Each robot wanders over its own share of the readable points: its points in a
    random order, again and again (never the same point twice in a row), enough for about `minutes`."""
    import random
    rng = random.Random("roam-%s" % seed)
    out = {}
    n_legs = max(4, int(minutes * 60.0 / ROAM_S_PER_POINT))
    for n, labs in orders.items():
        labs = list(labs)
        if len(labs) < 2:
            out[n] = labs
            continue
        seq = labs[:1]
        while len(seq) < n_legs:
            perm = list(labs)
            rng.shuffle(perm)
            if perm[0] == seq[-1]:
                perm = perm[1:] + perm[:1]
            seq += perm
        out[n] = seq[:n_legs]
    return out


def build(args, out):
    robots = [r.strip() for r in args.robots.split(",") if r.strip()]
    cal = getattr(args, "feel_cal_", None)
    if args.map:
        m = FM.load(args.map)
        m["tasks"] = {}
        if cal and getattr(args, "tracking_", "lights") == "tags":
            # the table's calibration from feeling it out (2026-11-02): the outline the robots felt
            import feel_table as FTB
            m, _w = FTB.apply_to_map(m, cal)
            for w_ in _w:
                say(w_)
    elif typed_site(args):
        # a site with typed production points (2026-11-03, Jay's sticker sheet): its points, homes, raised box and
        # default orders, in every tracking mode
        import production as PR
        home_at, orders_ = observed_split(args, robots, out)
        roam = float(getattr(args, "roam_min", 0) or 0)
        if roam > 0 and orders_:
            orders_ = roam_orders(orders_, roam, args.seed)
        if getattr(args, "visit_order", ""):
            orders_ = visit_orders(args, robots, out)       # (e11, Nick 9 Nov) the points in the order Nick chose
        m, _w, sprog, _o = PR.site_production(args.site_, robots, outline=(cal or {}).get("outline_used"),
                                              experiment=getattr(args, "experiment_", None) or None,
                                              swap=getattr(args, "swap_", False),
                                              wiggle=False if roam > 0 else
                                              {"on": True, "off": False}.get(getattr(args, "sync_wiggle", "auto")),
                                              home_at=home_at, orders=orders_,
                                              keep_order=roam > 0 or bool(getattr(args, "visit_order", "")),
                                              dwell_s=ROAM_DWELL_S if roam > 0 else (
                                                  visit_dwell_of(args) if getattr(args, "visit_order", "") else None))
        args.site_program_ = sprog
        for w_ in _w:
            say(w_)
    elif getattr(args, "tracking_", "lights") == "tags":
        # the AprilTag site (2026-11-02): the table with its production points and the homes near the middle
        import production as PR
        a_, b_ = (float(v) for v in args.table.lower().split("x"))
        pts = {}
        import tag_cam as TCAM
        dcfg = TCAM.default_config(table=(max(a_, b_), min(a_, b_)), seed=args.seed + 4, refs=uses_refs(args))
        if not args.program:
            # no program given (a dry run from the command line): the simulator's stickers as the points
            pts = dcfg["stickers"]
        kos = None
        if dcfg.get("ref_tags"):
            # the reference tiles (2026-11-03): no robot drives over them (the simulator's tiles where they lie)
            import ref_tags as RT
            kos = RT.keepouts(dcfg["ref_tags"]["poses"], dcfg["ref_tags"].get("config"))
        m, _w = PR.site_map((max(a_, b_), min(a_, b_)), robots, points=pts,
                            outline=(cal or {}).get("outline_used"), keepouts=kos)
        for w_ in _w:
            say(w_)
    else:
        m = table_map(args.table, ["Left", "Center", "Right"])
    if not ((getattr(args, "site_", None) or {}).get("walls", False)):
        # (11-05h, Nick 21:14) neither the home board nor the school table has a wall or rim: every side is a drop for
        # every safety layer, whatever the map says (a side marked barrier by a warm-up is kept out of as an edge)
        for k_ in FM.SIDES:
            sd = (m.get("sides") or {}).get(k_) or {}
            if sd.get("kind") in ("barrier", "partial"):
                say("SIDES: %s was marked %s; this site has no walls (site walls: false): treated as an open drop" % (
                    k_, sd.get("kind")))
                m = WS.set_side(m, k_, "open", source="site has no walls (11-05h)")
    if args.program:
        with open(args.program) as f:
            prog = json.load(f)
    elif getattr(args, "site_program_", None):
        prog = args.site_program_                      # the typed site's default production mission
    elif getattr(args, "tracking_", "lights") == "tags":
        # a production run over the map's points (half each, in order), then home (2026-11-02)
        import production as PR
        labs = sorted((m.get("spots") or {}), key=lambda k: (len(k), k))
        orders = {n: labs[k::len(robots)][:args.points_each] for k, n in enumerate(robots)}
        prog = PR.program(orders, dwell_s=args.dwell_s)
    else:
        prog = MS.monday_program(tuple(robots))
    sets = dict(kv.split("=", 1) for kv in args.setting)
    # the 8 cm lock (2026-10-29): no 2-robot mission under the net from the command line either (the simulator's
    # safety-net DOE: --dry --below-net-doe)
    raised = MS.gap_lock_settings(dict((k, float(v)) for k, v in sets.items() if k in MS.GAP_KEYS_MISSION),
                                  where="--setting")
    for w in raised:
        say("8 cm LOCK: " + w)
    for k in MS.GAP_KEYS_MISSION:
        if k in sets and not MS._BELOW_NET_DOE[0]:
            sets[k] = str(max(float(sets[k]), MS.gap_floor()))
    if "chicken_min_cm" in sets:
        # the program's own chicken min_cm (Monday's says 8) gives way to an explicit --setting (safety-net DOE)
        for bl in prog.get("blocks", []):
            if bl.get("block") == "chicken":
                bl["min_cm"] = float(sets["chicken_min_cm"])
    if not MS._BELOW_NET_DOE[0]:
        for bl in prog.get("blocks", []):
            # a program's own Chicken min_cm (Mission Control's program.json, a saved edit) is held to the net too
            if bl.get("block") == "chicken" and "min_cm" in bl and float(bl["min_cm"]) < MS.gap_floor():
                say("8 cm LOCK: the program's Chicken min_cm %.1f cm raised to %.1f cm" % (float(bl["min_cm"]),
                                                                                        MS.gap_floor()))
                bl["min_cm"] = MS.gap_floor()
    mm = MS.build(m, prog, s=sets or None)
    over = {"separation_cm": max(float(((mm.get("mission") or {}).get("planner") or {}).get("separation_cm", 8.0)),
                                 args.min_gap_cm)}
    if typed_site(args):
        # same speed in every experiment (Jay-Sun, 3 Nov): the site's production speeds pinned in the planner (re-plans
        # keep the plan's settings), recorded in the summary
        import experiments as EX
        sp = dict(((prog.get("production") or {}).get("experiment") or {}).get("speed") or EX.conf(args.site_)["speed"])
        if getattr(args, "visit_order", "") and getattr(args, "visit_speed_cms", None):
            sp["cruise_cms"] = float(args.visit_speed_cms)       # (e11, Jay-Sun's Experiment A tours: 10 cm/s)
        over.update(sp)
        args.speed_ = sp
        say("speeds pinned for this site: drive %.0f cm/s, turn %.0f deg/s, final approach %.0f cm/s" % (
            sp["cruise_cms"], sp["turn_dps"], sp["confirm_cms"]))
    if getattr(args, "visit_order", "") and getattr(args, "visit_chunk_cm", None) is not None:
        # (Nick 9 Nov, after the first tour stepped every ~10 cm) a visit tour's drives cut into pieces of at most this
        # (0 = each leg in one go); the planner's 15 cm pieces (camera re-checks between them) stay the default elsewhere
        ck = float(args.visit_chunk_cm)
        over["chunk_cm"] = 10000.0 if ck <= 0 else max(5.0, ck)
        say("VISIT DRIVES: %s" % ("each leg in one go (no stops between points)" if ck <= 0 else
                                  "pieces of at most %.0f cm" % over["chunk_cm"]))
    over["motion"] = args.motion       # (5 Nov) stop-turn-drive (default) or curves at the corners
    if args.motion == "curves":
        say("motion: curves at the corners where there is room (--motion stopturn: stop and turn at every corner)")
    t0 = time.time()
    if getattr(args, "on_problem_", "stop") == "keep_going":
        over["skip_unreachable"] = True     # keep going (2026-11-03): a point no route reaches is skipped and listed,
        #                                     not a plan refused
    plan = MS.plan_synced(mm, over=over, t_limit_s=PLAN_LIMIT_S, log=say)
    gap = EC.plan_gap(plan, mm, gate_cm=args.min_gap_cm)
    bad = EC.gate_words(gap, args.min_gap_cm)
    mp, pp = os.path.join(out, "mission.json"), os.path.join(out, "plan.json")
    with open(mp, "w") as f:
        json.dump(mm, f, indent=1)
    P.save_plan(plan, pp)
    with open(os.path.join(out, "program.json"), "w") as f:
        json.dump(prog, f, indent=1)
    f_ = mm["floor"]
    say("mission %s on a %.0f x %.0f cm table, robots %s: planned in %.1f s, done in %.0f s, closest %.1f cm"
        % (prog.get("name"), f_["x_max"] - f_["x_min"], f_["y_max"] - f_["y_min"], ", ".join(plan["robots"]),
           time.time() - t0, plan["summary"]["makespan_s"], gap["closest_cm"]))
    for n in mm["mission"].get("notes", []):
        say("  " + n)
    for w in plan.get("warnings") or []:
        if " skips task " in str(w) and "(at point" not in str(w):
            say("KEEP GOING: %s (skipped; listed in the summary)" % w)
    if bad:
        raise SystemExit("Plan refused: %s" % bad)
    return mm, plan, mp, pp


def start_bridge(args, out, mp, cal, scal, stream, side_stream):
    try:                       # (11-10a, 9 Nov school) a camera reader left by an earlier run holds its port
        import stale_bridge as SB
        SB.clear((5011,), log=say)
    except Exception as e:     # noqa: BLE001  (never stops a run)
        say("(leftover camera reader check skipped: %s)" % e)
    if getattr(args, "tracking_", "lights") == "tags" and not args.dry and not getattr(args, "no_camera_check", False):
        # (e12, Nick 10 Nov; Observe, then trust) the camera model checked against the reference tags before the robots
        # move; refitted (Measure) when it disagrees, what changed said and logged (camera_selfcheck.json)
        try:
            import camera_selfcheck as CSC
            CSC.run(args.site_ or {}, out=out, log=say)
        except Exception as e:  # noqa: BLE001  (never stops a run: the model it has is used)
            say("(camera self-check skipped: %s)" % e)
    if getattr(args, "tracking_", "lights") == "tags":
        # the AprilTag robots (2026-11-02): tag_bridge.py, both cameras, the tags' own ids and heights
        cmd = ["nice", "-n", "5", PY, os.path.join(HERE, "tag_bridge.py"), "--tags", args.tags_json,
               "--cam", "overhead=%s,%s" % (stream, cal), "--out", os.path.join(out, "bridge"), "--wait-calib-s", "90",
               "--robots", args.robots]
        try:
            fl_ = FM.load(mp)["floor"]           # (e10, C3) the table: the bridge's motion detector looks inside it
            cmd += ["--board", "%g,%g,%g,%g" % (fl_["x_min"], fl_["y_min"], fl_["x_max"], fl_["y_max"])]
        except (OSError, KeyError, ValueError, TypeError):
            pass
        if scal:
            cmd += ["--cam", "side=%s,%s" % (side_stream, scal)]
        f = open(os.path.join(out, "bridge.txt"), "w")
        return subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=REPO), f
    cmd = ["nice", "-n", "5", PY, os.path.join(HERE, "cam_bridge.py"), "--map", mp, "--calib", cal, "--stream", stream,
           "--out", os.path.join(out, "bridge"), "--wait-calib-s", "40"]
    if scal:
        cmd += ["--side-calib", scal, "--side-stream", side_stream]
    if args.farm:
        cmd += ["--farm"]
        if args.rings:
            cmd += ["--overhead-ring", "mirlt-overhead", "--side-ring", "mirlt-side"]
        if args.half:
            cmd += ["--half"]
    f = open(os.path.join(out, "bridge.txt"), "w")
    return subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=REPO), f


def space_watch(guard, child, said, stop_evt):
    """While run_plan drives: the card under its floor -> Ctrl+C to run_plan (every robot stops), once."""
    while not stop_evt.wait(DG.CHECK_S * 2):
        st = guard.state()
        if st["stop"] and child.poll() is None:
            said.append(st["why"])
            say("THE PI'S CARD IS NEARLY FULL: %s. Stopping every robot (as Ctrl+C); the logs are closed and "
                "uploaded next. Before the next run: ./run-fleet-pi.sh space" % st["why"])
            child.send_signal(signal.SIGINT)
            return


def bridge_watch(events_path, stop_evt, said):
    """While the run goes (2026-11-03): the tag bridge's camera_moved events (a bumped camera, its reference tags off
    where the setup saw them) said on this screen (Mission Control shows these lines) and kept for the summary. The
    run is not stopped by this (Nick decides: the robots' own sensors and the other camera still work)."""
    pos = 0
    while not stop_evt.wait(1.0):
        try:
            with open(events_path) as f:
                f.seek(pos)
                for ln in f:
                    if not ln.endswith("\n"):
                        break
                    pos += len(ln.encode())
                    try:
                        ev = json.loads(ln)
                    except ValueError:
                        continue
                    if ev.get("kind") == "camera_moved":
                        said.append(ev)
                        say("WARNING: camera %s has MOVED (bumped?): its reference tags are %.1f px, about %.1f cm on "
                            "the table, from where the setup saw them. Its positions are off by about that much. Stop "
                            "the run and Measure again (Cameras & table): with the tags it takes a minute." % (
                                ev.get("cam"), ev.get("median_px", 0), ev.get("about_cm", 0)))
                    elif ev.get("kind") == "camera_back":
                        say("camera %s: its reference tags are back where the setup saw them" % ev.get("cam"))
        except OSError:
            continue


def gap_args(args):
    """The 8 cm lock on the command line: --min-gap-cm under the net is raised for a 2-robot mission (the simulator's
    safety-net DOE: --dry --below-net-doe)."""
    if args.below_net_doe:
        if not args.dry:
            raise SystemExit("--below-net-doe is for the simulator only (--dry): nothing was moved")
        MS.below_net_doe(True)
        say("SIMULATOR DOE: robot gaps under the 8 cm safety net allowed for this dry run only")
        return
    n = len([r for r in args.robots.split(",") if r.strip()])
    if n >= 2 and args.min_gap_cm < MS.gap_floor():
        say("8 cm LOCK: --min-gap-cm %.1f raised to %.1f cm (no 2-robot mission runs under the safety net)"
            % (args.min_gap_cm, MS.gap_floor()))
        args.min_gap_cm = MS.gap_floor()


def typed_site(args):
    """The site's production points are typed (sticker_points.py; the site's points.use "typed")."""
    import sticker_points as SP
    return bool(getattr(args, "site_", None)) and SP.typed_on(args.site_)


def resolve_site(args):
    """--site / the site in use: its tracking (lights or tags), its tags and calibration folder (2026-11-02). Home (the
    default) changes nothing: lights, as before."""
    import sites as SI
    site = SI.load(args.site) if (args.site or args.tracking != "lights") else None
    if site is None:
        try:
            site = SI.load()
        except SystemExit as e:
            if SI.is_site_file_error(e):
                raise            # a broken or missing chosen site stops loudly (2026-11-03), never home in silence
            site = None
    tr = args.tracking if args.tracking != "auto" else ((site or {}).get("run_tracking") or (site or {}).get(
        "tracking") or "lights")
    args.tracking_ = tr
    args.site_ = site
    # the keep-going policy (2026-11-03): this run's --on-problem / --lost-robot-points / --feel-trust, else the site's,
    # else home's (stop, as before)
    import sites as SI_
    for k, a in (("on_problem", "on_problem"), ("lost_robot_points", "lost_robot_points"),
                 ("feel_out_trust", "feel_trust")):
        v = getattr(args, a, "auto") or "auto"
        setattr(args, k + "_", v if v != "auto" else ((site or {}).get(k) or SI_.POLICY_DEFAULTS[k]))
    if site and not os.environ.get("MIRLT_CONF_DIR"):
        # the site's own folder for the day's calibration records and stored corrections (run_plan inherits it)
        os.environ["MIRLT_CONF_DIR"] = site["conf_dir"]
    if tr in ("tags", "none", "lights") and args.robots == "Left,Right" and site and site.get("tracking") == "tags":
        args.robots = ",".join(r["name"] for r in site["robots"])        # the site's robots, not home's
    args.auto_robots = str(getattr(args, "robots", "")).strip().lower() == "auto"
    if args.auto_robots:
        # (11-05h, Observe then trust) however many robots are on the table: found by the wiggle. The simulator's
        # robots: --sim-robots of them, named as the binding would name them
        import wiggle_bind as WB
        args.robots = ",".join(WB.auto_slots(site, max(1, int(getattr(args, "sim_robots", 2) or 2)))) if args.dry \
            else ",".join(WB.auto_slots(site, 1))
    side = ((site or {}).get("cameras") or {}).get("side") or {}
    if side.get("use") is False and not getattr(args, "no_side", False) and tr == "tags":
        args.no_side = True
        say("CAMERAS: the site's camera 2 is switched off (cameras.side.use false): camera 1 alone; the table is "
            "shared over what it reads")
    if tr == "tags":
        tags = dict((site or {}).get("tags") or {})
        if not tags.get("ids"):
            tags.update({"family": "tag25h9", "tag_cm": 8.0, "roof_cm": 9.5,
                         "ids": {n: k for k, n in enumerate(r.strip() for r in args.robots.split(",") if r.strip())}})
        if getattr(args, "auto_robots", False):
            # (11-05h) ids for robots the site does not list (simulator: a third robot), from the spares
            ids = dict(tags.get("ids") or {})
            spare = [int(v) for v in tags.get("spares") or []] + list(range(10, 20))
            for n in (r.strip() for r in args.robots.split(",") if r.strip()):
                if n not in ids:
                    ids[n] = next(v for v in spare if v not in ids.values())
            tags["ids"] = ids
        if tags.get("height_cm") is None:
            tags["height_cm"] = 6.0
            say("NOTE: the roof's height is not measured yet: 6.0 cm used (PLACEHOLDER; tags.height_cm in the site)")
        args.tags_ = tags
    return site


def sync_mark(out, what, args, extra=None):
    """A sync mark in the session folder (the Pi's time and clock status; Mission Control's buttons add more)."""
    try:
        import sync_tools as ST
        m = ST.mark(out, what, session=getattr(args, "session_id", None), extra=extra)
        say("SYNC MARK %s: %s (Pi clock: %s)" % (what, m["t_iso"], m["clock"].get("words")))
    except Exception as e:     # noqa: BLE001  (a mark never stops a run)
        say("(sync mark %s not written: %s)" % (what, e))


def feel_conf_dir(args, out):
    """Where the table calibrations live: --feel-conf-dir, else the site's folder (robots), else the session's own
    (a dry run: nothing kept unless a folder is given)."""
    if args.feel_conf_dir:
        return os.path.abspath(os.path.expanduser(args.feel_conf_dir))
    if not args.dry and args.site_:
        return args.site_["conf_dir"]
    return os.path.join(out, "feel_conf")


def feel_first(args, out):
    """Feel out the table first (Nick, 2 Nov 18:33): on the first run on a table (per site, per table, per camera
    setup), before the production run: a run of its own (calibration, the crawls), then the comparison, saved as the
    table's calibration; later runs use the saved one. --feel-table: auto (default for the robots), always, skip
    (default for a dry run). Returns an exit code to stop with, or 0 to go on to the production run."""
    import feel_table as FTB
    import production as PR
    mode = args.feel_table or ("skip" if args.dry or (args.site_ or {}).get("feel_out", True) is False else "auto")
    args.feel_cal_ = None
    if mode == "skip":
        return 0
    robots = [r.strip() for r in args.robots.split(",") if r.strip()]
    if args.map:
        m = FM.load(args.map)
        m["tasks"] = {}
    elif typed_site(args):
        m = PR.site_production(args.site_, robots)[0]       # the site's homes (2026-11-03: at points P11 / P22)
        m["tasks"] = {}
    else:
        a_, b_ = (float(v) for v in args.table.lower().split("x"))
        m, _ = PR.site_map((max(a_, b_), min(a_, b_)), robots)
    conf = feel_conf_dir(args, out)
    tb_ = (args.site_ or {}).get("table") or {}
    # the table's name: the site's typed size when it has one (a refit or a moved side never renames it)
    key = FTB.table_key(table=(tb_["length_cm"], tb_["width_cm"])) if tb_.get("length_cm") and tb_.get("width_cm") \
        else FTB.table_key(m)
    sid = "simulator" if args.dry else FTB.setup_id(os.path.join(conf, "tagcal"))
    need, cal, why = FTB.needed(conf, key, sid, force=(mode == "always"))
    say("FEEL OUT THE TABLE: %s" % why)
    if not need:
        args.feel_cal_ = cal
        for w in (cal or {}).get("adjusted") and cal.get("words") or []:
            say("  (saved) " + w)
        return 0
    fdir = os.path.join(out, "feel")
    kg = args.on_problem_ == "keep_going"
    # the feel-out's own runs: the same policy; a lost robot's crawls are not handed on (they are its half's edges)
    extra = ["--robots", ",".join(robots), "--seed", str(args.seed), "--timeout-s", str(args.timeout_s),
             "--on-problem", args.on_problem_, "--lost-robot-points", "skip"]
    if args.dry and args.sim_true_floor:
        extra += ["--sim-true-floor", args.sim_true_floor]
    if getattr(args, "binding_file_", None):
        extra += ["--binding-file", args.binding_file_]     # (5 Nov) the feel-out runs use this run's wiggle binding
    for a in args.sim_arg:
        extra += ["--sim-arg=%s" % a]
    for ip in args.robot:
        extra += ["--robot", ip]
    if args.no_frames:
        extra += ["--no-frames"]
    site_name = (args.site_ or {}).get("name")
    # 1. the calibration, then each robot standing still at home: the surface check (the risk review, 2 Nov)
    sdir = os.path.join(out, "surface")
    smp, spp = FTB.prepare_surface(m, sdir)
    say("feel-out, 1 of 2: each robot calibrated, then standing still %.0f s: is the tabletop clearly apart from the "
        "edge sensors' thresholds?" % FTB.SURFACE_HOLD_S)
    code = subprocess.call([PY] + FTB.mission_run_args(smp, spp, sdir, site=site_name, dry=args.dry, extra=extra,
                                                       calibrate=True), cwd=REPO)
    if code and kg:
        say("KEEP GOING: the surface check did not finish (exit %s, see %s): no crawls; the production run uses the "
            "cameras' outline as it is" % (code, sdir))
        return 0
    if code:
        say("THE SURFACE CHECK DID NOT FINISH (exit %s): the production run is not started. Look at %s" % (code, sdir))
        return code or 1
    surf = FTB.surface_check(os.path.join(sdir, "run"))
    for w in surf["words"]:
        say("  " + w)
    if not surf["ok"]:
        cal = FTB.surface_cal(conf, key, sid, FTB.rect(FM.normalize(m)), surf, site=site_name)
        say("WARNING: no crawls: the production run uses the camera's outline as it is (saved: %s)"
            % FTB.cal_path(conf, key))
        args.feel_cal_ = cal
        return 0
    # 2. the crawls (the calibration is kept: same power-on)
    mp, pp, sp, mk = FTB.prepare(m, fdir)
    say("feel-out, 2 of 2: %d spots (%s), about %.0f s" % (
        len(sp), ", ".join("%s %d" % (n, len([s for s in sp if s["robot"] == n])) for n in robots), mk))
    cmd = [PY] + FTB.mission_run_args(mp, pp, fdir, site=site_name, dry=args.dry, extra=extra)
    code = subprocess.call(cmd, cwd=REPO)
    if code and not kg:
        say("THE FEEL-OUT DID NOT FINISH (exit %s): the production run is not started. Look at %s" % (code, fdir))
        return code or 1
    if code:
        say("KEEP GOING: the feel-out did not finish (exit %s, see %s): what it felt is used; the production run "
            "starts" % (code, fdir))
    try:
        res, cal = FTB.finish(FM.load(mp), os.path.join(fdir, "run"), sp, conf, key, sid, site=site_name, surface=surf,
                              trust=args.feel_out_trust_)
    except Exception as e:     # noqa: BLE001
        if not kg:
            raise
        say("KEEP GOING: the feel-out's result could not be read (%s: %s): the cameras' outline is used" % (
            type(e).__name__, e))
        return 0
    for w in res["words"]:
        say("  " + w)
    say("saved as this table's calibration: %s" % FTB.cal_path(conf, key))
    if res.get("hold") and kg:
        say("KEEP GOING: the outline is too far off on a side (above) and was not moved; the production run starts "
            "on the cameras' outline")
        args.feel_cal_ = cal
        return 0
    if res.get("hold"):
        say("NOT STARTED: the production run waits (the outline is too far off). Do the camera setup again.")
        return 4
    args.feel_cal_ = cal
    return 0


def wiggle_first(args, out):
    """(Nick, 5 Nov 2026) Before every run with roof tags: each robot wiggles, the tag that turns with it is its tag,
    and it is given its slot (args.robots: Alpha, Bravo ...), renamed if needed (wiggle_bind.py). The site's
    tags.binding "fixed" (or --binding fixed) keeps the typed tags.ids (the old way). The simulator's robots carry
    their slot names and tags already: not wiggled (said). Returns 0, or an exit code with nothing run."""
    import wiggle_bind as WB
    mode = WB.binding_mode(args.site_, getattr(args, "binding", "auto"))
    if mode != "wiggle":
        say("TAGS: fixed ids from the site (tags.binding %s): %s" % (mode, ", ".join(
            "%s=%s" % kv for kv in (args.tags_.get("ids") or {}).items())))
        return 0
    if getattr(args, "binding_file", None):
        # a feel-out run started by a run that has just bound them (the same robots, the same power-on)
        with open(args.binding_file) as f:
            rec = json.load(f)
        args.tags_ = WB.tags_with_binding(args.tags_, rec)
        args.binding_file_ = args.binding_file
        args.aliases_ = dict(rec.get("aliases") or {})
        say("TAGS from this run's wiggle (%s): %s" % (args.binding_file, ", ".join(
            "%s = tag %d" % kv for kv in sorted(rec["slots"].items()))))
        return 0
    if args.dry and not getattr(args, "sim_bind", False):
        say("TAGS: simulator: the robots carry their slot names and tags (no wiggle-first binding in a dry run; "
            "--sim-bind runs it)")
        return 0
    if args.dry:
        return dry_bind(args, out)
    slots = ["auto"] if getattr(args, "auto_robots", False) else [r.strip() for r in args.robots.split(",") if r.strip()]
    tc = os.path.join((args.site_ or {}).get("conf_dir") or os.path.expanduser("~/.config/mirlt"), "tagcal")
    say("WIGGLE FIRST: each robot wiggles in turn; the roof tag that turns with it is its tag, and it is given its "
        "slot (%s)" % ", ".join(slots))
    try:
        import calib_run as CRN
        rec = WB.run_for_mission(args.site_, slots, os.path.join(out, "bind"), stream=args.stream,
                                 calib=args.calib or os.path.join(tc, "overhead.json"), ips=args.robot)
    except (Exception, SystemExit) as e:   # noqa: BLE001  (BindError, RunEnd, the port in use)
        say("NOT STARTED: the wiggle-first binding did not finish: %s. To run with the typed tag ids instead: "
            "--binding fixed (or the site's tags.binding \"fixed\")." % e)
        return 5
    if rec["empty_slots"]:
        found = sorted(rec["slots"])
        say("NOT STARTED: no robot for %s (robots bound: %s). Switch the missing robot on, or run with --robots %s." % (
            ", ".join(rec["empty_slots"]), ", ".join(found) or "none", ",".join(found)))
        return 5
    args.tags_ = WB.tags_with_binding(args.tags_, rec)
    args.binding_file_ = os.path.join(out, "bind", "binding.json")
    args.aliases_ = dict(rec.get("aliases") or {})
    take_binding(args, rec)
    say("TAGS from the wiggle: %s" % ", ".join("%s = tag %d" % kv for kv in sorted(rec["slots"].items())))
    if args.aliases_:
        say("NAMES in this run only (the robots are not renamed: tags.rename_to_slot false): %s" % ", ".join(
            "%s is %s" % kv for kv in sorted(args.aliases_.items())))
    return 0


def stop_lead_specs(args, out):
    """(e07, Nick 6 Nov: missions use each robot's stop lead from its last calibration) {run name: "5:0.31,10:0.52"}
    from the Pi's stop_leads.json (calib_run keeps it by WiFi name), through the binding's names (WiFi name -> run
    name). A dry run reads <session>/stop_leads.json too (rehearsals seed it), by WiFi or run name."""
    files = [os.path.join((args.site_ or {}).get("conf_dir") or "", "stop_leads.json")]
    if args.dry:
        # a dry run never reads the Pi's; rehearsals seed stop_leads.json in the session or beside it (Mission
        # Control's runs folder)
        files = [os.path.join(os.path.dirname(out), "stop_leads.json"), os.path.join(out, "stop_leads.json")]
    leads = {}
    for f_ in files:
        try:
            with open(f_) as fh:
                leads.update(json.load(fh))
        except (OSError, ValueError):
            pass
    if not leads:
        return {}
    names = dict(getattr(args, "aliases_", None) or {})          # WiFi name -> run name
    run_names = [r.strip() for r in str(args.robots).split(",") if r.strip()]
    out_ = {}
    for key, rec in leads.items():
        n = names.get(key) or (key if key in run_names or args.dry else None)
        sl = (rec or {}).get("stop_lead_cm") or {}
        if n and sl:
            out_[n] = ",".join("%g:%g" % (float(a), float(b)) for a, b in sorted(sl.items(), key=lambda kv: float(kv[0])))
    if out_:
        say("STOP LEAD (each robot's last calibration): %s" % "; ".join("%s %s" % kv for kv in sorted(out_.items())))
    return out_


def experiment_unseen(args, out):
    """(11-05h, Jay-Sun 5 Nov: without the second camera any run on half the table needs Dr. Sodhi's OK) An experiment
    whose robots are assigned points the cameras cannot read is refused, saying which and why; --allow-half-table runs
    it anyway (with that OK). None when fine."""
    if not getattr(args, "experiment_", None) or args.tracking_ != "tags" or getattr(args, "allow_half_table", False):
        return None
    try:
        with open(os.path.join(out, "program.json")) as f:
            orders = ((json.load(f).get("production") or {}).get("orders")) or {}
    except (OSError, ValueError):
        return None
    rm = run_readmap(args, None)
    if rm is None:
        return None
    import sticker_points as SP
    pts, _z, _w = SP.points_cm(args.site_)
    unseen = {}
    for n, labs in orders.items():
        for l in labs:
            if l in pts and not rm.readable(*pts[l][:2])[0]:
                unseen.setdefault(n, []).append(l)
    if not unseen:
        return None
    return ("experiment %s assigns points the camera cannot read (%s): with one camera only half the table is seen, "
            "and a run on half the table needs Dr. Sodhi's OK (Jay-Sun 5 Nov). With that OK: --allow-half-table; or "
            "plug in camera 2" % (args.experiment_, "; ".join("%s: %s" % (n, ", ".join(v)) for n, v in unseen.items())))


def take_binding(args, rec):
    """(11-05h) What the binding observed, for the plan: the robots found (auto), where each stands, each tag's
    heading offset."""
    if getattr(args, "auto_robots", False):
        args.robots = ",".join(sorted(rec["slots"], key=lambda n: list(rec["slots"]).index(n)))
        say("ROBOTS FOUND: %d (%s)" % (len(rec["slots"]), args.robots))
    args.seen_ = {r["slot"]: r["seen_cm"] for r in rec.get("robots") or [] if r.get("seen_cm")}
    if rec.get("yaw_off"):
        say("TAG HEADINGS (observed by the nudge): %s" % ", ".join("%s %+.0f deg" % kv for kv in sorted(
            rec["yaw_off"].items())))


def dry_homes(args, names):
    """(11-05h, simulator) Where the simulated robots stand: the site's homes when the cameras read them, else the
    readable points furthest from the others (a third robot; a home the single camera cannot see)."""
    import sticker_points as SP
    import observe as OB
    pts, _z, _w = SP.points_cm(args.site_)
    rm = run_readmap(args, None)
    hp = (args.site_ or {}).get("homes") or {}
    out = {}
    for n in names:
        lab = hp.get(n)
        if isinstance(lab, str) and lab in pts and (rm is None or rm.readable(*pts[lab])[0]) and \
                all(math.dist(pts[lab], pts[o]) > 40 for o in out.values()):
            out[n] = lab
    for n in names:
        if n in out:
            continue
        cand = [l for l in pts if (rm is None or rm.readable(*pts[l])[0]) and l not in out.values() and
                OB_edge_ok(pts[l], args)]
        if not cand:
            # (e07, new-area rehearsal: every grid point 17 cm from an edge, so none passed the 25 cm rule and both
            # simulated robots were put on the same point) any readable point not taken, then any not taken
            cand = [l for l in pts if (rm is None or rm.readable(*pts[l])[0]) and l not in out.values()] or \
                [l for l in pts if l not in out.values()]
        cand.sort(key=lambda l: -min([math.dist(pts[l], pts[o]) for o in out.values()] or [1e9]))
        out[n] = cand[0] if cand else sorted(pts)[0]
    return out


def OB_edge_ok(p, args):
    tb = (args.site_ or {}).get("table") or {}
    L, W = float(tb.get("length_cm") or 396.2), float(tb.get("width_cm") or 167.6)
    return min(p[0], p[1], L - p[0], W - p[1]) >= 25.0


class _BindCam:
    """(11-05h, simulator) The point-fix camera for the dry binding, made from the SAME simulated cameras the mission's
    picture simulator draws (camera 2 left out when it is off), so a robot reads in the binding exactly where it reads
    in the run."""

    def __init__(self, args):
        import numpy as np
        import tag_cam as TCAM
        tb = (args.site_ or {}).get("table") or {}
        L, W = float(tb.get("length_cm") or 396.2), float(tb.get("width_cm") or 167.6)
        cfg = TCAM.default_config(table=(L, W), size=tuple(int(v) for v in args.cam_size.split("x")),
                                  seed=args.seed + 4, refs=uses_refs(args))
        self.cams = [c for c in TCAM.cams_from_config(cfg) if not (c.name == "side" and args.no_side)]
        self.C = np.array(self.cams[0].C, float)
        self.dark = [[float(v) for v in d.split(",")] for d in (args.sim_dark or [])]

    def tag_px(self, x, y, z, s, th_deg=0.0):
        import observe as OB
        if any(d[0] <= x <= d[2] and d[1] <= y <= d[3] for d in self.dark):
            return None
        best = None
        for c in self.cams:
            p = c.project([[x, y, z]])[0]
            v = OB.tag_px(lambda P, c=c: c.project(P), x, y, z, s)
            m_ = OB.PIC_MARGIN_TAGS * (v or 0)
            if v is None or not (m_ <= p[0] < c.W - m_ and m_ <= p[1] < c.H - m_):
                continue
            best = v if best is None else max(best, v)
        return best

    def flat(self, x, y, h):
        return x, y


def dry_bind(args, out):
    """(11-05h, simulator) The real binding (wiggle_bind.bind_for_run: robots found however many, firmware, compass
    check after a restart, safety layers read back, the wiggle, the heading nudge) against fleet_sim's robots and the
    point-fix camera (calib_run.SimEye), with the guesses broken as asked (--sim-tag-yaw: tags turned on their robots;
    --sim-asleep: robots woken from sleep restart). The mission's own simulator then starts the robots where they
    ended."""
    import argparse as _ap
    import calib_run as CRN
    import wiggle_bind as WB
    names = [r.strip() for r in args.robots.split(",") if r.strip()]
    homes = dry_homes(args, names)
    ids = dict(args.tags_.get("ids") or {})
    folder = os.path.join(out, "bind")
    os.makedirs(folder, exist_ok=True)
    yaw = dict(kv.split("=") for kv in (args.sim_tag_yaw.split(",") if args.sim_tag_yaw else []))
    os.environ["MIRLT_SIM_TAG_YAW"] = ",".join("%d=%s" % (ids[n], v) for n, v in yaw.items() if n in ids)
    CRN.loopback_up()
    S = CRN.Site(args.site_)
    sims = ["%s:%d:%s" % (n, ids[n], homes[n]) for n in names]
    sa = _ap.Namespace(sim_name=names[0], sim_also=sims[1:], seed=args.seed, sim_max_s=900, sim_start=homes[names[0]],
                       sim_true="1,1,0", sim_true_also="", sim_sleep="0.2:0.05", sim_fw=[])
    sp, ips = CRN.start_sim(sa, S, folder)
    link = CRN.Link(folder)
    eye = CRN.SimEye(folder, CRN.FIX_PORT + 100, _BindCam(args), {n: ids[n] for n in names}, seed=args.seed)
    try:
        time.sleep(1.0)
        rec = WB.bind_for_run(args.site_, ["auto"] if args.auto_robots else names, link, eye, ips=ips, wake_min=2.0,
                              dry=True, folder=folder, log=say)
    except (Exception, SystemExit) as e:     # noqa: BLE001
        say("NOT STARTED: the wiggle-first binding (simulator) did not finish: %s" % e)
        return 5
    finally:
        os.environ.pop("MIRLT_SIM_TAG_YAW", None)
        eye.close()
        link.close()
        sp.terminate()
        try:
            sp.wait(timeout=10)
        except subprocess.TimeoutExpired:
            sp.kill()
    args.tags_ = WB.tags_with_binding(args.tags_, rec)
    args.aliases_ = {}
    take_binding(args, rec)
    # the simulated robots are physical things: the slot the binding gave each one carries ITS spot and ITS turned tag
    # into the mission's picture simulator (slots may come out swapped: the robot standing nearer Alpha's home is Alpha)
    phys = {r_["slot"]: r_["name_before"] for r_ in rec.get("robots") or []}
    args.sim_homes_ = {sl: homes.get(p_) for sl, p_ in phys.items()}
    args.sim_yaw_slot_ = {sl: float(yaw[p_]) for sl, p_ in phys.items() if p_ in yaw}
    say("TAGS from the wiggle (simulator): %s" % ", ".join("%s = tag %d" % kv for kv in sorted(rec["slots"].items())))
    return 0


PRACTICE_WIFI = ("AL-1DC0", "AL-2B7F", "AL-E7DC", "AL-EB54", "AL-3F2A", "AL-6C11")   # (e11) practice WiFi names


def practice_aliases(slots):
    """(e11) {WiFi name: slot} for the real path's practice: the first robot answers as AL-1DC0 (the school's)."""
    return {PRACTICE_WIFI[k % len(PRACTICE_WIFI)] + ("" if k < len(PRACTICE_WIFI) else "-%d" % k): n
            for k, n in enumerate(slots)}


def roam_motion_of(args):
    """(e11, 9 Nov school: the report said curves when straight lines were wanted) The free roam's driving: the motion
    asked for (--motion, the page's Advanced > Free roam driving), else curves (free roam's default)."""
    return getattr(args, "motion_given_", None) or "curves"


def run_readmap(args, out):
    """(11-05h) Where the cameras read roof tags (observe.ReadMap): seeded from the run's camera models (the setup's
    tagcal on the robots; the simulator's cameras in a dry run), carried over from the site's readmap.json (earlier
    runs, half weight), saved in the session. None when no camera model is known (everything taken as readable)."""
    import observe as OB
    site = args.site_ or {}
    tb = site.get("table") or {}
    L, W = float(tb.get("length_cm") or 396.2), float(tb.get("width_cm") or 167.6)
    rm = OB.ReadMap(L, W)
    tags = getattr(args, "tags_", None) or site.get("tags") or {}
    z, tcm = float(tags.get("height_cm") or 6.0), float(tags.get("tag_cm") or 8.0)
    cams = []
    try:
        if args.dry:
            import tag_cam as TCAM
            cfg = TCAM.default_config(table=(L, W), size=tuple(int(v) for v in args.cam_size.split("x")),
                                      seed=args.seed + 4, refs=uses_refs(args))
            for c in TCAM.cams_from_config(cfg):
                if c.name == "side" and args.no_side:
                    continue
                if getattr(args, "practice_real", False):
                    # (e11) the real path: the simulated camera as a real CameraModel (recognition/cameras.py, what
                    # e02_camera.load_camera gives on the Pi), projected one point at a time as below
                    m = c.true_model()
                    cams.append((lambda P, m=m: [tuple(m.project(tuple(q))) for q in P], tuple(m.image_size)))
                    continue
                cams.append((lambda P, c=c: c.project(P), (c.W, c.H)))
        else:
            import e02_camera as CAM
            tc = os.path.join(site.get("conf_dir") or os.path.expanduser("~/.config/mirlt"), "tagcal")
            for nm in ("overhead", "side"):
                pth = os.path.join(tc, nm + ".json")
                if nm == "side" and args.no_side:
                    continue
                if os.path.exists(pth):
                    m = CAM.load_camera(pth)
                    cams.append((lambda P, m=m: [tuple(m.project(tuple(q))) for q in P], tuple(m.image_size)))   # (9 Nov, school)
                    # e02_camera's project takes ONE point (x, y, z) -> (u, v); the seed passes a list: one call per point
                    # (a list of points crashed the first school free roam: IndexError in observe.seed_from_cameras)
    except Exception as e:      # noqa: BLE001
        say("(readability: no camera model (%s): every point taken as readable)" % e)
        return None
    if not cams:
        return None
    OB.seed_from_cameras(rm, cams, tcm, z)
    cd = site.get("conf_dir")
    if cd and not args.dry and os.path.exists(os.path.join(cd, "readmap.json")):
        try:
            with open(os.path.join(cd, "readmap.json")) as f:
                if rm.merge_history(OB.ReadMap.from_json(json.load(f))):
                    say("READABILITY: earlier runs' tag reads carried in (readmap.json, half weight)")
        except (OSError, ValueError, KeyError):
            pass
    if out:
        rm.save(os.path.join(out, "readmap_start.json"))
    return rm


def learn_readmap(args, out, sess):
    """(e12) The run's readability map (sess/readmap.json, else the start map rebuilt) with this run's camera_track.csv
    fixes and stops.csv stops added (observe.learn_from_logs), saved back into the session (the site keeps it below)."""
    import observe as OB
    p = os.path.join(sess, "readmap.json")
    rm = None
    if os.path.exists(p):
        with open(p) as f:
            rm = OB.ReadMap.from_json(json.load(f))
    if rm is None:
        rm = run_readmap(args, None)
    if rm is None:
        return
    nf, ns = OB.learn_from_logs(rm, out)
    rm.save(p)
    bl = OB.blind_cells(rm)
    say("READABILITY learned from this run: %d camera fixes, %d stops; %d cm2 readable, %d spot%s still blind%s" % (
        nf, ns, rm.area_cm2(), len(bl), "" if len(bl) == 1 else "s", "" if args.dry else " (kept for the next run)"))


def resolve_experiment(args):
    """--experiment / --swap-halves (2026-11-03): args.experiment_, args.swap_, args.swap_words_. The shakedown is the
    no-camera run unless a tracking was chosen."""
    args.experiment_, args.swap_, args.swap_words_ = None, False, ""
    if not args.experiment:
        return
    if not typed_site(args):
        raise SystemExit("--experiment needs a site with typed production points (school-conference). Nothing moved.")
    import experiments as EX
    args.experiment_ = args.experiment
    if args.experiment == "shakedown":
        args.swap_, args.swap_words_ = False, "shakedown: halves as usual"
        if args.tracking == "auto":
            args.tracking_ = "none"
            say("SHAKEDOWN: tracking none (odometry only), the site's default orders")
    else:
        args.swap_, args.swap_words_ = EX.swap_for(args.site_, args.swap_halves)
    say("EXPERIMENT %s: %s; %s" % (args.experiment, EX.KINDS[args.experiment]["name"], args.swap_words_))


def run(args):
    gap_args(args)
    resolve_site(args)
    resolve_experiment(args)
    tag = "mission" + ("-dry" if args.dry else "")
    out = os.path.abspath(args.session or os.path.join(REPO, "logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") +
                                                       "-" + tag))
    os.makedirs(out, exist_ok=True)
    frames_dir = None if args.no_frames else DG.frames_dir_for(out)[0]
    fake = None
    if args.dry and args.sim_free:
        fake = dict((k, float(v)) for k, v in (kv.split("=") for kv in args.sim_free.split(",")))
    guard = DG.Guard(card=out, frames_dir=frames_dir, fake=fake)
    say(guard.words(frames_on=not args.no_frames))
    st0 = guard.state()
    if st0["stop"]:
        say("NOT STARTED: %s. Nothing moved. Free space first: ./run-fleet-pi.sh space, then "
            "./run-fleet-pi.sh clean-uploaded" % st0["why"])
        return 3
    if (args.site_ or {}).get("attended_only") and not args.dry:
        say("ATTENDED ONLY (Jay-Sun 5 Nov): at school the robots drive only while Jay-Sun is in the room")
    for lab in (args.site_ or {}).get("missing_stickers") or []:
        say("NOTE: sticker %s is missing on the table: its point is still visited by its typed coordinates (nothing "
            "here needs to see the sticker)" % lab)
    if args.tracking_ == "tags" and not args.plan_only:
        code_ = wiggle_first(args, out)
        if code_:
            return code_
    if args.tracking_ == "tags":
        args.tags_json = os.path.join(out, "tags.json")
        with open(args.tags_json, "w") as f:
            json.dump(args.tags_, f, indent=1)
    args.session_id = args.session_id or None
    with open(os.path.join(out, "mission_cmd.json"), "w") as f:
        json.dump({"argv": sys.argv, "args": {k: v for k, v in vars(args).items() if k != "site_"}}, f, indent=1,
                  default=str)
    if args.tracking_ == "tags" and not args.mission and not args.plan_only:
        code_ = feel_first(args, out)
        if code_ or args.feel_only:
            return code_
    if args.mission:
        # a mission made elsewhere (Mission Control's warm-up): its map and plan as they are
        mp, pp = os.path.abspath(args.mission), os.path.abspath(args.plan)
    else:
        mm, plan, mp, pp = build(args, out)
        bad_ = experiment_unseen(args, out)
        if bad_:
            say("NOT STARTED: %s. Nothing moved." % bad_)
            return 6
    if args.plan_only:
        return 0
    sess = os.path.join(out, "run")
    os.makedirs(sess, exist_ok=True)
    rp = [os.path.join(HERE, "run_plan.py"), "--map", mp, "--plan", pp, "--session", sess, "--fixes", "udp",
          "--poses-to", "127.0.0.1:5011", "--replan-from-camera", "--timeout-s", str(args.timeout_s),
          "--border-cm", str(args.border_cm), "--min-gap-cm", str(args.min_gap_cm),
          "--place-tol-cm", "4"]
    if args.tracking_ == "none":
        # no camera at all (2026-11-03): run_plan's own "none" source (each robot gets its start pose once, then wheels
        # and gyro); nothing waits for a camera (no facing check, no turn pause, no re-plan from the camera)
        rp[rp.index("--fixes") + 1] = "none"
        rp.remove("--replan-from-camera")
        args.no_facing_check = True
        args.no_turn_pause = True
        say("TRACKING: none (odometry only). Each robot must stand on its home sticker, facing +y (away from the "
            "A3-A4 edge), before Run: its start pose is taken as exact. The +- at each point grows 0.07 cm per cm "
            "driven (29 Oct logs, measured to 40 cm only); no camera checks anything during the run.")
    if not args.no_turn_pause:
        rp += ["--turn-pause"]         # after each turn, stand until the cameras agree (2026-10-29)
    # the keep-going policy (2026-11-03): the site's on_problem (e08: keep_going at home too)
    rp += ["--on-problem", args.on_problem_, "--lost-robot-points", args.lost_robot_points_]
    rp += ["--edge-tracking", args.tracking_ if args.tracking_ in ("lights", "tags", "none") else "lights"]   # (e08)
    if os.environ.get("MIRLT_ATTENDED") == "1":
        rp += ["--attended"]                 # (e08, A2) Mission Control's Start screen: a person is there
    say("on a problem: %s" % ("KEEP GOING (said once, logged, worked round; a lost robot's points %s)" % (
        "go to the robots still running" if args.lost_robot_points_ == "reassign" else "are listed as not done")
        if args.on_problem_ == "keep_going" else "stop the run (as before)"))
    if args.tracking_ == "tags":
        # confidence at every point: a bumped camera (the bridge's events) counts in the +-
        rp += ["--bridge-events", os.path.join(out, "bridge", "events.jsonl")]
        if os.path.exists(os.path.join(out, "readmap_start.json")):
            rp += ["--readmap", os.path.join(out, "readmap_start.json")]     # (11-05h)
    if args.no_mirror:
        rp += ["--no-mirror"]          # mirrored legs at their planned speed, no waits (the points still measured)
    if args.below_net_doe:
        rp += ["--below-net-doe"]
    for c in guard_cmds(args):
        rp += ["--send", c]          # (11-05h) run_plan reads each one back before anything moves
    for ip in args.robot:
        rp += ["--robot", ip]
    if getattr(args, "practice_real", False) and not getattr(args, "aliases_", None):
        # (e11, 9 Nov) the real path's practice: the simulated robots answer with WiFi names (AL-xxxx) and the run calls
        # them by their slots through aliases, as at school (tags.rename_to_slot false)
        args.aliases_ = practice_aliases([r_["name"] for r_ in FM.load(mp)["robots"]])
        args.sim_arg = list(args.sim_arg) + ["--wifi-name=%s=%s" % (v_, k_) for k_, v_ in args.aliases_.items()]
        say("PRACTICE ON THE REAL PATH: the simulated robots answer as %s" % ", ".join(
            "%s (the run's %s)" % kv for kv in sorted(args.aliases_.items())))
    for k_, v_ in sorted((getattr(args, "aliases_", None) or {}).items()):
        rp += ["--alias", "%s=%s" % (k_, v_)]      # (5 Nov) names in the run only (tags.rename_to_slot false)
    if float(getattr(args, "roam_min", 0) or 0) > 0 or getattr(args, "visit_order", ""):
        rp += ["--dedupe-points", "off"]           # (e07) a free roam visits its points again and again (e11: and a
        #                                            chosen order may repeat a point)
    roam_fr = float(getattr(args, "roam_min", 0) or 0) > 0 and args.tracking_ == "tags" and \
        roam_controller(args) == "freeroam"
    if roam_fr:
        # (e10, C3: one controller at every site) the school's free roam on the free-roam controller (freeroam_core:
        # reflex, reservations, curves, the filter and the tracking layer with its mid-run identity wiggle, the start's
        # proof drive, the proven workspace), fed by this run's tag bridge (tags + lock-on). The planner's roam over the
        # points stays behind --roam-controller planner (the switch)
        fl = FM.load(mp)["floor"]
        # (e11) the run's robots: the map's (the slots the wiggle bound), else the tag ids' names
        names_ = [r_["name"] for r_ in FM.load(mp).get("robots") or []] or \
            sorted((args.tags_ or {}).get("ids", {}).keys())
        roam_motion = roam_motion_of(args)
        rp = [os.path.join(HERE, "freeroam.py"), "run", "--external-bridge", "--tracking", "tags",
              "--robots", ",".join(names_), "--board", "%g,%g,%g,%g" % (fl["x_min"], fl["y_min"], fl["x_max"],
                                                                         fl["y_max"]),
              "--hours", "%.3f" % (float(args.roam_min) / 60.0), "--site", (args.site_ or {}).get("name") or "school",
              "--dir", sess, "--motion", roam_motion]   # (Nick 9 Nov) curves (default) or stopturn, from the Start screen's Advanced
        for k_, v_ in sorted((getattr(args, "aliases_", None) or {}).items()):
            rp += ["--alias", "%s=%s" % (k_, v_)]      # (e11, 9 Nov school: "No robot answered") AL-1DC0 is Alpha
        say("FREE ROAM DRIVING: %s" % ("straight lines (stopturn: stop and turn on the spot)" if roam_motion ==
                                       "stopturn" else "curves"))
        for ip in args.robot:
            rp += ["--robot", ip]
        if os.environ.get("MIRLT_ATTENDED") == "1":
            rp += ["--attended"]
        if args.dry:
            # practice: the simulated robots run_plan would start (fleet_sim on 127.0.0.2..), started by freeroam.py;
            # their pictures come from the tag cameras' simulator as for the other runs
            mnames = [r_["name"] for r_ in FM.load(mp)["robots"]]
            if getattr(args, "practice_real", False):
                rp += ["--practice-real"]             # (e11) not marked dry inside freeroam.py
            else:
                rp[rp.index("--robots") + 1] = ",".join(mnames)
            for k_ in range(len(mnames)):
                rp += ["--robot", "127.0.0.%d" % (2 + k_)]
            # rp_shim's simulator (the school's conditions: drops, a robot restarting), as for the other dry runs
            simc = [PY, os.path.join(HERE, "sweeps", "sim_shim.py"), "--map", mp, "--plan", pp, "--out",
                    os.path.join(sess, "sim"), "--seed", str(args.seed), "--quiet", "--fix-hz", "0", "--persist"] + \
                [a_.partition("=")[2] if a_.startswith("--sim-arg=") else a_ for a_ in args.sim_arg]
            rp += ["--sim-cmd", json.dumps(simc)]
        say("FREE ROAM on the free-roam controller (e10): the same controller as at home, fed by the tag bridge")
        fr_len = len(rp)
    for n_, spec_ in sorted((stop_lead_specs(args, out) if not getattr(args, "no_stop_lead", False) else {}).items()):
        rp += ["--stop-lead", "%s=%s" % (n_, spec_)]     # (e07) each robot's stop lead from its last calibration
    rp += list(args.rp_arg)
    cond = {}
    tag_setup_thread = None
    if args.dry and args.tracking_ == "tags":
        # the AprilTag robots in the simulator (2026-11-02): two tag cameras (tools/sim/tag_cam.py) over the table, the
        # setup (corner and sticker clicks from the truth + noise, the tags read, site_fit) run as Nick would run it,
        # and the bridge and the facing check on the FITTED cameras, not the true ones
        import tag_cam as TCAM
        import tag_setup as TSU
        mmap = FM.load(mp)
        fl = mmap["floor"]
        L_, W_ = fl["x_max"] - fl["x_min"], fl["y_max"] - fl["y_min"]
        if args.sim_true_floor:
            # the real table where the map's outline is wrong (feel out the table, 2026-11-02): the tag cameras draw it
            # and the edge sensors meet it; the planner keeps the map's outline
            tf_ = [float(v) for v in args.sim_true_floor.split(",")]
            L_, W_ = tf_[2] - tf_[0], tf_[3] - tf_[1]
            rp += ["--sim-arg=--true-floor=%s" % args.sim_true_floor]
        # the conference table's drop (2026-11-02): past an open edge the floor is far below, not home's plywood step
        drop_ = (((args.site_ or {}).get("table") or {}).get("drop_cm") or DROP_CM)
        if not any(str(a).startswith("--drop-cm") for a in args.sim_arg):
            rp += ["--sim-arg=--drop-cm=%g" % float(drop_)]
        w_, h_ = (int(v) for v in args.cam_size.split("x"))
        tcfg = TCAM.default_config(table=(L_, W_), size=(w_, h_), tags=args.tags_, seed=args.seed + 4,
                                   refs=uses_refs(args), hide_refs=[x for x in args.hide_ref.split(",") if x.strip()])
        for b_ in args.bump:
            # a camera knocked mid-run (2026-11-03): CAM@T:DEG[:AXIS], T s after the plan starts
            c_, _, rest = b_.partition("@")
            t_, _, rest = rest.partition(":")
            deg_, _, ax_ = rest.partition(":")
            tcfg.setdefault("bumps", []).append({"cam": c_, "t": float(t_), "deg": float(deg_ or 1.0),
                                                 "axis": ax_ or "x"})
        if mmap.get("spots"):
            tcfg["stickers"] = {k: v[:2] for k, v in mmap["spots"].items()}
        for g in args.tag_gap:
            n_, _, rest = g.partition("@")
            t_, _, d_ = rest.partition(":")
            tcfg.setdefault("gaps", []).append({"robot": n_, "t": float(t_), "dur": float(d_ or 10)})
        for lr in args.cam_low_res:
            c_, _, wh = lr.partition("=")
            tcfg.setdefault("low_res", {})[c_] = [int(v) for v in wh.split("x")]
        if getattr(args, "sim_yaw_slot_", None) is not None:
            tcfg["tag_yaw"] = dict(args.sim_yaw_slot_)      # (11-05h) by the binding's slots (the physical robots)
        elif args.sim_tag_yaw:
            # (11-05h) tags turned on their robots: drawn turned (the tag reads DEG less than the nose)
            tcfg["tag_yaw"] = {k: float(v) for k, v in (kv.split("=") for kv in args.sim_tag_yaw.split(","))}
        if args.sim_dark:
            tcfg["dark"] = [[float(v) for v in d.split(",")] for d in args.sim_dark]
        if args.sim_clock_skew:
            tcfg["clock_skew_s"] = float(args.sim_clock_skew)     # (11-05h) the pictures stamped on a skewed clock
        tcp = os.path.join(out, "tag_cams.json")
        with open(tcp, "w") as f:
            json.dump(tcfg, f, indent=1)
        tagcal = os.path.join(out, "tagcal")
        cal = os.path.join(tagcal, "overhead.json")
        scal = os.path.join(tagcal, "side.json") if not args.no_side else ""
        stream, side_stream = "127.0.0.1:8765", "127.0.0.1:8766"
        rp += ["--dry-run", "--yes", "--seed", str(args.seed), "--sim-arg=--fix-hz=0", "--sim-arg=--tag-cams=%s" % tcp]
        import threading as _th

        def _setup():
            try:
                TSU.sim_setup(os.path.join(sess, "sim"), {"overhead": stream, "side": side_stream}, tcfg, tagcal,
                              typed=not args.no_typed, say=say, corner_clicks=corner_clicks(args))
            except Exception as e:     # noqa: BLE001
                say("SETUP FAILED: %s" % e)
        tag_setup_thread = _th.Thread(target=_setup, daemon=True)
    if args.dry:
        if args.tracking_ == "none":
            cal = scal = ""
            stream, side_stream = args.stream, args.side_stream
            rp += ["--dry-run", "--yes", "--seed", str(args.seed)]
        elif args.tracking_ != "tags":
            cal = os.path.join(sess, "sim", "table_camera.json")
            scal = os.path.join(sess, "sim", "side_camera.json") if not args.no_side else ""
            stream, side_stream = "127.0.0.1:8765", "127.0.0.1:8766"
            rp += ["--dry-run", "--yes", "--seed", str(args.seed), "--sim-arg=--fix-hz=0",
                   "--sim-arg=--serve-live=8765", "--sim-arg=--live-calib-out=%s" % cal]
            if scal:
                rp += ["--sim-arg=--live-side=8766", "--sim-arg=--live-side-calib-out=%s" % scal]
            if args.washout:
                rp += ["--sim-arg=--live-washout"]
        if args.drop_pct:
            cond["drop_pct"] = args.drop_pct
        if args.dead:
            n, _, t = args.dead.partition("@")
            cond["robot_out"] = {"name": n, "t": float(t or 60), "mode": "dead"}
        if args.sim_cond:
            with open(args.sim_cond) as f:
                cond.update(json.load(f))
        rp += ["--sim-arg=%s" % a for a in args.sim_arg]
        if cond:
            cp = os.path.join(out, "sim_cond.json")
            with open(cp, "w") as f:
                json.dump(cond, f)
            rp += ["--sim-arg=--sweep-cond=%s" % cp]
        launcher = [PY, os.path.join(HERE, "sweeps", "rp_shim.py")]     # sim_shim's conditions (drops, a dead robot)
    elif args.tracking_ == "none":
        cal = scal = ""                        # no camera models needed (odometry only)
        stream, side_stream = args.stream, args.side_stream
        launcher = [PY]
    elif args.tracking_ == "tags":
        # the AprilTag site on the Pi: the camera models the tag setup saved (Mission Control's Setup, Measure)
        tc = os.path.join((args.site_ or {}).get("conf_dir") or os.path.expanduser("~/.config/mirlt"), "tagcal")
        cal = args.calib or os.path.join(tc, "overhead.json")
        scal = "" if args.no_side else (args.side_calib or os.path.join(tc, "side.json"))
        if scal and not os.path.exists(scal):
            say("no model for camera 2 (%s): camera 1 alone" % scal)
            scal = ""
        if not os.path.exists(cal):
            raise SystemExit("No camera model at %s: run the setup first (Mission Control, Setup, Measure). Nothing "
                             "moved." % cal)
        stream, side_stream = args.stream, args.side_stream
        launcher = [PY]
    else:
        import setup_conf as SCF
        # setup_conf returns (path, words): take the path, print the words (2026-10-29 20:46 on the Pi: the tuple
        # went to the camera bridge and the robots' run never started)
        if args.calib:
            cal = args.calib
        else:
            cal, words = SCF.overhead_calib(relative=False)
            say(words)
            if not cal:
                raise SystemExit("No lights floor calibration for this site (above). Nothing moved.")
        if args.no_side:
            scal = ""
        elif args.side_calib:
            scal = args.side_calib
        else:
            scal, words = SCF.side_calib()
            scal = scal or ""
            say(words)
        stream, side_stream = args.stream, args.side_stream
        launcher = [PY]
    if args.tracking_ == "tags" and cal:
        # confidence at every point (2026-11-03): each camera's fit error from the setup's site_fit.json
        rp += ["--cam-fit", os.path.join(os.path.dirname(os.path.abspath(cal)), "site_fit.json")]
    # the facing check (2026-10-29): run_plan measures which way each robot faces from its lights before anything
    # moves, through both cameras (facing.py)
    if not args.no_facing_check:
        rp += ["--facing-check", "--facing-calib", cal, "--facing-stream", stream,
               "--facing-tol-deg", str(args.facing_tol_deg)]
        if args.tracking_ == "tags":
            rp += ["--facing-tags", args.tags_json]
        if not args.no_straighten:
            # start straightening (Nick, 2026-10-29 15:41): a robot off its start pose is turned or driven to its spot,
            # the camera watching, and checked again, instead of a stop (straighten.py)
            rp += ["--straighten"]
        if scal:
            rp += ["--facing-side-calib", scal, "--facing-side-stream", side_stream]
    # the record: cpu.csv, frames (tracked pictures and clips)
    cpu = frames = tail = None
    try:
        import e02_cpulog as CL
        cpu = CL.CpuLog(out, procs={"mission": os.getpid()},
                        find={"recorder": "camrec.py", "bridge": "cam_bridge.py", "tag_bridge": "tag_bridge.py",
                              "run_plan": "run_plan.py"})
        cpu.start()
    except Exception as e:     # noqa: BLE001
        say("(cpu log not started: %s)" % e)
    br, brf = (None, None) if args.tracking_ == "none" else start_bridge(args, out, mp, cal, scal, stream, side_stream)
    if args.tracking_ == "none":
        say("no camera bridge (tracking none)")
    elif args.tracking_ == "tags":
        say("tag bridge starting (each robot found by its roof tag, wherever it is)")
    else:
        say("camera bridge starting (it learns the empty floor round the robots: keep hands out of the picture)")
    if args.tracking_ == "none":
        pass                                   # no camera pictures to save
    elif not args.no_frames and not st0["frames_ok"]:
        say("FRAMES NOT SAVED: %s" % st0["why_frames"])
    elif not args.no_frames:
        try:
            import doe_loop as DL
            if (args.site_ or {}).get("frames_part_s"):
                # (2026-11-03) the tracking-rate pictures are held in memory until each .avi part is written: 5 min
                # parts took the simulator's mission_run to 1.2 GB, 3 GB while writing (the 450 s experiments). A site
                # may cut shorter parts (school: 60 s); home keeps doe_loop's 300 s
                DL.PART_S = float(args.site_["frames_part_s"])
                say("frames: .avi parts of %.0f s (the site's frames_part_s)" % DL.PART_S)
            streams = [("overhead", stream, 12.0)] + ([("side", side_stream, 10.0)] if scal else [])
            if args.dry:
                time.sleep(0.1)
            frames = DL.Frames(out, streams, frames_dir=frames_dir, guard=guard)
            tail = DL.Tail(os.path.join(sess, "coordinator.jsonl"), frames)
            tail.start()
            frames.active(True, "mission")
        except Exception as e:     # noqa: BLE001
            say("(frame saving not started: %s)" % e)
    time.sleep(BRIDGE_START_S)
    say("the robots start now. To stop them at any moment: Ctrl+C")
    code = 1
    # (rp_shim.py stands in for run_plan.py on a dry run and takes run_plan's arguments)
    if roam_fr:
        launcher = [PY, rp[0]]             # (e10) freeroam.py takes its own arguments, dry or not
        # run_plan's arguments added after the hand-over: the simulator's go into its command, --stop-lead is kept,
        # the rest (facing check, --dry-run ...) are run_plan's own
        extra, keep, k_ = rp[fr_len:], [], 0
        while k_ < len(extra):
            if extra[k_] == "--stop-lead" and k_ + 1 < len(extra):
                keep += extra[k_:k_ + 2]
                k_ += 2
                continue
            k_ += 1
        sim_extra = [x.partition("=")[2] for x in extra if str(x).startswith("--sim-arg=")]
        rp = rp[:fr_len] + keep
        if "--sim-cmd" in rp and sim_extra:
            i_ = rp.index("--sim-cmd") + 1
            rp[i_] = json.dumps(json.loads(rp[i_]) + sim_extra)
    try:
        child = subprocess.Popen(launcher + (rp[1:] if (args.dry or roam_fr) else rp), cwd=REPO)   # (9 Nov, school)
        # the free-roam launcher already holds freeroam.py (rp[0]); passing rp whole on a real run gave freeroam.py its own
        # path as its first argument ("invalid choice"): the first real school free roam ended before anything moved
    except BaseException:
        if br is not None:
            br.send_signal(signal.SIGINT)    # never leave the bridge running on its own (2026-11-02: one did)
        raise
    if tag_setup_thread is not None:
        tag_setup_thread.start()
    sync_mark(out, "start", args)
    exp_meta = None
    try:
        with open(os.path.join(out, "program.json")) as f:
            exp_meta = ((json.load(f).get("production") or {}).get("experiment"))
    except (OSError, ValueError):
        pass
    if exp_meta and not args.dry and args.site_:
        # the run counter (swap halves every other run): one more production run with an experiment on the robots
        import experiments as EX
        d_ = EX.counter_add(args.site_, {"experiment": exp_meta.get("id"), "swap_halves": exp_meta.get("swap_halves"),
                                         "session": os.path.basename(out), "session_id": args.session_id})
        say("run counter at this site: %d (next run's halves: %s)" % (d_["runs"], "swapped" if d_["runs"] % 2 else
                                                                        "as usual"))
    import threading
    disk_said, disk_stop = [], threading.Event()
    threading.Thread(target=space_watch, args=(guard, child, disk_said, disk_stop), daemon=True).start()
    moved_said = []
    if args.tracking_ == "tags":
        threading.Thread(target=bridge_watch, args=(os.path.join(out, "bridge", "events.jsonl"), disk_stop,
                                                    moved_said), daemon=True).start()
    try:
        code = child.wait()
    except KeyboardInterrupt:
        say("stopping every robot")
        child.send_signal(signal.SIGINT)
        try:
            code = child.wait(timeout=15)
        except subprocess.TimeoutExpired:
            child.kill()
    finally:
        disk_stop.set()
        for p, f in ((br, brf),) if br is not None else ():
            try:
                p.send_signal(signal.SIGINT)
                p.wait(timeout=10)
            except (subprocess.TimeoutExpired, OSError):
                p.kill()
            f.close()
        if tail is not None:
            tail.stop()
        if frames is not None:
            frames.active(False)
            frames.close()
        if cpu is not None:
            try:
                cpu.stop()
            except Exception:     # noqa: BLE001
                pass
    sync_mark(out, "end", args)
    summ = summarise(out, sess, frames, cond)
    try:
        # the experiment (2026-11-03): id, orders, swap, seed, pinned speeds; the sync wiggle's times per robot
        import experiments as EX
        if exp_meta:
            summ["experiment"] = dict(exp_meta, swap_words=getattr(args, "swap_words_", ""),
                                      tracking=args.tracking_)
        if getattr(args, "speed_", None):
            summ["speed_pinned"] = args.speed_
        wg = EX.wiggle_events(os.path.join(sess, "coordinator.jsonl"))
        if wg:
            summ["sync_wiggle"] = wg
            for n_, w_ in sorted(wg.items()):
                say("SYNC WIGGLE %s: %.3f -> %.3f (Pi clock, %.1f s)" % (n_, w_["start"], w_["end"],
                                                                         w_["end"] - w_["start"]))
        with open(os.path.join(sess, "experiment.json"), "w") as f:
            json.dump({"experiment": exp_meta, "speed_pinned": getattr(args, "speed_", None), "sync_wiggle": wg,
                       "tracking": args.tracking_}, f, indent=1)
        if not args.no_run_exports:
            import run_exports as RX
            summ["exports"] = RX.export_all(out, say=say)
    except Exception as e:     # noqa: BLE001  (an export never stops the run's end)
        say("(experiment record / exports not made: %s)" % e)
    if (args.site_ or {}).get("conf_dir") and getattr(args, "tracking_", "") == "tags":
        # (e12, 9 Nov school: every stop of 7 tours had camera fixes where the seed said "unreadable") the run's own
        # camera fixes and stops (camera_track.csv, stops.csv: written by the exports just above) go into the map
        # before it is kept, so the evidence overturns the cautious seed
        try:
            learn_readmap(args, out, sess)
        except Exception as e:  # noqa: BLE001  (never stops the end of a run)
            say("(readability not learned from this run's logs: %s)" % e)
    if not args.dry and (args.site_ or {}).get("conf_dir") and os.path.exists(os.path.join(sess, "readmap.json")):
        # (11-05h) the site keeps where tags read (merged at half weight into the next run's start map)
        try:
            import shutil
            shutil.copy(os.path.join(sess, "readmap.json"), os.path.join(args.site_["conf_dir"], "readmap.json"))
        except OSError:
            pass
    with open(os.path.join(out, "mission_summary.json"), "w") as f:
        json.dump(summ, f, indent=1, default=str)
    if args.tracking_ == "tags":
        summ["camera_moved"] = [{k: e.get(k) for k in ("cam", "t", "median_px", "about_cm")} for e in moved_said]
    try:
        import sync_tools as ST
        p_, counts = ST.export_timeline(out)
        summ["pose_timeline"] = {"file": p_, "rows": counts}
        say("pose timeline: %s (%s)" % (p_, ", ".join("%s %d" % kv for kv in counts.items())))
        if args.tracking_ == "tags" and getattr(args, "refit_bg", False):
            import site_fit as SF
            ob = os.path.join(out, "tagcal", "obs.json") if args.dry else os.path.join(
                (args.site_ or {}).get("conf_dir") or "", "tagcal", "obs.json")
            to = os.path.join(out, "bridge", "tag_obs.csv")
            if os.path.exists(ob) and os.path.exists(to):
                st = SF.start_refit_bg(ob, to, os.path.join(out, "tagcal_refit"), python=PY, cwd=REPO)
                summ["refit"] = {"background": True, "folder": os.path.join(out, "tagcal_refit"), "state": st["state"]}
                say("refit started in the background (its own process, a few minutes; the run is over and nothing waits "
                    "for it): %s" % os.path.join(out, "tagcal_refit"))
        if args.tracking_ == "tags" and args.refit and not getattr(args, "refit_bg", False):
            import site_fit as SF
            ob = os.path.join(out, "tagcal", "obs.json") if args.dry else os.path.join(
                (args.site_ or {}).get("conf_dir") or "", "tagcal", "obs.json")
            to = os.path.join(out, "bridge", "tag_obs.csv")
            if os.path.exists(ob) and os.path.exists(to):
                rf = SF.refit(ob, to, os.path.join(out, "tagcal_refit"))
                summ["refit"] = {"size_cm": rf["size_cm"], "rms_px": rf["rms_px"], "words": rf["words"]}
                say("refit from this run's tag reads: table %.1f x %.1f cm (models in %s)" % (
                    rf["size_cm"][0], rf["size_cm"][1], os.path.join(out, "tagcal_refit")))
        with open(os.path.join(out, "mission_summary.json"), "w") as f:
            json.dump(summ, f, indent=1, default=str)
    except Exception as e:     # noqa: BLE001
        say("(pose timeline / refit not made: %s)" % e)
    if disk_said:
        summ["ended_by"] = "the card nearly full: " + disk_said[0]
        try:
            with open(os.path.join(out, "mission_summary.json"), "w") as f:
                json.dump(summ, f, indent=1, default=str)
        except OSError:
            pass
    if args.upload and not args.dry:
        up = [PY, os.path.join(TOOLS, "pi", "upload_session.py"), "--recordings", args.recordings, "--session", out,
              "--tag", "mission-" + os.path.basename(out), "--unique-tag", "--logs-only"]
        if frames_dir and os.path.isdir(frames_dir):
            # the frames as their own asset (rec-frames-small.zip), as the DOE loop's; not inside the logs zip
            up += ["--recording-dir", frames_dir, "--exclude", "frames"]
        say("uploading: " + " ".join(up[1:]))
        subprocess.call(up, cwd=REPO)
    for q in summ.get("points") or []:
        say("POINT %s%s" % (q.get("words"), "" if q.get("truth_err_cm") is None else
                            "  [simulator truth: %.1f cm off, %s]" % (q["truth_err_cm"], "inside" if q.get("within")
                                                                     else "OUTSIDE its \u00b1")))
    for q in summ.get("points_skipped") or []:
        say("SKIPPED %s (%s): %s" % (q["point"], q["robot"], q["why"]))
    for q in summ.get("points_reassigned") or []:
        say("REASSIGNED %s: %s -> %s" % (q["point"], q["frm"], q["to"]))
    if summ.get("on_problem") == "keep_going":
        say("keep going: %d problem(s) worked round without stopping the run" % len(summ.get("kept_going") or []))
    say("result: %s; saved in %s" % (summ.get("result"), out))
    return code


def point_truth(points, truth_csv, sess):
    """Simulator only: each point's estimate against the simulator's true pose at the point's time (the median over
    the dwell's last second): truth_err_cm and whether it fell inside its +- (within)."""
    import csv as _csv
    rows = {}
    try:
        with open(truth_csv) as f:
            for r in _csv.DictReader(f):
                rows.setdefault(r["robot"], []).append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"])))
        with open(os.path.join(sess, "coordinator.jsonl")) as f:
            t_run = None
            for ln in f:
                e = json.loads(ln)
                if e.get("event") == "message" and str(e.get("text", "")).startswith("plan ") and "started" in e["text"]:
                    t_run = e["t"]
                    break
    except (OSError, ValueError, KeyError):
        return
    if t_run is None:
        return
    for q in points:
        if q.get("x") is None or q.get("t_run") is None or q["robot"] not in rows:
            continue
        t1 = t_run + q["t_run"]
        near = [(x, y) for t, x, y in rows[q["robot"]] if t1 - 1.0 <= t <= t1]
        if not near:
            continue
        import statistics as _st
        tx, ty = _st.median([p[0] for p in near]), _st.median([p[1] for p in near])
        q["truth_err_cm"] = round(((q["x"] - tx) ** 2 + (q["y"] - ty) ** 2) ** 0.5, 2)
        q["within"] = q["truth_err_cm"] <= (q.get("plus_minus_cm") or 0)


def summarise(out, sess, frames, cond):
    """mission_summary.json: what the run did against the plan, the simulator's truth (dry), the line check."""
    s = {"folder": out, "conditions": cond}
    try:
        with open(os.path.join(sess, "run_plan_summary.json")) as f:
            rs = json.load(f)
    except (OSError, ValueError):
        rs = {}
    s.update(result=rs.get("result"), duration_s=rs.get("duration_s"), planned_s=rs.get("planned_makespan_s"),
             replans=len([r for r in rs.get("replans", []) if r.get("source") not in ("improve",)]),
             robots_out=rs.get("robots_out"), plans_refused=rs.get("plans_refused"),
             sensor_stops=len(rs.get("sensor_stops") or []), confirm_checks=rs.get("confirm_checks"),
             battery=rs.get("battery"), start_checks=rs.get("start_checks"))
    # keep going and confidence at every point (2026-11-03)
    s.update(on_problem=rs.get("on_problem"), kept_going=rs.get("kept_going") or [], points=rs.get("points") or [],
             points_skipped=rs.get("points_skipped") or [], points_reassigned=rs.get("points_reassigned") or [])
    tru = os.path.join(sess, "sim", "truth.csv")
    if s["points"] and os.path.exists(tru):
        point_truth(s["points"], tru, sess)
    mir = rs.get("mirror") or {}
    if mir.get("on"):
        s["mirror"] = {k: mir.get(k) for k in ("lane_ends", "all_points", "holds", "retimed_legs", "target_s",
                                                "target_cm")}
        le = mir.get("lane_ends") or {}
        if le.get("time_s"):
            say("mirroring: %d lane ends, time apart max %.2f s (median %.2f), position max %.1f cm (median %.1f); "
                "%d within %.1f s and %.0f cm" % (le["time_s"]["n"], le["time_s"]["max"], le["time_s"]["median"],
                                                   le["pos_cm"]["max"], le["pos_cm"]["median"], le["within_target"],
                                                   mir.get("target_s") or 0.5, mir.get("target_cm") or 3.0))
    sim = rs.get("simulator") or {}
    if sim:
        s["sim"] = {k: sim.get(k) for k in ("min_body_gap_cm", "min_body_gap_at", "collision", "min_open_edge_cm",
                                            "min_open_edge_at", "hung")}
    try:
        import line_check as LC
        tru = os.path.join(sess, "sim", "truth.csv")
        lc = LC.check(sess, os.path.join(out, "bridge"), truth=tru if os.path.exists(tru) else None,
                      png=os.path.join(out, "line_check.png"))
        s["line_check"] = {n: dict(p95_cm=r["camera"].get("p95_cm"), within_pct=r["camera"].get("within_gate_pct"),
                                   misses=r["camera"].get("misses"), sure_share_pct=r["sure_share_pct"],
                                   camera_vs_truth=r.get("camera_vs_truth"))
                           for n, r in lc["robots"].items()}
        s["closest_by_camera"] = lc.get("closest_pair_camera")
    except Exception as e:     # noqa: BLE001
        s["line_check"] = "not made: %s" % e
    try:
        with open(os.path.join(out, "bridge", "bridge_summary.json")) as f:
            bs = json.load(f)
        s["camera"] = {"frames": bs.get("frames"), "robots": bs.get("robots"), "farm": bs.get("farm")}
    except (OSError, ValueError):
        pass
    if frames is not None:
        s["frames"] = frames.words()
    # labels the robots reached (blocks done), from the step reports
    done = {}
    try:
        with open(os.path.join(sess, "coordinator.jsonl")) as f:
            plan_steps = {}
            for ln in f:
                e = json.loads(ln)
                if e.get("event") == "labelled_step" and e.get("state") == "done":
                    lab = str(e.get("label", "")).split(" lane")[0]
                    done.setdefault(e["robot"], [])
                    if lab not in done[e["robot"]]:
                        done[e["robot"]].append(lab)
        s["reached"] = done
    except (OSError, ValueError):
        pass
    try:
        with open(os.path.join(out, "mission_summary.json"), "w") as f:
            json.dump(s, f, indent=1, default=str)
    except OSError as e:
        say("(mission_summary.json not written: %s)" % e)
    return s


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--table", default="61x122", help="the table's size in cm, e.g. 61x122 (without --map)")
    ap.add_argument("--map", help="Mission Control's workspace map (sides, zones from the warm-up)")
    ap.add_argument("--program", help="a block program (Mission Control's program.json); default: Monday's")
    ap.add_argument("--robots", default="Left,Right", help="Monday's two robots (default Left,Right); auto (11-05h): "
                    "every robot on the table, found by the wiggle (however many)")
    ap.add_argument("--split", choices=["auto", "halves"], default="auto",
                    help="(11-05h) how the points are shared: auto (the site's production.split, else over the area the "
                         "cameras read, between the robots found) or halves (the site's fixed zones, the old way)")
    ap.add_argument("--no-camera-check", action="store_true", help="(e12) do not check the camera model against the "
                    "reference tags at the start (MIRLT_CAMERA_SELFCHECK=0 too; =check: checked, never refitted)")
    ap.add_argument("--mission", help="a mission map made elsewhere (Mission Control's warm-up), with --plan")
    ap.add_argument("--plan", help="with --mission: its plan")
    ap.add_argument("--session", help="the session folder (default logs/<date-time>-mission)")
    ap.add_argument("--plan-only", action="store_true", help="build and plan, then stop (nothing moves)")
    ap.add_argument("--min-gap-cm", type=float, default=EC.MIN_GAP_CM)
    ap.add_argument("--border-cm", type=float, default=6.0)
    ap.add_argument("--timeout-s", type=float, default=600.0)
    ap.add_argument("--robot", action="append", default=[], help="a robot's IP (default: found on the WiFi)")
    ap.add_argument("--calib", help="the overhead's floor calibration (default: what the setup page saved)")
    ap.add_argument("--side-calib", help="the side camera's (default: what the setup page saved)")
    ap.add_argument("--no-side", action="store_true", help="the overhead camera alone")
    ap.add_argument("--stream", default="127.0.0.1:8000")
    ap.add_argument("--side-stream", default="127.0.0.1:8001")
    ap.add_argument("--farm", action="store_true", help="each camera tracked in its own process (trackfarm.py)")
    ap.add_argument("--rings", action="store_true", help="--farm: the recorder shares its pictures (camrec.py --shm)")
    ap.add_argument("--half", action="store_true", help="--farm: track at half size")
    ap.add_argument("--no-facing-check", action="store_true",
                    help="skip the facing check (which way each robot faces, from its lights, before anything moves)")
    ap.add_argument("--facing-tol-deg", type=float, default=15.0,
                    help="the facing check: a robot more than this off its start heading stops the start")
    ap.add_argument("--no-straighten", action="store_true",
                    help="a robot off its start pose stops the start (before 2026-10-29 15:41); default: it is turned "
                         "on the spot or driven to its spot, then checked again (run_plan --straighten)")
    ap.add_argument("--no-frames", action="store_true", help="no tracked pictures or clips")
    ap.add_argument("--upload", action="store_true", help="upload the session (a release with a tag of its own)")
    ap.add_argument("--recordings", default=os.path.expanduser("~/recordings"))
    g = ap.add_argument_group("simulator (dry run)")
    g.add_argument("--dry", action="store_true")
    g.add_argument("--washout", action="store_true", help="the overhead's board washed out (27 Oct 11:20)")
    g.add_argument("--drop-pct", type=float, default=0.0, help="lost packets, %% each way")
    g.add_argument("--dead", help="NAME@T: that robot dies T s into the run (silent, still)")
    g.add_argument("--seed", type=int, default=1)
    ap.add_argument("--allow-half-table", action="store_true", help="(11-05h) run an experiment even though the cameras "
                    "cannot read part of its robots' area (Dr. Sodhi's OK needed, Jay-Sun 5 Nov)")
    g.add_argument("--sim-robots", type=int, default=2, help="(11-05h) with --robots auto: this many simulated robots")
    g.add_argument("--sim-bind", action="store_true", help="(11-05h) run the real wiggle-first binding against the "
                   "point-fix simulator first (found however many, heading nudge, compass check after a restart)")
    g.add_argument("--sim-tag-yaw", default="", metavar="NAME=DEG,...", help="(11-05h) those robots' roof tags are "
                   "turned on them by DEG (the simulator draws and reads them so)")
    g.add_argument("--sim-clock-skew", type=float, default=0.0, metavar="S",
                   help="(11-05h) the simulated cameras stamp their pictures S s off the Pi's clock")
    g.add_argument("--sim-dark", action="append", default=[], metavar="X0,Y0,X1,Y1",
                   help="(11-05h) the cameras cannot read tags in this rectangle (cm)")
    g.add_argument("--sim-free", default="", help="simulated free space (GB), e.g. card=0.8,drive=50: the guard's "
                                                  "refusal / frames stop")
    g.add_argument("--sim-cond", help="a JSON file of sim_shim.py conditions (sweeps/sim_shim.py), added to the above")
    g.add_argument("--sim-arg", action="append", default=[], metavar="ARG",
                   help="one more fleet_sim.py argument, e.g. --sim-arg=--turn-err=Left=7.5 (repeatable)")
    ap.add_argument("--binding", choices=["auto", "wiggle", "fixed"], default="auto",
                    help="roof tags: wiggle = before the run each robot wiggles and the tag that turns with it is its "
                         "tag, the robot given its slot (5 Nov 2026); fixed = the site's typed tags.ids (the old way); "
                         "auto = the site's tags.binding, else wiggle")
    ap.add_argument("--binding-file", default="", help=argparse.SUPPRESS)   # a feel-out child: the parent's binding
    ap.add_argument("--rp-arg", action="append", default=[], metavar="ARG",
                    help="one more run_plan.py argument, e.g. --rp-arg=--send --rp-arg='set gyro_scale 1.075'")
    ap.add_argument("--no-turn-pause", action="store_true",
                    help="drive straight on after each turn (before 2026-10-29); default: stand until the cameras "
                         "agree with the robot's own pose (run_plan --turn-pause)")
    g.add_argument("--below-net-doe", action="store_true",
                   help="simulator only: allow robot gaps under the 8 cm safety net (the safety-net DOE)")
    ap.add_argument("--motion", choices=("stopturn", "curves"), default=None,
                    help="(5 Nov) how the robots turn: stop and turn on the spot at every point (default) or curves at "
                         "the corners where the map leaves room (both always available; Mission Control's choice)")
    ap.add_argument("--setting", action="append", default=[], metavar="NAME=VALUE",
                    help="a mission setting (missions.SETTINGS), e.g. separation_cm=6.5 (repeatable)")
    ap.add_argument("--site", default="", help="the site profile (default: the one in use; home: lights as before)")
    ap.add_argument("--tracking", choices=["auto", "lights", "tags", "none"], default="auto",
                    help="how the robots are followed: the site's run_tracking (auto), their lights, their roof AprilTags, or none (odometry only, no camera)")
    ap.add_argument("--session-id", default="", help="the session id both teams write down (Mission Control's)")
    ap.add_argument("--refit", action="store_true", help="tags: after the run, fit the cameras again with the run's "
                                                          "tag reads (saved beside the run, not used until chosen); "
                                                          "the run's end waits for it (the old way)")
    ap.add_argument("--refit-bg", action="store_true", help="tags: the same refit, started in the background as its own "
                                                             "process (2026-11-03): the run ends at once; its folder's "
                                                             "tagcal_refit/status.json says when it is done")
    g.add_argument("--cam-size", default="1280x960", help="tags: the simulated cameras' picture size")
    g.add_argument("--tag-gap", action="append", default=[], metavar="NAME@T:DUR",
                   help="tags: that robot's tag covered (unreadable) from T s after the plan starts for DUR s")
    g.add_argument("--cam-low-res", action="append", default=[], metavar="CAM=WxH",
                   help="tags: that camera sends smaller pictures, e.g. side=640x480")
    g.add_argument("--no-typed", action="store_true", help="tags: the simulated setup types no table size")
    g.add_argument("--no-refs", action="store_true", help="tags: no reference tags on the simulated table")
    g.add_argument("--hide-ref", default="", metavar="IDS", help="tags: these reference tags not on the simulated "
                                                                  "table (e.g. 20: corner 1's)")
    g.add_argument("--bump", action="append", default=[], metavar="CAM@T:DEG[:AXIS]",
                   help="tags: that simulated camera knocked T s after the plan starts, turned DEG degrees")
    ap.add_argument("--corner-clicks", choices=["auto", "on", "off"], default="auto",
                    help="tags: the setup's corner clicks (auto: the site's corner_clicks switch; on without one)")
    g.add_argument("--sim-true-floor", default="", metavar="X0,Y0,X1,Y1",
                   help="tags: the real table when the map's outline is wrong (the feel-out's tests)")
    ap.add_argument("--feel-table", choices=["auto", "always", "skip"], default="",
                    help="tags: feel out the table first (its edges felt by the robots against the cameras' outline): "
                         "auto = when this table has no saved calibration for this camera setup (default for the "
                         "robots), always, skip (default for --dry)")
    ap.add_argument("--feel-only", action="store_true", help="tags: feel out the table (with --feel-table) and stop: "
                                                               "no production run")
    ap.add_argument("--feel-conf-dir", default="", help="where table calibrations are saved (default: the site's folder; "
                                                         "--dry: the session's)")
    ap.add_argument("--on-problem", choices=["auto", "keep_going", "stop"], default="auto",
                    help="this run only (2026-11-03): keep_going = a problem that used to stop the whole run is said "
                         "once, logged and worked round (only a full card, Stop / Ctrl+C or every robot out end it); "
                         "stop = as before; auto = the site's on_problem (e08: keep_going at home and school; stop is the switch)")
    ap.add_argument("--lost-robot-points", choices=["auto", "reassign", "skip"], default="auto",
                    help="with keep_going: a robot that is out hands its points not done to the others (reassign) or "
                         "they are listed as not done (skip); auto = the site's lost_robot_points")
    ap.add_argument("--feel-trust", choices=["auto", "camera", "sensor"], default="auto",
                    help="tags, feel-out: camera = a felt-vs-camera difference is reported and logged, the cameras' "
                         "outline stands and nothing waits; sensor = a side moved in up to 10 cm, held past 10 (as "
                         "built 2 Nov); auto = the site's feel_out_trust")
    ap.add_argument("--experiment", default="", choices=["", "shakedown", "e1", "e2", "e3", "e4", "default"],
                    help="a typed site's experiment routes (experiments.py, Jay-Sun 3 Nov): shakedown (no camera, default "
                         "orders), e1 in order, e2 reversed, e3 zig-zag, e4 shuffled, default; none = the site's default "
                         "orders as before")
    ap.add_argument("--swap-halves", default="auto", choices=["on", "off", "auto"],
                    help="with --experiment: the robots' halves exchanged; auto = every other run (the site's run "
                         "counter; simulator runs read it but never count)")
    ap.add_argument("--sync-wiggle", default="auto", choices=["on", "off", "auto"],
                    help="the sync wiggle at the start (both robots a quarter turn and back at their homes); auto = the "
                         "site's experiments.sync_wiggle.use (school on, home off)")
    ap.add_argument("--no-run-exports", action="store_true",
                    help="no camera_track.csv / odometry_raw.csv in the run folder at the end")
    ap.add_argument("--no-stop-lead", action="store_true", help="(e07) forward drives as planned, not sent each "
                    "robot's stop lead short (the way before e07)")
    ap.add_argument("--roam-min", type=float, default=0.0, help="(e07) free roam at a tag site: each robot wanders "
                    "over its share of the readable points in a random order for about this many minutes (with "
                    "--motion curves; the one-button Free roam at school)")
    ap.add_argument("--roam-controller", choices=("planner", "freeroam"), default=None,
                    help="(e10, C3) the school's free roam: the free-roam controller (default: freeroam.py, one "
                         "pipeline) or the planner over the points (e07, the switch)")
    ap.add_argument("--points-each", type=int, default=3, help="tags, no --program: points per robot")
    ap.add_argument("--dwell-s", type=float, default=3.0, help="tags, no --program: seconds at each point")
    ap.add_argument("--no-mirror", action="store_true",
                    help="Left and Right are not re-timed to mirror each other (before 2026-10-29 20:06); default: the "
                         "late one speeds up and the leader slows down (5-10 cm/s) or waits (run_plan's mirror sync)")
    ap.add_argument("--visit-order", default="", metavar="P01,P05,...",
                    help="(e11, Nick 9 Nov) Visit points in this order (repeats allowed): one list for the robots "
                         "(one robot: all of it) or NAME:P01,P05;NAME2:P11 (each its own)")
    ap.add_argument("--visit-speed-cms", type=float, default=None,
                    help="(e11) with --visit-order: the driving speed, cm/s (default: the site's; school 10)")
    ap.add_argument("--visit-dwell-s", type=float, default=None,
                    help="(e11) with --visit-order: the stop at each point, s (default %g, Jay-Sun's tours)" % 8.0)
    ap.add_argument("--visit-chunk-cm", type=float, default=None,
                    help="(Nick 9 Nov) with --visit-order: drives cut into pieces of at most this, cm; 0 = each leg in one "
                         "go (default: the planner's 15 cm)")
    ap.add_argument("--visit-keep-unreadable", action="store_true",
                    help="(e11) with --visit-order: keep points the camera cannot read (default: flagged and skipped)")
    g.add_argument("--practice-real", action="store_true",
                   help="(e11, 9 Nov) practice on the REAL path (implies --dry): simulated robots that answer with WiFi "
                        "names (AL-xxxx) under aliases, the real launcher arguments, real CameraModel camera models")
    args = ap.parse_args(argv)
    # (e11) --motion: the planner's default stays stopturn; free roam's default is curves (roam_motion_of); a motion
    # given in any form (--motion X or --motion=X) is the one used
    args.motion_given_ = args.motion
    args.motion = args.motion or "stopturn"
    if args.practice_real:
        args.dry = True
        if args.sim_bind:
            say("PRACTICE ON THE REAL PATH: the simulated wiggle binding is left out (the robots keep their WiFi names; "
                "the run gives them their slots)")
            args.sim_bind = False
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
