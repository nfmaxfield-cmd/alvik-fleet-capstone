#!/usr/bin/env python3
"""Wiggle-first naming before EVERY run (Nick, 5 Nov 2026: "wiggle first then attach the robot that wiggled to its
corresponding number"). Before a run with roof tags:

  1. find    every robot on the WiFi, at whatever address it answers from (`wake` + `status` to the broadcast and to
             each robot heard; a sleeping robot answers at its check-in), its compass check if it restarted;
  2. wiggle  each robot in turn does the sync wiggle (a quarter turn and back); the roof tag that turns with it is its
             tag (calib_run.bind_by_wiggle: refused in plain words when no tag or two tags turn);
  3. slots   each robot is given one of the run's robot slots (Alpha / Bravo, Left / Right ...): a robot the site pins
             (robots[].wifi = its WiFi name) gets that slot; the others by where the camera sees them, the slot whose
             home is nearest (tags.slot_by "position", the default) or keeping a name that already is a slot
             (tags.slot_by "name");
  4. name    a robot whose name is not its slot is renamed with `name <slot>` (kept on the robot; temporary names first
             when two robots swap); tags.rename_to_slot false (the school, 5 Nov 11:00): not renamed, the slot name
             lives in the run only (record "aliases"; run_plan --alias AL-1DC0=Alpha);
  5. record  <conf_dir>/robot_binding.json and the session's binding.json: slot -> tag id, WiFi name, address. The run's
             tag bridge then follows those tag ids (instead of the site's typed tags.ids).

The site's tags.binding switch: "wiggle" (the default since 5 Nov) or "fixed" (the typed tags.ids, the old way, kept).
(5 Nov 11:00, Nick) Before anything moves, a robot on firmware older than calibration.min_firmware is refused. Pins
(tags.pin, default on): <conf_dir>/robot_pins.json keeps, per WiFi name, the roof tag a wiggle found and the slot it
was given; a pinned robot does a short check wiggle (its pinned tag must turn; another tag turning re-binds it, said
loudly) and keeps its slot. A roof tag on the table that no robot of the run turned is refused (tags.unclaimed).

    python3 tools/fleet/wiggle_bind.py --robots Alpha,Bravo                    # on the Pi (mission_run calls it too)
    python3 tools/fleet/wiggle_bind.py --robots Alpha,Bravo --dry              # the simulator
"""
import argparse
import datetime as dt
import itertools
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition"),
          os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)

import calib_run as CR   # noqa: E402

VERSION = "2026-11-11e15"   # e15 (school 10 Nov 18:54): each tag's turn remembered across runs (tag_heading_store); a heading check that loses its tag is tried again where the tag reads (stand longer, longer window), then the remembered turn, then the finding stroke; record yaw_state / yaw_how (MIRLT_TAG_HEADING_MEMORY=0: per run only; MIRLT_HEADING_CHECK=once: the check once, as before). Before: "2026-11-10e14d"   # e14d: a robot brought back is bound by its tag's first read where the motion detector had it (said in its note). Before: "2026-11-10e14"   # e14: every robot found by its own small move first (motion_find.run_finding: anywhere on
#   the table, with or without a tag that reads; brought back to where tags read; a robot with no tag that reads kept
#   as "tagless" with its motion position and heading); the wiggle only for robots the finding did not bind.
#   MIRLT_CAMERA_DISCOVERY=off: the wiggle alone (before e14). e13: the camera check (camera_feed) before binding
VERSION_A = "2026-11-09a"
say = CR.say


def binding_mode(site, override="auto"):
    """'wiggle' or 'fixed': --binding, else the site's tags.binding, else wiggle."""
    if override and override != "auto":
        return override
    return str(((site or {}).get("tags") or {}).get("binding") or "wiggle")


def slot_homes(site, S, slots):
    """{slot: (x, y)} from the site's homes (a production point per slot), where known."""
    hp = (site or {}).get("homes") or {}
    out = {}
    for sl in slots:
        lab = hp.get(sl)
        if isinstance(lab, str) and lab in S.points:
            out[sl] = tuple(S.points[lab])
    return out


def assign_slots(slots, robots, seen, homes, pins=None, by="position"):
    """{robot name: slot}. robots: names as heard; seen: {name: (x, y)} from the camera; homes: {slot: (x, y)};
    pins: {robot name: slot} from the site. Raises CR.BindError in plain words when it cannot."""
    pins = dict(pins or {})
    if len(robots) > len(slots):
        raise CR.BindError("%d robots answered (%s) but the run has %d robot slots (%s): switch the extra one off or "
                           "add a slot" % (len(robots), ", ".join(robots), len(slots), ", ".join(slots)))
    out = {r: pins[r] for r in robots if pins.get(r) in slots}
    free_s = [s for s in slots if s not in out.values()]
    free_r = [r for r in robots if r not in out]
    if by == "name":
        for r in list(free_r):
            if r in free_s:
                out[r] = r
                free_s.remove(r)
                free_r.remove(r)
    if free_r and any(s in homes for s in free_s) and not all(s in homes for s in free_s) and \
            all(r in seen for r in free_r):
        # (11-05h) more robots than the site has homes for: the homed slots go to the nearest robots first
        for s_ in [s for s in free_s if s in homes]:
            if not free_r:
                break
            r_ = min(free_r, key=lambda r: math.hypot(seen[r][0] - homes[s_][0], seen[r][1] - homes[s_][1]))
            out[r_] = s_
            free_r.remove(r_)
            free_s.remove(s_)
    if free_r and all(s in homes for s in free_s) and all(r in seen for r in free_r):
        best = None
        for perm in itertools.permutations(free_s, len(free_r)):
            c = sum(math.hypot(seen[r][0] - homes[s][0], seen[r][1] - homes[s][1]) for r, s in zip(free_r, perm))
            if best is None or c < best[0]:
                best = (c, perm)
        out.update(dict(zip(free_r, best[1])))
    else:
        for r in free_r:                       # no homes or no positions: keep a slot name, else in order
            s = r if r in free_s else free_s[0]
            out[r] = s
            free_s.remove(s)
    return out


def rename_steps(current, assign):
    """[(ip, old name, new name)] so that no two robots ever share a name: robots whose new name another robot still
    has are first given a temporary name (t-<last part of the address>)."""
    move = {r: s for r, s in assign.items() if r != s}
    if not move:
        return []
    steps = []
    names = dict(current)        # name -> ip
    clash = [r for r, s in move.items() if s in names and s != r]
    tmp = {}
    for r in list(move):
        if clash:
            t = "t-%s" % current[r].split(".")[-1]
            steps.append((current[r], r, t))
            tmp[r] = t
    for r, s in move.items():
        steps.append((current[r], tmp.get(r, r), s))
    return steps


def rename(link, steps, log=say):
    """Send each `name`, waiting for its answer ("<old> is now called <new>"). Returns the steps that answered."""
    done = []
    for ip, old, new in steps:
        ok = None
        for _ in range(3):
            k = link.mark()
            link.send(ip, "name %s" % new)
            ok = link.wait_for(lambda n, l: l if "is now called %s" % new in l else None, k, 3.0)
            if ok:
                break
        if not ok:
            raise CR.BindError("the robot at %s did not answer `name %s` (it was %s): not renamed; nothing ran"
                               % (ip, new, old))
        log("  %s at %s is now called %s" % (old, ip, new))
        done.append((ip, old, new))
    return done


PHANTOM_ID = 900       # (e14) tag ids no roof tag carries (tag25h9 has 35): a tagless robot's name in the run's tags

AUTO_NAMES = ("Alpha", "Bravo", "Charlie", "Delta", "Echo", "Foxtrot")   # (11-05h) slot names when the run takes every
#                                                                         robot found (after the site's own names)


def auto_slots(site, n):
    """(11-05h) n slot names: the site's robots' names first, then AUTO_NAMES not already used."""
    out = [r["name"] for r in (site or {}).get("robots") or []][:n]
    for nm in AUTO_NAMES:
        if len(out) >= n:
            break
        if nm not in out:
            out.append(nm)
    k = 1
    while len(out) < n:
        out.append("Robot%d" % k)
        k += 1
    return out


def clear_space_ok(cl, dry_truth=None):
    """(e14) The person's word on the finding strokes' clear space: the simulator's person looks at the truth (robots
    with that much room from every edge); MIRLT_CLEAR_SPACE_OK=1 (Mission Control) yes; a terminal asks; else no (each
    robot then wiggles on the spot)."""
    if dry_truth is not None:
        L, W, truth = dry_truth
        return {n for n, p_ in truth.items() if min(p_[0], p_[1], L - p_[0], W - p_[1]) >= cl["radius_cm"]}
    if os.environ.get("MIRLT_CLEAR_SPACE_OK") == "1":
        return True
    if sys.stdin is not None and sys.stdin.isatty():
        try:
            return input("Has every robot that much clear space round it? (y/n) ").strip().lower().startswith("y")
        except EOFError:
            return False
    return False


def bind_for_run(site, slots, link, eye, ips=(), wake_min=3.0, dry=False, folder=None, log=say, by=None,
                 rename_ok=None, state_dir=None, min_firmware=None, use_pins=None, heading_check=True, cams=None,
                 rm=None, heading_store=None):
    """Steps 1-5 with a Link and an Eye already open. Returns the record (dict): slots {slot: tag}, robots [...],
    aliases {name it answers with: slot} when the robots are not renamed (tags.rename_to_slot false)."""
    S = CR.Site(site)
    tg = (site or {}).get("tags") or {}
    by = by or str(tg.get("slot_by") or "position")
    rename_ok = bool(tg.get("rename_to_slot", True)) if rename_ok is None else rename_ok
    use_pins = bool(tg.get("pin", True)) if use_pins is None else use_pins
    if min_firmware is None:
        min_firmware = ((site or {}).get("calibration") or {}).get("min_firmware", CR.MIN_FIRMWARE) or ""
    if state_dir is None:
        state_dir = (folder or "") if dry else ((site or {}).get("conf_dir") or "")
    auto = list(slots) in (["auto"], [])
    if auto:
        # (11-05h, Observe then trust) however many robots are on the table: the camera counts the roof tags there
        # and the run waits (up to wake_min) for that many robots to answer; slots are named after the site's robots,
        # then AUTO_NAMES
        n_tags = len(eye.seen_ids(2.0)) if eye is not None else 0
        log("AUTO: the camera sees %d roof tag%s on the table: waiting for %s robot%s to answer" % (
            n_tags, "" if n_tags == 1 else "s", n_tags or "any", "" if n_tags == 1 else "s"))
        found = CR.discover(link, list(ips), wake_min, dry, count=n_tags)
        slots = auto_slots(site, len(found))
    else:
        found = CR.discover(link, list(ips), wake_min, dry, count=len(slots))
    if not found:
        raise CR.BindError("no robot answered on the WiFi (wake and status to %s for %.0f min). Is it switched on and "
                           "on the school WiFi? `python3 tools/pi/robot_sleep.py wake any` wakes one from any address."
                           % (", ".join(ips) or "the broadcast", wake_min))
    log("Found: %s" % ", ".join("%s at %s" % kv for kv in found.items()))
    robots = [CR.Robot(link, n, ip) for n, ip in found.items()]
    # (Nick, 5 Nov 11:00) older firmware: refused before anything moves (switch calibration.min_firmware)
    fw = {rb.name: CR.firmware_of(rb) for rb in robots}
    old = [CR.firmware_refusal(n, v, min_firmware) for n, v in fw.items() if not CR.firmware_ok(v, min_firmware)]
    if old:
        raise CR.BindError("; ".join(old) + ". Nothing moved")
    import observe as OB
    se = OB.site_safe_edge(site, 3)
    for rb in robots:
        link.send(rb.ip, "wake")
        for c in [("set safe_edge %d" % se) if c.startswith("set safe_edge") else c for c in CR.EDGE_GUARD]:
            link.send(rb.ip, c)
            time.sleep(0.05)
    # the WiFi name (firmware 2026-11-05a `id`; older: the name it answered with) before the wiggle: the pins use it
    wifi = {rb.ip: CR.wifi_name_of(link, rb) for rb in robots}
    cc = {rb.name: CR.compass_check(link, rb, log=log)[0] for rb in robots}
    # (11-09, school: Jay-Sun's AL-5554 answered on the WiFi, refused its compass check and stopped a one-robot tour)
    # auto mode: a robot that refuses its compass check is left out of this run, said plainly, while another remains
    bad = [rb for rb in robots if not cc.get(rb.name)]
    if auto and bad and len(bad) < len(robots):
        for rb in bad:
            log("  LEFT OUT of this run: %s (at %s) refused its compass check; the run carries on without it. It is "
                "not driven. Switch it off if it is not meant to be here" % (rb.name, rb.ip))
        robots = [rb for rb in robots if cc.get(rb.name)]
        found = {n: ip for n, ip in found.items() if cc.get(n)}
        slots = auto_slots(site, len(found))
    for rb in robots:
        # (11-05h, Nick 20:46) the robot-side safety layers on and read back before the wiggle moves anything
        ok, words = CR.arm_safety(rb, se, log=log)
        if not ok:
            raise CR.BindError("%s: its safety layers (tilt stop, edge guard) did not read back (%s). Nothing else "
                               "moved" % (rb.name, words))
    import experiments as EX
    w = EX.conf(site)["sync_wiggle"]
    pp = CR.state_path(state_dir, CR.PINS_FILE) if use_pins else ""
    pins = CR.load_pins(site, pp) if use_pins else {}
    expect = {rb.name: pins[wifi[rb.ip]]["tag"] for rb in robots
              if wifi.get(rb.ip) in pins and pins[wifi[rb.ip]].get("tag") is not None}
    if expect:
        log("Pinned: %s: a %.0f deg check wiggle each" % (", ".join("%s = tag %s" % kv for kv in expect.items()),
                                                         CR.PIN_CHECK_DEG))
    time.sleep(1.0)
    notes = {}
    # (e14, Nick 10 Nov) every robot found by its own small move first: anywhere on the table, with or without a tag
    # that reads; brought back to where tags read; one carrying no tag that reads is kept as tagless (motion, lock-on,
    # odometry track it)
    import motion_find as MF
    pre, tagless, mres = {}, {}, None
    if MF.enabled() and cams:
        import camera_discovery as CD
        if rm is None:
            import observe as OB
            rm = OB.ReadMap(S.L, S.W)
            rm.seed(lambda x_, y_: 0.9, "no map given: taken as readable until learned")
        m0 = next((c.model for c in cams if c.model is not None), None)
        cpp = None
        if m0 is not None:
            try:
                cpp = 1.0 / float(m0.px_per_cm(S.L / 2.0, S.W / 2.0, 4.0))
            except Exception:  # noqa: BLE001
                cpp = None
        cl = CD.clearance(cpp)
        log("CLEAR SPACE (finding each robot by its own small move): " + cl["words"])
        ok = clear_space_ok(cl, (S.L, S.W, eye.truth_now()) if (dry and hasattr(eye, "truth_now")) else None)
        if ok is not True:
            log("  clear space confirmed for %s; the others wiggle on the spot instead (only their own footprint)" % (
                ", ".join(sorted(ok)) if isinstance(ok, set) and ok else "no robot"))
        guesses = {}
        for rb in robots:
            t_ = expect.get(rb.name)
            l_ = eye.last(int(t_)) if t_ is not None else None
            if l_ is not None and time.time() - l_[0] < 3.0:
                guesses[rb.name] = (l_[1], l_[2], l_[3])
        try:
            pre, mres = MF.run_finding({rb.name: rb for rb in robots}, eye, cams, S, rm,
                                       ok if isinstance(ok, set) else (True if ok else set()), guesses, log=log,
                                       folder=folder)
        except MF.StopAsked as e:
            raise CR.BindError(str(e))
        for n_ in mres.get("tagless") or []:
            f_ = mres["robots"][n_]
            tagless[n_] = dict(seen_cm=f_.get("now") or f_.get("where"), heading=f_.get("heading_now", f_.get(
                "heading")), sigma=f_.get("sigma"))
        for n_, t_ in pre.items():
            notes[n_] = "found by motion: " + ((mres["robots"].get(n_) or {}).get("bound_by") or
                                               "tag %d moved with its own stroke" % t_)
            log("  %s: %s" % (n_, notes[n_]))
    rest = [rb for rb in robots if rb.name not in pre and rb.name not in tagless]
    b = dict(pre)
    if rest:
        b.update(CR.bind_by_wiggle(rest, eye, wiggle_deg=float(w["deg"]), pause_s=float(w["pause_s"]), log=log,
                                   expect={k: v for k, v in expect.items() if k in {r.name for r in rest}},
                                   check_deg=CR.PIN_CHECK_DEG, notes=notes))
    # (5 Nov 11:00) a roof tag on the table that no robot of this run claimed (a third robot, asleep or not in the
    # run): the run would not know it is there. Refused (switch tags.unclaimed: refuse / ignore)
    claimed = set(b.values())
    stray = [t for t in eye.seen_ids(2.0) if t not in claimed]
    if stray and str(tg.get("unclaimed") or "refuse") == "refuse":
        raise CR.BindError("roof tag %s is on the table but no robot of this run turned it (a robot asleep, switched "
                           "off, refused, or one more than the run's %d slots %s). The run would not know it is there: "
                           "take that robot off the table, or add it to the run. Nothing else moved" % (
                               ", ".join(map(str, stray)), len(slots), ", ".join(slots)))
    seen = {}
    for n, t in b.items():
        m = eye.measure(t, 0.6)
        if m:
            seen[n] = (m[0], m[1])
    for n, tl in tagless.items():
        if tl.get("seen_cm"):
            seen[n] = tuple(tl["seen_cm"][:2])       # (e14) where the motion has it
    # (11-05h, Observe then trust) each robot's roof tag heading against its nose: a 6 cm nudge (clear both ways, the
    # other robots kept out), before any run drives on the tag's heading (school 5 Nov: AL-1DC0's tag read 180 off)
    yaw = {}
    if seen:
        # the camera's view limit is a typed guess (calibration's VIEW_X_MAX_CM): a robot the camera reads where it
        # stands is inside the view (observed); only the edges, the outlets and the other robots limit the nudge
        S.view_x = max(S.view_x, max(p_[0] for p_ in seen.values()) + 2 * OB.NUDGE_CM + 10.0)
    # (e15, school 10 Nov 18:54) each tag's turn remembered across runs (tag_heading_store): the heading check that
    # loses its tag is tried again where the tag reads, then the remembered turn, then the finding stroke's direction;
    # a robot is never left on its raw tag heading without the run knowing (yaw_state "unknown": run_plan crawls first)
    import tag_heading_store as THS
    if heading_store is None:
        heading_store = THS.path_for(site, state_dir) if THS.enabled(site) else ""
    yaw_state, yaw_how = {}, {}
    if heading_check and os.environ.get("MIRLT_HEADING_CHECK", "patient") == "once":
        # the way before e15 (MIRLT_HEADING_CHECK=once): the heading check once; lost tag: the raw heading
        for rb in robots:
            t = b.get(rb.name)
            if t is None:
                continue
            m = eye.measure(t, 0.8)
            if not m:
                log("  %s: no camera pose for the heading check (the run's own heading watch checks it on its first "
                    "drives)" % rb.name)
                continue
            S.set_parked({o: p_ for o, p_ in seen.items() if o != rb.name})
            try:
                off, raw, mv, c1 = CR.heading_nudge(rb, eye, int(t), m[:3],
                                                    lambda a_, b_: S.problems(a_, b_, buf=2.0, edge_buf=2.0), log=log)
                yaw[rb.name] = off
                yaw_state[rb.name], yaw_how[rb.name] = "measured", "heading check"
                seen[rb.name] = (c1[0], c1[1])
                if heading_store:
                    log("  " + THS.record(heading_store, t, off, "the binding's heading check",
                                          robot=wifi.get(rb.ip))["words"])
            except CR.RunEnd as e:
                log("  %s: heading check not done (%s); the run's heading watch checks it on its first drives" % (
                    rb.name, e))
        S.set_parked({})
    elif heading_check:
        for rb in robots:
            t = b.get(rb.name)
            if t is None:
                continue
            rem = THS.get(heading_store, t) if heading_store else None
            stroke = ((mres or {}).get("robots", {}).get(rb.name) or {}).get("heading") if mres else None
            m = eye.measure(t, 0.8)
            if not m:
                m = eye.measure(t, max(CR.MEASURE_S, CR.HC_READS_WANTED * CR.fix_interval(eye, t)))
            if not m:
                log("  %s: no camera pose for the heading check where it stands" % rb.name)
            S.set_parked({o: p_ for o, p_ in seen.items() if o != rb.name})
            res = CR.heading_check_patient(rb, eye, int(t), m[:3] if m else None,
                                           lambda a_, b_: S.problems(a_, b_, buf=2.0, edge_buf=2.0), log=log,
                                           remembered=rem, stroke_heading=stroke)
            yaw_state[rb.name], yaw_how[rb.name] = res["state"], res["how"]
            if res["offset"] is not None:
                yaw[rb.name] = res["offset"]
            if res.get("pose"):
                seen[rb.name] = (res["pose"][0], res["pose"][1])
            if res["state"] == "measured" and heading_store:
                r_ = THS.record(heading_store, t, res["offset"], "the binding's heading check", robot=wifi.get(rb.ip))
                log("  " + r_["words"])
        S.set_parked({})
    elif heading_store:
        for rb in robots:              # no heading check asked: the remembered turn as the starting guess
            t = b.get(rb.name)
            rem = THS.get(heading_store, t) if t is not None else None
            if rem:
                yaw[rb.name] = float(rem["offset_deg"])
                yaw_state[rb.name], yaw_how[rb.name] = "remembered", "remembered (%s)" % rem.get("last_seen")
    spins = {}
    for r in (site or {}).get("robots") or []:
        if r.get("wifi") and r["wifi"] in found:
            spins[r["wifi"]] = r["name"]
    for rb in robots:           # (5 Nov 11:00) a robot's slot pinned by its WiFi name (robot_pins.json)
        sl = (pins.get(wifi.get(rb.ip)) or {}).get("slot")
        if sl and rb.name not in spins:
            spins[rb.name] = sl
    assign = assign_slots(list(slots), list(found), seen, slot_homes(site, S, slots), spins, by)
    log("Slots (%s): %s" % ("pinned, then nearest home" if by == "position" else
                            "pinned, then names kept", ", ".join("%s -> %s" % kv for kv in assign.items())))
    steps = rename_steps(found, assign)
    renamed = []
    aliases = {}
    if steps and not rename_ok:
        # (Nick, 5 Nov 11:00) renaming on the robot switched off: the names live in the run only
        aliases = {r: s_ for r, s_ in assign.items() if r != s_}
        log("Not renamed on the robots (tags.rename_to_slot false): in this run %s" % ", ".join(
            "%s is %s" % kv for kv in aliases.items()))
    elif steps:
        renamed = rename(link, steps, log=log)
    if use_pins:
        CR.update_pins(pp, pins, {wifi.get(ip_): b[n] for n, ip_ in found.items() if n in b},
                       {wifi.get(ip_): assign[n] for n, ip_ in found.items() if n in assign})
    rec = {"how": "wiggle (wiggle_bind.py %s)" % VERSION, "time": dt.datetime.now().isoformat(timespec="seconds"),
           "slots": {assign[n]: int(t) for n, t in b.items()},
           "robots": [{"slot": assign[rb.name], "name_before": rb.name, "wifi_name": wifi.get(rb.ip), "ip": rb.ip,
                       "tag_id": b.get(rb.name), "seen_cm": seen.get(rb.name), "firmware": fw.get(rb.name),
                       "pin": notes.get(rb.name)} for rb in robots],
           "renamed": [{"ip": ip, "from": o, "to": n} for ip, o, n in renamed],
           "aliases": aliases,
           "yaw_off": {assign[n]: d for n, d in yaw.items() if n in assign},
           "yaw_state": {assign[n]: v for n, v in yaw_state.items() if n in assign},   # (e15) measured / remembered /
           "yaw_how": {assign[n]: v for n, v in yaw_how.items() if n in assign},       # motion / unknown, and how
           "tagless": {assign[n]: tl for n, tl in tagless.items() if n in assign},   # (e14) found by motion, no tag
           "found_by_motion": bool(mres),
           "auto": auto,
           "empty_slots": [s for s in slots if s not in assign.values()]}
    for p in ([os.path.join(folder, "binding.json")] if folder else []) + \
            ([os.path.join(site["conf_dir"], "robot_binding.json")] if (site.get("conf_dir") and not dry) else []):
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w") as f:
            json.dump(rec, f, indent=1)
    if rec["tagless"]:
        log("NO ROOF TAG on %s: %s" % (", ".join(sorted(rec["tagless"])), "; ".join(
            "%s found by motion at %.0f, %.0f%s" % (sl, tl["seen_cm"][0], tl["seen_cm"][1],
                                                     (" facing %.0f deg" % tl["heading"]) if tl.get("heading") is not None
                                                     else "") for sl, tl in sorted(rec["tagless"].items()))))
    log("Bound: %s%s" % (", ".join("%s = tag %d" % kv for kv in sorted(rec["slots"].items())),
                         ("; no robot for %s" % ", ".join(rec["empty_slots"])) if rec["empty_slots"] else ""))
    return rec


def tags_with_binding(tags, rec):
    """The run's tags block with the wiggle's ids (slot -> tag) in place of the typed tags.ids."""
    t = dict(tags or {})
    t["ids_typed"] = dict(t.get("ids") or {})
    t["ids"] = dict(rec["slots"])
    t["binding"] = "wiggle"
    for k, sl in enumerate(sorted(rec.get("tagless") or {})):
        # (e14) a robot with no roof tag that reads: a tag id no tag carries (PHANTOM_ID + k), so the bridge, the
        # pictures' simulator and the run know its name; the motion detector follows it from where it was found
        t["ids"][sl] = PHANTOM_ID + k
    if rec.get("tagless"):
        t["tagless"] = dict(rec["tagless"])
    if rec.get("yaw_off"):
        # (11-05h) the heading check's offsets, per tag id: the bridge corrects every heading of that tag
        t["yaw_off"] = {str(rec["slots"][sl]): d for sl, d in rec["yaw_off"].items() if sl in rec["slots"]}
    if rec.get("yaw_state"):
        # (e15) how each tag's offset is known (measured this run / remembered / motion / unknown): run_plan's facing
        # check never straightens on an unverified raw heading
        t["yaw_state"] = {str(rec["slots"][sl]): v for sl, v in rec["yaw_state"].items() if sl in rec["slots"]}
        t["yaw_how"] = {str(rec["slots"][sl]): v for sl, v in (rec.get("yaw_how") or {}).items() if sl in rec["slots"]}
    return t


def MF_on():
    import motion_find as MF
    return MF.enabled()


def real_cams(site, calib=""):
    """(e14) Every camera the Pi can open for robot finding by motion (camera_discovery.find_cameras): the recorder's
    streams, then /dev/video cameras it does not hold; the first stream with the setup's camera model."""
    import camera_discovery as CD
    from cameras import CameraModel
    tc = os.path.join(site.get("conf_dir") or os.path.expanduser("~/.config/mirlt"), "tagcal")
    cal = calib or os.path.join(tc, "overhead.json")
    try:
        model = CameraModel.load(cal) if os.path.exists(cal) else None
    except Exception:  # noqa: BLE001
        model = None
    found = CD.find_cameras()
    grabs = [CD.Grabber(c["src"]) for c in found]
    cams = [CD.CamInfo(c["name"], g.grab, model if (k == 0 and c["kind"] == "stream") else None)
            for k, (c, g) in enumerate(zip(found, grabs))]
    return cams, [g.close for g in grabs]


def run_for_mission(site, slots, folder, stream="127.0.0.1:8000", calib="", ips=(), log=say, rename_ok=None,
                    use_pins=None, rm=None, heading_store=None):
    """mission_run's step before its bridge (real robots): a tag bridge of its own on the recorder's pictures with
    every listed and spare tag id, the binding, the bridge stopped. Returns the record."""
    import argparse
    os.makedirs(folder, exist_ok=True)
    a = argparse.Namespace(calib=calib, stream=stream)
    bp, _cal, _h = CR.start_bridge(a, site, folder)
    link = eye = None
    try:
        link = CR.Link(folder)
        eye = CR.Eye(folder, CR.FIX_PORT)
        time.sleep(2.0)
        CR.feed_refusal(eye)        # (e13) every camera sending new pictures before the binding blames anything else
        cams, closers = real_cams(site, calib) if MF_on() else ([], [])
        try:
            return bind_for_run(site, slots, link, eye, ips=ips, folder=folder, log=log, rename_ok=rename_ok,
                                use_pins=use_pins, cams=cams, rm=rm, heading_store=heading_store)
        finally:
            for c in closers:
                c()
    finally:
        for o in (eye, link):
            if o is not None:
                o.close()
        if bp.poll() is None:
            bp.send_signal(signal.SIGINT)
            try:
                bp.wait(timeout=10)
            except subprocess.TimeoutExpired:
                bp.kill()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--site", default="")
    ap.add_argument("--robots", default="", help="the run's robot slots (default: the site's robots)")
    ap.add_argument("--robot", action="append", default=[], help="an address to ask too (besides the broadcast)")
    ap.add_argument("--wake-min", type=float, default=12.0)
    ap.add_argument("--slot-by", choices=["position", "name"], default=None)
    ap.add_argument("--rename", choices=["auto", "on", "off"], default="auto", help="rename each robot to its slot on "
                    "the robot (on) or keep the slot in the run only (off); auto: the site's tags.rename_to_slot")
    ap.add_argument("--no-pins", dest="pins", action="store_false", help="a full wiggle for every robot, no pins")
    ap.add_argument("--session", default="")
    ap.add_argument("--stream", default="127.0.0.1:8000")
    ap.add_argument("--calib", default="")
    g = ap.add_argument_group("simulator")
    g.add_argument("--dry", action="store_true")
    g.add_argument("--sim", action="append", default=[], metavar="NAME:TAG:POINT[:HEADING]",
                   help="a simulated robot (default AL-1DC0:2:P14 and AL-2B7F:3:P11)")
    g.add_argument("--seed", type=int, default=1)
    a = ap.parse_args(argv)
    import sites as SI
    if a.site:
        os.environ["MIRLT_SITE"] = a.site
    site = SI.load(a.site or None)
    slots = [s.strip() for s in (a.robots or ",".join(r["name"] for r in site.get("robots") or [])).split(",")
             if s.strip()]
    folder = a.session or os.path.join(REPO, "logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") + "-bind" +
                                       ("-dry" if a.dry else ""))
    os.makedirs(folder, exist_ok=True)
    if not a.dry:
        try:
            rec = run_for_mission(site, slots, folder, a.stream, a.calib, a.robot,
                                  rename_ok={"on": True, "off": False}.get(a.rename), use_pins=a.pins)
        except CR.BindError as e:
            say("NOT BOUND: %s" % e)
            return 1
        print(json.dumps(rec["slots"]))
        return 0
    # the simulator: fleet_sim robots named by their WiFi, each with its tag, standing on stickers
    CR.loopback_up()
    S = CR.Site(site)
    sims = a.sim or ["AL-1DC0:2:P14", "AL-2B7F:3:P11"]
    first = sims[0].split(":")
    sa = argparse.Namespace(sim_name=first[0], sim_also=sims[1:], seed=a.seed, sim_max_s=600, sim_start=first[2],
                            sim_true="1,1,0", sim_true_also="", sim_sleep="0.2:0.05", sim_fw=[])
    sp, ips = CR.start_sim(sa, S, folder)
    link = CR.Link(folder)
    cam = CR.SchoolCamera()
    tags = {q.split(":")[0]: int(q.split(":")[1]) for q in sims}
    eye = CR.SimEye(folder, CR.FIX_PORT + 100, cam, tags, seed=a.seed)
    try:
        time.sleep(1.0)
        rec = bind_for_run(site, slots, link, eye, ips=ips, wake_min=2.0, dry=True, folder=folder, by=a.slot_by,
                           rename_ok={"on": True, "off": False}.get(a.rename), use_pins=a.pins)
        if rec.get("aliases"):
            say("Names in the run only: %s" % ", ".join("%s is %s" % kv for kv in sorted(rec["aliases"].items())))
        print(json.dumps(rec["slots"]))
        return 0
    except CR.BindError as e:
        say("NOT BOUND: %s" % e)
        return 1
    finally:
        eye.close()
        link.close()
        sp.terminate()
        sp.wait(timeout=10)


if __name__ == "__main__":
    sys.exit(main())
