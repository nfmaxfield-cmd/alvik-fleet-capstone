#!/usr/bin/env python3
"""The live table for Mission Control's Start screen and the new map-first page (Mission Control build, Nick 12 Nov,
decision 1c; the audit's F3: robots were drawn only in Details > Run).

snapshot(starter) -> {"table": {x_min, y_min, x_max, y_max}, "points": {label: [x, y]}, "robots": [...], "running",
"source", "t"}. One robot:
  name (its nickname: Left, Center, Right, Alpha ...), ref (its WiFi name or address, for reference), colour,
  x, y, th (cm, deg; None while unknown), pm_cm (its 95 % error, None while unknown), seen (the camera has it now),
  source (in words), since_fix_s, battery, doing, held, needs_hand, out
Read from what the runs already write (nothing new in a run):
  a roof-tag site's run (mission_run -> run_plan): live.json (twice a second);
  home free roam (freeroam.py): camera_track.csv (the camera's fixes), follow.csv (each robot's own pose and its
  doubt), battery.csv, state.json (once a minute) and edge_parked.json (robots waiting for a hand).
Before a run: the site's robots at their homes or last known spots are not drawn (nothing is known); the table and its
points are.

Changelog
  2026-11-12a  first version (Mission Control build)
"""
import csv
import glob
import io
import json
import math
import os
import time

VERSION = "live-view-2026-11-12a"
FRESH_S = 1.0         # the robots take camera fixes up to 1 s old (params.h fix_max_age_ms): a fix older than this is
                      # not "the camera has it now". No DOE: the firmware's own limit
K95_2D = 2.45         # the 95 % circle of a 2-D normal is 2.45 standard deviations (chi-square, 2 degrees of freedom)
TAIL_BYTES = 16384


def tail_rows(path, n_bytes=TAIL_BYTES):
    """The CSV's header and its last rows (only the end of the file is read)."""
    try:
        with open(path, "rb") as f:
            head = f.readline().decode("utf-8", "replace")
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(len(head), size - n_bytes))
            body = f.read().decode("utf-8", "replace")
    except OSError:
        return []
    lines = body.splitlines()
    if size > n_bytes and lines:
        lines = lines[1:]               # a cut first line
    return list(csv.DictReader(io.StringIO(head + "\n".join(lines))))


def _f(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def wrap360(a):
    return None if a is None else a % 360.0


def from_live(live, names_colours, refs, needs=()):
    """A roof-tag run's live.json -> robots."""
    out = []
    now = time.time()
    for n, r in ((live or {}).get("robots") or {}).items():
        cam, own, conf = r.get("cam"), r.get("own"), r.get("conf") or {}
        p = cam or own
        src = conf.get("source") or ("camera" if cam else "own wheels + gyro" if own else "not known")
        blind = src == "odometry only" or not cam
        out.append(dict(name=n, ref=refs.get(n, ""), colour=names_colours.get(n),
                        x=_f(p[0]) if p else None, y=_f(p[1]) if p else None,
                        th=wrap360(_f(p[2])) if p and len(p) > 2 and p[2] is not None else None,
                        pm_cm=_f(conf.get("plus_minus_cm")), seen=not blind,
                        source="its own wheels and gyro (no camera)" if blind else src,
                        since_fix_s=_f(conf.get("since_fix_s")), battery=r.get("battery"), doing=r.get("doing") or "",
                        held=bool(r.get("held")), out=r.get("out") or "", edge_cm=r.get("edge_cm"),
                        needs_hand=any(n in w for w in needs)))
    return out


def from_freeroam(folder, names_colours, refs, needs=(), now=None):
    """Home free roam's files -> robots."""
    now = time.time() if now is None else now
    cam, own, bat = {}, {}, {}
    for r in tail_rows(os.path.join(folder, "camera_track.csv")):
        if r.get("robot"):
            cam[r["robot"]] = r
    for r in tail_rows(os.path.join(folder, "follow.csv")):
        if r.get("robot"):
            own[r["robot"]] = r
    for r in tail_rows(os.path.join(folder, "battery.csv"), 4096):
        if r.get("robot"):
            bat[r["robot"]] = r.get("pct")
    state = {}
    try:
        with open(os.path.join(folder, "state.json")) as f:
            state = json.load(f).get("robots") or {}
    except (OSError, ValueError):
        pass
    try:
        with open(os.path.join(folder, "edge_parked.json")) as f:
            ep = json.load(f)
        needs = list(needs) + [str(q.get("words") if isinstance(q, dict) else q) for q in (ep.get("needs_hand") or [])]
    except (OSError, ValueError, AttributeError):
        pass
    out = []
    for n in sorted(set(cam) | set(own) | set(names_colours)):
        c, o = cam.get(n), own.get(n)
        tc = _f((c or {}).get("t"))
        fresh = tc is not None and now - tc <= FRESH_S
        sd = _f((o or {}).get("pos_sd_cm"))
        ox, oy = _f((o or {}).get("x_cm")), _f((o or {}).get("y_cm"))
        own_ok = o is not None and (o.get("state") or "") != "nopose" and ox is not None
        if fresh:
            x, y, th = _f(c.get("x_cm")), _f(c.get("y_cm")), _f(c.get("th_deg"))
            src = "the overhead camera (its lights)"
        elif own_ok:
            x, y, th = ox, oy, _f(o.get("th_deg"))
            src = "its own wheels and gyro (no camera fix for %.0f s)" % (now - tc) if tc else \
                "its own wheels and gyro (no camera fix yet)"
        elif c is not None:
            x, y, th = _f(c.get("x_cm")), _f(c.get("y_cm")), _f(c.get("th_deg"))
            src = "the camera's last fix, %.0f s ago" % (now - tc) if tc else "the camera's last fix"
        else:
            x = y = th = None
            src = "not known yet"
        st = state.get(n) or {}
        if x is None and n not in names_colours:
            continue
        out.append(dict(name=n, ref=refs.get(n, ""), colour=names_colours.get(n), x=x, y=y, th=wrap360(th),
                        pm_cm=round(K95_2D * sd, 1) if sd else None, seen=fresh, source=src,
                        since_fix_s=round(now - tc, 1) if tc else None, battery=bat.get(n) or st.get("battery"),
                        doing=st.get("mode") or "", held=bool(st.get("lost")), out=st.get("out") or "",
                        needs_hand=any(n in w for w in needs)))
    return out


_SAID = {}


def wifi_names_said(path, n_bytes=400000):
    """{nickname: WiFi name} from a run's own lines ("AL-1DC0 is Alpha"), read once per file size."""
    import re
    try:
        size = os.path.getsize(path)
    except OSError:
        return {}
    c = _SAID.get(path)
    if c and c[0] == size:
        return c[1]
    try:
        with open(path, errors="replace") as f:
            txt = f.read(n_bytes)
    except OSError:
        return {}
    out = {m.group(2): m.group(1) for m in re.finditer(r"\b(AL-[0-9A-Fa-f]{4}) is (\w+)", txt)}
    _SAID[path] = (size, out)
    return out


def freeroam_folder(starter):
    """Home free roam's own folder: the practice run's --dir, else the newest logs/*-freeroam made since the start."""
    job = getattr(starter, "job", None)
    if job is None:
        return None
    if os.path.exists(os.path.join(job.folder, "run.json")):
        return job.folder
    repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    t0 = getattr(job, "t0", 0) - 5
    c = [d for d in glob.glob(os.path.join(repo, "logs", "*-freeroam*")) if os.path.isdir(d) and
         os.path.getmtime(d) >= t0]
    return max(c, key=os.path.getmtime) if c else None


def table_of(S, folder=None):
    m = getattr(S, "map", None) or {}
    f = m.get("floor") or {}
    if folder:
        try:
            with open(os.path.join(folder, "run.json")) as fh:
                r = json.load(fh).get("rect")
            if r:
                return {"x_min": r[0], "y_min": r[1], "x_max": r[2], "y_max": r[3]}
        except (OSError, ValueError):
            pass
    if f:
        return {k: f[k] for k in ("x_min", "y_min", "x_max", "y_max")}
    return {"x_min": 0.0, "y_min": 0.0, "x_max": 122.0, "y_max": 122.0}


def snapshot(starter):
    S = starter.S
    import start_job as SJB
    site = S.site or {}
    colours = {r["name"]: r.get("colour") for r in site.get("robots") or []}
    refs = SJB.robot_refs(site)
    kind = starter.active
    running = starter.busy()
    st_needs = []
    try:
        st_needs = list(starter.edge_parked_words(starter.run_folder(), key="needs_hand") or []) if running else []
    except Exception:  # noqa: BLE001
        st_needs = []
    robots, folder, src = [], None, ""
    if kind and S.run is not None and starter.tags() and kind in ("visit", "experiment", "freeroam"):
        stt = S.run.status()
        robots = from_live(stt.get("live"), colours, refs, st_needs)
        # binding: each nickname's WiFi name (a tag site's robots are named by their wiggle)
        try:
            for p in (os.path.join(S.run.session, "bind", "binding.json"),
                      os.path.join(S.run.folder, "bind", "binding.json")):
                if os.path.exists(p):
                    with open(p) as fh:
                        b = json.load(fh)
                    wn = {r.get("slot"): r.get("wifi_name") or r.get("ip") for r in b.get("robots") or []}
                    for r in robots:
                        r["ref"] = r["ref"] or wn.get(r["name"], "")
                    break
        except (OSError, ValueError, AttributeError):
            pass
        if any(not r["ref"] for r in robots):
            wn = wifi_names_said(os.path.join(S.run.folder, "run_plan_stdout.txt"))
            for r in robots:
                r["ref"] = r["ref"] or wn.get(r["name"], "")
        src = "live.json (the run's planner, twice a second)"
    elif kind and not starter.tags():
        folder = freeroam_folder(starter)
        if folder:
            robots = from_freeroam(folder, colours, refs, st_needs)
            src = "camera_track.csv and follow.csv (free roam's camera fixes and each robot's own pose)"
    for i, r in enumerate(robots):
        r["colour"] = r.get("colour") or ("#2952CC", "#C2410C", "#7A3EC8", "#0E7C86", "#B54708")[i % 5]
    pts = {}
    for k, v in ((getattr(S, "map", None) or {}).get("spots") or {}).items():
        if isinstance(v, (list, tuple)) and len(v) >= 2:
            pts[k] = [v[0], v[1]]
    return {"t": time.time(), "running": running, "kind": kind, "table": table_of(S, folder), "points": pts,
            "robots": robots, "source": src, "version": VERSION}
