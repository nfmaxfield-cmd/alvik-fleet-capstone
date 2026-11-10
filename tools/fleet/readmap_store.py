#!/usr/bin/env python3
"""Where roof tags read, kept per session with its light and its tag (build e14, 10 Nov 2026).

The school, 10 Nov: a dim session (mean grey about 70 against 91, about 7 pictures a second against 15) was merged
into the site's readmap.json at full weight while earlier runs were halved, and the Start screen then said the readable
area was 26 cm of the table. One session's failures overwrote many sessions' successes.

Now every run's OWN evidence (the tries and reads it added, not its seed or what it carried in) is kept as one session
in <conf_dir>/readmap_store.json, with the overhead camera's brightness (the bridge's median mean grey) and the roof
tag it is about ("*" when the run did not tell tags apart). Nothing is ever overwritten or halved. At a run's start the
map is CHOSEN for the light now and each robot's tag:

  - a session's weight falls with the difference in light: exp(-0.5 (dgrey / LIGHT_SCALE)^2), so a dim session counts
    for almost nothing in bright light (grey 70 against 91: 0.03) and fully in the same light;
  - the robot's own tag counts fully, another tag's evidence at OTHER_TAG_W ("*" fully: it may be this tag);
  - each session counts at most SESSION_CAP tries per cell, so one long session cannot outvote many short ones;
  - evidence counts: the camera model's seed is PRIOR_N tries (observe.ReadMap), so a cell decides by its evidence once
    there is more of it than the seed.

The site's readmap.json is still written (the old switch MIRLT_READMAP_STORE=0 uses it as before e14) but no longer
read at the start when the store is there.

    python3 tools/fleet/readmap_store.py show [--grey 70] [--tag 4]      what a run would use now, and why
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

VERSION = "readmap-store-2026-11-10e14"
FILE = "readmap_store.json"
LIGHT_SCALE = 8.0        # CHOSEN: grey levels over which another light's evidence falls to 61 % (16: 14 %; 21, the
                         # school's dim against bright: 3 %). The camera's own exposure noise between pictures is 1-2
                         # levels. DOE row (e14): the same route at 3 light levels, read rate per cell against grey
OTHER_TAG_W = 0.5        # CHOSEN: another tag's evidence, relative to this tag's own (tags differ in print and mount;
                         # tag 2 read at P11 on 9 Nov, tag 4 did not on 10 Nov). DOE row (e14)
SESSION_CAP = 10         # CHOSEN: one session's tries per cell are capped at this (its reads scaled with them): a robot
                         # parked unread for an hour is one session's evidence, not thousands of tries. No DOE (a cap)
UNKNOWN_LIGHT_W = 0.5    # CHOSEN: a session whose light was not measured, against a known light now. No DOE (a guess,
                         # only for sessions from before e14)
KEPT_APART_W = 0.1       # words only: a session counting under this is said as "kept apart" (it still counts that little)
MAX_SESSIONS = 400       # the oldest are dropped beyond this (about two months of school runs). Not tuned
DIM_BELOW = 80.0         # words only: "dim" under this mean grey (the school: 70 dim, 91 lit, 10 Nov)


def enabled():
    return os.environ.get("MIRLT_READMAP_STORE", "1") != "0"


def path_for(folder):
    return os.path.join(folder, FILE) if folder else ""


def light_words(grey):
    if grey is None:
        return "light not measured"
    return "mean grey %.0f (%s)" % (grey, "dim" if grey < DIM_BELOW else "lit")


def load(folder):
    p = path_for(folder)
    if p and os.path.exists(p):
        try:
            with open(p) as f:
                d = json.load(f)
            if isinstance(d.get("sessions"), list):
                return d
        except (OSError, ValueError):
            pass
    return {"version": VERSION, "sessions": []}


def save(folder, d):
    p = path_for(folder)
    if not p:
        return ""
    os.makedirs(folder, exist_ok=True)
    tmp = p + ".tmp"
    with open(tmp, "w") as f:
        json.dump(d, f)
    os.replace(tmp, p)
    return p


def snapshot(rm):
    """The map's tries and reads now (to take a run's own evidence out later: evidence(rm, snapshot))."""
    return (dict(rm.tries), dict(rm.reads))


def evidence(rm, base=None):
    """{(i, j): (tries, reads)} this run added since base (snapshot), or all of it."""
    bt, br = base if base else ({}, {})
    out = {}
    for k, n in rm.tries.items():
        dn = n - bt.get(k, 0)
        dr = rm.reads.get(k, 0) - br.get(k, 0)
        if dn > 0:
            out[k] = (dn, max(0, min(dn, dr)))
    return out


def add_session(folder, rm, ev, grey=None, tag="*", kind="", when=None, sid=""):
    """One session's own evidence (evidence()) kept, with its light and its tag. Returns words."""
    if not ev:
        return "no evidence of its own"
    d = load(folder)
    cells = []
    for (i, j), (n, r) in sorted(ev.items()):
        if n > SESSION_CAP:
            r, n = r * SESSION_CAP / float(n), SESSION_CAP
        cells.append([i, j, round(n, 3), round(r, 3)])
    d.update(version=VERSION, L=rm.L, W=rm.W, cell=rm.cell, x0=rm.x0, y0=rm.y0, cameras=rm.cameras)
    d["sessions"].append({"t": when or time.time(), "grey": None if grey is None else round(float(grey), 1),
                          "tag": str(tag), "kind": kind, "id": sid, "cells": cells})
    del d["sessions"][:-MAX_SESSIONS]
    save(folder, d)
    return "kept as one session (%s, tag %s, %d cells)" % (light_words(grey), tag, len(cells))


def import_old(folder, rm_cls):
    """(once) The site's readmap.json and e13's readmap_light.json carried in as sessions: readmap.json with its light
    not known (it mixed every light), each light-keyed map with its grey. Marked so it is done once."""
    d = load(folder)
    if d.get("imported") or not folder:
        return ""
    words = []
    lp = os.path.join(folder, "readmap_light.json")
    if os.path.exists(lp):
        try:
            with open(lp) as f:
                for g, m in json.load(f).items():
                    rm = rm_cls.from_json(m)
                    add_session(folder, rm, evidence(rm), grey=float(g), tag="*", kind="imported (readmap_light.json)")
                    words.append("grey %s" % g)
        except (OSError, ValueError, KeyError, TypeError):
            pass
    hp = os.path.join(folder, "readmap.json")
    if os.path.exists(hp) and not words:
        try:
            with open(hp) as f:
                rm = rm_cls.from_json(json.load(f))
            add_session(folder, rm, evidence(rm), grey=None, tag="*", kind="imported (readmap.json)")
            words.append("readmap.json (light not known)")
        except (OSError, ValueError, KeyError, TypeError):
            pass
    import glob as _g
    for tp in sorted(_g.glob(os.path.join(folder, "readmap_tag*.json"))):
        # e13's per-tag maps (10 Nov: tag 4 lost at P11 and P15): each tag's evidence, its light not known
        t = os.path.basename(tp)[len("readmap_tag"):-len(".json")]
        try:
            with open(tp) as f:
                rm = rm_cls.from_json(json.load(f))
            add_session(folder, rm, evidence(rm), grey=None, tag=t, kind="imported (%s)" % os.path.basename(tp))
            words.append("tag %s" % t)
        except (OSError, ValueError, KeyError, TypeError):
            pass
    d = load(folder)
    d["imported"] = time.time()
    save(folder, d)
    return ", ".join(words)


def weight(s, grey, tag):
    """A session's weight for a run in light `grey` with roof tag `tag` (None: any tag)."""
    if grey is None or s.get("grey") is None:
        lw = 1.0 if (grey is None and s.get("grey") is None) else (UNKNOWN_LIGHT_W if s.get("grey") is None else 1.0)
    else:
        lw = math.exp(-0.5 * ((float(grey) - float(s["grey"])) / LIGHT_SCALE) ** 2)
    st = str(s.get("tag", "*"))
    tw = 1.0 if (tag is None or st == "*" or st == str(tag)) else OTHER_TAG_W
    return lw * tw


def choose(folder, rm, grey=None, tag=None, only_tag=False):
    """Add to rm (a seeded observe.ReadMap) the evidence for this light and tag. only_tag: this tag's sessions alone
    (calib_run's per-tag map). Returns words saying what is used and why."""
    d = load(folder)
    ss = d.get("sessions") or []
    if not ss:
        return "no earlier evidence here"
    if d.get("L") and (abs(float(d["L"]) - rm.L) > 5 or abs(float(d["W"]) - rm.W) > 5 or float(d["cell"]) != rm.cell):
        return "the earlier evidence is for another table (%s x %s cm): not used" % (d.get("L"), d.get("W"))
    used = same = other = 0
    wsum = 0.0
    for s in ss:
        if only_tag and str(s.get("tag")) != str(tag):
            continue
        w = weight(s, grey, tag)
        if w < KEPT_APART_W:
            other += 1                    # other light: counts under 10 % (a dim session in lit light: 3 %)
        else:
            used += 1
            wsum += w
            if w > 0.6:
                same += 1
        if w < 1e-3:
            continue
        for i, j, n, r in s.get("cells") or []:
            k = (int(i), int(j))
            rm.tries[k] = rm.tries.get(k, 0) + w * n
            rm.reads[k] = rm.reads.get(k, 0) + w * r
    return ("%s%s: %d session%s at about this light, %d more at lower weight, %d in other light kept apart (each "
            "counts under 10 %%)" % (
        light_words(grey), "" if tag is None else ", tag %s" % tag, same, "" if same == 1 else "s", used - same, other))


def readable_extent(rm, L, ok=None):
    """The furthest x (cm) along the table where the map says tags read (for the Start screen's Area line)."""
    import observe as OB
    ok = OB.READ_OK if ok is None else ok
    xs = [rm.centre(i, j)[0] + rm.cell / 2.0 for i in range(rm.nx) for j in range(rm.ny)
          if rm.rate(*rm.centre(i, j)) >= ok]
    return min(max(xs), L) if xs else 0.0


def grey_of(folder):
    """The overhead camera's median mean grey in a run's bridge/brightness.csv (None if absent)."""
    import csv as _csv
    for p in (os.path.join(folder, "bridge", "brightness.csv"), os.path.join(folder, "brightness.csv")):
        if not os.path.exists(p):
            continue
        try:
            with open(p, newline="") as f:
                v = sorted(float(r["mean_grey"]) for r in _csv.DictReader(f) if r.get("mean_grey") and
                           r.get("camera", "overhead") == "overhead")
            return v[len(v) // 2] if v else None
        except (OSError, ValueError, KeyError):
            return None
    return None


def grey_now(stream="127.0.0.1:8000", timeout=2.0):
    """(real runs) The light now: the median mean grey of a few pictures from the recorder's stream (None if none)."""
    try:
        import camera_discovery as CD
        g = CD.Grabber(stream, timeout=timeout)
        vals = []
        for _ in range(3):
            p = g.grab()
            if p is not None:
                vals.append(float(p.mean()))
        g.close()
        return sorted(vals)[len(vals) // 2] if vals else None
    except Exception:  # noqa: BLE001
        return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["show"])
    ap.add_argument("--grey", type=float, default=None)
    ap.add_argument("--tag", default=None)
    ap.add_argument("--dir", default="")
    a = ap.parse_args(argv)
    import observe as OB
    import sites as SI
    site = SI.load()
    folder = a.dir or site.get("conf_dir") or ""
    d = load(folder)
    t = site.get("table") or {}
    rm = OB.ReadMap(float(d.get("L") or t.get("length_cm") or 396.2), float(d.get("W") or t.get("width_cm") or 167.6))
    print(choose(folder, rm, a.grey, a.tag))
    print("readable to x %.0f cm" % readable_extent(rm, rm.L))
    print(rm.text())
    return 0


if __name__ == "__main__":
    sys.exit(main())
