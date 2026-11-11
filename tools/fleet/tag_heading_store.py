#!/usr/bin/env python3
"""Each roof tag's measured turn on its robot, remembered across runs (build e15, 11 Nov 2026).

The school, 10 Nov 18:54 (code 02375cd): AL-1DC0's roof tag 2 is stuck on turned 180 deg. On 9 Nov the start's heading
check found it ("tag heading offset -180 deg"); on 10 Nov the camera lost the tag during the heading check, so the run
started on the RAW tag heading: the facing check said the robot faced backwards and turned it, the camera and its own
pose disagreed by 18 cm, and even after run_plan re-learned the offset (-180) the facing re-checks read the raw tag again
and turned it back. Each run forgot what the runs before it had measured.

Now every place that measures a tag's turn writes it here, per site (<conf_dir>/tag_headings.json) and per tag id:

    {"2": {"offset_deg": -180.0, "n": 3, "first_seen": ..., "last_seen": ..., "how": "heading check",
           "robot": "AL-1DC0", "changes": [{"from": 0.0, "to": -180.0, "when": ..., "how": ...}]}}

offset_deg is what is ADDED to the tag's camera heading to get the robot's nose (the convention of observe.heading_offset,
the tag bridge's yaw_off and run_plan's yaw_fix). n counts the measurements that agreed with it in a row.

Observe, then trust: a remembered offset is only a run's STARTING GUESS. The run checks it again (the binding's heading
check, run_plan's start crawl when a big turn would follow from it, the heading watch on every straight stretch, the
calibration's heading check and watch, free roam's proof drive). When a new measurement disagrees, the evidence wins:
the entry takes the new value, n starts again at 1, and the change is kept in "changes" (a tag re-mounted).

Switch: MIRLT_TAG_HEADING_MEMORY=0 (or the site's tags.heading_memory false): per run only, as before e15 (nothing read
or written here).

    python3 tools/fleet/tag_heading_store.py show [--file PATH]      what is remembered
"""
import argparse
import datetime as dt
import json
import os
import sys

VERSION = "tag-heading-store-2026-11-11e15"
FILE = "tag_headings.json"
SAME_DEG = 30.0          # two measurements within this are the same turn. Not a tuned number: offsets are snapped to a
                         # quarter turn (observe.SNAP_DEG 90, SNAP_TOL_DEG 20), so two right answers differ by under
                         # 2 x 20 = 40 only in the unsnapped case, and two different mountings by 90 or more; 30 sits
                         # between the measured heading noise (1-3 deg) and a quarter turn. No DOE: a definition
CHANGES_KEPT = 20        # the history of changes kept per tag. No DOE: a log length


def wrap180(a):
    return (float(a) + 180.0) % 360.0 - 180.0


def enabled(site=None):
    """False when the per-run-only behaviour (before e15) is asked for."""
    if os.environ.get("MIRLT_TAG_HEADING_MEMORY", "1").strip().lower() in ("0", "off", "no", "false"):
        return False
    tg = ((site or {}).get("tags") or {})
    return bool(tg.get("heading_memory", True))


def path_for(site=None, state_dir=None):
    """The store's file: a given state folder (a dry run's own), else the site's conf_dir. "" when there is neither."""
    d = state_dir if state_dir is not None else ((site or {}).get("conf_dir") or "")
    return os.path.join(d, FILE) if d else ""


def now_iso():
    return dt.datetime.now().isoformat(timespec="seconds")


def load(path):
    if not path:
        return {}
    try:
        with open(path) as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def save(path, d):
    if not path:
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(d, f, indent=1, sort_keys=True)
    os.replace(tmp, path)


def get(path, tag):
    """The remembered entry for a tag id (dict) or None."""
    if tag is None:
        return None
    e = load(path).get(str(int(tag)))
    return e if isinstance(e, dict) and e.get("offset_deg") is not None else None


def record(path, tag, offset, how, robot=None, when=None):
    """A measured offset for a tag: the evidence wins. Returns {"result": "new" | "agrees" | "changed", "entry": ...,
    "old": previous offset or None, "words": plain words}. Nothing written when path is ""."""
    if tag is None or offset is None:
        return {"result": "none", "entry": None, "old": None, "words": ""}
    when = when or now_iso()
    off = round(wrap180(offset), 1)
    d = load(path)
    k = str(int(tag))
    e = dict(d.get(k) or {})
    old = e.get("offset_deg")
    if old is None:
        e = {"offset_deg": off, "n": 1, "first_seen": when, "last_seen": when, "how": how, "robot": robot,
             "changes": []}
        res = "new"
        words = "TAG HEADING REMEMBERED: tag %s is turned %+.0f deg on its robot (%s); kept for the next runs" % (
            k, off, how)
    elif abs(wrap180(off - float(old))) <= SAME_DEG:
        e["n"] = int(e.get("n") or 0) + 1
        e["last_seen"], e["how"] = when, how
        if robot:
            e["robot"] = robot
        res = "agrees"
        words = "TAG HEADING CONFIRMED: tag %s %+.0f deg on its robot (%s; %d measurements in a row agree)" % (
            k, float(old), how, e["n"])
    else:
        ch = list(e.get("changes") or [])
        ch.append({"from": float(old), "to": off, "when": when, "how": how, "robot": robot, "n_before": e.get("n")})
        e.update(offset_deg=off, n=1, last_seen=when, how=how, changes=ch[-CHANGES_KEPT:])
        if robot:
            e["robot"] = robot
        res = "changed"
        words = ("TAG HEADING CHANGED: tag %s was remembered at %+.0f deg (%s measurement%s) but is now measured at "
                 "%+.0f deg (%s): the evidence wins, the memory is updated (a tag re-mounted or moved to another "
                 "robot?)" % (k, float(old), d.get(k, {}).get("n"), "" if d.get(k, {}).get("n") == 1 else "s",
                              off, how))
    d[k] = e
    save(path, d)
    return {"result": res, "entry": e, "old": old, "words": words}


def words_of(tag, e):
    """'tag 2 turned -180 deg (3 measurements, last 2026-11-10T19:04, heading check)'."""
    if not e:
        return "tag %s: nothing remembered" % tag
    return "tag %s turned %+.0f deg (%d measurement%s in a row, last %s, %s)" % (
        tag, float(e["offset_deg"]), int(e.get("n") or 0), "" if int(e.get("n") or 0) == 1 else "s",
        e.get("last_seen"), e.get("how"))


def harvest_discovery(path, discovery_json, tags):
    """(free roam) start_discovery's proof drive measured each robot's tag against its travel (robots[n].tag_off, on
    the tag bridge's corrected headings): the total (the bridge's yaw_off + tag_off, snapped) recorded per tag id.
    tags: the run's tags block (ids by robot name, yaw_off by tag id). Returns the record() results."""
    import observe as OB
    try:
        with open(discovery_json) as f:
            res = json.load(f)
    except (OSError, ValueError):
        return []
    out = []
    ids = (tags or {}).get("ids") or {}
    yo = (tags or {}).get("yaw_off") or {}
    for n, rob in (res.get("robots") or {}).items():
        t = ids.get(n)
        off = (rob or {}).get("tag_off")
        if t is None or off is None:
            continue
        total = OB.snap(wrap180(float(yo.get(str(t), yo.get(t, 0.0)) or 0.0) + float(off)))
        out.append(record(path, t, total, "free roam's proof drive", robot=n))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cmd", choices=["show"])
    ap.add_argument("--file", default="", help="the store (default: the site in use's conf_dir)")
    a = ap.parse_args(argv)
    p = a.file
    if not p:
        try:
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            import sites
            p = path_for(sites.load())
        except Exception:  # noqa: BLE001
            p = ""
    d = load(p)
    print("Tag headings remembered in %s:" % (p or "(no site folder)"))
    if not d:
        print("  nothing yet")
    for k in sorted(d, key=lambda s: int(s) if s.isdigit() else 0):
        print("  " + words_of(k, d[k]))
        for c in (d[k].get("changes") or [])[-3:]:
            print("    changed %+.0f -> %+.0f on %s (%s)" % (c["from"], c["to"], c["when"], c["how"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
