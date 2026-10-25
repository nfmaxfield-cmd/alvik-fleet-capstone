"""The sensor-trust fix (2026-10-25), logs only: the 2026-10-24/25 simulator grid's evidence replayed through the new
e02_trust, in time order. Per grid session: the trust file's items (camera, stretches, front) plus each run's wheel
scale from the session's fit_data (e02_state.json), which the old runner did not record.

    python3 tools/experiment/analyses_oct25/trust_fix_replay.py [/tmp/claude-0/grid]

Two ways per session: "cold" (no history, as the grid ran) and "trained" (the healthy baseline session first, as the
robots' own history: a sensor swapped on a trained system). Prints a markdown table: the flags the old code raised
(from the grid log) and the new code's."""
import glob
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(EXP))
sys.path.insert(0, EXP)
sys.path.insert(0, os.path.join(EXP, "analyses_oct24"))

import e02 as E  # noqa: E402
import e02_trust as TR  # noqa: E402
import grid_summary as GS  # noqa: E402

ROBOTS = ["Left", "Center", "Right"]


def session_dir(txt):
    m = re.search(r"logs/(e02-[0-9-]+dryrun)", open(txt, errors="replace").read())
    return os.path.join(REPO, "logs", m.group(1)) if m else None


def evidence(name, folder):
    """[(t, kind, args)] in time order for one grid session."""
    ev = []
    tp = os.path.join(folder, "trust-%s.json" % name)
    if os.path.exists(tp):
        for it in json.load(open(tp))["items"]:
            ev.append((it["t"], "item", it))
    d = session_dir(os.path.join(folder, name + ".txt"))
    if d and os.path.exists(os.path.join(d, "e02_state.json")):
        st = json.load(open(os.path.join(d, "e02_state.json")))
        for robot, obs in (st.get("fit_data") or {}).items():
            tm = {}
            for o in obs:
                tm[o.get("run")] = max(tm.get(o.get("run"), 0.0), o.get("tm") or 0.0)
            for run, sc, ln, pairs in E.run_scales(obs):
                ev.append((tm.get(run, 0.0) + 1.0, "run", (robot, sc, ln, pairs)))
    return sorted(ev, key=lambda e: e[0])


def feed(t, ev, robots=ROBOTS):
    """Feed evidence in order, checking the flags after each item as the runner does. Returns the changes."""
    out = []
    for i, (tt, kind, a) in enumerate(ev):
        if kind == "item":
            it = dict(a)
            it["session"] = t.session
            it["w"] = a["w"]
            t.items.append(it)
        else:
            t.run_scale(*a[:3], pairs=a[3]) if len(a) > 3 else t.run_scale(*a)
        if i + 1 < len(ev) and ev[i + 1][0] == tt:
            continue                     # the runner checks after a whole piece of evidence (x and y, wheels and gyro)
        for s, r, what, k in t.update_flags(robots):
            out.append((s, r, what, k, t.reason(s, r) if what == "degraded" else ""))
    return out


def replay(name, folder, base_ev=None):
    t = TR.SensorTrust()
    t.session = 1
    if base_ev is not None:
        feed(t, base_ev)
        t.new_session()
    return t, feed(t, evidence(name, folder))


def words(changes):
    who = lambda s, r: "camera" if r == "*" else "%s %s" % (r, s)
    fl = [c for c in changes if c[2] == "degraded"]
    return "; ".join("%s %.1f" % (who(s, r), k) for s, r, _, k, _ in fl) or "none"


def main(folder):
    names = [os.path.basename(p)[:-4] for p in sorted(glob.glob(os.path.join(folder, "*.txt")))
             if not p.endswith(("progress.txt", "done.txt", "specs.txt"))]
    base_ev = evidence("baseline", folder)
    print("| Session | Old flags (grid log) | New, cold start | New, trained (baseline as history) |")
    print("|---|---|---|---|")
    detail = []
    for n in names:
        old = GS.one(os.path.join(folder, n + ".txt"))
        oldw = "; ".join("%s %.1f" % (w, k) for w, s, k, _ in old["flags"] if s == "bad") or "none"
        t1, c1 = replay(n, folder)
        t2, c2 = replay(n, folder, base_ev) if n != "baseline" else (None, None)
        print("| %s | %s | %s | %s |" % (n, oldw, words(c1), words(c2) if c2 is not None else "(is the history)"))
        detail.append((n, t1, c1, t2, c2))
    print()
    for n, t1, c1, t2, c2 in detail:
        print("%s:" % n)
        for lab, t, c in (("cold", t1, c1), ("trained", t2, c2)):
            if t is None:
                continue
            for s, r, what, k, why in c:
                if what == "degraded":
                    print("    %s: %s/%s %.1f: %s" % (lab, s, r, k, why))
            for line in t.summary(ROBOTS):
                print("    %s   %s" % (lab, line))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/claude-0/grid")
