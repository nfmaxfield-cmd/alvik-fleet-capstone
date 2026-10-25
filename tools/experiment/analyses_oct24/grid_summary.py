"""The bad-sensor grid in the simulator (2026-10-24 night): one session per degradation, read from its runner log.

    python3 tools/experiment/analyses_oct24/grid_summary.py /tmp/claude-0/grid

For each session: runs done, holds (by kind), the sensors flagged and when (in runs done before the flag), the trust
summary at the end, and each condition's mean error at the start point and the far point (scored by the true camera
in a degraded-camera session: e02.py score_hist)."""
import glob
import os
import re
import sys

COND = re.compile(r"\]   (U|C|K|KC) +\(.*?\): (\d+) runs, end error mean ([\d.]+) cm, far point mean ([\d.]+|-) cm")


def one(path):
    text = open(path, errors="replace").read().splitlines()
    runs = 0
    out = dict(name=os.path.basename(path)[:-4], runs=0, holds={}, flags=[], trust=[], cond={}, ended=False)
    in_trust = False
    for line in text:
        if "=== Run " in line:
            runs += 1
        if "Result:" in line:
            out["runs"] += 1
        if "HOLD: every robot is stopped." in line:
            why = line.split("HOLD: every robot is stopped.")[1].strip()
            kind = ("border (robot's own)" if "own border" in why else "border (camera)" if "open edge" in why
                    else "front sensor" if "front sensor" in why else "lost / camera-wheels")
            out["holds"][kind] = out["holds"].get(kind, 0) + 1
        m = re.search(r"SENSOR: (.*?) looks (bad|healthy again): ([\d.]+) x", line)
        if m:
            out["flags"].append((m.group(1), m.group(2), float(m.group(3)), out["runs"]))
        if "Sensor trust (learned" in line:
            in_trust = True
            continue
        if in_trust:
            if line.startswith("[") and line[11:15] == "    ":
                out["trust"].append(line[15:].strip())
                continue
            in_trust = False
        m = COND.search(line)
        if m:
            out["cond"][m.group(1)] = (int(m.group(2)), float(m.group(3)),
                                       None if m.group(4) == "-" else float(m.group(4)))
        if "=== Session:" in line:
            out["ended"] = True
    return out


def main(folder):
    rows = [one(p) for p in sorted(glob.glob(os.path.join(folder, "*.txt"))) if not p.endswith(("progress.txt",
                                                                                                 "done.txt", "specs.txt"))]
    print("| Session | Runs scored | Holds | Sensors flagged (k, after runs) | C back at start | KC back at start | KC far |")
    print("|---|---|---|---|---|---|---|")
    for r in rows:
        holds = ", ".join("%s %d" % kv for kv in sorted(r["holds"].items())) or "none"
        fl = "; ".join("%s %s %.1f (%d)" % (w, "BAD" if s == "bad" else "ok again", k, n) for w, s, k, n in r["flags"]) or "none"
        c = r["cond"].get("C")
        kc = r["cond"].get("KC")
        print("| %s | %d%s | %s | %s | %s | %s | %s |" % (
            r["name"], r["runs"], "" if r["ended"] else " (did not finish)", holds, fl,
            "%.1f cm" % c[1] if c else "-", "%.1f cm" % kc[1] if kc else "-",
            "%.1f cm" % kc[2] if kc and kc[2] is not None else "-"))
    print()
    for r in rows:
        print("%s trust at the end:" % r["name"])
        for t in r["trust"]:
            print("    " + t)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/claude-0/grid")
