"""e17: two terminal-style pictures from the test rig (tools/fleet/tests/e17_harness.py). python3 terminal_pictures.py OUT"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(REPO, "tools", "fleet", "tests")); sys.path.insert(0, os.path.join(REPO, "tools", "fleet"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import e17_harness as H


def pic(lines, path, title):
    fig = plt.figure(figsize=(10, 0.22 * len(lines) + 0.6))
    fig.patch.set_facecolor("#1a1a19")
    fig.text(0.01, 0.98, title, color="#f0efec", family="monospace", fontsize=8.5, va="top", weight="bold")
    for k, l in enumerate(lines):
        fig.text(0.01, 0.98 - (k + 1.6) / (len(lines) + 2.2), l[:150], color="#c3c2b7", family="monospace",
                 fontsize=7.5, va="top")
    fig.savefig(path, dpi=150, facecolor=fig.get_facecolor())


OUT = sys.argv[1]
lines = []
for lab, elim in (("BEFORE e17 (the layer's gate only)", False), ("e17 (observe, then trust)", True)):
    R = H.Rig({"Left": (31.2, 64.2, -115.0), "Center": (95.0, 30.0, 90.0), "Right": (95.0, 70.0, 180.0)},
              own_off={"Left": (66.1 - 31.2, 97.9 - 64.2, 60.0)}, eliminate=elim, gate_record=elim)
    R.run(0.1)
    R.c.start_lost(R.c.R["Left"], "the camera lost it (as at 11:39:15)")
    R.run(20.0)
    lines.append("== %s ==" % lab)
    for t, n, k, txt in R.events:
        if n == "Left" and k not in ("motion",):
            lines.append("[%5.1f s] %-5s %s: %s" % (t, n, k, txt))
    lines.append("   after 20 s: the run's position for Left is %.1f cm from where it really stands" % R.gap("Left"))
    lines.append("")
pic(lines, os.path.join(OUT, "e17-feature-left-1139-corrected-from-the-camera-terminal.png"),
    "Simulated: Left standing at (31, 64), its own pose at (66, 98) as at 11:39 on 11 Nov (test rig, e17_harness.py)")
R = H.Rig({"Left": (60.0, 113.0, 90.0), "Center": (100.0, 30.0, 90.0), "Right": (20.0, 30.0, 180.0)}, never_still=True)
R.run(2.0)
R.c.R["Left"].busy_until = 0.0
R.c.R["Left"].unsure = dict(t=R.t, why="unsure near an edge", tried=0)
R.run(15.0)
lines = ["[%5.1f s] creep %.1f cm at %.1f cm/s from (%.0f, %.0f) toward (%.0f, %.0f), its doubt %.1f cm" % (
    l["t"], l["L_cm"], l["v"], l["x"], l["y"], l["tx"], l["ty"], l["unc_cm"]) for l in R.logs
         if l.get("table") == "creep" and l["robot"] == "Left"]
lines.append("Left's longest stand after it was held: %.1f s (the derived pause: %.2f s); y from 113 to %.0f cm" % (
    R.longest_still("Left", 3.0), R.c.still_pause(R.c.R["Left"]), R.truth["Left"][1]))
pic(lines, os.path.join(OUT, "e17-feature-never-still-creep-terminal.png"),
    "Simulated: Left held unsure 9 cm from the y = 122 edge, MIRLT_NEVER_STILL on (Nick 12 Nov): it creeps (test rig)")
print("ok")
