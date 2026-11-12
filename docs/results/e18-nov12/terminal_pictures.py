"""e18: terminal-style pictures from the test rig (tools/fleet/tests/e17_harness.py) and the mission coordinator.
python3 terminal_pictures.py OUT_DIR"""
import math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(REPO, "tools", "fleet", "tests")); sys.path.insert(0, os.path.join(REPO, "tools", "fleet"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import e17_harness as H
import test_e18_safety_blind as T


def pic(lines, path, title):
    fig = plt.figure(figsize=(11, 0.22 * len(lines) + 0.6))
    fig.patch.set_facecolor("#1a1a19")
    fig.text(0.01, 0.98, title, color="#f0efec", family="monospace", fontsize=8.5, va="top", weight="bold")
    for k, l in enumerate(lines):
        fig.text(0.01, 0.98 - (k + 1.6) / (len(lines) + 2.2), l[:165], color="#c3c2b7", family="monospace",
                 fontsize=7.3, va="top")
    fig.savefig(path, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)


def ev(R, names, kinds=None, t1=1e9):
    return ["[%5.1f s] %-6s %s: %s" % (t, n, k, txt) for t, n, k, txt in R.events
            if n in names and (kinds is None or k in kinds) and t <= t1]


OUT = sys.argv[1]
# 1. the out-and-back
R = T._two_unaccounted(T.TWO, secs=12.0)
lines = ["Left and Right both lost, standing 20 cm apart; each own pose 30 cm off; the home lights camera (a turn is invisible)"]
lines += ["[%5.1f s] layer  %s: %s" % (l["t"], l["what"], l["why"]) for l in R.logs if l.get("robot") in ("Left", "Right")
          and any(w in str(l.get("what")) for w in ("asked", "named", "nothing", "unclear"))]
lines += ev(R, ("Left", "Right"), ("identity wiggle", "identity drive", "CORRECTED"))
lines.append("after 12 s: Left %.1f cm and Right %.1f cm from where they really stand (no swap)" % (R.gap("Left"), R.gap("Right")))
pic(lines, os.path.join(OUT, "e18-feature-out-and-back-identity-terminal.png"),
    "Simulated (test rig): two robots unaccounted, the wiggle cannot tell, the out-and-back does (e18 decision 1)")
# 2. heading in doubt beside a known / an unknown neighbour
lines = []
for lab, kw in (("Center 16 cm away, the camera has it", dict(gap=16.0)),
                ("Center 24 cm away, the camera has it", dict(gap=24.0)),
                ("Center 18 cm away on a dark patch (the camera does not have it)", dict(gap=18.0, neigh_lost=True))):
    R, r = T._doubt_rig(**kw)
    R.run(8.0, hook=T._hold_others)
    lines.append("== Left's heading in doubt; %s ==" % lab)
    lines += ev(R, ("Left",), ("waits for room", "creeps", "heading check waits"))[:4]
    rows = [l for l in R.logs if l.get("table") == "creep" and l["robot"] == "Left"][:3]
    lines += ["          creep %.1f cm at %.1f cm/s (%s)" % (q["L_cm"], q["v"], q.get("doubt") or "normal") for q in rows]
    lines.append("          after 8 s the middles are %.1f cm apart (bodies touch under 13.4)" % math.dist(
        R.truth["Left"][:2], R.truth["Center"][:2]))
    lines.append("")
pic(lines, os.path.join(OUT, "e18-feature-heading-doubt-beside-neighbour-terminal.png"),
    "Simulated (test rig): a doubtful heading beside a known neighbour waits or creeps straight away; beside an unknown one, cautiously")
# 3. a lost robot first
R = H.Rig({"Left": (95.0, 60.0, 180.0), "Center": (62.0, 60.0, 0.0), "Right": (20.0, 110.0, 0.0)}, never_still=True)
R.c.ws = R.c.rect = T._readable_ws()
R.run(40.0)
lines = ["The camera reads x < 70 cm only. Left is outside it at x = 95; Center stands on its way back (x = 62)."]
lines += ev(R, ("Left", "Center"), ("makes room", "creeps", "stray return"))[:10]
lines.append("after 40 s: Left at x = %.0f (inside the readable area: %s); Center at (%.0f, %.0f)" % (
    R.truth["Left"][0], R.truth["Left"][0] < 70, R.truth["Center"][0], R.truth["Center"][1]))
pic(lines, os.path.join(OUT, "e18-feature-lost-robot-first-others-make-room-terminal.png"),
    "Simulated (test rig): a robot outside the readable area goes first; the robot standing in its way makes room")
# 4. never still in a mission
import run_plan as RP
RP.NEVER_STILL = True
RPm, c, b = T._held_coord()
t = [RP.now()]
RP.now = lambda: t[0]
for k in range(40):
    t[0] += 0.5
    b.last_heard = t[0]
    n0 = len(c.sent)
    c.held_creep_watch()
    for _, txt in c.sent[n0:]:
        if txt.startswith("fs ") and " D " in txt:
            q = txt.split()
            b.pose = (float(q[5]), float(q[6]), float(q[7]))
lines = ["Mission (run_plan, keep going): Alpha held after a blind stop at (80, 120); it still hears. 20 s:"]
lines += ["  " + x for x in c.said if "creeps" in x or "held" in x][:4]
lines += ["  creep %.1f cm at %.1f cm/s from (%.1f, %.1f) to (%.1f, %.1f), toward the %s" % (
    kv["L_cm"], kv["v"], kv["x"], kv["y"], kv["to"][0], kv["to"][1], kv["toward"]) for e_, kv in c.logged
          if e_ == "held_creep"][:10]
lines.append("  its held outline for the others' plans is grown by %.0f cm (its creep reach): %.1f cm net" % (
    RP.CREEP_REACH_CM, c.out_obstacle(b)["gap_cm"]))
pic(lines, os.path.join(OUT, "e18-feature-mission-held-robot-creeps-terminal.png"),
    "Mission coordinator (run_plan): a held robot never stands still (e18 decision 2), inside its own held outline")
# 5. blind excursions
lines = []
for lab, odo in (("good record (wheel 0.3 %, gyro 0 %)", (0.003, 0.0, 0.005)),
                 ("poor record (wheel 5 %, gyro 3 %, noisy)", (0.05, 0.03, 0.05))):
    R, r = T._dark_rig(odo)
    tr = R.c.trust["Left"]
    res = R.c.blind_excursion(r, (85.0, 62.0))
    R.run(30.0)
    lines.append("== %s: %s ==" % (lab, tr.words()))
    lines += ev(R, ("Left",), ("blind leg", "blind leg refused", "blind check"), t1=1e9)[-3:]
    lines.append("")
pic(lines, os.path.join(OUT, "e18-feature-blind-excursion-good-vs-poor-record-terminal.png"),
    "Simulated (test rig, a 30 x 35 cm dark patch): the same excursion asked of a good and a poor odometry record")
print("ok")
