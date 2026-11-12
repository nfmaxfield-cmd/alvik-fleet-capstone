"""e18 problem found in the rig: a robot's own heading error (true) over a 150 s roam, with fixes fused against its pose
NOW (the e17 rig) and against its pose AT THE PICTURE'S TIME (as the firmware's pose.h; the e18 rig).
python3 heading_problem.py OUT_DIR"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(REPO, "tools", "fleet", "tests")); sys.path.insert(0, os.path.join(REPO, "tools", "fleet"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import e17_harness as H
import proven_workspace as PW
BLUE, ORANGE, INK, MUTED = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e"
plt.rcParams.update({"font.size": 8, "axes.spines.top": False, "axes.spines.right": False, "axes.edgecolor": MUTED})
fig, ax = plt.subplots(figsize=(8, 3.2))
for lab, ft, col in (("fixes fused against its pose now (e17 rig)", False, ORANGE),
                     ("fused at the picture's time (e18 rig, as pose.h)", True, BLUE)):
    ws = PW.ProvenWorkspace(H.BOARD); ws.see(H.BOARD)
    R = H.Rig({"Left": (40.0, 60.0, 0.0), "Center": (100.0, 105.0, 90.0), "Right": (20.0, 15.0, 180.0)},
              odo={"Left": (0.003, 0.0, 0.005)}, workspace=ws, raw=True, seed=1, fuse_then=ft)
    R.run(0.5); R.c.R["Left"].busy_until = 0.0
    T, Y = [], []
    def hook(R_):
        if int(R_.t * 4) != int((R_.t - 0.05) * 4):
            T.append(R_.t); Y.append(abs(R_.off["Left"][2]))
    R.run(150.0, hook=hook)
    resets = sum(1 for e in R.events if e[1] == "Left" and "heading re-set" in e[3])
    ax.plot(T, Y, color=col, lw=1.6, label="%s: %d heading re-sets" % (lab, resets))
ax.set_ylabel("Left's own heading error, deg"); ax.set_xlabel("seconds of free roam")
ax.legend(frameon=False, loc="upper left")
ax.set_title("A robot whose gyro is perfect: its own heading error in the test rig", loc="left", color=INK)
fig.text(0.01, 0.01, "Simulated: test rig (e17_harness.py), 150 s of free roam, fixes 0.45 s late; own heading minus the true "
         "heading. Each jump: an off-plan heading re-set from the camera's course.",
         fontsize=6.3, color=MUTED)
fig.tight_layout(rect=(0, 0.05, 1, 1))
fig.savefig(os.path.join(sys.argv[1], "e18-problem-heading-resets-from-course-in-the-rig-chart.png"), dpi=150)
print("ok")
