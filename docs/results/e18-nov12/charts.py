"""e18 report charts from this folder's simulator data. python3 charts.py OUT_DIR"""
import glob, json, math, os, random, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
OUT = sys.argv[1]
BLUE, ORANGE, AQUA, INK, MUTED = "#2a78d6", "#eb6834", "#1baf7a", "#0b0b0b", "#52514e"
plt.rcParams.update({"font.size": 8, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
rng = random.Random(1)


def note(fig, text):
    fig.text(0.01, 0.01, text, fontsize=6.3, color=MUTED)


# 1. the identity batch
rows = [json.loads(l) for l in open(os.path.join(HERE, "outback", "trials.jsonl"))]
fig, ax = plt.subplots(figsize=(8, 3.6))
groups = [(c, m) for c in ("held", "held, never still") for m in ("e18", "e17")]
for k, (c, m) in enumerate(groups):
    R = [r for r in rows if r["cond"] == c and r["mode"] == m]
    col = BLUE if m == "e18" else ORANGE
    for r in R:
        y = r["t_both"] if r["both"] else 48.0
        x = k + rng.uniform(-0.22, 0.22)
        if r["wrong"]:
            ax.scatter([x], [y], marker="x", s=30, color=INK, zorder=3)
        else:
            ax.scatter([x], [y], s=14, color=col, alpha=0.8, edgecolors="none")
    nb = sum(r["both"] for r in R)
    nw = sum(1 for r in R if r["wrong"])
    ax.text(k, 52, "%d of %d told apart\n%d with a wrong name" % (nb, len(R), nw), ha="center", fontsize=6.8, color=INK)
ax.axhline(46, color=MUTED, lw=0.6, ls=":")
ax.text(3.45, 47, "not told apart in 45 s", fontsize=6.5, color=MUTED, ha="right")
ax.set_xticks(range(4))
ax.set_xticklabels(["held\ne18", "held\nout-and-back off", "held, never still\ne18", "held, never still\nout-and-back off"])
ax.set_ylim(0, 58); ax.set_ylabel("seconds until both were named")
ax.set_title("Two robots unaccounted, the home lights camera: time until both are named (30 random trials each)",
             loc="left", color=INK)
note(fig, "Simulated: test rig (tools/fleet/tests/e17_harness.py), outback/batch.py; separation 15-35 cm, own poses 20-50 cm off, "
     "camera noise 0.2-0.8 cm. A cross: one robot named on the other's spot.")
fig.tight_layout(rect=(0, 0.04, 1, 1))
fig.savefig(os.path.join(OUT, "e18-feature-out-and-back-identity-batch-chart.png"), dpi=150)
plt.close(fig)

# 2. the blind excursions: predicted against found
ex = [json.loads(l) for l in open(os.path.join(HERE, "blind", "excursions.jsonl"))]
chk = [r for r in ex if "pred_cm" in r]
fig, axs = plt.subplots(1, 2, figsize=(8, 3.6), sharey=True)
cols = {"good": BLUE, "typical": ORANGE, "poor": AQUA}
marks = {"good": "o", "typical": "s", "poor": "^"}
lim = max(max(r["pred_cm"] for r in chk), max(r["act_cm"] for r in chk), max(r["fixed_pred_cm"] for r in chk)) * 1.1
for ax, key, title in ((axs[0], "pred_cm", "its own record (e18)"), (axs[1], "fixed_pred_cm", "the fixed 29 Oct numbers (e17)")):
    for cls in ("good", "typical", "poor"):
        R = [r for r in chk if r["cls"] == cls]
        inside = sum(1 for r in R if r["act_cm"] <= r[key])
        ax.scatter([r[key] for r in R], [r["act_cm"] for r in R], s=18, marker=marks[cls], color=cols[cls],
                   alpha=0.85, edgecolors="none", label="%s record: %d of %d inside" % (cls, inside, len(R)))
    ax.plot([0, lim], [0, lim], color=MUTED, lw=0.8, ls="--")
    ax.text(lim * 0.97, lim * 0.9, "found = predicted", fontsize=6.5, color=MUTED, ha="right")
    ax.set_xlim(0, lim); ax.set_ylim(0, lim)
    ax.set_xlabel("predicted error, cm (%s)" % title)
    ax.legend(frameon=False, fontsize=6.5, loc="upper left")
axs[0].set_ylabel("error the camera found when it saw the robot again, cm")
fig.suptitle("Blind excursions through a dark patch: the error predicted against the error found", x=0.01, ha="left",
             color=INK, fontsize=9)
note(fig, "Simulated: test rig with a 30 x 35 cm dark patch, blind/batch.py, 3 records x 6 seeds x 5 excursions (only the "
     "allowed ones that came back into view). Below the dashed line = inside the prediction.")
fig.tight_layout(rect=(0, 0.04, 1, 0.95))
fig.savefig(os.path.join(OUT, "e18-feature-blind-excursion-predicted-vs-found-chart.png"), dpi=150)
plt.close(fig)

# 3. what the allowance did, per record
fig, ax = plt.subplots(figsize=(8, 3.2))
for k, cls in enumerate(("good", "typical", "poor")):
    R = [r for r in ex if r["cls"] == cls]
    for r in R:
        x = k + rng.uniform(-0.22, 0.22)
        if not r["ok"]:
            ax.scatter([x], [0], marker="x", s=26, color=INK)
        else:
            ax.scatter([x], [r["depth_cm"] / max(1e-6, r["want_cm"]) * 100], s=16, marker=marks[cls], color=cols[cls],
                       alpha=0.8, edgecolors="none")
    full = sum(1 for r in R if r["ok"] and r["depth_cm"] >= r["want_cm"] - 0.5)
    sh = sum(1 for r in R if r["ok"] and r["depth_cm"] < r["want_cm"] - 0.5)
    ref = sum(1 for r in R if not r["ok"])
    ax.text(k, 112, "%d full, %d shortened, %d refused" % (full, sh, ref), ha="center", fontsize=7, color=INK)
ax.set_xticks(range(3)); ax.set_xticklabels(["good record", "typical record", "poor record"])
ax.set_ylim(-8, 122); ax.set_ylabel("share of the asked depth allowed, %")
ax.set_title("How far each robot's record let it go (30 excursions asked per record; a cross: refused)", loc="left",
             color=INK)
note(fig, "Simulated: test rig, blind/batch.py; each excursion out to a random point in the dark patch and back; the allowance "
     "from odo_trust.py (its predicted error + body + stop inside the board, away from the others, back where the camera reads).")
fig.tight_layout(rect=(0, 0.05, 1, 1))
fig.savefig(os.path.join(OUT, "e18-feature-blind-allowance-by-record-chart.png"), dpi=150)
plt.close(fig)

# 4. the e17 comparison re-run
e17 = os.path.join(REPO, "docs", "results", "e17-nov12", "stress")
fig, axs = plt.subplots(1, 2, figsize=(8, 3.4))
for ax, key, lab in ((axs[0], "min_gap_cm", "closest the bodies came, cm"), (axs[1], "longest", "longest stand of any robot, s")):
    for k, (d, who) in enumerate([(d, w) for d in ("0.45", "1.0") for w in ("new", "e18")]):
        fs = sorted(glob.glob(os.path.join(e17 if who == "new" else os.path.join(HERE, "stress"),
                                           "%s-*-%s.json" % (who, d))))
        vals = []
        for f in fs:
            r = json.load(open(f))
            vals.append(r["min_gap_cm"] if key == "min_gap_cm" else max(r["longest_still"].values()))
        col = ORANGE if who == "new" else BLUE
        ax.scatter([k + rng.uniform(-0.15, 0.15) for _ in vals], vals, s=16, color=col, edgecolors="none")
        if vals:
            m = sorted(vals)[len(vals) // 2]
            ax.plot([k - 0.28, k + 0.28], [m, m], color=INK, lw=1.5)
    ax.set_xticks(range(4)); ax.set_xticklabels(["0.45 s late\ne17", "0.45 s late\ne18", "1.0 s late\ne17",
                                                "1.0 s late\ne18"])
    ax.set_ylabel(lab)
    if key == "longest":
        ax.set_yscale("log")
axs[0].set_title("The e17 simulator comparison re-run on e18 (6 runs of 30 min each)", loc="left", color=INK)
note(fig, "Simulated: freeroam_fastsim (3 robots, camera sightings through the tracking layer, edge physics, heading faults), "
     "stress/run1.py unchanged from e17; e17's own runs from docs/results/e17-nov12/stress. The line: the median.")
fig.tight_layout(rect=(0, 0.05, 1, 1))
fig.savefig(os.path.join(OUT, "e18-feature-e17-comparison-rerun-chart.png"), dpi=150)
plt.close(fig)
print("ok")
