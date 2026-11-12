"""e17 report charts. python3 charts.py RUN_0826 RUN_0858 OUT_DIR [STRESS_DIR]
RUN_*: the unpacked home run folders (logs). Writes the PNGs named in the e17 report."""
import datetime, glob, json, math, os, subprocess, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.dates as md
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(REPO, "tools", "analysis")); sys.path.insert(0, os.path.join(REPO, "tools", "fleet"))
import layer_replay as LR, tracking_layer as TL
R1, R2, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
BLUE, ORANGE, INK, MUTED = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e"
plt.rcParams.update({"font.size": 8, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
D = lambda t: datetime.datetime.fromtimestamp(t)      # noqa: E731

# 1. the 4-hour run, Left: seconds since the layer last named it, while the bridge saw it
def since(named, seen, t0, t1, step=5.0):
    import bisect
    T, Y = [], []
    t = t0
    while t <= t1:
        i = bisect.bisect_right(named, t) - 1
        last = named[i] if i >= 0 else t0
        T.append(D(t)); Y.append(max(0.1, t - last)); t += step
    return T, Y
fig, ax = plt.subplots(figsize=(8, 3.4))
for lab, rule, e17, col in (("before e17 (as the run had it)", "speed", False, ORANGE), ("e17", "velocity", True, BLUE)):
    TL.OWN_RESET_RULE = rule
    d = LR.replay(R2, lag_s=0.0, e17=e17, at_pictures=True, keep_times=True)
    nm, sb = sorted(d["named_t"]["Left"]), sorted(d["bridge_t"]["Left"])
    T, Y = since(nm, sb, 1794414600, sb[-1])
    ax.plot(T, Y, color=col, lw=2, label="%s: longest %.0f s here" % (lab, max(Y)))
ax.set_yscale("log"); ax.set_ylabel("seconds since Left was last named")
ax.xaxis.set_major_formatter(md.DateFormatter("%H:%M")); ax.legend(frameon=False, loc="upper left")
ax.set_title("Home 11 Nov 08:58 run, Left, replayed through the tracking layer", loc="left", color=INK)
fig.text(0.01, 0.01, "Measured: the run's own camera sightings (bridge/track.csv) and Left's own poses (follow.csv), fed to "
         "tracking_layer.py once a picture as the Pi did; replay by tools/analysis/layer_replay.py.", fontsize=6.3, color=MUTED)
fig.tight_layout(rect=(0, 0.04, 1, 1)); fig.savefig(os.path.join(OUT, "e17-problem-and-fix-left-84-min-unnamed-replay-chart.png"), dpi=150)

# 2. the heading jump when a re-seed lands, standing vs turning
fig, ax = plt.subplots(figsize=(8, 3.2))
import random
rng = random.Random(1)
groups = []
for run, lab in ((R1, "08:26"), (R2, "08:58")):
    js = os.path.join("/tmp", "e17_reseed_%s.json" % lab.replace(":", ""))
    subprocess.run([sys.executable, "-I", os.path.join(HERE, "reseed_heading.py"), run, js], check=True,
                   stdout=subprocess.DEVNULL)
    rows = json.load(open(js))
    groups.append(("%s standing" % lab, [abs(o["disc"]) for o in rows if abs(o["rate"]) <= 5]))
    groups.append(("%s turning" % lab, [abs(o["disc"]) for o in rows if abs(o["rate"]) > 20]))
for k, (lab, v) in enumerate(groups):
    col = BLUE if "standing" in lab else ORANGE
    ax.scatter([k + rng.uniform(-0.18, 0.18) for _ in v], v, s=10, color=col, alpha=0.7, edgecolors="none")
    m = sorted(v)[len(v) // 2]
    ax.plot([k - 0.3, k + 0.3], [m, m], color=INK, lw=2)
    ax.text(k + 0.32, m, "median %.1f (n %d)" % (m, len(v)), va="center", fontsize=7, color=INK)
ax.set_xticks(range(len(groups))); ax.set_xticklabels([g[0] for g in groups]); ax.set_xlim(-0.5, len(groups) + 0.3)
ax.set_ylabel("heading jump when the re-seed landed (deg)"); ax.set_yscale("symlog", linthresh=5); ax.set_ylim(0, 150)
ax.set_title("Re-seeds (fpose) landing while the robot stood vs while it turned", loc="left", color=INK)
fig.text(0.01, 0.01, "Measured: each robot's own heading (follow.csv) around every re-seed the run sent (pose.csv, events.csv), "
         "less its turn rate; turning = over 20 deg/s when sent. Script reseed_heading.py.", fontsize=6.3, color=MUTED)
fig.tight_layout(rect=(0, 0.05, 1, 1)); fig.savefig(os.path.join(OUT, "e17-problem-reseed-mid-turn-heading-jump-chart.png"), dpi=150)

# 3. curve ratios per robot and run (discover, don't pre-map)
sys.path.insert(0, os.path.join(REPO, "tools", "analysis"))
import drift_ratios as DR
fig, ax = plt.subplots(figsize=(8, 3.0))
names = ["Left", "Center", "Right"]
for j, (run, lab, col) in enumerate(((R1, "08:26 run", ORANGE), (R2, "08:58 run", BLUE))):
    rr = DR.ratios(run)
    for i, n in enumerate(names):
        s = DR.summary(rr.get(n, {}).get("curve_k", []))
        if s.get("n"):
            x = i + (j - 0.5) * 0.25
            ax.errorbar(x, s["median"], yerr=s["ci_half"], fmt="o", color=col, ms=6, capsize=3,
                        label=lab if i == 0 else None)
            ax.text(x + 0.05, s["median"], " n %d" % s["n"], fontsize=7, va="center", color=MUTED)
ax.axhline(1.0, color=MUTED, lw=0.8); ax.axhspan(1 - 0.03, 1 + 0.03, color="#e8e8e6", zorder=0)
ax.set_xticks(range(3)); ax.set_xticklabels(names); ax.set_ylabel("camera curvature / asked")
ax.legend(frameon=False); ax.set_title("Each robot's curve pieces: how much more or less it turned than asked", loc="left", color=INK)
fig.text(0.01, 0.01, "Measured: the runs' own curve fits (arcfit.csv); dot = median, bar = its 95 % half-width; grey band = "
         "within 3 % (the save rule's spread). Script tools/analysis/drift_ratios.py.", fontsize=6.3, color=MUTED)
fig.tight_layout(rect=(0, 0.05, 1, 1)); fig.savefig(os.path.join(OUT, "e17-feature-curve-ratios-per-robot-chart.png"), dpi=150)

# 4. the simulator: e17 vs everything e17 switched off
ONLY = os.environ.get("E17_CHART_ONLY", "")
if len(sys.argv) > 4:
    S = sys.argv[4]
    fig, axs = plt.subplots(1, 2, figsize=(8, 3.0))
    for k, (key, lab) in enumerate((("longest_still", "longest stand of any robot (s)"), ("min_gap_cm", "closest the bodies came (cm)"))):
        ax = axs[k]
        for j, (pre, col, nm) in enumerate((("old", ORANGE, "e17 switched off"), ("new", BLUE, "e17"))):
            for i, d in enumerate(("0.45", "1.0")):
                vs = []
                for f in glob.glob(os.path.join(S, "%s-*-%s.json" % (pre, d))):
                    r = json.load(open(f))
                    v = r.get(key)
                    if isinstance(v, dict):
                        v = max(v.values())
                    if v is not None:
                        vs.append(v)
                x = i + (j - 0.5) * 0.3
                ax.scatter([x] * len(vs), vs, color=col, s=14, label=nm if i == 0 else None)
        ax.set_xticks([0, 1]); ax.set_xticklabels(["fixes 0.45 s late", "fixes 1.0 s late"]); ax.set_title(lab, loc="left", fontsize=8, color=INK)
        if k == 0:
            ax.legend(frameon=False, fontsize=7)
    fig.text(0.01, 0.01, "Simulated: freeroam_fastsim, 3 robots, camera sightings through the tracking layer, heading faults, "
             "6 seeds x 30 min each way; one dot per run.", fontsize=6.3, color=MUTED)
    fig.tight_layout(rect=(0, 0.05, 1, 1)); fig.savefig(os.path.join(OUT, "e17-feature-simulator-old-vs-e17-chart.png"), dpi=150)
print("ok")
