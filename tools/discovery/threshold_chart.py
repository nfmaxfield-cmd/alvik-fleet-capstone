"""Chart for the acceptance-threshold experiment, from tools/discovery/results/threshold_summary.json and
threshold_rules.json. Writes docs/build-reports/e02-oct25-discovery-threshold.png.

    python3 tools/discovery/threshold_chart.py

(a) wrong share among accepted labels against the threshold, one line per amount of motion (held-out, plain).
(b) the rules' trade-off: questions per robot against wrong share, held-out, plain and disguised."""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                     # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
RES = os.path.join(HERE, "results")
OUT = os.path.join(REPO, "docs", "build-reports", "e02-oct25-discovery-threshold.png")
INK, MUTED, GRID = "#1f1f1d", "#6b6a66", "#e6e4df"
# motion is a magnitude: one hue, light (little motion) -> dark (much), the dataviz reference blue ramp
STEPS = ["#b7d3f6", "#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#104281", "#0d366b"]
BINS = ["0-5 s", "~10 s", "~20 s", "~40 s", "~80 s", "~160 s", "240 s +"]
SERIES = {"plain": "#2a78d6", "disguised": "#eb6834"}      # categorical slots 1 and 2


def style(ax):
    ax.grid(color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=8)


def main():
    s = json.load(open(os.path.join(RES, "threshold_summary.json")))
    ru = json.load(open(os.path.join(RES, "threshold_rules.json")))
    fig, (a, b) = plt.subplots(1, 2, figsize=(13.5, 5.8), gridspec_kw={"width_ratios": [1, 1.1]})
    cells = [c for c in s["cells"] if c["side"] == "heldout" and c["variant"] == "plain"]
    for i, bn in enumerate(BINS):
        cc = sorted([c for c in cells if c["bin"] == bn], key=lambda c: c["threshold"])
        if cc:
            a.plot([c["threshold"] for c in cc], [100 * c["wrong_share"] for c in cc], color=STEPS[i], lw=2,
                   marker="o", ms=4, label="%s of motion (%d pieces)" % (bn, cc[0]["n_pieces"]))
    a.set_xlabel("acceptance threshold (confidence needed to accept without asking)", fontsize=9, color=INK)
    a.set_ylabel("wrong labels per 100 accepted", fontsize=9, color=INK)
    a.set_title("(a) Held-out pieces, plain units: wrong share by threshold", fontsize=10, color=INK, loc="left")
    a.set_ylim(bottom=0)
    a.legend(fontsize=7.5, frameon=False, loc="upper right")
    style(a)

    ch = ru["chosen"]
    for v in ("plain", "disguised"):
        keep = {(0.80, 0), (0.90, 0), (0.95, 0), (0.90, 60), (0.90, 240)}
        pts = [c for c in ru["rules"] if c["side"] == "heldout" and c["variant"] == v and
               (round(c["threshold"], 2), c["gate_s"]) in keep]
        for c in pts:
            x, y = c["questions_per_robot"], 100 * c["wrong_share"]
            b.errorbar(x, y, yerr=[[y - 100 * c["wrong_share_lo"]], [100 * c["wrong_share_hi"] - y]],
                       fmt="o", ms=6, color=SERIES[v], ecolor=SERIES[v], elinewidth=1, capsize=2, alpha=0.9,
                       label=v + " units" if c is pts[0] else None)
            chosen = ch and c["threshold"] == ch["threshold"] and c["gate_s"] == ch["gate_s"]
            txt = "%.2f%s" % (c["threshold"], "" if c["gate_s"] == 0 else ", gate %d s" % c["gate_s"])
            if c["gate_s"] == 0:
                off, ha = ((7, 4), "left") if v == "plain" else ((-7, 4), "right")
            elif chosen:
                off, ha = ((10, 46), "left") if v == "plain" else ((-10, 62), "right")
            else:
                off, ha = ((10, 22), "left") if v == "plain" else ((-10, 34), "right")
            b.annotate(txt + (" (chosen)" if chosen else ""), (x, y), xytext=off, ha=ha,
                       textcoords="offset points", fontsize=7, color=INK if chosen else MUTED,
                       fontweight="bold" if chosen else "normal",
                       arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.6) if c["gate_s"] else None)
    b.set_xlim(right=18)
    b.set_xlabel("questions asked per robot (log piece)", fontsize=9, color=INK)
    b.set_ylabel("wrong labels per 100 accepted (95% interval)", fontsize=9, color=INK)
    b.set_title("(b) Held-out pieces: each rule's questions against its wrong share", fontsize=10, color=INK,
                loc="left")
    b.set_ylim(bottom=0)
    b.legend(fontsize=8, frameon=False, loc="upper right")
    style(b)
    fig.text(0.01, 0.01,
             "Measured: labels the blind discoverer would accept without asking that are wrong, and labels it would "
             "ask about, per robot. Rule = threshold, and gate = seconds of wheel motion before a motion-type label "
             "may be accepted. Data: held-out Alvik RAW logs (Right 10-21 to 10-23; all robots 10-24), whole and "
             "cut into pieces with 10-160 s of motion; disguised = streams rescaled, half sign-flipped. Intervals: "
             "bootstrap over cases.", fontsize=7.5, color=MUTED, wrap=True)
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.savefig(OUT, dpi=130, facecolor="white")
    print(OUT)


if __name__ == "__main__":
    main()
