"""e19 charts: the out-and-back (layer alone, the audit's harness) and the tag-free heading hand-on, before / after.
Reads the result text files next to it (docs/results/e19-nov12/identity, tagfree); writes PNGs to docs/screenshots.
    python3 charts_identity_heading.py"""
import os, re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.dirname(HERE)
SHOT = os.path.join(os.path.dirname(os.path.dirname(RES)), "screenshots", "2026-11-12")
B, O, INK, MUTED, GRID = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e6e5e0"


def style(ax):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.yaxis.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)


def bars(ax, labels, before, after, ylab):
    x = range(len(labels))
    w = 0.38
    ax.bar([i - w / 2 - 0.01 for i in x], before, w, color=O, label="before (c79c1df)")
    ax.bar([i + w / 2 + 0.01 for i in x], after, w, color=B, label="e19")
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, fontsize=8, color=INK)
    ax.set_ylabel(ylab, fontsize=9, color=MUTED)
    style(ax)
    for i, (a, b) in enumerate(zip(before, after)):
        ax.text(i - w / 2, a, "%g" % a, ha="center", va="bottom", fontsize=7, color=MUTED)
        ax.text(i + w / 2, b, "%g" % b, ha="center", va="bottom", fontsize=7, color=MUTED)


def ob():
    def parse(f):
        out = []
        for ln in open(os.path.join(RES, "identity", f)):
            m = re.search(r"(\w+)\s+sd ([\d.]+) cm a direction, (\d+) pictures/s: first name RIGHT\s+(\d+), WRONG\s+(\d+), "
                          r"nobody named in 90 s\s+(\d+)", ln)
            if m:
                out.append(("%s %s cm\n%s pic/s" % (m.group(1), m.group(2), m.group(3)), int(m.group(5)), int(m.group(6))))
        return out
    bef, aft = parse("outback_layer_before.txt"), parse("outback_layer_after.txt")

    def parse2(f):
        return [(("%s cm\n%s pic/s" % (m.group(2), m.group(1).rstrip("0").rstrip("."))), int(m.group(3)))
                for m in (re.search(r"hz\s+([\d.]+)\s+per-axis sd ([\d.]+) cm: named although nobody moved in\s+(\d+)", ln)
                          for ln in open(os.path.join(RES, "identity", f))) if m]
    lb, la = parse2("outback_lost_command_before.txt"), parse2("outback_lost_command_after.txt")
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.2), gridspec_kw=dict(width_ratios=[5, 8]))
    bars(a1, [q[0] for q in bef], [q[1] for q in bef], [q[1] for q in aft], "trials (of 200) whose first name was WRONG")
    a1.set_title("Robot drives out and back: first name wrong", fontsize=10, color=INK, loc="left")
    bars(a2, [q[0] for q in lb], [q[1] for q in lb], [q[1] for q in la], "trials (of 200) with a robot named")
    a2.set_title("Nobody moved (the command was lost): a robot named anyway", fontsize=10, color=INK, loc="left")
    a1.legend(frameon=False, fontsize=8, loc="upper left")
    fig.suptitle("Out-and-back identity drive, tracking layer alone, 200 simulated trials a cell (the audit's harness; "
                 "noise = camera sighting sd per direction)", fontsize=9, color=MUTED, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(os.path.join(SHOT, "e19-feature-h3-out-and-back-wrong-names-before-after-chart.png"), dpi=120,
                facecolor="#fcfcfb")


def hand():
    def parse(f):
        out = []
        for ln in open(os.path.join(RES, "tagfree", "handon_before_after.txt")).read().split("== ")[1 if f == "b" else 2].splitlines():
            m = re.search(r"^(.+?)\s+(\d+) s standing, own heading right: wrong hand-on\s+([\d.]+) %", ln)
            if m:
                lab = m.group(1).replace("independent ", "").replace("same error every picture (13.3 deg bias + 5 deg)",
                                                                         "same error\nevery picture")
                out.append(("%s\n%s s" % (lab, m.group(2)), float(m.group(3))))
        return out
    b, a = parse("b"), parse("a")
    fig, ax = plt.subplots(figsize=(12, 4.2))
    bars(ax, [q[0] for q in b], [q[1] for q in b], [q[1] for q in a], "% of stands with a WRONG heading handed on")
    ax.legend(frameon=False, fontsize=8, loc="upper left")
    ax.set_title("Tag-free: a robot standing with a CORRECT own heading; how often the detector's heading is handed on "
                 "anyway\n(heading fuser alone, 500 simulated stands a cell, 2 detector headings a second; x: the detector's heading "
                 "noise and how long the robot stood)", fontsize=9,
                 color=INK, loc="left")
    fig.tight_layout()
    fig.savefig(os.path.join(SHOT, "e19-feature-tagfree-wrong-heading-handon-before-after-chart.png"), dpi=120,
                facecolor="#fcfcfb")


ob()
hand()
