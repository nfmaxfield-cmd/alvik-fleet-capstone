"""Confusion chart for the discovery replay: true type (rows) against guessed type (columns) for the held-out cases,
from tools/discovery/results/replay_summary.json. Writes docs/build-reports/e02-oct25-discovery-replay.png.

    python3 tools/discovery/chart.py"""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                     # noqa: E402
from matplotlib.colors import LinearSegmentedColormap   # noqa: E402
import numpy as np                                  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
SUMMARY = os.path.join(HERE, "results", "replay_summary.json")
OUT = os.path.join(REPO, "docs", "build-reports", "e02-oct25-discovery-replay.png")

SHORT = {"gyro_rate": "gyro rate", "gyro_angle": "gyro angle", "wheel_angle": "wheel angle",
         "wheel_speed": "wheel speed", "accel": "accelerometer", "range": "range zone", "line": "line sensor",
         "switch": "switch", "battery": "battery", "constant": "constant", "clock": "clock", "counter": "counter",
         "mode": "step index", "id": "text id", "derived": "derived", "unknown": "unknown"}
# one-hue sequential ramp (light -> dark blue), the dataviz reference palette
BLUES = ["#f7fafe", "#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]
INK, MUTED, GRID = "#1f1f1d", "#6b6a66", "#e6e4df"


def panel(ax, conf, title):
    types = conf["types"]
    M = np.array(conf["matrix"])
    rows = [i for i in range(len(types)) if M[i].sum() > 0]
    cols = [j for j in range(len(types)) if M[:, j].sum() > 0 or j in rows]
    sub = M[np.ix_(rows, cols)]
    share = sub / np.maximum(sub.sum(axis=1, keepdims=True), 1)
    cmap = LinearSegmentedColormap.from_list("seq", BLUES)
    ax.imshow(share, cmap=cmap, vmin=0, vmax=1, aspect="auto")
    for i in range(len(rows)):
        for j in range(len(cols)):
            n = sub[i, j]
            if n:
                ax.text(j, i, str(n), ha="center", va="center", fontsize=8,
                        color="white" if share[i, j] > 0.55 else INK)
    ax.set_xticks(range(len(cols)))
    ax.set_xticklabels([SHORT[types[j]] for j in cols], rotation=55, ha="right", fontsize=8, color=INK)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels(["%s (%d)" % (SHORT[types[i]], M[i].sum()) for i in rows], fontsize=8, color=INK)
    ax.set_xlabel("guessed type (names hidden)", fontsize=9, color=INK)
    ax.set_ylabel("true type (streams in these cases)", fontsize=9, color=INK)
    ax.set_title(title, fontsize=10, color=INK, loc="left")
    ax.set_xticks(np.arange(-0.5, len(cols)), minor=True)
    ax.set_yticks(np.arange(-0.5, len(rows)), minor=True)
    ax.grid(which="minor", color="white", linewidth=1.5)
    ax.tick_params(which="minor", length=0)
    for sp in ax.spines.values():
        sp.set_visible(False)


def main():
    s = json.load(open(SUMMARY))
    rich, short = s["by_split"]["test_rich"], s["by_split"]["test_short"]
    fig, axes = plt.subplots(1, 2, figsize=(14, 6.6), gridspec_kw={"width_ratios": [1.15, 1]})
    panel(axes[0], s["confusion"]["test_rich"],
          "Held-out cases with motion: %d of %d streams right (%d cases)"
          % (rich["right"], rich["streams"], rich["cases"]))
    panel(axes[1], s["confusion"]["test_short"],
          "Held-out 9 s calibration logs: %d of %d right (%d cases)"
          % (short["right"], short["streams"], short["cases"]))
    fig.text(0.01, 0.01,
             "Measured: the type the discoverer guessed for each RAW stream with its name hidden, against the real "
             "column; the top guess counts whether it was accepted, asked about or left unknown (cell = number of "
             "streams; shade = share of that true type). Data: Alvik RAW logs, sessions 2026-10-21 to 10-24 on "
             "disk; held out = robot Right, and every robot on 10-24.",
             fontsize=8, color=MUTED, wrap=True)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(OUT, dpi=130, facecolor="white")
    print(OUT)


if __name__ == "__main__":
    main()
