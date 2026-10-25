"""Charts for docs/results/ (2026-10-25). Numbers are copied from the reports named beside each list, not recomputed.

    python3 docs/results/make_charts.py

Writes sim-grid.png, sim-catches.png and robot-differences.png next to this file."""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_COL = {"Left": "#3b6fb6", "Center": "#d08a2e", "Right": "#3f9a5a"}


def grid_chart():
    # docs/results/data/grid_summary_2026-10-25.txt (tools/experiment/analyses_oct24/grid_summary.py on /tmp/claude-0/grid)
    rows = [  # name, runs scored (of 21), holds, finished?
        ("No fault (baseline)", 21, 0, True),
        ("Camera loses 30% of positions", 21, 0, True),
        ("Center wheels 5% jitter", 21, 0, True),
        ("Center wheels 10% jitter", 21, 0, True),
        ("All wheels 8% jitter", 21, 1, True),
        ("Right gyro 5% jitter", 21, 3, True),
        ("Left front sensor +8 cm", 21, 0, True),
        ("Left front sensor noise 6 cm", 7, 0, False),
        ("Pose kept from last session", 17, 6, False),
        ("Camera noise 2 cm (night)", 8, 31, True),
        ("Camera noise 2 cm (after fixes)", 11, 5, False),
        ("Camera noise 4 cm (night)", 0, 17, True),
        ("Camera noise 4 cm (after fixes)", 0, 7, True),
    ]
    rows = rows[::-1]
    fig, ax = plt.subplots(figsize=(8.5, 5.6))
    y = range(len(rows))
    ax.barh([i + 0.2 for i in y], [r[1] for r in rows], height=0.4, color="#3b6fb6", label="Runs scored (of 21)")
    ax.barh([i - 0.2 for i in y], [r[2] for r in rows], height=0.4, color="#c0504d", label="Holds (all robots stopped)")
    for i, r in enumerate(rows):
        if not r[3]:
            ax.text(r[1] + 0.4, i + 0.2, "cut at 24 min", va="center", fontsize=7, color="#555555")
    ax.set_yticks(list(y))
    ax.set_yticklabels([r[0] for r in rows], fontsize=8)
    ax.set_xlabel("Count per simulated session (0.4 h limit)")
    ax.set_ylabel("Sensor degraded on purpose")
    ax.set_title("Bad-sensor grid in the simulator, 2026-10-24/25", fontsize=10)
    ax.legend(fontsize=8, loc="lower right")
    ax.grid(axis="x", alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "sim-grid.png"), dpi=130)
    plt.close(fig)


def catches_chart():
    # docs/results/simulation-register.md section 2 (catch log), counted from the sources cited there
    days = ["Oct 21", "Oct 22", "Oct 23", "Oct 24", "Oct 25"]
    sim = [0, 9, 5, 8, 9]
    board = [2, 6, 3, 4, 3]
    fig, ax = plt.subplots(figsize=(6.5, 3.6))
    x = range(len(days))
    ax.bar([i - 0.2 for i in x], sim, width=0.4, color="#3b6fb6", label="Found in the simulator first")
    ax.bar([i + 0.2 for i in x], board, width=0.4, color="#c0504d", label="Found on the real board or in its logs")
    ax.set_xticks(list(x))
    ax.set_xticklabels(days)
    ax.set_xlabel("Day the problem was found (2026)")
    ax.set_ylabel("Problems found (count)")
    ax.set_title("Where runner and firmware problems were caught", fontsize=10)
    ax.set_ylim(0, 12)
    ax.legend(fontsize=8, loc="upper center", ncol=2)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "sim-catches.png"), dpi=130)
    plt.close(fig)


def robot_chart():
    robots = ["Left", "Center", "Right"]
    # gyro scale: docs/session-notes/2026-10-21-real-footage.md (Oct 21, Left/Right only);
    # docs/build-reports/e02-oct25-logs-doe.md Appendix A1 (Oct 23 morning, Oct 23 evening, Oct 24 stage 2)
    gyro = {
        "Oct 21 footage": [1.069, None, 1.075],
        "Oct 23 morning": [1.076, 1.082, 1.077],
        "Oct 23 evening": [1.091, 1.071, 1.076],
        "Oct 24 stage 2": [1.087, 1.075, 1.081],
    }
    # wheel scale, track cut at 3 cm jumps: docs/build-reports/e02-oct24-night-analyses.md Appendix B1
    wheels = {
        "Oct 23 morning": [0.988, 0.989, 1.000],
        "Oct 23 evening": [0.975, 0.961, 0.992],
        "Oct 24 stage 1": [0.947, 0.978, 0.998],
        "Oct 24 stage 2": [0.960, 0.975, 0.984],
    }
    # battery drain standing, %/h: docs/build-reports/e02-oct24-night-analyses.md section C (fits and standing gaps)
    drain = {
        "Left": [15.3, 15.2, 15.7],
        "Center": [16.0, 14.4, 14.8, 12.6],
        "Right": [14.1, 12.7, 10.7, 11.2, 13.7],
    }
    markers = ["o", "s", "^", "D"]
    fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.6))
    for ax, data, title, ylab in ((axes[0], gyro, "Gyro scale (self-calibration fit)", "Real turn / gyro turn"),
                                  (axes[1], wheels, "Wheel scale (self-calibration fit)", "Camera distance / wheel distance")):
        for k, (sess, vals) in enumerate(data.items()):
            xs = [i + (k - 1.5) * 0.08 for i, v in enumerate(vals) if v is not None]
            ys = [v for v in vals if v is not None]
            ax.scatter(xs, ys, marker=markers[k], s=28, color="#444444", label=sess, zorder=3)
        ax.set_xticks(range(3))
        ax.set_xticklabels(robots)
        ax.set_xlabel("Robot")
        ax.set_ylabel(ylab)
        ax.set_title(title, fontsize=9)
        ax.grid(axis="y", alpha=0.3)
        ax.legend(fontsize=6.5)
    ax = axes[2]
    for i, r in enumerate(robots):
        ax.scatter([i] * len(drain[r]), drain[r], color=ROBOT_COL[r], s=28, zorder=3)
    ax.set_xticks(range(3))
    ax.set_xticklabels(robots)
    ax.set_xlabel("Robot")
    ax.set_ylabel("Battery drain standing, % per hour")
    ax.set_title("Battery drain (fits and standing gaps)", fontsize=9)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "robot-differences.png"), dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    grid_chart()
    catches_chart()
    robot_chart()
