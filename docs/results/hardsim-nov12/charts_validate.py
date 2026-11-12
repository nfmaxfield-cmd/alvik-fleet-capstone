"""Validation charts: the real 11 Nov 08:58 run against the hard simulator's measured profile (old code 4732a48 = the
code that ran it, and current main). python3 charts_validate.py REAL_MEASURED.json OLD_DIR NEW_DIR OUT_DIR"""
import glob
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

real_all, old_dir, new_dir, out = sys.argv[1:5]
real = [r for r in json.load(open(real_all))["runs"] if r["run"] == "20261111-085837-freeroam"][0]
NEWLBL = sys.argv[5] if len(sys.argv) > 5 else "current main"
old = [json.load(open(f))["measures"] for f in sorted(glob.glob(old_dir + "/*.json"))]
new = [json.load(open(f))["measures"] for f in sorted(glob.glob(new_dir + "/*.json"))]
C = dict(real="#52514e", old="#2a78d6", new="#eb6834")
LBL = dict(real="Real run, 11 Nov 08:58 (4 h, code 4732a48)",
           old="Simulator, measured profile, same code 4732a48 (%d runs)" % len(old),
           new="Simulator, measured profile, %s (%d runs)" % (NEWLBL, len(new)))
INK, MUTED = "#0b0b0b", "#52514e"


def pooled(rs, key, sub=None):
    """Pool quantile summaries by averaging each quantile across runs (runs are the same length)."""
    qs = ("p5", "p10", "p25", "p50", "p75", "p90", "p95", "p99")
    vs = [r[key] if sub is None else (r.get(key) or {}).get(sub) for r in rs]
    vs = [v for v in vs if v]
    return {q: sum(v[q] for v in vs) / len(vs) for q in qs} if vs else None


def qline(ax, q, color, label, ls="-"):
    if not q:
        return
    ps = [5, 10, 25, 50, 75, 90, 95, 99]
    ax.plot([q["p%d" % p] for p in ps], ps, color=color, lw=2, ls=ls, label=label, marker="o", ms=4)


plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED})
fig, axs = plt.subplots(1, 3, figsize=(12, 3.9))
for ax, key, sub, title, xl, logx in (
        (axs[0], "fix_delay_s", None, "Fix delay: picture to arrival at the run", "seconds", False),
        (axs[1], "quiet_s", None, "Time since the robot last heard the Pi", "seconds", False),
        (axs[2], "own_cam_gap_cm", None, "Robot's own pose vs the camera", "cm (log)", True)):
    qline(ax, real[key], C["real"], LBL["real"])
    qline(ax, pooled(old, key), C["old"], LBL["old"], "--")
    qline(ax, pooled(new, key), C["new"], LBL["new"], ":")
    ax.set_title(title, fontsize=9.5, color=INK)
    ax.set_xlabel(xl)
    if logx:
        ax.set_xscale("log")
    ax.set_ylabel("percentile")
    ax.grid(alpha=0.25)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
axs[0].legend(fontsize=7.5, loc="lower right", frameon=False)
fig.text(0.01, 0.01, "Measured: each line is the distribution (5th to 99th percentile). Real: the 08:58 run's logs "
         "(camera_track.csv, follow.csv, bridge/track.csv). Simulator: tools/fleet/hardsim_batch.py, profile "
         "measured-home-2026-11-11-0858, 0.5 h runs.", fontsize=7, color=MUTED)
fig.tight_layout(rect=(0, 0.05, 1, 1))
fig.savefig(os.path.join(out, "hardsim-feature-validation-distributions-chart.png"), dpi=130)

# rates per robot-hour: dot plot, log scale
M = [("Watchdog stops / h (all robots)", lambda m: m["watchdog_per_h"]),
     ("Quiet spells over 1 s / robot-h", lambda m: m["stalls"]["per_robot_h"]),
     ("Quiet spells over 1.5 s / robot-h", lambda m: m["stalls"]["over_1_5s_per_robot_h"]),
     ("Unnamed spells / robot-h", lambda m: m["unnamed"]["per_robot_h"]),
     ("Camera dropouts (2+ pictures) / robot-h", lambda m: m["cam_gaps"]["per_robot_h"]),
     ("\"Did not move\" / robot-h", lambda m: m["did_not_move"]["per_robot_h"]),
     ("Lost spells / robot-h", lambda m: m["lost"]["per_robot_h"]),
     ("NEEDS A HAND / robot-h", lambda m: m["needs_hand"]["per_robot_h"]),
     ("Heading errors noticed / robot-h", lambda m: m["heading_err_deg"]["per_robot_h"]),
     ("10 s windows, two middles under 15 cm / h", lambda m: (m.get("close") or {}).get("under_15")),
     ("... under 13.44 cm (circles touch) / h", lambda m: (m.get("close") or {}).get("under_13_44"))]
fig, ax = plt.subplots(figsize=(9, 5.2))
for i, (name, f) in enumerate(M):
    y = len(M) - i
    if f(real) is None:
        continue
    ax.scatter([max(0.05, f(real))], [y + 0.18], color=C["real"], s=46, zorder=3, label=LBL["real"] if i == 0 else None)
    for k, rs, dy in (("old", old, 0.0), ("new", new, -0.18)):
        v = [max(0.05, f(m)) for m in rs if f(m) is not None]
        ax.scatter(v, [y + dy] * len(v), color=C[k], s=22, alpha=0.75, zorder=3, label=LBL[k] if i == 0 else None)
ax.set_yticks([len(M) - i for i in range(len(M))])
ax.set_yticklabels([n for n, _ in M])
ax.set_xscale("log")
ax.set_xlabel("rate (log scale; 0 drawn at 0.05)")
ax.grid(axis="x", alpha=0.25)
for s in ("top", "right"):
    ax.spines[s].set_visible(False)
ax.legend(fontsize=7.5, loc="lower center", bbox_to_anchor=(0.4, 1.0), ncol=1, frameon=False)
fig.text(0.01, 0.01, "Measured: real = the 08:58 run's logs (step.csv, follow.csv, events.csv, lost.csv); simulator dots "
         "= one 0.5 h run each (hardsim_batch.py, measured profile).", fontsize=7, color=MUTED)
fig.tight_layout(rect=(0, 0.04, 1, 1))
fig.savefig(os.path.join(out, "hardsim-feature-validation-rates-chart.png"), dpi=130)
print("ok")
