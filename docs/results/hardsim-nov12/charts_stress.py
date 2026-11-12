"""Stress chart: what broke, per group of hard-simulator runs. python3 charts_stress.py OUT.png TITLE LABEL=DIR[:faults] ...
Each column a group; each row a kind of break, as a rate per simulated hour (or a share); the cell is shaded by how
bad it is against the worst in its row."""
import glob
import json
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

out, title = sys.argv[1], sys.argv[2]
groups = []
for spec in sys.argv[3:]:
    lab, d = spec.split("=", 1)
    flt = None
    if ":" in d:
        d, flt = d.split(":", 1)
    rs = [json.load(open(f)) for f in sorted(glob.glob(d + "/*.json"))]
    if flt:
        rs = [r for r in rs if r["faults"] == [flt]]
    groups.append((lab, rs))

ROWS = [("Contacts / h", lambda B, H: sum(b["contacts"] for b in B) / H),
        ("Runs with bodies under 1 cm apart (%)", lambda B, H: 100.0 * sum(1 for b in B if b["min_gap_cm"] < 1.0) / len(B)),
        ("Falls off the board / h", lambda B, H: sum(b["falls"] for b in B) / H),
        ("Hung at an edge (robot-min / h)", lambda B, H: sum(b["hung_s"] for b in B) / 60.0 / H),
        ("Name swaps / h", lambda B, H: sum(b["swaps"] for b in B) / H),
        ("NEEDS A HAND / h", lambda B, H: sum(b["needs_hand"] for b in B) / H),
        ("Stands over 5 s, free to move / h", lambda B, H: sum(b.get("stands_free", b["stands_over"]) for b in B) / H),
        ("Longest stand, free to move (s)", lambda B, H: max(b.get("longest_free_stand_s", b["longest_stand_s"])
                                                             for b in B)),
        ("Blind: run's error over its own doubt (%)",
         lambda B, H: 100.0 * sum(b["blind_breaches"] for b in B) / max(1, sum(b["blind_samples"] for b in B)))]
M = []
for lab, rs in groups:
    B = [r["broke"] for r in rs]
    H = sum(r["hours"] for r in rs)
    M.append([f(B, H) if B else float("nan") for _, f in ROWS])
fig, ax = plt.subplots(figsize=(1.35 * len(groups) + 4.2, 0.48 * len(ROWS) + 1.6))
for j in range(len(ROWS)):
    col = [M[i][j] for i in range(len(groups))]
    mx = max([c for c in col if c == c] + [1e-9])
    for i, v in enumerate(col):
        a = 0.0 if v != v or mx <= 0 else v / mx
        ax.add_patch(plt.Rectangle((i, len(ROWS) - 1 - j), 0.96, 0.92, color=(0.92 - 0.0 * a, 0.40 + 0.52 * (1 - a),
                                                                            0.20 + 0.72 * (1 - a))))
        ax.text(i + 0.48, len(ROWS) - 1 - j + 0.46, "-" if v != v else ("%.0f" % v if v >= 10 else "%.1f" % v),
                ha="center", va="center", fontsize=8.5, color="#0b0b0b")
ax.set_xlim(0, len(groups))
ax.set_ylim(0, len(ROWS))
ax.set_xticks([i + 0.48 for i in range(len(groups))])
ax.set_xticklabels(["%s\n(%d runs)" % (lab, len(rs)) for lab, rs in groups], fontsize=7.5)
ax.set_yticks([len(ROWS) - 1 - j + 0.46 for j in range(len(ROWS))])
ax.set_yticklabels([r for r, _ in ROWS], fontsize=8)
ax.tick_params(length=0)
for s in ax.spines.values():
    s.set_visible(False)
ax.set_title(title, fontsize=9.5)
fig.text(0.01, 0.01, "Measured: the hard simulator (tools/fleet/hardsim_batch.py), 0.5 h runs, the free-roam controller "
         "as on main; truth from the simulator's own bodies. Darker = worse within its row.", fontsize=7,
         color="#52514e")
fig.tight_layout(rect=(0, 0.05, 1, 1))
fig.savefig(out, dpi=130)
print("ok")
