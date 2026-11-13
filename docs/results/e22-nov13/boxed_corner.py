#!/usr/bin/env python3
"""e22c (Nick 13 Nov ~10:00: two robots stood for a long time and boxed the third into a corner): the simulator case,
run with MIRLT_BOX_ASK off (the way before) and on, drawn as the robots' true tracks over 3 simulated minutes.

    python3 docs/results/e22-nov13/boxed_corner.py
Writes boxed_corner.json here and docs/screenshots/2026-11-13/e22-*-boxed-corner-*.png. Simulator only.
Changelog: e22c-2026-11-13 first version.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "tools", "fleet"))
os.environ.setdefault("MIRLT_PLAN_SLOWDOWN", "0")
import freeroam_core as FC        # noqa: E402
import freeroam_fastsim as FS     # noqa: E402
import matplotlib                 # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402
from matplotlib.patches import Rectangle   # noqa: E402

STARTS = {"Left": (13.0, 13.0, 225.0), "Center": (31.0, 12.0, 90.0), "Right": (12.0, 31.0, 0.0)}
COL = {"Left": "#2952CC", "Center": "#D9822B", "Right": "#7A4FBF"}


def run(on):
    FC.BOX_ASK = on
    s = FS.Sim(1, 0.05, starts=STARTS, held={"Center": "heading", "Right": "heading"}, edge_physics=True, sensors=True)
    s.ctrl.plan_slowdown = 0.0
    tr = {n: [] for n in s.bots}
    out = [None]
    otick = s.ctrl.tick

    def tick(t):
        res = otick(t)
        for n, b in s.bots.items():
            if not tr[n] or t - tr[n][-1][0] >= 0.5:
                tr[n].append((round(t, 1), round(b.x, 1), round(b.y, 1)))
        b = s.bots["Left"]
        if out[0] is None and math.hypot(b.x, b.y) > 45.0:
            out[0] = round(t, 1)
        return res
    s.ctrl.tick = tick
    summ = s.run()
    return dict(box_ask=on, left_out_s=out[0], falls=summ.get("falls"), contacts=summ.get("contacts"),
                room_made=s.ctrl.counts.get("room_made", 0), asked=s.ctrl.counts.get("room_asked_boxed", 0),
                tracks=tr)


def main():
    res = [run(False), run(True)]
    with open(os.path.join(HERE, "boxed_corner.json"), "w") as f:
        json.dump([{k: v for k, v in r.items() if k != "tracks"} for r in res], f, indent=1)
    fig, axs = plt.subplots(1, 2, figsize=(10, 5.8))
    for ax, r in zip(axs, res):
        ax.add_patch(Rectangle((0, 0), 122.24, 121.92, fill=False, ec="#333", lw=1.2))
        for n, pts in r["tracks"].items():
            xs, ys = [p[1] for p in pts], [p[2] for p in pts]
            ax.plot(xs, ys, color=COL[n], lw=1.4, label=n + (" (cornered)" if n == "Left" else " (standing, held)"))
            ax.plot(xs[0], ys[0], "o", color=COL[n], ms=9, mfc="white")
            ax.plot(xs[-1], ys[-1], "s", color=COL[n], ms=6)
        ax.set_xlim(-3, 125)
        ax.set_ylim(-3, 125)
        ax.set_aspect("equal")
        ax.set_xlabel("x on the board (cm)")
        ax.set_ylabel("y (cm)")
        ax.set_title(("MIRLT_BOX_ASK=0 (before): Left never gets out" if not r["box_ask"] else
                      "MIRLT_BOX_ASK=1 (e22): out after %.0f s, %d moves made room" % (r["left_out_s"] or -1,
                                                                                       r["room_made"])),
                     fontsize=10)
        ax.legend(loc="upper right", fontsize=8)
    fig.suptitle("Boxed into a corner: each robot's true track over 3 simulated minutes (circle: start, square: end)",
                 fontsize=11)
    fig.text(0.01, 0.01, "Source: tools/fleet freeroam_fastsim, seed 1, Center and Right held (heading unknown) across "
             "Left's way out; docs/results/e22-nov13/boxed_corner.py", fontsize=8, color="#5B6474")
    fig.tight_layout(rect=(0, 0.06, 1, 0.95))
    p = os.path.join(REPO, "docs", "screenshots", "2026-11-13", "e22-problem-and-fix-boxed-corner-tracks.png")
    fig.savefig(p, dpi=90)
    print(p)
    print(json.dumps([{k: v for k, v in r.items() if k != "tracks"} for r in res]))


if __name__ == "__main__":
    main()
