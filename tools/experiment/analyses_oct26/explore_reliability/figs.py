"""figs.py : the figures of docs/build-reports/e02-oct26-explore-reliability.md, saved to
docs/screenshots/2026-10-26/explore-*.png.
  map RUN_FOLDER OUT TITLE   one explore on the simulator's board: each robot's true path, every ray end that met a
                             side (true place), the true board and, when explore found one, its board on the truth
  rates BEFORE.jsonl AFTER.jsonl OUT   pass rate before and after, by condition
  scale EVAL.jsonl AFTER.jsonl OUT     explore's camera scale error per run, before / offline variants / after"""
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import exsum  # noqa: E402

BLUE, ORANGE, AQUA, YELLOW = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
ROBOT_COL = {"Left": BLUE, "Center": ORANGE, "Right": AQUA}


def style(ax):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.grid(color=GRID, lw=0.6)
    ax.set_axisbelow(True)


def explore_map(d, out, title):
    st = json.load(open(os.path.join(d, "e02_state.json")))
    info = json.load(open(os.path.join(d, "sim", "sim_info.json")))
    W, H = info["board"]["yboard_cm"], info["board"]["xboard_cm"]
    tr = exsum.truth_reader(d)
    obs = st["setup"]["explore_obs"]
    t0 = min(v[0]["t"] for v in obs.values() if v)
    t1 = max(v[-1]["t"] for v in obs.values() if v)
    fig, ax = plt.subplots(figsize=(6.2, 6.2), dpi=130)
    style(ax)
    ax.plot([0, W, W, 0, 0], [0, 0, H, H, 0], color=INK, lw=1.2)
    ax.plot([W, W], [0, H], color=INK, lw=4, solid_capstyle="butt")
    ax.plot([0, W], [H, H], color=INK, lw=4, solid_capstyle="butt")
    for n, a in tr.items():
        m = (a[:, 0] >= t0 - 1) & (a[:, 0] <= t1 + 1)
        ax.plot(a[m, 1], a[m, 2], color=ROBOT_COL.get(n, MUTED), lw=1.0, alpha=0.8, label=n + "'s path")
        for q in obs.get(n, []):
            k = q.get("contact")
            if not k:
                continue
            x, y, h = exsum.at(tr, n, q["t"])
            ax.plot([x], [y], marker="s" if k == "board" else "^", ms=7, mfc=ROBOT_COL.get(n, MUTED), mec="white",
                    mew=1.0, ls="none")
    tries = st["setup"].get("explore_tries") or []
    ex = st["setup"].get("explore") or {}
    if tries and tries[-1].get("bf_R") and tries[-1].get("sol"):
        t = tries[-1]
        _, _, _, (mir, R, s, ma, mb) = exsum.cam_scale(t["sol"], d)
        bR, bt = np.array(t["bf_R"]), np.array(t["bf_t"])
        corners = [(0, 0), (t["W"], 0), (t["W"], t["H"]), (0, t["H"]), (0, 0)]
        pts = []
        for bx, by in corners:
            w = np.linalg.solve(bR, np.array([bx, by]) - bt)
            q = s * (R @ (np.array([w[0], mir * w[1]]) - ma)) + mb
            pts.append(q)
        pts = np.array(pts)
        ax.plot(pts[:, 0], pts[:, 1], color=YELLOW, lw=1.6, ls="--", label="board explore found")
    ax.plot([], [], marker="^", ms=7, color=MUTED, ls="none", label="ray met an open edge")
    ax.plot([], [], marker="s", ms=7, color=MUTED, ls="none", label="ray met a board")
    ax.set_xlim(-12, W + 12)
    ax.set_ylim(-12, H + 12)
    ax.set_aspect("equal")
    ax.set_xlabel("cm along the board (simulator truth); thick sides are the boards", fontsize=8, color=MUTED)
    ax.set_title(title, fontsize=10, color=INK, loc="left")
    ax.legend(fontsize=7, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=3)
    fig.tight_layout()
    fig.savefig(out)
    print("saved", out, "explore:", ex.get("W"), ex.get("H"), (st["setup"].get("explore_failed") or {}).get("why", ""))


def rows(p):
    return [json.loads(l) for l in open(p) if l.strip()]


def rates(bp, ap, out):
    b, a = rows(bp), rows(ap)
    groups = [("All runs", lambda r: True)]
    groups += [("%d pictures/s" % f, (lambda f: lambda r: int(r["fps"]) == f)(f)) for f in (10, 18, 25)]
    groups += [("%d%% commands lost" % p, (lambda p: lambda r: int(r["drop"]) == p)(p)) for p in (0, 2, 10)]
    groups += [("washed-out robot", lambda r: bool(r["washed"])), ("no washed-out robot", lambda r: not r["washed"])]
    fig, ax = plt.subplots(figsize=(7.4, 4.2), dpi=130)
    style(ax)
    y = np.arange(len(groups))[::-1]
    for k, (lab, f) in enumerate(groups):
        for rs, col, off, name in ((b, MUTED, 0.18, "before"), (a, BLUE, -0.18, "after")):
            sel = [r for r in rs if f(r)]
            if not sel:
                continue
            p = 100.0 * sum(r["passed"] for r in sel) / len(sel)
            ax.barh(y[k] + off, p, height=0.34, color=col, label=name if k == 0 else None)
            ax.text(p + 1, y[k] + off, "%d of %d" % (sum(r["passed"] for r in sel), len(sel)), va="center", fontsize=7,
                    color=MUTED)
    ax.set_yticks(y)
    ax.set_yticklabels([g[0] for g in groups], fontsize=8)
    ax.set_xlim(0, 110)
    ax.set_xlabel("explores that found the board (%)", fontsize=8, color=MUTED)
    ax.legend(fontsize=8, frameon=False, loc="lower right")
    fig.tight_layout()
    fig.savefig(out)
    print("saved", out)


def scale(ep, ap, out, bp=None):
    e = rows(ep)
    a = rows(ap)
    series = [("before, as run", [r for r in e if r["variant"] == "base+size"], MUTED),
              ("before's stops, this build's fit", [r for r in e if r["variant"] == "repair+sprior+seen+keep+size"],
               ORANGE),
              ("after, driven again", a, BLUE)]
    fig, ax = plt.subplots(figsize=(7.4, 3.6), dpi=130)
    style(ax)
    for k, (lab, rs, col) in enumerate(series):
        vals = [100 * (r["cam_scale"] - 1) for r in rs if r.get("cam_scale")]
        ok = [bool(r.get("ok", r.get("passed"))) for r in rs if r.get("cam_scale")]
        jit = np.linspace(-0.18, 0.18, max(1, len(vals)))
        for v, o, j in zip(vals, ok, jit):
            if v > 30:
                ax.annotate("%+.0f%%" % v, (30, k + j), fontsize=6, color=MUTED, va="center", xytext=(31, k + j))
                v = 30
            ax.plot(v, k + j, marker="o" if o else "x", ms=6, color=col, mfc=col if o else "none", ls="none")
        if vals:
            ax.text(-19, k - 0.32, "median %+.1f%%" % np.median(vals), fontsize=7, color=MUTED, va="center")
    ax.axvline(0, color=INK, lw=0.8)
    ax.set_xlim(-20, 36)
    ax.set_yticks(range(len(series)))
    ax.set_yticklabels([s[0] for s in series], fontsize=8)
    ax.invert_yaxis()
    ax.set_xlabel("explore's camera size against the true camera (%; dot = explore passed, cross = failed)", fontsize=8,
                  color=MUTED)
    fig.tight_layout()
    fig.savefig(out)
    print("saved", out)


if __name__ == "__main__":
    k = sys.argv[1]
    if k == "map":
        explore_map(sys.argv[2], sys.argv[3], sys.argv[4])
    elif k == "rates":
        rates(sys.argv[2], sys.argv[3], sys.argv[4])
    elif k == "scale":
        scale(sys.argv[2], sys.argv[3], sys.argv[4])
