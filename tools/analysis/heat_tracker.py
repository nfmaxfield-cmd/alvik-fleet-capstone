#!/usr/bin/env python3
"""Heat tracker (Nick, 2026-10-26: "Start a tracker for heat correlations").

For each session folder with cpu.csv (the Pi's temperature, throttle flags, clock, core loads; tools/experiment/
e02_cpulog.py) and perf.csv (the camera's pictures handled a second, picture age), this:
  - joins the two by time (perf.csv's row nearest each cpu.csv row, within 1.5 s);
  - sorts every row into a heat state from the throttle flags NOW (bits 0-3):
      "throttling"  = frequency capped or throttled now (bit 1 or 2),
      "soft limit"  = only the soft temperature limit now (bit 3),
      "none"        = no flag now;
  - summarises the session: temperature (median, max, minutes at 80 C and over), share of time in each state, the
    camera's pictures a second and picture age in each state, and the rank correlation (Spearman) between temperature
    and pictures a second;
  - appends one row to docs/results/heat-tracker.md (replacing that session's row if it is already there) and writes a
    chart docs/results/heat/<session>.png.

    python3 tools/analysis/heat_tracker.py <session folder> [<session folder> ...]

Spearman's rho, in fine print: the correlation of the two series' ranks, -1 to +1; it asks "when one is higher, is the
other higher too", without assuming a straight-line relation. A small |rho| with a long, nearly flat temperature
record says little: the temperature barely varied.

Caveat: freq_mhz in cpu.csv is core 0's reported clock (/sys scaling_cur_freq). On the 2026-10-26 session it read 1800
throughout while the firmware reported "frequency capped": the firmware's own capping may not show there. The throttle
flags are the record to trust.
"""
import math
import os
import sys

import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
TRACKER = os.path.join(REPO, "docs", "results", "heat-tracker.md")
CHART_DIR = os.path.join(REPO, "docs", "results", "heat")
HOT_C = 80.0          # the Raspberry Pi starts capping its clock around 80-85 C (the firmware's own limits); a
                      # reporting line, not a decision: no DOE

HEADER = """# Heat tracker: the Pi's temperature against the camera's speed

One row per session that logged cpu.csv (from 2026-10-26). Made by `tools/analysis/heat_tracker.py <session folder>`;
each session's chart is in `heat/`. States from the Pi's throttle flags at that moment: **throttling** = clock capped
or throttled now; **soft limit** = only the soft temperature limit now; **none** = no flag. Pictures a second = the
camera pictures the runner handled (perf.csv, of about 25). rho = Spearman rank correlation of temperature with
pictures a second (-1 to +1; near 0 = no link, or temperature barely varied).

| Session | Cooling | Temp median / max (C) | Min at 80 C+ | Time: none / soft limit / throttling | Pictures a second: none / soft / throttling (median) | Picture age 95% (s) | rho temp vs pictures |
|---|---|---|---|---|---|---|---|
"""


def state(flags):
    try:
        v = int(str(flags), 16)
    except ValueError:
        return "none"
    if v & 0b0110:
        return "throttling"
    if v & 0b1000:
        return "soft limit"
    return "none"


def spearman(a, b):
    ra, rb = pd.Series(a).rank(), pd.Series(b).rank()
    if ra.std() == 0 or rb.std() == 0:
        return float("nan")
    return float(ra.corr(rb))


def summarise(folder, cooling="none (bare Pi)"):
    name = os.path.basename(os.path.normpath(folder))
    c = pd.read_csv(os.path.join(folder, "cpu.csv"))
    p = pd.read_csv(os.path.join(folder, "perf.csv"))
    c = c.dropna(subset=["temp_c"]).sort_values("t")
    p = p.sort_values("t")
    j = pd.merge_asof(c, p[["t", "fps_handled", "age_max_s", "age_mean_s"]], on="t", direction="nearest",
                      tolerance=1.5)
    j["state"] = j["throttled"].map(state)
    dt = c["t"].diff().median() or 2.0
    out = dict(session=name, cooling=cooling, rows=len(j),
               temp_med=j.temp_c.median(), temp_max=j.temp_c.max(),
               hot_min=(j.temp_c >= HOT_C).sum() * dt / 60.0)
    for s in ("none", "soft limit", "throttling"):
        sub = j[j.state == s]
        out["share_" + s] = len(sub) / max(1, len(j))
        out["fps_" + s] = sub.fps_handled.median() if len(sub) else float("nan")
    out["age95"] = j.age_max_s.quantile(0.95)
    jj = j.dropna(subset=["fps_handled"])
    out["rho"] = spearman(jj.temp_c, jj.fps_handled)
    return out, j


def fmt(x, d=1):
    return "-" if x is None or (isinstance(x, float) and math.isnan(x)) else ("%.*f" % (d, x))


def row(o):
    return ("| %s | %s | %s / %s | %s | %s / %s / %s %% | %s / %s / %s | %s | %s |\n" % (
        o["session"], o["cooling"], fmt(o["temp_med"]), fmt(o["temp_max"]), fmt(o["hot_min"], 0),
        fmt(100 * o["share_none"], 0), fmt(100 * o["share_soft limit"], 0), fmt(100 * o["share_throttling"], 0),
        fmt(o["fps_none"]), fmt(o["fps_soft limit"]), fmt(o["fps_throttling"]), fmt(o["age95"], 2),
        fmt(o["rho"], 2)))


def chart(o, j):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(CHART_DIR, exist_ok=True)
    t0 = j.t.iloc[0]
    m = (j.t - t0) / 60.0
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(m, j.temp_c, color="#C8841F", lw=1.5, label="Pi temperature (C)")
    ax.set_xlabel("minutes into the session")
    ax.set_ylabel("Pi temperature (C)", color="#9A5F0C")
    thr = j.state == "throttling"
    ax.scatter(m[thr], j.temp_c[thr], s=6, color="#B23B3B", label="throttling now")
    ax2 = ax.twinx()
    ax2.plot(m, j.fps_handled.rolling(15, min_periods=1).median(), color="#2F7F78", lw=1.2,
             label="camera pictures a second (30 s median)")
    ax2.set_ylabel("pictures a second handled (of ~25)", color="#2F7F78")
    ax2.set_ylim(0, 30)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="lower right", fontsize=8)
    ax.set_title("%s: the Pi's temperature and the camera's speed" % o["session"])
    fig.text(0.01, 0.01, "Data: the session's cpu.csv (temperature, throttle flags, every 2 s) and perf.csv (pictures "
             "handled, every second).", fontsize=8)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    path = os.path.join(CHART_DIR, o["session"] + ".png")
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def append(o):
    text = open(TRACKER).read() if os.path.exists(TRACKER) else HEADER
    lines = [ln for ln in text.splitlines(True) if not ln.startswith("| %s |" % o["session"])]
    text = "".join(lines)
    if not text.endswith("\n"):
        text += "\n"
    marker = "|---|"
    i = text.find(marker)
    end = text.find("\n## ", i) if i >= 0 else -1
    r = row(o)
    text = (text[:end] + r + text[end:]) if end > 0 else text + r
    os.makedirs(os.path.dirname(TRACKER), exist_ok=True)
    with open(TRACKER, "w") as f:
        f.write(text)


def main(argv):
    cooling = "none (bare Pi)"
    folders = []
    for a in argv:
        if a.startswith("--cooling="):
            cooling = a.split("=", 1)[1]
        else:
            folders.append(a)
    if not folders:
        print(__doc__)
        return 1
    for f in folders:
        o, j = summarise(f, cooling)
        append(o)
        print(row(o).strip())
        print("chart:", chart(o, j))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
