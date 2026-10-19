#!/usr/bin/env python3
"""Can the robots see an open plywood edge? Reads the E0c edge rows and recommends
the edge guard settings (firmware edge.h, params safe_edge / edge_*).

    python tools/edge_check.py                                   # docs/experiments/E00c-rig-measurements
    python tools/edge_check.py path/to/experiment --skip-ms 1000

Reads data/measurements.csv (rows with a run_id) and the RAW logs in
data/logs/<session>/raw*.csv. Rows used (all robots standing still):
  plywood    warmup, still60, tof40..tof100, edge_n15, edge_n10, edge_n5, edge_in
             (line sensors on plain plywood; for the bottom zone only the rows
             with nothing within 40 cm ahead count: warmup, still60, tof40+)
  edge_over  line sensors about 1 cm past the open edge
  edge_tape  line sensors on the painter's tape
  edge_n15 / n10 / n5 / edge_in / edge_over   front face 15, 10, 5, ~2.2 cm
             inside the edge, ~2.3 cm past it (bottom zone, dbot_cm)

Line sensors: each robot's plywood level per sensor is the median of its plywood
rows (the firmware learns it at the start of every run, so only the change from
that level matters). The tool finds which way the readings move past the edge,
then whether one threshold (or a window, when tape moves them even further) puts
every edge reading on one side and every plywood and tape reading on the other.
Bottom zone: an absolute limit (the reading only depends on how the sensor is
mounted); the tool finds the furthest distance from the edge at which every
robot's reading has already jumped clear of the plywood readings.

Prints the verdict and the `set` commands (copy them into params.h once a
driving test agrees). The numbers come from one placement per robot and level:
a slow drive over a real open edge must confirm them before safe_edge goes on.
"""
import argparse
import csv
import glob
import os
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
DEFAULT = os.path.join(REPO, "docs", "experiments", "E00c-rig-measurements")
SENSORS = ("lineL", "lineC", "lineR")
LINE_PLY = ("warmup", "still60", "tof40", "tof60", "tof80", "tof100", "edge_n15", "edge_n10", "edge_n5", "edge_in")
BOT_PLY = ("warmup", "still60", "tof40", "tof60", "tof80", "tof100")
# front face to the open edge, cm (+ = still on the board); make_run_order.py / E0c README
BOT_DIST = {"edge_n15": 15.0, "edge_n10": 10.0, "edge_n5": 5.0, "edge_in": 2.2, "edge_over": -2.3}
FRONT_TO_AXLE_CM = 4.76         # make_run_order.py: front face to wheel axle
LINE_TO_AXLE_CM = 3.5           # grid_center_cm


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def read_csv(path):
    with open(path, newline="", encoding="utf-8-sig") as fh:
        return list(csv.DictReader(fh))


def load(folder, skip_ms=1000.0):
    """{(robot, level): {"lineL": [...], ..., "dbot_cm": [...]}} from the sheet and RAW logs."""
    sheet = os.path.join(folder, "data", "measurements.csv")
    if not os.path.isfile(sheet):
        sys.exit(f"no sheet at {sheet}")
    want = {}
    for r in read_csv(sheet):
        rid = (r.get("run_id") or "").strip()
        if not rid or (r.get("excluded") or "").strip().lower() in ("1", "yes", "y", "x", "true"):
            continue
        want[rid] = ((r.get("robot") or "").strip(), (r.get("level") or "").strip())
    raw = {}
    for path in sorted(glob.glob(os.path.join(folder, "data", "logs", "*", "raw*.csv"))):
        base = os.path.basename(path)[:-4]
        if base != "raw" and not (base.startswith("raw_") and base[4:].isdigit()):
            continue
        for row in read_csv(path):
            rid = row.get("run", "")
            if rid in want:
                raw.setdefault(rid, []).append(row)
    out = {}
    for rid, rows in raw.items():
        ms = [num(r.get("ms")) for r in rows]
        t0 = min(m for m in ms if m is not None)
        d = out.setdefault(want[rid], {c: [] for c in SENSORS + ("dbot_cm",)})
        for r, m in zip(rows, ms):
            if m is None or m - t0 < skip_ms or (r.get("lifted") or "0").strip() == "1":
                continue
            for c in d:
                v = num(r.get(c))
                if v is not None and (c != "dbot_cm" or v > 0):     # 0 = no distance reading
                    d[c].append(v)
    return out


def sd(v):
    return st.pstdev(v) if len(v) > 1 else 0.0


# ================================================================ line sensors
def line_cue(data):
    robots = sorted({rb for rb, _ in data})
    res = {"robots": robots, "ok": False, "lines": []}
    say = res["lines"].append
    ply, over, tape, base = [], [], [], {}
    noise = []
    for rb in robots:
        for c in SENSORS:
            p = [v for lv in LINE_PLY for v in data.get((rb, lv), {}).get(c, [])]
            o = data.get((rb, "edge_over"), {}).get(c, [])
            if not p or not o:
                continue
            b = st.median(p)
            base[(rb, c)] = b
            ply += [v - b for v in p]
            over.append([v - b for v in o])
            tape += [v - b for v in data.get((rb, "edge_tape"), {}).get(c, [])]
            noise.append(sd(data.get((rb, "edge_in"), {}).get(c, []) or p))
    if not over:
        say("line sensors: no plywood + edge_over rows with RAW data yet (needs E0c edge_in, edge_over, edge_tape)")
        return res
    meds = [st.median(o) for o in over]
    sign = 1 if st.median(meds) >= 0 else -1
    res["sign"] = sign
    if any(m * sign <= 0 for m in meds):
        say(f"line sensors: the edge moves the readings DIFFERENT ways on different robots/sensors "
            f"(medians {', '.join('%+.0f' % m for m in meds)}): one direction can't catch them all")
        return res
    P = [sign * v for v in ply]
    E = [sign * v for o in over for v in o]
    T = [sign * v for v in tape]
    n_sd = st.median(noise) or 1.0
    p_hi, e_lo, e_hi = max(P), min(E), max(E)
    say(f"line sensors past the edge read {'HIGHER' if sign > 0 else 'LOWER'} than on plywood "
        f"(change from each robot's plywood level: {min(E):+.0f} to {max(E):+.0f} in that direction; "
        f"plywood {min(P):+.0f} to {p_hi:+.0f}; tape {('%+.0f to %+.0f' % (min(T), max(T))) if T else 'no rows'})")
    if not T:
        say("  tape: no edge_tape rows yet, so tape can't be ruled out: CHECK")
    if e_lo <= p_hi:
        say(f"  NOT separable: some edge readings ({e_lo:+.0f}) are within the plywood spread (up to {p_hi:+.0f})")
        return res
    t_lo, t_hi = (min(T), max(T)) if T else (None, None)
    if t_hi is None or t_hi < e_lo:              # tape on the plywood side (or opposite): one threshold
        lo = max(p_hi, t_hi if t_hi is not None else p_hi)
        gap = e_lo - lo
        thr = lo + gap / 3                        # nearer the plywood/tape side: trips early in the edge's blur
        res.update(ok=True, thr=round(thr), mx=0, gap=gap, gap_sd=gap / n_sd)
        say(f"  SEPARABLE: gap of {gap:.0f} between the highest plywood/tape reading and the lowest edge reading "
            f"= {gap / n_sd:.1f} x the plywood noise SD")
    elif t_lo > e_hi:                             # tape moves it even further: a window
        gap_lo, gap_hi = e_lo - p_hi, t_lo - e_hi
        thr = p_hi + gap_lo / 3
        mx = e_hi + gap_hi / 2
        res.update(ok=True, thr=round(thr), mx=round(mx), gap=min(gap_lo, gap_hi), gap_sd=min(gap_lo, gap_hi) / n_sd,
                   window=True)
        say(f"  SEPARABLE ONLY BY A WINDOW: tape moves the reading further ({t_lo:+.0f}+) than the edge "
            f"({e_lo:+.0f} to {e_hi:+.0f}). Driving onto tape passes through the window, so edge_n must outlast "
            f"that crossing: CHECK with a slow drive over the tape before use")
    else:
        say(f"  NOT separable: tape readings ({t_lo:+.0f} to {t_hi:+.0f}) overlap the edge readings")
        return res
    if res["gap_sd"] < 5:
        say(f"  margin is thin ({res['gap_sd']:.1f} SD; 5+ wanted): CHECK")
    say("  (SD = standard deviation of one sensor's readings at one spot: its noise. The gap is how far apart "
        "the two groups' closest readings are, in counts and in those SDs.)")
    return res


# ================================================================ bottom zone
def bottom_cue(data):
    robots = sorted({rb for rb, _ in data})
    res = {"robots": robots, "ok": False, "lines": []}
    say = res["lines"].append
    ply = {rb: [v for lv in BOT_PLY for v in data.get((rb, lv), {}).get("dbot_cm", [])] for rb in robots}
    ply = {rb: v for rb, v in ply.items() if v}
    if not ply:
        say("bottom zone: no plywood rows with dbot readings yet (needs E0c warmup/still60/tof40+)")
        return res
    p_hi = max(max(v) for v in ply.values())
    noise = st.median(sd(v) for v in ply.values()) or 0.3
    margin = max(1.0, 3 * noise)
    say("bottom zone (dbot_cm) on plywood: " + ", ".join(f"{rb} {st.median(v):.1f} (max {max(v):.1f})"
                                                         for rb, v in sorted(ply.items())))
    rows = []
    for lv, dist in sorted(BOT_DIST.items(), key=lambda kv: -kv[1]):
        vals = {rb: data.get((rb, lv), {}).get("dbot_cm", []) for rb in ply}
        if not all(vals.values()):
            continue
        lo = min(min(v) for v in vals.values())
        jumped = lo > p_hi + margin
        rows.append((dist, lo, jumped))
        say(f"  front face {dist:+5.1f} cm from the edge ({lv}): " +
            ", ".join(f"{rb} {st.median(v):.1f}" for rb, v in sorted(vals.items())) +
            f"  lowest {lo:.1f}: {'jumped' if jumped else 'still plywood-like'}")
    if not rows:
        say("  no edge_n15/n10/n5/edge_in/edge_over rows with dbot readings yet")
        return res
    warn = None
    for dist, lo, jumped in rows:                 # furthest distance from which it stays jumped all the way in
        if jumped and all(j for d, _, j in rows if d <= dist):
            warn = (dist, lo)
            break
    if warn is None:
        say(f"  the bottom zone does NOT see the edge reliably at any measured distance (plywood max {p_hi:.1f}, "
            f"wanted {margin:.1f} cm clear of it): don't use safe_edge 2")
        return res
    thr = p_hi + (warn[1] - p_hi) / 2
    res.update(ok=True, warn_cm=warn[0], thr=round(thr, 1), gap=warn[1] - p_hi)
    say(f"  SEES THE EDGE from {warn[0]:.0f} cm (front face; wheel axle {warn[0] + FRONT_TO_AXLE_CM:.1f} cm) or closer: "
        f"limit {thr:.1f} cm (halfway between plywood max {p_hi:.1f} and the lowest reading there {warn[1]:.1f}). "
        f"It may already warn further out: only 15/10/5 cm were measured")
    say("  CHECK: the robot pitches when it speeds up or brakes, which tilts this zone; a slow and a fast drive "
        "over plain plywood must not trip it")
    return res


def recommend(line, bot):
    mode = (1 if line.get("ok") else 0) | (2 if bot.get("ok") else 0)
    cmds = [f"set safe_edge {mode}"]
    if line.get("ok"):
        cmds += [f"set edge_dir {line['sign']}", f"set edge_thr {line['thr']}", f"set edge_max {line['mx']}"]
        if line.get("window"):
            cmds.append("set edge_n 6")
    if bot.get("ok"):
        cmds.append(f"set edge_bot_cm {bot['thr']}")
    return mode, cmds


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder", nargs="?", default=DEFAULT, help="experiment folder (default: E0c)")
    ap.add_argument("--skip-ms", type=float, default=1000.0, help="ignore each run's first ms (being placed)")
    a = ap.parse_args(argv)
    data = load(a.folder, a.skip_ms)
    if not data:
        print("no RAW rows for any sheet row with a run_id yet: run the E0c edge rows first")
        return 1
    line, bot = line_cue(data), bottom_cue(data)
    for s in line["lines"] + [""] + bot["lines"]:
        print(s)
    mode, cmds = recommend(line, bot)
    print()
    if mode == 0:
        print("VERDICT: neither cue separates the edge reliably: keep safe_edge 0")
    else:
        which = {1: "line sensors only", 2: "bottom zone only", 3: "both cues"}[mode]
        stop = (f"the line cue stops with about {LINE_TO_AXLE_CM - 0.6:.1f} cm of wheel left on the board "
                f"(simulated, head-on)" if mode & 1 else "")
        print(f"VERDICT: {which} can see the open edge. Try these on one robot, driving slowly at an open edge "
              f"and over the tape, before putting them in params.h{'; ' + stop if stop else ''}:")
        for c in cmds:
            print("  " + c)
    return 0


if __name__ == "__main__":
    sys.exit(main())
