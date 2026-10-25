"""Acceptance-threshold experiment for blind discovery (logs only, no board time).

    python3 tools/discovery/threshold_doe.py            # run the discoverer on every log piece (about 20-40 min)
    python3 tools/discovery/threshold_doe.py --analyse  # only the analysis, from results/threshold_streams.csv

Factors:
  accept threshold   0.70 / 0.75 / 0.80 / 0.85 / 0.90 / 0.95 (applied after discovery: one run per log piece)
  motion in the log  seconds the wheels turned. Rich cases are cut into contiguous pieces holding 10, 20, 40, 80 or
                     160 s of motion (2 pieces per case and level, different random starts), plus the whole case;
                     the 9 s calibration logs come as they are (about 0-3 s of motion)
  disguise           plain / every numeric stream rescaled (x0.1, 10, 57.3, 1/57.3, 2.54, 1000) and half sign-flipped
Responses per robot (one log piece): wrong accepted labels, questions (confidence between the ask bar 0.60 and the
threshold), unknowns (below 0.60, or guessed "unknown").

Held-out discipline: the rule is chosen on the development cases (Left and Center, 10-21 to 10-23) only, then
checked once on the held-out cases (Right; all robots on 10-24). The criterion is fixed in CRITERION below,
before the held-out numbers are looked at.

Motion seconds come from the true wheel-angle columns (the experimenter reads the key; the discoverer does not).
A real discovery run knows its motion seconds without the key: it commanded them."""
import argparse
import csv
import json
import os
import sys
import time
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from discovery import load, features as F, replay as RP, truth as T          # noqa: E402

OUT = os.path.join(HERE, "results")
THRESHOLDS = (0.70, 0.75, 0.80, 0.85, 0.90, 0.95)
ASK = 0.60
LEVELS = (10, 20, 40, 80, 160)          # seconds of motion per piece (chosen: doubling steps)
PIECES = 2                              # pieces per case and level (chosen)
BINS = ((0, 5, "0-5 s"), (5, 15, "~10 s"), (15, 30, "~20 s"), (30, 60, "~40 s"), (60, 120, "~80 s"),
        (120, 240, "~160 s"), (240, 1e9, "240 s +"))
# The rule's criterion, fixed before looking at held-out numbers: at each motion level, the lowest threshold whose
# development wrong-accepted share is at most 1% (point) and whose 95% upper bound is at most 3%.
CRITERION = {"point": 0.01, "upper": 0.03}


def motion_per_row(case, key):
    """Seconds each row counts as motion: the true wheel angles changed since the previous row (dt capped at
    0.2 s so logging gaps do not count)."""
    inv = {v: k for k, v in key.items()}
    n = len(case.t)
    mv = np.zeros(n, bool)
    for col in ("wl_deg", "wr_deg"):
        if col not in inv:
            continue
        x, _ = F.parse(case.streams[inv[col]])
        d = np.r_[0.0, np.abs(np.diff(x))]
        mv |= np.nan_to_num(d) > 0
    dt = np.r_[0.0, np.diff(case.t)]
    dt = np.clip(dt, 0, 0.2)
    return np.where(mv, dt, 0.0)


def piece(case, i0, i1, tag):
    """Rows i0..i1-1 of a case as a new Case, with the legs and camera points inside that time."""
    ta, tb = case.t[i0], case.t[i1 - 1]
    streams = {s: v[i0:i1] for s, v in case.streams.items()}
    legs = [L for L in case.legs if L.t0 >= ta and L.t1 <= tb]
    cam = case.cam
    if cam is not None and len(cam):
        cam = cam[(cam[:, 0] >= ta - 1.0) & (cam[:, 0] <= tb + 1.0)]
    return load.Case(case.case_id + "/" + tag, case.session, case.robot, case.t[i0:i1], streams, legs, cam,
                     case.board, case.fw)


def cut(case, key, target_s, seed):
    """A contiguous piece holding target_s seconds of motion, from a random start; None if the case has less."""
    m = motion_per_row(case, key)
    cm = np.cumsum(m)
    if cm[-1] < target_s * 1.05:
        return None, 0.0
    rng = np.random.default_rng(seed)
    last_start = int(np.searchsorted(cm, cm[-1] - target_s))   # latest start that still leaves target_s after it
    for _ in range(30):
        i0 = int(rng.integers(0, max(1, last_start)))
        i1 = int(np.searchsorted(cm, cm[i0] + target_s)) + 1
        if i1 <= len(case.t):
            return (i0, i1), float(cm[i1 - 1] - cm[i0])
    return None, 0.0


def blind_motion(feats, R):
    """What the discoverer itself can say about motion: share of rows moving x duration, and its moving 1 s
    windows (both from the streams that are flat at rest)."""
    dur = max([f.get("duration_s", 0) or 0 for f in feats.values()] + [0])
    mf = R.get("moving_frac", np.nan)
    mw = max([f.get("moving_windows", 0) or 0 for f in feats.values()] + [0])
    return (float(mf * dur) if np.isfinite(mf) else np.nan), int(mw)


def run_pieces(args):
    cases = load.all_cases(args.root)
    out_rows, t_all = [], time.time()
    for sid, sdir, robot, n in cases:
        case, key = load.load_case(sdir, robot)
        split = RP.split_of(sid, robot)
        kind = "rich" if n >= RP.RICH_ROWS else "short"
        jobs = []
        if kind == "short":
            m = motion_per_row(case, key).sum()
            jobs.append(("whole", case, m))
        else:
            for lev in LEVELS:
                for k in range(PIECES):
                    seed = zlib.crc32(("%s/%d/%d" % (case.case_id, lev, k)).encode())
                    ij, m = cut(case, key, lev, seed)
                    if ij is None:
                        continue
                    jobs.append(("m%03d_%d" % (lev, k), piece(case, ij[0], ij[1], "m%03d_%d" % (lev, k)), m))
            jobs.append(("whole", case, motion_per_row(case, key).sum()))
        for tag, c, msec in jobs:
            for variant in ("plain", "disguised"):
                t0 = time.time()
                cc = c if variant == "plain" else RP.disguise(c, key, seed=zlib.crc32(c.case_id.encode()) % 1000)
                try:
                    feats, R, res, cal = RP.discover(cc)
                except Exception as e:          # a piece too small for some relation: record and go on
                    print("FAILED %s %s: %r" % (c.case_id, variant, e), flush=True)
                    continue
                bm, bw = blind_motion(feats, R)
                for s, r in res.items():
                    col = key[s]
                    out_rows.append({
                        "case": case.case_id, "piece": tag, "split": split, "kind": kind, "variant": variant,
                        "motion_s": round(msec, 2), "blind_motion_s": round(bm, 2) if np.isfinite(bm) else "",
                        "blind_moving_windows": bw, "rows": len(c.t),
                        "duration_s": round(float(c.t[-1] - c.t[0]), 1), "column": col,
                        "truth": T.truth_for(col, bool(feats[s].get("const"))), "guess": r["type"],
                        "confidence": round(r["confidence"], 4),
                        "top3": ";".join(t for t, v in r["top3"])})
                print("%-50s %-9s %-10s motion %6.1f s (blind %6.1f)  %5.1f s" %
                      (c.case_id, variant, split, msec, bm if np.isfinite(bm) else -1, time.time() - t0), flush=True)
    path = os.path.join(OUT, "threshold_streams.csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out_rows[0].keys()))
        w.writeheader()
        w.writerows(out_rows)
    print("wrote %s (%d rows) in %.0f s" % (path, len(out_rows), time.time() - t_all))


# ------------------------------------------------------------------ analysis

def load_rows(path=None):
    path = path or os.path.join(OUT, "threshold_streams.csv")
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r["confidence"] = float(r["confidence"])
        r["motion_s"] = float(r["motion_s"])
    return rows


def bin_of(m):
    for lo, hi, name in BINS:
        if lo <= m < hi:
            return name
    return BINS[-1][2]


def per_robot(rows, thr):
    """{(case, piece): (accepted, wrong, questions, unknown, not_offered)} at one threshold."""
    out = {}
    for r in rows:
        k = (r["case"], r["piece"])
        a = out.setdefault(k, [0, 0, 0, 0, 0])
        g, c = r["guess"], r["confidence"]
        if g != "unknown" and c >= thr:
            a[0] += 1
            a[1] += r["truth"] != g
        elif g != "unknown" and c >= ASK:
            a[2] += 1
            a[4] += r["truth"] not in r["top3"].split(";")
        else:
            a[3] += 1
    return out


def boot(rows, thr, n_boot=2000, seed=1):
    """Point values and 95% bootstrap intervals, resampling cases (robots in sessions), not streams: streams of
    one robot are not independent. Pieces of one case travel together."""
    pr = per_robot(rows, thr)
    cases = sorted({k[0] for k in pr})
    by_case = {c: np.array([v for k, v in pr.items() if k[0] == c], float) for c in cases}

    def stats(cs):
        M = np.vstack([by_case[c] for c in cs])
        acc, wr = M[:, 0].sum(), M[:, 1].sum()
        return np.array([wr / acc if acc else 0.0, M[:, 1].mean(), M[:, 2].mean(), M[:, 3].mean(), M[:, 0].mean()])
    point = stats(cases)
    rng = np.random.default_rng(seed)
    B = np.array([stats(rng.choice(cases, len(cases))) for _ in range(n_boot)])
    lo, hi = np.percentile(B, 2.5, axis=0), np.percentile(B, 97.5, axis=0)
    M = np.vstack(list(by_case.values()))
    acc_total, wrong_total = int(M[:, 0].sum()), int(M[:, 1].sum())
    # with no wrong labels at all the bootstrap interval is [0, 0], which overstates certainty; use the "rule of
    # three" bound 3 / n instead (95% upper bound for a rate when 0 of n were seen)
    upper_wrong = hi[0] if wrong_total else (3.0 / acc_total if acc_total else 1.0)
    return {"n_cases": len(cases), "n_pieces": len(pr), "accepted_total": acc_total, "wrong_total": wrong_total,
            "wrong_share": point[0], "wrong_share_lo": lo[0], "wrong_share_hi": upper_wrong,
            "wrong_per_robot": point[1], "wrong_per_robot_lo": lo[1], "wrong_per_robot_hi": hi[1],
            "questions_per_robot": point[2], "questions_lo": lo[2], "questions_hi": hi[2],
            "unknown_per_robot": point[3], "unknown_lo": lo[3], "unknown_hi": hi[3],
            "accepted_per_robot": point[4],
            "question_truth_not_offered": int(M[:, 4].sum())}


def analyse(path=None):
    rows = load_rows(path)
    for r in rows:
        r["bin"] = bin_of(r["motion_s"])
    out = {"thresholds": THRESHOLDS, "criterion": CRITERION, "cells": [], "rule": {}, "blind_vs_true": {}}
    bins = [b[2] for b in BINS]
    for side in ("dev", "heldout"):
        for variant in ("plain", "disguised"):
            for b in bins:
                sub = [r for r in rows if r["variant"] == variant and r["bin"] == b and
                       ((r["split"] == "dev") == (side == "dev"))]
                if not sub:
                    continue
                for thr in THRESHOLDS:
                    st = boot(sub, thr)
                    st.update(side=side, variant=variant, bin=b, threshold=thr)
                    out["cells"].append(st)
    # the rule, from development cells only
    for variant in ("plain", "disguised"):
        rule = {}
        for b in bins:
            cells = [c for c in out["cells"] if c["side"] == "dev" and c["variant"] == variant and c["bin"] == b]
            ok = [c for c in cells if c["accepted_total"] > 0 and c["wrong_share"] <= CRITERION["point"]
                  and c["wrong_share_hi"] <= CRITERION["upper"]]
            rule[b] = min(c["threshold"] for c in ok) if ok else None
        out["rule"][variant] = rule
    # how well the discoverer's own motion estimate tracks the true motion seconds (plain pieces)
    seen = {}
    for r in rows:
        if r["variant"] == "plain" and r["blind_motion_s"] != "":
            seen[(r["case"], r["piece"])] = (r["motion_s"], float(r["blind_motion_s"]))
    if seen:
        a = np.array(list(seen.values()))
        out["blind_vs_true"] = {"n": len(a), "r_log": float(np.corrcoef(np.log1p(a[:, 0]), np.log1p(a[:, 1]))[0, 1]),
                                "median_ratio": float(np.median((a[:, 1] + 1) / (a[:, 0] + 1)))}
    with open(os.path.join(OUT, "threshold_summary.json"), "w") as f:
        json.dump(out, f, indent=1, default=float)
    return out


PASSIVE = {"clock", "counter", "id", "constant", "battery", "mode", "switch"}
GATES = (0, 5, 60, 120, 160, 240)       # seconds of motion before a motion-type label may be accepted


def decide_gated(r, thr, gate):
    """Decision under a rule: passive types (clock, counter, id, constant, battery, step index, switch) are
    accepted at thr whatever the motion; every other type is only asked about until the log holds `gate` s of
    motion, then accepted at thr."""
    g, c = r["guess"], r["confidence"]
    if g == "unknown" or c < ASK:
        return "unknown"
    if g not in PASSIVE and r["motion_s"] < gate:
        return "ask"
    return "accept" if c >= thr else "ask"


def boot_rule(rows, thr, gate, n_boot=2000, seed=2):
    """Wrong share among accepted and questions per robot under one rule, with 95% bootstrap intervals over
    cases (as boot())."""
    per = {}
    for r in rows:
        a = per.setdefault((r["case"], r["piece"]), [0, 0, 0])
        d = decide_gated(r, thr, gate)
        if d == "accept":
            a[0] += 1
            a[1] += r["truth"] != r["guess"]
        elif d == "ask":
            a[2] += 1
    cases = sorted({k[0] for k in per})
    by = {c: np.array([v for k, v in per.items() if k[0] == c], float) for c in cases}

    def st(cs):
        M = np.vstack([by[c] for c in cs])
        return np.array([M[:, 1].sum() / max(M[:, 0].sum(), 1), M[:, 2].mean(), M[:, 1].mean()])
    p = st(cases)
    rng = np.random.default_rng(seed)
    B = np.array([st(rng.choice(cases, len(cases))) for _ in range(n_boot)])
    lo, hi = np.percentile(B, 2.5, axis=0), np.percentile(B, 97.5, axis=0)
    M = np.vstack(list(by.values()))
    acc, wr = int(M[:, 0].sum()), int(M[:, 1].sum())
    return {"threshold": thr, "gate_s": gate, "robots": len(per), "cases": len(cases), "accepted": acc, "wrong": wr,
            "wrong_share": p[0], "wrong_share_lo": lo[0], "wrong_share_hi": hi[0] if wr else 3.0 / max(acc, 1),
            "questions_per_robot": p[1], "questions_lo": lo[1], "questions_hi": hi[1], "wrong_per_robot": p[2]}


def analyse_rules(path=None):
    """Rules = threshold x motion gate. Chosen on development pieces by CRITERION (both plain and disguised must
    pass; among those, fewest questions), then reported on held-out pieces."""
    rows = load_rows(path)
    out = {"rules": [], "chosen": None}
    for side in ("dev", "heldout"):
        for variant in ("plain", "disguised"):
            sub = [r for r in rows if r["variant"] == variant and ((r["split"] == "dev") == (side == "dev"))]
            for thr in (0.80, 0.90, 0.95):
                for gate in GATES:
                    st = boot_rule(sub, thr, gate)
                    st.update(side=side, variant=variant)
                    out["rules"].append(st)
    ok = []
    for thr in (0.80, 0.90, 0.95):
        for gate in GATES:
            d = [c for c in out["rules"] if c["side"] == "dev" and c["threshold"] == thr and c["gate_s"] == gate]
            if all(c["wrong_share"] <= CRITERION["point"] and c["wrong_share_hi"] <= CRITERION["upper"] for c in d):
                ok.append((max(c["questions_per_robot"] for c in d), thr, gate))
    if ok:
        q, thr, gate = min(ok)
        out["chosen"] = {"threshold": thr, "gate_s": gate, "dev_questions_per_robot": q}
    with open(os.path.join(OUT, "threshold_rules.json"), "w") as f:
        json.dump(out, f, indent=1, default=float)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--analyse", action="store_true")
    ap.add_argument("--root", default=load.ROOT)
    args = ap.parse_args(argv)
    if not args.analyse:
        run_pieces(args)
    out = analyse()
    for v in ("plain", "disguised"):
        print(v, "threshold by motion bin (dev):", out["rule"][v])
    ru = analyse_rules()
    print("rule chosen on dev (threshold x motion gate):", ru["chosen"])


if __name__ == "__main__":
    main()
