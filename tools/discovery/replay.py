"""Blind discovery replay over every session on disk (logs only, no board time).

    python3 -m tools.discovery.replay              (from the repo root; or python3 tools/discovery/replay.py)
    python3 tools/discovery/replay.py --quick      (skip the disguise reruns)

For each robot in each session: hide the column names, run the discoverer, then score against the key. Splits
keep robots and dates apart so the rules are not judged on the data they were written on:
  dev     = Left and Center, sessions 2026-10-21 to 10-23 (the only cases looked at while writing the rules)
  test_robot = Right, 10-21 to 10-23 (a robot never looked at)
  test_date  = Left and Center, 10-24 (sessions recorded after the rules' data)
  test_both  = Right, 10-24
Kinds: 'rich' (5000+ rows: finds, routes, calibrations) and 'short' (the 9 s calibration logs, about 450 rows).

Writes tools/discovery/results/replay_streams.csv, replay_summary.json and the confusion chart PNG."""
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

from discovery import load, features as F, relations as REL, classify as C, calibrate as CAL   # noqa: E402
from discovery import truth as T, disc_score as S                                                    # noqa: E402

OUT = os.path.join(HERE, "results")
REPO = os.path.dirname(os.path.dirname(HERE))
RICH_ROWS = 5000


def split_of(session, robot):
    day = session.split("-")[1]
    late = day >= "20261024"
    if robot == "Right":
        return "test_both" if late else "test_robot"
    return "test_date" if late else "dev"


def discover(case):
    """The discoverer: features -> relations -> types -> calibration. Sees no names."""
    g, feats, cam_speed, num = F.case_features(case)
    R = REL.analyse(case, g, feats, num)
    res = C.classify_case(feats, R)
    cal = CAL.calibrate(feats, res)
    return feats, R, res, cal


def disguise(case, key, seed, delay_rows=0):
    """The same case with every numeric stream rescaled and some sign-flipped (units disguised), and
    optionally delayed by a few rows. Returns a new Case (same key)."""
    rng = np.random.default_rng(seed)
    out = {}
    for s, vals in case.streams.items():
        x, is_text = F.parse(vals)
        if is_text or key[s] in ("ms", "raw_n"):
            out[s] = vals
            continue
        a = rng.choice([0.1, 10.0, 57.2958, 1 / 57.2958, 2.54, 1000.0]) * rng.choice([1.0, -1.0])
        y = a * x          # rescaled and sign-flipped (design doc test 2); offsets are not disguised: a steady
        #                    level (gravity on one axis) is part of what identifies an accelerometer
        if delay_rows:
            y = np.r_[np.full(delay_rows, y[0]), y[:-delay_rows]]
        out[s] = np.array(["%.6g" % v if np.isfinite(v) else "" for v in y], dtype=object)
    c2 = load.Case(case.case_id + "/disguised", case.session, case.robot, case.t, out, case.legs, case.cam,
                   case.board, case.fw)
    return c2


def rows_for(case, key, feats, res, cal, split, kind, variant="plain"):
    rows = []
    for s, r in res.items():
        col = key[s]
        const = bool(feats[s].get("const"))
        c = cal.get(s, {})
        rows.append({
            "case": case.case_id, "session": case.session, "robot": case.robot, "split": split, "kind": kind,
            "variant": variant, "stream": s, "column": col, "truth": T.truth_for(col, const),
            "guess": r["type"], "confidence": round(r["confidence"], 4), "decision": r["decision"],
            "top3": r["top3"], "derived": bool(r["derived"]),
            "derived_of": [key[x] for x in r["derived_of"] if x in key],
            "evidence": " | ".join(r["evidence"]),
            "cal_reference": c.get("reference", ""), "cal_per_unit": c.get("physical_per_unit"),
            "cal_unit": c.get("physical_unit", ""), "cal_unit_guess": c.get("unit_guess", ""),
            "cal_scale_after_unit": c.get("scale_after_unit"), "cal_offset": c.get("offset_units"),
            "cal_quality": c.get("quality"),
        })
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--quick", action="store_true", help="skip the disguise reruns")
    ap.add_argument("--root", default=load.ROOT)
    ap.add_argument("--only", default="", help="comma list of session ids")
    ap.add_argument("--splits", default="", help="comma list of splits to run (default all)")
    args = ap.parse_args(argv)
    os.makedirs(OUT, exist_ok=True)
    cases = load.all_cases(args.root)
    if args.only:
        keep = set(args.only.split(","))
        cases = [c for c in cases if c[0] in keep]
    if args.splits:
        keep = set(args.splits.split(","))
        cases = [c for c in cases if split_of(c[0], c[2]) in keep]
    rows, timing, structure = [], [], []
    t_all = time.time()
    for sid, sdir, robot, n in cases:
        t0 = time.time()
        case, key = load.load_case(sdir, robot)
        split, kind = split_of(sid, robot), ("rich" if n >= RICH_ROWS else "short")
        feats, R, res, cal = discover(case)
        rows += rows_for(case, key, feats, res, cal, split, kind)
        structure.append({"case": case.case_id, "split": split, "kind": kind, "rows": n,
                          "legs_used": max([(f.get("turn_n", 0) or 0) + (f.get("go_n", 0) or 0)
                                            for f in feats.values()] + [0]), "cam_episodes":
                          max([f.get("cam_episodes", 0) or 0 for f in feats.values()] + [0]),
                          "turn_pairs": max([f.get("cam_turn_n", 0) or 0 for f in feats.values()] + [0]),
                          "groups": [[key[s] for s in gr] for gr in R["groups"] if len(gr) > 1],
                          "integration": [(key[a], key[b], round(r, 4)) for a, b, r in R["integration"]],
                          "combos": [(key[a], key[p], key[q], round(r2, 4)) for a, p, q, r2 in R["combos"]],
                          "planar": [(round(p[0], 4), key[p[1]], key[p[2]], key[p[3]]) for p in R["planar"][:1]],
                          "grouping": R.get("grouping"), "moving_frac": R.get("moving_frac")})
        if not args.quick and split != "dev" and kind == "rich":
            for variant, delay in (("disguised", 0), ("disguised+delay", 5)):
                c2 = disguise(case, key, seed=zlib.crc32(case.case_id.encode()) % 1000 + delay, delay_rows=delay)
                f2, R2, res2, cal2 = discover(c2)
                rows += rows_for(c2, key, f2, res2, cal2, split, kind, variant=variant)
        timing.append((case.case_id, n, round(time.time() - t0, 1)))
        print("%-40s %-10s %-5s %6d rows  %5.1f s" % (case.case_id, split, kind, n, time.time() - t0), flush=True)
    summary = summarise(rows)
    summary["structure"] = structure
    summary["timing_s"] = timing
    summary["total_s"] = round(time.time() - t_all, 1)
    with open(os.path.join(OUT, "replay_streams.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            r2 = dict(r)
            r2["top3"] = ";".join("%s:%.2f" % tv for tv in r["top3"])
            r2["derived_of"] = ";".join(r["derived_of"])
            w.writerow(r2)
    with open(os.path.join(OUT, "replay_summary.json"), "w") as f:
        json.dump(summary, f, indent=1, default=float)
    print(json.dumps(summary["headline"], indent=1, default=float))
    return rows, summary


def summarise(rows):
    out = {"headline": {}, "by_split": {}, "calibration": {}, "sweep": {}, "derived": {}, "per_column": {},
           "confusion": {}}
    plain = [r for r in rows if r["variant"] == "plain"]
    groups = {"all": plain,
              "dev": [r for r in plain if r["split"] == "dev"],
              "test": [r for r in plain if r["split"] != "dev"]}
    for kind in ("rich", "short"):
        groups["dev_" + kind] = [r for r in groups["dev"] if r["kind"] == kind]
        groups["test_" + kind] = [r for r in groups["test"] if r["kind"] == kind]
    for sp in ("test_robot", "test_date", "test_both"):
        for kind in ("rich", "short"):
            groups["%s_%s" % (sp, kind)] = [r for r in plain if r["split"] == sp and r["kind"] == kind]
    for v in ("disguised", "disguised+delay"):
        groups[v] = [r for r in rows if r["variant"] == v]
    for k, g in groups.items():
        ok, n = S.accuracy(g)
        acc = [r for r in g if r["decision"] == "accept"]
        ok_a = sum(r["truth"] == r["guess"] for r in acc)
        out["by_split"][k] = {"right": ok, "streams": n, "share": ok / n if n else None,
                              "accepted": len(acc), "accepted_right": ok_a,
                              "asked": sum(r["decision"] == "ask" for r in g),
                              "unknown": sum(r["decision"] == "unknown" for r in g),
                              "cases": len({r["case"] for r in g})}
    for k in ("dev_rich", "test_rich", "test_short", "dev_short"):
        out["calibration"][k] = S.calibration(groups[k])
        out["sweep"][k] = S.sweep(groups[k])
        out["derived"][k] = S.derived_scores(groups[k])
        out["per_column"][k] = S.per_column(groups[k])
        types, M = S.confusion(groups[k])
        out["confusion"][k] = {"types": types, "matrix": M.tolist()}
    # disguise agreement: same label as the plain run, stream by stream
    plain_by = {(r["case"], r["column"]): r["guess"] for r in plain}
    for v in ("disguised", "disguised+delay"):
        g = groups[v]
        same = sum(plain_by.get((r["case"].replace("/disguised", ""), r["column"])) == r["guess"] for r in g)
        out["by_split"][v]["same_as_plain"] = same
    out["headline"] = {k: out["by_split"][k] for k in ("dev_rich", "test_rich", "test_short", "disguised",
                                                       "disguised+delay")}
    return out


if __name__ == "__main__":
    main()
