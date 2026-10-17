#!/usr/bin/env python3
"""Measure the motion detector and tracker against a synthetic recording's
truth.csv (see synth.py).

    python tools/label/tests/evaluate.py <recording folder> [more folders...]
    python tools/label/tests/evaluate.py --make <scratch folder>   # make both scenarios, then measure

Numbers reported (robots = truth class alvik):
  detection rate   share of true robot positions with a guess within match_px
  false guesses    guesses not near any true object, per 100 frames
  centre error     distance guess -> true centre, px (median / 95th percentile)
  heading error    |guess - true heading| for guesses given as known, degrees
  heading coverage share of matched robot guesses that give a known heading
  identity switches times a true robot's guess changes object number
"""
import argparse
import collections
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import motion    # noqa: E402
import tracker   # noqa: E402


def load_truth(folder):
    by_frame = collections.defaultdict(list)
    with open(os.path.join(folder, "truth.csv"), newline="") as f:
        for r in csv.DictReader(f):
            by_frame[int(r["frame"])].append(dict(
                obj=int(r["obj"]), cls=r["cls"], x=float(r["x"]), y=float(r["y"]),
                heading=float(r["heading_deg"]) if r["heading_deg"] else None, phase=r["phase"]))
    return by_frame


def match(truth, guesses, max_px, hand_px=60.0):
    """Pair true objects with guesses, closest pairs first. Returns list of
    (truth index, guess index, distance). A hand's true point is its palm; a
    guess anywhere on hand + arm (within hand_px) counts."""
    pairs = []
    for i, t in enumerate(truth):
        lim = hand_px if t["cls"] == "hand" else max_px
        for j, g in enumerate(guesses):
            d = math.hypot(t["x"] - g["x"], t["y"] - g["y"])
            if d <= lim:
                pairs.append((d, i, j))
    pairs.sort()
    ut, ug, out = set(), set(), []
    for d, i, j in pairs:
        if i in ut or j in ug:
            continue
        ut.add(i)
        ug.add(j)
        out.append((i, j, d))
    return out


def pct(values, q):
    if not values:
        return float("nan")
    v = sorted(values)
    return v[min(len(v) - 1, int(round(q / 100.0 * (len(v) - 1))))]


def score(truth_by_frame, guesses_by_frame, n_frames, match_px=15.0, start_frame=0):
    """Compare guesses with truth. guesses_by_frame: dict or list frame -> list of dicts
    with x, y, cls, and optionally obj, heading_deg, heading_guess."""
    n_true = n_hit = n_fp = 0
    by_phase = collections.defaultdict(lambda: [0, 0])
    errs, herrs, hguess_errs = [], [], []
    n_hknown = 0
    last_id = {}
    switches = 0
    hand_true = hand_hit = 0
    cls_right = cls_total = 0
    for f in range(start_frame, n_frames):
        truth = truth_by_frame.get(f, [])
        g = guesses_by_frame.get(f, []) if isinstance(guesses_by_frame, dict) else guesses_by_frame[f]
        pairs = match(truth, g, match_px)
        matched_g = {j for (_, j, _) in pairs}
        n_fp += len(g) - len(matched_g)
        hit_t = {i: (j, d) for (i, j, d) in pairs}
        for i, t in enumerate(truth):
            if t["cls"] == "hand":
                hand_true += 1
                if i in hit_t:
                    hand_hit += 1
                continue
            n_true += 1
            by_phase[t["phase"]][1] += 1
            if i not in hit_t:
                continue
            j, d = hit_t[i]
            n_hit += 1
            by_phase[t["phase"]][0] += 1
            errs.append(d)
            gg = g[j]
            cls_total += 1
            cls_right += gg.get("cls") == "alvik"
            if gg.get("heading_deg") is not None:
                n_hknown += 1
                herrs.append(abs(motion.wrap180(gg["heading_deg"] - t["heading"])))
            elif gg.get("heading_guess") is not None:
                hguess_errs.append(abs(motion.wrap180(gg["heading_guess"] - t["heading"])))
            if "obj" in gg:
                if t["obj"] in last_id and last_id[t["obj"]] != gg["obj"]:
                    switches += 1
                last_id[t["obj"]] = gg["obj"]
    frames = max(1, n_frames - start_frame)
    return dict(
        detection_rate=n_hit / max(1, n_true),
        false_per_100_frames=100.0 * n_fp / frames,
        centre_err_median=pct(errs, 50), centre_err_p95=pct(errs, 95),
        heading_coverage=n_hknown / max(1, n_hit),
        heading_err_median=pct(herrs, 50), heading_err_p95=pct(herrs, 95),
        heading_wrong_over_30=sum(e > 30 for e in herrs) / max(1, len(herrs)),
        heading_guess_err_median=pct(hguess_errs, 50), n_heading_guess=len(hguess_errs),
        id_switches=switches, class_right=cls_right / max(1, cls_total),
        hand_detected=hand_hit / max(1, hand_true) if hand_true else None,
        by_phase={k: (v[0] / max(1, v[1]), v[1]) for k, v in sorted(by_phase.items())},
    )


def evaluate(folder, robot_px=None):
    """Run motion detector + tracker on a recording and score both."""
    video = os.path.join(folder, "video.avi")
    dets, info = motion.detect_recording(video, robot_px=robot_px)
    n = len(dets)
    fps = info["fps"]
    times = [f / fps for f in range(n)]
    rows, summary = tracker.run_tracker(dets, times, info["robot_px"])
    truth = load_truth(folder)
    raw = score(truth, dets, n)
    trk = score(truth, rows, n)
    return dict(info=info, motion=raw, tracked=trk, tracks=summary)


def fmt(res):
    lines = []
    for key in ("motion", "tracked"):
        s = res[key]
        lines.append("  %-8s detection %5.1f%%  false/100fr %5.1f  centre err med %.2f px p95 %.2f px  "
                     "heading known %5.1f%% err med %.1f p95 %.1f (>30 deg: %.1f%%)  id switches %d  class ok %.1f%%%s"
                     % (key, 100 * s["detection_rate"], s["false_per_100_frames"], s["centre_err_median"],
                        s["centre_err_p95"], 100 * s["heading_coverage"], s["heading_err_median"],
                        s["heading_err_p95"], 100 * s["heading_wrong_over_30"], s["id_switches"],
                        100 * s["class_right"],
                        "" if s["hand_detected"] is None else "  hand found %.0f%%" % (100 * s["hand_detected"])))
        lines.append("           by phase: " + ", ".join("%s %.0f%% (n=%d)" % (k, 100 * v[0], v[1])
                                                         for k, v in s["by_phase"].items()))
        if s["n_heading_guess"]:
            lines.append("           heading guesses (dashed): n=%d, err med %.1f" %
                         (s["n_heading_guess"], s["heading_guess_err_median"]))
    lines.append("  tracks: " + ", ".join("%d:%s %d-%d" % (t["obj"], t["cls"], t["first"], t["last"])
                                          for t in res["tracks"]))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folders", nargs="*")
    ap.add_argument("--make", help="make the synthetic scenarios in this folder first")
    args = ap.parse_args()
    folders = list(args.folders)
    if args.make:
        import synth
        for name in ("main", "crowd"):
            out = os.path.join(args.make, "synth_" + name)
            if not os.path.exists(os.path.join(out, "truth.csv")):
                synth.make_recording(out, name)
            folders.append(out)
    for folder in folders:
        res = evaluate(folder)
        print("%s  (robot size %.1f px, %s)" % (folder, res["info"]["robot_px"], res["info"]["robot_px_source"]))
        print(fmt(res))


if __name__ == "__main__":
    sys.exit(main())
