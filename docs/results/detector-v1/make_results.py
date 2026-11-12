#!/usr/bin/env python3
"""Detector-v1 results on the held-out time block (11 Nov home run, 11:11-12:54): every labelled robot scored, four ways
(as recorded, roof blurred, a made-up tag pasted on the roof, roof flattened), float and 8-bit models; charts.

    python3 docs/results/detector-v1/make_results.py --run <run folder> --model <best.onnx> [--int8 <best.int8.onnx>] \
        --variant clean=<labels root> --variant blur=<dir> --variant tag=<dir> --variant flat=<dir> \
        --test <rec,rec,...> --out docs/results/detector-v1

The labels roots are what tools/detector/autolabel.py --pictures and roofmask.py wrote (scratch, not in git).
Writes results.json, per_robot.csv and the charts next to this file.

Changelog
  2026-11-12 dv1  first version
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DET = os.path.join(HERE, "..", "..", "..", "tools", "detector")
sys.path.insert(0, os.path.abspath(DET))
import autolabel  # noqa: E402
import detcore  # noqa: E402
import infer  # noqa: E402

import cv2  # noqa: E402

COL = {"clean": "#2a78d6", "blur": "#eb6834", "tag": "#1baf7a", "flat": "#eda100"}
NAME = {"clean": "as recorded (no tags)", "blur": "roof blurred", "tag": "made-up tag on the roof",
        "flat": "roof flattened"}
RADIUS = 8.0       # the strict match radius, frame px (evaluate.py's default: about a quarter of a robot)
# the main match radius: half a robot's body at that spot (as autolabel.evaluate_stops), from the recording's camera
# model: about 12 px on the overhead camera, 17 px on the side camera. A detection inside it "found the robot".


def standing_flags(run):
    """{robot: function t -> True if the camera track moved < 1 cm in the 2 s round t}."""
    tr = autolabel._rows_by_robot(os.path.join(run, "bridge", "track.csv"), "t", ["x", "y"],
                                  keep=lambda r: r.get("status") == "seen")

    def f(rb, t):
        a, b = autolabel._lin(tr[rb], t - 1.0, 1.5), autolabel._lin(tr[rb], t + 1.0, 1.5)
        if a is None or b is None:
            return None
        return math.hypot(b[0] - a[0], b[1] - a[1]) < 1.0
    return f


def score(model_path, root, recs, run, thresh=0.05, half_body=True):
    """One row per labelled robot (matched or not) and one per prediction (for precision at any threshold)."""
    det = infer.load(model_path, score_thresh=thresh)
    still = standing_flags(run)
    robots, preds_all, ms = [], [], []
    for rec in recs:
        ld = os.path.join(root, rec, "labels")
        to_cm = detcore.find_homography_for(ld)
        import roofmask
        size = roofmask.body_px_from_calibration(os.path.join(root, rec))
        cam = "overhead" if "_overhead_" in rec else "side"
        rows = {}
        with open(os.path.join(ld, "labels.csv"), newline="") as f:
            for r in csv.DictReader(f):
                rows.setdefault(int(r["frame"]), []).append(r)
        for fr, rs in sorted(rows.items()):
            img = cv2.imread(os.path.join(ld, "frames", "%06d.jpg" % fr))
            import time
            t0 = time.perf_counter()
            preds = [p for p in infer.predict(det, img) if p["cls"] == "alvik"]
            ms.append(1000 * (time.perf_counter() - t0))
            gts = [dict(x=float(r["x_px"]), y=float(r["y_px"]), obj=r["obj"],
                        heading=float(r["heading_deg"]) if r["heading_known"] == "1" else None, t=float(r["t_wall"]))
                   for r in rs if r["cls"] == "alvik"]
            matched = {}
            for i in sorted(range(len(preds)), key=lambda i: -preds[i]["score"]):
                best, bd = None, 1e9
                for j, g in enumerate(gts):
                    if j in matched.values():
                        continue
                    d = math.hypot(preds[i]["x"] - g["x"], preds[i]["y"] - g["y"])
                    rad = 0.5 * size(g["x"], g["y"]) if (half_body and size) else RADIUS
                    if d <= rad and d <= bd:
                        best, bd = j, d
                if best is not None:
                    matched[i] = best
            for i, p in enumerate(preds):
                preds_all.append(dict(rec=rec, frame=fr, score=p["score"], tp=i in matched))
            inv = {j: i for i, j in matched.items()}
            for j, g in enumerate(gts):
                row = dict(rec=rec, frame=fr, cam=cam, robot=g["obj"], standing=still(g["obj"], g["t"]),
                           heading_known=g["heading"] is not None, found=j in inv, score=None, err_px=None,
                           err_cm=None, head_err=None, head_given=None)
                if j in inv:
                    p = preds[inv[j]]
                    row["score"] = p["score"]
                    row["err_px"] = math.hypot(p["x"] - g["x"], p["y"] - g["y"])
                    if to_cm:
                        a, b = to_cm(p["x"], p["y"]), to_cm(g["x"], g["y"])
                        row["err_cm"] = math.hypot(a[0] - b[0], a[1] - b[1])
                    row["head_given"] = p.get("heading_deg") is not None
                    if g["heading"] is not None and p.get("heading_deg") is not None:
                        row["head_err"] = abs(detcore.wrap_deg(p["heading_deg"] - g["heading"]))
                robots.append(row)
    return robots, preds_all, float(np.median(ms))


def summary(robots, preds, th):
    """Recall / precision / errors at score threshold th."""
    n = len(robots)
    found = [r for r in robots if r["found"] and r["score"] >= th]
    tp = sum(1 for p in preds if p["score"] >= th and p["tp"])
    npred = sum(1 for p in preds if p["score"] >= th)
    cm = [r["err_cm"] for r in found if r["err_cm"] is not None]
    he = [r["head_err"] for r in found if r["head_err"] is not None]
    hk = [r for r in found if r["heading_known"]]
    pc = lambda v, q: round(float(np.percentile(v, q)), 2) if v else None  # noqa: E731
    return dict(robots=n, found=len(found), recall=round(len(found) / max(1, n), 3), predictions=npred,
                false=npred - tp, precision=round(tp / max(1, npred), 3),
                centre_cm_median=pc(cm, 50), centre_cm_p95=pc(cm, 95),
                heading_deg_median=pc(he, 50), heading_deg_p95=pc(he, 95),
                heading_flipped_pct=round(100 * float(np.mean([h > 135 for h in he])), 1) if he else None,
                heading_given_pct=round(100 * sum(1 for r in hk if r["head_given"]) / max(1, len(hk)), 1))


def charts(res, out, th):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                         "axes.edgecolor": "#888", "axes.grid": True, "grid.color": "#e6e6e6"})
    ths = np.round(np.arange(0.1, 0.81, 0.05), 2)
    # 1: recall and precision against the score threshold, each variant
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.8), sharey=True)
    for v, d in res["float"].items():
        rr = [summary(d["robots"], d["preds"], t) for t in ths]
        axs[0].plot(ths, [r["recall"] for r in rr], color=COL[v], lw=2, label=NAME[v])
        axs[1].plot(ths, [r["precision"] if r["predictions"] else np.nan for r in rr], color=COL[v], lw=2,
                     label=NAME[v])
    for ax, t in zip(axs, ("Recall (share of robots found)", "Precision (share of detections that are robots)")):
        ax.set_title(t, loc="left", fontsize=10)
        ax.set_xlabel("score threshold")
        ax.axvline(th, color="#999", lw=1, ls=":")
        ax.set_ylim(0, 1.02)
    axs[0].legend(frameon=False, fontsize=8, loc="lower left")
    fig.suptitle("Detector-v1 on the held-out block (11 Nov home run, 11:11-12:54, 90 pictures, 270 robots); "
                 "truth: camera track + robots' own headings", fontsize=9, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "01-recall-precision.png"), dpi=110)
    plt.close(fig)
    # 2: centre error and heading error, cumulative, each variant
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.8))
    for v, d in res["float"].items():
        f = [r for r in d["robots"] if r["found"] and r["score"] >= th]
        x = np.sort([r["err_cm"] for r in f if r["err_cm"] is not None])
        if len(x):
            axs[0].plot(x, np.arange(1, len(x) + 1) / len(x), color=COL[v], lw=2, label=NAME[v])
        if v not in ("clean", "tag"):
            continue
        for cam, ls in (("overhead", "-"), ("side", "--")):
            x = np.sort([r["head_err"] for r in f if r["head_err"] is not None and r["cam"] == cam])
            if len(x):
                axs[1].plot(x, np.arange(1, len(x) + 1) / len(x), color=COL[v], lw=2, ls=ls,
                            label="%s, %s camera (%d)" % (NAME[v], cam, len(x)))
    axs[0].set_xlabel("centre error, cm on the floor")
    axs[1].set_xlabel("heading error, degrees")
    axs[0].set_xlim(0, 6)
    axs[1].set_xlim(0, 180)
    for ax, t in zip(axs, ("Centre error (found robots)", "Heading error (found robots with a checked heading)")):
        ax.set_title(t, loc="left", fontsize=10)
        ax.set_ylabel("share of robots at or under")
        ax.set_ylim(0, 1.02)
    axs[0].legend(frameon=False, fontsize=8, loc="lower right")
    axs[1].legend(frameon=False, fontsize=8, loc="center right", bbox_to_anchor=(1.0, 0.35))
    fig.suptitle("Detector-v1, held-out block, threshold %.2f; truth: camera track (centre), robots' own headings "
                 "checked by the camera (heading)" % th, fontsize=9, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "02-errors.png"), dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--model", required=True)
    ap.add_argument("--int8")
    ap.add_argument("--variant", action="append", required=True, help="name=labels root")
    ap.add_argument("--test", required=True)
    ap.add_argument("--thresh", type=float, default=0.3)
    ap.add_argument("--out", default=HERE)
    a = ap.parse_args()
    recs = a.test.split(",")
    res, table = {}, {}
    for kind, path, half in (("float", a.model, True), ("int8", a.int8, True), ("float_strict8px", a.model, False)):
        if not path:
            continue
        res[kind] = {}
        for v in a.variant:
            name, root = v.split("=", 1)
            robots, preds, ms = score(path, root, recs, a.run, half_body=half)
            res[kind][name] = dict(robots=robots, preds=preds)
            s = summary(robots, preds, a.thresh)
            s["ms_per_picture_here"] = round(ms, 1)
            for grp, sel in (("overhead", lambda r: r["cam"] == "overhead"), ("side", lambda r: r["cam"] == "side"),
                             ("standing", lambda r: r["standing"] is True),
                             ("moving", lambda r: r["standing"] is False)):
                rr = [r for r in robots if sel(r)]
                pp = [p for p in preds if (grp not in ("overhead", "side")) or ("_%s_" % grp) in p["rec"]]
                g = summary(rr, pp, a.thresh)
                if grp in ("standing", "moving"):
                    g = {k: g[k] for k in ("robots", "found", "recall", "centre_cm_median", "heading_deg_median")}
                s[grp] = g
            table.setdefault(kind, {})[name] = s
            print(kind, name, json.dumps(s))
    with open(os.path.join(a.out, "results.json"), "w") as f:
        json.dump(dict(threshold=a.thresh, radius="half a body (float, int8); 8 px (float_strict8px)", test=recs, results=table), f, indent=1)
    with open(os.path.join(a.out, "per_robot.csv"), "w", newline="") as f:
        cols = ["model", "variant", "rec", "frame", "cam", "robot", "standing", "heading_known", "found", "score",
                "err_px", "err_cm", "head_err", "head_given"]
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for kind, vs in res.items():
            for v, d in vs.items():
                for r in d["robots"]:
                    w.writerow(dict({k: (round(x, 3) if isinstance(x, float) else x) for k, x in r.items()},
                                    model=kind, variant=v))
    charts(res, a.out, a.thresh)


if __name__ == "__main__":
    main()
