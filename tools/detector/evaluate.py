"""Score detectors against labels: how many objects they find, how many
false alarms, how far off the centres and headings are.

    # our model on the test recordings of a training run
    python evaluate.py --labels D:/recordings --split runs/v1/split.json --model infer:runs/v1/best.onnx

    # compare several detectors in one table (same frames, same rules)
    python evaluate.py --labels D:/recordings --split runs/v1/split.json \\
        --model infer:runs/v1/best.onnx --model baseline_tv:runs/base/best.pt --model csv:other_preds.csv

    # new recordings never used in training: score every frame
    python evaluate.py --labels D:/new_session --all --model infer:best.onnx

--model is KIND:PATH, repeatable. KIND is a detector module in this folder
(infer = .onnx, predict_torch = .pt, baseline_tv = the torchvision baseline),
a path to any .py file with load(path)/predict(model, frame_bgr), or `csv`
for a predictions file with columns recording,frame,cls,x_px,y_px,heading_deg,score
(infer.py --csv writes these; so does --save-predictions here).

How a prediction is matched: within each frame and class, predictions are
taken from most to least confident, and each claims the nearest unclaimed
label within --radius px (default 8 px, about a quarter of a robot; per class
with e.g. --radius 8,hand=25, since a hand's "centre" is much vaguer).
- precision: share of predictions that matched a label (few false alarms = high)
- recall: share of labels that were found (few misses = high)
- F1: one number combining both, 2PR/(P+R)
- centre error: distance between matched prediction and label (mean, p95 = 95% are below)
- heading error: angle difference for matched robots whose label has a heading;
  flipped% = share more than 135 deg off (front and back confused);
  heading_given% = of the matched robots whose label has a heading, the share
  the detector gave a heading for
- cm errors: if a floor_calibration.json (3x3 matrix floor cm -> image px)
  is next to the recording or given with --calibration

Writes a CSV (--out) and prints a table.
"""
import argparse
import csv
import importlib
import importlib.util
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import detcore  # noqa: E402

COLS = ["detector", "cls", "n_labels", "n_preds", "precision", "recall", "f1", "centre_err_px_mean",
        "centre_err_px_p95", "centre_err_cm_mean", "centre_err_cm_p95", "n_heading", "heading_given_pct", "heading_err_deg_mean",
        "heading_err_deg_median", "heading_err_deg_p95", "heading_flipped_pct", "ms_per_frame"]
SHOW = ["detector", "cls", "n_labels", "n_preds", "precision", "recall", "f1", "centre_err_px_mean",
        "centre_err_px_p95", "centre_err_cm_mean", "heading_given_pct", "heading_err_deg_mean", "heading_err_deg_median",
        "heading_err_deg_p95",
        "heading_flipped_pct", "ms_per_frame"]


def load_module(kind):
    if kind.endswith(".py"):
        spec = importlib.util.spec_from_file_location(os.path.splitext(os.path.basename(kind))[0], kind)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    return importlib.import_module(kind)


def read_pred_csv(path):
    """predictions CSV -> dict (recording, frame) -> list of detections"""
    out = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            h = (r.get("heading_deg") or "").strip()
            d = dict(cls=r["cls"], x=float(r["x_px"]), y=float(r["y_px"]),
                     heading_deg=float(h) if h else None, score=float(r.get("score") or 1.0))
            out.setdefault((r.get("recording", ""), int(r["frame"])), []).append(d)
    return out


def run_detector(kind, path, samples, min_score):
    """Run a detector module on every sample. Returns ({sample index: dets}, ms per frame)."""
    mod = load_module(kind)
    model = mod.load(path)
    own = getattr(model, "score_thresh", None)       # the detector's own threshold
    if min_score is not None and own is not None:
        model.score_thresh = min_score               # let lower-scored guesses through, filter later
    preds, t = {}, 0.0
    for i, s in enumerate(samples):
        img = cv2.imread(s["image"])
        t0 = time.perf_counter()
        preds[i] = mod.predict(model, img)
        t += time.perf_counter() - t0
    return preds, 1000 * t / max(len(samples), 1), own


def score(samples, preds, classes, radius, thresh, homs):
    sc = detcore.Scorer(classes, radius)
    for i, s in enumerate(samples):
        dets = [d for d in preds.get(i, []) if d.get("score", 1.0) >= thresh]
        sc.add(dets, s["objects"], to_cm=homs.get(s["label_dir"]))
    return sc.rows()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--labels", nargs="+", required=True)
    ap.add_argument("--model", action="append", required=True, metavar="KIND:PATH")
    ap.add_argument("--split", help="split.json from train.py")
    ap.add_argument("--part", default="test", choices=["train", "val", "test"])
    ap.add_argument("--all", action="store_true", help="score every labelled frame (no split)")
    ap.add_argument("--val-frac", type=float, default=0.15)
    ap.add_argument("--test-frac", type=float, default=0.15)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--classes", help="comma list (default alvik,hand,unknown,machine)")
    ap.add_argument("--radius", default="8", help="match radius in image px; per class too, e.g. 8,hand=25")
    ap.add_argument("--score-thresh", type=float, help="ignore predictions below this (default: each detector's own)")
    ap.add_argument("--sweep", action="store_true", help="also show F1 at a range of score thresholds")
    ap.add_argument("--calibration", help="floor_calibration.json for recordings that lack their own")
    ap.add_argument("--out", default="eval.csv")
    ap.add_argument("--save-predictions", help="write all predictions to this CSV (first non-csv model)")
    a = ap.parse_args()

    classes = a.classes.split(",") if a.classes else detcore.DEFAULT_CLASSES
    a.radius = detcore.parse_radius(a.radius)
    samples = detcore.read_many(a.labels, classes, verbose=False)
    if not a.all:
        if a.split:
            assign = detcore.load_split(a.split)
        else:
            assign = detcore.make_split(samples, a.val_frac, a.test_frac, a.seed)
            print("No --split given: made one with the same rules as train.py (pass --split to be sure it matches).")
        samples = detcore.select(samples, assign, a.part)
    if not samples:
        sys.exit("No frames to score.")
    recs = sorted({s["recording"] for s in samples})
    print(f"Scoring {len(samples)} frames from {len(recs)} recording(s): {', '.join(recs)}")
    homs = {}
    for d in sorted({s["label_dir"] for s in samples}):
        h = detcore.find_homography_for(d, a.calibration)
        if h is not None:
            homs[d] = h
    if homs:
        print(f"cm errors from floor calibration for {len(homs)} recording(s)")

    all_rows, sweep_rows = [], []
    saved = False
    for spec in a.model:
        kind, _, path = spec.partition(":")      # infer:C:/runs/best.onnx also works
        if not path:
            sys.exit(f"--model {spec}: write it as KIND:PATH, e.g. infer:best.onnx or csv:preds.csv")
        name = f"{kind}:{os.path.basename(path)}"
        ms = float("nan")
        if kind == "csv":
            by_key = read_pred_csv(path)
            preds = {i: by_key.get((s["recording"], s["frame"]), []) for i, s in enumerate(samples)}
            thresh = a.score_thresh if a.score_thresh is not None else 0.0
        else:
            # with --sweep, run the detector with a low threshold and filter afterwards
            low = 0.05 if a.sweep else a.score_thresh
            preds, ms, own = run_detector(kind, path, samples, low)
            thresh = a.score_thresh if a.score_thresh is not None else (own if own is not None else 0.0)
            if a.save_predictions and not saved:
                with open(a.save_predictions, "w", newline="") as f:
                    w = csv.writer(f)
                    w.writerow(["recording", "frame", "cls", "x_px", "y_px", "heading_deg", "score"])
                    for i, s in enumerate(samples):
                        for d in preds[i]:
                            w.writerow([s["recording"], s["frame"], d["cls"], f"{d['x']:.2f}", f"{d['y']:.2f}",
                                        "" if d["heading_deg"] is None else f"{d['heading_deg']:.1f}",
                                        f"{d['score']:.3f}"])
                saved = True
        for r in score(samples, preds, classes, a.radius, thresh, homs):
            r.update(detector=name, ms_per_frame=ms)
            all_rows.append(r)
        if a.sweep:
            for t in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]:
                for r in score(samples, preds, classes, a.radius, t, homs):
                    if r["cls"] in ("all",) or len(classes) == 1 or r["cls"] == classes[0]:
                        sweep_rows.append(dict(detector=name, thresh=t, cls=r["cls"], precision=r["precision"],
                                               recall=r["recall"], f1=r["f1"]))

    show = [c for c in SHOW if not (c.startswith("centre_err_cm") and not homs)]
    print()
    print(detcore.format_table(all_rows, show, "{:.3g}"))
    if sweep_rows:
        print("\nScore threshold sweep:")
        print(detcore.format_table(sweep_rows, ["detector", "thresh", "cls", "precision", "recall", "f1"]))
    detcore.write_csv(all_rows, a.out, COLS)
    print(f"\nWrote {a.out}")


if __name__ == "__main__":
    main()
