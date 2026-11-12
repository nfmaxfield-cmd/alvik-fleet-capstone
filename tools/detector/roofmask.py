#!/usr/bin/env python3
"""Hide or change a robot's roof in a picture, so the detector cannot lean on what sits on the roof (detector-v1).

Why: at school every robot carries a roof tag; at home none do. A detector trained only on tagged robots can become a
tag detector (useless when a tag is unreadable, and for the tag-free mode); one trained only on bare robots may be
thrown by a tag. So in training some robots get their roof blurred, flattened or covered by a made-up tag, and the
held-out pictures are scored three ways (as recorded, roof blurred, tag pasted on).

Modes (each acts on a square over the roof, turned with the robot's heading when it is known):
  blur   the square replaced by a strongly blurred copy of itself: the robot's outline and brightness stay, the roof's
         detail (LEDs, buttons, a tag) goes
  flat   the square filled with its own mean grey
  tag    a made-up tag pasted on: white margin, black frame, a random 5 x 5 black/white pattern (like tag25h9), turned
         a random multiple of 90 deg from the heading (tags can be mounted turned)

    python3 roofmask.py --labels <labels root or folders> --out <dir> --mode blur [--frac 0.75]
        writes a copy of every recording with every labelled robot's roof changed (for scoring); the roof size comes
        from the recording's floor_calibration.json when it has one, else --size-px

Changelog
  2026-11-12 dv1  first version
"""
import argparse
import csv
import json
import math
import os
import shutil
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import detcore  # noqa: E402

VERSION = "roofmask-2026-11-12dv1"
MODES = ("blur", "flat", "tag")
ROOF_FRAC = 0.75   # the square's side in robot sizes: the school tag is 8 cm on a 9.5 cm roof (0.84); CHOSEN a little
                   # smaller so the body's outline stays. DOE: 0.6 / 0.75 / 0.9 against masked recall
BODY_CM = 9.5


def _square(x, y, heading_deg, side):
    """4 corners of a side x side square centred on (x, y), turned with the heading (labels convention: degrees
    counter-clockwise from picture-right)."""
    a = math.radians(heading_deg or 0.0)
    c, s = math.cos(a), -math.sin(a)          # picture y points down
    h = side / 2.0
    pts = [(-h, -h), (h, -h), (h, h), (-h, h)]
    return np.float32([(x + c * px - s * py, y + s * px + c * py) for px, py in pts])


def tag_patch(rng, px=70):
    """A made-up tag picture (grey uint8, px x px): white margin, black frame, random 5 x 5 inside (9 cells across)."""
    cells = 9
    g = np.full((cells, cells), 255, np.uint8)
    g[1:-1, 1:-1] = 0
    g[2:-2, 2:-2] = (rng.random((5, 5)) < 0.5) * 255
    return cv2.resize(g, (px, px), interpolation=cv2.INTER_NEAREST)


def apply(img, x, y, heading_deg, size_px, mode, rng, frac=ROOF_FRAC):
    """Change one robot's roof in place. img: BGR or grey uint8. size_px: the robot's size in this picture's px."""
    side = max(3.0, frac * size_px)
    quad = _square(x, y, heading_deg, side)
    mask = np.zeros(img.shape[:2], np.uint8)
    cv2.fillConvexPoly(mask, np.round(quad).astype(np.int32), 255)
    m = mask > 0
    if not m.any():
        return img
    if mode == "blur":
        b = cv2.GaussianBlur(img, (0, 0), max(1.0, side / 3.0))
        img[m] = b[m]
    elif mode == "flat":
        img[m] = img[m].reshape(-1, img.shape[2] if img.ndim == 3 else 1).mean(axis=0).astype(np.uint8).squeeze()
    elif mode == "tag":
        n = 70
        t = tag_patch(rng, n)
        turn = (heading_deg or 0.0) + 90.0 * int(rng.integers(0, 4))
        dst = _square(x, y, turn, side)
        M = cv2.getPerspectiveTransform(np.float32([(0, 0), (n, 0), (n, n), (0, n)]), dst)
        w = cv2.warpPerspective(t, M, (img.shape[1], img.shape[0]), flags=cv2.INTER_AREA)
        lo, hi = np.percentile(img[m], 5), np.percentile(img[m], 99)     # the picture's own black and white there
        hi = max(hi, lo + 60)
        val = (lo + (hi - lo) * (w.astype(np.float32) / 255.0)).clip(0, 255).astype(np.uint8)
        if img.ndim == 3:
            val = val[:, :, None].repeat(img.shape[2], axis=2)
        img[m] = val[m]
    else:
        raise ValueError("mode must be one of %s" % (MODES,))
    return img


def body_px_from_calibration(rec_dir):
    """A function (x_px, y_px) -> robot size in px from the recording's floor_calibration.json, or None."""
    p = os.path.join(rec_dir, "floor_calibration.json")
    if not os.path.exists(p):
        return None
    to_cm = detcore.load_homography(p)
    with open(p) as f:
        H = np.array(json.load(f)["H_floor_to_image"], float)

    def size(u, v):
        x, y = to_cm(u, v)
        a = H @ [x, y, 1.0]
        b = H @ [x + BODY_CM, y, 1.0]
        c = H @ [x, y + BODY_CM, 1.0]
        a, b, c = a[:2] / a[2], b[:2] / b[2], c[:2] / c[2]
        return 0.5 * (np.hypot(*(b - a)) + np.hypot(*(c - a)))
    return size


def copy_recordings(label_dirs, out, mode, frac=ROOF_FRAC, size_px=28.0, seed=0):
    rng = np.random.default_rng(seed)
    done = []
    for ld in label_dirs:
        rec = os.path.dirname(os.path.abspath(ld)) if os.path.basename(ld) == "labels" else ld
        name = detcore.recording_name(ld)
        dst = os.path.join(out, name)
        os.makedirs(os.path.join(dst, "labels", "frames"), exist_ok=True)
        for fn in ("floor_calibration.json", "autolabel.json"):
            if os.path.exists(os.path.join(rec, fn)):
                shutil.copy(os.path.join(rec, fn), os.path.join(dst, fn))
        shutil.copy(os.path.join(ld, "labels.csv"), os.path.join(dst, "labels", "labels.csv"))
        size = body_px_from_calibration(rec)
        rows = {}
        with open(os.path.join(ld, "labels.csv"), newline="") as f:
            for r in csv.DictReader(f):
                rows.setdefault(int(r["frame"]), []).append(r)
        for fr, rs in sorted(rows.items()):
            img = cv2.imread(os.path.join(ld, "frames", "%06d.jpg" % fr))
            for r in rs:
                if r["cls"] != "alvik" or not r["x_px"]:
                    continue
                x, y = float(r["x_px"]), float(r["y_px"])
                hd = float(r["heading_deg"]) if r.get("heading_deg") else None
                apply(img, x, y, hd, size(x, y) if size else size_px, mode, rng, frac)
            cv2.imwrite(os.path.join(dst, "labels", "frames", "%06d.jpg" % fr), img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        done.append(dst)
    return done


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--labels", nargs="+", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--mode", choices=MODES, default="blur")
    ap.add_argument("--frac", type=float, default=ROOF_FRAC)
    ap.add_argument("--size-px", type=float, default=28.0, help="robot size in px when a recording has no calibration")
    ap.add_argument("--only", help="comma list of recording names to copy (e.g. the test recordings)")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args(argv)
    dirs = detcore.find_label_dirs(a.labels)
    if a.only:
        keep = set(a.only.split(","))
        dirs = [d for d in dirs if detcore.recording_name(d) in keep]
    out = copy_recordings(dirs, a.out, a.mode, a.frac, a.size_px, a.seed)
    print("%d recording(s) with roofs changed (%s) in %s" % (len(out), a.mode, a.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
