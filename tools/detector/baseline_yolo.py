"""RESEARCH-ONLY comparison: Ultralytics YOLO (AGPL-3.0) fine-tuned on our labels.

This file exists only to put YOLO's numbers next to ours in the report.
It is NOT part of the program we deliver:
- nothing else in this repo imports it or `ultralytics` (a test checks that);
- `ultralytics` is listed in requirements-research.txt, not requirements.txt;
- YOLO weights trained here (runs/yolo*/best.pt) must not be shipped with the
  program (see README, "Licences").

Same interface and label handling as baseline_tv.py: each centre label becomes
a fixed-size square box (alvik 30 px, hand 70 px, ...; --box-size), the
predicted box centre is the position, heading is None.

    pip install -r requirements-research.txt          # ultralytics, research only
    # fine-tune from COCO-pretrained YOLO11n (downloads yolo11n.pt from GitHub once)
    python baseline_yolo.py train --labels D:/recordings --out runs/yolo --split runs/v1/split.json
    # random start (fair against our model, which has no pretrained weights)
    python baseline_yolo.py train --labels D:/recordings --out runs/yolo0 --weights none --split runs/v1/split.json
    # optional heading: pose model with one keypoint = the robot's front point
    python baseline_yolo.py train --labels D:/recordings --out runs/yolop --task pose --split runs/v1/split.json
    # score / time it next to our model
    python evaluate.py --labels D:/recordings --split runs/v1/split.json \\
        --model infer:runs/v1/best.onnx --model baseline_yolo:runs/yolo/best.pt
    python benchmark.py --detector baseline_yolo --weights runs/yolo/best.pt --threads 1,4

--task pose: YOLO's keypoint ("pose") model with a single keypoint placed at
the front point our own model uses (front_frac x object size ahead of the
centre, only where heading_known = 1). Heading = direction from the box
centre to that keypoint; None when the keypoint's confidence is low or the
class has no front. Costs almost nothing extra at run time.
(The oriented-box model, -obb, was not used: a square box's angle repeats
every 90 deg, and even a long box cannot tell front from back.)

Input: the frame is shrunk to --size (default 320x240) exactly as our
model's input, then YOLO pads it to 320x256 (a multiple of 32). Training
uses square 320x320 mosaics, as Ultralytics requires for training.
"""
import argparse
import json
import os
import shutil
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import detcore  # noqa: E402
from model import default_config, parse_size  # noqa: E402

CONFIG_NAME = "yolo_config.json"


def _yolo():
    """The only place ultralytics is imported (lazily, so this file loads without it)."""
    try:
        from ultralytics import YOLO
    except ImportError as e:
        raise ImportError("baseline_yolo needs the research-only package ultralytics: "
                          "pip install -r requirements-research.txt") from e
    return YOLO


def weights_name(model, task, weights):
    """yolo11n + detect + coco -> 'yolo11n.pt' (pretrained) or 'yolo11n.yaml' (random start)."""
    suffix = "-pose" if task == "pose" else ""
    return f"{model}{suffix}.pt" if weights == "coco" else f"{model}{suffix}.yaml"


def label_lines(objects, classes, box_px, img_w, img_h, task="detect", heading_classes=(), front_frac=0.35):
    """Centre labels (frame px) -> YOLO label lines 'cls cx cy w h [kx ky v]', all as fractions of the frame.
    Fixed-size square boxes; in pose mode one keypoint at the front point (visible=2) or absent (0 0 0)."""
    out = []
    for o in objects:
        s = float(box_px.get(o["cls"], 30))
        cx, cy = (o["x"] + 0.5) / img_w, (o["y"] + 0.5) / img_h        # pixel centres at whole numbers
        line = f"{classes.index(o['cls'])} {cx:.6f} {cy:.6f} {s / img_w:.6f} {s / img_h:.6f}"
        if task == "pose":
            if o.get("heading") is not None and o["cls"] in heading_classes:
                dx, dy = detcore.heading_to_vec(o["heading"])
                kx, ky = o["x"] + dx * front_frac * s, o["y"] + dy * front_frac * s
                line += f" {(kx + 0.5) / img_w:.6f} {(ky + 0.5) / img_h:.6f} 2"
            else:
                line += " 0 0 0"
        out.append(line)
    return out


def write_dataset(samples_by_part, root, cfg, task):
    """Write a YOLO dataset (frames shrunk to the network input size with the same
    INTER_AREA resize as at run time, plus label .txt files) and data.yaml."""
    in_w, in_h = cfg["in_w"], cfg["in_h"]
    for part, samples in samples_by_part.items():
        idir, ldir = os.path.join(root, "images", part), os.path.join(root, "labels", part)
        os.makedirs(idir, exist_ok=True)
        os.makedirs(ldir, exist_ok=True)
        for s in samples:
            img = cv2.imread(s["image"])
            h, w = img.shape[:2]
            name = f"{s['recording']}_{s['frame']:06d}"
            cv2.imwrite(os.path.join(idir, name + ".png"), cv2.resize(img, (in_w, in_h), interpolation=cv2.INTER_AREA))
            lines = label_lines(s["objects"], cfg["classes"], cfg["obj_size_px"], w, h, task,
                                cfg["heading_classes"], cfg["front_frac"])
            with open(os.path.join(ldir, name + ".txt"), "w") as f:
                f.write("\n".join(lines) + ("\n" if lines else ""))
    yaml = [f"path: {os.path.abspath(root)}", "train: images/train", "val: images/val", "names:"]
    yaml += [f"  {i}: {c}" for i, c in enumerate(cfg["classes"])]
    if task == "pose":
        yaml += ["kpt_shape: [1, 3]", "flip_idx: [0]"]
    path = os.path.join(root, "data.yaml")
    with open(path, "w") as f:
        f.write("\n".join(yaml) + "\n")
    return path


class YoloDetector:
    def __init__(self, path, score_thresh=0.3, threads=None):
        import torch
        prev = torch.get_num_threads()
        cfg_path = os.path.join(os.path.dirname(os.path.abspath(path)), CONFIG_NAME)
        self.yolo = _yolo()(path)
        if os.path.isfile(cfg_path):
            with open(cfg_path) as f:
                self.cfg = json.load(f)
        else:   # weights without our config: take what the model knows
            self.cfg = default_config(classes=[self.yolo.names[i] for i in sorted(self.yolo.names)])
            self.cfg["task"] = self.yolo.task
        self.classes = self.cfg["classes"]
        self.in_w, self.in_h = self.cfg["in_w"], self.cfg["in_h"]
        self.pose = self.cfg.get("task") == "pose"
        self.heading_classes = set(self.cfg.get("heading_classes", []))
        self.kpt_thresh = 0.5
        self.score_thresh = score_thresh
        # the first predict() sets Ultralytics up, which resets the torch thread count: warm up, then set it
        self(np.zeros((self.in_h, self.in_w, 3), np.uint8))
        torch.set_num_threads(int(threads) if threads else prev)

    def __call__(self, frame_bgr):
        h, w = frame_bgr.shape[:2]
        img = frame_bgr if (w, h) == (self.in_w, self.in_h) else cv2.resize(
            frame_bgr, (self.in_w, self.in_h), interpolation=cv2.INTER_AREA)
        sx, sy = self.in_w / w, self.in_h / h
        r = self.yolo.predict(img, imgsz=max(self.in_w, self.in_h), conf=self.score_thresh, device="cpu",
                              verbose=False)[0]
        boxes = r.boxes
        if boxes is None or len(boxes) == 0:
            return []
        xywh, conf, cls = boxes.xywh.numpy(), boxes.conf.numpy(), boxes.cls.numpy().astype(int)
        kp = None
        if self.pose and r.keypoints is not None and r.keypoints.data.shape[0] == len(xywh):
            kp = r.keypoints.data.numpy()          # n x 1 x 3 (x, y, confidence), input px
        dets = []
        for i, (b, s, c) in enumerate(zip(xywh, conf, cls)):
            name = self.classes[c]
            heading = None
            if kp is not None and name in self.heading_classes and kp[i, 0, 2] >= self.kpt_thresh:
                dx, dy = (kp[i, 0, 0] - b[0]) / sx, (kp[i, 0, 1] - b[1]) / sy
                if dx * dx + dy * dy > 1.0:      # front point at least 1 px from the centre
                    heading = detcore.vec_to_heading(dx, dy)
            # Ultralytics boxes use pixel edges (0..w); ours put pixel centres at whole numbers
            dets.append(dict(cls=name, x=float(b[0] / sx - 0.5), y=float(b[1] / sy - 0.5),
                             heading_deg=heading, score=float(s)))
        return dets


def load(path, score_thresh=0.3, threads=None):
    return YoloDetector(path, score_thresh, threads)


def predict(model, frame_bgr):
    return model(frame_bgr)


def train(a):
    import torch
    YOLO = _yolo()
    in_w, in_h = parse_size(a.size)
    cfg = default_config(classes=a.classes.split(",") if a.classes else None, in_w=in_w, in_h=in_h, in_ch=3)
    if a.box_size:
        for kv in a.box_size.split(","):
            k, v = kv.split("=")
            cfg["obj_size_px"][k] = float(v)
    cfg.update(task=a.task, yolo_model=a.model, yolo_weights=a.weights)
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    samples = detcore.read_many(a.labels, cfg["classes"], verbose=False)
    assign = detcore.load_split(a.split) if a.split else detcore.make_split(samples, seed=a.seed)
    detcore.save_split(assign, os.path.join(out, "split.json"))
    parts = {p: detcore.select(samples, assign, p) for p in ("train", "val")}
    print(f"train {len(parts['train'])} frames, val {len(parts['val'])} frames")
    data = write_dataset(parts, os.path.join(out, "data"), cfg, a.task)

    src = weights_name(a.model, a.task, a.weights)
    if a.weights == "coco" and a.weights_dir and os.path.isfile(os.path.join(a.weights_dir, src)):
        src = os.path.join(a.weights_dir, src)
    try:
        net = YOLO(src, task=a.task)
    except Exception as e:   # most likely the download was blocked
        sys.exit(f"Could not load {src}: {e}\nIf the download is blocked, put the .pt file in --weights-dir, "
                 "or use --weights none.")
    t0 = time.time()
    kw = dict(data=data, imgsz=max(in_w, in_h), epochs=a.epochs, batch=a.batch, device="cpu", workers=a.workers,
              project=out, name="ultralytics", exist_ok=True, seed=a.seed, pretrained=a.weights == "coco",
              patience=a.patience, flipud=0.5, fliplr=0.5, plots=False, amp=False, verbose=False, cache="ram")
    if a.max_minutes:
        kw["time"] = a.max_minutes / 60.0       # Ultralytics fits its epochs and schedule to this
    if a.threads:
        torch.set_num_threads(a.threads)
    net.train(**kw)
    best = os.path.join(out, "ultralytics", "weights", "best.pt")
    shutil.copy(best, os.path.join(out, "best.pt"))
    with open(os.path.join(out, CONFIG_NAME), "w") as f:
        json.dump(cfg, f, indent=1)
    print(f"Wrote {os.path.join(out, 'best.pt')} ({(time.time() - t0) / 60:.1f} min). "
          "Research only: do not ship these weights with the program (AGPL-3.0).")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    t = sub.add_parser("train")
    t.add_argument("--labels", nargs="+", required=True)
    t.add_argument("--out", required=True)
    t.add_argument("--model", default="yolo11n", help="yolo11n (default), yolo26n, yolov8n, ...")
    t.add_argument("--task", default="detect", choices=["detect", "pose"], help="pose = also learn the heading")
    t.add_argument("--weights", default="coco", choices=["coco", "none"], help="COCO-pretrained or random start")
    t.add_argument("--weights-dir", help="folder holding a downloaded yolo11n.pt etc. (if GitHub is blocked)")
    t.add_argument("--size", default="320x240")
    t.add_argument("--classes")
    t.add_argument("--box-size", help="e.g. alvik=30,hand=70 (camera px)")
    t.add_argument("--split", help="split.json (use the same one as our model, for a fair comparison)")
    t.add_argument("--epochs", type=int, default=100)
    t.add_argument("--max-minutes", type=float, default=0, help="training time budget (overrides --epochs)")
    t.add_argument("--batch", type=int, default=16)
    t.add_argument("--patience", type=int, default=100)
    t.add_argument("--workers", type=int, default=0)
    t.add_argument("--threads", type=int, default=0)
    t.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    if a.cmd == "train":
        train(a)


if __name__ == "__main__":
    main()
