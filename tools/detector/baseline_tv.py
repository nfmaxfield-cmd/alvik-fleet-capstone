"""Comparison baseline: an off-the-shelf, permissively licensed general object
detector from torchvision (BSD-3-Clause), fine-tuned on our labels.

Our labels are centre points, not boxes, so each label becomes a fixed-size
square box around its centre (size per class from the model config,
alvik 30 px, hand 70 px, ...; change with --box-size). After detection, the
centre of the predicted box is the object's position; heading is not
predicted (None).

    # fine-tune from COCO-pretrained weights (needs internet once to download them)
    python baseline_tv.py train --labels D:/recordings --out runs/base --split runs/v1/split.json
    # no download possible: random start (much weaker, needs far more training)
    python baseline_tv.py train --labels D:/recordings --out runs/base --weights none
    # score it next to our model
    python evaluate.py --labels D:/recordings --split runs/v1/split.json \\
        --model infer:runs/v1/best.onnx --model baseline_tv:runs/base/best.pt

Detector interface (same as infer.py): load(path) -> model; predict(model, frame_bgr).

Architectures (--arch):
- ssdlite: ssdlite320_mobilenet_v3_large, the fastest torchvision detector
- frcnn:   fasterrcnn_mobilenet_v3_large_320_fpn, slower, usually more accurate
Both are changed in two small ways for our tiny objects: the input is not
squeezed to 320x320 but kept at --size (default 320x240), and the smallest
anchor boxes (the box shapes the detector starts from) are made smaller.

--weights coco (default) downloads COCO-pretrained weights from
download.pytorch.org into the torch cache (~/.cache/torch/hub/checkpoints).
If the network blocks it, copy the .pth file there by hand, or use
--weights imagenet (backbone only) or --weights none.

Ultralytics YOLO was deliberately not used: it is AGPL-3.0, which would
oblige anyone running a modified version as a service to publish their source.
"""
import argparse
import csv
import os
import random
import sys
import time

import numpy as np
import torch
from torch.utils.data import DataLoader

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import detcore  # noqa: E402
from dataset import AugmentConfig, DetDataset, collate  # noqa: E402
from model import default_config, parse_size  # noqa: E402


def build(arch, n_classes, weights="coco", in_w=320, in_h=240):
    """A torchvision detector with n_classes + 1 outputs (0 = background)."""
    from torchvision.models.detection import (fasterrcnn_mobilenet_v3_large_320_fpn,
                                              ssdlite320_mobilenet_v3_large)
    from torchvision.models import MobileNet_V3_Large_Weights
    nc = n_classes + 1
    if arch == "ssdlite":
        from torchvision.models.detection import SSDLite320_MobileNet_V3_Large_Weights
        from torchvision.models.detection.anchor_utils import DefaultBoxGenerator
        from torchvision.models.detection.ssdlite import SSDLiteClassificationHead
        if weights == "coco":
            m = ssdlite320_mobilenet_v3_large(weights=SSDLite320_MobileNet_V3_Large_Weights.COCO_V1)
            # new class layer for our classes; everything else keeps the COCO training
            in_ch = [layer[0][0].in_channels for layer in m.head.classification_head.module_list]
            n_anchors = m.anchor_generator.num_anchors_per_location()
            m.head.classification_head = SSDLiteClassificationHead(
                in_ch, n_anchors, nc, _bn())
        else:
            wb = MobileNet_V3_Large_Weights.IMAGENET1K_V1 if weights == "imagenet" else None
            m = ssdlite320_mobilenet_v3_large(weights=None, weights_backbone=wb, num_classes=nc)
        # smaller starting boxes (same count, so the layers keep their shape)
        m.anchor_generator = DefaultBoxGenerator([[2, 3] for _ in range(6)], min_ratio=0.04, max_ratio=0.5)
        m.transform.fixed_size = (in_w, in_h)
        m.transform.min_size = (min(in_w, in_h),)
        m.transform.max_size = max(in_w, in_h)
        m.score_thresh = 0.05
    elif arch == "frcnn":
        from torchvision.models.detection import FasterRCNN_MobileNet_V3_Large_320_FPN_Weights
        from torchvision.models.detection.anchor_utils import AnchorGenerator
        from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
        if weights == "coco":
            m = fasterrcnn_mobilenet_v3_large_320_fpn(weights=FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.COCO_V1)
            m.roi_heads.box_predictor = FastRCNNPredictor(m.roi_heads.box_predictor.cls_score.in_features, nc)
        else:
            wb = MobileNet_V3_Large_Weights.IMAGENET1K_V1 if weights == "imagenet" else None
            m = fasterrcnn_mobilenet_v3_large_320_fpn(weights=None, weights_backbone=wb, num_classes=nc)
        m.rpn.anchor_generator = AnchorGenerator(((16, 32, 64, 128, 256),) * 3, ((0.5, 1.0, 2.0),) * 3)
        m.transform.min_size = (min(in_w, in_h),)
        m.transform.max_size = max(in_w, in_h)
        m.roi_heads.score_thresh = 0.05
    else:
        raise ValueError(f"Unknown arch {arch!r}: use ssdlite or frcnn")
    return m


def _bn():
    from functools import partial
    return partial(torch.nn.BatchNorm2d, eps=0.001, momentum=0.03)


def to_targets(objects, classes, box_px, scale):
    """Centre labels -> fixed-size boxes (input px) for torchvision."""
    boxes, labels = [], []
    for o in objects:
        half = box_px.get(o["cls"], 30) * scale / 2
        boxes.append([o["x"] - half, o["y"] - half, o["x"] + half, o["y"] + half])
        labels.append(classes.index(o["cls"]) + 1)
    return dict(boxes=torch.tensor(boxes, dtype=torch.float32).reshape(-1, 4),
                labels=torch.tensor(labels, dtype=torch.int64))


def images_from_batch(batch):
    """Our dataset gives normalised tensors; torchvision wants RGB in 0..1."""
    return [(im / 4.0 + 0.5).clamp(0, 1) for im in batch["image"]]


class TVDetector:
    def __init__(self, path, score_thresh=0.3, threads=None):
        if threads:
            torch.set_num_threads(int(threads))
        ck = torch.load(path, map_location="cpu", weights_only=False)
        self.cfg = ck["config"]
        self.classes = self.cfg["classes"]
        self.in_w, self.in_h = self.cfg["in_w"], self.cfg["in_h"]
        self.net = build(ck["arch"], len(self.classes), "none", self.in_w, self.in_h)
        self.net.load_state_dict(ck["model"])
        self.net.eval()
        self.score_thresh = score_thresh

    @torch.no_grad()
    def __call__(self, frame_bgr):
        x, (sx, sy) = detcore.preprocess(frame_bgr, self.in_w, self.in_h, 3)
        img = torch.from_numpy(x[0] / 4.0 + 0.5).clamp(0, 1)
        out = self.net([img])[0]
        dets = []
        for b, s, l in zip(out["boxes"].numpy(), out["scores"].numpy(), out["labels"].numpy()):
            if s < self.score_thresh or l < 1:
                continue
            cx, cy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
            dets.append(dict(cls=self.classes[l - 1], x=(cx + 0.5) / sx - 0.5, y=(cy + 0.5) / sy - 0.5,
                             heading_deg=None, score=float(s)))
        return dets


def load(path, score_thresh=0.3, threads=None):
    return TVDetector(path, score_thresh, threads)


def predict(model, frame_bgr):
    return model(frame_bgr)


@torch.no_grad()
def validate(net, samples, cfg, radius, thresh=0.3):
    net.eval()
    import cv2
    det = TVDetector.__new__(TVDetector)
    det.net, det.cfg, det.classes = net, cfg, cfg["classes"]
    det.in_w, det.in_h, det.score_thresh = cfg["in_w"], cfg["in_h"], thresh
    sc = detcore.Scorer(cfg["classes"], radius)
    for s in samples:
        sc.add(det(cv2.imread(s["image"])), s["objects"])
    net.train()
    rows = sc.rows()
    return rows[-1] if rows else {}


def train(a):
    a.radius = detcore.parse_radius(a.radius)
    random.seed(a.seed)
    np.random.seed(a.seed)
    torch.manual_seed(a.seed)
    if a.threads:
        torch.set_num_threads(a.threads)
    device = torch.device("cuda" if torch.cuda.is_available() and a.device != "cpu" else "cpu")
    os.makedirs(a.out, exist_ok=True)
    in_w, in_h = parse_size(a.size)
    cfg = default_config(classes=a.classes.split(",") if a.classes else None, in_w=in_w, in_h=in_h, in_ch=3)
    if a.box_size:
        for kv in a.box_size.split(","):
            k, v = kv.split("=")
            cfg["obj_size_px"][k] = float(v)
    samples = detcore.read_many(a.labels, cfg["classes"], verbose=False)
    assign = detcore.load_split(a.split) if a.split else detcore.make_split(samples, seed=a.seed)
    detcore.save_split(assign, os.path.join(a.out, "split.json"))
    tr, va = detcore.select(samples, assign, "train"), detcore.select(samples, assign, "val")
    print(f"train {len(tr)} frames, val {len(va)} frames")

    try:
        net = build(a.arch, len(cfg["classes"]), a.weights, in_w, in_h).to(device)
    except Exception as e:   # most likely the download was blocked
        sys.exit(f"Could not build {a.arch} with --weights {a.weights}: {e}\n"
                 "If the pretrained weights cannot be downloaded, copy the .pth into "
                 f"{os.path.join(torch.hub.get_dir(), 'checkpoints')} or use --weights none.")
    print(f"{a.arch} ({a.weights} start): {sum(p.numel() for p in net.parameters()):,} parameters")

    ds = DetDataset(tr, cfg, augment=AugmentConfig(rotate=not a.no_rotate), with_maps=False)
    dl = DataLoader(ds, batch_size=a.batch, shuffle=True, num_workers=a.workers, collate_fn=collate, drop_last=True)
    params = [p for p in net.parameters() if p.requires_grad]
    opt = torch.optim.AdamW(params, lr=a.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=a.lr, total_steps=a.epochs * len(dl), pct_start=0.1)
    log = open(os.path.join(a.out, "train_log.csv"), "w", newline="")
    w = csv.writer(log)
    w.writerow(["epoch", "seconds", "train_loss", "val_precision", "val_recall", "val_f1", "val_centre_err_px"])
    best, best_ep, t0 = -1.0, 0, time.time()
    for ep in range(1, a.epochs + 1):
        te = time.time()
        net.train()
        tot, n = 0.0, 0
        for batch in dl:
            imgs = [im.to(device) for im in images_from_batch(batch)]
            tg = [{k: v.to(device) for k, v in to_targets(o, cfg["classes"], cfg["obj_size_px"],
                                                            (sc[0] + sc[1]) / 2).items()}
                  for o, sc in zip(batch["objects"], batch["scale"])]
            losses = net(imgs, tg)
            loss = sum(losses.values())
            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(params, 10.0)
            opt.step()
            sched.step()
            tot += loss.item()
            n += 1
        v = validate(net, va, cfg, a.radius)
        f1 = v.get("f1", 0.0)
        if f1 > best:
            best, best_ep = f1, ep
            torch.save(dict(model=net.state_dict(), config=cfg, arch=a.arch, weights=a.weights, epoch=ep),
                       os.path.join(a.out, "best.pt"))
        w.writerow([ep, round(time.time() - te, 1), round(tot / max(n, 1), 4), round(v.get("precision", 0), 4),
                    round(v.get("recall", 0), 4), round(f1, 4), round(v.get("centre_err_px_mean", float("nan")), 3)])
        log.flush()
        print(f"epoch {ep:3d} {time.time() - te:5.0f}s loss {tot / max(n, 1):.3f}  val P {v.get('precision', 0):.3f} "
              f"R {v.get('recall', 0):.3f} F1 {f1:.3f} err {v.get('centre_err_px_mean', float('nan')):.2f}px",
              flush=True)
        if ep - best_ep >= a.patience:
            print("Stopping early.")
            break
        if a.max_minutes and (time.time() - t0) / 60 > a.max_minutes:
            print("Time limit reached.")
            break
    if best < 0:
        print("No epoch finished.")
    print(f"Best val F1 {best:.3f} at epoch {best_ep}; {(time.time() - t0) / 60:.1f} min")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    t = sub.add_parser("train")
    t.add_argument("--labels", nargs="+", required=True)
    t.add_argument("--out", required=True)
    t.add_argument("--arch", default="ssdlite", choices=["ssdlite", "frcnn"])
    t.add_argument("--weights", default="coco", choices=["coco", "imagenet", "none"])
    t.add_argument("--size", default="320x240")
    t.add_argument("--classes")
    t.add_argument("--box-size", help="e.g. alvik=30,hand=70 (camera px)")
    t.add_argument("--split", help="split.json (use the same one as our model, for a fair comparison)")
    t.add_argument("--epochs", type=int, default=30)
    t.add_argument("--batch", type=int, default=8)
    t.add_argument("--lr", type=float, default=1e-3)
    t.add_argument("--patience", type=int, default=8)
    t.add_argument("--max-minutes", type=float, default=0)
    t.add_argument("--radius", default="8", help="match radius for validation, e.g. 8,hand=25")
    t.add_argument("--no-rotate", action="store_true")
    t.add_argument("--workers", type=int, default=0)
    t.add_argument("--threads", type=int, default=0)
    t.add_argument("--device", default="auto")
    t.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    if a.cmd == "train":
        train(a)


if __name__ == "__main__":
    main()
