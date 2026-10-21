"""Train the centre-heatmap detector (model.py) on one or more labels folders.

    python train.py --labels D:/recordings --out runs/v1
    python train.py --labels rec1 rec2 rec3 rec4 --out runs/v1 --epochs 80 --size 320x240
    python train.py --labels D:/recordings --out runs/v2 --split runs/v1/split.json   # same split as v1

Runs on a laptop CPU or a GPU (e.g. free Google Colab); it picks the GPU when
there is one. Writes to --out:

    best.pt        weights + config of the epoch with the best validation score
    last.pt        weights of the last epoch (to resume or inspect)
    train_log.csv  one row per epoch: losses, val precision/recall/F1, errors, time
    split.json     which recording went to train / val / test (reuse it!)
    config.json    the settings used

Losses (what training pushes down):
- heatmap: "focal loss" (as in CenterNet). Almost every heatmap cell is
  background, so plain loss would be dominated by easy "nothing here"
  cells; focal loss down-weights easy cells and cells right next to a centre.
- offset: mean absolute error (L1), only on cells at an object centre.
- heading (front mode, default): focal loss on the front-point map plus L1 on
  its offset, only for robots whose label has a heading (around robots
  without one, the front-point map is left out of the loss).
  (sincos mode: L1 on sin and cos.)

Early stopping: after each epoch the model is run on the validation
recordings and scored (mainly F1 at the match radius, see evaluate.py, with
small penalties for centre and heading error). By default that F1 pools
all objects (--select pooled), so robots, which outnumber everything else,
dominate it; --select macro uses the average of the per-class F1s instead.
In 13-minute tests on synthetic data the two picked the same or similar
epochs; macro becomes worth trying once the validation recordings hold
enough different unknown objects and hands for their F1 to be stable.
train_log.csv shows every class's validation F1 per epoch either way.
Training stops when that has not improved for --patience epochs; best.pt
keeps the best. Until the model finds anything at all (validation F1 still 0), F1
cannot tell epochs apart, so the best epoch is the one with the lowest validation
loss and training never stops early; it also never stops early before --min-epochs.
"""
import argparse
import csv
import json
import os
import random
import sys
import time

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import detcore  # noqa: E402
from dataset import AugmentConfig, DetDataset, collate  # noqa: E402
from paste import PasteConfig  # noqa: E402
from model import CentreNet, check_size, count_params, default_config, describe, parse_size  # noqa: E402


def set_seed(seed, deterministic=False):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if deterministic:
        # Same numbers every run on the same machine (slower on GPU).
        torch.use_deterministic_algorithms(True, warn_only=True)
        torch.backends.cudnn.benchmark = False
        os.environ.setdefault("CUBLAS_WORKSPACE_CONFIG", ":4096:8")


def seed_worker(worker_id):
    s = torch.initial_seed() % 2 ** 32
    np.random.seed(s)
    random.seed(s)


def focal_loss(logits, target, mask=None, neg_w=None):
    """CenterNet focal loss. target is 1 at centres, a Gaussian falling off
    around them, 0 elsewhere. Averaged over the number of objects. mask
    (optional, 0/1): cells to leave out (e.g. around a robot whose heading
    was not labelled, where we do not know where its front is). neg_w
    (optional, same cells): how much each background cell counts; more than 1
    around a spot a person rejected ("not an object here"), 0 where nothing is known."""
    p = torch.sigmoid(logits)
    pos = target.eq(1).float()
    neg = 1.0 - pos
    if mask is not None:
        pos, neg = pos * mask, neg * mask
    if neg_w is not None:
        neg = neg * neg_w
    log_p = F.logsigmoid(logits)
    log_1mp = F.logsigmoid(-logits)
    pos_loss = (log_p * (1 - p) ** 2 * pos).sum()
    neg_loss = (log_1mp * p ** 2 * (1 - target) ** 4 * neg).sum()
    n = pos.sum().clamp(min=1.0)
    return -(pos_loss + neg_loss) / n


def masked_l1(pred, target, weight):
    """L1 on the 2-channel maps, only where weight > 0."""
    w = weight.unsqueeze(1)
    return (torch.abs(pred - target) * w).sum() / (w.sum() * pred.shape[1]).clamp(min=1.0)


def compute_loss(outputs, batch, w_off=1.0, w_hd=1.0):
    """Total loss and its three parts (heatmap, offset, heading)."""
    heat, off, hd = outputs
    negw = batch.get("negw")           # absent in older batches: every cell counts once
    l_heat = focal_loss(heat, batch["heat"], neg_w=negw)
    l_off = masked_l1(off, batch["off"], batch["off_w"])
    if "fheat" in batch:      # front-point mode: find the front, and its exact spot
        l_hd = focal_loss(hd[:, :1], batch["fheat"], batch["fmask"], neg_w=negw) + masked_l1(hd[:, 1:], batch["foff"], batch["foff_w"])
    else:                     # sin/cos mode
        l_hd = masked_l1(hd, batch["hd"], batch["hd_w"])
    return l_heat + w_off * l_off + w_hd * l_hd, (l_heat.item(), l_off.item(), l_hd.item())


@torch.no_grad()
def validate(model, loader, device, cfg, radius, score_thresh, w_hd=1.0):
    """Validation loss + detection scores in camera-frame pixels."""
    model.eval()
    scorer = detcore.Scorer(cfg["classes"], radius)
    tot, parts, n = 0.0, np.zeros(3), 0
    for batch in loader:
        x = batch["image"].to(device)
        t = {k: v.to(device) for k, v in batch.items() if isinstance(v, torch.Tensor) and k != "image"}
        heat_l, off, hd = model(x, logits=True)
        loss, p = compute_loss((heat_l, off, hd), t, w_hd=w_hd)
        tot += loss.item() * len(x)
        parts += np.array(p) * len(x)
        n += len(x)
        heat = torch.sigmoid(heat_l).cpu().numpy()
        off, hd = off.cpu().numpy(), hd.cpu().numpy()
        for b, idx in enumerate(batch["index"]):
            s = loader.dataset.samples[idx]
            hdb = hd[b].copy()
            if cfg.get("heading_mode") == "front":
                hdb[0] = 1 / (1 + np.exp(-hdb[0]))       # logits -> probability
            dets = detcore.decode(heat[b], off[b], hdb, cfg, batch["scale"][b], score_thresh)
            scorer.add(dets, s["objects"])
    model.train()
    rows = scorer.rows()
    allrow = dict(rows[-1]) if rows else {}
    per_cls = [r for r in rows if r["cls"] != "all" and r["n_labels"] > 0]
    allrow["macro_f1"] = float(np.mean([r["f1"] for r in per_cls])) if per_cls else allrow.get("f1", 0.0)
    allrow["per_class_f1"] = {r["cls"]: round(float(r["f1"]), 3) for r in per_cls}
    return tot / max(n, 1), parts / max(n, 1), allrow


def init_from(model, path):
    """Start from an earlier model, even one with fewer classes: every layer
    with the same shape is copied; the output layer copies the channels of
    the classes both models share (so adding a class keeps what was learned)."""
    ck = torch.load(path, map_location="cpu", weights_only=False)
    old_sd, old_cls = ck["model"], ck["config"]["classes"]
    new_sd, new_cls = model.state_dict(), model.cfg["classes"]
    copied = 0
    for k, v in old_sd.items():
        if k in new_sd and new_sd[k].shape == v.shape:
            new_sd[k] = v
            copied += 1
        elif k in ("out.weight", "out.bias"):
            nv = new_sd[k].clone()
            for i, c in enumerate(new_cls):
                if c in old_cls:
                    nv[i] = v[old_cls.index(c)]
            nv[len(new_cls):] = v[len(old_cls):]          # offset channels
            new_sd[k] = nv
            copied += 1
    model.load_state_dict(new_sd)
    print(f"Started from {path}: copied {copied} of {len(old_sd)} tensors (old classes {old_cls}, new {new_cls})")
    if copied < 0.9 * len(old_sd):
        print("WARNING: most layers did not match. Use the same --width, --depth, --in-ch and --heading-mode "
              f"as the old model (it used {ck['config'].get('width')}, {ck['config'].get('depth')}, "
              f"{ck['config'].get('in_ch')}, {ck['config'].get('heading_mode', 'sincos')}).")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--labels", nargs="+", required=True, help="labels folders, recording folders, or a parent folder")
    ap.add_argument("--out", required=True)
    ap.add_argument("--classes", help="comma list; default alvik,hand,unknown,machine (append new ones at the end)")
    ap.add_argument("--obj-size", help="object sizes in camera px, e.g. alvik=30,forklift=120 (sets the blob size)")
    ap.add_argument("--heading-classes", help="comma list of classes that have a front (default alvik,machine)")
    ap.add_argument("--size", default="320x240", help="network input WxH (multiples of 16)")
    ap.add_argument("--in-ch", type=int, default=3, choices=[1, 3], help="3 = colour, 1 = grey")
    ap.add_argument("--width", type=float, default=1.0)
    ap.add_argument("--depth", type=int, default=1)
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--lr", type=float, default=3e-3)
    ap.add_argument("--heading-weight", type=float, default=1.0, help="weight of the heading loss")
    ap.add_argument("--heading-mode", default="front", choices=["front", "sincos"],
                    help="front: find the robot's front point (default); sincos: regress sin/cos of heading")
    ap.add_argument("--weight-decay", type=float, default=1e-4)
    ap.add_argument("--patience", type=int, default=12, help="early stop after this many epochs without improvement")
    ap.add_argument("--min-epochs", type=int, default=0, help="never stop early before this many epochs")
    ap.add_argument("--max-minutes", type=float, default=0, help="stop after this long (0 = no limit)")
    ap.add_argument("--val-frac", type=float, default=0.15)
    ap.add_argument("--test-frac", type=float, default=0.15)
    ap.add_argument("--val", help="comma list of recordings for validation")
    ap.add_argument("--test", help="comma list of recordings held back for the final test")
    ap.add_argument("--split", help="reuse a split.json from an earlier run")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--deterministic", action="store_true", help="repeatable results (a bit slower)")
    ap.add_argument("--no-augment", action="store_true")
    ap.add_argument("--no-rotate", action="store_true", help="augment without rotation")
    ap.add_argument("--hue-p", type=float, default=0.5,
                    help="share of training frames whose colours are turned round the colour wheel (0 = off)")
    ap.add_argument("--select", default="pooled", choices=["pooled", "macro"],
                    help="best epoch by F1 over all objects pooled (default) or by the average of per-class F1s")
    ap.add_argument("--radius", default="8", help="match radius in frame px for validation scores (e.g. 8,hand=25)")
    ap.add_argument("--score-thresh", type=float, default=0.3)
    ap.add_argument("--workers", type=int, default=-1, help="data loader processes (-1 = auto)")
    ap.add_argument("--threads", type=int, default=0, help="CPU threads for PyTorch (0 = default)")
    ap.add_argument("--device", default="auto", help="auto, cpu or cuda")
    ap.add_argument("--init-from", help="start from an earlier best.pt (e.g. after adding a class)")
    ap.add_argument("--no-hard-negatives", action="store_true",
                    help="ignore the spots of rejected guesses (by default they are taught as 'not an object here')")
    ap.add_argument("--neg-boost", type=float, default=None,
                    help="how much more a rejected spot counts than ordinary background (default 3)")
    ap.add_argument("--paste", action="store_true",
                    help="copy-paste augmentation: labelled objects pasted onto other TRAINING frames (see paste.py)")
    ap.add_argument("--paste-p", type=float, default=0.5, help="share of training pictures that get objects pasted")
    ap.add_argument("--paste-areas", help="JSON {recording: [[x, y], ...]}: the work area to paste inside, per recording")
    ap.add_argument("--steps-per-epoch", type=int, default=0,
                    help="0 = one pass over the training frames; otherwise this many batches per epoch")
    a = ap.parse_args()

    a.radius = detcore.parse_radius(a.radius)
    set_seed(a.seed, a.deterministic)
    if a.threads:
        torch.set_num_threads(a.threads)
    device = torch.device("cuda" if (a.device == "auto" and torch.cuda.is_available()) else
                          ("cpu" if a.device == "auto" else a.device))
    os.makedirs(a.out, exist_ok=True)

    in_w, in_h = parse_size(a.size)
    check_size(in_w, in_h)
    classes = a.classes.split(",") if a.classes else None
    cfg = default_config(classes=classes, in_ch=a.in_ch, in_w=in_w, in_h=in_h, width=a.width, depth=a.depth,
                         heading_mode=a.heading_mode)
    cfg["score_thresh"] = a.score_thresh
    if a.obj_size:
        for kv in a.obj_size.split(","):
            k, v = kv.split("=")
            cfg["obj_size_px"][k.strip()] = float(v)
    if a.heading_classes is not None:
        cfg["heading_classes"] = [c for c in a.heading_classes.split(",") if c]
    for c in cfg["classes"]:
        if c not in cfg["obj_size_px"]:
            print(f"No --obj-size for class {c!r}: assuming 30 px")
            cfg["obj_size_px"][c] = 30.0

    # ---- data and split (whole recordings only) ----
    all_samples = detcore.read_many(a.labels, cfg["classes"], neg_only=not a.no_hard_negatives,
                                    sizes=cfg["obj_size_px"])
    # frames that hold only 'not an object here' spots join training only; they take no part in the split
    samples = [s for s in all_samples if not s.get("neg_only")]
    neg_frames = [s for s in all_samples if s.get("neg_only")]
    if a.no_hard_negatives:
        for s in all_samples:
            s["negatives"] = []
    if a.neg_boost is not None:
        DetDataset.NEG_BOOST = a.neg_boost
    if a.split:
        assign = detcore.load_split(a.split)
        missing = sorted({s["recording"] for s in samples} - set(assign))
        if missing:
            print(f"Recordings not in {a.split} go to train: {missing}")
            for m in missing:
                assign[m] = "train"
    else:
        assign = detcore.make_split(samples, a.val_frac, a.test_frac, a.seed,
                                    a.val.split(",") if a.val else None, a.test.split(",") if a.test else None)
    detcore.save_split(assign, os.path.join(a.out, "split.json"))
    tr, va = detcore.select(samples, assign, "train"), detcore.select(samples, assign, "val")
    tr += detcore.select(neg_frames, assign, "train")
    te = detcore.select(samples, assign, "test")
    # Validation picks the best epoch, so it is scored only on frames a person
    # checked; training may use auto-accepted frames too.
    n_va = len(va)
    va = detcore.human_checked(va)
    if n_va != len(va):
        print(f"val: left out {n_va - len(va)} auto-accepted frames")
    te = detcore.human_checked(te)
    for name, part in (("train", tr), ("val", va), ("test", te)):
        recs = sorted({s["recording"] for s in part})
        n_obj = sum(len(s["objects"]) for s in part)
        per = detcore.distinct_objects(part)
        print(f"{name:5s}: {len(part):5d} frames, {n_obj:5d} labels, {len(recs)} recordings; different objects: "
              + ", ".join(f"{c} {per.get(c, 0)}" for c in cfg["classes"]))
    if not [s for s in tr if not s.get("neg_only")] or not va:
        sys.exit("Need frames in both train and val.")
    n_negs = sum(len(s.get("negatives", [])) for s in tr)
    if n_negs:
        print(f"hard negatives: {n_negs} rejected spots in {sum(1 for s in tr if s.get('negatives'))} training frames "
              f"({len([s for s in tr if s.get('neg_only')])} of them frames used only for that)")

    aug = None if a.no_augment else AugmentConfig(rotate=not a.no_rotate, hue_p=a.hue_p)
    t0 = time.time()
    pcfg = None
    if a.paste and aug is not None:
        areas = {}
        if a.paste_areas:
            with open(a.paste_areas) as f:
                areas = json.load(f)
        pcfg = PasteConfig(p=a.paste_p, areas=areas, seed=a.seed)
        # honest scoring: only training frames are ever cut from or pasted onto
        assert all(assign.get(s["recording"]) == "train" for s in tr), "copy-paste must see training frames only"
    ds_tr = DetDataset(tr, cfg, augment=aug, paste=pcfg)
    ds_va = DetDataset(va, cfg, augment=None, use_negatives=False)
    if ds_tr.paste is not None:
        summ = ds_tr.paste.summary()
        summ.pop("train_frames")
        summ.pop("targets")
        with open(os.path.join(a.out, "paste_report.json"), "w") as f:
            json.dump(summ, f, indent=1)
        print(f"copy-paste: {summ['crops']} cut-outs ({summ['with_difference_mask']} outlined from the background "
              f"difference, {summ['with_circle']} with a soft circle), from {len(summ['sources'])} training frames")
    print(f"Loaded and shrank {len(tr) + len(va)} frames in {time.time() - t0:.0f} s")
    workers = a.workers if a.workers >= 0 else (0 if os.name == "nt" else min(4, os.cpu_count() or 1))
    g = torch.Generator()
    g.manual_seed(a.seed)
    sampler = None
    if a.steps_per_epoch:
        sampler = torch.utils.data.RandomSampler(ds_tr, replacement=True, num_samples=a.steps_per_epoch * a.batch,
                                                 generator=g)
    dl_tr = DataLoader(ds_tr, batch_size=a.batch, shuffle=sampler is None, sampler=sampler, num_workers=workers,
                       collate_fn=collate, drop_last=len(ds_tr) > a.batch, worker_init_fn=seed_worker,
                       generator=g, persistent_workers=workers > 0)
    dl_va = DataLoader(ds_va, batch_size=a.batch, shuffle=False, num_workers=0, collate_fn=collate)

    # ---- model ----
    model = CentreNet(cfg).to(device)
    if a.init_from:
        init_from(model, a.init_from)
    print(describe(model))
    print(f"Device: {device}, threads: {torch.get_num_threads()}, workers: {workers}")
    with open(os.path.join(a.out, "config.json"), "w") as f:
        json.dump(dict(model=cfg, args=vars(a), params=count_params(model)), f, indent=2)

    opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=a.weight_decay)
    steps_total = a.epochs * len(dl_tr)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=a.lr, total_steps=steps_total, pct_start=0.1)

    log_path = os.path.join(a.out, "train_log.csv")
    cols = ["epoch", "seconds", "lr", "train_loss", "train_heat", "train_off", "train_heading", "val_loss",
            "val_heat", "val_off", "val_heading", "val_precision", "val_recall", "val_f1",
            "val_centre_err_px", "val_heading_err_deg", "val_macro_f1", "val_class_f1", "best"]
    logf = open(log_path, "w", newline="")
    log = csv.DictWriter(logf, fieldnames=cols)
    log.writeheader()

    best, best_epoch, t_start = (-1, -1.0), -1, time.time()
    for epoch in range(1, a.epochs + 1):
        te0 = time.time()
        model.train()
        tl, tp, n = 0.0, np.zeros(3), 0
        for batch in dl_tr:
            x = batch["image"].to(device, non_blocking=True)
            t = {k: v.to(device, non_blocking=True) for k, v in batch.items()
                 if isinstance(v, torch.Tensor) and k != "image"}
            loss, parts = compute_loss(model(x, logits=True), t, w_hd=a.heading_weight)
            opt.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0)
            opt.step()
            sched.step()
            tl += loss.item() * len(x)
            tp += np.array(parts) * len(x)
            n += len(x)
        vl, vp, vrow = validate(model, dl_va, device, cfg, a.radius, a.score_thresh, a.heading_weight)
        f1 = vrow.get("macro_f1" if a.select == "macro" else "f1", 0.0)
        err = vrow.get("centre_err_px_mean", float("nan"))
        herr = vrow.get("heading_err_deg_mean", float("nan"))
        # Score used to pick the best epoch: mostly F1; then smaller centre error
        # (1 px = 0.01 F1) and smaller heading error (10 deg = 0.01 F1).
        score = f1 - 0.01 * (err if err == err else 10.0) - 0.001 * (herr if herr == herr else 180.0)
        # while nothing is found (F1 0) every epoch scores the same on F1: rank those by validation
        # loss instead; any epoch that finds something beats every epoch that finds nothing
        key = (1, score) if f1 > 0 else (0, -vl)
        improved = key > best
        if improved:
            best, best_epoch = key, epoch
            torch.save(dict(model=model.state_dict(), config=cfg, epoch=epoch, val=vrow),
                       os.path.join(a.out, "best.pt"))
        torch.save(dict(model=model.state_dict(), config=cfg, epoch=epoch, val=vrow), os.path.join(a.out, "last.pt"))
        row = dict(epoch=epoch, seconds=round(time.time() - te0, 1), lr=f"{sched.get_last_lr()[0]:.2e}",
                   train_loss=round(tl / n, 4), train_heat=round(tp[0] / n, 4), train_off=round(tp[1] / n, 4),
                   train_heading=round(tp[2] / n, 4), val_loss=round(vl, 4), val_heat=round(vp[0], 4),
                   val_off=round(vp[1], 4), val_heading=round(vp[2], 4),
                   val_precision=round(vrow.get("precision", 0), 4), val_recall=round(vrow.get("recall", 0), 4),
                   val_f1=round(vrow.get("f1", 0.0), 4), val_centre_err_px=round(err, 3),
                   val_macro_f1=round(vrow.get("macro_f1", 0.0), 4),
                   val_class_f1=" ".join(f"{k}={v}" for k, v in vrow.get("per_class_f1", {}).items()),
                   val_heading_err_deg=round(vrow.get("heading_err_deg_mean", float("nan")), 2),
                   best=int(improved))
        log.writerow(row)
        logf.flush()
        print(f"epoch {epoch:3d}  {row['seconds']:5.1f}s  loss {row['train_loss']:.3f}  val {row['val_loss']:.3f}  "
              f"P {row['val_precision']:.3f} R {row['val_recall']:.3f} F1 {row['val_f1']:.3f}  "
              f"err {row['val_centre_err_px']:.2f}px  hd {row['val_heading_err_deg']:.1f}deg  "
              f"[{row['val_class_f1']}]{'  *' if improved else ''}")
        if best[0] == 1 and epoch >= a.min_epochs and epoch - best_epoch >= a.patience:
            print(f"No improvement for {a.patience} epochs: stopping early.")
            break
        if a.max_minutes and (time.time() - t_start) / 60 > a.max_minutes:
            print(f"Time limit of {a.max_minutes} min reached.")
            break
    logf.close()
    print(f"Best epoch {best_epoch} (val score {best[1]:.4f}{'' if best[0] else ', found nothing yet: lowest val loss'}). Saved {os.path.join(a.out, 'best.pt')}, "
          f"total {(time.time() - t_start) / 60:.1f} min")


if __name__ == "__main__":
    main()
