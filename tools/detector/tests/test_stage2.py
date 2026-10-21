"""Stage 2 training extras in the detector: rejected guesses as 'not an object here' examples and
copy-paste augmentation."""
import math
import os
import random

import cv2
import numpy as np
import pytest
import torch

import detcore
import stage2_synth as S
from dataset import AugmentConfig, DetDataset
from model import default_config
from paste import PasteBank, PasteConfig
from train import compute_loss, focal_loss


def _cfg():
    cfg = default_config(classes=["alvik", "hand", "unknown", "machine"], in_w=320, in_h=240)
    cfg["obj_size_px"] = dict(alvik=24.0, hand=40.0, unknown=24.0, machine=60.0)
    return cfg


def _recording(tmp_path, name="A", n=12, with_stain=False, seed=1):
    rng = np.random.default_rng(seed)
    frames = []
    for i in range(n):
        img, rb = S.scene(rng, n_robots=int(rng.integers(1, 3)))
        frames.append((img, [S.robot_row(x, y, h, obj=k + 1) for k, (x, y, h) in enumerate(rb)]))
    return S.write_recording(str(tmp_path), name, frames), frames


def test_read_labels_negatives_and_default(tmp_path):
    img = np.full((240, 320, 3), 150, np.uint8)
    rows = [
        [0, "", 1, "alvik", 100, 100, 0, 1, "user", "confirmed", ""],
        [0, "", "", "alvik", 200, 50, "", 0, "motion", "rejected", "0.4"],         # frame 0: one object, one rejected spot
        [1, "", "", "alvik", 210, 60, "", 0, "motion", "rejected", "0.4"],         # frame 1: only a rejected spot
        [2, "", 1, "alvik", 100, 100, 0, 1, "track", "pending", "0.7"],
        [2, "", "", "alvik", 30, 30, "", 0, "motion", "rejected", "0.4"],          # frame 2: pending object + rejected spot
        [3, "", 1, "alvik", 100, 100, 0, 1, "track", "pending", "0.7"],            # frame 3: pending, nothing rejected
    ]
    from conftest import write_labels
    d = write_labels(str(tmp_path / "rec" / "labels"), rows, {i: img for i in range(4)})
    plain = {s["frame"]: s for s in detcore.read_labels(d, verbose=False)}
    assert sorted(plain) == [0]                                                  # as before: only the finished frame
    assert plain[0]["negatives"] == [dict(x=200.0, y=50.0, cls="alvik")] and not plain[0]["neg_only"]
    assert [o["cls"] for o in plain[0]["objects"]] == ["alvik"]
    withneg = {s["frame"]: s for s in detcore.read_labels(d, verbose=False, neg_only=True)}
    assert sorted(withneg) == [0, 1, 2]
    assert withneg[1]["neg_only"] and withneg[1]["objects"] == [] and not withneg[1]["auto"]
    assert withneg[2]["neg_only"] and withneg[2]["negatives"][0]["x"] == 30.0
    assert 3 not in withneg


def test_negative_targets_and_no_change_without_rejects(tmp_path):
    cfg = _cfg()
    d, _ = _recording(tmp_path)
    samples = detcore.read_many([d], cfg["classes"], verbose=False)
    ds = DetDataset(samples, cfg)
    it = ds[0]
    assert it["negw"].shape == (1, 60, 80) and float(it["negw"].min()) == 1.0 == float(it["negw"].max())   # nothing rejected: nothing changes
    # a rejected spot counts extra around itself, and a negative-only frame counts nowhere else
    s = dict(samples[0], negatives=[dict(x=160.0, y=120.0, cls="alvik")])
    boosted = DetDataset([s], cfg)[0]["negw"][0].numpy()
    assert boosted[30, 40] == DetDataset.NEG_BOOST and boosted[5, 5] == 1.0
    only = dict(samples[0], objects=[], neg_only=True, negatives=[dict(x=160.0, y=120.0, cls="alvik")])
    item = DetDataset([only], cfg)[0]
    w = item["negw"][0].numpy()
    assert w[30, 40] == DetDataset.NEG_BOOST and w[5, 5] == 0.0 and w.sum() > 0
    assert float(item["heat"].sum()) == 0 and float(item["fmask"][0][5, 5]) == 0.0
    # the same sample with negatives switched off (validation) is the plain one
    off = DetDataset([s], cfg, use_negatives=False)[0]["negw"]
    assert float(off.min()) == 1.0 == float(off.max())


def test_negatives_follow_the_augmentation(tmp_path):
    cfg = _cfg()
    cv2.imwrite(str(tmp_path / "f.jpg"), np.full((240, 320, 3), 150, np.uint8))
    s = dict(recording="r", frame=0, image=str(tmp_path / "f.jpg"), objects=[], auto=False, neg_only=True,
             negatives=[dict(x=160.0, y=120.0, cls="alvik"), dict(x=60.0, y=60.0, cls="alvik")])
    random.seed(3)
    np.random.seed(3)
    ds = DetDataset([s], cfg, augment=AugmentConfig(rotate=True, flip=True, scale=0.0, shift=0.0))
    for _ in range(6):                 # rotation and mirroring about the middle keep the middle spot in the middle
        w = ds[0]["negw"][0].numpy()
        assert w[30, 40] == DetDataset.NEG_BOOST
        assert 0 < (w > 0).sum() < 200            # the other spot moved with the picture (or left it)
    assert (ds[0]["heat"] == 0).all()


def test_focal_loss_negative_weights():
    torch.manual_seed(0)
    logits = torch.randn(1, 1, 8, 8)
    target = torch.zeros(1, 1, 8, 8)
    target[0, 0, 4, 4] = 1.0
    base = focal_loss(logits, target)
    assert torch.allclose(focal_loss(logits, target, neg_w=torch.ones(1, 1, 8, 8)), base)
    boost = torch.ones(1, 1, 8, 8)
    boost[0, 0, 1, 1] = 3.0
    assert focal_loss(logits, target, neg_w=boost) > base
    none = torch.zeros(1, 1, 8, 8)
    none[0, 0, 4, 4] = 1.0                                   # only the object's own cell counts
    only_pos = focal_loss(logits, target, neg_w=none)
    p = torch.sigmoid(logits)[0, 0, 4, 4]
    assert abs(float(only_pos) + float(torch.log(p) * (1 - p) ** 2)) < 1e-5
    # and the full loss accepts batches with and without the extra map
    heat = torch.randn(1, 4, 8, 8)
    batch = dict(heat=torch.zeros(1, 4, 8, 8), off=torch.zeros(1, 2, 8, 8), off_w=torch.zeros(1, 8, 8),
                 fheat=torch.zeros(1, 1, 8, 8), fmask=torch.ones(1, 1, 8, 8), foff=torch.zeros(1, 2, 8, 8),
                 foff_w=torch.zeros(1, 8, 8))
    outs = (heat, torch.zeros(1, 2, 8, 8), torch.zeros(1, 3, 8, 8))
    a = compute_loss(outs, batch)[0]
    b = compute_loss(outs, dict(batch, negw=torch.ones(1, 1, 8, 8)))[0]
    assert torch.allclose(a, b)


def test_paste_stays_in_training_frames_and_the_work_area(tmp_path):
    cfg = _cfg()
    dtr, _ = _recording(tmp_path, "A", n=14, seed=1)
    dva, _ = _recording(tmp_path, "B", n=6, seed=2)
    tr = detcore.read_many([dtr], cfg["classes"], verbose=False)
    va = detcore.read_many([dva], cfg["classes"], verbose=False)
    area = [[60, 40], [260, 40], [260, 200], [60, 200]]
    aug = AugmentConfig(rotate=False, flip=False, scale=0.0, shift=0.0, photometric=False)
    random.seed(1)
    ds = DetDataset(tr, cfg, augment=aug, paste=PasteConfig(p=1.0, max_n=2, areas={"A": area}))
    bank = ds.paste
    assert bank.bank and bank.images == {s["image"] for s in tr}
    assert bank.sources <= {s["image"] for s in tr} and not bank.sources & {s["image"] for s in va}
    before = [len(s["objects"]) for s in tr]
    got = 0
    for i in range(len(tr)):
        it = ds[i]
        for o in it["objects"][before[i]:]:
            got += 1
            assert 60 - 8 <= o["x"] <= 260 + 8 and 40 - 8 <= o["y"] <= 200 + 8          # inside the work area
        old = it["objects"][:before[i]]
        for o in it["objects"][before[i]:]:
            assert all(math.hypot(o["x"] - q["x"], o["y"] - q["y"]) > 14 for q in old)  # never on top of a labelled object
    assert got > 5 and bank.n_pasted == got and bank.targets <= {s["image"] for s in tr}
    # the pasted object really is in the picture: the pixels there changed from the plain frame
    plain = DetDataset(tr, cfg, augment=aug)
    random.seed(2)
    for i in range(len(tr)):
        a, b = ds[i], plain[i]
        if len(a["objects"]) > len(b["objects"]):
            o = a["objects"][-1]
            x, y = int(o["x"]), int(o["y"])
            assert np.abs(a["image"][:, y - 6:y + 6, x - 6:x + 6].numpy() - b["image"][:, y - 6:y + 6, x - 6:x + 6].numpy()).max() > 0.3
            break
    else:
        pytest.fail("nothing was pasted")
    # validation (and test) frames: a dataset built without paste is the plain frame, and the bank refuses them
    dsv = DetDataset(va, cfg, augment=None)
    assert dsv.paste is None
    foreign = PasteBank.__new__(PasteBank)
    foreign.__dict__.update(bank.__dict__)
    foreign.ds = DetDataset(va, cfg, augment=aug)                  # a different set of frames
    img2, objs2 = foreign.apply(0, np.zeros((240, 320, 3), np.uint8), [], [])
    assert not img2.any() and objs2 == []                           # untouched


def test_paste_needs_training_setup():
    with pytest.raises(ValueError):
        DetDataset([], _cfg(), augment=None, paste=PasteConfig())


def test_paste_headings_follow_the_cutout(tmp_path):
    cfg = _cfg()
    d, _ = _recording(tmp_path, n=14, seed=4)
    tr = detcore.read_many([d], cfg["classes"], verbose=False)
    aug = AugmentConfig(rotate=False, flip=False, scale=0.0, shift=0.0, photometric=False)
    ds = DetDataset(tr, cfg, augment=aug, paste=PasteConfig(p=1.0, max_n=1))
    heads = [o["heading"] for i in range(len(tr)) for o in ds[i]["objects"] if o.get("pasted")]
    assert heads and all(h is not None and 0 <= h < 360 for h in heads)


# --------------------------------------------------------------------------------------------
# training on synthetic scenes (about a minute each)
# --------------------------------------------------------------------------------------------

def _stain_world(root, n_pos=24, n_neg=20):
    """Recording A: robot frames (no stain) plus frames showing a stain where a guess was rejected and
    nothing else was labelled; recording B (validation): robots only."""
    import json
    rng = np.random.default_rng(3)
    a, b = [], []
    for _ in range(n_pos):
        img, rb = S.scene(rng, n_robots=int(rng.integers(1, 3)))
        a.append((img, [S.robot_row(x, y, h, obj=k + 1) for k, (x, y, h) in enumerate(rb)]))
    for _ in range(n_neg):
        sp = (rng.uniform(40, 280), rng.uniform(40, 200))
        img, _ = S.scene(rng, n_robots=0, stains=[sp])
        a.append((img, [S.reject_row(*sp)]))
    for _ in range(10):
        img, rb = S.scene(rng, n_robots=1)
        b.append((img, [S.robot_row(x, y, h) for (x, y, h) in rb]))
    S.write_recording(str(root), "A", a)
    S.write_recording(str(root), "B", b)
    with open(str(root / "split.json"), "w") as f:
        json.dump({"by_recording": {"A": "train", "B": "val"}}, f)


def _train(root, out, extra, epochs=16):
    import subprocess
    import sys
    det = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cmd = [sys.executable, os.path.join(det, "train.py"), "--labels", str(root / "A"), str(root / "B"),
           "--out", str(out), "--split", str(root / "split.json"), "--size", "192x160", "--width", "0.5", "--epochs", str(epochs),
           "--min-epochs", str(epochs), "--batch", "8", "--workers", "0", "--steps-per-epoch", "12", "--seed", "1",
           "--no-rotate", "--obj-size", "alvik=24", "--heading-classes", "alvik", "--classes", "alvik,hand,unknown,machine",
           "--deterministic"] + extra
    r = subprocess.run(cmd, capture_output=True, text=True)
    assert r.returncode == 0, r.stdout[-1500:] + r.stderr[-1500:]
    return r.stdout


def _stain_hits(pt, n=24):
    import predict_torch
    m = predict_torch.load(str(pt), score_thresh=0.3)
    rng = np.random.default_rng(99)
    hits = found = 0
    for _ in range(n):
        sp = (rng.uniform(40, 280), rng.uniform(40, 200))
        img, rb = S.scene(rng, n_robots=1, stains=[sp])
        dets = predict_torch.predict(m, img)
        hits += sum(math.hypot(d["x"] - sp[0], d["y"] - sp[1]) < 14 for d in dets)
        found += any(math.hypot(d["x"] - rb[0][0], d["y"] - rb[0][1]) < 8 for d in dets)
    return hits, found


def test_rejected_guesses_teach_the_model_to_ignore_a_stain(tmp_path):
    _stain_world(tmp_path)
    out_on = _train(tmp_path, tmp_path / "on", [])
    assert "hard negatives: 20 rejected spots" in out_on and "20 of them frames used only" in out_on
    out_off = _train(tmp_path, tmp_path / "off", ["--no-hard-negatives"])
    assert "hard negatives" not in out_off
    hits_on, found_on = _stain_hits(tmp_path / "on" / "best.pt")
    hits_off, found_off = _stain_hits(tmp_path / "off" / "best.pt")
    print("stain detections in 24 frames: with rejected spots %d, without %d; robots found %d / %d"
          % (hits_on, hits_off, found_on, found_off))
    assert hits_off >= 4 and hits_on <= hits_off // 4         # the stain fooled the model without them, not with them
    assert found_on >= 20                                       # and the robots are still found


def test_copy_paste_training_uses_training_frames_only(tmp_path):
    import json
    _stain_world(tmp_path, n_pos=16, n_neg=0)
    out = _train(tmp_path, tmp_path / "p", ["--paste", "--paste-p", "1.0"], epochs=2)
    assert "copy-paste:" in out
    rep = json.load(open(str(tmp_path / "p" / "paste_report.json")))
    train_imgs = {str(tmp_path / "A" / "labels" / "frames" / ("%06d.jpg" % i)) for i in range(16)}
    assert rep["sources"] and set(rep["sources"]) <= train_imgs          # cut only from training frames
    assert rep["crops"] > 0
