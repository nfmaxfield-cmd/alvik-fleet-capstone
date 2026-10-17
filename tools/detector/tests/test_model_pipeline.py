"""Model shapes, export to ONNX, the detector interface, and short end-to-end runs."""
import csv
import os
import subprocess
import sys

import numpy as np
import pytest
import torch

import detcore
from model import CentreNet, check_size, count_params, default_config

DET_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.mark.parametrize("in_ch,w,h,width,depth,mode", [(3, 320, 240, 1.0, 1, "front"), (1, 640, 480, 0.5, 2, "sincos")])
def test_shapes(in_ch, w, h, width, depth, mode):
    net = CentreNet(default_config(in_ch=in_ch, in_w=w, in_h=h, width=width, depth=depth, heading_mode=mode)).eval()
    heat, off, hd = net(torch.zeros(1, in_ch, h, w))
    assert heat.shape == (1, 4, h // 4, w // 4)
    assert off.shape == (1, 2, h // 4, w // 4)
    assert hd.shape == (1, 3 if mode == "front" else 2, h // 4, w // 4)
    assert 0 <= float(heat.min()) and float(heat.max()) <= 1
    if mode == "front":
        assert 0 <= float(hd[:, 0].min()) and float(hd[:, 0].max()) <= 1
    assert count_params(net) > 1000


def test_size_check():
    check_size(320, 240)
    with pytest.raises(ValueError):
        check_size(330, 240)


def _save_untrained(path, **cfg):
    c = default_config(**cfg)
    c["score_thresh"] = 0.3
    torch.save(dict(model=CentreNet(c).state_dict(), config=c), path)


def test_export_and_interfaces(tmp_path):
    import export
    import infer
    import predict_torch
    pt = str(tmp_path / "m.pt")
    _save_untrained(pt)
    model = export.export(pt, str(tmp_path / "m.onnx"))
    assert export.check(model, str(tmp_path / "m.onnx")) < 1e-3
    frame = np.random.default_rng(0).integers(0, 255, (480, 640, 3), dtype=np.uint8)
    om = infer.load(str(tmp_path / "m.onnx"), threads=1, score_thresh=0.0)
    tm = predict_torch.load(pt, score_thresh=0.0)
    a, b = infer.predict(om, frame), predict_torch.predict(tm, frame)
    assert len(a) > 0
    for d in a:
        assert set(d) == {"cls", "x", "y", "heading_deg", "score"}
        assert d["cls"] in detcore.DEFAULT_CLASSES
    assert isinstance(b, list) and all(set(d) == {"cls", "x", "y", "heading_deg", "score"} for d in b)
    # both paths see the same network output (an untrained net has many near-tied
    # peaks, so compare the maps rather than the order of the detections)
    heat, off, hd, _ = om.raw(frame)
    x, _ = detcore.preprocess(frame, 320, 240, 3)
    with torch.no_grad():
        t_heat, t_off, t_hd = tm.net(torch.from_numpy(x))
    assert np.abs(heat - t_heat[0].numpy()).max() < 1e-4
    assert np.abs(hd - t_hd[0].numpy()).max() < 1e-4
    # the same file runs at another input size
    big = infer.load(str(tmp_path / "m.onnx"), input_size=(640, 480))
    heat, _, _, _ = big.raw(frame)
    assert heat.shape == (4, 120, 160)


def test_init_from_adds_class(tmp_path):
    from train import init_from
    pt = str(tmp_path / "old.pt")
    _save_untrained(pt, classes=["alvik", "hand"])
    old = torch.load(pt, weights_only=False)["model"]
    new = CentreNet(default_config(classes=["alvik", "hand", "forklift"]))
    init_from(new, pt)
    sd = new.state_dict()
    assert torch.equal(sd["out.weight"][:2], old["out.weight"][:2])        # old classes kept
    assert torch.equal(sd["out.weight"][3:], old["out.weight"][2:])        # offset kept
    assert torch.equal(sd["out_hd.weight"], old["out_hd.weight"])          # heading head kept
    assert torch.equal(sd["stem.0.weight"], old["stem.0.weight"])


def test_train_export_evaluate_end_to_end(tiny_synth, tmp_path):
    """A couple of training steps, export, evaluate (detector and CSV), benchmark."""
    run = str(tmp_path / "run")
    py = sys.executable
    subprocess.run([py, os.path.join(DET_DIR, "train.py"), "--labels", tiny_synth, "--out", run, "--epochs", "1",
                    "--steps-per-epoch", "2", "--batch", "4", "--workers", "0", "--deterministic"], check=True)
    assert os.path.isfile(os.path.join(run, "best.pt"))
    with open(os.path.join(run, "train_log.csv")) as f:
        assert len(list(csv.DictReader(f))) == 1
    subprocess.run([py, os.path.join(DET_DIR, "export.py"), os.path.join(run, "best.pt")], check=True)
    onnx_path = os.path.join(run, "best.onnx")
    preds = str(tmp_path / "preds.csv")
    out = str(tmp_path / "eval.csv")
    subprocess.run([py, os.path.join(DET_DIR, "evaluate.py"), "--labels", tiny_synth, "--split",
                    os.path.join(run, "split.json"), "--model", f"infer:{onnx_path}", "--score-thresh", "0.05",
                    "--save-predictions", preds, "--out", out], check=True)
    subprocess.run([py, os.path.join(DET_DIR, "evaluate.py"), "--labels", tiny_synth, "--split",
                    os.path.join(run, "split.json"), "--model", f"csv:{preds}", "--out", out], check=True)
    with open(out) as f:
        assert len(list(csv.DictReader(f))) >= 1
    subprocess.run([py, os.path.join(DET_DIR, "benchmark.py"), "--model", onnx_path, "--sizes", "320x240",
                    "--threads", "1", "--warmup", "2", "--iters", "5"], check=True)


def test_baseline_one_step_and_interface(tiny_synth, tmp_path):
    """The torchvision baseline builds without downloads, trains a step and answers predict()."""
    import baseline_tv
    net = baseline_tv.build("ssdlite", 4, "none", 320, 240)
    net.train()
    imgs = [torch.rand(3, 240, 320), torch.rand(3, 240, 320)]      # 2 images: batch norm needs more than 1
    tg = [baseline_tv.to_targets([dict(cls="alvik", x=100, y=80, heading=None)], detcore.DEFAULT_CLASSES,
                                 detcore.DEFAULT_OBJ_SIZE_PX, 0.5),
          baseline_tv.to_targets([], detcore.DEFAULT_CLASSES, detcore.DEFAULT_OBJ_SIZE_PX, 0.5)]
    loss = sum(net(imgs, tg).values())
    loss.backward()
    cfg = default_config()
    p = str(tmp_path / "b.pt")
    torch.save(dict(model=net.state_dict(), config=cfg, arch="ssdlite", weights="none"), p)
    det = baseline_tv.load(p, score_thresh=0.0)
    dets = baseline_tv.predict(det, np.zeros((480, 640, 3), np.uint8))
    assert isinstance(dets, list)
    for d in dets:
        assert d["heading_deg"] is None and set(d) == {"cls", "x", "y", "heading_deg", "score"}
