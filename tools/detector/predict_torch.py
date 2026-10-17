"""The detector interface for PyTorch weights (best.pt from train.py), for
use on the laptop without exporting first (e.g. in the labelling tool).

    import predict_torch
    model = predict_torch.load("runs/v1/best.pt")
    dets = predict_torch.predict(model, frame_bgr)   # [dict(cls, x, y, heading_deg, score), ...]

Gives the same answers as infer.py on the exported .onnx (up to tiny
rounding differences); infer.py is the faster one and the one for the Pi.
"""
import os
import sys

import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import detcore  # noqa: E402
from model import load_checkpoint  # noqa: E402


class TorchDetector:
    def __init__(self, path, score_thresh=None, input_size=None, device="cpu", threads=None):
        if threads:
            torch.set_num_threads(int(threads))
        self.device = torch.device(device)
        self.net = load_checkpoint(path, map_location=self.device).to(self.device)
        self.cfg = self.net.cfg
        self.classes = self.cfg["classes"]
        self.in_w, self.in_h = input_size or (self.cfg["in_w"], self.cfg["in_h"])
        self.score_thresh = score_thresh if score_thresh is not None else self.cfg.get("score_thresh", 0.3)
        self.max_objects = 50

    @torch.no_grad()
    def __call__(self, frame_bgr):
        x, scale = detcore.preprocess(frame_bgr, self.in_w, self.in_h, self.cfg["in_ch"])
        heat, off, hd = self.net(torch.from_numpy(x).to(self.device))
        return detcore.decode(heat[0].cpu().numpy(), off[0].cpu().numpy(), hd[0].cpu().numpy(), self.cfg,
                              scale, self.score_thresh, self.max_objects)


def load(path, score_thresh=None, input_size=None, device="cpu", threads=None):
    return TorchDetector(path, score_thresh, input_size, device, threads)


def predict(model, frame_bgr):
    return model(frame_bgr)
