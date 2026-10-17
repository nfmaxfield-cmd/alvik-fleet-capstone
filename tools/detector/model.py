"""The project's own detector network: a small "centre heatmap" network.

Plain English: the network looks at a (shrunk) camera frame and paints, for
each class, a map that is bright where it thinks the CENTRE of an object of
that class is. The map is 4 times smaller than the input (output stride 4),
so for every 4x4 block of input pixels it also guesses:
  - offset (2 maps): where inside that block the centre really is, so the
    position is not rounded to the block grid
  - heading: by default ("front" mode) a second little heatmap that is
    bright at the robot's FRONT point (a spot a set distance ahead of the
    centre, where the Alvik's LEDs are), plus its own offset (3 maps). The
    heading is then the direction from the centre to the front point.
    Finding "the front" is a local look-for-a-pattern job, the same kind of
    job as finding centres, which a small network learns quickly. The other
    mode ("sincos", 2 maps: sin and cos of the heading at the centre) is
    kept for comparison; in tests on synthetic data it learned far more slowly.

Finding objects is then just "find the bright spots" (see detcore.decode).
This is the idea behind CenterNet (Zhou et al. 2019, "Objects as Points"),
but the network itself is our own compact design:

    input (1 or 3 channels, e.g. 320x240)
      stem       3x3 conv, stride 2                        -> 1/2 size
      stage s4   3x3 conv stride 2, then `depth` blocks    -> 1/4
      stage s8   downsample, then `depth` blocks           -> 1/8
      stage s16  downsample, then `depth` blocks           -> 1/16 (sees context)
      neck       bring 1/16 back up to 1/8 and 1/4, adding the finer stages
      heads      heatmap (one per class), offset (2); heading head (3 or 2)

A "block" is a MobileNetV2-style inverted residual (1x1 expand, 3x3
depthwise, 1x1 project, plus a skip connection): cheap on ARM CPUs.
`width` scales every channel count, `depth` sets blocks per stage, so speed
vs accuracy can be measured (see benchmark.py --arch).

    python model.py                       # print sizes for a few settings
    python model.py --width 1.0 --depth 2 --size 320x240
"""
import argparse
import math

import torch
import torch.nn as nn
import torch.nn.functional as F

try:
    from detcore import DEFAULT_CLASSES, DEFAULT_HEADING_CLASSES, DEFAULT_OBJ_SIZE_PX
except ImportError:  # imported as tools.detector.model
    from .detcore import DEFAULT_CLASSES, DEFAULT_HEADING_CLASSES, DEFAULT_OBJ_SIZE_PX

STRIDE = 4          # output map is 4x smaller than the input
DOWNSAMPLE = 16     # input width and height must be multiples of this


def default_config(**over):
    """Everything needed to rebuild the model; saved with the weights."""
    cfg = dict(
        classes=list(DEFAULT_CLASSES),
        obj_size_px=dict(DEFAULT_OBJ_SIZE_PX),        # rough object size in camera-frame px
        heading_classes=sorted(DEFAULT_HEADING_CLASSES),  # classes that have a front
        in_ch=3,           # 3 = colour (RGB), 1 = greyscale
        in_w=320, in_h=240,
        width=1.0,         # channel multiplier
        depth=1,           # blocks per stage
        expand=2,          # inverted-residual expansion
        stride=STRIDE,
        heading_mode="front",   # "front" (front-point heatmap) or "sincos"
        front_frac=0.35,        # front point distance ahead of the centre, as a share of obj size
        frame_w=640, frame_h=480,   # the camera frame size the labels are in
    )
    cfg.update({k: v for k, v in over.items() if v is not None})
    return cfg


def _ch(c, width):
    """Scale a channel count and round to a multiple of 8 (fast on CPUs)."""
    return max(8, int(round(c * width / 8)) * 8)


class ConvBNReLU(nn.Sequential):
    def __init__(self, cin, cout, k=3, s=1, groups=1, relu=True):
        layers = [nn.Conv2d(cin, cout, k, s, k // 2, groups=groups, bias=False), nn.BatchNorm2d(cout)]
        if relu:
            layers.append(nn.ReLU(inplace=True))
        super().__init__(*layers)


class InvertedResidual(nn.Module):
    """1x1 expand -> 3x3 depthwise -> 1x1 project, with a skip connection."""

    def __init__(self, c, expand=2):
        super().__init__()
        m = c * expand
        self.body = nn.Sequential(
            ConvBNReLU(c, m, 1),
            ConvBNReLU(m, m, 3, groups=m),
            ConvBNReLU(m, c, 1, relu=False),
        )

    def forward(self, x):
        return F.relu(x + self.body(x))


class Down(nn.Sequential):
    """Halve the size: 3x3 depthwise stride 2, then 1x1 to the new width."""

    def __init__(self, cin, cout):
        super().__init__(ConvBNReLU(cin, cin, 3, 2, groups=cin), ConvBNReLU(cin, cout, 1))


class SepConv(nn.Sequential):
    """3x3 depthwise + 1x1: a cheap 3x3 convolution."""

    def __init__(self, cin, cout):
        super().__init__(ConvBNReLU(cin, cin, 3, groups=cin), ConvBNReLU(cin, cout, 1))


class CentreNet(nn.Module):
    def __init__(self, cfg=None):
        super().__init__()
        self.cfg = cfg = default_config(**(cfg or {}))
        w, d, e = cfg["width"], cfg["depth"], cfg["expand"]
        c1, c2, c3, c4 = _ch(16, w), _ch(24, w), _ch(48, w), _ch(96, w)
        neck = _ch(32, w)
        n_cls = len(cfg["classes"])

        self.stem = ConvBNReLU(cfg["in_ch"], c1, 3, 2)
        self.s4 = nn.Sequential(ConvBNReLU(c1, c2, 3, 2), *[InvertedResidual(c2, e) for _ in range(d)])
        self.s8 = nn.Sequential(Down(c2, c3), *[InvertedResidual(c3, e) for _ in range(d)])
        self.s16 = nn.Sequential(Down(c3, c4), *[InvertedResidual(c4, e) for _ in range(d)])

        self.lat16 = ConvBNReLU(c4, neck, 1)
        self.lat8 = ConvBNReLU(c3, neck, 1)
        self.lat4 = ConvBNReLU(c2, neck, 1)
        self.fuse8 = SepConv(neck, neck)
        self.fuse4 = SepConv(neck, neck)

        # Two small heads on the shared features: one for "where" (heatmaps +
        # offset), one for "which way" (heading). Separate heads let the heading
        # learn its own features (e.g. the LEDs at the robot's front) instead of
        # competing with the heatmap for the same channels.
        self.head = SepConv(neck, neck)
        self.out = nn.Conv2d(neck, n_cls + 2, 1)
        self.head_hd = nn.Sequential(SepConv(neck, neck), SepConv(neck, neck))
        self.front = cfg.get("heading_mode", "front") == "front"
        self.out_hd = nn.Conv2d(neck, 3 if self.front else 2, 1)
        self.n_cls = n_cls
        self._init()

    def _init(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode="fan_out", nonlinearity="relu")
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
        # Start the heatmap near "nothing here" (probability 0.01), as in
        # focal-loss papers, so early training is stable.
        nn.init.normal_(self.out.weight, std=0.01)
        with torch.no_grad():
            self.out.bias[: self.n_cls] = -math.log((1 - 0.01) / 0.01)
            if self.front:
                nn.init.normal_(self.out_hd.weight, std=0.01)
                self.out_hd.bias[0] = -math.log((1 - 0.01) / 0.01)

    def forward(self, x, logits=False):
        """Returns (heat, offset, heading). heat (and, in front mode, heading
        channel 0) are probabilities unless logits=True (training uses logits
        for a stable loss)."""
        f4 = self.s4(self.stem(x))
        f8 = self.s8(f4)
        f16 = self.s16(f8)
        p = self.lat16(f16)
        p = self.fuse8(F.interpolate(p, scale_factor=2.0, mode="nearest") + self.lat8(f8))
        p = self.fuse4(F.interpolate(p, scale_factor=2.0, mode="nearest") + self.lat4(f4))
        o = self.out(self.head(p))
        heat = o[:, : self.n_cls]
        offset = o[:, self.n_cls:]
        heading = self.out_hd(self.head_hd(p))
        if not logits:
            heat = torch.sigmoid(heat)
            if self.front:   # channel 0 is the front-point heatmap
                heading = torch.cat([torch.sigmoid(heading[:, :1]), heading[:, 1:]], 1)
        return heat, offset, heading


def count_params(model):
    return sum(p.numel() for p in model.parameters())


def count_macs(model, in_w, in_h):
    """Multiply-accumulates for one frame (convolutions only, which dominate)."""
    total = [0]

    def hook(m, inp, out):
        k = m.kernel_size[0] * m.kernel_size[1]
        total[0] += out.numel() * (m.in_channels // m.groups) * k

    hs = [m.register_forward_hook(hook) for m in model.modules() if isinstance(m, nn.Conv2d)]
    was = model.training
    model.eval()
    with torch.no_grad():
        model(torch.zeros(1, model.cfg["in_ch"], in_h, in_w))
    model.train(was)
    for h in hs:
        h.remove()
    return total[0]


def check_size(w, h):
    if w % DOWNSAMPLE or h % DOWNSAMPLE:
        raise ValueError(f"Input size {w}x{h}: width and height must be multiples of {DOWNSAMPLE} "
                         f"(e.g. 320x240, 480x352, 640x480)")


def parse_size(s):
    w, h = s.lower().split("x")
    return int(w), int(h)


def describe(model, in_w=None, in_h=None):
    cfg = model.cfg
    in_w, in_h = in_w or cfg["in_w"], in_h or cfg["in_h"]
    return (f"CentreNet width={cfg['width']} depth={cfg['depth']} in={in_w}x{in_h}x{cfg['in_ch']} "
            f"classes={len(cfg['classes'])}: {count_params(model):,} parameters, "
            f"{count_macs(model, in_w, in_h) / 1e6:.0f} M multiply-adds per frame")


def load_checkpoint(path, map_location="cpu"):
    """Rebuild a model from a .pt file written by train.py."""
    ck = torch.load(path, map_location=map_location, weights_only=False)
    model = CentreNet(ck["config"])
    model.load_state_dict(ck["model"])
    model.eval()
    return model


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--width", type=float)
    ap.add_argument("--depth", type=int)
    ap.add_argument("--in-ch", type=int, choices=[1, 3])
    ap.add_argument("--size", help="input WxH, e.g. 320x240")
    a = ap.parse_args()
    if a.width or a.depth or a.size or a.in_ch:
        w, h = parse_size(a.size) if a.size else (320, 240)
        check_size(w, h)
        print(describe(CentreNet(default_config(width=a.width, depth=a.depth, in_ch=a.in_ch, in_w=w, in_h=h))))
        return
    for width, depth in [(0.5, 1), (1.0, 1), (1.0, 2), (1.5, 2)]:
        for size in [(320, 240), (640, 480)]:
            print(describe(CentreNet(default_config(width=width, depth=depth, in_w=size[0], in_h=size[1]))))


if __name__ == "__main__":
    main()
