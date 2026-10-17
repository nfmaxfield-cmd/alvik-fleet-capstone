"""Training data for the centre-heatmap detector (PyTorch).

Reads labels folders (via detcore), shrinks each frame to the network input
size, applies random changes that a fixed overhead camera could really see
(augmentation), and builds the targets the network learns from:

- heat:    one map per class, a small Gaussian "blob" of height 1 on each
           object centre, 0 elsewhere
- offset:  where inside the 4x4 output cell the true centre is
- heading (default "front" mode): a blob on each robot's front point (a
           spot a set distance ahead of its centre) and its offset; in
           "sincos" mode, sin and cos of the heading at the centre

Augmentations (training only), chosen for an overhead fixed camera:
- rotation by any angle: from above, a robot can face any way and be
  anywhere, so a rotated frame is still a realistic frame. Headings are
  rotated with the picture (detcore.transform_objects).
- mirror flip: also realistic from above; mirroring reverses which way is
  counter-clockwise, which transform_objects handles automatically.
- small zoom (default +/-10%) and shift: camera height / framing changes
- brightness, contrast, colour balance, gamma, blur, sensor noise and JPEG
  re-compression: lighting and webcam changes between sessions

Splitting is done in detcore (whole recordings, never single frames).
"""
import math
import random

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

import detcore


class AugmentConfig:
    """How strong the random changes are. Set rotate=False etc. to switch off."""

    def __init__(self, rotate=True, flip=True, scale=0.10, shift=0.08, photometric=True,
                 blur_p=0.3, noise_std=6.0, jpeg_p=0.3):
        self.rotate, self.flip, self.scale, self.shift = rotate, flip, scale, shift
        self.photometric, self.blur_p, self.noise_std, self.jpeg_p = photometric, blur_p, noise_std, jpeg_p


class DetDataset(Dataset):
    """samples: list from detcore.read_many. cfg: model config (model.default_config)."""

    def __init__(self, samples, cfg, augment=None, cache=True, with_maps=True):
        self.samples = samples
        self.cfg = cfg
        self.aug = augment          # an AugmentConfig, or None for no augmentation
        self.with_maps = with_maps  # False: return only image + objects (for the baseline)
        self.in_w, self.in_h = cfg["in_w"], cfg["in_h"]
        self.stride = cfg["stride"]
        self.classes = cfg["classes"]
        self.cls_index = {c: i for i, c in enumerate(self.classes)}
        self.heading_classes = set(cfg.get("heading_classes", detcore.DEFAULT_HEADING_CLASSES))
        self._cache = {}
        if cache:
            # Shrink every frame once, up front (fast later, about 0.25 MB per
            # frame at 320x240). Done before DataLoader workers start so they share it.
            for i in range(len(samples)):
                self._cache[i] = self._load(i)

    def __len__(self):
        return len(self.samples)

    def _load(self, i):
        """Read a frame and shrink it to the input size with area averaging,
        the same resize the live detector uses (detcore.preprocess)."""
        s = self.samples[i]
        img = cv2.imread(s["image"], cv2.IMREAD_COLOR)
        if img is None:
            raise IOError(f"Cannot read {s['image']}")
        h, w = img.shape[:2]
        small = cv2.resize(img, (self.in_w, self.in_h), interpolation=cv2.INTER_AREA)
        return small, (self.in_w / w, self.in_h / h)

    def __getitem__(self, i):
        small, (sx, sy) = self._cache[i] if i in self._cache else self._load(i)
        s = self.samples[i]
        # objects in input-pixel coordinates (same half-pixel convention as cv2.resize)
        objs = detcore.transform_objects(s["objects"], detcore.resize_matrix(sx, sy))
        img = small
        if self.aug is not None:
            img, objs = self._geometric(img, objs)
            img = self._photometric(img)
        objs = [o for o in objs if 0 <= o["x"] < self.in_w and 0 <= o["y"] < self.in_h]
        x = torch.from_numpy(detcore.normalize(detcore.to_channels(img, self.cfg["in_ch"])))
        item = dict(image=x, objects=objs, index=i, scale=(sx, sy))
        if self.with_maps:
            item.update(self._targets(objs, (sx + sy) / 2))
        return item

    # -- augmentation -----------------------------------------------------

    def _geometric(self, img, objs):
        a = self.aug
        cx, cy = self.in_w / 2, self.in_h / 2
        ang = random.uniform(0, 360) if a.rotate else 0.0
        k = math.exp(random.uniform(-a.scale, a.scale)) if a.scale else 1.0
        M = cv2.getRotationMatrix2D((cx, cy), ang, k)    # positive angle = counter-clockwise on screen
        if a.flip and random.random() < 0.5:
            # mirror left-right first, then rotate: M @ F
            F = np.array([[-1, 0, self.in_w - 1], [0, 1, 0], [0, 0, 1]], np.float64)
            M = M @ F
        if a.shift:
            M[0, 2] += random.uniform(-a.shift, a.shift) * self.in_w
            M[1, 2] += random.uniform(-a.shift, a.shift) * self.in_h
        if ang == 0 and k == 1 and M[0, 0] == 1 and M[0, 2] == 0 and M[1, 2] == 0:
            return img, objs
        # fill the uncovered corners with the floor's typical colour, not black
        fill = tuple(float(v) for v in np.median(img.reshape(-1, 3), axis=0))
        out = cv2.warpAffine(img, M, (self.in_w, self.in_h), flags=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=fill)
        return out, detcore.transform_objects(objs, M)

    def _photometric(self, img):
        a = self.aug
        if not a.photometric:
            return img
        f = img.astype(np.float32)
        contrast = random.uniform(0.7, 1.3)
        bright = random.uniform(-30, 30)
        gains = np.array([random.uniform(0.9, 1.1) for _ in range(3)], np.float32)   # white balance
        f = (f - 128) * contrast + 128 + bright
        f = f * gains
        f = np.clip(f, 0, 255)
        gamma = random.uniform(0.8, 1.25)
        f = 255.0 * (f / 255.0) ** gamma
        if random.random() < a.blur_p:
            f = cv2.GaussianBlur(f, (0, 0), random.uniform(0.3, 1.0))
        if a.noise_std:
            f = f + np.random.normal(0, random.uniform(0, a.noise_std), f.shape).astype(np.float32)
        out = np.clip(f, 0, 255).astype(np.uint8)
        if random.random() < a.jpeg_p:
            ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, random.randint(50, 90)])
            if ok:
                out = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return out

    # -- targets ------------------------------------------------------------

    def _targets(self, objs, scale):
        st = self.stride
        oh, ow = self.in_h // st, self.in_w // st
        heat = np.zeros((len(self.classes), oh, ow), np.float32)
        off = np.zeros((2, oh, ow), np.float32)
        off_w = np.zeros((oh, ow), np.float32)
        hd = np.zeros((2, oh, ow), np.float32)
        hd_w = np.zeros((oh, ow), np.float32)
        yy, xx = np.mgrid[0:oh, 0:ow].astype(np.float32)
        sizes = self.cfg.get("obj_size_px", detcore.DEFAULT_OBJ_SIZE_PX)
        for o in objs:
            c = self.cls_index[o["cls"]]
            cx, cy = o["x"] / st, o["y"] / st                      # centre in output cells
            ix, iy = int(cx), int(cy)
            size_cells = sizes.get(o["cls"], 30) * scale / st
            sigma = max(0.8, size_cells / 6.0)
            g = np.exp(-((xx - ix) ** 2 + (yy - iy) ** 2) / (2 * sigma * sigma))
            np.maximum(heat[c], g, out=heat[c])
            heat[c, iy, ix] = 1.0
            # offset and heading are learned on the peak cell (weight 1) and its
            # 8 neighbours (weight 0.5), so a peak that lands one cell off
            # still gives a good position and heading.
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    jx, jy = ix + dx, iy + dy
                    if not (0 <= jx < ow and 0 <= jy < oh):
                        continue
                    wgt = 1.0 if dx == 0 and dy == 0 else 0.5
                    if wgt < off_w[jy, jx]:
                        continue
                    off[0, jy, jx], off[1, jy, jx] = cx - jx, cy - jy
                    off_w[jy, jx] = wgt
                    if o.get("heading") is not None and o["cls"] in self.heading_classes:
                        r = math.radians(o["heading"])
                        hd[0, jy, jx], hd[1, jy, jx] = math.sin(r), math.cos(r)
                        hd_w[jy, jx] = wgt
                    else:
                        hd_w[jy, jx] = 0.0
        out = dict(heat=torch.from_numpy(heat), off=torch.from_numpy(off), off_w=torch.from_numpy(off_w))
        if self.cfg.get("heading_mode", "front") == "front":
            out.update(self._front_targets(objs, scale, xx, yy))
        else:
            out.update(hd=torch.from_numpy(hd), hd_w=torch.from_numpy(hd_w))
        return out

    def _front_targets(self, objs, scale, xx, yy):
        """Front-point mode: a small blob on each robot's front point (a spot
        front_frac x size ahead of the centre), with its sub-cell offset.
        Around a robot whose heading is NOT labelled the front-point map is
        masked out of the loss: its front is somewhere there, we just do not
        know where, so it must not be taught as "no front here"."""
        st = self.stride
        oh, ow = xx.shape
        fheat = np.zeros((1, oh, ow), np.float32)
        fmask = np.ones((1, oh, ow), np.float32)
        foff = np.zeros((2, oh, ow), np.float32)
        foff_w = np.zeros((oh, ow), np.float32)
        sizes = self.cfg.get("obj_size_px", detcore.DEFAULT_OBJ_SIZE_PX)
        frac = self.cfg.get("front_frac", 0.35)
        for o in objs:
            if o["cls"] not in self.heading_classes:
                continue
            size = sizes.get(o["cls"], 30) * scale            # input px
            if o.get("heading") is None:
                r = 0.75 * size / st
                fmask[0][(xx - o["x"] / st) ** 2 + (yy - o["y"] / st) ** 2 <= r * r] = 0.0
                continue
            dx, dy = detcore.heading_to_vec(o["heading"])
            fx, fy = (o["x"] + frac * size * dx) / st, (o["y"] + frac * size * dy) / st
            ix, iy = int(fx), int(fy)
            if not (0 <= ix < ow and 0 <= iy < oh) or fx < 0 or fy < 0:
                continue
            g = np.exp(-((xx - ix) ** 2 + (yy - iy) ** 2) / (2 * 0.8 ** 2))
            np.maximum(fheat[0], g, out=fheat[0])
            fheat[0, iy, ix] = 1.0
            for ddy in (-1, 0, 1):
                for ddx in (-1, 0, 1):
                    jx, jy = ix + ddx, iy + ddy
                    if 0 <= jx < ow and 0 <= jy < oh:
                        wgt = 1.0 if ddx == 0 and ddy == 0 else 0.5
                        if wgt >= foff_w[jy, jx]:
                            foff[0, jy, jx], foff[1, jy, jx] = fx - jx, fy - jy
                            foff_w[jy, jx] = wgt
        return dict(fheat=torch.from_numpy(fheat), fmask=torch.from_numpy(fmask),
                    foff=torch.from_numpy(foff), foff_w=torch.from_numpy(foff_w))


def collate(batch):
    """Stack the tensors; keep the object lists as plain Python lists."""
    out = {}
    for k in batch[0]:
        if isinstance(batch[0][k], torch.Tensor):
            out[k] = torch.stack([b[k] for b in batch])
        else:
            out[k] = [b[k] for b in batch]
    return out
