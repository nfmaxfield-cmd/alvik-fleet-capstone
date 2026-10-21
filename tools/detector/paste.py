"""Copy-paste augmentation: more training pictures from the labels already made.

Objects that a person labelled are cut out of the training frames and pasted onto other
training frames, at spots inside the work area where an object could plausibly be (not on
top of another object, not on a "not an object" spot). The pasted object brings its label
(type, position, heading), so the model sees each object in many more places and, with some
pairs pasted close together, in the crowded situations it finds hardest.

Rules that keep the scores honest:
- the pictures cut from, and pasted onto, are TRAINING frames only: a PasteBank is built from
  the training dataset's own frames, and refuses to paste onto any other frame;
- pasting happens in memory while training; nothing on disk is changed;
- scoring (picking the best round and the final test) never uses it.

The cut-out's outline (its "soft mask") comes from the difference between the object's frame
and a background picture (the middle value of a few training frames of the same recording,
where moving objects wash out). Where that gives nothing usable (an object that never moved is
part of the background), a soft-edged circle is used instead. Nothing here needs more than
OpenCV and numpy, and it adds a few milliseconds per picture.
"""
import math
import random

import cv2
import numpy as np

import detcore


class PasteConfig:
    """p: chance a training picture gets objects pasted; max_n: most objects pasted into one;
    areas: {recording name: polygon [[x, y], ...] in frame px} the work area (None/missing: the
    region where labelled objects were seen, padded); close_p: share of pastes put next to an
    existing object (1 to 1.6 sizes away) so pairs are practised; bank_max: crops kept."""

    def __init__(self, p=0.5, max_n=2, areas=None, close_p=0.3, bank_max=400, bg_frames=15, seed=0):
        self.p, self.max_n, self.areas = p, max_n, areas or {}
        self.close_p, self.bank_max, self.bg_frames, self.seed = close_p, bank_max, bg_frames, seed


def _hull(points, pad):
    pts = np.array(points, np.float32)
    if len(pts) < 3:
        return None
    hull = cv2.convexHull(pts)
    if len(hull) < 3 or cv2.contourArea(hull) < 4 * pad * pad:
        return None
    return hull


class PasteBank:
    def __init__(self, ds, cfg):
        """ds: the TRAINING DetDataset (frames already shrunk, ds._cache filled)."""
        self.ds, self.cfg = ds, cfg
        self.images = {s["image"] for s in ds.samples}          # the only frames it may touch
        self.scales = {s_["recording"]: (ds._cache[i][1][0] + ds._cache[i][1][1]) / 2
                       for i, s_ in enumerate(ds.samples)}
        self.sizes = ds.cfg.get("obj_size_px", detcore.DEFAULT_OBJ_SIZE_PX)
        self.targets = set()      # frames pasted onto (seen only in the process that did the pasting)
        self.sources = set()                                      # frames cut from
        self.n_pasted = 0
        self.n_mask = self.n_circle = 0
        self.bg = self._backgrounds()
        self.bank = self._build()
        self.regions = self._regions()

    # -- background and cut-outs --------------------------------------------------------------

    def _backgrounds(self):
        by = {}
        for i, s in enumerate(self.ds.samples):
            if s.get("neg_only"):
                continue
            by.setdefault(s["recording"], []).append(i)
        out = {}
        for rec, idx in by.items():
            if len(idx) < 5:
                continue
            empty = [i for i in idx if not self.ds.samples[i]["objects"]]
            pick = empty if len(empty) >= 5 else idx
            step = max(1, len(pick) // self.cfg.bg_frames)
            stack = np.stack([self.ds._cache[i][0] for i in pick[::step][: self.cfg.bg_frames]])
            out[rec] = np.median(stack, axis=0).astype(np.uint8)
        return out

    def _size_in(self, cls, scale):
        return self.sizes.get(cls, 30) * scale

    def _build(self):
        items = []
        for i, s in enumerate(self.ds.samples):
            if s.get("neg_only") or not s["objects"]:
                continue
            img, (sx, sy) = self.ds._cache[i]
            scale = (sx + sy) / 2
            objs = detcore.transform_objects(s["objects"], detcore.resize_matrix(sx, sy))
            negs = detcore.transform_objects([dict(o, heading=None) for o in s.get("negatives", [])],
                                             detcore.resize_matrix(sx, sy))
            for k, o in enumerate(objs):
                size = self._size_in(o["cls"], scale)
                # not a crop that holds a second object or a rejected spot, or hangs over the edge
                others = [p for j, p in enumerate(objs) if j != k] + negs
                if any(math.hypot(p["x"] - o["x"], p["y"] - o["y"]) < 1.3 * size for p in others):
                    continue
                S = int(math.ceil(1.5 * size)) | 1
                x0, y0 = int(round(o["x"])) - S // 2, int(round(o["y"])) - S // 2
                H, W = img.shape[:2]
                if x0 < 0 or y0 < 0 or x0 + S > W or y0 + S > H or S < 5:
                    continue
                crop = img[y0:y0 + S, x0:x0 + S].copy()
                bg = self.bg.get(s["recording"])
                mask, kind = self._mask(crop, None if bg is None else bg[y0:y0 + S, x0:x0 + S], size)
                items.append(dict(cls=o["cls"], crop=crop, mask=mask, heading=o["heading"], size=size, S=S,
                                  kind=kind, image=s["image"], rec=s["recording"]))
        # keep it balanced across types: at most bank_max, an equal share per type
        by = {}
        for it in items:
            by.setdefault(it["cls"], []).append(it)
        out = []
        per = max(1, self.cfg.bank_max // max(1, len(by)))
        for c, lst in by.items():
            step = max(1, len(lst) // per)
            out.extend(lst[::step][:per])
        for it in out:
            self.sources.add(it["image"])
            self.n_mask += it["kind"] == "difference"
            self.n_circle += it["kind"] == "circle"
        return out

    @staticmethod
    def circle_mask(S, size):
        yy, xx = np.mgrid[0:S, 0:S].astype(np.float32)
        d = np.hypot(xx - S // 2, yy - S // 2)
        sig = max(1.0, 0.12 * size)
        return np.clip((0.6 * size - d) / (2 * sig) + 0.5, 0, 1).astype(np.float32)

    def _mask(self, crop, bg, size):
        """(soft mask 0..1, how it was made): from the background difference when that picks out
        an object-sized blob, else a feathered circle."""
        S = crop.shape[0]
        circle = self.circle_mask(S, size)
        if bg is None:
            return circle, "circle"
        diff = np.abs(crop.astype(np.int16) - bg.astype(np.int16)).max(axis=2).astype(np.uint8)
        diff = cv2.GaussianBlur(diff, (0, 0), 1.0)
        m = (diff > 18).astype(np.uint8)
        yy, xx = np.mgrid[0:S, 0:S].astype(np.float32)
        m[np.hypot(xx - S // 2, yy - S // 2) > 0.72 * size] = 0
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
        if n <= 1:
            return circle, "circle"
        c = lab[S // 2, S // 2]
        if c == 0:                       # the middle itself is not different: take the largest blob
            c = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        blob = (lab == c).astype(np.uint8)
        frac = blob.sum() / (size * size)        # a square object of the labelled size would give about 1
        if not (0.4 <= frac <= 1.6):
            return circle, "circle"
        blob = cv2.dilate(blob, np.ones((3, 3), np.uint8))
        return np.clip(cv2.GaussianBlur(blob.astype(np.float32), (0, 0), 1.2) * 1.1, 0, 1), "difference"

    # -- where to paste -----------------------------------------------------------------------

    def _regions(self):
        """Per recording: (polygon in input px, or None for the padded hull of the labelled spots)."""
        out = {}
        pts = {}
        for i, s in enumerate(self.ds.samples):
            if s.get("neg_only"):
                continue
            img, (sx, sy) = self.ds._cache[i]
            for o in s["objects"]:
                pts.setdefault(s["recording"], []).append((o["x"] * sx, o["y"] * sy))
        recs = {s["recording"]: self.ds._cache[i][1] for i, s in enumerate(self.ds.samples)}
        for rec, (sx, sy) in recs.items():
            poly = self.cfg.areas.get(rec)
            if poly and len(poly) >= 3:
                out[rec] = np.array([[x * sx, y * sy] for x, y in poly], np.float32)
            else:
                h = _hull(pts.get(rec, []), 30.0)
                out[rec] = None if h is None else h.reshape(-1, 2)
        return out

    def _inside(self, rec, x, y, pad):
        poly = self.regions.get(rec)
        if poly is None:
            return True
        return cv2.pointPolygonTest(poly.reshape(-1, 1, 2), (float(x), float(y)), True) >= -pad

    # -- the augmentation ---------------------------------------------------------------------

    def apply(self, i, img, objs, negs):
        """Maybe paste objects onto sample i's (shrunk) picture. Returns (img, objs). Only frames
        this bank was built from; for any other frame the picture comes back untouched."""
        s = self.ds.samples[i]
        if (not self.bank or s["image"] not in self.images or s.get("neg_only")
                or random.random() >= self.cfg.p):
            return img, objs
        img = img.copy()
        objs = list(objs)
        H, W = img.shape[:2]
        rec = s["recording"]
        scale = self.scales.get(rec, 1.0)
        added = 0
        for _ in range(random.randint(1, self.cfg.max_n)):
            classes = sorted({it["cls"] for it in self.bank})
            c = random.choice(classes)                              # equal chance per type
            it = random.choice([b for b in self.bank if b["cls"] == c])
            pos = self._spot(rec, it, objs, negs, W, H, scale)
            if pos is None:
                continue
            ang = random.uniform(0, 360)
            self._blend(img, it, pos, ang)
            h = None
            if it["heading"] is not None:
                M = cv2.getRotationMatrix2D((it["S"] // 2, it["S"] // 2), ang, 1.0)
                h = detcore.transform_objects([dict(x=it["S"] // 2, y=it["S"] // 2, heading=it["heading"])], M)[0]["heading"]
            objs.append(dict(cls=it["cls"], x=float(pos[0]), y=float(pos[1]), heading=h, obj=None, pasted=True))
            added += 1
        if added:
            self.targets.add(s["image"])
            self.n_pasted += added
        return img, objs

    def _spot(self, rec, it, objs, negs, W, H, scale):
        """A spot for this cut-out: inside the work area, whole cut-out in the picture, not on another
        object or a rejected spot; a share of them next to an object so pairs are practised."""
        half = it["S"] // 2 + 1
        near = random.random() < self.cfg.close_p and bool(objs)
        for _try in range(30):
            if near:
                o = random.choice(objs)
                other = self._size_in(o["cls"], scale)
                d = random.uniform(1.0, 1.6) * 0.5 * (it["size"] + other)
                a = random.uniform(0, 2 * math.pi)
                x, y = int(round(o["x"] + d * math.cos(a))), int(round(o["y"] + d * math.sin(a)))
            else:
                x, y = random.randint(half, W - half - 1), random.randint(half, H - half - 1)
            if not (half <= x < W - half and half <= y < H - half):
                continue
            if not self._inside(rec, x, y, 0.3 * it["size"]):
                continue
            ok = True
            for o in objs:
                other = self._size_in(o["cls"], scale)
                gap = (0.8 if near else 1.6) * 0.5 * (it["size"] + other)
                if math.hypot(o["x"] - x, o["y"] - y) < gap:
                    ok = False
                    break
            if ok and any(math.hypot(n["x"] - x, n["y"] - y) < 0.8 * it["size"] for n in negs):
                ok = False
            if ok:
                return x, y
        return None

    @staticmethod
    def _blend(img, it, pos, ang):
        S = it["S"]
        M = cv2.getRotationMatrix2D((S // 2, S // 2), ang, 1.0)
        crop = cv2.warpAffine(it["crop"], M, (S, S), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        mask = cv2.warpAffine(it["mask"], M, (S, S), flags=cv2.INTER_LINEAR)
        x0, y0 = pos[0] - S // 2, pos[1] - S // 2
        region = img[y0:y0 + S, x0:x0 + S]
        if region.shape[:2] != (S, S):
            return
        # match the light of the place it lands in: ring of pixels outside the object, both sides
        ring = (mask < 0.05)
        crop_f = crop.astype(np.float32)
        if ring.sum() > 20:
            gain = (region[ring].mean(axis=0) + 1.0) / (crop[ring].mean(axis=0) + 1.0)
            crop_f = crop_f * np.clip(gain, 0.75, 1.33)[None, None, :]
        m = mask[:, :, None]
        out = region.astype(np.float32) * (1 - m) + np.clip(crop_f, 0, 255) * m
        img[y0:y0 + S, x0:x0 + S] = np.clip(out, 0, 255).astype(np.uint8)

    def summary(self):
        return dict(sources=sorted(self.sources), targets=sorted(self.targets), n_pasted=self.n_pasted,
                    crops=len(self.bank), with_difference_mask=self.n_mask, with_circle=self.n_circle,
                    train_frames=sorted(self.images))
