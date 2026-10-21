#!/usr/bin/env python3
"""What an object looks like: small picture patches, for the tracker's appearance chain.

The tracker (tracker.py) uses this to keep following an object that the motion finder or the
model does not see (a robot that stopped, an object that never moves): each frame it compares the
small patch around the object with the patch from before. Only small windows around objects are
ever compared (OpenCV matchTemplate on crops of a few dozen pixels), never the whole picture.

Pictures are worked on in grey and made smaller first, so an object is about 15 pixels across
(choose_scale): enough to match, and quick.

    frames = VideoFrames("rec/video.avi", robot_px=30)      # or ArrayFrames(fn, n, (w, h), robot_px)
    look = Looker(frames, robot_px=30)
    tpl = look.patch(f, x, y)                  # the picture around (x, y) on frame f (None: not available)
    s = look.same_place(f, x, y, tpl)          # how alike the patch at exactly (x, y) is (-1 .. 1)
    hit = look.search(f, x, y, radius_px, tpl) # best place near (x, y): dict(x, y, score, second, ambiguous)
    similarity(a, b)                           # how alike two patches are, with flat patches handled

Frames are fetched lazily and only where an object needs them; the crops are kept (up to a memory
budget) so following the guesses again after a label is saved does not decode the video again.
"""
import collections
import math

import numpy as np
import cv2


def choose_scale(robot_px):
    """How much smaller the pictures are made for matching: an object ends up ~14-28 px across.
    Only 1, 1/2, 1/4 and 1/8 (a Motion-JPEG frame can be decoded straight to those sizes)."""
    r = float(robot_px or 30.0)
    s = 1.0
    while r * s >= 28.0 and s > 0.125:
        s /= 2.0
    return s


class Frames:
    """Grey, made-smaller pictures of a recording, fetched by frame number.

    window(f, x0, y0, x1, y1) gives the part of frame f between those (small-picture) pixel
    coordinates, padded by repeating the edge where it runs off the picture; None when frame f
    can't be read. Crops are cached in blocks aligned to a GRID-pixel grid, so following the
    guesses again a little to the side still finds them in the cache."""

    GRID = 8

    def __init__(self, n, size, scale, budget_mb=256):
        self.n = int(n)
        self.scale = float(scale)
        self.w = int(round(size[0] * self.scale))
        self.h = int(round(size[1] * self.scale))
        self.budget = int(budget_mb * 1024 * 1024)
        self.cache = collections.OrderedDict()
        self.cached_bytes = 0
        self._last = (None, None)        # (frame, picture): the last decoded frame
        self.stats = collections.Counter()

    # --- for subclasses -------------------------------------------------------------------
    def _decode(self, f):
        """The whole grey small picture of frame f (uint8, h x w), or None."""
        raise NotImplementedError

    # --------------------------------------------------------------------------------------
    def frame(self, f):
        if self._last[0] == f:
            return self._last[1]
        if f < 0 or f >= self.n:
            return None
        img = self._decode(f)
        self.stats["decoded"] += 1
        self._last = (f, img)
        return img

    def window(self, f, x0, y0, x1, y1):
        x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
        g = self.GRID
        bx0 = max(0, (x0 // g) * g)
        by0 = max(0, (y0 // g) * g)
        bx1 = min(self.w, -(-x1 // g) * g)
        by1 = min(self.h, -(-y1 // g) * g)
        if bx1 <= bx0 or by1 <= by0:
            return None
        key = (f, bx0, by0, bx1, by1)
        block = self.cache.get(key)
        if block is not None:
            self.cache.move_to_end(key)
            self.stats["hits"] += 1
        else:
            img = self.frame(f)
            if img is None:
                return None
            block = img[by0:by1, bx0:bx1].copy()
            self.cache[key] = block
            self.cached_bytes += block.nbytes + 120
            while self.cached_bytes > self.budget and self.cache:
                _, old = self.cache.popitem(last=False)
                self.cached_bytes -= old.nbytes + 120
            self.stats["misses"] += 1
        # cut the asked part out of the block, repeating the edge where it runs off the picture
        cx0, cy0 = max(x0, bx0), max(y0, by0)
        cx1, cy1 = min(x1, bx1), min(y1, by1)
        if cx1 <= cx0 or cy1 <= cy0:
            return None
        out = block[cy0 - by0:cy1 - by0, cx0 - bx0:cx1 - bx0]
        pads = (cy0 - y0, y1 - cy1, cx0 - x0, x1 - cx1)
        if any(pads):
            out = cv2.copyMakeBorder(out, pads[0], pads[1], pads[2], pads[3], cv2.BORDER_REPLICATE)
        return out

    def close(self):
        pass


class ArrayFrames(Frames):
    """Frames from a function f -> picture (full size, grey or colour). For tests and for
    pictures already in memory."""

    def __init__(self, fn, n, size, robot_px=30.0, scale=None, budget_mb=256):
        super().__init__(n, size, scale or choose_scale(robot_px), budget_mb)
        self.fn = fn

    def _decode(self, f):
        img = self.fn(f)
        if img is None:
            return None
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if self.scale != 1.0:
            img = cv2.resize(img, (self.w, self.h), interpolation=cv2.INTER_AREA)
        return img


class VideoFrames(Frames):
    """Frames from a video file. Motion-JPEG (what the camera recorder writes) is decoded straight to
    the small grey size, which is several times quicker than a full decode; other videos are
    decoded fully and made smaller. Reads forward in order when it can, jumps when it must."""

    _REDUCED = {1.0: cv2.IMREAD_GRAYSCALE, 0.5: cv2.IMREAD_REDUCED_GRAYSCALE_2,
                0.25: cv2.IMREAD_REDUCED_GRAYSCALE_4, 0.125: cv2.IMREAD_REDUCED_GRAYSCALE_8}

    def __init__(self, path, robot_px=30.0, n=None, size=None, scale=None, budget_mb=256):
        self.path = path
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            raise IOError("could not open " + path)
        if size is None:
            size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if n is None:
            n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
        cap.release()
        super().__init__(n, size, scale or choose_scale(robot_px), budget_mb)
        self.size = size
        self.cap = None
        self.raw = None             # True: packets decoded by imdecode (Motion-JPEG)
        self.pos = None             # frame number the next read() gives

    def _open(self):
        self.cap = cv2.VideoCapture(self.path, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.path)
        self.pos = 0
        if self.raw is None:
            self.raw = False
            try:
                if self.cap.set(cv2.CAP_PROP_FORMAT, -1):
                    ok, buf = self.cap.read()
                    img = cv2.imdecode(buf.reshape(-1), cv2.IMREAD_GRAYSCALE) if ok and buf is not None else None
                    if img is not None and img.shape[:2] == (self.size[1], self.size[0]):
                        self.raw = True
            except cv2.error:
                pass
            self.cap.release()
            self.cap = cv2.VideoCapture(self.path, cv2.CAP_FFMPEG) if self.raw else cv2.VideoCapture(self.path)
        if self.raw:
            self.cap.set(cv2.CAP_PROP_FORMAT, -1)

    def _read(self):
        ok, data = self.cap.read()
        self.pos += 1
        if not ok or data is None:
            return None
        if self.raw:
            flag = self._REDUCED.get(self.scale)
            img = cv2.imdecode(data.reshape(-1), flag if flag is not None else cv2.IMREAD_GRAYSCALE)
            if img is None:
                return None
        else:
            img = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY) if data.ndim == 3 else data
        if img.shape[1] != self.w or img.shape[0] != self.h:
            img = cv2.resize(img, (self.w, self.h), interpolation=cv2.INTER_AREA)
        return img

    def _decode(self, f):
        if self.cap is None:
            self._open()
        if f < self.pos or f > self.pos + 30:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, f)
            self.pos = f
            self.stats["seeks"] += 1
        while self.pos < f:
            if self.raw:
                self.cap.grab()            # a packet read, no decode
                self.pos += 1
            else:
                if not self.cap.grab():
                    return None
                self.pos += 1
        return self._read()

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


# ------------------------------------------------------------------------------------------
# Comparing patches
# ------------------------------------------------------------------------------------------

FLAT_STD = 3.0          # grey levels: a patch this even has no shape to compare


def ncc(a, b):
    """Normalised cross-correlation of two same-size patches (-1 .. 1); 0 if either is flat."""
    a = a.ravel().astype(np.float32)
    b = b.ravel().astype(np.float32)
    n = a.size
    sa, sb = float(a.sum()), float(b.sum())
    va = float(np.dot(a, a)) - sa * sa / n
    vb = float(np.dot(b, b)) - sb * sb / n
    if va < 1e-3 * n or vb < 1e-3 * n:
        return 0.0
    return max(-1.0, min(1.0, (float(np.dot(a, b)) - sa * sb / n) / math.sqrt(va * vb)))


def patch_std(a):
    return float(cv2.meanStdDev(a)[1][0, 0])


def similarity(a, b):
    """How alike two patches look (0 .. 1), for "is this the same thing as before". Shape
    (correlation) and contrast both count: a patch with the same outline but much stronger contrast
    is something else (a dark robot over a faint stain). Two flat patches are alike when their
    brightness is."""
    if a is None or b is None or a.shape != b.shape:
        return 0.0
    sa, sb = patch_std(a), patch_std(b)
    if sa < FLAT_STD and sb < FLAT_STD:
        return max(0.0, 1.0 - abs(float(a.mean()) - float(b.mean())) / 25.0)
    if sa < FLAT_STD or sb < FLAT_STD:
        return 0.0
    c = max(0.0, ncc(a, b))
    contrast = min(sa, sb) / max(sa, sb)
    return c * math.sqrt(contrast)


class Looker:
    """Patches and searches around objects, in full-size picture coordinates."""

    def __init__(self, frames, robot_px, patch_size=1.4):
        self.frames = frames
        self.s = frames.scale
        self.r = float(robot_px)
        # half the patch side, in small-picture pixels (patch: patch_size robot sizes across)
        self.hp = max(4, int(round(0.5 * patch_size * self.r * self.s)))

    def _centre(self, x, y):
        return int(round(x * self.s)), int(round(y * self.s))

    def patch(self, f, x, y):
        cx, cy = self._centre(x, y)
        hp = self.hp
        if cx < -hp or cy < -hp or cx > self.frames.w + hp or cy > self.frames.h + hp:
            return None
        return self.frames.window(f, cx - hp, cy - hp, cx + hp + 1, cy + hp + 1)

    def same_place(self, f, x, y, tpl):
        """Correlation of the patch at exactly (x, y) with tpl (-1 .. 1), or None."""
        p = self.patch(f, x, y)
        if p is None or tpl is None or p.shape != tpl.shape:
            return None
        return ncc(p, tpl)

    def search(self, f, x, y, radius_px, tpl, min_sep=None):
        """Look for tpl within radius_px (full-size px) of (x, y) on frame f.
        Returns dict(x, y, score, second, second_dist, ambiguous) or None. second is the best score
        at least min_sep (default half a robot) away from the best: two good places = ambiguous."""
        if tpl is None:
            return None
        cx, cy = self._centre(x, y)
        rad = max(1, int(math.ceil(radius_px * self.s)))
        hp = self.hp
        win = self.frames.window(f, cx - hp - rad, cy - hp - rad, cx + hp + rad + 1, cy + hp + rad + 1)
        if win is None or win.shape[0] < tpl.shape[0] or win.shape[1] < tpl.shape[1]:
            return None
        if patch_std(tpl) < FLAT_STD:
            return None
        res = cv2.matchTemplate(win, tpl, cv2.TM_CCOEFF_NORMED)
        if not np.isfinite(res).all():
            res = np.nan_to_num(res, nan=-1.0, posinf=-1.0, neginf=-1.0)
        _, best, _, (bx, by) = cv2.minMaxLoc(res)
        # sub-pixel: a parabola through the peak and its neighbours
        dx = dy = 0.0
        if 0 < bx < res.shape[1] - 1:
            l, c, r = res[by, bx - 1], res[by, bx], res[by, bx + 1]
            d = l - 2 * c + r
            dx = 0.5 * (l - r) / d if abs(d) > 1e-6 else 0.0
        if 0 < by < res.shape[0] - 1:
            u, c, d_ = res[by - 1, bx], res[by, bx], res[by + 1, bx]
            d = u - 2 * c + d_
            dy = 0.5 * (u - d_) / d if abs(d) > 1e-6 else 0.0
        dx, dy = max(-0.5, min(0.5, dx)), max(-0.5, min(0.5, dy))
        ox = (bx + dx - rad) / self.s
        oy = (by + dy - rad) / self.s
        # second best place, away from the best
        sep = int(math.ceil((min_sep if min_sep is not None else 0.5 * self.r) * self.s))
        masked = res.copy()
        masked[max(0, by - sep):by + sep + 1, max(0, bx - sep):bx + sep + 1] = -1.0
        _, second, _, (sx, sy) = cv2.minMaxLoc(masked)
        second_dist = math.hypot(sx - bx, sy - by) / self.s if second > -1.0 else None
        # the offset is from where the patch would sit if the object hadn't moved: no rounding drift
        return dict(x=x + ox, y=y + oy,
                    score=float(best), second=float(second), second_dist=second_dist,
                    ambiguous=bool(second > -1.0 and best - second < 0.05 and best > 0.0))
