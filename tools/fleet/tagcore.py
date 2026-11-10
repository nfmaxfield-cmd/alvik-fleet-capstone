#!/usr/bin/env python3
"""AprilTags on the robots' roofs (conference-table build, Nick 2026-11-02): finding them in a picture.

One detector for every caller (the sizing tests, the camera check-out, the tag bridge, the facing check by tags):

  family     which tag family (TAG_FAMILY, chosen in docs/build-reports/e02-nov02-conference-table.md from the sizing
             tests; any of FAMILIES works)
  detector   OpenCV's aruco module when it has the family (OpenCV 4.6 as Raspberry Pi OS ships it: the old
             detectMarkers call; 4.7 and later: ArucoDetector), else pupil-apriltags (pip install pupil-apriltags)
  windows    a robot's tag is looked for in a window round where it was (and where its own pose says it went), the
             window enlarged UPSCALE times first: small tags read far better enlarged (the sizing tests), and a window
             costs a fraction of the whole picture. The whole picture is searched only to find a tag that was lost,
             every FULL_EVERY pictures at most.

A detection: tag id, its 4 corners in pixels (the printed tag's top-left, top-right, bottom-right, bottom-left: the
"top" edge is the one the sheet marks FRONT, so the robot's nose), its centre, its size in pixels and a read score.

    python3 tools/fleet/tagcore.py --selftest      # draws tags, finds them (each family the OpenCV here has)

Changelog: 2026-11-10e14 (school 17:46, a slow detector in dim light): WindowTracker.speed_px_s, each window's speed
allowance from the tag's own measured speed, a miss searched again with the full allowance (MIRLT_TAG_ROI=max: as before). No version string before.
"""
import math
import os
import sys
import time

import numpy as np

VERSION = "2026-11-10e14"
HERE = os.path.dirname(os.path.abspath(__file__))

try:
    import cv2
except ImportError:      # noqa: E402  (pupil-apriltags alone still works)
    cv2 = None

# family name -> (OpenCV dictionary constant name, cells across the black square including its 1-cell border,
#                 pupil-apriltags name or None)
FAMILIES = {
    "tag16h5": ("DICT_APRILTAG_16h5", 6, "tag16h5"),
    "tag25h9": ("DICT_APRILTAG_25h9", 7, "tag25h9"),
    "tag36h11": ("DICT_APRILTAG_36h11", 8, "tag36h11"),
    "aruco4x4": ("DICT_4X4_50", 6, None),
}
TAG_FAMILY = "tag25h9"    # CHOSEN by the sizing tests (report, section 2): at 8 cm and >= 3.8 px/cm (1280 x 720, two
                          # cameras over the 396 cm table) it read in 100 % of synthetic pictures with blur and noise;
                          # tag16h5 reads at fewer pixels but its 5-bit spacing gives false reads on clutter (the room
                          # has other tags and printed sheets), tag36h11 needs 15 % more pixels per cell. DOE: yes,
                          # tag16h5 / tag25h9 / tag36h11 printed at the same size on the real table and cameras: read
                          # rate at the farthest point and false reads over 10 minutes of the empty table
QUIET_CELLS = 0.5         # white round the black square, in cells, at the least (the roof's white margin). The tests
                          # read with 0.5; the AprilTag papers ask 1 cell. No DOE: printed as large as the roof allows
UPSCALE = 2.0             # a window is enlarged this much before it is searched. Chosen from the sizing tests (2 read
                          # tags of 1.6 px per cell the 1x search missed; 3 was no better and 2.2x slower). DOE: 1.5 /
                          # 2 / 3 on real pictures against read rate and ms a window
WINDOW_TAGS = 3.0         # a window is this many tag-widths across, plus the robot's possible travel since its last
                          # read. No DOE: geometry (the tag anywhere inside, with a quiet zone round it)
FULL_EVERY = 5            # a lost tag: the whole picture searched at most every this many pictures. DOE: 1 / 5 / 15
                          # against the time to find a tag again and the Pi's CPU
FULL_UPSCALE_MAX_PX = 1400  # the whole-picture search enlarges a picture only up to this many pixels across (cost)
EDGE_REFINE = True        # (e07, 2026-11-06) each found square's four corners moved to where its four EDGES cross, each
                          # edge found where the grey level is half-way between the black border and the white margin
                          # beside it (refine_edges). A blur moves a square's corners inwards (the detector's corners read
                          # 8 cm tags 2-8 % small in the simulator, so a table measured by the tags alone came out large);
                          # the half-way crossing of a straight edge does not move with a symmetric blur (a drawn square:
                          # 0.2 % off). Switch: False = the detector's own corners, as before e07
EDGE_PROFILE_CELLS = 0.6  # how far each side of an edge its grey levels are read, in tag cells. Chosen: the 8 cm tag's
                          # white margin on the roof is 0.66 cells (9.5 cm roof), so the white level is read on the roof,
                          # not on the robot's body beyond it. DOE note: 0.4 / 0.6 / 0.9 against the read-size ratio on
                          # printed tags of known size on the real camera
EDGE_MIN_CONTRAST = 15.0  # grey levels between black and white below this: that edge is not used (no refinement)


def refine_edges(grey, corners, cells=7):
    """(e07) The square's corners from its four edges: along the middle 60 % of each side, the grey profile across the
    edge (EDGE_PROFILE_CELLS each way) gives the point half-way between its darkest (black border) and lightest (white
    margin) levels; a line through those points per side; corners where the lines cross. corners: 4x2 (any order
    round the square). Returns 4x2 or None (an edge without contrast, or lines that do not cross sensibly)."""
    g = grey.astype(np.float32) if grey.dtype != np.float32 else grey
    c = np.asarray(corners, float).reshape(4, 2)
    H, W = g.shape[:2]
    for _ in range(2):
        ctr = c.mean(0)
        lines = []
        for k in range(4):
            a, b = c[k], c[(k + 1) % 4]
            L = float(np.linalg.norm(b - a))
            if L < 6.0:
                return None
            t = (b - a) / L
            n = np.array([-t[1], t[0]])
            if np.dot(ctr - (a + b) / 2.0, n) > 0:
                n = -n                                   # n points out of the square
            w = max(1.5, EDGE_PROFILE_CELLS * L / cells)
            o = np.linspace(-w, w, 25)
            pts = []
            for s in np.linspace(0.2, 0.8, 9):
                p = a + s * (b - a)
                xs, ys = (p[0] + o * n[0]).astype(np.float32), (p[1] + o * n[1]).astype(np.float32)
                if xs.min() < 0 or ys.min() < 0 or xs.max() > W - 1 or ys.max() > H - 1:
                    continue
                v = cv2.remap(g, xs.reshape(-1, 1), ys.reshape(-1, 1), cv2.INTER_LINEAR).ravel()
                dark, light = np.sort(v[:12])[:3].mean(), np.sort(v[13:])[-3:].mean()
                if light - dark < EDGE_MIN_CONTRAST:
                    continue
                m = 0.5 * (dark + light)
                j = np.where((v[:-1] < m) & (v[1:] >= m))[0]
                if len(j) == 0:
                    continue
                j = j[np.argmin(np.abs(j - 12))]
                f = (m - v[j]) / max(1e-6, v[j + 1] - v[j])
                pts.append(p + (o[j] + f * (o[1] - o[0])) * n)
            if len(pts) < 4:
                return None
            P = np.array(pts)
            mu = P.mean(0)
            lines.append((mu, np.linalg.svd(P - mu)[2][0]))
        new = []
        for k in range(4):
            (p1, d1), (p2, d2) = lines[(k - 1) % 4], lines[k]
            A = np.array([d1, -d2]).T
            if abs(np.linalg.det(A)) < 0.2:
                return None
            st = np.linalg.solve(A, p2 - p1)
            new.append(p1 + st[0] * d1)
        new = np.array(new)
        if np.max(np.linalg.norm(new - c, axis=1)) > 0.25 * np.mean(
                [np.linalg.norm(c[k] - c[(k + 1) % 4]) for k in range(4)]):
            return None                                  # moved too far: not this square's edges
        c = new
    return c


def opencv_has(family):
    """True when this OpenCV's aruco module has the family."""
    if cv2 is None or not hasattr(cv2, "aruco"):
        return False
    return hasattr(cv2.aruco, FAMILIES[family][0])


def pupil_has(family):
    if FAMILIES[family][2] is None:
        return False
    try:
        import pupil_apriltags  # noqa: F401
        return True
    except ImportError:
        return False


def backend_for(family, prefer=None):
    """'opencv' or 'pupil' (or None when neither has the family)."""
    order = [prefer] if prefer else []
    order += [b for b in ("opencv", "pupil") if b not in order]
    for b in order:
        if b == "opencv" and opencv_has(family):
            return "opencv"
        if b == "pupil" and pupil_has(family):
            return "pupil"
    return None


def _dictionary(family):
    const = getattr(cv2.aruco, FAMILIES[family][0])
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(const)
    return cv2.aruco.Dictionary_get(const)


def tag_image(family, tag_id, px_per_cell=40, quiet_cells=1.0):
    """The printed tag as a grey picture (black square with its data, white round it quiet_cells wide)."""
    n = FAMILIES[family][1]
    side = n * px_per_cell
    d = _dictionary(family)
    if hasattr(cv2.aruco, "generateImageMarker"):
        img = cv2.aruco.generateImageMarker(d, int(tag_id), side, borderBits=1)
    else:
        img = cv2.aruco.drawMarker(d, int(tag_id), side, borderBits=1)
    q = int(round(quiet_cells * px_per_cell))
    return cv2.copyMakeBorder(img, q, q, q, q, cv2.BORDER_CONSTANT, value=255)


def _params(family, small=True):
    if hasattr(cv2.aruco, "DetectorParameters") and not hasattr(cv2.aruco, "DetectorParameters_create"):
        p = cv2.aruco.DetectorParameters()
    else:
        p = cv2.aruco.DetectorParameters_create()
    # small tags (10-40 px across): OpenCV's defaults drop anything under 3 % of the picture's size
    p.minMarkerPerimeterRate = 0.01 if small else 0.03
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 23
    p.adaptiveThreshWinSizeStep = 10
    p.polygonalApproxAccuracyRate = 0.06      # chosen: 0.03 (default) refuses blurred small squares. DOE note in the report
    p.minCornerDistanceRate = 0.03
    # the roof's white margin on a darker table is itself a black-edged square round the tag; OpenCV 4.6 (Raspberry Pi
    # OS) drops one of two squares closer than minMarkerDistanceRate (0.05 of the perimeter) and kept the roof's, which
    # does not read: 3 % of 5 px/cm tags read (sizing tests under 4.6, 2 Nov). 0.02 keeps both; the roof's square
    # fails its own read and the tag's reads. Duplicates of one id are merged in detect()
    p.minMarkerDistanceRate = 0.02
    p.minDistanceToBorder = 1
    p.cornerRefinementMethod = getattr(cv2.aruco, "CORNER_REFINE_SUBPIX", 1)
    p.cornerRefinementWinSize = 2
    p.perspectiveRemovePixelPerCell = 6
    p.perspectiveRemoveIgnoredMarginPerCell = 0.2
    if family == "tag16h5":
        p.errorCorrectionRate = 0.0           # 16h5 has almost no room to correct: a corrected read is usually clutter
    return p


class Detection:
    __slots__ = ("id", "corners", "cam", "t", "score")

    def __init__(self, tag_id, corners, cam=None, t=None, score=1.0):
        self.id = int(tag_id)
        self.corners = np.asarray(corners, float).reshape(4, 2)
        self.cam, self.t, self.score = cam, t, float(score)

    @property
    def center(self):
        """The tag's middle in the picture: where its diagonals cross (exact under perspective, unlike the mean)."""
        p = self.corners
        a1, a2, b1, b2 = p[0], p[2], p[1], p[3]
        d1, d2 = a2 - a1, b2 - b1
        den = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(den) < 1e-9:
            return p.mean(0)
        s = ((b1[0] - a1[0]) * d2[1] - (b1[1] - a1[1]) * d2[0]) / den
        return a1 + s * d1

    @property
    def size_px(self):
        p = self.corners
        return float(np.mean([np.linalg.norm(p[k] - p[(k + 1) % 4]) for k in range(4)]))

    def to_dict(self):
        return {"id": self.id, "corners": np.round(self.corners, 2).tolist(), "cam": self.cam, "t": self.t,
                "score": round(self.score, 3)}


class TagDetector:
    """Finds tags of one family in grey pictures. ids: only these are reported (others are clutter: the room's other
    tags, a spare on the table). backend: 'opencv' or 'pupil' (default: what is available, OpenCV first)."""

    def __init__(self, family=TAG_FAMILY, ids=None, backend=None, upscale=UPSCALE, threads=1, refine=None):
        self.family = family
        self.refine = EDGE_REFINE if refine is None else bool(refine)    # (e07) corners from the edges
        self.cells = FAMILIES[family][1]
        self.refined = self.refine_failed = 0
        self.ids = None if ids is None else set(int(i) for i in ids)
        self.backend = backend or backend_for(family)
        if self.backend is None:
            raise RuntimeError("No AprilTag detector for %s here: OpenCV %s has no %s, and pupil-apriltags is not "
                               "installed (pip install pupil-apriltags)." % (
                                   family, getattr(cv2, "__version__", "missing"), FAMILIES[family][0]))
        self.upscale = float(upscale)
        if self.backend == "opencv":
            self._dict = _dictionary(family)
            self._p = _params(family)
            if hasattr(cv2.aruco, "ArucoDetector"):
                det = cv2.aruco.ArucoDetector(self._dict, self._p)
                self._raw = lambda g: det.detectMarkers(g)[:2]
            else:
                self._raw = lambda g: cv2.aruco.detectMarkers(g, self._dict, parameters=self._p)[:2]
        else:
            import pupil_apriltags as PA
            self._pa = PA.Detector(families=FAMILIES[family][2], nthreads=threads, quad_decimate=1.0,
                                   refine_edges=1, decode_sharpening=0.25)
            self._raw = self._pupil_raw
        self.calls, self.ms = 0, 0.0

    def _pupil_raw(self, g):
        out_c, out_i = [], []
        for d in self._pa.detect(g):
            if d.hamming > 1 or d.decision_margin < 15:
                continue
            c = np.asarray(d.corners, float)
            # pupil-apriltags (tested against OpenCV on the same drawn tags) gives top-right, top-left, bottom-left,
            # bottom-right, with (0, 0) at the first pixel's corner; OpenCV's order is top-left, top-right, bottom-right,
            # bottom-left with (0, 0) at the first pixel's middle
            c = c[[1, 0, 3, 2]] - 0.5
            out_c.append(c.reshape(1, 4, 2).astype(np.float32))
            out_i.append([d.tag_id])
        return out_c, (np.array(out_i) if out_i else None)

    def detect(self, grey, cam=None, t=None, scale=1.0, offset=(0.0, 0.0)):
        """All tags (of our ids) in a grey picture. scale: the picture was enlarged this much from the original (the
        corners are given back in the original's pixels, offset added)."""
        if grey.ndim == 3:
            grey = cv2.cvtColor(grey, cv2.COLOR_BGR2GRAY)
        t0 = time.perf_counter()
        corners, ids = self._raw(grey)
        self.ms += (time.perf_counter() - t0) * 1000.0
        self.calls += 1
        out = []
        if ids is None:
            return out
        seen = {}
        gf = None
        for c, i in zip(corners, np.asarray(ids).ravel()):
            if self.ids is not None and int(i) not in self.ids:
                continue
            q = np.asarray(c, float).reshape(4, 2)
            if self.refine:
                if gf is None:
                    gf = grey.astype(np.float32)
                r = refine_edges(gf, q, self.cells)
                if r is not None:
                    q = r
                    self.refined += 1
                else:
                    self.refine_failed += 1
            p = q / scale + np.asarray(offset, float)
            d = Detection(int(i), p, cam=cam, t=t)
            prev = seen.get(d.id)
            if prev is not None and np.linalg.norm(prev.center - d.center) < prev.size_px:
                continue          # the same square found twice (two threshold windows): the first kept
            seen[d.id] = d
            out.append(d)
        return out

    def detect_window(self, grey, cx, cy, half_px, cam=None, t=None, upscale=None):
        """Tags in a window of grey round (cx, cy), half_px each way, enlarged `upscale` times first."""
        H, W = grey.shape[:2]
        x0, y0 = int(max(0, math.floor(cx - half_px))), int(max(0, math.floor(cy - half_px)))
        x1, y1 = int(min(W, math.ceil(cx + half_px))), int(min(H, math.ceil(cy + half_px)))
        if x1 - x0 < 8 or y1 - y0 < 8:
            return []
        sub = grey[y0:y1, x0:x1]
        k = self.upscale if upscale is None else float(upscale)
        if k != 1.0:
            sub = cv2.resize(sub, None, fx=k, fy=k, interpolation=cv2.INTER_CUBIC)
        return self.detect(sub, cam=cam, t=t, scale=k, offset=(x0, y0))

    def detect_full(self, grey, cam=None, t=None, upscale=None):
        """The whole picture, enlarged up to FULL_UPSCALE_MAX_PX across (finding a lost tag)."""
        H, W = grey.shape[:2]
        k = self.upscale if upscale is None else float(upscale)
        k = max(1.0, min(k, FULL_UPSCALE_MAX_PX / float(W)))
        img = grey if k == 1.0 else cv2.resize(grey, None, fx=k, fy=k, interpolation=cv2.INTER_CUBIC)
        return self.detect(img, cam=cam, t=t, scale=k)

    def ms_per_call(self):
        return self.ms / max(1, self.calls)


DOWNSCALE_RETRY = 0.5    # (2026-11-05) a wanted tag missing after the full picture: searched again at this size (0 = off).
                         # Chosen: half (the school frame that read). DOE note: 0.5 / 0.65 / 0.75 against far reads per
                         # minute on the school table in room light


ROI_RULE = os.environ.get("MIRLT_TAG_ROI", "measured")   # (e14) "measured" (default): each tag's window allows
                         # its own measured speed (speed_px_s), and a miss is searched again with the full allowance;
                         # "max": every window allows max_speed_px_s (as before). detect_bench.py, simulated school
                         # camera, 4 standing tags, a picture every 0.43 s: dim 373 -> 58 ms a picture, lit 39 -> 14,
                         # the same reads (report e14)
ROI_SPEED_X = 3.0        # (e14) the measured speed's margin (a robot that starts moving). CHOSEN; DOE row (e14)
ROI_MIN_PX_S = 40.0      # (e14) the least allowance (about 5 cm/s near the school camera). CHOSEN; DOE row (e14)
ROI_HIST = 6             # (e14) reads kept per tag for its speed. Not tuned


class WindowTracker:
    """Per camera: where each tag was last read, so the next picture is searched in small windows; the whole picture
    every FULL_EVERY pictures while any wanted tag is missing. predict(tag_id) -> (u, v, radius_px) may come from the
    robot's own pose (the bridge passes it)."""

    def __init__(self, det, ids, full_every=FULL_EVERY):
        self.det, self.ids, self.full_every = det, list(ids), full_every
        self.last = {}          # id -> (u, v, size_px, t)
        self.since_full = 10 ** 9
        self.windows = self.fulls = 0

    def speed_px_s(self, i, max_speed_px_s):
        """(e14) The speed allowance for tag i's window: max_speed_px_s, or (ROI_RULE "measured", MIRLT_TAG_ROI) the
        tag's own measured speed in the picture over its last reads x ROI_SPEED_X, at least ROI_MIN_PX_S: a standing
        tag's window stays small however slowly the pictures come (school 17:46: as fixes slowed, every window grew
        towards its cap and each picture took longer)."""
        if ROI_RULE != "measured":
            return max_speed_px_s
        h = getattr(self, "hist", {}).get(i) or []
        if len(h) < 3:
            return max_speed_px_s
        sp = [math.hypot(b[0] - a[0], b[1] - a[1]) / (b[2] - a[2]) for a, b in zip(h, h[1:]) if b[2] > a[2]]
        return min(max_speed_px_s, max(ROI_MIN_PX_S, ROI_SPEED_X * max(sp))) if sp else max_speed_px_s

    def step(self, grey, t, cam=None, predict=None, max_speed_px_s=300.0):
        found = {}
        for i in self.ids:
            guess = predict(i) if predict else None
            last = self.last.get(i)
            if guess is None and last is None:
                continue
            if guess is not None:
                u, v, r = guess
                size = last[2] if last else max(10.0, r / WINDOW_TAGS)
            else:
                u, v, size, t_last = last
                r = 0.0
            dt = 0.0 if last is None else max(0.0, t - last[3])
            sp = self.speed_px_s(i, max_speed_px_s)
            half = WINDOW_TAGS * size / 2.0 + min(sp * dt, 4 * size) + r * 0.25
            full_half = WINDOW_TAGS * size / 2.0 + min(max_speed_px_s * dt, 4 * size) + r * 0.25
            tries = [half, full_half] if full_half > half + 1.0 else [half]
            for h_ in tries:
                # (e14) a measured-speed window that missed: the same picture again with the full allowance (a robot
                # that has just started to move is never lost to the smaller window)
                self.windows += 1
                for d in self.det.detect_window(grey, u, v, h_, cam=cam, t=t):
                    if d.id == i:
                        found[i] = d
                        break
                if i in found:
                    break
                if h_ is not half or len(tries) == 1:
                    break
                self.roi_retries = getattr(self, "roi_retries", 0) + 1
        missing = [i for i in self.ids if i not in found]
        self.since_full += 1
        if missing and self.since_full >= self.full_every:
            self.since_full = 0
            self.fulls += 1
            for d in self.det.detect_full(grey, cam=cam, t=t):
                if d.id in missing and d.id not in found:
                    found[d.id] = d
            still = [i for i in missing if i not in found]
            if still and getattr(self, "downscale", DOWNSCALE_RETRY):
                # (2026-11-05, school) a tag not found at full size: looked for again in a downscaled picture (a
                # downscaled frame read the small far tag when the full frame did not: the threshold windows fit it)
                k = float(getattr(self, "downscale", DOWNSCALE_RETRY))
                small = cv2.resize(grey, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
                for d in self.det.detect(small, cam=cam, t=t, scale=k):
                    if d.id in still and d.id not in found:
                        found[d.id] = d
                        self.downscaled = getattr(self, "downscaled", 0) + 1
        for i, d in found.items():
            c = d.center
            self.last[i] = (float(c[0]), float(c[1]), d.size_px, t)
            hh = self.__dict__.setdefault("hist", {}).setdefault(i, [])
            hh.append((float(c[0]), float(c[1]), t))
            del hh[:-ROI_HIST]
        return found


# ------------------------------------------------------------------------------------------------- self-test
def _selftest():
    ok = True
    for fam in FAMILIES:
        be = backend_for(fam)
        if be is None:
            print("%s: no detector here (skipped)" % fam)
            continue
        det = TagDetector(fam, backend=be)
        img = np.full((480, 640), 200, np.uint8)
        for k, (x, y) in enumerate([(100, 100), (400, 300)]):
            t = tag_image(fam, k, px_per_cell=8, quiet_cells=1.0)
            h, w = t.shape
            img[y:y + h, x:x + w] = t
        found = {d.id: d for d in det.detect(img)}
        good = set(found) == {0, 1}
        n = FAMILIES[fam][1]
        c0 = found[0].center if 0 in found else None
        want = np.array([100 + 8 + n * 8 / 2.0 - 0.5, 100 + 8 + n * 8 / 2.0 - 0.5])
        err = float(np.linalg.norm(c0 - want)) if c0 is not None else float("nan")
        print("%s (%s): found %s, tag 0's middle %.2f px off" % (fam, be, sorted(found), err))
        ok = ok and good and err < 1.0
    print("self-test %s" % ("passed" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(_selftest())
    print(__doc__)
