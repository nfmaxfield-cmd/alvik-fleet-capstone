"""Shared pieces of the camera detector that need only numpy + OpenCV.

Everything here runs on the Raspberry Pi without PyTorch:

- reading labels folders (docs/data-formats.md, "Labels for the camera detector")
- splitting recordings into train / val / test (whole recordings, never frames)
- turning a camera frame into the network's input array
- decoding the network's output maps into detections
- matching detections to labels and scoring them (precision, recall, errors)
- the floor homography (image px <-> floor cm)

Conventions (same as docs/data-formats.md):
- image pixels: origin top-left, x right, y down
- heading_deg: 0 = image +x (right), counter-clockwise as seen on screen is
  positive (towards image -y). So a robot facing heading h points along the
  image vector (cos h, -sin h).
"""
import csv
import glob
import json
import math
import os
import random
import warnings

import cv2
import numpy as np

# The classes the detector knows, in the order of the heatmap channels.
# To add a class, append it here (or pass --classes to train.py); never reorder,
# because a trained model's channel 0 must stay "alvik".
DEFAULT_CLASSES = ["alvik", "hand", "unknown", "machine"]

# Rough size of each class in a 640x480 frame, in pixels. Used to size the
# training "blobs" (Gaussian targets) and the fixed boxes for the baseline.
DEFAULT_OBJ_SIZE_PX = {"alvik": 30, "hand": 70, "unknown": 30, "machine": 80}

# Classes that have a front, so the network reports their heading. Hands and
# unknown objects have none, so their heading is always None.
DEFAULT_HEADING_CLASSES = {"alvik", "machine"}

# Only these label statuses count as ground truth.
GOOD_STATUS = {"confirmed", "corrected", "auto"}   # auto = accepted by the labelling tool without a person looking

LABEL_COLUMNS = ["frame", "t_wall", "obj", "cls", "x_px", "y_px", "heading_deg",
                 "heading_known", "source", "status", "score"]


# --------------------------------------------------------------------------
# Headings and angles
# --------------------------------------------------------------------------

def wrap_deg(a):
    """Wrap an angle (or array of angles) to the range (-180, 180]."""
    a = (np.asarray(a, dtype=np.float64) + 180.0) % 360.0 - 180.0
    a = np.where(a <= -180.0, a + 360.0, a)
    return float(a) if a.ndim == 0 else a


def heading_to_vec(h_deg):
    """Heading (deg, CCW on screen) -> unit vector in image coordinates (dx, dy)."""
    r = math.radians(h_deg)
    return math.cos(r), -math.sin(r)


def vec_to_heading(dx, dy):
    """Image vector (dx, dy) -> heading in degrees, CCW on screen, in [0, 360)."""
    return math.degrees(math.atan2(-dy, dx)) % 360.0


def transform_objects(objects, M):
    """Apply a 2x3 affine matrix M (image px -> new image px) to object centres
    and headings. Headings are carried by moving a point a little ahead of the
    centre and measuring the new direction, so rotation, flips and stretching
    all come out right without special cases. Returns a new list."""
    out = []
    for o in objects:
        x, y = o["x"], o["y"]
        nx = M[0, 0] * x + M[0, 1] * y + M[0, 2]
        ny = M[1, 0] * x + M[1, 1] * y + M[1, 2]
        h = o.get("heading")
        if h is not None:
            dx, dy = heading_to_vec(h)
            tx = M[0, 0] * dx + M[0, 1] * dy      # direction: no translation
            ty = M[1, 0] * dx + M[1, 1] * dy
            h = vec_to_heading(tx, ty)
        out.append(dict(o, x=nx, y=ny, heading=h))
    return out


# --------------------------------------------------------------------------
# Reading labels folders
# --------------------------------------------------------------------------

def find_label_dirs(paths):
    """Accept any mix of: a labels folder, a recording folder that has labels/
    inside, or a parent folder holding many recordings. Returns a sorted list
    of labels folders (each containing labels.csv)."""
    if isinstance(paths, str):
        paths = [paths]
    found = set()
    for p in paths:
        p = os.path.abspath(os.path.expanduser(p))
        if os.path.isfile(os.path.join(p, "labels.csv")):
            found.add(p)
        elif os.path.isfile(os.path.join(p, "labels", "labels.csv")):
            found.add(os.path.join(p, "labels"))
        else:
            for f in glob.glob(os.path.join(p, "**", "labels.csv"), recursive=True):
                found.add(os.path.dirname(f))
    if not found:
        raise FileNotFoundError(f"No labels.csv found under {paths}")
    return sorted(found)


def recording_name(label_dir):
    """The recording a labels folder belongs to: the folder that contains
    labels/ (e.g. 20261020-141500_E1). Used as the unit of the split."""
    label_dir = os.path.abspath(label_dir)
    if os.path.basename(label_dir) == "labels":
        return os.path.basename(os.path.dirname(label_dir))
    return os.path.basename(label_dir)


def _float_or_none(s):
    s = (s or "").strip()
    return float(s) if s else None


def read_labels(label_dir, classes=None, verbose=True):
    """Read one labels folder and return a list of samples (one per usable frame):

        dict(recording, label_dir, frame, image, objects=[dict(cls, x, y, heading, obj)])

    Rules (stated assumptions; see README):
    - rows with status confirmed / corrected are ground truth
    - rejected rows are dropped (they were wrong automatic guesses)
    - a frame is used only if every one of its non-rejected rows is
      confirmed / corrected. A frame that still has a pending row is
      half-reviewed, and its missing objects would wrongly teach the model
      "nothing here", so the whole frame is skipped.
    - a cls `none` row marks a confirmed empty frame (no objects)
    - heading is used only when heading_known == 1
    - classes not in `classes` are skipped with a warning
    """
    classes = classes or DEFAULT_CLASSES
    path = os.path.join(label_dir, "labels.csv")
    rows_by_frame = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                fr = int(row["frame"])
            except (KeyError, ValueError):
                continue
            rows_by_frame.setdefault(fr, []).append(row)

    samples, skipped_pending, missing_img, unknown_cls = [], 0, 0, set()
    rec = recording_name(label_dir)
    for fr in sorted(rows_by_frame):
        rows = [r for r in rows_by_frame[fr] if (r.get("status") or "").strip() != "rejected"]
        if not rows:
            continue
        if any((r.get("status") or "").strip() not in GOOD_STATUS for r in rows):
            skipped_pending += 1
            continue
        objects = []
        for r in rows:
            cls = (r.get("cls") or "").strip()
            if cls == "none":
                continue
            if cls not in classes:
                unknown_cls.add(cls)
                continue
            x, y = _float_or_none(r.get("x_px")), _float_or_none(r.get("y_px"))
            if x is None or y is None:
                continue
            known = (r.get("heading_known") or "").strip() in ("1", "1.0", "true", "True")
            h = _float_or_none(r.get("heading_deg")) if known else None
            objects.append(dict(cls=cls, x=x, y=y, heading=h, obj=r.get("obj")))
        img = os.path.join(label_dir, "frames", f"{fr:06d}.jpg")
        if not os.path.isfile(img):
            missing_img += 1
            continue
        auto = any((r.get("status") or "").strip() == "auto" for r in rows)
        samples.append(dict(recording=rec, label_dir=label_dir, frame=fr, image=img, objects=objects, auto=auto))

    if verbose and (skipped_pending or missing_img or unknown_cls):
        msg = f"{rec}: {len(samples)} frames used"
        if skipped_pending:
            msg += f", {skipped_pending} skipped (still pending)"
        if missing_img:
            msg += f", {missing_img} skipped (image missing)"
        if unknown_cls:
            msg += f", ignored unknown classes {sorted(unknown_cls)}"
        print(msg)
    return samples


def human_checked(samples):
    """Only frames a person looked at (drops labelling-tool auto-accepted
    frames). Score on these: auto frames are the easy, high-confidence ones,
    and may even be this model's own guesses."""
    return [s for s in samples if not s.get("auto")]


def read_many(paths, classes=None, verbose=True):
    """Read every labels folder found under `paths`. Returns a list of samples."""
    out = []
    for d in find_label_dirs(paths):
        out.extend(read_labels(d, classes, verbose))
    return out


# --------------------------------------------------------------------------
# Splitting by recording
# --------------------------------------------------------------------------

def split_recordings(counts, val_frac=0.15, test_frac=0.15, seed=0, val=None, test=None):
    """Assign whole recordings to train / val / test. Never splits a recording,
    because neighbouring frames are nearly identical and would leak.

    counts: dict recording -> number of frames
    val / test: optional explicit lists of recording names (these win)
    Otherwise recordings are shuffled with `seed` and handed out until each
    of val and test holds roughly its share of frames. With 3+ recordings,
    val and test each get at least one.
    Returns dict recording -> "train" | "val" | "test".
    """
    names = sorted(counts)
    val, test = list(val or []), list(test or [])
    for n in val + test:
        if n not in counts:
            raise ValueError(f"Recording {n!r} named in the split is not in the data. Have: {names}")
    assign = {n: "val" for n in val}
    assign.update({n: "test" for n in test})
    rest = [n for n in names if n not in assign]
    if not val and not test:
        if len(rest) < 3:
            raise ValueError(
                f"Only {len(rest)} recording(s) found ({rest}). Splitting by recording needs at least 3 "
                "(train, val, test). Record more sessions, or name the split with --val/--test.")
        rng = random.Random(seed)
        rng.shuffle(rest)
        total = sum(counts.values())
        for part, frac, reserve in (("test", test_frac, 2), ("val", val_frac, 1)):
            # reserve: recordings that must stay behind for the parts still to fill
            if frac <= 0:
                continue
            got = 0
            while len(rest) > reserve and (got == 0 or got + counts[rest[0]] / 2 <= frac * total):
                n = rest.pop(0)
                assign[n] = part
                got += counts[n]
    for n in rest:
        assign[n] = "train"
    return assign


def make_split(samples, val_frac=0.15, test_frac=0.15, seed=0, val=None, test=None):
    counts = {}
    for s in samples:
        counts[s["recording"]] = counts.get(s["recording"], 0) + 1
    return split_recordings(counts, val_frac, test_frac, seed, val, test)


def save_split(assign, path):
    with open(path, "w") as f:
        json.dump({"by_recording": assign}, f, indent=2, sort_keys=True)


def load_split(path):
    with open(path) as f:
        return json.load(f)["by_recording"]


def select(samples, assign, part):
    """Samples whose recording is in split `part` ("train" / "val" / "test")."""
    return [s for s in samples if assign.get(s["recording"]) == part]


# --------------------------------------------------------------------------
# Network input and output (shared by the PyTorch and ONNX Runtime paths)
# --------------------------------------------------------------------------

def preprocess(frame_bgr, in_w, in_h, in_ch):
    """Camera frame (H x W x 3, BGR, uint8) -> network input (1 x C x in_h x in_w, float32).
    Returns the array and the (sx, sy) scale from frame px to input px."""
    h, w = frame_bgr.shape[:2]
    if (w, h) != (in_w, in_h):
        img = cv2.resize(frame_bgr, (in_w, in_h), interpolation=cv2.INTER_AREA)
    else:
        img = frame_bgr
    img = to_channels(img, in_ch)
    x = normalize(img)[None]
    return x, (in_w / w, in_h / h)


def resize_matrix(sx, sy):
    """2x3 matrix taking frame px to px of the frame resized by (sx, sy).
    Pixel centres are at whole numbers (as in OpenCV), hence the 0.5 terms."""
    return np.array([[sx, 0, 0.5 * sx - 0.5], [0, sy, 0.5 * sy - 0.5]], np.float64)


def to_channels(img_bgr, in_ch):
    """BGR uint8 image -> H x W x C (C = 1 grey or 3 RGB), still uint8."""
    if in_ch == 1:
        return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)[:, :, None]
    return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)


def normalize(img_hwc_u8):
    """uint8 H x W x C -> float32 C x H x W, roughly centred on 0 with spread 1."""
    x = img_hwc_u8.astype(np.float32) * (1.0 / 255.0)
    x = (x - 0.5) * 4.0
    return np.ascontiguousarray(x.transpose(2, 0, 1))


def _peaks(hm, thresh, kernel):
    """Cells that are the largest in their neighbourhood and above thresh
    (the "maxpool NMS" trick: no box overlap maths needed)."""
    if hm.max() < thresh:
        return [], []
    peaks = (hm == cv2.dilate(hm, kernel)) & (hm >= thresh)
    return np.nonzero(peaks)


def _cell_to_frame(cx, cy, stride, sx, sy):
    """Output-cell coordinates -> input px -> frame px (undoing cv2.resize's half-pixel shift)."""
    return float((cx * stride + 0.5) / sx - 0.5), float((cy * stride + 0.5) / sy - 0.5)


def decode(heat, offset, heading, cfg, scale_xy, score_thresh=0.3, max_objects=50, front_thresh=0.15):
    """Network output maps -> detections in frame pixels.

    heat:    C x h x w, probabilities (already through the sigmoid)
    offset:  2 x h x w, sub-cell position (dx, dy) in output cells
    heading: front mode: 3 x h x w = front-point heatmap + its offset;
             sincos mode: 2 x h x w = (sin, cos) of the heading in the input frame
    cfg:     the model config (classes, stride, heading_mode, ...)
    A detection is a heatmap peak above score_thresh. Heading: in front mode,
    the direction from the centre to the best front-point peak found at about
    the expected distance; None if no front point was found (so an unsure
    heading is reported as unknown rather than guessed).
    """
    classes, stride = cfg["classes"], cfg["stride"]
    heading_classes = set(cfg.get("heading_classes", DEFAULT_HEADING_CLASSES))
    mode = cfg.get("heading_mode", "sincos" if heading.shape[0] == 2 else "front")
    sizes = cfg.get("obj_size_px", DEFAULT_OBJ_SIZE_PX)
    sx, sy = scale_xy
    kernel = np.ones((3, 3), np.uint8)

    fronts = []
    if mode == "front":
        fy, fx = _peaks(heading[0], front_thresh, kernel)
        for yy, xx in zip(fy, fx):
            fronts.append((*_cell_to_frame(xx + float(heading[1, yy, xx]), yy + float(heading[2, yy, xx]),
                                           stride, sx, sy), float(heading[0, yy, xx])))

    dets = []
    for c, name in enumerate(classes):
        ys, xs = _peaks(heat[c], score_thresh, kernel)
        for yy, xx in zip(ys, xs):
            px, py = _cell_to_frame(xx + float(offset[0, yy, xx]), yy + float(offset[1, yy, xx]), stride, sx, sy)
            h = None
            if name in heading_classes:
                if mode == "front":
                    d_exp = cfg.get("front_frac", 0.35) * sizes.get(name, 30)    # frame px
                    best = 0.0
                    for fx_, fy_, fs in fronts:
                        d = math.hypot(fx_ - px, fy_ - py)
                        if 0.4 * d_exp <= d <= 1.8 * d_exp and fs > best:
                            best, h = fs, vec_to_heading(fx_ - px, fy_ - py)
                else:
                    s, co = float(heading[0, yy, xx]), float(heading[1, yy, xx])
                    if s * s + co * co > 0.04:
                        # predicted in the (possibly stretched) input; undo the stretch
                        h = vec_to_heading(co / sx, -s / sy)
            dets.append(dict(cls=name, x=px, y=py, heading_deg=h, score=float(heat[c, yy, xx])))
    dets.sort(key=lambda d: -d["score"])
    return dets[:max_objects]


# --------------------------------------------------------------------------
# Matching detections to labels, and scores
# --------------------------------------------------------------------------

def match_frame(preds, gts, radius):
    """Greedy matching within one frame and one class: highest-score prediction
    first, to the nearest unmatched label within `radius` px.
    Returns list of (pred_index, gt_index, distance_px)."""
    order = sorted(range(len(preds)), key=lambda i: -preds[i].get("score", 1.0))
    used, pairs = set(), []
    for i in order:
        best, bd = None, radius
        for j, g in enumerate(gts):
            if j in used:
                continue
            d = math.hypot(preds[i]["x"] - g["x"], preds[i]["y"] - g["y"])
            if d <= bd:
                best, bd = j, d
        if best is not None:
            used.add(best)
            pairs.append((i, best, bd))
    return pairs


class Scorer:
    """Collects matches over many frames, then reports per-class numbers.

    add(frame_key, preds, gts): preds use keys cls,x,y,heading_deg,score;
    gts use cls,x,y,heading (None if unknown). Optional to_cm(x, y) maps image
    px to floor cm (from the homography) for cm errors.
    """

    def __init__(self, classes, radius=8.0, to_cm=None):
        """radius: match radius in px, one number, or a dict {class: px} with
        an optional "default" (big objects like hands deserve a bigger radius)."""
        self.classes = list(classes)
        self.radius = radius if isinstance(radius, dict) else {"default": float(radius)}
        self.to_cm = to_cm
        self.s = {c: dict(tp=0, fp=0, fn=0, err=[], err_cm=[], herr=[], hgt=0) for c in self.classes}

    def add(self, preds, gts, to_cm=None):
        to_cm = to_cm or self.to_cm
        for c in self.classes:
            p = [d for d in preds if d["cls"] == c]
            g = [d for d in gts if d["cls"] == c]
            pairs = match_frame(p, g, self.radius.get(c, self.radius.get("default", 8.0)))
            st = self.s[c]
            st["tp"] += len(pairs)
            st["fp"] += len(p) - len(pairs)
            st["fn"] += len(g) - len(pairs)
            for i, j, d in pairs:
                st["err"].append(d)
                if to_cm is not None:
                    a = to_cm(p[i]["x"], p[i]["y"])
                    b = to_cm(g[j]["x"], g[j]["y"])
                    st["err_cm"].append(math.hypot(a[0] - b[0], a[1] - b[1]))
                hg, hp = g[j].get("heading"), p[i].get("heading_deg")
                st["hgt"] += hg is not None
                if hg is not None and hp is not None:
                    st["herr"].append(abs(wrap_deg(hp - hg)))

    def rows(self):
        """One dict per class with data, plus an 'all' row (summed counts)."""
        out = []
        keys = [c for c in self.classes if (self.s[c]["tp"] + self.s[c]["fp"] + self.s[c]["fn"]) > 0]
        merged = dict(tp=0, fp=0, fn=0, err=[], err_cm=[], herr=[], hgt=0)
        for c in keys:
            for k in merged:
                merged[k] += self.s[c][k]
        for name, st in [(c, self.s[c]) for c in keys] + ([("all", merged)] if len(keys) > 1 else []):
            tp, fp, fn = st["tp"], st["fp"], st["fn"]
            prec = tp / (tp + fp) if tp + fp else float("nan")
            rec = tp / (tp + fn) if tp + fn else float("nan")
            f1 = 2 * prec * rec / (prec + rec) if tp else 0.0
            e, ecm, he = np.array(st["err"]), np.array(st["err_cm"]), np.array(st["herr"])
            out.append(dict(
                cls=name, n_labels=tp + fn, n_preds=tp + fp, tp=tp, fp=fp, fn=fn,
                precision=prec, recall=rec, f1=f1,
                centre_err_px_mean=e.mean() if e.size else float("nan"),
                centre_err_px_p95=np.percentile(e, 95) if e.size else float("nan"),
                centre_err_cm_mean=ecm.mean() if ecm.size else float("nan"),
                centre_err_cm_p95=np.percentile(ecm, 95) if ecm.size else float("nan"),
                n_heading=int(he.size),
                # of the found objects whose label has a heading: share given a heading by the detector
                heading_given_pct=100.0 * he.size / st["hgt"] if st["hgt"] else float("nan"),
                heading_err_deg_mean=he.mean() if he.size else float("nan"),
                heading_err_deg_median=np.median(he) if he.size else float("nan"),
                heading_err_deg_p95=np.percentile(he, 95) if he.size else float("nan"),
                heading_flipped_pct=100.0 * (he > 135).mean() if he.size else float("nan"),
            ))
        return out


def parse_radius(text):
    """'8' -> 8.0; '8,hand=25' -> {'default': 8.0, 'hand': 25.0}"""
    out = {}
    for part in str(text).split(","):
        if "=" in part:
            k, v = part.split("=")
            out[k.strip()] = float(v)
        elif part.strip():
            out["default"] = float(part)
    out.setdefault("default", 8.0)
    return out if len(out) > 1 else out["default"]


def format_table(rows, cols, floatfmt="{:.3g}"):
    """Plain-text table for printing."""
    def fmt(v):
        if isinstance(v, float):
            return "-" if math.isnan(v) else floatfmt.format(v)
        return str(v)
    cells = [[c for c in cols]] + [[fmt(r.get(c, "")) for c in cols] for r in rows]
    widths = [max(len(row[i]) for row in cells) for i in range(len(cols))]
    lines = ["  ".join(v.rjust(w) for v, w in zip(row, widths)) for row in cells]
    lines.insert(1, "  ".join("-" * w for w in widths))
    return "\n".join(lines)


def write_csv(rows, path, cols=None):
    cols = cols or list(rows[0].keys())
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


# --------------------------------------------------------------------------
# Floor homography (floor_calibration.json)
# --------------------------------------------------------------------------

def load_homography(path):
    """Read floor_calibration.json: a 3x3 matrix mapping floor cm -> image px.
    Accepts {"H": [[...]]}, {"homography": ...}, {"matrix": ...},
    {"H_floor_to_image": ...} or a bare 3x3 list.
    Returns a function to_cm(x_px, y_px) -> (x_cm, y_cm), using the inverse. If the file has a
    radial lens term k1 (with cx, cy, f_px), pixels are undistorted first."""
    with open(path) as f:
        data = json.load(f)
    dist = None
    if isinstance(data, dict) and data.get("k1"):     # radial lens term (tools/calibrate_floor.py, simulator)
        dist = (float(data["k1"]), float(data["cx"]), float(data["cy"]), float(data["f_px"]))
    if isinstance(data, dict):
        for k in ("H_floor_to_px", "homography_floor_cm_to_px", "H_floor_to_image", "H", "homography",
                  "matrix", "floor_to_image"):
            if k in data:
                data = data[k]
                break
        else:
            raise ValueError(f"{path}: no 3x3 matrix found (keys {list(data)})")
    H = np.array(data, dtype=np.float64).reshape(3, 3)
    Hinv = np.linalg.inv(H)

    def to_cm(x, y):
        if dist:
            k1, cx, cy, f = dist
            d = np.array([(x - cx) / f, (y - cy) / f])
            n = d.copy()
            for _ in range(20):
                n = d / (1 + k1 * (n @ n))
            x, y = cx + f * n[0], cy + f * n[1]
        v = Hinv @ np.array([x, y, 1.0])
        return v[0] / v[2], v[1] / v[2]
    return to_cm


def find_homography_for(label_dir, fallback=None):
    """floor_calibration.json next to the recording (or in the labels folder),
    otherwise the fallback path; None if neither exists."""
    for p in (os.path.join(os.path.dirname(os.path.abspath(label_dir)), "floor_calibration.json"),
              os.path.join(label_dir, "floor_calibration.json")):
        if os.path.isfile(p):
            return load_homography(p)
    if fallback:
        return load_homography(fallback)
    return None


def warn(msg):
    warnings.warn(msg, stacklevel=2)
