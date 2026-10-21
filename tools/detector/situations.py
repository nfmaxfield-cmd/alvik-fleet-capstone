"""Accuracy by situation: how well the detector does on objects that are alone, close together,
near a person, at the edge, or standing still.

Nothing here knows what the objects are. A project supplies object sizes (px), which of its
types are people (hands, arms ...), the picture size and an optional work area; the situations
follow from the labels themselves:

    alone           no other labelled object within 1.5 sizes, no person near
    close together  another labelled object within 1.5 sizes (sizes: the larger of the two)
    near a person   a person-type label within 1.5 sizes (the larger of the two)
    at the edge     within 1 size of the picture's border, or of the work area's border / outside it
    standing still  the same object (same obj id) stayed put over the last 2 s of labelled frames
                    (needs a label 2 s back and no gap over 2 s, else it is not counted)

A label can be in more than one group (close together AND at the edge); "alone" is exclusive of
close together and near a person. A group is flagged "few" when it rests on fewer than
MIN_LABELS labels or MIN_OBJECTS different objects: its percentage says little.

Used by the trainer (tags the held-back labels), evaluate.py (--situations) and the report.
"""
import math

import cv2
import numpy as np

import detcore

SITUATIONS = [("alone", "Object alone"), ("close", "Close together"), ("person", "Near a person"),
              ("edge", "At the edge of the work area or picture"), ("still", "Standing still")]
NAMES = dict(SITUATIONS)
CLOSE_SIZES = 1.5
EDGE_SIZES = 1.0
STILL_S = 2.0
STILL_TOL_SIZES = 0.08      # moved less than this many sizes (and at least 2 px) = did not move
MIN_LABELS = 30
MIN_OBJECTS = 5


def label_key(frame, cls, x, y):
    """Key of one label inside a block: frame, type and position (as written in labels.csv)."""
    return "%d|%s|%.2f|%.2f" % (int(frame), cls, float(x), float(y))


def tag_frames(frames, sizes, person_types, image_size, polygon=None, times=None):
    """frames: {frame: [dict(cls, x, y, obj)]} (every checked label of one recording, in time order
    of `times`: {frame: seconds}). sizes: {type: px}. image_size: (w, h). polygon: work area corners
    in px or None. Returns {(frame, index in that frame's list): set of tags}."""
    person_types = set(person_types or ())
    W, H = image_size
    poly = None
    if polygon and len(polygon) >= 3:
        poly = np.array(polygon, np.float32).reshape(-1, 1, 2)
    size_of = lambda c: float(sizes.get(c) or detcore.DEFAULT_OBJ_SIZE_PX.get(c, 30))   # noqa: E731
    # each object's path (for "standing still")
    path = {}
    for fr, objs in frames.items():
        t = (times or {}).get(fr, fr / 25.0)
        for o in objs:
            oid = str(o.get("obj") or "").strip()
            if oid:
                path.setdefault(oid, []).append((t, o["x"], o["y"]))
    for v in path.values():
        v.sort()
    out = {}
    for fr, objs in frames.items():
        t = (times or {}).get(fr, fr / 25.0)
        for i, o in enumerate(objs):
            size = size_of(o["cls"])
            tags = set()
            close = near_person = False
            for j, p in enumerate(objs):
                if i == j:
                    continue
                if math.hypot(o["x"] - p["x"], o["y"] - p["y"]) <= CLOSE_SIZES * max(size, size_of(p["cls"])):
                    if p["cls"] in person_types:
                        near_person = True
                    else:
                        close = True
            if close:
                tags.add("close")
            if near_person:
                tags.add("person")
            if not close and not near_person:
                tags.add("alone")
            edge = min(o["x"], o["y"], W - o["x"], H - o["y"]) <= EDGE_SIZES * size
            if poly is not None and not edge:
                edge = cv2.pointPolygonTest(poly, (float(o["x"]), float(o["y"])), True) <= EDGE_SIZES * size
            if edge:
                tags.add("edge")
            if _still(path.get(str(o.get("obj") or "").strip()), t, o["x"], o["y"], size):
                tags.add("still")
            out[(fr, i)] = tags
    return out


def _still(path, t, x, y, size):
    """True when the object's labelled positions over the last STILL_S seconds all lie within a few
    pixels of (x, y): there is a label at least STILL_S seconds back (looking back at most one
    more second, so labels about 0.7 s apart still count), and no gap between labels longer than
    STILL_S."""
    if not path:
        return False
    tol = max(2.0, STILL_TOL_SIZES * size)
    win = [(tt, px, py) for tt, px, py in path if t - STILL_S - 1.0 - 1e-6 <= tt <= t + 1e-6]
    if len(win) < 2 or min(w[0] for w in win) > t - STILL_S + 1e-6:
        return False
    if any(math.hypot(px - x, py - y) > tol for _, px, py in win):
        return False
    ts = sorted(w[0] for w in win)
    return all(b - a <= STILL_S for a, b in zip(ts, ts[1:]))


def score(samples, preds, tags, classes, radius, thresh=0.0):
    """Per situation: how many labels, how many different objects, how many were found, how far
    off the centres and headings are. samples: detcore samples (recording = block name);
    preds: {sample index: detections}; tags: {block: {label_key: [tag, ...]}}.
    Returns rows (one per situation, then 'all'), each: situation, name, n_labels, n_objects,
    tp, found, centre_err_px_mean, heading_err_deg_median, few."""
    acc = {k: dict(labels=0, tp=0, err=[], herr=[], objs=set()) for k, _ in SITUATIONS}
    allr = dict(labels=0, tp=0, err=[], herr=[], objs=set())
    rad = detcore.parse_radius(radius) if isinstance(radius, str) else radius
    rad = rad if isinstance(rad, dict) else {"default": float(rad)}
    for i, s in enumerate(samples):
        dets = [d for d in preds.get(i, []) if d.get("score", 1.0) >= thresh]
        block = tags.get(s["recording"], {})
        for c in classes:
            p = [d for d in dets if d["cls"] == c]
            g = [o for o in s["objects"] if o["cls"] == c]
            pairs = {j: (pi, d) for pi, j, d in detcore.match_frame(p, g, rad.get(c, rad.get("default", 8.0)))}
            for j, o in enumerate(g):
                tg = block.get(label_key(s["frame"], c, o["x"], o["y"]))
                if tg is None:
                    continue
                oid = str(o.get("obj") or "").strip()
                key = (s["recording"], oid) if oid else (s["recording"], s["frame"], c, j)
                for bucket in [allr] + [acc[t] for t in tg if t in acc]:
                    bucket["labels"] += 1
                    bucket["objs"].add(key)
                    if j in pairs:
                        pi, d = pairs[j]
                        bucket["tp"] += 1
                        bucket["err"].append(d)
                        hp = p[pi].get("heading_deg")
                        if o.get("heading") is not None and hp is not None:
                            bucket["herr"].append(abs(detcore.wrap_deg(hp - o["heading"])))
    rows = []
    for k, b in [(k, acc[k]) for k, _ in SITUATIONS] + [("all", allr)]:
        n = b["labels"]
        rows.append(dict(situation=k, name=NAMES.get(k, "All labels"), n_labels=n, n_objects=len(b["objs"]),
                         tp=b["tp"], found=(b["tp"] / n if n else float("nan")),
                         centre_err_px_mean=float(np.mean(b["err"])) if b["err"] else float("nan"),
                         heading_err_deg_median=float(np.median(b["herr"])) if b["herr"] else float("nan"),
                         few=int(n < MIN_LABELS or len(b["objs"]) < MIN_OBJECTS)))
    return rows
