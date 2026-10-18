"""The site map: landmarks (boards, blocks) and the plywood's edges, each with a
position and how sure it is, and the locked map file (docs/data-formats.md,
"Locked site map").

A landmark is a rigid shape (its faces, as the fusion's landmarks.py reads them)
placed on the floor by a move (dx, dy) and a turn (deg) about its centre. The
site setup estimates that move and turn; everything else about the shape
(length, thickness) is taken as given. An edge is a straight side of the
plywood: its offset along the outward normal from where the map puts it, and a
small turn about its middle.
"""
import csv
import datetime as dt
import hashlib
import json
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (TOOLS, os.path.join(TOOLS, "fusion"), os.path.join(TOOLS, "fleet")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fleetmap as FM  # noqa: E402
import landmarks as LM  # noqa: E402

SITE_FORMAT = "alvik-site-map"
SITE_VERSION = 1
SIDES = ("x0", "x1", "y0", "y1")
SIDE_WORDS = {"x0": "the x = min side", "x1": "the x = max side", "y0": "the y = min side", "y1": "the y = max side"}
D2R = math.pi / 180.0
CM_PER_IN = 2.54


def to_in(cm):
    """cm -> inches to the nearest 1/16, like 15 3/4" (as tools/landmark_pick.py)."""
    n = round(abs(cm) / CM_PER_IN * 16)
    w, f = divmod(n, 16)
    g = math.gcd(f, 16)
    frac = f"{f // g}/{16 // g}" if f else ""
    body = f"{w} {frac}" if (w and frac) else (frac or str(w))
    return f"{'-' if cm < 0 and n else ''}{body}\""


def cm_and_in(cm, digits=2):
    return f"{cm:.{digits}f} cm ({to_in(cm)})"


def rot(v, phi):
    c, s = math.cos(phi), math.sin(phi)
    v = np.asarray(v, float)
    return np.stack([c * v[..., 0] - s * v[..., 1], s * v[..., 0] + c * v[..., 1]], -1)


# ---------------------------------------------------------------- landmarks and edges
class Landmark:
    """One rigid landmark. kind: board (from map.json boards) or block (polygon obstacle with source user).
    faces: list of landmarks.Face at the prior (given) position. anchors: named floor points a person can
    tape-measure (a board's foot corners, a block's corners)."""

    def __init__(self, name, kind, faces, anchors, shape, prior_sigma_cm=1.0, source="map"):
        self.name, self.kind, self.faces = name, kind, faces
        self.anchors = {k: np.asarray(v, float) for k, v in anchors.items()}
        self.shape = shape                      # board: dict(p1, p2, thickness_cm, ...); block: dict(points)
        pts = np.array([f.a for f in faces] + [f.b for f in faces])
        self.center = pts.mean(0)
        if kind == "board":
            u = np.asarray(shape["p2"], float) - np.asarray(shape["p1"], float)
            self.length = float(np.hypot(*u))
            self.axis = u / self.length
        else:
            self.length, self.axis = 0.0, np.array([1.0, 0.0])
        self.size = float(max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1]), 1.0))
        self.prior_sigma_cm = float(prior_sigma_cm)     # each end / corner, from the clicks (or tape)
        self.source = source

    def move(self, pts, g):
        """Floor points of this landmark moved by g = (dx, dy, turn_deg, stretch_cm): a board is first made
        stretch_cm longer (each end half of it further from the middle, along the board), then turned about its
        centre and moved. Blocks have no stretch."""
        pts = np.asarray(pts, float)
        rel = pts - self.center
        if self.length > 0 and len(g) > 3 and g[3]:
            along = rel @ self.axis
            rel = rel + np.multiply.outer(along * (g[3] / self.length), self.axis)
        return self.center + rot(rel, g[2] * D2R) + np.asarray(g[:2], float)

    def moved_shape(self, g):
        s = dict(self.shape)
        if self.kind == "board":
            s["p1"] = [round(float(v), 3) for v in self.move(self.shape["p1"], g)]
            s["p2"] = [round(float(v), 3) for v in self.move(self.shape["p2"], g)]
        else:
            s["points"] = [[round(float(v), 3) for v in p] for p in self.move(self.shape["points"], g)]
        return s

    def moved_faces(self, g):
        return [LM.Face(f.name, self.move(f.a, g), self.move(f.b, g)) for f in self.faces]

    def prior_sigmas(self):
        """(position sd cm, turn sd deg, stretch sd cm) of the prior from end/corner clicks of prior_sigma_cm
        each: the centre of two independent ends is sigma/sqrt(2) off, the turn sigma*sqrt(2)/length, the
        length sigma*sqrt(2) (a tape-measured length: length_sd_cm). Blocks: no stretch (0.001)."""
        s = self.prior_sigma_cm
        st = getattr(self, "length_sd_cm", None)
        st = (s * math.sqrt(2.0) if st is None else st) if self.kind == "board" else 0.001
        return s / math.sqrt(2.0), math.degrees(s * math.sqrt(2.0) / max(self.size, 1.0)), st


def board_anchors(b):
    """A board's tape-measurable points: the two ends of each long face at the floor."""
    p1, p2 = np.asarray(b["p1"], float), np.asarray(b["p2"], float)
    u = (p2 - p1) / np.hypot(*(p2 - p1))
    n = np.array([-u[1], u[0]]) * b["thickness_cm"] / 2.0
    return {"left p1": p1 + n, "left p2": p2 + n, "right p1": p1 - n, "right p2": p2 - n}


def landmarks_from_map(m, prior_sigma_cm=1.0, include=None):
    """Landmarks of a (normalised) fleet map: every board, and polygon obstacles with source user."""
    out = []
    for b in m["boards"]:
        if include is not None and b["name"] not in include:
            continue
        faces = LM.faces_from_map({"boards": [b], "obstacles": []})
        out.append(Landmark(b["name"], "board", faces, board_anchors(b), dict(b), prior_sigma_cm))
    for o in m["obstacles"]:
        if o["shape"] != "polygon" or o.get("source") != "user":
            continue
        if include is not None and o["id"] not in include:
            continue
        faces = LM.faces_from_map({"boards": [], "obstacles": [o]})
        anchors = {f"corner {i + 1}": p for i, p in enumerate(o["points"])}
        out.append(Landmark(o["id"], "block", faces, anchors, dict(o), prior_sigma_cm))
    return out


class Edge:
    """One side of the plywood. open: no border board (the robots' edge guard sees it)."""

    def __init__(self, side, floor, open_, prior_sigma_cm=1.0):
        self.side, self.open = side, bool(open_)
        x0, x1, y0, y1 = floor["x_min"], floor["x_max"], floor["y_min"], floor["y_max"]
        if side == "x0":
            self.a, self.b, self.n = np.array([x0, y0]), np.array([x0, y1]), np.array([-1.0, 0.0])
        elif side == "x1":
            self.a, self.b, self.n = np.array([x1, y0]), np.array([x1, y1]), np.array([1.0, 0.0])
        elif side == "y0":
            self.a, self.b, self.n = np.array([x0, y0]), np.array([x1, y0]), np.array([0.0, -1.0])
        else:
            self.a, self.b, self.n = np.array([x0, y1]), np.array([x1, y1]), np.array([0.0, 1.0])
        self.mid = (self.a + self.b) / 2
        self.u = (self.b - self.a) / np.hypot(*(self.b - self.a))
        self.length = float(np.hypot(*(self.b - self.a)))
        self.prior_sigma_cm = float(prior_sigma_cm)

    def line(self, e):
        """(point, outward normal) of the edge moved by e = (offset_cm, turn_deg)."""
        n = rot(self.n, e[1] * D2R)
        return self.mid + e[0] * self.n, n

    def point(self, frac, e):
        """Floor point at `frac` (0..1) along the edge, moved by e."""
        m, _ = self.line(e)
        return m + (frac - 0.5) * self.length * rot(self.u, e[1] * D2R)


def edges_from_map(m, open_sides, prior_sigma_cm=1.0):
    return [Edge(s, m["floor"], s in open_sides, prior_sigma_cm) for s in SIDES]


# ---------------------------------------------------------------- estimate (what the solver returns)
class Estimate:
    """Landmark moves g[name] = (dx, dy, turn_deg) and edge moves e[side] = (offset_cm, turn_deg), with
    covariances (3x3 / 2x2, cm and deg) and the camera frame's own uncertainty (cm) that every position
    shares."""

    def __init__(self, landmarks, edges, g=None, e=None, cov_g=None, cov_e=None, cam_sys_cm=0.0):
        self.landmarks = {l.name: l for l in landmarks}
        self.edges = {x.side: x for x in edges}
        self.g = {k: np.zeros(4) for k in self.landmarks} if g is None else {k: np.asarray(v, float) for k, v in g.items()}
        self.e = {k: np.zeros(2) for k in self.edges} if e is None else {k: np.asarray(v, float) for k, v in e.items()}
        self.cov_g = cov_g or {k: np.diag([l.prior_sigmas()[0] ** 2] * 2 + [l.prior_sigmas()[1] ** 2,
                                                                           l.prior_sigmas()[2] ** 2])
                               for k, l in self.landmarks.items()}
        self.cov_e = cov_e or {k: np.diag([x.prior_sigma_cm ** 2, 1.0]) for k, x in self.edges.items()}
        self.cam_sys_cm = float(cam_sys_cm)
        self.extra = {}

    def anchor(self, name, anchor):
        l = self.landmarks[name]
        return l.move(l.anchors[anchor], self.g[name])

    def anchor_cov(self, name, anchor):
        """2x2 covariance (cm^2) of a landmark point: the move/turn covariance through the point's lever arm,
        plus the camera frame's share."""
        l = self.landmarks[name]
        g = np.asarray(self.g[name], float)
        J = np.zeros((2, 4))
        base = l.move(l.anchors[anchor], g)
        for k, eps in enumerate((1e-3, 1e-3, 1e-3, 1e-3)):
            gg = g.copy()
            gg[k] += eps
            J[:, k] = (l.move(l.anchors[anchor], gg) - base) / eps
        return J @ self.cov_g[name] @ J.T + np.eye(2) * self.cam_sys_cm ** 2

    def edge_point_sigma(self, side, frac):
        """sd (cm) of where the edge is, along its normal, at `frac` along it."""
        c = self.cov_e[side]
        L = self.edges[side].length
        J = np.array([1.0, (frac - 0.5) * L * D2R])
        return math.sqrt(max(float(J @ c @ J), 0.0) + self.cam_sys_cm ** 2)

    def points(self):
        """Every tape-measurable point: list of dict(key, landmark, anchor, xy, cov (2x2), sigma (larger
        semi-axis, cm), kind)."""
        out = []
        for name, l in self.landmarks.items():
            for a in l.anchors:
                c = self.anchor_cov(name, a)
                out.append(dict(key=f"{name}: {a}", landmark=name, anchor=a, xy=self.anchor(name, a), cov=c,
                                sigma=math.sqrt(max(np.linalg.eigvalsh(c))), kind=l.kind))
        for side, x in self.edges.items():
            if not x.open:
                continue
            for frac in (0.25, 0.5, 0.75):
                s = self.edge_point_sigma(side, frac)
                out.append(dict(key=f"edge {side}: {int(frac * 100)}%", landmark=f"edge {side}", anchor=frac,
                                xy=x.point(frac, self.e[side]), normal=self.edges[side].line(self.e[side])[1],
                                sigma=s, cov=None, kind="edge"))
        return out

    def landmark_sigma(self, name):
        """Largest sd (cm) of any of a landmark's points."""
        return max(math.sqrt(max(np.linalg.eigvalsh(self.anchor_cov(name, a)))) for a in self.landmarks[name].anchors)

    def faces(self):
        out = []
        for name, l in self.landmarks.items():
            out += l.moved_faces(self.g[name])
        return out


# ---------------------------------------------------------------- the locked map
def file_hash(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, "rb") as f:
        return "sha256:" + hashlib.sha256(f.read()).hexdigest()[:16]


def camera_info(calib_path):
    """What camera calibration the map was made with: path, hash, fit quality."""
    if not calib_path or not os.path.exists(calib_path):
        return dict(path=None, hash=None, note="no camera calibration given")
    with open(calib_path) as f:
        d = json.load(f)
    fit = d.get("fit") or {}
    return dict(path=os.path.abspath(calib_path), hash=file_hash(calib_path), mode=fit.get("mode"),
                rms_cm=fit.get("rms_cm"), loo_rms_cm=fit.get("loo_rms_cm"), n_points=fit.get("n_points"))


def locked_map(base_map, est, version, audit, camera, survey, notes="", robots=None, sensor=None, fill_in=None):
    """The fleet map with the estimated landmarks and floor, plus the "site" block (docs/data-formats.md)."""
    m = json.loads(json.dumps(base_map))
    boards, obstacles = [], []
    for b in m["boards"]:
        if b["name"] in est.landmarks:
            b = dict(b)
            b.update(est.landmarks[b["name"]].moved_shape(est.g[b["name"]]))
        boards.append(b)
    for o in m["obstacles"]:
        if o.get("id") in est.landmarks:
            o = dict(o)
            o.update(est.landmarks[o["id"]].moved_shape(est.g[o["id"]]))
        obstacles.append(o)
    m["boards"], m["obstacles"] = boards, obstacles
    fl = dict(m["floor"])
    for side, x in est.edges.items():
        if not x.open:
            continue
        p, _ = x.line(est.e[side])
        key = {"x0": "x_min", "x1": "x_max", "y0": "y_min", "y1": "y_max"}[side]
        fl[key] = round(float(p[0] if side[0] == "x" else p[1]), 3)
    m["floor"] = fl
    lms = {}
    for name, l in est.landmarks.items():
        c = est.cov_g[name]
        lms[name] = dict(kind=l.kind, move_cm=[round(float(v), 3) for v in est.g[name][:2]],
                         turn_deg=round(float(est.g[name][2]), 3),
                         stretch_cm=round(float(est.g[name][3]), 3),
                         sigma_cm=round(est.landmark_sigma(name), 3), sigma_in=to_in(est.landmark_sigma(name)),
                         sd_move_cm=[round(math.sqrt(c[0, 0]), 3), round(math.sqrt(c[1, 1]), 3)],
                         sd_turn_deg=round(math.sqrt(c[2, 2]), 3), sd_stretch_cm=round(math.sqrt(c[3, 3]), 3),
                         points={a: dict(x_cm=round(float(p[0]), 3), y_cm=round(float(p[1]), 3),
                                         sigma_cm=round(math.sqrt(max(np.linalg.eigvalsh(est.anchor_cov(name, a)))), 3))
                                 for a in l.anchors for p in [est.anchor(name, a)]})
    edges = {}
    for side, x in est.edges.items():
        p, n = x.line(est.e[side])
        edges[side] = dict(open=x.open, offset_cm=round(float(est.e[side][0]), 3), turn_deg=round(float(est.e[side][1]), 3),
                           sigma_cm=round(est.edge_point_sigma(side, 0.5), 3) if x.open else None,
                           position_cm=round(float(p[0] if side[0] == "x" else p[1]), 3))
    site = dict(format=SITE_FORMAT, version=int(version), locked=True,
                locked_at=dt.datetime.now().astimezone().isoformat(timespec="seconds"),
                camera_calibration=camera, camera_frame_sigma_cm=round(est.cam_sys_cm, 3),
                landmarks=lms, edges=edges, audit=audit, fill_in=fill_in or [], survey=survey,
                robots=robots or {}, sensor=sensor or {}, notes=notes)
    site["map_hash"] = FM.map_hash(m)
    m["site"] = site
    return m


def write_lock(folder, m, est):
    """map.json (with the site block) and landmarks.csv (faces + sigma) in `folder`; old files kept as .bak."""
    os.makedirs(folder, exist_ok=True)
    mp = os.path.join(folder, "map.json")
    for p in (mp, os.path.join(folder, "landmarks.csv")):
        if os.path.exists(p):
            os.replace(p, p + ".bak")
    with open(mp + ".tmp", "w") as f:
        json.dump(m, f, indent=1)
        f.write("\n")
    os.replace(mp + ".tmp", mp)
    site = m["site"]
    with open(os.path.join(folder, "landmarks.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["name", "x1_cm", "y1_cm", "x2_cm", "y2_cm", "landmark", "kind", "source", "sigma_cm", "sigma_in",
                    "map_version", "locked_at"])
        for name, l in est.landmarks.items():
            s = est.landmark_sigma(name)
            for fc in l.moved_faces(est.g[name]):
                w.writerow([fc.name] + [f"{v:.3f}" for v in (*fc.a, *fc.b)] +
                           [name, l.kind, "site-setup", f"{s:.3f}", to_in(s), site["version"], site["locked_at"]])
    return mp


def read_lock(folder_or_map):
    """(map dict, site block) of a locked map; site is None if the map has none or was edited since
    (its map_hash no longer matches: then it is not locked any more)."""
    p = folder_or_map if folder_or_map.endswith(".json") else os.path.join(folder_or_map, "map.json")
    with open(p) as f:
        m = json.load(f)
    site = m.get("site")
    if not site or site.get("format") != SITE_FORMAT:
        return m, None
    if site.get("map_hash") != FM.map_hash(m):
        return m, None
    return m, site


def next_version(folder):
    p = os.path.join(folder, "map.json")
    if not os.path.exists(p):
        return 1
    try:
        with open(p) as f:
            s = json.load(f).get("site") or {}
        return int(s.get("version", 0)) + 1
    except (OSError, ValueError):
        return 1


def tof_calibration_from(est, robots, noise, used_runs, systematic_cm=0.0, map_given=None):
    """The estimate as a distance-sensor calibration (format of tools/fusion/tof_calib.py), so the fusion
    (replay.py --tof-cal) uses the locked landmarks with their uncertainty and each robot's sensor model."""
    lms = []
    for name, l in est.landmarks.items():
        s = est.landmark_sigma(name)
        for fc in l.moved_faces(est.g[name]):
            lms.append(dict(name=fc.name, x1_cm=round(float(fc.a[0]), 3), y1_cm=round(float(fc.a[1]), 3),
                            x2_cm=round(float(fc.b[0]), 3), y2_cm=round(float(fc.b[1]), 3), board=name,
                            sigma_cm=round(s, 3)))
    return dict(format=LM.CAL_FORMAT, version=1, used_runs=sorted(used_runs), zone_width_deg=noise.get("width", 0.0),
                delay_s=0.0, noise_cm=round(noise["a"], 3), noise_rel=round(noise["b"], 4),
                frame_corr=0.0, lag1_corr=0.0, systematic_cm=round(systematic_cm, 3), robots=robots,
                boards={n: dict(dx_cm=round(float(est.g[n][0]), 3), dy_cm=round(float(est.g[n][1]), 3),
                                turn_deg=round(float(est.g[n][2]), 3)) for n in est.landmarks},
                landmarks=lms, map_given=map_given, face_sigma_cm=round(max(r["sigma_cm"] for r in lms), 3) if lms else 1.0,
                source="site-setup")


# ---------------------------------------------------------------- audit (verification by tape)
AUDIT_TAPE_SD_CM = 0.1        # tape reading to 1/16 in (0.16 cm) and holding the tape on the point
AUDIT_FLOOR_CM = 0.3          # below this the tape can't tell a difference
AUDIT_K = 3.0                 # allowed error in standard deviations


def audit_allowed(sigma_cm):
    """How far a tape measurement may differ from the map for the point to pass: max(0.3 cm, 3 x the combined
    sd of the map point and the tape). For a 2-D point with that sd the chance of a correct map failing one
    point is exp(-4.5) = 1.1%, 3.3% over three points (see tools/site/README.md, "Pass rule")."""
    return max(AUDIT_FLOOR_CM, AUDIT_K * math.hypot(sigma_cm, AUDIT_TAPE_SD_CM))


TARGET_SD_CM = 0.5            # a point the survey left less sure than this is tape-measured as data first


def pick_fill_in(points, target_cm=TARGET_SD_CM, per_landmark=1):
    """Points the robots could not pin down (sd above target_cm: e.g. a board end with another board right
    behind it, or an edge stretch nobody could drive to): the worst point of each such landmark/edge, to be
    tape-measured and merged as data before the audit."""
    worst = {}
    for p in points:
        if p["sigma"] > target_cm and (p["landmark"] not in worst or p["sigma"] > worst[p["landmark"]]["sigma"]):
            worst[p["landmark"]] = p
    return sorted(worst.values(), key=lambda p: -p["sigma"])


def pick_audit(points, n_worst=2, n_random=1, seed=None, exclude=()):
    """The audit set: the n_worst points with the largest uncertainty (each from a different landmark), plus
    n_random others at random (a check that does not trust the uncertainty model). exclude: keys already
    used as data."""
    pts = [p for p in points if p["key"] not in exclude]
    pts.sort(key=lambda p: -p["sigma"])
    out, used = [], set()
    for p in pts:
        if len(out) >= n_worst:
            break
        if p["landmark"] in used:
            continue
        out.append(dict(p, why="largest uncertainty"))
        used.add(p["landmark"])
    rest = [p for p in pts if p["key"] not in {q["key"] for q in out} and p["landmark"] not in used]
    rng = random.Random(seed)
    for p in rng.sample(rest, min(n_random, len(rest))):
        out.append(dict(p, why="random check"))
    return out


def audit_compare(point, tape_xy):
    """Compare one audited point with the tape. Edges are compared along their normal only (one number).
    Returns dict(error_cm, allowed_cm, passed, dx, dy)."""
    if point["kind"] == "edge":
        n = np.asarray(point["normal"], float)
        err = float(np.dot(np.asarray(tape_xy, float) - point["xy"], n))
        d = (err * n[0], err * n[1])
        e = abs(err)
    else:
        d = (float(tape_xy[0] - point["xy"][0]), float(tape_xy[1] - point["xy"][1]))
        e = math.hypot(*d)
    allowed = audit_allowed(point["sigma"])
    return dict(error_cm=e, allowed_cm=allowed, passed=e <= allowed, dx=d[0], dy=d[1])
