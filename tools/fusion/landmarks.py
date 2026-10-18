"""Landmark ranging: the robot's own front distance sensor measuring a board or
block whose position on the floor was tape-measured, as one more imperfect
sensor for the fusion.

Each RAW row carries the sensor's five zones (dL_cm, dCL_cm, dC_cm, dCR_cm,
dR_cm). For each zone the filter predicts what the reading should be: the
distance along the zone's ray, from the sensor (on the robot's front face) to
the first mapped face it hits. Readings that cannot be tied to one mapped face
are not used, and readings far from the prediction are rejected (gate), so a
hand, another robot or an unmapped brick in front of a board does not pull the
pose. No identity is needed: the map says which face the ray hits from the
predicted pose.

Map input (docs/data-formats.md):
  map.json        tools/fleet map: every board is a rectangle (centre line p1-p2,
                  thickness_cm): its two long faces and two end faces; polygon
                  obstacles with source "user" (tape-measured, e.g. a cinderblock)
                  give their edges too. Camera- or sensor-found obstacles and
                  circles are left out.
  landmarks.csv   name,x1_cm,y1_cm,x2_cm,y2_cm: one face (segment) per row; seen
                  from either side.

Zone model: one ray along the zone's direction (tof_zone_width_deg 0, the
default), or, with tof_zone_width_deg > 0, the nearest point of the face within
that slice (what a zone reporting its nearest target would read).
"""
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ZONE_COLS = ("dL_cm", "dCL_cm", "dC_cm", "dCR_cm", "dR_cm")
D2R = math.pi / 180.0

# what happened to each zone reading (counts per run, written by replay.py)
OUTCOMES = ("used", "gated", "no_face", "ambiguous", "edge", "grazing", "bad_reading", "pose_unknown")


class Face:
    """A straight face of a landmark: segment a -> b."""

    def __init__(self, name, a, b):
        self.name = name
        self.a = np.asarray(a, float)
        self.b = np.asarray(b, float)
        v = self.b - self.a
        self.length = float(np.hypot(*v))
        if self.length < 1e-6:
            raise ValueError(f"landmark face {name} has no length")
        self.u = v / self.length                      # along the face
        self.n = np.array([-self.u[1], self.u[0]])    # normal (either side may face the robot)

    def as_row(self):
        return [self.name] + [f"{v:.2f}" for v in (*self.a, *self.b)]


def _fleetmap():
    fleet = os.path.join(os.path.dirname(HERE), "fleet")
    if fleet not in sys.path:
        sys.path.insert(0, fleet)
    import fleetmap  # noqa: E402
    return fleetmap


def faces_from_map(m):
    """Faces of a normalised fleet map (fleetmap.normalize)."""
    out = []
    for b in m["boards"]:
        p1, p2 = np.asarray(b["p1"], float), np.asarray(b["p2"], float)
        u = (p2 - p1) / np.hypot(*(p2 - p1))
        n = np.array([-u[1], u[0]]) * b["thickness_cm"] / 2.0
        c = [p1 + n, p2 + n, p2 - n, p1 - n]           # left face, p2 end, right face, p1 end
        nm = b["name"]
        out += [Face(f"{nm} left", c[0], c[1]), Face(f"{nm} end p2", c[1], c[2]),
                Face(f"{nm} right", c[2], c[3]), Face(f"{nm} end p1", c[3], c[0])]
    for o in m["obstacles"]:
        if o["shape"] != "polygon" or o.get("source") != "user":
            continue
        pts = o["points"]
        for i in range(len(pts)):
            out.append(Face(f"{o['id']} edge {i + 1}", pts[i], pts[(i + 1) % len(pts)]))
    return out


def load(path):
    """map.json (tools/fleet format) or landmarks.csv -> list of Face."""
    if path.lower().endswith(".csv"):
        out = []
        with open(path, newline="", encoding="utf-8-sig") as f:
            for i, r in enumerate(csv.DictReader(f)):
                try:
                    a = (float(r["x1_cm"]), float(r["y1_cm"]))
                    b = (float(r["x2_cm"]), float(r["y2_cm"]))
                except (KeyError, TypeError, ValueError):
                    raise ValueError(f"{path} row {i + 2}: needs numbers in x1_cm,y1_cm,x2_cm,y2_cm")
                out.append(Face((r.get("name") or f"face{i + 1}").strip(), a, b))
        return out
    return faces_from_map(_fleetmap().load(path))


def write_csv(faces, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["name", "x1_cm", "y1_cm", "x2_cm", "y2_cm"])
        for fc in faces:
            w.writerow(fc.as_row())


# ---------------------------------------------------------------- geometry
def ray_face(p, d, face):
    """Ray p + s d (d unit) against one face: (s, g) with s > 0 the distance and
    g the hit point's position along the face from a (0..length), or None if it
    misses the segment."""
    den = d[0] * face.n[0] + d[1] * face.n[1]
    if abs(den) < 1e-12:
        return None
    s = ((face.a[0] - p[0]) * face.n[0] + (face.a[1] - p[1]) * face.n[1]) / den
    if s <= 1e-9:
        return None
    qx, qy = p[0] + s * d[0], p[1] + s * d[1]
    g = (qx - face.a[0]) * face.u[0] + (qy - face.a[1]) * face.u[1]
    if g < -1e-9 or g > face.length + 1e-9:
        return None
    return s, g


def nearest(p, d, faces):
    """First face the ray hits: (s, face index, g) or None."""
    best = None
    for i, fc in enumerate(faces):
        h = ray_face(p, d, fc)
        if h is not None and (best is None or h[0] < best[0]):
            best = (h[0], i, h[1])
    return best


def sensor_pose(s, fwd):
    """Sensor position (floor cm) for state s = [x, y, th, ...]."""
    c, sn = math.cos(s[2]), math.sin(s[2])
    return np.array([s[0] + fwd * c, s[1] + fwd * sn])


def line_range(s, ang, fwd, a, n):
    """Distance from the sensor along the ray at `ang` (rad, from the heading)
    to the infinite line through a with normal n, and its Jacobian with
    respect to (x, y, th). None if the ray runs away from or along the line."""
    th = s[2]
    c, sn = math.cos(th), math.sin(th)
    px, py = s[0] + fwd * c, s[1] + fwd * sn
    ux, uy = math.cos(th + ang), math.sin(th + ang)
    nu = n[0] * ux + n[1] * uy
    if abs(nu) < 1e-9:
        return None
    d = ((a[0] - px) * n[0] + (a[1] - py) * n[1]) / nu
    if d <= 0:
        return None
    dpx, dpy = -fwd * sn, fwd * c                   # d(sensor position)/d(th)
    dux, duy = -uy, ux                              # d(ray direction)/d(th)
    J = np.array([-n[0] / nu, -n[1] / nu,
                  -(n[0] * dpx + n[1] * dpy) / nu - d * (n[0] * dux + n[1] * duy) / nu])
    return d, J


def range_partials(s, ang, fwd, a, n):
    """For the ray at `ang` against the line (a, n): (d dh/d(zone direction), d dh/d(line moved along n)).
    The first is what a sensor turned on its mount does, the second what a board off its tape spot does.
    None if the ray misses the line."""
    r = line_range(s, ang, fwd, a, n)
    if r is None:
        return None
    d = r[0]
    th = s[2]
    ux, uy = math.cos(th + ang), math.sin(th + ang)
    nu = n[0] * ux + n[1] * uy
    return -d * (n[0] * -uy + n[1] * ux) / nu, 1.0 / nu


def zone_angle(s, ang, width, face, fwd):
    """Direction (rad, from the heading) whose ray gives the zone's reading of
    `face`: the zone's own direction for a single ray; with a slice `width`,
    the direction in the slice closest to square-on (the nearest point)."""
    if width <= 0:
        return ang
    p = sensor_pose(s, fwd)
    side = (face.a - p) @ face.n
    m = face.n if side > 0 else -face.n             # normal pointing from the sensor to the face
    rel = math.atan2(m[1], m[0]) - s[2]
    rel = (rel - ang + math.pi) % (2 * math.pi) - math.pi
    return ang + max(-width / 2, min(width / 2, rel))


def predict_zone(s, P, ang, faces, cfg):
    """Everything the filter needs to use one zone from the predicted state s
    (covariance P, 6x6 or 3x3 top-left used): returns (outcome, info). outcome is
    None if the zone can be used (then info = dict(h, J (3,), face, ang, sigma_g)),
    else one of OUTCOMES explaining why not."""
    fwd = cfg["tof_forward_cm"] - cfg["tof_recess_cm"]
    p = sensor_pose(s, fwd)
    th = s[2]
    hit = nearest(p, (math.cos(th + ang), math.sin(th + ang)), faces)
    if hit is None:
        return "no_face", None
    _, fi, _ = hit
    face = faces[fi]
    # association: every ray across +-tof_assoc_deg must hit this same face first
    da = cfg["tof_assoc_deg"] * D2R
    for k in (-1, 1):
        if da <= 0:
            break
        h2 = nearest(p, (math.cos(th + ang + k * da), math.sin(th + ang + k * da)), faces)
        if h2 is None or h2[1] != fi:
            return "ambiguous", None
    a_eff = zone_angle(s, ang, cfg["tof_zone_width_deg"] * D2R, face, fwd)
    r = line_range(s, a_eff, fwd, face.a, face.n)
    if r is None:
        return "no_face", None
    h, J = r
    if not (cfg["tof_min_cm"] <= h <= cfg["tof_max_cm"]):
        return "no_face", None
    u = np.array([math.cos(th + a_eff), math.sin(th + a_eff)])
    cos_inc = abs(u @ face.n)
    if cos_inc < math.cos(cfg["tof_max_incidence_deg"] * D2R):
        return "grazing", None
    # hit point along the face, and how unsure that is (numerically, from P)
    def along(st):
        pp = sensor_pose(st, fwd)
        rr = line_range(st, a_eff, fwd, face.a, face.n)
        if rr is None:
            return None
        q = pp + rr[0] * np.array([math.cos(st[2] + a_eff), math.sin(st[2] + a_eff)])
        return float((q - face.a) @ face.u)
    g = along(s)
    Jg = np.zeros(3)
    for j, eps in ((0, 1e-3), (1, 1e-3), (2, 1e-5)):
        sp = np.array(s[:3], float)
        sp[j] += eps
        gp = along(sp)
        if gp is None:
            return "grazing", None
        Jg[j] = (gp - g) / eps
    sig_g = math.sqrt(max(float(Jg @ np.asarray(P)[:3, :3] @ Jg), 0.0))
    margin = cfg["tof_end_margin_cm"] + 2.0 * sig_g
    if g < margin or g > face.length - margin:
        return "edge", None
    tan_inc = math.sqrt(max(1.0 - cos_inc * cos_inc, 0.0)) / cos_inc
    return None, dict(h=h, J=J, face=fi, ang=a_eff, sigma_g=sig_g, tan_inc=tan_inc)


def zone_sigma(h, cfg, tan_inc=0.0):
    """Reading noise (1 sd, cm): tof_noise_cm + tof_noise_rel x distance, plus
    what an error of tof_angle_sigma_deg in the zone's direction (sensor mounting,
    where in the zone the reading comes from) does to a face hit obliquely:
    distance x tan(incidence) x that angle."""
    return cfg["tof_noise_cm"] + cfg["tof_noise_rel"] * h + h * tan_inc * cfg["tof_angle_sigma_deg"] * D2R


# ---------------------------------------------------------------- calibration (tools/fusion/tof_calib.py)
CAL_FORMAT = "alvik-tof-calibration"


def load_cal(path):
    """tof_calibration.json (docs/data-formats.md) -> dict, with "faces" (list of Face) if it carries
    refined landmarks. Checks the format name."""
    import json
    with open(path) as f:
        d = json.load(f)
    if d.get("format") != CAL_FORMAT:
        raise ValueError(f"{path}: not a {CAL_FORMAT} file")
    d["path"] = os.path.abspath(path)
    d["used_runs"] = [str(r) for r in d.get("used_runs") or []]
    if d.get("landmarks"):
        d["faces"] = [Face(r["name"], (r["x1_cm"], r["y1_cm"]), (r["x2_cm"], r["y2_cm"])) for r in d["landmarks"]]
    return d


def read_cal_per_run(path):
    """tof_calibration_loro.csv -> {run_id: absolute path of the calibration made without it}."""
    base = os.path.dirname(os.path.abspath(path))
    out = {}
    with open(path, newline="", encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            rid = (r.get("run_id") or "").strip()
            c = (r.get("calibration") or "").strip()
            if rid and c:
                out[rid] = os.path.join(base, c)
    return out


def cal_robot(cal, robot):
    """Per-robot part of a calibration: (zone direction offsets deg [5], range bias cm, range scale)."""
    r = (cal or {}).get("robots", {}).get(robot) or (cal or {}).get("robots", {}).get("default") or {}
    off = r.get("zone_offset_deg") or [0.0] * len(ZONE_COLS)
    return [float(v) for v in off], float(r.get("range_bias_cm", 0.0)), float(r.get("range_scale", 1.0))
