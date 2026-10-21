"""Home positions: spots where an object's position (and facing) is known exactly: a robot's dock,
a robot arm's home pose, a parts tray, a fixture on a conveyor. Marked once in setup; every visit
(an object sitting there, still, for a few seconds) is a free confirmation, for the tracker (a
fresh, exact anchor for the chain) and for outside data (the log is re-aligned there and its
drift measured).

Stored in project.json beside the camera's work area, as plain dicts (cameras[<key>]["homes"];
Project.set_camera_settings keeps keys it does not know):

    {"name": "Left dock", "camera": "default", "x": 312.4, "y": 401.0, "facing_deg": 90.0, "radius": 12.0,
     "kind": "park", "owner": "Left", "cls": "alvik", "still_s": 2.0, "sd_px": 1.0, "sd_deg": 1.0}

x, y: image px in that camera (labelling convention); facing_deg: image heading (0 = image right,
counter-clockwise on screen), or None for a spot without a facing (a tray); radius px: how close
counts as "at home"; kind: park (the object stays a while) or station (a touch-and-go measuring
spot: shorter still time); owner: the object that lives there (optional: else the visitor is
whoever sits there). Optional floor values x_cm, y_cm, heading (floor deg) are kept when the home
came from a floor map.

The Alvik dock is the example preset (alvik_dock_homes): the person clicks the middle of the back
wall's inner top edge and the middle of the funnel's mouth; the docked robot's centre is the
bumper depth in front of the wall (hardware/dock/dock_params.py), facing the wall; dock version 2
also gives its three post stations.
"""
import math
import os
import sys

import numpy as np

try:
    from .outside import Anchor, floor_heading, image_heading
except ImportError:
    from outside import Anchor, floor_heading, image_heading

KINDS = ("park", "station")
DEFAULT_STILL_S = {"park": 2.0, "station": 0.4}
DEFAULT_STILL_PX = 2.0       # an object moving less than this (px) counts as still
MAX_GAP_S = 1.0              # samples further apart than this break a visit


# =============================================================================== stored form
def make_home(name, camera, x, y, facing_deg=None, radius=12.0, kind="park", owner=None, cls=None, still_s=None,
              sd_px=1.0, sd_deg=1.0, **extra):
    """A home position dict (validated). Raises ValueError with a plain message."""
    name = str(name or "").strip()
    if not name:
        raise ValueError("a home position needs a name")
    if kind not in KINDS:
        raise ValueError("kind must be one of %s" % ", ".join(KINDS))
    x, y, radius = float(x), float(y), float(radius)
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("a home position needs a place in the picture")
    if not radius > 0:
        raise ValueError("a home position needs a size (radius > 0)")
    h = dict(name=name, camera=str(camera or "default"), x=round(x, 2), y=round(y, 2),
             facing_deg=None if facing_deg is None else round(float(facing_deg) % 360.0, 2), radius=round(radius, 2),
             kind=kind, owner=owner or None, cls=cls or None,
             still_s=float(still_s) if still_s is not None else DEFAULT_STILL_S[kind],
             sd_px=float(sd_px), sd_deg=float(sd_deg))
    for k in ("x_cm", "y_cm", "heading", "note", "preset", "group"):
        if k in extra and extra[k] is not None:
            h[k] = extra[k]
    return h


def homes_of(project_data, camera_key=None):
    """Homes from a project.json dict (every camera, or one)."""
    cams = (project_data or {}).get("cameras") or {}
    out = []
    for key, c in cams.items():
        if camera_key is not None and key != camera_key:
            continue
        for h in (c or {}).get("homes") or []:
            out.append(dict(h, camera=h.get("camera") or key))
    return out


def set_homes(project_data, camera_key, homes):
    """Put a camera's homes into a project.json dict (the caller saves it). Returns the stored list."""
    stored = [make_home(**dict(h, camera=camera_key)) for h in homes]
    names = [h["name"] for h in stored]
    if len(set(names)) != len(names):
        raise ValueError("two home positions share a name")
    cams = project_data.setdefault("cameras", {})
    cams[camera_key] = dict(cams.get(camera_key) or {}, homes=stored)
    return stored


def home_floor_pose(home, camera, height_cm=0.0):
    """(x_cm, y_cm, heading floor deg or None) of a home: from its stored floor values, else from
    its pixel place through the camera, at the object's height."""
    if home.get("x_cm") is not None and home.get("y_cm") is not None:
        return float(home["x_cm"]), float(home["y_cm"]), (None if home.get("heading") is None else float(home["heading"]))
    x, y = camera.to_floor(float(home["x"]), float(home["y"]), height_cm)
    h = None
    if home.get("facing_deg") is not None:
        h = floor_heading(camera, float(home["x"]), float(home["y"]), height_cm, float(home["facing_deg"]))
    return float(x), float(y), h


def home_from_floor(name, camera_name, camera, x_cm, y_cm, heading=None, height_cm=0.0, radius_cm=4.0, **kw):
    """A home given on the floor (cm, floor heading): its pixel place, facing and radius in that camera."""
    u, v = camera.project((x_cm, y_cm, height_cm))
    f = None if heading is None else image_heading(camera, x_cm, y_cm, height_cm, heading)
    r = radius_cm * camera.px_per_cm(x_cm, y_cm, height_cm)
    return make_home(name, camera_name, u, v, f, r, x_cm=round(float(x_cm), 3), y_cm=round(float(y_cm), 3),
                     heading=None if heading is None else round(float(heading) % 360.0, 3), **kw)


# =============================================================================== visits
def tracks_from_labels(objects_by_frame, frame_times, statuses=None):
    """{frame: [objects]} (labels, tracker rows or guesses; Session.frames-style dicts work too) ->
    {obj: [(frame, t, x, y)]}. statuses: keep only these (None: all)."""
    out = {}
    for f, v in objects_by_frame.items():
        objs = v.get("objects", []) if isinstance(v, dict) else v
        t = frame_times.get(int(f))
        if t is None:
            continue
        for o in objs:
            if statuses is not None and o.get("status") not in statuses:
                continue
            if o.get("obj") is None:
                continue
            out.setdefault(o["obj"], []).append((int(f), float(t), float(o["x"]), float(o["y"])))
    for k in out:
        out[k].sort(key=lambda r: r[1])
    return out


def visitor_name(obj, obj_names=None):
    """The name of a followed object: obj_names[obj] (the roll call), or obj itself when it is a name."""
    if obj_names and obj in obj_names and obj_names[obj]:
        return obj_names[obj]
    return obj if isinstance(obj, str) else None


def find_visits(homes, tracks, still_px=DEFAULT_STILL_PX, camera=None, obj_names=None):
    """Visits: an object inside a home's radius and still (within still_px of where the stay
    started) for at least the home's still_s. tracks {obj: [(frame, t, x, y)]} in the home's camera
    (pass camera= to keep only that camera's homes). obj_names {obj: name} (the roll call): a home
    with an owner then counts only visits by an object of that name (or of no known name: the caller
    checks those). Returns a list of dicts: home, obj, frame_start, frame_end, t_start, t_end, x, y
    (mean place), still_s, off_px (distance from the home's centre), name (the visitor's known name or
    None)."""
    out = []
    for h in homes:
        if camera is not None and h.get("camera") != camera:
            continue
        need = float(h.get("still_s") or DEFAULT_STILL_S.get(h.get("kind"), 2.0))
        for obj, rows in tracks.items():
            vname = visitor_name(obj, obj_names)
            if h.get("owner") and vname is not None and vname != h["owner"]:
                continue
            run = []

            def close(run):
                if len(run) >= 2 and run[-1][1] - run[0][1] >= need:
                    xs, ys = [r[2] for r in run], [r[3] for r in run]
                    mx, my = float(np.mean(xs)), float(np.mean(ys))
                    out.append(dict(home=h["name"], obj=obj, name=vname, frame_start=run[0][0], frame_end=run[-1][0],
                                    t_start=run[0][1], t_end=run[-1][1], x=mx, y=my,
                                    still_s=run[-1][1] - run[0][1], off_px=math.hypot(mx - h["x"], my - h["y"])))

            for r in rows:
                inside = math.hypot(r[2] - h["x"], r[3] - h["y"]) <= h["radius"]
                if run and (not inside or r[1] - run[-1][1] > MAX_GAP_S or
                            math.hypot(r[2] - run[0][2], r[3] - run[0][3]) > still_px):
                    close(run)
                    run = []
                if inside:
                    run.append(r)
            close(run)
    out.sort(key=lambda v: v["t_start"])
    return out


def confirmations(visits, homes, frame_times=None):
    """Visits -> confirmation events for the tracker: one label-like dict per visit at the home's
    exact place and facing, for the frames of the stay (frame_start, frame_end), source "home"."""
    by = {h["name"]: h for h in homes}
    out = []
    for v in visits:
        h = by.get(v["home"])
        if h is None:
            continue
        out.append(dict(kind="home", home=h["name"], obj=v["obj"], frame_start=v["frame_start"], frame_end=v["frame_end"],
                        t_start=v["t_start"], t_end=v["t_end"], x=h["x"], y=h["y"], heading_deg=h.get("facing_deg"),
                        heading_known=h.get("facing_deg") is not None, cls=h.get("cls"), source="home",
                        owner=h.get("owner"),
                        message="%s confirmed at %s (frames %d-%d), from the home position" % (
                            "Object %s" % v["obj"], h["name"], v["frame_start"], v["frame_end"])))
    return out


def anchors_from_visits(visits, homes, camera, obj_names=None, height_cm=0.0, outside=None, settings_sd_cm=None):
    """Visits -> {name: [Anchor]} for outside-data alignment: two full-pose anchors per visit (start
    and end of the stay: the object sat still at the home). The name is the home's owner, else
    obj_names[obj], else (with outside=OutsideData) the one stream standing still then.
    A home's owner is never taken on trust: a visitor the roll call names differently is left out,
    and a visitor of no known name counts for the owner only when the owner's own stream stood still
    then (with outside=; without it the caller vouches for the visitor)."""
    by = {h["name"]: h for h in homes}
    out = {}
    for v in visits:
        h = by.get(v["home"])
        if h is None:
            continue
        known = visitor_name(v["obj"], obj_names)
        if h.get("owner"):
            if known is not None and known != h["owner"]:
                continue
            if known is None and outside is not None and not stream_still(outside, h["owner"], v["t_start"], v["t_end"]):
                continue
            name = h["owner"]
        else:
            name = known
        if name is None and outside is not None:
            name = still_stream_at(outside, v["t_start"], v["t_end"])
        if name is None:
            continue
        x, y, hd = home_floor_pose(h, camera, height_cm)
        ppc = camera.px_per_cm(x, y, height_cm)
        sd = settings_sd_cm if settings_sd_cm is not None else max(0.2, float(h.get("sd_px", 1.0)) / max(ppc, 1e-6))
        for t in (v["t_start"], v["t_end"]):
            out.setdefault(name, []).append(Anchor(t, x, y, hd, sd_cm=sd, sd_deg=float(h.get("sd_deg", 1.0)),
                                                   kind="home", note=h["name"]))
    return out


def _still_score(st, t0, t1):
    """How still one stream stood over [t0, t1] (0..1), or None when its rows can't tell."""
    if not len(st):
        return None
    m = (st.t >= t0) & (st.t <= t1)
    if m.sum() == 0:
        i = np.searchsorted(st.t, t0) - 1
        j = np.searchsorted(st.t, t1)
        if i < 0 or j >= len(st):
            return None
        moved = st.frame == "own" and (abs(st.dist[j] - st.dist[i]) >= 0.5 or
                                       abs(st.heading[j] - st.heading[i]) >= 2.0)
        return 0.0 if moved else 1.0
    return float(st.still[m].mean())


def stream_still(outside, name, t0, t1, need=0.6):
    """Did the stream called name stand still over [t0, t1]?"""
    st = (outside.streams or {}).get(name)
    if st is None:
        return False
    sc = _still_score(st, t0, t1)
    return sc is not None and sc >= need


def still_stream_at(outside, t0, t1):
    """The stream that clearly stood still over [t0, t1] (the most still samples, at least 60 %, and
    30 points ahead of every other; a stream with no rows then counts as still if it neither drove
    nor turned between the rows either side: a robot logs only while it runs). None if unclear."""
    score = {}
    for name, st in outside.streams.items():
        sc = _still_score(st, t0, t1)
        if sc is not None:
            score[name] = sc
    if not score:
        return None
    ranked = sorted(score.items(), key=lambda kv: -kv[1])
    best = ranked[0]
    if best[1] >= 0.6 and (len(ranked) == 1 or ranked[1][1] <= best[1] - 0.3):
        return best[0]
    return None


def start_anchor(stream, home, camera, height_cm=0.0, sd_cm=None):
    """'The log starts at this home': a full-pose anchor at the stream's first sample."""
    if not len(stream):
        return None
    x, y, hd = home_floor_pose(home, camera, height_cm)
    ppc = camera.px_per_cm(x, y, height_cm)
    sd = sd_cm if sd_cm is not None else max(0.2, float(home.get("sd_px", 1.0)) / max(ppc, 1e-6))
    return Anchor(float(stream.t[0]), x, y, hd, sd_cm=sd, sd_deg=float(home.get("sd_deg", 1.0)), kind="start",
                  note="starts at " + home["name"])


def events_to_anchors(events, homes, camera, height_cm=0.0, kinds=("docked",), owner_only=True):
    """Outside-data events that say 'I am at my home now' (an Alvik's dock.csv 'docked') -> {name:
    [Anchor]}: the home whose owner is that name (one each), else nothing (owner_only)."""
    own = {h.get("owner"): h for h in homes if h.get("owner") and h.get("kind") == "park"}
    out = {}
    for e in events:
        if e.get("kind") not in kinds:
            continue
        h = own.get(e.get("name"))
        if h is None:
            continue
        x, y, hd = home_floor_pose(h, camera, height_cm)
        ppc = camera.px_per_cm(x, y, height_cm)
        out.setdefault(e["name"], []).append(Anchor(e["t"], x, y, hd, sd_cm=max(0.2, float(h.get("sd_px", 1.0)) / ppc),
                                                    sd_deg=float(h.get("sd_deg", 1.0)), kind="home",
                                                    note="%s (log: docked)" % h["name"]))
    return out


# =============================================================================== the Alvik dock preset
def _dock_params():
    here = os.path.dirname(os.path.abspath(__file__))
    dock = os.path.join(os.path.dirname(os.path.dirname(here)), "hardware", "dock")
    if dock not in sys.path:
        sys.path.insert(0, dock)
    import dock_params  # noqa: E402
    return dock_params


def alvik_dock_geometry(version=2):
    """Dock numbers in cm, dock frame (x towards the back wall, 0 at the wall's inner face, y left):
    the parked robot's centre and facing, and (version 2) the post stations' robot poses."""
    D = _dock_params()
    park_x = D.DOCKED_AXLE_X_MM / 10.0     # the axle = the body's middle (axle to front 47.6, length 95.25 mm)
    g = dict(wall_h_cm=D.WALL_H_MM / 10.0, park=(park_x, 0.0, 0.0), mouth_x_cm=D.MOUTH_X_MM / 10.0,
             pocket_w_cm=D.POCKET_W_MM / 10.0, stations=[])
    if version >= 2:
        try:
            import build_v2 as B2
            st = B2.stations()
        except Exception:      # cadquery-free fallback: the same numbers as build_v2.py
            side_y = D.POCKET_W_MM / 2 + D.WALL_T_MM + 10.0
            st = [("back", None, (-1.0, 0.0), 110.0), ("left", None, (0.0, -1.0), side_y),
                  ("right", None, (0.0, 1.0), -side_y)]
        back = (D.BUMP_AHEAD_MM + max(0.0, D.SW_GAP_MM)) / 10.0
        for name, _tips, (ax, ay), line in st:
            c = 50.0 / 10.0 if name in ("left", "right") else 0.0      # side stations centred on the head
            if name == "back":
                tip = (line / 10.0, 0.0)
            else:
                tip = (c, line / 10.0)
            g["stations"].append((name, tip[0] - ax * back, tip[1] - ay * back, math.degrees(math.atan2(ay, ax))))
    return g


def alvik_dock_homes(name, camera_name, camera, wall_px, mouth_px, owner=None, version=2, robot_height_cm=4.0,
                     radius_cm=3.0, stations=True):
    """The Alvik dock as home positions, from two clicks: wall_px = the middle of the back wall's
    inner top edge, mouth_px = the middle of the funnel's mouth (both in the camera's picture).
    Returns [park home] (+ three station homes for dock version 2)."""
    g = alvik_dock_geometry(version)
    wx, wy = camera.to_floor(float(wall_px[0]), float(wall_px[1]), g["wall_h_cm"])
    mx, my = camera.to_floor(float(mouth_px[0]), float(mouth_px[1]), g["wall_h_cm"])
    facing = math.degrees(math.atan2(wy - my, wx - mx))
    return alvik_dock_homes_floor(name, camera_name, camera, wx, wy, facing, owner, version, robot_height_cm,
                                  radius_cm, stations)


def alvik_dock_homes_floor(name, camera_name, camera, wall_x_cm, wall_y_cm, heading, owner=None, version=2,
                           robot_height_cm=4.0, radius_cm=3.0, stations=True):
    """The same from the fleet map's home_docks entry (x, y = middle of the back wall's inner face,
    heading = the way a docked robot faces; docs/data-formats.md / hardware/dock/README.md)."""
    g = alvik_dock_geometry(version)
    c, s = math.cos(math.radians(heading)), math.sin(math.radians(heading))

    def to_floor(dx, dy):
        return wall_x_cm + c * dx - s * dy, wall_y_cm + s * dx + c * dy

    px, py = to_floor(g["park"][0], g["park"][1])
    out = [home_from_floor(name, camera_name, camera, px, py, heading, robot_height_cm, radius_cm, kind="park",
                           owner=owner, cls="alvik", sd_px=0.3, sd_deg=0.5, preset="alvik_dock_v%d" % version)]
    if stations and version >= 2:
        for sname, sx, sy, sh in g["stations"]:
            fx, fy = to_floor(sx, sy)
            out.append(home_from_floor("%s %s station" % (name, sname), camera_name, camera, fx, fy,
                                       (heading + sh) % 360.0, robot_height_cm, 1.5, kind="station", cls="alvik",
                                       sd_px=1.0, sd_deg=2.0, preset="alvik_dock_v%d" % version,
                                       note="a stop by a post: the robot only touches it briefly; sideways within about 0.5 cm"))
    return out


def alvik_docks_from_map(map_data, camera_name, camera, owners=None, version=2, **kw):
    """Every home_docks entry of a fleet map.json dict -> homes (owners {dock name: robot name})."""
    out = []
    for d in map_data.get("home_docks") or []:
        out += alvik_dock_homes_floor(d["name"], camera_name, camera, float(d["x"]), float(d["y"]),
                                      float(d.get("heading", 0.0)), (owners or {}).get(d["name"]), version, **kw)
    return out
