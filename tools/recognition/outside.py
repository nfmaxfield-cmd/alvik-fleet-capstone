"""Outside data: positions, headings and events that come from somewhere other than the camera
(a robot's own log, a robot arm's controller, a conveyor's encoder), on the shared clock t_wall,
turned into draft labels in the camera picture and into votes on the tracker's split branches.

Generic on purpose: nothing here knows about Alviks. A plug-in reads one kind of log folder and
returns an OutsideData (one Stream per object); tools/recognition/robotlogs.py is the Alvik one,
and GenericCSV below reads a plain positions file. Everything after the plug-in is the same for
every project.

Frames
------
A stream is either in the FLOOR frame already (x, y cm on the floor, the floor calibration's
axes; heading 0 = +x, counter-clockwise positive, as docs/data-formats.md) or in the object's OWN
frame: dead reckoning that starts wherever the object was (a robot's wheels and gyro). An own
frame stream is stored as raw path length (cumulative signed cm, forward positive) and raw
heading (deg, unwrapped), so the path can be re-integrated with corrections:

    h(t) = g * (heading(t) - heading(t0)) - d * (t - t0)         g: gyro scale, d: drift deg/s
    q(t) = sum  s * d_dist * (cos h, sin h)                         s: wheel (distance) scale

and placed on the floor by a rotation alpha and a translation P:  floor = P + R(alpha) q(t).

Alignment (Aligner)
-------------------
Anchors are moments where the floor pose is known: a home position (full pose, very exact), a
label the person confirmed (position; heading too if they set one), the direction of a forward
stretch seen in the detections (heading only). For each moment t the aligner solves a weighted
least-squares rotation + translation from ALL anchors of that stretch of log, each weighted by
how much the dead reckoning can have drifted between the anchor and t (time, distance driven,
angle turned). So an anchor dominates near itself (the pose is re-aligned at every confirmation
and every home visit, and drift is corrected there), and between two anchors the answer is a
blend of both. The known corrections (the plug-in's priors: for an Alvik the gyro and wheel scales
measured on real footage) are used as they are by default; fit_calibration() replaces them with
g, d and s fitted to the anchors (least squares with a robust loss, the priors as soft limits) only
when the anchors are many and spread over enough driving and turning, the fitted numbers stay within
3 sd of the priors, and the fit predicts the anchors left out in turn better than the priors do (on
roam1, Oct 21, a free fit gave gyro 1.29 where 1.07 is measured, and placed the labels worse).
After a kept fit the drift allowances shrink.

Confidence falls with the position uncertainty: score = exp(-0.5 (sd_cm / ref_cm)^2), ref_cm
about a quarter of a robot (2.5 cm by default).

Clocks: every time here is t_wall (the Pi clock of the recording's frames.csv). A plug-in is
responsible for converting its own clock (LinearClock helps); estimate_time_offset() can find a
leftover offset by matching the stream's speed with a detection track.

Integration points (other lanes wire these; this module edits nothing else):
  - studio / app:  draft_labels(...) -> {frame: [label dicts]} with source "log" (app.SOURCES)
    and status "pending", same keys as the labelling tool's objects plus name, x_cm, y_cm,
    sd_cm, heading_sd_deg, outside (plug-in id). Anchors from the person's labels:
    anchors_from_labels(); home visits: homes.anchors_from_visits().
  - tracker splits: score_branches(aligner, branches, camera, height_cm) -> per branch p (sums
    to 1 over the branches), mean error and n; score_candidates() for one moment.
  - live use (roll call): match_names(streams, tracks) pairs log names with camera tracks by
    their speed over time.
"""
import csv
import math
import os

import numpy as np

SOURCE = "log"             # label source name for draft labels (tools/label/app.py SOURCES has "log")
REF_CM = 2.5               # sd at which the score is exp(-1/2) = 0.61: about a quarter of a robot
HEADING_KNOWN_SD_DEG = 12.0  # heading sd under which a draft label's facing is shown as known


# =============================================================================== settings
class Settings:
    """How fast dead reckoning is allowed to drift away from an anchor, before and after the
    calibration fit (g, d, s fitted). Per-plug-in defaults; every number can be overridden.

    q_drift_dps   heading error growth per second (gyro drift left after correction), deg/s
    q_turn        heading error per degree of NET turn (gyro scale error left), fraction
    q_turn_abs    heading error per degree turned either way (random gyro noise), fraction
    q_scale       distance error per cm driven (wheel scale error left), fraction
    q_slip        random wheel slip, cm per sqrt(cm driven)
    Before the fit (raw), q_drift_dps, q_turn and q_scale are taken from the stream's priors' sds
    (the plug-in knows how well its numbers are known) unless given here.
    """

    def __init__(self, **kw):
        self.raw = dict(q_drift_dps=None, q_turn=None, q_turn_abs=0.005, q_scale=None, q_slip=0.15)
        # after the fit: about 3x what the synthetic sessions need (tests/synth_logs.py), as real
        # floors slip more; tune_allowances() replaces this guess with a measured factor
        self.fitted = dict(q_drift_dps=0.03, q_turn=0.005, q_turn_abs=0.002, q_scale=0.005, q_slip=0.08)
        self.ref_cm = REF_CM
        self.heading_known_sd_deg = HEADING_KNOWN_SD_DEG
        self.hold_s = 3.0          # draft labels this far before the first / after the last sample
        self.anchor_hold_s = 120.0  # anchors this far outside the log are moved to its edge (if still there)
        self.min_score = 0.05      # draft labels below this score are left out
        self.floor_sd_cm = 1.0     # sd of a floor-frame stream's own positions
        self.floor_sd_deg = 3.0
        self.label_sd_px = 1.5     # click accuracy of a confirmed label (per axis)
        self.label_heading_sd_deg = 10.0
        self.calib_sd_cm = 0.5     # floor calibration error
        self.priors = dict(g=1.0, g_sd=0.1, d=0.0, d_sd=0.5, s=1.0, s_sd=0.05)
        for k, v in kw.items():
            if k in ("raw", "fitted", "priors"):
                getattr(self, k).update(v)
            elif hasattr(self, k):
                setattr(self, k, v)
            else:
                raise TypeError("unknown setting %r" % k)


# =============================================================================== data
class Stream:
    """One object's outside data on the shared clock.

    frame "own":   t (s, t_wall), dist (cumulative signed path, cm), heading (deg, unwrapped, own
                   frame, counter-clockwise positive), segment (int per sample: a new number where
                   the path stops being continuous: lifted, rebooted), valid (False: don't place,
                   e.g. lifted), still (True: not moving). sign_known False: the heading's sign is
                   not checked (the fit tries both).
    frame "floor": t, x, y (cm), heading (deg or nan).
    """

    def __init__(self, name, t, dist=None, heading=None, x=None, y=None, frame="own", segment=None,
                 valid=None, still=None, cls=None, sign_known=True, priors=None, meta=None):
        self.name = str(name)
        self.frame = frame
        self.cls = cls
        self.sign_known = bool(sign_known)
        self.priors = dict(priors or {})
        self.meta = dict(meta or {})
        t = np.asarray(t, float)
        o = np.argsort(t, kind="stable")
        self.t = t[o]
        n = len(self.t)

        def arr(a, fill):
            return np.full(n, fill, float) if a is None else np.asarray(a, float)[o]

        self.heading = arr(heading, np.nan)
        if frame == "own":
            self.dist = arr(dist, 0.0)
            self.x = self.y = None
        elif frame == "floor":
            self.x, self.y = arr(x, np.nan), arr(y, np.nan)
            self.dist = None
        else:
            raise ValueError("frame must be 'own' or 'floor'")
        self.segment = np.zeros(n, int) if segment is None else np.asarray(segment, int)[o]
        self.valid = np.ones(n, bool) if valid is None else np.asarray(valid, bool)[o]
        self.still = np.zeros(n, bool) if still is None else np.asarray(still, bool)[o]
        if frame == "own" and n:
            dd = np.abs(np.diff(self.dist, prepend=self.dist[0]))
            hh = np.abs(np.diff(np.nan_to_num(self.heading), prepend=np.nan_to_num(self.heading[0])))
            self.path_abs = np.cumsum(dd)     # cm driven, any direction
            self.turn_abs = np.cumsum(hh)     # degrees turned, any direction
        else:
            self.path_abs = self.turn_abs = np.zeros(n)

    def __len__(self):
        return len(self.t)

    @property
    def span(self):
        return (float(self.t[0]), float(self.t[-1])) if len(self.t) else (math.nan, math.nan)

    def speed(self, ts, window_s=0.5):
        """Speed (cm/s, any direction) around times ts, from the stream itself."""
        ts = np.asarray(ts, float)
        if len(self.t) < 2:
            return np.full(len(ts), np.nan)
        if self.frame == "own":
            a = np.interp(ts - window_s / 2, self.t, self.path_abs)
            b = np.interp(ts + window_s / 2, self.t, self.path_abs)
            return (b - a) / window_s
        xa, xb = np.interp(ts - window_s / 2, self.t, self.x), np.interp(ts + window_s / 2, self.t, self.x)
        ya, yb = np.interp(ts - window_s / 2, self.t, self.y), np.interp(ts + window_s / 2, self.t, self.y)
        return np.hypot(xb - xa, yb - ya) / window_s


class OutsideData:
    """What a plug-in returns: streams by name, events, plain-words notes, how the clock was set."""

    def __init__(self, plugin="", streams=None, events=None, notes=None, clock="", folder=None, clock_known=True,
                 to_wall=None, time_words=""):
        self.plugin = plugin
        # the data's own time (as written in its files) -> t_wall, before any offset: lets a person line the
        # data up by a moment ("at this frame the data says 12.5 s"); time_words names that time in plain words
        self.to_wall = to_wall
        self.time_words = time_words
        self.clock_known = bool(clock_known)    # False: the plug-in had no way to tie its clock to the video's
        self.streams = dict(streams or {})
        self.events = list(events or [])     # dicts: t (t_wall), kind, name, ... (plug-in specific extras)
        self.notes = list(notes or [])
        self.clock = clock
        self.folder = folder

    def names(self):
        return sorted(self.streams)

    def summary(self):
        parts = ["%s: %d samples, %.0f s" % (n, len(s), (s.span[1] - s.span[0]) if len(s) else 0)
                 for n, s in sorted(self.streams.items())]
        return "%s: %s%s" % (self.plugin or "outside data", "; ".join(parts) or "nothing read",
                             (". " + self.clock) if self.clock else "")


class Anchor:
    """A moment where an object's floor pose is known. heading None = position only; x None =
    heading only. kind: home, label, start, motion, other. Positions cm, headings deg (floor)."""

    def __init__(self, t, x=None, y=None, heading=None, sd_cm=1.0, sd_deg=5.0, kind="label", note="", frame=None):
        self.t = float(t)
        self.x = None if x is None else float(x)
        self.y = None if y is None else float(y)
        self.heading = None if heading is None else float(heading)
        self.sd_cm = float(sd_cm)
        self.sd_deg = float(sd_deg)
        self.kind = kind
        self.note = note
        self.frame = frame

    @property
    def has_pos(self):
        return self.x is not None and self.y is not None

    def to_dict(self):
        return dict(t=self.t, x=self.x, y=self.y, heading=self.heading, sd_cm=self.sd_cm, sd_deg=self.sd_deg,
                    kind=self.kind, note=self.note, frame=self.frame)

    def __repr__(self):
        return "Anchor(%s t=%.2f pos=%s heading=%s)" % (self.kind, self.t, (self.x, self.y) if self.has_pos else None,
                                                        self.heading)


class LinearClock:
    """source time -> t_wall = a + b * source time (b = 1 when no drift is known)."""

    def __init__(self, a=0.0, b=1.0, text=""):
        self.a, self.b, self.text = float(a), float(b), text

    def __call__(self, ts):
        return self.a + self.b * np.asarray(ts, float)

    def then(self, other):
        """This clock followed by another (e.g. robot -> laptop, then laptop -> Pi)."""
        return LinearClock(other.a + other.b * self.a, other.b * self.b, "; ".join(x for x in (self.text, other.text) if x))


# =============================================================================== plug-ins
class Plugin:
    """Reads one kind of outside data. Subclass, set id / title / description, implement
    detect(folder) and read(folder, **options) -> OutsideData, and call register()."""
    id = ""
    title = ""
    description = ""

    def detect(self, folder):
        return False

    def read(self, folder, **options):
        raise NotImplementedError


_PLUGINS = {}


def register(plugin_cls):
    _PLUGINS[plugin_cls.id] = plugin_cls
    return plugin_cls


def plugins():
    """[(id, title, description)] of every registered plug-in."""
    _load_builtin()
    return [(p.id, p.title, p.description) for p in _PLUGINS.values()]


def _load_builtin():
    if "alvik" not in _PLUGINS:
        try:
            from . import robotlogs  # noqa: F401
        except ImportError:
            try:
                import robotlogs  # noqa: F401
            except ImportError:
                pass


def read_outside(path, plugin=None, **options):
    """Read outside data from path with the named plug-in, or the first one that recognises it."""
    _load_builtin()
    if plugin:
        if plugin not in _PLUGINS:
            raise KeyError("no outside-data plug-in %r (have: %s)" % (plugin, ", ".join(sorted(_PLUGINS))))
        return _PLUGINS[plugin]().read(path, **options)
    for p in _PLUGINS.values():
        inst = p()
        if inst.detect(path):
            return inst.read(path, **options)
    raise ValueError("%s: no outside-data plug-in recognises this (have: %s)" % (path, ", ".join(sorted(_PLUGINS))))


@register
class GenericCSV(Plugin):
    """positions.csv: t,name,x,y[,heading][,frame][,still]. t on the shared clock (t_wall), or give
    offset_s (t_wall = t + offset_s). frame 'floor' (default: positions already in floor cm, e.g. a
    robot arm's controller after a camera-to-arm calibration) or 'own' (dead reckoning from its
    own start: then x, y are its own path and get aligned like a robot log)."""
    id = "csv"
    title = "Positions file (CSV)"
    description = "A CSV of t, name, x, y, heading on the shared clock: any machine that knows where things are."
    FILE = "positions.csv"

    def detect(self, folder):
        p = folder if folder.endswith(".csv") else os.path.join(folder, self.FILE)
        if not os.path.isfile(p):
            return False
        with open(p, newline="") as f:
            head = next(csv.reader(f), [])
        return {"t", "name", "x", "y"} <= {h.strip() for h in head}

    def read(self, folder, offset_s=0.0, **options):
        p = folder if folder.endswith(".csv") else os.path.join(folder, self.FILE)
        rows, bad = {}, 0
        with open(p, newline="") as f:
            for r in csv.DictReader(f):
                try:
                    t = float(r["t"]) + offset_s
                    x, y = float(r["x"]), float(r["y"])
                except (TypeError, ValueError, KeyError):
                    bad += 1
                    continue
                h = _num(r.get("heading"))
                rows.setdefault(r["name"].strip(), []).append((t, x, y, h, (r.get("frame") or "floor").strip(),
                                                               (r.get("still") or "").strip() in ("1", "true", "True")))
        streams = {}
        for name, rs in rows.items():
            rs.sort()
            t = [r[0] for r in rs]
            frame = "own" if rs[0][4] == "own" else "floor"
            h = [r[3] for r in rs]
            still = [r[5] for r in rs]
            if frame == "floor":
                streams[name] = Stream(name, t, x=[r[1] for r in rs], y=[r[2] for r in rs], heading=h,
                                       frame="floor", still=still)
            else:
                streams[name] = stream_from_own_xy(name, t, [r[1] for r in rs], [r[2] for r in rs], h, still=still)
        notes = ["%d rows could not be read and were skipped" % bad] if bad else []
        return OutsideData(self.id, streams, [], notes, "times as given%s" % (
            " %+.3f s" % offset_s if offset_s else ""), folder, to_wall=lambda t: t,
            time_words="the t column of the positions file (seconds)")


def _num(v):
    try:
        f = float(v)
        return f if math.isfinite(f) else math.nan
    except (TypeError, ValueError):
        return math.nan


def stream_from_own_xy(name, t, x, y, heading=None, still=None, segment=None, valid=None, **kw):
    """An own-frame stream from dead-reckoned x, y (and heading, deg; None = from the path)."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    dx, dy = np.diff(x, prepend=x[:1]), np.diff(y, prepend=y[:1])
    if heading is None or np.all(~np.isfinite(np.asarray(heading, float))):
        h = np.degrees(np.arctan2(dy, dx))
        h[np.hypot(dx, dy) < 1e-6] = np.nan
        h = _ffill(h)
        sgn = np.ones(len(x))
    else:
        h = _ffill(np.asarray(heading, float))
        sgn = np.sign(dx * np.cos(np.radians(h)) + dy * np.sin(np.radians(h)))
        sgn[sgn == 0] = 1
    dist = np.cumsum(np.hypot(dx, dy) * sgn)
    hu = np.degrees(np.unwrap(np.radians(np.nan_to_num(h))))
    return Stream(name, t, dist=dist, heading=hu, frame="own", still=still, segment=segment, valid=valid, **kw)


def _ffill(a):
    a = np.array(a, float)
    ok = np.isfinite(a)
    if not ok.any():
        return np.zeros_like(a)
    idx = np.where(ok, np.arange(len(a)), 0)
    np.maximum.accumulate(idx, out=idx)
    out = a[idx]
    first = np.argmax(ok)
    out[:first] = a[first]
    return out


# =============================================================================== camera helpers
class HomographyMap:
    """Floor <-> image with only a floor homography (floor_calibration.json without a camera
    height): z is ignored unless height_cm and nadir are known, then a point z up is moved along
    the camera ray (the parallax the fusion uses)."""

    def __init__(self, H, image_size=(640, 480), height_cm=None, nadir=None):
        self.H = np.asarray(H, float)
        self.Hinv = np.linalg.inv(self.H)
        self.image_size = tuple(image_size)
        self.height_cm = height_cm
        self.nadir = None if nadir is None else np.asarray(nadir, float)

    @classmethod
    def load(cls, path):
        import json
        with open(path) as f:
            d = json.load(f)
        for k in ("H_floor_to_px", "homography_floor_cm_to_px", "H_floor_to_image", "H", "homography", "matrix"):
            if k in d:
                cam = d.get("camera") or {}
                return cls(d[k], d.get("image_size") or (640, 480), cam.get("height_cm"), cam.get("nadir_floor_cm"))
        raise ValueError("%s: no homography" % path)

    def _par(self, x, y, z, inverse=False):
        if not z or self.height_cm is None or self.nadir is None:
            return x, y
        k = self.height_cm / (self.height_cm - z)
        if inverse:
            k = 1.0 / k
        return self.nadir[0] + (x - self.nadir[0]) * k, self.nadir[1] + (y - self.nadir[1]) * k

    def project(self, xyz):
        p = np.asarray(xyz, float)
        single = p.ndim == 1
        p = p.reshape(-1, 3)
        x, y = self._par(p[:, 0], p[:, 1], p[:, 2] if p.shape[1] > 2 else 0.0)
        w = self.H[2, 0] * x + self.H[2, 1] * y + self.H[2, 2]
        u = (self.H[0, 0] * x + self.H[0, 1] * y + self.H[0, 2]) / w
        v = (self.H[1, 0] * x + self.H[1, 1] * y + self.H[1, 2]) / w
        if single:
            return float(u[0]), float(v[0])
        return u, v

    def to_floor(self, u, v, z=0.0):
        single = np.ndim(u) == 0
        u, v = np.atleast_1d(np.asarray(u, float)), np.atleast_1d(np.asarray(v, float))
        H = self.Hinv
        w = H[2, 0] * u + H[2, 1] * v + H[2, 2]
        x = (H[0, 0] * u + H[0, 1] * v + H[0, 2]) / w
        y = (H[1, 0] * u + H[1, 1] * v + H[1, 2]) / w
        x, y = self._par(x, y, z, inverse=True)
        if single:
            return float(x[0]), float(y[0])
        return x, y

    def in_image(self, u, v, margin=0.0):
        W, Hh = self.image_size
        u, v = np.asarray(u, float), np.asarray(v, float)
        return (u >= -0.5 + margin) & (u <= W - 0.5 - margin) & (v >= -0.5 + margin) & (v <= Hh - 0.5 - margin) & \
            np.isfinite(u) & np.isfinite(v)

    def px_per_cm(self, x, y, z=0.0):
        u0, v0 = self.project((x, y, z))
        u1, v1 = self.project((x + 1.0, y, z))
        u2, v2 = self.project((x, y + 1.0, z))
        return 0.5 * (math.hypot(u1 - u0, v1 - v0) + math.hypot(u2 - u0, v2 - v0))


def load_floor_map(path):
    """A camera for floor <-> image: a camera model file (cameras/<name>.json), else a
    floor_calibration.json as a full camera model (needs camera.height_cm for a straight-down
    view), else that file's homography alone."""
    import json
    try:
        from .cameras import CameraModel, CalibrationError
    except ImportError:
        from cameras import CameraModel, CalibrationError
    with open(path) as f:
        d = json.load(f)
    if "K" in d and "rvec" in d:
        return CameraModel.from_dict(d)
    try:
        return CameraModel.from_floor_calibration(path)
    except (CalibrationError, ValueError, KeyError, np.linalg.LinAlgError):
        return HomographyMap.load(path)


def image_heading(cam, x, y, z, heading_floor_deg, lead_cm=3.0):
    """Floor heading at floor point -> image heading (labelling convention: 0 = image right,
    counter-clockwise on screen)."""
    h = math.radians(heading_floor_deg)
    u0, v0 = cam.project((x, y, z))
    u1, v1 = cam.project((x + lead_cm * math.cos(h), y + lead_cm * math.sin(h), z))
    return math.degrees(math.atan2(-(v1 - v0), u1 - u0)) % 360.0


def floor_heading(cam, u, v, z, heading_img_deg, step_px=4.0):
    """Image heading (labelling convention) at pixel (u, v) -> floor heading (deg)."""
    h = math.radians(heading_img_deg)
    x0, y0 = cam.to_floor(u, v, z)
    x1, y1 = cam.to_floor(u + step_px * math.cos(h), v - step_px * math.sin(h), z)
    return math.degrees(math.atan2(y1 - y0, x1 - x0)) % 360.0


def _wrap180(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


# =============================================================================== alignment
class Aligner:
    """Places one stream on the floor from its anchors (see the module docstring)."""

    def __init__(self, stream, settings=None):
        self.stream = stream
        self.settings = settings or Settings()
        pri = dict(self.settings.priors)
        pri.update(stream.priors)
        self.priors = pri
        self.anchors = []
        self.calib = dict(g=pri["g"], d=pri["d"], s=pri["s"], fitted=False, rms_cm=None, n=0)
        self.calib_note = None      # fit_calibration(): what it did with the corrections, in plain words
        self.calib_refused = None   # fit_calibration(): a fit it did not use, and why
        self.notes = []
        self.allow_scale = 1.0     # tune_allowances() sets it from the person's labels
        self._path = None

    # -- anchors
    def add(self, *anchors):
        for a in anchors:
            if a is None:
                continue
            self.anchors.append(a)
        self.anchors.sort(key=lambda a: a.t)
        return self

    def clear(self, kind=None):
        self.anchors = [a for a in self.anchors if kind is not None and a.kind != kind]

    # -- the path with the current corrections
    def _integrate(self, g, d, s):
        st = self.stream
        if st.frame == "floor" or not len(st):
            return None
        h = g * (st.heading - st.heading[0]) - d * (st.t - st.t[0])
        dd = np.diff(st.dist) * s
        hm = np.radians((h[1:] + h[:-1]) / 2)
        qx = np.concatenate([[0.0], np.cumsum(dd * np.cos(hm))])
        qy = np.concatenate([[0.0], np.cumsum(dd * np.sin(hm))])
        return qx, qy, h

    def path(self):
        if self._path is None:
            self._path = self._integrate(self.calib["g"], self.calib["d"], self.calib["s"])
        return self._path

    def _at(self, ts, arrays):
        st = self.stream
        return [np.interp(ts, st.t, a) for a in arrays]

    def _seg_at(self, ts):
        st = self.stream
        i = np.clip(np.searchsorted(st.t, ts), 0, len(st.t) - 1)
        j = np.clip(i - 1, 0, len(st.t) - 1)
        near = np.where(np.abs(st.t[i] - ts) < np.abs(st.t[j] - ts), i, j)
        return st.segment[near], near

    def _anchor_time(self, a):
        """An anchor's time moved onto the log: inside it, or held at its edge when within
        anchor_hold_s (the object was parked before its first logged move / after its last: a
        robot logs only while it runs)."""
        t0, t1 = self.stream.span
        if t0 <= a.t <= t1:
            return a.t
        hold = self.settings.anchor_hold_s
        if t0 - hold <= a.t < t0:
            return t0
        if t1 < a.t <= t1 + hold:
            return t1
        return None

    # -- drift allowance between two moments
    def _allow(self, ta, ts):
        """(pos var cm^2, heading var rad^2) of the dead reckoning between anchor times ta and query
        times ts: shape (len(ts), len(ta)). Heading error grows with time (drift), net turn (scale)
        and turning either way (noise); position error with the straight-line distance between the
        two points of the path (a heading or scale error moves a point by that lever: on a path that
        comes back it cancels) plus random slip along the path driven."""
        q = self.allowances()
        st = self.stream
        ta = np.asarray(ta, float)
        qx, qy, _h = self.path()
        C_t, A_t, N_t, X_t, Y_t = self._at(ts, [st.path_abs, st.turn_abs, st.heading, qx, qy])
        C_a, A_a, N_a, X_a, Y_a = self._at(ta, [st.path_abs, st.turn_abs, st.heading, qx, qy])
        dt = np.abs(ts[:, None] - ta[None, :])
        D = np.abs(C_t[:, None] - C_a[None, :])
        A = np.abs(A_t[:, None] - A_a[None, :])
        N = np.abs(N_t[:, None] - N_a[None, :])
        L2 = (X_t[:, None] - X_a[None, :]) ** 2 + (Y_t[:, None] - Y_a[None, :]) ** 2
        sh = np.radians(np.sqrt((q["q_drift_dps"] * dt) ** 2 + (q["q_turn"] * N) ** 2 + (q["q_turn_abs"] * A) ** 2))
        sp2 = (q["q_scale"] ** 2 + sh ** 2) * L2 + q["q_slip"] ** 2 * D
        return sp2, sh ** 2

    def allowances(self):
        """The drift allowances in use (Settings.raw with the priors filled in, or Settings.fitted)."""
        if self.calib["fitted"]:
            q = dict(self.settings.fitted)
        else:
            q = dict(self.settings.raw)
            pr = self.priors
            if q["q_drift_dps"] is None:
                q["q_drift_dps"] = pr["d_sd"]
            if q["q_turn"] is None:
                q["q_turn"] = pr["g_sd"] / max(abs(pr["g"]), 1e-6)
            if q["q_scale"] is None:
                q["q_scale"] = pr["s_sd"] / max(pr["s"], 1e-6)
        k = self.allow_scale
        return {n: v * (k if n != "q_slip" else math.sqrt(k)) for n, v in q.items()}

    def pose_at(self, ts, causal=False):
        """Floor pose at times ts (t_wall): dict of arrays x, y, heading (deg), sd_cm, sd_deg, ok.
        causal=True uses only anchors at or before each time (for live use)."""
        ts = np.atleast_1d(np.asarray(ts, float))
        n = len(ts)
        out = dict(x=np.full(n, np.nan), y=np.full(n, np.nan), heading=np.full(n, np.nan),
                   sd_cm=np.full(n, np.inf), sd_deg=np.full(n, np.inf), ok=np.zeros(n, bool))
        st = self.stream
        if not len(st):
            return out
        t0, t1 = st.span
        inside = (ts >= t0 - self.settings.hold_s) & (ts <= t1 + self.settings.hold_s)
        tq = np.clip(ts, t0, t1)
        seg_q, near = self._seg_at(tq)
        valid = inside & st.valid[near]
        if st.frame == "floor":
            x, y = self._at(tq, [st.x, st.y])
            h = _interp_angle(tq, st.t, st.heading)
            out.update(x=np.where(valid, x, np.nan), y=np.where(valid, y, np.nan), heading=np.where(valid, h, np.nan),
                       sd_cm=np.where(valid, self.settings.floor_sd_cm, np.inf),
                       sd_deg=np.where(valid & np.isfinite(h), self.settings.floor_sd_deg, np.inf), ok=valid)
            return out
        qx, qy, h = self.path()
        Qx, Qy, Hq = self._at(tq, [qx, qy, h])
        anchors = [(a, ta) for a in self.anchors for ta in [self._anchor_time(a)] if ta is not None]
        if not anchors:
            return out
        ta = np.array([x[1] for x in anchors])
        seg_a, _ = self._seg_at(ta)
        Ax, Ay, Ah = self._at(ta, [qx, qy, h])
        sp2, sh2 = self._allow(ta, tq)
        same = seg_q[:, None] == seg_a[None, :]
        if causal:
            same &= ta[None, :] <= ts[:, None] + 1e-9
        haspos = np.array([a.has_pos for a, _ in anchors])
        hashead = np.array([a.heading is not None for a, _ in anchors])
        px = np.array([a.x if a.has_pos else 0.0 for a, _ in anchors])
        py = np.array([a.y if a.has_pos else 0.0 for a, _ in anchors])
        th = np.radians(np.array([a.heading if a.heading is not None else 0.0 for a, _ in anchors]))
        sdp = np.array([a.sd_cm for a, _ in anchors])
        sdh = np.radians(np.array([a.sd_deg for a, _ in anchors]))
        W = np.where(same & haspos[None, :], 1.0 / (sdp[None, :] ** 2 + sp2), 0.0)
        V = np.where(same & hashead[None, :], 1.0 / (sdh[None, :] ** 2 + sh2), 0.0)
        Sw = W.sum(1)
        Sv = V.sum(1)
        with np.errstate(invalid="ignore", divide="ignore"):
            qbx, qby = (W @ Ax) / Sw, (W @ Ay) / Sw
            pbx, pby = (W @ px) / Sw, (W @ py) / Sw
            C = W @ (Ax * px + Ay * py) - Sw * (qbx * pbx + qby * pby)
            S = W @ (Ax * py - Ay * px) - Sw * (qbx * pby - qby * pbx)
        C, S = np.nan_to_num(C), np.nan_to_num(S)
        M = np.hypot(C, S)
        phi = np.arctan2(S, C)
        beta = th[None, :] - np.radians(Ah)[None, :]          # rotation each heading anchor implies
        # reference angle: the position fit if it carries more information, else the headings' mean
        hb = np.arctan2(V @ np.sin(th - np.radians(Ah)), V @ np.cos(th - np.radians(Ah)))
        ref = np.where(M >= Sv, phi, hb)
        dphi = _wrap_rad(phi - ref)
        dbeta = _wrap_rad(beta - ref[:, None])
        info = M + Sv
        with np.errstate(invalid="ignore", divide="ignore"):
            alpha = ref + (M * dphi + (V * dbeta).sum(1)) / info
            var_a = 1.0 / info
        ok = valid & (Sw > 0) & (info > 1.0 / math.radians(60.0) ** 2)
        ca, sa = np.cos(alpha), np.sin(alpha)
        rx, ry = Qx - np.nan_to_num(qbx), Qy - np.nan_to_num(qby)
        x = np.nan_to_num(pbx) + ca * rx - sa * ry
        y = np.nan_to_num(pby) + sa * rx + ca * ry
        with np.errstate(invalid="ignore", divide="ignore"):
            sd2 = 1.0 / Sw + var_a * (rx ** 2 + ry ** 2)
        head = (Hq + np.degrees(alpha)) % 360.0
        out.update(x=np.where(ok, x, np.nan), y=np.where(ok, y, np.nan), heading=np.where(ok, head, np.nan),
                   sd_cm=np.where(ok, np.sqrt(sd2), np.inf), sd_deg=np.where(ok, np.degrees(np.sqrt(var_a)), np.inf),
                   ok=ok)
        return out

    # -- calibration fit
    def _known_words(self):
        pr = self.priors
        return "gyro x%.3f, %swheels x%.3f%s" % (
            abs(pr["g"]), ("drift %+.2f deg/s, " % pr["d"]) if abs(pr["d"]) > 1e-9 else "", pr["s"],
            (" (" + pr["from"] + ")") if pr.get("from") else "")

    def _keep_known(self, why, sign=None):
        """Back to the known corrections (the priors), with a note in the studio's words: what (nothing
        changed, or only the heading's sign), where, why. sign: the heading's sign found by a fit, when it
        was not checked (the known gyro scale is used with it)."""
        pr = self.priors
        g = pr["g"] if sign is None else math.copysign(abs(pr["g"]), sign)
        self.calib.update(g=g, d=pr["d"], s=pr["s"], fitted=False, rms_cm=None, n=0)
        self._path = None
        if sign is not None and (g < 0) != (pr["g"] < 0):
            tail = (" The heading's sign was not checked in the log, and the fit to the anchors shows it is turned "
                    "the other way, so that is used (with the known gyro scale); Z (Undo) brings back the guesses "
                    "from before this Line up.")
        else:
            tail = " Nothing to undo: these were the starting values."
        self.calib_note = ("%s: the log's turn and distance corrections stay at the known values (%s): %s.%s" % (
            self.stream.name, self._known_words(), why, tail))
        self.notes.append(self.calib_note)
        return self.calib

    def _loo_median(self, g, d, s, fitted, block_s=5.0):
        """Median leave-out error (cm) of the position anchors with these corrections, and how many: each
        anchor is predicted from the anchors more than block_s away from it (one home visit gives several
        anchors seconds apart, and they would predict each other whatever the corrections)."""
        keep_calib, keep = dict(self.calib), list(self.anchors)
        self.calib.update(g=g, d=d, s=s, fitted=fitted)
        self._path = None
        errs = []
        try:
            for a in keep:
                if not a.has_pos:
                    continue
                self.anchors = [b for b in keep if abs(b.t - a.t) > block_s]
                p = self.pose_at(np.array([a.t]))
                if p["ok"][0]:
                    errs.append(math.hypot(p["x"][0] - a.x, p["y"][0] - a.y))
        finally:
            self.anchors = keep
            self.calib = keep_calib
            self._path = None
        return (float(np.median(errs)) if errs else math.nan), len(errs)

    def fit_calibration(self, min_pos=6, min_span_cm=100.0, min_turn_deg=360.0, band_sd=3.0, min_gain=0.05,
                        min_loo=5):
        """Fit gyro scale g, drift d and wheel scale s from the anchors (least squares, robust loss,
        priors). The known corrections (the priors) stay unless the fit has enough evidence (min_pos
        position anchors over min_span_cm of driving and min_turn_deg of turning), its numbers are
        within band_sd sd of the priors, and it lowers the median leave-out error of the anchors (each
        predicted without those within 5 s; at least min_loo predictable) by at least min_gain (a
        fraction). Two exact full poses (home visits) also count as enough evidence, for a fit within the
        band that cannot be checked by leaving out. When the heading's sign was not checked
        (stream.sign_known False) a wrong sign is far worse than any scale, so the fit runs on as few as 3
        anchors over 20 cm, and its sign is kept even when its numbers are not. Returns the calib dict
        (fitted True/False); the plain-words result is self.calib_note (also added to self.notes)."""
        from scipy.optimize import least_squares
        st = self.stream
        if st.frame != "own" or not len(st):
            return self.calib
        used = [(a, ta) for a in self.anchors for ta in [self._anchor_time(a)] if ta is not None]
        pos = [(a, ta) for a, ta in used if a.has_pos]
        # exact full poses (a home visit, a start at home: position AND heading, to a few mm) pin the turn and
        # the drift directly: two of them are enough evidence for a fit held within the known range
        exact = sum(1 for a, _ in pos if a.heading is not None and a.sd_cm <= 1.0 and a.sd_deg <= 2.0)
        few_ok = exact >= 2 or not st.sign_known
        if few_ok:
            min_pos, min_span_cm, min_turn_deg = min(min_pos, 3), min(min_span_cm, 20.0), 0.0
        need = ("fitting them needs %d labels or home visits over %.0f cm of driving and %.0f degrees of turning"
                % (min_pos, min_span_cm, min_turn_deg))
        if len(pos) < min_pos:
            return self._keep_known("%s; there are %d" % (need, len(pos)))
        ta_all = np.array([ta for _, ta in used])
        seg_all, _ = self._seg_at(ta_all)
        segs = sorted(set(seg_all.tolist()))
        tpos = [ta for _, ta in pos]
        Cpos = np.interp(tpos, st.t, st.path_abs)
        Tpos = np.interp(tpos, st.t, st.turn_abs)
        if Cpos.max() - Cpos.min() < min_span_cm or Tpos.max() - Tpos.min() < min_turn_deg:
            return self._keep_known("%s; they cover %.0f cm and %.0f degrees" % (
                need, Cpos.max() - Cpos.min(), Tpos.max() - Tpos.min()))
        pr = self.priors
        si = {s: k for k, s in enumerate(segs)}
        is_pos = np.array([a.has_pos for a, _ in used])
        is_head = np.array([a.heading is not None for a, _ in used])
        px = np.array([a.x if a.has_pos else 0 for a, _ in used])
        py = np.array([a.y if a.has_pos else 0 for a, _ in used])
        th = np.array([a.heading if a.heading is not None else 0 for a, _ in used])
        sdp = np.array([a.sd_cm for a, _ in used])
        sdh = np.array([a.sd_deg for a, _ in used])
        sk = np.array([si[s] for s in seg_all])
        q = self.settings.fitted
        # per-anchor slack for slip that no calibration removes (grows with the distance from the
        # segment's anchors' middle): keeps far anchors from dominating
        slack = q["q_slip"] * np.sqrt(np.maximum(np.interp(ta_all, st.t, st.path_abs) - st.path_abs[0], 1.0))

        g_prior = [pr["g"]]

        def resid(p):
            g, d, s = p[:3]
            qx, qy, h = self._integrate(g, d, s)
            Ax, Ay, Ah = self._at(ta_all, [qx, qy, h])
            P = p[3:].reshape(-1, 3)
            al = np.radians(P[sk, 2])
            ex = P[sk, 0] + np.cos(al) * Ax - np.sin(al) * Ay - px
            ey = P[sk, 1] + np.sin(al) * Ax + np.cos(al) * Ay - py
            sd = np.hypot(sdp, 0.5 * slack)
            eh = _wrap180(P[sk, 2] + Ah - th) / sdh
            r = [np.where(is_pos, ex / sd, 0.0), np.where(is_pos, ey / sd, 0.0), np.where(is_head, eh, 0.0),
                 [(g - g_prior[0]) / pr["g_sd"], (d - pr["d"]) / pr["d_sd"], (s - pr["s"]) / pr["s_sd"]]]
            return np.concatenate([np.ravel(x) for x in r])

        g_starts = [pr["g"]] if st.sign_known else [abs(pr["g"]), -abs(pr["g"])]
        best = None
        for g0 in g_starts:
            g_prior[0] = g0
            p0 = [g0, pr["d"], pr["s"]]
            self.calib.update(g=g0, d=pr["d"], s=pr["s"], fitted=False)
            self._path = None
            for s in segs:      # start each segment's rigid fit from the local alignment
                ts = ta_all[seg_all == s]
                pa = self.pose_at(np.array([ts.mean()]))
                qx, qy, h = self.path()
                if pa["ok"][0]:
                    Q = self._at(np.array([ts.mean()]), [qx, qy, h])
                    al = math.radians(pa["heading"][0] - Q[2][0])
                    Px = pa["x"][0] - (math.cos(al) * Q[0][0] - math.sin(al) * Q[1][0])
                    Py = pa["y"][0] - (math.sin(al) * Q[0][0] + math.cos(al) * Q[1][0])
                    p0 += [Px, Py, math.degrees(al)]
                else:
                    a0 = next(a for (a, t_), sg in zip(used, seg_all) if sg == s and a.has_pos)
                    p0 += [a0.x, a0.y, 0.0]
            try:
                r = least_squares(resid, np.array(p0, float), loss="soft_l1", f_scale=2.0, x_scale="jac", max_nfev=400)
            except (ValueError, np.linalg.LinAlgError) as e:
                self.notes.append("calibration fit failed: %s" % e)
                continue
            if best is None or r.cost < best.cost:
                best = r
        if best is None:
            return self._keep_known("the fit to the anchors did not work out")
        g, d, s = (float(v) for v in best.x[:3])
        got = "gyro x%.3f, drift %+.2f deg/s, wheels x%.3f" % (g, d, s)
        lo_g, hi_g = abs(pr["g"]) - band_sd * pr["g_sd"], abs(pr["g"]) + band_sd * pr["g_sd"]
        lo_d, hi_d = pr["d"] - band_sd * pr["d_sd"], pr["d"] + band_sd * pr["d_sd"]
        lo_s, hi_s = pr["s"] - band_sd * pr["s_sd"], pr["s"] + band_sd * pr["s_sd"]
        if not (lo_g <= abs(g) <= hi_g and lo_d <= d <= hi_d and lo_s <= s <= hi_s
                and 0.7 <= abs(g) <= 1.4 and abs(d) <= 3.0 and 0.8 <= s <= 1.25):
            self.calib_refused = dict(g=g, d=d, s=s, why="outside")
            return self._keep_known(sign=None if st.sign_known else g, why=
                "a fit to the %d anchors gave %s, outside what this robot can plausibly be (gyro %.2f to %.2f, "
                "drift %+.2f to %+.2f deg/s, wheels %.3f to %.3f), so it was not used" % (
                    len(pos), got, lo_g, hi_g, lo_d, hi_d, lo_s, hi_s))
        # the fit must place the anchors left out in turn better than the known values (with the sign the
        # fit found, when the sign was not checked)
        g_known = math.copysign(abs(pr["g"]), g)
        e_known, n_known = self._loo_median(g_known, pr["d"], pr["s"], False)
        e_fit, n_fit = self._loo_median(g, d, s, False)      # same allowances: only the corrections differ
        checked = min(n_fit, n_known) >= min_loo and math.isfinite(e_known) and math.isfinite(e_fit)
        if not checked and not few_ok:
            self.calib_refused = dict(g=g, d=d, s=s, why="few")
            return self._keep_known(sign=None if st.sign_known else g, why=
                "a fit to the %d anchors gave %s, but only %d of them can be checked by leaving each out in turn "
                "(needs %d), so it was not used" % (len(pos), got, min(n_fit, n_known), min_loo))
        if checked and not e_fit <= (1.0 - min_gain) * e_known:
            self.calib_refused = dict(g=g, d=d, s=s, why="loo", loo_known_cm=e_known, loo_fit_cm=e_fit)
            return self._keep_known(sign=None if st.sign_known else g, why=
                "a fit to the %d anchors gave %s, but it placed each anchor left out in turn (with those within 5 s) "
                "no better than the known values (median %.1f cm against %.1f cm), so it was not used" % (len(pos), got, e_fit, e_known))
        rms = self._fit_rms(best.x, ta_all, sk, is_pos, px, py)
        self.calib.update(g=g, d=d, s=s, fitted=True, rms_cm=rms, n=len(used), loo_known_cm=e_known, loo_fit_cm=e_fit)
        self._path = None
        better = ("; it places each anchor left out in turn within %.1f cm (median) against %.1f cm with the known "
                  "values" % (e_fit, e_known)) if checked else (
            "; too few anchors to check it by leaving each out in turn, so it is used only because %s and it stays "
            "within the known range" % ("the home visits pin it down" if exact >= 2 else
                                        "the heading's sign was not checked in the log"))
        self.calib_note = ("%s: the log's turn and distance corrections changed from the known values (%s) to %s, "
                           "fitted to %d anchors (one-fit misfit %.1f cm)%s. Only the guesses from the log change; Z "
                           "(Undo) brings back the guesses from before this Line up." % (
                               self.stream.name, self._known_words(), got, len(used), rms, better))
        self.notes.append(self.calib_note)
        return self.calib

    def _fit_rms(self, p, ta_all, sk, is_pos, px, py):
        qx, qy, h = self._integrate(*p[:3])
        Ax, Ay, _ = self._at(ta_all, [qx, qy, h])
        P = p[3:].reshape(-1, 3)
        al = np.radians(P[sk, 2])
        ex = P[sk, 0] + np.cos(al) * Ax - np.sin(al) * Ay - px
        ey = P[sk, 1] + np.sin(al) * Ax + np.cos(al) * Ay - py
        e = np.hypot(ex, ey)[is_pos]
        return float(np.sqrt(np.mean(e ** 2))) if len(e) else math.nan

    def describe(self):
        c = self.calib
        kinds = {}
        for a in self.anchors:
            kinds[a.kind] = kinds.get(a.kind, 0) + 1
        s = "%s: %d anchors (%s)" % (self.stream.name, len(self.anchors),
                                     ", ".join("%d %s" % (v, k) for k, v in sorted(kinds.items())) or "none")
        if c["fitted"]:
            s += "; corrections gyro x%.3f, drift %+.2f deg/s, wheels x%.3f (fitted)" % (c["g"], c["d"], c["s"])
        elif self.stream.frame == "own":
            s += "; known corrections %s" % self._known_words()
        return s


RAYLEIGH_MEDIAN = math.sqrt(2 * math.log(2))    # median of |error| / sd for a well-judged 2-D sd


def loo_errors(aligner, kinds=("label",), causal=False):
    """Leave-one-out check of the alignment: each anchor of these kinds is left out in turn and
    predicted from the rest. Returns [(t, error cm, predicted sd cm)] (only where predictable)."""
    out = []
    keep = list(aligner.anchors)
    try:
        for a in keep:
            if a.kind not in kinds or not a.has_pos:
                continue
            aligner.anchors = [b for b in keep if b is not a]
            p = aligner.pose_at(np.array([a.t]), causal=causal)
            if p["ok"][0]:
                out.append((a.t, math.hypot(p["x"][0] - a.x, p["y"][0] - a.y),
                            math.sqrt(p["sd_cm"][0] ** 2 + a.sd_cm ** 2)))
    finally:
        aligner.anchors = keep
    return out


def tune_allowances(aligner, kinds=("label",), min_points=5, factors=None):
    """Scale the drift allowances so the predicted sd matches the leave-one-out errors of the
    person's labels (median |error| / sd = 1.18, as for a well-judged sd). Measured, not picked:
    the factor that makes the confidence honest for THIS robot and session. Returns dict(factor,
    n, median_ratio_before, median_ratio_after) or None when there are too few labels."""
    factors = np.geomspace(0.2, 5.0, 29) if factors is None else factors
    base = aligner.allow_scale
    first = loo_errors(aligner, kinds)
    if len(first) < min_points:
        return None
    best = None
    for f in factors:
        aligner.allow_scale = base * f
        r = loo_errors(aligner, kinds)
        if len(r) < min_points:
            continue
        ratio = float(np.median([e / sd for _, e, sd in r]))
        score = abs(math.log(max(ratio, 1e-9) / RAYLEIGH_MEDIAN))
        if best is None or score < best[0]:
            best = (score, f, ratio, len(r))
    if best is None:
        aligner.allow_scale = base
        return None
    aligner.allow_scale = base * best[1]
    rep = dict(factor=float(aligner.allow_scale), n=best[3], median_ratio_before=float(np.median([e / sd for _, e, sd in first])),
               median_ratio_after=best[2], median_error_cm=float(np.median([e for _, e, _ in first])))
    aligner.notes.append("drift allowances x%.2f from %d labels left out in turn (median error %.1f cm)" % (
        rep["factor"], rep["n"], rep["median_error_cm"]))
    return rep


def _wrap_rad(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _interp_angle(ts, t, h):
    h = np.asarray(h, float)
    if not np.isfinite(h).any():
        return np.full(len(ts), np.nan)
    hr = np.unwrap(np.radians(_ffill(h)))
    return np.degrees(np.interp(ts, t, hr)) % 360.0


def align(data, anchors_by_name, settings=None, fit=True, tune=True):
    """{name: Aligner} for every stream of an OutsideData, with its anchors added, (fit=True) the
    calibration fitted where the anchors allow it, and (tune=True) the drift allowances measured
    against the person's labels where there are at least 5 of them."""
    out = {}
    for name, st in data.streams.items():
        al = Aligner(st, settings)
        al.add(*anchors_by_name.get(name, []))
        if fit and st.frame == "own":
            al.fit_calibration()
        if tune and st.frame == "own":
            tune_allowances(al)
        out[name] = al
    return out


# =============================================================================== anchors
def anchors_from_labels(labels_by_frame, frame_times, camera, obj_names, class_heights=None, settings=None,
                        statuses=("confirmed", "corrected")):
    """The person's labels -> {name: [Anchor]}. labels_by_frame: {frame: [objects]} or Session.frames
    style {frame: {"objects": [...]}}; obj_names {obj id: stream name} (from the roll call or
    match_names); class_heights {cls: cm above the floor of the labelled point}."""
    s = settings or Settings()
    out = {}
    for f, v in labels_by_frame.items():
        objs = v.get("objects", []) if isinstance(v, dict) else v
        t = frame_times.get(int(f)) if hasattr(frame_times, "get") else None
        if t is None:
            continue
        for o in objs:
            if o.get("status") not in statuses:
                continue
            name = obj_names.get(o.get("obj"))
            if name is None:
                continue
            z = (class_heights or {}).get(o.get("cls"), 0.0)
            x, y = camera.to_floor(float(o["x"]), float(o["y"]), z)
            if not (np.isfinite(x) and np.isfinite(y)):
                continue
            ppc = camera.px_per_cm(x, y, z)
            sd = math.hypot(s.label_sd_px / max(ppc, 1e-6), s.calib_sd_cm)
            h = None
            if o.get("heading_known") and o.get("heading_deg") is not None:
                h = floor_heading(camera, float(o["x"]), float(o["y"]), z, float(o["heading_deg"]))
            out.setdefault(name, []).append(Anchor(t, x, y, h, sd_cm=sd, sd_deg=s.label_heading_sd_deg, kind="label",
                                                   note="frame %s" % f, frame=int(f)))
    return out


def heading_anchor_from_motion(stream, track, t_from=None, t_to=None, min_cm=8.0, max_turn_deg=6.0, track_sd_cm=1.0,
                               first_only=True):
    """Heading anchors from stretches where the stream says the object drives FORWARD and
    straight, and the camera track shows where it went: the track's direction over the stretch
    is the facing (motion direction is the facing only when driving forward: robots reverse and
    spin in place, so the log decides which stretches count).

    track: (t, x_cm, y_cm) rows on the shared clock (detections of that object on the floor).
    Returns [Anchor] (heading only), the first one only unless first_only=False."""
    tr = np.asarray(track, float)
    if stream.frame != "own" or len(tr) < 2 or len(stream) < 2:
        return []
    tr = tr[np.argsort(tr[:, 0])]
    st = stream
    lo = st.t[0] if t_from is None else t_from
    hi = st.t[-1] if t_to is None else t_to
    idx = np.where((st.t >= lo) & (st.t <= hi))[0]
    out = []
    i = 0
    while i < len(idx) - 1:
        a = idx[i]
        j = i + 1
        while j < len(idx):
            b = idx[j]
            if st.dist[b] - st.dist[idx[j - 1]] < -0.05 or abs(st.heading[b] - st.heading[a]) > max_turn_deg \
                    or st.segment[b] != st.segment[a] or not st.valid[b]:
                break
            j += 1
        b = idx[j - 1]
        if st.dist[b] - st.dist[a] >= min_cm:
            ta, tb = st.t[a], st.t[b]
            if tr[0, 0] <= ta and tr[-1, 0] >= tb:
                # the track must actually cover the stretch (no long hole)
                inside = tr[(tr[:, 0] >= ta) & (tr[:, 0] <= tb)]
                if len(inside) >= 2 and np.max(np.diff(np.concatenate([[ta], inside[:, 0], [tb]]))) < 1.0:
                    xa, ya = np.interp(ta, tr[:, 0], tr[:, 1]), np.interp(ta, tr[:, 0], tr[:, 2])
                    xb, yb = np.interp(tb, tr[:, 0], tr[:, 1]), np.interp(tb, tr[:, 0], tr[:, 2])
                    disp = math.hypot(xb - xa, yb - ya)
                    if disp >= 0.6 * min_cm:
                        hd = math.degrees(math.atan2(yb - ya, xb - xa))
                        # the stretch's mean own heading sits at its middle (in path terms)
                        tm = float(np.interp((st.dist[a] + st.dist[b]) / 2, st.dist[a:b + 1], st.t[a:b + 1]))
                        sd = math.degrees(math.atan2(track_sd_cm * math.sqrt(2), disp)) + max_turn_deg / 3
                        out.append(Anchor(tm, None, None, hd, sd_deg=sd, kind="motion",
                                          note="forward %.0f cm seen on video" % disp))
                        if first_only:
                            return out
        i = max(j, i + 1)
    return out


# =============================================================================== outputs
def draft_labels(aligners, frame_times, camera, obj_names=None, cls_of=None, class_heights=None, settings=None,
                 margin_px=0.0, frames=None, plugin=""):
    """Draft labels: {frame: [label dict]} from every aligned stream at every frame (or the given
    frames). obj_names {name: obj id} (None: obj left None, the tracker or roll call assigns it);
    cls_of {name: cls} (default: the stream's cls, else "unknown")."""
    s = settings or Settings()
    fr = sorted(frame_times) if frames is None else [f for f in frames if f in frame_times]
    ts = np.array([frame_times[f] for f in fr], float)
    out = {}
    for name, al in aligners.items():
        st = al.stream
        cls = (cls_of or {}).get(name) or st.cls or "unknown"
        z = (class_heights or {}).get(cls, 0.0)
        p = al.pose_at(ts)
        for k in np.where(p["ok"])[0]:
            score = math.exp(-0.5 * (p["sd_cm"][k] / s.ref_cm) ** 2)
            if score < s.min_score:
                continue
            x, y = float(p["x"][k]), float(p["y"][k])
            u, v = camera.project((x, y, z))
            if not bool(camera.in_image(u, v, margin_px)):
                continue
            hf = p["heading"][k]
            hk = bool(np.isfinite(hf) and p["sd_deg"][k] <= s.heading_known_sd_deg)
            hi = image_heading(camera, x, y, z, float(hf)) if np.isfinite(hf) else None
            out.setdefault(fr[k], []).append(dict(
                obj=(obj_names or {}).get(name), name=name, cls=cls, x=round(float(u), 2), y=round(float(v), 2),
                heading_deg=round(hi, 1) if hk and hi is not None else None, heading_known=hk,
                heading_guess=None if hk or hi is None else round(hi, 1),
                source=SOURCE, status="pending", score=round(score, 3), outside=plugin or "",
                x_cm=round(x, 2), y_cm=round(y, 2), sd_cm=round(float(p["sd_cm"][k]), 2),
                heading_sd_deg=round(float(p["sd_deg"][k]), 1)))
    return out


def score_branches(aligner, branches, camera, height_cm=0.0, meas_sd_px=2.0, use_heading=True):
    """Votes on the tracker's split branches for ONE object (the stream's): branches {id: [(t, u, v)
    or (t, u, v, heading_img_deg or None)]} on the shared clock. Returns {id: dict(p, loglik,
    mean_err_cm, n, informative)}; p sums to 1 over the branches (equal when the log can't tell:
    not aligned there, or no overlap)."""
    res = {}
    for bid, pts in branches.items():
        pts = [tuple(p) for p in pts]
        if not pts:
            res[bid] = dict(loglik=0.0, mean_err_cm=None, n=0, informative=False)
            continue
        ts = np.array([p[0] for p in pts], float)
        pose = aligner.pose_at(ts)
        ll, errs, n = 0.0, [], 0
        for k, p in enumerate(pts):
            if not pose["ok"][k]:
                continue
            x, y = camera.to_floor(float(p[1]), float(p[2]), height_cm)
            if not (np.isfinite(x) and np.isfinite(y)):
                continue
            ms = meas_sd_px / max(camera.px_per_cm(x, y, height_cm), 1e-6)
            var = pose["sd_cm"][k] ** 2 + ms ** 2
            e = math.hypot(x - pose["x"][k], y - pose["y"][k])
            ll += -0.5 * e * e / var - math.log(var)     # 2-D Gaussian, up to a constant
            if use_heading and len(p) > 3 and p[3] is not None and np.isfinite(pose["heading"][k]):
                hf = floor_heading(camera, float(p[1]), float(p[2]), height_cm, float(p[3]))
                dh = math.radians(float(_wrap180(hf - pose["heading"][k])))
                vh = math.radians(pose["sd_deg"][k]) ** 2 + math.radians(15.0) ** 2
                ll += -0.5 * dh * dh / vh - 0.5 * math.log(vh)
            errs.append(e)
            n += 1
        res[bid] = dict(loglik=ll, mean_err_cm=float(np.mean(errs)) if errs else None, n=n, informative=n > 0)
    # compare on equal footing: branches with fewer usable points don't win by omission
    nmin = min((r["n"] for r in res.values()), default=0)
    if nmin == 0:
        for r in res.values():
            r["p"] = 1.0 / max(1, len(res))
            r["informative"] = False
        return res
    lls = {b: r["loglik"] * nmin / r["n"] for b, r in res.items()}
    m = max(lls.values())
    z = sum(math.exp(v - m) for v in lls.values())
    for b, r in res.items():
        r["p"] = math.exp(lls[b] - m) / z
    return res


def score_candidates(aligners, t, candidates, camera, height_cm=0.0, meas_sd_px=2.0):
    """One moment: candidates [(u, v)] -> {name: [likelihood ratio 0..1 per candidate]} (1 = right
    where the log puts it, exp(-1/2) at one sd)."""
    out = {}
    for name, al in aligners.items():
        p = al.pose_at(np.array([t], float))
        if not p["ok"][0]:
            out[name] = [None] * len(candidates)
            continue
        row = []
        for (u, v) in candidates:
            x, y = camera.to_floor(float(u), float(v), height_cm)
            ms = meas_sd_px / max(camera.px_per_cm(x, y, height_cm), 1e-6)
            e = math.hypot(x - p["x"][0], y - p["y"][0])
            row.append(math.exp(-0.5 * e * e / (p["sd_cm"][0] ** 2 + ms ** 2)))
        out[name] = row
    return out


def track_floor(track_px, camera, height_cm=0.0):
    """[(t, u, v)] -> (n, 3) array of t, x_cm, y_cm."""
    a = np.asarray([(p[0], p[1], p[2]) for p in track_px], float).reshape(-1, 3)
    if not len(a):
        return a
    x, y = camera.to_floor(a[:, 1], a[:, 2], height_cm)
    return np.column_stack([a[:, 0], x, y])


def _track_speed(tr, ts, window_s=0.5):
    tr = tr[np.argsort(tr[:, 0])]
    xa, xb = np.interp(ts - window_s / 2, tr[:, 0], tr[:, 1]), np.interp(ts + window_s / 2, tr[:, 0], tr[:, 1])
    ya, yb = np.interp(ts - window_s / 2, tr[:, 0], tr[:, 2]), np.interp(ts + window_s / 2, tr[:, 0], tr[:, 2])
    sp = np.hypot(xb - xa, yb - ya) / window_s
    covered = (ts - window_s / 2 >= tr[0, 0]) & (ts + window_s / 2 <= tr[-1, 0])
    return np.where(covered, sp, np.nan)


def estimate_time_offset(stream, track, search_s=3.0, step_s=0.02, window_s=0.5):
    """How far the stream's clock is off from the track's: (offset_s, correlation). Add offset_s to
    the stream's times (t_wall = stream t + offset_s). From the speed over time (rotation and
    alignment don't matter), so it works before any anchor. track: (t, x_cm, y_cm) rows."""
    tr = np.asarray(track, float)
    if len(tr) < 10 or len(stream) < 10:
        return None, 0.0
    tg = np.arange(max(tr[0, 0], stream.t[0]) + search_s, min(tr[-1, 0], stream.t[-1]) - search_s, 0.05)
    if len(tg) < 20:
        return None, 0.0
    a = _track_speed(tr, tg, window_s)
    best = (None, -2.0)
    for off in np.arange(-search_s, search_s + 1e-9, step_s):
        b = stream.speed(tg - off, window_s)
        ok = np.isfinite(a) & np.isfinite(b)
        if ok.sum() < 20 or np.std(a[ok]) < 1e-6 or np.std(b[ok]) < 1e-6:
            continue
        c = float(np.corrcoef(a[ok], b[ok])[0, 1])
        if c > best[1]:
            best = (float(off), c)
    return best


def match_names(streams, tracks, window_s=0.5, step_s=0.25):
    """Which stream is which camera track, from speed over time: streams {name: Stream}, tracks
    {obj: (t, x_cm, y_cm) rows}. Returns ({obj: name}, {(obj, name): mean speed difference cm/s})."""
    from scipy.optimize import linear_sum_assignment
    names = sorted(streams)
    objs = sorted(tracks)
    if not names or not objs:
        return {}, {}
    cost = np.full((len(objs), len(names)), 1e3)
    detail = {}
    for i, o in enumerate(objs):
        tr = np.asarray(tracks[o], float)
        if len(tr) < 4:
            continue
        tg = np.arange(tr[:, 0].min(), tr[:, 0].max(), step_s)
        a = _track_speed(tr, tg, window_s)
        for j, n in enumerate(names):
            st = streams[n]
            if not len(st):
                continue
            b = st.speed(tg, window_s)
            inside = (tg >= st.t[0]) & (tg <= st.t[-1])
            ok = np.isfinite(a) & np.isfinite(b) & inside
            if ok.sum() < 8:
                continue
            c = float(np.mean(np.abs(a[ok] - b[ok])))
            cost[i, j] = c
            detail[(o, n)] = c
    r, c = linear_sum_assignment(cost)
    return {objs[i]: names[j] for i, j in zip(r, c) if cost[i, j] < 1e3}, detail
