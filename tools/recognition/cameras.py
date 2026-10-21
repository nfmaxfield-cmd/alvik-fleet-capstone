"""Full pinhole camera models in the floor frame (see CROSSCAM.md).

Floor frame: x, y in cm on the floor (the tape-sheet axes, as in
floor_calibration.json), z in cm UP from the floor. Image: pixels, origin
top-left, x right, y down, pixel centres on whole numbers (as OpenCV).

A CameraModel is K (3x3), dist (OpenCV coefficients; this module only fits k1),
rvec, tvec (floor -> camera, OpenCV convention: p_cam = R p_floor + t).

    cam = CameraModel.from_points(img_pts, floor_pts, (640, 480), height_cm=262)
    cam = CameraModel.from_floor_calibration("rec/floor_calibration.json")
    u, v = cam.project((x, y, z));  x, y = cam.to_floor(u, v, z=4.0)
    cam.save("camera_model.json");  CameraModel.load("camera_model.json")

How K is estimated when not given (from_points, K=None): principal point fixed
at the image centre, square pixels, no skew, and at most one radial term k1
(the same lens model calibrate_floor.py and tools/sim use). Only the focal
length f (plus k1) is fitted, jointly with the pose, by Levenberg-Marquardt on
the pixel residuals, started from several focal lengths (Zhang's closed form
from the homography + a spread of guesses) each with an SQPnP pose. Why this
and not cv2.calibrateCamera on one view: a single planar view fixes at most two
intrinsics, and for a camera looking nearly straight down even the focal length
is not fixed by floor points (zoom in + move up looks the same); calibrateCamera
then wanders to an arbitrary f. Here the fit is regularised instead:
  - height_cm (tape, lens to floor) and/or nadir_cm (plumb line) enter as
    priors on the camera position; with a height the overhead camera is fully
    determined;
  - clicked points at a known height (floor_z, e.g. robot tops at 4 cm) break
    the flatness and fix f from the data alone;
  - otherwise a weak prior (a 60 deg-wide view, sd = factor 2) keeps f sane,
    and fit["warnings"] says the height above the floor is a guess. Floor
    positions (z = 0) are right regardless; only lifted points suffer.
For an oblique camera the floor points fix f by themselves (the prior then
has no visible effect).
"""
import datetime as dt
import json
import math

import cv2
import numpy as np

DEFAULT_HFOV_DEG = 60.0     # weak prior on the focal length when nothing else fixes it
F_PRIOR_SD_LOG = 0.7        # sd of that prior, in log(f): about a factor 2
CLICK_SD_PX = 1.0           # assumed click noise (weights the priors against the pixels)
K1_KEEP_RATIO = 0.85        # k1='auto' keeps k1 only if left-one-out floor RMS drops below this share
K1_MIN_POINTS = 8
F_POORLY_FIXED = 0.10       # relative sd of f above which the focal length counts as not fixed by the data
H_KEYS = ("H_floor_to_px", "homography_floor_cm_to_px", "H_floor_to_image", "H", "homography", "matrix")


class CalibrationError(ValueError):
    pass


def _K(f, c):
    return np.array([[f, 0.0, c[0]], [0.0, f, c[1]], [0.0, 0.0, 1.0]])


def _dist5(dist):
    d = np.zeros(5)
    if dist is not None:
        a = np.asarray(dist, float).ravel()
        if len(a) > 5:
            d = np.zeros(len(a))
        d[:len(a)] = a
    return d


def _apply_h(H, xy):
    xy = np.asarray(xy, float).reshape(-1, 2)
    p = np.hstack([xy, np.ones((len(xy), 1))]) @ np.asarray(H, float).T
    return p[:, :2] / p[:, 2:3], p[:, 2]


# ---------------------------------------------------------------- the model
class CameraModel:
    def __init__(self, K, dist, rvec, tvec, image_size, fit=None, name=None, mirror_y=False):
        self.K = np.asarray(K, float).reshape(3, 3)
        self.dist = _dist5(dist)
        self.rvec = np.asarray(rvec, float).reshape(3)
        self.tvec = np.asarray(tvec, float).reshape(3)
        self.image_size = (int(image_size[0]), int(image_size[1]))
        self.fit = fit or {}
        self.name = name
        # mirror_y: the person's floor axes are mirrored against a camera above the floor (y runs the other
        # way round from x than a right-handed floor with z up): the pose is kept in the y-flipped frame and
        # every floor point is flipped on the way in and out, so callers always use the person's x, y
        self.mirror_y = bool(mirror_y)

    # -- geometry (internally in the pose's frame: the person's floor with y flipped when mirror_y)
    def _in(self, p):
        if not self.mirror_y:
            return p
        p = np.array(p, float, copy=True)
        p[..., 1] *= -1.0
        return p

    @property
    def R(self):
        return cv2.Rodrigues(self.rvec)[0]

    @property
    def _center_int(self):
        return -self.R.T @ self.tvec

    @property
    def center(self):
        """Lens position in the floor frame (cm; z = height above the floor)."""
        return self._in(self._center_int)

    @property
    def f(self):
        return 0.5 * (self.K[0, 0] + self.K[1, 1])

    def _depth_int(self, p):
        return (p @ self.R.T + self.tvec)[:, 2]

    def depth(self, xyz):
        p = np.asarray(xyz, float).reshape(-1, 3)
        return self._depth_int(self._in(p))

    def project(self, xyz):
        """Floor-frame point(s) (x, y, z cm) -> pixel (u, v). One point -> two floats;
        (n, 3) -> two arrays. Points behind the camera give nan."""
        p = np.asarray(xyz, float)
        single = p.ndim == 1
        p = self._in(p.reshape(-1, 3))
        uv, _ = cv2.projectPoints(p.reshape(-1, 1, 3), self.rvec, self.tvec, self.K, self.dist)
        uv = uv.reshape(-1, 2)
        uv[self._depth_int(p) <= 1e-9] = np.nan
        if single:
            return float(uv[0, 0]), float(uv[0, 1])
        return uv[:, 0], uv[:, 1]

    def _rays_int(self, u, v):
        uv = np.column_stack([np.ravel(u), np.ravel(v)]).astype(np.float64)
        if np.any(self.dist):
            crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-12)
            n = cv2.undistortPointsIter(uv.reshape(-1, 1, 2), self.K, self.dist, None, None, crit).reshape(-1, 2)
        else:
            n = (uv - self.K[:2, 2]) @ np.linalg.inv(self.K[:2, :2]).T
        d = np.column_stack([n, np.ones(len(n))]) @ self.R      # R.T @ [xn, yn, 1] for each row
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    def rays(self, u, v):
        """Unit ray directions (floor frame) through pixel(s); (n, 3)."""
        return self._in(self._rays_int(u, v))

    def to_floor(self, u, v, z=0.0):
        """Pixel(s) -> (x, y) cm where the ray meets the horizontal plane z cm up.
        nan if the ray does not reach that plane in front of the camera."""
        single = np.ndim(u) == 0
        d = self._rays_int(u, v)
        C = self._center_int
        z = np.broadcast_to(np.asarray(z, float), (len(d),))
        with np.errstate(divide="ignore", invalid="ignore"):
            s = (z - C[2]) / d[:, 2]
        s[~(s > 0)] = np.nan
        x, y = C[0] + s * d[:, 0], C[1] + s * d[:, 1]
        if self.mirror_y:
            y = -y
        if single:
            return float(x[0]), float(y[0])
        return x, y

    def in_image(self, u, v, margin=0.0):
        W, H = self.image_size
        u, v = np.asarray(u, float), np.asarray(v, float)
        ok = (u >= -0.5 + margin) & (u <= W - 0.5 - margin) & (v >= -0.5 + margin) & (v <= H - 0.5 - margin)
        return ok & np.isfinite(u) & np.isfinite(v)

    def px_per_cm(self, x, y, z=0.0):
        """Image scale (px per cm of floor-parallel movement) around a point."""
        u0, v0 = self.project((x, y, z))
        u1, v1 = self.project((x + 1.0, y, z))
        u2, v2 = self.project((x, y + 1.0, z))
        return 0.5 * (math.hypot(u1 - u0, v1 - v0) + math.hypot(u2 - u0, v2 - v0))

    # -- summaries
    def pose_summary(self):
        C = self.center
        axis = self.R.T @ np.array([0.0, 0.0, 1.0])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -axis[2]))))
        right = self._in(self.R.T @ np.array([1.0, 0.0, 0.0]))
        look = self.to_floor(self.K[0, 2], self.K[1, 2], 0.0)
        return dict(camera_position_cm=[round(float(c), 2) for c in C], tilt_from_vertical_deg=round(tilt, 2),
                    image_right_floor_deg=round(math.degrees(math.atan2(right[1], right[0])), 2),
                    look_at_floor_cm=[None if not np.isfinite(a) else round(a, 2) for a in look],
                    f_px=round(self.f, 2), principal_point=[round(float(a), 2) for a in self.K[:2, 2]],
                    k1=round(float(self.dist[0]), 5))

    def describe(self):
        s = self.pose_summary()
        C = s["camera_position_cm"]
        lines = [f"Camera{(' ' + self.name) if self.name else ''}: {self.image_size[0]}x{self.image_size[1]}, "
                 f"f {s['f_px']:.0f} px, principal point {s['principal_point']}, k1 {s['k1']:.4f}",
                 f"Lens at x {C[0]:.1f}, y {C[1]:.1f} cm, {C[2]:.1f} cm above the floor; tilted "
                 f"{s['tilt_from_vertical_deg']:.1f} deg from straight down, image centre on floor point "
                 f"{s['look_at_floor_cm']}"]
        fit = self.fit
        if fit.get("rms_px") is not None:
            loo = fit.get("loo_rms_cm")
            lines.append(f"Fit ({fit.get('method')}): {fit.get('n_points')} points, RMS {fit['rms_px']:.2f} px"
                         + (f", left-one-out floor error {loo:.2f} cm RMS" if loo is not None else "")
                         + f"; focal length from {fit.get('f_source')}")
        for w in fit.get("warnings") or []:
            lines.append("WARNING: " + w)
        return "\n".join(lines)

    # -- files
    def to_dict(self):
        return dict(
            model="pinhole",
            note=("Floor frame: x, y cm on the floor (tape-sheet axes), z cm up. p_cam = R(rvec) p + tvec "
                  "(OpenCV); pixel = cv2.projectPoints(p, rvec, tvec, K, dist); with mirror_y true, p = (x, -y, z) "
                  "first. Written by tools/recognition/cameras.py."),
            name=self.name, image_size=list(self.image_size), K=self.K.tolist(), dist=self.dist.tolist(),
            rvec=self.rvec.tolist(), tvec=self.tvec.tolist(), **self.pose_summary(),
            fit=self.fit, created=dt.datetime.now().isoformat(timespec="seconds"),
            **({"mirror_y": True} if self.mirror_y else {}))

    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=1)

    @classmethod
    def from_dict(cls, d):
        return cls(d["K"], d.get("dist"), d["rvec"], d["tvec"], d["image_size"], fit=d.get("fit"), name=d.get("name"),
                   mirror_y=bool(d.get("mirror_y")))

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls.from_dict(json.load(f))

    # -- fitting
    @classmethod
    def from_points(cls, img_pts, floor_pts, image_size, K=None, dist=None, floor_z=None, height_cm=None,
                    nadir_cm=None, height_sd_cm=1.0, nadir_sd_cm=2.0, k1="auto", name=None, mirror=False):
        """Fit from >= 6 clicked points. img_pts (n, 2) px; floor_pts (n, 2) cm (or (n, 3) with z);
        floor_z: per-point height (cm, default 0). K / dist: fixed intrinsics if known.
        height_cm / nadir_cm: tape-measured lens height and plumb-line floor point (optional priors).
        k1: 'auto' (keep a radial term only if it improves the left-one-out error), 'fit', 'off'
        (ignored when dist is given). fit info: rms_px, loo_rms_cm, per-point errors, warnings.
        mirror: False (the floor's x, y, z-up are right-handed as given), True (y runs the other way:
        the model keeps mirror_y), or 'auto': fit both and keep the one with the camera above the
        floor and the smaller pixel error (fit["mirror"] says which, and why)."""
        img = np.asarray(img_pts, float).reshape(-1, 2)
        fp = np.asarray(floor_pts, float)
        if fp.ndim == 2 and fp.shape[1] == 3:
            obj = fp.copy()
        else:
            fp = fp.reshape(-1, 2)
            z = np.zeros(len(fp)) if floor_z is None else np.broadcast_to(np.asarray(floor_z, float), (len(fp),))
            obj = np.column_stack([fp, z])
        if mirror == "auto":
            tries, err = [], None
            for m in (False, True):
                try:
                    tries.append(cls.from_points(img, obj, image_size, K=K, dist=dist, height_cm=height_cm,
                                                 nadir_cm=nadir_cm, height_sd_cm=height_sd_cm, nadir_sd_cm=nadir_sd_cm,
                                                 k1=k1, name=name, mirror=m))
                except CalibrationError as e:
                    err = e
            if not tries:
                raise err
            above = [t for t in tries if t.center[2] > 0]
            pool = above or tries
            best = min(pool, key=lambda t: t.fit["rms_px"])
            other = [t for t in tries if t is not best]
            one_way = len(above) == 1 and (len(tries) == 2 or err is not None)
            best.fit["mirror"] = dict(
                used=bool(best.mirror_y), tried_both=True,
                other_rms_px=other[0].fit["rms_px"] if other else None,
                other_failed=str(err) if err is not None and not other else None,
                why=("only this way round puts the camera above the floor" if one_way else "the smaller pixel error"))
            return best
        if mirror:
            obj = obj.copy()
            obj[:, 1] *= -1.0
            if nadir_cm is not None:
                nadir_cm = [float(nadir_cm[0]), -float(nadir_cm[1])]
        n = len(img)
        if n != len(obj):
            raise CalibrationError(f"{n} image points but {len(obj)} floor points")
        if n < 6:
            raise CalibrationError(f"need at least 6 points, have {n}")
        _check_spread(obj[:, :2])
        W, H = image_size
        priors = []
        if height_cm:
            priors.append(("height", float(height_cm), float(height_sd_cm)))
        if nadir_cm is not None:
            priors.append(("nadir", np.asarray(nadir_cm, float), float(nadir_sd_cm)))
        f_free = K is None
        Kb = _K(W / (2 * math.tan(math.radians(DEFAULT_HFOV_DEG) / 2)), (W / 2.0, H / 2.0)) if K is None \
            else np.asarray(K, float)
        if f_free:
            priors.append(("logf", math.log(Kb[0, 0]), F_PRIOR_SD_LOG))
        db = _dist5(dist)
        prob = _Problem(img, obj, Kb, db, f_free, priors)

        p0 = prob.multistart(fit_k1=False)
        fits = {"off": p0}
        loos = {}
        k1_info = None
        mode = "off" if dist is not None else (k1 or "off")
        if mode in ("auto", "fit") and (mode == "fit" or n >= K1_MIN_POINTS):
            p1 = prob.refine(p0, fit_k1=True)
            if mode == "fit":
                fits = {"fit": p1}
            else:
                loos["off"], loos["fit"] = prob.loo(p0, fit_k1=False), prob.loo(p1, fit_k1=True)
                l0, l1 = _rms(loos["off"]), _rms(loos["fit"])
                keep = l1 < K1_KEEP_RATIO * l0
                k1_info = dict(k1_tried=round(float(p1[1]), 5), loo_rms_cm_without=round(l0, 3),
                               loo_rms_cm_with=round(l1, 3), kept=bool(keep))
                if keep:
                    fits = {"fit": p1}
        (kmode, p), = fits.items()
        fit_k1 = kmode == "fit"
        model = prob.model(p, image_size, name)
        res = (prob.project(p) - img)
        err_px = np.hypot(res[:, 0], res[:, 1])
        loo = loos[kmode] if kmode in loos else prob.loo(p, fit_k1=fit_k1)
        sd_data, sd_post = prob.f_sd(p, fit_k1)
        warnings = []
        if f_free:
            if sd_post > F_POORLY_FIXED:
                f_source = f"a default {DEFAULT_HFOV_DEG:.0f} deg-wide view (NOT measured)"
                warnings.append(
                    "The focal length / camera height is not fixed by these points (the camera looks nearly "
                    "straight down at a flat floor). Floor positions are fine, but anything above the floor "
                    "(robot tops) will be placed using a guessed height of %.0f cm. Give height_cm (tape, "
                    "lens to floor) or click a few points at a known height (floor_z, e.g. robot tops at "
                    "4 cm)." % model.center[2])
            elif sd_data > F_POORLY_FIXED:
                f_source = "the points + the given camera height/nadir"
            else:
                f_source = "the points"
        else:
            f_source = "given K"
        if height_cm and abs(model.center[2] - height_cm) > 3 * height_sd_cm + 2:
            warnings.append(f"fitted lens height {model.center[2]:.1f} cm disagrees with the given "
                            f"{height_cm:g} cm: check the tape height or the clicked points")
        if np.median(err_px) > 0 and err_px.max() > max(3.0, 5 * np.median(err_px)):
            i = int(np.argmax(err_px))
            warnings.append(f"point {i} is off by {err_px[i]:.1f} px (median {np.median(err_px):.2f}): "
                            "mis-click or mistyped coordinates?")
        model.fit = dict(
            method="from_points", n_points=int(n), rms_px=round(_rms(err_px), 4), max_px=round(float(err_px.max()), 4),
            loo_rms_cm=round(_rms(loo), 4), loo_max_cm=round(float(np.nanmax(loo)), 4),
            f_source=f_source, f_rel_sd=round(float(sd_post), 4), f_rel_sd_points_only=round(float(sd_data), 4),
            height_prior_cm=height_cm, nadir_prior_cm=None if nadir_cm is None else list(map(float, nadir_cm)),
            k1=k1_info, warnings=warnings,
            points=[dict(u=float(img[i, 0]), v=float(img[i, 1]), x=float(obj[i, 0]),
                         y=float(-obj[i, 1] if mirror else obj[i, 1]),
                         z=float(obj[i, 2]), err_px=round(float(err_px[i]), 3), loo_cm=round(float(loo[i]), 3))
                    for i in range(n)])
        model.mirror_y = bool(mirror)
        return model

    @classmethod
    def from_floor_calibration(cls, path, height_sd_cm=1.0, nadir_sd_cm=2.0, name=None):
        """Camera model from a floor_calibration.json written by tools/calibrate_floor.py
        (homography floor cm -> ideal px, optional k1 / f_px / cx / cy lens term, optional
        camera.height_cm and camera.nadir_floor_cm). The pose is recovered by fitting a
        pinhole (principal point = the file's cx, cy) to points generated from the
        homography over the floor it sees. Raises CalibrationError when the file does
        not fix the camera (nearly straight-down view and no camera.height_cm)."""
        with open(path) as f:
            d = json.load(f)
        H = next((d[k] for k in H_KEYS if k in d), None)
        if H is None:
            raise CalibrationError(f"{path}: no homography (expected one of {', '.join(H_KEYS)})")
        H = np.asarray(H, float)
        size = d.get("image_size")
        cx, cy = d.get("cx"), d.get("cy")
        if not size:
            if cx is None or cy is None:
                raise CalibrationError(f"{path}: no image_size (and no cx, cy): add \"image_size\": [640, 480]")
            size = [int(round(2 * cx)), int(round(2 * cy))]
        W, Hh = int(size[0]), int(size[1])
        c = (float(cx) if cx is not None else W / 2.0, float(cy) if cy is not None else Hh / 2.0)
        k1 = float(d.get("k1") or 0.0)
        f_px = float(d.get("f_px") or W)
        cam = d.get("camera") or {}
        height = cam.get("height_cm")
        nadir = cam.get("nadir_floor_cm")
        nadir_src = str(cam.get("nadir_source") or "")
        plane_z = float(d.get("plane_height_cm") or 0.0)
        if plane_z and not height:
            # the homography maps the robot-top plane; fine for the pose, the floor is plane_z below it
            pass

        # floor grid over the area the fit covered (+ margin), kept where it is in front and inside the image
        area = ((d.get("fit") or {}).get("area_cm")) or [0.0, 0.0, 122.0, 122.0]
        x0, y0, x1, y1 = map(float, area)
        mx, my = 0.5 * (x1 - x0), 0.5 * (y1 - y0)
        gx, gy = np.meshgrid(np.linspace(x0 - mx, x1 + mx, 25), np.linspace(y0 - my, y1 + my, 25))
        xy = np.column_stack([gx.ravel(), gy.ravel()])
        uv, w = _apply_h(H, xy)
        front = np.sign(np.median(np.sign(_apply_h(H, [[0.5 * (x0 + x1), 0.5 * (y0 + y1)]])[1])))
        ok = (np.sign(w) == front) & (uv[:, 0] >= 0) & (uv[:, 0] <= W - 1) & (uv[:, 1] >= 0) & (uv[:, 1] <= Hh - 1)
        if ok.sum() < 12:
            raise CalibrationError(f"{path}: the homography puts too little of the floor area {area} in the "
                                   f"{W}x{Hh} image ({int(ok.sum())} grid points)")
        xy, uv = xy[ok], uv[ok]
        obj = np.column_stack([xy, np.full(len(xy), plane_z)])

        priors = []
        if height:
            priors.append(("height", float(height), float(height_sd_cm)))
        given_nadir = nadir is not None and not nadir_src.startswith("estimated")
        if given_nadir:
            priors.append(("nadir", np.asarray(nadir, float), float(nadir_sd_cm)))
        Kb = _K(f_px, c)
        priors.append(("logf", math.log(W / (2 * math.tan(math.radians(DEFAULT_HFOV_DEG) / 2))), F_PRIOR_SD_LOG))
        prob = _Problem(uv, obj, Kb, np.zeros(5), True, priors)
        p = prob.multistart(fit_k1=False)
        sd_data, sd_post = prob.f_sd(p, False)
        # the virtual points stand in for ~ the clicks: judge how well they fix f per point at 1 px noise,
        # scaled to the number of points the calibration really had
        n_real = int((d.get("fit") or {}).get("n_inliers") or 20)
        sd_data_real = sd_data * math.sqrt(len(uv) / max(n_real, 4))
        if not height and sd_data_real > F_POORLY_FIXED:
            raise CalibrationError(
                f"{path}: the camera looks nearly straight down, so the floor homography alone does not fix "
                "its focal length or height (the pose is ambiguous). Add the lens height: rerun "
                "tools/calibrate_floor.py with --height-cm <tape, lens to floor> (it writes camera.height_cm), or "
                "add \"camera\": {\"height_cm\": <cm>} to the file; better still also --nadir x,y (plumb line). "
                "Or build the model with CameraModel.from_points using a few clicks at a known height.")
        model = prob.model(p, (W, Hh), name)
        f = model.K[0, 0]
        if k1:   # the file's lens term is normalised by f_px; OpenCV's by the focal length
            model.dist[0] = k1 * (f / f_px) ** 2
        res = prob.project(p) - uv
        err = np.hypot(res[:, 0], res[:, 1])
        warnings = []
        if _rms(err) > 0.5:
            warnings.append(f"the homography is not exactly a centred pinhole view (RMS {_rms(err):.2f} px over "
                            "the floor): principal point off-centre or non-square pixels; transfers inherit this")
        if height and abs(model.center[2] - height) > 3 * height_sd_cm + 2:
            warnings.append(f"fitted lens height {model.center[2]:.1f} cm disagrees with camera.height_cm "
                            f"{height:g}: the floor homography and the tape height don't match")
        fit = d.get("fit") or {}
        model.fit = dict(method="from_floor_calibration", source=str(path), n_points=int(len(uv)),
                         rms_px=round(_rms(err), 4), max_px=round(float(err.max()), 4),
                         loo_rms_cm=fit.get("loo_rms_cm"), rms_cm=fit.get("rms_cm"),
                         f_source=("the homography" if sd_data_real <= F_POORLY_FIXED
                                   else "the homography + camera.height_cm"),
                         f_rel_sd=round(float(sd_post), 4), height_prior_cm=height,
                         nadir_prior_cm=nadir if given_nadir else None, plane_height_cm=plane_z or None,
                         k1_file=k1, f_px_file=f_px, warnings=warnings)
        return model


# ---------------------------------------------------------------- transfer between cameras
def _wrap(a):
    return float(a) % 360.0


def image_heading_to_floor(cam, u, v, heading_deg, z=0.0, step_px=4.0):
    """Image heading (0 = image +x, counter-clockwise on screen) at pixel (u, v) of a
    point z cm up -> floor heading (deg, atan2 of floor dy, dx)."""
    h = math.radians(heading_deg)
    x0, y0 = cam.to_floor(u, v, z)
    x1, y1 = cam.to_floor(u + step_px * math.cos(h), v - step_px * math.sin(h), z)
    return _wrap(math.degrees(math.atan2(y1 - y0, x1 - x0)))


def floor_heading_to_image(cam, x, y, z, heading_deg, lead_cm=3.0):
    """Floor heading at floor point (x, y, z) -> image heading in cam (label convention)."""
    h = math.radians(heading_deg)
    u0, v0 = cam.project((x, y, z))
    u1, v1 = cam.project((x + lead_cm * math.cos(h), y + lead_cm * math.sin(h), z))
    return _wrap(math.degrees(math.atan2(-(v1 - v0), u1 - u0)))


def transfer(cam_a, cam_b, u, v, height_cm, heading_deg=None, lead_cm=3.0, margin_px=0.0):
    """Pixel (u, v) in camera A of a point height_cm above the floor -> (u_b, v_b, heading_b, visible)
    in camera B. The heading (label convention) goes through a point lead_cm ahead on the
    object's plane. visible is False if the point is behind B, outside B's image (less
    margin_px), or if A's ray does not reach that height in front of A."""
    x, y = cam_a.to_floor(u, v, height_cm)
    if not (np.isfinite(x) and np.isfinite(y)):
        return float("nan"), float("nan"), None, False
    ub, vb = cam_b.project((x, y, height_cm))
    visible = bool(cam_b.in_image(ub, vb, margin_px))
    hb = None
    if heading_deg is not None and np.isfinite(ub):
        hf = image_heading_to_floor(cam_a, u, v, heading_deg, height_cm)
        hb = floor_heading_to_image(cam_b, x, y, height_cm, hf, lead_cm)
    return ub, vb, hb, visible


# ---------------------------------------------------------------- fitting internals
def _rms(a):
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    return float(np.sqrt(np.mean(a ** 2))) if len(a) else float("nan")


def _check_spread(xy):
    xy = np.asarray(xy, float)
    s = np.linalg.svd(xy - xy.mean(0), compute_uv=False)
    if s[-1] < 1e-6 * max(s[0], 1e-9) or s[-1] / math.sqrt(len(xy)) < 1.0:
        raise CalibrationError("the points lie (nearly) on one line: add points off that line")


class _Problem:
    """Parameters p = [log f, k1, rvec(3), tvec(3)]; pixel residuals / CLICK_SD_PX plus priors."""
    EPS = np.array([1e-6, 1e-6, 1e-7, 1e-7, 1e-7, 1e-5, 1e-5, 1e-5])

    def __init__(self, img, obj, K, dist, f_free, priors):
        self.img, self.obj, self.Kb, self.db, self.f_free, self.priors = img, obj, K, dist, f_free, priors
        self.mask = np.ones(len(img), bool)

    def free(self, fit_k1):
        m = np.ones(8, bool)
        m[0] = self.f_free
        m[1] = fit_k1
        return m

    def K(self, p):
        K = self.Kb.copy()
        if self.f_free:
            K[0, 0] = K[1, 1] = math.exp(p[0])
        return K

    def dist(self, p):
        d = self.db.copy()
        d[0] = p[1]
        return d

    def project(self, p, obj=None):
        obj = self.obj if obj is None else obj
        uv, _ = cv2.projectPoints(obj.reshape(-1, 1, 3), p[2:5], p[5:8], self.K(p), self.dist(p))
        return uv.reshape(-1, 2)

    def resid(self, p, priors=True):
        r = [((self.project(p, self.obj[self.mask]) - self.img[self.mask]) / CLICK_SD_PX).ravel()]
        if priors:
            C = -cv2.Rodrigues(p[2:5])[0].T @ p[5:8]
            for kind, val, sd in self.priors:
                if kind == "height":
                    r.append([(C[2] - val) / sd])
                elif kind == "nadir":
                    r.append((C[:2] - val) / sd)
                elif kind == "logf" and self.f_free:
                    r.append([(p[0] - val) / sd])
        return np.concatenate([np.ravel(a) for a in r])

    def jac(self, p, free, priors=True):
        idx = np.flatnonzero(free)
        cols = []
        for i in idx:
            e = np.zeros(8)
            e[i] = self.EPS[i] * max(1.0, abs(p[i]))
            cols.append((self.resid(p + e, priors) - self.resid(p - e, priors)) / (2 * e[i]))
        return np.column_stack(cols)

    def lm(self, p, free, iters=200):
        p = p.copy()
        r = self.resid(p)
        cost = r @ r
        lam = 1e-3
        for _ in range(iters):
            J = self.jac(p, free)
            A, g = J.T @ J, J.T @ r
            improved = False
            while lam < 1e10:
                try:
                    dp = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
                except np.linalg.LinAlgError:
                    lam *= 10
                    continue
                q = p.copy()
                q[free] += dp
                rq = self.resid(q)
                cq = rq @ rq
                if np.isfinite(cq) and cq < cost:
                    done = cost - cq < 1e-9 * cost + 1e-12 or np.abs(dp).max() < 1e-12
                    p, r, cost = q, rq, cq
                    lam = max(lam / 3, 1e-12)
                    improved = True
                    break
                lam *= 10
            if not improved or done:
                break
        return p

    def _init_pose(self, f, k1=0.0):
        p = np.zeros(8)
        p[0], p[1] = math.log(f), k1
        K = self.K(p)
        obj, img = self.obj[self.mask], self.img[self.mask]
        try:
            ok, rv, tv = cv2.solvePnP(obj.reshape(-1, 1, 3), img.reshape(-1, 1, 2), K, self.dist(p),
                                      flags=cv2.SOLVEPNP_SQPNP)
        except cv2.error:
            ok = False
        if not ok:
            return None
        p[2:5], p[5:8] = rv.ravel(), tv.ravel()
        return p

    def _zhang_f(self):
        """Focal length from the floor homography with the principal point known (None if not fixed)."""
        obj, img = self.obj[self.mask], self.img[self.mask]
        if np.ptp(obj[:, 2]) > 1e-9:
            return None
        Hm, _ = cv2.findHomography(obj[:, :2], img - self.Kb[:2, 2], 0)
        if Hm is None:
            return None
        h = Hm
        A = np.array([h[0, 0] * h[0, 1] + h[1, 0] * h[1, 1], h[0, 0] ** 2 + h[1, 0] ** 2 - h[0, 1] ** 2 - h[1, 1] ** 2])
        b = np.array([h[2, 0] * h[2, 1], h[2, 0] ** 2 - h[2, 1] ** 2])
        if A @ A < 1e-30:
            return None
        a = -(A @ b) / (A @ A)
        return 1 / math.sqrt(a) if a > 0 else None

    def multistart(self, fit_k1):
        W = 2 * self.Kb[0, 2]
        if not self.f_free:
            cands = [self.Kb[0, 0]]
        else:
            cands = [W * s for s in (0.5, 0.8, 1.2, 1.8, 2.7, 4.0)] + [self.Kb[0, 0]]
            fz = self._zhang_f()
            if fz and 0.1 * W < fz < 20 * W:
                cands.append(fz)
        tried = []
        for f in cands:            # a short run from every start, then the best two to the end
            p = self._init_pose(f, self.db[0])
            if p is None:
                continue
            p = self.lm(p, self.free(fit_k1), iters=25)
            r = self.resid(p)
            tried.append((r @ r, p))
        best, best_cost = None, np.inf
        for _, p in sorted(tried, key=lambda a: a[0])[:2]:
            p = self.lm(p, self.free(fit_k1))
            r = self.resid(p)
            if r @ r < best_cost:
                best, best_cost = p, r @ r
        if best is None:
            raise CalibrationError("could not find any camera pose for these points")
        C = -cv2.Rodrigues(best[2:5])[0].T @ best[5:8]
        if C[2] <= 0:
            raise CalibrationError("the fitted camera is below the floor: x and y axes swapped / mirrored?")
        return best

    def refine(self, p, fit_k1):
        return self.lm(p, self.free(fit_k1))

    def model(self, p, size, name=None):
        return CameraModel(self.K(p), self.dist(p), p[2:5], p[5:8], size, name=name)

    def loo(self, p, fit_k1):
        """Left-one-out floor error (cm) of each point: refit without it, then place its click
        on its own height plane."""
        n = len(self.img)
        out = np.full(n, np.nan)
        for i in range(n):
            self.mask = np.ones(n, bool)
            self.mask[i] = False
            q = self.lm(p, self.free(fit_k1), iters=6)
            m = self.model(q, (1, 1))
            x, y = m.to_floor(self.img[i, 0], self.img[i, 1], self.obj[i, 2])
            out[i] = math.hypot(x - self.obj[i, 0], y - self.obj[i, 1])
        self.mask = np.ones(n, bool)
        return out

    def f_sd(self, p, fit_k1):
        """Relative sd of f (Gauss-Newton, click noise CLICK_SD_PX): from the points alone, and
        with the priors (camera height / nadir / weak focal prior) included."""
        if not self.f_free:
            return 0.0, 0.0
        free = self.free(fit_k1)
        out = []
        for pri in (False, True):
            J = self.jac(p, free, priors=pri)
            A = J.T @ J
            try:
                cov = np.linalg.inv(A)
                v = cov[0, 0]
            except np.linalg.LinAlgError:
                v = np.inf
            out.append(math.sqrt(v) if v > 0 and np.isfinite(v) else float("inf"))
        return out[0], out[1]
