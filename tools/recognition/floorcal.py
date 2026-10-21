"""Floor calibration made in the studio by clicking on the picture: which pixel is which spot on the floor.

Two ways (the studio's Floor calibration screen, More menu):

  rectangle   the easy way: something rectangular of known size lies flat on the floor (a sheet of
              paper, a board, a mat, tape marks). Click its 4 corners in order around it and type its
              width (corner 1 to corner 2) and length (corner 1 to corner 4) in cm. Floor x runs from
              corner 1 to corner 2, floor y from corner 1 to corner 4, 0, 0 at corner 1.
  points      the careful way: 6-30 spots on the floor whose x, y (cm) you measured from any corner of
              your choice; click each spot and type its x and y.

Either way the camera is fitted with cameras.CameraModel.from_points (the pinhole model the rest of
the studio uses), trying both ways round for y (a floor measured with y running the "other" way is a
mirror image of what a camera above the floor sees; the way that puts the camera above the floor is
kept, nothing to do for the person). The result says the fit error in cm, in plain words, and gives
lines of a floor grid in picture pixels so the page can draw it for the person to check by eye.

Stored where every reader already looks (cameras.py, crosscam.py, outside data, use_model.py,
referee.py): the project's cameras/<camera>.json, as a camera model file (K, rvec, tvec ...), with the
clicks kept under "studio_calibration" so the screen can show them again.

    res = fit(dict(mode="rectangle", corners_px=[[..], ..4], width_cm=29.7, length_cm=21.0), (640, 480))
    res["ok"], res["err_cm"], res["text"], res["grid"]       # nothing saved
    before = save(proj, "default", spec, (640, 480))          # writes cameras/default.json; before: the old file
    put(proj, "default", before)                              # puts the old file back (None: removes it)
"""
import datetime as dt
import json
import math
import os

import numpy as np

try:
    from .cameras import CameraModel, CalibrationError
except ImportError:
    from cameras import CameraModel, CalibrationError

MIN_POINTS = 6
MAX_POINTS = 30
RECT_GRID = 5               # virtual points per side of the rectangle (the 4 clicks fix the floor exactly)
NICE_STEPS = (1, 2, 5, 10, 20, 25, 50, 100, 200, 500)


def _verdict(err_cm):
    if err_cm is None or not math.isfinite(err_cm):
        return "the fit could not be measured"
    if err_cm <= 0.5:
        return "very good"
    if err_cm <= 1.5:
        return "good"
    if err_cm <= 3.0:
        return "usable: check that the grid lies on the floor's lines"
    return "something is off: check the clicks and the numbers"


def _area_size(pts):
    pts = np.asarray(pts, float)
    return float(np.ptp(pts[:, 0])), float(np.ptp(pts[:, 1]))


def _crosses(quad):
    """True when the 4 clicks, joined in order, cross themselves (not clicked in order around)."""
    q = np.asarray(quad, float)

    def ccw(a, b, c):
        return (c[1] - a[1]) * (b[0] - a[0]) - (b[1] - a[1]) * (c[0] - a[0])

    def inter(a, b, c, d):
        return ccw(a, b, c) * ccw(a, b, d) < 0 and ccw(c, d, a) * ccw(c, d, b) < 0
    return inter(q[0], q[1], q[2], q[3]) or inter(q[1], q[2], q[3], q[0])


def _num(v, what):
    try:
        x = float(v)
    except (TypeError, ValueError):
        raise ValueError("%s is not a number (%r). Nothing was saved." % (what, v))
    if not math.isfinite(x):
        raise ValueError("%s is not a number. Nothing was saved." % what)
    return x


def _clean_points(px, what):
    out = []
    for i, p in enumerate(px or []):
        if p is None or len(p) < 2:
            raise ValueError("%s %d has no place on the picture yet: click it. Nothing was saved." % (what, i + 1))
        out.append([_num(p[0], "%s %d (picture)" % (what, i + 1)), _num(p[1], "%s %d (picture)" % (what, i + 1))])
    return out


def fit(spec, image_size):
    """Fit a floor calibration from the screen's clicks (nothing is saved). spec: dict(mode="rectangle",
    corners_px=[4 x [u, v]], width_cm, length_cm[, height_cm]) or dict(mode="points", points_px=[n x [u, v]],
    points_cm=[n x [x, y]][, height_cm]). Returns dict(ok, model (CameraModel), err_cm, max_err_cm,
    worst, points (per click: px, cm, err_cm), mirrored, grid, axes, verdict, text, notes, spec).
    Raises ValueError with plain words when the input can't be fitted."""
    W, H = int(image_size[0]), int(image_size[1])
    mode = (spec or {}).get("mode") or "rectangle"
    hc = spec.get("height_cm")
    height = None
    if hc not in (None, ""):
        height = _num(hc, "The lens height")
        if height <= 0:
            raise ValueError("The lens height must be above 0 cm (or leave it blank). Nothing was saved.")
    notes = []
    if mode == "rectangle":
        corners = _clean_points(spec.get("corners_px"), "Corner")
        if len(corners) != 4:
            raise ValueError("Click the rectangle's 4 corners first (%d clicked). Nothing was saved." % len(corners))
        w = _num(spec.get("width_cm"), "The width")
        ln = _num(spec.get("length_cm"), "The length")
        if w <= 0 or ln <= 0:
            raise ValueError("Type the rectangle's width and length in cm (both above 0). Nothing was saved.")
        if _crosses(corners):
            raise ValueError("The corners cross over each other: click them in order around the rectangle (corner 1, "
                             "then the next corner along one side, and so on). Nothing was saved.")
        floor = np.array([[0.0, 0.0], [w, 0.0], [w, ln], [0.0, ln]])
        import cv2
        Hm = cv2.getPerspectiveTransform(floor.astype(np.float32), np.asarray(corners, np.float32))
        gx, gy = np.meshgrid(np.linspace(0, w, RECT_GRID), np.linspace(0, ln, RECT_GRID))
        fg = np.column_stack([gx.ravel(), gy.ravel()])
        hp = np.hstack([fg, np.ones((len(fg), 1))]) @ Hm.T
        if np.any(hp[:, 2] * hp[0, 2] <= 0):
            raise ValueError("These 4 corners can't be a flat rectangle seen by a camera: check the clicks. Nothing "
                             "was saved.")
        ig = hp[:, :2] / hp[:, 2:3]
        try:
            model = CameraModel.from_points(ig, fg, (W, H), height_cm=height, k1="off", mirror="auto")
        except CalibrationError as e:
            raise ValueError("No camera fits these corners (%s): check the clicks and the numbers. Nothing was saved." % e)
        img_pts, cm_pts = np.asarray(corners, float), floor
        x, y = model.to_floor(ig[:, 0], ig[:, 1])
        grid_err = np.hypot(x - fg[:, 0], y - fg[:, 1])
        notes.append("With 4 corners a wrong click or a wrong number can only partly show in the error: the grid "
                     "on the picture is the real check (its lines should lie along the floor, square to the "
                     "rectangle's sides).")
    elif mode == "points":
        img_pts = np.asarray(_clean_points(spec.get("points_px"), "Point"), float).reshape(-1, 2)
        cm = spec.get("points_cm") or []
        if len(cm) != len(img_pts):
            raise ValueError("Every clicked point needs its measured x and y (%d clicked, %d typed). Nothing was saved."
                             % (len(img_pts), len(cm)))
        cm_pts = np.array([[_num(c[0] if c else None, "x of point %d" % (i + 1)),
                            _num(c[1] if c and len(c) > 1 else None, "y of point %d" % (i + 1))]
                           for i, c in enumerate(cm)], float).reshape(-1, 2)
        if len(img_pts) < MIN_POINTS:
            raise ValueError("Click at least %d points (%d so far): fewer can't show whether a click or a number is "
                             "wrong. Nothing was saved." % (MIN_POINTS, len(img_pts)))
        if len(img_pts) > MAX_POINTS:
            raise ValueError("At most %d points (there are %d): remove some. Nothing was saved." % (MAX_POINTS, len(img_pts)))
        d = np.hypot(*(cm_pts[:, None, :] - cm_pts[None, :, :]).transpose(2, 0, 1))
        np.fill_diagonal(d, np.inf)
        if d.min() < 1e-6:
            i, j = np.unravel_index(np.argmin(d), d.shape)
            raise ValueError("Points %d and %d have the same x and y: check the numbers. Nothing was saved."
                             % (min(i, j) + 1, max(i, j) + 1))
        try:
            model = CameraModel.from_points(img_pts, cm_pts, (W, H), height_cm=height, k1="auto", mirror="auto")
        except CalibrationError as e:
            if "one line" in str(e):
                raise ValueError("The points lie (nearly) on one line: add points away from that line. Nothing was saved.")
            raise ValueError("No camera fits these points (%s): check the clicks and the numbers. Nothing was saved." % e)
        grid_err = None
    else:
        raise ValueError("Unknown kind of floor calibration %r." % mode)

    x, y = model.to_floor(img_pts[:, 0], img_pts[:, 1])
    err = np.hypot(x - cm_pts[:, 0], y - cm_pts[:, 1])
    pts = [dict(px=[round(float(img_pts[i, 0]), 1), round(float(img_pts[i, 1]), 1)],
                cm=[round(float(cm_pts[i, 0]), 2), round(float(cm_pts[i, 1]), 2)],
                err_cm=None if not math.isfinite(err[i]) else round(float(err[i]), 2)) for i in range(len(err))]
    if mode == "points":
        loo = [p.get("loo_cm") for p in model.fit.get("points") or []]
        for p, l in zip(pts, loo):
            p["loo_cm"] = l
        # judged on each point left out in turn (an honest error: a point can't hide its own mistake)
        e_all = np.array([p["loo_cm"] if p.get("loo_cm") is not None else p["err_cm"] for p in pts], float)
    else:
        e_all = np.concatenate([err, grid_err])
    e_all = e_all[np.isfinite(e_all)]
    rms = float(np.sqrt(np.mean(e_all ** 2))) if len(e_all) else float("nan")
    mx = float(e_all.max()) if len(e_all) else float("nan")
    worst = None
    if mode == "points":
        key = [p.get("loo_cm") if p.get("loo_cm") is not None else (p["err_cm"] or 0) for p in pts]
        k = int(np.argmax(key))
        if key[k] > max(1.5, 3 * float(np.median(key))):
            worst = dict(index=k, err_cm=round(float(key[k]), 2))
    mirrored = bool(model.mirror_y)
    grid, axes = grid_lines(model, cm_pts, (W, H))
    fsrc = model.fit.get("f_source") or ""
    if "NOT measured" in fsrc:
        notes.append("The camera looks nearly straight down, so its height above the floor was guessed (about %.0f cm). "
                     "Spots on the floor are placed right; for things above the floor (the top of a robot) type the "
                     "lens height above the floor (measured with a tape) and check again." % model.center[2])
    else:
        h = model.center[2]
        if math.isfinite(h):
            notes.append("The camera's lens is about %.0f cm above the floor (from the fit)%s." % (
                h, "" if height is None else "; you typed %g cm" % height))
    if height is not None and abs(model.center[2] - height) > 5:
        notes.append("The fitted lens height (%.0f cm) does not match the %g cm you typed: check the tape height "
                     "or the clicks." % (model.center[2], height))
    if worst:
        notes.append("Point %d is %.1f cm off from where the others put it: check its click and its numbers."
                     % (worst["index"] + 1, worst["err_cm"]))
    verdict = _verdict(rms)
    text = ("Fit error about %.1f cm (largest %.1f cm): %s." % (rms, mx, verdict)) if math.isfinite(rms) else verdict
    return dict(ok=bool(math.isfinite(rms) and rms <= 3.0 and not worst), model=model, err_cm=round(rms, 2),
                max_err_cm=round(mx, 2), worst=worst, points=pts, mirrored=mirrored, grid=grid, axes=axes,
                verdict=verdict, text=text, notes=notes, spec=_spec_copy(spec, mode),
                axes_words=AXES_WORDS[mode] + (MIRROR_WORDS if mirrored else ""))


AXES_WORDS = dict(
    rectangle=("Floor x and y are in cm on the floor, from corner 1: x runs from corner 1 to corner 2, y from corner 1 "
               "to corner 4. A positions file uses the same x and y."),
    points=("Floor x and y are your own measurements in cm, from the corner you measured from. A positions file uses "
            "the same x and y."))
MIRROR_WORDS = (" (Your y runs the other way round from the usual one, like a mirror image; that is found by itself and "
                "needs nothing from you.)")


def _spec_copy(spec, mode):
    keep = ("mode", "corners_px", "width_cm", "length_cm", "points_px", "points_cm", "height_cm")
    out = {k: spec.get(k) for k in keep if spec.get(k) not in (None, "")}
    out["mode"] = mode
    return out


def _nice_step(span):
    want = max(span, 1e-6) / 5.0
    for s in NICE_STEPS:
        if s >= want:
            return float(s)
    return float(NICE_STEPS[-1])


def grid_lines(model, cm_pts, image_size, n_samples=40):
    """Lines of a floor grid as picture polylines: [{"pts": [[u, v], ...], "x": value or None, "y": ...}].
    The grid covers the clicked area and half as much again around it, in round steps, kept to the
    part of the floor that is in the picture. axes: picture points of 0, 0 and two steps along x and y."""
    W, H = image_size
    cm = np.asarray(cm_pts, float)
    x0, y0 = cm.min(0)
    x1, y1 = cm.max(0)
    step = _nice_step(max(x1 - x0, y1 - y0))
    mx, my = 0.5 * (x1 - x0) + step, 0.5 * (y1 - y0) + step
    gx0, gx1 = math.floor((x0 - mx) / step) * step, math.ceil((x1 + mx) / step) * step
    gy0, gy1 = math.floor((y0 - my) / step) * step, math.ceil((y1 + my) / step) * step
    lines = []

    def poly(xs, ys):
        u, v = model.project(np.column_stack([xs, ys, np.zeros(len(xs))]))
        ok = np.isfinite(u) & np.isfinite(v) & (u >= -W) & (u <= 2 * W) & (v >= -H) & (v <= 2 * H)
        segs, cur = [], []
        for k in range(len(xs)):
            if ok[k]:
                cur.append([round(float(u[k]), 1), round(float(v[k]), 1)])
            elif cur:
                segs.append(cur)
                cur = []
        if cur:
            segs.append(cur)
        return [s for s in segs if len(s) >= 2]
    for gx in np.arange(gx0, gx1 + step / 2, step):
        for s in poly(np.full(n_samples, gx), np.linspace(gy0, gy1, n_samples)):
            lines.append(dict(pts=s, x=round(float(gx), 2), y=None))
    for gy in np.arange(gy0, gy1 + step / 2, step):
        for s in poly(np.linspace(gx0, gx1, n_samples), np.full(n_samples, gy)):
            lines.append(dict(pts=s, x=None, y=round(float(gy), 2)))
    o = model.project((0.0, 0.0, 0.0))
    ax = model.project((2 * step, 0.0, 0.0))
    ay = model.project((0.0, 2 * step, 0.0))
    fin = lambda p: [round(p[0], 1), round(p[1], 1)] if all(map(math.isfinite, p)) else None   # noqa: E731
    return dict(lines=lines, step_cm=step), dict(origin=fin(o), x=fin(ax), y=fin(ay), step_cm=step)


# =============================================================================== stored file
def path_for(proj, key):
    return proj.camera_model_path(key or "default")


def summary(proj, key):
    """What the screen shows about this camera's calibration now: None, or dict(path, method, err_cm,
    mirrored, made, spec (the clicks, when made here), points)."""
    p = path_for(proj, key)
    if not p or not os.path.exists(p):
        return None
    try:
        with open(p, encoding="utf-8") as f:
            d = json.load(f)
    except (OSError, ValueError) as e:
        return dict(path=p, broken="the file could not be read (%s)" % e)
    sc = d.get("studio_calibration") or {}
    fit_ = d.get("fit") or {}
    return dict(path=p, method=sc.get("mode") or fit_.get("method") or "made elsewhere", err_cm=sc.get("err_cm"),
                max_err_cm=sc.get("max_err_cm"), mirrored=bool(d.get("mirror_y")), made=sc.get("made") or d.get("created"),
                spec=sc.get("spec"), points=sc.get("points"), image_size=d.get("image_size"),
                made_here=bool(sc), loo_rms_cm=fit_.get("loo_rms_cm"))


def read_file(proj, key):
    p = path_for(proj, key)
    if not p or not os.path.exists(p):
        return None
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def save(proj, key, spec, image_size):
    """Fit and write cameras/<key>.json. Returns (result of fit, the file as it was before or None)."""
    import project as projmod
    res = fit(spec, image_size)
    before = read_file(proj, key)
    d = res["model"].to_dict()
    d["name"] = key
    d["studio_calibration"] = dict(
        spec=res["spec"], mode=res["spec"]["mode"], err_cm=res["err_cm"], max_err_cm=res["max_err_cm"],
        mirrored=res["mirrored"], points=res["points"], made=dt.datetime.now().isoformat(timespec="seconds"),
        note="made in the studio's Floor calibration screen (tools/recognition/floorcal.py)")
    p = path_for(proj, key)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    projmod.write_json(p, d)
    return res, before


def put(proj, key, data):
    """Put a calibration file back as it was (undo / redo): data is the file's content, or None to remove
    it. Returns the file as it was before this."""
    import project as projmod
    p = path_for(proj, key)
    before = read_file(proj, key)
    if data is None:
        if os.path.exists(p):
            os.remove(p)
        return before
    CameraModel.from_dict(data)          # a plain error now if it isn't a camera file
    os.makedirs(os.path.dirname(p), exist_ok=True)
    projmod.write_json(p, data)
    return before


def public(res):
    """The fit result for the page (no model object)."""
    return {k: v for k, v in res.items() if k != "model"}
