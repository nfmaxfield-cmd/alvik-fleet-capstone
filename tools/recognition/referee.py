"""Second camera as a referee for Investigate: the other camera's view of the same floor spot at
the same moment.

An object standing up shows in the other camera at the matching spot; a flat mark on the floor
(a stain, a reflection) looks flat there, or doesn't show at all. Needs both cameras of a
two-camera session calibrated to the floor (cameras/<name>.json, CROSSCAM.md); otherwise it says
so in plain words and the rest of Investigate works as before.

    look(project, recording entry, frame, x, y, cls) -> dict(available, reason | picture, text, ...)
"""
import base64
import os

import numpy as np

HALF = 64          # half-size of the picture of the other camera, px


def _unavailable(reason):
    return dict(available=False, reason=reason)


def _other_camera(proj, rec):
    for other in proj.recordings():
        if other["id"] == rec["id"] or other.get("session") != rec.get("session"):
            continue
        p = proj.camera_model_path(other.get("camera"))
        if p and os.path.exists(p):
            return other, p
    return None, None


def look(proj, rec, frame, x, y, cls=None):
    import cv2
    import cameras
    import crosscam
    if not rec.get("session"):
        return _unavailable("This recording has no second camera (it isn't part of a two-camera session).")
    mine = proj.camera_model_path(rec.get("camera"))
    if not mine or not os.path.exists(mine):
        return _unavailable("This camera isn't calibrated to the floor yet, so the other camera can't be matched "
                            "to this spot (see Two cameras in the manual).")
    other, path_b = _other_camera(proj, rec)
    if other is None:
        return _unavailable("The other camera of this session isn't calibrated to the floor yet (see Two cameras in "
                            "the manual).")
    video_b = os.path.join(other["path"], "video.avi")
    if not os.path.exists(video_b):
        return _unavailable("The other camera's recording can't be found (%s)." % other["path"])
    cam_a = cameras.CameraModel.load(mine)
    cam_b = cameras.CameraModel.load(path_b)
    types = {c["name"]: c for c in proj.classes}
    c = types.get(cls) or {}
    varies = bool(c.get("height_varies")) or not c or c.get("height_cm") is None    # blank height: floor spot only
    z = 0.0 if varies else float(c.get("height_cm") or 0.0)
    fx, fy = cam_a.to_floor(float(x), float(y), z)
    if not (np.isfinite(fx) and np.isfinite(fy)):
        return _unavailable("This spot can't be placed on the floor with the calibration (it is above the horizon?).")
    ub, vb = cam_b.project((fx, fy, z))
    u0, v0 = cam_b.project((fx, fy, 0.0))
    if not cam_b.in_image(ub, vb):
        return _unavailable("The other camera doesn't see this spot (it is outside its picture).")
    times_a = crosscam.load_frame_times(os.path.join(rec["path"], "frames.csv"))
    times_b = crosscam.load_frame_times(os.path.join(other["path"], "frames.csv"))
    m = crosscam.match_frames(times_b, {int(frame): times_a.get(int(frame))} if int(frame) in times_a else {},
                              return_dt=True)
    if int(frame) not in m:
        return _unavailable("The other camera has no picture from the same moment.")
    fb, dt = m[int(frame)]
    cap = cv2.VideoCapture(video_b)
    cap.set(cv2.CAP_PROP_POS_FRAMES, fb)
    ok, img = cap.read()
    cap.release()
    if not ok:
        return _unavailable("The other camera's video could not be read at that moment.")
    pad = HALF + 2
    big = cv2.copyMakeBorder(img, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    cu, cv_ = int(round(ub)), int(round(vb))
    crop = big[cv_ + pad - HALF:cv_ + pad + HALF + 1, cu + pad - HALF:cu + pad + HALF + 1].copy()
    scale = 2
    crop = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    centre = (HALF * scale + scale // 2, HALF * scale + scale // 2)
    try:
        r_px = max(6, int(round(2.5 * cam_b.px_per_cm(fx, fy, z) * scale)))
    except Exception:
        r_px = 14
    cv2.circle(crop, centre, min(r_px, HALF * scale - 2), (255, 182, 108), 2)
    if not varies and z > 0 and np.isfinite(u0):
        fu = int(round((u0 - ub) * scale)) + centre[0]
        fv = int(round((v0 - vb) * scale)) + centre[1]
        cv2.drawMarker(crop, (fu, fv), (120, 120, 255), cv2.MARKER_CROSS, 12, 2)
    ok, buf = cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, 85])
    name = other.get("camera") or other["name"]
    if varies:
        text = ("The other camera (%s) at the same moment (%.2f s apart). The circle is the floor spot under this "
                "point (this type's height varies, so only the floor spot is known). An object standing there "
                "shows up near the circle; a flat mark on the floor looks flat, or isn't visible." % (name, abs(dt)))
    else:
        text = ("The other camera (%s) at the same moment (%.2f s apart). The circle is where a %s would be (its "
                "marked point %.1f cm above the floor); the cross is the floor right under it. A real %s shows up at "
                "the circle; a flat mark on the floor would only show at the cross, flat, or not at all."
                % (name, abs(dt), cls, z, cls))
    return dict(available=True, camera=name, frame=fb, dt_s=round(dt, 3), point=[round(ub, 1), round(vb, 1)],
                picture="data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("ascii"), text=text)
