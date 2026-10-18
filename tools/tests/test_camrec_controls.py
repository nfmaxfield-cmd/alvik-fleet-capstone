"""tools/pi/camrec.py camera controls, with a fake camera (no hardware)."""
import cv2
import numpy as np

import camrec


class FakeCap:
    """Acts like OpenCV's V4L2 capture of a UVC webcam: auto exposure brightens the
    picture to ~120; in manual mode brightness follows the exposure value. Reading
    the exposure while auto is on returns `stale` (some webcams don't update it)."""

    def __init__(self, supported=None, stale=None, refuse=()):
        P = cv2
        self.p = {P.CAP_PROP_AUTO_EXPOSURE: 3.0, P.CAP_PROP_EXPOSURE: 157.0, P.CAP_PROP_AUTOFOCUS: 1.0,
                  P.CAP_PROP_FOCUS: 40.0, P.CAP_PROP_AUTO_WB: 1.0, P.CAP_PROP_WB_TEMPERATURE: 4600.0,
                  P.CAP_PROP_GAIN: 0.0, P.CAP_PROP_BRIGHTNESS: 0.0}
        self.supported = set(self.p) if supported is None else set(supported)
        self.refuse = set(refuse)
        self.auto_exposure_now = 80.0       # what auto exposure settled on
        self.stale = stale
        self.sets = []

    def get(self, prop):
        if prop not in self.supported:
            return -1.0
        if prop == cv2.CAP_PROP_EXPOSURE and self.p[cv2.CAP_PROP_AUTO_EXPOSURE] == 3.0:
            return self.stale if self.stale is not None else self.auto_exposure_now
        return self.p[prop]

    def set(self, prop, v):
        self.sets.append((prop, v))
        if prop not in self.supported or prop in self.refuse:
            return False
        self.p[prop] = float(v)
        return True

    def read(self):
        if self.p[cv2.CAP_PROP_AUTO_EXPOSURE] == 3.0:
            b = 120.0
        else:
            b = min(255.0, 120.0 * self.p[cv2.CAP_PROP_EXPOSURE] / self.auto_exposure_now)
        img = np.full((48, 64, 3), b, np.uint8)
        img[::8, :] = min(255, b + 60)        # some texture for the sharpness measure
        return True, img


def clock():
    t = [0.0]

    def now():
        t[0] += 0.05
        return t[0]
    return now


def run(cap, **kw):
    lines = []
    rep = camrec.apply_controls(cap, "0", log=lines.append, run=lambda *a, **k: (_ for _ in ()).throw(OSError()),
                                now=clock(), **kw)
    return rep, lines


def test_lock_freezes_auto_values():
    cap = FakeCap()
    rep, lines = run(cap)
    assert cap.p[cv2.CAP_PROP_AUTO_EXPOSURE] == camrec.AE_MANUAL and cap.p[cv2.CAP_PROP_EXPOSURE] == 80.0
    assert cap.p[cv2.CAP_PROP_AUTOFOCUS] == 0 and cap.p[cv2.CAP_PROP_FOCUS] == 40.0
    assert cap.p[cv2.CAP_PROP_AUTO_WB] == 0 and cap.p[cv2.CAP_PROP_WB_TEMPERATURE] == 4600.0
    assert rep["after"]["exposure"] == 80.0 and not rep["warnings"]
    assert abs(rep["stats_after"]["brightness"] - rep["stats_auto"]["brightness"]) < 3
    assert any(l.startswith("Camera: exposure 80.0") for l in lines)


def test_stale_exposure_reading_is_corrected_by_brightness():
    cap = FakeCap(stale=157.0)               # camera reports its old value while auto runs
    rep, _ = run(cap)
    assert abs(cap.p[cv2.CAP_PROP_EXPOSURE] - 80.0) < 10
    assert rep["exposure_matched_to_auto_brightness"] and not rep["warnings"]


def test_manual_and_auto_values():
    cap = FakeCap()
    run(cap, exposure=200.0, focus="auto", wb=None)
    assert cap.p[cv2.CAP_PROP_EXPOSURE] == 200.0 and cap.p[cv2.CAP_PROP_AUTO_EXPOSURE] == 1
    assert cap.p[cv2.CAP_PROP_AUTOFOCUS] == 1
    assert not any(p == cv2.CAP_PROP_AUTO_WB for p, _ in cap.sets)        # None = untouched
    cap = FakeCap()
    run(cap, exposure="auto", focus=None, wb=None)
    assert cap.p[cv2.CAP_PROP_AUTO_EXPOSURE] == 3


def test_unsupported_controls_fall_back_with_a_message():
    cap = FakeCap(supported=[cv2.CAP_PROP_AUTO_EXPOSURE, cv2.CAP_PROP_EXPOSURE])   # no focus / WB controls
    rep, lines = run(cap)
    assert cap.p[cv2.CAP_PROP_EXPOSURE] == 80.0
    assert any("focus" in w for w in rep["warnings"]) and any("white balance" in w for w in rep["warnings"])
    assert any(l.startswith("  ! ") for l in lines)


def test_old_opencv_manual_value_tried():
    cap = FakeCap()
    real_set = cap.set

    def set_(prop, v):     # this build only accepts 0.25 for manual exposure
        if prop == cv2.CAP_PROP_AUTO_EXPOSURE and v == 1:
            cap.sets.append((prop, v))
            return False
        return real_set(prop, v)
    cap.set = set_
    rep, _ = run(cap, focus=None, wb=None)
    assert cap.p[cv2.CAP_PROP_AUTO_EXPOSURE] == 0.25
    assert rep["steps"][0]["ok"] and len(rep["steps"][0]["tried"]) == 2


def test_v4l2_ctl_fallback_and_listing():
    listing = ("                     auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=3\n"
               "            exposure_time_absolute 0x009a0902 (int)    : min=3 max=2047 step=1 default=250 value=80 "
               "flags=inactive\n"
               "                    focus_absolute 0x009a090a (int)    : min=0 max=250 step=5 default=0 value=40\n")
    calls = []

    class R:
        def __init__(self, out="", code=0):
            self.stdout, self.stderr, self.returncode = out, "", code

    def fake_run(cmd, **k):
        calls.append(cmd)
        return R(listing) if "--list-ctrls" in cmd else R()
    cap = FakeCap(refuse=[cv2.CAP_PROP_FOCUS])        # OpenCV can't set focus; v4l2-ctl can
    rep = camrec.apply_controls(cap, "0", focus="lock", exposure=None, wb=None, log=lambda *_: None, run=fake_run,
                                now=clock())
    assert ["v4l2-ctl", "-d", "/dev/video0", "-c", "focus_absolute=40"] in calls
    assert rep["v4l2_ctl"] == "found" and rep["v4l2_after"]["exposure_time_absolute"]["max"] == 2047
    focus_step = [s for s in rep["steps"] if s["control"] == "focus"][0]
    assert focus_step["ok"] and focus_step["tried"][-1]["via"].startswith("v4l2-ctl")


def test_file_replay_touches_nothing():
    cap = FakeCap()
    rep = camrec.apply_controls(cap, "video.avi", log=lambda *_: None)
    assert "replaying" in rep["note"] and not cap.sets
