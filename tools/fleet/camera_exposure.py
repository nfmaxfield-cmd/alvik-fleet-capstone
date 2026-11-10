#!/usr/bin/env python3
"""Speed up a camera's adaptation to a sudden drop in light (build e14; Nick 10 Nov 17:2x). OFF by default.

When DarkWatch sees a sharp drop, this sets the camera's exposure by hand through v4l2-ctl (the way
tools/pi/camera_focus.py sets its focus): manual exposure, the exposure time scaled up by the drop (normal / now,
capped at the camera's maximum) and the gain raised, instead of waiting for the C920e's own slow auto-exposure; once the
picture is back it hands the camera back to auto-exposure. Every command it sends is logged.

Switch MIRLT_EXPOSURE_NUDGE=1 (default 0: the simulator has no model of a real camera's exposure, so it cannot show
the nudge does no harm; a board test decides: DOE row "light drop against adaptation time, with and without the
nudge"). Never run against a real device by a build: tests stub v4l2-ctl.
"""
import os
import shutil
import subprocess

VERSION = "camera-exposure-2026-11-10e14"
EXPOSURE_MAX_X = 8.0     # CHOSEN: never more than 8x the exposure before the drop (motion blur grows with it; at
                         # 5 cm/s a robot moves 1.6 cm in 1/30 s x 8). DOE row (e14)
GAIN_UP = 1.5            # CHOSEN: the gain raised by half (noise grows with it). DOE row (e14)


def enabled():
    return os.environ.get("MIRLT_EXPOSURE_NUDGE", "0") == "1"


class Nudger:
    """dev_of(cam) -> "/dev/videoN" or None; run(cmd list) -> (rc, stdout) (stubbed in tests)."""

    def __init__(self, dev_of, run=None, log=print):
        self.dev_of, self.log = dev_of, log
        self.run = run or self._run
        self.saved, self.nudged, self.sent = {}, {}, []

    def _run(self, cmd):
        if not shutil.which(cmd[0]):
            return 127, ""
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        return r.returncode, r.stdout

    def _get(self, dev, ctrl):
        rc, out = self.run(["v4l2-ctl", "-d", dev, "--get-ctrl=%s" % ctrl])
        try:
            return float(out.strip().split(":")[-1]) if rc == 0 else None
        except ValueError:
            return None

    def _set(self, dev, ctrl, v):
        cmd = ["v4l2-ctl", "-d", dev, "--set-ctrl=%s=%d" % (ctrl, int(round(v)))]
        self.sent.append(cmd)
        return self.run(cmd)[0] == 0

    def dark(self, cam, grey, normal):
        dev = self.dev_of(cam)
        if not dev or cam in self.nudged:
            return False
        ex = self._get(dev, "exposure_absolute") or self._get(dev, "exposure_time_absolute")
        gain = self._get(dev, "gain")
        if ex is None:
            self.log("EXPOSURE: camera %s: its exposure cannot be read (%s): left to its own auto-exposure" % (cam, dev))
            return False
        self.saved[cam] = {"exposure": ex, "gain": gain}
        k = min(EXPOSURE_MAX_X, max(1.0, normal / max(1.0, grey)))
        ok = self._set(dev, "exposure_auto", 1)                       # manual
        ok = self._set(dev, "exposure_absolute", ex * k) and ok
        if gain is not None:
            self._set(dev, "gain", min(255.0, gain * GAIN_UP))
        self.nudged[cam] = ok
        self.log("EXPOSURE: camera %s nudged: exposure %.0f -> %.0f (x%.1f)%s" % (
            cam, ex, ex * k, k, "" if gain is None else ", gain %.0f -> %.0f" % (gain, min(255.0, gain * GAIN_UP))))
        return ok

    def back(self, cam):
        dev = self.dev_of(cam)
        if not dev or cam not in self.nudged:
            return False
        sv = self.saved.pop(cam, {})
        if sv.get("gain") is not None:
            self._set(dev, "gain", sv["gain"])
        ok = self._set(dev, "exposure_auto", 3)                       # back to the camera's own auto-exposure
        self.nudged.pop(cam, None)
        self.log("EXPOSURE: camera %s handed back to its own auto-exposure" % cam)
        return ok
