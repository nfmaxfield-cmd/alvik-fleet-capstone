#!/usr/bin/env python3
"""A picture that goes dark is a CAMERA problem, not a lost robot (build e14; Nick at school, 10 Nov 17:2x).

The room lights went out mid-calibration; the C920e's picture went black while pictures kept coming; a couple of
minutes later the TV's light and the camera's own auto-exposure brought the table back, but the run had already ended
and the robot was asleep.

DarkWatch (in the tag bridge, on every picture's mean grey): a picture far darker than the camera's own recent normal
(under DARK_FRAC of it, or under DARK_ABS grey) while pictures still arrive is "dark": the bridge says
"THE PICTURE WENT DARK: the camera is adjusting to the light" and sends {"type": "camera", "state": "dark"} to the run;
every run holds its robots still and safe (nothing ends, nothing sleeps) and carries on by itself once the brightness
has settled (RECOVER_FRAC of the normal for SETTLE_S) with "light_back" and how long the camera took. Every event goes
to dark_events.csv (the drop's size against the time to adapt: a DOE row), and <conf dir>/camera_dark.json tells the
idle sleeper (idle_after_run) not to count the dark time.

Optional (camera_exposure.py, MIRLT_EXPOSURE_NUDGE=1, off by default): on a sharp drop, nudge the camera's exposure
through v4l2 instead of waiting for its slow auto-exposure, then hand back to auto.
"""
import json
import os
import time

VERSION = "dark-watch-2026-11-10e14"
DARK_FRAC = 0.35         # CHOSEN: under 35 % of the camera's own normal brightness = dark (the school's lights-off picture
                         # went from about 91 to near 0; the dim room, 70 of 91 = 77 %, is NOT dark). DOE row (e14)
DARK_ABS = 12.0          # CHOSEN: under this mean grey the picture is dark whatever the normal (tags cannot read). DOE
RECOVER_FRAC = 0.6       # CHOSEN: back once 60 % of the normal (the dim room's 77 % passes), steady SETTLE_S. DOE row
SETTLE_S = 2.0           # CHOSEN: the brightness must stay back this long (auto-exposure overshoots). DOE row
NORMAL_N = 60            # the normal = the median of the last 60 non-dark samples (about 15 s at 4 a second). Not tuned
SAMPLE_S = 0.25          # brightness sampled this often per camera (cheap: every 8th pixel). Not tuned


def conf_dir():
    return os.path.expanduser(os.environ.get("MIRLT_CONF_DIR") or "~/.config/mirlt")


def state_path():
    return os.path.join(conf_dir(), "camera_dark.json")


def is_dark_now(max_age_s=30.0, path=None):
    """(the idle sleeper) True while a bridge says a picture is dark (its state written within max_age_s)."""
    p = path or state_path()
    try:
        with open(p) as f:
            d = json.load(f)
        return bool(d.get("dark")) and time.time() - float(d.get("updated") or 0) < max_age_s
    except (OSError, ValueError):
        return False


class DarkWatch:
    def __init__(self, send=None, event=None, say=None, out="", nudge=None, state_file=None):
        self.send = send or (lambda m: None)
        self.event = event or (lambda kind, **kw: None)
        self.say = say or (lambda s: None)
        self.out, self.nudge = out, nudge
        self.state_file = state_file if state_file is not None else state_path()
        self.cams = {}

    def _st(self, cam):
        return self.cams.setdefault(cam, {"normal": [], "dark": False, "t_dark": None, "t_ok": None, "min": None,
                                          "before": None, "t": -1e9, "said_t": 0.0})

    def normal(self, cam):
        n = self._st(cam)["normal"]
        return sorted(n)[len(n) // 2] if len(n) >= 8 else None

    def saw(self, cam, mean_grey, t):
        st = self._st(cam)
        if t - st["t"] < SAMPLE_S:
            return None
        st["t"] = t
        g = float(mean_grey)
        base = self.normal(cam)
        if not st["dark"]:
            if base is not None and (g < DARK_FRAC * base or g < DARK_ABS):
                st.update(dark=True, t_dark=t, t_ok=None, min=g, before=base, said_t=t)
                w = ("THE PICTURE WENT DARK on camera %s (mean grey %.0f, normally %.0f): the camera is adjusting to the "
                     "light (room lights off?). Every robot holds still and safe; the run carries on by itself when the "
                     "picture is back" % (cam, g, base))
                self.say(w)
                self.event("picture_dark", cam=cam, grey=round(g, 1), normal=round(base, 1))
                self.send({"type": "camera", "cam": cam, "state": "dark", "words": w})
                self._write(True, cam, t)
                if self.nudge is not None:
                    try:
                        self.nudge.dark(cam, g, base)
                    except Exception as e:  # noqa: BLE001  (a nudge never stops the bridge)
                        self.event("exposure_nudge_error", why=str(e))
                return "dark"
            st["normal"].append(g)
            del st["normal"][:-NORMAL_N]
            return None
        st["min"] = min(st["min"], g)
        if g >= RECOVER_FRAC * st["before"] and g >= DARK_ABS:
            if st["t_ok"] is None:
                st["t_ok"] = t
            if t - st["t_ok"] >= SETTLE_S:
                took = st["t_ok"] - st["t_dark"]
                st["dark"] = False
                w = ("THE PICTURE IS BACK on camera %s after %.0f s (mean grey %.0f; was %.0f before, %.0f at its "
                     "darkest): the run carries on" % (cam, took, g, st["before"], st["min"]))
                self.say(w)
                self.event("picture_back", cam=cam, adapt_s=round(took, 1), grey=round(g, 1))
                self.send({"type": "camera", "cam": cam, "state": "light_back", "adapt_s": round(took, 1),
                           "words": w})
                self._log(cam, st, g, took)
                self._write(False, cam, t)
                if self.nudge is not None:
                    try:
                        self.nudge.back(cam)
                    except Exception as e:  # noqa: BLE001
                        self.event("exposure_nudge_error", why=str(e))
                st["normal"] = [g] * 8          # the normal starts again from the light as it is now
                return "back"
        else:
            st["t_ok"] = None
        if t - st["said_t"] >= 30.0:
            st["said_t"] = t
            self.say("  camera %s still dark after %.0f s (mean grey %.0f): every robot still holding" % (
                cam, t - st["t_dark"], g))
            self._write(True, cam, t)
        return None

    def _log(self, cam, st, g, took):
        if not self.out:
            return
        p = os.path.join(self.out, "dark_events.csv")
        new = not os.path.exists(p)
        with open(p, "a") as f:
            if new:
                f.write("camera,t_dark,adapt_s,grey_before,grey_min,grey_after,drop_frac,nudged\n")
            f.write("%s,%.2f,%.1f,%.1f,%.1f,%.1f,%.3f,%d\n" % (
                cam, st["t_dark"], took, st["before"], st["min"], g, 1.0 - st["min"] / max(1e-6, st["before"]),
                int(self.nudge is not None and getattr(self.nudge, "nudged", {}).get(cam, False))))

    def _write(self, dark, cam, t):
        if not self.state_file:
            return
        try:
            os.makedirs(os.path.dirname(self.state_file), exist_ok=True)
            with open(self.state_file, "w") as f:
                json.dump({"dark": bool(dark), "cam": cam, "since": t if dark else None, "updated": time.time()}, f)
        except OSError:
            pass
