#!/usr/bin/env python3
"""Record the overhead webcam on the Raspberry Pi, with a timestamp for every
frame and a live preview in your laptop's browser.

    python3 camrec.py                      # /dev/video0, 640x480, 30 fps
    python3 camrec.py --device 2 --label E1
    python3 camrec.py --no-record          # preview only, for aiming the camera
    python3 camrec.py --exposure 150 --focus 0     # fixed values instead of locking the auto ones
    python3 camrec.py --no-lock            # leave auto exposure / focus / white balance running
    python3 camrec.py --list               # which camera numbers this Pi has
    python3 camrec.py --device 0,2 --names overhead,side --label E1    # two cameras at once

Two cameras: one session folder ~/recordings/<date-time>_<label>/ with cam0/,
cam1/ (each a normal recording folder, as below) and session.json (devices,
roles, ports). Both cameras run as threads of this one process: their t_mono
is the same clock (time.monotonic() minus one shared t0), so frames from the
two cameras can be matched by t_mono directly. Threads, not processes: the heavy
work (USB read, JPEG decode/encode) runs inside OpenCV, which lets other
threads run meanwhile, so two threads use two of the Pi's cores; one process
also means one notes listener and one clock. Camera i's preview and /live are
on port 8000 + i. The cameras are asked for MJPG (--pixfmt): raw YUYV from two
640x480 webcams overflows a USB 2 bus. Lines starting with '!' are warnings:
too few frames per second, lost frames, low disk space.

Exposure, focus and white balance are LOCKED by default: after a 2 s warm-up
the camera's own automatic values are read and frozen (exposure capped at one
frame time, 10000/fps x 100 us, so the frame rate can't drop; the frame rate is
then measured for 2 s and a loud warning given below 90%), so brightness and
sharpness can't drift during a recording (the flash and blink detectors look
for brightness changes). What was asked for, what the camera reports back and
the brightness/sharpness before and after are printed and saved in meta.json
("camera_controls"). See README.md for checking the controls with v4l2-ctl.

Preview: open http://<pi-hostname>.local:8000 on the laptop (e.g.
http://st1maxfini.local:8000). The picture shows the frame rate and the
recording time.

Live link: http://<pi-hostname>.local:8000/live sends every frame (grey JPEG,
quality 95; ?q=80 for less, but 90 or less lost blink pairs in simulation) with its frame number and t_wall (X-Frame, X-Time
headers) for tools/live/track_live.py --stream. It reuses the frame recorded
here (no second camera open) and costs nothing while no one is connected.

Each recording is a folder ~/recordings/<date-time>_<label>/ with:
  video.avi    the frames, Motion-JPEG
  frames.csv   frame, t_wall (Unix seconds), t_mono (seconds since start),
               cam_ms (the camera driver's own capture time, ms; for checking delay)
  events.csv   anything you type while recording, with its time and frame
  meta.json    camera, size, frame rate, host, start time

While it records, type a short note and press Enter to mark a moment, e.g.
"trial 5 run 3F2A-7 start". That is how a video is matched to rows on the
measurement sheet later. tools/run_sheet.py sends these notes automatically
over the network (UDP port --notes-port, default 5007). Ctrl+C stops and saves.

Timestamps are taken when the Pi receives each frame, so they run about one
frame (~33 ms) after the moment the picture was taken. The latency
experiments measure that offset; nothing here assumes it.

Needs OpenCV: sudo apt install -y python3-opencv
"""
import argparse
import csv
import datetime as dt
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import cv2
except ImportError:
    sys.exit("OpenCV is missing. Install it with: sudo apt install -y python3-opencv")


class Latest:
    """The newest preview JPEG, shared with the web server."""

    def __init__(self):
        self.jpeg = None
        self.cond = threading.Condition()

    def put(self, jpeg):
        with self.cond:
            self.jpeg = jpeg
            self.cond.notify_all()

    def wait(self, timeout=2.0):
        with self.cond:
            self.cond.wait(timeout)
            return self.jpeg


class LiveFrames:
    """Every frame for /live (tools/live/track_live.py on the laptop): the
    newest picture with its frame number and t_wall. Costs nothing while no
    one is connected; with a viewer, each frame is turned grey and JPEG-encoded
    in the viewer's own thread (OpenCV lets the recording loop run meanwhile)."""

    def __init__(self):
        self.item = None
        self.cond = threading.Condition()
        self.clients = 0

    def put(self, frame, n, t_wall):
        with self.cond:
            self.item = (frame, n, t_wall)
            self.cond.notify_all()

    def wait_newer(self, n_last, timeout=2.0):
        with self.cond:
            if self.item is None or self.item[1] <= n_last:
                self.cond.wait(timeout)
            return self.item


def live_part(frame, n, t_wall, quality=95):
    """One multipart part of /live: grey JPEG, with the frame number and the Pi time in its headers."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    ok, jpg = cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        return None
    body = jpg.tobytes()
    return (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(body)).encode()
            + b"\r\nX-Frame: " + str(n).encode() + b"\r\nX-Time: " + f"{t_wall:.4f}".encode()
            + b"\r\n\r\n" + body + b"\r\n")


def make_handler(latest, live=None):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path.split("?")[0] == "/live" and live is not None:
                q = 95   # 90 lost blink pairs in simulation (LED-pattern work, Oct 19); 95 kept them
                if "q=" in self.path:
                    try:
                        q = max(30, min(100, int(self.path.split("q=")[1].split("&")[0])))
                    except ValueError:
                        pass
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                with live.cond:
                    live.clients += 1
                last = -1
                try:
                    while True:
                        item = live.wait_newer(last)
                        if item is None or item[1] <= last:
                            continue
                        frame, n, t_wall = item
                        part = live_part(frame, n, t_wall, q)
                        last = n
                        if part:
                            self.wfile.write(part)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
                finally:
                    with live.cond:
                        live.clients -= 1
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    while True:
                        jpeg = latest.wait()
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    return
            elif self.path in ("/", "/index.html"):
                page = (b"<!doctype html><title>Alvik camera</title>"
                        b"<body style='margin:0;background:#111;display:grid;place-items:center;height:100vh'>"
                        b"<img src='/stream' style='max-width:100%;max-height:100vh'></body>")
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(page)
            else:
                self.send_error(404)

    return Handler


def fourcc_str(v):
    """OpenCV's CAP_PROP_FOURCC number as text ('MJPG', 'YUYV'), or None."""
    try:
        v = int(v)
    except (TypeError, ValueError):
        return None
    if v <= 0:
        return None
    s = "".join(chr((v >> 8 * i) & 0xFF) for i in range(4))
    return s if s.isprintable() and s.strip() else None


def open_camera(device, width, height, fps, buffers=4, pixfmt="MJPG", fail=sys.exit):
    src = int(device) if str(device).isdigit() else device
    cap = cv2.VideoCapture(src, cv2.CAP_V4L2) if isinstance(src, int) else cv2.VideoCapture(src)
    if isinstance(src, int):
        # MJPG = the webcam's 30 fps mode, and ~10x less USB traffic than raw YUYV
        # (640x480 YUYV at 30 fps is ~18 MB/s: two of those don't fit on one USB 2 bus)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*pixfmt))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        # Driver buffers. One buffer would keep frames fresh, but on many USB
        # webcams it halves the frame rate or worse (the camera waits for the
        # buffer to come back): measured 12.5 fps instead of 30 on the Vitade.
        # With the recorder keeping up, frames don't queue anyway, and each
        # frame's own driver timestamp is logged (cam_ms) to check.
        if buffers:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, buffers)
    if not cap.isOpened():
        return fail(f"Could not open camera {device}. Find the right number with: python3 tools/pi/camrec.py --list")
    return cap


# ------------------------------------------------------------------ camera controls
# V4L2 values for CAP_PROP_AUTO_EXPOSURE (OpenCV's V4L2 backend passes them through):
AE_MANUAL, AE_AUTO = 1, 3          # 1 = manual, 3 = aperture priority (the webcam's "auto")
AE_MANUAL_OLD = 0.25               # what some older OpenCV builds expect for manual instead
# OpenCV property -> v4l2-ctl control names (newer kernels first, older second)
CONTROLS = {
    "auto_exposure": (cv2.CAP_PROP_AUTO_EXPOSURE, ("auto_exposure", "exposure_auto")),
    "exposure": (cv2.CAP_PROP_EXPOSURE, ("exposure_time_absolute", "exposure_absolute")),
    "autofocus": (cv2.CAP_PROP_AUTOFOCUS, ("focus_automatic_continuous", "focus_auto")),
    "focus": (cv2.CAP_PROP_FOCUS, ("focus_absolute",)),
    "auto_wb": (cv2.CAP_PROP_AUTO_WB, ("white_balance_automatic", "white_balance_temperature_auto")),
    "wb_temperature": (cv2.CAP_PROP_WB_TEMPERATURE, ("white_balance_temperature",)),
    "gain": (cv2.CAP_PROP_GAIN, ("gain",)),
    "brightness": (cv2.CAP_PROP_BRIGHTNESS, ("brightness",)),
}


def v4l2_list(dev, run=subprocess.run):
    """Controls reported by `v4l2-ctl -d <dev> --list-ctrls` as {name: {min, max, value, ...}},
    or None if v4l2-ctl isn't installed or fails."""
    if not shutil.which("v4l2-ctl") and run is subprocess.run:
        return None
    try:
        out = run(["v4l2-ctl", "-d", dev, "--list-ctrls"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    ctrls = {}
    for line in out.splitlines():
        m = re.match(r"\s*(\w+)\s+0x[0-9a-f]+\s+\((\w+)\)\s*:\s*(.*)", line)
        if m:
            d = {"type": m.group(2)}
            for k, v in re.findall(r"(\w+)=(-?\w+)", m.group(3)):
                d[k] = int(v) if re.fullmatch(r"-?\d+", v) else v
            ctrls[m.group(1)] = d
    return ctrls


def v4l2_set(dev, name, value, run=subprocess.run):
    try:
        r = run(["v4l2-ctl", "-d", dev, "-c", f"{name}={int(round(value))}"], capture_output=True, text=True, timeout=5)
        return r.returncode == 0, (r.stderr or "").strip()
    except (OSError, subprocess.SubprocessError) as e:
        return False, str(e)


def read_controls(cap):
    """What the camera reports for each control (None if OpenCV can't read it)."""
    out = {}
    for name, (prop, _) in CONTROLS.items():
        try:
            v = cap.get(prop)
        except Exception:
            v = None
        out[name] = None if v is None or v == -1 else round(float(v), 4)
    return out


def frame_stats(cap, n=8):
    """Mean brightness (0-255) and sharpness (variance of the Laplacian) over n frames."""
    br, sh = [], []
    for _ in range(n):
        ok, f = cap.read()
        if not ok or f is None:
            continue
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) if f.ndim == 3 else f
        g = cv2.resize(g, (160, 120), interpolation=cv2.INTER_AREA)
        br.append(float(g.mean()))
        sh.append(float(cv2.Laplacian(g, cv2.CV_64F).var()))
    if not br:
        return None
    br.sort()
    sh.sort()
    return dict(brightness=round(br[len(br) // 2], 1), sharpness=round(sh[len(sh) // 2], 1))


class Controls:
    """Sets V4L2 camera controls through OpenCV, falls back to v4l2-ctl if OpenCV
    can't, and records what was asked, what was done and what the camera reports."""

    def __init__(self, cap, dev, log=print, run=subprocess.run):
        self.cap, self.dev, self.log, self.run = cap, dev, log, run
        self.v4l2 = v4l2_list(dev, run)
        self.steps, self.warnings = [], []

    def _v4l2_name(self, name):
        if not self.v4l2:
            return None
        return next((n for n in CONTROLS[name][1] if n in self.v4l2), None)

    def set(self, name, value, alt=None):
        """Set a control; returns True if the camera reports the new value."""
        prop = CONTROLS[name][0]
        tried = []
        for v in [value] + ([alt] if alt is not None else []):
            try:
                ok = bool(self.cap.set(prop, v))
            except Exception:
                ok = False
            back = self.cap.get(prop)
            tried.append(dict(via="opencv", value=v, accepted=ok, reads_back=back))
            if ok and back is not None and abs(back - v) <= max(1e-3, 0.02 * abs(v)):
                self.steps.append(dict(control=name, ok=True, tried=tried))
                return True
        vname = self._v4l2_name(name)
        if vname:
            vv = 1 if name == "auto_exposure" and value == AE_MANUAL else value
            ok, err = v4l2_set(self.dev, vname, vv, self.run)
            back = self.cap.get(prop)
            tried.append(dict(via=f"v4l2-ctl {vname}", value=vv, accepted=ok, reads_back=back, error=err or None))
            if ok:
                self.steps.append(dict(control=name, ok=True, tried=tried))
                return True
        self.steps.append(dict(control=name, ok=False, tried=tried))
        self.warnings.append(f"{name}: could not set {value} (camera reports {tried[-1]['reads_back']}); "
                             "check `v4l2-ctl -d " + self.dev + " --list-ctrls`")
        self.log(f"  ! {self.warnings[-1]}")
        return False


def exposure_ms(controls):
    """Exposure in ms when it is locked/manual (V4L2 exposure units are 100 us),
    else None. check_session.py uses it to time the sync flashes precisely."""
    try:
        af = controls.get("after") or {}
        if af.get("auto_exposure") in (AE_MANUAL, AE_MANUAL_OLD, 1) and af.get("exposure") not in (None, -1):
            v = float(af["exposure"])
            return v / 10.0 if v > 0 else None
    except (AttributeError, TypeError, ValueError):
        pass
    return None


def max_exposure(fps):
    """Longest exposure (V4L2 units of 100 us) that still fits in one frame at fps:
    10000 / fps (333 at 30 fps). Longer, and the webcam slows its frame rate."""
    return 10000.0 / fps if fps and fps > 0 else None


def measure_fps(cap, seconds=2.0, now=time.monotonic):
    """Frames per second actually delivered over about `seconds`: (frames, s, fps)."""
    t0 = now()
    n, t = 0, t0
    while t - t0 < seconds:
        ok, _ = cap.read()
        t = now()
        if ok:
            n += 1
        elif t - t0 > 2 * seconds:
            break
    dt_ = t - t0
    return n, dt_, (n / dt_ if dt_ > 0 else 0.0)


def apply_controls(cap, device, exposure="lock", focus="lock", wb="lock", warmup_s=2.0, log=print,
                   run=subprocess.run, now=time.monotonic, fps=30, fps_check_s=2.0):
    """Exposure, focus and white balance: 'lock' = after a warm-up, freeze the camera's
    own automatic values; 'auto' = leave/turn automatic on; a number = that manual
    value; None = don't touch. Returns a report for meta.json.

    The frame rate is protected: a locked or brightness-matched exposure is capped at
    one frame time (max_exposure(fps)); exposure_dynamic_framerate is set to 0 with
    v4l2-ctl when the camera has it (it lets auto exposure slow the frame rate); and
    after setting a manual exposure the frame rate is measured for fps_check_s and a
    loud warning given (rep["fps_check"], also in meta.json) below 0.9 x fps."""
    rep = dict(requested=dict(exposure=exposure, focus=focus, white_balance=wb), warmup_s=warmup_s)
    if not str(device).isdigit():
        rep["note"] = "replaying a file: no camera controls"
        return rep
    dev = f"/dev/video{device}"
    c = Controls(cap, dev, log, run)
    rep["v4l2_ctl"] = "found" if c.v4l2 is not None else "not installed (optional: sudo apt install v4l-utils)"
    rep["before"] = read_controls(cap)
    if "lock" in (exposure, focus, wb):
        t0 = now()
        while now() - t0 < warmup_s:          # let auto exposure / focus / white balance settle
            cap.read()
    auto_stats = frame_stats(cap)
    rep["stats_auto"] = auto_stats
    snap = read_controls(cap)
    rep["auto_values"] = snap

    # exposure
    cap_exp = max_exposure(fps)
    rep["exposure_cap"] = None if cap_exp is None else round(cap_exp, 1)
    manual_exposure = False
    if exposure is not None and c.v4l2 is not None:
        # auto exposure may otherwise lower the frame rate in dim light (UVC "dynamic framerate")
        if "exposure_dynamic_framerate" in c.v4l2:
            ok, err = v4l2_set(dev, "exposure_dynamic_framerate", 0, run)
            c.steps.append(dict(control="exposure_dynamic_framerate", ok=ok,
                                tried=[dict(via="v4l2-ctl exposure_dynamic_framerate", value=0, accepted=ok,
                                            error=err or None)]))
            rep["dynamic_framerate"] = "set to 0" if ok else f"could not set to 0 ({err})"
        else:
            rep["dynamic_framerate"] = "not offered by this camera"
    if exposure == "auto":
        c.set("auto_exposure", AE_AUTO)
    elif exposure is not None:
        val = snap["exposure"] if exposure == "lock" else float(exposure)
        if val is None:
            c.warnings.append("exposure: the camera doesn't report its exposure through OpenCV; not locked")
            log("  ! " + c.warnings[-1])
        else:
            if exposure == "lock" and cap_exp and val > cap_exp:
                c.warnings.append(f"exposure: the automatic value {val:g} is longer than one frame at {fps} fps "
                                  f"(max {cap_exp:.0f} x 100 us); locked at {cap_exp:.0f}. The picture may be darker: "
                                  "add light, or raise the gain")
                log("  ! " + c.warnings[-1])
                val = float(int(cap_exp))
            elif exposure != "lock" and cap_exp and val > cap_exp:
                c.warnings.append(f"exposure: --exposure {val:g} is longer than one frame at {fps} fps (max "
                                  f"{cap_exp:.0f}): the camera will drop its frame rate")
                log("  ! " + c.warnings[-1])
            manual_exposure = True
            if c.set("auto_exposure", AE_MANUAL, alt=AE_MANUAL_OLD) and c.set("exposure", val) and exposure == "lock" \
                    and auto_stats and auto_stats["brightness"] > 5:
                # the reported value can be stale while auto exposure runs: match the auto brightness
                target = auto_stats["brightness"]
                rng = (c.v4l2 or {}).get(c._v4l2_name("exposure") or "", {})
                lo, hi = rng.get("min", 1), rng.get("max", 10000)
                if cap_exp:
                    hi = min(hi, int(cap_exp))        # never longer than a frame: that would drop the fps
                for _ in range(4):
                    st = frame_stats(cap, 5)
                    if not st or abs(st["brightness"] / target - 1) <= 0.12 or st["brightness"] >= 250:
                        break
                    new = float(min(hi, max(lo, round(val * target / max(st["brightness"], 1.0)))))
                    if new == val:
                        break                         # at the cap (or the camera's limit): can't get brighter
                    val = new
                    c.set("exposure", val)
                rep["exposure_matched_to_auto_brightness"] = target
    # focus
    if focus == "auto":
        c.set("autofocus", 1)
    elif focus is not None:
        val = snap["focus"] if focus == "lock" else float(focus)
        if val is None:
            c.warnings.append("focus: the camera doesn't report its focus through OpenCV; not locked")
            log("  ! " + c.warnings[-1])
        else:
            c.set("autofocus", 0)
            c.set("focus", val)
    # white balance
    if wb == "auto":
        c.set("auto_wb", 1)
    elif wb is not None:
        val = snap["wb_temperature"] if wb == "lock" else float(wb)
        if val is None:
            c.warnings.append("white balance: the camera doesn't report its colour temperature; not locked")
            log("  ! " + c.warnings[-1])
        else:
            c.set("auto_wb", 0)
            c.set("wb_temperature", val)
    n_logged = len(c.warnings)
    rep["after"] = read_controls(cap)
    rep["stats_after"] = frame_stats(cap)
    a, b = rep["stats_auto"], rep["stats_after"]
    if a and b:
        if a["brightness"] > 5 and abs(b["brightness"] / a["brightness"] - 1) > 0.25:
            c.warnings.append(f"brightness changed {a['brightness']} -> {b['brightness']} when locking; "
                              "set --exposure by hand (see README)")
        if a["sharpness"] > 1 and b["sharpness"] < 0.7 * a["sharpness"]:
            c.warnings.append(f"sharpness dropped {a['sharpness']} -> {b['sharpness']} when locking focus; "
                              "set --focus by hand (see README)")
    if manual_exposure and fps and fps_check_s:
        try:
            exp_now = float(rep["after"].get("exposure"))
        except (TypeError, ValueError):
            exp_now = None
        n, secs, got = measure_fps(cap, fps_check_s, now)
        ok = got >= 0.9 * fps
        rep["fps_check"] = dict(requested=fps, measured=round(got, 2), frames=n, seconds=round(secs, 2), ok=ok)
        if not ok:
            c.warnings.append(f"FRAME RATE TOO LOW: {got:.1f} fps measured over {secs:.1f} s after setting the "
                              f"exposure, {fps} fps requested. Timing and blink codes need the full rate. "
                              + (f"Lower --exposure (at most {cap_exp:.0f}) or add light."
                                 if exp_now is not None and cap_exp and exp_now > 0.9 * cap_exp else
                                 "The exposure isn't the cause (it fits in a frame): try --buffers 8, another "
                                 "USB port, or check the Pi isn't overloaded (top)."))
            log("\n" + "!" * 78 + "\n  ! " + c.warnings[-1] + "\n" + "!" * 78)
    rep["steps"], rep["warnings"] = c.steps, c.warnings
    if c.v4l2 is not None:
        rep["v4l2_after"] = v4l2_list(dev, run)
    af = rep["after"]
    log(f"Camera: exposure {af['exposure']} (auto mode {af['auto_exposure']}), focus {af['focus']} "
        f"(autofocus {af['autofocus']}), white balance {af['wb_temperature']} K (auto {af['auto_wb']}), "
        f"gain {af['gain']}; brightness {a and a['brightness']} -> {b and b['brightness']}, "
        f"sharpness {a and a['sharpness']} -> {b and b['sharpness']}")
    for w in c.warnings[n_logged:]:
        log("  ! " + w)
    return rep


def control_arg(s):
    s = str(s).strip().lower()
    if s in ("auto", "lock"):
        return s
    try:
        return float(s)
    except ValueError:
        raise argparse.ArgumentTypeError("a number, 'auto' or 'lock'")


# ------------------------------------------------------------------ finding cameras
def list_cameras(run=subprocess.run, devices=None, opener=None, log=print, seconds=2.0):
    """Print the cameras the Pi sees and which /dev/videoN numbers deliver pictures.
    Each USB webcam usually makes two numbers (e.g. 0 and 1): one sends the
    picture, the other only information. Returns [(number, name, delivers, detail)]."""
    if shutil.which("v4l2-ctl") or run is not subprocess.run:
        try:
            out = run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True, timeout=5).stdout
            if out.strip():
                log("What v4l2-ctl --list-devices says (camera name, then its /dev/video numbers):")
                log("  " + out.strip().replace("\n", "\n  "))
                log("")
        except (OSError, subprocess.SubprocessError):
            pass
    if devices is None:
        devices = sorted(int(m.group(1)) for m in (re.fullmatch(r"video(\d+)", n) for n in os.listdir("/dev"))
                         if m) if os.path.isdir("/dev") else []
    if not devices:
        log("No cameras found (nothing called /dev/video*). Is the webcam plugged in? Try another USB port.")
        return []
    if opener is None:
        def opener(n):
            return open_camera(str(n), 640, 480, 30, 4, fail=lambda msg: None)
    log("Trying each number (a few seconds each)...")
    found = []
    for n in devices:
        name = ""
        try:
            with open(f"/sys/class/video4linux/video{n}/name") as f:
                name = f.read().strip()
        except OSError:
            pass
        cap = opener(n)
        delivers, detail = False, "could not open (in use by camrec.py already, or not a camera)"
        if cap is not None:
            t0, frame = time.monotonic(), None
            while time.monotonic() - t0 < seconds:
                ok, f = cap.read()
                if ok and f is not None:
                    frame = f
                    break
            if frame is not None:
                delivers = True
                fmt = fourcc_str(cap.get(cv2.CAP_PROP_FOURCC)) or "?"
                detail = f"sends pictures, {frame.shape[1]}x{frame.shape[0]}, {fmt}"
            else:
                detail = "opens but sends no picture (the information-only half of a webcam: normal)"
            try:
                cap.release()
            except Exception:
                pass
        found.append((n, name, delivers, detail))
        log(f"  {'USE ->' if delivers else '      '} {n:>2}  {name or '(no name)':<28} {detail}")
    good = [str(n) for n, _, d, _ in found if d]
    log("")
    if len(good) >= 2:
        log(f"Cameras that send pictures: {', '.join(good)}. Two cameras: "
            f"python3 tools/pi/camrec.py --device {good[0]},{good[1]} --names overhead,side --label E1")
    elif good:
        log(f"Only one camera sends pictures: {good[0]}. If two are plugged in, try another USB port and run --list again.")
    else:
        log("No camera sent a picture. If camrec.py is running, stop it first (Ctrl+C), then run --list again.")
    return found


# ------------------------------------------------------------------ disk space
DISK_GB_PER_CAM_10MIN = 0.5   # measured 2026-10-21/22: 2.9 GB for 63 min, 69 MB for 104 s at 25 fps (~0.45 GB per 10 min); was 2.0
DISK_WARN_GB = 1.0            # during recording: one loud warning below this
DISK_STOP_GB = 0.3            # during recording: stop and save cleanly below this (a full card corrupts video.avi)


def free_gb(path):
    p = os.path.abspath(path)
    while not os.path.exists(p):
        p = os.path.dirname(p)
    return shutil.disk_usage(p).free / 1e9


def disk_precheck(out, n_cams, seconds=0, log=print, free=None):
    """Before starting: warn (lines starting with '!') if the card has less than
    DISK_GB_PER_CAM_10MIN per camera per 10 minutes planned (10 minutes when no
    --seconds is given). Returns (free GB, GB wanted, minutes that fit)."""
    free = free_gb(out) if free is None else free
    minutes = seconds / 60.0 if seconds else 10.0
    want = DISK_GB_PER_CAM_10MIN * n_cams * minutes / 10.0
    fit = free / (DISK_GB_PER_CAM_10MIN * n_cams / 10.0)
    if free < want:
        log(f"! LOW DISK SPACE: {free:.1f} GB free in {out}; {n_cams} camera(s) for {minutes:.0f} min "
            f"need about {want:.1f} GB. Roughly {fit:.0f} min fit.")
        log("! Copy old recordings to the laptop and delete them from the Pi (README: Copy recordings).")
    else:
        log(f"Disk: {free:.1f} GB free, enough for about {fit:.0f} min with {n_cams} camera(s).")
    return free, want, fit


# ------------------------------------------------------------------ one camera's recording
class CamRec:
    """One camera: its video.avi, frames.csv, events.csv and meta.json, its own
    preview / live web server, and frame-rate and lost-frame checks. `cap` is an
    OpenCV capture, or anything with read(), get(prop) and release() (tests use a
    fake one)."""

    WINDOW_S = 5.0          # frame rate checked over this long
    WARN_EVERY_S = 20.0     # at most one frame-rate warning this often

    def __init__(self, cap, device, index=0, name="", port=0, fps_requested=30, replay=False, log=print, tag=""):
        self.cap, self.device, self.index, self.name = cap, str(device), index, name
        self.port, self.fps_requested, self.replay, self.log, self.tag = port, fps_requested, replay, log, tag
        self.latest, self.live = Latest(), LiveFrames()
        self.lock = threading.Lock()
        self.n = 0
        self.folder = self.writer = self.ff = self.fcsv = self.ef = self.ecsv = None
        self.w = self.h = None
        self.fps_real = fps_requested
        self.pixfmt = None
        self.gaps = self.missed = 0
        self.low_windows = []
        self.steady_fps = None       # what the camera really delivers, learnt from its first seconds
        self._intervals = []
        self.stopped = None
        self.t_first = self.t_last = None

    def first_frame(self):
        """Read one frame (not recorded: it was stamped late) for the picture size."""
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return False
        self.h, self.w = frame.shape[:2]
        self.fps_real = self.cap.get(cv2.CAP_PROP_FPS) or self.fps_requested
        self.pixfmt = fourcc_str(self.cap.get(cv2.CAP_PROP_FOURCC))
        return True

    def expected_fps(self):
        """The rate to hold the camera to: what it says it does, or, once learnt, what it
        steadily delivers if that's lower (two cameras on one Pi: 30 asked, 25 delivered,
        with no frame lost). Lost frames are counted from gaps, so a slow camera that
        never skips is not a false alarm."""
        f = self.fps_real
        f = f if f and 0 < f < 1000 else self.fps_requested
        return min(f, self.steady_fps) if self.steady_fps else f

    def serve(self):
        if not self.port:
            return None
        server = ThreadingHTTPServer(("", self.port), make_handler(self.latest, self.live))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.server = server
        return server

    def open_outputs(self, folder, meta):
        self.folder = folder
        os.makedirs(folder, exist_ok=True)
        self.writer = cv2.VideoWriter(os.path.join(folder, "video.avi"), cv2.VideoWriter_fourcc(*"MJPG"),
                                      self.fps_real, (self.w, self.h))
        self.ff = open(os.path.join(folder, "frames.csv"), "w", newline="")
        self.fcsv = csv.writer(self.ff)
        self.fcsv.writerow(["frame", "t_wall", "t_mono", "cam_ms"])
        self.ef = open(os.path.join(folder, "events.csv"), "w", newline="")
        self.ecsv = csv.writer(self.ef)
        self.ecsv.writerow(["t_wall", "frame", "text"])
        with open(os.path.join(folder, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)

    def mark(self, text, t_wall):
        """A note into events.csv, against this camera's current frame number."""
        if self.ecsv is None:
            return self.n
        with self.lock:
            n = self.n
            self.ecsv.writerow([f"{t_wall:.3f}", n, text])
            self.ef.flush()
        return n

    def flush_buffers(self, seconds):
        """Read and drop frames for `seconds`: frames that waited in the driver
        while the other camera was being set up would be stamped late."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            ok, _ = self.cap.read()
            if not ok:
                break

    def loop(self, t0, stop, seconds=0):
        """Record until stop is set, the camera stops, or t_mono reaches `seconds`.
        t_mono = time.monotonic() - t0: with the same t0 every camera is on one clock."""
        shown_fps, n_fps, t_fps = 0.0, 0, time.monotonic()
        win_n, win_t, last_warn = 0, time.monotonic(), -1e9
        last_ts = last_cam = None
        dt_nom = 1.0 / self.expected_fps()
        recording = self.writer is not None
        while not stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                self.stopped = "camera stopped sending pictures"
                self.log(f"{self.tag}Camera stopped sending pictures." if not self.tag else
                         f"! {self.tag}camera stopped sending pictures (the other camera keeps recording)")
                break
            t_wall, t_mono = time.time(), time.monotonic() - t0
            cam_ms = self.cap.get(cv2.CAP_PROP_POS_MSEC)   # driver's own capture time, if it gives one
            try:
                cam_ms = float(cam_ms)
            except (TypeError, ValueError):
                cam_ms = -1.0
            if recording:
                self.writer.write(frame)
                with self.lock:
                    self.fcsv.writerow([self.n, f"{t_wall:.4f}", f"{t_mono:.4f}", f"{cam_ms:.1f}"])
            if self.live.clients:
                self.live.put(frame, self.n, t_wall)   # same number and time as frames.csv
            # lost frames: a gap of more than 1.5 frame times (camera's own timestamps when it gives them)
            if not self.replay:
                if cam_ms > 0 and last_cam is not None and cam_ms > last_cam:
                    gap = (cam_ms - last_cam) / 1000.0
                elif last_ts is not None:
                    gap = t_mono - last_ts
                else:
                    gap = 0.0
                if gap > 1.5 * dt_nom:
                    self.gaps += 1
                    self.missed += max(1, int(round(gap / dt_nom)) - 1)
                # learn the steady rate from the typical frame interval of the first 3 s
                if self.steady_fps is None and gap > 0:
                    self._intervals.append(gap)
                    if t_mono - (self.t_first or t_mono) >= 3.0 and len(self._intervals) >= 20:
                        med = sorted(self._intervals)[len(self._intervals) // 2]
                        self.steady_fps = round(1.0 / med, 1)
                        if self.steady_fps < 0.9 * (self.fps_real or self.fps_requested):
                            self.log(f"({self.tag}the camera delivers a steady {self.steady_fps:g} fps "
                                     f"({self.fps_real:g} asked); that is its real rate, not lost frames)")
                        dt_nom = 1.0 / self.expected_fps()
            last_ts, last_cam = t_mono, (cam_ms if cam_ms > 0 else None)
            if self.t_first is None:
                self.t_first = t_mono
            self.t_last = t_mono
            with self.lock:
                self.n += 1
            n_fps += 1
            win_n += 1
            now = time.monotonic()
            if now - t_fps >= 1.0:
                shown_fps = n_fps / (now - t_fps)
                n_fps, t_fps = 0, now
                if recording:
                    with self.lock:
                        self.ff.flush()
            if now - win_t >= self.WINDOW_S:
                rate = win_n / (now - win_t)
                if not self.replay and rate < 0.9 * self.expected_fps():
                    self.low_windows.append([round(t_mono, 1), round(rate, 1)])
                    if now - last_warn >= self.WARN_EVERY_S:
                        last_warn = now
                        self.log(f"! {self.tag}only {rate:.1f} fps over the last {now - win_t:.0f} s "
                                 f"({self.expected_fps():g} expected): frames are being lost. Try: the cameras on "
                                 f"different USB ports, --pixfmt MJPG, a lower --fps, or check `top`.")
                win_n, win_t = 0, now
            if self.port and self.n % 2 == 0:   # preview at half rate to save CPU
                view = frame.copy()
                tag = f"{shown_fps:4.1f} fps  {'REC ' + time.strftime('%H:%M:%S' if t_mono >= 3600 else '%M:%S', time.gmtime(t_mono)) if recording else 'preview'}"
                if self.name:
                    tag = f"{self.name}  " + tag
                cv2.putText(view, tag, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                ok2, jpg = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok2:
                    self.latest.put(jpg.tobytes())
            if seconds and t_mono >= seconds:
                break
            if self.replay:
                time.sleep(1.0 / self.fps_real)   # replaying a file: keep real time

    def frame_check(self):
        """Frame rate and lost frames over the whole recording (meta.json frame_check)."""
        span = (self.t_last - self.t_first) if self.t_first is not None and self.t_last is not None else 0.0
        measured = (self.n - 1) / span if self.n > 1 and span > 0 else None
        exp = self.expected_fps()
        return dict(expected_fps=exp, steady_fps=self.steady_fps, measured_fps=None if measured is None else round(measured, 2),
                    frames=self.n, gaps=self.gaps, missed_estimate=self.missed,
                    missed_pct=round(100.0 * self.missed / max(1, self.n + self.missed), 2),
                    low_rate_windows=self.low_windows,
                    ok=(measured is None or measured >= 0.9 * exp) and self.missed <= 0.01 * max(1, self.n))

    def close(self, t0):
        """Save: the end camera controls and the frame check into meta.json, release."""
        fc = self.frame_check()
        if self.folder:
            try:
                mp = os.path.join(self.folder, "meta.json")
                with open(mp) as f:
                    meta = json.load(f)
                if self.device.isdigit():   # what the camera reports at the end: did anything drift?
                    meta["camera_controls_end"] = read_controls(self.cap)
                meta["frame_check"] = fc
                with open(mp, "w") as f:
                    json.dump(meta, f, indent=2)
            except Exception as e:
                self.log(f"({self.tag}couldn't add the end values to meta.json: {e})")
        try:
            self.cap.release()
        except Exception:
            pass
        if self.writer is not None:
            self.writer.release()
            with self.lock:
                self.ff.close()
                self.ef.close()
            secs = time.monotonic() - t0
            self.log(f"\n{self.tag}Saved {self.n} frames ({secs:.0f} s, {self.n / max(secs, 1e-9):.1f} fps) to {self.folder}")
            if not self.replay and not fc["ok"]:
                self.log(f"! {self.tag}{fc['measured_fps']} fps recorded ({fc['expected_fps']:g} expected), "
                         f"about {fc['missed_estimate']} frames lost ({fc['missed_pct']}%)")
        return fc


# ------------------------------------------------------------------ a session: one or more cameras
def run_session(caps, devices, names=None, out="~/recordings", label="", seconds=0, fps=30, port=8000,
                notes_port=5007, record=True, replays=None, controls=None, stdin_notes=True, log=print,
                stop=None, flush_s=0.5, disk_check_s=10.0, extra_meta=None):
    """Record one or more open cameras. One camera: ~/recordings/<date-time>_<label>/
    as always. Several: that folder holds cam0/, cam1/, ... (each a normal recording
    folder) and session.json. Cameras run as threads of this one process, so t_mono
    (time.monotonic() - one shared t0) and t_wall (time.time()) are the same clocks
    for all of them. Camera i's preview / live stream is on port + i.
    Returns (folder or None, [CamRec])."""
    n = len(caps)
    multi = n > 1
    names = list(names or [])
    names += [""] * (n - len(names))
    replays = replays or [not str(d).isdigit() for d in devices]
    controls = controls or [{} for _ in caps]
    extra_meta = extra_meta or [{} for _ in caps]
    stop = stop or threading.Event()
    out = os.path.expanduser(out)
    host = socket.gethostname()
    cams = []
    for i, (cap, dev) in enumerate(zip(caps, devices)):
        tag = f"cam{i}{' (' + names[i] + ')' if names[i] else ''}: " if multi else ""
        c = CamRec(cap, dev, i, names[i], (port + i) if port else 0, fps, replays[i], log, tag)
        if not c.first_frame():
            for cc in cams:
                cc.cap.release()
            cap.release()
            msg = f"Camera {dev} opened but sent no picture."
            if multi:
                msg += (" With two cameras this is often the USB bus being full: put the cameras on different USB "
                        "ports (not through a hub; try a few combinations), or try a lower --fps.")
            sys.exit(msg)
        if multi and str(dev).isdigit() and c.pixfmt and c.pixfmt != "MJPG":
            log(f"! {tag}the camera sends {c.pixfmt}, not MJPG: uncompressed video from two cameras can overflow "
                f"the USB bus (lost frames). Use --pixfmt MJPG if this camera offers it (--list shows the format).")
        if str(dev).isdigit() and c.fps_real and c.fps_real < 0.9 * fps:
            log(f"! {tag}the camera says it runs at {c.fps_real:g} fps, {fps} asked")
        cams.append(c)

    for c in cams:
        if c.serve():
            log(f"{c.tag}Preview: http://{host}.local:{c.port}   "
                f"(live link for tools/live/track_live.py: --stream {host}.local:{c.port})")

    t0 = time.monotonic()
    t0_wall = time.time()
    folder = None
    if record:
        stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        folder = os.path.join(out, stamp + (f"_{label}" if label else ""))
        os.makedirs(folder, exist_ok=True)
        for i, c in enumerate(cams):
            meta = dict(device=c.device, width=c.w, height=c.h, fps_requested=fps, fps_reported=c.fps_real,
                        host=host, start=dt.datetime.now().isoformat(timespec="seconds"),
                        label=label, note="t_wall is when the Pi received the frame",
                        camera_controls=controls[i], exposure_ms=exposure_ms(controls[i]),
                        fps_check=controls[i].get("fps_check"))
            meta["pixel_format"] = c.pixfmt
            if multi:
                meta.update(session=os.path.basename(folder), camera_index=i, role=c.name or None,
                            t0_monotonic=round(t0, 6), t0_wall=round(t0_wall, 4),
                            clock="t_mono = seconds on the Pi's monotonic clock since t0_monotonic, the same t0 "
                                  "for every camera in this session (one process): t_mono can be compared "
                                  "across cameras directly. t_wall is the Pi's system clock, also shared.")
            meta.update(extra_meta[i])
            c.open_outputs(os.path.join(folder, f"cam{i}") if multi else folder, meta)
            if multi:
                log(f"{c.tag}recording {c.w}x{c.h} to {c.folder}")
        if multi:
            write_session(folder, cams, label, host, t0, t0_wall, port)
        else:
            log(f"Recording {cams[0].w}x{cams[0].h} to {folder}")
        log("Type a note and press Enter to mark a moment. Ctrl+C stops.")
        start_notes(cams, stdin_notes, notes_port, log)
        if disk_check_s:
            threading.Thread(target=watch_disk, args=(out, stop, disk_check_s, log, cams), daemon=True).start()

    threads = []
    try:
        if not multi:
            cams[0].loop(t0, stop, seconds)
        else:
            def run(c):
                try:
                    if flush_s and not c.replay:     # a file would just be read ahead
                        c.flush_buffers(flush_s)
                    c.loop(t0, stop, seconds)
                except Exception as e:      # one camera failing must not lose the other's recording
                    c.stopped = f"error: {e}"
                    log(f"! {c.tag}stopped with an error: {e}")
            threads = [threading.Thread(target=run, args=(c,), daemon=True) for c in cams]
            for t in threads:
                t.start()
            while any(t.is_alive() for t in threads):
                for t in threads:
                    t.join(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if multi:
            for t in threads:
                t.join(5.0)
        for c in cams:
            c.close(t0)
        if record and multi:
            write_session(folder, cams, label, host, t0, t0_wall, port, end=True)
            log(f"\nSession saved: {folder}")
    return folder, cams


def write_session(folder, cams, label, host, t0, t0_wall, port, end=False):
    """session.json: which camera is in which subfolder, and the shared clock."""
    s = dict(
        session=os.path.basename(folder), label=label, host=host,
        start=dt.datetime.fromtimestamp(t0_wall).isoformat(timespec="seconds"),
        clock=dict(t_mono_shared=True, t0_monotonic=round(t0, 6), t0_wall=round(t0_wall, 4),
                   note="Every camera's frames.csv t_mono is time.monotonic() - t0_monotonic on this Pi (the "
                        "cameras run as threads of one camrec.py process), so t_mono values from different "
                        "cameras are on one clock and can be compared directly. t_wall is the Pi system clock "
                        "(also shared). Both are when the Pi received the frame: each camera's own delay "
                        "before that is not included and may differ between cameras."),
        capture="threads in one process",
        cameras=[])
    for c in cams:
        e = dict(index=c.index, folder=f"cam{c.index}", device=c.device, role=c.name or None,
                 port=c.port or None, width=c.w, height=c.h, fps_reported=c.fps_real, pixel_format=c.pixfmt)
        if end:
            e.update(frames=c.n, frame_check=c.frame_check(), stopped=c.stopped)
        s["cameras"].append(e)
    if end:
        s["end"] = dt.datetime.now().isoformat(timespec="seconds")
    tmp = os.path.join(folder, "session.json.tmp")
    with open(tmp, "w") as f:
        json.dump(s, f, indent=2)
    os.replace(tmp, os.path.join(folder, "session.json"))


def start_notes(cams, stdin_notes, notes_port, log=print):
    """Notes typed here, or sent by tools/run_sheet.py over UDP, go into every
    camera's events.csv against that camera's own frame number."""
    def mark_all(text, src=""):
        t = time.time()
        frames = [c.mark(text, t) for c in cams]
        where = (f"frame {frames[0]}" if len(cams) == 1 else
                 "frames " + ", ".join(f"cam{c.index} {f}" for c, f in zip(cams, frames)))
        log(f"  marked at {where}{src}: {text}")

    def notes():
        for line in sys.stdin:
            text = line.strip()
            if text:
                mark_all(text)

    def net_notes():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("", notes_port))
        except OSError as e:
            log(f"(network notes off: {e})")
            return
        while True:
            data, addr = sock.recvfrom(512)
            text = data.decode("utf-8", "replace").strip()
            if not text:
                continue
            mark_all(text, f" (from {addr[0]})")
            try:
                sock.sendto(b"ok " + data[:200], addr)   # receipt, so the sender knows the Pi heard it
            except OSError:
                pass
    if stdin_notes:
        threading.Thread(target=notes, daemon=True).start()
    if notes_port:
        threading.Thread(target=net_notes, daemon=True).start()


def watch_disk(out, stop, every_s, log=print, cams=(), free=None):
    """While recording: one loud warning below DISK_WARN_GB free; below
    DISK_STOP_GB stop and save cleanly (a full card leaves a broken video.avi)."""
    free = free or free_gb
    warned = False
    while not stop.wait(every_s):
        try:
            gb = free(out)
        except OSError:
            continue
        if gb < DISK_STOP_GB:
            log(f"\n! DISK ALMOST FULL ({gb:.2f} GB free): stopping and saving now.")
            for c in cams:
                c.stopped = c.stopped or "disk almost full"
            stop.set()
            return
        if gb < DISK_WARN_GB and not warned:
            warned = True
            log(f"\n! LOW DISK SPACE: {gb:.2f} GB free. Recording stops by itself at {DISK_STOP_GB} GB.")


def tag_line(ln, tag):
    """Put 'cam0 (overhead): ' into a log line, keeping a warning's '!' first."""
    s = ln.strip()
    if not s or set(s) == {"!"}:
        return ln
    if s.startswith("!"):
        return "  ! " + tag + s[1:].lstrip()
    return tag + ln


def list_arg(s):
    return [x.strip() for x in str(s).split(",") if x.strip()]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="0", help="camera number (0 = /dev/video0) or a video file to replay; "
                                                  "two cameras: 0,2 (see --list)")
    ap.add_argument("--names", type=list_arg, default=[], help="roles of the cameras, same order as --device, "
                                                               "e.g. overhead,side (saved in session.json)")
    ap.add_argument("--list", action="store_true", help="show the cameras this Pi sees and which numbers send pictures")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--pixfmt", default="MJPG", type=str.upper,
                    help="pixel format asked of the camera: MJPG (default, compressed: two cameras fit on USB) or YUYV")
    ap.add_argument("--buffers", type=int, default=4, help="driver frame buffers (1 can halve the frame rate)")
    ap.add_argument("--label", default="", help="added to the folder name, e.g. E1")
    ap.add_argument("--out", default=os.path.expanduser("~/recordings"))
    ap.add_argument("--port", type=int, default=8000, help="preview web page port (0 = no preview); "
                                                           "the second camera uses the next port (8001)")
    ap.add_argument("--no-record", action="store_true", help="preview only")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl+C)")
    ap.add_argument("--notes-port", type=int, default=5007, help="UDP port for notes sent by run_sheet.py (0 = off)")
    ap.add_argument("--exposure", type=control_arg, help="manual exposure (V4L2 units, usually 100 us), or auto")
    ap.add_argument("--focus", type=control_arg, help="manual focus (focus_absolute units), or auto")
    ap.add_argument("--wb", type=control_arg, help="manual white balance (K), or auto")
    ap.add_argument("--lock", dest="lock", action="store_true", default=True,
                    help="(default) after a 2 s warm-up, freeze the automatic exposure, focus and white balance")
    ap.add_argument("--no-lock", dest="lock", action="store_false", help="leave the camera's automatics alone")
    args = ap.parse_args(argv)

    if args.list:
        list_cameras()
        return
    devices = list_arg(args.device) or ["0"]
    if len(set(devices)) != len(devices):
        sys.exit(f"The same camera is listed twice in --device {args.device}.")
    if args.names and len(args.names) != len(devices):
        sys.exit(f"--names has {len(args.names)} name(s) but --device has {len(devices)} camera(s).")
    multi = len(devices) > 1
    if not args.no_record:
        disk_precheck(args.out, len(devices), args.seconds)

    caps = []
    for d in devices:
        def fail(msg, caps=caps):
            for c in caps:
                c.release()
            sys.exit(msg)
        caps.append(open_camera(d, args.width, args.height, args.fps, args.buffers, args.pixfmt, fail))
    dflt = "lock" if args.lock else None
    ctl = dict(exposure=args.exposure if args.exposure is not None else dflt,
               focus=args.focus if args.focus is not None else dflt,
               wb=args.wb if args.wb is not None else dflt)
    controls = [None] * len(caps)
    if not multi:
        controls[0] = apply_controls(caps[0], devices[0], ctl["exposure"], ctl["focus"], ctl["wb"], fps=args.fps)
    else:
        # both cameras' warm-up and frame-rate check at the same time (each ~4 s), each with its own values
        print_lock = threading.Lock()

        def setup(i):
            tag = f"cam{i}{' (' + args.names[i] + ')' if args.names else ''}: "

            def log(s):
                with print_lock:
                    print("\n".join(tag_line(ln, tag) for ln in str(s).split("\n")))
            try:
                controls[i] = apply_controls(caps[i], devices[i], ctl["exposure"], ctl["focus"], ctl["wb"],
                                             fps=args.fps, log=log)
            except Exception as e:
                controls[i] = dict(error=str(e))
                log(f"! could not set the camera controls: {e}")
        ts = [threading.Thread(target=setup, args=(i,)) for i in range(len(caps))]
        for t in ts:
            t.start()
        for t in ts:
            t.join()
    run_session(caps, devices, names=args.names, out=args.out, label=args.label, seconds=args.seconds,
                fps=args.fps, port=args.port, notes_port=args.notes_port, record=not args.no_record,
                controls=controls)


if __name__ == "__main__":
    main()
