#!/usr/bin/env python3
"""Record the overhead webcam on the Raspberry Pi, with a timestamp for every
frame and a live preview in your laptop's browser.

    python3 camrec.py                      # /dev/video0, 640x480, 30 fps
    python3 camrec.py --device 2 --label E1
    python3 camrec.py --no-record          # preview only, for aiming the camera
    python3 camrec.py --exposure 150 --focus 0     # fixed values instead of locking the auto ones
    python3 camrec.py --no-lock            # leave auto exposure / focus / white balance running

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


def make_handler(latest):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == "/stream":
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


def open_camera(device, width, height, fps, buffers=4):
    src = int(device) if str(device).isdigit() else device
    cap = cv2.VideoCapture(src, cv2.CAP_V4L2) if isinstance(src, int) else cv2.VideoCapture(src)
    if isinstance(src, int):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))   # the webcam's 30 fps mode
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
        sys.exit(f"Could not open camera {device}. Check it with: v4l2-ctl --list-devices")
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
                                 "The exposure isn't the cause (it fits in a frame): try --buffers 4, another "
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="0", help="camera number (0 = /dev/video0) or a video file to replay")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--buffers", type=int, default=4, help="driver frame buffers (1 can halve the frame rate)")
    ap.add_argument("--label", default="", help="added to the folder name, e.g. E1")
    ap.add_argument("--out", default=os.path.expanduser("~/recordings"))
    ap.add_argument("--port", type=int, default=8000, help="preview web page port (0 = no preview)")
    ap.add_argument("--no-record", action="store_true", help="preview only")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl+C)")
    ap.add_argument("--notes-port", type=int, default=5007, help="UDP port for notes sent by run_sheet.py (0 = off)")
    ap.add_argument("--exposure", type=control_arg, help="manual exposure (V4L2 units, usually 100 us), or auto")
    ap.add_argument("--focus", type=control_arg, help="manual focus (focus_absolute units), or auto")
    ap.add_argument("--wb", type=control_arg, help="manual white balance (K), or auto")
    ap.add_argument("--lock", dest="lock", action="store_true", default=True,
                    help="(default) after a 2 s warm-up, freeze the automatic exposure, focus and white balance")
    ap.add_argument("--no-lock", dest="lock", action="store_false", help="leave the camera's automatics alone")
    args = ap.parse_args()

    cap = open_camera(args.device, args.width, args.height, args.fps, args.buffers)
    dflt = "lock" if args.lock else None
    controls = apply_controls(cap, args.device, args.exposure if args.exposure is not None else dflt,
                              args.focus if args.focus is not None else dflt,
                              args.wb if args.wb is not None else dflt, fps=args.fps)
    ok, frame = cap.read()
    if not ok:
        sys.exit("The camera opened but sent no picture.")
    h, w = frame.shape[:2]
    real_fps = cap.get(cv2.CAP_PROP_FPS) or args.fps

    latest = Latest()
    if args.port:
        server = ThreadingHTTPServer(("", args.port), make_handler(latest))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print(f"Preview: http://{socket.gethostname()}.local:{args.port}")

    folder = writer = fcsv = ecsv = None
    lock = threading.Lock()
    state = {"frame": 0, "t0": time.monotonic()}
    if not args.no_record:
        stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        folder = os.path.join(args.out, stamp + (f"_{args.label}" if args.label else ""))
        os.makedirs(folder, exist_ok=True)
        writer = cv2.VideoWriter(os.path.join(folder, "video.avi"), cv2.VideoWriter_fourcc(*"MJPG"), real_fps, (w, h))
        ff = open(os.path.join(folder, "frames.csv"), "w", newline="")
        fcsv = csv.writer(ff)
        fcsv.writerow(["frame", "t_wall", "t_mono", "cam_ms"])
        ef = open(os.path.join(folder, "events.csv"), "w", newline="")
        ecsv = csv.writer(ef)
        ecsv.writerow(["t_wall", "frame", "text"])
        with open(os.path.join(folder, "meta.json"), "w") as f:
            json.dump(dict(device=args.device, width=w, height=h, fps_requested=args.fps, fps_reported=real_fps,
                           host=socket.gethostname(), start=dt.datetime.now().isoformat(timespec="seconds"),
                           label=args.label, note="t_wall is when the Pi received the frame",
                           camera_controls=controls, exposure_ms=exposure_ms(controls),
                           fps_check=controls.get("fps_check")), f, indent=2)
        print(f"Recording {w}x{h} to {folder}")
        print("Type a note and press Enter to mark a moment. Ctrl+C stops.")

        def notes():
            for line in sys.stdin:
                text = line.strip()
                if not text:
                    continue
                with lock:
                    ecsv.writerow([f"{time.time():.3f}", state["frame"], text])
                    ef.flush()
                print(f"  marked at frame {state['frame']}: {text}")
        threading.Thread(target=notes, daemon=True).start()

        def net_notes():
            """Notes sent by tools/run_sheet.py on the laptop, one UDP packet each."""
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.bind(("", args.notes_port))
            except OSError as e:
                print(f"(network notes off: {e})")
                return
            while True:
                data, addr = sock.recvfrom(512)
                text = data.decode("utf-8", "replace").strip()
                if not text:
                    continue
                with lock:
                    ecsv.writerow([f"{time.time():.3f}", state["frame"], text])
                    ef.flush()
                print(f"  marked at frame {state['frame']} (from {addr[0]}): {text}")
                try:
                    sock.sendto(b"ok " + data[:200], addr)   # receipt, so the sender knows the Pi heard it
                except OSError:
                    pass
        if args.notes_port:
            threading.Thread(target=net_notes, daemon=True).start()

    shown_fps, n_fps, t_fps = 0.0, 0, time.monotonic()
    try:
        while True:
            # (the frame read during setup is not recorded: it was stamped late)
            ok, frame = cap.read()
            if not ok:
                print("Camera stopped sending pictures.")
                break
            t_wall, t_mono = time.time(), time.monotonic() - state["t0"]
            cam_ms = cap.get(cv2.CAP_PROP_POS_MSEC)   # driver's own capture time, if it gives one
            if writer is not None:
                writer.write(frame)
                with lock:
                    fcsv.writerow([state["frame"], f"{t_wall:.4f}", f"{t_mono:.4f}", f"{cam_ms:.1f}"])
            state["frame"] += 1
            n_fps += 1
            if time.monotonic() - t_fps >= 1.0:
                shown_fps = n_fps / (time.monotonic() - t_fps)
                n_fps, t_fps = 0, time.monotonic()
                if writer is not None:
                    ff.flush()
            if args.port and state["frame"] % 2 == 0:   # preview at half rate to save CPU
                view = frame.copy()
                tag = f"{shown_fps:4.1f} fps  {'REC ' + time.strftime('%M:%S', time.gmtime(t_mono)) if writer is not None else 'preview'}"
                cv2.putText(view, tag, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                ok2, jpg = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok2:
                    latest.put(jpg.tobytes())
            if args.seconds and t_mono >= args.seconds:
                break
            if not str(args.device).isdigit():
                time.sleep(1.0 / real_fps)   # replaying a file: keep real time
    except KeyboardInterrupt:
        pass
    finally:
        if folder and str(args.device).isdigit():
            try:   # what the camera reports at the end: did anything drift?
                mp = os.path.join(folder, "meta.json")
                with open(mp) as f:
                    meta = json.load(f)
                meta["camera_controls_end"] = read_controls(cap)
                with open(mp, "w") as f:
                    json.dump(meta, f, indent=2)
            except Exception as e:
                print(f"(couldn't add the end control values to meta.json: {e})")
        cap.release()
        if writer is not None:
            writer.release()
            with lock:
                ff.close()
                ef.close()
            secs = time.monotonic() - state["t0"]
            print(f"\nSaved {state['frame']} frames ({secs:.0f} s, {state['frame'] / max(secs, 1e-9):.1f} fps) to {folder}")


if __name__ == "__main__":
    main()
