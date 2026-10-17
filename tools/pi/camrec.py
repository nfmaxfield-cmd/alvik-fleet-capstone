#!/usr/bin/env python3
"""Record the overhead webcam on the Raspberry Pi, with a timestamp for every
frame and a live preview in your laptop's browser.

    python3 camrec.py                      # /dev/video0, 640x480, 30 fps
    python3 camrec.py --device 2 --label E1
    python3 camrec.py --no-record          # preview only, for aiming the camera

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
import socket
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


def open_camera(device, width, height, fps):
    src = int(device) if str(device).isdigit() else device
    cap = cv2.VideoCapture(src, cv2.CAP_V4L2) if isinstance(src, int) else cv2.VideoCapture(src)
    if isinstance(src, int):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))   # the webcam's 30 fps mode
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        # Keep at most one frame waiting in the driver, so t_wall (stamped when
        # we receive the frame) isn't late by a varying number of frames.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        sys.exit(f"Could not open camera {device}. Check it with: v4l2-ctl --list-devices")
    return cap


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="0", help="camera number (0 = /dev/video0) or a video file to replay")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--label", default="", help="added to the folder name, e.g. E1")
    ap.add_argument("--out", default=os.path.expanduser("~/recordings"))
    ap.add_argument("--port", type=int, default=8000, help="preview web page port (0 = no preview)")
    ap.add_argument("--no-record", action="store_true", help="preview only")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl+C)")
    ap.add_argument("--notes-port", type=int, default=5007, help="UDP port for notes sent by run_sheet.py (0 = off)")
    args = ap.parse_args()

    cap = open_camera(args.device, args.width, args.height, args.fps)
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
                           label=args.label, note="t_wall is when the Pi received the frame"), f, indent=2)
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
