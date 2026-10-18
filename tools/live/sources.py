"""Where the live tracker's pictures come from, and the Pi -> laptop clock.

StreamSource   camrec.py's /live stream (every frame, grayscale JPEG, with the
               Pi's frame number and t_wall in each part's headers)
ReplaySource   a recorded folder (video.avi + frames.csv), played at the speed
               it was recorded (or faster, for tests)

Both yield Frame(index, pi_frame, t_pi, img, t_arrived): t_pi is the Pi's
t_wall (when the Pi received the frame), t_arrived the laptop time the frame
was ready here.

ClockEnvelope turns t_pi into laptop time: laptop arrival - Pi t_wall is the
clock offset plus the trip over the network, which is never negative and
often near its minimum, so the smallest value over the last 30 s is the
offset (plus the shortest trip, a few ms). No NTP needed. The camera's own
delay (picture taken -> Pi receives it, about one frame) is subtracted
separately (--capture-delay-ms)."""
import csv
import os
import socket
import time
from collections import deque
from dataclasses import dataclass
from urllib.parse import urlparse

import cv2
import numpy as np


@dataclass
class Frame:
    index: int
    pi_frame: int
    t_pi: float
    img: np.ndarray
    t_arrived: float


class ClockEnvelope:
    def __init__(self, window_s=30.0):
        self.window = window_s
        self.q = deque()          # (t_arrived, t_arrived - t_pi), increasing minimum kept
        self.n = 0

    def add(self, t_arrived, t_pi):
        d = t_arrived - t_pi
        while self.q and self.q[-1][1] >= d:
            self.q.pop()
        self.q.append((t_arrived, d))
        while self.q and self.q[0][0] < t_arrived - self.window:
            self.q.popleft()
        self.n += 1

    @property
    def offset(self):
        """laptop - Pi clock (s), or None before the first frame."""
        return self.q[0][1] if self.q else None


def read_frames_csv(folder):
    rows = []
    with open(os.path.join(folder, "frames.csv"), newline="") as f:
        for r in csv.DictReader(f):
            try:
                rows.append((int(r["frame"]), float(r["t_wall"])))
            except (KeyError, TypeError, ValueError):
                rows.append((len(rows), float("nan")))
    return rows


class ReplaySource:
    """Plays a recording folder in real time (speed 1), faster, or as fast as
    the tracker can go (speed 0). Arrival time = start + (t_wall - first t_wall) / speed."""

    def __init__(self, folder, speed=1.0, start_s=0.0, max_s=0.0, clock=time.time, sleep=time.sleep):
        self.folder = folder
        self.speed = speed
        self.rows = read_frames_csv(folder)
        self.cap = cv2.VideoCapture(os.path.join(folder, "video.avi"))
        if not self.cap.isOpened():
            raise SystemExit("Cannot open %s" % os.path.join(folder, "video.avi"))
        self.start_s, self.max_s = start_s, max_s
        self.clock, self.sleep = clock, sleep
        self.t0 = None

    def __iter__(self):
        tw0 = None
        k = 0
        for i, (pf, tw) in enumerate(self.rows):
            ok, img = self.cap.read()
            if not ok:
                break
            if not np.isfinite(tw):
                continue
            if tw0 is None:
                tw0 = tw
            if tw - tw0 < self.start_s:
                continue
            if self.max_s and tw - tw0 > self.start_s + self.max_s:
                break
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
            if self.t0 is None:
                self.t0 = self.clock() - (tw - tw0) / (self.speed or 1.0)
            if self.speed > 0:
                due = self.t0 + (tw - tw0) / self.speed
                wait = due - self.clock()
                if wait > 0:
                    self.sleep(wait)
                arrived = max(due, self.clock())
            else:
                arrived = self.clock()
            yield Frame(k, pf, tw, gray, arrived)
            k += 1
        self.cap.release()

    def close(self):
        self.cap.release()


class StreamSource:
    """Reads http://<pi>:8000/live (camrec.py): multipart JPEG parts with
    X-Frame and X-Time headers. Reconnects if the stream drops."""

    def __init__(self, url, timeout=5.0, log=print):
        if "://" not in url:
            url = "http://" + url
        u = urlparse(url)
        self.host, self.port = u.hostname, u.port or 8000
        self.path = (u.path if u.path not in ("", "/") else "/live") + (("?" + u.query) if u.query else "")
        self.timeout = timeout
        self.log = log
        self.sock = None
        self.buf = b""
        self.closed = False
        self.gaps = 0          # Pi frames we never got (the Pi skipped them: we or the network were slow)

    def _connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.sendall(("GET %s HTTP/1.1\r\nHost: %s\r\nConnection: keep-alive\r\n\r\n"
                           % (self.path, self.host)).encode())
        self.buf = b""
        head = self._until(b"\r\n\r\n")
        status = head.split(b"\r\n", 1)[0]
        if b" 200" not in status:
            raise ConnectionError("camera answered %r (is camrec.py new enough to have /live?)" % status)

    def _fill(self):
        data = self.sock.recv(262144)
        if not data:
            raise ConnectionError("stream closed")
        self.buf += data

    def _until(self, mark):
        while True:
            i = self.buf.find(mark)
            if i >= 0:
                out, self.buf = self.buf[:i], self.buf[i + len(mark):]
                return out
            self._fill()

    def _exact(self, n):
        while len(self.buf) < n:
            self._fill()
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def parts(self):
        """(headers dict, jpeg bytes, arrival time) for each part."""
        while True:
            line = self._until(b"\r\n")
            if not line.startswith(b"--"):
                continue
            hdr = self._until(b"\r\n\r\n")
            h = {}
            for ln in hdr.split(b"\r\n"):
                k, _, v = ln.partition(b":")
                h[k.strip().lower().decode()] = v.strip().decode()
            n = int(h.get("content-length", "0"))
            body = self._exact(n)
            yield h, body, time.time()

    def __iter__(self):
        k = 0
        last_pf = None
        while not self.closed:
            try:
                self._connect()
                for h, body, arrived in self.parts():
                    img = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_GRAYSCALE)
                    if img is None:
                        continue
                    pf = int(h.get("x-frame", k))
                    if last_pf is not None and pf > last_pf + 1:
                        self.gaps += pf - last_pf - 1
                    last_pf = pf
                    yield Frame(k, pf, float(h["x-time"]), img, arrived)
                    k += 1
            except (OSError, ConnectionError, KeyError, ValueError) as e:
                if self.closed:
                    break
                self.log("camera stream: %s; trying again in 1 s" % e)
                time.sleep(1.0)
            finally:
                if self.sock is not None:
                    try:
                        self.sock.close()
                    except OSError:
                        pass
                    self.sock = None

    def close(self):
        self.closed = True
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
