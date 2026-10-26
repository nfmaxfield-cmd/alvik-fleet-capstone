"""Frame saving for production runs (2026-10-25 evening, MIRLT; Nick approved 19:03): instead of a whole video,
the pictures that matter. Runs inside the tracker process, on the JPEG pictures it already has (the /live stream's
own bytes, or the camera's own MJPG bytes): nothing is decoded or encoded again except the thumbnails.

  - a ring of the last BEFORE_S + AFTER_S seconds of pictures (JPEG bytes, about 35 KB each);
  - on every event (a hold, a border stop, an edge stop, a help call, a Note tap, a failed turn; the runner says
    which), once AFTER_S has passed: the picture at that moment (still) and the clip from BEFORE_S before it to AFTER_S
    after it, as a Motion-JPEG .avi any player opens, with a .csv of each picture's time and recorder frame number;
    events that fall inside a clip still being collected join that clip (one clip, up to MAX_CLIP_S long);
  - a thumbnail (half size) every THUMB_EVERY_S;
  - events.csv: one row per event, with the files it went into.
Everything goes into <session>/frames/. Disk use is capped (MAX_MB): over it, clips stop (stills and the event rows
go on) and it says so once.

    fs = FrameSaver(folder, log=print)
    fs.add(jpeg_bytes, t, pi_frame)          # every picture
    fs.event("hold", "e02 hold Right lost", t=time.time(), robot="Right")
    fs.close()                               # writes what is pending
"""
import csv
import os
import queue
import struct
import threading
import time
from collections import deque

# Chosen numbers. DOE notes: docs/doe-backlog.md rows "Frame saving".
BEFORE_S = 10.0          # Nick's figure ("the buffered ~10 s clip"). DOE: from the first production sessions, how far
                         # back the cause of each event is (the hold's own report says when it began); 10 s may be short
                         # for a slow drift into a border, long for a help call
AFTER_S = 3.0            # chosen: what happened just after (did it stop, did Nick's hand come in). No DOE: cost is small
THUMB_EVERY_S = 30.0     # Nick's figure. No DOE: a record, not a measurement
MAX_CLIP_S = 60.0        # chosen: a burst of events is one clip at most this long. No DOE: a size limit
MAX_MB = 4000.0          # chosen: clips stop past this much in frames/ (a 2 h session with an event every 2 minutes is
                         # ~60 clips x 13 s x 25 x 35 KB = 700 MB). No DOE: a guard
THUMB_SCALE = 0.5        # half size thumbnails. No DOE

KINDS = ("hold", "border stop", "edge stop", "help call", "note", "failed turn", "parked robot moved", "other")


def write_mjpeg_avi(path, jpegs, fps, width, height):
    """A Motion-JPEG AVI (RIFF, one video stream, idx1 index) from JPEG byte strings, without re-encoding. Plays in
    VLC and the Windows player; OpenCV reads it."""
    fps = max(1.0, float(fps))
    n = len(jpegs)
    us = int(round(1e6 / fps))

    def chunk(cid, data):
        pad = b"\0" if len(data) % 2 else b""
        return cid + struct.pack("<I", len(data)) + data + pad

    def lst(kind, data):
        return b"LIST" + struct.pack("<I", len(data) + 4) + kind + data

    maxb = max((len(j) for j in jpegs), default=0)
    avih = struct.pack("<IIIIIIIIIIIIII", us, int(maxb * fps), 0, 0x10, n, 0, 1, maxb + 8, width, height, 0, 0, 0, 0)
    rate, scale = int(round(fps * 1000)), 1000
    strh = struct.pack("<4s4sIHHIIIIIIIIhhhh", b"vids", b"MJPG", 0, 0, 0, 0, scale, rate, 0, n, maxb + 8, 0xFFFFFFFF, 0,
                       0, 0, width, height)
    strf = struct.pack("<IiiHH4sIiiII", 40, width, height, 1, 24, b"MJPG", width * height * 3, 0, 0, 0, 0)
    hdrl = lst(b"hdrl", chunk(b"avih", avih) + lst(b"strl", chunk(b"strh", strh) + chunk(b"strf", strf)))
    movi_parts, idx = [], []
    off = 4                                     # offsets in idx1 count from the 'movi' fourcc
    for j in jpegs:
        c = chunk(b"00dc", j)
        idx.append(struct.pack("<4sIII", b"00dc", 0x10, off, len(j)))
        movi_parts.append(c)
        off += len(c)
    movi = lst(b"movi", b"".join(movi_parts))
    body = b"AVI " + hdrl + movi + chunk(b"idx1", b"".join(idx))
    tmp = path + ".part"
    with open(tmp, "wb") as f:
        f.write(b"RIFF" + struct.pack("<I", len(body)) + body)
    os.replace(tmp, path)


def jpeg_size(jpeg):
    """(width, height) from a JPEG's SOF marker, or None."""
    i = 2
    n = len(jpeg)
    while i + 9 < n:
        if jpeg[i] != 0xFF:
            i += 1
            continue
        m = jpeg[i + 1]
        if m in (0xC0, 0xC1, 0xC2):
            h, w = struct.unpack(">HH", jpeg[i + 5:i + 9])
            return w, h
        if m in (0xD8, 0x01) or 0xD0 <= m <= 0xD7:
            i += 2
            continue
        seg = struct.unpack(">H", jpeg[i + 2:i + 4])[0]
        i += 2 + seg
    return None


class FrameSaver:
    def __init__(self, folder, before_s=BEFORE_S, after_s=AFTER_S, thumb_every_s=THUMB_EVERY_S, keep_s=0.0,
                 max_mb=MAX_MB, log=print, clock=time.time):
        self.dir = folder
        os.makedirs(os.path.join(folder, "clips"), exist_ok=True)
        os.makedirs(os.path.join(folder, "stills"), exist_ok=True)
        os.makedirs(os.path.join(folder, "thumbs"), exist_ok=True)
        self.before, self.after = float(before_s), float(after_s)
        self.keep = max(self.before + self.after + 1.0, float(keep_s or 0.0))
        self.thumb_every = float(thumb_every_s)
        self.max_bytes = float(max_mb) * 1e6
        self.log = log
        self.clock = clock
        self.ring = deque()                     # (t, jpeg bytes, pi_frame)
        self.lock = threading.Lock()
        self.pending = []                       # open clips: dict(t0, t1, events=[...], name)
        self.jobs = queue.Queue()
        self.last_thumb = None
        self.used = 0
        self.full_said = False
        self.n_events = self.n_clips = self.n_stills = self.n_thumbs = 0
        self.closed = False
        self.ev_path = os.path.join(folder, "events.csv")
        new = not os.path.exists(self.ev_path)
        self.ev_f = open(self.ev_path, "a", newline="")
        self.ev_w = csv.writer(self.ev_f)
        if new:
            self.ev_w.writerow(["t", "clock", "kind", "robot", "text", "still", "clip", "pi_frame"])
            self.ev_f.flush()
        self.th = threading.Thread(target=self._work, daemon=True, name="frame-saver")
        self.th.start()

    # --- every picture ------------------------------------------------------------------------------------------
    def add(self, jpeg, t, pi_frame=None):
        if jpeg is None:
            return
        with self.lock:
            self.ring.append((t, jpeg, pi_frame))
            while self.ring and t - self.ring[0][0] > self.keep:
                self.ring.popleft()
            due = [c for c in self.pending if t >= c["t1"]]
            self.pending = [c for c in self.pending if t < c["t1"]]
        for c in due:
            self.jobs.put(("clip", c))
        if self.thumb_every > 0 and (self.last_thumb is None or t - self.last_thumb >= self.thumb_every):
            self.last_thumb = t
            self.jobs.put(("thumb", (t, jpeg)))

    def kept(self, t_from, t_to=None):
        """[(t, jpeg bytes, pi_frame)] between t_from and t_to, oldest first."""
        with self.lock:
            return [q for q in self.ring if q[0] >= t_from and (t_to is None or q[0] <= t_to)]

    # --- events -------------------------------------------------------------------------------------------------
    def event(self, kind, text="", t=None, robot=""):
        """Something worth keeping happened at time t (default: now)."""
        t = self.clock() if t is None else float(t)
        kind = kind if kind in KINDS else "other"
        ev = dict(t=t, kind=kind, text=str(text)[:200], robot=robot or "")
        with self.lock:
            self.n_events += 1
            for c in self.pending:
                if c["t0"] <= t <= c["t1"] and t + self.after - c["t0"] <= MAX_CLIP_S:
                    c["t1"] = max(c["t1"], t + self.after)
                    c["events"].append(ev)
                    break
            else:
                stamp = time.strftime("%H%M%S", time.localtime(t))
                slug = "".join(ch if ch.isalnum() else "-" for ch in kind + "-" + (robot or ""))[:40].strip("-")
                c = dict(t0=t - self.before, t1=t + self.after, events=[ev], name="%s_%s" % (stamp, slug))
                self.pending.append(c)
        return ev

    # --- the writer thread --------------------------------------------------------------------------------------
    def _work(self):
        while True:
            job = self.jobs.get()
            if job is None:
                return
            try:
                if job[0] == "clip":
                    self._write_clip(job[1])
                elif job[0] == "thumb":
                    self._write_thumb(*job[1])
            except Exception as e:
                self.log("(frame saving: %s)" % e)

    def _write_thumb(self, t, jpeg):
        import cv2
        import numpy as np
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_UNCHANGED)
        if img is None:
            return
        small = cv2.resize(img, None, fx=THUMB_SCALE, fy=THUMB_SCALE, interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            p = os.path.join(self.dir, "thumbs", time.strftime("%H%M%S", time.localtime(t)) + ".jpg")
            with open(p, "wb") as f:
                f.write(buf.tobytes())
            self.used += len(buf)
            self.n_thumbs += 1

    def _write_clip(self, c):
        frames = self.kept(c["t0"], c["t1"])
        stills = []
        for ev in c["events"]:
            # the still: the picture nearest the event
            best = min(frames, key=lambda q: abs(q[0] - ev["t"])) if frames else None
            sp = ""
            if best is not None:
                sp = os.path.join("stills", "%s_%s.jpg" % (time.strftime("%H%M%S", time.localtime(ev["t"])),
                                                          "".join(ch if ch.isalnum() else "-" for ch in
                                                                  ev["kind"] + "-" + ev["robot"]).strip("-")))
                with open(os.path.join(self.dir, sp), "wb") as f:
                    f.write(best[1])
                self.used += len(best[1])
                self.n_stills += 1
            stills.append((ev, sp, best[2] if best is not None else ""))
        clip = ""
        if frames and self.used < self.max_bytes:
            size = jpeg_size(frames[0][1]) or (640, 480)
            span = frames[-1][0] - frames[0][0]
            fps = (len(frames) - 1) / span if span > 0 and len(frames) > 1 else 25.0
            clip = os.path.join("clips", c["name"] + ".avi")
            write_mjpeg_avi(os.path.join(self.dir, clip), [q[1] for q in frames], fps, size[0], size[1])
            with open(os.path.join(self.dir, "clips", c["name"] + ".csv"), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["i", "t", "pi_frame"])
                for i, q in enumerate(frames):
                    w.writerow([i, "%.3f" % q[0], "" if q[2] is None else q[2]])
            self.used += sum(len(q[1]) for q in frames)
            self.n_clips += 1
        elif frames and not self.full_said:
            self.full_said = True
            self.log("(frame saving: %.0f MB used, the cap; clips stop, stills go on)" % (self.used / 1e6))
        with self.lock:
            for ev, sp, pf in stills:
                self.ev_w.writerow(["%.3f" % ev["t"], time.strftime("%H:%M:%S", time.localtime(ev["t"])), ev["kind"],
                                    ev["robot"], ev["text"], sp, clip, "" if pf is None else pf])
            self.ev_f.flush()

    def summary(self):
        return dict(events=self.n_events, clips=self.n_clips, stills=self.n_stills, thumbs=self.n_thumbs,
                    mb=round(self.used / 1e6, 1))

    def close(self, flush=True):
        """Writes every clip still being collected (with what it has) and stops the writer."""
        if self.closed:
            return
        self.closed = True
        if flush:
            with self.lock:
                due, self.pending = self.pending, []
            for c in due:
                self.jobs.put(("clip", c))
        self.jobs.put(None)
        self.th.join(timeout=30)
        try:
            self.ev_f.close()
        except Exception:
            pass


# The runner's marks (e02.py `mark`) and what each is, for the frame saver. Nick's list (2026-10-25): hold, border
# stop, edge stop, help call, Note tap, failed turn. A mark that is none of these saves nothing.
MARK_KINDS = (
    ("e02 hold answer", None),             # Nick answered: not a new event
    ("e02 hold border", "border stop"),    # a crossing of the virtual border stops every robot and holds
    ("e02 hold", "hold"),
    ("e02 help", "help call"),
    ("e02 watchdog stop", "edge stop"),    # the camera watchdog stops a robot heading past an open edge
    ("e02 speed guard stop", "border stop"),
    ("e02 border", "border stop"),
    ("e02 edge risk", "edge stop"),
    ("e02 drop", "edge stop"),
    ("e02 open edge", "edge stop"),
    ("e02 turn fault", "failed turn"),
    ("e02 turn failed", "failed turn"),
    ("e02 cant turn", "failed turn"),
    ("e02 can't turn", "failed turn"),
)


def kind_of_mark(text):
    """The frame saver's kind for a runner mark's text, or None (not an event worth a clip)."""
    t = (text or "").strip().lower()
    for prefix, kind in MARK_KINDS:
        if t.startswith(prefix):
            return kind
    return None
