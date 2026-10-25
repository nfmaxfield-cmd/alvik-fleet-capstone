"""Footage for the mark-the-robot page (mark_robot.py): read pictures from a recording by frame number, and fetch
only the part of a recording that is needed from a GitHub release.

A recording (tools/pi/camrec.py) is a Motion-JPEG `video.avi` plus `frames.csv` (frame, t_wall, ...): row j of
frames.csv is the j-th picture in the video. Every store here gives:
    store.frames      sorted list of frame numbers it holds
    store.t(frame)    the frame's t_wall (Pi clock, Unix seconds)
    store.jpeg(frame) the picture as JPEG bytes (what the page shows)
    store.gray(frame) the picture as a grey numpy array (what the tracker reads)
    store.index(t)    the frame at (or just after) time t

Stores:
    JpegStore   a folder made by fetch(): stretch.jpgs (the JPEGs one after the other) + stretch_index.csv
                (frame, t_wall, offset, size) + stretch.json (what the stretch is). Small: only the stretch.
    AviStore    a local video.avi (or one piece of one) + its frames.csv. Uses the AVI's own index when there is
                one (exact), else finds the pictures by their '00dc' chunk headers (a piece cut by bytes).
    FolderStore a folder of pictures named <frame>.jpg (e.g. tools/label's frames/000123.jpg) + frames.csv,
                or --fps when there is no frames.csv.
    CvStore     any other video OpenCV can read (+ frames.csv or --fps).

fetch(): reads the AVI's index from the release with two small ranged downloads (the header, then the index
chunks), then downloads only the bytes of the stretch's pictures (about 0.1 GB for two minutes instead of the
whole 1.5 GB video). A video split into parts for upload (video.part-aa, -ab, ...) is read as one file.
"""
import bisect
import csv
import json
import mmap
import os
import struct
import sys
import urllib.request

import cv2
import numpy as np

RELEASE_URL = "https://github.com/nfmaxfield-cmd/alvik-fleet-capstone/releases/download/%s/%s"


# ============================================================================ frames.csv
def read_frame_times(path):
    """frames.csv -> {frame: t_wall}. Rows without a frame column are numbered in order."""
    out = {}
    with open(path, newline="") as f:
        for k, r in enumerate(csv.DictReader(f)):
            fr = int(r["frame"]) if r.get("frame") not in (None, "") else k
            out[fr] = float(r["t_wall"])
    return out


class _Base:
    """Shared lookups; subclasses fill self.frames (sorted) and self._t (frame -> t)."""

    def t(self, frame):
        return self._t[frame]

    def index(self, t):
        """The frame held at (or just after) time t; the last one if t is past the end."""
        k = bisect.bisect_left(self._ts, t)
        return self.frames[min(k, len(self.frames) - 1)]

    def nearest(self, t):
        ts = self._ts
        k = bisect.bisect_left(ts, t)
        cands = [j for j in (k - 1, k) if 0 <= j < len(ts)]
        j = min(cands, key=lambda j: abs(ts[j] - t))
        return self.frames[j]

    def _finish(self):
        self.frames = sorted(self.frames)
        self._ts = [self._t[f] for f in self.frames]

    def gray(self, frame):
        b = self.jpeg(frame)
        if b is None:
            return None
        return cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_GRAYSCALE)

    def has(self, frame):
        return frame in self._t

    def neighbours(self, frame, before=0, after=0):
        """Frames held just before / after a frame (in store order)."""
        k = bisect.bisect_left(self.frames, frame)
        return self.frames[max(0, k - before):k + after + 1]

    def close(self):
        pass


class JpegStore(_Base):
    def __init__(self, folder):
        self.folder = folder
        self.meta = {}
        mp = os.path.join(folder, "stretch.json")
        if os.path.exists(mp):
            with open(mp) as f:
                self.meta = json.load(f)
        self._t, self._off = {}, {}
        with open(os.path.join(folder, "stretch_index.csv"), newline="") as f:
            for r in csv.DictReader(f):
                fr = int(r["frame"])
                self._t[fr] = float(r["t_wall"])
                self._off[fr] = (int(r["offset"]), int(r["size"]))
        self.frames = list(self._t)
        self._finish()
        self.f = open(os.path.join(folder, "stretch.jpgs"), "rb")
        self.m = mmap.mmap(self.f.fileno(), 0, access=mmap.ACCESS_READ) if os.path.getsize(
            os.path.join(folder, "stretch.jpgs")) else b""

    def jpeg(self, frame):
        if frame not in self._off:
            return None
        o, n = self._off[frame]
        return bytes(self.m[o:o + n])

    def close(self):
        try:
            self.m.close()
            self.f.close()
        except Exception:
            pass


# ============================================================================ AVI index
def avi_index(read, size):
    """Every picture's (offset, size) in an MJPG AVI, in order: from the OpenDML index ('indx' -> 'ix00' chunks,
    what OpenCV writes) or the old 'idx1'. read(off, n) -> bytes; size: the file's length. Offsets point at the
    JPEG itself (after the 8-byte chunk header). Returns None when the file has no usable index."""
    head = read(0, min(size, 1 << 16))
    if head[:4] != b"RIFF" or head[8:12] != b"AVI ":
        return None
    i = head.find(b"indx")
    if i > 0:
        w_longs, sub, typ, n_ent = struct.unpack("<HBBI", head[i + 8:i + 16])
        if typ == 0 and w_longs == 4:                       # a super index: entries point at standard indexes
            out = []
            for k in range(n_ent):
                q_off, d_size, _ = struct.unpack("<QII", head[i + 32 + 16 * k:i + 48 + 16 * k])
                ch = read(q_off, d_size + 8 if d_size else 32)
                if ch[:2] != b"ix":
                    return None
                wl, _, _, n_in = struct.unpack("<HBBI", ch[8:16])
                base = struct.unpack("<Q", ch[20:28])[0]
                if len(ch) < 32 + 8 * n_in:
                    ch = read(q_off, 32 + 8 * n_in)
                for e in range(n_in):
                    off, sz = struct.unpack("<II", ch[32 + 8 * e:40 + 8 * e])
                    out.append((base + off, sz & 0x7FFFFFFF))
            return out
    # the old index: after the 'movi' list
    m = head.find(b"movi")
    if m < 8:
        return None
    movi_size = struct.unpack("<I", head[m - 4:m])[0]
    p = m + movi_size                                        # end of the movi list = where idx1 starts
    p += p & 1
    hdr = read(p, 8)
    if len(hdr) < 8 or hdr[:4] != b"idx1":
        return None
    n = struct.unpack("<I", hdr[4:8])[0]
    body = read(p + 8, n)
    out = []
    first = None
    for e in range(n // 16):
        ck, _flags, off, sz = struct.unpack("<4sIII", body[16 * e:16 * e + 16])
        if ck[2:4] != b"dc":
            continue
        if first is None:                                    # offsets are from 'movi' or from the file's start
            first = off
            rel = off < m
        out.append(((m + off if rel else off) + 8, sz))
    return out


def scan_chunks(m):
    """(offset, size) of every '00dc' chunk holding a JPEG in a buffer (a piece of an AVI cut by bytes)."""
    idx, i = [], 0
    while True:
        j = m.find(b"00dc", i)
        if j < 0:
            break
        n = int.from_bytes(m[j + 4:j + 8], "little")
        if 0 < n < 2_000_000 and m[j + 8:j + 10] == b"\xff\xd8" and j + 8 + n <= len(m):
            idx.append((j + 8, n))
            i = j + 8 + n
        else:
            i = j + 4
    return idx


class AviStore(_Base):
    """A local MJPG AVI (whole, or one piece) + frames.csv. first_frame: the recording's frame number of the
    piece's first whole picture (only needed for a piece that is neither the start nor the end of the video)."""

    def __init__(self, video, frames_csv, first_frame=None):
        self.f = open(video, "rb")
        self.m = mmap.mmap(self.f.fileno(), 0, access=mmap.ACCESS_READ)
        times = read_frame_times(frames_csv)
        idx = avi_index(lambda o, n: self.m[o:o + n], len(self.m))
        if idx and all(o + n <= len(self.m) for o, n in idx[-3:]):
            first = 0
        else:
            idx = scan_chunks(self.m)
            if first_frame is not None:
                first = first_frame
            elif self.m[:4] == b"RIFF":
                first = 0
            else:                                            # the last piece of a split video
                first = max(times) + 1 - len(idx)
        self._off = {}
        self._t = {}
        for k, oc in enumerate(idx):
            fr = first + k
            if fr in times:
                self._off[fr] = oc
                self._t[fr] = times[fr]
        self.frames = list(self._t)
        self._finish()

    def jpeg(self, frame):
        oc = self._off.get(frame)
        if oc is None:
            return None
        return bytes(self.m[oc[0]:oc[0] + oc[1]])

    def close(self):
        try:
            self.m.close()
            self.f.close()
        except Exception:
            pass


class FolderStore(_Base):
    """A folder of pictures named by frame number (000123.jpg / 123.png / frame_000123.jpg)."""

    def __init__(self, folder, frames_csv=None, fps=None):
        import re
        self.paths = {}
        for name in os.listdir(folder):
            mt = re.search(r"(\d+)\.(jpe?g|png)$", name, re.I)
            if mt:
                self.paths[int(mt.group(1))] = os.path.join(folder, name)
        times = read_frame_times(frames_csv) if frames_csv else {}
        self._t = {}
        for fr in self.paths:
            if fr in times:
                self._t[fr] = times[fr]
            elif not times:
                self._t[fr] = fr / float(fps or 25.0)
        self.frames = list(self._t)
        self._finish()

    def jpeg(self, frame):
        p = self.paths.get(frame)
        if p is None:
            return None
        with open(p, "rb") as f:
            b = f.read()
        if b[:2] != b"\xff\xd8":
            ok, enc = cv2.imencode(".jpg", cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR))
            b = enc.tobytes()
        return b


class CvStore(_Base):
    """Any video OpenCV reads. Slow to jump around in; fine for a few minutes."""

    def __init__(self, video, frames_csv=None, fps=None):
        self.cap = cv2.VideoCapture(video)
        n = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        times = read_frame_times(frames_csv) if frames_csv else {}
        rate = fps or self.cap.get(cv2.CAP_PROP_FPS) or 25.0
        self._t = {k: times.get(k, k / rate) for k in range(n)} if not times else {k: times[k] for k in range(n)
                                                                                     if k in times}
        self.frames = list(self._t)
        self._finish()
        self.pos = None
        import threading
        self.lock = threading.Lock()            # the tracker and the page read from different threads

    def _read(self, frame):
        with self.lock:
            return self._read_locked(frame)

    def _read_locked(self, frame):
        if self.pos != frame:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame)
        ok, img = self.cap.read()
        self.pos = frame + 1 if ok else None
        return img if ok else None

    def jpeg(self, frame):
        img = self._read(frame)
        if img is None:
            return None
        return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tobytes()

    def gray(self, frame):
        img = self._read(frame)
        return None if img is None else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def open_store(path, frames_csv=None, fps=None, first_frame=None):
    """The right store for what was given: a fetched stretch folder, a recording folder (video.avi +
    frames.csv), a folder of pictures, or a video file."""
    if os.path.isdir(path):
        if os.path.exists(os.path.join(path, "stretch_index.csv")):
            return JpegStore(path)
        fc = frames_csv or (os.path.join(path, "frames.csv") if os.path.exists(os.path.join(path, "frames.csv"))
                            else None)
        vid = os.path.join(path, "video.avi")
        if os.path.exists(vid) and fc:
            return AviStore(vid, fc, first_frame)
        sub = os.path.join(path, "frames")
        if os.path.isdir(sub) and not any(n.lower().endswith((".jpg", ".png")) for n in os.listdir(path)):
            path = sub
        return FolderStore(path, fc, fps)
    if not frames_csv:
        cand = os.path.join(os.path.dirname(path), "frames.csv")
        frames_csv = cand if os.path.exists(cand) else None
    with open(path, "rb") as f:
        head = f.read(12)
    if frames_csv and (head[:4] == b"RIFF" or path.lower().endswith((".bin",)) or ".part-" in path):
        return AviStore(path, frames_csv, first_frame)
    return CvStore(path, frames_csv, fps)


# ============================================================================ fetching from a release
class RemoteFile:
    """One file on the web (or several parts that together are one file), read by byte ranges."""

    def __init__(self, urls, opener=None):
        self.urls = list(urls)
        self.open = opener or (lambda req: urllib.request.urlopen(req, timeout=60))
        self.sizes = [self._size(u) for u in self.urls]
        self.starts = [sum(self.sizes[:k]) for k in range(len(self.sizes))]
        self.size = sum(self.sizes)
        self.downloaded = 0

    def _size(self, url):
        req = urllib.request.Request(url, headers={"Range": "bytes=0-0"})
        with self.open(req) as r:
            cr = r.headers.get("Content-Range")
            if cr and "/" in cr:
                return int(cr.rsplit("/", 1)[1])
            return int(r.headers.get("Content-Length") or 0)

    def read(self, off, n):
        """n bytes from offset off of the whole file (across parts)."""
        out = bytearray()
        end = min(self.size, off + n)
        while off < end:
            k = bisect.bisect_right(self.starts, off) - 1
            lo = off - self.starts[k]
            hi = min(self.sizes[k], end - self.starts[k]) - 1
            req = urllib.request.Request(self.urls[k], headers={"Range": "bytes=%d-%d" % (lo, hi)})
            for attempt in range(4):
                try:
                    with self.open(req) as r:
                        got = bytearray()
                        while True:
                            b = r.read(1 << 20)
                            if not b:
                                break
                            got += b
                    break
                except OSError:
                    if attempt == 3:
                        raise
            if len(got) != hi - lo + 1:
                raise IOError("short download: got %d of %d bytes" % (len(got), hi - lo + 1))
            out += got
            self.downloaded += len(got)
            off += len(got)
        return bytes(out)


def fetch(out_dir, urls, frames_csv_url, t0, t1, extra_ranges=(), meta=None, before_s=1.0, opener=None,
          log=print):
    """Download the pictures from t0 - before_s to t1 (Pi clock) of a recording on the web into out_dir as a
    JpegStore. urls: the video (or its parts, in order); frames_csv_url: its frames.csv. extra_ranges: more
    (t_from, t_to) spans to keep, or ("start", s_from, s_to) in seconds after the recording's first picture (e.g. a
    picture of the empty board). meta: written to stretch.json."""
    os.makedirs(out_dir, exist_ok=True)
    op = opener or (lambda req: urllib.request.urlopen(req, timeout=60))
    fc_path = os.path.join(out_dir, "frames.csv")
    if not os.path.exists(fc_path):
        log("Getting the recording's frames.csv ...")
        with op(urllib.request.Request(frames_csv_url)) as r, open(fc_path + ".part", "wb") as f:
            f.write(r.read())
        os.replace(fc_path + ".part", fc_path)
    times = read_frame_times(fc_path)
    rf = RemoteFile(urls, opener)
    log("Reading the video's index (%.2f GB video, only the index is downloaded) ..." % (rf.size / 1e9))
    idx = avi_index(rf.read, rf.size)
    if not idx:
        raise SystemExit("This video has no index the tool can read; download the whole video instead.")
    frames = sorted(times)
    start = times[frames[0]]
    spans = [(t0 - before_s, t1)] + [(start + r[1], start + r[2]) if len(r) == 3 else tuple(r) for r in extra_ranges]
    want = sorted({fr for fr in frames if fr < len(idx) and any(a <= times[fr] <= b for a, b in spans)})
    if not want:
        raise SystemExit("No pictures in that time range.")
    # group neighbouring pictures into runs and download each run in one go
    runs, cur = [], [want[0]]
    for fr in want[1:]:
        if fr == cur[-1] + 1:
            cur.append(fr)
        else:
            runs.append(cur)
            cur = [fr]
    runs.append(cur)
    total = sum(idx[r[-1]][0] + idx[r[-1]][1] - idx[r[0]][0] for r in runs)
    log("Downloading %d pictures (%.0f MB) ..." % (len(want), total / 1e6))
    jp = os.path.join(out_dir, "stretch.jpgs")
    rows, pos, done = [], 0, 0
    with open(jp + ".part", "wb") as g:
        for run in runs:
            a = idx[run[0]][0]
            b = idx[run[-1]][0] + idx[run[-1]][1]
            for c0 in range(a, b, 32 << 20):                 # 32 MB at a time, so progress shows
                c1 = min(b, c0 + (32 << 20))
                blob = rf.read(c0, c1 - c0)
                g.write(blob)
                done += len(blob)
                log("  %3.0f%%  (%.0f of %.0f MB)" % (100.0 * done / total, done / 1e6, total / 1e6))
            for fr in run:
                o, n = idx[fr]
                rows.append((fr, times[fr], pos + (o - a), n))
            pos += b - a
    os.replace(jp + ".part", jp)
    with open(os.path.join(out_dir, "stretch_index.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "t_wall", "offset", "size"])
        for r in rows:
            w.writerow([r[0], "%.4f" % r[1], r[2], r[3]])
    m = dict(meta or {})
    m.setdefault("source", dict(video=list(urls), frames=frames_csv_url))
    m["recording_start_t"] = start                           # the first picture's time (times after the start)
    with open(os.path.join(out_dir, "stretch.json"), "w") as f:
        json.dump(m, f, indent=1)
    os.remove(fc_path)                                       # its times are in stretch_index.csv now
    log("Saved in %s" % out_dir)
    return out_dir


if __name__ == "__main__":
    sys.exit("Use tools/label/mark_robot.py (this file is its footage reader).")
