"""Camera tracking spread over the Pi's cores (one system, 2026-10-28; Nick 18:56: "optimise the Pi's load, not just
split it"). Item 2 of docs/build-reports/e02-oct28-one-system.md.

Before: the camera bridge (cam_bridge.py) and the ladder track both cameras in ONE Python process. The recorder
JPEG-encodes a /live stream for each camera and every tracker decodes it again; every picture is processed whole,
at full size, even while every robot stands still. On the Pi that was 70-87 ms an overhead picture and 92-120 ms a
side picture: 12 and 8 tracked a second, half of the pictures dropped (27 Oct ladder, tracking_times.csv).

Now, each step a setting so each can be measured (tools/fleet/bench_trackfarm.py):
  (a) shared pictures: the recorder (camrec.py --shm) writes each camera's decoded grey picture once into shared memory
      (e02_shmframes.FrameRing); the trackers read it there: no JPEG encode for /live, no decode in the tracker;
  (b) windows: between full scans a camera's worker looks only at a window round each robot's predicted spot (its
      floor memory, outlines and motion inside the window); a full scan every ROI_EVERY pictures, and whenever a robot
      is not seen for sure (lost, unsure, hidden), so new objects and lost robots are still found;
  (c) half size (a setting, off by default): the picture halved before tracking (320 x 240);
  (d) one process per camera, each pinned to its own core (os.sched_setaffinity); the fusion (who is who, both
      cameras' verdicts, the lights, the fixes) stays in the bridge's process: the workers send each picture's
      outlines with the Pi-clock time it was taken, and the bridge matches the two cameras by those times (as
      e02_twocam does inside one process);
  (e) still pictures skipped (a setting, off by default: STILL_SKIP): while every robot is seen for sure, its own pose
      (run_plan's echo) says it is not moving, and the windows did not change, a camera tracks one picture in
      STILL_EVERY.

The image work is e02_camera.MotionTracker's own (floor memory, outlines, learning), moved to the worker; the
fusion is e02_twocam.TwoCamTracker's own (_fuse_view, _decide), fed with the workers' outlines instead of pictures.
"""
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "recognition"), TOOLS):
    if p not in sys.path:
        sys.path.append(p)

import cv2  # noqa: E402

ROI_EVERY = 8          # chosen: a full-picture scan every 8th picture (about 3 a second at 25): a new object or a robot
                       # put down by hand is found within 0.3 s. DOE note: 4 / 8 / 16 against the cost per picture and
                       # the brick watch's time to confirm (simulator drops) and identity swaps (crossings)
ROI_SCALE = 2.2        # chosen: a window's half-size, in robot widths, round the predicted spot (the outline is ~1.1
                       # widths across; the rest covers prediction error and the dilation). DOE note: 1.6 / 2.2 / 3.0
                       # against "missed" verdicts in windowed pictures (replay) and the cost
ROI_MOVE_S = 0.25      # the window also grows by the robot's speed times this (its own pose's speed). No DOE: a
                       # margin for a stale snapshot
STILL_EVERY = 5        # chosen: with every robot still and seen, track one picture in 5 (5 a second at 25). DOE note:
                       # 3 / 5 / 10 against how late a robot that starts to move is first seen moving (replay)
STILL_CMS = 0.3        # a robot's own pose moving slower than this over STILL_WINDOW_S is still. No DOE: pose noise
STILL_WINDOW_S = 0.5
STILL_GREY = 2.0       # ... and its window's mean grey change since the last tracked picture under this. No DOE: noise
STILL_SKIP = False     # (e) off by default: on the 27 Oct L0 replay it cut the Pi cost to ~11 ms a picture but the
                       # side-only robot (Center) got spots up to 6.7 cm off its standing spot (full tracker: 0.75 cm).
                       # DOE note: on / off against spread about the standing spot and the cost (bench_trackfarm.py real)
HALF = False           # (c) track at half size. DOE note: 320 x 240 against 640 x 480 on the replay: position error
                       # against the full-size tracker and the cost per picture (bench_trackfarm.py measures both)


class ScaledCam:
    """A camera model for pictures resized by k (0.5 = half): pixels scale, the floor does not."""

    def __init__(self, cam, k):
        self._c = cam
        self.k = float(k)
        W, H = cam.image_size
        self.image_size = (int(round(W * k)), int(round(H * k)))

    def project(self, xyz):
        u, v = self._c.project(xyz)
        k = self.k
        if np.ndim(u):
            return (np.asarray(u) + 0.5) * k - 0.5, (np.asarray(v) + 0.5) * k - 0.5
        return float((u + 0.5) * k - 0.5), float((v + 0.5) * k - 0.5)

    def to_floor(self, u, v, z=0.0):
        k = self.k
        if np.ndim(u):
            return self._c.to_floor((np.asarray(u, float) + 0.5) / k - 0.5, (np.asarray(v, float) + 0.5) / k - 0.5, z)
        return self._c.to_floor((u + 0.5) / k - 0.5, (v + 0.5) / k - 0.5, z)

    def px_per_cm(self, x, y, z=0.0):
        return self._c.px_per_cm(x, y, z) * self.k

    def __getattr__(self, name):
        return getattr(self._c, name)


class _Mirror:
    """A track as a worker sees it (the fusion's snapshot): enough for the learning mask and the windows."""
    __slots__ = ("name", "x", "y", "bx", "by", "bt", "status", "vx", "vy", "t_snap", "need_lights", "still")

    def __init__(self, name, x, y, t):
        self.name, self.x, self.y, self.bx, self.by, self.bt = name, x, y, x, y, t
        self.status, self.vx, self.vy, self.t_snap, self.need_lights, self.still = "seen", 0.0, 0.0, t, False, False


def make_engine_class(base_kind):
    """The image half of a tracker, for one camera (base_kind "overhead" or "side")."""
    import e02_camera as CAM
    import e02_multitrack as MT
    import e02_twocam as TW

    class Engine(MT.MultiTracker):
        """MultiTracker's picture work (blur, motion, offset, outlines, floor memory) with _assign replaced: the
        outlines are kept for the fusion, not assigned here. Windows (ROI) between full scans."""

        def __init__(self, cam, board_rect, margin_cm, z_cm, size_per_spot, roi=True, roi_every=ROI_EVERY,
                     still_skip=STILL_SKIP):
            cx, cy = (board_rect[0] + board_rect[2]) / 2.0, (board_rect[1] + board_rect[3]) / 2.0
            super().__init__(cam, board_center=(cx, cy), board_rect=board_rect, margin_cm=margin_cm, z_cm=z_cm,
                             size_per_spot=size_per_spot)
            self.kind = base_kind
            self.roi, self.roi_every, self.still_skip = bool(roi), int(roi_every), bool(still_skip)
            self.px_off = (0, 0)
            self.out_cands = []
            self.full = True
            self.since_full = 0
            self.since_track = 0
            self.last_win = None          # {name: mean grey} of the windows at the last tracked picture
            self.stats = dict(full=0, roi=0, skipped=0)

        # pixels of a window are offset from the picture's
        def _floor(self, u, v):
            return super()._floor(u + self.px_off[0], v + self.px_off[1])

        def _assign(self, cands, t):
            self.out_cands = cands

        def set_background(self, frames, robots=None):
            super().set_background(frames, robots)
            with self.lock:
                TW.tune_unknown(self, robots or [])

        def _covered(self, shape):
            m = np.zeros(shape, np.uint8)
            for tr in list(self.tracks.values()):
                grow = 0.0
                if tr.status != "seen":
                    grow = min(1.5, MT.GATE_GROW_CMS * max(0.0, (self.frame_t or tr.bt) - tr.bt) / CAM.ROBOT_CM)
                m |= self._disk_mask(shape, [(tr.x, tr.y)], 1.25 + grow)
            return m

        # --- windows -------------------------------------------------------------------------------------
        def windows(self, t):
            """[(name, (u0, v0, u1, v1))] round each track's predicted spot (pixels), clipped to the picture."""
            H, W = self.bg.shape
            out = []
            for tr in self.tracks.values():
                dt = max(0.0, t - tr.t_snap)
                x, y = tr.x + tr.vx * dt, tr.y + tr.vy * dt
                u, v = self._px_full(x, y)
                if not (np.isfinite(u) and np.isfinite(v)):
                    return None
                sp = math.hypot(tr.vx, tr.vy)
                r = ROI_SCALE * self._rpx_at(x, y) + sp * ROI_MOVE_S * self.cam.px_per_cm(x, y, self.z)
                u0, v0 = int(max(0, math.floor(u - r))), int(max(0, math.floor(v - r)))
                u1, v1 = int(min(W, math.ceil(u + r) + 1)), int(min(H, math.ceil(v + r) + 1))
                if u1 - u0 < 4 or v1 - v0 < 4:
                    return None
                out.append((tr.name, [u0, v0, u1, v1]))
            # overlapping windows become one (two robots close together: one outline may cover both)
            merged = True
            while merged:
                merged = False
                for i in range(len(out)):
                    for j in range(i + 1, len(out)):
                        a, b = out[i][1], out[j][1]
                        if a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]:
                            out[i] = (out[i][0] + "+" + out[j][0], [min(a[0], b[0]), min(a[1], b[1]),
                                                                    max(a[2], b[2]), max(a[3], b[3])])
                            del out[j]
                            merged = True
                            break
                    if merged:
                        break
            return out

        def _px_full(self, x, y):
            return MT.MultiTracker._px(self, x, y)

        def need_full(self):
            if not self.roi or self.bg is None or not self.tracks:
                return True
            if self.since_full >= self.roi_every - 1:
                return True
            return any(tr.status != "seen" for tr in self.tracks.values())

        def all_still(self, t):
            return bool(self.tracks) and all(tr.still and tr.status == "seen" for tr in self.tracks.values())

        def step(self, img, t):
            """One picture: 'full', 'roi' or 'skipped'. Outlines in self.out_cands (picture coordinates)."""
            if self.bg is None:
                self.push(img, t)
                return "full"
            full = self.need_full()
            if not full and self.still_skip and self.all_still(t) and self.since_track < STILL_EVERY - 1:
                if self._windows_unchanged(img, t):
                    self.since_track += 1
                    self.since_full += 1
                    self.stats["skipped"] += 1
                    return "skipped"
            self.since_track = 0
            if full:
                self.push(img, t)
                self.since_full = 0
                self.stats["full"] += 1
                self._remember_windows(img, t)
                return "full"
            wins = self.windows(t)
            if wins is None:
                self.push(img, t)
                self.since_full = 0
                self.stats["full"] += 1
                return "full"
            self._roi_push(img, t, wins)
            self.since_full += 1
            self.stats["roi"] += 1
            self._remember_windows(img, t, wins)
            return "roi"

        def _remember_windows(self, img, t, wins=None):
            if not self.still_skip:
                return
            wins = wins if wins is not None else (self.windows(t) if self.tracks else None) or []
            self.last_win = {n: float(img[w[1]:w[3]:4, w[0]:w[2]:4].mean()) for n, w in wins}

        def _windows_unchanged(self, img, t):
            if not self.last_win:
                return False
            wins = self.windows(t)
            if not wins:
                return False
            for n, w in wins:
                ref = self.last_win.get(n)
                if ref is None or abs(float(img[w[1]:w[3]:4, w[0]:w[2]:4].mean()) - ref) > STILL_GREY:
                    return False
            return True

        def _roi_push(self, img, t, wins):
            """MultiTracker.push inside the windows only: no learning (the full scans learn), no offset update."""
            with self.lock:
                self.n += 1
                self.n_frames += 1
                self.frame_t = t
                if self.prev is None or self.prev.shape != self.bg.shape:
                    self.prev = cv2.GaussianBlur(img, (3, 3), 0).astype(np.float32)
                cut_full = self._cut_mask(self.bg.shape) if getattr(self, "walls", None) or self.board_rect else None
                cands = []
                saved = (self.bg, self.known, getattr(self, "cut", None))
                try:
                    for _, (u0, v0, u1, v1) in wins:
                        # a margin of 3 px so the blur and the morphology see past the window's edge
                        a0, b0 = max(0, u0 - 3), max(0, v0 - 3)
                        # (the full picture's size: self.bg is the previous window's crop inside this loop)
                        a1, b1 = min(saved[0].shape[1], u1 + 3), min(saved[0].shape[0], v1 + 3)
                        g = cv2.GaussianBlur(img[b0:b1, a0:a1], (3, 3), 0).astype(np.float32)
                        motion = (np.abs(g - self.prev[b0:b1, a0:a1]) > self.MOTION_THR).astype(np.uint8)
                        self.prev[b0:b1, a0:a1] = g
                        self.bg = saved[0][b0:b1, a0:a1]
                        self.known = saved[1][b0:b1, a0:a1]
                        self.cut = cut_full[b0:b1, a0:a1] if cut_full is not None else None
                        self.px_off = (a0, b0)
                        self.in_window = True
                        cs = [c for c in self.candidates(g, motion) if self.on_area(c["x"], c["y"])]
                        for c in cs:                     # back to picture coordinates
                            c["u"] += a0
                            c["v"] += b0
                        cands += cs
                finally:
                    self.bg, self.known, self.cut = saved
                    self.px_off = (0, 0)
                    self.in_window = False
                self.last_cands = cands
                self.out_cands = cands

        def _cut_mask(self, shape):
            cut = getattr(self, "cut", None)
            if cut is not None and getattr(self, "in_window", False):
                return cut                               # a window's crop of the full mask (_roi_push)
            if cut is not None and cut.shape != shape:
                self.cut = None
            return super()._cut_mask(shape)

        # --- lights (overhead): bright spots near the robots the fusion does not see by their body -------------
        def light_spots(self, raw):
            need = [tr for tr in self.tracks.values() if tr.need_lights]
            if not need or self.bg is None:
                return []
            g = raw.astype(np.float32)
            d = g - self.offset - self.bg
            m = ((d > TW.LIGHT_THR) & (g >= TW.LIGHT_ABS)).astype(np.uint8)
            n, lab, stats, cents = cv2.connectedComponentsWithStats(m, connectivity=8)
            spots = []
            for k in range(1, n):
                a = int(stats[k, cv2.CC_STAT_AREA])
                if a > 0.35 * self.area:
                    continue
                x, y = MT.MultiTracker._floor(self, *cents[k])
                if np.isfinite(x) and np.isfinite(y) and self.on_area(x, y):
                    spots.append([x, y, float(a)])
            merged = []
            for s in sorted(spots, key=lambda q: -q[2]):
                for o in merged:
                    if math.hypot(s[0] - o[0], s[1] - o[1]) < TW.LIGHT_PAIR_CM:
                        w = o[2] + s[2]
                        o[0], o[1], o[2] = (o[0] * o[2] + s[0] * s[2]) / w, (o[1] * o[2] + s[1] * s[2]) / w, w
                        break
                else:
                    merged.append(list(s))
            return merged

    return Engine


class CamWorker:
    """One camera's tracking: pictures in (a FrameRing, a stream, or step() calls), outlines out. Runs inline (tests,
    the bench) or as its own process (run_worker)."""

    def __init__(self, kind, cam, board_rect, margin_cm=8.0, starts=None, half=HALF, roi=True, still_skip=STILL_SKIP,
                 roi_every=ROI_EVERY, side_gain="auto", lights=True, floor_frames=5):
        import e02_twocam as TW
        import e02_camera as CAM
        self.kind = kind
        self.k = 0.5 if half else 1.0
        self.cam = ScaledCam(cam, self.k) if half else cam
        z = TW.SIDE_Z_CM if kind == "side" else CAM.ROBOT_TOP_CM
        E = make_engine_class(kind)
        self.eng = E(self.cam, board_rect, margin_cm, z, kind == "side", roi=roi, roi_every=roi_every,
                     still_skip=still_skip)
        self.eng.tracks = {}
        self.starts = dict(starts or {})
        for n, (x, y) in self.starts.items():
            self.eng.tracks[n] = _Mirror(n, x, y, 0.0)
        self.side_gain = side_gain if kind == "side" else 1.0
        self.lights = bool(lights) and kind == "overhead"
        self.floor_frames = int(floor_frames)
        self.first = []
        self.cost = []          # (t, ms, mode)

    def prep(self, img):
        if self.side_gain not in (1.0, "auto"):
            img = cv2.convertScaleAbs(img, alpha=self.side_gain)
        if self.k != 1.0:
            img = cv2.resize(img, self.cam.image_size, interpolation=cv2.INTER_AREA)
        return img

    def snapshot(self, snap):
        """The fusion's view of the robots: {name: dict(x, y, bx, by, bt, status, vx, vy, t, need_lights, still)}."""
        for n, d in snap.items():
            tr = self.eng.tracks.get(n)
            if tr is None:
                tr = self.eng.tracks[n] = _Mirror(n, d["x"], d["y"], d["t"])
            tr.x, tr.y, tr.bx, tr.by, tr.bt = d["x"], d["y"], d.get("bx", d["x"]), d.get("by", d["y"]), d.get("bt", d["t"])
            tr.status, tr.vx, tr.vy, tr.t_snap = d["status"], d.get("vx", 0.0), d.get("vy", 0.0), d["t"]
            tr.need_lights, tr.still = bool(d.get("need_lights")), bool(d.get("still"))

    def step(self, img, t):
        """One picture (grey, full size) at its Pi time t. Returns the record for the fusion, or None (still being
        set up, or skipped)."""
        c0 = time.thread_time()
        if len(self.first) < self.floor_frames and self.eng.bg is None:
            self.first.append(img)
            if len(self.first) >= self.floor_frames:
                if self.side_gain == "auto":
                    # a dark side picture is brightened (e02_twocam.set_side_background: the 13:01 lock, mean 20)
                    import e02_twocam as TW
                    mean = float(np.median([float(np.mean(f)) for f in self.first]))
                    self.side_gain = 1.0 if mean >= TW.DARK_MEAN else float(min(TW.DARK_GAIN_MAX,
                                                                              TW.DARK_TARGET / max(mean, 1.0)))
                self.eng.set_background([self.prep(f) for f in self.first], robots=list(self.starts.values()))
            self.cost.append((t, (time.thread_time() - c0) * 1000.0, "setup"))
            return None
        g = self.prep(img)
        mode = self.eng.step(g, t)
        rec = None
        if mode != "skipped":
            cands = [{k: (float(v) if isinstance(v, (float, np.floating)) else v) for k, v in c.items()}
                     for c in self.eng.out_cands]
            if self.k != 1.0:                          # outlines' pixels and areas back in the full picture's units
                for c in cands:
                    c["u"] = (c["u"] + 0.5) / self.k - 0.5
                    c["v"] = (c["v"] + 0.5) / self.k - 0.5
                    c["area"] = c["area"] / (self.k * self.k)
                    c["rw"] = c["rw"] / self.k
                    c["rh"] = c["rh"] / self.k
            lights = self.eng.light_spots(g) if self.lights else []
            rec = {"cam": self.kind, "t": t, "cands": cands, "lights": [[float(a), float(b), float(w)]
                                                                       for a, b, w in lights], "mode": mode}
        ms = (time.thread_time() - c0) * 1000.0
        self.cost.append((t, ms, mode))
        if rec is not None:
            rec["ms"] = round(ms, 2)
        return rec


def run_worker(conn, cfg):
    """A camera worker's process: pinned to its core, pictures from its FrameRing (or stream), outlines to conn."""
    if cfg.get("core") is not None and hasattr(os, "sched_setaffinity"):
        try:
            os.sched_setaffinity(0, {int(cfg["core"])})
        except OSError:
            pass
    import e02_camera as CAM
    import e02_twocam as TW
    if cfg["kind"] == "side":
        cam = TW.load_side(cfg["calib"])[0]
    else:
        cam = CAM.load_camera(cfg["calib"])
    w = CamWorker(cfg["kind"], cam, tuple(cfg["rect"]), cfg.get("margin_cm", 8.0), cfg.get("starts"),
                  half=cfg.get("half", HALF), roi=cfg.get("roi", True), still_skip=cfg.get("still_skip", STILL_SKIP),
                  roi_every=cfg.get("roi_every", ROI_EVERY), side_gain=cfg.get("side_gain", "auto"),
                  lights=cfg.get("lights", True))
    src = None
    ring = None
    if cfg.get("ring"):
        import e02_shmframes as SF
        t0 = time.time()
        while ring is None and time.time() - t0 < 20:
            try:
                ring = SF.FrameRing.attach(cfg["ring"])
            except (FileNotFoundError, ValueError):
                time.sleep(0.2)
        if ring is None:
            conn.send({"error": "no shared pictures %s" % cfg["ring"]})
            return
    else:
        import e02_trackproc as TP
        src = iter(TP.StreamPics(cfg["stream"]))
    last = -1
    every = 1.0 / float(cfg.get("max_hz", 30.0))
    t_last = 0.0
    try:
        while True:
            while conn.poll():
                m = conn.recv()
                if m.get("stop"):
                    return
                if "snap" in m:
                    w.snapshot(m["snap"])
                if "robot_at" in m:              # the bridge's seed (a robot found by its lights, cam_bridge.on_seed)
                    try:
                        w.eng.robot_at(*m["robot_at"])
                    except Exception:            # noqa: BLE001  (before its floor memory: nothing to forget)
                        pass
            if ring is not None:
                n = ring.newest()
                if n <= last:
                    time.sleep(0.003)
                    continue
                got = ring.read(n)
                if got is None:
                    continue
                img, meta = got
                last = n
                t = meta[1] if np.isfinite(meta[1]) else meta[0]
            else:
                pic = next(src)
                img = cv2.imdecode(np.frombuffer(pic.jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
                t = pic.t_pi
            if t - t_last < every - 1e-3:
                continue
            t_last = t
            rec = w.step(img, t - cfg.get("capture_s", 0.04))
            if rec is not None:
                rec["behind_s"] = round(time.time() - t, 3)
                conn.send(rec)
    except (EOFError, BrokenPipeError, KeyboardInterrupt, StopIteration):
        return
    finally:
        if ring is not None:
            ring.close()


def make_fusion_class(base):
    """The bridge's tracker (cam_bridge's OdoTracker, a TwoCamTracker) fed with the workers' outlines instead of
    pictures: apply(rec) does what push() / push_side() did after their picture work."""
    import e02_twocam as TW

    class Fusion(base):
        farm = True

        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.merged_lights = []
            self.rec_counts = dict(overhead=0, side=0)

        def apply(self, rec):
            t = rec["t"]
            if rec["cam"] == "overhead":
                with self.lock:
                    self.n += 1
                    self.n_frames += 1
                    self.frame_t = t
                    self.last_cands = rec["cands"]
                    self.merged_lights = rec.get("lights") or []
                    self._fuse_view(self, rec["cands"], t)
            elif self.side is not None:
                self.side_n += 1
                self.side.frame_t = t
                with self.lock:
                    self._fuse_view(self.side, rec["cands"], t)
            self.rec_counts[rec["cam"]] = self.rec_counts.get(rec["cam"], 0) + 1

        def _lights(self, trs, preds, t):
            need = [i for i, tr in enumerate(trs) if not self._body_seen(tr, t)]
            self.light_spots = []
            if not need:
                return
            merged = self.merged_lights
            self.light_spots = [(round(a, 1), round(b, 1)) for a, b, _ in merged]
            for i in need:
                tr = trs[i]
                h = self._heading_at(tr, t)
                best = None
                for lx, ly, _ in merged:
                    cx, cy = (lx - TW.LIGHT_FWD_CM * math.cos(math.radians(h)),
                              ly - TW.LIGHT_FWD_CM * math.sin(math.radians(h))) if h is not None else (lx, ly)
                    dd = math.hypot(cx - preds[i][0], cy - preds[i][1])
                    if dd > TW.LIGHT_GATE_CM + TW.MT.GATE_GROW_CMS * min(1.2, max(0.0, t - tr.bt)):
                        continue
                    others = [math.hypot(lx - preds[k][0], ly - preds[k][1]) for k in range(len(trs)) if k != i]
                    if any(o < math.hypot(lx - preds[i][0], ly - preds[i][1]) + TW.AMBIG_LIGHT_CM for o in others):
                        continue
                    if best is None or dd < best[0]:
                        best = (dd, cx, cy, h is not None)
                if best is not None:
                    tr.obs["lights"] = (t, best[1], best[2], "seen", "lights" + ("" if best[3] else " (heading unknown)"))

        def snapshot(self, t):
            """What the workers need: each robot's spot, status, own-pose speed (windows), whether it needs its
            lights looked for, whether it is still."""
            out = {}
            with self.lock:
                for n, tr in self.tracks.items():
                    vx = vy = 0.0
                    still = False
                    own = getattr(self, "own_at", None)
                    if own is not None:
                        a, b = own(n, t - STILL_WINDOW_S), own(n, t)
                        if a is not None and b is not None and b[0] > a[0]:
                            dt = b[0] - a[0]
                            vx, vy = (b[1] - a[1]) / dt, (b[2] - a[2]) / dt
                            still = math.hypot(vx, vy) < STILL_CMS and abs((b[3] - a[3] + 180.0) % 360.0 - 180.0) < 2.0
                    out[n] = dict(x=float(tr.x), y=float(tr.y), bx=float(tr.bx), by=float(tr.by), bt=float(tr.bt),
                                  status=tr.status, vx=vx, vy=vy, t=t, still=still,
                                  need_lights=not self._body_seen(tr, t))
            return out

    return Fusion
