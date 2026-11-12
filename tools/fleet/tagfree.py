#!/usr/bin/env python3
"""Tag-free mode (12 Nov 2026, build "tag-free"): robots with no roof tag (the home robots carry none) found, named and
given a heading from what exists: the trained detector (detector-v1, docs/results/detector-v1/report.md), the motion
detector's outlines, each robot's own odometry and gyro, and the tracking layer's naming rules (assignment by
prediction, by elimination, the identity wiggle, and e18's out-and-back where it exists).

What this module adds (the tracking layer itself is unchanged; attach() hooks this in from outside):

  DetectorSource   the detector as a tracking-layer source, ON DEMAND: the layer calls it only for a robot that is
                   unaccounted, unsure (close pass, wiggle) or asked for (a heading in doubt), at the layer's capped rate
                   (ML_ROBOT_HZ a robot, ML_TOTAL_HZ in all). On top of that, its own load limit: the detector's measured
                   time per call against a DUTY share of one core, and a per-call TIME BUDGET (a call over it backs the
                   source off; three in a row and it drops the side camera). One picture's result serves every robot that
                   asks in that picture. It learns "robot + heading" only and NEVER names a robot.
                   Cameras: the overhead first (position + heading); the side camera only where the overhead found
                   nothing near the robot, and for POSITION ONLY (its heading is not learned: 84 deg median error).
                   Where it runs (Layout B: recorder core 0, planner core 1, bridge cores 2-3): in a worker thread of the
                   planner (free roam), so on the planner's core 1 (MIRLT_DETECTOR_CORE=recorder moves the worker to
                   core 0). The bridge's cores 2-3 were at 99 % on 11 Nov; they are never used.
  HeadingFuser     each robot's heading = its own (gyro) heading corrected by a slowly-changing offset the overhead
                   detector measures (a small Kalman filter on the offset). The detector's own noise is learnt from its
                   residuals (starting guess from the report). Two detector headings in a row far outside the joint noise:
                   the heading is IN DOUBT, the robot's heading check (the camera-measured crawl, e17) is asked and the
                   detector is asked for that robot until the check ends; no fused heading is passed meanwhile.
  SimDetector      the fast simulator's model of detector-v1 (recall, centre and heading error from the report), with
                   the simulator's broken guesses: the model missing, a wrong heading for a robot, robots parked half
                   over an edge, washed-out lamp patches, a slow Pi.
  attach()         the switch, decided automatically by default (MIRLT_TAGFREE=auto): on when the model file is there;
                   then the detector is used for a robot only while no roof tag has named it lately. =1 always (where
                   the model is), =0 off (the tracking layer as before). A missing model is said once and the layer runs
                   without it (degrades, never stops).

Model file: NOT in git (a GitHub release "detector-v1" is pending). Looked for at MIRLT_DETECTOR_MODEL, then
MIRLT_ML_MODEL, then ~/.config/mirlt/detector/detector-v1.onnx, then ~/.config/mirlt/detector/best.onnx.

Changelog
  2026-11-12a  first version (build tag-free)
  2026-11-12b  (Mission Control build: the Pi efficiency monitor) read-only stats: Budget.share_now() (the detector's
               share of its core over the last window) and share_now in its summary. The core share is a setting for
               the bench (docs/how-to/pi-bench-core-share.md): MIRLT_DETECTOR_DUTY (0.10 or 10 = 10 %), else
               ~/.config/mirlt/detector/share.json {"duty": 0.25}, else DUTY (0.25, unchanged). Nothing else changes.
  2026-11-12e20  (camera trust) every detection says its camera; each call RUNS the camera most likely to answer that
               question (position / heading / telling robots apart) for that robot's square (camera_trust.TrustBook.choose:
               Thompson sampling), the next one only where it found nothing near the robot; calls counted per camera. A
               detector heading is used only from a camera TRUSTED for heading there (learned from its headings against
               each robot's sure gyro heading), never because of its name (MIRLT_DETECTOR_ROUTE=e19: the first camera
               first, the heading flag per camera, as before). SimDetector: ml_cams=[...] several cameras with faults.
"""
import math
import os
import random
import threading
import time

VERSION = "tagfree-2026-11-12e20"   # e20: calls routed by learned camera trust, headings only from cameras trusted for heading (MIRLT_DETECTOR_ROUTE). Before: "tagfree-2026-11-12e19"   # e19 (audit 9, Nick): a heading is handed on only past the detector's measured 95 % error and the robot's own doubt; the 45 deg cap is a risk limit from its clearance (MIRLT_TAGFREE_HAND=risk; e18 the old rule). Before: "tagfree-2026-11-12b" (Mission Control's read-only stats hook), "tagfree-2026-11-12a"

SWITCH_ENV = "MIRLT_TAGFREE"          # auto (default) | 1 | 0
MODEL_ENVS = ("MIRLT_DETECTOR_MODEL", "MIRLT_ML_MODEL")
MODEL_DEFAULTS = ("~/.config/mirlt/detector/detector-v1.onnx", "~/.config/mirlt/detector/best.onnx")

# -- the load limit (chosen; DOE: yes, on the Pi bench, scored on fixes late / watchdog stops against time-to-name) --
DUTY = 0.25              # CHOSEN: the detector may use at most this share of one core, by its measured time per call
CALL_BUDGET_MS = 80.0    # CHOSEN: a call over this is "over budget" (the Pi 4 estimate is 15-30 ms; 2-3 x that)
OVER_N = 3               # CHOSEN: this many over-budget calls in a row: the side camera is dropped (overhead only)
COOLDOWN_K = 4.0         # CHOSEN: after an over-budget call, nothing for COOLDOWN_K x its time
MAX_AGE_S = 1.0          # CHOSEN: a picture older than this is not used (the recorder stalled)
TAG_FRESH_S = 3.0        # CHOSEN: auto mode: a robot a roof tag named within this long does not need the detector

# -- where a detection sits on the floor (the visible body's centre, as autolabel's labels) --
Z_OVERHEAD_CM = 3.7      # the robot's top (e02_camera.ROBOT_TOP_CM): the overhead sees the roof
Z_SIDE_CM = 2.0          # CHOSEN: about half the body: the side camera sees the near wall as much as the roof
SIGMA_OVERHEAD_CM = 1.6  # MEASURED (report): centre error median 2.0 cm / 95 % 3.7 cm -> about 1.6 cm a direction
# (e19, item 12: "prove the 2 cm out from logs") the side camera's floor height cannot be measured from the uploaded logs
# (the Pi's 3-D side calibration stays on the Pi; the run's own side model is a picture-fitted homography that already
# absorbs the height). Instead its effect is MEASURED live: every SIDE_CAL_EVERY-th detector call runs the side camera
# where the overhead found robots too, and the median side-minus-overhead offset (SIDE_CAL_N pairs or more) is removed
# from every side-only detection. MIRLT_SIDE_OFFSET_LEARN=0: the assumed Z_SIDE_CM alone (before e19).
SIDE_LEARN = os.environ.get("MIRLT_SIDE_OFFSET_LEARN", "1") != "0"
SIDE_CAL_EVERY = 10      # CHOSEN: one call in ten pays for the side camera's measurement (DOE: no; the load limit rules)
SIDE_CAL_N = 20          # CHOSEN: pairs before the offset is used (its median's error ~1.25 x 2.6 / sqrt(20) = 0.7 cm)
SIDE_PAIR_CM = 10.0      # CHOSEN: a side and an overhead detection this close are the same robot (robots are 13 cm wide)
SIGMA_SIDE_CM = 2.6      # DERIVED: the side camera's fit spread is 4.1 px against the overhead's 2.5 px (x 1.6)

# -- heading fusion --
SIGMA_DET0_DEG = 15.0    # MEASURED starting guess (report, overhead): median 12 deg (-> 17.8 if normal), 95 % 26 deg
                         # (-> 13.3); learnt from each site's own residuals once LEARN_N exist
SIGMA_DET_MIN = 5.0      # CHOSEN: never trusted more than this (label noise of the report is 10-20 deg)
LEARN_N = 20             # CHOSEN: residuals before the learnt noise replaces the starting guess
P0_DEG = 20.0            # CHOSEN: how unsure a robot's own heading is when the fuser starts (e17: re-seed jumps 13-15 deg)
Q_DEG_PER_S = 0.5        # CHOSEN: the gyro heading's slow drift a second (DOE: yes, from long stands with the camera)
Q_TURN = 0.07            # MEASURED (e17 section 5): curves turned up to 7 % more or less than asked
DOUBT_K = 3.0            # CHOSEN: an innovation over 3 x its joint sigma is an outlier ...
DOUBT_N = 2              # ... two in a row (within DOUBT_WIN_S): the heading is in doubt
DOUBT_WIN_S = 10.0
OWN_JUMP_DPS = 400.0     # CHOSEN: the own heading changing faster than this is a re-set (fpose), not a turn: P back to P0
TURN_MAX_DPS = 8.0       # as e17's re-seed rule: no heading is compared while the robot turns faster than this (its own
                         # pose reaches the layer ~0.3 s late: mid-turn that is 10-30 deg of pure timing error)
SIGNIF_K = 2.0           # CHOSEN: a fused heading is handed on only when the offset it measures is over 2 x its own sigma
                         # (the own heading is kept otherwise: a gyro is better than the detector over minutes)
HAND_MAX_DEG = 45.0      # CHOSEN: an offset over this is not handed on: from headings alone a robot whose own heading is
                         # far off and a detector that is wrong about that robot look the same, so the camera-measured
                         # heading check (a short crawl, e17) settles it. DOE: yes (e17's travel agreement is 23 deg at 95 %)
HAND_MIN_N = 2           # CHOSEN: at least two agreeing detector headings before one is handed on
# (e19, audit finding 9 + Nick: "the 45 deg limit must come from risk, not a number") MIRLT_TAGFREE_HAND "risk" (default):
# a heading is handed on only when the disagreement cannot be explained by the detector's MEASURED heading error at 95 %
# (DET95_DEG, the report's overhead number; once the site's own residuals exist, 1.96 x the learnt sigma, never less than
# the report's) and the robot's own heading doubt at 95 % (2 x the controller's head_sd: 2 deg + 1.6 per 90 deg turned
# since its heading was set) together. The detector's error on a STANDING robot is likely the same in every picture, so
# repeated pictures do not shrink it (the audit's 20 % wrong hand-ons came from re-testing 2 sigmas of an average). The
# 45 deg cap becomes the risk limit: a hand-on that could be wrong by that much must not be able to take the robot over
# its clearance (to an edge or a neighbour, less its position doubt) within HAND_HORIZON_CM of driving; past that, the
# camera-measured heading check (a short crawl) is asked instead. "e18": SIGNIF_K / HAND_MAX_DEG as before.
DET95_DEG = 26.0         # MEASURED (detector-v1 report: overhead heading error 12 deg median, 26 deg at 95 %, never flipped)
HAND_HORIZON_CM = 20.0   # CHOSEN: how far a robot drives on a handed-on heading before the camera's next correction can
                         # matter (about one planned piece). DOE: yes, with the detector on real standing robots
try:
    import safety_group as _SG
    HAND_RULE = _SG.value("MIRLT_TAGFREE_HAND")
except Exception:  # noqa: BLE001
    HAND_RULE = "risk"
MATCH_CM = 3.5           # as tracking_layer.CLUSTER_CM: a detection this near a fix is that fix's object
try:
    ROUTE = _SG.value("MIRLT_DETECTOR_ROUTE")      # (e20) "trust" | "e19"
except Exception:  # noqa: BLE001
    ROUTE = "trust"


def job_of(why):
    """(e20) The question a detector call answers, from the layer's reason for asking."""
    w = str(why or "").lower()
    if "heading" in w:
        return "heading"
    if any(k in w for k in ("unsure", "pass", "wiggle", "contested", "merged", "identity")):
        return "identity"
    return "position"


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def find_model(path=None):
    """The detector's .onnx: path, else the environment, else the default places. None when there is none."""
    cands = [path] if path else []
    cands += [os.environ.get(e) for e in MODEL_ENVS]
    cands += [os.path.expanduser(p) for p in MODEL_DEFAULTS]
    for p in cands:
        if p and os.path.exists(p):
            return p
    return None


def switch(env=None):
    v = (env if env is not None else os.environ).get(SWITCH_ENV, "auto").strip().lower()
    return v if v in ("auto", "1", "0") else "auto"


# ==================================================================================================================
SHARE_FILE = "~/.config/mirlt/detector/share.json"


def duty_setting(env=None, path=None):
    """(12 Nov, the bench) The detector's core share: MIRLT_DETECTOR_DUTY (a fraction, or a percent over 1), else the
    bench's saved choice (share.json), else DUTY. Returns (duty, where it came from)."""
    env = os.environ if env is None else env
    v = (env.get("MIRLT_DETECTOR_DUTY") or "").strip()
    if v:
        try:
            x = float(v)
            x = x / 100.0 if x > 1.0 else x
            if 0.0 < x <= 1.0:
                return x, "MIRLT_DETECTOR_DUTY"
        except ValueError:
            pass
    try:
        import json
        with open(os.path.expanduser(path or SHARE_FILE)) as f:
            x = float(json.load(f).get("duty"))
        if 0.0 < x <= 1.0:
            return x, "share.json (the bench's choice)"
    except (OSError, ValueError, TypeError, AttributeError):
        pass
    return DUTY, "the default"


class Budget:
    """The detector's load limit: calls allowed while (time spent in the last WINDOW s) / WINDOW <= duty, never during a
    cool-down after an over-budget call. Clock-agnostic (the simulator passes its own time)."""
    WINDOW = 4.0

    def __init__(self, duty=None, budget_ms=CALL_BUDGET_MS):
        if duty is None:
            duty, self.duty_from = duty_setting()
        else:
            self.duty_from = "given"
        self.duty, self.budget_ms = float(duty), float(budget_ms)
        self.spent = []          # [(t, ms)]
        self.cool_until = -1e18
        self.over_run = 0
        self.n = self.n_over = self.n_refused = 0
        self.ms_all = []

    def allow(self, t):
        self.spent = [(a, m) for a, m in self.spent if t - a < self.WINDOW]
        if t < self.cool_until or sum(m for _, m in self.spent) / 1000.0 > self.duty * self.WINDOW:
            self.n_refused += 1
            return False
        return True

    def done(self, t, ms):
        self.n += 1
        self.spent.append((t, ms))
        self.ms_all.append(ms)
        del self.ms_all[:-500]
        if ms > self.budget_ms:
            self.n_over += 1
            self.over_run += 1
            self.cool_until = t + COOLDOWN_K * ms / 1000.0
        else:
            self.over_run = 0

    def share_now(self, t):
        """(read-only, 12 Nov) The share of one core the detector used over the last WINDOW s (0-1)."""
        return sum(m for a, m in list(self.spent) if t - a < self.WINDOW) / 1000.0 / self.WINDOW

    def summary(self):
        ms = sorted(self.ms_all)
        return dict(calls=self.n, over_budget=self.n_over, refused_for_load=self.n_refused,
                    ms_median=round(ms[len(ms) // 2], 1) if ms else None,
                    ms_p95=round(ms[int(0.95 * (len(ms) - 1))], 1) if ms else None,
                    duty=self.duty, duty_from=getattr(self, "duty_from", ""), budget_ms=self.budget_ms)


# ==================================================================================================================
class Camera:
    """One camera for the detector: picture() -> (t, picture) with picture a JPEG (bytes) or a BGR / grey array, or
    None; to_floor(u, v) -> (x, y) in the run's cm; heading: whether its detector heading is used (overhead only)."""

    def __init__(self, name, picture, to_floor, heading, sigma):
        self.name, self.picture, self.to_floor, self.heading, self.sigma = name, picture, to_floor, bool(heading), sigma


def homography_to_floor(H_floor_to_image):
    """to_floor(u, v) from a floor -> picture homography (autolabel's floor_calibration.json)."""
    import numpy as np
    Hi = np.linalg.inv(np.asarray(H_floor_to_image, dtype=float))

    def f(u, v):
        p = Hi @ np.array([u, v, 1.0])
        return float(p[0] / p[2]), float(p[1] / p[2])
    return f


def camera_model_to_floor(cam, z_cm):
    """to_floor(u, v) from an e02_camera CameraModel (the bridge's calibration file) at the height z_cm."""
    return lambda u, v: tuple(float(q) for q in cam.to_floor(u, v, z_cm)[:2])


def floor_heading(to_floor, u, v, h_img_deg, step_px=10.0):
    """The detector's heading (deg, counter-clockwise on screen, 0 = picture right) as a floor heading (deg, CCW from
    +x): two points along it, both mapped to the floor."""
    r = math.radians(h_img_deg)
    x0, y0 = to_floor(u, v)
    x1, y1 = to_floor(u + step_px * math.cos(r), v - step_px * math.sin(r))
    return math.degrees(math.atan2(y1 - y0, x1 - x0))


class DetectorSource:
    """The detector on demand, as the tracking layer's ml_fn(name, t, why, (x, y, r)) -> [Det]. sync=True runs it in the
    caller (replays, tests); sync=False hands it to a worker thread and returns the newest finished result (the run's
    planner never waits on it)."""

    def __init__(self, model, cameras, sync=False, budget=None, say=None, core=None, clock=time.time):
        self.model = model                    # infer.OnnxDetector (or anything with __call__(bgr) -> [dict])
        self.cams = list(cameras)
        self.sync = bool(sync)
        self.budget = budget or Budget()
        self.say = say or (lambda s: None)
        self.clock = clock
        self.cache = {}                       # cam name -> (picture t, [det dict])
        self.result = None                    # (t_done, picture t by cam, [Det]) newest finished (async)
        self.given = None                     # the pictures (cam, t) of the result last handed to the layer
        self.counts = dict(asked=0, ran=0, pictures=0, dets=0, no_picture=0, stale=0, side_used=0, side_dropped=0,
                           errors=0, skipped_tag=0)
        self.side_off = False
        self.tag_seen = {}                    # robot -> t of its last roof-tag fix (auto mode)
        self.auto = True
        self._lock = threading.Lock()
        self._job = None
        self._ev = threading.Event()
        self.core = core
        self.book = None                      # (e20) the run's camera_trust.TrustBook (attach() sets it from the layer)
        self.calls_by_cam = {}                # (e20, 3) pictures run per camera
        if not self.sync:
            threading.Thread(target=self._worker, daemon=True, name="tagfree-detector").start()

    # -- the layer's call -------------------------------------------------------------------------------------------
    def __call__(self, name, t, why, region):
        self.counts["asked"] += 1
        ts = self.tag_seen.get(name)
        if self.auto and ts is not None and t - ts < TAG_FRESH_S:
            self.counts["skipped_tag"] += 1
            return []
        if self.sync:
            now = self.clock()
            if not self.budget.allow(now):
                return []
            res = self._run(region, now, why)
            if res[1] == self.given:
                return []                     # the same pictures as last time: already in
            self.given = res[1]
            return res[2]
        with self._lock:
            res = self.result
            busy = self._job is not None
        out = []
        if res is not None and res[1] != self.given:
            self.given = res[1]               # a result goes in once (it holds every robot the pictures show)
            out = res[2]
        if not busy and self.budget.allow(self.clock()):
            with self._lock:
                self._job = (region, why)
            self._ev.set()
        return out

    def _worker(self):
        if self.core is not None:
            try:
                os.sched_setaffinity(0, {int(c) for c in str(self.core).split(",")})   # this thread only (Linux)
            except (AttributeError, OSError, ValueError):
                pass
        while True:
            self._ev.wait()
            self._ev.clear()
            with self._lock:
                job = self._job
            region, why = job if isinstance(job, tuple) and len(job) == 2 and not isinstance(job[0], (int, float)) \
                else (job, None)
            try:
                res = self._run(region, self.clock(), why)
                with self._lock:
                    self.result = res
            except Exception:  # noqa: BLE001  (a failing detector never stops the run)
                self.counts["errors"] += 1
            finally:
                with self._lock:
                    self._job = None

    # -- one run of the detector over the cameras it needs ------------------------------------------------------------
    def _detect(self, cam):
        got = cam.picture()
        if got is None:
            self.counts["no_picture"] += 1
            return None
        pt, pic = got
        if self.clock() - pt > MAX_AGE_S and not self.sync:
            self.counts["stale"] += 1
            return None
        c = self.cache.get(cam.name)
        if c is not None and c[0] == pt:
            return c
        import numpy as np
        if isinstance(pic, (bytes, bytearray)):
            import cv2
            pic = cv2.imdecode(np.frombuffer(pic, np.uint8), cv2.IMREAD_COLOR)
            if pic is None:
                return None
        elif pic.ndim == 2:
            import cv2
            pic = cv2.cvtColor(pic, cv2.COLOR_GRAY2BGR)
        dets = [d for d in self.model(pic) if d.get("cls", "alvik") == "alvik"]
        self.counts["pictures"] += 1
        self.cache[cam.name] = (pt, dets)
        return self.cache[cam.name]

    def _to_dets(self, cam, pt, raw):
        import tracking_layer as TL
        out = []
        for d in raw:
            try:
                x, y = cam.to_floor(float(d["x"]), float(d["y"]))
            except Exception:  # noqa: BLE001
                continue
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            th = None
            # (e20) every camera's heading is read (its trust decides whether it is used); e19: the flag per camera
            if (cam.heading or (ROUTE != "e19" and self.book is not None)) and d.get("heading_deg") is not None:
                th = floor_heading(cam.to_floor, float(d["x"]), float(d["y"]), float(d["heading_deg"]))
            out.append(TL.Det("ml", pt, x, y, th, None, cam.sigma, score=float(d.get("score", 1.0)), cam=cam.name))
        return out

    def order(self, region, why=None):
        """(e20, 3) The cameras in the order to try: the one most likely to answer this question for this robot's
        square first (Thompson sampling on its learned score), then the rest as configured. e19 / no book: as configured."""
        if ROUTE == "e19" or self.book is None or len(self.cams) < 2:
            return list(self.cams)
        x, y = (region[0], region[1]) if region is not None else (None, None)
        name = self.book.choose([c.name for c in self.cams], "ml", job_of(why), x, y)
        return [c for c in self.cams if c.name == name] + [c for c in self.cams if c.name != name]

    def _run(self, region, now, why=None):
        t0 = time.perf_counter()
        out, pts = [], {}
        first = True
        self.n_run = getattr(self, "n_run", 0) + 1
        cams = self.order(region, why)
        ref_first = cams[0] is self.cams[0]
        for cam in cams:
            calib = False
            if not first:
                # the side camera only where the overhead found nothing near the robot asked for, position only;
                # (e19, item 12) and every SIDE_CAL_EVERY-th call everywhere, to MEASURE its offset from the overhead
                found = region is not None and any(math.hypot(d.x - region[0], d.y - region[1]) <= region[2]
                                                   for d in out)
                calib = SIDE_LEARN and not self.side_off and bool(out) and self.n_run % SIDE_CAL_EVERY == 0 and \
                    ref_first
                if self.side_off or region is None or (found and not calib):
                    continue
            got = self._detect(cam)
            if got is not None:
                self.calls_by_cam[cam.name] = self.calls_by_cam.get(cam.name, 0) + 1
                dets = self._to_dets(cam, got[0], got[1])
                if cam is not self.cams[0]:
                    if not first and calib:
                        self._side_learn(out, dets)
                        if found:
                            pts[cam.name] = got[0]
                            continue
                    off = self.side_offset()
                    if off is not None:
                        import tracking_layer as TL
                        dets = [TL.Det(d.src, d.t, d.x - off[0], d.y - off[1], d.th, d.name, d.sigma, score=d.score,
                                       cam=d.cam) for d in dets]
                if not first:
                    dets = [d for d in dets if region is None or math.hypot(d.x - region[0], d.y - region[1]) <= region[2]]
                    self.counts["side_used"] += 1 if dets else 0
                out += dets
                pts[cam.name] = got[0]
            first = False
        ms = 1000.0 * (time.perf_counter() - t0)
        self.counts["ran"] += 1
        self.counts["dets"] += len(out)
        self.budget.done(now, ms)
        if self.budget.over_run >= OVER_N and not self.side_off and len(self.cams) > 1:
            self.side_off = True
            self.counts["side_dropped"] += 1
            self.say("tag-free: the detector took %.0f ms a call %d times in a row (budget %.0f ms): side camera dropped, "
                     "overhead only" % (ms, OVER_N, self.budget.budget_ms))
        return now, tuple(sorted(pts.items())), out

    def _side_learn(self, over, side):
        """(e19, item 12) Pairs of the same robot seen by both cameras in one call: the side camera's offset from the
        overhead (each side detection with its nearest overhead one within SIDE_PAIR_CM)."""
        prs = self.__dict__.setdefault("side_pairs", [])
        for d in side:
            o = min(over, key=lambda q: math.hypot(q.x - d.x, q.y - d.y)) if over else None
            if o is not None and math.hypot(o.x - d.x, o.y - d.y) <= SIDE_PAIR_CM:
                prs.append((d.x - o.x, d.y - o.y))
        del prs[:-400]

    def side_offset(self):
        """(e19) The side camera's measured offset from the overhead (median of the pairs) once SIDE_CAL_N exist, else
        None (the assumed floor height alone, Z_SIDE_CM)."""
        prs = self.__dict__.get("side_pairs") or []
        if not SIDE_LEARN or len(prs) < SIDE_CAL_N:
            return None
        xs, ys = sorted(q[0] for q in prs), sorted(q[1] for q in prs)
        return (xs[len(xs) // 2], ys[len(ys) // 2])

    def summary(self):
        off = self.side_offset()
        return dict(version=VERSION, counts=dict(self.counts), load=self.budget.summary(), side_off=self.side_off,
                    calls_by_cam=dict(self.calls_by_cam),
                    sync=self.sync, core=self.core, side_pairs=len(self.__dict__.get("side_pairs") or []),
                    side_offset_cm=[round(v, 2) for v in off] if off else None)


# ==================================================================================================================
class HeadingFuser:
    """Each robot's heading: own (gyro) heading - offset, the offset estimated from the overhead detector's headings."""

    def __init__(self, sigma0=SIGMA_DET0_DEG):
        self.sigma0 = sigma0
        self.S = {}           # robot -> dict(off, P, own_t, own_th, out=[(t)], doubt)
        self.res = []         # residuals (deg) of accepted updates, all robots: the detector's own noise
        self.counts = dict(updates=0, outliers=0, doubts=0, resets=0, distrusted=0)
        self.doubted = {}     # robot -> doubts so far this run
        self.distrusted = {}  # robot -> t: its detector heading is no longer used (a check settled it against it)

    def sigma_det(self):
        if len(self.res) < LEARN_N:
            return self.sigma0
        a = sorted(abs(r) for r in self.res[-200:])
        return max(SIGMA_DET_MIN, 1.4826 * a[len(a) // 2])     # robust (MAD): an outlier does not widen it

    def reset(self, name, why="reset"):
        self.S.pop(name, None)
        self.counts["resets"] += 1

    def _state(self, name, t, own_th):
        s = self.S.get(name)
        if s is None:
            s = self.S[name] = dict(off=0.0, P=P0_DEG ** 2, own_t=t, own_th=own_th, out=[], doubt=None, n=0)
            return s
        dt = max(0.0, t - s["own_t"])
        dth = abs(wrap180(own_th - s["own_th"]))
        if dt > 0 and dth / max(dt, 1e-3) > OWN_JUMP_DPS and dth > 10.0:
            s["P"] = P0_DEG ** 2              # its own heading was re-set: the offset starts again (not its centre)
        s["P"] += Q_DEG_PER_S ** 2 * dt + (Q_TURN * dth) ** 2
        s["own_t"], s["own_th"] = t, own_th
        return s

    def update(self, name, t, own_th, det_th):
        """One overhead detector heading for `name` with its own heading own_th (deg) at t. Returns (heading or None,
        sigma deg, verdict "ok" | "outlier" | "doubt" | "no own" | "distrusted")."""
        if name in self.distrusted:
            return None, None, "distrusted"
        if own_th is None:
            return det_th, self.sigma_det(), "no own"
        s = self._state(name, t, own_th)
        R = self.sigma_det() ** 2
        z = wrap180(own_th - det_th)                 # the offset this detection says
        innov = wrap180(z - s["off"])
        if abs(innov) > DOUBT_K * math.sqrt(s["P"] + R):
            s["out"] = [q for q in s["out"] if t - q < DOUBT_WIN_S] + [t]
            self.counts["outliers"] += 1
            if len(s["out"]) >= DOUBT_N and s["doubt"] is None:
                s["doubt"] = t
                self.counts["doubts"] += 1
                self.doubted[name] = self.doubted.get(name, 0) + 1
                s["new_doubt"] = True
                if self.doubted[name] >= 2:
                    # the heading check (the camera-measured crawl) settled it once and the detector still disagrees:
                    # the detector's heading for THIS robot is not trusted for the rest of the run (evidence wins)
                    self.distrusted[name] = t
                    self.counts["distrusted"] += 1
                return None, None, "doubt"
            return (None, None, "doubt") if s["doubt"] is not None else \
                (wrap180(own_th - s["off"]), math.sqrt(s["P"]), "outlier")
        s["out"] = []
        K = s["P"] / (s["P"] + R)
        s["off"] = wrap180(s["off"] + K * innov)
        s["P"] = (1 - K) * s["P"]
        s["n"] += 1
        if s["n"] > 1:
            self.res.append(innov)
            del self.res[:-400]
        self.counts["updates"] += 1
        if s["doubt"] is not None:
            return None, None, "doubt"
        return wrap180(own_th - s["off"]), math.sqrt(s["P"]), "ok"

    def significant(self, name):
        """The offset is real (over SIGNIF_K x its sigma): the own heading is off and the fused one should be used."""
        s = self.S.get(name)
        return s is not None and s["doubt"] is None and abs(s["off"]) > SIGNIF_K * math.sqrt(s["P"])

    def det95(self):
        """(e19) The detector's heading error at 95 % (deg): the report's, or the site's own once learnt (never less)."""
        if len(self.res) < LEARN_N:
            return DET95_DEG
        return max(DET95_DEG, 1.96 * self.sigma_det())

    def advice(self, name, own95=None, clear_cm=None):
        """What to do with this robot's heading now: None (keep its own), "hand" (hand the fused heading on: its own is
        measurably off), "check" (ask the camera-measured heading check instead). (e19, "risk") own95: the robot's own
        heading doubt at 95 % (deg; the controller's); clear_cm: its room to an edge or a neighbour less its position
        doubt (None: open ground)."""
        s = self.S.get(name)
        if HAND_RULE == "e18":
            if s is None or s["doubt"] is not None or s["n"] < HAND_MIN_N or not self.significant(name):
                return None
            return "check" if abs(s["off"]) > HAND_MAX_DEG else "hand"
        if s is None or s["doubt"] is not None or s["n"] < HAND_MIN_N:
            return None
        o95 = 4.0 if own95 is None else float(own95)
        bound = math.hypot(self.det95(), o95)          # what noise and its own doubt explain together
        off = abs(s["off"])
        if off <= bound:
            self.counts["kept_within_noise"] = self.counts.get("kept_within_noise", 0) + 1
            return None
        # the risk limit (Nick: from risk, not 45): if the detector were the wrong one, the hand-on would turn the robot
        # by `off`; over HAND_HORIZON_CM that is a sideways error of HORIZON x sin(off): it must stay inside its clearance
        if clear_cm is None:
            lim = 90.0
        else:
            lim = math.degrees(math.asin(max(0.0, min(1.0, float(clear_cm) / HAND_HORIZON_CM))))
        self.last_limit = dict(name=name, off=round(off, 1), bound=round(bound, 1), limit=round(lim, 1))
        return "hand" if off <= lim else "check"

    def take_new_doubt(self, name):
        s = self.S.get(name)
        if s is not None and s.pop("new_doubt", False):
            return True
        return False

    def heading(self, name, own_th):
        s = self.S.get(name)
        if s is None or own_th is None or s["doubt"] is not None:
            return None, None
        return wrap180(own_th - s["off"]), math.sqrt(s["P"])

    def summary(self):
        return dict(counts=dict(self.counts), sigma_det_deg=round(self.sigma_det(), 1), residuals=len(self.res),
                    robots={n: dict(offset_deg=round(s["off"], 1), sigma_deg=round(math.sqrt(s["P"]), 1),
                                    in_doubt=s["doubt"] is not None) for n, s in self.S.items()},
                    distrusted=sorted(self.distrusted))


# ==================================================================================================================
class TagFree:
    """What attach() leaves on the layer (layer.tagfree): the source, the fuser, the switch's decision and its words."""

    def __init__(self, layer, source, fuser, ctrl, mode, model_path, words):
        self.layer, self.source, self.fuser, self.ctrl = layer, source, fuser, ctrl
        self.mode, self.model_path, self.words = mode, model_path, words
        self.asked = set()            # robots we asked the detector for (heading in doubt)
        self.hd_seen = {}             # robot -> its hd_confirm dict last feed (to see the check end)
        self.last_ml = []
        self.events = []
        self.handed = 0               # fused headings handed on (the own heading measurably off)
        self.big_asked = set()        # robots a far-off heading asked the heading check for (until it ends)

    def before(self, t):
        c = self.ctrl
        if c is None:
            return
        for n, r in getattr(c, "R", {}).items():
            hc = r.__dict__.get("hd_confirm")
            prev = self.hd_seen.get(n)
            if hc is not None:
                self.layer.want_detector(n, "heading in doubt: %s" % hc.get("why", "?"))
                self.asked.add(n)
            elif n in self.asked:
                self.layer.want_detector(n, None)
                self.asked.discard(n)
            if prev is not None and hc is None:
                self.fuser.reset(n, "heading check ended")      # the controller confirmed or set it: trust it again
                self.big_asked.discard(n)
            self.hd_seen[n] = hc

    def after(self, t, fixes, ml_dets):
        layer, c = self.layer, self.ctrl
        for f in fixes:
            n = f["robot"]
            if f["src"] == "tags":
                if self.source is not None:
                    self.source.tag_seen[n] = t
                continue
            if f["src"] != "ml" and f.get("th") is not None:
                continue                          # a heading of its own (the bridge's travel heading): left as it is
            near = [d for d in ml_dets if d.th is not None and math.hypot(d.x - f["x"], d.y - f["y"]) < MATCH_CM]
            near = [d for d in near if self._heading_usable(n, d, t)]       # (e20) only cameras trusted for heading
            if not near:
                if f["src"] == "ml":
                    f["th"] = None                # a side-camera sighting: position only
                continue
            d = min(near, key=lambda q: math.hypot(q.x - f["x"], q.y - f["y"]))
            tr = layer.T.get(n)
            own_th = tr.own[-1][3] if tr is not None and tr.own and tr.own[-1][3] is not None else None
            if own_th is not None and len(tr.own) >= 2 and tr.own[-1][0] > tr.own[-2][0]:
                rate = abs(wrap180(tr.own[-1][3] - tr.own[-2][3])) / (tr.own[-1][0] - tr.own[-2][0])
                if rate > TURN_MAX_DPS:
                    self.fuser.counts["skipped_turning"] = self.fuser.counts.get("skipped_turning", 0) + 1
                    if f["src"] == "ml":
                        f["th"] = None
                    continue
            th, sig, verdict = self.fuser.update(n, t, own_th, d.th)
            f["th_src"] = "detector+own" if own_th is not None else "detector"
            adv = self.fuser.advice(n, *self._risk(n))
            if adv == "check":
                f["th"] = None
                if n not in self.big_asked:
                    self.big_asked.add(n)
                    self.fuser.counts["checks_asked"] = self.fuser.counts.get("checks_asked", 0) + 1
                    self._doubt(n, t, own_th, d.th, "the overhead detector puts its heading %.0f deg from its own%s"
                                % (abs(self.fuser.S[n]["off"]), "" if HAND_RULE == "e18" else
                                   ": more than noise explains and more than its room allows (a check, not a hand-on)"))
                continue
            if verdict == "no own" or adv == "hand":
                f["th"] = th                      # the own heading is measurably off (or there is none): corrected
                if sig is not None:
                    f["sigma_th"] = round(max(sig, 1.0), 2)
                if verdict != "no own":
                    # the robot is handed an ABSOLUTE heading; once it takes it, its own heading jumps by the offset, so
                    # the offset is learnt again from zero (a fix it refused shows the same offset again: handed again)
                    self.fuser.reset(n, "heading handed on")
                    self.handed += 1
            else:
                f["th"] = None                    # agrees within its noise: the robot's own heading is kept
            if self.fuser.take_new_doubt(n):
                self._doubt(n, t, own_th, d.th)

    def _heading_usable(self, n, d, t):
        """(e20) The detector's heading of this camera: checked against the robot's own gyro heading while that is sure
        (evidence for the camera's heading trust: camera_trust), and used only when the camera is TRUSTED for heading at
        that square. e19 / no book / a detection with no camera: used as before (the camera's heading flag)."""
        book = getattr(self.source, "book", None)
        if ROUTE == "e19" or book is None or d.cam is None:
            return True
        c = self.ctrl
        r = getattr(c, "R", {}).get(n) if c is not None else None
        if r is not None and r.own is not None and r.__dict__.get("hd_confirm") is None and \
                n not in self.big_asked:
            tr = self.layer.T.get(n)
            turning = False
            if tr is not None and len(tr.own) >= 2 and tr.own[-1][0] > tr.own[-2][0] and tr.own[-1][3] is not None \
                    and tr.own[-2][3] is not None:
                turning = abs(wrap180(tr.own[-1][3] - tr.own[-2][3])) / (tr.own[-1][0] - tr.own[-2][0]) > TURN_MAX_DPS
            own_th = tr.own[-1][3] if tr is not None and tr.own else None
            try:
                hsd = float(c.head_sd(r))
                import freeroam_core as FC
                sure = hsd <= FC.HD_CONFIRM_DEG       # its own heading judges a camera only while it is sure
            except Exception:  # noqa: BLE001
                hsd, sure = 2.0, True
            if not turning and own_th is not None and sure:
                import camera_trust as CT
                ok = abs(wrap180(d.th - own_th)) <= CT.HEAD_K * math.hypot(SIGMA_DET0_DEG, hsd)
                book.observe(d.cam, "ml", "heading", d.x, d.y, ok, 1.0, d.light, t, "its own gyro heading")
        return book.state(d.cam, "ml", "heading", d.x, d.y, d.light) == "trusted"

    def _risk(self, n):
        """(e19) (own heading doubt at 95 %, room to an edge or a neighbour less its position doubt) from the
        controller, or (None, None) without one."""
        c = self.ctrl
        try:
            r = c.R[n]
            import freeroam_core as FC
            own95 = 2.0 * c.head_sd(r)
            p = c.pos(r)
            if p is None:
                return own95, 0.0
            clear = FC.edge_dist(c.rect, *p) - FC.HALF
            for o in c.R.values():
                q = c.pos(o) if o is not r and o.mode != "gone" else None
                if q is not None:
                    clear = min(clear, math.dist(p, q) - 2 * FC.HALF)
            return own95, max(0.0, clear - c.unc(r))
        except Exception:  # noqa: BLE001
            return None, None

    def _doubt(self, n, t, own_th, det_th, why=None):
        why = why or "the overhead detector disagrees with its own heading twice (own %s, detector %.0f deg)" % (
            "?" if own_th is None else "%.0f" % own_th, det_th)
        self.events.append(dict(t=round(t, 2), robot=n, why=why))
        c = self.ctrl
        if c is None or n not in getattr(c, "R", {}):
            return
        r = c.R[n]
        try:
            import freeroam_core as FC
            on = FC.HEADING_CONFIRM
        except Exception:  # noqa: BLE001
            on = True
        self.layer.log("tracking", dict(t=round(t, 2), robot=n, what="heading in doubt (tag-free)", why=why))
        if on and r.__dict__.get("hd_confirm") is None and getattr(r, "mode", None) not in ("out", "gone"):
            r.hd_confirm = dict(t=getattr(c, "t", t), why="tag-free: " + why, tries=0, stage=None)
            try:
                c.event(r, "heading in doubt", why)
            except Exception:  # noqa: BLE001
                pass

    def summary(self):
        return dict(version=VERSION, mode=self.mode, model=self.model_path, words=self.words,
                    source=self.source.summary() if hasattr(self.source, "summary") else None,
                    fuser=self.fuser.summary(), headings_handed_on=self.handed, doubts=list(self.events[-20:]))


def attach(layer, source=None, ctrl=None, mode=None, model_path=None, say=None, need_model=True):
    """Hooks tag-free mode into a tracking layer (the run's or the simulator's). source: the layer's ml_fn (a
    DetectorSource, a SimDetector) or None when there is no model. mode: auto / 1 / 0 (default MIRLT_TAGFREE).
    Returns the TagFree state (also layer.tagfree) or None when off. A missing model is said, never an error."""
    say = say or (lambda s: None)
    mode = mode or switch()
    if mode == "0":
        say("Tag-free mode OFF (MIRLT_TAGFREE=0): the tracking layer as before.")
        return None
    if source is None:
        say("Tag-free mode: no detector model (looked at MIRLT_DETECTOR_MODEL, MIRLT_ML_MODEL, %s): the layer runs "
            "without it (motion, lights, elimination, wiggles as before)." % ", ".join(MODEL_DEFAULTS))
        return None
    if hasattr(source, "auto"):
        source.auto = mode == "auto"
    fuser = HeadingFuser()
    st = TagFree(layer, source, fuser, ctrl, mode, model_path,
                 "Tag-free mode ON (%s): the detector %s, on demand (lost robot, unclear name, heading in doubt), at "
                 "most %g a second a robot and %g in all, %s; overhead heading fused with each robot's own"
                 % ("auto" if mode == "auto" else "forced", model_path or "(simulated)", _tl().ML_ROBOT_HZ,
                    _tl().ML_TOTAL_HZ, "only while no roof tag names the robot" if mode == "auto" else "for every robot"))
    layer.ml_fn = source
    layer.ml_on = True
    if getattr(layer, "book", None) is not None and hasattr(source, "book") and source.book is None:
        source.book = layer.book                  # (e20) routing and heading trust from the run's camera trust
    orig = layer.feed
    orig_ml = layer._ml_on_demand

    def ml_on_demand(t, preds):
        got = orig_ml(t, preds)
        st.last_ml = list(got)
        return got

    def feed(t, dets):
        st.last_ml = []
        st.before(t)
        fixes = orig(t, dets)
        st.after(t, fixes, st.last_ml)
        return fixes

    layer._ml_on_demand = ml_on_demand
    layer.feed = feed
    layer.tagfree = st
    say(st.words)
    return st


def _tl():
    import tracking_layer as TL
    return TL


# ==================================================================================================================
# the Pi's run (free roam): the cameras from the recorder's newest pictures, the floor from the bridge's calibration
def for_runner(runner, say=None, env=None):
    """The DetectorSource for freeroam.Runner (its frames: each camera's newest JPEG; its run args: the calibrations).
    None (said) when the switch is off, the model is missing, onnxruntime is missing or there are no pictures."""
    env = os.environ if env is None else env
    say = say or print
    if switch(env) == "0":
        return None, None
    path = find_model()
    if path is None:
        return None, None
    frames = getattr(runner, "frames", None)
    a = getattr(runner, "a", None)
    L = getattr(a, "_ladder", None)
    calib = getattr(L, "calib", None) or getattr(a, "calib", None)
    if frames is None or not calib:
        say("Tag-free mode: model %s found, but no camera pictures in the planner (frames not saved / no calibration): "
            "the layer runs without it." % path)
        return None, path
    try:
        import sys
        here = os.path.dirname(os.path.abspath(__file__))
        tools = os.path.dirname(here)
        for p in (os.path.join(tools, "detector"), os.path.join(tools, "experiment")):
            if p not in sys.path:
                sys.path.append(p)
        import infer as INF
        import e02_camera as CAM
        model = INF.load(path, threads=1)
        cams = []
        by = {c.cam: c for c in getattr(frames, "cams", [])}

        def pic(name):
            return lambda: getattr(by.get(name), "latest", None)
        oc = CAM.load_camera(calib)
        cams.append(Camera("overhead", pic("overhead"), _run_frame(runner, camera_model_to_floor(oc, Z_OVERHEAD_CM)),
                           True, SIGMA_OVERHEAD_CM))
        side_calib = getattr(L, "side_calib", None)
        if side_calib and "side" in by:
            sc = CAM.load_camera(side_calib)
            cams.append(Camera("side", pic("side"), _run_frame(runner, camera_model_to_floor(sc, Z_SIDE_CM)), False,
                               SIGMA_SIDE_CM))
    except Exception as e:  # noqa: BLE001  (onnxruntime missing, a bad file: the run goes on without it)
        say("Tag-free mode: the detector could not start (%s): the layer runs without it." % e)
        return None, path
    core = None
    if (env.get("MIRLT_DETECTOR_CORE") or "planner").strip() == "recorder":
        try:
            import core_layout as CL
            core = CL.cores("recorder1") or None
        except Exception:  # noqa: BLE001
            core = None
    return DetectorSource(model, cams, sync=False, say=say, core=core), path


def _run_frame(runner, f):
    """The camera's floor position through the calibration the start's discovery chose (runner.cam_fix), as the
    bridge's fixes are (freeroam.Runner.fixes)."""
    def g(u, v):
        x, y = f(u, v)
        cf = getattr(runner, "cam_fix", None)
        return cf(x, y) if cf is not None else (x, y)
    return g


# ==================================================================================================================
# the fast simulator (freeroam_fastsim.Sim(camera="detections", layer_kw=dict(ml_fn=SimDetector(...))))
SIM_P = dict(moving=0.98, standing=0.96, edge=0.5, lamp=0.80)      # MEASURED (report) except edge / lamp: CHOSEN
SIM_TH_SIGMA = 15.0      # as SIGMA_DET0_DEG
SIM_XY_SIGMA = 1.6       # as SIGMA_OVERHEAD_CM
SIM_HALF_CM = 4.75       # half a robot: the middle this near an edge = half over it


class SimDetector:
    """detector-v1 in the fast simulator. scene keys (the simulator breaks the guesses):
       ml_heading_bias {robot: deg}  the detector's heading for that robot is this far off (a roof it never learnt)
       lamp [rects]                  washed-out lamp patches: recall SIM_P["lamp"]  (the scene's "glare" counts too)
       side_only [rects]             only the side camera sees there: position only, no heading
       ml_ms                         the detector's time a call (ms; default 25, the Pi 4 estimate)
       ml_false                      false detections a picture (default 0: the report found none)
    Robots whose middle is within half a robot of an edge (parked half over it) are found with SIM_P["edge"]."""

    def __init__(self, scene=None, seed=1, budget=None):
        self.scene = dict(scene or {})
        self.rng = random.Random(seed * 4241 + 7)
        self.sim = None
        self.budget = budget or Budget()
        self.counts = dict(asked=0, ran=0, dets=0, missed_edge=0, missed_lamp=0, refused_load=0, skipped_tag=0)
        self.tag_seen = {}
        self.auto = True
        self.last_t = None
        self.book = None                      # (e20) set by attach() from the layer's camera trust
        self.calls_by_cam = {}

    def _in(self, key, x, y):
        return any(r[0] <= x <= r[2] and r[1] <= y <= r[3] for r in self.scene.get(key) or ())

    def _call_cams(self, name, t, why, region):
        """(e20) ml_cams=[dict(name, sigma, th_sigma, th_any (heading not learnt: anywhere), th_bias, recall, cover
        rect, dark [rects] (recall x 0.3), bump (t0, deg): its picture turned about the board's middle from t0)] and
        ml_swap (t0, t1, a, b): two cameras' names swapped (each picture through the other's calibration: mirrored).
        One call runs the camera routed for this question (the book's choice; else the first), the next one only where
        it found nothing near the robot. Every detection says its camera."""
        import tracking_layer as TL
        s = self.sim
        cams = list(self.scene["ml_cams"])
        book = getattr(self, "book", None)
        if book is not None and ROUTE != "e19" and len(cams) > 1:
            x, y = (region[0], region[1]) if region is not None else (None, None)
            first = book.choose([c["name"] for c in cams], "ml", job_of(why), x, y)
            cams = [c for c in cams if c["name"] == first] + [c for c in cams if c["name"] != first]
        B = s.board
        cx, cy = 0.5 * (B[0] + B[2]), 0.5 * (B[1] + B[3])
        sw = self.scene.get("ml_swap")
        out = []
        for k, c in enumerate(cams):
            if k > 0 and region is not None and any(math.hypot(d.x - region[0], d.y - region[1]) <= region[2]
                                                     for d in out):
                break
            if k > 0 and region is None:
                break
            self.calls_by_cam[c["name"]] = self.calls_by_cam.get(c["name"], 0) + 1
            for b in s.bots.values():
                if b.dead or (hasattr(s, "in_cover") and not s.in_cover(b.x, b.y)):
                    continue
                cv = c.get("cover")
                if cv and not (cv[0] <= b.x <= cv[2] and cv[1] <= b.y <= cv[3]):
                    continue
                p = float(c.get("recall", SIM_P["standing"]))
                dark = any(r[0] <= b.x <= r[2] and r[1] <= b.y <= r[3] for r in c.get("dark") or ())
                if dark:
                    p *= 0.3
                if self.rng.random() >= p:
                    continue
                sg = float(c.get("sigma", SIM_XY_SIGMA))
                x, y = b.x + self.rng.gauss(0, sg), b.y + self.rng.gauss(0, sg)
                if c.get("th_any"):
                    th = self.rng.uniform(-180.0, 180.0)
                else:
                    th = wrap180(b.th + self.rng.gauss(0.0, float(c.get("th_sigma", SIM_TH_SIGMA))) +
                                 float(c.get("th_bias", 0.0)))
                bm = c.get("bump")
                label = c["name"]
                if bm and t >= bm[0]:
                    a_ = math.radians(bm[1])
                    x, y = (cx + (x - cx) * math.cos(a_) - (y - cy) * math.sin(a_),
                            cy + (x - cx) * math.sin(a_) + (y - cy) * math.cos(a_))
                    th = wrap180(th + bm[1])
                if sw and sw[0] <= t < sw[1] and label in (sw[2], sw[3]):
                    if label == sw[2]:                 # through the other's calibration: mirrored left-right ...
                        x, th = B[0] + B[2] - x, wrap180(180.0 - th)
                    else:                              # ... and the other one top-bottom
                        y, th = B[1] + B[3] - y, wrap180(-th)
                    label = sw[3] if label == sw[2] else sw[2]
                out.append(TL.Det("ml", t, x, y, th, None, sg, cam=label, light="dark" if dark else None))
        if getattr(s, "cam_fault", None) or getattr(s, "cam_correct", None) is not None:
            for d in out:
                d.x, d.y = s.cam_map(d.x, d.y)
        self.counts["dets"] += len(out)
        return out

    def __call__(self, name, t, why, region):
        import tracking_layer as TL
        self.counts["asked"] += 1
        ts = self.tag_seen.get(name)
        if self.auto and ts is not None and t - ts < TAG_FRESH_S:
            self.counts["skipped_tag"] += 1
            return []
        s = self.sim
        if s is None:
            return []
        if self.last_t == t:
            return []                         # one picture serves every robot asking in it (already returned)
        if not self.budget.allow(t):
            self.counts["refused_load"] += 1
            return []
        self.last_t = t
        self.counts["ran"] += 1
        self.budget.done(t, float(self.scene.get("ml_ms", 25.0)))
        if self.scene.get("ml_cams"):
            return self._call_cams(name, t, why, region)          # (e20) several cameras, routed
        out = []
        B = s.board
        for b in s.bots.values():
            if b.dead:
                continue
            if hasattr(s, "in_cover") and not s.in_cover(b.x, b.y):
                continue
            moving = getattr(b, "v", 0.0) > 0.5 or getattr(b, "state", "idle") not in ("idle",)
            p = SIM_P["moving"] if moving else SIM_P["standing"]
            if min(b.x - B[0], B[2] - b.x, b.y - B[1], B[3] - b.y) < SIM_HALF_CM:
                p = min(p, SIM_P["edge"])
                if self.rng.random() >= p:
                    self.counts["missed_edge"] += 1
                    continue
            if self._in("lamp", b.x, b.y) or self._in("glare", b.x, b.y):
                if self.rng.random() >= SIM_P["lamp"]:
                    self.counts["missed_lamp"] += 1
                    continue
            elif self.rng.random() >= p:
                continue
            side = self._in("side_only", b.x, b.y)
            sg = SIGMA_SIDE_CM if side else SIM_XY_SIGMA
            th = None
            if not side:
                th = wrap180(b.th + self.rng.gauss(0.0, SIM_TH_SIGMA) +
                             float((self.scene.get("ml_heading_bias") or {}).get(b.name, 0.0)))
            d = TL.Det("ml", t, b.x + self.rng.gauss(0, sg), b.y + self.rng.gauss(0, sg), th, None, sg)
            out.append(d)
        for _ in range(int(self.scene.get("ml_false", 0))):
            out.append(TL.Det("ml", t, self.rng.uniform(B[0], B[2]), self.rng.uniform(B[1], B[3]), None, None,
                              SIGMA_OVERHEAD_CM))
        if getattr(s, "cam_fault", None) or getattr(s, "cam_correct", None) is not None:
            for d in out:
                d.x, d.y = s.cam_map(d.x, d.y)
        self.counts["dets"] += len(out)
        return out

    def summary(self):
        return dict(counts=dict(self.counts), load=self.budget.summary(), calls_by_cam=dict(self.calls_by_cam))
