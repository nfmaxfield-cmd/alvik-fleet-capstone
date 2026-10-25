"""The tracker the tracker process runs (2026-10-25 evening, MIRLT; Nick approved 19:03): the same tracker as
e02_camera.MotionTracker (floor memory, detector, lock-on), plus two things.

1. Every robot in every picture, not only the followed one (15:45 session: a put-aside robot nobody watched blocked
   routes, a collision went unseen, parked positions went stale). The detector already outlines everything that
   differs from the floor in each picture; each parked robot takes the robot-shaped outline nearest its parked spot,
   within PARK_GATE_CM. Per parked robot this gives:
     seen     its outline is there, within PARK_MOVED_CM of the spot;
     moved    its outline stood PARK_MOVED_CM or more from the spot for PARK_MOVED_N pictures in a row: the spot moves
              to it (the floor memory's "robot here" mask moves too, so the old spot is learned as floor and no ghost
              is left), and the move is counted (a bump, a push, a hand);
     unseen   no outline near the spot for PARK_UNSEEN_S (picked up, pushed further than the gate, or merged with the
              followed robot or a board's edge in the picture);
     crowded  the followed robot is within PARK_CROWD_CM: the two outlines can't be told apart, nothing is changed.
   The followed robot is handled exactly as before.

2. Faster, same answers:
   - FastLockTracker: the lock-on's template matching (TM_CCOEFF_NORMED) for all the turns it tries at once, as one
     matrix product, instead of one OpenCV call per turn. It was 60 % of the tracker's time (27 of 43 ms a picture on
     the 15:45 footage, this machine): OpenCV picks its Fourier-transform path for these sizes, which costs about 10x
     a direct product here. Same formula, OpenCV's own rules for a flat window, same order and tie-breaking; checked
     against OpenCV on real footage (tests/test_e02_tracker_process.py).
   - _learn with whole-array selects (np.where / np.copyto) instead of boolean indexing: the same float32 arithmetic,
     bit for bit.
   Numpy's matrix product and OpenCV's filters use more than one core and let go of Python's lock, so in the tracker
   process (2 cores) they run beside the picture decoding.
"""
import math

import cv2
import numpy as np

import e02_camera as CAM
import e02_lock as LK

# Chosen numbers. DOE notes: docs/doe-backlog.md rows "Parked-robot watch".
PARK_GATE_CM = 8.0       # chosen: an outline this near a parked spot is that robot. Below OTHER_CM (10.45) so two
                         # robots parked side by side don't share; above a bump's typical shove (2-5 cm). DOE: replay
                         # footage with known pushes (the 15:45 collision) over 6/8/10 cm, count missed moves/false ones
PARK_MOVED_CM = 2.0      # chosen: a parked outline this far from its spot is a move (tracker jitter on a standing
                         # robot: 0.3-0.6 cm; the outline's centre sways ~1 cm by a board's edge). DOE with the gate
PARK_MOVED_N = 12        # chosen: ... for this many pictures in a row (~0.5 s at 25/s; the floor memory's own
                         # STABLE_FRAMES). DOE with the gate
PARK_UNSEEN_S = 3.0      # chosen: no outline near the spot this long: unseen. DOE: the same replay, false "unseen"
                         # per hour by a board's edge against the time a picked-up robot goes unnoticed
PARK_CROWD_CM = 14.0     # chosen: the followed robot this near a parked spot: the parked one is not judged (two
                         # touching robots are 9.5-13 cm apart centre to centre). No DOE: geometry
PARK_MEAN_N = 8          # a moved robot's new spot: the mean of its latest this-many sightings. No DOE: smoothing only
PARK_STILL_CM = 1.0      # chosen: a move counts once the outline stands still again: its last PARK_MOVED_N places within
                         # this of each other (the lock-on's own RELOCK_STILL_CM). DOE with the gate


# =============================================================================== faster lock-on
_FFT_CACHE = {}


def _tpl_fft(tp, shape):
    """The template's mean-removed spectrum at this size, and its norm (cached: a look's turned copies only change
    when it is refreshed)."""
    key = (id(tp), shape)
    hit = _FFT_CACHE.get(key)
    if hit is not None and hit[0] is tp:
        return hit[1], hit[2]
    t = np.asarray(tp, np.float64)
    tc = t - t.mean()
    f = np.conj(np.fft.rfft2(tc, s=shape))
    nrm = float(np.sqrt((tc * tc).sum()))
    if len(_FFT_CACHE) > 400:
        _FFT_CACHE.clear()
    _FFT_CACHE[key] = (tp, f, nrm)
    return f, nrm


def ccoeff_normed_batch(win, tpls):
    """cv2.matchTemplate(win, t, cv2.TM_CCOEFF_NORMED) for every template t in tpls (all the same size), as one
    array (len(tpls), H, W). The same formula as OpenCV's, by Fourier transform with the window transformed once and
    each template's transform kept, and OpenCV's rule for a window with no contrast (0, or +-1 just over the line),
    so nan_to_num afterwards gives what the old code gave."""
    win = np.asarray(win, np.float64)
    n = tpls[0].shape[0]
    H, W = win.shape[0] - n + 1, win.shape[1] - n + 1
    if H <= 0 or W <= 0:
        return np.zeros((len(tpls), max(H, 0), max(W, 0)), np.float32)
    shape = win.shape
    fw = np.fft.rfft2(win)
    K = n * n
    ii, ii2 = cv2.integral2(win, sdepth=cv2.CV_64F, sqdepth=cv2.CV_64F)

    def box(a):
        return a[n:n + H, n:n + W] - a[0:H, n:n + W] - a[n:n + H, 0:W] + a[0:H, 0:W]
    ws, ws2 = box(ii), box(ii2)
    wn = np.sqrt(np.maximum(ws2 - ws * ws / K, 0.0))                     # sqrt(sum((I - mean I)^2))
    out = np.empty((len(tpls), H, W), np.float32)
    for j, tp in enumerate(tpls):
        ft, tn = _tpl_fft(tp, shape)
        num = np.fft.irfft2(fw * ft, s=shape)[:H, :W]                   # sum(I * T'), T' = T - mean T
        t = wn * tn
        r = np.zeros((H, W))
        big = np.abs(num) < t
        r[big] = num[big] / t[big]
        edge = (~big) & (np.abs(num) < t * 1.125)
        r[edge] = np.sign(num[edge])
        out[j] = r
    return out


class FastLockTracker(LK.LockTracker):
    """e02_lock.LockTracker with every turn's match in one matrix product (ccoeff_normed_batch). Same order, same
    early stop (the NEAR_TURNS nearest the last turn first, the rest only if none of them is MATCH_OK), same ties
    (the first best turn, then OpenCV's first best spot in row order)."""

    def _best(self, feat, u, v, half_extra, tpls, mask, near_angle=None):
        n = tpls[0][1].shape[0]
        half = n // 2 + half_extra
        win = self._patch(feat, u, v, half)
        if win is None:
            return None
        order = tpls if near_angle is None else sorted(
            tpls, key=lambda at: abs(((at[0] - near_angle) + 180) % 360 - 180))
        first = len(order) if near_angle is None else min(LK.NEAR_TURNS, len(order))
        best = None
        for lo, hi in ((0, first), (first, len(order))):
            if lo >= hi:
                continue
            if lo > 0 and best is not None and best[0] >= LK.MATCH_OK:
                break
            res = ccoeff_normed_batch(win, [tp for _, tp in order[lo:hi]])
            for j in range(hi - lo):
                i = lo + j
                if near_angle is not None and i >= LK.NEAR_TURNS and best is not None and best[0] >= LK.MATCH_OK:
                    break
                r = res[j]
                k = int(np.argmax(r))
                mx = float(r.flat[k])
                ly, lx = divmod(k, r.shape[1])
                if best is None or mx > best[0]:
                    best = (mx, int(round(u)) - half + lx + n // 2, int(round(v)) - half + ly + n // 2, order[i][0])
            else:
                continue
            break
        return best


# =============================================================================== every robot, every picture
class AllTracker(CAM.MotionTracker):
    """MotionTracker plus every parked robot watched in every picture (module docstring). Thread-safe like it."""

    WATCH_PARKED = True                  # False: the parked watch is off (the equivalence test; a fallback)
    MOVE_SPOT = True                     # False: a moved parked robot is reported, its spot not moved

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.parked = {}                 # robot -> dict(x, y, t_seen, status, off_cm, run, hist, moves, since)
        self.park_events = []            # (t, robot, kind, dict): moved / unseen / seen again (the process sends them)
        self._cands_n = -1               # picture number of last_cands

    # the lock-on: the faster matcher
    def _lock_step(self, img, t, det, held_now, hint, det_moving=False):
        if self.lk is None:
            self.lk = FastLockTracker(self.cam, self.rpx, self.z)
        return super()._lock_step(img, t, det, held_now, hint, det_moving)

    def candidates(self, g=None, motion=None):
        out = super().candidates(g, motion)
        if g is None or g is self.frame:
            self._cands_n = self.n
        return out

    def _learn(self, g, dt_frame):
        """MotionTracker._learn, the same arithmetic with whole-array selects (bit-identical; tested)."""
        cov = self._covered(g.shape)
        ok = (cov == 0) & (self.stable >= self.STABLE_FRAMES)
        gc = g - self.offset
        d = np.abs(gc - self.bg)
        new_floor = ok & (self.known == 0)
        np.copyto(self.bg, gc, where=new_floor)
        np.copyto(self.known, np.uint8(1), where=new_floor)
        kn = ok & (self.known > 0)                               # after the update, as the original
        same = kn & (d <= self.thr)
        if same.any():
            upd = self.bg + self.alpha * (gc - self.bg)          # float32, as bg[same] += alpha * (...)
            np.copyto(self.bg, upd.astype(np.float32, copy=False), where=same)
        changed = kn & (d > self.thr)
        # still_new[changed] += dt; still_new[~changed] = 0
        add = (self.still_new + dt_frame).astype(np.float32, copy=False)
        np.copyto(self.still_new, np.where(changed, add, np.float32(0.0)))
        absorb = self.still_new > self.ABSORB_S
        if absorb.any():
            np.copyto(self.bg, gc, where=absorb)
            self.still_new[absorb] = 0.0

    # --- the parked robots -------------------------------------------------------------------------------------
    def park(self, robot, x, y):
        with self.lock:
            self.protect[robot] = (x, y)
            p = self.parked.get(robot)
            if p is None or math.hypot(p["x"] - x, p["y"] - y) > 0.01:
                self.parked[robot] = dict(x=float(x), y=float(y), t_seen=None, status="settling", off_cm=None, run=0,
                                          hist=[], ref=None, moves=0 if p is None else p["moves"],
                                          since=self.frame_t, sx=None, sy=None)

    def forget(self, robot):
        super().forget(robot)
        with self.lock:
            self.parked.pop(robot, None)

    def clear_protect(self):
        super().clear_protect()
        with self.lock:
            self.parked.clear()

    def follow(self, robot, x, y, t=None):
        super().follow(robot, x, y, t)
        with self.lock:
            self.parked.pop(robot, None)        # followed now, not parked

    def push(self, img, t):
        out = super().push(img, t)
        try:
            with self.lock:
                self._watch_parked(t)
        except Exception:                       # the parked watch must never stop the followed robot's tracking
            pass
        return out

    def _watch_parked(self, t):
        """Each parked robot against its own outline (module docstring). A move is measured from the outline's
        place when it was parked (its reference: the mean of its first PARK_MEAN_N sightings), not from the spot the
        runner gave: on the 15:45 footage the parked outlines sat 3.4-4 cm from the spots the runner stored, the same
        on every picture (the outline's centre is not the robot's turning centre, glare, a board's edge), and that
        must not read as a move. A move shifts the spot by the outline's own shift."""
        if not self.WATCH_PARKED:
            return
        names = [r for r in self.parked if r != self.active and r in self.protect]
        if not names or self.bg is None or self.frame is None:
            return
        if self._cands_n != self.n:
            self.last_cands = self.candidates(self.frame)
            self._cands_n = self.n
        cands = [c for c in self.last_cands if c["area"] >= 0.3 * self.area]
        merged = self._merge(cands)
        act = (self.pos[1], self.pos[2]) if (self.active is not None and self.pos is not None) else None
        taken = set()
        for r in sorted(names):
            p = self.parked[r]
            cx, cy = (p["ref"] if p["ref"] is not None else (p["x"], p["y"]))
            if act is not None and min(math.hypot(act[0] - p["x"], act[1] - p["y"]),
                                       math.hypot(act[0] - cx, act[1] - cy)) < PARK_CROWD_CM:
                if p["status"] != "unseen":
                    p["status"] = "crowded"
                p["run"] = 0
                continue
            best = None
            for i, c in enumerate(merged):
                if i in taken:
                    continue
                if act is not None and math.hypot(c["x"] - act[0], c["y"] - act[1]) < CAM.MotionTracker.OTHER_CM:
                    continue                    # the followed robot's own outline
                d = math.hypot(c["x"] - cx, c["y"] - cy)
                # a robot-shaped outline first; one that is not (merged with a board's edge, half in shadow) only
                # says the robot is there, never that it moved
                key = (0 if c.get("shape") else 1, d)
                if d <= PARK_GATE_CM and (best is None or key < best[3]):
                    best = (d, i, c, key)
            if best is None:
                last = p["t_seen"] if p["t_seen"] is not None else (p["since"] if p["since"] is not None else t)
                if t - last >= PARK_UNSEEN_S and p["status"] != "unseen":
                    p["status"] = "unseen"
                    self.park_events.append((t, r, "unseen", dict(x=round(p["x"], 2), y=round(p["y"], 2),
                                                                  unseen_s=round(t - last, 1))))
                p["run"] = 0
                continue
            d, i, c, _ = best
            taken.add(i)
            sure = bool(c.get("shape")) and c.get("moving", 0.0) < self.MOVING_MIN
            was_unseen = p["status"] == "unseen"
            p["t_seen"], p["sx"], p["sy"] = t, c["x"], c["y"]
            p["hist"].append((c["x"], c["y"]))
            del p["hist"][:-max(PARK_MEAN_N, PARK_MOVED_N)]
            if p["ref"] is None:
                p["status"] = "settling"
                last = p["hist"][-PARK_MEAN_N:]
                if len(last) >= PARK_MEAN_N and sure and \
                        max(math.hypot(q[0] - last[-1][0], q[1] - last[-1][1]) for q in last) <= PARK_STILL_CM:
                    # its outline's own place, once it stands still in the picture
                    p["ref"] = (float(np.mean([q[0] for q in last])), float(np.mean([q[1] for q in last])))
                    p["status"] = "seen"
                    p["off_cm"] = 0.0
            else:
                p["off_cm"] = d
                if d >= PARK_MOVED_CM and sure:
                    p["run"] += 1
                    p["status"] = "moving?"
                    last = p["hist"][-min(len(p["hist"]), PARK_MOVED_N):]
                    spread = max(math.hypot(q[0] - last[-1][0], q[1] - last[-1][1]) for q in last)
                    if p["run"] >= PARK_MOVED_N and spread <= PARK_STILL_CM:
                        recent = p["hist"][-min(PARK_MEAN_N, p["run"]):]
                        nx = float(np.mean([q[0] for q in recent]))
                        ny = float(np.mean([q[1] for q in recent]))
                        dx, dy = nx - p["ref"][0], ny - p["ref"][1]
                        ev = dict(x0=round(p["x"], 2), y0=round(p["y"], 2), x=round(p["x"] + dx, 2),
                                  y=round(p["y"] + dy, 2), cm=round(math.hypot(dx, dy), 2))
                        self.park_events.append((t, r, "moved", ev))
                        p["moves"] += 1
                        p["run"] = 0
                        p["status"] = "moved"
                        p["ref"] = (nx, ny)
                        if self.MOVE_SPOT:
                            p["x"], p["y"] = p["x"] + dx, p["y"] + dy
                            self.protect[r] = (p["x"], p["y"])     # the "robot here" mask follows it
                elif d >= PARK_MOVED_CM:
                    p["run"] = 0                # not sure enough to call a move: keep watching
                    p["status"] = "moving?"
                else:
                    p["run"] = 0
                    if p["status"] != "moved" or d < 0.5 * PARK_MOVED_CM:
                        p["status"] = "seen"
            if was_unseen:
                self.park_events.append((t, r, "seen again", dict(x=round(c["x"], 2), y=round(c["y"], 2))))

    def parked_state(self, now=None):
        """{robot: dict(x, y (its spot), status, seen_age_s, off_cm, moves)} for every parked robot."""
        with self.lock:
            now = self.frame_t if now is None else now
            return {r: dict(x=round(p["x"], 2), y=round(p["y"], 2), status=p["status"],
                            seen_age_s=None if p["t_seen"] is None or now is None else round(now - p["t_seen"], 2),
                            off_cm=None if p["off_cm"] is None else round(p["off_cm"], 2), moves=p["moves"])
                    for r, p in self.parked.items()}

    def take_park_events(self):
        with self.lock:
            ev, self.park_events = self.park_events, []
            return ev
