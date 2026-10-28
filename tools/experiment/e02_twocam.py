"""Two cameras, one set of robots (MIRLT, 2026-10-27): the overhead and the side camera follow the robots together.

Why. At 11:20 the remote ladder stopped at L0: the overhead camera found all three robots by their lights, but the
tracker never saw their bodies. The board sits under the lamp and is washed out (board 193 of 255; robot tops 14-24
grey levels from it for two robots, 0-3 for the third; the flash check, docs/build-reports/e02-oct27-camera-setup.md).
Nick: "This is why I wanted to use both cameras in conjunction with each other." The side camera looks at the robots'
sides, which the lamp does not wash out.

How. Each camera keeps its own floor memory (e02_multitrack.MultiTracker: what the empty board looks like there) and
finds outlines that differ from it, as before. The outlines are taken to the floor (the overhead's on the plane of the
robots' tops, the side's half way up the body, where its outline's centre sits) and matched to ONE set of robots:
  - who is who: by prediction (the robot's commands, or its own wheels and gyro in cam_bridge), never by looks, the
    same assignment and the same "unsure" rules as MultiTracker, run on each camera's outlines;
  - a robot counts as SEEN when either camera sees it clearly, or both do and agree within AGREE_CM; when both see
    it and disagree, it is UNSURE (flagged, not guessed);
  - the side camera's pictures are LATE by side_lag_s (measured by the setup page's flash check), so its outlines are
    matched to where the robot was then and carried to now by the prediction;
  - its view is slanted: a spot error stretches along its line of sight by 1/cos(angle from straight down) (1.08x near,
    ~1.4x at the far edge; docs/build-plans/two-camera-geometry.md). Distances to its outlines are measured with that
    stretch taken out;
  - its constant offset from the overhead (about 5 cm on 26 Oct, cause unknown) is learned while both see a robot
    standing still, and taken off its outlines, so the planner keeps the overhead's floor frame;
  - the side camera cannot see a robot outside its picture, or one hidden behind another from where it stands: those
    moments are not counted as misses (it says "hidden" or "in line with");
  - lights as markers: when neither camera sees a robot's body, the brightest small spot near its prediction (its
    lights: the ladder keeps them on while it watches, the robots blink them during runs) gives its position, moved
    LIGHT_FWD_CM back from the lights to the robot's middle along its heading (the lights sit at the front of its top).
The floor under a robot standing there at the start is filled in from the board round it; that filled-in floor
is trusted with a margin (MotionTracker.UNKNOWN_EXTRA, 14 grey levels), which hid robots whose tops stand only
14-24 from the board. The margin is now measured on the board itself (tune_unknown), so a plain board keeps it small.

    trk = TwoCamTracker(overhead_cam, board_rect=..., side_cam=side_cam, side_lag_s=0.04)
    trk.set_background(overhead_frames, robots=[(x, y), ...]); trk.set_side_background(side_frames, robots=...)
    trk.add("Left", x, y); trk.push(overhead_img, t); trk.push_side(side_img, t)
    trk.state("Left") -> dict(x, y, status, src: overhead | side | both | lights, sigma, ...)
"""
import math
import threading
import time

import cv2
import numpy as np

import e02_camera as CAM
import e02_multitrack as MT

SIDE_Z_CM = 0.5 * CAM.ROBOT_TOP_CM   # the side outline's centre: half way up the body (e02_sidecam.SIDE_Z_CM)
AGREE_CM = 6.0          # chosen: both cameras' spots this near (side stretch and offset taken out) = they agree. The
                        # two-camera study's held-out agreement: median 1.9 cm, 95 % within 6.0 cm. DOE note: 4 / 6 / 8 cm
                        # on the board against the rate of "cameras disagree" stops and the line check's p95
                        # (docs/build-reports/e02-oct27-two-camera-tracking.md)
FRESH_S = 0.5           # chosen: a camera's latest verdict counts this long when its pictures stop coming (a guard; each
                        # new picture replaces it). No DOE: a guard
SIDE_HZ = 10.0          # chosen: side pictures used a second (its camera sends 29). Two-camera study: robots move
                        # 3-5 cm/s median, 12-26 at the 90th percentile; the Pi's tracker costs ~26-29 ms a picture.
                        # DOE note: 5 / 10 / 15 a second against the Pi's picture rate on the overhead and the side's
                        # seen-rate on the washed-out board
SIDE_LAG_S = 0.04       # default side lag behind the overhead when the setup page measured none (the study: 0 to 0.08 s;
                        # one session -0.40 s, unexplained). Measured: the flash check's lag, saved with the side
                        # calibration. No DOE: a measurement replaces it
LAG_UNC_S = 0.08        # chosen: the lag is known to about this; a moving robot's side gate grows by it x its speed
SIDE_GATE_EXTRA_CM = 2.0  # chosen: the side's gate is this much wider than the overhead's (its outline centre wanders
                          # up and down the body). DOE note: 1 / 2 / 4 cm, side seen-rate against identity swaps in the
                          # simulator's crossings
BIAS_ALPHA = 0.05       # chosen: the side-to-overhead offset is learned this fast (about 20 still pictures). No DOE: a
BIAS_MAX_CM = 8.0       # filter; capped at 8 cm (the 26 Oct offset was 4.7 cm)
BIAS_STILL_CMS = 0.5    # learned only while the robot is told nothing (predicted speed under this)...
BIAS_AREA = 0.7         # ...and the overhead's outline is at least this share of a whole robot (a washed-out top shows
                        # in pieces: dry run 2026-10-27, a 4.5 cm offset learned from the front of the robots alone)
PIECE_GATE_CM = 5.0     # chosen (one system, 28 Oct): when the side looked and did not see it, an overhead piece is taken
                        # only this near where the last sure sighting + its own pose put it (Monday dry run, washed-out
                        # 61 x 122 table: a piece 5.9 cm off a robot turning on its start spot stopped the run; pieces
                        # of a whole robot sit up to ~4.5 cm off its middle, 27 Oct). DOE note: 4 / 5 / 6 cm against
                        # Left's losses at the side picture's edge and wrong fixes on the washed-out replay
BIAS_MIN_N = 30         # chosen: the learned offset is used once this many pictures made it (the file's until then).
                        # No DOE: a filter's start
LIGHT_THR = 30.0        # chosen: a light is this much brighter than the floor memory... (flash check on the real board:
LIGHT_ABS = 235.0       # lights on vs off 50-59 grey levels, board 193) ...and at least this bright. DOE note: 20 / 30 /
                        # 40 against false lights (the robots' white tops) on recorded footage with the lights on
LIGHT_FWD_CM = 3.0      # the lights' middle is this far in front of the robot's middle (two LEDs at the front of its top:
                        # the simulator's draw_robot and the flash check's fake robots). DOE note: measure it from L0
                        # pictures where body and lights are both seen
LIGHT_GATE_CM = 10.0    # chosen: a light this near a robot's prediction can be that robot's
LIGHT_FRESH_S = 0.6     # a light sighting counts this long (blink codes flash every 0.3-0.5 s during runs)
LIGHT_SIGMA_CM = 2.0    # chosen: a lights-only position's 1-sigma (its heading's error moves it); cam_bridge sends it
LIGHT_PAIR_CM = 7.0     # two lit spots this near are one robot's two LEDs (5 cm apart on the Alvik)
AMBIG_LIGHT_CM = 3.0    # another robot's prediction within this of being as near to a light: not used
OCCLUDE_RPX = 1.1       # chosen: two robots whose bodies' centres are this many body widths apart in the side picture
                        # are in line from the side camera: its outlines are not trusted for either
SIDE_BORDER = 0.6       # a side outline whose centre is within this many body widths of the picture's border is cut off
SIDE_HIDDEN = 1.0       # a robot predicted within this many body widths of the side picture's border is "hidden" from
                        # it, not missed (its top sticks out further up the picture than its middle). Chosen, last
                        # night's footage: Left at the picture's top edge. No DOE: geometry
UNKNOWN_EXTRA_RANGE = (4.0, 14.0)   # tune_unknown keeps the filled-in floor's margin within these (14: MotionTracker's)
DARK_MEAN = 50.0        # a side picture darker than this on average (0-255) is brightened before tracking. Real run
DARK_TARGET = 80.0      # 2026-10-27 13:01: the recorder's exposure lock left the side picture at mean 20 (79 under auto
DARK_GAIN_MAX = 5.0     # exposure); its robots stood only 7-10 grey levels from the floor, under the tracker's 16, so it
                        # found nothing. Brightened x4 (to mean 80, the setup page's side picture) all three were found.
                        # Chosen; DOE note: target 60 / 80 / 110 against the side's seen-rate and false outlines on
                        # recorded dark footage (noise is brightened too)


def stretch_at(cam, x, y, z):
    """(1 / cos(angle from straight down), unit floor vector along the line of sight) of camera cam at floor spot (x,
    y), height z: how much a spot error stretches along the line of sight there."""
    C = np.asarray(cam.center, float)
    dx, dy, dz = x - C[0], y - C[1], C[2] - z
    L = math.sqrt(dx * dx + dy * dy + dz * dz)
    h = math.hypot(dx, dy)
    s = L / max(dz, 1e-6)
    u = (dx / h, dy / h) if h > 1e-6 else (1.0, 0.0)
    return s, u


def sight_dist(cam, x, y, z, ex, ey):
    """Distance of the error (ex, ey) with the side camera's along-sight stretch at (x, y) taken out."""
    s, (ux, uy) = stretch_at(cam, x, y, z)
    a = ex * ux + ey * uy
    b = -ex * uy + ey * ux
    return math.hypot(a / s, b)


def tune_unknown(trk, robots, n_spots=10, seed=0):
    """Measure how well the floor under a robot can be filled in from the board round it: fill in n_spots robot-sized
    disks of EMPTY board the same way and compare with what is really there. The margin trusted on filled-in floor
    (UNKNOWN_EXTRA) becomes the 95th percentile of that error, within UNKNOWN_EXTRA_RANGE. Returns the margin."""
    if trk.bg is None or trk.board_rect is None:
        return trk.UNKNOWN_EXTRA
    rng = np.random.default_rng(seed)
    x0, y0, x1, y1 = trk.board_rect
    src = np.clip(trk.bg, 0, 255).astype(np.uint8)
    errs = []
    tries = 0
    while len(errs) < n_spots and tries < 200:
        tries += 1
        x, y = rng.uniform(x0 + 12, x1 - 12), rng.uniform(y0 + 12, y1 - 12)
        if any(math.hypot(x - a, y - b) < 2.5 * CAM.ROBOT_CM for a, b in robots):
            continue
        m = trk._disk_mask(trk.bg.shape, [(x, y)], 1.2)
        if not m.any() or m.sum() > 0.2 * m.size:
            continue
        filled = cv2.inpaint(src, m * 255, 5, cv2.INPAINT_TELEA).astype(np.float32)
        errs.append(np.abs(filled[m > 0] - trk.bg[m > 0]))
    if not errs:
        return trk.UNKNOWN_EXTRA
    p95 = float(np.percentile(np.concatenate(errs), 95))
    trk.UNKNOWN_EXTRA = float(min(UNKNOWN_EXTRA_RANGE[1], max(UNKNOWN_EXTRA_RANGE[0], p95)))
    trk.unknown_measured = round(p95, 1)
    return trk.UNKNOWN_EXTRA


class View(MT.MultiTracker):
    """One more camera on the same robots: its own floor memory and outlines; who is who is decided by the parent
    (TwoCamTracker), whose tracks it shares."""

    def __init__(self, parent, name, cam, board_rect, margin_cm, z_cm, size_per_spot):
        cx, cy = (board_rect[0] + board_rect[2]) / 2.0, (board_rect[1] + board_rect[3]) / 2.0
        super().__init__(cam, board_center=(cx, cy), board_rect=board_rect, margin_cm=margin_cm, z_cm=z_cm,
                         size_per_spot=size_per_spot)
        self.parent, self.name = parent, name
        self.tracks = parent.tracks

    def _covered(self, shape):
        m = np.zeros(shape, np.uint8)
        for tr in list(self.tracks.values()):
            grow = 0.0
            if tr.status != "seen":
                grow = min(1.5, MT.GATE_GROW_CMS * max(0.0, (self.frame_t or tr.bt) - tr.bt) / CAM.ROBOT_CM)
            m |= self._disk_mask(shape, [(tr.x, tr.y)], 1.25 + grow)
        return m

    def set_background(self, frames, robots=None):
        super().set_background(frames, robots)
        with self.lock:
            tune_unknown(self, robots or [])

    def _assign(self, cands, t):
        with self.parent.lock:
            self.parent._fuse_view(self, cands, t)


class TwoCamTracker(MT.MultiTracker):
    """MultiTracker on the overhead camera (it keeps the overhead's floor frame, API and floor memory) plus, when a
    side camera is given, a View on it; both cameras' outlines and the lights decide each robot's verdict."""

    def __init__(self, cam, board_center=(61.0, 61.0), board_rect=None, margin_cm=8.0, side_cam=None,
                 side_lag_s=SIDE_LAG_S, lights=True, side_bias=(0.0, 0.0), **kw):
        super().__init__(cam, board_center=board_center, board_rect=board_rect, margin_cm=margin_cm, **kw)
        self.name = "overhead"
        rect = board_rect or (0.0, 0.0, 2 * board_center[0], 2 * board_center[1])
        self.side = None
        if side_cam is not None:
            self.side = View(self, "side", side_cam, rect, margin_cm, SIDE_Z_CM, True)
        self.side_lag = float(side_lag_s)
        self.side_every = 1.0 / SIDE_HZ
        self.side_t = None
        self.side_n = 0
        self.lights = bool(lights)
        self.bias = [float(side_bias[0]), float(side_bias[1])]       # in use
        self.bias_est = list(self.bias)                                 # being learned
        self.bias_n = 0
        self.light_spots = []                  # the last picture's lights (x, y) on the floor, for the record
        self.counts = {}
        self.side_gain = 1.0                   # side pictures are multiplied by this (dark picture: set_side_background)
        self.side_mean = None

    # --- set-up ------------------------------------------------------------------------------------------
    def set_background(self, frames, robots=None):
        super().set_background(frames, robots)
        with self.lock:
            tune_unknown(self, robots or [])

    def set_side_background(self, frames, robots=None):
        if self.side is not None and frames:
            m = float(np.median([float(np.mean(f)) for f in frames]))
            self.side_mean = round(m, 1)
            self.side_gain = 1.0 if m >= DARK_MEAN else float(min(DARK_GAIN_MAX, DARK_TARGET / max(m, 1.0)))
            self.side.set_background([self._side_img(f) for f in frames], robots)

    def _side_img(self, img):
        if self.side_gain == 1.0:
            return img
        return cv2.convertScaleAbs(img, alpha=self.side_gain)

    def add(self, name, x, y, heading=None, t=None):
        super().add(name, x, y, heading, t)
        with self.lock:
            tr = self.tracks[name]
            tr.obs, tr.src, tr.sigma, tr.obs_t = {}, None, None, None
            self.counts[name] = dict(overhead=0, side=0, both=0, lights=0, disagree=0, unsure=0, missed=0)

    def push_side(self, img, t):
        """One side picture at its Pi time t (NOT corrected for the lag: that is done here). Pictures closer than
        1/SIDE_HZ to the last one used are skipped."""
        if self.side is None:
            return None
        if self.side_t is not None and t - self.side_t < self.side_every - 1e-3:
            return None
        self.side_t = t
        self.side_n += 1
        return self.side.push(self._side_img(img), t)

    # --- geometry of the side camera ---------------------------------------------------------------------
    def _side_xy(self, c):
        """A side outline's floor spot with the learned side-to-overhead offset taken off."""
        return c["x"] - self.bias[0], c["y"] - self.bias[1]

    def _side_dist(self, c, p):
        x, y = self._side_xy(c)
        return sight_dist(self.side.cam, p[0], p[1], SIDE_Z_CM, x - p[0], y - p[1])

    def _side_inside(self, c):
        W, H = self.side.cam.image_size
        r = SIDE_BORDER * self.side._rpx_at(c["x"], c["y"])
        return r <= c["u"] <= W - r and r <= c["v"] <= H - r

    def _side_hidden(self, names, preds):
        """{track index: (kind, why)} for robots the side camera cannot judge now: outside its picture, or in line
        with another robot from where it stands."""
        out, px = {}, []
        W, H = self.side.cam.image_size
        for i, p in enumerate(preds):
            u, v = self.side._px(p[0], p[1])
            r = self.side._rpx_at(p[0], p[1])
            px.append((u, v, r))
            if not (np.isfinite(u) and np.isfinite(v)) or not (r * SIDE_HIDDEN <= u <= W - r * SIDE_HIDDEN
                                                               and r * SIDE_HIDDEN <= v <= H - r * SIDE_HIDDEN):
                out[i] = ("hidden", "outside the side camera's picture")
        for i in range(len(preds)):
            for j in range(i + 1, len(preds)):
                if i in out or j in out:
                    continue
                (ui, vi, ri), (uj, vj, rj) = px[i], px[j]
                if math.hypot(ui - uj, vi - vj) < OCCLUDE_RPX * max(ri, rj):
                    out[i] = ("unsure", "in line with %s from the side camera" % names[j])
                    out[j] = ("unsure", "in line with %s from the side camera" % names[i])
        return out

    def _heading_at(self, tr, t):
        return tr.heading

    def _carry(self, tr, ob, t):
        """An observation (t_ob, x, y, ...) carried to time t by the prediction's motion in between."""
        a, b = self._predict(tr, ob[0]), self._predict(tr, t)
        return ob[1] + (b[0] - a[0]), ob[2] + (b[1] - a[1])

    # --- every picture ---------------------------------------------------------------------------------
    def push(self, img, t):
        self.raw = img                             # (the lights are judged on the picture before its blur)
        return super().push(img, t)

    def _assign(self, cands, t):
        self._fuse_view(self, cands, t)            # (the overhead's push holds self.lock)

    def _fuse_view(self, view, cands, t):
        side = view is not self
        cam = view.name
        te = t - self.side_lag if side else t
        names = list(self.tracks)
        trs = [self.tracks[n] for n in names]
        if not trs:
            return
        preds = [self._predict(tr, te) for tr in trs]
        gates = [self._gate(tr, te) for tr in trs]
        hidden = {}
        dist = None
        if side:
            cands = [c for c in cands if self._side_inside(c)]
            gates = [g + SIDE_GATE_EXTRA_CM + LAG_UNC_S * self._speed(tr, te) for g, tr in zip(gates, trs)]
            dist = self._side_dist
            hidden = self._side_hidden(names, preds)
        usable, verdict = view._verdicts(names, preds, gates, cands, dist)
        verdict.update(hidden)
        for i, tr in enumerate(trs):
            kind, what = verdict.get(i, ("missed", "nothing robot-shaped where it should be"))
            if kind == "seen":
                c = usable[what]
                x, y = self._side_xy(c) if side else (c["x"], c["y"])
                tr.obs[cam] = (te, x, y, "seen", "", c["area"])
            else:
                tr.obs[cam] = (te, None, None, kind, what)
        if not side and self.lights:
            self._lights(trs, preds, te)
        for tr in trs:
            self._decide(tr, te)

    def _lights(self, trs, preds, t):
        """Lights as markers for robots neither camera sees by their body now."""
        need = [i for i, tr in enumerate(trs) if not self._body_seen(tr, t)]
        self.light_spots = []
        if not need or self.frame is None or self.bg is None:
            return
        g = getattr(self, "raw", None)
        g = self.frame if g is None or g.shape != self.bg.shape else g.astype(np.float32)
        d = g - self.offset - self.bg
        m = ((d > LIGHT_THR) & (g >= LIGHT_ABS)).astype(np.uint8)
        n, lab, stats, cents = cv2.connectedComponentsWithStats(m, connectivity=8)
        spots = []
        for k in range(1, n):
            a = int(stats[k, cv2.CC_STAT_AREA])
            if a > 0.35 * self.area:
                continue                               # bigger than a light: a white top or a reflection
            x, y = self._floor(*cents[k])
            if np.isfinite(x) and np.isfinite(y) and self.on_area(x, y):
                spots.append([x, y, float(a)])
        merged = []
        for s in sorted(spots, key=lambda q: -q[2]):
            for o in merged:
                if math.hypot(s[0] - o[0], s[1] - o[1]) < LIGHT_PAIR_CM:
                    w = o[2] + s[2]
                    o[0], o[1], o[2] = (o[0] * o[2] + s[0] * s[2]) / w, (o[1] * o[2] + s[1] * s[2]) / w, w
                    break
            else:
                merged.append(list(s))
        self.light_spots = [(round(a, 1), round(b, 1)) for a, b, _ in merged]
        for i in need:
            tr = trs[i]
            h = self._heading_at(tr, t)
            best = None
            for lx, ly, _ in merged:
                cx, cy = (lx - LIGHT_FWD_CM * math.cos(math.radians(h)), ly - LIGHT_FWD_CM * math.sin(math.radians(h))) \
                    if h is not None else (lx, ly)
                dd = math.hypot(cx - preds[i][0], cy - preds[i][1])
                if dd > LIGHT_GATE_CM + MT.GATE_GROW_CMS * min(1.2, max(0.0, t - tr.bt)):
                    continue
                others = [math.hypot(lx - preds[k][0], ly - preds[k][1]) for k in range(len(trs)) if k != i]
                if any(o < math.hypot(lx - preds[i][0], ly - preds[i][1]) + AMBIG_LIGHT_CM for o in others):
                    continue                           # as near to another robot: not used
                if best is None or dd < best[0]:
                    best = (dd, cx, cy, h is not None)
            if best is not None:
                tr.obs["lights"] = (t, best[1], best[2], "seen", "lights" + ("" if best[3] else " (heading unknown)"))

    def _fresh(self, ob, t, lim):
        return ob is not None and abs(t - ob[0]) <= lim

    def _body_seen(self, tr, t):
        return any(self._fresh(tr.obs.get(c), t, FRESH_S) and tr.obs[c][3] == "seen" for c in ("overhead", "side"))

    def _decide(self, tr, t):
        o, s, li = tr.obs.get("overhead"), tr.obs.get("side"), tr.obs.get("lights")
        so = self._fresh(o, t, FRESH_S) and o[3] == "seen"
        ss = self._fresh(s, t, FRESH_S) and s[3] == "seen"
        sl = self._fresh(li, t, LIGHT_FRESH_S) and li[3] == "seen"
        old = tr.status
        cnt = self.counts.setdefault(tr.name, dict(overhead=0, side=0, both=0, lights=0, disagree=0, unsure=0,
                                                    missed=0))
        pos, src, sigma, why, obs_t = None, None, None, "", None
        if so and len(o) > 5 and o[5] < BIAS_AREA * self.area and self.side is not None and (
                ss or s is None or not self._fresh(s, t, FRESH_S)):
            # the overhead sees only part of it (washed out): the side's whole outline alone when it has one (dry run
            # 2026-10-27: averaging the two put it 2.4 cm off, the side alone 0.5); before the side's first picture, a
            # piece is not taken (a stray first sighting cost a start heading in a dry run). When the side looked and
            # did not see it, the piece is all there is, as with the overhead alone (last night's footage: dropping it
            # there lost Left, at the side picture's edge, twice as often as the overhead alone did)
            so = False
        elif so and len(o) > 5 and o[5] < BIAS_AREA * self.area and self.side is not None and not ss:
            pc, pp = self._carry(tr, o, t), self._predict(tr, t)
            if math.hypot(pc[0] - pp[0], pc[1] - pp[1]) > PIECE_GATE_CM:
                so = False                     # a piece far from where it should be (PIECE_GATE_CM): not taken
                why = "only a piece of it seen by the overhead, %.1f cm from where it should be" % math.hypot(
                    pc[0] - pp[0], pc[1] - pp[1])
        if so and ss:
            po, ps = self._carry(tr, o, t), self._carry(tr, s, t)
            st, _ = stretch_at(self.side.cam, po[0], po[1], SIDE_Z_CM)
            d = sight_dist(self.side.cam, po[0], po[1], SIDE_Z_CM, ps[0] - po[0], ps[1] - po[1])
            if d <= AGREE_CM:
                ws = 1.0 / (st * st)
                pos = ((po[0] + ws * ps[0]) / (1 + ws), (po[1] + ws * ps[1]) / (1 + ws))
                src, sigma, obs_t = "both", 1.0 / math.sqrt(1 + ws), max(o[0], s[0])
                if self._speed(tr, t) < BIAS_STILL_CMS and abs(o[0] - s[0]) < 0.3 and len(o) > 5 \
                        and o[5] >= BIAS_AREA * self.area:
                    # the offset the side camera has against the overhead (raw side minus overhead), learned
                    rx, ry = ps[0] + self.bias[0] - po[0], ps[1] + self.bias[1] - po[1]
                    b = self.bias_est
                    b[0] += BIAS_ALPHA * (rx - b[0])
                    b[1] += BIAS_ALPHA * (ry - b[1])
                    nb = math.hypot(*b)
                    if nb > BIAS_MAX_CM:
                        b[0], b[1] = b[0] * BIAS_MAX_CM / nb, b[1] * BIAS_MAX_CM / nb
                    self.bias_n += 1
                    if self.bias_n >= BIAS_MIN_N:
                        self.bias = list(b)
            else:
                why = "the two cameras disagree by %.1f cm" % d
                cnt["disagree"] += 1
        elif so:
            pos, src, sigma, obs_t = self._carry(tr, o, t), "overhead", 1.0, o[0]
        elif ss:
            st, _ = stretch_at(self.side.cam, s[1], s[2], SIDE_Z_CM)
            pos, src, sigma, obs_t = self._carry(tr, s, t), "side", st, s[0]
        elif sl:
            pos, src, sigma, obs_t = self._carry(tr, li, t), "lights", LIGHT_SIGMA_CM, li[0]
        if pos is not None:
            tr.x, tr.y = pos
            tr.bx, tr.by, tr.bt = pos[0], pos[1], t
            tr.status, tr.why = "seen", ""
            tr.src, tr.sigma = src, sigma
            tr.obs_t = obs_t if tr.obs_t is None else max(tr.obs_t, obs_t)
            self.protect[tr.name] = pos
            cnt[src] += 1
        else:
            tr.x, tr.y = self._predict(tr, t)
            tr.src, tr.sigma = None, None
            unsure = [(c, ob[4]) for c, ob in (("overhead", o), ("side", s))
                      if self._fresh(ob, t, FRESH_S) and ob[3] == "unsure"]
            if why or unsure:
                tr.status = "unsure"
                tr.why = why or "; ".join("%s: %s" % u for u in unsure)
                cnt["unsure"] += 1
            else:
                tr.status = "lost" if t - tr.bt > MT.LOST_S else "predicted"
                tr.why = ("not seen for sure for %.1f s" % (t - tr.bt)) if tr.status == "lost" else \
                    "neither camera sees it, nor its lights"
                cnt["missed"] += 1
        if tr.status != old or (tr.status == "seen" and getattr(tr, "_last_src", None) != tr.src):
            tr.t_status = t if tr.status != old else tr.t_status
            self.events.append((round(t, 3), tr.name, tr.status + ((" (%s)" % tr.src) if tr.status == "seen" else "")
                                + ((": " + tr.why) if tr.why else "")))
        tr._last_src = tr.src
        tr.hist.append((t, tr.x, tr.y, tr.status, tr.src))

    # --- what the runner and the view read ------------------------------------------------------------
    def state(self, name):
        with self.lock:
            tr = self.tracks.get(name)
            if tr is None:
                return None
            return dict(x=tr.x, y=tr.y, status=tr.status, why=tr.why, t_seen=tr.bt, since=tr.t_status,
                        heading=tr.heading, sure_x=tr.bx, sure_y=tr.by, src=getattr(tr, "src", None),
                        sigma=getattr(tr, "sigma", None), obs_t=getattr(tr, "obs_t", None))

    def history(self, name, t0=0.0, t1=None, sure_only=True, body_only=False):
        """[(t, x, y)] between t0 and t1, in time order: sure sightings only (unless sure_only=False); body_only: only
        those where a camera saw its body (not its lights alone)."""
        with self.lock:
            tr = self.tracks.get(name)
            if tr is None:
                return []
            out = [(q[0], q[1], q[2]) for q in tr.hist if q[0] >= t0 and (t1 is None or q[0] <= t1)
                   and (q[3] == "seen" or not sure_only)
                   and (not body_only or (len(q) > 4 and q[4] in ("overhead", "side", "both")))]
        out.sort(key=lambda q: q[0])
        return out

    def summary(self):
        with self.lock:
            return dict(robots={n: dict(c) for n, c in self.counts.items()}, side_bias_cm=[round(b, 2) for b in self.bias],
                        side_bias_n=self.bias_n, side_lag_s=self.side_lag, side_pictures=self.side_n,
                        side_brightness=self.side_mean, side_gain=round(self.side_gain, 2),
                        unknown_margin=dict(overhead=round(self.UNKNOWN_EXTRA, 1),
                                            side=None if self.side is None else round(self.side.UNKNOWN_EXTRA, 1)))


def load_side(path):
    """(CameraModel, lag_s, bias (x, y)) from a side calibration file (the setup page's, or any e02_camera camera
    file; lag and offset default when the file has none)."""
    import json
    cam = CAM.load_camera(path)
    with open(path) as f:
        d = json.load(f)
    lag = d.get("lag_s")
    bias = d.get("offset_cm") or (0.0, 0.0)
    return cam, (SIDE_LAG_S if lag is None else float(lag)), tuple(bias)


class SideFeed(threading.Thread):
    """The side camera's /live pictures into trk.push_side (newest only: a slow tracker skips, never queues)."""

    def __init__(self, trk_getter, stream, log=lambda s: None):
        super().__init__(daemon=True)
        import sources as S
        self.get = trk_getter
        self.src = S.StreamSource(stream, log=log)
        self.n = 0
        self.closed = False
        self.error = None

    def run(self):
        min_lag = None
        try:
            for fr in self.src:
                if self.closed:
                    break
                self.n += 1
                lag = fr.t_arrived - fr.t_pi           # clock offset + transit + queue (the offset cancels below)
                min_lag = lag if min_lag is None else min(min_lag, lag)
                if lag - min_lag > 0.3:
                    continue                           # behind (pictures queued in the stream): skip, as cam_bridge
                trk = self.get()
                if trk is not None and getattr(trk, "side", None) is not None:
                    trk.push_side(fr.img, fr.t_pi - 0.04)
        except Exception as e:
            self.error = str(e)

    def close(self):
        self.closed = True
        self.src.closed = True
