"""Lock-on tracker (Nick, 2026-10-22 evening): the detector (e02_camera.MotionTracker, "something robot-shaped
changed against my memory of the empty floor") finds a robot; this follows THAT robot picture by picture by its own
look, so ghosts, holes and shadows in the floor memory elsewhere don't matter to it.

How it follows:
  - its look: when handed a confirmed robot standing still, it keeps a picture of the robot's whole top as EDGES (the
    picture's gradient: its square outline and the parts on top; a light's glare or a shadow is soft and barely
    shows), inside a circle 0.75 robot widths across the middle, turned in TURN_STEP_DEG steps. 2026-10-22 evening,
    the first version matched plain brightness on a small inner square: it locked onto the glare of Left's lights
    on the board beside it and stayed there when Left drove off;
  - where exactly to lock on: the given place is refined first by matching a drawn square outline of the robot's
    size (all turns) within START_SEARCH_CM, so the look is centred on the body, not on what the detector saw;
  - each picture it searches near where the robot should be (its last place, moved on by its recent speed, or the
    end of the leg the runner sent), and takes the best normalised match over all turns (score 0..1);
  - while the robot is standing still and matches well, the look is refreshed slowly (light changes, its lights);
  - a poor match for LOST_S in a row is a loss: it lets go and the detector finds the robot again.
The steady white lights the runner switches on while a robot drives (firmware `led`) make its look sharper; nothing
here needs them.

2026-10-23 (Nick: "we shouldn't need to tell the program what the two lights look like ... the program should do
what it already does well: detect the robot based on motion. Once it locks on to that image, the tracker ... should
keep track of it"): two changes made the learned look hold on the 19:30-19:32 evening footage (Left, lights steady,
beside the x-board, where the runner lost it four times): seen in 91% of pictures, on the robot in every checked
picture, 6-28 ms a picture:
  - the scenery's own edges are taken out: the features are the picture's edges MINUS the empty floor's edges (the
    detector's floor memory, dilated a little), so a board's edge beside the robot is not part of its look (before
    this, after a re-take the look slid along the x-board);
  - after a turn on its spot (the wheels say it did not move), its look is taken again where it stands (retake()):
    a turn changes how its lights' glare falls on the floor, which turns wrongly with a stored look.
Nothing about the robot is given beforehand; whatever is on it (lights, an attachment) becomes part of its look.

Its score is one witness for the runner's confidence; the others (the detector's periodic check, wheels against the
tracked move, the lights, the front sensor against the map) are combined in e02.py (Confidence).
"""
import math

import cv2
import numpy as np

TURN_STEP_DEG = 10.0          # chosen: the look is matched at this many degrees apart (a DOE could set it: cost vs fit)
RADIUS = 0.75                 # the look: a circle this many robot widths across its middle (the turned corners: 0.71)
START_SEARCH_CM = 5.0         # the place it is handed is refined within this
SEARCH_CM = 6.0               # search this far round the predicted place each picture (25 pictures/s: 10 cm/s = 0.4 cm)
MATCH_OK = 0.55               # chosen: a match at least this good is the robot (to be set from footage: DOE)
REFRESH_MIN = 0.88            # ... and at least this good while standing still refreshes its look
REFRESH_RATE = 0.10           # ... by this share each time
NEAR_TURNS = 7                # turns tried first, nearest the last one (+-30 deg); all of them only if none matches
LOOK = "learned"              # "outline" (a drawn robot-sized square) or "learned" (its own edges): see start()
MIN_LOOK = 38.0               # chosen: a look with less edge (floor's taken out) than this is floor, not a robot
                              # (plain floor 27; robots 83-114, but 49-50 for Right washed out in the bright corner
                              # 2026-10-22 20:26: a thin margin, so a DOE on more footage should set it)
LOST_S = 1.0                  # poor matches this long in a row: let go


class LockTracker:
    def __init__(self, cam, rpx, z_cm, floor=None):
        self.cam = cam
        self.floor_feat = None
        if floor is not None:
            self.set_floor(floor)
        self.rpx = float(rpx)
        self.z = z_cm
        self.base = None              # float32 patch, 2 robot widths square, centred on the robot (not turned)
        self.tpls = []                # [(angle, template uint8)]
        self.pos = None               # (t, x, y)
        self.uv = None
        self.vel = (0.0, 0.0)         # cm/s
        self.score = 0.0
        self.last_ok = None
        self.angle = 0.0

    # --- geometry ---------------------------------------------------------------------------------------------
    def _px(self, x, y):
        return self.cam.project((x, y, self.z))

    def _floor(self, u, v):
        return self.cam.to_floor(u, v, self.z)

    def _px_per_cm(self, x, y):
        return self.cam.px_per_cm(x, y, self.z)

    # --- its look -------------------------------------------------------------------------------------------------
    @staticmethod
    def edges(img):
        """Edges: the gradient's size of the (blurred) grey picture, float32."""
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        g = cv2.GaussianBlur(img, (3, 3), 0).astype(np.float32)
        gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
        return cv2.magnitude(gx, gy)

    FLOOR_GROW_PX = 5             # the floor's edges are widened this much (a picture shifts by a pixel or two)

    def set_floor(self, floor_img):
        """The empty floor (the detector's floor memory): its edges are taken out of every picture's."""
        k = np.ones((self.FLOOR_GROW_PX, self.FLOOR_GROW_PX), np.uint8)
        self.floor_feat = cv2.dilate(self.edges(floor_img), k)

    def features(self, img):
        f = self.edges(img)
        if self.floor_feat is not None and self.floor_feat.shape == f.shape:
            f = np.maximum(0.0, f - self.floor_feat)
        return f

    def _patch(self, img, u, v, half):
        u, v, half = int(round(u)), int(round(v)), int(round(half))
        h, w = img.shape[:2]
        if u - half < 0 or v - half < 0 or u + half + 1 > w or v + half + 1 > h:
            return None
        return img[v - half:v + half + 1, u - half:u + half + 1].astype(np.float32)

    def _mask(self, n):
        c = (n - 1) / 2.0
        yy, xx = np.mgrid[0:n, 0:n]
        return ((xx - c) ** 2 + (yy - c) ** 2 <= (RADIUS * self.rpx) ** 2).astype(np.float32)

    def _rotations(self, base):
        n = base.shape[0]
        c = (n - 1) / 2.0
        out = []
        for k in range(int(round(360.0 / TURN_STEP_DEG))):
            a = k * TURN_STEP_DEG
            M = cv2.getRotationMatrix2D((c, c), a, 1.0)
            out.append((a, cv2.warpAffine(base, M, (n, n), flags=cv2.INTER_LINEAR)))
        return out

    def _make_templates(self):
        self.mask = self._mask(self.base.shape[0])
        # the look outside the circle is zeroed in each turned copy (an unmasked match is ~50x faster than a masked
        # one: 244 ms a picture with masks, 2026-10-22)
        self.tpls = [(a, t * self.mask) for a, t in self._rotations(self.base)]

    def _outline_model(self, n):
        """A drawn robot: its square outline (rpx a side) as edges, n x n."""
        m = np.zeros((n, n), np.float32)
        c = (n - 1) / 2.0
        h = self.rpx / 2.0
        cv2.rectangle(m, (int(round(c - h)), int(round(c - h))), (int(round(c + h)), int(round(c + h))), 1.0, 2)
        return cv2.GaussianBlur(m, (3, 3), 0)

    def _best(self, feat, u, v, half_extra, tpls, mask, near_angle=None):
        n = tpls[0][1].shape[0]
        half = n // 2 + half_extra
        win = self._patch(feat, u, v, half)
        if win is None:
            return None
        best = None
        order = tpls if near_angle is None else sorted(
            tpls, key=lambda at: abs(((at[0] - near_angle) + 180) % 360 - 180))
        for i, (a, tp) in enumerate(order):
            if near_angle is not None and i >= NEAR_TURNS and best is not None and best[0] >= MATCH_OK:
                break                    # a good match among the turns near the last one: no need for the rest
            res = cv2.matchTemplate(win, tp, cv2.TM_CCOEFF_NORMED)
            res = np.nan_to_num(res, nan=0.0, posinf=0.0, neginf=0.0)
            _, mx, _, loc = cv2.minMaxLoc(res)
            if best is None or mx > best[0]:
                best = (mx, int(round(u)) - half + loc[0] + n // 2, int(round(v)) - half + loc[1] + n // 2, a)
        return best

    def start(self, img, x, y, t, check=True):
        """Lock on to the robot at about (x, y) floor cm in this picture (it should stand still in it)."""
        feat = self.features(img)
        n = 2 * int(math.ceil(RADIUS * self.rpx)) + 1
        model = self._outline_model(n)
        u, v = self._px(x, y)
        k = self._px_per_cm(x, y)
        mk = self._mask(n)
        b = self._best(feat, u, v, int(math.ceil(START_SEARCH_CM * k)), [(a, t * mk) for a, t in self._rotations(model)],
                       mk)
        if b is not None:
            _, u, v, _ = b
            x, y = self._floor(u, v)
        p = self._patch(feat, u, v, n // 2)
        if p is None:
            return False
        # LOOK: "learned" = its own edges in this picture (refreshed while still); "outline" = a drawn square of a
        # robot's size, the same for every robot and every light (2026-10-23: the learned look fell to a 0.5 match
        # after a 167 deg turn: the glare of its lights and the floor round it turn with it, wrongly)
        self.look_energy = float((p * self._mask(n)).sum() / max(1.0, self._mask(n).sum()))
        if check and self.look_energy < MIN_LOOK:
            # nothing there but floor (its edges were taken out): no robot to lock on to (2026-10-21 footage,
            # handed a place 7 cm beside Center: plain floor scored 0.8 against itself for seconds)
            return False
        self.base = p if LOOK == "learned" else model
        self._make_templates()
        self.pos, self.uv, self.vel = (t, float(x), float(y)), (u, v), (0.0, 0.0)
        self.score, self.last_ok, self.angle = 1.0, t, 0.0
        return True

    def retake(self, img, t):
        """Take its look again where it stands (after a turn on its spot: the wheels say it did not move)."""
        if self.pos is None:
            return False
        return self.start(img, self.pos[1], self.pos[2], t, check=False)   # the wheels vouch for it

    def nudge(self, x, y, t):
        """Another witness (the detector's moving outline) says the robot is at (x, y): search from there next."""
        if self.pos is not None:
            self.pos = (t, float(x), float(y))
            self.last_ok = t

    def stop(self):
        self.base, self.tpls, self.pos = None, [], None

    @property
    def locked(self):
        return self.base is not None and self.pos is not None

    # --- every picture --------------------------------------------------------------------------------------------
    def step(self, img, t, predict=None, still=False, search_cm=SEARCH_CM, feat=None, in_leg=False, pin_cm=None):
        """One picture. predict: (x, y) where the robot should be, if the runner knows (else: last place moved on by
        its recent speed). still: the runner says it stands still (its look may be refreshed). pin_cm: the wheels
        say it stays at predict (a turn on its spot): a match further than this from it is not taken. Returns (t, x, y,
        score) when the match is good, else None; after LOST_S of poor matches it lets go (locked becomes False)."""
        if not self.locked:
            return None
        t0, x0, y0 = self.pos
        dt = max(0.0, min(1.0, t - t0))
        if predict is not None:
            px_, py_ = predict
        else:
            px_, py_ = x0 + self.vel[0] * dt, y0 + self.vel[1] * dt
        feat = self.features(img) if feat is None else feat
        u, v = self._px(px_, py_)
        k = self._px_per_cm(px_, py_)
        b = self._best(feat, u, v, int(math.ceil(search_cm * k)), self.tpls, self.mask, near_angle=self.angle)
        if b is None:
            return self._miss(t)
        score, cu, cv_, a = b
        self.score = float(score)
        if pin_cm is not None and predict is not None:
            mx, my = self._floor(cu, cv_)
            if not (np.isfinite(mx) and np.isfinite(my)) or math.hypot(mx - predict[0], my - predict[1]) > pin_cm:
                # 2026-10-23, Right 20:26:20 in the bright corner: mid-turn the match crept 9 cm onto the boards'
                # corner and stayed there; the wheels said it was turning on its spot
                score = min(score, MATCH_OK - 1e-3)
        if score < MATCH_OK:
            if in_leg:
                # mid-leg (a turn on its spot blurs the picture): a poor match is expected and not a loss; after the
                # leg the search starts from where the leg should have put it (2026-10-23: lost in every 167 deg turn)
                self.last_ok = t
                if predict is not None:
                    self.pos = (t, float(predict[0]), float(predict[1]))
                return None
            return self._miss(t)
        x, y = self._floor(cu, cv_)
        if not (np.isfinite(x) and np.isfinite(y)):
            return self._miss(t)
        if dt > 0:
            a_ = min(1.0, dt / 0.3)
            self.vel = ((1 - a_) * self.vel[0] + a_ * (x - x0) / dt, (1 - a_) * self.vel[1] + a_ * (y - y0) / dt)
        self.pos, self.uv, self.angle, self.last_ok = (t, float(x), float(y)), (cu, cv_), a, t
        if still:
            self.vel = (0.0, 0.0)
            if score >= REFRESH_MIN and LOOK == "learned":
                self._refresh(feat, cu, cv_, a)
        return (t, float(x), float(y), self.score)

    def _refresh(self, feat, u, v, a):
        n = self.base.shape[0]
        p = self._patch(feat, u, v, n // 2)
        if p is None:
            return
        c = (n - 1) / 2.0
        un = cv2.warpAffine(p, cv2.getRotationMatrix2D((c, c), -a, 1.0), (n, n), flags=cv2.INTER_LINEAR)
        self.base = (1 - REFRESH_RATE) * self.base + REFRESH_RATE * un
        self._make_templates()

    def _miss(self, t):
        if self.last_ok is not None and t - self.last_ok > LOST_S:
            self.stop()
        return None
