#!/usr/bin/env python3
"""Learned camera trust (e20, Nick 12 Nov: "any camera could be at any angle and the system will work out which is best
for what purpose").

Every camera's answers are checked against INDEPENDENT evidence and scored, never typed in. Nothing here knows the words
"overhead" or "side": a camera is a name the bridge or the detector gives its sightings.

Scores: one per CAMERA x METHOD (motion outline, lights, lock, roof tags, the trained detector "ml", heading from
motion "motion-heading") x JOB (position, heading, identity = telling robots apart) x AREA of the board (AREA_CM squares)
x LIGHT (the lighting key the sighting carries, else the run's). Each score is the share of checks that AGREED, kept as a
Beta count (agreed, disagreed) that forgets: every new check multiplies the old counts by 2^(-1/HALF_LIFE), so old
evidence fades and a camera that was wrong (bumped, wrongly named, in the dark) can earn trust back, and a camera that
was right and then bumped loses it within a few seconds of checks. An area with few checks of its own leans on the
camera's whole-board score for that job (POOL_N checks' worth); a NEW camera has no checks and starts unproven.

A check AGREES when the camera's answer lies within the 99 % band of the two errors together:
  position (relative motion): |camera's displacement - the robot's own wheels' and gyro's displacement| between two
           sightings of that camera <= AGREE_K x sqrt(2 x camera sigma^2 + own sigma(d)^2); own sigma(d) from the
           robot's odometry record (odo_trust) for the distance d driven (a camera's constant offset does not show here;
           another camera does: below)
  position-frame (other cameras at the same picture time): |a - b| <= AGREE_K x sqrt(sigma_a^2 + sigma_b^2) pairwise;
           the largest group of two or more cameras that agree agrees; a camera that disagrees with all of that group
           disagrees (a consensus against it). Two cameras alone that disagree: no evidence (which one moved cannot be
           told). A camera's POSITION state is distrusted when either its motion or its frame score is
  position (the whole picture, e19's whole-camera watch): a robot that drove by its own wheels and a sighting of this
           camera that moved the same way agrees; the watch's "none moved as they did" verdict disagrees (and stands as
           e19's verdict for the cameras in the picture: a camera-wide fault). A wrong-way sighting that lies on another
           robot is no evidence (it may be that robot's own)
  heading  |camera heading - the robot's own gyro heading| <= HEAD_K x sqrt(camera sigma^2 + own heading sigma^2), only
           while its own heading is sure (camera-confirmed, little turning since)
  identity a roof tag naming the robot the layer named (school), an out-and-back / wiggle that named it, against a
           swap or a revoked elimination
The bands are statistical (2-D and 1-D 99 %) on MEASURED sigmas: the camera's own (learned, see MotionHeading) and the
robot's own record. Nothing in them is chosen.

States (R_STAR, the agreement a camera must show; one-sided 95 % Wilson bounds on the forgetting counts):
  trusted     the lower bound is at least R_STAR: its answers are proven (heading from its motion is used, its frame
              checks count for other cameras)
  distrusted  the upper bound is under R_STAR: its answers are refused for that job there (still checked, so it can
              come back)
  unproven    in between (a new camera, a new area): its answers are used as before e20
Teaching the robots' odometry records (odo_trust's samples and checks): each weighted by the camera's lower bound there
(teach_weight: 0 for a new camera with no checks, rising with the evidence), 0 where it is distrusted.
R_STAR = 0.90 is derived: the odometry record's check (k_cal) is a 95th percentile, so it stays honest while at most 5 %
of its checks are gross; a camera must show at least twice that margin (90 % agreement at the 99 % band) to teach it.

Routing (the detector, on demand): Thompson sampling. For the question asked (position, heading, identity) at that
robot's area, each camera's score is drawn from its Beta counts and the highest draw is run. A camera is tried exactly
as often as the evidence leaves it a chance of being the best one, so a less-trusted camera is still tried now and then
(its score stays current), and less often the surer the evidence. No exploration rate is chosen.

Kept per site between runs (~/.config/mirlt/camera_trust/<site>.json; a dry run's MIRLT_CONF_DIR), with how much
evidence each score rests on. MIRLT_CAMERA_TRUST=e19 switches all of it off (e19: one whole-camera verdict).

Changelog
  2026-11-12e20  first version.
"""
import json
import math
import os
import random
import time

VERSION = "camera-trust-2026-11-12e20"

JOBS = ("position", "position-frame", "heading", "identity")
AREA_CM = 30.0        # CHOSEN: a board square of 30 cm (the hard simulator's dark patch; 16 squares on the 122 cm home
                      # board). DOE: no (the pooling below fills sparse squares; smaller squares only need more checks)
HALF_LIFE = 40.0      # CHOSEN: a check's weight halves after 40 newer checks of the same score (at the home Pi's
                      # 1.6 pictures a second and 3 robots, about 10 s of driving). DOE: yes, scored on how fast a bumped
                      # camera is distrusted against false distrust on the real logs (replay)
POOL_N = 5.0          # CHOSEN: an area square leans on its camera's whole-board score with 5 checks' worth. DOE: no
R_STAR = 0.90         # DERIVED (module doc): the agreement a camera must show to teach the odometry record
Z_BOUND = 1.645       # one-sided 95 % (statistical)
AGREE_K = 3.035       # sqrt(-2 ln 0.01): the 2-D 99 % radius in sigmas (statistical)
HEAD_K = 2.576        # the 1-D 99 % band in sigmas (statistical)
LAMBDA = 2.0 ** (-1.0 / HALF_LIFE)
WHOLE_W = 1.0         # one whole-camera watch verdict counts as one check per robot it names (as a robot's own check)
SAVE_KEEP_DAYS = 60   # a square not checked for 60 days is dropped from the site's file (CHOSEN; DOE: no)

# the claimed sigmas of each method when a camera has measured none of its own yet (all MEASURED elsewhere):
CLAIM_POS_CM = {"tags": 0.6, "motion": 1.0, "lock": 1.2, "lights": 1.5, "ml": 1.6, "both": 1.0}
CLAIM_HEAD_DEG = {"ml": 15.0, "tags": 3.0, "motion-heading": None}    # detector-v1 report (overhead) / tag fixes


def light_key(v):
    """A lighting key: a label the source gives ("dark", "lamp", "room"), or None for the run's own."""
    if v is None:
        return None
    return str(v).strip().lower() or None


def cam_of(src, cam=None):
    """The camera of a sighting: `cam` when given; else the part of a bridge source that names a camera
    ("cam_bridge:overhead" -> "overhead"); methods (lock, lights, tags, ml) carry no camera here: None."""
    if cam:
        return str(cam)
    s = str(src or "")
    parts = s.split(":")
    if len(parts) >= 2 and parts[-1] not in ("lock", "lights", "tags", "ml", "motion", ""):
        return parts[-1]
    return None


def _wilson(a, n, z=Z_BOUND):
    """(lower, upper) one-sided bounds of an agreement share a / n (n may be fractional); (0, 1) with no checks."""
    if n <= 1e-9:
        return 0.0, 1.0
    p = min(1.0, max(0.0, a / n))
    z2 = z * z
    c = (p + z2 / (2 * n)) / (1 + z2 / n)
    h = z * math.sqrt(max(0.0, p * (1 - p) / n + z2 / (4 * n * n))) / (1 + z2 / n)
    return max(0.0, c - h), min(1.0, c + h)


class Cell:
    """One score: forgetting counts of agreed / disagreed checks, the raw number of checks, and what they were checked
    against."""
    __slots__ = ("a", "b", "n", "t", "refs")

    def __init__(self, a=0.0, b=0.0, n=0, t=None, refs=None):
        self.a, self.b, self.n, self.t, self.refs = float(a), float(b), int(n), t, dict(refs or {})

    def add(self, ok, w=1.0, t=None, ref=None):
        lam = LAMBDA ** max(0.0, float(w))
        self.a = self.a * lam + (w if ok else 0.0)
        self.b = self.b * lam + (0.0 if ok else w)
        self.n += 1
        self.t = t if t is not None else self.t
        if ref:
            self.refs[ref] = self.refs.get(ref, 0) + 1

    def to_list(self):
        return [round(self.a, 3), round(self.b, 3), self.n, self.t, self.refs]

    @classmethod
    def of(cls, v):
        return cls(v[0], v[1], v[2], v[3] if len(v) > 3 else None, v[4] if len(v) > 4 else None)


class TrustBook:
    """Every camera's learned trust at one site. Keys: (cam, method, job, area, light); area None / light None are the
    camera's whole-board / any-light scores (every check also goes there)."""

    def __init__(self, site="site", path=None, board=None, light=None, seed=None, clock=time.time):
        self.site = str(site or "site")
        self.path = path
        self.board = tuple(board) if board else None
        self.light = light_key(light)      # the run's lighting key (sightings with none of their own)
        self.cells = {}
        self.rng = random.Random(seed)
        self.clock = clock
        self.events = []                    # state changes: dict(t, cam, method, job, area, light, state, why)
        self.state_was = {}
        self.calls = {}                     # (e20, 3) detector calls routed to each camera
        self.loaded_n = 0
        self.say_fn = None

    # -- keys ---------------------------------------------------------------------------------------------------------
    def area(self, x, y):
        if x is None or y is None:
            return None
        x0 = self.board[0] if self.board else 0.0
        y0 = self.board[1] if self.board else 0.0
        return "%d,%d" % (int((float(x) - x0) // AREA_CM), int((float(y) - y0) // AREA_CM))

    def _key(self, cam, method, job, area, light):
        return "%s|%s|%s|%s|%s" % (cam, method, job, "*" if area is None else area, "*" if light is None else light)

    def _cell(self, k, make=False):
        c = self.cells.get(k)
        if c is None and make:
            c = self.cells[k] = Cell()
        return c

    # -- evidence ----------------------------------------------------------------------------------------------------
    def observe(self, cam, method, job, x, y, ok, w=1.0, light=None, t=None, ref=None):
        """One check of `cam`'s `method` answer for `job` at (x, y): ok True (agreed) / False; w its weight (0-1)."""
        if cam is None or w <= 0.0:
            return None
        lt = light_key(light) or self.light
        a = self.area(x, y)
        t = self.clock() if t is None else t
        for ar in (a, None) if a is not None else (None,):
            for li in (lt, None) if lt is not None else (None,):
                self._cell(self._key(cam, method, job, ar, li), True).add(bool(ok), w, t, ref)
        self._state_change(cam, method, job, a, lt, t)
        return ok

    def _state_change(self, cam, method, job, a, lt, t):
        st = self.state(cam, method, job, None, None, light=lt, area=a)
        k = (cam, method, job, a, lt)
        was = self.state_was.get(k)
        if was != st:
            self.state_was[k] = st
            if was is not None:
                s = self.score(cam, method, job, area=a, light=lt)
                row = dict(t=round(t, 2), cam=cam, method=method, job=job, area=a, light=lt, state=st, was=was,
                           agree=s["agree"], checks=s["checks"])
                self.events.append(row)
                del self.events[:-200]
                if self.say_fn is not None and job == "position" and st == "distrusted":
                    try:
                        self.say_fn("camera %s (%s) not trusted for %s in square %s: %s" % (
                            cam, method, job, a, self.words_one(cam, method, job, a, lt)))
                    except Exception:  # noqa: BLE001
                        pass

    # -- scores ------------------------------------------------------------------------------------------------------
    def _counts(self, cam, method, job, area, light):
        """(agreed, total) for a square, with its camera's whole-board score as POOL_N checks of prior."""
        whole = self._cell(self._key(cam, method, job, None, light)) or (
            self._cell(self._key(cam, method, job, None, None)) if light is not None else None)
        if area is None:
            c = whole
            return (c.a, c.a + c.b, c.n) if c else (0.0, 0.0, 0)
        c = self._cell(self._key(cam, method, job, area, light)) or (
            self._cell(self._key(cam, method, job, area, None)) if light is not None else None)
        a = c.a if c else 0.0
        n = (c.a + c.b) if c else 0.0
        if whole is not None and (whole.a + whole.b) > 0:
            m = whole.a / (whole.a + whole.b)
            k = min(POOL_N, whole.a + whole.b)
            a, n = a + k * m, n + k
        return a, n, (c.n if c else 0)

    def score(self, cam, method, job, x=None, y=None, light=None, area="?"):
        ar = self.area(x, y) if area == "?" else area
        lt = light_key(light) or self.light
        a, n, raw = self._counts(cam, method, job, ar, lt)
        lo, hi = _wilson(a, n)
        st = "trusted" if lo >= R_STAR else ("distrusted" if hi < R_STAR else "unproven")
        return dict(cam=cam, method=method, job=job, area=ar, light=lt, agree=round(a / n, 3) if n else None,
                    lower=round(lo, 3), upper=round(hi, 3), evidence=round(n, 1), checks=raw, state=st)

    def state(self, cam, method, job, x=None, y=None, light=None, area="?"):
        """trusted / unproven / distrusted. POSITION combines two scores that test different things: its motion
        against the robots' own (position) and where it puts the board against other cameras (position-frame, a bump
        or an offset): distrusted when either is; trusted when its motion is and its frame is not distrusted."""
        if cam is None:
            return "legacy"
        st = self.score(cam, method, job, x, y, light, area)["state"]
        if job == "position" and st != "distrusted":
            fr = self.score(cam, method, "position-frame", x, y, light, area)
            if fr["state"] == "distrusted":
                return "distrusted"
        return st

    def weight(self, cam, method, job, x=None, y=None, light=None):
        """How much a check made against this camera may teach (0 unless trusted; then its lower bound)."""
        if cam is None:
            return 1.0
        if self.state(cam, method, job, x, y, light) != "trusted":
            return 0.0
        return self.score(cam, method, job, x, y, light)["lower"]

    def teach_weight(self, cam, method, job, x=None, y=None, light=None):
        """(e20, 4) How much a check made against this camera may TEACH the odometry record: 0 when it is distrusted
        there, else its lower bound (a new camera with no checks: 0; it grows with the evidence; a trusted camera at
        least R_STAR). Weighting rather than a cut at "trusted": a cut starved the record at every square with few
        checks (hard simulator, seed 2: no sample for the first 90 s, then a few slips set its 95th percentile)."""
        if cam is None:
            return 1.0
        if self.state(cam, method, job, x, y, light) == "distrusted":
            return 0.0
        return self.score(cam, method, job, x, y, light)["lower"]

    def refused(self, cam, method, job, x=None, y=None, light=None):
        return cam is not None and self.state(cam, method, job, x, y, light) == "distrusted"

    # -- routing (e20, 3) ------------------------------------------------------------------------------------------------
    def choose(self, cams, method, job, x=None, y=None, light=None, rng=None):
        """Thompson sampling: each camera's agreement drawn from Beta(agreed + 1, disagreed + 1) for this job at this
        square; the highest draw wins. Returns the camera name (cams: names)."""
        rng = rng or self.rng
        best, bv = None, -1.0
        lt = light_key(light) or self.light
        ar = self.area(x, y)
        for c in cams:
            a, n, _ = self._counts(c, method, job, ar, lt)
            v = rng.betavariate(a + 1.0, (n - a) + 1.0)
            if v > bv:
                best, bv = c, v
        if best is not None:
            k = "%s|%s" % (best, job)
            self.calls[k] = self.calls.get(k, 0) + 1
        return best

    # -- words and keeping -----------------------------------------------------------------------------------------------
    def cameras(self):
        return sorted({k.split("|")[0] for k in self.cells})

    def words_one(self, cam, method, job, area=None, light=None):
        s = self.score(cam, method, job, area=area, light=light)
        s["state"] = self.state(cam, method, job, area=area, light=light)
        c = self._cell(self._key(cam, method, job, area, light)) or self._cell(self._key(cam, method, job, area, None))
        refs = ", ".join("%d against %s" % (v, k) for k, v in sorted((c.refs if c else {}).items(), key=lambda q: -q[1]))
        if not s["checks"]:
            return "not checked yet"
        return "%s: agreed in %.0f %% of %d checks (%s)" % (s["state"], 100.0 * (s["agree"] or 0.0), s["checks"],
                                                             refs or "-")

    def summary(self):
        """Per camera, per method and job: whole-board score, and each square's state with its evidence."""
        out = {}
        for k, c in self.cells.items():
            cam, method, job, area, light = k.split("|")
            if light != "*":
                continue
            d = out.setdefault(cam, {}).setdefault(method, {}).setdefault(job, dict(whole=None, areas={}))
            ar = None if area == "*" else area
            s = self.score(cam, method, job, area=ar, light=None)
            s.update(refs=dict(c.refs), state=self.state(cam, method, job, area=ar))   # position: motion and frame
            if area == "*":
                d["whole"] = s
            else:
                d["areas"][area] = dict(state=s["state"], agree=s["agree"], checks=s["checks"], lower=s["lower"])
        return dict(version=VERSION, site=self.site, area_cm=AREA_CM, board=self.board, r_star=R_STAR,
                    half_life=HALF_LIFE, cameras=out, calls=dict(self.calls), events=list(self.events[-50:]),
                    lights=sorted({k.split("|")[4] for k in self.cells if not k.endswith("|*")}))

    def plain_words(self):
        """One plain sentence per camera: what it is trusted for, where, and on how much evidence."""
        lines = []
        summ = self.summary()["cameras"]
        for cam in sorted(summ):
            parts = []
            for method, jobs in sorted(summ[cam].items()):
                for job in JOBS:
                    d = jobs.get(job)
                    if not d or not d["whole"]:
                        continue
                    w = d["whole"]
                    areas = d["areas"]
                    tr = sum(1 for v in areas.values() if v["state"] == "trusted")
                    ds = sum(1 for v in areas.values() if v["state"] == "distrusted")
                    parts.append("%s by %s: %s (%d checks; %d of %d squares trusted%s)" % (
                        job, method, w["state"], w["checks"], tr, len(areas),
                        (", %d not trusted" % ds) if ds else ""))
            lines.append("%s: %s" % (cam, "; ".join(parts) if parts else "no checks yet"))
        return lines

    def to_dict(self):
        now = self.clock()
        keep = {k: c.to_list() for k, c in self.cells.items()
                if c.t is None or now - float(c.t) < SAVE_KEEP_DAYS * 86400.0 or now < 1e9}
        return dict(version=VERSION, site=self.site, saved=now, board=self.board, cells=keep)

    def load(self, path=None):
        p = path or self.path
        try:
            with open(p) as f:
                d = json.load(f)
            for k, v in (d.get("cells") or {}).items():
                self.cells[k] = Cell.of(v)
            self.loaded_n = len(d.get("cells") or {})
        except (OSError, ValueError, TypeError, KeyError, IndexError):
            self.loaded_n = 0
        return self

    def save(self, path=None, why=""):
        p = path or self.path
        if not p:
            return None
        try:
            os.makedirs(os.path.dirname(p), exist_ok=True)
            d = self.to_dict()
            d["why"] = why
            tmp = p + ".tmp"
            with open(tmp, "w") as f:
                json.dump(d, f)
            os.replace(tmp, p)
            return p
        except OSError:
            return None


def store_path(site, conf_dir=None):
    d = conf_dir or os.path.expanduser(os.environ.get("MIRLT_CONF_DIR") or "~/.config/mirlt")
    safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(site or "site"))
    return os.path.join(d, "camera_trust", safe + ".json")


def for_site(site, board=None, light=None, conf_dir=None, clock=time.time):
    """The site's book, loaded from its file (a new site: empty, every camera unproven)."""
    p = store_path(site, conf_dir)
    return TrustBook(site, p, board, light, clock=clock).load()


# ======================================================================================================================
# (e20, 2) heading from motion, on every camera
DIR_BINS = 4              # travel directions in 45 deg bins (mod 180: forward and back share a line's noise)
NOISE0_K = 2.0            # CHOSEN: a camera with no measured noise of its own counts as twice its method's claim (new
                          # cameras start cautious; DOE: no, it is replaced after NOISE_MIN segments)
NOISE_MIN = 3             # segments before a direction's own noise replaces the starting guess
SEG_MIN_PTS = 3           # sightings on a straight stretch before its direction is fitted


def dir_bin(deg):
    return int(((deg % 180.0) + 22.5) // 45.0) % DIR_BINS


class MotionHeading:
    """While a robot drives STRAIGHT on its own command (forward or reverse: front and back known), each camera's
    sightings of it are collected; the line through them gives the heading (reversed when it drives backwards).
    Precision: the camera's measured scatter ACROSS the line of travel (learned per camera and travel direction from the
    stretches' own residuals: a camera at a steep angle is worse along its depth) over the distance travelled:
    sigma_heading = sqrt(2) x sigma_across / distance (radians; 1.6 cm over 10 cm is about 13 deg)."""

    def __init__(self):
        self.seg = {}              # (robot, cam) -> dict(sign, pts [(t, x, y)], own0, t0)
        self.noise = {}            # (cam, bin) -> [residual rms ...] (the last 50)
        self.last = {}             # robot -> newest heading result dict
        self.counts = dict(segments=0, headings=0)

    def sigma_across(self, cam, deg, claim=1.0):
        v = self.noise.get((cam, dir_bin(deg))) or []
        if len(v) < NOISE_MIN:
            allv = [q for (c, _), vv in self.noise.items() if c == cam for q in vv]
            if len(allv) >= NOISE_MIN:
                return sorted(allv)[len(allv) // 2]
            return NOISE0_K * claim
        return sorted(v)[len(v) // 2]

    def sigma_cam(self, cam):
        """The camera's measured position noise (cm): the worst travel direction's median scatter across the line (a
        camera at a steep angle: its depth axis), once a direction has NOISE_MIN stretches; else None."""
        v = [sorted(vv)[len(vv) // 2] for (c, _), vv in self.noise.items() if c == cam and len(vv) >= NOISE_MIN]
        return max(v) if v else None

    def feed(self, robot, cam, t, x, y, sign, own_th=None, claim=1.0):
        """One sighting of `robot` by `cam`; sign +1 forward / -1 backward / 0 not driving straight. Returns a finished
        stretch's heading dict(th, sigma, L, cam, n, bin) when the stretch ends (sign changed, stopped, or turned),
        else None."""
        k = (robot, cam)
        s = self.seg.get(k)
        res = None
        turned = s is not None and own_th is not None and s.get("own0") is not None and \
            abs((own_th - s["own0"] + 180.0) % 360.0 - 180.0) > 10.0
        if s is not None and (sign != s["sign"] or turned):
            res = self._close(robot, cam, s, claim)
            s = None
            self.seg.pop(k, None)
        if sign != 0 and s is None:
            s = self.seg[k] = dict(sign=sign, pts=[], own0=own_th, t0=t)
        if s is not None:
            s["pts"].append((t, float(x), float(y)))
            del s["pts"][:-60]
        return res

    def _close(self, robot, cam, s, claim):
        pts = s["pts"]
        if len(pts) < SEG_MIN_PTS:
            return None
        n = len(pts)
        mx = sum(p[1] for p in pts) / n
        my = sum(p[2] for p in pts) / n
        sxx = sum((p[1] - mx) ** 2 for p in pts)
        syy = sum((p[2] - my) ** 2 for p in pts)
        sxy = sum((p[1] - mx) * (p[2] - my) for p in pts)
        ang = 0.5 * math.atan2(2 * sxy, sxx - syy)           # the line's direction (total least squares)
        ux, uy = math.cos(ang), math.sin(ang)
        a0 = (pts[0][1] - mx) * ux + (pts[0][2] - my) * uy
        a1 = (pts[-1][1] - mx) * ux + (pts[-1][2] - my) * uy
        if a1 < a0:                                          # the line pointed the way it went
            ux, uy, a0, a1 = -ux, -uy, -a0, -a1
        L = a1 - a0
        res_ = [abs(-(p[1] - mx) * uy + (p[2] - my) * ux) for p in pts]
        rms = math.sqrt(sum(r * r for r in res_) / max(1, n - 2))
        th_travel = math.degrees(math.atan2(uy, ux))
        self.counts["segments"] += 1
        sig_before = self.sigma_across(cam, th_travel, claim)
        if L > 2.0 * max(0.3, sig_before):
            v = self.noise.setdefault((cam, dir_bin(th_travel)), [])
            v.append(max(0.05, rms))
            del v[:-50]
        sig = self.sigma_across(cam, th_travel, claim)
        if L < 3.0 * sig:
            return None                                       # moved less than its noise: no heading
        th = th_travel + (180.0 if s["sign"] < 0 else 0.0)
        th = (th + 180.0) % 360.0 - 180.0
        out = dict(robot=robot, cam=cam, th=round(th, 2), sigma=round(math.degrees(math.sqrt(2.0) * sig / L), 2),
                   L=round(L, 2), n=n, bin=dir_bin(th_travel), t=pts[-1][0], t0=pts[0][0], rms=round(rms, 3))
        self.counts["headings"] += 1
        self.last[robot] = out
        return out

    def summary(self):
        return dict(counts=dict(self.counts),
                    noise={"%s|%d" % k: round(sorted(v)[len(v) // 2], 3) for k, v in self.noise.items() if v})
