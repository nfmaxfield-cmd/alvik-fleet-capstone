#!/usr/bin/env python3
"""Multi-object tracker: turns per-frame guesses into numbered objects that
keep their number from frame to frame.

Rules, in plain words:
  - Gating. An object can only move so far between two frames: at most its
    type's max speed x time since it was last seen, plus a margin, and never
    more than max_gate robot sizes. A look-alike further away than that cannot
    take over an existing track; it starts a new, tentative track instead.
  - Confirming. A new track has to be seen in confirm_n frames (with at most
    tentative_misses missed frames in a row) before it is shown. Once
    confirmed, its earlier tentative frames are filled in too.
  - Coasting. When a confirmed track is not detected (a robot stopped long
    enough to blend into the background, or two robots merged into one blob),
    it stays where it was last seen. Its score drops the longer it is unseen,
    so those frames come up for review.
  - Appearance chain (only when the tracker is given the recording's pictures,
    see appearance.py). An object that is not detected is followed by its
    look: each frame the small patch around it is compared with the patch from
    before. If nothing changed there, the position is held exactly (no drift).
    If it changed, the patch is looked for nearby. Each step is also checked
    against the patch the person confirmed (or, for an object only the motion
    finder saw, the patch from when it stopped); confidence falls as the chain
    gets longer and when that match weakens. Something passing over the object
    (a hand) breaks the chain: those frames are marked chain_broken for a look,
    and the chain picks up again when the object is seen at its place. A chained
    object is never dropped for being unseen; a broken chain ends after
    max_absence_s as before.
  - Ending. A track ends only after a long absence (max_absence_s); hands and
    tracks last seen at the picture edge end after a short absence.
  - Split branches. When two (or three) objects come close together, which one
    is which can get mixed up. Both readings are kept until they have been
    apart for split_decide_s; then the one that fits best (each object carrying
    on the way it moved before, at a speed it can reach, and looking like its
    confirmed patch) is kept. When the readings fit about equally well (two
    identical robots that stopped together), the frames are marked
    split_unresolved so a person can say which is which.
  - Heading. From the motion of the track (see motion.HeadingEstimator); a
    stopped robot keeps the heading it had when it last moved. A detector
    that gives its own heading (a trained model) is trusted instead.
  - User anchors. On a frame the user has labelled, the user's objects
    replace the tracker's: positions, classes, headings and ids are set from
    the user, tracks the user did not keep are ended, objects the user added
    start new tracks. Everything after that frame follows from there, so one
    correction fixes the frames that follow.
  - Rejected spots. Where the user rejected a guess, a new track that starts
    on that spot stays hidden. With pictures: while the spot still looks like
    it did when the guess was rejected (the stain); something that looks
    different there (a robot driving over or put down on it) is found as usual.
    Without pictures: until it moves away from the spot.
  - Per-type motion. Each object type can say how fast it moves, whether it can
    be carried, and whether it can sit still for a long time
    (TrackerParams.types, from_types()); types not listed get generic settings.

Output per frame per track: obj, cls, x, y, heading_deg, heading_known,
heading_guess, score, source (motion | model | track | user), close, where
source 'track' means the tracker filled the position in without a detection.
Added in stage 2 (always present):
  chain_len         frames in a row the position came from the appearance chain
                    (0 when detected or labelled on this frame)
  chain_broken      True: the object was being followed by its look and something
                    passed over it (or it vanished): the position is the last known one
  appearance_score  how alike the object's patch is to its confirmed patch (-1 .. 1),
                    None when not compared on this frame
  still             True: the position was held because nothing moved there
  since_confirmed   frames since a person last labelled this object (None: never)
  confirm_due       True: a chained guess whose confidence fell below ask_below
  split_unresolved  True: one of two or three lookalikes that came together here
                    and could not be told apart afterwards
  split_id          number of that split (TrackRun.splits), else None
  rejected_spot     True: it sits on a spot where a guess was rejected and that still
                    looks the same (scored down hard)
  flags             list of the flag names above that are set (for the queue)

    python tools/label/tracker.py <recording folder>    # prints a summary
"""
import argparse
import bisect
import collections
import copy
import itertools
import math
import os
import sys

from motion import HeadingEstimator, Params as MotionParams, wrap360

try:
    import appearance
except ImportError:          # pictures not used
    appearance = None

FLAG_KEYS = ("split_unresolved", "chain_broken")


class TrackerParams:
    """Tunable numbers. Speeds and distances are in robot sizes."""

    def __init__(self, **kw):
        self.max_speed = 3.0          # robot sizes per second (fastest a robot moves)
        self.hand_speed = 15.0        # robot sizes per second for hands / unknown things
        self.margin = 0.6             # robot sizes added to every gate
        self.max_gate = 3.0           # robot sizes: the gate never grows past this
        self.confirm_n = 5            # frames needed to confirm a new track
        self.tentative_misses = 2     # misses in a row that kill a tentative track
        self.max_absence_s = 45.0     # confirmed track ends after this long unseen
        self.short_absence_s = 1.5    # ... hands, unknown things, and robots last seen at the edge
        self.coast_decay_s = 3.0      # score halves roughly every ~2 s while coasting
        self.no_new_near = 0.6        # robot sizes: no new track this close to a confirmed one
        self.close_factor = 1.3       # robot sizes: two tracks this close are "close" (lower score)
        self.class_penalty = 0.3      # added cost for pairing a detection with a track of another class
        self.loose_classes = ("hand", "unknown")   # types that move fast, change shape and end soon
        self.front_classes = ("alvik", "machine")  # types with a front (they get a heading)
        # --- per-type motion (item 5). name -> dict(max_speed=robot sizes/s, can_be_carried=bool,
        #     can_sit_still=bool); missing names and keys use type_defaults().
        self.types = {}
        self.sit_still_loose = ("unknown",)   # loose types that can still sit still (the catch-all: a cup put down)
        # --- appearance chain (only with pictures) ---------------------------------------------
        self.chain = True             # follow undetected objects by their look
        self.patch_size = 1.4         # robot sizes: side of the compared patch
        self.still_ncc = 0.90         # patch at the same place this alike: nothing moved, position held exactly
        self.step_ok = 0.70           # a moving step needs a match at least this good
        self.step_margin = 0.15       # robot sizes added to the per-frame search radius
        self.ref_weak = 0.85          # match to the confirmed patch at or above this: full confidence
        self.ref_break = 0.50         # ... below this the chain is broken (it is not that object any more)
        self.reacquire_ok = 0.75      # a broken chain picks up again at a match this good
        self.reacquire_max = 2.0      # robot sizes: furthest a broken chain looks for the object
        self.snap = 0.15              # robot sizes: picked up this close to where it was: it never moved
        self.chain_half_life_s = 300.0  # chain confidence halves every this many seconds of chain
        self.held_weight = 0.2        # a still (held) frame counts this much of a moving step
        self.chain_floor = 0.3        # chain confidence factor never below this
        self.chain_min_travel = 1.0   # robot sizes: an object only the motion finder saw must have moved
        self.chain_min_age_s = 1.0    # ... this far, over this long, to be chained (noise doesn't)
        self.ask_below = 0.55         # a chained guess below this is marked confirm_due
        # --- split branches -----------------------------------------------------------------
        self.split = True
        self.split_release = 1.6      # robot sizes: a group of close objects is apart again beyond this
        self.split_decide_s = 1.5     # seconds apart before deciding which is which
        self.split_cap = 3            # most objects (and readings kept) per split
        self.split_margin = 0.6       # best reading must fit this much better than the next one
        self.split_score = 0.5        # score factor on frames of an unresolved split
        self.split_app_weight = 2.0   # weight of the look in the decision
        # --- rejected spots -----------------------------------------------------------------
        self.spot_radius = 0.7        # robot sizes: a guess this close to a rejected spot is "at" it
        self.spot_same = 0.6          # spot looks this alike to when it was rejected: the same stain
        self.spot_score = 0.1         # score factor for a shown guess sitting on such a spot
        for k, v in kw.items():
            if not hasattr(self, k):
                raise TypeError("unknown setting: %s" % k)
            setattr(self, k, v)

    # --- per-type motion ----------------------------------------------------------------------
    def type_defaults(self, cls):
        """Generic settings for a type the project says nothing about."""
        if cls in self.loose_classes:
            return dict(max_speed=self.hand_speed, can_be_carried=cls in self.sit_still_loose,
                        can_sit_still=cls in self.sit_still_loose)
        return dict(max_speed=self.max_speed, can_be_carried=True, can_sit_still=True)

    def motion_of(self, cls):
        m = self.type_defaults(cls)
        for k, v in (self.types.get(cls) or {}).items():
            if k in m and v is not None:
                m[k] = type(m[k])(v)
        return m

    @classmethod
    def from_types(cls, classes, **kw):
        """Settings from a project's object types (project.json "classes": dicts with name and the
        optional max_speed, can_be_carried, can_sit_still). Plain names are allowed (all defaults)."""
        p = cls(**kw)
        for c in classes or ():
            if isinstance(c, dict) and c.get("name"):
                keep = {k: c[k] for k in ("max_speed", "can_be_carried", "can_sit_still") if c.get(k) is not None}
                if keep:
                    p.types[c["name"]] = keep
        return p


# what moves with the object's path (swapped between tracks when a split is decided);
# everything else (number, type, confirmed patch, rows before) stays with the identity
PATH_FIELDS = ("x", "y", "vx", "vy", "pos_hist", "hist", "est", "heading", "h_status", "hits", "miss_run",
               "f_last", "t_last", "det_source", "matched_now", "last_score", "last_det_score", "tpl",
               "chain_state", "chain_len", "chain_w", "held", "broken_t", "app_score", "merged", "chain_tried", "x_lo", "x_hi",
               "y_lo", "y_hi")


class Track:
    """One object followed over time."""

    def __init__(self, tid, x, y, cls, f, t, robot_px, mp):
        self.id = tid
        self.x, self.y = x, y
        self.vx = self.vy = 0.0        # pixels per second
        self.f_last, self.t_last = f, t
        self.f_first, self.t_first = f, t
        self.user_seen = False         # the user has labelled this track on some frame
        self.spot = None               # rejected spot it was born on (kept hidden while there)
        self.hits, self.miss_run = 0, 0
        self.confirmed = False
        self.votes = collections.Counter()
        self.cls_locked = None
        self.est = HeadingEstimator(robot_px, mp)
        self.heading, self.h_status = None, None
        self.last_score = 0.5
        self.last_det_score = 0.5
        self.pos_hist = collections.deque()
        self.hist = collections.deque()  # (t, x, y) over the last ~1.2 s (for splits)
        self.rows = []                 # output rows, one per frame while alive
        self.matched_now = False
        self.anchored_now = False
        self.det_source = "motion"
        self.votes[cls] += 0.01
        self.voted = cls               # class with the most votes so far
        # appearance chain
        self.ref = None                # confirmed patch (the person's, or from when it stopped)
        self.ref_user = False
        self.tpl = None                # the chain's latest patch
        self.chain_state = None        # None (not chained), "ok", "broken"
        self.chain_len = 0
        self.chain_w = 0.0             # seconds of chain, held frames weighted down
        self.held = False
        self.broken_t = None
        self.app_score = None
        self.conf_f = None             # frame the person last labelled it
        self.merged = False            # moving along with a neighbour's blob (touching)
        self.chain_tried = False       # a chain was started since it was last detected
        self.x_lo = self.x_hi = x      # how far it has travelled
        self.y_lo = self.y_hi = y

    @property
    def cls(self):
        return self.cls_locked or self.voted

    def clone(self, with_rows=True):
        """Independent copy (the row list is copied, the rows themselves are shared)."""
        c = copy.copy(self)
        c.est = self.est.clone()
        c.pos_hist = collections.deque(self.pos_hist)
        c.hist = collections.deque(self.hist)
        c.votes = collections.Counter(self.votes)
        c.rows = list(self.rows) if with_rows else []
        return c

    def travel(self):
        return max(self.x_hi - self.x_lo, self.y_hi - self.y_lo)

    def note_pos(self, t):
        self.hist.append((t, self.x, self.y))
        while len(self.hist) > 2 and t - self.hist[0][0] > 1.2:
            self.hist.popleft()
        self.x_lo, self.x_hi = min(self.x_lo, self.x), max(self.x_hi, self.x)
        self.y_lo, self.y_hi = min(self.y_lo, self.y), max(self.y_hi, self.y)

    def velocity(self, t, span=0.5):
        """Average velocity (px/s) over about the last span seconds."""
        old = None
        for (th, xh, yh) in self.hist:
            if t - th <= span + 1e-6:
                old = (th, xh, yh)
                break
        if old is None or t - old[0] < 0.1:
            return self.vx, self.vy
        return (self.x - old[1]) / (t - old[0]), (self.y - old[2]) / (t - old[0])

    def predict(self, t):
        """Where we expect it now (a little extrapolation, then it stays put)."""
        dt = min(max(0.0, t - self.t_last), 0.2)
        return self.x + self.vx * dt, self.y + self.vy * dt

    def update(self, det, f, t, src):
        self.x, self.y = float(det["x"]), float(det["y"])
        self.pos_hist.append((t, self.x, self.y))
        while len(self.pos_hist) > 2 and t - self.pos_hist[0][0] > 0.25:
            self.pos_hist.popleft()
        t0, x0, y0 = self.pos_hist[0]
        if t - t0 > 1e-6:
            self.vx, self.vy = (self.x - x0) / (t - t0), (self.y - y0) / (t - t0)
        self.f_last, self.t_last = f, t
        self.note_pos(t)
        self.hits += 1
        self.miss_run = 0
        c = det.get("cls", "alvik")
        self.votes[c] += float(det.get("score", 0.5))
        if self.votes[c] > self.votes[self.voted]:
            self.voted = c
        self.det_source = src
        if det.get("heading_deg") is not None and "angle90" not in det and src != "motion":
            # A trained model's heading: trust it, but keep the motion history going.
            self.est.update(t, self.x, self.y, None)
            self.est.set_facing(det["heading_deg"], known=True)
            self.heading, self.h_status = wrap360(float(det["heading_deg"])), "known"
        else:
            blind = "angle90" in det and det["angle90"] is None   # outline hidden (e.g. robots touching)
            self.heading, self.h_status = self.est.update(t, self.x, self.y, det.get("angle90"), blind)
        self.last_det_score = float(det.get("score", 0.5))
        self.matched_now = True
        # seen by the detector: the chain starts afresh when it is lost again
        self.chain_state, self.chain_len, self.chain_w, self.held, self.broken_t = None, 0, 0.0, False, None
        self.tpl = None
        self.chain_tried = False

    def chain_move(self, f, t, x, y):
        """The appearance chain moved it (no detection)."""
        self.x, self.y = float(x), float(y)
        self.pos_hist.append((t, self.x, self.y))
        while len(self.pos_hist) > 2 and t - self.pos_hist[0][0] > 0.25:
            self.pos_hist.popleft()
        t0, x0, y0 = self.pos_hist[0]
        if t - t0 > 1e-6:
            self.vx, self.vy = (self.x - x0) / (t - t0), (self.y - y0) / (t - t0)
        self.heading, self.h_status = self.est.update(t, self.x, self.y, None)

    def coast(self, f, t, looked=True):
        """No detection this frame: stay put, keep the last heading."""
        if looked:
            self.miss_run += 1
        self.vx *= 0.5
        self.vy *= 0.5
        if self.est.moving:
            # It was moving when last seen: let the estimator notice it stopped.
            self.heading, self.h_status = self.est.update(t, self.x, self.y, None)
        self.matched_now = False


def _greedy_pairs(pairs):
    """pairs: (cost, track index, detection index). Take the cheapest pairs first."""
    pairs.sort(key=lambda p: p[0])
    used_t, used_d, out = set(), set(), []
    for cost, ti, di in pairs:
        if ti in used_t or di in used_d:
            continue
        used_t.add(ti)
        used_d.add(di)
        out.append((ti, di))
    return out


class Tracker:
    """Frame-by-frame tracker. step() once per frame, in time order.

    frames: optional appearance.Frames of the recording (the pictures). Without it the tracker
    works from the guesses alone, exactly as before stage 2 (no appearance chain; rejected spots by
    position only). spots: optional list of rejected spots from outside (TrackRun.remember_rejected):
    dicts with frame, x, y, r; each one counts from its frame on."""

    def __init__(self, robot_px, img_size=(640, 480), params=None, motion_params=None,
                 reserved_ids=(), det_source="motion", frames=None, spots=None, looker=None):
        self.p = params or TrackerParams()
        self.mp = motion_params or MotionParams()
        self.r = float(robot_px)
        self.w, self.h = img_size
        self.tracks = []
        self.finished = []             # tracks that have ended (kept for their rows)
        self.reserved = set(int(i) for i in reserved_ids)
        self.next_id = 1
        self.det_source = det_source
        self.rejected_spots = []       # (x, y) where the user rejected a guess
        self.spot_patches = {}         # (x, y) -> remembered empty patch (with pictures)
        self.look = looker
        if self.look is None and frames is not None and appearance is not None:
            self.look = appearance.Looker(frames, self.r, self.p.patch_size)
        self.ext_spots = list(spots or [])   # dicts frame, x, y, r, and "_patch" once looked at
        self.groups = []               # open splits
        self.splits = []               # decided splits
        self.next_split = 1
        self._spot_sim = {}

    # --- ids ---------------------------------------------------------------
    def _new_id(self):
        in_use = {tr.id for tr in self.tracks}
        while self.next_id in self.reserved or self.next_id in in_use:
            self.next_id += 1
        self.next_id += 1
        return self.next_id - 1

    # --- main step -----------------------------------------------------------
    def _gate(self, tr, t):
        """How far (px) a detection may be from where the track should be."""
        dt = max(0.0, t - tr.t_last)
        if tr.cls in self.p.loose_classes:
            # Hands move fast and change shape: wide gate (they end soon anyway).
            return self.p.hand_speed * self.r * min(dt, 1.0) + 2.0 * self.p.margin * self.r
        speed = self.p.motion_of(tr.cls)["max_speed"] if self.p.types else self.p.max_speed
        gate = speed * self.r * dt + self.p.margin * self.r
        return min(gate, max(self.p.max_gate, self.p.margin) * self.r)

    def _vetoed(self, tr, d, f):
        """With pictures: a detection that can't be this chained object (a hand passing over a still
        robot, or something that doesn't look like it)."""
        if self.look is None or tr.chain_state is None:
            return False
        dc = d.get("cls", "alvik")
        loose = self.p.loose_classes
        if dc in loose and tr.cls not in loose:
            return True
        if tr.chain_state == "ok" and tr.tpl is not None:
            s = self.look.same_place(f, d["x"], d["y"], tr.tpl)
            if s is not None and s < 0.3:
                return True
        return False

    def _merged_with(self, tr, old):
        """The track (seen this frame) whose blob this unseen one has likely merged into, or None.
        Only for objects that were being seen (not one followed by its look: a parked robot is not
        dragged along by one driving past)."""
        p = self.p
        if not tr.confirmed or tr.cls in p.loose_classes or tr.chain_state is not None:
            return None
        if not (tr.merged or tr.miss_run <= 1):
            return None
        ox, oy = old[id(tr)]
        best, best_d = None, p.close_factor * self.r
        for o in self.tracks:
            if o is tr or not o.matched_now or not o.confirmed or o.cls in p.loose_classes:
                continue
            px, py = old[id(o)]
            d = math.hypot(px - ox, py - oy)
            if d < best_d:
                best, best_d = o, d
        return best

    def _held_spot(self, tr, d, f):
        """(x, y, patch) when a chained, still object gets a detection right at its place and its
        patch there is unchanged (the detector's position wobbles; the object didn't move)."""
        if self.look is None or tr.chain_state != "ok" or tr.tpl is None:
            return None
        if math.hypot(d["x"] - tr.x, d["y"] - tr.y) > self.p.snap * self.r:
            return None
        s = self.look.same_place(f, tr.x, tr.y, tr.tpl)
        if s is None or s < self.p.still_ncc:
            return None
        return tr.x, tr.y, tr.tpl

    def step(self, f, t, dets, anchor=None):
        """Process one frame. dets: list of detection dicts. anchor: optional list
        of the user's objects for this frame. Returns rows for confirmed tracks.
        dets=None means the detector was not run on this frame (a model run on
        every n-th frame only): tracks are carried along without counting a miss."""
        looked = dets is not None
        dets = [d for d in (dets or []) if d.get("cls") != "none"]
        unmatched = set(range(len(dets)))
        self._spot_sim = {}
        old = {}
        for tr in self.tracks:
            tr.matched_now = tr.anchored_now = False
            old[id(tr)] = (tr.x, tr.y)
        # 1. Pair detections with tracks: confirmed tracks get first pick.
        for group in ([tr for tr in self.tracks if tr.confirmed], [tr for tr in self.tracks if not tr.confirmed]):
            pairs = []
            for ti, tr in enumerate(group):
                px, py = tr.predict(t)
                gate = self._gate(tr, t)
                for di in unmatched:
                    d = dets[di]
                    dist = math.hypot(d["x"] - px, d["y"] - py)
                    if dist <= gate:
                        if self._vetoed(tr, d, f):
                            continue
                        cost = dist / gate
                        dc = d.get("cls", "alvik")
                        if dc != tr.cls and "unknown" not in (dc, tr.cls):
                            cost += self.p.class_penalty
                        pairs.append((cost, ti, di))
            for ti, di in _greedy_pairs(pairs):
                tr = group[ti]
                held = self._held_spot(tr, dets[di], f)
                tr.update(dets[di], f, t, dets[di].get("source", self.det_source))
                if held is not None:
                    # seen by the detector, but its look says it hasn't moved: keep the exact place
                    tr.x, tr.y = held[0], held[1]
                    tr.pos_hist = collections.deque([(t, tr.x, tr.y)])
                    tr.vx = tr.vy = 0.0
                    tr.chain_state, tr.tpl, tr.held = "ok", held[2], True
                unmatched.discard(di)
        # 2. Tracks without a detection stay where they are (or follow their look). One that was
        #    seen until now right next to another that was seen this frame has most likely merged
        #    into that one's blob (two robots touching): it moves along with it, so both are still
        #    there when they come apart (and a split decides which is which).
        for tr in self.tracks:
            if not tr.matched_now:
                tr.coast(f, t, looked)
                partner = self._merged_with(tr, old) if (self.p.split and looked) else None
                if partner is not None:
                    ox, oy = old[id(partner)]
                    tr.x += partner.x - ox
                    tr.y += partner.y - oy
                    tr.merged = True
                    continue
                tr.merged = False
                if self.look is not None and self.p.chain and tr.confirmed:
                    self._chain_step(tr, f, t)
        # 3. Leftover detections start tentative tracks (not right on top of a confirmed one).
        for di in sorted(unmatched):
            d = dets[di]
            near = any(tr.confirmed and math.hypot(d["x"] - tr.x, d["y"] - tr.y) < self.p.no_new_near * self.r
                       for tr in self.tracks)
            if near:
                continue
            tr = Track(self._new_id(), float(d["x"]), float(d["y"]), d.get("cls", "alvik"), f, t, self.r, self.mp)
            tr.update(d, f, t, d.get("source", self.det_source))
            # Born where the user rejected a guess before: keep it hidden while it
            # is there (with pictures: while the spot looks as it did; a stain will).
            for sp in self._active_spots(f):
                if math.hypot(tr.x - sp[0], tr.y - sp[1]) < sp[2]:
                    tr.spot = sp
            self.tracks.append(tr)
        # 4. Confirm new tracks, drop failed tentative ones, end long-lost ones.
        keep = []
        for tr in self.tracks:
            if not tr.confirmed:
                if tr.spot is not None:
                    if self._spot_patch(tr.spot) is not None:
                        # remembered what the empty spot looked like: hidden only while it still does
                        if self._spot_looks_same(tr.spot, f) < self.p.spot_same:
                            tr.spot = None
                    elif math.hypot(tr.x - tr.spot[0], tr.y - tr.spot[1]) >= self.r:
                        tr.spot = None             # moved off the rejected spot: a real object
                if tr.hits >= self.p.confirm_n and tr.spot is None:
                    tr.confirmed = True
                elif tr.miss_run > self.p.tentative_misses:
                    continue                       # never confirmed: forget it
            else:
                absent = t - tr.t_last
                at_edge = min(tr.x, tr.y, self.w - tr.x, self.h - tr.y) < 0.75 * self.r
                # A track seen for less than a second that then vanishes was
                # probably noise (a robot needs far longer to blend into the floor).
                young = tr.t_last - tr.t_first < 1.0 and not tr.user_seen
                short = tr.cls in self.p.loose_classes or at_edge or young
                if tr.chain_state == "ok":
                    short = False                  # followed by its look: it is known to be there
                limit = self.p.short_absence_s if short else self.p.max_absence_s
                if absent > limit:
                    if limit == self.p.short_absence_s:
                        # It left: the frames we filled in after it vanished were wrong.
                        tr.rows = [r for r in tr.rows if r["frame"] <= tr.f_last]
                    self.finished.append(tr)
                    continue
            keep.append(tr)
        self.tracks = keep
        # 5. The user's labels for this frame override everything.
        if anchor is not None:
            self._apply_anchor(f, t, anchor)
        # 5b. A spot rejected on this very frame: the guesses sitting on it end here (a person's
        #     own object there is kept).
        for sp in self.ext_spots:
            if int(sp["frame"]) == f:
                rr = max(float(sp.get("r") or 0.0), self.p.spot_radius * self.r)
                gone = [tr for tr in self.tracks if not tr.anchored_now
                        and math.hypot(tr.x - float(sp["x"]), tr.y - float(sp["y"])) < rr]
                for tr in gone:
                    self.tracks.remove(tr)
                    if tr.confirmed:
                        self.finished.append(tr)
        # 6. Output rows. Rows are final once made (never changed later, only replaced by
        #    changed copies when a split is decided), so a re-run can reuse the rows of the
        #    frames before it.
        rows = [self._row(tr, f, t) for tr in self.tracks]
        lim = self.p.close_factor * self.r
        for i in range(len(rows)):
            for j in range(i + 1, len(rows)):
                a, b = rows[i], rows[j]
                if (self.tracks[i].hits >= 3 and self.tracks[j].hits >= 3
                        and math.hypot(a["x"] - b["x"], a["y"] - b["y"]) < lim):
                    for r in (a, b):
                        if not r["close"]:
                            r["close"] = True       # robots close together: where mix-ups happen
                            if r["source"] != "user":
                                r["score"] = round(r["score"] * 0.8, 3)
        out = []
        for tr, row in zip(self.tracks, rows):
            tr.rows.append(row)
            if tr.confirmed:
                out.append(row)
        # 7. Splits: objects that came close together, decided once they have been apart a while.
        if self.p.split:
            self._update_groups(f, t)
        return out

    # --- appearance chain ---------------------------------------------------------------
    def _chainable(self, tr):
        m = self.p.motion_of(tr.cls)
        if not m["can_sit_still"]:
            return False
        if tr.cls in self.p.loose_classes and not tr.user_seen:
            return False
        if tr.user_seen:
            return True
        return (tr.travel() >= self.p.chain_min_travel * self.r
                and tr.t_last - tr.t_first >= self.p.chain_min_age_s)

    def _chain_step(self, tr, f, t):
        """Follow an undetected confirmed track by its look for one frame."""
        p, look = self.p, self.look
        if tr.chain_state is None:
            if not self._chainable(tr):
                return
            # it stood still for its last moments in view: its place is the average of those
            # (the detector's positions wobble; the object didn't)
            recent = [(x, y) for (th, x, y) in tr.hist if t - th <= 1.2]
            if len(recent) >= 5 and not tr.anchored_now and tr.conf_f != tr.f_last:
                xs, ys = [q[0] for q in recent], [q[1] for q in recent]
                if max(max(xs) - min(xs), max(ys) - min(ys)) < 0.2 * self.r:
                    tr.x, tr.y = sum(xs) / len(xs), sum(ys) / len(ys)
            if tr.chain_tried or (not tr.user_seen and t - tr.t_last > 1.0):
                return
            tr.chain_tried = True
            # how it looked on the frame it was last seen, where it was seen: the chain's first
            # link (this frame's picture may already show it gone)
            tpl = look.patch(tr.f_last, tr.x, tr.y)
            if tpl is None or appearance.patch_std(tpl) < appearance.FLAT_STD:
                return
            tr.tpl = tpl
            if not tr.ref_user:
                tr.ref = tpl            # no patch from a person: the one from when it stopped
            tr.chain_state = "ok"
            if tr.ref_user and tr.ref is not None and appearance.ncc(tpl, tr.ref) < p.ref_break:
                tr.chain_state, tr.broken_t = "broken", t
            starting = True
        else:
            starting = False
        dt = max(1e-3, t - tr.t_last) if tr.chain_state == "ok" else 0.04
        speed = self.p.motion_of(tr.cls)["max_speed"]
        tr.held = False
        if tr.chain_state == "ok":
            cur = look.patch(f, tr.x, tr.y)
            same = appearance.ncc(cur, tr.tpl) if cur is not None and cur.shape == tr.tpl.shape else None
            ok = False
            if same is None:
                ok = False
            elif same >= p.still_ncc:
                tr.held, ok = True, True            # nothing moved there: the position stays exactly
            else:
                rad = speed * self.r * min(dt, 0.5) + p.step_margin * self.r
                hit = look.search(f, tr.x, tr.y, rad, tr.tpl)
                if hit is not None and hit["score"] >= p.step_ok and not hit["ambiguous"]:
                    tr.chain_move(f, t, hit["x"], hit["y"])
                    cur = look.patch(f, tr.x, tr.y)
                    if cur is not None and cur.shape == tr.tpl.shape:
                        tr.tpl = cur
                    ok = True
            if ok and tr.ref is not None and cur is not None and cur.shape == tr.ref.shape:
                tr.app_score = appearance.ncc(cur, tr.ref)
                if tr.app_score < p.ref_break:
                    ok = False                      # it no longer looks like the object at all
            if ok:
                tr.chain_len += 1
                tr.chain_w += dt * (p.held_weight if tr.held else 1.0)
                tr.f_last, tr.t_last = f, t
                tr.note_pos(t)
            elif starting and not tr.ref_user:
                # gone (or changed) the moment the detector lost it: not something to follow by its
                # look; it coasts as before stage 2
                tr.chain_state, tr.tpl, tr.held = None, None, False
            else:
                tr.chain_state, tr.broken_t = "broken", t
                tr.held = False
            return
        # broken: look for it again near where it was
        m = self.p.motion_of(tr.cls)
        sp = max(speed, self.p.hand_speed if m["can_be_carried"] else speed)
        rad = min(p.reacquire_max * self.r, 0.3 * self.r + sp * self.r * 0.25 * max(0.0, t - tr.broken_t))
        # look less often the longer it has been gone (each look costs a search)
        gone = t - tr.broken_t
        every = 1 if gone < 2.0 else (2 if gone < 10.0 else 5)
        if every > 1 and f % every:
            return
        # only the trusted patch (the person's, or from when it stopped): a chain that drifted
        # must not pick itself up again on the floor
        best = look.search(f, tr.x, tr.y, rad, tr.ref if tr.ref is not None else tr.tpl)
        if best is not None and best["score"] >= p.reacquire_ok and not best["ambiguous"]:
            if math.hypot(best["x"] - tr.x, best["y"] - tr.y) > p.snap * self.r:
                tr.chain_move(f, t, best["x"], best["y"])
            cur = look.patch(f, tr.x, tr.y)
            if cur is not None and tr.tpl is not None and cur.shape == tr.tpl.shape:
                tr.tpl = cur
            tr.app_score = appearance.ncc(cur, tr.ref) if (cur is not None and tr.ref is not None
                                                           and cur.shape == tr.ref.shape) else best["score"]
            tr.chain_state, tr.broken_t = "ok", None
            tr.chain_len += 1
            tr.f_last, tr.t_last = f, t
            tr.note_pos(t)

    def _chain_score(self, tr):
        p = self.p
        base = 0.9 if tr.ref_user else 0.75
        if tr.app_score is None:
            rf = 1.0
        else:
            rf = (tr.app_score - p.ref_break) / max(1e-6, p.ref_weak - p.ref_break)
            rf = min(1.0, max(0.2, rf))
        decay = max(p.chain_floor, 0.5 ** (tr.chain_w / p.chain_half_life_s))
        return base * rf * decay

    # --- rejected spots ----------------------------------------------------------------------
    def _active_spots(self, f):
        """(x, y, radius, key) of the rejected spots that count on frame f."""
        out = [(sx, sy, self.p.spot_radius * self.r, ("a", sx, sy)) for (sx, sy) in self.rejected_spots]
        for i, sp in enumerate(self.ext_spots):
            if int(sp["frame"]) <= f:
                rr = max(float(sp.get("r") or 0.0), self.p.spot_radius * self.r)
                out.append((float(sp["x"]), float(sp["y"]), rr, ("e", i)))
        return out

    def _spot_patch(self, sp):
        if self.look is None:
            return None
        key = sp[3]
        if key[0] == "a":
            got = self.spot_patches.get((key[1], key[2]))
            return got[1] if got else None
        ext = self.ext_spots[key[1]]
        if "_patch" not in ext:
            ext["_patch"] = self.look.patch(int(ext["frame"]), float(ext["x"]), float(ext["y"]))
        return ext["_patch"]

    def _spot_looks_same(self, sp, f):
        """How alike the spot looks now to when the guess there was rejected (0 .. 1)."""
        k = sp[3]
        if k not in self._spot_sim:
            ref = self._spot_patch(sp)
            now = self.look.patch(f, sp[0], sp[1]) if ref is not None else None
            self._spot_sim[k] = appearance.similarity(ref, now) if now is not None else 0.0
        return self._spot_sim[k]

    def _on_rejected_spot(self, tr, f):
        if self.look is None:
            return False
        for sp in self._active_spots(f):
            if math.hypot(tr.x - sp[0], tr.y - sp[1]) < sp[2] and self._spot_patch(sp) is not None:
                if self._spot_looks_same(sp, f) >= self.p.spot_same:
                    return True
        return False

    # --- rows -----------------------------------------------------------------------------------
    def _row(self, tr, f, t):
        p = self.p
        chained = False
        if tr.anchored_now:
            score, source = 1.0, "user"
        elif tr.matched_now:
            score = 0.6 * tr.last_det_score + 0.4 * min(1.0, tr.hits / (2.0 * p.confirm_n))
            if tr.cls in self.p.front_classes and tr.h_status == "guess":
                score *= 0.75
            if tr.est.first_motion:
                score *= 0.9
            tr.last_score = score
            source = tr.det_source
        elif tr.chain_state == "ok":
            score = self._chain_score(tr)
            tr.last_score = score
            source = "track"
            chained = True
        else:
            absent = t - tr.t_last
            score = tr.last_score * max(0.25, math.exp(-absent / p.coast_decay_s)) * 0.85
            if tr.chain_state == "broken":
                score *= 0.6
            source = "track"
        if not tr.confirmed:
            score *= 0.8            # shown only if the track is confirmed later
        on_spot = False
        if source != "user" and tr.confirmed and self.look is not None and (self.ext_spots or self.rejected_spots):
            on_spot = self._on_rejected_spot(tr, f)
            if on_spot:
                score *= p.spot_score
        heading_known = tr.h_status == "known" and tr.heading is not None and tr.cls in p.front_classes
        guess = tr.heading if (tr.h_status == "guess" and tr.cls in p.front_classes) else None
        broken = tr.chain_state == "broken" and not tr.anchored_now and not tr.matched_now
        app = tr.app_score if (chained or broken) and tr.app_score is not None else None
        score = round(min(1.0, max(0.0, score)), 3)
        return dict(frame=f, obj=tr.id, cls=tr.cls, x=round(tr.x, 1), y=round(tr.y, 1),
                    heading_deg=round(tr.heading, 1) if heading_known else None,
                    heading_known=bool(heading_known),
                    heading_guess=round(guess, 1) if guess is not None else None,
                    score=score, source=source, close=False,
                    chain_len=int(tr.chain_len) if chained else 0, chain_broken=bool(broken),
                    appearance_score=round(float(app), 3) if app is not None else None,
                    still=bool(chained and tr.held),
                    since_confirmed=(f - tr.conf_f) if tr.conf_f is not None else None,
                    confirm_due=bool(chained and score < p.ask_below),
                    split_unresolved=False, split_id=None, rejected_spot=bool(on_spot),
                    flags=["chain_broken"] if broken else [])

    # --- split branches -------------------------------------------------------------------------
    def _update_groups(self, f, t):
        p = self.p
        # objects whose identity matters: not hands, and followed for a while (not a flicker)
        alive = {tr.id: tr for tr in self.tracks if tr.confirmed and tr.cls not in p.loose_classes
                 and (tr.user_seen or tr.t_last - tr.t_first >= 1.0)}
        ids = sorted(alive)
        lim = p.close_factor * self.r
        member_of = {}
        for g in self.groups:
            for i in g["members"]:
                member_of[i] = g
        for a, b in itertools.combinations(ids, 2):
            A, B = alive[a], alive[b]
            if math.hypot(A.x - B.x, A.y - B.y) >= lim:
                continue
            ga, gb = member_of.get(a), member_of.get(b)
            if ga is None and gb is None:
                g = dict(sid=None, members=[], f0=f, t0=t, pre={}, t_sep=None, f_sep=None, sep={}, crowded=False,
                         blind=False)
                self.groups.append(g)
                for tr in (A, B):
                    self._join(g, tr, t)
                    member_of[tr.id] = g
            elif ga is not None and gb is not None:
                if ga is not gb:
                    if len(ga["members"]) + len(gb["members"]) > p.split_cap:
                        ga["crowded"] = gb["crowded"] = True
                    else:
                        ga["members"] += gb["members"]
                        ga["pre"].update(gb["pre"])
                        ga["f0"], ga["t0"] = min(ga["f0"], gb["f0"]), min(ga["t0"], gb["t0"])
                        ga["t_sep"] = None
                        self.groups.remove(gb)
                        for i in gb["members"]:
                            member_of[i] = ga
            else:
                g, tr = (ga, B) if ga is not None else (gb, A)
                if len(g["members"]) >= p.split_cap:
                    g["crowded"] = True
                else:
                    self._join(g, tr, t)
                    member_of[tr.id] = g
                    g["t_sep"] = None
        rel = p.split_release * self.r
        for g in list(self.groups):
            mem = [alive.get(i) for i in g["members"]]
            if g["t_sep"] is None and any(m is not None and not (m.matched_now or m.anchored_now
                                                                or (m.chain_state == "ok" and m.held)) for m in mem):
                g["blind"] = True          # one of them wasn't seen on its own: they may have swapped
            if any(m is None for m in mem):
                self.groups.remove(g)
                live = [m for m in mem if m is not None]
                self._decide(g, live, f, t, early=True)
                continue
            together = any(math.hypot(a.x - b.x, a.y - b.y) < rel for a, b in itertools.combinations(mem, 2))
            if together:
                g["t_sep"] = None
                continue
            if g["t_sep"] is None:
                g["t_sep"], g["f_sep"] = t, f
                g["sep"] = {m.id: (m.x, m.y) for m in mem}
            elif t - g["t_sep"] >= p.split_decide_s:
                self.groups.remove(g)
                self._decide(g, mem, f, t)

    def _join(self, g, tr, t):
        vx, vy = tr.velocity(t)
        g["members"].append(tr.id)
        g["pre"][tr.id] = (t, tr.x, tr.y, vx, vy, tr.cls)

    def _continuity(self, g, mem):
        """How much the way they moved before tells about after (0 .. 1). Objects that pass each
        other in one go: 1. Objects that lingered together far longer than passing takes (they
        stopped, touched, turned): towards 0, the motion before says little about who left where."""
        r = self.r
        pre = [g["pre"][m.id] for m in mem]
        rel = []
        for a, b in itertools.combinations(pre, 2):
            rel.append(math.hypot(a[3] - b[3], a[4] - b[4]))
        rel_speed = max(0.3 * r, sum(rel) / max(1, len(rel)))
        expected = (self.p.close_factor + self.p.split_release) * r / rel_speed + 0.3
        took = max(1e-3, g["t_sep"] - min(q[0] for q in pre))
        return max(0.0, min(1.0, expected / took))

    def _hyp_cost(self, g, ident, path, t):
        """How badly identity ident (its state when the group formed) fits path (a track now)."""
        p, r = self.p, self.r
        w = g.get("w", 1.0)
        t0, x0, y0, vx, vy, cls = g["pre"][ident.id]
        sx, sy = g["sep"].get(path.id, (path.x, path.y))
        ts = g["t_sep"] if g["t_sep"] is not None else t
        span = max(0.0, ts - t0)
        cost = 0.0
        # where it would be had it carried on as before
        px, py = x0 + vx * span, y0 + vy * span
        cost += w * min(math.hypot(sx - px, sy - py), 3.0 * r) / r
        # can it have got there at all?
        speed = self.p.motion_of(cls)["max_speed"]
        if math.hypot(sx - x0, sy - y0) > speed * r * span + 1.0 * r:
            cost += 5.0
        # does it leave in the direction it came, at about the speed?
        dt = max(1e-3, t - ts)
        ux, uy = (path.x - sx) / dt, (path.y - sy) / dt
        s1, s2 = math.hypot(vx, vy), math.hypot(ux, uy)
        if s1 > 0.3 * r and s2 > 0.3 * r:
            cost += w * (1.0 - (vx * ux + vy * uy) / (s1 * s2))
        cost += w * 0.5 * min(2.0, abs(s1 - s2) / r)
        # does it look like it? (only the person's own patch counts: identical objects look alike)
        if self.look is not None and ident.ref is not None and ident.ref_user:
            cur = self.look.patch(g["f_dec"], path.x, path.y)
            if cur is not None and cur.shape == ident.ref.shape:
                cost += p.split_app_weight * (1.0 - max(0.0, appearance.ncc(cur, ident.ref)))
        return cost

    def _decide(self, g, mem, f, t, early=False):
        p = self.p
        g["f_dec"] = f
        sid = self.next_split
        self.next_split += 1
        rec = dict(split_id=sid, objs=sorted(g["members"]), first=g["f0"], last=f, resolved=False, swapped=False,
                   margin=None, ask_frame=g["f_sep"] if g["f_sep"] is not None else f)
        if len(mem) >= 2 and not g.get("blind") and not g["crowded"]:
            # each was seen on its own on every frame: the frame-to-frame pairing can be trusted
            rec["resolved"] = True
        elif len(mem) >= 2 and g["t_sep"] is not None and not g["crowded"]:
            g["w"] = self._continuity(g, mem)
            rec["continuity"] = round(g["w"], 3)
            hyps = []
            for perm in itertools.permutations(range(len(mem))):
                c = sum(self._hyp_cost(g, mem[i], mem[perm[i]], t) for i in range(len(mem)))
                hyps.append((c, perm))
            hyps.sort(key=lambda h: h[0])
            hyps = hyps[:p.split_cap]               # readings kept
            margin = hyps[1][0] - hyps[0][0] if len(hyps) > 1 else 99.0
            rec["margin"] = round(margin, 3)
            if margin >= p.split_margin:
                rec["resolved"] = True
                perm = hyps[0][1]
                if any(perm[i] != i for i in range(len(mem))):
                    self._permute(mem, perm, g["f0"])
                    rec["swapped"] = True
        elif len(mem) < 2:
            rec["resolved"] = True                  # only one left: nothing to mix up
        elif g["t_sep"] is None:
            rec["resolved"] = None                  # still together at the end: nothing to decide yet
        if rec["resolved"] is False:
            # the frames just after they came apart: where a person can tell them apart
            self._flag_unresolved(mem, g["f_sep"] if g["f_sep"] is not None else g["f0"], f, sid)
        self.splits.append(rec)

    def _permute(self, mem, perm, f0):
        """Identity i takes the path of mem[perm[i]] from frame f0 on."""
        states = [{k: getattr(m, k) for k in PATH_FIELDS} for m in mem]
        tails = []
        for m in mem:
            frames = [r["frame"] for r in m.rows]
            k = bisect.bisect_left(frames, f0)
            tails.append((m.rows[:k], m.rows[k:]))
        for i, m in enumerate(mem):
            j = perm[i]
            for k, v in states[j].items():
                setattr(m, k, v)
            new_tail = []
            for r in tails[j][1]:
                c = dict(r)
                c["obj"], c["cls"] = m.id, m.cls
                new_tail.append(c)
            m.rows = list(tails[i][0]) + new_tail

    def _flag_unresolved(self, mem, f0, f1, sid):
        for m in mem:
            frames = [r["frame"] for r in m.rows]
            k = bisect.bisect_left(frames, f0)
            new = list(m.rows[:k])
            for r in m.rows[k:]:
                if r["frame"] <= f1:
                    c = dict(r)
                    c["split_unresolved"], c["split_id"] = True, sid
                    c["flags"] = list(c.get("flags") or []) + ["split_unresolved"]
                    if c["source"] != "user":
                        c["score"] = round(c["score"] * self.p.split_score, 3)
                    new.append(c)
                else:
                    new.append(r)
            m.rows = new

    def finish(self, f, t):
        """End of the recording: decide the splits still open."""
        for g in list(self.groups):
            self.groups.remove(g)
            mem = [tr for tr in self.tracks if tr.id in g["members"]]
            self._decide(g, mem, f, t, early=True)

    # --- user anchors ---------------------------------------------------------------------------
    def _apply_anchor(self, f, t, objs):
        """Make the tracker agree with the user's labels on this frame."""
        for o in objs:
            if o.get("status") == "rejected" and o.get("x") is not None:
                spot = (float(o["x"]), float(o["y"]))
                if spot not in self.rejected_spots:
                    self.rejected_spots.append(spot)
                    if self.look is not None:
                        self.spot_patches[spot] = (f, self.look.patch(f, spot[0], spot[1]))
        objs = [o for o in objs if o.get("status") != "rejected" and o.get("cls") != "none"]
        wanted_ids = {int(o["obj"]) for o in objs}
        by_id = {tr.id: tr for tr in self.tracks}
        claimed = {}
        for o in objs:
            tr = by_id.get(int(o["obj"]))
            if tr is not None:
                claimed[int(o["obj"])] = tr
        for o in objs:
            oid = int(o["obj"])
            if oid in claimed:
                continue
            # The user added or renumbered an object: take over the nearest
            # unclaimed track if there is one right there, else start fresh.
            best, best_d = None, None
            for tr in self.tracks:
                if tr in claimed.values() or tr.id in wanted_ids:
                    continue
                d = math.hypot(tr.x - o["x"], tr.y - o["y"])
                if d < 1.0 * self.r and (best is None or d < best_d):
                    best, best_d = tr, d
            if best is not None:
                new = best.clone(with_rows=False)
                new.id = oid
                new.f_first, new.t_first = f, t
            else:
                new = Track(oid, float(o["x"]), float(o["y"]), o.get("cls", "alvik"), f, t, self.r, self.mp)
            claimed[oid] = new
        # Tracks the user did not keep end here (unconfirmed ones are forgotten).
        kept = set(id(tr) for tr in claimed.values())
        for tr in self.tracks:
            if id(tr) not in kept and tr.confirmed:
                self.finished.append(tr)
        # the person has said which is which: open splits of these objects are settled
        touched = set(by_id) | wanted_ids
        self.groups = [g for g in self.groups if not (set(g["members"]) & touched)]
        self.tracks = []
        for o in objs:
            tr = claimed[int(o["obj"])]
            tr.x, tr.y = float(o["x"]), float(o["y"])
            tr.cls_locked = o.get("cls", "alvik")
            tr.confirmed = True
            tr.hits = max(tr.hits, self.p.confirm_n)
            tr.miss_run = 0
            tr.f_last, tr.t_last = f, t
            tr.last_score = 1.0
            tr.last_det_score = 1.0
            tr.anchored_now = True
            tr.user_seen = True
            if o.get("heading_known") and o.get("heading_deg") is not None:
                tr.est.set_facing(o["heading_deg"], known=True)
                tr.heading, tr.h_status = wrap360(float(o["heading_deg"])), "known"
            elif tr.h_status == "known" and o.get("heading_deg") is None:
                # The user saw the frame and left the heading blank: not sure either.
                tr.h_status = "guess"
                tr.est.known = False
            tr.est.reset_history(t, tr.x, tr.y)
            tr.pos_hist = collections.deque([(t, tr.x, tr.y)])
            tr.hist = collections.deque([(t, tr.x, tr.y)])
            # the appearance chain starts again from the person's patch
            tr.conf_f = f
            tr.chain_state, tr.chain_len, tr.chain_w, tr.held, tr.broken_t = None, 0, 0.0, False, None
            tr.app_score, tr.chain_tried = None, False
            if self.look is not None:
                ref = self.look.patch(f, tr.x, tr.y)
                if ref is not None and appearance.patch_std(ref) >= appearance.FLAT_STD:
                    tr.ref, tr.ref_user, tr.tpl = ref, True, ref
            self.reserved.add(int(o["obj"]))
            self.tracks.append(tr)

    def all_rows(self):
        """Rows of every track that was ever confirmed, by frame number.
        Treat them as read-only (they are shared with later re-runs)."""
        out = collections.defaultdict(list)
        for tr in self.finished + self.tracks:
            if not tr.confirmed:
                continue
            for r in tr.rows:
                out[r["frame"]].append(r)
        return out

    # --- snapshots (to redo the tracking from part-way through) --------------
    def snapshot(self):
        """Cheap copy of the state: rows are not copied, only how many there are."""
        live = []
        for tr in self.tracks:
            c = tr.clone(with_rows=False)
            live.append((c, tr.rows, len(tr.rows)))
        return dict(live=live, finished=list(self.finished), next_id=self.next_id, reserved=set(self.reserved),
                    spots=list(self.rejected_spots), spot_patches=dict(self.spot_patches),
                    groups=copy.deepcopy(self.groups), splits=list(self.splits), next_split=self.next_split)

    def restore(self, snap):
        self.tracks = []
        for c, rows, n in snap["live"]:
            tr = c.clone(with_rows=False)
            tr.rows = rows[:n]
            self.tracks.append(tr)
        self.finished = list(snap["finished"])
        self.next_id = snap["next_id"]
        self.reserved = set(snap["reserved"])
        self.rejected_spots = list(snap["spots"])
        self.spot_patches = dict(snap.get("spot_patches", {}))
        self.groups = copy.deepcopy(snap.get("groups", []))
        self.splits = list(snap.get("splits", []))
        self.next_split = snap.get("next_split", 1)

    def summary(self):
        tracks = [tr for tr in self.finished + self.tracks if tr.confirmed]
        return [dict(obj=tr.id, cls=tr.cls, first=tr.rows[0]["frame"] if tr.rows else tr.f_first,
                     last=tr.rows[-1]["frame"] if tr.rows else tr.f_last, frames=len(tr.rows))
                for tr in sorted(tracks, key=lambda tr: tr.id)]


class TrackRun:
    """Tracking of a whole recording that can be redone quickly from part-way.

    While it runs it keeps a snapshot every snap_every frames. When the user
    changes the labels of frame f, only the part from the last snapshot before
    f onwards is tracked again (frames before f cannot be affected).

    frames: optional appearance.Frames (e.g. appearance.VideoFrames(video, robot_px)): turns on
    the appearance chain and the look-based rejected spots. After run():
      splits                   list of decided splits: dict(split_id, objs, first, last, resolved,
                               swapped, margin, ask_frame)
      frame_flags(f)           flag names on frame f ("split_unresolved", "chain_broken")
      issues()                 the same as stretches, most urgent first, for the queue
      remember_rejected(frame, x, y, r=None) / forget_rejected(...) / rejected_spots()
                               the rejected-spot memory (call run() again from that frame after)
    """

    def __init__(self, dets_by_frame, times, robot_px, img_size=(640, 480), params=None,
                 motion_params=None, det_source="motion", snap_every=150, frames=None):
        self.dets = dets_by_frame
        self.times = times
        self.args = (robot_px, img_size, params, motion_params)
        self.det_source = det_source
        self.snap_every = snap_every
        self.snaps = {}
        self.frames = frames
        self.spots = []                # rejected spots from outside: dicts frame, x, y, r
        self.splits = []
        self.max_id = 0
        self._rows = {}
        self._flags = None
        self.looker = None
        if frames is not None and appearance is not None:
            self.looker = appearance.Looker(frames, float(robot_px), (params or TrackerParams()).patch_size)

    # --- rejected-spot memory -------------------------------------------------------------------
    def remember_rejected(self, frame, x, y, r=None):
        """Remember a spot where a guess was rejected (from frame on). Returns the frame to track
        again from (run(from_frame=...))."""
        sp = dict(frame=int(frame), x=float(x), y=float(y), r=float(r) if r else None)
        for s in self.spots:
            if (s["frame"], s["x"], s["y"]) == (sp["frame"], sp["x"], sp["y"]):
                return sp["frame"]
        self.spots.append(sp)
        self._drop_snaps_after(sp["frame"])
        return sp["frame"]

    def forget_rejected(self, frame, x, y, r=None):
        """Take back remember_rejected. Returns True if it was remembered."""
        n = len(self.spots)
        self.spots = [s for s in self.spots
                      if not (s["frame"] == int(frame) and abs(s["x"] - float(x)) < 0.05 and abs(s["y"] - float(y)) < 0.05)]
        if len(self.spots) != n:
            self._drop_snaps_after(int(frame))
            return True
        return False

    def rejected_spots(self):
        return [dict(frame=s["frame"], x=s["x"], y=s["y"], r=s["r"]) for s in self.spots]

    def _drop_snaps_after(self, f):
        self.snaps = {k: v for k, v in self.snaps.items() if k <= f}

    # --------------------------------------------------------------------------------------------
    def run(self, anchors=None, reserved_ids=(), from_frame=0):
        """Track from from_frame on (earlier frames are reused). Returns
        (rows by frame, track summary)."""
        anchors = anchors or {}
        robot_px, img_size, params, motion_params = self.args
        tk = Tracker(robot_px, img_size, params, motion_params, reserved_ids, self.det_source,
                     spots=self.spots, looker=self.looker)
        start = 0
        usable = [k for k in self.snaps if k <= from_frame]
        if usable:
            start = max(usable)
            tk.restore(self.snaps[start])
            tk.reserved |= set(int(i) for i in reserved_ids)
        self.snaps = {k: v for k, v in self.snaps.items() if k <= start}
        for f in range(start, len(self.dets)):
            if f % self.snap_every == 0 and f not in self.snaps:
                self.snaps[f] = tk.snapshot()
            tk.step(f, self.times[f], self.dets[f], anchors.get(f))
        if len(self.dets):
            tk.finish(len(self.dets) - 1, self.times[len(self.dets) - 1])
        rows = tk.all_rows()
        for f in rows:
            rows[f].sort(key=lambda r: r["obj"])
        self.max_id = tk.next_id - 1       # highest object number handed out (shown or not)
        self.splits = list(tk.splits)
        self._rows = rows
        self._flags = None
        return dict(rows), tk.summary()

    def frame_flags(self, f):
        """Flag names on frame f, from the last run (most urgent first)."""
        if self._flags is None:
            fl = {}
            for g, rs in self._rows.items():
                s = set()
                for r in rs:
                    for k in r.get("flags") or ():
                        s.add(k)
                if s:
                    fl[g] = [k for k in FLAG_KEYS if k in s] + sorted(s - set(FLAG_KEYS))
            self._flags = fl
        return list(self._flags.get(int(f), []))

    def issues(self):
        """Stretches a person should look at, most urgent first: dict(kind, objs, first, last,
        ask_frame). kind: split_unresolved (which is which after lookalikes came together) or
        chain_broken (something passed over an object being followed by its look)."""
        out = [dict(kind="split_unresolved", objs=s["objs"], first=s["first"], last=s["last"],
                    ask_frame=s["ask_frame"]) for s in self.splits if not s["resolved"]]
        runs = {}
        for f in sorted(self._rows):
            for r in self._rows[f]:
                if r.get("chain_broken"):
                    cur = runs.get(r["obj"])
                    if cur and cur[-1]["last"] >= f - 1:
                        cur[-1]["last"] = f
                    else:
                        runs.setdefault(r["obj"], []).append(dict(kind="chain_broken", objs=[r["obj"]], first=f,
                                                                  last=f, ask_frame=f))
        for lst in runs.values():
            for it in lst:
                it["ask_frame"] = min(it["last"] + 1, len(self.dets) - 1)   # the first frame after
                out.append(it)
        return out


def run_tracker(dets_by_frame, times, robot_px, img_size=(640, 480), anchors=None, reserved_ids=(),
                params=None, motion_params=None, det_source="motion", frames=None):
    """Track a whole recording in one go.

    dets_by_frame: list (one entry per frame) of lists of detection dicts
    (None for a frame the detector was not run on).
    times: frame times in seconds (same length). anchors: {frame: [user objects]}.
    frames: optional appearance.Frames (turns on the appearance chain).
    Returns (rows by frame: dict frame -> list of rows, track summary).
    """
    return TrackRun(dets_by_frame, times, robot_px, img_size, params, motion_params, det_source,
                    frames=frames).run(anchors, reserved_ids)


# ------------------------------------------------------------------------------------------------
# Check-in measurement (item 4): how far a chain drifts between two confirmations
# ------------------------------------------------------------------------------------------------

CHECKIN_BINS_S = (2.0, 10.0, 30.0, 120.0, 600.0, float("inf"))


def measure_checkins(dets_by_frame, times, robot_px, anchors, img_size=(640, 480), params=None,
                     motion_params=None, det_source="motion", frames=None, tolerance=0.25, share=0.95):
    """Replay the tracking from each labelled frame to the next labelled frame, with only the first
    one as a label, and compare where each object is guessed on the second with where the person put it.

    anchors: {frame: [user objects]} (as for run_tracker). tolerance: allowed error in robot sizes.
    Returns dict(
      pairs=[dict(obj, from_frame, to_frame, gap_s, gap_frames, chain_len, found, error_px, error_r,
                  within, other_id)],      other_id: the guess there had another number (a mix-up)
      by_length=[dict(up_to_s, n, found, within, median_r, p90_r)],  by time since the label
      suggested_every_s=longest gap (a bin's upper end) up to which at least `share` of the
                        objects were found within tolerance (None: not enough pairs, or not even
                        the shortest gap is good enough; 0.0 when the first bin already fails))
    """
    p = params or TrackerParams()
    r = float(robot_px)
    fs = sorted(f for f, objs in anchors.items() if any(o.get("status") != "rejected" for o in objs))
    look = appearance.Looker(frames, r, p.patch_size) if (frames is not None and appearance is not None) else None
    pairs = []
    for fa, fb in zip(fs, fs[1:]):
        tk = Tracker(r, img_size, p, motion_params, det_source=det_source, looker=look)
        rows_b = []
        for f in range(fa, fb + 1):
            rows_b = tk.step(f, times[f], dets_by_frame[f], anchors[fa] if f == fa else None)
        by_id = {row["obj"]: row for row in rows_b}
        for o in anchors[fb]:
            if o.get("status") == "rejected" or o.get("cls") == "none":
                continue
            if not any(int(a["obj"]) == int(o["obj"]) for a in anchors[fa] if a.get("status") != "rejected"):
                continue
            row = by_id.get(int(o["obj"]))
            nearest = min(rows_b, key=lambda q: math.hypot(q["x"] - o["x"], q["y"] - o["y"]), default=None)
            other = None
            if nearest is not None and nearest["obj"] != int(o["obj"]) and \
                    math.hypot(nearest["x"] - o["x"], nearest["y"] - o["y"]) < tolerance * r:
                other = nearest["obj"]
            err = math.hypot(row["x"] - o["x"], row["y"] - o["y"]) if row else None
            pairs.append(dict(obj=int(o["obj"]), from_frame=fa, to_frame=fb, gap_s=round(times[fb] - times[fa], 3),
                              gap_frames=fb - fa, chain_len=row["chain_len"] if row else None,
                              found=row is not None, error_px=round(err, 2) if err is not None else None,
                              error_r=round(err / r, 3) if err is not None else None,
                              within=bool(err is not None and err <= tolerance * r), other_id=other))
    bins = []
    lo = 0.0
    for hi in CHECKIN_BINS_S:
        sel = [q for q in pairs if lo <= q["gap_s"] < hi]
        if sel:
            errs = sorted(q["error_r"] for q in sel if q["error_r"] is not None)
            bins.append(dict(up_to_s=hi, n=len(sel), found=round(sum(q["found"] for q in sel) / len(sel), 3),
                             within=round(sum(q["within"] for q in sel) / len(sel), 3),
                             median_r=errs[len(errs) // 2] if errs else None,
                             p90_r=errs[min(len(errs) - 1, int(0.9 * len(errs)))] if errs else None))
        lo = hi
    suggested = None
    for b in bins:
        if b["within"] >= share:
            suggested = b["up_to_s"]
        else:
            if suggested is None:
                suggested = 0.0
            break
    return dict(pairs=pairs, by_length=bins, suggested_every_s=suggested, tolerance_r=tolerance)


def main():
    import motion
    ap = argparse.ArgumentParser(description="Track the motion detector's guesses over a recording.")
    ap.add_argument("recording", help="recording folder with video.avi")
    ap.add_argument("--look", action="store_true", help="also follow objects by their look (appearance chain)")
    args = ap.parse_args()
    video = os.path.join(args.recording, "video.avi")
    dets, info = motion.detect_recording(video, progress=lambda d, n: print("  %d / %d frames" % (d, n), end="\r"))
    print()
    fps = info["fps"]
    frames = appearance.VideoFrames(video, info["robot_px"], n=len(dets)) if args.look else None
    rows, summary = run_tracker(dets, [f / fps for f in range(len(dets))], info["robot_px"], frames=frames)
    for s in summary:
        print("obj %3d  %-7s frames %6d-%6d" % (s["obj"], s["cls"], s["first"], s["last"]))


if __name__ == "__main__":
    sys.exit(main())
