"""The live tracker: blink lights -> named robot poses, one frame at a time.

Built on tools/blink/blink.py's stages (spots, on-segments, LED pairs, tracks,
rate decoding), run causally: each frame is handled once, when it arrives.

Two ways a fix comes out:
  fast   a robot whose name and heading are already known: its two LEDs light
         up where and when its track says they should (on its beat, near the
         predicted spots, the right distance apart, dark in the frame before).
         Sent from the frame itself, so the only delay is the processing.
  late   when a flash has ended (about 3 frames later) and blink.py's checks
         (switched on, then off; at most 250 ms; the track's rhythm) accept
         it: every frame of it the fast path did not already send.

Identity comes from the rhythm of the track's last 12 flashes (blink.py's
decode_rate): named only if the best code has probability >= name_p (0.9),
from >= 5 flash-to-flash intervals, >= 75% of flashes on its beat, its spot
not shared with other flashing, and no other live track has the same name
(two tracks claiming one name: neither gets fixes until one ends).

Heading: the line between the two LEDs is square to the heading, which gives
the heading up to front/back. Front/back: the robot's body lies behind its
lights (they sit near the front), so the picture just behind the lights looks
less like the floor beside the robot than the picture just ahead of them
(appearance vote), plus, weakly, the way it drives (motion vote). Votes add up
over flashes; no fix is sent until they clearly agree (heading "resolved");
between flashes the heading is carried by continuity (the candidate nearest
the predicted heading).

The pose sent is the turning centre: the light midpoint minus the light's
offset turned to the heading (calib.py's spin_led_fwd/left_cm, default 2.75 cm
forward)."""
import math
import os
import sys
from collections import deque
from dataclasses import dataclass, field

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (TOOLS, os.path.join(TOOLS, "blink")):
    if p not in sys.path:
        sys.path.insert(0, p)

import blink as B  # noqa: E402

DEFAULT_LED_OFFSET = (2.75, 0.0)
DEBUG = bool(os.environ.get("LIVE_DEBUG"))


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


@dataclass
class LiveParams:
    name_p: float = 0.9               # identity: best code's probability at least this...
    name_intervals: int = 5           # ...from at least this many flash-to-flash intervals
    led_offset: dict = field(default_factory=dict)   # name -> (forward, left) cm, light from turning centre
    sigma_cm: float = 0.5             # fix position sd sent with each fix (guess, as tools/fusion's blink_xy)
    sigma_th_deg: float = 1.5         # heading sd floor (LED-pair angle noise is added)
    led_px_sd: float = 0.25           # one LED's centre, px sd (blink README: 0.21 px 95% in synthetic tests)
    resolve_votes: float = 2.0        # front/back evidence needed before fixes are sent
    motion_vote: float = 0.3          # weight of one motion vote (robots also drive backwards)
    body_behind_cm: float = 4.5       # appearance samples: this far ahead of / behind the lights
    side_cm: float = 7.5              # floor reference: this far beside the light midpoint
    sep_tol_cm: float = 1.2           # LED separation must match the track's within this
    max_turn_jump_deg: float = 50.0   # heading between flashes: further from the prediction = start again
    max_turn_dps: float = 100.0       # fastest turn (follow-run IMU turns: 40 rpm wheels = about 90 deg/s)
    max_votes: float = 4.0            # front/back evidence is capped, so contrary votes act quickly
    fast: bool = True                 # the fast path
    fast_amax_cms2: float = 30.0      # fast path: speed change allowed between flashes (sim robots: 20)
    frames_kept: int = 25             # pictures kept for the appearance vote


class LiveTrack(B.Track):
    KEEP = 40

    def __init__(self, tid):
        super().__init__(tid)
        self.n_lights = 0
        self.foreign = 0
        self.name, self.code, self.p_name = "", 0, 0.0
        self.why = "new"
        self.h = None            # heading (deg, floor) of the front hypothesis
        self.h_t = None
        self.rate = 0.0          # deg/s
        self.votes = 0.0
        self.sep_cm = deque(maxlen=9)
        self.sep_px = deque(maxlen=9)
        self.fast = {}           # frame -> fix sent by the fast path
        self.fast_open = None    # (fi, t, pa, pb) the fast path's last detection
        self.last_mid = None     # (t, x, y) floor light midpoint of the last paired light
        self.named_name, self.named_t = "", -1e9   # the last name it had, and when
        self.suspect = 0         # flashes still to come before fixes resume (after an ambiguous one)

    def add(self, L, p):
        super().add(L, p)
        self.n_lights += 1
        if len(self.lights) > self.KEEP:
            del self.lights[:len(self.lights) - self.KEEP]

    @property
    def resolved(self):
        return self.h is not None and self.votes >= self._resolve

    _resolve = 2.0

    def sep_med(self):
        return float(np.median(self.sep_cm)) if self.sep_cm else None

    def sep_px_med(self):
        return float(np.median(self.sep_px)) if self.sep_px else None


def seg_pos(L):
    """Per frame: (A, B) LED pixel positions where both LEDs were seen."""
    if len(L.segs) != 2:
        return {}
    A = {f: (x, y) for f, x, y in zip(L.segs[0].fi, L.segs[0].x, L.segs[0].y)}
    Bq = {f: (x, y) for f, x, y in zip(L.segs[1].fi, L.segs[1].x, L.segs[1].y)}
    return {f: (A[f], Bq[f]) for f in A if f in Bq}


class LiveTracker:
    """push(frame image, Pi time) -> list of messages (dicts): fixes (type fix,
    robot, t_pi, x, y, th, sigma, sigma_th, id_p, path, frame) and sightings
    (type sighting: a light whose identity or heading isn't sure; probabilities)."""

    def __init__(self, fmap, names=None, p=None, lp=None):
        self.fmap = fmap
        self.p = p or B.Params(names=dict(names or B.DEFAULT_NAMES), px_per_cm=fmap.px_per_cm())
        self.lp = lp or LiveParams()
        LiveTrack._resolve = self.lp.resolve_votes
        self.seg = B.SegmentFinder(self.p)
        self.pending = []        # finished LED segments waiting for their partner
        self.tracks = []
        self.n_tracks = 0
        self.frames = {}         # frame index -> picture (gray), the last few
        self.fi = -1
        self.stats = dict(frames=0, fixes=0, fast=0, late=0, sightings=0, contested=0, fast_unconfirmed=0,
                          unresolved=0, sep_reject=0, flips=0)

    # ------------------------------------------------------------ helpers
    @property
    def frame_t(self):
        return self.seg.frame_t

    def led_offset(self, name):
        return self.lp.led_offset.get(name) or self.lp.led_offset.get(None) or DEFAULT_LED_OFFSET

    def _new_track(self):
        self.n_tracks += 1
        return LiveTrack(self.n_tracks)

    def _patch(self, img, u, v, r):
        H, W = img.shape[:2]
        x0, x1 = int(round(u)) - r, int(round(u)) + r + 1
        y0, y1 = int(round(v)) - r, int(round(v)) + r + 1
        if x0 < 0 or y0 < 0 or x1 > W or y1 > H:
            return None
        return float(np.median(img[y0:y1, x0:x1]))

    def appearance_vote(self, img, mid, h_deg):
        """+1: heading h looks right (floor-like ahead of the lights, body behind),
        -1: the other way round, about 0: can't tell."""
        if img is None:
            return 0.0
        lp, fm = self.lp, self.fmap
        c, s = math.cos(math.radians(h_deg)), math.sin(math.radians(h_deg))
        r = max(2, int(round(1.0 * self.p.px_per_cm)))
        x, y = mid
        A = fm.to_px(x + lp.body_behind_cm * c, y + lp.body_behind_cm * s, 0.0)
        Bk = fm.to_px(x - lp.body_behind_cm * c, y - lp.body_behind_cm * s)
        refs = [self._patch(img, *fm.to_px(x - lp.side_cm * s * k, y + lp.side_cm * c * k, 0.0), r) for k in (1, -1)]
        refs = [q for q in refs if q is not None]
        a, b = self._patch(img, *A, r), self._patch(img, *Bk, r)
        if a is None or b is None or not refs:
            return 0.0
        F = float(np.median(refs)) if len(refs) == 1 else min(refs, key=lambda q: abs(q - a) + abs(q - b))
        da, db = abs(a - F), abs(b - F)
        return float(np.clip((db - da) / max(db, da, 12.0), -1.0, 1.0))

    def _floor_pair(self, pa, pb):
        fa = self.fmap.to_floor(*pa)
        fb = self.fmap.to_floor(*pb)
        return fa, fb, ((fa[0] + fb[0]) / 2, (fa[1] + fb[1]) / 2), math.hypot(fb[0] - fa[0], fb[1] - fa[1]), \
            math.degrees(math.atan2(fb[1] - fa[1], fb[0] - fa[0]))

    def _pick(self, axis_deg, pred):
        c1, c2 = wrap180(axis_deg + 90.0), wrap180(axis_deg - 90.0)
        return c1 if abs(wrap180(c1 - pred)) <= abs(wrap180(c2 - pred)) else c2

    def _pred_heading(self, T, t):
        dt = max(0.0, t - T.h_t)
        return T.h + float(np.clip(T.rate, -150, 150)) * min(dt, 1.0)

    # ------------------------------------------------------------ track state
    def _identity(self, T):
        p, lp = self.p, self.lp
        dec = T.dec
        if dec is None:
            T.name, T.code, T.p_name, T.why = "", 0, 0.0, "too few flashes"
            return
        best = dec["code"]
        pb = dec["probs"].get(best, 0.0)
        shared = T.foreign / max(1, T.n_lights)
        why = ""
        if not best:
            why = "no code"
        elif pb < lp.name_p:
            why = "code unclear (p %.2f)" % pb
        elif dec["n"] < lp.name_intervals:
            why = "too few flashes"
        elif dec["rhythm"] < p.min_rhythm:
            why = "off its beat"
        elif shared > p.max_foreign:
            why = "other flashing on its spot"
        T.why = why or "named"
        T.code = best if not why else 0
        T.name = p.names.get(best, "code%d" % best) if not why else ""
        T.p_name = pb
        if T.name:
            T.named_name, T.named_t = T.name, T.last.a

    def _heading_from_light(self, T, L):
        pos = seg_pos(L)
        if not pos:
            return None
        vec = []
        for f, (pa, pb) in pos.items():
            fa, fb, mid, sep, ang = self._floor_pair(pa, pb)
            vec.append((fb[0] - fa[0], fb[1] - fa[1], mid[0], mid[1], math.hypot(pb[0] - pa[0], pb[1] - pa[1])))
        v = np.median(np.array(vec), axis=0)
        sep = math.hypot(v[0], v[1])
        axis = math.degrees(math.atan2(v[1], v[0]))
        mid = (float(v[2]), float(v[3]))
        sm = T.sep_med()
        if sm is not None and len(T.sep_cm) >= 3 and abs(sep - sm) > self.lp.sep_tol_cm:
            self.stats["sep_reject"] += 1
            return None
        T.sep_cm.append(sep)
        T.sep_px.append(float(v[4]))
        t = L.a
        fmid = L.frames[len(L.frames) // 2]
        img = self.frames.get(fmid)
        if T.h is None:
            h = wrap180(axis + 90.0)
            vote = self.appearance_vote(img, mid, h)
            if vote < 0:
                h, vote = wrap180(h + 180.0), -vote
            T.h, T.h_t, T.rate, T.votes = h, t, 0.0, vote
        elif (t - T.h_t) * self.lp.max_turn_dps > 75.0:
            # so long since the last flash that the robot may have turned round: the
            # earlier evidence only counts if this flash's picture clearly agrees
            h = self._pick(axis, T.h)
            vote = self.appearance_vote(img, mid, h)
            if vote >= 0.5:
                votes = 0.5 * T.votes + vote
            elif vote <= -0.3:
                h, votes = wrap180(h + 180.0), -vote
                self.stats["flips"] += 1
            else:
                votes = min(T.votes, 1.0) * 0.5 + max(vote, 0.0)
            T.h, T.h_t, T.rate, T.votes = h, t, 0.0, min(votes, self.lp.max_votes)
        else:
            pred = self._pred_heading(T, t)
            h = self._pick(axis, pred)
            if abs(wrap180(h - pred)) > self.lp.max_turn_jump_deg:
                # turned too far between flashes to follow: start the front/back evidence again
                h = self._pick(axis, T.h)
                vote = self.appearance_vote(img, mid, h)
                if vote < 0:
                    h, vote = wrap180(h + 180.0), -vote
                T.h, T.h_t, T.rate, T.votes = h, t, 0.0, vote
            else:
                dt = t - T.h_t
                if dt > 0.05:
                    T.rate = 0.5 * T.rate + 0.5 * wrap180(h - T.h) / dt
                vote = self.appearance_vote(img, mid, h)
                if T.last_mid is not None and 0.1 < t - T.last_mid[0] < 1.0:
                    dx, dy = mid[0] - T.last_mid[1], mid[1] - T.last_mid[2]
                    d = math.hypot(dx, dy)
                    if d > 3.0:
                        vote += self.lp.motion_vote * math.cos(math.atan2(dy, dx) - math.radians(h))
                votes = 0.9 * T.votes + vote
                if vote < -0.4:
                    votes = min(votes, self.lp.resolve_votes - 0.01)   # this flash disagrees: hold fixes back
                if votes < -0.5:
                    h = wrap180(h + 180.0)
                    votes = -votes
                    self.stats["flips"] += 1
                T.h, T.h_t, T.votes = h, t, min(votes, self.lp.max_votes)
        T.last_mid = (t, mid[0], mid[1])
        if DEBUG:
            print('DBG t %.2f tr%d axis %.1f h %.1f votes %.2f rate %.0f name %s' % (t, T.id, axis, T.h, T.votes, T.rate, T.name))
        return mid

    def _claim(self, o, t):
        """The name track o is, or may be, carrying: its name; else its best code if
        that is at least half likely; else the name it had in the last 3 s."""
        if o.name:
            return o.name
        d = o.dec
        if d and d["code"] and d["probs"].get(d["code"], 0.0) >= 0.5:
            return self.p.names.get(d["code"], "code%d" % d["code"])
        if o.named_name and t - o.named_t < 3.0:
            return o.named_name
        return ""

    def _contested(self, T, t):
        """Another live track somewhere else (> 6 cm away: not a second track on the
        same robot) that carries, or may carry, T's name: no fixes for that name."""
        if not T.name:
            return False
        x0, y0 = T.last.end_xy
        for o in self.tracks:
            if o is T or t - o.last.b > self.p.track_gap_s:
                continue
            x1, y1 = o.last.end_xy
            if math.hypot(x1 - x0, y1 - y0) <= 6.0 * self.p.px_per_cm:
                continue
            if self._claim(o, t) == T.name:
                return True
        return False

    def make_fix(self, T, fi, t, pa, pb, path):
        """Fix from one frame's two LED pixels, or None if they don't fit the track."""
        lp = self.lp
        fa, fb, mid, sep, axis = self._floor_pair(pa, pb)
        sm = T.sep_med()
        if sm is not None and abs(sep - sm) > lp.sep_tol_cm:
            self.stats["sep_reject"] += 1
            return None
        pred = self._pred_heading(T, t)
        h = self._pick(axis, pred)
        if abs(wrap180(h - pred)) > lp.max_turn_jump_deg:
            return None
        fwd, left = self.led_offset(T.name)
        c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
        x = mid[0] - (fwd * c - left * s)
        y = mid[1] - (fwd * s + left * c)
        sep_px = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
        sth = math.hypot(lp.sigma_th_deg, math.degrees(math.sqrt(2) * lp.led_px_sd / max(sep_px, 1.0)))
        return dict(type="fix", robot=T.name, t_pi=round(float(t), 4), x=round(x, 2), y=round(y, 2), th=round(h, 2),
                    sigma=lp.sigma_cm, sigma_th=round(sth, 2), id_p=round(T.p_name, 4), path=path, frame=fi,
                    track=T.id)

    def _sighting(self, T, L, mid):
        d = T.dec or {}
        probs = {self.p.names.get(k, "code%d" % k): round(v, 3) for k, v in (d.get("probs") or {}).items()}
        return dict(type="sighting", t_pi=round(float(L.ts[L.frames[0]]), 4), x=round(mid[0], 2), y=round(mid[1], 2),
                    track=T.id, best=self.p.names.get(d.get("code"), "") if d.get("code") else "",
                    probs=probs, p_none=round(d.get("p_none", 1.0), 3), why=T.why,
                    heading="resolved" if T.resolved else "unsure")

    # ------------------------------------------------------------ the frame step
    def push(self, img, t):
        """One frame (grayscale or BGR picture, Pi time t). Returns messages."""
        self.fi += 1
        fi = self.fi
        self.stats["frames"] += 1
        if img.ndim == 3:
            img = img[:, :, 0] if img.shape[2] == 1 else np.min(img, axis=2)
        self.frames[fi] = img
        self.frames.pop(fi - self.lp.frames_kept, None)
        ft = self.frame_t
        # a frame that came in late is followed by one that comes almost at once:
        # pull the late one's time back (as blink.repair_late_frames does offline)
        if len(ft) >= 2 and t - ft[-1] < 0.5 * (ft[-1] - ft[-2]) and fi >= 2:
            T0 = ft[-1] - ft[-2]
            ft[-1] = min(ft[-1], t - max(T0, 0.02))
        done = self.seg.push(fi, img, t)
        out = []
        if self.lp.fast:
            out += self._fast(fi, t)
        # LED pairs
        for s in done:
            s.done_t = t
        self.pending += [s for s in done if s.kind in ("pulse", "sync")]
        others = [s for s in done if s.kind == "long"]
        lights = []
        if self.pending:
            keep = []
            cand = B.pair_lights(self.pending, self.p, kinds=("pulse", "sync"))
            for L in cand:
                if L.paired or all(t - s.done_t >= 0.09 for s in L.segs):
                    lights.append(L)
                else:
                    keep += L.segs
            self.pending = keep
        # tracks
        for T in [T for T in self.tracks if t - T.last.b > self.p.track_gap_s]:
            self.tracks.remove(T)
            self.stats["fast_unconfirmed"] += sum(1 for fx in T.fast.values() if not fx.get("ok"))
        pulses = sorted([L for L in lights if L.kind == "pulse"], key=lambda L: L.a)
        for L in [L for L in lights if L.kind != "pulse"]:
            others += L.segs
        i = 0
        while i < len(pulses):
            batch = [pulses[i]]
            j = i + 1
            while j < len(pulses) and pulses[j].a - pulses[i].a <= 0.06:
                batch.append(pulses[j])
                j += 1
            i = j
            # which tracks (that carry or may carry a name) could each light belong to?
            rivals = {}
            for L in batch:
                rivals[id(L)] = [o for o in self.tracks if self._claim(o, t)
                                 and (B.join_cost(o, L, self.p) or 1e9) < B.NEW_TRACK_COST]
            res = B.assign_batch(self.tracks, batch, self.p, self._new_track)
            for L, T, isnew in res:
                if not isnew and any(o is not T and not self._same_robot(o, T) for o in rivals[id(L)]):
                    # it could have been another robot's flash (two robots near each other,
                    # flashing at the same moment): no fixes from T until it has had
                    # 3 flashes in a row that only it could have had
                    T.suspect = 3
                    self.stats["ambiguous"] = self.stats.get("ambiguous", 0) + 1
                elif not isnew and T.suspect:
                    T.suspect -= 1
            for L, T, isnew in res:
                if DEBUG:
                    print('DBGL a %.3f b %.3f paired %d track %d new %d xy %.1f,%.1f released_at %.3f' % (
                        L.a, L.b, L.paired, T.id, isnew, L.start_xy[0], L.start_xy[1], t))
                # flashing on other tracks' spots
                for o in self.tracks:
                    if o is not T and self._near(o, L.a, L.start_xy):
                        o.foreign += 1
                out += self._after_light(T, L, t)
        for s in others:
            for o in self.tracks:
                if self._near(o, s.a, (s.x[0], s.y[0])):
                    o.foreign += 1
        if pulses:
            self._merge_duplicates(t)
        # fast-path fixes the full check never backed up
        for T in self.tracks:
            for f in [f for f, fx in T.fast.items() if not fx.get("ok") and t - fx["t_pi"] > 0.8]:
                self.stats["fast_unconfirmed"] += 1
                del T.fast[f]
            for f in [f for f, fx in T.fast.items() if fx.get("ok") and t - fx["t_pi"] > 2.0]:
                del T.fast[f]
        return out

    def _merge_duplicates(self, t, radius_cm=5.0):
        """One robot's flashes split over two tracks (e.g. a flash seen with one LED
        started a new track, and the two then took turns): two live tracks whose
        lights are within radius_cm of each other (allowing for motion) are one
        robot (two robots' lights can't be that close), so merge the younger into
        the older and decode again. A track that merges a lamp's flashes loses its
        rhythm and its name: that errs on the safe side."""
        alive = [T for T in self.tracks if t - T.last.b <= 1.0]
        gone = set()
        for i, A in enumerate(alive):
            for Bt in alive[i + 1:]:
                if id(A) in gone or id(Bt) in gone:
                    continue
                old, new = (A, Bt) if A.id < Bt.id else (Bt, A)
                first, second = (old, new) if old.last.a <= new.last.a else (new, old)
                dt = second.last.a - first.last.b
                if dt > 1.0:
                    continue
                v = first.velocity() or (0.0, 0.0)
                x0, y0 = first.last.end_xy
                x1, y1 = second.last.start_xy
                if math.hypot(x1 - (x0 + v[0] * max(dt, 0.0)), y1 - (y0 + v[1] * max(dt, 0.0))) > radius_cm * self.p.px_per_cm:
                    continue
                lights = sorted(old.lights + new.lights, key=lambda L: L.a)
                old.lights = lights[-LiveTrack.KEEP:]
                old.n_lights += new.n_lights
                old.foreign += new.foreign
                old.fast.update(new.fast)
                if len(old.lights) >= 3:
                    old.dec = B.decode_rate([L.a for L in old.lights[-12:]], self.p)
                    old.quality = max(old.dec["probs"].values())
                    old.beat = self.p.period(old.dec["code"]) if old.quality >= 0.8 else None
                self._identity(old)
                self.tracks.remove(new)
                gone.add(id(new))
                self.stats["merged"] = self.stats.get("merged", 0) + 1

    def _same_robot(self, o, T):
        """Were o's last light and T's light before the newest within 6 cm (one robot
        seen as two tracks), rather than two robots (which can't be that close)?"""
        prev = T.lights[-2] if len(T.lights) >= 2 else T.last
        dx = o.last.end_xy[0] - prev.end_xy[0]
        dy = o.last.end_xy[1] - prev.end_xy[1]
        return math.hypot(dx, dy) <= 6.0 * self.p.px_per_cm

    def _crowded(self, T, t, xy, r_px):
        """Is another robot's light (a track that carries or may carry a name, not a
        second track on this robot) expected near xy now, so these spots could be its?"""
        tx, ty = T.last.end_xy
        for o in self.tracks:
            if o is T or t - o.last.b > self.p.track_gap_s or not self._claim(o, t):
                continue
            if math.hypot(o.last.end_xy[0] - tx, o.last.end_xy[1] - ty) <= 6.0 * self.p.px_per_cm:
                continue
            dt = t - o.last.b
            v = o.velocity() or (0.0, 0.0)
            x0, y0 = o.last.end_xy
            reach = r_px + 6.0 + 0.5 * self.lp.fast_amax_cms2 * self.p.px_per_cm * dt * dt
            if math.hypot(xy[0] - (x0 + v[0] * dt), xy[1] - (y0 + v[1] * dt)) <= reach:
                return True
        return False

    def _near(self, T, t, xy, radius_cm=3.0):
        L = T.last
        dt = t - L.b
        if dt < -0.3 or dt > 1.5:
            return False
        v = T.velocity() or (0.0, 0.0)
        x0, y0 = L.end_xy
        px, py = x0 + v[0] * max(dt, 0.0), y0 + v[1] * max(dt, 0.0)
        return math.hypot(xy[0] - px, xy[1] - py) <= radius_cm * self.p.px_per_cm

    def _after_light(self, T, L, t_now):
        """A flash was added to track T: identity, heading, late fixes."""
        self._identity(T)
        out = []
        mid = self._heading_from_light(T, L) if L.paired else None
        for f in L.frames:
            if f in T.fast:
                T.fast[f]["ok"] = True
        if not T.name or not T.resolved or T.suspect or self._contested(T, t_now):
            if T.name and self._contested(T, t_now):
                self.stats["contested"] += 1
            elif T.name:
                self.stats["unresolved"] += 1
            if mid is not None and T.n_lights >= self.p.min_pulses:
                self.stats["sightings"] += 1
                out.append(self._sighting(T, L, mid))
            return out
        for f, (pa, pb) in sorted(seg_pos(L).items()):
            if f in T.fast:
                continue
            fx = self.make_fix(T, f, self.frame_t[f], pa, pb, "late")
            if fx:
                out.append(fx)
                self.stats["fixes"] += 1
                self.stats["late"] += 1
        return out

    def _fast(self, fi, t):
        """Fixes from this very frame for robots whose flash is due now."""
        p, lp = self.p, self.lp
        spots = self.seg.spots
        if not spots:
            for T in self.tracks:
                T.fast_open = None
            return []
        core = self.seg.core
        prev = self.seg.cores.get(fi - 1)
        claims = {}
        for T in self.tracks:
            if not (T.name and T.resolved and T.beat and T.last.paired) or T.suspect or self._contested(T, t):
                T.fast_open = None
                continue
            spm = T.sep_px_med()
            if spm is None:
                continue
            op = T.fast_open
            if op is not None and fi - op[0] <= 2 and t - op[4] <= p.max_pulse_s:
                # the flash goes on: the LEDs are where they were a frame ago
                dt = t - op[1]
                v = T.velocity() or (0.0, 0.0)
                preds = [(q[0] + v[0] * dt, q[1] + v[1] * dt) for q in (op[2], op[3])]
                gate = 3.0 + 0.3 * p.vmax_px * dt
                first = False
            else:
                L = T.last
                pos = seg_pos(L)
                if not pos:
                    continue
                fl = max(pos)
                tl = self.frame_t[fl]
                dt = t - tl
                if dt <= 0.05 or dt > 1.2 * T.beat + 0.1:     # a flash missed: leave it to the full check
                    T.fast_open = None
                    continue
                r = (t - L.a) % T.beat           # where in its beat this frame is
                if not (r <= min(0.25 + p.delay_s, T.beat - 0.05) or r >= T.beat - p.tol_s):
                    T.fast_open = None
                    continue
                v = T.velocity() or (0.0, 0.0)
                sp = math.hypot(*v)
                if sp > p.vmax_px:
                    v = (v[0] * p.vmax_px / sp, v[1] * p.vmax_px / sp)
                    sp = p.vmax_px
                preds = [(q[0] + v[0] * dt, q[1] + v[1] * dt) for q in pos[fl]]
                gate = 4.0 + 0.5 * lp.fast_amax_cms2 * p.px_per_cm * dt * dt + 0.3 * sp * dt
                first = True
            best = None
            for i, (x1, y1, _) in enumerate(spots):
                d1 = math.hypot(x1 - preds[0][0], y1 - preds[0][1])
                if d1 > gate:
                    continue
                for j, (x2, y2, _) in enumerate(spots):
                    if j == i:
                        continue
                    d2 = math.hypot(x2 - preds[1][0], y2 - preds[1][1])
                    if d2 > gate:
                        continue
                    sep = math.hypot(x2 - x1, y2 - y1)
                    if abs(sep - spm) > 0.2 * spm:
                        continue
                    if best is None or d1 + d2 < best[0]:
                        best = (d1 + d2, i, j)
            if best is None:
                T.fast_open = None
                continue
            _, i, j = best
            pa, pb = spots[i][:2], spots[j][:2]
            if first and self._crowded(T, t, ((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2), gate + spm):
                T.fast_open = None
                continue
            lit = [B.sample(core, q[0], q[1]) for q in (pa, pb)]
            if None in lit:
                T.fast_open = None
                continue
            if first:
                # it must have switched on: clearly darker there a frame ago
                if prev is None:
                    continue
                before = [B.sample(prev, q[0], q[1]) for q in (pa, pb)]
                if None in before or any(n - b < p.swing_min for n, b in zip(lit, before)):
                    T.fast_open = None
                    continue
                ref = (lit, [n - b for n, b in zip(lit, before)], t, (spots[i][2], spots[j][2]))
            else:
                # still lit: about as bright as when it switched on (not the robot's top once the LEDs are off)
                ref = op[5]
                resp = (spots[i][2], spots[j][2])
                if (any(n < l0 - max(p.swing_min, 0.25 * sw) for n, l0, sw in zip(lit, ref[0], ref[1]))
                        or any(r1 < 0.5 * r0 for r1, r0 in zip(sorted(resp), sorted(ref[3])))):
                    T.fast_open = None
                    continue
            claims.setdefault((i, j) if i < j else (j, i), []).append((T, pa, pb, ref))
        out = []
        for key, who in claims.items():
            if len(who) != 1:
                continue          # two robots want the same spots: neither
            T, pa, pb, ref = who[0]
            T.fast_open = (fi, t, pa, pb, ref[2], ref)
            fx = self.make_fix(T, fi, t, pa, pb, "fast")
            if fx:
                T.fast[fi] = dict(t_pi=t)
                out.append(fx)
                self.stats["fixes"] += 1
                self.stats["fast"] += 1
        return out

    def summary_tracks(self):
        return [dict(track=T.id, name=T.name, why=T.why, p=round(T.p_name, 3), flashes=T.n_lights,
                     heading=None if T.h is None else round(T.h, 1), votes=round(T.votes, 2))
                for T in self.tracks]
