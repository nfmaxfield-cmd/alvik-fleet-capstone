#!/usr/bin/env python3
"""Multi-object tracker: turns per-frame guesses into numbered objects that
keep their number from frame to frame.

Rules, in plain words:
  - Gating. A robot can only move so far between two frames: at most
    max_speed x time since it was last seen, plus a margin, and never more
    than max_gate robot sizes. A look-alike further away than that cannot
    take over an existing track; it starts a new, tentative track instead.
  - Confirming. A new track has to be seen in confirm_n frames (with at most
    tentative_misses missed frames in a row) before it is shown. Once
    confirmed, its earlier tentative frames are filled in too.
  - Coasting. When a confirmed track is not detected (a robot stopped long
    enough to blend into the background, or two robots merged into one blob),
    it stays where it was last seen. Its score drops the longer it is unseen,
    so those frames come up for review.
  - Ending. A track ends only after a long absence (max_absence_s); hands and
    tracks last seen at the picture edge end after a short absence.
  - Heading. From the motion of the track (see motion.HeadingEstimator); a
    stopped robot keeps the heading it had when it last moved. A detector
    that gives its own heading (a trained model) is trusted instead.
  - User anchors. On a frame the user has labelled, the user's objects
    replace the tracker's: positions, classes, headings and ids are set from
    the user, tracks the user did not keep are ended, objects the user added
    start new tracks. Everything after that frame follows from there, so one
    correction fixes the frames that follow.
  - Rejected spots. Where the user rejected a guess, a new track that starts
    on that spot stays hidden until it moves away from it, so a stain or a
    reflection does not come back every few frames.

Output per frame per track: obj, cls, x, y, heading_deg, heading_known,
heading_guess, score, source (motion | model | track | user), where source
'track' means the tracker filled the position in without a detection.

    python tools/label/tracker.py <recording folder>    # prints a summary
"""
import argparse
import collections
import copy
import math
import os
import sys

from motion import HeadingEstimator, Params as MotionParams, wrap360


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
        for k, v in kw.items():
            if not hasattr(self, k):
                raise TypeError("unknown setting: %s" % k)
            setattr(self, k, v)


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
        self.pos_hist = collections.deque()
        self.rows = []                 # output rows, one per frame while alive
        self.matched_now = False
        self.anchored_now = False
        self.det_source = "motion"
        self.votes[cls] += 0.01
        self.voted = cls               # class with the most votes so far

    @property
    def cls(self):
        return self.cls_locked or self.voted

    def clone(self, with_rows=True):
        """Independent copy (the row list is copied, the rows themselves are shared)."""
        c = copy.copy(self)
        c.est = self.est.clone()
        c.pos_hist = collections.deque(self.pos_hist)
        c.votes = collections.Counter(self.votes)
        c.rows = list(self.rows) if with_rows else []
        return c

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
    """Frame-by-frame tracker. step() once per frame, in time order."""

    def __init__(self, robot_px, img_size=(640, 480), params=None, motion_params=None,
                 reserved_ids=(), det_source="motion"):
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
        gate = self.p.max_speed * self.r * dt + self.p.margin * self.r
        return min(gate, max(self.p.max_gate, self.p.margin) * self.r)

    def step(self, f, t, dets, anchor=None):
        """Process one frame. dets: list of detection dicts. anchor: optional list
        of the user's objects for this frame. Returns rows for confirmed tracks.
        dets=None means the detector was not run on this frame (a model run on
        every n-th frame only): tracks are carried along without counting a miss."""
        looked = dets is not None
        dets = [d for d in (dets or []) if d.get("cls") != "none"]
        unmatched = set(range(len(dets)))
        for tr in self.tracks:
            tr.matched_now = tr.anchored_now = False
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
                        cost = dist / gate
                        dc = d.get("cls", "alvik")
                        if dc != tr.cls and "unknown" not in (dc, tr.cls):
                            cost += self.p.class_penalty
                        pairs.append((cost, ti, di))
            for ti, di in _greedy_pairs(pairs):
                group[ti].update(dets[di], f, t, dets[di].get("source", self.det_source))
                unmatched.discard(di)
        # 2. Tracks without a detection stay where they are.
        for tr in self.tracks:
            if not tr.matched_now:
                tr.coast(f, t, looked)
        # 3. Leftover detections start tentative tracks (not right on top of a confirmed one).
        for di in sorted(unmatched):
            d = dets[di]
            near = any(tr.confirmed and math.hypot(d["x"] - tr.x, d["y"] - tr.y) < self.p.no_new_near * self.r
                       for tr in self.tracks)
            if near:
                continue
            tr = Track(self._new_id(), float(d["x"]), float(d["y"]), d.get("cls", "alvik"), f, t, self.r, self.mp)
            tr.update(d, f, t, d.get("source", self.det_source))
            # Born where the user rejected a guess before: keep it hidden
            # unless it moves away (a real robot would; a stain will not).
            for (sx, sy) in self.rejected_spots:
                if math.hypot(tr.x - sx, tr.y - sy) < 0.7 * self.r:
                    tr.spot = (sx, sy)
            self.tracks.append(tr)
        # 4. Confirm new tracks, drop failed tentative ones, end long-lost ones.
        keep = []
        for tr in self.tracks:
            if not tr.confirmed:
                if tr.spot is not None and math.hypot(tr.x - tr.spot[0], tr.y - tr.spot[1]) >= self.r:
                    tr.spot = None                 # moved off the rejected spot: a real object
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
        # 6. Output rows. Rows are final once made (never changed later), so a
        #    re-run can reuse the rows of the frames before it.
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
        return out

    def _row(self, tr, f, t):
        p = self.p
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
        else:
            absent = t - tr.t_last
            score = tr.last_score * max(0.25, math.exp(-absent / p.coast_decay_s)) * 0.85
            source = "track"
        if not tr.confirmed:
            score *= 0.8            # shown only if the track is confirmed later
        heading_known = tr.h_status == "known" and tr.heading is not None and tr.cls in p.front_classes
        guess = tr.heading if (tr.h_status == "guess" and tr.cls in p.front_classes) else None
        return dict(frame=f, obj=tr.id, cls=tr.cls, x=round(tr.x, 1), y=round(tr.y, 1),
                    heading_deg=round(tr.heading, 1) if heading_known else None,
                    heading_known=bool(heading_known),
                    heading_guess=round(guess, 1) if guess is not None else None,
                    score=round(min(1.0, max(0.0, score)), 3), source=source, close=False)

    def _apply_anchor(self, f, t, objs):
        """Make the tracker agree with the user's labels on this frame."""
        for o in objs:
            if o.get("status") == "rejected" and o.get("x") is not None:
                spot = (float(o["x"]), float(o["y"]))
                if spot not in self.rejected_spots:
                    self.rejected_spots.append(spot)
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
                    spots=list(self.rejected_spots))

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
    """

    def __init__(self, dets_by_frame, times, robot_px, img_size=(640, 480), params=None,
                 motion_params=None, det_source="motion", snap_every=150):
        self.dets = dets_by_frame
        self.times = times
        self.args = (robot_px, img_size, params, motion_params)
        self.det_source = det_source
        self.snap_every = snap_every
        self.snaps = {}

    def run(self, anchors=None, reserved_ids=(), from_frame=0):
        """Track from from_frame on (earlier frames are reused). Returns
        (rows by frame, track summary)."""
        anchors = anchors or {}
        robot_px, img_size, params, motion_params = self.args
        tk = Tracker(robot_px, img_size, params, motion_params, reserved_ids, self.det_source)
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
        rows = tk.all_rows()
        for f in rows:
            rows[f].sort(key=lambda r: r["obj"])
        self.max_id = tk.next_id - 1       # highest object number handed out (shown or not)
        return dict(rows), tk.summary()


def run_tracker(dets_by_frame, times, robot_px, img_size=(640, 480), anchors=None, reserved_ids=(),
                params=None, motion_params=None, det_source="motion"):
    """Track a whole recording in one go.

    dets_by_frame: list (one entry per frame) of lists of detection dicts
    (None for a frame the detector was not run on).
    times: frame times in seconds (same length). anchors: {frame: [user objects]}.
    Returns (rows by frame: dict frame -> list of rows, track summary).
    """
    return TrackRun(dets_by_frame, times, robot_px, img_size, params, motion_params, det_source).run(
        anchors, reserved_ids)


def main():
    import motion
    ap = argparse.ArgumentParser(description="Track the motion detector's guesses over a recording.")
    ap.add_argument("recording", help="recording folder with video.avi")
    args = ap.parse_args()
    video = os.path.join(args.recording, "video.avi")
    dets, info = motion.detect_recording(video, progress=lambda d, n: print("  %d / %d frames" % (d, n), end="\r"))
    print()
    fps = info["fps"]
    rows, summary = run_tracker(dets, [f / fps for f in range(len(dets))], info["robot_px"])
    for s in summary:
        print("obj %3d  %-7s frames %6d-%6d" % (s["obj"], s["cls"], s["first"], s["last"]))


if __name__ == "__main__":
    sys.exit(main())
