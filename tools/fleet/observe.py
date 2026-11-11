#!/usr/bin/env python3
"""Observe, then trust (CLAUDE.md standing rule, Nick 5 Nov 2026): the shared checks every run type uses, so a belief
about the physical setup is a starting guess that live evidence corrects. Built after the school runs of 5 Nov, where
each of these was a typed setting that turned out wrong:

  heading       the roof tag's heading against the robot's nose: a short nudge's direction of motion in the picture
                (heading_offset), then checked on every straight drive for the rest of the run (HeadingWatch). AL-1DC0's
                tag was mounted 180 deg off: every leg ran backwards.
  readability   where the camera actually reads roof tags (ReadMap): read rates per 20 cm cell, seeded by Measure
                (the reference tags read / not read) and the camera model (a tag's size in pixels), kept per site and
                updated by every run. Points where tags do not read are skipped with a reason; the run carries on.
                School: the tag read to about x 100 cm in room light; a typed --view-x-max was needed.
  sharing       the table split between however many robots were found (split_points), over the readable area only;
                the fixed halves stay behind a switch.
  cameras       a single plugged camera is the overhead whatever its name (pick_cameras); names only tell two apart.
                School: the code looked for home's camera names.
  edge guard    a line-sensor trip that neither the bottom zone nor the camera confirms is a surface mark (tape over a
                table seam), logged and not an edge (edge_trip_kind).
  restarts      a robot whose boot id changed or whose uptime went back has restarted: it gets its compass check
                (restarted).
  sleep         robots stay awake at the end of a run; they sleep only after a long idle (IDLE_SLEEP_MIN), so values
                that are not saved on the robot are not wiped by the restart a wake causes.
  detection     a tag not found at full size is looked for again in a downscaled picture (detect_retry). School: a
                downscaled frame read the small tag when the full frame did not.

Every function here is pure (no sockets, no robots) and tested in tests/test_observe.py.
"""
import json
import math
import os

VERSION = "2026-11-11e16"   # e16 (home 11 Nov 08:45, Nick: "it can still back up but it's not, it's just still"): track_back_target, a reverse along the robot's own recorded track (curved too: straight chords that stay on the track) as far as the edge rule needs, derived; moved_in_evidence (a held robot counts as moved in only on a lift / tilt or a sustained camera move beyond its own measured noise). Before: "2026-11-10a"   # 11-10a (e12): blind_cells (the spots learned blind, for the site's blind_areas.json). 11-07a (e08): tilt limits from safety_rules (re-picked from the 5 / 6 Nov logs), trip
                          # keep-outs marked confirmed (the one keep-out rule)
import sys as _sys
if os.path.dirname(os.path.abspath(__file__)) not in _sys.path:
    _sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import safety_rules as _SR   # noqa: E402  (e08: the rules shared by every run type)

# ------------------------------------------------------------------------------------------------ chosen numbers
SNAP_DEG = 90.0          # a roof tag is stuck on square to the robot: the measured offset is snapped to a quarter turn
SNAP_TOL_DEG = 20.0      # ... when within this of one (else the measured value is used). Chosen as calib_run 11-05f
NUDGE_CM = 6.0           # the heading nudge's length (calib_run HEADING_CHECK_CM): far beyond the camera's ~0.5 cm
                         # noise, short enough that a wrong guess drives 6 cm the wrong way. DOE note: 4 / 6 / 10 cm
                         # against wrong offsets found (simulator: tag noise 0.3-1.5 cm)
NUDGE_MIN_FRAC = 0.5     # the nudge must move the tag at least half its length, else nothing is concluded
WATCH_SEG_CM = 8.0       # HeadingWatch: a straight stretch this long (camera) is one check of the heading
WATCH_STRAIGHT_DEG = 6.0  # ... straight = the robot's own heading changed less than this over it
WATCH_BAD_DEG = 45.0     # one check disagrees when the motion direction is this far from the expected one
WATCH_CONFIRM = 2        # this many disagreeing checks in a row, agreeing with each other, re-learn the offset.
                         # Chosen: one check can be a bump or a slip; two in a row are not. DOE note: 1 / 2 / 3 against
                         # false re-learns (simulator, tag noise) and centimetres driven wrong before a re-learn
CELL_CM = 20.0           # ReadMap's cell. Chosen: about two robot bodies. DOE note: 10 / 20 / 40 cm against points
                         # wrongly skipped / wrongly kept on the school table
READ_OK = 0.6            # a cell is readable when its read rate (prior + evidence) is at least this
PRIOR_N = 4.0            # the seed counts as this many tries (evidence of 4+ tries outweighs it)
TAG_PX_MIN = 14.0        # a roof tag reads when its side is at least this many pixels (tag25h9, 5 x 5 bits + border:
                         # 7 cells; 2 px a cell). From the 5 Nov school frames: the 8 cm tag read to x ~100 cm (~18 px)
                         # and failed past ~150 cm (~12 px). Chosen between; DOE note: 12 / 14 / 16 px against the live
                         # read rates kept in readmap.json
PIC_MARGIN_TAGS = 5.0    # a cell reads only when the tag is this many of its own sizes inside every picture border.
                         # Chosen after the rehearsals (3 lost robots at the picture's corner). DOE note: 3 / 5 / 8 against
                         # points skipped vs robots lost, from the run's own readmap.json
IDLE_SLEEP_MIN = 30.0    # robots sleep after this long with no run (Mission Control's idle sleeper). Chosen: longer
                         # than a break between runs, short enough to save the battery over an evening. DOE note: not
                         # needed (it trades battery against a restart); the battery log over an idle evening gives it
SURFACE_MARK_CAM_CM = 8.0  # a line trip with the camera putting the robot's middle further than this from every edge
                         # is not an edge (body half length 6 cm + 2 cm of camera error)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


# =============================================================================================== heading
def snap(off, step=SNAP_DEG, tol=SNAP_TOL_DEG):
    """The offset snapped to a quarter turn when within tol of one, else as measured (both wrapped to +-180)."""
    s = round(off / step) * step
    return wrap180(s) if abs(wrap180(off - s)) <= tol else wrap180(off)


def heading_offset(p0, p1, sign=1.0, min_cm=NUDGE_CM * NUDGE_MIN_FRAC):
    """From a straight drive (sign +1 forward, -1 backward) the camera saw from p0 to p1 ((x, y, tag heading deg)):
    (offset deg to ADD to the tag's heading to get the nose's, snapped; raw offset; moved cm), or None when it moved
    under min_cm (nothing concluded)."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    mv = math.hypot(dx, dy)
    if mv < min_cm:
        return None
    motion = math.degrees(math.atan2(dy, dx))
    if sign < 0:
        motion += 180.0
    raw = wrap180(motion - p0[2])
    return snap(raw), raw, mv


def nudge_clear(pose, problems, d=NUDGE_CM):
    """True when a nudge of d cm is clear both ways (the tag's heading and its opposite: a wrong guess cannot take it
    over an edge). problems(a, b) -> list of reasons for the segment a-b (empty = clear). Also tries the two sides
    (90 / 270) so a robot facing an edge can nudge sideways after a turn: returns (ok, heading_to_use)."""
    for base in (0.0, 90.0):
        ok = True
        for flip in (0.0, 180.0):
            h = math.radians(pose[2] + base + flip)
            q = (pose[0] + d * math.cos(h), pose[1] + d * math.sin(h))
            if problems(pose[:2], q):
                ok = False
                break
        if ok:
            return True, base
    return False, None


class HeadingWatch:
    """The tag's heading checked against the motion on every straight stretch, for one robot, all run long.
    feed(t, cam (x, y, tag heading deg), own_heading deg or None, sign +1 forward / -1 backward / 0 not driving)
    returns None or an event dict: {"kind": "heading_ok" | "heading_bad" | "heading_relearned", ...}. offset is the
    current correction (added to the tag's heading); a re-learn changes it."""

    def __init__(self, offset=0.0, seg_cm=WATCH_SEG_CM):
        self.offset = float(offset)
        self.seg_cm = seg_cm
        self.start = None
        self.own0 = None
        self.sign = 0
        self.bad = []
        self.checks = 0
        self.disagree = 0

    def reset(self):
        self.start, self.own0, self.sign = None, None, 0

    def feed(self, t, cam, own_heading=None, sign=1):
        if sign == 0 or cam is None:
            self.reset()
            return None
        if self.start is None or sign != self.sign:
            self.start, self.own0, self.sign = cam, own_heading, sign
            return None
        if own_heading is not None and self.own0 is not None and \
                abs(wrap180(own_heading - self.own0)) > WATCH_STRAIGHT_DEG:
            self.start, self.own0 = cam, own_heading          # turned: a new stretch
            return None
        dx, dy = cam[0] - self.start[0], cam[1] - self.start[1]
        mv = math.hypot(dx, dy)
        if mv < self.seg_cm:
            return None
        motion = math.degrees(math.atan2(dy, dx)) + (180.0 if sign < 0 else 0.0)
        # the nose as the camera reads it (tag heading + offset), averaged over the stretch's two ends
        h0, h1 = self.start[2] + self.offset, cam[2] + self.offset
        nose = h0 + wrap180(h1 - h0) / 2.0
        err = wrap180(motion - nose)
        self.start, self.own0 = cam, own_heading
        self.checks += 1
        if abs(err) <= WATCH_BAD_DEG:
            self.bad = []
            return {"kind": "heading_ok", "err_deg": round(err, 1), "moved_cm": round(mv, 1)}
        self.disagree += 1
        self.bad.append(snap(err))
        ev = {"kind": "heading_bad", "err_deg": round(err, 1), "moved_cm": round(mv, 1), "in_a_row": len(self.bad)}
        if len(self.bad) >= WATCH_CONFIRM and all(abs(wrap180(b - self.bad[-1])) < 30 for b in self.bad[-WATCH_CONFIRM:]):
            old = self.offset
            self.offset = wrap180(self.offset + self.bad[-1])
            self.bad = []
            return {"kind": "heading_relearned", "old_deg": old, "new_deg": self.offset, "err_deg": round(err, 1),
                    "moved_cm": round(mv, 1)}
        return ev


# =============================================================================================== readability
class ReadMap:
    """Where roof tags read: a grid of CELL_CM cells over the table, each with tries and reads (a try = a moment a
    robot's tag should have been read there; a read = it was). The seed (Measure, the camera model, an earlier run)
    counts as PRIOR_N tries at its own rate. readable(x, y) -> (bool, words)."""

    def __init__(self, L, W, cell=CELL_CM, x0=0.0, y0=0.0):
        self.L, self.W, self.cell, self.x0, self.y0 = float(L), float(W), float(cell), float(x0), float(y0)
        self.nx = max(1, int(math.ceil(self.L / self.cell)))
        self.ny = max(1, int(math.ceil(self.W / self.cell)))
        self.prior = {}          # (i, j) -> (rate, weight, why)
        self.tries = {}
        self.reads = {}
        self.cameras = 1

    def ij(self, x, y):
        i = int((x - self.x0) // self.cell)
        j = int((y - self.y0) // self.cell)
        return min(max(i, 0), self.nx - 1), min(max(j, 0), self.ny - 1)

    def centre(self, i, j):
        return self.x0 + (i + 0.5) * self.cell, self.y0 + (j + 0.5) * self.cell

    def seed(self, fn, why, weight=PRIOR_N):
        """fn(x, y) -> a read rate 0-1 (or None: no opinion) for each cell's middle."""
        for i in range(self.nx):
            for j in range(self.ny):
                r = fn(*self.centre(i, j))
                if r is None:
                    continue
                old = self.prior.get((i, j))
                if old is None or weight >= old[1]:
                    self.prior[(i, j)] = (float(r), float(weight), why)

    def seed_points(self, pts, rate, why, radius=None, weight=PRIOR_N):
        """Cells within radius (default one cell) of each (x, y) get this rate: e.g. a reference tag Measure read (1.0)
        or did not read (0.0)."""
        rad = radius if radius is not None else self.cell
        for (px, py) in pts:
            for i in range(self.nx):
                for j in range(self.ny):
                    cx, cy = self.centre(i, j)
                    if math.hypot(cx - px, cy - py) <= rad:
                        old = self.prior.get((i, j))
                        if old is None or weight >= old[1]:
                            self.prior[(i, j)] = (float(rate), float(weight), why)

    def add(self, x, y, read, n=1):
        k = self.ij(x, y)
        self.tries[k] = self.tries.get(k, 0) + n
        if read:
            self.reads[k] = self.reads.get(k, 0) + n

    def rate(self, x, y):
        k = self.ij(x, y)
        p = self.prior.get(k)
        pr, pw = (p[0], p[1]) if p else (0.5, 1.0)       # no seed: a weak 50/50
        n, r = self.tries.get(k, 0), self.reads.get(k, 0)
        return (pr * pw + r) / (pw + n)

    def readable(self, x, y, ok=READ_OK):
        k = self.ij(x, y)
        r = self.rate(x, y)
        n = self.tries.get(k, 0)
        p = self.prior.get(k)
        why = "%s%s" % (p[2] if p else "no seed", "; %d tries here, %d read" % (n, self.reads.get(k, 0)) if n else "")
        return r >= ok, "read rate %.0f %% (%s)" % (100 * r, why)

    def area_cm2(self, ok=READ_OK):
        return sum(self.cell ** 2 for i in range(self.nx) for j in range(self.ny)
                   if self.rate(*self.centre(i, j)) >= ok)

    def grow(self, cams):
        """A second camera appeared (or went): the seed is redone by the caller; this records the count."""
        self.cameras = cams

    def to_json(self):
        return {"version": VERSION, "L": self.L, "W": self.W, "cell": self.cell, "x0": self.x0, "y0": self.y0,
                "cameras": self.cameras,
                "cells": [[i, j, self.tries.get((i, j), 0), self.reads.get((i, j), 0)]
                          for i in range(self.nx) for j in range(self.ny) if self.tries.get((i, j))],
                "prior": [[i, j, r, w, why] for (i, j), (r, w, why) in sorted(self.prior.items())]}

    @classmethod
    def from_json(cls, d):
        m = cls(d["L"], d["W"], d.get("cell", CELL_CM), d.get("x0", 0.0), d.get("y0", 0.0))
        m.cameras = d.get("cameras", 1)
        for i, j, n, r in d.get("cells") or []:
            m.tries[(i, j)], m.reads[(i, j)] = n, r
        for i, j, r, w, why in d.get("prior") or []:
            m.prior[(i, j)] = (r, w, why)
        return m

    def save(self, path):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.to_json(), f)
        os.replace(tmp, path)

    def merge_history(self, old, decay=0.5):
        """An earlier run's counts (same table and cameras) carried in at half weight: the room's light changes."""
        if not old or abs(old.L - self.L) > 5 or abs(old.W - self.W) > 5 or old.cell != self.cell or \
                old.cameras != self.cameras:
            return False
        for k, n in old.tries.items():
            self.tries[k] = self.tries.get(k, 0) + int(round(n * decay))
            self.reads[k] = self.reads.get(k, 0) + int(round(old.reads.get(k, 0) * decay))
        return True

    def text(self, ok=READ_OK):
        """A small picture in characters (# readable, . not, ? unknown), y up, for the run's log."""
        rows = []
        for j in reversed(range(self.ny)):
            s = ""
            for i in range(self.nx):
                k = (i, j)
                r = self.rate(*self.centre(i, j))
                s += "?" if (k not in self.prior and not self.tries.get(k)) else ("#" if r >= ok else ".")
            rows.append(s)
        return "\n".join(rows)


def tag_px(model_project, x, y, z, tag_cm):
    """A tag's side in pixels at (x, y) for a camera: model_project([[x, y, z], ...]) -> pixel points (N, 2)."""
    h = tag_cm / 2.0
    p = model_project([[x - h, y, z], [x + h, y, z], [x, y - h, z], [x, y + h, z]])
    try:
        a = math.hypot(p[1][0] - p[0][0], p[1][1] - p[0][1])
        b = math.hypot(p[3][0] - p[2][0], p[3][1] - p[2][1])
    except (TypeError, IndexError):
        return None
    return min(a, b)


BLIND_MIN_TRIES = 3     # (e12) a cell is "learned blind" after this many tries with its rate under READ_OK. Chosen
                        # (calib_run's own skip rule uses 3 tries). DOE: no (a reporting threshold)


def blind_cells(rm, min_tries=BLIND_MIN_TRIES, ok=READ_OK):
    """(e12) The cells the robots' runs found the camera does not read in practice: [[x0, y0, x1, y1, tries, reads]]."""
    out = []
    for (i, j), n in sorted(rm.tries.items()):
        if n >= min_tries and rm.rate(*rm.centre(i, j)) < ok:
            x0, y0 = rm.x0 + i * rm.cell, rm.y0 + j * rm.cell
            out.append([round(x0, 1), round(y0, 1), round(x0 + rm.cell, 1), round(y0 + rm.cell, 1), n,
                        rm.reads.get((i, j), 0)])
    return out


LEARN_READS_PER_CELL = 20   # (e12) a run's camera fixes counted per cell at most (a robot standing in one cell for minutes
                            # must not swamp the map). Chosen; no DOE (a cap)
LEARN_STOP_READS = 10       # (e12) a stop with camera fixes counts as this many reads at most. Chosen; no DOE


def learn_from_logs(rm, folder):
    """(e12, 9 Nov school evidence: the seed flagged P03, P05, P12, P13, P15, P18, P19, P20 unreadable, but in 7 tours
    every one of the 20 stops had camera fixes) The run's own evidence into the map, so it can overturn the cautious
    seed: every camera_track.csv fix is a read where it was (at most LEARN_READS_PER_CELL per cell), every stops.csv stop
    with n_fixes > 0 is up to LEARN_STOP_READS reads at the camera's spot, a stop with none a try that did not read, at
    its point. Returns (fixes used, stops used)."""
    import csv as _csv
    import glob as _glob
    per, nf, ns = {}, 0, 0
    for p in _glob.glob(os.path.join(folder, "**", "camera_track.csv"), recursive=True):
        try:
            with open(p, newline="") as f:
                for row in _csv.DictReader(f):
                    try:
                        x, y = float(row.get("x_cm") or row.get("x")), float(row.get("y_cm") or row.get("y"))
                    except (TypeError, ValueError):
                        continue
                    k = rm.ij(x, y)
                    if per.get(k, 0) >= LEARN_READS_PER_CELL:
                        continue
                    per[k] = per.get(k, 0) + 1
                    rm.add(x, y, True)
                    nf += 1
        except OSError:
            continue
    for p in _glob.glob(os.path.join(folder, "**", "stops.csv"), recursive=True):
        try:
            with open(p, newline="") as f:
                for row in _csv.DictReader(f):
                    try:
                        n = int(float(row.get("n_fixes") or 0))
                        x, y = float(row.get("cam_x_cm") or row.get("own_x_cm")), float(row.get("cam_y_cm") or
                                                                                         row.get("own_y_cm"))
                    except (TypeError, ValueError):
                        continue
                    if n > 0:
                        rm.add(x, y, True, n=min(n, LEARN_STOP_READS))
                    else:
                        rm.add(x, y, False)
                    ns += 1
        except OSError:
            continue
    return nf, ns


def seed_from_cameras(rm, cams, tag_cm, z_cm, px_min=TAG_PX_MIN):
    """Seed rm from camera models: cams [(project, (W, H))]. A cell reads when some camera sees it inside its picture
    with the tag at least px_min pixels (rate 0.9), else 0.1 (the seed is only a guess; reads overrule it)."""
    def fn(x, y):
        best = 0.0
        for project, (Wpx, Hpx) in cams:
            try:
                c = project([[x, y, z_cm]])[0]
            except Exception:  # noqa: BLE001
                continue
            if c is None:
                continue
            s = tag_px(project, x, y, z_cm, tag_cm) or 0.0
            # the whole roof (and a robot's move round it) inside the picture: a margin of EDGE_TAGS tag sizes from every
            # border (rehearsal s1: tags at 3 sizes from the picture's corner were lost; the robot ended unseeable)
            m = PIC_MARGIN_TAGS * s
            if not (m <= c[0] < Wpx - m and m <= c[1] < Hpx - m):
                continue
            best = max(best, s)
        return 0.9 if best >= px_min else 0.1
    rm.seed(fn, "camera model: a tag needs %.0f px to read" % px_min)
    rm.grow(len(cams))
    return rm


# =============================================================================================== sharing the table
def split_points(points, robots, readable=None, mode="auto", zones=None, halves=None, labels=None):
    """Share the points between the robots found. points {label: (x, y)}; robots {name: (x, y) where it stands};
    readable(x, y) -> (bool, words) or None (all readable). mode "auto": only the readable points, cut along x (the
    table's length, the way the cameras split it) into len(robots) runs of equal count, each run given to the robot
    standing nearest its middle (every pairing tried for up to 6 robots); "halves": the old fixed split (halves:
    {name: [labels]}), kept behind the switch. Returns (orders {name: [labels]}, skipped {label: reason}, words)."""
    names = list(robots)
    skipped, words = {}, []
    labs = list(labels or points)
    if mode == "halves" and halves:
        out = {n: [l for l in halves.get(n, []) if l in points] for n in names}
        for n in halves:
            if n not in robots:
                for l in halves[n]:
                    skipped[l] = "its robot (%s) is not in this run" % n
        if readable:
            for n in out:
                keep = []
                for l in out[n]:
                    ok, why = readable(*points[l][:2])
                    if ok:
                        keep.append(l)
                    else:
                        skipped[l] = "the camera does not read tags there: %s" % why
                out[n] = keep
        words.append("table shared by the fixed halves (switch: production.split \"halves\")")
        return out, skipped, words
    ok_labs = []
    for l in labs:
        if readable:
            ok, why = readable(*points[l][:2])
            if not ok:
                skipped[l] = "the camera does not read tags there: %s" % why
                continue
        ok_labs.append(l)
    if not names:
        for l in ok_labs:
            skipped[l] = "no robot in this run"
        return {}, skipped, words
    ok_labs.sort(key=lambda l: (points[l][0], points[l][1]))
    k = len(names)
    runs, start = [], 0
    for g in range(k):
        n_g = len(ok_labs) // k + (1 if g < len(ok_labs) % k else 0)
        runs.append(ok_labs[start:start + n_g])
        start += n_g
    mids = []
    for r in runs:
        if r:
            mids.append((sum(points[l][0] for l in r) / len(r), sum(points[l][1] for l in r) / len(r)))
        else:
            mids.append(None)
    best, best_cost = None, None
    import itertools
    perms = itertools.permutations(range(k)) if k <= 6 else [tuple(range(k))]
    for perm in perms:
        cost = 0.0
        for g, ri in enumerate(perm):
            n = names[g]
            if mids[ri] is not None:
                cost += math.hypot(robots[n][0] - mids[ri][0], robots[n][1] - mids[ri][1])
        if best_cost is None or cost < best_cost - 1e-9:
            best, best_cost = perm, cost
    out = {names[g]: runs[ri] for g, ri in enumerate(best)}
    words.append("table shared between %d robot%s over the readable area (%d of %d points; %s)" % (
        k, "" if k == 1 else "s", len(ok_labs), len(labs), ", ".join("%s %d" % (n, len(v)) for n, v in out.items())))
    return out, skipped, words


# =============================================================================================== cameras
def pick_cameras(devices, want=None):
    """devices [(path, name)] plugged in; want {"overhead": name part, "side": name part or ""} (the site's or the
    saved cameras.json). One camera plugged: it is the overhead whatever its name (said when the name differs).
    Two or more: the names tell them apart; a wanted name not found falls back to the first unclaimed camera (said).
    Returns ({role: path}, words)."""
    want = dict(want or {})
    words = []
    if not devices:
        return {}, ["no camera is plugged in"]
    if len(devices) == 1:
        p, n = devices[0]
        w = want.get("overhead") or ""
        if w and w.lower() not in (n or "").lower():
            words.append("one camera plugged in (%s): used as the overhead although the setting names '%s' "
                         "(observed, not typed)" % (n, w))
        else:
            words.append("one camera plugged in (%s): the overhead" % n)
        if want.get("side"):
            words.append("the side camera (%s) is not plugged in: the run uses one camera" % want["side"])
        return {"overhead": p}, words
    out, used = {}, set()
    for role in ("overhead", "side"):
        w = (want.get(role) or "").lower()
        if role == "side" and not w and "side" in want:
            continue                       # switched off ("")
        hit = next(((p, n) for p, n in devices if w and w in (n or "").lower() and p not in used), None)
        if hit is None:
            hit = next(((p, n) for p, n in devices if p not in used), None)
            if hit and w:
                words.append("no camera named '%s' for the %s: using %s (observed, not typed)" % (w, role, hit[1]))
        if hit:
            out[role] = hit[0]
            used.add(hit[0])
    return out, words


# =============================================================================================== edge guard
def edge_trip_kind(line_trip, bottom_drop=None, cam_edge_cm=None, cam_age_s=None):
    """What a robot's edge guard trip was. line_trip: the line sensors saw dark; bottom_drop: the distance sensor's
    bottom zone saw a drop (True / False / None not known); cam_edge_cm: the camera's distance from the robot's middle
    to the nearest table edge (None: not seen recently). -> "edge" | "surface_mark" | "unknown".
    A line trip the bottom zone does not confirm, with the camera putting the robot well inside the table, is a mark
    on the surface (school 5 Nov: the black tape over the table seam)."""
    if bottom_drop:
        return "edge"
    if not line_trip:
        return "unknown"
    cam_ok = cam_edge_cm is not None and (cam_age_s is None or cam_age_s <= 1.0)
    if cam_ok and cam_edge_cm <= SURFACE_MARK_CAM_CM:
        return "edge"
    if bottom_drop is False and cam_ok:
        return "surface_mark"
    if bottom_drop is False and not cam_ok:
        return "unknown"
    if cam_ok:
        return "surface_mark"
    return "unknown"


# =============================================================================================== restarts
def restarted(prev, cur):
    """prev, cur: a robot's status dicts ({"boot": id, "up_ms": uptime}). True when it restarted between them (its
    boot id changed or its uptime went back)."""
    if not prev or not cur:
        return False
    if prev.get("boot") is not None and cur.get("boot") is not None and prev["boot"] != cur["boot"]:
        return True
    a, b = prev.get("up_ms"), cur.get("up_ms")
    return a is not None and b is not None and b + 1000 < a


# =============================================================================================== detection
def detect_retry(detect, grey, want_ids, scales=(0.5,), resize=None):
    """detect(grey) -> [det with .tag_id and .corners (4, 2)]. When some wanted id is not found at full size, the
    picture is downscaled (each scale in turn) and searched again; a found tag's corners are scaled back. Returns
    (detections, {id: scale used})."""
    dets = list(detect(grey))
    found = {int(d.tag_id) for d in dets}
    missing = set(int(i) for i in want_ids) - found
    used = {}
    if not missing:
        return dets, used
    if resize is None:
        import cv2
        resize = lambda g, s: cv2.resize(g, (max(1, int(g.shape[1] * s)), max(1, int(g.shape[0] * s))),  # noqa: E731
                                         interpolation=cv2.INTER_AREA)
    for s in scales:
        small = resize(grey, s)
        for d in detect(small):
            tid = int(d.tag_id)
            if tid in missing:
                try:
                    d.corners = d.corners / s
                    if getattr(d, "center", None) is not None:
                        d.center = d.center / s
                except (TypeError, AttributeError):
                    continue
                dets.append(d)
                missing.discard(tid)
                used[tid] = s
        if not missing:
            break
    return dets, used


# =============================================================================================== robot-side safety
# (Nick 5 Nov 20:46: at home a robot was seen tilted as if about to fall off the board.) Every run type switches the
# robot's own safety layers on at its start and reads them back: they are lost at every power-off and a wake from sleep
# restarts the robot. The tilt stop (firmware motion.h TILT) stops any drive or turn when the robot tips more than
# tilt_stop_deg from its level for tilt_ms (e02 legs and follow steps alike); the edge guard's bottom zone sees a real
# drop ahead. School: safe_edge 2 (the bottom zone only: the line sensors read the black seam tape as an edge).
TILT_STOP_DEG = _SR.tilt_limits()["tilt_stop_deg"]   # (e08, A4) 4 deg, re-picked from the 5 / 6 Nov home logs: 0 false
                         # stops in 12.1 robot-hours, 2 of the 3 real hangs caught (5 deg: none of them). Sent at every
                         # run's start (a runtime param: no reflash). Before e08: the firmware's 5 (MIRLT_TILT_RULE=old)
TILT_MS = _SR.tilt_limits()["tilt_ms"]                # 300 ms (kept: 3 false stops at 100 ms and 4 deg)
EDGE_BOT_CM = 11.0       # the bottom zone's drop cue (mission_run / calib_run EDGE_GUARD)
BACK_OFF_CM = 10.0       # after a tilt or drop trip: straight BACKWARDS this far (never forward): the body's half
                         # length (6 cm) plus 4, so the wheels come back over the board and the next leg starts clear of
                         # the kept-out spot. Within the firmware's e02_back_max_cm. Chosen; DOE note: 6 / 10 / 14 cm on
                         # the bench (one wheel off a step) against the robot levelling and later legs refused
TRIP_KEEPOUT_R_CM = 5.0   # a tilt / drop trip spot becomes a keep-out disc this big, centred NOSE_AHEAD_CM ahead of the
NOSE_AHEAD_CM = 5.0       # axle: it covers 0-10 cm ahead, where the edge must be (hung: at the wheels; the bottom zone
                          # sees ~8 cm ahead). Chosen


def safety_settings(safe_edge=3):
    """{param: value} every run sets at its start (and reads back). safe_edge: the site's (school 2, home 3)."""
    return {"safe_edge": float(safe_edge), "edge_bot_cm": EDGE_BOT_CM, "tilt_stop_deg": TILT_STOP_DEG,
            "tilt_ms": TILT_MS, "safe_lift": 1.0}


def safety_commands(safe_edge=3):
    return ["set %s %g" % (k, v) for k, v in safety_settings(safe_edge).items()]


def site_safe_edge(site, default=3):
    """The site's edge guard for every run type: safety.safe_edge, else calibration.safe_edge, else default."""
    s = site or {}
    v = (s.get("safety") or {}).get("safe_edge")
    if v is None:
        v = (s.get("calibration") or {}).get("safe_edge")
    return int(default if v is None else v)


def check_safety(want, got, tol=0.06):
    """want {param: value}, got {param: value or None} -> [(param, wanted, got)] that do not match."""
    bad = []
    for k, v in want.items():
        g = got.get(k)
        if g is None or abs(float(g) - float(v)) > tol * max(1.0, abs(float(v))):
            bad.append((k, v, g))
    return bad


def trip_kind(info):
    """A failed leg / step's words -> "tilt" (the tilt stop), "drop" (the edge guard: a real edge ahead), "line" (the
    line sensors alone), or None."""
    s = (info or "").lower()
    if "tilted" in s:
        return "tilt"
    if "plywood edge" in s or "edge ahead" in s:
        if "line" in s and "bot" not in s:
            return "line"
        return "drop"
    return None


def trip_keepout(x, y, th_deg, name="trip"):
    """The keep-out disc for a tilt / drop trip at pose (x, y, th): centred NOSE_AHEAD_CM ahead of the axle."""
    h = math.radians(th_deg)
    src = "tilt/drop trip (discovered edge)"
    return {"name": name, "center": [round(x + NOSE_AHEAD_CM * math.cos(h), 2),
                                     round(y + NOSE_AHEAD_CM * math.sin(h), 2)],
            "radius": TRIP_KEEPOUT_R_CM, "shape": "circle", "source": src,
            "confirmed": _SR.keepout_confirmed(src)}     # (e08, A6) a real trip: confirmed, never fades


def back_off_target(x, y, th_deg, d=BACK_OFF_CM):
    """Where a straight backward drive of d cm ends (the heading kept: no turn while tilted)."""
    h = math.radians(th_deg)
    return x - d * math.cos(h), y - d * math.sin(h)


# =============================================================================================== four protections
# (home free roam 5 Nov, 4 h analysis: Center drove diagonally into the x = 0 edge on a pose 17 cm off the camera's,
# sat tilted 3.4-4.1 deg, under the 5 deg tilt stop, for 64 min, was sent 13 blind reverses; Left did 4,289 reverses
# "done in 0.05 s without moving" 8 cm from an edge.) Shared by every run type: free roam, missions / experiments
# (run_plan), the calibration.
POSE_AGREE_CM = 5.0      # (1) own pose vs camera: more than this ...
POSE_AGREE_S = 2.0       # ... for this long (camera fresh) = stop that robot, re-seed its pose from the camera before
                         # any further move. Chosen (the analysis: 13-52 cm for 80 min went unnoticed; the camera's
                         # own error is under 1 cm). DOE note: 3 / 5 / 8 cm against false stops in a normal run
STAND_TILT_DEG = _SR.tilt_limits()["stand_tilt_deg"]  # (2) standing tilt: more than this from its level ...
STAND_TILT_S = _SR.tilt_limits()["stand_tilt_s"]      # ... for this long while not driving = a hazard (a wheel over an
                         # edge). (e08) checked against the 5 / 6 Nov logs: 0 false in 12.1 robot-hours, 2 of 3 hangs
                         # caught (2 deg: 2 false; 3 deg / 1 s: also 0, 1 s sooner). Kept. Chosen from the
                         # analysis (robots stood within 1.5 deg on the board; Center sat at 3.4). No bench test now
                         # (Nick 21:38): kept as chosen. DOE note: a wheel over a 1 / 2 / 3 cm drop, standing and
                         # driving, would set both
STAND_V_CMS = 1.0        # "not driving": its own speed under this
LEVEL_N = 25             # its level: the median of this many standing samples at the start (RAW 50 / s: half a second)
MIN_JUDGE_CM = 3.0       # (3') moves shorter than this are not judged (sensor noise is that size)
PHYS_MAX_CMS = 30.0      # (3') nothing an Alvik does covers ground faster than this (its top speed ~ 20 cm/s): a move
                         # "done" faster is physically impossible = a robot-side refusal or failure, whatever else says
REVERSE_MARGIN_CM = 3.0  # (4') a reverse shown safe by the camera: the body's back (BODY_BACK_CM behind the axle) stays
BODY_BACK_CM = 6.0       # this margin + the pose uncertainty inside every edge and off keep-outs and robots
TRAIL_TOL_CM = 2.0       # (4'a) the breadcrumb trail: a reverse follows the robot's own recent path when every point of
TRAIL_TOL_DEG = 8.0      # the reverse stays within this (plus the odometry bubble) of where it drove, along its heading
TURN_R_CM = 6.72         # the body's half diagonal: the circle a turn on the spot sweeps


class PoseAgree:
    """(1) Own pose against the camera, for one robot. feed(t, own (x, y), cam (x, y), cam_age_s) -> None or
    {"kind": "reseed", "d_cm", "for_s"} once the disagreement has lasted POSE_AGREE_S with a fresh camera."""

    def __init__(self, lim_cm=POSE_AGREE_CM, for_s=POSE_AGREE_S):
        self.lim, self.for_s, self.since, self.n = lim_cm, for_s, None, 0

    def feed(self, t, own, cam, cam_age_s=0.0):
        if own is None or cam is None or cam_age_s is None or cam_age_s > 0.6:
            self.since = None
            return None
        d = math.hypot(own[0] - cam[0], own[1] - cam[1])
        if d <= self.lim:
            self.since = None
            return None
        if self.since is None:
            self.since = t
            return None
        if t - self.since >= self.for_s:
            self.since = None
            self.n += 1
            return {"kind": "reseed", "d_cm": round(d, 2), "for_s": self.for_s, "n": self.n}
        return None


class TiltWatch:
    """(2) Standing tilt for one robot from its own roll / pitch (RAW rows). Its level is the median of the first
    LEVEL_N standing samples. feed(t, roll, pitch, speed_cms) -> None or {"kind": "tilt_hazard", "dev_deg", "for_s"}
    (once per episode; re-armed when it comes back under the limit)."""

    def __init__(self, lim=STAND_TILT_DEG, for_s=STAND_TILT_S):
        self.lim, self.for_s = lim, for_s
        self.samples, self.level = [], None
        self.since, self.fired, self.last = None, False, 0.0

    def feed(self, t, roll, pitch, speed_cms=0.0):
        standing = abs(speed_cms or 0.0) < STAND_V_CMS
        if self.level is None:
            if standing:
                self.samples.append((roll, pitch))
                if len(self.samples) >= LEVEL_N:
                    rs = sorted(s[0] for s in self.samples)
                    ps = sorted(s[1] for s in self.samples)
                    self.level = (rs[len(rs) // 2], ps[len(ps) // 2])
            return None
        dev = max(abs(roll - self.level[0]), abs(pitch - self.level[1]))
        self.last = dev
        if not standing or dev <= self.lim:
            if dev <= self.lim:
                self.since, self.fired = None, False
            return None
        if self.since is None:
            self.since = t
        if not self.fired and t - self.since >= self.for_s:
            self.fired = True
            return {"kind": "tilt_hazard", "dev_deg": round(dev, 2), "for_s": round(t - self.since, 2)}
        return None


def done_too_fast(expected_s, took_s, cam_moved_cm, length_cm):
    """Kept for the callers of the first version: physical impossibility only (see move_outcome)."""
    return impossible(length_cm, took_s)


def impossible(length_cm, took_s):
    """(3') A move of 3 cm or more "done" faster than PHYS_MAX_CMS allows did not happen (a robot-side refusal or
    failure), whatever any other signal says."""
    return length_cm is not None and abs(length_cm) >= 3.0 and took_s is not None and \
        took_s < abs(length_cm) / PHYS_MAX_CMS


def move_outcome(length_cm, took_s, wheel_cm=None, accel_rolling=None, gyro_deg=None, turn_deg=0.0,
                 front_change_cm=None, front_expect_cm=None, line_changed=None, cam_moved_cm=None, cam_fresh=False,
                 robot_info=""):
    """(3', Nick 5 Nov 21:11: move outcome by evidence fusion) Did a move happen? Each independent signal votes
    (0 = did not move, 1 = moved) with its own weight; no single one decides, except physical impossibility.
      wheel_cm        what the wheels counted (its LEG record / own pose), weight 1: wheels can spin on the spot
      accel_rolling   the accelerometer felt it roll (start / stop jolts, vibration), weight 1.5
      gyro_deg        the gyro's turn (for a turn), weight 1.5
      front_change_cm the front range to what is ahead changed by about front_expect_cm, weight 2 (only when a thing
                      is in range ahead)
      line_changed    the line sensors' surface readings changed while it moved, weight 0.5
      cam_moved_cm    the camera saw it move, weight 3, only with a fresh confident fix
    Returns dict(verdict "moved" | "not_moved" | "impossible", score 0-1, used [sources], words, blind_ok: moved with
    no camera = accept it and grow the dead-reckoning bubble). robot_info: its own reason for ending early, logged."""
    L = abs(length_cm or 0.0)
    used, num, den = [], 0.0, 0.0
    if L < MIN_JUDGE_CM:
        # a step of a few millimetres (already there): nothing can tell, and calling it a failure loops the re-plans
        # (rehearsal s2: a 0.4 cm step judged "did not move" again and again)
        return dict(verdict="moved", score=1.0, used=[], blind_ok=False, words="too short to judge (%.1f cm)" % L)
    if impossible(L, took_s):
        return dict(verdict="impossible", score=0.0, used=["time"], blind_ok=False,
                    words="%.0f cm in %.2f s is not physically possible: a robot-side refusal or failure%s" % (
                        L, took_s, (" (it said: %s)" % robot_info) if robot_info else ""))

    def vote(name, v, w):
        nonlocal num, den
        used.append(name)
        num += w * max(0.0, min(1.0, v))
        den += w
    if wheel_cm is not None and L > 0:
        vote("wheels", abs(wheel_cm) / L, 1.0)
    if gyro_deg is not None and abs(turn_deg) >= 10:
        vote("gyro", abs(gyro_deg) / abs(turn_deg), 1.5)
    if accel_rolling is not None and L > 0:
        vote("accelerometer", 1.0 if accel_rolling else 0.0, 1.5)
    if front_change_cm is not None and front_expect_cm and abs(front_expect_cm) >= 2.0:
        vote("front range", abs(front_change_cm) / abs(front_expect_cm), 2.0)
    if line_changed is not None and L > 0:
        vote("line sensors", 1.0 if line_changed else 0.3, 0.5)
    if cam_fresh and cam_moved_cm is not None and L > 0:
        vote("camera", abs(cam_moved_cm) / L, 3.0)
    if den == 0:
        return dict(verdict="moved", score=0.5, used=[], blind_ok=False, words="no evidence either way (taken as done)")
    sc = num / den
    verdict = "moved" if sc >= 0.5 else "not_moved"
    own = [u for u in used if u != "camera"]
    blind_ok = verdict == "moved" and "camera" not in used and len(own) >= 2
    return dict(verdict=verdict, score=round(sc, 2), used=used, blind_ok=blind_ok,
                words="%s (score %.2f from %s)%s" % ("moved" if verdict == "moved" else "did NOT move", sc,
                                                    ", ".join(used), (" (it said: %s)" % robot_info) if robot_info
                                                    else ""))


class FailMemory:
    """(3b) Never repeat a failed action unchanged. Each failed action is kept with where it was tried; the same kind,
    direction and length (within 25 %) from within SAME_SPOT_CM of a failed one is refused; failures piling up at one
    spot escalate (ESCALATE_N: a hazard there, the others kept out, re-seed / find-me)."""
    SAME_SPOT_CM = 5.0
    ESCALATE_N = 3
    ESCALATE_CM = 8.0

    def __init__(self):
        self.fails = []

    def add(self, t, kind, sign, length_cm, x, y):
        self.fails.append((t, kind, sign, float(length_cm or 0.0), float(x), float(y)))
        del self.fails[:-50]

    def same(self, kind, sign, length_cm, x, y):
        L = float(length_cm or 0.0)
        for _, k, sg, l0, fx, fy in self.fails:
            if k == kind and sg == sign and math.hypot(x - fx, y - fy) <= self.SAME_SPOT_CM and \
                    abs(L - l0) <= 0.25 * max(abs(l0), 1.0):
                return True
        return False

    def escalate(self, x, y):
        return sum(1 for f in self.fails if math.hypot(x - f[4], y - f[5]) <= self.ESCALATE_CM) >= self.ESCALATE_N


class Trail:
    """(4'a) The breadcrumb trail: the robot's own recent poses (its odometry, x, y, heading). Ground it has just
    driven over is known safe; a reverse of L cm is safe when the last L cm it drove were forwards along (about) its
    present heading, within the odometry bubble."""

    def __init__(self, keep_cm=60.0):
        self.pts = []          # (x, y, th)
        self.keep = keep_cm

    def add(self, x, y, th):
        if self.pts and math.hypot(x - self.pts[-1][0], y - self.pts[-1][1]) < 0.5:
            return
        self.pts.append((float(x), float(y), float(th)))
        d, k = 0.0, len(self.pts) - 1
        while k > 0 and d < self.keep:
            d += math.hypot(self.pts[k][0] - self.pts[k - 1][0], self.pts[k][1] - self.pts[k - 1][1])
            k -= 1
        del self.pts[:max(0, k - 1)]

    def clear(self):
        self.pts = []

    def reverse_ok(self, L, bubble_cm=0.0):
        """(ok, cm covered): the last L cm of the trail run straight back along the present heading."""
        if len(self.pts) < 2:
            return False, 0.0
        x, y, th = self.pts[-1]
        h = math.radians(th)
        ux, uy = math.cos(h), math.sin(h)
        cov = 0.0
        for k in range(len(self.pts) - 1, 0, -1):
            px, py = self.pts[k - 1][:2]
            back = -((px - x) * ux + (py - y) * uy)          # how far behind (along the heading)
            side = abs(-(px - x) * uy + (py - y) * ux)
            if side > TRAIL_TOL_CM + bubble_cm or abs(wrap180(self.pts[k - 1][2] - th)) > TRAIL_TOL_DEG + 10.0:
                break
            cov = max(cov, back)
            if cov >= L:
                return True, cov
        return cov >= L, cov


TRACK_CHORD_STEP_CM = 1.0   # (e16) a chord is checked against the track every this many cm. Not a tuning (sampling)


def _seg_dist(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def track_back_target(trail_pts, pos, heading, clear_fn, tol_cm=None, tol_deg=None):
    """(e16, item 7; Nick 11 Nov 08:45) A robot stopped at an edge came in over ground it has just driven: reversing
    back along its own last cm of track is reversing over positions it has proven are table. trail_pts: its recorded
    poses (its own odometry, oldest first: Trail.pts); pos, heading: where it is now (the run's position, the heading
    it holds); clear_fn((x, y)) -> True where the edge rule is met (the clearance with its error sum, from the caller).
    The track is laid onto the present pose (its newest point = pos, its heading = heading). Walking back along it, the
    reverse goes to the FIRST track point that is clear (the distance is derived from the edge rule, never picked), as
    one straight chord that stays within tol_cm of the track and runs straight back along the heading (tol_deg: the
    firmware's reverse is a straight drive, no turn at the edge). A curved track gives a shorter chord: the caller
    repeats from where that ends. Returns dict(L, end, arc_cm, cleared, words) or None (no track behind it)."""
    tol_cm = TRAIL_TOL_CM if tol_cm is None else tol_cm
    tol_deg = TRAIL_TOL_DEG if tol_deg is None else tol_deg
    if not trail_pts or len(trail_pts) < 2 or pos is None:
        return None
    x1, y1, th1 = trail_pts[-1]
    a = math.radians(wrap180(heading - th1))
    ca, sa = math.cos(a), math.sin(a)
    pts = [(pos[0] + ca * (x - x1) - sa * (y - y1), pos[1] + sa * (x - x1) + ca * (y - y1)) for x, y, _ in trail_pts]
    best, arc = None, 0.0
    for k in range(len(pts) - 2, -1, -1):
        arc += math.dist(pts[k + 1], pts[k])
        q = pts[k]
        L = math.dist(pos, q)
        if L < 0.5:
            continue
        d_ang = abs(wrap180(math.degrees(math.atan2(q[1] - pos[1], q[0] - pos[0])) - (heading + 180.0)))
        if d_ang > tol_deg:
            break
        ok = True
        n = max(2, int(L / TRACK_CHORD_STEP_CM) + 1)
        for j in range(1, n):
            c = (pos[0] + (q[0] - pos[0]) * j / n, pos[1] + (q[1] - pos[1]) * j / n)
            if min(_seg_dist(c, pts[i], pts[i + 1]) for i in range(k, len(pts) - 1)) > tol_cm:
                ok = False
                break
        if not ok:
            break
        best = dict(L=round(L, 2), end=(q[0], q[1]), arc_cm=round(arc, 2), cleared=bool(clear_fn(q)), k=k)
        if best["cleared"]:
            break
    if best is None:
        return None
    best["words"] = ("%.1f cm straight back along its own track (%s)" % (
        best["L"], "clear of the edge rule there" if best["cleared"] else
        "as far as a straight reverse stays on the track; it goes on from there"))
    return best


def moved_in_evidence(fixes_held, fixes_now, sigma_floor_cm=0.5, k_sigma=4.0, n_now=3, lifted=False):
    """(e16, 8a; home 11 Nov 08:48: "REJOINED ... was moved in" one second after NEEDS A HAND, nobody had touched it)
    Did a person really move a held robot? Evidence only: a lift / put-down (lifted: the robot's own accelerometer or
    tilt said so), or the camera's spot moved further than its own measured noise allows, sustained. fixes_held: the
    camera (x, y) seen while it was held; fixes_now: the newest ones. The noise is the held fixes' own scatter (one
    axis, sigma; never under sigma_floor_cm, the camera's resolution); the move is the median of the last n_now fixes
    against the median while held, and must exceed k_sigma x sqrt(2) x sigma (the difference of two noisy spots).
    Returns (bool, words)."""
    if lifted:
        return True, "it was lifted and put down (its own accelerometer)"
    if len(fixes_held) < 3 or len(fixes_now) < n_now:
        return False, "too few camera fixes to tell (%d held, %d now)" % (len(fixes_held), len(fixes_now))

    def med(v):
        v = sorted(v)
        return v[len(v) // 2]
    hx, hy = med([f[0] for f in fixes_held]), med([f[1] for f in fixes_held])
    sx = math.sqrt(sum((f[0] - hx) ** 2 for f in fixes_held) / max(1, len(fixes_held) - 1))
    sy = math.sqrt(sum((f[1] - hy) ** 2 for f in fixes_held) / max(1, len(fixes_held) - 1))
    sig = max(sigma_floor_cm, sx, sy)
    last = fixes_now[-n_now:]
    nx, ny = med([f[0] for f in last]), med([f[1] for f in last])
    moved = math.hypot(nx - hx, ny - hy)
    lim = k_sigma * math.sqrt(2.0) * sig
    # every one of the last n_now fixes past the limit too: sustained, not one jump
    each = all(math.hypot(f[0] - hx, f[1] - hy) > lim for f in last)
    if moved > lim and each:
        return True, "the camera saw it move %.1f cm (its noise %.1f cm: limit %.1f cm), on %d fixes in a row" % (
            moved, sig, lim, n_now)
    return False, "the camera has it %.1f cm from where it was held (limit %.1f cm from its noise %.1f cm)" % (
        moved, lim, sig)


def back_out_plan(L, trail=None, bubble_cm=0.0, cam=None, cam_age_s=None, rect=None, others=(), keepouts=(),
                  turn_clear=None, tilted=False):
    """(4', Nick 5 Nov 21:11) How to get out backwards, by the evidence that covers the swept space:
      (a) "reverse_trail": back along its own breadcrumb trail (no camera needed), up to what the trail covers;
      (c) "reverse_camera": a fresh confident camera fix shows room behind (reverse_room);
      (d) "turn_forward": look before backing: turn on the spot (turn_clear: the body circle's sweep is shown clear;
          never while tilted: the firmware refuses turns then) so the front sensors check the way, then drive forward;
      (e) "hold": nothing shows any direction safe: hold and call for the camera / a person.
    Returns (how, length_cm, words)."""
    if trail is not None:
        ok, cov = trail.reverse_ok(L, bubble_cm)
        if ok:
            return "reverse_trail", L, "back along its own trail (%.0f cm driven there)" % cov
        if cov >= 3.0 and cam is None:
            return "reverse_trail", round(cov, 1), "back along its own trail, only the %.0f cm it covers" % cov
    if cam is not None and rect is not None:
        ok, why = reverse_room(cam, L, rect, unc_cm=bubble_cm, others=others, keepouts=keepouts, cam_age_s=cam_age_s)
        if ok:
            return "reverse_camera", L, "the camera shows room behind"
    if turn_clear and not tilted:
        return "turn_forward", 0.0, "turns on the spot (its sweep is clear) so its front sensors check the way"
    return "hold", 0.0, "nothing shows a safe way out (no trail behind, no camera room%s): it holds; the camera or a " \
                        "person is needed" % (", tilted: no turn" if tilted else "")


def reverse_room(cam, length_cm, rect, unc_cm=0.0, others=(), keepouts=(), cam_age_s=0.0):
    """(4) A straight reverse of length_cm from cam (x, y, heading deg: the CAMERA's pose, fresh) is allowed only
    when the camera shows room behind: the back of the body (BODY_BACK_CM behind the axle) at the end stays
    REVERSE_MARGIN_CM + unc_cm inside every side of rect (x0, y0, x1, y1), off every keep-out ((x, y, r) discs) and
    clear of every other robot ((x, y) middles, 2 x BODY_BACK_CM + margin). (ok, words). Never without a camera."""
    if cam is None or cam_age_s is None or cam_age_s > 0.6:
        return False, "no fresh camera position: a reverse is blind (refused)"
    h = math.radians(cam[2])
    need = BODY_BACK_CM + REVERSE_MARGIN_CM + unc_cm
    for k in range(1, 11):
        d = length_cm * k / 10.0
        px, py = cam[0] - d * math.cos(h), cam[1] - d * math.sin(h)
        bx, by = px - BODY_BACK_CM * math.cos(h), py - BODY_BACK_CM * math.sin(h)     # the body's back
        e = min(bx - rect[0], rect[2] - bx, by - rect[1], rect[3] - by)
        if e < REVERSE_MARGIN_CM + unc_cm:
            return False, "the camera shows %.1f cm between its back and the edge (needs %.1f)" % (
                e, REVERSE_MARGIN_CM + unc_cm)
        for (kx, ky, kr) in keepouts:
            # the body's back (where the robot is blind) must stay off the keep-out; moving away from one ahead is fine
            if math.hypot(bx - kx, by - ky) < kr + REVERSE_MARGIN_CM + unc_cm:
                return False, "a keep-out behind it"
        for (ox, oy) in others:
            if math.hypot(bx - ox, by - oy) < BODY_BACK_CM + REVERSE_MARGIN_CM + unc_cm:
                return False, "another robot behind it"
    return True, "room behind (camera)"


# =============================================================================================== the side-edge invariant
# (Nick 5 Nov 21:14) The robot has no sideways sensor, so a shallow approach to a side edge is seen by nothing on the
# robot. Hard invariant in every run type: the robot's POSSIBLE-LOCATIONS circle (the body's half diagonal + its pose
# uncertainty: camera error, the dead-reckoning bubble, the camera's staleness x its speed, clock doubt) never overlaps
# the workspace perimeter (every table / board side is an open drop, home included, less EDGE_MARGIN_CM; discovered
# edges and keep-outs) - not at a move's end and not anywhere along its swept path (a turn's corner sweep included:
# the circle already covers any heading). When the circle has grown so the robot's spot violates it, the only moves
# allowed shrink the overlap.
HALF_DIAG_CM = 6.72      # the 9.5 cm body's half diagonal (any heading)
EDGE_MARGIN_CM = 1.0     # chosen: the circle stays this far inside every side (the table's edge itself is the drop)


def uncertainty_cm(base_cm=1.0, bubble_cm=0.0, cam_age_s=0.0, speed_cms=0.0, clock_doubt_s=0.0):
    """The pose uncertainty (cm): the camera's own error (base), the dead-reckoning bubble, how far it may have gone
    since the camera's (stale) picture, and a clock doubt (seconds the camera's time may be off) at its speed."""
    return base_cm + bubble_cm + abs(speed_cms) * (max(0.0, cam_age_s) + max(0.0, clock_doubt_s))


def overlap_cm(x, y, unc_cm, rect, keepouts=()):
    """How far the possible-locations circle at (x, y) reaches past the perimeter (cm; <= 0: inside). rect: (x0, y0,
    x1, y1) of the drop; keepouts: (x, y, r) discs."""
    R = HALF_DIAG_CM + unc_cm + EDGE_MARGIN_CM
    e = min(x - rect[0], rect[2] - x, y - rect[1], rect[3] - y)
    worst = R - e
    for kx, ky, kr in keepouts:
        worst = max(worst, R + kr - math.hypot(x - kx, y - ky))
    return worst


def path_ok(points, unc_cm, rect, keepouts=(), unc_per_cm=0.0):
    """(ok, words) for a move swept through points [(x, y), ...] (the whole path, sampled): the circle, growing by
    unc_per_cm per cm driven (odometry), never overlaps. A robot whose start already overlaps may only take a move
    whose overlap never grows and ends smaller (towards the interior)."""
    if not points:
        return True, ""
    o0 = overlap_cm(points[0][0], points[0][1], unc_cm, rect, keepouts)
    d, prev, worst = 0.0, points[0], o0
    for p in points[1:]:
        d += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
        o = overlap_cm(p[0], p[1], unc_cm + unc_per_cm * d, rect, keepouts)
        if o0 <= 0 and o > 0:
            return False, "its possible-locations circle would reach %.1f cm past the edge at %.0f, %.0f" % (o, p[0],
                                                                                                             p[1])
        if o0 > 0 and o > worst + 0.05:
            return False, "it already overlaps the edge by %.1f cm: only moves that shrink that are allowed" % o0
        worst = max(worst, o) if o0 <= 0 else min(worst, max(o, worst))
    o_end = overlap_cm(points[-1][0], points[-1][1], unc_cm + unc_per_cm * d, rect, keepouts)
    if o0 > 0 and o_end >= o0 - 0.05:
        return False, "it overlaps the edge by %.1f cm and this move does not shrink it" % o0
    return True, ""


def max_len_for(unc_cm, unc_per_cm, room_cm):
    """Planning shrinks moves as uncertainty grows: the longest straight move whose end circle still fits in room_cm
    (the distance from its middle to the perimeter along the way)."""
    if unc_per_cm <= 0:
        return float("inf")
    return max(0.0, (room_cm - HALF_DIAG_CM - EDGE_MARGIN_CM - unc_cm) / unc_per_cm)
