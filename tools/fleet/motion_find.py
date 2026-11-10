#!/usr/bin/env python3
"""Robot finding by motion at the start of every run (build e14; Nick's design, 10 Nov 2026).

Nick, 14:32: "Motion should be used for finding and getting it back, not moving it by hand." A robot is found ANYWHERE
on the table, with or without a roof tag that reads:

  1. CLEAR SPACE  the person confirms each robot has the clear space a stroke needs (camera_discovery.clearance: derived,
                  said in cm and inches). Not confirmed for a robot (or a pose guess shows a stroke would not be clear):
                  that robot WIGGLES on the spot instead (+-30 deg: only its own footprint).
  2. EVERY CAMERA every camera the Pi finds is opened (camera_discovery.find_cameras); a camera that sees no robot is
                  left out and said.
  3. ONE AT A TIME each robot in turn makes its small move (a 6 cm stroke forward and back, or the wiggle): what moves
                  in a picture is that robot. Its position (and, from a stroke, its heading) comes from the motion; the
                  roof tag whose read moved with it (where tags read) is its tag.
  4. BRING BACK   a robot found where its tag does not read drives itself back to where the readability map says tags
                  read, in short straight steps, each step itself a motion sighting (its new position and heading), under
                  the never-give-up rule (CLAUDE.md, Nick 12:51): a step is taken only while its whole possible error
                  (the position's error sum grown with every cm and degree since the last sighting, + half the body's
                  diagonal + its stop) cannot reach an edge or keep-out; else it holds, wiggles so the motion detector
                  can find it again (which resets the error sum), and asks for a hand only when that cannot help. It
                  never gives up: Stop ends it. The spots where its tag did not read are recorded as evidence.

Switch: MIRLT_CAMERA_DISCOVERY=off (the binding by the wiggle alone, as before e14). Decided automatically: on.

    python3 tools/fleet/motion_find.py sim [--out DIR]     the simulator rig (P15 unread, a dark patch, two robots close,
                                                           an extra blind camera)
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (HERE, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np   # noqa: E402

VERSION = "motion-find-2026-11-10e14b"   # e14b: tagless only on proven-readable spots (Laplace rate); a target reached with no read or blocked by a robot is not tried again (rehearsal: it circled a target in a dim patch; held beside a parked robot)

# ---------------------------------------------------------------------------------------------- numbers (and why)
STEP_CM = 5.0            # one bring-back step (calib_run RECOVER_STEP_CM, chosen there; DOE row there). Each step is
                         # also the motion detector's sighting, so it must be several of the detector's pixels long
STEP_SPEED = 5.0         # cm/s (calib_run FINAL_CMS: the slowest drive). Not chosen here
BLIND_BASE_CM = 2.0      # the error sum: MEASURED 29 Oct (run_plan BLIND_ERR_*): p95 2.0 cm + 0.07 cm per cm unseen;
BLIND_PER_CM = 0.07      # uncalibrated (a robot found at the start of a calibration) + 0.06 (calib_run BLIND_PER_CM_UNCAL)
BLIND_PER_CM_UNCAL = 0.13
TURN_FRAC_UNCAL = 0.07   # heading error per degree turned on the gyro alone, uncalibrated (calib_run). Chosen there
MOTION_PX_SIGMA = 6.0    # MEASURED in the simulator only (e14 rig, the school's low camera drawn at 1280 x 960, 30 random
                         # spots and headings): position error median 5.9 px (1.5-2 cm), p95 11.4 px; the e13 rig's steep
                         # cameras measured under 2 px. A low camera sees the body's sides. DOE row (e14): motion
                         # sightings against tag reads on the real camera
MIN_SIGMA_CM = 0.5       # never better than this (the roof's height over the table, unmodelled lean). Chosen; no DOE
FAILED_R_CM = 20.0       # a spot where the robot stood and its tag did not read rules out targets this near (one
                         # readability cell, observe.CELL_CM). Not chosen here
HALF_DIAG_CM = 6.72      # the 9.5 cm body's half diagonal (safety_rules.HALF_DIAG_CM). Not chosen
MIN_STROKE_CM = 2.0      # CHOSEN: the shortest stroke worth making for a heading (about 5 px at the school camera's far
                         # end: the template match needs a few pixels). DOE row (e14): stroke length against heading error
TAG_WAIT_S = 0.6         # after a step onto readable ground, wait this long for a read before counting a miss (about 4
                         # pictures at the dim room's 7 a second, measured 10 Nov). Not chosen beyond that; no DOE
TAG_NEAR_CM = 8.0        # a roof tag read this near the robot's motion position (plus its error sum) is its tag: the
                         # body's half diagonal + 1 cm. Chosen; no DOE (the wiggle binding confirms it after)


def enabled():
    """Robot finding by motion: on unless MIRLT_CAMERA_DISCOVERY=off (e14: on by default once the simulator passed)."""
    return os.environ.get("MIRLT_CAMERA_DISCOVERY", "on").strip().lower() not in ("off", "0", "no")


def say(s):
    print(time.strftime("%H:%M:%S ") + s, flush=True)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def sigma_of(cam_rec):
    """A motion sighting's position error (cm) from its camera's scale there."""
    ppc = cam_rec.get("px_per_cm_model") or cam_rec.get("px_per_cm_stroke") or 0
    return max(MIN_SIGMA_CM, MOTION_PX_SIGMA / ppc) if ppc else 2.0


# ---------------------------------------------------------------------------------------------- 1. clear space -> mode
def modes_for(names, confirmed, guesses=None, problems=None, stroke_cm=None):
    """{name: "stroke" | "wiggle"}: a stroke where the person confirmed the clear space for that robot (confirmed: True
    = every robot, or a set of names), and, when the run has a pose guess for it (a pinned roof tag read where it
    stands), only when the stroke is clear both ways from there; otherwise the wiggle on the spot."""
    import camera_discovery as CD
    L = CD.nudge_cm() if stroke_cm is None else float(stroke_cm)
    out = {}
    for n in names:
        ok = confirmed is True or (isinstance(confirmed, (set, list, tuple)) and n in confirmed)
        g = (guesses or {}).get(n)
        if ok and g is not None and problems is not None:
            for flip in (0.0, 180.0):
                h = math.radians(g[2] + flip)
                if problems(g[:2], (g[0] + L * math.cos(h), g[1] + L * math.sin(h))):
                    ok = False
        out[n] = "stroke" if ok else "wiggle"
    return out


# ---------------------------------------------------------------------------------------------- 2-3. find
def find(cams, names, mover, turner=None, modes=None, tag_moved=None, readable=None, log=say, typed=None):
    """Each robot in turn moves (stroke or wiggle); every camera's motion and the run's tag reads say where it is and
    which roof tag is its. tag_moved(name, mode, t0, t1) -> tag id or None (the run's own tag reads); readable(x, y) ->
    bool (the readability map). Returns {"robots": {name: {where, heading, sigma, tag, cameras, how, readable}},
    "coverage": camera_discovery's record}."""
    import camera_discovery as CD
    tags = {}

    def moved(n, mode, t0, t1):
        if tag_moved is not None:
            try:
                tags[n] = tag_moved(n, mode, t0, t1)
                return tags[n]
            except Exception as e:  # noqa: BLE001  (a tag read never stops the finding)
                log("  (%s: the tag reads during its move could not be read: %s)" % (n, e))
        return None
    rec = CD.discover(cams, list(names), mover, modes=modes, turner=turner, on_moved=moved, typed=typed, log=log)
    out = {}
    for n in names:
        r = rec["robots"].get(n) or {}
        best = (r.get("per_camera") or {}).get(r.get("best_camera") or "", {})
        w = r.get("where_cm")
        tg = tags.get(n) if tags.get(n) is not None else r.get("tag_id")
        rd = None
        if w is not None and readable is not None:
            try:
                rd = bool(readable(w[0], w[1]))
            except Exception:  # noqa: BLE001
                rd = None
        # a heading only from a stroke whose own picture was found again (template): the changed area's drift alone
        # pointed 100-130 deg wrong in the e14 rig; a position from a stroke with no sure heading is the stroke's middle
        # (half the stroke more doubt)
        sure = bool(best.get("heading_sure"))
        sig = sigma_of(best) + (0.0 if sure or r.get("how") == "wiggle" else 0.5 * float(rec.get("stroke_cm") or 6.0))
        out[n] = dict(where=w, heading=r.get("heading_deg") if sure else None, sigma=round(sig, 2) if w else None,
                      tag=tg, cameras=r.get("cameras") or [], how=r.get("how", "stroke"), readable=rd,
                      found=w is not None, end=r.get("end_cm"))
    for n, f in out.items():
        rf = (rec.get("refused") or {}).get(n)
        f["refused"] = rf
        if not f["found"] and rf:
            log("  %s: IT DID NOT MAKE ITS FINDING MOVE (the robot said: %s): it is not found by motion" % (n, rf))
        elif not f["found"]:
            log("  %s: NO CAMERA SAW IT MOVE (%s): it is not found by motion" % (
                n, "it is out of every camera's view, in a patch too dark for the motion detector, or did not move"))
        else:
            log("  %s FOUND BY MOTION at %.0f, %.0f cm (+- %.1f)%s%s, by its %s; %s" % (
                n, f["where"][0], f["where"][1], f["sigma"],
                (", facing %.0f deg" % f["heading"]) if f["heading"] is not None else " (heading not known yet)",
                (", roof tag %s moved with it" % f["tag"]) if f["tag"] is not None else ", no roof tag read moved with it",
                f["how"], ("its tag should read there" + ("" if f["tag"] is not None else
                                                          ", but none moved with it: evidence that it does not")) if f["readable"] else
                ("the camera does not read tags there" if f["readable"] is False else "readability not known")))
    return {"robots": out, "coverage": rec, "version": VERSION}


def tag_mover_from_eye(eye, stroke_cm=None):
    """tag_moved(name, mode, t0, t1) from a calib_run Eye: the tag whose reads moved by about the stroke (or turned by
    about the wiggle) between just before t0 and the forward end."""
    import camera_discovery as CD
    L = CD.nudge_cm() if stroke_cm is None else float(stroke_cm)

    def f(name, mode, t0, t1):
        best = None
        for tid in list(getattr(eye, "hist", {}).keys()):
            rs = eye.window(tid, t0 - 3.0, t1 + 0.5)
            pre = [r for r in rs if r[0] <= t0 + 0.2]
            mid = [r for r in rs if t0 + 0.2 < r[0] <= t1]
            if not pre or not mid:
                continue
            p0 = pre[-1]
            if mode == "wiggle":
                d = max(abs(wrap180(r[3] - p0[3])) for r in mid)
                score = d if d > 0.5 * CD.WIGGLE_DEG else None
            else:
                d = max(math.hypot(r[1] - p0[1], r[2] - p0[2]) for r in mid)
                score = d if d > 0.5 * L else None
            if score is not None and (best is None or score > best[0]):
                best = (score, tid)
        return None if best is None else int(best[1])
    return f


# ---------------------------------------------------------------------------------------------- 4. bring back
class BringBack:
    """Drive a robot found where its tag does not read back to where the readability map says tags read, in short
    straight steps each followed by motion, under the never-give-up rule. The run supplies:
      step(cm) -> {"where": (x, y) end, "heading": deg or None, "sigma": cm} or None (no camera saw the step)
      turn(deg) -> the degrees its own gyro turned (or None)
      wiggle() -> {"where": (x, y), "sigma": cm} or None
      problems(a, b, buf) -> [] when the robot's middle can go a -> b with buf cm of possible error clear of every edge
                             and keep-out (calib_run.Site.problems)
      readable(x, y) -> read rate 0-1 (the readability map for this robot's tag and today's light)
      tag_near(x, y, r) -> a tag id read within r cm, or None
      hold(words) -> called (repeatedly) while it waits for a hand: it must sleep a little and return
      stop() -> True when Stop / Ctrl+C was pressed"""

    def __init__(self, name, pose, sigma, step, turn, wiggle, problems, readable, tag_near, hold, stop=lambda: False,
                 log=say, ev=lambda kind, **kw: None, cells=None, stop_cm=1.4, uncal=True, ok_rate=0.6, proven=None,
                 target_ok=None):
        self.name = name
        self.x, self.y = float(pose[0]), float(pose[1])
        self.th = None if pose[2] is None else float(pose[2])
        self.sigma0 = float(sigma)
        self.blind_cm, self.blind_deg, self.after = 0.0, 0.0, 0.0
        self.step_f, self.turn_f, self.wiggle_f = step, turn, wiggle
        self.problems, self.readable, self.tag_near, self.hold = problems, readable, tag_near, hold
        self.stop, self.log, self.ev = stop, log, ev
        self.cells = cells             # [(x, y)] candidate spots (the readability map's cell middles)
        # proven(x, y) -> the read rate EVIDENCE shows there at this light (other runs, this run's legs, the other
        # robots' tags read just now), or None where there is none. Only proven spots count towards "no tag": the
        # camera model's guess is not evidence (10 Nov rehearsal: a dim patch the model calls readable made a robot
        # WITH a tag look tagless after two spots). None given: the map's rate (the rig and tests)
        self.proven = proven
        # target_ok(x, y): a spot fit to drive TO (the site's planning margins: 13 cm from the edge at school); the
        # never-give-up rule only governs the way there (rehearsal: a robot sought tags 10 cm from the edge)
        self.target_ok = target_ok
        self.stop_cm, self.ok_rate = stop_cm, ok_rate
        self.per = BLIND_PER_CM_UNCAL if uncal else BLIND_PER_CM
        self.tf = TURN_FRAC_UNCAL if uncal else 0.03
        self.failed = [(self.x, self.y)]          # where it stood and its tag did not read
        self.tried = []                           # targets reached with no read (seed-readable, not proven)
        self.blocked = []                         # targets another robot or a keep-out stands in the way of
        self.track = [(round(self.x, 1), round(self.y, 1), "found")]
        self.steps = self.sightings = self.wiggles = self.holds = 0
        self._since_wiggle = 0
        self.p_miss = 1.0
        self._inward_left, self._last_tgt = None, None

    def n_inside(self):
        return len(self.failed) - 1

    def bubble(self, extra=0.0):
        """The error sum (cm) after `extra` more cm straight: the last sighting's own error + the drift per cm since +
        the sideways effect of the degrees turned since (its heading's doubt) over the cm driven after them."""
        return self.sigma0 + self.per * (self.blind_cm + extra) + \
            (self.after + extra) * math.sin(math.radians(min(60.0, self.tf * self.blind_deg)))

    def target(self):
        cand = []
        for (cx, cy) in (self.cells or []):
            if self.readable(cx, cy) < self.ok_rate:
                continue
            if any(math.hypot(cx - fx, cy - fy) < FAILED_R_CM for fx, fy in self.failed + self.tried) or \
                    any(math.hypot(cx - bx, cy - by) < 1.0 for bx, by in self.blocked):
                continue
            if self.problems((cx, cy), (cx, cy), 0.0) or (self.target_ok is not None and not self.target_ok(cx, cy)):
                continue
            pv = self.proven(cx, cy) if self.proven else None
            # spots the evidence shows readable first, then the nearest
            cand.append((0 if (pv is not None and pv >= self.ok_rate) else 1, math.hypot(cx - self.x, cy - self.y),
                         (cx, cy)))
        return min(cand)[2] if cand else None

    def sighted(self, s, why):
        self.x, self.y = float(s["where"][0]), float(s["where"][1])
        if s.get("heading") is not None:
            self.th = float(s["heading"])
            self.blind_deg = 0.0
        self.sigma0 = float(s.get("sigma") or 1.0)
        self.blind_cm, self.after = 0.0, 0.0
        self.sightings += 1
        if len(self.track) > 2000:
            del self.track[1:1000]          # a hold that lasts hours keeps its first and newest steps only
        if why != "wiggle":
            self._since_wiggle = 0
        self.track.append((round(self.x, 1), round(self.y, 1), why))

    def find_heading(self):
        """No heading yet (found by a wiggle): a stroke forward, only where clear both ways with the error sum."""
        import camera_discovery as CD

        def clear(L):
            # its heading is unknown: the stroke may point any way, so the whole circle of radius L must be clear
            for a in range(0, 360, 15):
                h = math.radians(a)
                q = (self.x + L * math.cos(h), self.y + L * math.sin(h))
                if self.problems((self.x, self.y), q, self.bubble(L) + self.stop_cm):
                    return False
            return True
        L = CD.nudge_cm()
        while L >= MIN_STROKE_CM and not clear(L):
            L -= 0.5                     # as long as its error sum allows (derived from the room, not a fixed length)
        if L < MIN_STROKE_CM:
            return False
        self.log("  %s: a %.1f cm stroke to find its heading (as long as its error sum allows here)" % (self.name, L))
        s = self.step_f(L)
        if s and s.get("heading") is not None:
            self.sighted(s, "heading stroke")
            b = self.step_f(-L)
            if b is not None and b.get("where") is not None:
                self.sighted(dict(where=b["where"], heading=self.th, sigma=b.get("sigma")), "heading stroke back")
            else:
                self.x -= L * math.cos(math.radians(self.th))
                self.y -= L * math.sin(math.radians(self.th))
                self.blind_cm += L
            return True
        if s is None:
            self.blind_cm += L         # it moved somewhere along an unknown heading and back: more doubt
        self.step_f(-L)
        return False

    def run(self):
        """Returns (x, y, heading, tag) once a tag reads near it; never gives up (Stop raises StopAsked)."""
        t0 = time.time()
        said = 0.0
        self.ev("bring_back_start", robot=self.name, at=[round(self.x, 1), round(self.y, 1)], sigma=self.sigma0)
        if self.readable(self.x, self.y) >= self.ok_rate:
            self.log("BRING BACK: no roof tag read on %s where it stands (%.0f, %.0f), where tags should read: it drives "
                     "to another spot where tags read, in %.0f cm steps followed by the motion detector (a tag that "
                     "reads there is its tag; none at two such spots: it carries no tag that reads)" % (
                         self.name, self.x, self.y, STEP_CM))
        else:
            self.log("BRING BACK: %s is where its roof tag does not read (%.0f, %.0f): it drives itself back to where "
                     "tags read, in %.0f cm steps followed by the motion detector, while its error sum cannot reach an "
                     "edge" % (self.name, self.x, self.y, STEP_CM))
        while True:
            if self.stop():
                raise StopAsked("stopped (Ctrl+C or Mission Control's Stop) while %s was driving back" % self.name)
            t = self.tag_near(self.x, self.y, TAG_NEAR_CM + self.bubble())
            if t is not None and self._inward_left is None and self._last_tgt is not None and \
                    math.hypot(self._last_tgt[0] - self.x, self._last_tgt[1] - self.y) > STEP_CM:
                # (rehearsal: the tag first read at the very border of a dim patch, and the heading check's 6 cm nudge
                # took it back out) on into the readable ground by the heading check's nudge + its error sum
                import camera_discovery as CD
                self._inward_left = int(math.ceil((CD.nudge_cm() + self.bubble()) / STEP_CM))
            if t is not None and self._inward_left:
                self._inward_left -= 1
                t = None
            if t is not None:
                self.ev("bring_back_end", robot=self.name, tag=t, at=[round(self.x, 1), round(self.y, 1)],
                        steps=self.steps, sightings=self.sightings, wiggles=self.wiggles, holds=self.holds,
                        dur_s=round(time.time() - t0, 1), track=self.track)
                self.log("  %s: roof tag %s reads here (%.0f, %.0f) after %d steps (%d motion sightings, %.0f s): back "
                         "where it is seen" % (self.name, t, self.x, self.y, self.steps, self.sightings, time.time() - t0))
                return self.x, self.y, self.th, t
            if self.th is None:
                if not self.find_heading():
                    self._hold("its heading is not known and a stroke to find it is not clear here")
                continue
            tgt = self.target() if not (self._inward_left and self._last_tgt) else self._last_tgt
            self._last_tgt = tgt
            if tgt is None:
                self._hold("no spot where tags read is known that it can reach (it may carry no tag, or the light may be "
                           "too low for its tag: add light, or check its roof)")
                continue
            want = wrap180(math.degrees(math.atan2(tgt[1] - self.y, tgt[0] - self.x)) - self.th)
            if abs(want) > 3.0:
                if self.problems((self.x, self.y), (self.x, self.y), self.bubble() + self.stop_cm):
                    self._hold("a turn here: its error sum (%.1f cm) + body could reach an edge" % self.bubble())
                    continue
                got = self.turn_f(want)
                d = want if got is None else got
                self.th = wrap180(self.th + d)
                self.blind_deg += abs(d)
                continue
            dist = math.hypot(tgt[0] - self.x, tgt[1] - self.y)
            if dist < 2.0:
                # reached the target and no tag read: not that spot again (rehearsal: it circled a target 0.5 cm at a
                # time in a dim patch, its steps too short for the motion detector)
                self.tried.append(tgt)
                continue
            step = min(STEP_CM, dist + 0.5)
            h = math.radians(self.th)
            nxt = (self.x + step * math.cos(h), self.y + step * math.sin(h))
            why = self.problems((self.x, self.y), nxt, self.bubble(step) + self.stop_cm)
            if why and not str(why[0]).startswith("the table's edge"):
                # another robot or a keep-out in the way (rehearsal: a spot beside a parked robot): another spot, no hold
                self.blocked.append(tgt)
                self.log("  %s: the way to %.0f, %.0f is blocked (%s): another spot" % (self.name, tgt[0], tgt[1],
                                                                                       why[0]))
                continue
            if why:
                self._hold("the next step's error sum (%.1f cm) + its stop (%.1f cm) could reach an edge (%s)" % (
                    self.bubble(step), self.stop_cm, why[0]))
                continue
            s = self.step_f(step)
            self.steps += 1
            if s is not None and s.get("where") is not None:
                self.sighted(s, "step")
            else:
                self.x, self.y = nxt
                self.blind_cm += step
                self.after += step
                self.track.append((round(self.x, 1), round(self.y, 1), "dead reckoning"))
            r_ = self.proven(self.x, self.y) if self.proven else self.readable(self.x, self.y)
            if r_ is not None and r_ >= self.ok_rate and \
                    self.tag_near(self.x, self.y, TAG_NEAR_CM + self.bubble()) is None:
                time.sleep(TAG_WAIT_S)               # a read takes a picture or two (7 a second in dim light)
                if self.tag_near(self.x, self.y, TAG_NEAR_CM + self.bubble()) is None:
                    if all(math.hypot(self.x - fx, self.y - fy) >= FAILED_R_CM for fx, fy in self.failed):
                        self.p_miss *= max(0.0, 1.0 - r_)     # an independent readable cell, and still no tag
                        self.failed.append((self.x, self.y))
                    if self.p_miss < TAGLESS_P:
                        self.ev("tagless", robot=self.name, at=[round(self.x, 1), round(self.y, 1)], p=self.p_miss)
                        self.log("  %s: NO ROOF TAG READS ON IT: it stood on %d spot%s where tags are proven to read, "
                                 "with no tag read (a tag on it would have been missed at all of them with a chance "
                                 "under %.0f %%): it runs on motion, lock-on and its own odometry" % (
                                     self.name, self.n_inside(), "" if self.n_inside() == 1 else "s", 100 * TAGLESS_P))
                        return self.x, self.y, self.th, None
            if time.time() - said > 20.0:
                said = time.time()
                self.log("  %s: %d steps, at about %.0f, %.0f (error sum %.1f cm)" % (
                    self.name, self.steps, self.x, self.y, self.bubble()))

    def _hold(self, why):
        """Held still and safe: a find-me wiggle (the motion detector's sighting resets the error sum); when that
        cannot help, NEEDS A HAND, and the run's hold waits a little (never gives up)."""
        self.holds += 1
        # a wiggle first while it can still help: its error sum has grown since the last sighting, or the last
        # wiggles were few (each sighting can move the estimate, and with it what is clear)
        fresh = self.blind_cm > 0 or self._since_wiggle < 2
        if self.problems((self.x, self.y), (self.x, self.y), self.bubble()) == [] and fresh:
            self.wiggles += 1
            self._since_wiggle += 1
            w = self.wiggle_f()
            if w is not None and w.get("where") is not None:
                self.log("  %s: held (%s); its find-me wiggle was seen at %.0f, %.0f: error sum reset" % (
                    self.name, why, w["where"][0], w["where"][1]))
                self.sighted(dict(where=w["where"], heading=None, sigma=w.get("sigma")), "wiggle")
                return
        self.ev("needs_a_hand", robot=self.name, why=why, at=[round(self.x, 1), round(self.y, 1)])
        self.hold("%s NEEDS A HAND: %s. It is held, still and safe, at about %.0f, %.0f cm. Move it a hand's width "
                  "towards the middle of the table (or click it on a camera's picture): it carries on by itself" % (
                      self.name, why, self.x, self.y))
        w = self.wiggle_f() if self.problems((self.x, self.y), (self.x, self.y), self.bubble()) == [] else None
        if w is not None and w.get("where") is not None:
            self.sighted(dict(where=w["where"], heading=None, sigma=w.get("sigma")), "wiggle after a hand")


class StopAsked(Exception):
    pass


def site_problems(S):
    """problems(a, b, buf) for a robot found by motion, on a calib_run.Site: the table's EDGES (and drops the proven
    workspace knows) with the whole error sum (a fall); the other robots with it too (they are solid); the flat
    keep-outs (reference tiles, outlet plates) with the body's own clearance only, and a robot already nearer one than
    that (put down there) may move only away from it. The camera's typed view limit is not used: a robot found by
    motion beyond it is seen there (observe, then trust)."""
    import calib_run as CR

    def seg(a, b, k=10):
        return [(a[0] + (b[0] - a[0]) * i / float(k), a[1] + (b[1] - a[1]) * i / float(k)) for i in range(1, k + 1)]

    def f(a, b, buf):
        # the edge rule of "never give up" (CLAUDE.md, Nick 12:51): the error sum + half the body's diagonal must stay
        # inside every edge (and every drop the proven workspace knows) along the whole step; not the site's typed
        # margin, which is a planning allowance for robots whose position the camera knows
        for p in [a] + seg(a, b):
            e = S.edge_dist(p)
            if e - buf < HALF_DIAG_CM:
                return ["the table's edge (%.1f cm from it; error sum %.1f + half the body %.1f)" % (e, buf,
                                                                                                    HALF_DIAG_CM)]
        for k in S.keepouts:
            d0 = CR.poly_dist(a, k["polygon"])
            for p in seg(a, b):
                d = CR.poly_dist(p, k["polygon"])
                if k.get("kind") == "robot":
                    if d - buf < CR.KEEPOUT_CLEAR_CM - 0.5:
                        return ["%s (%.1f cm, %.1f with its error sum)" % (k["name"], d, d - buf)]
                elif d < CR.KEEPOUT_CLEAR_CM - 0.5 and d < d0 - 0.05:
                    return ["%s (%.1f cm, closer than where it stands)" % (k["name"], d)]
        return []
    return f


# ---------------------------------------------------------------------------------------------- the simulator's pictures
class SimPictures:
    """(e14) The simulator's cameras drawn from its true poses (tools/sim/tag_cam: the real drawing, blur and noise),
    so robot finding by motion runs the real motion analysis on simulated pictures. truth() -> {name: (x, y, th deg)};
    tags {name: id or None}; unread() -> set of names whose tag the simulated camera does not read now (drawn with a
    plain roof); darks [(x, y, r, factor)]: patches drawn darker (a shadow, a lamp off); extra_blind: one more camera
    looking at the wall."""

    def __init__(self, truth, tags, table, cams=None, darks=(), unread=None, extra_blind=False, size=(1280, 960),
                 seed=7, tagcams=None):
        import tag_cam as TCAM
        self.truth, self.tags, self.unread = truth, dict(tags), unread or (lambda: set())
        L, W = table
        self.table = (0.0, 0.0, float(L), float(W))
        # darks: (x, y, r, factor) circles, or (x0, y0, x1, y1, "rect", factor) rectangles (mission_run's --sim-dark)
        self.darks = [tuple(d) for d in darks]
        if tagcams:
            # (e14) the mission simulator's own cameras (tag_cam.TagCamera), drawn as they are
            self.cams = list(tagcams)
            self.dark_masks = {}
            if extra_blind:
                self.cams.append(TCAM.TagCamera((L / 2, -80.0, 60.0), (L / 2, -400.0, 80.0), size=size, hfov_deg=60,
                                                name="laptop webcam", table=self.table, seed=seed + 9, jpeg=0))
            return
        specs = cams or [dict(name="overhead", pos=(-45 * 2.54, W / 2.0, 145.0), look=(150.0, W / 2.0, 0.0),
                              hfov=70.4)]
        self.cams = []
        for k, sp in enumerate(specs):
            self.cams.append(TCAM.TagCamera(sp["pos"], sp["look"], size=size, hfov_deg=sp.get("hfov", 70.0),
                                            name=sp["name"], table=self.table, seed=seed + k, jpeg=0))
        if extra_blind:
            self.cams.append(TCAM.TagCamera((L / 2, -80.0, 60.0), (L / 2, -400.0, 80.0), size=size, hfov_deg=60,
                                            name="laptop webcam", table=self.table, seed=seed + 9, jpeg=0))
        self.dark_masks = {}

    def _dark(self, cam, img):
        if not self.darks:
            return img
        m = self.dark_masks.get(cam.name)
        if m is None:
            import cv2
            m = np.ones(img.shape, np.float32)
            for d in self.darks:
                if len(d) == 6 and d[4] == "rect":
                    x0, y0, x1, y1, _k, f = d
                    pts = [[x0, y0, 0.0], [x1, y0, 0.0], [x1, y1, 0.0], [x0, y1, 0.0]]
                else:
                    x, y, r, f = (float(v) for v in d)
                    pts = [[x + r * math.cos(a), y + r * math.sin(a), 0.0] for a in np.linspace(0, 2 * math.pi, 36)]
                uv = cam.project(pts)
                if np.all(np.isfinite(uv)):
                    mm = np.zeros(img.shape, np.uint8)
                    cv2.fillPoly(mm, [np.round(uv).astype(np.int32)], 1)
                    m[mm > 0] = f
            self.dark_masks[cam.name] = m
        return np.clip(img.astype(np.float32) * m, 0, 255).astype(np.uint8)

    def picture(self, cam):
        rob, cov = [], set()
        un = self.unread()
        for k, (n, p) in enumerate(sorted(self.truth().items())):
            tid = self.tags.get(n)
            rid = 900 + k if tid is None or tid < 0 else int(tid)
            if tid is None or tid < 0 or n in un:
                cov.add(rid)
            rob.append(dict(x=float(p[0]), y=float(p[1]), th=math.radians(float(p[2])), z=6.19, family="tag25h9",
                            id=rid, tag_cm=8.0, roof_cm=9.5))
        return self._dark(cam, cam.render(rob, covered=tuple(cov)))

    def caminfos(self, models=True):
        import camera_discovery as CD
        out = []
        for c in self.cams:
            out.append(CD.CamInfo(c.name, (lambda c=c: self.picture(c)),
                                  c.true_model() if (models and c.name != "laptop webcam") else None))
        return out


# ---------------------------------------------------------------------------------------------- runs' glue
def locate_step(cams, drive, cm, frames=4):
    """Drive cm straight (drive(cm, on_tick, n)) with every camera watching: {"where": its END (x, y), "heading": the
    direction it moved (deg; its nose for cm > 0), "sigma": cm} or None when no camera saw it move."""
    import camera_discovery as CD
    CD.still(cams, n=3)
    rec = {c.name: [] for c in cams}

    def snap(phase, k):
        for c in cams:
            g = c.grab()
            if g is not None:
                rec[c.name].append((phase, k, g))
    snap("pre", 0)
    drive(cm, lambda k: snap("fwd", k), frames)
    snap("fwd_end", 0)
    best = None
    for c in cams:
        r = CD.analyse(c, rec[c.name], abs(cm))
        if r.get("seen") and r.get("end_cm") is not None and (best is None or r.get("body_px", 0) > best.get("body_px", 0)):
            best = r
    if best is None:
        return None
    h = best.get("heading_deg")
    if h is not None and cm < 0:
        h = wrap180(h + 180.0)
    return {"where": tuple(best["end_cm"]), "heading": h, "sigma": sigma_of(best)}


def locate_wiggle(cams, turn, deg=None, frames=4):
    """A find-me wiggle with every camera watching: {"where": (x, y), "sigma": cm} or None."""
    import camera_discovery as CD
    deg = CD.WIGGLE_DEG if deg is None else deg
    CD.still(cams, n=3)
    fr = CD.wiggle_record(cams, lambda _r, d, on_tick, n: turn(d, on_tick, n), None, deg, frames)
    best = None
    for c in cams:
        r = CD.analyse_wiggle(c, fr[c.name])
        if r.get("seen") and r.get("where_cm") is not None and (best is None or r.get("body_px", 0) >
                                                               best.get("body_px", 0)):
            best = r
    return None if best is None else {"where": tuple(best["where_cm"]), "sigma": sigma_of(best)}


TAGLESS_P = 0.01         # CHOSEN (e14): a robot driven back into readable ground is taken to have NO roof tag that reads
                         # (a plain roof, a covered tag) once the chance that a tag on it went unread at every readable
                         # spot it stood on, the product of (1 - read rate) there, is under 1 %. DOE row (e14)


def run_finding(robots, eye, cams, S, rm, confirmed, guesses=None, log=say, ev=lambda kind, **kw: None,
                stop=lambda: False, hold_say_s=60.0, turn_dps=60.0, final_cms=5.0, folder=None, rm_tag=None):
    """(e14) The finding at the start of a run, shared by the calibration (calib_run) and the missions' and free
    roam's binding (wiggle_bind.bind_for_run): each robot's stroke or wiggle (modes_for), the coverage, the tag that
    moved with each; every robot found where no tag moved with it driven back to where tags read (BringBack), or found
    to carry no tag that reads. robots: {name: calib_run.Robot}; eye: calib_run.Eye (its tag reads); S: calib_run.Site;
    rm: the readability map (observe.ReadMap; rm_tag(tag) -> that tag's own map, optional). Returns (pre {name: tag
    id} for the robots whose own stroke moved their tag, res: the record, with "brought_back" and "tagless")."""
    import camera_discovery as CD
    import safety_rules as SR_
    names = sorted(robots)
    for rb in robots.values():
        rb.set("e02_cms", CD.DISC_SPEED)
    modes = modes_for(names, confirmed, guesses or {}, lambda a_, b_: S.problems(a_, b_, buf=2.0, edge_buf=2.0))
    res = find(cams, names, CD.leg_mover(robots), CD.leg_turner(robots, turn_dps), modes=modes,
               tag_moved=tag_mover_from_eye(eye), readable=lambda x, y: rm.readable(x, y)[0], log=log)
    res["confirmed"] = sorted(confirmed) if isinstance(confirmed, set) else confirmed
    res["modes"] = modes
    ev("found_by_motion", robots=res["robots"], modes=modes, cameras=res["coverage"].get("cameras"))
    pre = {n: int(f["tag"]) for n, f in res["robots"].items() if f["found"] and f["tag"] is not None}
    res["tagless"] = []
    for n, t in pre.items():
        w_ = (res["robots"][n] or {}).get("where")
        if w_:
            for _ in range(3):
                rm.add(w_[0], w_[1], True)    # its tag read there just now, at this light: evidence

    def proven(x, y):
        """The read rate the EVIDENCE shows in (x, y)'s own cell (3+ tries), else None. Not its neighbours' (practice:
        a robot just past the camera's reach was called tagless on the evidence of the cell beside it)."""
        k = rm.ij(x, y)
        n_, r_ = float(rm.tries.get(k, 0)), float(rm.reads.get(k, 0))
        return ((r_ + 1.0) / (n_ + 2.0)) if n_ >= 3 else None    # (Laplace: 3 of 3 reads is 0.8, not certain)
    try:
        for n, f in res["robots"].items():
            if not f["found"]:
                log("  %s was not found by motion: the wiggle binding looks for its tag" % n)
                continue
            if f["tag"] is not None:
                continue
            others = {o: tuple(g["where"]) for o, g in res["robots"].items() if o != n and g.get("where")}
            S.set_parked(others)
            rb = robots[n]
            cells = [rm.centre(i, j) for i in range(rm.nx) for j in range(rm.ny)]
            said = [0.0]

            def drive(cm, on_tick, k, n=n, rb=rb):
                return CD.leg_mover({n: rb})(n, cm, on_tick, k)

            def turn(deg, rb=rb):
                r = rb.leg("turn", deg, on_tick=stop)
                if r.get("start") and r.get("end"):
                    return wrap180(r["end"][2] - r["start"][2])
                return None

            def turn_ticks(deg, on_tick, k, n=n, rb=rb):
                return CD.leg_turner({n: rb}, turn_dps)(n, deg, on_tick, k)

            def tag_near(x, y, r):
                for t in eye.seen_ids(1.0):
                    if t in pre.values():
                        continue
                    l_ = eye.last(t)
                    if l_ is not None and math.hypot(l_[1] - x, l_[2] - y) <= r:
                        return int(t)
                return None

            def hold(words):
                if time.time() - said[0] >= hold_say_s:
                    said[0] = time.time()
                    log("!! " + words)
                    ev("needs_a_hand", words=words)
                time.sleep(1.0)
            rb.set("e02_cms", final_cms)
            bb = BringBack(n, (f["where"][0], f["where"][1], f["heading"]), f["sigma"],
                           step=lambda cm, drive=drive: locate_step(cams, drive, cm), turn=turn,
                           wiggle=lambda turn_ticks=turn_ticks: locate_wiggle(cams, turn_ticks),
                           problems=site_problems(S), readable=lambda x, y: rm.rate(x, y), tag_near=tag_near,
                           hold=hold, stop=stop, log=log, ev=ev, cells=cells, stop_cm=float(SR_.stop_cm(final_cms)),
                           uncal=True, proven=proven,
                           target_ok=lambda x, y: not [q for q in S.problems((x, y), (x, y)) if "camera's view" not in q])
            x, y, th, t = bb.run()
            f["now"] = [round(x, 1), round(y, 1)]
            f["heading_now"] = th
            f["brought_back"] = dict(tag=t, steps=bb.steps, sightings=bb.sightings, wiggles=bb.wiggles,
                                     holds=bb.holds, track=bb.track, failed=bb.failed)
            ev("brought_back", robot=n, tag=t, at=[round(x, 1), round(y, 1)], steps=bb.steps, sightings=bb.sightings,
               wiggles=bb.wiggles, track=bb.track)
            if t is None:
                res["tagless"].append(n)
                continue
            # the evidence: where it was found its tag did not read (3 tries, as a skipped point)
            fx, fy = bb.failed[0]
            for m in [rm] + ([rm_tag(t)] if rm_tag else []):
                if m is not None:
                    for _ in range(3):
                        m.add(fx, fy, False)
    finally:
        S.set_parked({})
    if folder:
        save(res, folder)
    return pre, res


def save(res, folder):
    os.makedirs(folder, exist_ok=True)
    p = os.path.join(folder, "found_by_motion.json")
    with open(p, "w") as f:
        json.dump(res, f, indent=1, default=str)
    return p


# ---------------------------------------------------------------------------------------------- a self-contained rig
class _RigWorld:
    """A tiny world for the rig and tests: robots that move instantly (no fleet_sim), a readability rule, tags that
    read only where readable."""

    def __init__(self, robots, tags, readable_fn, table=(396.2, 167.6)):
        self.p = {n: list(v) for n, v in robots.items()}
        self.tags, self.readable_fn, self.table = dict(tags), readable_fn, table

    def truth(self):
        return {n: tuple(v) for n, v in self.p.items()}

    def unread(self):
        return {n for n, v in self.p.items() if not self.readable_fn(v[0], v[1])}

    def mover(self, robot, cm, on_tick, n):
        p = self.p[robot]
        h = math.radians(p[2])
        for k in range(1, n + 1):
            p[0] += cm / n * math.cos(h)
            p[1] += cm / n * math.sin(h)
            on_tick(k)

    def turner(self, robot, deg, on_tick, n):
        p = self.p[robot]
        for k in range(1, n + 1):
            p[2] += deg / n
            on_tick(k)

    def tag_near(self, x, y, r):
        for n, v in self.p.items():
            t = self.tags.get(n)
            if t is not None and t >= 0 and self.readable_fn(v[0], v[1]) and math.hypot(v[0] - x, v[1] - y) <= r:
                return t
        return None


def run_rig(log=say, dark=None, extra_blind=True, close=True):
    """The simulator rig (Nick's cases): Charlie at P15 (179, 146) where its tag never reads (the camera reads tags
    only in x < 150 here), Delta in a dark patch, Alpha and Bravo 25 cm apart, an extra camera looking at the wall.
    Each robot is found by its stroke; Charlie is brought back to where its tag reads. Returns the record."""
    import calib_run as CR
    import sites as SI
    site = SI.load("school-conference")
    S = CR.Site(site)
    readable_fn = lambda x, y: x < 150.0 and not (dark and math.hypot(x - dark[0], y - dark[1]) <= dark[2])  # noqa: E731
    robots = {"Alpha": [60.0, 60.0, 0.0], "Bravo": [60.0, 85.0, 90.0], "Charlie": [179.0, 146.0, 90.0]}
    tags = {"Alpha": 2, "Bravo": 3, "Charlie": 4}
    if dark:
        robots["Delta"] = [dark[0], dark[1], 45.0]
        tags["Delta"] = 5
    if not close:
        robots["Bravo"] = [60.0, 120.0, 90.0]
    W = _RigWorld(robots, tags, readable_fn)
    sp = SimPictures(W.truth, tags, (S.L, S.W), darks=[dark] if dark else (), unread=W.unread, extra_blind=extra_blind)
    cams = sp.caminfos()
    t_reads = {}

    def tag_moved(n, mode, t0, t1):
        return tags[n] if readable_fn(*W.p[n][:2]) else None
    res = find(cams, sorted(robots), W.mover, W.turner, modes=modes_for(sorted(robots), True),
               tag_moved=tag_moved, readable=lambda x, y: readable_fn(x, y), log=log)
    res["truth_start"] = {n: list(v) for n, v in robots.items()}
    import observe as OB
    cells = []
    rm = OB.ReadMap(S.L, S.W)
    for i in range(rm.nx):
        for j in range(rm.ny):
            cells.append(rm.centre(i, j))
    back = {}
    for n, f in res["robots"].items():
        if not f["found"] or f["readable"] or f["tag"] is not None:
            continue
        S.set_parked({o: W.p[o][:2] for o in robots if o != n})
        bb = BringBack(n, (f["where"][0], f["where"][1], f["heading"]), f["sigma"],
                       step=lambda cm, n=n: locate_step(cams, lambda c, t, k: W.mover(n, c, t, k), cm),
                       turn=lambda d, n=n: (W.turner(n, d, lambda k: None, 1), d)[1],
                       wiggle=lambda n=n: locate_wiggle(cams, lambda d, t, k: W.turner(n, d, t, k)),
                       problems=site_problems(S),
                       readable=lambda x, y: 0.9 if readable_fn(x, y) else 0.1, tag_near=W.tag_near,
                       hold=lambda w: (log("!! " + w), time.sleep(0.01)), cells=cells, log=log)
        x, y, th, t = bb.run()
        back[n] = dict(at=[round(x, 1), round(y, 1)], truth=[round(W.p[n][0], 1), round(W.p[n][1], 1)], tag=t,
                       steps=bb.steps, sightings=bb.sightings, track=bb.track)
    res["brought_back"] = back
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd")
    s = sp.add_parser("sim")
    s.add_argument("--out", default="")
    a = ap.parse_args(argv)
    if a.cmd == "sim":
        res = run_rig(dark=(100.0, 40.0, 25.0, 0.35))
        if a.out:
            save(res, a.out)
        return 0
    ap.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
