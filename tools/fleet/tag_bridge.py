#!/usr/bin/env python3
"""Tag bridge (conference-table build, 2026-11-02): the cameras' pictures -> each robot's pose from its roof tag ->
camera fixes for run_plan.py (--fixes udp), the same messages the lights bridge (cam_bridge.py) sends.

What is different from the lights bridge, and why it is simpler:
  identity   the tag's id says which robot it is (no prediction needed to tell robots apart, no lights mix-ups);
  heading    every fix carries the tag's heading (the robots' lights sit under the roof and cannot be seen);
  anywhere   a robot is found wherever it is put down (no floor memory, no seeds needed);
  height     each corner's ray meets the roof's plane (tag height), not the table (tag_geom.py).

Per camera, a thread keeps only the newest picture (a slow moment never builds a queue), and a worker looks for each
robot's tag in a window round where it was (or where its own pose, run_plan's echo on --pose-port, says it is), the
whole picture only for a tag that is lost (tagcore.WindowTracker). The two cameras' reads of one robot within
PAIR_S of each other are fused (inverse-variance); a fix goes out at most FIX_HZ times a second per robot.

A read is refused (not sent) when its size on the roof's plane is off by more than SIZE_REFUSE (a wrong model there,
or something that is not our tag), or it jumps more than JUMP_CM from the robot's last pose within a second and the
next read does not confirm it.

    python3 tools/fleet/tag_bridge.py --site school-conference --cal-dir ~/.config/mirlt/sites/school-conference/tagcal
    python3 tools/fleet/tag_bridge.py --tags tags.json --cam overhead=127.0.0.1:8765,cal1.json --cam side=...

Bumped cameras (2026-11-03): with reference tags on the table (ref_tags.py) and their places from the setup
(site_fit.json beside the camera models), every ref_tags.BUMP_CHECK_S each camera's reference tags are looked for in
small windows where the setup saw them. When their median shift is over ref_tags.BUMP_PX for BUMP_CONFIRM checks in a
row, the camera has moved: a WARNING line, a camera_moved event (mission_run says it on the screen), ref_watch.csv.

Measured roofs (2026-11-03): roof_tags.json beside the camera models (the setup's or a used refit's, roof_cal.py) gives
each robot its own tag height, tilt and offset from the turning point; tag_geom.pose_from_detection uses them. The
site's tags.roof_measured / tags.tilt_correct switch them off (the typed/placeholder height for every robot, flat).
They are said at start and logged (events.jsonl "start", bridge_summary.json "roofs").

Writes to --out: track.csv (as cam_bridge's, for line_check.py), tag_poses.csv (every read: camera, robot, pose,
sigma, size check, picture position), tag_obs.csv (every read's corners: the site fit's input), ref_watch.csv,
events.jsonl, bridge_summary.json.
"""
VERSION = "2026-11-10a"   # 2026-11-10a (e12): frame_watch (no pictures from a camera: said loudly, an event, a message to the run); workers joined before the summary; orderly exit. No version string before

import argparse
import csv
import datetime as dt
import json
import math
import os
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, os.path.join(TOOLS, "live"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np       # noqa: E402

import tagcore as TC     # noqa: E402
import tag_geom as TG    # noqa: E402

CAPTURE_S = 0.04         # as cam_bridge.CAPTURE_S: the recorder stamps a picture one picture late
FIX_HZ = 10.0            # fixes per robot per second at most (the robots blend 5-10 a second, DESIGN.md). No DOE
PAIR_S = 0.07            # two cameras' reads this close in time are one moment (fused). Chosen: one picture at 15 a
                         # second. No DOE: a definition
SIZE_REFUSE = 0.20       # a read 20 % off the printed size on the roof's plane is not trusted. Chosen: 3x the 6 % warn
JUMP_CM = 25.0           # a read this far from the robot's last pose (within JUMP_S) needs a second read to agree.
JUMP_S = 1.0             # Chosen: a robot at 15 cm/s goes 15 cm in a second. No DOE: a guard against a misread id
POSE_PORT = 5011
STATUS_EVERY_S = 2.0
SIGMA_FLOOR_CM = 0.3     # a fix's sigma is never said to be better than this (the tag's placement over the axle,
                         # measured by Nick). No DOE: a floor
SIGMA_DEG_FLOOR = 1.0    # the same for the heading (a hand-placed sticker on the roof)


def load_tags(path=None, site=None):
    """{family, tag_cm, roof_cm, height_cm, ids {robot: id}} from a JSON file or a site profile."""
    if path:
        with open(path) as f:
            d = json.load(f)
    else:
        import sites as S
        s = S.load(site)
        d = dict(s.get("tags") or {})
    if d.get("height_cm") is None:
        d["height_cm"] = 6.0
        d["height_note"] = ("the roof's height was not measured yet: 6.0 cm used (PLACEHOLDER). Measure from the table "
                            "to the tag's printed face and put it in the site profile (tags.height_cm)")
    return d


class CamFeed(threading.Thread):
    """Newest picture of one camera stream (bytes kept, decoded only when the worker takes it)."""

    def __init__(self, name, url):
        super().__init__(daemon=True)
        import sources as S
        self.name = name
        self.src = S.StreamSource(url, log=lambda s: print("%s camera: %s" % (name, s), flush=True))
        self.cond = threading.Condition()
        self.latest = None        # (frame no, t_pi, jpeg bytes, t_arrived)
        self.n = 0
        self.skipped = 0
        self.stop = False
        self.on_frame = None      # (e12) called with the camera's name for every picture (frame_watch.FrameWatch.saw)

    def run(self):
        k = 0
        while not self.stop:
            try:
                self.src._connect()
                for h, body, arrived in self.src.parts():
                    if self.stop:
                        return
                    pf = int(h.get("x-frame", k))
                    k += 1
                    with self.cond:
                        if self.latest is not None:
                            self.skipped += 1
                        self.latest = (pf, float(h.get("x-time", arrived)), body, arrived)
                        self.n += 1
                        self.cond.notify_all()
                    if self.on_frame is not None:
                        self.on_frame(self.name)           # (e12) frame_watch: a new picture came
            except (OSError, ConnectionError, KeyError, ValueError) as e:
                if self.stop:
                    return
                print("%s camera stream: %s; trying again in 1 s" % (self.name, e), flush=True)
                time.sleep(1.0)
            finally:
                try:
                    if self.src.sock is not None:
                        self.src.sock.close()
                except OSError:
                    pass
                self.src.sock = None

    def take(self, timeout=0.5):
        with self.cond:
            if self.latest is None:
                self.cond.wait(timeout)
            item, self.latest = self.latest, None
            if item is not None:
                self.skipped = max(0, self.skipped)
            return item

    def close(self):
        self.stop = True
        try:
            if self.src.sock is not None:
                self.src.sock.close()
        except OSError:
            pass


class RefWatch:
    """One camera's reference tags against where the setup saw them (a bumped camera). expected {id: corners 4x2}."""

    def __init__(self, cam_name, expected, det, every_s=None, bump_px=None, confirm=None):
        import ref_tags as RT
        self.cam = cam_name
        self.expected = {str(i): np.asarray(v, float) for i, v in expected.items()}
        self.det = det
        self.every_s = RT.BUMP_CHECK_S if every_s is None else every_s
        self.bump_px = RT.BUMP_PX if bump_px is None else bump_px
        self.confirm = RT.BUMP_CONFIRM if confirm is None else confirm
        self.t_last = -1e18
        self.over = 0
        self.moved = False
        self.last = None

    def due(self, t):
        return t - self.t_last >= self.every_s

    def check(self, grey, t):
        """(shift px median or None, {id: shift px}, verdict 'ok' / 'moved' / 'back' / None)."""
        self.t_last = t
        shifts = {}
        for i, e in self.expected.items():
            c = e.mean(0)
            size = float(np.mean([np.linalg.norm(e[k] - e[(k + 1) % 4]) for k in range(4)]))
            for d in self.det.detect_window(grey, c[0], c[1], 1.2 * size + 3 * self.bump_px, upscale=1.0):
                if str(d.id) == i:
                    shifts[i] = float(np.median(np.linalg.norm(d.corners - e, axis=1)))
                    break
        if not shifts:
            self.last = (None, shifts)
            return None, shifts, None
        med = float(np.median(list(shifts.values())))
        self.last = (med, shifts)
        verdict = "ok"
        if med > self.bump_px:
            self.over += 1
            verdict = "still moved" if self.moved else "over"
            if self.over >= self.confirm and not self.moved:
                self.moved = True
                verdict = "moved"
        else:
            self.over = 0
            if self.moved:
                self.moved = False
                verdict = "back"
        return med, shifts, verdict


def ref_expected(models, site_fit_path, cfg=None):
    """{camera: {id: expected corners}} from the setup's reference tag places (site_fit.json) and the camera models;
    {} when the setup had no reference tags."""
    try:
        with open(site_fit_path) as f:
            rt = (json.load(f) or {}).get("ref_tags") or {}
    except (OSError, ValueError):
        return {}
    poses = rt.get("poses") or {}
    if not poses:
        return {}
    import site_fit as SF
    s, z = float(rt.get("tag_cm", 12.0)), float(rt.get("z_cm", 0.18))
    out = {}
    for c, m in models.items():
        seen = set((rt.get("seen") or {}).get(c) or poses)
        for i, (x, y, th) in poses.items():
            if str(i) not in seen:
                continue
            u, v = m.project(SF.tag_corners_world(x, y, th, z, s))
            uv = np.column_stack([u, v])
            if np.all(np.isfinite(uv)) and np.all(m.in_image(uv[:, 0], uv[:, 1], margin=2)):
                out.setdefault(c, {})[str(i)] = uv
    return out


class Bridge:
    def __init__(self, tags, cams, send, out, observe=False, robots=None, say=None, ref_expected=None, ref_cfg=None,
                 roofs=None, lock=True, motion=False, board=None):
        """tags: load_tags(); cams: {name: CameraModel}; send(msg). roofs: {tag id: roof dict} (roof_cal.roofs_for_bridge;
        None or {}: every robot flat at tags.height_cm, as before)."""
        self.tags = tags
        self.roofs = {int(k): v for k, v in (roofs or {}).items()}
        self.ids = {int(i): n for n, i in tags["ids"].items() if robots is None or n in robots}
        self.robot_of = self.ids
        self.cams = cams
        self.send = send
        self.out = out
        self.observe = observe
        self.say_ = say or (lambda s: print(time.strftime("%H:%M:%S ") + s, flush=True))
        self.det = {c: TC.TagDetector(tags.get("family", TC.TAG_FAMILY), ids=list(self.ids)) for c in cams}
        self.trk = {c: TC.WindowTracker(self.det[c], list(self.ids)) for c in cams}
        self.own = {}              # robot -> (x, y, th, t) from run_plan
        self.last = {}             # robot -> (x, y, th, t) last accepted
        self.pending = {}          # robot -> {cam: TagPose}
        self.pending_t = {}
        self.sent_t = {}
        self.unconfirmed = {}      # robot -> TagPose (a jump waiting for a second read)
        self.lock = threading.Lock()
        self.counts = {n: dict(reads=0, fixes=0, refused_size=0, refused_jump=0, both=0, lock=0, motion=0,
                               **{c: 0 for c in cams})
                       for n in self.ids.values()}
        # (e09, Phase B1) the lock-on, seeded by each robot's tag read while it stands still: where the tags stop
        # reading (beyond about x 100 cm of the school table) the robot is still followed, by its own look
        import lock_bank as LB
        self.lockbank = {c: LB.LockBank(m, float(tags.get("height_cm", 6.0)) - 1.0, enabled=bool(lock))
                         for c, m in cams.items()}
        self.lock_sent = {}
        # (e10, C3) the motion detector at school too (e02_multitrack with the robots' own poses as its prediction:
        # cam_bridge's tracker), on the overhead camera: re-seated on its robot by every tag read; where neither the
        # tag nor the lock-on gives a fix, its sure sightings go out as src tag_bridge:motion. Off: --no-motion /
        # MIRLT_TAG_MOTION=0. Never stops the tags' fixes (errors are logged)
        self.motion = None
        self.motion_cam = None
        self.motion_sent = {}
        if motion and board is not None:
            try:
                import cam_bridge as CB
                cname = "overhead" if "overhead" in cams else next(iter(cams))
                T = CB.make_tracker_class()
                b = tuple(float(v) for v in board)
                self.motion = T(cams[cname], board_center=((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0), board_rect=b,
                                margin_cm=6.0, lights=False)
                self.motion_cam = cname
                self.motion_added = set()
            except Exception as e:      # noqa: BLE001
                self.motion = None
                self.event("motion_error", why="the motion detector could not start: %s" % e)
        self.frames = {c: 0 for c in cams}
        self.ms = {c: [] for c in cams}
        self.spread = []
        self.status_t = 0.0
        os.makedirs(out, exist_ok=True)
        self.track_f = open(os.path.join(out, "track.csv"), "w", newline="")
        self.track_w = csv.writer(self.track_f)
        self.track_w.writerow(["t", "robot", "x", "y", "status", "sent_th", "why", "src"])
        self.pose_f = open(os.path.join(out, "tag_poses.csv"), "w", newline="")
        self.pose_w = csv.writer(self.pose_f)
        self.pose_w.writerow(["t", "cam", "robot", "id", "x", "y", "heading", "sigma_cm", "sigma_deg", "size_ratio",
                              "px_per_cm", "u", "v", "used", "mid_x", "mid_y"])
        self.fix_f = open(os.path.join(out, "fixes.csv"), "w", newline="")
        self.fix_w = csv.writer(self.fix_f)
        self.fix_w.writerow(["t", "robot", "x", "y", "heading", "sigma_cm", "sigma_deg", "cams", "cams_apart_cm"])
        self.obs_f = open(os.path.join(out, "tag_obs.csv"), "w", newline="")
        self.obs_w = csv.writer(self.obs_f)
        self.obs_w.writerow(["t", "cam", "id", "u0", "v0", "u1", "v1", "u2", "v2", "u3", "v3"])
        self.ev_f = open(os.path.join(out, "events.jsonl"), "a")
        self.event("start", tags=tags, cams=list(cams), robots=self.ids, roofs=self.roofs)
        # the reference tags' watch (2026-11-03): a bumped camera
        self.refw = {}
        if ref_expected:
            import ref_tags as RT
            for c, exp in ref_expected.items():
                if c in cams and exp:
                    self.refw[c] = RefWatch(c, exp, RT.detector(ref_cfg))
            self.refw_f = open(os.path.join(out, "ref_watch.csv"), "w", newline="")
            self.refw_w = csv.writer(self.refw_f)
            self.refw_w.writerow(["t", "cam", "tags_found", "median_shift_px", "max_shift_px", "verdict"])
            self.event("ref_watch", cams={c: sorted(w.expected, key=int) for c, w in self.refw.items()})

    def event(self, kind, **kw):
        kw.update(kind=kind, t=round(time.time(), 3))
        self.ev_f.write(json.dumps(kw, default=str) + "\n")
        self.ev_f.flush()

    def say(self, s):
        self.say_(s)

    def on_pose(self, msg):
        n = msg.get("robot")
        if n in self.counts:
            try:
                new = (float(msg["x"]), float(msg["y"]), float(msg["th"]), float(msg["t"]))
            except (KeyError, TypeError, ValueError):
                return
            old = self.own.get(n)
            st = self.__dict__.setdefault("_own_still", {})
            if old is not None and math.hypot(new[0] - old[0], new[1] - old[1]) < 0.3 and abs(new[2] - old[2]) < 1.0:
                st[n] = st.get(n, 0.0) + max(0.0, new[3] - old[3])        # (e09) how long it has stood still
            else:
                st[n] = 0.0
            self.own[n] = new
            if self.motion is not None:
                try:
                    self.motion.own_pose(n, new[0], new[1], new[2], new[3])
                except Exception:   # noqa: BLE001
                    pass

    def motion_step(self, cam_name, grey, t, read, out):
        """(e10, C3) The motion detector's picture: robots read by their tag re-seat its track there (or start it);
        robots neither the tag nor the lock-on gave a fix just now get its sure sighting as a fix."""
        if self.motion is None or cam_name != self.motion_cam:
            return
        here = {n: (p.x, p.y, p.heading) for n, p, u in out if u == "yes"}
        for n, (x, y, h) in here.items():
            st = self.motion.state(n)
            if n not in self.motion_added or st is None or math.hypot(st["x"] - x, st["y"] - y) > 3.0:
                self.motion.add(n, x, y, heading=h, t=t)
                self.motion_added.add(n)
        self.motion.push(grey, t)
        if self.observe:
            return
        for n in self.motion_added:
            if n in read:
                continue
            st = self.motion.state(n)
            if st is None or st["status"] != "seen" or abs((st.get("t_seen") or 0.0) - t) > 0.05:
                continue
            if t - self.sent_t.get(n, 0.0) < 0.5 or t - self.lock_sent.get(n, 0.0) < 0.5 or \
                    t - self.motion_sent.get(n, 0.0) < 1.0 / FIX_HZ * 0.9:
                continue                 # a tag or a lock-on fix is fresher
            self.motion_sent[n] = t
            self.counts[n]["motion"] = self.counts[n].get("motion", 0) + 1
            self.send({"type": "fix", "robot": n, "t": round(t, 4), "x": round(st["x"], 2), "y": round(st["y"], 2),
                       "th": None, "sigma": 1.0, "sigma_th": 9.0, "src": "tag_bridge:motion"})
            self.track_w.writerow(["%.4f" % t, n, "%.2f" % st["x"], "%.2f" % st["y"], "motion", "", "", cam_name])

    def predict(self, cam_name, t):
        cam = self.cams[cam_name]
        H = float(self.tags["height_cm"])

        def f(tag_id):
            n = self.robot_of.get(tag_id)
            o = self.own.get(n)
            if o is None or abs(t - o[3]) > 2.0:
                return None
            u, v = cam.project((o[0], o[1], H))
            if not (np.isfinite(u) and np.isfinite(v)) or not cam.in_image(u, v):
                return None
            r = TG.tag_px_at(cam, o[0], o[1], H, float(self.tags["tag_cm"]))
            return (u, v, max(8.0, 2.0 * r))
        return f

    def ref_check(self, cam_name, grey, t):
        w = self.refw.get(cam_name)
        if w is None or not w.due(t):
            return None
        med, shifts, verdict = w.check(grey, t)
        self.refw_w.writerow(["%.3f" % t, cam_name, " ".join(sorted(shifts, key=int)),
                              "" if med is None else "%.2f" % med,
                              "" if not shifts else "%.2f" % max(shifts.values()), verdict or "unseen"])
        self.refw_f.flush()
        if verdict == "moved":
            from ref_tags import TAG_CM as RT_TAG_CM
            try:
                pxcm = float(np.median([np.mean([np.linalg.norm(e[k] - e[(k + 1) % 4]) for k in range(4)])
                                        for e in w.expected.values()])) / RT_TAG_CM
            except Exception:      # noqa: BLE001
                pxcm = 5.0
            cm = med / max(pxcm, 0.1)
            self.event("camera_moved", cam=cam_name, median_px=round(med, 2), about_cm=round(cm, 1),
                       shifts={k: round(v, 2) for k, v in shifts.items()})
            self.say("WARNING: camera %s has MOVED (bumped?): its reference tags are %.1f px (about %.1f cm on the "
                     "table) from where the setup saw them. Positions from this camera are off by about that much. "
                     "Stop the run and Measure again (Cameras & table): the tags make it quick." % (cam_name, med, cm))
        elif verdict == "back":
            self.event("camera_back", cam=cam_name, median_px=round(med, 2))
            self.say("camera %s: its reference tags are back where the setup saw them (%.1f px)" % (cam_name, med))
        return verdict

    LIGHT_EVERY_S = 2.0       # (11-05h, Jay-Sun 5 Nov: lighting identical across experiments; the room lights go off by
    LIGHT_WARN_FRAC = 0.15    # themselves) each camera's picture brightness logged this often (brightness.csv); a change
    LIGHT_WARN_N = 3          # of 15 % from the run's first minute, 3 samples in a row, is said and logged. Chosen

    def light_check(self, cam_name, grey, t):
        st = self.__dict__.setdefault("light", {}).setdefault(cam_name, {"t": -1e9, "base": [], "bad": 0,
                                                                         "said": False})
        if t - st["t"] < self.LIGHT_EVERY_S:
            return
        st["t"] = t
        b = float(grey[::8, ::8].mean())
        if getattr(self, "_light_f", None) is None:
            self._light_f = open(os.path.join(self.out, "brightness.csv"), "w")
            self._light_f.write("t,camera,mean_grey\n")
        self._light_f.write("%.3f,%s,%.2f\n" % (t, cam_name, b))
        self._light_f.flush()
        if len(st["base"]) < 30:
            st["base"].append(b)
            return
        base = sorted(st["base"])[len(st["base"]) // 2]
        if base > 0 and abs(b - base) / base > self.LIGHT_WARN_FRAC:
            st["bad"] += 1
            if st["bad"] >= self.LIGHT_WARN_N and not st["said"]:
                st["said"] = True
                self.event("lighting_changed", cam=cam_name, base=round(base, 1), now=round(b, 1))
                self.say("LIGHTING CHANGED: camera %s's picture is %.0f %% %s than at the start (mean grey %.0f, was %.0f):"
                         " the room lights? Experiments need the same light throughout" % (
                             cam_name, 100 * abs(b - base) / base, "brighter" if b > base else "darker", b, base))
        else:
            st["bad"] = 0
            if st["said"]:
                st["said"] = False
                self.event("lighting_back", cam=cam_name, base=round(base, 1), now=round(b, 1))

    def on_picture(self, cam_name, grey, t):
        """One camera's picture (t: when it was taken, Pi clock)."""
        self.frames[cam_name] += 1
        try:
            self.light_check(cam_name, grey, t)
        except Exception:  # noqa: BLE001  (the brightness log never stops the fixes)
            pass
        if self.refw:
            try:
                self.ref_check(cam_name, grey, t)
            except Exception as e:      # noqa: BLE001  (the watch never stops the fixes)
                self.event("ref_watch_error", cam=cam_name, why=str(e))
        t0 = time.perf_counter()
        found = self.trk[cam_name].step(grey, t, cam=cam_name, predict=self.predict(cam_name, t))
        self.ms[cam_name].append((time.perf_counter() - t0) * 1000.0)
        H, S = float(self.tags["height_cm"]), float(self.tags["tag_cm"])
        cam = self.cams[cam_name]
        out = []
        for tag_id, d in found.items():
            n = self.robot_of[tag_id]
            p = TG.pose_from_detection(cam, d, H, S, roof=self.roofs.get(int(tag_id)))
            yo = float((self.tags.get("yaw_off") or {}).get(str(tag_id), (self.tags.get("yaw_off") or {}).get(
                int(tag_id), 0.0)) or 0.0)
            if p is not None and yo:
                # (2026-11-05h, Observe then trust) the tag is turned on its robot by this much (the heading check's
                # nudge found it): every heading of this tag corrected to the robot's nose
                p.heading = ((p.heading + yo + 180.0) % 360.0) - 180.0
            c = d.corners.ravel()
            self.obs_w.writerow(["%.4f" % t, cam_name, tag_id] + ["%.2f" % v for v in c])
            if p is None:
                continue
            p.t = t
            used = self.accept(n, p)
            cu, cv = d.center
            self.pose_w.writerow(["%.4f" % t, cam_name, n, tag_id, "%.2f" % p.x, "%.2f" % p.y, "%.2f" % p.heading,
                                  "%.3f" % p.sigma_cm, "%.2f" % p.sigma_deg, "%.3f" % p.size_ratio,
                                  "%.2f" % p.px_per_cm, "%.1f" % cu, "%.1f" % cv, used,
                                  # (2026-11-03) the tag's own middle (x, y are the turning point when an offset is
                                  # taken off): the refit measures the offset from these
                                  "%.2f" % (p.mid or (p.x, p.y))[0], "%.2f" % (p.mid or (p.x, p.y))[1]])
            out.append((n, p, used))
        with self.lock:
            self.flush(t)
        try:
            self.lock_step(cam_name, grey, t, {n for n, _, u in out if u == "yes"}, out)
        except Exception as e:      # noqa: BLE001  (the lock-on never stops the tags' fixes)
            self.event("lock_error", cam=cam_name, why=str(e))
        try:
            self.motion_step(cam_name, grey, t, {n for n, _, u in out if u == "yes"}, out)
        except Exception as e:      # noqa: BLE001  (the motion detector never stops the tags' fixes)
            self.event("motion_error", cam=cam_name, why=str(e))
        return out

    LOCK_STILL_S = 0.4      # CHOSEN: its own pose has not moved this long: standing still, its look can be taken

    def lock_step(self, cam_name, grey, t, read, out):
        """(e09) Robots read by their tag in this picture and standing still: their lock is (re)seeded there. Robots NOT
        read: their lock looks for them (near their own pose) and its place goes out as a fix, src tag_bridge:lock."""
        lb = self.lockbank.get(cam_name)
        if lb is None or not lb.enabled:
            return
        here = {n: (p.x, p.y) for n, p, u in out if u == "yes"}
        for n in self.counts:
            o = self.own.get(n)
            if n in read:
                still = o is not None and getattr(self, "_own_still", {}).get(n, 0.0) >= self.LOCK_STILL_S
                if still:
                    others = [v for m, v in here.items() if m != n] + [(q[0], q[1]) for m, q in self.last.items()
                                                                        if m != n]
                    lb.seed(n, grey, t, here[n][0], here[n][1], others)
                continue
            pred = (o[0], o[1]) if (o is not None and abs(t - o[3]) < 2.0) else None
            f = lb.step(n, grey, t, predict=pred, still=getattr(self, "_own_still", {}).get(n, 0.0) >= self.LOCK_STILL_S)
            if f is None or self.observe:
                continue
            if t - self.lock_sent.get(n, 0.0) < 1.0 / FIX_HZ * 0.9 or t - self.sent_t.get(n, 0.0) < 0.5:
                continue            # a tag fix is fresher: the lock is for when the tags do not read
            self.lock_sent[n] = t
            self.counts[n]["lock"] += 1
            self.send({"type": "fix", "robot": n, "t": round(t, 4), "x": round(f[0], 2), "y": round(f[1], 2),
                       "th": None, "sigma": 1.2, "sigma_th": 9.0, "src": "tag_bridge:lock", "score": round(f[2], 2)})
            self.track_w.writerow(["%.4f" % t, n, "%.2f" % f[0], "%.2f" % f[1], "lock", "", "", cam_name])

    def accept(self, n, p):
        c = self.counts[n]
        c["reads"] += 1
        c[p.cam] = c.get(p.cam, 0) + 1
        if abs(p.size_ratio - 1.0) > SIZE_REFUSE:
            c["refused_size"] += 1
            return "refused: size %.2f" % p.size_ratio
        last = self.last.get(n)
        if last is not None and abs(p.t - last[3]) < JUMP_S and math.hypot(p.x - last[0], p.y - last[1]) > JUMP_CM:
            u = self.unconfirmed.get(n)
            if u is None or math.hypot(p.x - u.x, p.y - u.y) > 5.0:
                self.unconfirmed[n] = p
                c["refused_jump"] += 1
                return "held: jump %.0f cm" % math.hypot(p.x - last[0], p.y - last[1])
        self.unconfirmed.pop(n, None)
        with self.lock:
            t0 = self.pending_t.get(n)
            if t0 is not None and p.t - t0 > PAIR_S:
                # a newer moment: the one waiting for the other camera goes out as it is (one camera's read)
                self.flush(p.t, only=n, force=True)
            self.pending.setdefault(n, {})[p.cam] = p
            self.pending_t.setdefault(n, p.t)
        return "yes"

    def flush(self, t_now, only=None, force=False):
        """Send each robot's fused fix once every camera that sees it has read this moment, or when a newer moment
        starts (force), and at most FIX_HZ a second."""
        for n in list(self.pending):
            if only is not None and n != only:
                continue
            got = self.pending[n]
            t_first = self.pending_t.get(n, t_now)
            seeing = [c for c in self.cams if self.trk[c].last.get(self.tags["ids"][n]) is not None]
            ready = force or len(got) >= max(1, min(len(self.cams), len(seeing))) or (t_now - t_first) > 3 * PAIR_S
            if not ready:
                continue
            if t_now - self.sent_t.get(n, 0.0) < 1.0 / FIX_HZ * 0.9:
                # over FIX_HZ: this moment is dropped whole (kept, its first camera's read would later go out alone,
                # out of step with the other camera's: the dry runs fused 2 of 478 fixes)
                del self.pending[n]
                self.pending_t.pop(n, None)
                continue
            ps = [p for p in got.values() if abs(p.t - max(q.t for q in got.values())) <= PAIR_S]
            f, spread = TG.fuse(ps)
            del self.pending[n]
            self.pending_t.pop(n, None)
            if f is None:
                continue
            if len(ps) > 1:
                self.counts[n]["both"] += 1
                self.spread.append(spread)
            self.last[n] = (f.x, f.y, f.heading, f.t)
            self.sent_t[n] = t_now
            sig = max(SIGMA_FLOOR_CM, f.sigma_cm)
            sth = max(SIGMA_DEG_FLOOR, f.sigma_deg)
            msg = {"type": "fix", "robot": n, "t": round(f.t, 4), "x": round(f.x, 2), "y": round(f.y, 2),
                   "th": round(f.heading, 2), "sigma": round(sig, 2), "sigma_th": round(sth, 2),
                   "src": "tag_bridge:%s" % f.cam,
                   # (2026-11-03, confidence at every point) the two cameras' positions apart, and px per cm
                   "apart": round(float(spread or 0.0), 2), "ppc": round(float(f.px_per_cm or 0.0), 2)}
            if not self.observe:
                self.send(msg)
                self.counts[n]["fixes"] += 1
            self.track_w.writerow(["%.4f" % f.t, n, "%.2f" % f.x, "%.2f" % f.y, "seen", "%.2f" % f.heading,
                                   "", f.cam])
            self.fix_w.writerow(["%.4f" % f.t, n, "%.2f" % f.x, "%.2f" % f.y, "%.2f" % f.heading, "%.3f" % sig,
                                 "%.2f" % sth, f.cam, "%.2f" % spread])

    def status(self, t):
        if time.time() - self.status_t < STATUS_EVERY_S:
            return
        self.status_t = time.time()
        parts = []
        for n, c in self.counts.items():
            la = self.last.get(n)
            parts.append("%s %s, %d fixes" % (n, "at (%.0f, %.0f) facing %.0f" % la[:3] if la else "not seen yet",
                                               c["fixes"]))
        ms = "; ".join("%s %.0f ms" % (c, np.median(v[-30:])) for c, v in self.ms.items() if v)
        self.say("%s; %s" % ("; ".join(parts), ms))

    def summary(self):
        s = {"frames": self.frames, "robots": self.counts,
             "ms_per_picture": {c: dict(median=round(float(np.median(v)), 2), p95=round(float(np.percentile(v, 95)), 2))
                                for c, v in self.ms.items() if v},
             "cams_apart_cm": (dict(median=round(float(np.median(self.spread)), 2),
                                    p95=round(float(np.percentile(self.spread, 95)), 2), n=len(self.spread))
                               if self.spread else None),
             "windows": {c: self.trk[c].windows for c in self.cams}, "full_searches": {c: self.trk[c].fulls
                                                                                       for c in self.cams},
             "tags": {k: v for k, v in self.tags.items() if k != "ids"},
             "roofs": {self.robot_of.get(i, str(i)): v for i, v in self.roofs.items()},
             "ref_watch": {c: {"moved": w.moved, "last_median_px": None if not w.last or w.last[0] is None
                               else round(w.last[0], 2), "tags": sorted(w.expected, key=int)}
                           for c, w in self.refw.items()},
             "lock": {c: lb.summary() for c, lb in self.lockbank.items()}}
        with open(os.path.join(self.out, "bridge_summary.json"), "w") as f:
            json.dump(s, f, indent=1, default=str)
        return s

    def close(self):
        for fh in (self.track_f, self.pose_f, self.obs_f, self.fix_f, self.ev_f, getattr(self, "refw_f", None)):
            if fh is None:
                continue
            try:
                fh.close()
            except OSError:
                pass


def load_cams(specs, wait_s=0.0):
    """--cam NAME=STREAM,CALIB -> ({name: CameraModel}, {name: stream})."""
    import e02_camera as CAM
    models, streams = {}, {}
    for s in specs:
        name, _, rest = s.partition("=")
        stream, _, cal = rest.partition(",")
        t0 = time.time()
        while not os.path.exists(cal) and time.time() - t0 < wait_s:
            time.sleep(0.2)
        if not os.path.exists(cal):
            print("%s camera: no model at %s: left out" % (name, cal), flush=True)
            continue
        models[name] = CAM.load_camera(cal)
        streams[name] = stream
    return models, streams


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--site", help="the site profile (default: the one in use)")
    ap.add_argument("--tags", help="a tags JSON instead of the site's ({family, tag_cm, roof_cm, height_cm, ids})")
    ap.add_argument("--cam", action="append", default=[], metavar="NAME=STREAM,CALIB",
                    help="a camera: its role, its /live stream and its model (repeatable)")
    ap.add_argument("--robots", default="", help="only these robots")
    ap.add_argument("--to", default="127.0.0.1:5010", help="run_plan.py's fix port")
    ap.add_argument("--pose-port", type=int, default=POSE_PORT)
    ap.add_argument("--observe", action="store_true", help="send nothing (print what it reads)")
    ap.add_argument("--out", help="log folder (default logs/<date-time>-tagbridge)")
    ap.add_argument("--max-s", type=float, default=0)
    ap.add_argument("--wait-calib-s", type=float, default=20.0)
    ap.add_argument("--no-ref-watch", action="store_true", help="do not watch the reference tags (a bumped camera)")
    ap.add_argument("--no-lock", action="store_true", help="(e09) no lock-on: tags only, as before e09")
    ap.add_argument("--no-motion", action="store_true", help="(e10) no motion detector (tags and lock-on only, as e09)")
    ap.add_argument("--board", default="", help="(e10) x0,y0,x1,y1 cm: the table (the motion detector's area; without "
                    "it the motion detector stays off)")
    ap.add_argument("--ref-fit", default="", help="the setup's site_fit.json (default: beside the first camera model)")
    a = ap.parse_args(argv)
    tags = load_tags(a.tags, a.site)
    if tags.get("height_note"):
        print("NOTE: " + tags["height_note"], flush=True)
    models, streams = load_cams(a.cam, a.wait_calib_s)
    if not models:
        raise SystemExit("No camera model: nothing to read tags with (give --cam NAME=STREAM,CALIB)")
    out = a.out or os.path.join(REPO, "logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") + "-tagbridge")
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    h, _, p = a.to.rpartition(":")
    dest = (h or "127.0.0.1", int(p))

    def send(msg):
        try:
            tx.sendto(json.dumps(msg).encode(), dest)
        except OSError:
            pass
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for k in range(15):
        try:
            rx.bind(("127.0.0.1", a.pose_port))
            break
        except OSError as e:
            if k == 14:
                raise SystemExit("Port %d is in use (another bridge still running?): %s" % (a.pose_port, e))
            time.sleep(1.0)
    rx.setblocking(False)
    robots = [r.strip() for r in a.robots.split(",") if r.strip()] or None
    rexp = {}
    if not a.no_ref_watch:
        fitp = a.ref_fit or os.path.join(os.path.dirname(a.cam[0].partition("=")[2].partition(",")[2]), "site_fit.json")
        for _ in range(50):              # the setup writes the camera models first, then site_fit.json
            if os.path.exists(fitp):
                break
            time.sleep(0.1)
        rexp = ref_expected(models, fitp)
        if rexp:
            print("Reference tags watched (a bumped camera): %s" % "; ".join(
                "%s %s" % (c, ", ".join(sorted(v, key=int))) for c, v in rexp.items()), flush=True)
    import roof_cal as RC
    cal_dir = os.path.dirname(a.cam[0].partition("=")[2].partition(",")[2]) if a.cam else ""
    roofs, rwords = RC.roofs_for_bridge(cal_dir, tags)
    for w in rwords:
        print("Roof: " + w, flush=True)
    br = Bridge(tags, models, send, out, observe=a.observe, robots=robots, ref_expected=rexp, roofs=roofs,
                lock=not a.no_lock and os.environ.get("MIRLT_TAG_LOCK", "1") != "0",   # (e10) Mission Control preferences
                motion=not a.no_motion and os.environ.get("MIRLT_TAG_MOTION", "1") != "0",
                board=[float(v) for v in a.board.split(",")] if a.board else None)
    print("Tag bridge: %s tags (%s cm, %s cm up) on %s; cameras %s; fixes to %s%s. Logs: %s" % (
        tags.get("family"), tags.get("tag_cm"), tags.get("height_cm"),
        ", ".join("%s=%d" % kv for kv in tags["ids"].items()), ", ".join(models), a.to,
        " (OBSERVE ONLY)" if a.observe else "", out), flush=True)
    import signal
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    import cv2
    feeds = {c: CamFeed(c, streams[c]) for c in models}
    fwatch = None
    import frame_watch as FW
    if FW.enabled():      # (e12) no pictures from a camera: said loudly, an event, a message to the run
        fwatch = FW.FrameWatch(list(models), send=send, event=br.event, say=lambda s_: print(s_, flush=True))
        for f in feeds.values():
            f.on_frame = fwatch.saw
    for f in feeds.values():
        f.start()
    stop = threading.Event()

    def worker(c):
        while not stop.is_set():
            item = feeds[c].take(0.5)
            if item is None:
                continue
            pf, t_pi, body, arrived = item
            g = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_GRAYSCALE)
            if g is None:
                continue
            try:
                br.on_picture(c, g, t_pi - CAPTURE_S)
            except Exception as e:      # noqa: BLE001  (one bad picture never stops the fixes)
                br.event("picture_error", cam=c, why=str(e))
    ws = [threading.Thread(target=worker, args=(c,), daemon=True) for c in models]
    for w in ws:
        w.start()
    t0 = time.time()
    try:
        while True:
            try:
                data, _ = rx.recvfrom(65536)
                msg = json.loads(data.decode("utf-8", "replace"))
                if isinstance(msg, dict) and msg.get("type") == "pose":
                    br.on_pose(msg)
            except (BlockingIOError, OSError):
                time.sleep(0.01)
            except ValueError:
                pass
            br.status(time.time())
            if a.max_s and time.time() - t0 > a.max_s:
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if fwatch is not None:
            fwatch.close()
        for f in feeds.values():
            f.close()
        for w in ws:
            w.join(timeout=8)         # (e12) a worker finishes its picture (detection) before the summary
        for f in feeds.values():
            f.join(timeout=2)
        s = br.summary()
        br.close()
        print("Tag bridge stopped: %s" % json.dumps(s["robots"]), flush=True)
    return 0


if __name__ == "__main__":
    _code = main()
    import orderly_exit as OX      # (e12) no thread torn down inside OpenCV / apriltag at exit
    OX.finish(_code)
