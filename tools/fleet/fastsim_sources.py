#!/usr/bin/env python3
"""The fast simulator's camera as SIGHTINGS (e09, Phase B): what each camera source would report each picture, with no
names except the roof tags' ids and the lock-ons' own robot, sent through the ONE tracking layer (tracking_layer.py),
so names CAN swap here (the plain fast simulator names every fix and never does: e04 sweep, 7 Nov audit).

Each source's model (per robot, per picture, 12 pictures a second; the measured ones say where from):
  motion   a robot that moves (drives, or turns on its spot) is found in 93 % of pictures, one standing still in 19 %
           (MEASURED: real footage, 21 Oct roam1, docs/session-notes/2026-10-21-real-footage.md); 0.5 s after it stops
           it still shows (the floor memory's lag, CHOSEN). On a dark patch or against a busy background (scene
           "dark" / "busy") 30 % (CHOSEN, to give the 22-23 Oct evening losses' 41 / 40 / 72 %). Two bodies under
           MERGE_GAP_CM apart: ONE outline at their middle, twice the area (e02_multitrack's merged case)
  lock     once seeded on a body (the nearest true body to the seed spot, within 6 cm: a seed on the wrong spot locks
           on the wrong robot), it reports THAT body under the name it was seeded for: 96 % of pictures, 85 % on a dark
           patch (CHOSEN, to give 22-23 Oct's 94 / 93 / 96 %); it lets go after LOCK_LOST_S of misses; two bodies under
           LOCK_JUMP_GAP_CM apart: it may jump to the look-alike (LOCK_JUMP_P a picture, CHOSEN)
  lights   on a body 1 cm ahead of its middle: 85 % normal, 20 % in glare ("glare" patches), 90 % in the dark (CHOSEN)
  tags     named, 90 % where the picture is good enough (x under scene "tag_x_max", the school's ~100 cm; not on dark
           patches), 0.4 cm noise (CHOSEN; e07's site fit is 0.37-0.59 cm worst)
  ml       the ML detector's slot: off (the layer's switch); when on here, 90 % moving or still, no names (CHOSEN)
Faults (rehearsal g): "mirror" (a wrong camera name: the other camera's calibration, sightings mirrored left to right),
"none" (no camera at all after t0), plus the controller's own clock skew (--cam-clock).
"""
import math
import random

import tracking_layer as TL

FPS = 12.0
MERGE_GAP_CM = 3.0          # CHOSEN: bodies this close merge into one outline (shadows between them)
LOCK_JUMP_GAP_CM = 2.0      # CHOSEN
LOCK_JUMP_P = 0.05          # CHOSEN: a picture's chance that a lock on one of two touching robots takes the other
LOCK_LOST_S = 1.0           # e02_lock.LOST_S
LINGER_S = 0.5              # CHOSEN: the floor memory still shows a robot this long after it stops
P = dict(motion_moving=0.93, motion_still=0.19, motion_dark=0.30, lock=0.96, lock_dark=0.85, lights=0.85,
         lights_glare=0.20, lights_dark=0.90, tags=0.90, ml=0.90)


def _in(rects, x, y):
    return any(r[0] <= x <= r[2] and r[1] <= y <= r[3] for r in rects or ())


class CameraSources:
    def __init__(self, sim, scene, layer_kw):
        self.sim = sim
        self.scene = dict(scene)
        self.rng = random.Random(sim.seed * 977 + 5)
        trk = self.scene.get("tracking", "lights")
        old = bool(layer_kw.pop("old", False))
        srcs = layer_kw.pop("sources", None) or TL.sources_for(trk, old=old)
        self.sources = tuple(srcs)
        if old:
            layer_kw.setdefault("identity", "predict")
            layer_kw.setdefault("cluster", False)
            layer_kw.setdefault("camera_watch", False)
            layer_kw.setdefault("parked_watch", False)
            layer_kw.setdefault("brick_watch", False)
        layer_kw.setdefault("board", tuple(sim.board))
        self.layer = TL.TrackingLayer(sim.names, srcs, log=self.log, lock_seed=self.lock_seed,
                                      wiggle=self.wiggle if not old else None, **layer_kw)
        if hasattr(sim.ctrl, "record_fn"):        # (e17) the controller's corrections widen that robot's gate
            sim.ctrl.record_fn = lambda n, t, d, trav: self.layer.record(n, t, d, trav)
        self.locks = {}             # name -> dict(body, last_ok)
        self.next_t = 0.0
        self.q = []                 # (due, tf, fixes)
        self.last = {n: None for n in sim.names}      # (t, x, y, th) last picture's truth
        self.stopped_t = {n: -1e9 for n in sim.names}
        self.stats = dict(frames=0, fixes={n: 0 for n in sim.names}, wrong={n: 0 for n in sim.names},
                          tracked_frames={n: 0 for n in sim.names}, alive_frames={n: 0 for n in sim.names},
                          far_frames={n: 0 for n in sim.names}, far_tracked={n: 0 for n in sim.names},
                          swaps=0, swap_s=0.0, wrong_run={n: 0 for n in sim.names}, by_src={})
        self.in_swap = {n: False for n in sim.names}

    def log(self, kind, row):
        self.sim.keep(kind, row)
        if row.get("what") == "wiggle asked":
            self.sim.__dict__.setdefault("wiggle_rows", []).append(row)

    # -- the lock-on and the wiggle (asked by the layer) -------------------------------------------------------------
    def lock_seed(self, name, t, x, y):
        best = min(self.sim.bots.values(), key=lambda b: math.hypot(b.x - x, b.y - y))
        if math.hypot(best.x - x, best.y - y) <= 6.0:
            self.locks[name] = dict(body=best.name, last_ok=t)
        else:
            self.locks.pop(name, None)

    def wiggle(self, name, why):
        c = self.sim.ctrl
        r = c.R.get(name)
        if r is not None and hasattr(c, "identity_wiggle"):
            spots = getattr(self.layer, "wiggle_spots", None)
            req = getattr(self.layer, "wiggle_req", None)
            if spots:
                # (e17) an elimination wiggle, safe at every candidate; (e18) or the out-and-back the layer asks
                return c.identity_wiggle(r, why, spots=spots, req=req)
            return c.identity_wiggle(r, why)

    # -- each picture -----------------------------------------------------------------------------------------------
    def frame(self):
        s = self.sim
        t = s.t
        if t < self.next_t:
            self._deliver()
            return
        self.next_t = t + 1.0 / FPS
        sc = self.scene
        fault = sc.get("fault")
        dets, truth = [], {}
        bots = [b for b in s.bots.values()]
        alive = [b for b in bots if not b.dead]
        moving = {}
        for b in bots:
            prev = self.last[b.name]
            mv = turn = False
            if prev is not None:
                dt = max(1e-6, t - prev[0])
                mv = math.hypot(b.x - prev[1], b.y - prev[2]) / dt > 0.5
                turn = abs(((b.th - prev[3] + 180) % 360) - 180) / dt > 8.0
            if mv or turn:
                self.stopped_t[b.name] = t
            moving[b.name] = (mv, turn, t - self.stopped_t[b.name] <= LINGER_S)
            self.last[b.name] = (t, b.x, b.y, b.th)
        if fault == "none" and t >= sc.get("fault_t", 0.0):
            self._deliver()
            self._count(t, [], {})
            return
        # motion: merged pairs first
        merged = set()
        if "motion" in self.layer.sources:
            for i in range(len(alive)):
                for j in range(i + 1, len(alive)):
                    a, b = alive[i], alive[j]
                    if math.hypot(a.x - b.x, a.y - b.y) - 9.5 < MERGE_GAP_CM and any(moving[a.name]) or \
                            math.hypot(a.x - b.x, a.y - b.y) - 9.5 < MERGE_GAP_CM and any(moving[b.name]):
                        if self.rng.random() < P["motion_moving"]:
                            d = TL.Det("motion", t, (a.x + b.x) / 2 + self.rng.gauss(0, 0.7),
                                       (a.y + b.y) / 2 + self.rng.gauss(0, 0.7), area=2.0)
                            dets.append(d)
                            truth[id(d)] = None
                        merged.update((a.name, b.name))
        for b in alive:
            if s.cam_gap(b.name, t) or not s.in_cover(b.x, b.y):
                continue                     # (rehearsal c) the camera does not see this robot now; (e10) out of view
            mv, turn, linger = moving[b.name]
            dark = _in(sc.get("dark"), b.x, b.y)
            busy = _in(sc.get("busy"), b.x, b.y)
            glare = _in(sc.get("glare"), b.x, b.y)
            if b.name in sc.get("hide_still", ()) and not (mv or turn or linger):
                continue                     # (rehearsal d) parked against the background: the outline and the lights
                                             # never show it (27 Oct, Right); only a lock-on seeded earlier follows it
            if "motion" in self.layer.sources and b.name not in merged:
                p = P["motion_moving"] if (mv or turn or linger) else P["motion_still"]
                if dark or busy:
                    p = min(p, P["motion_dark"])
                if self.rng.random() < p:
                    d = TL.Det("motion", t, b.x + self.rng.gauss(0, 0.7), b.y + self.rng.gauss(0, 0.7),
                               spin=bool(turn and not mv))
                    dets.append(d)
                    truth[id(d)] = b.name
            if "lights" in self.layer.sources:
                p = P["lights_glare"] if glare else (P["lights_dark"] if dark else P["lights"])
                if self.rng.random() < p:
                    c_, s_ = math.cos(math.radians(b.th)), math.sin(math.radians(b.th))
                    d = TL.Det("lights", t, b.x + c_ + self.rng.gauss(0, 1.0), b.y + s_ + self.rng.gauss(0, 1.0))
                    dets.append(d)
                    truth[id(d)] = b.name
            if "tags" in self.layer.sources and b.x <= sc.get("tag_x_max", 1e9) and not dark and \
                    self.rng.random() < P["tags"]:
                d = TL.Det("tags", t, b.x + self.rng.gauss(0, 0.4), b.y + self.rng.gauss(0, 0.4),
                           th=b.th + self.rng.gauss(0, 1.0), name=b.name)
                dets.append(d)
                truth[id(d)] = b.name
            if "ml" in self.layer.sources and self.rng.random() < P["ml"]:
                d = TL.Det("ml", t, b.x + self.rng.gauss(0, 1.0), b.y + self.rng.gauss(0, 1.0))
                dets.append(d)
                truth[id(d)] = b.name
        # the locks
        if "lock" in self.layer.sources:
            for name, lk in list(self.locks.items()):
                body = s.bots.get(lk["body"])
                if body is not None and (s.cam_gap(body.name, t) or not s.in_cover(body.x, body.y)):
                    continue
                if body is None or body.dead:
                    self.locks.pop(name)
                    continue
                for o in alive:          # a look-alike right beside it: the lock may jump
                    if o is not body and math.hypot(o.x - body.x, o.y - body.y) - 9.5 < LOCK_JUMP_GAP_CM and \
                            self.rng.random() < LOCK_JUMP_P:
                        lk["body"], body = o.name, o
                        s.__dict__.setdefault("lock_jumps", 0)
                        s.lock_jumps += 1
                        break
                dark = _in(sc.get("dark"), body.x, body.y)
                if self.rng.random() < (P["lock_dark"] if dark else P["lock"]):
                    lk["last_ok"] = t
                    mv, turn, _ = moving[body.name]
                    d = TL.Det("lock", t, body.x + self.rng.gauss(0, 0.8), body.y + self.rng.gauss(0, 0.8), name=name,
                               spin=bool(turn and not mv))
                    dets.append(d)
                    truth[id(d)] = body.name
                elif t - lk["last_ok"] > LOCK_LOST_S:
                    self.locks.pop(name)
        # objects put on the table (the brick watch): the motion detector sees what changed against its floor memory
        for (tb, bx, by) in s.bricks:
            if t >= tb and "motion" in self.layer.sources and self.rng.random() < 0.9:
                d = TL.Det("motion", t, bx + self.rng.gauss(0, 0.5), by + self.rng.gauss(0, 0.5))
                dets.append(d)
                truth[id(d)] = "brick"
        if s.cam_fault or s.cam_correct is not None or s.tag_yaw:
            # (e10) every sighting through the camera's calibration (its fault, the discovery's correction); a roof
            # tag's heading through its turn on the robot and the corrections
            for d in dets:
                d.x, d.y = s.cam_map(d.x, d.y)
                if d.src == "tags":
                    d.th = s.cam_heading(d.th, d.name)
        if fault == "mirror" and t >= sc.get("fault_t", 0.0):
            W = s.board[0] + s.board[2]
            for d in dets:
                d.x = W - d.x
        # the layer: own poses (what the robots say: wheels + gyro, no camera since the close pass)
        # The own poses reach the layer late (scene "own_lag_s": the robots' records over the WiFi and the camera's own
        # clock: the camera's delay 0.44 s, MEASURED 5 / 6 Nov; the lag left after alignment CHOSEN 0.3 s), so the
        # predictions are as wrong as on the Pi
        c = s.ctrl
        if hasattr(c, "unaccounted_why"):
            for n, r in c.R.items():                # (e17) who the run cannot account for
                self.layer.set_unaccounted(n, c.unaccounted_why(r))
        lag = float(sc.get("own_lag_s", 0.3))
        for n, r in c.R.items():
            if r.own is None:
                continue
            o = c.own_at(r, t - lag) if lag > 0 else r.own[1:4]
            if o is None:
                continue
            still = not moving.get(n, (0, 0, 0))[0] and not moving.get(n, (0, 0, 0))[1]
            self.layer.own(n, t, o[0], o[1], o[2], still)
        fixes = self.layer.feed(t, dets)
        if hasattr(c, "camera_proven"):
            c.camera_proven = self.layer.proven
        tb = {id(d): truth.get(id(d)) for d in dets}
        self._count(t, fixes, {f["robot"]: tb.get(f.get("_id")) for f in fixes}, dets, truth)
        lat = self.rng.uniform(0.12, 0.2) if s.cam_delay is None else max(0.05, s.cam_delay + self.rng.uniform(-0.1,
                                                                                                                0.1))
        self.q.append((t + lat, t, fixes))
        for ob in self.layer.take_obstacles():
            if hasattr(c, "on_obstacle"):
                self.q.append((t + lat, t, [dict(obstacle=ob)]))
        for pe in self.layer.take_parked():
            if hasattr(c, "on_parked_moved"):
                self.q.append((t + lat, t, [dict(parked=pe)]))
        for rv in self.layer.take_revoked():         # (e17)
            if hasattr(c, "on_elim_revoked"):
                self.q.append((t + lat, t, [dict(revoked=rv)]))
        self._deliver()

    def _count(self, t, fixes, _unused, dets=(), truth=None):
        st = self.stats
        st["frames"] += 1
        got = {}
        for f in fixes:
            got[f["robot"]] = f
            st["by_src"][f["src"]] = st["by_src"].get(f["src"], 0) + 1
        for n, b in self.sim.bots.items():
            if b.dead or self.sim.ctrl.R[n].mode in ("out", "gone"):
                continue
            st["alive_frames"][n] += 1
            far = b.x > self.scene.get("tag_x_max", 1e9)
            if far:
                st["far_frames"][n] += 1
            f = got.get(n)
            if f is None:
                continue
            st["tracked_frames"][n] += 1
            if far:
                st["far_tracked"][n] += 1
            st["fixes"][n] += 1
            # whose body was it?  the nearest body to the fix (sightings carry no truth through the layer)
            nb = min(self.sim.bots.values(), key=lambda o: math.hypot(o.x - f["x"], o.y - f["y"]))
            wrong = nb.name != n and math.hypot(nb.x - f["x"], nb.y - f["y"]) < 4.0 and \
                math.hypot(b.x - f["x"], b.y - f["y"]) > 6.0
            if wrong:
                st["wrong"][n] += 1
                st["wrong_run"][n] += 1
                if st["wrong_run"][n] == 3 and not self.in_swap[n]:
                    self.in_swap[n] = True
                    st["swaps"] += 1
                    self.sim.keep("swaps", dict(t=round(t, 2), robot=n, took=nb.name))
            else:
                st["wrong_run"][n] = 0
                self.in_swap[n] = False

    def _deliver(self):
        s = self.sim
        c = s.ctrl
        due = [e for e in self.q if e[0] <= s.t]
        if not due:
            return
        self.q = [e for e in self.q if e[0] > s.t]
        for _, tf, fixes in sorted(due, key=lambda e: e[0]):
            for f in fixes:
                if "obstacle" in f:
                    c.on_obstacle(f["obstacle"])
                    continue
                if "parked" in f:
                    c.on_parked_moved(f["parked"])
                    continue
                if "revoked" in f:
                    c.on_elim_revoked(f["revoked"])
                    continue
                n = f["robot"]
                if f.get("how"):                     # (e17) a correction: the controller sends the robot an fpose
                    c.on_fix(n, tf + s.cam_clock, f["x"], f["y"], f["th"], arrival=s.t, sigma=f["sigma"],
                             src=f["src"], how=f["how"])
                    continue
                c.on_fix(n, tf + s.cam_clock, f["x"], f["y"], f["th"], arrival=s.t, sigma=f["sigma"], src=f["src"])
                if s.t >= s.frozen_until and c.R[n].lost is None and getattr(c.R[n], "fix_used", True):
                    c.sent(n)
                    fx, fy = c.fix_out(n, tf + s.cam_clock, f["x"], f["y"]) if hasattr(c, "fix_out") else (f["x"],
                                                                                                           f["y"])
                    s.down.send(s.t, s.fuse, n, fx, fy, f["th"], tf)

    def summary(self):
        st = self.stats
        out = dict(sources=list(self.layer.sources), identity=self.layer.identity,
                   tracked_pct={n: round(100.0 * st["tracked_frames"][n] / max(1, st["alive_frames"][n]), 1)
                                for n in self.sim.names},
                   far_tracked_pct={n: (round(100.0 * st["far_tracked"][n] / st["far_frames"][n], 1)
                                        if st["far_frames"][n] else None) for n in self.sim.names},
                   far_frames={n: st["far_frames"][n] for n in self.sim.names},
                   wrong_fixes=dict(st["wrong"]), swaps=st["swaps"], by_src=dict(st["by_src"]),
                   lock_jumps=getattr(self.sim, "lock_jumps", 0), layer=self.layer.summary())
        return out
