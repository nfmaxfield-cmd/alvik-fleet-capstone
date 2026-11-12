"""Build e17: a small deterministic rig for the tracking layer + the free-roam controller (no robots, no Pi).

Robots stand where they are put (truth); each reports its OWN pose (truth + an offset that only an fpose removes, as
the firmware's pose.set does); the camera sees every robot's lights (no names) with noise, its fixes delayed; the
layer names them and the controller gets them as the Pi's runner gives them (freeroam.Runner.fixes): corrections
(how) as on_fix(..., how), the rest as fixes the robot's own filter would weigh, gated as firmware pose.h does
(fix_gate 4 sigmas and 3 cm: a far fix is refused). Turn steps (a wiggle) are carried out (the heading changes, the
lights spot swings round its middle, 1 cm ahead of it); drive steps are carried out straight at their speed.

(e18) Optional, all off by default (the e17 tests run as before): odo {robot: (wheel_err, gyro_err, noise)}: its own pose
drives on its wheels and gyro with these errors (a 3 % wheel error drives its own pose 3 % further than the truth; noise:
the wheel step's and the turn step's relative random error), and raw=True feeds its wheels and yaw rate as RAW rows
(on_raw_full); dark [rects]: the camera sees nothing there; workspace: the controller's proven workspace.
"""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
if FLEET not in sys.path:
    sys.path.insert(0, FLEET)

import freeroam_core as FC     # noqa: E402
import tracking_layer as TL    # noqa: E402

BOARD = (0.0, 0.0, 122.24, 121.92)


class Rig:
    def __init__(self, truth, own_off=None, eliminate=True, gate_record=True, delay=0.45, seed=1, noise=0.3,
                 lights_off_cm=1.0, wiggle=True, fps=5.0, never_still=False, odo=None, dark=(), workspace=None,
                 raw=False, layer_kw=None, spin_flag=True, fuse_then=False):
        self.rng = random.Random(seed)
        self.names = list(truth)
        self.truth = {n: list(v) for n, v in truth.items()}          # x, y, heading
        self.off = {n: list((own_off or {}).get(n, (0.0, 0.0, 0.0))) for n in self.names}
        self.events, self.logs, self.sent = [], [], []
        self.odo = {n: tuple((odo or {}).get(n, (0.0, 0.0, 0.0))) for n in self.names}
        self.dark = [tuple(d) for d in dark]
        self.raw = bool(raw)
        self.spin_flag = bool(spin_flag)     # (e18) False: the home lights bridge (no spin flag on its sightings)
        self.fuse_then = bool(fuse_then)     # (e18) a fix fused against its pose at the picture's time (as pose.h)
        self.own_hist = {}
        self.c = FC.Controller(self.names, BOARD, self.out, self.log, pose_source="blend", workspace=workspace)
        self.layer = TL.TrackingLayer(self.names, ("motion", "lock", "lights"), log=lambda k, r: self.logs.append(r),
                                      wiggle=(self.wiggle if wiggle else None), board=BOARD, eliminate=eliminate,
                                      gate_record=gate_record, camera_watch=False, **(layer_kw or {}))
        self.c.record_fn = lambda n, t, d, trav: self.layer.record(n, t, d, trav)
        self.c.never_still = never_still
        self.delay, self.noise, self.lights_off, self.fps = delay, noise, lights_off_cm, fps
        self.q = []
        self.t = 0.0
        self.steps, self.cur, self.done_ids = {}, {}, {}     # each robot's step queue, the step it runs
        self.scale = {}              # name -> fspd scale of the running drive
        self.fw_refused = {n: 0 for n in self.names}
        self.track = {}              # name -> [(t, x, y, h)] each change of its true pose
        self.c.t = 0.0
        for n in self.names:
            self.c.on_fw(n, "2026-11-05c")
            self.c.on_battery(n, 90, 0.0)
            self.c.on_heard(n, 0.0)

    # -- the robots ---------------------------------------------------------------------------------------------
    def own(self, n):
        x, y, h = self.truth[n]
        ox, oy, oh = self.off[n]
        return x + ox, y + oy, FC.wrap180(h + oh)

    def out(self, name, text):
        self.sent.append((round(self.t, 2), name, text))
        p = text.split()
        if p[0] == "fpose":                       # pose.set: its own pose is what it is told
            x, y, h = float(p[1]), float(p[2]), float(p[3])
            tx, ty, th = self.truth[name]
            self.off[name] = [x - tx, y - ty, FC.wrap180(h - th)]
        elif p[0] == "fix":                        # pose.fix with its gate (fix_gate 4, var ~1 cm2, sigma ~1.5)
            fx, fy = float(p[2]), float(p[3])
            ox, oy, _ = self.own(name)
            if self.fuse_then:
                # (e18) the firmware fuses a fix against its pose AT THE PICTURE'S TIME (pose.h keeps its history),
                # not its pose now: before, a delayed fix pulled a driving robot's own pose back along its track
                # (~1.5 cm a fix at 10 cm/s), and the course check read that as a 15 deg heading error
                tf = int(p[1]) / 1000.0
                hist = self.own_hist.get(name) or []
                then = next((q for q in reversed(hist) if q[0] <= tf + 1e-6), None)
                if then is not None:
                    ox, oy = then[1], then[2]
            if math.hypot(fx - ox, fy - oy) > max(3.0, 4.0 * math.sqrt(1.0 + 1.5 ** 2)):
                self.fw_refused[name] += 1
            else:
                self.off[name][0] += 0.35 * (fx - ox)
                self.off[name][1] += 0.35 * (fy - oy)
        elif p[0] == "fs" and len(p) >= 4 and p[2] in ("T", "D", "R"):
            sid = int(p[1])
            if any(q[0] == sid for q in self.steps.setdefault(name, [])) or sid in self.done_ids.setdefault(name, set()):
                return                                 # a resend
            self.steps[name].append((sid, p))
            self.c.on_step(name, sid, "queued")
        elif p[0] in ("fspd", "fsteer", "farc"):           # the running drive changed (the firmware acks: FSPD)
            if p[0] == "fspd":
                self.scale[name] = float(p[1])
            elif p[0] == "farc" and self.cur.get(name) is not None:
                self.cur[name]["k"] = float(p[1])
            self.c.on_fspd(name)
        elif p[0] == "halt":
            for sid, _ in self.steps.get(name, []):
                self.c.on_step(name, sid, "dropped", "halt")
            self.steps[name] = []
            self.cur.pop(name, None)
            self.c.on_step(name, 0, "halted")

    def _start_next(self, name):
        q = self.steps.get(name) or []
        if not q:
            return
        sid, p = q.pop(0)
        self.scale[name] = 1.0                                # every step starts at its own speed (follow.h)
        if p[2] == "T":
            h1 = float(p[3])
            turn = FC.wrap180(h1 - self.own(name)[2])        # the truth turns by what its own heading must turn
            th = self.truth[name][2]
            self.cur[name] = dict(kind="T", sid=sid, t0=self.t, t1=self.t + 0.25 + abs(turn) / 75.0, h0=th,
                                  h1=th + turn)
        elif p[2] == "R":                                     # a curve piece: x y h k L v (driven as an arc)
            k, L, v = float(p[6]), float(p[7]), float(p[8])
            self.cur[name] = dict(kind="D", sid=sid, L=L, v=max(0.5, v), done=0.0, rev=False, k=k)
        else:
            x0, y0, x1, y1, h, v = (float(q_) for q_ in p[3:9])
            rev = len(p) > 9 and p[9] == "1"
            self.cur[name] = dict(kind="D", sid=sid, L=math.hypot(x1 - x0, y1 - y0), v=max(0.5, v), done=0.0,
                                  rev=rev)
        self.c.on_step(name, sid, "started")

    def wiggle(self, name, why):
        return self.c.identity_wiggle(self.c.R[name], why, spots=getattr(self.layer, "wiggle_spots", None),
                                      req=getattr(self.layer, "wiggle_req", None))

    def log(self, table, row):
        if table == "events":
            self.events.append((round(self.t, 2), row["robot"], row["kind"], row["text"]))
        self.logs.append(dict(row, table=table))

    # -- the loop ------------------------------------------------------------------------------------------------
    def run(self, secs, dt=0.05, begin=True, hook=None):
        if begin and self.t == 0.0:
            for n in self.names:
                x, y, h = self.own(n)
                self.c.on_follow(n, 0.0, "idle", x, y, h)
                self.layer.seed(n, 0.0, x, y, "start")     # the layer starts where its own pose says (as the run)
                self.c.begin(n)
                self.c.R[n].busy_until = 1e9               # no roaming moves: only what the tests ask
        end = self.t + secs
        next_pic = self.t
        while self.t < end - 1e-9:
            self.t = round(self.t + dt, 6)
            self.c.t = self.t
            for n in list(self.names):
                cu = self.cur.get(n)
                if cu is None:
                    self._start_next(n)
                    cu = self.cur.get(n)
                th_before = self.truth[n][2]
                xy_before = (self.truth[n][0], self.truth[n][1])
                if cu is not None and cu["kind"] == "T":
                    f = min(1.0, (self.t - cu["t0"]) / max(1e-6, cu["t1"] - cu["t0"]))
                    self.truth[n][2] = FC.wrap180(cu["h0"] + f * FC.wrap180(cu["h1"] - cu["h0"]))
                    if f >= 1.0:
                        self.cur.pop(n)
                        self.done_ids.setdefault(n, set()).add(cu["sid"])
                        self.c.on_step(n, cu["sid"], "done")
                elif cu is not None:
                    step = min(cu["v"] * self.scale.get(n, 1.0) * dt, cu["L"] - cu["done"])
                    th = math.radians(self.truth[n][2])
                    sg = -1.0 if cu["rev"] else 1.0
                    self.truth[n][0] += sg * step * math.cos(th)
                    self.truth[n][1] += sg * step * math.sin(th)
                    cu["done"] += step
                    if cu.get("k"):
                        self.truth[n][2] = FC.wrap180(self.truth[n][2] + math.degrees(cu["k"] * step))
                    if cu["done"] >= cu["L"] - 1e-6:
                        self.cur.pop(n)
                        self.done_ids.setdefault(n, set()).add(cu["sid"])
                        self.c.on_step(n, cu["sid"], "done")
                self._odo_step(n, xy_before, th_before, dt, cu)
                if self.fuse_then:
                    oh_ = self.own_hist.setdefault(n, [])
                    oh_.append((self.t,) + tuple(self.own(n)))
                    del oh_[:-80]
                tr = self.track.setdefault(n, [])
                if not tr or tr[-1][1:] != tuple(round(v_, 3) for v_ in self.truth[n]):
                    tr.append((self.t,) + tuple(round(v_, 3) for v_ in self.truth[n]))
                x, y, h = self.own(n)
                moving = n in self.cur
                self.c.on_follow(n, self.t, "drive" if moving else "idle", x, y, h)
                self.c.on_heard(n, self.t)
            if self.t >= next_pic - 1e-9:
                next_pic = self.t + 1.0 / self.fps
                dets = []
                for n in self.names:
                    tx, ty, th = self.truth[n]
                    if any(d[0] <= tx <= d[2] and d[1] <= ty <= d[3] for d in self.dark):
                        continue                               # (e18) the camera sees nothing on a dark patch
                    hx = tx + self.lights_off * math.cos(math.radians(th)) + self.rng.gauss(0, self.noise)
                    hy = ty + self.lights_off * math.sin(math.radians(th)) + self.rng.gauss(0, self.noise)
                    dets.append(TL.Det("lights", self.t, hx, hy, spin=self.spin_flag and
                                       self.cur.get(n, {}).get("kind") == "T"))
                for n, r in self.c.R.items():
                    self.layer.set_unaccounted(n, self.c.unaccounted_why(r))
                    o = self.c.own_at(r, self.t)
                    if o is not None:
                        self.layer.own(n, self.t, o[0], o[1], o[2], n not in self.cur)
                for f in self.layer.feed(self.t, dets):
                    self.q.append((self.t + self.delay, f))
                for rv in self.layer.take_revoked():
                    self.c.on_elim_revoked(rv)
            due = [e for e in self.q if e[0] <= self.t]
            self.q = [e for e in self.q if e[0] > self.t]
            for _, f in due:
                n = f["robot"]
                self.c.on_fix(n, f["t"], f["x"], f["y"], f["th"], arrival=self.t, sigma=f["sigma"], src=f["src"],
                              how=f.get("how"))
                if not f.get("how") and self.c.R[n].lost is None:
                    self.out(n, "fix %d %.2f %.2f - %.2f 3.0" % (int(f["t"] * 1000), f["x"], f["y"], f["sigma"]))
            self.c.tick(self.t)
            if hook is not None:
                hook(self)

    def _odo_step(self, n, xy0, th0, dt, cu):
        """(e18) Its own pose drives on its wheels and gyro with its errors; RAW rows of what they measured."""
        we, ge, nz = self.odo[n]
        x1, y1, th1 = self.truth[n]
        d = math.hypot(x1 - xy0[0], y1 - xy0[1])
        dth = FC.wrap180(th1 - th0)
        sg = -1.0 if (cu is not None and cu.get("rev")) else 1.0
        if not (we or ge or nz or self.raw):
            return
        rd = self.odo_rng.gauss(0.0, nz) if nz else 0.0
        rt = self.odo_rng.gauss(0.0, nz) if nz else 0.0
        own_d = d * (1.0 + we + rd)
        own_dth = dth * (1.0 + ge + rt)
        ox, oy, oh = self.own(n)
        # its own pose: the truth's step re-done on its own heading with its errors, kept as an offset from the truth
        oh_mid = oh + 0.5 * own_dth
        ndx, ndy = sg * own_d * math.cos(math.radians(oh_mid)), sg * own_d * math.sin(math.radians(oh_mid))
        self.off[n][0] += ndx - (x1 - xy0[0])
        self.off[n][1] += ndy - (y1 - xy0[1])
        self.off[n][2] = FC.wrap180(self.off[n][2] + own_dth - dth)
        if self.raw and dt > 0:
            self.c.on_raw_full(n, self.t, sg * own_d / dt, own_dth / dt, None, d < 1e-6 and abs(dth) < 1e-6)

    @property
    def odo_rng(self):
        if not hasattr(self, "_odo_rng"):
            self._odo_rng = random.Random(4242)
        return self._odo_rng

    def kinds(self, name=None):
        return [e[2] for e in self.events if name is None or e[1] == name]

    def first(self, kind, name=None):
        return next((e for e in self.events if e[2] == kind and (name is None or e[1] == name)), None)

    def gap(self, n):
        """How far the controller's position for n is from the truth (cm)."""
        p = self.c.pos(self.c.R[n])
        return math.hypot(p[0] - self.truth[n][0], p[1] - self.truth[n][1])

    def own_gap(self, n):
        return math.hypot(self.off[n][0], self.off[n][1])

    def longest_still(self, n, t_from=0.0):
        """The longest the robot's true pose stood unchanged after t_from (s)."""
        tr = [q for q in self.track.get(n, []) if q[0] >= t_from]
        ts = [t_from] + [q[0] for q in tr] + [self.t]
        return max(b - a for a, b in zip(ts, ts[1:])) if len(ts) > 1 else self.t - t_from
