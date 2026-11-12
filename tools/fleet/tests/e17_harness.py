"""Build e17: a small deterministic rig for the tracking layer + the free-roam controller (no robots, no Pi).

Robots stand where they are put (truth); each reports its OWN pose (truth + an offset that only an fpose removes, as
the firmware's pose.set does); the camera sees every robot's lights (no names) with noise, its fixes delayed; the
layer names them and the controller gets them as the Pi's runner gives them (freeroam.Runner.fixes): corrections
(how) as on_fix(..., how), the rest as fixes the robot's own filter would weigh, gated as firmware pose.h does
(fix_gate 4 sigmas and 3 cm: a far fix is refused). Turn steps (a wiggle) are carried out (the heading changes, the
lights spot swings round its middle, 1 cm ahead of it); drive steps are carried out straight at their speed.
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
                 lights_off_cm=1.0, wiggle=True, fps=5.0, never_still=False):
        self.rng = random.Random(seed)
        self.names = list(truth)
        self.truth = {n: list(v) for n, v in truth.items()}          # x, y, heading
        self.off = {n: list((own_off or {}).get(n, (0.0, 0.0, 0.0))) for n in self.names}
        self.events, self.logs, self.sent = [], [], []
        self.c = FC.Controller(self.names, BOARD, self.out, self.log, pose_source="blend")
        self.layer = TL.TrackingLayer(self.names, ("motion", "lock", "lights"), log=lambda k, r: self.logs.append(r),
                                      wiggle=(self.wiggle if wiggle else None), board=BOARD, eliminate=eliminate,
                                      gate_record=gate_record, camera_watch=False)
        self.c.record_fn = lambda n, t, d, trav: self.layer.record(n, t, d, trav)
        self.c.never_still = never_still
        self.delay, self.noise, self.lights_off, self.fps = delay, noise, lights_off_cm, fps
        self.q = []
        self.t = 0.0
        self.steps, self.cur, self.done_ids = {}, {}, {}     # each robot's step queue, the step it runs
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
            if math.hypot(fx - ox, fy - oy) > max(3.0, 4.0 * math.sqrt(1.0 + 1.5 ** 2)):
                self.fw_refused[name] += 1
            else:
                self.off[name][0] += 0.35 * (fx - ox)
                self.off[name][1] += 0.35 * (fy - oy)
        elif p[0] == "fs" and len(p) >= 4 and p[2] in ("T", "D"):
            sid = int(p[1])
            if any(q[0] == sid for q in self.steps.setdefault(name, [])) or sid in self.done_ids.setdefault(name, set()):
                return                                 # a resend
            self.steps[name].append((sid, p))
            self.c.on_step(name, sid, "queued")
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
        if p[2] == "T":
            h1 = float(p[3])
            turn = FC.wrap180(h1 - self.own(name)[2])        # the truth turns by what its own heading must turn
            th = self.truth[name][2]
            self.cur[name] = dict(kind="T", sid=sid, t0=self.t, t1=self.t + 0.25 + abs(turn) / 75.0, h0=th,
                                  h1=th + turn)
        else:
            x0, y0, x1, y1, h, v = (float(q_) for q_ in p[3:9])
            rev = len(p) > 9 and p[9] == "1"
            self.cur[name] = dict(kind="D", sid=sid, L=math.hypot(x1 - x0, y1 - y0), v=max(0.5, v), done=0.0,
                                  rev=rev)
        self.c.on_step(name, sid, "started")

    def wiggle(self, name, why):
        self.c.identity_wiggle(self.c.R[name], why, spots=getattr(self.layer, "wiggle_spots", None))

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
                if cu is not None and cu["kind"] == "T":
                    f = min(1.0, (self.t - cu["t0"]) / max(1e-6, cu["t1"] - cu["t0"]))
                    self.truth[n][2] = FC.wrap180(cu["h0"] + f * FC.wrap180(cu["h1"] - cu["h0"]))
                    if f >= 1.0:
                        self.cur.pop(n)
                        self.done_ids.setdefault(n, set()).add(cu["sid"])
                        self.c.on_step(n, cu["sid"], "done")
                elif cu is not None:
                    step = min(cu["v"] * dt, cu["L"] - cu["done"])
                    th = math.radians(self.truth[n][2])
                    sg = -1.0 if cu["rev"] else 1.0
                    self.truth[n][0] += sg * step * math.cos(th)
                    self.truth[n][1] += sg * step * math.sin(th)
                    cu["done"] += step
                    if cu["done"] >= cu["L"] - 1e-6:
                        self.cur.pop(n)
                        self.done_ids.setdefault(n, set()).add(cu["sid"])
                        self.c.on_step(n, cu["sid"], "done")
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
                    hx = tx + self.lights_off * math.cos(math.radians(th)) + self.rng.gauss(0, self.noise)
                    hy = ty + self.lights_off * math.sin(math.radians(th)) + self.rng.gauss(0, self.noise)
                    dets.append(TL.Det("lights", self.t, hx, hy, spin=self.cur.get(n, {}).get("kind") == "T"))
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
