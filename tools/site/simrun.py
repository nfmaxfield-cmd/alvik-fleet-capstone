"""Site survey on the simulator, as fast as the computer allows (not in real
time): the simulated robots (tools/sim/robot.py) run the firmware's own follow
code (fw_follow.py, a copy of follow.h / pose.h), its calibration run and its
edge guard; a coordinator stands in for tools/fleet/run_plan.py (releases the
plan's steps one at a time after each ack, at most 2 ahead, and passes camera
fixes on 10 times a second, 0.1 s late); a camera stands in for the overhead
camera and the detector (the true pose plus noise plus a fixed, smooth
calibration error over the floor).

What it returns is what a real survey leaves for tools/site/observe.py: per run
the RAW rows on the laptop clock (Run objects as tools/fusion/runs.py builds
them), the EDGE records, and the camera's sightings (t, x, y, heading).

The live, real-time version of the same plans is `tools/fleet/run_plan.py
--dry-run` against tools/sim/fleet_sim.py (setup.py --dry-run uses it).
"""
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (HERE, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "fusion"), os.path.join(TOOLS, "fleet")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import config as SIMCFG  # noqa: E402
import robot as SIMROBOT  # noqa: E402
import geometry as G  # noqa: E402
import runs as RN  # noqa: E402

FIX_HZ, FIX_DELAY_S = 10.0, 0.1


class CameraModel:
    """The overhead camera as the survey sees it: sightings at `hz` with noise sd `sigma_cm` / `sigma_deg`,
    and a calibration error that is the same every time at a given floor spot (a smooth field with rms
    `sys_cm`: lens and plywood shape the floor calibration did not catch)."""

    def __init__(self, seed, sigma_cm=0.3, sigma_deg=3.0, sys_cm=0.15, hz=FIX_HZ, area=(0, 122, 0, 122)):
        rng = np.random.default_rng(seed)
        self.sigma_cm, self.sigma_deg, self.hz, self.sys_cm = sigma_cm, sigma_deg, hz, sys_cm
        self.rng = np.random.default_rng(seed + 1)
        # 3 smooth waves per axis (wavelengths 60-200 cm), scaled to rms sys_cm per axis
        self.waves = []
        for _ in range(2):
            ks = rng.uniform(2 * math.pi / 200, 2 * math.pi / 60, (3, 2)) * rng.choice([-1, 1], (3, 2))
            ph = rng.uniform(0, 2 * math.pi, 3)
            amp = rng.normal(0, 1, 3)
            self.waves.append((ks, ph, amp))
        xs, ys = np.meshgrid(np.linspace(area[0], area[1], 25), np.linspace(area[2], area[3], 25))
        P = np.c_[xs.ravel(), ys.ravel()]
        self.off = np.zeros(2)
        raw = self._field(P)
        self.off = -raw.mean(0)                        # no mean shift (the calibration's tape points fix that)
        rms = np.sqrt(((raw + self.off) ** 2).mean(0))
        self.scale = np.where(rms > 0, sys_cm / np.maximum(rms, 1e-9), 0.0)

    def _field(self, P):
        P = np.atleast_2d(P)
        out = np.zeros((len(P), 2))
        for ax, (ks, ph, amp) in enumerate(self.waves):
            out[:, ax] = (amp[None, :] * np.sin(P @ ks.T + ph[None, :])).sum(1)
        return out + self.off

    def error(self, x, y):
        return self._field([[x, y]])[0] * getattr(self, "scale", 1.0)

    def sight(self, x, y, th):
        e = self.error(x, y)
        return (x + e[0] + self.rng.normal(0, self.sigma_cm), y + e[1] + self.rng.normal(0, self.sigma_cm),
                math.degrees(th) + self.rng.normal(0, self.sigma_deg))


class CoordRadio:
    """run_plan.py's part: steps after acks (at most `ahead` beyond the one running), camera fixes, stop at
    the end. Also records the camera's sightings."""

    def __init__(self, sim, rb, steps, ahead=2):
        self.sim, self.rb = sim, rb
        self.steps = list(steps)
        self.ahead = ahead
        self.cmds = []
        self.next_i = 0
        self.unacked = None
        self.done = set()
        self.failed = None
        self.stop_sent = False
        self.next_fix = None
        self.fixq = []
        self.ids = {}

    def poll(self, rb):
        t = rb.t_at(rb.ms)
        self.sim.camera_tick(rb, t)
        if self.next_fix is None:
            self.next_fix = t
        while t >= self.next_fix:
            s = self.sim.last_sight
            if s is not None:
                self.fixq.append((s[0] + FIX_DELAY_S, "fix %d %.2f %.2f %.2f %.2f %.2f" % (
                    int(rb.ms_at(s[0])), s[1], s[2], s[3], self.sim.cam.sigma_cm, self.sim.cam.sigma_deg)))
            self.next_fix += 1.0 / FIX_HZ
        if self.fixq and self.fixq[0][0] <= t:
            return self.fixq.pop(0)[1]
        if self.stop_sent or not rb.f_active:      # run_plan.py waits for the FOLLOW records first
            return None
        finished = self.failed is not None or len(self.done) == len(self.steps)
        if finished:
            self.stop_sent = True
            return "stop"
        if self.unacked is None and self.next_i < len(self.steps):
            running = self.next_i - len(self.done)
            if running <= self.ahead:
                sid = self.sim.next_step_id(rb.name)
                st = self.steps[self.next_i]
                self.ids[sid] = self.next_i
                self.unacked = sid
                self.next_i += 1
                return step_command(sid, st)
        return None

    def transmit(self, rb, text):
        if not text.startswith("D,STEP,"):
            return
        v = text.split(",")
        sid, state = int(v[3]), v[4]
        if sid == self.unacked and state in ("queued", "dup"):
            self.unacked = None
        if state == "done" and sid in self.ids:
            self.done.add(sid)
        if state == "failed" and sid in self.ids:
            self.failed = (sid, ",".join(v[10:]))


def step_command(sid, st):
    """The same text tools/fleet/run_plan.py sends."""
    import run_plan as RP
    return RP.step_command(sid, st)


class SimSite:
    """Simulated robots on the true world map, one at a time on the board (the others lifted off)."""

    def __init__(self, world, cfg, seed=1, camera=None, board_cm=None, open_edges=(), params=None):
        self.world = world
        self.cfg = cfg
        if board_cm is not None:
            self.cfg.session.board_cm = list(board_cm)
            self.cfg.session.open_edges = list(open_edges)
        self.rng = random.Random(seed)
        self.cam = camera or CameraModel(seed)
        self.polys = [G.segment_rect(b["p1"], b["p2"], b["thickness_cm"]).tolist() for b in world["boards"]]
        for o in world["obstacles"]:
            if o["shape"] == "polygon":
                self.polys.append([list(p) for p in o["points"]])
            else:
                a = np.linspace(0, 2 * np.pi, 16, endpoint=False)
                c, r = o["center"], o["radius"]
                self.polys.append([[c[0] + r * math.cos(t), c[1] + r * math.sin(t)] for t in a])
        self.params = SIMCFG.firmware_params()
        self.fw = SIMCFG.firmware_version()
        self.robots = {}
        self.sightings = {}         # robot -> [(t, x, y, th_deg)]
        self.last_sight = None
        self.t_next_sight = {}
        self.step_ids = {}
        self.t = 1.0e6              # laptop clock (s) of the simulation
        self.robot_time_s = 0.0
        self.extra_params = dict(params or {})

    def robot(self, name, start):
        """Power the robot on and put it on its start spot (+-0.5 cm, 1.5 deg by hand)."""
        if name in self.robots:
            return self.robots[name]
        rb = SIMROBOT.Robot(name, self.cfg, random.Random(self.rng.random()), self.params, self.fw,
                            self.t - self.rng.uniform(60, 600), self.rng.randrange(0x1000, 0x10000), "sim")
        rb.idle_until(self.t)
        rb.x = start[0] + self.rng.uniform(-0.5, 0.5)
        rb.y = start[1] + self.rng.uniform(-0.5, 0.5)
        rb.th = math.radians(start[2] + self.rng.uniform(-1.5, 1.5))
        rb._record()
        rb.world_polys = self.polys
        rb.fw_yaw_sign = 1                           # calibrated since power-on (the per-robot calibration run)
        for k, v in self.extra_params.items():
            rb.params[k] = float(v)
        self.robots[name] = rb
        self.sightings[name] = []
        return rb

    def next_step_id(self, name):
        self.step_ids[name] = self.step_ids.get(name, 0) + 1
        return self.step_ids[name]

    def camera_tick(self, rb, t):
        nt = self.t_next_sight.get(rb.name, t)
        while t >= nt:
            x, y, th = rb.pose_at_ms(rb.ms_at(nt)) if hasattr(rb, "pose_at_ms") else (rb.x, rb.y, rb.th)
            s = (nt,) + self.cam.sight(x, y, th)
            self.sightings[rb.name].append(s)
            self.last_sight = s
            nt += 1.0 / self.cam.hz
        self.t_next_sight[rb.name] = nt

    def _loop(self, rb, radio, first_cmds, exp=None):
        """The firmware's idle loop: commands, then the experiment they start (blocks until it ends)."""
        rb.radio = radio
        for c in first_cmds:
            rb.handle_command(c, False)
        t0 = rb.t_at(rb.ms)
        guard = 0
        while rb.pending < 0 and guard < 1000:
            rb.advance(10)
            radio.poll(rb)
            guard += 1
        i = rb.pending
        rb.pending = -1
        run = rb.run_experiment(SIMROBOT.EXP_NAMES[i])
        for _ in range(30):                         # a moment of standing still after the run (camera)
            rb.advance(10)
            radio.poll(rb)
        self.robot_time_s += rb.t_at(rb.ms) - t0
        self.t = max(self.t, rb.t_at(rb.ms))
        rb.radio = None
        return run

    def calibrate(self, name, start):
        rb = self.robot(name, start)
        radio = CoordRadio(self, rb, [])
        radio.stop_sent = True                       # nothing to stop: the calibration run ends itself
        return self._loop(rb, radio, ["calibrate"])

    def follow(self, name, start, steps, settings=()):
        """One follow run with these plan steps. Returns (run dict, failed (step id, why) or None, indices of
        the steps reported done)."""
        rb = self.robot(name, start)
        for c in settings:
            rb.handle_command(c, False)
        radio = CoordRadio(self, rb, steps)
        run = self._loop(rb, radio, ["run follow"])
        return run, radio.failed, sorted(radio.ids[s] for s in radio.done)

    def true_pose(self, name):
        rb = self.robots[name]
        return rb.x, rb.y, math.degrees(rb.th)

    def seen_pose(self, name, last_s=1.0):
        """Where the camera puts the robot now: the median of the last second of sightings."""
        s = [v for v in self.sightings[name] if v[0] >= self.sightings[name][-1][0] - last_s]
        a = np.array(s)
        th = math.degrees(math.atan2(np.mean(np.sin(np.radians(a[:, 3]))), np.mean(np.cos(np.radians(a[:, 3])))))
        return float(np.median(a[:, 1])), float(np.median(a[:, 2])), th

    # ------------------------------------------------------------ logs as a real session gives them
    def runs(self, name):
        """Run objects (tools/fusion/runs.py) with RAW rows on the laptop clock, plus EDGE rows per run."""
        rb = self.robots[name]
        out, cols = {}, None
        edge_cols = touch_cols = None
        for ms, line in rb.lines:
            p = line.split(",")
            if line.startswith("H,RAW,"):
                cols = p[2:]
            elif line.startswith("D,RAW,") and cols:
                r = dict(zip(cols, p[2:]))
                run = out.get(r["run"])
                if run is None:
                    run = out[r["run"]] = RN.Run(r["run"], name)
                    run.edge = []
                r["t"] = rb.t_at(float(r["ms"]))
                run.raw.append(r)
            elif line.startswith("H,TOUCH,"):
                touch_cols = p[2:]
            elif line.startswith("D,TOUCH,") and touch_cols:
                r = dict(zip(touch_cols, p[2:]))
                r["t"] = rb.t_at(float(r["ms_click"]))
                r["t_start"] = rb.t_at(float(r["ms_start"]))
                if r["run"] in out:
                    out[r["run"]].touch.append(r)
            elif line.startswith("H,EDGE,"):
                edge_cols = p[2:]
            elif line.startswith("D,EDGE,") and edge_cols:
                r = dict(zip(edge_cols, p[2:]))
                r["t"] = rb.t_at(float(r["ms"]))
                if r["run"] in out:
                    out[r["run"]].edge.append(r)
        for r in rb.runs:
            if r["run"] in out:
                out[r["run"]].experiment = r["exp"]
                out[r["run"]].result = r["result"]
        return list(out.values())
