#!/usr/bin/env python3
"""Kinematic simulator: drives every robot through plan.json the way the
real robots will, with noise and delays, and checks the result.

    python tools/fleet/simulate_plan.py --map map.json --plan plan.json
    python tools/fleet/simulate_plan.py --map map.json --plan plan.json --brick auto@12
    python tools/fleet/simulate_plan.py --map map.json --plan plan.json --runs 20 --tof realistic

What each simulated robot does (a stand-in for the robot-side follower):
  * turn / park: turns in place until its pose estimate says it faces the
    target heading;
  * drive: drives straight at the target point, steering on its pose
    estimate, until the estimate says it got there (reverse for backing out);
  * approach: creeps forward; the front distance sensor's readings (15 a
    second, 0.2 cm noise) plus the wheel distance give the board's position,
    averaged; below the step's handover_cm the wheels finish so the front face
    stops gap_cm from the board, never travelling more than max_travel_cm
    (--tof ideal: a perfect sensor that stops exactly at gap_cm, the old default);
  * wait: a dwell at a board lasts its time; other waits only wait for the
    step order rules ("after"), or with --gate time also for the clock.

The pose estimate is the true pose plus a slowly wandering error (like the
fused camera + wheels + gyro estimate, 0.3 cm / 0.4 deg by default), so
errors do not pile up from step to step. Each step starts after a random
radio delay; --stall-prob adds occasional multi-second hiccups.

--brick puts a brick (20 x 9 cm) on the floor mid-run. The camera reports it
after --cam-delay seconds; a robot whose distance sensor sees it first stops
and reports it. Then the obstacle rules run (planner.obstacle_event): robots
that would reach it within the stop horizon stop at once, the others carry
on, and everyone gets a new plan.

Checks: every robot finishes; front face within dock_tol_cm of gap_cm at
every dock (and not touching); the closest two bodies ever came; nothing hit.
The robot-side follower itself (firmware follow.h) is simulated in more
detail by tools/sim/fleet_sim.py with tools/fleet/run_plan.py --dry-run.
"""
import argparse
import copy
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import fleetmap as FM    # noqa: E402
import geometry as G     # noqa: E402
import planner as P      # noqa: E402

DEFAULT_SIM = {
    "dt": 0.02,
    "pose_sigma_cm": 0.3,       # pose estimate error (fusion), standard deviation
    "heading_sigma_deg": 0.4,
    "estimate_tau_s": 1.0,      # how slowly that error wanders
    "speed_err": 0.03,          # actual speed / commanded, standard deviation
    "turn_rate_err": 0.05,
    "turn_stop_deg": 0.5,       # turn ends when the estimate is within this
    "turn_overshoot_deg": 0.5,  # extra random heading error at the end of a turn
    "drift_dps": 0.5,           # heading drift while driving, standard deviation per step
    "latency_s": 0.15,          # radio delay before each step starts, up to
    "stall_prob": 0.0,          # chance a step start is delayed by a hiccup
    "stall_s": 2.0,             # hiccup length, up to
    "cam_delay_s": 0.5,         # camera reports a new obstacle after this
    "cam_sigma_cm": 0.5,
    "safe_front_cm": 7.0,       # distance-sensor stop for unexpected things ahead
    "tof_fov_deg": 60.0,        # sideways field of view of the front sensor (VL53L7CX: 60 x 60 deg)
    "max_replans": 12,
    "retry_s": 3.0,             # a robot left without a route gets a new try this often
    "improve_after_s": 1.0,     # after a surprise: try a better plan this long after the new one starts (0 = off)
    "improve_budget_s": 6.0,    # time allowed for that try (robots keep going meanwhile)
    "brick_ahead_s": 3.0,       # --brick auto: put it about this many seconds of driving ahead of a robot
    "tof": "realistic",         # realistic (default: sensor rate, noise, wheels for the last cm) | ideal
    "tof_hz": 15.0,             # realistic: readings per second
    "tof_sigma_cm": 0.2,        # realistic: noise
    "tof_handover_cm": 3.0,     # realistic: below this, finish on the wheels (the step's handover_cm wins)
    "tof_fuse_from_cm": 10.0,   # realistic: readings below this are averaged (with wheel travel)
    "wheel_err": 0.02,          # realistic: wheel distance error over the last few cm (fraction)
    "gate": "deps",             # deps | time | both
    "dock_tol_cm": 0.25,        # a docking passes if the front face stops within this of the step's gap_cm
    "timeout_s": 400.0,
}

BRICK = (20.0, 9.0)


# ----------------------------------------------------------------------------
# Small helpers
# ----------------------------------------------------------------------------

def ray_hit(o, u, polys, max_d=200.0):
    """Distance along ray o + t u to the first polygon edge (and its index)."""
    best, who = max_d, None
    for k, poly in enumerate(polys):
        for i in range(len(poly)):
            a, b = poly[i], poly[(i + 1) % len(poly)]
            e = b - a
            den = u[0] * (-e[1]) - u[1] * (-e[0])
            if abs(den) < 1e-12:
                continue
            r = a - o
            t = (r[0] * (-e[1]) - r[1] * (-e[0])) / den
            sgm = (u[0] * r[1] - u[1] * r[0]) / den
            if t >= 0 and 0 <= sgm <= 1 and t < best:
                best, who = t, k
    return best, who


def brick_poly(cx, cy, ang):
    c, s = math.cos(ang), math.sin(ang)
    hx, hy = BRICK[0] / 2, BRICK[1] / 2
    pts = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
    return np.array([[cx + c * x - s * y, cy + s * x + c * y] for x, y in pts])


class Robot:
    def __init__(self, name, pose, rng, sp):
        self.name = name
        self.true = np.array(pose, float)
        self.err = np.zeros(3)
        self.rng = rng
        self.sp = sp
        self.step = 0            # index in the current plan
        self.state = "idle"      # idle | delay | run | settle | done
        self.t_ready = 0.0
        self.run = {}
        self.done_steps = set()
        self.finished_at = None
        self.stopped_reason = None

    def belief(self):
        return self.true + self.err

    def tick_estimate(self, dt):
        sp = self.sp
        a = math.exp(-dt / sp["estimate_tau_s"])
        q = math.sqrt(1 - a * a)
        sig = np.array([sp["pose_sigma_cm"], sp["pose_sigma_cm"], math.radians(sp["heading_sigma_deg"])])
        self.err = a * self.err + q * sig * self.rng.standard_normal(3)


# ----------------------------------------------------------------------------
# The simulation
# ----------------------------------------------------------------------------

class Sim:
    def __init__(self, m, plan, sp, seed=0, brick=None, verbose=True):
        self.m = FM.normalize(m)
        self.plan = plan
        self.sp = sp
        self.rng = np.random.default_rng(seed)
        self.body = FM.body_of(self.m)
        self.s = P.settings(plan["settings"])
        self.docks = FM.docks(self.m)
        self.boards = {b["name"]: G.segment_rect(b["p1"], b["p2"], b["thickness_cm"]) for b in self.m["boards"]}
        self.gone_boards = set()      # boards taken away without telling the plan (a backing-in touch finds none)
        self.robots = {n: Robot(n, FM.pose_in(r["start"]), np.random.default_rng(seed * 101 + i), sp)
                       for i, (n, r) in enumerate(plan["robots"].items())}
        self.t = 0.0
        self.plan_start = 0.0
        self.brick_spec = brick
        self.brick = None          # true polygon once on the floor
        self.brick_t = None
        self.brick_known = False
        self.events = []
        self.replans = []
        self.docking = []
        self.min_gap = math.inf
        self.min_gap_at = None
        self.min_static = math.inf
        self.min_static_at = None
        self.verbose = verbose
        self.world_static = FM.World(self.m)
        self.grid = P.Grid(self.world_static, self.body, self.s)   # for quick lower bounds
        self.pending_detection = None
        self.next_retry = 0.0
        self.improve_at = None
        self.retries = 0

    # -- step bookkeeping -----------------------------------------------------
    def steps(self, r):
        return self.plan["robots"][r.name]["steps"]

    def deps_done(self, r, st):
        for other, j in st.get("after", []):
            if j not in self.robots[other].done_steps:
                return False
        return True

    def can_start(self, r, st):
        g = self.sp["gate"]
        if g in ("deps", "both") and not self.deps_done(r, st):
            return False
        if g in ("time", "both") and self.t < self.plan_start + st["t_start"] - 1e-9:
            return False
        if st.get("not_before_s") is not None and self.t < self.plan_start + st["not_before_s"] - 1e-9:
            return False
        return True

    def start_step(self, r, st, immediate=False):
        r.run = {"st": st, "t0": self.t}
        typ = st["type"]
        if typ in ("turn", "park"):
            r.run["target"] = math.radians(st["to"][2])
            r.run["rate"] = math.radians(st["speed"]) * (1 + self.sp["turn_rate_err"] * r.rng.standard_normal())
        elif typ in ("drive", "approach"):
            r.run["target"] = np.array(st["to"][:2], float)
            # progress is measured along the step's own planned direction (its
            # heading; backwards for reverse), never along "here -> target",
            # which is noise when a re-started step is almost done
            h = math.radians(st["to"][2])
            r.run["u"] = np.array([math.cos(h), math.sin(h)]) * (-1.0 if st.get("reverse") else 1.0)
            r.run["v"] = st["speed"] * (1 + self.sp["speed_err"] * r.rng.standard_normal())
            r.run["drift"] = math.radians(self.sp["drift_dps"]) * r.rng.standard_normal()
            r.run["travel"] = 0.0
            r.run["tof_next"] = self.t
            r.run["tof_read"] = None
            r.run["wheel_left"] = None
            if typ == "approach":
                # firmware follow.h: the travel cap counts from the step's start point, so a robot
                # that stopped a little short of the pre-dock may roll that much further
                L = math.dist(st["from"][:2], st["to"][:2])
                along = float(np.dot(np.array(st["to"][:2]) - r.belief()[:2], r.run["u"]))
                r.run["cap"] = st.get("max_travel_cm", 20.0) - (L - along)
        else:  # wait
            if st["phase"] in ("dwell", "pause") or self.sp["gate"] in ("time", "both"):
                dur = st["t_end"] - st["t_start"]
                if self.sp["gate"] in ("time", "both"):
                    r.run["until"] = max(self.t, self.plan_start + st["t_end"]) if st["phase"] != "dwell" \
                        else self.t + dur
                else:
                    r.run["until"] = self.t + dur
            else:
                r.run["until"] = self.t
        r.state = "run"

    def finish_step(self, r, settle):
        r.state = "settle" if settle > 0 else "idle"
        r.t_ready = self.t + settle
        r.done_steps.add(r.run["st"]["i"])
        r.step = r.run["st"]["i"] + 1
        if r.step >= len(self.steps(r)):
            r.state = "done" if settle <= 0 else "settle"
            r.finished_at = self.t + settle

    # -- motion ---------------------------------------------------------------
    def tof(self, r, polys):
        """Front distance sensor: distance from the front face centre along the heading."""
        x, y, th = r.true
        u = np.array([math.cos(th), math.sin(th)])
        o = np.array([x, y]) + self.body.sensor * u
        d, who = ray_hit(o, u, polys)
        return d, who

    def obstacle_ahead(self, r):
        """Distance-sensor safety stop: something other than the target
        board within safe_front_cm across the front (three rays)."""
        if self.brick is None:
            return None
        x, y, th = r.true
        u = np.array([math.cos(th), math.sin(th)])
        nrm = np.array([-u[1], u[0]])
        st = r.run["st"]
        lim = self.sp["safe_front_cm"]
        if st.get("front_clear_cm") is not None:
            lim = min(lim, st["front_clear_cm"] - 2.0)
        best = None
        for off in (-self.body.width / 2 + 0.5, 0.0, self.body.width / 2 - 0.5):
            o = np.array([x, y]) + self.body.sensor * u + off * nrm
            d, who = ray_hit(o, u, [self.brick])
            if who is not None and d < lim:
                if best is None or d < best[0]:
                    best = (d, o + d * u)
        return best

    def tof_outline(self, r, max_range=40.0):
        """What the 5 front zones see of unexpected things: hit points
        across the sensor's field of view, given some depth."""
        x, y, th = r.true
        o = np.array([x, y]) + self.body.sensor * np.array([math.cos(th), math.sin(th)])
        pts = []
        half = math.radians(self.sp["tof_fov_deg"]) / 2
        for a in np.linspace(-half, half, 21):
            u = np.array([math.cos(th + a), math.sin(th + a)])
            d, who = ray_hit(o, u, [self.brick], max_range)
            if who is not None:
                pts.append(o + d * u)
        if not pts:
            return None
        pts = np.array(pts)
        u = np.array([math.cos(th), math.sin(th)])
        pts = np.vstack([pts, pts + 6.0 * u])
        hull = G.convex_hull(pts)
        c = hull.mean(0)
        return c + (hull - c) * 1.0 + 1.0 * (hull - c) / np.maximum(np.linalg.norm(hull - c, axis=1)[:, None], 1e-6)

    def step_motion(self, r, dt):
        st = r.run["st"]
        typ = st["type"]
        bel = r.belief()
        sv = self.s
        if typ in ("turn", "park"):
            err = G.wrap(r.run["target"] - bel[2])
            if abs(err) <= math.radians(self.sp["turn_stop_deg"]):
                r.true[2] += math.radians(self.sp["turn_overshoot_deg"]) * r.rng.standard_normal() * 0.5
                self.finish_step(r, sv.turn_settle_s)
                return
            w = math.copysign(min(abs(r.run["rate"]), abs(err) / dt), err)
            r.true[2] = G.wrap(r.true[2] + w * dt)
            return
        if typ in ("drive", "approach"):
            rev = st.get("reverse", False)
            if typ == "drive" or st["phase"] == "escape":
                # straight to the target point, steering on the estimate
                rem_vec = r.run["target"] - bel[:2]
                rem = float(np.dot(rem_vec, r.run["u"]))
                if rem <= r.run["v"] * dt * 0.5:
                    last = st["i"] + 1 >= len(self.steps(r)) or self.steps(r)[st["i"] + 1]["type"] != "drive"
                    self.finish_step(r, sv.drive_settle_s if last else 0.0)
                    return
                head = math.atan2(rem_vec[1], rem_vec[0])
                if rev:
                    head = G.wrap(head + math.pi)
                if np.linalg.norm(rem_vec) < 2.0:
                    head = bel[2]
                w = float(np.clip(3.0 * G.wrap(head - bel[2]), -math.radians(30), math.radians(30)))
                v = -r.run["v"] if rev else r.run["v"]
                if self.sp["safe_front_cm"] > 0 and not rev:
                    hit = self.obstacle_ahead(r)
                    if hit is not None:
                        self.distance_sensor_stop(r, hit)
                        return
                self.move(r, v, w + r.run["drift"], dt)
                return
            if st.get("touch"):
                self.touch_step(r, st, bel, dt)
                return
            # approach: creep until the front sensor reads the stop distance
            polys = list(self.boards.values()) + ([self.brick] if self.brick is not None else [])
            stop_at = st.get("gap_cm") or self.m["dock"]["gap_cm"]
            handover = st.get("handover_cm") or self.sp["tof_handover_cm"]
            ideal_d = None
            if self.sp["tof"] == "ideal":
                ideal_d, _ = self.tof(r, polys)
                if ideal_d <= stop_at + 1e-6:
                    self.end_approach(r, st)
                    return
            else:
                if r.run["wheel_left"] is not None:
                    if r.run["wheel_left"] <= 0:
                        self.end_approach(r, st)
                        return
                elif self.t >= r.run["tof_next"]:
                    # each reading + the wheel distance so far = where the board is
                    # along the approach; average them (sensor fusion), then finish
                    # the last few cm on the wheels below the sensor's useful range
                    d, _ = self.tof(r, polys)
                    rd = d + self.sp["tof_sigma_cm"] * r.rng.standard_normal()
                    r.run["tof_next"] = self.t + 1.0 / self.sp["tof_hz"]
                    if rd <= self.sp["tof_fuse_from_cm"]:
                        r.run.setdefault("board_at", []).append(rd + r.run["travel"])
                    if rd <= handover:
                        est = float(np.mean(r.run.get("board_at", [rd + r.run["travel"]])))
                        r.run["wheel_left"] = est - r.run["travel"] - stop_at
            cap = r.run.get("cap", st.get("max_travel_cm", 20.0))
            if r.run["travel"] >= cap - 1e-9:
                # the firmware stops at the cap (never closer than gap - margin) and carries on
                r.run["capped"] = True
                self.end_approach(r, st)
                return
            # hold the board-facing heading; small sideways correction early on only
            dock = r.run["target"]
            rem_vec = dock - bel[:2]
            u = r.run["u"]
            lat = float(u[0] * rem_vec[1] - u[1] * rem_vec[0])
            along = float(np.dot(rem_vec, u))
            corr = float(np.clip(0.1 * lat, -math.radians(2), math.radians(2))) if along > 3.0 else 0.0
            head = math.atan2(u[1], u[0]) + corr
            w = float(np.clip(3.0 * G.wrap(head - bel[2]), -math.radians(10), math.radians(10)))
            v = r.run["v"]
            ds = v * dt
            if ideal_d is not None:
                ds = min(ds, max(0.0, ideal_d - stop_at) / max(1e-6, math.cos(0.0)))   # ideal: stop right at it
            if r.run["wheel_left"] is not None:
                ds = min(ds, max(0.0, r.run["wheel_left"]))
                r.run["wheel_left"] -= ds
            ds = min(ds, max(0.0, cap - r.run["travel"]))      # travel counted on the wheels
            if r.run["wheel_left"] is not None:
                ds *= 1 + r.run.setdefault("wheel_scale", self.sp["wheel_err"] * r.rng.standard_normal())
            self.move(r, ds / dt if dt > 0 else 0.0, w, dt)
            r.run["travel"] += ds
            return
        # wait
        if self.t >= r.run["until"] - 1e-9:
            self.finish_step(r, 0.0)

    def touch_step(self, r, st, bel, dt):
        """Backing in (switches at the back; follow step P, firmware dock.h): creep backwards holding the board's
        heading until the back face meets the board (a switch clicks; pressing both squares it to the board), then
        drive forward back_cm on the wheels. No click within the step's travel cap: the step fails."""
        d = self.docks[st["board"]]
        n = np.array(d["normal"])
        x, y, th = r.true
        u = np.array([math.cos(th), math.sin(th)])
        gap = float(np.dot(np.array([x, y]) - self.body.back * u - np.array(d["surface"]), n))
        if r.run.get("touch_out") is None:
            cap = r.run.get("cap", st.get("max_travel_cm", 20.0))
            if gap <= 1e-6 and st["board"] not in self.gone_boards:
                r.true[0] -= gap * n[0]                       # pressed flat against it
                r.true[1] -= gap * n[1]
                r.true[2] = math.atan2(n[1], n[0])            # both switches pressed: square to the board
                r.run["touch_out"] = st.get("back_cm", 0.0) * (1 + self.sp["wheel_err"] * r.rng.standard_normal())
                return
            if r.run["travel"] >= cap - 1e-9:
                r.run["capped"] = True
                self.end_approach(r, st, failed=True)
                return
            hd = math.radians(st["to"][2])
            w = float(np.clip(3.0 * G.wrap(hd - bel[2]), -math.radians(10), math.radians(10)))
            ds = min(r.run["v"] * dt, cap - r.run["travel"])
            if st["board"] not in self.gone_boards:
                ds = min(ds, gap / max(0.2, abs(float(np.dot(u, n)))))
            self.move(r, -ds / dt if dt > 0 else 0.0, w, dt)
            r.run["travel"] += ds
            return
        if r.run["touch_out"] <= 1e-9:
            self.end_approach(r, st)
            return
        ds = min(3.0 * dt, r.run["touch_out"])
        self.move(r, ds / dt if dt > 0 else 0.0, 0.0, dt)
        r.run["touch_out"] -= ds

    def move(self, r, v, w, dt):
        th = r.true[2]
        r.true[0] += v * math.cos(th) * dt
        r.true[1] += v * math.sin(th) * dt
        r.true[2] = G.wrap(th + w * dt)

    def end_approach(self, r, st, failed=False):
        d = self.docks[st["board"]]
        x, y, th = r.true
        u = np.array([math.cos(th), math.sin(th)])
        # the face that meets the board: the front, or the back attachment's when backing in (a touch step)
        front = np.array([x, y]) + (-self.body.back if st.get("touch") else self.body.front) * u
        n = np.array(d["normal"])
        gap = float(np.dot(front - np.array(d["surface"]), n))
        tang = np.array([-n[1], n[0]])
        lateral = float(np.dot(np.array([x, y]) - np.array(d["dock"][:2]), tang))
        herr = math.degrees(G.wrap(th - d["dock"][2]))
        corners = self.body.corners([r.true])[0]
        body_gap = G.polygon_polygon_dist(corners, self.boards[st["board"]])
        self.docking.append({"robot": r.name, "board": st["board"], "t": round(self.t, 2),
                             "front_gap_cm": round(gap, 3), "gap_error_cm": round(gap - st["gap_cm"], 3),
                             "lateral_cm": round(lateral, 3), "heading_err_deg": round(herr, 2),
                             "closest_body_to_board_cm": round(body_gap, 3), "failed": failed,
                             "travel_cap_reached": bool(r.run.get("capped")),
                             "ok": (not failed) and 0.0 < body_gap and abs(gap - st["gap_cm"]) <= self.sp["dock_tol_cm"]
                             and abs(herr) <= 5.0})
        self.finish_step(r, self.s.drive_settle_s)

    # -- surprises ------------------------------------------------------------
    def place_brick(self):
        spec = self.brick_spec
        if spec["where"] != "auto":
            x, y, a = spec["where"]
            self.brick = brick_poly(x, y, math.radians(a))
        else:
            self.brick = self._auto_brick()
        self.brick_t = self.t
        if self.brick is None:
            self.log("no good spot for an automatic brick; none placed")
            return
        c = self.brick.mean(0)
        self.log("t=%.1f brick put down at (%.1f, %.1f)" % (self.t, c[0], c[1]))
        self.pending_detection = ("camera", self.t + self.sp["cam_delay_s"])

    def _auto_brick(self):
        """Put the brick on a robot's path, a few seconds ahead of it, away
        from docks and from every robot."""
        for name, r in self.robots.items():
            steps = self.steps(r)
            acc = 0.0
            for st in steps[r.step:]:
                acc += st["t_end"] - st["t_start"] if st["type"] != "wait" or st["phase"] in ("dwell", "pause") else 0.0
                if st["type"] != "drive" or st["phase"] != "travel" or acc < self.sp["brick_ahead_s"]:
                    continue
                a = np.array(st["from"][:2])
                b = np.array(st["to"][:2])
                for frac in (0.5, 0.25, 0.75, 1.0):
                    mid = a + frac * (b - a)
                    ang = math.atan2(b[1] - a[1], b[0] - a[0]) + math.pi / 2
                    poly = brick_poly(mid[0], mid[1], ang)
                    if self._brick_ok(poly, r):
                        return poly
        return None

    def _brick_ok(self, poly, target):
        if True:
            if True:
                ok = True
                for o in self.robots.values():
                    near = min(25.0, 4.0 + self.sp["brick_ahead_s"] * 8.0) if o is target else 6.0
                    if G.polygon_polygon_dist(self.body.corners([o.true])[0], poly) < near:
                        ok = False
                for rr in self.m["robots"]:
                    for spot in (rr["start"], rr["park"]):
                        if float(G.points_to_polygon(np.array([spot[:2]]), poly)[0]) < 20.0:
                            ok = False
                for d in self.docks.values():
                    if float(G.points_to_polygon(np.array([d["predock"][:2], d["dock"][:2]]), poly).min()) < 12.0:
                        ok = False
                if self.world_static.shape_clearance(poly) < 1.0:
                    ok = False
                return ok

    def distance_sensor_stop(self, r, hit):
        d, pt = hit
        r.stopped_reason = "distance sensor saw something %.1f cm ahead" % d
        self.log("t=%.1f %s: distance sensor stop, something %.1f cm ahead" % (self.t, r.name, d))
        hull = self.tof_outline(r)
        if hull is None or len(self.replans) >= self.sp["max_replans"]:
            r.state, r.stopped_reason = "done", "gave up: blocked and out of replans"
            self.log("t=%.1f %s gives up (blocked, %d replans used)" % (self.t, r.name, len(self.replans)))
            return
        ob = {"id": "seen-by-%s-%d" % (r.name, len(self.replans) + 1), "shape": "polygon",
              "points": hull.round(2).tolist(), "source": "distance_sensor", "time": FM.now_iso()}
        self.handle_obstacle(ob, stopped_by_sensor=r.name)

    def with_budget(self, budget, fn):
        """Run a re-plan with more time allowed (robots keep going meanwhile)."""
        keep = self.s.replan_budget_s
        try:
            self.s["replan_budget_s"] = budget
            self.plan["settings"]["replan_budget_s"] = budget
            fn()
        finally:
            self.s["replan_budget_s"] = keep
            self.plan["settings"]["replan_budget_s"] = keep

    def improve(self):
        """Follow-up plan with more time (no new obstacle); robots carry on meanwhile."""
        if len(self.replans) >= self.sp["max_replans"]:
            return
        self._improving = True
        try:
            self.with_budget(self.sp["improve_budget_s"], lambda: self.handle_obstacle(None))
        finally:
            self._improving = False

    def camera_detection(self):
        poly = self.brick + self.sp["cam_sigma_cm"] * self.rng.standard_normal(2)
        c = poly.mean(0)
        poly = c + (poly - c) * (1 + 1.0 / (BRICK[1] / 2))   # about 1 cm bigger all round
        ob = {"id": "brick-1", "shape": "polygon", "points": poly.round(2).tolist(), "source": "camera",
              "time": FM.now_iso()}
        self.handle_obstacle(ob)

    def handle_obstacle(self, ob, stopped_by_sensor=None):
        if ob:
            self.brick_known = True
        states = {}
        for n, r in self.robots.items():
            steps = self.steps(r)
            if r.state == "run":
                st = r.run["st"]
                dl = (r.run["until"] - self.t) if st["type"] == "wait" else None
                states[n] = {"pose": r.belief().tolist(), "step": st["i"], "in_progress": True,
                             "dwell_left_s": dl}
            else:
                states[n] = {"pose": r.belief().tolist(), "step": min(r.step, len(steps)), "in_progress": False,
                             "dwell_left_s": None}
            if n == stopped_by_sensor:
                states[n]["in_progress"] = True
        t0 = time.perf_counter()
        try:
            newplan, events = P.obstacle_event(self.m, self.plan, states, [ob] if ob else [])
        except P.PlanError as e:
            self.log("t=%.1f replan failed: %s" % (self.t, e))
            self.replans.append({"t": round(self.t, 2), "error": str(e)})
            for r in self.robots.values():
                r.state = "done"
            return
        cpu = time.perf_counter() - t0
        if stopped_by_sensor:
            for e in events:
                if e["robot"] == stopped_by_sensor:
                    e.update(action="stopped", reason="its distance sensor saw the obstacle ahead")
        self.replans.append({"t": round(self.t, 2), "source": ob["source"] if ob else "retry", "cpu_s": round(cpu, 3),
                             "budget_s": self.s.replan_budget_s, "events": events,
                             "new_makespan_s": newplan["summary"]["makespan_s"],
                             "plan_min_separation_cm": newplan["summary"]["min_separation_cm"],
                             "warnings": newplan["warnings"]})
        for e in events:
            self.log("t=%.1f %s %s: %s" % (self.t, e["robot"], e["action"], e["reason"]))
        self.log("t=%.1f new plan computed in %.2f s (budget %.1f s)" % (self.t, cpu, self.s.replan_budget_s))
        if ob:
            self.m["obstacles"].append(FM._norm_obstacle(ob, len(self.m["obstacles"])))
            self.m = FM.normalize(self.m)
        if ob is None and getattr(self, "_improving", False):
            # an improvement try: keep it only if everyone is done sooner
            old_end = self.plan_start + self.plan["summary"]["makespan_s"]
            new_end = self.t + newplan["summary"]["makespan_s"]
            # (a plan that leaves someone without a route only looks quicker)
            self.replans[-1]["kept"] = new_end < old_end - 1.0 and not newplan.get("stranded")
            self.log("t=%.1f improvement try: done at %.1f s instead of %.1f s, %s"
                     % (self.t, new_end, old_end, "kept" if self.replans[-1]["kept"] else "not kept"))
            if not self.replans[-1]["kept"]:
                return
        self.plan = newplan
        self.plan_start = self.t
        self.next_retry = self.t + self.sp["retry_s"]
        if ob is not None and self.sp["improve_after_s"] > 0:
            self.improve_at = self.t + self.s.replan_budget_s + self.sp["improve_after_s"]
        acts = {e["robot"]: e["action"] for e in events}
        for n, r in self.robots.items():
            r.done_steps = set()
            steps = self.steps(r)
            r.finished_at = None
            if not steps:
                r.state, r.step = "done", 0
                r.finished_at = r.finished_at or self.t
                continue
            r.step = 0
            if acts.get(n) == "continues" and r.state == "run" and steps[0].get("committed"):
                # the step under way carries on (now as the new plan's step 0)
                self.start_step(r, steps[0])
            else:
                if r.state in ("run", "delay"):
                    r.stopped_reason = r.stopped_reason or "stopped for the new plan"
                r.state = "idle"

    # -- main loop ------------------------------------------------------------
    def log(self, msg):
        if self.verbose:
            print(msg)
        self.events.append(msg)

    def check(self):
        names = list(self.robots)
        poses = np.array([self.robots[n].true for n in names])
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                if math.hypot(*(poses[i, :2] - poses[j, :2])) - 2 * self.body.r_turn >= self.min_gap:
                    continue      # cannot beat the closest so far
                g = float(G.body_gaps(self.body, poses[i:i + 1], poses[j:j + 1])[0])
                if g < self.min_gap:
                    self.min_gap, self.min_gap_at = g, (names[i], names[j], round(self.t, 2))
        # static: robot bodies against the brick and everything except a board it is docking at
        for n in names:
            r = self.robots[n]
            excl = ()
            steps = self.steps(r)
            near = [r.run.get("st")] if r.state in ("run", "settle") else []
            near += [steps[k] for k in (r.step - 1, r.step) if 0 <= k < len(steps)]
            for st in near:
                if st is not None and st.get("board") and st["phase"] in ("approach", "dwell", "backout"):
                    excl = (st["board"],)
            for bn, d in self.docks.items():      # sitting on a dock line (e.g. across a plan switch)
                rel = r.true[:2] - np.array(d["dock"][:2])
                nrm = np.array(d["normal"])
                if abs(rel[0] * nrm[1] - rel[1] * nrm[0]) < 3.0 and -1.0 < float(np.dot(rel, nrm)) < self.m["dock"]["predock_cm"] + 1:
                    excl = tuple(set(excl) | {bn})
            g = self.grid
            lb = float(g.clear[g.cell(r.true)]) - 0.71 * g.c - self.body.r_turn
            if self.brick is not None:
                lb = min(lb, float(G.points_to_polygon(r.true[None, :2], self.brick)[0]) - self.body.r_turn)
            if lb >= self.min_static and not excl:
                continue
            excl = tuple(excl)
            corners = self.body.corners([r.true])[0]
            c = self._static_clear(corners, excl)
            if c < self.min_static:
                self.min_static, self.min_static_at = c, (n, round(self.t, 2))

    def _static_clear(self, corners, excl):
        c = float(np.min(self.world_static.edge_dist(corners)))
        for b in self.m["boards"]:
            if b["name"] in excl:
                continue
            c = min(c, G.polygon_polygon_dist(corners, self.boards[b["name"]]))
        if self.brick is not None:
            c = min(c, G.polygon_polygon_dist(corners, self.brick))
        return c

    def run_sim(self):
        dt = self.sp["dt"]
        while self.t < self.sp["timeout_s"]:
            if self.brick_spec and self.brick is None and self.brick_t is None and self.t >= self.brick_spec["t"]:
                self.place_brick()
                if self.brick is None:
                    self.brick_t = self.t
            if self.pending_detection and self.t >= self.pending_detection[1]:
                self.pending_detection = None
                self.camera_detection()
            if self.improve_at is not None and self.t >= self.improve_at:
                self.improve_at = None
                if not self.plan.get("stranded"):
                    self.improve()
            if self.plan.get("stranded") and self.t >= self.next_retry:
                self.next_retry = self.t + self.sp["retry_s"]
                if len(self.replans) < self.sp["max_replans"]:
                    self.retries += 1
                    budget = min(12.0, self.s.replan_budget_s * 3 ** self.retries)
                    self.log("t=%.1f retrying a plan for %s (%.0f s allowed)"
                             % (self.t, ", ".join(self.plan["stranded"]), budget))
                    self.with_budget(budget, lambda: self.handle_obstacle(None))
            all_done = True
            for r in self.robots.values():
                r.tick_estimate(dt)
                steps = self.steps(r)
                if r.state == "done":
                    continue
                all_done = False
                if r.state == "settle":
                    if self.t >= r.t_ready:
                        r.state = "done" if r.step >= len(steps) else "idle"
                    continue
                if r.state == "idle":
                    if r.step >= len(steps):
                        r.state = "done"
                        r.finished_at = r.finished_at or self.t
                        continue
                    st = steps[r.step]
                    if self.can_start(r, st):
                        lat = self.sp["latency_s"] * r.rng.random()
                        if r.rng.random() < self.sp["stall_prob"]:
                            lat += self.sp["stall_s"] * r.rng.random()
                        if st["type"] == "wait" and st["phase"] != "dwell" and self.sp["gate"] == "deps":
                            lat = 0.0
                        r.state, r.t_ready = "delay", self.t + lat
                    continue
                if r.state == "delay":
                    if self.t >= r.t_ready:
                        self.start_step(r, steps[r.step])
                    else:
                        continue
                if r.state == "run":
                    self.step_motion(r, dt)
            self.check()
            if all_done and not self.plan.get("stranded"):
                break
            if all_done and len(self.replans) >= self.sp["max_replans"]:
                break       # out of re-plans with robots still waiting: counted as not finished
            self.t += dt
        return self.result()

    def result(self):
        waiting = set(self.plan.get("stranded") or [])
        fin = {n: (round(r.finished_at, 2) if r.finished_at is not None and n not in waiting else None)
               for n, r in self.robots.items()}
        dock_ok = all(d["ok"] for d in self.docking)
        gaps = [d["gap_error_cm"] for d in self.docking]
        lats = [abs(d["lateral_cm"]) for d in self.docking]
        hes = [abs(d["heading_err_deg"]) for d in self.docking]
        return {"finished": all(v is not None for v in fin.values()), "finish_s": fin,
                "makespan_s": max([v for v in fin.values() if v is not None] + [0.0]),
                "planned_makespan_s": None,
                "dockings": len(self.docking), "docking_all_ok": dock_ok,
                "dock_gap_error_cm_max": round(max(map(abs, gaps)), 3) if gaps else None,
                "dock_lateral_cm_max": round(max(lats), 3) if lats else None,
                "dock_heading_deg_max": round(max(hes), 2) if hes else None,
                "min_body_gap_cm": round(self.min_gap, 2), "min_body_gap_at": self.min_gap_at,
                "min_static_clearance_cm": round(self.min_static, 2), "min_static_at": self.min_static_at,
                "collision": self.min_gap <= 0.0 or self.min_static <= 0.0,
                "replans": self.replans, "dock_details": self.docking}


def parse_brick(s):
    if not s:
        return None
    where, _, t = s.partition("@")
    t = float(t or 10.0)
    if where == "auto":
        return {"where": "auto", "t": t}
    x, y, *a = [float(v) for v in where.split(",")]
    return {"where": (x, y, a[0] if a else 0.0), "t": t}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True)
    ap.add_argument("--plan", required=True)
    ap.add_argument("--brick", help="auto@T or x,y[,angle_deg]@T: a brick appears at T seconds")
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", help="write the results as JSON here")
    for k, v in DEFAULT_SIM.items():
        if k in ("dt",):
            continue
        ap.add_argument("--" + k.replace("_", "-"), type=type(v), default=v)
    args = ap.parse_args()
    m = FM.load(args.map)
    with open(args.plan, encoding="utf-8") as f:
        plan = json.load(f)
    if plan.get("map_hash") != FM.map_hash(m):
        print("Warning: plan.json was made for a different map (hash %s, map is %s). Plan again."
              % (plan.get("map_hash"), FM.map_hash(m)))
    sp = dict(DEFAULT_SIM)
    for k in DEFAULT_SIM:
        if hasattr(args, k):
            sp[k] = getattr(args, k)
    results = []
    for k in range(args.runs):
        sim = Sim(m, copy.deepcopy(plan), sp, seed=args.seed + k, brick=parse_brick(args.brick),
                  verbose=args.runs == 1)
        res = sim.run_sim()
        res["planned_makespan_s"] = plan["summary"]["makespan_s"]
        results.append(res)
    print_summary(results)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(results if args.runs > 1 else results[0], f, indent=1)
    bad = [r for r in results if not r["finished"] or r["collision"] or not r["docking_all_ok"]]
    return 1 if bad else 0


def print_summary(results):
    n = len(results)
    fin = sum(r["finished"] for r in results)
    col = sum(r["collision"] for r in results)
    dok = sum(r["docking_all_ok"] for r in results)
    print("\nRuns: %d   all robots finished: %d   collisions: %d   every docking passed: %d" % (n, fin, col, dok))
    gaps = [r["min_body_gap_cm"] for r in results]
    print("Closest two robots came (bodies): %.2f cm (worst run), %.2f cm (median)" % (min(gaps), float(np.median(gaps))))
    st = [r["min_static_clearance_cm"] for r in results]
    print("Closest a robot came to anything else (not counting the board it docks at): %.2f cm" % min(st))
    dg = [abs(d["gap_error_cm"]) for r in results for d in r["dock_details"]]
    if dg:
        lat = [abs(d["lateral_cm"]) for r in results for d in r["dock_details"]]
        he = [abs(d["heading_err_deg"]) for r in results for d in r["dock_details"]]
        print("Docking (%d stops): front gap error max %.3f cm, sideways max %.2f cm, heading max %.2f deg"
              % (len(dg), max(dg), max(lat), max(he)))
    ms = [r["makespan_s"] for r in results]
    print("Time until the last robot finished: %.1f s (plan said %.1f s)" % (float(np.median(ms)), results[0]["planned_makespan_s"]))
    rp = [x for r in results for x in r["replans"]]
    if rp:
        ev = [x["cpu_s"] for x in rp if x.get("source") not in ("retry", None) and "cpu_s" in x]
        rt = [x["cpu_s"] for x in rp if x.get("source") == "retry" and "cpu_s" in x]
        print("Re-plans after an obstacle: %d, computing time max %.2f s (allowed %.1f s); retries and "
              "improvement tries: %d, max %.2f s (allowed 6-12 s)"
              % (len(ev), max(ev) if ev else float("nan"), rp[0].get("budget_s", float("nan")), len(rt),
                 max(rt) if rt else float("nan")))
        stranded = sum(1 for r in results for x in r["replans"] if x.get("warnings") and any("stays where" in w for w in x["warnings"]))
        if stranded:
            print("Re-plans that left a robot waiting for a route (fixed by a later retry): %d" % stranded)


if __name__ == "__main__":
    sys.exit(main())
