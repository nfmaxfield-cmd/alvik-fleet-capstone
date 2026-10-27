#!/usr/bin/env python3
"""Live fleet simulator: simulated Alviks running the home_lab firmware's
follow and survey code (robot.py + fw_follow.py), in real time, on real UDP,
so tools/fleet/run_plan.py can drive them exactly as it drives the real robots
(through tools/alvik_console.py's Link).

    python tools/sim/fleet_sim.py --map tools/fleet/examples/demo_map.json
    python tools/fleet/run_plan.py --map ... --dry-run      # starts this for you

Each robot listens on its own loopback address (127.0.0.2, 127.0.0.3, ...,
port 5006, in map order) and sends its log lines to 127.0.0.1:5005, like a
robot on the WiFi. Every robot starts on its map start spot (plus a small
placement error), already calibrated (compass sign known) unless --uncalibrated.

Also simulated:
  camera fixes   the camera's view of each robot (true pose + noise), --fix-hz a
                 second, sent --fix-delay s late as JSON lines to --fixes-to
                 (run_plan.py's position source port), stamped with the time
                 the frame was taken;
  front sensor   sees the room walls, the map's boards and obstacles, a dropped
                 brick and (--tof-robots 1, default) the other robots;
  --brick        a 20 x 9 cm brick appears at T s after the first step arrives
                 (auto: on a robot's planned path about 3 s ahead of it; or
                 x,y[,angle]); the camera reports it --cam-delay s later;
  --wifi-drop    NAME@T:DUR: every packet to and from that robot is lost for
                 DUR s from T s after the first step arrives (repeatable).

Writes to --out: truth.csv (true poses, 20 a second), sim_report.json (dockings
measured on the true poses, closest two robots came, contact with boards or the
brick, the events above) and sim_log.txt. Ends when every robot has run a follow
run and stopped, or after --max-s.
"""
import argparse
import csv
import json
import math
import os
import random
import socket
import sys
import threading
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
FLEET = os.path.join(REPO, "tools", "fleet")
for p in (HERE, FLEET):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402

import config  # noqa: E402
import robot as R  # noqa: E402
import fleetmap as FM  # noqa: E402
import geometry as G  # noqa: E402

LOG_PORT, CMD_PORT = 5005, 5006
BRICK = (20.0, 9.0)


def brick_poly(cx, cy, ang):
    c, s = math.cos(ang), math.sin(ang)
    hx, hy = BRICK[0] / 2, BRICK[1] / 2
    return np.array([[cx + c * x - s * y, cy + s * x + c * y] for x, y in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))])


class Radio:
    """One robot's WiFi: commands in (with a small delay), log lines out, both lost during a drop."""

    def __init__(self, sim, name, ip, laptop):
        self.sim, self.name, self.ip, self.laptop = sim, name, ip, laptop
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, CMD_PORT))
        self.sock.settimeout(0.2)
        self.inbox = deque()
        self.lock = threading.Lock()
        self.seq = 0
        self.got_cmd = False
        self.lost_in = self.lost_out = 0
        self.rng = random.Random(hash(name) & 0xFFFF)
        threading.Thread(target=self._rx, daemon=True).start()

    def _rx(self):
        while not self.sim.stop.is_set():
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            now = time.time()
            md = getattr(self.sim, "drop_match", None)
            if self.sim.dropped(self.name, now) or self._random_drop("in") or \
                    (md is not None and md(self.name, data.decode("utf-8", "replace"))):
                self.lost_in += 1
                continue
            delay = (1.5 + self.rng.expovariate(1 / 3.0)) / 1000.0
            with self.lock:
                self.inbox.append((now + delay, data.decode("utf-8", "replace").strip()))

    ACK_KEEP = 64      # net.h ACK_KEEP

    def _random_drop(self, direction):
        """A packet lost at random (the e02 simulator's --drop-in-pct / --drop-out-pct: the 2026-10-25 21:14 link
        lost about 1 command in 10). The simulation decides the share (drop_pct(direction) in %); none by default."""
        f = getattr(self.sim, "drop_pct", None)
        if f is None:
            return False
        pct = f(direction)
        if pct <= 0:
            return False
        rng = self.__dict__.setdefault("drop_rng", random.Random((hash(self.name) & 0xFFFF) + 7))
        return rng.random() * 100.0 < pct

    def poll(self, rb):
        """net::poll(): one command, if one has arrived by the robot's current time. Firmware 2026-10-26a: a
        numbered command "#<n> cmd" is answered at once (D,ACK,n,new|dup,ms) and a number seen among the last
        ACK_KEEP is not run again."""
        t = rb.t_at(rb.ms)
        with self.lock:
            if not (self.inbox and self.inbox[0][0] <= t):
                return None
            self.got_cmd = True
            cmd = self.inbox.popleft()[1]
        if cmd.startswith("#") and getattr(rb, "fw_version", "") >= "2026-10-26a":
            head, _, rest = cmd.partition(" ")
            try:
                n = int(head[1:])
            except ValueError:
                return None
            seen = self.__dict__.setdefault("ack_seen", deque(maxlen=self.ACK_KEEP))
            cnt = self.__dict__.get("ack_count", 0)
            if cnt % 50 == 0:
                rb.send("H,ACK,n,result,ms")
            self.ack_count = cnt + 1
            dup = n in seen
            rb.send("D,ACK,%d,%s,%d" % (n, "dup" if dup else "new", rb.millis()))
            if dup:
                return None
            seen.append(n)
            cmd = rest.strip()
        return cmd or None

    def transmit(self, rb, text):
        payload = "%s|%d|%s" % (self.name, self.seq, text)
        self.seq += 1
        if self.sim.dropped(self.name, time.time()) or self._random_drop("out"):
            self.lost_out += 1
            return
        try:
            self.sock.sendto(payload.encode(), (self.laptop, LOG_PORT))
        except OSError:
            pass


class LiveRobot(R.Robot):
    """A simulated robot whose clock runs with the wall clock."""

    def advance(self, dt_ms, busy=False):
        super().advance(dt_ms, busy)
        lag = self.t_at(self.ms) - time.time()
        if lag > 0.001:
            time.sleep(lag)

    def firmware_loop(self, sim):
        """home_lab.ino loop(): read commands, start what was asked for, announce ourselves."""
        last_hello = -1e9
        while not sim.stop.is_set():
            cmd = self.radio.poll(self)
            if cmd:
                self.handle_command(cmd, False)
            if self.pending >= 0:
                i = self.pending
                self.pending = -1
                run = self.run_experiment(R.EXP_NAMES[i])
                if run is not None and run["exp"] == "follow":
                    sim.follow_done(self)
                continue
            if not self.radio.got_cmd and time.time() - last_hello > 2.0:
                last_hello = time.time()
                self.send("M,hello from %s at %s (type help in the console)" % (self.name, self.ip))
            self.advance(10)


class FleetSim:
    def __init__(self, args):
        self.args = args
        self.stop = threading.Event()
        self.m = FM.normalize(FM.load(args.map))
        self.plan = json.load(open(args.plan)) if args.plan else None
        self.body = FM.body_of(self.m)
        self.cfg = config.load_config(args.config)
        self.rng = random.Random(args.seed)
        self.t_plan0 = None          # laptop time the first step arrived at any robot
        self.drops = []              # (name, t0, t1) laptop time, filled once the plan starts
        self.brick = None
        self.brick_t = None
        self.events = []
        self.min_gap = (math.inf, None)
        self.min_static = (math.inf, None)
        self.min_open = (math.inf, None)     # closest a body came to an open part of the edge (school workspace)
        self.hung = []
        self.followed = set()
        self.log_lines = []
        self.lock = threading.Lock()
        # what the front sensors see
        self.static_polys = [G.segment_rect(b["p1"], b["p2"], b["thickness_cm"]).tolist() for b in self.m["boards"]]
        for o in self.m["obstacles"]:
            if o["shape"] == "polygon":
                self.static_polys.append([list(p) for p in o["points"]])
            else:
                a = np.linspace(0, 2 * np.pi, 16, endpoint=False)
                c, r = o["center"], o["radius"]
                self.static_polys.append([[c[0] + r * math.cos(t), c[1] + r * math.sin(t)] for t in a])
        # the table's real sides (school workspace, 2026-10-27): --true-sides, else the map's own. Open parts: the
        # plywood ends there (edge guard, hang model); barrier parts: a solid 1.5 cm strip just outside the line
        # that the front sensors see
        self.true_sides = None
        ts = getattr(args, "true_sides", None)
        if ts or self.m.get("sides"):
            mt = dict(self.m)
            if ts:
                mt["sides"] = parse_sides(ts)
            mt = FM.normalize(mt)
            self.true_sides = mt["sides"]
            f = mt["floor"]
            short = {"x_min": "x0", "x_max": "x1", "y_min": "y0", "y_max": "y1"}
            self.cfg.session.board_cm = [f["x_min"], f["y_min"], f["x_max"], f["y_max"]]
            self.cfg.session.open_edges = []
            self.cfg.session.open_spans = {}
            for sd in FM.SIDES:
                bar, opn = FM.side_parts(mt, sd)
                if opn:
                    self.cfg.session.open_edges.append(short[sd])
                    if bar:
                        self.cfg.session.open_spans[short[sd]] = opn
                for a, b in bar:
                    v = mt["floor"][sd]
                    out = 1.5 if sd.endswith("max") else -1.5
                    if sd.startswith("x"):
                        poly = [[v, a], [v + out, a], [v + out, b], [v, b]]
                    else:
                        poly = [[a, v], [b, v], [b, v + out], [a, v + out]]
                    self.static_polys.append(poly)
            for o in getattr(args, "true_objects", None) or []:       # things on the table the map does not know
                self.static_polys.append(o)
        if float(self.m["robot"].get("back_bumper_cm", 0.0) or 0.0) > 0:
            # switches at the back (map robot.back_bumper_cm): board visits back in until a switch clicks on the board
            # (follow step P), so the boards must be solid for the switches (fw_dock, as in the touch tests)
            self.cfg.dock.contact_boards = True
        if self.m.get("home_docks"):          # home docks (hardware/dock): solid, with their floor stripe
            self.cfg.dock.docks = [[d["x"], d["y"], d["heading"]] for d in self.m["home_docks"]]
        params = config.firmware_params()
        fw = config.firmware_version()
        now = time.time()
        self.robots = {}
        for k, rr in enumerate(self.m["robots"]):
            name = rr["name"]
            ip = "127.0.0.%d" % (2 + k)
            t_boot = now - self.rng.uniform(60, 600)
            rb = LiveRobot(name, self.cfg, random.Random(self.rng.random()), params, fw, t_boot,
                           self.rng.randrange(0x1000, 0x10000), ip)
            rb.idle_until(now)
            e = args.start_noise
            rb.x = rr["start"][0] + self.rng.uniform(-e, e)
            rb.y = rr["start"][1] + self.rng.uniform(-e, e)
            rb.th = math.radians(rr["start"][2] + self.rng.uniform(-3 * e, 3 * e))
            rb._record()
            rb.fw_yaw_sign = 0 if args.uncalibrated else 1
            rb.world_polys = self.static_polys       # shared list: a dropped brick shows up for everyone
            if self.true_sides:
                rb.hang_cm = 0.0                     # over an open part of the table's edge: it hangs there
            rb.radio = Radio(self, name, ip, args.laptop)
            if args.tof_robots:
                rb.others = (lambda me: (lambda: [self.outline(o) for o in self.robots.values() if o is not me]))(rb)
            self.robots[name] = rb
        self.fix_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        host, _, port = args.fixes_to.partition(":")
        self.fix_dest = (host, int(port or 5010))

    def log(self, msg):
        line = "%.2f %s" % (time.time() - self.t_start, msg)
        self.log_lines.append(line)
        if not self.args.quiet:
            print("[sim] " + line, flush=True)

    def outline(self, rb):
        return self.body.corners([[rb.x, rb.y, rb.th]])[0].tolist()

    def dropped(self, name, t):
        return any(n == name and a <= t < b for n, a, b in self.drops)

    def follow_done(self, rb):
        with self.lock:
            self.followed.add(rb.name)

    # ---------------------------------------------------------------- threads
    def camera_loop(self):
        a = self.args
        sent = deque()
        period = 1.0 / a.fix_hz if a.fix_hz > 0 else None
        nxt = time.time()
        rng = random.Random(self.args.seed + 7)
        while not self.stop.is_set():
            now = time.time()
            if period and now >= nxt:
                nxt += period
                for rb in self.robots.values():
                    if rb.lifted or self.cam_gap(rb.name, now):
                        continue
                    msg = {"type": "fix", "robot": rb.name, "t": now,
                           "x": rb.x + rng.gauss(0, a.fix_sigma), "y": rb.y + rng.gauss(0, a.fix_sigma),
                           "th": math.degrees(rb.th) + rng.gauss(0, a.fix_sigma_deg),
                           "sigma": a.fix_sigma, "sigma_th": a.fix_sigma_deg}
                    sent.append((now + a.fix_delay, msg))
            while sent and sent[0][0] <= now:
                self.send_json(sent.popleft()[1])
            self.events_due(now)
            time.sleep(0.005)

    HAND_BEFORE_S, HAND_AFTER_S = 1.2, 0.6      # the hand is in the picture this long before / after the brick lands

    def picture_loop(self):
        """--brick-camera: the overhead camera's pictures of the table (tools/sim/table_cam.py) at the brick watch's
        rate, through the real "what changed" watch (tools/fleet/brickwatch.py): a hand reaches in from off the table
        and puts the brick down; the watch reports the brick to run_plan as the camera would. Pictures where it reported
        are saved (brick_seen.png) for the build report."""
        import cv2
        for p in (os.path.join(REPO, "tools", "experiment"), os.path.join(REPO, "tools", "recognition")):
            if p not in sys.path:
                sys.path.append(p)          # after the simulator's own modules (config.py, camera.py)
        import table_cam as TC
        import brickwatch as BW
        f = self.m["floor"]
        rect = (f["x_min"], f["y_min"], f["x_max"], f["y_max"])
        tc = TC.TableCamera(rect, seed=self.args.seed + 11)
        import e02_camera as CAM
        cam = CAM.load_camera(tc.true_model(os.path.join(self.args.out, "table_camera.json")))
        rng = random.Random(self.args.seed + 13)

        def spots():
            return [(rb.x + rng.gauss(0, 0.5), rb.y + rng.gauss(0, 0.5)) for rb in self.robots.values()]

        def poses():
            return [(rb.x, rb.y, rb.th) for rb in self.robots.values()]

        def send(ob):
            msg = {k: v for k, v in ob.items() if k != "area_cm2"}
            self.send_json(msg)
            self.event(dict(kind="camera_reported_brick", t=round(time.time() - self.t_plan0, 2), id=ob["id"],
                            how="brick watch", area_cm2=ob.get("area_cm2")))
        w = BW.BrickWatch(cam, rect, send=send, log=self.log)
        while not self.stop.is_set() and self.t_plan0 is None:
            time.sleep(0.05)
        w.start([tc.render(poses()) for _ in range(5)], spots())
        spec = self.args.brick
        hand_from = None
        saved = False
        while not self.stop.is_set():
            now = time.time()
            boxes, hand = [], None
            t_rel = now - self.t_plan0
            if spec and self.brick is None and spec["t"] - self.HAND_BEFORE_S <= t_rel and self.brick_t is None:
                # the hand comes in towards where the brick will land (placed at spec t); before then aim at the middle
                hand = ((f["x_min"] + f["x_max"]) / 2, (f["y_min"] + f["y_max"]) / 2)
            if self.brick is not None:
                boxes = [self.brick.tolist()]
                if now - self.brick_t <= self.HAND_AFTER_S:
                    c = self.brick.mean(0)
                    hand = (float(c[0]), float(c[1]))
            if hand is not None:
                hand_from = (hand[0], f["y_max"] + 60.0)          # the arm comes from the far side, off the table
                img = tc.render(poses(), boxes, (hand, hand_from))
            else:
                img = tc.render(poses(), boxes)
            got = w.check(img, spots(), now)
            if got and not saved:
                cv2.imwrite(os.path.join(self.args.out, "brick_seen.png"), img)
                saved = True
            if hand is not None and not getattr(self, "_hand_png", False) and self.brick is not None:
                cv2.imwrite(os.path.join(self.args.out, "brick_hand.png"), img)
                self._hand_png = True
            time.sleep(max(0.0, 1.0 / BW.CHECK_HZ - (time.time() - now)))
        self.event(dict(kind="brick_watch", checks=w.checks, hand_checks=w.hand_seen, objects=len(w.confirmed)))

    def cam_gap(self, name, now):
        """--cam-gap NAME@T:DUR: the camera does not see that robot for DUR s from T s after the plan starts."""
        if self.t_plan0 is None:
            return False
        for g in getattr(self.args, "cam_gap", None) or []:
            n, _, rest = g.partition("@")
            t0, _, dur = rest.partition(":")
            if n == name and float(t0) <= now - self.t_plan0 < float(t0) + float(dur):
                return True
        return False

    def send_json(self, msg):
        try:
            self.fix_sock.sendto(json.dumps(msg).encode(), self.fix_dest)
        except OSError:
            pass

    def events_due(self, now):
        if self.t_plan0 is None:
            if any(rb.f_max_id > 0 for rb in self.robots.values()):
                self.t_plan0 = now
                self.log("first step arrived: plan clock starts")
                for spec in self.args.wifi_drop:
                    name, _, rest = spec.partition("@")
                    t, _, dur = rest.partition(":")
                    self.drops.append((name, now + float(t), now + float(t) + float(dur or 3.0)))
                    self.log("WiFi drop planned for %s at %.1f s for %.1f s" % (name, float(t), float(dur or 3.0)))
            return
        for name, t0, t1 in self.drops:
            key = ("drop", name, t0)
            if t0 <= now and key not in self.events_seen():
                self.event(dict(kind="wifi_drop_start", robot=name, t=round(now - self.t_plan0, 2), key=key))
            key2 = ("dropend", name, t0)
            if t1 <= now and key2 not in self.events_seen():
                self.event(dict(kind="wifi_drop_end", robot=name, t=round(now - self.t_plan0, 2), key=key2))
        spec = self.args.brick
        if spec and self.brick is None and self.brick_t is None and now - self.t_plan0 >= spec["t"]:
            self.brick_t = now
            poly = self.place_brick(spec)
            if poly is None:
                self.log("no good spot for the brick; none placed")
                return
            self.brick = poly
            self.static_polys.append(poly.tolist())
            c = poly.mean(0)
            self.event(dict(kind="brick", t=round(now - self.t_plan0, 2), center=[round(c[0], 1), round(c[1], 1)],
                            points=poly.round(2).tolist()))
            if not getattr(self.args, "brick_camera", False):
                self.cam_report_at = now + self.args.cam_delay
            # (--brick-camera: the picture loop below shows the hand and the brick, and the brick watch reports it)
        if self.brick is not None and getattr(self, "cam_report_at", None) and now >= self.cam_report_at:
            self.cam_report_at = None
            rng = np.random.default_rng(self.args.seed)
            poly = self.brick + 0.5 * rng.standard_normal(2)
            c = poly.mean(0)
            poly = c + (poly - c) * (1 + 1.0 / (BRICK[1] / 2))      # about 1 cm bigger all round
            self.send_json({"type": "obstacle", "id": "brick-1", "shape": "polygon", "points": poly.round(2).tolist(),
                            "source": "camera", "t": now})
            self.event(dict(kind="camera_reported_brick", t=round(now - self.t_plan0, 2)))

    def events_seen(self):
        return {tuple(e["key"]) for e in self.events if "key" in e}

    def event(self, e):
        self.events.append(e)
        self.log("event: %s" % {k: v for k, v in e.items() if k not in ("key", "points")})

    def place_brick(self, spec):
        if spec["where"] != "auto":
            x, y, a = spec["where"]
            return brick_poly(x, y, math.radians(a))
        if not self.plan:
            return None
        # a robot's planned straight drive a few seconds ahead of where the plan has it now,
        # away from every robot, the docks and the start/park spots
        t_now = time.time() - self.t_plan0
        for name, rp in self.plan["robots"].items():
            rb = self.robots.get(name)
            if rb is None:
                continue
            for st in rp["steps"]:
                if st["type"] != "drive" or st["phase"] != "travel" or st["t_start"] < t_now + self.args.brick_ahead_s:
                    continue
                a, b = np.array(st["from"][:2]), np.array(st["to"][:2])
                for frac in (0.5, 0.25, 0.75, 1.0):
                    mid = a + frac * (b - a)
                    poly = brick_poly(mid[0], mid[1], math.atan2(b[1] - a[1], b[0] - a[0]) + math.pi / 2)
                    if self.brick_ok(poly, rb):
                        return poly
        return None

    def brick_ok(self, poly, target):
        for o in self.robots.values():
            near = 20.0 if o is target else 8.0
            if G.polygon_polygon_dist(self.body.corners([[o.x, o.y, o.th]])[0], poly) < near:
                return False
        for rr in self.m["robots"]:
            for spot in (rr["start"], rr["park"]):
                if float(G.points_to_polygon(np.array([spot[:2]]), poly)[0]) < 20.0:
                    return False
        for d in FM.docks(self.m).values():
            if float(G.points_to_polygon(np.array([d["predock"][:2], d["dock"][:2]]), poly).min()) < 12.0:
                return False
        return FM.World(self.m).shape_clearance(poly) >= 1.0

    def monitor_loop(self):
        names = list(self.robots)
        boards = {b["name"]: G.segment_rect(b["p1"], b["p2"], b["thickness_cm"]) for b in self.m["boards"]}
        docks = FM.docks(self.m)
        f = open(os.path.join(self.args.out, "truth.csv"), "w", newline="")
        w = csv.writer(f)
        w.writerow(["t", "robot", "x_cm", "y_cm", "heading_deg"])
        nxt = time.time()
        while not self.stop.is_set():
            now = time.time()
            if now < nxt:
                time.sleep(nxt - now)
                continue
            nxt += 0.05
            P = {n: np.array([[r.x, r.y, r.th]]) for n, r in self.robots.items()}
            for n, r in self.robots.items():
                w.writerow(["%.3f" % now, n, "%.3f" % r.x, "%.3f" % r.y, "%.2f" % math.degrees(r.th)])
            if self.t_plan0 is None:
                continue
            for i in range(len(names)):
                for j in range(i + 1, len(names)):
                    g = float(G.body_gaps(self.body, P[names[i]], P[names[j]])[0])
                    if g < self.min_gap[0]:
                        self.min_gap = (g, [names[i], names[j], round(now - self.t_plan0, 2)])
            for n, r in self.robots.items():
                c = self.body.corners(P[n])[0]
                # the board it is docked at (or approaching / backing out) is allowed to be close
                near = [b for b, d in docks.items() if math.hypot(r.x - d["dock"][0], r.y - d["dock"][1]) < 17.0]
                for bn, poly in boards.items():
                    if bn in near:
                        continue
                    g = G.polygon_polygon_dist(c, poly)
                    if g < self.min_static[0]:
                        self.min_static = (g, [n, "board " + bn, round(now - self.t_plan0, 2)])
                if self.brick is not None:
                    g = G.polygon_polygon_dist(c, self.brick)
                    if g < self.min_static[0]:
                        self.min_static = (g, [n, "brick", round(now - self.t_plan0, 2)])
                if self.true_sides:
                    # the body's nearest corner to an open part of the table's edge (negative: over it)
                    g = max(r.past_open_edge(float(px), float(py)) for px, py in c)
                    if -g < self.min_open[0]:
                        self.min_open = (-g, [n, round(now - self.t_plan0, 2)])
                    if getattr(r, "hung", False) and n not in self.hung:
                        self.hung.append(n)
                        self.event(dict(kind="hung", robot=n, t=round(now - self.t_plan0, 2)))
        f.close()

    # ---------------------------------------------------------------- results
    def report(self):
        docks = FM.docks(self.m)
        out = []
        for rb in self.robots.values():
            for d in rb.dock_log:
                x, y, th = rb.settled_pose(d["ms"])
                bn = min(docks, key=lambda b: math.hypot(x - docks[b]["dock"][0], y - docks[b]["dock"][1]))
                dk = docks[bn]
                u = np.array([math.cos(th), math.sin(th)])
                # the face that met the board: the front, or the back attachment's (a backing-in touch step)
                front = np.array([x, y]) + (-self.body.back if d.get("face") == "back" else self.body.front) * u
                n = np.array(dk["normal"])
                gap = float(np.dot(front - np.array(dk["surface"]), n))
                tang = np.array([-n[1], n[0]])
                lat = float(np.dot(np.array([x, y]) - np.array(dk["dock"][:2]), tang))
                herr = math.degrees(G.wrap(th - dk["dock"][2]))
                out.append(dict(robot=rb.name, board=bn, t=round(rb.t_at(d["ms"]) - (self.t_plan0 or 0), 2),
                                front_gap_cm=round(gap, 3), gap_error_cm=round(gap - self.m["dock"]["gap_cm"], 3),
                                lateral_cm=round(lat, 3), heading_err_deg=round(herr, 2), info=d["info"],
                                own_pose=[round(v, 2) for v in d["pose"]]))
        rep = dict(dockings=out, min_body_gap_cm=round(self.min_gap[0], 2), min_body_gap_at=self.min_gap[1],
                   min_static_clearance_cm=round(self.min_static[0], 2), min_static_at=self.min_static[1],
                   collision=self.min_gap[0] <= 0 or self.min_static[0] <= 0,
                   events=[{k: v for k, v in e.items() if k != "key"} for e in self.events],
                   packets_lost={n: dict(to_robot=rb.radio.lost_in, from_robot=rb.radio.lost_out)
                                 for n, rb in self.robots.items()},
                   robot_runs={n: [dict(run=r["run"], exp=r["exp"], result=r["result"]) for r in rb.runs]
                               for n, rb in self.robots.items()},
                   tof_sees_robots=bool(self.args.tof_robots), seed=self.args.seed)
        if self.true_sides:
            rep["true_sides"] = self.true_sides
            rep["min_open_edge_cm"] = None if not math.isfinite(self.min_open[0]) else round(self.min_open[0], 2)
            rep["min_open_edge_at"] = self.min_open[1]
            rep["hung"] = list(self.hung)
        if out:
            ge = [abs(d["gap_error_cm"]) for d in out]
            rep["dock_gap_error_cm_max"] = max(ge)
            rep["dock_lateral_cm_max"] = max(abs(d["lateral_cm"]) for d in out)
            rep["dock_heading_deg_max"] = max(abs(d["heading_err_deg"]) for d in out)
        return rep

    def run(self):
        os.makedirs(self.args.out, exist_ok=True)
        self.t_start = time.time()
        self.log("robots: " + ", ".join("%s at %s" % (n, r.ip) for n, r in self.robots.items()))
        threads = [threading.Thread(target=rb.firmware_loop, args=(self,), daemon=True) for rb in self.robots.values()]
        threads += [threading.Thread(target=self.camera_loop, daemon=True),
                    threading.Thread(target=self.monitor_loop, daemon=True)]
        if getattr(self.args, "brick_camera", False):
            threads.append(threading.Thread(target=self.picture_loop, daemon=True))
        for t in threads:
            t.start()
        try:
            while time.time() - self.t_start < self.args.max_s:
                time.sleep(0.2)
                with self.lock:
                    if len(self.followed) == len(self.robots):
                        break
        except KeyboardInterrupt:
            pass
        time.sleep(0.5)
        self.stop.set()
        time.sleep(0.3)
        rep = self.report()
        with open(os.path.join(self.args.out, "sim_report.json"), "w") as f:
            json.dump(rep, f, indent=1)
        with open(os.path.join(self.args.out, "sim_log.txt"), "w") as f:
            f.write("\n".join(self.log_lines) + "\n")
        self.log("report written to %s" % os.path.join(self.args.out, "sim_report.json"))
        return rep


def parse_sides(spec):
    """"x_min=open,x_max=barrier,y_min=partial:0-60:90-120,y_max=open" -> the map's "sides"."""
    out = {}
    for part in spec.split(","):
        k, _, v = part.partition("=")
        kind, *spans = v.split(":")
        sd = {"kind": kind.strip()}
        if spans:
            sd["barrier"] = [[float(x) for x in sp.split("-")] for sp in spans]
        out[k.strip()] = sd
    return out


def parse_object(spec):
    """"x,y,w,h[,angle]": a box on the table (centre, size cm, angle deg) the map does not know."""
    v = [float(x) for x in spec.split(",")]
    x, y, w, h = v[:4]
    a = math.radians(v[4]) if len(v) > 4 else 0.0
    c, s_ = math.cos(a), math.sin(a)
    return [[x + c * u - s_ * w2, y + s_ * u + c * w2] for u, w2 in ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2),
                                                                     (-w / 2, h / 2))]


def parse_brick(s):
    if not s:
        return None
    where, _, t = s.partition("@")
    t = float(t or 10.0)
    if where == "auto":
        return {"where": "auto", "t": t}
    x, y, *a = [float(v) for v in where.split(",")]
    return {"where": (x, y, a[0] if a else 0.0), "t": t}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True)
    ap.add_argument("--plan", help="plan.json (only needed for --brick auto)")
    ap.add_argument("--out", default="fleet_sim_out")
    ap.add_argument("--config", help="simulator config JSON (simulate.py --write-config)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--laptop", default="127.0.0.1", help="where the robots send their log lines")
    ap.add_argument("--fixes-to", default="127.0.0.1:5010", help="where camera fixes go (run_plan.py --fixes udp)")
    ap.add_argument("--fix-hz", type=float, default=10.0, help="camera fixes per robot per second (0 = none)")
    ap.add_argument("--fix-sigma", type=float, default=0.3, help="camera position noise, cm")
    ap.add_argument("--fix-sigma-deg", type=float, default=1.0, help="camera heading noise, deg")
    ap.add_argument("--fix-delay", type=float, default=0.1, help="camera latency, s")
    ap.add_argument("--start-noise", type=float, default=0.5, help="placement error on the start spots, cm (x3 for deg)")
    ap.add_argument("--uncalibrated", action="store_true", help="robots need a calibrate first")
    ap.add_argument("--tof-robots", type=int, default=1, help="1: the front sensor sees the other robots")
    ap.add_argument("--brick-camera", action="store_true",
                    help="the brick is seen by the simulated overhead camera through the brick watch (a hand puts it "
                         "down), not reported perfectly")
    ap.add_argument("--cam-gap", action="append", default=[], metavar="NAME@T:DUR",
                    help="the camera loses that robot for DUR s from T s after the plan starts (repeatable)")
    ap.add_argument("--true-sides", help="the table's real side kinds (default: the map's), e.g. "
                                         "x_min=open,x_max=barrier,y_min=partial:0-60,y_max=open")
    ap.add_argument("--true-object", dest="true_objects", type=parse_object, action="append", default=[],
                    help="x,y,w,h[,angle]: a box on the table the map does not know (repeatable)")
    ap.add_argument("--brick", type=parse_brick, help="auto@T or x,y[,angle]@T (s after the first step)")
    ap.add_argument("--brick-ahead-s", type=float, default=3.0)
    ap.add_argument("--cam-delay", type=float, default=0.5, help="camera reports a new obstacle after this, s")
    ap.add_argument("--wifi-drop", action="append", default=[], metavar="NAME@T:DUR")
    ap.add_argument("--max-s", type=float, default=600.0)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    sim = FleetSim(args)
    rep = sim.run()
    print(json.dumps({k: v for k, v in rep.items() if k not in ("dockings", "events")}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
