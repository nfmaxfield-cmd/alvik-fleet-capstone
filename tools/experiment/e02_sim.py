#!/usr/bin/env python3
"""E02 dry-run world: simulated Alviks that speak the E02 text protocol over real UDP, a simulated
overhead camera (and a second "side" camera) served exactly like camrec.py's /live, and the Pi's notes
port. e02.py --dry-run starts this for you; it can also run on its own:

    python tools/experiment/e02_sim.py --out sim_out --robots Left,Center,Right

The robots are the firmware lane's simulator model (tools/sim/robot.py with fw_e02.py, a copy of
firmware/home_lab/e02.h), run in real time exactly as tools/sim/fleet_sim.py runs them: so the dry run
talks to the same E02 logic as the real robots (run find, run e02 in both modes, fix, go / turn / mark
legs, BUMP / FIND / E02 / LEG / FIX records). Real-robot imperfections (2026-10-21b defaults, from the real
footage and logs): gyros count 91.5-94.5% of every turn (real: 1.069-1.075 measured, i.e. ~93%); gyro creep
drawn with sd --drift-sd 0.15 deg/s (real Center: 0.23); wheel radius +-3% (real 0.979) and --slip 1% tyre slip
(more while speeding up); camera: sensor noise --cam-noise 6 grey levels, JPEG 80 (--cam-jitter shakes the whole picture: off by
default, the real mount is rigid and a shaking background breaks the motion tracker) (real moving-robot tracking: median ~1 px, p90 ~4-5 px); an occasional missed bump
(--miss-bump 0.03: the find runs on and fails safely; --miss-jolt 0.10: found by the stall alone, later).
The floor reads like the real plywood to the line sensors (52, sd 4; past the edge: the simulator's guess).
They start just switched on (they need `calibrate`), near their park spots, roughly facing the middle.
Bricks (the brick demo, --bricks "x,y,length,width[,deg];..."): boxes --brick-height cm tall, centred at (x, y),
put on the board while the file <out>/BRICKS_ON exists (e02.py makes it when it asks Nick to put the bricks down,
and removes it when it asks him to take them off): drawn in the camera picture, solid for the robots' bodies and
seen by their front distance sensor.

What runs:
  robots   127.0.0.2, .3, .4 ... port 5006 (map order), log lines to 127.0.0.1:5005, "name|seq|line"
  camera   http://127.0.0.1:8000/live (overhead, rendered with tools/sim/camera.py's Renderer and a
           real-footage-like camera: 2.34 px/cm (height a placeholder), origin at the bottom-right of the picture)
           and :8001/live (the "side" camera: the same picture, it only has to answer)
  notes    UDP 127.0.0.1:5007, answers "ok <text>" and writes notes.csv
Writes --out: truth.csv (true poses 20 a second), notes.csv, sim_info.json (each robot's true gyro
scale, drift and wheel scale: what the self-calibration should find), camera_model.json (the true
camera, as a studio camera file).
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
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)

import e02_plan as PL  # noqa: E402

LOG_PORT, CMD_PORT, NOTES_PORT = 5005, 5006, 5007
HALF = PL.ROBOT_HALF_CM


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def parse_bricks(spec):
    """'x,y,length,width[,deg];...' (cm, deg; centre and size along / across its long side) -> [dict]."""
    out = []
    for part in (spec or "").split(";"):
        v = [float(t) for t in part.replace(" ", "").split(",") if t != ""]
        if not v:
            continue
        if len(v) not in (4, 5):
            raise ValueError("a brick is x,y,length,width[,deg]: %r" % part)
        out.append(dict(x=v[0], y=v[1], length=v[2], width=v[3], deg=v[4] if len(v) == 5 else 0.0))
    return out


def brick_corners(b):
    """A brick's 4 floor corners [[x, y] ...]."""
    c, s_ = math.cos(math.radians(b["deg"])), math.sin(math.radians(b["deg"]))
    hl, hw = b["length"] / 2.0, b["width"] / 2.0
    return [[b["x"] + c * u - s_ * v, b["y"] + s_ * u + c * v] for u, v in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]


# =============================================================================== the robots (firmware model)
def add_identify(rb, R):
    """home_lab.ino's `identify` (the simulator's firmware model has none): both LEDs white 250 ms on / 250 ms off,
    6 times, blocking the robot for those 3 s; refused during a run. The dance flashes the lights with it (2026-10-22)."""
    orig = rb.handle_command

    def handle(raw, running):
        verb = raw.strip().split(" ")[0].lower()
        if verb == "led":                       # firmware 2026-10-22a: steady colour for ms, not blocking
            rb.last_rx = rb.millis()
            if running:
                rb.send("M,busy: send stop first")
                return False
            parts = (raw.split() + ["1", "1", "1", "2000"])[1:5]
            on = any(int(float(v)) for v in parts[:3])
            rb.set_leds(R.WHITE if on else R.OFF, R.WHITE if on else R.OFF)
            threading.Timer(max(0.05, float(parts[3]) / 1000.0), lambda: rb.set_leds(R.OFF, R.OFF)).start()
            rb.send("M,%s: led for %s ms" % (rb.name, parts[3]))
            return False
        if verb != "identify":
            return orig(raw, running)
        rb.last_rx = rb.millis()
        if running:
            rb.send("M,busy: send stop first")
            return False
        for _ in range(6):
            rb.set_leds(R.WHITE, R.WHITE)
            rb.advance(250)
            rb.set_leds(R.OFF, R.OFF)
            rb.advance(250)
        rb.send("M,%s: that was me" % rb.name)
        return False
    rb.handle_command = handle


class World:
    """The simulated robots: tools/sim/robot.py (with the firmware lane's fw_e02.py: run find, run e02, fix,
    legs, marks) running in real time on real UDP, as tools/sim/fleet_sim.py runs them, on a 122 x 122 cm
    plywood board with two blocked edges."""

    def __init__(self, args):
        import config
        import fleet_sim as FS
        import robot as R
        self.args = args
        self.stop = threading.Event()
        self.laptop = "127.0.0.1"
        bd = PL.Board(xboard_side=args.xboard_side, yboard_side=args.yboard_side)
        # where the boards really stand (their faces): a little inside the plywood edge, as on site
        bd.xboard_cm = (bd.y0 + args.board_inset) if bd.xboard_side == 0 else (bd.y1 - args.board_inset)
        bd.yboard_cm = (bd.x0 + args.board_inset) if bd.yboard_side == 0 else (bd.x1 - args.board_inset)
        self.board = bd
        cfg = config.load_config(None)
        cfg.dock.contact_boards = True
        cfg.dock.wall_stall_frac = args.wall_stall   # real (2026-10-22): the tyres slip at ~96% of speed
        cfg.dock.miss_bump_prob = args.miss_bump
        cfg.dock.miss_jolt_prob = args.miss_jolt
        cfg.imu.gyro_bias_dps = args.drift_sd
        cfg.imu.accel_vibration_g = 0.05                 # measured while creeping (roam1)
        cfg.robot.radius_scale_err_pct = 3.0
        cfg.robot.move_slip_mean_pct = args.slip
        cfg.robot.move_slip_sd_pct = 0.3 * args.slip
        cfg.robot.accel_slip_pct_per_g = 4.0
        cfg.line.floor_reading, cfg.line.floor_noise = 52.0, 4.0     # real plywood (roam1 RAW lineL/C/R)
        cfg.session.board_cm = [bd.x0, bd.y0, bd.x1, bd.y1]
        cfg.session.open_edges = ["x1" if bd.yboard_side == 0 else "x0", "y1" if bd.xboard_side == 0 else "y0"]
        self.cfg = cfg
        T = 3.0
        xb, yb = bd.xboard_cm, bd.yboard_cm
        oy, ox = -bd.uy, -bd.ux                     # outwards from each face
        xpoly = [[bd.x0 - 5, xb], [bd.x1 + 5, xb], [bd.x1 + 5, xb + oy * T], [bd.x0 - 5, xb + oy * T]]
        ypoly = [[yb, bd.y0 - 5], [yb, bd.y1 + 5], [yb + ox * T, bd.y1 + 5], [yb + ox * T, bd.y0 - 5]]
        def ccw(poly):     # the contact model wants the corners counter-clockwise
            a = sum(p_[0] * q[1] - q[0] * p_[1] for p_, q in zip(poly, poly[1:] + poly[:1]))
            return poly if a > 0 else poly[::-1]
        self.polys = [ccw(xpoly), ccw(ypoly)]
        self.brick_specs = parse_bricks(getattr(args, "bricks", "") or "")
        self.brick_polys = [ccw(brick_corners(b)) for b in self.brick_specs]
        self.brick_h = float(getattr(args, "brick_height", 6.0))
        self.bricks_on = False
        params = config.firmware_params()
        fw = config.firmware_version()
        rng = random.Random(args.seed)
        now = time.time()
        self.robots = []
        names = [n for n in args.robots.split(",") if n]
        for k, n in enumerate(names):
            ip = "127.0.0.%d" % (2 + k)
            rb = FS.LiveRobot(n, cfg, random.Random(rng.random()), params, fw, now - rng.uniform(60, 600),
                              rng.randrange(0x1000, 0x10000), ip)
            rb.idle_until(now)
            if args.start == "swapped":         # each on the next one's park spot (all spots taken: 2026-10-21)
                px, py, ph = bd.park((k + 1) % len(names))
            elif args.start == "corners":       # Nick's 2026-10-22 08:09 layout (camera frame): it dead-locked parking
                px, py, ph = [(87.1, 27.8, 125.0), (35.2, 86.1, -50.0), (88.2, 87.3, 45.0)][k % 3]
            elif args.start == "scatter":       # anywhere open, as Nick puts them down now
                while True:
                    px, py, ph = rng.uniform(30, 92), rng.uniform(30, 92), rng.uniform(-180, 180)
                    if all(math.hypot(px - o.x, py - o.y) > 30 for o in self.robots):
                        break
            else:
                px, py, ph = bd.park(k)
            rb.x, rb.y = px + rng.uniform(-6, 6), py + rng.uniform(-4, 4)
            rb.th = math.radians(ph + rng.uniform(-15, 15))
            rb._record()
            rb.fw_yaw_sign = 0                      # just switched on: needs `calibrate` (the compass check)
            rb.gyro_scale = rng.uniform(-0.085, -0.055)   # the gyro counts 91.5-94.5% of every turn (real: ~93%)
            rb.world_polys = [list(p_) for p_ in self.polys]
            rb.update_solids()
            rb.radio = FS.Radio(self, n, ip, self.laptop)
            rb.others = (lambda me: (lambda: [self.outline(o) for o in self.robots if o is not me]))(rb)
            add_identify(rb, R)
            self.robots.append(rb)
        import fleetmap as FM
        self._body = FM.body_of(FM.normalize(dict(format="alvik-fleet-map", version=1, floor=dict(
            x_min=bd.x0, x_max=bd.x1, y_min=bd.y0, y_max=bd.y1), robots=[], tasks={})))
        self.R = R

    def set_bricks(self, on):
        """Bricks on the board (solid for every robot's body and its front distance sensor) or off."""
        on = bool(on) and bool(self.brick_polys)
        if on == self.bricks_on:
            return False
        for rb in self.robots:
            polys = [list(p_) for p_ in self.polys] + ([list(p_) for p_ in self.brick_polys] if on else [])
            rb.world_polys = polys                  # one assignment: the robot's own thread sees old or new, never half
        self.bricks_on = on
        return True

    # what fleet_sim.Radio and LiveRobot.firmware_loop expect of the simulation
    def dropped(self, name, t):
        return False

    def follow_done(self, rb):
        pass

    def outline(self, rb):
        return self._body.corners([[rb.x, rb.y, rb.th]])[0].tolist()

    def start(self):
        for rb in self.robots:
            threading.Thread(target=rb.firmware_loop, args=(self,), daemon=True).start()

    @staticmethod
    def view(rb):
        """(x, y, heading deg, LED brightness 0-1) for the camera."""
        led = rb.leds[-1] if rb.leds else (0, (0, 0, 0), (0, 0, 0))
        lv = max(max(led[1]), max(led[2]))
        return rb.x, rb.y, math.degrees(rb.th), lv

    def truth(self, rb):
        p = getattr(rb, "pose", None)
        if p is not None and getattr(p, "known", False):
            return p.x, p.y, p.th()
        return float("nan"), float("nan"), float("nan")

    def truth_loop(self, path):
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "robot", "x_cm", "y_cm", "heading_deg", "believed_x", "believed_y", "believed_heading"])
            while not self.stop.is_set():
                t = time.time()
                for r in self.robots:
                    bx, by, bh = self.truth(r)
                    w.writerow(["%.3f" % t, r.name, "%.3f" % r.x, "%.3f" % r.y, "%.2f" % math.degrees(r.th),
                                "%.2f" % bx, "%.2f" % by, "%.2f" % bh])
                f.flush()
                time.sleep(0.05)


# =============================================================================== the camera
class SimCamera:
    def __init__(self, world, args):
        import camera as SC
        import config as SCFG
        self.w = world
        cfg = SCFG.load_config(os.path.join(TOOLS, "sim", "configs", "real_oct18.json"))
        c = cfg.camera
        c.floor_width_cm = 640.0 / args.px_per_cm
        c.tilt_deg = [args.tilt, -0.6 * args.tilt]
        c.yaw_deg = -90.0 + args.yaw
        c.center_floor_cm = [61.0, 61.0]
        c.noise_sd = getattr(args, "cam_noise", 2.5)          # (a bare Namespace, as the tests make: the old clean camera)
        self.jitter = getattr(args, "cam_jitter", 0.0)
        self.jpeg = getattr(args, "jpeg", 92)
        cfg.session.tape_lines = []
        self.cfg = cfg
        self.geom = SC.CameraGeometry(c)
        rng = np.random.default_rng(args.seed)
        self.R = SC.Renderer(cfg, self.geom, rng)
        self.rng_j = np.random.default_rng(args.seed + 99)
        self.bg = self.make_background(rng)
        self.bg_plain = self.bg
        self.bricks_drawn = False
        self.fps = args.fps
        self.pi_offset = args.pi_offset
        self.latest = None          # (jpeg bytes, frame number, t_wall)
        self.rec = None             # SimRecorder when recording (--record)
        self.cond = threading.Condition()
        self.n = 0

    def make_background(self, rng):
        g, bd = self.geom, self.w.board
        H, W = g.H, g.W
        img = np.full((H, W, 3), 70, np.uint8)
        img = np.clip(img.astype(np.float32) + rng.normal(0, 4, (H, W, 1)), 0, 255).astype(np.uint8)

        def poly(pts, col):
            u, v = g.project(np.array([p[0] for p in pts]), np.array([p[1] for p in pts]),
                             np.array([p[2] if len(p) > 2 else 0.0 for p in pts]))
            q = np.round(np.stack([u, v], 1) * 16).astype(np.int32)
            cv2.fillPoly(img, [q], col, lineType=cv2.LINE_AA, shift=4)
        n = 24
        x0, y0, x1, y1 = bd.x0, bd.y0, bd.x1, bd.y1
        edge = [(x0 + (x1 - x0) * k / n, y0) for k in range(n)] + [(x1, y0 + (y1 - y0) * k / n) for k in range(n)]
        edge += [(x1 - (x1 - x0) * k / n, y1) for k in range(n)] + [(x0, y1 - (y1 - y0) * k / n) for k in range(n)]
        poly(edge, (150, 170, 190))          # plywood
        # wood grain
        lay = img.astype(np.float32)
        grain = cv2.GaussianBlur(rng.normal(0, 1, (H, W)).astype(np.float32), (0, 0), 1.2) * 4
        img = np.clip(lay + grain[:, :, None], 0, 255).astype(np.uint8)
        # the two blocked boards: 15 cm tall, 1.8 cm thick, inner face at the board line
        Hb, T = 15.0, 1.8
        xs = np.linspace(x0 - 2, x1 + 2, n)
        yl = bd.xboard_cm
        o = -bd.uy
        face = [(x, yl, 0.0) for x in xs] + [(x, yl, Hb) for x in xs[::-1]]
        top = [(x, yl, Hb) for x in xs] + [(x, yl + o * T, Hb) for x in xs[::-1]]
        poly(face, (95, 100, 110))
        poly(top, (200, 205, 210))
        ys = np.linspace(y0 - 2, y1 + 2, n)
        xl = bd.yboard_cm
        o = -bd.ux
        face = [(xl, y, 0.0) for y in ys] + [(xl, y, Hb) for y in ys[::-1]]
        top = [(xl, y, Hb) for y in ys] + [(xl + o * T, y, Hb) for y in ys[::-1]]
        poly(face, (95, 100, 110))
        poly(top, (200, 205, 210))
        img = np.clip(img.astype(np.float32) * self.R.gain[:, :, None], 0, 255).astype(np.uint8)
        return img

    def draw_bricks(self, img, polys, height_cm):
        """Boxes height_cm tall on the floor polygons: the sides the camera sees (darker), then the top."""
        g = self.geom
        out = img.copy()
        for p_ in polys:
            for k in range(len(p_)):
                a, b = p_[k], p_[(k + 1) % len(p_)]
                pts = [(a[0], a[1], 0.0), (b[0], b[1], 0.0), (b[0], b[1], height_cm), (a[0], a[1], height_cm)]
                u, v = g.project(np.array([q[0] for q in pts]), np.array([q[1] for q in pts]), np.array([q[2] for q in pts]))
                cv2.fillPoly(out, [np.round(np.stack([u, v], 1) * 16).astype(np.int32)], (55, 62, 118),
                             lineType=cv2.LINE_AA, shift=4)
            u, v = g.project(np.array([q[0] for q in p_]), np.array([q[1] for q in p_]), np.full(len(p_), height_cm))
            cv2.fillPoly(out, [np.round(np.stack([u, v], 1) * 16).astype(np.int32)], (72, 86, 150),
                         lineType=cv2.LINE_AA, shift=4)
        return out

    def sync_bricks(self):
        """The picture follows the world's bricks (on / off)."""
        on = bool(getattr(self.w, "bricks_on", False))
        if on == self.bricks_drawn:
            return
        self.bg = self.draw_bricks(self.bg_plain, self.w.brick_polys, self.w.brick_h) if on else self.bg_plain
        self.bricks_drawn = on

    def true_camera_model(self, path):
        """The true camera as a studio camera file (what a perfect floor calibration would give)."""
        from cameras import CameraModel
        xs, ys = np.meshgrid(np.linspace(-10, 132, 9), np.linspace(-10, 132, 9))
        fl = np.column_stack([xs.ravel(), ys.ravel()])
        u, v = self.geom.project(fl[:, 0], fl[:, 1])
        m = CameraModel.from_points(np.column_stack([u, v]), fl, (self.geom.W, self.geom.H), height_cm=self.geom.Hc,
                                    k1="off", mirror="auto", name="overhead")
        d = m.to_dict()
        d["name"] = "overhead"
        with open(path, "w") as f:
            json.dump(d, f, indent=1)
        return path

    def render(self):
        self.sync_bricks()
        img = self.bg.copy()
        for r in self.w.robots:
            x, y, th, lv = self.w.view(r)
            col = (lv, lv, lv)
            self.R.draw_robot(img, x, y, math.radians(th), 0.0, col, col)
        self.R.add_noise(img)
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if self.jitter > 0:                      # the camera mount shakes a little every frame
            dx, dy = (float(v) for v in self.rng_j.normal(0, self.jitter, 2))
            g = cv2.warpAffine(g, np.float32([[1, 0, dx], [0, 1, dy]]), (g.shape[1], g.shape[0]),
                               flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        return g

    def loop(self):
        period = 1.0 / self.fps
        nxt = time.time()
        while not self.w.stop.is_set():
            now = time.time()
            if now < nxt:
                time.sleep(nxt - now)
                continue
            nxt += period
            if nxt < now:
                nxt = now + period
            tc = time.time()
            g = self.render()
            ok, jpg = cv2.imencode(".jpg", g, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg])
            if not ok:
                continue
            with self.cond:
                self.n += 1
                self.latest = (jpg.tobytes(), self.n, tc + 0.04 + self.pi_offset)
                self.cond.notify_all()
            if self.rec is not None:
                self.rec.frame(g, tc + 0.04 + self.pi_offset)

    def handler(self):
        cam = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                if self.path.split("?")[0] != "/live":
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                last = 0
                try:
                    while not cam.w.stop.is_set():
                        with cam.cond:
                            while cam.latest is None or cam.latest[1] <= last:
                                cam.cond.wait(0.5)
                                if cam.w.stop.is_set():
                                    return
                            body, n, tw = cam.latest
                        last = n
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(body)).encode()
                                         + b"\r\nX-Frame: " + str(n).encode() + b"\r\nX-Time: " + ("%.4f" % tw).encode()
                                         + b"\r\n\r\n" + body + b"\r\n")
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
        return H


class SimRecorder:
    """Optional (--record, or E02_SIM_RECORD=1 in the environment, so e02.py --dry-run needs no change): the
    overhead picture saved as tools/pi/camrec.py saves a two-camera session: <folder>/session.json and
    <folder>/cam0/ with video.avi (MJPG), frames.csv (frame, t_wall, t_mono, cam_ms; row i = frame i),
    events.csv (t_wall, frame, text: every note heard on the notes port) and meta.json. t_wall is the simulated
    Pi's clock (laptop + --pi-offset, + the 40 ms capture delay for frames), as on /live."""

    def __init__(self, folder, size, fps, pi_offset):
        self.folder = os.path.join(folder, "cam0")
        os.makedirs(self.folder, exist_ok=True)
        self.lock = threading.Lock()
        self.n = 0
        self.t0 = time.monotonic()
        self.pi_offset = pi_offset
        self.vw = cv2.VideoWriter(os.path.join(self.folder, "video.avi"), cv2.VideoWriter_fourcc(*"MJPG"), fps, size)
        try:
            self.vw.set(cv2.VIDEOWRITER_PROP_QUALITY, 80)
        except Exception:
            pass
        self.ff = open(os.path.join(self.folder, "frames.csv"), "w", newline="")
        self.fw = csv.writer(self.ff)
        self.fw.writerow(["frame", "t_wall", "t_mono", "cam_ms"])
        self.ef = open(os.path.join(self.folder, "events.csv"), "w", newline="")
        self.ew = csv.writer(self.ef)
        self.ew.writerow(["t_wall", "frame", "text"])
        meta = dict(device="simulator", width=size[0], height=size[1], fps_requested=fps, host="e02_sim",
                    role="overhead", camera_index=0, note="t_wall is the simulated Pi's clock")
        with open(os.path.join(self.folder, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        with open(os.path.join(folder, "session.json"), "w") as f:
            json.dump(dict(session=os.path.basename(folder), label="E02-sim", host="e02_sim",
                           cameras=[dict(index=0, folder="cam0", role="overhead", width=size[0], height=size[1])]),
                      f, indent=2)

    def frame(self, gray, t_wall):
        with self.lock:
            self.vw.write(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR))
            self.fw.writerow([self.n, "%.4f" % t_wall, "%.4f" % (time.monotonic() - self.t0), ""])
            self.n += 1
            if self.n % 50 == 0:
                self.ff.flush()

    def mark(self, text, t_laptop):
        with self.lock:
            self.ew.writerow(["%.3f" % (t_laptop + self.pi_offset), self.n, text])
            self.ef.flush()

    def close(self):
        with self.lock:
            self.vw.release()
            self.ff.close()
            self.ef.close()


def notes_loop(world, path, port, rec=None):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", port))
    s.settimeout(0.3)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "note"])
        while not world.stop.is_set():
            try:
                data, addr = s.recvfrom(1024)
            except socket.timeout:
                continue
            t = time.time()
            text = data.decode("utf-8", "replace")
            w.writerow(["%.3f" % t, text])
            f.flush()
            if rec is not None:
                rec.mark(text, t)
            s.sendto(b"ok " + data[:200], addr)
    s.close()


def bricks_loop(world, flag):
    """Bricks on the board while the flag file exists (e02.py makes and removes it)."""
    while not world.stop.is_set():
        if world.set_bricks(os.path.exists(flag)):
            print("e02_sim: bricks %s" % ("on the board" if world.bricks_on else "taken off"), flush=True)
        time.sleep(0.2)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="e02_sim_out")
    ap.add_argument("--robots", default="Left,Center,Right")
    ap.add_argument("--seed", type=int, default=2)
    ap.add_argument("--start", default="parks", choices=["parks", "swapped", "scatter", "corners"],
                    help="where the robots stand at the start: near their park spots, each on another's, anywhere, or "
                         "Nick's 2026-10-22 corners layout")
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--px-per-cm", type=float, default=2.34, help="camera scale on the floor (real footage: 2.34)")
    ap.add_argument("--tilt", type=float, default=3.0, help="camera tilt from straight down, deg")
    ap.add_argument("--yaw", type=float, default=1.5, help="camera turned against the board, deg")
    ap.add_argument("--xboard-side", type=int, default=1, help="x-board along y = max (1, Nick's layout) or y = 0 (0)")
    ap.add_argument("--yboard-side", type=int, default=1, help="y-board along x = max (1, Nick's layout) or x = 0 (0)")
    ap.add_argument("--board-inset", type=float, default=0.6, help="boards' inner faces this far inside the plywood edge")
    ap.add_argument("--pi-offset", type=float, default=3.3, help="the Pi's clock minus the laptop's, s")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--max-s", type=float, default=4 * 3600)
    ap.add_argument("--drift-sd", type=float, default=0.15, help="gyro creep per robot, sd, deg/s")
    ap.add_argument("--slip", type=float, default=1.0, help="tyre slip while driving, %%")
    ap.add_argument("--cam-noise", type=float, default=6.0, help="camera sensor noise, grey levels (sd)")
    ap.add_argument("--cam-jitter", type=float, default=0.0, help="whole picture shake per frame, px (sd)")
    ap.add_argument("--jpeg", type=int, default=80)
    ap.add_argument("--wall-stall", type=float, default=0.96,
                    help="share of its speed a wheel keeps when pushing a board (real Left, 2026-10-22: ~0.96: the tyres "
                         "slip, no stall; the firmware's touch cue finds the bump)")
    ap.add_argument("--miss-bump", type=float, default=0.03, help="chance a bump is missed altogether (the find fails)")
    ap.add_argument("--miss-jolt", type=float, default=0.10, help="chance a bump's jolt is missed (found by the stall)")
    ap.add_argument("--bricks", default="", help="brick demo: 'x,y,length,width[,deg];...' cm, put on the board while "
                                                  "<out>/BRICKS_ON exists")
    ap.add_argument("--brick-height", type=float, default=6.0, help="the bricks' height, cm")
    ap.add_argument("--record", action="store_true",
                    help="also save the overhead picture as a camrec.py recording in <out>/recording (cam0/video.avi, "
                         "frames.csv, events.csv): what e02_analysis.py reads. Also on with E02_SIM_RECORD=1")
    a = ap.parse_args(argv)
    a.record = a.record or os.environ.get("E02_SIM_RECORD", "") not in ("", "0")
    os.makedirs(a.out, exist_ok=True)
    world = World(a)
    world.start()
    cam = SimCamera(world, a)
    cam.true_camera_model(os.path.join(a.out, "camera_model.json"))
    rec = None
    if a.record:
        rec = SimRecorder(os.path.join(a.out, "recording"), (cam.geom.W, cam.geom.H), a.fps, a.pi_offset)
        cam.rec = rec
    with open(os.path.join(a.out, "sim_info.json"), "w") as f:
        json.dump(dict(note="the self-calibration should find gyro_scale ~ g_true, gyro_drift_dps ~ gyro_drift_dps, "
                            "wheel_scale ~ w_true (wheel radius only: the wheels' slip makes the real number a little "
                            "lower)",
                       board=world.board.to_dict(),
                       model="tools/sim/robot.py + fw_e02.py (the firmware lane's E02 model), real time",
                       bricks=[dict(b, corners=brick_corners(b), height_cm=world.brick_h) for b in world.brick_specs],
                       robots={r.name: dict(g_true=1.0 / (1.0 + r.gyro_scale), raw_drift_dps=r.gyro_bias,
                                            gyro_drift_dps=r.gyro_bias / (1.0 + r.gyro_scale),
                                            w_true=0.5 * (r.rL + r.rR) / r.m.wheel_radius_cm, ip=r.ip,
                                            start=[r.x, r.y, math.degrees(r.th)]) for r in world.robots}),
                  f, indent=1)
    threads = [threading.Thread(target=cam.loop, daemon=True),
               threading.Thread(target=world.truth_loop, args=(os.path.join(a.out, "truth.csv"),), daemon=True),
               threading.Thread(target=notes_loop, args=(world, os.path.join(a.out, "notes.csv"), NOTES_PORT, rec),
                                daemon=True)]
    if world.brick_polys:
        threads.append(threading.Thread(target=bricks_loop, args=(world, os.path.join(a.out, "BRICKS_ON")), daemon=True))
    servers = []
    for port in (a.port, a.port + 1):
        srv = ThreadingHTTPServer(("127.0.0.1", port), cam.handler())
        srv.daemon_threads = True
        servers.append(srv)
        threads.append(threading.Thread(target=srv.serve_forever, daemon=True))
    for t in threads:
        t.start()
    print("e02_sim: robots %s; camera http://127.0.0.1:%d/live and :%d/live; notes UDP %d"
          % (", ".join("%s at %s" % (r.name, r.ip) for r in world.robots), a.port, a.port + 1, NOTES_PORT), flush=True)
    try:
        t0 = time.time()
        while time.time() - t0 < a.max_s and not os.path.exists(os.path.join(a.out, "STOP")):
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    world.stop.set()
    for s in servers:
        s.shutdown()
    time.sleep(0.5)
    if rec is not None:
        rec.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
