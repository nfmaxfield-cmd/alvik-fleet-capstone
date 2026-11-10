#!/usr/bin/env python3
"""Free roam stress simulator (2026-11-05): the free-roam controller (freeroam_core.Controller, the same code the Pi
runs) against a kinematic model of three robots, many times faster than real time, for many seeds and hours.

What it models (each switchable, all on by default; numbers below, all CHOSEN to be harsher than the home logs):
  robots     9.5 cm squares; turn on the spot at up to 92 deg/s (slower near the end), drive with the firmware's speed
             ramp (drive_accel 20) and top speed 12.5 cm/s; each robot's own pose (wheels + gyro) with a wheel-scale
             error (+-3 %) and a gyro-scale error (+-1.5 %), pulled to the camera's fix as the firmware's fusion does;
             a bad turn now and then (1 in 50 ends 8-20 deg off); the step queue, acks, fspd / fsteer (new firmware)
             or halt + resend (old), the front sensor stop (follow_look_cm 8, path +-6 cm), the watchdog (1.5 s)
  network    each packet both ways 5-40 ms late, PACKET_LOSS lost (default 3 %)
  camera     a fix per robot 12 times a second, 0.5 cm noise, 0.12-0.2 s late, a heading only now and then;
             tag dropouts: about one every 8 minutes per robot, 2-25 s long (the camera does not see it)
  faults     one robot stops answering (dead where it stands) at a random time in some runs; the Pi frozen for 4 s
             (controller silent: the watchdog should stop every robot) once per run
  battery    100 % at the start, 24 %/h moving, 6 %/h standing (the 2 Nov low-battery run: about 14 %/h idle with the
             lights on; CHOSEN, the real curve is one of the run's outputs)
Truth it measures: the gap between the two square bodies (exact polygon distance) every step; contact = overlap.
(e08, Phase A rehearsals) options: --edge-physics (a wheel over the board's edge HANGS the robot, tilted 6.5 deg, as
Center on 6 Nov: the firmware's tilt stop ends the step, a straight reverse spins its wheels without moving it, a reverse
or pivot that works the wheel still on the board frees it; the line sensors stop a forward drive whose nose passes the
edge; a middle past the edge is a fall) - a CHOSEN model of one real hang, flagged in the e08 report; --own-off
NAME@T:DX,DY,DUR (its own pose off the truth for DUR s, the camera's fixes refused meanwhile: the 6 Nov run 1 case);
--silent NAME@T:DUR (deaf and mute, then back); --held NAME=KIND with --cam-misplace NAME@T0:T1:DX,DY (the camera puts
it DX,DY off for a while: Right on 6 Nov); --phantoms N (false front stops per robot-hour, nothing there); rule
switches --edge-rule / --pose-rule / --keepout-rule / --silent-out / --no-tilt-recovery (the controller's).
(11-05c, after the first real run) options: --cam-delay (the fixes' delay, the real Pi ~0.45 s; 1.0 s rehearsed) and
--cam-clock (the camera's stamps on a clock this far ahead); --wall-in (a real wall this far inside the believed board's
x_min edge: bodies stop at it, the front sensor sees it); --trap (Right starts 7 cm from an edge facing out); --fw
2026-10-28a (no fspd / fsteer / watchdog; reverse drives yes, as every firmware since 2026-10-18b). The robots' turn steps
fail when something is within safe_front_cm (7) of the front (firmware frontLimit for T), as on the real robots.

    python3 tools/fleet/freeroam_fastsim.py --hours 4 --seeds 1-8 --out logs/freeroam-stress
    python3 tools/fleet/freeroam_fastsim.py --hours 0.5 --seeds 1 --old-fw          # the halt + resend method
    python3 tools/fleet/freeroam_fastsim.py summarize logs/freeroam-stress/*.jsonl --png=gaps.png
"""
import argparse
import csv
import json
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
VERSION = "fastsim-2026-11-10e13"   # e13: nopic (a whole-camera picture outage). No version string before
import freeroam_core as FC   # noqa: E402

DT = 0.05
TICK = 0.05
BOARD = (0.0, 0.0, 122.24, 121.92)
PACKET_LOSS = 0.03
TURN_MAX_DPS = 92.0
V_TOP = 12.5
HUNG_TILT = 6.5     # (e08 model) a robot hung with a wheel over the edge reads this tilt (6 Nov: 6.1 deg stop, 4.5 standing)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def corners(x, y, th):
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
    h = FC.BODY / 2
    return [(x + c * a - s * b, y + s * a + c * b) for a, b in ((h, h), (-h, h), (-h, -h), (h, -h))]


def _pt_seg(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _inside(p, poly):
    sg = None
    for i in range(4):
        a, b = poly[i], poly[(i + 1) % 4]
        cr = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if sg is None:
            sg = cr > 0
        elif (cr > 0) != sg:
            return False
    return True


def body_gap(A, B):
    """Distance between two square bodies (poses (x, y, th)); 0 if they overlap."""
    if math.hypot(A[0] - B[0], A[1] - B[1]) > 2 * FC.HALF + 0.01:
        return math.hypot(A[0] - B[0], A[1] - B[1]) - 2 * FC.HALF + 0.0   # lower bound is enough far apart
    pa, pb = corners(*A), corners(*B)
    if any(_inside(p, pb) for p in pa) or any(_inside(p, pa) for p in pb):
        return 0.0
    d = min(_pt_seg(p, pb[i], pb[(i + 1) % 4]) for p in pa for i in range(4))
    return min(d, min(_pt_seg(p, pa[i], pa[(i + 1) % 4]) for p in pb for i in range(4)))


class Net:
    def __init__(self, rng, loss, lat_max=0.04):
        self.rng, self.loss, self.q, self.lat_max = rng, loss, [], lat_max

    def send(self, t, fn, *args):
        if self.rng.random() < self.loss:
            return
        self.q.append((t + self.rng.uniform(0.005, self.lat_max), fn, args))

    def due(self, t):
        if not self.q:
            return []
        out = [e for e in self.q if e[0] <= t]
        if out:
            self.q = [e for e in self.q if e[0] > t]
            out.sort(key=lambda e: e[0])
        return out


class Bot:
    def __init__(self, name, x, y, th, rng, new_fw, fw=None):
        self.name, self.rng = name, rng
        self.x, self.y, self.th = x, y, th          # truth
        self.ox, self.oy, self.oth = x, y, th       # own pose
        self.wheel_err = rng.uniform(-0.03, 0.03)
        self.gyro_err = rng.uniform(-0.015, 0.015)
        self.q, self.cur, self.max_id = [], None, 0
        self.last_done = self.last_failed = 0
        self.v, self.scale, self.steer = 0.0, 1.0, None
        self.state = "idle"
        self.battery = 100.0
        self.dead = False
        self.last_rx = 0.0
        self.new_fw = new_fw
        self.fw = fw or ("2026-11-05b" if new_fw else "2026-10-29b")
        self.hist = []                              # (t, ox, oy): the firmware fuses a fix against its pose THEN
        self.turn_target = None
        self.started = False
        self.following = True
        self.arc = self.fw >= FC.ARC_FW             # (11-05d) firmware 2026-11-05c: R pieces, farc, coast, dodge
        self.coast = None                           # dict(t, k, v, dodge) while rolling on after an R piece
        self.farc = None
        self.tx, self.ty = x, y                     # truth speed (speed-zero moments)

    def report(self, sim, sid, st, info=""):
        if st == "done" and sid:
            sim.fs_run[self.name] = 0
        elif st == "failed" and "obstacle" in info:
            sim.fs_run[self.name] += 1
            if sim.fs_run[self.name] == 3:          # a front-stop loop: 3 in a row with nothing finished between
                sim.stats["front_loops"] += 1
        if st == "done":
            self.last_done = sid
        elif st == "failed" and sid:
            self.last_failed = sid
        sim.up(self, "step", sid, st, info)


class Sim:
    def __init__(self, seed, hours, new_fw=True, dead_robot=True, pi_freeze=True, dropouts=True, loss=PACKET_LOSS,
                 lost_mode="reckon", out_dir=None, names=("Left", "Center", "Right"), lat_max=0.04, drill=0.0,
                 cam_delay=None, cam_clock=0.0, wall_in=0.0, trap=False, fw=None, starts=None, motion="curves",
                 fws=None, v_min=None, edge_physics=False, own_off=(), silent=(), held=None, cam_misplace=(),
                 phantoms=0.0, ctrl_kw=None, own_start=None, board=None, camera="names", scene=None, layer_kw=None,
                 sensors=False, pushes=(), bricks=(), cam_gaps=(), discover=False, workspace=False, cam_fault=None,
                 tag_yaw=None, cover=None, bottom_zone=False, saved_ws=None, hdg_fault=None, nopic=None):
        # (e09, Phase B) board: another table (the school's); camera "names" (the camera names every fix, as before) or
        # "detections" (fastsim_sources: tags / motion / lock-on / lights sightings with no names except tags and
        # locks, through the ONE tracking layer: names CAN swap); scene: dark / glare patches, tag read limit, two
        # robots driven past each other; sensors: RAW rows carry wheels, raw gyro rate (bias, scale), accelerometer,
        # front and line sensors (the filter's inputs); pushes: (name, t, dx, dy) a parked robot pushed; bricks:
        # (t, x, y) an object put on the table
        self.board = tuple(board) if board else BOARD
        # (e10, Phase C) the guesses the start must discover: cam_fault {scale: k (the board's size typed k x too big:
        # the camera's floor calibration then reads k x every distance), mirror: True (a wrong camera name: the picture
        # mirrored left to right), shift: (dx, dy), rot_deg}; tag_yaw {robot: deg} a roof tag turned on its robot; cover
        # [rects] what the camera can see (none elsewhere); bottom_zone: the look-ahead edge sensor (safe_edge 3: a drop
        # 9-11 cm ahead stops a forward drive); discover: the proof drive at the start (start_discovery.py); workspace: the
        # proven workspace (proven_workspace.py) for the planner; saved_ws: a saved workspace file
        self._scene0 = dict(scene or {})
        self.cam_fault = dict(cam_fault or {})
        self.cam_correct = None              # set by the discovery: the calibration it found
        self.tag_yaw = dict(tag_yaw or {})
        self.tag_yaw_fix = {}
        self.cover = [tuple(r) for r in cover] if cover else None
        self.bottom_zone = bool(bottom_zone)
        # (e10, the 7 Nov run 2 incident) hdg_fault {deg: 25, near_cm: 14, every_s: 60}: a long straight (35 cm or more)
        # starts with the robot's own heading `deg` off the truth, the way that takes it towards the nearest edge (only
        # when it would end within near_cm of one), and the firmware does not steer back by its position (Right's 55 cm
        # straight ended 23.6 cm off); the error stays until the robot is told its heading (fpose) or 30 s pass
        self.hdg_fault = dict(hdg_fault) if hdg_fault else None
        k_ = float(self.cam_fault.get("scale", 1.0))
        self.believed = tuple(self.board[i] if i < 2 else self.board[i - 2] + (self.board[i] - self.board[i - 2]) * k_
                              for i in range(4))
        self.typed = tuple(self.believed)    # the board as typed (the camera's calibration was made with it)
        self.rng = random.Random(seed * 7919 + 13)
        self.seed, self.hours = seed, hours
        self.t = 0.0
        self.down = Net(random.Random(seed * 31 + 1), loss, lat_max)
        self.upn = Net(random.Random(seed * 31 + 2), loss, lat_max)
        self.rows = {}
        self.out_dir = out_dir
        self.writers = {}
        self.names = list(names)
        # start spots: 30 cm in, 40 apart, random headings (as the ladder's dry board)
        while True:
            pts = [(self.rng.uniform(self.board[0] + 30, min(self.board[2] - 30, self.board[0] + 92)),
                    self.rng.uniform(self.board[1] + 30, self.board[3] - 30)) for _ in names]
            if all(math.dist(pts[i], pts[j]) >= 40 for i in range(len(pts)) for j in range(i + 1, len(pts))):
                break
        if fw:
            new_fw = fw >= FC.FRESH_FW
        self.bots = {n: Bot(n, p[0], p[1], self.rng.uniform(-180, 180), random.Random(seed * 101 + k), new_fw, fw)
                     for k, (n, p) in enumerate(zip(names, pts))}
        for n, f in (fws or {}).items():          # (11-05d) a firmware mix: each robot its own
            b = self.bots[n]
            b.fw, b.new_fw, b.arc = f, f >= FC.FRESH_FW, f >= FC.ARC_FW
        self.cam_delay, self.cam_clock = cam_delay, cam_clock
        # the real walls: the believed board, its x_min side wall_in cm further in (the planner is not told)
        self.walls = (self.board[0] + wall_in, self.board[1], self.board[2], self.board[3])
        if trap:            # Right: 7 cm from the y_max edge, facing out (Right, 5 Nov: stood 3 min 6.6 cm in)
            b = self.bots[names[-1]]
            b.x, b.y, b.th = 60.0, self.board[3] - 7.0, 88.0
            b.ox, b.oy, b.oth = b.x, b.y, b.th
            for o in self.bots.values():
                if o is not b and math.dist((o.x, o.y), (b.x, b.y)) < 40:
                    o.y = max(30.0, o.y - 40.0)
                    o.ox, o.oy = o.x, o.y
        for n, st in (starts or {}).items():
            b = self.bots[n]
            b.x, b.y, b.th = st
            b.ox, b.oy, b.oth = st
        self.ws = None
        self.discover_on = bool(discover)
        self.saved_ws = saved_ws
        if workspace:
            self.ws = self.make_ws(self.believed)
        self.ctrl = FC.Controller(names, self.believed, self.ctrl_out, self.ctrl_log, seed=seed, lost_mode=lost_mode,
                                  drill=drill, motion=motion, v_min_avoid=v_min, workspace=self.ws, **(ctrl_kw or {}))
        self.disc = None
        self.visits = {}                     # (e10) where each robot's true middle went: 5 cm cells -> seconds
        # (e08) rehearsal injections
        self.edge_physics = edge_physics
        self.own_off = list(own_off)            # (name, t, dx, dy, dur)
        self.silent_inj = list(silent)          # (name, t, dur)
        self.held0 = dict(held or {})
        self.misplace = list(cam_misplace)      # (name, t0, t1, dx, dy)
        self.phantoms = float(phantoms)
        self.tilt_lim = {n: 5.0 for n in names}
        for n, st in (own_start or {}).items():  # its own pose starts off the truth (a mis-placed start)
            b = self.bots[n]
            b.ox, b.oy, b.oth = st
        self.next_raw = {n: 0.0 for n in names}
        # (e09) the raw sensors (sensors=True) and the camera's sightings (camera="detections")
        self.sensors = bool(sensors)
        for k, b in enumerate(self.bots.values()):
            r = random.Random(seed * 211 + k)
            # MEASURED (e09, 5 / 6 Nov logs): the raw gyro's standing bias 0.40-0.76 deg/s, drifting ~0.02 deg/s over
            # hours; CHOSEN: its scale error +-0.5 % (it agrees with the camera-fitted firmware heading within ~1 %)
            b.gz_bias, b.gz_scale = r.uniform(0.40, 0.76), r.uniform(-0.005, 0.005)
            b.prev_th = b.th
            b.rawq = []
        self.camera_mode = camera
        self.pushes = list(pushes)
        self.bricks = list(bricks)
        self.src = None
        if camera == "detections":
            import fastsim_sources as FS
            self.src = FS.CameraSources(self, scene or {}, layer_kw or {})

        # (11-05d) truth: each robot's true speed every step; speed-zero moments (under 0.3 cm/s for 0.1 s or more)
        self.speed = {n: dict(moving_s=0.0, alive_s=0.0, zero_s=0.0, zero_run=0.0, zeros=0, why={}) for n in names}
        self.n_moving = [0.0, 0.0, 0.0, 0.0]     # seconds with 0, 1, 2, 3 robots moving (robots still in the roam)
        self.frozen_until = -1.0
        self.freeze_at = self.rng.uniform(0.2, 0.8) * hours * 3600 if pi_freeze else None
        self.dead_at = (self.rng.uniform(0.15, 0.7) * hours * 3600, self.rng.choice(names)) if dead_robot else None
        self.gaps = {n: [] for n in names}
        if dropouts:
            for n in names:
                t = self.rng.expovariate(1 / 480.0)
                while t < hours * 3600:
                    d = self.rng.uniform(2, 25)
                    self.gaps[n].append((t, t + d))
                    t += d + self.rng.expovariate(1 / 480.0)
        # (e13) nopic (t0, dur): the camera sends NO pictures for dur s from t0 (every robot unseen; the bridge's frame
        # watch message to the controller at the start and the end)
        self.nopic = tuple(nopic) if nopic else None
        self.nopic_said = 0
        if self.nopic:
            for n in names:
                self.gaps[n].append((self.nopic[0], self.nopic[0] + self.nopic[1]))
        for (nm, t0, dur) in cam_gaps:          # (e09) a camera gap for one robot (rehearsal c: 40 s)
            self.gaps[nm].append((t0, t0 + dur))
        self.cam_gaps = list(cam_gaps)
        self.cam_q = []
        self.next_fix = {n: self.rng.uniform(0, 0.08) for n in names}
        self.next_follow = {n: 0.0 for n in names}
        self.stats = dict(contacts=0, contact_s=0.0, min_gap=float("inf"), gap_windows=[], front_stops=0,
                          watchdog_stops=0, edge_min=float("inf"), off_board=0, wall_hits=0, front_loops=0)
        self.fs_run = {n: 0 for n in names}       # front stops in a row with no finished move between
        self.win_min = {}
        self.win_t = 0.0
        self.in_contact = set()
        for k in ("hangs", "falls", "edge_trips", "phantom_stops", "hung_s"):
            self.stats[k] = 0
        self.stats["edge_min_body"] = float("inf")
        self.stats["edge_min_centre"] = float("inf")

    # -- (e10) the camera's calibration, coverage, and the proven workspace ---------------------------------------
    def cam_map(self, x, y):
        """A true position as the camera reports it: through the fault (a wrong calibration), then the discovery's
        correction (the calibration it chose)."""
        f = self.cam_fault
        if f:
            k = float(f.get("scale", 1.0))
            x, y = self.board[0] + (x - self.board[0]) * k, self.board[1] + (y - self.board[1]) * k
            if f.get("rot_deg"):
                cx, cy = (self.typed[0] + self.typed[2]) / 2, (self.typed[1] + self.typed[3]) / 2
                a = math.radians(f["rot_deg"])
                x, y = cx + (x - cx) * math.cos(a) - (y - cy) * math.sin(a), cy + (x - cx) * math.sin(a) + (y - cy) * \
                    math.cos(a)
            if f.get("mirror"):
                x = self.typed[0] + self.typed[2] - x
            sx, sy = f.get("shift", (0.0, 0.0))
            x, y = x + sx, y + sy
        if self.cam_correct is not None:
            x, y = self.cam_correct(x, y)
        return x, y

    def cam_heading(self, th, name=None):
        """A heading as the camera reports it (a roof tag): the tag's turn on its robot, the fault's mirror, then the
        corrections."""
        if th is None:
            return None
        th = th + self.tag_yaw.get(name, 0.0)
        f = self.cam_fault
        if f.get("rot_deg"):
            th += f["rot_deg"]
        if f.get("mirror"):
            th = 180.0 - th
        if self.cam_correct is not None and getattr(self, "cam_correct_mirror", False):
            th = 180.0 - th
        th += self.tag_yaw_fix.get(name, 0.0)
        return wrap180(th)

    def in_cover(self, x, y):
        return self.cover is None or any(r[0] <= x <= r[2] and r[1] <= y <= r[3] for r in self.cover)

    def blind_rects(self):
        sc = self._scene0 or {}
        return list(sc.get("dark") or ()) + list(getattr(self, "extra_dark", ()))

    def make_ws(self, board):
        """The proven workspace at the start: what the camera sees of the board outline (its coverage, not a dark
        patch), plus each robot's own footprint (it stands there: felt)."""
        import proven_workspace as PW
        ws = PW.ProvenWorkspace(board)
        dd = getattr(self, "disc", None)
        cam_ok = not (dd is not None and dd.get("camera") in ("missing", "wrong", "unproven"))
        sc = getattr(self, "_scene0", None) or {}
        if cam_ok and not (sc.get("fault") == "none" and not sc.get("fault_t")):
            ws.see(board, self.cover, self.blind_rects())
        for b in self.bots.values() if hasattr(self, "bots") else ():
            ws.felt_strip(b.ox, b.oy, b.ox + 0.01, b.oy, 6.0)
        if self.saved_ws:
            ws.events.append(("saved", ws.merge_saved(self.saved_ws)))
        return ws

    def visit(self):
        if int(self.t * 2) == int((self.t - DT) * 2):
            return
        for b in self.bots.values():
            if not b.dead:
                k = (int(b.x // 5), int(b.y // 5))
                self.visits[k] = self.visits.get(k, 0.0) + 0.5

    # -- logs ---------------------------------------------------------------------------------------------------
    def longest_still(self):
        return _longest(self.__dict__.get("mot_rows", []))

    def ctrl_log(self, table, row):
        if table == "motion":
            self.__dict__.setdefault("mot_rows", []).append(dict(t=row["t"], robot=row["robot"], any=row["any"],
                                                                 mode=row["mode"]))
            m = self.__dict__.setdefault("mot", {})
            k = m.setdefault(row["t"], [0, 0, 0])
            k[0] += row["any"]
            k[1] += row["translating"]
            k[2] += int(row["mode"] not in ("out", "gone"))
            pr = self.__dict__.setdefault("mot_robot", {}).setdefault(row["robot"], [0, 0, 0])
            if row["mode"] not in ("out", "gone"):
                pr[0] += row["any"]
                pr[1] += row["translating"]
                pr[2] += 1
        self.rows.setdefault(table, 0)
        self.rows[table] += 1
        if table in ("reflex", "fallbacks", "lost", "breaks", "events", "moves", "recovery", "held", "edge_judgement",
                     "pose", "keepouts") or self.out_dir:
            self.keep(table, row)

    def keep(self, table, row):
        if self.out_dir is None:
            self.__dict__.setdefault("kept", {}).setdefault(table, []).append(row)
            return
        w = self.writers.get(table)
        if w is None:
            f = open(os.path.join(self.out_dir, table + ".csv"), "w", newline="")
            w = csv.DictWriter(f, fieldnames=list(row.keys()), extrasaction="ignore")
            w.writeheader()
            self.writers[table] = w
            self.__dict__.setdefault("files", []).append(f)
        w.writerow(row)

    # -- controller -> robots -----------------------------------------------------------------------------------
    def ctrl_out(self, name, text):
        if self.t < self.frozen_until:
            return
        self.down.send(self.t, self.robot_cmd, name, text)

    def up(self, b, kind, *args):
        if b.dead or getattr(b, "mute", False):
            return
        self.upn.send(self.t, self.ctrl_in, b.name, kind, args)

    def ctrl_in(self, name, kind, args):
        drv = getattr(self, "_disc_drv", None)
        if drv is not None:                     # (e10) the start's proof drive: its messages go to the discovery
            if kind == "step":
                drv.on_step(name, *args[:2], *(args[2:3] or ("",)))
            elif kind == "follow":
                drv.on_follow(name, self.t, args[1], args[2], args[3], args[0], args[5], args[6])
            return
        c = self.ctrl
        c.on_heard(name, self.t)
        if kind == "step":
            c.on_step(name, *args)
        elif kind == "follow":
            c.on_follow(name, self.t, *args)
        elif kind == "status":
            c.on_fw(name, args[0])
            c.on_battery(name, args[1], self.t)
        elif kind == "fspd":
            c.on_fspd(name)
        elif kind == "raw":
            c.on_raw(name, self.t, args[0], args[1], args[2])
        elif kind == "rawfull" and hasattr(c, "on_raw_full"):
            c.on_raw_full(name, args[0], *args[1:])

    def robot_cmd(self, name, text):
        b = self.bots[name]
        if b.dead:
            return
        b.last_rx = self.t
        parts = text.split()
        verb = parts[0]
        if getattr(b, "mute", False):
            return
        if verb == "set" and len(parts) >= 3 and parts[1] == "tilt_stop_deg":
            self.tilt_lim[name] = float(parts[2])
            return
        if verb == "status":
            self.up(b, "status", b.fw, int(b.battery))
        elif verb == "fix":
            pass
        elif verb == "fpose":
            b.ox, b.oy, b.oth = float(parts[1]), float(parts[2]), float(parts[3])
            if getattr(b, "hdg_off", None) is not None:
                b.hdg_off = None                    # (e10) told its heading: the injected error is over
                self.stats["hdg_fault_cured"] = self.stats.get("hdg_fault_cured", 0) + 1
            b.hist = []                     # pose.h set(): the history is cleared
        elif verb == "farc":
            if not b.arc:
                return
            k = float(parts[1])
            L = float(parts[2]) if len(parts) > 2 else None
            if b.cur and b.cur["type"] == "R":
                b.farc = (k, L)
                for s in b.q:
                    b.report(self, s["id"], "dropped", "farc")
                b.q = []
            elif b.coast:
                b.coast["k"], b.coast["t"] = k, self.t
            self.up(b, "fspd")
        elif verb == "halt":
            if b.coast and not b.cur:
                b.coast, b.v = None, 0.0
            for s in b.q:
                b.report(self, s["id"], "dropped", "halt")
            b.q = []
            if b.cur:
                b.report(self, b.cur["id"], "failed", "halted")
                b.cur, b.state = None, "idle"
            b.v = 0.0
            b.report(self, 0, "halted")
        elif verb == "hold":
            for s in b.q:
                b.report(self, s["id"], "dropped", "hold")
            b.q = []
            b.report(self, 0, "held")
        elif verb in ("fspd", "fsteer"):
            if not b.new_fw:
                return
            if verb == "fspd":
                b.scale = float(parts[1])
                if b.scale <= 0 and b.coast and not b.cur:
                    b.coast, b.v = None, 0.0
            else:
                b.steer = (float(parts[1]), float(parts[2]))
            self.up(b, "fspd")
        elif verb == "stop":
            b.q, b.cur, b.v, b.state, b.following = [], None, 0.0, "idle", False
        elif verb == "sleep":
            b.dead = True                      # asleep: deaf and still (it is parked: the run is over for it)
        elif verb == "fs":
            sid = int(parts[1])
            if sid <= b.max_id:
                b.report(self, sid, "dup")
                return
            if len(b.q) >= 8:
                b.report(self, sid, "full")
                return
            b.max_id = sid
            typ = parts[2]
            if typ == "T":
                b.q.append(dict(id=sid, type="T", h=float(parts[3])))
            elif typ == "D":
                v = [float(q) for q in parts[3:9]]
                b.q.append(dict(id=sid, type="D", x0=v[0], y0=v[1], x1=v[2], y1=v[3], h=v[4], v=v[5],
                                rev=len(parts) > 9 and parts[9] == "1"))
            elif typ == "R" and b.arc:
                v = [float(q) for q in parts[3:9]]
                b.q.append(dict(id=sid, type="R", x0=v[0], y0=v[1], h=v[2], k=v[3], L=v[4], v=v[5]))
            elif typ == "R":
                b.report(self, sid, "bad", "could not read the step")
                return
            b.report(self, sid, "queued")

    # -- physics ------------------------------------------------------------------------------------------------
    def front_cm(self, b, lat_max=6.0 + 4.75):
        """What the front sensor reads (cm from the front face): the nearest other body or wall ahead within
        lat_max of the centre line (robots: their middles; walls: the real walls, not the believed board)."""
        c, sn = math.cos(math.radians(b.th)), math.sin(math.radians(b.th))
        best = 999.0
        for o in self.bots.values():
            if o is b:
                continue
            dx, dy = o.x - b.x, o.y - b.y
            fwd, lat = dx * c + dy * sn, -dx * sn + dy * c
            if fwd > 0 and abs(lat) < lat_max:
                best = min(best, fwd - 9.5)
        x0, y0, x1, y1 = self.walls
        if self.edge_physics and self.walls == self.board:
            return best                 # (e08) the board's edges are open drops: nothing there for the front sensor
        for l in (-4.0, 0.0, 4.0):
            px, py = b.x + 4.75 * c - l * sn, b.y + 4.75 * sn + l * c
            for dist in ((px - x0) / -c if c < -1e-6 else 999, (x1 - px) / c if c > 1e-6 else 999,
                         (py - y0) / -sn if sn < -1e-6 else 999, (y1 - py) / sn if sn > 1e-6 else 999):
                best = min(best, dist)
        return best

    def bot_step(self, b):
        if b.dead or not b.following:
            return
        moving = b.state in ("turn", "drive", "curve", "coast", "dodge")
        b.battery -= (24.0 if moving else 6.0) * DT / 3600.0
        if b.battery < 9.5:
            b.q, b.v = [], 0.0          # the motor board stops the motors under 10 %
            if b.cur:
                b.report(self, b.cur["id"], "failed", "motors off (battery)")
                b.cur, b.state = None, "idle"
            return
        if b.new_fw and b.cur and self.t - b.last_rx > FC.WD_MS / 1000.0:
            b.report(self, b.cur["id"], "failed", "watchdog")
            for s in b.q:
                b.report(self, s["id"], "dropped", "an earlier step failed")
            b.q, b.cur, b.v, b.state = [], None, 0.0, "idle"
            self.stats["watchdog_stops"] += 1
            return
        if b.cur is None and b.coast is not None and not (b.q and b.q[0]["type"] == "R"):
            if self.coast_step(b):
                return
        if b.cur is None:
            if not b.q:
                b.state = "idle"
                return
            if b.new_fw and self.t - b.last_rx > FC.WD_MS / 1000.0:
                return
            rolling = b.coast is not None and b.q[0]["type"] == "R"
            nx = b.q[0]
            if getattr(b, "hung", False) and HUNG_TILT > self.tilt_lim[b.name] and not (nx["type"] == "D" and
                                                                                   nx.get("rev")):
                # firmware 2026-11-05d follow.h: tilted: only a straight backward drive starts
                b.q.pop(0)
                b.report(self, nx["id"], "failed", "tilted %.1f deg: only a backward drive is allowed" % HUNG_TILT)
                for q in b.q:
                    b.report(self, q["id"], "dropped", "an earlier step failed")
                b.q, b.state = [], "idle"
                return
            b.cur = b.q.pop(0)
            if not rolling:
                b.scale = 1.0
                b.coast = None
            b.steer = None
            b.state = {"T": "turn", "R": "curve"}.get(b.cur["type"], "drive")
            b.report(self, b.cur["id"], "started")
            if b.cur["type"] == "R":
                c = b.cur
                c["s"], c["prev"] = 0.0, None
                self.inject_heading(b, c)
                if not rolling:
                    b.v = 0.0
                    if abs(wrap180(c["h"] - b.oth)) > 30:
                        c["square"] = True
                b.coast = None
                if not c.get("square"):
                    self.curve_step(b, c)          # the firmware runs straight on: no lost time step
                return
            # firmware frontLimit for a turn (and a forward drive's square-up): safe_front_cm 7, centre zone
            if (b.cur["type"] == "T" or (not b.cur.get("rev") and abs(wrap180(b.cur["h"] - b.oth)) > 10)) and \
                    self.front_cm(b, lat_max=3.0) < 7.0:
                d = self.front_cm(b, lat_max=3.0)
                b.report(self, b.cur["id"], "failed", "obstacle %.1f cm ahead" % d)
                for q in b.q:
                    b.report(self, q["id"], "dropped", "an earlier step failed")
                b.q, b.cur, b.state = [], None, "idle"
                self.stats["front_stops"] += 1
                return
            if b.cur["type"] == "T":
                bad = b.rng.random() < 0.02
                b.cur["err"] = (b.rng.uniform(8, 20) * b.rng.choice((-1, 1))) if bad else b.rng.gauss(0, 1.5)
            else:
                if abs(wrap180(b.cur["h"] - b.oth)) > 10:
                    b.cur["square"] = True
            b.v = 0.0
        s = b.cur
        if s["type"] == "R" and not s.get("square"):
            self.curve_step(b, s)
            return
        if s["type"] == "T" or s.get("square"):
            tgt = s["h"]
            err = wrap180(tgt - b.oth)
            if abs(err) <= 1.5:
                if s["type"] == "T":
                    b.th = wrap180(b.th + s.get("err", 0.0))
                    self.edge_check(b)
                    b.report(self, s["id"], "done")
                    b.cur, b.state = None, "idle"
                else:
                    s["square"] = False
                    if s["type"] == "R":
                        b.state = "curve"
                return
            w = max(25.0, min(TURN_MAX_DPS, 2.3 * abs(err))) * (1 if err > 0 else -1)
            d = w * DT
            if abs(d) > abs(err):
                d = err
            b.oth = wrap180(b.oth + d)
            b.th = wrap180(b.th + d * (1 + b.gyro_err))
            self.edge_check(b)
            return
        # drive: on the own pose, as the firmware's driveLine
        if b.steer is not None:
            s["x1"], s["y1"] = b.steer
            s["h"] = math.degrees(math.atan2(b.steer[1] - b.oy, b.steer[0] - b.ox))
            b.steer = None
        sg = -1.0 if s.get("rev") else 1.0
        ux, uy = sg * math.cos(math.radians(s["h"])), sg * math.sin(math.radians(s["h"]))
        rem = (s["x1"] - b.ox) * ux + (s["y1"] - b.oy) * uy
        if rem <= b.v * 0.03 + 0.05:
            b.v = 0.0
            b.report(self, s["id"], "done")
            b.cur, b.state = None, "idle"
            return
        if getattr(b, "hung", False):
            # (e08 model) hung with a wheel over the edge: the wheels turn (its own pose counts them) but the body
            # stays, unless the reverse line works the wheel still on the board (its end offset towards that side)
            frees = s.get("rev") and self.frees(b, s)
            if not frees:
                sg_ = -1.0 if s.get("rev") else 1.0
                b.v = min(s["v"], b.v + 20.0 * DT + 0.5)
                b.ox += sg_ * b.v * DT * math.cos(math.radians(b.oth))
                b.oy += sg_ * b.v * DT * math.sin(math.radians(b.oth))
                return
        # (e08) false front stops (nothing there): --phantoms per robot-hour while driving forward
        if not s.get("rev") and self.phantoms > 0 and b.v > 0.5 and \
                b.rng.random() < self.phantoms * DT / 3600.0 / 0.4:
            b.v = 0.0
            b.report(self, s["id"], "failed", "obstacle 5.0 cm ahead; zones 5.0 5.0 5.0 5.0 5.0")
            for q in b.q:
                b.report(self, q["id"], "dropped", "an earlier step failed")
            b.q, b.cur, b.state = [], None, "idle"
            self.stats["phantom_stops"] += 1
            return
        # front sensor: another body (or a real wall) in the path within follow_look_cm (8) of the front face
        if not s.get("rev"):
            fc = self.front_cm(b)
            if fc < 8.0 and b.v > 0.5:
                b.v = 0.0
                b.report(self, s["id"], "failed", "obstacle %.1f cm ahead" % fc)
                for q in b.q:
                    b.report(self, q["id"], "dropped", "an earlier step failed")
                b.q, b.cur, b.state = [], None, "idle"
                self.stats["front_stops"] += 1
                return
        sc = max(0.0, min(1.5, b.scale))
        vt = min(s["v"] * sc, V_TOP, math.sqrt(2 * 20.0 * max(rem, 0)) + 2.0 * min(sc, 1.0))
        b.v = min(vt, b.v + 20.0 * DT + 0.5)
        cross = ux * (b.oy - s["y1"]) - uy * (b.ox - s["x1"])
        corr = max(-15.0, min(15.0, -3.0 * cross)) if rem > 3 else 0.0
        herr = wrap180(s["h"] + sg * corr - b.oth)
        w = max(-60.0, min(60.0, 4.0 * herr))
        b.oth = wrap180(b.oth + w * DT)
        b.th = wrap180(b.th + w * DT * (1 + b.gyro_err))
        d = sg * b.v * DT
        b.ox += d * math.cos(math.radians(b.oth))
        b.oy += d * math.sin(math.radians(b.oth))
        nx = b.x + d * (1 + b.wheel_err) * math.cos(math.radians(b.th))
        ny = b.y + d * (1 + b.wheel_err) * math.sin(math.radians(b.th))
        x0, y0, x1, y1 = self.walls
        if self.walls != self.board and nx < x0 + 4.75:        # a real wall: the body stops at it (wheels slip)
            self.stats["wall_hits"] += 1
            nx = x0 + 4.75
        b.x, b.y = nx, ny
        self.edge_check(b)

    # -- (e08) the edge physics (a CHOSEN model of the 6 Nov hang) ----------------------------------------------------
    def wheels(self, b):
        c, sn = math.cos(math.radians(b.th)), math.sin(math.radians(b.th))
        return [(b.x - sn * FC.HALF_TRACK * k, b.y + c * FC.HALF_TRACK * k) for k in (1, -1)]   # left, right

    def frees(self, b, s):
        """A reverse that works the wheel still on the board frees a hung robot: its line's end lies towards that
        wheel's side (the steering drives that wheel harder), or the robot is no longer over (a pivot freed it)."""
        on = [w for w in self.wheels(b) if FC.edge_dist(self.board, *w) > 0.0]
        if len(on) != 1:
            return True
        c, sn = math.cos(math.radians(b.th)), math.sin(math.radians(b.th))
        side = -sn * (s["x1"] - b.x) + c * (s["y1"] - b.y)          # + = towards its left
        left_on = FC.edge_dist(self.board, *self.wheels(b)[0]) > 0.0
        return (side > 0.8 and left_on) or (side < -0.8 and not left_on)

    def edge_check(self, b):
        if not self.edge_physics or b.dead:
            return
        if FC.edge_dist(self.board, b.x, b.y) < 0.0:
            self.stats["falls"] += 1
            b.dead, b.v, b.state = True, 0.0, "idle"
            self.ctrl.event(self.ctrl.R[b.name], "fell (sim)", "its middle went over the edge")
            return
        over = [FC.edge_dist(self.board, *w) < 0.0 for w in self.wheels(b)]
        if any(over) and not getattr(b, "hung", False):
            b.hung = True
            self.stats["hangs"] += 1
            b.v = 0.0
            if b.cur is not None and not (b.cur["type"] == "D" and b.cur.get("rev")) and \
                    HUNG_TILT > self.tilt_lim[b.name]:
                b.report(self, b.cur["id"], "failed", "tilted %.1f deg" % HUNG_TILT)
                for q in b.q:
                    b.report(self, q["id"], "dropped", "an earlier step failed")
                b.q, b.cur, b.state, b.coast = [], None, "idle", None
        elif not any(over) and getattr(b, "hung", False):
            b.hung = False
        # (e10) the look-ahead bottom zone (safe_edge 3; feel_table: notices a drop 9-11 cm ahead of the wheels): a
        # forward drive that would bring that spot over the edge stops (--bottom-zone; CHOSEN 9.5-10.5 cm here)
        if self.bottom_zone and b.cur is not None and b.cur["type"] in ("D", "R") and not b.cur.get("rev") and \
                not getattr(b, "hung", False):
            c, sn = math.cos(math.radians(b.th)), math.sin(math.radians(b.th))
            reach = 9.5 + b.rng.random()
            if FC.edge_dist(self.board, b.x + reach * c, b.y + reach * sn) < 0.0:
                self.stats["bottom_trips"] = self.stats.get("bottom_trips", 0) + 1
                b.v = 0.0
                b.report(self, b.cur["id"], "failed", "plywood edge at %.1f %.1f %.1f bottom" % (b.ox, b.oy, b.oth))
                for q in b.q:
                    b.report(self, q["id"], "dropped", "an earlier step failed")
                b.q, b.cur, b.state, b.coast = [], None, "idle", None
                return
        # the line sensors (safe_edge 3): a forward drive whose nose passes the edge stops
        if b.cur is not None and b.cur["type"] in ("D", "R") and not b.cur.get("rev") and not getattr(b, "hung", False):
            c, sn = math.cos(math.radians(b.th)), math.sin(math.radians(b.th))
            if FC.edge_dist(self.board, b.x + 4.0 * c, b.y + 4.0 * sn) < -0.5:
                self.stats["edge_trips"] += 1
                b.v = 0.0
                b.report(self, b.cur["id"], "failed", "plywood edge at %.1f %.1f %.1f line" % (b.ox, b.oy, b.oth))
                for q in b.q:
                    b.report(self, q["id"], "dropped", "an earlier step failed")
                b.q, b.cur, b.state, b.coast = [], None, "idle", None

    def raw(self):
        """RAW rows to the controller (roll / pitch / speed), 10 a second: hung = 6.5 deg."""
        for n, b in self.bots.items():
            if b.dead or getattr(b, "mute", False) or self.t < self.next_raw[n]:
                continue
            self.next_raw[n] = self.t + 0.1
            tilt = HUNG_TILT if getattr(b, "hung", False) else 0.0
            self.up(b, "raw", tilt + b.rng.gauss(0, 0.15), b.rng.gauss(0, 0.15), b.v)
            if self.sensors:
                # (e09) the filter's inputs, in true units (as fused_pose.raw_true gives from a real RAW row): the
                # wheels' speed and turn (they read what the robot's own pose does; the wheels turn 2.5 % more than the
                # body: MEASURED 1.02-1.035 x the gyro), the raw gyro (true turn x (1 + scale error) + its bias), the
                # accelerometer (a jolt when hung), front and line sensors
                dt = 0.1
                w_own = wrap180(b.oth - getattr(b, "prev_oth", b.oth)) / dt
                w_true = wrap180(b.th - b.prev_th) / dt
                b.prev_oth, b.prev_th = b.oth, b.th
                b.gz_bias += b.rng.gauss(0, 0.0005)
                still = abs(b.v) < 0.05 and abs(w_own) < 0.5
                gz = 0.0 if still and abs(w_true) < 0.5 else w_true * (1 + b.gz_scale)
                gz += b.gz_bias + b.rng.gauss(0, 0.03)
                rev = b.cur is not None and b.cur.get("rev")      # b.v is a speed; a reverse drive goes back
                v = 0.0 if still else b.v * (-1.0 if rev else 1.0)
                ww = 0.0 if still else w_own * 1.025 + b.rng.gauss(0, 0.5)
                line = (900.0, 900.0, 900.0) if (self.edge_physics and getattr(b, "hung", False)) else (50.0, 50.0,
                                                                                                         50.0)
                ax = (0.5 if getattr(b, "hung", False) and not still else b.rng.gauss(0, 0.02))
                self.up(b, "rawfull", self.t, v, ww, gz, still, ax, self.front_cm(b), line)

    # -- (11-05d) firmware 2026-11-05c curves ---------------------------------------------------------------------------
    def move_truth(self, b, v, w):
        """Own pose and truth moved by a forward speed v and turn rate w (deg/s) for DT (wheel and gyro errors)."""
        b.oth = wrap180(b.oth + w * DT)
        b.th = wrap180(b.th + w * DT * (1 + b.gyro_err))
        hf = getattr(b, "hdg_off", None)
        if hf is not None:
            if self.t > hf[1]:
                b.hdg_off = None
            else:
                b.oth = wrap180(b.th + hf[0])       # (e10) its own heading stays off (the camera's fixes do not cure it)
        d = v * DT
        b.ox += d * math.cos(math.radians(b.oth))
        b.oy += d * math.sin(math.radians(b.oth))
        nx = b.x + d * (1 + b.wheel_err) * math.cos(math.radians(b.th))
        ny = b.y + d * (1 + b.wheel_err) * math.sin(math.radians(b.th))
        x0 = self.walls[0]
        if self.walls != self.board and nx < x0 + 4.75:
            self.stats["wall_hits"] += 1
            nx = x0 + 4.75
        if getattr(b, "hung", False):
            return                       # (e08 model) hung: the wheels turn, the body stays
        b.x, b.y = nx, ny
        self.edge_check(b)

    def open_side(self, b):
        th0 = b.th
        b.th = wrap180(th0 + 25)
        left = self.front_cm(b, lat_max=4.0)
        b.th = wrap180(th0 - 25)
        right = self.front_cm(b, lat_max=4.0)
        b.th = th0
        return 1 if left >= right else -1

    def inject_heading(self, b, c):
        """(e10) hdg_fault: Right's 7 Nov straight, reproduced (see __init__)."""
        hf = self.hdg_fault
        if not hf or abs(c["k"]) > 0.003 or c["L"] < 35.0 or b.dead:
            return
        if self.t < getattr(self, "hdg_next", 0.0) or getattr(b, "hdg_off", None) is not None:
            return
        deg = float(hf.get("deg", 25.0))
        best = None
        for off in (deg, -deg):
            th = math.radians(c["h"] - off)
            ex, ey = b.x + c["L"] * math.cos(th), b.y + c["L"] * math.sin(th)
            e = FC.edge_dist(self.board, ex, ey)
            if best is None or e < best[0]:
                best = (e, off)
        if best[0] > float(hf.get("near_cm", 14.0)):
            return
        self.hdg_next = self.t + float(hf.get("every_s", 60.0))
        c["hdg_off"] = best[1]
        b.hdg_off = (best[1], self.t + 30.0)
        b.oth = wrap180(b.th + best[1])
        self.stats["hdg_faults"] = self.stats.get("hdg_faults", 0) + 1
        self.hdg_log = getattr(self, "hdg_log", [])
        self.hdg_log.append(dict(t=round(self.t, 1), robot=b.name, off=best[1], end_edge_cm=round(best[0], 1)))

    def curve_step(self, b, s):
        """pose.h driveCurve (2026-11-05c): a piece of constant curvature on the own pose; no slowing at the end; the
        front check dodges (a tight curve away) instead of stopping."""
        if b.farc is not None:
            k, L = b.farc
            b.farc = None
            left = s["L"] - s["s"]
            s.update(x0=b.ox, y0=b.oy, h=b.oth, k=k, L=L if L else left, s=0.0, prev=None, t0=self.t, slack=0.0)
        s.setdefault("t0", self.t)
        s.setdefault("slack", 0.0)
        if b.scale < 1.0:
            s["slack"] += (1.0 - max(0.0, b.scale)) * DT
        if self.t - s["t0"] > 3.0 + s["L"] / max(s["v"], 1.0) * 2.5 + s["slack"]:
            b.v = 0.0                                   # firmware: "curve timed out", brake, the queue dropped
            b.report(self, s["id"], "failed", "curve timed out")
            for q in b.q:
                b.report(self, q["id"], "dropped", "an earlier step failed")
            b.q, b.cur, b.state, b.coast = [], None, "idle", None
            return
        k = s["k"]
        hr = math.radians(s["h"])
        if abs(k) < 1e-4:
            ux, uy = math.cos(hr), math.sin(hr)
            s["s"] = (b.ox - s["x0"]) * ux + (b.oy - s["y0"]) * uy
            cross = ux * (b.oy - s["y0"]) - uy * (b.ox - s["x0"])
            href, lean = s["h"], max(-20.0, min(20.0, -3.0 * cross))
            if s.get("hdg_off") is not None:
                lean = 0.0                          # (e10) as Right's straight: no steering back by position
            ff = 0.0
        else:
            sg, R = (1.0 if k > 0 else -1.0), 1.0 / abs(k)
            cx, cy = s["x0"] - math.sin(hr) * sg * R, s["y0"] + math.cos(hr) * sg * R
            phi = math.degrees(math.atan2(b.oy - cy, b.ox - cx))
            if s["prev"] is not None:
                s["s"] += sg * math.radians(wrap180(phi - s["prev"])) * R
            s["prev"] = phi
            e = math.hypot(b.ox - cx, b.oy - cy) - R
            href, lean = phi + sg * 90.0, sg * max(-20.0, min(20.0, 2.0 * e))
            ff = sg * math.degrees(b.v / R)
        rem = s["L"] - s["s"]
        if rem <= b.v * 0.03 + 0.05:
            b.report(self, s["id"], "done")
            b.cur = None
            b.coast = dict(t=self.t, k=k, v=max(2.0, b.v), dodge=False)
            b.state = "coast"
            if not (b.q and b.q[0]["type"] == "R"):
                self.coast_step(b)                 # rolls on in the same step (the firmware does not pause)
            return
        fc = self.front_cm(b)
        if self.phantoms > 0 and b.v > 0.5 and b.rng.random() < self.phantoms * DT / 3600.0 / 0.9:
            fc = 5.0                               # (e08) a false front reading (nothing there): the dodge path
            self.stats["phantom_stops"] += 1
        if fc < 8.0 and b.v > 0.5 and b.scale <= 0:
            b.v = 0.0
            b.report(self, s["id"], "failed", "obstacle %.1f cm ahead" % fc)
            for q in b.q:
                b.report(self, q["id"], "dropped", "an earlier step failed")
            b.q, b.cur, b.state, b.coast = [], None, "idle", None
            self.stats["front_stops"] += 1
            return
        if fc < 8.0 and b.v > 0.5:
            sd = self.open_side(b)
            b.coast = dict(t=self.t, k=sd * FC.DODGE_K, v=min(max(b.v, 2.0), 4.0), dodge=True)
            b.report(self, s["id"], "failed", "dodged obstacle %.1f cm ahead; curving %s at %.1f cm/s" % (
                fc, "left" if sd > 0 else "right", b.coast["v"]))
            for q in b.q:
                b.report(self, q["id"], "dropped", "an earlier step failed")
            b.q, b.cur, b.state = [], None, "dodge"
            self.stats["dodges"] = self.stats.get("dodges", 0) + 1
            return
        sc = max(0.0, min(1.5, b.scale))
        vt = min(s["v"] * sc, V_TOP)
        b.v = min(vt, b.v + 20.0 * DT + 0.3) if b.v < vt else max(vt, b.v - 20.0 * DT - 0.3)
        herr = wrap180(href + lean - b.oth)
        w = ff + max(-60.0, min(60.0, 4.0 * herr))
        self.move_truth(b, b.v, w)

    def coast_step(self, b):
        """follow.h coast: on round the last curve at its speed until the next R piece; stops after arc_coast_ms, for
        the watchdog, or with something 4 cm ahead. True while it coasts."""
        c = b.coast
        if b.q or (b.new_fw and self.t - b.last_rx > FC.WD_MS / 1000.0) or self.t - c["t"] > FC.COAST_S or \
                self.front_cm(b) < 4.0:
            b.coast, b.v, b.state = None, 0.0, "idle"
            return False
        b.state = "dodge" if c["dodge"] else "coast"
        b.v = c["v"]
        self.move_truth(b, b.v, math.degrees(c["k"] * b.v))
        return True

    def cam_gap(self, n, t):
        return any(a <= t < b for a, b in self.gaps[n])

    def camera(self):
        if getattr(self, "camera_off", False):
            return                              # (e10) the discovery found no camera it can trust: none at all
        if self.src is not None:            # (e09) sightings through the one tracking layer
            self.src.frame()
            return
        for n, b in self.bots.items():
            if self.t >= self.next_fix[n]:
                self.next_fix[n] = self.t + 1 / 12.0
                if self.cam_gap(n, self.t) or not self.in_cover(b.x, b.y):
                    continue
                lat = self.rng.uniform(0.12, 0.2) if self.cam_delay is None else \
                    max(0.05, self.cam_delay + self.rng.uniform(-0.1, 0.1))
                fx, fy = b.x + self.rng.gauss(0, 0.5), b.y + self.rng.gauss(0, 0.5)
                if self.cam_fault or self.cam_correct is not None:
                    fx, fy = self.cam_map(fx, fy)          # (e10) through the camera's calibration (and its fault)
                for nm, t0, t1, dx, dy in self.misplace:     # (e08) the camera puts it off for a while
                    if nm == n and t0 <= self.t < t1:
                        fx, fy = fx + dx, fy + dy
                fth = b.th + self.rng.gauss(0, 2.0) if (b.state == "drive" and self.rng.random() < 0.3) else None
                if fth is not None and (self.cam_fault or self.cam_correct is not None):
                    fth = self.cam_heading(fth, None)
                self.cam_q.append((self.t + lat, n, self.t, fx, fy, fth))
        due = [e for e in self.cam_q if e[0] <= self.t]
        if due:
            self.cam_q = [e for e in self.cam_q if e[0] > self.t]
            for _, n, tf, fx, fy, fth in sorted(due):
                self.ctrl.on_fix(n, tf + self.cam_clock, fx, fy, fth, arrival=self.t)
                # the runner forwards each fix to the robot (fix command, stamped with the picture's robot time) unless
                # the controller has it lost: the robot's fusion pulls its pose over
                if self.t >= self.frozen_until and self.ctrl.R[n].lost is None:
                    self.ctrl.sent(n)
                    self.down.send(self.t, self.fuse, n, fx, fy, fth, tf)

    def fuse(self, n, fx, fy, fth, tf=None):
        b = self.bots[n]
        if b.dead or getattr(b, "mute", False):
            return
        b.last_rx = self.t                  # any packet keeps its watchdog quiet (follow.h), a refused fix too
        if any(nm == n and t0 <= self.t < t0 + dur for nm, t0, dx, dy, dur in self.own_off):
            return                          # (e08) its fixes refused while its own pose is off
        hx, hy = b.ox, b.oy
        if tf is not None:                  # the firmware weighs a fix against its own pose at the fix's time
            if self.t - tf > 1.0:           # params.h fix_max_age_ms 1000: older fixes are not used
                return
            older = [q for q in b.hist if q[0] <= tf]
            if not older:                   # no own pose that old (just set by fpose): the fix is not used
                return
            hx, hy = older[-1][1], older[-1][2]
        dx, dy = 0.35 * (fx - hx), 0.35 * (fy - hy)
        b.ox += dx
        b.oy += dy
        b.hist = [(t0, x0 + dx, y0 + dy) for t0, x0, y0 in b.hist]   # pose.h fix(): the whole history moves too
        if fth is not None:
            b.oth = wrap180(b.oth + 0.3 * wrap180(fth - b.oth))
        # the camera knows where it is: the own pose's heading error is pulled in slowly too (fixes along the line)
        b.oth = wrap180(b.oth + 0.02 * wrap180(b.th - b.oth))

    def follow_records(self):
        for n, b in self.bots.items():
            if self.t >= self.next_follow[n] and not b.dead and b.following:
                self.next_follow[n] = self.t + 0.1
                b.hist.append((self.t, b.ox, b.oy))
                if len(b.hist) > 40:
                    del b.hist[:10]
                self.up(b, "follow", b.state, b.ox, b.oy, b.oth, b.cur["id"] if b.cur else 0, b.last_done,
                        b.last_failed)

    def speed_truth(self):
        k = 0
        for n, b in self.bots.items():
            v = math.hypot(b.x - b.tx, b.y - b.ty) / DT
            b.tx, b.ty = b.x, b.y
            r = self.ctrl.R[n]
            sp = self.speed[n]
            if b.dead or r.mode in ("out", "gone", "start"):
                sp["zero_run"] = 0.0
                continue
            sp["alive_s"] += DT
            if v >= 0.3:
                sp["moving_s"] += DT
                k += 1
                if sp["zero_run"] >= 0.1:
                    self.zero_moment(n, r)
                sp["zero_run"] = 0.0
            else:
                sp["zero_s"] += DT
                sp["zero_run"] += DT
        self.n_moving[min(3, k)] += DT

    def zero_moment(self, n, r):
        sp = self.speed[n]
        sp["zeros"] += 1
        zw = r.zero_why
        why = zw[0] if zw and self.t - zw[1] <= 3.0 + sp["zero_run"] else (
            "stop-turn (turns on the spot)" if not r.curve else "unexplained")
        key = why.split(":")[0].split(" (")[0]
        sp["why"][key] = sp["why"].get(key, 0) + 1
        if len(self.__dict__.setdefault("zero_rows", [])) < 400:
            self.zero_rows.append(dict(t=round(self.t, 2), robot=n, dur_s=round(sp["zero_run"], 2), why=why,
                                       state=self.bots[n].state, kind=(r.move or {}).get("kind")))

    def pos_errors(self):
        """(e09) every 0.5 s: how far the controller's position for each robot is from the truth (and on the far side of
        the tag-readable area, when the scene has one)."""
        if self.t - getattr(self, "_pe_t", -1.0) < 0.5:
            return
        self._pe_t = self.t
        pe = self.__dict__.setdefault("pe", {n: [] for n in self.names})
        far = (self.src.scene.get("tag_x_max") if self.src is not None else None)
        for n, b in self.bots.items():
            r = self.ctrl.R[n]
            if b.dead or r.mode in ("out", "gone", "start"):
                continue
            p = self.ctrl.pos(r)
            if p is None:
                continue
            e = round(math.hypot(p[0] - b.x, p[1] - b.y), 2)
            pe[n].append((e, int(far is not None and b.x > far), round(self.ctrl.unc(r), 2)))
            self.__dict__.setdefault("pe_t", {}).setdefault(n, []).append((round(self.t, 2), e))

    def points_reached(self):
        """(e09, rehearsal a) points of a 30 cm grid (stickers) a robot's true middle came within 5 cm of."""
        pts = getattr(self, "points", None)
        if not pts or self.t - getattr(self, "_pr_t", -1.0) < 0.25:
            return
        self._pr_t = self.t
        got = self.__dict__.setdefault("reached", set())
        for b in self.bots.values():
            for k, (px, py) in enumerate(pts):
                if k not in got and abs(b.x - px) < 5.0 and abs(b.y - py) < 5.0 and math.hypot(b.x - px, b.y - py) < 5.0:
                    got.add(k)

    def truth(self):
        self.points_reached()
        self.pos_errors()
        self.speed_truth()
        bs = list(self.bots.values())
        for i in range(len(bs)):
            a = bs[i]
            e = FC.edge_dist(self.board, a.x, a.y) - FC.HALF
            self.stats["edge_min"] = min(self.stats["edge_min"], e + FC.HALF - FC.BODY / 2 * 1.0)
            if not a.dead:
                ec = FC.edge_dist(self.board, a.x, a.y)
                eb = min(FC.edge_dist(self.board, *q) for q in corners(a.x, a.y, a.th))
                if ec < self.stats["edge_min_centre"]:
                    self.stats["edge_min_centre"] = ec
                if eb < self.stats["edge_min_body"]:
                    self.stats["edge_min_body"] = eb
                    self.stats["edge_min_at"] = dict(t=round(self.t, 1), robot=a.name, x=round(a.x, 1), y=round(a.y, 1))
                if getattr(a, "hung", False):
                    self.stats["hung_s"] += DT
            if FC.edge_dist(self.board, a.x, a.y) < FC.BODY / 2:
                self.stats["off_board"] += 1
            for j in range(i + 1, len(bs)):
                b = bs[j]
                g = body_gap((a.x, a.y, a.th), (b.x, b.y, b.th))
                key = (a.name, b.name)
                self.win_min[key] = min(self.win_min.get(key, float("inf")), g)
                if g < self.stats["min_gap"]:
                    self.stats["min_gap"] = g
                    self.stats["min_gap_at"] = dict(t=round(self.t, 2), a=a.name, b=b.name, a_state=a.state,
                                                    b_state=b.state, a_dead=a.dead, b_dead=b.dead,
                                                    a_mode=self.ctrl.R[a.name].mode, b_mode=self.ctrl.R[b.name].mode,
                                                    a_lost=self.ctrl.R[a.name].lost is not None,
                                                    b_lost=self.ctrl.R[b.name].lost is not None,
                                                    a_kind=(self.ctrl.R[a.name].move or {}).get("kind"),
                                                    b_kind=(self.ctrl.R[b.name].move or {}).get("kind"))
                if g <= 0.0:
                    self.stats["contact_s"] += DT
                    if key not in self.in_contact:
                        self.in_contact.add(key)
                        self.stats["contacts"] += 1
                        self.keep("contacts", dict(t=round(self.t, 2), a=a.name, b=b.name, ax=round(a.x, 1),
                                                   ay=round(a.y, 1), bx=round(b.x, 1), by=round(b.y, 1)))
                elif g > 0.5:
                    self.in_contact.discard(key)
        if self.t - self.win_t >= 10.0:
            for k, v in self.win_min.items():
                self.stats["gap_windows"].append(v)
            self.win_min, self.win_t = {}, self.t

    def discover(self):
        """(e10, C1) The start's proof drive in this world (start_discovery.ProofDriver over the same robots, network and
        camera), then what the analysis found is applied as the Pi would: the camera's calibration (or none), the
        board's size, the clock, turned tags, each robot's wheel / gyro fit (corr save), robots not seen kept parked,
        the proven workspace re-made, each robot's own pose set from the corrected camera (fpose)."""
        import start_discovery as D
        c = self.ctrl
        tags = (self._scene0 or {}).get("tracking") == "tags"
        drv = D.ProofDriver(self.names, lambda n, txt: self.down.send(self.t, self.robot_cmd, n, txt), t0=self.t,
                            board=self.believed if c.start_edge else None, edge_cm=FC.START_EDGE_CM)
        self._disc_drv = drv
        q, next_pic = [], self.t
        sc = self._scene0 or {}
        none = sc.get("fault") == "none" and not sc.get("fault_t")
        dark = self.blind_rects()
        while not drv.done() and self.t < 90.0:
            self.t += DT
            for b in self.bots.values():
                self.bot_step(b)
            self.follow_records()
            if self.t >= next_pic and not none:
                next_pic = self.t + 1.0 / 12.0
                for n, b in self.bots.items():
                    if b.dead or not self.in_cover(b.x, b.y):
                        continue
                    if any(r[0] <= b.x <= r[2] and r[1] <= b.y <= r[3] for r in dark) and self.rng.random() < 0.7:
                        continue
                    sg = 0.4 if tags else 0.7
                    x, y = self.cam_map(b.x + self.rng.gauss(0, sg), b.y + self.rng.gauss(0, sg))
                    th = self.cam_heading(b.th + self.rng.gauss(0, 1.0), n) if tags else None
                    q.append((self.t + self.rng.uniform(0.12, 0.2), n, self.t + self.cam_clock, x, y, th))
            due = [e for e in q if e[0] <= self.t]
            if due:
                q = [e for e in q if e[0] > self.t]
                for _, n, ts, x, y, th in due:
                    drv.on_sighting(n, ts, x, y, th, "tags" if tags else "camera")
            for _, fn, args in self.down.due(self.t):
                fn(*args)
            for _, fn, args in self.upn.due(self.t):
                fn(*args)
            drv.tick(self.t)
            self.truth()
            self.visit()
        self._disc_drv = None
        for n, dd in drv.data().items():
            c.R[n].next_id = max(c.R[n].next_id, dd["last_id"] + 1)     # the robot takes only higher ids
        cands = {"mirrored": D.mirror_x(self.believed)}
        res = D.analyse(drv.data(), self.believed, cands)
        self.disc = res
        self.disc_t = round(self.t, 1)
        c.event(None, "discovery", " ".join(res["say"]))
        if res["camera"] in ("missing", "wrong", "unproven"):
            self.camera_off = True               # the run narrows: no camera fixes at all
        else:
            self.cam_correct = res["fix"]
            self.cam_correct_mirror = res["calibration"] == "mirrored"
            if res.get("k_used", 1.0) != 1.0:
                self.believed = tuple(res["board_found"])
            for n, yo in res["tag_yaw"].items():
                self.tag_yaw_fix[n] = yo
            if res.get("clock_s") is not None and abs(res["clock_s"]) >= D.SKEW_MIN_S:
                c.clock = float(res["clock_s"])
                c.clock_ests = [c.clock] * 5
                c.clock_seen = c.clock
            for n, rb in res["robots"].items():
                b = self.bots[n]
                b.ox, b.oy, b.oth = rb["x"], rb["y"], rb["heading"]       # fpose from the corrected camera
                b.hist = []
            for n, f in res["fits"].items():
                b = self.bots[n]
                w, g = f.get("wheel"), f.get("gyro")
                if w and abs(w - 1.0) > 2 * f["wheel_se"] and abs(w - 1.0) <= D.CAL_MAX_DEV:
                    b.wheel_err = (1.0 + b.wheel_err) / w - 1.0               # corr: its wheel_scale x the fit
                if g and abs(g - 1.0) > 2 * f["gyro_se"] and abs(g - 1.0) <= D.CAL_MAX_DEV:
                    b.gyro_err = (1.0 + b.gyro_err) / g - 1.0
        self.unseen = [n for n in res["unseen"]]
        if self.ws is not None:
            self.ws = self.make_ws(self.believed)
            c.ws, c.rect = self.ws, self.ws
        else:
            c.rect = tuple(self.believed)
        return res

    def run(self):
        c = self.ctrl
        T = self.hours * 3600.0
        if self.discover_on:
            self.discover()
        t0 = self.t                              # (e10) after the proof drive, if there was one
        realcam = bool(self.cam_fault) or self.cam_correct is not None
        for n, b in self.bots.items():
            c.on_fw(n, b.fw)
            c.on_battery(n, 100, t0)
            c.on_follow(n, t0, "idle", b.ox, b.oy, b.oth, 0)
            fx, fy = self.cam_map(b.x, b.y) if realcam else (b.x, b.y)
            if not getattr(self, "camera_off", False):
                c.on_fix(n, t0 + self.cam_clock, fx, fy, b.th, arrival=t0 + (self.cam_delay or 0.15))
            c.on_heard(n, t0)
        c.tick(t0)
        if self.src is not None and not getattr(self, "camera_off", False):
            for n, b in self.bots.items():
                fx, fy = self.cam_map(b.x, b.y) if realcam else (b.x, b.y)
                self.src.layer.seed(n, t0, fx, fy, "start")      # where the start found it (lights / tags)
        if hasattr(c, "fix_fn"):
            # (e09) the filter's position to the robot in camera gaps (the runner's fix command, its own time)
            c.fix_fn = lambda n, t, x, y: self.down.send(self.t, self.fuse, n, x, y, None, self.t)
        if self.sensors:
            # (e09) the robots stand a second before the run (the compass check and the start's find, as on the Pi):
            # their raw gyro's bias is taken there
            for k_ in range(10):
                for n, b in self.bots.items():
                    c.on_raw_full(n, -1.0 + 0.1 * k_, 0.0, 0.0, b.gz_bias + b.rng.gauss(0, 0.03), True)
        if self.edge_physics:
            # (e08) each robot's level, taken standing on the board before the run (the firmware's compass check)
            for n in self.names:
                for k_ in range(30):
                    c.on_raw(n, -1.0 + 0.02 * k_, 0.0, 0.0, 0.0)
        for n in self.names:
            k = self.held0.get(n)
            if k == "out":                      # (e08 "before") the old way: a failed heading crawl kept it out
                c.begin(n)
                c.R[n].mode, c.R[n].out_why = "out", "no heading (the old way)"
                c.R[n].traj = FC.Traj.stand(0.0, self.bots[n].x, self.bots[n].y, m=FC.PLAN_M0 + 1.0)
            elif k:
                c.begin(n, held=k)
            else:
                c.begin(n)
        for n in getattr(self, "unseen", ()):   # (e10) the start's discovery did not see it: it stays parked, named
            r_ = c.R[n]
            r_.mode, r_.out_why = "out", "not seen by the camera at the start (out of its view)"
            r_.traj = FC.Traj.stand(self.t, self.bots[n].ox, self.bots[n].oy, m=FC.PLAN_M0 + 1.0)
        for b in self.bots.values():           # (e08) a robot placed hung at the start (rehearsal b)
            if self.edge_physics and any(FC.edge_dist(self.board, *w) < 0.0 for w in self.wheels(b)):
                b.hung = True
                self.stats["hangs"] += 1
        next_tick = 0.0
        while self.t < T:
            self.t += DT
            if self.nopic and self.nopic_said == 0 and self.t >= self.nopic[0] + 0.5:
                self.nopic_said = 1                 # (e13) the frame watch notices after its own gap limit
                c.on_camera_state("no_pictures", self.t)
            elif self.nopic and self.nopic_said == 1 and self.t >= self.nopic[0] + self.nopic[1]:
                self.nopic_said = 2
                c.on_camera_state("pictures", self.t)
            if self.freeze_at is not None and self.t >= self.freeze_at:
                self.frozen_until = self.t + 4.0
                c.event(None, "pi frozen (sim)", "the controller sends nothing for 4 s")
                self.freeze_at = None
            if self.dead_at is not None and self.t >= self.dead_at[0]:
                b = self.bots[self.dead_at[1]]
                b.dead, b.v, b.state = True, 0.0, "idle"
                c.event(c.R[b.name], "dead (sim)", "stops answering where it is")
                self.dead_at = None
            for (nm, t0, dx, dy, dur) in self.own_off:      # (e08) its own pose jumps off the truth
                if t0 <= self.t < t0 + DT:
                    b = self.bots[nm]
                    if dx is None:          # "in": dy cm towards the board's middle (it believes it is further in)
                        mx, my = (self.board[0] + self.board[2]) / 2 - b.x, (self.board[1] + self.board[3]) / 2 - b.y
                        L_ = math.hypot(mx, my) or 1.0
                        dx, dy = dy * mx / L_, dy * my / L_
                    b.ox, b.oy = b.ox + dx, b.oy + dy
                    b.hist = [(a, x + dx, y + dy) for a, x, y in b.hist]
                    c.event(c.R[nm], "own pose off (sim)", "%.1f, %.1f cm for %.0f s, fixes refused" % (dx, dy, dur))
            for (nm, t0, dur) in self.silent_inj:           # (e08) deaf and mute, then back
                b = self.bots[nm]
                was = getattr(b, "mute", False)
                b.mute = t0 <= self.t < t0 + dur
                if b.mute != was:
                    c.event(c.R[nm], "silent (sim)" if b.mute else "back (sim)", "%.0f s" % dur)
            for (nm, t0, dx, dy) in self.pushes:           # (e09) a parked robot pushed (truth only)
                if t0 <= self.t < t0 + DT:
                    b = self.bots[nm]
                    b.x, b.y = b.x + dx, b.y + dy
                    self.ctrl.event(self.ctrl.R[nm], "pushed (sim)", "%.1f, %.1f cm" % (dx, dy))
            for b in self.bots.values():
                self.bot_step(b)
            self.raw()
            self.camera()
            self.follow_records()
            for _, fn, args in self.down.due(self.t):
                fn(*args)
            for _, fn, args in self.upn.due(self.t):
                fn(*args)
            if self.t >= next_tick:
                next_tick = self.t + TICK
                if self.t >= self.frozen_until:
                    c.tick(self.t)
            self.truth()
            self.visit()
            if c.all_out():
                break
        for f in getattr(self, "files", []):
            f.close()
        return self.summary()

    def summary(self):
        c, st = self.ctrl, self.stats
        h = self.t / 3600.0
        kept = getattr(self, "kept", {})
        refl = kept.get("reflex", [])
        gw = sorted(st["gap_windows"])

        def pct(q):
            return round(gw[min(len(gw) - 1, int(q * len(gw)))], 2) if gw else None
        mot = None
        m = getattr(self, "mot", {})
        if m:
            n = len(m)
            mot = dict(any_ge2=round(100.0 * sum(1 for v in m.values() if v[0] >= 2) / n, 1),
                       any_3=round(100.0 * sum(1 for v in m.values() if v[0] >= 3) / n, 1),
                       trans_ge2=round(100.0 * sum(1 for v in m.values() if v[1] >= 2) / n, 1),
                       trans_3=round(100.0 * sum(1 for v in m.values() if v[1] >= 3) / n, 1),
                       robot={k: (round(100.0 * v[0] / max(1, v[2]), 1), round(100.0 * v[1] / max(1, v[2]), 1))
                              for k, v in getattr(self, "mot_robot", {}).items()})
        hist = [0] * 41                          # 10 s windows by their smallest body gap: 1 cm bins, 40+ last
        for g in gw:
            hist[min(40, int(g))] += 1
        tot = sum(self.n_moving) or 1.0
        speed = dict(robot={n: dict(moving_pct=round(100.0 * v["moving_s"] / max(1e-6, v["alive_s"]), 1),
                                    zeros=v["zeros"], zeros_per_h=round(v["zeros"] / max(1e-6, v["alive_s"] / 3600), 1),
                                    why=v["why"], style="curve" if c.R[n].curve else "stopturn")
                             for n, v in self.speed.items()},
                     ge2_pct=round(100.0 * (self.n_moving[2] + self.n_moving[3]) / tot, 1),
                     all3_pct=round(100.0 * self.n_moving[3] / tot, 1),
                     zeros=sum(v["zeros"] for v in self.speed.values()),
                     zeros_per_h=round(sum(v["zeros"] for v in self.speed.values()) / max(1e-6, h), 1),
                     why=Counter_sum(v["why"] for v in self.speed.values()))
        return dict(seed=self.seed, hours=round(h, 3), gap_hist=hist, wall_hits=st["wall_hits"], speed=speed,
                    dodges=c.counts["dodges"], pieces=c.counts["pieces"], loops=c.counts["loops"],
                    motion_style=c.motion_style, v_min=c.v_min,
                    front_loops=st["front_loops"], keepouts=len(c.keepouts), backoffs=c.counts["backoffs"],
                    cam_refused=c.counts["cam_refused"], cam_delay_s=round(c.cam_delay, 3), clock_s=round(c.clock, 3),
                    clock_seen_s=c.clock_seen, longest_still_s=self.longest_still(), contacts=st["contacts"], contact_s=round(st["contact_s"], 2),
                    min_gap_cm=round(st["min_gap"], 2), min_gap_at=st.get("min_gap_at"), gap_p1=pct(0.01), gap_p5=pct(0.05), gap_p50=pct(0.5),
                    reflex=dict(c.counts["reflex"]), fallbacks=dict(c.counts["fallback"]),
                    trophy_breaks=c.counts["breaks"], front_stops=st["front_stops"],
                    watchdog_stops=st["watchdog_stops"], moves=sum(r.n_moves for r in c.R.values()),
                    out={n: (r.out_why, round(r.battery or 0)) for n, r in c.R.items()},
                    edge_min_cm=round(st["edge_min"], 2), off_board=st["off_board"],
                    lost_episodes=len(kept.get("lost", [])),
                    slow_moves=sum(1 for e in kept.get("events", []) if e["kind"] == "slow move"),
                    lost_reports=sum(1 for e in kept.get("events", []) if e["kind"] == "lost report"),
                    break_why=Counter_sum({e["why"].split(":")[0]: 1} for e in kept.get("breaks", [])),
                    reflex_outcomes={k: sum(1 for e in refl if e["outcome"] == k) for k in set(e["outcome"] for e in refl)},
                    menu_cells_used=sum(1 for v in c.menu.n.values() if v), menu_min=min(c.menu.n.values()),
                    menu_max=max(c.menu.n.values()), motion=mot,
                    # (e08) the Phase A rehearsals' measures
                    edge_min_body_cm=round(st["edge_min_body"], 2), edge_min_centre_cm=round(st["edge_min_centre"], 2),
                    edge_min_at=st.get("edge_min_at"), hangs=st["hangs"], falls=st["falls"], hung_s=round(st["hung_s"], 1),
                    edge_trips=st["edge_trips"], phantom_stops=st["phantom_stops"],
                    phase_a={k: c.counts.get(k) for k in ("edge_refused", "pose_doubts", "reseeds", "off_plan",
                                                           "judgements", "unsure_inward", "unsure_holds", "questions",
                                                           "recover_tries", "recovered", "recover_holds", "held",
                                                           "rejoined", "keepouts_faded", "keepouts", "tilt_trips",
                                                           "tilt_hazards", "border_fires", "surface_marks", "heading",
                                                           "leads_used")},
                    keepouts_end=len(c.keepouts),
                    held_end={n: (r.held or {}).get("kind") for n, r in c.R.items() if getattr(r, "held", None)},
                    moving_pct={n: v["moving_pct"] for n, v in speed["robot"].items()},
                    phase_b=self.phase_b(), phase_c=self.phase_c(),
                    # (e10) the 7 Nov run 2 incident's measures
                    incident=dict({k: c.counts.get(k, 0) for k in ("queue_watch", "reaimed", "queue_heading_resets",
                                                                    "marks_deferred", "start_held", "dropped_started", "start_recover", "start_freed",
                                                                    "surface_marks", "off_plan")},
                                  hdg_faults=st.get("hdg_faults", 0), hdg_fault_cured=st.get("hdg_fault_cured", 0),
                                  bottom_trips=st.get("bottom_trips", 0), log=getattr(self, "hdg_log", [])[:20]))

    def phase_c(self):
        """(e10) the Phase C measures: what the start's discovery found, the proven workspace, where the robots went."""
        d = self.disc
        out = dict(discovery=None if d is None else {k: v for k, v in d.items() if k != "fix"},
                   disc_s=getattr(self, "disc_t", None), workspace=self.ws.summary() if self.ws is not None else None,
                   counts={k: self.ctrl.counts.get(k) for k in ("probes", "empty_moves", "pictures_skipped",
                                                                "pictures")},
                   fallbacks=dict(self.ctrl.counts.get("fallback", {})), bottom_trips=self.stats.get("bottom_trips", 0),
                   visits={"%d,%d" % k: round(v, 1) for k, v in self.visits.items()},
                   board=list(self.board), believed=list(self.believed),
                   wheel_err={n: round(b.wheel_err, 4) for n, b in self.bots.items()},
                   gyro_err={n: round(b.gyro_err, 4) for n, b in self.bots.items()})
        if self.cover:
            tot = sum(v for v in self.visits.values())
            outside = sum(v for (i, j), v in self.visits.items() if not self.in_cover(i * 5 + 2.5, j * 5 + 2.5))
            out["unseen_visit_pct"] = round(100.0 * outside / max(1e-6, tot), 1)
        return out

    def phase_b(self):
        """(e09) the Phase B measures: tracking (% of pictures each robot had a fix, on the far side too; wrong fixes and
        name swaps against the truth), position error (the controller's estimate against the truth), the filter."""
        import statistics as STS
        c = self.ctrl
        pe = getattr(self, "pe", {})
        err = {}
        for n, rows in pe.items():
            if not rows:
                continue
            e = sorted(r[0] for r in rows)
            ef = sorted(r[0] for r in rows if r[1])
            cov = sum(1 for r in rows if r[0] <= r[2]) / len(rows)
            err[n] = dict(median=round(STS.median(e), 2), p95=e[min(len(e) - 1, int(0.95 * len(e)))], max=e[-1],
                          far_median=round(STS.median(ef), 2) if ef else None,
                          far_p95=ef[min(len(ef) - 1, int(0.95 * len(ef)))] if ef else None,
                          covered_pct=round(100.0 * cov, 1))
        out = dict(pose_source=c.pose_source, gyro_heading=c.gyro_heading, camera=self.camera_mode,
                   pos_err=err, counts={k: c.counts.get(k) for k in (
                       "ekf_disagree", "gap_forwards", "gyro_heading_fixes", "identity_wiggles", "camera_objects",
                       "parked_moved", "pictures")},
                   filters={n: (r.fp.summary() if r.fp is not None else None) for n, r in c.R.items()})
        if self.src is not None:
            out["tracking"] = self.src.summary()
        pts = getattr(self, "points", None)
        if pts:
            got = getattr(self, "reached", set())
            fx = self.src.scene.get("tag_x_max") if self.src is not None else None
            out["points"] = dict(n=len(pts), reached=len(got),
                                 far_n=sum(1 for p in pts if fx is not None and p[0] > fx),
                                 far_reached=sum(1 for k in got if fx is not None and pts[k][0] > fx))
        return out


def _longest(rows):
    out = {}
    for n in sorted(set(r["robot"] for r in rows)):
        best = cur = 0.0
        prev = None
        for r in rows:
            if r["robot"] != n:
                continue
            if r["any"] == 0 and r["mode"] not in ("out", "gone"):
                cur += (r["t"] - prev) if prev is not None else 0.0
                best = max(best, cur)
            else:
                cur = 0.0
            prev = r["t"]
        out[n] = round(best, 1)
    return out


def summarize(files, png=None):
    """The stress runs' results per group (one .jsonl per group), per simulated hour, as a markdown table; png: the
    distribution of the smallest body gap per 10 s window, all runs together."""
    rows, hist = [], [0] * 41
    for fp in files:
        rs = [json.loads(l) for l in open(fp) if l.startswith("{")]
        if not rs:
            continue
        H = sum(r["hours"] for r in rs)
        tot = lambda f: sum(f(r) for r in rs)      # noqa: E731
        for r in rs:
            for k, v in enumerate(r.get("gap_hist") or []):
                hist[k] += v
        mo = [r["motion"] for r in rs if r.get("motion")]
        rows.append(dict(
            group=os.path.basename(fp).rsplit(".", 1)[0], runs=len(rs), hours=round(H, 1),
            contacts=tot(lambda r: r["contacts"]), min_gap=min(r["min_gap_cm"] for r in rs),
            p1=min(r["gap_p1"] for r in rs if r["gap_p1"] is not None),
            speed=tot(lambda r: r["reflex"].get("speed", 0)) / H, steer=tot(lambda r: r["reflex"].get("steer", 0)) / H,
            replan=tot(lambda r: r["reflex"].get("replan", 0)) / H, stop=tot(lambda r: r["reflex"].get("stop", 0)) / H,
            breaks=tot(lambda r: r["trophy_breaks"]) / H,
            fallbacks=tot(lambda r: sum(v for k, v in r["fallbacks"].items() if k != "find-me wiggle")) / H,
            wiggles=tot(lambda r: r["fallbacks"].get("find-me wiggle", 0)) / H,
            front=tot(lambda r: r["front_stops"]) / H, watchdog=tot(lambda r: r["watchdog_stops"]),
            off_board=tot(lambda r: r["off_board"]), edge_min=min(r["edge_min_cm"] for r in rs),
            any2=sum(m["any_ge2"] for m in mo) / max(1, len(mo)), any3=sum(m["any_3"] for m in mo) / max(1, len(mo)),
            tr2=sum(m["trans_ge2"] for m in mo) / max(1, len(mo)), tr3=sum(m["trans_3"] for m in mo) / max(1, len(mo)),
            moves=tot(lambda r: r["moves"]) / H, slow=tot(lambda r: r.get("slow_moves", 0)) / H, outcomes=Counter_sum(r.get("reflex_outcomes", {}) for r in rs)))
    L = ["| group | runs | sim h | contacts | smallest gap cm | 1st pct of 10 s minima cm | speed /h | steer /h | "
         "new move /h | hard stop /h | full stops /h | fallbacks /h | find-me /h | front stops /h | watchdog stops | "
         "off board | closest to edge cm | >=2 moving % (driving %) | 3 moving % (driving %) | moves /h | slow moves /h |",
         "|" + "---|" * 21]
    for r in rows:
        L.append("| %s | %d | %.1f | %d | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.1f | %.1f | %.2f | %d | %d "
                 "| %.1f | %.1f (%.1f) | %.1f (%.1f) | %.0f | %.2f |" % (
                     r["group"], r["runs"], r["hours"], r["contacts"], r["min_gap"], r["p1"], r["speed"], r["steer"],
                     r["replan"], r["stop"], r["breaks"], r["fallbacks"], r["wiggles"], r["front"], r["watchdog"],
                     r["off_board"], r["edge_min"], r["any2"], r["tr2"], r["any3"], r["tr3"], r["moves"], r["slow"]))
    if png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 3.4))
        ax.bar(range(41), hist, color="#2952CC")
        ax.set_yscale("log")
        ax.set_xlabel("smallest gap between two robots' bodies in a 10 s window (cm; 40 = 40 or more)")
        ax.set_ylabel("10 s windows (log scale)")
        ax.set_title("How close the robots came: %d windows, %.0f simulated hours" % (
            sum(hist), sum(r["hours"] for r in rows)))
        fig.text(0.01, 0.01, "Measured: the true body gap (exact square-to-square distance) in the stress simulator, "
                 "every pair, every 0.05 s; source: tools/fleet/freeroam_fastsim.py runs, logs/freeroam-stress",
                 fontsize=7, color="#555")
        fig.tight_layout(rect=(0, 0.05, 1, 1))
        fig.savefig(png, dpi=110)
        plt.close(fig)
    return "\n".join(L), rows, hist


def Counter_sum(ds):
    out = {}
    for d in ds:
        for k, v in d.items():
            out[k] = out.get(k, 0) + v
    return out


def main(argv=None):
    if argv is None and len(sys.argv) > 1 and sys.argv[1] == "summarize":
        png = None
        fs = [f for f in sys.argv[2:] if not f.startswith("--png=")]
        for f in sys.argv[2:]:
            if f.startswith("--png="):
                png = f[6:]
        print(summarize(fs, png)[0])
        return 0
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hours", type=float, default=1.0)
    ap.add_argument("--seeds", default="1")
    ap.add_argument("--out", default="")
    ap.add_argument("--old-fw", action="store_true", help="robots on 2026-10-29b: no fspd / fsteer / watchdog")
    ap.add_argument("--loss", type=float, default=PACKET_LOSS)
    ap.add_argument("--lat-max-ms", type=float, default=40.0, help="each packet 5 ms to this late (default 40)")
    ap.add_argument("--drill", type=float, default=0.0, help="share of moves planned ignoring the others (reflex drill)")
    ap.add_argument("--no-dead", action="store_true")
    ap.add_argument("--no-freeze", action="store_true")
    ap.add_argument("--no-dropouts", action="store_true")
    ap.add_argument("--lost-mode", default="reckon", choices=("reckon", "hold"))
    ap.add_argument("--cam-delay", type=float, default=None, help="camera fixes this many s late (+-0.1); default "
                                                                   "0.12-0.2 s")
    ap.add_argument("--cam-clock", type=float, default=0.0, help="the camera's stamps this many s ahead")
    ap.add_argument("--wall-in", type=float, default=0.0, help="a real wall this far inside the board's x_min edge")
    ap.add_argument("--trap", action="store_true", help="Right starts 7 cm from an edge facing out")
    ap.add_argument("--fw", default=None, help="the robots' firmware, e.g. 2026-10-28a (default: 11-05b, or "
                                               "10-29b with --old-fw)")
    ap.add_argument("--motion", default="curves", choices=("curves", "stopturn"),
                    help="(11-05d) motion style (curves need firmware 2026-11-05c: --fw 2026-11-05c)")
    ap.add_argument("--fw-mix", default="", help="each robot's firmware, e.g. Left=2026-11-05c,Right=2026-10-28a")
    ap.add_argument("--v-min", type=float, default=None, help="curves: the slowest an avoiding robot goes (cm/s)")
    # (e08 Phase A)
    ap.add_argument("--edge-physics", action="store_true", help="a wheel over the edge hangs the robot (chosen model)")
    ap.add_argument("--own-off", action="append", default=[], metavar="NAME@T:DX,DY,DUR")
    ap.add_argument("--silent", action="append", default=[], metavar="NAME@T:DUR")
    ap.add_argument("--held", action="append", default=[], metavar="NAME=KIND")
    ap.add_argument("--cam-misplace", action="append", default=[], metavar="NAME@T0:T1:DX,DY")
    ap.add_argument("--phantoms", type=float, default=0.0, help="false front stops per robot-hour")
    ap.add_argument("--edge-rule", default=None, choices=("derived", "fixed"))
    ap.add_argument("--pose-rule", default=None, choices=("widen", "wait2s"))
    ap.add_argument("--keepout-rule", default=None, choices=("fade", "permanent"))
    ap.add_argument("--silent-out", action="store_true")
    ap.add_argument("--no-tilt-recovery", action="store_true")
    ap.add_argument("--attended", action="store_true")
    a = ap.parse_args(argv)

    def at(spec):
        n, rest = spec.split("@", 1)
        return n, rest
    own_off = []
    for x in a.own_off:
        n, rest = at(x)
        t, v = rest.split(":")
        q = v.split(",")
        own_off.append((n, float(t), None if q[0] == "in" else float(q[0]), float(q[1]), float(q[2])))
    silent = [(at(x)[0], float(at(x)[1].split(":")[0]), float(at(x)[1].split(":")[1])) for x in a.silent]
    misplace = []
    for x in a.cam_misplace:
        n, rest = at(x)
        t0, t1, v = rest.split(":")
        dx, dy = (float(q) for q in v.split(","))
        misplace.append((n, float(t0), float(t1), dx, dy))
    ckw = dict(edge_rule=a.edge_rule, pose_rule=a.pose_rule, keepout_rule=a.keepout_rule, silent_out=a.silent_out,
               tilt_recovery=not a.no_tilt_recovery, attended=a.attended)
    lo, _, hi = a.seeds.partition("-")
    seeds = range(int(lo), int(hi or lo) + 1)
    res = []
    for sd in seeds:
        od = None
        if a.out:
            od = os.path.join(a.out, "seed%02d%s" % (sd, "-oldfw" if a.old_fw else ""))
            os.makedirs(od, exist_ok=True)
        t0 = time.time()
        s = Sim(sd, a.hours, new_fw=not a.old_fw, dead_robot=not a.no_dead, pi_freeze=not a.no_freeze,
                dropouts=not a.no_dropouts, loss=a.loss, lost_mode=a.lost_mode, out_dir=od,
                lat_max=a.lat_max_ms / 1000.0, drill=a.drill, cam_delay=a.cam_delay, cam_clock=a.cam_clock,
                wall_in=a.wall_in, trap=a.trap, fw=a.fw, motion=a.motion, v_min=a.v_min,
                fws=dict(x.split("=") for x in a.fw_mix.split(",") if x), edge_physics=a.edge_physics,
                own_off=own_off, silent=silent, held=dict(x.split("=") for x in a.held), cam_misplace=misplace,
                phantoms=a.phantoms, ctrl_kw=ckw)
        r = s.run()
        r["wall_s"] = round(time.time() - t0, 1)
        r.update(drill=a.drill, old_fw=a.old_fw, loss=a.loss, lat_max_ms=a.lat_max_ms, lost_mode=a.lost_mode,
                 cam_delay=a.cam_delay, cam_clock=a.cam_clock, wall_in=a.wall_in, trap=a.trap, fw=a.fw,
                 motion=a.motion, fw_mix=a.fw_mix, v_min=a.v_min)
        print(json.dumps(r), flush=True)
        res.append(r)
        if od:
            json.dump(r, open(os.path.join(od, "summary.json"), "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
