#!/usr/bin/env python3
"""Synthetic robot logs that match a synthetic recording, with known truth (not a test module).

Writes a console session folder exactly like tools/roam_session.py does (raw.csv, runinfo.csv,
runend.csv, param.csv, sync.csv, clock.csv, clock_sync.csv, dock.csv, session.json) and a
recording folder like tools/pi/camrec.py (frames.csv on the Pi clock, events.csv, meta.json, and
optionally video.avi) plus truth.csv, the camera model (camera.json, from tests/synth.py's
overhead camera), floor_calibration.json, video_clock.json and a fleet map.json with the docks.

The robots start parked in their docks, back out, do a calibration spin, roam short random routes
(L, circle, square: each a `run demo` whose library pose starts at 0), optionally visit their dock
in the middle, can be picked up and put down elsewhere, and park in their docks at the end.

Errors (per robot, around these values; docs/session-notes/2026-10-19.md):
  gyro reports 0.925 of the real turn (it under-reports ~7.5 %), plus drift ~0.3 deg/s and noise;
  wheels count real/0.98 (wheel scale ~0.98), 0.4 % left/right mismatch, random slip, wheel
  heading scrubs 5 % in turns; robot clock offset and +-80 ppm drift, the Pi clock 0.35 s ahead of
  the laptop, Wi-Fi delay 3-25 ms with rare slow packets, 2 % lost rows and a few lost bursts;
  one robot with the gyro and odometry signs flipped (yaw_sign / odo_sign = -1).

    python tools/recognition/tests/synth_logs.py <out folder> [--seconds 300] [--measure]
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(os.path.dirname(HERE))
for p in (TOOLS, os.path.join(TOOLS, "recognition"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import synth as camsynth  # noqa: E402  (tools/recognition/tests/synth.py)

DT = 0.02                     # robot RAW rate 50 Hz
FPS = 25.0
T_START = 1792400000.0        # laptop clock at the session start (true time)
PI_OFFSET_S = 0.35            # Pi clock = laptop + this
BOARD = (0.0, 122.0)
ROBOT_HEIGHT_CM = 4.0
BUMP_AHEAD_CM = 6.1           # docked robot centre in front of the dock's back wall (dock_params)
RAW_COLS = ("run,ms,step,wl_deg,wr_deg,wl_rpm,wr_rpm,v_cms,w_dps,x_cm,y_cm,th_deg,ax,ay,az,gx,gy,gz,roll_deg,"
            "pitch_deg,yaw_deg,dL_cm,dCL_cm,dC_cm,dCR_cm,dR_cm,dtop_cm,dbot_cm,lineL,lineC,lineR,battery,charging,"
            "lifted,swL,swR,raw_n").split(",")
DOCK_COLS = ("run,ms_start,ms_first,ms_left,ms_right,ms_both,first,result,travel_cm,travel_left_cm,travel_right_cm,"
             "pivot_deg,stripe_l_cm,stripe_c_cm,stripe_r_cm,frame,x0_cm,y0_cm,th0_deg,x_cm,y_cm,th_deg,still_ms,"
             "drift_dps,gz_mean,drift_used,n,funnel_deg,twist_at_cm").split(",")


class RobotSpec:
    def __init__(self, name, obj, dock_xy, dock_heading, home_centre, gyro=0.925, drift=0.3, wheel=0.98,
                 yaw_sign=1, odo_sign=1, boot="a1f0", ppm=40.0, boot_age_s=60.0):
        self.name, self.obj = name, obj
        self.dock_wall = dock_xy                  # middle of the back wall's inner face (floor cm)
        self.dock_heading = dock_heading          # the way a docked robot faces
        self.home_centre = home_centre            # where it prefers to roam
        self.gyro, self.drift, self.wheel = gyro, drift, wheel
        self.yaw_sign, self.odo_sign = yaw_sign, odo_sign
        self.boot, self.ppm, self.boot_age_s = boot, ppm, boot_age_s

    @property
    def park(self):
        c, s = math.cos(math.radians(self.dock_heading)), math.sin(math.radians(self.dock_heading))
        return (self.dock_wall[0] - BUMP_AHEAD_CM * c, self.dock_wall[1] - BUMP_AHEAD_CM * s, self.dock_heading)


def default_robots():
    return [RobotSpec("Left", 1, (25.0, 4.0), -90.0, (35.0, 70.0), gyro=0.922, drift=0.31, wheel=0.976,
                      boot="1a2b", ppm=55.0, boot_age_s=80.0),
            RobotSpec("Center", 2, (61.0, 4.0), -90.0, (61.0, 75.0), gyro=0.928, drift=-0.27, wheel=0.981,
                      yaw_sign=-1, odo_sign=-1, boot="3c4d", ppm=-70.0, boot_age_s=45.0),
            RobotSpec("Right", 3, (97.0, 4.0), -90.0, (88.0, 70.0), gyro=0.925, drift=0.35, wheel=0.979,
                      boot="5e6f", ppm=20.0, boot_age_s=120.0)]


# =============================================================================== motion script
class Motion:
    """Truth motion of one robot at DT steps: a list of (t, x, y, th, v, w, run, lifted, phase)."""

    def __init__(self, spec, rng):
        self.spec, self.rng = spec, rng
        x, y, th = spec.park
        self.x, self.y, self.th = x, y, th
        self.t = 0.0
        self.rows = []
        self.run = None
        self.runs = []            # (run id, kind, t0, t1)
        self.nrun = 0
        self.docked = []          # (t_start, t_end) parked in the dock
        self.lifts = []

    def _step(self, v, w, phase, lifted=False, slip=True):
        self.rows.append((self.t, self.x, self.y, self.th, v, w, self.run, lifted, phase))
        th0 = self.th
        self.th = self.th + w * DT
        hm = math.radians((th0 + self.th) / 2)
        dx, dy = v * DT * math.cos(hm), v * DT * math.sin(hm)
        if slip and (abs(v) > 0 or abs(w) > 0):
            # slip the wheels never see: a little sideways and along
            lat = self.rng.normal(0, 0.012)
            dx += -math.sin(hm) * lat
            dy += math.cos(hm) * lat
        self.x += dx
        self.y += dy
        self.t += DT

    def start_run(self, kind):
        self.nrun += 1
        self.run = "%s-%d" % (self.spec.boot, self.nrun)
        self.runs.append([self.run, kind, self.t, None])

    def end_run(self):
        if self.run:
            self.runs[-1][3] = self.t
        self.run = None

    def pause(self, secs, phase="stop"):
        for _ in range(int(round(secs / DT))):
            self._step(0.0, 0.0, phase)

    def drive(self, dist, speed=10.0, phase="drive"):
        n = max(1, int(round(abs(dist) / speed / DT)))
        v = math.copysign(abs(dist) / (n * DT), dist)
        for _ in range(n):
            self._step(v, 0.0, phase if dist >= 0 else "reverse")

    def turn(self, deg, rate=90.0):
        n = max(1, int(round(abs(deg) / rate / DT)))
        w = deg / (n * DT)
        for _ in range(n):
            self._step(0.0, w, "turn")

    def arc(self, speed, rate, secs):
        for _ in range(int(round(secs / DT))):
            self._step(speed, rate, "arc")

    def face(self, x, y):
        want = math.degrees(math.atan2(y - self.y, x - self.x))
        d = (want - self.th + 180.0) % 360.0 - 180.0
        if abs(d) > 0.5:
            self.turn(d)

    def goto(self, x, y, speed=10.0):
        self.face(x, y)
        self.drive(math.hypot(x - self.x, y - self.y), speed)

    def lift(self, secs, x, y, th):
        """Picked up, carried, put down (the wheels don't turn; the gyro sees the turn)."""
        t0 = self.t
        x0, y0, th0 = self.x, self.y, self.th
        n = int(round(secs / DT))
        dth = (th - th0 + 180.0) % 360.0 - 180.0
        for k in range(n):
            a = (k + 1) / n
            self.rows.append((self.t, self.x, self.y, self.th, 0.0, dth / secs, self.run, True, "carried"))
            self.x, self.y, self.th = x0 + a * (x - x0), y0 + a * (y - y0), th0 + a * dth
            self.t += DT
        self.lifts.append((t0, self.t))


def _route(m, rng):
    """One roam_session route (route_commands): L, circle or square, ending near its start."""
    kind = rng.choice(["L", "circle", "square"])
    left = rng.random() < 0.5
    sgn = 1 if left else -1
    if kind == "L":
        a, b = rng.integers(15, 31), rng.integers(10, 26)
        m.drive(a)
        m.turn(90 * sgn)
        m.drive(b)
        m.turn(180)
        m.drive(b)
        m.turn(-90 * sgn)
        m.drive(a)
        m.turn(180)
    elif kind == "circle":
        a, r = rng.integers(5, 16), rng.integers(10, 19)
        m.drive(a)
        speed = 10.0
        rate = math.degrees(speed / r) * sgn
        m.arc(speed, rate, 360.0 / abs(rate))
        m.turn(180)
        m.drive(a)
        m.turn(180)
    else:
        s = rng.integers(15, 31)
        for _ in range(4):
            m.drive(s)
            m.turn(90 * sgn)
    return kind


def _fits(m, extent=34.0):
    lo, hi = BOARD[0] + 12.0, BOARD[1] - 12.0
    c, s = math.cos(math.radians(m.th)), math.sin(math.radians(m.th))
    for fx, fy in ((extent, 0), (extent, extent), (extent, -extent), (0, extent), (0, -extent)):
        x, y = m.x + c * fx - s * fy, m.y + s * fx + c * fy
        if not (lo <= x <= hi and lo <= y <= hi + 12.0):
            return False
    return True


def _dock_in(m):
    """Drive to 20 cm in front of the dock, face it, creep in, sit still."""
    px, py, ph = m.spec.park
    c, s = math.cos(math.radians(ph)), math.sin(math.radians(ph))
    m.start_run("dock")
    m.goto(px - 20 * c, py - 20 * s)
    m.face(px + 50 * c, py + 50 * s)
    # the funnel straightens it: end exactly at the park pose
    m.drive(math.hypot(px - m.x, py - m.y), speed=4.0, phase="dock")
    m.x, m.y, m.th = px, py, ph + 360.0 * round((m.th - ph) / 360.0)
    t_in = m.t
    m.pause(1.0, "docked")
    m.end_run()
    return t_in


def _dock_out(m):
    m.start_run("out")
    m.drive(-16.0, speed=6.0)
    m.turn(180.0)
    m.end_run()


def simulate(spec, seconds, rng, visit_at=None, lift_at=None):
    m = Motion(spec, rng)
    m.pause(2.0, "docked")                         # parked before anything (no RAW: no run)
    m.docked.append((0.0, m.t))
    _dock_out(m)
    m.pause(rng.uniform(0.5, 1.5))
    m.start_run("calib")                           # blink, compass wiggle, spin both ways, roll
    m.pause(3.0)
    m.turn(20)
    m.turn(-20)
    m.pause(1.0)
    m.turn(360, rate=120)
    m.turn(-360, rate=120)
    m.drive(10)
    m.drive(-10)
    m.end_run()
    visited = visit_at is None
    lifted = lift_at is None
    end_by = seconds - 25.0
    while m.t < end_by:
        m.pause(rng.uniform(0.5, 2.5))
        if not visited and m.t >= visit_at:
            t_in = _dock_in(m)
            m.pause(5.0, "docked")
            m.docked.append((t_in, m.t))
            _dock_out(m)
            visited = True
            continue
        if not lifted and m.t >= lift_at:
            m.start_run("demo")
            m.drive(5)
            hx, hy = spec.home_centre
            m.lift(2.5, hx + rng.uniform(-8, 8), hy + rng.uniform(-8, 8), rng.uniform(0, 360))
            m.pause(0.6)
            m.drive(6)
            m.end_run()
            lifted = True
            continue
        if not _fits(m):
            m.start_run("demo")                    # head back towards its own area first
            hx, hy = spec.home_centre
            m.goto(hx + rng.uniform(-5, 5), hy + rng.uniform(-5, 5))
            m.turn(rng.uniform(-180, 180))
            m.end_run()
            continue
        m.start_run("demo")
        _route(m, rng)
        m.end_run()
    m.pause(1.0)
    t_in = _dock_in(m)
    m.pause(2.0, "docked")
    m.docked.append((t_in, m.t))
    return m


# =============================================================================== sensors -> files
def sense(m, rng):
    """RAW rows from the truth (what the robot would report), in robot time (s since session start)."""
    sp = m.spec
    rows = []
    yaw = rng.uniform(-30, 30)                    # the gyro's own zero
    wl = wr = 0.0
    odo = None
    b_eff = 9.0                                   # wheel track, cm (for the library's wheel heading)
    mismatch = 1.0 + rng.normal(0, 0.004)
    for (t, x, y, th, v, w, run, lifted, phase) in m.rows:
        # gyro: under-reports turns, drifts, noise (also while carried)
        yaw += (sp.gyro * w + sp.drift) * DT + rng.normal(0, 0.02)
        if run is None:
            odo = None
            continue
        if odo is None or odo["run"] != run:
            odo = dict(run=run, x=0.0, y=0.0, th=0.0)
        if not lifted:
            ds = v * DT / sp.wheel * (1.0 + rng.normal(0, 0.02))
            dth = w * DT * 1.05 + rng.normal(0, 0.01)         # wheel heading scrubs in turns
            dl = (ds - math.radians(dth) * b_eff / 2) * mismatch
            dr = ds + math.radians(dth) * b_eff / 2
            ds_c, dth_c = (dl + dr) / 2, math.degrees((dr - dl) / b_eff)
            hm = math.radians(odo["th"] + dth_c / 2)
            odo["x"] += ds_c * math.cos(hm)
            odo["y"] += ds_c * math.sin(hm)
            odo["th"] += dth_c
            wl += dl / (math.pi * 3.4) * 360
            wr += dr / (math.pi * 3.4) * 360
        raw_yaw = ((yaw * sp.yaw_sign) + 180.0) % 360.0 - 180.0
        rows.append(dict(t=t, run=run, x=odo["x"], y=odo["y"] * sp.odo_sign, th=odo["th"] * sp.odo_sign,
                         yaw=raw_yaw, gz=(sp.gyro * w + sp.drift) * sp.yaw_sign, wl=wl, wr=wr, v=v, w=w,
                         lifted=1 if lifted else 0))
    return rows


def write_session(m, rows, out, rng, writers):
    """Append this robot's rows to the shared CSV writers (robot, t_recv first, as the console)."""
    sp = m.spec
    boot_t = -sp.boot_age_s                       # robot powered on this long before the session start

    def robot_ms(t):
        return int(round((t - boot_t) * 1000.0 * (1.0 + sp.ppm * 1e-6)))

    def recv(t):
        lat = 0.003 + rng.exponential(0.008)
        if rng.random() < 0.003:
            lat += rng.uniform(0.1, 0.3)
        return T_START + t + lat

    # lost rows: 2 % one by one, and three bursts of 0.3-0.6 s
    n = len(rows)
    keep = rng.random(n) > 0.02
    for _ in range(3):
        a = int(rng.integers(0, max(1, n - 40)))
        keep[a:a + int(rng.integers(15, 31))] = False
    raw_n = 0
    for k, r in enumerate(rows):
        raw_n += 1
        if not keep[k]:
            continue
        vals = dict(run=r["run"], ms=robot_ms(r["t"]), step=0, wl_deg="%.2f" % r["wl"], wr_deg="%.2f" % r["wr"],
                    wl_rpm="%.1f" % 0, wr_rpm="%.1f" % 0, v_cms="%.2f" % r["v"], w_dps="%.2f" % r["w"],
                    x_cm="%.3f" % r["x"], y_cm="%.3f" % r["y"], th_deg="%.3f" % r["th"], ax="0.0", ay="0.0",
                    az="1.0", gx="0.0", gy="0.0", gz="%.3f" % r["gz"], roll_deg="0.0", pitch_deg="0.0",
                    yaw_deg="%.3f" % r["yaw"], dL_cm=100, dCL_cm=100, dC_cm=100, dCR_cm=100, dR_cm=100, dtop_cm=100,
                    dbot_cm=3, lineL=0, lineC=0, lineR=0, battery=88, charging=0, lifted=r["lifted"], swL=0, swR=0,
                    raw_n=raw_n)
        writers["raw"].writerow([sp.name, "%.4f" % recv(r["t"])] + [vals[c] for c in RAW_COLS])
    for run, kind, t0, t1 in m.runs:
        writers["runinfo"].writerow([sp.name, "%.4f" % recv(t0), run, kind, "2026-10-20b", "1.0", robot_ms(t0), 88, 0,
                                     -55, kind, sp.yaw_sign, sp.odo_sign])
        writers["runend"].writerow([sp.name, "%.4f" % recv(t1 or t0), run, "done", robot_ms(t1 or t0),
                                    int(((t1 or t0) - t0) * 1000), 87, -55, 0, 0])
        writers["param"].writerow([sp.name, "%.4f" % recv(t0), run, "gyro_scale", "1"])
        writers["param"].writerow([sp.name, "%.4f" % recv(t0), run, "wheel_scale", "1"])
        writers["sync"].writerow([sp.name, "%.4f" % recv(t0), run, "start", robot_ms(t0), robot_ms(t0) + 300,
                                  robot_ms(t0) + 2])
        if kind == "dock":
            ms_end = robot_ms(t1 or t0)
            row = dict(run=run, ms_start=robot_ms(t0), ms_first=ms_end - 1200, ms_left=ms_end - 1200,
                       ms_right=ms_end - 1000, ms_both=ms_end - 1000, first="L", result="docked", travel_cm=20.4,
                       travel_left_cm=20.1, travel_right_cm=20.3, pivot_deg=0.4, stripe_l_cm=-1, stripe_c_cm=-1,
                       stripe_r_cm=-1, frame="odo", x0_cm=0, y0_cm=0, th0_deg=0, x_cm=0, y_cm=0, th_deg=0,
                       still_ms=900, drift_dps=0, gz_mean=0, drift_used=0, n=1, funnel_deg=0.5, twist_at_cm=-1)
            writers["dock"].writerow([sp.name, "%.4f" % recv(t1 or t0)] + [row[c] for c in DOCK_COLS])
    # clock checks every 10 s: bursts of three, as alvik_console.py
    t = 0.5
    tok = 1000 * sp.obj
    while t < m.t:
        for k in range(3):
            ts = t + 0.1 * k
            up, down = 0.002 + rng.exponential(0.006), 0.002 + rng.exponential(0.006)
            if rng.random() < 0.1:
                up += rng.uniform(0.02, 0.08)
            ms = robot_ms(ts + up)
            t_send, t_recv = T_START + ts, T_START + ts + up + down
            tok += 1
            writers["clock_sync"].writerow([sp.name, sp.boot, tok, "%.4f" % t_send, "%.4f" % t_recv, ms,
                                            "%.1f" % ((t_recv - t_send) * 1000), "%.4f" % ((t_send + t_recv) / 2 - ms / 1000)])
            writers["clock"].writerow([sp.name, "%.4f" % t_recv, tok, ms, sp.boot])
        t += 10.0


def make_session(out, seconds=300.0, seed=3, robots=None, visit_at=150.0, lift=True, render=False):
    """Write <out>/logs (console session) and <out>/recording (camera). Returns a dict with paths,
    the camera, the robots' specs and truth (per robot: t_wall, x, y, heading arrays, phases,
    docked intervals on t_wall)."""
    rng = np.random.default_rng(seed)
    robots = robots or default_robots()
    logs = os.path.join(out, "logs")
    rec = os.path.join(out, "recording")
    os.makedirs(logs, exist_ok=True)
    os.makedirs(rec, exist_ok=True)
    motions = []
    for k, sp in enumerate(robots):
        lift_at = 0.4 * seconds if (lift and sp.name == "Center") else None
        motions.append(simulate(sp, seconds, rng, visit_at=visit_at, lift_at=lift_at))
    files = {}
    heads = dict(raw=RAW_COLS, runinfo="run,experiment,fw_version,motor_fw,ms_start,battery,charging,wifi_dbm,route,"
                 "yaw_sign,odo_sign".split(","),
                 runend="run,result,ms_end,duration_ms,battery,wifi_dbm,raw_rows_sent,packets_sent".split(","),
                 param="run,name,value".split(","), sync="run,tag,ms_on,ms_off,ms_on_after".split(","),
                 dock=DOCK_COLS, clock="token,robot_ms,boot".split(","))
    writers = {}
    for name, cols in heads.items():
        f = open(os.path.join(logs, name + ".csv"), "w", newline="")
        files[name] = f
        writers[name] = csv.writer(f)
        writers[name].writerow(["robot", "t_recv"] + list(cols))
    f = open(os.path.join(logs, "clock_sync.csv"), "w", newline="")
    files["clock_sync"] = f
    writers["clock_sync"] = csv.writer(f)
    writers["clock_sync"].writerow(["robot", "boot", "token", "t_send", "t_recv", "robot_ms", "rtt_ms", "offset_s"])
    for m in motions:
        write_session(m, sense(m, rng), out, rng, writers)
    for f in files.values():
        f.close()
    with open(os.path.join(logs, "session.json"), "w") as f:
        json.dump(dict(start="synthetic", start_unix=T_START, laptop="synth", note="synthetic (synth_logs.py)"), f)

    # ---- the camera side
    cam = camsynth.cam_overhead()
    cam.save(os.path.join(rec, "camera.json"))
    import cv2
    g = camsynth.board_points(step=20.0)
    u, v = cam.project(np.column_stack([g, np.zeros(len(g))]))
    H, _ = cv2.findHomography(g.astype(np.float64), np.column_stack([u, v]).astype(np.float64))
    C = cam.center
    with open(os.path.join(rec, "floor_calibration.json"), "w") as f:
        json.dump(dict(H_floor_to_px=H.tolist(), image_size=[640, 480],
                       camera=dict(height_cm=float(C[2]), nadir_floor_cm=[float(C[0]), float(C[1])])), f, indent=1)
    with open(os.path.join(rec, "video_clock.json"), "w") as f:
        json.dump(dict(offset_s=PI_OFFSET_S, drift_ppm=0.0, use_drift=False, n_flashes=12, note="synthetic"), f)
    with open(os.path.join(out, "map.json"), "w") as f:
        json.dump(dict(home_docks=[dict(name="dock " + sp.name, x=sp.dock_wall[0], y=sp.dock_wall[1],
                                        heading=sp.dock_heading) for sp in robots]), f, indent=1)
    t_end = max(m.t for m in motions)
    t0_wall = T_START - 3.0 + PI_OFFSET_S
    nfr = int((t_end + 3.0) * FPS)
    jitter = rng.normal(0, 0.002, nfr)
    frame_t = t0_wall + np.arange(nfr) / FPS + jitter
    with open(os.path.join(rec, "frames.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "t_wall", "t_mono"])
        for k, t in enumerate(frame_t):
            w.writerow([k, "%.4f" % t, "%.4f" % (t - t0_wall)])
    with open(os.path.join(rec, "events.csv"), "w", newline="") as f:
        csv.writer(f).writerows([["t_wall", "frame", "text"], ["%.4f" % frame_t[0], 0, "synthetic roam start"]])
    with open(os.path.join(rec, "meta.json"), "w") as f:
        json.dump(dict(device="synthetic", width=640, height=480, fps_requested=FPS, fps_reported=FPS, host="synth",
                       label="synth_roam", note="synthetic recording with truth.csv (synth_logs.py)"), f, indent=1)
    truth = {}
    rows_out = []
    for m in motions:
        a = np.array([(r[0], r[1], r[2], r[3]) for r in m.rows])
        tw = T_START + a[:, 0] + PI_OFFSET_S
        ph = [r[8] for r in m.rows]
        truth[m.spec.name] = dict(t=tw, x=a[:, 1], y=a[:, 2], heading=a[:, 3], phase=ph, obj=m.spec.obj,
                                  docked=[(T_START + s + PI_OFFSET_S, T_START + e + PI_OFFSET_S) for s, e in m.docked],
                                  lifts=[(T_START + s + PI_OFFSET_S, T_START + e + PI_OFFSET_S) for s, e in m.lifts])
        fx = np.interp(frame_t, tw, a[:, 1])
        fy = np.interp(frame_t, tw, a[:, 2])
        fh = np.degrees(np.interp(frame_t, tw, np.radians(a[:, 3])))
        idx = np.clip(np.searchsorted(tw, frame_t), 0, len(tw) - 1)
        uu, vv = cam.project(np.column_stack([fx, fy, np.full(len(fx), ROBOT_HEIGHT_CM)]))
        truth[m.spec.name].update(frame_x=fx, frame_y=fy, frame_h=fh, frame_u=uu, frame_v=vv)
        hi_all = np.array([_img_heading(cam, fx[k], fy[k], fh[k]) for k in range(nfr)])
        truth[m.spec.name]["frame_himg"] = hi_all
        for k in range(nfr):
            hi = hi_all[k]
            rows_out.append([k, "%.4f" % frame_t[k], m.spec.obj, "alvik", "%.2f" % uu[k], "%.2f" % vv[k], "%.2f" % hi,
                             ph[idx[k]], m.spec.name, "%.3f" % fx[k], "%.3f" % fy[k], "%.3f" % (fh[k] % 360.0)])
    rows_out.sort(key=lambda r: (r[0], r[2]))
    with open(os.path.join(rec, "truth.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "t", "obj", "cls", "x", "y", "heading_deg", "phase", "name", "x_cm", "y_cm",
                    "heading_floor"])
        w.writerows(rows_out)
    if render:
        _render(rec, cam, truth, frame_t)
    return dict(out=out, logs=logs, recording=rec, camera=cam, robots=robots, truth=truth,
                frame_times={k: float(t) for k, t in enumerate(frame_t)}, pi_offset=PI_OFFSET_S)


def _img_heading(cam, x, y, h):
    u0, v0 = cam.project((x, y, ROBOT_HEIGHT_CM))
    u1, v1 = cam.project((x + 3 * math.cos(math.radians(h)), y + 3 * math.sin(math.radians(h)), ROBOT_HEIGHT_CM))
    return math.degrees(math.atan2(-(v1 - v0), u1 - u0)) % 360.0


def _render(rec, cam, truth, frame_t):
    """video.avi with the robots drawn (tools/label/tests/synth.py's floor and robot drawing)."""
    import cv2
    import importlib.util
    spec = importlib.util.spec_from_file_location("label_synth", os.path.join(TOOLS, "label", "tests", "synth.py"))
    ls = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ls)
    rng = np.random.default_rng(0)
    floor = ls.make_floor(rng)
    ls.ROBOT = 9.6 * cam.px_per_cm(61, 61, ROBOT_HEIGHT_CM)
    wr = cv2.VideoWriter(os.path.join(rec, "video.avi"), cv2.VideoWriter_fourcc(*"MJPG"), FPS, (640, 480))
    for k in range(len(frame_t)):
        img = floor.copy()
        for name, tr in truth.items():
            u, v = tr["frame_u"][k], tr["frame_v"][k]
            ls.draw_robot(img, u, v, tr["frame_himg"][k])
        wr.write(np.clip(img, 0, 255).astype(np.uint8))
    wr.release()


# =============================================================================== measuring
def _nearest_gap(ts, anchor_ts, causal):
    a = np.sort(np.asarray(anchor_ts, float))
    if not len(a):
        return np.full(len(ts), np.inf)
    i = np.searchsorted(a, ts, side="right")
    before = np.where(i > 0, ts - a[np.clip(i - 1, 0, len(a) - 1)], np.inf)
    if causal:
        return before
    after = np.where(i < len(a), a[np.clip(i, 0, len(a) - 1)] - ts, np.inf)
    return np.minimum(before, after)


def measure(sess, scenario, causal=False, label_every_s=30.0, every=5, fit=True, seed=11, prior_g=None):
    """Draft-label errors against truth for one way of anchoring. Returns dict(rows=[(name, t, gap_s, err_cm,
    err_px, herr_deg, sd_cm, score)], anchors={name: n}, notes=[...])."""
    from recognition import homes as HM
    from recognition import outside as O
    from recognition import robotlogs as RL
    rng = np.random.default_rng(seed)
    cam = sess["camera"]
    data = RL.read_alvik_logs(sess["logs"], recording=sess["recording"])
    mp = json.load(open(os.path.join(sess["out"], "map.json")))
    owners = {"dock " + sp.name: sp.name for sp in sess["robots"]}
    homes = HM.alvik_docks_from_map(mp, "default", cam, owners=owners, stations=False)
    ft = sess["frame_times"]
    frames = sorted(ft)
    st_settings = O.Settings()
    if prior_g is not None:            # no Alvik knowledge: generic priors
        for st in data.streams.values():
            st.priors.update(g=prior_g, g_sd=0.1, s=1.0, s_sd=0.05, d_sd=0.5)
    anchors = {n: [] for n in data.streams}
    truth = sess["truth"]
    # truth tracks in px for visits / motion (1 px click-like noise)
    tracks = {}
    for name, tr in truth.items():
        rows = [(f, ft[f], tr["frame_u"][f] + rng.normal(0, 0.5), tr["frame_v"][f] + rng.normal(0, 0.5))
                for f in frames[::2]]
        tracks[tr["obj"]] = rows
    obj_names = {tr["obj"]: name for name, tr in truth.items()}
    if "start" in scenario:
        for h in homes:
            if h["owner"] in data.streams:
                anchors[h["owner"]].append(HM.start_anchor(data.streams[h["owner"]], h, cam, ROBOT_HEIGHT_CM))
    if "visits" in scenario:
        visits = HM.find_visits(homes, tracks)
        for n, al in HM.anchors_from_visits(visits, homes, cam, obj_names, ROBOT_HEIGHT_CM).items():
            anchors[n] += al
    if "labels" in scenario or "onelabel" in scenario:
        labels = {}
        for name, tr in truth.items():
            t0 = data.streams[name].t[0]
            step = label_every_s if "labels" in scenario else 1e9
            first = t0 + 25.0
            want = np.arange(first, data.streams[name].t[-1], step) if step < 1e8 else np.array([first])
            for tw in want:
                f = int(np.argmin(np.abs(np.array([ft[k] for k in frames]) - tw)))
                ph = truth[name]["phase"][min(len(truth[name]["phase"]) - 1,
                                              int(np.searchsorted(truth[name]["t"], ft[f])))]
                if ph == "carried":
                    continue
                labels.setdefault(f, []).append(dict(obj=tr["obj"], cls="alvik", x=tr["frame_u"][f] + rng.normal(0, 1.5),
                                                     y=tr["frame_v"][f] + rng.normal(0, 1.5), status="confirmed",
                                                     heading_known=False, heading_deg=None))
        for n, al in O.anchors_from_labels(labels, ft, cam, obj_names, {"alvik": ROBOT_HEIGHT_CM}).items():
            anchors[n] += al
    if "motion" in scenario:
        for name, tr in truth.items():
            trf = O.track_floor([(r[1], r[2], r[3]) for r in tracks[tr["obj"]]], cam, ROBOT_HEIGHT_CM)
            first_label = min((a.t for a in anchors[name] if a.kind == "label"), default=None)
            anchors[name] += O.heading_anchor_from_motion(data.streams[name], trf, t_from=first_label)
    als = O.align(data, anchors, st_settings, fit=fit and ("labels" in scenario or "visits" in scenario))
    rows = []
    sel = frames[::every]
    ts = np.array([ft[f] for f in sel])
    for name, al in als.items():
        tr = truth[name]
        p = al.pose_at(ts, causal=causal)
        gaps = _nearest_gap(ts, [a.t for a in al.anchors], causal)
        idx = np.clip(np.searchsorted(tr["t"], ts), 0, len(tr["t"]) - 1)
        for k, f in enumerate(sel):
            if not p["ok"][k] or tr["phase"][idx[k]] == "carried":
                continue
            if ts[k] < tr["t"][0] or ts[k] > tr["t"][-1]:
                continue
            ex = p["x"][k] - tr["frame_x"][f]
            ey = p["y"][k] - tr["frame_y"][f]
            e = math.hypot(ex, ey)
            u, v = cam.project((p["x"][k], p["y"][k], ROBOT_HEIGHT_CM))
            epx = math.hypot(u - tr["frame_u"][f], v - tr["frame_v"][f])
            he = abs((p["heading"][k] - tr["frame_h"][f] + 180.0) % 360.0 - 180.0)
            rows.append((name, ts[k], gaps[k], e, epx, he, p["sd_cm"][k],
                         math.exp(-0.5 * (p["sd_cm"][k] / st_settings.ref_cm) ** 2)))
    return dict(rows=rows, anchors={n: len(al.anchors) for n, al in als.items()},
                calib={n: dict(al.calib) for n, al in als.items()}, notes=data.notes + sum((al.notes for al in als.values()), []))


BINS = [(0, 5), (5, 15), (15, 30), (30, 60), (60, 120), (120, 1e9)]


def table(res):
    """Per time-since-anchor bin: n, median / p90 position error (cm, px), median / p90 heading error."""
    rows = res["rows"]
    out = []
    for lo, hi in BINS:
        sel = [r for r in rows if lo <= r[2] < hi]
        if not sel:
            continue
        e = np.array([r[3] for r in sel])
        ep = np.array([r[4] for r in sel])
        h = np.array([r[5] for r in sel])
        sd = np.array([r[6] for r in sel])
        out.append(dict(bin="%g-%s s" % (lo, "%g" % hi if hi < 1e8 else "+"), n=len(sel),
                        med_cm=float(np.median(e)), p90_cm=float(np.percentile(e, 90)),
                        med_px=float(np.median(ep)), med_deg=float(np.median(h)), p90_deg=float(np.percentile(h, 90)),
                        med_sd_cm=float(np.median(sd)), within_2sd=float(np.mean(e <= 2 * sd)),
                        within_1sd=float(np.mean(e <= sd))))
    if rows:
        e = np.array([r[3] for r in rows])
        h = np.array([r[5] for r in rows])
        sd = np.array([r[6] for r in rows])
        out.append(dict(bin="all", n=len(rows), med_cm=float(np.median(e)), p90_cm=float(np.percentile(e, 90)),
                        med_px=float(np.median([r[4] for r in rows])), med_deg=float(np.median(h)),
                        p90_deg=float(np.percentile(h, 90)), med_sd_cm=float(np.median(sd)),
                        within_2sd=float(np.mean(e <= 2 * sd)), within_1sd=float(np.mean(e <= sd))))
    return out


def print_table(title, res):
    print("\n" + title)
    print("  anchors: " + ", ".join("%s %d" % kv for kv in sorted(res["anchors"].items())))
    for n, c in sorted(res["calib"].items()):
        if c.get("fitted"):
            print("  %s fitted: gyro x%.3f drift %+.2f deg/s wheels x%.3f" % (n, c["g"], c["d"], c["s"]))
    print("  %-10s %5s %8s %8s %7s %8s %8s %7s %6s %6s" % ("since", "n", "med cm", "p90 cm", "med px", "med deg",
                                                           "p90 deg", "sd cm", "in1sd", "in2sd"))
    for r in table(res):
        print("  %-10s %5d %8.2f %8.2f %7.1f %8.1f %8.1f %7.2f %6.2f %6.2f" % (
            r["bin"], r["n"], r["med_cm"], r["p90_cm"], r["med_px"], r["med_deg"], r["p90_deg"], r["med_sd_cm"],
            r["within_1sd"], r["within_2sd"]))


SCENARIOS = [   # (title, anchors, live, prior gyro scale: None = the Alvik plug-in's 1.08)
    ("A. start in the dock only (no re-alignment)", "start", False, None),
    ("B. start + dock visits (re-aligned at each visit)", "start+visits", False, None),
    ("C. person's labels every 30 s (+ calibration fit), no docks", "labels", False, None),
    ("C generic: as C, but no Alvik knowledge (prior gyro scale 1.0)", "labels", False, 1.0),
    ("D. start + visits + labels every 30 s (+ fit)", "start+visits+labels", False, None),
    ("E. one label + first forward stretch on video (no dock)", "onelabel+motion", False, None),
    ("B live (only anchors before each moment)", "start+visits", True, None),
    ("D live", "start+visits+labels", True, None),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out")
    ap.add_argument("--seconds", type=float, default=300.0)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--render", action="store_true", help="also write video.avi")
    ap.add_argument("--measure", action="store_true", help="print draft-label errors for several ways of anchoring")
    a = ap.parse_args()
    sess = make_session(a.out, a.seconds, a.seed, render=a.render)
    print("wrote", a.out)
    if a.measure:
        for title, sc, causal, prior in SCENARIOS:
            print_table(title, measure(sess, sc, causal=causal, prior_g=prior))


if __name__ == "__main__":
    sys.exit(main())
