#!/usr/bin/env python3
"""Camera bridge (Phase 0, 2026-10-27): the Pi's overhead camera -> camera fixes for run_plan.py (--fixes udp).

What it joins up. run_plan.py takes camera fixes as JSON lines on UDP 5010 (docs/data-formats.md, "Step follower").
Until today only the simulator and tools/live/track_live.py (blink naming, laptop side) sent them, and blink naming
named Left and Center too rarely on real footage (e02_camera.py's note). The E02 runner's tracker on the Pi does
work on real footage, but it lives inside e02.py. This bridge runs the same tracker family on the Pi, next to
run_plan.py:

  pictures   the recorder's /live stream (tools/pi/camrec.py, started by the launcher, which also records the video);
  tracker    e02_multitrack.MultiTracker: every robot at once against the floor memory, who is who by PREDICTION,
             never by looks. Here the prediction is the robot's OWN pose (wheels + gyro, sent by run_plan.py
             --poses-to): the last sure camera spot moved on by what the robot's own pose did since. A robot whose
             outline is merged with another's, contested, or missing gets NO fix (unsure is flagged, not guessed:
             the identity swap of 2026-10-27 01:00 was a guess);
  heading    at the start (before the robot has a pose): the map's start heading (the robot is put down facing
             it; --start-heading camera takes the outline's square instead, not reliable yet); while driving straight: the direction the camera saw it travel (e02_steer's
             heading_from_track, forwards or backwards from its own pose), at most every HEADING_EVERY_S;
             otherwise position only (the firmware keeps its gyro heading);
  out        {"type": "fix", ...} to run_plan's port; with --brick-watch also the brick watch's obstacles
             (tools/fleet/brickwatch.py, the "what changed" watch, hands ignored).

    python3 tools/fleet/cam_bridge.py --map M.json --calib tools/experiment/data/overhead_floorcal_roam1.json
    python3 tools/fleet/cam_bridge.py --map M.json --calib C.json --observe     # watch only: nothing sent

Started by run-fleet-pi.sh; the dry runs start it against tools/sim/fleet_sim.py --serve-live (tests).
Writes to --out (default logs/<date-time>-bridge): track.csv (every picture: each robot's spot and status),
events.jsonl (status changes, headings sent, obstacles), bridge_summary.json.
"""
import argparse
import csv
import datetime as dt
import json
import math
import os
import socket
import sys
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "live"), os.path.join(TOOLS, "recognition"),
          TOOLS):
    if p not in sys.path:
        sys.path.append(p)

import cv2              # noqa: E402
import numpy as np      # noqa: E402

CAPTURE_S = 0.04        # the recorder stamps a picture about one picture after it was taken (25 a second; the same
                        # allowance as e02_sim.py and track_live.py's default). No DOE: measured by
                        # tools/live/capture_delay.py when it matters
FIX_SIGMA_CM = 1.0      # chosen: the camera spot's 1-sigma error the robot weighs fixes by. Real E02 sessions: the
                        # outline centre is within 1-2 cm away from edges (it rides 5-9 cm onto the nose AT an open
                        # edge; Phase 0's routes stay 25 cm in). DOE note: 0.5 / 1 / 2 cm on the board, cross-track
                        # error against the planned lines (docs/doe-backlog.md, "Camera bridge")
HEADING_SIGMA_DEG = 3.0  # chosen: a heading from 1 s of straight travel (>= 5 cm). DOE: same runs, 2 / 3 / 6 deg
START_SIGMA_DEG = 3.0   # chosen: the outline's square at the start (median of START_FRAMES pictures). No DOE: the
                        # start check measures it (run_plan's 5 deg)
HEADING_EVERY_S = 0.5   # a travel heading goes out at most this often (the firmware takes each one it gets)
HEADING_WINDOW_S = 1.0  # camera spots this far back make a travel heading
HEADING_MIN_CM = 5.0    # ... spanning at least this far (e02_steer uses 4 cm on single legs)
STRAIGHT_DEG = 2.0      # the robot's own heading changed less than this over the window: it drove straight
START_FRAMES = 12       # pictures whose outline headings make the start heading (about 0.5-0.8 s)
FLOOR_FRAMES = 5        # pictures (median) for the floor memory at the start
POSE_PORT = 5011        # run_plan.py --poses-to 127.0.0.1:5011
STATUS_EVERY_S = 2.0
MAX_BACKLOG_S = 0.3     # chosen: a picture that reaches the bridge this much later than the quickest ones (the bridge
                        # fell behind: pictures queue in the stream) is skipped, so fixes never trail by a queue.
                        # The robot takes fixes up to 1 s old (params.h fix_max_age_ms). No DOE: a guard
STILL_S = 1.0           # --observe: a robot still this long gets its outline heading printed (the turn check)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def outline_heading(trk, g, x, y, ref_deg):
    """The robot's heading from its outline's square at floor spot (x, y), the one of the four sides nearest
    ref_deg (deg), or None. g: the tracker's current (blurred, float) picture."""
    try:
        _, mask = trk._diff(g)
    except Exception:
        return None
    u, v = trk._px(x, y)
    if not (np.isfinite(u) and np.isfinite(v)):
        return None
    r = int(round(0.9 * trk.rpx))
    H, W = mask.shape
    u0, v0, u1, v1 = max(0, int(u) - r), max(0, int(v) - r), min(W, int(u) + r + 1), min(H, int(v) + r + 1)
    sub = mask[v0:v1, u0:u1]
    if sub.size == 0:
        return None
    sub = cv2.morphologyEx(sub, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(sub, connectivity=8)
    if n < 2:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[k, cv2.CC_STAT_AREA] < 0.3 * trk.area:
        return None
    ys, xs = np.nonzero(lab == k)
    (cu, cv_), _, ang = cv2.minAreaRect(np.column_stack([xs + u0, ys + v0]).astype(np.float32))
    a = math.radians(ang)
    p0 = trk._floor(cu, cv_)
    p1 = trk._floor(cu + 10 * math.cos(a), cv_ + 10 * math.sin(a))
    if not all(np.isfinite(q) for q in (*p0, *p1)):
        return None
    side = math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))
    best = min((wrap180(side + 90 * k2) for k2 in range(4)), key=lambda h: abs(wrap180(h - ref_deg)))
    return best


def make_tracker_class():
    import e02_multitrack as MT

    class OdoTracker(MT.MultiTracker):
        """MultiTracker whose prediction is the robot's own pose: last sure spot + (own pose now - own pose then)."""

        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.odo = {}         # robot -> deque[(t, x, y, th)]

        def own_pose(self, name, x, y, th, t):
            with self.lock:
                self.odo.setdefault(name, deque(maxlen=600)).append((float(t), float(x), float(y), float(th)))

        def own_at(self, name, t):
            h = self.odo.get(name)
            if not h:
                return None
            if t <= h[0][0]:
                return h[0]
            if t >= h[-1][0]:
                return h[-1]
            lo, hi = 0, len(h) - 1
            while hi - lo > 1:
                mid = (lo + hi) // 2
                if h[mid][0] <= t:
                    lo = mid
                else:
                    hi = mid
            a, b = h[lo], h[hi]
            f = (t - a[0]) / max(1e-6, b[0] - a[0])
            return (t, a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * wrap180(b[3] - a[3]))

        def _predict(self, tr, t):
            a, b = self.own_at(tr.name, tr.bt), self.own_at(tr.name, t)
            if a is None or b is None:
                return super()._predict(tr, t)
            return tr.bx + (b[1] - a[1]), tr.by + (b[2] - a[2])

    return OdoTracker


class Bridge:
    def __init__(self, args, cam, rect, starts, send, log_dir):
        self.args = args
        self.cam = cam
        self.rect = rect
        self.starts = starts                     # name -> (x, y, heading deg)
        self.send = send
        self.out = log_dir
        T = make_tracker_class()
        cx, cy = (rect[0] + rect[2]) / 2.0, (rect[1] + rect[3]) / 2.0
        self.trk = T(cam, board_center=(cx, cy), board_rect=rect, margin_cm=args.margin_cm)
        for n, (x, y, h) in starts.items():
            self.trk.add(n, x, y, heading=h)
        self.floor_frames = []
        self.ready = False
        self.has_pose = {n: False for n in starts}       # the robot has a pose of its own (run_plan's echo)
        self.moved = {n: False for n in starts}
        self.start_heads = {n: deque(maxlen=START_FRAMES) for n in starts}
        self.last_head_t = {n: 0.0 for n in starts}
        self.counts = {n: dict(seen=0, unsure=0, predicted=0, lost=0, fixes=0, headings=0) for n in starts}
        self.frames = 0
        self.t_first = None
        self.status_t = 0.0
        self.still = {n: None for n in starts}            # --observe: (since t, x, y, printed)
        self.last_still_head = {n: None for n in starts}
        self.bw = None
        self.obstacles = []
        self.track_f = open(os.path.join(log_dir, "track.csv"), "w", newline="")
        self.track_w = csv.writer(self.track_f)
        self.track_w.writerow(["t", "robot", "x", "y", "status", "sent_th", "why"])
        self.ev_f = open(os.path.join(log_dir, "events.jsonl"), "a")

    def event(self, kind, **kw):
        kw.update(kind=kind, t=round(time.time(), 3))
        self.ev_f.write(json.dumps(kw) + "\n")
        self.ev_f.flush()

    def say(self, s):
        print(time.strftime("%H:%M:%S ") + s, flush=True)

    # -- inputs ------------------------------------------------------------------------------
    def on_pose(self, msg):
        n = msg.get("robot")
        if n not in self.starts:
            return
        self.trk.own_pose(n, msg["x"], msg["y"], msg["th"], msg["t"])
        if not self.has_pose[n]:
            self.has_pose[n] = True
            self.event("own_pose_first", robot=n, x=msg["x"], y=msg["y"], th=msg["th"])

    def on_frame(self, img, t):
        self.frames += 1
        if self.t_first is None:
            self.t_first = t
        if not self.ready:
            self.floor_frames.append(img)
            if len(self.floor_frames) >= FLOOR_FRAMES:
                spots = [(x, y) for x, y, _ in self.starts.values()]
                self.trk.set_background(self.floor_frames, robots=spots)
                if self.args.brick_watch:
                    import brickwatch as BW
                    self.bw = BW.BrickWatch(self.cam, self.rect, send=self.on_obstacle, log=self.say)
                    self.bw.start(self.floor_frames, spots)
                self.ready = True
                self.say("floor memory taken (%d pictures) with the robots on their start spots: %s"
                         % (len(self.floor_frames), ", ".join(self.starts)))
            return
        self.trk.push(img, t)
        sts = self.trk.states()
        for n, st in sts.items():
            c = self.counts[n]
            c[st["status"]] = c.get(st["status"], 0) + 1
            sent_th = None
            if st["status"] == "seen" and abs(st["t_seen"] - t) < 1e-6:
                sx, sy, h0 = self.starts[n]
                if math.hypot(st["x"] - sx, st["y"] - sy) > 3.0:
                    self.moved[n] = True
                th, sth = None, None
                if not self.has_pose[n] and not self.moved[n] and self.args.start_heading == "map":
                    th, sth = h0, START_SIGMA_DEG          # as placed: the start check then only checks the position
                elif not self.has_pose[n] and not self.moved[n]:
                    oh = outline_heading(self.trk, self.trk.frame, st["x"], st["y"], h0)
                    if oh is not None:
                        self.start_heads[n].append(oh)
                    if len(self.start_heads[n]) >= min(START_FRAMES, 5):
                        hs = list(self.start_heads[n])
                        ref = hs[0]
                        th = wrap180(ref + float(np.median([wrap180(h - ref) for h in hs])))
                        sth = START_SIGMA_DEG
                elif self.has_pose[n] and not self.args.no_travel_heading:
                    th = self.travel_heading(n, t)
                    sth = HEADING_SIGMA_DEG if th is not None else None
                if th is not None:
                    sent_th = th
                if not self.args.observe and (th is not None or self.has_pose[n] or self.moved[n]):
                    # (a robot with no pose yet takes no position-only fix: its first must carry a heading)
                    msg = {"type": "fix", "robot": n, "t": round(t, 4), "x": round(st["x"], 2), "y": round(st["y"], 2),
                           "th": None if th is None else round(th, 2), "sigma": FIX_SIGMA_CM,
                           "sigma_th": sth or HEADING_SIGMA_DEG, "src": "cam_bridge"}
                    self.send(msg)
                    c["fixes"] += 1
                    if th is not None and self.has_pose[n]:
                        c["headings"] += 1
                        self.event("heading", robot=n, th=round(th, 2), how="travel")
            self.track_w.writerow(["%.4f" % t, n, "%.2f" % st["x"], "%.2f" % st["y"], st["status"],
                                   "" if sent_th is None else "%.2f" % sent_th, st.get("why") or ""])
            if self.args.observe:
                self.turn_watch(n, st, t)
        for (te, robot, what) in list(self.trk.events):
            if te > getattr(self, "_ev_t", 0.0):
                self.event("status", robot=robot, what=what, t_frame=te)
                if not what.startswith("seen"):
                    self.say("%s: %s" % (robot, what))
        if self.trk.events:
            self._ev_t = self.trk.events[-1][0]
        if self.bw is not None:
            spots = [(st["x"], st["y"]) for st in sts.values()]
            self.bw.check(img, spots, t)
        if time.time() - self.status_t >= STATUS_EVERY_S:
            self.status_t = time.time()
            rate = self.frames / max(1e-6, t - self.t_first) if self.t_first and t > self.t_first else 0.0
            self.say("%.0f pictures/s; %s" % (rate, "; ".join(
                "%s %s %d fixes" % (n, sts[n]["status"], self.counts[n]["fixes"]) for n in sts)))

    def on_obstacle(self, ob):
        self.obstacles.append(ob)
        self.event("obstacle", **{k: v for k, v in ob.items() if k != "type"})
        if not self.args.observe:
            self.send({k: v for k, v in ob.items() if k != "area_cm2"})

    def travel_heading(self, n, t):
        """Direction of travel over the last HEADING_WINDOW_S when the robot's own pose says it drove straight."""
        if t - self.last_head_t[n] < HEADING_EVERY_S:
            return None
        import e02_steer as ST
        a, b = self.trk.own_at(n, t - HEADING_WINDOW_S), self.trk.own_at(n, t)
        if a is None or b is None:
            return None
        dx, dy = b[1] - a[1], b[2] - a[2]
        if math.hypot(dx, dy) < HEADING_MIN_CM or abs(wrap180(b[3] - a[3])) > STRAIGHT_DEG:
            return None
        smp = self.trk.history(n, t - HEADING_WINDOW_S, t)
        h = ST.heading_from_track(smp, min_cm=HEADING_MIN_CM) if len(smp) >= 5 else None
        if h is None:
            return None
        own_dir = math.degrees(math.atan2(dy, dx))
        if abs(wrap180(h - own_dir)) > 45:          # the camera and the robot disagree on which way it went: skip
            return None
        backwards = math.cos(math.radians(b[3] - own_dir)) < 0
        self.last_head_t[n] = t
        return wrap180(h + (180.0 if backwards else 0.0))

    def turn_watch(self, n, st, t):
        """--observe: when a robot has stood still STILL_S, print its outline heading and the change since its last
        stop. A turn of 90 deg comes back to the same side of the square: a change of 0 is a good turn."""
        if st["status"] != "seen":
            return
        s = self.still[n]
        if s is None or math.hypot(st["x"] - s[1], st["y"] - s[2]) > 1.0:
            self.still[n] = (t, st["x"], st["y"], False)
            return
        if not s[3] and t - s[0] >= STILL_S:
            ref = self.last_still_head[n] if self.last_still_head[n] is not None else self.starts[n][2]
            hs = [outline_heading(self.trk, self.trk.frame, st["x"], st["y"], ref)]
            h = hs[0]
            self.still[n] = (s[0], s[1], s[2], True)
            if h is None:
                self.say("%s still at (%.1f, %.1f): outline heading not clear" % (n, st["x"], st["y"]))
                return
            prev = self.last_still_head[n]
            ch = "" if prev is None else ", %+.1f deg from its last stop (mod 90: a good turn 90 shows about 0)" % \
                wrap180(h - prev)
            self.say("%s still at (%.1f, %.1f): outline square at %.1f deg%s" % (n, st["x"], st["y"], h, ch))
            self.event("still", robot=n, x=st["x"], y=st["y"], outline_deg=round(h, 2),
                       change_deg=None if prev is None else round(wrap180(h - prev), 2))
            self.last_still_head[n] = h

    def summary(self):
        s = dict(frames=self.frames, skipped_behind=getattr(self, "skipped", 0), robots=self.counts, obstacles=len(self.obstacles),
                 duration_s=None if self.t_first is None else round(time.time() - self.t_first, 1))
        with open(os.path.join(self.out, "bridge_summary.json"), "w") as f:
            json.dump(s, f, indent=1)
        return s

    def close(self):
        self.track_f.close()
        self.ev_f.close()


def starts_from(args):
    out = {}
    if args.map:
        import fleetmap as FM
        m = FM.normalize(FM.load(args.map))
        f = m["floor"]
        rect = (f["x_min"], f["y_min"], f["x_max"], f["y_max"])
        for r in m["robots"]:
            s = r["start"]
            out[r["name"]] = (float(s[0]), float(s[1]), float(s[2]) if len(s) > 2 else 0.0)
    else:
        rect = tuple(float(v) for v in args.rect.split(","))
    for a in args.at:
        n, _, xyz = a.partition("=")
        v = [float(q) for q in xyz.split(",")]
        out[n] = (v[0], v[1], v[2] if len(v) > 2 else 0.0)
    if args.robots:
        keep = [q.strip() for q in args.robots.split(",") if q.strip()]
        out = {n: out[n] for n in keep if n in out}
    if not out:
        raise SystemExit("No robots to follow: give --map (its robots' start spots) or --at NAME=x,y,heading")
    return rect, out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", help="map.json: the table and each robot's start spot (where it stands now)")
    ap.add_argument("--rect", default="0,0,122,122", help="without --map: the table x0,y0,x1,y1 cm")
    ap.add_argument("--at", action="append", default=[], metavar="NAME=x,y,heading",
                    help="a robot's spot now (instead of / on top of the map's start)")
    ap.add_argument("--robots", default="", help="only these robots, e.g. Left,Right")
    ap.add_argument("--calib", required=True, help="the overhead camera's floor calibration (roam1 at home)")
    ap.add_argument("--stream", default="127.0.0.1:8000", help="the recorder's /live (the Pi itself by default)")
    ap.add_argument("--to", default="127.0.0.1:5010", help="run_plan.py's fix port")
    ap.add_argument("--pose-port", type=int, default=POSE_PORT, help="run_plan.py --poses-to sends here")
    ap.add_argument("--brick-watch", action="store_true", help="also report new objects (bricks) as obstacles")
    ap.add_argument("--observe", action="store_true",
                    help="send nothing; print what it sees, and each robot's outline heading when it stands still "
                         "(the bench turn check)")
    ap.add_argument("--start-heading", choices=["camera", "map"], default="map",
                    help="the first fix's heading: the map's start heading (map, default: the robot is put down facing "
                         "it; the camera checks the position, the first straight drive's travel heading corrects the "
                         "rest) or the outline's square (camera: NOT reliable yet: on simulated pictures single "
                         "pictures were up to 37 deg off away from the picture's middle, a perspective effect)")
    ap.add_argument("--no-travel-heading", action="store_true",
                    help="position-only fixes once the robot has its pose (the A/B check of the travel heading)")
    ap.add_argument("--margin-cm", type=float, default=8.0, help="a robot's centre may be this far past an open side")
    ap.add_argument("--out", help="log folder (default logs/<date-time>-bridge)")
    ap.add_argument("--max-s", type=float, default=0, help="stop after this long (0: until Ctrl+C)")
    ap.add_argument("--wait-calib-s", type=float, default=20.0, help="wait this long for --calib to appear (dry runs)")
    args = ap.parse_args(argv)

    t0 = time.time()
    while not os.path.exists(args.calib) and time.time() - t0 < args.wait_calib_s:
        time.sleep(0.2)
    import e02_camera as CAM
    cam = CAM.load_camera(args.calib)
    rect, starts = starts_from(args)
    out = args.out or os.path.join(REPO, "logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") + "-bridge")
    os.makedirs(out, exist_ok=True)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    h, _, p = args.to.rpartition(":")
    dest = (h or "127.0.0.1", int(p))

    def send(msg):
        try:
            tx.sendto(json.dumps(msg).encode(), dest)
        except OSError:
            pass
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        rx.bind(("127.0.0.1", args.pose_port))
    except OSError as e:
        raise SystemExit("Port %d is in use (another camera bridge still running?): %s" % (args.pose_port, e))
    rx.setblocking(False)
    br = Bridge(args, cam, rect, starts, send, out)
    br.event("start", calib=args.calib, map=args.map, starts=starts, observe=args.observe, brick_watch=args.brick_watch)
    print("Camera bridge: following %s; camera %s; fixes to %s%s. Logs: %s" % (
        ", ".join("%s (start %.0f, %.0f, %.0f deg)" % (n, *s) for n, s in starts.items()), args.stream, args.to,
        " (OBSERVE ONLY: nothing is sent)" if args.observe else "", out), flush=True)
    import signal
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))   # kill / tmux: save first
    import sources as S
    src = S.StreamSource(args.stream, log=lambda s: print(s, flush=True))
    min_lag, skipped = None, 0
    try:
        for fr in src:
            lag = fr.t_arrived - fr.t_pi            # clock offset + transit + queue (the offset cancels below)
            min_lag = lag if min_lag is None else min(min_lag, lag)
            if br.ready and lag - min_lag > MAX_BACKLOG_S:
                skipped += 1
                br.skipped = skipped
                continue
            while True:
                try:
                    data, _ = rx.recvfrom(65536)
                except (BlockingIOError, OSError):
                    break
                try:
                    msg = json.loads(data.decode("utf-8", "replace"))
                except ValueError:
                    continue
                if isinstance(msg, dict) and msg.get("type") == "pose":
                    br.on_pose(msg)
            br.on_frame(fr.img, fr.t_pi - CAPTURE_S)
            if args.max_s and time.time() - t0 > args.max_s:
                break
    except KeyboardInterrupt:
        pass
    finally:
        src.closed = True
        s = br.summary()
        br.close()
        print("Camera bridge stopped: %s" % json.dumps(s["robots"]), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
