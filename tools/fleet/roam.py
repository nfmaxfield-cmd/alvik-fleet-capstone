#!/usr/bin/env python3
"""Roam: every robot drives continuously to random spots on the board, steered by the
overhead camera, never closer than --margin-in inches (default 3) to any edge.
For recording varied footage for the recognition studio, and a first real test of the
camera-in-the-loop system.

    # window 1 (laptop): the live camera tracker, sending camera fixes on UDP 5010
    python tools/live/track_live.py --stream st1maxfini.local:8000 --calib <floor_calibration.json>
    # window 2 (laptop):
    python tools/fleet/roam.py --pi st1maxfini.local --calibrate
    # practice against simulated robots (no camera, no robots needed):
    python tools/fleet/roam.py --dry-run --minutes 1

What it does:
  1. Waits for the robots, calibrates them (--calibrate; once per power-on), starts their
     step follower (`run follow`), and waits for a camera fix on every robot. It checks every
     robot is inside the board (--board, in the floor calibration's centimetres) before
     anything moves.
  2. For --minutes: whenever a robot is free, picks a random spot inside the safe area (the
     board minus the margin minus half a robot, so the robot's corners stay --margin-in
     inches inside), whose straight path stays clear of the other robots and where they are
     heading, and sends "turn to face it, drive there". The camera corrects each robot's own
     position several times a second, so it does not drift.
  3. Safety: a robot the camera has not seen for --lost-s (1.5 s) while moving is halted
     until it is seen again; a robot the camera sees outside the safe area is halted and
     sent back inwards; the robots' own front stop (set to 8 cm here) stops a drive when
     something is in the way, and that robot gets a new spot.
  4. Then it stops them and asks you to put objects on and off the board; Enter ends it.
Ctrl+C stops every robot at once. Logs go to logs/<date-time>-roam/ like the console's.
"""
import argparse
import datetime as dt
import json
import math
import os
import random
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import alvik_console as AC  # noqa: E402
from run_plan import UdpSource, Bot, wait_for  # noqa: E402

INCH = 2.54
ROBOT_HALF_DIAG_CM = 6.7        # turning centre to a shell corner (Alvik 9.5 cm square, turns on its centre)
FIRMWARE_MIN = "2026-10-18b"


def seg_dist(p, q, a, b):
    """Shortest distance between segments p-q and a-b (2D)."""
    def pt_seg(o, s, e):
        dx, dy = e[0] - s[0], e[1] - s[1]
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((o[0] - s[0]) * dx + (o[1] - s[1]) * dy) / L))
        return math.hypot(o[0] - s[0] - t * dx, o[1] - s[1] - t * dy)

    def cross(o, a_, b_):
        return (a_[0] - o[0]) * (b_[1] - o[1]) - (a_[1] - o[1]) * (b_[0] - o[0])
    d1, d2, d3, d4 = cross(p, q, a), cross(p, q, b), cross(a, b, p), cross(a, b, q)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return 0.0
    return min(pt_seg(p, a, b), pt_seg(q, a, b), pt_seg(a, p, q), pt_seg(b, p, q))


class Roamer:
    def __init__(self, args, link, source, names, log):
        self.a = args
        self.link = link
        self.source = source
        self.names = names
        self.log = log
        self.bots = {n: Bot(n) for n in names}
        for b in self.bots.values():
            b.cam = None          # latest camera fix dict (x, y, th, t) + arrival time
            b.cam_seen = 0.0
            b.state = "idle"      # idle, moving, halting, halted, done
            b.steps = {}          # step id -> dict(text, status, t_sent)
            b.drive_id = None
            b.target = None
            b.leg_start = None
            b.busy_until = 0.0
            b.legs = 0
            b.halts = 0
            b.fails = 0
            b.halt_t = 0.0
        link.session.listeners.append(self._listen)
        x0, y0, x1, y1 = args.board
        # + 1.5 cm for the camera's own error (0.5-0.8 cm on this floor) and the robot stopping a little past its spot
        m = args.margin_in * INCH + ROBOT_HALF_DIAG_CM + 1.5
        self.safe = (min(x0, x1) + m, min(y0, y1) + m, max(x0, x1) - m, max(y0, y1) - m)
        if self.safe[0] >= self.safe[2] or self.safe[1] >= self.safe[3]:
            raise SystemExit("The board is too small for that margin.")
        self.rng = random.Random(args.seed)
        self.events = []

    # --- incoming ----------------------------------------------------------------------
    def _listen(self, name, kind, rtype, vals):
        b = self.bots.get(name)
        if b is None:
            return
        t = AC.now()
        if kind == "M":
            b.said.append((t, vals))
            low = vals.lower()
            if low.startswith("status ") and "(firmware " in vals:
                k = vals.find("(firmware ")
                b.firmware = vals[k + 10:vals.find(")", k)]
            if "not calibrated" in low:
                b.calibrated = False
            return
        if rtype == "RUNINFO" and len(vals) >= 2:
            b.run_id, b.run_exp = vals[0], vals[1]
        elif rtype == "RUNEND" and len(vals) >= 2 and vals[0] == b.run_id:
            b.last_runend = (b.run_exp, vals[1])
            if b.run_exp == "follow":
                b.ended = vals[1]
        elif rtype == "STEP" and len(vals) >= 9:
            try:
                sid, state, ms = int(vals[1]), vals[2], int(float(vals[3]))
            except ValueError:
                return
            b.note_clock(t, ms)
            info = ",".join(vals[8:])
            if state in ("held", "halted"):
                b.controls = [c for c in b.controls if c["text"].split()[0] not in ("halt", "hold")]
                if b.state == "halting":
                    b.state = "halted"
                return
            st = b.steps.get(sid)
            if st is None:
                return
            if state in ("queued", "dup") and st["status"] == "sent":
                st["status"] = "acked"
            elif state == "bad":
                st["status"] = "failed"
                self.say("%s could not read %s (%s)" % (name, st["text"], info))
            elif state in ("done", "failed", "dropped"):
                st["status"] = state
                if state == "failed":
                    b.fails += 1
                    if b.state == "moving":
                        self.say("%s stopped short (%s): new spot" % (name, info or "failed"))
                if b.state == "moving" and (sid == b.drive_id or state != "done"):
                    b.state = "idle"
                    b.busy_until = t + (1.0 if state != "done" else self.rng.uniform(0.0, 1.5))
            elif state == "started" and st["status"] in ("sent", "acked"):
                st["status"] = "started"
        elif rtype == "FOLLOW" and len(vals) >= 10:
            try:
                b.note_clock(t, int(float(vals[1])))
                b.follow = dict(t=t, state=vals[3], x=float(vals[7]), y=float(vals[8]), th=float(vals[9]))
            except ValueError:
                pass

    def say(self, msg):
        line = "[%6.1f] %s" % (AC.now() - self.t0, msg) if hasattr(self, "t0") else msg
        print(line, flush=True)
        self.log.write(json.dumps(dict(t=round(AC.now(), 3), msg=msg)) + "\n")
        self.log.flush()

    # --- camera fixes --------------------------------------------------------------------
    def fixes(self):
        t = AC.now()
        for ev in self.source.poll():
            if ev.get("type") != "fix":
                continue
            b = self.bots.get(ev.get("robot"))
            if b is None:
                continue
            b.cam = ev
            b.cam_seen = t
            b.fix = ev
        for b in self.bots.values():
            ev = b.fix
            if ev is None or b.offset is None or t - b.fix_sent_t < 1.0 / self.a.fix_hz:
                continue
            ms = int(round((ev["t"] - b.offset) * 1000))
            th = ev.get("th")
            self.link.send_name(b.name, "fix %d %.2f %.2f %s %.2f %.2f" % (
                ms, ev["x"], ev["y"], "-" if th is None else "%.2f" % th, ev.get("sigma", 0.5), ev.get("sigma_th", 1.5)))
            b.fix, b.fix_sent_t = None, t
            b.fix_sent += 1

    # --- outgoing ----------------------------------------------------------------------------
    def send_step(self, b, text_fmt):
        sid = b.next_id
        b.next_id += 1
        text = text_fmt % sid
        b.steps[sid] = dict(text=text, status="sent", t_sent=AC.now())
        self.link.send_name(b.name, text)
        return sid

    def control(self, b, text):
        b.controls = [c for c in b.controls if c["text"].split()[0] != text.split()[0]]
        t = AC.now()
        b.controls.append(dict(text=text, t=t, t_last=t, sends=1))
        self.link.send_name(b.name, text)

    def resend(self):
        t = AC.now()
        for b in self.bots.values():
            for st in b.steps.values():
                if st["status"] == "sent" and t - st["t_sent"] > self.a.resend_s:
                    st["t_sent"] = t
                    self.link.send_name(b.name, st["text"])
            for c in b.controls:
                if t - c["t_last"] > self.a.resend_s:
                    c["t_last"] = t
                    self.link.send_name(b.name, c["text"])

    # --- choosing spots --------------------------------------------------------------------
    def inside(self, x, y, tol=0.0):
        s = self.safe
        return s[0] - tol <= x <= s[2] + tol and s[1] - tol <= y <= s[3] + tol

    def pick(self, b):
        p = (b.cam["x"], b.cam["y"])
        others = []
        for o in self.bots.values():
            if o is b or o.cam is None:
                continue
            q = (o.cam["x"], o.cam["y"])
            others.append((q, o.target if (o.state == "moving" and o.target) else q))
        s = self.safe
        best = None
        for _ in range(300):
            x = self.rng.uniform(s[0], s[2])
            y = self.rng.uniform(s[1], s[3])
            d = math.hypot(x - p[0], y - p[1])
            if d < self.a.min_leg_cm or d > self.a.max_leg_cm:
                continue
            clear = min([seg_dist(p, (x, y), q, r) for q, r in others] + [1e9])
            if clear >= self.a.clear_cm:
                return (x, y)
            if best is None or clear > best[0]:
                best = (clear, (x, y))
        return None

    def outside_by(self, x, y):
        """How far a point is outside the safe area (0 inside), cm."""
        s = self.safe
        dx = max(s[0] - x, 0.0, x - s[2])
        dy = max(s[1] - y, 0.0, y - s[3])
        return math.hypot(dx, dy)

    def start_leg(self, b):
        x0, y0, th = b.cam["x"], b.cam["y"], b.cam["th"]
        out = self.outside_by(x0, y0)
        bx0, by0, bx1, by1 = self.a.board
        edge = min(x0 - min(bx0, bx1), max(bx0, bx1) - x0, y0 - min(by0, by1), max(by0, by1) - y0)
        if out > 0 and edge < ROBOT_HALF_DIAG_CM + 1.0:
            # started (or put) near an edge: no turning on the spot here, its corners would swing over the
            # edge. Drive straight ahead out of the margin first, if it faces inwards.
            s = self.safe
            cx, cy = (s[0] + s[2]) / 2, (s[1] + s[3]) / 2
            hx, hy = math.cos(math.radians(th)), math.sin(math.radians(th))
            to_c = math.hypot(cx - x0, cy - y0) or 1.0
            if (hx * (cx - x0) + hy * (cy - y0)) / to_c < 0.3:
                if not getattr(b, "warned_out", False):
                    self.say("%s is near an edge facing outwards: turn it to face the middle" % b.name)
                    b.warned_out = True
                b.busy_until = AC.now() + 2.0
                return
            dist = 5.0
            while dist < 60 and self.outside_by(x0 + hx * dist, y0 + hy * dist) > 0:
                dist += 2.0
            dist += 5.0
            tgt = (x0 + hx * dist, y0 + hy * dist)
            b.drive_id = self.send_step(b, "fs %%d D %.2f %.2f %.2f %.2f %.2f %.2f 0 -"
                                        % (x0, y0, tgt[0], tgt[1], th, self.a.speed))
            b.target, b.leg_start, b.state, b.leg_out = tgt, AC.now(), "moving", out
            b.legs += 1
            return
        # inside the safe area, or close enough to an edge's margin that turning on the spot keeps every
        # corner on the board: a normal leg to a random spot (which is always inside the safe area)
        tgt = self.pick(b)
        if tgt is None:
            b.busy_until = AC.now() + 1.0
            return
        x0, y0 = b.cam["x"], b.cam["y"]
        h = math.degrees(math.atan2(tgt[1] - y0, tgt[0] - x0))
        self.send_step(b, "fs %%d T %.2f -" % h)
        b.drive_id = self.send_step(b, "fs %%d D %.2f %.2f %.2f %.2f %.2f %.2f 0 -" % (x0, y0, tgt[0], tgt[1], h, self.a.speed))
        b.target, b.leg_start, b.state, b.leg_out = tgt, AC.now(), "moving", out
        b.legs += 1

    def halt(self, b, why):
        if b.state in ("halting", "halted"):
            return
        self.control(b, "halt")
        b.state, b.halt_t, b.target = "halting", AC.now(), None
        b.halts += 1
        self.say("%s halted: %s" % (b.name, why))

    # --- the loop ----------------------------------------------------------------------------
    def step(self, stopping=False):
        t = AC.now()
        self.fixes()
        self.resend()
        for b in self.bots.values():
            fresh = b.cam is not None and t - b.cam_seen < self.a.lost_s
            if b.state == "moving":
                if not fresh:
                    self.halt(b, "camera lost it for %.1f s" % (t - b.cam_seen))
                elif self.outside_by(b.cam["x"], b.cam["y"]) > getattr(b, "leg_out", 0.0) + 2.0:
                    self.halt(b, "camera sees it %.1f, %.1f: outside the safe area" % (b.cam["x"], b.cam["y"]))
                elif t - b.leg_start > 8 + 3 * self.a.max_leg_cm / self.a.speed:
                    self.halt(b, "leg took too long")
            if b.state == "halting" and t - b.halt_t > 3.0:
                b.state = "halted"            # ack lost; it has stopped by now
            if b.state == "halted" and fresh and t - b.halt_t > 1.0:
                b.state = "idle"
                b.busy_until = t + 0.5
                for st in b.steps.values():
                    if st["status"] in ("sent", "acked", "started"):
                        st["status"] = "dropped"
            if stopping:
                continue
            if b.state == "idle" and fresh and t >= b.busy_until and b.cam.get("th") is not None:
                if not any(st["status"] in ("sent", "acked", "started") for st in b.steps.values()):
                    self.start_leg(b)


def make_dry_map(path, board, names):
    x0, y0, x1, y1 = board
    cx = (x0 + x1) / 2
    # as on the real board: one near each corner of one edge and one in the middle, backs to that edge
    starts = [[x0 + 12, y0 + 5.5, 90], [cx, y0 + 5.5, 90], [x1 - 12, y0 + 5.5, 90]]
    m = {"format": "alvik-fleet-map", "version": 1, "floor": {"x_min": x0, "x_max": x1, "y_min": y0, "y_max": y1},
         "robots": [{"name": n, "start": s} for n, s in zip(names, starts)], "tasks": {}}
    with open(path, "w") as f:
        json.dump(m, f, indent=1)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robots", default="Left,Center,Right")
    ap.add_argument("--minutes", type=float, default=4.0)
    ap.add_argument("--board", default="0,0,122.24,121.92",
                    help="board corners x0,y0,x1,y1 in the floor calibration's cm (default the 122.24 x 121.92 cm board "
                         "from 0,0)")
    ap.add_argument("--margin-in", type=float, default=3.0, help="closest any part of a robot may come to an edge, inches")
    ap.add_argument("--speed", type=float, default=8.0, help="driving speed, cm/s")
    ap.add_argument("--min-leg-cm", type=float, default=15.0)
    ap.add_argument("--max-leg-cm", type=float, default=60.0)
    ap.add_argument("--clear-cm", type=float, default=20.0, help="a path must stay this far (centre to centre) from the others'")
    ap.add_argument("--lost-s", type=float, default=1.5, help="halt a moving robot the camera hasn't seen for this long")
    ap.add_argument("--fixes", default="udp", help="udp[:PORT] (default 5010, where track_live.py sends)")
    ap.add_argument("--fix-hz", type=float, default=10.0)
    ap.add_argument("--resend-s", type=float, default=0.3)
    ap.add_argument("--front-cm", type=float, default=8.0,
                    help="robots' own front stop while driving (the plywood reads ~10 cm ahead, so 12 would stop every drive)")
    ap.add_argument("--calibrate", action="store_true", help="calibration run on every robot first (needed once per power-on)")
    ap.add_argument("--pi", help="Pi recorder address: phases are marked in the video")
    ap.add_argument("--no-objects", action="store_true", help="end after roaming (no objects phase)")
    ap.add_argument("--robot", action="append", default=[], help="robot IP, if robots aren't found automatically")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--dry-run", action="store_true", help="simulated robots and camera (tools/sim/fleet_sim.py)")
    a = ap.parse_args(argv)
    a.board = [float(v) for v in a.board.split(",")]
    names = [n.strip() for n in a.robots.split(",") if n.strip()]
    outdir = os.path.join("logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") + ("-roam-dryrun" if a.dry_run else "-roam"))
    os.makedirs(outdir, exist_ok=True)

    sim = None
    robot_ips = list(a.robot)
    port = int(a.fixes.partition(":")[2] or 5010)
    if a.dry_run:
        mp = os.path.join(outdir, "dry_map.json")
        make_dry_map(mp, a.board, names)
        robot_ips = robot_ips or ["127.0.0.%d" % (2 + k) for k in range(len(names))]
        cmd = [sys.executable, os.path.join(TOOLS, "sim", "fleet_sim.py"), "--map", mp, "--out", os.path.join(outdir, "sim"),
               "--seed", str(a.seed or 1), "--quiet", "--fixes-to", "127.0.0.1:%d" % port,
               "--max-s", str(a.minutes * 60 + 120)]
        sim = subprocess.Popen(cmd, stdout=open(os.path.join(outdir, "sim_stdout.txt"), "w"), stderr=subprocess.STDOUT)
        time.sleep(1.5)
    source = UdpSource(port)
    link = AC.Link(outdir, robot_ips)
    link.session.quiet = True
    log = open(os.path.join(outdir, "roam.jsonl"), "a")
    r = Roamer(a, link, source, names, log)
    pi = None
    if a.pi:
        from run_sheet import PiNotes
        pi = PiNotes(a.pi)

    def note(text):
        if pi and not pi.send(text):
            print("  (the Pi recorder didn't hear the mark '%s'; type it there if you want it)" % text)

    def pump(cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            r.fixes()
            if cond():
                return True
            time.sleep(0.05)
        return False

    code = 0
    try:
        print("Waiting for robots: %s ..." % ", ".join(names))
        if not pump(lambda: set(names) <= set(link.robots()), 30):
            raise SystemExit("Not heard from: %s" % ", ".join(sorted(set(names) - set(link.robots()))))
        for n in names:
            link.send_name(n, "stop")
        time.sleep(1.0)
        for n in names:
            link.send_name(n, "status")
            link.send_name(n, "set follow_look_cm %.1f" % a.front_cm)
        pump(lambda: all(r.bots[n].firmware for n in names), 5)
        for n in names:
            fw = r.bots[n].firmware
            if fw and fw < FIRMWARE_MIN:
                raise SystemExit("%s runs firmware %s; roaming needs %s or later." % (n, fw, FIRMWARE_MIN))
        if a.calibrate:
            note("phase calibration")
            print("Calibrating all robots at once (about a minute: blink, spin, roll) ...")
            for n in names:
                # spin only: at an edge the roll could head for the drop (a drop reads as open floor). The
                # firmware caps the roll at 3 cm minimum, so also ask for more room than it can ever find.
                link.send_name(n, "set calib_roll_cm 0")
                link.send_name(n, "set calib_roll_min_cm 99")
            time.sleep(0.3)
            for n in names:
                link.send_name(n, "calibrate")
            pump(lambda: all(getattr(r.bots[n], "last_runend", (None,))[0] == "calib" for n in names), 120)
            for n in names:
                le = getattr(r.bots[n], "last_runend", None)
                print("  %s: %s" % (n, le[1] if le else "no end report"))
        for attempt in range(3):
            for n in names:
                if r.bots[n].run_exp != "follow":
                    link.send_name(n, "run follow")
            if pump(lambda: all(r.bots[n].run_exp == "follow" or r.bots[n].calibrated is False for n in names), 4):
                break
        bad = [n for n in names if r.bots[n].calibrated is False]
        if bad:
            raise SystemExit("Not calibrated since power-on: %s. Start again with --calibrate." % ", ".join(bad))
        missing = [n for n in names if r.bots[n].run_exp != "follow"]
        if missing:
            raise SystemExit("Did not start following: %s" % ", ".join(missing))
        print("Waiting for the camera to see every robot (with a heading) ...")
        if not pump(lambda: all(r.bots[n].cam is not None and r.bots[n].cam.get("th") is not None for n in names), 20):
            raise SystemExit("The camera has not found: %s. Is track_live.py running and showing them?"
                             % ", ".join(n for n in names if r.bots[n].cam is None))
        x0, y0, x1, y1 = a.board
        for n in names:
            c = r.bots[n].cam
            on_board = min(x0, x1) <= c["x"] <= max(x0, x1) and min(y0, y1) <= c["y"] <= max(y0, y1)
            print("  %s: camera sees it at %.1f, %.1f cm, facing %.0f deg%s" % (
                n, c["x"], c["y"], c["th"], "" if on_board else "  <-- NOT on the board"))
            if not on_board:
                raise SystemExit("The camera puts %s off the board: the --board corners don't match this floor "
                                 "calibration. Nothing moved." % n)
        pump(lambda: all(r.bots[n].follow.get("state") not in (None, "nopose") for n in names), 6)
        note("phase roaming")
        r.t0 = AC.now()
        end = r.t0 + a.minutes * 60
        print("Roaming for %.1f minutes (safe area x %.0f-%.0f, y %.0f-%.0f cm). Ctrl+C stops everything."
              % (a.minutes, *r.safe[0::2], *r.safe[1::2]))
        last_report = r.t0
        while AC.now() < end:
            r.step()
            if AC.now() - last_report > 30:
                last_report = AC.now()
                r.say("legs so far: " + ", ".join("%s %d" % (n, r.bots[n].legs) for n in names))
            time.sleep(0.05)
        for n in names:
            r.control(r.bots[n], "halt")
        pump(lambda: False, 1.5)
        r.say("Roaming done. Legs: %s; camera halts: %s; stopped short: %s" % (
            ", ".join("%s %d" % (n, r.bots[n].legs) for n in names),
            ", ".join("%s %d" % (n, r.bots[n].halts) for n in names),
            ", ".join("%s %d" % (n, r.bots[n].fails) for n in names)))
        if not a.no_objects and not a.dry_run:
            for n in names:
                link.send_name(n, "stop")
            note("phase objects")
            input("\nNow put things on and off the board by hand (boxes, cups, tools, a hand on its own, objects "
                  "touching the robots; move a robot by hand now and then). Press Enter when done ... ")
            note("phase end")
        print("Done. Stop the Pi recorder now (Ctrl+C in its window). Logs: %s" % outdir)
    except KeyboardInterrupt:
        print("\nStopping every robot.")
        note("stopped early")
    except SystemExit as e:
        print(e)
        code = 1
    finally:
        try:
            link.close()
        except Exception:
            for n in names:
                link.send_name(n, "stop")
        source.close()
        if sim is not None:
            try:
                sim.wait(timeout=20)
            except subprocess.TimeoutExpired:
                sim.terminate()
            rp = os.path.join(outdir, "sim", "sim_report.json")
            if os.path.exists(rp):
                with open(rp) as f:
                    rep = json.load(f)
                print("Simulator truth: " + json.dumps({k: rep[k] for k in list(rep)[:12]}))
    return code


if __name__ == "__main__":
    sys.exit(main())
