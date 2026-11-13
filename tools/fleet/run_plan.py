#!/usr/bin/env python3
"""Run plan.json on the robots: releases each robot's steps as the steps in
its "after" list are reported done, resends steps until the robot acks them,
passes camera fixes on, and re-plans when something new is on the floor.

    python tools/fleet/run_plan.py --map map.json --plan plan.json            # the real robots
    python tools/fleet/run_plan.py --map map.json --plan plan.json --fixes udp  # + live camera fixes
    python tools/fleet/run_plan.py --map tools/fleet/examples/demo_map.json --dry-run
    python tools/fleet/run_plan.py --map ... --dry-run --brick auto@15 --wifi-drop Center@20:3

Robots need firmware 2026-10-18b or later (the `follow` experiment) and a
calibration since power-on (`calibrate` in the console, or --calibrate here).
Put each robot on its start spot (map.json "robots" start) first.

One program on the robots' port: this uses tools/alvik_console.py's Link, so
close the console (and run_sheet.py) first. Everything the robots send is saved
as in a console session (step.csv, follow.csv, raw.csv, ...) in --session
(default logs/<date-time>-plan/), plus:
  coordinator.jsonl   every step sent, ack, start, done, fix count, re-plan
  plan_0.json, ...    the plan in force (0 = the one given, then each re-plan)
  run_plan_summary.json  timings, resends, re-plans, dockings (robots' own reports)

Position source (--fixes): where camera poses come from (data-formats.md,
"Step follower"):
  none            no camera: each robot gets its plan start pose once (fpose)
                  and then drives on wheels + gyro alone;
  udp[:PORT]      JSON lines on UDP PORT (default 5010), e.g. from the live
                  link (tools/live/track_live.py) or the simulator: {"type": "fix", "robot", "t" (laptop
                  Unix time of the camera frame), "x", "y", "th" (deg),
                  "sigma" (cm), "sigma_th" (deg)} and {"type": "obstacle", "id",
                  "shape", "points" or "center"/"radius", "source"};
  replay:FILE     a CSV t,robot,x_cm,y_cm,heading_deg[,sigma_cm] replayed in
                  time (t in s from the start, or laptop Unix time).

--dry-run starts tools/sim/fleet_sim.py (simulated robots on 127.0.0.2..) and
runs against it with --fixes udp; at the end it prints the simulator's ground
truth: docking errors, closest robots, contact.
"""
VERSION = "2026-11-12e21"   # e21 (Nick 12 Nov, decisions 1 and 2): a blind mission robot's possible error is the ONE error shape (error_shape.Shape: the cone along its own track, one-sided, plus the disc) instead of a circle, for its stop (MIRLT_ERROR_SHAPE); and missions get the VEER stage where the lane allows (MIRLT_STAGES): when its shape would reach an edge within its next moves, it does not wait for the limit but turns back early along the recovery line to where the camera last read it (more room, and ground the camera reads), the lane's own check of the others kept. Before: "2026-11-12e19"   # e19 (the Fable audit, 12 Nov): SIGTERM ends a mission the Ctrl+C way (stop to every robot first, then the saves; install_signals). Before: "2026-11-12e18"   # e18 (Nick 12 Nov decisions 2, 4, 5, 6): never still in missions too: a held robot that still hears creeps (2-8 cm, scaled by its doubt, at most every derived pause) toward being found (where the camera last read it) and safety (the board's middle), inside its own held outline grown by the creep's reach so the others' plans stay valid (held_creep_watch; MIRLT_NEVER_STILL=0 the e16 hold). A lost robot first: a robot standing in its recovery's straight path steps aside (MIRLT_LOST_PRIORITY). Each robot's odometry trust (odo_trust) gives its blind error instead of the fixed 2 + 0.07 cm per cm, is checked when it is seen again, and lets a point the camera does not read stay on the list when an out-and-back from readable ground is allowed (MIRLT_BLIND_ALLOW). The trust records saved at any end (MIRLT_SAVE_CAL_AT_END). Before: "2026-11-11e16"   # e16 (6a, Claude's suggestion, Nick not asked yet; school 18:26 ended "every robot is out (none heard again within 90 s)", 18:54 by the keep-going cap): 6b: every robot whose roof-tag turn is not verified is crawled at the start, even one that needs no straightening (MIRLT_CRAWL_UNKNOWN=need: e15's way). 6a: every robot held never ends the run by itself: it holds them safely, says so every minute (NEEDS A HAND) and goes on as soon as one rejoins; the cap (--all-held-max 3 / --all-held-max-s 180) and the 90 s wait are the switch MIRLT_HELD_NEVER_END=0. (item 7, home 11 Nov 08:45): a tilt or drop trip backs out along the robot's own recorded track as far as the derived edge rule needs (observe.track_back_target: clearance with its uncertainty and own-vs-camera doubt, the stop where it ends), checked against the other robots, instead of a fixed 10 cm; the fixed back-off where the track is unknown or blocked (MIRLT_EDGE_BACKOUT=0: always the fixed 10 cm). Before: "2026-11-11e15"   # e15 (school 10 Nov 18:54): every facing check and re-check reads the tag CORRECTED by the turn this run knows (facing.corrected_tags: the tags file plus yaw_fix); a robot whose tag turn is unverified (not measured this run, nothing remembered; or remembered and a big turn would follow) is measured by the start's watched crawl before any straightening, and the crawl's measurement sets its yaw_fix; what is re-learned or crawled is remembered (--tag-heading-store, tag_heading_store). Before: "2026-11-10e14c"   # e14c (school 17:46): how old a fix may be and still count as seen comes from each robot's measured fix interval (blind_after; a slow detector treated like a slow camera; MIRLT_FIX_GAP_RULE=fixed the 0.6 s). e14b: a dark picture pauses every robot until the camera has adjusted (dark_watch). e14: a visit point whose tag was lost on the way is moved to the end of the robot's list once, then skipped (defer_current_point; MIRLT_RETRY_LOST=again the old way); the spot where it was lost kept as evidence. 2026-11-10e13 (e13): never give up blind (no BLIND_MAX_CM cap: it stops only when blind error + body + stop could reach an edge; MIRLT_BLIND_RULE=cap the old caps); no pictures: each blind robot's risk-based limit (safety_rules.no_picture_limit; MIRLT_NOPIC_RULE=off: e12's rules alone).
VERSION_MONITOR = "2026-11-12mc"   # (Mission Control build, 12 Nov) write_live ticks the Pi efficiency monitor's feed (monitor_feed.json: the loop period; read-only)
VERSION_B = "2026-11-10b"   # 2026-11-10b (e12 item 9): a lock-only fix never resets a pose (MIRLT_LOCK_GUARD=0: as before); a dwell broken by a halt where the robot did not move keeps its start (MIRLT_DWELL_KEEP=0). 2026-11-10a (e12): the bridge's "camera" messages (no pictures / pictures again) said and logged. No version before
LOCK_GUARD = __import__("os").environ.get("MIRLT_LOCK_GUARD", "1") != "0"
DWELL_KEEP = __import__("os").environ.get("MIRLT_DWELL_KEEP", "1") != "0"

import argparse
import csv
import datetime as dt
import json
import math
import os
HERE_ = os.path.dirname(os.path.abspath(__file__))
if HERE_ not in __import__("sys").path:
    __import__("sys").path.insert(0, HERE_)
import safety_group as SG          # noqa: E402  (e18: the safety switches' automatic defaults in one place)
HELD_NEVER_END = SG.flag("MIRLT_HELD_NEVER_END")   # (e16, 6a) every robot held never ends the run
NEVER_STILL = SG.flag("MIRLT_NEVER_STILL")         # (e18, Nick 12 Nov decision 2) held robots creep in missions too
LOST_PRIORITY = SG.flag("MIRLT_LOST_PRIORITY")     # (e18, 4) a robot in a lost robot's recovery path steps aside
BLIND_ALLOW = SG.flag("MIRLT_BLIND_ALLOW")         # (e18, 5) each robot's measured odometry trust
SAVE_CAL_AT_END = SG.flag("MIRLT_SAVE_CAL_AT_END")  # (e18, 6) the trust records saved at any end
CREEP_MAX_CM = 8.0       # (e18) as freeroam_core.CREEP_MAX_CM (CHOSEN there; DOE noted there)
CREEP_MIN_CM = 2.0
CREEP_UNC0_CM = 5.0
CREEP_V_MIN, CREEP_V_MAX = 2.0, 5.0
CREEP_REACH_CM = CREEP_MAX_CM   # (e18) CHOSEN: a held robot creeps within this of where it was held (its held outline is
                                # grown by it, so the others' plans never meet it). DOE: yes, with CREEP_MAX_CM
import queue
import re
import socket
import statistics
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import alvik_console as AC  # noqa: E402
import fleetmap as FM       # noqa: E402
import session_checks as SC  # noqa: E402
import planner as P         # noqa: E402
import engine_checks as EC  # noqa: E402
import calib_room as CR     # noqa: E402   (calibration room and roll caps, 2026-10-29)
import front_stop as FSTOP  # noqa: E402   (3 readings, the floor check, unconfirmed obstacles: 2026-10-29 16:2x)

MIN_FIRMWARE = "2026-10-18b"
BORDER4_FIRMWARE = "2026-10-27a"     # the four-sided virtual border (border.h) and the second WiFi network
OBS_N_FIRMWARE = "2026-10-28a"       # obs_n (front stops counted in sensor readings), the arc and dance steps, boot ID
                                     # in `status`
OBSTACLE_READINGS = FSTOP.STOP_READINGS   # 3: Nick, 2026-10-29 16:2x (rules audit rec 1), for every robot. Was 2 (Nick,
                        # brief 2026-10-28). Replay of the five DOE releases (tools/analysis/front_stop_replay.py): 164
                        # false stops in 232 drives with 2 in a row at 12 cm, 10 with 3, 0 with 3 and Left at 10 cm. Cost:
                        # one more reading (~66 ms, 0.5-0.8 cm nearer a real brick; tools/sim/brick_reach.py). DOE note:
                        # 2 / 3 readings on the board (front_stop.py STOP_READINGS); docs/doe-backlog.md
OWN_POSE_WINDOW_S = 10.0   # Nick (2026-10-27 18:26, Q17): after a side feel finds a barrier, the robot's own pose stands
                           # (the camera guard leaves it alone) for at most this long. No DOE: Nick's limit
STATUS_EVERY_S = 20.0   # during a run, ask every robot's `status` this often (battery). No DOE: a polling period
PING_EVERY_S = 2.0      # start-up: re-ping the robots not heard yet this often (15:27: one lost ping, 20 s of silence).
                        # No DOE: a retry period
BORDER_CM = 6.0     # chosen: the robots' own border holds their centre this far inside an open side: the body's half
                    # width (4.75 cm) plus 1.25 cm. The planner keeps the centre r_turn + safety + open margin (about
                    # 11 cm) inside, so it trips only on a pose error of about 5 cm. DOE note: 5 / 6 / 8 cm on the
                    # school table (border stops per hour against edge-guard trips); docs/doe-backlog.md
ACTIVE = ("sent", "acked", "started")


def now():
    return AC.now()


# ----------------------------------------------------------------------------
# Position sources
# ----------------------------------------------------------------------------

class NullSource:
    live = False

    def poll(self):
        return []

    def close(self):
        pass


class UdpSource:
    """JSON lines on a UDP port: camera fixes and obstacle reports."""
    live = True

    def __init__(self, port, host="0.0.0.0"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((host, port))
        except OSError as e:
            raise SystemExit("Fix port %d is in use (another run_plan?): %s" % (port, e))
        self.sock.setblocking(False)
        self.bad = 0

    def poll(self):
        out = []
        while True:
            try:
                data, _ = self.sock.recvfrom(65536)
            except (BlockingIOError, socket.timeout):
                break
            except OSError:
                break
            try:
                msg = json.loads(data.decode("utf-8", "replace"))
                if isinstance(msg, dict) and msg.get("type") in ("fix", "obstacle", "camera"):   # (e12) camera: no pictures
                    out.append(msg)
            except ValueError:
                self.bad += 1
        return out

    def close(self):
        self.sock.close()


class ReplaySource:
    """CSV of camera poses replayed in time: t,robot,x_cm,y_cm,heading_deg[,sigma_cm]."""
    live = True

    def __init__(self, path):
        with open(path, newline="") as f:
            rows = list(csv.DictReader(f))
        self.rows = sorted(rows, key=lambda r: float(r["t"]))
        self.t0 = None
        self.k = 0

    def poll(self):
        if self.t0 is None:
            self.t0 = now()
        out = []
        while self.k < len(self.rows):
            r = self.rows[self.k]
            t = float(r["t"])
            t_abs = t if t > 1e9 else self.t0 + t
            if t_abs > now():
                break
            self.k += 1
            out.append({"type": "fix", "robot": r["robot"], "t": t_abs, "x": float(r["x_cm"]), "y": float(r["y_cm"]),
                        "th": float(r["heading_deg"]), "sigma": float(r.get("sigma_cm") or 0.5), "sigma_th": 1.5})
        return out

    def close(self):
        pass


def make_source(spec):
    if not spec or spec == "none":
        return NullSource()
    if spec.startswith("udp"):
        _, _, port = spec.partition(":")
        return UdpSource(int(port or 5010))
    if spec.startswith("replay:"):
        return ReplaySource(spec.split(":", 1)[1])
    raise SystemExit("--fixes must be none, udp[:PORT] or replay:FILE")


# ----------------------------------------------------------------------------
# Step messages (format: docs/data-formats.md "Step follower", firmware follow.h)
# ----------------------------------------------------------------------------

def _f(v):
    return "-" if v is None else "%.2f" % v


def border_commands(m, border_cm):
    """`set` commands for the robots' own four-sided border (workspace.firmware_params)."""
    import workspace as WS
    p = WS.firmware_params(m, border_cm)
    order = ["bd_x0", "bd_x1", "bd_y0", "bd_y1"] + sorted(k for k in p if k.startswith(("bd_k_", "bd_lo_", "bd_hi_")))
    return ["set %s %g" % (k, p[k]) for k in order] + ["set border_cm %g" % p["border_cm"], "set bd_on 1"]


def step_command(sid, st, wait_ms=None):
    t = st["type"]
    if t == "arc":
        # firmware 2026-10-28a follow step C: round the circle centre (cx, cy), radius r, from angle a0 through sweep
        # (deg, + anticlockwise), at speed cm/s along the circle; fmin as for a turn
        a = st["arc"]
        return "fs %d C %.2f %.2f %.2f %.2f %.2f %.2f %s" % (sid, a["c"][0], a["c"][1], a["r"], a["a0_deg"],
                                                             a["sweep_deg"], st["speed"], _f(st.get("front_min_cm")))
    if t == "dance":
        return "fs %d Z %d" % (sid, int(st.get("dance_ms", 0)))     # 0 = until switched off
    if t == "wait" and wait_ms is not None:
        return "fs %d W %d" % (sid, int(wait_ms))
    if t in ("turn", "park"):
        return "fs %d T %.2f %s" % (sid, st["to"][2], _f(st.get("front_min_cm")))
    if t == "drive":
        return "fs %d D %.2f %.2f %.2f %.2f %.2f %.2f %d %s" % (
            sid, st["from"][0], st["from"][1], st["to"][0], st["to"][1], st["from"][2], st["speed"],
            1 if st.get("reverse") else 0, _f(st.get("front_min_cm")))
    if t == "approach" and st.get("touch"):
        # backing in (switches at the back, firmware sw_back 1): creep back until a switch clicks, forward gap_cm
        return "fs %d P %.2f %.2f %.2f" % (sid, st["to"][2], st["max_travel_cm"], st.get("back_cm", 0.0))
    if t == "approach":
        return "fs %d A %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %s" % (
            sid, st["from"][0], st["from"][1], st["to"][0], st["to"][1], st["to"][2], st["speed"],
            st["handover_cm"] + st.get("bumper_cm", 0.0), st["gap_cm"] + st.get("bumper_cm", 0.0), st["max_travel_cm"],
            _f(st.get("front_min_cm")))
    if t == "wait":
        ms = int(round((st["t_end"] - st["t_start"]) * 1000)) if st["phase"] in ("dwell", "pause") else 0
        return "fs %d W %d" % (sid, ms)
    if t == "home":      # creep into a home dock until both nose switches click (firmware dock.h)
        w = st["wall"]
        return "fs %d K %.2f %.2f %.2f %.2f %.2f" % (sid, w[0], w[1], w[2], st["max_travel_cm"], st.get("back_cm", 0.0))
    if t == "touch":     # creep until a nose switch clicks (touch probe)
        return "fs %d P %.2f %.2f %.2f" % (sid, st["to"][2], st["max_travel_cm"], st.get("back_cm", 0.0))
    raise ValueError("unknown step type %r" % t)


# ----------------------------------------------------------------------------
# One robot as the coordinator sees it
# ----------------------------------------------------------------------------

class Bot:
    def __init__(self, name):
        self.name = name
        self.next_id = 1
        self.ids = []           # per step of the current plan: robot step id or None
        self.stat = []          # per step: dict(status, t_sent, t_ack, t_started, t_done, text, info)
        self.id_index = {}      # step id -> index in the current plan
        self.pose = None        # (x, y, th_deg) from the robot's own reports
        self.pose_t = 0.0
        self.follow = {}        # latest FOLLOW record
        self.offsets = []       # (t_recv, t_recv - ms / 1000), for robot ms <-> laptop time
        self.offset = None
        self.fix = None         # newest camera fix not yet sent
        self.fix_sent_t = 0.0
        self.fix_sent = 0
        self.controls = []      # dict(text, t, until): hold/halt resent until confirmed
        self.firmware = None
        self.run_id = None
        self.run_exp = None
        self.ended = None       # RUNEND result once its follow run ended
        self.calibrated = None
        self.calib_look = None  # its last CALIB_LOOK record (spin / roll / reason: what it skipped for lack of room)
        self.said = []
        self.resends = 0
        self.battery = None     # % from its status / RUNINFO
        self.boot = None        # boot ID (CLOCK records, RUNINFO run ids, status from 2026-10-28a)
        self.boot_start = None  # its boot ID when this run started (a new one mid-run: it restarted)
        self.last_heard = None  # laptop time of its latest record of any kind
        self.params = {}        # PARAM rows of its current run, and `get` answers: name -> value
        self.out = None         # why it is out of this run (silent, restarted): a fixed obstacle for the others
        self.own_until = 0.0    # the camera guard leaves it alone until then (after a barrier find)

    def note_clock(self, t_recv, ms):
        o = t_recv - ms / 1000.0
        self.offsets.append((t_recv, o))
        if len(self.offsets) > 400:
            self.offsets = self.offsets[-300:]
        # the fastest packet had the least delay: lower envelope over the last 30 s
        recent = [v for t, v in self.offsets if t > t_recv - 30.0]
        self.offset = min(recent) if recent else o


# ----------------------------------------------------------------------------
# The coordinator
# ----------------------------------------------------------------------------

class Coordinator:
    def __init__(self, args, m, plan, link, source, outdir):
        self.args = args
        self.m = FM.normalize(m)
        self.plan = plan
        self.link = link
        self.source = source
        self.outdir = outdir
        # (11-05h) where the cameras read the roof tags: the mission's start map (--readmap) learned on during the run;
        # no start map: every cell taken as readable until evidence says otherwise
        import observe as OB
        f_ = self.m["floor"]
        self.readmap = OB.ReadMap(f_["x_max"] - f_["x_min"], f_["y_max"] - f_["y_min"], x0=f_["x_min"], y0=f_["y_min"])
        rmp = getattr(args, "readmap", "") or ""
        try:
            if rmp and os.path.exists(rmp):
                with open(rmp) as fh:
                    self.readmap = OB.ReadMap.from_json(json.load(fh))
            else:
                self.readmap.seed(lambda x_, y_: 0.9, "no start map: taken as readable")
        except (OSError, ValueError, KeyError):
            self.readmap.seed(lambda x_, y_: 0.9, "no start map: taken as readable")
        self.names = list(plan["robots"])
        self.bots = {n: Bot(n) for n in self.names}
        self.q = queue.Queue()
        link.session.listeners.append(self._listen)
        self.log_f = open(os.path.join(outdir, "coordinator.jsonl"), "a")
        self.outdir_ = outdir                    # (e13) sources.csv goes here
        self.plan_no = 0
        self.plan_start = None
        self.replans = []
        self.retries = 0
        self.next_retry = math.inf
        self.improve_at = None
        self.obstacles_seen = set()
        self.pending_replan = None   # (reason, deadline): a re-plan to do soon (e.g. after a halted robot moved on)
        self.n_sensor_obstacles = 0
        self.planning = None         # a re-plan being computed in a worker thread
        self.limit = {}              # while planning: robot -> first step index it may not be sent
        self.deferred = []           # re-plan requests that came while one was being computed
        self.abort = None
        self.dockings = []
        self.change = None           # tools/site/change.py: does a landmark disagree with the locked site map?
        self.homes = getattr(args, "homes", None) or {}   # robot -> home dock (--home): creep in at the end
        if getattr(args, "site_map", None):
            sys.path.insert(0, os.path.join(TOOLS, "site"))
            import change as SITE_CHANGE
            self.change = SITE_CHANGE.ChangeCheck(args.site_map, log_path=os.path.join(outdir, "site_changes.jsonl"),
                                                  say=lambda msg: self.say("site map: " + msg))
        self.plan_obstacle_keys = {self.obstacle_key(o) for o in self.m.get("obstacles", [])}
        self.save_plan()
        self._install(plan, keep={}, first={}, old=None)

    # -- plumbing -------------------------------------------------------------
    def _listen(self, name, kind, rtype, vals):
        self.q.put((now(), name, kind, rtype, vals))

    def log(self, event, **kw):
        rec = {"t": round(now(), 3), "event": event}
        rec.update(kw)
        try:
            self.log_f.write(json.dumps(rec) + "\n")
            self.log_f.flush()
        except OSError as e:
            # 2026-10-27 22:03: the card filled and this flush crashed run_plan mid-drive. Now: the run ends as on a
            # FAIL (the loop below sees abort, halts everyone; the link's close sends stop), no crash
            if not self.__dict__.get("abort"):
                self.abort = "the Pi's disk is full (%s): stopping everyone" % e
                print("[%s]" % self.abort, flush=True)

    def disk_watch(self):
        """Every 5 s: the card's free space; under diskguard.CARD_HARD_GB the run ends (every robot halted). The loop
        around run_plan (DOE loop, mission_run) stops first at CARD_STOP_GB; this is the last resort inside a drive."""
        t = now()
        if t < self.__dict__.get("_disk_next", 0.0):
            return
        self._disk_next = t + 5.0
        try:
            import diskguard as DG
        except ImportError:
            return
        g = DG.free_gb(self.outdir)
        if g is not None and g < DG.CARD_HARD_GB and not self.abort:
            self.abort = "the Pi's card is nearly full (%.2f GB free, under %.1f GB): stopping everyone" % (
                g, DG.CARD_HARD_GB)
            self.say(self.abort)
            self.halt_all()

    def say(self, msg):
        t0 = getattr(self, "t_run", None)
        t = now() - t0 if t0 else 0.0
        print("[%6.1f] %s" % (t, msg), flush=True)
        self.log("message", text=msg)

    def message_row(self, t, name, text):
        """Every M line a robot says (status, led replies, awake, ...), to messages.csv (none were kept before)."""
        if getattr(self, "_msg_f", None) is None:
            self._msg_f = open(os.path.join(self.outdir, "messages.csv"), "a", newline="")
            self._msg_w = csv.writer(self._msg_f)
            if self._msg_f.tell() == 0:
                self._msg_w.writerow(["t", "robot", "text"])
        self._msg_w.writerow(["%.3f" % t, name, text])
        self._msg_f.flush()

    def note_boot(self, b, boot, how):
        if not boot:
            return
        if b.boot and boot != b.boot:
            self.log("boot_changed", robot=b.name, old=b.boot, new=boot, how=how)
            if self.t_run_started() and b.boot_start and boot != b.boot_start and not b.out:
                self.retire(b, "restarted mid-run (boot %s, was %s): it lost its calibration and its steps" % (boot,
                                                                                                          b.boot_start),
                            kind="restarted")
                b.boot_back = boot
        b.boot = boot

    def t_run_started(self):
        return getattr(self, "t_run", None) is not None and self.plan_start is not None

    def save_plan(self):
        P.save_plan(self.plan, os.path.join(self.outdir, "plan_%d.json" % self.plan_no))

    def steps(self, name):
        return self.plan["robots"][name]["steps"]

    def send(self, name, text):
        self.link.send_name(name, text)

    # -- incoming -------------------------------------------------------------
    def drain(self):
        while True:
            try:
                t, name, kind, rtype, vals = self.q.get_nowait()
            except queue.Empty:
                return
            b = self.bots.get(name)
            if b is None:
                continue
            b.last_heard = t
            if b.out and getattr(b, "out_kind", None) == "recovering" and rtype == "STEP":
                # (e07) a robot on its reserved recovery is out of the plan: its STEP reports go to the recovery alone
                # (rehearsal: through out_heard it never rejoined; through on_step its old steps crashed the run)
                self.recovering_step(b, t, vals)
                continue
            if b.out and not getattr(b, "rechecking", False):
                # (while it is checked again, its records are read as any robot's: its `led` answers, status, RAW for
                # the floor check; before 30 Oct a held robot's re-check could not see its own facing check's answers)
                self.out_heard(b, t, kind, rtype, vals)
                continue
            if kind == "M":
                b.said.append((t, vals))
                self.message_row(t, name, vals)
                low = vals.lower()
                st_ = EC.parse_status(vals) if low.startswith("status ") else None
                if st_:
                    b.firmware = st_["firmware"]
                    b.battery = st_["battery"]
                    if st_["boot"]:
                        self.note_boot(b, st_["boot"], "status")
                elif low.startswith("status "):
                    k = vals.find("(firmware ")
                    if k >= 0:
                        b.firmware = vals[k + 10:vals.find(")", k)].split(",")[0]
                if " = " in vals and not low.startswith("status"):
                    nm, _, rest = vals.partition(" = ")
                    try:
                        b.params[nm.strip()] = float(rest.split()[0])
                    except (ValueError, IndexError):
                        pass
                if "not calibrated" in low:
                    b.calibrated = False
                if self.plan_start is not None and ("follow:" in low or "turn timed out" in low or "stopped" in low):
                    self.say("%s: %s" % (name, vals))
                continue
            if rtype == "CLOCK" and len(vals) >= 3:
                self.note_boot(b, vals[2].strip().upper(), "clock")
                continue
            if rtype == "PARAM" and len(vals) >= 3:
                try:
                    b.params[vals[1]] = float(vals[2])
                except ValueError:
                    pass
                continue
            if rtype == "RUNINFO" and len(vals) >= 2:
                b.run_id, b.run_exp = vals[0], vals[1]
                bt = EC.boot_of_run(vals[0])
                if bt:
                    self.note_boot(b, bt, "runinfo")
                if len(vals) >= 6:
                    try:
                        b.battery = int(float(vals[5]))
                    except ValueError:
                        pass
                if vals[1] == "follow":
                    b.params = {}          # this run's PARAM rows follow
                if len(vals) >= 10 and vals[1] != "calib":     # a calib run logs its sign before checking it
                    b.calibrated = vals[9].strip() not in ("0", "")
                continue
            if rtype == "RUNEND" and len(vals) >= 2:
                if vals[0] == b.run_id:
                    b.last_runend = (b.run_exp, vals[1])
                    if b.run_exp == "follow":
                        b.ended = vals[1]
                        self.log("runend", robot=name, result=vals[1])
                continue
            if rtype == "CALIB_LOOK" and len(vals) >= 10:
                b.calib_look = dict(result=vals[9])
                if len(vals) >= 15:
                    b.calib_look.update(spin=vals[12], roll=vals[13], reason=vals[14])
                # what its own front sensor saw (firmware look.h): why a roll was skipped, in numbers (30 Oct: both
                # robots "no roll" on a 126 cm board the laptop's caps allowed 15 cm on)
                for k, i in (("near_cm", 2), ("heading_deg", 3), ("room_cm", 5), ("cap_cm", 6), ("live_cm", 7),
                             ("roll_cm", 8), ("room8", 10), ("caps", 11)):
                    if len(vals) > i:
                        b.calib_look[k] = vals[i]
                self.log("calib_look", robot=name, **b.calib_look)
                continue
            if rtype == "RAW":
                self.tilt_feed(b, t, vals)     # (11-05h, protection 2) the standing tilt
                self.fp_raw(b, t, vals)        # (e09, B2) the robot's filter
            if rtype == "RAW" and getattr(self, "floor_raw", None) is not None and name in self.floor_raw:
                self.floor_feed(b, vals)
            if rtype == "RAW" and self.change is not None:
                self.site_check(b, t, vals)
                continue
            if rtype == "STEP" and len(vals) >= 9:
                self.on_step(b, t, vals)
            elif rtype == "FOLLOW" and len(vals) >= 17:
                if b.run_exp is None and b.ended is None:
                    # its RUNINFO lost over WiFi: FOLLOW records only come from a follow run (Monday sweep 2026-10-28:
                    # at 15 % packet loss the start-up said "did not start their follow run" for a robot that had)
                    b.run_id, b.run_exp = vals[0], "follow"
                    self.log("follow_seen", robot=name, run=vals[0], why="RUNINFO not heard; FOLLOW records are")
                self.on_follow(b, t, vals)

    # -- (e09, Phase B2) one filter per robot: fed by RAW and the camera fixes; its pose and uncertainty for the edge
    #    watch, and its position sent to the robot in camera gaps (the planner plans from the robot's own pose)
    def fp_on(self):
        import fused_pose as FPm
        return (getattr(self.args, "pose_source", None) or FPm.POSE_SOURCE) == "ekf"

    def fp_raw(self, b, t, vals):
        import fused_pose as FPm      # (the filter runs either way; with --pose-source blend it is only logged)
        cols = self.link.session.headers.get("RAW") or []
        if not cols:
            return
        row = dict(zip(cols, vals))
        try:
            v, w = FPm.raw_true(row)
            still = abs(float(row.get("wl_rpm") or 0)) < FPm.STILL_RPM and abs(float(row.get("wr_rpm") or 0)) < \
                FPm.STILL_RPM
            ln = []
            for k in ("lineL", "lineC", "lineR"):
                try:
                    ln.append(float(row.get(k)))
                except (TypeError, ValueError):
                    pass
            fp = b.__dict__.setdefault("fp", FPm.FusedPose(b.name, gyro_heading=getattr(self.args, "gyro_heading",
                                                                                          None) or None))
            gz = row.get("gz")
            ax = row.get("ax")
            fp.raw(t, v, w, float(gz) if gz not in (None, "") else None, still,
                   float(ax) if ax not in (None, "") else None, None, tuple(ln) if len(ln) == 3 else None)
            if BLIND_ALLOW:
                pt = getattr(b, "raw_prev_t", None)       # (e18, 5) its wheels and yaw rate drive its trust shadows
                if pt is not None and 0.0 < t - pt < 0.5:
                    self.trust_of(b).motion(t, float(v) * (t - pt), float(w) * (t - pt))
                b.raw_prev_t = t
        except (TypeError, ValueError):
            return

    def fp_fix(self, b, ev):
        import fused_pose as FPm
        fp = b.__dict__.setdefault("fp", FPm.FusedPose(b.name, gyro_heading=getattr(self.args, "gyro_heading",
                                                                                      None) or None))
        src = str(ev.get("src") or "camera")
        kind = "tags" if src.startswith("tag_bridge") and not src.endswith(":lock") else src.split(":")[-1]
        kind = kind if kind in FPm.SRC_SIGMA else "camera"
        try:
            fp.fix(float(ev.get("t", now())), float(ev["x"]), float(ev["y"]),
                   None if ev.get("th") is None else float(ev["th"]), ev.get("sigma"), kind, now=now())
        except (TypeError, ValueError, KeyError):
            pass
        if BLIND_ALLOW and b.pose is not None and getattr(b, "blind", None) is None:
            try:                                         # (e18, 5) the camera against its shadow dead reckoning
                tf = float(ev.get("t", now()))
                own = self.pose_at(b, tf)
                if now() - (getattr(b, "raw_prev_t", None) or -1e9) > 1.0:
                    self.trust_of(b).own_pose(now(), b.pose[0], b.pose[1], b.pose[2])   # no RAW: its own pose's steps
                self.trust_of(b).fix(tf, float(ev["x"]), float(ev["y"]), own[3] if own else b.pose[2])
            except (TypeError, ValueError, KeyError):
                pass

    def fp_live(self, b):
        fp = getattr(b, "fp", None)
        return self.fp_on() and fp is not None and fp.started and fp.last_raw_t is not None and \
            now() - fp.last_raw_t < 1.0

    GAP_FWD_S = 1.0           # as freeroam_core.GAP_FWD_S (CHOSEN)
    GAP_FWD_MAX_CM = 8.0

    def gap_forward(self):
        """(e09, B2) A robot the cameras have not fixed for over a second (the far half of the school table with tags
        only, a dropout): the filter's position (wheels, raw gyro, learned wheel scale) goes to it as a fix every
        GAP_FWD_S while the filter's uncertainty is under GAP_FWD_MAX_CM."""
        t = now()
        for b in self.bots.values():
            if b.out or b.offset is None or not self.fp_live(b):
                continue
            if t - (getattr(b, "cam_t", None) or -1e9) < 1.0 or t - getattr(b, "gap_fwd_t", -1e9) < self.GAP_FWD_S:
                continue
            u = b.fp.unc_cm(t)
            if u is None or u > self.GAP_FWD_MAX_CM:
                continue
            q = b.fp.pose(t)
            b.gap_fwd_t = t
            ms = int(round((t - b.offset) * 1000))
            self.send(b.name, "fix %d %.2f %.2f - %.2f %.2f" % (ms, q[0], q[1], max(1.0, u / 2.0), 5.0))
            self.n_gap_fwd = getattr(self, "n_gap_fwd", 0) + 1
            if self.n_gap_fwd % 50 == 1:
                self.log("gap_forward", robot=b.name, x=round(q[0], 2), y=round(q[1], 2), unc_cm=round(u, 2))

    def tilt_feed(self, b, t, vals):
        """(11-05h, protection 2; home 5 Nov: Center stood tilted 3.4 deg, under the 5 deg tilt stop, for 64 min) The
        robot's own roll / pitch: tilted over observe.STAND_TILT_DEG for STAND_TILT_S while standing = a hazard,
        handled as a tilt trip (a discovered edge, kept out; it backs away only with camera-confirmed room)."""
        import observe as OB
        cols = self.link.session.headers.get("RAW") or []
        if not cols or b.out or getattr(b, "backoff", None):
            return
        row = dict(zip(cols, vals))
        try:
            rows_ = b.__dict__.setdefault("raw_ev", {}).setdefault("rows", [])
            fr_ = row.get("dC_cm")
            rows_.append((t, float(row.get("ax") or 0.0), float(fr_) if fr_ not in (None, "") else None))
            del rows_[:-600]
            w = b.__dict__.setdefault("twatch", OB.TiltWatch())
            ev = w.feed(t, float(row["roll_deg"]), float(row["pitch_deg"]), float(row.get("v_cms") or 0.0))
        except (KeyError, ValueError, TypeError):
            return
        if ev and getattr(self, "t_run", None) and not getattr(self, "straightening", False):
            self.log("tilt_hazard", robot=b.name, **ev)
            self.send(b.name, "halt")
            i = next((k for k, st in enumerate(b.stat) if st["status"] in ACTIVE), -1)
            self.trip_back_off(b, i, b.pose, "standing tilt %.1f deg for %.1f s (tilted: a wheel over an edge?)" % (
                ev["dev_deg"], ev["for_s"]), "tilt")

    def fast_done(self, b, i, st, t):
        """(11-05h, 3') A drive step reported done: did it happen? observe.move_outcome fuses its own pose (the wheels),
        the accelerometer and front range during the step (RAW), the time against what is physically possible and the
        camera only when fresh; impossibility decides alone. (3b) the failure is remembered: the re-plan starts from
        where it is, so the same step is not sent again unchanged; three at one spot = a hazard kept out."""
        import observe as OB
        stp = self.steps(b.name)[i]
        if stp.get("type") != "drive" or not st.get("t_started"):
            return False
        L = math.hypot(stp["to"][0] - stp["from"][0], stp["to"][1] - stp["from"][1])
        took = t - st["t_started"]
        p0 = getattr(b, "step_pose0", {}).get(i)
        wheel = math.hypot(b.pose[0] - p0[0], b.pose[1] - p0[1]) if (p0 and b.pose) else None
        h = getattr(b, "cam_hist", None) or []
        moved, fresh = None, bool(h and getattr(b, "cam_t", None) is not None and t - b.cam_t <= self.blind_after(b))
        if fresh:
            before = [q for q in h if q[1] <= st["t_started"]]
            q0 = before[-1] if before else h[0]
            moved = math.hypot(h[-1][2] - q0[2], h[-1][3] - q0[3])
        ev = getattr(b, "raw_ev", None) or {}
        acc = None
        ax = [a for tt, a, fr in ev.get("rows", []) if tt >= st["t_started"]]
        if len(ax) >= 5:
            a = sorted(ax)
            acc = max(abs(v - a[len(a) // 2]) for v in a) >= 0.03
        fr = [f_ for tt, a, f_ in ev.get("rows", []) if tt >= st["t_started"] and f_ is not None and 0 < f_ < 60]
        out = OB.move_outcome(L, took, wheel_cm=wheel, accel_rolling=acc,
                              front_change_cm=(fr[0] - fr[-1]) if len(fr) >= 4 else None,
                              front_expect_cm=(-L if stp.get("reverse") else L) if len(fr) >= 4 else None,
                              cam_moved_cm=moved, cam_fresh=fresh, robot_info=st.get("info") or "")
        if out["verdict"] == "moved":
            return False
        self.n_fast_done = getattr(self, "n_fast_done", 0) + 1
        fm = b.__dict__.setdefault("fmem", OB.FailMemory())
        pos = b.pose or stp["from"]
        fm.add(t, "drive", -1 if stp.get("reverse") else 1, L, pos[0], pos[1])
        self.log("did_not_move", robot=b.name, i=i, took_s=round(took, 3), length_cm=round(L, 1), **{
            k: v for k, v in out.items() if k != "blind_ok"})
        self.say("%s: step %d (%.0f cm) reported done in %.2f s: %s: counted as NOT done; re-planning from where it is "
                 "(never the same step unchanged)" % (b.name, i, L, took, out["words"]))
        if fm.escalate(pos[0], pos[1]):
            ob = {"id": "hazard-%s-%d" % (b.name, len(fm.fails)), "shape": "circle", "center": [round(pos[0], 1),
                  round(pos[1], 1)], "radius": 6.0, "source": "user",
                  "why": "hazard: moves failed here (11-05h)"}
            self.say("%s: %d moves failed at %.0f, %.0f: a hazard, kept out for everyone" % (
                b.name, OB.FailMemory.ESCALATE_N, pos[0], pos[1]))
            self.add_obstacle(ob)
            b.fmem = OB.FailMemory()
        return True

    def floor_feed(self, b, vals):
        """One RAW row for the floor check: the five front zones, when the robot stands still (no step, no speed)."""
        cols = self.link.session.headers.get("RAW") or []
        if not cols:
            return
        row = dict(zip(cols, vals))
        try:
            if abs(float(row.get("v_cms") or 0.0)) > 0.3 or abs(float(row.get("w_dps") or 0.0)) > 3.0:
                return
            if str(row.get("step", "0")).strip() not in ("0", "", "-1"):
                return
            z = tuple(float(row[c]) for c in ("dL_cm", "dCL_cm", "dC_cm", "dCR_cm", "dR_cm"))
        except (ValueError, KeyError, TypeError):
            return
        self.floor_raw[b.name].append(z)

    def site_check(self, b, t, vals):
        """Change check on one RAW row, at the robot's own latest pose (FOLLOW, at most 0.3 s old)."""
        cols = self.link.session.headers.get("RAW") or []
        f = b.follow
        if not cols or not f or f.get("state") == "nopose" or t - f.get("t", 0) > 0.3:
            return
        row = dict(zip(cols, vals))
        try:
            zones = [float(row[c]) if row.get(c) not in (None, "") else None
                     for c in ("dL_cm", "dCL_cm", "dC_cm", "dCR_cm", "dR_cm")]
            sd = float(f.get("sd") or 0.0)
        except ValueError:
            return
        try:
            self.change.feed(b.name, t, (f["x"], f["y"], f["th"]), zones, sd)
        except Exception as e:      # noqa: BLE001  (the check must never stop a plan)
            self.say("site map check error: %s" % e)
            self.change = None

    def on_step(self, b, t, v):
        try:
            sid, state, ms = int(v[1]), v[2], int(float(v[3]))
            x, y, th, front = float(v[4]), float(v[5]), float(v[6]), float(v[7])
        except ValueError:
            return
        info = ",".join(v[8:])
        b.note_clock(t, ms)
        if state in ("started", "done", "failed"):
            b.pose, b.pose_t = (x, y, th), t
        self.log("step_report", robot=b.name, id=sid, state=state, x=x, y=y, th=th, front_cm=front, info=info)
        if state in ("held", "halted"):
            want = "halt" if state == "halted" else ("hold %d" % sid if sid else "hold")
            b.controls = [c for c in b.controls if c["text"] != want]
            return
        if self.backoff_report(b, sid, state, info):
            return
        i = b.id_index.get(sid)
        if i is None:
            # an id from before a re-plan
            if state in ("started", "done") and self.plan_start is not None and not self._expected_old(b, sid):
                self.say("%s %s step %d that the current plan no longer has: re-planning from where it is"
                         % (b.name, state, sid))
                self.request_replan("robot ran an old step")
            return
        st = b.stat[i]
        if state in ("queued", "dup"):
            if st["status"] == "sent":
                st["status"] = "acked"
                st["t_ack"] = t
        elif state == "full":
            st["t_sent"] = t - 10.0 if st["status"] == "sent" else st["t_sent"]   # resend soon
        elif state == "bad":
            self.say("%s could not read step %d (%s): %s" % (b.name, sid, info, st["text"]))
            st["status"] = "failed"
            st["info"] = "bad step"
        elif state == "started":
            if st["status"] in ("sent", "acked"):
                st["status"] = "started"
                st["t_started"] = t
                b.__dict__.setdefault("step_pose0", {})[i] = (x, y, th)
                st["pose_started"] = (x, y, th)
                stp = self.steps(b.name)[i]
                if stp["type"] == "dance" and not stp.get("dance_ms"):
                    # dancing until switched off (Monday's end): its part of the plan is done once it starts
                    st["status"] = "done"
                    st["t_done"] = t
                    st["info"] = "dancing until switched off"
                    b.dancing = True
                    self.say("%s is dancing (until it is switched off)" % b.name)
        elif state == "done" and st["status"] != "done" and self.fast_done(b, i, st, t):
            # (11-05h, protection 3) reported done far too fast with no camera motion: a failure, not a success
            st["status"] = "failed"
            st["info"] = "done without moving"
            self.on_failed(b, i, "done without moving", (x, y, th), front)
        elif state == "done":
            if st["status"] != "done":
                st["status"] = "done"
                st["t_done"] = t
                st["info"] = info
                self.mirror_arrived(b, i, t)
                stp = self.steps(b.name)[i]
                if stp.get("phase") == "confirm" and stp["type"] == "drive":
                    self.confirm_check(b, i, (x, y, th))
                if stp["type"] == "home":
                    self.say("%s is home in dock %s (%s)" % (b.name, stp.get("dock"), info))
                    self.dockings.append(dict(robot=b.name, home=stp.get("dock"), info=info,
                                              t=round(t - self.t_run, 2), own_pose=[x, y, th]))
                if stp["type"] == "approach":
                    self.say("%s docked at %s (%s)" % (b.name, stp.get("board"), info))
                    self.dockings.append(dict(robot=b.name, board=stp.get("board"), info=info, t=round(t - self.t_run, 2),
                                              own_pose=[x, y, th]))
        elif state == "failed":
            if st["status"] not in ("done", "failed"):
                st["status"] = "failed"
                st["info"] = info
                self.labelled(b, i, state, t, (x, y, th), front, info)
                self.on_failed(b, i, info, (x, y, th), front)
        elif state == "dropped":
            if st["status"] in ACTIVE:
                st["status"] = "dropped"
        if state in ("started", "done"):
            self.labelled(b, i, state, t, (x, y, th), front, info)

    def labelled(self, b, i, state, t, pose, front, info):
        """Steps the plan labelled (the warm-up's corner pauses and side feels): when they start and end, with the
        robot's own pose and the camera's latest, for the warm-up's frame list and side answers (warmup.py)."""
        stp = self.steps(b.name)[i]
        if not stp.get("label"):
            return
        self.point_step(b, stp, state, t)
        cam = getattr(b, "last_cam", None)
        self.log("labelled_step", robot=b.name, plan=self.plan_no, i=i, label=stp["label"], phase=stp["phase"],
                 state=state, t_run=round(t - self.t_run, 2) if self.t_run else None, pose=list(pose), front_cm=front,
                 info=info, cam=cam, to=stp["to"], frm=stp["from"])

    CAM_GUARD_CM = 6.0        # chosen: the camera's spot and the robot's own pose further apart than this...
    CAM_GUARD_S = 1.0         # ...for this long stops everyone. Found in the Phase 0 dry runs (2026-10-27): with an 18 %
                              # gyro scale error (Center's turns were 15-20 deg off on 2026-10-27) the robot's pose
                              # drifts more than its fix gate allows, it refuses every later fix and drives off the
                              # table blind. Normal: the two agree within 1-2 cm (fixes 10 a second), plus up to 3 cm
                              # of timing at 10 cm/s. DOE note: 4 / 6 / 8 cm x 0.5 / 1 / 2 s on the board's Phase 0
                              # runs (false stops against the distance driven blind); docs/doe-backlog.md

    LOCK_AGREE_CM = 3.0       # (e12 item 9) a lock fix counts as the camera only within this of a roof-tag read made in
    LOCK_AGREE_S = 2.0        # the last LOCK_AGREE_S. Both chosen (the tag's +- at the far corner is 3-5 cm; a robot at
                              # dwell moves nothing in 2 s). No DOE: a guard

    def lock_trusted(self, b, cam, t):
        """(e12 item 9) Is this camera spot good enough to reset the robot's pose? A roof-tag read: yes. A lock-only fix
        (tag_bridge's lock bank: position only, heading null): only when a tag read in the last LOCK_AGREE_S agrees."""
        if not str(getattr(b, "last_cam_src", "")).endswith(":lock"):
            return True
        lt = getattr(b, "last_tag", None)
        return bool(lt and t - lt[0] <= self.LOCK_AGREE_S and math.hypot(cam[0] - lt[1], cam[1] - lt[2]) <=
                    self.LOCK_AGREE_CM)

    def cam_guard(self):
        """Stop everyone when a robot's own pose and the camera disagree for long (its fixes are being refused)."""
        lim = getattr(self.args, "cam_guard_cm", self.CAM_GUARD_CM)
        if not lim or not self.source.live:
            return
        t = now()
        for b in self.bots.values():
            cam, ct = getattr(b, "last_cam", None), getattr(b, "cam_t", None)
            if b.out or t < b.own_until or getattr(b, "backoff", None):
                # out of the run; or its own pose stands after a barrier find (10 s); or (11-05h) backing off a trip /
                # recovering: its own move is not interrupted, its report re-plans
                b.guard_since = None
                continue
            if cam is None or b.pose is None or ct is None or t - ct > 0.3 or t - b.pose_t > 0.3:
                b.guard_since = None
                continue
            d = math.hypot(cam[0] - b.pose[0], cam[1] - b.pose[1])
            # (11-05h, rehearsal s2: after a restart a robot's own heading was 105 deg off the camera's while its
            # position still agreed; it refused every fix and drove to 9.5 cm of the x = 0 edge before its drop check
            # stopped it) the heading counts too: over HEAD_GUARD_DEG for the same time = the same reset, heading
            # from the camera
            dth = 0.0
            if cam[2] is not None:
                ch = getattr(b, "cam_hist", None) or []
                own_ = self.pose_at(b, ch[-1][0]) if ch else None      # its own heading AT THE PICTURE'S TIME
                oth = own_[3] if own_ is not None else b.pose[2]          # (the camera lags ~0.3 s: turns)
                dth = abs(((cam[2] - oth + 180.0) % 360.0) - 180.0)
            if d <= lim and dth <= self.HEAD_GUARD_DEG:
                b.guard_since = None
                continue
            if getattr(b, "guard_since", None) is None:
                b.guard_since = t
                self.log("cam_guard_start", robot=b.name, d_cm=round(d, 2), dth_deg=round(dth, 1), cam=cam,
                         own=list(b.pose))
            elif t - b.guard_since >= self.CAM_GUARD_S and LOCK_GUARD and not self.lock_trusted(b, cam, t):
                # (e12 item 9, Jay-Sun's 9 Nov tour at P15: a lock fix 7 cm off moved the robot's pose and broke its
                # dwell) a lock-only fix never resets a pose: declined and logged; the robot keeps its own pose
                if not getattr(b, "lock_declined_t", None) or t - b.lock_declined_t > 5.0:
                    b.lock_declined_t = t
                    self.log("cam_guard_lock_declined", robot=b.name, d_cm=round(d, 2), cam=cam, own=list(b.pose),
                             last_tag=getattr(b, "last_tag", None))
                    self.say("%s: the camera's lock (no roof-tag read) disagrees with its own pose by %.1f cm: not "
                             "trusted, its pose kept (it resets only from a roof-tag read)" % (b.name, d))
                b.guard_since = None
                continue
            elif t - b.guard_since >= self.CAM_GUARD_S and self.holds_one() and not \
                    getattr(self, "straightening", False):
                # its own pose has drifted from the camera's: that robot alone is halted and its pose set from the
                # camera (the heading its own gyro has); a third time within a minute it is held and checked again
                hist = [x for x in getattr(b, "guard_resets", []) if t - x < 60.0] + [t]
                b.guard_resets = hist
                self.log("cam_guard_reset", robot=b.name, d_cm=round(d, 2), cam=cam, own=list(b.pose), n=len(hist))
                b.guard_since = None
                if len(hist) >= 3:
                    self.warnings = getattr(self, "warnings", []) + ["%s: camera and own pose disagreed 3 times in a "
                                                                     "minute" % b.name]
                    self.say("WARNING: %s: camera and its own pose disagree again (%.1f cm, 3 times in a minute): "
                             "held and checked again; the others carry on" % (b.name, d))
                    self.retire(b, "camera and its own pose disagreed 3 times in a minute", kind="restarted")
                    continue
                th = cam[2] if cam[2] is not None else b.pose[2]
                if self.keep_going():
                    self.carry_on("cam_guard", b.name, "%s: the camera and its own pose disagreed by %.1f cm for %.1f s: "
                                  "its pose was set to the camera's (the camera is trusted), the run goes on"
                                  % (b.name, d, self.CAM_GUARD_S), d_cm=round(d, 2), cam=cam, own=list(b.pose))
                self.control(b, "halt")
                self.send(b.name, "fpose %.2f %.2f %.2f 1.0 3.0" % (cam[0], cam[1], th))
                b.pose, b.pose_t = (cam[0], cam[1], th), t
                b.acc_t, b.acc_odo = t, getattr(b, "odo", 0.0)     # its pose is the camera's again (confidence)
                self.warnings = getattr(self, "warnings", []) + ["%s: camera and own pose disagreed %.1f cm: pose "
                                                                 "reset from the camera" % (b.name, d)]
                self.say("WARNING: %s: camera and its own pose disagree by %.1f cm: halted, its pose set from the "
                         "camera, and the others re-plan round it" % (b.name, d))
                self.pending_replan = (b.name, now() + 0.5)
                continue
            elif t - b.guard_since >= self.CAM_GUARD_S:
                rej = b.follow.get("fixes_rejected")
                self.abort = ("camera and %s's own pose disagree by %.1f cm for %.1f s (fixes refused so far: %s): "
                              "stopping everyone" % (b.name, d, t - b.guard_since, rej))
                self.log("cam_guard", robot=b.name, d_cm=round(d, 2), cam=cam, own=list(b.pose), refused=rej)
                self.say(self.abort)
                self.halt_all()
                return

    def echo_pose(self, b, t, ms, x, y, th, state):
        """--poses-to HOST:PORT: the robot's own pose (wheels + gyro + the fixes it took), as JSON lines, for the
        camera bridge (tools/fleet/cam_bridge.py): it predicts each robot's next camera spot from these, so who is
        who never rests on looks. Time: the laptop's clock at the robot's own ms (its clock offset)."""
        dest = getattr(self, "pose_dest", None)
        if dest is None:
            return
        tp = (ms / 1000.0 + b.offset) if b.offset is not None else t
        msg = {"type": "pose", "robot": b.name, "t": round(tp, 4), "x": round(x, 2), "y": round(y, 2),
               "th": round(th, 2), "state": state}
        try:
            self.pose_sock.sendto(json.dumps(msg).encode(), dest)
        except OSError:
            pass

    def _expected_old(self, b, sid):
        return sid in getattr(b, "halted_ids", set())

    def on_follow(self, b, t, v):
        try:
            ms = int(float(v[1]))
            cur, state, queued = int(v[2]), v[3], int(v[4])
            last_done, last_failed = int(v[5]), int(v[6])
            x, y, th = float(v[7]), float(v[8]), float(v[9])
        except ValueError:
            return
        b.note_clock(t, ms)
        b.follow = dict(t=t, ms=ms, cur=cur, state=state, queued=queued, last_done=last_done, last_failed=last_failed,
                        x=x, y=y, th=th, sd=v[10], fixes_used=v[12], fixes_rejected=v[13], front=v[14])
        if state != "nopose":
            self.odometer(b, t, x, y, v[12])
            b.pose, b.pose_t = (x, y, th), t
            import observe as OB
            b.__dict__.setdefault("trail", OB.Trail()).add(x, y, th)     # (11-05h, 4'a) its breadcrumb trail
            self.echo_pose(b, t, ms, x, y, th, state)
            ph = b.__dict__.setdefault("pose_hist", [])       # (laptop time of the pose, x, y, th): blind_watch
            ph.append(((ms / 1000.0 + b.offset) if b.offset is not None else t, x, y, th))
            if len(ph) > 120:
                del ph[:-80]
        # done reports lost over WiFi: the robot runs its steps in order, so everything up to last_done is done
        for sid, i in b.id_index.items():
            st = b.stat[i]
            if sid <= last_done and sid != last_failed and st["status"] in ACTIVE:
                st["status"] = "done"
                st["t_done"] = t
                st["info"] = st.get("info") or "(seen in FOLLOW)"
                self.mirror_arrived(b, i, t)
            if sid == last_failed and st["status"] in ACTIVE:
                st["status"] = "failed"
                st["info"] = "failed (STEP record lost)"
                self.on_failed(b, i, st["info"], (x, y, th), float(v[14]) if v[14] else 0.0)
        # its "started" report lost over WiFi: the step it is running now has started (Monday sweep 2026-10-28: the
        # dance's "started" was lost at 15 % packet loss, and a dance until switched off never reports "done", so
        # run_plan waited on it until the time limit and stopped everyone)
        i = b.id_index.get(cur) if cur > 0 else None
        if i is not None and b.stat[i]["status"] in ("sent", "acked"):
            self.on_step(b, t, ["STEP", str(cur), "started", str(ms), str(x), str(y), str(th), v[14] or "0",
                                "(seen in FOLLOW)"])

    # -- outgoing -------------------------------------------------------------
    def pause(self, on):
        """Pause (Mission Control, or SIGUSR1): every robot halts where it is and no step is released; resume
        (SIGUSR2): a re-plan from where they stand, then on as before."""
        if on and not getattr(self, "paused", False):
            self.paused = True
            self.halt_all()
            self.say("paused: every robot halted where it is")
            self.log("pause")
        elif not on and getattr(self, "paused", False):
            self.paused = False
            self.say("resumed: re-planning from where they stand")
            self.log("resume")
            self.request_replan("resume after a pause")

    def release(self):
        if getattr(self, "paused", False):
            return
        t = now()
        for n in self.names:
            b = self.bots[n]
            steps = self.steps(n)
            outstanding = sum(1 for st in b.stat if st["status"] in ACTIVE)
            if b.controls or any(st["status"] == "sent" for st in b.stat):
                continue        # in order: the step before (or a hold/halt) must be acked first
            if getattr(b, "backoff", None):
                continue        # (11-05h) backing away after a tilt / drop trip: nothing else is sent until it reports
            for i, stp in enumerate(steps):
                st = b.stat[i]
                if st["status"] == "done" or st["status"] in ACTIVE:
                    continue
                if st["status"] in ("failed", "dropped"):
                    break                                  # waiting for a re-plan
                if outstanding >= self.args.ahead:
                    break
                if self.planning is not None and i >= self.limit.get(n, 0):
                    break                                  # a re-plan is being computed: carry on only so far
                if any(self.bots[o].stat[j]["status"] != "done" for o, j in stp.get("after", [])):
                    break
                nb = stp.get("not_before_s")
                if nb is not None and t < self.plan_start + nb:
                    break
                if not self.turn_pause_open(b, i, t):
                    break                                  # standing after a turn until the cameras agree
                m_ok, m_v = self.mirror_gate(b, i, t)
                if not m_ok:
                    break                                  # the leader waits for its mirror partner (mirroring)
                sid = b.next_id
                b.next_id += 1
                b.ids[i] = sid
                b.id_index[sid] = i
                wms = None
                if stp["type"] == "wait" and stp.get("until_s") is not None:
                    # a wait for a mission time (a beat, a meet): only what is left of it now, so a robot that
                    # arrives late does not wait the whole planned wait on top
                    wms = max(0, int(round((self.plan_start + stp["t_end"] - t) * 1000)))
                if m_v is not None:
                    stp = dict(stp, speed=m_v)            # re-timed for mirroring: this drive's speed only
                stp = self.with_stop_lead(n, stp)         # (e07) sent short by its stop lead (its last calibration)
                st.update(status="sent", t_sent=t, text=step_command(sid, stp, wms), sends=1,
                          speed_sent=m_v if m_v is not None else stp.get("speed"))
                self.send(n, st["text"])
                self.log("step_sent", robot=n, id=sid, i=i, plan=self.plan_no, text=st["text"])
                break           # the next one goes once this one is acked

    def resend(self):
        t = now()
        for b in self.bots.values():
            if getattr(b, "backoff", None):
                continue        # (11-05h) backing away after a trip: no plan step is resent meanwhile
            for st in b.stat:
                if st["status"] == "sent" and t - st["t_sent"] > self.args.resend_s:
                    st["t_sent"] = t
                    st["sends"] += 1
                    b.resends += 1
                    self.send(b.name, st["text"])
                    self.log("step_resent", robot=b.name, text=st["text"])
            for c in b.controls:
                if t - c["t_last"] > self.args.resend_s:
                    c["t_last"] = t
                    c["sends"] += 1
                    self.send(b.name, c["text"])
                    self.log("control_resent", robot=b.name, text=c["text"])

    def control(self, b, text, ok=None):
        """hold / halt: resent until the robot acks it (STEP held / halted)."""
        b.controls = [c for c in b.controls if c["text"].split()[0] != text.split()[0]]
        t = now()
        b.controls.append(dict(text=text, t=t, t_last=t, sends=1))
        self.send(b.name, text)
        self.log("control", robot=b.name, text=text)

    def fixes(self):
        evs_ = self.source.poll()
        try:                                         # (e13) which sources tracked each robot, each second
            sl = self.__dict__.get("srclog")
            if sl is None:
                import source_log as SL_
                sl = self.srclog = SL_.SourceLog(self.outdir_, list(self.bots))
            for ev in evs_:
                if ev.get("type") == "fix":
                    sl.add(ev.get("robot"), ev.get("src") or "camera")
            sl.tick(time.time())
        except Exception:  # noqa: BLE001  (the log never stops the run)
            pass
        for ev in evs_:
            if ev.get("type") == "camera":
                # (e12) the bridge's frame watch: no new picture from a camera (or pictures again). Said loudly and
                # logged; the robots it no longer sees are held by the lost-robot rules as they are
                self.say(str(ev.get("words") or "CAMERA %s: %s" % (ev.get("cam"), ev.get("state"))))
                self.log("camera_" + str(ev.get("state")), cam=ev.get("cam"), age_s=ev.get("age_s"),
                         gap_s=ev.get("gap_s"))
                # (e13) from here each robot's own risk-based limit (blind_watch: safety_rules.no_picture_limit)
                self.nopic_since = (getattr(self, "nopic_since", None) or now()) if ev.get("state") == "no_pictures" \
                    else None
                # (e14, school 17:2x) a picture gone dark: every robot holds still (the run's own pause) until the
                # camera has adjusted; nothing ends; it carries on by itself
                if ev.get("state") == "dark" and not getattr(self, "dark_paused", False):
                    self.dark_paused = True
                    self.pause_req = True
                elif ev.get("state") == "light_back" and getattr(self, "dark_paused", False):
                    self.dark_paused = False
                    self.pause_req = False
                continue
            if ev.get("type") == "obstacle":
                oid = ev.get("id") or "camera-%d" % len(self.obstacles_seen)
                if oid in self.obstacles_seen:
                    continue
                self.obstacles_seen.add(oid)
                ob = {k: ev[k] for k in ("shape", "points", "center", "radius") if k in ev}
                ob.update(id=oid, source=ev.get("source", "camera"), time=FM.now_iso())
                self.say("camera: new obstacle %s" % oid)
                self.handle_obstacle(ob)
                continue
            b = self.bots.get(ev.get("robot"))
            if b is not None and str(ev.get("src") or "").endswith(":lock") and self.lock_close(b, ev):
                continue            # (e09) a lock-on's name is not trusted while another robot is this close to it
            if b is not None:
                ev = self.observe_fix(b, ev)
                self.fp_fix(b, ev)
                b.fix = ev
                self.cam_seen(b, ev)
                # for the pause after a turn: (frame time, arrived, x, y, which camera); the camera bridge says
                # "cam_bridge:both" / ":overhead" / ":side" / ":lights"; the simulator's direct fixes say nothing
                # (truth plus noise: counted as both)
                src = str(ev.get("src") or "both").split(":")[-1]
                self.note_fix(b, ev, src)
                h = b.__dict__.setdefault("cam_hist", [])
                h.append((float(ev.get("t", now())), now(), float(ev["x"]), float(ev["y"]), src))
                if len(h) > 40:
                    del h[:-30]
        t = now()
        for b in self.bots.values():
            ev = b.fix
            if b.out or ev is None or b.offset is None or t - b.fix_sent_t < 1.0 / self.args.fix_hz:
                continue
            ms = int(round((ev["t"] - b.offset) * 1000))
            th = ev.get("th")
            if getattr(self.args, "straighten", False) and (getattr(b, "fposed_t", None) is None or
                                                            t < b.fposed_t + self.STRAIGHT_TH_DROP_S):
                # the start's heading is the facing check's, sent by fpose: the bridge's first fixes carry the map's
                # start heading (it takes the robot as placed), which may be the wrong way round
                th = None
            if th is None and b.pose is None:
                # a robot with no pose (its follow run has none yet: before the start's fpose) refuses a position-only
                # fix (2026-10-30 09:52, Right: 76 of 77 refused while the start waited for Left). Not sent: the start
                # gives it its pose (fpose) from the camera's position and the facing check's heading
                b.fix = None
                b.fix_held = getattr(b, "fix_held", 0) + 1
                continue
            self.send(b.name, "fix %d %.2f %.2f %s %.2f %.2f" % (ms, ev["x"], ev["y"], "-" if th is None else "%.2f" % th,
                                                                ev.get("sigma", 0.5), ev.get("sigma_th", 1.5)))
            b.fix, b.fix_sent_t = None, t
            b.fix_sent += 1

    def lock_close(self, b, ev):
        """(e09, B1 identity) A lock-on fix within tracking_layer.CLOSE_CM of another robot's latest camera spot: the
        lock may have jumped to the look-alike: not used (logged, at most once a robot every 5 s)."""
        import tracking_layer as TL
        x, y = float(ev["x"]), float(ev["y"])
        for o in self.bots.values():
            if o is b or o.out or getattr(o, "last_cam", None) is None:
                continue
            if math.hypot(o.last_cam[0] - x, o.last_cam[1] - y) < TL.CLOSE_CM:
                if now() - getattr(b, "lock_close_t", 0.0) > 5.0:
                    b.lock_close_t = now()
                    self.log("lock_refused", robot=b.name, near=o.name, x=round(x, 2), y=round(y, 2))
                return True
        return False

    def observe_fix(self, b, ev):
        """(11-05h, Observe then trust) Every camera fix: the tag's heading corrected by what this run has learned
        (b.yaw_fix), then checked against the motion (observe.HeadingWatch on each straight stretch of 8 cm the robot
        drives on its own pose, forwards or backwards). Two stretches in a row the other way round re-learn the
        offset: said, logged, the run carries on."""
        import observe as OB
        yf = getattr(b, "yaw_fix", 0.0)
        if ev.get("th") is not None and yf:
            ev = dict(ev, th=OB.wrap180(float(ev["th"]) + yf))
        if ev.get("th") is None or b.pose is None or not getattr(self, "t_run", None):
            return ev
        w = b.__dict__.setdefault("hwatch", OB.HeadingWatch(0.0))
        own = b.pose
        last = getattr(b, "hw_own", None)
        b.hw_own = own
        sign = 0
        if last is not None:
            dx, dy = own[0] - last[0], own[1] - last[1]
            along = dx * math.cos(math.radians(own[2])) + dy * math.sin(math.radians(own[2]))
            sign = 1 if along > 0.05 else (-1 if along < -0.05 else getattr(b, "hw_sign", 0))
        b.hw_sign = sign
        r = w.feed(now(), (float(ev["x"]), float(ev["y"]), float(ev["th"])), own[2], sign)
        if r and r["kind"] == "heading_bad":
            self.log("heading_bad", robot=b.name, **r)
        elif r and r["kind"] == "heading_relearned":
            d = OB.wrap180(r["new_deg"] - r["old_deg"])
            b.yaw_fix = OB.wrap180(yf + d)
            w.offset = 0.0
            self.__dict__.setdefault("heading_relearned", []).append(dict(robot=b.name, change_deg=d))
            self.log("heading_relearned", robot=b.name, change_deg=d, yaw_fix=b.yaw_fix, **r)
            self.say("%s: HEADING RE-LEARNED: two stretches in a row it moved %+.0f deg from where its roof tag says "
                     "it faces: its tag's headings are corrected by %+.0f deg from now on (observed); its pose is set "
                     "again" % (b.name, d, b.yaw_fix))
            ev = dict(ev, th=OB.wrap180(float(ev["th"]) + d))
            self.send(b.name, "fpose %.2f %.2f %.2f %.2f %.2f" % (float(ev["x"]), float(ev["y"]), float(ev["th"]),
                                                                  1.0, 3.0))
            b.yaw_known = True               # (e15) measured this run: the facing checks use it from now on
            remember_tag_turn(self, b.name, "run_plan's heading watch (re-learned)")
        return ev

    READ_EVERY_S = 0.5        # (11-05h) the readability map: each robot's spot (its own pose) and whether a camera read
                              # its tag there, this often

    def readmap_watch(self):
        """(11-05h) Where the cameras read the roof tags, learned from the run (observe.ReadMap): each robot's own
        spot every READ_EVERY_S, read or not."""
        rm = getattr(self, "readmap", None)
        if rm is None or not getattr(self, "t_run", None) or not self.source.live:
            return
        t = now()
        if t - getattr(self, "_rm_t", 0.0) < self.READ_EVERY_S:
            return
        self._rm_t = t
        for b in self.bots.values():
            if b.out or b.pose is None or getattr(b, "cam_t", None) is None:
                continue
            rm.add(b.pose[0], b.pose[1], t - b.cam_t <= self.blind_after(b))

    TRACK_EVERY_S = 0.2       # cam_track.csv: one camera pose per robot this often (the mapping view's lines)
    TRACK_LOST_S = 1.0        # chosen: no camera pose for this long = the tracker lost the robot (10 fixes missed at
                              # 10 a second); the warm-up shows those moments to Nick. DOE: not needed (it only picks
                              # frames for a person to look at)

    def cam_seen(self, b, ev):
        t = now()
        b.last_cam = [round(float(ev["x"]), 2), round(float(ev["y"]), 2),
                      None if ev.get("th") is None else round(float(ev["th"]), 1)]
        b.last_cam_src = str(ev.get("src") or "")              # (e12 item 9) "tag_bridge:lock" = the lock's guess
        if not b.last_cam_src.endswith(":lock"):
            b.last_tag = (t, float(ev["x"]), float(ev["y"]))    # the newest roof-tag (or other non-lock) read
        if getattr(b, "lost_since", None) is not None:
            self.log("tracker_back", robot=b.name, t_run=round(t - self.t_run, 2) if self.t_run else None,
                     lost_s=round(t - b.lost_since, 2), cam=b.last_cam)
            b.lost_since = None
        b.cam_t = t
        if t - getattr(b, "track_t", 0.0) >= self.TRACK_EVERY_S:
            b.track_t = t
            if getattr(self, "_track_f", None) is None:
                self._track_f = open(os.path.join(self.outdir, "cam_track.csv"), "w", newline="")
                self._track_w = csv.writer(self._track_f)
                self._track_w.writerow(["t", "robot", "x_cm", "y_cm", "heading_deg"])
            self._track_w.writerow(["%.3f" % float(ev.get("t", t)), b.name] + b.last_cam)   # the frame's time
            self._track_f.flush()

    def tracker_watch(self):
        """A robot whose camera poses stop for TRACK_LOST_S while the plan runs: logged once (tracker_lost)."""
        if not self.t_run:
            return
        t = now()
        for b in self.bots.values():
            if getattr(b, "cam_t", None) is None or getattr(b, "lost_since", None) is not None:
                continue
            if t - b.cam_t > self.TRACK_LOST_S:
                b.lost_since = b.cam_t
                self.log("tracker_lost", robot=b.name, t_run=round(b.cam_t - self.t_run, 2), last_cam=b.last_cam,
                         own_pose=list(b.pose) if b.pose else None)

    # -- plans ----------------------------------------------------------------
    def _install(self, plan, keep, first, old):
        """Make `plan` the current one. Steps it carries on from the old plan (the
        first keep[r] steps, old indices first[r]...) keep their robot step ids."""
        if getattr(self, "homes", None):
            P.append_home(plan, self.homes)    # re-plans end at the dock's mouth too: creep in after
        self.plan = plan
        for n in self.names:
            b = self.bots[n]
            steps = plan["robots"][n]["steps"]
            ids = [None] * len(steps)
            stat = [dict(status="unsent", t_sent=0.0, text="", info="", sends=0) for _ in steps]
            if old is not None:
                k = min(keep.get(n, 0), len(steps))
                for j in range(k):
                    oi = first[n] + j
                    if oi < len(old[n][0]) and old[n][0][oi] is not None and \
                            old[n][1][oi]["status"] not in ("failed", "dropped"):
                        # still on the robot (queued, running or done): same id
                        # (a failed or dropped one is no longer there: it goes again, under a new id)
                        ids[j] = old[n][0][oi]
                        stat[j] = old[n][1][oi]
            b.ids, b.stat = ids, stat
            b.id_index = {sid: i for i, sid in enumerate(ids) if sid is not None}

    def states(self, cause=None):
        """Each robot's pose and step now, for the planner (obstacle_decide)."""
        out = {}
        t = now()
        for n in self.names:
            b = self.bots[n]
            steps = self.steps(n)
            i = next((k for k, st in enumerate(b.stat) if st["status"] in ("started", "failed")), None)
            if i is None:
                i = next((k for k, st in enumerate(b.stat) if st["status"] != "done"), len(steps))
                prog = False
            else:
                prog = True
            dwell_left = None
            if prog and i < len(steps) and steps[i]["type"] == "wait":
                st = b.stat[i]
                dur = steps[i]["t_end"] - steps[i]["t_start"] if steps[i]["phase"] in ("dwell", "pause") else 0.0
                dwell_left = max(0.0, dur - (t - st.get("t_started", t)))
            pose = b.pose or tuple(FM.pose_out(FM.pose_in(self.plan["robots"][n]["start"])))
            ended = None
            if prog and i < len(b.stat) and b.stat[i]["status"] == "failed" and \
                    not str(b.stat[i].get("info", "")).startswith("halted"):
                ended = b.stat[i].get("info", "")        # it ended by itself (a warm-up feel's answer), not by a halt
            out[n] = {"pose": [pose[0], pose[1], math.radians(pose[2])], "step": i, "in_progress": prog,
                      "dwell_left_s": dwell_left, "ended": ended}
        return out

    def handle_obstacle(self, ob, cause=None, budget=None, improving=False):
        """Nick's rules (planner.obstacle_decide): stop the imminent robots at once,
        let the others carry on a few steps, then switch to the new plan. The
        decision is quick and acted on here; the new plan is computed in a worker
        thread while the loop keeps running (fixes, acks, the carried-on steps)."""
        if self.planning is not None:
            self.deferred.append((ob, cause, budget, improving))
            return
        if len(self.replans) >= self.args.max_replans and self.keep_going():
            # keep going (2026-11-03): past the cap, re-planning goes on (it is how a blocked point is skipped
            # safely), spaced REPLAN_SPACING_S apart so a loop of re-plans cannot spin; the run's time limit still ends
            # it (keep_going_deadline)
            self.carry_on("out_of_replans", None, "%d re-plans so far (the cap): the run goes on; from now on "
                          "re-plans are at least %.0f s apart" % (len(self.replans), self.REPLAN_SPACING_S))
            wait = getattr(self, "last_replan_t", 0.0) + self.REPLAN_SPACING_S - now()
            if wait > 0:
                self.halt_all()
                self.add_obstacle(ob)
                if self.pending_replan is None or self.pending_replan[1] > now() + wait:
                    self.pending_replan = ("re-plan after the cap, spaced", now() + wait)
                return
        elif len(self.replans) >= self.args.max_replans:
            self.say("out of re-plans (%d); stopping everyone" % len(self.replans))
            self.halt_all()
            self.abort = "out of re-plans"
            return
        self.last_replan_t = now()
        t_event = now()
        states = self.states(cause)
        over = dict(self.plan["settings"])
        gate = getattr(self.args, "min_gap_cm", EC.MIN_GAP_CM) or 0.0
        over["separation_cm"] = max(float(over.get("separation_cm", 0.0)), gate)    # re-plans are made at the gate
        if budget is not None:
            over["replan_budget_s"] = budget
        if improving:
            over["replan_first_safe"] = False     # the background search for a shorter plan: the full search
        news = [ob] if ob else []
        if self.gap_fix_on():
            # robot gap (3 Nov, seed-2 school run): an obstacle put on the map without a decision (keep going's
            # spaced re-plan past the cap, a held robot while every robot was held, a confirmed stop coming back) is
            # checked here too, so no robot carries on a committed step into it (Bravo drove its old step into a
            # held Alpha: bodies 6.6 cm). Off (--gap-fix off): only the new obstacle, as before
            known = getattr(self, "plan_obstacle_keys", None)
            if known is not None:
                seen = {self.obstacle_key(o) for o in news}
                for o in self.m.get("obstacles", []):
                    k = self.obstacle_key(o)
                    if k not in known and k not in seen:
                        news.append(o)
                        seen.add(k)
        try:
            dec = P.obstacle_decide(self.m, self.plan, states, news, over=over,
                                    t_now=t_event - self.plan_start)
        except Exception as e:      # noqa: BLE001  (never leave robots driving on a planner bug)
            if self.keep_going():
                self.replan_trouble("the re-plan failed (%s: %s)" % (type(e).__name__, e), ob, cause)
                return
            self.say("re-plan failed (%s): stopping everyone" % e)
            self.halt_all()
            self.abort = "re-plan failed"
            return
        if not improving:
            for n in self.names:
                self.act_on_decision(n, dec["keep"][n], dec["first_step"][n], states[n]["in_progress"])
        self.limit = {n: dec["first_step"][n] + dec["keep"][n] for n in self.names}
        job = dict(dec=dec, ob=ob, cause=cause, improving=improving, t_event=t_event, over=over, states=states,
                   result=None, error=None, cpu=None,
                   map_keys={self.obstacle_key(o) for o in dec["m2"].get("obstacles", [])})

        def work():
            t0 = time.perf_counter()
            try:
                job["result"] = P.obstacle_replan(dec)
            except Exception as e:  # noqa: BLE001
                job["error"] = e
            job["cpu"] = time.perf_counter() - t0
        job["thread"] = threading.Thread(target=work, daemon=True)
        self.planning = job
        job["thread"].start()

    def finish_replan(self):
        job = self.planning
        if job is None or job["thread"].is_alive():
            return
        self.planning = None
        dec, ob, cause, improving, t_event, over = (job[k] for k in ("dec", "ob", "cause", "improving", "t_event", "over"))
        if job["error"] is not None:
            if isinstance(job["error"], P.PlanError) and improving:
                self.say("improvement try found no plan (%s)" % job["error"])
                return self.next_deferred()
            if self.no_plan_drop(ob, cause, job["error"]):
                return
            if self.keep_going():
                self.replan_trouble("no new plan (%s)" % job["error"], ob, cause)
                return
            self.say("no new plan (%s): stopping everyone" % job["error"])
            self.halt_all()
            self.abort = "no plan"
            return
        newplan, events = job["result"]
        cpu = job["cpu"]
        outs = [n for n in newplan["robots"] if self.bots[n].out]
        if outs:                     # a robot went out while this was being computed: it is not in the new plan
            newplan = self.without(newplan, outs)
            events = [e for e in events if e["robot"] not in outs]
        if newplan.get("stranded") and not improving and self.no_plan_drop(
                ob, cause, "no route for %s" % ", ".join(newplan["stranded"])):
            return
        rj = getattr(self, "rejoin_pending", None)
        if rj and not improving and now() > rj[1]:
            self.rejoin_pending = rj = None
        if rj and not improving and newplan.get("stranded"):
            # within REJOIN_TRIAL_S of a rejoin, robots left without a route undo it (dry run 70 x 130: the rejoin's
            # own plan was fine, the next re-plan, after a false stop, left both robots without a route for good)
            rj = rj[0]
            self.rejoin_pending = None
            # the program cannot take it back from here (a dry run, 29 Oct: Left went silent in the circle while
            # Right carried on mowing alone; together, neither had a route): it stays where it is as a fixed
            # obstacle and the others carry on, as before rejoining existed
            b = self.bots[rj]
            why = "no plan fits it back in (%s left without a route)" % ", ".join(newplan["stranded"])
            self.log("rejoin_undone", robot=rj, stranded=newplan["stranded"])
            self.say("%s could not be fitted back into the plan (%s): it is held where it is, as a fixed obstacle; "
                     "the others carry on, and it is tried again later (the others will have moved on)" % (rj, why))
            self.retire(b, "heard again, but %s" % why, kind="restarted")
            self.retry_later(b, why)
            return
        gate = getattr(self.args, "min_gap_cm", EC.MIN_GAP_CM) or 0.0
        if gate:
            gap = EC.plan_gap(newplan, self.m, gate_cm=gate)
            bad = EC.gate_words(gap, gate)
            if bad:
                self.log("plan_refused", why=bad, gap=gap, improving=improving)
                if improving:
                    self.say("improvement try refused: %s" % bad)
                    return self.next_deferred()
                self.say("new plan refused: %s; everyone holds, planning again in %.0f s" % (bad, self.args.retry_s))
                self.n_refused = getattr(self, "n_refused", 0) + 1
                self.halt_all()
                self.replans.append({"t_run": round(t_event - self.t_run, 2), "source": "refused", "why": bad,
                                     "cpu_s": round(cpu, 3)})
                self.pending_replan = ("plan refused under the safety net", now() + self.args.retry_s)
                return
        if cause:
            for e in events:
                if e["robot"] == cause:
                    e.update(action="stopped", reason="its distance sensor saw something ahead"
                             if ob and ob.get("source") == "distance_sensor" else
                             "its edge guard stopped it at an open plywood edge (limited, holding)"
                             if ob and ob.get("source") == "edge_guard" else "its step failed")
        # did a robot get past the steps it was allowed to carry on with? then this plan is stale
        stale = [n for n in self.names
                 if any((st["status"] == "started" and st.get("t_started", 0) > t_event)
                        or (st["status"] == "done" and st.get("t_done", 0) > t_event)
                        for st in self.bots[n].stat[self.limit.get(n, 0):])]
        rec = {"t_run": round(t_event - self.t_run, 2), "source": ob["source"] if ob else ("improve" if improving else "retry"),
               "cpu_s": round(cpu, 3), "budget_s": over["replan_budget_s"], "events": events,
               "new_makespan_s": newplan["summary"]["makespan_s"], "stranded": newplan.get("stranded", []),
               "obstacle": ob, "states": job["states"], "old_plan": self.plan_no}
        if improving:
            old_end = self.plan_start + self.plan["summary"]["makespan_s"]
            new_end = t_event + newplan["summary"]["makespan_s"]
            # (a plan that leaves someone without a route only looks quicker)
            rec["kept"] = new_end < old_end - 1.0 and not stale and not newplan.get("stranded")
            self.replans.append(rec)
            self.log("replan", **rec)
            self.say("improvement try: done at %.0f s instead of %.0f s, %s" % (
                new_end - self.t_run, old_end - self.t_run, "kept" if rec["kept"] else "not kept"))
            if not rec["kept"]:
                return self.next_deferred()
            for n in self.names:      # the improved plan takes over: hold anything sent beyond the carried-on steps
                self.act_on_decision(n, dec["keep"][n], dec["first_step"][n], False)
        else:
            if stale:
                rec["stale"] = stale
                self.replans.append(rec)
                self.log("replan", **rec)
                self.say("%s went past the steps it could carry on with while planning; planning again" % ", ".join(stale))
                self.handle_obstacle(ob, cause=cause, budget=over["replan_budget_s"])
                return
            self.replans.append(rec)
            self.log("replan", **rec)
        for e in events:
            self.say("%s %s: %s" % (e["robot"], e["action"], e["reason"]))
        # every board a robot still had to visit must still be in its plan (or be reported skipped)
        for n in self.names:
            left = [st.get("board") for i, st in enumerate(self.steps(n))
                    if st["type"] == "approach" and self.bots[n].stat[i]["status"] != "done"]
            new = [st.get("board") for st in newplan["robots"][n]["steps"] if st["type"] == "approach"]
            pend = newplan["robots"][n].get("pending") or {}
            new += [v for v in pend.get("visits", []) if isinstance(v, str)] + [pend.get("lead_board")]
            lost = [x for x in left if x not in new]
            if lost and not any(("skips" in w and n in w) for w in newplan.get("warnings", [])):
                self.say("WARNING: the new plan dropped %s's visit to %s without saying why" % (n, ", ".join(lost)))
                self.log("lost_task", robot=n, boards=lost, plan=self.plan_no + 1)
        self.say("new plan in %.2f s (%s); everyone done about %.0f s after the event"
                 % (cpu, "first safe plan, robots told %.1f s ahead; a shorter one is searched for next"
                    % P.carry_on_s(P.settings(over)) if newplan["summary"].get("first_safe_only")
                    else "budget %.1f s" % over["replan_budget_s"], newplan["summary"]["makespan_s"]))
        old = {n: (list(self.bots[n].ids), [dict(x) for x in self.bots[n].stat]) for n in self.names}
        if ob:
            self.m["obstacles"].append(FM._norm_obstacle(ob, len(self.m["obstacles"])))
            self.m = FM.normalize(self.m)
        self._install(newplan, dec["keep"], dec["first_step"], old)
        self.plan_obstacle_keys = job["map_keys"]      # what the plan in force was made around
        self.plan_no += 1
        self.plan_start = t_event
        self.save_plan()
        self.replan_fails = 0
        self.trouble_cause = None
        self.note_skips(newplan)
        self.keep_reassigned()
        self.next_retry = now() + self.args.retry_s if newplan.get("stranded") else math.inf
        if newplan["summary"].get("first_safe_only") and not newplan.get("stranded"):
            # the first safe plan went out at once: the search for a shorter one starts now, in the background
            # (Nick, 2026-10-27 00:08); it is swapped in only if everyone finishes sooner (the improvement try)
            self.improve_at = now()
        elif ob is not None and self.args.improve_after_s > 0:
            self.improve_at = t_event + over["replan_budget_s"] + self.args.improve_after_s
        self.next_deferred()

    def next_deferred(self):
        if self.deferred and self.planning is None and not self.abort:
            ob, cause, budget, improving = self.deferred.pop(0)
            self.handle_obstacle(ob, cause=cause, budget=budget, improving=improving)

    def act_on_decision(self, n, keep, first, in_progress):
        b = self.bots[n]
        b.halted_ids = getattr(b, "halted_ids", set())
        sent = [(i, sid) for i, sid in enumerate(b.ids) if sid is not None and b.stat[i]["status"] in ACTIVE]
        if keep == 0:
            if any(b.stat[i]["status"] == "started" for i, _ in sent) or in_progress:
                b.halted_ids |= {sid for _, sid in sent}
                self.control(b, "halt")
            elif sent:
                b.halted_ids |= {sid for _, sid in sent}
                self.control(b, "hold")
            return
        last = first + keep - 1
        beyond = [(i, sid) for i, sid in sent if i > last]
        if beyond:
            b.halted_ids |= {sid for _, sid in beyond}
            keep_id = b.ids[last] if last < len(b.ids) and b.ids[last] is not None else 0
            if keep_id:
                self.control(b, "hold %d" % keep_id)
            else:
                self.control(b, "hold")

    def halt_all(self):
        for b in self.bots.values():
            self.control(b, "halt")

    # -- a robot out of the run (silent, restarted): a fixed obstacle; the others re-plan and carry on ----------
    HELD_PM_MAX_CM = 10.0  # CHOSEN (Claude, 3 Nov): the most odometry drift added round a held robot (40 cm blind = 2.8
                           # cm; with no camera at all the +- grows to ~50 cm, which would wall the table off). DOE note:
                           # held robots' true offset against this +- on the board (blind 40 cm, 5 runs) and the
                           # skipped points it costs at 5 / 10 / 20 cm

    def out_obstacle(self, b):
        """Where a robot that went out may be: its body at its last pose, plus the area of every step it had been
        sent and not reported done (it may still drive what it buffered), 1 cm bigger all round."""
        import numpy as np
        import geometry as G
        body = FM.body_of(self.m)
        pose = b.pose or (tuple(b.last_cam[:2]) + (b.last_cam[2] or 0.0,) if getattr(b, "last_cam", None) else None)
        if pose is None:
            pose = tuple(FM.pose_out(FM.pose_in(self.plan["robots"][b.name]["start"]))) \
                if b.name in self.plan["robots"] else (0.0, 0.0, 0.0)
        pts = [body.corners([[pose[0], pose[1], math.radians(pose[2])]])[0]]
        steps = self.plan["robots"].get(b.name, {}).get("steps", [])
        for i, st in enumerate(b.stat):
            if st["status"] not in ACTIVE or i >= len(steps):
                continue
            stp = steps[i]
            fr, to = FM.pose_in(stp["from"]), FM.pose_in(stp["to"])
            if stp["type"] in ("drive", "approach"):
                pts.append(body.drive_sweep(fr[:2], to[:2], fr[2]))
            elif stp["type"] == "arc":
                pts += list(body.corners(P.step_poses(stp, 24)))
            elif stp["type"] in ("turn", "park", "dance"):
                a = np.linspace(0, 2 * math.pi, 16, endpoint=False)
                pts.append(np.c_[fr[0] + body.r_turn * np.cos(a), fr[1] + body.r_turn * np.sin(a)])
        hull = G.convex_hull(np.vstack(pts))
        c = hull.mean(0)
        # Nick (2026-10-29 20:06, open question 44): a moving robot keeps the same 8 cm from a held or parked robot as
        # from a live one. The planner keeps safety_margin_cm from an obstacle, so the outline grows by the rest of the
        # net (8 - 1.5 = 6.5 cm). Before: 1 cm, and Right passed a held Left at 3.4-5.9 cm (false-stops report)
        sep = max(float(getattr(self.args, "min_gap_cm", EC.MIN_GAP_CM) or EC.MIN_GAP_CM), EC.MIN_GAP_CM)
        grow = max(1.0, sep - float((self.plan.get("settings") or {}).get("safety_margin_cm", 1.5)))
        # robot gap (3 Nov, Claude's choice): a held robot whose position is its own odometry (no camera sees it) may
        # be off by the drift since its last camera fix; that drift is added to the net round it, at most
        # --held-pm-max-cm (0 = off, the old outline). The camera's own error is already inside the 8 cm
        extra_pm = 0.0
        cap = float(getattr(self.args, "held_pm_max_cm", self.HELD_PM_MAX_CM) or 0.0)
        if cap > 0 and b.pose is not None:
            try:
                c = self.conf_now(b)
            except Exception:       # noqa: BLE001  (an outline is always made)
                c = None
            if c and c.get("source") == "odometry only":
                extra_pm = min(cap, self.ODO_P95_PER_CM * float(c.get("since_fix_cm") or 0.0))
        grow += extra_pm
        if NEVER_STILL and getattr(b, "out_kind", None) in ("restarted", "silent"):
            grow += CREEP_REACH_CM     # (e18) it creeps within this of where it was held: the others plan round it all
        # a true offset (each corner rounded by a disc of that radius, 16 points), not a push out from the middle,
        # which along the sides gave only 70 % of it
        a_ = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        disc = np.c_[np.cos(a_), np.sin(a_)] * (grow / math.cos(math.pi / 16))
        hull = G.convex_hull((hull[:, None, :] + disc[None, :, :]).reshape(-1, 2))
        return {"id": "out-%s" % b.name, "shape": "polygon", "points": hull.round(2).tolist(), "source": "robot_out",
                "time": FM.now_iso(), "gap_cm": round(grow, 2), "odometry_pm_cm": round(extra_pm, 2)}

    @staticmethod
    def without(plan, names):
        """A copy of a plan with these robots taken out (their steps, and every "after" that waits for them)."""
        if not names:
            return plan
        import copy
        out = copy.deepcopy(plan)
        for n in names:
            out["robots"].pop(n, None)
        for r in out["robots"].values():
            for st in r["steps"]:
                st["after"] = [a for a in st.get("after", []) if a[0] not in names]
        out["stranded"] = [n for n in out.get("stranded", []) if n not in names]
        return out

    def retire(self, b, why, kind="out", halt=True, reassign=True):
        """Nick (2026-10-27 16:43): a robot that dies mid-run is a brick; the others carry on. kind "silent" or
        "restarted" (Nick, 2026-10-29 16:2x): it is held, not out for good: when it is heard again it rejoins
        (rejoin_watch). What it needs for that is kept: its part of the plan and its steps' states.
        (e07) kind "recovering": a lost robot on its straight recovery, taken out of the plan while it drives so the
        others re-plan round its reserved corridor (halt False: its recovery steps are already sent; reassign False:
        its points stay its own); backoff_report brings it back."""
        if b.out:
            return
        import copy
        b.out = why
        b.out_kind = kind
        b.blind = None             # (11-05h) its blind distance starts again when it rejoins (seen at the re-check)
        b.out_entry = copy.deepcopy(self.plan["robots"].get(b.name)) if b.name in self.plan["robots"] else None
        b.out_ids, b.out_stat = list(b.ids), [dict(x) for x in b.stat]
        b.back_t = None
        b.out_t = now()
        ob = self.out_obstacle(b)
        if kind == "recovering":
            self.say("%s: its recovery path is RESERVED: the others re-plan round it (they do not just check their next "
                     "steps) and carry on" % b.name)
        elif self.rejoinable(b):
            self.say("%s is held: %s. It is a fixed obstacle where it stopped until it is heard again (then it rejoins); "
                     "the others re-plan and carry on" % (b.name, why))
        else:
            self.say("%s is out: %s. It is now a fixed obstacle where it stopped; the others re-plan and carry on"
                     % (b.name, why))
        self.log("robot_out", robot=b.name, why=why, obstacle=ob, pose=list(b.pose) if b.pose else None, kind=kind)
        if halt:
            self.control(b, "halt")          # if it can still hear: stop at once
        self.names = [n for n in self.names if n != b.name]
        self.outs = getattr(self, "outs", []) + [dict(robot=b.name, why=why, t_run=round(now() - self.t_run, 2)
                                                      if self.t_run_started() else None)]
        if not self.names:
            if self.rejoinable(b) and self.all_held_capped():
                self.abort = ("every robot held %d times (first %.0f s ago): the run ends here, the points not done "
                              "are listed (keep-going cap: --all-held-max %d, --all-held-max-s %.0f)" % (
                                  self.all_held_n, now() - self.all_held_t0, self.all_held_max(),
                                  self.all_held_max_s()))
                self.log("all_held_cap", n=self.all_held_n, since_s=round(now() - self.all_held_t0, 1))
                self.say(self.abort)
                return
            if self.rejoinable(b):
                self.say("every robot is held: waiting up to %.0f s for %s to be heard again" % (self.REJOIN_WAIT_S,
                                                                                               b.name))
                self.plan = self.without(self.plan, [b.name])
                if self.gap_fix_on():
                    # robot gap (3 Nov): it is on the map while everyone is held, so the robot that rejoins first
                    # plans round it (before: no outline, and Bravo's rejoin plan drove straight through Alpha)
                    self.add_obstacle(ob)
                return
            self.abort = "every robot is out (%s)" % why
            self.say(self.abort)
            return
        self.plan = self.without(self.plan, [b.name])
        if reassign and self.keep_going() and self.t_run_started():
            self.reassign_points(b, why)
        self.handle_obstacle(ob)

    # -- unconfirmed sensor obstacles (Nick, 2026-10-29 16:2x; front_stop.py) -------------------------------------
    # The robot always stops. What its sensor alone saw is on the map as "unconfirmed": it expires after
    # --unconfirmed-s (15 s) unless the camera or a second stop at the same place confirms it, and is dropped (the plan
    # made again) rather than the run ending with "no plan". The 5 "no plan" ends of 29 Oct each followed a false stop.
    def unconf(self):
        u = self.__dict__.get("_unconf")
        if u is None:
            u = self._unconf = FSTOP.Unconfirmed(now, expire_s=getattr(self.args, "unconfirmed_s", FSTOP.UNCONFIRMED_S))
        return u

    MOVING_CM = 2.0     # chosen: a robot "was moving" at a sensor stop when the step it stopped in had taken it this far
                        # (a drive or an arc). The false stops of 29 Oct came standing after a turn, at the drive's
                        # start (0-1 cm). DOE note: worth one in the logs: 1 / 2 / 4 cm against the five releases'
                        # sensor stops (how many false stops each would count as moving)

    def was_moving(self, b, i, pose):
        """Nick (2026-10-29 20:06): a second stop at the same place confirms an obstacle only if the robot was moving at
        one of the two stops. Moving: step i is a drive or arc and took it MOVING_CM or more before the stop."""
        stp = self.steps(b.name)[i] if i < len(self.steps(b.name)) else {}
        if stp.get("type") not in ("drive", "arc", "approach"):
            return False
        p0 = b.stat[i].get("pose_started") if i < len(b.stat) else None
        if p0 is None:
            p0 = stp.get("from")
        return p0 is not None and math.hypot(pose[0] - p0[0], pose[1] - p0[1]) >= self.MOVING_CM

    # -- (e08, A2) sure / unsure near an edge: the rule free roam uses (safety_rules.edge_judgement) ---------------
    def edge_watch(self):
        """Twice a second: for every robot with a fresh camera fix, the derived edge clearance's margin at the camera's
        spot (its uncertainty: the camera's sigma + its own pose's distance from the camera), the sure / unsure
        judgement logged on every change (edge_judgement); unsure with a person there (--attended): a question on
        Mission Control's Start screen (safety_rules.Questions), else logged only. The plan's own edge room (the same
        derived rule) and the robots' edge guards stay the protection while it is asked."""
        import safety_rules as SR
        t = now()
        if t - getattr(self, "_edge_t", 0.0) < 0.5 or not self.t_run_started():
            return
        self._edge_t = t
        f = self.m["floor"]
        rect = (f["x_min"], f["y_min"], f["x_max"], f["y_max"])
        trk = getattr(self.args, "edge_tracking", "lights") or "lights"
        for b in self.bots.values():
            if b.out or b.pose is None or getattr(b, "last_cam", None) is None:
                continue
            fresh = t - (getattr(b, "cam_t", None) or -1e9) <= self.blind_after(b, SR.SURE_CAM_AGE_S)
            cam = b.last_cam
            dis = math.hypot(b.pose[0] - cam[0], b.pose[1] - cam[1]) if fresh else None
            if self.fp_live(b):
                # (e09, B2) the filter's position and uncertainty (staleness and blind driving inside it), the own pose
                # vs camera disagreement still added at once (Phase A's A1)
                q = b.fp.pose(t)
                need = SR.clearance_cm(b.fp.unc_cm(t) + (dis or 0.0), None, trk, stops_here=False)
                margin = SR.edge_dist(rect, q[0], q[1]) - need
            else:
                need = SR.clearance_cm(SR.CAM_SIGMA_CM + (dis or 0.0), None, trk, stops_here=False)
                margin = SR.edge_dist(rect, cam[0], cam[1]) - need
            j = SR.edge_judgement(margin, fresh, dis)
            if j["state"] == getattr(b, "edge_state", None):
                continue
            b.edge_state = j["state"]
            self.log("edge_judgement", robot=b.name, state=j["state"], why=j["why"], margin_cm=round(margin, 2),
                     need_cm=round(need, 2), disagree_cm=None if dis is None else round(dis, 2))
            if j["state"] == "unsure":
                self.n_unsure = getattr(self, "n_unsure", 0) + 1
                self.say("%s: unsure near an edge (%s)" % (b.name, j["why"]))
                if not getattr(self.args, "attended", False):
                    # (e09, B4: Nick 7 Nov) nobody there: a camera picture of this moment for him to look at later
                    # (mission_run's frame saver takes it: doe_loop.Tail, event picture_wanted)
                    self.log("picture_wanted", robot=b.name, why="unsure near an edge (%s)" % j["why"])
                if getattr(self.args, "attended", False):
                    if getattr(self, "_asks", None) is None:
                        self._asks = SR.Questions(run=os.path.basename(self.outdir.rstrip("/")))
                    b.edge_q = self._asks.ask(b.name, "%s may be near the table's edge (%s). Is it near the edge?" % (
                        b.name, j["why"]), ("It is near the edge", "It is clear of the edge"))
            elif getattr(b, "edge_q", None) and getattr(self, "_asks", None) is not None:
                self._asks.close(b.edge_q, "settled by the evidence: %s" % j["why"])
                b.edge_q = None

    def mark_unconfirmed(self, b, ob):
        if ob is None or not getattr(self.args, "unconfirmed_s", FSTOP.UNCONFIRMED_S):
            return
        u = self.unconf()
        by = (getattr(self, "sensor_stops", None) or [{}])[-1].get("camera")
        near = u.near(ob["points"], moving=ob.get("moving"))
        if not by and not near and u.near(ob["points"]):
            self.say("%s: stopped again where an earlier stop was, but it was standing both times (a reading after a "
                     "turn, not a drive into something): not confirmed" % b.name)
            self.log("obstacle_not_confirmed", id=ob["id"], robot=b.name, why="standing at both stops",
                     also=u.near(ob["points"]))
        if by or near:
            on_map = {o.get("id") for o in self.m.get("obstacles", [])}
            back = []
            for k in near:
                u.confirm(k, by or "a second stop by %s" % b.name)
                rec = next((h for h in u.history if h["id"] == k), None)
                if k not in on_map and rec is not None:
                    # the earlier stop's outline had expired or been dropped: it comes back, confirmed (simulator,
                    # 29 Oct: without it the dance spot was planned on the brick corner the first stop had seen)
                    self.m["obstacles"].append(FM._norm_obstacle({"id": k, "shape": "polygon", "points": rec["points"],
                                                                  "source": "distance_sensor"},
                                                                 len(self.m["obstacles"])))
                    back.append(k)
            if back:
                self.m = FM.normalize(self.m)
            ob["confirmed"] = by or "a second stop"
            if near:
                self.say("%s: stopped again where an earlier stop was: the obstacle is confirmed and stays on the map"
                         % b.name)
            self.log("obstacle_confirmed", id=ob["id"], robot=b.name, by=by or "second stop", also=near)
            return
        u.add(ob, b.name, moving=ob.get("moving"))
        ob["unconfirmed"] = True
        self.say("%s: no camera sees it: mapped as unconfirmed (it leaves the map in %.0f s unless the camera or a "
                 "second stop confirms it)" % (b.name, u.expire_s))
        self.log("obstacle_unconfirmed", id=ob["id"], robot=b.name, expires_s=u.expire_s, points=ob["points"])

    def drop_obstacles(self, ids, how):
        u = self.unconf()
        before = len(self.m.get("obstacles", []))
        self.m["obstacles"] = [o for o in self.m.get("obstacles", []) if o.get("id") not in ids]
        if len(self.m["obstacles"]) != before:
            self.m = FM.normalize(self.m)
        for k in ids:
            u.drop(k, how)
        self.log("unconfirmed_" + how.split()[0], ids=list(ids))

    def unconfirmed_watch(self):
        u = self.__dict__.get("_unconf")
        if u is None or not u.items:
            return
        for o in self.m.get("obstacles", []):
            if o.get("source") not in ("camera", "brickwatch"):
                continue
            pts = o.get("points") or ([o["center"]] if o.get("center") else [])
            for k in u.near(pts):
                if k in u.items:
                    u.confirm(k, "the camera (%s)" % o.get("id"))
                    self.say("obstacle %s: the camera sees it too: confirmed, it stays on the map" % k)
                    self.log("obstacle_confirmed", id=k, by="camera %s" % o.get("id"))
        gone = u.expired()
        if not gone:
            return
        self.drop_obstacles(gone, "expired")
        self.say("%s: unconfirmed for %.0f s (no camera, no second stop): taken off the map"
                 % (", ".join(gone), u.expire_s))
        if self.plan.get("stranded"):
            self.next_retry = now()
        elif self.planning is None and self.improve_at is None and not self.abort:
            self.improve_at = now()     # the room it freed may give a shorter plan (kept only if everyone ends sooner)

    def no_plan_drop(self, ob, cause, err):
        """A re-plan found no plan: with unconfirmed obstacles on the map (or the new one unconfirmed), drop them and
        plan again instead of stopping everyone. True when it did."""
        if not getattr(self.args, "unconfirmed_s", FSTOP.UNCONFIRMED_S):
            return False
        u = self.unconf()
        on_map = {o.get("id") for o in self.m.get("obstacles", [])}
        ids = [k for k in u.items if k in on_map]
        new_unc = bool(ob and ob.get("unconfirmed") and ob.get("id") in u.items)
        if new_unc:
            ids.append(ob["id"])
        if not ids:
            return False
        self.drop_obstacles(ids, "dropped (no plan)")
        self.no_plan_drops = getattr(self, "no_plan_drops", 0) + 1
        self.say("no plan with %s on the map (%s): %s, planning again" % (
            "an unconfirmed obstacle" if len(ids) == 1 else "%d unconfirmed obstacles" % len(ids), err,
            "dropped it" if len(ids) == 1 else "dropped them"))
        self.handle_obstacle(None if new_unc else ob, cause=cause)
        return True

    def live_obstacles(self):
        u = self.__dict__.get("_unconf")
        t = now()
        out = []
        for o in self.m.get("obstacles", []):
            d = {k: o[k] for k in ("id", "points", "center", "radius", "source") if k in o}
            if u is not None and o.get("id") in u.items:
                d["unconfirmed"] = True
                d["expires_in_s"] = round(max(0.0, u.expire_s - (t - u.items[o["id"]]["t"])), 1)
            out.append(d)
        return out

    # -- a silent or restarted robot rejoins (Nick, 2026-10-29 16:2x; rules audit rec 7) ----------------------------
    REJOIN_HEARD_S = 1.0   # chosen: heard steadily this long before it rejoins (not a flicker of the WiFi). No DOE: the
                           # simulator's silent-then-back runs (5 s, 2-6 s restarts) check it works; a flapping link
                           # is capped by REJOIN_MAX
    REJOIN_CAM_S = 0.6     # its camera position no older than this (fixes come ~10 a second). No DOE: plumbing
    REJOIN_MAX = 10        # chosen: rejoins per robot per run; after the 3rd its next silence keeps it out (a link that
                           # keeps dropping). No DOE: a stop to a loop
    REJOIN_BOOT_WAIT_S = 4.0  # its boot ID (status, CLOCK, RUNINFO) not heard in this long: it is checked as restarted
    REJOIN_TRIAL_S = 30.0  # chosen: for this long after a rejoin, a re-plan that leaves robots without a route undoes it
                           # (the returning robot becomes a fixed obstacle again). No DOE: a guard against a deadlock
    REJOIN_STALL_S = 20.0  # chosen: after a rejoin, no step done by anyone this long undoes it (a stuck joint plan).
                           # Monday's steps finish every 1-3 s. No DOE: a guard against a deadlock
    REJOIN_WAIT_S = 90.0   # chosen: when everyone else is done (or every robot is held), the run waits this long for a
                           # held robot before it ends. No DOE: a policy (the run's length against a robot left behind)

    def rejoin_on(self):
        return not getattr(self.args, "no_rejoin", False)

    # The both-held loop (3 Nov, robot-gap report: keep going ground on ~10 min with both robots held and re-held).
    ALL_HELD_MAX = 3        # chosen: the 3rd time every robot is held at once ends the run (points not done listed).
    ALL_HELD_MAX_S = 180.0  # chosen: or every robot held again more than this long after the first time. DOE note:
                            # 2 / 3 / 5 times and 120 / 180 / 300 s against points done and minutes lost in the seed-2
                            # stress case (simulator) and at school; 0 = off (the old way)

    def all_held_max(self):
        v = getattr(self.args, "all_held_max", None)
        if v is None and HELD_NEVER_END:
            return 0                       # (e16, 6a) never give up: no cap unless asked for
        return self.ALL_HELD_MAX if v is None else int(v)

    def all_held_max_s(self):
        v = getattr(self.args, "all_held_max_s", None)
        if v is None and HELD_NEVER_END:
            return 0.0
        return self.ALL_HELD_MAX_S if v is None else float(v)

    def all_held_capped(self):
        """Count this time every robot is held; True when the cap is reached (either limit; 0 switches one off)."""
        self.all_held_n = getattr(self, "all_held_n", 0) + 1
        if getattr(self, "all_held_t0", None) is None:
            self.all_held_t0 = now()
        n_max, s_max = self.all_held_max(), self.all_held_max_s()
        return bool((n_max > 0 and self.all_held_n >= n_max) or
                    (s_max > 0 and self.all_held_n > 1 and now() - self.all_held_t0 > s_max))

    RETRY_BASE_S = 10.0    # a held robot whose re-check failed is checked again after 10, 20, 40, 60, 60 ... s (Nick
                           # 30 Oct 10:08: no robot sits out for good). Chosen; no DOE (a patience against the others'
                           # holds: every re-check halts them)
    RETRY_MAX_S = 60.0

    def rejoinable(self, b):
        return (self.rejoin_on() and getattr(b, "out_kind", None) in ("silent", "restarted") and
                getattr(b, "rejoins_n", 0) < self.REJOIN_MAX and not getattr(b, "rejoin_failed", None) and
                getattr(b, "out_entry", None) is not None)

    def retry_later(self, b, why):
        """A held robot's re-check or rejoin did not work: not out for good, checked again after a wait that grows."""
        k = getattr(b, "retry_n", 0)
        b.retry_n = k + 1
        wait = min(self.RETRY_MAX_S, self.RETRY_BASE_S * 2 ** k)
        b.retry_at = now() + wait
        b.rejoin_failed = None
        b.out_kind = "restarted"
        b.back_t = None
        b.boot_back = None
        # (out_t is NOT renewed: when everyone else is done, the run waits REJOIN_WAIT_S from when it was set aside,
        # so a robot that never passes cannot keep the run going for ever: a warning and a failure, not a hang)
        self.warnings = getattr(self, "warnings", []) + ["%s: %s; checked again in %.0f s" % (b.name, why, wait)]
        self.log("warning", text="%s: %s; checked again in %.0f s" % (b.name, why, wait))
        self.say("WARNING: %s: %s; it stays held and is checked again in %.0f s" % (b.name, why, wait))

    def out_heard(self, b, t, kind, rtype, vals):
        """A record from a robot that is out of the run: its boot ID (did it restart?), and since when it is heard."""
        boot = None
        if kind == "M" and vals.lower().startswith("status "):
            st_ = EC.parse_status(vals)
            if st_:
                b.firmware, b.battery, boot = st_["firmware"], st_["battery"], st_["boot"]
        elif rtype == "CLOCK" and len(vals) >= 3:
            boot = vals[2].strip().upper()
        elif rtype == "RUNINFO" and len(vals) >= 2:
            boot = EC.boot_of_run(vals[0])
            b.run_id, b.run_exp = vals[0], vals[1]
        elif rtype == "RUNEND" and len(vals) >= 2 and vals[0] == b.run_id and b.run_exp == "follow":
            b.ended = vals[1]
        elif rtype == "FOLLOW" and len(vals) >= 17:
            # its own pose while out (it may have driven its buffered steps: a circle turns it right round); the
            # rejoin takes its heading from here when the camera gives none
            boot = EC.boot_of_run(vals[0]) or None    # its follow run's id carries its boot ID (no status needed)
            try:
                ms, x, y, th = int(float(vals[1])), float(vals[7]), float(vals[8]), float(vals[9])
                b.note_clock(t, ms)
                if vals[3] != "nopose":
                    b.pose, b.pose_t = (x, y, th), t
                    b.follow_back_t = t
            except (ValueError, IndexError):
                pass
        if boot:
            b.boot_back = boot
        if getattr(b, "back_t", None) is None:
            b.back_t = t
            self.log("robot_back", robot=b.name, why=b.out, rejoins=self.rejoinable(b))
            if self.rejoinable(b):
                self.say("%s is talking again: checking it (boot ID, camera) before it rejoins" % b.name)
                self.send(b.name, "status")
                b.status_asked_t = t
            elif not getattr(b, "back_said", False) and getattr(b, "out_kind", None) != "recovering":
                b.back_said = True
                self.say("%s is talking again (%s); it stays out of this run, as a fixed obstacle" % (b.name, b.out))

    def rejoin_stall(self, t):
        """After a rejoin: no step finished by anyone for REJOIN_STALL_S (and no re-plan running) = the joint plan is
        stuck (dry run 70 x 130: Left waited for Right, which had finished and was dancing): the returning robot
        becomes a fixed obstacle again and the others re-plan, as before rejoining existed."""
        rj = getattr(self, "rejoined", None)
        if not rj or rj not in self.names or self.planning is not None:
            return
        last = max([st.get("t_done") or 0.0 for b in self.bots.values() for st in b.stat] +
                   [getattr(self, "rejoined_t", 0.0)])
        if t - last < self.REJOIN_STALL_S:
            return
        b = self.bots[rj]
        self.rejoined = None
        why = "nothing moved on for %.0f s after it rejoined" % (t - last)
        self.log("rejoin_undone", robot=rj, why=why)
        self.say("%s: %s: it is held where it is, as a fixed obstacle; the others carry on" % (rj, why))
        self.retire(b, "heard again, but %s" % why, kind="restarted")
        self.retry_later(b, why)

    def rejoin_watch(self):
        if not self.rejoin_on() or not self.t_run_started() or self.planning is not None:
            return
        t = now()
        self.rejoin_stall(t)
        if not self.names and HELD_NEVER_END and any(b.out and self.rejoinable(b) for b in self.bots.values()):
            # (e16, 6a; never give up) every robot held: the run holds them safely and waits for them, said every minute
            if t - getattr(self, "_all_held_said", -1e9) >= 60.0:
                self._all_held_said = t
                held = [b.name for b in self.bots.values() if b.out and self.rejoinable(b)]
                self.log("all_held_wait", robots=held, since_s=round(t - min(getattr(b, "out_t", t) for b in
                                                                              self.bots.values() if b.out), 1))
                self.say("NEEDS A HAND: every robot is held (%s), still and safe; the run waits for them and goes on as "
                         "soon as one is heard and checked again (it never ends by itself: Ctrl+C or Stop ends it; "
                         "MIRLT_HELD_NEVER_END=0 gives the old 90 s end)" % ", ".join(held))
        elif not self.names and not any(b.out and self.rejoinable(b) and t - getattr(b, "out_t", 0.0) <
                                         self.REJOIN_WAIT_S for b in self.bots.values()):
            self.abort = "every robot is out (none heard again within %.0f s)" % self.REJOIN_WAIT_S
            self.say(self.abort)
            return
        for b in self.bots.values():
            if not b.out or not self.rejoinable(b):
                continue
            if now() < (getattr(b, "retry_at", None) or 0.0):
                continue
            if b.last_heard is None or getattr(b, "back_t", None) is None:
                continue
            if t - b.last_heard > 0.5:
                # a robot that restarted is idle and says little (a CLOCK record now and then): ask it, its answer
                # counts as heard. Quiet for 3 s even so: it went quiet again
                if t - b.last_heard > 3.0:
                    b.back_t = None
                elif t - getattr(b, "status_asked_t", 0.0) > 1.0:
                    self.send(b.name, "status")
                    b.status_asked_t = t
                continue
            if getattr(b, "back_t", None) is None or t - b.back_t < self.REJOIN_HEARD_S:
                continue
            boot0 = b.boot_start or b.boot
            if getattr(b, "boot_back", None) is None:
                if t - getattr(b, "status_asked_t", 0.0) > 1.0:
                    self.send(b.name, "status")
                    b.status_asked_t = t
                if t - b.back_t < self.REJOIN_BOOT_WAIT_S:
                    continue
            same = getattr(b, "boot_back", None) is not None and b.boot_back == boot0 and b.ended is None and \
                b.out_kind == "silent"
            cam = getattr(b, "last_cam", None)
            if (cam is None or t - (getattr(b, "cam_t", None) or 0.0) > self.REJOIN_CAM_S) and not same:
                self.recheck(b)                  # the re-check's facing check finds it by its lights (and tells the bridge)
                return
            if cam is None or t - (getattr(b, "cam_t", None) or 0.0) > self.REJOIN_CAM_S:
                if not getattr(b, "rejoin_cam_said", False):
                    b.rejoin_cam_said = True
                    self.say("%s: heard again, but no camera sees it: it stays held until one does" % b.name)
                continue
            if same:
                self.rejoin(b, "heard again on the same boot (%s) after %s" % (b.boot_back, b.out.split(";")[0]))
            else:
                self.recheck(b)
            return

    def rejoin(self, b, why):
        """Back into the plan: its steps as they were when it went out (nothing to wait for), its pose from the camera,
        its out-obstacle off the map; then everyone re-plans from where they are."""
        n = b.name
        t_end = now() + 2.0
        while now() < t_end and (getattr(b, "cam_t", None) is None or now() - b.cam_t > self.REJOIN_FRESH_S):
            self.drain()          # (11-05h) the re-check blocked the fixes for a while: take the newest before judging
            self.fixes()
            time.sleep(0.05)
        cam = b.last_cam
        if cam is None or now() - (getattr(b, "cam_t", None) or 0.0) > self.REJOIN_FRESH_S:
            # (11-05h, rehearsal s3: a robot rejoined on the camera's LAST spot, 36 cm from where it really was, and
            # drove to 5 cm of another robot) its pose is set only from a fresh camera fix; else it stays held
            self.log("rejoin_refused", robot=n, why="no fresh camera position",
                     cam_age_s=None if getattr(b, "cam_t", None) is None else round(now() - b.cam_t, 2))
            self.retry_later(b, "cannot rejoin yet: the camera's position of it is %s" % (
                "missing" if cam is None else "%.1f s old" % (now() - b.cam_t)))
            if n not in self.names:
                self.plan["robots"].pop(n, None)
            self.handle_obstacle(None)
            return
        own_fresh = b.pose is not None and now() - (getattr(b, "follow_back_t", None) or 0.0) < 1.0
        th = cam[2] if cam[2] is not None else (b.pose[2] if own_fresh else None)
        if th is None and getattr(b, "out_kind", None) == "silent" and now() - (getattr(b, "back_t", None) or now()) < 5.0:
            return                              # its own pose (FOLLOW) not heard yet: try again in a moment
        if th is None:
            self.retry_later(b, "cannot rejoin yet: no heading (no camera heading, no pose of its own)")
            if n not in self.names:
                self.plan["robots"].pop(n, None)
            self.handle_obstacle(None)          # the others were held for its re-check: they go on
            return
        self.control(b, "halt")                 # whatever it still had queued is dropped
        self.send(n, "fpose %.2f %.2f %.2f 1.0 3.0" % (cam[0], cam[1], th))
        b.pose, b.pose_t = (cam[0], cam[1], th), now()
        b.acc_t, b.acc_odo = now(), getattr(b, "odo", 0.0)         # its pose is the camera's again (confidence)
        entry = b.out_entry
        for st in entry["steps"]:
            st["after"] = []
        self.plan["robots"][n] = entry
        stat = [dict(x) for x in b.out_stat]
        for x in stat:
            if x["status"] in ACTIVE:
                x["status"], x["info"] = "failed", "halted: rejoined"
        b.ids, b.stat = list(b.out_ids), stat
        b.id_index = {sid: i for i, sid in enumerate(b.ids) if sid is not None}
        self.m["obstacles"] = [o for o in self.m.get("obstacles", []) if o.get("id") != "out-%s" % n]
        self.m = FM.normalize(self.m)
        self.names = [x for x in self.bots if x in self.names or x == n]
        was = b.out
        b.out = b.out_kind = None
        b.back_t = b.boot_back = None
        b.back_said = b.rejoin_cam_said = False
        b.boot_start = b.boot = b.boot or b.boot_start
        b.rejoins_n = getattr(b, "rejoins_n", 0) + 1
        rec = dict(robot=n, why=why, was=was, t_run=round(now() - self.t_run, 2), cam=list(cam))
        self.rejoins = getattr(self, "rejoins", []) + [rec]
        self.log("robot_rejoined", **rec)
        self.say("%s rejoins (%s): its pose set from the camera (%.1f, %.1f); everyone re-plans from where they are"
                 % (n, why, cam[0], cam[1]))
        self.rejoin_pending = (n, now() + self.REJOIN_TRIAL_S)
        self.rejoined, self.rejoined_t = n, now()
        self.handle_obstacle(None)
        if self.planning is not None:
            # it sends no step of its old plan before the new plan is in: that plan began where it stood when it went
            # out (a robot held since the start: its start spot), not where it is (30 Oct simulator: Right drove its
            # old first leg head-on towards Left, bodies 3.9 cm, before the re-plan came)
            first = next((i for i, x in enumerate(b.stat) if x["status"] == "unsent"), len(b.stat))
            self.limit[n] = min(self.limit.get(n, first), first)

    def recheck(self, b):
        """A robot that restarted (a new boot ID, or its follow run ended) while out: everyone else holds while it is
        checked as at the start (status, battery, its settings, calibration if it lost it, the facing check, its follow
        run, the floor check). Passed: it rejoins and everyone re-plans. Failed: it stays out, said why."""
        n = b.name
        self.say("%s restarted or its run ended (boot %s, was %s): everyone holds while it is checked again"
                 % (n, getattr(b, "boot_back", None) or "?", b.boot_start or b.boot))
        for o in self.bots.values():
            if o.name in self.names:
                self.control(o, "halt")
        self.log("recheck_start", robot=n, boot=getattr(b, "boot_back", None))
        # its part of the plan, back for the checks (the facing check reads its start), starting where the camera has it
        cam = getattr(b, "last_cam", None) or [None, None, None]
        entry = dict(b.out_entry)
        if cam[0] is not None:
            entry["start"] = [cam[0], cam[1], cam[2] if cam[2] is not None else
                              (b.pose[2] if b.pose else b.out_entry["start"][2])]
        self.plan["robots"][n] = entry
        b.rechecking = True
        try:
            ok, words = recheck_robot(self, self.link, self.args, n, self.source)
        except SystemExit as e:
            ok, words = False, str(e)
        except Exception as e:      # noqa: BLE001  (a failed re-check keeps it out; it never stops the others)
            ok, words = False, "the re-check failed (%s: %s)" % (type(e).__name__, e)
        finally:
            b.rechecking = False
        self.log("recheck_done", robot=n, ok=ok, words=words)
        if not ok:
            self.plan["robots"].pop(n, None)
            self.retry_later(b, "its re-check did not pass (%s)" % words)
            self.handle_obstacle(None)
            return
        b.boot_start = b.boot
        b.out_kind = "silent"
        if getattr(b, "last_cam", None) is None:
            self.plan["robots"].pop(n, None)
            self.retry_later(b, "no camera position after the re-check")
            self.handle_obstacle(None)
            return
        self.rejoin(b, "restarted, checked again: %s" % words)

    # -- confidence at every point (2026-11-03, Nick 2 Nov 21:01: "the highest confidence positional assessment it
    # can make every step of the way, sometimes that won't be 100%") ----------------------------------------------------
    # Every position is said with a "+-" (95 %: about 2 standard deviations) and where it came from:
    #   two cameras / one camera   the bridge's own sigma for the fix (its pixel noise), combined with the camera fit's
    #                              own error (the setup's site_fit.json: robot-tag residuals in px / px per cm, at
    #                              least its sticker uncertainty), at least half the two cameras' distance apart, plus
    #                              the size of a bump the bridge reports for that camera;
    #   odometry only              the robot's own pose (wheels and gyro) since the last camera fix it accepted: that
    #                              fix's +- plus ODO_P95_PER_CM for every cm driven since.
    CONF_Z = 2.0              # +- = 2 sigma (about 95 %). No DOE: a convention
    ODO_P95_PER_CM = 0.07     # the drift's p95 per cm driven with no camera: 29 Oct logs, 4,124
                              # stretches of Left's and Center's straight drives (docs/build-reports/e02-oct29-start-
                              # straighten-blind-strip.md, section 3: p95 2.17 cm after 5 cm to 4.44 after 40, i.e.
                              # +0.065-0.07 per cm over the camera's own error). MEASURED, but only up to 40 cm and on
                              # straight drives: beyond 40 cm and over turns it is extrapolated (DOE backlog)
    CAM_CAL_DEFAULT_CM = 0.5  # CHOSEN: a tag camera's fit error (1 sigma) until its site_fit.json is read. DOE backlog:
                              # tape-measured points against the camera's at the school table
    PPC_DEFAULT = 5.0         # CHOSEN: px per cm when no fix has said it (1280 x 960 at about 2 m). Only a fallback
    LIVE_FIX_S = 0.6          # a camera fix this recent = the camera sees it now (as BLIND_AFTER_S)

    def note_fix(self, b, ev, src):
        cams = src.split("+") if "+" in src else (["overhead", "side"] if src == "both" else [src])
        f = dict(t=now(), x=float(ev["x"]), y=float(ev["y"]), sigma=float(ev.get("sigma") or 0.5), cams=cams,
                 apart=float(ev.get("apart") or 0.0), ppc=ev.get("ppc"), tagged=str(ev.get("src", "")).startswith(
                     "tag_bridge"))
        f["unc"] = self.cam_unc(f)
        lg = b.__dict__.setdefault("fix_log", [])
        lg.append(f)
        if len(lg) > 300:
            del lg[:-200]
        b.last_fix_unc = f["unc"]
        if not hasattr(b, "acc_t"):
            b.acc_t, b.acc_odo = f["t"], getattr(b, "odo", 0.0)

    def odometer(self, b, t, x, y, used):
        last = getattr(b, "odo_last", None)
        b.odo = getattr(b, "odo", 0.0) + (math.hypot(x - last[0], y - last[1]) if last else 0.0)
        b.odo_last = (x, y)
        try:
            u = int(float(used))
        except (TypeError, ValueError):
            return
        if u > getattr(b, "used_n", -1):
            if getattr(b, "used_n", -1) >= 0:
                b.acc_t, b.acc_odo = t, b.odo       # it took a camera fix: its own pose is the camera's again
            b.used_n = u
        elif u < getattr(b, "used_n", -1):
            b.used_n = u                            # a new follow run counts from 0 again (a rejoin, a re-check)

    def cam_info(self):
        """Per camera: the fit's error (cm, 1 sigma) from site_fit.json (read again every 15 s until found) and the
        bridge's bumps (events.jsonl: camera_moved / camera_back)."""
        t = now()
        ci = self.__dict__.setdefault("_cam_info", {"cal": {}, "moved": {}, "fit_t": 0.0, "ev_t": 0.0, "ev_pos": 0,
                                                     "fit": None})
        fp = getattr(self.args, "cam_fit", None)
        if fp and ci["fit"] is None and t - ci["fit_t"] > 15.0:
            ci["fit_t"] = t
            try:
                with open(fp) as fh:
                    ci["fit"] = json.load(fh)
                self.log("cam_fit_read", path=fp, rms_px=ci["fit"].get("rms_px"))
            except (OSError, ValueError):
                pass
        ep = getattr(self.args, "bridge_events", None)
        if ep and t - ci["ev_t"] > 2.0:
            ci["ev_t"] = t
            try:
                with open(ep) as fh:
                    fh.seek(ci["ev_pos"])
                    for ln in fh:
                        if not ln.endswith("\n"):
                            break
                        ci["ev_pos"] += len(ln.encode())
                        try:
                            ev = json.loads(ln)
                        except ValueError:
                            continue
                        if ev.get("kind") == "camera_moved":
                            ci["moved"][ev.get("cam")] = float(ev.get("about_cm") or 0.0)
                            self.log("camera_moved_seen", cam=ev.get("cam"), about_cm=ev.get("about_cm"))
                            self.bump_back(ev.get("cam"), float(ev.get("about_cm") or 0.0))
                        elif ev.get("kind") == "camera_back":
                            ci["moved"].pop(ev.get("cam"), None)
            except OSError:
                pass
        return ci

    BUMP_LOOKBACK_S = 12.0    # the bridge says a camera moved after 2 checks 5 s apart: points stood at this long before
                              # it are counted as possibly moved too. No DOE: from the bridge's own check timing

    def bump_back(self, cam, about):
        """A camera reported moved: the points recorded in the BUMP_LOOKBACK_S before, from that camera, get its shift
        in their +- (the move happened before it was noticed)."""
        t = now()
        for q in self.__dict__.get("points", []):
            if q.get("source") in (None, "none", "odometry only") or q.get("t_abs") is None:
                continue
            if t - q["t_abs"] > self.BUMP_LOOKBACK_S or ("cam " + cam) in q.get("words", ""):
                continue
            cams = ["overhead", "side"] if q["source"] == "two cameras" else [q["source"][len("one camera ("):-1]]
            if cam not in cams:
                continue
            q["plus_minus_cm"] = round(q["plus_minus_cm"] + about * (0.5 if len(cams) > 1 else 1.0), 1)
            q["words"] = re.sub(r"\u00b1[0-9.]+ cm", "\u00b1%.1f cm" % q["plus_minus_cm"], q["words"], count=1) + \
                " (cam %s reported moved %.0f s later: counted in)" % (cam, t - q["t_abs"])
            self.log("point_confidence_bumped", robot=q["robot"], point=q["point"], cam=cam, plus_minus_cm=q["plus_minus_cm"])
            self.say("POINT %s (updated)" % q["words"])

    FRAME_K = 2.0             # CHOSEN (2026-11-03): the table's frame (where the corners are) has its own error, the
                              # fit's residual on the corners (clicked corners, or the corner tags), in px. Not in the
                              # +- before: with corners clicked, 6 of 40 points fell outside their +- in the school
                              # simulator (34 of 40), with corner tags 3 of 40, all on the frame's bias (a whole half's
                              # points off the same way). A 4-corner frame's residual underestimates the click error
                              # (the fit absorbs part of it) and points far from the corners are extrapolated: x 2.
                              # 0 = the old +- (no frame term). DOE note: set from tape-measured points at the school
                              # table (the real click and lens errors), not from the simulator alone

    def cam_cal(self, cam, ppc=None):
        fit = self.cam_info()["fit"]
        if not fit:
            return self.CAM_CAL_DEFAULT_CM
        rp = ((fit.get("rms_px") or {}).get(cam) or {})
        px = rp.get("tags") if rp.get("tags") is not None else rp.get("ref")
        sd = [float(v) for v in (fit.get("sticker_sd_cm") or {}).values() if v is not None]
        sd_med = statistics.median(sd) if sd else 0.0
        if px is None:
            return max(sd_med, self.CAM_CAL_DEFAULT_CM)
        base = max(sd_med, float(px) / float(ppc or self.PPC_DEFAULT))
        # the frame's own error (2026-11-03): clicked corners' residual, else the corner tags'
        fpx = rp.get("corners") if rp.get("corners") is not None else rp.get("ref")
        if fpx is None or not self.FRAME_K:
            return base
        return math.sqrt(base ** 2 + (self.FRAME_K * float(fpx) / float(ppc or self.PPC_DEFAULT)) ** 2)

    def cam_unc(self, f):
        """+- (cm, 95 %) of one camera fix."""
        moved = self.cam_info()["moved"]
        if f.get("tagged"):
            cals = [self.cam_cal(c, f.get("ppc")) for c in f["cams"]]
            cal = 1.0 / math.sqrt(sum(1.0 / max(c, 0.05) ** 2 for c in cals)) if cals else self.CAM_CAL_DEFAULT_CM
        else:
            cal = 0.0                     # the lights' bridge and the simulator's own fixes: their sigma says it all
        sig = math.sqrt(max(f["sigma"], 0.0) ** 2 + cal ** 2)
        if f.get("apart"):
            sig = max(sig, f["apart"] / 2.0)
        pm = self.CONF_Z * sig
        bump = [moved[c] for c in f["cams"] if c in moved]
        if bump:
            # a bumped camera's shift is a bias, not noise: added once (all of it with that camera alone, half when
            # the other camera, not moved, is in the fix; first build counted it twice: point 6 of the bump run)
            pm += max(bump) * (0.5 if len(bump) < len(f["cams"]) else 1.0)
        return round(pm, 2)

    @staticmethod
    def source_words(cams):
        return "two cameras" if len(cams) > 1 else "one camera (%s)" % cams[0]

    def odo_unc(self, b):
        d = max(0.0, getattr(b, "odo", 0.0) - getattr(b, "acc_odo", 0.0))
        base = getattr(b, "last_fix_unc", None)
        base = 2.0 if base is None else base
        return round(base + self.ODO_P95_PER_CM * d, 2), round(d, 1)

    def conf_now(self, b, t=None):
        """{x, y, plus_minus_cm, source, since_fix_cm, since_fix_s} now: the camera's when it sees the robot, else
        the robot's own pose with the odometry's growing +-."""
        t = t or now()
        lg = getattr(b, "fix_log", None) or []
        if lg and t - lg[-1]["t"] <= self.LIVE_FIX_S:
            f = lg[-1]
            return {"x": round(f["x"], 1), "y": round(f["y"], 1), "plus_minus_cm": f["unc"],
                    "source": self.source_words(f["cams"]), "since_fix_cm": 0.0, "since_fix_s": 0.0}
        if b.pose is None:
            return None
        u, d = self.odo_unc(b)
        return {"x": round(b.pose[0], 1), "y": round(b.pose[1], 1), "plus_minus_cm": u, "source": "odometry only",
                "since_fix_cm": d, "since_fix_s": round(t - getattr(b, "acc_t", t), 1)}

    def point_step(self, b, stp, state, t):
        """The dwell at a production point ("at point 7"): the fixes during it make the point's estimate."""
        lab = str(stp.get("label") or "")
        if not lab.startswith("at ") or stp.get("type") != "wait":
            if state == "done" and lab:
                self.__dict__.setdefault("done_labels", set()).add(lab)
            return
        if state == "started":
            d = getattr(b, "dwell", None)
            tag = getattr(b, "last_tag", None)
            if DWELL_KEEP and d and d["point"] == lab[3:] and t - d["t0"] < self.DWELL_KEEP_S and self.dwell_still(b, d):
                # (e12 item 9, 9 Nov P15) the same stop again after a halt, and the robot did not move: one stop, its
                # dwell from the first start (not a short second one), marked interrupted
                d["interrupted"] = True
                self.log("dwell_kept", robot=b.name, point=d["point"], t0=round(d["t0"], 3), restarted=round(t, 3))
            else:
                b.dwell = dict(point=lab[3:], t0=t, pose=list(b.pose) if b.pose else None,
                               tag=tag if (tag and t - tag[0] < 2.0) else None)
        elif state == "done":
            self.__dict__.setdefault("done_labels", set()).add(lab)
            d = getattr(b, "dwell", None)
            t0 = d["t0"] if d and d["point"] == lab[3:] else t - float(stp.get("t_end", 0) - stp.get("t_start", 0))
            b.dwell = None
            self.record_point(b, lab[3:], t0, t, interrupted=bool(d and d.get("interrupted")))

    DWELL_STILL_CM = 2.0      # (e12 item 9) "did not move" during an interrupted dwell: the roof-tag reads (else its own
                              # pose) within this. Chosen (Nick's "about 2 cm"). No DOE
    DWELL_KEEP_S = 60.0       # ... and the stop restarted within this of its first start. Chosen. No DOE

    def dwell_still(self, b, d):
        """(e12 item 9) The robot stayed put since the dwell started: its newest roof-tag read within DWELL_STILL_CM of
        the one at the start when both exist (a lock fix does not count), else its own pose."""
        tag, t = getattr(b, "last_tag", None), now()
        if d.get("tag") and tag and t - tag[0] < 2.0:
            return math.hypot(tag[1] - d["tag"][1], tag[2] - d["tag"][2]) <= self.DWELL_STILL_CM
        if d.get("pose") and b.pose:
            return math.hypot(b.pose[0] - d["pose"][0], b.pose[1] - d["pose"][1]) <= self.DWELL_STILL_CM
        return False

    def record_point(self, b, point, t0, t1, interrupted=False):
        lg = [f for f in (getattr(b, "fix_log", None) or []) if t0 - 0.05 <= f["t"] <= t1 + 0.05]
        own = list(b.pose) if b.pose else None
        rec = dict(robot=b.name, point=point, t_run=round(t1 - self.t_run, 2) if self.t_run else None,
                   dwell_s=round(t1 - t0, 2), n_fixes=len(lg), t_abs=round(t1, 3), interrupted=int(bool(interrupted)))
        if lg:
            xs, ys = [f["x"] for f in lg], [f["y"] for f in lg]
            x, y = statistics.median(xs), statistics.median(ys)
            spread = math.sqrt(statistics.pvariance(xs) + statistics.pvariance(ys)) if len(lg) > 1 else 0.0
            two = sum(1 for f in lg if len(f["cams"]) > 1) / float(len(lg))
            # each fix's +- with what is known NOW (bump run, 3 Nov: a camera_moved event read after the stand's
            # fixes arrived left point 4 at +-1.2 cm, 3.1 cm off the truth)
            unc = max(statistics.median([self.cam_unc(f) for f in lg]), round(self.CONF_Z * spread, 2))
            cams = ["overhead", "side"] if two >= 0.5 else [statistics.mode([f["cams"][0] for f in lg
                                                                             if len(f["cams"]) == 1] or ["?"])]
            src = self.source_words(cams)
            rec.update(x=round(x, 2), y=round(y, 2), plus_minus_cm=round(unc, 1), source=src,
                       two_camera_share=round(two, 2), since_fix_cm=0.0, since_fix_s=0.0)
            words = "%s (%s): \u00b1%.1f cm, %s" % (point, b.name, unc, src)
        elif own is not None:
            u, d = self.odo_unc(b)
            ago = t1 - getattr(b, "acc_t", t1)
            rec.update(x=round(own[0], 2), y=round(own[1], 2), plus_minus_cm=round(u, 1), source="odometry only",
                       two_camera_share=0.0, since_fix_cm=d, since_fix_s=round(ago, 1))
            if getattr(b, "fix_log", None) or getattr(b, "last_fix_unc", None) is not None:
                words = "%s (%s): \u00b1%.1f cm, odometry for %.0f cm since the last camera fix (%.0f s ago)" % (
                    point, b.name, u, d, ago)
            else:
                # no camera this run (tracking none, 2026-11-03): from the hand-placed start, not a camera fix
                words = "%s (%s): \u00b1%.1f cm, odometry for %.0f cm since the start (no camera this run)" % (
                    point, b.name, u, d)
        else:
            rec.update(x=None, y=None, plus_minus_cm=None, source="none", since_fix_cm=None, since_fix_s=None)
            words = "%s (%s): no position (no camera fix and no pose of its own)" % (point, b.name)
        if own is not None:
            rec.update(own_x=round(own[0], 2), own_y=round(own[1], 2))
            if rec.get("x") is not None and lg:
                rec["own_vs_camera_cm"] = round(math.hypot(own[0] - rec["x"], own[1] - rec["y"]), 2)
        moved = self.cam_info()["moved"]
        if moved:
            rec["cameras_moved"] = dict(moved)
            words += " (camera %s reported moved: counted in)" % ", ".join(sorted(moved))
        rec["words"] = words
        self.__dict__.setdefault("points", []).append(rec)
        self.log("point_confidence", **rec)
        self.say("POINT " + words)
        self.point_csv(rec)

    POINT_COLS = ["robot", "point", "t_run", "x", "y", "plus_minus_cm", "source", "n_fixes", "two_camera_share",
                  "since_fix_cm", "since_fix_s", "own_x", "own_y", "own_vs_camera_cm", "dwell_s", "words"]

    def point_csv(self, rec):
        try:
            if getattr(self, "_pts_f", None) is None:
                self._pts_f = open(os.path.join(self.outdir, "points.csv"), "a", newline="")
                self._pts_w = csv.writer(self._pts_f)
                if self._pts_f.tell() == 0:
                    self._pts_w.writerow(self.POINT_COLS)
            self._pts_w.writerow(["" if rec.get(k) is None else rec.get(k) for k in self.POINT_COLS])
            self._pts_f.flush()
        except OSError:
            pass

    LIVE_EVERY_S = 0.5    # live.json for Mission Control's Run screen. No DOE: a display rate

    def write_live(self, final=False):
        """live.json: each robot now (camera spot, own pose, battery, what it is doing, its last step's result,
        how far its middle is from the nearest open side), for Mission Control's Run screen."""
        t = now()
        if self.__dict__.get("mfeed") is None:       # (12 Nov) the Pi efficiency monitor's feed: write_live runs once a
            import monitor_feed as MF_                # loop, so its period is the loop's (read-only, monitor_feed.py)
            self.mfeed = MF_.Feed(self.outdir)
        self.mfeed.tick(None)
        if not final and t - getattr(self, "_live_t", 0.0) < self.LIVE_EVERY_S:
            return
        self._live_t = t
        f = self.m["floor"]
        sides = self.m.get("sides") or {}
        rob = {}
        for n, b in self.bots.items():
            cam = getattr(b, "last_cam", None)
            p = cam if cam else (list(b.pose) if b.pose else None)
            edge = None
            if p:
                ds = []
                for sd, v in (("x_min", p[0] - f["x_min"]), ("x_max", f["x_max"] - p[0]), ("y_min", p[1] - f["y_min"]),
                              ("y_max", f["y_max"] - p[1])):
                    if (sides.get(sd) or {"kind": "barrier" if not sides else "unknown"})["kind"] != "barrier":
                        ds.append(v)
                edge = round(min(ds), 1) if ds else None
            doing, last = "", ""
            if n in self.names:
                steps = self.steps(n)
                for i, st in enumerate(b.stat):
                    if st["status"] in ACTIVE:
                        doing = steps[i].get("label") or steps[i]["type"]
                        break
                fin = [st for st in b.stat if st["status"] in ("done", "failed")]
                if fin:
                    lastst = max(fin, key=lambda q: q.get("t_done", 0.0) or 0.0)
                    last = "%s %s" % (lastst["status"], (lastst.get("info") or "")[:60])
            rob[n] = {"cam": cam, "own": [round(v, 2) for v in b.pose] if b.pose else None, "battery": b.battery,
                      "doing": doing, "last": last, "edge_cm": edge, "out": b.out,
                      "done": sum(st["status"] == "done" for st in b.stat), "steps": len(b.stat),
                      "held": bool(b.out) and self.rejoinable(b), "stop_cm": getattr(b, "stop_cm", None),
                      "conf": self.conf_now(b, t)}
        live = {"t": round(t, 2), "t_run": round(t - self.t_run, 1) if self.t_run_started() else None,
                "plan_no": self.plan_no, "replans": len(self.replans), "result": self.abort or ("done" if final else None),
                "robots": rob, "obstacles": self.live_obstacles(),
                "points": [{k: q.get(k) for k in ("robot", "point", "plus_minus_cm", "source", "words")}
                           for q in self.__dict__.get("points", [])],
                "keep_going": self.keep_going(),
                "kept_going": [q["words"] for q in self.__dict__.get("kept_going", [])][-12:],
                "skipped": self.__dict__.get("skipped", []),
                "reassigned": [{k: q[k] for k in ("point", "frm", "to")} for q in self.__dict__.get("reassigned", [])]}
        mp = getattr(self, "mirror_points", None)
        if self.mirror_cfg():
            live["mirror"] = {"robots": self.mirror_cfg()["robots"], "n": len(mp or []),
                              "last": mp[-1] if mp else None,
                              "worst_s": max((q["time_err_s"] for q in mp), default=None) if mp else None,
                              "worst_cm": max((q["pos_err_cm"] for q in mp), default=None) if mp else None}
        try:
            tmp = os.path.join(self.outdir, "live.json.tmp")
            with open(tmp, "w") as fh:
                json.dump(live, fh)
            os.replace(tmp, os.path.join(self.outdir, "live.json"))
        except OSError:
            pass

    def silence_watch(self):
        lim = getattr(self.args, "silent_s", EC.SILENT_S)
        if not lim or not self.t_run_started():
            return
        t = now()
        for n in list(self.names):
            b = self.bots[n]
            if b.last_heard is not None and t - b.last_heard > lim:
                self.retire(b, "nothing heard from it for %.1f s (battery dead or WiFi gone)" % (t - b.last_heard),
                            kind="silent")

    def status_poll(self):
        t = now()
        if t - getattr(self, "_status_t", 0.0) < STATUS_EVERY_S:
            return
        self._status_t = t
        floor = getattr(self.args, "battery_floor", EC.BATTERY_FLOOR_PCT)
        for n in self.names:
            b = self.bots[n]
            self.send(n, "status")
            if b.battery is not None and floor and b.battery < floor and not getattr(b, "low_said", False):
                b.low_said = True
                self.say("%s's battery is at %d %%, under the %s floor: it finishes this run; charge it before the "
                         "next" % (n, b.battery, floor_words(self.args)))
                self.log("battery_low", robot=n, battery=b.battery)

    def request_replan(self, why):
        if self.pending_replan is None:
            self.pending_replan = (why, now() + 0.3)

    # -- keep going (2026-11-03, Nick 2 Nov 21:01: "stay focused on finishing the run with the highest confidence
    # positional assessment it can make every step of the way") -------------------------------------------------------
    # --on-problem keep_going (the site's on_problem; mission_run passes it): a problem that used to stop everyone is
    # said once in plain words, logged (coordinator.jsonl "keep_going", the summary's kept_going) and worked round.
    # Still real stops: the Pi's card nearly full, the operator's Stop / stopall / Ctrl+C, every robot out, and the
    # time limit (stretched to the newest plan's end). --on-problem stop (default, home): exactly as before.
    REPLAN_SPACING_S = 5.0     # past --max-replans, re-plans at least this far apart. Chosen; no DOE (a guard against a
                               # loop; the robots stand meanwhile)
    REPLAN_FAIL_MAX = 3        # failed re-plans in a row before the robot that cannot be planned is set out. Chosen;
                               # no DOE (a guard)
    DEADLINE_TAIL_S = 120.0    # keep going: the time limit is at least the newest plan's planned end plus this. Chosen;
    DEADLINE_MAX_X = 3.0       # no DOE; never more than this many times --timeout-s (a hang still ends)

    def keep_going(self):
        return getattr(self.args, "on_problem", "stop") == "keep_going"

    def edge_end_strip(self, b):
        """Keep going: a robot whose edge guard ended its follow run: the spot becomes a no-go strip on the map, so no
        robot is planned there again (it is then held and checked again, as the self-correcting run does)."""
        if not (self.keep_going() and SC.run_outcome(b.ended) == SC.CALIB_LIMITED and b.pose):
            return None
        self.n_sensor_obstacles += 1
        ob = self.edge_boundary(b, b.pose, "")
        self.add_obstacle(ob)
        self.carry_on("edge_guard_ended", b.name, "%s: its edge guard ended its run at (%.0f, %.0f): that spot is a "
                      "no-go strip now; %s is checked again, the others go on" % (b.name, b.pose[0], b.pose[1], b.name),
                      obstacle=ob)
        return ob

    def holds_one(self):
        """One robot's trouble holds that robot, never stops everyone: the self-correcting run (mission_run's
        --straighten, 30 Oct) or keep going."""
        return bool(getattr(self, "self_correct", False)) or self.keep_going()

    def carry_on(self, kind, robot, words, **kw):
        """A problem that would have stopped the run, worked round: said once (per kind and robot), always logged."""
        rec = dict(kind=kind, robot=robot, words=words, t_run=round(now() - self.t_run, 2)
                   if getattr(self, "t_run", None) else None)
        rec.update({k: v for k, v in kw.items() if k not in rec})
        self.__dict__.setdefault("kept_going", []).append(rec)
        self.log("keep_going", **rec)
        said = self.__dict__.setdefault("_carry_said", set())
        if (kind, robot) not in said:
            said.add((kind, robot))
            self.say("KEEP GOING: " + words)
            self.warnings = getattr(self, "warnings", []) + [words]

    def deadline(self, timeout_s):
        if not self.keep_going():
            return self.t_run + timeout_s
        end = (self.plan_start or self.t_run) + float(self.plan.get("summary", {}).get("makespan_s") or 0.0)
        d = min(self.t_run + self.DEADLINE_MAX_X * timeout_s, max(self.t_run + timeout_s, end + self.DEADLINE_TAIL_S))
        if d > self.t_run + timeout_s and not getattr(self, "_deadline_said", False) and now() > self.t_run + timeout_s:
            self._deadline_said = True
            self.carry_on("time_limit", None, "the time limit (%.0f s) is reached but the newest plan ends later: "
                          "the run goes on to %.0f s at most" % (timeout_s, d - self.t_run))
        return d

    def gap_fix_on(self):
        return getattr(self.args, "gap_fix", "on") != "off"

    @staticmethod
    def obstacle_key(o):
        """An obstacle by its shape (ids are renumbered when a map is normalised): source, points or centre/radius."""
        if o.get("points"):
            return (o.get("source"), tuple((round(float(p[0]), 1), round(float(p[1]), 1)) for p in o["points"]))
        c = o.get("center") or (0.0, 0.0)
        return (o.get("source"), round(float(c[0]), 1), round(float(c[1]), 1), round(float(o.get("radius", 0.0)), 1))

    def add_obstacle(self, ob):
        if ob and ob.get("id") not in {o.get("id") for o in self.m.get("obstacles", [])}:
            self.m["obstacles"].append(FM._norm_obstacle(ob, len(self.m["obstacles"])))
            self.m = FM.normalize(self.m)

    @staticmethod
    def point_of(label):
        """'point 7' / 'at point 7' -> 'point 7'; anything else None."""
        lab = str(label or "")
        if lab.startswith("at point "):
            return lab[3:]
        if lab.startswith("point "):
            return lab
        return None

    def current_task(self, n):
        """(index in the plan's visit_order, the item) of the work robot n is on now, or (None, None)."""
        steps = self.steps(n) if n in self.plan["robots"] else []
        b = self.bots[n]
        i = next((k for k, st in enumerate(b.stat) if st["status"] != "done"), None)
        entry = self.plan["robots"].get(n) or {}
        vo = [v for v in entry.get("visit_order") or [] if v != "park"]
        off = int(entry.get("task_offset") or 0)
        if i is None or i >= len(steps) or steps[i].get("task") is None:
            return None, None
        k = max(int(steps[i]["task"]), off) - off
        return (k, vo[k]) if 0 <= k < len(vo) else (None, None)

    def skip_current_point(self, n, why):
        """Keep going, no plan: robot n's current production point is taken out of its list (the goto and its dwell,
        or only the dwell when it is already there). Returns the point's label, or None (nothing to skip)."""
        k, item = self.current_task(n)
        pt = self.point_of(item.get("label")) if isinstance(item, dict) else None
        if pt is None:
            return None
        vo = self.plan["robots"][n]["visit_order"]
        idx = [j for j, v in enumerate(vo) if isinstance(v, dict) and j >= k and self.point_of(v.get("label")) == pt]
        for j in reversed(idx):
            del vo[j]
        self.__dict__.setdefault("skipped", []).append(dict(robot=n, point=pt, why=why, t_run=round(now() - self.t_run, 2)))
        self.log("point_skipped", robot=n, point=pt, why=why)
        return pt

    def defer_current_point(self, n, why):
        """(e14, "no re-trying a point that just failed") Robot n lost its tag on the way to its current point: the
        point is moved to the END of its list (once), said and logged, instead of being aimed at again at once; the
        second time it is skipped with the reason. MIRLT_RETRY_LOST=again: the old way (aimed at again). Returns (label,
        "deferred" | "skipped") or (None, None)."""
        if os.environ.get("MIRLT_RETRY_LOST", "defer") == "again":
            return None, None
        k, item = self.current_task(n)
        pt = self.point_of(item.get("label")) if isinstance(item, dict) else None
        if pt is None:
            return None, None
        done = self.__dict__.setdefault("deferred_pts", {})
        if done.get((n, pt)):
            return self.skip_current_point(n, why + "; a second time, after it was moved to the end once"), "skipped"
        vo = self.plan["robots"][n]["visit_order"]
        idx = [j for j, v in enumerate(vo) if isinstance(v, dict) and j >= k and self.point_of(v.get("label")) == pt]
        moved = [vo[j] for j in idx]
        for j in reversed(idx):
            del vo[j]
        vo.extend(moved)
        done[(n, pt)] = 1
        self.log("point_deferred", robot=n, point=pt, why=why)
        return pt, "deferred"

    def replan_trouble(self, why, ob, cause):
        """Keep going when a re-plan fails: everyone stands, the obstacle stays on the map, the robot whose step failed
        skips the point it was going to, and the plan is made again; after REPLAN_FAIL_MAX failures in a row the robot
        that cannot be planned is set out (its points go to the others) and the rest go on."""
        self.halt_all()
        self.add_obstacle(ob)
        self.replan_fails = getattr(self, "replan_fails", 0) + 1
        who = cause if cause in self.names else None
        if who is None and getattr(self, "trouble_cause", None) in self.names:
            who = self.trouble_cause           # (the retry re-plans with no cause: the robot that had it is remembered)
        self.trouble_cause = who
        if self.replan_fails > self.REPLAN_FAIL_MAX:
            n = who or next((x for x in (self.plan.get("stranded") or []) if x in self.names), None) or \
                (self.names[-1] if self.names else None)
            self.replan_fails = 0
            if n is not None:
                self.carry_on("replan_failed_out", n, "%s could not be planned %d times in a row (%s): it is set out "
                              "where it stands; the others go on" % (n, self.REPLAN_FAIL_MAX + 1, why))
                self.retire(self.bots[n], "no plan could be made for it (%s)" % why, kind="out")
                return
        pt = self.skip_current_point(who, why) if who else None
        if pt:
            self.carry_on("point_skipped", who, "%s: %s: %s is skipped (listed in the summary); the plan is made again "
                          "with its next point" % (who, why, pt))
        else:
            self.carry_on("replan_failed", who, "%s: everyone stands where they are and the plan is made again in "
                          "%.0f s" % (why, self.args.retry_s))
        self.pending_replan = ("again after a failed re-plan", now() + (0.5 if pt else self.args.retry_s))

    def note_skips(self, plan):
        """Production points the planner skipped (unreachable: 'robot X skips task N (point 7): ...'): listed once."""
        seen = {(x["robot"], x["point"]) for x in self.__dict__.get("skipped", [])}
        for w in plan.get("warnings") or []:
            mm = re.match(r"robot (\S+) skips task \d+ \(([^)]*)\): (.*)", str(w))
            if not mm:
                continue
            n, lab, why = mm.group(1), mm.group(2), mm.group(3)
            pt = self.point_of(lab)
            if pt is None or (n, pt) in seen or lab.startswith("at "):
                continue
            seen.add((n, pt))
            self.__dict__.setdefault("skipped", []).append(dict(robot=n, point=pt, why="cannot be reached: " + why,
                                                                 t_run=round(now() - self.t_run, 2) if self.t_run else 0))
            self.log("point_skipped", robot=n, point=pt, why=why)
            self.say("%s: %s cannot be reached (%s): skipped, listed in the summary; on to the next" % (n, pt, why))

    def remaining_points(self, entry, stat):
        """A robot's production points not done yet, from its plan entry: [[goto item, dwell item or None], ...]."""
        if not entry:
            return []
        done = set(self.__dict__.get("done_labels", set()))
        vo = [v for v in entry.get("visit_order") or [] if isinstance(v, dict)]
        pend = entry.get("pending") or {}
        if pend and pend.get("visits"):
            vo = [v for v in pend["visits"] if isinstance(v, dict)]
        out = []
        for j, v in enumerate(vo):
            lab = str(v.get("label") or "")
            if not lab.startswith("point ") or "goto" not in v:
                continue
            nxt = vo[j + 1] if j + 1 < len(vo) and str(vo[j + 1].get("label") or "") == "at " + lab else None
            if ("at " + lab) in done or (nxt is None and lab in done):
                continue
            out.append([v, nxt])
        return out

    def reassign_points(self, b, why):
        """Keep going, lost_robot_points reassign (Claude's assumption, 2026-11-03: Nick did not answer): the points the
        robot that went out had not done go to the robots still running (each to the one whose last point, or where it
        stands, is nearest, in order), before their trip home. If it rejoins later it only goes home."""
        if getattr(self.args, "lost_robot_points", "reassign") != "reassign":
            left = [g[0]["label"] for g in self.remaining_points(b.out_entry, b.out_stat)]
            if left:
                self.carry_on("points_not_done", b.name, "%s is %s: its points %s are not done unless it rejoins "
                              "(lost-robot points: skip)" % (b.name, "held" if self.rejoinable(b) else "out",
                                                             ", ".join(x.replace("point ", "") for x in left)))
            return
        groups = self.remaining_points(b.out_entry, b.out_stat)
        live = [n for n in self.names if n in self.plan["robots"]]
        if not live:
            return
        b.handed_over = True                 # its production work is the others' now (finished() need not wait)
        if not groups:
            return

        def home_idx(vo):
            return next((j for j in range(len(vo) - 1, -1, -1) if isinstance(vo[j], dict) and
                         str(vo[j].get("label", "")) == "home"), len(vo))

        def tail(n):
            entry = self.plan["robots"][n]
            vo = (entry.get("pending") or {}).get("visits") or entry.get("visit_order") or []
            pts = [v["goto"] for v in vo[:home_idx(vo)] if isinstance(v, dict) and "goto" in v]
            p = self.bots[n].pose or (getattr(self.bots[n], "last_cam", None)) or entry["start"]
            return pts[-1][:2] if pts else list(p[:2])
        got = {}
        tails = {n: tail(n) for n in live}
        for g in groups:
            xy = g[0]["goto"]
            n = min(live, key=lambda r: math.hypot(tails[r][0] - xy[0], tails[r][1] - xy[1]))
            got.setdefault(n, []).append(g)
            tails[n] = list(xy[:2])
        recs = self.__dict__.setdefault("reassigned", [])
        for n, gs in got.items():
            items = [x for g in gs for x in g if x is not None]
            self.insert_visits(n, items)
            for g in gs:
                recs.append(dict(point=g[0]["label"], items=[x for x in g if x is not None], frm=b.name, to=n,
                                 t_run=round(now() - self.t_run, 2)))
        # the robot that went out: if it rejoins, it only goes home (its points are the others' now)
        home = next((v for v in reversed((b.out_entry or {}).get("visit_order") or []) if isinstance(v, dict) and
                     str(v.get("label", "")) == "home"), None)
        vis = [home] if home else []
        b.out_entry = {"start": (b.out_entry or {}).get("start") or [0.0, 0.0, 0.0], "steps": [], "visit_order": vis,
                       "task_offset": 0, "finish_s": 0.0,
                       "pending": {"visits": vis, "task_offset": 0, "park": False, "lead_board": None,
                                   "lead_phase": None}}
        b.out_ids, b.out_stat = [], []
        words = "; ".join("%s to %s" % (", ".join(g[0]["label"].replace("point ", "") for g in gs), n)
                          for n, gs in got.items())
        self.log("points_reassigned", robot=b.name, why=why, to={n: [g[0]["label"] for g in gs] for n, gs in got.items()})
        self.carry_on("points_reassigned", b.name, "%s is %s (%s): its points not done go to the robots still "
                      "running: %s (points %s)" % (b.name, "held" if self.rejoinable(b) else "out", why,
                                                   ", ".join(sorted(got)), words))

    def dedupe_on(self):
        """--dedupe-points on (default, 3 Nov): a point is done once; a handed-over point is never listed twice."""
        return getattr(self.args, "dedupe_points", "on") != "off"

    def listed_labels(self, n):
        """The production point labels in robot n's plan (its visit order and its pending work)."""
        entry = self.plan["robots"].get(n) or {}
        return {str(v.get("label")) for v in (entry.get("visit_order") or []) + list((entry.get("pending") or {})
                                                                                   .get("visits") or [])
                if isinstance(v, dict) and str(v.get("label", "")).startswith("point ")}

    def insert_visits(self, n, items):
        """Production items into robot n's list, before its trip home (a finished robot gets them as pending work)."""
        import copy
        entry = self.plan["robots"][n]
        items = copy.deepcopy(items)
        if self.dedupe_on():
            # a point is done once (3 Nov): a point already done, or already in this robot's list, is not added again
            # (its "at point" stand goes with it)
            done = set(self.__dict__.get("done_labels", set()))
            have = self.listed_labels(n)
            drop = {str(v.get("label")) for v in items if isinstance(v, dict) and
                    str(v.get("label", "")).startswith("point ") and
                    (str(v.get("label")) in have or ("at " + str(v.get("label"))) in done)}
            if drop:
                self.log("duplicate_points_dropped", robot=n, points=sorted(drop))
            items = [v for v in items if not (isinstance(v, dict) and (str(v.get("label")) in drop or
                                                                       str(v.get("label", ""))[3:] in drop))]
            if not items:
                return
        b = self.bots[n]
        finished = bool(b.stat) and all(st["status"] == "done" for st in b.stat) or not entry.get("steps")
        vo = entry.setdefault("visit_order", [])
        if finished and not (entry.get("pending") or {}).get("visits"):
            home = next((v for v in reversed(vo) if isinstance(v, dict) and str(v.get("label", "")) == "home"), None)
            off = int(entry.get("task_offset") or 0) + len(vo)
            entry["pending"] = {"visits": items + ([copy.deepcopy(home)] if home else []), "task_offset": off,
                                "park": False, "lead_board": None, "lead_phase": None}
            return
        for lst in ([vo] + ([entry["pending"]["visits"]] if (entry.get("pending") or {}).get("visits") else [])):
            hi = next((j for j in range(len(lst) - 1, -1, -1) if isinstance(lst[j], dict) and
                       str(lst[j].get("label", "")) == "home"), len(lst))
            k, _ = self.current_task(n) if lst is vo else (None, None)
            if k is not None and hi < k:
                hi = len(lst)
            lst[hi:hi] = copy.deepcopy(items)

    def keep_reassigned(self):
        """After a new plan: a reassigned point that is not done and not in its new robot's list any more (a re-plan
        that was being computed when it was handed over) goes back in, and the plan is made again."""
        missing = False
        done = set(self.__dict__.get("done_labels", set()))
        skipped = {x["point"] for x in self.__dict__.get("skipped", [])}
        recs = list(self.__dict__.get("reassigned", []))
        if self.dedupe_on():
            # duplicated points (3 Nov, robot-gap report): a point handed back and forth left an old record ("P05 to
            # Bravo") that put it back in Bravo's list after it had gone to Alpha: P05-P10 three times. Only the newest
            # hand-over of each point counts, and a point already in any running robot's list is not put in again
            last = {}
            for r in recs:
                last[r["point"]] = r
            recs = list(last.values())
        for r in recs:
            n, pt = r["to"], r["point"]
            if n not in self.names or n not in self.plan["robots"] or ("at " + pt) in done or pt in skipped:
                continue
            if self.dedupe_on() and any(pt in self.listed_labels(m) for m in self.names if m in self.plan["robots"]):
                continue
            entry = self.plan["robots"][n]
            labs = {str(v.get("label")) for v in (entry.get("visit_order") or []) + list((entry.get("pending") or {})
                                                                                        .get("visits") or [])
                    if isinstance(v, dict)}
            if pt not in labs and ("at " + pt) not in labs:
                self.insert_visits(n, r["items"])
                missing = True
        if missing:
            self.request_replan("reassigned points back into the plan")

    def on_failed(self, b, i, info, pose, front):
        if getattr(self, "straightening", False):
            return                  # the start's correction: a failed step is that correction failing (straighten)
        if b.out or b.name not in getattr(self, "names", [b.name]):
            # a held robot's late step report (2026-11-02 dry run, a tag covered: the robot was held after 41 cm blind,
            # then its halted step came back "failed" and the re-plan, which no longer has it, stopped everyone with
            # "re-plan failed ('Alpha')"): it is already a fixed obstacle; its rejoin re-plans for it
            self.log("step_failed_while_held", robot=b.name, i=i, info=info)
            return
        stp = self.steps(b.name)[i]
        self.log("step_failed", robot=b.name, i=i, info=info)
        if b.ended or info.startswith(("robot lifted", "stop command", "cancel button")):
            return                                      # the run ended: handled in the main loop
        if info.startswith("halted"):
            return                                      # we stopped it for a re-plan
        import observe as OB
        tk = OB.trip_kind(info)
        sm_mode = getattr(getattr(self, "args", None), "surface_marks", "log")
        if tk == "line" and sm_mode == "log":
            # (e07, Nick 6 Nov) a line-sensor trip with no drop: LOGGED as data only (where, what the camera saw),
            # never an edge and never a behaviour change: the line sensors stay as they are, nothing is mapped, the
            # robot re-plans from where it stands (FailMemory keeps the same move from being repeated unchanged).
            # Only a real drop (the distance sensor's bottom zone) is an edge. e06's way: --surface-marks act
            ct = getattr(b, "cam_t", None)
            cam = getattr(b, "last_cam", None)
            f = self.m["floor"] if cam else None
            ced = None if not cam else min(cam[0] - f["x_min"], f["x_max"] - cam[0], cam[1] - f["y_min"],
                                           f["y_max"] - cam[1])
            self.log("surface_mark", robot=b.name, i=i, info=info, cam=cam,
                     cam_edge_cm=None if ced is None else round(ced, 1),
                     cam_age_s=None if ct is None else round(now() - ct, 2), acted=False)
            self.__dict__.setdefault("surface_marks", []).append(dict(robot=b.name, at=cam, info=info, acted=False))
            self.say("%s: its line sensors tripped with no drop below (%s): logged as a surface mark, not an edge; "
                     "nothing changed on the robot; re-planning from where it stands" % (
                         b.name, "camera %.0f cm inside the table" % ced if ced is not None else "camera not fresh"))
            self.handle_obstacle(None, cause=b.name)
            return
        if tk == "line" and sm_mode in ("act", "auto"):
            # (11-05h, Observe then trust) the line sensors alone: with the camera putting the robot well inside the
            # table it is a mark on the surface (school 5 Nov: the black tape over the seam), not an edge. Logged;
            # that robot's line cue is switched off for the rest of the run (the bottom zone stays on), re-planned
            ct = getattr(b, "cam_t", None)
            cam = getattr(b, "last_cam", None)
            f = self.m["floor"] if cam else None
            ced = None if not cam else min(cam[0] - f["x_min"], f["x_max"] - cam[0], cam[1] - f["y_min"],
                                           f["y_max"] - cam[1])
            kind = OB.edge_trip_kind(True, False, ced, None if ct is None else now() - ct)
            if kind == "surface_mark":
                self.log("surface_mark", robot=b.name, i=i, info=info, cam=cam, cam_edge_cm=round(ced, 1))
                self.__dict__.setdefault("surface_marks", []).append(dict(robot=b.name, at=cam, info=info))
                self.send(b.name, "set safe_edge 2")
                self.say("%s: its line sensors saw a dark line %.0f cm inside the table (the camera): a SURFACE MARK, "
                         "not an edge (logged); its line cue is off for this run, the bottom zone's drop check stays "
                         "on; re-planning" % (b.name, ced))
                self.handle_obstacle(None, cause=b.name)
                return
        if tk in ("tilt", "drop"):
            # (11-05h, Nick 20:46) its tilt stop or its bottom zone's drop: a discovered edge, kept out; it backs
            # straight away (never forward) before anything is re-planned for it
            self.trip_back_off(b, i, pose, info, tk)
            return
        if info.startswith("plywood edge"):
            # the edge guard (follow mode) refused to drive off the board: not a failure. The robot holds;
            # the edge becomes a no-go strip in the map and the affected robot(s) re-plan around it
            self.n_sensor_obstacles += 1
            ob = self.edge_boundary(b, pose, info)
            self.log("edge_boundary", robot=b.name, i=i, info=info, obstacle=ob)
            self.say("%s: limited at a plywood edge during %s step (%s); holding, mapped the edge as a no-go strip, "
                     "re-planning around it" % (b.name, stp["type"], info))
            self.handle_obstacle(ob, cause=b.name)
            return
        if info.startswith("virtual border"):
            # its own four-sided border (firmware 2026-10-27a) stopped it near an open side: its pose and the plan
            # disagree by about 5 cm. Re-plan from where it is (the plan's margins bring it back in)
            self.n_border_stops = getattr(self, "n_border_stops", 0) + 1
            self.log("border_stop", robot=b.name, i=i, info=info)
            self.say("%s: its own border stopped it near an open side (%s); re-planning from where it is" % (b.name, info))
            self.handle_obstacle(None, cause=b.name)
            return
        if info.startswith("obstacle"):
            self.n_sensor_obstacles += 1
            if stp.get("phase") == "probe":
                # a side feel found a barrier: its own pose stands for OWN_POSE_WINDOW_S (Nick, Q17: about 10 s)
                b.own_until = now() + OWN_POSE_WINDOW_S
            ob = self.sensor_obstacle(b, pose, front, info)
            self.sensor_agreement(b, ob, info)
            if ob is not None:
                ob["moving"] = self.was_moving(b, i, pose)
            self.mark_unconfirmed(b, ob)
            if ob is None:
                self.say("%s: its distance sensor saw another robot ahead (%s); re-planning" % (b.name, info))
                self.handle_obstacle(None, cause=b.name)
            else:
                self.say("%s: %s during %s step; re-planning around it" % (b.name, info, stp["type"]))
                self.handle_obstacle(ob, cause=b.name)
            return
        self.say("%s: step %d (%s) failed: %s; re-planning from where it is" % (b.name, i, stp["type"], info))
        self.handle_obstacle(None, cause=b.name)

    def trip_back_off(self, b, i, pose, info, kind):
        """(11-05h) After a tilt or drop trip: the edge strip at its nose (a no-go area for everyone), a straight
        BACKWARD drive of observe.BACK_OFF_CM (follow step D rev 1: firmware 2026-11-05d runs it while tilted and
        refuses anything else), then, on its report (or 8 s), the re-plan around the strip."""
        import observe as OB
        x, y, th = (pose[0], pose[1], pose[2]) if pose else (b.pose or (0, 0, 0))
        self.n_sensor_obstacles += 1
        cam_, ct_ = getattr(b, "last_cam", None), getattr(b, "cam_t", None)
        if cam_ is not None and cam_[2] is not None and ct_ is not None and now() - ct_ <= 1.0:
            # (11-05h, rehearsal s2: the robot's own pose was 13 cm and 105 deg off) the discovered edge is placed where
            # the CAMERA has the robot, when it has it fresh; the robot's own pose is the fallback
            ex_, ey_, eth_ = cam_[0], cam_[1], cam_[2]
        else:
            ex_, ey_, eth_ = x, y, th
        ob = self.edge_boundary(b, (ex_, ey_, eth_), "plywood edge at %.2f %.2f %.2f %s" % (
            ex_, ey_, eth_, "bottom" if kind == "drop" else "line"))
        f0 = self.m["floor"]
        if min(ex_ - f0["x_min"], f0["x_max"] - ex_, ey_ - f0["y_min"], f0["y_max"] - ey_) > 20.0:
            # well inside the table by the camera: not the table's edge (a hole, a box's edge, a false reading): a
            # small keep-out where the nose was, not an 80 cm strip across the table
            k_ = OB.trip_keepout(ex_, ey_, eth_)
            ob = {"id": ob["id"], "shape": "circle", "center": k_["center"], "radius": k_["radius"] + 3.0,
                  "source": "edge_guard", "time": ob.get("time"), "cue": kind}
        ob["cue"] = kind                       # source stays edge_guard (a discovered edge: tilt or drop)
        ob["why"] = "%s trip (discovered edge)" % kind
        bx, by = OB.back_off_target(x, y, th)
        # (11-05h, 4') the back-off is a reverse: only over space shown safe: its own trail (the way it came, no camera
        # needed) or a fresh camera fix with room behind; a tilted robot cannot turn (look before backing), so else it
        # holds
        cam, ct = getattr(b, "last_cam", None), getattr(b, "cam_t", None)
        f = self.m["floor"]
        others = [o.pose[:2] for o in self.bots.values() if o is not b and not o.out and o.pose is not None]
        fresh = cam is not None and ct is not None and now() - ct <= self.blind_after(b)
        how, L_, why_ = OB.back_out_plan(OB.BACK_OFF_CM, trail=getattr(b, "trail", None), bubble_cm=1.0,
                                         cam=(cam[0], cam[1], cam[2] if cam[2] is not None else th) if fresh else None,
                                         cam_age_s=0.0 if fresh else None,
                                         rect=(f["x_min"], f["y_min"], f["x_max"], f["y_max"]), others=others,
                                         turn_clear=False, tilted=True)
        ok_ = how in ("reverse_trail", "reverse_camera")
        if ok_ and L_ < OB.BACK_OFF_CM:
            bx, by = OB.back_off_target(x, y, th, L_)
        L_said = L_ if ok_ else OB.BACK_OFF_CM
        tb_ = self.track_back(b, x, y, th, others) if __import__("safety_group").flag("MIRLT_EDGE_BACKOUT") else None
        if tb_ is not None:
            # (e16, item 7) back along its own track as far as the edge rule needs (derived), not a fixed 10 cm
            bx, by = tb_["end"]
            ok_, why_, L_said = True, tb_["words"], tb_["L"]
        if not ok_:
            self.log("trip", robot=b.name, i=i, kind=kind, info=info, obstacle=ob, back_off="refused: " + why_)
            self.say("%s: its %s stopped it (%s): kept out; backing away is NOT safe (%s): it is held where it stands"
                     % (b.name, "tilt stop" if kind == "tilt" else "edge guard", info, why_))
            self.add_obstacle(ob)
            if self.holds_one():
                self.retire(b, "%s trip, no safe way back" % kind, kind="restarted")
            else:
                self.abort = "%s: %s trip and no safe way back: stopping everyone" % (b.name, kind)
                self.halt_all()
            return
        sid = b.next_id
        b.next_id += 1
        text = "fs %d D %.2f %.2f %.2f %.2f %.2f %.2f 1 -" % (sid, x, y, bx, by, th, 5.0)
        self.send(b.name, text)
        b.backoff = dict(id=sid, t=now(), ob=ob, kind=kind)
        self.__dict__.setdefault("trips", []).append(dict(robot=b.name, kind=kind, at=[round(x, 1), round(y, 1),
                                                                                       round(th, 1)], info=info))
        self.log("trip", robot=b.name, i=i, kind=kind, info=info, obstacle=ob, back_off=text, evidence=why_)
        self.say("%s: its %s stopped it (%s): that spot is kept out (a discovered edge); it backs straight away %.0f "
                 "cm, never forward, then the robots re-plan" % (
                     b.name, "tilt stop" if kind == "tilt" else "edge guard's drop check", info, L_said))

    def track_back(self, b, x, y, th, others):
        """(e16, item 7) The reverse along b's own recorded track to its first point that meets the derived edge rule
        (half diagonal + its uncertainty and own-vs-camera doubt + the stop at 5 cm/s + the tracking's edge bias), as a
        straight chord on the track (observe.track_back_target), clear of the other robots by two body circles plus
        the camera's sigma. None: no track, or blocked (the caller's fixed back-off then)."""
        import observe as OB
        import safety_rules as SR
        tr_ = getattr(b, "trail", None)
        if tr_ is None or len(tr_.pts) < 2:
            return None
        f = self.m["floor"]
        rect = (f["x_min"], f["y_min"], f["x_max"], f["y_max"])
        trk = getattr(self.args, "edge_tracking", "lights") or "lights"
        cam = getattr(b, "last_cam", None)
        dis = math.hypot(x - cam[0], y - cam[1]) if cam is not None else 0.0
        unc = (b.fp.unc_cm(now()) if self.fp_live(b) else SR.CAM_SIGMA_CM) + dis
        need = SR.clearance_cm(unc, 5.0, trk, stops_here=True)
        tgt = OB.track_back_target(tr_.pts, (x, y), th, lambda q: SR.edge_dist(rect, q[0], q[1]) >= need)
        if tgt is None or tgt["L"] < 1.0:
            return None
        gap = 2 * SR.HALF_DIAG_CM + SR.CAM_SIGMA_CM
        for k in range(11):
            c = (x + (tgt["end"][0] - x) * k / 10.0, y + (tgt["end"][1] - y) * k / 10.0)
            if any(math.dist(c, o) < gap for o in others):
                return None
        self.log("track_back", robot=b.name, L=tgt["L"], need=round(need, 2), cleared=tgt["cleared"])
        return tgt

    def backoff_report(self, b, sid, state, info):
        bo = getattr(b, "backoff", None)
        if bo and sid in (bo.get("id"), bo.get("turn_id")) and state not in ("done", "failed"):
            return True                        # (11-05h) its own back-off / recovery step started or queued: not "old"
        if bo and sid == bo.get("turn_id"):
            if state == "failed" and bo.get("kind") == "recover" and info.startswith("halted"):
                # (11-05h) the run itself halted it (everyone holds while another robot is checked): not a failed
                # recovery; it may recover again once the run goes on
                b.backoff = None
                if getattr(b, "blind", None):
                    b.blind["recovered"] = False
                self.log("recover_interrupted", robot=b.name, info=info)
                if bo.get("reserved"):
                    self.release_reserved(b, bo)
                return True
            if state == "failed":
                return self.backoff_report(b, bo["id"], "failed", "the turn: " + info)
            return True                        # the recovery's turn: its drive follows
        if not bo or sid != bo["id"] or state not in ("done", "failed"):
            return False
        b.backoff = None
        self.log("trip_back_off", robot=b.name, result=state, info=info)
        if bo["kind"] == "recover" and state == "failed" and (info or "").startswith("halted"):
            if getattr(b, "blind", None):
                b.blind["recovered"] = False          # halted by the run itself: it may recover again
            self.log("recover_interrupted", robot=b.name, info=info)
            if bo.get("reserved"):
                self.release_reserved(b, bo)
            return True
        if bo["kind"] == "recover" and bo.get("reserved"):
            # (e07) its reserved corridor comes off the map; seen again: it rejoins (the others re-plan with it);
            # not seen: it stays held where it ended, found again by the usual re-check
            self.m["obstacles"] = [o for o in self.m.get("obstacles", []) if o.get("id") != bo["corridor"]["id"]]
            self.m = FM.normalize(self.m)
            seen = state == "done" and getattr(b, "cam_t", None) is not None and now() - b.cam_t <= 1.5
            self.log("recover_end", robot=b.name, result=state, info=info, seen=seen, reserved=True)
            b.out_kind = "restarted"
            if seen and b.out:
                self.rejoin(b, "seen again after its straight recovery")
            elif b.out:
                b.back_t = now()
                self.m["obstacles"] = [o for o in self.m.get("obstacles", []) if o.get("id") != "out-%s" % b.name]
                self.add_obstacle(self.out_obstacle(b))
                self.say("%s: not seen again after its recovery (%s %s): it is held where it stands; the others carry "
                         "on" % (b.name, state, info))
                self.handle_obstacle(None)
            return True
        if bo["kind"] == "recover":
            self.log("recover_end", robot=b.name, result=state, info=info,
                     seen=getattr(b, "cam_t", None) is not None and now() - b.cam_t <= self.blind_after(b))
            if state == "done" and getattr(b, "cam_t", None) is not None and now() - b.cam_t <= 1.5:
                self.say("%s: seen again after its straight recovery: re-planning from where everyone is" % b.name)
                self.handle_obstacle(None, cause=b.name)
            else:
                self.say("%s: not seen again after its recovery (%s %s): it is held where it stands; the others carry "
                         "on" % (b.name, state, info))
                self.retire(b, "lost tag, not seen after its recovery", kind="restarted")
            return True
        if state != "done":
            self.say("%s could not back away (%s): it is held where it stands; the others carry on" % (b.name, info))
            self.add_obstacle(bo["ob"])
            if self.holds_one():
                self.retire(b, "%s trip, could not back away" % bo["kind"], kind="restarted")
            else:
                self.abort = "%s tripped its %s stop and could not back away: stopping everyone" % (b.name, bo["kind"])
                self.halt_all()
            return True
        self.handle_obstacle(bo["ob"], cause=b.name)
        return True

    def backoff_watch(self):
        for b in self.bots.values():
            bo = getattr(b, "backoff", None)
            if bo and now() - bo["t"] > (25.0 if bo.get("kind") == "recover" else 8.0):
                self.backoff_report(b, bo["id"], "failed", "no report in 8 s")

    ZONE_DEG = (22.0, 11.0, 0.0, -11.0, -22.0)   # the five front zones' directions (firmware motion.h)

    # Where the open edge is, ahead of the wheel axle, when the guard trips (ASSUMPTIONS until the E0c edge rows
    # and hardware trips confirm them): the line sensors sit 3.5 cm ahead (grid_center_cm) and trip right over
    # the edge; the bottom zone's floor patch is ~7-9 cm ahead of the sensor (4.5 cm ahead), so the edge is
    # somewhere between the front face and ~12 cm: 8 cm is used, erring near.
    EDGE_AHEAD_CM = {"line": 3.5, "bottom": 8.0, "": 3.5}
    EDGE_STRIP_HALF_CM = 40.0   # the strip runs this far either side of the trip point
    EDGE_STRIP_DEPTH_CM = 25.0  # and this far past the edge (off the board: nothing to plan there anyway)

    def edge_boundary(self, b, pose, info=""):
        """The no-go strip for an open plywood edge from a follow step stopped by the edge guard (STEP failed
        "plywood edge at <x> <y> <th> <line|bottom>"): a rectangle starting at the edge, square to the map's floor
        side the robot drove at when that side is near (edge_side), else to the robot's heading, EDGE_STRIP_HALF_CM
        either side, EDGE_STRIP_DEPTH_CM deep. pose: the STEP record's pose
        (used if the info has none)."""
        x, y, thd, cue = pose[0], pose[1], pose[2], ""
        parts = info.split()
        try:
            k = parts.index("at")
            x, y, thd = float(parts[k + 1]), float(parts[k + 2]), float(parts[k + 3])
            cue = parts[k + 4] if len(parts) > k + 4 else ""
        except (ValueError, IndexError):
            pass
        th = math.radians(thd)
        u = (math.cos(th), math.sin(th))
        a = self.EDGE_AHEAD_CM.get(cue, 3.5)
        h, dep = self.EDGE_STRIP_HALF_CM, self.EDGE_STRIP_DEPTH_CM
        ex, ey = x + a * u[0], y + a * u[1]
        # the plywood's outline is known (the map's floor): square the strip to the side the robot was driving at
        # (within 40 deg of square on and 12 cm of where the map has that side), not to the heading, which is
        # rarely exactly square to the edge
        side = self.edge_side(ex, ey, u)
        if side is not None:
            u = side
        n = (-u[1], u[0])
        pts = [(ex + s * h * n[0] + t * u[0], ey + s * h * n[1] + t * u[1]) for s, t in ((-1, 0), (1, 0), (1, dep), (-1, dep))]
        return {"id": "edge-by-%s-%d" % (b.name, self.n_sensor_obstacles), "shape": "polygon",
                "points": [[round(px, 2), round(py, 2)] for px, py in pts], "source": "edge_guard",
                "cue": cue or "?", "time": FM.now_iso()}

    EDGE_SNAP_DEG = 40.0
    EDGE_SNAP_CM = 12.0

    def edge_side(self, ex, ey, u):
        """Outward normal of the map's floor side the robot was driving at, or None."""
        f = self.m["floor"]
        best = None
        for nrm, d in (((-1.0, 0.0), ex - f["x_min"]), ((1.0, 0.0), f["x_max"] - ex),
                       ((0.0, -1.0), ey - f["y_min"]), ((0.0, 1.0), f["y_max"] - ey)):
            c = u[0] * nrm[0] + u[1] * nrm[1]
            if c >= math.cos(math.radians(self.EDGE_SNAP_DEG)) and abs(d) <= self.EDGE_SNAP_CM:
                if best is None or abs(d) < best[0]:
                    best = (abs(d), nrm)
        return best[1] if best else None

    def sensor_obstacle(self, b, pose, front, info=""):
        """The obstacle's outline from the front zones that see something (STEP
        "...; zones L CL C CR R"): the hit points and the same points 6 cm further
        on (it has some depth), 1 cm bigger all round. Points within 8 cm of
        another robot are that robot, not an obstacle. None if nothing is left."""
        import numpy as np
        import geometry as G
        body = FM.body_of(self.m)
        x, y, th = pose[0], pose[1], math.radians(pose[2])
        reads = []
        if "zones" in info:
            try:
                reads = [float(v) for v in info.split("zones", 1)[1].split()[:5]]
            except ValueError:
                reads = []
        if len(reads) != 5:
            reads = [0, 0, front if front and front < 100 else 5.0, 0, 0]
        sx, sy = x + (body.sensor - 0.25) * math.cos(th), y + (body.sensor - 0.25) * math.sin(th)
        pts = []
        for a, d in zip(self.ZONE_DEG, reads):
            if not (0 < d < 40.0):
                continue
            ang = th + math.radians(a)
            px, py = sx + d * math.cos(ang), sy + d * math.sin(ang)
            if any(o is not b and o.pose and math.hypot(o.pose[0] - px, o.pose[1] - py) < 8.0 for o in self.bots.values()):
                continue
            pts.append((px, py))
        if not pts:
            return None
        # one object: the hits near the nearest one (the zones also see things further off)
        d0 = [math.hypot(px - sx, py - sy) for px, py in pts]
        p0 = pts[d0.index(min(d0))]
        pts = [p for p in pts if math.hypot(p[0] - p0[0], p[1] - p0[1]) <= 15.0]
        u = np.array([math.cos(th), math.sin(th)])
        n = np.array([-u[1], u[0]])
        P0 = np.array(pts)
        P0 = np.vstack([P0, P0 + 6.0 * u, P0 + 2.0 * n, P0 - 2.0 * n])     # depth, and the zone's own width
        hull = G.convex_hull(P0)
        c = hull.mean(0)
        hull = hull + 1.0 * (hull - c) / np.maximum(np.linalg.norm(hull - c, axis=1)[:, None], 1e-6)
        return {"id": "seen-by-%s-%d" % (b.name, self.n_sensor_obstacles), "shape": "polygon",
                "points": hull.round(2).tolist(), "source": "distance_sensor", "time": FM.now_iso()}

    def sensor_agreement(self, b, ob, info):
        """Does the camera agree with a distance-sensor stop? Yes when its outline is near something the camera
        already reported (a brick it saw, another robot's spot); else it is the sensor's word alone. Logged and
        counted (the DOE on obs_n 1 / 2 / 3 needs it); the stop is handled the same either way (safety first)."""
        import numpy as np
        by = None
        if ob is None:
            by = "another robot (its spot is where the sensor saw something)"
        else:
            pts = np.array(ob["points"], float)
            for o in self.m.get("obstacles", []):
                if o.get("source") in ("camera", "brickwatch"):
                    c = np.array(o.get("center") or np.mean(np.array(o.get("points") or [[0, 0]]), 0))
                    if float(np.min(np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1]))) < 8.0:
                        by = "the camera's obstacle %s" % o.get("id")
        self.sensor_stops = getattr(self, "sensor_stops", []) + [dict(robot=b.name, info=info, camera=by,
                                                                      t_run=round(now() - self.t_run, 2))]
        self.log("sensor_stop", robot=b.name, info=info, camera_agrees=by)

    def confirm_check(self, b, i, own):
        """The end of a run-in (phase "confirm"): the camera has just watched the robot drive straight, so its
        heading is known. Nick (14:53): the heading error that matters is the sideways drift it causes before the
        next camera correction; anything risky confirms its heading first. Off by more than allowed: its pose is set
        from the camera (fpose) before it goes on."""
        steps = self.steps(b.name)
        if i + 1 < len(steps) and steps[i + 1].get("phase") == "confirm":
            return                                     # the run-in goes on
        cam, ct = getattr(b, "last_cam", None), getattr(b, "cam_t", None)
        if cam is None or cam[2] is None or ct is None or now() - ct > 0.6:
            self.log("confirm_check", robot=b.name, i=i, result="no camera heading")
            return
        line = sum(math.dist(st["from"][:2], st["to"][:2]) for st in steps[max(0, i - 3):i + 1]
                   if st.get("phase") == "confirm" and st["type"] == "drive")
        nxt = next((st for st in steps[i + 1:] if st["type"] in ("drive", "arc")), None)
        L = math.dist(nxt["from"][:2], nxt["to"][:2]) if nxt else 0.0
        drift = EC.RISKY_DRIFT_CM
        allowed = EC.allowed_heading_deg(L, drift, line)
        err = abs((cam[2] - own[2] + 180.0) % 360.0 - 180.0)
        ok = err <= allowed
        self.log("confirm_check", robot=b.name, i=i, cam=cam, own=list(own), err_deg=round(err, 2),
                 allowed_deg=round(allowed, 2), line_cm=round(line, 1), next_cm=round(L, 1), ok=ok)
        self.confirms = getattr(self, "confirms", []) + [dict(robot=b.name, err_deg=round(err, 2),
                                                              allowed_deg=round(allowed, 2), ok=ok)]
        if not ok:
            self.say("%s: its heading is %.1f deg off the camera's after the run-in (allowed %.1f for the next %.0f cm):"
                     " pose set from the camera" % (b.name, err, allowed, L))
            self.send(b.name, "fpose %.2f %.2f %.2f %.2f %.2f" % (cam[0], cam[1], cam[2], 0.5,
                                                                 max(1.0, math.degrees(math.atan2(1.0, max(line, 1.0))))))

    # -- the pause after a turn (Nick, 2026-10-28 21:27 / 2026-10-29 07:3x): after a turn on the spot the robot stands
    # until a fresh camera fix (both cameras' if they both see it) agrees with its own pose, then drives. The camera's
    # correction is taken while it stands still, not after a drift (the camera is 0.18-0.27 s behind at most; the worst
    # real moments, 2-3 cm p95, were at the turn-arounds).
    TURN_PAUSE_MIN_DEG = 20.0   # chosen: turns smaller than this are heading trims (the planner's line-ups); no pause.
                                # DOE note: not worth one (Monday's turns are 70-180 deg; the trims under 5 deg)
    TURN_PAUSE_TOL_CM = 1.5     # chosen: the camera's spot (the mean of the newest fixes, up to 3) and the robot's own
                                # pose within this. A robot standing still wobbles 0.5-1.5 cm in the camera (28 Oct);
                                # the line check's gate is 3 cm. DOE note: worth one: 1.0 / 1.5 / 2.5 cm against the
                                # pause's length and the line check's p95 on the leg after each turn (board, Left and
                                # Right, 10 lane turns each)
    TURN_PAUSE_ONE_CAM_S = 1.0  # chosen: with no fix from both cameras this long after the turn, one camera's fix is
                                # taken (the other cannot see it: the side camera's far end, a washed-out overhead).
                                # DOE note: worth one with the timeout: 0.5 / 1 / 2 s against how often a two-camera
                                # fix would still have come (bridge_track.csv src per picture after each turn)
    TURN_PAUSE_MAX_S = 3.0      # chosen: then it drives anyway, as before this pause existed (logged; the camera guard
                                # still stops everyone on a 6 cm disagreement). The camera is 0.18-0.27 s behind and
                                # the robot takes a fix every 0.1 s, so an agreeing pose comes in 0.5-1 s. DOE note:
                                # worth one with ONE_CAM_S: 2 / 3 / 5 s against the timeouts' share and their cause
    TURN_BOTH_LATELY_S = 2.0    # chosen: "the other camera cannot see it" = no two-camera fix in this long before the turn
                                # ended (the robot was on one camera already). No DOE beyond ONE_CAM_S's (the same
                                # bridge_track.csv count answers both)
    TURN_POSE_AFTER_S = 0.2    # the robot's own pose must be reported this long after the fix arrived (it took the
                                # fix: fixes go out every 0.1 s, FOLLOW records every 0.1 s). No DOE: plumbing

    def turn_before(self, n, i):
        """The turn on the spot that step i of robot n drives after (only waits between), or None."""
        steps = self.steps(n)
        if steps[i]["type"] not in ("drive", "arc", "approach"):
            return None
        j = i - 1
        while j >= 0 and steps[j]["type"] == "wait":
            j -= 1
        if j < 0 or steps[j]["type"] != "turn":
            return None
        a = (steps[j]["to"][2] - steps[j]["from"][2] + 180.0) % 360.0 - 180.0
        if abs(a) < self.TURN_PAUSE_MIN_DEG:
            return None
        if self.chicken_keep_going(steps, i):
            return None
        return j

    # Chicken's swerve and pass are not paused (Nick, 2026-10-29): the robot swerves and keeps going past the other.
    # The goal a step belongs to is its "task"; the planner puts the goal's label on the goal's last step only, so the
    # drive straight after the jog's turn may carry no label of its own.
    CHICKEN_KEEP_GOING = re.compile(r"^chicken \d+ (sidestep|pass)$")

    def chicken_keep_going(self, steps, i):
        """Is step i part of Chicken's jog (sidestep) or its turn back and pass?"""
        task = steps[i].get("task")
        same = [s for s in steps if task is not None and s.get("task") == task] or [steps[i]]
        return any(self.CHICKEN_KEEP_GOING.match(s.get("label") or "") for s in same)

    def turn_pause_open(self, b, i, t):
        """May step i (a drive after a turn) be sent? Logs each pause (turn_pause) and adds it to the summary."""
        if not getattr(self.args, "turn_pause", False) or not self.source.live or b.out:
            return True
        j = self.turn_before(b.name, i)
        if j is None:
            return True
        tj = b.stat[j]
        if tj["status"] != "done":
            return False                           # the turn first (the drive is not queued behind it)
        t_done = tj.get("t_done") or t
        key = (self.plan_no, i)
        pz = getattr(b, "tpause", None)
        if pz is None or pz["key"] != key:
            pz = b.tpause = dict(key=key, t_block=t, t_done=t_done, waited=False)
        allh = getattr(b, "cam_hist", [])
        # both cameras saw it in the 2 s before the turn ended: wait up to ONE_CAM_S for a two-camera fix. If only one
        # camera has been seeing it (a washed-out overhead, the side camera's blind end), take that camera's fix at
        # once: waiting for the other only cost 1 s at every turn (washed-out dry run, 29 Oct: 13-15 s per robot)
        both_lately = any(f[4] == "both" and t_done - self.TURN_BOTH_LATELY_S <= f[0] for f in allh)
        hist = [f for f in allh if f[0] >= t_done]
        # only fixes the robot had time to take before its newest own pose (fixes keep coming every 0.1 s)
        pt = b.pose_t if b.pose is not None else -1e9
        hist = [f for f in hist if f[1] + self.TURN_POSE_AFTER_S <= pt]
        both = [f for f in hist if f[4] == "both"]
        how, use = None, []
        if both:
            how, use = "both", both
        elif hist and (t - t_done >= self.TURN_PAUSE_ONE_CAM_S or not both_lately):
            how, use = "one camera (%s)" % hist[-1][4], hist
        d = None
        if use:
            use = use[-3:]
            cx, cy = sum(f[2] for f in use) / len(use), sum(f[3] for f in use) / len(use)
            d = math.hypot(cx - b.pose[0], cy - b.pose[1])
        ok = d is not None and d <= self.TURN_PAUSE_TOL_CM
        timed_out = t - t_done >= self.TURN_PAUSE_MAX_S
        if not ok and not timed_out:
            pz["waited"] = True
            return False
        extra = t - pz["t_block"]          # the time this pause added (the drive was otherwise ready to go)
        rec = dict(robot=b.name, i=i, plan=self.plan_no, label=self.steps(b.name)[i].get("label", ""),
                   since_turn_s=round(t - t_done, 3), added_s=round(extra, 3), how=how or "no fix",
                   d_cm=None if d is None else round(d, 2), fixes=len(hist), result="agreed" if ok else "timed out",
                   t_run=round(t - self.t_run, 2) if self.t_run else None)
        self.log("turn_pause", **rec)
        tp = self.__dict__.setdefault("turn_pauses", [])
        tp.append(rec)
        if not ok:
            self.say("%s: after its turn the camera did not agree within %.1f cm in %.0f s (%s, %s): driving on, as "
                     "before the pause" % (b.name, self.TURN_PAUSE_TOL_CM, self.TURN_PAUSE_MAX_S, rec["how"],
                                           "no fix" if d is None else "%.1f cm apart" % d))
        b.tpause = None
        return True

    def turn_pause_summary(self):
        out = {}
        for r in getattr(self, "turn_pauses", []):
            o = out.setdefault(r["robot"], dict(pauses=0, added_s=0.0, both=0, one_camera=0, timed_out=0,
                                                longest_s=0.0))
            o["pauses"] += 1
            o["added_s"] = round(o["added_s"] + r["added_s"], 2)
            o["longest_s"] = max(o["longest_s"], r["added_s"])
            if r["result"] == "timed out":
                o["timed_out"] += 1
            elif r["how"] == "both":
                o["both"] += 1
            else:
                o["one_camera"] += 1
        return out

    # -- mirroring (Nick, 2026-10-29 20:06-20:08): Left and Right mirror each other across the table's middle line ------
    # Both follow a shared timeline of mirrored points (missions.build gives each mirrored goto a key its partner's
    # mirror image shares: plan steps "mirror" = the point, "mirror_leg" = the steps driving to it). When they drift out
    # of step (one corrected, paused or slowed), each drive towards the next matching point is sent at a speed that
    # brings both there together: the target time comes from each robot's own time to go and its partner's time to
    # go to the matching spot; the late one speeds up and the leader slows down, within the mission's
    # mirror_min_cms-mirror_max_cms (Nick: 5-10 cm/s). If speed alone cannot close the gap, the leader waits at the
    # point where it stands before its next leg. Every matching point is logged (time and position mirror error).
    MIRROR_TURN_PAUSE_EST_S = 0.5   # chosen: the turn pause's expected length per turn of 20 deg or more, in the time to
                                    # go (simulator 0.25-1.0 s, turn-pause report). DOE note: not worth one; the time to
                                    # go is re-measured at every step, so an error here is corrected one step later
    MIRROR_HOLD_TOL_S = 0.1         # chosen: the leader waits until even its slowest drive would bring it no more than
                                    # this early. 0.25 left the first lane after the circle 0.53 s apart (61 x 122 dry
                                    # run, 29 Oct: the wait ends early, then the time-to-go estimates' own error adds
                                    # to it). DOE note: 0.1 / 0.25 / 0.5 s in the simulator against the lane-end time
                                    # error and the number of waits
    MIRROR_SAY_S = 1.0              # the event log names a re-timing when the two are more than this apart (smaller ones
                                    # are logged only). No DOE: wording

    def mirror_cfg(self):
        mc = ((self.m.get("mission") or {}).get("mirror") or {})
        if len(mc.get("robots") or []) != 2:
            return None
        return mc

    def mirror_partner(self, n):
        mc = self.mirror_cfg()
        if not mc or n not in mc["robots"]:
            return None
        p = [x for x in mc["robots"] if x != n][0]
        if p not in self.names or self.bots[p].out or self.bots[n].out:
            return None
        return p

    def mirror_xy(self, x, y):
        mc = self.mirror_cfg()
        a = float(mc["at_cm"])
        return (2 * a - x, y) if mc.get("axis", "x") == "x" else (x, 2 * a - y)

    def _m_adjustable(self, stp):
        return (stp["type"] == "drive" and stp.get("mirror_leg") and stp.get("phase") != "confirm" and
                not stp.get("reverse") and stp.get("speed"))

    def _m_dur(self, stp, v=None):
        """A step's planned time (s) at drive speed v (its own speed if None); waits the planner put in for the other
        robot count 0 (the "after" order makes them), a wait for a mission time counts what is left of it."""
        d = float(stp["t_end"] - stp["t_start"])
        if stp["type"] == "wait":
            if stp.get("phase") in ("dwell", "pause"):
                if stp.get("until_s") is not None and self.plan_start is not None:
                    return max(0.0, self.plan_start + stp["t_end"] - now())
                return d
            return 0.0
        if stp["type"] == "drive" and v and stp.get("speed"):
            mv = float(stp.get("move_s") or 0.0)
            d = d - mv + mv * float(stp["speed"]) / float(v)
        if stp["type"] == "turn" and getattr(self.args, "turn_pause", False) and self.source.live and \
                abs(stp.get("turn_deg") or 0.0) >= self.TURN_PAUSE_MIN_DEG:
            d += self.MIRROR_TURN_PAUSE_EST_S
        return d

    def _m_left(self, b, i, stp, st, t):
        """What is left of a step that was sent: from the robot's own pose when it has started (a drive: the distance
        to its end at the speed it was sent; a turn: the angle left), else its whole time at the sent speed."""
        v = st.get("speed_sent") or stp.get("speed")
        full = self._m_dur(stp, v)
        if st["status"] != "started" or b.pose is None:
            return full
        if stp["type"] == "drive" and v:
            rem = math.dist(b.pose[:2], stp["to"][:2])
            return rem / float(v) + 0.2
        if stp["type"] == "turn":
            a = abs((stp["to"][2] - b.pose[2] + 180.0) % 360.0 - 180.0)
            return a / float(self.plan["settings"].get("turn_dps", 60.0)) + 0.3
        return max(0.0, full - (t - (st.get("t_started") or t)))

    def _m_key_index(self, n, key):
        for j, stp in enumerate(self.steps(n)):
            if stp.get("mirror") == key:
                return j
        return None

    def mirror_eta(self, n, iK, t, i_new=None):
        """(fixed s, adjustable drive cm, arrived at) for robot n to reach its step iK: time that no speed changes (steps
        sent, turns, settles) and the drive length still to be sent (from step i_new on: the one being sent now).
        arrived: the time it finished iK if it already has."""
        b = self.bots[n]
        steps = self.steps(n)
        if b.stat[iK]["status"] == "done":
            return 0.0, 0.0, b.stat[iK].get("t_done") or t
        fixed, D = 0.0, 0.0
        for j in range(0, iK + 1):
            st, stp = b.stat[j], steps[j]
            if st["status"] == "done":
                continue
            if st["status"] in ACTIVE:
                fixed += self._m_left(b, j, stp, st, t)
            elif self._m_adjustable(stp):
                L = math.dist(stp["from"][:2], stp["to"][:2])
                D += L
                fixed += self._m_dur(stp) - float(stp.get("move_s") or 0.0)
            else:
                fixed += self._m_dur(stp)
        return fixed, D, None

    def mirror_gate(self, b, i, t):
        """May step i of robot b go now, and at what speed (None: as planned)? (ok, speed)."""
        stp = self.steps(b.name)[i]
        if not stp.get("mirror_leg") or getattr(self, "straightening", False) or getattr(self.args, "no_mirror", False):
            return True, None
        p = self.mirror_partner(b.name)
        if p is None:
            return True, None
        mc = self.mirror_cfg()
        steps = self.steps(b.name)
        iK = next((j for j in range(i, len(steps)) if steps[j].get("mirror")), None)
        if iK is None:
            return True, None
        key = steps[iK]["mirror"]
        jK = self._m_key_index(p, key)
        if jK is None:
            return True, None
        vmin, vmax = float(mc.get("min_cms", 5.0)), float(mc.get("max_cms", 10.0))
        vnom = float(mc.get("cruise_cms", 8.0))
        fn, Dn, _ = self.mirror_eta(b.name, iK, t)
        fp, Dp, arr_p = self.mirror_eta(p, jK, t)

        def rng(f, D):
            return (t + f + D / vmax, t + f + D / vmin, t + f + D / vnom) if D > 0 else (t + f,) * 3
        lo_n, hi_n, nom_n = rng(fn, Dn)
        if arr_p is not None:
            lo_p = hi_p = nom_p = arr_p
        else:
            lo_p, hi_p, nom_p = rng(fp, Dp)
        lo, hi = max(lo_n, lo_p), min(hi_n, hi_p)
        target = min(max((nom_n + nom_p) / 2.0, lo), hi) if lo <= hi else lo
        # hold: even its slowest leaves it early, and it stands at a point (the first step of a leg)
        first_of_leg = i == 0 or steps[i - 1].get("mirror") or steps[i - 1].get("mirror_leg") != stp.get("mirror_leg")
        ms = self.__dict__.setdefault("mirror_state", {})
        hk = (self.plan_no, b.name, key)
        if hi_n < target - self.MIRROR_HOLD_TOL_S and first_of_leg:
            h = ms.setdefault(hk, {"t0": t, "said": False})
            if t - h["t0"] < float(mc.get("wait_max_s", 8.0)):
                if not h["said"]:
                    h["said"] = True
                    self.say("mirroring: %s waits where it is for %s (about %.1f s; even at %.0f cm/s it would reach "
                             "the next matching point early)" % (b.name, p, target - hi_n, vmin))
                    self.log("mirror_hold", robot=b.name, partner=p, key=key, i=i, wait_s=round(target - hi_n, 2))
                return False, None
            if not h.get("gave_up"):
                h["gave_up"] = True
                self.say("%s waited %.0f s for %s at a mirrored point: it carries on (not mirrored until they meet "
                         "again)" % (b.name, t - h["t0"], p))
                self.log("mirror_give_up", robot=b.name, partner=p, key=key, waited_s=round(t - h["t0"], 2))
            return True, None
        h = ms.get(hk)
        if h is not None and "added_s" not in h:
            h["added_s"] = round(t - h["t0"], 2)
            self.__dict__.setdefault("mirror_holds", []).append(dict(robot=b.name, key=key, s=h["added_s"],
                                                                     t_run=round(t - self.t_run, 2)))
        if not self._m_adjustable(stp):
            return True, None
        if Dn <= 0:
            return True, None
        room = target - t - fn
        v = vmax if room <= 0 else min(vmax, max(vmin, Dn / room))
        v = round(v, 2)
        gap = nom_n - nom_p
        rec = dict(robot=b.name, partner=p, i=i, key=key, speed=v, planned=stp["speed"], behind_s=round(gap, 2),
                   target_in_s=round(target - t, 2), partner_in_s=round((arr_p or nom_p) - t, 2))
        self.log("mirror_retime", **rec)
        rt = self.__dict__.setdefault("mirror_retimes", {})
        rt[b.name] = rt.get(b.name, 0) + (1 if abs(v - float(stp["speed"])) > 0.05 else 0)
        last = ms.get(("said", b.name))
        if abs(gap) > self.MIRROR_SAY_S and (last is None or t - last > 5.0):
            ms[("said", b.name)] = t
            self.say("mirroring: at the planned speed %s would reach the next matching point %.1f s %s %s: its next "
                     "leg at %.1f cm/s (planned %.0f)" % (b.name, abs(gap), "after" if gap > 0 else "before", p, v,
                                                          stp["speed"]))
        return True, v

    def mirror_arrived(self, b, i, t):
        """Robot b finished step i: if it is a mirrored point, keep its time and position; once both have it, the time
        and position mirror error (positions from the camera when it has a fresh spot for both, else their own)."""
        stp = self.steps(b.name)[i] if i < len(self.steps(b.name)) else {}
        key = stp.get("mirror")
        mc = self.mirror_cfg()
        if not key or not mc or b.name not in mc["robots"]:
            return
        arr = self.__dict__.setdefault("mirror_arr", {})
        cam = getattr(b, "last_cam", None)
        fresh = cam is not None and getattr(b, "cam_t", None) is not None and t - b.cam_t <= 0.4
        arr.setdefault(key, {})[b.name] = dict(t=t, own=list(b.pose[:2]) if b.pose else list(stp["to"][:2]),
                                               cam=list(cam[:2]) if fresh else None, label=stp.get("label", ""))
        if len(arr[key]) != 2 or arr[key].get("_done"):
            return
        a, c = mc["robots"]
        A, C = arr[key][a], arr[key][c]
        use = "cam" if A["cam"] and C["cam"] else "own"
        mx, my = self.mirror_xy(*C[use])
        perr = math.hypot(A[use][0] - mx, A[use][1] - my)
        terr = abs(A["t"] - C["t"])
        rec = dict(key=key, label=A["label"] or C["label"], time_err_s=round(terr, 3), pos_err_cm=round(perr, 2),
                   pos_from=use, first=a if A["t"] <= C["t"] else c, t_run=round(max(A["t"], C["t"]) - self.t_run, 2),
                   lane_end="mow lane" in (A["label"] or C["label"] or ""),
                   ok=terr <= float(mc.get("tol_s", 0.5)) and perr <= float(mc.get("tol_cm", 3.0)))
        arr[key]["_done"] = True
        self.log("mirror_point", **rec)
        self.__dict__.setdefault("mirror_points", []).append(rec)

    def mirror_summary(self):
        mc = self.mirror_cfg()
        pts = getattr(self, "mirror_points", [])
        if not mc:
            return {"on": False}

        def stats(xs):
            if not xs:
                return None
            xs = sorted(xs)
            return {"max": round(xs[-1], 3), "median": round(statistics.median(xs), 3), "n": len(xs)}
        lane = [q for q in pts if q["lane_end"]]
        return {"on": True, "robots": mc["robots"], "target_s": mc.get("tol_s"), "target_cm": mc.get("tol_cm"),
                "points": pts, "lane_ends": {"time_s": stats([q["time_err_s"] for q in lane]),
                                             "pos_cm": stats([q["pos_err_cm"] for q in lane]),
                                             "within_target": sum(1 for q in lane if q["ok"])},
                "all_points": {"time_s": stats([q["time_err_s"] for q in pts]),
                               "pos_cm": stats([q["pos_err_cm"] for q in pts])},
                "holds": getattr(self, "mirror_holds", []), "retimed_legs": getattr(self, "mirror_retimes", {})}

    def waiting_for(self, n):
        """Why robot n's next step has not been sent (for the status line)."""
        b = self.bots[n]
        if b.controls:
            return "its %s is not acknowledged" % b.controls[0]["text"]
        for i, stp in enumerate(self.steps(n)):
            st = b.stat[i]
            if st["status"] == "done":
                continue
            if st["status"] in ACTIVE:
                return "step %d %s" % (i, st["status"])
            if st["status"] in ("failed", "dropped"):
                return "step %d %s, waiting for a re-plan" % (i, st["status"])
            if self.planning is not None and i >= self.limit.get(n, 0):
                return "a re-plan is being computed"
            deps = [(o, j) for o, j in stp.get("after", []) if self.bots[o].stat[j]["status"] != "done"]
            if deps:
                return "step %d waits for %s" % (i, ", ".join("%s step %d (%s)" % (o, j, self.bots[o].stat[j]["status"])
                                                             for o, j in deps))
            nb = stp.get("not_before_s")
            if nb is not None and now() < self.plan_start + nb:
                return "step %d not before %.1f s" % (i, nb)
            if getattr(b, "tpause", None) and b.tpause["key"] == (self.plan_no, i):
                return "step %d: standing after its turn until the camera agrees" % i
            return "step %d ready" % i
        return "done"

    # -- main loop ------------------------------------------------------------
    # -- the start's correction (straighten.py, Nick 2026-10-29 15:41) ------------------------------------------
    STRAIGHT_TH_DROP_S = 1.5  # after an fpose, the bridge's fixes carry no heading this long (it learns the robot's own
                              # pose from the echo, then sends only headings it measured from travel). No DOE: a guard

    def run_correction(self, plan_st, timeout_s):
        """Drive the start's correction steps (straighten.plan_all) with the camera watching (fixes, the camera guard,
        the turn pause), apart from the mission plan. Returns (ok, words)."""
        main = self.plan
        robots = {}
        for n in self.names:
            r = plan_st["robots"].get(n) or {"steps": []}
            steps = [dict(st) for st in r["steps"]]
            a = plan_st["after"].get(n)
            if isinstance(a, (tuple, list)):          # (robot, its step index): a detour's park move (straighten.py)
                if steps and plan_st["robots"][a[0]]["steps"]:
                    steps[0]["after"] = [(a[0], int(a[1]))]
            elif steps and a and plan_st["robots"][a]["steps"]:
                steps[0]["after"] = [(a, len(plan_st["robots"][a]["steps"]) - 1)]
            for i, o, j in r.get("waits") or []:      # a later step waiting for another robot's step
                if i < len(steps):
                    steps[i]["after"] = list(steps[i].get("after") or []) + [(o, int(j))]
            robots[n] = {"start": list(main["robots"][n]["start"]), "steps": steps}
        self.plan = {"plan_id": "%s-straighten" % main.get("plan_id"), "robots": robots,
                     "summary": {"makespan_s": 0.0}, "settings": main.get("settings") or {}}
        self._install(self.plan, keep={}, first={}, old=None)
        self.straightening = True
        self.plan_start = self.t_run = now()
        self.log("straighten_run", steps={n: [st.get("why") for st in r["steps"]] for n, r in robots.items()})
        ok, words = False, "timed out after %.0f s" % timeout_s
        try:
            while now() - self.t_run < timeout_s:
                self.drain()
                self.fixes()
                moving = [self.bots[n] for n in self.names]      # (a robot held at the start is not in it)
                for b in moving:
                    if b.ended and not self.abort:
                        self.abort = "%s's follow run ended (%s)" % (b.name, b.ended)
                self.cam_guard()
                lim = getattr(self.args, "silent_s", EC.SILENT_S)
                for b in moving:
                    if lim and b.last_heard is not None and now() - b.last_heard > lim and not self.abort:
                        self.abort = "nothing heard from %s for %.1f s" % (b.name, now() - b.last_heard)
                if self.abort:
                    self.halt_all()
                    words = self.abort
                    break
                failed = [(n, st) for n in self.names for st in self.bots[n].stat if st["status"] == "failed"]
                if failed:
                    self.halt_all()
                    words = "; ".join("%s's %s did not finish (%s)" % (n, "step", st.get("info") or "failed")
                                      for n, st in failed)
                    break
                self.release()
                self.resend()
                if all(all(st["status"] == "done" for st in self.bots[n].stat) for n in self.names):
                    ok, words = True, "done in %.1f s" % (now() - self.t_run)
                    break
                time.sleep(0.01)
            else:
                self.halt_all()
        finally:
            self.straightening = False
            self.plan = main
            self._install(main, keep={}, first={}, old=None)
            self.plan_start = self.t_run = None
        self.log("straighten_done", ok=ok, words=words)
        return ok, words

    # -- driving blind (Nick 2026-10-29 15:41, question 3b) -------------------------------------------------------
    # -- (e18, Nick 12 Nov decisions 2, 4, 5, 6) ---------------------------------------------------------------------
    def trust_of(self, b):
        """Its odometry trust record (odo_trust.Trust), loaded from the store the first time (simulated robots on
        127.0.0.x start from the prior: their record is not the real robot's)."""
        tr = getattr(b, "trust", None)
        if tr is None:
            import odo_trust as OT
            sim_ = bool(getattr(self.args, "dry_run", False)) or getattr(self.args, "sim_persist", False)
            tr = (OT.load([b.name]) if not sim_ else {b.name: OT.Trust(b.name)})[b.name]
            tr.async_fit = True         # (e19, audit 12) its refits off the coordinator's loop
            b.trust = tr
            fl = self.__dict__.setdefault("trust_fleet", {})
            fl[b.name] = tr
            OT.link(fl)             # (e18b, the replay) one fleet record, a robot's own once its samples differ
        return tr

    def trust_save(self, why="the run ends"):
        """Every robot's record into the store (not for simulated robots: a dry run's record is the simulator's)."""
        if not (BLIND_ALLOW and SAVE_CAL_AT_END) or getattr(self.args, "dry_run", False):
            return None
        import odo_trust as OT
        trs = {n: b.trust for n, b in self.bots.items() if getattr(b, "trust", None) is not None}
        return OT.save(trs, why=why) if trs else None

    def trust_checkpoint(self):
        if now() - getattr(self, "_trust_cp", 0.0) >= 60.0:
            self._trust_cp = now()
            self.trust_save("checkpoint (a crash or power cut loses at most a minute)")

    def blind_err(self, b, bl, record=False):
        """Its possible error after this blind stretch: the fixed 29 Oct numbers (2 cm + 0.07 cm per cm) for an unplanned
        loss (MIRLT_BLIND_BUBBLE=fixed, the default), its own record with record=True (the check) or
        MIRLT_BLIND_BUBBLE=record."""
        if not BLIND_ALLOW or SG.value("MIRLT_BLIND_BUBBLE") != "record" and not record:
            return self.BLIND_ERR_BASE_CM + self.BLIND_ERR_PER_CM * bl["dist"]
        d = bl["dist"]
        ts, tc = bl.get("ts", 0.0), bl.get("tc", 0.0)
        return self.trust_of(b).radius(d, ts, tc, d * ts - bl.get("ms", 0.0), d * tc - bl.get("mc", 0.0))

    def blind_leg_allowed(self, n, xy, readable):
        """(e18, 5) May robot n visit a point the camera does not read with an out-and-back from the nearest readable
        ground? Its record's predicted error at every point + body + stop inside the table's edges and away from the
        others, ending back where the camera reads. Returns the allowance (dict) or None."""
        import odo_trust as OT  # noqa: F401
        b = self.bots.get(n)
        rm = getattr(self, "readmap", None)
        if b is None or rm is None:
            return None
        f = self.m["floor"]
        rect = (f["x_min"], f["y_min"], f["x_max"], f["y_max"])
        best = None
        for i in range(rm.nx):
            for j in range(rm.ny):
                c = rm.centre(i, j)
                if rm.readable(*c)[0] and (best is None or math.dist(c, xy) < math.dist(best, xy)):
                    best = c
        if best is None:
            return None
        L = math.dist(best, xy)
        n_ = max(2, int(L / 2.0))
        th = math.degrees(math.atan2(xy[1] - best[1], xy[0] - best[0]))
        pts = [(best[0] + (xy[0] - best[0]) * k / n_, best[1] + (xy[1] - best[1]) * k / n_, th) for k in range(n_ + 1)]
        pts += [(best[0] + (xy[0] - best[0]) * k / n_, best[1] + (xy[1] - best[1]) * k / n_, th + 180.0)
                for k in range(n_, -1, -1)]
        others = [(o.pose[0], o.pose[1], 2.0) for o in self.bots.values() if o is not b and o.pose is not None and
                  not o.out]
        res = self.trust_of(b).allow(pts, lambda x, y: min(x - rect[0], rect[2] - x, y - rect[1], rect[3] - y),
                                     lambda x, y: rm.readable(x, y)[0], others, 6.72, 2.4, 2.5)
        res["from"] = [round(best[0], 1), round(best[1], 1)]
        return res

    def held_creep_watch(self):
        """(e18, Nick 12 Nov decision 2: never still in missions too) A held robot that still hears (held after a blind
        stop, a trip with no safe way back, a lost tag, a camera disagreement, at the start) never just stands: after
        the derived pause (its fix interval + 0.5 s) it creeps 2-8 cm (scaled by its doubt) at 2-5 cm/s toward being
        found (where the camera last read it, when no camera reads it now) or toward the board's middle (safety), never
        further than CREEP_REACH_CM from where it was held (its held outline is grown by that, so the others' plans
        stay valid), keeping RECOVER_GAP_CM from every other robot. MIRLT_NEVER_STILL=0: it stands (e16)."""
        if not NEVER_STILL or not self.t_run_started() or getattr(self, "straightening", False):
            return
        t = now()
        f = self.m["floor"]
        rect = (f["x_min"], f["y_min"], f["x_max"], f["y_max"])
        for b in self.bots.values():
            if not b.out or getattr(b, "out_kind", None) != "restarted" or getattr(b, "rechecking", False) or \
                    getattr(b, "backoff", None) or b.pose is None:
                continue
            if b.last_heard is None or t - b.last_heard > 1.0:
                continue                         # it does not hear now: nothing to send
            hold = getattr(b, "hold_at", None)
            if hold is None or getattr(b, "hold_out_t", None) != getattr(b, "out_t", None):
                b.hold_at = hold = tuple(b.pose[:2])
                b.hold_out_t = getattr(b, "out_t", None)
                b.creep_t = t
            pause = self.blind_after(b) + 0.5
            if t - getattr(b, "creep_t", t) < pause + getattr(b, "creep_dur", 0.0):
                continue
            x, y, th = b.pose[0], b.pose[1], b.pose[2] if len(b.pose) > 2 else 0.0
            ct = getattr(b, "cam_t", None)
            seen = ct is not None and t - ct <= self.blind_after(b)
            lc = getattr(b, "last_cam", None)
            if not seen and lc:
                tgt, words = (lc[0], lc[1]), "toward where the camera last read it"
            else:
                tgt, words = ((rect[0] + rect[2]) / 2.0, (rect[1] + rect[3]) / 2.0), "toward the board's middle"
            c = self.conf_now(b) if hasattr(self, "conf_now") else None
            unc = float((c or {}).get("plus_minus_cm") or 2.0) if isinstance(c, dict) else 2.0
            fr = CREEP_UNC0_CM / (CREEP_UNC0_CM + max(0.0, unc))
            L = max(CREEP_MIN_CM, CREEP_MAX_CM * fr)
            v = max(CREEP_V_MIN, CREEP_V_MAX * fr)
            sent = False
            for k, base in enumerate((tgt, hold)):
                if math.dist((x, y), base) < 0.5:
                    continue
                hd = math.degrees(math.atan2(base[1] - y, base[0] - x))
                for off in (0.0, 45.0, -45.0, 90.0, -90.0):
                    h_ = hd + off
                    nx, ny = x + L * math.cos(math.radians(h_)), y + L * math.sin(math.radians(h_))
                    if math.dist((nx, ny), hold) > CREEP_REACH_CM:
                        continue
                    if min(nx - rect[0], rect[2] - nx, ny - rect[1], rect[3] - ny) < 6.72 + unc:
                        continue
                    if any(o is not b and o.pose is not None and math.hypot(nx - o.pose[0], ny - o.pose[1]) <
                           self.RECOVER_GAP_CM for o in self.bots.values()):
                        continue
                    tid, sid = b.next_id, b.next_id + 1
                    b.next_id += 2
                    self.send(b.name, "fs %d T %.2f -" % (tid, h_))
                    self.send(b.name, "fs %d D %.2f %.2f %.2f %.2f %.2f %.2f 0 -" % (sid, x, y, nx, ny, h_, v))
                    b.creep_t = t
                    b.creep_dur = L / v + 1.0
                    b.creeps = getattr(b, "creeps", 0) + 1
                    self.creeps_n = getattr(self, "creeps_n", 0) + 1
                    self.log("held_creep", robot=b.name, L_cm=round(L, 1), v=round(v, 1), x=round(x, 1),
                             y=round(y, 1), to=[round(nx, 1), round(ny, 1)], toward="target" if k == 0 else "hold",
                             unc_cm=round(unc, 1))
                    if t - getattr(b, "creep_said", -1e9) >= 30.0:
                        b.creep_said = t
                        self.say("%s (held: %s) creeps %.0f cm at %.0f cm/s %s (never still; within %.0f cm of where "
                                 "it was held)" % (b.name, (b.out or "").split(";")[0], L, v,
                                                   words if k == 0 else "back toward where it was held",
                                                   CREEP_REACH_CM))
                    sent = True
                    break
                if sent:
                    break
            if not sent:
                b.creep_t = t                     # every way is blocked: a brief pause, then tried again

    def step_aside(self, o, x, y, bx, by, err):
        """(e18, 4) Robot o stands in a lost robot's straight recovery path: it steps sideways out of the corridor
        (its half width + o's half diagonal), checked against the edges and the others. True when sent."""
        if o.pose is None:
            return False
        f = self.m["floor"]
        rect = (f["x_min"], f["y_min"], f["x_max"], f["y_max"])
        L = max(1e-6, math.hypot(bx - x, by - y))
        ux, uy = (bx - x) / L, (by - y) / L
        side = -(o.pose[0] - x) * uy + (o.pose[1] - y) * ux
        need = self.RECOVER_GAP_CM + err + 1.0 - abs(side)
        if need <= 0:
            return False
        for sg in ((1.0 if side >= 0 else -1.0), (-1.0 if side >= 0 else 1.0)):
            d = need if sg == (1.0 if side >= 0 else -1.0) else need + 2 * abs(side)
            nx, ny = o.pose[0] - uy * sg * d, o.pose[1] + ux * sg * d
            if min(nx - rect[0], rect[2] - nx, ny - rect[1], rect[3] - ny) < 6.72 + 2.0:
                continue
            if any(q is not o and q.pose is not None and math.hypot(nx - q.pose[0], ny - q.pose[1]) < self.RECOVER_GAP_CM
                   for q in self.bots.values()):
                continue
            h_ = math.degrees(math.atan2(ny - o.pose[1], nx - o.pose[0]))
            self.control(o, "halt")
            tid, sid = o.next_id, o.next_id + 1
            o.next_id += 2
            self.send(o.name, "fs %d T %.2f -" % (tid, h_))
            self.send(o.name, "fs %d D %.2f %.2f %.2f %.2f %.2f %.2f 0 -" % (sid, o.pose[0], o.pose[1], nx, ny, h_, 5.0))
            self.log("step_aside", robot=o.name, d_cm=round(d, 1), to=[round(nx, 1), round(ny, 1)])
            self.say("%s steps %.0f cm aside: a lost robot's way back to where the camera reads passes here (it goes "
                     "first)" % (o.name, d))
            self.request_replan("%s stepped aside for a lost robot" % o.name)
            return True
        return False

    def blind_after(self, b, base=None):
        """(e14, school 17:46: a slow tag detector in dim light, 2.3 fixes a second) How old this robot's newest fix may
        be and it still counts as seen: from its MEASURED fix interval (safety_rules.fresh_fix_s: at least
        BLIND_AFTER_S, else 1.5 x its longest recent gap between fix arrivals). MIRLT_FIX_GAP_RULE=fixed: 0.6 s."""
        import safety_rules as SR
        h = getattr(b, "cam_hist", None) or []
        v = SR.fresh_fix_s([q[1] for q in h], self.BLIND_AFTER_S if base is None else base)
        if v > (self.BLIND_AFTER_S if base is None else base) + 1e-6 and not getattr(b, "_slow_fix_said", False):
            b._slow_fix_said = True
            self.log("slow_fixes", robot=b.name, fresh_s=round(v, 2),
                     words="fixes come slowly (the detector or camera): a fix counts as seen for %.2f s" % v)
        return v

    BLIND_AFTER_S = 0.6       # no camera fix for this long = no camera sees it (fixes come ~10 a second; the longest
                              # gap in any 29 Oct drive was 0.57 s, rules audit; 0.4 s gave spurious blind starts on
                              # a loaded dry-run machine). No DOE: a definition
    BLIND_MAX_CM = 40.0       # the farthest a robot may drive with no camera seeing it, on its own wheels and gyro.
                              # Nick (2026-10-29 20:06, question 1 of the straightening report): 40 cm (was 30). At 30
                              # a lane inside a strip both cameras lose (x = 30-45) stopped Monday at 74-84 s. 29 Oct
                              # logs: blind error p95 3.8 cm after 25 cm (median 1.5), 4.4 after 40 (BLIND_ERR_*), still
                              # under the 6 cm camera guard. DOE: 25 / 30 / 40 cm on the board at the sun's hour,
                              # against blind stops and the error when seen again
    BLIND_ERR_BASE_CM = 2.0   # the blind pose's p95 error, from 29 Oct's logs (dead reckoning from wheel_scale 0.979 /
    BLIND_ERR_PER_CM = 0.07   # gyro_scale 1.075 against both cameras, 22,000 stretches): p95 2.2 cm after 5 cm, 3.8 after
                              # 25, 4.4 after 40 = about 2.0 + 0.07 per cm driven blind (report, appendix)
    BLIND_EDGE_CM = 5.0       # while blind, its body must stay this far from an edge even if its pose is off by the
                              # p95 blind error: else everyone stops. Chosen (the edge guard sees an edge ahead at 11 cm
                              # only forwards). DOE: 3 / 5 / 8 cm against blind stops near lane ends
    REACQUIRE_CM = 2.5        # seen again after driving blind: its pose is reset to the camera's (fpose) when they
                              # disagree by more than this (the robot's fix gate would refuse a fix 3 cm+ off a pose it
                              # is sure of, and the camera guard stop everyone at 6 cm). Chosen; DOE: 1.5 / 2.5 / 4 cm
    REACQUIRE_MAX_CM = 12.0   # ... but not over this: the camera may have the wrong robot; the camera guard decides

    def pose_at(self, b, t):
        ph = getattr(b, "pose_hist", None) or []
        best = None
        for q in ph:
            if best is None or abs(q[0] - t) < abs(best[0] - t):
                best = q
        return best

    def blind_watch(self):
        """A robot no camera sees drives on on its own wheels and gyro (the firmware does). Capped: BLIND_MAX_CM driven
        blind, or its body nearer an edge than BLIND_EDGE_CM allowing for the blind error, stops everyone. Seen again:
        its pose is set to the camera's when they disagree (REACQUIRE_CM); the plan's steps are positions on the board,
        so it steers back onto its planned line and goes on with the pattern."""
        if not self.source.live or not self.t_run or getattr(self, "straightening", False):
            return
        t = now()
        f = self.m["floor"]
        rect = (f["x_min"], f["y_min"], f["x_max"], f["y_max"])
        for b in self.bots.values():
            if b.out or b.pose is None or getattr(b, "backoff", None):
                continue                       # (11-05h) a robot recovering / backing off is handled by its report
            ct = getattr(b, "cam_t", None)
            bl = getattr(b, "blind", None)
            if ct is not None and t - ct <= self.blind_after(b):
                if bl is not None:
                    b.blind = None
                    self.blind_end(b, bl)
                continue
            if ct is None:
                continue                       # never seen yet (the start handles that)
            if bl is None:
                p0 = self.pose_at(b, ct) or (ct,) + tuple(b.pose)
                b.blind = bl = dict(t0=ct, start=list(p0[1:3]), last=list(b.pose[:2]), dist=math.hypot(
                    b.pose[0] - p0[1], b.pose[1] - p0[2]), said=False)
                self.log("blind_start", robot=b.name, own=list(b.pose), last_cam=getattr(b, "last_cam", None),
                         t_run=round(t - self.t_run, 2))
            step_ = math.hypot(b.pose[0] - bl["last"][0], b.pose[1] - bl["last"][1])
            bl["dist"] += step_
            bl["last"] = list(b.pose[:2])
            if len(b.pose) > 2 and bl.get("th") is not None:      # (e18, 5) its turns, as its trust record counts them
                dth_ = abs((b.pose[2] - bl["th"] + 180.0) % 360.0 - 180.0)
                k_ = "ts" if step_ < 0.3 else "tc"
                bl[k_] = bl.get(k_, 0.0) + dth_
                bl["m" + k_[1]] = bl.get("m" + k_[1], 0.0) + dth_ * bl["dist"]
            bl["th"] = b.pose[2] if len(b.pose) > 2 else None
            if bl["dist"] >= 5.0 and not bl["said"]:
                bl["said"] = True
                self.say("%s: no camera sees it: driving on its own wheels and gyro (at most %.0f cm)"
                         % (b.name, self.BLIND_MAX_CM))
            body_edge = min(b.pose[0] - rect[0], rect[2] - b.pose[0], b.pose[1] - rect[1], rect[3] - b.pose[1]) - 4.75
            err = self.blind_err(b, bl)
            why = None
            import observe as OB
            ov = OB.overlap_cm(b.pose[0], b.pose[1], err, rect)
            veer = None
            if SG.value("MIRLT_ERROR_SHAPE") == "cone":
                # (e21) the one error shape: its reach toward each edge, not a circle
                import error_shape as ES_
                import safety_rules as SRs_
                sh = bl.get("shape")
                if sh is None:
                    sh = bl["shape"] = ES_.Shape(b.name, self.trust_of(b))
                    sh.anchor(t, bl["start"][0], bl["start"][1], b.pose[2] if len(b.pose) > 2 else None,
                              2.45 * 1.0, "lost by the camera")
                sh.drive(b.pose[0], b.pose[1], b.pose[2] if len(b.pose) > 2 else 0.0, step_,
                         abs((b.pose[2] - (bl.get("th_prev") if bl.get("th_prev") is not None else b.pose[2]) + 180.0)
                             % 360.0 - 180.0) if len(b.pose) > 2 else 0.0, step_ < 0.3)
                bl["th_prev"] = b.pose[2] if len(b.pose) > 2 else None
                v_s = bl["dist"] / max(0.2, t - bl["t0"])
                room = ES_.rect_margin(rect, b.pose[0], b.pose[1], *sh.ellipse()) - SRs_.HALF_DIAG_CM - \
                    SRs_.stop_cm(v_s if v_s > 0.3 else None)
                err = max(err, sh.extent())
                ov = -room if room < 0 else 0.0
                if room >= 0 and SG.value("MIRLT_STAGES") == "four" and not bl.get("veered"):
                    f2, k2, c2 = sh.axes_at(sh.s + 27.5, sh.head_doubt_rad())     # freeroam_core.VEER_AHEAD_CM
                    if room - max(0.0, sh.iso() + max(f2, k2, c2) - sh.extent()) < 0:
                        veer = "veer: its error shape (%.1f cm at most) would reach an edge within its next moves" % \
                            sh.extent()
            if veer is not None:
                bl["veered"] = True
                self.log("veer", robot=b.name, dist_cm=round(bl["dist"], 2), own=list(b.pose), why=veer)
                if not bl.get("recovered") and self.recover_blind(b, bl, veer):
                    self.say("%s: %s: it turns back early toward where the camera last read it (the veer stage)" % (
                        b.name, veer))
                    continue
            if ov > 0:
                # (11-05h, Nick 21:14, hard invariant) its possible-locations circle (half diagonal + the blind error)
                # reaches past an edge: it stops; only a move that shrinks that (the straight recovery inwards) follows
                why = "with no camera seeing it, its possible-locations circle (body + %.1f cm blind error) reaches " \
                      "%.1f cm past an edge" % (err, ov)
            elif bl.get("shape") is not None and __import__("safety_group").value("MIRLT_BLIND_RULE") != "cap":
                pass                           # (e21) the shape's own room decided above (ov)
            elif __import__("safety_group").value("MIRLT_BLIND_RULE") != "cap":
                # (e13, Nick 12:51 "never give up", CLAUDE.md) no blind-distance cap: it drives on while its whole
                # possible error (the blind error + half the body's diagonal + its stop at its blind speed, measured)
                # cannot reach an edge; BLIND_MAX_CM / BLIND_EDGE_CM are the switch MIRLT_BLIND_RULE=cap
                import safety_rules as SRb_
                v_b = bl["dist"] / max(0.2, t - bl["t0"])
                e_mid = min(b.pose[0] - rect[0], rect[2] - b.pose[0], b.pose[1] - rect[1], rect[3] - b.pose[1])
                need = err + SRb_.HALF_DIAG_CM + SRb_.stop_cm(v_b if v_b > 0.3 else None)
                if e_mid < need:
                    why = ("with no camera seeing it, its possible error (blind %.1f cm + body %.1f + stop %.1f) could "
                           "reach an edge %.1f cm away" % (err, SRb_.HALF_DIAG_CM, need - err - SRb_.HALF_DIAG_CM,
                                                           e_mid))
            elif bl["dist"] > self.BLIND_MAX_CM:
                why = "it drove %.0f cm with no camera seeing it (the limit is %.0f cm)" % (bl["dist"], self.BLIND_MAX_CM)
            elif body_edge - err < self.BLIND_EDGE_CM:
                why = ("with no camera seeing it, its body may be %.1f cm from an edge (its own pose %.1f cm, less the "
                       "blind error %.1f cm; it must keep %.0f cm)" % (body_edge - err, body_edge, err,
                                                                       self.BLIND_EDGE_CM))
            if why is None and getattr(self, "nopic_since", None) is not None and \
                    __import__("safety_group").value("MIRLT_NOPIC_RULE") != "off":
                # (e13, Nick 10 Nov: derived, risk-based; replaces e12's fixed 3 s) the camera sends no pictures: this
                # robot's own limit from its blind speed (measured: its dead-reckoned distance over the blind time), its
                # heading, its blind error and the room to the edge rule's clearance; over it, the same hold as above
                import safety_rules as SR_
                v_ = bl["dist"] / max(0.2, t - bl["t0"])
                lim = SR_.no_picture_limit(b.pose[0], b.pose[1], b.pose[2] if len(b.pose) > 2 else None, v_, err, rect,
                                           tracking="tags")
                if not bl.get("nopic_logged"):
                    bl["nopic_logged"] = True
                    self.log("picture_limit", robot=b.name, gap_s=round(t - ct, 2), limit_s=lim["limit_s"],
                             room_cm=lim["room_cm"], closing_cms=lim["closing_cms"], words=lim["words"])
                    self.say("%s: NO CAMERA PICTURES: its limit %s (%s)" % (
                        b.name, "none while it stands" if lim["limit_s"] == float("inf") else "%.1f s" % lim["limit_s"],
                        lim["words"]))
                if t - ct >= lim["limit_s"]:
                    why = "no camera pictures for %.1f s, over its own limit of %.1f s (%s)" % (
                        t - ct, lim["limit_s"], lim["words"])
            if why and not bl.get("recovered") and self.recover_blind(b, bl, why):
                continue                       # (11-05h) driving straight back to where it was read
            if why and self.holds_one() and not getattr(self, "straightening", False):
                # that robot alone stops (halted at once) and is held; the re-check finds it by its lights and rejoins
                self.log("blind_stop", robot=b.name, dist_cm=round(bl["dist"], 2), own=list(b.pose),
                         body_edge_cm=round(body_edge, 2), held=True)
                self.warnings = getattr(self, "warnings", []) + ["%s: %s" % (b.name, why)]
                self.say("WARNING: %s: %s: it is halted and held (found again by its lights, or its roof tag); the "
                         "others carry on" % (b.name, why))
                b.blind = None
                self.retire(b, why, kind="restarted")
                continue
            if why:
                self.abort = "%s: %s: stopping everyone" % (b.name, why)
                self.log("blind_stop", robot=b.name, dist_cm=round(bl["dist"], 2), own=list(b.pose),
                         body_edge_cm=round(body_edge, 2))
                self.say(self.abort)
                self.halt_all()
                return

    HEAD_GUARD_DEG = 25.0     # (11-05h) own heading vs the camera's (tag, offset corrected) over this = a reset, as
                              # the position guard. Chosen: well over a turn's error (3-7 deg) and the tag's noise (1 deg)
    REJOIN_FRESH_S = 1.0      # (11-05h) a held robot rejoins only on a camera fix this fresh (chosen: 10 fixes a second,
                              # the blind threshold 0.6 s plus slack)
    RECOVER_BEYOND_CM = 10.0  # (11-05h) the recovery drives this far past the last read spot, back into readable ground
                              # (chosen: a robot body; DOE note: 5 / 10 / 15 cm against recoveries seen again)
    RECOVER_GAP_CM = 16.0     # (11-05h) a lost robot's straight recovery must keep its middle this far (plus its blind
                              # error) from every other robot's middle, now and along their next planned steps: two
                              # half bodies (4.75 x 2) + the 8 cm net, rounded down; chosen. DOE note: not needed,
                              # it is the net

    def recover_blind(self, b, bl, why):
        """(11-05h, the calibration's 11-05c recovery for the missions) A robot no camera has read for too long:
        instead of holding it at once, it drives STRAIGHT BACK along the way it came (a backward drive: no turn, no
        new ground) towards where it was last read, at most as far as it drove blind, when that line keeps its whole
        blind-error bubble inside the table's edge room and away from every other robot's spot and planned path (the
        others' reservations win; they are not moved). Its plan is halted first; when seen again, everyone re-plans
        from where they are; not seen: the old hold. Returns True when a recovery was started."""
        import observe as OB
        if getattr(self, "straightening", False) or not self.holds_one() or getattr(b, "backoff", None):
            return False
        lc = getattr(b, "last_cam", None)
        if not lc or b.pose is None:
            return False
        x, y, th = b.pose
        # to where it was last read and RECOVER_BEYOND_CM on (the last read is at the unreadable patch's border: the
        # rehearsal's robot stopped 1 cm inside the dark patch, still unseen)
        back = min(bl["dist"], math.hypot(x - lc[0], y - lc[1])) + self.RECOVER_BEYOND_CM
        back = min(back, OB.BACK_OFF_CM * 4 + self.RECOVER_BEYOND_CM)
        if back < 3.0:
            return False
        # (11-05h, protection 4: never a blind reverse) it turns on the spot to face where it was last read and drives
        # there FORWARDS (its edge guard and front sensor work forwards only)
        hd = math.degrees(math.atan2(lc[1] - y, lc[0] - x))
        bx, by = x + back * math.cos(math.radians(hd)), y + back * math.sin(math.radians(hd))
        err = self.BLIND_ERR_BASE_CM + self.BLIND_ERR_PER_CM * (bl["dist"] + back)
        f = self.m["floor"]
        rect = (f["x_min"], f["y_min"], f["x_max"], f["y_max"])
        # (11-05h, hard invariant) the turn on the spot sweeps its circle where it stands; the drive must keep the
        # circle (growing with the blind error per cm) inside, or at least shrink an overlap it already has
        ok_, why_ = OB.path_ok([(x, y)] + [(x + (bx - x) * k / 10.0, y + (by - y) * k / 10.0) for k in range(1, 11)],
                               self.BLIND_ERR_BASE_CM + self.BLIND_ERR_PER_CM * bl["dist"], rect,
                               unc_per_cm=self.BLIND_ERR_PER_CM)
        if not ok_:
            self.log("recover_refused", robot=b.name, why=why_)
            return False
        # (e07, Nick 6 Nov) the recovery path is RESERVED in the others' plans: they re-plan round its corridor (the
        # robot is out of the plan while it drives). Their bodies where they stand now are still checked (a re-plan
        # cannot move a robot that is there). --recover-reserve off: e06's way, their next 4 steps checked only
        others_active = [o for o in self.bots.values() if o is not b and not o.out and o.name in self.names]
        reserve = getattr(self.args, "recover_reserve", "on") != "off" and bool(others_active)
        for k in range(11):
            px, py = x + (bx - x) * k / 10.0, y + (by - y) * k / 10.0
            for o in self.bots.values():
                if o is b or o.out or o.pose is None:
                    continue
                pts = [o.pose[:2]] + ([] if reserve else [
                    tuple(st["to"][:2]) for st, rec in zip(self.steps(o.name), o.stat)
                    if rec["status"] != "done" and st.get("to")][:4])
                for q in pts:
                    if math.hypot(px - q[0], py - q[1]) < self.RECOVER_GAP_CM + err:
                        if LOST_PRIORITY and q is pts[0] and not bl.get("asked_aside") and \
                                self.step_aside(o, x, y, bx, by, err):
                            bl["asked_aside"] = o.name      # (e18, 4) the lost robot first: o makes room, tried again
                            self.log("recover_wait", robot=b.name, for_=o.name)
                            return False
                        self.log("recover_refused", robot=b.name, why="the path of %s" % o.name,
                                 at=[round(px, 1), round(py, 1)])
                        return False
        bl["recovered"] = True
        self.send(b.name, "halt")
        tid = b.next_id
        sid = b.next_id + 1
        b.next_id += 2
        time.sleep(0.05)
        self.send(b.name, "fs %d T %.2f -" % (tid, hd))
        text = "fs %d D %.2f %.2f %.2f %.2f %.2f %.2f 0 -" % (sid, x, y, bx, by, hd, 5.0)
        ob = self.recovery_corridor(b.name, sid, x, y, bx, by, err)
        b.backoff = dict(id=sid, t=now(), ob=None, kind="recover", corridor=ob, turn_id=tid, reserved=reserve)
        self.send(b.name, text)
        if reserve:
            self.add_obstacle(ob)
            self.log("recover_reserved", robot=b.name, corridor=ob, others=[o.name for o in others_active])
            self.retire(b, "recovering (%s)" % why, kind="recovering", halt=False, reassign=False)
        self.__dict__.setdefault("recoveries", []).append(dict(robot=b.name, back_cm=round(back, 1),
                                                               blind_cm=round(bl["dist"], 1), why=why))
        self.log("recover_start", robot=b.name, back_cm=round(back, 1), from_=[round(x, 1), round(y, 1)],
                 to=[round(bx, 1), round(by, 1)], err_cm=round(err, 1), why=why)
        self.say("%s: %s: RECOVERING: it turns to face where the camera last read it and drives %.0f cm straight there, "
                 "forwards (clear of the edges and of the others' paths, checked)" % (b.name, why, back))
        rm = getattr(self, "readmap", None)
        skipped_ = None
        if rm is not None:
            rm.add(x, y, False)            # (e14) where the tag was lost: evidence
            k_, item = self.current_task(b.name)
            lab_ = self.point_of(item.get("label")) if isinstance(item, dict) else None
            xy_ = (self.m.get("spots") or {}).get(lab_[6:]) if lab_ else None
            if xy_ is not None and not rm.readable(xy_[0], xy_[1])[0]:
                lab = self.skip_current_point(b.name, "the camera does not read tags there (%s)" %
                                              rm.readable(xy_[0], xy_[1])[1])
                if lab:
                    skipped_ = lab
                    self.say("%s: point %s skipped: the camera does not read tags there (learned in this run)" % (
                        b.name, lab))
            self.skip_unreadable()
        if skipped_ is None:
            # (e14) "try anyway" visits: a point whose tag was just lost on the way is not aimed at again at once
            lab, how = self.defer_current_point(b.name, "the camera lost %s's roof tag on the way to it (at %.0f, %.0f)"
                                                % (b.name, x, y))
            if how == "deferred":
                self.say("%s: point %s moved to the end of its list: the camera lost its tag on the way (not tried "
                         "again at once)" % (b.name, lab))
            elif how == "skipped":
                self.say("%s: point %s skipped: the camera lost its tag on the way a second time" % (b.name, lab))
        return True

    LEAD_MIN_LEG_CM = 3.0     # (e07) legs shorter than this are sent as planned (chosen: the stop lead is a few mm)
    LEAD_MAX_SHARE = 0.4      # ... and a lead never takes more than this share of a leg (chosen; no DOE: a guard)

    def stop_leads(self):
        """(e07, Nick 6 Nov: missions use each robot's stop lead from its last calibration) {robot: {speed: cm}} from
        --stop-lead NAME=SPEED:CM,... (mission_run passes them from the Pi's stop_leads.json by WiFi name)."""
        if getattr(self, "_leads", None) is None:
            out = {}
            for spec in getattr(self.args, "stop_lead", None) or []:
                try:
                    n, rest = spec.split("=", 1)
                    out[n.strip()] = {float(a): float(b) for a, b in (kv.split(":") for kv in rest.split(",") if kv)}
                except ValueError:
                    continue
            self._leads = out
        return self._leads

    def with_stop_lead(self, n, stp):
        """A forward drive sent this robot's stop lead short (the fixed few mm it rolls on after the stop, measured
        per speed by its calibration), so it comes to rest on the planned spot. Anything else as planned."""
        leads = self.stop_leads().get(n)
        if not leads or stp.get("type") != "drive" or stp.get("reverse"):
            return stp
        fr, to = stp["from"], stp["to"]
        L = math.hypot(to[0] - fr[0], to[1] - fr[1])
        if L < self.LEAD_MIN_LEG_CM:
            return stp
        v = float(stp.get("speed") or 0.0)
        import safety_rules as SR        # (e08) the interpolation shared with free roam (one copy)
        lead = SR.stop_lead_for(leads, v, L)
        if lead <= 0.0:
            return stp
        k = (L - lead) / L
        nt = [fr[0] + (to[0] - fr[0]) * k, fr[1] + (to[1] - fr[1]) * k] + list(to[2:])
        if not getattr(self, "_lead_said", {}).get(n):
            self.__dict__.setdefault("_lead_said", {})[n] = True
            self.log("stop_lead_used", robot=n, leads={("%g" % a): b for a, b in leads.items()})
            self.say("%s: forward drives sent %s short (its stop lead from its last calibration), so it stops on the "
                     "spot" % (n, ", ".join("%.2f cm at %g cm/s" % (b, a) for a, b in sorted(leads.items()))))
        return dict(stp, to=nt, stop_lead_cm=round(lead, 3))

    def recovering_step(self, b, t, v):
        """A STEP record from a robot on its reserved recovery: its pose, the log, and the recovery's own steps."""
        try:
            sid, state, ms = int(v[1]), v[2], int(float(v[3]))
            x, y, th, front = float(v[4]), float(v[5]), float(v[6]), float(v[7])
        except (ValueError, IndexError):
            return
        info = ",".join(v[8:])
        b.note_clock(t, ms)
        if state in ("started", "done", "failed"):
            b.pose, b.pose_t = (x, y, th), t
        self.log("step_report", robot=b.name, id=sid, state=state, x=x, y=y, th=th, front_cm=front, info=info,
                 recovering=True)
        if state in ("held", "halted"):
            return
        self.backoff_report(b, sid, state, info)

    def release_reserved(self, b, bo):
        """(e07) A reserved recovery that was interrupted: its corridor off the map; the robot held where it is (the
        usual re-check brings it back), never left out for good."""
        self.m["obstacles"] = [o for o in self.m.get("obstacles", []) if o.get("id") != bo["corridor"]["id"]]
        self.m = FM.normalize(self.m)
        if b.out and getattr(b, "out_kind", None) == "recovering":
            b.out_kind = "restarted"
            b.back_t = now()
        self.handle_obstacle(None)

    def recovery_corridor(self, name, sid, x, y, bx, by, err):
        """(e07) The ground a straight recovery may sweep, as a map obstacle the others' re-plan keeps out of: a
        rectangle round the line from (x, y) to (bx, by), half as wide as the robot's half diagonal plus its blind
        error and 1 cm, and that much past each end (the turn on the spot at its start included)."""
        hw = 6.72 + float(err) + 1.0
        L = max(1e-6, math.hypot(bx - x, by - y))
        ux, uy = (bx - x) / L, (by - y) / L
        nx, ny = -uy, ux
        a = (x - ux * hw, y - uy * hw)
        c = (bx + ux * hw, by + uy * hw)
        pts = [[a[0] + nx * hw, a[1] + ny * hw], [c[0] + nx * hw, c[1] + ny * hw], [c[0] - nx * hw, c[1] - ny * hw],
               [a[0] - nx * hw, a[1] - ny * hw]]
        return {"id": "recover-%s-%d" % (name, sid), "shape": "polygon", "source": "robot_out",
                "points": [[round(p[0], 2), round(p[1], 2)] for p in pts], "note": "lost-tag recovery corridor"}

    def skip_unreadable(self, min_tries=3):
        """(11-05h) Every robot's points not done that lie where this run has LEARNED the camera does not read tags
        (min_tries reads missed there, not just the start guess) are skipped with that reason; the run carries on."""
        rm = getattr(self, "readmap", None)
        if rm is None:
            return []
        out = []
        for n, entry in (self.plan.get("robots") or {}).items():
            k0, _ = self.current_task(n)
            vo = entry.get("visit_order") or []
            for j in reversed(range(len(vo))):
                v = vo[j]
                if k0 is None or j <= k0 or not isinstance(v, dict):
                    continue
                lab_ = self.point_of(v.get("label"))
                xy_ = (self.m.get("spots") or {}).get(lab_[6:]) if lab_ else None
                if xy_ is None:
                    continue
                cell = rm.ij(xy_[0], xy_[1])
                ok_, why_ = rm.readable(xy_[0], xy_[1])
                if not ok_ and rm.tries.get(cell, 0) >= min_tries and BLIND_ALLOW:
                    al = self.blind_leg_allowed(n, xy_, rm.readable)    # (e18, 5) a blind out-and-back from readable
                    if al is not None and al["ok"]:
                        self.log("blind_leg_allowed", robot=n, point=lab_, **{k: v for k, v in al.items()
                                                                               if k != "first_bad"})
                        if not getattr(self, "_blind_said", {}).get((n, lab_)):
                            self.__dict__.setdefault("_blind_said", {})[(n, lab_)] = True
                            self.say("%s: point %s is where the camera does not read, but its odometry record allows "
                                     "a blind out-and-back from %s (predicted error at most %.1f cm): kept" % (
                                         n, lab_, al["from"], al["r_max"]))
                        continue
                    if al is not None:
                        why_ = "%s; a blind leg there is not allowed: %s" % (why_, al["why"])
                if not ok_ and rm.tries.get(cell, 0) >= min_tries:
                    del vo[j]
                    rec = dict(robot=n, point=lab_, why="the camera does not read tags there (learned: %s)" % why_,
                               t_run=round(now() - self.t_run, 2) if self.t_run else None)
                    self.__dict__.setdefault("skipped", []).append(rec)
                    self.log("point_skipped", **rec)
                    out.append(lab_)
        if out:
            self.say("SKIPPED (learned in this run, the camera does not read tags there): %s; the run carries on" %
                     ", ".join(out))
        return out

    def blind_end(self, b, bl):
        """Seen again after driving blind: how far its own pose was off, and the reset when needed."""
        h = getattr(b, "cam_hist", None) or []
        if not h:
            return
        tf, _, cx, cy, src = h[-1]
        own = self.pose_at(b, tf)
        if own is None:
            return
        d = math.hypot(cx - own[1], cy - own[2])
        rec = dict(robot=b.name, dist_cm=round(bl["dist"], 2), blind_s=round(tf - bl["t0"], 2), err_cm=round(d, 2),
                   cam=[round(cx, 2), round(cy, 2)], own=[round(own[1], 2), round(own[2], 2)], src=src,
                   t_run=round(now() - self.t_run, 2) if self.t_run else None)
        if BLIND_ALLOW and bl["dist"] >= 2.0:
            pred = self.blind_err(b, bl, record=True)    # (e18, 5) what its record predicted against what was found
            chk = self.trust_of(b).check(pred, d, kind="seen again (mission)", t=now())
            rec.update(pred_cm=round(pred, 2), within=chk["within"], k_cal=round(self.trust_of(b).k_cal, 3))
        if self.REACQUIRE_CM < d <= self.REACQUIRE_MAX_CM and b.pose is not None:
            # the camera's spot moved on by what the robot did since that picture
            nx, ny = cx + (b.pose[0] - own[1]), cy + (b.pose[1] - own[2])
            self.send(b.name, "fpose %.2f %.2f %.2f %.2f %.2f" % (nx, ny, b.pose[2], 1.0, 3.0))
            b.acc_t, b.acc_odo = now(), getattr(b, "odo", 0.0)     # its pose is the camera's again (confidence)
            rec["reset"] = [round(nx, 2), round(ny, 2)]
            self.say("%s: seen again after %.0f cm with no camera; its own pose was %.1f cm off: set to the camera's, "
                     "and it steers back onto its planned line" % (b.name, bl["dist"], d))
        elif bl["dist"] >= 5.0:
            self.say("%s: seen again after %.0f cm with no camera; its own pose was %.1f cm off" % (b.name, bl["dist"], d))
        self.log("blind_end", **rec)
        self.__dict__.setdefault("blinds", []).append(rec)

    def finished(self):
        if self.plan.get("stranded"):
            return False
        if not self.names:
            return False
        if any(b.out and getattr(b, "out_kind", None) == "recovering" for b in self.bots.values()):
            return False                # (e07) a robot on its reserved recovery comes back with points to do
        undone = [n for n in self.names if not all(st["status"] == "done" for st in self.bots[n].stat)]
        if undone:
            if any(getattr(self.bots[n], "retry_n", 0) == 0 for n in undone):
                self.others_done_t = None       # (a robot trying to rejoin does not restart the wait)
            return False
        if getattr(self, "others_done_t", None) is None:
            self.others_done_t = now()
        # everyone running is done: a held robot may still rejoin (rejoin_watch), for REJOIN_WAIT_S at most from now
        # (each retry of it renews its own out time; the run must still end: 30 Oct simulator, a run that never did)
        held = [b for b in self.bots.values() if b.out and self.rejoinable(b)]
        if held and self.keep_going() and all(getattr(b, "handed_over", False) for b in held):
            # keep going, points reassigned (2026-11-03): a held robot's points were done by the others; it would only
            # drive home if it rejoined, so the run does not wait REJOIN_WAIT_S for it
            if not getattr(self, "_handover_end_said", False):
                self._handover_end_said = True
                self.say("%s still held: its points were handed over and are done, so the run ends without waiting "
                         "for it" % ", ".join(b.name for b in held))
            held = []
        if held and now() - self.others_done_t < self.REJOIN_WAIT_S:
            return False
        if held:
            self.warnings = getattr(self, "warnings", []) + ["%s never joined this run (still held: %s)" % (
                ", ".join(b.name for b in held), "; ".join(str(b.out) for b in held))]
        return True

    def run(self, timeout_s):
        self.plan_start = self.t_run = now()
        self.say("plan %s started: %d robots, %d steps, planned %.1f s"
                 % (self.plan.get("plan_id"), len(self.names), sum(len(self.steps(n)) for n in self.names),
                    self.plan["summary"]["makespan_s"]))
        self.note_skips(self.plan)           # points the first plan could not reach (keep going plans them skipped)
        for n, why in list((getattr(self, "start_aside", None) or {}).items()):
            # held at the start (the self-correcting start, 30 Oct): a fixed obstacle where it stands; checked again
            # every so often and rejoins when it passes (rejoin_watch, recheck)
            if n in self.bots and n in self.names:
                self.retire(self.bots[n], "held at the start: %s" % why, kind="restarted")
        last_status = now()
        while True:
            self.drain()
            self.fixes()
            for b in self.bots.values():
                if b.ended and not self.abort and not b.out:
                    if self.holds_one():
                        # Nick 30 Oct 10:08: one robot's trouble is not everyone's stop. Its edge guard (or anything)
                        # ended its follow run: it has stopped already; it is held and checked again, the others go on
                        why = ("its edge guard stopped it before an open plywood edge" if SC.run_outcome(b.ended) ==
                               SC.CALIB_LIMITED else "its follow run ended (%s)" % b.ended)
                        self.edge_end_strip(b)
                        self.warnings = getattr(self, "warnings", []) + ["%s: %s" % (b.name, why)]
                        self.say("WARNING: %s: %s: held and checked again; the others carry on" % (b.name, why))
                        self.retire(b, why, kind="restarted")
                        b.ended = None
                        continue
                    if SC.run_outcome(b.ended) == SC.CALIB_LIMITED:   # (follow runs hold instead: firmware older than
                        #                                                 2026-10-18c's step-only edge stop)
                        self.abort = "limited: %s's edge guard stopped it before an open plywood edge" % b.name
                    else:
                        self.abort = "%s's follow run ended (%s)" % (b.name, b.ended)
                    self.say(self.abort + ": stopping everyone")
                    self.halt_all()
            if self.abort:
                break
            self.cam_guard()
            if self.abort:
                break
            self.blind_watch()
            self.backoff_watch()
            self.readmap_watch()
            if self.abort:
                break
            self.silence_watch()
            if self.abort:
                break
            self.unconfirmed_watch()
            self.edge_watch()
            self.gap_forward()            # (e09, B2)
            self.held_creep_watch()       # (e18, Nick 12 Nov decision 2) never still: held robots creep
            self.trust_checkpoint()       # (e18, 6) the odometry records written out every minute
            self.rejoin_watch()
            if self.abort:
                break
            self.disk_watch()
            if self.abort:
                self.halt_all()
                break
            self.status_poll()
            self.write_live()
            self.finish_replan()
            self.tracker_watch()
            pr = getattr(self, "pause_req", None)
            if pr is not None:
                self.pause_req = None
                self.pause(pr)
            if self.pending_replan and now() >= self.pending_replan[1] and self.planning is None:
                self.pending_replan = None
                self.handle_obstacle(None)
            if self.improve_at is not None and now() >= self.improve_at and self.planning is None:
                self.improve_at = None
                if not self.plan.get("stranded"):
                    self.handle_obstacle(None, budget=self.args.improve_budget_s, improving=True)
            if self.plan.get("stranded") and now() >= self.next_retry and self.planning is None:
                self.retries += 1
                budget = min(12.0, self.plan["settings"]["replan_budget_s"] * 3 ** min(self.retries, 2))
                self.say("retrying a route for %s (%.0f s allowed)" % (", ".join(self.plan["stranded"]), budget))
                self.handle_obstacle(None, budget=budget)
            self.release()
            self.resend()
            if self.finished() and self.planning is None:
                self.say("all robots done")
                self.write_live(final=True)
                break
            if now() > self.deadline(timeout_s):
                self.say("timed out; stopping everyone")
                self.halt_all()
                self.abort = "timeout"
                break
            if now() - last_status > 10.0:
                last_status = now()
                self.say("  " + "  ".join("%s %d/%d" % (n, sum(st["status"] == "done" for st in self.bots[n].stat),
                                                          len(self.bots[n].stat)) for n in self.names))
                for n in self.names:
                    self.log("waiting", robot=n, why=self.waiting_for(n))
                    if all(self.bots[x].follow.get("cur", 0) == 0 for x in self.names):
                        self.say("    %s: %s" % (n, self.waiting_for(n)))
            time.sleep(0.01)

    def missed_boards(self):
        """Boards in each robot's tasks (map.json) it never reported docking at."""
        out = {}
        for n in self.bots:
            want = [v for v in self.m["tasks"].get(n, {}).get("visit", []) if isinstance(v, str)]
            got = [d["board"] for d in self.dockings if d["robot"] == n]
            miss = list(want)
            for g in got:
                if g in miss:
                    miss.remove(g)
            if miss:
                out[n] = miss
        return out

    def summary(self):
        if self.__dict__.get("points") and getattr(self, "outdir", None):
            try:                                   # final rows (a later bump may have widened a point's +-)
                with open(os.path.join(self.outdir, "points.csv"), "w", newline="") as fh:
                    w = csv.writer(fh)
                    w.writerow(self.POINT_COLS)
                    for q in self.points:
                        w.writerow(["" if q.get(k) is None else q.get(k) for k in self.POINT_COLS])
            except OSError:
                pass
        rm = getattr(self, "readmap", None)
        if rm is not None and getattr(self, "outdir", None):
            try:
                rm.save(os.path.join(self.outdir, "readmap.json"))      # (11-05h) where tags read in this run
            except OSError:
                pass
        return {"observe": {"trips": list(getattr(self, "trips", []) or []),            # (11-05h)
                            "recoveries": list(getattr(self, "recoveries", []) or []),
                            "heading_relearned": list(getattr(self, "heading_relearned", []) or []),
                            "surface_marks": list(getattr(self, "surface_marks", []) or []),
                            "did_not_move": getattr(self, "n_fast_done", 0),
                            "skipped": list(getattr(self, "skipped", []) or [])},
                "safety_group": SG.snapshot(), "held_creeps": getattr(self, "creeps_n", 0),
                "odo_trust": {n: b.trust.summary() for n, b in self.bots.items() if getattr(b, "trust", None)},
                "plan_id_first": self.first_plan_id, "plans": self.plan_no + 1, "result": self.abort or "done",
                "duration_s": round(now() - self.started, 2),
                "planned_makespan_s": self.first_makespan,
                "resends": {n: b.resends for n, b in self.bots.items()},
                "steps_sent": {n: b.next_id - 1 for n, b in self.bots.items()},
                "fixes_sent": {n: b.fix_sent for n, b in self.bots.items()},
                "fixes_held_no_pose": {n: getattr(b, "fix_held", 0) for n, b in self.bots.items()},
                "warnings": list(getattr(self, "warnings", []) or []),
                "held_at_start": dict(getattr(self, "start_aside", None) or {}),
                "fixes_used_rejected": {n: (b.follow.get("fixes_used"), b.follow.get("fixes_rejected"))
                                        for n, b in self.bots.items()},
                "replans": self.replans, "dockings_reported": self.dockings,
                "boards_not_visited": self.missed_boards(),
                "sensor_obstacles": self.n_sensor_obstacles,
                "sensor_stops": getattr(self, "sensor_stops", []),
                "robots_out": getattr(self, "outs", []),
                "plans_refused": getattr(self, "n_refused", 0),
                "front_stop": {"readings": getattr(self.args, "obstacle_readings", None),
                               "stop_cm": {n: getattr(b, "stop_cm", None) for n, b in self.bots.items()},
                               "unconfirmed": self.unconf().summary() if "_unconf" in self.__dict__ else {"added": 0},
                               "no_plan_drops": getattr(self, "no_plan_drops", 0)},
                "rejoins": getattr(self, "rejoins", []),
                "confirm_checks": getattr(self, "confirms", []),
                "battery": {n: b.battery for n, b in self.bots.items()},
                "boot": {n: b.boot for n, b in self.bots.items()},
                "start_checks": getattr(self, "start_checks", {}),
                "straighten": getattr(self, "straighten_record", None),
                "blind": getattr(self, "blinds", []),
                "on_problem": getattr(self.args, "on_problem", "stop"),
                "lost_robot_points": getattr(self.args, "lost_robot_points", "reassign"),
                "kept_going": self.__dict__.get("kept_going", []),
                "points": self.__dict__.get("points", []),
                "points_skipped": self.__dict__.get("skipped", []),
                "points_reassigned": [{k: q[k] for k in ("point", "frm", "to", "t_run")}
                                      for q in self.__dict__.get("reassigned", [])],
                "turn_pause": {"on": bool(getattr(self.args, "turn_pause", False)) and self.source.live,
                               "robots": self.turn_pause_summary()},
                "mirror": self.mirror_summary()}


# ----------------------------------------------------------------------------
# Start-up: robots found, firmware, follow runs, poses
# ----------------------------------------------------------------------------

def wait_for(cond, timeout, coord, poll=0.05):
    t0 = time.time()
    while time.time() - t0 < timeout:
        coord.drain()
        if cond():
            return True
        time.sleep(poll)
    return False


def _say(msg):
    print(msg, flush=True)


FIND_ROUNDS_SOFT = 3     # the self-correcting start waits this many times --find-s for every robot before it starts
                         # with the ones heard (the others held, joining when they answer). No DOE: a patience


def find_robots(coord, link, args, soft=False):
    """Wait for every robot, re-pinging the ones not heard yet every PING_EVERY_S (15:27: one lost ping and run_plan
    heard nobody for 20 s). Says which were heard at all: none heard is a link problem, not a robot problem.
    soft (the self-correcting start): waits longer, then returns when at least one robot is heard (the caller holds
    the others); raises only when none is heard at all."""
    names = coord.names
    print("Waiting for robots: %s ..." % ", ".join(names))
    t0 = time.time()
    last_ping = t0
    while time.time() - t0 < args.find_s * (FIND_ROUNDS_SOFT if soft else 1):
        coord.drain()
        if set(names) <= set(link.robots()):
            return
        if time.time() - last_ping >= PING_EVERY_S:
            last_ping = time.time()
            link.send_all("ping")
            for ip in args.robot or []:
                link.send_ip(ip, "ping")
        time.sleep(0.05)
    heard = sorted(link.robots())
    missing = sorted(set(names) - set(heard))
    if soft and set(heard) & set(names):
        return
    if not heard:
        raise SystemExit("Not heard from: %s. No robot answered at all in %.0f s (pinged every %.0f s): a link "
                         "problem (this computer's WiFi, or another program talking to the robots), not a robot "
                         "problem." % (", ".join(missing), args.find_s, PING_EVERY_S))
    raise SystemExit("Not heard from: %s (heard: %s). Are they on, on this WiFi, and named as in the map?"
                     % (", ".join(missing), ", ".join(heard)))


def read_status(coord, link, names, wait_s=5.0):
    """status from every robot (firmware, battery, boot ID); re-asked once halfway."""
    for n in names:
        coord.bots[n].firmware = None
        link.send_name(n, "status")
    t0 = time.time()
    asked = False
    while time.time() - t0 < wait_s:
        coord.drain()
        if all(coord.bots[n].firmware and coord.bots[n].battery is not None and coord.bots[n].boot for n in names):
            return
        if not asked and time.time() - t0 > wait_s / 2:
            asked = True
            for n in names:
                if not (coord.bots[n].firmware and coord.bots[n].battery is not None):
                    link.send_name(n, "status")
        time.sleep(0.05)


def calib_poses(coord, names, source=None, wait_s=2.0):
    """{robot: (x, y, heading or None, how)} for the roll caps: the camera's fixes when a live source sends them within
    wait_s, else the plan's start spots (where the robots were put)."""
    poses = {}
    lit = getattr(coord, "facing_at", None) or {}
    for n in names:
        if n in lit:
            # where its lights are (the facing check, anywhere on the board) and the heading they gave; before 30 Oct
            # the camera's fix or the plan's start spot was used, and a robot put down elsewhere got the plan's
            x, y, h, how, _ = lit[n]
            poses[n] = (x, y, h, how)
    if source is not None and getattr(source, "live", False) and len(poses) < len(coord.names):
        t_end = time.time() + wait_s
        while time.time() < t_end and len(poses) < len(coord.names):
            for ev in source.poll():
                if ev.get("type") == "fix" and ev.get("robot") in coord.names and ev.get("x") is not None and \
                        ev["robot"] not in poses:
                    poses[ev["robot"]] = (float(ev["x"]), float(ev["y"]),
                                          None if ev.get("th") is None else float(ev["th"]), "camera")
            time.sleep(0.05)
    for n in coord.names:
        if n not in poses:
            st = (coord.plan.get("robots") or {}).get(n, {}).get("start")
            if st:
                poses[n] = (float(st[0]), float(st[1]), float(st[2]) if len(st) > 2 else None, "plan start")
    return poses


def calib_caps(coord, link, names, source=None):
    """Before calibrating (2026-10-29): each robot's room to spin, and its roll capped per direction towards open
    space (`set calib_dir_limits`), from where it stands, the floor and the other robots (those calibrating at the
    same time keep to their own half of the floor between them). Returns the robots without room to spin (not
    calibrated: they need moving in)."""
    f = coord.m["floor"]
    rect = (f["x_min"], f["y_min"], f["x_max"], f["y_max"])
    poses = calib_poses(coord, names, source)
    no_room = []
    for n in names:
        if n not in poses:
            print("  %s: where it stands is not known: its roll is not capped (its edge guard still stops it at an "
                  "edge)" % n)
            continue
        x, y, h, how = poses[n]
        others = {m: poses[m][:2] for m in poses if m != n}
        ok, why = CR.spin_room((x, y), rect, others)
        if not ok and getattr(coord, "self_correct", False):
            print("  %s: no room to spin where it stands (%s, %s): its calibration is the compass check alone (a few "
                  "degrees' wiggle)" % (n, why, how))
            no_room.append(n)
            continue
        if not ok:
            print("  %s: no room to calibrate where it stands (%s, %s): not calibrated. Move it in (%.0f cm from every "
                  "edge, %.0f cm from the other robots) and start again." % (n, why, how, CR.ROOM_EDGE_CM,
                                                                             CR.ROOM_ROBOT_CM))
            no_room.append(n)
            continue
        fixed = [others[m] for m in others if m not in names]
        moving = [others[m] for m in others if m in names]
        cp = CR.caps((x, y), h, rect, fixed=fixed, moving=moving, unc=CR.HEADING_UNC_LOOSE_DEG)
        link.send_name(n, CR.caps_cmd(cp))
        print("  %s at (%.1f, %.1f) (%s): %s (the most it may roll; its own front sensor must also see room)"
              % (n, x, y, how, CR.caps_words(cp, h, CR.HEADING_UNC_LOOSE_DEG)))
        coord.log("calib_caps", robot=n, x=x, y=y, h=h, how=how, caps=cp)
    time.sleep(0.2)
    return no_room


def calib_roll_words(n, lk):
    """Why the robot's own look skipped (or shortened) its roll, from its CALIB_LOOK numbers, in plain words. The
    laptop's caps only LIMIT the roll; the robot rolls only where its front sensor also saw room: calib_roll_min_cm (5)
    + safe_front_cm (7) + calib_margin_cm (3) = 15 cm clear ahead. A sensor that reads the floor (Left's floor reads
    10.6-15 cm standing, the floor check of 29 Oct) sees 'something' at that distance all round."""
    try:
        room8 = [float(v) for v in str(lk.get("room8") or "").split(";") if v != ""]
    except ValueError:
        room8 = []
    if not room8 and lk.get("room_cm") in (None, ""):
        return ""
    seen = [r for r in room8 if r >= 0]
    near = [r for r in seen if r < 15.0]
    bits = []
    if room8:
        bits.append("its front sensor saw something %s cm ahead at 0, 45, ... 315 deg" % "/".join(
            "-" if r < 0 else ("clear" if r >= 900 else "%.0f" % r) for r in room8))
    if lk.get("live_cm") not in (None, "", "0.0", "0"):
        bits.append("%s cm in its lane after turning" % lk.get("live_cm"))
    if lk.get("caps"):
        bits.append("laptop caps %s" % lk.get("caps"))
    w = "%s's own look: %s." % (n, "; ".join(bits))
    if seen and len(near) == len(seen):
        w += (" Something under 15 cm in every direction it looked: on an open board that is most likely its sensor "
              "reading the floor (a roll needs 15 cm clear ahead)")
    return w


def calibrate(coord, link, args, names, source=None, compass_only=False):
    """The calibration run on these robots (all at once; about 65 s each). compass_only: no spin and no roll (the
    compass check's few degrees of wiggle alone, which needs almost no room): the robot can drive afterwards, with its
    stored scales ('limited')."""
    if compass_only:
        no_room = []
        for n in names:
            link.send_name(n, "set calib_spin_deg 0")
            link.send_name(n, CR.caps_cmd([(d, 0.0) for d in CR.CAP_DEGS]))
            coord.log("calib_compass_only", robot=n)
        time.sleep(0.3)
    else:
        try:
            no_room = calib_caps(coord, link, names, source)
        except Exception as e:      # noqa: BLE001  (the caps only make the roll safer: never stop a run for them)
            print("  (roll caps not worked out: %s; the edge guard still stops a roll at an edge)" % e)
            no_room = []
    coord.calib_no_room = list(no_room)
    names = [n for n in names if n not in no_room]
    if not names:
        return []
    for n in names:
        coord.bots[n].last_runend = None
        link.send_name(n, "calibrate")
    print("Calibrating %s (about 65 s, all at once: each needs %.0f cm from every edge and %.0f cm from the other "
          "robots to spin; its roll goes towards open space, or is skipped where there is none)..."
          % (", ".join(names), CR.ROOM_EDGE_CM, CR.ROOM_ROBOT_CM))
    if not wait_for(lambda: all((getattr(coord.bots[n], "last_runend", None) or (None,))[0] == "calib" for n in names),
                    120, coord):
        print("  not every calibration run reported its end; carrying on")
    time.sleep(1.0)
    coord.drain()
    done = []
    for n in names:
        b = coord.bots[n]
        le = getattr(b, "last_runend", None)
        oc = SC.calib_outcome(le[1] if le and le[0] == "calib" else None, b.calib_look)
        if oc == SC.CALIB_LIMITED:    # not a failure: it did what the space allowed
            lk = b.calib_look or {}
            what = []
            if lk.get("spin") == "skipped":
                what.append("no spin (something inside its circle)")
            if lk.get("roll") == "edge":
                what.append("roll cut short at a plywood edge (kept)")
            elif lk.get("roll") == "skipped" or not lk.get("roll"):
                what.append("no roll (stopped at a plywood edge too soon)" if lk.get("result") == "edge"
                            else "no roll (no room in any direction)")
            print("  %s: calibrated, %s: %s" % (n, SC.calib_outcome_text(le[1], lk), "; ".join(what)))
            why = calib_roll_words(n, lk)
            if why:
                print("    " + why)
            done.append(n)
        elif oc == SC.CALIB_FAILED:
            print("  %s: calibration did not finish (%s)" % (n, le[1] if le and le[0] == "calib" else "no end record"))
        else:
            done.append(n)
    if compass_only:
        for n in names:
            link.send_name(n, "set calib_spin_deg %g" % CALIB_SPIN_DEFAULT)
    for n in done:
        coord.day.note_calibrated(n, coord.bots[n].boot)
        coord.bots[n].calibrated = None
    coord.day.save()
    return done


def send_settings(coord, link, args, names):
    """Every `set` this run needs, per robot: the --send commands (the edge guard), the stored driving corrections
    (real robots), the four-sided border, obs_n. Returns {robot: {param: value}} to read back."""
    want = {n: {} for n in names}
    notes = []
    for n in names:
        b = coord.bots[n]
        cmds = list(getattr(args, "send", None) or [])
        if args.corrections:
            cmds += EC.corrections_cmds(n, coord.day, notes)
        own = EC.robot_settings_cmds(n)          # its own settings (Left's 10 cm stop limit, question 59): every run
        if own:
            cmds += own
            notes.append("%s: its own settings: %s" % (n, ", ".join(c[4:] for c in own)))
        fw = b.firmware or ""
        if coord.m.get("sides") and getattr(args, "border_cm", 0) > 0:
            if fw and fw < BORDER4_FIRMWARE:
                print("  %s runs firmware %s: no four-sided border on the robot (needs %s); the planner's margins "
                      "and the edge guard still apply" % (n, fw, BORDER4_FIRMWARE))
            else:
                cmds += border_commands(coord.m, args.border_cm)
        if args.obstacle_readings and fw >= OBS_N_FIRMWARE:
            cmds.append("set obs_n %d" % args.obstacle_readings)
        elif args.obstacle_readings and fw:
            notes.append("%s runs firmware %s: its front stop is time-based (100 ms, about 1-2 readings), not %d "
                         "readings (needs %s)" % (n, fw, args.obstacle_readings, OBS_N_FIRMWARE))
        for c in cmds:
            link.send_name(n, c)
            kv = EC.parse_setting(c)
            if kv:
                want[n][kv[0]] = kv[1]
            time.sleep(0.02)
    for x in notes:
        print("  " + x)
    coord.sent_settings = want
    return want


def read_back(coord, link, want, tries=EC.READBACK_TRIES):
    """Compare each robot's PARAM rows (its follow run's start) with what was sent; ask (`get`) for any missing,
    resend (`set`) any that differ. Returns {robot: [(param, sent, got)]} still wrong."""
    bad = {}
    for k in range(tries):
        coord.drain()
        bad = {}
        for n, kv in want.items():
            b = coord.bots[n]
            for name, v in kv.items():
                got = b.params.get(name)
                if not EC.setting_matches(v, got):
                    bad.setdefault(n, []).append((name, v, got))
        if not bad:
            return {}
        for n, lst in bad.items():
            for name, v, got in lst:
                if got is not None:                     # a different value: the set was lost or overwritten
                    link.send_name(n, "set %s %g" % (name, v))
                    coord.n_setting_resent = getattr(coord, "n_setting_resent", 0) + 1
                    print("  %s: %s reads %g, sent %g: sent again" % (n, name, got, v))
                    coord.log("setting_resent", robot=n, name=name, sent=v, got=got, round=k + 1)
                coord.bots[n].params.pop(name, None)
                link.send_name(n, "get %s" % name)
                time.sleep(0.02)
        wait_for(lambda: all(coord.bots[n].params.get(name) is not None for n, lst in bad.items()
                             for name, _, _ in lst), 1.5, coord)
    coord.drain()
    return {n: [(nm, v, coord.bots[n].params.get(nm)) for nm, v, _ in lst
                if not EC.setting_matches(v, coord.bots[n].params.get(nm))] for n, lst in bad.items()
            if any(not EC.setting_matches(v, coord.bots[n].params.get(nm)) for nm, v, _ in lst)}


def floor_check(coord, link, args, names):
    """The floor check (Nick, 2026-10-29 16:2x; front_stop.py): each robot stands still while its five front zones are
    read (FLOOR_SAMPLE_S); its stop limit (follow_look_cm) becomes the lowest of 12 cm, its own ceiling (Left 10 cm,
    engine_checks.ROBOT_SETTINGS) and its floor's 3-in-a-row level less --floor-margin-cm, never under 8 cm. Every
    connect (the floor moved 15 -> 12 cm after a restart on 29 Oct, and drive to drive within one boot). Said in plain
    words, logged (floor_check events, floor_check.json, the summary's start_checks)."""
    if getattr(args, "floor_check", "on") == "off":
        return {}
    own = getattr(EC, "ROBOT_SETTINGS", {}) or {}
    coord.floor_raw = {n: [] for n in names}
    t0 = time.time()
    print("Floor check: each robot stands still while its distance sensor reads the floor (%.1f s) ..."
          % FSTOP.FLOOR_SAMPLE_S)
    while time.time() - t0 < FSTOP.FLOOR_SAMPLE_S:
        coord.drain()
        time.sleep(0.03)
    rows, coord.floor_raw = coord.floor_raw, None
    want, out = {}, {}
    sent = getattr(coord, "sent_settings", {}) or {}
    for n in names:
        b = coord.bots[n]
        ceiling = float((own.get(n) or {}).get("follow_look_cm", FSTOP.FLOOR_CAP_CM))
        fl = FSTOP.floor_level(rows.get(n) or [])
        ahead = floor_blocked(coord, n)
        if ahead and fl.get("floor_cm") is not None:
            fl = dict(fl, floor_cm=None, why="%s is %.0f cm in front of it (it would be read as the floor)" % ahead)
        lim, rule = FSTOP.stop_limit(fl.get("floor_cm"), ceiling, getattr(args, "floor_margin_cm",
                                                                           FSTOP.FLOOR_MARGIN_CM),
                                     lo_cm=FSTOP.robot_min_cm(n))
        words = FSTOP.limit_words(n, fl, lim, rule, ceiling, getattr(args, "floor_margin_cm", FSTOP.FLOOR_MARGIN_CM))
        coord.say("floor check: " + words)          # (said: Mission Control's event log shows it)
        coord.log("floor_check", robot=n, limit_cm=lim, rule=rule, ceiling_cm=ceiling, floor=fl, words=words)
        b.stop_cm = lim
        out[n] = dict(limit_cm=lim, rule=rule, ceiling_cm=ceiling, floor_cm=fl.get("floor_cm"),
                      lowest_cm=fl.get("lowest_cm"), median_cm=fl.get("median_cm"), readings=fl.get("n"),
                      zones=fl.get("zones"), why=fl.get("why"), words=words)
        cur = (sent.get(n) or {}).get("follow_look_cm", b.params.get("follow_look_cm", FSTOP.FLOOR_CAP_CM))
        if cur is None or abs(float(cur) - lim) > 0.05:
            link.send_name(n, "set follow_look_cm %.1f" % lim)
            want[n] = {"follow_look_cm": lim}
            sent.setdefault(n, {})["follow_look_cm"] = lim
        if rule == "minimum":
            coord.say("WARNING: %s's floor reads very near (%.1f cm): its stop limit is the lowest allowed, %.1f cm. "
                      "Look at its sensor and what is in front of it" % (n, fl["floor_cm"], lim))
    if want:
        time.sleep(0.1)
        for n in want:
            coord.bots[n].params.pop("follow_look_cm", None)
            link.send_name(n, "get follow_look_cm")
        wait_for(lambda: all(coord.bots[n].params.get("follow_look_cm") is not None for n in want), 1.5, coord)
        wrong = read_back(coord, link, want)
        if wrong:
            raise SystemExit("The floor check's stop limit was not confirmed by %s. Nothing was moved."
                             % ", ".join(wrong))
    if hasattr(coord, "start_checks"):
        for n, v in out.items():
            coord.start_checks.setdefault(n, {})["stop_limit"] = {k: v[k] for k in ("limit_cm", "rule", "floor_cm",
                                                                                   "ceiling_cm")}
    try:
        old = {}
        p = os.path.join(coord.outdir, "floor_check.json")
        if os.path.exists(p):
            with open(p) as f:
                old = json.load(f) or {}
        old.setdefault("checks", []).append(dict(t=round(time.time(), 2), robots=out, margin_cm=getattr(
            args, "floor_margin_cm", FSTOP.FLOOR_MARGIN_CM), readings_in_a_row=FSTOP.STOP_READINGS))
        with open(p, "w") as f:
            json.dump(old, f, indent=1)
    except (OSError, ValueError):
        pass
    return out


def floor_blocked(coord, n):
    """(name, distance) of another robot or a known obstacle within FLOOR_ROBOT_AHEAD_CM in front of robot n (+-35 deg
    of its heading): the zones would read it as the floor. None when nothing is."""
    b = coord.bots[n]
    st = (coord.plan.get("robots") or {}).get(n, {}).get("start")
    pose = b.pose or (tuple(st[:3]) if st else None)
    if pose is None:
        return None
    hx, hy = math.cos(math.radians(pose[2])), math.sin(math.radians(pose[2]))
    others = []
    for o in coord.bots.values():
        if o.name == n:
            continue
        ost = (coord.plan.get("robots") or {}).get(o.name, {}).get("start")
        op = o.pose or (getattr(o, "last_cam", None) and tuple(o.last_cam[:2])) or (tuple(ost[:2]) if ost else None)
        if op:
            others.append((o.name, op[0], op[1]))
    for ob in coord.m.get("obstacles", []):
        for p in (ob.get("points") or ([ob["center"]] if ob.get("center") else [])):
            others.append(("an obstacle", p[0], p[1]))
    best = None
    for nm, x, y in others:
        dx, dy = x - pose[0], y - pose[1]
        d = math.hypot(dx, dy)
        if d < 1e-6 or d > FSTOP.FLOOR_ROBOT_AHEAD_CM:
            continue
        if (dx * hx + dy * hy) / d >= math.cos(math.radians(35.0)) and (best is None or d < best[1]):
            best = (nm, d)
    return best


def floor_words(args):
    """The battery floor in force and where it comes from (2026-11-02: printed at start-up and in every refusal)."""
    f = getattr(args, "battery_floor", EC.BATTERY_FLOOR_PCT)
    if not f:
        return "off (--battery-floor 0)"
    w = EC.battery_floor_words(f)
    return w if f == EC.battery_floor() else "%d %% (--battery-floor %d)" % (f, f)


def recheck_robot(coord, link, args, n, source):
    """A robot that restarted mid-run, checked again before it rejoins (Coordinator.recheck): status (battery),
    its settings (read back), calibration if it says it is not calibrated, the facing check (its heading), its follow
    run, the floor check. (ok, words)."""
    b = coord.bots[n]
    link.send_name(n, "stop")
    time.sleep(0.5)
    b.ended = None
    b.boot = None
    read_status(coord, link, [n])
    if not b.boot:
        wait_for(lambda: b.boot, 3.0, coord)
    if b.battery is not None and args.battery_floor and b.battery < args.battery_floor:
        return False, "its battery is at %d %%, under the %s floor" % (b.battery, floor_words(args))
    prev = dict(getattr(coord, "sent_settings", {}) or {})
    want = send_settings(coord, link, args, [n])
    coord.sent_settings = dict(prev, **want)
    did = []
    heading = None
    if getattr(args, "facing_check", False) and getattr(source, "live", False):
        try:
            import facing as FC
            try:
                res = FC.run_plan_check(coord, link, args, [n], straighten=True, tag="facing-rejoin-%s" % n)
            except TypeError:
                res = FC.run_plan_check(coord, link, args, [n])
            heading = ((res or {}).get("headings") or {}).get(n) if isinstance(res, dict) else None
            if isinstance(res, dict):
                fp = after_facing(coord, res, [n])     # the bridge finds it by its lights, wherever it stands now
                if n in fp:
                    b.last_cam = [fp[n][0], fp[n][1], None if heading is None else round(float(heading), 1)]
                    b.cam_t = now()
            did.append("facing checked")
        except SystemExit as e:
            return False, "the facing check: %s" % e
    start_follow(coord, link, [n])
    if b.calibrated is False:
        if args.recalibrate == "no":
            return False, "not calibrated since its restart (--recalibrate no)"
        done = calibrate_robust(coord, link, args, [n], source, auto=True)
        if n not in done:
            return False, "its calibration did not finish (three tries, the last the compass check alone)"
        did.append("calibrated")
        link.send_name(n, "stop")
        time.sleep(0.5)
        start_follow(coord, link, [n])
    if b.run_exp != "follow" or b.calibrated is False:
        return False, "its follow run did not start"
    if not wait_for(lambda: b.follow, 8, coord):
        return False, "no FOLLOW records"
    wait_for(lambda: all(k in b.params for k in want[n]), 1.5, coord)
    wrong = read_back(coord, link, {n: want[n]})
    if wrong:
        return False, "its settings were not confirmed (%s)" % ", ".join("%s sent %g, says %s" % x for x in wrong[n])
    did.append("settings read back")
    floor_check(coord, link, args, [n])
    did.append("floor checked")
    if heading is not None and getattr(b, "last_cam", None):
        b.last_cam = [b.last_cam[0], b.last_cam[1], round(float(heading), 1)]
    return True, "boot %s: %s" % (b.boot, ", ".join(did))


def start_follow(coord, link, names):
    for n in names:
        coord.bots[n].ended = None
        coord.bots[n].run_exp = None
    for attempt in range(3):
        for n in names:
            if coord.bots[n].run_exp != "follow":
                link.send_name(n, "run follow")
        if wait_for(lambda: all(coord.bots[n].run_exp == "follow" or coord.bots[n].calibrated is False for n in names),
                    4, coord):
            break


def replan_from(coord, poses, why):
    """Plan again with each robot starting where the camera has it (Mission Control, Monday's command, and robots the
    engine itself moved for a calibration)."""
    import copy
    m2 = copy.deepcopy(coord.m)
    for r in m2["robots"]:
        if r["name"] in poses:
            r["start"] = [round(v, 2) for v in poses[r["name"]]]
    over = dict(coord.plan.get("settings") or {})
    gate = getattr(coord.args, "min_gap_cm", EC.MIN_GAP_CM) or 0.0
    if len(coord.plan.get("robots") or {}) >= 2:
        over["separation_cm"] = max(float(over.get("separation_cm", 0.0)), gate)   # at the net (the 8 cm lock)
    plan = P.make_plan(m2, over=over)
    coord.m = FM.normalize(m2)
    coord.plan = plan
    coord._install(plan, keep={}, first={}, old=None)
    coord.plan_no += 1
    coord.save_plan()
    coord.first_makespan = plan["summary"]["makespan_s"]
    print("  planned again from where the camera has them (%s): done in %.0f s" % (why, plan["summary"]["makespan_s"]))
    coord.log("replan_from_camera", why=why, poses=poses, makespan_s=plan["summary"]["makespan_s"])


SAFETY_PARAMS = ("safe_edge", "edge_", "bd_")    # settings that are the edge guard or the border: a robot that will not
                                                # confirm them is held (rechecked), never run without them
CALIB_SPIN_DEFAULT = 720.0                      # firmware params.h calib_spin_deg (restored after a compass-only one)


def warn(coord, text):
    """A WARNING instead of a stop (Nick, 2026-10-30 10:08: "I'd rather have a warning and a failure than a stop"):
    said, logged, and counted in the summary."""
    print("WARNING: " + text, flush=True)
    coord.warnings = getattr(coord, "warnings", []) + [text]
    try:
        coord.log("warning", text=text)
    except Exception:       # noqa: BLE001
        pass


def hold_at_start(coord, link, n, why):
    """A robot that could not be made ready is held, not a stop: the others start, and the run checks it again (status,
    settings, calibration, the facing check, its follow run) every so often and rejoins it when it passes
    (Coordinator.retire(kind="restarted") at the run's start, then rejoin_watch / recheck)."""
    aside = coord.__dict__.setdefault("start_aside", {})
    if n in aside:
        return
    aside[n] = why
    warn(coord, "%s is held for now: %s. The others start; it is checked again during the run and joins when it "
                "passes" % (n, why))
    try:
        link.send_name(n, "stop")
    except Exception:       # noqa: BLE001  (it may not be answering at all)
        pass


def live_names(coord, names):
    return [n for n in names if n not in (getattr(coord, "start_aside", None) or {})]


def calibrate_robust(coord, link, args, names, source, auto=True):
    """calibrate(), and for a robot it did not calibrate (no room to spin, or the run failed): once more, then the
    compass check alone (no spin, no roll: the gyro and wheel scales stored for it are used, 'limited'). Returns the
    robots calibrated."""
    done = calibrate(coord, link, args, names, source)
    if not auto:
        return done
    left = [n for n in names if n not in done and n not in (getattr(coord, "calib_no_room", None) or [])]
    if left:
        warn(coord, "%s: calibration did not finish: calibrating again" % ", ".join(left))
        for n in left:
            link.send_name(n, "stop")
        time.sleep(1.0)
        done += calibrate(coord, link, args, left, source)
    left = [n for n in names if n not in done]
    if left:
        warn(coord, "%s: no room to spin, or the calibration failed twice: the compass check alone (a few degrees' "
                    "wiggle; no spin, no roll); its stored gyro and wheel scales are used" % ", ".join(left))
        for n in left:
            link.send_name(n, "stop")
        time.sleep(0.5)
        for n in left:
            # one at a time: robots this short of room may stand within each other's wiggle (simulator, robots put 14
            # cm apart: bodies came to 2.6 cm with both wiggling at once)
            done += calibrate(coord, link, args, [n], source, compass_only=True)
    return done


def follow_robust(coord, link, args, names, source, auto=True):
    """Start the follow runs; a robot that does not start is asked again (twice, waiting longer each time); one that
    says it is not calibrated is calibrated (calibrate_robust). Returns the robots whose follow run is up."""
    start_follow(coord, link, names)
    for k in range(2 if auto else 0):
        miss = [n for n in names if coord.bots[n].run_exp != "follow" and coord.bots[n].calibrated is not False]
        if not miss:
            break
        warn(coord, "%s did not start its follow run: asked again (%d of 2)" % (", ".join(miss), k + 1))
        for n in miss:
            link.send_name(n, "stop")
        time.sleep(0.5 * (k + 1))
        start_follow(coord, link, miss)
    return [n for n in names if coord.bots[n].run_exp == "follow" and coord.bots[n].calibrated is not False]


def start_robots(coord, link, args, source):
    straight = bool(getattr(args, "straighten", False) and getattr(args, "facing_check", False) and source.live)
    auto = straight            # the self-correcting start (Monday and Mission Control: mission_run passes --straighten)
    coord.self_correct = auto  # ... and run (one robot's trouble holds that robot, never stops the others)
    coord.start_aside = {}
    all_names = list(coord.names)
    for attempt in range(2 if auto else 1):
        try:
            _start_robots(coord, link, args, source, straight, auto)
        except SystemExit:
            coord.names = all_names
            raise
        live = live_names(coord, all_names)
        coord.names = all_names
        if live or not auto:
            break
        if attempt == 0:
            warn(coord, "every robot was held (%s): the whole start once more"
                 % "; ".join("%s: %s" % kv for kv in coord.start_aside.items()))
            coord.start_aside = {}
            time.sleep(3.0)
    if auto and not live_names(coord, all_names):
        raise SystemExit("Start-up failed: no robot could be made ready, twice (%s)"
                         % "; ".join("%s: %s" % kv for kv in coord.start_aside.items()))


def _start_robots(coord, link, args, source, straight, auto):
    names = coord.names
    find_robots(coord, link, args, soft=auto)
    for n in [n for n in names if n not in link.robots()]:
        hold_at_start(coord, link, n, "not heard at the start (off, asleep or out of WiFi)")
    names = live_names(coord, names)
    for n in names:
        link.send_name(n, "stop")          # a follow run left over from an earlier run_plan would refuse a new one
    time.sleep(1.5)
    coord.drain()
    # ---- status: firmware, battery, boot ID (every run: a resumed ladder step or a second Monday run checks it too)
    read_status(coord, link, names)
    if not all(coord.bots[n].boot for n in names):
        wait_for(lambda: all(coord.bots[n].boot for n in names), 3.0, coord)     # CLOCK records carry it
    if auto and not all(coord.bots[n].firmware for n in names):
        read_status(coord, link, [n for n in names if not coord.bots[n].firmware])   # once more (a lost status)
    for n in names:
        b = coord.bots[n]
        fw = b.firmware
        if not fw:
            print("  %s did not say its firmware version (status lost?); carrying on" % n)
        elif fw < MIN_FIRMWARE:
            if not auto:
                raise SystemExit("%s runs firmware %s; the step follower needs %s or later. Flash it first."
                                 % (n, fw, MIN_FIRMWARE))
            hold_at_start(coord, link, n, "it runs firmware %s; the step follower needs %s (flash it)" % (fw, MIN_FIRMWARE))
    names = live_names(coord, names)
    low = [(n, coord.bots[n].battery) for n in names
           if coord.bots[n].battery is not None and args.battery_floor and coord.bots[n].battery < args.battery_floor]
    coord.start_checks = {n: {"firmware": coord.bots[n].firmware, "battery": coord.bots[n].battery,
                              "boot": coord.bots[n].boot} for n in coord.names}
    if low:
        if not auto:
            raise SystemExit("Battery under the %d %% floor: %s. Charge %s first (nothing was moved). [floor: %s]"
                             % (args.battery_floor, ", ".join("%s %d %%" % x for x in low),
                                "it" if len(low) == 1 else "them", floor_words(args)))
        warn(coord, "battery under the %d %% floor: %s: it runs anyway (charge it after)"
             % (args.battery_floor, ", ".join("%s %d %%" % x for x in low)))
    print("  " + "; ".join("%s: firmware %s, battery %s %%, boot %s" % (n, coord.bots[n].firmware,
                                                                       coord.bots[n].battery, coord.bots[n].boot)
                           for n in names) + "  [battery floor: %s]" % floor_words(args))
    # ---- restarts: a new boot ID since the robot's last calibration today
    need_cal = list(names) if args.calibrate else []
    for n in names:
        rs = coord.day.restarted(n, coord.bots[n].boot)
        prev = coord.day.robot(n).get("calibrated_boot")
        coord.start_checks[n]["restarted"] = bool(rs and prev)
        if rs and prev:
            print("  %s has restarted since its calibration today (boot %s, then %s): calibration and corrections "
                  "again" % (n, prev, coord.bots[n].boot))
            coord.log("restarted", robot=n, old=prev, new=coord.bots[n].boot)
            if args.recalibrate == "auto" and n not in need_cal:
                need_cal.append(n)
    want = send_settings(coord, link, args, names)
    moved = []
    if need_cal and args.recalibrate != "no":
        moved = calibrate_robust(coord, link, args, need_cal, source, auto=auto)
        if args.calibrate and not args.yes and not args.dry_run and not args.replan_from_camera:
            input("Calibrated. Put every robot on its start spot, then press Enter: ")
            moved = []
    # ---- the facing check (Nick, 2026-10-29): which way each robot faces, from its lights, before its follow run
    # (`led` is refused during one) and after any calibration (its roll turns the robot). facing.py
    fres = None
    if getattr(args, "facing_check", False):
        import facing as FC
        fres = FC.run_plan_check(coord, link, args, names, straighten=straight)
        given_start_poses(coord, args, fres, names)   # (e14) robots with no tag that reads: where motion found them
        if source.live:
            after_facing(coord, fres, names)       # the bridge follows each robot from its lights, wherever it stands
    # ---- the follow runs; a robot that says "not calibrated" is calibrated now (auto), then started again
    follow_robust(coord, link, args, names, source, auto=auto)
    bad = [n for n in names if coord.bots[n].calibrated is False]
    if bad and args.recalibrate == "auto":
        print("  not calibrated since power-on: %s" % ", ".join(bad))
        cal_done = calibrate_robust(coord, link, args, bad, source, auto=auto)
        moved += cal_done
        for n in bad:
            link.send_name(n, "stop")
        time.sleep(0.5)
        if straight and cal_done:
            # the calibration's spin and roll turned and moved them (the roll ends where it began, but facing the way
            # it rolled): measure again before straightening, and tell the bridge where they are now (30 Oct)
            print("  calibrated: the facing check again (the calibration turned them)")
            for n in names:
                link.send_name(n, "stop")
            wait_for(lambda: all(coord.bots[n].run_exp != "follow" or coord.bots[n].ended for n in names), 2.0, coord)
            time.sleep(0.3)
            coord.drain()
            fres = FC.run_plan_check(coord, link, args, names, straighten=True, tag="facing-after-calibration")
            after_facing(coord, fres, names)
            follow_robust(coord, link, args, names, source, auto=auto)
        else:
            follow_robust(coord, link, args, bad, source, auto=auto)
        bad = [n for n in names if coord.bots[n].calibrated is False]
    if bad:
        if not auto:
            raise SystemExit("Not calibrated since power-on: %s. Lift each off its spot, send `calibrate` (console) or "
                             "use --calibrate, put it back, and start again." % ", ".join(bad))
        for n in bad:
            hold_at_start(coord, link, n, "it still says it is not calibrated after three calibrations")
        names = live_names(coord, names)
    missing = [n for n in names if coord.bots[n].run_exp != "follow"]
    if missing:
        if not auto:
            raise SystemExit("These robots did not start their follow run: %s" % ", ".join(missing))
        for n in missing:
            hold_at_start(coord, link, n, "its follow run did not start (asked three times)")
        names = live_names(coord, names)
    # wait until each follow loop is up (its FOLLOW records arrive)
    if not wait_for(lambda: all(coord.bots[n].follow for n in names), 8, coord):
        quiet = [n for n in names if not coord.bots[n].follow]
        if not auto:
            raise SystemExit("No FOLLOW records from: %s" % ", ".join(quiet))
        warn(coord, "no FOLLOW records from %s: its follow run is started again" % ", ".join(quiet))
        for n in quiet:
            link.send_name(n, "stop")
        time.sleep(0.5)
        follow_robust(coord, link, args, quiet, source, auto=auto)
        if not wait_for(lambda: all(coord.bots[n].follow for n in quiet), 8, coord):
            for n in [n for n in quiet if not coord.bots[n].follow]:
                hold_at_start(coord, link, n, "no FOLLOW records from its follow run (started twice)")
        names = live_names(coord, names)
    # ---- read back every setting sent (its PARAM rows), resend what differs
    wait_for(lambda: all(all(k in coord.bots[n].params for k in want[n]) for n in names), 1.5, coord)
    wrong = read_back(coord, link, {n: want[n] for n in names})
    if wrong and auto:
        wrong = read_back(coord, link, {n: dict((nm, v) for nm, v, _ in lst) for n, lst in wrong.items()})
    coord.start_checks["settings_resent"] = getattr(coord, "n_setting_resent", 0)
    if wrong:
        words = "; ".join("%s %s" % (n, ", ".join("%s sent %g, says %s" % x for x in lst)) for n, lst in wrong.items())
        if not auto:
            raise SystemExit("Settings not confirmed by the robot after %d tries: %s. Nothing was moved."
                             % (EC.READBACK_TRIES, words))
        for n, lst in wrong.items():
            safety = [x[0] for x in lst if str(x[0]).startswith(SAFETY_PARAMS)]
            if safety:
                hold_at_start(coord, link, n, "its edge guard / border settings were not confirmed (%s)"
                              % ", ".join(safety))
            else:
                warn(coord, "%s did not confirm %s: it runs with what it has" % (
                    n, ", ".join("%s (sent %g, says %s)" % x for x in lst)))
        names = live_names(coord, names)
    print("  settings read back: %s" % ", ".join("%s %d ok" % (n, len(want[n])) for n in names))
    try:
        floor_check(coord, link, args, names)
    except SystemExit as e:
        if not auto:
            raise
        warn(coord, "the floor check: %s: the usual stop limit (12 cm; Left 10) is kept" % e)
    coord.names = names
    # where are they?
    starts = {n: coord.plan["robots"][n]["start"] for n in names}
    if straight:
        # start straightening (Nick, 2026-10-29 15:41): measured pose against the start pose; a turn on the spot, or a
        # drive to its spot, then a re-check, instead of a stop (straighten.py)
        straighten_start(coord, link, args, source, names, fres, auto=auto)
        names = live_names(coord, names)
        coord.names = names
    elif source.live:
        print("Waiting for camera fixes...")
        seen = {}
        for _ in range(200):          # what queued up meanwhile (a calibration: a minute of fixes) is old: drop it
            if not source.poll():
                break

        def got():
            for ev in source.poll():
                if ev.get("robot") in moved:
                    # its calibration roll moved it, which the camera bridge cannot know (it counts a robot as moved
                    # only by its own pose since 29 Oct 16:2x): the map's start heading it sends is not vouched for
                    continue
                if ev.get("type") == "fix" and ev.get("robot") in starts and ev.get("th") is not None:
                    seen[ev["robot"]] = ev     # (the start check needs a heading: position-only fixes wait)
            return set(seen) == set(names)
        wait_for(got, args.fix_wait_s, coord)
        for n, (x_, y_, th_) in start_poses_given(args).items():
            if n in names and seen.get(n) is None and th_ is not None:
                # (e14) no tag that reads: where the motion finding put it, before the run
                seen[n] = {"type": "fix", "robot": n, "x": x_, "y": y_, "th": th_, "src": "motion find"}
                warn(coord, "%s has no roof tag that reads: it starts from where the motion finding put it (%.1f, %.1f, "
                            "facing %.0f deg)" % (n, x_, y_, th_))
        for n in names:
            if seen.get(n) is None:
                if n in moved:
                    # 19:23, 28 Oct (DOE loop, Right drive 1): the calibration's roll moved it off its start spot, so
                    # the camera bridge no longer vouches for the start heading and no fix with a heading comes
                    raise SystemExit("No camera fix with a heading for %s: it was calibrated just now and its roll "
                                     "moved it. It IS calibrated now: put it back on its start spot facing its start "
                                     "direction if it is off it, and start the run again (no calibration this time)."
                                     % n)
                raise SystemExit("No camera fix for %s: is the position source running?" % n)
        off = {n: math.hypot(seen[n]["x"] - starts[n][0], seen[n]["y"] - starts[n][1]) for n in names}
        offh = {n: abs((seen[n]["th"] - starts[n][2] + 180) % 360 - 180) for n in names}
        if (args.replan_from_camera or moved) and any(off[n] > args.place_tol_cm or offh[n] > 5.0 for n in names):
            replan_from(coord, {n: (seen[n]["x"], seen[n]["y"], seen[n]["th"]) for n in names},
                        "after a calibration" if moved else "Mission Control: start where they stand")
            starts = {n: coord.plan["robots"][n]["start"] for n in names}
        for n in names:
            ev = seen[n]
            s = starts[n]
            d = math.hypot(ev["x"] - s[0], ev["y"] - s[1])
            dh = abs((ev["th"] - s[2] + 180) % 360 - 180)
            if args.place_tol_deg is not None:
                tol_deg, how = args.place_tol_deg, "fixed"
            else:
                hc = EC.start_heading_check(coord.plan, n, coord.m, dh, float(ev.get("line_cm") or EC.START_LINE_CM))
                tol_deg, how = (hc["allowed_deg"] if hc["allowed_deg"] is not None else 180.0), hc["words"]
            # only the side camera sees it (Nick, 29 Oct, question 60): it reads ~3 cm off, so 5 cm is allowed
            side_only = str(ev.get("src") or "").endswith(":side")
            tol_cm = max(args.place_tol_cm, SIDE_ONLY_PLACE_CM) if side_only else args.place_tol_cm
            print("  %s: camera puts it %.1f cm / %.1f deg from its start spot (allowed %.1f cm%s; heading allowed "
                  "%.1f: %s)" % (n, d, dh, tol_cm, ", side camera alone" if side_only else "", tol_deg, how))
            coord.start_checks[n].update(place_cm=round(d, 2), heading_deg=round(dh, 2), place_allowed_cm=tol_cm,
                                         heading_allowed_deg=round(tol_deg, 2), heading_rule=how)
            if (d > tol_cm or dh > tol_deg) and not args.yes:
                raise SystemExit("%s is not on its start spot (%.1f cm, %.1f deg off; allowed %.1f cm, %.1f deg)."
                                 % (n, d, dh, tol_cm, tol_deg))
            coord.bots[n].fix = ev
    else:
        if not args.yes and not args.dry_run:
            input("No camera: each robot starts from its map start spot. Check they are on their spots, then press Enter: ")
        for n in names:
            s = starts[n]
            link.send_name(n, "fpose %.2f %.2f %.2f 0.5 2" % (s[0], s[1], s[2]))
    t0 = time.time()
    while time.time() - t0 < 6:
        coord.drain()
        coord.fixes()
        if all(coord.bots[n].follow.get("state") not in (None, "nopose") for n in names):
            break
        time.sleep(0.1)
        if not source.live and time.time() - t0 > 2:
            for n in names:
                if coord.bots[n].follow.get("state") == "nopose":
                    s = starts[n]
                    link.send_name(n, "fpose %.2f %.2f %.2f 0.5 2" % (s[0], s[1], s[2]))
    nopose = [n for n in names if coord.bots[n].follow.get("state") in (None, "nopose")]
    if nopose and auto:
        warn(coord, "no pose on %s yet: its pose sent again" % ", ".join(nopose))
        for n in nopose:
            p_ = getattr(coord.bots[n], "last_cam", None)
            if p_ and p_[2] is not None:
                link.send_name(n, "fpose %.2f %.2f %.2f 1.0 2.0" % tuple(p_))
        wait_for(lambda: all(coord.bots[n].follow.get("state") not in (None, "nopose") for n in nopose), 4.0, coord)
        for n in [n for n in nopose if coord.bots[n].follow.get("state") in (None, "nopose")]:
            hold_at_start(coord, link, n, "it did not take its pose")
        names = live_names(coord, names)
        coord.names = names
        nopose = []
    if nopose:
        raise SystemExit("No pose on: %s" % ", ".join(nopose))
    # ---- the safety net: refuse a plan under the gate (Nick, 16:43: 8 cm)
    if args.min_gap_cm:
        gap = EC.plan_gap(coord.plan, coord.m, gate_cm=args.min_gap_cm)
        coord.start_checks["plan_gap"] = gap
        bad = EC.gate_words(gap, args.min_gap_cm)
        if bad:
            raise SystemExit("Plan refused: %s. Nothing was moved." % bad)
        print("  plan's closest approach %.1f cm (safety net %.1f cm)" % (gap["closest_cm"], args.min_gap_cm))
    for n in names:
        b = coord.bots[n]
        b.boot_start = b.boot
        r = coord.day.robot(n)
        r.update(boot=b.boot, battery=b.battery, firmware=b.firmware, seen_t=round(time.time(), 1),
                 sent=want.get(n, {}))
        if b.boot and not r.get("calibrated_boot"):
            r["calibrated_boot"] = b.boot      # it ran follow: its compass check is done since this boot
    coord.day.save()
    print("Robots ready: " + ", ".join("%s at %.1f,%.1f,%.0f" % (n, *coord.bots[n].pose) for n in names))


SIDE_ONLY_PLACE_CM = 5.0   # the start check's allowance when only the side camera sees the robot (question 60, Nick
                           # 29 Oct 15:41; as cam_bridge.MOVED_SIDE_CM, where the DOE note is)


def camera_positions(coord, names, wait_s, settle_s=0.8, soft=False):
    """Where the camera has each robot, standing: the median of its fixes over the last settle_s (position only), and
    whether only the side camera saw it. {name: (x, y)}, {name: side_only}. SystemExit when one is not seen."""
    t0 = now()
    for b in coord.bots.values():
        b.__dict__.setdefault("cam_hist", [])
    got = {}
    while now() - t0 < wait_s:
        coord.drain()
        coord.fixes()
        for n in names:
            h = [f for f in coord.bots[n].cam_hist if f[1] >= t0]
            if h and h[-1][1] - h[0][1] >= settle_s and len(h) >= 4:
                use = [f for f in h if f[1] >= h[-1][1] - settle_s]
                got[n] = ((float(statistics.median([f[2] for f in use])),
                           float(statistics.median([f[3] for f in use]))), all(f[4] == "side" for f in use))
        if set(got) == set(names) and now() - t0 >= settle_s:
            break
        time.sleep(0.03)
    miss = [n for n in names if n not in got]
    if miss and not soft:
        raise SystemExit("No camera fix for %s: no camera finds %s on the board. Nothing moved."
                         % (", ".join(miss), "it" if len(miss) == 1 else "them"))
    return {n: got[n][0] for n in names if n in got}, {n: got[n][1] for n in names if n in got}


def set_poses(coord, link, poses):
    """fpose each robot (x, y, heading), then wait until its FOLLOW record carries it."""
    for n, (x, y, h) in poses.items():
        link.send_name(n, "fpose %.2f %.2f %.2f 1.0 2.0" % (x, y, h))
        coord.bots[n].fposed_t = now()
    for attempt in range(3):
        def ok():
            for n, (x, y, h) in poses.items():
                b = coord.bots[n]
                if b.pose is None or b.pose_t < b.fposed_t or math.hypot(b.pose[0] - x, b.pose[1] - y) > 2.0 or \
                        abs((b.pose[2] - h + 180) % 360 - 180) > 5.0:
                    return False
            return True
        if wait_for(ok, 2.0, coord):
            return
        for n, (x, y, h) in poses.items():
            link.send_name(n, "fpose %.2f %.2f %.2f 1.0 2.0" % (x, y, h))
            coord.bots[n].fposed_t = now()
    raise SystemExit("No pose on: %s (fpose not taken)" % ", ".join(poses))


def restart_follow(coord, link, names):
    """End the follow runs (the facing check's `led` is refused during one), for the re-check."""
    for n in names:
        link.send_name(n, "stop")
    wait_for(lambda: all(coord.bots[n].ended for n in names), 3.0, coord)
    time.sleep(0.3)
    coord.drain()
    for n in names:
        b = coord.bots[n]
        b.follow, b.pose, b.fposed_t = {}, None, None
        b.controls = []          # a halt from a failed correction: the run it was for has ended


LIGHTS_AHEAD_CM = 1.0     # the lights' middle sits 0.4-1.5 cm ahead of the robot's middle (facing.py, 28 Oct find
                          # steps): the robot's middle is taken this far behind it. No DOE: the bridge's tracker
                          # re-centres on the outline within a picture or two
SEED_SENDS = 3            # each seed goes this many times (UDP), 0.1 s apart; the bridge drops repeats


def facing_poses(fres, names):
    """{robot: (x, y, heading or None, how)} from a facing check: where its lights are (anywhere on the board) and the
    heading they gave. heading None when front and back were not told apart (or the cameras disagreed)."""
    out = {}
    for n in names:
        r = ((fres or {}).get("robots") or {}).get(n) or {}
        meas = r.get("meas") or {}
        mid = meas.get("mid")
        if not mid:
            continue
        h = meas.get("heading") if meas.get("sure") else None
        hh = ((fres or {}).get("headings") or {}).get(n, h)
        x, y = float(mid[0]), float(mid[1])
        if meas.get("centre"):
            # read from its roof tag (2026-11-02): the tag is over the robot's middle, no lights' offset
            x, y = float(meas["centre"][0]), float(meas["centre"][1])
            out[n] = (round(x, 2), round(y, 2), None if h is None else float(h), "roof tag (facing check)",
                      None if hh is None else float(hh))
            continue
        if hh is not None:
            x -= LIGHTS_AHEAD_CM * math.cos(math.radians(hh))
            y -= LIGHTS_AHEAD_CM * math.sin(math.radians(hh))
        out[n] = (round(x, 2), round(y, 2), None if h is None else float(h), "lights (facing check)",
                  None if hh is None else float(hh))
    return out


def start_poses_given(args):
    """(e14) {name: (x, y, th)} from --start-pose NAME=X,Y,TH: robots with no roof tag that reads, found by motion
    before the run (wiggle_bind's finding)."""
    out = {}
    for sp in getattr(args, "start_pose", None) or []:
        n, _, v = sp.partition("=")
        try:
            q = [float(x) for x in v.split(",")]
            out[n.strip()] = (q[0], q[1], q[2] if len(q) > 2 else None)
        except (ValueError, IndexError):
            continue
    return out


def given_start_poses(coord, args, fres, names):
    """(e14) The facing check cannot read a robot with no roof tag: where the motion finding put it and the heading
    its stroke showed are used instead (its heading checked again by its first drives' motion, observe), and the camera
    bridge's motion detector is seeded there (a "seed" message, how "motion find")."""
    sp = start_poses_given(args)
    if not sp:
        return
    fres = fres if isinstance(fres, dict) else {}
    for n in names:
        if n not in sp:
            continue
        x, y, th = sp[n]
        coord.__dict__.setdefault("facing_at", {})[n] = (x, y)
        coord.__dict__.setdefault("motion_found", {})[n] = (x, y, th)
        if th is not None:
            fres.setdefault("headings", {})[n] = float(th)
            for k in ("unknown", "unsure"):
                if n in (fres.get(k) or []):
                    fres[k].remove(n)
        r = (fres.get("robots") or {}).get(n)
        if isinstance(r, dict):
            r.setdefault("meas", {}).update(heading=th, how="motion find (no tag that reads)", sure=th is not None)
        warn(coord, "%s has no roof tag that reads: it starts from where the motion finding put it (%.1f, %.1f%s) and "
                    "is followed by motion, lock-on and its own odometry" % (
                        n, x, y, ", facing %.0f deg" % th if th is not None else ""))
    dest = getattr(coord, "pose_dest", None)
    if dest is not None:
        for _k in range(SEED_SENDS):
            for n, (x, y, th) in sp.items():
                msg = {"type": "seed", "robot": n, "x": x, "y": y, "th": th, "how": "motion find",
                       "t": round(now(), 3)}
                try:
                    coord.pose_sock.sendto(json.dumps(msg).encode(), dest)
                except OSError:
                    pass
            time.sleep(0.1)
    coord.log("motion_found_start", robots={n: list(v) for n, v in sp.items()})


def after_facing(coord, fres, names):
    """After a facing check: keep where each robot's lights are (the calibration's roll caps use it) and tell the
    camera bridge (cam_bridge.on_seed), which otherwise looks for each robot round its map start spot only and never
    finds one put down elsewhere (2026-10-30 09:52: 'No camera fix for Left'). Robots with a pose of their own are
    followed by it; the bridge ignores their seeds."""
    fp = facing_poses(fres, names)
    coord.facing_at = dict(getattr(coord, "facing_at", None) or {}, **fp)
    dest = getattr(coord, "pose_dest", None)
    if dest is None or not fp:
        return fp
    for k in range(SEED_SENDS):
        for n, (x, y, h, how, hh) in fp.items():
            msg = {"type": "seed", "robot": n, "x": x, "y": y, "th": hh, "how": how, "t": round(now(), 3)}
            try:
                coord.pose_sock.sendto(json.dumps(msg).encode(), dest)
            except OSError:
                pass
        if k < SEED_SENDS - 1:
            time.sleep(0.1)
    coord.log("bridge_seed", robots={n: list(v[:2]) + [v[4]] for n, v in fp.items()})
    starts = (coord.plan.get("robots") or {})
    for n, (x, y, h, how, hh) in fp.items():
        st = (starts.get(n) or {}).get("start")
        if st:
            d = math.hypot(x - st[0], y - st[1])
            if d > 3.0:
                print("  %s: its lights are at (%.1f, %.1f), %.1f cm from its start spot: the camera bridge follows it "
                      "from there" % (n, x, y, d))
    return fp


CRAWL_CM = 3.0            # the watched crawl that finds which way a robot faces when its lights could not say (Nick
                          # 10:08: "move 2-3 cm, see which way the camera sees it go"). Chosen: over the camera's ~1 cm
                          # standing wobble, short enough for any spot 9 cm in from an edge. DOE: 2 / 3 / 5 cm against
                          # the heading's error (the facing check as truth) on the board
CRAWL_MIN_CM = 1.5        # the camera must see it move at least this far for a heading
CRAWL_CMS = 5.0
CRAWL_GUARD_S = 1.0       # (e15) after a crawl that measured a tag's turn, the camera guard waits this long: the camera's
                          # lag (~0.3 s) plus the pose history it compares with (pose_at). No DOE: the pictures from
                          # before the new heading are not a disagreement


def tag_file_info(coord, n):
    """(e15) Robot n's roof tag id and the turn the run's tags file gives it (what the tag bridge already adds)."""
    cfg = getattr(coord, "_tags_cfg", None)
    if cfg is None:
        cfg = {}
        p = getattr(coord.args, "facing_tags", "") or ""
        if p:
            try:
                with open(p) as f:
                    cfg = json.load(f)
            except (OSError, ValueError):
                cfg = {}
        coord._tags_cfg = cfg
    tid = (cfg.get("ids") or {}).get(n)
    if tid is None:
        return None, 0.0
    yo = cfg.get("yaw_off") or {}
    return tid, float(yo.get(str(tid), yo.get(tid, 0.0)) or 0.0)


def remember_tag_turn(coord, n, how):
    """(e15) The robot's tag turn as this run now knows it (the tags file's + its yaw_fix) into the store
    (--tag-heading-store): the evidence wins over what was remembered."""
    path = getattr(coord.args, "tag_heading_store", "") or ""
    tid, foff = tag_file_info(coord, n)
    if not path or tid is None:
        return None
    import tag_heading_store as THS
    b = coord.bots[n]
    total = THS.wrap180(foff + float(getattr(b, "yaw_fix", 0.0) or 0.0))
    try:
        r = THS.record(path, tid, total, how, robot=n)
    except OSError as e:
        coord.log("tag_heading_memory", robot=n, tag=tid, error=str(e))
        return None
    coord.log("tag_heading_memory", robot=n, tag=tid, offset_deg=total, result=r["result"], old=r["old"], how=how)
    if r.get("words"):
        print("  " + r["words"])
    return r


def learn_turn_from_crawl(coord, n, fres, h):
    """(e15) The watched crawl showed which way robot n faces (h, from the camera's displacement): its tag's turn is
    the crawl's heading against the tag's RAW heading (the facing check's heading less the turn it had added), snapped
    to a quarter turn when near one. Sets yaw_fix (on top of the tags file's turn), marks it measured, remembers it."""
    import observe as OB
    meas = (((fres or {}).get("robots") or {}).get(n) or {}).get("meas") or {}
    if meas.get("heading") is None:
        return None
    raw = OB.wrap180(float(meas["heading"]) - float(meas.get("tag_off") or 0.0))
    total = OB.snap(OB.wrap180(float(h) - raw))
    tid, foff = tag_file_info(coord, n)
    b = coord.bots[n]
    old = OB.wrap180(foff + float(getattr(b, "yaw_fix", 0.0) or 0.0))
    b.yaw_fix = OB.wrap180(total - foff)
    b.yaw_known = True
    coord.log("tag_turn_from_crawl", robot=n, tag=tid, crawl_heading=round(float(h), 1), tag_raw=round(raw, 1),
              offset_deg=total, before=old)
    print("  %s: TAG TURN MEASURED by the crawl: its roof tag reads %.0f deg, it moved at %.0f deg: tag %s is turned "
          "%+.0f deg on the robot (was taken as %+.0f); every heading of it corrected from now on (observed)" % (
              n, raw, h, tid, total, old))
    remember_tag_turn(coord, n, "run_plan's start crawl")
    return total


def crawl_heading(coord, link, args, n, pos, guess, rect):
    """Which way robot n faces, from a short drive the camera watches: it is given the pose (pos, guess) and drives
    CRAWL_CM forwards (the follower drives straight ahead whatever the guess); the camera's displacement is its
    heading. A forward crawl the edge guard refuses becomes a crawl backwards (only 9 cm or more from every edge).
    Returns (heading, (x, y)) or None."""
    import straighten as ST
    for rev in (False, True):
        if rev and ST.edge_dist(rect, pos[0], pos[1]) < ST.SPIN_EDGE_CM + 2.0:
            break
        try:
            set_poses(coord, link, {n: (pos[0], pos[1], guess)})
        except SystemExit:
            return None
        d = -CRAWL_CM if rev else CRAWL_CM
        to = (pos[0] + d * math.cos(math.radians(guess)), pos[1] + d * math.sin(math.radians(guess)))
        step = {"type": "drive", "from": [round(pos[0], 2), round(pos[1], 2), round(guess, 2)],
                "to": [round(to[0], 2), round(to[1], 2), round(guess, 2)], "speed": CRAWL_CMS, "reverse": rev,
                "front_min_cm": None, "phase": "straighten", "why": "a short crawl: which way does it face?"}
        plan_st = {"robots": {n: {"steps": [step]}}, "after": {}}
        names0 = coord.names
        coord.names = [n]
        try:
            ok, words = coord.run_correction(plan_st, 15.0)
        finally:
            coord.names = names0
        coord.abort = None
        time.sleep(0.6)
        got, _ = camera_positions(coord, [n], 3.0, soft=True)
        if n in got:
            dx, dy = got[n][0] - pos[0], got[n][1] - pos[1]
            if math.hypot(dx, dy) >= CRAWL_MIN_CM:
                h = math.degrees(math.atan2(dy, dx)) + (180.0 if rev else 0.0)
                h = (h + 180.0) % 360.0 - 180.0
                coord.log("crawl_heading", robot=n, heading=round(h, 1), moved_cm=round(math.hypot(dx, dy), 2),
                          reverse=rev, guess=guess)
                print("  %s: the crawl shows it faces %.0f deg (it moved %.1f cm %s)" % (
                    n, h, math.hypot(dx, dy), "backwards" if rev else "forwards"))
                return h, got[n]
            pos = got[n]
        print("  %s: the %s crawl did not move it far enough to see (%s)" % (n, "backward" if rev else "forward",
                                                                            words))
    return None


def straighten_start(coord, link, args, source, names, fres, auto=False):
    """Monday's start: each robot's measured pose (camera position, the facing check's heading) against its start pose
    in the plan. Off: it turns on the spot, or drives to its spot and turns (straighten.py), the camera watching; then
    the facing check again. auto (the self-correcting start, Nick 30 Oct 10:08): nothing here stops the start: a robot
    the camera cannot find is placed by its lights, one whose heading is not known crawls to show it, one that cannot be
    corrected safely is held (the others start), and robots still off after the corrections start from where they
    stand (the plan made again from there) with a WARNING."""
    import straighten as ST
    import facing as FC
    f = coord.m["floor"]
    rect = (f["x_min"], f["y_min"], f["x_max"], f["y_max"])
    starts = {n: tuple(float(v) for v in coord.plan["robots"][n]["start"][:3]) for n in names}
    heads = dict(fres.get("headings") or {})
    check_crawl = set((fres.get("unknown") or []) + (fres.get("unsure") or []))
    unverified = set(fres.get("unverified") or [])      # (e15) its tag's turn not verified: crawl before any move
    crawled = set()
    record = coord.straighten_record = {"tries": [], "result": None}
    tries = 0
    max_tries = ST.TRIES + (1 if auto else 0)
    stay = set()              # robots that start from where they stand (their start spot cannot be reached safely)
    print("Start straightening: each robot's measured pose against its start pose")
    meas = {}
    while True:
        names = live_names(coord, names)
        if not names:
            record["result"] = "every robot held"
            return
        pos, side_only = camera_positions(coord, names, args.fix_wait_s, soft=auto)
        for n in names:
            if n not in pos:
                lit = (getattr(coord, "facing_at", None) or {}).get(n)
                if lit is not None:
                    warn(coord, "no camera fix for %s: its position is taken from %s (%.1f, %.1f)"
                         % (n, "where the motion finding put it" if n in (getattr(coord, "motion_found", None) or {})
                            else "its lights", lit[0], lit[1]))
                    pos[n], side_only[n] = (lit[0], lit[1]), False
                else:
                    hold_at_start(coord, link, n, "no camera and none of its lights find it on the board")
        names = live_names(coord, names)
        for n in names:
            if n not in heads:
                b = coord.bots[n]
                if b.pose is not None and tries:
                    heads[n] = b.pose[2]           # its own gyro after the correction (the crawl set it)
                else:
                    heads[n] = starts[n][2]
                    check_crawl.add(n)
        for n in sorted(check_crawl & set(names)):
            warn(coord, "%s: which way it faces is checked by a short watched crawl (%.0f cm)" % (n, CRAWL_CM))
            got = crawl_heading(coord, link, args, n, pos[n], heads[n], rect)
            if got is None:
                hold_at_start(coord, link, n, "which way it faces is not known (lights not read, and the crawl did "
                                              "not show it)")
                continue
            heads[n], pos[n] = got[0], got[1]
        check_crawl = set()
        names = live_names(coord, names)
        if not names:
            record["result"] = "every robot held"
            return
        meas = {n: (pos[n][0], pos[n][1], heads[n]) for n in names}
        try:
            set_poses(coord, link, meas)
        except SystemExit as e:
            if not auto:
                raise
            warn(coord, "%s: its pose sent again" % e)
            try:
                set_poses(coord, link, meas)
            except SystemExit:
                for n in names:
                    b = coord.bots[n]
                    if b.pose is None or b.pose_t < (b.fposed_t or 0):
                        hold_at_start(coord, link, n, "it did not take its pose (fpose)")
                continue
        for n in names:
            coord.bots[n].fix = None
        entry = {"measured": {n: [round(v, 2) for v in meas[n]] for n in names},
                 "side_only": side_only, "start": {n: list(starts[n]) for n in names}}
        record["tries"].append(entry)
        if tries == 0:
            for n in names:
                d = math.hypot(meas[n][0] - starts[n][0], meas[n][1] - starts[n][1])
                coord.start_checks.setdefault(n, {}).update(place_cm=round(d, 2), heading_deg=round(abs(ST.wrap180(
                    meas[n][2] - starts[n][2])), 2), heading_rule="measured (facing check); straightened if off")
        else:
            res = [ST.recheck(n, meas[n], starts[n], side_only.get(n, False)) for n in names]
            for ok, w in res:
                print("  " + w)
            entry["recheck"] = [w for _, w in res]
            if all(ok for ok, _ in res):
                break
        st_all = {n: starts[n] for n in names}
        # robots held at the start stand where they are: every correction keeps the 8 cm net from them (30 Oct
        # simulator: Left's correction passed 3.5 cm from Right, held, before this)
        stand = {}
        for n, _why in (getattr(coord, "start_aside", None) or {}).items():
            b = coord.bots.get(n)
            p_ = (getattr(coord, "facing_at", None) or {}).get(n) or (getattr(b, "last_cam", None) if b else None)
            if p_:
                stand[n] = (float(p_[0]), float(p_[1]), float(p_[2] if len(p_) > 2 and p_[2] is not None else 0.0))
        meas_all = dict(meas, **stand)
        st_all = dict(st_all, **stand)
        try:
            pl = ST.plan_all(meas_all, st_all, rect, dict(side_only, **{n: False for n in stand}))
            for n in stand:
                pl["robots"].pop(n, None)
                pl["after"].pop(n, None)
        except ST.Refuse as e:
            if not auto:
                record["result"] = "refused: %s" % e
                raise SystemExit("Start straightening: %s" % e)
            # hold one robot where it stands (the last named that needs a correction) and plan the others round it
            pl = None
            for n in reversed(names):
                st2 = dict(st_all, **{n: meas[n]})            # it stays: its start "is" where it stands
                try:
                    pl = ST.plan_all(meas_all, st2, rect, dict(side_only, **{m: False for m in stand}))
                    for m in stand:
                        pl["robots"].pop(m, None)
                        pl["after"].pop(m, None)
                except ST.Refuse:
                    continue
                # it starts from where it stands (the plan is made again from there, the held robots as obstacles):
                # its start spot is taken, or no safe way leads there
                warn(coord, "%s cannot reach its start spot safely (%s): it starts from where it stands (the plan made "
                            "again from there)" % (n, e))
                stay.add(n)
                starts[n] = tuple(meas[n])
                pl["robots"][n] = {"steps": [], "words": [], "path": None, "need": False}
                break
            if pl is None:
                for n in names:
                    hold_at_start(coord, link, n, "no safe correction (%s)" % e)
                record["result"] = "refused: %s" % e
                return
        need = [n for n in names if pl["robots"][n]["steps"] and n not in coord.start_aside]
        # (e16, 6b, Claude's suggestion, Nick not asked yet) every robot whose tag turn is not verified is crawled, even
        # one that needs no straightening: a robot with a 180 deg tag put down backwards looks straight on its raw
        # heading (e15's sweep 2). MIRLT_CRAWL_UNKNOWN=need: only those that need straightening (e15)
        pool_ = need if os.environ.get("MIRLT_CRAWL_UNKNOWN", "all") == "need" else \
            [n for n in names if n not in coord.start_aside]
        first = [n for n in pool_ if n in unverified and n not in crawled]
        if first:
            # (e15, school 10 Nov 18:54) never straightened on an unverified raw tag heading: which way it faces is
            # measured first by the watched crawl, its tag's turn learned from it, then everything is planned again
            for n in first:
                crawled.add(n)
                warn(coord, "%s: its roof tag's turn is not verified: a short watched crawl (%.0f cm) shows which way it "
                            "faces before anything turns it" % (n, CRAWL_CM))
                got = crawl_heading(coord, link, args, n, pos[n], heads[n], rect)
                if got is None:
                    got = crawl_heading(coord, link, args, n, pos[n], heads[n], rect)     # measured again
                if got is None:
                    hold_at_start(coord, link, n, "which way it faces is not known (its tag's turn is not verified, "
                                                  "and two crawls did not show it)")
                    continue
                learn_turn_from_crawl(coord, n, fres, got[0])
                heads[n], pos[n] = got[0], got[1]
                unverified.discard(n)
                # its own pose dead-reckoned the crawl along the unverified guess: set at once from the crawl, and the
                # camera guard's half-second of pictures from before (taken at the old heading) not counted
                b_ = coord.bots[n]
                b_.guard_since = None
                b_.own_until = max(getattr(b_, "own_until", 0.0), now() + CRAWL_GUARD_S)
                try:
                    set_poses(coord, link, {n: (pos[n][0], pos[n][1], heads[n])})
                except SystemExit as e:
                    warn(coord, "%s: its pose after the crawl not taken yet (%s): set again before it moves" % (n, e))
            continue
        for w in pl["words"]:
            print("  " + w)
        if not need:
            if tries:
                print("  every robot is within its re-check allowance: ready")
            break
        if tries >= max_tries:
            record["result"] = "still off after %d corrections: %s" % (tries, ", ".join(need))
            if not auto:
                raise SystemExit("Start straightening: %s still off its start pose after %d corrections (above). "
                                 "Stopped: put %s on its start spot facing its start direction and start again."
                                 % (", ".join(need), tries, "it" if len(need) == 1 else "them"))
            warn(coord, "%s still off its start pose after %d corrections: %s from where %s stand%s (the plan made "
                        "again from there)" % (", ".join(need), tries, "they start" if len(need) > 1 else "it starts",
                                               "they" if len(need) > 1 else "it", "" if len(need) > 1 else "s"))
            start_where_they_stand(coord, link, names, meas)
            break
        tries += 1
        entry["steps"] = {n: [st.get("why") for st in pl["robots"][n]["steps"]] for n in need}
        timeout = 15.0 + sum(ST.est_s(pl["robots"][n]["steps"]) for n in need)
        names0 = coord.names
        coord.names = names
        try:
            ok, words = coord.run_correction(pl, timeout)
        finally:
            coord.names = names0
        entry["run"] = words
        for n in need:
            r = pl["robots"][n]
            w = ST.done_words(n, r, rect, starts[n][2]) if ok else \
                "%s: the correction did not finish (%s)" % (n, words)
            print("  " + w)
            entry.setdefault("words", []).append(w)
        if not ok and coord.abort:
            if not auto:
                record["result"] = "stopped: %s" % coord.abort
                raise SystemExit("Start straightening stopped: %s" % coord.abort)
            warn(coord, "the correction was stopped (%s): measured again and tried again" % coord.abort)
            coord.abort = None
        if not ok and tries >= max_tries and not auto:
            record["result"] = "correction failed twice: %s" % words
            raise SystemExit("Start straightening: the correction failed twice (%s). Stopped." % words)
        # re-check: the facing check again (the follow runs end for it), then the follow runs again
        own = {n: coord.bots[n].pose for n in names if coord.bots[n].pose is not None}   # (kept: restart clears it)
        restart_follow(coord, link, names)
        fr = FC.run_plan_check(coord, link, args, names, straighten=True, tag="facing-recheck-%d" % tries)
        after_facing(coord, fr, names)
        heads = dict(fr.get("headings") or {})
        fres = fr                                        # (e15) its raw tag headings for a crawl's measurement
        unverified |= set(fr.get("unverified") or []) - crawled
        for n in names:
            if n not in heads and n in own:
                # its lights still not read: its own gyro heading after the correction (the crawl found it) is used,
                # not a new crawl (each crawl leaves it 3 cm off its spot again)
                heads[n] = own[n][2]
        check_crawl = set()
        follow_robust(coord, link, args, names, source, auto=auto)
        if not wait_for(lambda: all(coord.bots[n].follow for n in names), 8, coord):
            quiet = [n for n in names if not coord.bots[n].follow]
            if not auto:
                raise SystemExit("No FOLLOW records after the re-check from: %s" % ", ".join(quiet))
            for n in quiet:
                hold_at_start(coord, link, n, "no FOLLOW records after the re-check")
    record["result"] = "ready after %d correction(s)" % tries
    if stay:
        record["result"] += "; from where they stand: %s" % ", ".join(sorted(stay))
        start_where_they_stand(coord, link, live_names(coord, names), {n: meas[n] for n in live_names(coord, names)
                                                                       if n in meas})
    coord.log("straighten", **record)
    for n in live_names(coord, names):
        if n not in meas:
            continue
        b = coord.bots[n]
        b.fix = None
        b.last_cam = [round(meas[n][0], 2), round(meas[n][1], 2), round(meas[n][2], 1)]
        print("  %s: ready at (%.1f, %.1f) facing %.0f deg" % (n, *meas[n]))


def start_where_they_stand(coord, link, names, meas):
    """Robots still off after the corrections start from where they stand: the plan made again from there (as Mission
    Control's 'start where they stand'); if no plan comes, the plan as it is (the follower drives from where each
    robot is to its first step's end)."""
    poses = {n: meas[n] for n in names if n in meas}
    for n in (getattr(coord, "start_aside", None) or {}):
        p_ = (getattr(coord, "facing_at", None) or {}).get(n)
        if p_ is not None:                       # a held robot is planned from where it stands (then retired there)
            poses[n] = (p_[0], p_[1], p_[2] if p_[2] is not None else (p_[4] if p_[4] is not None else 0.0))
            coord.bots[n].last_cam = [p_[0], p_[1], poses[n][2]]
    try:
        replan_from(coord, poses, "the start's corrections did not reach the start spots")
    except Exception as e:      # noqa: BLE001
        warn(coord, "no plan from where they stand (%s): the plan as made; each robot drives from where it is" % e)


def gap_lock(args, n_robots):
    """The 8 cm lock (2026-10-29): a run with 2 or more robots has its safety net at 8 cm at least (re-plans are made at
    it, and the start refuses a plan under it). Under it only in the simulator's safety-net DOE (--dry-run
    --below-net-doe)."""
    if n_robots < 2 or args.min_gap_cm >= EC.MIN_GAP_CM:
        return
    if args.below_net_doe and args.dry_run:
        if not getattr(args, "_doe_said", False):
            args._doe_said = True
            print("SIMULATOR DOE: safety net %.1f cm, under the %.0f cm net (dry run only)" % (args.min_gap_cm,
                                                                                          EC.MIN_GAP_CM))
        return
    print("8 cm LOCK: --min-gap-cm %.1f raised to %.1f cm (no 2-robot run under the safety net%s)"
          % (args.min_gap_cm, EC.MIN_GAP_CM, "; --below-net-doe is for --dry-run only" if args.below_net_doe else ""))
    args.min_gap_cm = EC.MIN_GAP_CM


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True)
    ap.add_argument("--plan", help="plan.json (default: plan now from the map)")
    ap.add_argument("--session", help="log folder (default logs/<date-time>-plan)")
    ap.add_argument("--fixes", help="position source: none, udp[:PORT], replay:FILE (default none; udp with --dry-run)")
    ap.add_argument("--robot", action="append", default=[], help="robot IP, if not found automatically")
    ap.add_argument("--alias", action="append", default=[], metavar="ANSWERS_AS=RUN_NAME",
                    help="(5 Nov 2026) a robot that answers as AL-1DC0 is this run's Alpha (names in the run only, "
                         "the site's tags.rename_to_slot false; mission_run passes the wiggle's pairs)")
    ap.add_argument("--ahead", type=int, default=2, help="steps a robot may hold beyond the one it runs (default 2)")
    ap.add_argument("--resend-s", type=float, default=0.3, help="resend an unacknowledged step after this")
    ap.add_argument("--fix-hz", type=float, default=10.0, help="most camera fixes per robot per second")
    ap.add_argument("--fix-wait-s", type=float, default=10.0,
                    help="at the start, wait this long for a camera fix of every robot (the live link needs about "
                         "6 flashes to name a robot: 2-4 s)")
    ap.add_argument("--retry-s", type=float, default=3.0, help="a robot left without a route gets a new try this often")
    ap.add_argument("--improve-after-s", type=float, default=1.0)
    ap.add_argument("--improve-budget-s", type=float, default=6.0)
    ap.add_argument("--max-replans", type=int, default=30, help="give up (stop everyone) after this many re-plans")
    ap.add_argument("--dedupe-points", choices=("on", "off"), default="on",
                    help="duplicated points fix (3 Nov): a point is done once; a handed-over point is never listed "
                         "twice (only the newest hand-over counts). off: the old way")
    ap.add_argument("--all-held-max", type=int, default=None,
                    help="keep going: the run ends the Nth time every robot is held at once (default 3; 0 = no cap)")
    ap.add_argument("--all-held-max-s", type=float, default=None,
                    help="keep going: or when every robot is held again this many s after the first time (default "
                         "180; 0 = no cap)")
    ap.add_argument("--gap-fix", choices=("on", "off"), default="on",
                    help="robot gap fix (3 Nov): on (default): a re-plan checks the robots' carried-on steps against "
                         "every obstacle the plan in force was not made with, and a robot held while every robot is "
                         "held is on the map; off: the old way (only the newest obstacle checked)")
    ap.add_argument("--held-pm-max-cm", type=float, default=10.0,
                    help="a held robot seen by no camera: its odometry drift since the last fix is added to the net "
                         "round it, at most this (cm); 0 = off (the old outline). Claude's choice, 3 Nov")
    ap.add_argument("--timeout-s", type=float, default=400.0)
    ap.add_argument("--find-s", type=float, default=20.0)
    ap.add_argument("--place-tol-cm", type=float, default=2.0)
    ap.add_argument("--place-tol-deg", type=float, default=None,
                    help="a fixed start heading tolerance (deg); default: scaled by the first leg's length and risk "
                         "(engine_checks.start_heading_check)")
    ap.add_argument("--facing-check", action="store_true",
                    help="before anything moves, measure which way each robot faces from its lights (both cameras) "
                         "and stop when one is more than --facing-tol-deg off its start heading (facing.py)")
    ap.add_argument("--facing-tol-deg", type=float, default=15.0)
    ap.add_argument("--straighten", action="store_true",
                    help="with --facing-check and camera fixes: a robot off its start pose is turned on the spot or "
                         "driven to its spot (the camera watching), then checked again, instead of stopping the start "
                         "(straighten.py; Monday and Mission Control: mission_run.py passes it)")
    ap.add_argument("--facing-stream", default="127.0.0.1:8000", help="the overhead camera's pictures")
    ap.add_argument("--facing-side-stream", default="", help="the side camera's pictures ('' = overhead alone)")
    ap.add_argument("--facing-calib", default="", help="the overhead camera's calibration")
    ap.add_argument("--facing-side-calib", default="", help="the side camera's calibration")
    ap.add_argument("--facing-tags", default="", help="the AprilTag robots (2026-11-02): a tags JSON (family, sizes, "
                                                      "height, ids); the facing check reads the roof tags, no lights")
    ap.add_argument("--calibrate", action="store_true", help="send calibrate to every robot first")
    ap.add_argument("--recalibrate", choices=("auto", "no"), default="auto",
                    help="auto (default): calibrate a robot that restarted since its calibration today (a new boot ID) "
                         "or says it is not calibrated, then carry on; no: stop and say so")
    ap.add_argument("--replan-from-camera", action="store_true",
                    help="plan again from where the camera has the robots when they are off their start spots "
                         "(Mission Control, Monday's command); without it, a robot off its spot stops the start")
    ap.add_argument("--start-pose", action="append", default=[], metavar="NAME=X,Y,TH",
                    help="(e14) a robot with no roof tag that reads: where the motion finding put it before the run")
    ap.add_argument("--battery-floor", type=int, default=EC.BATTERY_FLOOR_PCT,
                    help="refuse to start a robot under this battery %% (default %(default)s: engine_checks."
                         "battery_floor(), 25 unless MIRLT_MIN_BATTERY is set (never under 5); 0 = off)")
    ap.add_argument("--silent-s", type=float, default=EC.SILENT_S,
                    help="a robot not heard for this long mid-run is out: a fixed obstacle, the others re-plan "
                         "(default %(default)s; 0 = off)")
    ap.add_argument("--min-gap-cm", type=float, default=EC.MIN_GAP_CM,
                    help="refuse any plan bringing two robots closer than this (default %(default)s; 0 = off)")
    ap.add_argument("--obstacle-readings", type=int, default=OBSTACLE_READINGS, choices=(0, 1, 2, 3),
                    help="distance-sensor readings in a row that stop a step (firmware obs_n, 2026-10-28a; 0 = "
                         "leave the robot's own setting; default %(default)s)")
    ap.add_argument("--floor-check", choices=("on", "off"), default="on",
                    help="at start-up each robot stands while its front zones read the floor; its stop limit becomes "
                         "the lowest of 12 cm, its own ceiling (Left 10) and its floor less --floor-margin-cm, not "
                         "under 8 cm (front_stop.py; default %(default)s)")
    ap.add_argument("--floor-margin-cm", type=float, default=FSTOP.FLOOR_MARGIN_CM,
                    help="the floor check: the limit sits this far under the floor's level (default %(default)s)")
    ap.add_argument("--pose-source", choices=("ekf", "blend"), default=None,
                    help="(e09, B2) one filter per robot in charge (ekf) or the one-step blend with the filter beside it (blend, the default after e09)")
    ap.add_argument("--gyro-heading", action="store_true", help="(e09, B3, off by default) heading from the raw gyro")
    ap.add_argument("--attended", action="store_true", help="(e08, A2) a person is with the robots: a robot the run is "
                    "unsure about near an edge is asked about on Mission Control's Start screen (else logged only)")
    ap.add_argument("--edge-tracking", default="lights", choices=["lights", "tags", "none", "motion"],
                    help="(e08, A2) what the camera tracks (its edge bias: lights 5 cm, roof tags 0): mission_run passes it")
    import safety_rules as _SR
    ap.add_argument("--unconfirmed-s", type=float,
                    default=FSTOP.UNCONFIRMED_S if _SR.KEEPOUT_RULE != "permanent" else 0.0,   # (e08, A6) one rule
                    help="a distance-sensor obstacle the camera (or a second stop) did not confirm leaves the map "
                         "after this long, and before a re-plan gives up with no plan (default %(default)s; 0 = "
                         "every one stays, as before)")
    ap.add_argument("--no-rejoin", action="store_true",
                    help="a robot that went silent or restarted stays out for the rest of the run (before "
                         "2026-10-29); default: it rejoins when heard again (same boot) or after a re-check")
    ap.add_argument("--corrections", choices=("auto", "yes", "no"), default="auto",
                    help="send each robot's stored driving corrections (gyro_scale, wheel_scale, gyro_drift_dps: "
                         "e02_fit.PRIOR, corrections.json, today's warm-up); auto = real robots only")
    ap.add_argument("--day-state", help="the day's robot state file (default ~/.config/mirlt/day/<date>.json; dry "
                                        "runs: one in the session folder)")
    ap.add_argument("--yes", action="store_true", help="don't ask before starting")
    ap.add_argument("--send", action="append", default=[], metavar="COMMAND",
                    help="console command for every robot before its follow run starts, e.g. \"set safe_edge 1\" "
                         "(repeatable)")
    ap.add_argument("--home", action="append", default=[], metavar="ROBOT[=DOCK]",
                    help="send ROBOT home: plan its route to a home dock's mouth (map home_docks; nearest free one "
                         "unless =DOCK), then it creeps in until both nose switches click (firmware dock.h; with the "
                         "back attachment, robot.back_bumper_cm in the map: it backs in until both back switches "
                         "click). Repeatable; 'all' = every robot. Also sends 'set sw_on 1'.")
    ap.add_argument("--site-map", help="locked site map (tools/site/setup.py): warn when the robots' front sensors "
                                       "keep disagreeing with a landmark on it (tools/site/change.py)")
    ap.add_argument("--below-net-doe", action="store_true",
                    help="with --dry-run only: let --min-gap-cm stay under the 8 cm safety net (the simulator's "
                         "safety-net DOE); otherwise a 2-robot plan always runs with the net at 8 cm or more")
    ap.add_argument("--no-mirror", action="store_true",
                    help="a mirrored mission (Monday: Left and Right) without the mirror sync: every leg at its planned "
                         "speed, no waits for the partner (the matching points are still measured)")
    ap.add_argument("--turn-pause", action="store_true",
                    help="after each turn on the spot, stand until a fresh camera fix (both cameras', or one when the "
                         "other cannot see it) agrees with the robot's own pose within %.1f cm, at most %.0f s, then "
                         "drive (Monday's mission and Mission Control's runs: mission_run.py passes it)"
                         % (Coordinator.TURN_PAUSE_TOL_CM, Coordinator.TURN_PAUSE_MAX_S))
    ap.add_argument("--cam-guard-cm", type=float, default=Coordinator.CAM_GUARD_CM,
                    help="stop everyone when a robot's own pose and the camera disagree by more than this for 1 s "
                         "(0 = off; default %(default)s)")
    ap.add_argument("--on-problem", choices=("stop", "keep_going"), default="keep_going",
                    help="keep_going (2026-11-03; a site's on_problem, mission_run passes it): a problem that stopped "
                         "everyone (camera and pose disagree for long, out of re-plans, no plan, a robot lost) is said "
                         "once, logged and worked round; only a full card, Stop / Ctrl+C and every robot out end the "
                         "run (and the time limit, stretched to the newest plan's end; the default since e08). stop: as before e08 (a switch)")
    ap.add_argument("--lost-robot-points", choices=("reassign", "skip"), default="reassign",
                    help="with --on-problem keep_going: a robot that is out hands its production points not done to "
                         "the robots still running (reassign, default), or they are listed as not done (skip)")
    ap.add_argument("--stop-lead", action="append", default=[], metavar="NAME=SPEED:CM,...",
                    help="(e07) send this robot's forward drives this much short (its stop lead per speed, from its "
                         "last calibration; mission_run passes them)")
    ap.add_argument("--recover-reserve", choices=["on", "off"], default="on",
                    help="(e07, Nick 6 Nov) on: a lost robot's straight recovery path is reserved in the others' plans "
                         "(they re-plan round it); off: e06's way (only checked against their next 4 steps)")
    ap.add_argument("--surface-marks", choices=["log", "act", "auto", "off"], default="log",
                    help="(e07, Nick 6 Nov) log (default): a line-sensor trip with no drop is logged as data and "
                         "nothing else (not an edge, the line sensors untouched); act (= e06's auto, kept switched "
                         "off): a trip the camera puts well inside the table is a surface mark and that robot's line "
                         "cue is switched off; off: every line trip is an edge (before e06)")
    ap.add_argument("--readmap", default="", help="(11-05h) the mission's readability map at the start "
                    "(observe.ReadMap JSON); learned on during the run, saved as readmap.json")
    ap.add_argument("--bridge-events", default="", help="the tag bridge's events.jsonl: a bumped camera's size counts "
                                                         "in the positions' +- (confidence at every point)")
    ap.add_argument("--cam-fit", default="", help="the camera setup's site_fit.json: each camera's fit error counts in "
                                                  "the positions' +- (read when it appears)")
    ap.add_argument("--poses-to", metavar="HOST:PORT",
                    help="send each robot's own pose (FOLLOW records) as JSON lines here: the camera bridge "
                         "(tools/fleet/cam_bridge.py, default port 5011) predicts who is who from them")
    g = ap.add_argument_group("dry run against the simulator (tools/sim/fleet_sim.py)")
    ap.add_argument("--border-cm", type=float, default=BORDER_CM,
                    help="the robots' own four-sided border (firmware 2026-10-27a), sent when the map has side kinds "
                         "(default %(default)s; 0 = off)")
    g.add_argument("--dry-run", action="store_true")
    g.add_argument("--tag-heading-store", default="", help="(e15) the remembered tag turns (tag_heading_store.py): what "
                   "this run re-learns or measures by its start crawl is recorded there (mission_run passes it)")
    g.add_argument("--brick", help="auto@T or x,y[,angle]@T")
    g.add_argument("--wifi-drop", action="append", default=[], metavar="NAME@T:DUR")
    g.add_argument("--seed", type=int, default=1)
    g.add_argument("--sim-arg", action="append", default=[], help="extra argument for fleet_sim.py (repeatable)")
    args = ap.parse_args(argv)

    m = FM.load(args.map)
    homes = {}
    for h in args.home:
        r, _, d = h.partition("=")
        for name in ([x["name"] for x in m["robots"]] if r == "all" else [r]):
            homes[name] = d or None
    assigned = {}
    if homes:
        if args.plan:
            raise SystemExit("--home plans the route itself: leave out --plan")
        if not m["robot"].get("bumper_cm") and not FM.switches_back(FM.normalize(m)):
            m["robot"]["bumper_cm"] = FM.BUMPER_FITTED_CM
            print("--home: the robots wear the nose bumper: planning with it (%.2f cm in front of the shell); set "
                  "robot.bumper_cm in the map to change it" % FM.BUMPER_FITTED_CM)
        m, assigned = P.with_home(m, homes)
        args.send = list(args.send) + ["set sw_on 1"]
    if FM.switches_back(FM.normalize(m)):
        # the one attachment at the back (Nick, 2026-10-25): board visits and home docks back in (follow steps P, K)
        args.send = list(args.send) + ["set sw_on 1", "set sw_back 1",
                                       "set e02_back_face_cm %.2f" % FM.back_face_cm(FM.normalize(m))]
        print("Switches at the back (map robot.back_bumper_cm): every board visit and home dock backs in")
    gap_lock(args, len([n for n, v in (m.get("tasks") or {}).items() if (v or {}).get("visit")]) or len(m["robots"]))
    if args.plan:
        with open(args.plan, encoding="utf-8") as f:
            plan = json.load(f)
        if plan.get("map_hash") != FM.map_hash(m):
            raise SystemExit("plan.json was made for a different map (plan %s, map %s): plan again."
                             % (plan.get("map_hash"), FM.map_hash(m)))
    else:
        print("Planning...")
        sep = float(((m.get("mission") or {}).get("planner") or {}).get("separation_cm", P.DEFAULT_SETTINGS["separation_cm"]))
        over = {"separation_cm": max(sep, args.min_gap_cm)} if args.min_gap_cm else None
        plan = P.make_plan(m, over=over)
    gap_lock(args, len(plan["robots"]))
    for n in plan["robots"]:
        for st in plan["robots"][n]["steps"]:
            if st["type"] == "approach" and "handover_cm" not in st:
                raise SystemExit("plan.json is from before 2026-10-18b (approach steps without handover_cm): plan again.")
    outdir = args.session or os.path.join("logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") + ("-dryrun" if args.dry_run else "-plan"))
    os.makedirs(outdir, exist_ok=True)
    try:                                        # 2026-10-28: not started on a nearly full card (nothing moved yet)
        import diskguard as DG
        _g = DG.free_gb(outdir)
        if _g is not None and _g < DG.CARD_HARD_GB:
            raise SystemExit("NOT STARTED: the Pi's card has only %.2f GB free (runs need %.1f GB). Nothing moved. "
                             "Free space first: ./run-fleet-pi.sh space" % (_g, DG.CARD_HARD_GB))
    except ImportError:
        pass
    if assigned:
        P.append_home(plan, assigned)
        for r, d in assigned.items():
            print("%s goes home to dock %s" % (r, d["name"]))
    plan_path = os.path.join(outdir, "plan_given.json")
    P.save_plan(plan, plan_path)

    sim = None
    robot_ips = list(args.robot)
    fixes = args.fixes
    if args.dry_run:
        fixes = fixes or "udp"
        robot_ips = robot_ips or ["127.0.0.%d" % (2 + k) for k in range(len(m["robots"]))]
        cmd = [sys.executable, os.path.join(TOOLS, "sim", "fleet_sim.py"), "--map", args.map, "--plan", plan_path,
               "--out", os.path.join(outdir, "sim"), "--seed", str(args.seed), "--quiet"]
        if fixes.startswith("udp"):
            cmd += ["--fixes-to", "127.0.0.1:%s" % (fixes.partition(":")[2] or "5010")]
        else:
            cmd += ["--fix-hz", "0"]
        if args.brick:
            cmd += ["--brick", args.brick]
        for w in args.wifi_drop:
            cmd += ["--wifi-drop", w]
        cmd += args.sim_arg
        if getattr(args, "straighten", False) and "--persist" not in cmd:
            cmd += ["--persist"]
            args.sim_persist = True  # the re-check ends the follow runs and starts them again: the simulator must not end
        print("Starting the simulated robots: " + " ".join(cmd[1:]))
        sim = subprocess.Popen(cmd, stdout=open(os.path.join(outdir, "sim_stdout.txt"), "w"), stderr=subprocess.STDOUT)
        time.sleep(1.5)
    source = make_source(fixes)
    aliases = dict(a_.split("=", 1) for a_ in getattr(args, "alias", []) if "=" in a_)
    if aliases:
        print("Names in this run only (not on the robots): %s" % ", ".join("%s is %s" % kv for kv in aliases.items()))
    link = AC.Link(outdir, robot_ips, aliases=aliases)
    link.session.quiet = True
    args.homes = assigned
    # auto: real robots only. The simulated robots (a dry run, or the ladder's / DOE loop's dry runs, whose robots are on
    # 127.0.0.x) keep factory values: their gyros are true, and the stored 1.075 made every simulated turn 7.5 % too big
    # (ladder dry run, L3 p95 3.2-4.0 cm instead of 0.5-1.0)
    simulated = args.dry_run or (bool(args.robot) and all(ip.startswith("127.") for ip in args.robot))
    args.corrections = (args.corrections == "yes") or (args.corrections == "auto" and not simulated)
    coord = Coordinator(args, m, plan, link, source, outdir)
    coord.day = EC.DayState(args.day_state or (os.path.join(outdir, "day_state.json") if args.dry_run else None))
    coord.pose_dest = None
    if args.poses_to:
        h, _, pt = args.poses_to.rpartition(":")
        coord.pose_dest = (h or "127.0.0.1", int(pt))
        coord.pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    coord.first_plan_id = plan.get("plan_id")
    coord.first_makespan = plan["summary"]["makespan_s"]
    coord.started = now()
    code = 0
    install_signals(coord)
    try:
        start_robots(coord, link, args, source)
        coord.started = now()
        coord.run(args.timeout_s)
    except KeyboardInterrupt:
        print("\nStopping every robot.")
        coord.abort = "interrupted"
    except SystemExit as e:
        print(e)
        coord.abort = coord.__dict__.get("abort") or "start-up failed"
        code = 1
    finally:
        # sends stop to every robot (ends the follow runs) and closes the files; a finished run leaves the robots that
        # are dancing until switched off (Monday's end) dancing
        dancing = [n for n, b in coord.bots.items() if getattr(b, "dancing", False) and not b.out] \
            if coord.__dict__.get("abort") is None and hasattr(coord, "bots") else []
        if dancing and not args.dry_run:
            print("Left dancing until switched off: %s" % ", ".join(dancing))
        link.close(spare=dancing if not args.dry_run else ())
        source.close()
        try:
            coord.trust_save("the run ended: %s" % (coord.__dict__.get("abort") or "done"))   # (e18, 6) at ANY end
        except Exception:  # noqa: BLE001
            pass
        summ = coord.summary() if hasattr(coord, "dockings") else {"result": coord.__dict__.get("abort")}
        if sim is not None:
            if getattr(args, "sim_persist", False):
                sim.terminate()            # (--persist: it reports on SIGTERM)
            try:
                sim.wait(timeout=15)
            except subprocess.TimeoutExpired:
                sim.terminate()
            rp = os.path.join(outdir, "sim", "sim_report.json")
            if os.path.exists(rp):
                with open(rp) as f:
                    summ["simulator"] = json.load(f)
        with open(os.path.join(outdir, "run_plan_summary.json"), "w") as f:
            json.dump(summ, f, indent=1)
        print_summary(summ)
        print("Saved to %s" % outdir)
    if summ.get("result") != "done" and not str(summ.get("result")).startswith("limited:"):
        code = code or 1          # a plan cut short at a plywood edge ("limited: ...") is not a failure
    return code


def install_signals(coord):
    """Mission Control's Pause / Resume (SIGUSR1 / SIGUSR2); the SSH link dropping (SIGHUP) and (e19, audit 17) SIGTERM
    (systemd, `timeout`, Mission Control's Stop by signal) end the run the Ctrl+C way: main's finally sends stop to every
    robot FIRST (link.close), then saves. Before e19 SIGTERM had no handler: the process died with the robots in follow
    mode until their own watchdogs stopped them."""
    try:
        import signal
        if hasattr(signal, "SIGUSR1"):        # Mission Control's Pause / Resume
            signal.signal(signal.SIGUSR1, lambda *_: setattr(coord, "pause_req", True))
            signal.signal(signal.SIGUSR2, lambda *_: setattr(coord, "pause_req", False))
        for nm in ("SIGHUP", "SIGTERM"):      # stop every robot
            if hasattr(signal, nm):
                signal.signal(getattr(signal, nm), lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    except ValueError:
        pass                                  # not the main thread (tests)


def print_summary(s):
    print("\nResult: %s after %.1f s (first plan said %.1f s), %d plan(s)"
          % (s.get("result"), s.get("duration_s", 0.0), s.get("planned_makespan_s") or 0.0, s.get("plans", 0)))
    if "resends" in s:
        print("Steps sent %s; resent %s" % (s["steps_sent"], s["resends"]))
        if s.get("boards_not_visited"):
            print("NOT VISITED: %s" % s["boards_not_visited"])
        print("Camera fixes sent %s; used/ignored on the robots %s; held (the robot had no pose yet) %s"
              % (s["fixes_sent"], s["fixes_used_rejected"], s.get("fixes_held_no_pose")))
    if s.get("warnings"):
        print("WARNINGS (%d; each was handled without stopping the run):" % len(s["warnings"]))
        for w in s["warnings"]:
            print("  - " + w)
    tp = (s.get("turn_pause") or {}).get("robots") or {}
    for n, o in sorted(tp.items()):
        print("Pause after turns, %s: %d pauses added %.1f s (longest %.1f s); both cameras %d, one camera %d, timed "
              "out %d" % (n, o["pauses"], o["added_s"], o["longest_s"], o["both"], o["one_camera"], o["timed_out"]))
    for r in s.get("replans", []):
        print("  re-plan at %.1f s (%s): %.2f s to compute%s" % (r["t_run"], r["source"], r["cpu_s"],
                                                                "" if r.get("kept", True) else ", not kept"))
    if s.get("on_problem") == "keep_going":
        kg = s.get("kept_going") or []
        print("Keep going: %d problem(s) worked round without stopping the run" % len(kg))
    if s.get("points"):
        print("Confidence at each point (\u00b1 = 95 %%; %d points):" % len(s["points"]))
        for q in s["points"]:
            print("  " + q["words"])
    for q in s.get("points_skipped") or []:
        print("  SKIPPED: %s (%s): %s" % (q["point"], q["robot"], q["why"]))
    for q in s.get("points_reassigned") or []:
        print("  REASSIGNED: %s from %s to %s" % (q["point"], q["frm"], q["to"]))
    sim = s.get("simulator")
    if sim:
        print("Simulator truth: closest two robots %.2f cm (%s); closest to a board/brick %.2f cm (%s); contact: %s"
              % (sim["min_body_gap_cm"], sim["min_body_gap_at"], sim["min_static_clearance_cm"], sim["min_static_at"],
                 "YES" if sim["collision"] else "no"))
        if sim.get("true_sides") is not None:
            print("Simulator truth, table edge: closest a body came to an open part %s cm (%s); hung over the edge: %s"
                  % (sim.get("min_open_edge_cm"), sim.get("min_open_edge_at"), ", ".join(sim.get("hung") or []) or "none"))
        for d in sim["dockings"]:
            print("  dock %-6s at %s: front gap %.2f cm (error %+.2f), sideways %+.2f cm, heading %+.1f deg  [%s]"
                  % (d["robot"], d["board"], d["front_gap_cm"], d["gap_error_cm"], d["lateral_cm"], d["heading_err_deg"], d["info"]))
        for e in sim.get("events", []):
            print("  sim event: %s" % e)


if __name__ == "__main__":
    sys.exit(main())
