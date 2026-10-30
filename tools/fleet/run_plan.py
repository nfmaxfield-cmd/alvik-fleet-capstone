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
import argparse
import csv
import datetime as dt
import json
import math
import os
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
                if isinstance(msg, dict) and msg.get("type") in ("fix", "obstacle"):
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
        self.names = list(plan["robots"])
        self.bots = {n: Bot(n) for n in self.names}
        self.q = queue.Queue()
        link.session.listeners.append(self._listen)
        self.log_f = open(os.path.join(outdir, "coordinator.jsonl"), "a")
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
                st["pose_started"] = (x, y, th)
                stp = self.steps(b.name)[i]
                if stp["type"] == "dance" and not stp.get("dance_ms"):
                    # dancing until switched off (Monday's end): its part of the plan is done once it starts
                    st["status"] = "done"
                    st["t_done"] = t
                    st["info"] = "dancing until switched off"
                    b.dancing = True
                    self.say("%s is dancing (until it is switched off)" % b.name)
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

    def cam_guard(self):
        """Stop everyone when a robot's own pose and the camera disagree for long (its fixes are being refused)."""
        lim = getattr(self.args, "cam_guard_cm", self.CAM_GUARD_CM)
        if not lim or not self.source.live:
            return
        t = now()
        for b in self.bots.values():
            cam, ct = getattr(b, "last_cam", None), getattr(b, "cam_t", None)
            if b.out or t < b.own_until:      # out of the run; or its own pose stands after a barrier find (10 s)
                b.guard_since = None
                continue
            if cam is None or b.pose is None or ct is None or t - ct > 0.3 or t - b.pose_t > 0.3:
                b.guard_since = None
                continue
            d = math.hypot(cam[0] - b.pose[0], cam[1] - b.pose[1])
            if d <= lim:
                b.guard_since = None
                continue
            if getattr(b, "guard_since", None) is None:
                b.guard_since = t
                self.log("cam_guard_start", robot=b.name, d_cm=round(d, 2), cam=cam, own=list(b.pose))
            elif t - b.guard_since >= self.CAM_GUARD_S and getattr(self, "self_correct", False) and not \
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
                self.control(b, "halt")
                self.send(b.name, "fpose %.2f %.2f %.2f 1.0 3.0" % (cam[0], cam[1], th))
                b.pose, b.pose_t = (cam[0], cam[1], th), t
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
            b.pose, b.pose_t = (x, y, th), t
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
                st.update(status="sent", t_sent=t, text=step_command(sid, stp, wms), sends=1,
                          speed_sent=m_v if m_v is not None else stp.get("speed"))
                self.send(n, st["text"])
                self.log("step_sent", robot=n, id=sid, i=i, plan=self.plan_no, text=st["text"])
                break           # the next one goes once this one is acked

    def resend(self):
        t = now()
        for b in self.bots.values():
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
        for ev in self.source.poll():
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
            if b is not None:
                b.fix = ev
                self.cam_seen(b, ev)
                # for the pause after a turn: (frame time, arrived, x, y, which camera); the camera bridge says
                # "cam_bridge:both" / ":overhead" / ":side" / ":lights"; the simulator's direct fixes say nothing
                # (truth plus noise: counted as both)
                src = str(ev.get("src") or "both").split(":")[-1]
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

    TRACK_EVERY_S = 0.2       # cam_track.csv: one camera pose per robot this often (the mapping view's lines)
    TRACK_LOST_S = 1.0        # chosen: no camera pose for this long = the tracker lost the robot (10 fixes missed at
                              # 10 a second); the warm-up shows those moments to Nick. DOE: not needed (it only picks
                              # frames for a person to look at)

    def cam_seen(self, b, ev):
        t = now()
        b.last_cam = [round(float(ev["x"]), 2), round(float(ev["y"]), 2),
                      None if ev.get("th") is None else round(float(ev["th"]), 1)]
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
        if len(self.replans) >= self.args.max_replans:
            self.say("out of re-plans (%d); stopping everyone" % len(self.replans))
            self.halt_all()
            self.abort = "out of re-plans"
            return
        t_event = now()
        states = self.states(cause)
        over = dict(self.plan["settings"])
        gate = getattr(self.args, "min_gap_cm", EC.MIN_GAP_CM) or 0.0
        over["separation_cm"] = max(float(over.get("separation_cm", 0.0)), gate)    # re-plans are made at the gate
        if budget is not None:
            over["replan_budget_s"] = budget
        if improving:
            over["replan_first_safe"] = False     # the background search for a shorter plan: the full search
        try:
            dec = P.obstacle_decide(self.m, self.plan, states, [ob] if ob else [], over=over,
                                    t_now=t_event - self.plan_start)
        except Exception as e:      # noqa: BLE001  (never leave robots driving on a planner bug)
            self.say("re-plan failed (%s): stopping everyone" % e)
            self.halt_all()
            self.abort = "re-plan failed"
            return
        if not improving:
            for n in self.names:
                self.act_on_decision(n, dec["keep"][n], dec["first_step"][n], states[n]["in_progress"])
        self.limit = {n: dec["first_step"][n] + dec["keep"][n] for n in self.names}
        job = dict(dec=dec, ob=ob, cause=cause, improving=improving, t_event=t_event, over=over, states=states,
                   result=None, error=None, cpu=None)

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
        self.plan_no += 1
        self.plan_start = t_event
        self.save_plan()
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
        # a true offset (each corner rounded by a disc of that radius, 16 points), not a push out from the middle,
        # which along the sides gave only 70 % of it
        a_ = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        disc = np.c_[np.cos(a_), np.sin(a_)] * (grow / math.cos(math.pi / 16))
        hull = G.convex_hull((hull[:, None, :] + disc[None, :, :]).reshape(-1, 2))
        return {"id": "out-%s" % b.name, "shape": "polygon", "points": hull.round(2).tolist(), "source": "robot_out",
                "time": FM.now_iso(), "gap_cm": round(grow, 2)}

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

    def retire(self, b, why, kind="out"):
        """Nick (2026-10-27 16:43): a robot that dies mid-run is a brick; the others carry on. kind "silent" or
        "restarted" (Nick, 2026-10-29 16:2x): it is held, not out for good: when it is heard again it rejoins
        (rejoin_watch). What it needs for that is kept: its part of the plan and its steps' states."""
        if b.out:
            return
        import copy
        b.out = why
        b.out_kind = kind
        b.out_entry = copy.deepcopy(self.plan["robots"].get(b.name)) if b.name in self.plan["robots"] else None
        b.out_ids, b.out_stat = list(b.ids), [dict(x) for x in b.stat]
        b.back_t = None
        b.out_t = now()
        ob = self.out_obstacle(b)
        if self.rejoinable(b):
            self.say("%s is held: %s. It is a fixed obstacle where it stopped until it is heard again (then it rejoins); "
                     "the others re-plan and carry on" % (b.name, why))
        else:
            self.say("%s is out: %s. It is now a fixed obstacle where it stopped; the others re-plan and carry on"
                     % (b.name, why))
        self.log("robot_out", robot=b.name, why=why, obstacle=ob, pose=list(b.pose) if b.pose else None)
        self.control(b, "halt")              # if it can still hear: stop at once
        self.names = [n for n in self.names if n != b.name]
        self.outs = getattr(self, "outs", []) + [dict(robot=b.name, why=why, t_run=round(now() - self.t_run, 2)
                                                      if self.t_run_started() else None)]
        if not self.names:
            if self.rejoinable(b):
                self.say("every robot is held: waiting up to %.0f s for %s to be heard again" % (self.REJOIN_WAIT_S,
                                                                                               b.name))
                self.plan = self.without(self.plan, [b.name])
                return
            self.abort = "every robot is out (%s)" % why
            self.say(self.abort)
            return
        self.plan = self.without(self.plan, [b.name])
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
        b.out_t = now()
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
            elif not getattr(b, "back_said", False):
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
        if not self.names and not any(b.out and self.rejoinable(b) and t - getattr(b, "out_t", 0.0) < self.REJOIN_WAIT_S
                                      for b in self.bots.values()):
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
        cam = b.last_cam
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

    LIVE_EVERY_S = 0.5    # live.json for Mission Control's Run screen. No DOE: a display rate

    def write_live(self, final=False):
        """live.json: each robot now (camera spot, own pose, battery, what it is doing, its last step's result,
        how far its middle is from the nearest open side), for Mission Control's Run screen."""
        t = now()
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
                      "held": bool(b.out) and self.rejoinable(b), "stop_cm": getattr(b, "stop_cm", None)}
        live = {"t": round(t, 2), "t_run": round(t - self.t_run, 1) if self.t_run_started() else None,
                "plan_no": self.plan_no, "replans": len(self.replans), "result": self.abort or ("done" if final else None),
                "robots": rob, "obstacles": self.live_obstacles()}
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
                self.say("%s's battery is at %d %%, under the %d %% floor: it finishes this run; charge it before the "
                         "next" % (n, b.battery, floor))
                self.log("battery_low", robot=n, battery=b.battery)

    def request_replan(self, why):
        if self.pending_replan is None:
            self.pending_replan = (why, now() + 0.3)

    def on_failed(self, b, i, info, pose, front):
        if getattr(self, "straightening", False):
            return                  # the start's correction: a failed step is that correction failing (straighten)
        stp = self.steps(b.name)[i]
        self.log("step_failed", robot=b.name, i=i, info=info)
        if b.ended or info.startswith(("robot lifted", "stop command", "cancel button")):
            return                                      # the run ended: handled in the main loop
        if info.startswith("halted"):
            return                                      # we stopped it for a re-plan
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
            if b.out or b.pose is None:
                continue
            ct = getattr(b, "cam_t", None)
            bl = getattr(b, "blind", None)
            if ct is not None and t - ct <= self.BLIND_AFTER_S:
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
            bl["dist"] += math.hypot(b.pose[0] - bl["last"][0], b.pose[1] - bl["last"][1])
            bl["last"] = list(b.pose[:2])
            if bl["dist"] >= 5.0 and not bl["said"]:
                bl["said"] = True
                self.say("%s: no camera sees it: driving on its own wheels and gyro (at most %.0f cm)"
                         % (b.name, self.BLIND_MAX_CM))
            body_edge = min(b.pose[0] - rect[0], rect[2] - b.pose[0], b.pose[1] - rect[1], rect[3] - b.pose[1]) - 4.75
            err = self.BLIND_ERR_BASE_CM + self.BLIND_ERR_PER_CM * bl["dist"]
            why = None
            if bl["dist"] > self.BLIND_MAX_CM:
                why = "it drove %.0f cm with no camera seeing it (the limit is %.0f cm)" % (bl["dist"], self.BLIND_MAX_CM)
            elif body_edge - err < self.BLIND_EDGE_CM:
                why = ("with no camera seeing it, its body may be %.1f cm from an edge (its own pose %.1f cm, less the "
                       "blind error %.1f cm; it must keep %.0f cm)" % (body_edge - err, body_edge, err,
                                                                       self.BLIND_EDGE_CM))
            if why and getattr(self, "self_correct", False) and not getattr(self, "straightening", False):
                # that robot alone stops (halted at once) and is held; the re-check finds it by its lights and rejoins
                self.log("blind_stop", robot=b.name, dist_cm=round(bl["dist"], 2), own=list(b.pose),
                         body_edge_cm=round(body_edge, 2), held=True)
                self.warnings = getattr(self, "warnings", []) + ["%s: %s" % (b.name, why)]
                self.say("WARNING: %s: %s: it is halted and held (found again by its lights); the others carry on"
                         % (b.name, why))
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
        if self.REACQUIRE_CM < d <= self.REACQUIRE_MAX_CM and b.pose is not None:
            # the camera's spot moved on by what the robot did since that picture
            nx, ny = cx + (b.pose[0] - own[1]), cy + (b.pose[1] - own[2])
            self.send(b.name, "fpose %.2f %.2f %.2f %.2f %.2f" % (nx, ny, b.pose[2], 1.0, 3.0))
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
        if any(b.out and self.rejoinable(b) and now() - getattr(b, "out_t", 0.0) < self.REJOIN_WAIT_S
               for b in self.bots.values()):
            return False             # a held robot may still rejoin (rejoin_watch)
        if not self.names:
            return False
        return all(all(st["status"] == "done" for st in self.bots[n].stat) for n in self.names)

    def run(self, timeout_s):
        self.plan_start = self.t_run = now()
        self.say("plan %s started: %d robots, %d steps, planned %.1f s"
                 % (self.plan.get("plan_id"), len(self.names), sum(len(self.steps(n)) for n in self.names),
                    self.plan["summary"]["makespan_s"]))
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
                    if getattr(self, "self_correct", False):
                        # Nick 30 Oct 10:08: one robot's trouble is not everyone's stop. Its edge guard (or anything)
                        # ended its follow run: it has stopped already; it is held and checked again, the others go on
                        why = ("its edge guard stopped it before an open plywood edge" if SC.run_outcome(b.ended) ==
                               SC.CALIB_LIMITED else "its follow run ended (%s)" % b.ended)
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
            if self.abort:
                break
            self.silence_watch()
            if self.abort:
                break
            self.unconfirmed_watch()
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
            if now() - self.t_run > timeout_s:
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
        return {"plan_id_first": self.first_plan_id, "plans": self.plan_no + 1, "result": self.abort or "done",
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
        return False, "its battery is at %d %%, under the %d %% floor" % (b.battery, args.battery_floor)
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
        warn(coord, "%s: no room to spin, or the calibration failed twice: the compass check alone (a few degrees' wiggle; no spin, no roll); "
                    "its stored gyro and wheel scales are used" % ", ".join(left))
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
            raise SystemExit("Battery under the %d %% floor: %s. Charge %s first (nothing was moved)."
                             % (args.battery_floor, ", ".join("%s %d %%" % x for x in low),
                                "it" if len(low) == 1 else "them"))
        warn(coord, "battery under the %d %% floor: %s: it runs anyway (charge it after)"
             % (args.battery_floor, ", ".join("%s %d %%" % x for x in low)))
    print("  " + "; ".join("%s: firmware %s, battery %s %%, boot %s" % (n, coord.bots[n].firmware,
                                                                       coord.bots[n].battery, coord.bots[n].boot)
                           for n in names))
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
        if hh is not None:
            x -= LIGHTS_AHEAD_CM * math.cos(math.radians(hh))
            y -= LIGHTS_AHEAD_CM * math.sin(math.radians(hh))
        out[n] = (round(x, 2), round(y, 2), None if h is None else float(h), "lights (facing check)",
                  None if hh is None else float(hh))
    return out


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
    record = coord.straighten_record = {"tries": [], "result": None}
    tries = 0
    max_tries = ST.TRIES + (1 if auto else 0)
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
                    warn(coord, "no camera fix for %s: its position is taken from its lights (%.1f, %.1f)"
                         % (n, lit[0], lit[1]))
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
                hold_at_start(coord, link, n, "it cannot be corrected safely where it stands (%s)" % e)
                pl["robots"][n] = {"steps": [], "words": [], "path": None, "need": False}
                break
            if pl is None:
                for n in names:
                    hold_at_start(coord, link, n, "no safe correction (%s)" % e)
                record["result"] = "refused: %s" % e
                return
        need = [n for n in names if pl["robots"][n]["steps"] and n not in coord.start_aside]
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
    try:
        replan_from(coord, {n: meas[n] for n in names}, "the start's corrections did not reach the start spots")
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
    ap.add_argument("--calibrate", action="store_true", help="send calibrate to every robot first")
    ap.add_argument("--recalibrate", choices=("auto", "no"), default="auto",
                    help="auto (default): calibrate a robot that restarted since its calibration today (a new boot ID) "
                         "or says it is not calibrated, then carry on; no: stop and say so")
    ap.add_argument("--replan-from-camera", action="store_true",
                    help="plan again from where the camera has the robots when they are off their start spots "
                         "(Mission Control, Monday's command); without it, a robot off its spot stops the start")
    ap.add_argument("--battery-floor", type=int, default=EC.BATTERY_FLOOR_PCT,
                    help="refuse to start a robot under this battery %% (default %(default)s; 0 = off)")
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
    ap.add_argument("--unconfirmed-s", type=float, default=FSTOP.UNCONFIRMED_S,
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
    ap.add_argument("--poses-to", metavar="HOST:PORT",
                    help="send each robot's own pose (FOLLOW records) as JSON lines here: the camera bridge "
                         "(tools/fleet/cam_bridge.py, default port 5011) predicts who is who from them")
    g = ap.add_argument_group("dry run against the simulator (tools/sim/fleet_sim.py)")
    ap.add_argument("--border-cm", type=float, default=BORDER_CM,
                    help="the robots' own four-sided border (firmware 2026-10-27a), sent when the map has side kinds "
                         "(default %(default)s; 0 = off)")
    g.add_argument("--dry-run", action="store_true")
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
    link = AC.Link(outdir, robot_ips)
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
    try:
        import signal
        if hasattr(signal, "SIGUSR1"):        # Mission Control's Pause / Resume
            signal.signal(signal.SIGUSR1, lambda *_: setattr(coord, "pause_req", True))
            signal.signal(signal.SIGUSR2, lambda *_: setattr(coord, "pause_req", False))
        if hasattr(signal, "SIGHUP"):         # the SSH link to the Pi dropped (the remote ladder): stop every robot
            signal.signal(signal.SIGHUP, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    except ValueError:
        pass                                  # not the main thread (tests)
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
