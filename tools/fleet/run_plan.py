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
import socket
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

MIN_FIRMWARE = "2026-10-18b"
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


def step_command(sid, st):
    t = st["type"]
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
        ms = int(round((st["t_end"] - st["t_start"]) * 1000)) if st["phase"] == "dwell" else 0
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
        self.log_f.write(json.dumps(rec) + "\n")
        self.log_f.flush()

    def say(self, msg):
        t0 = getattr(self, "t_run", None)
        t = now() - t0 if t0 else 0.0
        print("[%6.1f] %s" % (t, msg), flush=True)
        self.log("message", text=msg)

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
            if kind == "M":
                b.said.append((t, vals))
                low = vals.lower()
                if low.startswith("status "):
                    k = vals.find("(firmware ")
                    if k >= 0:
                        b.firmware = vals[k + 10:vals.find(")", k)]
                if "not calibrated" in low:
                    b.calibrated = False
                if self.plan_start is not None and ("follow:" in low or "turn timed out" in low or "stopped" in low):
                    self.say("%s: %s" % (name, vals))
                continue
            if rtype == "RUNINFO" and len(vals) >= 2:
                b.run_id, b.run_exp = vals[0], vals[1]
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
                continue
            if rtype == "RAW" and self.change is not None:
                self.site_check(b, t, vals)
                continue
            if rtype == "STEP" and len(vals) >= 9:
                self.on_step(b, t, vals)
            elif rtype == "FOLLOW" and len(vals) >= 17:
                self.on_follow(b, t, vals)

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
        elif state == "done":
            if st["status"] != "done":
                st["status"] = "done"
                st["t_done"] = t
                st["info"] = info
                stp = self.steps(b.name)[i]
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
                self.on_failed(b, i, info, (x, y, th), front)
        elif state == "dropped":
            if st["status"] in ACTIVE:
                st["status"] = "dropped"

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
        # done reports lost over WiFi: the robot runs its steps in order, so everything up to last_done is done
        for sid, i in b.id_index.items():
            st = b.stat[i]
            if sid <= last_done and sid != last_failed and st["status"] in ACTIVE:
                st["status"] = "done"
                st["t_done"] = t
                st["info"] = st.get("info") or "(seen in FOLLOW)"
            if sid == last_failed and st["status"] in ACTIVE:
                st["status"] = "failed"
                st["info"] = "failed (STEP record lost)"
                self.on_failed(b, i, st["info"], (x, y, th), float(v[14]) if v[14] else 0.0)

    # -- outgoing -------------------------------------------------------------
    def release(self):
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
                sid = b.next_id
                b.next_id += 1
                b.ids[i] = sid
                b.id_index[sid] = i
                st.update(status="sent", t_sent=t, text=step_command(sid, stp), sends=1)
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
        t = now()
        for b in self.bots.values():
            ev = b.fix
            if ev is None or b.offset is None or t - b.fix_sent_t < 1.0 / self.args.fix_hz:
                continue
            ms = int(round((ev["t"] - b.offset) * 1000))
            th = ev.get("th")
            self.send(b.name, "fix %d %.2f %.2f %s %.2f %.2f" % (ms, ev["x"], ev["y"], "-" if th is None else "%.2f" % th,
                                                                ev.get("sigma", 0.5), ev.get("sigma_th", 1.5)))
            b.fix, b.fix_sent_t = None, t
            b.fix_sent += 1

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
                dur = steps[i]["t_end"] - steps[i]["t_start"] if steps[i]["phase"] == "dwell" else 0.0
                dwell_left = max(0.0, dur - (t - st.get("t_started", t)))
            pose = b.pose or tuple(FM.pose_out(FM.pose_in(self.plan["robots"][n]["start"])))
            out[n] = {"pose": [pose[0], pose[1], math.radians(pose[2])], "step": i, "in_progress": prog,
                      "dwell_left_s": dwell_left}
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
        if budget is not None:
            over["replan_budget_s"] = budget
        try:
            dec = P.obstacle_decide(self.m, self.plan, states, [ob] if ob else [], over=over)
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
            self.say("no new plan (%s): stopping everyone" % job["error"])
            self.halt_all()
            self.abort = "no plan"
            return
        newplan, events = job["result"]
        cpu = job["cpu"]
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
        self.say("new plan in %.2f s (budget %.1f s); everyone done about %.0f s after the event"
                 % (cpu, over["replan_budget_s"], newplan["summary"]["makespan_s"]))
        old = {n: (list(self.bots[n].ids), [dict(x) for x in self.bots[n].stat]) for n in self.names}
        if ob:
            self.m["obstacles"].append(FM._norm_obstacle(ob, len(self.m["obstacles"])))
            self.m = FM.normalize(self.m)
        self._install(newplan, dec["keep"], dec["first_step"], old)
        self.plan_no += 1
        self.plan_start = t_event
        self.save_plan()
        self.next_retry = now() + self.args.retry_s if newplan.get("stranded") else math.inf
        if ob is not None and self.args.improve_after_s > 0:
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

    def request_replan(self, why):
        if self.pending_replan is None:
            self.pending_replan = (why, now() + 0.3)

    def on_failed(self, b, i, info, pose, front):
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
        if info.startswith("obstacle"):
            self.n_sensor_obstacles += 1
            ob = self.sensor_obstacle(b, pose, front, info)
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
            return "step %d ready" % i
        return "done"

    # -- main loop ------------------------------------------------------------
    def finished(self):
        if self.plan.get("stranded"):
            return False
        return all(all(st["status"] == "done" for st in self.bots[n].stat) for n in self.names)

    def run(self, timeout_s):
        self.plan_start = self.t_run = now()
        self.say("plan %s started: %d robots, %d steps, planned %.1f s"
                 % (self.plan.get("plan_id"), len(self.names), sum(len(self.steps(n)) for n in self.names),
                    self.plan["summary"]["makespan_s"]))
        last_status = now()
        while True:
            self.drain()
            self.fixes()
            for b in self.bots.values():
                if b.ended and not self.abort:
                    if SC.run_outcome(b.ended) == SC.CALIB_LIMITED:   # (follow runs hold instead: firmware older than
                        #                                                 2026-10-18c's step-only edge stop)
                        self.abort = "limited: %s's edge guard stopped it before an open plywood edge" % b.name
                    else:
                        self.abort = "%s's follow run ended (%s)" % (b.name, b.ended)
                    self.say(self.abort + ": stopping everyone")
                    self.halt_all()
            if self.abort:
                break
            self.finish_replan()
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
        for n in self.names:
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
                "fixes_used_rejected": {n: (b.follow.get("fixes_used"), b.follow.get("fixes_rejected"))
                                        for n, b in self.bots.items()},
                "replans": self.replans, "dockings_reported": self.dockings,
                "boards_not_visited": self.missed_boards(),
                "sensor_obstacles": self.n_sensor_obstacles}


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


def start_robots(coord, link, args, source):
    names = coord.names
    print("Waiting for robots: %s ..." % ", ".join(names))
    if not wait_for(lambda: set(names) <= set(link.robots()), args.find_s, coord):
        missing = sorted(set(names) - set(link.robots()))
        raise SystemExit("Not heard from: %s. Are they on, on this WiFi, and named as in the map?" % ", ".join(missing))
    for n in names:
        link.send_name(n, "stop")          # a follow run left over from an earlier run_plan would refuse a new one
    time.sleep(1.5)
    for cmd in getattr(args, "send", None) or []:      # e.g. set safe_edge 1 (tools/site/setup.py's edge approaches)
        for n in names:
            link.send_name(n, cmd)
        time.sleep(0.3)
    for n in names:
        link.send_name(n, "status")
    wait_for(lambda: all(coord.bots[n].firmware for n in names), 5, coord)
    for n in names:
        fw = coord.bots[n].firmware
        if not fw:
            print("  %s did not say its firmware version (status lost?); carrying on" % n)
        elif fw < MIN_FIRMWARE:
            raise SystemExit("%s runs firmware %s; the step follower needs %s or later. Flash it first."
                             % (n, fw, MIN_FIRMWARE))
    if args.calibrate:
        for n in names:
            link.send_name(n, "calibrate")
        print("Calibrating (about 65 s each, all at once: each needs a 25 cm circle to spin in and, in some direction, "
              "35 cm of clear floor from its back to roll)...")
        if not wait_for(lambda: all(getattr(coord.bots[n], "last_runend", (None,))[0] == "calib" for n in names), 120, coord):
            print("  not every calibration run reported its end; carrying on")
        time.sleep(1.0)
        coord.drain()
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
            elif oc == SC.CALIB_FAILED:
                print("  %s: calibration did not finish (%s)" % (n, le[1] if le and le[0] == "calib" else "no end record"))
        if not args.yes and not args.dry_run:
            input("Calibrated. Put every robot on its start spot, then press Enter: ")
    for n in names:
        coord.bots[n].ended = None
        coord.bots[n].run_exp = None
        if args.calibrate:
            coord.bots[n].calibrated = None
    # start the follow runs
    for attempt in range(3):
        for n in names:
            if coord.bots[n].run_exp != "follow":
                link.send_name(n, "run follow")
        if wait_for(lambda: all(coord.bots[n].run_exp == "follow" or coord.bots[n].calibrated is False for n in names),
                    4, coord):
            break
    bad = [n for n in names if coord.bots[n].calibrated is False]
    if bad:
        raise SystemExit("Not calibrated since power-on: %s. Lift each off its spot, send `calibrate` (console) or "
                         "use --calibrate, put it back, and start again." % ", ".join(bad))
    missing = [n for n in names if coord.bots[n].run_exp != "follow"]
    if missing:
        raise SystemExit("These robots did not start their follow run: %s" % ", ".join(missing))
    # wait until each follow loop is up (its FOLLOW records arrive)
    if not wait_for(lambda: all(coord.bots[n].follow for n in names), 8, coord):
        raise SystemExit("No FOLLOW records from: %s" % ", ".join(n for n in names if not coord.bots[n].follow))
    # where are they?
    starts = {n: coord.plan["robots"][n]["start"] for n in names}
    if source.live:
        print("Waiting for camera fixes...")
        seen = {}

        def got():
            for ev in source.poll():
                if ev.get("type") == "fix" and ev.get("robot") in starts and ev.get("th") is not None:
                    seen[ev["robot"]] = ev     # (the start check needs a heading: position-only fixes wait)
            return set(seen) == set(names)
        wait_for(got, args.fix_wait_s, coord)
        for n in names:
            ev = seen.get(n)
            if ev is None:
                raise SystemExit("No camera fix for %s: is the position source running?" % n)
            s = starts[n]
            d = math.hypot(ev["x"] - s[0], ev["y"] - s[1])
            dh = abs((ev["th"] - s[2] + 180) % 360 - 180)
            print("  %s: camera puts it %.1f cm / %.1f deg from its start spot" % (n, d, dh))
            if (d > args.place_tol_cm or dh > args.place_tol_deg) and not args.yes:
                raise SystemExit("%s is not on its start spot (%.1f cm, %.1f deg off; allowed %.1f cm, %.1f deg)."
                                 % (n, d, dh, args.place_tol_cm, args.place_tol_deg))
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
    if nopose:
        raise SystemExit("No pose on: %s" % ", ".join(nopose))
    print("Robots ready: " + ", ".join("%s at %.1f,%.1f,%.0f" % (n, *coord.bots[n].pose) for n in names))


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
    ap.add_argument("--place-tol-deg", type=float, default=5.0)
    ap.add_argument("--calibrate", action="store_true", help="send calibrate to every robot first")
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
    g = ap.add_argument_group("dry run against the simulator (tools/sim/fleet_sim.py)")
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
    if args.plan:
        with open(args.plan, encoding="utf-8") as f:
            plan = json.load(f)
        if plan.get("map_hash") != FM.map_hash(m):
            raise SystemExit("plan.json was made for a different map (plan %s, map %s): plan again."
                             % (plan.get("map_hash"), FM.map_hash(m)))
    else:
        print("Planning...")
        plan = P.make_plan(m)
    for n in plan["robots"]:
        for st in plan["robots"][n]["steps"]:
            if st["type"] == "approach" and "handover_cm" not in st:
                raise SystemExit("plan.json is from before 2026-10-18b (approach steps without handover_cm): plan again.")
    outdir = args.session or os.path.join("logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") + ("-dryrun" if args.dry_run else "-plan"))
    os.makedirs(outdir, exist_ok=True)
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
        print("Starting the simulated robots: " + " ".join(cmd[1:]))
        sim = subprocess.Popen(cmd, stdout=open(os.path.join(outdir, "sim_stdout.txt"), "w"), stderr=subprocess.STDOUT)
        time.sleep(1.5)
    source = make_source(fixes)
    link = AC.Link(outdir, robot_ips)
    link.session.quiet = True
    args.homes = assigned
    coord = Coordinator(args, m, plan, link, source, outdir)
    coord.first_plan_id = plan.get("plan_id")
    coord.first_makespan = plan["summary"]["makespan_s"]
    coord.started = now()
    code = 0
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
        link.close()          # sends stop to every robot (ends the follow runs) and closes the files
        source.close()
        summ = coord.summary() if hasattr(coord, "dockings") else {"result": coord.__dict__.get("abort")}
        if sim is not None:
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
        print("Camera fixes sent %s; used/ignored on the robots %s" % (s["fixes_sent"], s["fixes_used_rejected"]))
    for r in s.get("replans", []):
        print("  re-plan at %.1f s (%s): %.2f s to compute%s" % (r["t_run"], r["source"], r["cpu_s"],
                                                                "" if r.get("kept", True) else ", not kept"))
    sim = s.get("simulator")
    if sim:
        print("Simulator truth: closest two robots %.2f cm (%s); closest to a board/brick %.2f cm (%s); contact: %s"
              % (sim["min_body_gap_cm"], sim["min_body_gap_at"], sim["min_static_clearance_cm"], sim["min_static_at"],
                 "YES" if sim["collision"] else "no"))
        for d in sim["dockings"]:
            print("  dock %-6s at %s: front gap %.2f cm (error %+.2f), sideways %+.2f cm, heading %+.1f deg  [%s]"
                  % (d["robot"], d["board"], d["front_gap_cm"], d["gap_error_cm"], d["lateral_cm"], d["heading_err_deg"], d["info"]))
        for e in sim.get("events", []):
            print("  sim event: %s" % e)


if __name__ == "__main__":
    sys.exit(main())
