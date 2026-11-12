#!/usr/bin/env python3
"""Home free roam (MIRLT, 2026-11-05): Left, Center and Right moving at the same time, all the time, on the home board
until each battery reaches 10 % (or the time cap, 4 h), unattended on the home Pi, then everything uploaded.

Nick, 5 Nov: "Continuous simultaneous movement of multiple robots is the accomplishment, that's a trophy. Everything
else is bonus." 11:23: "the trophy is shiniest if they never stop moving".

    python3 tools/fleet/freeroam.py run [--hours 4] --robot IP ... --stream 127.0.0.1:8000 --calib C.json
    python3 tools/fleet/freeroam.py run --dry [--hours 0.5] [--sim-minutes 30]     the simulator (fleet_sim.py), no robots
    python3 tools/fleet/freeroam.py status [--dir logs/<...>-freeroam]
    python3 tools/fleet/freeroam.py report <folder>                                   the report again
run-fleet-pi.sh starts it in tmux session "freeroam" (./run-fleet-pi.sh freeroam [hours]), so it keeps running when
the SSH window closes or Tailscale is switched to another account.

  start   (robots) the remote ladder's L0 (find the robots by their lights, battery, firmware, the edge guard) and L1
          (calibrate; kept when calibrated today on this power-on), a heading crawl for a robot whose heading is not
          known; (--dry) the simulated board, robots at random spots 30 cm in and 40 cm apart
  roam    the camera bridge (cam_bridge.py, every robot followed from where L0 found it), then the controller
          (freeroam_core.py: balanced menu, reservations, fallbacks, reflex, dead reckoning, battery) at 20 Hz;
          every robot in follow mode, its watchdog on (firmware 2026-11-05b), its edge guard and own border on
  end     every robot parked and stopped at 10 % (asleep: `sleep 0`, switch off and on to wake), or the time cap, or
          the card nearly full (diskguard), or Ctrl+C / freeroam-stop: every robot stops; the logs, the camera track and
          the report (report.md + charts) in logs/<date-time>-freeroam/; run-fleet-pi.sh uploads it.
"""
import argparse
import csv
import datetime as dt
import glob
import json
import math
import os
import queue
import random
import re
import signal
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import freeroam_core as FC   # noqa: E402

PY = sys.executable
VERSION = "freeroam-runner-2026-11-12e17"   # e17: the layer told who is unaccounted, its corrections (how) to the controller (an fpose, never a fix the robot's filter would refuse), revoked eliminations, the board for the gate's cap, the ML hook (off). Before: "freeroam-runner-2026-11-11e16"
VERSION_E16 = "freeroam-runner-2026-11-11e16"   # e16 (home 11 Nov 08:48): each RAW row's accelerometer (ax, ay) and lift flag to the controller (on_raw_motion: a jolt while standing is a contact; a lift is evidence a person moved it). Before: "freeroam-runner-2026-11-10e14b"   # e14b: a dark picture holds every robot until the camera has adjusted. e14 (10 Nov): the battery floor from MIRLT_MIN_BATTERY, else Mission Control's Advanced, else the site file, said at the start
VERSION_E13 = "freeroam-runner-2026-11-10e13"   # e13 (10 Nov): --readable-map (school: plan only where the camera reads tags; MIRLT_ROAM_READABLE=0 the old whole table)
VERSION_E12 = "freeroam-runner-2026-11-10e12"   # e12 (10 Nov): the bridge's "camera" messages (no pictures) said and logged; one robot held at an edge parked and shown, the others roam on; orderly shutdown
VERSION_E11 = "freeroam-runner-2026-11-09e11"   # e11 (9 Nov school): --alias WIFI=SLOT (names in the run only), --practice-real
VERSION_D = "freeroam-runner-2026-11-07d"   # 11-07d: a robot at an edge makes no proof drive (the run 2 incident d)
VERSION_C = "freeroam-runner-2026-11-07c"   # 11-07c (e10 Phase C): the start's proof drive (start_discovery.py), the proven
                                          # workspace (proven_workspace.py, saved per site), --no-discovery,
                                          # --typed-board, --probe-ahead, --tracking tags (school free roam)
                                          # 11-07b (e09 Phase B): the tracking layer on the bridge's fixes, RAW to each
                                          # robot's filter, pictures (B4), the brick watch on, --pose-source,
                                          # --gyro-heading, --old-tracking, --no-brick-watch
                                          # 11-07a (e08 Phase A): --edge-rule, --pose-rule, --keepout-rule,
                                          # --silent-out, --no-tilt-recovery, --stop-lead; attended runs ask questions
                                          # (Mission Control's card); a failed heading crawl joins held, not out
                                          # 11-05d: --motion curves|stopturn, arc calibration save
HOME_BOARD = (0.0, 0.0, 122.24, 121.92)    # roam.py's board (the home plywood in the floor calibration's cm)
FOLLOW_LOOK_CM = 8.0    # the robots' own front stop while driving: roam.py's 8 (the plywood reads ~10 cm ahead, 12
                        # stopped drives). Another robot's face at the planned gap is 7.9 cm or more away; the reflex
                        # keeps 2.5 cm between the circles (about 6.5 face to face). No DOE: a backstop
FIX_HZ = 10.0           # camera fixes forwarded to each robot a second (run_plan --fix-hz). Not chosen here
TICK_S = 0.05           # the controller's loop (20 a second; the camera bridge sends ~12 fixes a second per robot)
SAFE = ["set safe_edge 3", "set edge_dir 0", "set edge_thr 150", "set edge_n 3", "set edge_bot_cm 11",
        "set bumper_cm 0"]                 # the robots' edge guard (ladder.SAFE; not kept over a power-off)
DISK_CHECK_S = 60.0
READABLE_MIN_CM2 = 2 * 2 * 40.0 * 40.0   # (e13) under this much readable ground (two robots' 40 cm squares, twice) the
                        # limit is ignored and said: a run that could not move is worse. Chosen; no DOE (a floor)



def bridge_pin():
    """(e17) The camera bridge's cores under the core layout (MIRLT_BRIDGE_CORES, set by run-fleet-pi.sh): a taskset
    prefix, or nothing."""
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "pi"))
        import core_layout as CL
        return CL.bridge_prefix()
    except Exception:  # noqa: BLE001
        return []

def _fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def site_safe_edge(a):
    """--safe-edge, else the site in use's (school 2), else 3."""
    if getattr(a, "safe_edge", None) is not None:
        return int(a.safe_edge)
    try:
        import sites as SI
        import observe as OB
        return OB.site_safe_edge(SI.load(None), 3)
    except Exception:  # noqa: BLE001
        return 3


def safe_cmds(safe_edge=None):
    """(11-05h) SAFE with the edge guard's mode from --safe-edge (default 3, home) and the tilt stop added."""
    import observe as OB
    se = 3 if safe_edge is None else int(safe_edge)
    out = [("set safe_edge %d" % se) if c.startswith("set safe_edge") else c for c in SAFE]
    return out + [c for c in OB.safety_commands(se) if not c.startswith(("set safe_edge", "set edge_bot_cm"))]


def say(s=""):
    print(s, flush=True)


class Tables:
    def __init__(self, folder):
        self.dir = folder
        self.w = {}

    def __call__(self, table, row):
        w = self.w.get(table)
        if w is None:
            p = os.path.join(self.dir, table + ".csv")
            f = open(p, "w", newline="", buffering=1)
            w = (csv.DictWriter(f, fieldnames=list(row.keys()), extrasaction="ignore"), f)
            w[0].writeheader()
            self.w[table] = w
        try:
            w[0].writerow(row)
        except ValueError:
            pass

    def close(self):
        for _, f in self.w.values():
            try:
                f.close()
            except OSError:
                pass


class Runner:
    """The controller on a live link: robots (alvik_console.Link) and camera fixes (UDP JSON lines)."""

    def __init__(self, a, outdir, names, ips, rect, fix_port=5010, pose_to=None, frames=None, guard=None):
        import alvik_console as AC
        from run_plan import UdpSource
        self.AC = AC
        self.a, self.dir, self.names = a, outdir, list(names)
        self.rect = rect
        self.safe_edge_used = site_safe_edge(a)
        self.params_read = {}
        self.boots = {}
        self.q = queue.Queue()
        # (e11, 9 Nov school) names in the run only: a robot answering with its WiFi name (AL-1DC0) is the run's slot
        # (Alpha) everywhere here (link, fixes, logs, status, the identity wiggle), as run_plan's --alias
        self.aliases = dict(kv.split("=", 1) for kv in (getattr(a, "alias", None) or []) if "=" in kv)
        if self.aliases:
            say("Names in this run only (the robots keep their WiFi names): %s" % ", ".join(
                "%s is %s" % kv for kv in sorted(self.aliases.items())))
        self.link = AC.Link(outdir, ips, aliases=self.aliases)
        self.link.session.quiet = True
        self.link.session.listeners.append(lambda *v: self.q.put((AC.now(),) + v))
        self.source = UdpSource(fix_port)
        self.tables = Tables(outdir)
        import safety_rules as SR
        asks = SR.Questions(run=os.path.basename(outdir.rstrip("/"))) if a.attended else None
        self.ctrl = FC.Controller(names, rect, self.send, self.logrow, seed=a.seed, lost_mode=a.lost_mode,
                                  battery_floor=a.battery_floor, park_sleep=not a.no_sleep,
                                  drill=a.reflex_drill if a.dry else 0.0, motion=a.motion,
                                  edge_rule=a.edge_rule, pose_rule=a.pose_rule, keepout_rule=a.keepout_rule,
                                  silent_out=a.silent_out, tilt_recovery=not a.no_tilt_recovery,
                                  attended=a.attended, questions=asks, leads=stop_leads(a, names),
                                  pose_source=getattr(a, "pose_source", None),
                                  gyro_heading=getattr(a, "gyro_heading", False) or None,
                                  workspace=self.make_workspace(rect),
                                  probe_ahead=getattr(a, "probe_ahead", False) and not getattr(a, "careful", False),
                                  shuffle=not getattr(a, "no_shuffle", False),
                                  picture_every_s=getattr(a, "picture_every_s", None),
                                  tracking=getattr(a, "tracking", "lights") or "lights")
        # (e09, Phase B1) the ONE tracking layer: the bridge's fixes (outlines, lights, lock-on) are named again here
        # against the robots' own poses, with the close-pass guard (wiggle first), the camera watch, the parked watch
        import tracking_layer as TL
        old = bool(getattr(a, "old_tracking", False))
        self.layer = TL.TrackingLayer(names, TL.sources_for(getattr(a, "tracking", "lights") or "lights", old=old),
                                      identity="predict" if old else None, parked_watch=not old,
                                      brick_watch=False, cluster=not old,
                                      # (e10) the layer's default (MIRLT_CAMERA_WATCH, on): e09 passed True here, so
                                      # e09b's "off" never reached the home runner
                                      camera_watch=None if not old else False,
                                      log=lambda kind, row: self.tables(kind, row),
                                      wiggle=None if old else (lambda n, why: self.ctrl.identity_wiggle(
                                          self.ctrl.R[n], why, spots=getattr(self.layer, "wiggle_spots", None))),
                                      board=tuple(rect)[:4], ml_fn=ml_detector_fn())
        # (e17, 1c / 6) the controller's measured corrections widen the layer's gate for that robot
        self.ctrl.record_fn = lambda n, t, d, trav: self.layer.record(n, t, d, trav)
        self.ctrl.fix_fn = self.fix_to_robot
        self.ctrl.place_fn = self.ask_place          # (e13, Nick 12:56) a held robot: a person clicks it on a picture
        self.ctrl.place_residual_fn = self.place_residual
        self.ctrl.place_cancel_fn = lambda n: __import__("place_click").cancel(n)
        self.place_poll_t = 0.0
        self.disc_drv = None                 # (e10, C1) the start's proof drive while it runs
        self.cam_fix = None                  # (e10) the calibration the discovery chose (None: as configured)
        self.cam_mirror = False
        self.tag_yaw = {}
        self.camera_off = False
        self.held = dict(getattr(a, "_held", None) or {})
        self.ctrl.arc_save_on = not a.no_arc_save
        self.offsets = {n: [] for n in names}
        self.fix_last = {n: 0.0 for n in names}
        self.run_exp = {n: None for n in names}
        self.fw = {}
        self.restarts = {n: 0 for n in names}
        self.track = open(os.path.join(outdir, "camera_track.csv"), "w", buffering=1)
        self.track.write("t,robot,x_cm,y_cm,th_deg,arrived\n")
        self.pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if pose_to else None
        self.pose_to = pose_to
        self.frames, self.guard = frames, guard
        self.t0 = None
        self.joined = set()

    # -- (e10) the proven workspace and the start's discovery ------------------------------------------------------
    def make_workspace(self, rect, camera=True):
        """(e10, C2) The planner's workspace: what the camera's setup shows of the board (its outline, the whole of it at
        home: the home camera sees the board) plus what earlier runs at this site felt (saved per site, used again only
        on the same outline). --typed-board: the plain rectangle, as before e10."""
        if getattr(self.a, "typed_board", False):
            return None
        import proven_workspace as PW
        ws = PW.ProvenWorkspace(rect)
        if camera:
            ws.see(rect)
        p = PW.site_path(self.a.conf_dir, self.a.site)
        if not getattr(self.a, "careful", False):
            ws.events.append(("saved", ws.merge_saved(p)))
        self.ws_path = p
        self.limit_readable(ws)
        return ws

    def limit_readable(self, ws):
        """(e13, e12's school practice: robots roamed past x 254 cm where the camera cannot read and stood lost) At a
        roof-tag site the planner's ground is the proven workspace AND the area the camera has proven readable (the
        run's readability map: the camera model's seed, earlier runs' reads and stops). Switch MIRLT_ROAM_READABLE=0
        (the whole proven table, before e13). An empty or tiny readable area is said and ignored (degrade, not stop)."""
        rp = getattr(self.a, "readable_map", "") or ""
        if not rp or os.environ.get("MIRLT_ROAM_READABLE", "1") == "0":
            return None
        try:
            import observe as OB
            with open(rp) as f:
                rm = OB.ReadMap.from_json(json.load(f))
        except (OSError, ValueError, KeyError) as e:
            say("(the readability map %s could not be read (%s): the whole proven table is used)" % (rp, e))
            return None
        area = ws.limit_to(lambda x, y: rm.readable(x, y)[0], "readable")
        if area < READABLE_MIN_CM2:
            ws.unlimit()
            say("!! THE READABLE AREA IS ONLY %d cm2 (under %d): the whole proven table is used; robots may be lost "
                "where the camera cannot read" % (area, READABLE_MIN_CM2))
            return None
        say("FREE ROAM AREA: only where the camera reads roof tags (%d cm2; readability map %s). A robot that strays "
            "out comes back along the ground it drove." % (area, os.path.basename(rp)))
        self.readable_cm2 = area
        return area

    def discover(self, live):
        """(e10, C1) The proof drive on every robot that joined (start_discovery.py), then what it found is used: the
        camera's calibration (or none), the board's size, the clock, turned tags, robots not seen, each robot's wheel and
        gyro fit (pooled on the Pi, saved on the robot by corr save once sure). Said in plain words and written to
        discovery.json."""
        import start_discovery as D
        c = self.ctrl
        say("Discovery: each robot makes a short proof drive on its spot (a turn, 12 cm, a quarter turn, 12 cm), "
            "slowly, its edge sensors on ...")
        first = {n: c.R[n].next_id for n in live}
        # (e10, run 2 incident d) a robot over or at an edge does not make the proof drive (the controller's start
        # check then holds it with a plain message)
        self.disc_drv = D.ProofDriver(live, self.send, t0=self.AC.now(), first_id=max(first.values()) if first else 1,
                                      board=tuple(self.rect) if c.start_edge else None, edge_cm=FC.START_EDGE_CM)
        end = time.time() + D.TIMEOUT_S + 5
        while time.time() < end and not self.disc_drv.done():
            self.drain()
            self.fixes()
            self.disc_drv.tick(self.AC.now())
            time.sleep(0.02)
        data = self.disc_drv.data()
        self.disc_drv = None
        for n, d in data.items():
            c.R[n].next_id = max(c.R[n].next_id, d["last_id"] + 1)          # the robot takes only higher ids
        res = D.analyse(data, tuple(self.rect), {"mirrored": D.mirror_x(self.rect)})
        for line in res["say"]:
            say("  " + line)
        c.event(None, "discovery", " ".join(res["say"]))
        if res["camera"] in ("missing", "unproven") and not getattr(self.a, "discovery_strict", False):
            # (e11, 9 Nov home: a 0 cm2 workspace froze every robot for 9 min) discovery could not prove the camera:
            # the TYPED board, as before e10 (the camera stays on; nothing is taken as proven by the camera's drive)
            msg = ("DISCOVERY COULD NOT PROVE THE CAMERA (%s): FALLING BACK TO THE TYPED BOARD %.0f x %.0f cm and the "
                   "camera as configured (the way before e10). Check the camera if the robots drift." % (
                       res["camera"], self.rect[2] - self.rect[0], self.rect[3] - self.rect[1]))
            say("!" * 20)
            say(msg)
            say("!" * 20)
            c.event(None, "discovery fallback", msg)
            res["fallback"] = "typed board"
            if c.ws is not None:
                ws = self.make_workspace(self.rect, camera=True)
                c.ws, c.rect = ws, ws
        elif res["camera"] in ("missing", "wrong", "unproven"):
            self.camera_off = True             # (e10) --discovery-strict: the run narrows to the robots' own sensors
            if c.ws is not None:
                ws = self.make_workspace(self.rect, camera=False)
                c.ws, c.rect = ws, ws
        else:
            if res["calibration"] != "as configured" or res.get("k_used", 1.0) != 1.0:
                self.cam_fix = res["fix"]
                self.cam_mirror = res["calibration"] == "mirrored"
            if res.get("k_used", 1.0) != 1.0:
                self.rect = tuple(res["board_found"])
                if c.ws is not None:
                    ws = self.make_workspace(self.rect)
                    c.ws, c.rect = ws, ws
                else:
                    c.rect = self.rect
            self.tag_yaw.update(res["tag_yaw"])
            if res.get("clock_s") is not None and abs(res["clock_s"]) >= D.SKEW_MIN_S:
                c.clock = float(res["clock_s"])
                c.clock_ests = [c.clock] * 5
                c.clock_seen = c.clock
            for n, rb in res["robots"].items():
                self.send(n, "fpose %.2f %.2f %.2f 1.00 2.00" % (rb["x"], rb["y"], rb["heading"]))
        for n in res["unseen"]:
            if c.R[n].mode not in ("gone", "out"):
                c.R[n].mode, c.R[n].out_why = "gone", "not seen by the camera at the start"
        for n, fit in res["fits"].items():
            pooled = D.pool_fit(self.a.conf_dir, n, fit)
            for cmd in D.corr_commands(pooled, getattr(self, "corr_now", {}).get(n)):
                if not self.a.dry:
                    self.send(n, cmd)
                c.event(c.R[n], "calibration saved", "%s (pooled over %d runs, the robot keeps it at power-off)" % (
                    cmd, pooled.get("wheel", pooled.get("gyro", {})).get("runs", 0)))
        with open(os.path.join(self.dir, "discovery.json"), "w") as f:
            json.dump({k: v for k, v in res.items() if k != "fix"}, f, indent=1, default=str)
        self.pump(0.5)
        return res

    def save_workspace(self):
        ws = self.ctrl.ws
        if ws is None or not getattr(self, "ws_path", None):
            return
        try:
            ws.save(self.ws_path)
            with open(os.path.join(self.dir, "workspace.json"), "w") as f:
                json.dump(ws.summary(), f, indent=1, default=str)
        except OSError as e:
            say("(the workspace could not be saved: %s)" % e)

    # -- (e13) a person's click on a held robot ---------------------------------------------------------------------
    def ask_place(self, name, last, stage="position", first=None, drive_cm=None, sigma_cm=None):
        import place_click as PC
        PC.request(name, last, run=os.path.basename(self.dir.rstrip("/")),
                   why="its possible error could reach an edge" if stage == "position" else
                   "it drove %.0f cm straight to show its heading: click it again" % (drive_cm or 0),
                   stage=stage, first=first, drive_cm=drive_cm, sigma_cm=sigma_cm)
        import socket as _s
        say("!! %s NEEDS A HAND: click it on a camera's picture in Mission Control (http://%s:%d, the card at the top), "
            "or put it on a sticker and click that point on the map; it moves again from there" % (
                name, _s.gethostname(), 8095))

    def place_poll(self):
        if time.time() - self.place_poll_t < 0.5:
            return
        self.place_poll_t = time.time()
        import place_click as PC
        for n, r in self.ctrl.R.items():
            if r.__dict__.get("place_asked") is None:
                continue
            hd = r.__dict__.get("hd_drive")
            if hd is not None and hd.get("asked") and not hd.get("suggested"):
                # (Nick 13:01) the motion detector / lock-on saw the drive's end: offered as the second click
                for u in list(getattr(self.layer, "unclaimed", []) or []):
                    d_ = math.hypot(u["x"] - hd["p0"][0], u["y"] - hd["p0"][1])
                    if abs(d_ - hd["L"]) <= max(3.0, 2 * hd["sig"]):         # (the layer keeps only recent ones)
                        PC.set_suggest(n, u["x"], u["y"], "the motion detector", 1.0)
                        hd["suggested"] = True
                        break
            a = PC.take_answer(n)
            if a is not None:
                self.ctrl.on_place(n, a)

    def place_residual(self, name, pl, x, y, res):
        import place_click as PC
        PC.residual(name, pl["t"], pl["x"], pl["y"], x, y, self.ctrl.t, driven_cm=pl.get("driven_cm"), res_cm=res)

    # -- out ---------------------------------------------------------------------------------------------------
    def send(self, name, text):
        self.link.send_name(name, text)

    def logrow(self, table, row):
        self.tables(table, row)
        if table == "events":
            k = row.get("kind", "")
            if k not in ("halt",):
                say("[%7.1f] %-7s %s: %s" % (row["t"] - (self.t0 or row["t"]), row["robot"], k, row["text"]))
            if self.frames is not None and k in ("trophy break", "front stop", "stuck", "lost", "find me", "silent",
                                                 "watchdog", "out"):
                self.frames.event(k, row["text"], robot=row["robot"])

    # -- in ----------------------------------------------------------------------------------------------------
    def offset(self, name, t, ms):
        o = self.offsets.setdefault(name, [])
        o.append((t, t - ms / 1000.0))
        if len(o) > 300:
            del o[:100]
        rec = [v for tt, v in o if tt > t - 30.0]
        return min(rec) if rec else None

    def drain(self):
        c = self.ctrl
        while True:
            try:
                t, name, kind, rtype, vals = self.q.get_nowait()
            except queue.Empty:
                return
            if name not in c.R:
                continue
            c.on_heard(name, t)
            if kind == "M":
                low = vals.lower()
                m = re.search(r"\(firmware ([0-9a-z-]+)", vals)
                if m:
                    self.fw[name] = m.group(1)
                    c.on_fw(name, m.group(1))
                m = re.search(r"boot ([0-9A-F]{4})\)", vals)
                if m:
                    self.on_boot(name, m.group(1))
                m = re.search(r"battery (\d+)%", vals)
                if m:
                    c.on_battery(name, int(m.group(1)), t)
                m = re.search(r"arc_k_scale ([0-9.]+).*arc_d_scale ([0-9.]+)", vals)
                if m:                       # (11-05d) the curves' calibration in use (boot / saved lines)
                    c.R[name].arc_scales = (float(m.group(1)), float(m.group(2)))
                if "not calibrated" in low:
                    c.event(c.R[name], "not calibrated", vals)
                m = re.match(r"\s*([a-z_0-9]+) = (-?[0-9.]+)", vals)
                if m:                       # (11-05h) a `get` answer: the safety layers' read back
                    self.params_read.setdefault(name, {})[m.group(1)] = float(m.group(2))
                continue
            try:
                if rtype == "STEP" and len(vals) >= 9:
                    self.offset(name, t, int(float(vals[3])))
                    if self.disc_drv is not None:            # (e10) the start's proof drive
                        self.disc_drv.on_step(name, int(vals[1]), vals[2], ",".join(vals[8:]))
                        continue
                    c.on_step(name, int(vals[1]), vals[2], ",".join(vals[8:]))
                elif rtype == "FOLLOW" and len(vals) >= 10:
                    ms = int(float(vals[1]))
                    self.offset(name, t, ms)
                    if vals[3] != "nopose":
                        c.on_follow(name, t, vals[3], float(vals[7]), float(vals[8]), float(vals[9]), int(vals[2]),
                                    int(vals[5]), int(vals[6]))
                        if self.disc_drv is not None:        # (e10) the start's proof drive
                            self.disc_drv.on_follow(name, t, float(vals[7]), float(vals[8]), float(vals[9]), vals[3],
                                                    int(vals[5]), int(vals[6]))
                        if self.pose_sock is not None:
                            off = self.offset(name, t, ms)
                            msg = {"type": "pose", "robot": name, "t": round(ms / 1000.0 + off, 4),
                                   "x": float(vals[7]), "y": float(vals[8]), "th": float(vals[9]), "state": vals[3]}
                            try:
                                self.pose_sock.sendto(json.dumps(msg).encode(), self.pose_to)
                            except OSError:
                                pass
                elif rtype == "RUNINFO" and len(vals) >= 6:
                    self.run_exp[name] = vals[1]
                    self.fw[name] = vals[2]
                    c.on_fw(name, vals[2])
                    try:
                        c.on_battery(name, int(float(vals[5])), t)
                    except ValueError:
                        pass
                elif rtype == "RUNEND" and len(vals) >= 2:
                    if self.run_exp.get(name) == "follow" and c.R[name].mode not in ("out", "gone"):
                        self.run_exp[name] = None
                        c.event(c.R[name], "follow ended", "its follow run ended (%s): started again" % vals[1])
                        self.restarts[name] += 1
                        r = c.R[name]
                        if r.move is not None:
                            c.end_move(r)
                        if self.restarts[name] <= 20:
                            self.send(name, "run follow")
                elif rtype == "FSPD":
                    c.on_fspd(name)
                elif rtype == "RAW":
                    # (11-05h, protection 2) the standing tilt from the robot's own roll / pitch
                    cols = self.link.session.headers.get("RAW") or []
                    if cols:
                        row = dict(zip(cols, vals))
                        c.on_raw(name, t, float(row["roll_deg"]), float(row["pitch_deg"]), float(row.get("v_cms") or 0))
                        c.on_raw_evidence(name, t, float(row.get("ax") or 0.0), _fnum(row.get("dC_cm")),
                                          _fnum(row.get("lineC")))
                        c.on_raw_motion(name, t, _fnum(row.get("ax")), _fnum(row.get("ay")),   # (e16, 8c / 8a)
                                        str(row.get("lifted") or "0") not in ("0", "", "-1"))
                        self.raw_to_filter(name, t, row)
            except (ValueError, IndexError):
                continue

    def on_boot(self, name, boot):
        """(11-05h, Observe then trust: robot state) Its boot id changed: the robot restarted (a power dip, a sleep
        check-in), which wipes every `set` (the safety layers too) and its compass check (it then refuses to turn).
        Shared rule (observe.restarted): the safety layers are set again and the compass check run (a few degrees of
        wiggle, no spin, no roll) before its follow run starts again; said and logged."""
        import observe as OB
        import calib_room as CRM
        prev = self.boots.get(name)
        self.boots[name] = boot
        if prev is None or not OB.restarted({"boot": prev}, {"boot": boot}):
            return
        c = self.ctrl
        c.event(c.R[name], "restarted", "boot %s (was %s): safety layers set again, compass check, then its follow run"
                % (boot, prev))
        r = c.R[name]
        if r.move is not None:
            c.end_move(r)
        for cmd in safe_cmds(self.safe_edge_used) + ["set follow_look_cm %.1f" % FOLLOW_LOOK_CM] + \
                border_cmds(self.rect):
            self.send(name, cmd)
        self.send(name, "set calib_spin_deg 0")
        self.send(name, CRM.caps_cmd([(d, 0.0) for d in CRM.CAP_DEGS]))
        self.run_exp[name] = "follow"          # the calibrate's RUNEND starts the follow run again (drain)
        self.send(name, "calibrate")
        self.restarts[name] += 1

    def raw_to_filter(self, name, t, row):
        """(e09, B2) A RAW row to the robot's filter, in true units (raw_units), on this clock (its ms + offset)."""
        import fused_pose as FPm
        try:
            v, w = FPm.raw_true(row)
            ms = float(row.get("ms") or 0.0)
            o = self.offsets.get(name) or []
            tt = ms / 1000.0 + min(q[1] for q in o) if o else t
            still = abs(float(row.get("wl_rpm") or 0)) < FPm.STILL_RPM and abs(float(row.get("wr_rpm") or 0)) < \
                FPm.STILL_RPM
            line = tuple(_fnum(row.get(k)) for k in ("lineL", "lineC", "lineR"))
            self.ctrl.on_raw_full(name, tt, v, w, _fnum(row.get("gz")), still, _fnum(row.get("ax")),
                                  _fnum(row.get("dC_cm")), line)
        except (TypeError, ValueError):
            pass

    def fix_to_robot(self, name, t, x, y):
        """(e09, B2) The filter's position sent to the robot in a camera gap (its own pose drifts more)."""
        o = self.offsets.get(name) or []
        if not o:
            return
        off = min(v for tt, v in o)
        ms = int(round((t - off) * 1000))
        self.send(name, "fix %d %.2f %.2f - %.2f %.2f" % (ms, x, y, 2.0, 5.0))

    def pictures(self):
        """(e09, B4) Pictures the controller asked for (unsure near an edge with nobody there, a parked robot moved):
        each camera's newest picture into <session>/pictures/."""
        for p in self.ctrl.take_pictures():
            name = "%s_%s_%s" % (time.strftime("%H%M%S"), p["robot"], "".join(
                ch if ch.isalnum() else "-" for ch in p["why"])[:40].strip("-"))
            paths = self.frames.snapshot(os.path.join(self.dir, "pictures"), name) if self.frames is not None else []
            self.tables("pictures", dict(t=p["t"], robot=p["robot"], why=p["why"], files=";".join(
                os.path.basename(q) for q in paths) or "(no camera pictures: frames not saved in this run)"))
            if self.frames is not None:
                self.frames.event("other", p["why"], robot=p["robot"])

    @staticmethod
    def det_of(ev):
        """A bridge fix as a tracking-layer sighting: roof tags and lock-ons keep their name; outlines and lights are
        named again by the layer."""
        import tracking_layer as TL
        src = str(ev.get("src") or "")
        kind = src.split(":")[-1]
        th = ev.get("th")
        th = None if th is None else float(th)
        if kind == "ml":
            # (e13) the trained detector (tag_bridge --ml, off by default): an outline with no name, as motion
            return TL.Det("ml", float(ev["t"]), ev["x"], ev["y"], th, None, ev.get("sigma"))
        if src.startswith("tag_bridge") or kind == "tags":
            return TL.Det("tags", float(ev["t"]), ev["x"], ev["y"], th, ev.get("robot"), ev.get("sigma"))
        if kind == "lock":
            return TL.Det("lock", float(ev["t"]), ev["x"], ev["y"], th, ev.get("robot"), ev.get("sigma"))
        if kind == "lights":
            return TL.Det("lights", float(ev["t"]), ev["x"], ev["y"], th, None, ev.get("sigma"))
        return TL.Det("motion", float(ev["t"]), ev["x"], ev["y"], th, None, ev.get("sigma"))

    def fixes(self):
        c = self.ctrl
        now = self.AC.now()
        evs = self.source.poll()
        sd, sc = getattr(self.a, "sim_cam_delay", 0.0), getattr(self.a, "sim_cam_clock", 0.0)
        if self.a.dry and (sd or sc):
            # SIMULATOR ONLY: the camera's fixes held back sd s more and stamped on a clock sc s ahead (the Pi's real
            # pipeline: 0.45 s; a clock mismatch the run must discover)
            # (e11) its own list: self.held is the held robots (e08) and the two collided (TypeError)
            self.cam_held = getattr(self, "cam_held", []) + [(now + sd, dict(ev, t=float(ev["t"]) + sc)) for ev in evs
                                                             if ev.get("type") == "fix"]
            evs = [e for due, e in self.cam_held if due <= now]
            self.cam_held = [(due, e) for due, e in self.cam_held if due > now]
        try:
            self.place_poll()                         # (e13) a person's click on a held robot
        except Exception:  # noqa: BLE001
            pass
        try:                                         # (e13) which sources tracked each robot, each second
            sl = self.__dict__.get("srclog")
            if sl is None:
                import source_log as SL_
                sl = self.srclog = SL_.SourceLog(self.dir, list(c.R))
            for ev in evs:
                if ev.get("type") == "fix":
                    sl.add(ev.get("robot"), ev.get("src") or ("lights" if not getattr(self.a, "external_bridge", False)
                                                              else "tag_bridge"))
            sl.tick(time.time())
        except Exception:  # noqa: BLE001  (the log never stops the run)
            pass
        # (e12) the bridge's frame watch: no new picture from a camera (or pictures again): said loudly, an event;
        # the robots then run on the lost-by-the-camera rules as they are (bubble, find-me, hold) until fixes return
        for ev in evs:
            if ev.get("type") == "camera":
                w = str(ev.get("words") or "CAMERA %s: %s" % (ev.get("cam"), ev.get("state")))
                print(w, flush=True)
                self.camera_pictures = ev.get("state") != "no_pictures"
                c.on_camera_state(ev.get("state"))      # (e13) each robot's own risk-based limit from here
                if ev.get("state") == "dark":
                    # (e14, school 17:2x) the picture went dark: held at once (as if its no-pictures limit had passed)
                    c.on_camera_state("no_pictures")
                    c.nopic_since = c.t - 1e6
                elif ev.get("state") == "light_back":
                    c.on_camera_state("pictures")
                try:
                    for r in c.R.values():
                        c.event(r, "camera " + str(ev.get("state")), w)
                        break
                except Exception:   # noqa: BLE001
                    pass
        # (e09, B1) obstacles from the bridge's brick watch: kept out by every robot
        for ev in evs:
            if ev.get("type") == "obstacle" and ev.get("center") is not None:
                cx, cy = ev["center"][:2]
                c.on_obstacle(dict(x=float(cx), y=float(cy), r=float(ev.get("radius") or 6.0),
                                   src="camera: new object (brick watch)"))
        # (e10, C1) the camera as the start's discovery found it: none at all, or through the calibration it chose; a
        # turned roof tag's heading corrected; during the proof drive every named fix also goes to the discovery
        if self.camera_off:
            evs = [e for e in evs if e.get("type") != "fix"]
        for ev in evs:
            if ev.get("type") != "fix":
                continue
            if self.cam_fix is not None:
                ev["x"], ev["y"] = self.cam_fix(float(ev["x"]), float(ev["y"]))
                if self.cam_mirror and ev.get("th") is not None:
                    ev["th"] = 180.0 - float(ev["th"])
            if ev.get("th") is not None and ev.get("robot") in self.tag_yaw:
                ev["th"] = (float(ev["th"]) + self.tag_yaw[ev["robot"]] + 180.0) % 360.0 - 180.0
            if self.disc_drv is not None and ev.get("robot") in c.R:
                self.disc_drv.on_sighting(ev["robot"], float(ev["t"]) - c.clock, float(ev["x"]), float(ev["y"]),
                                          ev.get("th"), str(ev.get("src") or "camera"))
        # (e09, B1) one picture's fixes through the tracking layer (one batch per picture time)
        batch = {}
        for ev in evs:
            if ev.get("type") == "fix" and ev.get("robot") in c.R:
                batch.setdefault(round(float(ev["t"]), 3), []).append(ev)
        named = []
        for n, r in c.R.items():                      # (e17) who the run cannot account for (the layer's elimination)
            self.layer.set_unaccounted(n, c.unaccounted_why(r))
        for tp in sorted(batch):
            for ev in batch[tp]:
                if self.layer.T[ev["robot"]].sure is None:
                    # its first fix: the bridge found it at the start (lights, L0) and named it; the layer starts there
                    self.layer.seed(ev["robot"], tp, float(ev["x"]), float(ev["y"]), "start")
            for n, r in c.R.items():
                if r.own is not None:
                    oa = c.own_at(r, tp - c.clock) or r.own[1:4]      # its own pose at the picture's time
                    self.layer.own(n, tp, oa[0], oa[1], oa[2], r.own[4] == "idle")
            for f in self.layer.feed(tp, [self.det_of(ev) for ev in batch[tp]]):
                named.append(dict(type="fix", robot=f["robot"], t=f["t"], x=f["x"], y=f["y"], th=f["th"],
                                  sigma=f["sigma"], src=f["src"], sigma_th=3.0, how=f.get("how")))
        c.camera_proven = self.layer.proven
        for rv in self.layer.take_revoked():         # (e17) an elimination the robot's own drive disproved
            c.on_elim_revoked(rv)
        for pe in self.layer.take_parked():
            c.on_parked_moved(pe)
        for ev in named:
            n = ev.get("robot")
            th = ev.get("th")
            # (11-05c) the arrival time too: the controller measures the camera's delay and compares each fix with
            # the robot's own pose at the picture's time
            c.on_fix(n, float(ev["t"]), float(ev["x"]), float(ev["y"]), None if th is None else float(th),
                     arrival=now, sigma=ev.get("sigma"), src=ev.get("src"), how=ev.get("how"))
            if ev.get("how"):
                continue                              # (e17) a correction: the robot got an fpose, not a fix
            self.track.write("%.3f,%s,%.2f,%.2f,%s,%.3f\n" % (ev["t"], n, ev["x"], ev["y"], "" if th is None else
                                                               "%.1f" % th, now))
            o = self.offsets.get(n) or []
            if not o or now - self.fix_last[n] < 1.0 / FIX_HZ or c.R[n].lost is not None or \
                    not getattr(c.R[n], "fix_used", True):
                continue
            off = min(v for tt, v in o if tt > now - 30.0) if any(tt > now - 30.0 for tt, _ in o) else o[-1][1]
            ms = int(round((float(ev["t"]) - c.clock - off) * 1000))     # the picture's time on this clock
            self.send(n, "fix %d %.2f %.2f %s %.2f %.2f" % (ms, ev["x"], ev["y"], "-" if th is None else "%.2f" % th,
                                                            ev.get("sigma", 1.0), ev.get("sigma_th", 3.0)))
            self.fix_last[n] = now
            c.sent(n)

    def pump(self, s):
        end = time.time() + s
        while time.time() < end:
            self.drain()
            self.fixes()
            self.ctrl.t = self.AC.now()
            time.sleep(0.02)

    # -- start ---------------------------------------------------------------------------------------------------
    def start(self):
        c = self.ctrl
        say("Waiting for the robots: %s ..." % ", ".join(self.names))
        end = time.time() + 30
        while time.time() < end and not set(self.names) <= set(self.link.robots()):
            self.pump(0.3)
        missing = sorted(set(self.names) - set(self.link.robots()))
        if missing:
            say("Not heard from: %s: they stay out of the run." % ", ".join(missing))
            for n in missing:
                c.R[n].mode, c.R[n].out_why = "gone", "not heard at the start"
        live = [n for n in self.names if n not in missing]
        if not live:
            raise SystemExit("No robot answered.")
        for n in live:
            self.send(n, "stop")
        self.pump(1.0)
        safe = safe_cmds(self.safe_edge_used)
        for n in live:
            for cmd in safe + ["set follow_look_cm %.1f" % FOLLOW_LOOK_CM] + border_cmds(self.rect):
                self.send(n, cmd)
                time.sleep(0.01)
            self.send(n, "status")
        self.pump(2.0)
        # (11-05h, Nick 20:46) the robot-side safety layers (tilt stop, bottom-zone drop) READ BACK; a robot whose
        # layers do not read back after two tries stays out of the run
        import observe as OB
        want = OB.safety_settings(self.safe_edge_used)
        self.params_read = getattr(self, "params_read", {})
        for attempt in range(2):
            for n in live:
                for k in want:
                    self.params_read.setdefault(n, {}).pop(k, None)
                    self.send(n, "get %s" % k)
                    time.sleep(0.01)
            self.pump(2.0)
            bad = {n: OB.check_safety(want, self.params_read.get(n, {})) for n in live}
            bad = {n: v for n, v in bad.items() if v}
            if not bad:
                break
            for n, lst in bad.items():
                for k, v, g in lst:
                    self.send(n, "set %s %g" % (k, v))
            self.pump(0.5)
        for n in list(live):
            lst = OB.check_safety(want, self.params_read.get(n, {}))
            if lst:
                say("SAFETY: %s's safety layers did not read back (%s): it stays out of the run" % (
                    n, "; ".join("%s sent %g, read %s" % x for x in lst)))
                c.R[n].mode, c.R[n].out_why = "gone", "safety layers not read back"
                live.remove(n)
            else:
                say("  %s: safety layers on and read back (tilt stop %g deg, edge guard %g)" % (
                    n, want["tilt_stop_deg"], want["safe_edge"]))
                c.event(c.R[n], "safety armed", ", ".join("%s %g" % (k, v) for k, v in want.items()))
        if not live:
            raise SystemExit("No robot's safety layers read back: nothing moved.")
        for n in live:
            fw = self.fw.get(n) or "?"
            new = fw >= FC.FRESH_FW
            if new:
                self.send(n, "set follow_wd_ms %d" % FC.WD_MS)
            curves = c.motion_style == "curves" and fw >= FC.ARC_FW
            say("  %s: firmware %s, battery %s%%: %s; motion: %s" % (
                n, fw, c.R[n].battery if c.R[n].battery is not None else "?",
                "speed and steer changes while driving, watchdog on" if new else
                "OLDER FIRMWARE: speed and steer changes by halt + resend (a short stop each), no watchdog",
                "CURVES (never stops on purpose)" if curves else
                "stop-turn-drive (%s)" % ("chosen: stopturn" if c.motion_style == "stopturn" else
                                         "its firmware is older than %s: reflash for curves" % FC.ARC_FW)))
            c.event(c.R[n], "firmware", "%s (%s)" % (fw, "fspd/fsteer/watchdog" if new else "halt + resend"))
        for attempt in range(3):
            for n in live:
                if self.run_exp.get(n) != "follow":
                    self.send(n, "run follow")
            self.pump(3.0)
            if all(self.run_exp.get(n) == "follow" for n in live):
                break
        for n in live:
            if self.run_exp.get(n) != "follow":
                say("  %s did not start following: out of the run." % n)
                c.R[n].mode, c.R[n].out_why = "gone", "did not start following"
        live = [n for n in live if c.R[n].mode != "gone"]
        say("Waiting for the camera and each robot's own pose ...")
        # (e10) a bridge handed over by another runner (the school's tag bridge) may still be fitting its cameras
        end = time.time() + (90 if getattr(self.a, "external_bridge", False) else 25)
        told = {}
        while time.time() < end:
            self.pump(0.3)
            if getattr(self.a, "external_bridge", False):
                # (e10, C3) a robot handed over with no pose of its own yet (the school's runner tells each robot
                # where it is from its roof tag): told from its first tag fix, which carries the heading too
                for n in live:
                    r = c.R[n]
                    if r.own is None and r.cam is not None and r.cam[3] is not None and \
                            time.time() - told.get(n, 0.0) > 2.0:
                        told[n] = time.time()
                        self.send(n, "fpose %.2f %.2f %.2f 1.0 3.0" % (r.cam[1], r.cam[2], r.cam[3]))
            if all(c.R[n].cam is not None and c.R[n].own is not None for n in live):
                break
        if not getattr(self.a, "no_discovery", False):
            import start_discovery as D
            posed = [n for n in live if c.R[n].own is not None and c.R[n].cam is not None]
            if D.ENABLED and posed:
                self.discover(posed)     # (e10, C1)
            elif D.ENABLED:
                say("Discovery skipped: no robot has a camera fix and its own pose yet.")
        for n in live:
            r = c.R[n]
            if r.mode == "gone":
                say("  %s: %s: out of the run." % (n, r.out_why))
                continue
            if (r.cam is None and not self.camera_off) or r.own is None:
                say("  %s: %s: out of the run." % (n, "no camera fix" if r.cam is None else "no own pose"))
                r.mode, r.out_why = "gone", "not seen at the start"
                continue
            c.t = self.AC.now()
            c.begin(n, held=self.held.get(n))
            self.joined.add(n)
            q = r.cam if r.cam is not None else (None, r.own[1], r.own[2])
            say("  %s joins at %.1f, %.1f (facing %.0f deg)" % (n, q[1], q[2], r.own[3]))
        if not self.joined:
            raise SystemExit("No robot could join.")

    # -- the roam --------------------------------------------------------------------------------------------------
    def roam(self, hours):
        c = self.ctrl
        self.t0 = self.AC.now()
        t_end = self.t0 + hours * 3600.0
        last_disk = last_say = self.t0
        if self.frames is not None:
            self.frames.active(True, "freeroam")
        say("Free roam: %s, until each is at %d %% or %.1f h. Ctrl+C (or ./run-fleet-pi.sh freeroam-stop) stops "
            "every robot." % (", ".join(sorted(self.joined)), c.floor, hours))
        say(board_words(self.rect, "" if self.a.dry else getattr(self.a, "calib", ""), self.a.edge_rule))
        why = None
        while True:
            now = self.AC.now()
            self.drain()
            self.fixes()
            c.tick(now)
            self.pictures()                        # (e09, B4)
            if c.all_out():
                why = "every robot is out (battery %d %%, or silent)" % c.floor
                break
            if now >= t_end:
                why = "the time cap (%.1f h)" % hours
                break
            if self.guard is not None and now - last_disk > DISK_CHECK_S:
                last_disk = now
                st = self.frames.check_space() if self.frames is not None else self.guard.state()
                if st and st.get("stop"):
                    why = "the Pi's card is nearly full (%s)" % st.get("why")
                    break
            if now - last_say > 60.0:
                last_say = now
                say("[%7.1f] " % (now - self.t0) + "; ".join(
                    "%s %s %s%% %d moves" % (n, r.mode, r.battery if r.battery is not None else "?", r.n_moves)
                    for n, r in c.R.items()))
                self.write_state(why=None)
            time.sleep(max(0.0, TICK_S - (self.AC.now() - now)))
        say("\nThe run ends: %s." % why)
        c.event(None, "end", why)
        if os.environ.get("MIRLT_ARC_SAVE_AT_END", "1") != "0":
            # (e17, item 6: the 4-hour run ended on its time cap with 126-176 curve fits a robot, consistent, and
            # nothing saved: arc_save ran only when a robot parked at its battery floor) the same agreement rule at
            # any end, for every robot still in the run (one that parked has saved already)
            for r in c.R.values():
                if r.mode not in ("out", "gone"):
                    try:
                        c.arc_save(r)
                    except Exception:  # noqa: BLE001
                        pass
            time.sleep(0.3)
        return why

    def write_state(self, why):
        c = self.ctrl
        st = dict(version=VERSION, core=FC.VERSION, t0=self.t0, now=self.AC.now(), ended=why, seed=self.a.seed,
                  robots={n: dict(mode=r.mode, out=r.out_why, battery=r.battery, moves=r.n_moves, fw=self.fw.get(n),
                                  lost=r.lost is not None) for n, r in c.R.items()},
                  counts=c.counts, tracking=self.layer.summary(),
                  filters={n: (r.fp.summary() if r.fp is not None else None) for n, r in c.R.items()})
        ep = list(getattr(c, "edge_parked", None) or [])
        st["edge_parked"] = [q["words"] for q in ep]       # (e12) robots parked at an edge: someone must move them
        nh = c.needs_hand() if hasattr(c, "needs_hand") else []
        st["needs_hand"] = [q["words"] for q in nh]        # (e13) robots waiting for a hand now (they rejoin when moved)
        if ep or nh or os.path.exists(os.path.join(self.dir, "edge_parked.json")):
            write_edge_parked(self.dir, ep, nh)
        tmp = os.path.join(self.dir, "state.json.tmp")
        with open(tmp, "w") as f:
            json.dump(st, f, indent=1, default=str)
        os.replace(tmp, os.path.join(self.dir, "state.json"))

    def close(self):
        for n in self.names:
            try:
                self.link.send_name(n, "stop")
            except Exception:
                pass
        time.sleep(0.5)
        try:
            self.link.close()
        except Exception:
            pass
        self.source.close()
        self.tables.close()
        self.track.close()


def write_edge_parked(folder, ep, nh=()):
    """(e12) edge_parked.json in the run's folder: Mission Control's status box reads it (each robot left at an edge,
    where, why, and that someone must move it before the next run). (e13) nh: robots waiting for a hand NOW (they rejoin
    by themselves once moved in; the list empties when they do)."""
    try:
        tmp = os.path.join(folder, "edge_parked.json.tmp")
        with open(tmp, "w") as f:
            json.dump({"robots": ep, "words": [q["words"] for q in ep], "needs_hand": [q["words"] for q in nh],
                       "needs_hand_robots": list(nh)}, f, indent=1, default=str)
        os.replace(tmp, os.path.join(folder, "edge_parked.json"))
    except OSError:
        pass


def ml_detector_fn():
    """(e17, 4) The ML detector on demand: OFF (no model exists yet). With MIRLT_ML_ON_DEMAND=1 and a model file in
    MIRLT_ML_MODEL, a function the tracking layer calls only for robots that are unaccounted, unsure or asked for (a
    heading check), at a capped rate. Until a model and its bridge hook exist this returns None (the layer runs as
    without it), said once."""
    if os.environ.get("MIRLT_ML_ON_DEMAND", "0") != "1":
        return None
    path = os.environ.get("MIRLT_ML_MODEL", "")
    if not path or not os.path.exists(path):
        say("(MIRLT_ML_ON_DEMAND=1 but no detector model at MIRLT_ML_MODEL: the layer runs without it)")
        return None
    say("(MIRLT_ML_ON_DEMAND=1: model %s found, but the bridge hook that runs it on a picture is not built yet (e17 "
        "stub): the layer runs without it)" % path)
    return None


def stop_leads(a, names):
    """(e08, A5) each robot's stop lead {name: {speed: cm}}: --stop-lead NAME=SPEED:CM,... else the Pi's stop_leads.json
    (calib_run keeps it by WiFi name; at home the robots' names), else none (drives as planned)."""
    out = {}
    for spec in getattr(a, "stop_lead", None) or []:
        try:
            n, rest = spec.split("=", 1)
            out[n.strip()] = {float(x): float(y) for x, y in (kv.split(":") for kv in rest.split(",") if kv)}
        except ValueError:
            continue
    if a.dry or out:
        return out
    try:
        p = os.path.join(os.path.expanduser("~/.config/mirlt"), "stop_leads.json")
        recs = json.load(open(p))
        for n in names:
            sl = (recs.get(n) or {}).get("stop_lead_cm") or {}
            if sl:
                out[n] = {float(k): float(v) for k, v in sl.items()}
    except (OSError, ValueError, AttributeError):
        pass
    return out


def board_words(rect, calib="", edge_rule=None):
    """(11-05c) What board the planner assumes, said at the start (Nick 5 Nov: say what board size it assumes).
    (e08) and the edge rule in force."""
    import safety_rules as SR
    x0, y0, x1, y1 = rect
    src = os.path.basename(calib) if calib else "the simulator's board"
    head = "Board assumed: x %.0f to %.0f cm, y %.0f to %.0f cm (%.0f x %.0f cm, from %s). " % (
        x0, x1, y0, y1, x1 - x0, y1 - y0, src)
    tail = ("Walls or objects inside it are found during the run: a front stop felt twice, or an edge-guard or tilt "
            "stop, becomes a keep-out; one felt once fades after %.0f s (events: keep-out; keepouts.csv)."
            % SR.UNCONFIRMED_S)
    if (edge_rule or SR.EDGE_RULE) == "fixed":
        return head + ("Edge rule: fixed (before e08). Moves end %.0f cm or more inside it (%.0f x %.0f cm usable); "
                       "turns on the spot %.1f cm in. " % (FC.EDGE_CC, x1 - x0 - 2 * FC.EDGE_CC, y1 - y0 - 2 * FC.EDGE_CC,
                                                           FC.TURN_EDGE_CC)) + tail
    return head + ("Edge rule: derived (e08): a robot's middle stays half its diagonal (6.7 cm) + its position "
                   "uncertainty now + the camera's edge error (%.0f cm) from every edge along the whole path, plus its "
                   "stopping distance where it stops: about %.1f cm along a path and %.1f cm where a move ends when "
                   "the camera and the robot agree, more when they do not. " % (
                       SR.tracking_bias_cm("lights"), SR.clearance_cm(1.0, None, "lights", False),
                       SR.clearance_cm(1.0, 13.0, "lights", True))) + tail


def loopback_up():
    """In a fresh network namespace (unshare -rn) the loopback starts down (calib_run.py / sweeps/rp_shim.py's way);
    a no-op where it is up already."""
    import fcntl
    import struct
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        ifr = struct.pack("16sH14s", b"lo", 0, b"\0" * 14)
        flags = struct.unpack("16sH14s", fcntl.ioctl(s, 0x8913, ifr))[1]
        if not flags & 1:
            fcntl.ioctl(s, 0x8914, struct.pack("16sH14s", b"lo", flags | 1, b"\0" * 14))
    except OSError:
        pass
    finally:
        s.close()


def border_cmds(rect):
    """The robots' own four-sided border 6 cm inside the board (run_plan.border_commands on a plain map)."""
    try:
        import run_plan as RP
        m = {"format": "alvik-fleet-map", "version": 1,
             "floor": {"x_min": rect[0], "y_min": rect[1], "x_max": rect[2], "y_max": rect[3]}, "robots": [],
             "tasks": {}}
        import fleetmap as FM
        return RP.border_commands(FM.normalize(m), RP.BORDER_CM)
    except Exception as e:      # noqa: BLE001  (the edge guard still stands)
        say("  (the robots' own border was not set: %s)" % e)
        return []


# ---------------------------------------------------------------------------------------------- the dry board
def dry_board(a, outdir, names):
    rng = random.Random(a.seed)
    x0, y0, x1, y1 = HOME_BOARD
    while True:
        pts = [(rng.uniform(30, x1 - 30), rng.uniform(30, y1 - 30)) for _ in names]
        if all(math.dist(pts[i], pts[j]) >= 40 for i in range(len(pts)) for j in range(i + 1, len(pts))):
            break
    starts = {n: [round(p[0], 1), round(p[1], 1), round(rng.uniform(-180, 180), 1)] for n, p in zip(names, pts)}
    for spec in getattr(a, "sim_start", None) or []:
        # (e12) --sim-start NAME=X,Y,HEADING: that robot starts there (e.g. at an edge, facing out: the rehearsal of
        # one robot stuck at an edge while the others roam)
        n_, _, v_ = spec.partition("=")
        if n_ in starts:
            starts[n_] = [float(q) for q in v_.split(",")][:3]
    sd = os.path.join(outdir, "sim")
    os.makedirs(sd, exist_ok=True)
    m = {"format": "alvik-fleet-map", "version": 1, "floor": {"x_min": x0, "x_max": x1, "y_min": y0, "y_max": y1},
         "robots": [{"name": n, "start": s} for n, s in starts.items()], "tasks": {}}
    mp = os.path.join(sd, "dry_map.json")
    json.dump(m, open(mp, "w"), indent=1)
    rate = 90.0 / (a.sim_minutes / 60.0)          # moving: 100 % -> 10 % in --sim-minutes of driving
    cfg = {"robot": {"battery_start_pct": [100.0, 100.0], "battery_moving_pct_per_h": rate,
                     "battery_idle_pct_per_h": rate / 4.0}}
    cp = os.path.join(sd, "sim_config.json")
    json.dump(cfg, open(cp, "w"), indent=1)
    cmd = [PY, os.path.join(TOOLS, "sim", "fleet_sim.py"), "--map", mp, "--out", sd, "--seed", str(a.seed), "--quiet",
           "--fixes-to", "127.0.0.1:%d" % a.fix_port, "--fix-hz", "%g" % float(getattr(a, "sim_fix_hz", 12.0) or 12.0), "--fix-delay", "0.15", "--fix-sigma", "0.5",
           "--config", cp, "--max-s", str(int(a.hours * 3600 + 300))]
    gaps = []
    T = a.hours * 3600.0
    for n in names:                                # tag dropouts: about one per robot every 6 min, 3-20 s long
        t = rng.uniform(60, 360)
        while t < T - 30:
            d = rng.uniform(3, 20)
            gaps.append("%s@%.0f:%.0f" % (n, t, d))
            t += d + rng.uniform(200, 520)
    for g in gaps:
        cmd += ["--cam-gap", g]
    for fw in a.sim_fw:
        cmd += ["--fw", fw]
    for s in a.sim_silent:
        cmd += ["--silent", s]
    for w in a.sim_wifi_drop:
        cmd += ["--wifi-drop", w]
    json.dump(dict(starts=starts, gaps=gaps, config=cfg, cmd=cmd), open(os.path.join(sd, "dry_setup.json"), "w"),
              indent=1)
    p = subprocess.Popen(cmd, stdout=open(os.path.join(sd, "sim_stdout.txt"), "w"), stderr=subprocess.STDOUT,
                         start_new_session=True)
    time.sleep(2.0)
    return p, ["127.0.0.%d" % (2 + k) for k in range(len(names))], gaps


# ---------------------------------------------------------------------------------------------- the real start
def real_start(a, outdir):
    """L0 + L1 of the remote ladder (as the DOE loop starts), a heading for every robot, then the camera bridge.
    Returns (names, ips, bridge process, frames, guard)."""
    import ladder as LD
    import diskguard as DG
    from ladder import LadderStop
    a.dir = outdir
    L = LD.Ladder(a)                  # a.dry (--dry-chain): the ladder's simulated board (pictures, not fixes)
    a._ladder = L
    st = DG.Guard(card=outdir).state()
    if st["stop"]:
        raise SystemExit("NOT STARTED: %s. Nothing moved. Free space first (./run-fleet-pi.sh space)." % st["why"])
    if not L.run_step("L0", in_order=True):
        raise SystemExit("L0 found no robot that can go on: nothing moved.")
    if not any(L.st["passed"].get("L1:%s" % n) for n in L.active()):
        L.run_step("L1", in_order=True)
    names = [n for n in L.active("L1")]
    if not names:
        raise SystemExit("No robot passed L1 (calibration).")
    sub = L.sub("headings")
    for n in names:
        if L.heading(n) is None:
            say("  %s: heading not known: heading crawl first" % n)
            try:
                L.need_bench(sub)
                L.need_eye()
                q0 = L.eye.still(n) or L.pos(n)
                h, _ = L.crawl(n, q0, None, **L.hint_for(n, q0))
                q = L.eye.still(n) or q0
                L.set_pose(n, q[0], q[1], h, "crawl")
            except LadderStop as e:
                if getattr(a, "heading_out", False):
                    say("  %s: no heading (%s): it stays out of the run, parked where it is." % (n, e))
                    L.keep_out(n, "no heading for the free roam: %s" % e)
                else:
                    # (e08, A7; 6 Nov: Right parked for the whole run after the camera re-found it 15 cm from its first
                    # placement: a camera mis-placement, not its wheels) it joins HELD as an obstacle; the controller
                    # finds its heading by a nudge the camera watches and it rejoins
                    q = L.eye.still(n) if getattr(L, "eye", None) else None
                    q = q or L.pos(n)
                    h = getattr(e, "heading", None)
                    L.set_pose(n, q[0], q[1], h if h is not None else 0.0, "crawl failed: held")
                    a._held = dict(getattr(a, "_held", None) or {}, **{n: "heading"})
                    say("  %s: no heading (%s): it joins HELD as an obstacle; a short nudge the camera watches will "
                        "find its heading, then it rejoins." % (n, e))
    names = [n for n in L.active("L1") if L.heading(n) is not None]
    rect = tuple(L.rect)
    L.drop_bench()
    L.drop_eye()
    m = {"format": "alvik-fleet-map", "version": 1,
         "floor": {"x_min": rect[0], "y_min": rect[1], "x_max": rect[2], "y_max": rect[3]},
         "robots": [{"name": n, "start": [*L.pos(n), L.heading(n)]} for n in names], "tasks": {},
         "notes": "home free roam"}
    mp = os.path.join(outdir, "map.json")
    json.dump(m, open(mp, "w"), indent=1)
    bcmd = bridge_pin() + [PY, os.path.join(HERE, "cam_bridge.py"), "--map", mp, "--calib", L.calib, "--stream", L.stream,
            "--out", os.path.join(outdir, "bridge")]
    for n in names:
        x, y = L.pos(n)
        bcmd += ["--at", "%s=%.2f,%.2f,%.2f" % (n, x, y, L.heading(n))]
    if L.side_calib:
        bcmd += ["--side-stream", L.side_stream, "--side-calib", L.side_calib]
        kb = L.kept_bias()
        if kb is not None:
            bcmd += LD.side_bias_args(kb)
    if a.no_lights:
        bcmd += ["--no-lights"]
    if not getattr(a, "no_brick_watch", False):
        bcmd += ["--brick-watch"]          # (e09) the "what changed" watch on in free roam (7 Nov audit: never on)
    if getattr(a, "old_tracking", False):
        bcmd += ["--no-lock"]              # (e09) the lock-on off: the tracking before e09
    bf = open(os.path.join(outdir, "bridge.txt"), "w")
    br = subprocess.Popen(["nice", "-n", "5"] + bcmd, stdout=bf, stderr=subprocess.STDOUT)
    say("Camera bridge started (%s)." % ", ".join(names))
    time.sleep(5.0)
    frames = guard = None
    if not a.no_frames and not a.dry:
        try:
            import doe_loop as DL
            fdir, _ = DG.frames_dir_for(outdir)
            guard = DG.Guard(card=outdir, frames_dir=fdir)
            ho_, hs_ = DL.frames_rates(LD.Eye.OVERHEAD_HZ, DL.SIDE_HZ)     # (e17) default 4 a second (MIRLT_FRAMES_HZ)
            streams = [("overhead", L.stream, ho_)]
            if L.side_stream:
                streams.append(("side", L.side_stream, hs_))
            frames = DL.Frames(outdir, streams, frames_dir=fdir, guard=guard)
        except Exception as e:     # noqa: BLE001  (the run goes on without frames)
            say("  (frames not saved: %s)" % e)
    if guard is None:
        guard = DG.Guard(card=outdir)
    return names, L.ips, br, frames, guard, rect


# ---------------------------------------------------------------------------------------------- status
def latest_dir(dry=False):
    ds = sorted(glob.glob(os.path.join(REPO, "logs", "*-freeroam" + ("-dryrun" if dry else ""))))
    return ds[-1] if ds else None


def status(folder):
    if not folder or not os.path.isdir(folder):
        say("No free-roam folder found.")
        return 1
    p = os.path.join(folder, "state.json")
    if not os.path.exists(p):
        say("%s: starting (no state yet)." % folder)
        return 0
    try:
        with open(p) as f:
            st = json.load(f)
    except (OSError, ValueError) as e:
        say("%s: its state.json could not be read (%s): it may be being written; try again." % (folder, e))
        return 0
    robots = st.get("robots") if isinstance(st.get("robots"), dict) else {}
    if "t0" not in st and not any(isinstance(r, dict) and "mode" in r for r in robots.values()):
        # (e12, 10 Nov: KeyError 'mode') the start-up's ladder (L0 + L1) keeps ITS state.json in the same folder until
        # the free roam writes its own: the run is still starting (or stopped during the start)
        say("Free roam %s: starting (the start-up checks: finding the robots, their headings, the camera). Robots so "
            "far: %s" % (os.path.basename(folder), ", ".join(sorted(robots)) or "none yet"))
        return 0
    mins = (st["now"] - st["t0"]) / 60.0 if st.get("t0") and st.get("now") else 0.0
    say("Free roam %s: %.0f min%s" % (os.path.basename(folder), mins, "; ENDED: %s" % st["ended"] if st.get("ended")
                                       else ""))
    for n, r in robots.items():
        r = r if isinstance(r, dict) else {}
        say("  %-7s %-8s battery %s%%  moves %s  %s%s" % (n, r.get("mode", "?"), r.get("battery", "?"),
                                                       r.get("moves", "?"),
                                                       "firmware %s" % r["fw"] if r.get("fw") else "",
                                                       ("  out: %s" % r["out"]) if r.get("out") else ""))
    for w in st.get("edge_parked") or []:
        say("  " + str(w))                 # (e12) a robot parked at an edge: someone must move it
    for w in st.get("needs_hand") or []:
        say("  !! " + str(w))              # (e13) waiting for a hand now: it rejoins once moved in
    cn = st.get("counts", {})
    say("  reflex events %s; fallbacks %s; full stops (trophy breaks) %s" % (
        cn.get("reflex"), cn.get("fallback"), cn.get("breaks")))
    return 0


# ---------------------------------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["run", "status", "report"])
    ap.add_argument("folder", nargs="?", default="")
    ap.add_argument("--hours", type=float, default=4.0, help="time cap (Nick: 4 h)")
    ap.add_argument("--battery-floor", type=int, default=None, help="(e14 default: MIRLT_MIN_BATTERY, else Mission "
                    "Control's Advanced 'Battery floor', else the site file's battery_floor, else free roam's %d %%)"
                    % FC.BATTERY_FLOOR)
    ap.add_argument("--lost-mode", choices=("reckon", "hold"), default="reckon",
                    help="reckon (default): dead reckoning with a growing bubble; hold: the old cautious halt")
    ap.add_argument("--no-sleep", action="store_true", default=True, help="(default since 11-05h) a parked robot is "
                    "stopped and stays awake (a wake restarts it and wipes values not saved on it)")
    ap.add_argument("--sleep-parked", dest="no_sleep", action="store_false", help="(the old way, kept) a parked "
                    "robot is put to sleep")
    ap.add_argument("--attended", action="store_true", help="(11-05h) a person is in the room with the robots: "
                    "required on a site with attended_only (school, Jay-Sun 5 Nov)")
    ap.add_argument("--safe-edge", type=int, default=None, choices=[0, 1, 2, 3], help="(11-05h) the edge guard: 3 both "
                    "cues (home, default), 2 the bottom zone's drop only (school: the seam tape trips the line sensors)")
    ap.add_argument("--motion", choices=("curves", "stopturn"), default="curves",
                    help="(5 Nov) curves (default: continuous curving pieces on firmware 2026-11-05c+, older robots "
                         "stop-turn-drive) or stopturn (everyone as before)")
    # (e08 Phase A) the new rules, each with the old one behind its switch
    ap.add_argument("--edge-rule", choices=("derived", "fixed"), default=None,
                    help="(e08, A2) derived (default: half diagonal + uncertainty + stop distance + camera edge bias, on "
                         "the whole path) or fixed (the 16 cm end / 9.2 cm path rule before e08)")
    ap.add_argument("--pose-rule", choices=("widen", "wait2s"), default=None,
                    help="(e08, A1) widen (default: own pose vs camera widens the uncertainty at once) or wait2s (before)")
    ap.add_argument("--keepout-rule", choices=("fade", "permanent"), default=None,
                    help="(e08, A6) fade (default: sensor-only keep-outs fade after 15 s) or permanent (before)")
    ap.add_argument("--silent-out", action="store_true",
                    help="(e08, A7: the old way) a robot silent 120 s is out for the rest of the run (default: held, "
                         "checked, rejoins)")
    ap.add_argument("--heading-out", action="store_true",
                    help="(e08, A7: the old way) a robot whose heading crawl fails stays out (default: joins held)")
    ap.add_argument("--pose-source", choices=("ekf", "blend"), default=None,
                    help="(e09, B2) the robots' poses from one filter per robot (ekf) or the one-step blend as before "
                         "with the filter beside it, logged (blend, the default after e09); MIRLT_POSE_SOURCE sets it too")
    ap.add_argument("--gyro-heading", action="store_true",
                    help="(e09, B3, OFF by default) the heading from the raw gyro rate, its bias taken at every "
                         "standstill; MIRLT_GYRO_HEADING=1 sets it too")
    ap.add_argument("--old-tracking", action="store_true",
                    help="(e09, B1) the tracking before e09: the bridge's names by prediction, no close-pass guard, no "
                         "camera watch, no parked watch")
    ap.add_argument("--no-brick-watch", action="store_true",
                    help="(e09) the camera's \"what changed\" watch off (on by default since e09)")
    ap.add_argument("--no-discovery", action="store_true",
                    help="(e10, the switch) no proof drive at the start: the camera and the typed board as configured")
    ap.add_argument("--typed-board", action="store_true",
                    help="(e10, the switch) plan on the typed board's rectangle, not the proven workspace")
    ap.add_argument("--probe-ahead", action="store_true",
                    help="(e10, a setting, off by default) robots may drive slowly into ground nobody has proven yet, "
                         "their own edge sensors guarding them, and prove it")
    ap.add_argument("--tracking", choices=("lights", "tags"), default="lights",
                    help="(e10, C3) the site's camera sources: lights (home) or roof tags (school, with motion and "
                         "lock-on); the same controller either way")
    ap.add_argument("--no-shuffle", action="store_true", help="(e10, the switch) no corner shuffle: a robot in a corner "
                    "trap stands, as before")
    ap.add_argument("--picture-every-s", type=float, default=None,
                    help="(e10) at most one 'unsure near an edge' picture per robot per this many s (default 60)")
    ap.add_argument("--careful", action="store_true", help="(e10, preferences: very careful) only ground the camera "
                    "sees this run: no felt ground from earlier runs, no probing")
    ap.add_argument("--site", default="home", help="(e10) the site's name: the workspace is saved per site")
    ap.add_argument("--external-bridge", action="store_true",
                    help="(e10, C3) robots (--robot IP, --robots names) and a camera bridge already set up by another "
                         "runner (mission_run at a roof-tag site): no ladder start, the bridge's fixes on --fix-port")
    ap.add_argument("--board", default="", help="(e10) x0,y0,x1,y1 cm: the board's outline (--external-bridge)")
    ap.add_argument("--readable-map", default="", help="(e13) the run's readability map (observe.ReadMap JSON): plan "
                    "only where the camera reads roof tags (MIRLT_ROAM_READABLE=0: the whole proven table)")
    ap.add_argument("--alias", action="append", default=[], metavar="ANSWERS_AS=RUN_NAME",
                    help="(e11, 9 Nov) a robot that answers with its WiFi name (e.g. AL-1DC0) is called RUN_NAME (its "
                         "slot, e.g. Alpha) in this run: the site keeps the robots' names (tags.rename_to_slot false)")
    ap.add_argument("--practice-real", action="store_true",
                    help="(e11) --sim-cmd: the simulated robots on the REAL path (not marked dry: the same code as at "
                         "school; they still are simulated)")
    ap.add_argument("--sim-cmd", default="", help="(e10, practice) --external-bridge: start these simulated robots "
                    "(a JSON list: the fleet_sim command run_plan would use)")
    ap.add_argument("--conf-dir", default=os.path.expanduser("~/.config/mirlt"),
                    help="(e10) where the per-site workspace and the per-robot calibration are kept on the Pi")
    ap.add_argument("--no-tilt-recovery", action="store_true",
                    help="(e08, A3: the old way) a tilted robot with no room to back away stands")
    ap.add_argument("--stop-lead", action="append", default=[], metavar="NAME=SPEED:CM,...",
                    help="(e08, A5) a robot's stop lead (default: the Pi's stop_leads.json)")
    ap.add_argument("--no-arc-save", action="store_true", help="(5 Nov) do not save the curves' calibration found "
                    "in the run on a robot when it parks")
    ap.add_argument("--seed", type=int, default=None, help="the menu's seed (default: from the clock; recorded)")
    ap.add_argument("--robots", default="Left,Center,Right")
    ap.add_argument("--dir")
    ap.add_argument("--dry", action="store_true", help="tools/sim/fleet_sim.py instead of the robots and camera")
    ap.add_argument("--dry-chain", action="store_true",
                    help="the real start (ladder L0 + L1, heading crawls, the camera bridge on simulated pictures) "
                         "against the ladder's simulated board: the whole chain, slower")
    ap.add_argument("--sim-minutes", type=float, default=200.0,
                    help="--dry: minutes of driving from 100 %% to 10 %% (time-compressed battery)")
    ap.add_argument("--reflex-drill", type=float, default=0.0, metavar="SHARE",
                    help="--dry only: this share of moves ignores the others' reservations, so the reflex has to act "
                         "(refused on the robots)")
    ap.add_argument("--sim-fw", action="append", default=[], metavar="NAME=VERSION")
    ap.add_argument("--sim-silent", action="append", default=[], metavar="NAME@T:DUR")
    ap.add_argument("--sim-start", action="append", default=[], metavar="NAME=X,Y,HEADING",
                    help="(e12) --dry: that robot starts there (e.g. 3,60,180: at the x = 0 edge, facing out)")
    ap.add_argument("--sim-wifi-drop", action="append", default=[], metavar="NAME@T:DUR")
    ap.add_argument("--discovery-strict", action="store_true",
                    help="(e10's way, kept off) a camera discovery cannot prove is switched off and only felt ground "
                         "is used; default (e11): the typed board, said loudly")
    ap.add_argument("--sim-fix-hz", type=float, default=12.0, metavar="HZ",
                    help="(e11) the simulated camera's fixes a second per robot (the 9 Nov home Pi gave about 4-5)")
    ap.add_argument("--sim-cam-delay", type=float, default=0.0, metavar="S",
                    help="--dry: hold every camera fix back this many s more (the real Pi: ~0.45 s in all)")
    ap.add_argument("--sim-cam-clock", type=float, default=0.0, metavar="S",
                    help="--dry: stamp the camera's fixes on a clock this many s ahead of the controller's")
    ap.add_argument("--fix-port", type=int, default=5010)
    ap.add_argument("--no-frames", action="store_true")
    # the ladder's connection (the real start; as doe_loop.py)
    ap.add_argument("--robot", action="append", default=[])
    ap.add_argument("--stream", default="127.0.0.1:8000")
    ap.add_argument("--calib", default=os.path.join(REPO, "tools", "experiment", "data", "overhead_floorcal_roam1.json"))
    ap.add_argument("--side-stream", default="127.0.0.1:8001")
    ap.add_argument("--side-calib", default="")
    ap.add_argument("--no-lights", action="store_true")
    ap.add_argument("--no-facing", action="store_true")
    ap.add_argument("--ignore", default="")
    # (the ladder's --attended, same meaning: a person is there; declared once above since 11-05h)
    ap.add_argument("--full", action="store_true")
    ap.add_argument("--day-state", default="")
    ap.add_argument("--turn-err", action="append", default=[])
    ap.add_argument("--sim-washout", action="store_true")
    ap.add_argument("--sim-side-bias", default="")
    ap.add_argument("--sim-no-side", action="store_true")
    ap.add_argument("--sim-calibrated", action="store_true")
    ap.add_argument("--sim-place", action="append", default=[])
    ap.add_argument("--sim-from", default="")
    a = ap.parse_args(argv)
    if a.what == "status":
        return status(a.folder or a.dir or latest_dir(a.dry) or latest_dir(True))
    if a.what == "report":
        import freeroam_report as FR
        say(FR.make(a.folder or a.dir or latest_dir(a.dry)))
        return 0
    if a.seed is None:
        a.seed = int(time.time()) % 100000
    if a.reflex_drill and not (a.dry or a.dry_chain):
        say("--reflex-drill is for the simulator only: refused (nothing started).")
        return 2
    names = [n.strip() for n in a.robots.split(",") if n.strip()]
    outdir = a.dir or os.path.join(REPO, "logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") + "-freeroam" +
                                   ("-dryrun" if a.dry else ""))
    os.makedirs(outdir, exist_ok=True)

    def hup(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGHUP, hup)
    signal.signal(signal.SIGTERM, hup)
    sim = br = frames = guard = None
    runner = None
    why = "stopped before the roam"
    code = 0
    cpu = None
    if not a.dry and not a.attended and not getattr(a, "sim_cmd", ""):     # (e10) --sim-cmd: simulated robots
        try:
            import sites as SI
            st_ = SI.load(None)
        except (Exception, SystemExit):  # noqa: BLE001
            st_ = {}
        if st_.get("attended_only"):
            say("NOT STARTED: site %s is attended only (Jay-Sun 5 Nov: the robots drive only while he is in the room; "
                "no unattended or overnight driving). With him there, start again with --attended. Nothing moved."
                % st_.get("name"))
            return 2
    if a.battery_floor is None:
        try:
            import engine_checks as EC_
            a.battery_floor, bw_ = EC_.battery_floor_used("freeroam")
        except Exception as e:  # noqa: BLE001
            a.battery_floor, bw_ = FC.BATTERY_FLOOR, "%d %% (free roam's; %s)" % (FC.BATTERY_FLOOR, e)
    else:
        bw_ = "%d %% (--battery-floor)" % a.battery_floor
    say("BATTERY FLOOR: %s" % bw_)
    say("Free roam: %s (seed %d, battery floor %d %%, lost: %s)" % (outdir, a.seed, a.battery_floor, a.lost_mode))
    import safety_rules as SR
    run_info = dict(version=VERSION, core=FC.VERSION, motion=a.motion, seed=a.seed, hours=a.hours, battery_floor=a.battery_floor,
                    rules=dict(safety_rules=SR.VERSION, edge=a.edge_rule or SR.EDGE_RULE, pose=a.pose_rule or SR.POSE_RULE,
                               keepouts=a.keepout_rule or SR.KEEPOUT_RULE, silent_out=a.silent_out,
                               tilt_recovery=not a.no_tilt_recovery, tilt=SR.tilt_limits(), attended=a.attended,
                               queue_watch=FC.QUEUE_WATCH, mark_unc=FC.MARK_UNC, start_edge=FC.START_EDGE,
                               start_edge_mode=FC.START_EDGE_MODE),
                    tracking=dict(layer=__import__("tracking_layer").VERSION, filter=__import__("fused_pose").VERSION,
                                  pose_source=a.pose_source or __import__("fused_pose").POSE_SOURCE,
                                  gyro_heading=bool(a.gyro_heading), old_tracking=bool(a.old_tracking),
                                  brick_watch=not a.no_brick_watch),
                    lost_mode=a.lost_mode, dry=a.dry, reflex_drill=a.reflex_drill, started=dt.datetime.now().isoformat(timespec="seconds"),
                    argv=sys.argv[1:])
    try:
        if a.dry_chain:
            loopback_up()
            a.dry = True
            names, ips, br, frames, guard, rect = real_start(a, outdir)
            pose_to = ("127.0.0.1", 5011)
        elif a.dry:
            loopback_up()              # run inside `unshare -rn` (its own network namespace): the loopback starts down
            sim, ips, gaps = dry_board(a, outdir, names)
            run_info["sim_gaps"] = gaps
            rect = HOME_BOARD
            pose_to = None
        elif getattr(a, "external_bridge", False):
            # (e10, C3: one controller at every site) the robots and a camera bridge already set up by another runner
            # (mission_run at a roof-tag site: robots found by their wiggle, the feel-out, the tag bridge with its
            # lock-on sending to --fix-port and reading poses on 5011): this free-roam controller drives them
            ips = list(a.robot)
            br, frames, guard = None, None, None
            if a.sim_cmd:
                # (practice) the simulated robots, as run_plan would start them (mission_run's dry school free roam)
                loopback_up()
                br = subprocess.Popen(json.loads(a.sim_cmd), cwd=REPO, stdout=open(os.path.join(outdir, "sim_stdout.txt"),
                                                                                  "w"), stderr=subprocess.STDOUT)
                time.sleep(1.5)
                if getattr(a, "practice_real", False):
                    run_info["practice_real"] = True      # (e11) the real path, simulated robots: not marked dry
                else:
                    a.dry = True
            rect = tuple(float(v) for v in a.board.split(",")) if a.board else HOME_BOARD
            pose_to = ("127.0.0.1", 5011)
        else:
            names, ips, br, frames, guard, rect = real_start(a, outdir)
            pose_to = ("127.0.0.1", 5011)
        run_info.update(robots=names, rect=rect)
        json.dump(run_info, open(os.path.join(outdir, "run.json"), "w"), indent=1)
        if not a.dry:
            # (11-05h, CLAUDE.md heat tracker) cpu.csv, as the missions have
            import calib_run as CRN
            cpu = CRN.start_cpu_log(outdir, {"freeroam": os.getpid()}, {"bridge": "cam_bridge.py",
                                                                        "recorder": "camrec.py"})
        runner = Runner(a, outdir, names, ips, rect, fix_port=a.fix_port, pose_to=pose_to, frames=frames, guard=guard)
        runner.start()
        why = runner.roam(a.hours)
    except KeyboardInterrupt:
        why = "stopped by hand (Ctrl+C, freeroam-stop, or the tmux session closed)"
        say("\n%s: every robot is sent stop." % why)
        code = 130
    except SystemExit as e:
        why = "STOPPED: %s" % e
        say(why)
        code = 1
    finally:
        if cpu is not None:
            try:
                cpu.stop()
            except Exception:  # noqa: BLE001
                pass
        if runner is not None:
            runner.write_state(why)
            runner.save_workspace()            # (e10, C2) the proven workspace, kept for this site
            runner.close()
        if br is not None:
            try:
                br.send_signal(signal.SIGINT)
                br.wait(timeout=8)
            except Exception:
                br.kill()
        if frames is not None:
            say("Frames: %s" % frames.words())
            frames.close()
        if getattr(a, "_ladder", None) is not None:
            # (e12, 9 Nov home 12:52: "FATAL: exception not rethrown" right after "L0 found no robot") the start-up
            # ladder's camera threads (its Eye, lights keeper: OpenCV in threads) were left running at exit: stopped
            try:
                a._ladder.drop_eye()
            except Exception:  # noqa: BLE001
                pass
            if a.dry:
                try:
                    a._ladder.stop_dry()
                except Exception:
                    pass
        if sim is not None:
            try:
                os.killpg(sim.pid, signal.SIGTERM)
                sim.wait(timeout=20)
            except Exception:
                pass
        ep = list(getattr(getattr(runner, "ctrl", None), "edge_parked", None) or [])
        if ep:
            # (e12, Nick 10 Nov) a robot left at an edge: said loudly at the very end, where the person looks
            say("\n" + "!" * 100)
            for q in ep:
                say("!! " + q["words"])
            say("!" * 100)
            run_info["edge_parked"] = ep
        nh = list(getattr(getattr(runner, "ctrl", None), "needs_hand", lambda: [])() or [])
        if nh:
            # (e13) still waiting for a hand when the run ended (its cap, or the others' batteries)
            say("\n" + "!" * 100)
            for q in nh:
                say("!! STILL WAITING FOR A HAND AT THE END: " + q["words"])
            say("!" * 100)
            run_info["needs_hand"] = nh
        run_info.update(ended=why, finished=dt.datetime.now().isoformat(timespec="seconds"))
        json.dump(run_info, open(os.path.join(outdir, "run.json"), "w"), indent=1)
        try:
            import freeroam_report as FR
            say("Report: %s" % FR.make(outdir))
        except Exception as e:     # noqa: BLE001
            say("(report not made: %s)" % e)
        say("Free roam folder: %s" % outdir)
    return code


if __name__ == "__main__":
    import idle_after_run as IA     # (e13) robots asleep 10 min after the run ends if no new run starts
    with (IA.track("freeroam") if sys.argv[1:2] == ["run"] else IA.contextlib.nullcontext()):
        _code = main()
    import orderly_exit as OX      # (e12) no thread torn down inside OpenCV at exit (the 9 Nov abort)
    OX.finish(_code)
