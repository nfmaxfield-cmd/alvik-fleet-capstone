#!/usr/bin/env python3
"""The DOE loop (MIRLT, 2026-10-27 evening): the remote ladder's machinery left running unattended on the Pi.

Nick starts it and walks away. It runs until every robot is out (battery, a FAIL) or the time cap:

  start   L0 (find the robots, battery, firmware) and L1 (calibrate), exactly as the ladder does.
  check   before EVERY loop and EVERY robot move, each robot's `status` (battery), `clock` (boot id: a new one means it
          restarted, e.g. a sleep check-in, 15:27 today) and `get` of gyro_scale / wheel_scale / gyro_drift_dps:
            - restarted: `wake`, the edge guard and the stored corrections again, then calibrate again (L1, alone);
            - battery under 25 %: parked (left where it is, on every map as an obstacle);
            - a correction that does not read back (Center's lost wheel_scale, 14:12): sent again; still wrong: parked.
  loop    one robot at a time (round robin; the others parked as obstacles):
            a. every TURN_EVERY-th drive of a robot (and its first): the ladder's camera-measured turn check (4 quarter
               turns, per-turn error);
            b. a planned straight out-and-back in the middle of the board: to A, A->B, B->A, at speeds cycling
               5 / 7.5 / 10 / 13 cm/s and lengths 25 / 40 cm (all 8 pairs every 8 drives of a robot): the line check
               (p95) and the miss pattern (are the > 3 cm spots single pictures or runs of pictures in a row), and
               the heading the camera reads from the last straight (25 vs 40 cm line);
            c. every distance-sensor reading the robot sent during the drive (raw.csv): how many readings under
               LOW_CM, in runs of how many pictures in a row, and for each stop whether a rule of 2 or 3 readings in a
               row would have avoided it (run_plan's own stop rule is not changed);
            d. tracking time per picture per camera (tracking_times.csv, the bridge's summary), battery per robot
               (battery.csv), the Pi's temperature and throttling (cpu.csv + perf.csv: the heat tracker reads them).
  safety  every planned spot and line 25 cm from every edge and 30 cm from every parked robot's middle (the planner's
          floor is shrunk 13.5 cm and parked robots are 17 cm circles, so every turn stays 24.7 cm in from an edge
          and 28 cm from a parked robot; FLOOR_INSET_CM, PARK_R_CM say the rest); a FAIL or an unknown state
          parks that robot and the loop goes on with the others; Ctrl+C (or `./run-fleet-pi.sh doe-stop`, or the tmux
          session killed) stops every robot. Disconnecting SSH does NOT stop it (it runs in tmux: that is the point).
  record  the pictures at the tracking rate while a robot moves (Motion-JPEG .avi parts, both cameras) and a full-rate
          clip 5 s either side of every stop, hold or FAIL (e02_framesaver), instead of the full video (--full-video
          keeps the recorder's video too).

    python3 tools/fleet/doe_loop.py run [--hours 4] [--dry ...]
    python3 tools/fleet/doe_loop.py status [--dir logs/<...>-doe]
Everything goes to logs/<date-time>-doe/ (drives.csv, turns.csv, sensor_runs.csv, battery.csv, events.csv,
messages.csv, cpu.csv, perf.csv, one folder per loop). run-fleet-pi.sh doe starts it in tmux.
Frames (2026-10-28, after the 21:34 loop filled the card): on the USB drive, <drive>/fleet-frames/<date-time>-doe/frames
(frames_where.txt in the session folder says where; no drive: logs/<...>-doe/frames). Free space (diskguard.py): checked
before the start and between loops, and the frames' every 2 s; under the frames floor frame saving stops, under the
card's floor the loop ends cleanly (robots stopped, logs closed, the upload starts).
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import signal
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ladder as LD   # noqa: E402
from ladder import say, LadderStop, edge_dist, segs_clear   # noqa: E402
import diskguard as DG   # noqa: E402

import numpy as np    # noqa: E402

# ---------------------------------------------------------------------------------------------- chosen numbers
HOURS = 4.0             # Nick's figure: the time cap. No DOE: a limit
SPEEDS = (5.0, 7.5, 10.0, 13.0)    # Nick's figures (cm/s). The DOE itself: speed against p95, misses, heading
LENGTHS = (25.0, 40.0)  # Nick's figures (cm): heading from a short vs a long last straight. The DOE itself
TURN_EVERY = 4          # chosen: the turn check every 4th drive of a robot (and its first): about 1 turn check per
                        # speed cycle, ~25 % of the robot's time. DOE note: 2 / 4 / 8 against how fast the turn error
                        # drifts over a battery (turns.csv per robot over time); docs/doe-backlog.md
CLEAR_CM = 25.0         # Nick (2026-10-27): every spot and line 25 cm from every edge and parked robot. No DOE: safety
FLOOR_INSET_CM = 13.5   # the planner's floor shrunk this much on every side. Turning on the spot needs the middle
                        # 11.2 cm inside the floor (planner: 8.2 cm clear + 3), so a turn stays 24.7 cm in, and a
                        # robot L0 accepted at 25 cm can still turn where it stands (dry run 2 with 20: Right at 31 cm
                        # had 7.8 cm of 8.2: no plan). Planned spots and lines: CLEAR_CM. A detour's straight travel
                        # could come to 18.25 cm (13.5 + 4.75). No DOE: follows from CLEAR_CM and the planner
PARK_R_CM = 17.0        # a parked robot on a DOE map: a 17 cm circle: a turn next to it keeps the middles 28.2 cm
                        # apart (17 + 11.2), straight travel 21.75 (17 + 4.75). Shrunk for a robot that starts nearer
                        # than that (else no plan exists). No DOE: follows from CLEAR_CM and the planner's turn room
PARKED_CLEAR_CM = 30.0  # the drive's line and both its ends this far from a parked robot's middle: its 17 cm circle
                        # plus the planner's room to turn on the spot at A and B (11.2 cm; dry run 1: B 28 cm from a
                        # 20 cm circle had 8.1 cm of 8.2 and no plan). No DOE: follows from PARK_R_CM and the planner
MISS_CM = 3.0           # Nick's question: misses over 3 cm (the Phase 0 gate). No DOE: his threshold
SAFETY_P95_CM = 8.0     # chosen: a drive whose p95 is over 8 cm parks the robot (it is not following its line: a
                        # gate miss at 13 cm/s is data, a robot 8 cm off is a fault). No DOE: a safety limit
LOW_CM = 13.0           # chosen: a distance-sensor zone reading under this is "low" (today's five phantom stops
                        # tripped at 11.0-11.9 cm). DOE note: re-run the counts at 9 / 11 / 13 / 15 from raw.csv
                        # (logs only); docs/doe-backlog.md
CLIP_S = 5.0            # Nick's figure: full-rate clip 5 s either side of a stop / hold / FAIL. DOE note: from the
                        # first night's clips, how far back each stop's cause is visible; docs/doe-backlog.md
PART_S = 300.0          # chosen: tracking-rate frames are cut into 5 min .avi parts (one bad file loses little).
                        # No DOE: a file size
SIDE_HZ = 9.0           # the side camera's tracked rate measured today (8-9 a second); overhead: LD.Eye.OVERHEAD_HZ
STATUS_WAIT_S = 4.0     # chosen: how long the check waits for every answer (round trips 17-20 ms today). No DOE
PARAM_TOL = 0.0006      # a read-back value "matches" within this (the firmware prints 3 decimals). No DOE
PARAMS = ("gyro_scale", "wheel_scale", "gyro_drift_dps")
FACTORY = dict(gyro_scale=1.0, wheel_scale=1.0, gyro_drift_dps=0.0)


# ---------------------------------------------------------------------------------------------- frame saving
class CamFrames(threading.Thread):
    """One camera's /live stream as JPEG bytes (nothing decoded): every picture into a FrameSaver ring (full-rate
    clips around events), and while `active`, one picture every 1/rate s into .avi parts (the tracking rate)."""

    def __init__(self, cam, stream, folder, rate_hz, perf=None):
        super().__init__(daemon=True, name="frames-" + cam)
        import e02_framesaver as FS
        import sources as S
        self.FS = FS
        self.cam, self.rate = cam, float(rate_hz)
        self.dir = os.path.join(folder, cam)
        os.makedirs(os.path.join(self.dir, "track"), exist_ok=True)
        self.saver = FS.FrameSaver(self.dir, before_s=CLIP_S, after_s=CLIP_S, thumb_every_s=60.0, log=say)
        self.src = S.StreamSource(stream, log=lambda s: None)
        self.active, self.label = False, "idle"
        self.part, self.part_t0, self.last_kept = [], None, None
        self.lock = threading.Lock()
        self.closed = False
        self.n_in, self.n_kept, self.mb = 0, 0, 0.0
        self.perf = perf
        self.arrivals = []        # (t_arrived, age_s), for perf.csv
        self.saving = True        # False: frame saving stopped for good (free space under the floor)

    def set_active(self, on, label=""):
        with self.lock:
            if not self.saving:
                return
            if not on and self.active:
                self._flush()
            if on and (not self.active or label != self.label):
                self._flush()
                self.label = label or "move"
            self.active = on

    def _flush(self):
        """Writes the current part (lock held)."""
        if not self.part:
            self.part, self.part_t0 = [], None
            return
        stamp = time.strftime("%H%M%S", time.localtime(self.part[0][0]))
        slug = "".join(ch if ch.isalnum() else "-" for ch in self.label)[:50]
        base = os.path.join(self.dir, "track", "%s_%s" % (stamp, slug))
        size = self.FS.jpeg_size(self.part[0][1]) or (640, 480)
        try:
            self.FS.write_mjpeg_avi(base + ".avi", [q[1] for q in self.part], self.rate, size[0], size[1])
            with open(base + ".csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["i", "t", "pi_frame"])
                for i, q in enumerate(self.part):
                    w.writerow([i, "%.3f" % q[0], q[2]])
            self.mb += sum(len(q[1]) for q in self.part) / 1e6
        except OSError as e:
            say("  (frame saving, %s: %s)" % (self.cam, e))
        self.part, self.part_t0 = [], None

    def run(self):
        k = 0
        while not self.closed:
            try:
                self.src._connect()
                for h, body, arrived in self.src.parts():
                    if self.closed:
                        break
                    try:
                        t = float(h.get("x-time", arrived))
                    except ValueError:
                        t = arrived
                    pf = h.get("x-frame", k)
                    k += 1
                    self.n_in += 1
                    if not self.saving:
                        continue
                    self.saver.add(body, t, pf)
                    self.arrivals.append((arrived, max(0.0, arrived - t)))
                    with self.lock:
                        if self.active and (self.last_kept is None or t - self.last_kept >= 0.8 / self.rate):
                            self.last_kept = t
                            self.part.append((t, body, pf))
                            self.n_kept += 1
                            if self.part_t0 is None:
                                self.part_t0 = t
                            elif t - self.part_t0 >= PART_S:
                                self._flush()
            except (OSError, ConnectionError, ValueError):
                if self.closed:
                    break
                time.sleep(1.0)

    def stop_saving(self):
        """No more pictures kept (the part so far is written while there is still room)."""
        with self.lock:
            self._flush()
            self.active = False
            self.saving = False

    def saved_bytes(self):
        return self.mb * 1e6 + float(self.saver.summary().get("mb") or 0.0) * 1e6

    def perf_row(self, now):
        """(pictures a second over the last 2 s, mean age, max age) of this camera's arrivals."""
        a = [q for q in self.arrivals if q[0] >= now - 2.0]
        self.arrivals = a
        if not a:
            return 0.0, None, None
        ages = [q[1] for q in a]
        return len(a) / 2.0, float(np.mean(ages)), float(max(ages))

    def close(self):
        self.closed = True
        with self.lock:
            self._flush()
            self.active = False
        try:
            self.src.close()
        except Exception:
            pass
        try:
            self.saver.close()
        except OSError as e:
            say("  (frame saving, %s: %s)" % (self.cam, e))


class Frames:
    """Both cameras' frame saving, perf.csv every 2 s (the overhead's pictures a second and their age: the heat
    tracker joins it to cpu.csv; since 2026-10-28 also the free space on the card and the frames' drive, GB).
    frames_dir: where the pictures go (diskguard.frames_dir_for: the USB drive); default <folder>/frames.
    guard (diskguard.Guard): checked with every perf row; under the frames floor, frame saving stops for good."""

    def __init__(self, folder, streams, frames_dir=None, guard=None):
        self.dir = frames_dir or os.path.join(folder, "frames")
        os.makedirs(self.dir, exist_ok=True)
        self.guard = guard
        self.stopped = None           # (clock, why) once frame saving was stopped for lack of room
        self.cams = []
        for cam, stream, hz in streams:
            if stream:
                c = CamFrames(cam, stream, self.dir, hz)
                c.start()
                self.cams.append(c)
        self.closed = False
        self.perf = open(os.path.join(folder, "perf.csv"), "a", buffering=1)
        if self.perf.tell() == 0:
            self.perf.write("t,fps_handled,age_mean_s,age_max_s,side_fps,card_free_gb,frames_free_gb\n")
        threading.Thread(target=self._perf, daemon=True).start()

    def _perf(self):
        while not self.closed:
            time.sleep(2.0)
            now = time.time()
            rows = {c.cam: c.perf_row(now) for c in self.cams}
            o = rows.get("overhead", (0.0, None, None))
            s = rows.get("side", (0.0, None, None))
            self.check_space()
            cg, fg = DG.perf_cols(self.guard)
            try:
                self.perf.write("%.2f,%.1f,%s,%s,%.1f,%s,%s\n" % (
                    now, o[0], "" if o[1] is None else "%.3f" % o[1], "" if o[2] is None else "%.3f" % o[2], s[0],
                    cg, fg))
            except ValueError:
                return
            except OSError:
                pass

    def saved_bytes(self):
        return sum(c.saved_bytes() for c in self.cams)

    def check_space(self):
        """Frame saving stops for good when the frames' drive (or the card) is under its floor. Returns the state."""
        if self.guard is None:
            return None
        st = self.guard.state(self.saved_bytes())
        if not st["frames_ok"] and self.stopped is None:
            self.stop_saving(st.get("why_frames", "no room"))
        return st

    def stop_saving(self, why):
        if self.stopped is not None:
            return
        self.stopped = (time.strftime("%H:%M:%S"), why)
        for c in self.cams:
            c.stop_saving()
        say("  FRAME SAVING STOPPED: %s. The loop goes on; its logs are small." % why)

    def active(self, on, label=""):
        for c in self.cams:
            c.set_active(on, label)

    def event(self, kind, text, t=None, robot=""):
        if self.stopped is not None:
            return
        for c in self.cams:
            c.saver.event(kind, text, t=t, robot=robot)

    def close(self):
        self.closed = True
        for c in self.cams:
            c.close()
        try:
            self.perf.close()
        except OSError:
            pass

    def words(self):
        w = "; ".join("%s: %d pictures in, %d kept at the tracking rate (%.0f MB), clips %s" % (
            c.cam, c.n_in, c.n_kept, c.mb, c.saver.summary()) for c in self.cams)
        w += " (in %s)" % self.dir
        if self.stopped:
            w += "; frame saving stopped at %s: %s" % self.stopped
        return w


class Tail(threading.Thread):
    """Follows a run_plan session's coordinator.jsonl while it runs: every stop, hold or FAIL becomes a clip."""

    def __init__(self, path, frames):
        super().__init__(daemon=True)
        self.path, self.frames = path, frames
        self.stopping = False

    def run(self):
        f, pos = None, 0
        while not self.stopping:
            if f is None:
                if os.path.exists(self.path):
                    f = open(self.path)
                else:
                    time.sleep(0.3)
                    continue
            ln = f.readline()
            if not ln or not ln.endswith("\n"):
                if ln:
                    f.seek(pos)
                time.sleep(0.2)
                continue
            pos = f.tell()
            try:
                e = json.loads(ln)
            except ValueError:
                continue
            kind = clip_kind(e)
            if kind:
                self.frames.event(kind, e.get("info") or e.get("text") or e.get("state") or "", t=e.get("t"),
                                  robot=e.get("robot", ""))
        if f is not None:
            f.close()

    def stop(self):
        self.stopping = True


def clip_kind(e):
    """The frame saver's kind for a coordinator event worth a clip, or None."""
    ev, st, info = e.get("event"), e.get("state"), str(e.get("info") or "")
    if ev == "step_report" and st == "failed":
        if info.startswith("obstacle"):
            return "near-robot stop"          # e02_framesaver.KINDS: a distance-sensor stop
        if "edge" in info:
            return "edge stop"
        return "other"
    if ev == "step_report" and st in ("held", "halted"):
        return "hold"
    return None


# ---------------------------------------------------------------------------------------------- analyses
def miss_pattern(sess, bridge, robot, gate=MISS_CM):
    """Each camera spot's distance to the planned lines, in time order: how many over gate, and in runs of how many
    pictures in a row (Nick: single frames or runs?)."""
    import line_check as LC
    lines = LC.load_lines(sess).get(robot) or []
    t0, t1 = LC.run_window(sess)
    tr = LC.read_track(os.path.join(bridge, "track.csv"), t0, t1).get(robot, [])
    if not lines or not tr:
        return dict(n=0)
    d = [min(LC.seg_dist((x, y), a, b) for a, b in lines) for _, x, y in sorted(tr)]
    runs, k = [], 0
    for v in d + [0.0]:
        if v > gate:
            k += 1
        elif k:
            runs.append(k)
            k = 0
    return dict(n=len(d), over=sum(runs), runs=len(runs), single=sum(1 for r in runs if r == 1),
                longest=max(runs) if runs else 0, in_runs_of_3_plus=sum(r for r in runs if r >= 3))


def sensor_runs(sess, robot, low=LOW_CM):
    """The robot's distance-sensor readings during the run (raw.csv; a new reading = the five zones changed): every
    run of low readings in a row (any zone under low), and each distance-sensor stop (step failed "obstacle") with
    the run it ended. Returns (summary dict, [run rows])."""
    rp = os.path.join(sess, "raw.csv")
    zones = ("dL_cm", "dCL_cm", "dC_cm", "dCR_cm", "dR_cm")
    reads = []
    if os.path.exists(rp):
        with open(rp, newline="") as f:
            for r in csv.DictReader(f):
                if r.get("robot") != robot:
                    continue
                try:
                    z = tuple(float(r[c]) for c in zones)
                    t = float(r["t_recv"])
                except (KeyError, ValueError, TypeError):
                    continue
                if reads and reads[-1][1] == z:
                    continue                    # the same sensor picture repeated in the next RAW row
                reads.append((t, z))
    stops = []
    cp = os.path.join(sess, "coordinator.jsonl")
    if os.path.exists(cp):
        with open(cp) as f:
            for ln in f:
                try:
                    e = json.loads(ln)
                except ValueError:
                    continue
                if e.get("event") == "step_report" and e.get("robot") == robot and e.get("state") == "failed" \
                        and str(e.get("info", "")).startswith("obstacle"):
                    stops.append((e["t"], e.get("info")))
    runs, cur = [], None
    for t, z in reads:
        lo = min((v for v in z if v > 0), default=999.0) < low
        if lo:
            if cur is None:
                cur = dict(t0=t, t1=t, n=0, min_cm=999.0, zones=set())
            cur["t1"] = t
            cur["n"] += 1
            cur["min_cm"] = min(cur["min_cm"], min(v for v in z if v > 0))
            cur["zones"].update(zn for zn, v in zip("L CL C CR R".split(), z) if 0 < v < low)
        elif cur is not None:
            runs.append(cur)
            cur = None
    if cur is not None:
        runs.append(cur)
    stop_rows = []
    for ts, info in stops:
        # the run of low readings that ended at (or within 0.3 s before) the stop
        # (t_recv: the laptop's receive times on both, so they compare; the nearest end wins)
        near = [q for q in runs if q["t0"] - 0.3 <= ts and q["t1"] >= ts - 0.5]
        r = min(near, key=lambda q: abs(ts - q["t1"])) if near else None
        n = r["n"] if r else 0
        stop_rows.append(dict(t=ts, info=info, run=n, avoided_2=n < 2, avoided_3=n < 3))
    summ = dict(readings=len(reads), low_readings=sum(q["n"] for q in runs), low_runs=len(runs),
                single=sum(1 for q in runs if q["n"] == 1), runs_2=sum(1 for q in runs if q["n"] == 2),
                runs_3_plus=sum(1 for q in runs if q["n"] >= 3), stops=len(stops),
                stops_avoided_by_2=sum(1 for s in stop_rows if s["avoided_2"]),
                stops_avoided_by_3=sum(1 for s in stop_rows if s["avoided_3"]), stop_detail=stop_rows)
    rows = [dict(t0=round(q["t0"], 3), t1=round(q["t1"], 3), readings=q["n"], min_cm=round(q["min_cm"], 1),
                 zones=" ".join(sorted(q["zones"]))) for q in runs]
    return summ, rows


def pick_segment(rect, p, length, parked, clear=CLEAR_CM, parked_clear=PARKED_CLEAR_CM):
    """A straight A->B of the given length, both ends and the line CLEAR_CM from every edge and every parked robot,
    A as near the robot as possible (a little preference for the board's middle). (A, B) or None."""
    best = None
    mid = ((rect[0] + rect[2]) / 2.0, (rect[1] + rect[3]) / 2.0)
    for ang in range(0, 180, 15):
        ux, uy = math.cos(math.radians(ang)), math.sin(math.radians(ang))
        for cx in np.arange(rect[0] + clear, rect[2] - clear + 0.01, 3.0):
            for cy in np.arange(rect[1] + clear, rect[3] - clear + 0.01, 3.0):
                a = (cx - ux * length / 2, cy - uy * length / 2)
                b = (cx + ux * length / 2, cy + uy * length / 2)
                if min(edge_dist(rect, *a), edge_dist(rect, *b)) < clear:
                    continue
                if parked and segs_clear([(a, b)], parked, parked_clear) < parked_clear:
                    continue
                for A, B in ((a, b), (b, a)):
                    s = math.dist(p, A) + 0.15 * math.dist((cx, cy), mid)
                    if best is None or s < best[0]:
                        best = (s, A, B)
    return None if best is None else (best[1], best[2])


# ---------------------------------------------------------------------------------------------- the loop
class DoeLoop(LD.Ladder):
    def __init__(self, a):
        if not a.dir:
            a.dir = os.path.join(LD.REPO, "logs", time.strftime("%Y%m%d-%H%M%S") + "-doe" + ("-dryrun" if a.dry else ""))
        super().__init__(a)
        self.st.setdefault("doe", {"i": {}, "boot": {}, "uptime": {}, "loops": 0})
        self.save()
        self.got = {}
        self.frames = None
        self.cpu = None
        self.files = {}
        self.sim_events = parse_sim_events(getattr(a, "sim_event", []) or [])
        self.t_start = time.time()
        self.guard = None
        self.frames_dir = None
        self.sim_fall = 0.0
        self.ended = None             # "disk": the loop ended because the card was nearly full
        self.row_err = False

    # -- csv files ----------------------------------------------------------------------------------------------
    def row(self, name, d):
        f = self.files.get(name)
        if f is None:
            p = os.path.join(self.dir, name + ".csv")
            new = not os.path.exists(p)
            try:
                fh = open(p, "a", newline="", buffering=1)
            except OSError as e:
                say("  (could not open %s.csv: %s)" % (name, e))
                return
            w = csv.DictWriter(fh, fieldnames=list(d.keys()), extrasaction="ignore")
            if new:
                w.writeheader()
            f = self.files[name] = (fh, w)
        try:
            f[1].writerow(d)
            f[0].flush()
        except OSError as e:                 # (a full disk: the guard ends the loop before this; never a crash)
            if not self.row_err:
                say("  (could not write %s.csv: %s)" % (name, e))
            self.row_err = True

    def event(self, kind, robot, text, clip=True):
        say("  [%s] %s: %s" % (kind, robot, text))
        self.row("events", dict(t=round(time.time(), 2), clock=time.strftime("%H:%M:%S"), kind=kind, robot=robot,
                                text=text))
        if clip and self.frames is not None:
            self.frames.event("other", "%s: %s" % (kind, text), robot=robot)

    def park(self, n, why, safety=False):
        """Out of the loop for good: it stays where it is (stopped), on every later map as an obstacle.
        --keep-going (Nick, 2026-10-28 18:00, "without any stops, just to see what happens"): only a SAFETY park
        (battery floor, not answering, a restart that could not be recalibrated, settings that will not read back,
        both cameras lost it, the firmware's own edge / border stop) parks; any other (turn check, heading crawl,
        line check, one camera's evidence) is logged as "would have parked" and the robot carries on."""
        if getattr(self.a, "keep_going", False) and not safety:
            self.st["doe"].setdefault("would_park", []).append(dict(t=round(time.time(), 1), robot=n, why=why))
            self.save()
            self.event("would have parked", n, why)
            return
        if n in self.st["kept_out"]:
            return
        try:
            if self.bench is not None:
                self.bench.send(n, "stop")
        except Exception:
            pass
        self.keep_out(n, "DOE loop: " + why)
        self.event("parked", n, why)

    # -- the check before every loop and move -------------------------------------------------------------------
    def expect(self, n):
        return dict(FACTORY) if self.a.dry else {k: LD.corrections_for(n)[k] for k in PARAMS}

    def _listen(self, name, kind, rtype, vals):
        g = self.got.setdefault(name, {})
        if kind == "D" and rtype == "CLOCK" and len(vals) >= 3:
            try:
                g.update(boot=vals[2].strip(), uptime_s=int(vals[1]) / 1000.0, t_clock=time.time())
            except ValueError:
                pass
        elif kind == "M":
            text = vals if isinstance(vals, str) else ",".join(vals)
            self.row("messages", dict(t=round(time.time(), 3), robot=name, text=text[:300]))
            m = re.match(r"\s*(\w+)\s*=\s*([-+0-9.eE]+)", text)
            if m and m.group(1) in PARAMS:
                g.setdefault("params", {})[m.group(1)] = float(m.group(2))

    def bench_for(self, sub):
        bench = self.need_bench(sub)
        if not getattr(bench, "_doe", False):
            bench.link.session.listeners.append(self._listen)
            bench._doe = True
        return bench

    def ask(self, bench, names):
        for n in names:
            bench.fw.pop(n, None)
            g = self.got.setdefault(n, {})
            g.pop("params", None)
            g.pop("boot", None)
            bench.send(n, "status")
            bench.send(n, "clock doe%d" % int(time.time() * 10 % 100000))
            for p in PARAMS:
                bench.send(n, "get " + p)
                time.sleep(0.02)

        def done():
            return all(n in bench.fw and self.got.get(n, {}).get("boot") and
                       len(self.got.get(n, {}).get("params", {})) == len(PARAMS) for n in names)
        bench.wait(done, STATUS_WAIT_S)

    def check(self, sub, names, why):
        """Every robot in names: answering, battery, restart, corrections. Parks the ones that cannot go on, recalibrates
        the ones that restarted. Returns the names still fine."""
        names = [n for n in names if n not in self.st["kept_out"]]
        if not names:
            return []
        bench = self.bench_for(sub)
        self.ask(bench, names)
        silent = [n for n in names if n not in bench.fw]
        if silent:                                       # asleep (a check-in missed) or a lost answer: wake, ask again
            for n in silent:
                bench.send(n, "wake")
            time.sleep(1.0)
            self.ask(bench, silent)
        fine, recal = [], []
        d = self.st["doe"]
        for n in names:
            g = self.got.get(n, {})
            fw = bench.fw.get(n)
            if fw is None:
                self.park(n, "not answering (asleep, off or out of WiFi) at the check before %s" % why, safety=True)
                continue
            boot, up = g.get("boot"), g.get("uptime_s")
            self.row("battery", dict(t=round(time.time(), 1), clock=time.strftime("%H:%M:%S"), robot=n,
                                     battery_pct=fw[1], boot=boot or "", uptime_s="" if up is None else round(up, 1),
                                     before=why))
            old_boot, old_up = d["boot"].get(n), d["uptime"].get(n)
            restarted = bool(old_boot and boot and boot != old_boot) or \
                (old_up is not None and up is not None and up + 5.0 < old_up)
            if boot:
                d["boot"][n] = boot
            if up is not None:
                d["uptime"][n] = up
            self.save()
            if fw[1] < LD.MIN_BATTERY:
                self.park(n, "battery %d%% (under %d%%)" % (fw[1], LD.MIN_BATTERY), safety=True)
                continue
            if restarted:
                self.event("restarted", n, "boot id %s -> %s (uptime %.0f s): wake, edge guard and corrections again, "
                           "then calibrate again" % (old_boot, boot, up or -1))
                bench.send(n, "wake")
                for c in LD.SAFE + self.correction_cmds(n):
                    bench.send(n, c)
                    time.sleep(0.03)
                recal.append(n)
                time.sleep(0.3)
                self.ask(bench, [n])
                g = self.got.get(n, {})
            ok = self.params_ok(bench, n, g)
            if not ok:
                continue
            fine.append(n)
        for n in recal:
            self.st["passed"].pop("L1:%s" % n, None)
            self.st["robots"].get(n, {}).pop("h", None)
            self.save()
            say("  %s restarted: calibrating it again (alone) ..." % n)
            self.L1(sub, [n])        # (no tracking-rate frames: a calibration is not tracked; clips still work)
            if not self.st["passed"].get("L1:%s" % n):
                self.park(n, "restarted, and calibrating it again failed", safety=True)
                fine = [m for m in fine if m != n]
            else:
                self.st["doe"].setdefault("turn_due", {})[n] = True     # a fresh calibration: turn check it first
                self.save()
        return fine

    def correction_cmds(self, n):
        e = self.expect(n)
        return ["set gyro_scale %.4f" % e["gyro_scale"], "set gyro_drift_dps %.4f" % e["gyro_drift_dps"],
                "set wheel_scale %.4f" % e["wheel_scale"]]

    def params_ok(self, bench, n, g):
        exp = self.expect(n)

        alt = {k: LD.corrections_for(n)[k] for k in PARAMS} if self.a.dry else exp

        def bad(g):
            # dry runs: the simulated robots keep factory values, but a run_plan that sends the stored corrections
            # leaves those on them: either is fine (dry run 1: a resend back and forth every loop)
            got = g.get("params", {})
            return [p for p in PARAMS if p not in got or (abs(got[p] - exp[p]) > PARAM_TOL and
                                                          abs(got[p] - alt[p]) > PARAM_TOL)]
        b = bad(g)
        if b:
            self.event("setting resent", n, "read back %s (expected %s): sent again" % (
                ", ".join("%s=%s" % (p, g.get("params", {}).get(p, "?")) for p in b),
                ", ".join("%s=%g" % (p, exp[p]) for p in b)), clip=False)
            for p in b:
                bench.send(n, "set %s %.4f" % (p, exp[p]))
                time.sleep(0.05)
            time.sleep(0.3)
            self.ask(bench, [n])
            b = bad(self.got.get(n, {}))
            if b:
                self.park(n, "its %s does not read back as sent, twice" % ", ".join(b), safety=True)
                return False
        return True

    # -- frames -------------------------------------------------------------------------------------------------
    def frames_on(self, label):
        if self.frames is not None:
            self.frames.active(True, label)

    def frames_off(self):
        if self.frames is not None:
            self.frames.active(False)

    # -- the dry run's scripted events ---------------------------------------------------------------------------
    def sim_hooks(self, k):
        """--dry --sim-event restart:Center@2 / battery:Left=24@3: sent to the simulated robot before loop k."""
        if not self.a.dry:
            return
        for kind, n, val, at in self.sim_events:
            if at == k and self.bench is not None:
                say("  (dry run: simulated %s for %s%s)" % (kind, n, "" if val is None else " = %s" % val))
                self.bench.send(n, "simctl restart" if kind == "restart" else "simctl battery %s" % val)
                time.sleep(0.5)

    def keep_going_back(self, names):
        """--keep-going: a robot find_all kept out because neither camera saw its BODY, but whose lights it found, is
        not set aside: "would have parked" is logged and it stays in (cam_bridge follows its lights; a robot lost by
        both cameras and its lights is still parked, as a robot found nowhere is)."""
        for m in names:
            why = self.st["kept_out"].get(m, "")
            if why.startswith("no camera can see its body"):
                self.st["kept_out"].pop(m, None)
                self.save()
                self.park(m, why + " (keep-going: it stays in)")

    # -- one turn check -----------------------------------------------------------------------------------------
    def turn(self, sub, n, k):
        self.frames_on("turncheck-%s" % n)
        try:
            ok, res = self.turncheck(n, sub)
        except LadderStop as e:
            self.frames_off()
            self.park(n, "turn check could not finish: %s" % e, safety=lost_words(str(e)))
            return False
        self.frames_off()
        for r in res["turns"]:
            self.row("turns", dict(t=round(time.time(), 1), loop=k, robot=n, turn=r["turn"], camera_deg=r["camera_deg"],
                                   own_deg=r["own_deg"], error_deg=r["error_deg"], total_error_deg=res["total_error_deg"],
                                   pass_=ok, battery_pct=(self.bench.fw.get(n) or ("", ""))[1] if self.bench else ""))
        if not ok:
            self.park(n, "turn check FAIL (errors %s, total %+.1f deg)" % (
                ", ".join("%+.1f" % r["error_deg"] for r in res["turns"]), res["total_error_deg"]))
        return ok

    # -- one drive ----------------------------------------------------------------------------------------------
    def drive(self, sub, n, i, k):
        speed, length = SPEEDS[i % len(SPEEDS)], LENGTHS[(i // len(SPEEDS)) % len(LENGTHS)]
        p, h = self.pos(n), self.heading(n)
        if h is None:
            say("  %s: heading not known: heading crawl first" % n)
            self.need_bench(sub)
            self.need_eye()
            self.frames_on("crawl-" + n)
            try:
                h, _ = self.crawl(n, self.eye.still(n) or p, None)
            except LadderStop as e:
                self.frames_off()
                self.park(n, "heading crawl: %s" % e, safety=lost_words(str(e)))
                h = getattr(e, "heading", None)
                if not getattr(self.a, "keep_going", False) or lost_words(str(e)):
                    return None
                if h is None:
                    # keep-going, no heading at all from the crawl: no drive this time (it tries again next turn)
                    self.event("no heading", n, "no camera heading from the crawl: drive %d not tried this loop" %
                               (i + 1), clip=False)
                    return "no heading"
                say("  %s: keep-going: the crawl's camera heading %.1f deg is used anyway" % (n, h))
            self.frames_off()
            q = self.eye.still(n) or p
            self.set_pose(n, q[0], q[1], h, "crawl")
            p = q
        others = self.others_of(n)
        seg = pick_segment(self.rect, p, length, list(others.values()))
        if seg is None:
            self.event("skipped", n, "no straight %.0f cm line fits %.0f cm from every edge and parked robot" % (
                length, CLEAR_CM), clip=False)
            return "skipped"
        A, B = seg
        A, B = (round(A[0], 1), round(A[1], 1)), (round(B[0], 1), round(B[1], 1))   # spots and park the same point
        m = LD.base_map(self.rect, "DOE loop %d: %s out and back %.0f cm at %.1f cm/s" % (k, n, length, speed),
                        others, speed)
        f = m["floor"]
        f.update(x_min=f["x_min"] + FLOOR_INSET_CM, y_min=f["y_min"] + FLOOR_INSET_CM,
                 x_max=f["x_max"] - FLOOR_INSET_CM, y_max=f["y_max"] - FLOOR_INSET_CM)
        for ob in m["obstacles"]:
            dc = math.dist(p, ob["center"])
            ob["radius"] = PARK_R_CM if dc - 11.5 >= PARK_R_CM else max(LD.OBSTACLE_R_CM, min(PARK_R_CM, dc - 11.5))
        m["spots"] = {"A": list(A), "B": list(B)}
        # it parks on A facing the way it came back (B->A): no move after the last straight, whose camera line gives
        # the heading (dry run 1: parked back on its start spot, the last straight was 0.9 cm and no heading came)
        h_back = math.degrees(math.atan2(A[1] - B[1], A[0] - B[0]))
        m["robots"] = [{"name": n, "start": [round(p[0], 2), round(p[1], 2), round(h, 2)],
                        "park": [A[0], A[1], round(h_back, 2)]}]
        m["tasks"] = {n: {"visit": [{"spot": "A"}, {"spot": "B"}, {"spot": "A"}], "park": True}}
        say("  %s: drive %d (%.1f cm/s, %.0f cm): to A (%.0f, %.0f), to B (%.0f, %.0f) and back" % (
            n, i + 1, speed, length, A[0], A[1], B[0], B[1]))
        tag = "drive%02d-%s-%gcms-%gcm" % (i + 1, n, speed, length)
        tail = Tail(os.path.join(sub, tag, "coordinator.jsonl"), self.frames) if self.frames else None
        if tail:
            tail.start()
        self.frames_on(tag)
        try:
            r = self.run_planned(tag, m, {n: (p[0], p[1], h)}, sub)
        finally:
            self.frames_off()
            if tail:
                tail.stop()
        res = r["summary"].get("result")
        c = ((r["check"] or {}).get("robots") or {}).get(n, {}).get("camera", {})
        mp = miss_pattern(r["sess"], os.path.join(r["sess"], "bridge"), n) if r["check"] else dict(n=0)
        ss, runs = sensor_runs(r["sess"], n)
        for q in runs:
            self.row("sensor_runs", dict(loop=k, robot=n, drive=tag, **q))
        for q in ss["stop_detail"]:
            self.row("sensor_stops", dict(loop=k, robot=n, drive=tag, t=round(q["t"], 3), info=q["info"],
                                          low_in_a_row=q["run"], avoided_by_2=q["avoided_2"],
                                          avoided_by_3=q["avoided_3"]))
        e = None
        if r["check"] is not None:
            try:
                e = self.end_poses(r["sess"], [n])[n]
            except (OSError, KeyError, ValueError):
                e = None
        if e:
            self.set_pose(n, e["x"], e["y"], e["h"], e["how"])
        bs = {}
        try:
            bs = json.load(open(os.path.join(r["sess"], "bridge", "bridge_summary.json")))
        except (OSError, ValueError):
            pass
        rp = r["summary"].get("replans")
        replans = len(rp) if isinstance(rp, (list, tuple)) else (rp if rp is not None else "")
        row = dict(t=round(time.time(), 1), clock=time.strftime("%H:%M:%S"), loop=k, robot=n, drive=i + 1,
                   speed_cms=speed, length_cm=length, result=res, p95_cm=c.get("p95_cm"), median_cm=c.get("median_cm"),
                   max_cm=c.get("max_cm"), spots=mp.get("n"), over_3cm=mp.get("over"), miss_runs=mp.get("runs"),
                   miss_single=mp.get("single"), miss_longest=mp.get("longest"),
                   miss_in_runs_3plus=mp.get("in_runs_of_3_plus"),
                   heading_cam_deg=None if not e or e["h"] is None else round(e["h"], 2), heading_how=e["how"] if e else "",
                   sensor_readings=ss["readings"], sensor_low=ss["low_readings"], sensor_low_runs=ss["low_runs"],
                   sensor_single=ss["single"], sensor_stops=ss["stops"], stops_avoided_by_2=ss["stops_avoided_by_2"],
                   stops_avoided_by_3=ss["stops_avoided_by_3"], replans=replans,
                   bridge_fps=round(bs["frames"] / bs["duration_s"], 1) if bs.get("frames") and bs.get("duration_s")
                   else "", sess=os.path.relpath(r["sess"], self.dir))
        self.row("drives", row)
        say("  %s drive %d: %s; p95 %s cm, >3 cm %s spots in %s runs (%s single, longest %s); heading %s; "
            "distance sensor: %d low readings in %d runs (%d single), %d stops (a 2-in-a-row rule avoids %d, 3: %d)" % (
                n, i + 1, res, c.get("p95_cm"), mp.get("over"), mp.get("runs"), mp.get("single"), mp.get("longest"),
                row["heading_cam_deg"], ss["low_readings"], ss["low_runs"], ss["single"], ss["stops"],
                ss["stops_avoided_by_2"], ss["stops_avoided_by_3"]))
        if res is None and not os.path.exists(os.path.join(r["sess"], "step.csv")):
            # no plan was made (run_plan stopped before anything was sent: nothing moved): not the robot's fault
            self.event("no plan", n, "the planner found no plan for drive %d (nothing moved): skipped; look at %s" % (
                i + 1, os.path.relpath(r["sess"], LD.REPO)), clip=False)
            return "skipped"
        if res != "done":
            self.park(n, "drive %d did not finish (%s); look at %s" % (i + 1, res, os.path.relpath(r["sess"], LD.REPO)),
                      safety=lost_words(str(res)))
            return "fail"
        if c.get("p95_cm") is None or c["p95_cm"] > SAFETY_P95_CM or e is None:
            lost = e is None or c.get("p95_cm") is None
            self.park(n, "drive %d: %s" % (i + 1, "the camera lost it" if lost else
                                            "p95 %.1f cm, over the %.0f cm safety limit" % (c["p95_cm"], SAFETY_P95_CM)),
                      safety=lost)
            return "fail"
        return "done"

    # -- the whole run ------------------------------------------------------------------------------------------
    def start_logs(self):
        try:
            sys.path.insert(0, os.path.join(LD.TOOLS, "experiment"))
            import e02_cpulog as CL
            self.cpu = CL.CpuLog(self.dir, procs={"doe": os.getpid()},
                                 find={"recorder": "camrec.py", "bridge": "cam_bridge.py", "run_plan": "run_plan.py"})
            self.cpu.start()
        except Exception as e:
            say("  (cpu log not started: %s)" % e)
        streams = [("overhead", self.stream, LD.Eye.OVERHEAD_HZ)]
        if self.side_stream:
            streams.append(("side", self.side_stream, SIDE_HZ))
        fake = None
        if self.a.dry and getattr(self.a, "sim_free", ""):
            fake = dict((k, float(v)) for k, v in (kv.split("=") for kv in self.a.sim_free.split(",")))
            self.sim_fall = fake.pop("fall", 0.0)
        if not self.a.no_frames:
            self.frames_dir, _ = DG.frames_dir_for(self.dir)
        self.guard = DG.Guard(card=self.dir, frames_dir=self.frames_dir, fake=fake)
        say(self.guard.words(frames_on=not self.a.no_frames))
        st = self.guard.state()
        if not self.a.no_frames:
            if st["frames_ok"] and not st["stop"]:
                self.frames = Frames(self.dir, streams, frames_dir=self.frames_dir, guard=self.guard)
            else:
                say("  FRAMES NOT SAVED: %s." % (st.get("why_frames") or st["why"]))
                self.event("disk", "Pi", "frames not saved from the start: %s" % (st.get("why_frames") or st["why"]),
                           clip=False)

    def space_check(self, k):
        """Between loops: the frames' floor (frame saving stops) and the card's (the loop ends). True: end now."""
        if self.guard is None:
            return False
        if self.sim_fall:
            self.guard.fall(self.sim_fall)
            say("  (dry run: simulated free space now card %.1f GB, drive %.1f GB)" % (
                self.guard.fake.get("card", -1), self.guard.fake.get("drive", -1)))
        st = self.frames.check_space() if self.frames is not None else self.guard.state()
        if self.frames is not None and self.frames.stopped and not getattr(self, "_frames_said", False):
            self._frames_said = True
            self.event("disk", "Pi", "frame saving stopped before loop %d: %s" % (k, self.frames.stopped[1]), clip=False)
        if st["stop"]:
            self.ended = "disk"
            self.event("disk", "Pi", "the loop ends before loop %d: %s" % (k, st["why"]), clip=False)
            say("")
            say("=" * 100)
            say("THE PI'S CARD IS NEARLY FULL (%.2f GB free): the DOE loop ends here, cleanly, before it can fill." % (
                st["card_gb"]))
            say("Every robot is stopped where it is; the logs are closed next; the upload starts after that.")
            say("Before the next run, see what uses the space:  ./run-fleet-pi.sh space")
            say("=" * 100)
            return True
        return False

    def run(self, start=None, pause=False):
        say("DOE loop: %s (time cap %.1f h)" % (self.dir, self.a.hours))
        self.start_logs()
        st = self.guard.state()
        if st["stop"]:
            self.ended = "disk"
            say("\nNOT STARTED: %s. Nothing moved. Free space first (./run-fleet-pi.sh space, then "
                "./run-fleet-pi.sh clean-uploaded)." % st["why"])
            return False
        t_end = self.t_start + self.a.hours * 3600.0
        if not self.st["passed"].get("L0"):
            if not self.run_step("L0", in_order=True):
                say("\nL0 found no robot that can go on: the DOE loop stops.")
                return False
        names = self.active("L0")
        sub = self.sub("check0")
        self.check(sub, names, "L1")                # the first boot ids (the restart reference), battery, settings
        if not any(self.st["passed"].get("L1:%s" % n) for n in self.active()):
            self.run_step("L1", in_order=True)
        k = self.st["doe"]["loops"]
        idle_rounds = 0
        while time.time() < t_end:
            act = self.active("L1")
            if not act:
                say("\nEvery robot is out: the DOE loop ends.")
                break
            if self.space_check(k):
                break
            sub = self.sub("loop%03d" % k)
            say("")
            say("=" * 100)
            say("DOE loop %d  (%s; %.0f min left of the cap)" % (k, ", ".join(act), (t_end - time.time()) / 60.0))
            say("=" * 100)
            self.need_bench(sub)
            self.sim_hooks(k)
            act = self.check(sub, act, "loop %d" % k)
            if not act:
                k += 1
                self.st["doe"]["loops"] = k
                self.save()
                continue
            n = act[k % len(act)]
            i = self.st["doe"]["i"].get(n, 0)
            try:
                self.find_all(sub)
                if getattr(self.a, "keep_going", False):
                    self.keep_going_back(act)
                if n in self.st["kept_out"]:
                    raise LadderStop("the camera cannot see it now: lost by both cameras (and its lights)")
                if i % TURN_EVERY == 0 or self.st["doe"].get("turn_due", {}).pop(n, False):
                    if not self.check(sub, [n], "%s's turn check" % n):
                        raise LadderStop(None)
                    say("  %s: turn check (drive %d is next)" % (n, i + 1))
                    if not self.turn(sub, n, k) and n in self.st["kept_out"]:
                        raise LadderStop(None)
                if not self.check(sub, [n], "%s's drive" % n):
                    raise LadderStop(None)
                out = self.drive(sub, n, i, k)
                if out in ("done", "fail"):
                    self.st["doe"]["i"][n] = i + 1
                idle_rounds = idle_rounds + 1 if out == "skipped" else 0
            except LadderStop as e:
                if e.args and e.args[0]:
                    self.park(n, str(e), safety=lost_words(str(e)))
            k += 1
            self.st["doe"]["loops"] = k
            self.save()
            if idle_rounds >= 2 * max(1, len(act)):
                say("\nNo robot has room for a drive (every one skipped twice): the DOE loop ends.")
                break
        else:
            say("\nThe time cap (%.1f h) is reached: the DOE loop ends." % self.a.hours)
        return True

    def finish(self):
        """Every robot stopped and left where it is; the logs closed; a summary."""
        try:
            if self.bench is not None:
                for n in self.bench.robots():
                    self.bench.send(n, "stop")
        except Exception:
            pass
        if self.frames is not None:
            say("Frames: %s" % self.frames.words())
            self.frames.close()
            self.frames = None
        if self.guard is not None:
            st = self.guard.state()
            say("Free space at the end: the card %s GB%s." % (
                "?" if st["card_gb"] is None else "%.2f" % st["card_gb"],
                "" if st["frames_gb"] is None or st["frames_on_card"] else ", the frames' USB drive %.1f GB"
                % st["frames_gb"]))
        if self.cpu is not None:
            try:
                self.cpu.stop()
            except Exception:
                pass
        for fh, _ in self.files.values():
            try:
                fh.close()
            except OSError:
                pass
        self.files = {}
        write_summary(self.dir)


LOST_WORDS = ("lost", "edge", "border", "battery", "restart", "not answering")


def lost_words(text):
    """A failure that is about safety, not measurement (--keep-going still parks it): both cameras lost it, the
    firmware's own edge / border stop, battery, a restart, no answer."""
    t = (text or "").lower()
    return any(w in t for w in LOST_WORDS)


def stop_children(sig=signal.SIGINT):
    """Signal this process's own children (run_plan, cam_bridge) by parent pid, never by name: a Ctrl+C that reached
    only this process (doe-stop to a single pid, a SIGTERM) must still stop a plan that is driving."""
    me = os.getpid()
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open("/proc/%s/stat" % d) as f:
                ppid = int(f.read().rsplit(")", 1)[1].split()[1])
            if ppid == me:
                os.kill(int(d), sig)
        except (OSError, ValueError, IndexError):
            pass


def stop_all(L):
    """Every robot told to stop, whether or not the loop's own link is open (it is closed while run_plan drives)."""
    stop_children()
    time.sleep(1.5)                                  # run_plan's own Ctrl+C sends stop and closes its link
    try:
        if L.bench is None and L.ips:
            L.need_bench(L.dir)
        if L.bench is not None:
            for n in L.bench.robots():
                L.bench.send(n, "stop")
    except Exception as e:
        say("  (stop: %s)" % e)


def write_summary(folder):
    """summary.md: drives per speed and length (p95, misses, sensor), turn checks, why each robot left."""
    def rd(name):
        p = os.path.join(folder, name + ".csv")
        if not os.path.exists(p):
            return []
        with open(p, newline="") as f:
            return list(csv.DictReader(f))
    dr, tu, ev = rd("drives"), rd("turns"), rd("events")
    out = ["# DOE loop summary: %s" % os.path.basename(folder), ""]
    out.append("%d drives, %d turn-check turns, %d events." % (len(dr), len(tu), len(ev)))
    out.append("")
    out.append("| speed cm/s | length cm | drives | p95 median cm | spots > 3 cm | of them single | sensor low readings "
               "| sensor stops | avoided by 2 / 3 in a row |")
    out.append("|---|---|---|---|---|---|---|---|---|")

    def num(v):
        try:
            return float(v)
        except (TypeError, ValueError):
            return None
    for s in SPEEDS:
        for L in LENGTHS:
            g = [r for r in dr if num(r["speed_cms"]) == s and num(r["length_cm"]) == L]
            if not g:
                continue
            p95 = [num(r["p95_cm"]) for r in g if num(r["p95_cm"]) is not None]
            out.append("| %g | %g | %d | %s | %d | %d | %d | %d | %d / %d |" % (
                s, L, len(g), "%.2f" % float(np.median(p95)) if p95 else "-",
                sum(int(num(r["over_3cm"]) or 0) for r in g), sum(int(num(r["miss_single"]) or 0) for r in g),
                sum(int(num(r["sensor_low"]) or 0) for r in g), sum(int(num(r["sensor_stops"]) or 0) for r in g),
                sum(int(num(r["stops_avoided_by_2"]) or 0) for r in g),
                sum(int(num(r["stops_avoided_by_3"]) or 0) for r in g)))
    out.append("")
    if tu:
        out.append("Turn checks: %d turns, mean error %+.2f deg, largest %.1f deg." % (
            len(tu), float(np.mean([num(r["error_deg"]) for r in tu])), max(abs(num(r["error_deg"])) for r in tu)))
    for r in ev:
        if r["kind"] in ("parked", "restarted", "setting resent", "disk"):
            out.append("- %s %s %s: %s" % (r["clock"], r["kind"], r["robot"], r["text"]))
    with open(os.path.join(folder, "summary.md"), "w") as f:
        f.write("\n".join(out) + "\n")
    say("\n".join(out))


def parse_sim_events(items):
    """restart:Center@2, battery:Left=24@3 -> [(kind, robot, value, loop)]."""
    out = []
    for s in items:
        m = re.match(r"(restart|battery):(\w+)(?:=([\d.]+))?@(\d+)$", s.strip())
        if not m:
            raise SystemExit("--sim-event %s: use restart:NAME@LOOP or battery:NAME=PCT@LOOP" % s)
        out.append((m.group(1), m.group(2), m.group(3), int(m.group(4))))
    return out


def latest_dir(dry=False):
    ds = [d for d in glob.glob(os.path.join(LD.REPO, "logs", "*-doe" + ("-dryrun" if dry else ""))) if os.path.isdir(d)]
    return max(ds, key=os.path.getmtime) if ds else None


def status(folder):
    if not folder:
        say("No DOE loop folder yet (logs/<date-time>-doe).")
        return 1
    st = json.load(open(os.path.join(folder, "state.json")))
    say("DOE loop folder: %s" % folder)
    say("  loops so far: %s; drives per robot: %s" % (st.get("doe", {}).get("loops"), st.get("doe", {}).get("i")))
    for n, r in st["robots"].items():
        say("  %s at (%s, %s), heading %s%s" % (n, r.get("x"), r.get("y"), r.get("h", "?"),
                                                "  OUT: " + st["kept_out"][n] if n in st["kept_out"] else ""))
    p = os.path.join(folder, "drives.csv")
    if os.path.exists(p):
        rows = list(csv.DictReader(open(p)))
        say("  %d drives; the last 5:" % len(rows))
        for r in rows[-5:]:
            say("    %s %s drive %s, %s cm/s %s cm: %s, p95 %s cm, >3 cm %s (single %s), sensor stops %s" % (
                r["clock"], r["robot"], r["drive"], r["speed_cms"], r["length_cm"], r["result"], r["p95_cm"],
                r["over_3cm"], r["miss_single"], r["sensor_stops"]))
    p = os.path.join(folder, "battery.csv")
    if os.path.exists(p):
        last = {}
        for r in csv.DictReader(open(p)):
            last[r["robot"]] = r
        say("  battery: %s" % ", ".join("%s %s%% (%s)" % (n, r["battery_pct"], r["clock"]) for n, r in last.items()))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["run", "status", "stop", "summary"])
    ap.add_argument("--hours", type=float, default=HOURS, help="time cap (default %.0f h)" % HOURS)
    ap.add_argument("--no-frames", action="store_true", help="no frame saving (the recorder's full video only)")
    ap.add_argument("--robots", default="")
    ap.add_argument("--ignore", default="")
    ap.add_argument("--attended", action="store_true")
    ap.add_argument("--robot", action="append", default=[])
    ap.add_argument("--stream", default="127.0.0.1:8000")
    ap.add_argument("--calib", default=os.path.join(LD.REPO, "tools", "experiment", "data",
                                                    "overhead_floorcal_roam1.json"))
    ap.add_argument("--side-stream", default="127.0.0.1:8001")
    ap.add_argument("--side-calib", default="")
    ap.add_argument("--no-lights", action="store_true")
    ap.add_argument("--sim-washout", action="store_true")
    ap.add_argument("--sim-no-side", action="store_true")
    ap.add_argument("--dir")
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--turn-err", action="append", default=[])
    ap.add_argument("--sim-calibrated", action="store_true")
    ap.add_argument("--sim-event", action="append", default=[], help="--dry: restart:NAME@LOOP, battery:NAME=PCT@LOOP")
    ap.add_argument("--keep-going", action="store_true",
                    help="never park a robot on a failed turn check, heading crawl, line check or one camera's "
                         "evidence: log \"would have parked\" and carry on (safety stops still park)")
    ap.add_argument("--sim-free", default="", help="--dry: simulated free space, e.g. card=2.6,drive=6.2,fall=0.5 "
                                                   "(GB; fall: less before every loop): shows the guard's clean stop")
    a = ap.parse_args(argv)
    if a.what == "status":
        return status(a.dir or latest_dir(a.dry))
    if a.what == "summary":
        write_summary(a.dir or latest_dir(a.dry))
        return 0
    if a.what == "stop":
        d = a.dir or latest_dir(True)
        if d:
            a.dir = d
            LD.Ladder(argparse.Namespace(**{**vars(a), "what": "stop"})).stop_dry()
        say("Stopped the simulated board.")
        return 0

    def hup(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGHUP, hup)
    signal.signal(signal.SIGTERM, hup)
    L = None
    code = 0
    try:
        L = DoeLoop(a)
        L.run()
    except LadderStop as e:
        say("\nSTOPPED: %s" % e)
        code = 1
    except KeyboardInterrupt:
        if L is not None:
            stop_all(L)
        say("\nStopped (Ctrl+C, doe-stop, or the tmux session closed): every robot was sent stop.")
        code = 130
    finally:
        if L is not None:
            try:
                L.close()                       # the link's close sends stop to every robot
            finally:
                L.finish()
            say("\nDOE loop folder: %s" % L.dir)
    return code


if __name__ == "__main__":
    sys.exit(main())
