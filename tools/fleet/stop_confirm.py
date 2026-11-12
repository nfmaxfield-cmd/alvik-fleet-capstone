#!/usr/bin/env python3
"""STOP that really stops, and says so (Mission Control build, Nick 12 Nov, decision 1a).

The design audit (docs/design/mission-control/proposal.md, F1) found that STOP replied "stopped the run" at once, the
page said "running" for about 4 minutes, and the practice tour finished anyway. Mission Control's Stop sent Ctrl+C
(SIGINT) to mission_run's own process only; whatever drove the robots (run_plan, its practice shim, the simulator, the
camera bridge) got it only if mission_run passed it on in time.

What this adds:
  * The run's whole process group gets the Ctrl+C at once, as a terminal's Ctrl+C does (the SSH way that is known to
    work). Mission Control's Run, the Start screen's home runs and the calibration each run in their own group.
  * A StopWatch, from the press until every robot has CONFIRMED it stopped. The page shows "stopping..." with each
    robot's state until then, and "stopped" after. Never "ended" for a run that was stopped.
      - A robot's driver is gone: no process left in the run's group that drives robots (DRIVER_WORDS). mission_run
        itself drives while it finds the robots, so it counts until its "end" sync mark is written.
      - Real robots: once the drivers are gone (the log port is free again), a `stop` goes to every robot and each
        must ANSWER on the log port (the firmware answers "nothing running", or "run ...: stopped"). A robot that does
        not answer is sent `stop` again every RETRY_S, for as long as it takes (robots never give up, nor does this),
        and the page names it.
      - Practice: the simulated robots live inside the simulator, which is one of the drivers: gone = stopped.
  * Escalation, if a driver is still alive ESCALATE_S after the press: Ctrl+C again, then SIGTERM, then SIGKILL, to
    the drivers only (an upload or a refit started after the robots stopped is left to finish).

Switch (keep features, switch them off): MIRLT_STOP_GROUP=0 sends the Ctrl+C to the leader only, as before; the
StopWatch still reports. Decided automatically: on.

Changelog
  2026-11-12a  first version (Mission Control build)
"""
import os
import signal
import threading
import time

VERSION = "stop-confirm-2026-11-12a"

# A process whose command line holds one of these drives robots (or simulates them).
DRIVER_WORDS = ("run_plan.py", "rp_shim.py", "freeroam.py", "calib_run.py", "fleet_sim.py", "sim_shim.py",
                "sim_cameras", "tag_bridge.py", "cam_bridge.py", "e02.py", "doe_loop.py", "ladder.py", "wiggle_bind.py",
                "mission_run.py", "fake_robot")
# ... and these are after-run work that must be left to finish (never killed by the escalation).
AFTER_WORDS = ("upload_session.py", "site_fit.py", "replay_clips.py", "pi_monitor.py", "robot_sleep.py",
               "clean_uploaded.py")
# DERIVED: mission_run waits 15 s for its child after a Ctrl+C, then kills it, then 10 s for the camera bridge
# (mission_run.py, "child.wait(timeout=15)" and "p.wait(timeout=10)"): a driver alive after 25 s is past every stop
# path the run has. No DOE: the run's own limits.
ESCALATE_S = 25.0
RETRY_S = 3.0             # chosen: a robot that has not answered is sent `stop` again this often (three UDP copies a
                          # time, stop_all.STOP_REPEATS). No DOE: a repeated `stop` is harmless, and it never ends
LISTEN_S = 1.0            # stop_all.LISTEN_S: idle robots answer a `stop` at once and send a status line about once a s


def group_on():
    return os.environ.get("MIRLT_STOP_GROUP", "1") != "0"


def _stat(pid, proc="/proc"):
    """(pgid, state) of a process from /proc/<pid>/stat, or None."""
    try:
        with open(os.path.join(proc, str(pid), "stat")) as f:
            s = f.read()
        p = s[s.rindex(")") + 2:].split()
        return int(p[2]), p[0]
    except (OSError, ValueError, IndexError):
        return None


def _cmd(pid, proc="/proc"):
    try:
        with open(os.path.join(proc, str(pid), "cmdline"), "rb") as f:
            return f.read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
    except OSError:
        return ""


def group_procs(pgid, proc="/proc"):
    """[(pid, command line)] of the live (not zombie) processes in process group pgid."""
    out = []
    try:
        names = os.listdir(proc)
    except OSError:
        return out
    for n in names:
        if not n.isdigit():
            continue
        st = _stat(int(n), proc)
        if st and st[0] == pgid and st[1] != "Z":
            out.append((int(n), _cmd(int(n), proc)))
    return out


def is_driver(cmd):
    return any(w in cmd for w in DRIVER_WORDS) and not any(w in cmd for w in AFTER_WORDS)


def run_ended_mark(folder):
    """mission_run's "end" sync mark (written right after its robots were sent stop and its bridge closed)."""
    if not folder:
        return False
    for p in (os.path.join(folder, "sync_marks.jsonl"), os.path.join(os.path.dirname(folder), "sync_marks.jsonl")):
        try:
            with open(p) as f:
                if any('"what": "end"' in ln for ln in f):
                    return True
        except OSError:
            continue
    return False


def drivers(pgid, folder=None, proc="/proc"):
    """The processes in the group that still drive robots: [(pid, cmd)]."""
    out = []
    ended = None
    for pid, cmd in group_procs(pgid, proc):
        if not is_driver(cmd):
            continue
        if "mission_run.py" in cmd:
            if ended is None:
                ended = run_ended_mark(folder)
            if ended:
                continue                       # past its stop path: summarising or uploading
        out.append((pid, cmd))
    return out


def signal_group(pgid, sig):
    try:
        os.killpg(pgid, sig)
        return True
    except OSError:
        return False


class Job:
    """One thing that was running when STOP was pressed: its process group, its folder, its robots."""

    def __init__(self, name, pgid, folder=None, robots=None):
        self.name, self.pgid, self.folder = name, pgid, folder
        self.robots = list(robots or [])     # [{"name": nickname, "ref": WiFi name or address, "ip": ...}]


class StopWatch:
    """From the press until every robot has confirmed. state() for the page; a thread does the work."""

    def __init__(self, jobs, practice, listen=None, send_stop=None, escalate_s=ESCALATE_S, retry_s=RETRY_S,
                 clock=time.time, proc="/proc", say=None):
        self.jobs = list(jobs)
        self.practice = bool(practice)
        self.listen = listen                  # (seconds) -> {name: ip} or None (the port is held)
        self.send_stop = send_stop            # (ips) -> None: `stop` to these robots (and the broadcast)
        self.escalate_s, self.retry_s = float(escalate_s), float(retry_s)
        self.clock, self.proc = clock, proc
        self.say = say or (lambda s: None)
        self.t0 = clock()
        self.events = []
        self.forced = []
        self.robots = {}
        for j in self.jobs:
            for r in j.robots:
                self.robots[r["name"]] = dict(r, state="stop sent", words="stop sent", job=j.name, tries=0)
        self.drivers_gone_t = None
        self.done_t = None
        self.thread = None
        self._stage = 0                       # escalation steps taken

    # -- the work ------------------------------------------------------------------------------------------------
    def live_drivers(self):
        out = []
        for j in self.jobs:
            if j.pgid:
                out += [(j.name, pid, cmd) for pid, cmd in drivers(j.pgid, j.folder, self.proc)]
        return out

    def escalate(self, live):
        """Ctrl+C again, then SIGTERM, then SIGKILL, one step every escalate_s, to the drivers only."""
        el = self.clock() - self.t0
        step = int(el // self.escalate_s)
        if step <= self._stage or not live:
            return
        self._stage = step
        sig = {1: signal.SIGINT, 2: signal.SIGTERM}.get(step, signal.SIGKILL)
        for _job, pid, cmd in live:
            try:
                os.kill(pid, sig)
            except OSError:
                continue
            prog = next((os.path.basename(x) for x in cmd.split(" ") if x.endswith(".py")), cmd[:40])
            w = "%s still running %.0f s after STOP: sent %s (pid %d)" % (prog, el, signal.Signals(sig).name, pid)
            self.forced.append(w)
            self.events.append([round(el, 1), w])
            self.say(w)

    def step(self):
        """One pass. Returns True when every robot has confirmed."""
        if self.done_t is not None:
            return True
        live = self.live_drivers()
        now = self.clock()
        if live:
            self.escalate(live)
            for r in self.robots.values():
                r["state"], r["words"] = "stopping", "stop sent; its run is still closing (%.0f s)" % (now - self.t0)
            return False
        if self.drivers_gone_t is None:
            self.drivers_gone_t = now
            self.events.append([round(now - self.t0, 1), "every driving process has ended"])
        if self.practice or not self.robots:
            for r in self.robots.values():
                if r["state"] != "confirmed":
                    r["state"], r["words"] = "confirmed", "stopped with the simulator" if self.practice else \
                        "its run has ended"
            self.done_t = now
            return True
        # real robots: each must answer a `stop` on the log port
        todo = [r for r in self.robots.values() if r["state"] != "confirmed"]
        if not todo:
            self.done_t = now
            return True
        due = [r for r in todo if now - r.get("asked_t", -1e9) >= self.retry_s]
        if not due:
            return False
        for r in due:
            r["tries"] += 1
            r["asked_t"] = now
        if self.send_stop:
            try:
                self.send_stop([r.get("ip") for r in due if r.get("ip")])
            except Exception:  # noqa: BLE001  (a stop that cannot be sent is said, and tried again)
                pass
        heard = self.listen(LISTEN_S) if self.listen else None
        if heard is None:
            for r in due:
                r["state"], r["words"] = "waiting", "stop sent (try %d); the robots' log port is held by another " \
                                                    "program, so its answer cannot be read yet" % r["tries"]
            return False
        ips = {ip: n for n, ip in heard.items()}
        for r in todo:
            if r["name"] in heard or (r.get("ref") and r["ref"] in heard) or (r.get("ip") and r["ip"] in ips):
                r["state"], r["words"] = "confirmed", "answered: stopped"
                r["confirmed_t"] = round(now - self.t0, 1)
            elif r in due:
                r["state"], r["words"] = "no answer", "stop sent %d time%s, no answer yet (asleep, off, or out of " \
                                                      "WiFi?)" % (r["tries"], "" if r["tries"] == 1 else "s")
        if all(r["state"] == "confirmed" for r in self.robots.values()):
            self.done_t = self.clock()
            return True
        return False

    def run(self, tick_s=0.5):
        while not self.step():
            time.sleep(tick_s)

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True, name="stop-watch")
        self.thread.start()
        return self

    # -- what the page shows -------------------------------------------------------------------------------------
    def state(self):
        done = self.done_t is not None
        rs = [{k: r.get(k) for k in ("name", "ref", "state", "words", "tries", "confirmed_t")}
              for r in self.robots.values()]
        n_ok = sum(r["state"] == "confirmed" for r in rs)
        if done:
            words = "Stopped: %s confirmed %.0f s after STOP." % (
                "every robot" if rs else "the run", self.done_t - self.t0)
        else:
            words = "Stopping... %d of %d robot%s confirmed (%.0f s)." % (
                n_ok, len(rs), "" if len(rs) == 1 else "s", self.clock() - self.t0)
        return {"phase": "stopped" if done else "stopping", "t": self.t0, "words": words, "robots": rs,
                "forced": list(self.forced), "events": list(self.events)[-12:], "practice": self.practice,
                "seconds": round((self.done_t or self.clock()) - self.t0, 1), "version": VERSION}
