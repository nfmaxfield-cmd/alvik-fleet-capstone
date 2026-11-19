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

(e23, school 16 and 18 Nov, cause A) A Stop pressed BEFORE the robots are named (no binding yet) watched the site's
slot names (Alpha / Bravo, no address at school). No robot ever answers under a slot name there, so the watcher sent
`stop` to the whole WiFi every 3 s for ever and listened on the robots' log port each time: the next runs' compass
checks and follow runs were stopped 3 s after they began, and two starts found "port 5005 is in use". Now:
  * a robot that is only a GUESS (not from this run's binding) and has neither an address nor a WiFi name cannot be
    told apart from any other robot: it is settled by the first `stop` that went out with the log port read (every
    robot that answered is listed), not asked for by name again;
  * cancel(): a new Start ends the watcher (no more `stop`s, the log port free) once every driver has ended;
  * the listen and the `stop` are skipped while the watcher is cancelled.
Switch MIRLT_STOP_UNNAMED=wait: the watch as before e23 (asks for the slot names until someone presses "carry on").

Changelog
  2026-11-12a  first version (Mission Control build)
  2026-11-19e23  robots that were never named are settled by one answered `stop` (school 16 / 18 Nov: the watcher that
                 never ended); cancel() for a new Start; MIRLT_STOP_UNNAMED
"""
import os
import signal
import threading
import time

VERSION = "stop-confirm-2026-11-19e23"   # e23: unnamed robots settled by one answered stop, cancel(). Before: "stop-confirm-2026-11-12a"

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


def unnamed_once():
    """(e23) A robot that was never named in this run is settled by one answered `stop` (MIRLT_STOP_UNNAMED=once, the
    default); wait: asked for by its slot name for ever, as before e23."""
    try:
        import safety_group as SG
        return SG.value("MIRLT_STOP_UNNAMED") != "wait"
    except Exception:  # noqa: BLE001  (the stop never depends on another module)
        return os.environ.get("MIRLT_STOP_UNNAMED", "once") != "wait"


def nameable(r):
    """Can this robot's own answer be recognised? Robots from the run's binding always (their WiFi name and address
    are known); a guessed robot only when it has an address or a WiFi name (home's fixed addresses)."""
    return bool(r.get("bound", True) or r.get("ip") or r.get("ref"))


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
        self.cancelled = None                 # (e23) why it was ended early (a new Start)
        self.heard_all = {}                   # (e23) every robot that answered a `stop` of this watch: {name: ip}
        self.unnamed_once = unnamed_once()

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
        if self.done_t is not None or self.cancelled:
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
        if self.cancelled:
            return True
        ips = {ip: n for n, ip in heard.items()}
        self.heard_all.update(heard)
        for r in todo:
            if r["name"] in heard or (r.get("ref") and r["ref"] in heard) or (r.get("ip") and r["ip"] in ips):
                r["state"], r["words"] = "confirmed", "answered: stopped"
                r["confirmed_t"] = round(now - self.t0, 1)
            elif self.unnamed_once and not nameable(r):
                # (e23) never named in this run and no address: nobody can answer as "Alpha". The `stop` went to every
                # robot on the WiFi and the log port was read: that is all that can be known, so it is settled here
                who = ", ".join("%s at %s" % kv for kv in sorted(self.heard_all.items())) or "no robot"
                r["state"] = "confirmed"
                r["words"] = "not named yet in this run: `stop` went to every robot on the WiFi (%s answered)" % who
                r["confirmed_t"] = round(now - self.t0, 1)
                r["unnamed"] = True
            elif r in due:
                r["state"], r["words"] = "no answer", "stop sent %d time%s, no answer yet (asleep, off, or out of " \
                                                      "WiFi?)" % (r["tries"], "" if r["tries"] == 1 else "s")
        if all(r["state"] == "confirmed" for r in self.robots.values()):
            self.done_t = self.clock()
            return True
        return False

    def go_on(self, who=""):
        """The person says to carry on without the robots that have not answered (switched off, asleep, out of the
        WiFi). Only once every driver has ended; those robots are named in the record as not confirmed."""
        if self.done_t is not None or self.drivers_gone_t is None:
            return False
        left = [r["name"] for r in self.robots.values() if r["state"] != "confirmed"]
        for r in self.robots.values():
            if r["state"] != "confirmed":
                r["state"], r["words"] = "not confirmed", "no answer; you said to carry on without it"
        self.events.append([round(self.clock() - self.t0, 1), "carried on without %s (%s)" % (", ".join(left) or
                                                                                            "nobody", who or "page")])
        self.done_t = self.clock()
        return True

    def cancel(self, why="a new run was started"):
        """(e23) End the watch early: no more `stop`s, the robots' log port left free. Only once every driver has ended
        (the escalation must finish its work first): returns False while one is still alive. The thread is joined, so
        the port is free when this returns."""
        if self.done_t is not None:
            return True
        if self.drivers_gone_t is None and self.live_drivers():
            return False
        left = [r["name"] for r in self.robots.values() if r["state"] != "confirmed"]
        for r in self.robots.values():
            if r["state"] != "confirmed":
                r["state"], r["words"] = "not confirmed", "no answer before %s" % why
        self.events.append([round(self.clock() - self.t0, 1), "ended: %s%s" % (
            why, (" (%s had not answered)" % ", ".join(left)) if left else "")])
        self.cancelled = why
        self.done_t = self.clock()
        th = self.thread
        if th is not None and th.is_alive() and th is not threading.current_thread():
            th.join(LISTEN_S + 2.0)           # at most one listen (LISTEN_S) is under way
        return True

    def add_jobs(self, jobs):
        """(e23, one watcher at a time) A second Stop while this watch runs: anything new that was running joins it."""
        have = {j.pgid for j in self.jobs}
        for j in jobs:
            if j.pgid in have:
                continue
            self.jobs.append(j)
            for r in j.robots:
                self.robots.setdefault(r["name"], dict(r, state="stop sent", words="stop sent", job=j.name, tries=0))
        self.events.append([round(self.clock() - self.t0, 1), "Stop pressed again: the same watch carries on"])

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
        rs = [{k: r.get(k) for k in ("name", "ref", "state", "words", "tries", "confirmed_t", "unnamed")}
              for r in self.robots.values()]
        n_ok = sum(r["state"] == "confirmed" for r in rs)
        if done and self.cancelled:
            words = "Stopped (%s)." % self.cancelled
        elif done and any(r["state"] == "not confirmed" for r in rs):
            words = "Stopped; %s never answered (you carried on without %s)." % (
                ", ".join(r["name"] for r in rs if r["state"] == "not confirmed"),
                "it" if sum(r["state"] == "not confirmed" for r in rs) == 1 else "them")
        elif done:
            words = "Stopped: %s confirmed %.0f s after STOP." % (
                "every robot" if rs else "the run", self.done_t - self.t0)
        else:
            words = "Stopping... %d of %d robot%s confirmed (%.0f s)." % (
                n_ok, len(rs), "" if len(rs) == 1 else "s", self.clock() - self.t0)
        return {"phase": "stopped" if done else "stopping", "can_go_on": not done and self.drivers_gone_t is not None, "t": self.t0, "words": words, "robots": rs,
                "forced": list(self.forced), "events": list(self.events)[-12:], "practice": self.practice,
                "seconds": round((self.done_t or self.clock()) - self.t0, 1), "version": VERSION}
