"""Commands that arrive (MIRLT, 2026-10-26 runner fixes, item 1).

The 2026-10-25 21:14 session's radio link lost about 1 command in 10 from the first minute (1 in 100 in the afternoon).
A lost move was resent once after 9-19 s; a lost `set` was never resent: Center's own 6 cm border was off all session,
Right's find aimed at the open edge, Left's front stop stayed at 3 cm. This module sits between the runner and the
console's Link (tools/alvik_console.py) and gives every command three things:

1. A NUMBER and an ACKNOWLEDGMENT (firmware 2026-10-26a, net.h): the runner sends "#<n> <command>"; the robot answers
   "D,ACK,<n>,new|dup,<ms>" at once, before it runs the command, and never runs a number it has already seen. A command
   with no answer within RESEND_WAITS[0] is sent again with the same number, up to len(RESEND_WAITS) times. Robots on
   older firmware get plain commands (no numbers: they would not understand them) and only (2) and the gauge apply.
2. SETTINGS READ BACK: every `set <name> <value>` is confirmed by the robot's own words for that name ("M,<name> =
   <value>", which both `set` and `get` print): no matching words within SET_CHECK_S, the runner asks `get <name>`;
   still no match, the `set` is sent again, up to SET_TRIES times. Before any move (go, turn, run, calibrate ...) the
   runner waits until that robot's settings are confirmed (wait_settings), at most SET_WAIT_MAX_S.
3. A LINK GAUGE: the share of commands that needed a resend (or were never answered) and of clock pings never
   answered, over the last LINK_WINDOW_S; logged per command in link.csv and shown on the live page. The runner warns
   at LINK_WARN and pauses new moves at LINK_PAUSE (analysis: docs/build-reports/e02-oct26-runner-fixes.md, "The link").

Every number below has its DOE note.
"""
import csv
import os
import re
import threading
import time
from collections import deque

NUMBERED_FW = "2026-10-26a"   # firmware from this version on understands "#<n> cmd" (string order = date order)

# RESEND_WAITS: how long to wait for an answer before each resend (s). The first, 1.0 s, from the robots' reply times
# in every real session's clock pings (34,300 answered pings on the Pi, 2026-10-24/25: median 18 ms, 99.9 % within
# 0.64 s, 0.07 % over 1 s): a resend after 1 s is almost never for a command that was only slow, and one that was is
# answered `dup` and not run again. The later waits grow a little so 6 sends span 8 s, longer than the bursts of
# loss seen (a lost ping is followed by another 20-40 % of the time 50 ms later, 1-9 % at random). DOE note: the
# first wait was set from data (no DOE needed); the spacing and count are chosen, and a bench test of the link
# (pings at 1, 2, 4 s spacing, near and far from the router) would show how long loss bursts last.
RESEND_WAITS = (1.0, 1.0, 1.5, 2.0, 2.5)
STOP_COPIES = 3               # a `stop` goes out 3 times 30 ms apart under one number (chosen: the watchdog already sent
STOP_GAP_S = 0.03             # 3 stops 0.1-0.2 s apart; one number keeps the copies from counting as resends). No DOE
SET_CHECK_S = 1.0             # chosen, as RESEND_WAITS[0]: no "name = value" words within this, ask `get`
SET_TRIES = 4                 # chosen: a setting sent at most this often (with the per-send resends: 24 tries in all)
SET_WAIT_MAX_S = 12.0         # chosen: a move waits at most this long for its robot's settings to be confirmed, then
                              # goes with a loud warning (6 sends x 4 tries of a set need ~10 s in the worst case).
                              # DOE note: no; a cap
SET_TOL = 0.0015              # the robot prints settings with 3 decimals ("%.3f"): within this they match
NO_VERIFY = ("calib_dir_limits",)   # not one number (look.h): no read-back possible

# The link gauge. Analysis of every real session's logs (2026-10-22 to 25; linkloss.py, bursts.py in
# tools/experiment/analyses_oct26/link_loss):
#   normal sessions (all but 21:14): lost pings 0-3.4 % per session, no 5-minute window over 4.8 % with commands
#   being sent; holds 2.9 an hour where windows were under 2 %, none in the few 2-5 % windows;
#   21:14: 8-11 % from the first minute: 8.7 holds an hour, 10.6 % of moves never reached the robot.
# LINK_WARN 5 %: above every normal session's worst 5-minute window, so a warning means "worse than any session
#   before 21:14". DOE note: set from data, no DOE.
# LINK_PAUSE 15 %: with resends a command is lost only if every send is lost; what stays at risk is time: a `stop` that
#   needs a resend arrives about 1 s late (10 cm at full speed). Sent 3 times (STOP_COPIES), a stop is late only if
#   all copies are lost: about p x q x q, q = the chance a lost packet is followed by another (0.20 in the 21:14
#   session, 0.31-0.42 in the others). At p = 15 % that is 0.6-2.6 % of stops, against 1.2 % for a single stop on a
#   normal link (p 1.2 %): above 15 % the stops are less reliable than they used to be. DOE note: yes, chosen within
#   a derived range (8-29 % depending on q); the bench link test above would measure q at 1 s spacing and set it.
# LINK_RESUME: a pause ends when the last LINK_RESUME_S are back under LINK_PAUSE (chosen, no DOE: a debounce; the
#   pause itself needs 2 standard errors of evidence, LINK_PAUSE_SE, so it does not flap at a steady 10-15 %).
LINK_PAUSE_SE = 2.0           # chosen: 2 standard errors (about 98 % sure the share is above LINK_PAUSE). No DOE: a
                              # confidence level
LINK_WINDOW_S = 300.0
LINK_MIN_N = 30               # chosen: fewer events than this in the window say nothing (pings alone give ~50 a minute)
LINK_WARN = 0.05
LINK_PAUSE = 0.15
LINK_RESUME_S = 120.0
LINK_RESUME_MIN_N = 15
SERIES_MIN = 30              # the live page's link chart: the last 30 minutes, one point a minute (display only)
SILENT_S = 60.0               # chosen: a robot that answers nothing for a minute is off, asleep or flat, not on a bad link
                              # (pings go every 10 s: 18 in a row lost at 10 % loss is 1e-18). No DOE: a definition
PING_LOST_S = 2.0             # a clock ping unanswered this long is lost (99.98 % of pings answered within 2 s)

_WORDS = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(-?[0-9.]+(?:[eE][-+]?[0-9]+)?|nan|-?inf)")


def numbered_fw(version):
    return bool(version) and str(version) >= NUMBERED_FW


class Gauge:
    """(t, ok) events: a command answered on its first send, or not (resent / never answered); a clock ping answered
    or not. share(window) = the share not ok."""

    def __init__(self):
        self.ev = deque(maxlen=20000)
        self.lock = threading.Lock()
        self.last_ok = {}          # robot -> when it last answered anything

    def add(self, ok, robot=None, t=None, kind="cmd"):
        t = time.time() if t is None else t
        with self.lock:
            self.ev.append((t, bool(ok), robot, kind))
            if ok and robot is not None:
                self.last_ok[robot] = max(t, self.last_ok.get(robot, 0.0))

    def silent(self, now=None):
        """Robots that have answered nothing for SILENT_S (switched off, asleep, flat): not the link's fault, so left
        out of the share (review, 2026-10-26: one silent robot of three would read as 33 % and pause everything)."""
        now = time.time() if now is None else now
        with self.lock:
            robots = {e[2] for e in self.ev if e[2] is not None}
            return {r for r in robots if now - self.last_ok.get(r, -1e18) > SILENT_S}

    def share(self, window_s=LINK_WINDOW_S, now=None, min_n=LINK_MIN_N, robot=None):
        """(share not ok, n events) over the last window_s, robots that are silent left out; share None below min_n
        events."""
        now = time.time() if now is None else now
        quiet = self.silent(now)
        with self.lock:
            ev = [e for e in self.ev if e[0] >= now - window_s and (robot is None or e[2] == robot)
                  and e[2] not in quiet]
        n = len(ev)
        if n < min_n:
            return None, n
        return sum(1 for e in ev if not e[1]) / float(n), n

    def level(self, now=None):
        """'ok' | 'warn' | 'pause' | None (too few events), from the share over LINK_WINDOW_S. Pause only when the share
        is above LINK_PAUSE by LINK_PAUSE_SE standard errors (dry run D3, 2026-10-26: at a steady 12 % the 5-minute
        share wandered 9-16 % and touched 15 % by chance)."""
        s, n = self.share(now=now)
        if s is None:
            return None
        se = (s * (1.0 - s) / n) ** 0.5
        return "pause" if s - LINK_PAUSE_SE * se >= LINK_PAUSE else "warn" if s >= LINK_WARN else "ok"

    def recovered(self, now=None):
        """A pause ends when the last LINK_RESUME_S are back under the pause level."""
        s, _ = self.share(LINK_RESUME_S, now=now, min_n=LINK_RESUME_MIN_N)
        return s is not None and s < LINK_PAUSE


class ReliableLink:
    """The console's Link with numbered, acknowledged commands, settings read back, and the link gauge. Same
    interface as Link for what the runner uses (send_name, send_ip, send_all, robots, close, session, summary)."""

    def __init__(self, link, firmware=None, folder=None, say=None, waits=RESEND_WAITS):
        self.link = link
        self.session = link.session
        self.firmware = firmware if firmware is not None else {}      # robot -> firmware version (the listener's)
        self.say = say or (lambda *a, **k: None)
        self.waits = tuple(waits)
        self.lock = threading.RLock()
        self.cv = threading.Condition(self.lock)
        self.n = int(time.time() * 10) % 2000000000 or 1     # from the clock: a restarted runner never reuses one
        self.pending = {}          # n -> dict(robot, cmd, t0, tries, next)
        self.acked = deque(maxlen=4000)   # (n, robot, cmd, t_sent, t_ack, tries)
        self.acks = {}             # n -> (t_recv, 'new'|'dup')
        self.sets = {}             # robot -> {name: dict(value, t, tries, asked)}
        self.unacked = deque(maxlen=500)  # (t, robot, cmd)
        self.gauge = Gauge()
        self.counts = dict(sent=0, resends=0, unacked=0, first_try=0, set_resends=0, set_failed=0)
        self.pings_seen = set()
        self.quit = threading.Event()
        self.csv = None
        if folder:
            try:
                path = os.path.join(folder, "link.csv")
                new = not os.path.exists(path)
                self.csvf = open(path, "a", newline="")
                self.csv = csv.writer(self.csvf)
                if new:
                    self.csv.writerow(["t", "robot", "n", "cmd", "event", "tries", "wait_ms"])
            except OSError:
                self.csv = None
        self.session.listeners.append(self._on_line)
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # --- the Link interface ---------------------------------------------------------------------------------
    def robots(self):
        return self.link.robots()

    def send_ip(self, ip, cmd):
        return self.link.send_ip(ip, cmd)

    def send_all(self, cmd):
        for n in self.robots():
            self.send_name(n, cmd)
        if not self.robots():
            self.link.send_all(cmd)

    def close(self):
        self.quit.set()
        try:
            self.link.close()
        finally:
            try:
                if self.csv is not None:
                    self.csvf.close()
            except OSError:
                pass

    def summary(self):
        return self.link.summary()

    def __getattr__(self, k):            # anything else the console's Link has
        return getattr(self.link, k)

    # --- sending ----------------------------------------------------------------------------------------------
    def numbered(self, robot):
        return numbered_fw(self.firmware.get(robot))

    def send_name(self, robot, cmd):
        """Send cmd to robot: numbered (and resent until answered) on firmware that understands it; a `set` is also
        read back. Returns what the console's send_name returns (False: the robot is not connected)."""
        cmd = cmd.strip()
        verb = cmd.split(" ", 1)[0].lower()
        if verb == "set":
            self._note_set(robot, cmd)
        if not self.numbered(robot) or verb == "clock":
            return self.link.send_name(robot, cmd)
        with self.lock:
            self.n += 1
            n = self.n
            now = time.time()
            self.pending[n] = dict(robot=robot, cmd=cmd, t0=now, tries=1, next=now + self.waits[0])
            self.counts["sent"] += 1
        ok = self.link.send_name(robot, "#%d %s" % (n, cmd))
        if not ok:
            with self.lock:
                self.pending.pop(n, None)
            return ok
        if verb == "stop":
            def copies():
                for _ in range(STOP_COPIES - 1):
                    time.sleep(STOP_GAP_S)
                    self.link.send_name(robot, "#%d %s" % (n, cmd))
            threading.Thread(target=copies, daemon=True).start()
        return ok

    def ack_time(self, robot, cmd):
        """When the robot answered this exact command (its first answer, laptop time), or None."""
        with self.lock:
            for n, rb, c, t0, ta, tries in reversed(self.acked):
                if rb == robot and c == cmd:
                    return ta
        return None

    # --- settings read back -----------------------------------------------------------------------------------
    def _note_set(self, robot, cmd):
        parts = cmd.split()
        if len(parts) < 3 or parts[1] in NO_VERIFY:
            return
        try:
            v = float(parts[2])
        except ValueError:
            return
        with self.lock:
            st = self.sets.setdefault(robot, {})
            st[parts[1]] = dict(value=v, t=time.time(), tries=1, asked=False, ok=False, cmd=cmd)

    def settings_pending(self, robot):
        with self.lock:
            return [k for k, s in (self.sets.get(robot) or {}).items() if not s["ok"] and not s.get("failed")]

    def wait_settings(self, robot, timeout=SET_WAIT_MAX_S):
        """Block until every setting sent to robot is confirmed (or given up on), at most timeout. Returns the names
        still not confirmed ([] = all good)."""
        end = time.time() + timeout
        with self.cv:
            while self.settings_pending(robot) and time.time() < end and not self.quit.is_set():
                self.cv.wait(min(0.1, max(0.0, end - time.time())))
            return self.settings_pending(robot)

    def failed_settings(self, robot=None):
        with self.lock:
            return [(r, k, s["value"]) for r, d in self.sets.items() for k, s in d.items()
                    if s.get("failed") and (robot is None or r == robot)]

    # --- what the robots say ----------------------------------------------------------------------------------
    def _on_line(self, name, kind, rtype, vals):
        if kind == "D" and rtype == "CLOCK":
            # a clock ping answered (the console pairs it with its question): an "ok" for the gauge, counted here
            # because an answer can come and go between two looks at clock_sent (dry run D1, 2026-10-26: counting
            # only what was seen leaving clock_sent read 16 % lost at a 5 % drop)
            v = vals if isinstance(vals, (list, tuple)) else str(vals).split(",")
            tok = v[0] if v else ""
            with self.lock:
                if tok and tok not in self.pings_seen:
                    self.pings_seen.add(tok)
                    self.gauge.add(True, name, kind="ping")
            return
        if kind == "D" and rtype == "ACK":
            v = vals if isinstance(vals, (list, tuple)) else str(vals).split(",")
            try:
                n = int(v[0])
            except (ValueError, IndexError):
                return
            res = v[1] if len(v) > 1 else "new"
            self._acked(n, res)
        elif kind == "M":
            text = vals if isinstance(vals, str) else ",".join(vals)
            m = _WORDS.match(text)
            if m:
                self._setting_said(name, m.group(1), m.group(2))

    def _acked(self, n, res):
        with self.cv:
            p = self.pending.pop(n, None)
            if n not in self.acks:
                self.acks[n] = (time.time(), res)
                if len(self.acks) > 6000:
                    for k in list(self.acks)[:3000]:
                        del self.acks[k]
            if p is None:
                return
            t = time.time()
            self.acked.append((n, p["robot"], p["cmd"], p["t0"], t, p["tries"]))
            first = p["tries"] == 1
            if first:
                self.counts["first_try"] += 1
            self.gauge.add(first, p["robot"], t)
            self._log(t, p["robot"], n, p["cmd"], "ack" if first else "ack_after_resend", p["tries"],
                      1000.0 * (t - p["t0"]))
            self.cv.notify_all()

    def _setting_said(self, robot, name, val):
        try:
            v = float(val)
        except ValueError:
            return
        with self.cv:
            s = (self.sets.get(robot) or {}).get(name)
            if s is None or s["ok"]:
                return
            if abs(v - s["value"]) <= SET_TOL or (s["value"] != 0 and abs(v / s["value"] - 1.0) < 1e-4):
                s["ok"] = True
                s["t_ok"] = time.time()
                self.cv.notify_all()
            elif s.get("asked"):
                s["mismatch"] = v          # the answer to our `get` says it is still the old value: resend

    # --- the background loop: resends, read-backs, pings ------------------------------------------------------
    def _loop(self):
        while not self.quit.wait(0.05):
            try:
                self._resends()
                self._readbacks()
                self._pings()
            except Exception as e:          # never let the link's helper die
                self.say("  (link helper: %s)" % e, quiet=True)

    def _resends(self):
        now = time.time()
        todo = []
        with self.lock:
            for n, p in list(self.pending.items()):
                if now < p["next"]:
                    continue
                if p["tries"] > len(self.waits):
                    self.pending.pop(n, None)
                    self.counts["unacked"] += 1
                    self.unacked.append((now, p["robot"], p["cmd"]))
                    self.gauge.add(False, p["robot"], now)
                    self._log(now, p["robot"], n, p["cmd"], "never_answered", p["tries"], 1000.0 * (now - p["t0"]))
                    continue
                p["tries"] += 1
                w = self.waits[min(len(self.waits) - 1, p["tries"] - 1)]
                p["next"] = now + w
                todo.append((n, p))
        for n, p in todo:
            self.counts["resends"] += 1
            self._log(now, p["robot"], n, p["cmd"], "resend", p["tries"], 1000.0 * (now - p["t0"]))
            self.link.send_name(p["robot"], "#%d %s" % (n, p["cmd"]))

    def _readbacks(self):
        now = time.time()
        todo = []
        with self.cv:
            for robot, d in self.sets.items():
                for name, s in d.items():
                    if s["ok"] or s.get("failed"):
                        continue
                    if s.get("mismatch") is not None or (s["asked"] and now - s["t_asked"] > SET_CHECK_S +
                                                          (self.waits[0] * 2 if self.numbered(robot) else 0)):
                        # the robot says (or stays silent about) something else: send it again
                        if s["tries"] >= SET_TRIES:
                            s["failed"] = True
                            self.counts["set_failed"] += 1
                            self._log(now, robot, "", s["cmd"], "set_failed", s["tries"], 1000.0 * (now - s["t"]))
                            self.cv.notify_all()
                            continue
                        s["tries"] += 1
                        s["asked"] = False
                        s["mismatch"] = None
                        s["t"] = now
                        self.counts["set_resends"] += 1
                        todo.append((robot, s["cmd"], "set_resend", s["tries"]))
                    elif not s["asked"] and now - s["t"] > SET_CHECK_S:
                        s["asked"] = True
                        s["t_asked"] = now
                        todo.append((robot, "get %s" % name, "get", s["tries"]))
        for robot, cmd, ev, tries in todo:
            self._log(now, robot, "", cmd, ev, tries, "")
            if ev == "set_resend":
                # (not through _note_set: the entry is updated above)
                if self.numbered(robot):
                    with self.lock:
                        self.n += 1
                        n = self.n
                        self.pending[n] = dict(robot=robot, cmd=cmd, t0=now, tries=1, next=now + self.waits[0])
                    self.link.send_name(robot, "#%d %s" % (n, cmd))
                else:
                    self.link.send_name(robot, cmd)
            else:
                self.send_name(robot, cmd)

    def _pings(self):
        """Clock pings (the console's own, every 10 s per robot, unnumbered) answered or not: the gauge's steady
        part, also for robots on older firmware and while no commands are sent."""
        s = self.session
        now = time.time()
        try:
            with s.lock:
                sent = dict(s.clock_sent)
        except Exception:
            return
        # answered pings are popped from clock_sent by the console (and counted in _on_line); one still there after
        # PING_LOST_S is lost
        with self.lock:
            for tok, (name, t_send) in sent.items():
                if t_send is None or tok in self.pings_seen:
                    continue
                if now - t_send > PING_LOST_S:
                    self.pings_seen.add(tok)
                    self.gauge.add(False, name, t_send, kind="ping")
            if len(self.pings_seen) > 20000:
                keep = set(sent)
                self.pings_seen = {t for t in self.pings_seen if t in keep}

    # --- the log and the page ---------------------------------------------------------------------------------
    def _log(self, t, robot, n, cmd, event, tries, wait_ms):
        if self.csv is None:
            return
        try:
            self.csv.writerow(["%.3f" % t, robot, n, cmd[:80], event, tries,
                               "" if wait_ms == "" else "%.0f" % wait_ms])
            self.csvf.flush()
        except (OSError, ValueError):
            pass

    def status(self, now=None):
        """For the live page and the state: the share needing a resend now, its level, and counts."""
        now = time.time() if now is None else now
        s, n = self.gauge.share(now=now)
        s1, n1 = self.gauge.share(60.0, now=now, min_n=1)
        series = []
        for k in range(SERIES_MIN, 0, -1):                 # per minute, oldest first: [minutes ago, share, events]
            a = now - 60.0 * k
            with self.gauge.lock:
                ev = [e for e in self.gauge.ev if a <= e[0] < a + 60.0]
            series.append([k, None if not ev else round(sum(1 for e in ev if not e[1]) / float(len(ev)), 4), len(ev)])
        return dict(series=series, series_min=SERIES_MIN, silent=sorted(self.gauge.silent(now)), share=None if s is None else round(s, 4), n=n, share_1min=None if s1 is None else round(s1, 4),
                    level=self.gauge.level(now=now), warn=LINK_WARN, pause=LINK_PAUSE,
                    counts=dict(self.counts), set_failed=[list(x) for x in self.failed_settings()],
                    numbered=sorted(r for r in self.robots() if self.numbered(r)))
