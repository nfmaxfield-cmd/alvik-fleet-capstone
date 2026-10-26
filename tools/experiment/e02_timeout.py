"""Session timeouts: an unattended session always ends, saves and uploads (Nick, 2026-10-26).

2026-10-25 night: a `--hours 1.5` session started 21:14 reached its planned end about 22:45, then a hold sat unanswered
all night while Nick slept. Nothing uploaded until he stopped it at 06:40, and the recorder filled 23 GB of video.

Three rules (each a command-line option of e02.py; 0 switches that rule off):
  1. --after-end-timeout-min (10): after the session's planned time is up, a hold or a question nobody answers for
     this long ends the session. A question asked before the planned end gets this long after the planned end.
  2. --answer-timeout-min (30): at any time, a hold or a question nobody answers for this long ends the session
     ("ended: no answer for 30 min").
  3. --hard-cap-min (30): the planned hours plus this is a hard cap. The runner starts no new run, lets the current
     one finish and park if it does so within WRAP_GRACE_S, else every robot is just stopped.
Ending means: every robot stopped, the state saved, the runner exits normally, so run-e02-pi.sh's window goes on to
upload_session.py --stop-recorder (the upload, and the recorder stopped).

A question that already takes its own default after a time limit (the live page's setup questions: the default after
15 min, WEB_ASK_TIMEOUT_S; a help call: --help-wait-min) keeps that: it never strands a session. Only the hard cap
applies to it.

The planned end is this launch's start plus --hours (a --resume launch counts from its own start).

DOE note (all four numbers here): chosen convenience settings, not measured. They decide only how long an unattended
session waits for Nick before giving up, not anything the experiment measures, so a design of experiment is not
needed for them. Change them on the command line if they turn out too short or too long in practice.
"""
import datetime as dt
import time

ANSWER_TIMEOUT_MIN = 30.0     # chosen (Nick, 2026-10-26); convenience setting, no DOE needed (see above)
AFTER_END_TIMEOUT_MIN = 10.0  # chosen (Nick, 2026-10-26); convenience setting, no DOE needed
HARD_CAP_MIN = 30.0           # chosen (Nick, 2026-10-26): planned hours + this; convenience setting, no DOE needed
WRAP_GRACE_S = 180.0          # chosen: at the hard cap, the current run gets this long to finish and park (a run with
                              # its parking takes about 70 s, PL.RUN_S) before every robot is just stopped. Convenience
                              # setting, no DOE needed
ENSURE_END_S = 90.0           # chosen: after a timeout stops the session, the runner gets this long to reach its own
                              # end; if it is stuck somewhere that never checks (a console input(), a long sleep), it
                              # is interrupted like Ctrl+C. Convenience setting, no DOE needed

# reason keys -> words (the log, e02_state.json "session_end", the analysis report)
NO_ANSWER = "no_answer"
NO_ANSWER_AFTER_END = "no_answer_after_end"
CAP = "cap"


def hm(t):
    """'23:05' (this computer's local time)."""
    return dt.datetime.fromtimestamp(t).strftime("%H:%M")


def fmt_min(m):
    return ("%g" % round(m, 2)) if m != int(m) else "%d" % m


class SessionClock:
    """When a wait on Nick ends the session. t_start: this launch's start (epoch s); hours: --hours."""

    def __init__(self, t_start, hours, answer_min=ANSWER_TIMEOUT_MIN, after_end_min=AFTER_END_TIMEOUT_MIN,
                 cap_min=HARD_CAP_MIN):
        self.t_start = float(t_start)
        self.hours = float(hours or 0.0)
        self.planned_end = self.t_start + self.hours * 3600.0
        self.answer_min = float(answer_min or 0.0)
        self.after_end_min = float(after_end_min or 0.0)
        self.cap_min = float(cap_min or 0.0)
        self.cap = self.planned_end + self.cap_min * 60.0 if self.cap_min > 0 else None

    def wait_deadline(self, t_asked, own_timeout_s=None):
        """(t, key): when a hold or question first put to Nick at t_asked ends the session if nobody answers, or
        (None, None). own_timeout_s: the question takes its own default after this long (then only the hard cap
        applies, and only if it comes first)."""
        c = []
        if own_timeout_s is None:
            if self.answer_min > 0:
                c.append((t_asked + self.answer_min * 60.0, NO_ANSWER))
            if self.after_end_min > 0:
                c.append((max(t_asked, self.planned_end) + self.after_end_min * 60.0, NO_ANSWER_AFTER_END))
        if self.cap is not None:
            c.append((self.cap, CAP))
        if not c:
            return None, None
        t, key = min(c)
        if own_timeout_s is not None and t_asked + own_timeout_s <= t:
            return None, None                   # its own default comes first: the session is not ended by it
        return t, key

    def why(self, key, t_asked=None, t_now=None):
        """Plain words for the reason (after 'ended: ')."""
        if key == NO_ANSWER:
            return "no answer for %s min" % fmt_min(self.answer_min)
        if key == NO_ANSWER_AFTER_END:
            return "no answer for %s min after the planned end (%s)" % (fmt_min(self.after_end_min),
                                                                       hm(self.planned_end))
        if key == CAP:
            return "hard cap reached, planned time + %s min (%s)" % (fmt_min(self.cap_min), hm(self.cap))
        return str(key)

    def page_words(self, t_end, uploads=True):
        """The live page's line under a hold or question: 'No answer by 23:05 ends the session and uploads'."""
        return "No answer by %s ends the session%s" % (hm(t_end), " and uploads" if uploads else "")

    def words(self):
        """One line for the start of the session's log."""
        parts = ["planned end %s" % hm(self.planned_end)]
        if self.after_end_min > 0:
            parts.append("after it, %s min with no answer ends the session" % fmt_min(self.after_end_min))
        if self.answer_min > 0:
            parts.append("at any time, %s min with no answer ends it" % fmt_min(self.answer_min))
        if self.cap is not None:
            parts.append("hard cap %s" % hm(self.cap))
        return "Session timeouts: " + "; ".join(parts) + "."


def from_args(a, t_start=None):
    """The clock for e02.py's arguments (a stand-in without the options gets the defaults)."""
    return SessionClock(time.time() if t_start is None else t_start, getattr(a, "hours", 2.0),
                        getattr(a, "answer_timeout_min", ANSWER_TIMEOUT_MIN),
                        getattr(a, "after_end_timeout_min", AFTER_END_TIMEOUT_MIN),
                        getattr(a, "hard_cap_min", HARD_CAP_MIN))
