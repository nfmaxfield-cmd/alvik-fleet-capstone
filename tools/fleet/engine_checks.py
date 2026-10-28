"""Engine checks for every run on the robots (one system, Nick 2026-10-27 18:26-18:36; build e02-oct28-one-system.md).
run_plan.py uses these at start-up and during a run, so the ladder (which runs run_plan for each step), Mission
Control and `./run-fleet-pi.sh monday` all get the same checks:

  DayState        what the engine knows about each robot today (~/.config/mirlt/day/<date>.json): its boot ID when
                  it was last calibrated, the settings sent, the warm-up's wheel / gyro results. A new boot ID = the
                  robot restarted (a sleep check-in, a power cycle): calibrate again and resend its corrections.
                  Warm-up results are kept for the day and reused (Nick 18:26: "they're allowed to retain that data").
  corrections_for the driving corrections a robot gets (e02_fit.PRIOR, the per-robot file corrections.json, then
                  today's warm-up results within WARMUP_SCALE_MAX)
  parse_status    firmware, battery and boot ID from a robot's `status` answer
  plan_gap        a plan's closest approach of two bodies, not counting robots already closer than that where they
                  stand at the start (they can only be planned apart): the "refuse any plan under 8 cm" gate
  silent          a robot whose records stopped while it should be running

Chosen numbers (DOE notes here and in docs/doe-backlog.md, "One system"):
"""
import datetime as dt
import itertools
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)

BATTERY_FLOOR_PCT = 25     # Nick's ladder floor (a run at 20 % sagged on 2026-10-25), now checked before every run, so a
                           # resumed ladder step or a second Monday run checks it too (the 15:27 L4 started Left at 23 %).
                           # No DOE: a floor below which nothing starts
SILENT_S = 4.0             # chosen: no record at all from a robot for this long while its follow run should be going =
                           # it went silent (battery dead, WiFi gone, restarted): it becomes a fixed obstacle and the
                           # others re-plan. FOLLOW records come 10 a second; the simulated 3 s WiFi drop the step
                           # follower rides out (DESIGN.md) must not count, so 4 s; a robot that is really out only
                           # finishes its buffered steps, which the obstacle covers. DOE note: 3 / 4 / 6 s in the
                           # simulator (5 % and 15 % packet loss, WiFi drops of 1-4 s, a dead robot) against false
                           # "silent" calls and the time the others stand still
MIN_GAP_CM = 8.0           # Nick (2026-10-27 16:43): "refuse any plan under 8 cm"; he wants to narrow it this week.
                           # DOE note: 8 / 7 / 6 cm on the board: the camera's closest approach over 10 Chickens and 10
                           # crossings against the plan's (the 8 = two half-widths + ~3 cm camera error, rounded up)
WARMUP_SCALE_MAX = 0.05    # chosen: a warm-up wheel or gyro ratio further than 5 % from 1 is not applied (an angled
                           # camera made a false +6.9 % in the Monday sweep); it is reported instead. DOE note: the
                           # warm-up's measured ratio against L2's turn check on the same robots, 3 days (does a 3-5 %
                           # ratio repeat?)
READBACK_TRIES = 3         # read back every setting sent, resend a mismatch, up to this many rounds (Center's lost
                           # wheel_scale `set` at 14:12). No DOE: a retry count


def conf_dir():
    return os.path.expanduser(os.environ.get("MIRLT_CONF_DIR", "~/.config/mirlt"))


def today():
    return dt.datetime.now().strftime("%Y%m%d")


class DayState:
    """Per robot, today: {"boot": "C12D", "calibrated_boot": "C12D", "calibrated_t": ..., "warmup": {"wheel_ratio",
    "gyro_ratio", "t"}, "sent": {param: value}, "battery": pct, "seen_t": ...}. One JSON file per day."""

    def __init__(self, path=None):
        self.path = path or os.path.join(conf_dir(), "day", today() + ".json")
        self.d = {}
        try:
            with open(self.path) as f:
                self.d = json.load(f) or {}
        except (OSError, ValueError):
            self.d = {}

    def robot(self, name):
        return self.d.setdefault("robots", {}).setdefault(name, {})

    def save(self):
        os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.d, f, indent=1)
        os.replace(tmp, self.path)

    def restarted(self, name, boot):
        """True when the robot's boot ID differs from the one it had when last calibrated today (or it was never
        calibrated today by the engine). None when its boot ID is not known."""
        if not boot:
            return None
        r = self.robot(name)
        return r.get("calibrated_boot") != boot

    def note_calibrated(self, name, boot, t=None):
        import time
        r = self.robot(name)
        r["calibrated_boot"] = boot
        r["calibrated_t"] = round(t or time.time(), 1)

    def note_warmup(self, name, wheel_ratio=None, gyro_ratio=None, t=None):
        import time
        r = self.robot(name)
        w = r.setdefault("warmup", {})
        if wheel_ratio:
            w["wheel_ratio"] = float(wheel_ratio)
        if gyro_ratio:
            w["gyro_ratio"] = float(gyro_ratio)
        w["t"] = round(t or time.time(), 1)


def base_corrections(name):
    """e02_fit.PRIOR (the robots' gyros under-read ~7.5 %, wheels over-read ~2 %), then ~/.config/mirlt/corrections.json
    for that robot (the ladder's rule since 2026-10-27 13:58)."""
    try:
        sys.path.insert(0, os.path.join(TOOLS, "experiment"))
        import e02_fit as FIT
        c = dict(FIT.PRIOR)
    except Exception:  # noqa: BLE001
        c = dict(gyro_scale=1.075, gyro_drift_dps=0.0, wheel_scale=0.979)
    try:
        with open(os.path.join(conf_dir(), "corrections.json")) as f:
            c.update((json.load(f) or {}).get(name) or {})
    except (OSError, ValueError):
        pass
    return {k: float(c[k]) for k in ("gyro_scale", "gyro_drift_dps", "wheel_scale")}


def corrections_for(name, day=None, notes=None):
    """The corrections to send: base_corrections, times today's warm-up ratios (real / stored) when within
    WARMUP_SCALE_MAX of 1."""
    c = base_corrections(name)
    w = (day.robot(name).get("warmup") if day else None) or {}
    for key, ratio in (("wheel_scale", w.get("wheel_ratio")), ("gyro_scale", w.get("gyro_ratio"))):
        if not ratio:
            continue
        if abs(ratio - 1.0) <= WARMUP_SCALE_MAX:
            c[key] = round(c[key] * ratio, 4)
            if notes is not None:
                notes.append("%s: %s x %.3f from today's warm-up" % (name, key, ratio))
        elif notes is not None:
            notes.append("%s: today's warm-up measured %s x %.3f, more than %.0f %% off: NOT applied (check the "
                         "camera's view of that robot)" % (name, key, ratio, 100 * WARMUP_SCALE_MAX))
    return c


def corrections_cmds(name, day=None, notes=None):
    c = corrections_for(name, day, notes)
    return ["set gyro_scale %.4f" % c["gyro_scale"], "set gyro_drift_dps %.4f" % c["gyro_drift_dps"],
            "set wheel_scale %.4f" % c["wheel_scale"]]


_STATUS = re.compile(r"status (\S+) \(firmware ([^,)]+)(?:, boot ([0-9A-Fa-f]{4}))?\): (\w+).*?battery (-?\d+)%")


def parse_status(text):
    """`M,status Left (firmware 2026-10-28a, boot 4305): idle, selected turn, battery 43%, WiFi -51 dBm` ->
    dict(name, firmware, boot (None before 2026-10-28a), state, battery)."""
    m = _STATUS.search(text)
    if not m:
        return None
    return {"name": m.group(1), "firmware": m.group(2), "boot": (m.group(3) or "").upper() or None,
            "state": m.group(4), "battery": int(m.group(5))}


def boot_of_run(run_id):
    """Run ids are <boot>-<n> (firmware runExperiment): the boot ID from a RUNINFO / STEP run id."""
    if run_id and "-" in run_id:
        b = run_id.split("-", 1)[0]
        if re.fullmatch(r"[0-9A-Fa-f]{4}", b):
            return b.upper()
    return None


def parse_setting(cmd):
    """'set gyro_scale 1.075' -> ('gyro_scale', 1.075); anything else None."""
    p = cmd.split()
    if len(p) == 3 and p[0] == "set":
        try:
            return p[1], float(p[2])
        except ValueError:
            return None
    return None


def setting_matches(sent, got, tol=None):
    """The robot keeps floats; PARAM rows print 4 decimals, `get` 3."""
    if got is None:
        return False
    return abs(float(sent) - float(got)) <= (tol if tol is not None else max(1e-3, 1e-4 * abs(float(sent))))


# ----------------------------------------------------------------------------------------------------------- plan gap

def plan_gap(plan, m=None, step_s=0.05, gate_cm=MIN_GAP_CM):
    """Closest approach of two robots' bodies over a plan, in cm, with the moment and pair. A pair already closer than
    that at the plan's start (robots stopped near each other, a re-plan after a front-sensor stop) is judged only on
    getting closer than where they started: such a plan is refused only if it brings them closer still. Returns
    dict(closest_cm, pair, t, start_cm (for that pair), judged_cm (the value the gate compares))."""
    import numpy as np
    sys.path.insert(0, HERE)
    import fleetmap as FM
    import geometry as G
    import planner as P
    names = list(plan["robots"])
    if len(names) < 2:
        return {"closest_cm": math.inf, "pair": None, "t": None, "start_cm": None, "judged_cm": math.inf}
    body = FM.body_of(FM.normalize(m)) if m is not None else G.Body()
    T = max([r["finish_s"] for r in plan["robots"].values()] + [0.0])
    ts = np.arange(0.0, T + step_s, step_s)
    poses = {n: np.array([P.pose_at(plan, n, t) for t in ts]) for n in names}
    best = {"closest_cm": math.inf, "pair": None, "t": None, "start_cm": None, "judged_cm": math.inf}
    for a, b in itertools.combinations(names, 2):
        g = G.body_gaps(body, poses[a], poses[b])
        g0 = float(g[0])
        k = int(np.argmin(g))
        gm = float(g[k])
        judged = gm if gm < g0 - 0.1 else math.inf      # never closer than where they stand: nothing to refuse
        if g0 >= gate_cm:
            judged = gm
        if gm < best["closest_cm"]:
            best.update(closest_cm=round(gm, 2), pair=[a, b], t=round(float(ts[k]), 2), start_cm=round(g0, 2))
        if judged < best["judged_cm"]:
            best["judged_cm"] = round(judged, 2)
            best["judged_pair"] = [a, b]
            best["judged_t"] = round(float(ts[k]), 2)
    return best


def gate_words(gap, gate_cm):
    if gap["judged_cm"] >= gate_cm:
        return None
    a, b = gap.get("judged_pair") or gap["pair"]
    return ("the plan brings %s and %s %.1f cm apart at %.1f s, under the %.1f cm safety net"
            % (a, b, gap["judged_cm"], gap.get("judged_t") or 0.0, gate_cm))


# ----------------------------------------------------------------------------------------------- heading checks
# Nick (2026-10-27 14:53): "should we incorporate distance into that? Short drives matter less but should be noted that
# they need to confirm position before anything risky". A heading error matters by the sideways drift it causes before
# the next camera correction: allowed = atan(allowed drift / distance to the next fix), less the measurement's own
# uncertainty, atan(camera wobble / length of the line the heading was read from).

NEXT_FIX_CM = 10.0       # chosen: how far a robot drives before the camera bridge has a travel heading for it again
                         # (cam_bridge HEADING_MIN_CM 5 cm inside its window, plus the window's lag). DOE note: Logs:
                         # the distance from a drive's start to its first fix with a heading, ladder L3/L4 drives
OPEN_DRIFT_CM = 3.0      # allowed drift on an ordinary leg: the line check's 3 cm gate. No DOE: a pass mark
RISKY_DRIFT_CM = 1.5     # allowed drift on a risky leg (Chicken, the circle, near an edge or a robot): half the gate.
                         # DOE note: 1 / 1.5 / 2 cm against confirm crawls that fail (time) and Chicken's closest
                         # approach by the camera (board)
WOBBLE_CM = 1.0          # the camera's spot wobble on a still robot (~1 cm, 27 Oct L4). Measured, no DOE
START_LINE_CM = 10.0     # chosen: the line a start heading is read from when the source does not say (cam_bridge's
                         # travel heading over its window). DOE note: Logs: the line lengths behind the start headings
                         # of ladder L3-L5 against their first-leg cross-track error
RISK_ROOM_CM = 6.0       # a leg whose body passes within this of the table's edge or a no-go area is risky (the
                         # lanes' ends and the run-ins' starts). No DOE: geometry (half a body plus the gate)
LONG_LEG_CM = 40.0       # a leg longer than this is risky (the drift grows with it). No DOE: about half the table


def allowed_heading_deg(next_cm, drift_cm, line_cm, wobble_cm=WOBBLE_CM, next_fix_cm=NEXT_FIX_CM):
    """Heading error (deg) a leg can take: atan(drift / distance to the next fix) - atan(wobble / line). May be
    zero or less: then the heading cannot be judged from that line, and the robot confirms it with a crawl first."""
    d = max(0.5, min(float(next_cm) if next_cm else next_fix_cm, next_fix_cm))
    a = math.degrees(math.atan2(float(drift_cm), d))
    u = math.degrees(math.atan2(float(wobble_cm), max(1.0, float(line_cm))))
    return a - u


def leg_risk(step, m, room_cm=None):
    """Why a plan step's leg is risky ([] = not): its label (Chicken, circle, meet), its body near the table's edge or
    a no-go area (room_cm: the body's clearance along the leg, from the planner's world), a long leg."""
    why = []
    lab = (step.get("label") or "").lower()
    for k in ("chicken", "circle", "meet"):
        if k in lab:
            why.append(k)
    if room_cm is not None and room_cm < RISK_ROOM_CM:
        why.append("%.1f cm from an edge or no-go area" % room_cm)
    if step["type"] == "drive" and math.dist(step["from"][:2], step["to"][:2]) > LONG_LEG_CM:
        why.append("a long leg")
    return why


def first_leg(plan, name, m=None):
    """The first moving leg of a robot's plan: dict(step, length_cm, room_cm, risk) or None."""
    sys.path.insert(0, HERE)
    import fleetmap as FM
    steps = plan["robots"][name]["steps"]
    st = next((x for x in steps if x["type"] in ("drive", "arc")), None)
    if st is None:
        return None
    L = math.dist(st["from"][:2], st["to"][:2])
    room = None
    if m is not None and st["type"] == "drive":
        mm = FM.normalize(m)
        body = FM.body_of(mm)
        w = FM.World(mm)
        fr = FM.pose_in(st["from"])
        room = float(w.shape_clearance(body.drive_sweep(fr[:2], FM.pose_in(st["to"])[:2], fr[2])))
    return {"step": st, "length_cm": L, "room_cm": room, "risk": leg_risk(st, m, room)}


def start_heading_check(plan, name, m, err_deg, line_cm=START_LINE_CM):
    """The scaled start check: dict(ok, allowed_deg, words)."""
    fl = first_leg(plan, name, m)
    if fl is None:
        return {"ok": True, "allowed_deg": None, "words": "no drive in its plan"}
    drift = RISKY_DRIFT_CM if fl["risk"] else OPEN_DRIFT_CM
    if fl["room_cm"] is not None:
        drift = min(drift, max(0.5, fl["room_cm"] - 1.0))
    allowed = allowed_heading_deg(fl["length_cm"], drift, line_cm)
    ok = err_deg <= allowed
    words = ("first leg %.0f cm%s: %.1f cm of drift allowed before the next camera heading, so %.1f deg (heading "
             "read off a %.0f cm line)" % (fl["length_cm"], (" (risky: %s)" % ", ".join(fl["risk"])) if fl["risk"]
                                           else "", drift, allowed, line_cm))
    return {"ok": ok, "allowed_deg": round(allowed, 2), "words": words, "risk": fl["risk"]}
