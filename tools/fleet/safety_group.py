#!/usr/bin/env python3
"""The Safety group (e18, Nick 12 Nov, decision 2): every safety switch of the fleet in ONE place.

    python3 tools/fleet/safety_group.py show        # each switch, what it does, its value now and its old way
    python3 tools/fleet/safety_group.py doc         # the same as a Markdown table (docs/how-to/safety-group.md)
    python3 tools/fleet/safety_group.py env NAME=0  # the export line for one switch (the old way back)
    python3 tools/fleet/safety_group.py exports     # the switches set in this shell, K=V (run-fleet-pi.sh's tmux)

Every switch is decided automatically by default (the value in DEFAULT below) and lives out of a new user's way:
nothing has to be set for a normal run. Each one keeps its old behaviour behind it (CLAUDE.md "keep features, switch
them off"), reached in one of two ways, the first that is set wins:
  1. the environment variable for one run (on the Pi: `MIRLT_NEVER_STILL=0 ./run-fleet-pi.sh freeroam`);
  2. Mission Control's Run screen > More > Safety box, saved into the Pi's ~/.config/mirlt/safety.json (MIRLT_CONF_DIR):
     every run on that Pi then uses it (free roam, missions, calibration) until it is set back to "auto".

The code reads each switch through flag() / value() here, so the default lives in this one table: a test checks that
every MIRLT_* switch the safety code reads is listed here (tools/fleet/tests/test_e18_safety_blind.py).

Changelog
  2026-11-12e18  first version: the group, the e18 switches (out-and-back identity drive, never still in missions and
                 calibration, a doubtful heading beside a known / unknown neighbour, lost robots first, the blind-drive
                 allowance, calibration saved at every run end) and the earlier safety switches gathered from
                 freeroam_core, safety_rules, tracking_layer, run_plan, calib_run and mission_run.
  2026-11-12e19  the Fable audit (e19): MIRLT_HANG_RECOVERY (H6) and the e19 switches listed with "e19" in their row.
"""
import os
import sys

VERSION = "safety-group-2026-11-12e19"   # e19: the Fable audit's switches (see the changelog). Before: "safety-group-2026-11-12e18b"   # e18b: five switches from the replay (ELIM_PAIR_GUARD, FAR_NAMED_CORRECT, RESEED_HEADING_KEEP, TRUST_FLEET, CURVE_SCALE_SAVE)

# Each row: env name, default (automatic), the old way's value, the areas it touches, what it does (plain words),
# the build that added it. "areas": F free roam, M missions (run_plan, School Visit tours), C calibration, T tracking
# layer (every run type that names camera sightings).
SWITCHES = [
    # -- never still and never give up -----------------------------------------------------------------------------
    ("MIRLT_NEVER_STILL", "1", "0", "F M C",
     "Held, unsure, lost or needs-a-hand robots keep making small slow moves toward safety and being found (step and "
     "speed scaled by their doubt); 0: the e16 hold-and-wait everywhere", "e17 (free roam), e18 (missions, calibration)"),
    ("MIRLT_HELD_NEVER_END", "1", "0", "M",
     "Every robot held never ends a mission by itself; 0: the 90 s wait and the keep-going cap", "e16"),
    ("MIRLT_EDGE_PARK", "0", "1", "F",
     "1: a robot held at an edge for 60 s is parked for the rest of the run (e12); 0: it stays in and rejoins", "e13"),
    ("MIRLT_PARK_GIVEUP", "0", "1", "C",
     "1: parking after a calibration gives up on a lost tag (e13); 0: never gives up", "e14"),
    ("MIRLT_CALIB_GIVEUP", "0", "1", "C",
     "1: a calibration whose roof tag is lost ends after 30 s (e12); 0: held, never gives up", "e13"),
    ("MIRLT_HANG_RECOVERY", "grow", "e08", "F",
     "A robot hung at an edge is never given up on: it first tries the opposite of what tilted it (backed over: forward "
     "in; turned: the other way), forward tries whenever its nose points inward (the tilt stop raised for that step), and "
     "each new round of tries is longer and wider, one derived pause after the last; a failed try is remembered only "
     "within its round. e08: the fixed reverse / pivot tries, a 20 s hold, failures remembered for ever", "e19 (audit H6)"),
    ("MIRLT_ASK_UNATTENDED", "0", "1", "F",
     "1: keeps asking a person when nobody answers; 0: decides by itself after 30 s", "e16"),
    # -- telling robots apart, trusting the camera -------------------------------------------------------------------
    ("MIRLT_ELIMINATE", "1", "0", "T",
     "A steady sighting nobody else can claim names the one unaccounted robot (on probation)", "e17"),
    ("MIRLT_OUTBACK_ID", "1", "0", "T F",
     "Two or more robots unaccounted and the wiggle cannot tell them apart: one at a time drives a short slow straight "
     "out-and-back (2-4 cm, from the camera's measured noise) so the camera sees which sighting moved", "e18"),
    ("MIRLT_OUTBACK_SOUND", "1", "0", "T F",
     "(audit H3) The out-and-back is as long as the camera's measured noise needs (it may pass 4 cm; up to 12 cm), "
     "refused where there is no room for that length (never shortened below it), and 'moved' is judged on the median of "
     "the sightings at the time the robot's own wheels say it was out (a robot whose own track did not move names "
     "nobody). 0: e18 (2-4 cm, the 3 farthest sightings)", "e19"),
    ("MIRLT_OUTBACK_INWARD", "1", "0", "F",
     "(Nick 12 Nov) A robot near or half over an edge gets an inward-only out-and-back (forward or back, whichever takes "
     "every place the camera might have it further in). 0: e18 (refused near an edge)", "e19"),
    ("MIRLT_ID_PAUSE", "1", "0", "F",
     "(Nick asked 'why unnamed, fix it'; his answer pending: on by default, flip it here) With two or more robots "
     "unaccounted, an unnamed robot pauses its never-still creeps for a few seconds (the camera's steady time plus one "
     "identity move) ONLY while it is being identified, then never still resumes. 0: e18 (creeps go on)", "e19"),
    ("MIRLT_ELIM_PAIR_GUARD", "1", "0", "T",
     "The elimination counts robots in a close pass as unaccounted too, and never gives a spot that lies on another "
     "robot's own pose (within the 6 cm fix gate) to an unaccounted robot (5 Nov 14:55: Right corrected onto Left's spot "
     "56 cm away)", "e18 (replay of all home runs)"),
    ("MIRLT_FAR_NAMED_CORRECT", "1", "0", "F",
     "A standing robot the layer keeps naming whose own pose is beyond its firmware's fix gate (6 cm) for a second, the "
     "offset steady and no other robot on that spot: its pose is set from the camera (a correction, as after an "
     "elimination; 11 Nov 08:26: Center held 86 s at 31 cm, then a false REJOINED)", "e18 (replay)"),
    ("MIRLT_RESEED_HEADING_KEEP", "1", "0", "F",
     "A re-seed to a standing robot keeps its own heading unless the camera's heading is better evidence (a roof tag's "
     "1-2 deg against its own doubt, grown 1.6 deg per 90 deg turned since its heading was last set; the replay: standing "
     "re-seeds jumped the heading 16-64 deg, worse in 4 of 6 checked)", "e18 (replay)"),
    ("MIRLT_TRUST_FLEET", "1", "0", "F M",
     "The blind-drive trust model is fitted to the whole fleet's pooled samples, updated live each run; a robot gets its "
     "own record only once its own samples differ (Mann-Whitney p < 0.01); curve samples count only in their own run (the "
     "replay: drift 0.06 cm/cm for all three robots, stable between runs; curve response moves by session). 0: each "
     "robot's own record, every run's curves", "e18 (replay)"),
    ("MIRLT_CURVE_SCALE_SAVE", "0", "1", "F",
     "Curve scales (arc_k_scale, arc_d_scale) measured in a run are logged, NOT saved on the robot, until Nick decides "
     "(the replay: the curve response moves by session for all robots together; Left's 1.0736 came from one session). 1: "
     "saved by the agreement rule (e17)", "e18 (replay; Nick to decide)"),
    ("MIRLT_IDENTITY", "wiggle", "predict", "T",
     "Close passes resolved by a wiggle (predict: by the prediction alone, before e09)", "e09"),
    ("MIRLT_GATE_RECORD", "1", "0", "T", "Each robot's gate grows with its own measured drift record", "e17"),
    ("MIRLT_OWN_RESET_RULE", "velocity", "speed", "T",
     "An own-pose re-set is found by its own velocity at any picture rate", "e17"),
    ("MIRLT_CAMERA_CORRECT", "1", "0", "F",
     "A sighting named by elimination or a wiggle / out-and-back is a CORRECTION (pose set at once)", "e17"),
    ("MIRLT_RESEED_TURN", "1", "0", "F", "A re-seed is projected to now and never lands mid-turn", "e17"),
    ("MIRLT_REJOIN_EVIDENCE", "1", "0", "F", "'Moved in' (REJOINED) only on evidence", "e16"),
    ("MIRLT_CAMERA_WATCH", "1", "0", "T", "The camera's turn directions checked against the robots' own", "e10"),
    ("MIRLT_WHOLE_CAMERA_WATCH", "1", "0", "T",
     "(audit finding 10) When most of the camera's sightings fall outside EVERY robot's gate in each picture for a few "
     "seconds (a wrong camera name, a bumped camera), the camera as a whole is not trusted: no fixes or eliminations from "
     "it, said on screen; trusted again when its sightings agree again. 0: e18 (each robot alone says 'camera "
     "disagrees')", "e19"),
    ("MIRLT_POSE_RULE", "widen", "wait2s", "F M", "Own pose vs camera disagreement widens the doubt at once", "e08"),
    # -- edges ---------------------------------------------------------------------------------------------------------
    ("MIRLT_EDGE_RULE", "derived", "fixed", "F M C",
     "One derived edge clearance (body + doubt + stop + camera bias) on the whole swept path", "e08"),
    ("MIRLT_CAM_EDGE_BIAS", "on", "off", "F M", "The lights camera's measured 5 cm edge bias added", "e08"),
    ("MIRLT_EDGE_BACKOUT", "1", "0", "F M", "An edge stop reverses along the robot's own recorded track", "e16"),
    ("MIRLT_START_EDGE", "1", "0", "F", "A robot put down at or over an edge is checked before it starts", "e10"),
    ("MIRLT_START_EDGE_MODE", "recover", "wait", "F", "... and frees itself first (wait: always waits for a person)",
     "e10"),
    ("MIRLT_MARK_UNC", "1", "0", "F", "A line-sensor trip near an edge (with its doubt) is an edge, not a mark", "e10"),
    ("MIRLT_QUEUE_WATCH", "1", "0", "F", "Queued curve pieces re-checked from where the robot really is", "e10"),
    ("MIRLT_PINNED_OBSTACLE", "1", "0", "F", "A robot held at an edge is a fixed obstacle with its doubt", "e16"),
    ("MIRLT_CONTACT_WATCH", "1", "0", "F", "Pushes and collisions noticed (camera and accelerometer)", "e16"),
    ("MIRLT_TILT_RULE", "logs", "old", "F M", "Tilt stop 4 deg (from the 5-6 Nov logs); old: 5 deg", "e08"),
    ("MIRLT_KEEPOUT_RULE", "fade", "permanent", "F M", "Sensor-only keep-outs fade after 15 s", "e08"),
    # -- the camera lost, blind driving, watchdogs ------------------------------------------------------------------
    ("MIRLT_BLIND_RULE", "edge", "cap", "F M C",
     "A robot with no camera drives on while its whole possible error cannot reach an edge (cap: the 8 / 40 cm caps)",
     "e13"),
    ("MIRLT_BLIND_ALLOW", "1", "0", "F M",
     "Each robot's own measured odometry record (cm per cm, deg per turn and per curve) predicts its error on a blind "
     "leg; a blind excursion is allowed only where that prediction keeps the margin from edges and robots with a way "
     "back; checked against the error found when it is seen again. 0: the fixed 29 Oct numbers (2 cm + 0.07 cm per cm) "
     "and no planned blind legs", "e18"),
    ("MIRLT_BLIND_BUBBLE", "record", "fixed", "F M",
     "(e19, the audit's finding 13a: lost robots used 2 + 0.07 cm per cm while the record said otherwise) Every loss of "
     "the camera, planned or not, grows the robot's doubt by the SAME model: its fleet record, started from calibration-"
     "run data (MIRLT_TRUST_PRIOR) and widened by its own checks. fixed: an unplanned loss by the fixed 29 Oct numbers "
     "(e18; with e18's cautious 0.28 cm/cm prior the record made robots with 1 s late fixes stand up to 20 min)",
     "e18, default changed e19"),
    ("MIRLT_TRUST_PRIOR", "calibration", "replay", "F M",
     "(Nick 12 Nov) The odometry trust record starts from calibration-run data: dead reckoning on the calibrated scales "
     "against both cameras, 22,000 stretches (29 Oct): 2.0 cm + 0.07 cm per cm at the 95th percentile. replay: e18b's "
     "cautious 2.3 + 0.28 (free-roam stretches, mostly curves)", "e19"),
    ("MIRLT_MOTION_PROOF", "1", "0", "F T",
     "(audit H4) A camera correction that would move a robot's pose further than its own stated doubt (its 95 % doubt "
     "and the camera's measured noise together) is applied only with motion proof: a wiggle or out-and-back the camera "
     "saw; an elimination or a 'named far' beyond it asks for that drive first. Until a correction is confirmed by the "
     "robot's own move it permits no reverse toward an edge and does not widen the camera's gate. 0: e17/e18 (applied at "
     "once, any distance)", "e19"),
    ("MIRLT_NOPIC_RULE", "risk", "off", "F M", "No pictures: each robot's risk-based limit", "e13"),
    ("MIRLT_FIX_GAP_RULE", "derived", "fixed", "F M", "How old a fix may be, from each robot's measured fix gap", "e14"),
    ("MIRLT_STAND_TRUST", "1", "0", "F", "A standing robot keeps its fix (not lost while it stands)", "e13"),
    ("MIRLT_STAND_UNC", "1", "0", "F", "... and its doubt does not grow while it stands", "e16"),
    ("MIRLT_LATE_FIX", "1", "0", "F", "Late fixes slow a drive to what its measured drift allows", "e17"),
    ("MIRLT_STRAY_RETURN", "1", "0", "F", "A robot outside the readable area comes back first", "e13"),
    ("MIRLT_FRAME_WATCH", "1", "0", "F M C", "The bridge says when the camera stops sending pictures (watchdog)",
     "e13"),
    # -- headings --------------------------------------------------------------------------------------------------------
    ("MIRLT_HEADING_CONFIRM", "1", "0", "F", "After a heading re-set, a short crawl the camera measures", "e17"),
    ("MIRLT_TAGFREE_HAND", "risk", "e18", "F T",
     "(audit finding 9, Nick) Tag-free hands a detector heading on only when the disagreement is more than the detector's "
     "measured heading error (95 %: 26 deg) and the robot's own heading doubt explain; the limit for handing on (was a "
     "fixed 45 deg) comes from the robot's room to an edge or a neighbour; past it, the camera-measured heading check is "
     "asked. e18: 2 sigmas of the averaged offset, 45 deg", "e19"),
    ("MIRLT_HEADING_CLICKS", "1", "0", "F", "After a click, a straight drive and a second click give the heading",
     "e13"),
    ("MIRLT_DOUBT_NEIGHBOUR", "1", "0", "F",
     "A robot whose heading is in doubt beside another: the neighbour camera-confirmed -> wait for room (or creep "
     "straight away when even its worst heading cannot close the gap); not confirmed -> creep cautiously (half steps, "
     "slowest) away from where the neighbour was last believed to be. 0: e17 (creeps keep the reflex gap on the "
     "heading it has)", "e18"),
    # -- priority ----------------------------------------------------------------------------------------------------------
    ("MIRLT_LOST_PRIORITY", "1", "0", "F M",
     "A lost robot (or one with no readable point) goes first to the nearest readable point nobody else uses; the "
     "others yield (free roam: it outranks them in the reflex and standing robots in its way make room; missions: "
     "right of way in the plan, a robot standing in its recovery path steps aside)", "e18"),
    ("MIRLT_EMPTY_TO_READABLE", "1", "0", "M", "A robot with no readable points goes to the nearest one nobody has",
     "e17"),
    # -- calibration kept --------------------------------------------------------------------------------------------------
    ("MIRLT_ARC_SAVE_AT_END", "1", "0", "F", "The curves' calibration saved at any run end (agreement rule)", "e17"),
    ("MIRLT_SAVE_CAL_AT_END", "1", "0", "F M",
     "Every calibration correction a run measured (curves, straight drives, turns: each by the agreement rule; each "
     "robot's odometry record) saved at every ending: the time cap, Stop / Ctrl+C, an error (a checkpoint every minute "
     "covers a crash or power cut). 0: only the curves at a normal end (e17)", "e18"),
]

DEFAULT = {s[0]: s[1] for s in SWITCHES}
OLD = {s[0]: s[2] for s in SWITCHES}
_OFF = ("0", "off", "no", "false")
_FILE = None


def file_path():
    d = os.path.expanduser(os.environ.get("MIRLT_CONF_DIR") or "~/.config/mirlt")
    return os.path.join(d, "safety.json")


def _file():
    """The Pi's safety.json (Mission Control's Safety box), read once; {} when there is none."""
    global _FILE
    if _FILE is None:
        _FILE = {}
        try:
            import json
            with open(file_path()) as f:
                d = json.load(f)
            _FILE = {k: str(v) for k, v in (d.get("switches") or {}).items() if k in DEFAULT and v not in (None, "",
                                                                                                         "auto")}
        except (OSError, ValueError, AttributeError):
            pass
    return _FILE


def write_file(settings, path=None):
    """Mission Control's Safety box -> safety.json (only the switches not on "auto"). Returns the path or None."""
    import json
    p = path or file_path()
    if not env_from_settings(settings) and not os.path.exists(p):
        return None                            # every switch on auto and no file yet: nothing to keep
    try:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        tmp = p + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"version": VERSION, "switches": env_from_settings(settings)}, f, indent=1)
        os.replace(tmp, p)
        global _FILE
        _FILE = None
        return p
    except OSError:
        return None


def value(env, default=None):
    """The switch's value now: the environment's, else the Pi's safety.json, else the group's automatic default."""
    d = DEFAULT.get(env, default)
    v = os.environ.get(env)
    if v is None or v == "":
        v = _file().get(env)
    return d if v is None or v == "" else v


def flag(env, default=None):
    """An on / off switch: False for 0 / off / no / false, True otherwise (the group's default when unset)."""
    v = value(env, default)
    return str(v).strip().lower() not in _OFF


def changed():
    """[(env, value, default)] for every switch set away from its default now (said at each run's start)."""
    out = []
    for env, d, _o, _a, _w, _b in SWITCHES:
        v = value(env)
        if str(v) != str(d):
            out.append((env, v, d))
    return out


def start_words():
    """One line for a run's start: the group's version and any switch set away from its automatic default."""
    ch = changed()
    if not ch:
        return "Safety group: every switch on its automatic default (%d switches; tools/fleet/safety_group.py show)" % \
            len(SWITCHES)
    return "Safety group: %d of %d switches set by hand: %s" % (len(ch), len(SWITCHES), ", ".join(
        "%s=%s (default %s)" % c for c in ch))


def snapshot():
    """{env: value} for a run's meta (run.json / summary)."""
    return {env: value(env) for env, *_ in SWITCHES}


def env_from_settings(d):
    """Mission Control's Settings > Safety box -> environment for the run it starts: "auto" (or empty) = the default;
    anything else is passed as it is. Unknown names are ignored (said by the caller)."""
    out = {}
    for k, v in (d or {}).items():
        if k in DEFAULT and v not in (None, "", "auto"):
            out[k] = str(v)
    return out


def settings_defaults():
    """Mission Control's settings group: every switch "auto"."""
    return {env: "auto" for env, *_ in SWITCHES}


def doc():
    rows = ["| Switch | Automatic default | Old way | Used in | What it does | Since |", "|---|---|---|---|---|---|"]
    names = {"F": "free roam", "M": "missions", "C": "calibration", "T": "tracking layer"}
    for env, d, o, a, w, b in SWITCHES:
        rows.append("| `%s` | %s | %s | %s | %s | %s |" % (env, d, o, ", ".join(names[x] for x in a.split()),
                                                            w.replace("|", "/"), b))
    return "\n".join(rows)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    cmd = argv[0] if argv else "show"
    if cmd == "doc":
        print(doc())
        return 0
    if cmd == "exports":
        # (for run-fleet-pi.sh) the Safety switches set in THIS shell, as K=V words for a tmux command line (a tmux
        # session does not inherit the caller's environment once its server runs)
        print(" ".join("%s=%s" % (k, os.environ[k]) for k in DEFAULT if os.environ.get(k) not in (None, "")
                       and all(c.isalnum() or c in "._-" for c in os.environ[k])))
        return 0
    if cmd == "env":
        for kv in argv[1:]:
            k, _, v = kv.partition("=")
            if k not in DEFAULT:
                print("# %s is not in the Safety group" % k)
                continue
            print("export %s=%s" % (k, v or OLD[k]))
        return 0
    print(VERSION)
    for env, d, o, a, w, b in SWITCHES:
        v = value(env)
        print("%-24s %-9s (default %s, old way %s)  [%s]  %s" % (env, v, d, o, a, w))
    print(start_words())
    return 0


if __name__ == "__main__":
    sys.exit(main())
