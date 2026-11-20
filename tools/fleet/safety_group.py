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
  2026-11-12e19  the Fable audit (e19): MIRLT_HANG_RECOVERY (H6) and the e19 switches listed with "e19" in their row;
                 (audit 8) each switch takes only its two choices or auto (normalize; a typed "Auto" no longer turns a
                 switch on), refusals said at the start; missions, calibration and the frame watch read through here.
  2026-11-12e20  learned camera trust (e20): MIRLT_CAMERA_TRUST, MIRLT_ODO_LEARN_TRUST, MIRLT_DETECTOR_ROUTE,
                 MIRLT_MOTION_HEADING; MIRLT_PAIR_STAND_PART (a close pair standing apart is parted).
  2026-11-12e21  one error shape (e21): MIRLT_ERROR_SHAPE, MIRLT_STAGES, MIRLT_LOST_CREEP (the creep with no edge
                 rule, now off), MIRLT_SHAPE_PLAN, MIRLT_CONSISTENT_ESCAPE, MIRLT_INWARD_TWO_HEADINGS,
                 MIRLT_CAMERA_VERDICT (per camera), MIRLT_BRIDGE_PER_CAMERA, MIRLT_FW_FILTER_PARAMS.
  2026-11-13e21b MIRLT_CAMERA_VERDICT's automatic default is "whole" (e19's one verdict): in the 25-run comparison the
                 per-camera verdict let robots drive on a faulty camera's names (8 falls against 3, harsh and wrong-name
                 runs); per_camera stays one switch away. MIRLT_PAIR_STAND_PART and MIRLT_UNCONF_REVERSE stay on (their
                 "off" runs: more falls and stands; e21 report section 9).
  2026-11-16ko   MIRLT_KEEPOUT_ROOM: the paths keep the edge rule's room (track error + stopping distance) from a site's
                 keep-outs, not the body and 1.5 cm only (school Visit points ran into the outlet plates, 16 Nov).
  2026-11-19e23  the fixes for Jay-Sun's school runs of 16 / 18 Nov: MIRLT_STOP_UNNAMED, MIRLT_CAMERA_PROBE,
                 MIRLT_MOVE_CHECK, MIRLT_IMPROVE_SKIP_OVER, MIRLT_PLAN_AHEAD, MIRLT_POINT_CREDIT, MIRLT_OLD_FIX,
                 MIRLT_EDGE_SLOW_END, MIRLT_BIND_CAMS, MIRLT_WAKE_WAIT, MIRLT_VISIT_UNSEEN.
  2026-11-19e23b MIRLT_VEER_AT_POINT (found while finishing e23): a robot standing in its dwell at a point is not sent
                 on the veer stage's early turn-back when the camera loses it.
  2026-11-19e23c the audit of e23: MIRLT_STOP_NO_ANSWER, MIRLT_OLD_FIX_JUMPS, MIRLT_REACH_TOL, MIRLT_WAKE_PARTIAL;
                 MIRLT_WAKE_WAIT's and MIRLT_STOP_UNNAMED's words corrected to what the code does.
"""
import os
import sys

VERSION = "safety-group-2026-11-19e23c"   # e23c: four switches from the audit of e23, two descriptions corrected. Before: "safety-group-2026-11-19e23b"   # e23b: MIRLT_VEER_AT_POINT. Before: "safety-group-2026-11-19e23"   # e23: eleven switches for the school fixes of 19 Nov (see the changelog). Before: "safety-group-2026-11-16ko"   # 2026-11-16ko: MIRLT_KEEPOUT_ROOM (the room the paths keep from a site's keep-outs). Before: "safety-group-2026-11-13e22c"   # e22c: MIRLT_BOX_ASK (off by default). Before: "safety-group-2026-11-13e22b"   # e22b: MIRLT_RESCUE_STILL. Before: "safety-group-2026-11-13e22"   # e22: MIRLT_SELF_RESCUE, MIRLT_ID_GUARD, MIRLT_FW_ESCAPE, MIRLT_PLAN_BUDGET (see the changelog). Before: "safety-group-2026-11-13e21b"   # e21b: the camera verdict whole by default (see the changelog). Before: "safety-group-2026-11-12e21"   # e21: one error shape and Nick's four stages (see the changelog). Before: "safety-group-2026-11-12e20"   # e20: MIRLT_CAMERA_TRUST, MIRLT_ODO_LEARN_TRUST, MIRLT_DETECTOR_ROUTE, MIRLT_MOTION_HEADING, MIRLT_PAIR_STAND_PART. Before: "safety-group-2026-11-12e19"   # e19: the Fable audit's switches (see the changelog). Before: "safety-group-2026-11-12e18b"   # e18b: five switches from the replay (ELIM_PAIR_GUARD, FAR_NAMED_CORRECT, RESEED_HEADING_KEEP, TRUST_FLEET, CURVE_SCALE_SAVE)

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
    ("MIRLT_CAMERA_TRUST", "learn", "e19", "T F",
     "(Nick 12 Nov: any camera at any angle, the system works out which is best for what) Each camera earns a trust "
     "score per job (position, heading, telling robots apart), per board square and per lighting, from checks against "
     "the robots' own wheels and gyro, other cameras, roof tags and heading from motion; a camera is refused where its own "
     "evidence says it is wrong, and can earn trust back; kept per site. The whole-camera watch's verdict still stands for a "
     "camera-wide fault (and counts as evidence), but a wrong-way sighting lying on another robot no longer counts toward it (its false alarm). "
     "e19: the whole-camera watch alone", "e20"),
    ("MIRLT_ODO_LEARN_TRUST", "trust", "e19", "F",
     "(Nick 12 Nov) A robot's odometry record learns only from cameras trusted for position where the check was made, "
     "each check weighted by that trust. e19: every accepted fix teaches it", "e20"),
    ("MIRLT_DETECTOR_ROUTE", "trust", "e19", "F T",
     "Each on-demand detector call runs the camera most likely to answer that question (position, heading, telling robots "
     "apart) for that robot's square, now and then a less-trusted one (Thompson sampling) so the scores stay current. e19: "
     "the first camera, the second only where the first found nothing", "e20"),
    ("MIRLT_MOTION_HEADING", "1", "0", "F",
     "Heading from motion on every camera: while a robot drives straight (its command gives front and back), each camera's "
     "line of sightings gives its heading, with a precision from that camera's measured noise across the line and the "
     "distance driven; it is a heading source, evidence for the cameras' trust, and it settles a heading in doubt without "
     "a crawl when it is precise enough. 0: e19 (the crawl)", "e20"),
    ("MIRLT_CAMERA_VERDICT", "whole", "per_camera", "T F",
     "A camera found wrong: e19's one verdict refuses every camera's sightings until the evidence clears (the automatic "
     "default since e21's 25-run comparison: fewer falls). per_camera: (e20's open problem) only the camera(s) the "
     "evidence implicates are refused, and a trusted camera's agreement releases it; shorter stands, but robots drove "
     "on a faulty camera's names (more falls in the hard simulator)", "e21"),
    ("MIRLT_BRIDGE_PER_CAMERA", "1", "0", "T",
     "The camera bridge sends each camera's sighting of a robot on its own (its camera named), so every camera earns "
     "its own trust; 0: one merged 'both' sighting when both cameras see it", "e21"),
    ("MIRLT_PAIR_STAND_PART", "1", "0", "T",
     "Two robots that passed close and then BOTH stand still beyond the close-pass distance count as parted, so who is "
     "who is checked (kept, swapped or a wiggle) instead of neither taking a camera fix until one moves 22 cm away (hard "
     "simulator: two lost robots stood 17 cm apart for 105 s). 0: e19 (parted only past 22 cm)", "e20"),
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
    ("MIRLT_ERROR_SHAPE", "cone", "circle", "F M",
     "(Nick 12 Nov) ONE error shape per robot (tools/fleet/error_shape.py): a cone along its own track (its measured "
     "along and sideways growth, from its own record and the fleet's) plus a disc for the error already seen when it was "
     "lost and the events it reports since (bumps, slips, stuck wheels, re-seeds, merges, restarts, each by its measured "
     "jump); it shrinks only with an accepted fix. Plans, the edge rule, the pause, corrections and Mission Control read "
     "it. circle: e20's lost bubble (2 cm + its record per cm, a circle) and every user's own radius", "e21"),
    ("MIRLT_STAGES", "four", "e20", "F M",
     "(Nick 12 Nov) Nick's four stages: run; veer (when the shape would reach an edge within the next moves, moves are "
     "scored by the room they gain and how well the cameras read the ground ahead); found (the shape shrinks); pause and "
     "search only when NO move gains room (it turns on the spot, the cameras and detector look at its shape, nearby "
     "robots hold still only if they must, then a person is asked). e20: moves refused at the edge, a pause by the "
     "bubble where it stands", "e21"),
    ("MIRLT_LOST_CREEP", "0", "1", "F",
     "1: e17's never-still creep at the blind limit, which ignored the edge rule; 0: no creep without the edge rule "
     "(the pause and search instead)", "e21"),
    ("MIRLT_SHAPE_PLAN", "ellipse", "circle", "F",
     "Reservations carry each robot's shape (a disc and an ellipse); two shapes are kept apart along the line between "
     "them, planned ahead. circle: each shape as the circle of its largest reach", "e21"),
    ("MIRLT_CONSISTENT_ESCAPE", "1", "0", "F",
     "(the ratchet) When the camera's sightings of a robot agree with each other but not with its own pose, for more "
     "sightings than the camera's measured noise can explain, its own pose is wrong: the camera's pose is sent (fpose) "
     "and its heading is taken from its next short move as the camera sees it (a 180 deg flip is caught). 0: e20", "e21"),
    ("MIRLT_INWARD_TWO_HEADINGS", "1", "0", "F",
     "The 'inward by both' check uses each position's own heading (its own heading for its own pose, the camera's "
     "measured heading for the camera's spot); 0: one heading for both (e08)", "e21"),
    ("MIRLT_UNCONF_REVERSE", "1", "0", "F",
     "(e19, audit H4) No reverse toward an edge while the robot's pose is unconfirmed (no rear sensor). 0: the error "
     "shape's edge rule alone carries a reverse's whole margin (its sensors cannot see an edge behind it). Nick 12 Nov: "
     "off only if the long checks show no difference", "e19, switch e21"),
    ("MIRLT_SLIP_TAG", "1", "0", "F T",
     "(Nick 12 Nov) A move the robot's own wheels, accelerometer, line sensors and the camera say did not move is a slip: "
     "it does not count against the camera's trust in that square, its own pose is corrected from the camera at once, "
     "and a square that keeps causing slips is logged as a surface feature. 0: as e20", "e21"),
    ("MIRLT_SELF_RESCUE", "ladder", "ask", "F",
     "(Nick 13 Nov: robots never pause for a person) A robot that stands longer than the camera needs to answer its last "
     "move (2 fix delays + the time to be called lost, measured) takes the next rung of a never-ending ladder: its "
     "heading from a short out-and-back the camera sees (or a turn on the spot); a straight move that gains room for its "
     "error shape; else one that keeps its room toward the ground where the cameras find robots best (learned in the "
     "run); else a turn on the spot; again after the dwell. Questions, NEEDS A HAND and click requests become notes in "
     "the log. ask: e13-e21 (held while a person is asked)", "e22"),
    ("MIRLT_KEEPOUT_ROOM", "edge", "body", "M",
     "(Nick 16 Nov: school Visit points ran into the outlet plates) The paths keep the edge rule's room from a site's "
     "keep-outs (outlet plates, the middle box, reference tiles): the robot's body + 1.5 cm, plus its measured track "
     "error and stopping distance (school: 4.1 cm more). body: the body + 1.5 cm only, as before", "2026-11-16"),
    ("MIRLT_BOX_ASK", "0", "1", "F",
     "(Nick 13 Nov ~10:00: two standing robots boxed the third into a corner) A robot with no move that gains room asks "
     "every robot in its way out (the straight way with the fewest robots in it) to step aside, nearest first, held or "
     "not, again at each ladder rung until the way is open; a held robot makes room and takes the ladder too. OFF by "
     "default in e22: in the 25 hard-simulator runs it brought body contact into 2 runs (a lost robot making room, a held "
     "robot's rung) and 5 falls against 1 without it; on only to try Nick's corner case. 0: only a lost robot asks, and "
     "only the nearest robot on its line", "e22"),
    ("MIRLT_RESCUE_STILL", "pose", "idle", "F",
     "(e22b) The ladder's clock restarts only when the robot's own pose really moved (3 cm, or a turn of half the "
     "wiggle), so moves planned and then refused every second no longer keep it standing for minutes. idle: e22's first "
     "rule (the clock restarted by any planned move)", "e22"),
    ("MIRLT_ID_GUARD", "1", "0", "F",
     "(e21's falls: a sighting of another robot or a mirrored camera taken as its own) A correction, re-seed, 'seen "
     "again' or escape beyond the camera's noise only from a sighting that is the robot's: inside its own error shape "
     "and no other robot's (shapes never overlap), its joined-up sightings moving as its wheels did (or standing as it "
     "stood), and not elsewhere than its last fix when it has not moved since. 0: e21", "e22"),
    ("MIRLT_FW_ESCAPE", "pi", "robot", "F",
     "(Nick 13 Nov) The robots' own consistency escape (firmware 2026-11-12e21's fix_escape_n) is set to 0: the Pi's "
     "escape, which checks who the camera saw, is the only one. robot: the Pi sends the derived count (e21)", "e22"),
    ("MIRLT_PLAN_BUDGET", "derived", "every_tick", "F",
     "(Pi speed) Planning is paced by its own measured cost: in each 50 ms tick steering (the reflex, adjustments, the "
     "queue watch, heartbeats) always runs; a robot's plan runs if it is the tick's first or the time used plus "
     "planning's measured 95th percentile stays inside the loop less steering's slowest 1 in 100 ticks, else the next "
     "tick (it goes first then). Far-apart reservations are not compared sample by sample (a box test), and a paused "
     "robot's 'no move gains room' is kept until its pose, shape or the others change. every_tick: e21 (every free "
     "robot plans in the same tick, every check run in full)", "e22"),
    ("MIRLT_FW_FILTER_PARAMS", "derived", "off", "F",
     "Robots on firmware 2026-11-12e21 or newer get their filter's settings from the measured record at the start of a "
     "run (variance growth, fix age limit from the measured delay, the consistency escape); off: their own defaults",
     "e21"),
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
    ("MIRLT_WHEEL_OWNER", "pool", "e18", "F",
     "(audit finding 7) ONE owner of wheel_scale / gyro_scale: the robot's pool on the Pi, in absolute values (each run's "
     "fit kept with the value in use when it was measured); a run's end adds its measurement to the pool; the start saves "
     "the pooled value on the robot only when it is sure and differs from the value in use by more than its own 95 % error "
     "bar. e18: the start's pool and the end's saver both wrote it", "e19"),
    ("MIRLT_CURVE_LIVE", "1", "0", "F",
     "(Nick 12 Nov: the best, most accurate numbers) The curve scales in use are this session's fleet-wide estimate, set "
     "live on every robot (never saved); a robot gets its own only when its pieces differ from the rest's (Mann-Whitney, "
     "p < 0.01). 0: the scales saved on each robot", "e19"),
    ("MIRLT_CURVE_FLASH_RESET", "1", "0", "F",
     "(Nick 12 Nov: Left's arc_k_scale 1.0736 back to 1.0) A curve scale saved on a robot is reset to 1.0 at a real run's "
     "start (it came from one session); the live fleet estimate replaces it. 0: kept", "e19"),
    ("MIRLT_SAVE_CAL_AT_END", "1", "0", "F M",
     "Every calibration correction a run measured (curves, straight drives, turns: each by the agreement rule; each "
     "robot's odometry record) saved at every ending: the time cap, Stop / Ctrl+C, an error (a checkpoint every minute "
     "covers a crash or power cut). 0: only the curves at a normal end (e17)", "e18"),
    # -- e23: Jay-Sun's school runs of 16 and 18 Nov -----------------------------------------------------------------------
    ("MIRLT_STOP_UNNAMED", "once", "wait", "F M C",
     "A Stop pressed before the robots are named: a robot known only by its slot name (no address, no WiFi name) is "
     "settled by a `stop` that went out with the robots' log port read, and a new Start ends the watch. wait: "
     "asked for by its slot name every 3 s for ever (school 16 / 18 Nov: it stopped the next three starts). wait is "
     "not the whole way before e23: a new Start still ends the watch and a second Stop still joins the running "
     "watch instead of starting another (those two have no switch)", "e23"),
    ("MIRLT_STOP_NO_ANSWER", "resend", "first", "F M C",
     "A Stop before the robots are named that NO robot answers: the `stop` is sent 3 times, 3 s apart, and the page "
     "then says it was sent and no robot answered (never 'every robot confirmed'). first: settled after the first "
     "`stop`, as e23 was built (the page still says that no robot answered)", "e23c"),
    ("MIRLT_CAMERA_PROBE", "usb", "e13", "M C",
     "Looking for cameras at a start: only USB cameras that no other program holds are opened, each in a child process "
     "with a time limit (3 x the longest open on record), and what was found or skipped is said. e13: every /dev/video "
     "node opened in the run's own process, no limit (school 18 Nov: two starts hung 4-5 min)", "e23"),
    ("MIRLT_MOVE_CHECK", "e23", "e22", "M",
     "Did a drive step happen? Judged from where the robot started the step, done when its own pose ended at the "
     "target, and the camera votes only with a picture newer than the step's end (it has its say afterwards, logged). "
     "e22: the plan's length and the newest picture whatever its age (school 18 Nov: 18 of 19 'did NOT move' false)",
     "e23"),
    ("MIRLT_IMPROVE_SKIP_OVER", "1", "0", "M",
     "No search for a shorter plan after a re-plan that already took longer than its budget. 0: always (school 18 Nov: "
     "the planner was busy 87 % of a run)", "e23"),
    ("MIRLT_PLAN_AHEAD", "near", "all", "M",
     "A list of points in a fixed order is planned a few points ahead for each robot (how many: from the planning "
     "time measured in this run against its budget), the rest added as it gets there. all: the whole list in every "
     "re-plan (school 18 Nov: 15 s a re-plan for 20 + 20 points)", "e23"),
    ("MIRLT_POINT_CREDIT", "reached", "always", "M",
     "A stop at a point is recorded as reached only when the robot's best position (camera during the stop, else its "
     "own pose) is within the point's tolerance; else 'not reached' with the distance, and the point stays not "
     "visited. always: every stop the plan made counts (school 18 Nov: stops credited 11-77 cm from their points)",
     "e23"),
    ("MIRLT_OLD_FIX", "project", "send", "M",
     "A camera fix whose picture is older than the robot accepts is carried forward by the robot's own motion since "
     "that picture and sent as a fix for now, with a wider +-. send: sent as it is and refused by the robot as old "
     "(school 18 Nov 17:54: no fix used for the last 40 cm before the edge)", "e23"),
    ("MIRLT_EDGE_SLOW_END", "1", "0", "M",
     "The last part of a leg that ends inside the edge rule's room is driven at the slow speed, so the stop is short "
     "and fixes arrive before the edge. 0: the leg's speed to its end", "e23"),
    ("MIRLT_VEER_AT_POINT", "stand", "recover", "M",
     "A robot standing in its dwell at a point when the camera loses it is not sent on the veer stage's early "
     "turn-back: it has not moved, and where the camera last read it is where it stands. Its stops (its possible error "
     "reaching an edge) still apply, and the veer stage applies again once it drives on unseen. recover: it turns and "
     "drives about 10 cm away mid-dwell (school 18 Nov: 6 of 36 veers; the dwell then ended off the point)", "e23"),
    ("MIRLT_BIND_CAMS", "run", "one", "M C",
     "Naming the robots by their wiggle reads roof tags with every camera the run uses. one: camera 1 only (school 18 "
     "Nov 14:27: a robot standing where only camera 2 sees it could not be named)", "e23"),
    ("MIRLT_WAKE_WAIT", "site", "fixed", "M C",
     "A start that hears NO robot keeps trying for as long as the site's own sleep check-in (school: 60 min check-in, "
     "so up to 60.5 min; home: 10.5 min), and says the real interval and how to wake the robots now. fixed: it waits "
     "the time asked for (3 min) and its message says 'every 10 min' whatever the site", "e23"),
    ("MIRLT_WAKE_PARTIAL", "go", "wait", "M C",
     "A start where some robots answer and others do not: the others are waited for the usual 3 min, then the run "
     "starts with the robots that answered and says loudly which did not (a robot that did not answer is not driven "
     "and not tracked in that run). wait: every robot is waited for the whole sleep check-in (school: Start stood up "
     "to 60.5 min for one sleeping robot), as e23 was built", "e23c"),
    ("MIRLT_OLD_FIX_JUMPS", "remove", "count", "M",
     "A camera picture too old for the robot is carried forward by what the robot DROVE since: the pose corrections "
     "it took since the picture (camera fixes it used, pose resets) are taken out of its pose history first. count: "
     "the newest pose minus the pose at the picture's time, corrections counted as driving, as e23 was built (a "
     "standing robot corrected by 5 cm had the picture moved 5 cm)", "e23c"),
    ("MIRLT_REACH_TOL", "bounded", "e23", "M",
     "How far from its point a stop may be and still count: learned from the stops that were REACHED only, the stop's "
     "own +- counted against it (position within the tolerance AND position + its +- within a second one), and both "
     "kept between the table's measured value and 1.5 times it. e23: learned from every stop, the +- added to the "
     "tolerance, no bound (16:11 run: it grew from 6 to 25.5 cm as stops missed)", "e23c"),
    ("MIRLT_VISIT_UNSEEN", "warn", "quiet", "M",
     "Visit points: a listed point outside every camera the run uses is warned about loudly at the start (Mission "
     "Control and the run log). quiet: a NOTE line only", "e23"),
]

DEFAULT = {s[0]: s[1] for s in SWITCHES}
OLD = {s[0]: s[2] for s in SWITCHES}
# (e19, audit finding 8: a typed "Auto" turned five off-by-default switches ON; "Fixed" was reported as set but was not
# "fixed") each switch takes only its automatic default or its old way (or "auto"), any case, spaces trimmed; anything
# else is refused: the default is used and the refusal said (refused())
CHOICES = {s[0]: (s[1], s[2]) for s in SWITCHES}
_REFUSED = {}


def normalize(env, v):
    """(e19) The switch's value as one of its choices, or None for "auto" / empty / not one of them (refused)."""
    if v is None:
        return None
    w = str(v).strip()
    if w == "" or w.lower() == "auto":
        return None
    for c in CHOICES.get(env, ()):
        if w.lower() == str(c).lower():
            return c
    _REFUSED[env] = w
    return None


def refused():
    """{env: the refused value} seen so far (said at a run's start)."""
    return dict(_REFUSED)
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
            _FILE = {k: normalize(k, v) for k, v in (d.get("switches") or {}).items() if k in DEFAULT}
            _FILE = {k: v for k, v in _FILE.items() if v is not None}
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
    if env in CHOICES:
        v = normalize(env, v)                    # (e19) only its choices; "auto" and anything else: the default
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
    ref = refused()
    if ref:
        return ("Safety group: REFUSED (not one of its choices, its automatic default used): %s; " % ", ".join(
            "%s=%r" % kv for kv in sorted(ref.items()))) + (start_words_ch(ch))
    return start_words_ch(ch)


def start_words_ch(ch):
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
        if k in DEFAULT:
            n = normalize(k, v)                  # (e19) only its own choices pass
            if n is not None:
                out[k] = str(n)
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
